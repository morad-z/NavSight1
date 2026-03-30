#pragma once
#include <vector>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include "IMUPreintegrator.h"

struct VisionOutput {
    cv::Mat R;               // 3x3 rotation matrix (CV_64F) - FUSED
    cv::Mat t;               // 3x1 translation vector (CV_64F) - FUSED
    cv::Mat rawR;            // 3x3 rotation matrix (CV_64F) - RAW CAMERA
    cv::Mat rawT;            // 3x1 translation vector (CV_64F) - RAW CAMERA
    double quality;          // 0.0 - 1.0
    int trackedCount;
    int totalCount;
    double estimatedScale;
    bool valid;
    std::vector<float> trackedPoints; // [x0,y0, x1,y1, ...]

    // Diagnostic fields for simulation recording
    double meanFlow;         // Mean optical flow (pixels)
    int    inlierCount;      // Essential Matrix inlier count
    int    stepCount;        // Cumulative step detector count
    double stepFreq;         // Current step frequency (Hz)
    double strideLength;     // Current estimated stride (meters)
    int    poseFlags;        // Bitfield: bit0=isStatic, bit1=isPureRotation, bit2=poseValid, bit3=usedFallback
    double heading;          // Current heading from rotation matrix (radians)
};

class VisionModule {
public:
    VisionModule();
    ~VisionModule();

    VisionOutput processFrame(const uint8_t* yuv_data, int width, int height, int64_t timestamp_ns);
    void addGyroData(int64_t timestamp_ns, float x, float y, float z);
    void addAccelData(int64_t timestamp_ns, float x, float y, float z);
    void setIntrinsics(double fx, double fy, double cx, double cy);
    void reset();

    IMUPreintegrator& getIMU() { return imu_; }

private:
    double calculateTrackingQuality(const std::vector<uchar>& status);
    double estimateScaleFromSteps(double vision_disp, int64_t dt_ns);
    void evictOldPoints();

    mutable std::mutex mutex_;
    mutable std::mutex pose_mutex_; // Granular lock for global_t_ and global_R_
    IMUPreintegrator imu_;

    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    int64_t prev_timestamp_ns_{0};

    // Reusable buffers to minimize re-allocations
    cv::Mat gray_buf_;
    std::vector<cv::Point2f> current_prev_pts_buf_;
    std::vector<cv::Point2f> next_pts_buf_;
    std::vector<uchar> status_buf_;
    std::vector<float> err_buf_;
    std::vector<cv::Point2f> prev_good_buf_;
    std::vector<cv::Point2f> next_good_buf_;
    std::vector<cv::Point2f> new_pts_buf_;

    cv::Mat global_R_;   // global rotation (CV_64F, 3x3)
    cv::Mat global_t_;   // global translation (CV_64F, 3x1)

    // Intrinsics (0 = auto from width)
    double fx_{0.}, fy_{0.}, cx_{0.}, cy_{0.};

    double smooth_scale_{0.20}; // Initial estimate for walking (raised for faster bootstrap)
    int scale_obs_count_{0};     // Number of accepted scale observations (for bootstrap)

    // Parallax gate
    int frames_since_keyframe_{0};
    static constexpr double MIN_PARALLAX_PX = 0.8; // Lowered from 2.0 to accept more frames

    // Accelerometer bias estimation (diagnostic)
    cv::Mat accel_bias_;       // 3x1 CV_64F running bias estimate
    int     accel_bias_count_{0};
    static constexpr int    ACCEL_BIAS_WARMUP = 150;
    static constexpr double ACCEL_BIAS_ALPHA  = 0.005;

    // Forward-backward optical flow check buffers
    std::vector<cv::Point2f> back_pts_buf_;
    std::vector<uchar> back_status_buf_;
    std::vector<float> back_err_buf_;

    // Gyro bias estimation
    cv::Mat gyro_bias_;
    int gyro_bias_count_{0};

    bool initialized_{false};

    static constexpr int MAX_FEATURES    = 500; // Increased from 200
    static constexpr int MIN_FEATURES    = 150;
    static constexpr double QUALITY_LEVEL = 0.01;   // Lowered from 0.03
    static constexpr double MIN_DIST      = 10.0;
    static constexpr double RANSAC_CONF   = 0.9999;
    static constexpr double RANSAC_THRESH = 0.5;
    static constexpr double ALPHA_FUSION  = 0.98;
    static constexpr int64_t MAX_DT_NS   = 5'000'000'000LL; // 5 seconds

    // Forward-backward check
    static constexpr double FB_CHECK_THRESH  = 9.0;  // Relaxed for more point acceptance (squared pixel distance)
    static constexpr double MIN_FLOW_PX      = 0.4;
    static constexpr int    MIN_INLIERS      = 8;    // Lowered from 10
    static constexpr double MIN_INLIER_RATIO = 0.25; // Lowered from 0.35

    // Drift-Kill Thresholds
    static constexpr double GYRO_ROT_ONLY_THRESH = 2.0; // Raised from 0.5 rad/s - allow fast arm swings
    static constexpr double ZUPT_GYRO_THRESH = 0.04;    // Slightly tighter static detection
};
