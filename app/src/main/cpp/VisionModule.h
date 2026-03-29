#pragma once
#include <vector>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include "IMUPreintegrator.h"

struct VisionOutput {
    cv::Mat R;               // 3x3 rotation matrix (CV_64F)
    cv::Mat t;               // 3x1 translation vector (CV_64F)
    double quality;          // 0.0 - 1.0
    int trackedCount;
    int totalCount;
    double estimatedScale;
    bool valid;
    std::vector<float> trackedPoints; // [x0,y0, x1,y1, ...]
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
    double estimateScaleFromAccel(double vision_disp, int64_t dt_ns);
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

    double smooth_scale_{1.0}; // Thread-safe class member for scale smoothing

    bool initialized_{false};

    static constexpr int MAX_FEATURES    = 200;
    static constexpr int MIN_FEATURES    = 100;
    static constexpr double QUALITY_LEVEL = 0.01;
    static constexpr double MIN_DIST      = 10.0;
    static constexpr double RANSAC_CONF   = 0.999;
    static constexpr double RANSAC_THRESH = 1.0;
    static constexpr double ALPHA_FUSION  = 0.98;  // gyro weight
    static constexpr int64_t MAX_DT_NS   = 5'000'000'000LL; // 5 seconds

    // Drift-Kill Thresholds
    static constexpr double GYRO_ROT_ONLY_THRESH = 0.5; // rad/s - switch to rotation-only
    static constexpr double ZUPT_ACCEL_THRESH = 0.2;    // m/s^2 deviation from gravity (raised from 0.1)
    static constexpr double ZUPT_GYRO_THRESH = 0.1;     // rad/s - nearly static (raised from 0.05)
};
