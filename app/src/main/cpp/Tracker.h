#pragma once
#include <vector>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "VioTypes.h"
#include "IMUPreintegrator.h"
#include "EKFState.h"
#include "FeatureManager.h"
#include "LensCorrector.h"

// Intermediate frame data exported from Tracker to Mapper each frame.
struct TrackerFrame {
    cv::Mat gray;                        // Current grayscale (clone for Mapper)
    std::vector<cv::Point2f> prev_good;  // FB-checked prev points
    std::vector<cv::Point2f> next_good;  // FB-checked next points
    std::vector<cv::Point3f> points_3d;  // Triangulated 3D points
    cv::Point3f global_position;         // Global position after pose update
    bool pose_valid;
    double quality;
    int tracked;
    int frame_counter;
    int64_t timestamp_ns;
    int64_t prev_timestamp_ns;
    double estimated_scale;
    double heading;
    double fx, fy, cx, cy;              // Intrinsics used this frame
};

class Tracker {
public:
    Tracker();

    // Core tracking: optical flow, essential matrix, rotation fusion, pose update.
    // Exports intermediate data in frame_out for Mapper.
    VisionOutput processFrame(const uint8_t* yuv_data, int width, int height,
                              int64_t timestamp_ns, IMUPreintegrator& imu,
                              TrackerFrame& frame_out);

    void setIntrinsics(double fx, double fy, double cx, double cy);
    void setInitialHeading(double azimuth_rad);
    void setUserScaleCorrection(double correction);
    void reset();

    // Called by VioEngine after Mapper returns scale corrections.
    void blendScale(double target_scale, double alpha);

    // Called by VioEngine after Mapper detects a loop closure.
    void applyLoopCorrection(double target_x, double target_y, double target_z,
                             double target_heading, double blend,
                             VisionOutput& out);

    // Thread-safe read-only accessors
    double getSmoothScale() const;
    double getHeading() const;

private:
    double estimateScaleFromSteps(double vision_disp, int64_t dt_ns,
                                  IMUPreintegrator& imu);

    mutable std::mutex mutex_;
    mutable std::mutex pose_mutex_;

    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    int64_t prev_timestamp_ns_{0};

    // Reusable buffers (pre-reserved to avoid per-frame allocation)
    cv::Mat gray_buf_;
    std::vector<cv::Point2f> current_prev_pts_buf_;
    std::vector<cv::Point2f> next_pts_buf_;
    std::vector<uchar> status_buf_;
    std::vector<float> err_buf_;
    std::vector<cv::Point2f> prev_good_buf_;
    std::vector<cv::Point2f> next_good_buf_;
    std::vector<cv::Point2f> new_pts_buf_;
    std::vector<cv::Point2f> back_pts_buf_;
    std::vector<uchar> back_status_buf_;
    std::vector<float> back_err_buf_;

    cv::Mat global_R_;   // 3x3 CV_64F
    cv::Mat global_t_;   // 3x1 CV_64F

    double fx_{0.}, fy_{0.}, cx_{0.}, cy_{0.};
    double smooth_scale_{0.20};
    int scale_obs_count_{0};
    double user_scale_correction_{1.0};
    cv::Mat accel_bias_;
    int accel_bias_count_{0};
    cv::Mat gyro_bias_;
    int gyro_bias_count_{0};

    std::vector<cv::Point3f> points_3d_current_;
    bool initialized_{false};
    int frame_counter_{0};

    // Step speed interpolation: maintain last known speed for short gaps
    double last_step_speed_{0.0};
    int64_t last_step_speed_ns_{0};

    cv::Ptr<cv::CLAHE> clahe_;

    // Accuracy modules
    EKFState ekf_;
    FeatureManager feature_mgr_;
    LensCorrector lens_;

    // Keyframe tracking state
    int frames_since_keyframe_{0};

    // Constants
    static constexpr int    MAX_FEATURES       = 400;
    static constexpr int    MIN_FEATURES       = 120;
    static constexpr double QUALITY_LEVEL      = 0.05;
    static constexpr double MIN_DIST           = 10.0;
    static constexpr double RANSAC_CONF        = 0.9999;
    static constexpr double RANSAC_THRESH      = 0.5;
    static constexpr double MIN_PARALLAX_PX    = 0.8;
    static constexpr double FB_CHECK_THRESH    = 16.0; // 4px threshold (squared)
    static constexpr double MIN_FLOW_PX        = 0.4;
    static constexpr double MAX_FLOW_PX        = 150.0;
    static constexpr int    MIN_INLIERS        = 8;
    static constexpr double MIN_INLIER_RATIO   = 0.25;
    static constexpr double GYRO_ROT_ONLY_THRESH = 2.0;
    static constexpr double ZUPT_GYRO_THRESH   = 0.08;
    static constexpr int    ACCEL_BIAS_WARMUP  = 150;
    static constexpr double ACCEL_BIAS_ALPHA   = 0.005;
};
