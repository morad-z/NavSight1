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
#include "TrackKLT.h"
#include "UpdaterZeroVelocity.h"
#include "UpdaterMSCKF.h"

#include "InertialInitializer.h"

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

    // Handle IMU data for initialization
    void addImuData(int64_t ts, float ax, float ay, float az, float gx, float gy, float gz);

    // DEAD CODE: blendScale/applyLoopCorrection — VioEngine.applyMapperResult is a no-op
    // void blendScale(double target_scale, double alpha);
    // void applyLoopCorrection(double target_x, double target_y, double target_z,
    //                          double target_heading, double blend,
    //                          VisionOutput& out);

    // Thread-safe read-only accessors
    double getSmoothScale() const;
    double getHeading() const;
    bool isInitialized() const { return initialized_; }
    const EKFState* getEKF() const { return &ekf_; }

private:
    double estimateScaleFromSteps(double vision_disp, int64_t dt_ns,
                                  IMUPreintegrator& imu);

    mutable std::mutex mutex_;
    mutable std::mutex pose_mutex_;

    InertialInitializer initializer_;
    bool initialized_{false};
    
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    int64_t prev_timestamp_ns_{0};

    // Reusable buffers (pre-reserved to avoid per-frame allocation)
    cv::Mat gray_buf_;
    std::vector<cv::Point2f> current_prev_pts_buf_;
    std::vector<cv::Point2f> next_pts_buf_;
    std::vector<cv::Point2f> prev_good_buf_;
    std::vector<cv::Point2f> next_good_buf_;
    std::vector<cv::Point2f> new_pts_buf_;

    cv::Mat global_R_;   // 3x3 CV_64F
    cv::Mat global_t_;   // 3x1 CV_64F

    double fx_{0.}, fy_{0.}, cx_{0.}, cy_{0.};
    double smooth_scale_{0.20};
    int scale_obs_count_{0};
    double user_scale_correction_{1.0};
    cv::Mat accel_bias_;
    int accel_bias_count_{0};
    // gyro_bias_ removed — unified: use IMUPreintegrator::getGyroBias() only

    std::vector<cv::Point3f> points_3d_current_;
    std::vector<int> feature_ages_;  // Phase 2: track age per feature (frames survived)
    int frame_counter_{0};

    // Phase 5: Magnetometer heading used ONCE at startup only
    bool heading_initialized_{false};

    // Scalar heading tracker (gravity-projected yaw rate from gyro)
    // Fixes coordinate system bug: atan2(R[1,0], R[0,0]) only captures Z-rotation,
    // but phone yaw is around gravity (Y-axis), so turns were under-reported.
    double scalar_heading_{0.0};

    // Phase 7: FEJ heading — locked at first pose_valid frame
    double heading_fej_{0.0};
    bool heading_fej_set_{false};

    // Phase 6: Time-offset cross-correlation warmup
    struct TdSample { double flow_rate; double gyro_rate; int64_t ts_ns; };
    std::vector<TdSample> td_warmup_buf_;
    bool td_warmup_done_{false};
    static constexpr int TD_WARMUP_FRAMES = 60;  // ~2 seconds at 30fps

    // Step speed interpolation: maintain last known speed for short gaps
    double last_step_speed_{0.0};
    int64_t last_step_speed_ns_{0};

    cv::Ptr<cv::CLAHE> clahe_;

    // Accuracy modules
    EKFState ekf_;
    FeatureManager feature_mgr_;
    LensCorrector lens_;
    TrackKLT klt_;
    UpdaterZeroVelocity zupt_detector_;
    // DEAD CODE: MSCKF updater — processLostFeatures never called (MSCKF update disabled)
    UpdaterMSCKF msckf_updater_; // kept as member to avoid header changes cascading

    // MSCKF feature ID tracking (parallel to prev_pts_)
    std::vector<int> feature_ids_;

    // Keyframe tracking state
    int frames_since_keyframe_{0};

    // Constants
    static constexpr int    MAX_FEATURES       = 200;  // OpenVINS default for monocular
    static constexpr int    MIN_FEATURES       = 80;
    static constexpr double QUALITY_LEVEL      = 0.05;
    static constexpr double MIN_DIST           = 10.0;
    static constexpr double RANSAC_CONF        = 0.9999;
    static constexpr double RANSAC_THRESH      = 1.5;
    static constexpr double MIN_PARALLAX_PX    = 0.8;
    static constexpr double FB_CHECK_THRESH    = 4.0;  // 2px threshold (squared); was 9.0
    static constexpr double MIN_FLOW_PX        = 0.4;
    static constexpr double MAX_FLOW_PX        = 150.0;
    static constexpr int    MIN_INLIERS        = 8;
    static constexpr double MIN_INLIER_RATIO   = 0.25;
    static constexpr double GYRO_ROT_ONLY_THRESH = 2.0;
    static constexpr double ZUPT_GYRO_THRESH   = 0.04;
    static constexpr int    ACCEL_BIAS_WARMUP  = 150;
    static constexpr double ACCEL_BIAS_ALPHA   = 0.005;
};
