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
    IMUPreintegrator imu_;

    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    int64_t prev_timestamp_ns_{0};

    cv::Mat global_R_;   // global rotation (CV_64F, 3x3)
    cv::Mat global_t_;   // global translation (CV_64F, 3x1)

    // Intrinsics (0 = auto from width)
    double fx_{0.}, fy_{0.}, cx_{0.}, cy_{0.};

    bool initialized_{false};

    static constexpr int MAX_FEATURES    = 200;
    static constexpr int MIN_FEATURES    = 100;
    static constexpr double QUALITY_LEVEL = 0.01;
    static constexpr double MIN_DIST      = 10.0;
    static constexpr double RANSAC_CONF   = 0.999;
    static constexpr double RANSAC_THRESH = 1.0;
    static constexpr double ALPHA_FUSION  = 0.98;  // gyro weight
    static constexpr int64_t MAX_DT_NS   = 5'000'000'000LL; // 5 seconds
};
