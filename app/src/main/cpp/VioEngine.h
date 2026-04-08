#pragma once
#include "VioTypes.h"
#include "IMUPreintegrator.h"
#include "Tracker.h"
// DISABLED: Mapper pipeline — applyMapperResult was a no-op (corrections caused teleportation).
// Mapper, LoopClosureDetector, PoseGraph all run but output is discarded. Disabled to save CPU/battery.
// #include "Mapper.h"
#include <mutex>

// Top-level VIO orchestrator.
// Tracker runs on the camera thread (fast path).
// Mapper pipeline DISABLED — was running but output discarded.
class VioEngine {
public:
    VioEngine();
    ~VioEngine();

    VisionOutput processFrame(const uint8_t* yuv_data, int width, int height,
                              int64_t timestamp_ns);
    void addGyroData(int64_t timestamp_ns, float x, float y, float z);
    void addAccelData(int64_t timestamp_ns, float x, float y, float z);
    void setIntrinsics(double fx, double fy, double cx, double cy);
    void setInitialHeading(double azimuth_rad);
    void setMagnetometerHeading(float yaw_rad);
    void setDepthMap(const float* depth_data, int width, int height);
    void setUserScaleCorrection(double correction);
    void setUserHeight(float height_m);
    void updateDepthScale(double scale, double alpha);
    void reset();

    IMUPreintegrator& getIMU() { return imu_; }

private:
    // DISABLED: Mapper pipeline (applyMapperResult was a no-op)
    // void mapperThreadFunc();
    // void applyMapperResult(const MapperResult& mr, VisionOutput& out);

    IMUPreintegrator imu_;
    Tracker tracker_;

    // DISABLED: Mapper + background thread — output was discarded via no-op applyMapperResult
    // Mapper mapper_;
    // std::thread mapper_thread_;
    // std::mutex mapper_mutex_;
    // std::condition_variable mapper_cv_;
    // std::atomic<bool> mapper_stop_{false};
    // bool mapper_has_work_{false};
    // TrackerFrame mapper_pending_frame_;
    // double mapper_pending_scale_{0.0};

    // DISABLED: Depth map storage — only forwarded to Mapper which is disabled
    // std::mutex         depth_mutex_;
    // std::vector<float> latest_depth_map_;
    // int                depth_width_{0}, depth_height_{0};

    // DISABLED: Mapper result pickup
    // std::mutex result_mutex_;
    // MapperResult latest_mapper_result_{};
    // bool has_mapper_result_{false};
};
