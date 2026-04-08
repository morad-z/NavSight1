#pragma once
#include "VioTypes.h"
#include "IMUPreintegrator.h"
#include "Tracker.h"
#include "Mapper.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

// Top-level VIO orchestrator.
// Tracker runs on the camera thread (fast path).
// Mapper runs on a dedicated background thread (heavy optimization).
// Mapper corrections are applied asynchronously when ready.
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
    void reset();

    IMUPreintegrator& getIMU() { return imu_; }

private:
    void mapperThreadFunc();
    void applyMapperResult(const MapperResult& mr, VisionOutput& out);

    IMUPreintegrator imu_;
    Tracker tracker_;
    Mapper mapper_;

    // Mapper background thread
    std::thread mapper_thread_;
    std::mutex mapper_mutex_;
    std::condition_variable mapper_cv_;
    std::atomic<bool> mapper_stop_{false};
    bool mapper_has_work_{false};
    TrackerFrame mapper_pending_frame_;
    double mapper_pending_scale_{0.0};

    // Depth map (monocular inference result)
    std::mutex         depth_mutex_;
    std::vector<float> latest_depth_map_;
    int                depth_width_{0}, depth_height_{0};

    // Latest mapper result (consumed by next processFrame)
    std::mutex result_mutex_;
    MapperResult latest_mapper_result_{};
    bool has_mapper_result_{false};
};
