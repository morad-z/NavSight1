#include "VioEngine.h"
#include <android/log.h>
#include <cmath>

#define TAG "NavSight-VioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

VioEngine::VioEngine() {
    // DISABLED: Mapper background thread — output was discarded (applyMapperResult was a no-op)
    // mapper_thread_ = std::thread(&VioEngine::mapperThreadFunc, this);
    LOGI("VioEngine created (Mapper pipeline disabled — output was discarded)");
}

VioEngine::~VioEngine() {
    // DISABLED: Mapper thread join
    // {
    //     std::lock_guard<std::mutex> lock(mapper_mutex_);
    //     mapper_stop_.store(true);
    //     mapper_cv_.notify_one();
    // }
    // if (mapper_thread_.joinable()) {
    //     mapper_thread_.join();
    // }
    LOGI("VioEngine destroyed");
}

// DISABLED: Mapper background thread — Mapper, LoopClosureDetector, PoseGraph all
// ran but applyMapperResult was a no-op. Corrections caused teleportation spikes.
/*
void VioEngine::mapperThreadFunc() {
    LOGI("Mapper thread started");
    while (!mapper_stop_.load()) {
        TrackerFrame frame;
        double scale;
        {
            std::unique_lock<std::mutex> lock(mapper_mutex_);
            mapper_cv_.wait(lock, [this] {
                return mapper_has_work_ || mapper_stop_.load();
            });
            if (mapper_stop_.load()) break;
            frame = std::move(mapper_pending_frame_);
            scale = mapper_pending_scale_;
            mapper_has_work_ = false;
        }
        MapperResult mr = mapper_.process(frame, scale, tracker_.getEKF());
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            latest_mapper_result_ = mr;
            has_mapper_result_ = true;
        }
    }
    LOGI("Mapper thread stopped");
}
*/

// DISABLED: applyMapperResult — was already a no-op
/*
void VioEngine::applyMapperResult(const MapperResult& mr, VisionOutput& out) {
    (void)mr;
    (void)out;
}
*/

// ── processFrame ────────────────────────────────────────────────────────────
// Tracker only (fast path, ~5-10ms). Mapper pipeline disabled.

VisionOutput VioEngine::processFrame(const uint8_t* yuv_data, int width, int height,
                                      int64_t timestamp_ns) {
    TrackerFrame frame;
    VisionOutput out = tracker_.processFrame(yuv_data, width, height,
                                              timestamp_ns, imu_, frame);

    // DISABLED: Mapper result pickup — applyMapperResult was a no-op
    // {
    //     std::lock_guard<std::mutex> lock(result_mutex_);
    //     if (has_mapper_result_) {
    //         applyMapperResult(latest_mapper_result_, out);
    //         has_mapper_result_ = false;
    //     }
    // }

    // DISABLED: Mapper thread submission — output was discarded
    // {
    //     std::lock_guard<std::mutex> lock(mapper_mutex_);
    //     if (!mapper_has_work_) {
    //         mapper_pending_frame_ = std::move(frame);
    //         mapper_pending_scale_ = tracker_.getSmoothScale();
    //         mapper_has_work_ = true;
    //         mapper_cv_.notify_one();
    //     }
    // }

    return out;
}

// ── Sensor data forwarding ──────────────────────────────────────────────────

void VioEngine::addGyroData(int64_t timestamp_ns, float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("addGyroData: NaN/Inf value, dropping");
        return;
    }
    imu_.addGyroReading(timestamp_ns, x, y, z);
    
    // Also pass to tracker for initialization
    tracker_.addImuData(timestamp_ns, imu_.lastAccelX(), imu_.lastAccelY(), imu_.lastAccelZ(), x, y, z);
}

void VioEngine::addAccelData(int64_t timestamp_ns, float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("addAccelData: NaN/Inf value, dropping");
        return;
    }
    imu_.addAccelReading(timestamp_ns, x, y, z);

    // Also pass to tracker for initialization
    tracker_.addImuData(timestamp_ns, x, y, z, imu_.lastGyroX(), imu_.lastGyroY(), imu_.lastGyroZ());
}

void VioEngine::setIntrinsics(double fx, double fy, double cx, double cy) {
    tracker_.setIntrinsics(fx, fy, cx, cy);
}

void VioEngine::setInitialHeading(double azimuth_rad) {
    tracker_.setInitialHeading(azimuth_rad);
}

void VioEngine::setMagnetometerHeading(float yaw_rad) {
    imu_.setMagnetometerHeading(yaw_rad);
}

void VioEngine::setDepthMap(const float* depth_data, int width, int height) {
    // DISABLED: Depth map storage + Mapper forwarding — Mapper pipeline is disabled,
    // depth results were never applied (applyMapperResult was a no-op).
    // Kotlin DepthEstimator also disabled to save CPU/GPU/battery.
    (void)depth_data;
    (void)width;
    (void)height;
}

void VioEngine::setUserScaleCorrection(double correction) {
    tracker_.setUserScaleCorrection(correction);
}

void VioEngine::setUserHeight(float height_m) {
    imu_.setUserHeight(height_m);
    // DISABLED: mapper_.setCameraHeight — Mapper pipeline disabled
    // mapper_.setCameraHeight(static_cast<double>(height_m));
}

void VioEngine::updateDepthScale(double scale, double alpha) {
    // Apply depth-based scale from MiDaS estimation
    // Blends the depth scale into the tracker's smooth scale
    tracker_.blendScale(scale, alpha);
    LOGI("updateDepthScale: scale=%.3f alpha=%.3f", scale, alpha);
}

void VioEngine::reset() {
    // DISABLED: Mapper thread/result reset — Mapper pipeline disabled
    // {
    //     std::lock_guard<std::mutex> lock(mapper_mutex_);
    //     mapper_has_work_ = false;
    // }
    // {
    //     std::lock_guard<std::mutex> lock(result_mutex_);
    //     has_mapper_result_ = false;
    //     latest_mapper_result_ = {};
    // }
    tracker_.reset();
    // mapper_.reset();
    imu_.reset();
    LOGI("VioEngine reset");
}
