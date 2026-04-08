#include "VioEngine.h"
#include <android/log.h>
#include <cmath>

#define TAG "NavSight-VioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

VioEngine::VioEngine() {
    // Start Mapper background thread
    mapper_thread_ = std::thread(&VioEngine::mapperThreadFunc, this);
    LOGI("VioEngine created (Mapper on background thread)");
}

VioEngine::~VioEngine() {
    // Signal mapper thread to stop and join
    {
        std::lock_guard<std::mutex> lock(mapper_mutex_);
        mapper_stop_.store(true);
        mapper_cv_.notify_one();
    }
    if (mapper_thread_.joinable()) {
        mapper_thread_.join();
    }
    LOGI("VioEngine destroyed");
}

// ── Mapper background thread ────────────────────────────────────────────────
// Waits for work, processes one frame at a time, stores result for pickup.

void VioEngine::mapperThreadFunc() {
    LOGI("Mapper thread started");
    while (!mapper_stop_.load()) {
        TrackerFrame frame;
        double scale;

        // Wait for work
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

        // Run mapper (heavy: ground plane, BA, loop closure)
        MapperResult mr = mapper_.process(frame, scale, tracker_.getEKF());

        // Store result for next processFrame to pick up
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            latest_mapper_result_ = mr;
            has_mapper_result_ = true;
        }
    }
    LOGI("Mapper thread stopped");
}

// ── processFrame ────────────────────────────────────────────────────────────
// 1. Apply any pending Mapper corrections from previous frame (non-blocking).
// 2. Run Tracker (fast path, ~5-10ms).
// 3. Submit frame to Mapper thread (non-blocking).

VisionOutput VioEngine::processFrame(const uint8_t* yuv_data, int width, int height,
                                      int64_t timestamp_ns) {
    // Fast path: core tracking
    TrackerFrame frame;
    VisionOutput out = tracker_.processFrame(yuv_data, width, height,
                                              timestamp_ns, imu_, frame);

    // Apply latest Mapper result if available (from previous frame's background work)
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (has_mapper_result_) {
            applyMapperResult(latest_mapper_result_, out);
            has_mapper_result_ = false;
        }
    }

    // Submit to Mapper thread (non-blocking: if Mapper is still busy, skip this frame)
    {
        std::lock_guard<std::mutex> lock(mapper_mutex_);
        if (!mapper_has_work_) {
            mapper_pending_frame_ = std::move(frame);
            mapper_pending_scale_ = tracker_.getSmoothScale();
            mapper_has_work_ = true;
            mapper_cv_.notify_one();
        }
    }

    return out;
}

// ── Apply Mapper corrections to Tracker state ───────────────────────────────

void VioEngine::applyMapperResult(const MapperResult& mr, VisionOutput& out) {
    // SIMPLIFIED PIPELINE:
    // Scale comes from step detector only (via Tracker's EKF).
    // Mapper corrections (BA, ground plane, depth, loop closure) are DISABLED.
    // They were all fighting each other and causing teleportation spikes.
    //
    // MiDaS depth and ground plane are still computed by the Mapper thread
    // for future use, but their corrections are not applied.
    //
    // TODO: Re-enable depth scale once the core pipeline is stable,
    // using it as a consistency check rather than a competing scale source.
    (void)mr;
    (void)out;
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
    if (!depth_data || width <= 0 || height <= 0) return;

    std::lock_guard<std::mutex> lock(depth_mutex_);
    size_t size = static_cast<size_t>(width * height);
    if (latest_depth_map_.size() != size) {
        latest_depth_map_.resize(size);
    }
    std::copy(depth_data, depth_data + size, latest_depth_map_.begin());
    depth_width_ = width;
    depth_height_ = height;

    // Also update mapper state
    mapper_.setDepthMap(depth_data, width, height);
}

void VioEngine::setUserScaleCorrection(double correction) {
    tracker_.setUserScaleCorrection(correction);
}

void VioEngine::setUserHeight(float height_m) {
    imu_.setUserHeight(height_m);
    mapper_.setCameraHeight(static_cast<double>(height_m));
}

void VioEngine::reset() {
    // Stop mapper thread work before resetting
    {
        std::lock_guard<std::mutex> lock(mapper_mutex_);
        mapper_has_work_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        has_mapper_result_ = false;
        latest_mapper_result_ = {};
    }
    tracker_.reset();
    mapper_.reset();
    imu_.reset();
    LOGI("VioEngine reset");
}
