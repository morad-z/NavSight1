#include "VioEngine.h"
#include <cmath>

#include "CameraCalibration.h"

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-VioEngine"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#endif

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

bool VioEngine::loadCalibration(const std::string& path) {
    CameraIntrinsics ci;
    if (!loadCalibrationFromJson(path, ci)) {
        LOGI("VIO: no calibration found at '%s', running with zero-distortion "
             "passthrough — accuracy reduced.", path.c_str());
        return false;
    }
    tracker_.setIntrinsics(ci.fx, ci.fy, ci.cx, ci.cy);
    tracker_.setDistortion(ci.k1, ci.k2, ci.k3, ci.k4, ci.k5, ci.k6,
                           ci.p1, ci.p2);
    LOGI("VIO: loaded calibration from %s, RMS=%.3fpx", path.c_str(), ci.rms_px);
    return true;
}

bool VioEngine::loadLoopClosureVocabulary(const std::string& vocab_path) {
    // Plan Step 7 (ADR-013): forward to the Tracker. The Tracker owns the
    // LoopClosureDetector + the 1 Hz worker thread; on first successful
    // load it starts the worker.
    return tracker_.loadLoopClosureVocabulary(vocab_path);
}

void VioEngine::setInitialHeading(double azimuth_rad) {
    tracker_.setInitialHeading(azimuth_rad);
    // Also seed Madgwick's internal quaternion. Without this, Madgwick
    // initializes at yaw=0 (North) regardless of the device's actual
    // compass heading, and the moment Tracker starts refreshing
    // scalar_heading_ from imu.getHeading() (Tracker.cpp:1422), the
    // displayed heading flips from azimuth to 0 — a 180° (or any-magnitude)
    // jump depending on how far from North the device was facing. Bug
    // surfaced 2026-05-09 on sim 1778258249750 ("at first heading is
    // correct then it instantly rotates 180 degrees").
    imu_.setInitialMadgwickYaw(static_cast<float>(azimuth_rad));
}

void VioEngine::setMagnetometerHeading(float yaw_rad) {
    imu_.setMagnetometerHeading(yaw_rad);
}

void VioEngine::setDepthMap(const float* depth_data, int width, int height) {
    // Forward to Tracker for depth-based scale constraint (bypasses Mapper)
    tracker_.setDepthMap(depth_data, width, height);
}

void VioEngine::setUserScaleCorrection(double correction) {
    tracker_.setUserScaleCorrection(correction);
}

// Step 8c: forward rolling-shutter skew into the Tracker.
void VioEngine::setRollingShutterSkew(int64_t row_skew_ns) {
    tracker_.setRollingShutterSkew(row_skew_ns);
}

// Step 8b: seed the EKF body→camera rotation from Android camera metadata.
void VioEngine::setExtrinsicsRotation(const float* R_bc_flat) {
    tracker_.setExtrinsicsRotation(R_bc_flat);
}

void VioEngine::setUserHeight(float height_m) {
    imu_.setUserHeight(height_m);
    // DISABLED: mapper_.setCameraHeight — Mapper pipeline disabled
    // mapper_.setCameraHeight(static_cast<double>(height_m));
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

// ── Step 5: Calibration & Initialization ─────────────────────────────────────

int VioEngine::getInitStatus() const {
    return static_cast<int>(tracker_.getInitStatus());
}

void VioEngine::clearInitTimeout() {
    tracker_.clearInitTimeout();
}

void VioEngine::loadStoredCalibration(const float R_GtoI[9],
                                       const float gyro_bias[3],
                                       const float accel_bias[3]) {
    cv::Mat R(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R.at<double>(r, c) = static_cast<double>(R_GtoI[r * 3 + c]);
        }
    }
    cv::Point3f gb(gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    cv::Point3f ab(accel_bias[0], accel_bias[1], accel_bias[2]);
    tracker_.loadStoredCalibration(R, gb, ab);
    imu_.setGyroBias(gb.x, gb.y, gb.z);
    LOGI("VioEngine: stored calibration loaded (gyro_bias=(%.4f,%.4f,%.4f))",
         gb.x, gb.y, gb.z);
}

bool VioEngine::getCalibration(float R_GtoI[9],
                                float gyro_bias[3],
                                float accel_bias[3]) const {
    if (tracker_.getInitStatus() == InertialInitializer::Status::WAIT_STATIONARY
        || tracker_.getInitStatus() == InertialInitializer::Status::TIMEOUT_NEEDS_USER) {
        return false;
    }
    cv::Mat R = tracker_.getInitialRotation();
    cv::Point3f gb = tracker_.getCalibratedGyroBias();
    cv::Point3f ab = tracker_.getCalibratedAccelBias();
    if (R.empty() || R.rows != 3 || R.cols != 3) return false;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R_GtoI[r * 3 + c] = static_cast<float>(R.at<double>(r, c));
        }
    }
    gyro_bias[0] = gb.x; gyro_bias[1] = gb.y; gyro_bias[2] = gb.z;
    accel_bias[0] = ab.x; accel_bias[1] = ab.y; accel_bias[2] = ab.z;
    return true;
}

bool VioEngine::getPositionCovarianceXZ(double out[3]) const {
    return tracker_.getPositionCovarianceXZ(out);
}

bool VioEngine::getPose(double& x, double& y, double& z, double& yaw_rad) const {
    const EKFState* ekf = tracker_.getEKF();
    if (ekf == nullptr || !ekf->isFullInitialized()) return false;
    cv::Mat p = ekf->getPosition();
    if (p.empty() || p.rows < 3 || p.type() != CV_64F) return false;
    // Coordinate-frame boundary swap (Z-up internal -> Y-up exposed).
    // EKF stores p_G_ in Z-up ENU world (X=East, Y=North, Z=Up). Callers
    // (replay_harness CSV, JNI VioData) historically expect Y-up
    // (X=East, Y=Up, Z=North). Swap indices 1 and 2 here. See
    // scripts/test_z_up_conventions.py.
    x = p.at<double>(0, 0);  // East — same in both
    y = p.at<double>(2, 0);  // Up — Z-up index 2 -> exposed as Y
    z = p.at<double>(1, 0);  // North — Z-up index 1 -> exposed as Z
    // Use the cached scalar_heading_ that the runtime pipeline publishes —
    // it is the gravity-aligned EKF yaw refreshed each frame by processFrame.
    yaw_rad = tracker_.getHeading();
    return true;
}
