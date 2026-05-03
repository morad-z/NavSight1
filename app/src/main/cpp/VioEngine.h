#pragma once
#include <string>

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

    // Step 1 (Visual Production Plan) — load fx/fy/cx/cy + 8-coef rational
    // distortion from the in-app camera calibration JSON. On success pushes
    // intrinsics into the Tracker and distortion into the LensCorrector.
    // Returns false if the file is missing, malformed, or fails any
    // validation gate (RMS > 1.0 px, fx/fy <= 0, cx/cy out of frame, etc.) —
    // the engine then keeps its existing zero-distortion passthrough.
    bool loadCalibration(const std::string& path);

    void setInitialHeading(double azimuth_rad);
    void setMagnetometerHeading(float yaw_rad);
    void setDepthMap(const float* depth_data, int width, int height);
    void setUserScaleCorrection(double correction);
    void setUserHeight(float height_m);
    void reset();

    // Step 5 — Calibration & Initialization
    // Status: 0=WAIT_STATIONARY, 1=WAIT_MOTION, 2=READY, 3=TIMEOUT_NEEDS_USER
    int  getInitStatus() const;
    void clearInitTimeout();
    // Inject stored calibration to skip the stationary gate.
    // R_GtoI is row-major 3x3; biases are body-frame xyz.
    void loadStoredCalibration(const float R_GtoI[9],
                               const float gyro_bias[3],
                               const float accel_bias[3]);
    // Pull current calibration for SharedPreferences write-back.
    // Outputs valid only after the gate has passed (status >= WAIT_MOTION).
    bool getCalibration(float R_GtoI[9],
                        float gyro_bias[3],
                        float accel_bias[3]) const;

    // Step 6 — Horizontal-plane position covariance (m²) extracted from EKF.
    // out = [σ_xx, σ_xz, σ_zz]. Returns false (zeros) if EKF not yet ready.
    bool getPositionCovarianceXZ(double out[3]) const;

    // Step 7 — Replay harness pose accessor.
    // Reads EKFState's body position p_G_ and the gravity-aligned yaw used by the
    // navigation pipeline (CW-positive, North=0). Returns false if the EKF has
    // not yet reached full initialization (output left untouched).
    bool getPose(double& x, double& y, double& z, double& yaw_rad) const;

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
