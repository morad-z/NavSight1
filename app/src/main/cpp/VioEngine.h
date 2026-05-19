#pragma once
#include <string>

#include "VioTypes.h"
#include "IMUPreintegrator.h"
#include "Tracker.h"
#include "ViewerServer.h"
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

    // Step 7 (Visual Production Plan, ADR-013) — push the on-device path of
    // the ORB DBoW2 vocabulary into the Tracker's owned LoopClosureDetector.
    // Returns true on success. False if the file is missing or parse fails;
    // the Tracker keeps running with loop closure disabled.
    bool loadLoopClosureVocabulary(const std::string& vocab_path);

    void setInitialHeading(double azimuth_rad);
    // 2026-05-13 heading-startup fix: seed Madgwick's yaw IMMEDIATELY from
    // the compass, without waiting for VIO init. Only touches the IMU
    // integrator; no Tracker/EKF side effects. The full setInitialHeading
    // path stays for EKF rotation seeding. Calling this before
    // setInitialHeading is safe (Madgwick re-initializes on every yaw seed
    // call). Calling it AGAIN after GPS arrives is also safe — it re-runs
    // tryInitMadgwickLocked with the declination-corrected azimuth.
    void seedMadgwickYaw(double yaw_rad);
    void setMagnetometerHeading(float yaw_rad);
    void setDepthMap(const float* depth_data, int width, int height);
    void setUserScaleCorrection(double correction);
    // Step 8c: forward rolling-shutter row-skew into the Tracker. See Tracker.h.
    void setRollingShutterSkew(int64_t row_skew_ns);

    // Step 8b: seed the EKF body→camera rotation from Android camera metadata.
    // R_bc_flat: 9 floats row-major.  Called once after camera open.
    void setExtrinsicsRotation(const float* R_bc_flat);
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

    // Camera-overlay Phase 3: const read access to the EKF state so the
    // JNI layer can expose SLAM feature world positions and the current
    // camera pose without granting Tracker write access. Returns nullptr
    // only if Tracker has not yet been initialised — never under normal
    // running conditions.
    const EKFState* getEKFState() const { return tracker_.getEKF(); }
    // v23.11: expose the Tracker for native-lib SLAM debug snapshot access.
    Tracker* getTracker() { return &tracker_; }
    const Tracker* getTracker() const { return &tracker_; }
    // Access the Tracker's body→camera extrinsics for projection. Read
    // through the EKFState to avoid duplicating R_bc state.
    cv::Matx33d getExtrinsicsRotation() const {
        const EKFState* ekf = tracker_.getEKF();
        return ekf ? ekf->getExtrinsicsRotation() : cv::Matx33d::eye();
    }

private:
    // DISABLED: Mapper pipeline (applyMapperResult was a no-op)
    // void mapperThreadFunc();
    // void applyMapperResult(const MapperResult& mr, VisionOutput& out);

    IMUPreintegrator imu_;
    Tracker tracker_;

    // 2026-05-17 — PC live viewer. HTTP server bound to 0.0.0.0:8765 on
    // VioEngine construction. PC on same WiFi opens http://<phone-ip>:8765/
    // to view a 3D scene of the current LandmarkMap + EKF pose + trajectory.
    // Read-only relative to the VIO state.
    navsight::ViewerServer viewer_server_;

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
