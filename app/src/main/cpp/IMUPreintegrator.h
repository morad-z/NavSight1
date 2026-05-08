#pragma once
#include <vector>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <opencv2/core.hpp>

struct GyroSample {
    int64_t timestamp_ns;
    float x, y, z;
};

struct AccelSample {
    int64_t timestamp_ns;
    float x, y, z;
};

struct PreintegratedMeasurement {
    cv::Mat deltaR; // 3x3 rotation
    cv::Mat deltaV; // 3x1 velocity
    cv::Mat deltaP; // 3x1 position
    cv::Mat cov;    // 9x9 covariance (order: rotation, velocity, position)
    
    // Jacobians w.r.t. biases
    cv::Mat J_R_bg; // 3x3
    cv::Mat J_V_bg; // 3x3
    cv::Mat J_V_ba; // 3x3
    cv::Mat J_P_bg; // 3x3
    cv::Mat J_P_ba; // 3x3

    double dt;
    int sample_count;

    PreintegratedMeasurement();
};

class IMUPreintegrator {
public:
    IMUPreintegrator();

    void addGyroReading(int64_t timestamp_ns, float x, float y, float z);
    void addAccelReading(int64_t timestamp_ns, float x, float y, float z);

    // DEAD CODE: superseded by integrate()
    // cv::Mat integrateGyro(int64_t start_ns, int64_t end_ns);

    // Full preintegration (returns deltaR, deltaV, deltaP for the interval)
    PreintegratedMeasurement integrate(int64_t start_ns, int64_t end_ns);

    // DEAD CODE: interpolateGyro/Accel — never called, never implemented in .cpp
    // GyroSample interpolateGyro(int64_t ts_ns) const;
    // AccelSample interpolateAccel(int64_t ts_ns) const;

    // DEAD CODE: correctMeasurement — never called
    // void correctMeasurement(PreintegratedMeasurement& meas,
    //                         const cv::Point3f& delta_bg,
    //                         const cv::Point3f& delta_ba);

    // Set IMU noise parameters
    void setNoiseParameters(float accel_noise, float gyro_noise, 
                            float accel_rw, float gyro_rw);

    // DEAD CODE: initializeFromGravity, setGravity — never called
    // void initializeFromGravity(const std::vector<cv::Point3f>& accel_samples);
    // void setGravity(float gx, float gy, float gz);

    // DEAD CODE: getGravityVector — never called (only getFilteredGravity is used)
    // cv::Point3f getGravityVector() const;
    // Low-pass filtered accelerometer — tracks current gravity direction
    // even as the phone tilts during use (unlike getGravityVector which is
    // frozen from initialization).
    cv::Point3f getFilteredGravity() const;
    // DEAD CODE: getRoll/getPitch — never called
    // float getRoll() const;
    // float getPitch() const;
    bool isInitialized() const;

    void reset();
    std::vector<AccelSample> getAccelBuffer() const;
    std::vector<GyroSample> getGyroBuffer() const;

    // Last known sensor values (for VioData passthrough)
    float lastAccelX() const { return last_ax; }
    float lastAccelY() const { return last_ay; }
    float lastAccelZ() const { return last_az; }
    float lastGyroX()  const { return last_gx; }
    float lastGyroY()  const { return last_gy; }
    float lastGyroZ()  const { return last_gz; }

    struct StepInfo {
        int step_count;           // Total steps since reset
        double stride_length_m;   // Estimated stride length in meters
        double speed_mps;         // Estimated walking speed in m/s
        int64_t last_step_ns;     // Timestamp of last detected step

        // Step 3 Observer A: confidence components.
        // Lower variance + recent step → higher PDR confidence.
        double step_period_variance_s2;  // var of last N step periods (s²); -1 if unset
        double accel_variance;           // running variance of |accel| (m²/s⁴)
        double time_since_last_step_s;   // seconds since last detected step
        bool   stride_calibrated;        // true if a per-user stride was set
    };

    StepInfo getStepInfo() const;

    // Step 3 Observer A: per-user calibrated stride length.
    // Set after stride-learning walk: stride = distance / steps. Persisted in
    // SharedPreferences on the Java side; pushed in here on app startup.
    // <= 0 disables calibration → stride model falls back to height×0.415.
    void setUserStride(double stride_m);
    double getUserStride() const;
    bool hasCalibratedStride() const;

    // Motion mode detection: walking vs driving vs stationary
    enum class MotionMode { STATIONARY, WALKING, DRIVING };
    // DEAD CODE: getMotionMode — never called externally
    // MotionMode getMotionMode() const;

    // Vehicle speed estimate from integrated acceleration (reset at ZUPT)
    double getVehicleSpeedEstimate() const;

    // User height for stride estimation (default 1.70m)
    void setUserHeight(float height_m);
    float getUserHeight() const;

    // Magnetometer heading fusion
    void setMagnetometerHeading(float yaw_rad);
    // DEAD CODE: getCorrectedHeading — never called
    // float getCorrectedHeading(float gyro_yaw_rad);
    bool hasMagHeading() const { return has_mag_heading_.load(); }
    float getMagHeading() const { return mag_heading_; }

    // Gyro bias accessor (unified — Tracker no longer maintains its own)
    cv::Point3f getGyroBias() const;
    // Step 5: seed gyro bias from stored calibration (skips warmup samples).
    void setGyroBias(float bx, float by, float bz);

    // Refine gyro bias during stationary periods (called by Tracker ZUPT)
    // Uses recent gyro samples to nudge bias estimate with small alpha
    void refineGyroBiasDuringZUPT();

    // ── Madgwick IMU-only attitude filter ───────────────────────────────────
    // Full orientation as a unit quaternion (Hamilton, body→world).
    // Driven from every gyro sample with accelerometer correction toward
    // gravity. Replaces the scalar gravity-projected-yaw accumulator that
    // was corrupted by centripetal acceleration during fast rotation.
    // Returns (q0=w, q1=x, q2=y, q3=z).
    void getOrientationQuaternion(double& q0, double& q1, double& q2, double& q3) const;
    // Heading (yaw) in radians, CW-positive (navigation convention,
    // North=0, East=+π/2). Wraps to [-π, π].
    float getHeading() const;
    float getMadgwickRoll() const;
    float getMadgwickPitch() const;
    bool  isOrientationInitialized() const { return madgwick_init_.load(); }
    void  resetOrientationFilter();

    // Bootstrap-only: tell Madgwick what compass heading (CW-positive nav,
    // North=0, East=+π/2) the device is at, so its quaternion doesn't init
    // at yaw=0. Forces a re-init using the current accel for roll/pitch and
    // this yaw. Without this call, getHeading() returns 0 until the user
    // turns or visual yaw correction drags it. Bug discovered 2026-05-09:
    // a 180° heading flip immediately after init was caused by Madgwick
    // ignoring the magnetometer-derived setInitialHeading and starting at
    // yaw=0.
    void setInitialMadgwickYaw(float azimuth_rad_nav);

private:
    mutable std::mutex mutex_;
    std::vector<GyroSample>  gyro_buf_;
    std::vector<AccelSample> accel_buf_;
    static constexpr size_t MAX_BUF = 2000;
    std::vector<cv::Point3f> gravity_init_samples_;
    static constexpr size_t GRAVITY_INIT_WINDOW = 40;
    static constexpr float GRAVITY_INIT_MAX_VAR = 0.08f;
    static constexpr float GRAVITY_INIT_GYRO_MAX = 0.12f;

    std::atomic<bool> gravity_initialized_{false};
    cv::Point3f gravity_vec_{0.f, 0.f, 9.81f};
    float roll_{0.f}, pitch_{0.f};

    // Low-pass filtered gravity (tracks phone tilt changes during use)
    cv::Point3f filtered_gravity_{0.f, 0.f, 9.81f};
    bool filtered_gravity_init_{false};

    // Last sensor values for passthrough
    float last_ax{0.f}, last_ay{0.f}, last_az{0.f};
    float last_gx{0.f}, last_gy{0.f}, last_gz{0.f};

    // Step detector state
    int step_count_{0};
    int64_t last_step_ns_{0};
    double step_period_s_{0.0};       // Time between last two steps
    float accel_mag_filtered_{9.81f}; // Low-pass filtered accel magnitude
    float accel_mag_prev_{9.81f};     // Previous filtered value for peak detection
    bool was_above_thresh_{false};    // For peak detection hysteresis
    static constexpr float STEP_ACCEL_THRESH_HIGH = 10.1f; // m/s² peak threshold
    static constexpr float STEP_ACCEL_THRESH_LOW  = 9.3f;  // m/s² valley threshold
    static constexpr double MIN_STEP_PERIOD_S = 0.25;      // Max 4 steps/s (running)
    static constexpr double MAX_STEP_PERIOD_S = 1.5;       // Min ~0.67 steps/s (slow walk)
    static constexpr double DEFAULT_STRIDE_M  = 0.65;      // Average human stride
    float user_height_m_{1.70f};                              // User height for stride model

    // Step 3 Observer A: per-user calibrated stride. >0 = calibrated.
    // Set via setUserStride(). When set, overrides height-based model.
    double user_stride_m_{-1.0};

    // Step 3 Observer A: rolling buffer of last N step periods (seconds).
    // Used to compute step_period_variance for PDR confidence.
    static constexpr size_t STEP_PERIOD_BUF = 8;
    std::vector<double> step_period_buf_;

    // Step 3 Observer A: rotation gate for step suppression. 0.8 rad/s ≈ 46°/s,
    // well above normal arm-swing yaw (≈10°/s) but below in-place turn rate
    // (≈90°/s). Above this, step counter is suppressed to prevent the V-shape
    // PDR-during-rotation artifact (see project_vio_session_april08 memory).
    static constexpr float ROTATION_STEP_GATE_RADPS = 0.8f;

    // ── Step 8 Cleanup status (kept, NOT deleted) ───────────────────────────
    // The production-readiness plan (Step 8) calls for removing the
    // hand-rolled motion classifier and vehicle-speed integrator in favor of
    // a unified step detector + explicit rotation gate. The fields below are
    // retained as a per-user request ("comment, don't delete") because they
    // still serve as live gates inside detectStep() and updateMotionMode():
    //   • is_walking_pattern_ — gates peak detection in detectStep so road
    //     vibrations are not counted as steps.
    //   • vehicle_speed_mps_ / in_vehicle_mode_ — exposed via
    //     getVehicleSpeedEstimate() for the scooter-mode UI hint.
    // If/when Step 3's VI scale observer fully owns these signals, this
    // block becomes dead code; for now it is legacy-but-live.
    float accel_mag_slow_{9.81f};     // Very slow LP for walking detection (alpha ~0.02)
    float accel_variance_est_{0.0f};  // Running variance of accel magnitude
    bool is_walking_pattern_{false};  // LEGACY (Step 8): walking-vs-vibration classifier
    float gyro_mag_filtered_{0.0f};   // Low-pass filtered |gyro| in rad/s
                                      // Used to suppress phantom steps during in-place rotation

    // Vehicle motion detection — LEGACY (Step 8 cleanup), retained as
    // commented above. Used by getVehicleSpeedEstimate().
    double vehicle_speed_mps_{0.0};   // Integrated forward acceleration
    int64_t last_accel_ts_ns_{0};     // For dt computation
    double sustained_accel_s_{0.0};   // How long accel has been above threshold
    bool in_vehicle_mode_{false};     // True if we think we're in a vehicle

    void detectStep(int64_t timestamp_ns, float ax, float ay, float az);
    void updateMotionMode(int64_t timestamp_ns, float ax, float ay, float az);
    void tryInitializeGravityLocked(float ax, float ay, float az);
    void tryInitializeGyroBiasLocked(float ax, float ay, float az);

    // Madgwick: called from addGyroReading under mutex_. Uses bias-corrected
    // gyro and (when accel magnitude is in [5, 20] m/s²) the last accel
    // sample as a gravity reference.
    void updateMadgwickLocked(int64_t timestamp_ns, float gx, float gy, float gz);
    // Lazy init from current filtered_gravity_ (roll/pitch only; yaw = 0).
    void tryInitMadgwickLocked();

    // Madgwick state (unit quaternion, Hamilton body→world, w first).
    double q0_{1.0}, q1_{0.0}, q2_{0.0}, q3_{0.0};
    int64_t madgwick_last_ns_{0};
    std::atomic<bool> madgwick_init_{false};
    // Pending compass heading (CW-positive nav, radians) supplied by
    // setInitialMadgwickYaw before tryInitMadgwickLocked fires. Defaults
    // to 0 (= North) preserving prior behaviour when the bootstrap
    // never calls setInitialMadgwickYaw.
    float pending_madgwick_yaw_nav_{0.f};
    // Filter gain — tuned for ~200 Hz IMU. Higher β → faster accel
    // correction (more drift suppression) but more noise bleed-through.
    static constexpr float MADGWICK_BETA = 0.033f;
    // Accel validity gate: reject accel correction when |a| is outside
    // this window (centripetal accel during fast rotation, freefall, or
    // shock). Gyro still integrates, so orientation keeps advancing.
    static constexpr float MADGWICK_ACC_MIN = 5.0f;   // m/s²
    static constexpr float MADGWICK_ACC_MAX = 20.0f;  // m/s²

    // Bias estimates
    cv::Point3f gyro_bias_{0.f, 0.f, 0.f};              // Estimated bias in rad/sec
    cv::Point3f b_a_{0.f, 0.f, 0.f};                    // Accelerometer bias in m/s²
    std::atomic<bool> gyro_bias_initialized_{false};
    int gyro_bias_samples_{0};
    static constexpr int GYRO_BIAS_INIT_SAMPLES = 200;  // ~6-7 frames at 30fps

    // Magnetometer heading fusion
    float mag_heading_{0.f};                            // Yaw from magnetometer (rad)
    std::atomic<bool> has_mag_heading_{false};
    int64_t last_mag_update_ns_{0};
    static constexpr float HEADING_CORRECTION_DAMPING = 0.95f;  // Smooth fusion

    // Noise parameters
    float accel_noise_sigma_{0.1f};   // m/s^2/sqrt(Hz)
    float gyro_noise_sigma_{0.01f};    // rad/s/sqrt(Hz)
    float accel_rw_sigma_{0.001f};     // m/s^3/sqrt(Hz)
    float gyro_rw_sigma_{0.0001f};     // rad/s^2/sqrt(Hz)
};
