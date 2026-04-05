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

struct PreintegratedDelta {
    cv::Mat deltaR; // 3x3 rotation
    cv::Mat deltaV; // 3x1 velocity
    cv::Mat deltaP; // 3x1 position
    double dt;
    int sample_count;
};

class IMUPreintegrator {
public:
    IMUPreintegrator();

    void addGyroReading(int64_t timestamp_ns, float x, float y, float z);
    void addAccelReading(int64_t timestamp_ns, float x, float y, float z);

    // Integrate gyroscope readings between start_ns and end_ns into a rotation matrix
    cv::Mat integrateGyro(int64_t start_ns, int64_t end_ns);

    // Full preintegration (returns deltaR, deltaV, deltaP for the interval)
    PreintegratedDelta integrate(int64_t start_ns, int64_t end_ns);

    // Initialize gravity vector from a batch of accelerometer samples
    void initializeFromGravity(const std::vector<cv::Point3f>& accel_samples);
    void setGravity(float gx, float gy, float gz);

    cv::Point3f getGravityVector() const;
    float getRoll() const;
    float getPitch() const;
    bool isInitialized() const;

    void reset();

    std::vector<AccelSample> getAccelBuffer() const;

    // Last known sensor values (for VioData passthrough)
    float lastAccelX() const { return last_ax; }
    float lastAccelY() const { return last_ay; }
    float lastAccelZ() const { return last_az; }
    float lastGyroX()  const { return last_gx; }
    float lastGyroY()  const { return last_gy; }
    float lastGyroZ()  const { return last_gz; }

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

    // Last sensor values for passthrough
    float last_ax{0.f}, last_ay{0.f}, last_az{0.f};
    float last_gx{0.f}, last_gy{0.f}, last_gz{0.f};

    // ── Step detection ──────────────────────────────────────────────────────
public:
    struct StepInfo {
        int step_count;           // Total steps since reset
        double stride_length_m;   // Estimated stride length in meters
        double speed_mps;         // Estimated walking speed in m/s
        int64_t last_step_ns;     // Timestamp of last detected step
    };

    StepInfo getStepInfo() const;

    // Motion mode detection: walking vs driving vs stationary
    enum class MotionMode { STATIONARY, WALKING, DRIVING };
    MotionMode getMotionMode() const;

    // Vehicle speed estimate from integrated acceleration (reset at ZUPT)
    double getVehicleSpeedEstimate() const;

private:
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

    // Walking validation: reject car vibrations masquerading as steps
    // Real walking has accel magnitude variance > 0.3 m/s² over ~1 second
    // Car vibrations have smaller, higher-frequency oscillations
    float accel_mag_slow_{9.81f};     // Very slow LP for walking detection (alpha ~0.02)
    float accel_variance_est_{0.0f};  // Running variance of accel magnitude
    bool is_walking_pattern_{false};  // True if recent accel matches walking, not vibration

    // Vehicle motion detection
    double vehicle_speed_mps_{0.0};   // Integrated forward acceleration
    int64_t last_accel_ts_ns_{0};     // For dt computation
    double sustained_accel_s_{0.0};   // How long accel has been above threshold
    bool in_vehicle_mode_{false};     // True if we think we're in a vehicle

    void detectStep(int64_t timestamp_ns, float ax, float ay, float az);
    void updateMotionMode(int64_t timestamp_ns, float ax, float ay, float az);
    void tryInitializeGravityLocked(float ax, float ay, float az);
    void tryInitializeGyroBiasLocked(float ax, float ay, float az);

    // ── Gyroscope bias correction ──────────────────────────────────────────
    // Gyroscope bias is a constant offset that accumulates over time.
    // Estimate it during stationary initialization, subtract during integration.
private:
    cv::Point3f gyro_bias_{0.f, 0.f, 0.f};              // Estimated bias in rad/sec
    std::atomic<bool> gyro_bias_initialized_{false};
    int gyro_bias_samples_{0};
    static constexpr int GYRO_BIAS_INIT_SAMPLES = 200;  // ~6-7 frames at 30fps

    // ── Magnetometer heading fusion ────────────────────────────────────────
    // Magnetometer provides absolute heading reference (doesn't accumulate error)
public:
    void setMagnetometerHeading(float yaw_rad);
    float getCorrectedHeading(float gyro_yaw_rad);

    // User height for stride estimation (default 1.70m)
    void setUserHeight(float height_m);
    float getUserHeight() const;

private:
    float mag_heading_{0.f};                            // Yaw from magnetometer (rad)
    std::atomic<bool> has_mag_heading_{false};
    int64_t last_mag_update_ns_{0};
    static constexpr float HEADING_CORRECTION_DAMPING = 0.95f;  // Smooth fusion
};
