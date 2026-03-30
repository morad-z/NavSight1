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

private:
    // Step detector state
    int step_count_{0};
    int64_t last_step_ns_{0};
    double step_period_s_{0.0};       // Time between last two steps
    float accel_mag_filtered_{9.81f}; // Low-pass filtered accel magnitude
    float accel_mag_prev_{9.81f};     // Previous filtered value for peak detection
    bool was_above_thresh_{false};    // For peak detection hysteresis
    static constexpr float STEP_ACCEL_THRESH_HIGH = 10.1f; // m/s² peak threshold (lowered for sensitivity)
    static constexpr float STEP_ACCEL_THRESH_LOW  = 9.3f;  // m/s² valley threshold (tightened hysteresis)
    static constexpr double MIN_STEP_PERIOD_S = 0.25;      // Max 4 steps/s (running)
    static constexpr double MAX_STEP_PERIOD_S = 1.5;       // Min ~0.67 steps/s (slow walk)
    static constexpr double DEFAULT_STRIDE_M  = 0.65;      // Average human stride

    void detectStep(int64_t timestamp_ns, float ax, float ay, float az);
};
