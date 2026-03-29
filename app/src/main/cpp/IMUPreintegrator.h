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
};
