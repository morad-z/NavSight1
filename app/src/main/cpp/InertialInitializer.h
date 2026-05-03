#pragma once
#include <vector>
#include <deque>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include "IMUPreintegrator.h"

/**
 * @brief Stationary-gate VIO initialization.
 *
 * Step 5 (Production Readiness Plan):
 *   1. Collect 5 s of stationary IMU samples.
 *   2. Require |mean(gyro)| < 0.02 rad/s and var(accel) < 0.01 m²/s⁴.
 *   3. If the gate passes within 15 s the initializer enters WAIT_MOTION,
 *      and READY once the user takes their first step.
 *   4. If the gate has not passed within 15 s the initializer enters
 *      TIMEOUT_NEEDS_USER. The Java side is expected to surface a
 *      "Place phone flat for 5 s" dialog. Calling `clearTimeout()` after
 *      the user acknowledges restarts the 15 s window.
 *
 * Stored calibration may be injected via `loadCalibration()` at startup —
 * this skips the gate entirely and transitions directly to WAIT_MOTION.
 */
class InertialInitializer {
public:
    enum class Status {
        WAIT_STATIONARY = 0,
        WAIT_MOTION = 1,
        READY = 2,
        TIMEOUT_NEEDS_USER = 3,
    };

    struct Options {
        double stationary_seconds;     // Plan: 5.0 s of quiet data
        double max_accel_var;          // Plan: 0.01 m²/s⁴
        double max_gyro_mag;           // Plan: 0.02 rad/s
        double timeout_seconds;        // Plan: 15.0 s before user prompt
        double gravity_mag;

        Options() :
            stationary_seconds(5.0),
            max_accel_var(0.01),
            max_gyro_mag(0.02),
            timeout_seconds(15.0),
            gravity_mag(9.81) {}
    };

    explicit InertialInitializer(const Options& options = Options());

    void addImuData(int64_t timestamp_ns, float ax, float ay, float az,
                    float gx, float gy, float gz);

    Status getStatus() const;
    bool isReady() const { return getStatus() == Status::READY; }
    bool needsUserPrompt() const { return getStatus() == Status::TIMEOUT_NEEDS_USER; }

    cv::Mat getInitialRotation() const;
    cv::Point3f getGyroBias() const;
    cv::Point3f getAccelBias() const;

    // Inject pre-computed calibration (e.g. SharedPreferences). Skips the
    // stationary gate and transitions directly to WAIT_MOTION.
    void loadCalibration(const cv::Mat& R_GtoI,
                         const cv::Point3f& gyro_bias,
                         const cv::Point3f& accel_bias);

    // User has acknowledged the timeout dialog. Restart the gate window.
    void clearTimeout();

    void reset();

private:
    bool runStationaryCalibration();
    bool detectMotion(float ax, float ay, float az);
    void trimAccelWindow();   // drop samples older than stationary_seconds

    Options options_;
    mutable std::mutex mutex_;
    Status state_{Status::WAIT_STATIONARY};

    std::deque<AccelSample> accel_window_;
    std::deque<GyroSample> gyro_window_;

    int64_t first_sample_ns_{0};      // Stamp of first sample since reset
    bool    has_first_sample_{false};

    cv::Mat R_GtoI_init_;
    cv::Point3f gyro_bias_;
    cv::Point3f accel_bias_;
};
