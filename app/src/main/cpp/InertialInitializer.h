#pragma once
#include <vector>
#include <deque>
#include <mutex>
#include <opencv2/core.hpp>
#include "IMUPreintegrator.h"

/**
 * @brief Handles VIO initialization by estimating initial gravity and gyro bias.
 */
class InertialInitializer {
public:
    enum class State { WAIT_STATIONARY, WAIT_MOTION, READY };

    struct Options {
        int min_stationary_samples;
        double max_accel_var;
        double max_gyro_var;
        double gravity_mag;

        Options() :
            min_stationary_samples(60),    // ~0.3s at 200Hz; faster init
            max_accel_var(0.15),           // Phone in hand has ~0.03-0.10 accel var
            max_gyro_var(0.02),            // Phone gyro var ~0.005-0.015 when hand-held
            gravity_mag(9.81) {}
    };

    InertialInitializer(const Options& options = Options());

    void addImuData(int64_t timestamp_ns, float ax, float ay, float az,
                    float gx, float gy, float gz);

    bool isReady() const { return state_ == State::READY; }

    cv::Mat getInitialRotation() const { return R_GtoI_init_.clone(); }
    cv::Point3f getGyroBias() const { return gyro_bias_; }

    void reset();

private:
    bool runStationaryCalibration();
    bool detectMotion(float ax, float ay, float az);

    Options options_;
    mutable std::mutex mutex_;
    State state_{State::WAIT_STATIONARY};

    std::deque<AccelSample> accel_window_;
    std::deque<GyroSample> gyro_window_;

    cv::Mat R_GtoI_init_;
    cv::Point3f gyro_bias_;
    cv::Point3f accel_bias_;
};
