#include "InertialInitializer.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Init"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#endif

InertialInitializer::InertialInitializer(const Options& options) 
    : options_(options) {
    reset();
}

void InertialInitializer::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::WAIT_STATIONARY;
    accel_window_.clear();
    gyro_window_.clear();
    gyro_bias_ = cv::Point3f(0,0,0);
    accel_bias_ = cv::Point3f(0,0,0);
    R_GtoI_init_ = cv::Mat::eye(3, 3, CV_64F);
}

void InertialInitializer::addImuData(int64_t timestamp_ns, float ax, float ay, float az,
                                    float gx, float gy, float gz) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == State::WAIT_STATIONARY) {
        accel_window_.push_back({timestamp_ns, ax, ay, az});
        gyro_window_.push_back({timestamp_ns, gx, gy, gz});

        if (accel_window_.size() >= static_cast<size_t>(options_.min_stationary_samples)) {
            if (runStationaryCalibration()) {
                state_ = State::WAIT_MOTION;
                LOGI("Inertial Calibration SUCCESS: roll/pitch aligned, bias estimated. Waiting for motion...");
            } else {
                // Keep windows sliding if calibration fails
                accel_window_.pop_front();
                gyro_window_.pop_front();
            }
        }
    } else if (state_ == State::WAIT_MOTION) {
        if (detectMotion(ax, ay, az)) {
            state_ = State::READY;
            LOGI("Motion DETECTED: Initialization complete, VIO ready.");
        }
    }
}

bool InertialInitializer::runStationaryCalibration() {
    // 1. Compute means and variances
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (const auto& a : accel_window_) { sum_ax += a.x; sum_ay += a.y; sum_az += a.z; }
    for (const auto& g : gyro_window_) { sum_gx += g.x; sum_gy += g.y; sum_gz += g.z; }

    double n = static_cast<double>(accel_window_.size());
    cv::Point3f mean_a(sum_ax / n, sum_ay / n, sum_az / n);
    cv::Point3f mean_g(sum_gx / n, sum_gy / n, sum_gz / n);

    double var_a = 0, var_g = 0;
    for (const auto& a : accel_window_) {
        var_a += std::pow(a.x - mean_a.x, 2) + std::pow(a.y - mean_a.y, 2) + std::pow(a.z - mean_a.z, 2);
    }
    for (const auto& g : gyro_window_) {
        var_g += std::pow(g.x - mean_g.x, 2) + std::pow(g.y - mean_g.y, 2) + std::pow(g.z - mean_g.z, 2);
    }
    var_a /= n; var_g /= n;

    // 2. Statistical checks
    if (var_a > options_.max_accel_var || var_g > options_.max_gyro_var) {
        return false;
    }

    // 3. Gravity Alignment
    // Gravity in Inertial frame is mean_a. We want to find R_GtoI such that
    // R_GtoI * [0,0,g] = mean_a  => R_ItoG * mean_a = [0,0,g]
    cv::Point3f z_axis = mean_a;
    float mag = std::sqrt(z_axis.x * z_axis.x + z_axis.y * z_axis.y + z_axis.z * z_axis.z);
    z_axis.x /= mag; z_axis.y /= mag; z_axis.z /= mag;

    // Use Gram-Schmidt to find orientation (arbitrary yaw)
    cv::Point3f x_axis(1.0, 0.0, 0.0);
    if (std::abs(z_axis.x) > 0.9) x_axis = cv::Point3f(0.0, 1.0, 0.0);
    
    cv::Point3f y_axis = z_axis.cross(x_axis);
    mag = std::sqrt(y_axis.x * y_axis.x + y_axis.y * y_axis.y + y_axis.z * y_axis.z);
    y_axis.x /= mag; y_axis.y /= mag; y_axis.z /= mag;

    x_axis = y_axis.cross(z_axis);

    // R_ItoG = [x_axis, y_axis, z_axis]
    cv::Mat R_ItoG = (cv::Mat_<double>(3, 3) << 
        x_axis.x, y_axis.x, z_axis.x,
        x_axis.y, y_axis.y, z_axis.y,
        x_axis.z, y_axis.z, z_axis.z);
    
    R_GtoI_init_ = R_ItoG.t();
    gyro_bias_ = mean_g;
    accel_bias_ = cv::Point3f(0,0,0); // bias_a is hard to separate from gravity while stationary

    return true;
}

bool InertialInitializer::detectMotion(float ax, float ay, float az) {
    // Look for a sudden change in acceleration magnitude
    double mag = std::sqrt(ax*ax + ay*ay + az*az);
    if (std::abs(mag - options_.gravity_mag) > 0.5) {  // Was 0.8; too high for gentle motion
        return true;
    }
    return false;
}
