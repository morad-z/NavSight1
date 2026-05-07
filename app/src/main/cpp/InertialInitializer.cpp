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
    state_ = Status::WAIT_STATIONARY;
    accel_window_.clear();
    gyro_window_.clear();
    has_first_sample_ = false;
    first_sample_ns_ = 0;
    force_accept_ = false;
    gyro_bias_ = cv::Point3f(0, 0, 0);
    accel_bias_ = cv::Point3f(0, 0, 0);
    R_GtoI_init_ = cv::Mat::eye(3, 3, CV_64F);
}

InertialInitializer::Status InertialInitializer::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

cv::Mat InertialInitializer::getInitialRotation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return R_GtoI_init_.clone();
}

cv::Point3f InertialInitializer::getGyroBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gyro_bias_;
}

cv::Point3f InertialInitializer::getAccelBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accel_bias_;
}

void InertialInitializer::loadCalibration(const cv::Mat& R_GtoI,
                                           const cv::Point3f& gyro_bias,
                                           const cv::Point3f& accel_bias) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!R_GtoI.empty() && R_GtoI.rows == 3 && R_GtoI.cols == 3) {
        R_GtoI.convertTo(R_GtoI_init_, CV_64F);
    }
    gyro_bias_ = gyro_bias;
    accel_bias_ = accel_bias;
    state_ = Status::WAIT_MOTION;
    accel_window_.clear();
    gyro_window_.clear();
    has_first_sample_ = false;
    LOGI("Inertial calibration LOADED from store: gyro_bias=(%.4f,%.4f,%.4f) "
         "accel_bias=(%.4f,%.4f,%.4f)",
         gyro_bias.x, gyro_bias.y, gyro_bias.z,
         accel_bias.x, accel_bias.y, accel_bias.z);
}

void InertialInitializer::clearTimeout() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == Status::TIMEOUT_NEEDS_USER) {
        state_ = Status::WAIT_STATIONARY;
        accel_window_.clear();
        gyro_window_.clear();
        has_first_sample_ = false;
        first_sample_ns_ = 0;
        // User explicitly confirmed the phone is flat — skip quality checks
        // on the next 5-second window and accept whatever mean we collect.
        force_accept_ = true;
        LOGI("Init timeout cleared by user; force_accept enabled for next window");
    }
}

void InertialInitializer::trimAccelWindow() {
    if (accel_window_.empty()) return;
    int64_t window_ns = static_cast<int64_t>(options_.stationary_seconds * 1e9);
    int64_t now_ns = accel_window_.back().timestamp_ns;
    while (!accel_window_.empty()
           && now_ns - accel_window_.front().timestamp_ns > window_ns) {
        accel_window_.pop_front();
    }
    while (!gyro_window_.empty()
           && now_ns - gyro_window_.front().timestamp_ns > window_ns) {
        gyro_window_.pop_front();
    }
}

void InertialInitializer::addImuData(int64_t timestamp_ns,
                                      float ax, float ay, float az,
                                      float gx, float gy, float gz) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_ == Status::READY || state_ == Status::TIMEOUT_NEEDS_USER) {
        return;
    }

    if (!has_first_sample_) {
        first_sample_ns_ = timestamp_ns;
        has_first_sample_ = true;
    }

    if (state_ == Status::WAIT_STATIONARY) {
        accel_window_.push_back({timestamp_ns, ax, ay, az});
        gyro_window_.push_back({timestamp_ns, gx, gy, gz});
        trimAccelWindow();

        // Need a full stationary_seconds window before evaluating.
        int64_t window_ns = static_cast<int64_t>(options_.stationary_seconds * 1e9);
        bool window_full = !accel_window_.empty()
            && (accel_window_.back().timestamp_ns - accel_window_.front().timestamp_ns) >= window_ns;

        if (window_full) {
            if (runStationaryCalibration()) {
                state_ = Status::WAIT_MOTION;
                LOGI("Inertial calibration SUCCESS (window=%.1fs samples=%zu); "
                     "waiting for first step…",
                     options_.stationary_seconds, accel_window_.size());
                accel_window_.clear();
                gyro_window_.clear();
                return;
            }
            // Calibration rejected — keep sliding. The trim above already
            // bounds the buffer, so a steadier 5 s window can still pass.
        }

        // Timeout check: if user hasn't held it still in 15 s, prompt.
        int64_t elapsed_ns = timestamp_ns - first_sample_ns_;
        if (elapsed_ns > static_cast<int64_t>(options_.timeout_seconds * 1e9)) {
            state_ = Status::TIMEOUT_NEEDS_USER;
            LOGE("Inertial calibration TIMEOUT after %.1fs; user prompt required",
                 elapsed_ns * 1e-9);
        }
    } else if (state_ == Status::WAIT_MOTION) {
        if (detectMotion(ax, ay, az)) {
            state_ = Status::READY;
            LOGI("Motion DETECTED: VIO ready.");
        }
    }
}

bool InertialInitializer::runStationaryCalibration() {
    if (accel_window_.empty() || gyro_window_.empty()) return false;

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (const auto& a : accel_window_) { sum_ax += a.x; sum_ay += a.y; sum_az += a.z; }
    for (const auto& g : gyro_window_)  { sum_gx += g.x; sum_gy += g.y; sum_gz += g.z; }

    double n_a = static_cast<double>(accel_window_.size());
    double n_g = static_cast<double>(gyro_window_.size());
    cv::Point3f mean_a(sum_ax / n_a, sum_ay / n_a, sum_az / n_a);
    cv::Point3f mean_g(sum_gx / n_g, sum_gy / n_g, sum_gz / n_g);

    double var_a = 0;
    for (const auto& a : accel_window_) {
        var_a += std::pow(a.x - mean_a.x, 2)
               + std::pow(a.y - mean_a.y, 2)
               + std::pow(a.z - mean_a.z, 2);
    }
    var_a /= n_a;

    // Gyro variance — check stability, NOT mean magnitude.
    // The mean IS the bias we are measuring; it can be 0.05+ rad/s on many
    // devices. Checking mean magnitude against a small threshold (the old bug)
    // caused permanent rejection on any phone whose factory gyro bias exceeds
    // ~0.02 rad/s, which is most consumer handsets.
    double var_g = 0;
    for (const auto& g : gyro_window_) {
        var_g += std::pow(g.x - mean_g.x, 2)
               + std::pow(g.y - mean_g.y, 2)
               + std::pow(g.z - mean_g.z, 2);
    }
    var_g /= n_g;

    if (force_accept_) {
        // User explicitly said the phone is flat: skip variance thresholds.
        // Even a noisy bias estimate is better than never starting VIO.
        // The EKF will refine b_g online within the first few seconds of walking.
        LOGI("Init gate FORCE-ACCEPT (user-confirmed): var_a=%.5f var_g=%.6f", var_a, var_g);
        force_accept_ = false;
    } else {
        if (var_a > options_.max_accel_var) {
            LOGI("Init gate REJECT: var_a=%.5f > %.5f", var_a, options_.max_accel_var);
            return false;
        }
        if (var_g > options_.max_gyro_var) {
            LOGI("Init gate REJECT: var_g=%.6f > %.6f (rad/s)^2",
                 var_g, options_.max_gyro_var);
            return false;
        }
    }

    // Gravity alignment: compute roll/pitch from mean_a; yaw stays free
    // (mag-yaw is consumed once via Tracker::setInitialHeading at startup).
    cv::Point3f z_axis = mean_a;
    float mag = std::sqrt(z_axis.x * z_axis.x
                        + z_axis.y * z_axis.y
                        + z_axis.z * z_axis.z);
    if (mag < 1e-6f) return false;
    z_axis.x /= mag; z_axis.y /= mag; z_axis.z /= mag;

    cv::Point3f x_axis(1.0f, 0.0f, 0.0f);
    if (std::abs(z_axis.x) > 0.9f) x_axis = cv::Point3f(0.0f, 1.0f, 0.0f);

    cv::Point3f y_axis = z_axis.cross(x_axis);
    mag = std::sqrt(y_axis.x * y_axis.x
                  + y_axis.y * y_axis.y
                  + y_axis.z * y_axis.z);
    if (mag < 1e-6f) return false;
    y_axis.x /= mag; y_axis.y /= mag; y_axis.z /= mag;
    x_axis = y_axis.cross(z_axis);

    cv::Mat R_ItoG = (cv::Mat_<double>(3, 3) <<
        x_axis.x, y_axis.x, z_axis.x,
        x_axis.y, y_axis.y, z_axis.y,
        x_axis.z, y_axis.z, z_axis.z);

    R_GtoI_init_ = R_ItoG.t();
    gyro_bias_ = mean_g;
    // Stationary alone cannot separate accel bias from gravity; report 0
    // and rely on Madgwick + IMUPreintegrator for online refinement.
    accel_bias_ = cv::Point3f(0, 0, 0);
    double bias_mag = std::sqrt(mean_g.x * mean_g.x
                              + mean_g.y * mean_g.y
                              + mean_g.z * mean_g.z);
    LOGI("Init gate PASS: var_a=%.5f var_g=%.6f "
         "gyro_bias=(%.4f,%.4f,%.4f) |bias|=%.4f rad/s samples=%zu",
         var_a, var_g, mean_g.x, mean_g.y, mean_g.z, bias_mag, accel_window_.size());
    return true;
}

bool InertialInitializer::detectMotion(float ax, float ay, float az) {
    double mag = std::sqrt(ax * ax + ay * ay + az * az);
    return std::abs(mag - options_.gravity_mag) > 0.5;
}
