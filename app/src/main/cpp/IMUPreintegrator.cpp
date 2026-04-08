#include "IMUPreintegrator.h"
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#endif

// ── Constructor ───────────────────────────────────────────────────────────────

IMUPreintegrator::IMUPreintegrator() {
    gyro_buf_.reserve(MAX_BUF);
    accel_buf_.reserve(MAX_BUF);
}

// ── PreintegratedMeasurement ──────────────────────────────────────────────────

PreintegratedMeasurement::PreintegratedMeasurement() {
    deltaR = cv::Mat::eye(3, 3, CV_64F);
    deltaV = cv::Mat::zeros(3, 1, CV_64F);
    deltaP = cv::Mat::zeros(3, 1, CV_64F);
    cov    = cv::Mat::zeros(9, 9, CV_64F);
    
    J_R_bg = cv::Mat::zeros(3, 3, CV_64F);
    J_V_bg = cv::Mat::zeros(3, 3, CV_64F);
    J_V_ba = cv::Mat::zeros(3, 3, CV_64F);
    J_P_bg = cv::Mat::zeros(3, 3, CV_64F);
    J_P_ba = cv::Mat::zeros(3, 3, CV_64F);
    
    dt = 0.0;
    sample_count = 0;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static cv::Mat skewSymmetric(const cv::Mat& v) {
    cv::Mat m = cv::Mat::zeros(3, 3, CV_64F);
    m.at<double>(0, 1) = -v.at<double>(2);
    m.at<double>(0, 2) =  v.at<double>(1);
    m.at<double>(1, 0) =  v.at<double>(2);
    m.at<double>(1, 2) = -v.at<double>(0);
    m.at<double>(2, 0) = -v.at<double>(1);
    m.at<double>(2, 1) =  v.at<double>(0);
    return m;
}

static inline bool isValidSensorFloat(float v) {
    return !std::isnan(v) && !std::isinf(v);
}

void IMUPreintegrator::setNoiseParameters(float accel_noise, float gyro_noise, 
                                        float accel_rw, float gyro_rw) {
    std::lock_guard<std::mutex> lock(mutex_);
    accel_noise_sigma_ = accel_noise;
    gyro_noise_sigma_ = gyro_noise;
    accel_rw_sigma_ = accel_rw;
    gyro_rw_sigma_ = gyro_rw;
}

// DEAD CODE: correctMeasurement — never called
// void IMUPreintegrator::correctMeasurement(PreintegratedMeasurement& meas,
//                                          const cv::Point3f& delta_bg,
//                                          const cv::Point3f& delta_ba) {
//     cv::Mat dbg = (cv::Mat_<double>(3,1) << delta_bg.x, delta_bg.y, delta_bg.z);
//     cv::Mat dba = (cv::Mat_<double>(3,1) << delta_ba.x, delta_ba.y, delta_ba.z);
//     cv::Mat dR_cor_vec = meas.J_R_bg * dbg;
//     cv::Mat dR_cor;
//     cv::Rodrigues(dR_cor_vec, dR_cor);
//     meas.deltaR = meas.deltaR * dR_cor;
//     meas.deltaV += meas.J_V_bg * dbg + meas.J_V_ba * dba;
//     meas.deltaP += meas.J_P_bg * dbg + meas.J_P_ba * dba;
// }

// ── addGyroReading ────────────────────────────────────────────────────────────

void IMUPreintegrator::addGyroReading(int64_t timestamp_ns, float x, float y, float z) {
    if (!isValidSensorFloat(x) || !isValidSensorFloat(y) || !isValidSensorFloat(z)) {
        LOGE("addGyroReading: NaN/Inf value, dropping sample");
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_gx = x; last_gy = y; last_gz = z;
    if (gyro_buf_.size() >= MAX_BUF) {
        // Erase oldest half in one shot to amortise O(n) cost
        gyro_buf_.erase(gyro_buf_.begin(),
                        gyro_buf_.begin() + static_cast<ptrdiff_t>(MAX_BUF / 2));
    }
    gyro_buf_.push_back({timestamp_ns, x, y, z});
}

// ── addAccelReading ───────────────────────────────────────────────────────────

void IMUPreintegrator::addAccelReading(int64_t timestamp_ns, float x, float y, float z) {
    if (!isValidSensorFloat(x) || !isValidSensorFloat(y) || !isValidSensorFloat(z)) {
        LOGE("addAccelReading: NaN/Inf value, dropping sample");
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_ax = x; last_ay = y; last_az = z;
    if (accel_buf_.size() >= MAX_BUF) {
        accel_buf_.erase(accel_buf_.begin(),
                         accel_buf_.begin() + static_cast<ptrdiff_t>(MAX_BUF / 2));
    }
    accel_buf_.push_back({timestamp_ns, x, y, z});

    // Low-pass filter accelerometer to track current gravity direction.
    // At ~200 Hz IMU rate, alpha=0.02 gives ~1.6 s time constant —
    // fast enough to follow phone tilts, slow enough to reject steps/vibration.
    if (!filtered_gravity_init_) {
        filtered_gravity_ = cv::Point3f(x, y, z);
        filtered_gravity_init_ = true;
    } else {
        constexpr float alpha = 0.02f;
        filtered_gravity_.x = alpha * x + (1.0f - alpha) * filtered_gravity_.x;
        filtered_gravity_.y = alpha * y + (1.0f - alpha) * filtered_gravity_.y;
        filtered_gravity_.z = alpha * z + (1.0f - alpha) * filtered_gravity_.z;
    }

    updateMotionMode(timestamp_ns, x, y, z);
    detectStep(timestamp_ns, x, y, z);
    tryInitializeGravityLocked(x, y, z);
    tryInitializeGyroBiasLocked(x, y, z);  // ✅ Gyro bias initialization during stationary phase
}

void IMUPreintegrator::tryInitializeGravityLocked(float ax, float ay, float az) {
    if (gravity_initialized_.load()) {
        gravity_init_samples_.clear();
        return;
    }

    gravity_init_samples_.emplace_back(ax, ay, az);
    if (gravity_init_samples_.size() > GRAVITY_INIT_WINDOW) {
        gravity_init_samples_.erase(gravity_init_samples_.begin());
    }

    if (gravity_init_samples_.size() < GRAVITY_INIT_WINDOW) {
        return;
    }

    double mean_mag = 0.0;
    for (const auto& sample : gravity_init_samples_) {
        mean_mag += std::sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);
    }
    mean_mag /= static_cast<double>(gravity_init_samples_.size());

    double var_mag = 0.0;
    cv::Point3f avg(0.f, 0.f, 0.f);
    for (const auto& sample : gravity_init_samples_) {
        const double mag = std::sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);
        const double diff = mag - mean_mag;
        var_mag += diff * diff;
        avg.x += sample.x;
        avg.y += sample.y;
        avg.z += sample.z;
    }
    var_mag /= static_cast<double>(gravity_init_samples_.size());

    const float inv_n = 1.0f / static_cast<float>(gravity_init_samples_.size());
    avg.x *= inv_n;
    avg.y *= inv_n;
    avg.z *= inv_n;

    const double gyro_mag = std::sqrt(
        static_cast<double>(last_gx) * last_gx +
        static_cast<double>(last_gy) * last_gy +
        static_cast<double>(last_gz) * last_gz
    );

    if (var_mag > GRAVITY_INIT_MAX_VAR || gyro_mag > GRAVITY_INIT_GYRO_MAX) {
        return;
    }

    const float mag = std::sqrt(avg.x * avg.x + avg.y * avg.y + avg.z * avg.z);
    if (mag <= 1e-3f) {
        return;
    }

    const float scale = 9.81f / mag;
    gravity_vec_ = cv::Point3f(avg.x * scale, avg.y * scale, avg.z * scale);
    roll_ = std::atan2(gravity_vec_.y, gravity_vec_.z);
    pitch_ = std::atan2(-gravity_vec_.x,
                        std::sqrt(gravity_vec_.y * gravity_vec_.y + gravity_vec_.z * gravity_vec_.z));
    gravity_initialized_.store(true);
    gravity_init_samples_.clear();

    LOGI("Gravity initialized from stationary window: gravity=(%.3f,%.3f,%.3f) var=%.5f gyro=%.4f",
         gravity_vec_.x, gravity_vec_.y, gravity_vec_.z, var_mag, gyro_mag);
}

// DEAD CODE: integrateGyro — superseded by full integrate() which returns PreintegratedMeasurement
// cv::Mat IMUPreintegrator::integrateGyro(int64_t start_ns, int64_t end_ns) { ... }

// ── integrate ─────────────────────────────────────────────────────────────────

PreintegratedMeasurement IMUPreintegrator::integrate(int64_t start_ns, int64_t end_ns) {
    PreintegratedMeasurement out;
    if (start_ns >= end_ns) {
        LOGE("integrate: invalid time window");
        return out;
    }

    std::vector<GyroSample> g_samples;
    std::vector<AccelSample> a_samples;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& s : gyro_buf_) {
            if (s.timestamp_ns >= start_ns && s.timestamp_ns <= end_ns) g_samples.push_back(s);
        }
        for (const auto& s : accel_buf_) {
            if (s.timestamp_ns >= start_ns && s.timestamp_ns <= end_ns) a_samples.push_back(s);
        }
    }

    if (g_samples.empty() || a_samples.empty()) {
        out.dt = (end_ns - start_ns) * 1e-9;
        return out;
    }

    auto sort_fn = [](const auto& a, const auto& b) { return a.timestamp_ns < b.timestamp_ns; };
    std::sort(g_samples.begin(), g_samples.end(), sort_fn);
    std::sort(a_samples.begin(), a_samples.end(), sort_fn);

    cv::Mat current_R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat current_V = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat current_P = cv::Mat::zeros(3, 1, CV_64F);
    
    cv::Mat P = cv::Mat::zeros(9, 9, CV_64F);
    cv::Mat J_R_bg = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat J_V_bg = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat J_V_ba = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat J_P_bg = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat J_P_ba = cv::Mat::zeros(3, 3, CV_64F);

    cv::Mat Q = cv::Mat::eye(6, 6, CV_64F);
    cv::Mat Q_gyro = Q(cv::Range(0,3), cv::Range(0,3));
    Q_gyro *= (gyro_noise_sigma_ * gyro_noise_sigma_);
    cv::Mat Q_accel = Q(cv::Range(3,6), cv::Range(3,6));
    Q_accel *= (accel_noise_sigma_ * accel_noise_sigma_);

    size_t gi = 0, ai = 0;
    int64_t t_curr = start_ns;

    while (t_curr < end_ns) {
        int64_t t_next = end_ns;
        if (gi < g_samples.size()) t_next = std::min(t_next, g_samples[gi].timestamp_ns);
        if (ai < a_samples.size()) t_next = std::min(t_next, a_samples[ai].timestamp_ns);
        
        if (t_next <= t_curr) {
            if (gi < g_samples.size() && g_samples[gi].timestamp_ns <= t_curr) gi++;
            if (ai < a_samples.size() && a_samples[ai].timestamp_ns <= t_curr) ai++;
            continue;
        }

        double dt = (t_next - t_curr) * 1e-9;
        
        cv::Mat w = (cv::Mat_<double>(3,1) << 0,0,0);
        cv::Mat a = (cv::Mat_<double>(3,1) << 0,0,0);
        if (gi > 0) w = (cv::Mat_<double>(3,1) << g_samples[gi-1].x - gyro_bias_.x, 
                                                 g_samples[gi-1].y - gyro_bias_.y, 
                                                 g_samples[gi-1].z - gyro_bias_.z);
        // Subtract accel bias (Bug #2: was missing — only gyro bias was subtracted)
        if (ai > 0) a = (cv::Mat_<double>(3,1) << a_samples[ai-1].x - b_a_.x,
                                                   a_samples[ai-1].y - b_a_.y,
                                                   a_samples[ai-1].z - b_a_.z);

        // ── Rotation: proper SO(3) via exponential map (Rodrigues) ──────────
        // Bug #1 fix: never add matrices on SO(3). Use R_new = R * Exp(w*dt).
        cv::Mat w_dt = w * dt;
        cv::Mat dR_step;
        cv::Rodrigues(w_dt, dR_step);

        // ── Velocity & Position: midpoint integration ───────────────────────
        // Use rotation at midpoint for better accuracy (Forster 2017 §IV.A)
        cv::Mat w_half = w * (dt * 0.5);
        cv::Mat dR_half;
        cv::Rodrigues(w_half, dR_half);
        cv::Mat R_mid = current_R * dR_half;

        cv::Mat dV = R_mid * a * dt;
        cv::Mat dP = current_V * dt + 0.5 * R_mid * a * dt * dt;

        // ── Covariance & Jacobian Propagation (Forster 2017) ────────────────
        cv::Mat F = cv::Mat::eye(9, 9, CV_64F);
        // w_dt and dR_step already computed above
        cv::Mat dR_step_t = dR_step.t();
        dR_step_t.copyTo(F(cv::Range(0,3), cv::Range(0,3)));
        cv::Mat F30 = -current_R * skewSymmetric(a) * dt;
        F30.copyTo(F(cv::Range(3,6), cv::Range(0,3)));
        cv::Mat F60 = -0.5 * current_R * skewSymmetric(a) * dt * dt;
        F60.copyTo(F(cv::Range(6,9), cv::Range(0,3)));
        cv::Mat F63 = cv::Mat::eye(3, 3, CV_64F) * dt;
        F63.copyTo(F(cv::Range(6,9), cv::Range(3,6)));

        cv::Mat G = cv::Mat::zeros(9, 6, CV_64F);
        cv::Mat G00 = cv::Mat::eye(3, 3, CV_64F) * dt;
        G00.copyTo(G(cv::Range(0,3), cv::Range(0,3)));
        cv::Mat G33 = current_R * dt;
        G33.copyTo(G(cv::Range(3,6), cv::Range(3,6)));
        cv::Mat G63 = 0.5 * current_R * dt * dt;
        G63.copyTo(G(cv::Range(6,9), cv::Range(3,6)));

        P = F * P * F.t() + G * Q * G.t();

        // Update Jacobians
        J_P_bg += J_V_bg * dt - 0.5 * current_R * skewSymmetric(a) * dt * dt;
        J_P_ba += J_V_ba * dt + 0.5 * current_R * dt * dt;
        J_V_bg += -current_R * skewSymmetric(a) * dt;
        J_V_ba += current_R * dt;
        
        cv::Mat Jr_inv; // Right Jacobian inverse approximation
        Jr_inv = cv::Mat::eye(3, 3, CV_64F) - 0.5 * skewSymmetric(w_dt);
        J_R_bg = dR_step.t() * J_R_bg - Jr_inv * dt;

        // Apply state updates — rotation via multiplication on SO(3)
        current_R = current_R * dR_step;
        current_V += dV;
        current_P += dP;

        t_curr = t_next;
    }

    out.deltaR = current_R;
    out.deltaV = current_V;
    out.deltaP = current_P;
    out.cov = P;
    out.J_R_bg = J_R_bg;
    out.J_V_bg = J_V_bg;
    out.J_V_ba = J_V_ba;
    out.J_P_bg = J_P_bg;
    out.J_P_ba = J_P_ba;
    out.dt = (end_ns - start_ns) * 1e-9;
    out.sample_count = (int)g_samples.size();

    return out;
}

// DEAD CODE: initializeFromGravity — never called (gravity init happens in tryInitializeGravityLocked)
// void IMUPreintegrator::initializeFromGravity(const std::vector<cv::Point3f>& accel_samples) { ... }

// DEAD CODE: setGravity — never called
// void IMUPreintegrator::setGravity(float gx, float gy, float gz) { ... }

// ── accessors ─────────────────────────────────────────────────────────────────

std::vector<AccelSample> IMUPreintegrator::getAccelBuffer() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accel_buf_;
}

std::vector<GyroSample> IMUPreintegrator::getGyroBuffer() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gyro_buf_;
}

// DEAD CODE: getGravityVector — never called (only getFilteredGravity is used)
// cv::Point3f IMUPreintegrator::getGravityVector() const {
//     std::lock_guard<std::mutex> lock(mutex_);
//     return gravity_vec_;
// }

cv::Point3f IMUPreintegrator::getFilteredGravity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return filtered_gravity_;
}

// DEAD CODE: getRoll/getPitch — never called
// float IMUPreintegrator::getRoll() const { return roll_; }
// float IMUPreintegrator::getPitch() const { return pitch_; }

bool IMUPreintegrator::isInitialized() const {
    return gravity_initialized_.load();
}

// ── Motion mode detection ─────────────────────────────────────────────────────

void IMUPreintegrator::updateMotionMode(int64_t timestamp_ns, float ax, float ay, float az) {
    float mag = std::sqrt(ax * ax + ay * ay + az * az);

    // Filter baseline gravity
    constexpr float SLOW_ALPHA = 0.02f;
    accel_mag_slow_ = (1.0f - SLOW_ALPHA) * accel_mag_slow_ + SLOW_ALPHA * mag;

    // Running variance (energy)
    float diff = mag - accel_mag_slow_;
    accel_variance_est_ = (1.0f - SLOW_ALPHA) * accel_variance_est_ + SLOW_ALPHA * (diff * diff);

    // ── SMARTER CLASSIFICATION ──────────────────────────────────────────────
    // Use Hysteresis: harder to start walking than to keep walking.
    if (!is_walking_pattern_ && accel_variance_est_ > 0.20f) {
        is_walking_pattern_ = true;
    } else if (is_walking_pattern_ && accel_variance_est_ < 0.05f) {
        is_walking_pattern_ = false;
    }

    // Vehicle speed integration
    if (last_accel_ts_ns_ > 0) {
        double dt = (timestamp_ns - last_accel_ts_ns_) * 1e-9;
        if (dt > 0.0 && dt < 0.1) {
            float forward_accel = mag - accel_mag_slow_;

            if (std::abs(forward_accel) > 0.2f) {
                vehicle_speed_mps_ += forward_accel * dt;
            } else {
                vehicle_speed_mps_ *= 0.997; // Slower decay for better continuity
            }

            vehicle_speed_mps_ = std::max(0.0, std::min(50.0, vehicle_speed_mps_));

            // CRITICAL FIX: Only exit vehicle mode if we detect a SUSTAINED walking pattern.
            // This prevents road bumps from accidentally zeroing out car speed.
            if (accel_variance_est_ < 0.10f && vehicle_speed_mps_ > 0.5) {
                in_vehicle_mode_ = true;
            } else if (is_walking_pattern_ && accel_variance_est_ > 0.30f) {
                // If it really looks like walking (high rhythmic energy), stop driving.
                in_vehicle_mode_ = false;
                vehicle_speed_mps_ = 0.0;
            }
        }
    }
    last_accel_ts_ns_ = timestamp_ns;
}

// DEAD CODE: getMotionMode — never called externally
// IMUPreintegrator::MotionMode IMUPreintegrator::getMotionMode() const {
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (in_vehicle_mode_) return MotionMode::DRIVING;
//     if (is_walking_pattern_) return MotionMode::WALKING;
//     return MotionMode::STATIONARY;
// }

double IMUPreintegrator::getVehicleSpeedEstimate() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return in_vehicle_mode_ ? vehicle_speed_mps_ : 0.0;
}

// ── Step detection ────────────────────────────────────────────────────────────

void IMUPreintegrator::detectStep(int64_t timestamp_ns, float ax, float ay, float az) {
    // Must be called under mutex_ lock
    float mag = std::sqrt(ax * ax + ay * ay + az * az);

    // Low-pass filter
    constexpr float LP_ALPHA = 0.20f;
    accel_mag_filtered_ = (1.0f - LP_ALPHA) * accel_mag_filtered_ + LP_ALPHA * mag;

    // Reject steps if accel pattern doesn't match walking (filters car vibrations)
    if (!is_walking_pattern_) return;

    // Peak detection with hysteresis
    if (!was_above_thresh_ && accel_mag_filtered_ > STEP_ACCEL_THRESH_HIGH) {
        was_above_thresh_ = true;

        // Check timing constraint
        if (last_step_ns_ > 0) {
            double period = (timestamp_ns - last_step_ns_) * 1e-9;
            if (period >= MIN_STEP_PERIOD_S && period <= MAX_STEP_PERIOD_S) {
                step_count_++;
                step_period_s_ = period;
                last_step_ns_ = timestamp_ns;
            } else if (period > MAX_STEP_PERIOD_S) {
                step_count_++;
                step_period_s_ = 0.0;
                last_step_ns_ = timestamp_ns;
            }
        } else {
            step_count_++;
            last_step_ns_ = timestamp_ns;
        }
    } else if (was_above_thresh_ && accel_mag_filtered_ < STEP_ACCEL_THRESH_LOW) {
        was_above_thresh_ = false;
    }

    accel_mag_prev_ = accel_mag_filtered_;
}

IMUPreintegrator::StepInfo IMUPreintegrator::getStepInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    StepInfo info{};
    info.step_count = step_count_;
    info.stride_length_m = DEFAULT_STRIDE_M;
    info.last_step_ns = last_step_ns_;

    // ── SMART STOP DETECTION ────────────────────────────────────────────────
    // We detect a stop by checking if the recent acceleration variance is low.
    // walking has variance > 0.15. If variance < 0.08, we are almost certainly
    // stationary or moving very smoothly.
    bool is_stationary = (accel_variance_est_ < 0.08f);

    // Also keep a watchdog: if no step for 3 seconds, force stop (safety)
    int64_t current_time_ns = last_accel_ts_ns_;
    double ns_since_last = static_cast<double>(current_time_ns - last_step_ns_);
    bool watchdog_stop = (last_step_ns_ > 0 && ns_since_last > 3'000'000'000.0);

    // Estimate speed from step frequency + user height
    if (step_period_s_ > 0.0 && !is_stationary && !watchdog_stop) {
        double freq = 1.0 / step_period_s_;
        // Height-based stride: stride ≈ height * 0.415 at normal walk,
        // scales with step frequency (faster steps = longer strides)
        double base_stride = static_cast<double>(user_height_m_) * 0.415;
        double freq_factor = 0.7 + 0.3 * std::min(2.5, freq);  // 0.7-1.45x
        info.stride_length_m = std::max(0.3, std::min(1.5, base_stride * freq_factor));
        info.speed_mps = info.stride_length_m * freq;
    } else {
        info.speed_mps = 0.0;
    }

    return info;
}

// ── User height for stride model ──────────────────────────────────────────────

void IMUPreintegrator::setUserHeight(float height_m) {
    std::lock_guard<std::mutex> lock(mutex_);
    user_height_m_ = std::max(1.0f, std::min(2.5f, height_m));
    LOGI("User height set: %.2f m → base stride ≈ %.2f m", user_height_m_, user_height_m_ * 0.415f);
}

float IMUPreintegrator::getUserHeight() const { return user_height_m_; }

// ── reset ─────────────────────────────────────────────────────────────────────

void IMUPreintegrator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    gyro_buf_.clear();
    accel_buf_.clear();
    gravity_initialized_.store(false);
    gravity_vec_ = cv::Point3f(0.f, 0.f, 9.81f);
    filtered_gravity_ = cv::Point3f(0.f, 0.f, 9.81f);
    filtered_gravity_init_ = false;
    gravity_init_samples_.clear();
    roll_  = 0.f;
    pitch_ = 0.f;
    last_ax = last_ay = last_az = 0.f;
    last_gx = last_gy = last_gz = 0.f;
    step_count_ = 0;
    last_step_ns_ = 0;
    step_period_s_ = 0.0;
    accel_mag_filtered_ = 9.81f;
    accel_mag_prev_ = 9.81f;
    was_above_thresh_ = false;
    accel_mag_slow_ = 9.81f;
    accel_variance_est_ = 0.0f;
    is_walking_pattern_ = false;
    vehicle_speed_mps_ = 0.0;
    last_accel_ts_ns_ = 0;
    sustained_accel_s_ = 0.0;
    in_vehicle_mode_ = false;
    gyro_bias_ = cv::Point3f(0.f, 0.f, 0.f);
    b_a_ = cv::Point3f(0.f, 0.f, 0.f);
    gyro_bias_initialized_.store(false);
    gyro_bias_samples_ = 0;
    has_mag_heading_.store(false);  // Reset mag heading
    mag_heading_ = 0.f;
}

// ── Gyroscope bias initialization ─────────────────────────────────────────────
// 
// Gyroscope sensors have a constant offset (bias) that doesn't average out.
// During stationary initialization, we estimate this bias from the raw gyro readings
// and then subtract it during integration to prevent unbounded heading drift.
//
// This fixes the problem where heading drifts by ~270° over 22 seconds due to 
// an unestimated ~0.2°/sec bias accumulating.

void IMUPreintegrator::tryInitializeGyroBiasLocked(float ax, float ay, float az) {
    if (gyro_bias_initialized_.load()) {
        return;  // Already initialized, don't update
    }

    // Only initialize gyro bias when device is stationary (low acceleration variance)
    // This check is approximate since we're called inline with accel reading
    if (accel_variance_est_ > 0.05f) {
        return;  // Still moving, can't estimate bias yet
    }

    // Accumulate samples while stationary
    if (gyro_bias_samples_ < GYRO_BIAS_INIT_SAMPLES) {
        gyro_bias_samples_++;
        return;
    }

    // Collect last N stationary gyro samples
    double avg_gx = 0.0, avg_gy = 0.0, avg_gz = 0.0;
    int count = 0;

    // Get most recent GYRO_BIAS_INIT_SAMPLES from gyro buffer
    if (gyro_buf_.size() >= static_cast<size_t>(GYRO_BIAS_INIT_SAMPLES)) {
        auto it = gyro_buf_.end() - GYRO_BIAS_INIT_SAMPLES;
        for (; it != gyro_buf_.end(); ++it) {
            avg_gx += it->x;
            avg_gy += it->y;
            avg_gz += it->z;
            count++;
        }
    }

    if (count > 0) {
        gyro_bias_.x = avg_gx / count;
        gyro_bias_.y = avg_gy / count;
        gyro_bias_.z = avg_gz / count;
        gyro_bias_initialized_.store(true);

        LOGI("Gyro bias initialized from %d stationary samples: (%.5f, %.5f, %.5f) rad/sec",
             count, gyro_bias_.x, gyro_bias_.y, gyro_bias_.z);
    }
}

// ── Magnetometer heading fusion ───────────────────────────────────────────────
//
// Magnetometer provides an absolute heading reference (compass reading) that doesn't
// accumulate error over time. By gently pulling the gyro-integrated heading toward
// the magnetometer reading, we prevent unbounded heading drift while preserving
// short-term rotating motion accuracy.
//
// Blending: corrected_heading = gyro_heading + (mag_heading - gyro_heading) * (1 - damping)
//           With damping = 0.95: 95% gyro, 5% magnetometer
//           This prevents magnetic disturbances from causing jumps, but still bounds drift.

void IMUPreintegrator::setMagnetometerHeading(float yaw_rad) {
    std::lock_guard<std::mutex> lock(mutex_);
    mag_heading_ = yaw_rad;
    has_mag_heading_.store(true);
    last_mag_update_ns_ = last_accel_ts_ns_;
}

// DEAD CODE: getCorrectedHeading — never called (mag heading used once at startup via hasMagHeading/getMagHeading)
// float IMUPreintegrator::getCorrectedHeading(float gyro_yaw_rad) { ... }

cv::Point3f IMUPreintegrator::getGyroBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gyro_bias_;
}

void IMUPreintegrator::refineGyroBiasDuringZUPT() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gyro_bias_initialized_.load()) return;
    if (gyro_buf_.size() < 10) return;

    // Average recent gyro readings — when truly stationary, this IS the bias
    double avg_gx = 0.0, avg_gy = 0.0, avg_gz = 0.0;
    int count = 0;
    int n = std::min(static_cast<int>(gyro_buf_.size()), 20);
    auto it = gyro_buf_.end() - n;
    for (; it != gyro_buf_.end(); ++it) {
        avg_gx += it->x;
        avg_gy += it->y;
        avg_gz += it->z;
        count++;
    }
    if (count == 0) return;
    avg_gx /= count;
    avg_gy /= count;
    avg_gz /= count;

    // Gentle update (alpha=0.01) — prevents overcorrection from brief pauses
    constexpr float alpha = 0.01f;
    gyro_bias_.x = (1.0f - alpha) * gyro_bias_.x + alpha * static_cast<float>(avg_gx);
    gyro_bias_.y = (1.0f - alpha) * gyro_bias_.y + alpha * static_cast<float>(avg_gy);
    gyro_bias_.z = (1.0f - alpha) * gyro_bias_.z + alpha * static_cast<float>(avg_gz);
}
