#include "IMUPreintegrator.h"
#include <algorithm>
#include <cmath>
#include <android/log.h>
#include <opencv2/calib3d.hpp>

#define TAG "NavSight-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

// ── Constructor ───────────────────────────────────────────────────────────────

IMUPreintegrator::IMUPreintegrator() {
    gyro_buf_.reserve(MAX_BUF);
    accel_buf_.reserve(MAX_BUF);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static inline bool isValidSensorFloat(float v) {
    return !std::isnan(v) && !std::isinf(v);
}

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

// ── integrateGyro ─────────────────────────────────────────────────────────────

cv::Mat IMUPreintegrator::integrateGyro(int64_t start_ns, int64_t end_ns) {
    cv::Mat result = cv::Mat::eye(3, 3, CV_64F);

    if (start_ns >= end_ns) {
        LOGE("integrateGyro: start_ns (%lld) >= end_ns (%lld), returning identity",
             (long long)start_ns, (long long)end_ns);
        return result;
    }

    // Collect samples in the time window
    std::vector<GyroSample> samples;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& s : gyro_buf_) {
            if (s.timestamp_ns >= start_ns && s.timestamp_ns <= end_ns) {
                samples.push_back(s);
            }
        }
    }

    if (samples.empty()) {
        return result;
    }

    // Sort by timestamp to handle out-of-order arrivals
    std::sort(samples.begin(), samples.end(),
              [](const GyroSample& a, const GyroSample& b) {
                  return a.timestamp_ns < b.timestamp_ns;
              });

    // Integrate consecutive pairs via Rodrigues formula
    // ✅ CRITICAL FIX: Apply gyro bias correction before integration
    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        double dt = static_cast<double>(samples[i + 1].timestamp_ns - samples[i].timestamp_ns) * 1e-9;
        if (dt <= 0.0) continue;

        // Subtract estimated gyro bias before integration
        double gx = static_cast<double>(samples[i].x) - static_cast<double>(gyro_bias_.x);
        double gy = static_cast<double>(samples[i].y) - static_cast<double>(gyro_bias_.y);
        double gz = static_cast<double>(samples[i].z) - static_cast<double>(gyro_bias_.z);

        cv::Mat rvec = (cv::Mat_<double>(3, 1)
                        << gx * dt,
                           gy * dt,
                           gz * dt);
        cv::Mat R_delta;
        cv::Rodrigues(rvec, R_delta);
        result = R_delta * result;
    }

    // Handle single-sample case: integrate from sample to end_ns
    if (samples.size() == 1) {
        double dt = static_cast<double>(end_ns - samples[0].timestamp_ns) * 1e-9;
        if (dt > 0.0) {
            // ✅ CRITICAL FIX: Apply gyro bias correction here too
            double gx = static_cast<double>(samples[0].x) - static_cast<double>(gyro_bias_.x);
            double gy = static_cast<double>(samples[0].y) - static_cast<double>(gyro_bias_.y);
            double gz = static_cast<double>(samples[0].z) - static_cast<double>(gyro_bias_.z);

            cv::Mat rvec = (cv::Mat_<double>(3, 1)
                            << gx * dt,
                               gy * dt,
                               gz * dt);
            cv::Mat R_delta;
            cv::Rodrigues(rvec, R_delta);
            result = R_delta * result;
        }
    }

    return result;
}

// ── integrate ─────────────────────────────────────────────────────────────────

PreintegratedDelta IMUPreintegrator::integrate(int64_t start_ns, int64_t end_ns) {
    PreintegratedDelta out;
    out.deltaR = cv::Mat::eye(3, 3, CV_64F);
    out.deltaV = cv::Mat::zeros(3, 1, CV_64F);
    out.deltaP = cv::Mat::zeros(3, 1, CV_64F);
    out.dt = 0.0;
    out.sample_count = 0;

    if (start_ns >= end_ns) {
        LOGE("integrate: invalid time window");
        return out;
    }

    std::vector<GyroSample> g_samples;
    std::vector<AccelSample> a_samples;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        g_samples.reserve(gyro_buf_.size()); // Conservative estimate
        a_samples.reserve(accel_buf_.size());
        for (const auto& s : gyro_buf_) {
            if (s.timestamp_ns >= start_ns && s.timestamp_ns <= end_ns) g_samples.push_back(s);
        }
        for (const auto& s : accel_buf_) {
            if (s.timestamp_ns >= start_ns && s.timestamp_ns <= end_ns) a_samples.push_back(s);
        }
    }

    if (g_samples.empty() || a_samples.empty()) {
        out.deltaR = integrateGyro(start_ns, end_ns);
        out.dt = (end_ns - start_ns) * 1e-9;
        out.sample_count = (int)g_samples.size();
        return out;
    }

    // Sort both
    auto sort_fn = [](const auto& a, const auto& b) { return a.timestamp_ns < b.timestamp_ns; };
    std::sort(g_samples.begin(), g_samples.end(), sort_fn);
    std::sort(a_samples.begin(), a_samples.end(), sort_fn);

    cv::Mat current_R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat current_V = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat current_P = cv::Mat::zeros(3, 1, CV_64F);

    // Mid-point integration
    size_t gi = 0, ai = 0;
    int64_t t_current = start_ns;
    
    cv::Mat gravity = (cv::Mat_<double>(3, 1) << (double)gravity_vec_.x, (double)gravity_vec_.y, (double)gravity_vec_.z);

    while (t_current < end_ns) {
        int64_t t_next = end_ns;
        if (gi < g_samples.size()) t_next = std::min(t_next, g_samples[gi].timestamp_ns);
        if (ai < a_samples.size()) t_next = std::min(t_next, a_samples[ai].timestamp_ns);
        
        if (t_next <= t_current) {
            if (gi < g_samples.size() && g_samples[gi].timestamp_ns <= t_current) gi++;
            if (ai < a_samples.size() && a_samples[ai].timestamp_ns <= t_current) ai++;
            continue;
        }

        double dt = (t_next - t_current) * 1e-9;
        
        // Get sensors at current (or nearest), with gyro bias correction
        cv::Point3d g_vec(0,0,0), a_vec(0,0,0);
        if (gi > 0) {
            g_vec = cv::Point3d(
                g_samples[gi-1].x - static_cast<double>(gyro_bias_.x),
                g_samples[gi-1].y - static_cast<double>(gyro_bias_.y),
                g_samples[gi-1].z - static_cast<double>(gyro_bias_.z));
        }
        if (ai > 0) a_vec = cv::Point3d(a_samples[ai-1].x, a_samples[ai-1].y, a_samples[ai-1].z);

        // 1. Rotation update (bias-corrected)
        cv::Mat r_vec = (cv::Mat_<double>(3, 1) << g_vec.x * dt, g_vec.y * dt, g_vec.z * dt);
        cv::Mat dR;
        cv::Rodrigues(r_vec, dR);
        
        // 2. Position and velocity update (Delta P = V*dt + 0.5*a*dt^2)
        // a_global = R * a_local - gravity
        cv::Mat a_local = (cv::Mat_<double>(3, 1) << a_vec.x, a_vec.y, a_vec.z);
        cv::Mat a_global = current_R * a_local - gravity;
        
        current_P += current_V * dt + 0.5 * a_global * dt * dt;
        current_V += a_global * dt;
        current_R = current_R * dR;

        t_current = t_next;
    }

    out.deltaR = current_R;
    out.deltaV = current_V;
    out.deltaP = current_P;
    out.dt = (end_ns - start_ns) * 1e-9;
    return out;
}

// ── initializeFromGravity ─────────────────────────────────────────────────────

void IMUPreintegrator::initializeFromGravity(const std::vector<cv::Point3f>& accel_samples) {
    if (accel_samples.empty()) {
        LOGE("initializeFromGravity: empty sample set, skipping");
        return;
    }

    // Compute average acceleration
    float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
    for (const auto& s : accel_samples) {
        sum_x += s.x;
        sum_y += s.y;
        sum_z += s.z;
    }
    float n = static_cast<float>(accel_samples.size());
    float avg_x = sum_x / n;
    float avg_y = sum_y / n;
    float avg_z = sum_z / n;

    float mag = std::sqrt(avg_x * avg_x + avg_y * avg_y + avg_z * avg_z);
    if (mag <= 0.f) {
        LOGE("initializeFromGravity: zero magnitude, skipping");
        return;
    }

    // Only initialize once — use compare_exchange_strong to prevent concurrent double-init
    bool expected = false;
    if (!gravity_initialized_.compare_exchange_strong(expected, true)) {
        // Another call already completed initialization
        return;
    }

    // Normalize and scale to 9.81 m/s²
    float inv_mag = 9.81f / mag;
    // gravity_vec_ and roll_/pitch_ are written once here after CAS succeeds;
    // no additional lock needed since only one thread can win the CAS.
    gravity_vec_ = cv::Point3f(avg_x * inv_mag, avg_y * inv_mag, avg_z * inv_mag);

    float gx = gravity_vec_.x;
    float gy = gravity_vec_.y;
    float gz = gravity_vec_.z;
    roll_  = std::atan2(gy, gz);
    pitch_ = std::atan2(-gx, std::sqrt(gy * gy + gz * gz));

    LOGI("initializeFromGravity: gravity=(%.3f,%.3f,%.3f) roll=%.3f pitch=%.3f",
         gravity_vec_.x, gravity_vec_.y, gravity_vec_.z, roll_, pitch_);
}

// ── setGravity ────────────────────────────────────────────────────────────────

void IMUPreintegrator::setGravity(float gx, float gy, float gz) {
    float mag = std::sqrt(gx * gx + gy * gy + gz * gz);
    if (mag <= 0.f) {
        LOGE("setGravity: zero magnitude, ignoring");
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    gravity_vec_ = cv::Point3f(gx, gy, gz);
    roll_  = std::atan2(gy, gz);
    pitch_ = std::atan2(-gx, std::sqrt(gy * gy + gz * gz));
    gravity_initialized_.store(true);
    gravity_init_samples_.clear();
}

// ── accessors ─────────────────────────────────────────────────────────────────

std::vector<AccelSample> IMUPreintegrator::getAccelBuffer() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accel_buf_;
}

cv::Point3f IMUPreintegrator::getGravityVector() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gravity_vec_;
}

float IMUPreintegrator::getRoll() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return roll_;
}

float IMUPreintegrator::getPitch() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pitch_;
}

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

IMUPreintegrator::MotionMode IMUPreintegrator::getMotionMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (in_vehicle_mode_) return MotionMode::DRIVING;
    if (is_walking_pattern_) return MotionMode::WALKING;
    return MotionMode::STATIONARY;
}

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

float IMUPreintegrator::getUserHeight() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return user_height_m_;
}

// ── reset ─────────────────────────────────────────────────────────────────────

void IMUPreintegrator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    gyro_buf_.clear();
    accel_buf_.clear();
    gravity_initialized_.store(false);
    gravity_vec_ = cv::Point3f(0.f, 0.f, 9.81f);
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
    gyro_bias_ = cv::Point3f(0.f, 0.f, 0.f);  // Reset gyro bias
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

    // Get most recent GYRO_BIAS_INIT_SAMPLES from gyro buffer (they should be oldest)
    if (gyro_buf_.size() >= static_cast<size_t>(GYRO_BIAS_INIT_SAMPLES)) {
        auto it = gyro_buf_.begin();
        for (int i = 0; i < GYRO_BIAS_INIT_SAMPLES && it != gyro_buf_.end(); ++i, ++it) {
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
//           With damping = 0.80: 80% VIO, 20% magnetometer
//           This prevents magnetic disturbances from causing jumps, but still bounds drift.

void IMUPreintegrator::setMagnetometerHeading(float yaw_rad) {
    std::lock_guard<std::mutex> lock(mutex_);
    mag_heading_ = yaw_rad;
    has_mag_heading_.store(true);
    last_mag_update_ns_ = last_accel_ts_ns_;
}

float IMUPreintegrator::getCorrectedHeading(float gyro_yaw_rad) {
    if (!has_mag_heading_.load()) {
        return gyro_yaw_rad;  // No magnetometer data yet, use pure gyro
    }

    // Compute angular difference, handling wrap-around at ±π
    float diff = mag_heading_ - gyro_yaw_rad;
    
    // Normalize to [-π, π]
    while (diff > M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;

    // Apply damped correction: 95% gyro + 5% magnetometer
    // This creates a gentle "magnetic pull" that prevents drift while avoiding jumps
    float correction = diff * (1.0f - HEADING_CORRECTION_DAMPING);
    
    return gyro_yaw_rad + correction;
}
