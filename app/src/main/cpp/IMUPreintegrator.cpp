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
    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        double dt = static_cast<double>(samples[i + 1].timestamp_ns - samples[i].timestamp_ns) * 1e-9;
        if (dt <= 0.0) continue;

        cv::Mat rvec = (cv::Mat_<double>(3, 1)
                        << static_cast<double>(samples[i].x) * dt,
                           static_cast<double>(samples[i].y) * dt,
                           static_cast<double>(samples[i].z) * dt);
        cv::Mat R_delta;
        cv::Rodrigues(rvec, R_delta);
        result = R_delta * result;
    }

    // Handle single-sample case: integrate from sample to end_ns
    if (samples.size() == 1) {
        double dt = static_cast<double>(end_ns - samples[0].timestamp_ns) * 1e-9;
        if (dt > 0.0) {
            cv::Mat rvec = (cv::Mat_<double>(3, 1)
                            << static_cast<double>(samples[0].x) * dt,
                               static_cast<double>(samples[0].y) * dt,
                               static_cast<double>(samples[0].z) * dt);
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
        
        // Get sensors at current (or nearest)
        cv::Point3d g_vec(0,0,0), a_vec(0,0,0);
        if (gi > 0) g_vec = cv::Point3d(g_samples[gi-1].x, g_samples[gi-1].y, g_samples[gi-1].z);
        if (ai > 0) a_vec = cv::Point3d(a_samples[ai-1].x, a_samples[ai-1].y, a_samples[ai-1].z);

        // 1. Rotation update
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
    // Must be called under mutex_ lock
    float mag = std::sqrt(ax * ax + ay * ay + az * az);

    // Very slow LP filter to get baseline accel magnitude
    constexpr float SLOW_ALPHA = 0.02f;
    accel_mag_slow_ = (1.0f - SLOW_ALPHA) * accel_mag_slow_ + SLOW_ALPHA * mag;

    // Running variance estimate (exponential moving variance)
    float diff = mag - accel_mag_slow_;
    accel_variance_est_ = (1.0f - SLOW_ALPHA) * accel_variance_est_ + SLOW_ALPHA * (diff * diff);

    // Walking has large periodic variance (>0.15), car vibrations are smaller (<0.10)
    is_walking_pattern_ = (accel_variance_est_ > 0.15f);

    // Vehicle speed estimation via forward accel integration
    if (last_accel_ts_ns_ > 0) {
        double dt = (timestamp_ns - last_accel_ts_ns_) * 1e-9;
        if (dt > 0.0 && dt < 0.1) {
            // Remove gravity component (approximate: subtract magnitude baseline)
            float forward_accel = mag - accel_mag_slow_;

            // Only integrate significant acceleration (> 0.3 m/s² after gravity removal)
            if (std::abs(forward_accel) > 0.3f) {
                vehicle_speed_mps_ += forward_accel * dt;
                sustained_accel_s_ += dt;
            } else {
                // Decay speed when not accelerating (friction model)
                vehicle_speed_mps_ *= 0.995;
                sustained_accel_s_ = 0.0;
            }

            // Clamp to plausible range
            vehicle_speed_mps_ = std::max(0.0, std::min(50.0, vehicle_speed_mps_));

            // Detect vehicle mode: sustained non-walking motion
            if (!is_walking_pattern_ && vehicle_speed_mps_ > 1.0) {
                in_vehicle_mode_ = true;
            } else if (is_walking_pattern_) {
                in_vehicle_mode_ = false;
                vehicle_speed_mps_ = 0.0; // Reset when walking detected
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

    // ── STALE STEP CHECK (Fix for 'Ghost Walking') ──────────────────────────
    // If it's been more than 2 seconds since the last step, assume we stopped.
    // We use the last known accel timestamp to determine current 'VIO time'.
    int64_t current_time_ns = last_accel_ts_ns_;
    double ns_since_last = static_cast<double>(current_time_ns - last_step_ns_);
    bool is_stale = (last_step_ns_ > 0 && ns_since_last > 2'000'000'000.0);

    // Estimate speed from step frequency
    if (step_period_s_ > 0.0 && !is_stale) {
        // Stride length scales with step frequency: faster steps = longer strides
        // Refined model: 0.4m base + 0.3 * freq to avoid underestimating slow walkers
        double freq = 1.0 / step_period_s_;
        info.stride_length_m = std::max(0.4, std::min(1.2, 0.4 + 0.3 * freq));
        info.speed_mps = info.stride_length_m * freq;
    } else {
        info.speed_mps = 0.0;
    }

    return info;
}

// ── reset ─────────────────────────────────────────────────────────────────────

void IMUPreintegrator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    gyro_buf_.clear();
    accel_buf_.clear();
    gravity_initialized_.store(false);
    gravity_vec_ = cv::Point3f(0.f, 0.f, 9.81f);
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
}
