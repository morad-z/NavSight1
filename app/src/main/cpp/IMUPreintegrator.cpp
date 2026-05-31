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
    // Low-pass filtered |gyro| magnitude — used to suppress phantom step
    // detection and walking classification while the user is rotating in place
    // or turning during a walk (otherwise arm swing during turns triggers
    // is_walking_pattern_ and phantom position advance).
    const float gmag = std::sqrt(x * x + y * y + z * z);
    constexpr float GYRO_LP_ALPHA = 0.15f;  // ~7 samples time constant at 200 Hz
    gyro_mag_filtered_ = (1.0f - GYRO_LP_ALPHA) * gyro_mag_filtered_ + GYRO_LP_ALPHA * gmag;
    if (gyro_buf_.size() >= MAX_BUF) {
        // Erase oldest half in one shot to amortise O(n) cost
        gyro_buf_.erase(gyro_buf_.begin(),
                        gyro_buf_.begin() + static_cast<ptrdiff_t>(MAX_BUF / 2));
    }
    gyro_buf_.push_back({timestamp_ns, x, y, z});

    // Madgwick attitude update — driven by every gyro sample so orientation
    // tracks at full IMU rate regardless of camera frame cadence.
    updateMadgwickLocked(timestamp_ns, x, y, z);
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
        
        // 2026-05-19 V-SHAPE ROOT CAUSE FIX.
        //
        // Cause:    The first iteration of this loop has gi == 0 and ai == 0,
        //           because the loop never pre-advances them past start_ns.
        //           With the original `if (gi > 0)` gate, w stayed at (0,0,0)
        //           for the leading sub-interval [start_ns → first_sample_ts].
        //           For a 500 Hz IMU this gap is up to ~2 ms; per camera frame
        //           at ω ≈ 0.25 rad/s walking rate the missed rotation is
        //           ~0.5 mrad = 0.029°. At 30 fps × 132 s = 3960 frames the
        //           accumulated under-rotation reaches ~115° — exactly the
        //           107° EKF-Madgwick gap observed in v50. Madgwick integrates
        //           at IMU rate with no such leading gap, so it tracked truth
        //           while EKF systematically fell behind.
        //
        // Change:   Initialise w (and a) from the FIRST sample in the window
        //           (zero-order forward hold) instead of zero. The leading gap
        //           gets the same gyro/accel as the sample that immediately
        //           follows it — better than zero, and the standard
        //           extrapolation when no prior sample is available.
        //
        // Falsifier: post-fix, the HEADING log M-EKF_gap should remain in
        //           single-digit ° throughout a two-loop walk (was growing to
        //           ~100°). Cumulative EKF rotation should match Madgwick's
        //           within a few ° on a 2-loop walk (was off by 107°).
        cv::Mat w = (cv::Mat_<double>(3,1) << 0,0,0);
        cv::Mat a = (cv::Mat_<double>(3,1) << 0,0,0);
        if (gi > 0) {
            w = (cv::Mat_<double>(3,1) << g_samples[gi-1].x - gyro_bias_.x,
                                          g_samples[gi-1].y - gyro_bias_.y,
                                          g_samples[gi-1].z - gyro_bias_.z);
        } else if (!g_samples.empty()) {
            // Leading gap: zero-order forward hold from the first sample.
            w = (cv::Mat_<double>(3,1) << g_samples[0].x - gyro_bias_.x,
                                          g_samples[0].y - gyro_bias_.y,
                                          g_samples[0].z - gyro_bias_.z);
        }
        if (ai > 0) {
            a = (cv::Mat_<double>(3,1) << a_samples[ai-1].x - b_a_.x,
                                          a_samples[ai-1].y - b_a_.y,
                                          a_samples[ai-1].z - b_a_.z);
        } else if (!a_samples.empty()) {
            // Same forward-hold fix for accel (was also returning zero for
            // the leading gap → under-integrated velocity / position).
            a = (cv::Mat_<double>(3,1) << a_samples[0].x - b_a_.x,
                                          a_samples[0].y - b_a_.y,
                                          a_samples[0].z - b_a_.z);
        }

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
    // BUG FIX (V-shape / rotate-in-place): also drop out of walking when the
    // user is rotating fast (>~46°/s). Arm swing and centripetal accel during
    // turns produce plenty of accel variance, so without this gate the pattern
    // latches true and phantom steps keep firing through the turn.
    const bool rotating_fast = (gyro_mag_filtered_ > 0.8f);
    if (!is_walking_pattern_ && accel_variance_est_ > 0.20f && !rotating_fast) {
        is_walking_pattern_ = true;
    } else if (is_walking_pattern_ &&
               (accel_variance_est_ < 0.05f || rotating_fast)) {
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

    // Reject steps during fast rotation. Turning in place (or mid-walk turn)
    // shakes the phone enough to trip peak detection; without this gate the
    // step counter keeps ticking through a turn and Tracker's PDR fallback
    // advances global position along the rotating heading → V-shape / arc.
    if (gyro_mag_filtered_ > ROTATION_STEP_GATE_RADPS) return;

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
                // Step 3 Observer A: track recent periods for variance →
                // PDR confidence. Big variance = irregular gait = low conf.
                step_period_buf_.push_back(period);
                if (step_period_buf_.size() > STEP_PERIOD_BUF) {
                    step_period_buf_.erase(step_period_buf_.begin());
                }
            } else if (period > MAX_STEP_PERIOD_S) {
                step_count_++;
                step_period_s_ = 0.0;
                last_step_ns_ = timestamp_ns;
                // Long gap → gait broke. Reset variance buffer so the next
                // few periods don't get penalized by the gap.
                step_period_buf_.clear();
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
    info.step_period_variance_s2 = -1.0;
    info.accel_variance = static_cast<double>(accel_variance_est_);
    info.time_since_last_step_s = -1.0;
    info.stride_calibrated = (user_stride_m_ > 0.0);
    // 2026-05-30 (Scale fix Step 0) — step cadence for gait classification.
    // step_period_s_ is the interval between the last two detected steps (s),
    // set in detectStep(); 0 when no recent step → report 0 Hz.
    // 2026-05-31 (FIX B Part 1) — also report 0 Hz when the last step is STALE (older
    // than MAX_STEP_PERIOD_S). step_period_s_ does NOT time-decay (detectStep only zeroes
    // it on a late detected step at :536), so a paused walker — or a walk→ride transition
    // — would otherwise carry a frozen ~2 Hz cadence forever. This staleness is what makes
    // classifyGait's VEHICLE veto safe: a genuine ride emits no steps for >MAX_STEP_PERIOD_S
    // → step_freq_hz=0 → still promotes to VEHICLE. Reuses MAX_STEP_PERIOD_S (no new const).
    const double tsls_freq = (last_step_ns_ > 0 && last_accel_ts_ns_ > 0)
        ? std::max(0.0, static_cast<double>(last_accel_ts_ns_ - last_step_ns_) * 1e-9)
        : 1e9;
    info.step_freq_hz = (step_period_s_ > 0.0 && tsls_freq <= MAX_STEP_PERIOD_S)
        ? (1.0 / step_period_s_) : 0.0;

    // Step 3 Observer A: time-since-last-step. Used by Tracker to fade PDR
    // confidence smoothly as a step ages, instead of a hard cutoff.
    if (last_step_ns_ > 0 && last_accel_ts_ns_ > 0) {
        double dt = static_cast<double>(last_accel_ts_ns_ - last_step_ns_) * 1e-9;
        info.time_since_last_step_s = std::max(0.0, dt);
    }

    // Step 3 Observer A: variance of recent step periods. Need at least 3
    // samples to make this meaningful.
    if (step_period_buf_.size() >= 3) {
        double mean = 0.0;
        for (double p : step_period_buf_) mean += p;
        mean /= static_cast<double>(step_period_buf_.size());
        double var = 0.0;
        for (double p : step_period_buf_) {
            double d = p - mean;
            var += d * d;
        }
        var /= static_cast<double>(step_period_buf_.size());
        info.step_period_variance_s2 = var;
    }

    // ── SMART STOP DETECTION ────────────────────────────────────────────────
    // We detect a stop by checking if the recent acceleration variance is low.
    // walking has variance > 0.15. If variance < 0.08, we are almost certainly
    // stationary or moving very smoothly.
    bool is_stationary = (accel_variance_est_ < 0.08f);

    // Also keep a watchdog: if no step for 3 seconds, force stop (safety)
    int64_t current_time_ns = last_accel_ts_ns_;
    double ns_since_last = static_cast<double>(current_time_ns - last_step_ns_);
    bool watchdog_stop = (last_step_ns_ > 0 && ns_since_last > 3'000'000'000.0);

    // Estimate speed from step frequency + stride model
    if (step_period_s_ > 0.0 && !is_stationary && !watchdog_stop) {
        double freq = 1.0 / step_period_s_;
        double stride;
        if (user_stride_m_ > 0.0) {
            // Step 3 Observer A: per-user calibrated stride. We still allow
            // a small frequency-based modulation (people lengthen strides at
            // higher cadence) but anchor on the calibrated value.
            double freq_factor = 0.85 + 0.15 * std::min(2.5, freq);  // 0.85–1.225x
            stride = user_stride_m_ * freq_factor;
        } else {
            // Height-based fallback: stride ≈ height * 0.415 at normal walk,
            // scales with step frequency (faster steps = longer strides).
            double base_stride = static_cast<double>(user_height_m_) * 0.415;
            double freq_factor = 0.7 + 0.3 * std::min(2.5, freq);  // 0.7–1.45x
            stride = base_stride * freq_factor;
        }
        info.stride_length_m = std::max(0.3, std::min(1.5, stride));
        info.speed_mps = info.stride_length_m * freq;
    } else {
        info.speed_mps = 0.0;
    }

    return info;
}

// ── Step 3 Observer A: per-user stride calibration ────────────────────────────
void IMUPreintegrator::setUserStride(double stride_m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stride_m > 0.0) {
        // Sanity-bound to physically plausible stride lengths.
        user_stride_m_ = std::max(0.3, std::min(1.5, stride_m));
        LOGI("setUserStride: calibrated stride = %.3f m (clamped from %.3f)",
             user_stride_m_, stride_m);
    } else {
        user_stride_m_ = -1.0;  // Disable calibration → fall back to height model.
        LOGI("setUserStride: calibration cleared (using height-based fallback)");
    }
}

double IMUPreintegrator::getUserStride() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return user_stride_m_;
}

bool IMUPreintegrator::hasCalibratedStride() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return user_stride_m_ > 0.0;
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
    step_period_buf_.clear();
    // Note: user_stride_m_ deliberately preserved across reset() — it is a
    // persistent per-user calibration, not a session-local quantity.
    accel_mag_filtered_ = 9.81f;
    accel_mag_prev_ = 9.81f;
    was_above_thresh_ = false;
    accel_mag_slow_ = 9.81f;
    accel_variance_est_ = 0.0f;
    is_walking_pattern_ = false;
    gyro_mag_filtered_ = 0.0f;
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
    mag_initialized_once_ = false;  // re-snap the absolute heading on the next clean fix
    mag_fuse_count_ = 0;
    mag_reject_count_ = 0;
    // Madgwick state
    q0_ = 1.0; q1_ = 0.0; q2_ = 0.0; q3_ = 0.0;
    madgwick_last_ns_ = 0;
    madgwick_init_.store(false);
}

void IMUPreintegrator::resetOrientationFilter() {
    std::lock_guard<std::mutex> lock(mutex_);
    q0_ = 1.0; q1_ = 0.0; q2_ = 0.0; q3_ = 0.0;
    madgwick_last_ns_ = 0;
    madgwick_init_.store(false);
    mag_initialized_once_ = false;  // orientation re-init -> re-snap absolute heading from compass
}

// ── Madgwick IMU-only attitude filter ───────────────────────────────────────
//
// Reference: S. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays", 2010. IMU-only (no magnetometer) branch.
//
// State: unit quaternion q = (q0, q1, q2, q3) representing the rotation that
// takes a vector from the sensor body frame to the world frame. Hamilton
// convention. World frame is gravity-aligned: +Z up, +X arbitrary horizontal
// (locked at init), yaw accumulates freely from gyro and is corrected by
// accelerometer only through roll/pitch (accel cannot constrain yaw alone).
//
// Update rule per sample:
//   1. q̇_ω = 0.5 · q ⊗ (0, gx, gy, gz)             (gyro prediction)
//   2. If accel in valid window, compute gradient of gravity-alignment
//      objective ∇f and correct: q̇ = q̇_ω − β · ∇f/|∇f|
//   3. q ← normalize(q + q̇ · dt)
//
// β is the convergence gain — trades drift suppression vs noise bleed-through.
// Per plan, β = 0.033 for ~200 Hz IMU.

void IMUPreintegrator::tryInitMadgwickLocked() {
    if (madgwick_init_.load()) return;
    // Need at least one accel sample to orient the quaternion to gravity.
    if (!filtered_gravity_init_) return;
    const float ax = filtered_gravity_.x;
    const float ay = filtered_gravity_.y;
    const float az = filtered_gravity_.z;
    const float amag = std::sqrt(ax * ax + ay * ay + az * az);
    if (amag < 1e-3f) return;

    // Tait-Bryan roll/pitch from gravity in body frame.
    // World frame: gravity points along +Z_world at +9.81, which matches
    // the existing filtered_gravity_ convention (phone held flat reads
    // body +Z ≈ 9.81). Z-up world.
    const double roll  = std::atan2(ay, az);
    const double pitch = std::atan2(-static_cast<double>(ax),
                                    std::sqrt(static_cast<double>(ay) * ay +
                                              static_cast<double>(az) * az));
    // Yaw seed: convert the pending compass heading (CW-positive nav)
    // into Madgwick's internal math-CCW convention. Madgwick's getHeading
    // does yaw_nav = -yaw_math, so to get getHeading() = azimuth_nav we
    // need yaw_math = -azimuth_nav. If no setInitialMadgwickYaw call was
    // made (pending_madgwick_yaw_nav_ default = 0) yaw stays 0 = North.
    const double yaw   = -static_cast<double>(pending_madgwick_yaw_nav_);

    // ZYX Euler (yaw-pitch-roll) → quaternion (Hamilton).
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    q0_ = cr * cp * cy + sr * sp * sy;
    q1_ = sr * cp * cy - cr * sp * sy;
    q2_ = cr * sp * cy + sr * cp * sy;
    q3_ = cr * cp * sy - sr * sp * cy;
    madgwick_init_.store(true);
    LOGI("Madgwick: initialized roll=%.1f° pitch=%.1f° yaw_nav=%.1f° "
         "(q=[%.3f %.3f %.3f %.3f])",
         roll * 180.0 / M_PI, pitch * 180.0 / M_PI,
         pending_madgwick_yaw_nav_ * 180.0 / M_PI,
         q0_, q1_, q2_, q3_);
}

void IMUPreintegrator::setInitialMadgwickYaw(float azimuth_rad_nav) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_madgwick_yaw_nav_ = azimuth_rad_nav;
    // Force re-initialization so the stored yaw seed is used. tryInitMadgwickLocked
    // will fire as soon as filtered_gravity_ has settled (which it has well
    // before setInitialHeading is called from Kotlin, since VIO init is
    // gated on InertialInitializer being READY which already required the
    // accel window to settle). Calling tryInitMadgwickLocked here makes
    // the new yaw take effect immediately if accel is ready.
    madgwick_init_.store(false);
    tryInitMadgwickLocked();
    LOGI("setInitialMadgwickYaw: azimuth_nav=%.1f° (Madgwick %s)",
         azimuth_rad_nav * 180.0 / M_PI,
         madgwick_init_.load() ? "re-initialized" : "pending accel");
}

void IMUPreintegrator::nudgeMadgwickYawAroundWorldZ(double delta_yaw_nav_rad) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!madgwick_init_.load()) return;
    if (!std::isfinite(delta_yaw_nav_rad) || std::abs(delta_yaw_nav_rad) < 1e-6) return;

    // Madgwick stores body→world quaternion q. getHeading returns yaw_nav
    // (CW-positive North=0), while the internal q is Hamilton math convention
    // (CCW-positive). yaw_nav = -yaw_math, so to add +Δ to yaw_nav we apply
    // a world-frame Z rotation of α_math = -Δ. The world-frame rotation
    // pre-multiplies (left-multiplies) the body→world q:
    //   q_world_rot = (cos(α/2), 0, 0, sin(α/2))
    //               = (cos(Δ/2), 0, 0, -sin(Δ/2))   substituting α = -Δ
    //   q_new = q_world_rot ⊗ q_old   (Hamilton product)
    const double cz = std::cos(delta_yaw_nav_rad * 0.5);
    const double sz = std::sin(delta_yaw_nav_rad * 0.5);
    // a=cz, b=0, c=0, d=-sz  applied to q=(q0,q1,q2,q3) via Hamilton product
    const double q0n = cz * q0_ + sz * q3_;
    const double q1n = cz * q1_ + sz * q2_;
    const double q2n = cz * q2_ - sz * q1_;
    const double q3n = cz * q3_ - sz * q0_;
    q0_ = q0n; q1_ = q1n; q2_ = q2n; q3_ = q3n;
    // Renormalise (Hamilton product of unit quats stays unit modulo float
    // error, but normalise defensively).
    const double n = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    if (n > 1e-9) {
        const double inv = 1.0 / n;
        q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
    }
}

void IMUPreintegrator::updateMadgwickLocked(int64_t timestamp_ns,
                                            float gx_raw, float gy_raw, float gz_raw) {
    if (!madgwick_init_.load()) {
        tryInitMadgwickLocked();
        if (!madgwick_init_.load()) {
            madgwick_last_ns_ = timestamp_ns;
            return;
        }
        madgwick_last_ns_ = timestamp_ns;
        return;
    }

    // dt from consecutive gyro samples
    double dt = 0.0;
    if (madgwick_last_ns_ > 0) {
        dt = static_cast<double>(timestamp_ns - madgwick_last_ns_) * 1e-9;
    }
    madgwick_last_ns_ = timestamp_ns;
    // Guard against clock hiccups and the very first step
    if (dt <= 0.0 || dt > 0.1) return;

    // Bias-corrected gyro
    const double gx = static_cast<double>(gx_raw) - gyro_bias_.x;
    const double gy = static_cast<double>(gy_raw) - gyro_bias_.y;
    const double gz = static_cast<double>(gz_raw) - gyro_bias_.z;

    // Quaternion derivative from gyro
    // q̇ω = 0.5 · q ⊗ (0, gx, gy, gz)
    double qDot0 = 0.5 * (-q1_ * gx - q2_ * gy - q3_ * gz);
    double qDot1 = 0.5 * ( q0_ * gx + q2_ * gz - q3_ * gy);
    double qDot2 = 0.5 * ( q0_ * gy - q1_ * gz + q3_ * gx);
    double qDot3 = 0.5 * ( q0_ * gz + q1_ * gy - q2_ * gx);

    // Accelerometer correction — only if accel is a trustworthy gravity
    // sample. During fast rotation centripetal accel biases the reference,
    // during freefall gravity disappears, during shocks the magnitude spikes.
    const double ax_raw = last_ax;
    const double ay_raw = last_ay;
    const double az_raw = last_az;
    const double aNormSq = ax_raw * ax_raw + ay_raw * ay_raw + az_raw * az_raw;
    const double aNorm = std::sqrt(aNormSq);
    if (aNorm > MADGWICK_ACC_MIN && aNorm < MADGWICK_ACC_MAX) {
        // Normalise accel
        const double ax = ax_raw / aNorm;
        const double ay = ay_raw / aNorm;
        const double az = az_raw / aNorm;

        // Gradient descent correction step (standard Madgwick IMU-only):
        // objective f = R^T · ĝ − â, where ĝ=(0,0,1), â = normalized accel.
        // Closed-form ∇f computed below (from Madgwick 2010 eq. 25).
        const double _2q0 = 2.0 * q0_;
        const double _2q1 = 2.0 * q1_;
        const double _2q2 = 2.0 * q2_;
        const double _2q3 = 2.0 * q3_;
        const double _4q0 = 4.0 * q0_;
        const double _4q1 = 4.0 * q1_;
        const double _4q2 = 4.0 * q2_;
        const double _8q1 = 8.0 * q1_;
        const double _8q2 = 8.0 * q2_;
        const double q0q0 = q0_ * q0_;
        const double q1q1 = q1_ * q1_;
        const double q2q2 = q2_ * q2_;
        const double q3q3 = q3_ * q3_;

        double s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        double s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1_ - _2q0 * ay
                    - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        double s2 = 4.0 * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                    - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        double s3 = 4.0 * q1q1 * q3_ - _2q1 * ax + 4.0 * q2q2 * q3_ - _2q2 * ay;

        // Normalise gradient (avoid div-by-zero)
        const double sNorm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sNorm > 1e-9) {
            const double inv = 1.0 / sNorm;
            s0 *= inv; s1 *= inv; s2 *= inv; s3 *= inv;

            // Apply feedback
            qDot0 -= MADGWICK_BETA * s0;
            qDot1 -= MADGWICK_BETA * s1;
            qDot2 -= MADGWICK_BETA * s2;
            qDot3 -= MADGWICK_BETA * s3;
        }
    }

    // Integrate
    q0_ += qDot0 * dt;
    q1_ += qDot1 * dt;
    q2_ += qDot2 * dt;
    q3_ += qDot3 * dt;

    // Renormalise
    const double qNormSq = q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_;
    if (qNormSq > 1e-9) {
        const double invQ = 1.0 / std::sqrt(qNormSq);
        q0_ *= invQ; q1_ *= invQ; q2_ *= invQ; q3_ *= invQ;
    } else {
        // Catastrophic — reinit on next sample
        madgwick_init_.store(false);
    }

    // 2026-05-25 (rev 2) — GYRO-PRIMARY heading with a GATED compass correction.
    // Supersedes the k=1.0 snap. Data (rv2 walk, straight-and-back): the OS
    // rotation-vector compass read 22-65 deg on the RETURN legs where a U-turn
    // from 150 deg needs ~330 (its azimuth jumped ~160 deg mid-turn = magnetic
    // disturbance), while the gyro tracked the U-turns cleanly (158<->332). The
    // snap piped that disturbance straight into the heading -> the heading-
    // projected trajectory V-shaped. This is the standard AHRS / compass-app
    // design: the gyro+accel Madgwick integration OWNS the heading (immune to
    // magnetic disturbance, nails turns); the compass only (a) sets the absolute
    // heading ONCE at startup, and (b) gently corrects slow gyro yaw drift when
    // it AGREES with the gyro. A disagreement larger than kMagDisturbRejectRad is
    // rejected as a disturbed field and the gyro is trusted. Inline: mutex_ is
    // already held (called from addGyroReading) — must NOT call getHeading() /
    // nudgeMadgwickYawAroundWorldZ() (they re-lock -> deadlock). Same world-Z
    // quaternion nudge math as nudgeMadgwickYawAroundWorldZ.
    //
    /* LEGACY 2026-05-25 — superseded approaches, kept per no-deletions rule:
       (1) complementary k=dt/tau, tau=4s — gentle pull toward the compass; then
       (2) SNAP k=1.0 — "RV is the ONLY heading source" (user directive). Both
       made the compass authoritative, so magnetic disturbance on the return legs
       flowed into the heading -> V-shape. The snap was: const double k = 1.0; */
    constexpr double kMagFusionTauS = 5.0;          // complementary time constant on a clean field
    constexpr double kMagDisturbRejectRad = 0.611;  // 35 deg: reject compass past this gyro disagreement
    mag_actively_fusing_.store(false, std::memory_order_relaxed);
    if (madgwick_init_.load() && has_mag_heading_.load()) {
        const int64_t mag_age_ns = timestamp_ns - last_mag_update_ns_;
        constexpr int64_t kMagFreshnessNs = 300'000'000LL;  // 0.3s; Kotlin sends ~20Hz
        if (mag_age_ns >= 0 && mag_age_ns < kMagFreshnessNs) {
            // A fresh compass is the heading authority for this tick, so the
            // visual / loop-closure yaw nudges stay OFF (they gate on
            // isMagActivelyFusing()). The gyro still drives every tick the compass
            // is rejected below — that is what keeps the turns correct.
            mag_actively_fusing_.store(true, std::memory_order_relaxed);

            const double siny = 2.0 * (q0_ * q3_ + q1_ * q2_);
            const double cosy = 1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_);
            const double yaw_nav = -std::atan2(siny, cosy);  // matches getHeading()
            double err = static_cast<double>(mag_heading_) - yaw_nav;
            while (err >  M_PI) err -= 2.0 * M_PI;
            while (err < -M_PI) err += 2.0 * M_PI;

            double k;
            const char* mode;
            if (!mag_initialized_once_) {
                // Startup: snap once to set the absolute initial heading (init is
                // done holding steady, so the field is clean here).
                k = 1.0;
                mag_initialized_once_ = true;
                mode = "INIT";
            } else if (std::fabs(err) <= kMagDisturbRejectRad) {
                // Clean field: gentle gyro-primary complementary correction.
                k = dt / kMagFusionTauS;
                if (k > 1.0) k = 1.0;
                ++mag_fuse_count_;
                mode = "FUSE";
            } else {
                // Disturbed field: reject the compass, trust the gyro.
                k = 0.0;
                ++mag_reject_count_;
                mode = "REJECT";
            }

            if (k > 0.0) {
                const double dyaw = k * err;
                const double cz = std::cos(dyaw * 0.5);
                const double sz = std::sin(dyaw * 0.5);
                const double q0n = cz * q0_ + sz * q3_;
                const double q1n = cz * q1_ + sz * q2_;
                const double q2n = cz * q2_ - sz * q1_;
                const double q3n = cz * q3_ - sz * q0_;
                q0_ = q0n; q1_ = q1n; q2_ = q2n; q3_ = q3n;
                const double nn = std::sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
                if (nn > 1e-9) { const double iv = 1.0 / nn; q0_*=iv; q1_*=iv; q2_*=iv; q3_*=iv; }
            }

            if (timestamp_ns - last_mag_fuse_log_ns_ > 1'000'000'000LL) {
                last_mag_fuse_log_ns_ = timestamp_ns;
                LOGI("MAG_FUSE[%s]: mag=%.1fdeg gyro_yaw=%.1fdeg err=%.1fdeg k=%.4f fuse=%lld reject=%lld",
                     mode,
                     static_cast<double>(mag_heading_) * 180.0 / M_PI,
                     yaw_nav * 180.0 / M_PI, err * 180.0 / M_PI, k,
                     static_cast<long long>(mag_fuse_count_),
                     static_cast<long long>(mag_reject_count_));
            }
        }
    }
}

void IMUPreintegrator::getOrientationQuaternion(double& q0, double& q1,
                                                double& q2, double& q3) const {
    std::lock_guard<std::mutex> lock(mutex_);
    q0 = q0_; q1 = q1_; q2 = q2_; q3 = q3_;
}

cv::Mat IMUPreintegrator::getRotationGtoI() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!madgwick_init_.load()) return cv::Mat();
    // R_btw (body→world) from Hamilton quaternion (q0=w, q1=x, q2=y, q3=z).
    // Standard formula, Sola "Quaternion kinematics" eq. 116. R_GtoI is
    // world→body = R_btw^T.
    const double q0 = q0_, q1 = q1_, q2 = q2_, q3 = q3_;
    cv::Mat R_btw = (cv::Mat_<double>(3, 3) <<
        1.0 - 2.0*(q2*q2 + q3*q3),  2.0*(q1*q2 - q0*q3),       2.0*(q1*q3 + q0*q2),
        2.0*(q1*q2 + q0*q3),        1.0 - 2.0*(q1*q1 + q3*q3), 2.0*(q2*q3 - q0*q1),
        2.0*(q1*q3 - q0*q2),        2.0*(q2*q3 + q0*q1),       1.0 - 2.0*(q1*q1 + q2*q2));
    return R_btw.t();
}

float IMUPreintegrator::getHeading() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!madgwick_init_.load()) return 0.f;
    // Extract yaw (ZYX convention) from Hamilton quaternion.
    //   yaw_math = atan2(2(q0*q3 + q1*q2), 1 - 2(q2² + q3²))  (CCW-positive)
    // Navigation convention is CW-positive (North=0, East=+π/2), so negate.
    const double siny_cosp = 2.0 * (q0_ * q3_ + q1_ * q2_);
    const double cosy_cosp = 1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_);
    const double yaw_math  = std::atan2(siny_cosp, cosy_cosp);
    double yaw_nav = -yaw_math;
    while (yaw_nav >  M_PI) yaw_nav -= 2.0 * M_PI;
    while (yaw_nav < -M_PI) yaw_nav += 2.0 * M_PI;
    return static_cast<float>(yaw_nav);
}

float IMUPreintegrator::getMadgwickRoll() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!madgwick_init_.load()) return 0.f;
    const double sinr_cosp = 2.0 * (q0_ * q1_ + q2_ * q3_);
    const double cosr_cosp = 1.0 - 2.0 * (q1_ * q1_ + q2_ * q2_);
    return static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));
}

float IMUPreintegrator::getMadgwickPitch() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!madgwick_init_.load()) return 0.f;
    const double sinp = 2.0 * (q0_ * q2_ - q3_ * q1_);
    if (std::abs(sinp) >= 1.0) {
        return static_cast<float>(std::copysign(M_PI / 2.0, sinp));
    }
    return static_cast<float>(std::asin(sinp));
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

void IMUPreintegrator::setGyroBias(float bx, float by, float bz) {
    std::lock_guard<std::mutex> lock(mutex_);
    gyro_bias_ = cv::Point3f(bx, by, bz);
    gyro_bias_initialized_.store(true);
    gyro_bias_samples_ = GYRO_BIAS_INIT_SAMPLES;
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
