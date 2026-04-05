#include "EKFState.h"
#include <cmath>
#include <algorithm>
#include <android/log.h>

#define TAG "NavSight-EKF"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

EKFState::EKFState() { reset(); }

void EKFState::reset() {
    scale_ = 0.20;
    P_ = 0.5 * 0.5;  // high initial uncertainty
    initialized_ = false;
    update_count_ = 0;
}

void EKFState::initialize(double initial_scale) {
    scale_ = initial_scale;
    P_ = 0.5 * 0.5;  // still uncertain until step observations arrive
    initialized_ = true;
    LOGI("EKF initialized: scale=%.3f", initial_scale);
}

// ── Scale update ────────────────────────────────────────────────────────────
// z = observed_scale, H = [1], simple scalar Kalman update

void EKFState::updateScale(double observed_scale, double confidence) {
    if (observed_scale <= 0.005 || confidence <= 0.0) return;

    // Add process noise (random walk between updates)
    P_ += SIGMA_SCALE_RW * SIGMA_SCALE_RW;

    // Measurement noise inversely proportional to confidence
    double R = SIGMA_SCALE_MEAS / (confidence + 0.01);
    R = std::max(R, 0.001);

    double innov = observed_scale - scale_;

    // Mahalanobis gate: reject outliers beyond 3-sigma
    double S = P_ + R;
    double maha = (innov * innov) / S;
    if (maha > 9.0) return;

    // Kalman gain
    double K = P_ / S;

    // State update
    scale_ += K * innov;
    scale_ = std::max(0.005, std::min(20.0, scale_));

    // Covariance update: P = (1 - K) * P
    P_ = (1.0 - K) * P_;

    update_count_++;
}

// ── ZUPT: stationary → tighten scale uncertainty ────────────────────────────

void EKFState::updateZUPT() {
    // When stationary, scale should not drift. Tighten variance.
    P_ *= 0.99;
}

// ── Cross-validation ────────────────────────────────────────────────────────

double EKFState::checkConsistency(double camera_disp, double step_disp) const {
    if (camera_disp < 0.001 && step_disp < 0.001) return 1.0;
    double max_d = std::max(camera_disp, step_disp);
    if (max_d < 0.001) return 1.0;
    double diff = std::abs(camera_disp - step_disp);
    double ratio = 1.0 - std::min(1.0, diff / max_d);
    return ratio * ratio;
}

double EKFState::getScaleStd() const {
    return std::sqrt(std::max(0.0, P_));
}
