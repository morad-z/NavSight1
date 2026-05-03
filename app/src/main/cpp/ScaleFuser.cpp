#include "ScaleFuser.h"

#include <algorithm>
#include <cmath>

ScaleFuser::ScaleFuser(double initial_scale, double initial_variance)
    : s_(std::clamp(initial_scale, SCALE_MIN, SCALE_MAX)),
      P_(std::max(0.0, initial_variance)) {}

void ScaleFuser::predict(double dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    P_ += PROCESS_NOISE_PER_SEC * dt_seconds;
}

bool ScaleFuser::update(double z, double r) {
    if (!std::isfinite(z) || !std::isfinite(r) || r <= 0.0) return false;
    if (z < SCALE_MIN || z > SCALE_MAX) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const double K = P_ / (P_ + r);
    s_ = s_ + K * (z - s_);
    P_ = (1.0 - K) * P_;
    s_ = std::clamp(s_, SCALE_MIN, SCALE_MAX);
    if (!std::isfinite(P_) || P_ < 0.0) P_ = 0.0;
    return true;
}

double ScaleFuser::scale() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return s_;
}

double ScaleFuser::variance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return P_;
}

void ScaleFuser::reset(double initial_scale, double initial_variance) {
    std::lock_guard<std::mutex> lock(mutex_);
    s_ = std::clamp(initial_scale, SCALE_MIN, SCALE_MAX);
    P_ = std::max(0.0, initial_variance);
}
