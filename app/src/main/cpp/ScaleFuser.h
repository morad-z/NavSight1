#pragma once
#include <mutex>

// Step 3 Fusion — 1-D Kalman filter on metric scale.
//
// State:    s   (scale)        ∈ [SCALE_MIN, SCALE_MAX]
// Cov:      P   (scale variance)
//
// Predict (between observations):
//   s ← s,  P ← P + q · Δt
// where q is a small process-noise rate that lets the filter slowly forget
// stale estimates (true scale shifts very slowly — ankle height, posture,
// scooter vs walking — so q is small).
//
// Update (per observer, with measurement variance r):
//   K = P / (P + r)
//   s ← s + K · (z − s)
//   P ← (1 − K) · P
//
// Observers feed (z, r):
//   • PDR  — z = stride_pdr / vo_dist, r from step-period variance
//   • MiDaS — z = current_scale · median_ratio, r from MAD/N
//   • Hesch/Martinelli VI — z = s, r = (AᵀA)⁻¹_{0,0} · σ²_resid
//
// Updates with very large r contribute almost nothing — dead observers fade
// out automatically, no cascading fallback required.
class ScaleFuser {
public:
    explicit ScaleFuser(double initial_scale = 0.20,
                        double initial_variance = 1.0);

    // Time update. Adds q · dt_seconds to variance. dt_seconds clamped ≥ 0.
    void predict(double dt_seconds);

    // Measurement update. Returns false (no-op) if z or r is non-finite,
    // r ≤ 0, or z is outside [SCALE_MIN, SCALE_MAX].
    bool update(double z, double r);

    double scale() const;
    double variance() const;

    // Reset to a fresh state. Use on tracker reset / re-init.
    void reset(double initial_scale = 0.20, double initial_variance = 1.0);

    // Hard clamp range — mirrors the production smooth_scale_ bound.
    static constexpr double SCALE_MIN = 0.01;
    static constexpr double SCALE_MAX = 10.0;
    // Process-noise rate: 1e-5 / s ⇒ variance grows by ~3.6e-2 per hour, so a
    // long-stationary observer slowly loses weight relative to fresh ones.
    // Tuned conservatively — true scale rarely drifts.
    static constexpr double PROCESS_NOISE_PER_SEC = 1e-5;

private:
    mutable std::mutex mutex_;
    double s_;
    double P_;
};
