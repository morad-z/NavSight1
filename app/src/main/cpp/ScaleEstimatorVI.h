#pragma once
#include <vector>
#include <mutex>
#include <opencv2/core.hpp>

// Step 3 Observer C — Hesch/Martinelli closed-form visual-inertial scale.
//
// Stacks N consecutive keyframe pairs and solves for unknowns x = [s, v0]ᵀ
// (scalar metric scale + initial body velocity in world frame) using ordinary
// least squares.
//
// Per-pair equation (in world frame; g points down, e.g. (0,0,−9.81)):
//   s · R_w_b_i · t_vis_i = R_w_b_i · Δp_i + v_i · Δt_i + 0.5 · g · Δt_i²
// where v_i = v_0 + γ_i, and γ_i = Σ_{k<i} (R_w_b_k · Δv_k + g · Δt_k).
//
// Rearranged:  A · x = b   with   A_i = [R_w_b_i · t_vis_i  |  −Δt_i · I_3]
//                                  b_i = R_w_b_i · Δp_i + γ_i · Δt_i + 0.5 · g · Δt_i²
//
// Solved via the normal equations  x = (AᵀA)⁻¹ Aᵀ b.
// Variance of the scale s = (AᵀA)⁻¹_{0,0} · σ²_resid.
//
// Why this isn't enabled in current production: it was disabled in Phase 8
// because IMU preintegration ΔP was dominated by gravity-subtraction error.
// That assumption is only valid when attitude is wrong; with Madgwick (Step 1)
// the gravity term is clean and this path becomes well-conditioned.
class ScaleEstimatorVI {
public:
    ScaleEstimatorVI();

    struct KeyframePair {
        // Rotation world←body at keyframe i (3×3, CV_64F).
        cv::Mat R_w_b;
        // Visual translation between keyframe i and i+1, expressed in body
        // frame at i. Magnitude is in VIO units (scale-ambiguous): metric
        // displacement = s · R_w_b · t_vis.
        cv::Vec3d t_vis_body;
        // IMU-preintegrated position delta (body frame at i) over [t_i, t_{i+1}].
        cv::Vec3d delta_p_body;
        // IMU-preintegrated velocity delta (body frame at i) over [t_i, t_{i+1}].
        cv::Vec3d delta_v_body;
        // Time gap Δt_i in seconds.
        double dt;
    };

    // Append one keyframe pair. Older pairs beyond capacity are dropped.
    void addKeyframePair(const KeyframePair& pair);

    // Attempt the closed-form solve. Returns true on success.
    //   scale_out  — recovered metric scale (s).
    //   variance_out — variance of the scale estimate.
    //   v0_out (optional) — recovered initial velocity in world frame.
    bool solve(double& scale_out, double& variance_out,
               cv::Vec3d* v0_out = nullptr) const;

    // Number of buffered pairs currently held.
    size_t size() const;

    // Replace the gravity vector. Default is (0, 0, −9.81). Caller can refine.
    void setGravity(const cv::Vec3d& g);
    cv::Vec3d getGravity() const;

    // Drop all buffered pairs.
    void reset();

    // Minimum pairs required for a solve. 2 pairs ⇒ 6 equations, 4 unknowns —
    // technically solvable, but underdetermined enough to be brittle. Default
    // 4 → 12 equations vs 4 unknowns ≈ healthy overdetermination.
    static constexpr size_t MIN_PAIRS = 4;
    // Capacity for the rolling pair buffer.
    static constexpr size_t MAX_PAIRS = 16;

private:
    mutable std::mutex mutex_;
    std::vector<KeyframePair> pairs_;
    cv::Vec3d gravity_w_{0.0, 0.0, -9.81};
};
