#include "ScaleEstimatorVI.h"

#include <opencv2/core.hpp>

ScaleEstimatorVI::ScaleEstimatorVI() {
    pairs_.reserve(MAX_PAIRS);
}

void ScaleEstimatorVI::addKeyframePair(const KeyframePair& pair) {
    std::lock_guard<std::mutex> lock(mutex_);
    pairs_.push_back(pair);
    if (pairs_.size() > MAX_PAIRS) {
        pairs_.erase(pairs_.begin(),
                     pairs_.begin() + (pairs_.size() - MAX_PAIRS));
    }
}

size_t ScaleEstimatorVI::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pairs_.size();
}

void ScaleEstimatorVI::setGravity(const cv::Vec3d& g) {
    std::lock_guard<std::mutex> lock(mutex_);
    gravity_w_ = g;
}

cv::Vec3d ScaleEstimatorVI::getGravity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gravity_w_;
}

void ScaleEstimatorVI::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    pairs_.clear();
}

bool ScaleEstimatorVI::solve(double& scale_out, double& variance_out,
                             cv::Vec3d* v0_out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t N = pairs_.size();
    if (N < MIN_PAIRS) return false;

    // γ_i = Σ_{k<i} (R_w_b_k · Δv_k + g · Δt_k). γ_0 = 0.
    // Velocity update: v_{i+1} = v_i + R·ΔV_body + g·dt (g points down, e.g.
    // (0,0,-9.81)). γ_i ≡ v_i − v_0 = Σ_{k<i} (R·ΔV + g·dt).
    std::vector<cv::Vec3d> gamma(N, cv::Vec3d(0.0, 0.0, 0.0));
    for (size_t i = 1; i < N; ++i) {
        const KeyframePair& pk = pairs_[i - 1];
        cv::Mat dvw = pk.R_w_b * cv::Mat(pk.delta_v_body);
        cv::Vec3d dvw_v(dvw.at<double>(0), dvw.at<double>(1), dvw.at<double>(2));
        gamma[i] = gamma[i - 1] + dvw_v + gravity_w_ * pk.dt;
    }

    // Stack normal equations: AᵀA (4×4), Aᵀb (4×1).
    cv::Matx<double, 4, 4> AtA = cv::Matx<double, 4, 4>::zeros();
    cv::Matx<double, 4, 1> Atb = cv::Matx<double, 4, 1>::zeros();

    // Also accumulate full A and b for residual variance later.
    std::vector<cv::Matx<double, 3, 4>> A_blocks;
    std::vector<cv::Vec3d> b_blocks;
    A_blocks.reserve(N);
    b_blocks.reserve(N);

    for (size_t i = 0; i < N; ++i) {
        const KeyframePair& p = pairs_[i];
        // a_col = R_w_b · t_vis_body  (3×1)
        cv::Mat a_col_mat = p.R_w_b * cv::Mat(p.t_vis_body);
        cv::Vec3d a_col(a_col_mat.at<double>(0),
                        a_col_mat.at<double>(1),
                        a_col_mat.at<double>(2));

        // R_w_b · Δp_body  (3×1)
        cv::Mat dpw_mat = p.R_w_b * cv::Mat(p.delta_p_body);
        cv::Vec3d dpw(dpw_mat.at<double>(0),
                      dpw_mat.at<double>(1),
                      dpw_mat.at<double>(2));

        // b_i = R_w_b · Δp + γ_i · Δt + 0.5 · g · Δt²
        // Position update: p_{i+1} − p_i = v_i·Δt + R·Δp + 0.5·g·Δt². With
        // v_i = v_0 + γ_i and disp = s·R·t_vis, rearrange to A·x = b.
        cv::Vec3d b_i = dpw + gamma[i] * p.dt
                      + 0.5 * gravity_w_ * (p.dt * p.dt);

        // A_i = [a_col | −Δt · I_3]   (3×4)
        cv::Matx<double, 3, 4> A_i;
        for (int r = 0; r < 3; ++r) {
            A_i(r, 0) = a_col(r);
            A_i(r, 1) = (r == 0) ? -p.dt : 0.0;
            A_i(r, 2) = (r == 1) ? -p.dt : 0.0;
            A_i(r, 3) = (r == 2) ? -p.dt : 0.0;
        }

        // Accumulate AᵀA += A_iᵀ A_i ;  Aᵀb += A_iᵀ b_i
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                double s = 0.0;
                for (int k = 0; k < 3; ++k) s += A_i(k, r) * A_i(k, c);
                AtA(r, c) += s;
            }
            double sb = 0.0;
            for (int k = 0; k < 3; ++k) sb += A_i(k, r) * b_i(k);
            Atb(r, 0) += sb;
        }

        A_blocks.push_back(A_i);
        b_blocks.push_back(b_i);
    }

    // Solve (AᵀA) x = Aᵀb. Use cv::solve with SVD for robustness on borderline
    // conditioning (e.g., near-collinear translations).
    cv::Mat AtA_m(4, 4, CV_64F);
    cv::Mat Atb_m(4, 1, CV_64F);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) AtA_m.at<double>(r, c) = AtA(r, c);
        Atb_m.at<double>(r, 0) = Atb(r, 0);
    }
    cv::Mat x_m;
    if (!cv::solve(AtA_m, Atb_m, x_m, cv::DECOMP_SVD)) {
        return false;
    }
    double s = x_m.at<double>(0, 0);
    cv::Vec3d v0(x_m.at<double>(1, 0),
                 x_m.at<double>(2, 0),
                 x_m.at<double>(3, 0));

    // Reject non-physical scales. Production smooth_scale_ is bounded
    // [0.01, 10.0]; matching that.
    if (!std::isfinite(s) || s < 0.01 || s > 10.0) return false;

    // Residual variance: σ² ≈ (1 / (3N − 4)) · Σ ‖A_i x − b_i‖².
    double rss = 0.0;
    for (size_t i = 0; i < N; ++i) {
        cv::Vec3d r = cv::Vec3d(0.0, 0.0, 0.0);
        for (int k = 0; k < 3; ++k) {
            double v = -b_blocks[i](k);
            for (int c = 0; c < 4; ++c) v += A_blocks[i](k, c) * x_m.at<double>(c, 0);
            r(k) = v;
        }
        rss += r(0) * r(0) + r(1) * r(1) + r(2) * r(2);
    }
    int dof = static_cast<int>(3 * N) - 4;
    double sigma2 = (dof > 0) ? (rss / static_cast<double>(dof)) : rss;

    // Variance of s = sigma² · (AᵀA)⁻¹_{0,0}.
    cv::Mat AtA_inv;
    if (!cv::invert(AtA_m, AtA_inv, cv::DECOMP_SVD)) return false;
    double var_s = sigma2 * AtA_inv.at<double>(0, 0);
    if (!std::isfinite(var_s) || var_s < 0.0) return false;

    scale_out = s;
    variance_out = var_s;
    if (v0_out) *v0_out = v0;
    return true;
}
