// WindowedBA.cpp — Plan Step 6 / ADR-012
//
// Hand-rolled Levenberg-Marquardt local bundle adjustment with Schur
// complement on the landmark block. See WindowedBA.h for the conventions.
//
// Linearisation summary
// ---------------------
// Projection (pinhole, no distortion — frames are rectified upstream):
//     p_c   = R * (p_w - t)         R is world->cam, t is cam-in-world
//     u_hat = fx * p_c.x / p_c.z + cx
//     v_hat = fy * p_c.y / p_c.z + cy
//     e     = (u_hat, v_hat)
//     r     = obs - e
//
// Right-mult pose perturbation, additive landmark:
//     R_new = R * Exp(phi)
//     t_new = t + dt
//     p_w_new = p_w + dp_w
//
// First-order:
//     dp_c/dt   = -R
//     dp_c/dphi = -R * [p_w - t]_x
//     dp_c/dp_w =  R
//
// With  J_proj = de/dp_c =
//     [ fx/z      0       -fx*x/z^2
//        0       fy/z     -fy*y/z^2 ]
//
// Per-observation Jacobian rows (2x6 pose, 2x3 landmark):
//     J_pose = [ -J_proj*R ,  J_proj*R*[p_w - t]_x ]    // (dt | phi)
//     J_lm   =   J_proj*R
//
// Wait: dp_c/dphi = -R*[q]_x, so de/dphi = J_proj * (-R*[q]_x) = -J_proj*R*[q]_x.
// Therefore J_pose = [-J_proj*R, -J_proj*R*[q]_x] where q = p_w - t.

#include "WindowedBA.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

#include <opencv2/calib3d.hpp>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-BA"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#endif

namespace {

// -------------------------------- helpers --------------------------------

inline cv::Matx33d skew(const cv::Vec3d& v) {
    return cv::Matx33d(   0, -v[2],  v[1],
                       v[2],     0, -v[0],
                      -v[1],  v[0],     0);
}

// Rodrigues exp: rotation vector -> rotation matrix.
// (cv::Rodrigues handles small-angle correctly via Taylor.)
inline cv::Matx33d expSO3(const cv::Vec3d& phi) {
    cv::Mat R;
    cv::Rodrigues(cv::Mat(phi), R);
    cv::Matx33d Rm;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Rm(i, j) = R.at<double>(i, j);
    return Rm;
}

// Compute J_proj (2x3) at p_c.  Returns false if p_c.z is non-positive.
inline bool projJacobian(const cv::Vec3d& p_c,
                         double fx, double fy,
                         cv::Matx<double, 2, 3>& J_proj,
                         cv::Vec2d& uv_hat) {
    const double z = p_c[2];
    if (z <= 1e-3) return false;
    const double inv_z  = 1.0 / z;
    const double inv_z2 = inv_z * inv_z;
    uv_hat[0] = fx * p_c[0] * inv_z;   // caller adds cx
    uv_hat[1] = fy * p_c[1] * inv_z;   // caller adds cy
    J_proj(0, 0) = fx * inv_z;
    J_proj(0, 1) = 0.0;
    J_proj(0, 2) = -fx * p_c[0] * inv_z2;
    J_proj(1, 0) = 0.0;
    J_proj(1, 1) = fy * inv_z;
    J_proj(1, 2) = -fy * p_c[1] * inv_z2;
    return true;
}

// --------------------------- residual evaluation ---------------------------

struct ResidualBuffers {
    double total_sq = 0.0;
    int    huber_rejects = 0;
    int    valid_obs = 0;
    int    pruned_obs = 0;
};

// Evaluate residuals at given (R_kf[], t_kf[], p_w[]) without building
// Jacobians.  Used to test step acceptance in LM.
double evaluateResidualSq(const std::vector<cv::Matx33d>& Rs,
                          const std::vector<cv::Vec3d>&   ts,
                          const std::vector<cv::Vec3d>&   pws,
                          // For each observation: (kf_idx, lm_idx, obs_uv)
                          const std::vector<std::tuple<int, int, cv::Vec2d>>& obs_table,
                          double fx, double fy, double cx, double cy,
                          double huber_thresh_px,
                          int* huber_rejects_out = nullptr,
                          int* pruned_out = nullptr) {
    double sum = 0.0;
    int rejects = 0;
    int pruned  = 0;
    for (const auto& [kf_idx, lm_idx, uv] : obs_table) {
        const cv::Vec3d q  = pws[lm_idx] - ts[kf_idx];
        const cv::Vec3d pc = Rs[kf_idx] * q;
        if (pc[2] <= 1e-3) { ++pruned; continue; }
        const double u_hat = fx * pc[0] / pc[2] + cx;
        const double v_hat = fy * pc[1] / pc[2] + cy;
        const double rx = uv[0] - u_hat;
        const double ry = uv[1] - v_hat;
        const double r_norm = std::sqrt(rx * rx + ry * ry);
        // Huber-weighted squared cost: rho(r) = r^2 if |r| < d,
        // else d*(2|r| - d).
        double cost;
        if (r_norm <= huber_thresh_px) {
            cost = r_norm * r_norm;
        } else {
            cost = huber_thresh_px * (2.0 * r_norm - huber_thresh_px);
            ++rejects;
        }
        sum += cost;
    }
    if (huber_rejects_out) *huber_rejects_out = rejects;
    if (pruned_out)        *pruned_out        = pruned;
    return sum;
}

}  // anonymous namespace

// ---------------------------------- solve ----------------------------------

WindowedBA::Result WindowedBA::solve(std::vector<PoseObs>&    poses,
                                     std::vector<FeatureObs>& features,
                                     double fx, double fy,
                                     double cx, double cy,
                                     int    max_iters,
                                     double huber_thresh_px) {
    const auto t_start = std::chrono::steady_clock::now();
    Result result;

    const int K = static_cast<int>(poses.size());
    const int N = static_cast<int>(features.size());

    // Default: copy inputs to outputs so callers always have a valid state
    // even on early-exit failure paths.
    for (auto& p : poses)    { p.R_out = p.R_in; p.t_out = p.t_in; }
    for (auto& f : features) { f.p_w_out = f.p_w_in; }

    if (K == 0 || N == 0) {
        result.solve_us = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_start).count());
        LOGE("BA: empty problem K=%d N=%d", K, N);
        return result;
    }

    // Find anchor index. If the caller forgot to flag one, anchor pose 0.
    int anchor_idx = -1;
    for (int i = 0; i < K; ++i) if (poses[i].is_anchor) { anchor_idx = i; break; }
    if (anchor_idx < 0) anchor_idx = 0;

    // Build kf_id -> kf_idx and feature-flat lookup.
    std::unordered_map<int, int> kf_id_to_idx;
    kf_id_to_idx.reserve(K);
    for (int i = 0; i < K; ++i) kf_id_to_idx[poses[i].keyframe_id] = i;

    // Working state (mutated each LM iter).
    std::vector<cv::Matx33d> Rs(K);
    std::vector<cv::Vec3d>   ts(K);
    std::vector<cv::Vec3d>   pws(N);
    for (int i = 0; i < K; ++i) { Rs[i]  = poses[i].R_in;    ts[i]  = poses[i].t_in;  }
    for (int j = 0; j < N; ++j) { pws[j] = features[j].p_w_in; }

    // Flatten observations: (kf_idx, lm_idx, uv).  Drop any obs whose kf_id
    // does not resolve — caller bug, but we silently ignore for robustness.
    std::vector<std::tuple<int, int, cv::Vec2d>> obs_table;
    obs_table.reserve(N * 4);
    for (int j = 0; j < N; ++j) {
        for (const auto& [kf_id, uv] : features[j].obs) {
            auto it = kf_id_to_idx.find(kf_id);
            if (it == kf_id_to_idx.end()) continue;
            obs_table.emplace_back(it->second, j, uv);
        }
    }
    if (obs_table.empty()) {
        result.solve_us = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_start).count());
        LOGE("BA: no observations after kf_id resolution");
        return result;
    }

    // Initial residual (Huber).
    int initial_pruned = 0;
    int initial_huber  = 0;
    const double initial_r2 = evaluateResidualSq(
        Rs, ts, pws, obs_table, fx, fy, cx, cy, huber_thresh_px,
        &initial_huber, &initial_pruned);
    result.initial_residual_sq = initial_r2;

    if (initial_pruned > static_cast<int>(obs_table.size()) / 2) {
        // More than half the points behind the cameras — geometry is too
        // degenerate to refine. Bail out cleanly (outputs already = inputs).
        result.final_residual_sq = initial_r2;
        result.huber_rejects = initial_huber;
        result.solve_us = static_cast<int>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_start).count());
        LOGE("BA: degenerate input — %d/%zu obs behind camera; abort.",
             initial_pruned, obs_table.size());
        return result;
    }

    // -------------------- LM main loop --------------------
    const int    pose_dim = K * 6;
    const int    lm_dim   = N * 3;
    double       lambda   = 1e-3;        // LM damping
    const double lambda_min = 1e-12;
    const double lambda_max = 1e+12;
    double       prev_r2  = initial_r2;
    int          iter     = 0;
    bool         converged = false;

    // Pre-allocated working matrices.
    cv::Mat Hpp(pose_dim, pose_dim, CV_64F);  // pose-pose block
    cv::Mat bp (pose_dim, 1,        CV_64F);  // pose gradient
    // Hpl is K*6 x N*3 — but only entries (kf_idx, lm_idx) for which an
    // obs exists are non-zero.  We store the dense full matrix; for K=10
    // N=30 that's 60*90 = 5400 doubles, trivial.
    cv::Mat Hpl(pose_dim, lm_dim, CV_64F);
    // Hll is block-diagonal in 3x3 chunks — store as N separate 3x3 mats.
    std::vector<cv::Matx33d> Hll(N);
    cv::Mat bl(lm_dim, 1, CV_64F);

    for (iter = 0; iter < max_iters; ++iter) {
        // --- assemble normal equations at current state ---
        Hpp.setTo(0.0);
        bp.setTo(0.0);
        Hpl.setTo(0.0);
        for (auto& m : Hll) m = cv::Matx33d::zeros();
        bl.setTo(0.0);

        int huber_count = 0;
        for (const auto& [kf_idx, lm_idx, uv] : obs_table) {
            const cv::Matx33d& R = Rs[kf_idx];
            const cv::Vec3d&   t = ts[kf_idx];
            const cv::Vec3d&   pw = pws[lm_idx];
            const cv::Vec3d    q  = pw - t;
            const cv::Vec3d    pc = R * q;
            if (pc[2] <= 1e-3) continue;

            cv::Matx<double, 2, 3> J_proj;
            cv::Vec2d              uv_hat_no_c;
            if (!projJacobian(pc, fx, fy, J_proj, uv_hat_no_c)) continue;

            const cv::Vec2d r(uv[0] - (uv_hat_no_c[0] + cx),
                              uv[1] - (uv_hat_no_c[1] + cy));
            const double r_norm = std::sqrt(r[0] * r[0] + r[1] * r[1]);

            // Huber IRLS weight: w = 1 if |r| <= delta, else sqrt(delta/|r|).
            // We apply w to *both* the residual and J so that w^2 r^2 ≈
            // Huber(r).
            double w = 1.0;
            if (r_norm > huber_thresh_px) {
                w = std::sqrt(huber_thresh_px / r_norm);
                ++huber_count;
            }

            // J_pose (2x6) = [ -J_proj*R ,  -J_proj*R*[q]_x ]
            // J_lm   (2x3) =    J_proj*R
            const cv::Matx<double, 2, 3> JpR = J_proj * R;
            const cv::Matx33d            qx  = skew(q);

            cv::Matx<double, 2, 6> J_pose;
            for (int rr = 0; rr < 2; ++rr) {
                // dt block
                J_pose(rr, 0) = -JpR(rr, 0);
                J_pose(rr, 1) = -JpR(rr, 1);
                J_pose(rr, 2) = -JpR(rr, 2);
                // phi block:  -JpR * qx
                const double a = -(JpR(rr, 0) * qx(0, 0) + JpR(rr, 1) * qx(1, 0) + JpR(rr, 2) * qx(2, 0));
                const double b = -(JpR(rr, 0) * qx(0, 1) + JpR(rr, 1) * qx(1, 1) + JpR(rr, 2) * qx(2, 1));
                const double c = -(JpR(rr, 0) * qx(0, 2) + JpR(rr, 1) * qx(1, 2) + JpR(rr, 2) * qx(2, 2));
                J_pose(rr, 3) = a;
                J_pose(rr, 4) = b;
                J_pose(rr, 5) = c;
            }
            const cv::Matx<double, 2, 3> J_lm = JpR;

            // Gauss-Newton normal-equation contributions (Jacobian of
            // residual = -J_pred; sign convention used here:
            //     min  sum  r^T r,   r = obs - pred
            //     dr/dx = -de/dx
            //     normal eq:  (de/dx)^T (de/dx) dx = (de/dx)^T r
            // i.e. H = J^T J,  b = J^T r  (with J = de/dx).
            // Apply IRLS weight w:  J -> w*J,  r -> w*r, so
            //   H += (wJ)^T (wJ) = w^2 J^T J
            //   b += (wJ)^T (wr) = w^2 J^T r
            const double w2 = w * w;

            // ---- pose-pose 6x6 ----
            // Skip the whole block contribution if this is the anchor —
            // we want gauge fixing.  Exception: anchor's *self* coupling
            // still gets a +lambda*I in the damping step, but we don't
            // accumulate observation Jacobian rows for it.
            const bool is_anchor = (kf_idx == anchor_idx);
            const int  p_off     = kf_idx * 6;
            const int  l_off     = lm_idx * 3;

            if (!is_anchor) {
                // Hpp block += w2 * J_pose^T * J_pose   (6x6)
                for (int a = 0; a < 6; ++a) {
                    for (int b = 0; b < 6; ++b) {
                        double s = 0.0;
                        for (int k = 0; k < 2; ++k)
                            s += J_pose(k, a) * J_pose(k, b);
                        Hpp.at<double>(p_off + a, p_off + b) += w2 * s;
                    }
                    // bp += w2 * J_pose^T * r
                    bp.at<double>(p_off + a, 0) += (J_pose(0, a) * w2 * r[0] +
                                                    J_pose(1, a) * w2 * r[1]);
                }
            }

            // ---- landmark-landmark 3x3 (always accumulated) ----
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b) {
                    double s = 0.0;
                    for (int k = 0; k < 2; ++k)
                        s += J_lm(k, a) * J_lm(k, b);
                    Hll[lm_idx](a, b) += w2 * s;
                }
                bl.at<double>(l_off + a, 0) += (J_lm(0, a) * w2 * r[0] +
                                                J_lm(1, a) * w2 * r[1]);
            }

            // ---- pose-landmark 6x3 cross block ----
            if (!is_anchor) {
                for (int a = 0; a < 6; ++a) {
                    for (int b = 0; b < 3; ++b) {
                        double s = 0.0;
                        for (int k = 0; k < 2; ++k)
                            s += J_pose(k, a) * J_lm(k, b);
                        Hpl.at<double>(p_off + a, l_off + b) += w2 * s;
                    }
                }
            }
        }  // end obs_table loop

        // Zero-row gauge fix on the anchor: ensure its 6 rows/cols of Hpp
        // and Hpl are zero (they already are because we skipped accumulation),
        // and seed the diagonal with a strong identity so the linear solve
        // is non-singular.  After the solve, the anchor's dx_p slot will be
        // identically zero (any tiny residual is < 1e-12 of the rest).
        const int a_off = anchor_idx * 6;
        for (int a = 0; a < 6; ++a) {
            // Defensive — already zero, but clear in case of FP noise.
            for (int c = 0; c < pose_dim; ++c) {
                Hpp.at<double>(a_off + a, c) = 0.0;
                Hpp.at<double>(c, a_off + a) = 0.0;
            }
            for (int c = 0; c < lm_dim; ++c) {
                Hpl.at<double>(a_off + a, c) = 0.0;
            }
            Hpp.at<double>(a_off + a, a_off + a) = 1.0;  // identity gauge
            bp.at<double>(a_off + a, 0)          = 0.0;
        }

        // --- LM damping: H += lambda * diag(H) ---
        // Snapshot diagonals before mutation so we can restore on reject.
        std::vector<double> Hpp_diag(pose_dim);
        for (int i = 0; i < pose_dim; ++i)
            Hpp_diag[i] = Hpp.at<double>(i, i);
        std::vector<cv::Matx33d> Hll_diag = Hll;  // copy

        // Try-step inner loop.
        bool   step_accepted = false;
        int    inner_tries   = 0;
        cv::Mat dx_p;
        while (!step_accepted && inner_tries < 8) {
            ++inner_tries;

            // Apply damping to Hpp diagonal
            for (int i = 0; i < pose_dim; ++i)
                Hpp.at<double>(i, i) = Hpp_diag[i] * (1.0 + lambda) + 1e-12;

            // Apply damping to each Hll 3x3 block, then invert it.
            std::vector<cv::Matx33d> Hll_inv(N);
            bool lm_invert_ok = true;
            for (int j = 0; j < N; ++j) {
                cv::Matx33d damped = Hll_diag[j];
                for (int d = 0; d < 3; ++d)
                    damped(d, d) = damped(d, d) * (1.0 + lambda) + 1e-12;
                double det = cv::determinant(cv::Mat(damped));
                if (std::abs(det) < 1e-30) {
                    // landmark unobservable — set inv to small identity so
                    // the Schur reduction reduces to a pose-only step for it.
                    Hll_inv[j] = cv::Matx33d::eye() * 1e6;
                    continue;
                }
                Hll_inv[j] = damped.inv();
            }
            (void)lm_invert_ok;

            // --- Schur complement: solve  S * dx_p = bs ---
            //   S  = Hpp - Hpl * Hll_inv * Hpl^T
            //   bs = bp  - Hpl * Hll_inv * bl
            //
            // We exploit that Hll_inv is block-diagonal: we don't form a
            // (lm_dim x lm_dim) matrix.  Instead per landmark j we
            // compute  Hpl_j * Hll_inv_j * Hpl_j^T  and add to S.
            cv::Mat S  = Hpp.clone();              // (pose_dim x pose_dim)
            cv::Mat bs = bp.clone();               // (pose_dim x 1)

            for (int j = 0; j < N; ++j) {
                const int l_off = j * 3;
                // Extract Hpl column slice (pose_dim x 3) for landmark j.
                cv::Mat Hpl_j = Hpl(cv::Rect(l_off, 0, 3, pose_dim));  // x,y,w,h
                // tmp1 (pose_dim x 3) = Hpl_j * Hll_inv_j
                cv::Mat HllInv_j_mat(3, 3, CV_64F);
                for (int a = 0; a < 3; ++a)
                    for (int b = 0; b < 3; ++b)
                        HllInv_j_mat.at<double>(a, b) = Hll_inv[j](a, b);
                cv::Mat tmp1 = Hpl_j * HllInv_j_mat;            // P x 3
                // S -= tmp1 * Hpl_j^T   (P x P)
                S -= tmp1 * Hpl_j.t();
                // bs -= tmp1 * bl_j     (P x 1)
                cv::Mat bl_j = bl(cv::Rect(0, l_off, 1, 3));
                bs -= tmp1 * bl_j;
            }

            // Solve S * dx_p = bs.  S is symmetric PD (after damping +
            // anchor gauge fix), so Cholesky.
            const bool solve_ok = cv::solve(S, bs, dx_p, cv::DECOMP_CHOLESKY);
            if (!solve_ok) {
                lambda = std::min(lambda * 10.0, lambda_max);
                if (lambda >= lambda_max) break;
                continue;
            }

            // Back-substitute landmark deltas:
            //   dx_l_j = Hll_inv_j * (bl_j - Hpl_j^T * dx_p)
            std::vector<cv::Vec3d> dx_l(N, cv::Vec3d(0, 0, 0));
            for (int j = 0; j < N; ++j) {
                const int l_off = j * 3;
                cv::Mat Hpl_j = Hpl(cv::Rect(l_off, 0, 3, pose_dim));
                cv::Mat rhs_j = bl(cv::Rect(0, l_off, 1, 3)) - Hpl_j.t() * dx_p;
                cv::Vec3d rhs(rhs_j.at<double>(0, 0),
                              rhs_j.at<double>(1, 0),
                              rhs_j.at<double>(2, 0));
                dx_l[j] = Hll_inv[j] * rhs;
            }

            // --- candidate state ---
            std::vector<cv::Matx33d> Rs_try = Rs;
            std::vector<cv::Vec3d>   ts_try = ts;
            std::vector<cv::Vec3d>   pws_try = pws;
            double max_pose_step = 0.0;

            for (int i = 0; i < K; ++i) {
                if (i == anchor_idx) continue;  // gauge-fixed
                const int p_off = i * 6;
                cv::Vec3d dt(dx_p.at<double>(p_off + 0, 0),
                             dx_p.at<double>(p_off + 1, 0),
                             dx_p.at<double>(p_off + 2, 0));
                cv::Vec3d dphi(dx_p.at<double>(p_off + 3, 0),
                               dx_p.at<double>(p_off + 4, 0),
                               dx_p.at<double>(p_off + 5, 0));
                ts_try[i] = ts[i] + dt;
                Rs_try[i] = Rs[i] * expSO3(dphi);
                const double step_norm =
                    std::sqrt(dt.dot(dt) + dphi.dot(dphi));
                if (step_norm > max_pose_step) max_pose_step = step_norm;
            }
            for (int j = 0; j < N; ++j) {
                pws_try[j] = pws[j] + dx_l[j];
            }

            // --- evaluate new residual ---
            int new_huber = 0;
            int new_pruned = 0;
            const double new_r2 = evaluateResidualSq(
                Rs_try, ts_try, pws_try, obs_table,
                fx, fy, cx, cy, huber_thresh_px,
                &new_huber, &new_pruned);

            if (new_r2 < prev_r2 && new_pruned <= initial_pruned) {
                // Accept step.
                Rs = std::move(Rs_try);
                ts = std::move(ts_try);
                pws = std::move(pws_try);
                lambda = std::max(lambda * 0.5, lambda_min);
                step_accepted = true;

                const double rel_change =
                    (prev_r2 > 1e-30) ? (prev_r2 - new_r2) / prev_r2 : 0.0;
                prev_r2 = new_r2;
                result.huber_rejects = new_huber;

                if (rel_change < 1e-4 || max_pose_step < 1e-6) {
                    converged = true;
                }
            } else {
                // Reject — increase damping, restore Hpp diagonal, retry.
                lambda = std::min(lambda * 10.0, lambda_max);
                for (int i = 0; i < pose_dim; ++i)
                    Hpp.at<double>(i, i) = Hpp_diag[i];
                if (lambda >= lambda_max) break;
            }
        }  // end inner LM try-loop

        result.iterations = iter + 1;
        if (!step_accepted) {
            // Could not find a step at this lambda — declare done.
            break;
        }
        if (converged) break;
    }

    // -------------------- write outputs --------------------
    for (int i = 0; i < K; ++i) {
        poses[i].R_out = Rs[i];
        poses[i].t_out = ts[i];
    }
    for (int j = 0; j < N; ++j) {
        features[j].p_w_out = pws[j];
    }

    result.final_residual_sq = prev_r2;
    result.converged         = converged;

    const auto t_end = std::chrono::steady_clock::now();
    result.solve_us = static_cast<int>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count());

    LOGI("BA: solve_us=%d iters=%d initial_r2=%.2f final_r2=%.2f huber=%d",
         result.solve_us, result.iterations,
         result.initial_residual_sq, result.final_residual_sq,
         result.huber_rejects);
    return result;
}
