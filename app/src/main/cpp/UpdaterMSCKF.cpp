#include "UpdaterMSCKF.h"
#include "EKFState.h"
#include <cmath>
#include <algorithm>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-MSCKF"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGD(...) (void)0
#endif

// ── Triangulation (DLT) ────────────────────────────────────────────────────

bool UpdaterMSCKF::triangulate(const LostFeature& feat, const EKFState& state,
                                cv::Point3d& pt3d_out) const {
    if (static_cast<int>(feat.observations.size()) < options_.min_obs) return false;

    // Build DLT system: for each observation, 2 equations
    int M = static_cast<int>(feat.observations.size());
    cv::Mat A(2 * M, 4, CV_64F, cv::Scalar(0));

    for (int i = 0; i < M; i++) {
        const auto& obs = feat.observations[i];
        cv::Mat R_CtoG, p_C;
        if (!state.getClonePose(obs.clone_state_id, R_CtoG, p_C)) return false;

        // Projection matrix P = K * [R | -R*p] but we use normalized coords
        // so P = [R^T | -R^T * p] (3x4)
        cv::Mat R_GtoC = R_CtoG.t();
        cv::Mat t_GtoC = -R_GtoC * p_C;
        cv::Mat P(3, 4, CV_64F);
        R_GtoC.copyTo(P(cv::Range(0,3), cv::Range(0,3)));
        t_GtoC.copyTo(P(cv::Range(0,3), cv::Range(3,4)));

        double u = obs.pixel_ud.x;
        double v = obs.pixel_ud.y;

        // x * P[2,:] - P[0,:]
        for (int j = 0; j < 4; j++) {
            A.at<double>(2*i,   j) = u * P.at<double>(2,j) - P.at<double>(0,j);
            A.at<double>(2*i+1, j) = v * P.at<double>(2,j) - P.at<double>(1,j);
        }
    }

    // SVD: solution is last column of V^T
    cv::SVD svd(A, cv::SVD::FULL_UV);
    cv::Mat V = svd.vt.t();
    double w = V.at<double>(3, 3);
    if (std::abs(w) < 1e-10) return false;

    pt3d_out.x = V.at<double>(0, 3) / w;
    pt3d_out.y = V.at<double>(1, 3) / w;
    pt3d_out.z = V.at<double>(2, 3) / w;

    // Validate: check positive depth in all cameras
    for (const auto& obs : feat.observations) {
        cv::Mat R_CtoG, p_C;
        if (!state.getClonePose(obs.clone_state_id, R_CtoG, p_C)) return false;
        cv::Mat R_GtoC = R_CtoG.t();
        cv::Mat pt_G = (cv::Mat_<double>(3,1) << pt3d_out.x, pt3d_out.y, pt3d_out.z);
        cv::Mat pt_C = R_GtoC * (pt_G - p_C);
        if (pt_C.at<double>(2) <= 0.1) return false;  // Behind camera
    }

    return true;
}

// ── Feature Jacobian ────────────────────────────────────────────────────────

void UpdaterMSCKF::getFeatureJacobian(const LostFeature& feat, const EKFState& state,
                                       const cv::Point3d& pt3d,
                                       double fx, double fy, double cx, double cy,
                                       cv::Mat& H_f, cv::Mat& H_x, cv::Mat& res) const {
    int M = static_cast<int>(feat.observations.size());
    int state_dim = state.getStateDim();

    H_f = cv::Mat::zeros(2 * M, 3, CV_64F);
    H_x = cv::Mat::zeros(2 * M, state_dim, CV_64F);
    res = cv::Mat::zeros(2 * M, 1, CV_64F);

    cv::Mat p_f = (cv::Mat_<double>(3,1) << pt3d.x, pt3d.y, pt3d.z);

    for (int i = 0; i < M; i++) {
        const auto& obs = feat.observations[i];
        cv::Mat R_CtoG, p_C;
        // Use FEJ clone pose for Jacobians (observability preservation).
        // If the clone has been evicted between triangulation and Jacobian
        // construction, leave this row at zeros (contributes nothing to the
        // null-space projection or χ² gate) instead of running on
        // uninitialised R_CtoG/p_C. Mirrors the guarded pattern at lines 29
        // and 62 in triangulate().
        if (!state.getCloneFEJ(obs.clone_state_id, R_CtoG, p_C)) continue;

        cv::Mat R_GtoC = R_CtoG.t();
        cv::Mat p_f_C = R_GtoC * (p_f - p_C);  // Feature in camera frame

        double X = p_f_C.at<double>(0);
        double Y = p_f_C.at<double>(1);
        double Z = p_f_C.at<double>(2);
        double Z_inv = 1.0 / Z;
        double Z_inv2 = Z_inv * Z_inv;

        // Predicted measurement (normalized coordinates)
        double u_pred = X * Z_inv;
        double v_pred = Y * Z_inv;

        // Residual (in normalized coordinates)
        res.at<double>(2*i,   0) = obs.pixel_ud.x - u_pred;
        res.at<double>(2*i+1, 0) = obs.pixel_ud.y - v_pred;

        // dz/dp_f_C (2x3): Jacobian of projection w.r.t. point in camera frame
        cv::Mat dz_dpfc = (cv::Mat_<double>(2,3) <<
            Z_inv,  0,      -X * Z_inv2,
            0,      Z_inv,  -Y * Z_inv2);

        // dp_f_C / dp_f_G = R_GtoC (3x3)
        // H_f_i = dz/dp_f_C * R_GtoC
        cv::Mat H_fi = dz_dpfc * R_GtoC;
        H_fi.copyTo(H_f(cv::Range(2*i, 2*i+2), cv::Range(0, 3)));

        // Jacobian w.r.t. clone pose (rotation and position).
        //
        // 2026-05-09 v15 sign fix — same family of bug as gravity-alignment.
        // The clone rotation update at EKFState.cpp:760 is left-multiply:
        //   R_GtoC_new = exp([δθ]×) · R_GtoC_old.
        // Linearising p_f_C = R_GtoC · (p_f − p_C) under that:
        //   p_f_C_new ≈ (I + [δθ]×) · p_f_C = p_f_C − [p_f_C]× · δθ
        //   ∂p_f_C / ∂δθ = −[p_f_C]×
        //   ∂h / ∂δθ     = dz/dp_f_C · (−[p_f_C]×) = −dz_dpfc · [p_f_C]×
        // applyMSCKFUpdate is standard Kalman (dx = K·r with H = ∂h/∂x),
        // so we MUST pass the negated form. Pre-fix code used +dz·[p_f_C]×
        // which is ∂r/∂δθ (residual derivative) — the wrong sign for K·r.
        // With the wrong sign, every MSCKF update walked clone rotations in
        // the OPPOSITE direction of truth → predicted reprojections drifted
        // → real residuals looked like outliers → Huber mass-rejected the
        // very 3D-point constraints we need. v14 sim showed this directly:
        // "27 lost features all rejected (chi²/triang)".
        // Position Jacobian was already correct: p_f_C_new = R_GtoC·(p_f−(p_C+δp))
        // gives ∂p_f_C/∂δp = −R_GtoC, so H_pos = dz_dpfc · (−R_GtoC) is
        // ∂h/∂δp = the right sign. No change needed there.
        cv::Mat skew_pfc = (cv::Mat_<double>(3,3) <<
            0, -Z, Y,
            Z, 0, -X,
            -Y, X, 0);

        // H_theta = -dz/dp_f_C * [p_f_C]_x   (standard Kalman ∂h/∂x convention)
        cv::Mat H_theta = -1.0 * dz_dpfc * skew_pfc;
        // H_pos = dz/dp_f_C * (-R_GtoC)      (already correct)
        cv::Mat H_pos = dz_dpfc * (-R_GtoC);

        // Place in H_x at the clone's covariance index
        int clone_idx = state.getCloneCovIdx(obs.clone_state_id);
        if (clone_idx >= 0) {
            H_theta.copyTo(H_x(cv::Range(2*i, 2*i+2), cv::Range(clone_idx, clone_idx+3)));
            H_pos.copyTo(H_x(cv::Range(2*i, 2*i+2), cv::Range(clone_idx+3, clone_idx+6)));
        }
    }
}

// ── Null-Space Projection ───────────────────────────────────────────────────

void UpdaterMSCKF::nullspaceProject(const cv::Mat& H_f, cv::Mat& H_x, cv::Mat& res) {
    // Left null-space projection of H_f via SVD.
    //
    // H_f is (2M x 3). SVD: H_f = U · Σ · V^T where U is (2M x 2M).
    // Columns 0..2 of U span the column space of H_f; columns 3..2M-1 span
    // the left null-space. Multiplying H_x and res by Q2 = U[:, 3:]^T removes
    // the 3 feature DOF, leaving (2M-3) measurement rows the EKF can consume.
    //
    // Implementation note: the function is named "nullspaceProject" and was
    // originally intended to use Householder QR — but OpenCV doesn't expose a
    // direct Householder QR factorisation, so the SVD-based equivalent below
    // is what actually runs.

    int rows = H_f.rows;
    if (rows <= 3) return;  // Need ≥ 2 observations (4 rows) for 3 DOF feature

    cv::SVD svd(H_f, cv::SVD::FULL_UV);
    cv::Mat Q2 = svd.u(cv::Range::all(), cv::Range(3, rows));  // (2M x (2M-3))

    // Project
    cv::Mat H_projected = Q2.t() * H_x;  // (2M-3) x state_dim
    cv::Mat r_projected = Q2.t() * res;   // (2M-3) x 1

    H_x = H_projected;
    res = r_projected;
}

// ── Chi-squared threshold (95% confidence) ──────────────────────────────────

double UpdaterMSCKF::getChi2Threshold(int dof) const {
    // Approximate chi-squared 95% threshold for common DOFs
    // For large DOF: chi2_95 ≈ dof + 1.645 * sqrt(2*dof)
    if (dof <= 0) return 0;
    static const double table[] = {
        3.841, 5.991, 7.815, 9.488, 11.070, 12.592, 14.067, 15.507,
        16.919, 18.307, 19.675, 21.026, 22.362, 23.685, 24.996, 26.296,
        27.587, 28.869, 30.144, 31.410
    };
    if (dof <= 20) return table[dof - 1] * options_.chi2_multiplier;
    return (dof + 1.645 * std::sqrt(2.0 * dof)) * options_.chi2_multiplier;
}

// ── Main MSCKF Update ───────────────────────────────────────────────────────

int UpdaterMSCKF::processLostFeatures(EKFState& state,
                                       const std::vector<LostFeature>& lost_features,
                                       double fx, double fy, double cx, double cy) {
    if (lost_features.empty()) return 0;

    // Collect all valid features' projected Jacobians and residuals
    std::vector<cv::Mat> H_stack, r_stack;
    int total_rows = 0;

    for (const auto& feat : lost_features) {
        if (static_cast<int>(feat.observations.size()) < options_.min_obs) continue;

        // 1. Triangulate
        cv::Point3d pt3d;
        if (!triangulate(feat, state, pt3d)) continue;

        // 2. Form Jacobians
        cv::Mat H_f, H_x, res;
        getFeatureJacobian(feat, state, pt3d, fx, fy, cx, cy, H_f, H_x, res);

        // 3. Null-space projection (eliminates 3 feature DOF)
        nullspaceProject(H_f, H_x, res);

        if (H_x.rows == 0) continue;

        // 4. Chi-squared gate per feature
        int dof = H_x.rows;
        cv::Mat P = state.getCovariance();
        if (P.empty() || P.rows < H_x.cols) continue;

        cv::Mat P_sub = cv::Mat(P(cv::Range(0, H_x.cols), cv::Range(0, H_x.cols)));
        cv::Mat S = cv::Mat(H_x * P_sub * H_x.t());
        double sigma2 = options_.pixel_noise * options_.pixel_noise;
        S = S + sigma2 * cv::Mat::eye(dof, dof, CV_64F);

        cv::Mat S_inv;
        if (!cv::invert(S, S_inv, cv::DECOMP_CHOLESKY)) continue;

        cv::Mat chi2_mat = cv::Mat(res.t() * S_inv * res);
        double chi2 = chi2_mat.at<double>(0);
        if (chi2 > getChi2Threshold(dof)) continue;

        H_stack.push_back(H_x);
        r_stack.push_back(res);
        total_rows += H_x.rows;
    }

    if (total_rows == 0) return 0;

    // Stack all features into one big update
    int state_dim = state.getStateDim();
    cv::Mat H_all(total_rows, state_dim, CV_64F);
    cv::Mat r_all(total_rows, 1, CV_64F);
    int row = 0;
    for (size_t i = 0; i < H_stack.size(); i++) {
        H_stack[i].copyTo(H_all(cv::Range(row, row + H_stack[i].rows), cv::Range::all()));
        r_stack[i].copyTo(r_all(cv::Range(row, row + r_stack[i].rows), cv::Range::all()));
        row += H_stack[i].rows;
    }

    // Compress if measurement dimension > state dimension (QR compression)
    if (total_rows > state_dim) {
        cv::SVD svd(H_all, cv::SVD::FULL_UV);
        int rank = std::min(total_rows, state_dim);
        cv::Mat Q1 = svd.u(cv::Range::all(), cv::Range(0, rank));
        H_all = Q1.t() * H_all;  // rank x state_dim
        r_all = Q1.t() * r_all;  // rank x 1
        total_rows = rank;
    }

    // Apply EKF update
    double sigma2 = options_.pixel_noise * options_.pixel_noise;
    cv::Mat R_noise = sigma2 * cv::Mat::eye(total_rows, total_rows, CV_64F);
    state.applyMSCKFUpdate(H_all, r_all, R_noise);

    int used = static_cast<int>(H_stack.size());
    LOGI("MSCKF: updated with %d features (%d measurement rows)", used, total_rows);
    return used;
}
