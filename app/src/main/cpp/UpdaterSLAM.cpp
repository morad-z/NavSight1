#include "UpdaterSLAM.h"
#include "EKFState.h"
#include <opencv2/core.hpp>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-UpdaterSLAM"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#endif

bool UpdaterSLAM::update_skf(EKFState& state, int slot,
                             const std::vector<cv::Point2f>& observations,
                             const std::vector<int>& clone_ids,
                             double pixel_noise_sq) {
    auto& slam_features = state.getSlamFeatures();
    if (slot < 0 || slot >= static_cast<int>(slam_features.size())) return false;
    if (observations.size() != clone_ids.size() || observations.empty()) return false;

    EKFState::SlamFeature& f = slam_features[slot];
    const int dim = state.getStateDim();
    const cv::Mat P = state.getCovariance();

    // v23.15 (2026-05-11): Extracted from EKFState to separate UpdaterSLAM class.
    // Implements OpenVINS-style gating: parallax, depth-observability, chi-squared.
    std::vector<cv::Mat> H_rows;
    std::vector<cv::Mat> r_rows;
    H_rows.reserve(observations.size());
    r_rows.reserve(observations.size());

    double rms_acc = 0.0;
    int rms_n = 0;

    const cv::Mat R_noise_2x2 = cv::Mat::eye(2, 2, CV_64F) * std::max(pixel_noise_sq, 1e-8);

    for (size_t k = 0; k < observations.size(); k++) {
        const int clone_id = clone_ids[k];
        const int clone_cov = state.getCloneCovIdx(clone_id);
        if (clone_cov < 0) continue;

        cv::Mat R_now, p_now;
        if (!state.getClonePose(clone_id, R_now, p_now)) continue;
        cv::Mat R_FEJ, p_FEJ;
        if (!state.getCloneFEJ(clone_id, R_FEJ, p_FEJ)) {
            R_FEJ = R_now.clone();
            p_FEJ = p_now.clone();
        }

        // Sequential skip: update is degenerate against the anchor itself.
        if (f.anchor_clone_id == clone_id) continue;

        cv::Mat H_feat, H_clone;
        cv::Point2d pred;
        if (!state.slamReprojectionJacobian(f, R_FEJ, p_FEJ, R_now, p_now,
                                            H_feat, H_clone, pred)) {
            continue;
        }

        // ── 1. Parallax gate ──────────────────────────────────────────────
        const double rho = f.state.at<double>(2, 0);
        if (rho <= 0.0) continue;
        const double depth = 1.0 / rho;
        const double baseline = cv::norm(p_now - f.anchor_p_FEJ);
        if (baseline / depth < options_.min_parallax_ratio) continue;

        // ── 2. Depth observability check ──────────────────────────────────
        if (cv::norm(H_feat.col(2)) < options_.min_depth_observability) continue;

        // Sparse 2 x dim Jacobian:
        //   - clone slot        at [clone_cov, clone_cov+6)
        //   - SLAM slot         at [slam_idx, slam_idx+5)
        const int slam_idx = state.getSlamFeatureCovIdx(slot);
        cv::Mat H = cv::Mat::zeros(2, dim, CV_64F);
        H_clone.copyTo(H(cv::Range::all(),
                         cv::Range(clone_cov, clone_cov + EKFState::CLONE_DIM)));
        H_feat .copyTo(H(cv::Range::all(),
                         cv::Range(slam_idx, slam_idx + EKFState::SLAM_FEATURE_DIM)));

        // Residual = obs - predicted (normalised image coords).
        cv::Mat r = (cv::Mat_<double>(2, 1) <<
                     observations[k].x - pred.x,
                     observations[k].y - pred.y);

        // ── 3. Chi-squared gate ───────────────────────────────────────────
        const cv::Mat S = H * P * H.t() + R_noise_2x2;
        cv::Mat S_inv;
        if (!cv::invert(S, S_inv, cv::DECOMP_CHOLESKY)) {
            if (!cv::invert(S, S_inv, cv::DECOMP_SVD)) continue;
        }
        const cv::Mat m2_mat = r.t() * S_inv * r;
        if (m2_mat.at<double>(0, 0) > options_.chi2_threshold) continue;

        rms_acc += r.at<double>(0, 0) * r.at<double>(0, 0)
                 + r.at<double>(1, 0) * r.at<double>(1, 0);
        rms_n += 1;

        H_rows.push_back(H);
        r_rows.push_back(r);
    }

    if (H_rows.empty()) return false;

    // Stack rows into a single (2K x dim) H and (2K x 1) r.
    const int K = static_cast<int>(H_rows.size());
    cv::Mat H_stack = cv::Mat::zeros(2 * K, dim, CV_64F);
    cv::Mat r_stack = cv::Mat::zeros(2 * K, 1,   CV_64F);
    for (int k = 0; k < K; k++) {
        H_rows[k].copyTo(H_stack(cv::Range(2 * k, 2 * k + 2), cv::Range::all()));
        r_rows[k].copyTo(r_stack(cv::Range(2 * k, 2 * k + 2), cv::Range::all()));
    }
    cv::Mat R_noise = cv::Mat::eye(2 * K, 2 * K, CV_64F)
                      * std::max(pixel_noise_sq, 1e-8);

    // Apply the update to the EKF state.
    state.applyMSCKFUpdate(H_stack, r_stack, R_noise);

    // Diagnostics: feature RMS for the lifecycle decision in Tracker.
    if (rms_n > 0) {
        f.last_obs_rms = std::sqrt(rms_acc / static_cast<double>(rms_n));
    }
    return true;
}
