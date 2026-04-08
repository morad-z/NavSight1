#include "EKFState.h"
#include <cmath>
#include <algorithm>
#include <opencv2/calib3d.hpp>
#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-EKF"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#endif

EKFState::EKFState() { reset(); }

void EKFState::reset() {
    scale_ = 0.20;
    scale_fej_ = -1.0;
    P_scale_ = 0.5 * 0.5;
    initialized_ = false;
    full_initialized_ = false;
    next_state_id_ = 0;
    window_.clear();
    global_fej_initialized_ = false;
    global_first_estimate_R_ = cv::Mat::eye(3, 3, CV_64F);
    global_first_estimate_p_ = cv::Mat::zeros(3, 1, CV_64F);
    t_offset_cam_imu_ = 0.010;
    P_td_ = 0.005 * 0.005;

    R_GtoI_ = cv::Mat::eye(3, 3, CV_64F);
    b_g_ = cv::Mat::zeros(3, 1, CV_64F);
    v_G_ = cv::Mat::zeros(3, 1, CV_64F);
    b_a_ = cv::Mat::zeros(3, 1, CV_64F);
    p_G_ = cv::Mat::zeros(3, 1, CV_64F);
    P_ = cv::Mat();
}

void EKFState::initialize(double initial_scale) {
    scale_ = initial_scale;
    P_scale_ = 0.5 * 0.5;
    initialized_ = true;
    LOGI("EKF initialized: scale=%.3f", initial_scale);
}

// ── Full IMU State Initialization ───────────────────────────────────────────

void EKFState::initializeFull(const cv::Mat& R_GtoI, const cv::Point3f& gyro_bias,
                               const cv::Point3f& accel_bias) {
    R_GtoI_ = R_GtoI.clone();
    b_g_ = (cv::Mat_<double>(3,1) << gyro_bias.x, gyro_bias.y, gyro_bias.z);
    b_a_ = (cv::Mat_<double>(3,1) << accel_bias.x, accel_bias.y, accel_bias.z);
    v_G_ = cv::Mat::zeros(3, 1, CV_64F);
    p_G_ = cv::Mat::zeros(3, 1, CV_64F);

    // Initialize 15x15 IMU covariance
    P_ = cv::Mat::zeros(IMU_STATE_DIM, IMU_STATE_DIM, CV_64F);
    // Initial uncertainties: rotation(3), bg(3), velocity(3), ba(3), position(3)
    double init_rot_std = 0.02;    // ~1 degree
    double init_bg_std = 0.01;     // rad/s
    double init_vel_std = 0.5;     // m/s
    double init_ba_std = 0.1;      // m/s^2
    double init_pos_std = 0.01;    // meters

    for (int i = 0; i < 3; i++) {
        P_.at<double>(i, i) = init_rot_std * init_rot_std;
        P_.at<double>(3+i, 3+i) = init_bg_std * init_bg_std;
        P_.at<double>(6+i, 6+i) = init_vel_std * init_vel_std;
        P_.at<double>(9+i, 9+i) = init_ba_std * init_ba_std;
        P_.at<double>(12+i, 12+i) = init_pos_std * init_pos_std;
    }

    full_initialized_ = true;
    LOGI("EKF full state initialized (15-DOF error-state)");
}

// ── IMU Propagation ─────────────────────────────────────────────────────────

void EKFState::propagateIMU(const cv::Mat& deltaR, const cv::Mat& deltaV,
                             const cv::Mat& deltaP, double dt,
                             const cv::Mat& imu_cov,
                             const cv::Mat& J_R_bg, const cv::Mat& J_V_bg,
                             const cv::Mat& J_V_ba, const cv::Mat& J_P_bg,
                             const cv::Mat& J_P_ba) {
    if (!full_initialized_ || dt <= 0 || P_.empty()) return;

    // Propagate mean state (gravity in global frame: [0, -9.81, 0] for Y-up)
    cv::Mat g = (cv::Mat_<double>(3,1) << 0.0, -9.81, 0.0);
    cv::Mat R_new = R_GtoI_ * deltaR;
    cv::Mat v_new = v_G_ + g * dt + R_GtoI_.t() * deltaV;
    cv::Mat p_new = p_G_ + v_G_ * dt + 0.5 * g * dt * dt + R_GtoI_.t() * deltaP;

    // Build discrete state transition matrix Phi (15x15)
    // Error-state: [δθ, δb_g, δv, δb_a, δp]
    // Linearized dynamics (simplified for pedestrian VIO):
    //   δθ_new  = deltaR^T * δθ - J_R_bg * δb_g
    //   δb_g    = δb_g (random walk)
    //   δv_new  = δv + R^T * [deltaV]_x * δθ - R^T * J_V_bg * δb_g - R^T * J_V_ba * δb_a
    //   δb_a    = δb_a (random walk)
    //   δp_new  = δp + δv * dt + R^T * [deltaP]_x * δθ - R^T * J_P_bg * δb_g - R^T * J_P_ba * δb_a

    cv::Mat Phi = cv::Mat::eye(IMU_STATE_DIM, IMU_STATE_DIM, CV_64F);
    cv::Mat Rt = R_GtoI_.t();

    // Skew-symmetric matrices for deltaV and deltaP
    auto skew = [](const cv::Mat& v) -> cv::Mat {
        return (cv::Mat_<double>(3,3) <<
            0, -v.at<double>(2), v.at<double>(1),
            v.at<double>(2), 0, -v.at<double>(0),
            -v.at<double>(1), v.at<double>(0), 0);
    };

    // Phi blocks (row, col) — indices: θ=0, bg=3, v=6, ba=9, p=12
    cv::Mat dRt = deltaR.t();
    dRt.copyTo(Phi(cv::Range(0,3), cv::Range(0,3)));           // dθ/dθ = deltaR^T

    if (!J_R_bg.empty()) {
        cv::Mat block = -J_R_bg;
        block.copyTo(Phi(cv::Range(0,3), cv::Range(3,6)));     // dθ/db_g
    }

    // dv/dθ: R^T * [deltaV]_x
    if (!deltaV.empty()) {
        cv::Mat dv_dtheta = Rt * skew(deltaV);
        dv_dtheta.copyTo(Phi(cv::Range(6,9), cv::Range(0,3)));
    }

    // dv/db_g: -R^T * J_V_bg
    if (!J_V_bg.empty()) {
        cv::Mat block = -Rt * J_V_bg;
        block.copyTo(Phi(cv::Range(6,9), cv::Range(3,6)));
    }

    // dv/db_a: -R^T * J_V_ba
    if (!J_V_ba.empty()) {
        cv::Mat block = -Rt * J_V_ba;
        block.copyTo(Phi(cv::Range(6,9), cv::Range(9,12)));
    }

    // dp/dθ: R^T * [deltaP]_x
    if (!deltaP.empty()) {
        cv::Mat dp_dtheta = Rt * skew(deltaP);
        dp_dtheta.copyTo(Phi(cv::Range(12,15), cv::Range(0,3)));
    }

    // dp/dv: I * dt
    cv::Mat dt_eye = dt * cv::Mat::eye(3, 3, CV_64F);
    dt_eye.copyTo(Phi(cv::Range(12,15), cv::Range(6,9)));

    // dp/db_g: -R^T * J_P_bg
    if (!J_P_bg.empty()) {
        cv::Mat block = -Rt * J_P_bg;
        block.copyTo(Phi(cv::Range(12,15), cv::Range(3,6)));
    }

    // dp/db_a: -R^T * J_P_ba
    if (!J_P_ba.empty()) {
        cv::Mat block = -Rt * J_P_ba;
        block.copyTo(Phi(cv::Range(12,15), cv::Range(9,12)));
    }

    // Process noise Q (15x15)
    cv::Mat Q = cv::Mat::zeros(IMU_STATE_DIM, IMU_STATE_DIM, CV_64F);
    // Use preintegration covariance if available, otherwise construct from noise params
    if (!imu_cov.empty() && imu_cov.rows >= 9) {
        // Map 9x9 preintegration cov (R,V,P) to 15x15 state noise
        // Rotation noise -> θ block
        imu_cov(cv::Range(0,3), cv::Range(0,3)).copyTo(Q(cv::Range(0,3), cv::Range(0,3)));
        // Velocity noise -> v block
        imu_cov(cv::Range(3,6), cv::Range(3,6)).copyTo(Q(cv::Range(6,9), cv::Range(6,9)));
        // Position noise -> p block
        imu_cov(cv::Range(6,9), cv::Range(6,9)).copyTo(Q(cv::Range(12,15), cv::Range(12,15)));
    } else {
        double dt2 = dt * dt;
        for (int i = 0; i < 3; i++) {
            Q.at<double>(i, i) = sigma_g_ * sigma_g_ * dt;
            Q.at<double>(6+i, 6+i) = sigma_a_ * sigma_a_ * dt;
            Q.at<double>(12+i, 12+i) = sigma_a_ * sigma_a_ * dt2 * dt / 3.0;
        }
    }
    // Bias random walk noise
    for (int i = 0; i < 3; i++) {
        Q.at<double>(3+i, 3+i) = sigma_bg_ * sigma_bg_ * dt;
        Q.at<double>(9+i, 9+i) = sigma_ba_ * sigma_ba_ * dt;
    }

    // Propagate covariance: full state includes clones
    int total_dim = P_.rows;
    if (total_dim >= IMU_STATE_DIM) {
        // P_II = Phi * P_II * Phi^T + Q
        cv::Mat P_II = P_(cv::Range(0, IMU_STATE_DIM), cv::Range(0, IMU_STATE_DIM));
        cv::Mat P_II_new = Phi * P_II * Phi.t() + Q;
        P_II_new.copyTo(P_(cv::Range(0, IMU_STATE_DIM), cv::Range(0, IMU_STATE_DIM)));

        // P_IC = Phi * P_IC (cross-correlation with clones)
        if (total_dim > IMU_STATE_DIM) {
            cv::Mat P_IC = P_(cv::Range(0, IMU_STATE_DIM),
                             cv::Range(IMU_STATE_DIM, total_dim));
            cv::Mat P_IC_new = Phi * P_IC;
            P_IC_new.copyTo(P_(cv::Range(0, IMU_STATE_DIM),
                              cv::Range(IMU_STATE_DIM, total_dim)));
            // P_CI = P_IC^T
            cv::Mat P_CI_new = P_IC_new.t();
            P_CI_new.copyTo(P_(cv::Range(IMU_STATE_DIM, total_dim),
                              cv::Range(0, IMU_STATE_DIM)));
        }
    }

    // Update mean state
    R_GtoI_ = R_new;
    v_G_ = v_new;
    p_G_ = p_new;

    // Clamp velocity to physically reasonable bounds (pedestrian: max 5 m/s)
    double v_norm = cv::norm(v_G_);
    if (v_norm > 5.0) {
        v_G_ *= 5.0 / v_norm;
    }
}

// ── Legacy Scale update ─────────────────────────────────────────────────────

void EKFState::updateScale(double observed_scale, double confidence) {
    if (observed_scale <= 0.005 || confidence <= 0.0) return;
    P_scale_ += SIGMA_SCALE_RW * SIGMA_SCALE_RW;
    double R = SIGMA_SCALE_MEAS / (confidence + 0.01);
    R = std::max(R, 0.001);

    // Scale is observable (from steps/gravity) — use current estimate for innovation,
    // NOT FEJ. FEJ is only for unobservable directions (yaw, abs position).
    double innov = observed_scale - scale_;

    double S = P_scale_ + R;
    if ((innov * innov) / S > 9.0) return;
    double K = P_scale_ / S;
    scale_ += K * innov;
    scale_ = std::max(0.005, std::min(20.0, scale_));
    P_scale_ = (1.0 - K) * P_scale_;
}

void EKFState::updateZUPT() {
    P_scale_ *= 0.99;
    P_td_ *= 0.999;

    // Zero velocity in full EKF state (prevents IMU-propagated drift)
    if (full_initialized_) {
        v_G_ = cv::Mat::zeros(3, 1, CV_64F);
        // Shrink velocity covariance — scale factor applied once to full block
        if (!P_.empty() && P_.rows >= IMU_STATE_DIM) {
            constexpr double shrink = 0.01; // Equivalent to "velocity is near-zero"
            for (int i = 6; i < 9; i++) {
                for (int j = 0; j < P_.cols; j++) {
                    if (j >= 6 && j < 9) {
                        // Velocity-velocity block: scale by shrink
                        P_.at<double>(i, j) *= shrink;
                    } else {
                        // Cross-correlation: scale by sqrt(shrink) to maintain PSD
                        P_.at<double>(i, j) *= std::sqrt(shrink);
                        P_.at<double>(j, i) *= std::sqrt(shrink);
                    }
                }
            }
        }
    }
}

// DEAD CODE: checkConsistency — never called
// double EKFState::checkConsistency(double camera_disp, double step_disp) const { ... }

// DEAD CODE: getScaleStd — never called
// double EKFState::getScaleStd() const { ... }

// ── Clone Management ────────────────────────────────────────────────────────

void EKFState::addClone(const cv::Mat& R_GtoC, const cv::Mat& p_G, int64_t timestamp_ns) {
    CameraPose pose;
    pose.R_GtoC = R_GtoC.clone();
    pose.p_G = p_G.clone();
    pose.timestamp_ns = timestamp_ns;
    pose.state_id = next_state_id_++;
    pose.R_FEJ = R_GtoC.clone();
    pose.p_FEJ = p_G.clone();

    if (!global_fej_initialized_) {
        global_first_estimate_R_ = R_GtoC.clone();
        global_first_estimate_p_ = p_G.clone();
        global_fej_initialized_ = true;
    }

    // Augment covariance if full state is initialized
    if (full_initialized_ && !P_.empty()) {
        int old_dim = P_.rows;
        int new_dim = old_dim + CLONE_DIM;

        cv::Mat P_new = cv::Mat::zeros(new_dim, new_dim, CV_64F);
        // Copy existing covariance
        P_.copyTo(P_new(cv::Range(0, old_dim), cv::Range(0, old_dim)));

        // Augmentation Jacobian J_clone: maps IMU state to new clone
        // Clone rotation = IMU rotation -> dθ_clone/dθ_imu = I
        // Clone position = IMU position -> dp_clone/dp_imu = I
        cv::Mat J = cv::Mat::zeros(CLONE_DIM, old_dim, CV_64F);
        // dθ_c / dθ = I(3x3)
        cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);
        I3.copyTo(J(cv::Range(0,3), cv::Range(0,3)));
        // dp_c / dp = I(3x3) (position index = 12 in IMU state)
        if (old_dim >= IMU_STATE_DIM) {
            I3.copyTo(J(cv::Range(3,6), cv::Range(12,15)));
        }

        // New clone covariance blocks
        cv::Mat P_sub = cv::Mat(P_(cv::Range(0, std::min(old_dim, J.cols)),
                                   cv::Range(0, std::min(old_dim, J.cols))));
        cv::Mat P_cc = cv::Mat(J * P_sub * J.t());
        P_cc.copyTo(P_new(cv::Range(old_dim, new_dim), cv::Range(old_dim, new_dim)));

        // Cross-correlation: P_xc = P_old * J^T
        cv::Mat P_old_sub = cv::Mat(P_(cv::Range(0, old_dim),
                                       cv::Range(0, std::min(old_dim, J.cols))));
        cv::Mat P_xc = cv::Mat(P_old_sub * J.t());
        P_xc.copyTo(P_new(cv::Range(0, old_dim), cv::Range(old_dim, new_dim)));
        cv::Mat P_cx = cv::Mat(P_xc.t());
        P_cx.copyTo(P_new(cv::Range(old_dim, new_dim), cv::Range(0, old_dim)));

        P_ = P_new;
    }

    window_.push_back(std::move(pose));

    // Prune if over limit
    if (static_cast<int>(window_.size()) > MAX_CLONES) {
        marginalizeOldestClone();
    }
}

void EKFState::pruneWindow(size_t max_poses) {
    while (window_.size() > max_poses) {
        marginalizeOldestClone();
    }
}

void EKFState::marginalizeOldestClone() {
    if (window_.empty()) return;

    if (full_initialized_ && !P_.empty() && P_.rows > IMU_STATE_DIM) {
        // Schur complement marginalization of the oldest clone
        int marg_idx = IMU_STATE_DIM;  // First clone starts at index 15
        int marg_dim = CLONE_DIM;
        int total_dim = P_.rows;
        int remain_dim = total_dim - marg_dim;

        if (remain_dim > 0) {
            // Build index mapping: [0..marg_idx-1, marg_idx+marg_dim..total_dim-1]
            cv::Mat P_new = cv::Mat::zeros(remain_dim, remain_dim, CV_64F);

            // Top-left: IMU block (unchanged)
            if (marg_idx > 0) {
                P_(cv::Range(0, marg_idx), cv::Range(0, marg_idx))
                    .copyTo(P_new(cv::Range(0, marg_idx), cv::Range(0, marg_idx)));
            }

            int after = marg_idx + marg_dim;
            int after_dim = total_dim - after;

            if (after_dim > 0) {
                // Top-right: IMU to remaining clones
                P_(cv::Range(0, marg_idx), cv::Range(after, total_dim))
                    .copyTo(P_new(cv::Range(0, marg_idx),
                                 cv::Range(marg_idx, remain_dim)));
                // Bottom-left: remaining clones to IMU
                P_(cv::Range(after, total_dim), cv::Range(0, marg_idx))
                    .copyTo(P_new(cv::Range(marg_idx, remain_dim),
                                 cv::Range(0, marg_idx)));
                // Bottom-right: remaining clones to remaining clones
                P_(cv::Range(after, total_dim), cv::Range(after, total_dim))
                    .copyTo(P_new(cv::Range(marg_idx, remain_dim),
                                 cv::Range(marg_idx, remain_dim)));
            }

            P_ = P_new;
        }
    }

    window_.pop_front();
}

// ── MSCKF EKF Update ────────────────────────────────────────────────────────

void EKFState::applyMSCKFUpdate(const cv::Mat& H, const cv::Mat& res,
                                 const cv::Mat& R_noise) {
    if (!full_initialized_ || P_.empty()) return;
    int state_dim = P_.rows;
    if (H.cols != state_dim) return;

    // S = H * P * H^T + R
    cv::Mat S = H * P_ * H.t() + R_noise;

    // K = P * H^T * S^{-1}
    cv::Mat S_inv;
    if (!cv::invert(S, S_inv, cv::DECOMP_CHOLESKY)) {
        if (!cv::invert(S, S_inv, cv::DECOMP_SVD)) return;
    }

    cv::Mat K = P_ * H.t() * S_inv;

    // State correction: dx = K * res
    cv::Mat dx = K * res;

    // Apply IMU state correction
    if (dx.rows >= IMU_STATE_DIM) {
        // Rotation correction via Rodrigues
        cv::Mat dtheta = dx(cv::Range(0,3), cv::Range::all());
        cv::Mat dR;
        cv::Rodrigues(dtheta, dR);
        R_GtoI_ = dR * R_GtoI_;

        // Bias corrections
        b_g_ += dx(cv::Range(3,6), cv::Range::all());
        v_G_ += dx(cv::Range(6,9), cv::Range::all());
        b_a_ += dx(cv::Range(9,12), cv::Range::all());
        p_G_ += dx(cv::Range(12,15), cv::Range::all());
    }

    // Apply clone corrections
    int clone_start = IMU_STATE_DIM;
    for (size_t i = 0; i < window_.size(); i++) {
        int idx = clone_start + static_cast<int>(i) * CLONE_DIM;
        if (idx + CLONE_DIM > dx.rows) break;

        cv::Mat dtheta_c = dx(cv::Range(idx, idx+3), cv::Range::all());
        cv::Mat dp_c = dx(cv::Range(idx+3, idx+6), cv::Range::all());

        cv::Mat dR_c;
        cv::Rodrigues(dtheta_c, dR_c);
        window_[i].R_GtoC = dR_c * window_[i].R_GtoC;
        window_[i].p_G += dp_c;
    }

    // Covariance update: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T (Joseph form)
    cv::Mat I_KH = cv::Mat::eye(state_dim, state_dim, CV_64F) - K * H;
    P_ = I_KH * P_ * I_KH.t() + K * R_noise * K.t();

    // Enforce symmetry
    P_ = (P_ + P_.t()) * 0.5;

    LOGI("MSCKF update applied: max_correction=%.4f", cv::norm(dx, cv::NORM_INF));
}

// ── Clone Accessors ─────────────────────────────────────────────────────────

bool EKFState::getClonePose(int state_id, cv::Mat& R_GtoC, cv::Mat& p_G) const {
    for (const auto& pose : window_) {
        if (pose.state_id == state_id) {
            R_GtoC = pose.R_GtoC.clone();
            p_G = pose.p_G.clone();
            return true;
        }
    }
    return false;
}

bool EKFState::getCloneFEJ(int state_id, cv::Mat& R_FEJ, cv::Mat& p_FEJ) const {
    for (const auto& pose : window_) {
        if (pose.state_id == state_id) {
            R_FEJ = pose.R_FEJ.clone();
            p_FEJ = pose.p_FEJ.clone();
            return true;
        }
    }
    return false;
}

int EKFState::getCloneCovIdx(int state_id) const {
    for (size_t i = 0; i < window_.size(); i++) {
        if (window_[i].state_id == state_id) {
            return IMU_STATE_DIM + static_cast<int>(i) * CLONE_DIM;
        }
    }
    return -1;
}

int EKFState::getStateDim() const {
    if (!full_initialized_ || P_.empty()) return 0;
    return P_.rows;
}

int EKFState::getLatestCloneId() const {
    if (window_.empty()) return -1;
    return window_.back().state_id;
}

// DEAD CODE: getFEJ — only called from UpdaterMSCKF which is disabled
// void EKFState::getFEJ(int state_id, cv::Mat& R_fej, cv::Mat& p_fej) const { ... }

// ── Online Temporal Calibration ─────────────────────────────────────────────

void EKFState::setTimeOffset(double td_seconds) {
    t_offset_cam_imu_ = std::max(-0.1, std::min(0.1, td_seconds));
    P_td_ = 0.002 * 0.002;
    LOGI("EKF: Time offset warm-started to %.3fms (std=2.0ms)", td_seconds * 1000.0);
}

// DEAD CODE: updateTemporal — never called
// void EKFState::updateTemporal(double observed_scale, double confidence, double H_td) { ... }
