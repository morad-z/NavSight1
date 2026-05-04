#include "EKFState.h"
#include "EventCounters.h"
#include <cmath>
#include <algorithm>
#include <chrono>
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
    // Plan Step 6 (ADR-012): hold snapshot_mutex_ across the window clear so
    // a concurrent BA reader cannot iterate a half-cleared deque.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

    // Plan Step 3a (ADR-008): MSCKF damping / Huber state
    msckf_frames_since_call_ = MSCKF_QUIET_PROPAGATION;  // start "cold" → 0.5x first call
    msckf_damping_step_ = 0;
    msckf_huber_rejected_count_ = 0;

    // Plan Step 3b (ADR-009): drop all SLAM features on reset.
    slam_features_.clear();
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

    // Plan Step 3a (ADR-008): tick the "frames since last MSCKF" counter so
    // the damping ramp resets after a quiet period (≥ MSCKF_QUIET_PROPAGATION
    // propagation steps without an MSCKF correction).
    if (msckf_frames_since_call_ < MSCKF_QUIET_PROPAGATION + 1) {
        msckf_frames_since_call_++;
    }
    if (msckf_frames_since_call_ >= MSCKF_QUIET_PROPAGATION) {
        msckf_damping_step_ = 0;  // cold start on the next MSCKF call
    }

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
    // Plan Step 6 (ADR-012): hold the snapshot mutex around the whole splice
    // so the BA worker thread cannot observe a torn `window_` mid-augmentation.
    // Internal `marginalizeOldestClone` call below MUST go through the
    // private no-lock variant to avoid deadlocking against this same lock.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

    // Augment covariance if full state is initialized.
    //
    // Plan Step 3b (ADR-009): SLAM features sit at the END of P_, AFTER all
    // clone blocks. The new clone must be spliced BEFORE the SLAM block,
    // not appended at the end — otherwise SLAM features lose their cross-
    // correlations with the IMU state on every clone augmentation, which
    // is the failure mode ADR-006 documents (5–11 m teleportations after
    // a clone churn). Layout:
    //
    //   old:  [ IMU(15) | C0..C{K-1}(6 each) | S0..S{N-1}(5 each) ]
    //   new:  [ IMU(15) | C0..C{K-1}(6 each) | C_new(6) | S0..S{N-1}(5 each) ]
    //
    // The splice strategy: copy the pre-SLAM block into P_new[0..clone_end),
    // copy the SLAM block into P_new[clone_end+CLONE_DIM..end), zero the
    // CLONE_DIM rows/cols inserted in between, then build the new clone's
    // own covariance and cross-terms via the augmentation Jacobian.
    if (full_initialized_ && !P_.empty()) {
        const int old_dim   = P_.rows;
        const int slam_n    = static_cast<int>(slam_features_.size());
        const int slam_block = slam_n * SLAM_FEATURE_DIM;
        const int clone_end = old_dim - slam_block;     // pre-SLAM end
        const int new_dim   = old_dim + CLONE_DIM;

        cv::Mat P_new = cv::Mat::zeros(new_dim, new_dim, CV_64F);

        // Copy the [IMU | clones] block into the top-left of P_new.
        if (clone_end > 0) {
            P_(cv::Range(0, clone_end), cv::Range(0, clone_end))
                .copyTo(P_new(cv::Range(0, clone_end), cv::Range(0, clone_end)));
        }
        // Copy the SLAM block (and its cross-terms) into the bottom-right,
        // shifted down/right by CLONE_DIM.
        if (slam_block > 0) {
            // SLAM-SLAM block.
            P_(cv::Range(clone_end, old_dim), cv::Range(clone_end, old_dim))
                .copyTo(P_new(cv::Range(clone_end + CLONE_DIM, new_dim),
                              cv::Range(clone_end + CLONE_DIM, new_dim)));
            // Cross [IMU+clones] ↔ SLAM: shift only the columns.
            if (clone_end > 0) {
                P_(cv::Range(0, clone_end), cv::Range(clone_end, old_dim))
                    .copyTo(P_new(cv::Range(0, clone_end),
                                  cv::Range(clone_end + CLONE_DIM, new_dim)));
                P_(cv::Range(clone_end, old_dim), cv::Range(0, clone_end))
                    .copyTo(P_new(cv::Range(clone_end + CLONE_DIM, new_dim),
                                  cv::Range(0, clone_end)));
            }
        }

        // Build the augmentation Jacobian J (CLONE_DIM x clone_end). Maps
        // [IMU + existing_clones] → new clone's δθ_c, δp_c. Note: the
        // Jacobian columns DO NOT extend into the SLAM block — the new
        // clone is a function of the IMU state only, and the SLAM block
        // sits to the right of where the new clone is being inserted, so
        // there is nothing to read from there even on the source side.
        cv::Mat J = cv::Mat::zeros(CLONE_DIM, clone_end, CV_64F);
        cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);
        I3.copyTo(J(cv::Range(0,3), cv::Range(0,3)));        // dθ_c/dθ = I
        if (clone_end >= IMU_STATE_DIM) {
            I3.copyTo(J(cv::Range(3,6), cv::Range(12,15))); // dp_c/dp = I
        }

        // Insertion column range for the new clone in P_new.
        const int ins_lo = clone_end;
        const int ins_hi = clone_end + CLONE_DIM;

        // New clone's own covariance: J * P_old(pre-SLAM) * J^T.
        if (clone_end > 0) {
            cv::Mat P_pre = P_(cv::Range(0, clone_end), cv::Range(0, clone_end));
            cv::Mat P_cc  = J * P_pre * J.t();
            P_cc.copyTo(P_new(cv::Range(ins_lo, ins_hi),
                              cv::Range(ins_lo, ins_hi)));

            // Cross-correlation new-clone ↔ pre-SLAM block: P_pre * J^T.
            cv::Mat P_xc = P_pre * J.t();
            P_xc.copyTo(P_new(cv::Range(0, clone_end),
                              cv::Range(ins_lo, ins_hi)));
            cv::Mat P_cx = P_xc.t();
            P_cx.copyTo(P_new(cv::Range(ins_lo, ins_hi),
                              cv::Range(0, clone_end)));

            // Cross-correlation new-clone ↔ SLAM block: J * P_pre_to_SLAM
            // (the SLAM features were correlated with the IMU/clones via
            // the same dynamics that generated the new clone, so the new
            // clone inherits that cross-correlation through J).
            if (slam_block > 0) {
                cv::Mat P_pre_slam = P_(cv::Range(0, clone_end),
                                        cv::Range(clone_end, old_dim));
                cv::Mat P_cs = J * P_pre_slam;     // CLONE_DIM x slam_block
                P_cs.copyTo(P_new(cv::Range(ins_lo, ins_hi),
                                  cv::Range(clone_end + CLONE_DIM, new_dim)));
                cv::Mat P_sc = P_cs.t();
                P_sc.copyTo(P_new(cv::Range(clone_end + CLONE_DIM, new_dim),
                                  cv::Range(ins_lo, ins_hi)));
            }
        }

        P_ = P_new;
    }

    window_.push_back(std::move(pose));

    // Prune if over limit. We already hold snapshot_mutex_ at this point
    // (taken at the top of addClone), so call the no-lock variant.
    if (static_cast<int>(window_.size()) > MAX_CLONES) {
        marginalizeOldestCloneNoLock();
    }
}

void EKFState::pruneWindow(size_t max_poses) {
    // Plan Step 6 (ADR-012): single lock around the whole prune loop. Calling
    // the public marginalizeOldestClone in a loop would re-acquire the
    // snapshot mutex on every iteration; one outer lock + the no-lock body is
    // both cheaper and ensures the BA reader observes a coherent post-prune
    // window rather than possibly mid-prune.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    while (window_.size() > max_poses) {
        marginalizeOldestCloneNoLock();
    }
}

void EKFState::marginalizeOldestClone() {
    // Plan Step 6 (ADR-012): public entry takes the lock; the body is the
    // no-lock variant so the addClone / pruneWindow paths can call it
    // directly without re-locking.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    marginalizeOldestCloneNoLock();
}

void EKFState::marginalizeOldestCloneNoLock() {
    if (window_.empty()) return;

    if (full_initialized_ && !P_.empty() && P_.rows > IMU_STATE_DIM) {
        // Plan Step 3b (ADR-009): the oldest clone sits at rows/cols
        // [IMU_STATE_DIM, IMU_STATE_DIM + CLONE_DIM). The SLAM block sits at
        // the END of P_. Drop only the clone rows/cols by copy-splice — IMU,
        // remaining clones, and SLAM rows/cols all keep their cross-terms
        // intact. (We do NOT zero out the marginalised clone's contribution
        // to the remaining state via Schur — for the OpenVINS-style sliding
        // window the discard-only "drop" is the standard, since the clone
        // has no measurements yet not folded into IMU/SLAM cross-terms.)
        const int marg_idx   = IMU_STATE_DIM;  // First clone starts at 15
        const int marg_dim   = CLONE_DIM;
        const int total_dim  = P_.rows;
        const int remain_dim = total_dim - marg_dim;

        if (remain_dim > 0) {
            cv::Mat P_new = cv::Mat::zeros(remain_dim, remain_dim, CV_64F);

            // IMU-IMU block (unchanged).
            if (marg_idx > 0) {
                P_(cv::Range(0, marg_idx), cv::Range(0, marg_idx))
                    .copyTo(P_new(cv::Range(0, marg_idx), cv::Range(0, marg_idx)));
            }

            const int after     = marg_idx + marg_dim;
            const int after_dim = total_dim - after;

            if (after_dim > 0) {
                // IMU ↔ {remaining clones + SLAM}.
                P_(cv::Range(0, marg_idx), cv::Range(after, total_dim))
                    .copyTo(P_new(cv::Range(0, marg_idx),
                                  cv::Range(marg_idx, remain_dim)));
                P_(cv::Range(after, total_dim), cv::Range(0, marg_idx))
                    .copyTo(P_new(cv::Range(marg_idx, remain_dim),
                                  cv::Range(0, marg_idx)));
                // {remaining clones + SLAM} self block.
                P_(cv::Range(after, total_dim), cv::Range(after, total_dim))
                    .copyTo(P_new(cv::Range(marg_idx, remain_dim),
                                  cv::Range(marg_idx, remain_dim)));
            }

            P_ = P_new;
        }
    }

    // SLAM features that were anchored at the marginalised clone are now
    // un-anchored. We drop them — keeping them with a stale anchor would
    // make the FEJ Jacobian reference a pose that no longer exists in the
    // window, and the chi² test would diverge. This is the conservative
    // choice; a later optimisation could re-anchor to a surviving clone.
    //
    // Order matters: the P_ splice above already shifted the SLAM block
    // up by CLONE_DIM, so `slamBlockStart()` (which uses window_.size())
    // is wrong until we pop the front clone. Pop first, then call
    // removeSlamFeature in reverse-slot order so erase() doesn't shift
    // the remaining slots out from under the loop.
    int dropped_id = -1;
    if (!window_.empty()) dropped_id = window_.front().state_id;
    window_.pop_front();
    if (dropped_id >= 0) {
        for (int slot = static_cast<int>(slam_features_.size()) - 1; slot >= 0; --slot) {
            if (slam_features_[slot].anchor_clone_id == dropped_id) {
                removeSlamFeature(slot);
            }
        }
    }
}

// ── MSCKF EKF Update ────────────────────────────────────────────────────────

double EKFState::computeMSCKFDampingFactor() const {
    // Linear ramp 0.5 → 1.0 over MSCKF_DAMPING_RAMP_FRAMES calls.
    // call 0 → 0.5, call 1 → 0.6, … call 5+ → 1.0.
    int s = msckf_damping_step_;
    if (s < 0) s = 0;
    if (s >= MSCKF_DAMPING_RAMP_FRAMES) return 1.0;
    return 0.5 + 0.1 * static_cast<double>(s);
}

void EKFState::applyMSCKFUpdate(const cv::Mat& H, const cv::Mat& res,
                                 const cv::Mat& R_noise) {
    if (!full_initialized_ || P_.empty()) return;
    int state_dim = P_.rows;
    if (H.cols != state_dim) return;
    if (H.rows == 0 || res.rows != H.rows) return;

    auto t_amu_start = std::chrono::steady_clock::now();

    // ── Plan Step 3a (ADR-008): per-residual Huber kernel ────────────────
    // δ ≈ 2.4477 = √χ²(0.95, 2 dof). For each row i compute the normalised
    // residual m_i = |r_i| / sqrt(S_ii), with
    //     S = H*P*H^T + R_noise.
    // Weights:
    //   m_i ≤ δ          : w = 1     (full influence)
    //   δ < m_i < 3δ     : w = δ/m_i (linear de-weighting, classic Huber)
    //   m_i ≥ 3δ         : w = 0     (hard reject — outlier kill)
    //
    // The weights are folded into H and res (row scaling) so the downstream
    // S/K/dx computation runs unmodified. This is mathematically the IRLS
    // form — equivalent (up to weight=0) to scaling R_noise[i,i] by
    // 1/w² but cleaner when weights collapse to zero.
    cv::Mat H_w = H.clone();
    cv::Mat res_w = res.clone();
    msckf_huber_rejected_count_ = 0;
    {
        cv::Mat S_pre = H * P_ * H.t() + R_noise;
        const double delta = MSCKF_HUBER_DELTA;
        const double hard_reject = 3.0 * delta;
        int huber_dampened = 0;
        for (int i = 0; i < H.rows; i++) {
            double s_ii = S_pre.at<double>(i, i);
            if (s_ii <= 1e-12) continue;
            double m = std::abs(res.at<double>(i, 0)) / std::sqrt(s_ii);
            double w = 1.0;
            if (m >= hard_reject) {
                w = 0.0;
                msckf_huber_rejected_count_++;
            } else if (m > delta) {
                w = delta / m;
                huber_dampened++;
            }
            if (w != 1.0) {
                H_w.row(i) *= w;
                res_w.at<double>(i, 0) *= w;
            }
        }
        if (msckf_huber_rejected_count_ > 0 || huber_dampened > 0) {
            LOGI("MSCKF Huber: rejected=%d dampened=%d total_rows=%d (δ=%.3f)",
                 msckf_huber_rejected_count_, huber_dampened, H.rows, delta);
        }
    }

    // S = H_w * P * H_w^T + R
    cv::Mat S = H_w * P_ * H_w.t() + R_noise;

    // K = P * H_w^T * S^{-1}
    cv::Mat S_inv;
    if (!cv::invert(S, S_inv, cv::DECOMP_CHOLESKY)) {
        if (!cv::invert(S, S_inv, cv::DECOMP_SVD)) return;
    }

    cv::Mat K = P_ * H_w.t() * S_inv;

    // State correction: dx = K * res_w
    cv::Mat dx = K * res_w;

    // ── Plan Step 3a (ADR-008): position-correction damping ──────────────
    // Scale δp (rows 12..14 of the IMU error-state) by the ramp factor so
    // the polyline does not jump on the first MSCKF call after a quiet
    // period. Velocity (6..8), attitude (0..2), and bias (3..5, 9..11)
    // corrections are unchanged. Clone corrections are also unchanged —
    // damping is intentionally local to the world-frame body position.
    double damping = computeMSCKFDampingFactor();
    if (dx.rows >= 15 && damping < 1.0) {
        for (int i = 12; i < 15; i++) {
            dx.at<double>(i, 0) *= damping;
        }
    }

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

    // ── Plan Step 3b (ADR-009): apply SLAM-feature mean corrections ─────
    // The SLAM block sits at the end of P_/dx. Each SLAM feature carries
    // 5 DOFs `[α, β, ρ, pad0, pad1]`; we apply the dx update to the
    // active 3 (α, β, ρ) and to the 2 pad rows (which any sane filter
    // will leave near zero — they have no measurement coupling). The
    // application is purely additive on the inverse-depth state: SO(3)
    // wrapping is not needed because (α, β, ρ) lie in flat ℝ³.
    {
        const int slam_start = IMU_STATE_DIM
                             + static_cast<int>(window_.size()) * CLONE_DIM;
        for (size_t s = 0; s < slam_features_.size(); s++) {
            const int idx = slam_start + static_cast<int>(s) * SLAM_FEATURE_DIM;
            if (idx + SLAM_FEATURE_DIM > dx.rows) break;
            for (int k = 0; k < SLAM_FEATURE_DIM; k++) {
                slam_features_[s].state.at<double>(k, 0) +=
                    dx.at<double>(idx + k, 0);
            }
        }
    }

    // Covariance update: P = (I - K*H_w) * P * (I - K*H_w)^T + K*R*K^T (Joseph form)
    cv::Mat I_KH = cv::Mat::eye(state_dim, state_dim, CV_64F) - K * H_w;
    P_ = I_KH * P_ * I_KH.t() + K * R_noise * K.t();

    // Enforce symmetry
    P_ = (P_ + P_.t()) * 0.5;

    // Plan Step 3a (ADR-008): advance the damping schedule and reset the
    // quiet-period counter so the next propagateIMU step starts fresh.
    msckf_frames_since_call_ = 0;
    if (msckf_damping_step_ < MSCKF_DAMPING_RAMP_FRAMES) {
        msckf_damping_step_++;
    }

    auto t_amu_end = std::chrono::steady_clock::now();
    long long t_amu_us = std::chrono::duration_cast<std::chrono::microseconds>(
        t_amu_end - t_amu_start).count();
    // Gate the per-call PERF line behind a slow-call threshold. With SLAM
    // active applyMSCKFUpdate fires 2–4× per camera frame; logging every
    // call at 30 Hz floods logcat (60–120 lines/sec from this function
    // alone) which itself adds latency on slow log drivers. 500 µs is
    // well above the steady-state cost (~115 µs measured 2026-05-04 in
    // the perf-fix pass) so the gate stays silent in normal operation
    // and only fires when something is actually slow.
    if (t_amu_us > 500) {
        LOGI("PERF: section=applyMSCKFUpdate us=%lld H_rows=%d state_dim=%d slam=%d clones=%d",
             t_amu_us, H.rows, state_dim,
             (int)slam_features_.size(), (int)window_.size());
    }
    LOGI("MSCKF update applied: max_correction=%.4f damping=%.2f huber_rejected=%d",
         cv::norm(dx, cv::NORM_INF), damping, msckf_huber_rejected_count_);
    // EventCounters: one applied update = one log line. Sum the huber
    // rejections across every applied update so the total tells us how
    // many measurement rows the robust kernel rejected over the recording.
    navsight::eventCounters().msckf_update_lines.fetch_add(
        1, std::memory_order_relaxed);
    if (msckf_huber_rejected_count_ > 0) {
        navsight::eventCounters().msckf_huber_rejected_sum.fetch_add(
            static_cast<long long>(msckf_huber_rejected_count_),
            std::memory_order_relaxed);
    }
}

// ── Step 4: VIO/PDR/Yaw Measurement Updates ─────────────────────────────────

bool EKFState::updateRelativePose(const cv::Mat& t_world_metric,
                                   int clone_id,
                                   double var_t) {
    if (!full_initialized_ || P_.empty()) return false;
    if (t_world_metric.rows != 3 || t_world_metric.cols != 1) return false;

    int clone_cov_idx = getCloneCovIdx(clone_id);
    if (clone_cov_idx < 0) return false;

    cv::Mat R_clone, p_clone;
    if (!getClonePose(clone_id, R_clone, p_clone)) return false;

    int dim = P_.rows;
    if (clone_cov_idx + CLONE_DIM > dim) return false;

    // Predicted relative position in world frame: z_pred = p_G - p_clone.
    cv::Mat z_pred = p_G_ - p_clone;
    cv::Mat res = t_world_metric - z_pred;

    // H (3 x dim): +I on δp current (cols 12..14), -I on δp_clone
    // (clone_cov_idx + 3 .. +5). δp_clone is the second 3-block of the
    // clone slot (CLONE_DIM = [δθ_c(3), δp_c(3)]).
    cv::Mat H = cv::Mat::zeros(3, dim, CV_64F);
    for (int i = 0; i < 3; i++) {
        H.at<double>(i, 12 + i) = 1.0;             // ∂z/∂δp_current
        H.at<double>(i, clone_cov_idx + 3 + i) = -1.0;  // ∂z/∂δp_clone
    }

    cv::Mat R_noise = cv::Mat::eye(3, 3, CV_64F) * std::max(var_t, 1e-6);
    applyMSCKFUpdate(H, res, R_noise);
    return true;
}

bool EKFState::updateRelativeRotation(const cv::Mat& R_meas_body,
                                      double sigma_axis_sq,
                                      int clone_id) {
    if (!full_initialized_ || P_.empty()) return false;
    if (R_meas_body.rows != 3 || R_meas_body.cols != 3 ||
        R_meas_body.type() != CV_64F) {
        return false;
    }

    int clone_cov_idx = getCloneCovIdx(clone_id);
    if (clone_cov_idx < 0) return false;

    cv::Mat R_clone, p_clone;
    if (!getClonePose(clone_id, R_clone, p_clone)) return false;

    int dim = P_.rows;
    if (clone_cov_idx + CLONE_DIM > dim) return false;
    if (R_clone.rows != 3 || R_clone.cols != 3) return false;

    // Predicted body-frame rotation from clone to current:
    //   R_pred = R_GtoI_(current) * R_GtoC(clone).t()
    // R_GtoI_ takes world→current-body. R_clone takes world→clone-body
    // (treating the clone as the body pose at clone-time). So
    // R_clone.t() takes clone-body→world, and the product takes
    // clone-body→current-body, matching the convention of R_meas_body.
    cv::Mat R_pred = R_GtoI_ * R_clone.t();

    // Residual on SO(3) in the Lie algebra:  r = log(R_meas * R_pred^T)
    cv::Mat R_err = R_meas_body * R_pred.t();
    cv::Mat r_vec;
    cv::Rodrigues(R_err, r_vec);     // 3x1
    if (r_vec.rows != 3 || r_vec.cols != 1) return false;

    // Wrap each axis to [-π, π] (Rodrigues already returns within ±π,
    // but be defensive against numerical drift on near-π rotations).
    for (int i = 0; i < 3; i++) {
        double v = r_vec.at<double>(i, 0);
        while (v >  M_PI) v -= 2.0 * M_PI;
        while (v < -M_PI) v += 2.0 * M_PI;
        r_vec.at<double>(i, 0) = v;
    }

    // Jacobian (3 x dim) for the small-angle relative-rotation residual.
    // For δθ (current body) and δθ_c (clone body) the linearisation is
    //     r ≈ δθ - R_meas_body * δθ_c
    // i.e. +I on the current-pose δθ block (cols 0..2) and -R_meas_body
    // on the clone's δθ_c block (the first 3-vector of CLONE_DIM at
    // clone_cov_idx + 0..2).
    cv::Mat H = cv::Mat::zeros(3, dim, CV_64F);
    for (int i = 0; i < 3; i++) {
        H.at<double>(i, i) = 1.0;
        for (int j = 0; j < 3; j++) {
            H.at<double>(i, clone_cov_idx + j) = -R_meas_body.at<double>(i, j);
        }
    }

    cv::Mat R_noise = cv::Mat::eye(3, 3, CV_64F) * std::max(sigma_axis_sq, 1e-8);
    applyMSCKFUpdate(H, r_vec, R_noise);
    return true;
}

// Plan Step 7 (ADR-013 §"Correction injection — absolute pose path"):
// World-frame absolute pose measurement. Independent of the clone window —
// loop-closure's matched keyframe is almost always older than the EKF
// sliding window (temporal exclusion 30 s vs. ~5–10 s clone window), so
// the relative-pose channels cannot reach it. This channel composes the
// 6-DOF residual on the IMU state directly:
//
//   r_R = log(target_R_world_imu * R_GtoI_.t())   (rotation residual,
//                                                   3 DOF, world frame)
//   r_p = target_p_world - p_G_                   (translation residual)
//
// Sign convention (matches updateRelativeRotation, line 760+):
//   H_R = +I on δθ (rows 0..2)  — derivation below
//   H_p = +I on δp (rows 12..14)
//
// Why H_R = +I (not −I): the EKF uses the OpenVINS / MSCKF error-state
// convention R_estimate = exp(−[δθ]×) · R_true with the state update
// applied as R_GtoI_ ← Rodrigues(dθ) · R_GtoI_ (line 613). Substituting
// R_GtoI_(true) = exp([δθ]×) · R_GtoI_(estimate) into r_R, with R_target
// being noise-free truth, gives r_R ≈ +δθ — so the Kalman gain with
// H = +I drives dθ in the direction that zeroes the residual. This is
// the same derivation that makes updateRelativeRotation's H = +I correct
// for its current-pose δθ block. A reviewer reading the code as if it
// used the right-perturbation convention would expect H = −I; we have
// chosen left-perturbation in the body frame, so +I is correct.
//
// The χ² gate is intentionally wide (≈ 22.5 = χ²(0.999, 6 DOF)). Loop
// closures are tolerant by design: damping fades a wrong match across
// 10 frames; a tight χ² would defeat that. We only reject *wildly*
// wrong matches before damping kicks in.
bool EKFState::updateAbsolutePose(const cv::Mat& target_R_world_imu,
                                  const cv::Mat& target_p_world,
                                  double sigma_axis_sq_R,
                                  double var_p) {
    if (!full_initialized_ || P_.empty()) return false;
    if (target_R_world_imu.rows != 3 || target_R_world_imu.cols != 3 ||
        target_R_world_imu.type() != CV_64F) {
        return false;
    }
    if (target_p_world.rows != 3 || target_p_world.cols != 1 ||
        target_p_world.type() != CV_64F) {
        return false;
    }

    const int dim = P_.rows;
    if (dim < IMU_STATE_DIM) return false;

    // ── Residual (6×1) ────────────────────────────────────────────────
    // Rotation residual on SO(3), expressed in world frame, exactly the
    // pattern updateRelativeRotation uses. Both R_GtoI_ and the target
    // are world→imu; their composition target * R_GtoI_.t() lives on
    // SO(3) and Rodrigues linearises it to a 3-vector axis-angle.
    //   r_R = log(target * R_pred.t())
    // With H_R = +I on δθ (rows 0..2) the Kalman gain produces a
    // correction dθ that, applied as dR = Rodrigues(dθ); R_GtoI_ =
    // dR * R_GtoI_, drives R_GtoI_ toward target. Sign-cross-checked
    // against updateRelativeRotation's identical structure.
    cv::Mat R_err = target_R_world_imu * R_GtoI_.t();
    cv::Mat r_R;
    cv::Rodrigues(R_err, r_R);  // 3x1
    if (r_R.rows != 3 || r_R.cols != 1) return false;
    // Wrap each axis to [-π, π] (defensive — Rodrigues already returns
    // within ±π, but numerical drift on near-π rotations can leak).
    for (int i = 0; i < 3; i++) {
        double v = r_R.at<double>(i, 0);
        while (v >  M_PI) v -= 2.0 * M_PI;
        while (v < -M_PI) v += 2.0 * M_PI;
        r_R.at<double>(i, 0) = v;
    }

    cv::Mat r_p = target_p_world - p_G_;  // 3x1

    cv::Mat r = cv::Mat::zeros(6, 1, CV_64F);
    for (int i = 0; i < 3; i++) {
        r.at<double>(i, 0)     = r_R.at<double>(i, 0);
        r.at<double>(3 + i, 0) = r_p.at<double>(i, 0);
    }

    // ── Jacobian (6 × dim) ───────────────────────────────────────────
    // IMU error-state layout: [δθ(0..2), δb_g(3..5), δv(6..8),
    // δb_a(9..11), δp(12..14)]. With r_R = log(R_target * R_pred.t())
    // the Jacobian rows are +I on δθ (current rotation) and +I on δp
    // (current position) — same structure updateRelativeRotation uses
    // for its current-pose block, just without the "minus on the clone"
    // that channel needs because we have no clone reference here.
    cv::Mat H = cv::Mat::zeros(6, dim, CV_64F);
    for (int i = 0; i < 3; i++) {
        H.at<double>(i, i)         = 1.0;        // ∂r_R / ∂δθ
        H.at<double>(3 + i, 12 + i) = 1.0;        // ∂r_p / ∂δp
    }

    // ── Measurement noise (6×6) ─────────────────────────────────────
    cv::Mat R_noise = cv::Mat::zeros(6, 6, CV_64F);
    const double sR = std::max(sigma_axis_sq_R, 1e-8);
    const double sp = std::max(var_p, 1e-6);
    for (int i = 0; i < 3; i++) {
        R_noise.at<double>(i, i)         = sR;
        R_noise.at<double>(3 + i, 3 + i) = sp;
    }

    // ── Outer χ² gate ───────────────────────────────────────────────
    // χ²(0.999, 6 DOF) ≈ 22.458. Wide gate: loop closures are
    // intentionally tolerant. A residual that fails this gate is
    // effectively a teleportation request the filter would never
    // recover from; rejecting it preserves the existing state.
    static constexpr double kChi2Threshold = 22.5;
    cv::Mat S = H * P_ * H.t() + R_noise;
    cv::Mat S_inv;
    if (!cv::invert(S, S_inv, cv::DECOMP_CHOLESKY)) {
        if (!cv::invert(S, S_inv, cv::DECOMP_SVD)) return false;
    }
    cv::Mat m_mat = r.t() * S_inv * r;
    const double m2 = m_mat.at<double>(0, 0);
    if (m2 > kChi2Threshold) {
        LOGI("LC_ABS: chi2_reject m=%.3f thresh=%.1f", m2, kChi2Threshold);
        navsight::eventCounters().loop_closure_chi2_rejected.fetch_add(
            1, std::memory_order_relaxed);
        return false;
    }

    LOGI("LC_ABS: r_R=[%.3f %.3f %.3f] r_p=[%.3f %.3f %.3f] "
         "var_R=%.4e var_p=%.4f m2=%.3f applied",
         r_R.at<double>(0, 0), r_R.at<double>(1, 0), r_R.at<double>(2, 0),
         r_p.at<double>(0, 0), r_p.at<double>(1, 0), r_p.at<double>(2, 0),
         sR, sp, m2);

    applyMSCKFUpdate(H, r, R_noise);
    return true;
}

bool EKFState::updateGravityAlignedYaw(double yaw_meas, double var,
                                       double roll, double pitch) {
    if (!full_initialized_ || P_.empty()) return false;
    int dim = P_.rows;

    // Predicted yaw from current R_GtoI_, gravity-aligned via Madgwick
    // roll/pitch. Residual normalized to [-π, π].
    double yaw_pred = getYaw(roll, pitch);
    double res_val = yaw_meas - yaw_pred;
    while (res_val >  M_PI) res_val -= 2.0 * M_PI;
    while (res_val < -M_PI) res_val += 2.0 * M_PI;

    // H_yaw: world-Y axis expressed in body frame. For body-frame error
    // δθ_body, the induced world-yaw change is (R_GtoI_ * e_y_world) · δθ.
    cv::Mat e_y_world = (cv::Mat_<double>(3, 1) << 0.0, 1.0, 0.0);
    cv::Mat h_body = R_GtoI_ * e_y_world;  // 3x1

    cv::Mat H = cv::Mat::zeros(1, dim, CV_64F);
    H.at<double>(0, 0) = h_body.at<double>(0);
    H.at<double>(0, 1) = h_body.at<double>(1);
    H.at<double>(0, 2) = h_body.at<double>(2);
    (void)roll; (void)pitch;  // alignment carried in yaw_pred, not in H

    cv::Mat res = (cv::Mat_<double>(1, 1) << res_val);
    cv::Mat R_noise = (cv::Mat_<double>(1, 1) << std::max(var, 1e-6));
    applyMSCKFUpdate(H, res, R_noise);
    return true;
}

bool EKFState::updatePDRStep(double dx_world, double dz_world, double var) {
    if (!full_initialized_ || P_.empty()) return false;
    int dim = P_.rows;

    // 2-DOF position constraint on world X and Z (Y handled by gravity).
    // Treat as direct observation of δp_x and δp_z (state, not delta).
    // We model the step as observing the absolute world position increment
    // since the last step, but for simplicity here we apply it as a direct
    // constraint on δp relative to current state — caller is responsible
    // for accumulating step displacement into the desired anchor.
    cv::Mat res = (cv::Mat_<double>(2, 1) << dx_world, dz_world);

    cv::Mat H = cv::Mat::zeros(2, dim, CV_64F);
    H.at<double>(0, 12) = 1.0;  // ∂(dx)/∂δp_x
    H.at<double>(1, 14) = 1.0;  // ∂(dz)/∂δp_z

    cv::Mat R_noise = cv::Mat::eye(2, 2, CV_64F) * std::max(var, 1e-6);
    applyMSCKFUpdate(H, res, R_noise);
    return true;
}

double EKFState::getYaw(double roll, double pitch) const {
    cv::Mat Rx, Ry;
    cv::Rodrigues(cv::Vec3d(roll, 0.0, 0.0), Rx);
    cv::Rodrigues(cv::Vec3d(0.0, pitch, 0.0), Ry);
    cv::Mat R_align = Ry * Rx;  // R_phone_to_world
    cv::Mat R_aligned = R_align * R_GtoI_ * R_align.t();
    // Y-up navigation convention (matches IMUPreintegrator::getHeading):
    // yaw is rotation around world-Y axis, North=0, East=+π/2. Extracted
    // from the X-Z components of the aligned rotation:
    //   R_yaw = | cos y, 0, sin y; 0, 1, 0; -sin y, 0, cos y |
    return std::atan2(R_aligned.at<double>(0, 2),
                      R_aligned.at<double>(0, 0));
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

// ──────────────────────────────────────────────────────────────────────────────
// Plan Step 3b (ADR-009): SLAM features in EKF state
// ──────────────────────────────────────────────────────────────────────────────

void EKFState::setSlamIntrinsics(double fx, double fy, double cx, double cy) {
    if (fx > 1.0) slam_fx_ = fx;
    if (fy > 1.0) slam_fy_ = fy;
    slam_cx_ = cx;
    slam_cy_ = cy;
}

int EKFState::slamBlockStart() const {
    return IMU_STATE_DIM + static_cast<int>(window_.size()) * CLONE_DIM;
}

int EKFState::slamFeatureCovIdxInternal(int slot) const {
    if (slot < 0 || slot >= static_cast<int>(slam_features_.size())) return -1;
    return slamBlockStart() + slot * SLAM_FEATURE_DIM;
}

int EKFState::getSlamFeatureCovIdx(int slot) const {
    if (!full_initialized_ || P_.empty()) return -1;
    return slamFeatureCovIdxInternal(slot);
}

int EKFState::getSlamFeatureCount() const {
    return static_cast<int>(slam_features_.size());
}

int EKFState::getSlamFeatureSlot(int feature_id) const {
    for (size_t i = 0; i < slam_features_.size(); ++i) {
        if (slam_features_[i].feature_id == feature_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool EKFState::getSlamFeatureGlobalPosition(int slot, cv::Mat& p_global_out) const {
    if (slot < 0 || slot >= static_cast<int>(slam_features_.size())) return false;
    const auto& f = slam_features_[slot];
    cv::Mat R_anchor, p_anchor;
    if (!getClonePose(f.anchor_clone_id, R_anchor, p_anchor)) {
        return false;
    }
    const double alpha = f.state.at<double>(0, 0);
    const double beta  = f.state.at<double>(1, 0);
    const double rho   = f.state.at<double>(2, 0);
    if (std::abs(rho) < 1e-9) return false;
    cv::Mat p_anchor_pt = (cv::Mat_<double>(3, 1) << alpha / rho, beta / rho, 1.0 / rho);
    // anchor frame → world: world = R_anchor.t() * p_anchor + p_anchor_world
    p_global_out = R_anchor.t() * p_anchor_pt + p_anchor;
    return true;
}

int EKFState::addSlamFeature(int feature_id,
                             const cv::Mat& p_global_init,
                             const CameraPose& anchor_clone) {
    if (!full_initialized_ || P_.empty()) return -1;
    if (p_global_init.rows != 3 || p_global_init.cols != 1 ||
        p_global_init.type() != CV_64F) {
        return -1;
    }
    if (anchor_clone.R_GtoC.rows != 3 || anchor_clone.R_GtoC.cols != 3 ||
        anchor_clone.p_G.rows != 3 || anchor_clone.p_G.cols != 1) {
        return -1;
    }
    if (static_cast<int>(slam_features_.size()) >= MAX_SLAM_FEATURES) {
        // Cap policy: refuse new promotions when full. The caller (Tracker)
        // is responsible for first invoking removeSlamFeature on a stale
        // candidate when it wants to make room.
        return -1;
    }

    // Verify the anchor exists in the current sliding window so the
    // covariance entries we read are real.
    int anchor_cov_idx = getCloneCovIdx(anchor_clone.state_id);
    if (anchor_cov_idx < 0) return -1;

    cv::Mat R_anchor_now, p_anchor_now;
    if (!getClonePose(anchor_clone.state_id, R_anchor_now, p_anchor_now)) {
        return -1;
    }

    // Project the global init point into the anchor's camera frame.
    //     p_anchor_cam = R_anchor * (p_global - p_anchor_world)
    cv::Mat p_anchor_cam = R_anchor_now * (p_global_init - p_anchor_now);
    const double Xc = p_anchor_cam.at<double>(0, 0);
    const double Yc = p_anchor_cam.at<double>(1, 0);
    const double Zc = p_anchor_cam.at<double>(2, 0);
    if (Zc < 0.01) {
        // Behind or too close to the anchor — degenerate inverse depth.
        return -1;
    }

    SlamFeature f;
    f.feature_id      = feature_id;
    f.anchor_clone_id = anchor_clone.state_id;
    f.state           = cv::Mat::zeros(SLAM_FEATURE_DIM, 1, CV_64F);
    f.state.at<double>(0, 0) = Xc / Zc;        // α
    f.state.at<double>(1, 0) = Yc / Zc;        // β
    f.state.at<double>(2, 0) = 1.0 / Zc;       // ρ
    // pad rows 3..4 stay zero — never updated by any measurement.

    f.p_global_FEJ  = p_global_init.clone();
    f.anchor_R_FEJ  = anchor_clone.R_FEJ.empty() ? R_anchor_now.clone()
                                                  : anchor_clone.R_FEJ.clone();
    f.anchor_p_FEJ  = anchor_clone.p_FEJ.empty() ? p_anchor_now.clone()
                                                  : anchor_clone.p_FEJ.clone();

    // Initial 5x5 SLAM covariance.
    //
    // The reasoning: at promotion the (α, β) bearing is essentially as
    // accurate as one pixel of measurement, σ_uv = √pixel_noise_sq /
    // focal. Triangulation noise σ_ρ scales with depth squared and
    // baseline:
    //
    //     σ_α ≈ σ_uv,  σ_β ≈ σ_uv,
    //     σ_ρ ≈ ρ² * σ_uv * sqrt(2) / baseline_estimate
    //
    // We don't know the baseline at promotion time; use a conservative
    // initial σ_ρ = 0.5 * ρ (50% relative depth uncertainty), which the
    // first few updateSlamFeature calls will quickly tighten. The pad
    // rows take a small pinned variance — they hold no information but
    // need to be PSD-positive for the Joseph update to stay PSD.
    cv::Mat P_ff = cv::Mat::zeros(SLAM_FEATURE_DIM, SLAM_FEATURE_DIM, CV_64F);
    const double rho       = 1.0 / Zc;
    const double sigma_uv  = 1.0 / 500.0;  // ≈1px / 500-focal — calibrated trend
    const double sigma_ab  = std::max(sigma_uv, 1e-3);
    const double sigma_rho = std::max(0.5 * rho, 1e-3);
    P_ff.at<double>(0, 0) = sigma_ab  * sigma_ab;
    P_ff.at<double>(1, 1) = sigma_ab  * sigma_ab;
    P_ff.at<double>(2, 2) = sigma_rho * sigma_rho;
    P_ff.at<double>(3, 3) = SLAM_PAD_VARIANCE;
    P_ff.at<double>(4, 4) = SLAM_PAD_VARIANCE;

    // Augment P_: append SLAM_FEATURE_DIM rows/cols at the END of P_, with
    // zero cross-correlation to the existing state. Promotion is treated as
    // a parameter add — the linearisation we'd otherwise propagate through
    // is captured in P_ff above (depth uncertainty dominates and is
    // independent of IMU/clone errors at promotion).
    const int old_dim = P_.rows;
    const int new_dim = old_dim + SLAM_FEATURE_DIM;
    cv::Mat P_new = cv::Mat::zeros(new_dim, new_dim, CV_64F);
    P_.copyTo(P_new(cv::Range(0, old_dim), cv::Range(0, old_dim)));
    P_ff.copyTo(P_new(cv::Range(old_dim, new_dim),
                      cv::Range(old_dim, new_dim)));
    P_ = P_new;

    slam_features_.push_back(std::move(f));
    LOGI("SLAM: promoted feature %d (slot=%zu) at depth %.2fm anchor=%d",
         feature_id, slam_features_.size() - 1, Zc, anchor_clone.state_id);
    return static_cast<int>(slam_features_.size()) - 1;
}

bool EKFState::removeSlamFeature(int slot) {
    if (slot < 0 || slot >= static_cast<int>(slam_features_.size())) return false;
    if (!full_initialized_ || P_.empty()) {
        // No covariance to splice; just drop the metadata.
        slam_features_.erase(slam_features_.begin() + slot);
        return true;
    }

    const int slam_start = slamBlockStart();
    const int marg_idx   = slam_start + slot * SLAM_FEATURE_DIM;
    const int marg_dim   = SLAM_FEATURE_DIM;
    const int total_dim  = P_.rows;

    if (marg_idx + marg_dim > total_dim) return false;

    // Block-Schur complement marginalisation.
    //
    //     P = | P_kk  P_ks |
    //         | P_sk  P_ss |
    //
    // After dropping the SLAM block at position [marg_idx, marg_idx+5):
    //     P_kk' = P_kk - P_ks * inv(P_ss) * P_sk
    //
    // We pull P_ss (5x5), invert, then for each pair (i, j) outside the
    // marginalised range, subtract the contribution. PSD diagonal sniff
    // afterwards; if it fails, fall back to the naïve drop (delete rows/
    // cols only) which is still correct (it's a Schur complement with
    // P_ss → ∞, i.e. infinitely uncertain feature, no info to subtract).
    cv::Mat P_ss = P_(cv::Range(marg_idx, marg_idx + marg_dim),
                      cv::Range(marg_idx, marg_idx + marg_dim)).clone();
    cv::Mat P_ss_inv;
    bool inverted = cv::invert(P_ss, P_ss_inv, cv::DECOMP_CHOLESKY);
    if (!inverted) {
        inverted = cv::invert(P_ss, P_ss_inv, cv::DECOMP_SVD);
    }

    // Build the keep-index ranges and the new dimension.
    const int new_dim = total_dim - marg_dim;
    cv::Mat P_new = cv::Mat::zeros(new_dim, new_dim, CV_64F);

    auto keep_range_lo = [&](int i) -> int {
        return (i < marg_idx) ? i : (i + marg_dim);
    };
    (void)keep_range_lo;  // helper retained for documentation symmetry

    // Pre-Schur copy of the keep block via row/col splicing.
    auto copy_keep_block = [&](const cv::Mat& src, cv::Mat& dst) {
        // Top-left
        if (marg_idx > 0) {
            src(cv::Range(0, marg_idx), cv::Range(0, marg_idx))
                .copyTo(dst(cv::Range(0, marg_idx), cv::Range(0, marg_idx)));
        }
        const int after = marg_idx + marg_dim;
        const int after_dim = total_dim - after;
        if (after_dim > 0) {
            // Top-right
            if (marg_idx > 0) {
                src(cv::Range(0, marg_idx), cv::Range(after, total_dim))
                    .copyTo(dst(cv::Range(0, marg_idx),
                                cv::Range(marg_idx, new_dim)));
                // Bottom-left
                src(cv::Range(after, total_dim), cv::Range(0, marg_idx))
                    .copyTo(dst(cv::Range(marg_idx, new_dim),
                                cv::Range(0, marg_idx)));
            }
            // Bottom-right
            src(cv::Range(after, total_dim), cv::Range(after, total_dim))
                .copyTo(dst(cv::Range(marg_idx, new_dim),
                            cv::Range(marg_idx, new_dim)));
        }
    };

    copy_keep_block(P_, P_new);

    if (inverted) {
        // P_ks: (new_dim x marg_dim). Build it by stacking the rows of P_
        // at columns [marg_idx, marg_idx + marg_dim), skipping rows in
        // the marginalised range.
        cv::Mat P_ks = cv::Mat::zeros(new_dim, marg_dim, CV_64F);
        if (marg_idx > 0) {
            P_(cv::Range(0, marg_idx),
               cv::Range(marg_idx, marg_idx + marg_dim))
                .copyTo(P_ks(cv::Range(0, marg_idx), cv::Range::all()));
        }
        const int after = marg_idx + marg_dim;
        const int after_dim = total_dim - after;
        if (after_dim > 0) {
            P_(cv::Range(after, total_dim),
               cv::Range(marg_idx, marg_idx + marg_dim))
                .copyTo(P_ks(cv::Range(marg_idx, new_dim), cv::Range::all()));
        }
        cv::Mat correction = P_ks * P_ss_inv * P_ks.t();   // new_dim x new_dim
        P_new = P_new - correction;
        // Enforce symmetry
        P_new = (P_new + P_new.t()) * 0.5;

        // PSD diagonal sniff. If any diagonal went negative, the Schur
        // step over-corrected (numerical drift on a near-singular P_ss);
        // fall back to drop-only and warn.
        bool diag_ok = true;
        for (int i = 0; i < P_new.rows; i++) {
            if (P_new.at<double>(i, i) < 0.0) {
                diag_ok = false;
                break;
            }
        }
        if (!diag_ok) {
            LOGE("SLAM remove(slot=%d): Schur produced non-PSD diag — "
                 "falling back to drop-only marginalisation", slot);
            P_new = cv::Mat::zeros(new_dim, new_dim, CV_64F);
            copy_keep_block(P_, P_new);
        }
    }

    P_ = P_new;
    slam_features_.erase(slam_features_.begin() + slot);
    return true;
}

bool EKFState::slamReprojectionJacobian(const SlamFeature& f,
                                        const cv::Mat& clone_R_FEJ,
                                        const cv::Mat& clone_p_FEJ,
                                        const cv::Mat& clone_R_now,
                                        const cv::Mat& clone_p_now,
                                        cv::Mat& H_feature_2x5,
                                        cv::Mat& H_clone_2x6,
                                        cv::Point2d& pred_uv) const {
    // Defensive empty-checks. Crash on 2026-05-04 (Step 6 first real
    // SLAM features after the chicken-and-egg fix): cv::operator- threw
    // here. Either clone_R_now or clone_R_FEJ was empty/wrong shape.
    // Without these guards the throw propagates through __cxa_throw and
    // aborts the process. Returning false here is the documented
    // skip-this-observation contract — caller continues with the
    // remaining observations.
    if (clone_R_now.empty() || clone_R_now.rows != 3 || clone_R_now.cols != 3 ||
        clone_p_now.empty() || clone_p_now.rows != 3 || clone_p_now.cols != 1 ||
        f.state.empty()     || f.state.rows < 3) {
        LOGI("slamReprojectionJacobian: malformed input "
             "(clone_R_now=%dx%d clone_p_now=%dx%d state=%dx%d)",
             clone_R_now.rows, clone_R_now.cols,
             clone_p_now.rows, clone_p_now.cols,
             f.state.rows, f.state.cols);
        return false;
    }

    // Belt-and-braces: catch ANY cv::Exception thrown from inside the
    // function body and convert to the skip-this-observation return
    // value. The defensive checks above and below catch the known
    // empty-input cases, but the function does ~30 cv::Mat operations
    // and a thrown exception from any of them aborts the process via
    // libc++_shared's terminate handler. The function's documented
    // contract is "either succeed or return false"; catch+return false
    // is the only way to honour that contract under unexpected input.
    // The body is moved into an inner lambda solely so the catch can
    // wrap it — semantically identical to the pre-2026-05-04 body.
    int step = 0;  // diagnostic checkpoint — incremented after each Mat op
    try {
    // Layout reminder:
    //   p_anchor_cam = (1/ρ) * (α, β, 1)
    //   p_world      = R_anchor_FEJ.t() * p_anchor_cam + p_anchor_FEJ
    //   p_C          = clone_R * (p_world - clone_p)
    //   z = (p_C.x / p_C.z, p_C.y / p_C.z)
    //
    // The residual builder uses CURRENT anchor & clone poses (so the
    // residual reflects the current state). The Jacobian uses FEJ poses
    // (locked at promotion / first observation).
    step = 1;
    const double alpha = f.state.at<double>(0, 0);
    const double beta  = f.state.at<double>(1, 0);
    const double rho   = f.state.at<double>(2, 0);
    if (std::abs(rho) < 1e-9) return false;

    // Predicted image coords using CURRENT poses (residual side).
    cv::Mat anchor_R_now, anchor_p_now;
    if (!getClonePose(f.anchor_clone_id, anchor_R_now, anchor_p_now)) {
        return false;
    }
    step = 2;
    cv::Mat p_anchor_cam = (cv::Mat_<double>(3, 1) <<
                            alpha / rho, beta / rho, 1.0 / rho);
    step = 3;
    cv::Mat p_world_now  = anchor_R_now.t() * p_anchor_cam + anchor_p_now;
    step = 4;
    cv::Mat p_C_now      = clone_R_now * (p_world_now - clone_p_now);
    step = 5;
    const double zCn = p_C_now.at<double>(2, 0);
    if (zCn < 1e-4) return false;
    // Pixel-space prediction (test fixture uses pixel coords).
    pred_uv.x = slam_fx_ * p_C_now.at<double>(0, 0) / zCn + slam_cx_;
    pred_uv.y = slam_fy_ * p_C_now.at<double>(1, 0) / zCn + slam_cy_;

    // FEJ-side projection for the Jacobian.
    cv::Mat anchor_R_F = f.anchor_R_FEJ.empty() ? anchor_R_now : f.anchor_R_FEJ;
    cv::Mat anchor_p_F = f.anchor_p_FEJ.empty() ? anchor_p_now : f.anchor_p_FEJ;
    cv::Mat clone_R_F  = clone_R_FEJ.empty() ? clone_R_now : clone_R_FEJ;
    cv::Mat clone_p_F  = clone_p_FEJ.empty() ? clone_p_now : clone_p_FEJ;

    step = 6;
    cv::Mat p_world_F = anchor_R_F.t() * p_anchor_cam + anchor_p_F;
    step = 7;
    cv::Mat p_C_F     = clone_R_F * (p_world_F - clone_p_F);
    step = 8;
    const double xCf = p_C_F.at<double>(0, 0);
    const double yCf = p_C_F.at<double>(1, 0);
    const double zCf = p_C_F.at<double>(2, 0);
    if (zCf < 1e-4) return false;
    const double zinv  = 1.0 / zCf;
    const double zinv2 = zinv * zinv;

    // ∂(u_px, v_px) / ∂p_C  =   |fx/z  0    -fx*x/z²|
    //                            |0    fy/z  -fy*y/z²|
    cv::Mat dproj_dpC = (cv::Mat_<double>(2, 3) <<
                         slam_fx_ * zinv, 0.0,             -slam_fx_ * xCf * zinv2,
                         0.0,             slam_fy_ * zinv, -slam_fy_ * yCf * zinv2);

    // ∂p_C / ∂p_world = clone_R_F
    cv::Mat dpC_dpw = clone_R_F;

    // ∂p_world / ∂(α, β, ρ):
    //   p_world = R_a.t() * (1/ρ) * (α, β, 1) + p_a
    //   ∂/∂α = R_a.t() * (1/ρ, 0, 0)^T
    //   ∂/∂β = R_a.t() * (0, 1/ρ, 0)^T
    //   ∂/∂ρ = R_a.t() * (-α/ρ², -β/ρ², -1/ρ²)^T
    const double rho_inv  = 1.0 / rho;
    const double rho_inv2 = rho_inv * rho_inv;
    cv::Mat dpw_dab = cv::Mat::zeros(3, 3, CV_64F);
    step = 9;
    cv::Mat col_a = anchor_R_F.t() * (cv::Mat_<double>(3, 1) << rho_inv, 0.0, 0.0);
    step = 10;
    cv::Mat col_b = anchor_R_F.t() * (cv::Mat_<double>(3, 1) << 0.0, rho_inv, 0.0);
    step = 11;
    cv::Mat col_r = anchor_R_F.t() * (cv::Mat_<double>(3, 1) <<
                                       -alpha * rho_inv2,
                                       -beta  * rho_inv2,
                                       -rho_inv2);
    step = 12;
    col_a.copyTo(dpw_dab(cv::Range::all(), cv::Range(0, 1)));
    col_b.copyTo(dpw_dab(cv::Range::all(), cv::Range(1, 2)));
    col_r.copyTo(dpw_dab(cv::Range::all(), cv::Range(2, 3)));

    // 2x3 chain on (α, β, ρ); pad cols 3..4 stay zero.
    H_feature_2x5 = cv::Mat::zeros(2, SLAM_FEATURE_DIM, CV_64F);
    cv::Mat H_active = dproj_dpC * dpC_dpw * dpw_dab;   // 2x3
    H_active.copyTo(H_feature_2x5(cv::Range::all(), cv::Range(0, 3)));

    // Jacobian w.r.t. observing clone (δθ_c, δp_c):
    //   ∂p_C / ∂δθ_c = -[clone_R_F * (p_world - clone_p)]_x = -[p_C_F]_x
    //   ∂p_C / ∂δp_c = -clone_R_F
    auto skew = [](const cv::Mat& v) {
        return (cv::Mat_<double>(3, 3) <<
                0.0,                 -v.at<double>(2, 0),  v.at<double>(1, 0),
                v.at<double>(2, 0),   0.0,                -v.at<double>(0, 0),
               -v.at<double>(1, 0),   v.at<double>(0, 0),  0.0);
    };
    // Final defensive sanity on clone_R_F before unary minus — the
    // crash on 2026-05-04 hit cv::operator- here and the abort came
    // from an empty Mat. The top-of-function gate above should make
    // this unreachable; assert-and-return rather than UB if it ever
    // fires.
    if (clone_R_F.empty() || clone_R_F.rows != 3 || clone_R_F.cols != 3) {
        LOGI("slamReprojectionJacobian: clone_R_F malformed "
             "(rows=%d cols=%d empty=%d) — aborting jacobian",
             clone_R_F.rows, clone_R_F.cols, clone_R_F.empty() ? 1 : 0);
        return false;
    }
    // Use `mat * -1.0` instead of `-mat` to avoid OpenCV 4.5.3's unary
    // operator- bug. The unary operator internally constructs a MatExpr
    // with an empty placeholder operand, which then trips
    // checkOperandsExist (matrix_expressions.cpp:24) and throws
    // "Matrix operand is an empty matrix" — observed every frame after
    // 12 SLAM features were promoted (2026-05-04). `Mat * scalar` is a
    // separate operator that takes only one Mat operand and bypasses
    // the broken path entirely.
    step = 13;
    cv::Mat dpC_dtheta = skew(p_C_F) * -1.0;        // 3x3
    step = 14;
    cv::Mat dpC_dpc    = clone_R_F * -1.0;          // 3x3
    step = 15;
    H_clone_2x6 = cv::Mat::zeros(2, CLONE_DIM, CV_64F);
    cv::Mat H_clone_theta = dproj_dpC * dpC_dtheta;  // 2x3
    cv::Mat H_clone_p     = dproj_dpC * dpC_dpc;     // 2x3
    H_clone_theta.copyTo(H_clone_2x6(cv::Range::all(), cv::Range(0, 3)));
    H_clone_p    .copyTo(H_clone_2x6(cv::Range::all(), cv::Range(3, 6)));
    return true;
    } catch (const cv::Exception& e) {
        // Dump every Mat dimension so we can spot which operand is empty.
        // f.anchor_R_FEJ / f.anchor_p_FEJ / f.state are SlamFeature fields;
        // clone_R_now/p_now/R_FEJ/p_FEJ are caller-passed.
        LOGI("slamReprojectionJacobian: cv::Exception at step=%d: %s", step, e.what());
        LOGI("  state=%dx%d anchor_R_FEJ=%dx%d anchor_p_FEJ=%dx%d "
             "clone_R_FEJ=%dx%d clone_p_FEJ=%dx%d "
             "clone_R_now=%dx%d clone_p_now=%dx%d anchor_id=%d",
             f.state.rows, f.state.cols,
             f.anchor_R_FEJ.rows, f.anchor_R_FEJ.cols,
             f.anchor_p_FEJ.rows, f.anchor_p_FEJ.cols,
             clone_R_FEJ.rows, clone_R_FEJ.cols,
             clone_p_FEJ.rows, clone_p_FEJ.cols,
             clone_R_now.rows, clone_R_now.cols,
             clone_p_now.rows, clone_p_now.cols,
             f.anchor_clone_id);
        return false;
    }
}

bool EKFState::updateSlamFeature(int slot,
                                 const std::vector<cv::Point2f>& observations,
                                 const std::vector<int>& clone_ids,
                                 double pixel_noise_sq) {
    if (!full_initialized_ || P_.empty()) return false;
    if (slot < 0 || slot >= static_cast<int>(slam_features_.size())) return false;
    if (observations.size() != clone_ids.size() || observations.empty()) return false;

    const int dim = P_.rows;
    const int slam_idx = slamFeatureCovIdxInternal(slot);
    if (slam_idx < 0 || slam_idx + SLAM_FEATURE_DIM > dim) return false;

    SlamFeature& f = slam_features_[slot];

    // Gather the per-clone reprojection rows. Skip clone IDs that are no
    // longer in the sliding window — they were marginalised between when
    // the observation was recorded and when the update fires. If every
    // clone was marginalised, the update is a no-op (return false).
    std::vector<cv::Mat> H_rows;
    std::vector<cv::Mat> r_rows;
    H_rows.reserve(observations.size());
    r_rows.reserve(observations.size());

    double rms_acc = 0.0;
    int rms_n = 0;

    for (size_t k = 0; k < observations.size(); k++) {
        const int clone_id = clone_ids[k];
        const int clone_cov = getCloneCovIdx(clone_id);
        if (clone_cov < 0) continue;

        cv::Mat R_now, p_now;
        if (!getClonePose(clone_id, R_now, p_now)) continue;
        cv::Mat R_FEJ, p_FEJ;
        if (!getCloneFEJ(clone_id, R_FEJ, p_FEJ)) {
            R_FEJ = R_now.clone();
            p_FEJ = p_now.clone();
        }

        cv::Mat H_feat, H_clone;
        cv::Point2d pred;
        if (!slamReprojectionJacobian(f, R_FEJ, p_FEJ, R_now, p_now,
                                      H_feat, H_clone, pred)) {
            continue;
        }

        // Sparse 2 x dim Jacobian:
        //   - clone slot at [clone_cov, clone_cov+6)
        //   - SLAM slot   at [slam_idx, slam_idx+5)
        cv::Mat H = cv::Mat::zeros(2, dim, CV_64F);
        H_clone.copyTo(H(cv::Range::all(),
                         cv::Range(clone_cov, clone_cov + CLONE_DIM)));
        H_feat .copyTo(H(cv::Range::all(),
                         cv::Range(slam_idx, slam_idx + SLAM_FEATURE_DIM)));

        // Residual = obs - predicted (normalised image coords).
        cv::Mat r = (cv::Mat_<double>(2, 1) <<
                     observations[k].x - pred.x,
                     observations[k].y - pred.y);

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

    // Inherit Step 3a's damping + Huber kernel by going through the
    // canonical update path.
    applyMSCKFUpdate(H_stack, r_stack, R_noise);

    // Diagnostics: feature RMS for the lifecycle decision in Tracker.
    if (rms_n > 0) {
        f.last_obs_rms = std::sqrt(rms_acc / static_cast<double>(rms_n));
    }
    return true;
}

bool EKFState::applyMSCKFFeature(const std::vector<cv::Point2f>& observations,
                                 const std::vector<int>& clone_ids,
                                 const cv::Mat& triangulated_p_global,
                                 double pixel_noise_sq) {
    if (!full_initialized_ || P_.empty()) return false;
    if (observations.size() != clone_ids.size() || observations.empty()) return false;
    if (triangulated_p_global.rows != 3 || triangulated_p_global.cols != 1 ||
        triangulated_p_global.type() != CV_64F) {
        return false;
    }

    const int dim = P_.rows;

    // Build per-observation residual rows + Jacobians w.r.t. (clone pose,
    // 3-D point). The point itself is NOT in the EKF state — we project
    // it onto the left null space of H_f to eliminate the feature DOF
    // (OpenVINS / Mourikis 2007).
    //
    //     z_k = π( R_k * (p_w - p_k) )
    //
    // with R_k, p_k taken at the FEJ pose of clone k. The 2K x 3 stack
    // H_f times the 3-DOF feature update is what we project away.
    std::vector<cv::Mat> H_x_rows;   // 2 x dim (clone block only)
    std::vector<cv::Mat> H_f_rows;   // 2 x 3
    std::vector<cv::Mat> r_rows;     // 2 x 1

    auto skew = [](const cv::Mat& v) {
        return (cv::Mat_<double>(3, 3) <<
                0.0,                 -v.at<double>(2, 0),  v.at<double>(1, 0),
                v.at<double>(2, 0),   0.0,                -v.at<double>(0, 0),
               -v.at<double>(1, 0),   v.at<double>(0, 0),  0.0);
    };

    for (size_t k = 0; k < observations.size(); k++) {
        const int clone_id  = clone_ids[k];
        const int clone_cov = getCloneCovIdx(clone_id);
        if (clone_cov < 0) continue;

        cv::Mat R_now, p_now;
        if (!getClonePose(clone_id, R_now, p_now)) continue;
        cv::Mat R_FEJ, p_FEJ;
        if (!getCloneFEJ(clone_id, R_FEJ, p_FEJ)) {
            R_FEJ = R_now.clone();
            p_FEJ = p_now.clone();
        }

        cv::Mat p_C_F = R_FEJ * (triangulated_p_global - p_FEJ);
        const double zCf = p_C_F.at<double>(2, 0);
        if (zCf < 1e-4) continue;

        cv::Mat p_C_now = R_now * (triangulated_p_global - p_now);
        const double zCn = p_C_now.at<double>(2, 0);
        if (zCn < 1e-4) continue;
        const double pred_u = slam_fx_ * p_C_now.at<double>(0, 0) / zCn + slam_cx_;
        const double pred_v = slam_fy_ * p_C_now.at<double>(1, 0) / zCn + slam_cy_;

        const double xCf = p_C_F.at<double>(0, 0);
        const double yCf = p_C_F.at<double>(1, 0);
        const double zinv  = 1.0 / zCf;
        const double zinv2 = zinv * zinv;
        cv::Mat dproj_dpC = (cv::Mat_<double>(2, 3) <<
                             slam_fx_ * zinv, 0.0,             -slam_fx_ * xCf * zinv2,
                             0.0,             slam_fy_ * zinv, -slam_fy_ * yCf * zinv2);

        // ∂p_C / ∂(δθ_c, δp_c) and ∂p_C / ∂p_w:
        cv::Mat dpC_dtheta = -skew(p_C_F);
        cv::Mat dpC_dpc    = -R_FEJ;
        cv::Mat dpC_dpw    =  R_FEJ;

        cv::Mat H_clone = cv::Mat::zeros(2, CLONE_DIM, CV_64F);
        cv::Mat H_clone_t = dproj_dpC * dpC_dtheta;
        cv::Mat H_clone_p = dproj_dpC * dpC_dpc;
        H_clone_t.copyTo(H_clone(cv::Range::all(), cv::Range(0, 3)));
        H_clone_p.copyTo(H_clone(cv::Range::all(), cv::Range(3, 6)));

        cv::Mat H_x = cv::Mat::zeros(2, dim, CV_64F);
        H_clone.copyTo(H_x(cv::Range::all(),
                            cv::Range(clone_cov, clone_cov + CLONE_DIM)));

        cv::Mat H_f = dproj_dpC * dpC_dpw;          // 2x3

        cv::Mat r = (cv::Mat_<double>(2, 1) <<
                     observations[k].x - pred_u,
                     observations[k].y - pred_v);

        H_x_rows.push_back(H_x);
        H_f_rows.push_back(H_f);
        r_rows  .push_back(r);
    }

    if (H_x_rows.size() < 2) return false;  // null-space needs ≥ 2 obs

    const int K = static_cast<int>(H_x_rows.size());
    cv::Mat H_x = cv::Mat::zeros(2 * K, dim, CV_64F);
    cv::Mat H_f = cv::Mat::zeros(2 * K, 3,   CV_64F);
    cv::Mat r   = cv::Mat::zeros(2 * K, 1,   CV_64F);
    for (int k = 0; k < K; k++) {
        H_x_rows[k].copyTo(H_x(cv::Range(2 * k, 2 * k + 2), cv::Range::all()));
        H_f_rows[k].copyTo(H_f(cv::Range(2 * k, 2 * k + 2), cv::Range::all()));
        r_rows  [k].copyTo(r  (cv::Range(2 * k, 2 * k + 2), cv::Range::all()));
    }

    // QR null-space projection: A^T such that A^T * H_f = 0. SVD-based for
    // robustness on K = 2 (the QR path requires K > rank(H_f) = 3).
    cv::Mat U, W, Vt;
    cv::SVD::compute(H_f, W, U, Vt, cv::SVD::FULL_UV);
    // U is (2K x 2K). The left null space of H_f is U[:, rank..2K).
    int rank = 0;
    if (!W.empty()) {
        const double tol = 1e-6 * W.at<double>(0, 0);
        for (int i = 0; i < W.rows; i++) {
            if (W.at<double>(i, 0) > tol) rank++;
        }
    }
    if (rank >= 2 * K) return false;

    cv::Mat A = U(cv::Range::all(), cv::Range(rank, 2 * K)).clone();
    // Project: H' = A^T * H_x, r' = A^T * r, R' = A^T * (σ²I) * A = σ² * I.
    cv::Mat H_proj = A.t() * H_x;
    cv::Mat r_proj = A.t() * r;
    cv::Mat R_proj = cv::Mat::eye(H_proj.rows, H_proj.rows, CV_64F)
                     * std::max(pixel_noise_sq, 1e-8);

    applyMSCKFUpdate(H_proj, r_proj, R_proj);
    return true;
}

// ── Plan Step 6 (ADR-012): thread-safe clone snapshot for windowed BA ────────
//
// Returns the up-to-`max_clones` most-recent CameraPose entries, ordered
// oldest first so the BA caller can pick window[0] as the gauge-fix anchor.
// Locks `snapshot_mutex_` for the deque walk; addClone / pruneWindow /
// marginalizeOldestClone / reset all hold the same mutex while mutating
// `window_`, so the snapshot can never observe a torn state.
//
// The returned snapshot copies CameraPose::R_GtoC and ::p_G into cv::Matx33d /
// cv::Vec3d so the BA worker can move the data off-thread without retaining
// pointers into the EKF's internal cv::Mat storage. State_id and timestamp_ns
// pass through unchanged so the BA can later reconcile its refined poses
// against the EKF's clones by id.
std::vector<EKFState::CloneSnapshot>
EKFState::getCloneSnapshot(int max_clones) const {
    std::vector<CloneSnapshot> out;
    if (max_clones <= 0) return out;

    std::lock_guard<std::mutex> lock(snapshot_mutex_);

    if (window_.empty()) return out;

    const int n_clones = static_cast<int>(window_.size());
    const int n_take   = std::min(max_clones, n_clones);
    const int start    = n_clones - n_take;  // oldest of the take-window

    out.reserve(static_cast<size_t>(n_take));
    for (int i = start; i < n_clones; ++i) {
        const CameraPose& p = window_[static_cast<size_t>(i)];
        if (p.R_GtoC.rows != 3 || p.R_GtoC.cols != 3 ||
            p.p_G.rows    != 3 || p.p_G.cols    != 1) {
            // Skip malformed clones rather than crash the BA thread; in
            // practice this never fires because addClone always writes 3x3
            // and 3x1 cv::Mat instances.
            continue;
        }
        CloneSnapshot s;
        s.clone_id     = p.state_id;
        s.timestamp_ns = p.timestamp_ns;
        s.R = cv::Matx33d(
            p.R_GtoC.at<double>(0, 0), p.R_GtoC.at<double>(0, 1), p.R_GtoC.at<double>(0, 2),
            p.R_GtoC.at<double>(1, 0), p.R_GtoC.at<double>(1, 1), p.R_GtoC.at<double>(1, 2),
            p.R_GtoC.at<double>(2, 0), p.R_GtoC.at<double>(2, 1), p.R_GtoC.at<double>(2, 2));
        s.t = cv::Vec3d(
            p.p_G.at<double>(0, 0),
            p.p_G.at<double>(1, 0),
            p.p_G.at<double>(2, 0));
        out.push_back(s);
    }
    return out;
}
