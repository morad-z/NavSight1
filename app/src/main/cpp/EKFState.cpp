#include "EKFState.h"
#include "EventCounters.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <opencv2/calib3d.hpp>
#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-EKF"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGW(...) (void)0
#define LOGE(...) (void)0
#endif

EKFState::EKFState() {
    reset();
    // 2026-05-17 — log loaded noise constants so post-hoc analysis can confirm
    // the Allan-calibrated values are in effect. If a future change reverts
    // them silently, this LOGI exposes the regression in logcat.
    LOGI("EKF_NOISE: sigma_g=%.4e sigma_a=%.4e sigma_bg=%.4e sigma_ba=%.4e (Allan 2026-05-17)",
         sigma_g_, sigma_a_, sigma_bg_, sigma_ba_);
}

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
    last_marginalized_slam_feature_ids_.clear();

    // 2026-05-19 R_bc DERIVED — first-principles for SENSOR_ORIENTATION=90
    // rear camera, vertical portrait phone.
    //
    // Body convention (SensorRepository.kt:401):
    //   +X = forward (device top), +Y = your left, +Z = up (out of screen)
    // Held vertically in portrait, rear camera faces body +X (forward).
    //
    // OpenCV camera frame:
    //   +X = image right (column ++), +Y = image down (row ++), +Z = optical fwd
    //
    // SENSOR_ORIENTATION=90 = "rotate raw 90° CW to get upright". So:
    //   raw image axes map to scene:
    //     raw top    = upright right  = scene right = body -Y
    //     raw right  = upright bottom = scene down  = body -Z
    //     raw bottom = upright left   = scene left  = body +Y
    //     raw left   = upright top    = scene up    = body +Z
    //
    //   ∴ camera axes in body coords:
    //     camera +X (raw right) = body -Z
    //     camera +Y (raw down ≡ raw bottom direction) = body +Y
    //     camera +Z (optical fwd) = body +X
    //
    // R_bc maps body → camera. Columns = camera coords of body basis vectors:
    //   body +X → camera +Z  → col0 = (0, 0, 1)
    //   body +Y → camera +Y  → col1 = (0, 1, 0)
    //   body +Z → camera -X  → col2 = (-1, 0, 0)
    //
    //   R_bc = ((0, 0, -1),
    //           (0, 1,  0),
    //           (1, 0,  0))
    //
    // Verification: R_bc · (1,0,0)body = (0,0,1)cam = camera +Z (forward) ✓
    //               R_bc · (0,0,1)body = (-1,0,0)cam = camera -X (image left) ✓
    //                                                  (sky points to image-left)
    //
    // Falsifier: SLAM orange dots should stay anchored to scene features when
    // user rotates the phone in place. Per v51 visual: KLT (cyan/green/yellow)
    // anchored fine but SLAM dots drifted → projection R_bc · R_GtoI wrong.
    // V-shape fix proved R_GtoI is now correct → R_bc is the remaining error.
    // 2026-05-19 FINAL R_bc — re-derived carefully and confirmed by visual
    // test (user reported "much better, dots anchored, moved only slightly").
    //
    // The previous "first principles" attempt had a sign error from
    // conflating "looking at the scene through the lens" vs "photographer's
    // perspective behind the camera." Correct derivation:
    //
    //   For back camera (optical axis = body -Z), photographer stands behind
    //   the screen looking through camera in body -Z direction.
    //   - Photographer's "right" (camera +X / image right):
    //       (body -Z) × (body +X) = (0,0,-1) × (1,0,0) = (0,-1,0) = body -Y
    //   - Photographer's "down" (camera +Y / image down):
    //       scene down (in portrait body, +X is up against gravity)
    //                                                            = body -X
    //   - Camera +Z = body -Z (optical fwd opposite to screen-out)
    //
    //   R_bc columns = camera coords of body basis vectors:
    //     body +X → camera -Y → col0 = (0, -1, 0)
    //     body +Y → camera -X → col1 = (-1, 0, 0)
    //     body +Z → camera -Z → col2 = (0, 0, -1)
    //
    //   R_bc = ((0,-1,0),(-1,0,0),(0,0,-1))     det = +1 ✓
    //
    // Equivalent to diag(1,-1,-1) · Rz(+90°) — the "v23.7 sign-flipped"
    // version that was never tried before today. Residual "dots move
    // slightly" likely comes from sub-degree variation in actual sensor
    // mounting (Step 8b online R_bc calibration is currently SKIPPED;
    // re-enabling it could absorb this residual but is a separate task).
    R_bc_ = cv::Matx33d(0.0, -1.0,  0.0,
                       -1.0,  0.0,  0.0,
                        0.0,  0.0, -1.0);
    /* SUPERSEDED 2026-05-19 — wrong-sign first-principles attempt
       (drifted in opposite direction):
    R_bc_ = cv::Matx33d(0.0, 0.0, -1.0,
                        0.0, 1.0,  0.0,
                        1.0, 0.0,  0.0);
    */
    /* SUPERSEDED 2026-05-19 — pre-derivation default (was for some other body
       convention or sensor orientation; SLAM orange dots drifted on rotation):
    R_bc_ = cv::Matx33d(1.0,  0.0,  0.0,
                        0.0, -1.0,  0.0,
                        0.0,  0.0, -1.0);
    */
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

    // Initialize 19x19 IMU covariance.
    // Row layout: δθ(0-2), δb_g(3-5), δv(6-8), δb_a(9-11), δp(12-14),
    //             δt_d(15, Step 8a), δφ_bc(16-18, Step 8b).
    P_ = cv::Mat::zeros(IMU_STATE_DIM, IMU_STATE_DIM, CV_64F);
    // Initial uncertainties
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

    // Step 8a (ADR-014): seed td variance from the pre-init scalar P_td_.
    // The TD warmup in Tracker.cpp may have already called setTimeOffset(), which
    // sets P_td_ to (2ms)². If the warmup has not run yet, P_td_ holds the
    // default (5ms)². Either way, the initial P_(15,15) matches the best prior
    // we have — consistent with OpenVINS default initialisation for td.
    P_.at<double>(15, 15) = P_td_;

    // Step 8b: initialise extrinsics covariance for δφ_bc at rows 16–18.
    // INIT_EXTR_STD = 0.05 rad (≈ 3°).
    // Source: Android getCameraOrientation reports physical sensor orientation
    // to within 5° (Android CDD §7.7.1). 3° is a conservative prior that
    // lets the filter refine without premature lock-in.
    constexpr double INIT_EXTR_STD = 0.05;  // ~3 deg in rad
    for (int i = 0; i < EXTR_STATE_DIM; i++) {
        P_.at<double>(EXTR_STATE_OFFSET + i, EXTR_STATE_OFFSET + i) =
            INIT_EXTR_STD * INIT_EXTR_STD;
    }

    full_initialized_ = true;

    // 2026-05-16 v26 diagnostic: emit the initial R_GtoI's tilt vs gravity
    // at the moment of full-init. R_GtoI maps world→body. World +Z is
    // up. R_GtoI · (0,0,1)_world tells us where "up" lives in the body
    // frame. For a perfectly vertical phone (body Y aligned with world Z),
    // R_GtoI · (0,0,1) = (0, 1, 0). The angle between that and (0,1,0)
    // is the "tilt from vertical" at init. v26 evidence: bootstrap with
    // a tilted phone caused subsequent MSCKF chaos; this log will let
    // us see the tilt magnitude per session and correlate with downstream
    // r_R residuals.
    if (R_GtoI_.rows == 3 && R_GtoI_.cols == 3 && R_GtoI_.type() == CV_64F) {
        const double up_body_x = R_GtoI_.at<double>(0, 2);
        const double up_body_y = R_GtoI_.at<double>(1, 2);
        const double up_body_z = R_GtoI_.at<double>(2, 2);
        // Tilt-from-vertical = angle between (up_body_x, up_body_y, up_body_z)
        // and the expected (0, 1, 0) for vertical phone. For phone-flat
        // expected would be (0, 0, 1); we report against vertical-hold
        // since that's the documented use case.
        const double dot_vertical = up_body_y;  // dot with (0,1,0)
        const double dot_flat     = up_body_z;  // dot with (0,0,1)
        const double tilt_vert_deg = std::acos(std::max(-1.0, std::min(1.0, dot_vertical)))
                                      * 180.0 / M_PI;
        const double tilt_flat_deg = std::acos(std::max(-1.0, std::min(1.0, dot_flat)))
                                      * 180.0 / M_PI;
        LOGI("EKF_INIT_TILT: up_body=(%.3f,%.3f,%.3f) "
             "tilt_from_vertical_deg=%.2f tilt_from_flat_deg=%.2f "
             "R_bc_angle_from_initial_deg=%.2f",
             up_body_x, up_body_y, up_body_z,
             tilt_vert_deg, tilt_flat_deg,
             getExtrinsicsAngleDeg());
    }

    LOGI("EKF full state initialized (19-DOF: δθ,δb_g,δv,δb_a,δp,δt_d,δφ_bc)");
}

// ── IMU Propagation ─────────────────────────────────────────────────────────

void EKFState::propagateIMU(const cv::Mat& deltaR, const cv::Mat& deltaV,
                             const cv::Mat& deltaP, double dt,
                             const cv::Mat& imu_cov,
                             const cv::Mat& J_R_bg, const cv::Mat& J_V_bg,
                             const cv::Mat& J_V_ba, const cv::Mat& J_P_bg,
                             const cv::Mat& J_P_ba) {
    if (!full_initialized_ || dt <= 0 || P_.empty()) {
        // Count sub-reasons so the JSON event_summary shows exactly which gate fired.
        auto& ec = navsight::eventCounters();
        ec.propagate_skipped.fetch_add(1, std::memory_order_relaxed);
        if (!full_initialized_) {
            ec.propagate_skipped_not_init.fetch_add(1, std::memory_order_relaxed);
        } else if (dt <= 0) {
            ec.propagate_skipped_dt_nonpositive.fetch_add(1, std::memory_order_relaxed);
            LOGW("propagateIMU: dt=%f <= 0 (timestamp wraparound or JNI unit mismatch)", dt);
        } else {
            ec.propagate_skipped_p_empty.fetch_add(1, std::memory_order_relaxed);
            LOGW("propagateIMU: P_ is empty (covariance not initialized)");
        }
        return;
    }

    // Plan Step 3a (ADR-008): tick the "frames since last MSCKF" counter so
    // the damping ramp resets after a quiet period (≥ MSCKF_QUIET_PROPAGATION
    // propagation steps without an MSCKF correction).
    if (msckf_frames_since_call_ < MSCKF_QUIET_PROPAGATION + 1) {
        msckf_frames_since_call_++;
    }
    if (msckf_frames_since_call_ >= MSCKF_QUIET_PROPAGATION) {
        msckf_damping_step_ = 0;  // cold start on the next MSCKF call
    }

    // Propagate mean state. World frame is ENU Z-up (X=East, Y=North, Z=Up),
    // matching the Madgwick filter (IMUPreintegrator.cpp:677,774) and
    // Android sensor body frame (body Z is gravity-aligned when phone is
    // held screen-up). Gravity vector therefore points along world -Z.
    cv::Mat g = (cv::Mat_<double>(3,1) << 0.0, 0.0, -9.81);
    // 2026-05-09 propagation fix — `R_new = R_GtoI_ * deltaR` was inconsistent
    // with both the Phi linearization at line 170 (`δθ_new = deltaR^T · δθ`,
    // i.e. LEFT perturbation by deltaR^T) and the world-to-body convention
    // pinned by scripts/test_z_up_conventions.py (`R_GtoI · v_world = v_body`).
    //
    // Forster preintegration produces deltaR as the body-to-world relative
    // rotation Δ over the integration interval (`current_R = current_R *
    // dR_step` in IMUPreintegrator.cpp:316). Standard navigation:
    //
    //   R_W_b(j)  =  R_W_b(i) · ΔR_ij           (Forster)
    //   R_GtoI(j) = R_W_b(j)^T = ΔR_ij^T · R_W_b(i)^T = ΔR_ij^T · R_GtoI(i)
    //
    // The previous formula (R_GtoI · ΔR) is the body-to-world right-perturb
    // rule — applied to a world-to-body matrix it walks R_GtoI in the wrong
    // direction. Verified by example with flat→portrait rotation: the old
    // formula produces the TRANSPOSE of the correct R_GtoI, exactly matching
    // the v9/v10 trace where LC_GA r_norm grew from ~0 to 1.97 (max=2.0)
    // over 90 s of walking — i.e. R_GtoI degenerated toward its inverse as
    // wrong-direction increments accumulated.
    cv::Mat R_new = deltaR.t() * R_GtoI_;
    cv::Mat v_new = v_G_ + g * dt + R_GtoI_.t() * deltaV;
    cv::Mat p_new = p_G_ + v_G_ * dt + 0.5 * g * dt * dt + R_GtoI_.t() * deltaP;

    // Build discrete state transition matrix Phi (19x19)
    // Error-state: [δθ(0-2), δb_g(3-5), δv(6-8), δb_a(9-11), δp(12-14), δt_d(15), δφ_bc(16-18)]
    // Linearized dynamics (simplified for pedestrian VIO):
    //   δθ_new  = deltaR^T * δθ - J_R_bg * δb_g
    //   δb_g    = δb_g (random walk)
    //   δv_new  = δv + R^T * [deltaV]_x * δθ - R^T * J_V_bg * δb_g - R^T * J_V_ba * δb_a
    //   δb_a    = δb_a (random walk)
    //   δp_new  = δp + δv * dt + R^T * [deltaP]_x * δθ - R^T * J_P_bg * δb_g - R^T * J_P_ba * δb_a
    //   δt_d    = δt_d (constant with slow random walk — Phi(15,15)=1 from eye init)
    //   δφ_bc   = δφ_bc (constant — extrinsics don't drift; Phi(16-18,16-18)=I from eye init)

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

    // dv/dθ: -[R_ItoG · deltaV_body]_× = -skew(Rt * deltaV)
    //
    // Cause:  OLD code computed `Rt * skew(deltaV)`.  For a LEFT-perturbation
    //         world-frame EKF, R*skew(v) ≠ skew(R*v) in general, so the block
    //         was wrong in both sign and skew/rotation order.
    // Change: Forster 2017 TRO eq. (A22) for LEFT-perturbation world-frame:
    //           ∂δv/∂δθ = -[R_ItoG · deltaV_body]_× = -skew(Rt * deltaV)
    // Falsifier: unit test with deltaR=I, deltaV=(1,0,0), R_GtoI_=I:
    //            expected Phi(6:9, 0:3) = -skew((1,0,0)) = [[0,0,0],[0,0,1],[0,-1,0]]
    //            OLD code produced +[[0,0,0],[0,0,1],[0,-1,0]] — wrong sign.
    // Date:   2026-05-16 (ekf_audit.md Finding 3)
    /* OLD (2026-05-16 superseded):
    if (!deltaV.empty()) {
        cv::Mat dv_dtheta = Rt * skew(deltaV);
        dv_dtheta.copyTo(Phi(cv::Range(6,9), cv::Range(0,3)));
    }
    */
    if (!deltaV.empty()) {
        // 2026-05-16 Tier 1 revert (agent A2): swarm's Finding-3 sign flip was
        // mathematically incorrect. Original `+Rt * skew(deltaV)` is correct for
        // NavSight's world-frame δθ on world→body R_GtoI convention (pinned by
        // EKFState.cpp:1006, 1474-1477, 1895-1896, 1976). Falsifier (verified
        // by A2): with R=I, deltaV=(1,0,0)_body, yaw δθ_z=ε rotates body-X
        // toward world-Y, so ∂v_y/∂δθ_z = 1, giving block [[0,0,0],[0,0,1],
        // [0,-1,0]] = +skew((1,0,0)) — matches OLD `+Rt*skew(v)`, NOT new form.
        cv::Mat dv_dtheta = Rt * skew(deltaV);
        dv_dtheta.copyTo(Phi(cv::Range(6,9), cv::Range(0,3)));
        /* SUPERSEDED 2026-05-16 Tier 1 revert: sign was inverted by swarm.
        cv::Mat dv_dtheta = -skew(Rt * deltaV);
        dv_dtheta.copyTo(Phi(cv::Range(6,9), cv::Range(0,3)));
        */
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

    // dp/dθ: -[R_ItoG · deltaP_body]_× = -skew(Rt * deltaP)
    //
    // Cause:  Same wrong-frame skew as dv/dθ above.  See Forster 2017 TRO
    //         eq. (A22).  The negative sign was missing AND the matrix
    //         product order was reversed (R·skew ≠ skew·R in general).
    // Change: Use -skew(Rt * deltaP), consistent with LEFT-perturbation.
    // Falsifier: with deltaR=I, deltaP=(0,1,0), R_GtoI_=I:
    //            expected Phi(12:15, 0:3) = -skew((0,1,0))
    //            OLD code produced +skew((0,1,0)) — wrong sign.
    // Date:   2026-05-16 (ekf_audit.md Finding 3)
    /* OLD (2026-05-16 superseded):
    if (!deltaP.empty()) {
        cv::Mat dp_dtheta = Rt * skew(deltaP);
        dp_dtheta.copyTo(Phi(cv::Range(12,15), cv::Range(0,3)));
    }
    */
    if (!deltaP.empty()) {
        // 2026-05-16 Tier 1 revert (agent A2): same as dv_dtheta above — the
        // OLD form `+Rt * skew(deltaP)` matches the world-frame δθ convention.
        cv::Mat dp_dtheta = Rt * skew(deltaP);
        dp_dtheta.copyTo(Phi(cv::Range(12,15), cv::Range(0,3)));
        /* SUPERSEDED 2026-05-16 Tier 1 revert:
        cv::Mat dp_dtheta = -skew(Rt * deltaP);
        dp_dtheta.copyTo(Phi(cv::Range(12,15), cv::Range(0,3)));
        */
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

    // Process noise Q (19x19)
    cv::Mat Q = cv::Mat::zeros(IMU_STATE_DIM, IMU_STATE_DIM, CV_64F);
    // Use preintegration covariance if available, otherwise construct from noise params
    if (!imu_cov.empty() && imu_cov.rows >= 9) {
        // Map 9x9 preintegration covariance (R,V,P order) to 19x19 Q.
        //
        // Cause:  OLD code only copied the 3 diagonal 3×3 blocks (R-R, V-V,
        //         P-P), discarding the 6 off-diagonal cross-blocks (R-V, R-P,
        //         V-P and their transposes).  Dropping cross-terms underestimates
        //         the noise covariance and breaks the positive-semi-definiteness
        //         guarantee of Q when preintegration produces significant
        //         off-diagonal noise (e.g. at high angular velocity).
        // Change: Copy all 9 sub-blocks of the 9×9 preintegration covariance
        //         into the correct slots in Q, preserving full correlation.
        //         Index mapping: imu_cov rows/cols 0-2=R, 3-5=V, 6-8=P;
        //         Q rows/cols: 0-2=θ, 6-8=v, 12-14=p.
        // Falsifier: build Q from imu_cov; verify Q symmetry and PSD
        //            via Cholesky.  Q.at<double>(0,6) must equal
        //            imu_cov.at<double>(0,3) (R-V cross-term).
        // Date:   2026-05-16 (ekf_audit.md Finding 2 / preintegration Q)
        /* OLD (2026-05-16 superseded — diagonal-only copy):
        imu_cov(cv::Range(0,3), cv::Range(0,3)).copyTo(Q(cv::Range(0,3), cv::Range(0,3)));
        imu_cov(cv::Range(3,6), cv::Range(3,6)).copyTo(Q(cv::Range(6,9), cv::Range(6,9)));
        imu_cov(cv::Range(6,9), cv::Range(6,9)).copyTo(Q(cv::Range(12,15), cv::Range(12,15)));
        */
        // Diagonal blocks
        imu_cov(cv::Range(0,3), cv::Range(0,3)).copyTo(Q(cv::Range(0,3),   cv::Range(0,3)));
        imu_cov(cv::Range(3,6), cv::Range(3,6)).copyTo(Q(cv::Range(6,9),   cv::Range(6,9)));
        imu_cov(cv::Range(6,9), cv::Range(6,9)).copyTo(Q(cv::Range(12,15), cv::Range(12,15)));
        // Off-diagonal: R-V
        imu_cov(cv::Range(0,3), cv::Range(3,6)).copyTo(Q(cv::Range(0,3),   cv::Range(6,9)));
        imu_cov(cv::Range(3,6), cv::Range(0,3)).copyTo(Q(cv::Range(6,9),   cv::Range(0,3)));
        // Off-diagonal: R-P
        imu_cov(cv::Range(0,3), cv::Range(6,9)).copyTo(Q(cv::Range(0,3),   cv::Range(12,15)));
        imu_cov(cv::Range(6,9), cv::Range(0,3)).copyTo(Q(cv::Range(12,15), cv::Range(0,3)));
        // Off-diagonal: V-P
        imu_cov(cv::Range(3,6), cv::Range(6,9)).copyTo(Q(cv::Range(6,9),   cv::Range(12,15)));
        imu_cov(cv::Range(6,9), cv::Range(3,6)).copyTo(Q(cv::Range(12,15), cv::Range(6,9)));
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
    // Step 8a (ADR-014): td random walk process noise.
    // Models slow thermal drift of camera-IMU time offset. Value 1e-7 s/√Hz
    // is the OpenVINS default for MEMS IMUs (Li & Mourikis 2014, §III-B).
    // Discrete-time variance per step = (1e-7)^2 * dt.
    constexpr double SIGMA_TD_RW = 1e-7;  // s/√Hz — OpenVINS default for MEMS
    Q.at<double>(15, 15) = SIGMA_TD_RW * SIGMA_TD_RW * dt;

    // Step 8b: extrinsics rotation random walk process noise for δφ_bc (rows 16-18).
    // The body→camera rotation is physically constant; this tiny regularisation
    // prevents P_(16..18,16..18) from collapsing to zero (which would lock out
    // future refinement if the geometry changes slightly e.g. phone orientation
    // shifts). Value 1e-8 rad/√Hz per axis is intentionally negligible —
    // extrinsics do not drift. Source: OpenVINS online extrinsic calibration
    // reference implementation (Geneva et al. 2020, §IV-B, "near-zero" prior).
    constexpr double SIGMA_EXTR_RW = 1e-8;  // rad/√Hz — near-zero regularisation
    for (int i = 0; i < EXTR_STATE_DIM; i++) {
        Q.at<double>(EXTR_STATE_OFFSET + i, EXTR_STATE_OFFSET + i) =
            SIGMA_EXTR_RW * SIGMA_EXTR_RW * dt;
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

    // ── FIX 3: P_bc cross-covariance decoupling ─────────────────────────────
    // Cause:  δφ_bc has no measurement update (H_bc=0, rows 16-18 of dx never
    //         applied — see applyStateCorrection ~line 808).  Yet Phi*P*Phi^T
    //         propagates P_(0:3, 16:19) every frame via Phi(0:3,0:3)=deltaR^T,
    //         growing unconstrained cross-correlation between δθ_IMU and δφ_bc.
    //         Over 1000 frames this deflects the Kalman gain for θ in a direction
    //         with no physical measurement backing (ekf_audit.md Finding 2).
    // Change: After propagation, zero the entire P_(16:19, :) row block and
    //         P_(:, 16:19) column block so δφ_bc stays fully decoupled while
    //         its update is disabled (Step 8b bypass).
    // Falsifier: log cv::norm(P_(cv::Range(0,3), cv::Range(16,19))) after 500
    //            frames of walking.  Must remain < 1e-10 with this fix.
    // Date:   2026-05-16 (ekf_audit.md Finding 2)
    // 2026-05-16 Tier 1 revert (agent A10): P_bc cross-cov zeroing every frame
    // breaks the symmetry/PSD invariants the Joseph form relies on, and could
    // trigger silent Cholesky fallbacks in MSCKF. Existing 1e-8 process-noise
    // regularization (see Q matrix init) is sufficient to bound unconstrained
    // growth of P_(0:3, 16:19) cross-correlation. Keep δφ_bc state in place
    // but stop the per-frame zeroing.
    /* SUPERSEDED 2026-05-16 Tier 1 revert:
    static bool pbc_decouple_logged_ = false;
    if (total_dim >= IMU_STATE_DIM) {
        // EXTR_STATE_OFFSET = 16, EXTR_STATE_DIM = 3 (δφ_bc rows 16-18).
        P_(cv::Range(EXTR_STATE_OFFSET, EXTR_STATE_OFFSET + EXTR_STATE_DIM),
           cv::Range(0, total_dim)).setTo(0.0);
        P_(cv::Range(0, total_dim),
           cv::Range(EXTR_STATE_OFFSET, EXTR_STATE_OFFSET + EXTR_STATE_DIM)).setTo(0.0);
        if (!pbc_decouple_logged_) {
            LOGI("P_BC_DECOUPLE: zeroed cross-cov rows 16-18 (Step 8b consistency fix)");
            pbc_decouple_logged_ = true;
        }
    }
    */

    // 2026-05-16 velocity-divergence diagnostic (v29 walk showed v sitting at
    // 5 m/s clamp for entire walk with 467 clamp events / 622 ticks). To
    // identify whether divergence is from (A) accel bias init wrong, (B)
    // gravity not canceling, or (C) propagation math error, log every Nth
    // call the components feeding v_new:
    //   - gravity contribution g·dt
    //   - body-frame deltaV rotated to world (R^T · deltaV)
    //   - resulting velocity v_new
    //   - current accel bias estimate b_a_
    //   - current gyro bias estimate
    // If g*dt + R^T*deltaV doesn't sum to ~0 when stationary, gravity isn't
    // cancelling (bug A or B). If b_a_ drifts during motion, bias estimate is
    // absorbing motion (bug B variant).
    static int prop_dbg_counter = 0;
    if ((prop_dbg_counter++ % 30) == 0) {
        const cv::Mat g_dt        = g * dt;
        const cv::Mat dV_world    = R_GtoI_.t() * deltaV;
        const cv::Mat sum         = g_dt + dV_world;
        LOGI("EKF_PROP_DBG: dt=%.4f "
             "g_dt=(%+.3f,%+.3f,%+.3f) "
             "R^T*dV=(%+.3f,%+.3f,%+.3f) "
             "sum=(%+.3f,%+.3f,%+.3f) "
             "v_pre=(%+.3f,%+.3f,%+.3f) |v_pre|=%.3f "
             "v_new=(%+.3f,%+.3f,%+.3f) |v_new|=%.3f "
             "b_a=(%+.4f,%+.4f,%+.4f) "
             "b_g=(%+.5f,%+.5f,%+.5f)",
             dt,
             g_dt.at<double>(0), g_dt.at<double>(1), g_dt.at<double>(2),
             dV_world.at<double>(0), dV_world.at<double>(1), dV_world.at<double>(2),
             sum.at<double>(0), sum.at<double>(1), sum.at<double>(2),
             v_G_.at<double>(0), v_G_.at<double>(1), v_G_.at<double>(2),
             cv::norm(v_G_),
             v_new.at<double>(0), v_new.at<double>(1), v_new.at<double>(2),
             cv::norm(v_new),
             b_a_.at<double>(0), b_a_.at<double>(1), b_a_.at<double>(2),
             b_g_.at<double>(0), b_g_.at<double>(1), b_g_.at<double>(2));
    }

    // Update mean state
    R_GtoI_ = R_new;
    v_G_ = v_new;
    p_G_ = p_new;

    // Clamp velocity to physically reasonable bounds (pedestrian: max 5 m/s)
    double v_norm = cv::norm(v_G_);
    if (v_norm > 5.0) {
        LOGW("propagateIMU: velocity clamped from %.2f m/s to 5.0 m/s (preintegration bug?)", v_norm);
        navsight::eventCounters().velocity_clamped.fetch_add(1, std::memory_order_relaxed);
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
    if ((innov * innov) / S > 9.0) {
        LOGI("updateScale: chi2 reject (innov=%.4f S=%.6f chi2=%.2f > 9.0)",
             innov, S, (innov * innov) / S);
        navsight::eventCounters().scale_chi2_rejected.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    double K = P_scale_ / S;
    scale_ += K * innov;
    scale_ = std::max(0.005, std::min(20.0, scale_));
    P_scale_ = (1.0 - K) * P_scale_;
}

void EKFState::updateZUPT() {
    P_scale_ *= 0.99;
    // Step 8a (ADR-014): when full state is initialized, td variance is in P_(15,15).
    // ZUPT (stationary period) is weak evidence about td, so apply a mild 0.1%
    // shrink — same as the legacy P_td_ shrink, now applied to the in-state entry.
    // Also keep P_td_ in sync for pre-init/fallback getTimeOffsetStd().
    P_td_ *= 0.999;
    if (full_initialized_ && !P_.empty() && P_.rows > 15) {
        P_.at<double>(15, 15) *= 0.999;
    }

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

// ── Zero-Rotation-Rate Update (ZRUP) ────────────────────────────────────────
// 2026-05-16: dual of updateZUPT. ZUPT observes v=0 when stationary; ZRUP
// observes ω=0 by the equivalent formulation: when stationary, the TRUE
// angular velocity is zero, so the gyro reading equals the bias by
// definition. Innovation: mean(gyro_window) - b_g_, with H = -I on rows
// 3:6 (the b_g_ error-state block). Standard Kalman update propagates the
// observation into b_g_ and reduces P_(3:6, 3:6). The EKF→Madgwick gyro-
// bias feedback loop (Tracker.cpp:914) then correctly propagates the
// improved estimate to Madgwick's integrator.
//
// Cause: when stationary, no visual update fires (no parallax) so b_g_
//        was previously frozen indefinitely. Gyro integration with stale
//        bias produced ~0.5-1°/s heading drift — exactly the symptom
//        user reported 2026-05-16 ("heading slowly rotating when standing
//        still, SLAM dots sliding top-right").
// Change: dual-of-ZUPT Kalman update. Fires alongside updateZUPT at the
//        same is_stationary trigger.
// Falsifier: 60s stationary → b_g_ converges to gyro mean in ~3s;
//        zrup_fired_total > 0; heading drift < 1° over the window;
//        SLAM dots stay anchored on screen.
//
// Returns true if the update fired, false if full state not initialized
// or window size N < 1 (insufficient data).
bool EKFState::updateZRUP(double mean_gx, double mean_gy, double mean_gz,
                           double sigma_gyro_per_sample, int N_samples) {
    if (!full_initialized_) return false;
    if (P_.empty() || P_.rows < IMU_STATE_DIM) return false;
    if (N_samples < 1) return false;
    if (sigma_gyro_per_sample <= 0.0) return false;

    const int dim = P_.rows;

    // Residual: r = mean(gyro_window) - b_g_  (3x1)
    cv::Mat r = (cv::Mat_<double>(3, 1) <<
                  mean_gx - b_g_.at<double>(0, 0),
                  mean_gy - b_g_.at<double>(1, 0),
                  mean_gz - b_g_.at<double>(2, 0));

    // 2026-05-18 falsifier instrumentation: emit entry log so we can
    // time-align ZRUP fires against the anonymous MSCKF_DX_BLOCKS lines
    // that pump b_g during stationary boot. If a ZRUP_FIRE timestamp
    // matches an MSCKF_DX_BLOCKS dbg!=0 line, ZRUP is the b_g-leak source.
    // If b_g moves AWAY from mean_gyro after the update, the H-sign at the
    // lines below is wrong (treating x_true = x_est + δx as error-state).
    LOGI("ZRUP_FIRE: mean_g=(%+.5f,%+.5f,%+.5f) pre_b_g=(%+.5f,%+.5f,%+.5f) "
         "r=(%+.5f,%+.5f,%+.5f) sigma_g_sample=%.3e N=%d",
         mean_gx, mean_gy, mean_gz,
         b_g_.at<double>(0, 0), b_g_.at<double>(1, 0), b_g_.at<double>(2, 0),
         r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0),
         sigma_gyro_per_sample, N_samples);

    // 2026-05-18 SIGN FIX (root cause of the multi-week 1°/s stationary
    // heading drift documented in HANDOFF_2026_05_18.md).
    //
    // Cause:   H was -I on b_g rows. With Kalman gain K = P·H^T·S^(-1), a
    //          negative H produced K_bg = -P_bg/(P_bg+R) (negative gain), so
    //          δb_g = K_bg · r pushed b_g AWAY from the observed gyro mean
    //          on every fire. Falsifier instrumentation (ZRUP_FIRE/POST)
    //          captured 374/374 fires where residual GREW post-update —
    //          structural, not noise. b_g.z then saturated near ±0.011-0.017
    //          rad/s when gain decayed to match process-noise injection. The
    //          every-6-frame Tracker.cpp:1243 push broadcast this corrupted
    //          estimate to Madgwick, producing the user-visible drift at
    //          exactly EKF.b_g_.z °/s.
    //
    // Change:  H = +I on b_g rows (standard error-state convention).
    //          Derivation: z = h(x_true) + ε, h(x) = b_g (when stationary).
    //          x_true = x_est + δx (additive bias state).
    //          h(x_est + δx) = b_g_est + (∂h/∂δb_g)·δb_g = b_g_est + (+I)·δb_g.
    //          Therefore H = +I, NOT -I. The author's original comment
    //          ("residual ≈ -δb_g") conflated the residual r = z - h(x_est)
    //          with the linearization h(x_est + δx) - h(x_est), getting one
    //          sign flipped.
    //
    // Falsifier: post-fix ZRUP_POST residual must be SMALLER than ZRUP_FIRE
    //          residual every fire (b_g converging toward mean_gyro). b_g.z
    //          should stay within Allan σ_bg = 7.46e-7 √Hz · √t of zero on
    //          a calibrated stationary phone — well under 1e-4 rad/s after
    //          10s. Madgwick stationary heading drift must drop from ~1°/s
    //          to <0.05°/s (Allan-predicted).
    cv::Mat H = cv::Mat::zeros(3, dim, CV_64F);
    H.at<double>(0, 3) = +1.0;
    H.at<double>(1, 4) = +1.0;
    H.at<double>(2, 5) = +1.0;
    /* SUPERSEDED 2026-05-18 (sign-flip bug, drove the multi-week heading-drift symptom):
    // H: 3 rows, -I at cols 3:6 (b_g_ error-state block), 0 elsewhere.
    // Sign is negative because the error-state convention is x_true = x_est + δx;
    // raw_gyro - b_g_true = 0 → raw - (b_g_est + δb_g) = -δb_g → residual ≈ -δb_g.
    H.at<double>(0, 3) = -1.0;
    H.at<double>(1, 4) = -1.0;
    H.at<double>(2, 5) = -1.0;
    */

    // R: combined noise — IMU sensor noise (sigma_g²) divided by sample count
    // (averaging N samples reduces variance by N) + an extra "stationarity
    // assumption" term that we leave at zero here because the is_stationary
    // detector already gated the window for low variance.
    const double r_var = (sigma_gyro_per_sample * sigma_gyro_per_sample) /
                          static_cast<double>(N_samples);
    cv::Mat R_noise = cv::Mat::eye(3, 3, CV_64F) * r_var;

    // Standard Joseph-form Kalman update via the existing pipeline.
    applyMSCKFUpdate(H, r, R_noise);

    // 2026-05-18 falsifier: post-update b_g tells us if the H sign above is
    // correct. After ZRUP, b_g should move TOWARD mean_gyro (residual shrinks).
    // If post_b_g moves AWAY from mean_gyro (residual grows), the H = -I
    // convention at line 617-620 is inverted vs the standard error-state
    // x_true = x_est + δx, and ZRUP is pumping b_g instead of draining it.
    LOGI("ZRUP_POST: post_b_g=(%+.5f,%+.5f,%+.5f) post_r=(%+.5f,%+.5f,%+.5f)",
         b_g_.at<double>(0, 0), b_g_.at<double>(1, 0), b_g_.at<double>(2, 0),
         mean_gx - b_g_.at<double>(0, 0),
         mean_gy - b_g_.at<double>(1, 0),
         mean_gz - b_g_.at<double>(2, 0));

    return true;
}

// DEAD CODE: checkConsistency — never called
// double EKFState::checkConsistency(double camera_disp, double step_disp) const { ... }

// DEAD CODE: getScaleStd — never called
// double EKFState::getScaleStd() const { ... }

// ── Clone Management ────────────────────────────────────────────────────────

void EKFState::addClone(const cv::Mat& R_GtoC, const cv::Mat& p_G, int64_t timestamp_ns) {
    // 2026-05-09 telemetry fix: clear the per-addClone "dropped SLAM
    // feature ids" buffer so the caller (Tracker) sees only ids dropped
    // during THIS addClone's internal marginalisation passes. See
    // takeLastMarginalizedSlamFeatureIds() for the full rationale.
    last_marginalized_slam_feature_ids_.clear();
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
    //   old:  [ IMU(19) | C0..C{K-1}(6 each) | S0..S{N-1}(5 each) ]
    //   new:  [ IMU(19) | C0..C{K-1}(6 each) | C_new(6) | S0..S{N-1}(5 each) ]
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

    // Phase 1 Step 4a (post_v19_sprint_plan.md): try to re-anchor SLAM
    // features whose anchor clone is about to fall off the sliding window.
    // This MUST happen before the P_ splice and the window_.pop_front() so
    // that getClonePose() can still find both the old and new anchors.
    //
    // v23.8 (2026-05-11): switched new-anchor choice from window_.back()
    // (newest) to window_[1] (second-oldest).
    //
    // Rationale: re-anchor's Zc<0.01 geometry gate (reanchorSlamFeature
    // line 1659) was rejecting features when the new anchor's pose differs
    // significantly from the dropping anchor — feature projects behind /
    // too close to the new view. window_.back() has the MAXIMUM baseline
    // from window_.front(), so it has the highest chance of triggering
    // the gate. window_[1] has near-identical pose to window_.front()
    // (added in the very next addClone after it), so geometric change
    // across re-anchor is essentially zero. Trade-off: feature must
    // re-anchor every frame (since window_[1] becomes window_.front()
    // next marginalize), but each re-anchor is cheap and near-lossless.
    //
    // User-visible symptom this addresses: SLAM dots disappear within ~1 s
    // (= MAX_CLONES at ~14 FPS) of promotion, because the first re-anchor
    // attempt was failing and falling back to removeSlamFeature.
    const int dropped_id    = window_.front().state_id;
    const int new_anchor_id = (window_.size() >= 2) ? (window_.begin() + 1)->state_id : -1;

    std::vector<int> slots_to_remove;
    {
        auto& ec_re = navsight::eventCounters();
        for (int slot = static_cast<int>(slam_features_.size()) - 1; slot >= 0; --slot) {
            if (slam_features_[slot].anchor_clone_id != dropped_id) continue;
            if (new_anchor_id < 0) {
                // Window had only one clone — no surviving anchor candidate.
                ec_re.slam_reanchor_failed_no_clone.fetch_add(1, std::memory_order_relaxed);
                slots_to_remove.push_back(slot);
                continue;
            }
            if (reanchorSlamFeature(slot, new_anchor_id)) {
                ec_re.slam_reanchor_total.fetch_add(1, std::memory_order_relaxed);
            } else {
                // Geometry failed (Zc <= 0.01, NaN, etc.) — fall back to remove.
                ec_re.slam_reanchor_failed_geometry.fetch_add(1, std::memory_order_relaxed);
                slots_to_remove.push_back(slot);
            }
        }
    }

    if (full_initialized_ && !P_.empty() && P_.rows > IMU_STATE_DIM) {
        // Plan Step 3b (ADR-009): the oldest clone sits at rows/cols
        // [IMU_STATE_DIM, IMU_STATE_DIM + CLONE_DIM). The SLAM block sits at
        // the END of P_. Drop only the clone rows/cols by copy-splice — IMU,
        // remaining clones, and SLAM rows/cols all keep their cross-terms
        // intact. (We do NOT zero out the marginalised clone's contribution
        // to the remaining state via Schur — for the OpenVINS-style sliding
        // window the discard-only "drop" is the standard, since the clone
        // has no measurements yet not folded into IMU/SLAM cross-terms.)
        const int marg_idx   = IMU_STATE_DIM;  // First clone starts at IMU_STATE_DIM (19)
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

    // Phase 1 Step 4a: features successfully re-anchored above now have
    // anchor_clone_id == new_anchor_id and survive this marginalization. Only
    // the slots that failed to re-anchor (collected in slots_to_remove) are
    // dropped here. Pre-Step-4a behaviour was to drop ALL features anchored at
    // the dropping clone — that was the "flashing dots" bug (v22 had
    // slam_promotions_total=65, slam_lifetime_count=0, i.e. EVERY feature
    // dying before accumulating lifetime).
    //
    // Order matters: the P_ splice above already shifted the SLAM block up by
    // CLONE_DIM, so `slamBlockStart()` (which uses window_.size()) is wrong
    // until we pop the front clone. Pop first, then call removeSlamFeature.
    // slots_to_remove is in reverse-slot order (we iterated --slot above) so
    // erase() inside removeSlamFeature doesn't shift the remaining slots out
    // from under the loop.
    window_.pop_front();
    for (int slot : slots_to_remove) {
        // 2026-05-09 telemetry: capture the dropped feature_id BEFORE
        // removeSlamFeature erases the entry. Tracker drains this list after
        // addClone() and forwards each id to FeatureManager::setSlamSlot(-1)
        // + dropLifecycle so the lifetime-histogram demotion path runs.
        const int dropped_fid = slam_features_[slot].feature_id;
        if (dropped_fid >= 0) {
            last_marginalized_slam_feature_ids_.push_back(dropped_fid);
        }
        removeSlamFeature(slot);
    }
}

std::vector<int> EKFState::takeLastMarginalizedSlamFeatureIds() {
    // Move-and-clear so the caller owns a unique copy and the next addClone()
    // starts from an empty buffer. Cheap — typically 0-3 entries per call.
    std::vector<int> out = std::move(last_marginalized_slam_feature_ids_);
    last_marginalized_slam_feature_ids_.clear();
    return out;
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
                                 const cv::Mat& R_noise, bool apply_huber) {
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
    // v23.14 (2026-05-12): skip Huber for hard-physical-constraint updates
    // (gravity-alignment). For those, large normalised residuals ARE the
    // signal the EKF needs to act on — they reflect attitude error the
    // Kalman update is supposed to absorb. Huber-rejecting them creates a
    // self-reinforcing trap where the state never gets corrected. Caller
    // sets apply_huber=false when the input is a physical constraint
    // already pre-validated by a gate (e.g. updateGravityAlignment's accel
    // magnitude + gyro band gate in Tracker.cpp).
    if (apply_huber) {
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
        if (!cv::invert(S, S_inv, cv::DECOMP_SVD)) {
            LOGW("applyMSCKFUpdate: both Cholesky and SVD inversion failed — update dropped (NaN/Inf in S?)");
            navsight::eventCounters().applyMSCKFUpdate_inversion_failed.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }
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
    if (dx.rows >= IMU_STATE_DIM && damping < 1.0) {
        for (int i = 12; i < 15; i++) {  // δp is always at rows 12-14
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

        // Step 8a (ADR-014): apply time-offset correction (row 15 = δt_d).
        // Clamp to ±100 ms — physical camera-IMU lag cannot exceed this on
        // any mobile device (Li & Mourikis 2014, §III-B practical bound).
        t_offset_cam_imu_ += dx.at<double>(15, 0);
        {
            double td_unclamped = t_offset_cam_imu_;
            double td_clamped   = std::max(-0.1, std::min(0.1, td_unclamped));
            if (td_clamped != td_unclamped) {
                LOGW("applyMSCKFUpdate: time offset clamped from %.4f s to %.4f s (±100ms bound saturated)",
                     td_unclamped, td_clamped);
                navsight::eventCounters().time_offset_clamped.fetch_add(
                    1, std::memory_order_relaxed);
            }
            t_offset_cam_imu_ = td_clamped;
        }

        // Step 8b: R_bc update SKIPPED (NavSight 2026-05-09 Step 7 fix).
        // Clones bake R_bc at storage time, and H_bc is disabled in both
        // applyMSCKFFeature and slamReprojectionJacobian — so δφ_bc has no
        // measurement coupling and Kalman gain on those rows is always zero.
        // Defensive skip prevents process-noise drift from silently moving
        // R_bc and desynchronising clone storage from live R_bc.
        //
        // PHASE_A_8B_REENABLE_2026_05_19 REVERTED — Phase A attempt to
        // re-enable produced dphi_bc=0 in 8769/8769 MSCKF events. H_bc was
        // perfectly correlated with H_clone_theta (both = dproj_dpC · -skew(p_C_F))
        // → linearly dependent columns → null-space projection killed both.
        // Architectural incompatibility with Option C clone storage; would
        // need un-baking R_bc from clones to fix properly.
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

    // 2026-05-16 v26 walk diagnostic: per-block correction magnitudes so we
    // can see WHICH error-state component is being moved by each MSCKF
    // update. v26 evidence: max_correction=0.59 rad on the first MSCKF
    // update at bootstrap — that magnitude could go entirely into δθ
    // (orientation), or into δb_g (gyro bias), or into δp (position), or be
    // split. Without per-block logging we can't tell which.
    // Block layout (per IMU error-state §):
    //   0-2:   δθ      (rotation rad)
    //   3-5:   δb_g    (gyro bias rad/s)
    //   6-8:   δv      (velocity m/s)
    //   9-11:  δb_a    (accel bias m/s²)
    //   12-14: δp      (position m)
    //   15:    δt_d    (camera-IMU time offset s)
    //   16-18: δφ_bc   (extrinsics rad — skipped per defensive policy)
    // L2 norm of each block; for δθ also report as deg for legibility.
    if (dx.rows >= IMU_STATE_DIM) {
        auto blockNorm = [&dx](int row0, int n_rows) {
            double s = 0.0;
            for (int i = 0; i < n_rows; ++i) {
                const double v = dx.at<double>(row0 + i, 0);
                s += v * v;
            }
            return std::sqrt(s);
        };
        const double n_dtheta = blockNorm(0, 3);
        const double n_dbg    = blockNorm(3, 3);
        const double n_dv     = blockNorm(6, 3);
        const double n_dba    = blockNorm(9, 3);
        const double n_dp     = blockNorm(12, 3);
        const double n_dtd    = std::abs(dx.at<double>(15, 0));
        const double n_dphi   = blockNorm(16, 3);
        // 2026-05-18 diag A+B: per-axis dbg AND per-axis dtheta.
        // - dbg_xyz attributes b_g leaks across update callers.
        // - dtheta_xyz lets us see WORLD-Z (yaw) component of each rotation
        //   correction. v47/v48 showed EKF and Madgwick diverge at -0.35°/s
        //   on average, with both responding to gyro but visual updates
        //   pulling EKF independently. Summing dtheta_xyz[2] across update
        //   call sites identifies which is the systematic-bias source for
        //   the V-shape under-rotation (engine trace observation that EKF
        //   under-rotates fast turns by ~20°/turn).
        LOGI("MSCKF_DX_BLOCKS: dtheta_rad=%.5f dtheta_deg=%.3f "
             "dtheta_xyz=(%+.5f,%+.5f,%+.5f) "
             "dbg=%.5f dbg_xyz=(%+.5f,%+.5f,%+.5f) "
             "dv=%.5f dba=%.5f dp=%.4f dtd=%.5f dphi_bc=%.5f",
             n_dtheta, n_dtheta * 180.0 / M_PI,
             dx.at<double>(0, 0), dx.at<double>(1, 0), dx.at<double>(2, 0),
             n_dbg,
             dx.at<double>(3, 0), dx.at<double>(4, 0), dx.at<double>(5, 0),
             n_dv, n_dba, n_dp, n_dtd, n_dphi);
    }
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
    // Step 8b: publish current extrinsics deviation to event_summary JSON.
    // Throttled to every 30 MSCKF updates (~1 Hz at typical keyframe cadence)
    // because getExtrinsicsAngleDeg() triggers a Rodrigues computation on the
    // hot path. The counter is not thread-safe, but a missed increment is benign.
    if (++extr_log_skip_ >= 30) {
        extr_log_skip_ = 0;
        const double angle_deg = getExtrinsicsAngleDeg();
        const long long angle_mdeg = static_cast<long long>(angle_deg * 1000.0 + 0.5);
        navsight::eventCounters().extrinsics_rotation_angle_mdeg.store(
            angle_mdeg, std::memory_order_relaxed);
        const long long td_us = static_cast<long long>(t_offset_cam_imu_ * 1e6 + 0.5);
        navsight::eventCounters().cam_imu_time_offset_us.store(
            td_us, std::memory_order_relaxed);
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

    // 2026-05-12: post Option C (2026-05-09) clones store R_GtoC = R_bc·R_GtoI
    // (world→camera, see Tracker.cpp:1919), NOT world→body. Recover the
    // clone's world→body rotation by stripping R_bc:
    //     R_GtoI(clone) = R_bc.t() * R_clone.
    // The clone covariance δθ_c is the LEFT perturbation of R_GtoC, so post
    // Option C it lives in CAMERA frame. The body-frame measurement
    // linearisation
    //     r ≈ δθ_current - R_meas_body · δθ_c(body)
    // requires δθ_c(body) = R_bc.t() · δθ_c(camera). The clone-column
    // Jacobian therefore picks up an extra R_bc.t() factor compared with
    // the pre-Option-C form (which was `-R_meas_body` directly).
    cv::Mat R_bc_cv(3, 3, CV_64F);
    for (int rr = 0; rr < 3; ++rr) {
        for (int cc = 0; cc < 3; ++cc) {
            R_bc_cv.at<double>(rr, cc) = R_bc_(rr, cc);
        }
    }
    cv::Mat R_GtoI_clone = R_bc_cv.t() * R_clone;

    // Predicted body-frame rotation from clone to current:
    //   R_pred = R_GtoI_(current) * R_GtoI(clone).t()
    cv::Mat R_pred = R_GtoI_ * R_GtoI_clone.t();

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

    // Jacobian (3 x dim).
    //   +I on current-pose δθ block (cols 0..2)
    //   -R_meas_body · R_bc.t() on clone-pose δθ_c block (cols
    //   clone_cov_idx + 0..2), because δθ_c lives in CAMERA frame and the
    //   residual lives in body frame.
    cv::Mat H_clone_block = -R_meas_body * R_bc_cv.t();
    cv::Mat H = cv::Mat::zeros(3, dim, CV_64F);
    for (int i = 0; i < 3; i++) {
        H.at<double>(i, i) = 1.0;
        for (int j = 0; j < 3; j++) {
            H.at<double>(i, clone_cov_idx + j) = H_clone_block.at<double>(i, j);
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

    // Per-block Mahalanobis diagnostic (added 2026-05-07).
    // m2_R / m2_p are the 3-DOF marginal Mahalanobis distances over
    // rotation-only and position-only blocks of S. Each has its own
    // χ²(0.999, 3) ≈ 16.27 budget. They DON'T sum to m2 because S has
    // off-diagonal cross terms, but they do reveal which block dominates
    // a chi²-reject. Cheap (two 3×3 inverses).
    //
    // If m2_R alone ≥ 16.27 → rotation residual saturates the budget
    // (heading-misalignment fingerprint). If m2_p ≥ 16.27 → position
    // residual saturates (drift-too-far fingerprint). If both are
    // moderate but m2 > 22.5 → cross-correlation between r_R and r_p
    // is the culprit (rare; signals R_GtoI vs p_G inconsistency).
    double m2_R_only = 0.0, m2_p_only = 0.0;
    {
        cv::Mat S_R = S(cv::Range(0, 3), cv::Range(0, 3));
        cv::Mat S_p = S(cv::Range(3, 6), cv::Range(3, 6));
        cv::Mat S_R_inv, S_p_inv;
        if (cv::invert(S_R, S_R_inv, cv::DECOMP_CHOLESKY) ||
            cv::invert(S_R, S_R_inv, cv::DECOMP_SVD)) {
            cv::Mat m_R = r_R.t() * S_R_inv * r_R;
            m2_R_only = m_R.at<double>(0, 0);
        }
        if (cv::invert(S_p, S_p_inv, cv::DECOMP_CHOLESKY) ||
            cv::invert(S_p, S_p_inv, cv::DECOMP_SVD)) {
            cv::Mat m_p = r_p.t() * S_p_inv * r_p;
            m2_p_only = m_p.at<double>(0, 0);
        }
    }

    // Log residual + state for diagnosis BEFORE the chi² gate, so a
    // chi²-reject still leaves a trace we can use to figure out whether
    // the residual itself is wrong (frame mismatch, stale candidate
    // pose) or the predicted variance S is too tight.
    LOGI("LC_ABS: r_R=[%.3f %.3f %.3f] r_p=[%.3f %.3f %.3f] "
         "p_G=[%.3f %.3f %.3f] target_p=[%.3f %.3f %.3f] "
         "var_R=%.4e var_p=%.4f m2=%.3f m2_R=%.3f m2_p=%.3f thresh=%.1f",
         r_R.at<double>(0, 0), r_R.at<double>(1, 0), r_R.at<double>(2, 0),
         r_p.at<double>(0, 0), r_p.at<double>(1, 0), r_p.at<double>(2, 0),
         p_G_.at<double>(0, 0), p_G_.at<double>(1, 0), p_G_.at<double>(2, 0),
         target_p_world.at<double>(0, 0), target_p_world.at<double>(1, 0),
         target_p_world.at<double>(2, 0),
         sR, sp, m2, m2_R_only, m2_p_only, kChi2Threshold);

    if (m2 > kChi2Threshold) {
        navsight::eventCounters().loop_closure_chi2_rejected.fetch_add(
            1, std::memory_order_relaxed);
        return false;
    }

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

    // H_yaw derivation (NavSight 2026-05-09 left-mult / world-frame δθ).
    //
    // World frame: ENU Z-up. yaw is rotation around world Z axis.
    // R_GtoI_ is world→body. The post-update site at line 670 applies
    //     R_GtoI_new = Rodrigues(δθ) * R_GtoI_old      (left-multiply)
    // which means the EKF error state δθ is in WORLD frame (not body
    // frame). Under this convention, the analytical derivative of yaw
    // w.r.t. δθ is the same regardless of body orientation:
    //
    //   For any state R_old, R_new = exp(δθ_world) * R_old. Expanding:
    //     R_new[i,0] = R_old[i,0] + (skew(δθ) * R_old)[i,0]
    //   For δθ = (0,0,ε):
    //     skew((0,0,ε)) * R_old[*,0] = (-ε·R_old[1,0], +ε·R_old[0,0], 0)
    //     R_new[0,0] = R_old[0,0] - ε·R_old[1,0]
    //     R_new[1,0] = R_old[1,0] + ε·R_old[0,0]
    //     yaw_new = atan2(R_new[1,0], R_new[0,0]); ∂yaw/∂ε = +1.
    //   Symmetric derivations give ∂yaw/∂δθ_x_world = 0 and ∂yaw/∂δθ_y_world = 0.
    //
    // Therefore h_world = e_z_world = (0, 0, +1), CONSTANT, independent of R.
    //
    // Pre-2026-05-09 the formula was `±R_GtoI_ * e_z_world` (body-frame
    // style). For pure-yaw states the formula coincides with e_z_world
    // because Z-rotations don't move e_z, so the bug was invisible in flat
    // poses. For typical walking poses (phone vertical, pitch ≈ −π/2),
    // R_GtoI * e_z_world = body Y-axis ≠ e_z_world, and the EKF's yaw
    // correction was distributed over the wrong body axes — driving Step
    // 7 chi² to reject every loop closure (sim 1778262445638: 100/100
    // chi² rejects, vyaw drifts away from Madgwick by 50° in 117 s).
    cv::Mat h_world = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    cv::Mat& h_body = h_world;  // alias kept to minimise diff churn below

    cv::Mat H = cv::Mat::zeros(1, dim, CV_64F);
    H.at<double>(0, 0) = h_body.at<double>(0);
    H.at<double>(0, 1) = h_body.at<double>(1);
    H.at<double>(0, 2) = h_body.at<double>(2);
    (void)roll; (void)pitch;  // alignment carried in yaw_pred, not in H

    cv::Mat res = (cv::Mat_<double>(1, 1) << res_val);
    cv::Mat R_noise = (cv::Mat_<double>(1, 1) << std::max(var, 1e-6));
    // 2026-05-18 diag B: log every visual-yaw injection so we can see if
    // these are pumping EKF yaw during walking. v42 showed Madgwick yaw
    // drift -0.31°/s during walking with b_g stable (so b_g is innocent);
    // updateGravityAlignedYaw injecting wrong residuals into EKF then
    // bleeding via b_g push is the next-most-likely suspect.
    LOGI("LC_YAW_FIRE: yaw_meas=%.2f° yaw_pred=%.2f° residual=%+.3f° var=%.4e",
         yaw_meas * 180.0 / M_PI, yaw_pred * 180.0 / M_PI,
         res_val * 180.0 / M_PI, var);
    applyMSCKFUpdate(H, res, R_noise);
    return true;
}

bool EKFState::updatePDRStep(double dx_world, double dy_world, double var) {
    if (!full_initialized_ || P_.empty()) return false;
    int dim = P_.rows;

    // 2-DOF position constraint on world X (East) and Y (North); Z (Up)
    // is gravity-aligned and handled by the IMU propagation. Treat as a
    // direct observation of δp_x and δp_y (state, not delta). The caller
    // accumulates step displacement into the desired anchor before this
    // call. Z-up ENU world (post 2026-05-07 alignment).
    cv::Mat res = (cv::Mat_<double>(2, 1) << dx_world, dy_world);

    cv::Mat H = cv::Mat::zeros(2, dim, CV_64F);
    H.at<double>(0, 12) = 1.0;  // ∂(dx)/∂δp_x  — East
    H.at<double>(1, 13) = 1.0;  // ∂(dy)/∂δp_y  — North

    cv::Mat R_noise = cv::Mat::eye(2, 2, CV_64F) * std::max(var, 1e-6);
    applyMSCKFUpdate(H, res, R_noise);
    return true;
}

// ── Phase 1 Step 6.3 — fixed-landmark pose-only measurement update ──────────
//
// ORB-SLAM3 "Tracking" pattern (Campos et al. 2021, IEEE TR0, §III.B):
// the camera pose is updated against fixed map points so the landmark
// columns of the Jacobian are zero. The active blocks are δθ (rows 0..2)
// and δp_G (rows 12..14) of the IMU error state; everything else picks
// up corrections only through the covariance correlation in applyMSCKFUpdate.
//
// Sign convention — matches updateAbsolutePose (EKFState.cpp:1285) and the
// left-perturbation derivation documented there: the EKF carries world→body
// as R_GtoI with the update model `R_GtoI ← Rodrigues(dθ) · R_GtoI`
// (applyMSCKFUpdate line 968), i.e. LEFT perturbation in the body frame.
// Under that convention, for the residual r = z − h(x) the camera-frame
// derivative chain is:
//
//   p_I = R_GtoI · (p_world − p_G)
//   p_C = R_bc · p_I
//   ∂p_C / ∂δθ   = −R_bc · skew(p_I)
//   ∂p_C / ∂δp_G = −R_bc · R_GtoI
//
// and the residual gradient gets a MINUS sign on top of that (because
// r = z − h(x), so ∂r/∂x = −∂h/∂x). The two minus signs cancel for the
// δθ block via the skew anti-symmetry argument that already underpins
// updateRelativeRotation. Net result: H is built as
//     H_θ = J_proj · (−R_bc · skew(p_I))
//     H_p = J_proj · (−R_bc · R_GtoI)
// exactly as the math contract specifies. This is the same sign the
// existing visual updaters use, so the Joseph form in applyMSCKFUpdate
// drives the correction in the direction that zeros the residual.
//
// Per-obs gates:
//   1. depth gate (camera-frame Zc band) — matches LandmarkMap's
//      [kDepthMinM, kDepthMaxM] so observations folded back into the EKF
//      have the same physical envelope as the observations folded into
//      LandmarkMap during keyframe storage.
//   2. in-image gate — only ever fires when something upstream is wrong
//      (data association handed us a landmark that does not project into
//      this frame). We count it instead of silently masking it.
//   3. per-obs χ² gate — χ²(0.95, 2-DOF) = 5.991 (standard table value).
//   4. per-obs Huber kernel — δ = 3.0 px (eyeballed to project a 3σ
//      residual under nominal σ ≈ 1.0 px; matches the upstream MSCKF
//      Huber semantics qualitatively). Hard reject above 5·δ; linear
//      de-weighting in (δ, 5·δ).
EKFState::LandmarkUpdateResult EKFState::applyLandmarkObservations(
    const std::vector<LandmarkObservation>& obs,
    const cv::Matx33d& R_bc,
    double fx, double fy,
    double cx, double cy,
    int img_width, int img_height) {

    LandmarkUpdateResult result;

    // χ²(0.95, 2-DOF) — Abramowitz & Stegun Table 26.8 (standard chi-square
    // critical value for the two-tailed 95% confidence level at 2 DOF).
    static constexpr double kLandmarkChi2Gate95 = 5.991;
    // Huber threshold in pixels — eyeball ~3σ on σ_px ≈ 1.0 px; matches the
    // qualitative behaviour of the MSCKF Huber kernel (EKFState.cpp:893,
    // MSCKF_HUBER_DELTA = √χ²(0.95, 2) = 2.4477) applied per-residual-row.
    static constexpr double kLandmarkHuberPx = 3.0;
    // Camera-frame depth band. Must match LandmarkMap's kDepthMinM /
    // kDepthMaxM so a landmark accepted at storage time is also accepted
    // at re-observation time; mismatched bands would silently shrink the
    // observable map.
    static constexpr double kLandmarkDepthMinM = 0.5;
    static constexpr double kLandmarkDepthMaxM = 50.0;

    if (!full_initialized_ || P_.empty()) {
        LOGI("LM_APPLY: rejected_all reason=not_initialized obs=%zu",
             obs.size());
        return result;
    }
    if (obs.empty()) {
        return result;
    }
    if (!(fx > 0.0) || !(fy > 0.0) || img_width <= 0 || img_height <= 0) {
        LOGI("LM_APPLY: rejected_all reason=bad_intrinsics fx=%.2f fy=%.2f W=%d H=%d obs=%zu",
             fx, fy, img_width, img_height, obs.size());
        return result;
    }

    const int dim = P_.rows;
    if (dim < IMU_STATE_DIM) {
        return result;
    }

    // Local skew helper. Matches the convention used by propagateIMU
    // (EKFState.cpp:228) — row 0 of skew(v) cross-multiplies to v × x.
    auto skew = [](const cv::Vec3d& v) -> cv::Matx33d {
        return cv::Matx33d(
             0.0,  -v[2],   v[1],
             v[2],   0.0,  -v[0],
            -v[1],  v[0],   0.0);
    };

    // Read state once. R_GtoI is world→body (left-perturbation convention).
    cv::Matx33d R_GtoI;
    cv::Vec3d   p_G;
    {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_GtoI(i, j) = R_GtoI_.at<double>(i, j);
            }
            p_G[i] = p_G_.at<double>(i, 0);
        }
    }

    // ── Pass 1: per-obs gating + Jacobian row build ──────────────────────
    //
    // Stack accepted rows in a 2N × dim matrix with zeros except at cols
    // 0..2 (δθ) and 12..14 (δp_G). We also stack the per-obs noise
    // (Huber-weighted) into the diagonal of R_noise and the residual into
    // r_stack. Pre-gate Mahalanobis is computed using only the 6×6
    // pose-pose covariance block; that's exactly the variance the
    // pose-only Jacobian sees through S = H·P·H^T + R_noise (off-diagonal
    // δp_G ↔ velocity / bias coupling shows up only after the full Joseph
    // step propagates correlation back into those rows).
    cv::Matx66d P_pose_pose;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P_pose_pose(i,     j)     = P_.at<double>(0  + i, 0  + j);  // δθ ↔ δθ
            P_pose_pose(i,     j + 3) = P_.at<double>(0  + i, 12 + j);  // δθ ↔ δp
            P_pose_pose(i + 3, j)     = P_.at<double>(12 + i, 0  + j);  // δp ↔ δθ
            P_pose_pose(i + 3, j + 3) = P_.at<double>(12 + i, 12 + j);  // δp ↔ δp
        }
    }

    struct AcceptedRow {
        cv::Matx<double, 2, 3> H_theta;  // ∂r / ∂δθ   (2×3)
        cv::Matx<double, 2, 3> H_p;      // ∂r / ∂δp_G (2×3)
        cv::Vec2d              r;        // residual (huber-weighted)
        double                 sigma_px; // measurement noise σ (huber-weighted)
        int                    landmark_id;
    };
    std::vector<AcceptedRow> rows;
    rows.reserve(obs.size());

    for (const auto& o : obs) {
        // p_I = R_GtoI · (p_world − p_G)
        const cv::Vec3d dpw(o.p_world[0] - p_G[0],
                            o.p_world[1] - p_G[1],
                            o.p_world[2] - p_G[2]);
        const cv::Vec3d p_I = R_GtoI * dpw;

        // p_C = R_bc · p_I
        const cv::Vec3d p_C = R_bc * p_I;

        // Depth gate.
        if (!(p_C[2] >= kLandmarkDepthMinM) || !(p_C[2] <= kLandmarkDepthMaxM)) {
            result.rejected_depth++;
            LOGI("LM_APPLY_REJECT: reason=depth fid=%d depth_m=%.2f",
                 o.landmark_id, p_C[2]);
            continue;
        }

        // Predicted pixel.
        const double inv_z = 1.0 / p_C[2];
        const double u_pred = fx * p_C[0] * inv_z + cx;
        const double v_pred = fy * p_C[1] * inv_z + cy;

        // In-image gate.
        if (!(u_pred >= 0.0) || !(u_pred < static_cast<double>(img_width)) ||
            !(v_pred >= 0.0) || !(v_pred < static_cast<double>(img_height))) {
            result.rejected_outside_image++;
            LOGI("LM_APPLY_REJECT: reason=image fid=%d u_pred=%.1f v_pred=%.1f W=%d H=%d",
                 o.landmark_id, u_pred, v_pred, img_width, img_height);
            continue;
        }

        // Residual r = z − h(x). 2×1.
        const double r_u = static_cast<double>(o.pixel_meas.x) - u_pred;
        const double r_v = static_cast<double>(o.pixel_meas.y) - v_pred;

        // Pixel Jacobian wrt camera-frame point (2×3):
        //   J_proj = (1/z) · [fx, 0, -fx·x/z;
        //                      0, fy, -fy·y/z]
        cv::Matx<double, 2, 3> J_proj;
        J_proj(0, 0) = fx * inv_z;
        J_proj(0, 1) = 0.0;
        J_proj(0, 2) = -fx * p_C[0] * inv_z * inv_z;
        J_proj(1, 0) = 0.0;
        J_proj(1, 1) = fy * inv_z;
        J_proj(1, 2) = -fy * p_C[1] * inv_z * inv_z;

        // ∂p_C / ∂δθ   = −R_bc · skew(p_I)
        // ∂p_C / ∂δp_G = −R_bc · R_GtoI
        // (signs match updateAbsolutePose / updateRelativeRotation — see
        // header comment block above for the left-perturbation derivation.)
        const cv::Matx33d J_pC_theta = -R_bc * skew(p_I);
        const cv::Matx33d J_pC_p     = -R_bc * R_GtoI;

        const cv::Matx<double, 2, 3> H_theta = J_proj * J_pC_theta;
        const cv::Matx<double, 2, 3> H_p     = J_proj * J_pC_p;

        // ── Per-obs χ² (pose-only block) ─────────────────────────────────
        // Build H_pose (2×6) = [H_theta | H_p], then
        //   S = H_pose · P_pose_pose · H_pose^T + σ² · I_2
        cv::Matx<double, 2, 6> H_pose;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                H_pose(i, j)     = H_theta(i, j);
                H_pose(i, j + 3) = H_p(i, j);
            }
        }
        const double sigma_px_safe = std::max(o.sigma_px, 1e-3);
        const double sigma_sq      = sigma_px_safe * sigma_px_safe;

        const cv::Matx22d S = H_pose * P_pose_pose * H_pose.t()
                            + cv::Matx22d(sigma_sq, 0.0, 0.0, sigma_sq);
        const double det_S = S(0,0) * S(1,1) - S(0,1) * S(1,0);
        if (!(std::abs(det_S) > 1e-12) || !std::isfinite(det_S)) {
            result.rejected_chi2++;
            LOGI("LM_APPLY_REJECT: reason=chi2_singular fid=%d det_S=%.3e",
                 o.landmark_id, det_S);
            continue;
        }
        // S^-1 by closed-form 2×2 inverse.
        const double inv_det = 1.0 / det_S;
        const cv::Matx22d S_inv(
             S(1,1) * inv_det, -S(0,1) * inv_det,
            -S(1,0) * inv_det,  S(0,0) * inv_det);

        const cv::Vec2d r_vec(r_u, r_v);
        const cv::Vec2d Sinv_r = S_inv * r_vec;
        const double m2 = r_vec[0] * Sinv_r[0] + r_vec[1] * Sinv_r[1];

        if (!(m2 <= kLandmarkChi2Gate95) || !std::isfinite(m2)) {
            result.rejected_chi2++;
            LOGI("LM_APPLY_REJECT: reason=chi2 fid=%d m2=%.3f gate=%.3f",
                 o.landmark_id, m2, kLandmarkChi2Gate95);
            continue;
        }

        // ── Per-obs Huber kernel (pixel residual norm) ────────────────────
        const double r_px = std::sqrt(r_u * r_u + r_v * r_v);
        double huber_w = 1.0;
        if (r_px > kLandmarkHuberPx) {
            if (r_px > 5.0 * kLandmarkHuberPx) {
                result.rejected_huber++;
                LOGI("LM_APPLY_REJECT: reason=huber fid=%d r_px=%.2f",
                     o.landmark_id, r_px);
                continue;
            }
            huber_w = kLandmarkHuberPx / r_px;
        }

        // Accept. Stash the row, Huber-scale residual + sigma so the
        // downstream Joseph step sees a consistent (H, R, r) triple.
        // Row scaling by huber_w applied to both res and H is mathematically
        // equivalent to inflating R by 1/w² (the IRLS form used in
        // applyMSCKFUpdate); we follow the per-residual scale here for
        // clarity since each row has its own w.
        AcceptedRow row;
        row.H_theta     = H_theta * huber_w;
        row.H_p         = H_p     * huber_w;
        row.r           = cv::Vec2d(r_u * huber_w, r_v * huber_w);
        row.sigma_px    = sigma_px_safe;  // R uses unweighted σ; H/r carry the weight
        row.landmark_id = o.landmark_id;
        rows.push_back(row);

        result.accepted++;
        result.chi2_total += m2;  // accumulate ONLY accepted observations
    }

    // ── Pass 2: assemble and apply the batched update ────────────────────
    if (rows.empty()) {
        LOGI("LM_APPLY: obs=%zu accepted=0 rej_chi2=%d rej_huber=%d rej_depth=%d rej_image=%d chi2_total=0.00",
             obs.size(), result.rejected_chi2, result.rejected_huber,
             result.rejected_depth, result.rejected_outside_image);
        return result;
    }

    const int N = static_cast<int>(rows.size());
    cv::Mat H_stack = cv::Mat::zeros(2 * N, dim, CV_64F);
    cv::Mat r_stack = cv::Mat::zeros(2 * N, 1, CV_64F);
    cv::Mat R_stack = cv::Mat::zeros(2 * N, 2 * N, CV_64F);

    for (int i = 0; i < N; ++i) {
        const AcceptedRow& row = rows[i];
        for (int r_off = 0; r_off < 2; ++r_off) {
            const int dst_row = 2 * i + r_off;
            // δθ block (cols 0..2)
            for (int j = 0; j < 3; ++j) {
                H_stack.at<double>(dst_row, 0 + j) = row.H_theta(r_off, j);
            }
            // δp_G block (cols 12..14)
            for (int j = 0; j < 3; ++j) {
                H_stack.at<double>(dst_row, 12 + j) = row.H_p(r_off, j);
            }
            r_stack.at<double>(dst_row, 0) = row.r[r_off];
            // R_stack diagonal (per-residual σ²). Block-diagonal: residual
            // u and v of the same observation are uncorrelated noise samples
            // by the pinhole model, so the 2×2 sub-block is σ²·I.
            R_stack.at<double>(dst_row, dst_row) =
                row.sigma_px * row.sigma_px;
        }
    }

    // Hand off to the existing Joseph-form / Huber path. apply_huber=false
    // because we've already applied a per-observation Huber here; the
    // downstream Huber works per-row of the stacked matrix and would
    // double-de-weight the rows we've already softened. The downstream
    // χ² gate (rows in applyMSCKFUpdate) sees only the post-Huber
    // residuals so its hard-reject still has correct semantics.
    applyMSCKFUpdate(H_stack, r_stack, R_stack, /*apply_huber=*/false);

    LOGI("LM_APPLY: obs=%zu accepted=%d rej_chi2=%d rej_huber=%d rej_depth=%d rej_image=%d chi2_total=%.2f",
         obs.size(), result.accepted, result.rejected_chi2,
         result.rejected_huber, result.rejected_depth,
         result.rejected_outside_image, result.chi2_total);

    return result;
}

bool EKFState::updateGravityAlignment(const cv::Mat& accel_body, double var) {
    if (!full_initialized_ || P_.empty()) return false;
    if (accel_body.empty() || accel_body.rows != 3 || accel_body.cols != 1) return false;
    if (accel_body.type() != CV_64F) return false;

    // ── Validity gate from sensor physics, not magic numbers ──────────
    //
    // Reject samples with magnitude far from g — those are under linear
    // acceleration and the "accel ≈ -gravity" assumption breaks.
    //
    // Bounds derived from accelerometer 3σ noise floor:
    //   σ_acc ≈ 0.1 m/s² (typical Android MEMS; same value used as
    //                     accel_noise_sigma_ in EKFState ctor and as
    //                     UpdaterZeroVelocity::sigma_a default)
    //   3σ_acc ≈ 0.3 m/s²
    // Walking-induced linear-accel transients add another ~0.5 m/s² on
    // peaks, so an outer bound of g ± 0.8 m/s² admits walking but rejects
    // jolts (handheld drop, bus brakes). The EKF sees one accel sample
    // per frame; missed samples don't compound — we just skip those.
    constexpr double kGravityMagN = 9.81;          // m/s², standard Earth gravity
    constexpr double kAccelMagBand = 0.8;          // m/s² (3σ_acc + walking-band)
    const double a_norm = cv::norm(accel_body);
    if (a_norm < kGravityMagN - kAccelMagBand ||
        a_norm > kGravityMagN + kAccelMagBand) {
        return false;
    }

    // ── Observation model ─────────────────────────────────────────────
    //
    // Specific-force model: when body is approximately stationary or
    // moving at constant velocity, the accelerometer measures
    //
    //   accel_body  =  -g_body  =  -R_GtoI · g_world
    //
    // with g_world = (0, 0, -9.81) in Z-up ENU. So accel_body_normalised
    // points along body's "+up" direction:
    //
    //   z_obs  = accel_body / |accel_body|     in body frame  (measurement z)
    //   z_pred = R_GtoI · (0, 0, +1)           predicted body-frame +up = h(x)
    //
    // Residual:
    //   r = z_obs - z_pred = z - h(x)
    //
    // Jacobian. applyMSCKFUpdate is standard Kalman: K = P·H^T·S^{-1},
    // dx = K·r, state += dx with r = z − h and H = ∂h/∂x. So we MUST
    // pass H = ∂h/∂x, not ∂r/∂x.
    //
    // With left-perturbation R_GtoI_new = exp([δθ]×) · R_GtoI:
    //   z_pred_new ≈ z_pred + [δθ]× · z_pred = z_pred − [z_pred]× · δθ
    // so  ∂h / ∂δθ = −[z_pred]×.   (Other state slots: zero.)
    //
    // 2026-05-09 v14 sign fix: pre-fix code used H_theta = +[z_pred]×
    // (which is ∂r/∂δθ). With the wrong sign, every Kalman update
    // pushed δθ in the OPPOSITE direction — verified in v13 walk
    // (LC_SA r_norm climbed 1.5 → 8.28, b_a runaway to 0.36 m/s²
    // even with stationary phone). Same bug existed in updateStationaryAccel.
    //
    // Yaw observability: rotation about world-Z leaves body-Z unchanged →
    // the row in H_θ for δθ_z is zero. The 3D residual has rank-2
    // observability (roll + pitch); the EKF Kalman update naturally
    // applies correction only along the observed sub-space.
    cv::Mat z_obs = accel_body / a_norm;
    cv::Mat z_pred(3, 1, CV_64F);
    z_pred.at<double>(0, 0) = R_GtoI_.at<double>(0, 2);
    z_pred.at<double>(1, 0) = R_GtoI_.at<double>(1, 2);
    z_pred.at<double>(2, 0) = R_GtoI_.at<double>(2, 2);
    cv::Mat r = z_obs - z_pred;

    // H_θ = -[z_pred]× (negated skew-symmetric of z_pred).
    const double zx = z_pred.at<double>(0, 0);
    const double zy = z_pred.at<double>(1, 0);
    const double zz = z_pred.at<double>(2, 0);
    cv::Mat H_theta = (cv::Mat_<double>(3, 3) <<
            0.0,  zz, -zy,
            -zz, 0.0,  zx,
             zy, -zx, 0.0);

    int dim = P_.rows;
    cv::Mat H = cv::Mat::zeros(3, dim, CV_64F);
    H_theta.copyTo(H(cv::Range(0, 3), cv::Range(0, 3)));

    // ── Measurement noise ─────────────────────────────────────────────
    //
    // Caller supplies var in (m/s²)². Convert to unit-vector variance
    // by dividing by g²:  σ²_unit = σ²_acc / g²  ≈ 0.01 / 96.24 ≈ 1.04e-4.
    // Same per-axis variance for all three rows (isotropic noise model).
    const double var_unit = std::max(1e-8, var / (kGravityMagN * kGravityMagN));
    cv::Mat R_noise = cv::Mat::eye(3, 3, CV_64F) * var_unit;

    // navsight-vio-specialist Playbook A6: every measurement update
    // emits a single LOGI line so we can replay the gate behaviour
    // from logcat without re-instrumenting the build. Format mirrors
    // the LC_* family used by other updates (gate value, residual norm,
    // observation variance) — see EventCounters.h for the chi² counters
    // these lines feed cross-checking against.
    LOGI("LC_GA: a_norm=%.4f r_norm=%.6f var_unit=%.6e",
         a_norm, cv::norm(r), var_unit);
    // v23.14: bypass Huber. Gravity is a HARD physical constraint with
    // pre-validated input (accel magnitude band + gyro gate already applied
    // upstream in Tracker.cpp). Huber-rejecting these treats real attitude
    // errors as "outliers" and creates a self-reinforcing trap where R_GtoI
    // never gets corrected, gyro bias integrates unchecked, clone poses
    // drift, and SLAM dots wander. Pre-fix logcat: 1882/2011 = 93.6% Huber
    // rejection rate on these 3-row updates → R_GtoI corrections of only
    // ~0.003 rad per fired update → no chance to overcome real attitude
    // drift.
    applyMSCKFUpdate(H, r, R_noise, /*apply_huber=*/false);
    return true;
}

bool EKFState::updateStationaryAccel(const cv::Mat& accel_body, double sigma_a) {
    if (!full_initialized_ || P_.empty()) return false;
    if (accel_body.empty() || accel_body.rows != 3 || accel_body.cols != 1) return false;
    if (accel_body.type() != CV_64F) return false;
    if (P_.rows < IMU_STATE_DIM) return false;  // need full b_a slot at rows 9-11

    // ── Specific-force model under body-stationary assumption ─────────
    //
    // Body acceleration ≈ 0 (caller guaranteed by ZUPT detector).
    // IMU specific-force model:
    //   a_imu  =  -R_GtoI · g_world  +  b_a  +  noise         (= z = h(x) + n)
    // In Z-up ENU,  g_world = (0, 0, -9.81)  → -R_GtoI·g_world = 9.81·R_GtoI[:,2]
    //   h(x)       =  9.81 · R_GtoI[:,2]  +  b_a
    //   r          =  a_imu_measured  -  h(x)        (3 × 1, m/s²)
    //
    // Jacobian H = ∂h/∂x (standard Kalman convention; applyMSCKFUpdate
    // does dx = K·r with r = z−h and K = P·H^T·S^{-1}):
    //
    //   ∂(R_GtoI[:,2]) / ∂δθ  =  -[R_GtoI[:,2]]×          (left perturbation)
    //   ∂h / ∂δθ              = -9.81 · [R_GtoI[:,2]]×    (3 × 3)
    //   ∂h / ∂b_a             = +I                         (3 × 3)
    //
    // 2026-05-09 v14 sign fix: pre-fix code passed +9.81·[z]× and -I
    // (which is ∂r/∂x, not ∂h/∂x). With both signs flipped the EKF
    // pushed δθ AND δb_a in opposing directions on every measurement —
    // direct cause of the LC_SA divergence in v13 logcat (b_a ran from 0
    // to (0.08, 0.23, -0.26) within 80 s while residual climbed to 8.28
    // m/s², physically impossible for a body the ZUPT detector confirmed
    // stationary).
    //
    // Yaw observability: same as updateGravityAlignment — [R_GtoI[:,2]]×
    // has null direction along R_GtoI[:,2] itself, so yaw is structurally
    // unobservable from gravity alone (correctly so).
    //
    // Noise:  R_noise = σ_a² · I  (isotropic accel measurement noise).
    constexpr double kGravityMagS = 9.81;

    cv::Mat z_dir(3, 1, CV_64F);
    z_dir.at<double>(0, 0) = R_GtoI_.at<double>(0, 2);
    z_dir.at<double>(1, 0) = R_GtoI_.at<double>(1, 2);
    z_dir.at<double>(2, 0) = R_GtoI_.at<double>(2, 2);

    cv::Mat predicted = kGravityMagS * z_dir + b_a_;
    cv::Mat r = accel_body - predicted;

    // -[R_GtoI[:,2]]× — negated skew-symmetric of body-frame world-up direction.
    const double zx = z_dir.at<double>(0, 0);
    const double zy = z_dir.at<double>(1, 0);
    const double zz = z_dir.at<double>(2, 0);
    cv::Mat neg_skew_z = (cv::Mat_<double>(3, 3) <<
            0.0,  zz, -zy,
            -zz, 0.0,  zx,
             zy, -zx, 0.0);

    int dim = P_.rows;
    cv::Mat H = cv::Mat::zeros(3, dim, CV_64F);
    cv::Mat H_theta = kGravityMagS * neg_skew_z;
    H_theta.copyTo(H(cv::Range(0, 3), cv::Range(0, 3)));

    // δb_a slot is rows 9-11 in the IMU state layout
    // (δθ:0-2, δb_g:3-5, δv:6-8, δb_a:9-11, δp:12-14, δt_d:15, δφ_bc:16-18).
    cv::Mat H_ba = cv::Mat::eye(3, 3, CV_64F);   // ∂h/∂b_a = +I
    H_ba.copyTo(H(cv::Range(0, 3), cv::Range(9, 12)));

    const double var_a = std::max(1e-6, sigma_a * sigma_a);
    cv::Mat R_noise = cv::Mat::eye(3, 3, CV_64F) * var_a;

    // navsight-vio-specialist Playbook A6 — every measurement update
    // emits one LOGI line so we can replay gate behaviour from logcat.
    LOGI("LC_SA: a_norm=%.4f r_norm=%.6f b_a=(%.4f,%.4f,%.4f) var_a=%.6e",
         cv::norm(accel_body), cv::norm(r),
         b_a_.at<double>(0, 0), b_a_.at<double>(1, 0), b_a_.at<double>(2, 0),
         var_a);
    applyMSCKFUpdate(H, r, R_noise);
    return true;
}

void EKFState::setPosition(const cv::Mat& p_G) {
    if (p_G.empty() || p_G.rows != 3 || p_G.cols != 1) return;
    p_G.convertTo(p_G_, CV_64F);
}

void EKFState::setRotation(const cv::Mat& R_GtoI) {
    if (R_GtoI.empty() || R_GtoI.rows != 3 || R_GtoI.cols != 3) return;
    // Replace only R_GtoI_ in-place. Position, velocity, biases, td, R_bc,
    // covariance and the clone window are deliberately left untouched —
    // this call is a bootstrap correction for the rotation seed only,
    // and the EKF has only been propagating for milliseconds when it
    // fires (between ekf_.initializeFull and Kotlin's UI-thread dispatch
    // of handleVioInitialized). The clone window typically holds 1
    // identity-rotated clone at this point; subsequent clones stamp
    // ekf_.getRotation() so they pick up the corrected R from here.
    R_GtoI.convertTo(R_GtoI_, CV_64F);
}

double EKFState::getYaw(double roll, double pitch) const {
    cv::Mat Rx, Ry;
    cv::Rodrigues(cv::Vec3d(roll, 0.0, 0.0), Rx);
    cv::Rodrigues(cv::Vec3d(0.0, pitch, 0.0), Ry);
    cv::Mat R_align = Ry * Rx;  // tilt-removal sandwich (Madgwick ZYX)
    cv::Mat R_aligned = R_align * R_GtoI_ * R_align.t();
    // Z-up nav convention (matches IMUPreintegrator::getHeading and
    // Tracker::setInitialHeading): yaw is rotation around world-Z axis,
    // CW-positive nav heading (North=0, East=+π/2), range [-π, π].
    //
    // World→body matrix R_GtoI_ for body at compass heading +ψ has the
    // structure built by setInitialHeading at Tracker.cpp:322:
    //     R_GtoI_ = [[cos ψ, -sin ψ, 0], [sin ψ, cos ψ, 0], [0, 0, 1]]
    // So R_aligned[1,0] = sin ψ and R_aligned[0,0] = cos ψ, giving:
    //     atan2(R_aligned[1,0], R_aligned[0,0]) = atan2(sin ψ, cos ψ) = ψ
    //
    // The pre-2026-05-07 (c1c15b2) form atan2(-R[0,2], R[0,0]) was Y-up:
    // it extracted yaw around world-Y, which doesn't match the rest of the
    // EKF's Z-up world (gravity along -Z, Madgwick filter Z-up). It also
    // gave degenerate output for the Tracker-init Rz matrix (R[0,2]=0
    // for any ψ), producing the ~180° heading offset observed on real
    // walks. Verified via scripts/test_z_up_conventions.py.
    return std::atan2(R_aligned.at<double>(1, 0),
                      R_aligned.at<double>(0, 0));
}

// ── Fix B (2026-05-16 audit) — canonical single-source heading ───────────────
//
// Cause: architecture audit Finding 2.1 documented 4 distinct yaw-extraction
// formulas across the codebase. scalar_heading_ (Tracker mirror of Madgwick) and
// global_R_ (Tracker mirror of EKF R_GtoI) diverge when EKF and Madgwick drift
// apart (v26: 55° by 53s). Every consumer that crossed the boundary amplified
// rather than corrected the drift.
//
// Change: expose getWorldHeadingRad() as the single EKF-internal heading query.
// Derives roll/pitch from R_GtoI_ col-2 (gravity direction in body frame) using
// the same tilt-angle decomposition that getYaw(roll,pitch) performs, removing
// the dependency on the caller to supply Madgwick angles.
//
// Falsifier: with a horizontal phone (R_GtoI = I), col-2 = (0,0,1) → roll=0,
// pitch=0 → getWorldHeadingRad() == getYaw(0,0). With a vertical phone facing
// North, col-2 ≈ (0,-1,0) → pitch ≈ -π/2 → R_align removes the tilt → atan2
// extracts the nav heading correctly. Unit-test expectations documented in
// EKFState.h getWorldHeadingRad() declaration.
double EKFState::getWorldHeadingRad() const {
    if (!full_initialized_ || R_GtoI_.empty()) return 0.0;
    // Extract roll and pitch from R_GtoI_ column 2 (world-Z in body frame).
    // R_GtoI_ col 2 = R_GtoI_ * (0,0,1)^T = body-frame gravity direction
    // (i.e. which body axis is "up" in world frame).
    const double gx = R_GtoI_.at<double>(0, 2);  // body-x component of world-Z
    const double gy = R_GtoI_.at<double>(1, 2);  // body-y component of world-Z
    const double gz = R_GtoI_.at<double>(2, 2);  // body-z component of world-Z
    // Tait-Bryan ZYX: pitch = asin(-gx), roll = atan2(gy, gz).
    // Matches IMUPreintegrator.cpp:704-707 and the standard gravity→RPY formula.
    const double pitch = std::asin(std::max(-1.0, std::min(1.0, -gx)));
    const double roll  = std::atan2(gy, gz);
    return getYaw(roll, pitch);
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

// Step 8a (ADR-014): retrieve the nanosecond timestamp of a clone by state_id.
// Used by applyMSCKFFeature to compute the Δt denominator for feature velocity.
// Returns 0 if the clone is not found.
int64_t EKFState::getCloneTimestamp(int state_id) const {
    for (const auto& pose : window_) {
        if (pose.state_id == state_id) {
            return pose.timestamp_ns;
        }
    }
    return 0;
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
    // Post-warmup variance: ±2 ms std (0.002 s). The cross-correlation warmup
    // narrows from the prior (5 ms) to this refined estimate.
    // Value matches OpenVINS td init after cross-correlation (Li & Mourikis 2014).
    constexpr double td_post_warmup_var = 0.002 * 0.002;  // (2ms)²
    P_td_ = td_post_warmup_var;

    // Step 8a (ADR-014): if the full state is already initialized, propagate the
    // warmup variance into P_(15,15) so the EKF starts from the refined prior.
    // This path fires when the TD warmup completes after initializeFull — rare in
    // practice (warmup typically completes at frame ~10, EKF full-init at frame ~5)
    // but correct to handle.
    if (full_initialized_ && !P_.empty() && P_.rows > 15) {
        P_.at<double>(15, 15) = td_post_warmup_var;
    }

    LOGI("EKF: Time offset warm-started to %.3fms (std=2.0ms, in-state=%s)",
         td_seconds * 1000.0, full_initialized_ ? "yes" : "pending");
}

// DEAD CODE: updateTemporal — never called
// void EKFState::updateTemporal(double observed_scale, double confidence, double H_td) { ... }

// ── Plan Step 8b: Online IMU-Camera Extrinsics Calibration ───────────────────

void EKFState::setExtrinsicsRotation(const cv::Matx33d& R_bc) {
    R_bc_ = R_bc;
    // If the full state is already initialized, leave the covariance unchanged
    // (the caller is providing a better prior, not a correction — the EKF will
    // begin refining from this new nominal on the next visual update).
    LOGI("EKF: Extrinsics R_bc set from external source (angle from identity=%.2f deg)",
         getExtrinsicsAngleDeg());
}

double EKFState::getExtrinsicsAngleDeg() const {
    // Angle of drift from the *initial* nominal extrinsic R_bc_initial,
    // not from identity. R_bc_initial = diag(1, -1, -1) (rear camera,
    // vertical phone — see reset() for derivation) has angle 180° from
    // identity, so a "from-identity" metric reports a constant 180°
    // regardless of any actual drift, masking real Step 8b extrinsic
    // calibration changes (bug spotted in sim 1778147132092).
    //
    // The deviation is ||Rodrigues(R_bc_ * R_bc_initial^T)||₂. Since
    // R_bc_initial = diag(1,-1,-1) is symmetric and self-inverse,
    // R_bc_initial^T = R_bc_initial, so the product is R_bc_ * R_bc_initial.
    cv::Matx33d R_bc_initial(1.0,  0.0,  0.0,
                              0.0, -1.0,  0.0,
                              0.0,  0.0, -1.0);
    cv::Matx33d R_drift = R_bc_ * R_bc_initial;
    cv::Mat R_mat(3, 3, CV_64F);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R_mat.at<double>(r, c) = R_drift(r, c);
    cv::Mat rvec;
    cv::Rodrigues(R_mat, rvec);
    const double angle_rad = cv::norm(rvec);
    return angle_rad * (180.0 / M_PI);
}

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

int EKFState::getSlamFeatureId(int slot) const {
    if (slot < 0 || slot >= static_cast<int>(slam_features_.size())) return -1;
    return slam_features_[slot].feature_id;
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

// Phase 1 Step 4a (post_v19_sprint_plan.md). Transfer the feature's inverse-
// depth parameterisation (α, β, ρ) from its current anchor clone to
// `new_anchor_id`, preserving the world-frame position. Must be called BEFORE
// the old anchor is popped from window_; both clones must currently be present.
// Covariance is intentionally not transformed (plan §4a) — the EKF absorbs the
// small numerical mismatch on the next measurement update.
bool EKFState::reanchorSlamFeature(int slot, int new_anchor_id) {
    if (slot < 0 || slot >= static_cast<int>(slam_features_.size())) return false;
    auto& f = slam_features_[slot];
    if (f.anchor_clone_id == new_anchor_id) return true;  // already anchored there

    // OpenVINS-style FEJ-consistent re-anchoring (v23.1 fix, 2026-05-11):
    // The earlier draft of this function computed p_world from the OLD anchor's
    // CURRENT pose (via getClonePose) and then projected into the NEW anchor's
    // CURRENT pose. That captured the OLD anchor's pose drift since promotion
    // INTO the new state, making the new (α, β, ρ) inconsistent with what the
    // residual + Jacobian expect. Visible in v23.1 logcat: 112 "SLAM demote
    // (bad RMS streak)" events in 1 min as features failed the 3px reprojection
    // gate within 3 frames of being re-anchored.
    //
    // Correct algorithm: use the locked FIRST-ESTIMATE world point
    // (f.p_global_FEJ) and the NEW anchor's FEJ pose (getCloneFEJ). That way
    // the new (α, β, ρ) is consistent with the new anchor_R_FEJ + anchor_p_FEJ
    // we store, and slamReprojectionJacobian's FEJ-based linearisation stays
    // self-consistent across re-anchors. World point is preserved exactly
    // (same physical 3D point) because we re-use f.p_global_FEJ verbatim.

    if (f.p_global_FEJ.empty() ||
        f.p_global_FEJ.rows != 3 || f.p_global_FEJ.cols != 1) return false;

    // NEW anchor's FEJ pose. Fall back to current pose if FEJ wasn't recorded
    // (matches the convention in addSlamFeature for fresh clones).
    cv::Mat R_new_FEJ, p_new_FEJ;
    if (!getCloneFEJ(new_anchor_id, R_new_FEJ, p_new_FEJ) ||
        R_new_FEJ.empty() || p_new_FEJ.empty()) {
        if (!getClonePose(new_anchor_id, R_new_FEJ, p_new_FEJ)) return false;
    }
    if (R_new_FEJ.rows != 3 || R_new_FEJ.cols != 3 ||
        p_new_FEJ.rows != 3 || p_new_FEJ.cols != 1) return false;

    // Project the locked FEJ world point into the NEW anchor's FEJ camera
    // frame. Same formula as addSlamFeature line 1612.
    cv::Mat p_new_cam = R_new_FEJ * (f.p_global_FEJ - p_new_FEJ);
    const double Xc = p_new_cam.at<double>(0, 0);
    const double Yc = p_new_cam.at<double>(1, 0);
    const double Zc = p_new_cam.at<double>(2, 0);
    if (!std::isfinite(Xc) || !std::isfinite(Yc) || !std::isfinite(Zc)) return false;
    if (Zc < 0.01) return false;  // behind / too close — would produce degenerate ρ

    // Update state mean to new parameterisation.
    f.state.at<double>(0, 0) = Xc / Zc;
    f.state.at<double>(1, 0) = Yc / Zc;
    f.state.at<double>(2, 0) = 1.0 / Zc;
    // pad rows 3, 4 stay zero — never updated by any measurement.

    // Update anchor metadata to NEW anchor's FEJ pose. p_global_FEJ is
    // preserved — it's the same physical 3D point in the world.
    f.anchor_clone_id = new_anchor_id;
    f.anchor_R_FEJ    = R_new_FEJ.clone();
    f.anchor_p_FEJ    = p_new_FEJ.clone();
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
    // sigma_uv = 1 pixel of bearing uncertainty in normalised image coords = 1/focal.
    //
    // Cause:  OLD value 1.0/500.0 hardcoded an assumed focal of 500 px.  The
    //         Samsung S21 Ultra rear camera at the working zoom delivers
    //         slam_fx_ ≈ 700-900 px (set by Tracker::setIntrinsics -> setSlamIntrinsics).
    //         500 overstates bearing uncertainty 40-80%, making P_ff at promotion
    //         too large and slowing SLAM dot convergence (ekf_audit.md Finding 6).
    // Change: Use 1.0/slam_fx_ (actual focal from live intrinsics).
    //         slam_fx_ defaults to 500.0 (EKFState.h:643) until setSlamIntrinsics
    //         is called, so this is safe before calibration initialises.
    // Falsifier: At runtime print slam_fx_ and sigma_uv.  For S21 Ultra at 1x zoom,
    //            slam_fx_ ≈ 750 → sigma_uv ≈ 0.00133 (was 0.002).
    // Date:   2026-05-16 (ekf_audit.md Finding 6)
    /* OLD (2026-05-16 superseded):
    const double sigma_uv  = 1.0 / 500.0;  // ≈1px / 500-focal — calibrated trend
    */
    const double sigma_uv  = 1.0 / std::max(slam_fx_, 1.0);  // 1px / actual focal
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
    // Explicit -> cv::Mat return type AND materialize via a local: without
    // this, auto deduction made the return type cv::MatCommaInitializer_<double>,
    // which holds a pointer to the temporary Mat_<double>(3,3) constructed
    // inline. That temporary was destroyed at end-of-return — leaving the
    // returned MatCommaInitializer with a dangling pointer. The caller's
    // implicit conversion to Mat then read freed memory, manifesting downstream
    // as "Matrix operand is an empty matrix" at step=13 (skew(p_C_F) * -1.0).
    // v22 logcat: 104 occurrences; v23 with Step 4 keeping features alive: 131
    // occurrences in seconds. Same pattern as the line 180 skew lambda which
    // already had the explicit return type fix.
    auto skew = [](const cv::Mat& v) -> cv::Mat {
        cv::Mat result = (cv::Mat_<double>(3, 3) <<
                0.0,                 -v.at<double>(2, 0),  v.at<double>(1, 0),
                v.at<double>(2, 0),   0.0,                -v.at<double>(0, 0),
               -v.at<double>(1, 0),   v.at<double>(0, 0),  0.0);
        return result;
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

        // Skip self-observation: anchor clone observing itself has zero
        // baseline by definition; the residual is degenerate.
        if (f.anchor_clone_id == clone_id) continue;

        cv::Mat H_feat, H_clone;
        cv::Point2d pred;
        if (!slamReprojectionJacobian(f, R_FEJ, p_FEJ, R_now, p_now,
                                      H_feat, H_clone, pred)) {
            continue;
        }

        // Sparse 2 x dim Jacobian:
        //   - clone slot at [clone_cov, clone_cov+6)
        //   - SLAM slot  at [slam_idx, slam_idx+5)
        cv::Mat H = cv::Mat::zeros(2, dim, CV_64F);
        H_clone.copyTo(H(cv::Range::all(),
                         cv::Range(clone_cov, clone_cov + CLONE_DIM)));
        H_feat.copyTo(H(cv::Range::all(),
                        cv::Range(slam_idx, slam_idx + SLAM_FEATURE_DIM)));

        // Residual = obs - predicted (pixel coords).
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

    // v23.13 (2026-05-12): per-update chi² gate (OpenVINS doctrine,
    // rpng/open_vins UpdaterSLAM.cpp:520-556).
    //
    //   chi² = r^T · (H·P·H^T + R)^-1 · r
    //   reject if chi² > chi2_multipler · χ²_{0.95}(dof=2K)
    //
    // OpenVINS defaults: chi2_multipler = 5, σ_pix = 1.0.
    // Replaces the previous parallax gate which blocked valid corrections
    // on slow motion / still phone, letting clone-pose drift accumulate.
    {
        cv::Mat S_chi = H_stack * P_ * H_stack.t() + R_noise;
        cv::Mat S_inv_chi;
        bool ok = cv::invert(S_chi, S_inv_chi, cv::DECOMP_CHOLESKY);
        if (!ok) ok = cv::invert(S_chi, S_inv_chi, cv::DECOMP_SVD);
        if (ok) {
            cv::Mat chi2_mat = r_stack.t() * S_inv_chi * r_stack;
            const double chi2 = chi2_mat.at<double>(0, 0);
            const double chi2_095_table[] = {
                0.0, 3.841, 5.991, 7.815, 9.488,
                11.070, 12.592, 14.067, 15.507
            };
            const int dof = 2 * K;
            const double base = (dof >= 1 && dof < 9)
                                 ? chi2_095_table[dof]
                                 : (5.991 + 2.0 * (dof - 2));
            const double thresh = 5.0 * base;   // chi2_multipler = 5
            if (!std::isfinite(chi2) || chi2 > thresh) {
                if (rms_n > 0) {
                    f.last_obs_rms = std::sqrt(rms_acc /
                                                static_cast<double>(rms_n));
                }
                return false;
            }
        }
    }

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

    // Explicit -> cv::Mat return type AND materialize via a local: without
    // this, auto deduction made the return type cv::MatCommaInitializer_<double>,
    // which holds a pointer to the temporary Mat_<double>(3,3) constructed
    // inline. That temporary was destroyed at end-of-return — leaving the
    // returned MatCommaInitializer with a dangling pointer. The caller's
    // implicit conversion to Mat then read freed memory, manifesting downstream
    // as "Matrix operand is an empty matrix" at step=13 (skew(p_C_F) * -1.0).
    // v22 logcat: 104 occurrences; v23 with Step 4 keeping features alive: 131
    // occurrences in seconds. Same pattern as the line 180 skew lambda which
    // already had the explicit return type fix.
    auto skew = [](const cv::Mat& v) -> cv::Mat {
        cv::Mat result = (cv::Mat_<double>(3, 3) <<
                0.0,                 -v.at<double>(2, 0),  v.at<double>(1, 0),
                v.at<double>(2, 0),   0.0,                -v.at<double>(0, 0),
               -v.at<double>(1, 0),   v.at<double>(0, 0),  0.0);
        return result;
    };

    // Step 8a (ADR-014): per-observation index into the VALID observations list.
    // We need to know which valid observations are adjacent (in time) to compute
    // feature velocity for the H_td column. Track accepted observations as we go.
    // After collecting all valid rows, we fill column 15 using finite differences
    // on the normalized image coordinates of consecutive accepted observations.
    // Indices into the observations/clone_ids input arrays that were accepted:
    std::vector<size_t> accepted_src_idx;

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

        // Step 8b: H_bc DISABLED (NavSight 2026-05-09 Step 7 fix).
        // R_bc is baked into clone storage at addClone time (Tracker.cpp:1622),
        // so live R_bc no longer factors into projection — treating it as a
        // free state would double-count it. R_bc stays at the physical
        // fixed-mount value read from Android SENSOR_ORIENTATION; online
        // estimation is unnecessary for a fixed-mount phone camera.
        // (H_x is zero-initialized; the EXTR columns remain 0.)
        //
        // PHASE_A_8B_REENABLE_2026_05_19 REVERTED — re-enabling H_bc here
        // produced dphi_bc=0 because H_bc was perfectly correlated with
        // H_clone_theta. R_bc unobservable under Option C clone storage.

        cv::Mat r = (cv::Mat_<double>(2, 1) <<
                     observations[k].x - pred_u,
                     observations[k].y - pred_v);

        accepted_src_idx.push_back(k);
        H_x_rows.push_back(H_x);
        H_f_rows.push_back(H_f);
        r_rows  .push_back(r);
    }

    // Step 8a (ADR-014): fill H_x column 15 (δt_d) for each accepted observation.
    // The time-offset Jacobian for observation k is H_td_k = -v_feature_k (2×1),
    // where v_feature_k is the pixel velocity of the feature (px/s) divided by
    // focal length to obtain normalised-image-coordinate velocity (1/s).
    // Derivation: t_corrected = t_obs + δt_d → residual changes by -v * δt_d.
    // Source: Li & Mourikis 2014 §III-B, OpenVINS online td estimator.
    //
    // We use finite differences on the raw pixel observations; the clone
    // timestamps give Δt. The column index for δt_d is 15, fixed by the 19-DOF
    // IMU block layout: [δθ(0-2), δb_g(3-5), δv(6-8), δb_a(9-11), δp(12-14),
    // δt_d(15), δφ_bc(16-18)]. Step 8a row, not IMU_STATE_DIM-1.
    const int td_col = 15;  // δt_d column in the full state vector
    const int n_accepted = static_cast<int>(accepted_src_idx.size());
    for (int ai = 0; ai < n_accepted; ai++) {
        // Pick neighbour indices for finite difference (central where possible)
        int ai_prev = (ai > 0) ? (ai - 1) : ai;
        int ai_next = (ai < n_accepted - 1) ? (ai + 1) : ai;

        if (ai_prev == ai_next) {
            // Only one valid observation — cannot estimate velocity; skip td term
            continue;
        }

        const size_t src_prev = accepted_src_idx[ai_prev];
        const size_t src_next = accepted_src_idx[ai_next];

        const int64_t ts_prev = getCloneTimestamp(clone_ids[src_prev]);
        const int64_t ts_next = getCloneTimestamp(clone_ids[src_next]);
        const double dt_fd = (ts_next - ts_prev) * 1e-9;  // nanoseconds → seconds
        if (std::abs(dt_fd) < 1e-6) continue;  // degenerate: clones at same timestamp

        // Pixel velocity via finite difference (pixels/s)
        const double dpx_du = (observations[src_next].x - observations[src_prev].x) / dt_fd;
        const double dpx_dv = (observations[src_next].y - observations[src_prev].y) / dt_fd;

        // H_td in PIXEL velocity (px/s), consistent with the pixel-space residual.
        //
        // Cause:  OLD code divided by slam_fx_/slam_fy_ to convert to normalised
        //         image velocity (1/s).  But the residual r (line ~2452) is in PIXEL
        //         coords (observations are pixels, pred_u/v from slam_fx_*pC/zC+cx).
        //         The unit mismatch made the Kalman gain on δt_d ~700× wrong
        //         (focal ≈ 700 px), causing dtd to oscillate rather than converge.
        //         All other H columns (H_clone) are already in pixels/radian and
        //         pixels/meter — H_td must match (ekf_audit.md Finding 5).
        // Change: Remove the /slam_fx_ and /slam_fy_ divisions so H_td stays px/s.
        // Falsifier: with slow walking (feature velocity ~150 px/s), H_td should be
        //            ~150.  With the OLD code it was ~150/700 ≈ 0.21.  Log column 15
        //            of H_x_rows[ai] and verify its magnitude matches pixel velocity.
        // Date:   2026-05-16 (ekf_audit.md Finding 5)
        /* OLD (2026-05-16 superseded — wrong unit: normalised, not pixels):
        const double h_td_u = -dpx_du / slam_fx_;
        const double h_td_v = -dpx_dv / slam_fy_;
        */
        const double h_td_u = -dpx_du;
        const double h_td_v = -dpx_dv;

        // ai maps to rows [2*ai, 2*ai+1] in H_x_rows — already stored.
        // We need to write into column td_col of the stored H_x matrix.
        if (td_col < H_x_rows[ai].cols) {
            H_x_rows[ai].at<double>(0, td_col) = h_td_u;
            H_x_rows[ai].at<double>(1, td_col) = h_td_v;
        }
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

// 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
//
// Consolidated read of every EKF field consumed by the JNI overlay path. All
// reads happen under one snapshot_mutex_ acquisition so the camera pose and
// SLAM feature world positions are guaranteed to be derived from the SAME
// state-version (no mid-snapshot processFrame mutation). The mutex is the
// same one addClone / marginalizeOldestClone / pruneWindow take while
// splicing window_ + slam_features_, so we never observe a torn deque or
// vector here. R_GtoI_/p_G_/R_bc_ are read in the same critical section;
// the camera thread does not take snapshot_mutex_ for those individually,
// but bundling the reads in one function call closes the dominant gap
// (multi-millisecond Kotlin-side gap between separate JNI hops).
EKFState::OverlaySnapshot EKFState::snapshotForOverlay() const {
    OverlaySnapshot out;
    std::lock_guard<std::mutex> lock(snapshot_mutex_);

    out.full_initialized = full_initialized_;
    if (!full_initialized_) return out;

    // IMU state (R_GtoI, p_G) — copy into POD types so the caller has no
    // dangling cv::Mat headers into our internal storage.
    if (R_GtoI_.rows == 3 && R_GtoI_.cols == 3 && R_GtoI_.type() == CV_64F) {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                out.R_GtoI(r, c) = R_GtoI_.at<double>(r, c);
            }
        }
    }
    if (p_G_.rows == 3 && p_G_.cols == 1 && p_G_.type() == CV_64F) {
        out.p_G = cv::Vec3d(p_G_.at<double>(0, 0),
                            p_G_.at<double>(1, 0),
                            p_G_.at<double>(2, 0));
    }

    // Body→camera extrinsic (already a Matx33d — direct copy).
    out.R_bc = R_bc_;

    // Cached SLAM intrinsics.
    out.slam_fx = slam_fx_;
    out.slam_fy = slam_fy_;
    out.slam_cx = slam_cx_;
    out.slam_cy = slam_cy_;

    // Walk SLAM features inline (no helper calls — keep everything in one
    // critical section). Math is the same as getSlamFeatureGlobalPosition:
    //   p_anchor_pt = (α/ρ, β/ρ, 1/ρ)
    //   p_world     = R_anchor^T · p_anchor_pt + p_anchor_world
    out.slam_features.reserve(slam_features_.size());
    for (const SlamFeature& f : slam_features_) {
        if (f.state.empty() || f.state.rows < 3) continue;
        const double alpha = f.state.at<double>(0, 0);
        const double beta  = f.state.at<double>(1, 0);
        const double rho   = f.state.at<double>(2, 0);
        if (std::abs(rho) < 1e-9) continue;

        // Find the anchor clone in window_ (same scan getClonePose does, but
        // inline so we stay under our one mutex acquisition and don't risk
        // a future refactor adding a second lock site to getClonePose).
        cv::Matx33d R_anchor_x(1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0);
        cv::Vec3d   p_anchor_w(0.0, 0.0, 0.0);
        bool found = false;
        for (const CameraPose& cp : window_) {
            if (cp.state_id != f.anchor_clone_id) continue;
            if (cp.R_GtoC.rows != 3 || cp.R_GtoC.cols != 3 ||
                cp.p_G.rows    != 3 || cp.p_G.cols    != 1) continue;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R_anchor_x(r, c) = cp.R_GtoC.at<double>(r, c);
                }
            }
            p_anchor_w = cv::Vec3d(cp.p_G.at<double>(0, 0),
                                   cp.p_G.at<double>(1, 0),
                                   cp.p_G.at<double>(2, 0));
            found = true;
            break;
        }
        if (!found) continue;  // anchor marginalised; skip silently (existing semantics)

        const cv::Vec3d p_anchor_pt(alpha / rho, beta / rho, 1.0 / rho);
        // p_world = R_anchor^T · p_anchor_pt + p_anchor_w
        const cv::Vec3d p_world = R_anchor_x.t() * p_anchor_pt + p_anchor_w;

        OverlaySnapshot::SlamFeatureEntry e;
        e.feature_id = f.feature_id;
        e.p_world    = p_world;
        out.slam_features.push_back(e);
    }

    // 2026-05-19 — Orange-dot anchor-relative render support. Copy the live
    // (state_id, R_GtoC, p_G) of every clone in window_ so getLandmarkSnapshot
    // can look up each landmark's host clone CURRENT pose under the same
    // snapshot_mutex_ tick that produced R_GtoI / p_G / slam_features above.
    // Field types match OverlaySnapshot::CloneEntry — cheap POD copy out of
    // the cv::Mat-backed CameraPose storage. Skip clones with degenerate
    // matrices (defensive; shouldn't occur in healthy operation).
    out.clones.reserve(window_.size());
    for (const CameraPose& cp : window_) {
        if (cp.R_GtoC.rows != 3 || cp.R_GtoC.cols != 3 ||
            cp.R_GtoC.type() != CV_64F) continue;
        if (cp.p_G.rows != 3 || cp.p_G.cols != 1 ||
            cp.p_G.type() != CV_64F) continue;
        OverlaySnapshot::CloneEntry ce;
        ce.state_id = cp.state_id;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                ce.R_GtoC(r, c) = cp.R_GtoC.at<double>(r, c);
            }
        }
        ce.p_G = cv::Vec3d(cp.p_G.at<double>(0, 0),
                            cp.p_G.at<double>(1, 0),
                            cp.p_G.at<double>(2, 0));
        out.clones.push_back(ce);
    }
    return out;
}
