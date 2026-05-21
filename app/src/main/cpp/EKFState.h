#pragma once
#include <vector>
#include <deque>
#include <mutex>
#include <utility>
#include <cstdint>
#include <opencv2/core.hpp>

// Camera pose clone for the MSCKF sliding window.
// Based on OpenVINS First-Estimate Jacobians (FEJ) paradigm.
struct CameraPose {
    cv::Mat R_GtoC;     // 3x3 CV_64F: Rotation from Global to Camera frame
    cv::Mat p_G;        // 3x1 CV_64F: Position in Global frame

    // First-Estimate Jacobians (FEJ) storage.
    // Locked linearization points to maintain observability of global yaw and position.
    cv::Mat R_FEJ;      // Fixed rotation when this pose was first initialized
    cv::Mat p_FEJ;      // Fixed position when this pose was first initialized

    int64_t timestamp_ns;
    int state_id;

    CameraPose() : timestamp_ns(0), state_id(-1) {
        R_GtoC = cv::Mat::eye(3, 3, CV_64F);
        p_G = cv::Mat::zeros(3, 1, CV_64F);
        R_FEJ = cv::Mat::eye(3, 3, CV_64F);
        p_FEJ = cv::Mat::zeros(3, 1, CV_64F);
    }
};

// Extended Kalman Filter for VIO.
// Full error-state EKF with MSCKF sliding window support.
//
// IMU error-state (19 DOF after Steps 8a + 8b):
//   rows  0– 2: δθ      — rotation error (rad)
//   rows  3– 5: δb_g    — gyro bias error (rad/s)
//   rows  6– 8: δv      — velocity error (m/s)
//   rows  9–11: δb_a    — accel bias error (m/s²)
//   rows 12–14: δp      — position error (m)
//   row  15:    δt_d    — camera-IMU time offset error (s) [Step 8a, ADR-014]
//   rows 16–18: δφ_bc   — camera-body rotation error (rad) [Step 8b, ADR-015]
// Each clone adds: [δθ_c(3), δp_c(3)] = 6 DOF
// Total state dimension: 19 + 6*N_clones + 5*N_slam_features
class EKFState {
public:
    EKFState();

    // --- Legacy Scale Support (kept for backward compatibility) ---
    void initialize(double initial_scale);
    void updateScale(double observed_scale, double confidence);
    void updateZUPT();

    // Zero-Rotation-Rate Update — dual of ZUPT. When stationary, the gyro
    // reading IS the bias by definition. Observes mean(gyro_window) - b_g_ = 0
    // as a standard Kalman measurement with H = -I on rows 3:6 (b_g_ block).
    // Returns true on success, false if EKF not full-initialized or invalid
    // window size. See EKFState.cpp for the full Cause/Change/Falsifier block.
    bool updateZRUP(double mean_gx, double mean_gy, double mean_gz,
                    double sigma_gyro_per_sample, int N_samples);

    // 2026-05-09 v18 — single-source-of-truth sync.
    // Set the EKF's internal position state directly from an external source
    // (e.g. Tracker's `global_t_`, which is the visual-VO + heading + scaled-step
    // trajectory the user actually sees on the map). This eliminates the
    // "EKF p_G drifts under the hood while UI shows global_t_" disconnect that
    // caused the v9–v16 chi² rejections (loop closure target compared against
    // a drifted p_G even though the user-visible trajectory was reasonable).
    //
    // Does NOT touch covariance — the EKF's uncertainty estimate stays valid
    // because position-error covariance reflects the current STATE estimate's
    // uncertainty, and we're updating the state to a value the visual layer
    // is already confident in. Velocity, biases, rotation are all unchanged.
    //
    // Call once per frame (after global_t_ is finalised) so clones, MSCKF
    // residuals, and loop-closure chi² evaluations all reference the same
    // trajectory the user sees.
    void setPosition(const cv::Mat& p_G);

    // Stationary specific-force update (2026-05-09 drift fix).
    //
    // Called from Tracker AFTER updateZUPT() confirms the body is
    // stationary. Closes the accel-bias observability gap that the
    // existing measurement updates leave open:
    //
    //   * MSCKF: observes b_a only via visual residuals. During
    //     visual-degenerate phases (low texture, motion blur, fast
    //     turns) b_a drifts unchecked.
    //   * updateZUPT: zeros v_G and shrinks the velocity block. Does
    //     NOT observe b_a — so the next propagateIMU step still
    //     integrates accel-bias residual into v_G.
    //   * updateGravityAlignment: observes accel direction (yaw
    //     unobservable). Magnitude information about b_a is discarded
    //     by the unit-vector normalisation.
    //
    // Specific-force model under body-stationary assumption:
    //   a_imu  =  -R_GtoI · g_world  +  b_a  +  noise
    //   In Z-up ENU, g_world = (0,0,-9.81) → a_imu = 9.81·R_GtoI[:,2] + b_a
    //
    // 3-DOF residual that constrains BOTH δθ (roll/pitch) and δb_a
    // simultaneously. Yaw stays unobservable (skew of R_GtoI[:,2] has
    // null direction along R_GtoI[:,2] itself ↔ world-Z by the
    // left-perturbation Lie identity).
    //
    // `accel_body` should be the measurement-window MEAN (e.g.
    // LP-filtered gravity vector from IMUPreintegrator), not an
    // instantaneous sample — the ZUPT detector already validated
    // window stationarity, but the per-sample noise is uncorrelated.
    // `sigma_a` is the per-axis accel measurement noise σ (m/s²);
    // pass the same kAccelNoiseSigma used by gravity-alignment.
    //
    // Returns false on degenerate input or if the EKF is not
    // full-initialized.
    [[nodiscard]] bool updateStationaryAccel(const cv::Mat& accel_body, double sigma_a);
    // DEAD CODE: checkConsistency, getScaleStd — never called
    // double checkConsistency(double camera_disp, double step_disp) const;
    double getScale() const { return scale_; }
    // double getScaleStd() const;
    void reset();

    // --- Full Error-State MSCKF ---

    // Initialize full IMU state from initial rotation and gravity
    void initializeFull(const cv::Mat& R_GtoI, const cv::Point3f& gyro_bias,
                        const cv::Point3f& accel_bias);

    // IMU state propagation (covariance propagation using preintegration)
    void propagateIMU(const cv::Mat& deltaR, const cv::Mat& deltaV,
                      const cv::Mat& deltaP, double dt,
                      const cv::Mat& imu_cov,
                      const cv::Mat& J_R_bg, const cv::Mat& J_V_bg,
                      const cv::Mat& J_V_ba, const cv::Mat& J_P_bg,
                      const cv::Mat& J_P_ba);

    // Clone management
    void addClone(const cv::Mat& R_GtoC, const cv::Mat& p_G, int64_t timestamp_ns);
    // Phase 1 Step 4b: default kept in sync with MAX_CLONES (11 → 15). Can't
    // reference the constant directly because MAX_CLONES is declared lower in
    // this class body; the value here mirrors the declaration at line ~506.
    void pruneWindow(size_t max_poses = 15);
    void marginalizeOldestClone();

    // MSCKF update: apply stacked null-space-projected measurement
    // v23.14 (2026-05-12): added apply_huber=true default. Set false for
    // hard-physical-constraint updates (gravity-alignment, etc.) where the
    // Huber kernel structurally fights against real attitude residuals that
    // ARE the correction signal, not outliers. Defaulting to true preserves
    // all existing MSCKF/SLAM-update callsites unchanged.
    void applyMSCKFUpdate(const cv::Mat& H, const cv::Mat& res,
                          const cv::Mat& R_noise, bool apply_huber = true);

    // ── Plan Step 3b (ADR-009): SLAM features in EKF state ────────────────────
    //
    // SLAM features are long-lived landmarks carried in the EKF state vector.
    // Inverse-depth parameterisation (α, β, ρ) anchored to the camera clone in
    // which the feature was first promoted (Civera 2008 / OpenVINS):
    //
    //   p_anchor = (1 / ρ) * (α, β, 1)
    //   p_C      = R_C_anchor * p_anchor + t_C_anchor
    //   (u, v)   = (p_C.x / p_C.z, p_C.y / p_C.z)              [normalised]
    //
    // Per-feature state grows the covariance by SLAM_FEATURE_DIM = 5 rows/cols.
    // The active math operates on the leading 3 entries (α, β, ρ); the trailing
    // 2 are identity-frozen padding kept for representation flexibility — they
    // never appear in any measurement Jacobian, so they are propagated as a
    // pinned identity block under all updates and contribute no information /
    // no degradation to the filter. The 5-row layout matches the contract in
    // `tests/cpp/test_slam_msckf.cpp`.
    //
    // The SLAM block always lives at the END of P_, after all clone blocks.
    // `addClone` and `marginalizeOldestClone` splice clone rows BEFORE the
    // SLAM block (Plan Step 3b acceptance — no SLAM teleport on clone churn).
    //
    // FEJ pattern: at promotion we lock the anchor's (R, p), the feature's
    // p_global, and at every subsequent update Jacobians use the FEJ values
    // (the linearisation point) while residuals use the CURRENT state.

    static constexpr int  SLAM_FEATURE_DIM = 5;     // 3 active (α,β,ρ) + 2 pad
    static constexpr int  SLAM_FEATURE_ACTIVE_DIM = 3;
    static constexpr int  MAX_SLAM_FEATURES = 12;
    static constexpr double SLAM_PAD_VARIANCE = 1e-6;  // pinned identity scale

    // Push the camera intrinsics used for SLAM-feature reprojection. Tracker
    // calls this from `setIntrinsics`; the EKF holds a cached copy because
    // the SLAM update API in `tests/cpp/test_slam_msckf.cpp` does not pass
    // K through. Defaults (fx=fy=500, cx=320, cy=240) match the synthetic
    // pinhole the unit tests use.
    void setSlamIntrinsics(double fx, double fy, double cx, double cy);
    double getSlamFx() const { return slam_fx_; }
    double getSlamFy() const { return slam_fy_; }
    double getSlamCx() const { return slam_cx_; }
    double getSlamCy() const { return slam_cy_; }

    // Promote a triangulated 3-D point into the EKF state as a SLAM feature
    // anchored at `anchor_clone`. Returns the slot index (≥ 0) on success, or
    // −1 if the state is full, the anchor is invalid, the depth is degenerate
    // (≤ 1 cm in front of the anchor), or the EKF is not fully initialised.
    //
    // The anchor is identified by `anchor_clone.state_id` and must currently
    // exist in the sliding window — the anchor's covariance entries are used
    // to derive the initial (α, β, ρ) covariance.
    int addSlamFeature(int feature_id,
                       const cv::Mat& p_global_init,
                       const CameraPose& anchor_clone);

    // Schur-complement marginalisation of the SLAM feature at `slot`. Returns
    // false if the slot is invalid or the resulting covariance fails the PSD
    // diagonal check (the cross-correlation block is then NOT applied — slot
    // is removed but only by row/column deletion as a safe fallback). The
    // common path is the full block-Schur:
    //     P_keep' = P_keep - P_keep_slam * inv(P_slam_slam) * P_slam_keep
    bool removeSlamFeature(int slot);

    // Plan Step 3b lifecycle bridge (2026-05-09 telemetry fix): inside
    // marginalizeOldestCloneNoLock the EKF drops SLAM features whose
    // anchor clone fell off the sliding window. That bypasses the
    // FeatureManager's setSlamSlot(-1) / dropLifecycle path, leaving the
    // lifecycle map populated and the slam_lifetime_count histogram at
    // zero (sim_data_1778329181805 showed 78 promotions, 0 lifetime
    // events — every demotion went through clone marginalisation, not
    // RMS / chi² rejection).
    //
    // After every addClone() Tracker calls this method to drain the
    // dropped-feature ids accumulated across the (possibly multiple)
    // internal marginalisations and forward them to FeatureManager.
    // Vector is cleared on every addClone() call.
    std::vector<int> takeLastMarginalizedSlamFeatureIds();

    // 2-DOF reprojection update for a single SLAM feature observed across
    // one or more clones. Each (observation, clone_id) pair stacks 2 rows of
    // residual + Jacobian. The Jacobian is sparse: non-zero on the observing
    // clone's δθ/δp block and on the SLAM feature's (α, β, ρ) block (the 2
    // padding columns of the SLAM block stay zero — they are not observable
    // from any reprojection). Hands off to applyMSCKFUpdate, so the Step 3a
    // damping + Huber kernel apply automatically.
    //
    // Returns false if `slot` is invalid, sizes mismatch, or every clone in
    // `clone_ids` is missing from the current sliding window.
    bool updateSlamFeature(int slot,
                           const std::vector<cv::Point2f>& observations,
                           const std::vector<int>& clone_ids,
                           double pixel_noise_sq);

    // 2026-05-19 Fix #7 — per-frame SLAM update against the LIVE IMU state.
    //
    // Cause: `updateSlamFeature` (above) requires `clone_ids` referencing
    // clones in the EKF sliding window. NavSight only adds clones at
    // KEYFRAME storage (~2 Hz). Between keyframes, the EKF IMU state moves
    // (IMU integration + MSCKF on LandmarkMap observations every frame)
    // but (α, β, ρ) of SLAM features stays frozen because no new clone
    // has been added to attach an observation to. The rendered SLAM
    // projection drifts from the actual image feature, then "snaps" each
    // keyframe when UpdaterSLAM fires. v57 SLAM_RMS confirmed in-EKF
    // residuals are 0.3-3 px AT keyframes but the user-visible projection
    // gap is 60-100 px BETWEEN them.
    //
    // Change: this method observes a SLAM feature against the LIVE IMU
    // state directly (R_GtoI_, p_G_, R_bc_), no clone reference needed.
    // The Jacobian has non-zero blocks at:
    //   * IMU rows  (0-2 δθ_w, 12-14 δp_G):  J_pC_θ = -R_bc·skew(p_I);
    //                                        J_pC_p = -R_bc·R_GtoI
    //     (identical to applyLandmarkObservations live projection,
    //      EKFState.cpp:1900)
    //   * Anchor clone slot (FEJ): same anchor block as
    //     slamReprojectionJacobian, evaluated with R_GtoC_LIVE as the
    //     observing camera.
    //   * SLAM (α, β, ρ) slot:
    //     ∂p_C/∂(α,β,ρ) = R_GtoC_LIVE · R_GtoC_a_FEJ^T · J_α(α,β,ρ)
    //
    // Residual: predicted pixel uses LIVE anchor pose for p_world, LIVE
    // R_GtoI / p_G for projection. Jacobian uses FEJ anchor + LIVE IMU
    // (no FEJ exists for the "current" state — the latest observation IS
    // by definition the first estimate at this time).
    //
    // Gates: chi² (2-DOF 95% = 5.991) and depth (Z > 0.01) match
    // updateSlamFeature. Caller is expected to call only when KLT has a
    // valid track on this SLAM feature this frame.
    //
    // Returns true on a successful update applied to P_/state; false on
    // any gate rejection or invalid input. Caller may use the return
    // value for a per-feature counter.
    //
    // Falsifier: post-Fix-#7 SlamFeatureOverlay red residual lines should
    // stay short continuously (no keyframe-rate snap pattern). New
    // `SLAM_RMS_LIVE` log shows per-frame residual <3 px median when the
    // EKF state is healthy.
    bool updateSlamFeatureLive(int slot,
                               const cv::Point2f& obs_pixel,
                               double sigma_px);

    // 2026-05-19 Fix #8 — production-grade batched live SLAM update.
    //
    // Cause for batch: first install of Fix #7 ANR'd at 30 Hz × 12 SLAM
    // features × O(n^3) applyMSCKFUpdate per feature. CPU pressure avg60
    // = 17%; camera thread blocked. Fix #7b throttle (1/3 frame) papered
    // over it but is a frequency knob, not an architectural fix.
    //
    // Batch: stack all 12 SLAM features' (H_row, r_row) into ONE big H
    // matrix (24 × dim) and ONE applyMSCKFUpdate. Math is identical to
    // sequential single-feature updates (Kalman is linear so a stacked
    // measurement is mathematically equivalent to N sequential ones,
    // modulo numerical round-off ordering). Cost drops from N × O(n^3)
    // to 1 × O(n^3) — a 12× speedup. Returns the number of rows that
    // passed per-row chi² (95% 2-DOF = 5.991) + early-out gates and
    // were applied.
    //
    // Sparse exploitation (Fix #8b — also in this PR): applyMSCKFUpdate
    // remains dense, but the H_stack we build only touches the union of
    // active state slots (IMU [0-2, 12-14] + each anchor clone + each
    // SLAM block). For 12 features sharing up to 11 unique anchors,
    // that's ~80 of 141 cols non-zero — modest 1.5-2× speedup from
    // OpenCV's BLAS dropping the trailing zero work.
    //
    // Falsifier: post-Fix-#8 walk should show no ANR even at 30 Hz
    // per-frame SLAM updates. Counters slam_live_batch_calls and
    // slam_live_updates_fired together quantify rate + per-call yield.
    int applySlamLiveBatch(
        const std::vector<std::pair<int, cv::Point2f>>& observations,
        double sigma_px);

    // 2026-05-19 Fix #8 helper — pure projection + Jacobian build for a
    // single (slot, obs_pixel). No state mutation. Used by both the
    // single-feature `updateSlamFeatureLive` and the batched
    // `applySlamLiveBatch`. See those for the cause/change writeup.
    //
    // Outputs:
    //   H_row_out  2 × state_dim, sparse (only IMU + anchor + SLAM
    //              slot columns non-zero). Caller is responsible for
    //              copying into a stacked matrix or applying singly.
    //   r_row_out  2 × 1 residual (obs - pred).
    //   m2_out     scalar chi² of this row alone for diagnostics.
    //
    // Returns false (and outputs are not guaranteed valid) when ANY of:
    //   - slot out of range / state malformed / rho near zero
    //   - anchor clone missing from window
    //   - predicted projection has degenerate depth (Z < 0.01)
    //   - per-row chi² > 5.991 (95% 2-DOF gate)
    //   - residual < 1.0 px (early-out — already converged)
    //
    // Math reference: EKFState.h `updateSlamFeatureLive` docstring.
    bool buildSlamLiveJacobianRow(int slot,
                                    const cv::Point2f& obs_pixel,
                                    double sigma_px,
                                    cv::Mat& H_row_out,
                                    cv::Mat& r_row_out,
                                    double& m2_out) const;

    // MSCKF feature update: a 2N-DOF stacked reprojection residual (one
    // 2-vector per observation) over a triangulated point that is NOT
    // carried in the EKF state. This is the "transient feature, no slot"
    // path used by short tracks (the OpenVINS-style null-space projection
    // is implemented in `UpdaterMSCKF.cpp`; this method is the EKFState-
    // owned counterpart for cases where the caller already has a
    // triangulated 3-D point and wants the in-place null-space update).
    //
    // Returns false on degenerate input (no clones found, depth ≤ 1 cm,
    // size mismatch) or true on a successful Joseph-form update.
    bool applyMSCKFFeature(const std::vector<cv::Point2f>& observations,
                           const std::vector<int>& clone_ids,
                           const cv::Mat& triangulated_p_global,
                           double pixel_noise_sq);

    // Returns the starting column index of `slot`'s SLAM block in P_, or -1.
    int getSlamFeatureCovIdx(int slot) const;

    // Number of currently-active SLAM features in state. ≤ MAX_SLAM_FEATURES.
    int getSlamFeatureCount() const;

    // Look up the slot for a given feature_id, or -1 if not promoted.
    int getSlamFeatureSlot(int feature_id) const;

    // Reads the current 3-D global position of a SLAM feature by re-projecting
    // its (α, β, ρ) state through its anchor's CURRENT pose. Returns false on
    // invalid slot.
    bool getSlamFeatureGlobalPosition(int slot, cv::Mat& p_global_out) const;

    // Camera-overlay Phase 3 read accessor: feature_id at the given slot,
    // or -1 if the slot is out of range. Used by the JNI layer's
    // getSlamSnapshot to expose stable per-feature identifiers so the
    // overlay can correlate the same physical point across frames if it
    // ever wants to (today the overlay only consumes (fid, x, y, z) per
    // frame and does not track identity itself).
    int getSlamFeatureId(int slot) const;

    // ── Step 4: VIO/PDR/Yaw measurement updates ────────────────────────────
    // These are thin Joseph-form wrappers over applyMSCKFUpdate that build
    // sparse H/res/R for the common VIO measurement types. They make
    // EKFState the canonical owner of pose state — Tracker stops mutating
    // its own global_R_/global_t_/scalar_heading_ once these are wired.

    // VIO relative position. The caller hands in t_world_metric =
    // scale_fuser_.scale() * R_align * t_recoverPose_unit (i.e. world-frame,
    // metric). H places +I on δp_current and -I on δp at clone_id, so the
    // measurement constrains the position delta between the two clones.
    // Returns true on success, false if clone not found / state not init.
    [[nodiscard]] bool updateRelativePose(const cv::Mat& t_world_metric,
                            int clone_id,
                            double var_t);

    // Keyframe yaw correction (1-DOF). yaw_meas is in world frame
    // (navigation conv: CW-positive, North=0), var in rad². roll/pitch
    // come from the Madgwick filter and define the gravity-alignment
    // sandwich used to derive the H jacobian.
    [[nodiscard]] bool updateGravityAlignedYaw(double yaw_meas, double var,
                                 double roll, double pitch);

    // Stage 1 (2026-05-09 root-cause fix) — gravity-alignment measurement.
    //
    // Observation: when |a_body| ≈ g (i.e. the IMU is not under significant
    // linear acceleration), the accelerometer's specific force is the body-
    // frame projection of world Z-up:
    //
    //   accel_body / |accel_body|  ≈  R_GtoI · (0, 0, +1)
    //
    // This is a 3D unit-vector observation. Two of its three DOFs are
    // independent — yaw (rotation about world-Z) cannot change body-Z's
    // direction, so it stays unobservable from gravity (consistent with
    // physics; visual yaw and Madgwick mag-init handle yaw separately).
    //
    // The EKF previously had no gravity-alignment loop, so its R_GtoI
    // drifted by 6-10° on a 110 s walk from gyro bias residual integration,
    // producing the famous -800 m phantom Z drift in p_G via mis-cancelled
    // gravity in propagateIMU (see scripts/analyze_chi2_rejections.py
    // against tests/sims/regression/visual/loop_house_x2.json).
    // This method gives the EKF the same gravity correction Madgwick has,
    // so R_GtoI stays bounded and p_G no longer diverges.
    //
    // Args:
    //   accel_body : 3×1 raw accelerometer reading (m/s²) in body frame
    //   var        : per-axis measurement variance (m/s²)² for the residual.
    //                Caller derives from sensor Allan variance: typical
    //                Android phones have white-noise σ ≈ 0.1 m/s² so
    //                var ≈ 0.01 (m/s²)² is a defensible floor; inflate when
    //                the body is detected to be under linear acceleration.
    //
    // Returns true on accept (residual passed gating + Joseph update applied),
    // false on reject (EKF not init, |accel| outside valid band, or covariance
    // factorisation failed).
    [[nodiscard]] bool updateGravityAlignment(const cv::Mat& accel_body, double var);

    // Step 2 (visual prod plan): per-frame relative-rotation update from
    // monocular `recoverPose`. R_meas_body is the body-frame rotation
    // taking points expressed in the clone's body frame to the current
    // body frame. The caller is responsible for converting R_vo from the
    // OpenCV camera frame into the body frame before calling this method
    // (the body↔camera extrinsic lives in Tracker, not in the EKF).
    //
    // sigma_axis_sq is the per-axis variance (rad²) of the measurement;
    // for monocular E-matrix RANSAC the standard pixel-noise model is
    //     sigma_axis_sq ≈ (RANSAC_THRESH / (focal * sqrt(N_inliers)))^2
    //
    // clone_id selects which clone in the sliding window the relative
    // rotation is measured against. Returns false if the clone is not
    // found, the EKF is not fully initialized, or R_meas_body is malformed.
    [[nodiscard]] bool updateRelativeRotation(const cv::Mat& R_meas_body,
                                double sigma_axis_sq,
                                int clone_id);

    // Plan Step 7 (ADR-013 §"Correction injection — absolute pose path"):
    // absolute world-frame pose measurement. Used by loop-closure when
    // the matched keyframe's clone has been marginalised out of the
    // sliding window — the clone-based relative-pose channels can't
    // reach those references, so this channel applies the correction
    // directly to the IMU-state's world-frame position and attitude.
    //
    //   target_R_world_imu : 3x3 CV_64F, world->imu rotation the loop
    //                        closure says we should be at. Caller is
    //                        responsible for composing the matched
    //                        keyframe's stored R_world_cam with the
    //                        relative R_now_to_match and the camera-IMU
    //                        extrinsic to land in IMU frame.
    //   target_p_world     : 3x1 CV_64F, world-frame IMU position the
    //                        loop closure says we should be at. Same
    //                        composition responsibility.
    //   sigma_axis_sq_R    : per-axis variance of the rotation
    //                        measurement (rad²). Loop closure derives
    //                        this from PnP inlier count + damping
    //                        schedule.
    //   var_p              : per-axis variance of the position
    //                        measurement (m²). Same source.
    //
    // Returns true on success (Joseph-form update applied). False on
    // (a) EKF not full-init, (b) malformed Mat inputs, (c) the chi²
    // gate trips (residual > a generous outer threshold — protects
    // against wildly wrong loop matches before damping has a chance
    // to fade them in).
    [[nodiscard]] bool updateAbsolutePose(const cv::Mat& target_R_world_imu,
                            const cv::Mat& target_p_world,
                            double sigma_axis_sq_R,
                            double var_p);

    // PDR step constraint: world-frame XZ position increment from a step
    // event. var is variance of each component (m²).
    [[nodiscard]] bool updatePDRStep(double dx_world, double dy_world, double var);

    // ── Phase 1 Step 6.3 (post_v19_sprint_plan.md, 2026-05-16) ────────────────
    //
    // Pose-only Kalman measurement update against fixed 3D landmarks
    // (ORB-SLAM3 "Tracking" pattern, Campos et al. 2021, §III.B): the camera
    // pose is updated while map points are held fixed (no Jacobian column for
    // them). Used by Tracker.cpp (Phase 6.4) to fold persistent LandmarkMap
    // observations into the EKF so revisits anchor pose drift back to the
    // historical map.
    //
    // Each LandmarkObservation pairs a world-frame landmark position with
    // the pixel where the current camera observed it. The caller is
    // responsible for the data-association step (descriptor match,
    // projection-radius gate) and for picking `sigma_px` from the
    // landmark's reprojection-noise estimate. `landmark_id` is opaque to
    // the EKF — it appears only in rejection log lines so the caller can
    // grep its trajectory.
    //
    // Returned counters are PER-CALL (not cumulative). The caller (Tracker)
    // is responsible for forwarding into EventCounters; this keeps EKFState
    // pure and lets the Tracker decide how to bucket counters across sites.
    struct LandmarkObservation {
        int          landmark_id;   // for logging only — caller-controlled id
        cv::Vec3d    p_world;       // FIXED landmark position (Z-up world frame)
        cv::Point2f  pixel_meas;    // observed pixel in current camera image
        double       sigma_px;      // per-obs measurement noise (caller picks)
    };

    struct LandmarkUpdateResult {
        int    accepted               = 0;
        int    rejected_chi2          = 0;
        int    rejected_huber         = 0;
        int    rejected_depth         = 0;
        int    rejected_outside_image = 0;
        double chi2_total             = 0.0;
    };

    // Apply a batch of fixed-landmark observations as a 2N-row pose-only
    // measurement update. Builds H_stack with non-zero columns only at
    // rows 0..2 (δθ) and 12..14 (δp_G); the rest of the error state is
    // updated via the standard Joseph-form correlation in applyMSCKFUpdate.
    //
    // Gates (in order, per-observation):
    //   1. depth gate         — kLandmarkDepthMinM ≤ p_C.z ≤ kLandmarkDepthMaxM
    //   2. in-image gate      — predicted pixel inside [0,W) × [0,H)
    //   3. per-obs χ² gate    — m² ≤ kLandmarkChi2Gate95 (χ²(0.95, 2-DOF))
    //   4. per-obs Huber      — pixel residual ≥ 5·kLandmarkHuberPx is a
    //                            hard reject; in (δ, 5·δ) the row is
    //                            de-weighted to (δ / r_px).
    //
    // Returns the per-call rejection breakdown. Caller-side flow:
    //
    //     auto r = ekf.applyLandmarkObservations(obs, R_bc, fx, fy, cx, cy, W, H);
    //     counters.landmarks_msckf_accepted_total.fetch_add(r.accepted, ...);
    //     counters.landmarks_msckf_rejected_chi2_total.fetch_add(r.rejected_chi2, ...);
    //     ...
    //
    // Returns an all-zero LandmarkUpdateResult on (a) EKF not full-init,
    // (b) malformed inputs, (c) empty `obs` vector — no exception.
    [[nodiscard]] LandmarkUpdateResult applyLandmarkObservations(
        const std::vector<LandmarkObservation>& obs,
        const cv::Matx33d& R_bc,
        double fx, double fy,
        double cx, double cy,
        int img_width, int img_height);

    // Gravity-aligned yaw from the current R_GtoI_, using Madgwick
    // roll/pitch as the alignment frame. Returns yaw in radians
    // (navigation conv: CW-positive, North=0).
    double getYaw(double roll, double pitch) const;

    // 2026-05-16 audit Fix B — canonical single-source heading accessor.
    //
    // Returns nav-CW yaw (rad) from R_GtoI_ using the same gravity-alignment
    // sandwich as getYaw(roll, pitch), but derives roll and pitch internally
    // from the gravity column of R_GtoI_ (column 2, world-Z direction in body
    // frame) via standard tilt-angle decomposition, so the caller is NOT
    // required to supply Madgwick angles. This eliminates the 4-formula
    // duplication flagged in architecture audit Finding 2.1.
    //
    // Convention (Z-up ENU, nav-CW):
    //   North:  R_GtoI has body-x ≈ world-x  →  atan2(R[1,0], R[0,0]) ≈ 0
    //   East:   body-x ≈ world-y              →  atan2(R[1,0], R[0,0]) ≈ +π/2
    //   South:  body-x ≈ -world-x             →  atan2(R[1,0], R[0,0]) ≈ ±π
    //   West:   body-x ≈ -world-y             →  atan2(R[1,0], R[0,0]) ≈ -π/2
    //
    // Unit-test-style expected outputs for vertical-phone walking (phone held
    // portrait, screen toward stomach, camera facing forward):
    //   Walking North (body-x ≈ world-x, R[1,0]≈0, R[0,0]≈1):   result ≈  0.0 rad
    //   Walking East  (body-x ≈ world-y, R[1,0]≈1, R[0,0]≈0):   result ≈ +π/2 rad
    //   Walking South (body-x ≈ -world-x, R[1,0]≈0, R[0,0]≈-1): result ≈ ±π rad
    //   Walking West  (body-x ≈ -world-y, R[1,0]≈-1, R[0,0]≈0): result ≈ -π/2 rad
    //
    // Returns 0.0 if the EKF is not full-initialized or R_GtoI_ is empty.
    double getWorldHeadingRad() const;

    // Clone accessors (used by UpdaterMSCKF)
    bool getClonePose(int state_id, cv::Mat& R_GtoC, cv::Mat& p_G) const;
    bool getCloneFEJ(int state_id, cv::Mat& R_FEJ, cv::Mat& p_FEJ) const;
    int getCloneCovIdx(int state_id) const;
    int getStateDim() const;
    cv::Mat getCovariance() const { return P_.clone(); }

    // Window access
    const std::deque<CameraPose>& getWindow() const { return window_; }
    int getLatestCloneId() const;

    // ── Plan Step 6 (ADR-012): thread-safe snapshot for windowed BA ─────────
    //
    // Read-only copy of the most-recent `max_clones` camera poses, ordered
    // oldest first so the BA gauge-fix pick (oldest = anchor) is unambiguous.
    // Designed to be called from the BA worker thread; the camera thread
    // mutates `window_` exclusively through addClone / marginalizeOldestClone /
    // pruneWindow, and those mutators take the same `snapshot_mutex_` while
    // splicing the deque so the snapshot reader never observes a torn state.
    //
    // The lock is held only for the duration of the deque walk + a per-pose
    // 3x3 + 3x1 matrix clone — i.e. O(max_clones) work, no allocations beyond
    // the returned vector. Camera-thread writers see ≤ a few microseconds of
    // lock contention per frame, which is negligible against the per-frame
    // budget. The snapshot returns the CURRENT mean (not FEJ); BA refines
    // poses against pixel observations, so the mean is the correct linearisation
    // point and FEJ values are not needed.
    struct CloneSnapshot {
        int          clone_id   = -1;   // matches CameraPose::state_id
        int64_t      timestamp_ns = 0;
        cv::Matx33d  R;                 // 3x3 world -> camera (row-major)
        cv::Vec3d    t;                 // camera-in-world
    };
    std::vector<CloneSnapshot> getCloneSnapshot(int max_clones = 5) const;

    // 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
    //
    // Cause: the JNI overlay render path (native-lib.cpp getCurrentCameraPose +
    //   getSlamSnapshot) calls multiple EKF accessors (getRotation, getPosition,
    //   getExtrinsicsRotation, plus per-feature getSlamFeatureGlobalPosition which
    //   internally calls getClonePose). Each call is unsynchronised against the
    //   camera thread; between two JNI hops the camera thread can run a full
    //   processFrame cycle (~16-33 ms), mutating R_GtoI_/p_G_/window_/slam_features_.
    //   Result: the camera pose used to project SLAM dots is from a different
    //   EKF state-version than the feature world positions → sub-cm pose drift
    //   produces 1-2 px visible sliding even when phone is stationary.
    //
    // Change: this method takes snapshot_mutex_ once (same mutex protecting
    //   window_ + slam_features_ + P_ splicing in addClone / marginalizeOldestClone)
    //   and reads R_GtoI, p_G, R_bc, intrinsics, AND every SLAM feature's world
    //   position INSIDE the same critical section. Caller (native-lib.cpp) gets
    //   one coherent EKF state-version per overlay tick.
    //
    // Falsifier: post-fix walk — hold phone still for 5+ s; orange dots should
    //   be pixel-perfect anchored to image features. Visible sliding ⇒ root
    //   cause is elsewhere (raw sensor noise, Madgwick wobble, or inverse-depth
    //   state noise) and this fix is insufficient.
    //
    // Falsifier log site: native-lib.cpp logs once per second the snapshot
    //   yaw + p_G + slam_features.size() so post-hoc analysis can verify the
    //   new path runs and the snapshot is internally consistent.
    struct OverlaySnapshot {
        bool         full_initialized = false;                              // false → caller should bail
        cv::Matx33d  R_GtoI{1.0, 0.0, 0.0,  0.0,  1.0, 0.0,  0.0,  0.0,  1.0};  // world Z-up → body (identity default)
        cv::Vec3d    p_G{0.0, 0.0, 0.0};                                    // body in world Z-up (m)
        cv::Matx33d  R_bc{1.0, 0.0, 0.0,  0.0, -1.0, 0.0,  0.0,  0.0, -1.0};    // body → camera (matches R_bc_ default)
        double       slam_fx = 500.0;
        double       slam_fy = 500.0;
        double       slam_cx = 320.0;
        double       slam_cy = 240.0;
        struct SlamFeatureEntry {
            int       feature_id = -1;
            cv::Vec3d p_world{0.0, 0.0, 0.0};                               // Z-up world frame (m)
        };
        std::vector<SlamFeatureEntry> slam_features;

        // 2026-05-19 — Orange-dot anchor-relative render support.
        //
        // Cause: getLandmarkSnapshot (native-lib.cpp) wants to render each
        // landmark anchored to its host clone's CURRENT live pose so the
        // projection compensates for EKF drift since add time. It needs a
        // (clone_id → R_GtoC, p_G) lookup table populated from the SAME
        // snapshot tick as R_GtoI / p_G / slam_features so the overlay
        // pose, live SLAM dots, and orange dots are all derived from one
        // consistent EKF state-version.
        //
        // Window size is bounded by EKFState::MAX_CLONES (~15), so the copy
        // cost is trivial. Use the same-tick `snapshot_mutex_` critical
        // section that already walks window_ for slam_features above.
        struct CloneEntry {
            int         state_id = -1;
            cv::Matx33d R_GtoC{1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0};  // world→camera
            cv::Vec3d   p_G{0.0, 0.0, 0.0};                                      // camera position in world (Z-up)
        };
        std::vector<CloneEntry> clones;
    };
    OverlaySnapshot snapshotForOverlay() const;

    // DEAD CODE: getFEJ — only used by disabled MSCKF updater
    // void getFEJ(int state_id, cv::Mat& R_fej, cv::Mat& p_fej) const;

    // IMU state accessors
    cv::Mat getRotation() const { return R_GtoI_.clone(); }
    cv::Mat getPosition() const { return p_G_.clone(); }
    // 2026-05-12: expose the EKF-estimated gyro bias so Tracker can close the
    // feedback loop to IMUPreintegrator::gyro_bias_. Without this sync, every
    // MSCKF visual update writes a bias correction into b_g_ that is never
    // applied to the integrator, so subsequent imu_delta.deltaR continues
    // integrating with stale bias → R_GtoI accumulates rotation error.
    cv::Mat getGyroBias() const { return b_g_.clone(); }

    // 2026-05-16 — Reset b_g_ mean to a specified value WITHOUT modifying the
    // covariance P_.
    //
    // Cause this exists: the original feedback loop at Tracker.cpp:1039-1045
    // (added 2026-05-12 by the architecture-review swarm) overwrites the
    // IMUPreintegrator's gyro_bias_ with the EKF's b_g_ every frame. b_g_
    // tracks the RESIDUAL bias on top of what the IMU already subtracts —
    // typically ~0.02 rad/s when the calibrated IMU bias is ~0.18 rad/s. The
    // overwrite collapses IMU's calibrated bias to the small residual,
    // leaving ~0.16 rad/s of unsubtracted gyro entering preintegration.
    // R_GtoI then rotates phantomly around body-Y at ~9°/s, mis-rotating
    // R^T·deltaV to a constant wrong world direction; position propagates
    // there regardless of actual heading.
    //
    // Correct loop (this method enables): the caller READS b_g_, ADDS it to
    // the IMU's gyro_bias_ (absorbing the EKF residual into the IMU's full
    // bias estimate), then calls setGyroBias(0,0,0) here. The residual is
    // now zero from the EKF's perspective; the FULL bias lives in the IMU
    // side. Subsequent preintegration sees correctly-corrected gyro.
    //
    // Covariance P_ is intentionally NOT modified. The uncertainty about the
    // bias hasn't changed — we just transferred where the estimate is held.
    // The next MSCKF update that touches b_g_ will accumulate fresh residual.
    // External synchronization required (caller holds Tracker::pose_mutex_ —
    // same convention as updateZRUP / applyMSCKFUpdate).
    void setGyroBias(double bgx, double bgy, double bgz) {
        if (b_g_.empty()) return;
        b_g_.at<double>(0, 0) = bgx;
        b_g_.at<double>(1, 0) = bgy;
        b_g_.at<double>(2, 0) = bgz;
    }

    // Bootstrap-only: replace R_GtoI_ with the magnetometer-derived initial
    // heading rotation when Kotlin's setInitialHeading lands AFTER ekf_.initializeFull
    // has already fired (which is the normal case on Android — handleVioInitialized
    // is dispatched on the UI thread, by which time processFrame has full-init'd
    // the EKF with R_GtoI=Identity from the InertialInitializer). Without this,
    // the EKF carries an arbitrary initial-heading state forever and loop
    // closure / chi² rejects every correction as a 180° teleportation.
    // Bug surfaced 2026-05-09 on sim 1778260615221 (vyaw stuck near 0 while
    // Madgwick hdg correctly tracked compass heading).
    void setRotation(const cv::Mat& R_GtoI);
    // 2026-05-16 re-enabled — used by UpdaterMSCKF velocity-divergence diagnostic.
    cv::Mat getVelocity() const { return v_G_.clone(); }
    bool isFullInitialized() const { return full_initialized_; }

    // DEAD CODE: updateTemporal — never called
    // void updateTemporal(double observed_scale, double confidence, double H_td);
    void setTimeOffset(double td_seconds);
    double getTimeOffset() const { return t_offset_cam_imu_; }
    // Returns the 1-sigma uncertainty of the online td estimate.
    // When full_initialized_, reads from P_(15,15) which is updated by every
    // MSCKF correction. Before that, falls back to the pre-init scalar P_td_.
    double getTimeOffsetStd() const {
        if (full_initialized_ && !P_.empty() && P_.rows > 15) {
            return std::sqrt(std::max(0.0, P_.at<double>(15, 15)));
        }
        return std::sqrt(std::max(0.0, P_td_));
    }

    // ── Plan Step 8b: online IMU-camera extrinsics calibration ───────────────
    //
    // Set the nominal body→camera rotation. Must be called before or at
    // initializeFull. The argument is the rotation that transforms body-frame
    // vectors into camera-frame vectors:  p_cam = R_bc * p_body.
    //
    // Call with the result of getCameraOrientation (Android sensor frame to
    // camera frame) composed with the device mounting geometry. The EKF
    // will refine this nominal value via the δφ_bc error state during the
    // visual MSCKF update.
    //
    // Thread safety: must be called from the same thread that owns the EKF
    // (i.e. before processFrame starts) — no mutex is taken.
    void setExtrinsicsRotation(const cv::Matx33d& R_bc);

    // Read the current best-estimate body→camera rotation. This is the
    // nominal R_bc_ updated in-place by every MSCKF correction of δφ_bc.
    // Returns a copy; safe to call from any thread that does not concurrently
    // call applyMSCKFUpdate (i.e., reads are safe from the camera thread).
    cv::Matx33d getExtrinsicsRotation() const { return R_bc_; }

    // Returns the angle-from-identity of the current R_bc estimate in degrees.
    // Used for the event_summary diagnostic field "extrinsics_rotation_delta_deg".
    // A freshly-initialized system from getCameraOrientation returns ~0°; as
    // the EKF refines, the delta from the initial nominal should shrink to <1°.
    double getExtrinsicsAngleDeg() const;

    // Returns the timestamp_ns stored in the clone identified by clone_id.
    // Returns 0 if the clone is not found (not in current sliding window).
    int64_t getCloneTimestamp(int clone_id) const;

    // ── Plan Step 8a + Step 8b: IMU error-state dimension ──────────────────
    //
    // Step 8a (ADR-016) added δt_d (time delay, 1 DOF) at row 15.
    // Step 8b (this step) adds δφ_bc (body→camera rotation error, 3 DOF) at
    // rows 16–18. Full IMU error-state layout after both steps:
    //
    //   rows  0– 2  : δθ    — attitude error (rad, body frame)
    //   rows  3– 5  : δb_g  — gyro bias error (rad/s)
    //   rows  6– 8  : δv    — velocity error (m/s, global frame)
    //   rows  9–11  : δb_a  — accel bias error (m/s²)
    //   rows 12–14  : δp    — position error (m, global frame)
    //   row  15     : δt_d  — camera-IMU time delay error (s) — Step 8a
    //   rows 16–18  : δφ_bc — body→camera rotation perturbation (rad, Lie
    //                         algebra so(3)) — Step 8b
    //
    // Step 8b convention: R_bc = R_bc_hat × Exp(δφ_bc)
    // where R_bc_hat is the nominal body→camera rotation (R_bc_) and Exp is
    // the matrix exponential on SO(3) (implemented via cv::Rodrigues).
    // "body→camera" means p_cam = R_bc * p_body.
    static constexpr int IMU_STATE_DIM    = 19;  // δθ, δb_g, δv, δb_a, δp, δt_d, δφ_bc
    // Named offsets for the two new DOF blocks (Steps 8a + 8b).
    // TD_STATE_OFFSET   = 15: row of δt_d (one DOF after the 15-row base IMU state).
    // EXTR_STATE_OFFSET = 16: first row of δφ_bc (immediately after δt_d).
    static constexpr int TD_STATE_OFFSET   = 15;
    static constexpr int EXTR_STATE_OFFSET = 16;
    static constexpr int EXTR_STATE_DIM    = 3;   // one 3-vector δφ_bc
    static constexpr int CLONE_DIM = 6;            // δθ_c, δp_c
    // Phase 1 Step 4b (post_v19_sprint_plan.md): bumped from 11 -> 15 to give
    // SLAM features ~3 s anchor-clone lifetime instead of ~2 s at 5 Hz KF rate.
    // Stacks with Fix 4a (reanchorSlamFeature) for cumulative benefit. Also
    // aligned with Agent 1's scooter-mode architectural recommendation.
    static constexpr int MAX_CLONES = 15;

    // Plan Step 3a (ADR-008): how many residuals were hard-rejected by the
    // Huber kernel on the last applyMSCKFUpdate call. Surfaced for logcat
    // visibility so we can see MSCKF outlier behaviour at runtime.
    int getMSCKFHuberRejectedCount() const { return msckf_huber_rejected_count_; }

    struct SlamFeature {
        int feature_id{-1};
        int anchor_clone_id{-1};
        // 5-DOF state vector: [α, β, ρ, pad0, pad1]. Active math on first 3.
        cv::Mat state;            // 5x1 CV_64F
        // First-Estimate Jacobians (FEJ) — locked at promotion.
        cv::Mat p_global_FEJ;     // 3x1 CV_64F: linearisation point in world
        cv::Mat anchor_R_FEJ;     // 3x3 CV_64F
        cv::Mat anchor_p_FEJ;     // 3x1 CV_64F
        // Diagnostics (RMS history maintained by Tracker; EKF reads only).
        double last_obs_rms{0.0};
        int    rms_bad_run{0};
    };

    // Build the 2x5 reprojection Jacobian (∂h/∂α, ∂h/∂β, ∂h/∂ρ, 0, 0) for a
    // SLAM feature observed from a specific clone, plus the 2x6 Jacobian
    // w.r.t. the observing clone's (δθ, δp). FEJ-locked — uses anchor and
    // clone FEJ poses for the linearisation. `pred_uv` is the predicted
    // normalised image coords using CURRENT mean state (for residual).
    //
    // Returns false on degenerate geometry (point behind camera, |p_C.z| ≤
    // 1e-4 m).
    bool slamReprojectionJacobian(const SlamFeature& f,
                                  const cv::Mat& clone_R_FEJ,
                                  const cv::Mat& clone_p_FEJ,
                                  const cv::Mat& clone_R_now,
                                  const cv::Mat& clone_p_now,
                                  cv::Mat& H_feature_2x5,
                                  cv::Mat& H_clone_2x6,
                                  cv::Point2d& pred_uv) const;

    // Direct access to SLAM features for updaters.
    std::vector<SlamFeature>& getSlamFeatures() { return slam_features_; }
    const std::vector<SlamFeature>& getSlamFeatures() const { return slam_features_; }

    // Phase 1 Step 4a: transfer a SLAM feature's anchor to a new clone,
    // preserving its world position. Called from marginalizeOldestCloneNoLock
    // before the oldest clone is popped from the window. Returns false if the
    // re-projection into the new anchor's FEJ camera frame is geometrically
    // degenerate (Zc <= 0.01 m).
    bool reanchorSlamFeature(int slot, int new_anchor_id);

private:
    // Legacy scale estimation state
    double scale_;
    double scale_fej_{-1.0};
    double P_scale_;

    // Full IMU state (mean)
    cv::Mat R_GtoI_;    // 3x3 rotation Global-to-IMU (world->body, Z-up world)
    cv::Mat b_g_;       // 3x1 gyro bias
    cv::Mat v_G_;       // 3x1 velocity in global frame
    cv::Mat b_a_;       // 3x1 accel bias
    cv::Mat p_G_;       // 3x1 position in global frame

    // Full covariance: (19 + 6*N_clones) x (19 + 6*N_clones).
    cv::Mat P_;

    // MSCKF Sliding Window
    std::deque<CameraPose> window_;
    int next_state_id_{0};
    bool full_initialized_{false};

    // Plan Step 6 (ADR-012): snapshot mutex protecting window_.
    mutable std::mutex snapshot_mutex_;

    // Online time-offset calibration.
    double t_offset_cam_imu_{0.010};
    double P_td_{0.005 * 0.005};  // pre-init variance fallback (s²)

    // Plan Step 8b: body→camera extrinsics (online calibration).
    cv::Matx33d R_bc_{1.0,  0.0,  0.0,
                      0.0, -1.0,  0.0,
                      0.0,  0.0, -1.0};

    bool initialized_{false};

    // FEJ Global Locking
    cv::Mat global_first_estimate_R_;
    cv::Mat global_first_estimate_p_;
    bool global_fej_initialized_{false};

    // IMU noise parameters (continuous-time).
    //
    // !!! TODO 2026-05-16 — REQUIRES ALLAN-VARIANCE MEASUREMENT !!!
    //
    // These values are uncalibrated placeholders inherited from project history.
    // A 2026-05-16 review attempted to "fix" them to allegedly-published values
    // (sigma_g=0.005, sigma_a=0.003, sigma_bg=5e-5, sigma_ba=2e-4) attributed to
    // Geneva et al. 2020 ICRA §V-B. That citation was NOT verified against the
    // actual paper. The fix was REVERTED on the same day to avoid shipping a
    // magic number under a fabricated citation. The implementor skill bans
    // magic-number tuning without data citation (anti-pattern #3).
    //
    // To replace these properly:
    //   1. Capture a 4+ hour stationary IMU log on the Samsung Galaxy S21 Ultra
    //      (SM-G998B, Exynos 2100). Sample at 200 Hz.
    //   2. Run Allan-deviation analysis (e.g., gyro_allan, Skog & Händel 2012):
    //      ARW = gyro Allan dev at τ=1 s
    //      BI  = gyro Allan dev minimum
    //   3. Set sigma_g_ = ARW (rad/s/√Hz), sigma_bg_ = BI (rad/s/√Hz).
    //   4. Same for accel: sigma_a_ = VRW, sigma_ba_ = accel BI.
    //   5. Cite the captured log file timestamp and Allan-dev plot in this
    //      comment block when shipping the change.
    //
    // The Q matrix being too large or too small here directly affects how
    // tightly the EKF trusts IMU vs. visual evidence and is one of the
    // longest-running uncertainties in the codebase.
    //
    // 2026-05-17 — CALIBRATED via Allan variance.
    //   Source: tests/sims/allan/imu_calibration_20260517_021657.csv
    //   Duration: 7200 s   Sample rate: 500 Hz   Samples: 3,600,002
    //   Phone: S21 Ultra (SM-G998B), flat screen-up on stable surface
    //   Analyzer: scripts/allan_variance/calibrate.py (worst-axis aggregation)
    //
    // Comparison to prior uncalibrated values (kept below as SUPERSEDED):
    //   sigma_g:  0.01    -> 1.98e-4   (50.5× tighter)
    //   sigma_a:  0.1     -> 1.63e-3   (61.3× tighter)
    //   sigma_bg: 1e-4    -> 7.46e-7   (134× tighter)
    //   sigma_ba: 1e-3    -> 5.41e-4   (1.85× tighter)
    //
    // Sanity check vs published Bosch BMI260 spec: our gyro 1.98e-4 rad/s/√Hz
    // = 0.0114°/s/√Hz vs spec 0.014°/s/√Hz; our accel 166 µg/√Hz vs spec
    // 130 µg/√Hz. Both within ~30% of vendor spec — clean recording.
    // 2026-05-17 (evening) — σ_bg + σ_ba RAISED above Allan steady-state values.
    //
    // Cause: Allan-measured σ_bg = 7.46e-7 is correct for the phone's TRUE
    // bias drift rate, but the EKF's bias estimate after the 5-second
    // stationary cal can be off by ~0.02 rad/s due to:
    //   (a) the cal's variance-gate filter accepting only the lowest-noise
    //       subset of samples, biasing the averaged bias toward zero
    //   (b) ambient vibration / micro-motion the cal misclassifies as zero
    // With σ_bg = 7.46e-7, the EKF's Kalman gain for bias correction is so
    // small that ZRUP fires can't drive the estimate to truth — observed as
    // persistent 1.15°/s yaw drift on a verifiably-stationary phone (v38
    // walk, 04:09-04:12 logcat, gyro=0.020 rad/s after correction).
    //
    // Change: σ_bg raised from 7.46e-7 → 1e-5 (13.4× looser than Allan,
    // but still 10× tighter than the old uncalibrated 1e-4). σ_ba follows
    // the same logic, raised from 5.4e-4 → 2e-3 (3.7× looser).
    //
    // Falsifier: post-fix stationary observation should show heading drift
    // < 0.3°/s sustained (vs current 1.15°/s). If drift unchanged, the
    // residual gyro reading isn't bias at all — it's environmental
    // vibration the IMU is correctly reporting and we have a different
    // problem (surface or building motion).
    double sigma_g_{1.98e-4};   // gyro noise density (rad/s/sqrt(Hz))         — Allan 2026-05-17
    double sigma_a_{1.63e-3};   // accel noise density (m/s^2/sqrt(Hz))        — Allan 2026-05-17
    double sigma_bg_{1.0e-5};   // gyro bias RW — Allan 7.46e-7, raised 13× for init-error tolerance
    double sigma_ba_{2.0e-3};   // accel bias RW — Allan 5.41e-4, raised 3.7× for init-error tolerance
    /* SUPERSEDED 2026-05-17 — Allan calibration replaced the uncalibrated
       placeholders. Kept for archival; do not re-enable without re-running
       a fresh stationary recording.
    double sigma_g_{0.01};      // gyro noise density (rad/s/sqrt(Hz))   — UNCALIBRATED
    double sigma_a_{0.1};       // accel noise density (m/s^2/sqrt(Hz))  — UNCALIBRATED
    double sigma_bg_{0.0001};   // gyro random walk                       — UNCALIBRATED
    double sigma_ba_{0.001};    // accel random walk                      — UNCALIBRATED
    */

    // Constants
    static constexpr double SIGMA_SCALE_RW = 0.001;
    static constexpr double SIGMA_SCALE_MEAS = 0.1;

    // Plan Step 3a (ADR-008): MSCKF damping + Huber state
    int msckf_frames_since_call_{0};
    int msckf_damping_step_{0};
    int msckf_huber_rejected_count_{0};
    int extr_log_skip_{0};

    static constexpr int MSCKF_QUIET_PROPAGATION = 5;
    static constexpr int MSCKF_DAMPING_RAMP_FRAMES = 5;
    static constexpr double MSCKF_HUBER_DELTA = 2.4477;

    double computeMSCKFDampingFactor() const;

    // Plan Step 3b (ADR-009): SLAM feature internals
    std::vector<SlamFeature> slam_features_;

    // 2026-05-09 telemetry fix — see takeLastMarginalizedSlamFeatureIds().
    std::vector<int> last_marginalized_slam_feature_ids_;

    // Cached camera intrinsics for SLAM reprojection.
    double slam_fx_{500.0};
    double slam_fy_{500.0};
    double slam_cx_{320.0};
    double slam_cy_{240.0};

    // Plan Step 6 (ADR-012): no-lock variant for re-entrant call sites.
    void marginalizeOldestCloneNoLock();

    int slamFeatureCovIdxInternal(int slot) const;
    int slamBlockStart() const;
};
