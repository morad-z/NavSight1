#pragma once
#include <vector>
#include <deque>
#include <mutex>
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
    bool updateStationaryAccel(const cv::Mat& accel_body, double sigma_a);
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
    void pruneWindow(size_t max_poses = 11);
    void marginalizeOldestClone();

    // MSCKF update: apply stacked null-space-projected measurement
    void applyMSCKFUpdate(const cv::Mat& H, const cv::Mat& res, const cv::Mat& R_noise);

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
    bool updateRelativePose(const cv::Mat& t_world_metric,
                            int clone_id,
                            double var_t);

    // Keyframe yaw correction (1-DOF). yaw_meas is in world frame
    // (navigation conv: CW-positive, North=0), var in rad². roll/pitch
    // come from the Madgwick filter and define the gravity-alignment
    // sandwich used to derive the H jacobian.
    bool updateGravityAlignedYaw(double yaw_meas, double var,
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
    bool updateGravityAlignment(const cv::Mat& accel_body, double var);

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
    bool updateRelativeRotation(const cv::Mat& R_meas_body,
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
    bool updateAbsolutePose(const cv::Mat& target_R_world_imu,
                            const cv::Mat& target_p_world,
                            double sigma_axis_sq_R,
                            double var_p);

    // PDR step constraint: world-frame XZ position increment from a step
    // event. var is variance of each component (m²).
    bool updatePDRStep(double dx_world, double dy_world, double var);

    // Gravity-aligned yaw from the current R_GtoI_, using Madgwick
    // roll/pitch as the alignment frame. Returns yaw in radians
    // (navigation conv: CW-positive, North=0).
    double getYaw(double roll, double pitch) const;

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

    // DEAD CODE: getFEJ — only used by disabled MSCKF updater
    // void getFEJ(int state_id, cv::Mat& R_fej, cv::Mat& p_fej) const;

    // IMU state accessors
    cv::Mat getRotation() const { return R_GtoI_.clone(); }
    cv::Mat getPosition() const { return p_G_.clone(); }

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
    // DEAD CODE: getVelocity — never called
    // cv::Mat getVelocity() const { return v_G_.clone(); }
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
    static constexpr int MAX_CLONES = 11;

    // Plan Step 3a (ADR-008): how many residuals were hard-rejected by the
    // Huber kernel on the last applyMSCKFUpdate call. Surfaced for logcat
    // visibility so we can see MSCKF outlier behaviour at runtime.
    int getMSCKFHuberRejectedCount() const { return msckf_huber_rejected_count_; }

private:
    // Legacy scale estimation state
    double scale_;
    double scale_fej_{-1.0};
    double P_scale_;

    // Full IMU state (mean)
    //
    // World frame: ENU Z-up (X=East, Y=North, Z=Up), right-handed. Matches
    // the Madgwick attitude filter (IMUPreintegrator.cpp:677,774) and
    // Android sensor body frame (body Z is gravity-aligned when phone is
    // held screen-up flat). gravity = (0, 0, -9.81) m/s².
    //
    // R_GtoI_ is world→body: takes a vector expressed in world coordinates
    // and returns its representation in body (IMU) coordinates. For body
    // at compass heading ψ (CW-positive nav, North=0, East=+π/2) with
    // zero roll/pitch, R_GtoI_ has the structure built by
    // Tracker::setInitialHeading at Tracker.cpp:322:
    //     R_GtoI_ = [[cos ψ, -sin ψ, 0], [sin ψ, cos ψ, 0], [0, 0, 1]]
    // and getYaw extracts ψ via atan2(R[1,0], R[0,0]).
    cv::Mat R_GtoI_;    // 3x3 rotation Global-to-IMU (world->body, Z-up world)
    cv::Mat b_g_;       // 3x1 gyro bias
    cv::Mat v_G_;       // 3x1 velocity in global frame
    cv::Mat b_a_;       // 3x1 accel bias
    cv::Mat p_G_;       // 3x1 position in global frame

    // Full covariance: (19 + 6*N_clones) x (19 + 6*N_clones).
    // IMU block rows 0..18; clone blocks follow; SLAM block at the end.
    cv::Mat P_;

    // MSCKF Sliding Window
    std::deque<CameraPose> window_;
    int next_state_id_{0};
    bool full_initialized_{false};

    // ── Plan Step 6 (ADR-012): snapshot mutex ────────────────────────────────
    // Guards `window_` against torn reads from the BA worker thread. Camera-
    // thread writers (addClone, marginalizeOldestClone, pruneWindow) take it
    // briefly while splicing; getCloneSnapshot takes it for the deque walk.
    // mutable because getCloneSnapshot is const and locks for read.
    mutable std::mutex snapshot_mutex_;

    // Online time-offset calibration.
    // t_offset_cam_imu_ is the mean estimate (seconds). δt_d (row 15 of the
    // error state) is the CORRECTION — each MSCKF update adds dx[15] to this
    // mean and P_(15,15) tracks the variance. P_td_ is kept only as the
    // pre-initializeFull fallback for getTimeOffsetStd() and setTimeOffset().
    double t_offset_cam_imu_{0.010};
    double P_td_{0.005 * 0.005};  // pre-init variance fallback (s²)

    // ── Plan Step 8b: body→camera extrinsics (online calibration) ────────────
    //
    // R_bc_ is the nominal (best-estimate) body→camera rotation matrix.
    // Updated in-place by every applyMSCKFUpdate: R_bc_ = R_bc_ * Exp(δφ_bc).
    //
    // Default initialisation: diag(1, -1, -1) — the canonical rear-camera /
    // vertical-phone convention NavSight uses (camera +X = body +X, camera +Y =
    // body -Y, camera +Z = body -Z). This is the same matrix that was previously
    // hardcoded at three sites in Tracker.cpp (lines ~1129, ~2066, ~3440).
    //
    // Caller sets the real value via setExtrinsicsRotation() before the first
    // processFrame, using Android's CameraCharacteristics.SENSOR_ORIENTATION
    // combined with the device mounting geometry. The default keeps the system
    // functional for devices not providing the rotation at startup.
    cv::Matx33d R_bc_{1.0,  0.0,  0.0,
                      0.0, -1.0,  0.0,
                      0.0,  0.0, -1.0};

    bool initialized_{false};

    // FEJ Global Locking
    cv::Mat global_first_estimate_R_;
    cv::Mat global_first_estimate_p_;
    bool global_fej_initialized_{false};

    // IMU noise parameters (continuous-time, used for covariance propagation)
    double sigma_g_{0.01};      // gyro noise density (rad/s/sqrt(Hz))
    double sigma_a_{0.1};       // accel noise density (m/s^2/sqrt(Hz))
    double sigma_bg_{0.0001};   // gyro random walk
    double sigma_ba_{0.001};    // accel random walk

    // Constants
    static constexpr double SIGMA_SCALE_RW = 0.001;
    static constexpr double SIGMA_SCALE_MEAS = 0.1;

    // ── Plan Step 3a (ADR-008): MSCKF damping + Huber state ────────────────
    // ADR-006 documented 5–11 m teleportations when MSCKF corrections were
    // applied without damping. We re-enable MSCKF (Plan Step 3a) but ramp
    // the position-correction in linearly across the first ≤5 frames after
    // each "quiet" period (≥ MSCKF_QUIET_PROPAGATION steps without an MSCKF
    // call). Velocity / attitude / bias corrections are NOT damped — only
    // the world-frame δp rows (12..14 of the IMU error-state).
    int msckf_frames_since_call_{0};   // # propagateIMU calls since last MSCKF
    int msckf_damping_step_{0};        // 0..5+; 0=>0.5x, ramps to 1.0 at 5
    int msckf_huber_rejected_count_{0};
    int extr_log_skip_{0};             // throttle extrinsics angle logging (every 30 updates)

    static constexpr int MSCKF_QUIET_PROPAGATION = 5;
    static constexpr int MSCKF_DAMPING_RAMP_FRAMES = 5;
    // Huber kernel for MSCKF residuals. δ = √χ²(0.95, 2 dof) ≈ 2.4477.
    // Hard-reject (weight = 0) above 3δ.
    static constexpr double MSCKF_HUBER_DELTA = 2.4477;

    // Returns the position-correction damping factor for the current call:
    //   call 0 (first after quiet period): 0.5
    //   call 1: 0.6, 2: 0.7, 3: 0.8, 4: 0.9, ≥5: 1.0
    double computeMSCKFDampingFactor() const;

    // ── Plan Step 3b (ADR-009): SLAM feature internals ───────────────────────
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
    std::vector<SlamFeature> slam_features_;

    // 2026-05-09 telemetry fix — see takeLastMarginalizedSlamFeatureIds().
    // Drained by Tracker after every addClone(). Cleared at the head of
    // addClone() so multiple internal marginalisations within a single
    // addClone (pruneWindow path) accumulate.
    std::vector<int> last_marginalized_slam_feature_ids_;

    // Cached camera intrinsics for SLAM reprojection (see setSlamIntrinsics).
    // Default 500/500/320/240 matches the test fixture in test_slam_msckf.cpp.
    double slam_fx_{500.0};
    double slam_fy_{500.0};
    double slam_cx_{320.0};
    double slam_cy_{240.0};

    // Plan Step 6 (ADR-012): caller already holds `snapshot_mutex_`. Public
    // marginalizeOldestClone / pruneWindow / addClone delegate here so the
    // single-locking discipline doesn't deadlock against itself.
    void marginalizeOldestCloneNoLock();

    // Returns starting row/col of `slot` in the full P_. -1 on invalid slot.
    int slamFeatureCovIdxInternal(int slot) const;
    // Returns the column offset where the SLAM block starts in P_. Equal to
    // IMU_STATE_DIM + window_.size() * CLONE_DIM. Only valid when P_ is sized.
    int slamBlockStart() const;

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
};
