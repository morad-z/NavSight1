#pragma once
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "VioTypes.h"
#include "IMUPreintegrator.h"
#include "EKFState.h"
#include "FeatureManager.h"
#include "LensCorrector.h"
#include "ScaleFuser.h"
#include "ScaleEstimatorVI.h"
#include "TrackKLT.h"
#include "UpdaterZeroVelocity.h"
#include "UpdaterMSCKF.h"
#include "UpdaterSLAM.h"
// 2026-05-13: Phase 1 Step 5 — 4-DOF pose-graph back-end for retroactive
// drift correction at loop closure. See PoseGraph.h header for math.
#include "PoseGraph.h"
#include <unordered_map>
// 2026-05-12: VisualMap include reverted — see Tracker.h scaffold note below.
// #include "VisualMap.h"
#include "WindowedBA.h"
#include "GroundPlaneEstimator.h"

#include "InertialInitializer.h"
#include "LoopClosureDetector.h"
// 2026-05-16: Phase 1 Step 6 (post_v19_sprint_plan.md §205-298) — visual-only
// persistent 3D LandmarkMap, ORB-SLAM3-inspired. Phase 6.1 shipped the class +
// unit tests; Phase 6.2 (this commit) instantiates the member and populates it
// from keyframe triangulation. No EKF behavior change yet — that lands in
// Phase 6.3-6.4.
#include "LandmarkMap.h"

#include <condition_variable>

// ─────────────────────────────────────────────────────────────────────────────
// Step 8 — Cleanup status (per docs/PRODUCTION_READINESS_PLAN.md §8)
// ─────────────────────────────────────────────────────────────────────────────
// Items the plan flags for deletion are kept here as commented LEGACY blocks
// (per user request — "comment, don't delete"). Search this file for "LEGACY"
// or "DEAD CODE" to find them. Specifically retained:
//   • global_R_ / global_t_ — Tracker-owned pose (mirror of EKFState).
//   • scalar_heading_       — yaw mirror published via getHeading()/JNI.
//   • UpdaterMSCKF member   — re-enabled per ADR-008 (Step 3a); update site
//                             is Tracker.cpp:1729-1742 (processLostFeatures).
//                             Damping ramp + per-row Huber kernel applied.
//   • Mapper pipeline       — see VioEngine.h banner; was a no-op corrector.
// New code MUST read pose state from ekf_ accessors, not from these mirrors.
// ─────────────────────────────────────────────────────────────────────────────

// Intermediate frame data exported from Tracker to Mapper each frame.
struct TrackerFrame {
    cv::Mat gray;                        // Current grayscale (clone for Mapper)
    std::vector<cv::Point2f> prev_good;  // FB-checked prev points
    std::vector<cv::Point2f> next_good;  // FB-checked next points
    std::vector<cv::Point3f> points_3d;  // Triangulated 3D points
    cv::Point3f global_position;         // Global position after pose update
    bool pose_valid;
    double quality;
    int tracked;
    int frame_counter;
    int64_t timestamp_ns;
    int64_t prev_timestamp_ns;
    double estimated_scale;
    double heading;
    double fx, fy, cx, cy;              // Intrinsics used this frame
};

class Tracker {
public:
    Tracker();
    ~Tracker();

    // Core tracking: optical flow, essential matrix, rotation fusion, pose update.
    // Exports intermediate data in frame_out for Mapper.
    VisionOutput processFrame(const uint8_t* yuv_data, int width, int height,
                              int64_t timestamp_ns, IMUPreintegrator& imu,
                              TrackerFrame& frame_out);

    void setIntrinsics(double fx, double fy, double cx, double cy);
    void setCameraHeight(double height_m) { camera_height_m_ = height_m; }
    // CameraX rotationDegrees (0/90/180/270) — lets the ground-plane estimator search the analyzer
    // region that maps to the BOTTOM of the upright display (the road) on a rotated/portrait mount.
    void setAnalyzerRotation(int deg) { analyzer_rotation_deg_.store(deg, std::memory_order_relaxed); }

    // Ground-plane metric-scale eval (READ-ONLY; gpt_speed_suggestion.md Phase 0). Computed every
    // kGroundPlaneInterval frames in processFrame from the lower-image triangulated points + the known
    // camera height; never feeds the dot. Getters for the offline harness + the camera debug overlay.
    bool   isGroundPlaneValid()       const { return gp_valid_.load(std::memory_order_relaxed); }
    double getGroundPlaneScale()      const { return gp_scale_.load(std::memory_order_relaxed); }
    double getGroundPlaneConfidence() const { return gp_conf_.load(std::memory_order_relaxed); }
    double getGroundPlaneHvio()       const { return gp_hvio_.load(std::memory_order_relaxed); }
    double getGroundPlaneHorizonV()   const { return gp_horizon_v_.load(std::memory_order_relaxed); }
    int    getGroundPlaneCandidates() const { return gp_cands_.load(std::memory_order_relaxed); }
    int    getGroundPlaneInliers()    const { return gp_inliers_.load(std::memory_order_relaxed); }

    // Step 1 (Visual Production Plan) — push 8-coefficient rational
    // distortion into the owned LensCorrector. Caller is responsible for
    // gating validity (the JSON loader's RMS gate is what protects the
    // pipeline from junk coefficients).
    void setDistortion(double k1, double k2, double k3,
                       double k4, double k5, double k6,
                       double p1, double p2);

    // 2026-05-16 v27 fix: imu is now a required parameter so the post-init
    // path can re-sync R_GtoI from Madgwick's full quaternion (preserving
    // roll/pitch) instead of overwriting with pure-yaw Rz(azimuth) that
    // assumed phone-flat orientation. Caller is VioEngine which owns both.
    void setInitialHeading(double azimuth_rad, const IMUPreintegrator& imu);
    void setUserScaleCorrection(double correction);

    // Step 8c (Visual Production Plan) — rolling-shutter per-row time skew.
    // Source: Android Camera2 API — CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW
    // (API level 21+) reports the duration from first-row to last-row read-out
    // in nanoseconds. Zero disables the correction (global-shutter devices, or
    // devices that do not report the key).  Called once per captured frame from
    // the JNI layer before processFrame.
    void setRollingShutterSkew(int64_t row_skew_ns);

    // Step 8b (Visual Production Plan) — seed the EKF's body→camera rotation
    // R_bc from Android CameraCharacteristics.SENSOR_ORIENTATION.
    // R_bc_flat: 9 floats, row-major 3×3.  Called once after camera open,
    // before the first frame.  Safe to call while the VIO engine is running —
    // guarded by the Tracker's own mutex.
    void setExtrinsicsRotation(const float* R_bc_flat);

    void reset();

    // v23.11: expose FeatureManager for native-lib SLAM debug overlay
    // (last observed obs pixel per SLAM feature for residual line viz).
    const FeatureManager& getFeatureManager() const { return feature_mgr_; }

    // Handle IMU data for initialization
    void addImuData(int64_t ts, float ax, float ay, float az, float gx, float gy, float gz);

    // Step 5: Calibration & Initialization
    InertialInitializer::Status getInitStatus() const;
    void clearInitTimeout();
    void loadStoredCalibration(const cv::Mat& R_GtoI,
                               const cv::Point3f& gyro_bias,
                               const cv::Point3f& accel_bias);
    cv::Mat     getInitialRotation() const;
    cv::Point3f getCalibratedGyroBias() const;
    cv::Point3f getCalibratedAccelBias() const;

    // Depth-based scale constraint: receives MiDaS depth map at ~1Hz
    void setDepthMap(const float* depth_data, int width, int height);

    // 2026-05-19 Fix #12 — MiDaS metric-depth sampler.
    //
    // Cause: SLAM (α, β, ρ) and LandmarkMap p_world both depend on metric
    // depth, which today comes ONLY from triangulation across keyframes.
    // Triangulation fails on pure-axial motion (walk-back/forward facing
    // wall, scooter-straight) → ρ drifts → cascading bugs (LC PnP target
    // 65 m off, chi² rejects all corrections, dots don't reappear). The
    // 2026-05-19 fix9_revisit_v2 walk surfaced this. Independent depth
    // source needed.
    //
    // Change: cache the VI-Depth affine fit `(s, t)` computed by
    // applyDepthScaleConstraint (Phase 2 Step 4.2.1). When any consumer
    // needs per-pixel metric depth, sample disparity at the requested
    // pixel via bilinear interp on `depth_map_` and apply:
    //     metric_depth = 1 / (s · disparity + t)
    // Returns false when the affine fit isn't ready yet (no keyframe
    // has fired with a successful fit), depth_map_ is empty, the pixel
    // is off-grid, the MiDaS sample is < 0.01 (network's "very far /
    // unstable" band), or the resulting metric depth is outside
    // [kMinMidasDepthM, kMaxMidasDepthM] = [0.3, 30] m. The band matches
    // the existing applyDepthScaleConstraint inlier band (lines 340-341,
    // metric_z in [0.3, 12]) widened to 30 m to accommodate longer
    // sightlines on a scooter.
    //
    // Threading: locks midas_affine_mutex_ for the cached (s, t) read,
    // then depth_mutex_ for the disparity sample. Safe to call from
    // camera thread (the producer) and any consumer thread (mutex
    // serialises). Cost ≈ 1 µs per call (one mutex, four float reads,
    // five multiplies).
    //
    // Falsifier: post-Fix-#12 walk should show midas_depth_samples > 0
    // and slam_promotions_seeded_with_midas > 0 (when MiDaS depth
    // disagrees with triangulation by > 2× — the case we want MiDaS to
    // win on).
    bool sampleMidasMetricDepth(float u, float v, double& depth_m_out) const;

    // Thread-safe read-only accessors
    double getSmoothScale() const;
    double getHeading() const;
    // 2026-05-26 — #2 loop-overlay path redraw. getLoopCorrectionVersion()
    // increments each time a loop closure re-optimizes the pose graph. The UI
    // polls it and, on a change, calls getCorrectedTrajectory() to rebuild its
    // (drifted) pathHistory from the CORRECTED keyframe-node polyline — only the
    // now-node delta reaches global_t_, so the PAST path must be redrawn here for
    // the loops to visually overlay. out_xz is filled (x=East, z=North; same
    // ground frame as VioData x/z), returns the number of (x,z) pairs written
    // (<= max_pairs). Thread-safe (pose_mutex_).
    int getLoopCorrectionVersion() const {
        return loop_correction_version_.load(std::memory_order_relaxed);
    }
    int getCorrectedTrajectory(float* out_xz, int max_pairs) const;
    // 2026-05-26 — locomotion-agnostic reported speed. Returns the smoothed
    // depth-weighted metric speed (m/s) computed by updateDepthFlowSpeed from the
    // tracked feature points' MiDaS depths + the recoverPose translation — NO
    // stride model, independent of the (diverging) EKF v_G_. Works for walking,
    // scooter, bike, etc. Returns -1.0 before the first valid estimate (caller
    // shows 0/"--").
    double getFusedSpeedMps() const;
    // 2026-05-28 — cross-app-launch persistence of the MiDaS scale K (the
    // metric-per-relative depth-flow scale calibrated inside updateDepthFlowSpeed).
    // The Kotlin layer loads K from SharedPreferences on app start and pushes it
    // here via setMidasScaleK, then periodically reads via getMidasScaleK and
    // writes back to prefs. Without this, every cold app start re-pays the K
    // calibration tax — which depends on essential-matrix verification passing,
    // which fails on slow walks with close scenes → on a cold-start walk K stays
    // at -1 forever, looming bails (it requires K>0), and the UI shows 0.
    void setMidasScaleK(double k);
    double getMidasScaleK() const;
    // 2026-05-31 (map-as-sensor HEADING leg) — the Kotlin matcher pushes the matched ROAD bearing
    // (deg, 0=N CW) when confidently railed on a straight road in FREE_ROAD; processFrame nudges the
    // Madgwick heading toward it (gated) so the trajectory stops drifting off the road. A sentinel
    // outside [-180,360] = no correction this tick. Doc ROAD_FOLLOWER_MATCHER_DESIGN_2026_05_31.md.
    void setRoadHeadingHint(double bearing_deg);
    // Map-as-sensor POSITION leg: push the world-frame (east,north) error = matched-ball − raw-VIO, in
    // metres. The camera thread bleeds its cross-track component into global_t_ (see members above).
    void setMapPositionCorrection(double d_east_m, double d_north_m);
    // Runtime enable for the POSITION leg (default-OFF; heading leg is separate + ON). Lets the device
    // validate the heading leg alone first, then turn the position feedback on without a rebuild.
    void setMapPositionEnabled(bool enabled);
    // 2026-05-31 — REPLAY-ONLY experiment flag (docs/MSCKF_PG_WIRING_VERDICT_
    // 2026_05_31.md §4). Default OFF: the device/JNI path never calls this, so
    // the shipped build is byte-identical. When ON, processFrame STOPS calling
    // ekf_.setPosition(global_t_) before propagateIMU (Tracker.cpp:~1862), so
    // p_G_ propagates autonomously and the MSCKF/ZUPT updates are no longer
    // overwritten every frame. Lets the replay A/B measure whether MSCKF+ZUPT
    // geometrically BOUND p_G_ (end-displacement ≈ loop GPS ~12m) or run away
    // like v31 (50-100m+). Heading is untouched by construction — setPosition
    // writes only p_G_ (EKFState.cpp:2314-2317), never R_GtoI_.
    void setAutonomousPgExperiment(bool on) { autonomous_pg_ = on; }
    bool isInitialized() const { return initialized_; }
    const EKFState* getEKF() const { return &ekf_; }
    // Step 2.4: variance of the last keyframe-derived visual yaw measurement
    // (rad²). -1.0 if not yet computed. Consumed by Step 6 ESKF update.
    double getLastVisualYawVariance() const;
    // Step 3 Observer B: variance of the last MiDaS depth-scale measurement,
    // expressed as variance of the scale ratio (dimensionless²). -1.0 if not
    // yet computed. Consumed by EKF scale-update fusion (Step 3 Fusion).
    double getLastDepthScaleVariance() const;

    // Step 6: Horizontal-plane position covariance (m²) extracted from EKFState's
    // 15-DOF IMU error-state block at indices 12..14 (position component). Returns
    // the 2x2 (x, z) sub-block as [σ_xx, σ_xz, σ_zz]. Returns false if EKF is not
    // yet fully initialized; out-array is then filled with zeros.
    bool getPositionCovarianceXZ(double out[3]) const;

    // ── Map-matching Step B* (MAP_MATCHING_PLAN.md §8M) — VIO→lat/lng ─────────
    //
    // Read-only projection of the user-facing position dot to geographic
    // (lat,lng) for the Kotlin map-matching layer. Two methods, both additive
    // and read-only on the EKF (the ONLY engine touch the osm-migration branch
    // makes; §0.0 rule 2). One bootstrap GPS fix sets the SessionAnchor
    // (allowed by ADR-004); thereafter every position is derived purely from
    // VIO — no raw GPS sample ever feeds the EKF or the matcher.
    //
    // SOURCE DECISION (binding): the projection reads the user-facing
    // `global_t_` (the dot the user sees), NOT `ekf_.getPosition()` / p_G_.
    // The plan body text said getPosition(), but project_visual_audit_2026_05_30
    // established the displayed dot IS global_t_ and that p_G_ over-reads
    // ~1.4–2× — so the matcher must consume global_t_ to snap what's on screen.
    struct VioLla {
        double  lat_rad{0.0};    // VIO-projected latitude  (radians)
        double  lng_rad{0.0};    // VIO-projected longitude (radians)
        int64_t t_ns{0};         // EKF/Tracker state timestamp
        double  var_xy_m2{0.0};  // EKF horizontal position-covariance trace (East+North), m²
        bool    valid{false};    // false => no SessionAnchor yet; matcher must silent-disable
    };

    // Set the one-shot session anchor from the first valid bootstrap GPS fix.
    // Idempotent: a second call is logged + ignored (re-anchoring mid-session
    // would teleport the whole track). Thread: called from the JNI/Kotlin side.
    void setSessionAnchor(double lat_deg, double lng_deg, int64_t t_ns);

    // Project the current user-facing position to (lat,lng). Returns valid=false
    // (and increments vio_lla_unanchored_reads) until a SessionAnchor exists.
    VioLla current_vio_lla() const;

    // ── Plan Step 7 (ADR-013): same-session loop closure (DBoW2) ──────────────
    //
    // Push the absolute on-device path of the ORB DBoW2 vocabulary into the
    // Tracker's owned LoopClosureDetector. Called from the JNI startup path
    // after the engine is created (the Android side copies
    // assets/ORBvoc.bin → <filesDir>/ORBvoc.bin once and passes the absolute
    // path here — AssetManager paths are not real filesystem paths and
    // cv::FileStorage / DBoW2 readers need a path they can fopen).
    //
    // Returns true if the vocabulary loaded and the detector is now ready to
    // accept keyframes / queries. False on missing file, parse error, or
    // dimension mismatch — the loop-closure worker thread will not run on
    // false (it gates on `loop_closure_.isReady()` each query tick), so the
    // rest of the pipeline keeps functioning unchanged.
    bool loadLoopClosureVocabulary(const std::string& vocab_path);

    // Read accessor used by the JNI shim — exposed because the loop-closure
    // detector is otherwise a private member of Tracker. The shim only
    // calls `loadVocabulary` on the returned reference; the worker thread
    // and addKeyframe path stay strictly internal.
    LoopClosureDetector& getLoopClosureDetector() { return loop_closure_; }

    // 2026-05-16 Phase 1 Step 6.4 — IDs of landmarks the EKF accepted on the
    // most recent keyframe. Consumed by Phase 6.5 (native-lib.cpp overlay) to
    // colour matched landmarks orange. Snapshot semantics: returns a copy
    // under last_observed_mutex_, so the caller's vector is decoupled from
    // the producer thread. Empty until the first keyframe-wide EKF update
    // fires; cleared on reset().
    std::vector<int> getLastObservedLandmarkIds() const {
        std::lock_guard<std::mutex> lk(last_observed_mutex_);
        return last_observed_landmark_ids_;
    }

    // 2026-05-19 — Orange-dot anchor fix #3. Per-landmark matched pixel from
    // the last keyframe's local-map tracking pass. Render path (native-lib
    // getLandmarkSnapshot) uses this so OBSERVED orange dots are drawn at
    // their actual image-feature pixel instead of at the projection of a
    // possibly-stale stored p_world. The residual "red line" then collapses
    // to zero for observed dots — the user-visible falsifier for the bug.
    // Returns false if `landmark_id` was not in last_observed_landmark_ids_
    // (i.e. not matched this frame). Same lock as getLastObservedLandmarkIds.
    bool getLastObservedLandmarkPixel(int landmark_id,
                                       float& out_u, float& out_v) const {
        std::lock_guard<std::mutex> lk(last_observed_mutex_);
        for (size_t i = 0; i < last_observed_landmark_ids_.size(); ++i) {
            if (last_observed_landmark_ids_[i] == landmark_id) {
                if (i < last_observed_landmark_pixels_.size()) {
                    out_u = last_observed_landmark_pixels_[i].x;
                    out_v = last_observed_landmark_pixels_[i].y;
                    return true;
                }
                return false;
            }
        }
        return false;
    }

    // 2026-05-16 Phase 1 Step 6.5 — read-only access to the persistent
    // LandmarkMap for the JNI getLandmarkSnapshot overlay path (native-lib.cpp).
    // LandmarkMap is internally mutex-protected so the caller may invoke
    // getLandmarksInRadius / getLandmark from any thread.
    const navsight::LandmarkMap& getLandmarkMap() const { return landmark_map_; }

private:
    double estimateScaleFromSteps(double vision_disp, int64_t dt_ns,
                                  IMUPreintegrator& imu);

    // ── Plan Step 4 (ADR-010): ORB descriptor relocalization ──────────────
    // Runs when low_inlier_streak_ trips RELOC_TRIGGER_FRAMES. Extracts ORB
    // on the current gray frame, brute-force matches (Hamming + knnMatch
    // k=2 + Lowe ratio) against the most recent RELOC_RECENT_KFS keyframe
    // descriptor records, runs cv::findEssentialMat with RANSAC, and
    // accepts the best-inlier keyframe if ≥ RELOC_MIN_INLIERS survive.
    // On accept: re-adopts feature ids from the matched keyframe by
    // attaching them to the nearest current KLT track within
    // RELOC_ID_REATTACH_RADIUS pixels. Returns true on accept.
    //
    // Caller must hold no FeatureManager-related locks for this call (the
    // method only reads `feature_mgr_.getKeyframeDescriptors()` and writes
    // back into `feature_ids_`, which is owned by Tracker exclusively).
    bool tryRelocalizeWithORB(const cv::Mat& gray,
                              const std::vector<cv::Point2f>& current_pts);

    // ── Plan Step 5: motion-blur detector ─────────────────────────────────
    // Variance of Laplacian on the centre 50%×50% crop of `gray`. Below
    // BLUR_VAR_THRESH the frame is too blurry to support reliable visual
    // measurement updates; the EKF prediction (propagateIMU + ZUPT) still
    // runs, but SLAM-feature updates, the MSCKF processLostFeatures call,
    // and the ORB relocalization trigger are skipped for that frame so the
    // filter does not consume high-noise residuals.
    //
    // Returns the variance (so callers can log it). A non-positive return
    // value indicates an empty / undersized input — treat that as
    // "not blurry" because it means we cannot make a measurement either way.
    double measureBlur(const cv::Mat& gray) const;

    mutable std::mutex mutex_;
    mutable std::mutex pose_mutex_;

    InertialInitializer initializer_;
    bool initialized_{false};
    
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    int64_t prev_timestamp_ns_{0};

    // Reusable buffers (pre-reserved to avoid per-frame allocation)
    cv::Mat gray_buf_;
    std::vector<cv::Point2f> current_prev_pts_buf_;
    std::vector<cv::Point2f> next_pts_buf_;
    std::vector<cv::Point2f> prev_good_buf_;
    std::vector<cv::Point2f> next_good_buf_;
    std::vector<cv::Point2f> new_pts_buf_;

    // ── Step 8 Cleanup status (kept, NOT deleted) ───────────────────────────
    // Fix B (2026-05-16 audit Finding 2.1): EKFState is the SSOT for rotation
    // and position post-init. global_R_ and global_t_ are retained ONLY as
    // bootstrap seeds (pre-EKF-init) and legacy mirrors (commented-out notes
    // below document which role each site serves). Do not read global_R_ in
    // new code post-init — use ekf_.getRotation() directly.
    //
    // global_R_ bootstrap write sites still active (pre-init only):
    //   Tracker.cpp ctor ~L33, reset() ~L389, loadStoredCalibration ~L603,626,
    //   mag one-shot ~L746, pending_init_heading_ apply ~L762,
    //   setInitialHeading post-init re-sync ~L345 (reads Madgwick, not EKF).
    // global_R_ post-init read sites migrated to ekf_.getRotation():
    //   ScaleEstimatorVI kp.R_w_b ~L1726 (now ekf_.getRotation() — Fix B)
    //   §9.0 mirror write ~L1845 (global_R_ = ekf_.getRotation() — still live)
    //   addClone fallback ~L2033 (already ekf_.isFullInitialized() ? ekf_ : ...)
    //   out.R output ~L3365  (already ekf_.isFullInitialized() ? ekf_ : ...)
    // When ALL pre-init bootstrap sites are removed (i.e. EKF init fires
    // unconditionally from Madgwick), comment out the declaration below.
    cv::Mat global_R_;   // 3x3 CV_64F — LEGACY mirror / bootstrap seed
                         // (2026-05-16 Fix B: post-init reads migrated to
                         //  ekf_.getRotation(); pre-init bootstrap writes kept)
    cv::Mat global_t_;   // 3x1 CV_64F — LEGACY mirror; Fix A makes EKF
                         // authoritative for position. global_t_ is now a
                         // read mirror refreshed by §11.9 in processFrame.
                         // (2026-05-16 Fix A)

    // Map-matching Step B* (MAP_MATCHING_PLAN.md §8M) — one-shot session anchor.
    // Set exactly once from a bootstrap GPS fix (ADR-004), never mutated after.
    // Read-only consumer of global_t_; does not affect any VIO state.
    struct SessionAnchor {
        double  anchor_lat_rad{0.0};
        double  anchor_lng_rad{0.0};
        int64_t anchor_t_ns{0};
        bool    valid{false};
    };
    SessionAnchor session_anchor_;

    double fx_{0.}, fy_{0.}, cx_{0.}, cy_{0.};
    // Step 8c: row read-out time skew from Camera2 SENSOR_ROLLING_SHUTTER_SKEW.
    // Units: nanoseconds per image_height rows.  0 = correction disabled.
    // Reset to 0 in reset() so a new VIO session starts without stale skew.
    int64_t rolling_shutter_row_skew_ns_{0};
    int scale_obs_count_{0};
    double user_scale_correction_{1.0};
    // Scale bootstrap: collect first N observations and take median
    static constexpr int SCALE_BOOTSTRAP_COUNT = 15;
    std::vector<double> scale_bootstrap_buf_;
    cv::Mat accel_bias_;
    int accel_bias_count_{0};
    // gyro_bias_ removed — unified: use IMUPreintegrator::getGyroBias() only

    std::vector<cv::Point3f> points_3d_current_;
    std::vector<int> feature_ages_;  // Phase 2: track age per feature (frames survived)
    int frame_counter_{0};

    // Phase 1 Step 2c verification: rolling-window frame-interval tracking.
    // Used to log CAM_FPS so we can confirm the Camera2Interop fps lock
    // ([30, 30] AE_TARGET_FPS_RANGE) is actually taking effect on device.
    // Per-frame Δt accumulates here; once `kCamFpsWindow` frames have been
    // sampled we emit a log line and reset. ~1 line/sec at 30 fps.
    int64_t prev_camera_frame_ts_ns_{0};
    double  cam_dt_sum_ms_{0.0};
    int     cam_dt_count_{0};
    static constexpr int kCamFpsWindow = 30;

    // Session-wide running statistics over per-window fps measurements.
    // Welford's online algorithm — single pass, numerically stable, no
    // per-frame accumulator drift, no division until query. Values are
    // pushed to EventCounters' atomic milli-Hz fields after each window
    // closes so the sim JSON's event_summary reflects the latest mean+stdev
    // without needing logcat retention.
    int     cam_fps_running_count_{0};
    double  cam_fps_running_mean_hz_{0.0};
    double  cam_fps_running_m2_{0.0};   // sum of squared deviations

    // Phase 1 Step 3: drift-since-last-loop-closure tracker for the
    // chi² variance budget in `consumeLoopClosureMatchIfReady`.
    //
    // Problem this solves: v18's `setPosition(global_t_)` sync per frame
    // overrides EKF p_G but does NOT touch P_pp, so MSCKF visual updates
    // collapse the EKF horizontal-position covariance to ~3 cm even
    // though `global_t_` has accumulated 5–15 m of real drift since the
    // last loop closure. The chi² gate then rejects almost everything
    // (m² hovers at the threshold instead of either accepting cleanly
    // or rejecting genuine outliers).
    //
    // Fix: track the integrated path length since the last accepted
    // loop closure. The variance budget for chi² becomes
    //
    //     σ²_p_total = σ²_p_pnp + max(σ²_p_ekf, σ²_p_drift)
    //
    // where σ²_p_drift = (LOOP_CLOSURE_DRIFT_RATE × path_since_last_lc_m_)².
    // This grows monotonically with walking distance and resets to 0 on
    // every accepted loop closure (`ok=1` in consumeLoopClosureMatchIfReady).
    // Incremented after every per-frame update to `global_t_`. Touched
    // only on the camera thread (the same thread that owns global_t_).
    double path_since_last_lc_m_{0.0};

    // 2026-05-09 v13: count of consecutive frames where the ZUPT detector
    // flagged the body stationary. Used to gate the stationary specific-force
    // measurement update (EKFState::updateStationaryAccel) so it only fires
    // on SUSTAINED stationarity (e.g. user paused at a corner) — never on
    // walking heel-strike micro-stationary frames that sneaks past a
    // single-frame ZUPT trigger. Cleared whenever is_static is false.
    int consecutive_static_frames_{0};

    // Phase 5: Magnetometer heading used ONCE at startup only
    bool heading_initialized_{false};

    // Fix B (2026-05-16 audit Finding 2.1): scalar_heading_ is a per-frame
    // cache of imu.getHeading() (Madgwick CW-positive nav yaw, Z-up ENU).
    // It is WRITTEN each frame in §9.0 from imu.getHeading() — never from
    // ekf_.getYaw() — because EKF yaw under-rotates fast turns (V-shape bug
    // fixed 2026-05-03). Retained as a member because Tracker::getHeading()
    // (JNI-called, no imu reference) and LC keyframe stores need the heading
    // value AT a specific frame captured under pose_mutex_. When JNI migrates
    // to a direct IMUPreintegrator query, this member can be removed.
    // DO NOT derive heading from ekf_.getWorldHeadingRad() for position
    // update math — use imu.getHeading() (same value, correct fast-turn
    // behaviour).
    double scalar_heading_{0.0};
    // Cumulative odometric path length (metres). Incremented in both the
    // visual update path and the PDR step so the loop-closure dynamic sigma
    // (LOOP_CLOSURE_DRIFT_RATE * total_path_m_) tracks real walked distance.
    double total_path_m_{0.0};
    // Pending init heading from setInitialHeading() — applied as a bootstrap
    // seed for ekf_.initializeFull on the first frame.
    double pending_init_heading_{0.0};
    bool   pending_init_heading_set_{false};
    // Fix D (2026-05-16 audit): post-init setInitialHeading azimuth queued
    // for retry on the next IMU sample when Madgwick is not yet ready.
    // Set inside setInitialHeading() under pose_mutex_; consumed by
    // processFrame §3.1-equiv retry block before §4 first-frame detection.
    // Cause: silent "leaving EKF as-is" when Madgwick not ready at
    //   setInitialHeading post-init means the azimuth update is lost.
    // Change: queue it here; processFrame retries on the next camera frame.
    // Falsifier: pending_post_init_heading_set_ == false after any walk
    //   (always consumed within 1-2 frames).
    double pending_post_init_azimuth_{0.0};
    bool   pending_post_init_heading_set_{false};
    double filtered_yaw_rate_{0.0};  // Low-pass filtered yaw rate (removes step oscillation)

    // Step 2.3: heading_fej_ deleted — Madgwick is the heading reference.
    // FEJ remains for position only.

    // Step 2.4: most recent keyframe visual yaw variance (rad²); -1 if unset.
    double last_visual_yaw_variance_{-1.0};

    // Step 3 Observer B: variance of the last MiDaS depth-scale ratio
    // (dimensionless²). -1.0 if no valid measurement yet. Computed via
    // MAD * 1.4826 over sampled depth ratios.
    double last_depth_scale_variance_{-1.0};

    // Step 3 Fusion: 1-D Kalman fuser for metric scale. All observers
    // (PDR, MiDaS, VI) feed (z, variance) here; smooth_scale_ is mirrored from
    // scale_fuser_.scale() after each update. Initial variance large so the
    // first confident observer pulls the estimate strongly.
    ScaleFuser scale_fuser_{0.20, 1.0};
    int64_t last_scale_predict_ns_{0};

    // Step 3 Observer C wiring: closed-form Hesch/Martinelli VI scale.
    // Per-frame keyframe pairs (one pair = one frame interval) feed the
    // estimator; every OBSERVER_C_SOLVE_INTERVAL pairs we run solve() and,
    // if the residuals are healthy, push (s, var) into scale_fuser_.
    // Unit-norm t_vo from recoverPose means the recovered s is the metric
    // distance per frame, directly comparable to smooth_scale_.
    ScaleEstimatorVI scale_estimator_vi_;
    int observer_c_pair_count_{0};
    static constexpr int OBSERVER_C_SOLVE_INTERVAL = 10;

    // Phase 6: Time-offset cross-correlation warmup
    struct TdSample { double flow_rate; double gyro_rate; int64_t ts_ns; };
    std::vector<TdSample> td_warmup_buf_;
    bool td_warmup_done_{false};
    static constexpr int TD_WARMUP_FRAMES = 60;  // ~2 seconds at 30fps

    // Step speed interpolation: maintain last known speed for short gaps
    double last_step_speed_{0.0};
    int64_t last_step_speed_ns_{0};

    // 2026-05-26 — depth-weighted metric speed (Tracker::updateDepthFlowSpeed):
    // the metric scale of the recoverPose translation, recovered from the tracked
    // feature points' MiDaS depths → reported speed |v|/dt, EMA-smoothed (τ≈0.5s).
    // Atomic: written on the camera thread, read internally by the depth-flow
    // K-calibration and the trajectory wire-up. -1.0 = no estimate yet.
    // Independent of the (diverging) EKF v_G_ and of the global appliedScale.
    std::atomic<double> depth_flow_speed_mps_{-1.0};

    // 2026-06-02 — ground-plane optical flow speed (Phase 1, gpt_speed_suggestion.md).
    // Uses known camera height + de-rotated vertical flow of road pixels to recover
    // metric speed. Independent of KLT triangulation or MiDaS.
    std::atomic<double> ground_flow_speed_mps_{-1.0};

    // 2026-05-28 — trajectory-applied speed (m/s). What the trajectory accumulator
    // is ACTUALLY using to advance global_t_ this frame, regardless of source:
    //   - depth_flow_speed_mps_ when depth-flow / looming is valid;
    //   - legacy appliedScale * |t_vo| / dt when essential matrix passed but
    //     depth-flow hadn't calibrated K yet;
    //   - PDR step speed when the camera path is degenerate (slow walk +
    //     close scene → low inlier ratio → verification_ok=false);
    //   - 0.0 during is_static / rotation_dominated frames.
    // Read by getFusedSpeedMps for the UI speedometer so the displayed speed
    // always tracks the actual trajectory motion the user sees on the map. The
    // depth_flow_speed_mps_ atomic stays as the K-calibrated estimate used by
    // the depth-flow internals; trajectory_speed_mps_ is the UI-facing view.
    std::atomic<double> trajectory_speed_mps_{0.0};

    // 2026-06-02 — read-only access to the ground-plane speed estimate (m/s).
    double getGroundFlowSpeedMps() const {
        return ground_flow_speed_mps_.load(std::memory_order_relaxed);
    }

    // 2026-05-27 — expansion-rate metric speed (Tracker::updateExpansionSpeed):
    // implements "Vz = expansion_rate * Z_rel * K" robustly via Median + WLS.
    // Atomic: written camera-thread, read by getFusedSpeedMps (UI).
    std::atomic<double> expansion_speed_mps_{-1.0};

    // Camera-thread helper computing depth_flow_speed_mps_ from the matched KLT
    // pairs + MiDaS depth + recoverPose (R_vo, t_vo). Called from processFrame.
    // 2026-05-30 (Scale fix Steps 2/5): imu drives classifyGait at the top;
    // gyro_norm (rad/s, from the call site) drives the turn-suppression gate on
    // the K calibration so turns don't inflate K_walk toward the UTURN k_obs.
    void updateDepthFlowSpeed(const std::vector<cv::Point2f>& prev_ud,
                              const std::vector<cv::Point2f>& next_ud,
                              const cv::Mat& R_vo, const cv::Mat& t_vo,
                              double dt_s,
                              const IMUPreintegrator& imu, double gyro_norm);

    // 2026-05-27 expansion-rate (looming / flow-divergence) variant: per-point
    // Vz = (radial flow / radius / dt) · Z_rel · K. Best-conditioned for forward
    // motion (where the essential matrix degenerates). Requires the gyro rotation
    // vector in CAMERA frame (omega_cam · dt) to de-rotate the flow (Heeger-Jepson
    // 1992) — without it, head turns leak into false speed. The result is FUSED
    // into depth_flow_speed_mps_ by the forward-motion fraction so the UI shows a
    // smooth blend (looming-dominant when forward, recoverPose-dominant sideways).
    // 2026-05-30 (Scale fix Steps 2/5): imu drives classifyGait at the top;
    // gyro_norm (rad/s) drives the turn-suppression gate on the looming K calib.
    void updateExpansionSpeed(const std::vector<cv::Point2f>& prev_ud,
                              const std::vector<cv::Point2f>& next_ud,
                              double dt_s,
                              const cv::Vec3d& gyro_rot_cam,
                              const IMUPreintegrator& imu, double gyro_norm);

    // 2026-06-02 — Ground-plane optical flow speed estimator (exact pitch derivation).
    // Estimates forward speed by de-rotating flow of road pixels (lower image) and
    // projecting to the ground plane via known camera height and EKF attitude.
    void updateGroundFlowSpeed(const std::vector<cv::Point2f>& prev_ud,
                               const std::vector<cv::Point2f>& next_ud,
                               double dt_s,
                               const cv::Vec3d& gyro_rot_cam);

    // 2026-05-30 (Scale fix Steps 1-3, docs/SCALE_FIX_DESIGN_2026_05_30.md) —
    // PER-GAIT K. A single shared K cannot serve walk/run/vehicle: the measured
    // accel-K observations are WALK k_obs≈1340, RUN≈831 (ratio 0.62), UTURN≈1652.
    // A walk-stop accel spike pushed the shared K up (~2400) and then over-scaled
    // the following run. Fix: keep midas_scale_K_/expansion_scale_K_ as the ACTIVE
    // mode's K (used everywhere exactly as before — no rename) and store a per-mode
    // copy in k_slots_. classifyGait() picks the mode each frame; onModeSwitch()
    // SAVES the active K into the old slot and LOADS the new slot (seeding a virgin
    // RUN slot at walk*0.62, VEHICLE at the current K). All of this runs on the
    // camera thread (single writer/reader), so active_mode_/mode_*_frames_/k_slots_
    // need no atomics; only midas_scale_K_/expansion_scale_K_ stay atomic (UI reads).
    enum class GaitMode { WALK = 0, RUN = 1, VEHICLE = 2 };
    struct KSlot { double df{-1.0}; double loom{-1.0}; };  // camera-thread only
    KSlot k_slots_[3];
    GaitMode active_mode_{GaitMode::WALK};
    int mode_hold_frames_{0};               // consecutive frames a NEW candidate held
    int mode_switch_fast_alpha_frames_{0};  // remaining frames of fast-converge EMA
    // classifyGait runs in BOTH updateExpansionSpeed and updateDepthFlowSpeed within
    // the same processFrame; this guards the hysteresis counter so it advances at
    // most once per frame (keeps kModeHoldFrames a true ~0.5 s regardless of how many
    // speed paths fired). -1 = never classified.
    long long last_gait_frame_{-1};
    // Same once-per-frame guard for the fast-converge EMA counter: both calib paths
    // (depth-flow + looming) decrement mode_switch_fast_alpha_frames_ on a
    // verification_ok frame; without this they halve the kFastConvergeFrames window
    // on frames where both fire. -1 = unset.
    long long last_fast_alpha_frame_{-1};
    // 2026-05-31 — one-shot COLD fast-converge arm. A pure walk on a device with a
    // STALE PERSISTED K (setMidasScaleK leaves cur_k>0) never hits onModeSwitch, so it
    // crawls at kNormalAlpha from the seed and cannot reach the live k_obs in a short
    // walk (val_2026_05_31_pm: 13m walk read 0.68x). Arm the EXISTING fast-converge
    // window once per session at the first accepted calib so the persisted seed is
    // treated like a slot seed, not a calibrated value. Reset to false in reset().
    bool cold_fast_converge_armed_{false};
    // 15 frames @ 30 Hz ≈ 0.5 s hysteresis before any mode switch (debounces a
    // misclassified frame from swapping K slots).
    static constexpr int    kModeHoldFrames     = 15;
    // After a switch, run the EMA fast for this many frames so the new slot
    // converges from its seed instead of lagging at α=0.05 over a whole leg.
    static constexpr int    kFastConvergeFrames = 10;
    static constexpr double kFastAlpha          = 0.3;   // fast post-switch EMA
    static constexpr double kNormalAlpha        = 0.05;  // steady-state EMA (== legacy 0.05)
    // RUN seed = WALK*0.62 from the measured k_obs ratio (RUN 831 / WALK 1340).
    static constexpr double kRunWalkSeedRatio   = 0.62;

    // Camera-thread gait classification + slot swap (Scale fix Steps 2-3).
    GaitMode classifyGait(const IMUPreintegrator& imu, double accel_speed);
    void onModeSwitch(GaitMode old_mode, GaitMode new_mode);

    // 2026-05-26 — MiDaS relative->metric scale K, calibrated from the accelerometer
    // during the clean window right after a ZUPT stop. This BREAKS the circular
    // dependency where MiDaS metric depth inherited the weak VIO scale (the affine
    // fit calibrated to scale_fuser_). Published basis: VINS-Mono / VI-Depth (Wofk
    // 2023) / DynaDepth. speed = K * median(flow x relative_depth). -1 until first
    // calibration. Atomic: written camera-thread, read by getFusedSpeedMps (UI).
    // 2026-05-30: now holds the ACTIVE GaitMode's depth-flow K (see k_slots_).
    std::atomic<double> midas_scale_K_{-1.0};
    // 2026-05-31 map-as-sensor HEADING leg — road bearing hint (deg, 0=N CW) pushed from the Kotlin
    // matcher when railed on a straight road; -1000 sentinel = none. frames_since_road_hint_ ages it
    // (camera thread increments, the JNI setter zeroes) so a stale hint (matcher paused) stops
    // correcting. The nudge is gated (see kRoad* below) so it can only un-drift, never re-aim.
    std::atomic<double> road_heading_hint_deg_{-1000.0};
    std::atomic<int>    frames_since_road_hint_{1000000};
    bool road_heading_correction_enabled_{true};        // heading unlocked by owner 2026-05-31; ON for test
    static constexpr double kRoadSyncStrength    = 0.10;      // fractional nudge (mirrors kBug5SyncStrength)
    // NOT a tuning knob — a PHYSICAL boundary: after the nearer-road-direction pick (Tracker.cpp §9.0b)
    // |resid| ≤ 90°; a road exactly perpendicular (90°) to your heading is one you are CROSSING, not
    // driving along, so it is rejected. The real wrong-road protection is upstream: the Kotlin side only
    // pushes a road bearing when the matcher is RAIL-LOCKED on that road (its geometry fits the
    // trajectory), so a crossing/parallel road you are not on never reaches this nudge. No 35°/80° magic
    // alignment threshold — the target is each road's OWN bearing, trusted by road identity, not degrees.
    static constexpr double kRoadMaxResidualRad  = 1.5707963267948966;  // π/2 = 90° = perpendicular = crossing
    static constexpr int    kRoadHintMaxAgeFrames = 45;       // ~1.5 s @ 30 fps before a hint is stale
    // 2026-05-31 map-as-sensor POSITION leg (owner: "fix the drifting vio based on the map matcher so
    // it doesn't keep drifting") — the Kotlin matcher pushes the world-frame error vector (ball-on-rail
    // position − raw VIO), in metres (east,north), via setMapPositionCorrection. Each frame we bleed a
    // small fraction of its CROSS-TRACK component (perpendicular to heading) into global_t_, pulling the
    // dot back onto the road. Along-track is IGNORED (that is the speed/scale lever; the map must not
    // fight it). Same delta-injection channel loop-closure uses (NOT a re-integrator). Heavily gated:
    // confident+fresh, magnetometer not fusing, plausible error band, capped per frame — so a wrong-road
    // lock cannot run away (ADR-004) and it can only un-drift, never teleport.
    std::atomic<double> map_pos_err_east_{0.0};
    std::atomic<double> map_pos_err_north_{0.0};
    std::atomic<int>    frames_since_map_pos_{1000000};
    // ON (2026-06-02): the ball-on-rail arrow tracked the road correctly on-device (owner confirmed),
    // which validates the heading leg + road-ID — so the position leg is enabled to pull the raw VIO
    // (orange) trail onto the road too (owner: "the orange trail isn't fixed to be on the road, it still
    // drifts"). The earlier wrong-road runaway risk (ADR-004) is mitigated HARD on the Kotlin side
    // (push only on a sustained high-confidence FREE_ROAD straight rail, streak≥3) plus kMapPosMaxErrM=15
    // and decrement-on-apply convergence below. Toggle off via setMapPositionEnabled / NativeBridge if it
    // ever drags the dot — but it cannot move the arrow (display ball is separate), only the orange.
    bool map_pos_correction_enabled_{true};
    static constexpr double kMapPosSyncStrength = 0.02;      // per-frame fraction of the residual lateral error
    static constexpr double kMapPosMaxStepM     = 0.05;      // cap per frame (m) → ≤ ~1 m / 0.67 s pull
    static constexpr double kMapPosMinErrM      = 1.5;       // ignore sub-noise lateral error
    static constexpr double kMapPosMaxErrM      = 15.0;      // urban parallel-road bound (was 50; review HIGH)
    static constexpr int    kMapPosMaxAgeFrames = 20;        // ~0.67 s — covers one missed 500 ms push (was 45)
    // Ground-plane metric-scale eval (read-only). Estimator + cached latest result (atomics so the
    // camera-thread getters are lock-free). Runs every kGroundPlaneInterval frames in processFrame.
    GroundPlaneEstimator ground_plane_estimator_;
    std::atomic<bool>   gp_valid_{false};
    std::atomic<double> gp_scale_{0.0};
    std::atomic<double> gp_conf_{0.0};
    std::atomic<double> gp_hvio_{0.0};
    std::atomic<double> gp_horizon_v_{-1.0};
    std::atomic<int>    gp_cands_{0};
    std::atomic<int>    gp_inliers_{0};
    std::atomic<int>    analyzer_rotation_deg_{0};   // CameraX rotationDegrees (road-region orientation)
    static constexpr int kGroundPlaneInterval = 3;
    // World-frame velocity from accel (g*dt + R_GtoI^T*deltaV, same increment as
    // EKFState.cpp:295), integrated from a ZUPT stop and re-zeroed at the next stop.
    // Trustworthy ONLY in the short post-stop window (drift grows after) -> used to
    // calibrate midas_scale_K_ there. Camera-thread only (no atomic needed).
    cv::Vec3d accel_vel_w_{0.0, 0.0, 0.0};
    double secs_since_zupt_{-1.0};  // seconds since last ZUPT stop; -1 until first stop
    // 2026-05-27 — low-pass of world linear acceleration = the slow gravity-leak +
    // accel-bias residual that makes the raw integral ramp (12 m/s on a walk). We
    // subtract it (high-pass) before integrating accel_vel_w_, killing the drift at
    // its source. Updated every frame (converges to the residual incl. during stops).
    cv::Vec3d accel_drift_lp_{0.0, 0.0, 0.0};
    // Stub for the inline setCameraHeight setter at the top of the class (forward-cam
    // ground-plane scale path, not currently used by speed estimator). Unused until wired.
    double camera_height_m_{0.0};
    // 2026-05-27 — post-ZUPT accumulated path lengths for ROBUST scale calibration:
    // K = accel_dist / visual_rel_dist (ratio of ACCUMULATED distances since the stop).
    // Far more stable than the per-frame ratio, which swung ~2.4x because MiDaS
    // renormalises its relative scale every frame (research Rec 3). Reset at each stop.
    double accel_dist_accum_{0.0};       // metres: integral |accel velocity| dt since stop
    double visual_rel_dist_accum_{0.0};  // relative units (DEPTH-FLOW disp_rel): for K_df
    // 2026-05-29 — SEPARATE looming accumulator + K. The depth-flow path calibrates
    // midas_scale_K_ from visual_rel_dist_accum_ (recoverPose disp_rel) and consumes it
    // — self-consistent, run-proven (K=519 read the run right). Looming uses a DIFFERENT
    // relative measure (vz_rel = tau*Z_rel), so it must calibrate + consume its OWN K to
    // avoid the cross-basis mismatch a single shared K would create off forward motion.
    // Both reduce to v/K for pure-forward motion (the calibration regime) so they agree
    // there; keeping them separate removes all doubt and preserves the run path exactly.
    // expansion_scale_K_ is seeded from the same persisted value as midas_scale_K_ on
    // startup (setMidasScaleK) so cold-start walks aren't penalised. Reset at each stop.
    double visual_rel_dist_loom_{0.0};   // relative units (LOOMING vz_rel*dt): for K_loom
    std::atomic<double> expansion_scale_K_{-1.0};  // looming relative->metric scale (K_loom)
    // 2026-05-29 (Step B) — VINS-Mono (ScaleEstimatorVI / Hesch-Martinelli) metric
    // SPEED estimate (m/s). On solve success, vi_speed = s / mean_pair_dt, where s is
    // the recovered metric magnitude of the per-pair visual translation. This is an
    // unbiased joint IMU+visual+gravity metric reference (unlike the windowed accel-K
    // which under-reads steady motion) — used to calibrate the depth-flow K when the
    // recoverPose translation is well-conditioned (fast motion: scooter / run). On
    // slow walks recoverPose is degenerate so the solve is rejected and this stays
    // stale; the accel-K path then drives (with its known ~1.5x bias → user does a
    // one-time known-distance walk calibration). -1 until first valid solve.
    std::atomic<double>    vi_metric_speed_mps_{-1.0};
    std::atomic<long long> vi_speed_ts_ns_{0};      // timestamp of last valid vi_speed (freshness)
    double                 vi_pair_dt_mean_{0.033};  // running mean of pair dt (for s -> speed)
    int64_t                cur_frame_ts_ns_{0};       // current frame ts, set at processFrame entry
    // Raw MiDaS disparity (relative depth = 1/disparity; high=near) at an image
    // pixel, BEFORE the metric affine fit. No midas_affine_valid_ gate, so it works
    // even when the affine fit bails (too few 3D points). false if no depth map yet.
    bool sampleMidasRawDisparity(float u, float v, double& disparity_out) const;

    // 2026-05-31 — REPLAY-ONLY autonomous-p_G_ experiment flag (verdict §4).
    // Default false → device build is byte-identical (JNI never sets it). Set
    // true only via VioEngine::setAutonomousPgExperiment from the replay harness
    // --autonomous-pg flag. Gates the single setPosition line in processFrame.
    bool autonomous_pg_{false};
    // Running max |p_G_ - global_t_| divergence (m), for the verdict §4 read-only
    // diagnostic log. Only updated/logged when autonomous_pg_ is true.
    double autonomous_pg_max_div_m_{0.0};

    cv::Ptr<cv::CLAHE> clahe_;

    // Accuracy modules
    EKFState ekf_;
    FeatureManager feature_mgr_;
    LensCorrector lens_;
    TrackKLT klt_;
    UpdaterZeroVelocity zupt_detector_;
    // Plan Step 3a (ADR-008): MSCKF re-enabled with damping + Huber.
    // Invoked from `processFrame` section 11.1 against lost-feature candidates
    // returned by FeatureManager::getMSCKFCandidates(min_obs=4).
    UpdaterMSCKF msckf_updater_;
    // 2026-05-12: SLAM-feature updater with parallax + depth-observability +
    // per-observation chi² gates (OpenVINS doctrine). Without this, the older
    // EKFState::updateSlamFeature path applied Kalman updates to ρ even when
    // the camera had no parallax-bearing motion → ρ randomized → diverged to
    // zero → produced 100+ km world positions that poisoned PnP via the
    // keyframe pts3d_world buffer.
    UpdaterSLAM slam_updater_;

    // 2026-05-13: Phase 1 Step 5 — pose-graph back-end (post_v19_sprint_plan.md).
    // pose_graph_ stores 4-DOF (x, y, z, yaw) keyframe poses + odometry /
    // loop edges, and runs Gauss-Newton optimization on LC accept to
    // redistribute drift across all keyframes between K_match and K_now.
    //
    // clone_id_to_pg_node_ maps the EKF clone-id used by LoopClosureDetector
    // as its keyframe-id back to the pose-graph node-id assigned by
    // pose_graph_.addNode(). When a loop edge fires, this mapping converts
    // the LoopClosureDetector::LoopMatch.{matched_kf_id, now_kf_id} into
    // pose-graph node ids for addLoopEdge.
    //
    // Implementor-skill note: the pose graph is wired but the back-write to
    // LoopClosureDetector's stored keyframe poses (which would let loop 2
    // visually overlay loop 1 in the live trajectory) requires a new
    // LoopClosureDetector::updateStoredKeyframePose method — deferred to
    // the next chunk of Step 5. For now optimize() runs and publishes
    // counters; the math gets verified by event_summary residual ratios
    // before the back-write is added.
    PoseGraph pose_graph_;
    std::unordered_map<uint64_t, int> clone_id_to_pg_node_;
    // Reverse map for the post-optimize back-write loop. Tracker.cpp iterates
    // pose_graph_.snapshotNodes() after optimize(); for each node-id this map
    // resolves the corresponding LoopClosureDetector keyframe-id so
    // applyKeyframePoseCorrection can find the stored KeyframeRecord by id.
    std::unordered_map<int, uint64_t> pg_node_to_clone_id_;
    // Latest pose-graph node id assigned. Tracks the "now" side of a future
    // loop edge so consumeLoopClosureMatchIfReady can pair the matched
    // keyframe's pg-node with the current end of the chain.
    int last_pg_node_id_{-1};
    // 2026-05-26 — #2 loop-overlay path redraw. After optimize() corrects the
    // pose-graph nodes, consumeLoopClosureMatchIfReady snapshots the corrected
    // node (x=East, y=North) polyline here under pose_mutex_ and bumps
    // loop_correction_version_; the UI rebuilds its (drifted) pathHistory from it
    // (only the now-node delta reaches global_t_, so the PAST path needs redraw).
    std::vector<cv::Point2f> corrected_traj_xz_;       // guarded by pose_mutex_
    std::atomic<int> loop_correction_version_{0};

    // 2026-05-12: VisualMap member + getter reverted. Belongs to Phase 1
    // Step 6 (persistent landmark map) per post_v19_sprint_plan.md, which
    // is queued behind Step 5 (pose-graph back-end). Scaffolding kept on
    // disk at app/src/main/cpp/VisualMap.{h,cpp} for that step.
    // public:
    //     const VisualMap& getVisualMap() const { return visual_map_; }
    // private:
    //     VisualMap visual_map_;

    // MSCKF feature ID tracking (parallel to prev_pts_)
    std::vector<int> feature_ids_;

    // Keyframe tracking state
    int frames_since_keyframe_{0};

    // ── Plan Step 5: motion-blur skip counter ─────────────────────────────
    // Counts consecutive frames that have been classified as blurry
    // (variance of Laplacian < BLUR_VAR_THRESH) and therefore had their
    // visual measurement updates suppressed. Reset to 0 on the first frame
    // that survives the blur gate. Logged once per blur event so a sustained
    // blur (e.g. a whole-second head turn) is visible in the trace without
    // spamming a line per frame.
    int blur_skipped_streak_{0};

    // ── Plan Step 4 (ADR-010): ORB relocalization debounce ─────────────────
    // Counts consecutive frames where the geometric-verification inlier
    // count fell below MIN_INLIERS / 2 (= 4). When it reaches
    // RELOC_TRIGGER_FRAMES (= 3) we run the ORB descriptor relocalization
    // path against the recently-stored keyframe descriptor ring buffer.
    // Reset on any frame where inliers are healthy or the pose path is
    // skipped entirely (no measurement to evaluate).
    int low_inlier_streak_{0};

    // ── Plan Step 4 (ADR-010): ORB extractor for the relocalization path
    // Lazy-instantiated on first call to tryRelocalizeWithORB. Held as
    // an instance member (not a function-static inside that helper) so
    // future multi-threaded callers don't trip the C++11 function-
    // static-init race, and so reset() can clear it cleanly if needed.
    cv::Ptr<cv::ORB> reloc_orb_;

    // ── Plan Step 6 (ADR-012): windowed BA off-thread runner ──────────────
    //
    // Each new keyframe (1) consumes the previous BA round's refined
    // landmarks (if any) by re-seeding via removeSlamFeature + addSlamFeature
    // on the EKF, and (2) kicks off the next BA round in a detached worker
    // thread. The BA thread pulls a snapshot of the most-recent 5 EKF clones
    // and the SLAM landmarks observed by ≥ 2 of them, runs hand-rolled
    // Gauss-Newton with Huber loss in WindowedBA::solve, and publishes the
    // refined landmark positions back through ba_result_*_ below. The
    // camera thread reads + clears ba_result_pending_ on the next
    // keyframe; if the previous round is still in flight we LOG a "skipped"
    // line and do not start a new round (the plan §6 ordering — "one BA
    // round at a time").
    //
    // Thread-safety invariant: ba_thread_ writes ba_result_* under
    // ba_result_mutex_ exactly once per launched round, then sets
    // ba_in_flight_ = false. The camera thread holds ba_result_mutex_
    // while moving the result out and clearing ba_result_pending_.
    // ba_in_flight_ is std::atomic so the camera thread can cheaply
    // check it without taking the mutex.
    struct BARefinedLandmark {
        int       feature_id   = -1;
        int       slam_slot    = -1;     // slot at the time the BA was launched
        int       anchor_clone_id = -1;  // anchor when re-seeding via addSlamFeature
        cv::Mat   p_world_refined;       // 3x1 CV_64F, BA output
    };
    std::thread             ba_thread_;
    std::atomic<bool>       ba_in_flight_{false};
    bool                    ba_result_pending_{false};
    mutable std::mutex      ba_result_mutex_;
    std::vector<BARefinedLandmark> ba_result_landmarks_;
    int                     ba_round_counter_{0};

    // Plan Step 6 (ADR-012): hard cap on accepted BA solve wall time.
    // 2x the plan's 100 ms target so a single throttled solve under
    // thermal pressure can still publish; anything past this is a sign
    // the LM is thrashing on a degenerate scene and the result is
    // discarded rather than fed back into the EKF.
    static constexpr int BA_MAX_SOLVE_US = 200'000;

    // Plan Step 6 (ADR-012): launch the next BA round in a detached worker.
    // Cheaply snapshots EKF clones + FeatureManager landmarks from the
    // camera thread via the new thread-safe accessors, then std::thread's
    // its way through WindowedBA::solve. Returns true if a round was
    // launched, false if a previous round is still in flight (we skip
    // and LOGI "skipped"). Caller is the keyframe storage block in
    // processFrame.
    bool kickOffBARound(int64_t timestamp_ns);

    // Plan Step 6 (ADR-012): consume the result of the previous BA round
    // (if it has finished) and re-seed the corresponding SLAM features in
    // the EKF via removeSlamFeature + addSlamFeature with the BA-refined
    // world point. ADR-006 forbids direct EKF mean / covariance mutation
    // from side channels; this re-seeding goes through the canonical
    // EKFState API which sets up covariance correctly. Called from the
    // keyframe storage block, BEFORE kickOffBARound for the new keyframe.
    void consumeBAResultIfReady();

    // Plan Step 6 (ADR-012): terminate the BA worker cleanly. Called from
    // reset() and the destructor. Joins the thread (no detach) so any
    // in-flight round finishes before we return — that keeps the EKF /
    // FeatureManager mutexes from being touched after we've started
    // tearing the rest of the Tracker down.
    void shutdownBA();

    // ── Plan Step 7 (ADR-013): same-session loop closure (DBoW2) ──────────
    //
    // Owned LoopClosureDetector + 1 Hz worker thread + double-buffered
    // result handoff. Mirrors the BA worker pattern at section 6 above:
    //   * Camera thread snapshots the most recent keyframe (descriptors,
    //     keypoints, clone_id, intrinsics, ts) into `loop_closure_pending_*`
    //     under `loop_closure_query_mutex_` after every storeKeyframeDescriptors.
    //   * The worker thread wakes every LOOP_CLOSURE_QUERY_PERIOD_S seconds
    //     via `loop_closure_cv_`. It snapshots that buffer, calls
    //     `tryDetectLoop` outside the mutex (the call may be slow under
    //     dense vocabularies), and on success publishes a LoopMatch under
    //     `loop_closure_result_mutex_`.
    //   * The camera thread consumes the result on the next frame via
    //     `consumeLoopClosureMatchIfReady`, which decays a 10-frame damping
    //     ramp through `EKFState::updateRelativePose` /
    //     `updateRelativeRotation`. ADR-006 forbids direct EKF
    //     mean/covariance writes from a side channel — this is the same
    //     canonical observation channel Step 2 already uses.
    LoopClosureDetector       loop_closure_;

    // 2026-05-16 Phase 1 Step 6 (post_v19_sprint_plan.md §205-298) — visual-only
    // persistent 3D landmark database. Populated at every keyframe storage
    // event (Phase 6.2 wire-up; see Tracker.cpp §11.5). Landmarks are anchored
    // to keyframe ids that match `loop_closure_`'s id-space; loop-closure
    // back-write rotates/shifts landmarks via
    // `landmark_map_.applyKeyframePoseCorrection(kf_id, dx, dy, dz, dyaw)`
    // (Phase 6.4 wiring). EKF state is NOT coupled to landmark positions —
    // landmarks remain fixed during the pose-only measurement update
    // (Phase 6.3 EKFState::applyLandmarkObservations).
    navsight::LandmarkMap     landmark_map_;

    // 2026-05-16 Phase 1 Step 6.4 — accepted-this-frame landmark ids snapshot.
    // Written from the camera thread at the end of the EKF landmark update
    // block (Tracker.cpp §11.5b) under last_observed_mutex_. Read from any
    // thread via getLastObservedLandmarkIds(); the lock guarantees the reader
    // gets a coherent snapshot. Sized at most by the # of matched
    // observations per keyframe — typically O(10s), never O(1000s).
    mutable std::mutex        last_observed_mutex_;
    std::vector<int>          last_observed_landmark_ids_;
    // 2026-05-19 — Orange-dot anchor fix #3. Parallel array with
    // last_observed_landmark_ids_: index i holds the matched current-frame
    // pixel (KLT keypoint coords) for the landmark at ids[i]. Used by the
    // JNI overlay path to draw observed orange dots AT their actual image
    // feature instead of projecting a stale stored p_world through the
    // current camera pose.
    std::vector<cv::Point2f>  last_observed_landmark_pixels_;

    // 2026-05-19 Fix #11 — per-frame landmark pixel refresh.
    //
    // Cause: last_observed_landmark_pixels_ is set ONCE per keyframe (~1 Hz)
    // at the local-map descriptor-match site. Between keyframes (28-29 frames
    // at 30 Hz), the stored pixel does NOT track the corresponding image
    // feature as the camera moves — so orange dots draw at where the feature
    // WAS at the last keyframe, not where it IS now. User-visible symptom:
    // dots appear "stuck in screen space" while the image content slides
    // past them. The 2026-05-19 fix10_revisit walk's
    // landmarks_rendered_anchor_total = 58924 (~73/frame) but the user
    // reported seeing essentially no anchored dots — this is the gap.
    //
    // Fix: at every keyframe match, also record the feature_id of the KLT
    // track that descriptor-matched the landmark. Per-frame (after KLT
    // tracking), walk the live KLT features and for any feature_id linked
    // to a landmark, refresh the landmark's pixel from the live KLT pos.
    // The dot now slides with its feature at full 30 Hz.
    //
    // Parallel array (same lifetime + lock as ids/pixels). feature_ids[k]
    // = -1 means the link was lost (e.g., set when ids/pixels updated but
    // we didn't capture a feature_id, or when KLT track died and the
    // refresh detected the absence and cleared the link).
    std::vector<int>          last_observed_landmark_feature_ids_;

    // Worker-thread plumbing.
    std::thread               loop_closure_thread_;
    std::atomic<bool>         loop_closure_should_stop_{false};
    std::atomic<bool>         loop_closure_thread_running_{false};
    std::condition_variable   loop_closure_cv_;
    mutable std::mutex        loop_closure_query_mutex_;

    // Latest keyframe snapshot for the worker. Written by the camera thread
    // in section 11.5 right after each successful `storeKeyframeDescriptors`,
    // read by the worker thread on each query tick. Cheap copy: descriptors
    // is at most 250×32 ≈ 8 KB; keypoints is ≤ 250 entries.
    bool                      loop_closure_query_has_data_{false};
    cv::Mat                   loop_closure_query_descriptors_;
    std::vector<cv::KeyPoint> loop_closure_query_keypoints_;
    int                       loop_closure_query_kf_id_{-1};
    int64_t                   loop_closure_query_ts_ns_{0};
    double                    loop_closure_query_fx_{0.}, loop_closure_query_fy_{0.};
    double                    loop_closure_query_cx_{0.}, loop_closure_query_cy_{0.};
    double                    loop_closure_query_yaw_rad_{0.};
    // Step 7.1 — extra payload for the geometric loop-closure path. Snapshot
    // alongside the BoW query so both detectors see the SAME query frame.
    // The geometric path uses these to spatial-filter candidates and project
    // their pts3d_world into the current camera. See spec at
    // docs/VISUAL_PLAN_STEP_7_1_GEOMETRIC_LOOP.md.
    std::vector<cv::Point2f>  loop_closure_query_klt_corners_;
    cv::Matx33d               loop_closure_query_R_world_cam_{cv::Matx33d::eye()};
    cv::Vec3d                 loop_closure_query_t_cam_world_{0., 0., 0.};
    int                       loop_closure_query_img_w_{0};
    int                       loop_closure_query_img_h_{0};
    double                    loop_closure_query_search_radius_m_{0.};

    // Result handoff. The worker writes the latest accepted LoopMatch under
    // this mutex; the camera thread takes (and clears) it before applying
    // the EKF correction. `loop_closure_result_pending_` is a plain bool
    // protected by the mutex (not atomic) because it is only ever read and
    // cleared by the camera thread under the lock.
    mutable std::mutex                  loop_closure_result_mutex_;
    bool                                loop_closure_result_pending_{false};
    LoopClosureDetector::LoopMatch      loop_closure_pending_match_{};

    // Damping ramp consumed on the camera thread. While > 0, every call to
    // `consumeLoopClosureMatchIfReady` re-applies the cached match with
    // diminishing strength (var inflated by 1/strength²) and decrements
    // the counter. `loop_closure_active_match_` holds the LAST accepted
    // match while the ramp is in flight — a fresh match arriving mid-ramp
    // overwrites it, which is the desired "most-recent loop wins" behaviour.
    int                                  loop_closure_damping_remaining_{0};
    LoopClosureDetector::LoopMatch       loop_closure_active_match_{};
    bool                                 loop_closure_active_match_set_{false};

    // Camera thread: cache the latest keyframe's snapshot (descriptors +
    // keypoints + clone_id + intrinsics + ts) under loop_closure_query_mutex_,
    // and notify the worker via loop_closure_cv_ that a new tick is ready.
    // Called from the keyframe-storage block right after addKeyframe.
    void publishLoopClosureQueryKeyframe(int kf_id, int64_t ts_ns,
                                         const cv::Mat& descriptors,
                                         const std::vector<cv::KeyPoint>& keypoints,
                                         double fx, double fy,
                                         double cx, double cy,
                                         double yaw_rad,
                                         // Step 7.1 — extras for the geometric
                                         // loop-closure path (additive). Pass
                                         // empty vector / identity matrix /
                                         // zero radius to disable the geom
                                         // path for a given query.
                                         const std::vector<cv::Point2f>& klt_corners,
                                         const cv::Matx33d& R_world_cam_pred,
                                         const cv::Vec3d&   t_cam_world_pred,
                                         int img_w, int img_h,
                                         double search_radius_m);

    // 1 Hz query worker. Wakes on either loop_closure_cv_ or a timeout,
    // copies the pending query under the query mutex, calls tryDetectLoop,
    // and on success publishes the LoopMatch under loop_closure_result_mutex_.
    // Bumps event counters at every gate.
    void loopClosureWorkerLoop();

    // Camera thread: if the worker has published a fresh LoopMatch since the
    // last call, copy it into `loop_closure_active_match_` and reset the
    // damping ramp. Then, while the ramp is non-zero, inject a damped
    // relative-pose / relative-rotation correction through the canonical
    // EKF measurement channel (NOT a direct mean / covariance write —
    // ADR-006). Decrements the ramp counter on each call.
    void consumeLoopClosureMatchIfReady(IMUPreintegrator& imu);

    // Cleanly stop the worker thread. Sets should_stop_, notifies the cv,
    // and joins. Called from reset() and the destructor.
    void shutdownLoopClosure();

    // Magic numbers cited inline in the implementation:
    //   * 1.0 s query period            — Step 7 plan, line 718
    //   * 30 s temporal exclusion       — Step 7 plan, line 719
    //   * 10 frames damping ramp        — ADR-006 schedule
    //   * 0.05 m² translation variance  — Step 7 acceptance criteria
    //   * (3°)² ≈ 2.74e-3 rad² rotation — typical PnP-inlier σ at N≥30
    static constexpr double  LOOP_CLOSURE_QUERY_PERIOD_S      = 1.0;
    static constexpr int64_t LOOP_CLOSURE_TEMPORAL_EXCL_NS    = 30LL * 1'000'000'000LL;
    static constexpr int     LOOP_CLOSURE_DAMPING_FRAMES      = 10;
    /* LEGACY: Stage 2 v1 (2026-05-09) — direct global_t_ injection ramp.
       Reverted in Stage 2 v2 once Stage 1 (EKFState::updateGravityAlignment)
       bounded R_GtoI tilt and the EKF absolute-pose channel
       (consumeLoopClosureMatchIfReady → updateAbsolutePose) became viable
       again. The active correction ramp is LOOP_CLOSURE_DAMPING_FRAMES = 10
       above; nothing in any .cpp references the two constants below.
       Kept commented per project "comment, don't delete" rule — do NOT
       re-enable without restoring the global_t_ injection path.

    // Step χ-3 (2026-05-09) — direct global_t_ injection ramp parameters.
    // sim_data_1778329181805 showed 30 jumps > 0.5 m, max 3.32 m at 10
    // frames × 1.82 m first-frame strength. Extending the ramp to 30
    // frames + capping per-frame translation made corrections naturalistic
    // (comparable to walking-pace motion).
    static constexpr int     LOOP_CLOSURE_GLOBAL_RAMP_FRAMES  = 30;
    static constexpr double  LOOP_CLOSURE_GLOBAL_MAX_STEP_M   = 0.20;
    */
    // Dynamic 1-σ (m) for the loop-closure world-position measurement:
    //   sigma = max(LOOP_CLOSURE_PNP_SIGMA_FLOOR_M,
    //               LOOP_CLOSURE_DRIFT_RATE * total_path_m_)
    //
    // Why dynamic: tight MSCKF updates collapse P_[pp] to ~0, so
    //   S ≈ R_noise  and  m² ≈ |r_p|² / var_p.
    // The χ²(0.999,6)=22.5 gate then requires sigma ≥ actual_drift.
    // VIO drift grows with path length at ~15 %/100 m (measured from
    // sim_data_1778077139237: 14.6 m drift over ~115 m GPS path).
    //
    // LOOP_CLOSURE_PNP_SIGMA_FLOOR_M = 2.0 m
    //   Lower bound from PnP accuracy: with ≥30 inliers at ~3-5 m
    //   landmark depth, solvePnPRansac translation error is ~0.5-1.5 m;
    //   2.0 m is the safe floor that prevents the gate from being tighter
    //   than the sensor noise floor even on very short paths.
    //
    // LOOP_CLOSURE_DRIFT_RATE = 0.032 m drift per metre walked
    //   Derivation: 15 %/100 m drift rate divided by chi²_sigma_factor
    //   (√(22.5−0.84) = 4.65) = 0.15/4.65 = 0.032.
    //   Source: sim_data_1778077139237 + χ²(0.999,6) table.
    static constexpr double  LOOP_CLOSURE_PNP_SIGMA_FLOOR_M   = 2.0;
    static constexpr double  LOOP_CLOSURE_DRIFT_RATE           = 0.032;
    // 1-σ-axis (rad) on the rotation block of the loop-closure chi² gate.
    // 3° was far too tight: with m²_rot = |r_R|²/σ²_R the chi²(0.999,6)=22.5
    // threshold only accepts |r_R| < 14.2°, but typical VIO heading drift over
    // a 50-100 m loop is 5-20°, so all PnP accepts were rejected.
    // 20° floor (0.349 rad) allows corrections whenever heading residual
    // < sqrt(22.5 × 0.349²) ≈ 1.66 rad (limited in practice by the π/2
    // heading gate upstream). Source: sim_data_1778100250961 — 40 PnP accepts,
    // 392 chi² rejects, 0 corrections with 3° sigma.
    static constexpr double  LOOP_CLOSURE_BASE_ROT_SIGMA_RAD  = 0.34907;  // 20°

    // ── Pose-graph LOOP-EDGE weights (2026-05-25; BUG: loops don't overlay) ──
    // The pose-graph loop edge was being weighted with the chi²-GATE budget
    // var_p (= var_p_pnp 4 m² + drift inflation), giving loop info ≤ 0.25 while
    // each odometry edge has info ≈ 1/SIGMA_POS_FLOOR_SQ = 400 (5 cm floor).
    // The loop edge was ~1600× too weak, so PoseGraph::optimize() trusted the
    // drifted odometry chain and left the loop OPEN (residual ratio 0.99 on
    // lc2loop_2026_05_25). The drift budget belongs to the chi² ACCEPTANCE gate
    // (which must tolerate the large pre-correction gap) — NOT to the edge
    // WEIGHT. A loop closure is an absolute revisit constraint from an
    // appearance-verified PnP match, so its edge variance must be << the
    // ACCUMULATED odometry variance over the loop (≈ N·(5 cm)² ≈ 0.2 m² for an
    // ~80-edge loop) for the optimizer to deform the chain. var=0.01 m² closes
    // ~95% of the gap (0.2/(0.2+0.01)). Reproj geometry (4 px @ ~5 m depth ≈
    // 3 cm/inlier) supports sub-decimetre relative precision; 10 cm / 2° are
    // conservative floors. These weight the EDGE only; var_p and
    // sigma_axis_sq_R still gate acceptance unchanged.
    static constexpr double  LOOP_CLOSURE_EDGE_SIGMA_P_M     = 0.10;   // (10 cm)² edge weight
    static constexpr double  LOOP_CLOSURE_EDGE_SIGMA_YAW_RAD = 0.035;  // 2° edge weight

    // 2026-05-24 BUG (LC heading) — heading (world-Z yaw) uncertainty growth
    // per metre walked since the last loop closure. In monocular VIO global yaw
    // is UNOBSERVABLE, so its 1-σ should grow ~linearly with distance until an
    // absolute heading fix (loop closure). Before each LC rotation update we
    // inject σ²_yaw = (rate × path_since_last_lc_m_)² into P[2,2] so the update
    // has Kalman gain on yaw (rotation analog of LOOP_CLOSURE_DRIFT_RATE for
    // position). Derivation: observed heading drift ≈ 20° per ≈ 50 m loop in the
    // 2026-05-22/24 walks → 0.349 rad / 50 m ≈ 0.007 rad/m. Cross-check: at a
    // 50 m LC gap σ²_yaw = (0.007×50)² ≈ 0.122 rad² ≈ σ²_R (PnP rot floor
    // 0.349² = 0.122), so LC yaw gain K = σ²_yaw/(σ²_yaw+σ²_R) ≈ 0.5.
    static constexpr double  LOOP_CLOSURE_HEADING_DRIFT_RATE_RAD_PER_M = 0.007;

    // 2026-05-24 BUG (LC heading) — cap on injected heading 1-σ. Bounds the
    // single-update yaw gain so the 10-frame damping ramp still smooths rather
    // than snapping in one step, and stays within the ±90° LC heading gate.
    // π/4 (45°) → max K = (π/4)²/((π/4)²+0.122) ≈ 0.83.
    static constexpr double  LOOP_CLOSURE_HEADING_MAX_SIGMA_RAD = 0.785398;  // 45°

    // Depth-based scale constraint (MiDaS)
    mutable std::mutex depth_mutex_;
    std::vector<float> depth_map_;
    int depth_width_{0}, depth_height_{0};
    void applyDepthScaleConstraint(const std::vector<cv::Point2f>& pts2d,
                                    const std::vector<cv::Point3f>& pts3d,
                                    int img_width, int img_height,
                                    const IMUPreintegrator& imu);

    // 2026-05-19 Fix #12 — Cached VI-Depth affine fit (s, t) such that
    // inv_metric_depth = s · disparity + t. Updated by
    // applyDepthScaleConstraint on each keyframe where the fit passes the
    // ≥ 50 % inlier acceptance bar. Consumed by sampleMidasMetricDepth.
    // `midas_affine_valid_` gates use until the first successful fit.
    mutable std::mutex midas_affine_mutex_;
    double midas_affine_s_{0.0};
    double midas_affine_t_{0.0};
    bool   midas_affine_valid_{false};

    // Tuning constants for sampleMidasMetricDepth. See its declaration in
    // the public section above for the derivation. Widened from the
    // applyDepthScaleConstraint inlier band [0.3, 12] to [0.3, 30] to
    // accommodate longer sightlines on a scooter / outdoor walks.
    static constexpr double kMinMidasDepthM = 0.3;
    static constexpr double kMaxMidasDepthM = 30.0;

    // Constants
    static constexpr int    MAX_FEATURES       = 200;  // OpenVINS default for monocular
    static constexpr int    MIN_FEATURES       = 80;
    static constexpr double QUALITY_LEVEL      = 0.05;
    static constexpr double MIN_DIST           = 10.0;
    static constexpr double RANSAC_CONF        = 0.9999;
    static constexpr double RANSAC_THRESH      = 1.5;
    static constexpr double MIN_PARALLAX_PX    = 0.8;
    static constexpr double FB_CHECK_THRESH    = 4.0;  // 2px threshold (squared); was 9.0
    static constexpr double MIN_FLOW_PX        = 0.4;
    static constexpr double MAX_FLOW_PX        = 150.0;
    static constexpr int    MIN_INLIERS        = 8;
    // RELOC_LOW_INLIER_BAR floors integer division of MIN_INLIERS / 2.
    // If MIN_INLIERS is bumped to an odd value the bar silently rounds
    // down — make the assumption explicit at compile time.
    static_assert(MIN_INLIERS % 2 == 0,
                  "RELOC_LOW_INLIER_BAR assumes MIN_INLIERS is even");
    static constexpr double MIN_INLIER_RATIO   = 0.25;
    static constexpr double GYRO_ROT_ONLY_THRESH = 2.0;
    static constexpr double GYRO_ROT_ONLY_THRESH_SCOOTER = 4.0;
    // Plan Step 5: Rayleigh resultant cutoff for the dual-gate pure-rotation
    // detector. R/N is the mean resultant length of the per-feature optical-
    // flow direction vectors on the unit circle. R/N < 0.3 means the flow
    // directions are statistically uniform (Mardia 1972 — "Statistics of
    // Directional Data" §3.4), which is strong evidence the camera is
    // rotating rather than translating; under translation the flow points
    // of a static scene radiate from / converge to the focus-of-expansion
    // and concentrate strongly. Used in conjunction with the gyro-magnitude
    // gate — the Rayleigh test alone over-fires in the presence of
    // independent moving objects, so it only ARMS the existing gyro gate.
    static constexpr double FLOW_RAYLEIGH_REJECT = 0.3;
    // Plan Step 5: motion-blur acceptance threshold. Variance of Laplacian
    // is the OpenCV community heuristic for blur scoring (Pech-Pacheco et al.
    // 2000 — "Diatom autofocusing in brightfield microscopy: a comparative
    // study"); below ~80 a printed page becomes unreadable. Tuned on
    // synthetic Gaussian σ=5 blur in tests/cpp/test_visual_robustness.cpp;
    // re-tune from real Haifa head-turn sims if observed false-positive
    // rate climbs.
    static constexpr double BLUR_VAR_THRESH    = 80.0;
    static constexpr double BLUR_VAR_THRESH_SCOOTER = 50.0;
    static constexpr double ZUPT_GYRO_THRESH   = 0.04;
    static constexpr double ZUPT_GYRO_THRESH_SCOOTER = 0.15;
    static constexpr int    ACCEL_BIAS_WARMUP  = 150;
    static constexpr double ACCEL_BIAS_ALPHA   = 0.005;

    // ── Plan Step 4 (ADR-010): ORB relocalization tunables ─────────────────
    // Consecutive low-inlier frames before reloc fires. ~100 ms at 30 Hz —
    // long enough to debounce a single bad frame from motion blur, short
    // enough to recover before KLT collapses entirely.
    static constexpr int    RELOC_TRIGGER_FRAMES   = 3;
    // Inlier floor that arms the streak counter. MIN_INLIERS is the steady-
    // state acceptance bar; half of it is "the geometry has degenerated".
    static constexpr int    RELOC_LOW_INLIER_BAR   = MIN_INLIERS / 2;
    // Recent-appearance window: only the last N keyframes are scanned per
    // reloc. Keeps BFMatcher cost bounded; older keyframes have lower
    // appearance overlap with the present view anyway.
    static constexpr int    RELOC_RECENT_KFS       = 5;
    // Lowe's ratio (Lowe 2004 — "Distinctive Image Features from
    // Scale-Invariant Keypoints", §7.1). Standard 0.75 — slacker than the
    // 0.7 commonly used for SIFT because ORB's binary distance bins more
    // coarsely.
    static constexpr float  RELOC_LOWE_RATIO       = 0.75f;
    // RANSAC inlier accept threshold. Matches the steady-state mid-band of
    // the keyframe-yaw correction path (recoverPose ≥ 20 inliers there);
    // 30 keeps us above noise on dim or repetitive scenes.
    static constexpr int    RELOC_MIN_INLIERS      = 30;
    // Radius used when re-attaching restored feature ids to current KLT
    // tracks. Same 3 px as the ORB↔KLT inheritance step in FeatureManager.
    static constexpr float  RELOC_ID_REATTACH_RADIUS = 3.0f;
};
