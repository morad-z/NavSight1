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
#include "WindowedBA.h"

#include "InertialInitializer.h"
#include "LoopClosureDetector.h"

#include <condition_variable>

// ─────────────────────────────────────────────────────────────────────────────
// Step 8 — Cleanup status (per docs/PRODUCTION_READINESS_PLAN.md §8)
// ─────────────────────────────────────────────────────────────────────────────
// Items the plan flags for deletion are kept here as commented LEGACY blocks
// (per user request — "comment, don't delete"). Search this file for "LEGACY"
// or "DEAD CODE" to find them. Specifically retained:
//   • global_R_ / global_t_ — Tracker-owned pose (mirror of EKFState).
//   • scalar_heading_       — yaw mirror published via getHeading()/JNI.
//   • UpdaterMSCKF member   — never exercised after Step 4; its update site
//                             at Tracker.cpp:1193 is wrapped in DISABLED.
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

    // Step 1 (Visual Production Plan) — push 8-coefficient rational
    // distortion into the owned LensCorrector. Caller is responsible for
    // gating validity (the JSON loader's RMS gate is what protects the
    // pipeline from junk coefficients).
    void setDistortion(double k1, double k2, double k3,
                       double k4, double k5, double k6,
                       double p1, double p2);

    void setInitialHeading(double azimuth_rad);
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

    // Thread-safe read-only accessors
    double getSmoothScale() const;
    double getHeading() const;
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
    // The production-readiness plan (Step 4 / Step 8) calls for removing
    // these in favor of EKFState as the canonical owner of pose. They are
    // retained per user request ("comment, don't delete") because they
    // still serve TWO live roles:
    //   1. Init-bootstrap seed before EKFState::isFullInitialized() — the
    //      magnetometer one-shot writes here (Tracker.cpp:491,507) and the
    //      InertialInitializer drains here (Tracker.cpp:397,420) before
    //      ekf_.initializeFull is called.
    //   2. Read-only mirror of ekf_.getRotation()/getPosition() refreshed
    //      each frame (Tracker.cpp:1014-1015) for downstream consumers
    //      that still read the legacy field directly (KeyframePair clone
    //      seed at 945, addClone fallback at 1140-1141, log line 1373).
    // When all consumers migrate to ekf_ accessors, this pair becomes dead.
    cv::Mat global_R_;   // 3x3 CV_64F  — LEGACY mirror / bootstrap seed
    cv::Mat global_t_;   // 3x1 CV_64F  — LEGACY mirror / bootstrap seed

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

    // Phase 5: Magnetometer heading used ONCE at startup only
    bool heading_initialized_{false};

    // Heading cache (read-only mirror of EKFState yaw, refreshed each frame
    // by processFrame). Step 4: EKFState owns yaw; this is a snapshot for
    // const accessors and downstream consumers that want a scalar.
    //
    // Step 8 cleanup status: the plan calls for removing this mirror once
    // all callers go through ekf_.getYaw() directly. Kept (not deleted) per
    // user request — Tracker::getHeading() and the JNI layer still read it,
    // and replay_harness::getPose relies on it as the canonical scalar yaw.
    double scalar_heading_{0.0};
    // Cumulative odometric path length (metres). Incremented in both the
    // visual update path and the PDR step so the loop-closure dynamic sigma
    // (LOOP_CLOSURE_DRIFT_RATE * total_path_m_) tracks real walked distance.
    double total_path_m_{0.0};
    // Pending init heading from setInitialHeading() — applied as a bootstrap
    // seed for ekf_.initializeFull on the first frame.
    double pending_init_heading_{0.0};
    bool   pending_init_heading_set_{false};
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
                                         double yaw_rad);

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
    void consumeLoopClosureMatchIfReady();

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

    // Depth-based scale constraint (MiDaS)
    mutable std::mutex depth_mutex_;
    std::vector<float> depth_map_;
    int depth_width_{0}, depth_height_{0};
    void applyDepthScaleConstraint(const std::vector<cv::Point2f>& pts2d,
                                    const std::vector<cv::Point3f>& pts3d,
                                    int img_width, int img_height,
                                    const IMUPreintegrator& imu);

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
    static constexpr double ZUPT_GYRO_THRESH   = 0.04;
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
