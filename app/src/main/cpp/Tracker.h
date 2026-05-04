#pragma once
#include <vector>
#include <mutex>
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

#include "InertialInitializer.h"

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
