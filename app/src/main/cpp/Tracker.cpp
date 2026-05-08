#include "Tracker.h"
#include "EventCounters.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>

#include <chrono>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Tracker"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGE(...) (void)0
#define LOGD(...) (void)0
#endif

// Timing helper
static inline int64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ── Constructor ──────────────────────────────────────────────────────────────

Tracker::Tracker() {
    global_R_ = cv::Mat::eye(3, 3, CV_64F);
    global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    accel_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    current_prev_pts_buf_.reserve(MAX_FEATURES);
    next_pts_buf_.reserve(MAX_FEATURES);
    prev_good_buf_.reserve(MAX_FEATURES);
    next_good_buf_.reserve(MAX_FEATURES);
    new_pts_buf_.reserve(MAX_FEATURES);

    clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
    LOGI("Tracker created (EKF+GridFeatures+LensCorrector)");
}

// ── Setters ──────────────────────────────────────────────────────────────────

void Tracker::setIntrinsics(double fx, double fy, double cx, double cy) {
    std::lock_guard<std::mutex> lock(mutex_);
    fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    lens_.setIntrinsics(fx, fy, cx, cy);
    // Plan Step 3b (ADR-009): mirror intrinsics into EKFState so SLAM
    // reprojection (pixel-space) sees the live calibration.
    ekf_.setSlamIntrinsics(fx, fy, cx, cy);
    LOGI("setIntrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy);
}

void Tracker::setDistortion(double k1, double k2, double k3,
                            double k4, double k5, double k6,
                            double p1, double p2) {
    std::lock_guard<std::mutex> lock(mutex_);
    lens_.setDistortion(k1, k2, k3, k4, k5, k6, p1, p2);
}

void Tracker::setUserScaleCorrection(double correction) {
    std::lock_guard<std::mutex> lock(mutex_);
    user_scale_correction_ = std::max(0.1, std::min(5.0, correction));
}

// Step 8c (Visual Production Plan): store rolling-shutter row-skew and
// mirror it into EventCounters so the sim JSON shows the device value.
// Source: Android Camera2 API — CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW
// (API level 21+): nanoseconds from first-row to last-row read-out.
void Tracker::setRollingShutterSkew(int64_t row_skew_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    rolling_shutter_row_skew_ns_ = row_skew_ns;
    navsight::eventCounters().rolling_shutter_skew_ns.store(
        static_cast<long long>(row_skew_ns), std::memory_order_relaxed);
}

// Step 8b: seed the EKF's R_bc from the Android camera sensor-orientation matrix.
// R_bc_flat is 9 floats in row-major order.  Converted to Matx33d and forwarded
// to EKFState::setExtrinsicsRotation which logs the angle from identity.
void Tracker::setExtrinsicsRotation(const float* R_bc_flat) {
    if (!R_bc_flat) return;
    cv::Matx33d R_bc(
        static_cast<double>(R_bc_flat[0]), static_cast<double>(R_bc_flat[1]), static_cast<double>(R_bc_flat[2]),
        static_cast<double>(R_bc_flat[3]), static_cast<double>(R_bc_flat[4]), static_cast<double>(R_bc_flat[5]),
        static_cast<double>(R_bc_flat[6]), static_cast<double>(R_bc_flat[7]), static_cast<double>(R_bc_flat[8])
    );
    std::lock_guard<std::mutex> lock(mutex_);
    ekf_.setExtrinsicsRotation(R_bc);
}

void Tracker::setDepthMap(const float* depth_data, int width, int height) {
    if (!depth_data || width <= 0 || height <= 0) return;
    std::lock_guard<std::mutex> lock(depth_mutex_);
    size_t sz = static_cast<size_t>(width * height);
    if (depth_map_.size() != sz) depth_map_.resize(sz);
    std::copy(depth_data, depth_data + sz, depth_map_.begin());
    depth_width_ = width;
    depth_height_ = height;
}

// Depth-based scale constraint: uses MiDaS relative depth + camera height
// to estimate absolute scale and blend it into smooth_scale_.
// Only applies when confidence is high (many matching points, consistent ratios).
void Tracker::applyDepthScaleConstraint(
        const std::vector<cv::Point2f>& pts2d,
        const std::vector<cv::Point3f>& pts3d,
        int img_width, int img_height,
        const IMUPreintegrator& imu) {

    auto& ec_md = navsight::eventCounters();
    ec_md.midas_entries.fetch_add(1, std::memory_order_relaxed);

    LOGI("DEPTH_SCALE: entry pts3d=%zu pts2d=%zu", pts3d.size(), pts2d.size());

    std::vector<float> depth_copy;
    int dw, dh;
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (depth_map_.empty()) {
            ec_md.midas_bailout_no_depth.fetch_add(1, std::memory_order_relaxed);
            LOGI("DEPTH_SCALE: BAILOUT no depth map");
            return;
        }
        depth_copy = depth_map_;
        dw = depth_width_;
        dh = depth_height_;
    }

    if (pts3d.size() < 15 || pts2d.size() != pts3d.size()) {
        ec_md.midas_bailout_few_pts3d.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT too few pts3d=%zu pts2d=%zu (need 15)", pts3d.size(), pts2d.size());
        return;
    }
    if (fx_ <= 0 || fy_ <= 0) {
        ec_md.midas_bailout_invalid_intrinsics.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT invalid intrinsics fx=%.1f fy=%.1f", fx_, fy_);
        return;
    }

    // Camera height from user height (phone held ~0.85 * user_height)
    float user_h = imu.getUserHeight();
    double camera_h = static_cast<double>(user_h) * 0.85;
    if (camera_h < 0.8 || camera_h > 2.2) {
        ec_md.midas_bailout_camera_h.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT camera_h=%.2f out of range [0.8, 2.2]", camera_h);
        return;
    }

    // Use gravity vector to estimate camera pitch (how much phone tilts down)
    float ax = imu.lastAccelX(), ay = imu.lastAccelY(), az = imu.lastAccelZ();
    double g_mag = std::sqrt(ax*ax + ay*ay + az*az);
    if (g_mag < 5.0) {
        ec_md.midas_bailout_low_g.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT g_mag=%.2f too low (no valid gravity)", g_mag);
        return;
    }
    // Pitch: angle between phone's Z-axis and horizontal plane
    double pitch = std::asin(std::min(1.0, std::max(-1.0, static_cast<double>(az) / g_mag)));

    // Step 3 Observer B: gravity unit vector in camera/phone frame.
    // A feature's component along this axis tells us how far below the camera
    // it sits in world frame — independent of where it lands in the image.
    // This unlocks scooter mode: camera looks forward, but the road is still
    // geometrically below the phone, so it should still be treated as floor.
    const double inv_g = 1.0 / g_mag;
    const double gxu = static_cast<double>(ax) * inv_g;
    const double gyu = static_cast<double>(ay) * inv_g;
    const double gzu = static_cast<double>(az) * inv_g;

    // Snapshot current scale once for the geometric floor test (avoids holding
    // pose_mutex_ across the whole feature loop).
    double current_scale_snapshot = scale_fuser_.scale();
    // A feature is geometrically a "floor" feature if its gravity-projected
    // metric depth (using current scale) is below the phone, with a margin.
    // 0.3m margin avoids accepting features at roughly head/torso height as
    // floor. Note: this is a soft inclusion test — we still bound metric_z
    // and ratio below before accepting.
    const double FLOOR_BELOW_PHONE_MARGIN_M = 0.3;
    const double floor_geom_threshold_m =
        FLOOR_BELOW_PHONE_MARGIN_M;

    // For features in the lower 40% of the image OR features that geometric
    // analysis says are below phone height in world frame (scooter case),
    // compute: metric_depth = camera_h / (norm_y * cos(pitch) + sin(pitch))
    // Then compare to VIO triangulated depth → scale ratio
    std::vector<double> scale_ratios;
    float fh = static_cast<float>(img_height);
    float fw = static_cast<float>(img_width);
    int floor_via_image = 0;
    int floor_via_geom = 0;

    for (size_t i = 0; i < pts3d.size(); i++) {
        // Image-space floor heuristic (handheld walking, phone tilted down)
        bool is_floor_by_image = (pts2d[i].y >= fh * 0.6f);

        // Gravity-projected floor heuristic (scooter, phone forward-facing).
        // pts3d[i] is in VIO units (scale-ambiguous); gravity-axis component
        // gives "depth below camera" in those same units. Convert with the
        // current scale snapshot to get an approximate metric value.
        double down_vio = static_cast<double>(pts3d[i].x) * gxu
                        + static_cast<double>(pts3d[i].y) * gyu
                        + static_cast<double>(pts3d[i].z) * gzu;
        double down_metric_est = down_vio * current_scale_snapshot;
        bool is_floor_by_geom = (down_metric_est > floor_geom_threshold_m);

        if (!is_floor_by_image && !is_floor_by_geom) continue;
        if (is_floor_by_image) floor_via_image++;
        else                   floor_via_geom++;

        // Depth bounds are in METERS — but pts3d[i] comes out of
        // cv::triangulatePoints in VIO baseline-units (line 1118 builds the
        // P matrix with the unit-norm t_vo from cv::recoverPose, so triangulated
        // depth is "1 baseline = 1 unit"). Convert to metric using the current
        // scale snapshot before bounds-checking. Pre-2026-05-09 the bounds were
        // applied in VIO units which rejected nearly every feature at typical
        // 1-10 m metric range — MiDaS bailed out with "few_floor" 100 % of the
        // time. Bug surfaced when MiDaS was instrumented (sims 1778147132092
        // onward all show midas_fused=0 with 30-50 entries per ~120 s walk).
        const double pts3d_z_metric = static_cast<double>(pts3d[i].z) * current_scale_snapshot;
        if (pts3d_z_metric < 0.3 || pts3d_z_metric > 12.0) continue;

        // Map 2D point to depth map coordinates
        int dx = static_cast<int>((pts2d[i].x / fw) * dw);
        int dy = static_cast<int>((pts2d[i].y / fh) * dh);
        if (dx < 0 || dx >= dw || dy < 0 || dy >= dh) continue;

        float rel_depth = depth_copy[dy * dw + dx];
        if (rel_depth < 0.01f) continue;

        // Normalized image coordinate (vertical)
        double norm_y = (static_cast<double>(pts2d[i].y) - cy_) / fy_;

        // Ground plane constraint: metric_Z = camera_h / (norm_y * cos(pitch) + sin(pitch))
        double denom = norm_y * std::cos(pitch) + std::sin(pitch);
        if (denom < 0.05) continue; // too close to horizon

        double metric_z = camera_h / denom;
        if (metric_z < 0.3 || metric_z > 10.0) continue;

        // Scale ratio: how much should we multiply VIO depth to get metric depth?
        double ratio = metric_z / pts3d[i].z;
        if (ratio > 0.1 && ratio < 10.0) {
            scale_ratios.push_back(ratio);
        }
    }

    if (scale_ratios.size() < 8) {
        ec_md.midas_bailout_few_floor_matches.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT only %zu floor matches (need 8) [image=%d geom=%d]",
             scale_ratios.size(), floor_via_image, floor_via_geom);
        return;
    }

    // Take median ratio
    std::sort(scale_ratios.begin(), scale_ratios.end());
    const size_t N = scale_ratios.size();
    double median_ratio = scale_ratios[N / 2];

    // Step 3 Observer B: MAD-based confidence. MAD = median(|x - median|).
    // sigma_robust ≈ 1.4826 * MAD (consistent estimator of stddev under a
    // normal). Variance of the median itself is sigma_robust² / N. We use
    // this for two things: (1) modulate the EMA alpha so noisy frames nudge
    // the scale less, (2) expose to the EKF for proper fusion later.
    std::vector<double> abs_dev;
    abs_dev.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        abs_dev.push_back(std::abs(scale_ratios[i] - median_ratio));
    }
    std::sort(abs_dev.begin(), abs_dev.end());
    double mad = abs_dev[N / 2];
    // Floor MAD to avoid divide-by-near-zero when ratios coincidentally line
    // up; 1% of the median is a sane lower bound for monocular depth.
    const double mad_floor = std::max(1e-3, 0.01 * std::abs(median_ratio));
    if (mad < mad_floor) mad = mad_floor;
    const double sigma_ratio = 1.4826 * mad;
    const double median_variance =
        (sigma_ratio * sigma_ratio) / static_cast<double>(N);

    // Compute target scale: current_scale * median_ratio
    double current = scale_fuser_.scale();
    double target_scale = current * median_ratio;
    target_scale = std::max(0.01, std::min(10.0, target_scale));

    // Safety gate: only apply if correction is within 3x of current
    if (target_scale > 3.0 * current || target_scale < current / 3.0) {
        ec_md.midas_rejected_extreme.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: REJECTED target=%.4f current=%.4f ratio=%.4f (too extreme)",
             target_scale, current, median_ratio);
        return;
    }

    // Step 3 Fusion: feed (target_scale, median_variance) into the 1-D Kalman
    // fuser. Tight ratios (low sigma) → small r → strong update; noisy ratios
    // → large r → minimal update. Replaces the previous confidence-weighted
    // alpha-blend so that PDR/MiDaS/VI all share one statistically-grounded
    // estimator instead of fighting via independent EMAs.
    bool accepted = scale_fuser_.update(target_scale, median_variance);
    if (accepted) {
        ec_md.midas_fused.fetch_add(1, std::memory_order_relaxed);
    } else {
        ec_md.midas_skipped.fetch_add(1, std::memory_order_relaxed);
    }
    {
        std::lock_guard<std::mutex> slock(pose_mutex_);
        last_depth_scale_variance_ = median_variance;
    }

    if (frame_counter_ % 30 == 0) {
        LOGI("DEPTH_SCALE: %s target=%.4f smooth=%.4f ratio=%.4f "
             "samples=%zu (img=%d geom=%d) sigma=%.3f var=%.5f P=%.5f",
             accepted ? "fused" : "skipped",
             target_scale, scale_fuser_.scale(), median_ratio, N,
             floor_via_image, floor_via_geom,
             sigma_ratio, median_variance, scale_fuser_.variance());
    }
}

void Tracker::setInitialHeading(double azimuth_rad) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // Build world→body Z-up matrix for compass-CW azimuth (matches the
    // convention pinned by scripts/test_z_up_conventions.py).
    double c = std::cos(azimuth_rad), s = std::sin(azimuth_rad);
    cv::Mat new_R = (cv::Mat_<double>(3, 3) << c,-s,0, s,c,0, 0,0,1);

    if (ekf_.isFullInitialized()) {
        // Post-init bootstrap correction. Kotlin's handleVioInitialized
        // typically fires AFTER ekf_.initializeFull (the UI-thread dispatch
        // of `vio.isInitialized` lands one or two frames after the C++
        // pipeline initializes the EKF with R_GtoI=Identity). Previously
        // this was a no-op, leaving the EKF's R_GtoI at Identity for the
        // entire session — loop closure chi² then rejected every PnP
        // correction as a 180° teleportation, and Step 7 acceptance was
        // permanently blocked. Apply the heading correction directly to
        // R_GtoI_ instead. Bug surfaced 2026-05-09 on sim 1778260615221
        // where vyaw (g_yaw of R_GtoI) stayed near 0 while Madgwick hdg
        // correctly tracked the user's compass heading.
        ekf_.setRotation(new_R);
        global_R_ = new_R;
        scalar_heading_ = azimuth_rad;
        LOGI("setInitialHeading: post-init R_GtoI corrected, az=%.1f deg",
             azimuth_rad * 180.0 / M_PI);
        return;
    }

    // Pre-init path: stash the heading and let processFrame's bootstrap
    // block initialize the EKF with it.
    global_R_ = new_R;
    pending_init_heading_ = azimuth_rad;
    pending_init_heading_set_ = true;
    scalar_heading_ = azimuth_rad;
    ekf_.initialize(scale_fuser_.scale());
    LOGI("setInitialHeading: pre-init az=%.1f deg (EKF scale initialized)",
         azimuth_rad * 180.0 / M_PI);
}

void Tracker::reset() {
    // Plan Step 6 (ADR-012): join the BA worker BEFORE we lock mutex_ or
    // reset ekf_ / feature_mgr_, so the worker is guaranteed to have
    // released its read locks on the snapshot mutexes by the time the
    // EKF / FeatureManager reset paths reach for them.
    shutdownBA();

    // Plan Step 7 (ADR-013): same protocol for the loop-closure worker —
    // joined first so the next reset of `loop_closure_` / EKF / camera
    // intrinsics is uncontested.
    shutdownLoopClosure();

    std::lock_guard<std::mutex> lock(mutex_);
    prev_gray_.release();
    prev_pts_.clear();
    prev_timestamp_ns_ = 0;
    initialized_ = false;
    frame_counter_ = 0;
    global_R_ = cv::Mat::eye(3, 3, CV_64F);
    global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    accel_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    accel_bias_count_ = 0;
    // Lowered from (0.20, 1.0) on 2026-05-04: every walk's vsc trace
    // converged to 0.025-0.075 within ~15 PDR steps, so 0.20 wasted
    // ~10 s of early frames at a 4x over-scaled state. 0.10 is a
    // less-biased prior for typical phone (focal ~525 px) + walking
    // (~0.65 m stride), and the wider initial variance (4.0 vs 1.0)
    // lets the first PDR/MiDaS/VI observation pull strongly so the
    // bootstrap median reset (Tracker.cpp:1139) still wins on real
    // device-specific differences. Per-device persistence is a future
    // step (would land in camera_calib.json or a sibling file).
    scale_fuser_.reset(0.10, 4.0);
    scale_obs_count_ = 0;
    scale_bootstrap_buf_.clear();
    points_3d_current_.clear();
    feature_ages_.clear();
    feature_ids_.clear();
    heading_initialized_ = false;
    scalar_heading_ = 0.0;
    total_path_m_ = 0.0;
    loop_closure_query_yaw_rad_ = 0.0;
    pending_init_heading_set_ = false;
    pending_init_heading_ = 0.0;
    filtered_yaw_rate_ = 0.0;
    last_visual_yaw_variance_ = -1.0;
    last_depth_scale_variance_ = -1.0;
    // Lowered from (0.20, 1.0) on 2026-05-04: every walk's vsc trace
    // converged to 0.025-0.075 within ~15 PDR steps, so 0.20 wasted
    // ~10 s of early frames at a 4x over-scaled state. 0.10 is a
    // less-biased prior for typical phone (focal ~525 px) + walking
    // (~0.65 m stride), and the wider initial variance (4.0 vs 1.0)
    // lets the first PDR/MiDaS/VI observation pull strongly so the
    // bootstrap median reset (Tracker.cpp:1139) still wins on real
    // device-specific differences. Per-device persistence is a future
    // step (would land in camera_calib.json or a sibling file).
    scale_fuser_.reset(0.10, 4.0);
    last_scale_predict_ns_ = 0;
    scale_estimator_vi_.reset();
    observer_c_pair_count_ = 0;
    td_warmup_done_ = false;
    td_warmup_buf_.clear();
    frames_since_keyframe_ = 0;
    low_inlier_streak_ = 0;
    blur_skipped_streak_ = 0;
    last_step_speed_ = 0.0;
    last_step_speed_ns_ = 0;
    rolling_shutter_row_skew_ns_ = 0;  // Step 8c: clear stale skew on session reset
    ekf_.reset();
    feature_mgr_.reset();
    // Plan Step 6 (ADR-012): drop any pending BA result so the next session
    // does not consume a stale refinement.
    {
        std::lock_guard<std::mutex> rlock(ba_result_mutex_);
        ba_result_landmarks_.clear();
        ba_result_pending_ = false;
    }
    ba_round_counter_ = 0;

    // Plan Step 7 (ADR-013): clear the loop-closure handoff buffers so a
    // stale pending match from the previous session cannot trigger a
    // correction on the first frame after reset. The detector itself is
    // NOT cleared here — the vocabulary stays loaded, and the keyframe
    // database emptiness is restored on the next session naturally
    // (addKeyframe is only called once a real keyframe is created).
    {
        std::lock_guard<std::mutex> qlock(loop_closure_query_mutex_);
        loop_closure_query_has_data_ = false;
        loop_closure_query_descriptors_.release();
        loop_closure_query_keypoints_.clear();
        loop_closure_query_kf_id_ = -1;
        loop_closure_query_ts_ns_ = 0;
    }
    {
        std::lock_guard<std::mutex> rlock(loop_closure_result_mutex_);
        loop_closure_result_pending_ = false;
        loop_closure_pending_match_ = LoopClosureDetector::LoopMatch{};
    }
    loop_closure_damping_remaining_ = 0;
    loop_closure_active_match_set_ = false;
    loop_closure_active_match_    = LoopClosureDetector::LoopMatch{};

    LOGI("Tracker reset");
}

// ── Plan Step 6 (ADR-012): destructor — clean BA worker join ────────────────
// ── Plan Step 7 (ADR-013): also stops the loop-closure worker thread.
Tracker::~Tracker() {
    shutdownBA();
    shutdownLoopClosure();
}

// ── Helpers ──────────────────────────────────────────────────────────────────

double Tracker::measureBlur(const cv::Mat& gray) const {
    // Plan Step 5: variance of Laplacian on the centre 50%×50% crop. The
    // crop bounds the cost (we never look at the whole frame) and avoids
    // edge artefacts (vignetting / motion-induced edge darkening) that
    // would inflate the score when the scene-content middle is actually
    // soft. Returns -1 for unusable inputs so the caller can treat that
    // case as "do not skip" (we cannot tell either way).
    if (gray.empty() || gray.cols < 8 || gray.rows < 8) {
        return -1.0;
    }
    const int crop_w = gray.cols / 2;
    const int crop_h = gray.rows / 2;
    const int x0     = (gray.cols - crop_w) / 2;
    const int y0     = (gray.rows - crop_h) / 2;
    cv::Mat roi = gray(cv::Rect(x0, y0, crop_w, crop_h));

    cv::Mat lap;
    cv::Laplacian(roi, lap, CV_32F);

    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    const double sigma = stddev[0];
    return sigma * sigma;  // variance
}

double Tracker::estimateScaleFromSteps(double vision_disp, int64_t dt_ns,
                                        IMUPreintegrator& imu) {
    if (vision_disp < 1e-5) return -1.0;
    double dt = dt_ns * 1e-9;
    if (dt <= 0.0 || dt > 1.5) return -1.0;

    auto step_info = imu.getStepInfo();
    double speed = step_info.speed_mps;
    if (speed < 0.2) {
        speed = imu.getVehicleSpeedEstimate();
        if (speed < 1.0) return -1.0;
    }
    double real_disp = speed * dt;
    if (real_disp < 1e-4) return -1.0;

    double obs_scale = std::max(0.005, std::min(10.0, real_disp / vision_disp));

    double current_smooth = scale_fuser_.scale();
    int obs_count;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        obs_count = scale_obs_count_;
    }
    if (obs_count >= 30 &&
        (obs_scale > 3.0 * current_smooth || obs_scale < current_smooth / 3.0)) {
        return -1.0;
    }
    return obs_scale;
}

// DEAD CODE: blendScale/applyLoopCorrection — VioEngine.applyMapperResult is a no-op, never calls these
// void Tracker::blendScale(double target_scale, double alpha) { ... }
// void Tracker::applyLoopCorrection(double tx, double ty, double tz, ...) { ... }

double Tracker::getSmoothScale() const {
    return scale_fuser_.scale();
}
double Tracker::getHeading() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return scalar_heading_;
}
double Tracker::getLastVisualYawVariance() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return last_visual_yaw_variance_;
}
double Tracker::getLastDepthScaleVariance() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return last_depth_scale_variance_;
}

bool Tracker::getPositionCovarianceXZ(double out[3]) const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!ekf_.isFullInitialized()) {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        return false;
    }
    cv::Mat P = ekf_.getCovariance();
    if (P.empty() || P.rows < 15 || P.cols < 15 || P.type() != CV_64F) {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 0.0;
        return false;
    }
    // EKFState 15-DOF error state layout: [δθ(3), δb_g(3), δv(3), δb_a(3), δp(3)].
    // Position block is at indices 12..14. Z-up ENU world: horizontal plane
    // is X-Y (Z is vertical). Sub-block is (12,12), (12,13), (13,13).
    // Note: the function name retains the "XZ" suffix for ABI/JNI stability;
    // the indices below correctly reflect the X-Y horizontal plane post-Z-up
    // alignment (2026-05-07, see scripts/test_z_up_conventions.py).
    out[0] = P.at<double>(12, 12);  // σ_xx (m²)  — East variance
    out[1] = P.at<double>(12, 13);  // σ_xy (m²)  — East/North covariance
    out[2] = P.at<double>(13, 13);  // σ_yy (m²)  — North variance
    return true;
}

void Tracker::addImuData(int64_t ts, float ax, float ay, float az, float gx, float gy, float gz) {
    if (!initialized_) {
        initializer_.addImuData(ts, ax, ay, az, gx, gy, gz);
        if (initializer_.isReady()) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            global_R_ = initializer_.getInitialRotation();
            // Gyro bias now managed solely by IMUPreintegrator (unified)
            ekf_.initialize(scale_fuser_.scale());
            initialized_ = true;
            LOGI("Tracker: System initialized via InertialInitializer");
        }
    }
}

InertialInitializer::Status Tracker::getInitStatus() const {
    return initializer_.getStatus();
}

void Tracker::clearInitTimeout() {
    initializer_.clearTimeout();
}

void Tracker::loadStoredCalibration(const cv::Mat& R_GtoI,
                                     const cv::Point3f& gyro_bias,
                                     const cv::Point3f& accel_bias) {
    initializer_.loadCalibration(R_GtoI, gyro_bias, accel_bias);
    if (!initialized_) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        global_R_ = initializer_.getInitialRotation();
        ekf_.initialize(scale_fuser_.scale());
        initialized_ = true;
    }
}

cv::Mat Tracker::getInitialRotation() const {
    return initializer_.getInitialRotation();
}

cv::Point3f Tracker::getCalibratedGyroBias() const {
    return initializer_.getGyroBias();
}

cv::Point3f Tracker::getCalibratedAccelBias() const {
    return initializer_.getAccelBias();
}

// ── processFrame ─────────────────────────────────────────────────────────────

VisionOutput Tracker::processFrame(const uint8_t* yuv_data, int width, int height,
                                    int64_t timestamp_ns, IMUPreintegrator& imu,
                                    TrackerFrame& frame_out) {
    VisionOutput out{};
    frame_out = {};

    if (!yuv_data || width <= 0 || height <= 0) {
        LOGE("processFrame: invalid args (yuv_data=%p, %dx%d)", yuv_data, width, height);
        return out;
    }

    int64_t t0_total = now_us();

    // PERF: log analyzer frame size once per second so we can confirm 640×480.
    // If ResolutionSelector ever falls back to 1280×960 the per-frame KLT and
    // cornersSB cost is 4× by pixel area; this line is the single source of
    // truth for "are we actually at the design resolution?"
    if (frame_counter_ % 30 == 0) {
        LOGI("PERF: section=frame_size w=%d h=%d", width, height);
    }

    // ── 1. YUV NV21 → grayscale + adaptive CLAHE ────────────────────────────
    cv::Mat yuv(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(yuv_data));
    cv::cvtColor(yuv, gray_buf_, cv::COLOR_YUV2GRAY_NV21);

    double frame_brightness = cv::mean(gray_buf_)[0] / 255.0;
    bool is_low_light = (frame_brightness < 0.12);

    // Skip CLAHE in well-lit conditions (saves ~2ms/frame)
    if (frame_brightness < 0.55) {
        clahe_->apply(gray_buf_, gray_buf_);
    }
    if (is_low_light && frame_counter_ % 60 == 0) {
        LOGI("LOW_LIGHT: brightness=%.2f -> dead reckoning mode", frame_brightness);
    }

    // ── 1.b Motion-blur gate (Plan Step 5) ───────────────────────────────────
    // Compute variance of Laplacian on the centre crop. When the score drops
    // below BLUR_VAR_THRESH we still propagate IMU + run ZUPT (single-frame
    // dead reckoning is fine), but skip the EKF visual measurement updates
    // (geometric verification, SLAM-feature update, MSCKF processLostFeatures,
    // ORB reloc trigger) so the filter does not consume high-noise residuals.
    // measureBlur returning -1 means "unusable input" — treat as not-blurry
    // so we never silently skip on a degenerate buffer.
    const double blur_var       = measureBlur(gray_buf_);
    const bool   frame_is_blurry = (blur_var >= 0.0 && blur_var < BLUR_VAR_THRESH);
    if (frame_is_blurry) {
        const int prev_streak = blur_skipped_streak_;
        blur_skipped_streak_++;
        // Every blurry frame is a frame the pipeline skipped; record it.
        navsight::eventCounters().blur_total_skip_frames.fetch_add(
            1, std::memory_order_relaxed);
        // Rate-limit: log only on entry to a blur burst (avoids 30 lines/s
        // during a 1 s head-turn). The exit transition is logged in the
        // else branch below when a non-trivial streak ends.
        if (prev_streak == 0) {
            LOGI("BLUR: enter var=%.1f thresh=%.1f", blur_var, BLUR_VAR_THRESH);
            navsight::eventCounters().blur_enter_events.fetch_add(
                1, std::memory_order_relaxed);
        }
    } else {
        if (blur_skipped_streak_ > 0) {
            LOGI("BLUR: exit after %d frames", blur_skipped_streak_);
        }
        blur_skipped_streak_ = 0;
    }
    // Keep the `frame_blurry_`-named local available to existing call sites
    // below without renaming every use (the trailing-underscore form was a
    // lint nit; the rename above is the canonical one).
    const bool frame_blurry_ = frame_is_blurry;

    // ── 2. Camera intrinsics + lens corrector ────────────────────────────────
    double fx_use = (fx_ > 0) ? fx_ : 0.7 * width;
    double fy_use = (fy_ > 0) ? fy_ : fx_use;
    double cx_use = (cx_ > 0) ? cx_ : width  / 2.0;
    double cy_use = (cy_ > 0) ? cy_ : height / 2.0;
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx_use,0,cx_use, 0,fy_use,cy_use, 0,0,1);

    if (!lens_.isReady()) {
        lens_.setIntrinsics(fx_use, fy_use, cx_use, cy_use);
    }

    // ── 3. Initialization Check (InertialInitializer) ───────────────────────
    if (!initialized_) {
        return out;
    }

    // ── 3.1 Apply magnetometer heading ONCE at startup (per project policy)
    // Mag is a one-shot bootstrap. Sets global_R_ as a seed for the eventual
    // ekf_.initializeFull below (no continuous mag fusion — see project rules).
    if (!heading_initialized_ && imu.hasMagHeading() && !ekf_.isFullInitialized()) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        float mag_yaw = imu.getMagHeading();
        double cy = std::cos(mag_yaw), sy = std::sin(mag_yaw);
        global_R_ = (cv::Mat_<double>(3,3) <<
            cy, -sy, 0,
            sy,  cy, 0,
             0,   0, 1);
        scalar_heading_ = mag_yaw;
        heading_initialized_ = true;
        LOGI("Tracker: Initial heading bootstrap from magnetometer: %.1f deg",
             mag_yaw * 180.0 / M_PI);
    }

    // Apply any pending setInitialHeading() — bootstrap-only as well.
    if (pending_init_heading_set_ && !ekf_.isFullInitialized() &&
        imu.isOrientationInitialized()) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        double az = pending_init_heading_;
        double cy = std::cos(az), sy = std::sin(az);
        global_R_ = (cv::Mat_<double>(3,3) <<
            cy, -sy, 0,
            sy,  cy, 0,
             0,   0, 1);
        scalar_heading_ = az;
        pending_init_heading_set_ = false;
        LOGI("Tracker: bootstrap init heading=%.1f deg",
             az * 180.0 / M_PI);
    }

    // ── 4. First-frame: grid-based feature detection ─────────────────────────
    cv::Mat current_prev_gray;
    int64_t current_prev_ts = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || prev_pts_.empty()) {
            feature_mgr_.detectGridFeatures(gray_buf_, prev_pts_,
                                             MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            feature_ages_.assign(prev_pts_.size(), 0);
            feature_ids_ = feature_mgr_.assignIds(static_cast<int>(prev_pts_.size()));
            gray_buf_.copyTo(prev_gray_);
            prev_timestamp_ns_ = timestamp_ns;
            initialized_ = true;
            // Initialize full MSCKF state if not yet done
            if (!ekf_.isFullInitialized()) {
                auto gb = imu.getGyroBias();
                ekf_.initializeFull(global_R_, gb, cv::Point3f(0,0,0));
            }
            feature_mgr_.storeKeyframe(gray_buf_, prev_pts_, timestamp_ns, 0,
                                      scalar_heading_,
                                      cv::Point3f(0, 0, 0));
            // Plan Step 4 (ADR-010): mirror the keyframe into the ORB
            // descriptor ring buffer for relocalization.
            feature_mgr_.storeKeyframeDescriptors(
                static_cast<uint64_t>(frame_counter_),
                static_cast<double>(timestamp_ns),
                gray_buf_, prev_pts_, feature_ids_);
            LOGI("processFrame: first frame, grid-detected %zu features", prev_pts_.size());
            return out;
        }
        current_prev_gray = prev_gray_;
        current_prev_pts_buf_ = prev_pts_;
        current_prev_ts = prev_timestamp_ns_;
    }

    frame_counter_++;

    // Step 3 Fusion: time-update the scale fuser once per frame so its
    // variance grows when no observer fires (allowing future observers to
    // pull it more strongly). Δt clamped — first frame has no baseline.
    if (last_scale_predict_ns_ != 0) {
        double dt_predict = (timestamp_ns - last_scale_predict_ns_) * 1e-9;
        if (dt_predict > 0.0 && dt_predict < 5.0) {
            scale_fuser_.predict(dt_predict);
        }
    }
    last_scale_predict_ns_ = timestamp_ns;

    // ── 4. IMU integration with TEMPORAL CALIBRATION ────────────────────────
    double td = ekf_.getTimeOffset();
    int64_t td_ns = static_cast<int64_t>(td * 1e9);
    
    // Shift integration window by estimated latency
    PreintegratedMeasurement imu_delta = imu.integrate(current_prev_ts + td_ns, timestamp_ns + td_ns);
    
    // Propagate full EKF state with IMU preintegration
    if (ekf_.isFullInitialized()) {
        ekf_.propagateIMU(imu_delta.deltaR, imu_delta.deltaV, imu_delta.deltaP,
                          imu_delta.dt, imu_delta.cov,
                          imu_delta.J_R_bg, imu_delta.J_V_bg, imu_delta.J_V_ba,
                          imu_delta.J_P_bg, imu_delta.J_P_ba);
    }

    double gyro_norm = 0.0;
    {
        cv::Mat rv;
        cv::Rodrigues(imu_delta.deltaR, rv);
        if (imu_delta.dt > 0) gyro_norm = cv::norm(rv) / imu_delta.dt;
    }

    // ── Statistical Zero-Velocity Detection (ZUPT) ──
    // (Moved below mean_flow calculation)
    bool is_static = false;
    // Plan Step 5: dual-gate pure-rotation detector. The gyro-magnitude gate
    // is the necessary condition (rotation is what produces high gyro), but
    // it is not sufficient — a sustained scooter "looking around" still
    // shows large flow because the rider is also translating, so the
    // existing single-gate version misclassified those frames as
    // "pure rotation" and (correctly) suppressed the visual yaw update but
    // (incorrectly) also suppressed the scale / SLAM updates. We tighten by
    // requiring the optical-flow direction distribution to also look like
    // rotation (Rayleigh resultant length R/N below FLOW_RAYLEIGH_REJECT
    // means flow directions are approximately uniform on the unit circle,
    // i.e. they are NOT pointing away from a focus-of-expansion). Computed
    // below after the per-feature flow loop; until then we use the
    // gyro-only candidate, which is preserved as the conservative default
    // for code paths that fire BEFORE the Rayleigh stage (none today, but
    // explicit for future maintenance).
    bool gyro_pure_rotation_candidate = (gyro_norm > GYRO_ROT_ONLY_THRESH);
    bool is_pure_rotation = gyro_pure_rotation_candidate;

    // ── 5. Optical flow tracking (TrackKLT) ──────────────────────────────────
    // Plan Step 5: adaptive KLT search-window sizing from gyro magnitude.
    // Expected per-frame pixel displacement of a stationary point under the
    // observed angular rate: expected_disp_px = focal * |gyro| * dt. KLT
    // loses tracks when its window is < ~1.5× the inter-frame displacement;
    // growing the window to 41 px covers ~3 rad/s on a 30 Hz frame at
    // f≈525 px (typical phone telephoto-ish narrow FOV). Steady-state walk
    // (gyro ≈ 0) clamps to 21 px so the cost matches pre-Step-5 builds.
    const double dt_klt = (imu_delta.dt > 0.0) ? imu_delta.dt : 0.0;
    const double expected_disp_px = fx_use * gyro_norm * dt_klt;
    int win_sz = static_cast<int>(2.0 * expected_disp_px + 11.0);
    win_sz = std::max(21, std::min(41, win_sz));
    if ((win_sz & 1) == 0) win_sz += 1;  // KLT requires an odd window
    if (win_sz > 21) {
        navsight::eventCounters().klt_adaptive_window_hits.fetch_add(
            1, std::memory_order_relaxed);
        if (frame_counter_ % 30 == 0) {
            LOGI("KLT: adaptive win=%d gyro=%.2f rad/s expected_disp=%.1f px",
                 win_sz, gyro_norm, expected_disp_px);
        }
    }

    int64_t t_klt_start = now_us();
    std::vector<uchar> status;
    next_pts_buf_.clear();
    klt_.track(current_prev_gray, gray_buf_, current_prev_pts_buf_,
               next_pts_buf_, status, imu_delta.deltaR, K, win_sz);
    int64_t t_klt_end = now_us();
    if (frame_counter_ % 30 == 0) {
        LOGI("PERF: section=klt us=%lld n_pts=%zu",
             (long long)(t_klt_end - t_klt_start),
             current_prev_pts_buf_.size());
    }

    // ── 6. Filter valid points + maintain feature ages + IDs ─────────────────
    prev_good_buf_.clear(); next_good_buf_.clear();
    std::vector<int> surviving_ages;
    std::vector<int> surviving_ids;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            prev_good_buf_.push_back(current_prev_pts_buf_[i]);
            next_good_buf_.push_back(next_pts_buf_[i]);
            int age = (i < feature_ages_.size()) ? feature_ages_[i] + 1 : 1;
            surviving_ages.push_back(age);
            int fid = (i < feature_ids_.size()) ? feature_ids_[i] : -1;
            surviving_ids.push_back(fid);
        }
    }
    feature_ages_ = std::move(surviving_ages);
    feature_ids_ = std::move(surviving_ids);

    int tracked = static_cast<int>(next_good_buf_.size());
    int total   = static_cast<int>(current_prev_pts_buf_.size());

    // Two-tier quality:
    // - track_quality: fraction of features successfully tracked (for gate decisions)
    // - quality: weighted by mature features (for confidence/weighting)
    double track_quality = (total > 0) ? static_cast<double>(tracked) / total : 0.0;
    int mature_count = 0;
    for (int a : feature_ages_) { if (a >= 3) mature_count++; }
    // Quality blends tracking ratio with maturity — avoids zero quality at low FPS
    // where features rarely survive 3 frames
    double maturity_ratio = (tracked > 0) ? static_cast<double>(mature_count) / tracked : 0.0;
    double quality = track_quality * (0.5 + 0.5 * maturity_ratio);

    // In low light, scale quality down (noise tracks inflate it).
    // Don't clamp to zero — let the pipeline still attempt vision with low weight.
    if (is_low_light) quality *= 0.2;

    std::vector<float> tracked_pts_flat;
    tracked_pts_flat.reserve(next_good_buf_.size() * 2);
    for (const auto& pt : next_good_buf_) {
        tracked_pts_flat.push_back(pt.x);
        tracked_pts_flat.push_back(pt.y);
    }

    // ── 7. Mean flow + motion gates ──────────────────────────────────────────
    double mean_flow = 0.0;
    if (!prev_good_buf_.empty()) {
        for (size_t i = 0; i < prev_good_buf_.size(); ++i) {
            double dx = next_good_buf_[i].x - prev_good_buf_[i].x;
            double dy = next_good_buf_[i].y - prev_good_buf_[i].y;
            mean_flow += std::sqrt(dx * dx + dy * dy);
        }
        mean_flow /= static_cast<double>(prev_good_buf_.size());
    }
    bool motion_blur = (mean_flow > MAX_FLOW_PX);
    bool sufficient_motion = (mean_flow >= MIN_FLOW_PX) && !motion_blur;
    bool has_parallax = (mean_flow >= MIN_PARALLAX_PX) && !motion_blur;

    // ── Plan Step 5: Rayleigh dual-gate for pure rotation ────────────────────
    // The mean resultant length R/N of the per-feature flow direction unit
    // vectors. Rotation produces an approximately uniform direction
    // distribution (R/N → 0); translation produces directions that converge
    // on the focus-of-expansion (R/N → 1). FLOW_RAYLEIGH_REJECT (=0.3)
    // gates the rotation classification — only when BOTH gyro magnitude is
    // high AND flow direction is uniform do we declare pure rotation. If
    // we can't compute a meaningful R/N (too few flow vectors or all-zero
    // flow), we fall back to the gyro-only result so this gate can only
    // ADD restrictions, never loosen the existing detector. Concern: on a
    // legitimate scooter ride forward where flow IS concentrated toward
    // the FoE, R/N stays high → the gate keeps `is_pure_rotation` FALSE,
    // which is the correct behaviour (we want scale / SLAM updates to
    // proceed during forward motion).
    // Size invariant: the FB-check + boundary filter earlier in the frame
    // can shrink next_good_buf_ below prev_good_buf_, so iterating
    // prev_good_buf_ while indexing next_good_buf_[i] would read OOB. We
    // require the two buffers to be co-indexed before computing R/N. If
    // they ever diverge mid-frame (a future refactor), the dual gate
    // silently falls back to the gyro-only result rather than UB-reading
    // a stale slot.
    if (gyro_pure_rotation_candidate
        && prev_good_buf_.size() >= 10
        && next_good_buf_.size() == prev_good_buf_.size()) {
        double sx = 0.0, sy = 0.0;
        int    n_dirs = 0;
        for (size_t i = 0; i < prev_good_buf_.size(); ++i) {
            const double dx = next_good_buf_[i].x - prev_good_buf_[i].x;
            const double dy = next_good_buf_[i].y - prev_good_buf_[i].y;
            const double mag = std::sqrt(dx * dx + dy * dy);
            if (mag < 1e-3) continue;  // sub-pixel noise carries no direction
            sx += dx / mag;
            sy += dy / mag;
            n_dirs++;
        }
        if (n_dirs >= 10) {
            const double R   = std::sqrt(sx * sx + sy * sy);
            const double r_n = R / static_cast<double>(n_dirs);
            // Concentrated flow (R/N >= reject threshold) → translation
            // present → relax the gyro classification back to "not pure
            // rotation". Uniform flow (R/N < reject threshold) → confirm
            // pure rotation.
            is_pure_rotation = (r_n < FLOW_RAYLEIGH_REJECT);
            if (is_pure_rotation) {
                navsight::eventCounters().rot_gate_pure_rot_confirmed.fetch_add(
                    1, std::memory_order_relaxed);
            }
            if (frame_counter_ % 30 == 0) {
                LOGI("ROT_GATE: gyro=%.2f R/N=%.2f thresh=%.2f -> pure_rot=%d",
                     gyro_norm, r_n, FLOW_RAYLEIGH_REJECT,
                     is_pure_rotation ? 1 : 0);
                navsight::eventCounters().rot_gate_log_lines.fetch_add(
                    1, std::memory_order_relaxed);
            }
        }
    }

    // ── Phase 6: Time-offset cross-correlation warmup ───────────────────────
    if (!td_warmup_done_ && frame_counter_ <= TD_WARMUP_FRAMES) {
        // Buffer optical flow rate and gyro rate for cross-correlation
        double flow_rate = mean_flow;  // pixels/frame (proportional to angular rate)
        double gyro_rate = gyro_norm;  // rad/s from IMU
        td_warmup_buf_.push_back({flow_rate, gyro_rate, timestamp_ns});

        if (static_cast<int>(td_warmup_buf_.size()) >= TD_WARMUP_FRAMES) {
            // Cross-correlate flow_rate with gyro_rate at lags [-50ms, +50ms]
            // Find the lag that maximizes normalized cross-correlation
            double best_corr = -1e9;
            int best_lag_idx = 0;

            // Compute mean and std of both signals
            double mean_f = 0, mean_g = 0;
            for (const auto& s : td_warmup_buf_) { mean_f += s.flow_rate; mean_g += s.gyro_rate; }
            mean_f /= td_warmup_buf_.size();
            mean_g /= td_warmup_buf_.size();

            double std_f = 0, std_g = 0;
            for (const auto& s : td_warmup_buf_) {
                std_f += (s.flow_rate - mean_f) * (s.flow_rate - mean_f);
                std_g += (s.gyro_rate - mean_g) * (s.gyro_rate - mean_g);
            }
            std_f = std::sqrt(std_f / td_warmup_buf_.size());
            std_g = std::sqrt(std_g / td_warmup_buf_.size());

            if (std_f > 1e-6 && std_g > 1e-6) {
                int N = static_cast<int>(td_warmup_buf_.size());
                // Try lags from -3 to +3 frames (~[-100ms, +100ms] at 30fps)
                for (int lag = -3; lag <= 3; lag++) {
                    double corr = 0;
                    int count = 0;
                    for (int i = 0; i < N; i++) {
                        int j = i + lag;
                        if (j >= 0 && j < N) {
                            corr += (td_warmup_buf_[i].flow_rate - mean_f)
                                  * (td_warmup_buf_[j].gyro_rate - mean_g);
                            count++;
                        }
                    }
                    if (count > 0) {
                        corr /= (count * std_f * std_g);
                        if (corr > best_corr) {
                            best_corr = corr;
                            best_lag_idx = lag;
                        }
                    }
                }

                // Convert lag in frames to time offset in seconds
                // Average frame interval from buffer
                double total_dt = (td_warmup_buf_.back().ts_ns - td_warmup_buf_.front().ts_ns) * 1e-9;
                double avg_frame_dt = total_dt / (N - 1);
                double estimated_td = best_lag_idx * avg_frame_dt;

                // Clamp to reasonable range [-50ms, +50ms]
                estimated_td = std::max(-0.05, std::min(0.05, estimated_td));

                // Warm-start the EKF time offset — let it refine from here
                ekf_.setTimeOffset(estimated_td);
                LOGI("TD_WARMUP: best_lag=%d frames (%.1fms), corr=%.3f, setting td=%.3fms",
                     best_lag_idx, estimated_td * 1000.0, best_corr, estimated_td * 1000.0);
            }

            td_warmup_done_ = true;
            td_warmup_buf_.clear();  // Free memory
            td_warmup_buf_.shrink_to_fit();
        }
    }

    // ZUPT: Statistical stationary detection (OpenVINS style)
    is_static = zupt_detector_.is_stationary(imu.getAccelBuffer(), imu.getGyroBuffer(), mean_flow);
    
    // Safety overrides (relaxed: KLT has ~0.5-1.5px noise even stationary)
    if (mean_flow > 2.5) is_static = false;
    // Don't trust step speed to break ZUPT while rotating fast — phantom steps
    // during in-place rotation used to un-freeze translation and produce arcs.
    if (is_static && gyro_norm < 0.8 && imu.getStepInfo().speed_mps > 0.3) is_static = false;
    if (is_static) {
        ekf_.updateZUPT();
        // Refine gyro bias while stationary — prevents heading drift during pauses
        imu.refineGyroBiasDuringZUPT();
    }

    // ── 8. Lens undistortion + Essential matrix + pose ───────────────────────
    bool pose_valid = false;
    bool used_fallback = false;
    bool translation_degenerate = false;
    int inlier_count_out = 0;
    cv::Mat R_vo, t_vo, R_fused;
    double estimatedScale = 1.0;

    if (frame_counter_ % 30 == 0) {
        LOGI("GATES: flow=%.2f blur=%d motion=%d parallax=%d static=%d rot=%d pts=%d gyro=%.3f lowlight=%d",
             mean_flow, motion_blur, sufficient_motion, has_parallax, is_static,
             is_pure_rotation, tracked, gyro_norm, is_low_light);
    }

    int64_t t_pose_start = now_us();
    // Plan Step 4 (ADR-010): track whether the geometric-verification path
    // actually executed this frame. The relocalization streak counter
    // advances ONLY on frames where we attempted verification and got a
    // weak inlier count — frames skipped due to is_static / no-parallax
    // do not penalise the streak (they are legitimately scale-blind).
    bool geo_verification_attempted = false;
    // Plan Step 5: motion-blur skip — visual measurement updates would
    // inject high-noise residuals into the EKF, so this entire section
    // (geometric verification + R_vo fusion + scale observers + MSCKF
    // block at section 11.1 + SLAM block at section 11.1b) is gated on
    // !frame_blurry_. propagateIMU + ZUPT have already run above; the
    // filter still advances on IMU dead reckoning for the blurred frame.
    // Because geo_verification_attempted stays false, the ORB reloc
    // trigger streak (section 7.x below) is also naturally suppressed.
    if (sufficient_motion && has_parallax && !is_static && tracked >= 8
        && !frame_blurry_) {
        geo_verification_attempted = true;
        // Undistort matched points before geometric estimation
        std::vector<cv::Point2f> prev_ud = prev_good_buf_;
        std::vector<cv::Point2f> next_ud = next_good_buf_;
        lens_.undistortMatchedPoints(prev_ud, next_ud);

        std::vector<uchar> status_verification(prev_ud.size(), 1);
        bool verification_ok = klt_.geometricVerification(prev_ud, next_ud, status_verification,
                                                          K, R_vo, t_vo, inlier_count_out);

        if (verification_ok) {
            // Check SVD condition for translation degeneracy on the recovered E-matrix context
            // In OpenVINS, we often check if the translation is significant.
            double inlier_ratio = static_cast<double>(inlier_count_out) / tracked;

            // Translation degeneracy: check if t_vo is too small (pure rotation)
            // NOTE: Do NOT use SVD condition of E — essential matrix is always rank 2
            // by definition (σ,σ,0), so condition number is always infinite.
            double t_norm = t_vo.empty() ? 0.0 : cv::norm(t_vo);
            if (t_norm < 0.001) translation_degenerate = true;
            // Also check: if flow is mostly explained by rotation (low parallax)
            if (mean_flow < 1.5 && t_norm < 0.01) translation_degenerate = true;
            double svd_cond = 0.0;  // kept for logging

            if (inlier_count_out >= MIN_INLIERS && inlier_ratio >= MIN_INLIER_RATIO) {
                // ── Triangulation (only when translation is reliable) ───
                if (!translation_degenerate && !t_vo.empty()) {
                    cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);
                    cv::Mat Rt(3, 4, CV_64F);
                    R_vo.copyTo(Rt(cv::Range(0,3), cv::Range(0,3)));
                    t_vo.copyTo(Rt(cv::Range(0,3), cv::Range(3,4)));
                    cv::Mat P2 = K * Rt;

                    cv::Mat pts4d;
                    cv::triangulatePoints(P1, P2, prev_ud, next_ud, pts4d);
                    points_3d_current_.clear();

                    // Phase 3: Reprojection error gating (chi-squared, 2 DOF, 95% = 5.991)
                    int reproj_outliers = 0;
                    for (int i = 0; i < pts4d.cols; ++i) {
                        cv::Mat p = pts4d.col(i);
                        double w = p.at<double>(3);
                        if (w > 1e-6) {
                            double X = p.at<double>(0)/w;
                            double Y = p.at<double>(1)/w;
                            double Z = p.at<double>(2)/w;

                            // Reproject into second camera frame
                            cv::Mat pt3 = (cv::Mat_<double>(3,1) << X, Y, Z);
                            cv::Mat proj = K * (R_vo * pt3 + t_vo);
                            double pz = proj.at<double>(2);
                            if (pz > 1e-6) {
                                double px = proj.at<double>(0) / pz;
                                double py = proj.at<double>(1) / pz;
                                double err_sq = (next_ud[i].x - px) * (next_ud[i].x - px)
                                              + (next_ud[i].y - py) * (next_ud[i].y - py);
                                if (err_sq > 5.991) {
                                    reproj_outliers++;
                                    points_3d_current_.emplace_back(0, 0, 0);  // Mark invalid
                                    continue;
                                }
                            }
                            points_3d_current_.emplace_back(X, Y, Z);
                        } else {
                            points_3d_current_.emplace_back(0, 0, 0);
                        }
                    }
                    if (reproj_outliers > 0 && frame_counter_ % 30 == 0) {
                        LOGI("REPROJ_GATE: rejected %d/%d outlier points", reproj_outliers, pts4d.cols);
                    }
                }

                // ── Rotation handling (Step 2, visual production plan) ────
                // Per-frame R_fused below stays gyro-derived (imu_delta.deltaR
                // is already bias-corrected by IMUPreintegrator, lines
                // ~227-229). It drives the legacy global_R_ mirror, which we
                // intentionally keep on the gyro path for now.
                //
                // *Absolute* yaw against the world frame is unobservable from
                // a monocular camera (only gravity + scale give an absolute
                // reference), but the *relative* rotation between two camera
                // poses is fully observable from R_vo. So R_vo is fused into
                // the EKF separately via updateRelativeRotation below for
                // state-level consistency, instead of being discarded.
                R_fused = imu_delta.deltaR.clone();

                // Fuse visual relative rotation into the EKF. R_vo from
                // recoverPose is in the OpenCV camera frame; convert to the
                // body frame via the same self-inverse extrinsic used for
                // the keyframe yaw correction (see line ~1325):
                //     R_b2c = diag(1, -1, -1)   (camera↔body for vertical
                //     phone, rear camera) is symmetric and self-inverse, so
                //     R_vo_body = R_b2c * R_vo * R_b2c.
                // Variance per axis follows the same pixel-noise model used
                // for the keyframe yaw update (Step 2.4 of the inertial plan):
                //     σ_axis = RANSAC_THRESH / (focal * sqrt(N_inliers))
                // Note: pose_valid is set to true at the bottom of this
                // (inlier_count_out >= MIN_INLIERS && inlier_ratio >= MIN_INLIER_RATIO)
                // block, so reaching this point implies pose_valid will be true
                // for the rest of the frame. We gate explicitly on the geometric
                // conditions instead of forward-referencing the flag.
                if (!R_vo.empty() && inlier_count_out >= 12 &&
                    !translation_degenerate && !is_pure_rotation &&
                    ekf_.isFullInitialized()) {
                    int prev_clone_id = ekf_.getLatestCloneId();
                    if (prev_clone_id >= 0) {
                        // Step 8b: use EKF-maintained R_bc (body→camera) instead
                        // of the previously hardcoded diag(1,-1,-1). The EKF refines
                        // R_bc_ online; getExtrinsicsRotation() returns the current
                        // best estimate. Since R_bc is body→camera (p_cam=R_bc*p_body)
                        // and is not necessarily self-inverse, the similarity
                        // transform to re-express R_vo in body frame is:
                        //   R_vo_body = R_bc^T * R_vo * R_bc
                        // (R_bc^T = R_bc^{-1} because R_bc is a rotation matrix).
                        const cv::Matx33d R_bc_mx = ekf_.getExtrinsicsRotation();
                        cv::Mat R_bc_cv(3, 3, CV_64F);
                        for (int ri = 0; ri < 3; ri++)
                            for (int ci = 0; ci < 3; ci++)
                                R_bc_cv.at<double>(ri, ci) = R_bc_mx(ri, ci);
                        cv::Mat R_vo_body = R_bc_cv.t() * R_vo * R_bc_cv;
                        double focal = K.at<double>(0, 0);
                        double sigma_axis = (focal > 1e-6)
                            ? (RANSAC_THRESH /
                               (focal * std::sqrt(static_cast<double>(inlier_count_out))))
                            : 1e-2;
                        double sigma_axis_sq = sigma_axis * sigma_axis;
                        ekf_.updateRelativeRotation(R_vo_body, sigma_axis_sq,
                                                    prev_clone_id);
                    }
                }

                if (frame_counter_ % 90 == 0) {
                    LOGI("POSE: transDegen=%d q=%.2f rotation=gyro_only(mirror)+vo(ekf) scale=%.4f",
                         translation_degenerate ? 1 : 0, quality, scale_fuser_.scale());
                }

                // ── Scale estimation from steps (median bootstrap + EMA) ─
                // Phase 1: Collect first N observations, take median (avoids bad init)
                // Phase 2: EMA refinement with outlier rejection relative to median
                if (!translation_degenerate) {
                    double vo_dist = cv::norm(t_vo);
                    int64_t dt_ns_frame = timestamp_ns - current_prev_ts;
                    double dt_sec = dt_ns_frame * 1e-9;

                    if (vo_dist > 0.01 && dt_sec > 0.01 && dt_sec < 1.5
                        && !is_pure_rotation && !is_static) {
                        auto si = imu.getStepInfo();
                        if (frame_counter_ % 30 == 0) {
                            LOGI("SCALE_GATE: vo=%.4f dt=%.3f speed=%.3f rot=%d static=%d",
                                 vo_dist, dt_sec, si.speed_mps, is_pure_rotation, is_static);
                        }
                        if (si.speed_mps > 0.3) {
                            double step_disp = si.speed_mps * dt_sec;
                            double obs_scale = step_disp / vo_dist;

                            // Clamp to sane range
                            obs_scale = std::max(0.005, std::min(10.0, obs_scale));

                            if (scale_obs_count_ < SCALE_BOOTSTRAP_COUNT) {
                                // Phase 1: Bootstrap — collect observations.
                                // Initial fuser variance is large (1.0); we
                                // seed it once we have a stable median so
                                // the first cold-start estimate isn't yanked
                                // around by individual noisy samples.
                                scale_bootstrap_buf_.push_back(obs_scale);
                                scale_obs_count_++;

                                if (scale_obs_count_ >= SCALE_BOOTSTRAP_COUNT) {
                                    std::vector<double> sorted = scale_bootstrap_buf_;
                                    std::sort(sorted.begin(), sorted.end());
                                    double median = sorted[sorted.size() / 2];
                                    // Seed fuser with bootstrap median and a
                                    // moderate variance (range/2)² so later
                                    // observers can still steer it.
                                    double range = sorted.back() - sorted.front();
                                    double seed_var = std::max(1e-4, (range * 0.5) * (range * 0.5));
                                    scale_fuser_.reset(median, seed_var);

                                    estimatedScale = scale_fuser_.scale();
                                    LOGI("SCALE_BOOTSTRAP: median=%.4f var=%.5f from %d samples (range %.4f-%.4f)",
                                         median, seed_var, SCALE_BOOTSTRAP_COUNT, sorted.front(), sorted.back());
                                    scale_bootstrap_buf_.clear();
                                    scale_bootstrap_buf_.shrink_to_fit();
                                }
                            } else {
                                // Phase 2: Observer A feeds (z, r) into the
                                // 1-D Kalman fuser. Variance from step-period
                                // jitter — smooth gait → tight r → strong
                                // pull. Outlier rejection retained as a
                                // safety net beyond what variance alone gates.
                                double current_smooth = scale_fuser_.scale();
                                if (obs_scale > 2.5 * current_smooth ||
                                    obs_scale < current_smooth / 2.5) {
                                    if (frame_counter_ % 30 == 0) {
                                        LOGI("SCALE: REJECTED obs=%.4f smooth=%.4f (>2.5× ratio)",
                                             obs_scale, current_smooth);
                                    }
                                } else {
                                    // Convert step-period variance σ²_T to a
                                    // scale variance: speed = stride / T, so
                                    // σ_v / v ≈ σ_T / T. obs_scale scales
                                    // linearly with speed, so its CoV equals
                                    // the speed CoV plus VO uncertainty.
                                    double mean_period = (si.speed_mps > 0.0 && si.stride_length_m > 0.0)
                                        ? (si.stride_length_m / si.speed_mps) : 0.5;
                                    double period_var = (si.step_period_variance_s2 > 0.0)
                                        ? si.step_period_variance_s2 : 0.0025; // default ~5% CoV
                                    double cov_period = std::sqrt(period_var) / std::max(0.1, mean_period);
                                    // Add a 10% VO baseline uncertainty in quadrature.
                                    double cov_total = std::sqrt(cov_period * cov_period + 0.01);
                                    double r_var = std::max(1e-6, (cov_total * obs_scale) * (cov_total * obs_scale));

                                    bool accepted = scale_fuser_.update(obs_scale, r_var);
                                    if (accepted) scale_obs_count_++;

                                    estimatedScale = scale_fuser_.scale();

                                    if (frame_counter_ % 30 == 0) {
                                        LOGI("SCALE: obs=%.4f smooth=%.4f vo=%.4f step_d=%.3f r=%.5f P=%.5f count=%d",
                                             obs_scale, scale_fuser_.scale(), vo_dist, step_disp,
                                             r_var, scale_fuser_.variance(), scale_obs_count_);
                                    }
                                }
                            }
                        }
                    }
                }

                // Phase 8 (gravity-aided scale) was disabled because IMU
                // preintegration ΔP was dominated by gravity-subtraction
                // errors when attitude was wrong. Madgwick (Step 1) produces
                // clean attitude, so the Hesch/Martinelli closed-form VI
                // scale path is now well-conditioned and re-enabled here.
                //
                // Step 3 Observer C: record one keyframe pair per frame
                // (R_w_b = global_R_ at frame start; t_vis_body = unit-norm
                // recoverPose translation; Δp/Δv/dt from imu.integrate over
                // the frame). Every OBSERVER_C_SOLVE_INTERVAL pairs, run
                // solve() and feed (s, var) into scale_fuser_ if healthy.
                if (!is_pure_rotation && !is_static
                    && imu_delta.dt > 0.005 && imu_delta.dt < 1.0
                    && !t_vo.empty() && cv::norm(t_vo) > 0.5) {
                    ScaleEstimatorVI::KeyframePair kp;
                    kp.R_w_b = global_R_.clone();
                    kp.dt = imu_delta.dt;
                    kp.t_vis_body = cv::Vec3d(t_vo.at<double>(0),
                                              t_vo.at<double>(1),
                                              t_vo.at<double>(2));
                    kp.delta_p_body = cv::Vec3d(imu_delta.deltaP.at<double>(0),
                                                imu_delta.deltaP.at<double>(1),
                                                imu_delta.deltaP.at<double>(2));
                    kp.delta_v_body = cv::Vec3d(imu_delta.deltaV.at<double>(0),
                                                imu_delta.deltaV.at<double>(1),
                                                imu_delta.deltaV.at<double>(2));
                    scale_estimator_vi_.addKeyframePair(kp);
                    observer_c_pair_count_++;

                    if (observer_c_pair_count_ % OBSERVER_C_SOLVE_INTERVAL == 0
                        && scale_estimator_vi_.size() >= ScaleEstimatorVI::MIN_PAIRS) {
                        double s_obs = 0.0, var_obs = 0.0;
                        if (scale_estimator_vi_.solve(s_obs, var_obs)
                            && std::isfinite(s_obs) && std::isfinite(var_obs)
                            && s_obs > 0.01 && s_obs < 10.0 && var_obs > 0.0) {
                            // Inflate variance: per-frame unit-norm visual
                            // translations are noisy and Hesch/Martinelli
                            // assumes consistent scale across pairs. Floor
                            // keeps Observer C from dominating the fuser.
                            double r_var = std::max(var_obs, 0.04);
                            if (scale_fuser_.update(s_obs, r_var)) {
                                if (frame_counter_ % 30 == 0) {
                                    LOGI("OBS_C: s=%.4f var=%.5f -> "
                                         "fuser_s=%.4f fuser_P=%.5f",
                                         s_obs, var_obs, scale_fuser_.scale(),
                                         scale_fuser_.variance());
                                }
                            }
                        }
                    }
                }

                // Depth-based scale constraint (MiDaS): runs every ~30 frames (~1Hz)
                // Uses floor features + camera height to estimate absolute scale
                if (frame_counter_ % 30 == 0 && !points_3d_current_.empty()) {
                    applyDepthScaleConstraint(next_good_buf_, points_3d_current_,
                                              width, height, imu);
                }

                pose_valid = true;
            }
        }
    }

    // ── 7.x ORB relocalization debounce + trigger (Plan Step 4 / ADR-010) ──
    // Maintain low_inlier_streak_: increments on attempted-but-degraded
    // geometric verification, resets on healthy frames or skipped frames.
    // When the streak hits RELOC_TRIGGER_FRAMES, run ORB descriptor
    // matching against the recent keyframe ring buffer. On accept, the
    // helper re-adopts the matched keyframe's feature ids onto current
    // KLT tracks so SLAM/MSCKF lifecycle continuity is preserved.
    if (geo_verification_attempted) {
        if (inlier_count_out < RELOC_LOW_INLIER_BAR) {
            low_inlier_streak_++;
        } else {
            low_inlier_streak_ = 0;
        }
        if (low_inlier_streak_ >= RELOC_TRIGGER_FRAMES) {
            const bool relocked = tryRelocalizeWithORB(gray_buf_, next_good_buf_);
            if (relocked) {
                LOGI("RELOC_ORB: re-adopted feature ids after %d-frame degradation",
                     low_inlier_streak_);
            }
            // Reset regardless — the trigger is "diagnose once, then
            // re-arm". A failing reloc must not pin the trigger high
            // every subsequent frame (BFMatcher cost), and a successful
            // reloc has already restored ids.
            low_inlier_streak_ = 0;
        }
    } else {
        low_inlier_streak_ = 0;
    }

    // Always report current scale (even if no new observation this frame)
    estimatedScale = scale_fuser_.scale();
    double appliedScale = estimatedScale;
    { std::lock_guard<std::mutex> lock(mutex_); appliedScale *= user_scale_correction_; }

    // ── 9. Global pose update (heading-based 2D) ─────────────────────────────
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // ── 9.0 Heading & rotation sourced from EKFState (Step 4) ─────────
        // EKFState owns rotation propagation via propagateIMU and yaw
        // correction via updateGravityAlignedYaw. scalar_heading_ and
        // global_R_ are read-only mirrors refreshed each frame so legacy
        // intra-frame consumers keep working until they are migrated to
        // direct ekf_ accessors.
        double heading;
        if (ekf_.isFullInitialized()) {
            // Heading: source DIRECTLY from Madgwick (imu.getHeading()),
            // not from ekf_.getYaw(). 2026-05-03 BUG FIX: a 10-step
            // forward-and-back sim with a clean 180° turn produced a
            // V-shape trajectory — GPS confirmed the user walked nearly
            // straight there and back, but VIO recorded only ~130° of the
            // 180° turn. EKF's R_GtoI_ is propagated by propagateIMU using
            // imu_delta.deltaR, which integrates gyro through the
            // IMUPreintegrator's bias estimate; Madgwick maintains its own
            // independent gyro bias. When the two diverge, the EKF yaw
            // captures less rotation than Madgwick's. Sourcing heading
            // from Madgwick (the same source that gave the
            // "radar tracks 180° turns correctly" behaviour on the
            // Madgwick merge) restores correct turn capture. Visual yaw
            // correction (updateGravityAlignedYaw) still fires and keeps
            // EKF R_GtoI_ consistent for any downstream consumer; we just
            // don't read EKF yaw for output.
            scalar_heading_ = static_cast<double>(imu.getHeading());
            while (scalar_heading_ >  M_PI) scalar_heading_ -= 2.0 * M_PI;
            while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;
            // global_R_ continues to come from EKF — its rotation is
            // gravity-corrected by the visual yaw update (Bug C fixed
            // earlier), and the only output consumer of global_R_ is the
            // clone storage / camera-pose snapshot, which benefits from
            // the visual correction. Keeping it.
            global_R_ = ekf_.getRotation();
            // global_t_ deliberately NOT mirrored from EKF — see fix
            // comment near the visual position update below.
        } else {
            // Bootstrap-only: before EKF initializeFull, fall back to
            // Madgwick yaw.
            scalar_heading_ = static_cast<double>(imu.getHeading());
            while (scalar_heading_ >  M_PI) scalar_heading_ -= 2.0 * M_PI;
            while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;
        }
        heading = scalar_heading_;

        if (frame_counter_ % 30 == 0) {
            const float m_yaw   = imu.getHeading() * 180.0f / static_cast<float>(M_PI);
            const float m_roll  = imu.getMadgwickRoll()  * 180.0f / static_cast<float>(M_PI);
            const float m_pitch = imu.getMadgwickPitch() * 180.0f / static_cast<float>(M_PI);
            LOGI("HEADING: madgwick_yaw=%.1f° roll=%.1f° pitch=%.1f° -> ekf_heading=%.1f°",
                 m_yaw, m_roll, m_pitch,
                 heading * 180.0 / M_PI);
        }

        // Step 2.3: Madgwick is the heading reference; FEJ is for position only.

        if (is_static) {
            // Translation already frozen (no update)
        } else if (pose_valid && !is_pure_rotation && !translation_degenerate && quality >= 0.15) {
            double disp = appliedScale * cv::norm(t_vo);

            // Sanity cap: max displacement per frame = 2 m/s * dt
            // Walking is ~1.3 m/s, running ~3 m/s. Anything above 2*dt is noise.
            double dt_frame = (timestamp_ns - current_prev_ts) * 1e-9;
            double max_disp = 2.0 * std::max(dt_frame, 0.03);
            if (disp > max_disp) {
                LOGI("DISP_CAP: disp=%.2f capped to %.2f (dt=%.3f)", disp, max_disp, dt_frame);
                disp = max_disp;
            }

            // Accumulate path length. Pre-2026-05-09 this counter only
            // incremented in the PDR fallback branch (line ~1575), missing
            // every visual-VO frame. The result was that
            // event_summary.total_path_dm reported only ~1/3 of the actual
            // walked distance (e.g. 41.8 m for a 122 m walk). Step 7's
            // dynamic translation sigma uses total_path_m_ to grow the
            // chi² gate as drift accumulates, so under-counting tightened
            // the gate prematurely and contributed to chi² rejections.
            total_path_m_ += disp;
            navsight::eventCounters().total_path_dm.store(
                static_cast<long long>(total_path_m_ * 10.0 + 0.5),
                std::memory_order_relaxed);

            // Z-up ENU world frame: X=East, Y=North, Z=Up.
            // heading is CW-positive nav (North=0, East=+π/2). Project
            // horizontal step (disp, in metres) onto world (X, Y) axes:
            //   X (East)  = disp · sin(heading)
            //   Y (North) = disp · cos(heading)
            double dx_world = disp * std::sin(heading);   // +X = East
            double dy_world = disp * std::cos(heading);   // +Y = North
            // dz_world: vertical component. t_vo is in OpenCV camera frame
            // (camera Y points DOWN). World Z is UP. Negate camera-Y to
            // recover world-Z. For flat-ground walking dz ≈ 0; this matters
            // on stairs / slopes / scooter-on-curb.
            double dz_world = 0.0;
            if (!t_vo.empty())
                dz_world = -appliedScale * t_vo.at<double>(1);

            // Tracker-owned position output (see comment at the top of
            // section 9 for why). Also fed to EKF below as a measurement
            // so EKF state stays consistent with what the UI displays.
            global_t_.at<double>(0) += dx_world;
            global_t_.at<double>(1) += dy_world;
            global_t_.at<double>(2) += dz_world;

            // Step 4 Phase C: mirror VO relative-pose update into EKFState
            // with refined variance. updateRelativePose constrains
            // (p_current - p_prev_clone) toward the visual delta. Latest clone
            // id is the previous frame (current-frame addClone happens below).
            //
            // σ_t² has three sources:
            //   1. Visual reprojection: ~5% of displacement (RANSAC inliers)
            //   2. Scale uncertainty: scale_fuser_.var() scales with disp²
            //   3. Floor: 1cm to prevent over-tight fusion when disp~0
            if (ekf_.isFullInitialized()) {
                int prev_clone_id = ekf_.getLatestCloneId();
                if (prev_clone_id >= 0) {
                    cv::Mat t_world_metric = (cv::Mat_<double>(3, 1)
                        << dx_world, dy_world, dz_world);
                    double sigma_visual = 0.05 * disp;
                    double scale_var = scale_fuser_.variance();
                    double sigma_scale = std::sqrt(std::max(0.0, scale_var)) * cv::norm(t_vo);
                    double sigma_floor = 0.01;
                    double var_t = sigma_visual * sigma_visual
                                 + sigma_scale * sigma_scale
                                 + sigma_floor * sigma_floor;
                    ekf_.updateRelativePose(t_world_metric, prev_clone_id, var_t);
                }
            }
        } else if (!is_static) {
            used_fallback = true;
            auto si = imu.getStepInfo();
            double dt_s = (timestamp_ns - current_prev_ts) * 1e-9;
            double since = (si.last_step_ns > 0)
                ? (timestamp_ns - si.last_step_ns) * 1e-9
                : std::numeric_limits<double>::infinity();

            // Update last known step speed when steps are active
            if (si.speed_mps > 0.2 && since <= 1.0) {
                last_step_speed_ = si.speed_mps;
                last_step_speed_ns_ = timestamp_ns;
            }

            // Use current step speed, or interpolate from last known speed
            double speed = 0.0;
            if (since <= 1.5 && si.speed_mps > 0.2) {
                speed = si.speed_mps;
            } else if (last_step_speed_ > 0.2) {
                double age = (timestamp_ns - last_step_speed_ns_) * 1e-9;
                if (age <= 2.5 && mean_flow >= 0.3) {
                    // Decay speed over the gap period
                    speed = last_step_speed_ * std::max(0.0, 1.0 - age / 3.0);
                }
            }

            // BUG FIX (V-shape / rotate-in-place): don't advance position from
            // step speed while the phone is rotating fast (>~46°/s). Phantom
            // steps that slip through the IMU gate must not translate the
            // global pose along a sweeping heading, or the path traces an arc
            // instead of a sharp turnaround.
            if (speed > 0.1 && gyro_norm < 0.8) {
                double d = std::min(std::min(speed, 2.0) * dt_s, 1.0 * dt_s);
                total_path_m_ += d;
                navsight::eventCounters().total_path_dm.store(
                    static_cast<long long>(total_path_m_ * 10.0 + 0.5),
                    std::memory_order_relaxed);
                // Z-up ENU world: project step displacement onto X (East), Y (North).
                double dx_step = d * std::sin(heading);   // +X = East
                double dy_step = d * std::cos(heading);   // +Y = North

                // Tracker-owned position output (visual-degenerate fallback).
                // Same rationale as the visual path above: EKF position is
                // unreliable as a display source; Tracker integrates here
                // and feeds the EKF as a measurement for state consistency.
                global_t_.at<double>(0) += dx_step;
                global_t_.at<double>(1) += dy_step;

                // Step 4 Phase C: mirror PDR fallback step into EKFState
                // with refined per-step variance.
                // σ ≈ 5cm floor + 10% of step distance:
                //   dt at 30 Hz ⇒ d ~3-7cm/frame, σ ~5-6cm, var ~3-4e-3
                //   dt at 1 Hz  ⇒ d ~30-70cm,    σ ~8-12cm, var ~1-2e-2
                if (ekf_.isFullInitialized()) {
                    double sigma_step = 0.05 + 0.10 * d;
                    ekf_.updatePDRStep(dx_step, dy_step, sigma_step * sigma_step);
                }
            }
        }

        // ── 9.1 FEJ & MSCKF: Store Camera Clone ──
        // Clone the EKF's IMU-state pose, composing R_bc * R_GtoI so that
        // clones store true world→camera (matching the field name R_GtoC).
        //
        // Convention bridge (NavSight 2026-05-09 Step 7 fix): every reader of
        // clone.R_GtoC — MSCKF projection at EKFState.cpp:1642, SLAM-feature
        // anchor inverse-depth at EKFState.cpp:1390/1640, loop-closure target
        // at Tracker.cpp:3549 — uses the formula `p_C = clone_R · (p_world - p_clone)`,
        // which only produces a camera-frame point if clone_R is world→camera.
        // Pre-fix the caller passed R_GtoI (world→body), and the EKF's R_bc
        // estimator drifted to absorb the missing factor (Step 8b drift to
        // ~90° within 30s on every walk). Composing R_bc here pre-bakes the
        // extrinsic, so live R_bc no longer factors in projection — H_bc
        // is disabled and R_bc stays at its physical fixed-mount value
        // (read from Android SENSOR_ORIENTATION via SensorRepository.kt:285).
        cv::Mat clone_R_GtoI = ekf_.isFullInitialized() ? ekf_.getRotation() : global_R_;
        cv::Mat clone_p      = ekf_.isFullInitialized() ? ekf_.getPosition() : global_t_;
        const cv::Matx33d R_bc_for_clone = ekf_.getExtrinsicsRotation();
        cv::Mat R_bc_mat(3, 3, CV_64F);
        for (int rr = 0; rr < 3; ++rr)
            for (int cc = 0; cc < 3; ++cc)
                R_bc_mat.at<double>(rr, cc) = R_bc_for_clone(rr, cc);
        cv::Mat clone_R = R_bc_mat * clone_R_GtoI;  // world→camera
        ekf_.addClone(clone_R, clone_p, timestamp_ns);
        int clone_id = ekf_.getLatestCloneId();

        // Record MSCKF observations: feature pixels in normalized coordinates
        if (clone_id >= 0 && !next_good_buf_.empty()) {
            for (size_t i = 0; i < next_good_buf_.size() && i < feature_ids_.size(); i++) {
                if (feature_ids_[i] >= 0) {
                    // Convert to normalized coordinates for MSCKF
                    cv::Point2f nrm(
                        (next_good_buf_[i].x - cx_use) / fx_use,
                        (next_good_buf_[i].y - cy_use) / fy_use);
                    feature_mgr_.addObservation(feature_ids_[i], clone_id, nrm);
                    // Plan Step 3b (ADR-009): per-feature lifecycle counter
                    // for SLAM-feature promotion / demotion. Keyframe tag
                    // is set later in the frame (section 11.5) once the
                    // keyframe decision lands.
                    // Step 8c: per-row timestamp for rolling-shutter compensation.
                    // Camera2 SENSOR_ROLLING_SHUTTER_SKEW is the total read-out
                    // duration (ns); dividing by image height gives ns-per-row.
                    // Source: Android Camera2 API reference,
                    // CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW (API level 21+).
                    const int64_t row_ts_ns = (rolling_shutter_row_skew_ns_ > 0)
                        ? (timestamp_ns + static_cast<int64_t>(
                               next_good_buf_[i].y / static_cast<double>(height)
                               * static_cast<double>(rolling_shutter_row_skew_ns_)))
                        : timestamp_ns;
                    feature_mgr_.noteObservation(feature_ids_[i], row_ts_ns,
                                                 /*is_keyframe=*/false);
                }
            }
        }
    }

    // ── 10. Accel bias estimation (diagnostic, when static) ──────────────────
    if (is_static && accel_bias_count_ < ACCEL_BIAS_WARMUP && imu_delta.dt > 0.001) {
        cv::Mat bias_obs = imu_delta.deltaV / imu_delta.dt;
        accel_bias_ = (1.0 - ACCEL_BIAS_ALPHA) * accel_bias_ + ACCEL_BIAS_ALPHA * bias_obs;
        accel_bias_count_++;
    }

    int64_t t_pose_end = now_us();
    // ── 11. Grid-based feature replenishment + keyframe re-localization ─────
    int64_t t_replenish_start = now_us();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tracked < MIN_FEATURES) {
            new_pts_buf_.clear();
            feature_mgr_.replenishSparse(gray_buf_, next_good_buf_, new_pts_buf_,
                                          MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            // Assign new IDs to replenished features
            auto new_ids = feature_mgr_.assignIds(static_cast<int>(new_pts_buf_.size()));
            feature_ids_.insert(feature_ids_.end(), new_ids.begin(), new_ids.end());
            next_good_buf_.insert(next_good_buf_.end(), new_pts_buf_.begin(), new_pts_buf_.end());
            feature_ages_.resize(next_good_buf_.size(), 0);
        }
        // Keyframe re-localization
        if (static_cast<int>(next_good_buf_.size()) < MIN_FEATURES / 2) {
            std::vector<cv::Point2f> kf_matched, cur_matched;
            if (feature_mgr_.matchAgainstKeyframe(gray_buf_, next_good_buf_,
                                                    kf_matched, cur_matched)) {
                next_good_buf_ = cur_matched;
                feature_ages_.assign(cur_matched.size(), 0);
                feature_ids_ = feature_mgr_.assignIds(static_cast<int>(cur_matched.size()));
                LOGI("RELOCALIZE: recovered %zu pts from keyframe", cur_matched.size());
            }
        }

        // ── 11.1 MSCKF Update: RE-ENABLED (Plan Step 3a, supersedes ADR-006) ──
        // ADR-006 disabled this because corrections produced 5–11 m teleport-
        // ations: Tracker held its own global pose mirror that the EKF could
        // not see, so MSCKF residuals injected against a stale state. Step 4
        // of the inertial plan removed those mirrors (EKF is now the single
        // pose owner). The remaining failure modes — large outliers and
        // sudden first-frame corrections — are mitigated by ADR-008's Huber
        // kernel and damping ramp inside `EKFState::applyMSCKFUpdate`.
        //
        // Note: this runs AFTER section 9.1 added the new clone for the
        // current frame and AFTER the Step 2 R_vo fusion in section 6, so
        // the sliding window has the latest pose and the residual is built
        // against the post-rotation-update state.
        if (ekf_.isFullInitialized() && !feature_ids_.empty()) {
            // Plan Step 5: skip the MSCKF measurement update on blurred
            // frames — the lost-feature reprojection residual is dominated
            // by KLT noise on a soft frame and would inject a bad
            // correction. The observation history kept by FeatureManager
            // is unchanged (still appended in section 9.1 above), so a
            // post-blur frame can still reuse the same lost features once
            // they're picked up again. The window-prune below MUST still
            // run unconditionally to keep the sliding-window slot
            // accounting consistent with the EKF state.
            if (!frame_blurry_) {
                auto lost = feature_mgr_.getMSCKFCandidates(feature_ids_, 4);
                if (!lost.empty()) {
                    int used = msckf_updater_.processLostFeatures(
                        ekf_, lost, fx_use, fy_use, cx_use, cy_use);
                    if (used > 0) {
                        LOGI("MSCKF: ran update with %zu lost features, %d used "
                             "(huber_rejected=%d)",
                             lost.size(), used,
                             ekf_.getMSCKFHuberRejectedCount());
                    } else if (lost.size() >= 2) {
                        LOGI("MSCKF: %zu lost features all rejected (chi²/triang)",
                             lost.size());
                    }
                }
            }
            if (!ekf_.getWindow().empty()) {
                int min_id = ekf_.getWindow().front().state_id;
                feature_mgr_.pruneObservations(min_id);
            }
        }

        // ── 11.1b SLAM features in EKF state (Plan Step 3b, ADR-009) ─────
        //
        // Hybrid SLAM + MSCKF: long-lived features (≥ 12 obs spanning ≥ 2
        // keyframes) are promoted into the EKF state vector with a 5-DOF
        // inverse-depth (α, β, ρ + 2 pad) parameterisation anchored at
        // their first keyframe. They contribute a 2-DOF reprojection
        // residual every frame, bounding drift between keyframes far
        // better than transient MSCKF tracks alone.
        //
        // Order of operations in this block:
        //   1. PROMOTE: take getPromotableFeatures(), triangulate the
        //      first/last observation pair, gate on chirality + RMSE,
        //      then EKFState::addSlamFeature.
        //   2. UPDATE:  for each SLAM feature with a current obs, build
        //      a single-frame updateSlamFeature call. Track per-feature
        //      RMS; drive markSlamFeatureRMS for the demotion gate.
        //   3. DEMOTE:  features with rms_bad_consecutive ≥ 3 → remove.
        //   4. EXPIRE:  features last seen > 1 s ago → remove.
        //
        // All of this is gated on isFullInitialized(); the EKF must own
        // the pose (Step 4 of the inertial plan) before SLAM updates land.
        int64_t t_slam_block_start = now_us();
        int64_t t_slam_promote_us = 0;
        int64_t t_slam_update_us  = 0;
        int64_t t_slam_demote_us  = 0;
        int64_t t_slam_decrement_us = 0;
        int     n_slam_updates_ran = 0;
        int t_slam_prune_us = 0;
        int n_lifecycle_dropped = 0;
        if (ekf_.isFullInitialized() && !ekf_.getWindow().empty()) {
            // ── (0) PRUNE LIFECYCLE ─────────────────────────────────────
            // Step 3b lifecycle entries are created on every observation
            // by `noteObservation`, but were only being removed when a
            // SLAM feature got demoted/expired. A KLT track that lives
            // 10 frames and dies leaves a permanent lifecycle record
            // (and orphaned active_tracks_ observation history). After
            // ~17 minutes of normal walking the map grows to thousands
            // of stale entries, and the three lifecycle scans below
            // (getPromotableFeatures, getDemoteCandidates,
            // getLostSlamFeatures) each become a multi-millisecond walk.
            //
            // Drop entries that are neither tracked this frame nor
            // currently promoted as a SLAM feature. The feature_id space
            // is monotonic (FeatureManager::assignIds) so a dropped KLT
            // track never re-appears under the same id; the prune is
            // safe with no risk of losing a still-relevant record.
            int64_t t_prune_start = now_us();
            n_lifecycle_dropped = feature_mgr_.pruneStaleLifecycle(
                feature_ids_, timestamp_ns);
            t_slam_prune_us = static_cast<int>(now_us() - t_prune_start);

            // Push intrinsics into EKFState so SLAM reprojection (pixel
            // space) uses the live calibration, not the test default.
            ekf_.setSlamIntrinsics(fx_use, fy_use, cx_use, cy_use);

            // Build current-observation lookup: feature_id → pixel uv. SLAM
            // measurement updates run in pixel space (matches the test
            // contract); the legacy MSCKF path uses normalised coords via
            // its own intrinsics argument.
            std::unordered_map<int, cv::Point2f> cur_obs;
            cur_obs.reserve(feature_ids_.size());
            for (size_t i = 0; i < feature_ids_.size() &&
                               i < next_good_buf_.size(); i++) {
                int fid = feature_ids_[i];
                if (fid < 0) continue;
                cur_obs[fid] = next_good_buf_[i];  // pixel coords
            }

            const int latest_clone_id = ekf_.getLatestCloneId();

            // ── (1) PROMOTE ─────────────────────────────────────────────
            int64_t t_promote_start = now_us();
            // min_obs lowered 12 -> 8 on 2026-05-04: an outdoor 100 m walk
            // showed BA never fired because msckf_update_lines was 3517 (KLT
            // tracks were dying at <12 obs and feeding MSCKF instead of
            // surviving long enough to qualify for SLAM). 8 obs ~ 0.6 s of
            // tracking at 14 Hz is still long enough for two-view
            // triangulation to be well-conditioned, and unlocks BA on
            // realistic motion.
            auto promotable = feature_mgr_.getPromotableFeatures(
                /*min_obs=*/8, /*min_kf=*/2, /*max_init_rms_px=*/1.5);
            for (int fid : promotable) {
                if (ekf_.getSlamFeatureCount() >= EKFState::MAX_SLAM_FEATURES) {
                    break;  // cap reached this frame; try again next frame
                }
                const auto* obs = feature_mgr_.getObservations(fid);
                if (!obs || obs->size() < 2) continue;
                // Pick first / last observations whose clones still exist.
                int   anchor_clone_id = -1;
                int   far_clone_id    = -1;
                cv::Point2f obs_anchor, obs_far;
                cv::Mat R_anchor, p_anchor, R_far, p_far;
                for (const auto& o : *obs) {
                    cv::Mat R, p;
                    if (!ekf_.getClonePose(o.clone_state_id, R, p)) continue;
                    if (anchor_clone_id < 0) {
                        anchor_clone_id = o.clone_state_id;
                        obs_anchor = o.pixel_ud;
                        R_anchor = R; p_anchor = p;
                    } else {
                        far_clone_id = o.clone_state_id;
                        obs_far = o.pixel_ud;
                        R_far = R; p_far = p;
                    }
                }
                if (anchor_clone_id < 0 || far_clone_id < 0 ||
                    anchor_clone_id == far_clone_id) {
                    continue;
                }
                // Two-view midpoint triangulation in world frame.
                //
                //   Ray from clone A in world: r_A(t) = p_A + R_A.t() * d_A * t
                //   where d_A = (u_A, v_A, 1)^T (normalised image coords).
                //
                // Solve for the world point closest to both rays in the
                // least-squares sense:  min || (r_A(t_A) - r_B(t_B)) ||²
                cv::Mat dA = (cv::Mat_<double>(3, 1) <<
                               obs_anchor.x, obs_anchor.y, 1.0);
                cv::Mat dB = (cv::Mat_<double>(3, 1) <<
                               obs_far.x, obs_far.y, 1.0);
                cv::Mat dAw = R_anchor.t() * dA;
                cv::Mat dBw = R_far.t()    * dB;
                // Normalise the world-frame ray directions.
                cv::Mat dAw_n = dAw / cv::norm(dAw);
                cv::Mat dBw_n = dBw / cv::norm(dBw);
                cv::Mat M(3, 2, CV_64F);
                dAw_n.copyTo(M(cv::Range::all(), cv::Range(0, 1)));
                cv::Mat negB = -dBw_n;
                negB.copyTo(M(cv::Range::all(), cv::Range(1, 2)));
                cv::Mat rhs = p_far - p_anchor;
                cv::Mat ts;
                if (!cv::solve(M, rhs, ts, cv::DECOMP_SVD)) continue;
                const double tA = ts.at<double>(0, 0);
                const double tB = ts.at<double>(1, 0);
                if (tA <= 0.05 || tB <= 0.05) continue;  // chirality gate
                cv::Mat p_world =
                    0.5 * ((p_anchor + dAw_n * tA) + (p_far + dBw_n * tB));

                // Reprojection RMSE gate over ALL surviving observations.
                double rms2 = 0.0;
                int    n_used = 0;
                for (const auto& o : *obs) {
                    cv::Mat R, p;
                    if (!ekf_.getClonePose(o.clone_state_id, R, p)) continue;
                    cv::Mat p_C = R * (p_world - p);
                    const double zC = p_C.at<double>(2, 0);
                    if (zC < 0.05) { rms2 = 1e9; break; }
                    const double u = p_C.at<double>(0, 0) / zC;
                    const double v = p_C.at<double>(1, 0) / zC;
                    const double du = (u - o.pixel_ud.x) * fx_use;
                    const double dv = (v - o.pixel_ud.y) * fy_use;
                    rms2 += du * du + dv * dv;
                    n_used++;
                }
                if (n_used < 2) continue;
                const double rms = std::sqrt(rms2 / static_cast<double>(n_used));
                if (rms > 1.5) continue;

                // Build the anchor CameraPose. Use the EKF's stored FEJ.
                CameraPose anchor;
                cv::Mat R_anchor_FEJ, p_anchor_FEJ;
                if (!ekf_.getCloneFEJ(anchor_clone_id,
                                       R_anchor_FEJ, p_anchor_FEJ)) {
                    R_anchor_FEJ = R_anchor.clone();
                    p_anchor_FEJ = p_anchor.clone();
                }
                anchor.R_GtoC = R_anchor.clone();
                anchor.p_G    = p_anchor.clone();
                anchor.R_FEJ  = R_anchor_FEJ.clone();
                anchor.p_FEJ  = p_anchor_FEJ.clone();
                anchor.state_id = anchor_clone_id;
                anchor.timestamp_ns = timestamp_ns;

                int slot = ekf_.addSlamFeature(fid, p_world, anchor);
                if (slot >= 0) {
                    feature_mgr_.setSlamSlot(fid, slot);
                    navsight::eventCounters().slam_promotions_total.fetch_add(
                        1, std::memory_order_relaxed);
                    feature_mgr_.noteTriangulation(
                        fid,
                        cv::Point3f(static_cast<float>(p_world.at<double>(0, 0)),
                                    static_cast<float>(p_world.at<double>(1, 0)),
                                    static_cast<float>(p_world.at<double>(2, 0))),
                        anchor_clone_id);
                    LOGI("SLAM promote fid=%d slot=%d rms=%.2fpx anchor=%d",
                         fid, slot, rms, anchor_clone_id);
                }
            }

            t_slam_promote_us = now_us() - t_promote_start;

            // ── (2) UPDATE ──────────────────────────────────────────────
            // Plan Step 5: skip the per-SLAM-feature reprojection update on
            // blurred frames — `cur_obs` would be a soft pixel that
            // produces a meaningless residual. The SLAM feature stays in
            // the EKF state with its existing inverse-depth estimate;
            // PROMOTE / DEMOTE / EXPIRE bookkeeping above and below still
            // run because they use the stored observation history rather
            // than the current frame's pixel.
            int64_t t_update_start = now_us();
            const int n_slam = (frame_blurry_) ? 0 : ekf_.getSlamFeatureCount();
            for (int slot = 0; slot < n_slam; slot++) {
                // Translate slot ↔ feature_id by walking lifecycle map. We
                // could cache the inverse map, but n_slam ≤ 12 — linear
                // scan is fine.
                int slot_fid = -1;
                for (int fid : feature_ids_) {
                    const auto* lc = feature_mgr_.getLifecycle(fid);
                    if (lc && lc->slam_slot == slot) {
                        slot_fid = fid;
                        break;
                    }
                }
                if (slot_fid < 0) continue;
                auto cur_it = cur_obs.find(slot_fid);
                if (cur_it == cur_obs.end() || latest_clone_id < 0) continue;
                std::vector<cv::Point2f> obs1{cur_it->second};
                std::vector<int> ids1{latest_clone_id};
                n_slam_updates_ran++;
                if (ekf_.updateSlamFeature(
                        slot, obs1, ids1,
                        RANSAC_THRESH * RANSAC_THRESH)) {
                    // Read back the residual via reprojection check using
                    // the SLAM feature's current (corrected) global point.
                    cv::Mat p_world;
                    if (ekf_.getSlamFeatureGlobalPosition(slot, p_world)) {
                        cv::Mat R_now, p_now;
                        if (ekf_.getClonePose(latest_clone_id,
                                              R_now, p_now)) {
                            cv::Mat p_C = R_now * (p_world - p_now);
                            const double zC = p_C.at<double>(2, 0);
                            double rms_px = 1e9;
                            if (zC > 0.05) {
                                const double u_px =
                                    fx_use * p_C.at<double>(0, 0) / zC + cx_use;
                                const double v_px =
                                    fy_use * p_C.at<double>(1, 0) / zC + cy_use;
                                const double du = u_px - cur_it->second.x;
                                const double dv = v_px - cur_it->second.y;
                                rms_px = std::sqrt(du * du + dv * dv);
                            }
                            feature_mgr_.markSlamFeatureRMS(slot_fid, rms_px);
                        }
                    }
                }
            }

            t_slam_update_us = now_us() - t_update_start;

            // ── (3) + (4) DEMOTE + EXPIRE in one pass ──────────────────
            int64_t t_demote_start = now_us();
            // Snapshot all (fid, slot, reason) tuples from BOTH demote and
            // expire queues, then sort by slot DESCENDING and process
            // highest-first. Removing slot N never shifts the index of any
            // slot < N, so this avoids the stale-slot bug the previous
            // per-loop reconciliation had: that walk only iterated
            // `feature_ids_` (current-frame tracks), missing expired SLAM
            // features whose feature_ids_ row no longer existed — their
            // lifecycle slot stayed at the pre-removal value and the next
            // iteration's removeSlamFeature(stale_slot) silently failed.
            //
            // After this loop runs, the EKF and lifecycle slot maps are
            // consistent. No follow-up reconciliation walk required.
            struct SlamRemoval {
                int fid;
                int slot;
                bool is_demote;  // true = demote, false = expire
            };
            std::vector<SlamRemoval> removals;

            for (int fid : feature_mgr_.getDemoteCandidates()) {
                const auto* lc = feature_mgr_.getLifecycle(fid);
                if (!lc || lc->slam_slot < 0) continue;
                removals.push_back({fid, lc->slam_slot, true});
            }
            for (int fid : feature_mgr_.getLostSlamFeatures(timestamp_ns)) {
                const auto* lc = feature_mgr_.getLifecycle(fid);
                if (!lc || lc->slam_slot < 0) continue;
                // Skip if already queued by demote (same fid via two paths
                // would attempt double-removal).
                bool already = false;
                for (const auto& r : removals) {
                    if (r.fid == fid) { already = true; break; }
                }
                if (already) continue;
                removals.push_back({fid, lc->slam_slot, false});
            }

            // Highest slot first.
            std::sort(removals.begin(), removals.end(),
                      [](const SlamRemoval& a, const SlamRemoval& b) {
                          return a.slot > b.slot;
                      });

            for (const SlamRemoval& r : removals) {
                if (ekf_.removeSlamFeature(r.slot)) {
                    if (r.is_demote) {
                        LOGI("SLAM demote fid=%d slot=%d (bad RMS streak)",
                             r.fid, r.slot);
                    } else {
                        LOGI("SLAM expire fid=%d slot=%d (≥1s since last obs)",
                             r.fid, r.slot);
                    }
                }
                feature_mgr_.setSlamSlot(r.fid, -1);
                feature_mgr_.dropLifecycle(r.fid);
            }

            // After all higher-slot removals, every lifecycle entry whose
            // EKF slot was above any removed slot needs decrementing. Walk
            // all live lifecycles (not just feature_ids_, which is the bug
            // the previous code had — it missed SLAM features not tracked
            // this frame). Iterate from lowest removed slot upward; every
            // surviving slot above each removal slot decrements once.
            // Since we removed in descending order, and remaining lifecycle
            // slots are all < the smallest removed slot OR fall in gaps
            // between, we count for each surviving lifecycle entry how many
            // removed slots were strictly below its current value.
            t_slam_demote_us = now_us() - t_demote_start;

            int64_t t_decrement_start = now_us();
            if (!removals.empty()) {
                std::vector<int> removed_slots;
                removed_slots.reserve(removals.size());
                for (const auto& r : removals) removed_slots.push_back(r.slot);
                std::sort(removed_slots.begin(), removed_slots.end());

                for (int fid : feature_mgr_.getAllLifecycleFeatureIds()) {
                    const auto* lc = feature_mgr_.getLifecycle(fid);
                    if (!lc || lc->slam_slot < 0) continue;
                    int orig = lc->slam_slot;
                    int shift = 0;
                    for (int rs : removed_slots) {
                        if (rs < orig) shift++;
                    }
                    if (shift > 0) {
                        feature_mgr_.setSlamSlot(fid, orig - shift);
                    }
                }
            }
            t_slam_decrement_us = now_us() - t_decrement_start;
        }
        int64_t t_slam_block_us = now_us() - t_slam_block_start;
        // PERF: log only when the SLAM block did real work, OR every 30 frames
        // for a baseline. Keeps the log volume sane.
        if (n_slam_updates_ran > 0 || frame_counter_ % 30 == 0) {
            LOGI("PERF: section=slam total_us=%lld prune_us=%d promote_us=%lld "
                 "update_us=%lld demote_us=%lld decrement_us=%lld "
                 "n_updates=%d n_slam=%d state_dim=%d "
                 "lifecycle_size=%zu dropped=%d",
                 (long long)t_slam_block_us,
                 t_slam_prune_us,
                 (long long)t_slam_promote_us,
                 (long long)t_slam_update_us,
                 (long long)t_slam_demote_us,
                 (long long)t_slam_decrement_us,
                 n_slam_updates_ran,
                 ekf_.getSlamFeatureCount(),
                 ekf_.getStateDim(),
                 feature_mgr_.getLifecycleSize(),
                 n_lifecycle_dropped);
        }

        // ── 11.2 Keyframe heading drift correction (Step 2.1, 2.2) ──
        // Every keyframe interval, match current frame against the last keyframe.
        // The essential matrix R_kf gives the visual rotation between the two
        // camera poses. We rotate R_kf into the gravity-aligned frame using the
        // current Madgwick roll/pitch, then extract yaw from the aligned matrix.
        // This produces a physically-meaningful yaw change (rotation around
        // world-up) regardless of phone tilt, so the correction is safe to
        // apply during turns — no gyro_norm gate needed.
        //
        // ASSUMPTION (load-bearing): the codebase already treats body-frame
        // imu_delta.deltaR and camera-frame visual rotation as the same frame
        // (see line 811: global_R_ = global_R_ * imu_delta.deltaR). The
        // gravity-alignment below inherits that simplification — Madgwick body
        // roll/pitch is used to de-tilt the camera-frame R_kf. If a real
        // body→camera extrinsic is ever introduced, it must be applied
        // uniformly to both deltaR propagation and R_align below.
        if (frames_since_keyframe_ >= 14 && pose_valid && tracked >= MIN_INLIERS * 2) {
            double kf_heading = 0.0;
            cv::Point3f kf_pos;
            if (feature_mgr_.getLastKeyframeInfo(kf_heading, kf_pos)) {
                std::vector<cv::Point2f> kf_matched, cur_matched;
                if (feature_mgr_.matchAgainstKeyframe(gray_buf_, next_good_buf_,
                                                       kf_matched, cur_matched)
                    && kf_matched.size() >= 30) {
                    // Compute essential matrix between keyframe and current frame
                    cv::Mat E, R_kf, t_kf;
                    std::vector<uchar> mask;
                    E = cv::findEssentialMat(kf_matched, cur_matched, K,
                                             cv::RANSAC, RANSAC_CONF, RANSAC_THRESH, mask);
                    if (!E.empty()) {
                        int inl = cv::recoverPose(E, kf_matched, cur_matched, K, R_kf, t_kf, mask);
                        if (inl >= 20 && !R_kf.empty()) {
                            // Step 2.1: gravity-aligned yaw extraction.
                            // Build R_align that undoes current Madgwick
                            // roll/pitch, sandwich R_kf into the gravity-
                            // aligned frame, then extract yaw via atan2 of
                            // the (1,0)/(0,0) components.
                            // Z-up world: yaw is rotation around world-Z axis.
                            // visual_delta_heading is extracted as
                            // atan2(R_aligned[1,0], R_aligned[0,0]) — see
                            // EKFState::getYaw for the matching convention.
                            // Both the legacy heading_offset_ blend (deprecated)
                            // and the EKF updateGravityAlignedYaw call below
                            // consume this single Z-up delta.
                            double visual_delta_heading = 0.0;
                            double current_roll  = 0.0;
                            double current_pitch = 0.0;
                            bool aligned_ok = false;
                            try {
                                double roll  = static_cast<double>(imu.getMadgwickRoll());
                                double pitch = static_cast<double>(imu.getMadgwickPitch());
                                current_roll = roll;
                                current_pitch = pitch;
                                cv::Mat Rx, Ry;
                                // R_align = R_body_to_world: undoes Madgwick pitch
                                // then roll. Together with the camera→body
                                // similarity transform below, this lifts R_kf
                                // (which recoverPose returns in the OpenCV camera
                                // frame: X-right, Y-down, Z-forward) into the
                                // gravity-aligned world frame where the Z-up nav
                                // yaw is pure rotation about world Z.
                                cv::Rodrigues(cv::Vec3d(roll, 0.0, 0.0), Rx);
                                cv::Rodrigues(cv::Vec3d(0.0, pitch, 0.0), Ry);
                                cv::Mat R_align = Ry * Rx;

                                // Camera→body extrinsic for the rear-camera /
                                // vertical-phone configuration NavSight runs in:
                                //   camera +X = body +X (right)
                                //   camera +Y = body -Y (down vs up)
                                //   camera +Z = body -Z (forward vs back)
                                // Hence R_b2c = diag(1, -1, -1). It is symmetric
                                // and self-inverse, so the similarity transform
                                // that re-expresses R_kf in body coordinates is
                                //   R_kf_body = R_b2c · R_kf_camera · R_b2c
                                // BUG FIX 2026-05-03: previously this conversion
                                // was missing, so the body-frame Madgwick
                                // sandwich was applied to a camera-frame R_kf.
                                // For a body yaw of +θ, the resulting yaw_meas
                                // came out as -θ (sign inverted), which
                                // updateGravityAlignedYaw then yanked EKF yaw
                                // toward the wrong direction every keyframe →
                                // 47 direction flips per 16 s sim, 3.6× path
                                // inflation.
                                // Step 8b: use EKF-maintained R_bc (body→camera).
                                // R_kf is in camera frame (recoverPose output).
                                // R_kf_body = R_bc^T * R_kf * R_bc converts it
                                // to body frame for the gravity-alignment sandwich.
                                const cv::Matx33d R_bc_mx2 = ekf_.getExtrinsicsRotation();
                                cv::Mat R_bc_cv2(3, 3, CV_64F);
                                for (int ri2 = 0; ri2 < 3; ri2++)
                                    for (int ci2 = 0; ci2 < 3; ci2++)
                                        R_bc_cv2.at<double>(ri2, ci2) = R_bc_mx2(ri2, ci2);
                                cv::Mat R_kf_body = R_bc_cv2.t() * R_kf * R_bc_cv2;

                                cv::Mat R_aligned = R_align * R_kf_body * R_align.t();
                                // Z-up extraction: yaw is rotation around world-Z.
                                // For a pure-yaw R_aligned (which a gravity-aligned
                                // sandwich produces), atan2(R[1,0], R[0,0]) returns
                                // the CW-positive nav yaw delta directly. Matches
                                // EKFState::getYaw and IMUPreintegrator::getHeading.
                                visual_delta_heading = std::atan2(
                                    R_aligned.at<double>(1, 0),
                                    R_aligned.at<double>(0, 0));
                                aligned_ok = true;
                            } catch (const cv::Exception& e) {
                                LOGI("KF_HEADING_CORR: gravity-align failed: %s", e.what());
                                // Fall back to raw atan2 (legacy behaviour)
                                visual_delta_heading = std::atan2(
                                    R_kf.at<double>(1, 0), R_kf.at<double>(0, 0));
                            }

                            // Step 2.4: visual yaw variance from RANSAC inliers.
                            // σ_yaw ≈ pixel_noise / (focal · √N), pixel_noise ≈
                            // 1.0 px (RANSAC inlier threshold). Consumed by
                            // Step 6 ESKF update.
                            // Written under pose_mutex_ (matches getSmoothScale/
                            // getHeading pattern; getter reads under pose_mutex_).
                            {
                                double focal = K.at<double>(0, 0);
                                if (focal > 1e-6) {
                                    double sigma_yaw = 1.0 /
                                        (focal * std::sqrt(static_cast<double>(inl)));
                                    std::lock_guard<std::mutex> slock(pose_mutex_);
                                    last_visual_yaw_variance_ = sigma_yaw * sigma_yaw;
                                }
                            }

                            // Madgwick heading change since keyframe (kf_heading
                            // was the cached scalar_heading_ at keyframe time,
                            // which already includes heading_offset_, so the
                            // difference is a valid angular delta).
                            double gyro_delta_heading = scalar_heading_ - kf_heading;
                            // Normalize to [-pi, pi]
                            while (gyro_delta_heading > M_PI) gyro_delta_heading -= 2.0 * M_PI;
                            while (gyro_delta_heading < -M_PI) gyro_delta_heading += 2.0 * M_PI;
                            while (visual_delta_heading > M_PI) visual_delta_heading -= 2.0 * M_PI;
                            while (visual_delta_heading < -M_PI) visual_delta_heading += 2.0 * M_PI;

                            double drift = gyro_delta_heading - visual_delta_heading;
                            while (drift > M_PI) drift -= 2.0 * M_PI;
                            while (drift < -M_PI) drift += 2.0 * M_PI;

                            // Correct drift if within reasonable range
                            // Walking oscillation can cause 2-4°/step drift; allow up to 20°
                            // to catch multi-step accumulation between keyframes
                            if (std::abs(drift) < 20.0 * M_PI / 180.0) {
                                // Step 4 Phase B: feed visual yaw measurement into
                                // EKFState as the single yaw-correction path.
                                // The Joseph-form Kalman update applies its own
                                // gain — the legacy 30%-blend below has been
                                // removed (see KF_HEADING_CORR log: "EKF-only").
                                if (aligned_ok && ekf_.isFullInitialized()) {
                                    // Z-up: yaw_meas = absolute world yaw the
                                    // visual evidence wants the EKF to be at.
                                    // kf_heading is the cached scalar_heading_
                                    // at keyframe time (Madgwick CW-positive nav,
                                    // Z-up). visual_delta_heading is the Z-up
                                    // nav yaw delta from atan2(R[1,0], R[0,0])
                                    // above. Sum is the absolute Z-up nav yaw,
                                    // matching EKFState::getYaw's output.
                                    double yaw_meas = kf_heading + visual_delta_heading;
                                    while (yaw_meas >  M_PI) yaw_meas -= 2.0 * M_PI;
                                    while (yaw_meas < -M_PI) yaw_meas += 2.0 * M_PI;
                                    // Step 4 Phase C: refined variance.
                                    // σ_yaw ≈ pixel_noise / (focal · √N) with
                                    // pixel_noise = RANSAC_THRESH (=1px). Floor
                                    // at (0.5°)² to account for residual
                                    // gravity misalignment Madgwick can't
                                    // correct (Madgwick steady-state error is
                                    // ~0.5-1°). Without this floor, large-N
                                    // observations drive var below 1e-5 and
                                    // the EKF clamps too tightly to noisy
                                    // visual yaw at the expense of the IMU.
                                    double focal = K.at<double>(0, 0);
                                    double var_yaw = 1e-4;  // ~(0.6°)² floor
                                    if (focal > 1e-6 && inl > 0) {
                                        double sigma = RANSAC_THRESH /
                                            (focal * std::sqrt(
                                                static_cast<double>(inl)));
                                        var_yaw = std::max(var_yaw, sigma * sigma);
                                    }
                                    ekf_.updateGravityAlignedYaw(
                                        yaw_meas, var_yaw,
                                        current_roll, current_pitch);
                                }

                                // Step 4: legacy 30% heading_offset_ blend
                                // deleted — EKFState::updateGravityAlignedYaw
                                // above is the single yaw-correction path.
                                if (frame_counter_ % 30 == 0) {
                                    LOGI("KF_HEADING_CORR: drift=%.2f° inliers=%d (EKF-only)",
                                         drift * 180.0 / M_PI, inl);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Keyframe storage (with heading + position for drift correction)
        frames_since_keyframe_++;
        if (frames_since_keyframe_ >= 15 || (tracked < MIN_FEATURES / 2 && frames_since_keyframe_ > 3)) {
            cv::Point3f kf_pos(
                static_cast<float>(global_t_.at<double>(0)),
                static_cast<float>(global_t_.at<double>(1)),
                static_cast<float>(global_t_.at<double>(2)));
            feature_mgr_.storeKeyframe(gray_buf_, next_good_buf_, timestamp_ns, frame_counter_,
                                       scalar_heading_, kf_pos);
            // Plan Step 4 (ADR-010): mirror the keyframe into the ORB
            // descriptor ring buffer for relocalization. Pass the surviving
            // KLT corners + their FeatureManager ids so the stored ORB
            // keypoints inherit the same ids by spatial proximity.
            feature_mgr_.storeKeyframeDescriptors(
                static_cast<uint64_t>(frame_counter_),
                static_cast<double>(timestamp_ns),
                gray_buf_, next_good_buf_, feature_ids_);
            // Plan Step 3b (ADR-009): tag every currently-tracked feature
            // ID as having spanned this keyframe. SLAM promotion needs
            // kf_count ≥ 2; without this tick the gate never opens.
            for (int fid : feature_ids_) {
                if (fid >= 0) feature_mgr_.noteKeyframe(fid);
            }
            frames_since_keyframe_ = 0;

            // ── Plan Step 7 (ADR-013): loop-closure database update ─────
            //
            // Mirror the keyframe into the loop-closure detector. Use the
            // EKF clone_id as the loop-closure kf_id so the detector's
            // returned `matched_kf_id` doubles as a clone_id we can pass
            // straight into ekf_.getCloneCovIdx / getClonePose / the
            // updateRelativePose channel without a side-table.
            //
            // pts3d_world: SLAM-promoted features in current state. We
            // do NOT triangulate transient KLT tracks here — the
            // detector's PnP path uses these world points to verify the
            // candidate, so passing a partial set is safer than passing
            // junk. If no SLAM features are promoted yet, an empty
            // vector is forwarded; the detector falls back to the BoW
            // score + 2D-2D essential matrix path per its contract.
            const int latest_clone_for_kf = ekf_.getLatestCloneId();
            if (loop_closure_.isReady() && latest_clone_for_kf >= 0) {
                // Pull keypoints + descriptors from the freshly-stored
                // keyframe descriptor record (so the same ORB output that
                // FeatureManager built feeds the detector — Step 7 plan
                // line 716, "BoW vector at keyframe creation"). The deque
                // back() is the latest entry; not empty because we just
                // pushed. The clone pose is read back from EKFState via
                // getClonePose — the same clone we just added in section
                // 9.1, which keeps this site agnostic to whether Tracker
                // is mirroring (`global_R_/global_t_`) or whether full-init
                // has happened.
                cv::Mat kf_R_mat, kf_p_mat;
                const bool kf_pose_ok = ekf_.getClonePose(
                    latest_clone_for_kf, kf_R_mat, kf_p_mat);
                const auto& kf_ring = feature_mgr_.getKeyframeDescriptors();
                if (kf_pose_ok && !kf_R_mat.empty() && !kf_p_mat.empty() &&
                    !kf_ring.empty()) {
                    const auto& kf_back = kf_ring.back();

                    // ── pts3d_world MUST be aligned to ORB keypoint rows ──
                    // LoopClosureDetector.h:86 contract:
                    //   "per-row triangulated 3D world points
                    //    (cv::Point3f(NaN,NaN,NaN) for rows without depth —
                    //    those rows skip PnP but stay BoW-active)"
                    // Earlier this site emplaced raw SLAM features in slot
                    // order, which left pts3d_world.size() ~30 vs keypoints
                    // size ~500 → BFMatcher's trainIdx (range [0, N_orb))
                    // almost always failed `t_idx < pts3d_world.size()` and
                    // PnP saw < 30 candidate pairs even on real revisits.
                    //
                    // Each ORB keypoint already inherits a FeatureManager
                    // feature_id by spatial proximity (FeatureManager.cpp:559
                    // — ORB_KLT_MATCH_RADIUS). We map feature_id → SLAM
                    // slot via EKFState::getSlamFeatureSlot, then read the
                    // world position. ORB rows without a SLAM-promoted
                    // feature stay NaN.
                    constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
                    std::vector<cv::Point3f> pts3d_world(
                        kf_back.keypoints.size(), cv::Point3f(kNan, kNan, kNan));
                    int filled_3d = 0;
                    for (size_t k = 0; k < kf_back.keypoints.size(); ++k) {
                        const int fid = (k < kf_back.feature_ids.size())
                                        ? kf_back.feature_ids[k] : -1;
                        if (fid < 0) continue;
                        const int slot = ekf_.getSlamFeatureSlot(fid);
                        if (slot < 0) continue;
                        cv::Mat p_w;
                        if (!ekf_.getSlamFeatureGlobalPosition(slot, p_w)) continue;
                        if (p_w.empty() || p_w.rows < 3) continue;
                        pts3d_world[k] = cv::Point3f(
                            static_cast<float>(p_w.at<double>(0, 0)),
                            static_cast<float>(p_w.at<double>(1, 0)),
                            static_cast<float>(p_w.at<double>(2, 0)));
                        ++filled_3d;
                    }

                    cv::Matx33d R_world_cam(
                        kf_R_mat.at<double>(0, 0), kf_R_mat.at<double>(0, 1), kf_R_mat.at<double>(0, 2),
                        kf_R_mat.at<double>(1, 0), kf_R_mat.at<double>(1, 1), kf_R_mat.at<double>(1, 2),
                        kf_R_mat.at<double>(2, 0), kf_R_mat.at<double>(2, 1), kf_R_mat.at<double>(2, 2));
                    cv::Vec3d t_cam_world(
                        kf_p_mat.at<double>(0, 0),
                        kf_p_mat.at<double>(1, 0),
                        kf_p_mat.at<double>(2, 0));

                    // Publish this keyframe's world pose into the
                    // FeatureManager descriptor record so the *next*
                    // keyframe can triangulate against us.
                    feature_mgr_.setLatestKeyframePose(R_world_cam, t_cam_world);

                    // ── Triangulate ORB pairs vs the previous keyframe ──
                    //
                    // Why this is needed: with MAX_SLAM_FEATURES=12 and
                    // ORB_KLT_MATCH_RADIUS=3px, only ~0-2 of the 500 ORB
                    // rows inherit a SLAM-promoted 3D position, so PnP at
                    // detection time sees ~3-10 pairs per candidate match
                    // — far below RANSAC's ability to find inliers.
                    //
                    // Approach is the same as ORB-SLAM2's
                    // LocalMapping::CreateNewMapPoints: BFMatch this
                    // keyframe's ORB descriptors against the previous
                    // keyframe's, run Lowe ratio (0.75), then triangulate
                    // matches via cv::triangulatePoints with both
                    // projection matrices = K * [R_world->cam | -R*t_cam].
                    //
                    // Validation: positive depth in BOTH views, depth in
                    // [0.5, 50] m. We do NOT enforce a reprojection check
                    // here — that's PnP's job at detection time.
                    //
                    // Baseline gate: skip the entire pass if the camera
                    // moved < 0.1 m between keyframes (pure rotation /
                    // near-zero baseline → infinite-distance triangulation
                    // is garbage). The Step 5 rotation gate already drops
                    // pure-rot keyframes earlier in the pipeline, but a
                    // user who turns slowly mid-walk could slip through.
                    int triangulated = 0;
                    double tri_baseline = 0.0;
                    int    tri_back_off = 0;
                    if (kf_ring.size() >= 2 &&
                        kf_back.keypoints.size() ==
                            static_cast<size_t>(kf_back.descriptors.rows)) {
                        // QA fix (BLOCKER-1, 2026-05-05): walk backwards to
                        // find the most recent neighbor that satisfies the
                        // pose-published / valid-baseline / valid-descriptor
                        // gate. Originally we blindly took kf_ring[size-2],
                        // but if the prior keyframe landed before the
                        // vocabulary loaded (or before EKF full-init) it has
                        // has_pose=false and triangulation skipped forever.
                        // Bound the search at 10 entries so cost stays O(1).
                        const KeyframeDescriptors* kf_prev_ptr = nullptr;
                        const size_t max_back =
                            std::min<size_t>(kf_ring.size(), 10);
                        for (size_t back_off = 2; back_off <= max_back; ++back_off) {
                            const auto& cand = kf_ring[kf_ring.size() - back_off];
                            if (!cand.has_pose) continue;
                            if (cand.descriptors.empty() ||
                                cand.descriptors.type() != CV_8U ||
                                cand.descriptors.cols != 32) continue;
                            const cv::Vec3d bv =
                                t_cam_world - cand.t_cam_world;
                            const double bn =
                                std::sqrt(bv[0] * bv[0] + bv[1] * bv[1] + bv[2] * bv[2]);
                            // Reject too-short (degenerate triangulation) and
                            // too-long (descriptors won't survive viewpoint
                            // change anyway, so a Lowe-pass is unlikely).
                            if (bn < 0.1 || bn > 5.0) continue;
                            kf_prev_ptr = &cand;
                            tri_baseline = bn;
                            tri_back_off = static_cast<int>(back_off);
                            break;
                        }
                        if (kf_prev_ptr) {
                            const KeyframeDescriptors& kf_prev = *kf_prev_ptr;
                            // Build projection matrices in world frame.
                            //  P = K * [R_world->cam | t_world->cam]
                            //  R_world->cam = R_world_cam.t()
                            //  t_world->cam = -R_world->cam * t_cam_world
                            auto buildP = [&](const cv::Matx33d& Rwc,
                                              const cv::Vec3d&   twc) {
                                const cv::Matx33d R_w2c = Rwc.t();
                                const cv::Vec3d   t_w2c = -(R_w2c * twc);
                                cv::Mat Rt = (cv::Mat_<double>(3, 4) <<
                                    R_w2c(0,0), R_w2c(0,1), R_w2c(0,2), t_w2c[0],
                                    R_w2c(1,0), R_w2c(1,1), R_w2c(1,2), t_w2c[1],
                                    R_w2c(2,0), R_w2c(2,1), R_w2c(2,2), t_w2c[2]);
                                return cv::Mat(K * Rt);
                            };
                            const cv::Mat P_now  = buildP(R_world_cam,
                                                          t_cam_world);
                            const cv::Mat P_prev = buildP(kf_prev.R_world_cam,
                                                          kf_prev.t_cam_world);

                            cv::BFMatcher tri_matcher(cv::NORM_HAMMING, false);
                            std::vector<std::vector<cv::DMatch>> tri_knn;
                            tri_matcher.knnMatch(kf_back.descriptors,
                                                 kf_prev.descriptors,
                                                 tri_knn, 2);

                            const cv::Matx33d R_w2c_now  = R_world_cam.t();
                            const cv::Vec3d   t_w2c_now  = -(R_w2c_now * t_cam_world);
                            const cv::Matx33d R_w2c_prev = kf_prev.R_world_cam.t();
                            const cv::Vec3d   t_w2c_prev =
                                -(R_w2c_prev * kf_prev.t_cam_world);

                            // Granular instrumentation (walk #4 produced
                            // triangulated=0 even with healthy baselines —
                            // need to see WHICH filter is dropping pairs).
                            int n_knn        = static_cast<int>(tri_knn.size());
                            int n_lowe       = 0;
                            int n_bounds     = 0;
                            int n_avail      = 0;  // not already SLAM-anchored
                            int n_ok_w       = 0;
                            int n_depth_now_neg  = 0;
                            int n_depth_prev_neg = 0;
                            int n_depth_now_far  = 0;
                            int n_depth_prev_far = 0;
                            double avg_depth_now  = 0.0;
                            for (const auto& pair : tri_knn) {
                                if (pair.size() < 2) continue;
                                if (pair[0].distance >= 0.75f * pair[1].distance) continue;
                                ++n_lowe;
                                const int q_idx = pair[0].queryIdx;
                                const int t_idx = pair[0].trainIdx;
                                if (q_idx < 0 ||
                                    q_idx >= static_cast<int>(pts3d_world.size())) continue;
                                if (t_idx < 0 ||
                                    t_idx >= static_cast<int>(kf_prev.keypoints.size())) continue;
                                ++n_bounds;
                                // Don't overwrite SLAM-derived 3D — the
                                // EKF estimate is more accurate than a
                                // single-baseline triangulation.
                                if (std::isfinite(pts3d_world[q_idx].x)) continue;
                                ++n_avail;

                                const cv::Point2f& p_now  = kf_back.keypoints[q_idx].pt;
                                const cv::Point2f& p_prev = kf_prev.keypoints[t_idx].pt;
                                std::vector<cv::Point2f> pts_now  = {p_now};
                                std::vector<cv::Point2f> pts_prev = {p_prev};
                                cv::Mat pt4d;
                                cv::triangulatePoints(P_now, P_prev,
                                                      pts_now, pts_prev, pt4d);
                                const double w = pt4d.at<float>(3, 0);
                                if (std::abs(w) < 1e-6) continue;
                                ++n_ok_w;
                                const cv::Vec3d p_world(
                                    pt4d.at<float>(0, 0) / w,
                                    pt4d.at<float>(1, 0) / w,
                                    pt4d.at<float>(2, 0) / w);

                                // Positive-depth check in both views.
                                const cv::Vec3d p_cam_now  = R_w2c_now  * p_world + t_w2c_now;
                                const cv::Vec3d p_cam_prev = R_w2c_prev * p_world + t_w2c_prev;
                                avg_depth_now += p_cam_now[2];
                                if (p_cam_now[2] < 0.5)  { ++n_depth_now_neg;  continue; }
                                if (p_cam_now[2] > 50.0) { ++n_depth_now_far;  continue; }
                                if (p_cam_prev[2] < 0.5) { ++n_depth_prev_neg; continue; }
                                if (p_cam_prev[2] > 50.0){ ++n_depth_prev_far; continue; }

                                pts3d_world[q_idx] = cv::Point3f(
                                    static_cast<float>(p_world[0]),
                                    static_cast<float>(p_world[1]),
                                    static_cast<float>(p_world[2]));
                                ++triangulated;
                            }
                            if (n_ok_w > 0) avg_depth_now /= n_ok_w;
                            LOGI("LC_TRI_DBG: knn=%d lowe=%d bounds=%d avail=%d "
                                 "tri_ok=%d depth_now_neg=%d depth_now_far=%d "
                                 "depth_prev_neg=%d depth_prev_far=%d "
                                 "avg_depth_now=%.2fm",
                                 n_knn, n_lowe, n_bounds, n_avail, n_ok_w,
                                 n_depth_now_neg, n_depth_now_far,
                                 n_depth_prev_neg, n_depth_prev_far,
                                 avg_depth_now);
                        }
                    }

                    // QA pass (2026-05-05): log every keyframe (not every
                    // 30th frame) so walk #4's logcat reveals exactly how
                    // many 3D points actually got into pts3d_world per
                    // keyframe. tri_baseline / tri_back_off help diagnose
                    // why a keyframe got no triangulated pairs (e.g.,
                    // pure-rotation skipped baseline check, or
                    // back_off ran out of valid neighbors).
                    LOGI("LC_KF: kp=%zu filled_3d=%d triangulated=%d "
                         "tri_baseline=%.2fm tri_back=%d (clone=%d)",
                         kf_back.keypoints.size(), filled_3d,
                         triangulated, tri_baseline, tri_back_off,
                         latest_clone_for_kf);

                    loop_closure_.addKeyframe(
                        static_cast<uint64_t>(latest_clone_for_kf),
                        static_cast<double>(timestamp_ns),
                        kf_back.descriptors,
                        kf_back.keypoints,
                        pts3d_world,
                        R_world_cam,
                        t_cam_world,
                        scalar_heading_);

                    // Counter (Agent A): kf-count-in-database. Sample once
                    // per addKeyframe call to keep cost bounded.
                    auto& ec = navsight::eventCounters();
                    ec.loop_closure_kf_count_in_db.store(
                        static_cast<long long>(loop_closure_.getKeyframeCount()),
                        std::memory_order_relaxed);

                    // Publish the same descriptors + keypoints to the 1 Hz
                    // worker thread so its next query tick has a fresh
                    // most-recent keyframe to fingerprint.
                    publishLoopClosureQueryKeyframe(
                        latest_clone_for_kf, timestamp_ns,
                        kf_back.descriptors, kf_back.keypoints,
                        fx_, fy_, cx_, cy_,
                        scalar_heading_);
                }
            }

            // ── Plan Step 6 (ADR-012): windowed BA at keyframe boundary ──
            // First consume any pending refinement from the previous
            // round (re-seed SLAM features via the canonical add/remove
            // EKF channel), then kick off the next round on the worker
            // thread. The order matters: consuming first means the new
            // round optimises against the already-refined state instead
            // of layering refinements on top of stale ones.
            consumeBAResultIfReady();
            kickOffBARound(timestamp_ns);
        }

        // ── Plan Step 7 (ADR-013): apply pending loop-closure correction ──
        //
        // Runs UNCONDITIONALLY every frame (not gated on the keyframe
        // boundary): the damping ramp injects 10 successive frames of
        // diminishing correction once a fresh match arrives, so we want
        // to consume on every frame for the smoothest transition.
        // Internally cheap when no match is pending (atomic-flag check).
        consumeLoopClosureMatchIfReady();

        gray_buf_.copyTo(prev_gray_);
        prev_pts_ = next_good_buf_;
        prev_timestamp_ns_ = timestamp_ns;
    }

    int64_t t_replenish_end = now_us();

    // Log timing every 10 frames during first 200, then every 30
    int64_t total_us = t_replenish_end - t0_total;
    if (frame_counter_ % (frame_counter_ < 200 ? 10 : 30) == 0) {
        LOGI("TIMING: total=%lldms klt=%lldms pose=%lldms replenish=%lldms pts=%d q=%.2f",
             (long long)(total_us/1000),
             (long long)((t_klt_end - t_klt_start)/1000),
             (long long)((t_pose_end - t_pose_start)/1000),
             (long long)((t_replenish_end - t_replenish_start)/1000),
             tracked, quality);
    }

    // ── 12. Output assembly ──────────────────────────────────────────────────
    // Rotation: source from EKFState when full-init. EKF rotation is
    // gravity-aligned and corrected by the keyframe yaw update, so it is
    // strictly better than the gyro-only global_R_ before EKF init.
    //
    // Position: source from Tracker's global_t_, which is incremented by
    // the heading×scaled-disp formula in section 9 (visual path) and the
    // PDR fallback below it. This is the OLD working architecture.
    // Phase C of the plan delegated position output to the EKF, but the
    // EKF's IMU-only propagation drifts unboundedly during standstill —
    // the 2026-05-03 sims showed 0.13 m/s position drift with the phone
    // genuinely stationary because ZUPT cancels v_G_ but residual
    // accel-bias × Δt² and Madgwick tilt-bleed accumulate in p_G_ before
    // ZUPT fires each frame. EKF position is still maintained
    // (updateRelativePose, updatePDRStep are still called) so its
    // covariance stays meaningful for UI uncertainty reporting, but
    // it is not the output source.
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        out.R = ekf_.isFullInitialized() ? ekf_.getRotation() : global_R_.clone();
        out.t = global_t_.clone();
    }
    out.rawR = R_vo.clone();
    out.rawT = t_vo.clone();
    out.quality = quality;
    out.trackedCount = tracked;
    out.totalCount = total;
    out.estimatedScale = appliedScale;
    out.valid = true;
    out.trackedPoints = std::move(tracked_pts_flat);
    out.meanFlow = mean_flow;
    out.inlierCount = inlier_count_out;
    {
        auto si = imu.getStepInfo();
        out.stepCount = si.step_count;
        out.stepFreq = (si.speed_mps > 0 && si.stride_length_m > 0)
                     ? si.speed_mps / si.stride_length_m : 0.0;
        out.strideLength = si.stride_length_m;
    }
    out.poseFlags = (is_static ? 1 : 0) | (is_pure_rotation ? 2 : 0)
                  | (pose_valid ? 4 : 0) | (used_fallback ? 8 : 0);

    // ── 13. Output heading + export frame data for Mapper ────────────────────
    // 2026-05-03 BUG FIX: this block previously overwrote scalar_heading_ and
    // out.heading with ekf_.getYaw(), silently reverting the V-shape fix that
    // section 9.0 just established (Madgwick is the heading source — see the
    // long comment block in section 9.0 for why EKF yaw under-rotates fast
    // turns). Use the Madgwick-sourced scalar_heading_ that section 9.0 set,
    // and keep position output sourced from Tracker's global_t_ — for the
    // same architectural reason that EKF position drifts during standstill.
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        out.heading = scalar_heading_;
        frame_out.global_position = cv::Point3f(
            static_cast<float>(global_t_.at<double>(0)),
            static_cast<float>(global_t_.at<double>(1)),
            static_cast<float>(global_t_.at<double>(2)));
    }
    frame_out.gray = gray_buf_.clone();
    frame_out.prev_good = prev_good_buf_;
    frame_out.next_good = next_good_buf_;
    frame_out.points_3d = points_3d_current_;
    frame_out.pose_valid = pose_valid;
    frame_out.quality = quality;
    frame_out.tracked = tracked;
    frame_out.frame_counter = frame_counter_;
    frame_out.timestamp_ns = timestamp_ns;
    frame_out.prev_timestamp_ns = current_prev_ts;
    frame_out.estimated_scale = estimatedScale;
    frame_out.heading = out.heading;
    frame_out.fx = fx_use; frame_out.fy = fy_use;
    frame_out.cx = cx_use; frame_out.cy = cy_use;
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Plan Step 4 (ADR-010): ORB descriptor relocalization
// ─────────────────────────────────────────────────────────────────────────────
//
// Triggered by `low_inlier_streak_ ≥ RELOC_TRIGGER_FRAMES`. Extracts ORB on
// the current grayscale frame using the same parameters FeatureManager used
// at keyframe storage time (250 features, FAST 10, σ=1.0 pre-blur), then
// brute-force matches against each of the most recent RELOC_RECENT_KFS
// keyframe descriptor records. Each candidate keyframe is scored by its
// RANSAC inlier count from cv::findEssentialMat at RANSAC_THRESH px. The
// best-scoring keyframe is accepted iff inliers ≥ RELOC_MIN_INLIERS, and
// its feature ids are re-attached to the closest current KLT tracks.
//
// The path does NOT change Tracker output sources: Madgwick still owns
// heading and EKFState->global_t_ still owns position. Its only side
// effect is restoring `feature_ids_` continuity so MSCKF/SLAM lifecycle
// counters survive a tracking dropout. A future refinement (gated on a
// clean σ_yaw estimate from this match) could feed the matched keyframe's
// stored heading through `EKFState::updateGravityAlignedYaw`, but that
// requires a defensible variance derivation — without one, we let the
// regular keyframe-yaw path catch up on the next keyframe instead of
// injecting an ad-hoc correction here.
bool Tracker::tryRelocalizeWithORB(const cv::Mat& gray,
                                   const std::vector<cv::Point2f>& current_pts) {
    if (gray.empty() || gray.type() != CV_8UC1) return false;
    if (fx_ <= 1e-6 || fy_ <= 1e-6) return false;

    const auto& kf_ring = feature_mgr_.getKeyframeDescriptors();
    if (kf_ring.empty()) return false;

    // Build ORB extractor with the SAME params as FeatureManager. Lazy-
    // init on the instance member (not a function-static) so we never
    // hit the C++11 function-static-init data race under future
    // multi-threaded callers, and so that future Tracker re-instantiation
    // doesn't share state with a prior instance.
    if (!reloc_orb_) {
        reloc_orb_ = cv::ORB::create(
            FeatureManager::ORB_TARGET_FEATURES,
            1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31,
            FeatureManager::ORB_FAST_THRESHOLD);
    }

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(0, 0),
                     FeatureManager::ORB_PREBLUR_SIGMA);

    std::vector<cv::KeyPoint> cur_kps;
    cv::Mat cur_desc;
    reloc_orb_->detectAndCompute(blurred, cv::noArray(), cur_kps, cur_desc);
    if (cur_kps.empty() || cur_desc.empty()) return false;

    // Camera intrinsic matrix for findEssentialMat. Use the same values the
    // pose path runs against (set via setIntrinsics).
    const cv::Mat K = (cv::Mat_<double>(3, 3) <<
        fx_, 0.0, cx_,
        0.0, fy_, cy_,
        0.0, 0.0, 1.0);

    // Walk the last RELOC_RECENT_KFS keyframes. The deque is ordered
    // oldest-first; rbegin gives newest-first.
    cv::BFMatcher matcher(cv::NORM_HAMMING, /*crossCheck=*/false);

    int  best_inliers = 0;
    int  best_kf_idx  = -1;             // index into kf_ring (front-relative)
    std::vector<cv::DMatch>     best_matches_kept;   // surviving Lowe + RANSAC
    std::vector<unsigned char>  best_ransac_mask;

    int scanned = 0;
    for (auto it = kf_ring.rbegin();
         it != kf_ring.rend() && scanned < RELOC_RECENT_KFS;
         ++it, ++scanned) {

        const KeyframeDescriptors& kfd = *it;
        if (kfd.descriptors.empty() || kfd.keypoints.empty()) continue;

        // knnMatch k=2 (Lowe ratio test requires the 2nd-best distance).
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(cur_desc, kfd.descriptors, knn, 2);

        std::vector<cv::DMatch>   ratio_matches;
        std::vector<cv::Point2f>  cur_pts;
        std::vector<cv::Point2f>  kf_pts;
        ratio_matches.reserve(knn.size());
        cur_pts.reserve(knn.size());
        kf_pts.reserve(knn.size());

        for (const auto& pair : knn) {
            if (pair.size() < 2) continue;
            // Lowe 2004 §7.1: best/second_best < ratio → distinctive match.
            if (pair[0].distance < RELOC_LOWE_RATIO * pair[1].distance) {
                const auto& m = pair[0];
                ratio_matches.push_back(m);
                cur_pts.push_back(cur_kps[m.queryIdx].pt);
                kf_pts.push_back(kfd.keypoints[m.trainIdx].pt);
            }
        }

        // findEssentialMat needs ≥ 5 points; require RELOC_MIN_INLIERS so
        // even a perfect run could clear the accept bar.
        if (static_cast<int>(ratio_matches.size()) < RELOC_MIN_INLIERS) continue;

        std::vector<unsigned char> ransac_mask;
        cv::Mat E = cv::findEssentialMat(kf_pts, cur_pts, K,
                                         cv::RANSAC, RANSAC_CONF,
                                         RANSAC_THRESH, ransac_mask);
        if (E.empty()) continue;

        const int inliers = cv::countNonZero(ransac_mask);
        if (inliers > best_inliers) {
            best_inliers      = inliers;
            best_kf_idx       = static_cast<int>(kf_ring.size())
                              - 1 - static_cast<int>(it - kf_ring.rbegin());
            best_matches_kept = std::move(ratio_matches);
            best_ransac_mask  = std::move(ransac_mask);
        }
    }

    if (best_inliers < RELOC_MIN_INLIERS || best_kf_idx < 0) {
        LOGI("RELOC_ORB: no keyframe accepted (best=%d need=%d, scanned=%d)",
             best_inliers, RELOC_MIN_INLIERS, scanned);
        navsight::eventCounters().reloc_orb_rejects.fetch_add(
            1, std::memory_order_relaxed);
        return false;
    }

    const KeyframeDescriptors& matched_kfd = kf_ring[best_kf_idx];

    // Re-adopt feature ids onto the current KLT tracks. For each surviving
    // RANSAC inlier, look at the keyframe-side keypoint's stored
    // feature_id. If it is non-negative AND the current-frame keypoint
    // lies within RELOC_ID_REATTACH_RADIUS of an entry in `current_pts`,
    // overwrite that slot's `feature_ids_` entry. This restores
    // FeatureManager track continuity (MSCKF / SLAM lifecycle keeps the
    // same id) without minting any new ids.
    if (feature_ids_.size() != current_pts.size()) {
        // Size invariant is maintained by the KLT/replenish loop in
        // section 11; if it ever drifts, abort the re-adopt step rather
        // than corrupt the parallel arrays. The accept itself still
        // counts as a successful diagnosis (logs above).
        LOGI("RELOC_ORB: id/pts size mismatch (%zu vs %zu) — skipping re-adopt",
             feature_ids_.size(), current_pts.size());
        navsight::eventCounters().reloc_orb_size_skipped.fetch_add(
            1, std::memory_order_relaxed);
        return true;
    }

    // Defensive co-index check before the walk — best_matches_kept and
    // best_ransac_mask are moved together inside the per-keyframe winner
    // branch above, so they should always pair, but if a future edit
    // splits the moves an assert fires loudly instead of silently
    // mis-indexing the RANSAC mask.
    assert(best_matches_kept.size() == best_ransac_mask.size());

    const float r2 = RELOC_ID_REATTACH_RADIUS * RELOC_ID_REATTACH_RADIUS;
    int reattached = 0;
    int slam_guarded = 0;  // count of overwrites skipped to protect SLAM state
    for (size_t i = 0; i < best_matches_kept.size(); ++i) {
        if (!best_ransac_mask[i]) continue;
        const cv::DMatch& m = best_matches_kept[i];
        const int kf_fid = matched_kfd.feature_ids[m.trainIdx];
        if (kf_fid < 0) continue;  // keyframe ORB kp had no KLT corner aligned
        const cv::Point2f& cur_kp = cur_kps[m.queryIdx].pt;

        // Find nearest current KLT track within radius. O(N) per inlier;
        // N ≤ MAX_FEATURES (200) so this is bounded.
        float best_d2 = r2;
        int   best_slot = -1;
        for (size_t s = 0; s < current_pts.size(); ++s) {
            const float dx = cur_kp.x - current_pts[s].x;
            const float dy = cur_kp.y - current_pts[s].y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2  = d2;
                best_slot = static_cast<int>(s);
            }
        }
        if (best_slot >= 0 && feature_ids_[best_slot] != kf_fid) {
            // Guard: never overwrite the id of a slot that currently
            // holds a SLAM-promoted feature. The SLAM lifecycle in
            // FeatureManager keys by feature_id; replacing the id under
            // a live SLAM column orphans the EKF P_ row and the next
            // markSlamFeatureRMS / getLostSlamFeatures pass would
            // misroute removeSlamFeature → wrong column dropped from
            // P_ → ADR-006 5–11 m teleportation regime. Skip silently
            // and count.
            const auto* dst_lc = feature_mgr_.getLifecycle(
                feature_ids_[best_slot]);
            if (dst_lc && dst_lc->slam_slot >= 0) {
                slam_guarded++;
                continue;
            }
            feature_ids_[best_slot] = kf_fid;
            reattached++;
        }
    }

    LOGI("RELOC_ORB: kf=%llu inliers=%d/%d reattached=%d slam_guarded=%d",
         static_cast<unsigned long long>(matched_kfd.keyframe_id),
         best_inliers, static_cast<int>(best_matches_kept.size()),
         reattached, slam_guarded);
    navsight::eventCounters().reloc_orb_accepts.fetch_add(
        1, std::memory_order_relaxed);
    if (slam_guarded > 0) {
        navsight::eventCounters().reloc_orb_slam_guarded.fetch_add(
            static_cast<long long>(slam_guarded),
            std::memory_order_relaxed);
    }
    return true;
}

// ──────────────────────────────────────────────────────────────────────────────
// Plan Step 6 (ADR-012): Local windowed bundle adjustment — off-thread runner
// ──────────────────────────────────────────────────────────────────────────────
//
// On each new keyframe the Tracker:
//   1. Consumes the previous round's BA result (if it has finished) by
//      re-seeding the corresponding SLAM features in the EKF — remove the
//      old slot, add a new slot anchored at the same clone with the
//      BA-refined world point. The EKF reconstructs covariance through
//      addSlamFeature, so the canonical observation channel stays in
//      charge of mean/covariance updates (ADR-006: no side-channel mean
//      mutation).
//   2. Kicks off the next BA round on a single worker thread. The worker
//      cheap-copies a CloneSnapshot (≤ 5 most recent EKF clones) plus a
//      LandmarkSnapshot (SLAM-promoted features observed by ≥ 2 of those
//      clones). Both snapshot APIs are mutex-protected and copy the data
//      out, so the camera thread is freed as soon as the snapshots return.
//
// Magic numbers cited inline:
//   * max_clones=5          — Step 6 plan, "5 most recent keyframes".
//   * min_obs=2             — landmark must be visible in ≥ 2 of the 5
//                             clones to constrain the joint solve.
//   * huber_thresh_px=1.5   — matches Tracker::RANSAC_THRESH; Step 6 plan.
//   * max_iters=10          — Ceres-style cap for small problems; Step 6.
//   * 200 ms wall-clock cap — 2× the plan's 100 ms target; thermal headroom
//                             before the result is rejected as too slow.
//
// Threading invariant: at most one round in flight at a time (ba_in_flight_).
// If the previous round hasn't finished by the next keyframe we LOGI a
// "skipped" line and do not start a new round — the next keyframe will
// retry. The result buffer is published under ba_result_mutex_ before
// ba_in_flight_ is cleared, so the camera thread reading
// ba_result_pending_ → ba_result_landmarks_ is always coherent.

bool Tracker::kickOffBARound(int64_t timestamp_ns) {
    // Each early-return path bumps a dedicated counter (and rate-limited
    // LOGI every 30 hits) so the next sim's event_summary tells us
    // exactly which gate failed if BA didn't fire. Pre-2026-05-04
    // versions returned silently and we had no way to distinguish "EKF
    // not initialised" from "no SLAM features yet" in the JSON.
    auto& ec = navsight::eventCounters();

    if (!ekf_.isFullInitialized()) {
        const long long n = ec.ba_skipped_no_init.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (n % 30 == 1) LOGI("BA: skipped (ekf not full-init) count=%lld", n);
        return false;
    }
    if (ba_in_flight_.load(std::memory_order_acquire)) {
        LOGI("BA: skipped (prev_round_in_flight)");
        ec.ba_skipped_in_flight.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    // Step 6 plan magic numbers — 5 most-recent clones, ≥ 2 obs per landmark.
    constexpr int kMaxClones = 5;
    constexpr int kMinObs    = 2;
    auto clone_snap = ekf_.getCloneSnapshot(kMaxClones);
    if (static_cast<int>(clone_snap.size()) < 2) {
        // BA needs at least one anchor + one free pose.
        const long long n = ec.ba_skipped_too_few_clones.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (n % 30 == 1) LOGI("BA: skipped (clones=%d<2) count=%lld",
                              static_cast<int>(clone_snap.size()), n);
        return false;
    }

    // Build the clone-id whitelist for the landmark snapshot.
    std::vector<int> clone_ids;
    clone_ids.reserve(clone_snap.size());
    for (const auto& c : clone_snap) clone_ids.push_back(c.clone_id);

    // Live intrinsics — must be valid (setIntrinsics has run before the EKF
    // is full-init, so this is normally redundant, but stay defensive).
    double fx = fx_, fy = fy_, cx = cx_, cy = cy_;
    if (fx <= 1.0 || fy <= 1.0) {
        const long long n = ec.ba_skipped_no_intrinsics.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (n % 30 == 1) LOGI("BA: skipped (intrinsics fx=%.1f fy=%.1f) count=%lld",
                              fx, fy, n);
        return false;
    }

    auto lm_snap = feature_mgr_.getLandmarkSnapshot(clone_ids, fx, fy, cx, cy,
                                                     kMinObs);
    if (lm_snap.size() < 3) {
        // < 3 landmarks → solver is under-determined. The most common cause
        // (per the 100 m walk on 2026-05-04) is that SLAM-promotion is
        // bottlenecked: KLT tracks die before reaching min_obs, never get
        // promoted, never reach BA. The slam_promotions_total counter
        // tells us how many features have ever been promoted across the
        // whole session.
        const long long n = ec.ba_skipped_too_few_landmarks.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (n % 30 == 1) LOGI("BA: skipped (landmarks=%zu<3) count=%lld promoted_total=%lld",
                              lm_snap.size(), n,
                              ec.slam_promotions_total.load(std::memory_order_relaxed));
        return false;
    }

    // ── Translate snapshots into the WindowedBA contract ────────────────
    // CloneSnapshot.R / .t are already in world->cam / camera-in-world,
    // which is exactly WindowedBA::PoseObs's convention.
    std::vector<WindowedBA::PoseObs> poses;
    poses.reserve(clone_snap.size());
    for (size_t i = 0; i < clone_snap.size(); ++i) {
        WindowedBA::PoseObs p;
        p.keyframe_id = clone_snap[i].clone_id;
        p.R_in        = clone_snap[i].R;
        p.t_in        = clone_snap[i].t;
        // Gauge fix: oldest clone (snapshot is ordered oldest first).
        p.is_anchor   = (i == 0);
        poses.push_back(p);
    }

    std::vector<WindowedBA::FeatureObs> features;
    features.reserve(lm_snap.size());
    // Parallel array so we can write the refined world points back to the
    // right (feature_id, slam_slot, anchor_clone_id) tuples after solve.
    struct Meta { int feature_id; int slam_slot; int anchor_clone_id; };
    std::vector<Meta> meta;
    meta.reserve(lm_snap.size());

    // Lifecycle holds the original anchor; we need it for re-seeding.
    for (const auto& l : lm_snap) {
        const auto* lc = feature_mgr_.getLifecycle(l.feature_id);
        if (!lc) continue;
        WindowedBA::FeatureObs f;
        f.feature_id = l.feature_id;
        f.p_w_in     = l.p_world;
        f.obs        = l.obs;  // already (clone_id, pixel_uv) pairs
        features.push_back(std::move(f));
        meta.push_back({l.feature_id, l.slam_slot, lc->anchor_clone_id});
    }
    if (features.size() < 3) return false;

    const int round_id = ++ba_round_counter_;

    // Mark in-flight BEFORE launching so a racing camera-thread call sees
    // the busy state immediately.
    ba_in_flight_.store(true, std::memory_order_release);

    // Join any previously joinable thread before re-launching. We only ever
    // launch when ba_in_flight_ was false (checked above), so the thread
    // body has finished and only awaits a join.
    if (ba_thread_.joinable()) ba_thread_.join();

    ba_thread_ = std::thread([this, poses_in = std::move(poses),
                                features_in = std::move(features),
                                meta_in = std::move(meta),
                                fx, fy, cx, cy, round_id]() mutable {
        const auto t_launch = std::chrono::steady_clock::now();

        WindowedBA solver;
        auto result = solver.solve(poses_in, features_in,
                                    fx, fy, cx, cy,
                                    // Bumped 10 -> 25 on 2026-05-04
                                    // after the first real BA solves
                                    // (sim 1777919741934) all hit
                                    // max_iters=10 with avg_iters=10.0
                                    // and got rejected. 25 is still
                                    // well within the 200 ms wall-clock
                                    // budget at K=5/N=20 (~5-8 ms per
                                    // iter -> ~125-200 ms p99).
                                    /*max_iters=*/25,
                                    /*huber_thresh_px=*/1.5);

        const auto t_done = std::chrono::steady_clock::now();
        const int64_t wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                    t_done - t_launch).count();

        // Plan Step 6 (ADR-012): acceptance gate is permissive enough to
        // capture genuine improvements without blessing pathological
        // results. A bad BA result is strictly worse than no BA result,
        // so we still reject slow rounds and degenerate residuals.
        //   - converged                                 → solver hit one of its convergence criteria
        //   - residual decreased by ≥ 10% OR final ≤ 4 px² avg → step bought us something
        //   - solve_us < 200 ms                         → thermal-headroom budget
        // The original "residual halved" gate (50%) was too strict on
        // real device data: when the SLAM landmarks were already roughly
        // correct (two-view midpoint triangulation seed is decent),
        // residual_halved required the step to undo work that wasn't
        // wrong. 10% improvement is the smallest signal that meaningfully
        // separates "BA helped" from "BA noise". The absolute floor
        // (4 px² avg per 2-DOF residual ≈ 2 px reproj RMS) catches the
        // case where the seed is already inside the chi² gate so the
        // relative improvement is small but absolute is fine.
        const double n_residual_pairs = std::max(
            1.0, static_cast<double>(features_in.size()));
        const double avg_final_per_residual = result.final_residual_sq /
            (2.0 * n_residual_pairs);  // 2 = u,v components
        const bool residual_improved =
            result.initial_residual_sq > 0.0 &&
            (result.final_residual_sq < 0.9 * result.initial_residual_sq ||
             avg_final_per_residual < 4.0);
        const bool fast_enough = result.solve_us < BA_MAX_SOLVE_US;
        const bool accept = result.converged && residual_improved && fast_enough;

        LOGI("BA: solve_us=%d iters=%d initial_r2=%.3f final_r2=%.3f huber=%d accept=%s wall_us=%lld round=%d",
             result.solve_us, result.iterations,
             result.initial_residual_sq, result.final_residual_sq,
             result.huber_rejects,
             accept ? "Y" : "N",
             static_cast<long long>(wall_us),
             round_id);
        // EventCounters: every completed solve counts toward total; the
        // accept gate above (converged && residual_halved && fast_enough)
        // decides accepted vs rejected. Timing fields are summed across
        // all solves, max is monotonic via CAS.
        auto& ec = navsight::eventCounters();
        ec.ba_solves_total.fetch_add(1, std::memory_order_relaxed);
        if (accept) {
            ec.ba_solves_accepted.fetch_add(1, std::memory_order_relaxed);
        } else {
            ec.ba_solves_rejected.fetch_add(1, std::memory_order_relaxed);
        }
        ec.ba_solve_us_sum.fetch_add(
            static_cast<long long>(result.solve_us),
            std::memory_order_relaxed);
        ec.update_ba_solve_us_max(static_cast<long long>(result.solve_us));
        ec.ba_iters_sum.fetch_add(
            static_cast<long long>(result.iterations),
            std::memory_order_relaxed);

        if (accept) {
            std::vector<BARefinedLandmark> refined;
            refined.reserve(features_in.size());
            for (size_t i = 0; i < features_in.size() && i < meta_in.size(); ++i) {
                BARefinedLandmark r;
                r.feature_id      = meta_in[i].feature_id;
                r.slam_slot       = meta_in[i].slam_slot;
                r.anchor_clone_id = meta_in[i].anchor_clone_id;
                r.p_world_refined = (cv::Mat_<double>(3, 1)
                    << features_in[i].p_w_out[0],
                       features_in[i].p_w_out[1],
                       features_in[i].p_w_out[2]);
                refined.push_back(std::move(r));
            }
            std::lock_guard<std::mutex> lock(ba_result_mutex_);
            ba_result_landmarks_ = std::move(refined);
            ba_result_pending_   = true;
        }

        // Release the in-flight gate AFTER publishing the result so the
        // camera thread sees pending=true before it sees in_flight=false.
        ba_in_flight_.store(false, std::memory_order_release);
    });

    return true;
}

void Tracker::consumeBAResultIfReady() {
    // Cheap atomic check — avoid taking the mutex on the (common) "no
    // result this keyframe" path. Camera thread is otherwise hot here.
    if (ba_in_flight_.load(std::memory_order_acquire)) return;

    std::vector<BARefinedLandmark> refined;
    {
        std::lock_guard<std::mutex> lock(ba_result_mutex_);
        if (!ba_result_pending_) return;
        refined = std::move(ba_result_landmarks_);
        ba_result_landmarks_.clear();
        ba_result_pending_ = false;
    }

    if (!ekf_.isFullInitialized()) return;

    // Re-seed each refined landmark by removing its old slot and adding a
    // new one anchored at the same clone. addSlamFeature rebuilds the
    // covariance entries cleanly, so the EKF observation channel stays in
    // charge of state mutation. ADR-006 forbids overwriting EKF mean /
    // covariance from a side channel; remove + add is the canonical
    // re-promotion path.
    int reseeded = 0;
    int skipped  = 0;
    for (const auto& r : refined) {
        if (r.feature_id < 0 || r.anchor_clone_id < 0) { skipped++; continue; }

        // Re-look up the slot — it may have changed since the BA was
        // launched (other features removed/added between rounds shift slot
        // indices). Use the lifecycle's current slot, not the stashed one.
        const auto* lc = feature_mgr_.getLifecycle(r.feature_id);
        if (!lc || lc->slam_slot < 0) { skipped++; continue; }
        const int cur_slot = lc->slam_slot;

        // The anchor clone may have been marginalised. Validate first.
        cv::Mat R_anchor_now, p_anchor_now;
        if (!ekf_.getClonePose(r.anchor_clone_id,
                                R_anchor_now, p_anchor_now)) {
            // Anchor gone; the EKF will retire this SLAM slot through its
            // existing demotion path. Skip.
            skipped++;
            continue;
        }
        cv::Mat R_anchor_FEJ, p_anchor_FEJ;
        if (!ekf_.getCloneFEJ(r.anchor_clone_id,
                               R_anchor_FEJ, p_anchor_FEJ)) {
            R_anchor_FEJ = R_anchor_now.clone();
            p_anchor_FEJ = p_anchor_now.clone();
        }

        // Drop the old slot, then re-add with the BA-refined world point.
        if (!ekf_.removeSlamFeature(cur_slot)) { skipped++; continue; }
        feature_mgr_.setSlamSlot(r.feature_id, -1);

        CameraPose anchor;
        anchor.R_GtoC       = R_anchor_now;
        anchor.p_G          = p_anchor_now;
        anchor.R_FEJ        = R_anchor_FEJ;
        anchor.p_FEJ        = p_anchor_FEJ;
        anchor.state_id     = r.anchor_clone_id;
        anchor.timestamp_ns = 0;  // unused by addSlamFeature

        const int new_slot = ekf_.addSlamFeature(
            r.feature_id, r.p_world_refined, anchor);
        if (new_slot < 0) {
            // Re-add failed (max features reached, depth degenerate, etc.).
            // Lifecycle slot stays at -1 so the next promotion gate run can
            // re-promote the feature normally.
            skipped++;
            continue;
        }
        feature_mgr_.setSlamSlot(r.feature_id, new_slot);
        // Re-cache the refined world point on the lifecycle so the next
        // promotion gate (if this slot ever gets demoted again) starts
        // from the BA-improved position.
        cv::Point3f p_refined(
            static_cast<float>(r.p_world_refined.at<double>(0, 0)),
            static_cast<float>(r.p_world_refined.at<double>(1, 0)),
            static_cast<float>(r.p_world_refined.at<double>(2, 0)));
        feature_mgr_.noteTriangulation(r.feature_id, p_refined,
                                       r.anchor_clone_id);
        reseeded++;
    }

    LOGI("BA: consumed refined=%d reseeded=%d skipped=%d",
         static_cast<int>(refined.size()), reseeded, skipped);
}

void Tracker::shutdownBA() {
    // Step 6 (ADR-012): wait for any in-flight round; do NOT detach. The
    // BA worker reads from ekf_ / feature_mgr_, which the destructor /
    // reset path is about to tear down — joining ensures the worker has
    // released its read locks before we touch those structures.
    if (ba_thread_.joinable()) ba_thread_.join();
    ba_in_flight_.store(false, std::memory_order_release);
}

// ─────────────────────────────────────────────────────────────────────────────
// Plan Step 7 (ADR-013): same-session loop closure (DBoW2)
// ─────────────────────────────────────────────────────────────────────────────
//
// Architecture:
//
//   Camera thread (processFrame, ~30 Hz)
//       ├── on every keyframe (~2 Hz):
//       │     loop_closure_.addKeyframe(...)
//       │     publishLoopClosureQueryKeyframe(...)  → wakes worker
//       │
//       └── on every frame:
//             consumeLoopClosureMatchIfReady()      → applies damped EKF update
//
//   Loop-closure worker thread (loopClosureWorkerLoop, 1 Hz)
//       └── while (!should_stop_):
//             cv.wait_for(LOOP_CLOSURE_QUERY_PERIOD_S)
//             snapshot pending query keyframe under loop_closure_query_mutex_
//             loop_closure_.tryDetectLoop(...)
//             on success: publish LoopMatch under loop_closure_result_mutex_
//
// Thread-safety invariants:
//   * loop_closure_ itself is internally synchronised by Agent A's
//     LoopClosureDetector implementation. The worker thread and the
//     camera thread both call into it (worker: tryDetectLoop; camera:
//     addKeyframe + isReady + getKeyframeCount).
//   * loop_closure_query_mutex_ guards the most-recent query snapshot
//     buffer. Camera thread writes once per keyframe; worker reads once
//     per query tick.
//   * loop_closure_result_mutex_ guards the LoopMatch handoff. Worker
//     publishes; camera consumes.
//   * loop_closure_active_match_ / loop_closure_damping_remaining_ are
//     touched only by the camera thread inside consumeLoopClosureMatchIfReady,
//     so they need no lock.
//   * shutdownLoopClosure() joins the worker before any of the above
//     buffers are torn down (called from reset() and ~Tracker()).

bool Tracker::loadLoopClosureVocabulary(const std::string& vocab_path) {
    // Forward to the detector. On success we lazily start the worker
    // thread — until the vocabulary loads `tryDetectLoop` would no-op
    // anyway (gated on isReady()), so deferring the thread launch keeps
    // the cold-start path light.
    const bool ok = loop_closure_.loadVocabulary(vocab_path);
    if (!ok) {
        LOGE("Tracker: loop-closure vocabulary load FAILED for %s",
             vocab_path.c_str());
        return false;
    }
    LOGI("Tracker: loop-closure vocabulary loaded from %s", vocab_path.c_str());

    // Idempotent: if we already started a worker (e.g. reload after
    // session reset reused the same Tracker instance), leave it running.
    bool expected = false;
    if (loop_closure_thread_running_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        loop_closure_should_stop_.store(false, std::memory_order_release);
        loop_closure_thread_ = std::thread(&Tracker::loopClosureWorkerLoop, this);
        LOGI("Tracker: loop-closure worker thread started (period=%.2fs, "
             "temporal_excl=%llds, damping_frames=%d)",
             LOOP_CLOSURE_QUERY_PERIOD_S,
             static_cast<long long>(LOOP_CLOSURE_TEMPORAL_EXCL_NS / 1'000'000'000LL),
             LOOP_CLOSURE_DAMPING_FRAMES);
    }
    return true;
}

void Tracker::publishLoopClosureQueryKeyframe(
        int kf_id, int64_t ts_ns,
        const cv::Mat& descriptors,
        const std::vector<cv::KeyPoint>& keypoints,
        double fx, double fy, double cx, double cy,
        double yaw_rad) {
    {
        std::lock_guard<std::mutex> lock(loop_closure_query_mutex_);
        // Cheap deep copy: descriptors at most 250×32 bytes ≈ 8 KB,
        // keypoints ≤ 250 entries. The worker takes ownership of the
        // snapshot — we cannot share with FeatureManager's deque
        // because that deque mutates from the camera thread.
        descriptors.copyTo(loop_closure_query_descriptors_);
        loop_closure_query_keypoints_ = keypoints;
        loop_closure_query_kf_id_     = kf_id;
        loop_closure_query_ts_ns_     = ts_ns;
        loop_closure_query_fx_        = fx;
        loop_closure_query_fy_        = fy;
        loop_closure_query_cx_        = cx;
        loop_closure_query_cy_        = cy;
        loop_closure_query_yaw_rad_   = yaw_rad;
        loop_closure_query_has_data_  = true;
    }
    // Wake the worker. notify_one is correct — the worker is the only
    // waiter on this cv.
    loop_closure_cv_.notify_one();
}

void Tracker::loopClosureWorkerLoop() {
    using namespace std::chrono;
    auto& ec = navsight::eventCounters();
    LOGI("LOOP_CLOSURE: worker thread entered loop");

    // Per-query scratch buffers held outside the mutex so the snapshot
    // copy under the lock is as short as possible.
    cv::Mat                   q_descriptors;
    std::vector<cv::KeyPoint> q_keypoints;
    int                       q_kf_id = -1;
    int64_t                   q_ts_ns = 0;
    double                    q_fx = 0., q_fy = 0., q_cx = 0., q_cy = 0.;
    double                    q_yaw_rad = 0.;

    while (!loop_closure_should_stop_.load(std::memory_order_acquire)) {
        // Wait either for a fresh keyframe publish or the 1 Hz timeout.
        // Predicate-form wait avoids the spurious-wakeup pitfall.
        {
            std::unique_lock<std::mutex> lock(loop_closure_query_mutex_);
            const auto wait_dur = duration_cast<steady_clock::duration>(
                duration<double>(LOOP_CLOSURE_QUERY_PERIOD_S));
            loop_closure_cv_.wait_for(lock, wait_dur, [this]() {
                return loop_closure_should_stop_.load(std::memory_order_acquire) ||
                       loop_closure_query_has_data_;
            });
            if (loop_closure_should_stop_.load(std::memory_order_acquire)) break;
            if (!loop_closure_query_has_data_) continue;  // no keyframe yet

            // Snapshot. Clear the has_data_ flag so we don't re-process the
            // same keyframe on a spurious wake; a fresh publish will set
            // it true and notify again.
            loop_closure_query_descriptors_.copyTo(q_descriptors);
            q_keypoints = loop_closure_query_keypoints_;
            q_kf_id     = loop_closure_query_kf_id_;
            q_ts_ns     = loop_closure_query_ts_ns_;
            q_fx        = loop_closure_query_fx_;
            q_fy        = loop_closure_query_fy_;
            q_cx        = loop_closure_query_cx_;
            q_cy        = loop_closure_query_cy_;
            q_yaw_rad   = loop_closure_query_yaw_rad_;
            loop_closure_query_has_data_ = false;
        }

        // Defensive bail: malformed snapshot shouldn't reach here, but
        // skip it instead of feeding the detector a degenerate input.
        if (q_descriptors.empty() || q_keypoints.empty() ||
            q_fx <= 1.0 || q_fy <= 1.0 || q_kf_id < 0) {
            continue;
        }

        ec.loop_closure_attempts.fetch_add(1, std::memory_order_relaxed);

        LoopClosureDetector::LoopMatch match;
        const bool detected = loop_closure_.tryDetectLoop(
            static_cast<uint64_t>(q_kf_id),
            q_ts_ns,
            q_descriptors, q_keypoints,
            q_fx, q_fy, q_cx, q_cy,
            LOOP_CLOSURE_TEMPORAL_EXCL_NS,
            q_yaw_rad,
            match);

        if (!detected) {
            // 2026-05-04 cpp-reviewer HIGH-1 fix: detector now owns BOTH
            // rejection counters (rejects_low_score + rejects_pnp), so the
            // worker no longer bumps them. Otherwise PnP rejections were
            // double-counted (detector bumped rejects_pnp inside, then
            // worker bumped rejects_low_score on the false return). attempts
            // and accepts stay here at the worker tick boundary because
            // they are tick-level accounting, not per-rejection-reason.
            continue;
        }

        ec.loop_closure_accepts.fetch_add(1, std::memory_order_relaxed);
        LOGI("LOOP_CLOSURE: ACCEPT now_kf=%d match_kf=%llu bow=%.3f pnp_inl=%d",
             q_kf_id,
             static_cast<unsigned long long>(match.matched_kf_id),
             match.bow_score, match.pnp_inliers);

        // Publish under the result mutex. Camera thread reads + clears
        // pending=true on its next consume call.
        {
            std::lock_guard<std::mutex> rlock(loop_closure_result_mutex_);
            loop_closure_pending_match_ = match;
            loop_closure_result_pending_ = true;
        }
    }

    LOGI("LOOP_CLOSURE: worker thread exiting cleanly");
}

void Tracker::consumeLoopClosureMatchIfReady() {
    // Step 1 — pull any newly-published match into the active slot. We
    // explicitly do NOT block when no match is pending: the lock is taken
    // for at most a few atomic loads / a single struct copy, all O(1).
    bool fresh_match_picked_up = false;
    {
        std::lock_guard<std::mutex> rlock(loop_closure_result_mutex_);
        if (loop_closure_result_pending_) {
            loop_closure_active_match_     = loop_closure_pending_match_;
            loop_closure_active_match_set_ = true;
            loop_closure_damping_remaining_ = LOOP_CLOSURE_DAMPING_FRAMES;
            loop_closure_result_pending_ = false;
            loop_closure_pending_match_  = LoopClosureDetector::LoopMatch{};
            fresh_match_picked_up = true;
        }
    }

    if (!loop_closure_active_match_set_ ||
        loop_closure_damping_remaining_ <= 0) {
        return;
    }

    // Step 2 — apply a damped relative-pose / relative-rotation update
    // through the canonical EKF measurement channel. ADR-006 forbids
    // direct mean / covariance writes from a side channel; updateRelativePose
    // and updateRelativeRotation are the same observation channels
    // Step 2 (visual relative pose) and Step 3a (MSCKF) already use.
    if (!ekf_.isFullInitialized()) {
        // Cannot inject through the EKF until full-init. Drop the ramp
        // counter so we don't accumulate 10 frames of waiting state.
        loop_closure_damping_remaining_ = 0;
        loop_closure_active_match_set_  = false;
        return;
    }

    const int matched_clone_id = static_cast<int>(
        loop_closure_active_match_.matched_kf_id);

    // Step 3 — damping schedule. strength on frame `k` of N (N=10):
    //     strength_k = 1 - k/N    →  k=0: 1.0,  k=9: 0.1
    // Inflate the measurement variance by 1/strength² so the EKF gain
    // shrinks with strength — same pattern as ADR-006 / Step 3a damping.
    const int    k          = LOOP_CLOSURE_DAMPING_FRAMES -
                              loop_closure_damping_remaining_;  // 0..N-1
    const double strength   = 1.0 - static_cast<double>(k) /
                                    static_cast<double>(LOOP_CLOSURE_DAMPING_FRAMES);
    const double strength_sq = strength * strength;
    // Floor strength_sq so the divide can never explode (k=N-1 gives
    // 0.01; the floor of 0.01 keeps var inflation bounded at 100×).
    const double damping_inv = 1.0 / std::max(1e-2, strength_sq);

    // ────────────────────────────────────────────────────────────────────
    // ADR-013 §"Correction injection — absolute pose path":
    //
    // The matched keyframe's clone is almost always older than the EKF
    // sliding window (temporal exclusion 30 s vs. ~5–10 s clone window).
    // updateRelativePose / updateRelativeRotation both REQUIRE the
    // matched clone to live in the window — when it doesn't, the
    // correction would silently drop. Use the world-frame absolute-pose
    // channel instead: it consumes a target world-frame IMU pose and
    // applies the correction directly, independent of clone availability.
    //
    // Compose the target IMU pose from the match payload:
    //
    //   target_R_world_cam = R_world_cam_match * R_now_to_match
    //   target_t_cam_world = R_world_cam_match * t_now_to_match
    //                      + t_cam_world_match
    //
    // (`R_world_cam_match` is camera→world; composing with `R_now_to_match`
    // — the cam-now → cam-match relative — chains "now-cam → match-cam →
    // world", which is the world pose of the now-camera the loop closure
    // says we should be at.)
    //
    // Convert camera→world to world→IMU using the EKF-maintained body→camera
    // extrinsic R_bc (Step 8b). Previously this was hardcoded as diag(1,-1,-1).
    //
    // R_bc is body→camera: p_cam = R_bc * p_body.
    // R_bc^T = R_bc^{-1} is camera→body.
    //
    //   target_R_GtoI = R_bc^T * target_R_world_cam.t()
    //
    // because target_R_world_cam takes cam→world; its transpose is world→cam;
    // left-multiplying by R_bc^T (camera→body) gives world→body = world→IMU.
    //
    // Position: handheld phone, body and camera are co-located (lever
    // arm absorbed into the per-frame R_vo path). World-frame IMU position
    // equals the world-frame camera position.
    // ────────────────────────────────────────────────────────────────────
    const auto& R_wc_match = loop_closure_active_match_.R_world_cam_match;
    const auto& t_cw_match = loop_closure_active_match_.t_cam_world_match;
    const auto& R_n2m      = loop_closure_active_match_.R_now_to_match;
    const auto& t_n2m      = loop_closure_active_match_.t_now_to_match;

    // target_R_world_cam (cam→world for now-cam)
    const cv::Matx33d target_R_world_cam = R_wc_match * R_n2m;
    // target world position of now-cam
    const cv::Vec3d   target_t_cam_world = R_wc_match * t_n2m + t_cw_match;

    // Step 8b: use EKF-maintained R_bc (body→camera, refined online).
    // R_bc^T is the transpose (= inverse for rotation matrices) giving camera→body.
    const cv::Matx33d R_bc_lc = ekf_.getExtrinsicsRotation();
    const cv::Matx33d target_R_GtoI_mx = R_bc_lc.t() * target_R_world_cam.t();

    cv::Mat target_R_GtoI(3, 3, CV_64F);
    cv::Mat target_p_world(3, 1, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            target_R_GtoI.at<double>(r, c) = target_R_GtoI_mx(r, c);
        }
        target_p_world.at<double>(r, 0) = target_t_cam_world[r];
    }

    // Variance: same damping schedule as the original relative-pose
    // attempt — sigma_axis_sq_R inflated by 1/strength², var_p too.
    const double sigma_axis_sq_R = (LOOP_CLOSURE_BASE_ROT_SIGMA_RAD *
                                     LOOP_CLOSURE_BASE_ROT_SIGMA_RAD) * damping_inv;
    // Dynamic translation sigma grows with path length to stay above the
    // actual VIO drift (15 %/100 m from sim_data_1778077139237).
    // Floor ensures the chi² gate is never tighter than PnP sensor noise.
    const double sigma_p = std::max(LOOP_CLOSURE_PNP_SIGMA_FLOOR_M,
                                    LOOP_CLOSURE_DRIFT_RATE * total_path_m_);
    const double var_p   = (sigma_p * sigma_p) * damping_inv;

    const bool ok = ekf_.updateAbsolutePose(target_R_GtoI, target_p_world,
                                             sigma_axis_sq_R, var_p);
    if (ok) {
        navsight::eventCounters().loop_closure_corrections_applied.fetch_add(
            1, std::memory_order_relaxed);
    }

    // Log every step of the ramp at INFO so a real loop closure shows
    // up in the trace as a coherent 10-line sequence.
    LOGI("LOOP_CLOSURE: damp k=%d/%d strength=%.2f var_p=%.4f var_R=%.4e "
         "target_p=[%.3f %.3f %.3f] match_kf=%d ok=%d (abs-pose channel)",
         k + 1, LOOP_CLOSURE_DAMPING_FRAMES, strength,
         var_p, sigma_axis_sq_R,
         target_p_world.at<double>(0),
         target_p_world.at<double>(1),
         target_p_world.at<double>(2),
         matched_clone_id,
         ok ? 1 : 0);
    if (fresh_match_picked_up) {
        LOGI("LOOP_CLOSURE: fresh match picked up (kf=%d) — beginning "
             "%d-frame damped absolute-pose ramp.",
             matched_clone_id, LOOP_CLOSURE_DAMPING_FRAMES);
    }

    --loop_closure_damping_remaining_;
    if (loop_closure_damping_remaining_ <= 0) {
        loop_closure_active_match_set_ = false;
        loop_closure_active_match_     = LoopClosureDetector::LoopMatch{};
    }
}

void Tracker::shutdownLoopClosure() {
    // Step 7 (ADR-013): mirror shutdownBA. Set should_stop, notify the
    // cv so the worker drops out of wait_for, then join. We do NOT
    // detach: the worker reads loop_closure_ / ekf_ which may be torn
    // down right after this call returns.
    loop_closure_should_stop_.store(true, std::memory_order_release);
    loop_closure_cv_.notify_all();
    if (loop_closure_thread_.joinable()) loop_closure_thread_.join();
    // Reset the running flag so a subsequent loadLoopClosureVocabulary
    // can re-launch the worker (e.g. after an in-place reset()).
    loop_closure_thread_running_.store(false, std::memory_order_release);
}
