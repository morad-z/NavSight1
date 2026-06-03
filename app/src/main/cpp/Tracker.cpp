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

// 2026-05-29 — minimum accel-integrated distance (m) before the accel-K
// calibration fires. Derivation (NOT a magic tweak): the calibration window is
// [0.3, 2.5] s after a ZUPT re-zero (~2.2 s usable). The slowest credible walk
// is ~0.5 m/s, so the window accrues ~0.5 * 2.2 ≈ 1.1 m of true travel; floor at
// 1.0 m so a slow walk calibrates while a single push-off frame (~0.1 m) cannot.
// Was 2.0 m (tuned for the run, which over-accrued) — unreachable on slow walks
// once the HP-filter velocity-decay was removed (the 2026-05-29 integrator fix).
// Shared by updateDepthFlowSpeed and updateExpansionSpeed (DRY — avoids split-brain K).
static constexpr double kAccelKMinDistM = 1.0;

// 2026-05-29 — KNOWN-DISTANCE WALK CALIBRATION (accel-K bias correction).
// The raw accel integration under-reads true distance on a steady walk because
// integrating acceleration loses the sustained cruise velocity (the well-known
// pedestrian-INS limitation — see the offline analyze_accel_speed.py zupt-mode
// result: it measured ~0.78 m/s on a walk that was faster). Measured on a tape
// ~13 m walk (v6_a, 2026-05-29): VIO read 7.31 m → correction = 13 / 7.31 ≈ 1.78.
// 2026-05-29 REVERTED to 1.0 (was 1.78). The 1.78 was applied to accel_dist
// GLOBALLY in the integrator, so it inflated the RUN's K too (not just walks) —
// the run read ~27 km/h vs a true ~20 max (user-flagged). The premise that runs
// would use the unbiased vi_speed (Step B) and bypass this correction FAILED:
// ScaleEstimatorVI produced garbage (s ~3x too small, relσ 0.35-1.38 mostly
// rejected — only 1 [vi] calib vs 252 accel-K in v7), so EVERYTHING is on accel-K
// and the walk-specific 1.78 wrongly scaled runs. The bias is gait-dependent (a
// single factor cannot fix both walk and run), AND non-portable across phones
// (user-flagged). So: NO global correction (1.0). The walk under-read returns,
// to be fixed by a PER-DEVICE, WALK-ONLY runtime calibration (design in progress)
// — NOT a hardcoded universal constant. Kept as a 1.0 hook for that wiring.
static constexpr double kAccelKBiasCorrection = 1.0;

// ── Phase 2 Step 4.2.1 (docs/study/phase2_productization_plan.md §4.2.1) ────
//
// VI-Depth Stage-1 affine fit between MiDaS disparity and an inverse-depth
// target. Verbatim port of compute_scale_and_shift_ls from
// isl-org/VI-Depth/modules/estimator.py (Wofk et al., ICRA 2023) — plain
// closed-form 2×2 weighted least squares, NO RANSAC, NO Huber. Outlier
// robustness is delegated to the caller, which inspects residuals after
// the fit (and to the existing MAD-based variance estimator further
// downstream in applyDepthScaleConstraint).
//
// Solves   target_i ≈ s · prediction_i + t   for all i.
//
// Returns cv::Vec2d(s, t), or (NaN, NaN) if N < min_points or det ≤ 0.
static cv::Vec2d fitDisparityAffine(
        const std::vector<double>& prediction,
        const std::vector<double>& target,
        int min_points = 8) {
    const size_t N = prediction.size();
    if (N != target.size() || static_cast<int>(N) < min_points) {
        return cv::Vec2d(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN());
    }
    double a00 = 0.0, a01 = 0.0, a11 = 0.0;
    double b0  = 0.0, b1  = 0.0;
    for (size_t i = 0; i < N; ++i) {
        const double p = prediction[i];
        const double y = target[i];
        a00 += p * p;
        a01 += p;
        a11 += 1.0;
        b0  += p * y;
        b1  += y;
    }
    const double det = a00 * a11 - a01 * a01;
    if (!(det > 0.0) || !std::isfinite(det)) {
        return cv::Vec2d(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN());
    }
    const double s = ( a11 * b0 - a01 * b1) / det;
    const double t = (-a01 * b0 + a00 * b1) / det;
    if (!std::isfinite(s) || !std::isfinite(t)) {
        return cv::Vec2d(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN());
    }
    return cv::Vec2d(s, t);
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

    // ── Step 4.2.1 (Phase 2): VI-Depth affine fit in disparity space ─────
    //
    // Cause: the LEGACY floor-plane path (commented out below at /* LEGACY
    // ... */) computed metric_z = camera_h / (norm_y*cos(pitch) + sin(pitch)),
    // assuming the camera was pitched down at a flat floor. For forward-
    // pointing scooter cams, chest-mount, or handheld-not-pitched, < 8
    // features pass `is_floor_by_image || is_floor_by_geom` and the whole
    // scale anchor bails out. v21 sim 32/32 keyframes bailed; v22 sim 31/31;
    // midas_fused=0 in both. The actual per-pixel MiDaS disparity signal was
    // wasted (used only as a < 0.01 validity gate).
    //
    // Change: replace the floor heuristic with VI-Depth Stage-1 (algorithm
    // from isl-org/VI-Depth/modules/estimator.py — Wofk et al. ICRA 2023).
    // For every triangulated feature i with VIO depth d_vio_i and MiDaS
    // disparity m_i sampled at the feature pixel, fit
    //
    //     inv_metric_depth_i  ≈  s · m_i + t
    //
    // by closed-form 2×2 weighted LSQ. The target is approximate-metric
    // inverse depth, derived from VIO baseline depth × current_scale: this
    // matches the plan author's intent at line 170 ("the implied metric depth
    // is 1/(s·m+t)"), correcting the plan's literal step-3 wording that
    // treated VIO depth as already metric. After the fit, predicted metric
    // depth for any feature is `1/(s·m + t)`, and `ratio = metric_z / d_vio`
    // matches the legacy data contract that `scale_fuser_` consumes.
    //
    // Falsifier: a forward-cam sim post-fix must show midas_fused > 0,
    // midas_affine_fit_singular ≈ 0, and midas_affine_fit_inlier_ratio_milli
    // ≥ 500 (≥ 50 % inliers). If midas_fused still reads zero, the residual
    // bug is either (a) DepthEstimator.kt input normalisation (research
    // found ImageNet mean/std missing) or (b) depth_map orientation not
    // matching feature pixel coords (depth is 256×256, image is fw×fh —
    // verify dw/dh axes aren't swapped vs the network's H/W axes).

    // Snapshot current scale once (used to anchor MiDaS to approximate-metric
    // space). Snapshotting avoids holding pose_mutex_ across the whole loop.
    const double current_scale_snapshot = scale_fuser_.scale();

    // Build (prediction, target) pairs:
    //   prediction_i = m_i  = MiDaS disparity at the feature pixel
    //   target_i     = 1 / (z_vio_i · current_scale_snapshot)
    //                = approximate metric inverse depth
    // Discard non-finite VIO depths, off-grid pixels, and MiDaS < 0.01
    // (network's "very far / unstable" range).
    std::vector<double> midas_disparity;
    std::vector<double> metric_inv_depth_target;
    midas_disparity.reserve(pts3d.size());
    metric_inv_depth_target.reserve(pts3d.size());

    const float fh = static_cast<float>(img_height);
    const float fw = static_cast<float>(img_width);

    // 2026-05-18 falsifier instrumentation: per-gate counters so we can see
    // which of the three filters (z_vio invalid, pixel out of bounds, midas
    // disparity < 0.01) is killing all the points. v40 walk shows N=0 (all
    // points dropped) but no log tells us which gate fired. After fix the
    // dominant counter identifies the upstream bug. Expected on a healthy
    // forward-cam walk: ~80% pass all gates.
    int n_gate_zvio = 0, n_gate_pixel = 0, n_gate_midas = 0;
    double zvio_min = 1e9, zvio_max = -1e9;
    for (size_t i = 0; i < pts3d.size(); ++i) {
        const double z_vio = static_cast<double>(pts3d[i].z);
        if (z_vio < zvio_min) zvio_min = z_vio;
        if (z_vio > zvio_max) zvio_max = z_vio;
        if (!std::isfinite(z_vio) || z_vio <= 1e-3) { n_gate_zvio++; continue; }

        const int dx = static_cast<int>((pts2d[i].x / fw) * dw);
        const int dy = static_cast<int>((pts2d[i].y / fh) * dh);
        if (dx < 0 || dx >= dw || dy < 0 || dy >= dh) { n_gate_pixel++; continue; }

        const float m = depth_copy[dy * dw + dx];
        if (m < 0.01f) { n_gate_midas++; continue; }

        midas_disparity.push_back(static_cast<double>(m));
        metric_inv_depth_target.push_back(1.0 / (z_vio * current_scale_snapshot));
    }
    LOGI("DEPTH_SCALE: gate stats pts=%zu gate_zvio=%d gate_pixel=%d gate_midas=%d "
         "passed=%zu zvio_range=[%.3f,%.3f] scale=%.5f",
         pts3d.size(), n_gate_zvio, n_gate_pixel, n_gate_midas,
         midas_disparity.size(), zvio_min, zvio_max, current_scale_snapshot);

    // Solve the global (s, t) — verbatim port of VI-Depth's
    // compute_scale_and_shift_ls (file-static helper at top of this TU).
    const cv::Vec2d st = fitDisparityAffine(midas_disparity,
                                            metric_inv_depth_target,
                                            /*min_points=*/8);
    if (!std::isfinite(st[0]) || !std::isfinite(st[1])) {
        ec_md.midas_affine_fit_singular.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT affine fit singular (det<=0 or N<8) N=%zu",
             midas_disparity.size());
        return;
    }
    const double s_fit = st[0];
    const double t_fit = st[1];

    // Publish shift as a gauge for offline tuning (event_summary JSON).
    ec_md.midas_affine_fit_shift_milli.store(
        static_cast<long long>(std::lround(t_fit * 1000.0)),
        std::memory_order_relaxed);

    // Residual-based inlier classification. Threshold = max(0.01, 3 · MAD)
    // — MAD is computed on absolute residuals so it self-adapts to scene
    // complexity (flat residuals get a tight gate, noisy residuals get a
    // loose one). The 0.01 floor avoids a coincidentally-tight fit
    // rejecting everything.
    std::vector<double> residuals;
    residuals.reserve(midas_disparity.size());
    for (size_t i = 0; i < midas_disparity.size(); ++i) {
        const double pred = s_fit * midas_disparity[i] + t_fit;
        residuals.push_back(std::abs(metric_inv_depth_target[i] - pred));
    }
    std::vector<double> sorted_res = residuals;
    std::nth_element(sorted_res.begin(),
                     sorted_res.begin() + sorted_res.size() / 2,
                     sorted_res.end());
    const double res_median   = sorted_res[sorted_res.size() / 2];
    const double res_threshold = std::max(0.01, 3.0 * res_median);

    int n_inliers = 0;
    for (const double r : residuals) {
        if (r < res_threshold) ++n_inliers;
    }
    const double inlier_ratio = static_cast<double>(n_inliers)
                              / static_cast<double>(midas_disparity.size());
    ec_md.midas_affine_fit_inlier_ratio_milli.store(
        static_cast<long long>(std::lround(inlier_ratio * 1000.0)),
        std::memory_order_relaxed);

    // Plan §4.2.1 / §4.6 acceptance bar: ≥ 50 % inliers.
    if (inlier_ratio < 0.5) {
        ec_md.midas_affine_fit_low_inliers.fetch_add(1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT low inliers ratio=%.2f s=%.4f t=%.4f "
             "N=%zu n_in=%d res_med=%.4f",
             inlier_ratio, s_fit, t_fit, midas_disparity.size(),
             n_inliers, res_median);
        return;
    }

    // 2026-05-19 Fix #12 — cache the affine fit for per-pixel depth queries.
    // Consumers (SLAM promotion, LandmarkMap add, future live-depth update)
    // call sampleMidasMetricDepth(u, v) which reads these. Update only AFTER
    // the inlier-ratio acceptance bar passes, so we don't poison consumers
    // with a low-confidence fit.
    {
        std::lock_guard<std::mutex> lock(midas_affine_mutex_);
        midas_affine_s_     = s_fit;
        midas_affine_t_     = t_fit;
        midas_affine_valid_ = true;
    }

    // Convert each inlier feature to a metric/baseline ratio. Same shape
    // as the legacy `ratio = metric_z / pts3d[i].z` so the existing MAD +
    // Kalman fuser path consumes it unchanged.
    std::vector<double> scale_ratios;
    scale_ratios.reserve(static_cast<size_t>(n_inliers));
    for (size_t i = 0; i < midas_disparity.size(); ++i) {
        if (residuals[i] >= res_threshold) continue;

        const double inv_metric_est = s_fit * midas_disparity[i] + t_fit;
        if (inv_metric_est <= 1e-4) continue;  // far/sky numerical guard

        const double metric_z = 1.0 / inv_metric_est;
        if (metric_z < 0.3 || metric_z > 12.0) continue;

        // Reconstruct z_vio (baseline-units) from the target we stored:
        //   target = 1 / (z_vio · current_scale)
        //   ⇒ z_vio = 1 / (target · current_scale)
        const double z_vio_baseline =
            1.0 / (metric_inv_depth_target[i] * current_scale_snapshot);

        const double ratio = metric_z / z_vio_baseline;
        if (ratio > 0.1 && ratio < 10.0) {
            scale_ratios.push_back(ratio);
        }
    }

    if (scale_ratios.size() < 8) {
        // Counter name retained ("few_floor_matches") for back-compat with
        // existing sim analysis tooling that diffs event_summary JSON. The
        // semantic now is "too few valid post-affine-fit ratios", not
        // "too few floor matches".
        ec_md.midas_bailout_few_floor_matches.fetch_add(
            1, std::memory_order_relaxed);
        LOGI("DEPTH_SCALE: BAILOUT only %zu valid ratios after affine fit "
             "(N=%zu n_in=%d s=%.4f t=%.4f)",
             scale_ratios.size(), midas_disparity.size(), n_inliers,
             s_fit, t_fit);
        return;
    }

    /* LEGACY (Phase 2 Step 4.2.1, 2026-05-16) — floor-plane fusion replaced
       by the VI-Depth affine fit above. Retained per project policy
       (feedback_no_deletions). Re-enabling requires reverting the new code
       and restoring the camera_h / gravity-vector setup.

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
    const double inv_g = 1.0 / g_mag;
    const double gxu = static_cast<double>(ax) * inv_g;
    const double gyu = static_cast<double>(ay) * inv_g;
    const double gzu = static_cast<double>(az) * inv_g;

    const double FLOOR_BELOW_PHONE_MARGIN_M = 0.3;
    const double floor_geom_threshold_m = FLOOR_BELOW_PHONE_MARGIN_M;

    std::vector<double> scale_ratios;
    int floor_via_image = 0;
    int floor_via_geom = 0;

    for (size_t i = 0; i < pts3d.size(); i++) {
        bool is_floor_by_image = (pts2d[i].y >= fh * 0.6f);
        double down_vio = static_cast<double>(pts3d[i].x) * gxu
                        + static_cast<double>(pts3d[i].y) * gyu
                        + static_cast<double>(pts3d[i].z) * gzu;
        double down_metric_est = down_vio * current_scale_snapshot;
        bool is_floor_by_geom = (down_metric_est > floor_geom_threshold_m);

        if (!is_floor_by_image && !is_floor_by_geom) continue;
        if (is_floor_by_image) floor_via_image++;
        else                   floor_via_geom++;

        const double pts3d_z_metric = static_cast<double>(pts3d[i].z) * current_scale_snapshot;
        if (pts3d_z_metric < 0.3 || pts3d_z_metric > 12.0) continue;

        int dx = static_cast<int>((pts2d[i].x / fw) * dw);
        int dy = static_cast<int>((pts2d[i].y / fh) * dh);
        if (dx < 0 || dx >= dw || dy < 0 || dy >= dh) continue;

        float rel_depth = depth_copy[dy * dw + dx];
        if (rel_depth < 0.01f) continue;

        double norm_y = (static_cast<double>(pts2d[i].y) - cy_) / fy_;
        double denom = norm_y * std::cos(pitch) + std::sin(pitch);
        if (denom < 0.05) continue;

        double metric_z = camera_h / denom;
        if (metric_z < 0.3 || metric_z > 10.0) continue;

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
    */

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
        // Step 4.2.1: emit affine-fit diagnostics (s, t, inlier ratio) in
        // place of the legacy (img=,geom=) floor-source split. Same cadence,
        // same log tag, so existing logcat parsers keep working.
        LOGI("DEPTH_SCALE: %s target=%.4f smooth=%.4f ratio=%.4f "
             "samples=%zu s=%.4f t=%.4f inlier_ratio=%.2f "
             "sigma=%.3f var=%.5f P=%.5f",
             accepted ? "fused" : "skipped",
             target_scale, scale_fuser_.scale(), median_ratio, N,
             s_fit, t_fit, inlier_ratio,
             sigma_ratio, median_variance, scale_fuser_.variance());
    }
}

// 2026-05-19 Fix #12 — Per-pixel MiDaS metric depth sampler.
// See Tracker.h declaration for full cause/change/falsifier writeup.
bool Tracker::sampleMidasRawDisparity(float u, float v, double& disparity_out) const {
    // 2026-05-26 — raw MiDaS disparity (relative depth = 1/disparity; high=near),
    // BEFORE the metric affine fit. Deliberately NO midas_affine_valid_ gate: this
    // works even when the affine fit bailed (e.g. few_pts3d on the walk), as long as
    // a depth map has arrived. The metric scale is supplied separately by the accel
    // calibration in updateDepthFlowSpeed (breaking the circular VIO-scale dependency).
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (depth_map_.empty() || depth_width_ <= 1 || depth_height_ <= 1) {
        return false;
    }
    // Image-pixel -> depth-grid coords (same mapping as sampleMidasMetricDepth).
    const double img_w = (cx_ > 0.0) ? (2.0 * cx_) : 640.0;
    const double img_h = (cy_ > 0.0) ? (2.0 * cy_) : 480.0;
    const double fdx = static_cast<double>(u) / img_w * static_cast<double>(depth_width_);
    const double fdy = static_cast<double>(v) / img_h * static_cast<double>(depth_height_);
    if (!std::isfinite(fdx) || !std::isfinite(fdy)) return false;
    if (fdx < 0.0 || fdx >= static_cast<double>(depth_width_ - 1) ||
        fdy < 0.0 || fdy >= static_cast<double>(depth_height_ - 1)) {
        return false;
    }
    const int x0 = static_cast<int>(std::floor(fdx));
    const int y0 = static_cast<int>(std::floor(fdy));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const double ax = fdx - x0;
    const double ay = fdy - y0;
    const float d00 = depth_map_[y0 * depth_width_ + x0];
    const float d10 = depth_map_[y0 * depth_width_ + x1];
    const float d01 = depth_map_[y1 * depth_width_ + x0];
    const float d11 = depth_map_[y1 * depth_width_ + x1];
    const double disp =
        (1.0 - ax) * (1.0 - ay) * d00 + ax * (1.0 - ay) * d10 +
        (1.0 - ax) * ay * d01 + ax * ay * d11;
    if (!std::isfinite(disp) || disp < 0.01) return false;  // <0.01 = far/unstable
    disparity_out = disp;
    return true;
}

bool Tracker::sampleMidasMetricDepth(float u, float v, double& depth_m_out) const {
    // 1. Read cached affine fit.
    double s_fit, t_fit;
    {
        std::lock_guard<std::mutex> lock(midas_affine_mutex_);
        if (!midas_affine_valid_) return false;
        s_fit = midas_affine_s_;
        t_fit = midas_affine_t_;
    }

    // 2. Read depth map snapshot (small copy avoided — keep lock briefly
    //    and sample under it; map is 256×256 typically). Bilinear interp
    //    handles sub-pixel sampling.
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (depth_map_.empty() || depth_width_ <= 1 || depth_height_ <= 1) {
        return false;
    }

    // Image-pixel → depth-grid coords. Per applyDepthScaleConstraint
    // (lines 257-258), the image pixel (u, v) is scaled by (depth_w/img_w,
    // depth_h/img_h). We don't have img_w/img_h here; the consumer passes
    // u, v already in IMAGE pixel space, but the depth_map's grid is
    // disparity at the MiDaS-resolved tile. The image-w/h is implicit in
    // the analyzer frame size; getOutputDimensions() of the analyzer
    // resolves to (640, 480) for S21 Ultra after FILL_CENTER. To keep
    // this self-contained without yet another setter, use the cached
    // intrinsics image dimensions — the optical centre (cx, cy) is
    // ~ (img_w/2, img_h/2), so 2·cx ≈ img_w. Robust at < 1 % error for
    // any reasonable intrinsics.
    const double img_w = (cx_ > 0.0) ? (2.0 * cx_) : 640.0;
    const double img_h = (cy_ > 0.0) ? (2.0 * cy_) : 480.0;

    const double fdx = static_cast<double>(u) / img_w *
                       static_cast<double>(depth_width_);
    const double fdy = static_cast<double>(v) / img_h *
                       static_cast<double>(depth_height_);
    if (!std::isfinite(fdx) || !std::isfinite(fdy)) return false;
    if (fdx < 0.0 || fdx >= static_cast<double>(depth_width_ - 1) ||
        fdy < 0.0 || fdy >= static_cast<double>(depth_height_ - 1)) {
        return false;
    }

    // Bilinear interp.
    const int x0 = static_cast<int>(std::floor(fdx));
    const int y0 = static_cast<int>(std::floor(fdy));
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const double ax = fdx - x0;
    const double ay = fdy - y0;
    const float d00 = depth_map_[y0 * depth_width_ + x0];
    const float d10 = depth_map_[y0 * depth_width_ + x1];
    const float d01 = depth_map_[y1 * depth_width_ + x0];
    const float d11 = depth_map_[y1 * depth_width_ + x1];
    const double disp =
        (1.0 - ax) * (1.0 - ay) * d00 +
        ax        * (1.0 - ay) * d10 +
        (1.0 - ax) * ay        * d01 +
        ax        * ay        * d11;

    // MiDaS < 0.01 is the network's "very far / unstable" range — same
    // gate applyDepthScaleConstraint uses (line 262).
    if (!std::isfinite(disp) || disp < 0.01) return false;

    // Apply affine fit: metric_depth = 1 / (s · disparity + t).
    const double inv_metric = s_fit * disp + t_fit;
    if (!std::isfinite(inv_metric) || inv_metric <= 1e-4) return false;

    const double depth_m = 1.0 / inv_metric;
    if (!std::isfinite(depth_m) ||
        depth_m < kMinMidasDepthM ||
        depth_m > kMaxMidasDepthM) {
        return false;
    }

    depth_m_out = depth_m;
    navsight::eventCounters().midas_depth_samples.fetch_add(
        1, std::memory_order_relaxed);
    return true;
}

void Tracker::setInitialHeading(double azimuth_rad, const IMUPreintegrator& imu) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    // Build world→body Z-up matrix for compass-CW azimuth (matches the
    // convention pinned by scripts/test_z_up_conventions.py).
    double c = std::cos(azimuth_rad), s = std::sin(azimuth_rad);
    cv::Mat new_R = (cv::Mat_<double>(3, 3) << c,-s,0, s,c,0, 0,0,1);

    if (ekf_.isFullInitialized()) {
        // 2026-05-16 v27 fix: do NOT overwrite the EKF's R_GtoI with the
        // pure-yaw new_R = Rz(azimuth). new_R has body Z = world Z (phone-
        // flat assumption) and zeroes out the roll/pitch that the EKF was
        // initialized with (from Madgwick, via the v27 fix at the
        // initializeFull call site). v26/v27 evidence: overwriting with
        // pure yaw immediately followed by visual updates produced the
        // 91° |r_R| residual at first LC.
        //
        // The heading was already pushed to Madgwick via the early-seed
        // path in SensorRepository (seedMadgwickYaw on first compass
        // reading). If Madgwick is ready, re-sync R_GtoI from it —
        // Madgwick carries the correct full orientation.
        cv::Mat R_GtoI_full = imu.getRotationGtoI();
        if (!R_GtoI_full.empty()) {
            ekf_.setRotation(R_GtoI_full);
            global_R_ = R_GtoI_full;
            LOGI("setInitialHeading: post-init R_GtoI re-synced from "
                 "Madgwick full orientation (az=%.1f deg)",
                 azimuth_rad * 180.0 / M_PI);
        } else {
            // Fix D (2026-05-16 audit): Madgwick not ready at setInitialHeading
            // post-init call site. Old code silently left EKF R_GtoI unchanged,
            // losing the azimuth update. New code: queue the azimuth for retry
            // on the next camera frame (processFrame retries under pose_mutex_).
            // Cause: if setInitialHeading fires during EKF init window and
            //   Madgwick is still settling, the compass azimuth is discarded.
            // Change: store azimuth in pending_post_init_azimuth_; processFrame
            //   retries setRotation on next camera frame.
            // Falsifier: pending_post_init_heading_set_ stays false after
            //   normal startup (consumed within 1-2 frames; if it persists,
            //   Madgwick init is broken).
            pending_post_init_azimuth_  = azimuth_rad;
            pending_post_init_heading_set_ = true;
            LOGI("setInitialHeading: post-init Madgwick not ready — "
                 "queuing az=%.1f deg for retry next camera frame",
                 azimuth_rad * 180.0 / M_PI);
        }
        scalar_heading_ = azimuth_rad;
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
    // 2026-05-26 — clear depth-flow speed on reset (no estimate until moving again).
    depth_flow_speed_mps_.store(-1.0, std::memory_order_relaxed);
    // 2026-05-28 — DO NOT reset midas_scale_K_ here: K is a slowly-varying scene
    // property (MiDaS relative-to-metric ratio), not session-scoped state. Resetting
    // it on every VIO reset forced a fresh calibration tax on each recording, which
    // we measured to vary K by 40-70% between back-to-back recordings of the same
    // motion. Persisting it lets the EMA continue to refine across recordings.
    accel_vel_w_ = cv::Vec3d(0.0, 0.0, 0.0);
    // 2026-05-29 (cold-start fix) — open the accel-K calibration window at INIT (0.0),
    // not -1.0 (closed-until-first-detected-stop). VIO starts after the init flow's
    // mandatory stationary period (initStatus WAIT_STATIONARY), so the start IS a clean
    // v=0 reference — treating it as a ZUPT lets K calibrate during the FIRST acceleration
    // instead of waiting for the first mid-ride stop. Without this, a cold start (no
    // persisted K) that doesn't stop early reads ~0 speed for the whole first segment
    // (offline sim: that was the entire ~20% cold-start drift; with the window open at
    // init it drops to the steady-state ~5%). accel_vel_w_ is zeroed just above, so the
    // assumed-zero start velocity is correct for the stationary init. Persisted K already
    // covers subsequent rides; this makes the FIRST-ever ride correct too.
    secs_since_zupt_ = 0.0;
    accel_dist_accum_ = 0.0;
    visual_rel_dist_accum_ = 0.0;
    visual_rel_dist_loom_ = 0.0;   // 2026-05-29 — looming's separate accumulator
    // 2026-05-31 — re-arm cold fast-converge each session. K itself is NOT reset (it
    // persists across recordings, see note below), so the first accepted calib of a new
    // session treats the persisted seed as uncalibrated and fast-converges to the live
    // k_obs instead of crawling at α=0.05 (fixes the 0.68x cold-walk under-read).
    cold_fast_converge_armed_ = false;
    // NOTE: expansion_scale_K_ (K_loom) is NOT reset here, same as midas_scale_K_:
    // both are slowly-varying scene scales persisted across resets/recordings.
    accel_drift_lp_ = cv::Vec3d(0.0, 0.0, 0.0);
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
    // 2026-05-13 Phase 1 Step 5: clear pose graph and its id mapping on
    // reset. The pose graph carries cross-session-incompatible state
    // (node positions in the previous session's world frame) so a fresh
    // session must start from empty.
    pose_graph_.reset();
    clone_id_to_pg_node_.clear();
    pg_node_to_clone_id_.clear();
    last_pg_node_id_ = -1;
    // 2026-05-16 Phase 1 Step 6: clear LandmarkMap on reset. Landmark
    // positions are anchored to the previous session's world frame and
    // keyframe id-space — keeping them across a reset would silently
    // contaminate the next session's map.
    landmark_map_.reset();
    // 2026-05-16 Phase 1 Step 6.4: drop accepted-this-frame snapshot so the
    // next session's overlay starts from an empty set (otherwise the JNI
    // overlay would briefly draw orange dots from the previous walk).
    {
        std::lock_guard<std::mutex> lk(last_observed_mutex_);
        last_observed_landmark_ids_.clear();
        last_observed_landmark_pixels_.clear();
        last_observed_landmark_feature_ids_.clear();
    }
    // 2026-05-12: visual_map_.clear() reverted alongside the VisualMap
    // member revert above. See Tracker.h scaffold note.
    // visual_map_.clear();
    heading_initialized_ = false;
    scalar_heading_ = 0.0;
    total_path_m_ = 0.0;
    path_since_last_lc_m_ = 0.0;  // Phase 1 Step 3 — reset drift estimate
    loop_closure_query_yaw_rad_ = 0.0;
    pending_init_heading_set_ = false;
    pending_init_heading_ = 0.0;
    // Fix D (2026-05-16): reset post-init deferred azimuth queue on session reset.
    pending_post_init_heading_set_ = false;
    pending_post_init_azimuth_ = 0.0;
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
int Tracker::getCorrectedTrajectory(float* out_xz, int max_pairs) const {
    // 2026-05-26 — #2 loop-overlay path redraw. Copies the corrected pose-graph
    // node polyline (x=East -> VioData x, y=North -> VioData z) snapshotted after
    // the last optimize() in consumeLoopClosureMatchIfReady. Called from the UI
    // thread; corrected_traj_xz_ is written under pose_mutex_.
    if (out_xz == nullptr || max_pairs <= 0) return 0;
    std::lock_guard<std::mutex> lock(pose_mutex_);
    const int n = std::min(static_cast<int>(corrected_traj_xz_.size()), max_pairs);
    for (int i = 0; i < n; ++i) {
        out_xz[2 * i]     = corrected_traj_xz_[i].x;  // East  -> VioData x
        out_xz[2 * i + 1] = corrected_traj_xz_[i].y;  // North -> VioData z
    }
    return n;
}
double Tracker::getFusedSpeedMps() const {
    // 2026-05-28 — return the TRAJECTORY-applied speed, not the depth-flow estimate.
    //
    // Cause of UI bug (2026-05-28 walk): on slow walks with close scenes the
    //   essential-matrix verification fails (inlier ratio <0.25, trans_degen=1),
    //   so updateDepthFlowSpeed and updateExpansionSpeed never fire and
    //   depth_flow_speed_mps_ stays at the -1 warm-up sentinel. Returning that
    //   to the UI made currentSpeedKmh = 0 while the trajectory was still
    //   moving via the PDR fallback path — speedometer and dot disagreed.
    //
    // Change: the trajectory writes trajectory_speed_mps_ at every accumulation
    //   site (main camera/depth-flow path, PDR fallback, static/rotation-frozen
    //   branches). The UI reads THIS atomic so the displayed speed always
    //   reflects what's actually driving the trajectory the user sees on the map.
    return trajectory_speed_mps_.load(std::memory_order_relaxed);
}

void Tracker::setMidasScaleK(double k) {
    // 2026-05-28 — Kotlin pushes the persisted K here at app start (before any
    // recording). Only accept positive values so a -1 sentinel from
    // SharedPreferences "key missing" never clobbers an in-progress calibration.
    // 2026-05-29 — seed BOTH the depth-flow K (midas_scale_K_) AND the looming K
    // (expansion_scale_K_) from the one persisted value, so a cold-start walk
    // (where only looming runs) shows speed from frame 1 instead of waiting for a
    // fresh calibration. The two K's agree for forward motion (the calibration
    // regime), so one persisted seed is a valid warm start for both; each then
    // refines independently from its own relative-distance basis during the session.
    if (std::isfinite(k) && k > 0.0) {
        midas_scale_K_.store(k, std::memory_order_relaxed);
        expansion_scale_K_.store(k, std::memory_order_relaxed);
    }
}

double Tracker::getMidasScaleK() const {
    // 2026-05-29 — return whichever K calibrated. On runs the depth-flow K leads;
    // on slow walks only the looming K calibrates (depth-flow's recoverPose path
    // bails). Persisting the calibrated one keeps the next cold start seeded.
    const double k_df = midas_scale_K_.load(std::memory_order_relaxed);
    if (k_df > 0.0) return k_df;
    return expansion_scale_K_.load(std::memory_order_relaxed);
}

// 2026-05-31 (map-as-sensor HEADING leg) — the Kotlin matcher pushes the matched road's bearing
// (deg, 0=N CW) here whenever it is confidently railed on a straight road (FREE_ROAD); processFrame
// nudges the Madgwick heading toward it (gated) so the trajectory stops drifting off the road.
// Pushed ~2 Hz from the snap loop; frames_since_road_hint_ ages it so a paused matcher stops it.
void Tracker::setRoadHeadingHint(double bearing_deg) {
    road_heading_hint_deg_.store(bearing_deg, std::memory_order_relaxed);
    frames_since_road_hint_.store(0, std::memory_order_relaxed);
}

// Map-as-sensor POSITION leg setter (JNI thread) — store the latest world-frame error vector
// (matched-ball − raw-VIO, metres east/north) and zero its age so the camera thread applies it.
void Tracker::setMapPositionCorrection(double d_east_m, double d_north_m) {
    map_pos_err_east_.store(d_east_m, std::memory_order_relaxed);
    map_pos_err_north_.store(d_north_m, std::memory_order_relaxed);
    frames_since_map_pos_.store(0, std::memory_order_relaxed);
}

void Tracker::setMapPositionEnabled(bool enabled) {
    map_pos_correction_enabled_ = enabled;
    LOGI("MAP_POS_ENABLE: position-leg %s", enabled ? "ON" : "OFF");
}

// ── Scale fix Step 2 — gait classifier (camera thread) ──────────────────────────
// docs/SCALE_FIX_DESIGN_2026_05_30.md §3 Step 2. Thresholds are physics-derived,
// NOT tuned magic numbers:
//   • VEHICLE: no step for >3 s while still moving (accel_speed>0.5 m/s) ⇒ riding,
//     not walking. The 0.5 m/s floor = kAccelKMinDistM(1.0 m) / the ~2.2 s usable
//     calibration window — below it the accel-K window cannot accrue a metre.
//   • RUN: step cadence > 2.7 Hz. Running cadence is 2.7-4 Hz (textbook); 2.7 sits
//     0.2 Hz above the 2.5 Hz top of normal walking (1.5-2.5 Hz), giving a
//     hysteresis gap. The step detector caps cadence at 4 Hz (MIN_STEP_PERIOD_S).
//   • WALK: cadence ≥ 0.6 Hz (a slow walk; MAX_STEP_PERIOD_S=1.5 s ≈ 0.67 Hz).
//   • else: hold the current mode (cold start / ambiguous — no spurious switch).
// A candidate must persist kModeHoldFrames (~0.5 s @ 30 Hz) before the slot swaps.
Tracker::GaitMode Tracker::classifyGait(const IMUPreintegrator& imu, double accel_speed) {
    const auto si = imu.getStepInfo();
    GaitMode cand;
    // 2026-05-31 (FIX B Part 2) — VEHICLE veto: also require ABSENCE of recent cadence
    // (step_freq_hz < 0.6). A walk pause/turn freezes last_step_ns_ (detectStep gyro-gates
    // step detection), so time_since_last_step_s climbs past 3 s while accel drift keeps
    // accel_speed > 0.5 → walking gets mislabeled VEHICLE (val_2026_05_31_pm: 8/17 df
    // calibs landed at gait=2 on walking-pace k_obs ~920-968, starving the WALK slot).
    // 0.6 reuses the WALK floor below. Paired with the getStepInfo() staleness fix, a TRUE
    // ride reports step_freq_hz=0 (<0.6) within MAX_STEP_PERIOD_S and STILL promotes to
    // VEHICLE — a bare step-freq veto alone would pin a walk→ride in WALK forever.
    // OLD: if (si.time_since_last_step_s > 3.0 && accel_speed > 0.5) cand = GaitMode::VEHICLE;
    const bool vehicle_cand = (si.time_since_last_step_s > 3.0 && accel_speed > 0.5);
    if (vehicle_cand && si.step_freq_hz < 0.6)               cand = GaitMode::VEHICLE;
    else if (si.step_freq_hz > 2.7)                          cand = GaitMode::RUN;
    else if (si.step_freq_hz >= 0.6)                         cand = GaitMode::WALK;
    else                                                     cand = active_mode_;  // hold on cold/ambiguous
    if (vehicle_cand && si.step_freq_hz >= 0.6) {  // VEHICLE candidate blocked by recent cadence
        navsight::eventCounters().gait_vehicle_suppressed.fetch_add(1, std::memory_order_relaxed);
        if (frame_counter_ % 30 == 0)
            LOGI("GAIT_VEHICLE_SUPPRESS: tsls=%.2f step_freq=%.2f accel_spd=%.2f held=%d",
                 si.time_since_last_step_s, si.step_freq_hz, accel_speed, static_cast<int>(active_mode_));
    }

    // Hysteresis: only switch after kModeHoldFrames consecutive candidate frames.
    if (cand == active_mode_) { mode_hold_frames_ = 0; last_gait_frame_ = frame_counter_; return active_mode_; }
    // Advance the hold counter at most ONCE per processFrame (classifyGait is called
    // from both speed paths in the same frame). Without this, kModeHoldFrames would
    // count speed-path invocations, not frames, and halve the ~0.5 s debounce on
    // frames where both paths fire.
    if (last_gait_frame_ == frame_counter_) return active_mode_;
    last_gait_frame_ = frame_counter_;
    if (++mode_hold_frames_ >= kModeHoldFrames) {
        const GaitMode old_mode = active_mode_;
        active_mode_ = cand;
        mode_hold_frames_ = 0;
        onModeSwitch(old_mode, cand);  // save old slot, load/seed new slot (Step 3)
    }
    return active_mode_;
}

// ── Scale fix Step 3 — mode-switch slot swap + fast-converge EMA ─────────────────
// docs/SCALE_FIX_DESIGN_2026_05_30.md §3 Step 3. The run-overshoot root cause was a
// stale K inherited across the EMA on a walk→run transition. On every switch:
//   1) SAVE the active mode's live K into the OLD slot (so a calibrated WALK K is
//      captured the first time we leave WALK — k_slots_[WALK] is otherwise unset).
//   2) LOAD the NEW slot into the live K. A virgin slot (df<0) is seeded
//      conservatively from the WALK slot (RUN = walk*0.62 measured ratio; VEHICLE =
//      walk as a coarse start) — NEVER inheriting the prior mode's raw K.
//   3) Arm the fast-converge EMA (kFastConvergeFrames).
// We touch ONLY the K state + mode counters here. The physical accumulators
// (accel_dist_accum_ / visual_rel_dist_*) keep running — only a ZUPT resets those —
// so the new mode calibrates as soon as it accrues distance (TRAP CHECKLIST #4).
void Tracker::onModeSwitch(GaitMode old_mode, GaitMode new_mode) {
    KSlot& old_s = k_slots_[static_cast<int>(old_mode)];
    KSlot& new_s = k_slots_[static_cast<int>(new_mode)];

    // (1) Save the currently-active K into the old slot.
    old_s.df   = midas_scale_K_.load(std::memory_order_relaxed);
    old_s.loom = expansion_scale_K_.load(std::memory_order_relaxed);

    // (2) Load (or seed) the new slot into the live, atomic active K.
    if (new_s.df > 0.0) {
        midas_scale_K_.store(new_s.df, std::memory_order_relaxed);
        expansion_scale_K_.store(new_s.loom, std::memory_order_relaxed);
    } else {
        // Virgin slot — seed from the WALK estimate (use the current live K as the
        // walk estimate when the WALK slot itself is still empty), never raw-inherit.
        double w = k_slots_[static_cast<int>(GaitMode::WALK)].df;
        if (w <= 0.0) w = midas_scale_K_.load(std::memory_order_relaxed);
        if (w > 0.0) {
            const double seed = (new_mode == GaitMode::RUN) ? (w * kRunWalkSeedRatio) : w;
            midas_scale_K_.store(seed, std::memory_order_relaxed);
            expansion_scale_K_.store(seed, std::memory_order_relaxed);
            new_s.df = seed;     // leave the seed in BOTH the live K and the slot
            new_s.loom = seed;
        }
    }

    mode_switch_fast_alpha_frames_ = kFastConvergeFrames;
    LOGI("GAIT_MODE_SWITCH: old=%d new=%d K=%.1f",
         static_cast<int>(old_mode), static_cast<int>(new_mode),
         midas_scale_K_.load(std::memory_order_relaxed));
}

void Tracker::updateDepthFlowSpeed(const std::vector<cv::Point2f>& prev_ud,
                                   const std::vector<cv::Point2f>& next_ud,
                                   const cv::Mat& R_vo, const cv::Mat& t_vo,
                                   double dt_s,
                                   const IMUPreintegrator& imu, double gyro_norm) {
    // ── Depth-weighted metric speed from tracked feature points ─────────────────
    // Cause: recoverPose gives camera rotation R_vo and a UNIT translation
    //   direction t_vo — the metric magnitude is unobservable from a monocular
    //   essential matrix alone (classic scale ambiguity). The displayed speed has
    //   been wrong because it ultimately rode on a single weakly-observable global
    //   scale (or the pedestrian stride model), and the EKF velocity v_G_ diverges.
    // Change: recover the one missing scalar s (metric camera displacement this
    //   frame, |T| = s) by requiring each tracked point — back-projected to its
    //   MiDaS metric depth — to reproject onto where it is actually observed next
    //   frame:
    //       P_prev = Z · [ (u-cx)/fx, (v-cy)/fy, 1 ]   (Z = MiDaS depth, prev cam)
    //       project( R_vo · P_prev + s · t_vo ) = observed next pixel
    //   Each point gives a linear equation in s; take the ROBUST MEDIAN of the
    //   per-point estimates (rejects KLT mismatches + bad-depth outliers). The
    //   depth prior is what makes scale observable even at constant velocity
    //   (where IMU/triangulation scale degenerates). Independent of v_G_ and of
    //   the global appliedScale. speed = |s| / dt, EMA-smoothed.
    //   Refs: Longuet-Higgins & Prazdny 1980 (motion field); depth-prior scale
    //   recovery, VI-Depth (Wofk et al. ICRA 2023).
    // Falsifier: on a ride, depth_flow_updates climbs while moving, the reported
    //   speed tracks GPS and reads ~0 at true stops. If it reads 0 while clearly
    //   moving, MiDaS depth is unavailable (depth_flow_skipped_few_pts climbs) —
    //   look there, do NOT add a fudge factor.
    if (dt_s <= 1e-4 || R_vo.empty() || t_vo.empty() ||
        R_vo.rows != 3 || R_vo.cols != 3 || t_vo.rows != 3) {
        return;
    }
    if (fx_ <= 0.0 || fy_ <= 0.0) return;
    const double tx = t_vo.at<double>(0);
    const double ty = t_vo.at<double>(1);
    const double tz = t_vo.at<double>(2);

    // 2026-05-29 — Step A per-point affine DISABLED on the speed path (it was the
    // ~5x-undershoot poison). Why: the affine (s,t) from applyDepthScaleConstraint
    // is fit against target = 1/(z_vio * scale_fuser_.scale()) (Tracker.cpp ~265),
    // and scale_fuser_ is stuck at the rejected-PDR seed ~0.10. So the affine's
    // "metric" depth = 1/(s*disp+t) ≈ z_vio*0.10 — in scale_fuser units, ~5x too
    // small. Treating it as "already metric" (the Step A branch) made walks read
    // ~1.2 km/h vs ~5. The affine ALSO cannot add scene-invariance here: with only
    // the noisy VIO triangulation (z_vio in [0,49]) as a per-point metric reference
    // the fit degenerates to s≈0 (flat depth). The ONLY trustworthy metric anchor
    // is accel-K (IMU-derived; K=519 read the run correctly). So the speed path now
    // ALWAYS uses relative MiDaS depth (Z=1/disp_raw) × K. Scene transfer is handled
    // by K recalibrating per-scene on each stop-go (accel anchor), not a per-frame
    // affine. The affine/applyDepthScaleConstraint stays for its SLAM consumers.
    // Snapshot kept commented for an easy revert if a true per-point metric depth
    // source (not scale_fuser-anchored) is ever wired.
    /* LEGACY 2026-05-28 Step A affine-on-speed-path (scale_fuser-poisoned):
    double s_aff = 0.0, t_aff = 0.0;
    bool aff_valid_now = false;
    {
        std::lock_guard<std::mutex> lock(midas_affine_mutex_);
        s_aff = midas_affine_s_;
        t_aff = midas_affine_t_;
        aff_valid_now = midas_affine_valid_;
    }
    */
    constexpr bool aff_valid_now = false;   // affine off the speed path (see above)
    const double s_aff = 0.0, t_aff = 0.0;  // logged only; unused while aff_valid_now=false

    const size_t n = std::min(prev_ud.size(), next_ud.size());
    std::vector<double> s_est, z_est, flow_est;   // parallel: per-point scale, MiDaS depth, pixel flow
    s_est.reserve(n); z_est.reserve(n); flow_est.reserve(n);
    int n_no_depth = 0;   // tracked points dropped because MiDaS depth was unavailable/implausible
    constexpr double kMinCoef = 1e-3;   // ignore equations with ~no sensitivity to s
    for (size_t i = 0; i < n; ++i) {
        // 2026-05-26 — depth from MiDaS. RELATIVE if affine fit invalid (K
        // applied later); METRIC if affine valid (s_med becomes direct meters).
        // Mask the sky strip (upper 18% of image — MiDaS unstable there).
        const double img_h_mask = (cy_ > 0.0) ? (2.0 * cy_) : 480.0;
        if (prev_ud[i].y < 0.18 * img_h_mask) { ++n_no_depth; continue; }
        double disp_raw = 0.0;
        if (!sampleMidasRawDisparity(prev_ud[i].x, prev_ud[i].y, disp_raw) ||
            disp_raw < 0.02) {   // <0.02 disparity = far/unstable
            ++n_no_depth;
            continue;
        }
        double Z;
        if (aff_valid_now) {
            // METRIC depth (scale_fuser units). Scene-invariant per-point.
            const double inv_m = s_aff * disp_raw + t_aff;
            if (inv_m <= 1e-4) { ++n_no_depth; continue; }
            Z = 1.0 / inv_m;
        } else {
            // RELATIVE depth. K applied at output.
            Z = 1.0 / disp_raw;
        }
        // Back-project prev pixel using relative depth, rotate to cur frame.
        const double xn = (prev_ud[i].x - cx_) / fx_;
        const double yn = (prev_ud[i].y - cy_) / fy_;
        cv::Mat Pp = (cv::Mat_<double>(3, 1) << Z * xn, Z * yn, Z);
        cv::Mat A = R_vo * Pp;
        const double Ax = A.at<double>(0), Ay = A.at<double>(1), Az = A.at<double>(2);
        const double uo = next_ud[i].x, vo = next_ud[i].y;
        // Two linear equations in s; use the better-conditioned one per point.
        //   (uo-cx)*(Az + s*tz) = fx*(Ax + s*tx)  ⇒  s*coef_u = rhs_u
        const double coef_u = (uo - cx_) * tz - fx_ * tx;
        const double rhs_u  = fx_ * Ax - (uo - cx_) * Az;
        const double coef_v = (vo - cy_) * tz - fy_ * ty;
        const double rhs_v  = fy_ * Ay - (vo - cy_) * Az;
        double s_i;
        if (std::abs(coef_u) >= std::abs(coef_v)) {
            if (std::abs(coef_u) < kMinCoef) continue;
            s_i = rhs_u / coef_u;
        } else {
            if (std::abs(coef_v) < kMinCoef) continue;
            s_i = rhs_v / coef_v;
        }
        if (!std::isfinite(s_i)) continue;
        const double dfx = next_ud[i].x - prev_ud[i].x;
        const double dfy = next_ud[i].y - prev_ud[i].y;
        s_est.push_back(s_i);
        z_est.push_back(Z);
        flow_est.push_back(std::sqrt(dfx * dfx + dfy * dfy));
    }

    constexpr size_t kMinPts = 6;   // quorum for a trustworthy median
    if (s_est.size() < kMinPts) {
        navsight::eventCounters().depth_flow_skipped_few_pts.fetch_add(
            1, std::memory_order_relaxed);
        return;
    }
    std::sort(s_est.begin(), s_est.end());
    const double s_med  = s_est[s_est.size() / 2];
    const double disp_rel = std::abs(s_med);   // relative displacement this frame (proportional to metres / K)
    const double speed_rel = disp_rel / dt_s;   // RELATIVE speed (arbitrary units, proportional to true speed)

    // ── Calibrate the relative->metric scale K from the accelerometer ────────────
    // K = true_metric_speed / speed_rel. The true speed comes from accel_vel_w_
    // (integrated world accel) but ONLY in the clean window right after a ZUPT stop,
    // where short-window accel integration is reliable (drift grows after ~2.5 s,
    // proven offline). EMA-smoothed (K is a slowly-varying scale). This BREAKS the
    // circular dependency that collapsed MiDaS metric depth — no VIO/affine scale is
    // used. Basis: VINS-Mono / VI-Depth (Wofk 2023) velocity-alignment.
    const double accel_speed = std::hypot(accel_vel_w_[0], accel_vel_w_[1]);  // horizontal m/s
    // ── Scale fix Step 2 — classify gait BEFORE loading/using cur_k, so the calib
    //    below reads/writes THIS gait's slot (midas_scale_K_ == active mode's K).
    //    classifyGait may onModeSwitch() here (load/seed the new slot + arm fast EMA).
    active_mode_ = classifyGait(imu, accel_speed);
    // ── Step B (vi_speed → K_df) DISABLED 2026-05-29 — ScaleEstimatorVI is not robust.
    //    Proven OFFLINE (scripts/test_scale_estimator.py) AND by an independent numpy
    //    replication (debug workflow): the per-pair Hesch-Martinelli solve suffers
    //    ERRORS-IN-VARIABLES dilution — the scale column a_col = R*t_vis is built from
    //    the NOISY recoverPose unit direction, so OLS attenuates s toward zero (1% dir
    //    noise → s drops 2.5x; 5% → collapse). The frame fix (camera→body, kept above)
    //    is necessary and correct but NOT sufficient: even with strong scooter-like
    //    accel/brake excitation and long (sub-sampled) baselines, s/s_true stayed 0.01-0.09
    //    across all noise levels. Total-Least-Squares over-corrected (s ~6e4). And relσ
    //    CANNOT gate it out (a confidently-WRONG diluted s passes: relσ 0.07 with s 2.5x
    //    off). So vi_speed is untrustworthy as a K anchor → do NOT feed it into K.
    //    The robust scale source is accel-K below (a ratio of ACCUMULATED magnitudes, no
    //    per-pair regression dilution); its residual gait/mode bias is handled by a
    //    per-device, per-mode known-distance calibration (the real path forward).
    //    The frame-fixed solver + OBS_C log are KEPT (diagnostic / future TLS-structured
    //    estimator), they just no longer drive K. (void the unused freshness members.)
    (void)vi_metric_speed_mps_; (void)vi_speed_ts_ns_; (void)cur_frame_ts_ns_;
    const bool vi_calibrated = false;
    /* LEGACY 2026-05-29 Step B vi->K_df (errors-in-variables diluted, see above):
    const double vi_speed = vi_metric_speed_mps_.load(std::memory_order_relaxed);
    const long long vi_ts = vi_speed_ts_ns_.load(std::memory_order_relaxed);
    const double vi_age_s = (cur_frame_ts_ns_ > 0 && vi_ts > 0)
        ? (cur_frame_ts_ns_ - vi_ts) * 1e-9 : 1e9;
    if (vi_speed > 0.0 && vi_age_s >= 0.0 && vi_age_s < 1.5 && speed_rel > 1e-4) {
        const double k_obs = vi_speed / speed_rel;
        if (std::isfinite(k_obs) && k_obs > 0.0) {
            const double cur_k = midas_scale_K_.load(std::memory_order_relaxed);
            const double new_k = (cur_k <= 0.0) ? k_obs : (0.95 * cur_k + 0.05 * k_obs);
            midas_scale_K_.store(new_k, std::memory_order_relaxed);
            navsight::eventCounters().depth_flow_calib_updates.fetch_add(1, std::memory_order_relaxed);
            vi_calibrated = true;
        }
    }
    */

    // ── DEPTH-FLOW K (midas_scale_K_) — accel-K fallback when VINS-Mono is stale
    //    (slow walk / degenerate recoverPose). Calibrated from THIS path's own
    //    relative measure (disp_rel = recoverPose translation scale) and consumed
    //    below as K * speed_rel. Self-consistent (same disp_rel basis in numerator +
    //    K's denominator) — the run-proven path. Looming keeps a SEPARATE K
    //    (expansion_scale_K_) from its own vz_rel basis (no cross-basis / double-count).
    //    Skipped when the VINS-Mono anchor already calibrated this frame (preferred).
    visual_rel_dist_accum_ += disp_rel;
    // 2026-05-30 (Scale fix Step 5) — TURN SUPPRESSION. Turning lowers the forward
    // vis_rel (recoverPose translation scale) while accel_dist keeps accruing, so
    // k_obs inflates toward the measured UTURN≈1652 and would drag K_walk up. Skip
    // the calibration while |gyro| ≥ 0.5 rad/s (≈29°/s) — milder than the existing
    // kGyroGateRadS=1.2 ZUPT gate, but well above straight-walk yaw RMS (~0.17 rad/s
    // ≈10°/s), so straight-line calibration is unchanged.
    constexpr double kTurnSuppressGyroRadS = 0.5;  // rad/s; physics: above walk-yaw RMS, below in-place turn
    const bool turning = (gyro_norm >= kTurnSuppressGyroRadS);
    if (!vi_calibrated &&
        secs_since_zupt_ >= 0.3 && secs_since_zupt_ <= 2.5 &&
        accel_dist_accum_ > kAccelKMinDistM && visual_rel_dist_accum_ > 1e-4 &&
        !turning) {
        const double k_obs = accel_dist_accum_ / visual_rel_dist_accum_;
        // 2026-05-29 — BLOW-UP GUARD: when the visual relative distance goes near-zero
        // (weak/degenerate flow) while accel_dist accrued, k_obs explodes (v7 looming
        // hit 22534 vs normal ~1800 → K=10602 → 27 km/h spike). Reject k_obs that is a
        // >3x outlier vs the current K (same pattern as Observer A's 2.5x reject). The
        // first calibration (cur_k<=0) is unguarded — seeded by the median bootstrap.
        const double cur_k = midas_scale_K_.load(std::memory_order_relaxed);
        const bool k_outlier = (cur_k > 0.0) && (k_obs > 3.0 * cur_k || k_obs < cur_k / 3.0);
        if (k_outlier && frame_counter_ % 30 == 0) {
            LOGI("ACCEL_K_CALIB[df]: REJECT outlier k_obs=%.1f vs cur_K=%.1f (vis_rel=%.5f) gait=%d",
                 k_obs, cur_k, visual_rel_dist_accum_, static_cast<int>(active_mode_));
        }
        if (std::isfinite(k_obs) && k_obs > 0.0 && !k_outlier) {
            // 2026-05-31 (FIX A — cold fast-converge arm). A pure walk on a device with a
            // STALE PERSISTED K (setMidasScaleK left cur_k>0) never hits onModeSwitch, so
            // it crawls at kNormalAlpha and cannot reach the live k_obs in a short walk
            // (val_2026_05_31_pm: 13m walk read 0.68x; K crawled 745->897 vs observed
            // ~1340). Arm the EXISTING fast window ONCE per session at the first accepted
            // calib so the persisted seed is treated like a slot seed. No new tunable
            // (reuses kFastConvergeFrames); sits inside the !k_outlier guard so a
            // degenerate k_obs still cannot arm. Heading-safe: only the EMA blend weight
            // on midas_scale_K_ (a SPEED scale) changes — no rotation/azimuth state.
            // 2026-05-31 (a.3) — gate the cold-arm OFF for VEHICLE: forward-motion degeneracy makes
            // the vehicle k_obs the LEAST trustworthy, and the fast-α(0.3) blend amplified a noisy
            // k_obs into the 785→4006 K ratchet on the scooter sims (project_scooter_speed_rootcause).
            // The cold-arm was built for the pedestrian stale-persisted-seed under-read (WALK/RUN).
            if (!cold_fast_converge_armed_ && cur_k > 0.0 && active_mode_ != GaitMode::VEHICLE) {
                mode_switch_fast_alpha_frames_ = kFastConvergeFrames;
                cold_fast_converge_armed_ = true;
                navsight::eventCounters().cold_fast_converge_armed.fetch_add(1, std::memory_order_relaxed);
                LOGI("ACCEL_K_COLD_ARM[df]: cur_K=%.1f k_obs=%.1f gait=%d fast_frames=%d",
                     cur_k, k_obs, static_cast<int>(active_mode_), mode_switch_fast_alpha_frames_);
            }
            // 2026-05-30 (Scale fix Step 3) — fast-converge EMA for the first
            // kFastConvergeFrames after a mode switch (so the new slot leaves its
            // seed quickly); otherwise the steady-state α (== legacy 0.05). On a
            // pure walk (no switch) fast frames are 0 → α=0.05 exactly as before.
            double alpha = (mode_switch_fast_alpha_frames_ > 0) ? kFastAlpha : kNormalAlpha;
            // Decrement at most once per frame: df + loom both calibrate on a
            // verification_ok frame, and a naive decrement in each would halve the
            // intended kFastConvergeFrames window. Mirror the last_gait_frame_ guard.
            if (mode_switch_fast_alpha_frames_ > 0 &&
                last_fast_alpha_frame_ != frame_counter_) {
                --mode_switch_fast_alpha_frames_;
                last_fast_alpha_frame_ = frame_counter_;
            }
            const double new_k = (cur_k <= 0.0) ? k_obs : ((1.0 - alpha) * cur_k + alpha * k_obs);
            midas_scale_K_.store(new_k, std::memory_order_relaxed);
            navsight::eventCounters().depth_flow_calib_updates.fetch_add(1, std::memory_order_relaxed);
            LOGI("ACCEL_K_CALIB[df]: k_obs=%.1f accel_dist=%.2fm vis_rel=%.4f tsz=%.2fs "
                 "cur_K=%.1f -> new_K=%.1f gait=%d", k_obs, accel_dist_accum_,
                 visual_rel_dist_accum_, secs_since_zupt_, cur_k, new_k,
                 static_cast<int>(active_mode_));
            const long long k_milli = static_cast<long long>(new_k * 1000.0 + 0.5);
            navsight::eventCounters().midas_scale_k_milli.store(k_milli, std::memory_order_relaxed);
            const long long kmax = navsight::eventCounters().midas_scale_k_max_milli.load(std::memory_order_relaxed);
            if (k_milli > kmax) navsight::eventCounters().midas_scale_k_max_milli.store(k_milli, std::memory_order_relaxed);
            const long long kmin = navsight::eventCounters().midas_scale_k_min_milli.load(std::memory_order_relaxed);
            if (kmin == 0 || k_milli < kmin) navsight::eventCounters().midas_scale_k_min_milli.store(k_milli, std::memory_order_relaxed);
        }
    } else if (turning && !vi_calibrated &&
               secs_since_zupt_ >= 0.3 && secs_since_zupt_ <= 2.5 &&
               accel_dist_accum_ > kAccelKMinDistM && visual_rel_dist_accum_ > 1e-4 &&
               frame_counter_ % 30 == 0) {
        // Scale fix Step 5 — would have calibrated, but suppressed by the turn gate.
        LOGI("ACCEL_K_CALIB[df]: SKIP turn gyro_norm=%.3f>=0.5 accel_dist=%.2fm vis_rel=%.4f gait=%d",
             gyro_norm, accel_dist_accum_, visual_rel_dist_accum_, static_cast<int>(active_mode_));
    }

    // 2026-05-28 (Step A) — when affine is valid, speed_rel is ALREADY metric
    // (scale_fuser units). Skip the K multiplication. When affine is invalid,
    // multiply by the global K bootstrap.
    double speed_metric;
    const double K = midas_scale_K_.load(std::memory_order_relaxed);
    if (aff_valid_now) {
        speed_metric = speed_rel;   // already metric per-point via affine
    } else if (K > 0.0) {
        speed_metric = K * speed_rel;
    } else {
        // Neither path available — bail.
        if (frame_counter_ % 30 == 0) {
            LOGI("DEPTH_FLOW_SPEED: K=uncalibrated speed_rel=%.4f/s accel_spd=%.2f "
                 "tsz=%.2f n=%zu (awaiting accel-excitation window after a stop)",
                 speed_rel, accel_speed, secs_since_zupt_, s_est.size());
        }
        return;
    }

    // Trust-boundary sanity: reject a single absurd frame (> 30 m/s = 108 km/h).
    constexpr double kMaxFrameSpeedMps = 30.0;
    if (!std::isfinite(speed_metric) || speed_metric > kMaxFrameSpeedMps) {
        navsight::eventCounters().depth_flow_outlier_rejected.fetch_add(
            1, std::memory_order_relaxed);
        return;
    }

    // EMA low-pass (tau ~ 0.5 s) on the reported metric speed; alpha = dt/(tau+dt).
    constexpr double kTauS = 0.5;
    const double alpha = dt_s / (kTauS + dt_s);
    const double cur = depth_flow_speed_mps_.load(std::memory_order_relaxed);
    const double next = (cur < 0.0) ? speed_metric : (cur + alpha * (speed_metric - cur));
    depth_flow_speed_mps_.store(next, std::memory_order_relaxed);
    navsight::eventCounters().depth_flow_updates.fetch_add(1, std::memory_order_relaxed);
    // 2026-05-28 (Step A) — speed_metric * dt is per-frame metric distance,
    // valid for both affine (already metric) and K (multiplied above) paths.
    navsight::eventCounters().depth_flow_total_mm.fetch_add(
        static_cast<long long>(speed_metric * dt_s * 1000.0 + 0.5),
        std::memory_order_relaxed);

    // Diagnostic (2026-05-26): relative units + calibrated K + accel cross-check, so a
    // walk/run shows whether K converged and the metric speed tracks reality.
    if (frame_counter_ % 10 == 0) {
        std::sort(z_est.begin(), z_est.end());
        std::sort(flow_est.begin(), flow_est.end());
        const size_t m = s_est.size();
        LOGI("DEPTH_FLOW_SPEED: src=%s n=%zu no_depth=%d Z[min/med/max]=%.2f/%.2f/%.2f "
             "flow[med/max]=%.1f/%.1f speed=%.4f/s K=%.3f s=%.4f t=%.4f accel_spd=%.2f "
             "tsz=%.2f gait=%d -> ema=%.2f m/s (%.1f km/h)",
             aff_valid_now ? "affine" : "K",
             m, n_no_depth,
             z_est.front(), z_est[z_est.size() / 2], z_est.back(),
             flow_est[flow_est.size() / 2], flow_est.back(),
             speed_rel, K, s_aff, t_aff, accel_speed, secs_since_zupt_,
             static_cast<int>(active_mode_), next, next * 3.6);
    }
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

// ── Map-matching Step B* (MAP_MATCHING_PLAN.md §8M) — VIO→lat/lng ────────────
// Read-only on the EKF. See Tracker.h VioLla/SessionAnchor for the contract.

void Tracker::setSessionAnchor(double lat_deg, double lng_deg, int64_t t_ns) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (session_anchor_.valid) {
        // Never re-anchor mid-session: a second anchor would teleport the whole
        // track. Log + count, then return (idempotent).
        navsight::eventCounters().vio_lla_anchor_reanchor_ignored.fetch_add(
            1, std::memory_order_relaxed);
        LOGI("VIO_LLA: anchor already set (%.7f,%.7f), ignoring re-anchor (%.7f,%.7f)",
             session_anchor_.anchor_lat_rad * 180.0 / M_PI,
             session_anchor_.anchor_lng_rad * 180.0 / M_PI,
             lat_deg, lng_deg);
        return;
    }
    session_anchor_.anchor_lat_rad = lat_deg * M_PI / 180.0;
    session_anchor_.anchor_lng_rad = lng_deg * M_PI / 180.0;
    session_anchor_.anchor_t_ns    = t_ns;
    session_anchor_.valid          = true;
    navsight::eventCounters().vio_lla_anchor_set.fetch_add(1, std::memory_order_relaxed);
    LOGI("VIO_LLA: SessionAnchor set lat=%.7f lng=%.7f t_ns=%lld",
         lat_deg, lng_deg, static_cast<long long>(t_ns));
}

Tracker::VioLla Tracker::current_vio_lla() const {
    // EARTH_RADIUS_M matches SnappedLatLng.distanceTo (RoadSnapper.kt:238) and
    // Step K-search's forward projection, so the inverse/forward pair round-trips
    // and distance math is consistent across the JNI boundary (no magic-number drift).
    constexpr double EARTH_RADIUS_M = 6371000.0;

    std::lock_guard<std::mutex> lock(pose_mutex_);
    VioLla result;
    result.t_ns = cur_frame_ts_ns_;

    if (!session_anchor_.valid) {
        const long long n = navsight::eventCounters().vio_lla_unanchored_reads.fetch_add(
            1, std::memory_order_relaxed) + 1;
        // Rate-limited falsifier: first read + every 64th thereafter (the read
        // cadence is set by the Kotlin caller; no wall-clock dependency here).
        if (n == 1 || (n % 64) == 0) {
            LOGI("VIO_LLA: no SessionAnchor — matcher disabled (unanchored_reads=%lld)", n);
        }
        result.valid = false;
        return result;
    }

    // Z-up ENU world: global_t_(0)=East, global_t_(1)=North, global_t_(2)=Up
    // (verified Tracker.cpp:3682-3739). The matcher consumes the USER-FACING
    // dot global_t_, NOT ekf_.getPosition()/p_G_ (project_visual_audit_2026_05_30).
    double p_east_m  = 0.0;
    double p_north_m = 0.0;
    if (!global_t_.empty() && global_t_.rows >= 3 && global_t_.type() == CV_64F) {
        p_east_m  = global_t_.at<double>(0);
        p_north_m = global_t_.at<double>(1);
    }

    const double anchor_lat_rad = session_anchor_.anchor_lat_rad;
    // Equirectangular inverse projection (city-scale adequate; exact inverse of
    // Step K-search's forward (lat,lng)->local-metres). p_north -> lat, p_east -> lng.
    result.lat_rad = anchor_lat_rad + p_north_m / EARTH_RADIUS_M;
    result.lng_rad = session_anchor_.anchor_lng_rad +
                     p_east_m / (EARTH_RADIUS_M * std::cos(anchor_lat_rad));
    result.valid = true;

    // var_xy_m2 = trace of the horizontal EKF position-covariance block
    // (East var + North var), 0 before full init. Inlined (cannot call
    // getPositionCovarianceXZ — it would re-lock the non-recursive pose_mutex_).
    if (ekf_.isFullInitialized()) {
        cv::Mat P = ekf_.getCovariance();
        if (!P.empty() && P.rows >= 15 && P.cols >= 15 && P.type() == CV_64F) {
            result.var_xy_m2 = P.at<double>(12, 12) + P.at<double>(13, 13);
        }
    }
    return result;
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

    // 2026-05-29 (Step B) — current frame timestamp, for vi_speed freshness checks
    // in updateDepthFlowSpeed (which doesn't receive timestamp_ns directly).
    cur_frame_ts_ns_ = timestamp_ns;

    // Step 9 (ADR-014): set true when this frame commits a keyframe. Declared
    // at function scope so the assembly site at the end of processFrame can
    // surface it on `out` regardless of where the keyframe-storage branch
    // sat in the per-frame logic.
    bool stored_keyframe_this_frame = false;

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
    const double blur_thresh    = (active_mode_ == GaitMode::VEHICLE) ? BLUR_VAR_THRESH_SCOOTER : BLUR_VAR_THRESH;
    const bool   frame_is_blurry = (blur_var >= 0.0 && blur_var < blur_thresh);
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
            LOGI("BLUR: enter var=%.1f thresh=%.1f", blur_var, blur_thresh);
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

    // ── 3.2 Fix D retry: post-init azimuth queued by setInitialHeading ────
    // Retries the EKF rotation sync when Madgwick was not ready at
    // setInitialHeading call time (see Fix D in setInitialHeading).
    if (pending_post_init_heading_set_ && ekf_.isFullInitialized()) {
        cv::Mat R_GtoI_retry = imu.getRotationGtoI();
        if (!R_GtoI_retry.empty()) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            ekf_.setRotation(R_GtoI_retry);
            global_R_ = R_GtoI_retry;
            scalar_heading_ = pending_post_init_azimuth_;
            pending_post_init_heading_set_ = false;
            LOGI("Fix D retry: post-init azimuth=%.1f deg applied from "
                 "Madgwick (was deferred, now Madgwick ready)",
                 pending_post_init_azimuth_ * 180.0 / M_PI);
        }
        // If still empty: keep pending_post_init_heading_set_=true and
        // retry next frame. Madgwick needs more IMU samples to settle.
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
            // Initialize full MSCKF state if not yet done.
            // 2026-05-16 v27 fix: prior code passed global_R_ which
            // setInitialHeading set as pure Rz(azimuth) (yaw-only,
            // assumes body Z = world Z = phone flat). For a vertical
            // phone (user holds screen-facing-stomach), body Y is up,
            // not body Z — passing pure-yaw seeded EKF R_GtoI 90° off
            // in the roll direction. v27 evidence:
            //   EKF_INIT_TILT: up_body=(0,0,1) tilt_from_vertical=90°
            //   MSCKF first update dtheta=0.43 rad (24.9°)
            //   GPS path=150m, VIO path=115m (24% short)
            // Madgwick already has the full orientation (roll/pitch
            // from gravity correction + yaw from seedMadgwickYaw early-
            // seed). Use it directly. Fall back to global_R_ only if
            // Madgwick isn't initialized yet (shouldn't happen since
            // seedMadgwickYaw fires on first compass reading, well
            // before motion-detected → READY).
            if (!ekf_.isFullInitialized()) {
                // ── Fix C (2026-05-16 audit Finding 2.2) ─────────────────────
                // Cause: falling back to global_R_ (pure-Rz yaw-only) seeds
                // R_GtoI_ 90° wrong in roll for a vertical phone. First MSCKF
                // update must correct ~25° tilt (observed dtheta=0.43 rad in
                // v27 walk); this burns covariance budget the filter never
                // recovers. Madgwick is always ready before motion-detect fires
                // (seedMadgwickYaw runs on the first compass reading, which
                // precedes the motion-detected → READY transition that starts
                // VIO). If it ever isn't ready, defer: the next frame will
                // retry and Madgwick will have had another IMU sample to settle.
                // Change: abort initializeFull and return early; increment the
                // deferred counter for observability. No global_R_ fallback.
                // Falsifier: ekf_init_deferred_madgwick_not_ready stays 0 on
                // every real-device walk (Madgwick is always ready by then).
                // If it ever increments, check IMUPreintegrator::madgwick_init_
                // and the compass-ready sequence.
                auto gb = imu.getGyroBias();
                cv::Mat R_GtoI_seed = imu.getRotationGtoI();
                if (R_GtoI_seed.empty()) {
                    navsight::eventCounters()
                        .ekf_init_deferred_madgwick_not_ready
                        .fetch_add(1, std::memory_order_relaxed);
                    LOGI("EKF init: Madgwick not ready — deferring initializeFull "
                         "to next frame (counter=%lld)",
                         navsight::eventCounters()
                             .ekf_init_deferred_madgwick_not_ready
                             .load(std::memory_order_relaxed));
                    // Reset initialized_ so next frame re-enters the first-frame
                    // branch and retries EKF init.
                    initialized_ = false;
                    /* 2026-05-16 audit Fix C — global_R_ fallback removed.
                     * Old code:
                     *   R_GtoI_seed = global_R_;
                     *   ekf_.initializeFull(R_GtoI_seed, gb, cv::Point3f(0,0,0));
                     * Seeded with pure-Rz → 90° roll error for vertical phone.
                     */
                    return out;
                }
                LOGI("EKF init: R_GtoI seeded from Madgwick quaternion "
                     "(full roll/pitch/yaw)");
                ekf_.initializeFull(R_GtoI_seed, gb, cv::Point3f(0,0,0));
            }
            feature_mgr_.storeKeyframe(gray_buf_, prev_pts_, timestamp_ns, 0,
                                      scalar_heading_,
                                      cv::Point3f(0, 0, 0));
            // Plan Step 4 (ADR-010): mirror the keyframe into the ORB
            // descriptor ring buffer for relocalization.
            feature_mgr_.storeKeyframeDescriptors(
                static_cast<uint64_t>(frame_counter_),
                static_cast<double>(timestamp_ns),
                gray_buf_, prev_pts_, feature_ids_,
                /*lens=*/&lens_);
            LOGI("processFrame: first frame, grid-detected %zu features", prev_pts_.size());
            return out;
        }
        current_prev_gray = prev_gray_;
        current_prev_pts_buf_ = prev_pts_;
        current_prev_ts = prev_timestamp_ns_;
    }

    frame_counter_++;

    // Phase 1 Step 2c verification: log measured camera frame rate so we
    // can confirm the [30, 30] AE_TARGET_FPS_RANGE Camera2Interop lock
    // (CameraUi.kt) is actually in effect. Without this, we have no
    // way to know if the AE algorithm honored the request — different
    // devices/lighting can silently fall back to [15, 30] or [10, 60].
    // Emit one CAM_FPS line per ~kCamFpsWindow frames.
    if (prev_camera_frame_ts_ns_ != 0) {
        const double dt_ms = (timestamp_ns - prev_camera_frame_ts_ns_) * 1e-6;
        // Sanity-bound dt to reject impossible spikes (e.g. first frame
        // after a long pause); a > 1 s gap means the camera was stopped,
        // not "we measured an actual 1 Hz frame rate".
        if (dt_ms > 0.0 && dt_ms < 1000.0) {
            cam_dt_sum_ms_ += dt_ms;
            ++cam_dt_count_;
        }
        if (cam_dt_count_ >= kCamFpsWindow) {
            const double avg_dt_ms = cam_dt_sum_ms_ /
                                     static_cast<double>(cam_dt_count_);
            const double fps = (avg_dt_ms > 0.0) ? (1000.0 / avg_dt_ms) : 0.0;
            LOGI("CAM_FPS: avg_dt=%.2fms fps=%.2f (window=%d frames)",
                 avg_dt_ms, fps, cam_dt_count_);

            // Phase 1 Step 2c verification: persist mean+stdev to
            // event_summary so we can verify the [30, 30] AE_TARGET_FPS
            // lock from the sim JSON without needing logcat retention.
            // Welford's online algorithm — numerically stable, single pass.
            ++cam_fps_running_count_;
            const double delta = fps - cam_fps_running_mean_hz_;
            cam_fps_running_mean_hz_ += delta /
                                        static_cast<double>(cam_fps_running_count_);
            const double delta2 = fps - cam_fps_running_mean_hz_;
            cam_fps_running_m2_ += delta * delta2;

            const double variance =
                (cam_fps_running_count_ > 1)
                ? cam_fps_running_m2_ /
                  static_cast<double>(cam_fps_running_count_ - 1)
                : 0.0;
            const double stdev = std::sqrt(std::max(0.0, variance));

            // Push to EventCounters as milli-Hz (atomic<long long>;
            // C++17 lacks atomic<double>). Truncate, not round, to keep
            // the read trivially convertible: hz = milli_hz / 1000.0.
            navsight::eventCounters().cam_fps_mean_milli_hz.store(
                static_cast<long long>(cam_fps_running_mean_hz_ * 1000.0),
                std::memory_order_relaxed);
            navsight::eventCounters().cam_fps_stdev_milli_hz.store(
                static_cast<long long>(stdev * 1000.0),
                std::memory_order_relaxed);
            navsight::eventCounters().cam_fps_window_count.store(
                cam_fps_running_count_,
                std::memory_order_relaxed);

            cam_dt_sum_ms_ = 0.0;
            cam_dt_count_  = 0;
        }
    }
    prev_camera_frame_ts_ns_ = timestamp_ns;

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
    
    // 2026-05-09 v18 — single-source-of-truth sync.
    //
    // The EKF used to maintain its own internal `p_G_` updated by
    // `propagateIMU` (pure-IMU integration), which DRIFTED unboundedly
    // because pedestrian/scooter motion is bias-quadratic in IMU-only
    // integration. Meanwhile the user-facing trajectory `global_t_` was
    // computed separately from visual VO + heading + scaled-step and
    // stayed accurate. The disconnect meant: clones, MSCKF residuals,
    // loop-closure χ² gates all evaluated against the drifted p_G even
    // though the user saw a reasonable global_t_ — chi² rejected real
    // loop closures because `target_p − p_G` had 200+ m residuals while
    // `target_p − global_t_` was within metres.
    //
    // Architecture per Morad's v17→v18 directive: ONE trajectory.
    // global_t_ IS the position. Sync EKF state to it before the IMU
    // propagation runs, so propagateIMU integrates from the visually-
    // grounded base. Any small drift `propagateIMU` adds within a frame
    // is corrected by the next frame's sync (after global_t_ updates
    // again from visual VO). Camera leads, IMU assists.
    //
    // This eliminates the "two trajectories" problem: there is now no
    // p_G under the hood that diverges from what the UI shows.
    if (ekf_.isFullInitialized()) {
        // 2026-05-16 audit Finding 1 fix: EKF owns p_G_ post-init. Per-frame
        // setPosition collapsed P_pp via repeated Joseph form (K → 0.02%).
        // Direction reversed: EKF propagates autonomously; global_t_ is updated
        // AFTER all MSCKF/SLAM/LC updates have run (see mirror below output §12).
        // Old code:
        //   ekf_.setPosition(global_t_);   // ← collapsed P_pp every frame
        // Cause: hard-overwriting p_G_ before propagateIMU reset covariance P_pp
        //   toward zero via successive Joseph-form updates. With P_pp ≈ 0.0009 m²
        //   and PnP residual |r_p|=3-5m, chi² saturated near threshold and Kalman
        //   gain K = P_pp/(P_pp + var_p) ≈ 0.02% — corrections negligible.
        // Change: removed. EKF p_G_ propagates from initializeFull seed onward;
        //   all visual measurements (updateRelativePose, updateAbsolutePose) apply
        //   correctly. global_t_ is now mirrored FROM EKF after updates each frame.
        // Falsifier: log P_.at<double>(12,12) before updateAbsolutePose during a
        //   walk. Post-fix it should grow to O(drift²) rather than collapsing to
        //   ~0.0009 m². Loop-closure Kalman gain should be >> 1% on first LC.
        //
        // 2026-05-16 Tier 1 revert (agent A9+A10 confirmed): the audit-Finding-1
        // "fix" REMOVED setPosition and made EKF p_G autonomous. Result: pure
        // IMU integration drifts quadratically with bias. v31: 75 m drift on
        // 10 m walked, velocity_clamped = 177. Pre-fix v22 walked 109 m at 1.1 %.
        // Restored: setPosition(global_t_) before propagateIMU. Proper long-term
        // fix is the audit's *alternative* (P_pp drift-rate floor) — separate work.
        // 2026-05-31 — REPLAY-ONLY experiment gate (docs/MSCKF_PG_WIRING_VERDICT_
        // 2026_05_31.md §4). Default (autonomous_pg_==false) → identical to the
        // restored Tier-1 behaviour above: setPosition runs every frame and the
        // device build is byte-for-byte unchanged. When the harness --autonomous-pg
        // flag is set, this overwrite is SKIPPED so p_G_ propagates autonomously and
        // the MSCKF/ZUPT updates are no longer wiped each frame — the A/B that
        // measures whether those updates BOUND p_G_ or it drifts like v31. Line 1863
        // propagateIMU is INTENTIONALLY left intact in both arms.
        if (!autonomous_pg_) ekf_.setPosition(global_t_);
        ekf_.propagateIMU(imu_delta.deltaR, imu_delta.deltaV, imu_delta.deltaP,
                          imu_delta.dt, imu_delta.cov,
                          imu_delta.J_R_bg, imu_delta.J_V_bg, imu_delta.J_V_ba,
                          imu_delta.J_P_bg, imu_delta.J_P_ba);

        // 2026-05-31 — REPLAY-ONLY read-only diagnostics for the autonomous-p_G_
        // A/B (verdict §4). Gated on autonomous_pg_ so the device build emits
        // nothing. Throttled ~every 30 frames. Three signals make the A/B
        // measurable from logcat even if the CSV is post-processed:
        //   * |p_G_| net displacement from the world origin (the drift magnitude),
        //   * running max |p_G_ - global_t_| (how far the EKF diverges from the dot),
        //   * |v_G_| (confirms ZUPT re-zeros velocity at stops — v31 failed here).
        // Purely observational: no state is mutated.
        if (autonomous_pg_) {
            const cv::Mat p_ekf = ekf_.getPosition();
            const cv::Mat v_ekf = ekf_.getVelocity();
            if (!p_ekf.empty() && p_ekf.rows >= 3 && p_ekf.type() == CV_64F &&
                !global_t_.empty() && global_t_.rows >= 3 &&
                global_t_.type() == CV_64F) {
                const double pg_norm = cv::norm(p_ekf);
                const double div = cv::norm(p_ekf - global_t_);
                if (div > autonomous_pg_max_div_m_) autonomous_pg_max_div_m_ = div;
                const double vg_norm =
                    (!v_ekf.empty() && v_ekf.rows >= 3 && v_ekf.type() == CV_64F)
                        ? cv::norm(v_ekf) : std::nan("");
                if ((frame_counter_ % 30) == 0) {
                    LOGI("AUTONOMOUS_PG_DIAG: pG_disp_m=%.3f max_div_m=%.3f "
                         "vG_mps=%.3f", pg_norm, autonomous_pg_max_div_m_, vg_norm);
                }
            }
        }

        // 2026-05-16 — EKF→IMU gyro-bias feedback RE-ADDED at keyframe cadence.
        //
        // Cause: with the feedback loop removed (after my failed Option B made
        // things worse), the IMUPreintegrator's gyro_bias_ never gets refreshed
        // with the EKF's converged b_g estimate. Each frame the EKF accumulates
        // residual bias error in R_GtoI; clone storage uses that drifted pose;
        // MSCKF chi² then rejects 89% of features in v32 because predicted vs
        // observed pixels disagree by 25-40× the gate threshold (Agent A
        // finding). User-visible: SLAM dots only appear when stationary because
        // promotions need RMS < 1.5 px and motion produces 5+ px reprojection
        // errors against the drifted clones.
        //
        // Change: push ekf_.getGyroBias() into imu.setGyroBias once per ~6
        // camera frames (≈ 5 Hz at 30 Hz capture). This matches v22's keyframe-
        // cadence pattern, which walked 109 m at 1.1 % drift. The per-frame
        // push (swarm pattern, 2026-05-12) oscillates and broke v25-v31; zero
        // feedback (post-Option-B) lets bias estimates drift apart.
        //
        // Falsifier: post-fix walk should show
        //   msckf_chi2_rejected / msckf_update_lines < 0.20
        //   gyro_bias_pushed_total ≈ 5 / sec
        //   Visible orange dots accumulating during walking, not only stationary.
        // If chi² rejection stays > 50 %, the bias-feedback hypothesis is wrong
        // and we look at clone-position pipeline or post-Tier-1 Phi block.
        if ((frame_counter_ % 6) == 0) {
            cv::Mat ekf_bg = ekf_.getGyroBias();
            if (!ekf_bg.empty() && ekf_bg.rows == 3 && ekf_bg.type() == CV_64F) {
                // 2026-05-21 BUG-02 ROOT-CAUSE FIX — ADD-and-ZERO per
                // EKFState.h:701-731.
                //
                // Cause: this block previously called
                //   imu.setGyroBias(ekf_bg.x, ekf_bg.y, ekf_bg.z)
                // which REPLACED the IMU's calibrated gyro_bias_ (~0.18 rad/s
                // on a vertically-held S21 Ultra) with the EKF's RESIDUAL b_g_
                // (~0.007 rad/s after MSCKF convergence). Net effect: ~0.17
                // rad/s of un-subtracted gyro entered preintegration → phantom
                // yaw drift of ~0.4°/s = +46° per 120s walk, matching the user's
                // observed 30-60° displayed-heading drift per loop. Counter
                // evidence: `ekf_bg_absorbed_total = 0` across all 6 walks
                // 2026-05-19 to 2026-05-21 despite `gyro_bias_pushed_total
                // = 391` — the absorb path was never engaged.
                //
                // Correct pattern (this fix): READ ekf b_g_, ADD it to the
                // IMU's gyro_bias_ (absorbing the EKF residual into the IMU's
                // full bias estimate), then call ekf_.setGyroBias(0,0,0) to
                // zero the residual. Covariance P_ is intentionally NOT
                // touched — uncertainty about the bias hasn't changed; we
                // just transferred where the estimate is held. The next MSCKF
                // update accumulates fresh residual on top of the now-correct
                // full bias.
                //
                // Falsifier: post-fix walk →
                //   ekf_bg_absorbed_total > 0 (this code path now firing)
                //   debug-panel heading drift per 2 loops < 5° (was 30-60°)
                //   Madgwick yaw-rate p99 during stationary < 0.5°/s (was
                //                                              ~2.4°/s)
                //
                // Hard constraint: caller (this frame's processFrame) holds
                // Tracker::pose_mutex_ — same convention as updateZRUP /
                // applyMSCKFUpdate. setGyroBias on EKFState is documented as
                // requiring this lock.
                cv::Point3f imu_bg = imu.getGyroBias();
                const float new_bg_x = imu_bg.x +
                    static_cast<float>(ekf_bg.at<double>(0, 0));
                const float new_bg_y = imu_bg.y +
                    static_cast<float>(ekf_bg.at<double>(1, 0));
                const float new_bg_z = imu_bg.z +
                    static_cast<float>(ekf_bg.at<double>(2, 0));
                imu.setGyroBias(new_bg_x, new_bg_y, new_bg_z);
                ekf_.setGyroBias(0.0, 0.0, 0.0);
                navsight::eventCounters().gyro_bias_pushed_total.fetch_add(
                    1, std::memory_order_relaxed);
                navsight::eventCounters().ekf_bg_absorbed_total.fetch_add(
                    1, std::memory_order_relaxed);
                if ((frame_counter_ % 60) == 0) {
                    LOGI("IMU_BG_ABSORB: was_imu_bg=(%+.5f,%+.5f,%+.5f) "
                         "ekf_residual=(%+.5f,%+.5f,%+.5f) "
                         "new_imu_bg=(%+.5f,%+.5f,%+.5f) rad/s "
                         "(frame=%d, ADD-and-ZERO)",
                         imu_bg.x, imu_bg.y, imu_bg.z,
                         ekf_bg.at<double>(0, 0),
                         ekf_bg.at<double>(1, 0),
                         ekf_bg.at<double>(2, 0),
                         new_bg_x, new_bg_y, new_bg_z,
                         frame_counter_);
                }
            }
        }

        // 2026-05-16 — EKF→IMU gyro-bias feedback loop REMOVED entirely.
        //
        // History:
        //   - Pre-2026-05-12 (v22 era): no feedback loop. IMU calibrated its
        //     bias once at startup and held it. EKF tracked b_g_ via the
        //     Phi linearization (J_R_bg Jacobian). Walks were ~1-2 % drift.
        //   - 2026-05-12 architecture-review swarm: added
        //       imu.setGyroBias(ekf_.getGyroBias())  // OVERWRITE every frame
        //     under the comment "Forster-style preintegration assumes this
        //     feedback loop exists". v25-v27 walks degraded to ~7 m drift on
        //     100 m; v28 with Phase 6.4 EKF coupling collapsed to 417 m.
        //   - 2026-05-16 Option B (additive merge): accumulated EKF residual
        //     into IMU's bias every frame, then reset EKF b_g. v31 walk
        //     showed 75 m drift on 10 m (WORSE than original swarm code).
        //     Root cause: every frame the accumulator added the EKF's
        //     small residual (~0.02 rad/s) → IMU bias grew unbounded at
        //     ~0.02 × frame_rate rad/s per second.
        //
        // Decision: REMOVE the loop entirely, restore v22-era behavior.
        //
        //   * IMU's gyro_bias_ is calibrated once at startup and held.
        //   * EKF's b_g_ tracks the residual independently via MSCKF/SLAM.
        //   * The Phi J_R_bg Jacobian accounts for the residual in the
        //     state-transition linearization — no feedback needed.
        //
        // The swarm comment ("Forster-style preintegration assumes this
        // feedback loop exists") was incorrect. Forster preintegration
        // works with a static (per-segment) bias estimate AND a J_R_bg
        // Jacobian that lets MSCKF correct the state without needing the
        // IMU side to be re-calibrated mid-walk.
        //
        // Falsifier: post-revert walk should show |p|_final tracking GPS
        // within ±10 m on a 30 m walk, position direction reversing after
        // 180° turns, and velocity_clamped near 0.
        /* SUPERSEDED 2026-05-16 (both v22→v31 attempts):
        cv::Mat ekf_bg = ekf_.getGyroBias();
        if (!ekf_bg.empty() && ekf_bg.rows == 3 && ekf_bg.type() == CV_64F) {
            imu.setGyroBias(
                static_cast<float>(ekf_bg.at<double>(0, 0)),
                static_cast<float>(ekf_bg.at<double>(1, 0)),
                static_cast<float>(ekf_bg.at<double>(2, 0)));
        }
        */

        // ── Stage 1: EKF gravity-alignment measurement update ─────────
        //
        // The principled fix to R_GtoI drift: feed the accelerometer
        // back to the EKF as a 2-DOF roll/pitch observation (yaw
        // unobservable from gravity). Same structure as Madgwick, but
        // applied as a Kalman measurement update with covariance —
        // not a state override. This bounds R_GtoI within physics
        // limits set by σ_accel, instead of trusting Madgwick blindly.
        //
        // 2026-05-09 fix #2 — instantaneous accel is too noisy:
        // walking-induced body sway tilts the per-frame accel vector by
        // 5-10° per heel-strike. With var_unit ≈ (5.7°)² the EKF's
        // R_GtoI tracked each tilt, producing visible heading wobble.
        // Two mitigations now in effect:
        //   (1) Use imu.getFilteredGravity() — the LP-filtered accel
        //       vector with α=0.02 effective time constant ~1.6 s
        //       (IMUPreintegrator.h). Smooths out walking transients
        //       at the source, same buffer Madgwick uses for its own
        //       gravity correction.
        //   (2) Gate on |gyro| being slow. During fast turns the body
        //       is genuinely rotating; accel-as-gravity assumption
        //       breaks. Threshold derived from walking-gait baseline:
        //       typical walking gyro RMS is ≈ 0.4 rad/s on body axes;
        //       3σ ceiling ≈ 1.2 rad/s. Above that, gravity update is
        //       suspended until motion calms.
        constexpr double kAccelNoiseSigma   = 0.1;       // m/s² (white noise floor)
        constexpr double kGravityMag        = 9.81;      // m/s²
        constexpr double kGyroGateRadS      = 1.2;       // 3σ above walking-gait RMS
        // 2026-05-09 fix #3 — Tracker outer gate must match the EKFState
        // inner gate (g ± 0.8 m/s², derived as 3σ_acc + walking-band in
        // EKFState::updateGravityAlignment). The previous outer gate was
        // [0.5g, 1.5g] = [4.9, 14.7] m/s² — 10× wider than the
        // authoritative inner gate, so it never rejected anything the
        // inner gate would accept. Looking like a guard but providing
        // none invites future maintainers to "tune" it without realising
        // the real filter is elsewhere.
        constexpr double kAccelMagBand      = 0.8;       // m/s² (matches EKFState inner gate)
        cv::Point3f g_filt = imu.getFilteredGravity();
        const double ax = g_filt.x;
        const double ay = g_filt.y;
        const double az = g_filt.z;
        const double a_norm = std::sqrt(ax*ax + ay*ay + az*az);
        const float gx = imu.lastGyroX();
        const float gy = imu.lastGyroY();
        const float gz = imu.lastGyroZ();
        const double gyro_norm = std::sqrt(static_cast<double>(gx)*gx +
                                           static_cast<double>(gy)*gy +
                                           static_cast<double>(gz)*gz);
        const bool accel_ok = (std::abs(a_norm - kGravityMag) < kAccelMagBand);
        const bool gyro_ok  = (gyro_norm < kGyroGateRadS);
        if (accel_ok && gyro_ok) {
            // Variance scales with |residual| — EKF down-weights samples
            // taken under residual linear accel even after LP smoothing.
            // Floor at sensor white-noise σ.
            const double residual_g = std::abs(a_norm - kGravityMag);
            const double sigma_acc  = std::max(kAccelNoiseSigma,
                                               residual_g);
            const double var_acc    = sigma_acc * sigma_acc;

            // 2026-05-09 v17 — LP-filter rotation lag inflation.
            //
            // imu.getFilteredGravity() is an exponential-moving-average
            // with α = 0.02 (IMUPreintegrator.cpp:131). At Android's
            // 200 Hz IMU rate (Δt = 5 ms), the 1/e time constant is
            //   τ_filter = Δt / α = 0.005 / 0.02 = 0.25 s.
            // Under steady body rotation rate ω, the filtered gravity
            // vector lags the true body-frame gravity direction by
            //   θ_lag ≈ τ_filter · ω   (rad)   for ω·τ << 1.
            // Without this inflation, LC_GA fires during rotation with
            // z_obs reflecting the OLD gravity direction (before the
            // rotation started), tries to "correct" R_GtoI toward the
            // stale observation, and corrupts R_GtoI in the lag direction.
            // v16 logcat showed r_norm pinned at 1.50 during left-right
            // rotation (= ~90° error), exactly the failure mode.
            //
            // The unit-vector residual variance contribution from
            // rotation lag is θ_lag² = (τ_filter · ω)². Adding it to
            // the sensor noise variance gives a Kalman gain that
            // naturally collapses to zero during fast rotation —
            // gravity-alignment becomes a no-op when its observation is
            // unreliable, exactly as it should. Conversely, when the
            // body slows down (ω → 0), the lag term vanishes and the
            // update fires at full strength.
            //
            // var_total (m²/s⁴, what updateGravityAlignment expects) =
            //   var_acc  +  (τ_filter · ω · g)²
            //   ── sensor   ── rotation-lag contribution
            constexpr double kGravityFilterTau_s = 0.25;   // 1/e settling at 200 Hz, α=0.02
            const double rot_lag_rad   = kGravityFilterTau_s * gyro_norm;
            const double var_rot_lag   = (rot_lag_rad * kGravityMag) * (rot_lag_rad * kGravityMag);
            const double var_total     = var_acc + var_rot_lag;
            cv::Mat accel_body = (cv::Mat_<double>(3, 1) << ax, ay, az);
            if (!ekf_.updateGravityAlignment(accel_body, var_total)) {
                navsight::eventCounters().gravity_alignment_rejected.fetch_add(
                    1, std::memory_order_relaxed);
                LOGI("updateGravityAlignment rejected (EKF not init / accel band / factorisation)");
            }
        }
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
    const double rot_thresh = (active_mode_ == GaitMode::VEHICLE) ? GYRO_ROT_ONLY_THRESH_SCOOTER : GYRO_ROT_ONLY_THRESH;
    bool gyro_pure_rotation_candidate = (gyro_norm > rot_thresh);
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

    // ── B-PROBE (2026-06-03, LOG-ONLY, never feeds anything) ─────────────────
    // Is the fast near-ground flow the production KLT DROPS actually recoverable, or motion-blurred away?
    // The production window is capped at 41 px (rotation-sized), so road points flowing 30-80 px/frame at
    // scooter speed get dropped → the IPM saturates (corr(GPS,IPM)≈0). Re-track the SAME points with a BIG
    // window + deep pyramid and report: how many the big window keeps that production dropped ("recovered"),
    // and their flow magnitude. High recovered flow ⇒ Fix B = enlarge the window; few/low ⇒ blurred ⇒ pivot.
    // Every 15th frame to bound the cost; TEMP probe, remove once B is decided.
    if (frame_counter_ % 15 == 0 && current_prev_pts_buf_.size() >= 8 &&
        !current_prev_gray.empty() && !gray_buf_.empty() &&
        next_pts_buf_.size() == current_prev_pts_buf_.size()) {
        std::vector<cv::Point2f> bn; std::vector<uchar> bs; std::vector<float> be;
        cv::calcOpticalFlowPyrLK(current_prev_gray, gray_buf_, current_prev_pts_buf_, bn, bs, be,
                                 cv::Size(61, 61), 5,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        int prod_surv = 0, big_surv = 0, recovered = 0;
        std::vector<double> rec_flow, prod_flow;
        for (size_t i = 0; i < current_prev_pts_buf_.size(); ++i) {
            const bool ps  = (i < status.size() && status[i]);
            const bool bsv = (i < bs.size() && bs[i]);
            if (ps)  { ++prod_surv; prod_flow.push_back(cv::norm(next_pts_buf_[i] - current_prev_pts_buf_[i])); }
            if (bsv) { ++big_surv; if (!ps) { ++recovered; rec_flow.push_back(cv::norm(bn[i] - current_prev_pts_buf_[i])); } }
        }
        auto medd = [](std::vector<double>& v) { if (v.empty()) return -1.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
        LOGI("BPROBE: n=%zu prod_surv=%d prod_flow_px=%.1f | bigwin_surv=%d recovered=%d rec_flow_px=%.1f",
             current_prev_pts_buf_.size(), prod_surv, medd(prod_flow), big_surv, recovered, medd(rec_flow));
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
    // 2026-06-02 overlay — per-point recoverPose inlier flag, parallel to tracked_pts_flat. Filled
    // from status_verification after geometricVerification below; stays all-0 ("unverified") on frames
    // where verification was NOT attempted (blurry / static / low-parallax) so the overlay can show the
    // points the VIO actually USED (bright) vs tracked-but-rejected/unused (dim) — the owner's
    // "the KLT points I see now are pre-calculations".
    std::vector<unsigned char> tracked_pts_inlier_flags(next_good_buf_.size(), 0);

    // Phase 2 camera overlay (camera_overlay_phase23_plan.md, Task C):
    // populate per-feature age (frames survived) parallel to
    // tracked_pts_flat. feature_ages_ was just rebuilt above to match
    // next_good_buf_ exactly (line ~999), so size() is guaranteed to
    // equal next_good_buf_.size() here. Stored on VisionOutput so the
    // overlay can color-code dots without an extra JNI round trip.
    std::vector<int> tracked_pts_ages;
    tracked_pts_ages.reserve(next_good_buf_.size());
    for (size_t i = 0; i < next_good_buf_.size(); ++i) {
        const int age = (i < feature_ages_.size()) ? feature_ages_[i] : 0;
        tracked_pts_ages.push_back(age);
    }

    // 2026-05-19 Fix #6 — per-frame refresh of SLAM observation pixel.
    // (See SlamFeatureOverlay Fix #6 in CameraUi.kt for the full writeup.)
    //
    // 2026-05-19 Fix #7 — per-frame SLAM update against LIVE IMU state,
    //               so the EKF (α, β, ρ) state tracks the live observation
    //               at frame rate instead of waiting for the next keyframe.
    //
    // 2026-05-19 Fix #8 — batched applySlamLiveBatch.
    //
    // Cause for batch: Fix #7's per-feature updateSlamFeatureLive loop
    // ANR'd at 30 Hz × 12 SLAM features × O(n^3) Joseph form ≈ 1080 ms/sec
    // of camera-thread work. Fix #7b throttle (1/3 frame) papered over it
    // but was a frequency knob, not an architectural fix.
    // Batch stacks all per-feature Jacobian rows into ONE (2K × dim) H
    // matrix and runs a single Joseph-form update. Mathematically
    // equivalent to N sequential updates (Kalman is linear). Cost drops
    // from N × O(n^3) to 1 × O(n^3) — 12× speedup at K=12 features.
    //
    // Falsifier: post-Fix-#8 the throttle is removed (every frame batches)
    // AND no ANR fires across a multi-minute walk. Counter
    // `slam_live_batch_calls` ≈ frames-with-SLAM-features; ratio
    // `slam_live_updates_fired / slam_live_batch_calls` shows per-call
    // yield (should be ≥ 1 row applied on average when features are
    // tracked and EKF state is healthy).

    // 1. Overlay refresh — runs every frame regardless of EKF state so
    //    the user sees orange dots stay anchored to KLT pixels even when
    //    the EKF can't apply an update (blurry, static, not initialized).
    //    Fix #6: overlay obs is RAW (distorted) pixel for camera-preview
    //    alignment; the linear-K EKF projection uses the undistorted form.
    //
    // SLAM observation refresh loop (Fix #6).
    for (size_t i = 0;
         i < next_good_buf_.size() && i < feature_ids_.size(); ++i) {
        const int fid = feature_ids_[i];
        if (fid < 0) continue;
        const auto* lc = feature_mgr_.getLifecycle(fid);
        if (lc && lc->slam_slot >= 0) {
            feature_mgr_.noteSlamLastObs(
                fid, next_good_buf_[i].x, next_good_buf_[i].y);
        }
    }

    // 2026-05-19 Fix #11b — Per-frame proximity-based landmark pixel
    // refresh. Replaces the v11a feature_id-linkage approach which only
    // landed 110 refreshes over 65k observed-dot render events because
    // most ORB rows have feature_ids[t_idx] = -1 (no KLT-spatial-proximity
    // link at descriptor-storage time per KeyframeDescriptors.h:11).
    //
    // Strategy: for each landmark in last_observed_landmark_pixels_, find
    // the nearest CURRENT-FRAME KLT feature within `kRefreshRadiusPx`. If
    // found, snap the landmark's observed pixel to that KLT position. The
    // dot tracks the feature at full KLT rate (30 Hz) instead of being
    // stuck at the last keyframe's pixel for 28-29 frames.
    //
    // Radius 5 px derived from: KLT inter-frame motion under walking
    // (mean_flow typically 2-5 px at 30 Hz, S21 Ultra 640×480 analyzer
    // frame) + 1 σ ORB localisation accuracy (~1 px). Larger radius
    // risks landmark-grabbing-wrong-feature. Smaller risks the KLT
    // feature drifting out of radius between keyframes (rebound at next
    // keyframe). 5 px is the floor below which inter-frame KLT motion
    // exceeds the radius.
    //
    // Falsifier: post-Fix-#11b walk should show
    //   landmarks_pixel_refreshed_total ≈ landmarks_rendered_anchor_total
    // (each observed-dot render gets a refresh). User-visible: orange
    // dots track features as the camera moves instead of being stuck.
    constexpr float kRefreshRadiusPx   = 5.0f;
    constexpr float kRefreshRadiusSqPx = kRefreshRadiusPx * kRefreshRadiusPx;
    std::vector<std::pair<size_t, cv::Point2f>> landmark_pixel_updates;
    {
        std::lock_guard<std::mutex> lk(last_observed_mutex_);
        landmark_pixel_updates.reserve(last_observed_landmark_pixels_.size());
        for (size_t k = 0; k < last_observed_landmark_pixels_.size(); ++k) {
            const float u_old = last_observed_landmark_pixels_[k].x;
            const float v_old = last_observed_landmark_pixels_[k].y;
            // Find nearest KLT feature within radius.
            float best_d2  = kRefreshRadiusSqPx;
            size_t best_i  = SIZE_MAX;
            for (size_t i = 0; i < next_good_buf_.size(); ++i) {
                const float du = next_good_buf_[i].x - u_old;
                const float dv = next_good_buf_[i].y - v_old;
                const float d2 = du * du + dv * dv;
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_i  = i;
                }
            }
            if (best_i != SIZE_MAX) {
                landmark_pixel_updates.emplace_back(k, next_good_buf_[best_i]);
            }
        }
        // Apply updates under the same lock (avoids a second take).
        for (const auto& [idx, pix] : landmark_pixel_updates) {
            last_observed_landmark_pixels_[idx] = pix;
        }
    }
    if (!landmark_pixel_updates.empty()) {
        navsight::eventCounters().landmarks_pixel_refreshed_total.fetch_add(
            static_cast<long long>(landmark_pixel_updates.size()),
            std::memory_order_relaxed);
    }

    // 2. Batched live SLAM Kalman update — only when EKF is healthy.
    //    is_static guard mirrors the keyframe SLAM update gate at line
    //    ~3141 (during stationary, KLT subpixel noise was injecting δp
    //    drift into p_G via SLAM residuals).
    if (ekf_.isFullInitialized() && !frame_blurry_ && !is_static) {
        // Bulk-undistort the current frame's KLT pixels once so the EKF
        // sees linear-K coords. Skip when lens not calibrated.
        std::vector<cv::Point2f> klt_ud;
        klt_ud.reserve(next_good_buf_.size());
        for (const auto& p : next_good_buf_) klt_ud.push_back(p);
        if (lens_.isReady() && lens_.hasDistortion()) {
            lens_.undistortPoints(klt_ud);
        }

        // Collect (slot, undistorted-pixel) for every KLT-tracked SLAM
        // feature this frame. The batch helper handles per-row chi² +
        // depth + early-out gates internally; rows that fail are not
        // stacked.
        std::vector<std::pair<int, cv::Point2f>> batch_obs;
        batch_obs.reserve(next_good_buf_.size());
        for (size_t i = 0;
             i < next_good_buf_.size() && i < feature_ids_.size(); ++i) {
            const int fid = feature_ids_[i];
            if (fid < 0) continue;
            const auto* lc = feature_mgr_.getLifecycle(fid);
            if (!lc || lc->slam_slot < 0) continue;
            const cv::Point2f& obs_ud =
                (i < klt_ud.size()) ? klt_ud[i] : next_good_buf_[i];
            batch_obs.emplace_back(lc->slam_slot, obs_ud);
        }

        if (!batch_obs.empty()) {
            const int n_applied = ekf_.applySlamLiveBatch(batch_obs, /*sigma_px=*/1.0);
            const int n_skipped = static_cast<int>(batch_obs.size()) - n_applied;
            navsight::eventCounters().slam_live_batch_calls.fetch_add(
                1, std::memory_order_relaxed);
            if (n_applied > 0) {
                navsight::eventCounters().slam_live_updates_fired.fetch_add(
                    n_applied, std::memory_order_relaxed);
            }
            if (n_skipped > 0) {
                navsight::eventCounters().slam_live_updates_skipped.fetch_add(
                    n_skipped, std::memory_order_relaxed);
            }
        }
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

    // ── 2026-06-02: Gait-aware ZUPT thresholds (scooter vibration fix) ───────
    UpdaterZeroVelocity::Options zupt_opts;
    if (active_mode_ == GaitMode::VEHICLE) {
        // Scooter: allow more IMU variance (engine vibration) but tighten
        // the visual gate (prevent false static while cruising).
        // 0.075 allows ~0.15 rad/s RMS (matching ZUPT_GYRO_THRESH_SCOOTER).
        zupt_opts.sigma_g = 0.075;
        zupt_opts.sigma_a = 0.50;
        zupt_opts.max_disparity = 1.0;
    }
    zupt_detector_.setOptions(zupt_opts);

    // ZUPT: Statistical stationary detection (OpenVINS style)
    is_static = zupt_detector_.is_stationary(imu.getAccelBuffer(), imu.getGyroBuffer(), mean_flow);

    // Safety overrides (relaxed: KLT has ~0.5-1.5px noise even stationary)
    // Scooter (VEHICLE): lower the flow gate to 1.5px (from 2.5px) to break
    // ZUPT more aggressively during smooth cruising.
    double flow_gate = (active_mode_ == GaitMode::VEHICLE) ? 1.5 : 3.5;
    if (mean_flow > flow_gate) is_static = false;

    // Don't trust step speed to break ZUPT while rotating fast — phantom steps
    // during in-place rotation used to un-freeze translation and produce arcs.
    if (is_static && gyro_norm < 0.8 && imu.getStepInfo().speed_mps > 0.45) is_static = false;


    // 2026-05-26 — accel world-velocity for MiDaS scale calibration (consumed in
    // updateDepthFlowSpeed). Integrate the SAME per-frame world velocity increment
    // the EKF uses (g*dt + R_GtoI^T*deltaV, EKFState.cpp:295) from a ZUPT stop, and
    // re-zero it at the next stop. Trustworthy ONLY in the short post-stop window
    // (raw accel integration drifts after ~2-3 s — proven offline); there it yields
    // a clean metric speed that calibrates the relative->metric scale K.
    if (ekf_.isFullInitialized() && !imu_delta.deltaV.empty() &&
        imu_delta.dt > 0.0 && imu_delta.dt < 0.5) {
        cv::Mat R_gi = ekf_.getRotation();   // R_GtoI (world->body)
        if (!R_gi.empty() && R_gi.rows == 3 && R_gi.cols == 3) {
            cv::Mat dvw = R_gi.t() * imu_delta.deltaV;   // body->world velocity increment (gravity in)
            const double dt = imu_delta.dt;
            // World LINEAR acceleration this frame (~0 when not truly accelerating).
            const double ax = dvw.at<double>(0) / dt;
            const double ay = dvw.at<double>(1) / dt;
            const double az = dvw.at<double>(2) / dt - 9.81;   // remove gravity (Z-down ENU)
            // 2026-05-29 — HIGH-PASS DRIFT FILTER REMOVED (was the K=0-on-walks root cause).
            //
            // Cause: the HP filter (tau=2.0 s, subtracting accel_drift_lp_) was added
            //   2026-05-26 to stop raw accel velocity ramping to ~12 m/s. But a STEADY
            //   walk has near-constant velocity, which the HP filter treats as "drift"
            //   and cancels — so accel_vel_w_ read 0.08-0.45 m/s while the user walked
            //   ~1.3 m/s (v5_a logcat: accel_spd=0.08 at tsz=2.5). accel_dist_accum_
            //   then never reached the 2 m calibration gate => K never calibrated on
            //   walks (v5_a/v5_c K=0), and depth-flow speed read ~5x low.
            // Proof: offline analyze_accel_speed.py on the SAME walk:
            //   [zupt] (raw integrate + ZUPT re-zero, NO HP) -> median 0.78 m/s,
            //          FINAL drift 0.00 m/s  (CLEAN).
            //   [hp]   (current on-device) -> median 1.45 but FINAL 3.92 m/s (drifts).
            //   The ZUPT re-zero (below, is_static branch) alone bounds drift when the
            //   user stops periodically; the HP filter was both unnecessary AND harmful.
            // Change: integrate RAW gravity-removed linear accel. Drift is bounded by
            //   (a) ZUPT re-zero at every is_static, and (b) the K-calibration gate only
            //   trusting the window <=2.5 s after a stop (raw drift is small that soon
            //   after a clean v=0). accel_drift_lp_ kept (commented) for easy revert.
            /* LEGACY 2026-05-26 HP drift filter — decayed steady-walk velocity:
            constexpr double kAccelDriftTau = 2.0;
            const double a_lp = dt / (kAccelDriftTau + dt);
            accel_drift_lp_[0] += a_lp * (ax - accel_drift_lp_[0]);
            accel_drift_lp_[1] += a_lp * (ay - accel_drift_lp_[1]);
            accel_drift_lp_[2] += a_lp * (az - accel_drift_lp_[2]);
            */
            if (!is_static) {
                // Integrate RAW linear accel (gravity already removed above). ZUPT
                // re-zero at each stop bounds the drift (data-proven clean).
                accel_vel_w_[0] += ax * dt;
                accel_vel_w_[1] += ay * dt;
                accel_vel_w_[2] += az * dt;
                if (secs_since_zupt_ >= 0.0) secs_since_zupt_ += dt;
                // 2026-05-29 — only grow accel_dist_accum_ inside the trustworthy
                // post-ZUPT window (<=2.5 s). Past that, raw integration drift would
                // inflate the distance with no ZUPT to re-zero (the review's HIGH
                // finding: a run with few stops could ramp accel_dist and inflate K).
                // The K-calibration gate already requires secs_since_zupt_<=2.5, so
                // capping accumulation here changes nothing in-window but stops stale
                // drift from carrying numbers a later (still <=2.5 after a fresh stop)
                // calibration could mis-read. accel_vel_w_ keeps integrating (it is
                // re-zeroed at the next stop) — only the DISTANCE accumulator is frozen.
                if (secs_since_zupt_ >= 0.0 && secs_since_zupt_ <= 2.5) {
                    // 2026-05-29 — apply the known-distance walk bias correction so
                    // the accel-K reference matches tape-measured reality (see
                    // kAccelKBiasCorrection derivation). Scooter/run use vi_speed and
                    // are unaffected by this accel-K reference in practice.
                    accel_dist_accum_ += std::hypot(accel_vel_w_[0], accel_vel_w_[1])
                                         * dt * kAccelKBiasCorrection;
                }
            }
        }
    }
    if (is_static) {
        accel_vel_w_ = cv::Vec3d(0.0, 0.0, 0.0);
        secs_since_zupt_ = 0.0;
        accel_dist_accum_ = 0.0;
        visual_rel_dist_accum_ = 0.0;
        visual_rel_dist_loom_ = 0.0;   // 2026-05-29 — looming's separate accumulator
        // 2026-05-29 (Step B) — clear the VINS-Mono pair buffer at every stop so each
        // contiguous MOVING segment solves on a single coherent v0. The buffer only
        // appends moving pairs; carrying pre-stop pairs across a ZUPT would leave the
        // gamma (velocity reconstruction) blind to the unbuffered decel→0→reaccel Δv,
        // corrupting the solve exactly at stop-go (the scooter's dominant pattern).
        scale_estimator_vi_.reset();
        observer_c_pair_count_ = 0;
    }

    // 2026-05-29 (Fix A diagnosis) — consolidated accel-K calibration GATE STATE,
    // throttled. Behavior-neutral (logging only). The per-path calib logs live
    // INSIDE updateDepthFlowSpeed (essential-matrix verification_ok-gated) and
    // updateExpansionSpeed, so on slow-walk frames where verification fails they
    // never print — leaving us blind to whether the calib window even opens. This
    // line prints EVERY frame so a single real-walk logcat pins which gate binds:
    //   window_open = secs_since_zupt_ in [0.3, 2.5]  (only opens after a ZUPT / init reset)
    //   accel_dist  > kAccelKMinDistM (1.0 m) within that window
    //   K_df/K_loom = current calibrated scales (-1 = never calibrated)
    // Read alongside WALK_BG (is_static rate) to see if ZUPT ever fires at stops.
    if (frame_counter_ % 15 == 0 && ekf_.isFullInitialized()) {
        const double accel_spd_w = std::hypot(accel_vel_w_[0], accel_vel_w_[1]);
        const bool window_open = (secs_since_zupt_ >= 0.3 && secs_since_zupt_ <= 2.5);
        LOGI("ACCEL_K_STATE: is_static=%d tsz=%.2f window_open=%d accel_dist=%.2fm "
             "accel_spd=%.2f vis_rel_df=%.4f vis_rel_loom=%.4f K_df=%.1f K_loom=%.1f gait=%d",
             is_static ? 1 : 0, secs_since_zupt_, window_open ? 1 : 0,
             accel_dist_accum_, accel_spd_w, visual_rel_dist_accum_,
             visual_rel_dist_loom_,
             midas_scale_K_.load(std::memory_order_relaxed),
             expansion_scale_K_.load(std::memory_order_relaxed),
             static_cast<int>(active_mode_));
    }

    // 2026-05-18 falsifier: log EKF.b_g_ tagged with is_static so we can see
    // whether walking-phase b_g drifts differently from stationary. v42 walk
    // showed Madgwick yaw drift -0.31°/s during motion (zero stationary).
    // If walking b_g.z values trend systematically, MSCKF visual updates are
    // pumping b_g (visual rotation residuals attributed to bias). If walking
    // b_g.z stays near stationary value, the drift is from a different path.
    if (frame_counter_ % 30 == 0 && ekf_.isFullInitialized()) {
        cv::Mat bg = ekf_.getGyroBias();
        if (!bg.empty() && bg.rows == 3) {
            LOGI("WALK_BG: is_static=%d gyro_norm=%.3f ekf_bg=(%+.5f,%+.5f,%+.5f) "
                 "madgwick_yaw_deg=%.2f",
                 is_static ? 1 : 0, gyro_norm,
                 bg.at<double>(0, 0), bg.at<double>(1, 0), bg.at<double>(2, 0),
                 imu.getHeading() * 180.0 / M_PI);
        }
    }
    if (is_static) {
        ekf_.updateZUPT();
        // 2026-05-16 ZRUP: dual-of-ZUPT measurement update — observes gyro=0
        // when stationary, pulling EKF b_g_ toward the gyro window mean.
        // Without this, b_g_ was frozen indefinitely during stationary periods
        // (no parallax → no MSCKF update on b_g_), and the EKF→Madgwick
        // gyro-bias feedback (Tracker.cpp:914) carried a stale bias forward
        // every frame — producing the user-visible heading-drift symptom
        // (2026-05-16). Compute the gyro window mean from the same buffer
        // is_stationary used; sigma is the IMU per-sample noise density.
        if (ekf_.isFullInitialized()) {
            const auto& gyro_buf = imu.getGyroBuffer();
            constexpr int kZrupWindow = 20;
            const int N = std::min(static_cast<int>(gyro_buf.size()), kZrupWindow);
            if (N >= 5) {
                double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
                const int start = static_cast<int>(gyro_buf.size()) - N;
                for (int i = 0; i < N; ++i) {
                    sum_gx += gyro_buf[start + i].x;
                    sum_gy += gyro_buf[start + i].y;
                    sum_gz += gyro_buf[start + i].z;
                }
                const double mean_gx = sum_gx / N;
                const double mean_gy = sum_gy / N;
                const double mean_gz = sum_gz / N;
                // Per-sample gyro noise — current uncalibrated value lives in
                // EKFState::sigma_g_. Until Allan-variance characterization
                // lands, use the same uncalibrated source so ZRUP shares the
                // EKF's noise budget (TODO: read from EKF rather than hardcode).
                constexpr double kSigmaGyroSample = 0.01;
                if (ekf_.updateZRUP(mean_gx, mean_gy, mean_gz, kSigmaGyroSample, N)) {
                    navsight::eventCounters().zrup_fired_total.fetch_add(
                        1, std::memory_order_relaxed);
                }
            }
        }
        // Legacy Madgwick-side refinement: kept as a defense-in-depth backup
        // in case the EKF→Madgwick feedback loop is bypassed. Soft EMA on
        // IMUPreintegrator::gyro_bias_ — was structurally overwritten by the
        // setGyroBias feedback every frame before ZRUP landed.
        imu.refineGyroBiasDuringZUPT();
        consecutive_static_frames_++;

        // 2026-05-09 v13 — re-enable stationary specific-force update with
        // STRICT gating that the v9 attempt lacked. Closes the accel-bias
        // observability gap that produces the user-visible "drift in a
        // fixed world direction" symptom (gravity miscancel from residual
        // b_a integrates into v_G then p_G regardless of phone heading).
        //
        // Why the stricter gate works where v9 didn't:
        //   * v9 fired on EVERY ZUPT trigger → walking heel-strike frames
        //     (gyro low for ~30 ms, accel briefly stable) sneaked through
        //     and the residual got absorbed into b_a, runaway to 0.5 m/s².
        //   * v13 requires SUSTAINED stationarity: ≥15 consecutive ZUPT
        //     frames (~0.5 s @ 30 Hz) AND a much tighter gyro gate
        //     (< 0.1 rad/s vs walking-gait RMS 0.4 rad/s). Heel-strikes
        //     don't stay still for 0.5 s; only an intentional pause does.
        //
        // Constants are physics-derived, not magic:
        //   kSustainedStaticFrames = 15 — covers walking-stride period
        //     (typical 0.4-0.5 s gait cycle at 2 Hz) so a single foot
        //     contact never accumulates the count.
        //   kRotationStillGate = 0.1 rad/s — 4σ below walking-gait RMS
        //     (0.4 rad/s), corresponds to ~5.7 deg/s phone rotation,
        //     well below user's intentional pan-around motion.
        //   sigma_a = 0.1 m/s² — same kAccelNoiseSigma the gravity-
        //     alignment update uses, matches IMUPreintegrator
        //     accel_noise_sigma_.
        constexpr int kSustainedStaticFrames = 15;
        constexpr double kRotationStillGate  = 0.1;   // rad/s
        constexpr double kAccelNoiseSigmaZ   = 0.1;   // m/s²
        if (ekf_.isFullInitialized() &&
            consecutive_static_frames_ >= kSustainedStaticFrames &&
            gyro_norm < kRotationStillGate) {
            cv::Point3f g_zupt = imu.getFilteredGravity();
            cv::Mat a_zupt = (cv::Mat_<double>(3, 1) <<
                              g_zupt.x, g_zupt.y, g_zupt.z);
            // Return value checked: false means degenerate input (handled gracefully; no counter needed).
            static_cast<void>(ekf_.updateStationaryAccel(a_zupt, kAccelNoiseSigmaZ));
        }
    } else {
        consecutive_static_frames_ = 0;
        // 2026-05-09 v16 — "no translation" detector based on PDR step speed.
        // [SUPERSEDED 2026-05-26: the PDR step-speed proxy was replaced by a
        //  locomotion-agnostic gyro+velocity gate — see the code block below. The
        //  rationale here is retained as history for WHY the step proxy was tried.]
        //
        // The previous v13 path gated on `is_pure_rotation` (Rayleigh test
        // on optical flow direction) but that flag stayed FALSE across the
        // entire v15 walk (verified by grep: 0 PURE-ROTATION lines in
        // logcat, every GATES line shows rot=0). The Rayleigh test only
        // catches rotations where features fan out tangentially — yaw/pan
        // rotations produce parallel feature motion that LOOKS like
        // translation to that test, so it never fires for the typical
        // "user spins phone in hand" case.
        //
        // PDR step detector is far more reliable: it measures stride peaks
        // in body-frame accel and reports speed_mps. Walking gait gives
        // 0.5-1.5 m/s; in-place rotation/standing gives ~0 because there's
        // no stride peak. Gating on step_speed_mps < 0.1 catches:
        //   * Standing still (no movement at all)
        //   * Rotating phone in place (gyro non-zero, no translation)
        //   * Looking around / surveying without walking
        // Walking always produces step_speed >> 0.1 (the existing gate at
        // Tracker.cpp:1127 uses 0.3 to break is_static during real walks).
        //
        // Fire updateZUPT (zeros v_G + shrinks velocity covariance) but
        // NOT updateStationaryAccel — accel is dynamic during rotation and
        // the LP-filtered gravity lags too much to support a bias update.
        // The rotation rate constraint is implicit (gyro feeds R_GtoI via
        // propagateIMU; bounded by LC_GA gravity-alignment).
        // ── 2026-05-26: locomotion-agnostic rotate-in-place / no-translation guard ──
        // Cause:  the old gate fired updateZUPT() (which zeros v_G_) whenever the
        //         PDR step-speed < 0.1 m/s while the camera saw motion. That uses
        //         walking strides as the "am I translating?" proxy — pedestrian-
        //         only. A scooter/bike produces NO strides, so step_speed ≈ 0 on
        //         every cruising frame → this zeroed the fused velocity v_G_ every
        //         frame → the reported speed (now |v_G_|, see getFusedSpeedMps)
        //         could never build up for any non-walking motion. This IS the
        //         "speed calculated from steps is wrong" the user reported.
        // Change: drop the step proxy. The only failure this branch must still
        //         guard is the documented "user spins the phone in place → phantom
        //         translation arc" case (is_static misses it because rotational
        //         optical flow > 2.5 px forces is_static = false above). That case
        //         is, by definition, high rotation with no real body velocity. So
        //         fire ZUPT only when BOTH hold: gyro_norm > 0.8 rad/s (the
        //         project's established "rotating fast" threshold —
        //         IMUPreintegrator.cpp:449, Tracker.cpp:1993) AND |v_G_| < 0.5 m/s
        //         (below the pedestrian gait floor ~1.4 m/s — the filter is not
        //         meaningfully translating). Never fires for a cruising scooter
        //         (low gyro, sustained v_G_), nor mid-turn (high gyro BUT high
        //         v_G_), nor a straight walk (low gyro). Still fires for
        //         spin-in-hand-while-stopped.
        // Falsifier: zupt_rotinplace_fired ≈ 0 on a real moving ride; > 0 only on a
        //         deliberate stationary spin test. If v_G_ collapses to 0 while
        //         clearly moving, this gate (or the is_static path) is mis-firing —
        //         read the counter + the ZUPT_ROTINPLACE log line before tuning.
        constexpr double kRotInPlaceGyroRad = 0.8;   // rad/s — IMUPreintegrator.cpp:449
        constexpr double kRotInPlaceVelMps  = 0.5;   // m/s — below gait floor (~1.4 m/s)
        if (ekf_.isFullInitialized()) {
            double v_g_norm_else = 0.0;
            cv::Mat v_g_else = ekf_.getVelocity();
            if (!v_g_else.empty()) v_g_norm_else = cv::norm(v_g_else);
            if (gyro_norm > kRotInPlaceGyroRad && v_g_norm_else < kRotInPlaceVelMps) {
                ekf_.updateZUPT();
                navsight::eventCounters().zupt_rotinplace_fired.fetch_add(
                    1, std::memory_order_relaxed);
                LOGI("ZUPT_ROTINPLACE: fired gyro_norm=%.3f rad/s |v_G|=%.3f m/s "
                     "(rotate-in-place guard; locomotion-agnostic)",
                     gyro_norm, v_g_norm_else);
            }
        }
        /* SUPERSEDED 2026-05-26 (pedestrian step-speed translation proxy — see the
           Cause block above; this zeroed v_G_ on every non-stepping moving frame,
           which broke scooter/bike speed):
        const double step_speed_mps = imu.getStepInfo().speed_mps;
        constexpr double kNoTranslationStepGate = 0.1;  // m/s — well below walking-gait floor
        if (ekf_.isFullInitialized() &&
            step_speed_mps < kNoTranslationStepGate) {
            ekf_.updateZUPT();
        }
        */
    }

    // ── 8. Lens undistortion + Essential matrix + pose ───────────────────────
    bool pose_valid = false;
    // 2026-05-28 — `used_fallback` now stays false because the PDR fallback was
    // commented out (user request: legacy). Kept here so the poseFlags bit 8
    // packing at the bottom of processFrame keeps the same width; reads as 0.
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
    // 2026-05-18 falsifier: attribute pose_valid=false. v42 trans gates
    // showed 54% of sampled frames have pose_valid=false. That happens
    // when (a) outer gate below fails, (b) geometricVerification fails,
    // or (c) inlier_count_out / ratio gate fails. Per-30-frames log lets
    // us count each failure mode and identify the dominant cause.
    if (frame_counter_ % 30 == 0) {
        LOGI("POSE_VALID_GATE: tracked=%d sufficient_motion=%d has_parallax=%d "
             "!is_static=%d !frame_blurry=%d outer_pass=%d",
             tracked, sufficient_motion ? 1 : 0, has_parallax ? 1 : 0,
             !is_static ? 1 : 0, !frame_blurry_ ? 1 : 0,
             (sufficient_motion && has_parallax && !is_static && tracked >= 8 && !frame_blurry_) ? 1 : 0);
    }
    // ── IPM ground-plane speed runs OUTSIDE the recoverPose gate ─────────────────────────────────────
    // It needs only road-pixel FLOW + GRAVITY + camera height — NOT parallax, essential-matrix
    // verification, or !is_static. Gating it on the full gate above starved it: in offline replay (and on
    // a real ride whenever the gate flickers) it ran only sporadically, so its EMA held a STALE value and
    // the dt ballooned (10-frame gaps → wrong de-rotation → speed collapse). Run it on EVERY frame that has
    // enough tracked points. READ-ONLY (writes ground_flow_speed_mps_ only) so this never affects the dot.
    if (camera_height_m_ > 0.0 && !frame_blurry_ &&
        current_prev_pts_buf_.size() >= 8 && !current_prev_gray.empty() && !gray_buf_.empty()) {
        // FIX B (2026-06-03, BPROBE-proven): feed the IPM a DEDICATED BIG-WINDOW optical flow instead of
        // prev_good_buf_ (the production KLT survivors). The production window is rotation-sized (≤41px,
        // Tracker.cpp:2315), so it DROPS the fast near-ground road points (30-80px/frame at scooter speed) —
        // exactly the ones carrying the speed signal — leaving the IPM only slow far points → saturation
        // (corr(GPS,IPM)≈0). The live probe proved a 61px window + deep pyramid RECOVERS them (up to 164 pts
        // @ 45px flow on the ride, not blurred). Re-track the SAME detected points with that window and feed
        // the survivors; updateGroundFlowSpeed's forward-coherence + road-geometry + median gates reject any
        // big-window mismatches. LEGACY (prev_good_buf_ source) replaced 2026-06-03; see git history.
        std::vector<cv::Point2f> gp_prev, gp_next;
        {
            std::vector<cv::Point2f> bn; std::vector<uchar> bs; std::vector<float> be;
            cv::calcOpticalFlowPyrLK(current_prev_gray, gray_buf_, current_prev_pts_buf_, bn, bs, be,
                                     cv::Size(61, 61), 5,
                                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
            gp_prev.reserve(bn.size()); gp_next.reserve(bn.size());
            for (size_t i = 0; i < current_prev_pts_buf_.size() && i < bn.size(); ++i)
                if (bs[i]) { gp_prev.push_back(current_prev_pts_buf_[i]); gp_next.push_back(bn[i]); }
        }
        if (gp_prev.size() >= 8) {
            lens_.undistortMatchedPoints(gp_prev, gp_next);
            cv::Vec3d gp_gyro(0.0, 0.0, 0.0);
            if (!imu_delta.deltaR.empty() && imu_delta.deltaR.rows == 3 && imu_delta.deltaR.cols == 3) {
                cv::Mat gp_rvm; cv::Rodrigues(imu_delta.deltaR, gp_rvm);
                if (gp_rvm.rows == 3 && gp_rvm.cols == 1) {
                    const cv::Matx33d gp_Rbc = ekf_.getExtrinsicsRotation();
                    gp_gyro = gp_Rbc * cv::Vec3d(gp_rvm.at<double>(0), gp_rvm.at<double>(1), gp_rvm.at<double>(2));
                }
            }
            const double gp_dt = (timestamp_ns - current_prev_ts) * 1e-9;
            updateGroundFlowSpeed(gp_prev, gp_next, gp_dt, gp_gyro, imu);
        }
    }

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

        // Overlay: carry the recoverPose RANSAC inlier mask out to the per-point flags (parallel to
        // tracked_pts_flat). Only when the sizes align positionally — else leave all-0 rather than risk
        // a misaligned colouring (next_good_buf_ is unchanged between the flat-buffer build and here).
        if (status_verification.size() == tracked_pts_inlier_flags.size()) {
            for (size_t i = 0; i < status_verification.size(); ++i)
                tracked_pts_inlier_flags[i] = status_verification[i] ? 1 : 0;
        }

        // 2026-05-28 — gyro rotation vector in CAMERA frame (omega · dt) for
        // de-rotating the optical flow before computing looming/divergence.
        // Computed BEFORE the verification_ok gate because updateExpansionSpeed
        // (looming) now fires independently of essential-matrix verification —
        // looming uses only de-rotated flow + FOE from EKF heading, never
        // R_vo / t_vo from recoverPose. Without de-rotation, head turns
        // produce false forward-speed (research Rec 3: rotational flow corrupts
        // divergence at ~3r/f · |Δω|).
        cv::Vec3d gyro_rot_cam(0.0, 0.0, 0.0);
        if (!imu_delta.deltaR.empty() && imu_delta.deltaR.rows == 3 &&
            imu_delta.deltaR.cols == 3) {
            cv::Mat rv_body_mat;
            cv::Rodrigues(imu_delta.deltaR, rv_body_mat);
            if (rv_body_mat.rows == 3 && rv_body_mat.cols == 1) {
                const cv::Matx33d R_bc_ext = ekf_.getExtrinsicsRotation();
                const cv::Vec3d rv_body(rv_body_mat.at<double>(0),
                                        rv_body_mat.at<double>(1),
                                        rv_body_mat.at<double>(2));
                gyro_rot_cam = R_bc_ext * rv_body;
            }
        }
        const double dt_s_speed = (timestamp_ns - current_prev_ts) * 1e-9;

        // 2026-05-28 — looming/expansion-rate path. Moved OUT of the
        // verification_ok gate so it fires on slow walks where the essential
        // matrix degenerates (today's bug: walk had inlier ratio 0.03-0.19,
        // verification_ok=false for every frame → depth-flow never fired →
        // K never calibrated → UI=0). Looming only needs prev_ud/next_ud +
        // de-rotated flow + FOE-from-EKF-heading + K. It bails internally
        // if K hasn't been calibrated yet; K persists across app launches
        // (SharedPreferences) so cold-start walks inherit it from any prior
        // session that calibrated.
        // 2026-05-30 (Scale fix Steps 2/5): pass imu (gait classify) + gyro_norm
        // (turn-suppression). gyro_norm is the function-scope value computed at the
        // top of processFrame (rad/s); imu is in scope here.
        updateExpansionSpeed(prev_ud, next_ud, dt_s_speed, gyro_rot_cam, imu, gyro_norm);
        // (updateGroundFlowSpeed moved OUT of this gate — see the IPM block above the gate; it must run on
        //  every frame, not only when recoverPose's parallax/!is_static gate passes.)

        // Depth-weighted metric speed (see updateDepthFlowSpeed): recovers the
        // metric scale of the recoverPose translation from the tracked points'
        // MiDaS depths. STAYS gated on verification_ok because it consumes
        // R_vo/t_vo and would produce garbage otherwise.
        if (verification_ok && inlier_count_out >= static_cast<int>(MIN_INLIERS) &&
            !R_vo.empty() && !t_vo.empty()) {
            updateDepthFlowSpeed(prev_ud, next_ud, R_vo, t_vo, dt_s_speed, imu, gyro_norm);
        }
        // 2026-05-18 falsifier: log inner outcomes when outer gate passed
        if (frame_counter_ % 30 == 0) {
            const double r = tracked > 0 ? (double)inlier_count_out / tracked : 0.0;
            LOGI("POSE_VALID_INNER: verification_ok=%d inliers=%d/%d ratio=%.2f "
                 "min_inliers=%d min_ratio=%.2f inner_pass=%d",
                 verification_ok ? 1 : 0, inlier_count_out, tracked, r,
                 (int)MIN_INLIERS, (double)MIN_INLIER_RATIO,
                 (verification_ok && inlier_count_out >= MIN_INLIERS && r >= MIN_INLIER_RATIO) ? 1 : 0);
        }
        // 2026-05-19 #1 diag: when essential-matrix verification fails OR inlier
        // ratio is below threshold, log geometric context to identify failure
        // mode. Candidates: planar scene (homography degeneracy), pure rotation
        // (t_vo ≈ 0), narrow baseline (mean parallax small), or noisy KLT
        // matches (mean parallax small AND high motion). Mean parallax measured
        // as average displacement between matched FB-filtered points.
        if (frame_counter_ % 30 == 0 &&
            (!verification_ok || (tracked > 0 &&
             (double)inlier_count_out / tracked < MIN_INLIER_RATIO))) {
            double sum_d = 0.0, max_d = 0.0;
            int n = 0;
            for (size_t i = 0; i < prev_ud.size() && i < next_ud.size(); ++i) {
                const double dx_p = next_ud[i].x - prev_ud[i].x;
                const double dy_p = next_ud[i].y - prev_ud[i].y;
                const double d = std::sqrt(dx_p*dx_p + dy_p*dy_p);
                sum_d += d;
                if (d > max_d) max_d = d;
                n++;
            }
            const double mean_disp = n > 0 ? sum_d / n : 0.0;
            const double t_norm_dbg = t_vo.empty() ? 0.0 : cv::norm(t_vo);
            LOGI("EMAT_FAIL: ok=%d inl=%d/%d ratio=%.2f mean_disp_px=%.2f "
                 "max_disp_px=%.2f |t_vo|=%.4f gyro=%.3f",
                 verification_ok ? 1 : 0, inlier_count_out, tracked,
                 tracked > 0 ? (double)inlier_count_out / tracked : 0.0,
                 mean_disp, max_d, t_norm_dbg, gyro_norm);
        }

        if (verification_ok) {
            // Check SVD condition for translation degeneracy on the recovered E-matrix context
            // In OpenVINS, we often check if the translation is significant.
            double inlier_ratio = static_cast<double>(inlier_count_out) / tracked;

            // 2026-05-20 ROOT-CAUSE FIX — translation-degeneracy via parallax,
            // not |t_vo|.
            //
            // Cause: cv::recoverPose returns t as a unit vector (essential
            // matrix only determines translation direction — see Hartley &
            // Zisserman §9.6.2). The previous `t_norm < 0.001` and
            // `mean_flow < 1.5 && t_norm < 0.01` checks were mathematically
            // unreachable. Empirical proof in the 2026-05-20 heading_walk
            // sims: every EMAT_FAIL line logged |t_vo|=1.0000 (sample at
            // tests/sims/regression/visual/heading_walk_2026_05_20.logcat.txt
            // 15:01:58.732 … 15:02:22.667, n=14, std=0.0000). With the gate
            // hard-wired false, both downstream paths
            //   - Tracker.cpp:2380 updateRelativeRotation  (per-frame, 30Hz)
            //   - Tracker.cpp:3904 updateGravityAlignedYaw (per-keyframe, ~2Hz)
            // accepted catastrophic recoverPose decompositions when the user
            // walked toward a flat wall (low translation parallax → known
            // 4-fold (R,t) ambiguity in essential-matrix decomposition flips
            // sign frame-to-frame). hunt_ekf_yaw_jumps.py output: EKF R_GtoI
            // yaw-rate p99 = 65°/s vs Madgwick p99 = 5.3°/s, 13 jumps >30°/s
            // per walk. LC_ABS r_R median = 33-80°.
            //
            // Change: replace dead |t_vo| check with mean-parallax angle in
            // radians (mean KLT pixel displacement divided by focal length).
            // Threshold kVisualMinParallaxRad = 0.01 (≈ 0.57°) matches Fix #10
            // (EKFState::kSlamMinParallaxCos = 0.99995 → angle threshold
            // 0.57°), which itself cites OpenVINS UpdaterSLAM
            // min_parallax_ratio = 0.01 (Geneva et al. 2020 §III.D).
            //
            // The mean-displacement loop below is the same computation
            // already used in the EMAT_FAIL diagnostic at lines ~2138-2148;
            // hoisting it from the failure-only conditional to every-frame
            // execution adds ~150 µs/frame for typical N≈100 matches.
            //
            // Falsifier: re-walk facing a flat wall →
            //   visual_translation_degenerate_total > 0 (gate firing)
            //   OVERLAY_SNAPSHOT R_GtoI_yaw_deg tracks Madgwick within ±10°
            //   LC_ABS r_R median < 15°
            //   loop_closure_accepts climbs above 1/walk
            //
            // LEGACY: previous unreachable t_norm-based gate, kept commented
            // per project no-deletion convention; remove only after this
            // parallax gate is walk-validated on at least 3 sims.
            /* LEGACY 2026-05-20:
            // Translation degeneracy: check if t_vo is too small (pure rotation)
            // NOTE: Do NOT use SVD condition of E — essential matrix is always rank 2
            // by definition (σ,σ,0), so condition number is always infinite.
            double t_norm = t_vo.empty() ? 0.0 : cv::norm(t_vo);
            if (t_norm < 0.001) translation_degenerate = true;
            // Also check: if flow is mostly explained by rotation (low parallax)
            if (mean_flow < 1.5 && t_norm < 0.01) translation_degenerate = true;
            */
            double sum_disp_px = 0.0;
            int    n_disp_px   = 0;
            const size_t n_pairs = std::min(prev_ud.size(), next_ud.size());
            for (size_t i = 0; i < n_pairs; ++i) {
                const double dx_p = next_ud[i].x - prev_ud[i].x;
                const double dy_p = next_ud[i].y - prev_ud[i].y;
                sum_disp_px += std::sqrt(dx_p * dx_p + dy_p * dy_p);
                n_disp_px++;
            }
            const double mean_disp_px = n_disp_px > 0
                ? sum_disp_px / static_cast<double>(n_disp_px)
                : 0.0;
            const double focal_px = K.at<double>(0, 0);
            const double parallax_rad = focal_px > 1e-6
                ? mean_disp_px / focal_px
                : 0.0;
            // 0.01 rad ≈ 0.57°. Mirrors Fix #10 (EKFState::kSlamMinParallaxCos).
            constexpr double kVisualMinParallaxRad = 0.01;
            if (parallax_rad < kVisualMinParallaxRad) {
                translation_degenerate = true;
                navsight::eventCounters()
                    .visual_translation_degenerate_total
                    .fetch_add(1, std::memory_order_relaxed);
            }
            if (frame_counter_ % 30 == 0) {
                LOGI("PARALLAX_GATE: mean_disp_px=%.2f focal_px=%.1f "
                     "parallax_rad=%.5f thresh=%.5f degenerate=%d",
                     mean_disp_px, focal_px, parallax_rad,
                     kVisualMinParallaxRad,
                     translation_degenerate ? 1 : 0);
            }
            double svd_cond = 0.0;  // kept for logging

            if (inlier_count_out >= MIN_INLIERS && inlier_ratio >= MIN_INLIER_RATIO) {
                // ── Triangulation (only when translation is reliable) ───
                if (!translation_degenerate && !t_vo.empty()) {
                    cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);
                    cv::Mat Rt(3, 4, CV_64F);
                    R_vo.copyTo(Rt(cv::Range(0,3), cv::Range(0,3)));
                    t_vo.copyTo(Rt(cv::Range(0,3), cv::Range(3,4)));
                    cv::Mat P2 = K * Rt;

                    // 2026-05-18 TRIANGULATION FIX (root cause of dead MiDaS).
                    //
                    // Cause:   v40-v44 walks showed 99.7% of points exit
                    //          triangulation with w_homogeneous ≤ 1e-6 → all
                    //          marked as (0,0,0) sentinel → applyDepthScale
                    //          Constraint filter kept zero points → MiDaS
                    //          observer B silently dead since session start.
                    //          TWO compounding bugs:
                    //            (1) `cv::triangulatePoints(P1, P2, prev_ud,
                    //                next_ud, ...)` ignored RANSAC inlier mask
                    //                from geometricVerification. Outliers have
                    //                inconsistent epipolar geometry → near-
                    //                singular SVD → w ≈ 0 even for nearby
                    //                inliers (numerical contamination).
                    //            (2) Passing std::vector<Point2f> relies on
                    //                OpenCV 4.5.3's implicit conversion to
                    //                2×N matrix which can produce 1×N CV_32FC2
                    //                instead — fragile and version-dependent.
                    //
                    // Change:  (a) Filter prev_ud/next_ud to inliers only
                    //          (status_verification flags from
                    //          geometricVerification, which DID compute them
                    //          but the call site was ignoring).
                    //          (b) Build pts1_mat / pts2_mat as explicit
                    //          2×N CV_64F matrices.
                    //          (c) Map triangulated results BACK to original
                    //          feature indices so MiDaS pts2d[i]↔pts3d[i]
                    //          alignment is preserved (outlier slots stay
                    //          (0,0,0) sentinel — same as before).
                    //
                    // Falsifier: next walk should show
                    //              gate_zvio counter % << 100% (was 100%),
                    //              midas_fused > 0 (was always 0),
                    //              midas_affine_fit_inlier_ratio_milli ≥ 500,
                    //              and TRIANG_DIST showing g_pass > 0 and
                    //              w_small fraction in the 0-30% range
                    //              (down from 99.7%).
                    std::vector<int> inlier_idx;
                    inlier_idx.reserve(status_verification.size());
                    for (size_t i = 0; i < status_verification.size(); ++i) {
                        if (status_verification[i]) inlier_idx.push_back(static_cast<int>(i));
                    }
                    const int N_in = static_cast<int>(inlier_idx.size());

                    points_3d_current_.assign(prev_ud.size(), cv::Point3f(0, 0, 0));

                    int g_w_small = 0, g_pz_small = 0, g_pass = 0;
                    int b_lt1=0, b_1_6=0, b_6_25=0, b_25_100=0, b_gt100=0;
                    double err_min = 1e18, err_max = -1.0;
                    int reproj_outliers = 0;

                    cv::Mat pts4d;
                    if (N_in >= 8) {
                        cv::Mat pts1_mat(2, N_in, CV_64F);
                        cv::Mat pts2_mat(2, N_in, CV_64F);
                        for (int j = 0; j < N_in; ++j) {
                            const int i = inlier_idx[j];
                            pts1_mat.at<double>(0, j) = static_cast<double>(prev_ud[i].x);
                            pts1_mat.at<double>(1, j) = static_cast<double>(prev_ud[i].y);
                            pts2_mat.at<double>(0, j) = static_cast<double>(next_ud[i].x);
                            pts2_mat.at<double>(1, j) = static_cast<double>(next_ud[i].y);
                        }
                        cv::triangulatePoints(P1, P2, pts1_mat, pts2_mat, pts4d);

                        // 2026-05-18 second pass: v46 walk showed 100%
                        // w_small after inlier filtering. Suspecting OpenCV
                        // 4.5.3 outputs negative w for valid points (SVD
                        // sign convention is arbitrary). Switch to |w| > eps,
                        // and one-shot log the actual w/Z values so we can
                        // verify magnitudes are sane.
                        if (frame_counter_ % 30 == 0 && pts4d.cols >= 3) {
                            LOGI("TRIANG_RAW: pts4d.type=%d cols=%d "
                                 "sample0=(%.4f,%.4f,%.4f,%.6f) "
                                 "sample1=(%.4f,%.4f,%.4f,%.6f) "
                                 "sample2=(%.4f,%.4f,%.4f,%.6f)",
                                 pts4d.type(), pts4d.cols,
                                 pts4d.at<double>(0,0), pts4d.at<double>(1,0),
                                 pts4d.at<double>(2,0), pts4d.at<double>(3,0),
                                 pts4d.at<double>(0,1), pts4d.at<double>(1,1),
                                 pts4d.at<double>(2,1), pts4d.at<double>(3,1),
                                 pts4d.at<double>(0,2), pts4d.at<double>(1,2),
                                 pts4d.at<double>(2,2), pts4d.at<double>(3,2));
                        }

                        for (int j = 0; j < pts4d.cols; ++j) {
                            const int i_orig = inlier_idx[j];
                            cv::Mat p = pts4d.col(j);
                            double w = p.at<double>(3);
                            if (std::abs(w) > 1e-6) {
                                double X = p.at<double>(0)/w;
                                double Y = p.at<double>(1)/w;
                                double Z = p.at<double>(2)/w;

                                cv::Mat pt3 = (cv::Mat_<double>(3,1) << X, Y, Z);
                                cv::Mat proj = K * (R_vo * pt3 + t_vo);
                                double pz = proj.at<double>(2);
                                if (pz > 1e-6) {
                                    double px = proj.at<double>(0) / pz;
                                    double py = proj.at<double>(1) / pz;
                                    double err_sq = (next_ud[i_orig].x - px) * (next_ud[i_orig].x - px)
                                                  + (next_ud[i_orig].y - py) * (next_ud[i_orig].y - py);
                                    if (err_sq < err_min) err_min = err_sq;
                                    if (err_sq > err_max) err_max = err_sq;
                                    if      (err_sq < 1.0)   b_lt1++;
                                    else if (err_sq < 6.0)   b_1_6++;
                                    else if (err_sq < 25.0)  b_6_25++;
                                    else if (err_sq < 100.0) b_25_100++;
                                    else                     b_gt100++;
                                    if (err_sq > 5.991) {
                                        reproj_outliers++;
                                        continue;  // leave sentinel in place
                                    }
                                    g_pass++;
                                    points_3d_current_[i_orig] = cv::Point3f(
                                        static_cast<float>(X),
                                        static_cast<float>(Y),
                                        static_cast<float>(Z));
                                } else {
                                    g_pz_small++;
                                }
                            } else {
                                g_w_small++;
                            }
                        }
                    }
                    if (reproj_outliers > 0 && frame_counter_ % 30 == 0) {
                        LOGI("REPROJ_GATE: rejected %d/%d outlier points",
                             reproj_outliers, pts4d.cols);
                    }
                    // 2026-05-18 falsifier: dump distribution per triangulation.
                    // pass+reproj_out = points reaching err_sq test; sum of
                    // buckets equals that. Disjoint: g_w_small (w≈0),
                    // g_pz_small (behind camera), g_pass (kept).
                    if (frame_counter_ % 30 == 0 && pts4d.cols > 0) {
                        LOGI("TRIANG_DIST: pts=%d w_small=%d pz_small=%d pass=%d reproj_out=%d "
                             "err_sq_buckets[<1:%d,1-6:%d,6-25:%d,25-100:%d,>100:%d] "
                             "err_range=[%.2f,%.2f] |t_vo|=%.3f",
                             pts4d.cols, g_w_small, g_pz_small, g_pass, reproj_outliers,
                             b_lt1, b_1_6, b_6_25, b_25_100, b_gt100,
                             err_min < 1e17 ? err_min : 0.0,
                             err_max,
                             cv::norm(t_vo));
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
                    !is_static &&
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

                        // 2026-05-21 BUG 4 ROOT-CAUSE FIX — gyro-vs-visual
                        // consistency gate at the sensor-disagreement layer.
                        //
                        // Cause: cv::recoverPose decomposes E into 4 candidate
                        // (R, t) solutions and picks via cheirality. On planar
                        // / low-parallax scenes the vote is noisy and the wrong
                        // (sign-flipped) R can be picked even at high inlier
                        // counts. The previous chi² gate (now reverted) used
                        // state covariance which self-defeats as P grows from
                        // propagation-without-updates.
                        //
                        // Change: compare R_vo_body to the gyro-integrated
                        // rotation between consecutive frames (imu_delta.deltaR
                        // is body-frame rotation, computed by IMUPreintegrator
                        // with bias correction). If they disagree by > 2°,
                        // trust the gyro (its drift over 33ms is negligible)
                        // and discard the visual measurement.
                        //
                        // Threshold derivation (sensor-physics):
                        //   Madgwick yaw-rate p99               = 1.87°/s
                        //   Frame interval                       = 33 ms
                        //   Real body rotation per frame (3σ)    = 0.06°
                        //   Plus turn rate (~30°/s × 33ms)        = 1.0°
                        //   Plus visual measurement noise         = 0.1°
                        //   Sum (worst legit disagreement)        ≈ 1.2°
                        //   Safety factor 1.7×                    → 2°
                        //
                        // Falsifier: post-fix walk →
                        //   visual_relative_rotation_gyro_mismatch_total > 0
                        //   EKF R_GtoI yaw-rate p99 → < 10°/s
                        constexpr double kFrameGyroVisualMaxDisagreeRad =
                            2.0 * M_PI / 180.0;  // 2°
                        bool gyro_visual_ok = true;
                        if (!imu_delta.deltaR.empty() &&
                            imu_delta.deltaR.rows == 3 &&
                            imu_delta.deltaR.cols == 3 &&
                            imu_delta.deltaR.type() == CV_64F) {
                            cv::Mat R_disagree = R_vo_body * imu_delta.deltaR.t();
                            cv::Mat r_axis;
                            cv::Rodrigues(R_disagree, r_axis);
                            const double mag = cv::norm(r_axis);
                            if (std::isfinite(mag) &&
                                mag > kFrameGyroVisualMaxDisagreeRad) {
                                navsight::eventCounters()
                                    .visual_relative_rotation_gyro_mismatch_total
                                    .fetch_add(1, std::memory_order_relaxed);
                                gyro_visual_ok = false;
                                if (frame_counter_ % 30 == 0) {
                                    LOGI("VISUAL_ROT_GYRO_REJECT: |R_vo·R_gyro^T|=%.2f° "
                                         "threshold=%.2f° inliers=%d "
                                         "(recoverPose sign-flip suspected)",
                                         mag * 180.0 / M_PI,
                                         kFrameGyroVisualMaxDisagreeRad * 180.0 / M_PI,
                                         inlier_count_out);
                                }
                            }
                        }

                        if (gyro_visual_ok &&
                            !ekf_.updateRelativeRotation(R_vo_body, sigma_axis_sq,
                                                         prev_clone_id)) {
                            navsight::eventCounters().relative_rotation_rejected.fetch_add(
                                1, std::memory_order_relaxed);
                            LOGI("updateRelativeRotation rejected (clone missing / EKF not init / malformed R)");
                        }
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
                    // Fix B (2026-05-16): use EKF rotation directly (SSOT
                    // post-init). global_R_ was a per-frame mirror — same
                    // value but semantically EKF is authoritative.
                    // 2026-05-29 (Step B) — FRAME FIX: ScaleEstimatorVI needs R_w_b =
                    // body->world (it rotates body-frame t_vis/Δp/Δv into the WORLD
                    // frame to match gravity_w_=(0,0,-9.81)). ekf_.getRotation() returns
                    // R_GtoI = world->body, and global_R_ is also R_GtoI — so BOTH must
                    // be TRANSPOSED. Passing the un-transposed (world->body) matrix put
                    // the visual/Δp/Δv terms in the body frame while gravity stayed in
                    // world → frame-mixed A·x=b → biased s. This pre-existing bug (plus
                    // the 0.5s buffer) is why Observer C never produced a usable scale.
                    kp.R_w_b = (ekf_.isFullInitialized()
                                    ? ekf_.getRotation()
                                    : global_R_).t();
                    kp.dt = imu_delta.dt;
                    // 2026-05-29 (Step B FRAME FIX #2) — t_vo from cv::recoverPose is in
                    // the CAMERA frame (TrackKLT.cpp:118 R_cam,t_cam), unit-norm. The
                    // solver rotates t_vis_body by R_w_b (body->world) to live in the
                    // SAME world frame as delta_p_body (which IS body-frame). So t_vo
                    // must first be rotated CAMERA->BODY by R_bc.t() (R_bc =
                    // getExtrinsicsRotation = body->camera, per EKFState.h:259). Passing
                    // the raw camera-frame t_vo as body-frame left the visual term
                    // mis-rotated by the extrinsic (S21 R_bc ~ diag(1,-1,-1), a near-180
                    // flip) -> visual and inertial displacements in different frames ->
                    // s came out small/negative & rejected (v7: s=0.03, relsig 0.35).
                    // PROVEN offline (scripts/test_scale_estimator.py): camera_bug ->
                    // s rejected; camera_fixed (R_bc.t()*t_vo) -> s ratio 0.99. Root-cause
                    // frame correction, NOT a tuned factor.
                    {
                        const cv::Matx33d R_bc = ekf_.getExtrinsicsRotation();  // body->camera
                        const cv::Vec3d t_cam(t_vo.at<double>(0), t_vo.at<double>(1),
                                              t_vo.at<double>(2));
                        kp.t_vis_body = R_bc.t() * t_cam;                       // camera->body
                    }
                    kp.delta_p_body = cv::Vec3d(imu_delta.deltaP.at<double>(0),
                                                imu_delta.deltaP.at<double>(1),
                                                imu_delta.deltaP.at<double>(2));
                    kp.delta_v_body = cv::Vec3d(imu_delta.deltaV.at<double>(0),
                                                imu_delta.deltaV.at<double>(1),
                                                imu_delta.deltaV.at<double>(2));
                    scale_estimator_vi_.addKeyframePair(kp);
                    observer_c_pair_count_++;
                    // 2026-05-29 (Step B) — running mean of pair dt, to convert the
                    // solved per-pair metric displacement s into a metric SPEED
                    // (vi_speed = s / mean_dt). EMA so it tracks the ~uniform frame dt.
                    vi_pair_dt_mean_ += 0.05 * (kp.dt - vi_pair_dt_mean_);

                    // 2026-05-18 falsifier: observer C never fires in v40 walk
                    // (OBS_C log count = 0) despite outer gate looking permissive.
                    // Log per-30-frames so we see pair count vs solve cadence.
                    if (frame_counter_ % 30 == 0) {
                        LOGI("OBS_C_STATE: pairs_added=%d size=%zu interval=%d min_pairs=%zu "
                             "|t_vo|=%.3f dt=%.4f gate_static=%d gate_pure_rot=%d",
                             observer_c_pair_count_, scale_estimator_vi_.size(),
                             OBSERVER_C_SOLVE_INTERVAL, (size_t)ScaleEstimatorVI::MIN_PAIRS,
                             cv::norm(t_vo), imu_delta.dt,
                             is_static ? 1 : 0, is_pure_rotation ? 1 : 0);
                    }

                    if (observer_c_pair_count_ % OBSERVER_C_SOLVE_INTERVAL == 0
                        && scale_estimator_vi_.size() >= ScaleEstimatorVI::MIN_PAIRS) {
                        double s_obs = 0.0, var_obs = 0.0;
                        const bool solved = scale_estimator_vi_.solve(s_obs, var_obs);
                        const bool accept = solved && std::isfinite(s_obs) && std::isfinite(var_obs)
                            && s_obs > 0.01 && s_obs < 10.0 && var_obs > 0.0;
                        if (accept) {
                            // Inflate variance: per-frame unit-norm visual
                            // translations are noisy and Hesch/Martinelli
                            // assumes consistent scale across pairs. Floor
                            // keeps Observer C from dominating the fuser.
                            double r_var = std::max(var_obs, 0.04);
                            scale_fuser_.update(s_obs, r_var);

                            // 2026-05-29 (Step B) — publish the VINS-Mono metric SPEED.
                            // s_obs = recovered metric magnitude of the per-pair visual
                            // translation; with per-frame pairs that is displacement over
                            // ~one frame, so speed = s_obs / mean_pair_dt. ACCEPT only a
                            // CONFIDENT solve (low variance) as the metric anchor —
                            // var_obs is the scale variance; require its 1σ < 25% of s so
                            // an unobservable (constant-velocity / degenerate-recoverPose)
                            // solve does not poison K. Consumed by updateDepthFlowSpeed.
                            const double rel_sigma = std::sqrt(var_obs) / std::max(1e-6, s_obs);
                            if (rel_sigma < 0.25 && vi_pair_dt_mean_ > 1e-3) {
                                const double vi_speed = s_obs / vi_pair_dt_mean_;
                                if (std::isfinite(vi_speed) && vi_speed > 0.0 && vi_speed < 30.0) {
                                    vi_metric_speed_mps_.store(vi_speed, std::memory_order_relaxed);
                                    vi_speed_ts_ns_.store(timestamp_ns, std::memory_order_relaxed);
                                }
                            }
                        }
                        // 2026-05-29 — UNGATED log (was %30-gated inside the accept
                        // branch, so failures + most successes were invisible). Shows
                        // whether the solve fires and why it is/ isn't accepted.
                        LOGI("OBS_C: solved=%d s=%.4f var=%.5f relσ=%.2f N=%zu accept=%d "
                             "-> vi_speed=%.2f m/s (%.1f km/h) fuser_s=%.4f",
                             solved ? 1 : 0, s_obs, var_obs,
                             (accept && s_obs > 1e-6) ? std::sqrt(var_obs) / s_obs : -1.0,
                             scale_estimator_vi_.size(), accept ? 1 : 0,
                             accept ? s_obs / std::max(1e-3, vi_pair_dt_mean_) : -1.0,
                             accept ? (s_obs / std::max(1e-3, vi_pair_dt_mean_)) * 3.6 : -1.0,
                             scale_fuser_.scale());
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
        // Fix B (2026-05-16 audit Finding 2.1): establish single-source-of-
        // truth for heading and rotation. Architecture:
        //   - scalar_heading_: per-frame WRITE-only cache of imu.getHeading().
        //     Read by getHeading() (JNI) and by keyframe stores that need the
        //     heading AT a specific frame. Never derives from EKF yaw (EKF
        //     under-rotates fast turns — 2026-05-03 V-shape bug). Stays as a
        //     member because getHeading() has no imu reference.
        //   - global_R_: REMOVED as autonomous state post-init. Now a pure
        //     read mirror of ekf_.getRotation() in the post-init branch.
        //     Pre-init bootstrap writes remain (needed before EKF fires).
        // Cause: 4 distinct yaw-extraction formulas (audit Finding 2.1).
        // Change: heading always = imu.getHeading() (V-shape fix preserved);
        //   global_R_ post-init always = ekf_.getRotation() (EKF is SSOT).
        // Falsifier: getHeading() in walking test == imu.getHeading() exactly;
        //   global_R_ == ekf_.getRotation() after first propagateIMU.
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
            // Fix B: global_R_ is now a read mirror of ekf_.getRotation()
            // (EKF is SSOT for rotation post-init). The EKF rotation is
            // gravity-corrected by updateGravityAlignedYaw; all consumers
            // that need R_w_b now read through ekf_.getRotation() directly.
            // Old code (pre-Fix B):
            //   global_R_ = ekf_.getRotation();  // still needed as mirror
            // New code: same value, but semantically: EKF is authoritative.
            global_R_ = ekf_.getRotation();  // Fix B mirror (2026-05-16)
        } else {
            // Bootstrap-only: before EKF initializeFull, fall back to
            // Madgwick yaw. global_R_ holds the bootstrap Rz(azimuth) or
            // Madgwick R_GtoI set by setInitialHeading / mag-one-shot.
            scalar_heading_ = static_cast<double>(imu.getHeading());
            while (scalar_heading_ >  M_PI) scalar_heading_ -= 2.0 * M_PI;
            while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;
        }
        heading = scalar_heading_;

        // ── 9.0b Map-as-sensor HEADING leg (2026-05-31) ─────────────────────────────
        // Correct the Madgwick heading toward the matched ROAD bearing so the trajectory stops
        // drifting off the road ("we should always be on the road we start on"). The Kotlin matcher
        // pushes the railed road's bearing (FREE_ROAD + confident + straight) via setRoadHeadingHint;
        // here we nudge a FRACTIONAL step toward it, mirroring the visual yaw nudge (kBug5SyncStrength):
        // gated on a SMALL residual (|hdg-road| < kRoadMaxResidualRad=35°, so a ~90° crossing road is
        // REJECTED — this can only un-drift, never re-aim), a FRESH hint, the magnetometer NOT actively
        // fusing (mag is the absolute reference), and the enable flag. Corrects the Madgwick SOURCE
        // (not scalar_heading_, which is re-read from it) so the fix persists, then re-reads so THIS
        // frame's dot advance uses it.
        if (road_heading_correction_enabled_ && ekf_.isFullInitialized() && !imu.isMagActivelyFusing()) {
            const int hint_age = frames_since_road_hint_.fetch_add(1, std::memory_order_relaxed);
            const double road_deg = road_heading_hint_deg_.load(std::memory_order_relaxed);
            if (hint_age < kRoadHintMaxAgeFrames && road_deg >= -180.0 && road_deg <= 360.0) {
                const double road_rad = road_deg * M_PI / 180.0;
                double resid = road_rad - scalar_heading_;
                while (resid >  M_PI) resid -= 2.0 * M_PI;
                while (resid < -M_PI) resid += 2.0 * M_PI;
                // The matched road bearing is ±180° ambiguous. Align to the road direction that matches
                // TRAVEL (nearer the current heading), not its reverse — this prevents the instant 180°
                // flip the owner saw AND lets us correct a LARGE initial offset (the road we are ON).
                double resid_rev = resid + M_PI;
                while (resid_rev >  M_PI) resid_rev -= 2.0 * M_PI;
                while (resid_rev < -M_PI) resid_rev += 2.0 * M_PI;
                if (std::abs(resid_rev) < std::abs(resid)) resid = resid_rev;
                if (std::isfinite(resid) && std::abs(resid) < kRoadMaxResidualRad) {
                    imu.nudgeMadgwickYawAroundWorldZ(kRoadSyncStrength * resid);
                    navsight::eventCounters().madgwick_road_yaw_nudges_total.fetch_add(1, std::memory_order_relaxed);
                    scalar_heading_ = static_cast<double>(imu.getHeading());
                    while (scalar_heading_ >  M_PI) scalar_heading_ -= 2.0 * M_PI;
                    while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;
                    heading = scalar_heading_;
                    if (frame_counter_ % 30 == 0) {
                        LOGI("MADG_ROAD_SYNC: road=%.1f° hdg=%.1f° resid=%+.1f° applied=%+.1f° age=%d",
                             road_deg, scalar_heading_ * 180.0 / M_PI, resid * 180.0 / M_PI,
                             kRoadSyncStrength * resid * 180.0 / M_PI, hint_age);
                    }
                }
            }
        }

        if (frame_counter_ % 30 == 0) {
            const float m_yaw   = imu.getHeading() * 180.0f / static_cast<float>(M_PI);
            const float m_roll  = imu.getMadgwickRoll()  * 180.0f / static_cast<float>(M_PI);
            const float m_pitch = imu.getMadgwickPitch() * 180.0f / static_cast<float>(M_PI);
            // 2026-05-19 diag: OVERLAY_SNAPSHOT yaw uses raw atan2(R[1,0],R[0,0])
            // which is corrupted by tilt. The TRUE EKF yaw uses getYaw(roll,pitch)
            // which applies a tilt-removal sandwich. Print both so we know if
            // the EKF-Madgwick "divergence" we've been measuring is real or just
            // tilt contamination in the overlay extractor.
            const double ekf_yaw_true = ekf_.isFullInitialized()
                ? ekf_.getYaw(imu.getMadgwickRoll(), imu.getMadgwickPitch()) * 180.0 / M_PI
                : 0.0;
            // Raw atan2 (what OVERLAY_SNAPSHOT prints) for cross-reference.
            cv::Mat R_GtoI_raw = ekf_.getRotation();
            const double ekf_yaw_raw = R_GtoI_raw.empty() ? 0.0 :
                std::atan2(R_GtoI_raw.at<double>(1,0),
                           R_GtoI_raw.at<double>(0,0)) * 180.0 / M_PI;
            LOGI("HEADING: madgwick_yaw=%.1f° roll=%.1f° pitch=%.1f° "
                 "ekf_yaw_TRUE=%.1f° ekf_yaw_raw_overlay=%.1f° "
                 "M-EKF_gap=%+.1f° raw_overlay_gap=%+.1f°",
                 m_yaw, m_roll, m_pitch,
                 ekf_yaw_true, ekf_yaw_raw,
                 m_yaw - static_cast<float>(ekf_yaw_true),
                 m_yaw - static_cast<float>(ekf_yaw_raw));
        }

        // Step 2.3: Madgwick is the heading reference; FEJ is for position only.

        // 2026-05-17 — widened gate for "sitting and rotating" motion that
        // defeats is_static (gyro != 0) and is_pure_rotation (Rayleigh
        // threshold too strict). Per viewer screenshot: phone in chair,
        // user looking around → trail accumulated 50 m of phantom motion
        // in seconds because KLT subpixel noise produces tiny t_vo that
        // gets scaled to meters by scale_fuser.
        //
        // rotation_dominated = high gyro + no PDR step detection. Catches
        // the casual "look around" case that strict gates miss.
        const bool rotation_dominated =
            (gyro_norm > 0.2 && imu.getStepInfo().speed_mps < 0.1);
        // 2026-05-30 Fix B — decouple the user-facing dot advance from the
        // essential-matrix pose_valid flag. ROOT CAUSE: forward locomotion moves
        // the camera along its optical axis (focus-of-expansion in-image, ~0
        // central parallax, recoverPose inlier ratio 0.03-0.19), so pose_valid is
        // false on ~74% of walk frames and the dot froze 93% of the time. The
        // forward-speed signal IS observable via looming (depth_flow_speed_mps_,
        // set by updateExpansionSpeed independently of recoverPose); the dot's
        // translation needs only (heading, forward-speed), neither of which needs
        // the essential matrix. So advance whenever a trusted visual pose OR a
        // valid looming speed exists. is_static/rotation_dominated above still
        // freeze true stops and in-place rotation (anti-phantom-drift preserved);
        // the EKF relative-pose fusion and the t_vo-based vertical stay gated on
        // the real pose (pose_path_ok) since they consume recoverPose's t_vo.
        const double df_speed_gate =
            depth_flow_speed_mps_.load(std::memory_order_relaxed);
        const bool pose_path_ok = pose_valid && !is_pure_rotation
                               && !translation_degenerate && quality >= 0.15;
        // 1 cm/s floor (sub-noise): the freeze branch decays depth_flow_speed_mps_
        // toward 0 (never < 0) at a stop, and warm-up holds the -1 sentinel; a bare
        // ">= 0" would fire the fallback on those decay/sentinel artifacts with
        // disp~0. Below 1 cm/s there is no real locomotion, so require a genuinely
        // positive looming speed before advancing the dot without a visual pose.
        const bool looming_speed_ok = (df_speed_gate > 0.01);
        // 2026-05-18 falsifier: per-gate skip counters so we can attribute
        // the 58m PDR-vs-VIO gap. The accumulator at the else-if branch only
        // fires when ALL gates pass; each gate that fires here is "lost"
        // displacement that PDR sees but VIO doesn't record.
        if (frame_counter_ % 30 == 0) {
            LOGI("TRANS_ACC_GATES: is_static=%d rot_dom=%d pose_valid=%d "
                 "pure_rot=%d trans_degen=%d quality=%.3f q_gate_ok=%d gyro=%.3f",
                 is_static ? 1 : 0, rotation_dominated ? 1 : 0, pose_valid ? 1 : 0,
                 is_pure_rotation ? 1 : 0, translation_degenerate ? 1 : 0,
                 quality, (quality >= 0.15) ? 1 : 0, gyro_norm);
        }
        if (is_static || rotation_dominated) {
            // 2026-05-26 — stopped / rotating-in-place ⇒ no translation ⇒ decay the
            // reported depth-flow speed toward 0 (EMA τ≈0.5s) so a true stop reads ~0.
            {
                const double dt_dec = (timestamp_ns - current_prev_ts) * 1e-9;
                const double cur_sp = depth_flow_speed_mps_.load(std::memory_order_relaxed);
                if (cur_sp > 0.0 && dt_dec > 0.0 && dt_dec < 1.0) {
                    const double a_dec = dt_dec / (0.5 + dt_dec);
                    depth_flow_speed_mps_.store(cur_sp * (1.0 - a_dec),
                                                std::memory_order_relaxed);
                }
            }
            // 2026-05-28 — trajectory frozen ⇒ UI shows 0 km/h instantly. Don't
            // decay; the user expects "you stopped" to register at once.
            trajectory_speed_mps_.store(0.0, std::memory_order_relaxed);
            // Translation frozen — no global_t_ update.
            if (rotation_dominated && !is_static) {
                navsight::eventCounters().global_t_gated_rotation_dominated_total
                    .fetch_add(1, std::memory_order_relaxed);
                if (frame_counter_ % 30 == 0) {
                    LOGI("TRANS_GATE: rotation_dominated gyro=%.3f step_speed=%.3f",
                         gyro_norm, imu.getStepInfo().speed_mps);
                }
            }
        } else if (pose_path_ok || looming_speed_ok) {
            double dt_frame = (timestamp_ns - current_prev_ts) * 1e-9;

            // 2026-05-28 — Wire depth-flow speed into the trajectory.
            //
            // Cause of disconnect: until today, two parallel speed paths ran
            //   without talking to each other —
            //   (a) trajectory: disp = scale_fuser_.scale() * |t_vo|, where
            //       scale_fuser_ is the LEGACY ScaleFuser bootstrapped from
            //       PDR (stride-based, user-rejected for non-walk gaits) and
            //       refined by MiDaS affine-fit / Observer-A;
            //   (b) UI speedometer: getFusedSpeedMps() returns
            //       depth_flow_speed_mps_ — the new K-calibrated
            //       depth-weighted recoverPose result plus looming fusion.
            //   Result: speedometer read 11 km/h on a run while the dot
            //   crept at ~3 km/h equivalent because the trajectory was
            //   still on the legacy path.
            //
            // Change: when depth-flow speed is valid (>=0; -1 sentinel
            //   during warm-up before the first updateDepthFlowSpeed),
            //   override disp = df_speed * dt_frame. Heading/direction
            //   are unchanged — only the magnitude. Falls back to the
            //   scale_fuser path on warm-up frames so trajectory drawing
            //   is never blocked.
            // Falsifier: dot's per-second motion matches the speedometer
            //   reading (within EMA lag) on a walk and a run. Legacy
            //   scale_fuser path still computes (kept for fallback) but
            //   no longer drives the trajectory once depth-flow is live.
            double disp = (t_vo.empty() ? 0.0 : appliedScale * cv::norm(t_vo));
            const double df_speed = df_speed_gate;  // same atomic, same frame (loaded above)
            bool used_depth_flow = false;
            if (df_speed >= 0.0 && dt_frame > 0.0) {
                disp = df_speed * dt_frame;
                used_depth_flow = true;
            }
            // 2026-05-30 Fix B falsifier: count frames where the dot advanced on
            // the looming speed WITHOUT a trusted essential-matrix pose — the
            // forward-motion-degenerate frames this fix rescues from freezing.
            if (used_depth_flow && !pose_path_ok) {
                navsight::eventCounters().global_t_advanced_via_depthflow_fallback_total
                    .fetch_add(1, std::memory_order_relaxed);
                if (frame_counter_ % 30 == 0) {
                    LOGI("TRANS_FALLBACK: dot advanced on looming df_speed=%.2f "
                         "dt=%.3f disp=%.3f pose_valid=%d (essential-matrix degenerate)",
                         df_speed, dt_frame, disp, pose_valid ? 1 : 0);
                }
            }

            // Sanity cap: max displacement per frame.
            // Pre-2026-05-28 cap was 2.0 m/s (walking), which truncated
            // running (~3 m/s) and any vehicle locomotion. Raised to 8 m/s
            // (~28.8 km/h) to cover walk/run/scooter/bike. The depth-flow
            // path is itself bounded by the looming EMA + recoverPose
            // robust median, so this cap mostly guards the legacy
            // scale_fuser branch.
            double max_disp = 8.0 * std::max(dt_frame, 0.03);
            if (disp > max_disp) {
                LOGI("DISP_CAP: disp=%.2f capped to %.2f (dt=%.3f) src=%s",
                     disp, max_disp, dt_frame, used_depth_flow ? "df" : "fuser");
                disp = max_disp;
            }

            // 2026-05-28 — publish the speed actually applied to the trajectory
            // so the UI speedometer (getFusedSpeedMps) reflects what the user
            // sees on the map. Source = depth-flow when valid, else legacy fuser.
            // 2026-05-31 (a.1 speed spike fix) — divide by the FLOORED dt (the same 0.03 floor the
            // disp cap above uses), NOT the raw dt_frame. A tiny inter-frame dt was dividing a
            // disp-capped displacement into a 100-600 km/h reading (the scooter speed spikes). With
            // the floor the published speed inherits the disp cap's 8 m/s ceiling. Heading-safe.
            const double dt_pub = std::max(dt_frame, 0.03);
            trajectory_speed_mps_.store(disp / dt_pub, std::memory_order_relaxed);

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

            // 2026-05-17 REVERT — restore v22/v32 heading-projection.
            //
            // Cause of regression: the full-3D rotation block below
            // (SUPERSEDED) used t_vo's DIRECTION (from essential-matrix
            // recoverPose) as ground-truth world-frame translation
            // direction. t_vo direction is noisy per-frame (KLT inlier
            // scatter, near-degenerate geometry on textured surfaces) and
            // each step pointed in a slightly random direction —
            // cumulative path inflated 2.3× (23.49m for a 10m walk) and
            // net displacement undershot to 60% on a 5m out-and-back.
            //
            // Restoration: project the scalar magnitude `disp = |t_vo|·scale`
            // onto world X/Y using gyro-integrated heading (Madgwick),
            // which is low-noise. Direction is FORCED to match heading
            // (walking direction = forward = heading), magnitude comes
            // from VO. This is the form that delivered v32's 1.1%
            // close-loop on the same 5m walk.
            //
            // Trade: lift-up not tracked (dz only from camera-Y projection,
            // which is small for vertical-phone). Acceptable — the scooter
            // 1km @ 5% goal is horizontal.
            //
            // Falsifier: 5m out-and-back should give peak |p| ≈ 5m,
            // final |p| < 0.5m, cum_path ≈ 10-11m. If still bad, suspect
            // is elsewhere (rotation_dominated over-firing, MSCKF gate).
            //
            // Z-up ENU world frame: X=East, Y=North, Z=Up.
            // heading is CW-positive nav (North=0, East=+π/2).
            //   X (East)  = disp · sin(heading)
            //   Y (North) = disp · cos(heading)
            double dx_world = disp * std::sin(heading);   // +X = East
            double dy_world = disp * std::cos(heading);   // +Y = North
            // dz_world: vertical component. t_vo is in OpenCV camera frame
            // (camera Y points DOWN). World Z is UP. Negate camera-Y to
            // recover world-Z. For flat-ground walking dz ≈ 0.
            double dz_world = 0.0;
            // 2026-05-30 Fix B — only trust t_vo for the vertical component when we
            // have a real visual pose; on the looming-only path t_vo may be empty
            // or stale (failed verification), so leave dz=0 (flat-ground walking).
            if (pose_path_ok && !t_vo.empty())
                dz_world = -appliedScale * t_vo.at<double>(1);

            navsight::eventCounters().translation_heading_projection_total
                .fetch_add(1, std::memory_order_relaxed);
            if (frame_counter_ % 30 == 0) {
                LOGI("TRANS_PROJECT: disp=%.3f yaw=%.1f° dx=%.3f dy=%.3f dz=%.3f",
                     disp, heading * 180.0 / M_PI,
                     dx_world, dy_world, dz_world);
            }

            /* SUPERSEDED 2026-05-17 — full-3D rotation regressed walking
               accuracy (close-loop 1.1% → 9.6%, cum_path 2.3× inflated).
               Kept for archival; do not re-enable without first solving
               per-frame t_vo direction noise (e.g. averaging or gating).

            // Original Step 2 (Production Visual Plan) intent: get vertical
            // motion correctly for lift-up detection. Failed in practice
            // because t_vo noise dominated over the signal we wanted.

            cv::Vec3d dp_world_3d(0.0, 0.0, 0.0);
            if (!t_vo.empty() && ekf_.isFullInitialized()) {
                const cv::Matx33d R_bc_mx = ekf_.getExtrinsicsRotation();
                cv::Mat R_bc(3, 3, CV_64F);
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        R_bc.at<double>(r, c) = R_bc_mx(r, c);
                cv::Mat t_body = R_bc.t() * t_vo;
                if (t_body.at<double>(0) < 0.0) {
                    t_body = -t_body;
                }
                cv::Mat R_GtoI_now = ekf_.getRotation();
                cv::Mat t_world_dir = R_GtoI_now.t() * t_body;
                dp_world_3d[0] = appliedScale * t_world_dir.at<double>(0);
                dp_world_3d[1] = appliedScale * t_world_dir.at<double>(1);
                dp_world_3d[2] = appliedScale * t_world_dir.at<double>(2);
            }
            const double dx_world_3d = dp_world_3d[0];
            const double dy_world_3d = dp_world_3d[1];
            const double dz_world_3d = dp_world_3d[2];
            */

            // Tracker-owned position output (see comment at the top of
            // section 9 for why). Also fed to EKF below as a measurement
            // so EKF state stays consistent with what the UI displays.
            global_t_.at<double>(0) += dx_world;
            global_t_.at<double>(1) += dy_world;
            global_t_.at<double>(2) += dz_world;

            // ── Map-as-sensor POSITION leg (2026-05-31) ─────────────────────────────────
            // Bleed the CROSS-TRACK component of the matcher's position error into global_t_ so the
            // VIO trajectory stops drifting off the road (owner: "fix the drifting vio based on the
            // map matcher"). forward = (sin h, cos h); cross-track(right) unit = (cos h, -sin h). We
            // project the pushed error onto cross-track and apply a small, CAPPED fraction — the
            // along-track component is deliberately dropped (that is the speed/scale lever). Same
            // delta-injection channel as loop-closure; gated like the heading leg so a wrong-road lock
            // cannot run away and the mag (absolute ref) wins when it is actively fusing.
            if (map_pos_correction_enabled_ && ekf_.isFullInitialized() && !imu.isMagActivelyFusing()) {
                const int mp_age = frames_since_map_pos_.fetch_add(1, std::memory_order_relaxed);
                if (mp_age < kMapPosMaxAgeFrames) {
                    const double e_east  = map_pos_err_east_.load(std::memory_order_relaxed);
                    const double e_north = map_pos_err_north_.load(std::memory_order_relaxed);
                    const double cux =  std::cos(heading);   // cross-track (right) unit, east comp
                    const double cuy = -std::sin(heading);   // cross-track (right) unit, north comp
                    const double cross = e_east * cux + e_north * cuy;   // signed lateral error (m)
                    if (std::isfinite(cross) && std::abs(cross) >= kMapPosMinErrM &&
                        std::abs(cross) <= kMapPosMaxErrM) {
                        double step = kMapPosSyncStrength * cross;
                        if (step >  kMapPosMaxStepM) step =  kMapPosMaxStepM;
                        if (step < -kMapPosMaxStepM) step = -kMapPosMaxStepM;
                        global_t_.at<double>(0) += step * cux;
                        global_t_.at<double>(1) += step * cuy;
                        // Subtract what we just applied from the stored snapshot so the residual
                        // CONVERGES to zero within the age window (instead of re-applying the SAME full
                        // error every frame, which never converged while above the dead-band). review MED.
                        map_pos_err_east_.store(e_east - step * cux, std::memory_order_relaxed);
                        map_pos_err_north_.store(e_north - step * cuy, std::memory_order_relaxed);
                        navsight::eventCounters().map_position_corrections_total.fetch_add(1, std::memory_order_relaxed);
                        if (frame_counter_ % 30 == 0) {
                            LOGI("MAP_POS_SYNC: cross_err=%+.2fm step=%+.3fm hdg=%.1f° age=%d",
                                 cross, step, heading * 180.0 / M_PI, mp_age);
                        }
                    }
                }
            }

            // Phase 1 Step 3: integrate the per-frame world-frame
            // displacement magnitude into path_since_last_lc_m_ — used by
            // consumeLoopClosureMatchIfReady to derive σ²_p_drift for the
            // chi² variance budget. Reset to 0 on accepted loop closure.
            path_since_last_lc_m_ += std::sqrt(dx_world * dx_world +
                                               dy_world * dy_world +
                                               dz_world * dz_world);

            // Step 4 Phase C: mirror VO relative-pose update into EKFState
            // with refined variance. updateRelativePose constrains
            // (p_current - p_prev_clone) toward the visual delta. Latest clone
            // id is the previous frame (current-frame addClone happens below).
            //
            // σ_t² has three sources:
            //   1. Visual reprojection: ~5% of displacement (RANSAC inliers)
            //   2. Scale uncertainty: scale_fuser_.var() scales with disp²
            //   3. Floor: 1cm to prevent over-tight fusion when disp~0
            // 2026-05-30 Fix B — feed the visual relative-pose to the EKF only on
            // the trusted-pose path; the looming-only fallback has no t_vo, so its
            // σ_scale (∝ ‖t_vo‖) would be ill-defined. The dot still advances above;
            // ekf_.setPosition(global_t_) keeps the EKF in sync regardless.
            if (pose_path_ok && ekf_.isFullInitialized()) {
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
                    // Return value: false = clone not found / not init; no counter needed (already
                    // guarded by isFullInitialized + prev_clone_id >= 0 above).
                    static_cast<void>(ekf_.updateRelativePose(t_world_metric, prev_clone_id, var_t));
                }
            }
        }
        /* 2026-05-28 — PDR step-based fallback DISABLED. Reason: it injects a
           stride-derived translation (≤1 m/s capped) whenever the visual path
           fails (else-if branch fires when pose_valid=false). On today's walk,
           essential-matrix verification failed across the whole recording (inlier
           ratio 0.03-0.19 vs MIN_INLIER_RATIO=0.25 — slow walk + close scene),
           so PDR was the sole trajectory source, the dot moved, but the
           speedometer sat at 0 because depth_flow_speed_mps_ stayed at the -1
           sentinel. User-rejected (2026-05-28): "remove the step based speed,
           comment it out since its legacy now."

           Going forward: the trajectory advances via the visual / depth-flow
           path only. When essential-matrix verification fails, looming now
           still fires (it was moved out of the verification_ok gate above) and
           contributes via depth_flow_speed_mps_ as long as K is seeded. K is
           seeded from SharedPreferences on each cold start. If K is still
           uncalibrated AND essential matrix fails, the trajectory genuinely
           stops — which is honest and a clear signal to fix depth-flow rather
           than papering over with PDR.

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
                if (dt_s > 0.0) {
                    trajectory_speed_mps_.store(d / dt_s, std::memory_order_relaxed);
                }
                // Z-up ENU world: project step displacement onto X (East), Y (North).
                double dx_step = d * std::sin(heading);   // +X = East
                double dy_step = d * std::cos(heading);   // +Y = North

                // Tracker-owned position output (visual-degenerate fallback).
                // Same rationale as the visual path above: EKF position is
                // unreliable as a display source; Tracker integrates here
                // and feeds the EKF as a measurement for state consistency.
                global_t_.at<double>(0) += dx_step;
                global_t_.at<double>(1) += dy_step;

                // Phase 1 Step 3: PDR step magnitude is exactly `d`
                // (computed at line above as a clamped per-frame distance).
                // Z is left unchanged in this fallback path, so the step
                // magnitude is the full path increment.
                path_since_last_lc_m_ += d;

                // Step 4 Phase C: mirror PDR fallback step into EKFState
                // with refined per-step variance.
                // σ ≈ 5cm floor + 10% of step distance:
                //   dt at 30 Hz ⇒ d ~3-7cm/frame, σ ~5-6cm, var ~3-4e-3
                //   dt at 1 Hz  ⇒ d ~30-70cm,    σ ~8-12cm, var ~1-2e-2
                if (ekf_.isFullInitialized()) {
                    double sigma_step = 0.05 + 0.10 * d;
                    // Return value: false = not init (guarded by isFullInitialized above).
                    static_cast<void>(ekf_.updatePDRStep(dx_step, dy_step, sigma_step * sigma_step));
                }
            }
        }
        */

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

        // 2026-05-09 telemetry fix — bridge SLAM features dropped via
        // EKFState::marginalizeOldestCloneNoLock back into FeatureManager
        // so the slam_lifetime_count histogram counts EVERY demotion,
        // not just RMS/chi² rejections. Without this bridge the EKF
        // erases an entry from slam_features_ but FeatureManager keeps
        // the lifecycle entry forever (slam_slot stays >= 0 in its map),
        // so promoted_age - age never gets accumulated and the per-walk
        // mean SLAM lifetime metric stays NaN.
        for (int dropped_fid : ekf_.takeLastMarginalizedSlamFeatureIds()) {
            feature_mgr_.setSlamSlot(dropped_fid, -1);
            feature_mgr_.dropLifecycle(dropped_fid);
        }

        // Record MSCKF observations: feature pixels in normalized UNDISTORTED
        // coordinates.
        //
        // CRITICAL (v23.4 fix, 2026-05-11): pre-2026-05-11 this code did
        //     nrm = ((next_good_buf_[i].x - cx) / fx, (.y - cy) / fy)
        // without ever undistorting, then stored the result in
        // FeatureObservation::pixel_ud — a misleading name because "ud"
        // was never undistorted. Downstream consumers (triangulation at
        // ~line 2175 using `(obs.x, obs.y, 1)` as a camera-frame RAY, MSCKF
        // residuals in UpdaterMSCKF.cpp:112) treated these as TRUE rays
        // through the principal point, producing world points biased by
        // the camera's lens distortion. KLT-rendered dots looked fine
        // because they're drawn at raw pixel coords with no EKF round-trip;
        // SLAM-rendered dots (reprojected through the EKF state) drifted
        // because the underlying world point was off by the distortion
        // amount. Symptom: v23.x had SLAM dots drift visibly even when
        // phone was still; KLT dots stayed pinned. Fix: undistort
        // next_good_buf_ before the (u-cx)/fx normalization so the stored
        // pixel_ud is the actual normalised TRUE ray direction.
        if (clone_id >= 0 && !next_good_buf_.empty()) {
            // Undistort the entire batch once, then normalise.
            std::vector<cv::Point2f> next_obs_undist = next_good_buf_;
            if (lens_.isReady() && lens_.hasDistortion()) {
                lens_.undistortPoints(next_obs_undist);
            }
            for (size_t i = 0; i < next_obs_undist.size() && i < feature_ids_.size(); i++) {
                if (feature_ids_[i] >= 0) {
                    // Convert to normalized coordinates for MSCKF / triangulation.
                    cv::Point2f nrm(
                        (next_obs_undist[i].x - cx_use) / fx_use,
                        (next_obs_undist[i].y - cy_use) / fy_use);
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
                    // 2026-05-16 root-cause gate for p_G stationary drift
                    // (EKF-update audit Q3): processLostFeatures was firing
                    // 56×/sec during 9.7s stationary in v35, injecting ~1mm
                    // δp per call from KLT subpixel noise → 40cm accumulated
                    // drift while phone was completely still. Each MSCKF
                    // residual during stationary is geometrically zero-signal
                    // — the camera isn't moving so the visual evidence carries
                    // no information about EKF position. Gate eliminates the
                    // leak. pruneObservations below stays UNGATED (window
                    // accounting must remain consistent).
                    if (is_static) {
                        navsight::eventCounters().msckf_gated_static_total.fetch_add(
                            1, std::memory_order_relaxed);
                    } else {
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

            // Build current-observation lookup: feature_id → undistorted pixel.
            //
            // CRITICAL: undistort BEFORE storing. slamReprojectionJacobian
            // predicts pixels via a pure pinhole model (slam_fx_ * Xc/Zc +
            // slam_cx_) with NO distortion correction. The KLT pixels in
            // next_good_buf_ come straight from the distorted raw frame.
            // Passing distorted observations to the EKF means the residual
            // includes a built-in radial-distortion bias of several pixels
            // (worst at image corners). That biased residual is what fired
            // the 3-frame bad-RMS streak gate within ~135 ms of promotion —
            // visible v23.1: 112 demotions; v23.2: 189. Pre-promotion path
            // already uses pixel_ud (undistorted) via FeatureManager;
            // matching here closes the unit-mismatch.
            std::vector<cv::Point2f> next_undist = next_good_buf_;
            if (lens_.isReady() && lens_.hasDistortion()) {
                lens_.undistortPoints(next_undist);
            }
            std::unordered_map<int, cv::Point2f> cur_obs;
            cur_obs.reserve(feature_ids_.size());
            for (size_t i = 0; i < feature_ids_.size() &&
                               i < next_undist.size(); i++) {
                int fid = feature_ids_[i];
                if (fid < 0) continue;
                cur_obs[fid] = next_undist[i];  // undistorted pixel coords
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
            // 2026-05-12: pass the current EKF window's clone IDs so the
            // gate can verify that promotable candidates have ≥2 live
            // in-window observations (the prereq for the triangulation
            // loop below).
            std::vector<int> window_ids;
            window_ids.reserve(EKFState::MAX_CLONES);
            for (const auto& cp : ekf_.getWindow()) {
                window_ids.push_back(cp.state_id);
            }
            auto promotable = feature_mgr_.getPromotableFeatures(
                window_ids,
                /*min_obs=*/8, /*min_kf=*/2, /*max_init_rms_px=*/1.5);
            // 2026-05-20 promotion-gate instrumentation. Each `continue`
            // increments a distinct counter so we can see which gate kills
            // SLAM promotions in replay (slam_promotions_total stays 0 in
            // the desktop harness despite landmarks_added_total > 0 and
            // MSCKF firing). Per the implementor-skill rule: counters at
            // every decision point.
            navsight::eventCounters().slam_promo_candidates_total.fetch_add(
                static_cast<long long>(promotable.size()),
                std::memory_order_relaxed);
            for (int fid : promotable) {
                if (ekf_.getSlamFeatureCount() >= EKFState::MAX_SLAM_FEATURES) {
                    navsight::eventCounters().slam_promo_rejected_cap_total
                        .fetch_add(1, std::memory_order_relaxed);
                    break;  // cap reached this frame; try again next frame
                }
                const auto* obs = feature_mgr_.getObservations(fid);
                if (!obs || obs->size() < 2) {
                    navsight::eventCounters().slam_promo_rejected_no_obs_total
                        .fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
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
                    navsight::eventCounters().slam_promo_rejected_no_anchor_far_total
                        .fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                // v23.10 (2026-05-11): loosened promotion baseline gate
                // from 5 cm → 1.5 cm. With 5 cm features rarely promoted
                // even during walking (hand-stabilised phone, slow walk,
                // 8-obs window ~0.6 s). The parallax UPDATE gate at 3 cm
                // in EKFState::updateSlamFeature still prevents post-
                // promotion depth runaway; the promotion gate just needs
                // to filter "literally no parallax" (mm-scale) cases.
                {
                    const double promo_baseline = cv::norm(p_far - p_anchor);
                    if (promo_baseline < 0.015) {
                        navsight::eventCounters().slam_promo_rejected_baseline_total
                            .fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }
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

                // 2026-05-20 PROMOTION PARALLAX GATE — mirrors Fix #10
                // (EKFState::buildSlamLiveJacobianRow) and the visual front-
                // end parallax gate at Tracker.cpp:2162-2225.
                //
                // Cause: the only pre-triangulation geometry filter before
                // this point is the baseline gate at line 3349
                // (`cv::norm(p_far - p_anchor) < 0.015 → reject`). Baseline
                // ≠ parallax. Two cameras 5 cm apart looking at a wall
                // 1.5 m away with NEARLY-PARALLEL rays produce midpoint
                // triangulation depth ≈ baseline / sin(parallax_angle).
                // parallax = 0.01° → depth ≈ 286 m, parallax = 0.1° →
                // depth ≈ 28.6 m. The reprojection-RMS gate at line 3414
                // doesn't catch these because RMS is in PIXEL space: when
                // all observations of the feature are from low-parallax
                // viewpoints, projecting the wrong-depth p_world through
                // any of them gives ~the same pixel as the observation
                // (the bad p_world sits ALONG the ray, not perpendicular
                // to it). RMS gate is necessary-but-not-sufficient.
                //
                // Empirical evidence: parallax_fix_walk_2026_05_20 had
                // 12 of 24 SLAM promotions get MiDaS-seeded — 6 of those
                // at z_tri = 403-680 m with z_midas = 1.4-1.6 m (250-450×
                // depth error) yet they all passed baseline+chirality+RMS.
                //
                // Change: compute cos(parallax) = dAw_n · dBw_n. Below
                // threshold (cos > 0.99985 ⇔ angle < ~1°), skip promotion.
                // The feature stays in the candidate pool and will be
                // re-attempted as more observations accumulate; eventually
                // parallax grows (user moves laterally past the wall) and
                // promotion succeeds with a sane depth.
                //
                // Threshold derivation: kSlamPromoMinParallaxCos = 0.99985
                // (≈ 1.0°). Matches ORB-SLAM3's triangulation-promotion
                // threshold (Mur-Artal & Tardós 2017 §V.B "Triangulating
                // Map Points": parallax ≥ 1°). Slightly looser than
                // Fix #10's 0.99995 (0.57°) because promotion is one-shot
                // and tolerates more uncertainty than per-frame depth
                // refinement. Same family of constants as Fix #10 and the
                // front-end parallax gate (kVisualMinParallaxRad = 0.01).
                //
                // Falsifier: post-fix walk →
                //   slam_promo_rejected_parallax_total > 0 (gate firing)
                //   SLAM_PROMOTE_MIDAS_SEED log shows no z_tri > 50 m
                //   (the kMidasReplaceRatio = 2.0 gate should still fire
                //   occasionally on borderline cases, but never on the
                //   100s-of-metres outliers we saw)
                //   slam_promotions_seeded_with_midas / slam_promotions_total
                //   ratio drops from 0.50 (today) to < 0.10.
                constexpr double kSlamPromoMinParallaxCos = 0.99985;
                const double cos_parallax =
                    dAw_n.dot(dBw_n) /
                    (cv::norm(dAw_n) * cv::norm(dBw_n));
                if (std::isfinite(cos_parallax) &&
                    cos_parallax > kSlamPromoMinParallaxCos) {
                    navsight::eventCounters()
                        .slam_promo_rejected_parallax_total
                        .fetch_add(1, std::memory_order_relaxed);
                    continue;
                }

                cv::Mat M(3, 2, CV_64F);
                dAw_n.copyTo(M(cv::Range::all(), cv::Range(0, 1)));
                cv::Mat negB = -dBw_n;
                negB.copyTo(M(cv::Range::all(), cv::Range(1, 2)));
                cv::Mat rhs = p_far - p_anchor;
                cv::Mat ts;
                if (!cv::solve(M, rhs, ts, cv::DECOMP_SVD)) {
                    navsight::eventCounters().slam_promo_rejected_solve_total
                        .fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                const double tA = ts.at<double>(0, 0);
                const double tB = ts.at<double>(1, 0);
                if (tA <= 0.05 || tB <= 0.05) {
                    navsight::eventCounters().slam_promo_rejected_chirality_total
                        .fetch_add(1, std::memory_order_relaxed);
                    continue;  // chirality gate
                }
                cv::Mat p_world =
                    0.5 * ((p_anchor + dAw_n * tA) + (p_far + dBw_n * tB));

                // Reprojection RMSE over ALL surviving observations. Lambda so we
                // can RE-SCORE after a MiDaS depth re-seed (Hidden Bug #3). Returns
                // the 1e9 behind-camera sentinel directly so the stats below can
                // exclude it (the prior inline path let sqrt(1e9/2)=22360px pollute
                // the max/mean).
                auto reprojRms = [&](const cv::Mat& pw, int& n_out) -> double {
                    double s2 = 0.0; int n = 0;
                    for (const auto& o : *obs) {
                        cv::Mat R, p;
                        if (!ekf_.getClonePose(o.clone_state_id, R, p)) continue;
                        cv::Mat p_C = R * (pw - p);
                        const double zC = p_C.at<double>(2, 0);
                        if (zC < 0.05) { n_out = n; return 1e9; }
                        const double u = p_C.at<double>(0, 0) / zC;
                        const double v = p_C.at<double>(1, 0) / zC;
                        const double du = (u - o.pixel_ud.x) * fx_use;
                        const double dv = (v - o.pixel_ud.y) * fy_use;
                        s2 += du * du + dv * dv;
                        ++n;
                    }
                    n_out = n;
                    return (n >= 1) ? std::sqrt(s2 / static_cast<double>(n)) : 1e9;
                };

                int n_used = 0;
                double rms = reprojRms(p_world, n_used);
                if (n_used < 2) {
                    navsight::eventCounters().slam_promo_rejected_n_used_total
                        .fetch_add(1, std::memory_order_relaxed);
                    continue;
                }

                // 2026-05-26 Hidden Bug #3 — MiDaS depth RESCUE before the RMS gate.
                // Monocular triangulation is baseline-starved on indoor/axial walks;
                // the existing MiDaS re-seed (below) ran only AFTER this gate, so a
                // degenerate point was already rejected. When the two-view solve is
                // degenerate (rms>1.5), re-seed p_world from baseline-independent
                // MiDaS metric depth at the anchor pixel and RE-SCORE vs ALL clones.
                // Promote only if it passes the SAME 1.5px gate (NO loosening — a
                // wrong MiDaS depth still fails the re-score and is rejected, per the
                // chi² lesson). Scope: this rescues RMS-rejected (has-baseline-but-
                // degenerate) candidates; the dominant baseline-gate rejects (~0 cm
                // translation, line 3469) are a separate riskier follow-up because
                // reprojection can't validate depth at near-zero baseline.
                bool midas_rescued = false;
                if (rms > 1.5) {
                    const float u_img = static_cast<float>(obs_anchor.x * fx_use + cx_use);
                    const float v_img = static_cast<float>(obs_anchor.y * fy_use + cy_use);
                    double z_midas = 0.0;
                    const bool sample_ok =
                        sampleMidasMetricDepth(u_img, v_img, z_midas) &&
                        std::isfinite(z_midas) && z_midas > 0.05;
                    if (sample_ok) {
                        cv::Mat p_cam_midas = (cv::Mat_<double>(3, 1) <<
                            obs_anchor.x * z_midas, obs_anchor.y * z_midas, z_midas);
                        cv::Mat p_world_midas = R_anchor.t() * p_cam_midas + p_anchor;
                        int n_midas = 0;
                        const double rms_midas = reprojRms(p_world_midas, n_midas);
                        if (n_midas >= 2 && rms_midas <= 1.5) {
                            LOGI("SLAM_PROMOTE_MIDAS_RESCUE: fid=%d z_midas=%.2fm "
                                 "rms_tri=%.1fpx rms_midas=%.2fpx", fid, z_midas,
                                 rms, rms_midas);
                            p_world = p_world_midas;
                            rms = rms_midas;
                            midas_rescued = true;
                            navsight::eventCounters().slam_promotions_seeded_with_midas
                                .fetch_add(1, std::memory_order_relaxed);
                        } else {
                            // 2026-05-31 DIAG (verdict §3, failure mode (b)) — depth
                            // sampled OK but the MiDaS-seeded point still re-projects
                            // over the 1.5px gate (or < 2 clones survived). The depth
                            // disagrees with the multi-clone geometry; the gate is
                            // correctly rejecting it (NO loosening). Throttled: this
                            // runs per-candidate (~24k/walk) — emit ~every 90 frames,
                            // matching the per-candidate-loop throttle convention.
                            navsight::eventCounters().slam_promo_midas_rescore_failed
                                .fetch_add(1, std::memory_order_relaxed);
                            if ((frame_counter_ % 90) == 0) {
                                LOGI("SLAM_PROMOTE_MIDAS_DIAG: rescore_fail "
                                     "z_midas=%.2f rms_midas=%.1fpx n=%d",
                                     z_midas, rms_midas, n_midas);
                            }
                        }
                    } else {
                        // 2026-05-31 DIAG (verdict §3, failure mode (a)) —
                        // sampleMidasMetricDepth returned false OR z_midas was
                        // <= 0.05 / non-finite, so the rescue could not even
                        // attempt. If this dominates the device walk, the MiDaS
                        // sampler (depth map / pixel mapping / range) is the
                        // bottleneck, not the triangulation geometry. Same 90-frame
                        // throttle as above.
                        navsight::eventCounters().slam_promo_midas_sample_failed
                            .fetch_add(1, std::memory_order_relaxed);
                        if ((frame_counter_ % 90) == 0) {
                            LOGI("SLAM_PROMOTE_MIDAS_DIAG: sample_fail "
                                 "u=%.0f v=%.0f z=%.3f", u_img, v_img, z_midas);
                        }
                    }
                }

                if (rms > 1.5) {
                    auto& ecr = navsight::eventCounters();
                    ecr.slam_promo_rejected_rms_total
                        .fetch_add(1, std::memory_order_relaxed);
                    const long long rms_mi =
                        static_cast<long long>(std::lround(rms * 1000.0));
                    ecr.slam_promo_rms_milli_p95
                        .store(rms_mi, std::memory_order_relaxed);  // legacy: latest sample
                    // 2026-05-26 Hidden Bug #3 — distribution sum (-> mean) + max,
                    // EXCLUDING the 1e9 behind-camera sentinel so the stats reflect
                    // REAL reprojection error (prior max=22360px was sqrt(1e9/2)).
                    if (rms < 1e6) {
                        ecr.slam_promo_rms_sum_milli
                            .fetch_add(rms_mi, std::memory_order_relaxed);
                        long long prev_max =
                            ecr.slam_promo_rms_milli_max.load(std::memory_order_relaxed);
                        while (rms_mi > prev_max &&
                               !ecr.slam_promo_rms_milli_max.compare_exchange_weak(
                                   prev_max, rms_mi, std::memory_order_relaxed)) {}
                    }
                    continue;
                }

                // 2026-05-19 Fix #12 — MiDaS sanity check on the triangulated
                // depth. Pure-axial motion (walk-back/forward facing scene,
                // scooter-straight) gives degenerate triangulation; the
                // result can be off by >2× even when the reprojection-RMS
                // gate passes (RMS is in image space, not depth space, so
                // a 5m-vs-15m world point projects nearly the same pixel
                // for a stationary camera). Cross-check with MiDaS metric
                // depth: if MiDaS available AND disagrees with triangulated
                // depth in the anchor camera frame by > kMidasReplaceRatio,
                // replace p_world with MiDaS-derived position. The MiDaS
                // depth is independent of motion baseline, so it's the
                // trustworthy source in axial-motion cases.
                //
                // Math: at anchor pixel (obs_anchor in normalised
                // coords) and MiDaS depth z_midas, the point in anchor
                // camera frame is (obs_anchor.x · z, obs_anchor.y · z, z).
                // World point = R_anchor.t() · cam + p_anchor (R_anchor is
                // world→camera per EKFState convention; transpose gives
                // camera→world).
                //
                // Threshold derivation: 2× ratio matches the
                // applyDepthScaleConstraint inlier band (lines 350,
                // ratio ∈ [0.1, 10] is the absolute trust window; > 2×
                // disagreement is the "geometry suspect, prefer MiDaS"
                // band). Picked conservatively to avoid replacing healthy
                // triangulations.
                // 2026-05-26 — run the MiDaS sanity-REPLACE only when we kept the
                // triangulated point; if already MiDaS-rescued above, skip it.
                constexpr double kMidasReplaceRatio = 2.0;
                if (!midas_rescued) {
                    cv::Mat p_anchor_cam_mat = R_anchor * (p_world - p_anchor);
                    const double z_tri = p_anchor_cam_mat.at<double>(2, 0);
                    if (std::isfinite(z_tri) && z_tri > 0.05) {
                        // Convert normalised obs_anchor to image pixel for
                        // the MiDaS sample. fx_use / fy_use / cx_use /
                        // cy_use are this frame's effective intrinsics
                        // (post user-scale correction). Undistortion already
                        // applied at addObservation time, so this is the
                        // pinhole pixel; MiDaS distortion error is small
                        // (< 5 px at frame edges with NavSight's k1=0.263).
                        const float u_img = static_cast<float>(
                            obs_anchor.x * fx_use + cx_use);
                        const float v_img = static_cast<float>(
                            obs_anchor.y * fy_use + cy_use);
                        double z_midas = 0.0;
                        if (sampleMidasMetricDepth(u_img, v_img, z_midas)) {
                            const double ratio = z_midas / z_tri;
                            if (ratio > kMidasReplaceRatio ||
                                ratio < 1.0 / kMidasReplaceRatio) {
                                // Triangulation disagrees with MiDaS — prefer
                                // MiDaS. Reconstruct p_world from anchor pose
                                // + obs ray + z_midas.
                                cv::Mat p_cam_midas = (cv::Mat_<double>(3, 1) <<
                                    obs_anchor.x * z_midas,
                                    obs_anchor.y * z_midas,
                                    z_midas);
                                cv::Mat p_world_midas =
                                    R_anchor.t() * p_cam_midas + p_anchor;
                                LOGI("SLAM_PROMOTE_MIDAS_SEED: fid=%d "
                                     "z_tri=%.2fm z_midas=%.2fm ratio=%.2f "
                                     "p_tri=(%.2f,%.2f,%.2f) p_midas=(%.2f,%.2f,%.2f)",
                                     fid, z_tri, z_midas, ratio,
                                     p_world.at<double>(0, 0),
                                     p_world.at<double>(1, 0),
                                     p_world.at<double>(2, 0),
                                     p_world_midas.at<double>(0, 0),
                                     p_world_midas.at<double>(1, 0),
                                     p_world_midas.at<double>(2, 0));
                                p_world = p_world_midas;
                                navsight::eventCounters()
                                    .slam_promotions_seeded_with_midas
                                    .fetch_add(1, std::memory_order_relaxed);
                            }
                        }
                    }
                }

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
                    // 2026-05-12: VisualMap addOrUpdate call reverted. The
                    // persistent landmark map belongs to Phase 1 Step 6
                    // (proper LandmarkMap with KD-tree + descriptors + per-
                    // frame ORB matching + MSCKF updates), not a one-shot
                    // rendering cache. Pick this back up after Step 5.
                    // visual_map_.addOrUpdate(fid, p_world, anchor_clone_id,
                    //                         timestamp_ns);
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
                // v23.12: stash current KLT obs every frame so the visual
                // debug overlay's cyan crosshair tracks where the feature
                // ACTUALLY is right now (not stale from last parallax-gated
                // SLAM update).
                feature_mgr_.noteSlamLastObs(
                    slot_fid, cur_it->second.x, cur_it->second.y);
                std::vector<cv::Point2f> obs1{cur_it->second};
                std::vector<int> ids1{latest_clone_id};
                n_slam_updates_ran++;
                // 2026-05-16: same stationarity gate as MSCKF processLostFeatures.
                // SLAM reprojection residuals during stationary are KLT subpixel
                // noise injecting δp drift into p_G. Skip when is_static.
                if (is_static) {
                    navsight::eventCounters().slam_update_gated_static_total.fetch_add(
                        1, std::memory_order_relaxed);
                    continue;
                }
                // 2026-05-12: route SLAM update through UpdaterSLAM (parallax
                // gate, depth-observability gate, per-obs chi²) instead of
                // the older EKFState::updateSlamFeature (per-stack chi² only).
                if (slam_updater_.update_skf(
                        ekf_, slot, obs1, ids1,
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
                            double du = 0, dv = 0, u_px = 0, v_px = 0;
                            if (zC > 0.05) {
                                u_px = fx_use * p_C.at<double>(0, 0) / zC + cx_use;
                                v_px = fy_use * p_C.at<double>(1, 0) / zC + cy_use;
                                du = u_px - cur_it->second.x;
                                dv = v_px - cur_it->second.y;
                                rms_px = std::sqrt(du * du + dv * dv);
                            }
                            // v23.5 DIAGNOSTIC: log per-feature RMS so we can
                            // see actual residual magnitudes vs the 3 px gate.
                            // Per sim-debugging cardinal rule: read the data
                            // before tuning. Throttle to once per feature per
                            // 5 frames (slot is stable; fid changes).
                            LOGI("SLAM_RMS fid=%d slot=%d rms=%.2fpx "
                                 "obs=(%.1f,%.1f) pred=(%.1f,%.1f) "
                                 "du=%.2f dv=%.2f zC=%.2f "
                                 "p_world=(%.3f,%.3f,%.3f)",
                                 slot_fid, slot, rms_px,
                                 cur_it->second.x, cur_it->second.y,
                                 u_px, v_px, du, dv, zC,
                                 p_world.at<double>(0,0),
                                 p_world.at<double>(1,0),
                                 p_world.at<double>(2,0));
                            // v23.11: also stash the obs pixel for the visual
                            // debug overlay's residual line.
                            feature_mgr_.markSlamFeatureRMS(
                                slot_fid, rms_px,
                                cur_it->second.x, cur_it->second.y, true);
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

                            // 2026-05-21 — tightened 20° → 3° per Bug 4 root-cause.
                            //
                            // Cause: 20° gate accepted nearly every visual-yaw
                            // measurement (logged drift p99 ≈ 10°, never >20°).
                            // But residuals 3-10° produce 30-78°/s EKF R_GtoI
                            // jumps because the Kalman update applies them at
                            // gain ≈ 0.75 against var_yaw_floor=(0.57°)².
                            //
                            // Threshold derivation (sensor-physics, not stats):
                            //   Madgwick yaw-rate p99 (measured)  = 1.87°/s
                            //   Keyframe interval                  = 0.5 s
                            //   Real body rotation per KF (3σ)     = 0.94°
                            //   Gyro bias drift per KF (×0.5s)     = 0.14°
                            //   Sum (worst-case legit disagreement) ≈ 1.1°
                            //   Safety factor 2.7×                 → 3°
                            //
                            // Anything above 3° is recoverPose returning a
                            // sign-flipped or otherwise wildly wrong R from
                            // E-matrix decomposition (known-unstable on
                            // planar / low-parallax scenes even with high
                            // inlier counts). Gyro is more trustworthy on
                            // the 0.5s timescale than visual.
                            //
                            // Falsifier: post-fix walk → EKF R_GtoI yaw-rate
                            // p99 drops from 78°/s to < 10°/s; trajectory
                            // loop-bearing delta < 5°.
                            //
                            // LEGACY: was 20° before this fix.
                            // /* legacy: if (std::abs(drift) < 20.0 * M_PI / 180.0) */
                            constexpr double kKfHeadingDriftMaxRad = 3.0 * M_PI / 180.0;
                            if (std::abs(drift) < kKfHeadingDriftMaxRad) {
                                // Step 4 Phase B: feed visual yaw measurement into
                                // EKFState as the single yaw-correction path.
                                // The Joseph-form Kalman update applies its own
                                // gain — the legacy 30%-blend below has been
                                // removed (see KF_HEADING_CORR log: "EKF-only").
                                if (aligned_ok && ekf_.isFullInitialized()) {
                                    // 2026-05-16 root-cause gate — match the
                                    // updateRelativeRotation gate at line 1741.
                                    //
                                    // Cause (v33 walk diagnostic, stationary
                                    // camera + slight rotation): EKF R_GtoI yaw
                                    // drifted +0.73°/sec while Madgwick drifted
                                    // -0.55°/sec — a +1.28°/sec divergence.
                                    // The visual yaw measurement that feeds
                                    // updateGravityAlignedYaw is geometrically
                                    // degenerate under monocular pure rotation:
                                    // essential matrix decomposition can't
                                    // separate small translation from rotation
                                    // when |t|→0, and KLT subpixel noise + hand
                                    // tremor get factored into the "rotation"
                                    // component → systematic ~1-2°/sec false
                                    // yaw bias injected at every keyframe.
                                    //
                                    // Change: same translation_degenerate /
                                    // is_pure_rotation guards already gating
                                    // updateRelativeRotation (Tracker.cpp:1741)
                                    // — visual yaw correction skipped when
                                    // motion isn't translation-rich enough to
                                    // disambiguate. Trust gyro/Madgwick during
                                    // pure-rotation periods.
                                    //
                                    // Falsifier: rotate-only walk → EKF R_GtoI
                                    // yaw should drift only at the Madgwick
                                    // rate (~0.55°/sec residual gyro bias),
                                    // not at the current +0.73°/sec divergent.
                                    // visual_yaw_gated_pure_rotation_total > 0
                                    // confirms the gate is firing.
                                    // 2026-05-16 widened gate (v34 data showed
                                    // pure-rotation flag too strict — user's
                                    // small left-right rotations don't trip the
                                    // Rayleigh-test). Add `is_static` so that
                                    // when ZUPT detector reports stationary,
                                    // we ALSO skip visual yaw correction.
                                    // is_static fires correctly during true
                                    // stationary periods (data in v31/v33).
                                    if (is_static || translation_degenerate || is_pure_rotation) {
                                        navsight::eventCounters()
                                            .visual_yaw_gated_pure_rotation_total
                                            .fetch_add(1, std::memory_order_relaxed);
                                        if (frame_counter_ % 30 == 0) {
                                            LOGI("VISUAL_YAW_GATE: skipped (is_static=%d translation_degenerate=%d is_pure_rotation=%d)",
                                                 is_static ? 1 : 0,
                                                 translation_degenerate ? 1 : 0,
                                                 is_pure_rotation ? 1 : 0);
                                        }
                                    } else {
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
                                    // Return value: false = not init / chi² reject / singular.
                                    // No separate counter — rejection rate visible via keyframe-accept ratio.
                                    static_cast<void>(ekf_.updateGravityAlignedYaw(
                                        yaw_meas, var_yaw,
                                        current_roll, current_pitch));
                                    }  // close inner else (is_static gate)

                                    // 2026-05-21 BUG-02b ROOT-CAUSE FIX — Bug 5
                                    // moved OUTSIDE the is_static/translation_degenerate
                                    // /is_pure_rotation gate so it actually fires.
                                    //
                                    // Cause: pre-fix, Bug 5 was nested inside the
                                    // `else { }` block (line 4140-4235) gated by
                                    // `is_static || translation_degenerate ||
                                    // is_pure_rotation`. Hive-worker-05 cross-walk
                                    // analysis showed `madgwick_visual_yaw_nudges_total
                                    // = 0` across all 6 walks 2026-05-19 to 2026-05-21
                                    // despite `updateGravityAlignedYaw` (LC_YAW_FIRE)
                                    // firing 8-16× per walk. Hive-worker-02 confirmed
                                    // independently: 67% of `PARALLAX_GATE` logs show
                                    // `parallax_rad < 0.01` → translation_degenerate
                                    // skips both updates. Bug 5 was structurally
                                    // unreachable in the typical walking scene
                                    // (forward, low parallax against walls).
                                    //
                                    // Change: hoist Bug 5 out of the inner else.
                                    // Still inside the outer `if (drift < 3°)` gate
                                    // so we trust the visual yaw measurement (gyro-
                                    // consistency proven), and still inside
                                    // `if (aligned_ok && ekf_.isFullInitialized())`
                                    // for safety. Recompute yaw_meas locally so we
                                    // don't depend on the inner else's scope.
                                    //
                                    // Why is_static/translation_degenerate are NOT
                                    // valid gates for Bug 5 specifically:
                                    //   - is_static: Madgwick yaw drifts on bias
                                    //     even when user is stationary; visual yaw
                                    //     during is_static has zero motion-noise.
                                    //     Pushing visual → Madgwick is HELPFUL.
                                    //   - translation_degenerate: gyro-consistency
                                    //     gate (drift < 3°) already ensures visual
                                    //     agrees with gyro within 3°, so visual yaw
                                    //     is trustworthy for Madgwick nudging.
                                    //   - is_pure_rotation: visual yaw IS observable
                                    //     during pure rotation (essential matrix
                                    //     decomposes consistently for pure rotation),
                                    //     so Bug 5 nudge is sound here too.
                                    //
                                    // Falsifier (post-BUG-02b walk):
                                    //   madgwick_visual_yaw_nudges_total > 100 per
                                    //                                           walk
                                    //   Debug-panel heading drift across 2 loops < 5°
                                    constexpr double kBug5SyncStrength    = 0.10;
                                    constexpr double kBug5MaxResidualRad  = M_PI / 4.0;
                                    double yaw_meas_b5 = kf_heading + visual_delta_heading;
                                    while (yaw_meas_b5 >  M_PI) yaw_meas_b5 -= 2.0 * M_PI;
                                    while (yaw_meas_b5 < -M_PI) yaw_meas_b5 += 2.0 * M_PI;
                                    const double madg_now = imu.getHeading();
                                    double yaw_resid_madg = yaw_meas_b5 - madg_now;
                                    while (yaw_resid_madg > M_PI)
                                        yaw_resid_madg -= 2.0 * M_PI;
                                    while (yaw_resid_madg < -M_PI)
                                        yaw_resid_madg += 2.0 * M_PI;
                                    // 2026-05-25 mag-primary: skip the visual yaw
                                    // nudge while the magnetometer is actively
                                    // fusing (clean field). The mag is the
                                    // absolute heading reference; the visual
                                    // estimate disagreed with gyro 253x on the
                                    // 2026-05-25 walk, so it must NOT override the
                                    // mag. Stays active as a fallback when the mag
                                    // is unavailable (dirty field / indoors).
                                    // 2026-06-02 (owner decision): the VISUAL yaw nudge is DISABLED. The
                                    // displayed heading must be ROAD + GYRO + MAGNETOMETER only — NEVER the
                                    // essential-matrix-ambiguous visual yaw. On a degenerate scene (a flat
                                    // video / dirty-mag indoors, where this nudge used to fire because the
                                    // mag isn't fusing) the visual yaw leaked into the Madgwick yaw that the
                                    // rail's gyro-relative steering reads (lastVioHeadingDeg) → spurious
                                    // "rotation" → false U-turn 180° flips. The road-bearing nudge (§9.0b)
                                    // and the magnetometer are the heading sources now. The application is
                                    // commented (kept per the no-delete rule) — we still LOG what it WOULD
                                    // have done so the visual/gyro disagreement stays observable.
                                    if (std::isfinite(yaw_resid_madg) &&
                                        std::abs(yaw_resid_madg) < kBug5MaxResidualRad &&
                                        !imu.isMagActivelyFusing()) {
                                        /* DISABLED 2026-06-02 — visual must not touch the heading:
                                        imu.nudgeMadgwickYawAroundWorldZ(
                                            kBug5SyncStrength * yaw_resid_madg);
                                        navsight::eventCounters()
                                            .madgwick_visual_yaw_nudges_total
                                            .fetch_add(1, std::memory_order_relaxed); */
                                        if (frame_counter_ % 30 == 0) {
                                            LOGI("MADG_VISUAL_SYNC[DISABLED]: would_resid=%+.2f° "
                                                 "would_apply=%+.2f° madg=%+.2f° (visual no longer affects heading)",
                                                 yaw_resid_madg * 180.0 / M_PI,
                                                 kBug5SyncStrength * yaw_resid_madg *
                                                     180.0 / M_PI,
                                                 madg_now * 180.0 / M_PI);
                                        }
                                    }
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
        const bool kf_cond_A = (frames_since_keyframe_ >= 15);
        const bool kf_cond_B = (tracked < MIN_FEATURES / 2 && frames_since_keyframe_ > 3);
        if (kf_cond_A || kf_cond_B) {
            stored_keyframe_this_frame = true;
            // 2026-05-19 KF_RATE diag: distinguish condition A (15-frame cadence)
            // vs condition B (low-tracked early storage). v53 had 2× the KF rate
            // of v50 (362 vs 179) which dilutes BoW DB and tanks LC scores.
            // If kf_cond_B dominates, the early-store trigger is firing too often.
            LOGI("KF_STORE: cond_A=%d cond_B=%d tracked=%d frames_since_kf=%d",
                 kf_cond_A ? 1 : 0, kf_cond_B ? 1 : 0,
                 tracked, frames_since_keyframe_);
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
                gray_buf_, next_good_buf_, feature_ids_,
                /*lens=*/&lens_);
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
                // Stage 2 revert (2026-05-09): restore EKF clone-pose
                // anchor for keyframe storage. Now that Stage 1 has added
                // a gravity-alignment measurement update to the EKF, p_G
                // stays bounded by physics — keyframes can once again be
                // anchored in the principled EKF state instead of the
                // working-trajectory patch.
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

                    // v23.15 (2026-05-12): TRANSPOSE the rotation here.
                    //
                    // Bug: getClonePose returns kf_R_mat = R_GtoC = WORLD→CAMERA
                    // (EKFState convention). But the KeyframeDescriptors field
                    // `R_world_cam` is contractually CAMERA→WORLD — both the
                    // explicit comment in LoopClosureDetector.cpp:636-637
                    //     "R_world_cam takes cam→world, its transpose takes
                    //      world→cam_match"
                    // and the prediction-side code at Tracker.cpp:3006
                    //     R_world_cam_pred = R_GtoI.t() * R_bc.t();   // cam→world
                    // confirm the cam→world convention.
                    //
                    // Without the transpose, every keyframe stores R_GtoC (world→cam)
                    // into a field interpreted as cam→world. Loop closure then
                    // applies another .t() expecting to get world→cam but instead
                    // gets cam→world. The composition R_match_world * R_now_world.t()
                    // is then geometrically meaningless and produces ~110-150°
                    // rotation residuals around world-Z (the yaw axis) — exactly
                    // the LC_ABS pattern in v23_14_walk_2026_05_12.logcat
                    // (median |r_R|=150.8°, 90% rotation-dominated chi² rejects).
                    //
                    // Cited derivation: r_R = log(target_R_GtoI · R_GtoI_current^T).
                    // With the storage bug, target_R_GtoI = R_bc^T · target_R_world_cam.t()
                    // where target_R_world_cam itself absorbs an extra .t() from
                    // R_match_world being wrong direction → 90-180° around z.
                    cv::Mat kf_R_mat_cw_t = kf_R_mat.t();  // world→cam → cam→world
                    cv::Matx33d R_world_cam(
                        kf_R_mat_cw_t.at<double>(0, 0), kf_R_mat_cw_t.at<double>(0, 1), kf_R_mat_cw_t.at<double>(0, 2),
                        kf_R_mat_cw_t.at<double>(1, 0), kf_R_mat_cw_t.at<double>(1, 1), kf_R_mat_cw_t.at<double>(1, 2),
                        kf_R_mat_cw_t.at<double>(2, 0), kf_R_mat_cw_t.at<double>(2, 1), kf_R_mat_cw_t.at<double>(2, 2));
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

                                // 2026-05-19 ORB-distortion fix — use the
                                // undistorted keypoint positions so triangulation
                                // (which builds P = K * [R|t] with linear K) is
                                // consistent with the pixel coords being fed in.
                                // See KeyframeDescriptors.h `keypoints_ud` for
                                // the root-cause writeup.
                                const cv::Point2f& p_now  = kf_back.keypoints_ud[q_idx].pt;
                                const cv::Point2f& p_prev = kf_prev.keypoints_ud[t_idx].pt;
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

                    // 2026-05-16 Phase 1 Step 6 — visual-only LandmarkMap
                    // wire-up. Each finite pts3d_world[i] corresponds 1:1 with
                    // kf_back.descriptors.row(i) and kf_back.keypoints[i] (the
                    // pre-allocation at L2993 + the size assertion at
                    // L3081-3082 guarantee this). Anchor every accepted
                    // landmark to `latest_clone_for_kf` (the same id-space
                    // the loop-closure DB uses); ts_ns is the keyframe's
                    // sensor timestamp. addOrMergeLandmark internally
                    // dedupes by 3D-distance + Hamming so revisits don't
                    // double-count.
                    //
                    // Phase 6.2 deliberately does NOT close the loop yet —
                    // the populated map is only observable via
                    // landmarks_added_total / landmark_map_size in
                    // event_summary. The EKF pose-only update wires in
                    // Phase 6.3-6.4; the JNI overlay in Phase 6.5.
                    {
                        const auto& kf_kps  = kf_back.keypoints;
                        const auto& kf_desc = kf_back.descriptors;
                        const bool desc_ok =
                            !kf_desc.empty() &&
                            kf_desc.type() == CV_8U &&
                            kf_desc.cols == 32 &&
                            static_cast<size_t>(kf_desc.rows) == kf_kps.size() &&
                            kf_kps.size() == pts3d_world.size();
                        int lm_added_before  =
                            landmark_map_.landmarksAddedTotal();
                        int lm_merged_before =
                            landmark_map_.landmarksMergedTotal();
                        int lm_invalid       = 0;
                        if (desc_ok) {
                            for (size_t i = 0; i < pts3d_world.size(); ++i) {
                                const cv::Point3f& p = pts3d_world[i];
                                if (!std::isfinite(p.x) ||
                                    !std::isfinite(p.y) ||
                                    !std::isfinite(p.z)) {
                                    continue;
                                }
                                const cv::Vec3d p_w(
                                    static_cast<double>(p.x),
                                    static_cast<double>(p.y),
                                    static_cast<double>(p.z));
                                // 2026-05-19 Fix #2 — host-clone (R, t) for
                                // the (now-superseded by Fix #3) anchor-
                                // relative render. Kept harmless because the
                                // anchor fields are still set on the
                                // Landmark struct; they're just ignored by
                                // the current render path.
                                // 2026-05-19 Fix #4 — also pass the host
                                // keyframe's keypoint pixel. LandmarkMap
                                // pushes it onto Landmark.observation_pixels
                                // so the windowed BA has at least one
                                // observation of this landmark from the
                                // host clone (needed for cross-frame
                                // refinement of p_world).
                                // 2026-05-19 ORB-distortion fix — pass the
                                // UNDISTORTED host keyframe pixel. This is the
                                // landmark's first observation entry in the BA
                                // ring buffer, and BA projects through linear
                                // K, so the observation pixel must live in the
                                // undistorted-pinhole space.
                                const auto& kf_kps_ud_for_add = kf_back.keypoints_ud;
                                const cv::Point2f host_kp_px =
                                    (i < kf_kps_ud_for_add.size())
                                        ? kf_kps_ud_for_add[i].pt
                                        : cv::Point2f(std::numeric_limits<float>::quiet_NaN(),
                                                       std::numeric_limits<float>::quiet_NaN());
                                const int id = landmark_map_.addOrMergeLandmark(
                                    p_w,
                                    kf_desc.row(static_cast<int>(i)),
                                    static_cast<uint64_t>(latest_clone_for_kf),
                                    R_world_cam,
                                    t_cam_world,
                                    host_kp_px,
                                    static_cast<int64_t>(timestamp_ns));
                                if (id < 0) ++lm_invalid;
                            }
                        } else {
                            // The whole keyframe is unusable — count it once
                            // so a sim with persistently bad descriptor shape
                            // shows in event_summary instead of going silent.
                            ++lm_invalid;
                            LOGI("LM_SKIP: reason=desc_shape rows=%d cols=%d "
                                 "type=%d kps=%zu pts3d=%zu",
                                 kf_desc.rows, kf_desc.cols, kf_desc.type(),
                                 kf_kps.size(), pts3d_world.size());
                        }
                        const int culled = landmark_map_.cullStaleLandmarks(
                            static_cast<int64_t>(timestamp_ns));
                        const int lm_added_delta  =
                            landmark_map_.landmarksAddedTotal()  - lm_added_before;
                        const int lm_merged_delta =
                            landmark_map_.landmarksMergedTotal() - lm_merged_before;
                        auto& ec_lm = navsight::eventCounters();
                        ec_lm.landmarks_added_total.fetch_add(
                            lm_added_delta,  std::memory_order_relaxed);
                        ec_lm.landmarks_merged_total.fetch_add(
                            lm_merged_delta, std::memory_order_relaxed);
                        ec_lm.landmarks_culled_total.fetch_add(
                            culled, std::memory_order_relaxed);
                        if (lm_invalid > 0) {
                            ec_lm.landmarks_skipped_invalid.fetch_add(
                                lm_invalid, std::memory_order_relaxed);
                        }
                        ec_lm.landmark_map_size.store(
                            static_cast<long long>(landmark_map_.size()),
                            std::memory_order_relaxed);
                        // 2026-05-24 BUG-01 fix — mirror the monotonic
                        // descriptor-refresh total (absolute store, like
                        // landmark_map_size above). Captures refreshes from
                        // both the merge path (this block) and the per-frame
                        // local-map match (touchLandmark, later this tick).
                        ec_lm.landmarks_descriptor_refreshed_total.store(
                            static_cast<long long>(
                                landmark_map_.descriptorRefreshesTotal()),
                            std::memory_order_relaxed);
                        LOGI("LM_KF: kf=%d added=%d merged=%d culled=%d "
                             "invalid=%d map_size=%zu",
                             latest_clone_for_kf, lm_added_delta,
                             lm_merged_delta, culled, lm_invalid,
                             landmark_map_.size());
                    }

                    // ── 2026-05-16 Phase 1 Step 6.4 — TRACK LOCAL MAP ──────
                    //
                    // Adapted from ORB-SLAM3 §III.B "Tracking the local map".
                    // After landmark storage, query landmarks near the current
                    // pose, project them into the camera, match against
                    // current-frame keypoints, and feed the accepted
                    // observations to the EKF as a single pose-only update.
                    // Landmarks remain FIXED (no SLAM-feature state) — this
                    // matches ORB-SLAM3's pose-only optimization pattern
                    // documented in LandmarkMap.h §"ARCHITECTURAL DECISIONS"
                    // line 19-26.
                    //
                    // Cause: without this update, the 3D landmarks accumulated
                    //  in Phase 6.2 are write-only — the EKF never reads them
                    //  and the trajectory still drifts on long walks.
                    // Change: build LandmarkObservations, call
                    //  ekf_.applyLandmarkObservations once per keyframe.
                    // Falsifier: landmarks_matched_total stays 0 across a walk
                    //  that has populated landmark_map_size > 100, OR
                    //  landmarks_msckf_accepted_total ≪ landmarks_matched_total
                    //  (matches reaching the EKF but all chi²-rejected).
                    {
                        const cv::Vec3d p_center(
                            global_t_.at<double>(0),
                            global_t_.at<double>(1),
                            global_t_.at<double>(2));
                        // 30 s age cap — older landmarks accumulated drift
                        // before any LC back-write touched them; using them
                        // for current-pose tracking risks re-injecting that
                        // stale geometry. Matches the spec in the Phase 6.4
                        // contract.
                        static constexpr int64_t kLandmarkMaxAgeNs =
                            30LL * 1'000'000'000LL;
                        const std::vector<int> nearby_ids =
                            landmark_map_.getLandmarksInRadius(
                                p_center,
                                navsight::LandmarkMap::kDefaultSearchRadiusM,
                                static_cast<int64_t>(timestamp_ns),
                                kLandmarkMaxAgeNs);

                        auto& ec_lm = navsight::eventCounters();
                        ec_lm.landmarks_observed_total.fetch_add(
                            static_cast<long long>(nearby_ids.size()),
                            std::memory_order_relaxed);

                        if (nearby_ids.empty()) {
                            // Guard the EKF against no-op O(0) updates. A
                            // single LOGI line per skipped keyframe is enough
                            // to surface this in logcat without spamming.
                            LOGI("LM_TRACK_SKIP: reason=empty kf=%d "
                                 "map_size=%zu",
                                 latest_clone_for_kf, landmark_map_.size());
                            // Clear the overlay snapshot so Phase 6.5 stops
                            // drawing landmark dots from the previous keyframe
                            // (otherwise stale orange dots persist visually
                            // when the user walks into a region with no
                            // mapped features).
                            {
                                std::lock_guard<std::mutex> lk(last_observed_mutex_);
                                last_observed_landmark_ids_.clear();
                            }
                        } else {
                            // ── 1) Project candidates + collect descriptors ──
                            //
                            // For each id returned by getLandmarksInRadius,
                            // attempt pinhole projection into the current
                            // camera. Landmarks behind / too-near / too-far /
                            // outside the image are dropped by LandmarkMap's
                            // own gates (cf. kDepthMinM / kDepthMaxM). We
                            // keep the per-landmark descriptor for the
                            // subsequent KNN match.
                            std::vector<cv::Point2f> pred_pixels;
                            pred_pixels.reserve(nearby_ids.size());
                            std::vector<int>          kept_ids;
                            kept_ids.reserve(nearby_ids.size());
                            std::vector<cv::Vec3d>    kept_p_world;
                            kept_p_world.reserve(nearby_ids.size());
                            cv::Mat                   query_descriptors(
                                0, 32, CV_8U);
                            query_descriptors.reserve(nearby_ids.size());
                            for (int id : nearby_ids) {
                                navsight::Landmark lm;
                                if (!landmark_map_.getLandmark(id, lm)) continue;
                                if (lm.descriptor.empty() ||
                                    lm.descriptor.type() != CV_8U ||
                                    lm.descriptor.cols != 32 ||
                                    lm.descriptor.rows != 1) {
                                    continue;
                                }
                                cv::Point2f px;
                                if (!landmark_map_.projectIntoCamera(
                                        id, R_world_cam, t_cam_world,
                                        fx_, fy_, cx_, cy_,
                                        width, height, px)) {
                                    continue;
                                }
                                pred_pixels.push_back(px);
                                kept_ids.push_back(id);
                                kept_p_world.push_back(lm.p_world);
                                query_descriptors.push_back(lm.descriptor);
                            }

                            // ── 2) KNN-match landmark descriptors against
                            //       the current keyframe's ORB descriptors,
                            //       Lowe ratio 0.75, then enforce spatial
                            //       15 px gate AND Hamming ≤ 50.
                            //
                            // 15 px search radius matches the canonical
                            // ORB-SLAM3 §III.B "tracking the local map"
                            // gate; 50 / 256 Hamming matches LandmarkMap's
                            // kDescriptorMaxHamming so the local-map
                            // tracker agrees with the dedup path on what
                            // "the same physical feature" means.
                            std::vector<EKFState::LandmarkObservation> obs_vec;
                            std::vector<int>                          accepted_ids_match;
                            // 2026-05-19 — Orange-dot anchor fix #3. Parallel
                            // to accepted_ids_match: the current-frame KLT
                            // pixel each matched landmark was observed at.
                            // Published with the ids into the last-observed
                            // snapshot so JNI getLandmarkSnapshot can render
                            // observed dots AT the observation pixel.
                            std::vector<cv::Point2f>                  accepted_pixels_match;
                            // 2026-05-19 Fix #11 — parallel feature-id array so
                            // per-frame KLT loop can refresh observed pixels
                            // between keyframes. See Tracker.h
                            // `last_observed_landmark_feature_ids_` for the
                            // cause/change writeup.
                            std::vector<int>                          accepted_feature_ids_match;
                            // 2026-05-24 BUG-01 verified-only refresh — map
                            // landmark_id -> its current-frame matched ORB
                            // descriptor (deep clone). Populated in the match
                            // loop; consumed AFTER applyLandmarkObservations to
                            // refresh ONLY chi2-accepted landmarks' descriptors.
                            std::unordered_map<int, cv::Mat>          matched_descriptors_match;
                            if (!query_descriptors.empty() &&
                                !kf_back.descriptors.empty() &&
                                kf_back.descriptors.type() == CV_8U &&
                                kf_back.descriptors.cols == 32 &&
                                static_cast<size_t>(kf_back.descriptors.rows)
                                    == kf_back.keypoints.size()) {
                                cv::BFMatcher matcher(cv::NORM_HAMMING, false);
                                std::vector<std::vector<cv::DMatch>> knn;
                                matcher.knnMatch(query_descriptors,
                                                 kf_back.descriptors,
                                                 knn, 2);
                                static constexpr float kLoweRatio        = 0.75f;
                                static constexpr float kPxSearchRadius   = 15.0f;
                                static constexpr float kPxSearchRadiusSq =
                                    kPxSearchRadius * kPxSearchRadius;
                                static constexpr int   kHammingMax       =
                                    navsight::LandmarkMap::kDescriptorMaxHamming;
                                obs_vec.reserve(knn.size());
                                accepted_ids_match.reserve(knn.size());
                                accepted_pixels_match.reserve(knn.size());
                                for (size_t qi = 0; qi < knn.size(); ++qi) {
                                    const auto& pair = knn[qi];
                                    if (pair.size() < 2) continue;
                                    if (pair[0].distance >=
                                        kLoweRatio * pair[1].distance) continue;
                                    if (pair[0].distance >
                                        static_cast<float>(kHammingMax)) continue;
                                    const int q_idx = pair[0].queryIdx;
                                    const int t_idx = pair[0].trainIdx;
                                    if (q_idx < 0 ||
                                        q_idx >= static_cast<int>(kept_ids.size())) continue;
                                    if (t_idx < 0 ||
                                        t_idx >= static_cast<int>(kf_back.keypoints.size())) continue;
                                    // 2026-05-19 ORB-distortion fix — math
                                    // uses UNDISTORTED keypoint, display uses
                                    // RAW. `pred_pixels` are computed from
                                    // projection via linear K (undistorted
                                    // space), so the comparison and the
                                    // MSCKF observation must both use
                                    // undistorted. The Fix #3 render path
                                    // (accepted_pixels_match) keeps raw so
                                    // overlay dots align with the camera
                                    // preview's distorted image.
                                    // Spatial gate: 15 px between predicted
                                    // pixel and matched current-frame keypoint.
                                    const cv::Point2f& px_pred = pred_pixels[q_idx];
                                    const cv::Point2f& px_meas_ud =
                                        kf_back.keypoints_ud[t_idx].pt;
                                    const cv::Point2f& px_meas_raw =
                                        kf_back.keypoints[t_idx].pt;
                                    const float dx = px_meas_ud.x - px_pred.x;
                                    const float dy = px_meas_ud.y - px_pred.y;
                                    if (dx * dx + dy * dy > kPxSearchRadiusSq) continue;

                                    EKFState::LandmarkObservation o;
                                    o.landmark_id  = kept_ids[q_idx];
                                    o.p_world      = kept_p_world[q_idx];
                                    o.pixel_meas   = px_meas_ud;
                                    // 1.0 px ORB localization accuracy at
                                    // pyramid level 0 — see ORB-SLAM3 §III.B
                                    // sigma calculation (Campos et al. 2021).
                                    o.sigma_px     = 1.0;
                                    obs_vec.push_back(o);
                                    accepted_ids_match.push_back(kept_ids[q_idx]);
                                    // 2026-05-19 — record the RAW KLT match
                                    // pixel so Fix #3 overlay can render
                                    // observed dots aligned with the raw
                                    // camera preview the user is looking at.
                                    // Math (BA, MSCKF) uses undistorted; this
                                    // is the only spot that needs raw.
                                    accepted_pixels_match.push_back(px_meas_raw);
                                    // 2026-05-19 Fix #11 — capture the KLT
                                    // feature_id linked to this landmark match
                                    // so per-frame refresh can update the
                                    // observed pixel between keyframes.
                                    // kf_back.feature_ids is parallel to its
                                    // keypoints array; -1 means the ORB row
                                    // had no KLT-spatial-proximity link at
                                    // descriptor-storage time.
                                    const int linked_fid =
                                        (t_idx >= 0 &&
                                         t_idx < static_cast<int>(kf_back.feature_ids.size()))
                                            ? kf_back.feature_ids[t_idx]
                                            : -1;
                                    accepted_feature_ids_match.push_back(linked_fid);

                                    // 2026-05-17 — touch landmark on every
                                    // matched observation (investigator
                                    // finding #3). Without this, only the
                                    // initial addOrMergeLandmark bumps
                                    // times_observed, so landmarks die at
                                    // cull time even when actively
                                    // re-observed every walk pass. With
                                    // touch + relaxed cull policy
                                    // (LandmarkMap.h:136 = 1 obs / 120 s
                                    // grace), landmarks now naturally
                                    // accumulate retention from re-visits.
                                    // 2026-05-19 Fix #4 — also push the
                                    // matched pixel into Landmark
                                    // observation_pixels. The windowed BA
                                    // refines `p_world` using these per-
                                    // keyframe observations. Use the
                                    // UNDISTORTED pixel (px_meas_ud) — BA
                                    // projects through linear K, so the
                                    // observation must be in undistorted
                                    // pinhole space. Same root-cause fix as
                                    // KeyframeDescriptors::keypoints_ud.
                                    // 2026-05-24 BUG-01 verified-only refresh —
                                    // touch (retention + pixel ring) on every
                                    // match, but DEFER the descriptor refresh:
                                    // capture the matched current-frame
                                    // descriptor here and apply it ONLY to
                                    // landmarks the EKF chi2 gate accepts
                                    // (below). Refreshing pre-verification
                                    // corrupts the descriptor toward a wrong
                                    // feature and regresses heading (2026-05-23).
                                    // t_idx is the train-side (current-frame)
                                    // index, bounds-checked above;
                                    // descriptors.rows == keypoints.size() per
                                    // the matcher guard, so .row(t_idx) valid.
                                    landmark_map_.touchLandmark(
                                        kept_ids[q_idx],
                                        static_cast<uint64_t>(latest_clone_for_kf),
                                        static_cast<int64_t>(timestamp_ns),
                                        px_meas_ud);
                                    matched_descriptors_match[kept_ids[q_idx]] =
                                        kf_back.descriptors.row(t_idx).clone();
                                }
                            }

                            ec_lm.landmarks_matched_total.fetch_add(
                                static_cast<long long>(obs_vec.size()),
                                std::memory_order_relaxed);

                            // ── 3) Hand to the EKF.
                            //
                            // 2026-05-16 — DISABLED after v28 walk validation.
                            //
                            // Cause: applyLandmarkObservations stacks N=256
                            // simultaneous landmark observations into a single
                            // Kalman update with σ_px=1.0 each. The 256 obs are
                            // NOT statistically independent — they all derive
                            // from the same LandmarkMap state which was
                            // triangulated using the same EKF pose. The Joseph
                            // form treats them as N redundant measurements of
                            // p_G and slams the state with N-fold over-
                            // confidence. Worse: landmarks are anchored to
                            // p_G at creation time, so each "observation" is
                            // effectively measuring the EKF's own past state
                            // → positive feedback → divergence.
                            //
                            // v28 walk evidence: 108 m walked, |p_G|_final =
                            // 417 m. First 9 s |p_G| stayed at ~0.5 m (map
                            // empty); from kf=59 onward (first apply fires)
                            // |p_G| jumps to 73 m in 13 s and continues to
                            // diverge. loop_closure_corrections_applied = 0
                            // because LC chi² gate correctly refuses 200+ m
                            // corrections. Phase 6.4 was the only new EKF
                            // coupling vs v25 (which had healthy LC corrections).
                            //
                            // Why this can't be fixed with a clamp / cap /
                            // sigma tweak: those are symptom gates — the
                            // architecture is wrong. ORB-SLAM3 handles this
                            // via full BA that jointly optimizes p_G + every
                            // landmark; our filter has no such mechanism, so
                            // when landmark observations pull p_G, only p_G
                            // moves while the landmarks themselves remain
                            // anchored to creation time. A correct EKF
                            // coupling would require either (a) landmark
                            // states marginalized via MSCKF (Mourikis-Roumeliotis
                            // 2007 style) where each landmark's null-space
                            // projection removes p_G from H, OR (b) a windowed
                            // BA wrapping the filter. Neither lands today.
                            //
                            // Architectural decision (not a patch): the
                            // LandmarkMap remains a passive observability
                            // layer. Phase 6.2 (populate) and Phase 6.5
                            // (overlay) stay live so the visual map is
                            // observable and the orange-dot overlay renders.
                            // Phase 6.3's applyLandmarkObservations method
                            // stays compiled but UNCALLED — to be revisited
                            // when MSCKF-style null-space projection lands
                            // (Phase 7 candidate, NOT in scope for Phase 1).
                            //
                            // Falsifier on next walk: |p_G|_final should
                            // track GPS path within ±20 m on a 100 m walk;
                            // loop_closure_corrections_applied > 0;
                            // landmarks_msckf_accepted_total = 0 (disabled);
                            // landmarks_observed_total > 0 (overlay works).
                            EKFState::LandmarkUpdateResult res;
                            // PHASE_B_6_4_REENABLE_2026_05_19 — apply landmark
                            // observations to EKF so loop-1 landmarks anchor
                            // visually during loop 2.
                            //
                            // Safety:  v28 4× position runaway is the precedent.
                            //          Root cause never identified, but the R_bc
                            //          and V-shape fixes (2026-05-18/19) MIGHT
                            //          have eliminated the trigger. We re-enable
                            //          behind a hard guard: capture p_G before/
                            //          after the call; if |Δp| > 1.0 m in a
                            //          single update, RESTORE p_G_before and
                            //          increment landmark_obs_runaway_total.
                            //
                            // Falsifier (per-call): LANDMARK_OBS_APPLY log shows
                            //          n_matched, |dp|, |dtheta|. Per-call |dp|
                            //          should be < 0.3 m typically; > 1 m fires
                            //          the safety reject. landmark_obs_runaway
                            //          _total should stay 0 in a healthy walk.
                            //
                            // Easy revert: grep PHASE_B_6_4_REENABLE_2026_05_19
                            // and revert this hunk to a single line creating an
                            // empty LandmarkUpdateResult.
                            if (!obs_vec.empty()) {
                                const cv::Matx33d R_bc = ekf_.getExtrinsicsRotation();
                                // Safety: capture p_G before so we can roll back
                                // on runaway. clone() because getPosition returns
                                // a reference to the live EKF state.
                                cv::Mat p_G_before = ekf_.getPosition().clone();
                                res = ekf_.applyLandmarkObservations(
                                    obs_vec, R_bc,
                                    fx_, fy_, cx_, cy_,
                                    width, height);
                                cv::Mat p_G_after = ekf_.getPosition();
                                cv::Mat dp = p_G_after - p_G_before;
                                const double dp_mag = cv::norm(dp);
                                constexpr double kLandmarkMaxDp = 1.0;  // 1m hard cap
                                if (dp_mag > kLandmarkMaxDp) {
                                    // ROLLBACK: restore EKF position to
                                    // pre-update state. Other state changes
                                    // (rotation, velocity) are kept since the
                                    // safety is specifically for the v28
                                    // position-runaway mode.
                                    ekf_.setPosition(p_G_before);
                                    ec_lm.landmark_obs_runaway_total.fetch_add(
                                        1, std::memory_order_relaxed);
                                    LOGI("LANDMARK_OBS_REJECT: |dp|=%.3fm > %.1fm "
                                         "cap — restored p_G, no landmarks applied. "
                                         "n_matched=%zu accepted=%d",
                                         dp_mag, kLandmarkMaxDp,
                                         obs_vec.size(), res.accepted);
                                } else {
                                    LOGI("LANDMARK_OBS_APPLY: n_matched=%zu "
                                         "accepted=%d rej_chi2=%d rej_huber=%d "
                                         "rej_depth=%d rej_img=%d |dp|=%.3fm",
                                         obs_vec.size(), res.accepted,
                                         res.rejected_chi2, res.rejected_huber,
                                         res.rejected_depth, res.rejected_outside_image,
                                         dp_mag);
                                    ec_lm.landmarks_msckf_accepted_total.fetch_add(
                                        res.accepted, std::memory_order_relaxed);
                                    ec_lm.landmarks_msckf_rejected_chi2_total.fetch_add(
                                        res.rejected_chi2, std::memory_order_relaxed);
                                    ec_lm.landmarks_msckf_rejected_huber_total.fetch_add(
                                        res.rejected_huber, std::memory_order_relaxed);
                                    ec_lm.landmarks_msckf_rejected_depth_total.fetch_add(
                                        res.rejected_depth, std::memory_order_relaxed);
                                    ec_lm.landmarks_msckf_rejected_image_total.fetch_add(
                                        res.rejected_outside_image, std::memory_order_relaxed);
                                    // 2026-05-24 BUG-01 verified-only refresh —
                                    // the chi2 gate accepted these obs AND the
                                    // update applied (|dp|<=1m, not rolled
                                    // back), so refresh the representative ORB
                                    // descriptor ONLY for these verified ids.
                                    for (int acc_id : res.accepted_landmark_ids) {
                                        auto dit =
                                            matched_descriptors_match.find(acc_id);
                                        if (dit != matched_descriptors_match.end()) {
                                            landmark_map_.refreshLandmarkDescriptor(
                                                acc_id, dit->second);
                                        }
                                    }
                                }
                            }
                            /* SUPERSEDED PHASE_B_6_4_REENABLE_2026_05_19 — was
                               disabled per v28 divergence:
                            // (no call — applyLandmarkObservations not invoked)
                            */
                            // Phase 6.5 overlay: publish the ids that survived
                            // matching (descriptor + spatial gates) and were
                            // handed to the EKF. The Phase 6.3 contract returns
                            // only aggregate accept/reject counts, NOT which
                            // specific ids survived its chi²/Huber/depth gates,
                            // so we cannot label a subset. Publishing the
                            // matched set keeps the overlay honest (orange =
                            // "this map landmark was visible AND matched a
                            // current keypoint AND was handed to the EKF") and
                            // avoids a symptom-patch where we'd silently lie
                            // about which ids survived the EKF gates.
                            std::vector<int> accepted_ids =
                                std::move(accepted_ids_match);
                            // 2026-05-19 — move parallel pixel array alongside.
                            // Same lifetime + same lock as accepted_ids;
                            // ordering preserved so ids[i] ↔ pixels[i].
                            std::vector<cv::Point2f> accepted_pixels =
                                std::move(accepted_pixels_match);

                            // Per-keyframe summary so a single grep
                            // LM_TRACK: in logcat reveals the full match
                            // pipeline at every keyframe tick.
                            LOGI("LM_TRACK: kf=%d nearby=%zu matched=%zu accepted=%d "
                                 "rej_chi2=%d rej_huber=%d rej_depth=%d rej_image=%d "
                                 "chi2_total=%.2f",
                                 latest_clone_for_kf, nearby_ids.size(),
                                 obs_vec.size(),
                                 res.accepted, res.rejected_chi2,
                                 res.rejected_huber, res.rejected_depth,
                                 res.rejected_outside_image,
                                 res.chi2_total);

                            // Publish accepted ids for Phase 6.5 overlay.
                            // Snapshot copy under last_observed_mutex_ so any
                            // reader (JNI thread, debug overlay) sees a
                            // coherent vector even if a new keyframe arrives
                            // mid-read.
                            {
                                std::lock_guard<std::mutex> lk(last_observed_mutex_);
                                last_observed_landmark_ids_    = std::move(accepted_ids);
                                last_observed_landmark_pixels_ = std::move(accepted_pixels);
                                // 2026-05-19 Fix #11 — parallel feature-id
                                // array. Same lifetime + lock. Per-frame
                                // refresh path (in the KLT-pixel loop) reads
                                // these to know which KLT track maps to each
                                // landmark observation.
                                last_observed_landmark_feature_ids_ =
                                    std::move(accepted_feature_ids_match);
                            }
                        }
                    }

                    // 2026-05-19 ORB-distortion fix — LC path. Pass
                    // UNDISTORTED keypoints so the LC PnP solver
                    // (LoopClosureDetector.cpp pts2d → solvePnPRansac with
                    // linear K) operates in the same pinhole space as the
                    // projection matrices. Pre-fix the PnP was solving with
                    // distorted pts2d and linear K, biasing every loop-
                    // closure pose estimate by the lens distortion magnitude.
                    loop_closure_.addKeyframe(
                        static_cast<uint64_t>(latest_clone_for_kf),
                        static_cast<double>(timestamp_ns),
                        kf_back.descriptors,
                        kf_back.keypoints_ud,
                        pts3d_world,
                        R_world_cam,
                        t_cam_world,
                        scalar_heading_);

                    // 2026-05-13 Phase 1 Step 5: add a corresponding node to
                    // the pose graph. The graph mirrors the LC keyframe DB —
                    // same id-space, same cadence. addNode auto-creates an
                    // odometry edge from the previous node (relative pose
                    // expressed in previous node's yawed frame), so the chain
                    // is built incrementally as keyframes accrue. The
                    // mapping clone_id_to_pg_node_ lets us resolve
                    // LoopMatch.{matched_kf_id, now_kf_id} → pose-graph
                    // node-ids when a loop edge fires.
                    {
                        // 2026-05-13 Step 5 plan-compliance (line 187):
                        // Σ_odom MUST come from the EKF clone covariance.
                        // Pull the latest clone's 6-DOF block from P_
                        // (layout: [δθ(3), δp(3)]) and extract:
                        //   σ²_xy = P[idx+3,idx+3] + P[idx+4,idx+4]
                        //   σ²_z  = P[idx+5,idx+5]
                        //   σ²_yaw ≈ P[idx+2,idx+2] (body-z rotation; for a
                        //           vertical phone with R_bc=diag(1,-1,-1)
                        //           this maps approximately to world-yaw).
                        // PoseGraph then derives the auto-odom edge info
                        // from the variance INCREMENT between consecutive
                        // clones — see addNode in PoseGraph.cpp.
                        double sigma_xy_sq  = 0.0;
                        double sigma_z_sq   = 0.0;
                        double sigma_yaw_sq = 0.0;
                        const int cov_idx = ekf_.getCloneCovIdx(latest_clone_for_kf);
                        if (cov_idx >= 0) {
                            const cv::Mat P_full = ekf_.getCovariance();
                            if (!P_full.empty() &&
                                P_full.type() == CV_64F &&
                                cov_idx + 5 < P_full.rows) {
                                const double s2_x = P_full.at<double>(cov_idx + 3, cov_idx + 3);
                                const double s2_y = P_full.at<double>(cov_idx + 4, cov_idx + 4);
                                const double s2_z = P_full.at<double>(cov_idx + 5, cov_idx + 5);
                                const double s2_yaw_body = P_full.at<double>(cov_idx + 2, cov_idx + 2);
                                if (std::isfinite(s2_x) && std::isfinite(s2_y) &&
                                    std::isfinite(s2_z) && std::isfinite(s2_yaw_body)) {
                                    sigma_xy_sq  = std::max(0.0, s2_x) + std::max(0.0, s2_y);
                                    sigma_z_sq   = std::max(0.0, s2_z);
                                    sigma_yaw_sq = std::max(0.0, s2_yaw_body);
                                }
                            }
                        }
                        const int pg_node_id = pose_graph_.addNode(
                            static_cast<double>(t_cam_world[0]),
                            static_cast<double>(t_cam_world[1]),
                            static_cast<double>(t_cam_world[2]),
                            scalar_heading_,
                            timestamp_ns,
                            sigma_xy_sq, sigma_z_sq, sigma_yaw_sq);
                        clone_id_to_pg_node_[
                            static_cast<uint64_t>(latest_clone_for_kf)] =
                            pg_node_id;
                        pg_node_to_clone_id_[pg_node_id] =
                            static_cast<uint64_t>(latest_clone_for_kf);
                        last_pg_node_id_ = pg_node_id;
                        LOGI("POSE_GRAPH_ADD_NODE: pg_node=%d clone_id=%d "
                             "pos=(%.3f,%.3f,%.3f) yaw_deg=%.2f "
                             "sigma_xy_m=%.4f sigma_z_m=%.4f sigma_yaw_deg=%.3f "
                             "n_nodes=%d n_odom=%d n_loop=%d",
                             pg_node_id, latest_clone_for_kf,
                             static_cast<double>(t_cam_world[0]),
                             static_cast<double>(t_cam_world[1]),
                             static_cast<double>(t_cam_world[2]),
                             scalar_heading_ * 180.0 / M_PI,
                             std::sqrt(sigma_xy_sq),
                             std::sqrt(sigma_z_sq),
                             std::sqrt(sigma_yaw_sq) * 180.0 / M_PI,
                             pose_graph_.getNodeCount(),
                             pose_graph_.getOdomEdgeCount(),
                             pose_graph_.getLoopEdgeCount());
                    }

                    // Counter (Agent A): kf-count-in-database. Sample once
                    // per addKeyframe call to keep cost bounded.
                    auto& ec = navsight::eventCounters();
                    ec.loop_closure_kf_count_in_db.store(
                        static_cast<long long>(loop_closure_.getKeyframeCount()),
                        std::memory_order_relaxed);

                    // Stage 2 revert: predicted current camera pose now
                    // comes back from the EKF (post-Stage-1 it's bounded).
                    // R_world_cam = R_GtoI^T · R_bc^T (cam→world).
                    cv::Matx33d R_GtoI;
                    {
                        cv::Mat R_GtoI_mat = ekf_.getRotation();
                        for (int r = 0; r < 3; ++r) {
                            for (int c = 0; c < 3; ++c) {
                                R_GtoI(r, c) = R_GtoI_mat.at<double>(r, c);
                            }
                        }
                    }
                    cv::Matx33d R_bc = ekf_.getExtrinsicsRotation();
                    cv::Matx33d R_world_cam_pred = R_GtoI.t() * R_bc.t();
                    cv::Vec3d t_cam_world_pred(0., 0., 0.);
                    {
                        cv::Mat p_mat = ekf_.getPosition();
                        t_cam_world_pred[0] = p_mat.at<double>(0);
                        t_cam_world_pred[1] = p_mat.at<double>(1);
                        t_cam_world_pred[2] = p_mat.at<double>(2);
                    }
                    // Inline the same calc as Tracker::getPositionCovarianceXZ
                    // (Tracker.cpp:548-573) without taking pose_mutex_ — we're
                    // already on the camera thread which is the sole EKF
                    // writer, so no race. P[12,12] / P[13,13] are the X / Y
                    // (East / North) position-error variances post-Z-up
                    // alignment.
                    double sigma_p_xy = 0.0;
                    if (ekf_.isFullInitialized()) {
                        cv::Mat P = ekf_.getCovariance();
                        if (!P.empty() && P.rows >= 15 && P.cols >= 15 && P.type() == CV_64F) {
                            const double v_xx = P.at<double>(12, 12);
                            const double v_yy = P.at<double>(13, 13);
                            sigma_p_xy = std::sqrt(std::max(v_xx, 0.0) + std::max(v_yy, 0.0));
                        }
                    }
                    const double search_radius_m =
                        std::min(30.0, std::max(2.0, 3.0 * sigma_p_xy));

                    // Publish the same descriptors + keypoints to the 1 Hz
                    // worker thread so its next query tick has a fresh
                    // most-recent keyframe to fingerprint.
                    // 2026-05-19 ORB-distortion fix — LC query path. The
                    // worker thread's `q_keypoints` flows into
                    // tryDetectLoopWithCandidates / tryDetectLoopGeometric,
                    // both of which use linear K for projection / PnP. Pass
                    // undistorted keypoints to match the addKeyframe side.
                    publishLoopClosureQueryKeyframe(
                        latest_clone_for_kf, timestamp_ns,
                        kf_back.descriptors, kf_back.keypoints_ud,
                        fx_, fy_, cx_, cy_,
                        scalar_heading_,
                        next_good_buf_,           // KLT corners in current frame
                        R_world_cam_pred,
                        t_cam_world_pred,
                        width, height,
                        search_radius_m);
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
        consumeLoopClosureMatchIfReady(imu);

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

    // ── 11.9 Tier 1 revert: §11.9 mirror block disabled ──────────────────────
    // The audit's Fix-A mirror (EKF authoritative, global_t_ as read mirror)
    // is reverted as of the Tier 1 revert. setPosition(global_t_) is now
    // called BEFORE propagateIMU (restored above), so global_t_ remains the
    // canonical position source. The MSCKF/LC updates touch p_G_ inside the
    // EKF for the duration of one frame, then setPosition overwrites it
    // again next frame — this is the v22 architecture.
    //
    // global_t_ is updated by the visual-VO + PDR pipeline (see §11 below),
    // not by the EKF mirror. Output §12 reads global_t_ directly.
    /* SUPERSEDED 2026-05-16 (Tier 1 revert per agent A10):
    if (ekf_.isFullInitialized()) {
        cv::Mat p_now = ekf_.getPosition();
        if (!p_now.empty() && p_now.rows == 3 && p_now.cols == 1) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            global_t_ = p_now.clone();
        }
    }
    */

    // ── 12. Output assembly ───────────────────────────────────────────────────
    // Rotation: source from EKFState when full-init. EKF rotation is
    // gravity-aligned and corrected by the keyframe yaw update, so it is
    // strictly better than the gyro-only global_R_ before EKF init.
    //
    // Position: 2026-05-16 Fix A — global_t_ is EKF read-mirror (see §11.9).
    // Old v17 note preserved: "pure IMU integration grows quadratically with bias
    // errors; EKF p_G flew around before MSCKF features built up." That was caused
    // by setPosition() collapsing P_pp to zero — now removed. With position entering
    // EKF via measurement updates only (updateRelativePose/PDRStep), the bias
    // quadratic problem is resolved. Visual corrections flow through EKF Kalman
    // gain; global_t_ is updated from EKF in §11.9 above, not by direct VO write.
    //
    // Architecture: `global_t_` is the user-facing trajectory.
    // EKF is the primary position owner: updateRelativePose, updatePDRStep, and
    // updateAbsolutePose (LC) constrain p_G_; global_t_ reads back from EKF.
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
    out.trackedPointAges = std::move(tracked_pts_ages);
    out.trackedPointInlierFlags = std::move(tracked_pts_inlier_flags);
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
    out.keyframe_stored = stored_keyframe_this_frame;

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

    // ── Ground-plane metric-scale eval (READ-ONLY; gpt_speed_suggestion.md Phase 0) ─────────────────
    // Recover the monocular scale VIO lacks by fitting a plane to the lower-image triangulated points
    // and dividing the KNOWN camera height by the plane's VIO-scale height. Runs every kGroundPlaneInterval
    // frames, only when a camera height has been set (scooter mount). NEVER feeds the dot — it is logged
    // for offline scoring vs GPS and drawn on the camera overlay. Cheap (~1-2 ms / few frames).
    if (camera_height_m_ > 0.0 && (frame_counter_ % kGroundPlaneInterval) == 0 &&
        points_3d_current_.size() >= 12 && next_good_buf_.size() >= points_3d_current_.size()) {
        GroundPlaneResult gp = ground_plane_estimator_.estimate(
            next_good_buf_, points_3d_current_, gray_buf_.cols, gray_buf_.rows,
            fx_use, fy_use, cx_use, cy_use, camera_height_m_,
            analyzer_rotation_deg_.load(std::memory_order_relaxed));
        gp_valid_.store(gp.is_valid, std::memory_order_relaxed);
        gp_scale_.store(gp.ground_scale, std::memory_order_relaxed);
        gp_conf_.store(gp.confidence, std::memory_order_relaxed);
        gp_hvio_.store(gp.h_vio, std::memory_order_relaxed);
        gp_horizon_v_.store(gp.horizon_v_px, std::memory_order_relaxed);
        gp_cands_.store(gp.n_candidates, std::memory_order_relaxed);
        gp_inliers_.store(gp.n_inliers, std::memory_order_relaxed);
        if (gp.is_valid && (frame_counter_ % 15) == 0) {
            LOGI("GROUND_PLANE: scale=%.3f conf=%.2f h_vio=%.3f cand=%d inl=%d horizon_v=%.0f",
                 gp.ground_scale, gp.confidence, gp.h_vio, gp.n_candidates, gp.n_inliers, gp.horizon_v_px);
        }
    }
    // AV-style ground-plane GRID for the camera overlay — runs every frame when a camera height is set
    // (independent of is_static, so it shows while stopped too), gated internally on IMU gravity-settle.
    if (camera_height_m_ > 0.0) computeGroundGrid(imu);
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

    // 2026-05-19 ORB-distortion root cause fix — extension to reloc path.
    //
    // cur_kps come straight from ORB on the raw camera image, so .pt is in
    // distorted pixel coords. findEssentialMat below uses the linear K, so
    // mixing them produces a biased pose estimate (same bug class as
    // LandmarkMap triangulation that v57 just fixed). Undistort the
    // keypoint positions in-place so both cur_kps and kfd.keypoints_ud
    // (used below) live in the same undistorted-pinhole space.
    if (lens_.isReady() && lens_.hasDistortion()) {
        std::vector<cv::Point2f> raw_pts;
        raw_pts.reserve(cur_kps.size());
        for (const auto& kp : cur_kps) raw_pts.push_back(kp.pt);
        lens_.undistortPoints(raw_pts);
        for (size_t i = 0; i < cur_kps.size() && i < raw_pts.size(); ++i) {
            cur_kps[i].pt = raw_pts[i];
        }
    }

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
                // 2026-05-19 ORB-distortion fix — reloc PnP path.
                // cur_kps were undistorted above; kfd.keypoints_ud was
                // populated at storeKeyframeDescriptors time. Both inputs
                // to findEssentialMat are now in the linear-K-consistent
                // undistorted pinhole space.
                cur_pts.push_back(cur_kps[m.queryIdx].pt);
                kf_pts.push_back(kfd.keypoints_ud[m.trainIdx].pt);
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

    // 2026-05-19 Fix #4 — Peek the LandmarkMap candidates BEFORE the
    // too-few-landmarks gate so the gate considers the COMBINED feature
    // count (SLAM features + LandmarkMap entries). v56 walk: gate fired 44
    // times because `lm_snap.size() < 3` (i.e. SLAM features < 3), even
    // though the LandmarkMap had >>3 candidates with multi-keyframe
    // observations. Without this fix BA fires ~0.08 Hz instead of
    // per-keyframe (~3-5 Hz), and the whole landmark-refinement pipeline
    // stalls. The peek is cheap (same KD-tree-backed lookup that the BA
    // worker will do for real).
    const auto lm_obs_preview = landmark_map_.getLandmarksWithObsInClones(
        clone_ids, /*min_obs=*/kMinObs);
    const size_t combined_features = lm_snap.size() + lm_obs_preview.size();
    if (combined_features < 3) {
        // < 3 features (SLAM + Landmark) → solver is under-determined.
        // Pre-Fix-#4 this gate only counted SLAM features (lm_snap); now
        // the LandmarkMap can keep BA running even when SLAM promotion is
        // bottlenecked, which is the dominant failure mode in walks v54-v55.
        const long long n = ec.ba_skipped_too_few_landmarks.fetch_add(
            1, std::memory_order_relaxed) + 1;
        if (n % 30 == 1) LOGI("BA: skipped (combined=%zu<3 slam=%zu lm=%zu) count=%lld promoted_total=%lld",
                              combined_features, lm_snap.size(),
                              lm_obs_preview.size(), n,
                              ec.slam_promotions_total.load(std::memory_order_relaxed));
        return false;
    }
    /* SUPERSEDED 2026-05-19 — kept for greppability of the original gate:
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
    */

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
    //
    // 2026-05-19 Fix #4 — Meta extended with `is_landmark` so the post-
    // solve dispatch can branch: SLAM features go through removeSlamFeature
    // + addSlamFeature (existing path), LandmarkMap entries go through
    // setLandmarkPosition (new path). landmark_id is the LandmarkMap id,
    // valid only when is_landmark = true.
    struct Meta {
        int  feature_id;
        int  slam_slot;
        int  anchor_clone_id;
        bool is_landmark = false;
        int  landmark_id = -1;
    };
    std::vector<Meta> meta;
    meta.reserve(lm_snap.size());

    // ── (a) SLAM features ──────────────────────────────────────────────
    // Lifecycle holds the original anchor; we need it for re-seeding.
    for (const auto& l : lm_snap) {
        const auto* lc = feature_mgr_.getLifecycle(l.feature_id);
        if (!lc) continue;
        WindowedBA::FeatureObs f;
        f.feature_id = l.feature_id;
        f.p_w_in     = l.p_world;
        f.obs        = l.obs;  // already (clone_id, pixel_uv) pairs
        features.push_back(std::move(f));
        meta.push_back({l.feature_id, l.slam_slot, lc->anchor_clone_id,
                         /*is_landmark=*/false, /*landmark_id=*/-1});
    }

    // ── (b) 2026-05-19 Fix #4 — LandmarkMap entries with ≥ 2 obs in window.
    //
    // Cause this exists: LandmarkMap entries were never refined after
    // initial triangulation. As the EKF state evolves via MSCKF/SLAM/
    // ZRUP/IMU propagation (every frame, including at standstill), the
    // stored `p_world` becomes inconsistent with the live camera pose
    // belief, and their projections drift on the overlay (the user-
    // visible bug). ORB-SLAM3 fixes this by including MapPoints as
    // optimization variables in per-keyframe Local BA. NavSight's BA
    // already supports joint pose+feature refinement (WindowedBA::solve
    // optimises features[].p_w_out alongside poses[]), we just weren't
    // feeding LandmarkMap entries to it.
    //
    // Cap kMaxLmInBA prevents the BA solve cost from blowing up: at
    // K=5 clones × N landmarks, the Schur-complement on the landmark
    // block is O(K²N + N) per LM iteration. 80 landmarks + 12 SLAM
    // features ≈ 92 features, still comfortably under the 200 ms budget.
    constexpr int kMaxLmInBA = 80;
    // 2026-05-19 Fix #4 — reuse the lm_obs_preview computed above for the
    // combined-gate check. Avoids a second KD-tree-backed query on the same
    // window. Same data, same lifetime (we're still on the camera thread).
    const auto& lm_obs = lm_obs_preview;
    int lm_added_to_ba = 0;
    int min_obs_seen   = INT_MAX;
    for (const auto& lo : lm_obs) {
        if (lm_added_to_ba >= kMaxLmInBA) break;
        if (lo.id < 0) continue;
        if (static_cast<int>(lo.obs.size()) < kMinObs) continue;
        if (static_cast<int>(lo.obs.size()) < min_obs_seen) {
            min_obs_seen = static_cast<int>(lo.obs.size());
        }
        WindowedBA::FeatureObs f;
        f.feature_id = lo.id;
        f.p_w_in     = lo.p_world;
        f.obs.reserve(lo.obs.size());
        for (const auto& [clone_id, px] : lo.obs) {
            // Convert cv::Point2f → cv::Vec2d (BA expects double).
            f.obs.emplace_back(clone_id,
                                cv::Vec2d(static_cast<double>(px.x),
                                          static_cast<double>(px.y)));
        }
        features.push_back(std::move(f));
        meta.push_back({/*feature_id=*/lo.id, /*slam_slot=*/-1,
                         /*anchor_clone_id=*/-1,
                         /*is_landmark=*/true, /*landmark_id=*/lo.id});
        ++lm_added_to_ba;
    }
    if (lm_added_to_ba > 0) {
        auto& ec_ba_lm = navsight::eventCounters();
        ec_ba_lm.landmarks_in_ba_solve_sum.fetch_add(
            lm_added_to_ba, std::memory_order_relaxed);
        if (min_obs_seen != INT_MAX) {
            // Track the min so we know if all landmarks are stuck at the
            // floor of 2 obs vs accumulating more across keyframes.
            long long expected = ec_ba_lm.landmark_obs_in_ba_history_min.load(
                std::memory_order_relaxed);
            const long long candidate = static_cast<long long>(min_obs_seen);
            // CAS-update to min; lock-free monotonic-min.
            while (expected == 0 || candidate < expected) {
                if (ec_ba_lm.landmark_obs_in_ba_history_min.compare_exchange_weak(
                        expected, candidate, std::memory_order_relaxed)) {
                    break;
                }
            }
        }
    }
    LOGI("BA: enqueue clones=%zu slam_features=%zu lm_added=%d "
         "min_lm_obs=%d total_features=%zu",
         clone_snap.size(), lm_snap.size(), lm_added_to_ba,
         (min_obs_seen == INT_MAX ? 0 : min_obs_seen), features.size());

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
            // Split refined results: SLAM features are deferred to the
            // camera thread (needs EKF mutex via removeSlamFeature +
            // addSlamFeature), LandmarkMap entries are written inline on
            // the BA worker thread because LandmarkMap has its own mutex
            // and setLandmarkPosition is thread-safe.
            std::vector<BARefinedLandmark> refined_slam;
            refined_slam.reserve(features_in.size());
            int refined_lm_count = 0;
            double max_lm_dp_m  = 0.0;
            for (size_t i = 0; i < features_in.size() && i < meta_in.size(); ++i) {
                if (meta_in[i].is_landmark) {
                    // 2026-05-19 Fix #4 — write back to LandmarkMap.
                    const cv::Vec3d p_in  = features_in[i].p_w_in;
                    const cv::Vec3d p_out = features_in[i].p_w_out;
                    const cv::Vec3d dp_v  = p_out - p_in;
                    const double dp = std::sqrt(dp_v[0]*dp_v[0] +
                                                  dp_v[1]*dp_v[1] +
                                                  dp_v[2]*dp_v[2]);
                    if (landmark_map_.setLandmarkPosition(meta_in[i].landmark_id,
                                                           p_out)) {
                        ++refined_lm_count;
                        if (dp > max_lm_dp_m) max_lm_dp_m = dp;
                    }
                    continue;
                }
                BARefinedLandmark r;
                r.feature_id      = meta_in[i].feature_id;
                r.slam_slot       = meta_in[i].slam_slot;
                r.anchor_clone_id = meta_in[i].anchor_clone_id;
                r.p_world_refined = (cv::Mat_<double>(3, 1)
                    << features_in[i].p_w_out[0],
                       features_in[i].p_w_out[1],
                       features_in[i].p_w_out[2]);
                refined_slam.push_back(std::move(r));
            }
            if (refined_lm_count > 0) {
                navsight::eventCounters().landmarks_refined_total.fetch_add(
                    refined_lm_count, std::memory_order_relaxed);
                LOGI("BA: refined_landmarks=%d max_lm_dp_cm=%.2f",
                     refined_lm_count, max_lm_dp_m * 100.0);
            }
            std::lock_guard<std::mutex> lock(ba_result_mutex_);
            ba_result_landmarks_ = std::move(refined_slam);
            ba_result_pending_   = !ba_result_landmarks_.empty();
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
        double yaw_rad,
        const std::vector<cv::Point2f>& klt_corners,
        const cv::Matx33d& R_world_cam_pred,
        const cv::Vec3d&   t_cam_world_pred,
        int img_w, int img_h,
        double search_radius_m) {
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
        // Step 7.1 — geometric path payload. Empty/zero values disable the
        // geom fallback for this query (worker checks corners.empty() and
        // search_radius_m > 0 before attempting).
        loop_closure_query_klt_corners_     = klt_corners;
        loop_closure_query_R_world_cam_     = R_world_cam_pred;
        loop_closure_query_t_cam_world_     = t_cam_world_pred;
        loop_closure_query_img_w_           = img_w;
        loop_closure_query_img_h_           = img_h;
        loop_closure_query_search_radius_m_ = search_radius_m;
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
    // Step 7.1 — extras snapshotted alongside the BoW query.
    std::vector<cv::Point2f>  q_klt_corners;
    cv::Matx33d               q_R_world_cam = cv::Matx33d::eye();
    cv::Vec3d                 q_t_cam_world(0., 0., 0.);
    int                       q_img_w = 0, q_img_h = 0;
    double                    q_search_radius_m = 0.;

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
            // Step 7.1 — geom path extras (additive; OK to be empty/zero).
            q_klt_corners      = loop_closure_query_klt_corners_;
            q_R_world_cam      = loop_closure_query_R_world_cam_;
            q_t_cam_world      = loop_closure_query_t_cam_world_;
            q_img_w            = loop_closure_query_img_w_;
            q_img_h            = loop_closure_query_img_h_;
            q_search_radius_m  = loop_closure_query_search_radius_m_;
            loop_closure_query_has_data_ = false;
        }

        // Defensive bail: malformed snapshot shouldn't reach here, but
        // skip it instead of feeding the detector a degenerate input.
        if (q_descriptors.empty() || q_keypoints.empty() ||
            q_fx <= 1.0 || q_fy <= 1.0 || q_kf_id < 0) {
            continue;
        }

        ec.loop_closure_attempts.fetch_add(1, std::memory_order_relaxed);

        // Phase 6.4c — Spatial pre-filter via LandmarkMap.
        //
        // Cause: DBoW2 alone searches every keyframe for visual similarity;
        //   with hundreds of KFs (and after Step 5 pose-graph optimization
        //   shifts old poses), it can pick visually-similar but
        //   geographically-distant places as the best candidate.
        // Change: union of observed_in_kfs across landmarks within
        //   kLcSpatialSearchRadiusM of the EKF-predicted camera position.
        //   Hand that filter to tryDetectLoopWithCandidates so DBoW2's
        //   score list is intersected against it before geometric
        //   verification. Empty filter = fall back to legacy full-DB search.
        // Falsifier: a multi-loop walk should show
        //   loop_closure_spatial_candidates_total > 0 once the LandmarkMap
        //   populates, loop_closure_rejects_pnp DROP (fewer geographic
        //   false positives), loop_closure_accepts RISE. If candidates
        //   stay 0 across a 100m walk, LandmarkMap isn't populating fast
        //   enough — fall back path keeps the system functional.
        //
        // Radius source: LandmarkMap::kDefaultSearchRadiusM (30 m, LandmarkMap.h
        //   §108 — typical KLT max depth × 3). Same radius as the existing
        //   Phase 6.1 local-map tracking query (Tracker.cpp:3694) so the
        //   pre-filter and the tracking path agree on "near".
        // Age cap: 60 s. Older keyframes' poses may have drifted enough that
        //   the LandmarkMap association is stale; 60 s ≈ 60 m at 1 m/s walks.
        //   Plan-cited bound in Phase 6.4c contract; reuses the same value
        //   pattern as the Phase 6.1 local-map kLandmarkMaxAgeNs (30s) but
        //   relaxed for LC since LC queries on a longer cadence (1 Hz).
        static constexpr int64_t kLcSpatialMaxAgeNs = 60LL * 1'000'000'000LL;
        const cv::Vec3d lc_query_center = q_t_cam_world;
        const std::vector<uint64_t> spatial_candidate_kfs =
            landmark_map_.getKeyframesNearPosition(
                lc_query_center,
                navsight::LandmarkMap::kDefaultSearchRadiusM,
                q_ts_ns,
                kLcSpatialMaxAgeNs);

        ec.loop_closure_spatial_candidates_total.fetch_add(
            static_cast<long long>(spatial_candidate_kfs.size()),
            std::memory_order_relaxed);
        if (spatial_candidate_kfs.empty()) {
            ec.loop_closure_spatial_fallback_total.fetch_add(
                1, std::memory_order_relaxed);
        }
        LOGI("LC_SPATIAL: n_candidates=%zu radius=%.1f m",
             spatial_candidate_kfs.size(),
             navsight::LandmarkMap::kDefaultSearchRadiusM);

        LoopClosureDetector::LoopMatch match;
        const bool detected = loop_closure_.tryDetectLoopWithCandidates(
            static_cast<uint64_t>(q_kf_id),
            q_ts_ns,
            q_descriptors, q_keypoints,
            q_fx, q_fy, q_cx, q_cy,
            LOOP_CLOSURE_TEMPORAL_EXCL_NS,
            q_yaw_rad,
            spatial_candidate_kfs,
            match);

        if (!detected) {
            // 2026-05-04 cpp-reviewer HIGH-1 fix: detector now owns BOTH
            // rejection counters (rejects_low_score + rejects_pnp), so the
            // worker no longer bumps them. Otherwise PnP rejections were
            // double-counted (detector bumped rejects_pnp inside, then
            // worker bumped rejects_low_score on the false return). attempts
            // and accepts stay here at the worker tick boundary because
            // they are tick-level accounting, not per-rejection-reason.

            // ── Step 7.1 — Geometric loop closure (fallback) ─────────────
            // Try the direction-invariant path only when BoW didn't fire
            // on the same query. Same LoopMatch shape, same downstream
            // injection. Disabled when the publish-side didn't supply the
            // extras (corners empty or radius zero).
            // 2026-05-13 Step 7.1 fix: hand the query's ORB descriptors +
            // keypoints (the same matrices the BoW path consumes) to the
            // geom path. Replaces q_klt_corners — the prior pixel-only NN
            // admitted appearance-blind correspondences in low-texture
            // indoor scenes (v24 walk evidence). q_klt_corners stays
            // snapshotted because the SLAM/feature pipeline may still want
            // it; only the geom matcher's input changes.
            if (!q_descriptors.empty() &&
                !q_keypoints.empty() &&
                q_search_radius_m > 0.0 &&
                q_img_w > 0 && q_img_h > 0) {
                LoopClosureDetector::LoopMatch geom_match;
                const bool geom_ok = loop_closure_.tryDetectLoopGeometric(
                    static_cast<uint64_t>(q_kf_id),
                    q_ts_ns,
                    q_descriptors,
                    q_keypoints,
                    q_R_world_cam,
                    q_t_cam_world,
                    q_fx, q_fy, q_cx, q_cy,
                    q_img_w, q_img_h,
                    q_search_radius_m,
                    LOOP_CLOSURE_TEMPORAL_EXCL_NS,
                    geom_match);
                if (geom_ok) {
                    // Geometric accepts are bumped inside the detector
                    // (loop_closure_geom_accepts). Bump the unified
                    // worker-tick accept counter so loop_closure_accepts
                    // stays the ground truth for "any path accepted".
                    ec.loop_closure_accepts.fetch_add(
                        1, std::memory_order_relaxed);
                    LOGI("LOOP_CLOSURE: ACCEPT (geom) now_kf=%d match_kf=%llu inl=%d",
                         q_kf_id,
                         static_cast<unsigned long long>(geom_match.matched_kf_id),
                         geom_match.pnp_inliers);
                    std::lock_guard<std::mutex> rlock(loop_closure_result_mutex_);
                    loop_closure_pending_match_ = geom_match;
                    loop_closure_result_pending_ = true;
                    continue;
                }
            }
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

void Tracker::consumeLoopClosureMatchIfReady(IMUPreintegrator& imu) {
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

    // Stage 2 revert: damping ramp restored to original 10-frame schedule
    // for the EKF-channel updateAbsolutePose injection. With Stage 1's
    // gravity-alignment loop in place, p_G stays bounded by sensor
    // physics, so the chi²(0.999, 6) gate at EKFState.cpp:1000 has a
    // realistic residual to evaluate again.
    const int    k          = LOOP_CLOSURE_DAMPING_FRAMES -
                              loop_closure_damping_remaining_;  // 0..N-1
    const double strength   = 1.0 - static_cast<double>(k) /
                                    static_cast<double>(LOOP_CLOSURE_DAMPING_FRAMES);
    const double strength_sq = strength * strength;
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

    // 2026-05-09 v19 — principled chi² variance using the EKF's own tracked
    // covariance + PnP measurement noise.
    //
    // Pre-fix recipe: var_p = sigma_p_floor² × damping_inv where damping_inv
    // ramped 1 → 100 over 10 frames. The ramp inflated variance to LOOSEN
    // chi² so the gate would eventually accept — but the same inflation made
    // the Kalman gain K = P / (P + R) collapse to ~0 once R ≫ P. Net effect
    // (verified in v18 walk loop_house_x2_v18.json): chi² accepts at frames
    // 8–10 with `ok=1` logged, but actual cumulative correction across the
    // ramp was 1–2 m for a 15 m residual — not enough to overlay loop-2 on
    // loop-1. User reported "second loop doesn't match the first."
    //
    // The principled formulation (textbook Kalman filter with two correlated
    // pose estimates):
    //   r = target_p − p_G            (residual)
    //   var(r) = var(target_p) + var(p_G)
    //          = σ_pnp² + σ_p_ekf²
    // where σ_pnp is the PnP measurement accuracy floor (~2 m with ≥30
    // inliers at 3–5 m landmarks; cited in Tracker.h:553-557), and
    // σ_p_ekf is the EKF's tracked horizontal-position uncertainty —
    // P[12,12] (East variance) + P[13,13] (North variance) post Z-up
    // alignment. Sum-of-variances is the right additive form because the
    // two pose estimates are independent measurements of the same physical
    // location.
    //
    // No damping_inv needed because:
    //   - chi² evaluates the full residual against the natural variance
    //     budget — accepts/rejects on physical grounds, not artificial
    //     "easing" timeline.
    //   - Across the existing 10-frame ramp, residual SHRINKS each frame
    //     as the Kalman update applies a partial correction; this gives
    //     a natural exponential decay of the correction without any
    //     hand-coded damping schedule.
    //   - K = P_ekf / (P_ekf + σ_pnp²) stays at full physical Kalman gain
    //     instead of being driven to 0 by inflated R.
    //
    // Reading the EKF covariance: P[12,12] is East variance, P[13,13] is
    // North variance under the Z-up convention. The horizontal position
    // uncertainty is the trace of the 2×2 horizontal block.
    double sigma_p_ekf_sq = 0.0;
    double sigma_R_ekf_sq = 0.0;
    {
        cv::Mat P = ekf_.getCovariance();
        if (!P.empty() && P.rows >= 15 && P.type() == CV_64F) {
            sigma_p_ekf_sq = std::max(0.0, P.at<double>(12, 12)) +
                             std::max(0.0, P.at<double>(13, 13));
            // Rotation tracked uncertainty: sum of attitude diagonal
            // (rows 0,1,2 of the error-state vector are δθ_x, δθ_y, δθ_z).
            sigma_R_ekf_sq = std::max(0.0, P.at<double>(0, 0)) +
                             std::max(0.0, P.at<double>(1, 1)) +
                             std::max(0.0, P.at<double>(2, 2));
        }
    }
    // Phase 1 Step 3: drift-based position uncertainty.
    //
    // The v19 sum-of-variances `var_p = var_pnp + sigma_p_ekf_sq` decays
    // to ≈ var_pnp because v18's `setPosition(global_t_)` per frame
    // overrides p_G but doesn't touch P_pp; MSCKF visual updates then
    // continuously collapse P_pp. Verified in v19 walk
    // (loop_house_x2_v19.json): EKF P[12,12]+P[13,13] ≈ 0.0009 m² while
    // global_t_ had 5–15 m of accumulated drift; chi² m² hovered 23-24
    // (threshold 22.5) and only 6/213 attempts produced corrections.
    //
    // Replace with sum-of-variances using the LARGER of (a) EKF tracked
    // covariance and (b) integrated drift since last loop closure:
    //
    //     σ²_p_drift = (LOOP_CLOSURE_DRIFT_RATE × path_since_last_lc_m_)²
    //     σ²_p_total = σ²_p_pnp + max(σ²_p_ekf, σ²_p_drift)
    //
    // LOOP_CLOSURE_DRIFT_RATE = 0.032 m/m (Tracker.h:564), already in
    // tree, derived from sim_data drift measurement. path_since_last_lc_m_
    // is reset to 0 on every accepted loop closure (see ok-block below).
    // The max() picks the LARGER uncertainty source — covers both:
    //   * "EKF is genuinely uncertain" (after long visual-degenerate phase)
    //   * "EKF is overconfident due to recent MSCKF collapse"
    const double sigma_p_drift     = LOOP_CLOSURE_DRIFT_RATE * path_since_last_lc_m_;
    const double sigma_p_drift_sq  = sigma_p_drift * sigma_p_drift;
    const double sigma_p_used_sq   = std::max(sigma_p_ekf_sq, sigma_p_drift_sq);

    const double var_p_pnp     = LOOP_CLOSURE_PNP_SIGMA_FLOOR_M *
                                 LOOP_CLOSURE_PNP_SIGMA_FLOOR_M;     // ≈ 4 m²
    const double var_R_pnp     = LOOP_CLOSURE_BASE_ROT_SIGMA_RAD *
                                 LOOP_CLOSURE_BASE_ROT_SIGMA_RAD;    // ≈ 0.122 rad²
    const double var_p         = var_p_pnp + sigma_p_used_sq;
    const double sigma_axis_sq_R = var_R_pnp + sigma_R_ekf_sq / 3.0;  // per-axis avg
    (void)damping_inv;  // ramp counter still controls when to STOP; variance no longer scales

    // 2026-05-09 wiring fix: capture EKF p_G BEFORE updateAbsolutePose so we
    // can mirror the EKF-applied delta into global_t_ on success. Without this
    // mirror, loop-closure corrections accumulate in p_G_ (the EKF state) but
    // out.t = global_t_ at Tracker.cpp:2841 — meaning the user-facing trajectory
    // never sees a correction even when chi² accepts. The user reported exactly
    // this on v9 (loop_house_x2_v9.json: 18 PnP-accepts, 0 corrections_applied,
    // second-loop trajectory drifted away from the first because nothing
    // wrote into global_t_). Capturing the actual Kalman-applied delta keeps
    // both trajectories in sync without bypassing the chi² gate, and inherits
    // the 10-frame damping ramp's smoothness automatically (per-ramp-frame
    // delta is ~ strength × full_correction / 10).
    // 2026-05-18 architectural fix: capture EKF yaw before/after LC so we
    // can push the same Δyaw into Madgwick (user-visible heading source).
    // Without this, LC corrects EKF yaw but Madgwick keeps integrating gyro
    // independently, and the user-visible heading never benefits from LC
    // corrections. v44 walk showed EKF and Madgwick diverging by ~50° over
    // the walk (LC_YAW_FIRE residuals); this nudge keeps them in sync.
    // Use current Madgwick roll/pitch to extract a tilt-removed yaw — same
    // convention both Madgwick.getHeading() and EKF.getYaw() use.
    const double madg_roll  = imu.getMadgwickRoll();
    const double madg_pitch = imu.getMadgwickPitch();
    const double yaw_ekf_before = ekf_.getYaw(madg_roll, madg_pitch);
    cv::Mat p_G_before = ekf_.getPosition().clone();

    // 2026-05-21 BUG 3 — post-PnP rotation-residual sanity gate.
    //
    // Cause: the existing heading-gate at LoopClosureDetector.cpp:649
    // checks Madgwick yaw delta between candidate and current keyframes
    // BEFORE PnP runs. It does NOT verify the PnP-derived target_R against
    // current EKF R_GtoI. Planar scenes induce essential-matrix sign
    // ambiguity: PnP can return a target_R that is π-flipped from truth
    // even though the BoW score is good, the inlier count is high, and
    // the heading-gate at retrieval passes. EKF chi² then correctly
    // rejects all 10 damp-frame updates (wasted work), but if the
    // rotation residual is just-shy of the chi² threshold the wrong
    // target_R can leak into state.
    //
    // Evidence: 2026-05-20 walk parallax_fix_walk_2026_05_20 had one LC
    // accept at 15:03:36 with r_R = [2.821 0.750 0.418] (≈ 169° axis-angle
    // magnitude) and m²_R = 71 vs threshold 22.5 — chi² caught it ×10
    // damp frames. Across both 2026-05-20 walks the median r_R was 13°
    // and p99 was ~30°, so a 169° residual is a 5+ σ outlier under any
    // reasonable physical model.
    //
    // Change: compute |r_R| = ||Rodrigues(target_R_GtoI · R_current.t())||
    // directly here. If |r_R| > π/2 (90°), short-circuit updateAbsolutePose
    // to false. Threshold rationale: matches the existing heading-gate
    // (±π/2) at LoopClosureDetector.cpp retrieval — same physical intent
    // ("the user is not facing a wildly different direction from the
    // matched keyframe"), just applied to the geometric PnP solution
    // instead of the Madgwick yaw. Keeps everything coherent and gates
    // the right quantity at the right layer.
    //
    // Falsifier: post-fix walk →
    //   loop_closure_rejects_rot_sanity_total > 0 ONLY if a bad PnP
    //   solution arrived (which is rare); on a clean walk should stay
    //   at 0. If it fires on EVERY LC, the threshold is too tight (look
    //   for a real PnP rotation that's > 90° but legitimate — none seen
    //   in 4 walks so far, but worth checking).
    constexpr double kLcPostPnpMaxRotResidualRad = M_PI / 2.0;  // 90°
    bool rot_sanity_ok = true;
    {
        cv::Mat R_current = ekf_.getRotation();
        if (!R_current.empty() && R_current.rows == 3 && R_current.cols == 3 &&
            R_current.type() == CV_64F) {
            cv::Mat R_delta = target_R_GtoI * R_current.t();
            cv::Mat r_axis;
            cv::Rodrigues(R_delta, r_axis);
            const double r_mag = cv::norm(r_axis);
            if (std::isfinite(r_mag) && r_mag > kLcPostPnpMaxRotResidualRad) {
                navsight::eventCounters()
                    .loop_closure_rejects_rot_sanity_total
                    .fetch_add(1, std::memory_order_relaxed);
                if (fresh_match_picked_up || k == 0) {
                    LOGI("LC_POST_PNP_ROT_REJECT: |r_R|=%.1f° threshold=%.1f° "
                         "match_kf=%d (PnP target is π-flipped or essentially "
                         "rotated from current EKF; aborting damp ramp)",
                         r_mag * 180.0 / M_PI,
                         kLcPostPnpMaxRotResidualRad * 180.0 / M_PI,
                         matched_clone_id);
                }
                rot_sanity_ok = false;
            }
        }
    }

    // 2026-05-24 BUG (LC heading) — inject world-Z yaw uncertainty so the LC
    // rotation update can actually MOVE the heading. ROOT CAUSE (code-verified):
    // VIO yaw is unobservable and P[2,2] collapses via MSCKF/landmark updates,
    // so the LC yaw gain K ≈ P[2,2]/(P[2,2]+σ²_R) → 0; the EKF yaw barely moved,
    // delta_yaw_nav ≈ 0, and nudgeMadgwickYawAroundWorldZ (user-visible heading)
    // was skipped (two-loop walk 2026-05-24: 30 corrections, loop still 9.72 m
    // open). Inject once per accept (k==0), only when about to apply the update
    // (rot_sanity_ok), with the drift-based heading variance accrued since the
    // last LC, capped. Rotation analog of the existing sigma_p_drift floor.
    if (rot_sanity_ok && k == 0) {
        double sigma_yaw = LOOP_CLOSURE_HEADING_DRIFT_RATE_RAD_PER_M *
                           path_since_last_lc_m_;
        if (sigma_yaw > LOOP_CLOSURE_HEADING_MAX_SIGMA_RAD) {
            sigma_yaw = LOOP_CLOSURE_HEADING_MAX_SIGMA_RAD;
        }
        const double var_yaw_inject = sigma_yaw * sigma_yaw;
        if (var_yaw_inject > 1e-9) {
            const double p_yaw_before = ekf_.getYawVariance();
            ekf_.addYawUncertainty(var_yaw_inject);
            LOGI("LC_YAW_INJECT: path_since_lc=%.1fm sigma_yaw=%.1fdeg "
                 "P_yaw: %.4e -> %.4e rad^2 (enabling LC heading gain)",
                 path_since_last_lc_m_, sigma_yaw * 180.0 / M_PI,
                 p_yaw_before, ekf_.getYawVariance());
        }
    }

    const bool ok = rot_sanity_ok && ekf_.updateAbsolutePose(
                                         target_R_GtoI, target_p_world,
                                         sigma_axis_sq_R, var_p);
    if (ok) {
        const double yaw_ekf_after = ekf_.getYaw(madg_roll, madg_pitch);
        double delta_yaw_nav = yaw_ekf_after - yaw_ekf_before;
        while (delta_yaw_nav >  M_PI) delta_yaw_nav -= 2.0 * M_PI;
        while (delta_yaw_nav < -M_PI) delta_yaw_nav += 2.0 * M_PI;
        // 2026-05-25 RV-only heading: loop closure must NOT move the displayed
        // heading while the rotation-vector is the active source (it still
        // corrects EKF position via updateAbsolutePose above). Only nudge
        // Madgwick yaw when RV is unavailable (fallback).
        if (std::abs(delta_yaw_nav) > 1e-5 && !imu.isMagActivelyFusing()) {
            imu.nudgeMadgwickYawAroundWorldZ(delta_yaw_nav);
            LOGI("LC_MADG_NUDGE: yaw_before=%.2f° yaw_after=%.2f° delta=%+.2f° "
                 "(Madgwick rotated by same delta around world-Z)",
                 yaw_ekf_before * 180.0 / M_PI,
                 yaw_ekf_after  * 180.0 / M_PI,
                 delta_yaw_nav  * 180.0 / M_PI);
        }
        cv::Mat p_G_after = ekf_.getPosition();
        cv::Mat delta_p   = p_G_after - p_G_before;      // 3×1 world frame
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!global_t_.empty() && global_t_.rows == 3 &&
                global_t_.cols == 1 && global_t_.type() == CV_64F) {
                global_t_ += delta_p;
            }
        }
        navsight::eventCounters().loop_closure_corrections_applied.fetch_add(
            1, std::memory_order_relaxed);

        // Phase 1 Step 3: reset the drift estimate on accepted correction.
        // The next chi² evaluation starts with σ²_p_drift = 0 and grows
        // again as the user walks. Reset only on the FIRST frame of a
        // damping ramp (k == 0) so partial corrections inside the same
        // ramp don't reset prematurely — at frame k=0 we just transitioned
        // from "drifted" to "corrected" for this revisit, frames k=1..9
        // are smoothing tail.
        if (k == 0) {
            path_since_last_lc_m_ = 0.0;

            // 2026-05-19 Fix #9 — re-anchor LandmarkMap entries from the
            // corrected EKF clone poses. See LandmarkMap.h
            // `reanchorLandmarksFromClonePoses` for the cause/change
            // writeup. In short: the EKF clones just shifted via
            // updateAbsolutePose's covariance propagation, but each
            // landmark's `p_world` is still the snapshot from its host
            // clone's PRE-correction pose. Recompute `p_world` from each
            // landmark's stored `p_anchor_cam` against the host clone's
            // CURRENT pose, so the dots project at their visible features
            // on the very next render frame instead of waiting for a
            // re-observation.
            //
            // The EKF stores `R_GtoC` (world→camera) in each clone. The
            // LandmarkMap math expects `R_world_cam` (camera→world), so
            // we transpose at the boundary. Camera position in world
            // ≈ p_G (lever-arm to IMU centre is ≪ feature depth — same
            // approximation as native-lib.cpp getCurrentCameraPose).
            const int n_reanchored =
                landmark_map_.reanchorLandmarksFromClonePoses(
                    [this](int clone_id,
                            cv::Matx33d& R_wc_out,
                            cv::Vec3d&   t_cw_out) -> bool {
                        cv::Mat R_GtoC_mat, p_G_mat;
                        if (!ekf_.getClonePose(clone_id, R_GtoC_mat,
                                                p_G_mat)) {
                            return false;
                        }
                        if (R_GtoC_mat.empty() ||
                            R_GtoC_mat.rows != 3 ||
                            R_GtoC_mat.cols != 3 ||
                            p_G_mat.rows != 3 || p_G_mat.cols != 1) {
                            return false;
                        }
                        const cv::Mat R_cw_mat = R_GtoC_mat.t();
                        for (int r = 0; r < 3; ++r) {
                            for (int c = 0; c < 3; ++c) {
                                R_wc_out(r, c) = R_cw_mat.at<double>(r, c);
                            }
                        }
                        t_cw_out = cv::Vec3d(p_G_mat.at<double>(0, 0),
                                              p_G_mat.at<double>(1, 0),
                                              p_G_mat.at<double>(2, 0));
                        return true;
                    });
            if (n_reanchored > 0) {
                navsight::eventCounters()
                    .landmarks_reanchored_total.fetch_add(
                        n_reanchored, std::memory_order_relaxed);
            }
            LOGI("LC_REANCHOR: n_reanchored=%d (LC accepted, "
                 "landmarks shifted from stale to current clone poses)",
                 n_reanchored);

            // 2026-05-13 Phase 1 Step 5: pose-graph optimization fires once
            // per LC accept (first damp frame). The EKF updateAbsolutePose
            // above already snapped p_G/R_GtoI for the current pose; the
            // pose graph's job is to redistribute the same loop constraint
            // across all keyframes between K_match and K_now so future
            // LC operations see a corrected reference chain.
            //
            // Implementor-skill scope note: the back-write to
            // LoopClosureDetector's stored keyframe poses (which would let
            // loop 2 visually overlay loop 1 in the rendered trajectory)
            // requires a new LoopClosureDetector::updateStoredKeyframePose
            // method and is the next chunk of Step 5. For now optimize()
            // runs and publishes residual_pre/_post counters; the math
            // gets validated by event_summary diff before the back-write
            // reaches the renderer.
            int match_pg = -1;
            auto it_match = clone_id_to_pg_node_.find(
                static_cast<uint64_t>(matched_clone_id));
            if (it_match != clone_id_to_pg_node_.end()) {
                match_pg = it_match->second;
            }
            const int now_pg = last_pg_node_id_;

            if (match_pg >= 0 && now_pg >= 0 && match_pg != now_pg) {
                double mx, my, mz, myaw;
                double nx, ny, nz, nyaw;
                const bool got_m =
                    pose_graph_.getNode(match_pg, mx, my, mz, myaw);
                const bool got_n =
                    pose_graph_.getNode(now_pg,   nx, ny, nz, nyaw);
                if (got_m && got_n) {
                    // Pre-fix symptom log per implementor skill §5. After
                    // the LC-DB back-write lands in the next chunk this
                    // same line will gain a `gap_after_optimize_m` field
                    // and we can verify the optimizer reduces the gap on
                    // real walks.
                    const double gap_x = nx - mx;
                    const double gap_y = ny - my;
                    const double gap_z = nz - mz;
                    const double gap_m = std::sqrt(
                        gap_x * gap_x + gap_y * gap_y + gap_z * gap_z);
                    const int intermediate_n =
                        std::abs(now_pg - match_pg) - 1;
                    LOGI("LC_TRAJECTORY_GAP_PRE: match_pg=%d now_pg=%d "
                         "match_p=[%.3f %.3f %.3f] now_p=[%.3f %.3f %.3f] "
                         "gap_m=%.3f intermediate_n=%d",
                         match_pg, now_pg, mx, my, mz, nx, ny, nz,
                         gap_m, intermediate_n);

                    // Loop edge measurement: in match's yawed frame, where
                    // is the now-keyframe per the EKF-derived target?
                    const double dx_w =
                        target_p_world.at<double>(0) - mx;
                    const double dy_w =
                        target_p_world.at<double>(1) - my;
                    const double dz_w =
                        target_p_world.at<double>(2) - mz;
                    const double cm = std::cos(myaw);
                    const double sm = std::sin(myaw);
                    const double dx_loop =  cm * dx_w + sm * dy_w;
                    const double dy_loop = -sm * dx_w + cm * dy_w;
                    const double dz_loop =  dz_w;

                    // 2026-05-16 v26 walk root-cause fix:
                    // Prior formula `dyaw_loop = target_yaw − myaw` mixed
                    // two different state sources:
                    //   - target_yaw: extracted from target_R_GtoI (EKF state)
                    //   - myaw:        Madgwick scalar_heading_ stored at
                    //                  match-kf creation (in pose-graph node)
                    // The pose-graph optimizer's pred_dyaw uses Madgwick
                    // at BOTH endpoints (now_pg.yaw and match_pg.yaw both
                    // come from scalar_heading_ at addNode time). So:
                    //   residual = pred_dyaw − dyaw_loop
                    //            = (Madg_now − Madg_match)
                    //              − (EKF_now − Madg_match)
                    //            = Madg_now − EKF_now
                    // This is the EKF-vs-Madgwick yaw disagreement, not
                    // the visual loop-closure measurement. v26 walk
                    // evidence: dyaw drifted from 2.67° at first LC to
                    // 55° by 53s later as EKF R_GtoI (corrupted by
                    // MSCKF visual updates with 2270 Huber rejects)
                    // diverged from Madgwick. The optimizer then tried
                    // to "fix" the spurious 55° yaw error by rotating
                    // nodes → trajectory drift / 8m loop gap.
                    //
                    // Correct construction: derive dyaw_loop purely from
                    // the PnP chain using world-frame camera headings.
                    // R_wc_match (stored at match time) and
                    // target_R_world_cam = R_wc_match · R_n2m^{-1} are
                    // both cam→world matrices. The camera forward in world
                    // is each matrix's third column. Heading is nav-CW
                    // atan2(fwd_x, fwd_y) — same convention as
                    // Madgwick's getHeading. dyaw_loop becomes the
                    // visual-PnP-measured yaw change, independent of EKF
                    // R_GtoI state evolution.
                    auto camWorldHeading = [](const cv::Matx33d& R_wc) {
                        // Camera forward in world = R_wc * (0,0,1)
                        // = R_wc's third column. Nav-CW heading from
                        // world +Y (north). atan2(x, y) gives angle
                        // measured CW from +Y, matching Madgwick.
                        return std::atan2(R_wc(0, 2), R_wc(1, 2));
                    };
                    const double match_world_heading =
                        camWorldHeading(R_wc_match);
                    const double now_world_heading =
                        camWorldHeading(target_R_world_cam);
                    double dyaw_loop = now_world_heading - match_world_heading;
                    while (dyaw_loop >  M_PI) dyaw_loop -= 2.0 * M_PI;
                    while (dyaw_loop < -M_PI) dyaw_loop += 2.0 * M_PI;

                    // Per implementor-skill verification log: capture
                    // BOTH the new PnP-derived dyaw AND the Madgwick
                    // pred_dyaw so we can confirm on next walk that they
                    // agree (within visual measurement noise) and that
                    // dyaw_loop no longer accumulates the EKF-vs-Madgwick
                    // disagreement seen in v26.
                    LOGI("POSE_GRAPH_YAW_FIX_VERIFY: dyaw_loop_deg=%.2f "
                         "match_hdg_deg=%.2f now_hdg_deg=%.2f "
                         "myaw_deg=%.2f (pred=now_pg.yaw-match_pg.yaw)",
                         dyaw_loop * 180.0 / M_PI,
                         match_world_heading * 180.0 / M_PI,
                         now_world_heading * 180.0 / M_PI,
                         myaw * 180.0 / M_PI);

                    // 2026-05-13 Step 5 plan-compliance (line 188): Σ_loop
                    // MUST come from var_p_total (Step 3's PnP + EKF + drift
                    // budget). var_p is built at Tracker.cpp:4288 as
                    // var_p_pnp + sigma_p_used_sq. sigma_axis_sq_R built at
                    // Tracker.cpp:4289 carries the per-axis rotation
                    // variance, so info_yaw = 1/sigma_axis_sq_R uses the
                    // same gauge as the EKF chi² gate consumes. Floors
                    // protect against numerical pathologies (var_p_total
                    // can be tiny in a clean LC; the floor matches the
                    // PoseGraph's own SIGMA_*_FLOOR_SQ).
                    // 2026-05-25 — loop EDGE weight = PnP measurement precision
                    // (LOOP_CLOSURE_EDGE_SIGMA_*), NOT the drift-inflated var_p /
                    // sigma_axis_sq_R the chi² gate consumes. See Tracker.h
                    // derivation: the old var_p-based weight made the loop edge
                    // ~1600× weaker than odometry so optimize() never closed the
                    // loop (residual ratio 0.99). The chi² gate above is unchanged
                    // (still var_p / sigma_axis_sq_R) so acceptance still tolerates
                    // the large pre-correction gap; only the EDGE WEIGHT changes.
                    const double edge_var_p   = LOOP_CLOSURE_EDGE_SIGMA_P_M *
                                                LOOP_CLOSURE_EDGE_SIGMA_P_M;
                    const double edge_var_yaw = LOOP_CLOSURE_EDGE_SIGMA_YAW_RAD *
                                                LOOP_CLOSURE_EDGE_SIGMA_YAW_RAD;
                    const double lc_info_xy  = 1.0 /
                        std::max(edge_var_p,   PoseGraph::SIGMA_POS_FLOOR_SQ);
                    const double lc_info_z   = lc_info_xy;  // isotropic
                    const double lc_info_yaw = 1.0 /
                        std::max(edge_var_yaw, PoseGraph::SIGMA_YAW_FLOOR_SQ);

                    // 2026-05-30 — POSE-GRAPH INCOHERENCE GUARD. ROOT CAUSE: the
                    // auto-odom edges are built from ABSOLUTE node-position diffs
                    // (PoseGraph.cpp:47 `dx = x - prev.x`), so every EKF loop-
                    // closure teleport (`global_t_ += delta_p`, updateAbsolutePose
                    // above, + the pose-graph now-correction below) gets BAKED into
                    // the chain as fake physical motion. The frozen-node chain then
                    // disagrees with the PnP loop measurement by FAR more than real
                    // drift, and optimize() smears a physically-wrong warp across
                    // every node which the UI redraws as the whole trajectory.
                    // House-loop 2026-05-30: 47.9 m loop-edge residual on a 100 m
                    // loop -> 4.7 m + 12.3deg warp (pose_graph_max_correction_mm=4691).
                    // A TRUE closure's residual ~= accumulated drift
                    // (LOOP_CLOSURE_DRIFT_RATE x path); 5x that is the outlier
                    // boundary (~16% of path — beyond it the constraint is frame
                    // incoherence or a false match, not drift). On reject we SKIP
                    // optimize+apply+redraw; the EKF direct correction already moved
                    // the dot, so the trajectory stays on the clean EKF-corrected
                    // path (the v54 behaviour where the pose-graph was inert and
                    // loops still overlaid). HEADING-SAFE: scalar_heading_/Madgwick
                    // is untouched. The deeper root fix (teleport-free odom edges)
                    // is tracked separately; this guard makes the warp impossible
                    // regardless.
                    double pg_mx, pg_my, pg_mz, pg_myaw;
                    double pg_nx, pg_ny, pg_nz, pg_nyaw;
                    const bool pg_got =
                        pose_graph_.getNode(match_pg, pg_mx, pg_my, pg_mz, pg_myaw) &&
                        pose_graph_.getNode(now_pg,   pg_nx, pg_ny, pg_nz, pg_nyaw);
                    double pg_lc_resid = 0.0;
                    if (pg_got) {
                        const double ddx = pg_nx - pg_mx;
                        const double ddy = pg_ny - pg_my;
                        const double ddz = pg_nz - pg_mz;
                        const double cmp = std::cos(pg_myaw);
                        const double smp = std::sin(pg_myaw);
                        const double pred_dx =  cmp * ddx + smp * ddy;
                        const double pred_dy = -smp * ddx + cmp * ddy;
                        pg_lc_resid = std::sqrt(
                            (pred_dx - dx_loop) * (pred_dx - dx_loop) +
                            (pred_dy - dy_loop) * (pred_dy - dy_loop) +
                            (ddz     - dz_loop) * (ddz     - dz_loop));
                    }
                    const double pg_expected_drift =
                        std::max(LOOP_CLOSURE_DRIFT_RATE * total_path_m_,
                                 LOOP_CLOSURE_PNP_SIGMA_FLOOR_M);
                    constexpr double kPgIncoherenceFactor = 5.0;  // outlier boundary
                    const bool pg_coherent =
                        pg_got &&
                        (pg_lc_resid <= kPgIncoherenceFactor * pg_expected_drift);
                    if (!pg_coherent) {
                        navsight::eventCounters().pose_graph_rejected_incoherent
                            .fetch_add(1, std::memory_order_relaxed);
                        LOGI("POSE_GRAPH_REJECT_INCOHERENT: lc_resid=%.2fm "
                             "expected_drift=%.2fm thresh=%.2fm path=%.1fm "
                             "match_pg=%d now_pg=%d got=%d "
                             "(frozen-node frame incoherence / false match -> "
                             "skip optimize+redraw; EKF direct correction stands)",
                             pg_lc_resid, pg_expected_drift,
                             kPgIncoherenceFactor * pg_expected_drift,
                             total_path_m_, match_pg, now_pg, pg_got ? 1 : 0);
                    } else {
                    pose_graph_.addLoopEdge(match_pg, now_pg,
                                             dx_loop, dy_loop,
                                             dz_loop, dyaw_loop,
                                             lc_info_xy, lc_info_z, lc_info_yaw);
                    LOGI("POSE_GRAPH_LOOP_EDGE: match_pg=%d now_pg=%d "
                         "dx=%.3f dy=%.3f dz=%.3f dyaw_deg=%.2f "
                         "var_p=%.4f var_R=%.4e "
                         "info_xy=%.2f info_yaw=%.2f",
                         match_pg, now_pg,
                         dx_loop, dy_loop, dz_loop, dyaw_loop * 180.0 / M_PI,
                         var_p, sigma_axis_sq_R,
                         lc_info_xy, lc_info_yaw);
                    const double max_corr = pose_graph_.optimize();

                    // 2026-05-13 Phase 1 Step 5 closure: LC DB back-write.
                    // Iterates the optimized PoseGraph snapshot, applies
                    // per-node Δp/Δyaw to the stored keyframe DB via
                    // loop_closure_.applyKeyframePoseCorrection so future
                    // BoW matches resolve against corrected poses. Applies
                    // the now-node's Δp to global_t_ under pose_mutex_ so
                    // the legacy pose mirror reflects the correction
                    // visible in the trajectory. Companion to Step 7.1 fix
                    // above — Step 7.1 ensures the loop edge fed into
                    // pose_graph_.addLoopEdge is appearance-verified, so
                    // back-writes propagate clean corrections, not the
                    // appearance-blind PnP failures seen in v24.
                    auto& ec_pg = navsight::eventCounters();
                    const auto snap = pose_graph_.snapshotNodes();
                    int applied = 0;
                    double now_dx = 0.0, now_dy = 0.0, now_dz = 0.0;
                    double now_dyaw = 0.0;
                    for (const auto& node : snap) {
                        if (node.id == 0) continue;
                        const double pdx   = node.x   - node.x0;
                        const double pdy   = node.y   - node.y0;
                        const double pdz   = node.z   - node.z0;
                        double       pdyaw = node.yaw - node.yaw0;
                        while (pdyaw >  M_PI) pdyaw -= 2.0 * M_PI;
                        while (pdyaw < -M_PI) pdyaw += 2.0 * M_PI;
                        const double pmag2 = pdx*pdx + pdy*pdy + pdz*pdz;
                        if (pmag2 < 1e-6 && std::abs(pdyaw) < 1e-4) continue;
                        auto it_cid = pg_node_to_clone_id_.find(node.id);
                        if (it_cid == pg_node_to_clone_id_.end()) continue;
                        const uint64_t kf_id = it_cid->second;
                        if (loop_closure_.applyKeyframePoseCorrection(
                                kf_id, pdx, pdy, pdz, pdyaw)) {
                            ++applied;
                            ec_pg.pose_graph_apply_calls.fetch_add(
                                1, std::memory_order_relaxed);
                        }
                        if (node.id == now_pg) {
                            now_dx = pdx; now_dy = pdy;
                            now_dz = pdz; now_dyaw = pdyaw;
                        }
                    }
                    if (now_pg >= 0 &&
                        (std::abs(now_dx) > 1e-4 ||
                         std::abs(now_dy) > 1e-4 ||
                         std::abs(now_dz) > 1e-4)) {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        if (!global_t_.empty() && global_t_.rows == 3 &&
                            global_t_.cols == 1 &&
                            global_t_.type() == CV_64F) {
                            global_t_.at<double>(0) += now_dx;
                            global_t_.at<double>(1) += now_dy;
                            global_t_.at<double>(2) += now_dz;
                        }
                    }
                    LOGI("POSE_GRAPH_APPLIED: applied=%d/%d max_corr_m=%.3f "
                         "now_dx=%.3f now_dy=%.3f now_dz=%.3f "
                         "now_dyaw_deg=%.2f",
                         applied, static_cast<int>(snap.size()) - 1,
                         max_corr,
                         now_dx, now_dy, now_dz,
                         now_dyaw * 180.0 / M_PI);

                    // 2026-05-26 — #2 loop-overlay path redraw: snapshot the
                    // CORRECTED node polyline (pn.x=East, pn.y=North; same ground
                    // frame as VioData x/z) so the UI can rebuild its drifted
                    // pathHistory. The apply loop above only moved the now-node
                    // into global_t_; the PAST trajectory is redrawn from here on
                    // the loop_correction_version_ bump.
                    {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        corrected_traj_xz_.clear();
                        corrected_traj_xz_.reserve(snap.size());
                        for (const auto& pn : snap) {
                            corrected_traj_xz_.emplace_back(
                                static_cast<float>(pn.x),
                                static_cast<float>(pn.y));
                        }
                    }
                    loop_correction_version_.fetch_add(1, std::memory_order_relaxed);
                    }  // end else (pg_coherent) — 2026-05-30 incoherence guard
                }
            } else {
                LOGI("LC_TRAJECTORY_GAP_SKIP: reason=no_pg_node "
                     "match_pg=%d now_pg=%d matched_clone_id=%d",
                     match_pg, now_pg, matched_clone_id);
            }
        }
    }

    LOGI("LOOP_CLOSURE: damp k=%d/%d strength=%.2f var_p=%.4f "
         "(pnp=%.4f drift_path=%.2fm drift²=%.4f ekf²=%.4f) var_R=%.4e "
         "target_p=[%.3f %.3f %.3f] match_kf=%d ok=%d (abs-pose channel)",
         k + 1, LOOP_CLOSURE_DAMPING_FRAMES, strength,
         var_p,
         var_p_pnp, path_since_last_lc_m_, sigma_p_drift_sq, sigma_p_ekf_sq,
         sigma_axis_sq_R,
         target_p_world.at<double>(0),
         target_p_world.at<double>(1),
         target_p_world.at<double>(2),
         matched_clone_id,
         ok ? 1 : 0);
    (void)target_R_world_cam;
    (void)R_bc_lc;
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

void Tracker::updateExpansionSpeed(const std::vector<cv::Point2f>& prev_ud,
                                  const std::vector<cv::Point2f>& next_ud,
                                  double dt_s,
                                  const cv::Vec3d& gyro_rot_cam,
                                  const IMUPreintegrator& imu, double gyro_norm) {
    // ── Forward speed from optical-flow LOOMING (per-point Vz = (ṙ/r)·Z_rel·K) ─────
    // Per the research (Koenderink-van Doorn 1987, Nelson-Aloimonos 1989,
    // Longuet-Higgins-Prazdny 1980, Heeger-Jepson 1992): the radial component of the
    // DE-ROTATED translational flow gives forward speed / depth directly. Robust for
    // forward motion exactly where the essential matrix (recoverPose) degenerates.
    // We DE-ROTATE the flow with the gyro (mandatory — head turns would otherwise
    // look like forward speed), anchor the FOE to the EKF heading (well-conditioned
    // even when free-FOE estimation breaks down for pure-forward motion), and FUSE
    // into depth_flow_speed_mps_ by the forward-motion fraction so the UI sees a
    // smooth blend (looming-dominant when forward, recoverPose-dominant sideways).
    if (dt_s <= 1e-4 || fx_ <= 0.0 || fy_ <= 0.0) return;

    // ── Scale fix Step 2 — classify gait BEFORE loading/using cur_k below, so the
    //    looming K calib reads/writes THIS gait's slot (expansion_scale_K_ == active
    //    mode's loom K). accel_speed = horizontal world-frame accel velocity (same
    //    quantity used in updateDepthFlowSpeed / ACCEL_K_STATE). May onModeSwitch().
    const double accel_speed = std::hypot(accel_vel_w_[0], accel_vel_w_[1]);  // m/s
    active_mode_ = classifyGait(imu, accel_speed);

    // 2026-05-28 — K bail REMOVED here so the per-point loop can produce a
    // K-INDEPENDENT relative speed (vz_rel = tau * Z_rel). That value lets us
    // bootstrap K from accel-distance correlation BELOW, breaking the
    // chicken-and-egg where K only calibrated inside updateDepthFlowSpeed
    // (which requires essential-matrix verification to pass — never true on
    // slow walks). With the bootstrap, K calibrates from any forward-motion
    // window regardless of essential-matrix success, so cold-start walks
    // produce metric speed on the first stop→go after the K-calibration
    // window opens.

    // 1. Focus of Expansion (FOE) from the EKF heading projected to image plane.
    //    Forward motion is assumed along the world-frame heading direction (Z-up ENU).
    const cv::Matx33d R_GtoI = ekf_.getRotation();
    const cv::Matx33d R_bc   = ekf_.getExtrinsicsRotation();
    const cv::Matx33d R_GtoC = R_bc * R_GtoI;
    const cv::Vec3d fwd_w(std::sin(scalar_heading_), std::cos(scalar_heading_), 0.0);
    const cv::Vec3d fwd_c = R_GtoC * fwd_w;
    if (std::abs(fwd_c(2)) < 0.1) return;   // motion ⊥ to viewing axis — no expansion
    const double u_foe = cx_ + fx_ * (fwd_c(0) / fwd_c(2));
    const double v_foe = cy_ + fy_ * (fwd_c(1) / fwd_c(2));

    const double wx = gyro_rot_cam[0];
    const double wy = gyro_rot_cam[1];
    const double wz = gyro_rot_cam[2];

    // 2026-05-29 — Step A per-point affine DISABLED on the looming speed path,
    // same reason as updateDepthFlowSpeed: the affine (s,t) is anchored to
    // scale_fuser_ (~0.10, the rejected-PDR seed) so its "metric" depth is ~5x
    // too small, and it degenerates to flat depth (s≈0) on noisy VIO targets.
    // The accel-K path is the single metric anchor. Looming still computes the
    // K-independent vz_rel = tau * Z_rel and feeds the shared K bootstrap; the
    // metric conversion is K * vz_rel (Path B below). Snapshot kept commented
    // for revert if a non-scale_fuser per-point metric depth is ever wired.
    /* LEGACY 2026-05-28 Step A affine snapshot (scale_fuser-poisoned):
    double s_aff = 0.0;
    double t_aff = 0.0;
    bool   aff_valid = false;
    {
        std::lock_guard<std::mutex> lock(midas_affine_mutex_);
        s_aff = midas_affine_s_;
        t_aff = midas_affine_t_;
        aff_valid = midas_affine_valid_;
    }
    */
    constexpr bool aff_valid = false;       // affine off the speed path (see above)
    const double s_aff = 0.0, t_aff = 0.0;  // logged only; unused while aff_valid=false

    // 2. Per-point expansion rate from de-rotated radial flow.
    //    K-INDEPENDENT: vz_rel_i = tau * Z_rel; vz_metric_i = vz_rel_i * K.
    //    AFFINE: vz_metric_i = tau / (s * disp + t) — computed in parallel below.
    const size_t n = std::min(prev_ud.size(), next_ud.size());
    std::vector<double> vz_rel_estimates;     // K-independent: tau * Z_rel
    std::vector<double> rho_values;           // aligned with vz_rel_estimates
    std::vector<double> vz_metric_aff_est;    // affine direct metric: tau / (s*disp+t)
    std::vector<double> rho_aff_values;       // aligned with vz_metric_aff_est
    vz_rel_estimates.reserve(n);
    rho_values.reserve(n);
    vz_metric_aff_est.reserve(n);
    rho_aff_values.reserve(n);
    int total_valid = 0;     // points that produced a depth + flow measurement
    int positive    = 0;     // ... of which, how many were expanding outward (forward)
    constexpr double kRhoMin  = 0.05;   // mask near-FOE noise blow-up (research)
    constexpr double kRhoMax  = 0.80;   // mask peripheral lens distortion
    constexpr double kDispMin = 0.02;   // far-point / sky cutoff (raw MiDaS disparity)
    for (size_t i = 0; i < n; ++i) {
        // Distance from FOE in normalized image coords (for radial / r_dot).
        const double dx = (prev_ud[i].x - u_foe) / fx_;
        const double dy = (prev_ud[i].y - v_foe) / fy_;
        const double rho = std::sqrt(dx * dx + dy * dy);
        if (rho < kRhoMin || rho > kRhoMax) continue;

        // De-rotate the flow using gyro angular displacement in camera frame.
        // Heeger-Jepson 1992 with focal length absorbed into normalized coords (f=1):
        //   u_rot = x·y·wx − (1+x²)·wy + y·wz
        //   v_rot = (1+y²)·wx − x·y·wy − x·wz
        // x, y are normalized prev coords *from the principal point* (not the FOE —
        // Heeger-Jepson is derived about the principal point).
        const double xp = (prev_ud[i].x - cx_) / fx_;
        const double yp = (prev_ud[i].y - cy_) / fy_;
        const double u_rot = xp * yp * wx - (1.0 + xp * xp) * wy + yp * wz;
        const double v_rot = (1.0 + yp * yp) * wx - xp * yp * wy - xp * wz;
        const double du = (next_ud[i].x - prev_ud[i].x) / fx_ - u_rot;
        const double dv = (next_ud[i].y - prev_ud[i].y) / fy_ - v_rot;

        // Radial component of the (de-rotated) translational flow.
        const double drho = (dx * du + dy * dv) / rho;
        const double tau  = drho / (rho * dt_s);   // expansion rate, 1/s (= Vz/Z)

        double disp_raw = 0.0;
        if (!sampleMidasRawDisparity(prev_ud[i].x, prev_ud[i].y, disp_raw) ||
            disp_raw < kDispMin) continue;
        const double Z_rel = 1.0 / disp_raw;
        ++total_valid;
        if (tau > 0.0) ++positive;

        const double vz_rel_i = tau * Z_rel;   // K-independent
        // Sanity: finite & positive. The metric upper bound (30 m/s) is applied
        // after multiplying by K below.
        if (std::isfinite(vz_rel_i) && vz_rel_i > 0.0) {
            vz_rel_estimates.push_back(vz_rel_i);
            rho_values.push_back(rho);
        }
        // 2026-05-28 (Step A) — affine direct metric (per-point, scene-invariant).
        if (aff_valid) {
            const double inv_metric = s_aff * disp_raw + t_aff;
            if (inv_metric > 1e-4) {
                const double vz_metric_aff_i = tau / inv_metric;
                if (std::isfinite(vz_metric_aff_i) &&
                    vz_metric_aff_i > 0.0 && vz_metric_aff_i < 30.0) {
                    vz_metric_aff_est.push_back(vz_metric_aff_i);
                    rho_aff_values.push_back(rho);
                }
            }
        }
    }

    constexpr size_t kMinPts = 10;
    if (vz_rel_estimates.size() < kMinPts) {
        navsight::eventCounters().depth_flow_looming_skipped.fetch_add(
            1, std::memory_order_relaxed);
        return;
    }

    // 3. Robust median + ρ²-weighted inlier mean of the RELATIVE vz.
    std::sort(vz_rel_estimates.begin(), vz_rel_estimates.end());
    const double vz_rel_med = vz_rel_estimates[vz_rel_estimates.size() / 2];
    double sum_num = 0.0, sum_den = 0.0;
    int n_inliers = 0;
    const double tol_rel = 0.3 * std::abs(vz_rel_med) + 1e-4;
    for (size_t i = 0; i < vz_rel_estimates.size(); ++i) {
        if (std::abs(vz_rel_estimates[i] - vz_rel_med) < tol_rel) {
            const double w = rho_values[i] * rho_values[i];
            sum_num += w * vz_rel_estimates[i];
            sum_den += w;
            ++n_inliers;
        }
    }
    const double vz_rel_fused = (sum_den > 0.0) ? (sum_num / sum_den) : vz_rel_med;
    const double forward_fraction_early = (total_valid > 0)
        ? (static_cast<double>(positive) / total_valid) : 0.0;

    // 4. K BOOTSTRAP from looming — calibrates LOOMING's OWN K (expansion_scale_K_),
    //    NOT the depth-flow K. Looming fires on ALL locomotion (outside the
    //    essential-matrix verification gate), so on slow walks (where depth-flow's
    //    recoverPose path bails) this is the path that calibrates + drives speed.
    //    Its relative measure is vz_rel = tau*Z_rel — a different basis from
    //    depth-flow's disp_rel, so it keeps a SEPARATE accumulator
    //    (visual_rel_dist_loom_) and a SEPARATE K to stay self-consistent (no
    //    cross-basis mismatch, no shared-accumulator double-count). For pure-forward
    //    motion both K's reduce to v/(relative speed) so they agree numerically.
    //    forward_fraction>0.5 gates to the well-conditioned forward regime; the raw
    //    (HP-removed) accel integrator supplies the ZUPT-re-zeroed metric reference.
    // 2026-05-30 (Scale fix Step 5) — TURN SUPPRESSION (mirrors the depth-flow path):
    // turning lowers forward looming vz_rel while accel_dist accrues, inflating k_obs
    // toward the measured UTURN≈1652 and dragging K up. Skip the calib while
    // |gyro| ≥ 0.5 rad/s — above straight-walk yaw RMS (~0.17 rad/s), below in-place
    // turns. The accumulator keeps running (same as depth-flow) so the ratio spans the
    // same physical interval once calibration resumes (TRAP CHECKLIST #4).
    constexpr double kTurnSuppressGyroRadS = 0.5;  // rad/s
    const bool turning = (gyro_norm >= kTurnSuppressGyroRadS);
    if (forward_fraction_early > 0.5 &&
        secs_since_zupt_ >= 0.3 && secs_since_zupt_ <= 2.5 &&
        vz_rel_fused > 0.0) {
        visual_rel_dist_loom_ += vz_rel_fused * dt_s;
        if (!turning && accel_dist_accum_ > kAccelKMinDistM && visual_rel_dist_loom_ > 1e-4) {
            const double k_obs = accel_dist_accum_ / visual_rel_dist_loom_;
            // 2026-05-29 — BLOW-UP GUARD (this is the exact path that hit k_obs=22534 /
            // K=10602 / 27 km/h on v7 walkrun: vis_rel→0.0002 while accel_dist=4.76m).
            // Reject k_obs that is a >3x outlier vs the current looming K. First calib
            // (cur_k<=0) is unguarded. Mirrors the depth-flow guard + Observer A's reject.
            const double cur_k = expansion_scale_K_.load(std::memory_order_relaxed);
            const bool k_outlier = (cur_k > 0.0) && (k_obs > 3.0 * cur_k || k_obs < cur_k / 3.0);
            if (k_outlier && frame_counter_ % 30 == 0) {
                LOGI("ACCEL_K_CALIB[loom]: REJECT outlier k_obs=%.1f vs cur_K=%.1f (vis_rel=%.5f) gait=%d",
                     k_obs, cur_k, visual_rel_dist_loom_, static_cast<int>(active_mode_));
            }
            if (std::isfinite(k_obs) && k_obs > 0.0 && !k_outlier) {
                // 2026-05-31 (FIX A — cold fast-converge arm; see updateDepthFlowSpeed).
                // Arm the existing fast window once per session at the first accepted
                // calib when running on a persisted seed (cur_k>0), so a pure walk
                // converges to the live k_obs instead of crawling at α=0.05. Shared
                // one-shot flag + shared mode_switch_fast_alpha_frames_ with the
                // depth-flow path; inside the !k_outlier guard. Heading-safe (EMA weight
                // on expansion_scale_K_, a SPEED scale).
                // 2026-05-31 (a.3) — gate cold-arm OFF vehicle (see updateDepthFlowSpeed): fast-α
                // amplified the noisy scooter k_obs into the K ratchet; cold-arm is for WALK/RUN.
                if (!cold_fast_converge_armed_ && cur_k > 0.0 && active_mode_ != GaitMode::VEHICLE) {
                    mode_switch_fast_alpha_frames_ = kFastConvergeFrames;
                    cold_fast_converge_armed_ = true;
                    navsight::eventCounters().cold_fast_converge_armed.fetch_add(1, std::memory_order_relaxed);
                    LOGI("ACCEL_K_COLD_ARM[loom]: cur_K=%.1f k_obs=%.1f gait=%d fast_frames=%d",
                         cur_k, k_obs, static_cast<int>(active_mode_), mode_switch_fast_alpha_frames_);
                }
                // 2026-05-30 (Scale fix Step 3) — fast-converge EMA after a mode
                // switch; else steady-state α (== legacy 0.05). Pure walk: 0 fast
                // frames → α=0.05 exactly as before.
                double alpha = (mode_switch_fast_alpha_frames_ > 0) ? kFastAlpha : kNormalAlpha;
                // Decrement at most once per frame (see updateDepthFlowSpeed note —
                // shared counter with the depth-flow path).
                if (mode_switch_fast_alpha_frames_ > 0 &&
                    last_fast_alpha_frame_ != frame_counter_) {
                    --mode_switch_fast_alpha_frames_;
                    last_fast_alpha_frame_ = frame_counter_;
                }
                const double new_k = (cur_k <= 0.0) ? k_obs
                                                    : ((1.0 - alpha) * cur_k + alpha * k_obs);
                expansion_scale_K_.store(new_k, std::memory_order_relaxed);
                navsight::eventCounters().depth_flow_calib_updates.fetch_add(
                    1, std::memory_order_relaxed);
                // Visible, UNGATED log of every accepted looming-K calibration.
                LOGI("ACCEL_K_CALIB[loom]: k_obs=%.1f accel_dist=%.2fm vis_rel=%.4f "
                     "tsz=%.2fs fwd=%.2f cur_K=%.1f -> new_K=%.1f gait=%d", k_obs, accel_dist_accum_,
                     visual_rel_dist_loom_, secs_since_zupt_, forward_fraction_early,
                     cur_k, new_k, static_cast<int>(active_mode_));
                // Reuse midas_scale_k_milli min/max gauges for K visibility in
                // event_summary (looming K shares the same physical scale family).
                const long long k_milli = static_cast<long long>(new_k * 1000.0 + 0.5);
                navsight::eventCounters().midas_scale_k_milli.store(
                    k_milli, std::memory_order_relaxed);
                const long long kmax = navsight::eventCounters()
                    .midas_scale_k_max_milli.load(std::memory_order_relaxed);
                if (k_milli > kmax) navsight::eventCounters()
                    .midas_scale_k_max_milli.store(k_milli, std::memory_order_relaxed);
                const long long kmin = navsight::eventCounters()
                    .midas_scale_k_min_milli.load(std::memory_order_relaxed);
                if (kmin == 0 || k_milli < kmin) navsight::eventCounters()
                    .midas_scale_k_min_milli.store(k_milli, std::memory_order_relaxed);
            }
        } else if (turning && accel_dist_accum_ > kAccelKMinDistM &&
                   visual_rel_dist_loom_ > 1e-4 && frame_counter_ % 30 == 0) {
            // Scale fix Step 5 — would have calibrated, but suppressed by the turn gate.
            LOGI("ACCEL_K_CALIB[loom]: SKIP turn gyro_norm=%.3f>=0.5 accel_dist=%.2fm vis_rel=%.4f gait=%d",
                 gyro_norm, accel_dist_accum_, visual_rel_dist_loom_, static_cast<int>(active_mode_));
        }
    }

    // 5. Decide metric source: per-point affine (preferred — scene-invariant) or
    //    global K (fallback).
    double vz_fused_raw = -1.0;
    double vz_med       = -1.0;
    std::vector<double> vz_estimates;  // for log line
    bool used_affine = false;
    if (aff_valid && vz_metric_aff_est.size() >= kMinPts) {
        // Path A: per-point affine direct metric. Robust median + ρ²-WLS in metric
        // units. No global K multiplication — (s, t) absorbs the scene-dependent
        // disparity-to-metric mapping per-point.
        std::sort(vz_metric_aff_est.begin(), vz_metric_aff_est.end());
        const double vz_aff_med = vz_metric_aff_est[vz_metric_aff_est.size() / 2];
        double sum_num_a = 0.0, sum_den_a = 0.0;
        const double tol_aff = 0.3 * std::abs(vz_aff_med) + 0.1;
        for (size_t i = 0; i < vz_metric_aff_est.size(); ++i) {
            if (std::abs(vz_metric_aff_est[i] - vz_aff_med) < tol_aff) {
                const double w = rho_aff_values[i] * rho_aff_values[i];
                sum_num_a += w * vz_metric_aff_est[i];
                sum_den_a += w;
            }
        }
        vz_fused_raw = (sum_den_a > 0.0) ? (sum_num_a / sum_den_a) : vz_aff_med;
        vz_med = vz_aff_med;
        vz_estimates = vz_metric_aff_est;
        used_affine = true;
    } else {
        // Path B: looming's metric speed = K_loom * vz_rel. Uses LOOMING's own K
        // (expansion_scale_K_), calibrated above from the same vz_rel basis —
        // self-consistent, no cross-basis with depth-flow's midas_scale_K_.
        const double K = expansion_scale_K_.load(std::memory_order_relaxed);
        if (K <= 0.0) {
            navsight::eventCounters().depth_flow_looming_skipped.fetch_add(
                1, std::memory_order_relaxed);
            return;
        }
        vz_fused_raw = vz_rel_fused * K;
        vz_estimates.reserve(vz_rel_estimates.size());
        for (double r : vz_rel_estimates) {
            const double m = r * K;
            if (std::isfinite(m) && m > 0.0 && m < 30.0) vz_estimates.push_back(m);
        }
        vz_med = vz_estimates.empty() ? vz_fused_raw
                                      : vz_estimates[vz_estimates.size() / 2];
    }
    if (!std::isfinite(vz_fused_raw) || vz_fused_raw <= 0.0 || vz_fused_raw > 30.0) {
        navsight::eventCounters().depth_flow_looming_skipped.fetch_add(
            1, std::memory_order_relaxed);
        return;
    }

    // EMA on the LOOMING speed itself (kept as an independent diagnostic / output).
    // 2026-05-28 — tau dropped 0.5 -> 0.2 s: the slow EMA was washing out brief
    // running-speed bursts (vz_med spikes to ~3.5 m/s only reached ema ~0.6 m/s).
    constexpr double kTauS = 0.2;
    const double alpha = dt_s / (kTauS + dt_s);
    const double cur_loom = expansion_speed_mps_.load(std::memory_order_relaxed);
    const double next_loom = (cur_loom < 0.0) ? vz_fused_raw
                                              : (cur_loom + alpha * (vz_fused_raw - cur_loom));
    expansion_speed_mps_.store(next_loom, std::memory_order_relaxed);
    navsight::eventCounters().depth_flow_looming_updates.fetch_add(
        1, std::memory_order_relaxed);

    // 4. FUSE into the displayed speed by forward-motion fraction. Looming is
    //    best-conditioned for forward motion (its strength) and degenerate for pure
    //    lateral motion; recoverPose is the opposite. Continuous blend so transitions
    //    are smooth: w_loom = 0 at fwd_frac=0.5, 1 at fwd_frac=1.0.
    // 2026-05-28 — when looming is DOMINANT (w_loom > 0.5) use the RAW vz_fused
    //   (instant) instead of the EMA'd next_loom — the depth_flow EMA on the fused
    //   result still smooths it, but we get a responsive looming-dominated reading
    //   instead of one lagging behind the user's bursts.
    const double forward_fraction = (total_valid > 0)
        ? (static_cast<double>(positive) / total_valid) : 0.0;
    const double w_loom = std::clamp((forward_fraction - 0.5) * 2.0, 0.0, 1.0);
    const double cur_disp = depth_flow_speed_mps_.load(std::memory_order_relaxed);
    // 2026-05-28 — also store when cur_disp<0 (depth-flow never fired this
    // session, slow-walk scene). Without this, the looming-only case still
    // left depth_flow_speed_mps_ at -1 → UI sat at 0. Now: when w_loom>0,
    // looming is reliable enough to be the sole source.
    if (w_loom > 0.0) {
        const double loom_for_fusion = (w_loom > 0.5) ? vz_fused_raw : next_loom;
        double fused = loom_for_fusion;
        if (cur_disp >= 0.0) {
            // Both sources available: blend by forward-motion fraction.
            fused = w_loom * loom_for_fusion + (1.0 - w_loom) * cur_disp;
        }
        depth_flow_speed_mps_.store(fused, std::memory_order_relaxed);
        if (w_loom > 0.5) {
            navsight::eventCounters().depth_flow_looming_used.fetch_add(
                1, std::memory_order_relaxed);
        }
    }

    if (frame_counter_ % 30 == 0) {
        const double K_log = expansion_scale_K_.load(std::memory_order_relaxed);  // looming's own K
        LOGI("EXPANSION_SPEED: src=%s n=%zu vz_med=%.2f vz_fused=%.2f ema=%.2f "
             "fwd_frac=%.2f w_loom=%.2f K_loom=%.3f s=%.4f t=%.4f -> %.1f km/h",
             used_affine ? "affine" : "K",
             vz_estimates.size(), vz_med, vz_fused_raw, next_loom,
             forward_fraction, w_loom, K_log, s_aff, t_aff, next_loom * 3.6);
    }
}

void Tracker::updateGroundFlowSpeed(const std::vector<cv::Point2f>& prev_ud,
                                    const std::vector<cv::Point2f>& next_ud,
                                    double dt_s,
                                    const cv::Vec3d& gyro_rot_cam,
                                    const IMUPreintegrator& imu) {
    if (dt_s <= 1e-4 || fx_ <= 0.0 || fy_ <= 0.0 || camera_height_m_ <= 0.0) {
        if (frame_counter_ % 30 == 0)
            LOGI("GROUND_FLOW_FUNNEL: skipped dt=%.3f fx=%.1f h=%.2f (need h>0, fx>0)", dt_s, fx_, camera_height_m_);
        return;
    }

    // 1. Ground-plane normal in the CAMERA frame, straight from the FILTERED GRAVITY (the LP-filtered
    // accelerometer, which points UP in the body frame), rotated by the camera extrinsic: n_up_c = R_bc·ĝ.
    // AUDIT 2026-06-02 + offline-replay: this needs NO VIO init and NO Madgwick gravity-settle flag — gravity
    // is available from the first accel sample, even while static. The earlier versions failed because
    // ekf_.getRotation() is IDENTITY pre-VIO-init (→ n_up=(0,0,-1)=a wall ahead) AND the imu.isInitialized()
    // gate (gravity_initialized_, which needs a stationary startup window) blocked the replay harness and
    // cold starts entirely (gp_flow fired 0×). Gravity gives a correct "down" immediately. Yaw is NOT used
    // (forward is the optical axis below), so this is fully VIO/heading-independent. READ-ONLY: writes only
    // ground_flow_speed_mps_, never the dot.
    const cv::Point3f g_filt = imu.getFilteredGravity();
    const double g_norm = std::sqrt(static_cast<double>(g_filt.x) * g_filt.x +
                                    static_cast<double>(g_filt.y) * g_filt.y +
                                    static_cast<double>(g_filt.z) * g_filt.z);
    if (g_norm < 1.0) {   // no gravity sample yet (very first frames) — nothing to do
        if (frame_counter_ % 30 == 0) LOGI("GROUND_FLOW_FUNNEL: skipped reason=no_gravity_sample");
        return;
    }
    const cv::Matx33d R_bc = ekf_.getExtrinsicsRotation();
    // world-up in camera frame = R_bc · (filtered-accel direction, which points up in body when level)
    const cv::Vec3d n_up_c = R_bc * (cv::Vec3d(g_filt.x, g_filt.y, g_filt.z) / g_norm);

    // Skip when the camera is looking UP (no ground in front). n_up_c(2) is the optical-axis component of
    // world-up: <0 = camera tilted DOWN (ground ahead). When aimed up the projection is degenerate (the
    // grid sprayed into the sky, and any speed would be garbage), so bail. -0.05 ≈ at/just-below the horizon.
    if (n_up_c(2) > -0.05) {
        if (frame_counter_ % 30 == 0)
            LOGI("GROUND_FLOW_FUNNEL: skipped reason=camera_looking_up n_up_z=%.2f", n_up_c(2));
        return;
    }

    // 2. Forward = the camera's OPTICAL AXIS projected onto the ground plane (where the camera is looking,
    // flattened to the road) — heading-INDEPENDENT. AUDIT/on-device 2026-06-02 (screenshot: grid skewed off
    // the road + 19× under-read): the old code derived forward from the WORLD heading (scalar_heading_),
    // which is drifting/offset, so the predicted per-unit-speed flow direction `a` was rotated ~perpendicular
    // to the real road-ahead flow → grid lay off the road AND f·a≈0 → v_i rejected (speed_ok=0, under-read by
    // ~cos θ). The optical axis projected to the ground IS the road the camera sees = the scooter's travel
    // direction (forward-mounted), so it aligns `a` with the real flow. Must be UNIT (the speed scale needs it).
    cv::Vec3d u_fwd_c = cv::Vec3d(0.0, 0.0, 1.0) - cv::Vec3d(0.0, 0.0, 1.0).dot(n_up_c) * n_up_c;
    const double fwd_norm = cv::norm(u_fwd_c);
    if (fwd_norm < 1e-6) return;   // camera looking straight along gravity → no forward on the plane
    u_fwd_c /= fwd_norm;

    // 3. De-rotate flow and project each point to ground plane.
    const double wx = gyro_rot_cam[0];
    const double wy = gyro_rot_cam[1];
    const double wz = gyro_rot_cam[2];

    const int rot = ((analyzer_rotation_deg_.load() % 360) + 360) % 360;   // kept for the funnel log only
    // GEOMETRIC road-point selection (AUDIT 2026-06-02, 3 converging lenses). A "road" point is one whose ray
    // is BELOW the gravity horizon (n_up·ray < 0 → Z = -h/(n_up·ray) > 0, the ground plane IN FRONT) within a
    // plausible depth band. This REPLACES the old hardcoded pixel strip (px > 0.55·width): in this sensor-
    // native R_bc frame the horizon is a near-VERTICAL line, so the fixed strip STRADDLED it and fed OFF-PLANE
    // (sky/horizon, D2>0 → Z<0) points into the median → cos(f,a)≈0 → the ~20× under-read. Following gravity is
    // rotation/mount-agnostic and matches how the overlay grid (computeGroundGrid) already selects the road.
    constexpr double kIpmMinRoadDepthM = 0.5;    // closest plausible road point for a ~1 m forward-down mount
    constexpr double kIpmMaxRoadDepthM = 50.0;   // beyond this the road flow is sub-pixel + the planar assumption fails
    // FORWARD-COHERENCE floor (2026-06-03, harness-proven). A genuine static road point under forward
    // translation has de-rotated flow f_tr = -v*a, i.e. ANTI-parallel to the predicted ground-flow
    // direction a → cos(f,a) → -1. Require the backward-aligned component to EXCEED the perpendicular
    // component → cos_fa < -1/√2 (a derived 45° cone: |f·â| > |f_perp| ⟺ |cos| > sin ⟺ |cos| > 1/√2 —
    // NOT a tuned constant). This rejects the incoherent flow that FABRICATED the standstill speed
    // (replay ipm_diag on the standstill clip: spurious rows cos_fa = +0.1..+0.97, the one real-motion
    // row = -0.94) and the off-plane / large-dt de-rotation-residual spikes. cos_fa was already computed
    // here (dbg_align) but only logged — this finally USES it.
    constexpr double kIpmFwdCoherenceMin = 0.70710678118654752;   // 1/√2 = cos(135°)
    // The Heeger-Jepson de-rotation below is FIRST-ORDER in the per-frame rotation θ; its leading
    // truncation error is ~θ/2. Cap the per-frame rotation magnitude at 0.2 rad (~11°) so that error
    // stays <10%. Above it (dropped frames / fast pan, e.g. dt=0.3 s × 1 rad/s) the rotational flow is
    // mis-removed and the residual masquerades as translation (replay: large-dt rows produced coherent-
    // looking cos_fa from pure de-rotation error). Derived from the model order, not tuned.
    constexpr double kIpmMaxFrameRotRad = 0.2;

    // Bail when the per-frame rotation exceeds the first-order de-rotation's validity (see above). DECAY
    // toward 0 rather than HOLD, so a dropped-frame / fast-pan burst can't freeze a stale speed.
    const double frame_rot_rad = std::sqrt(wx * wx + wy * wy + wz * wz);
    if (frame_rot_rad > kIpmMaxFrameRotRad) {
        double cur_v = ground_flow_speed_mps_.load(std::memory_order_relaxed);
        if (cur_v > 0.0) ground_flow_speed_mps_.store((1.0 - 0.15) * cur_v, std::memory_order_relaxed);
        if (frame_counter_ % 20 == 0)
            LOGI("GROUND_FLOW_FUNNEL: skipped reason=large_frame_rotation rot_rad=%.3f thresh=%.2f dt=%.3f → decay",
                 frame_rot_rad, kIpmMaxFrameRotRad, dt_s);
        return;
    }

    std::vector<double> speeds;
    std::vector<cv::Point2f> inlier_px;   // 2026-06-02 VIZ — road pixels the IPM used (current frame)
    int n_in_road = 0;   // FUNNEL diag: points in the lower-image road region
    int n_denom_ok = 0;  // FUNNEL diag: of those, points with a well-conditioned ground-plane denom
    int n_coh_reject = 0;  // FUNNEL diag: points rejected by the forward-coherence gate (incoherent flow)
    std::vector<double> dbg_raw_vi, dbg_fmag_px, dbg_Z, dbg_align;   // TEMP IPM_DIAG (raw, ungated)
    const size_t n_pts = std::min(prev_ud.size(), next_ud.size());
    for (size_t i = 0; i < n_pts; ++i) {
        const float px = prev_ud[i].x;
        const float py = prev_ud[i].y;
        
        const double xp = (px - cx_) / fx_;
        const double yp = (py - cy_) / fy_;

        // GEOMETRIC road test: keep the point only if its ray is BELOW the gravity horizon (D2<0 → Z>0,
        // ground plane in front) within the plausible depth band. Subsumes the missing Z>0 filter and
        // replaces the off-plane-admitting pixel strip — see the note above.
        const double D2 = n_up_c(0) * xp + n_up_c(1) * yp + n_up_c(2);   // n_up·ray  (= -h/Z)
        const double inv_Z = -D2 / camera_height_m_;                      // 1/Z from the known mount height
        if (inv_Z <= 0.0) continue;                                       // D2 ≥ 0 → at/above horizon, not road
        const double Z = 1.0 / inv_Z;
        if (Z < kIpmMinRoadDepthM || Z > kIpmMaxRoadDepthM) continue;     // implausible depth → off-plane, reject
        ++n_in_road;

        // De-rotate flow (Heeger-Jepson 1992, normalized angular flow per frame interval).
        const double u_rot = xp * yp * wx - (1.0 + xp * xp) * wy + yp * wz;
        const double v_rot = (1.0 + yp * yp) * wx - xp * yp * wy - xp * wz;
        const double dx = (next_ud[i].x - px) / fx_;
        const double dy = (next_ud[i].y - py) / fy_;
        const double f_tr_x = dx - u_rot;   // de-rotated TRANSLATIONAL flow, both axes
        const double f_tr_y = dy - v_rot;

        // A static ground point's flow is f = -v·a, where a = u_fwd⊥/Z is the per-unit-speed direction:
        //   u_fwd⊥ = (u_fwd.x - xp·u_fwd.z,  u_fwd.y - yp·u_fwd.z). Least-squares over BOTH axes:
        //   v = -(f·a)/(a·a). For a true road point cos(f,a) → -1 (the geometric gate above guarantees the
        //   point is on the plane, so the perpendicular cos≈0 case — off-plane points — can no longer occur).
        const double a_x = (u_fwd_c(0) - xp * u_fwd_c(2)) * inv_Z;
        const double a_y = (u_fwd_c(1) - yp * u_fwd_c(2)) * inv_Z;
        const double aa = a_x * a_x + a_y * a_y;
        if (aa > 1e-8) {
            ++n_denom_ok;
            const double v_i = -((f_tr_x * a_x + f_tr_y * a_y) / aa) / dt_s;   // metres/frame → m/s
            const double f_mag = std::sqrt(f_tr_x * f_tr_x + f_tr_y * f_tr_y);
            const double cos_fa = (f_tr_x * a_x + f_tr_y * a_y) / (f_mag * std::sqrt(aa) + 1e-12);
            // TEMP IPM_DIAG — raw (ungated) per-point internals.
            dbg_raw_vi.push_back(v_i);
            dbg_fmag_px.push_back(f_mag * fx_);   // flow magnitude px/frame
            dbg_Z.push_back(Z);                   // depth (m), now > 0
            dbg_align.push_back(cos_fa);          // cos(f,a)
            // COHERENCE GATE: keep the point only when its de-rotated flow is dominantly forward-aligned
            // (cos_fa < -1/√2). cos_fa < 0 already implies v_i > 0, so this SUBSUMES the old positive-
            // speed gate; the 50 m/s cap stays as the upper sanity bound. Incoherent standstill / off-
            // plane / de-rotation-residual flow (cos_fa not near -1) no longer fabricates a speed.
            if (v_i < 50.0 && cos_fa < -kIpmFwdCoherenceMin) {
                speeds.push_back(v_i);
                inlier_px.push_back(next_ud[i]);   // VIZ — this road pixel fed the speed (current frame)
            } else {
                ++n_coh_reject;
            }
        }
    }

    // FUNNEL diagnostic — logs WHERE the road pixels are lost, even when the IPM produces nothing (the
    // GROUND_SPEED_DBG line only fires on success, so a failing ride was previously invisible). Fires
    // whenever it under-produces (<5) or every 30 frames. Read it as a funnel:
    //   in_road=0          → the road region is empty (wrong rotation, or the road isn't in the lower
    //                         image — e.g. filming a vertical screen, or camera aimed too high).
    //   in_road>0,denom~0  → the ground-plane geometry is degenerate (EKF/heading/gravity not valid yet,
    //                         or the surface isn't a horizontal ground plane — the screen-video case).
    //   denom_ok>0,speed=0 → all v_i fell outside (0.1,50) m/s (not actually translating / wrong scale).
    // FUNNEL + RAW INTERNALS — on-device-visible (LOGI → logcat) so a real ride pinpoints the under-read.
    // raw_vi = ungated per-point speed (signed); fmag_px = de-rotated flow px/frame; Z = recovered depth
    // (must be POSITIVE — a road point in front); cos_fa = alignment of measured flow with the predicted
    // ground-flow direction (should be ≈ −1 for forward motion; ≈0 = the forward/geometry is wrong → speed
    // collapses). The harness can't exercise this (KLT tracks <8 pts on fast/blurred scooter frames), so
    // these are read from logcat on-device.
    auto med = [](std::vector<double>& v) -> double {
        if (v.empty()) return -9.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
    if (speeds.size() < 5 || frame_counter_ % 20 == 0) {
        LOGI("GROUND_FLOW_FUNNEL: in_road=%d denom_ok=%d speed_ok=%zu dt=%.3f raw_vi=%.2f fmag_px=%.1f "
             "Z=%.2f cos_fa=%.2f n_up=(%.2f,%.2f,%.2f) u_fwd=(%.2f,%.2f,%.2f) rot=%d h=%.2f",
             n_in_road, n_denom_ok, speeds.size(), dt_s, med(dbg_raw_vi), med(dbg_fmag_px),
             med(dbg_Z), med(dbg_align), n_up_c(0), n_up_c(1), n_up_c(2),
             u_fwd_c(0), u_fwd_c(1), u_fwd_c(2), rot, camera_height_m_);
    }
    // FILE mirror for the harness (stderr buffering ate prints there; fopen no-ops on device).
    static FILE* dbgf = std::fopen("ipm_diag.txt", "w");
    static int dbgc = 0;
    if (dbgf && (dbgc++ % 5 == 0)) {
        std::fprintf(dbgf, "IPM_DIAG in_road=%d denom_ok=%d speed_ok=%zu dt=%.3f raw_vi=%.2f fmag_px=%.1f "
                     "Z=%.2f cos_fa=%.2f n_up=(%.2f,%.2f,%.2f) u_fwd=(%.2f,%.2f,%.2f)\n",
                     n_in_road, n_denom_ok, speeds.size(), dt_s, med(dbg_raw_vi), med(dbg_fmag_px),
                     med(dbg_Z), med(dbg_align), n_up_c(0), n_up_c(1), n_up_c(2),
                     u_fwd_c(0), u_fwd_c(1), u_fwd_c(2));
        std::fflush(dbgf);
    }
    /* RESEARCH 2026-06-03 — per-point (v_i,Z,flow,cos_fa) dump used to test the depth/flow-WEIGHTING idea.
       VERDICT: do NOT add weighting. On real data v_i rises monotonically with Z (0.58 m/s @ Z<3 → 41 m/s @
       Z>20) while the measured flow is ~constant ~12px — i.e. the flow is CAPPED at the tracker limit, not
       ∝1/Z, so v_i is depth-BIASED, not unbiased-noisy. Reweighting only slides along that biased curve
       (inverse-Z would favour the LOW-reading near points → worse). The unweighted MEDIAN is robust and lands
       mid-curve; KEEP it. The real lever is un-capping the near flow (window / full frame rate). Dump removed.
    static FILE* ptf = std::fopen("ipm_points.csv", "w"); ... (re-add to re-run the analysis) */

    // 4. Robust median and EMA.
    if (speeds.size() >= 5) {
        std::sort(speeds.begin(), speeds.end());
        double v_med = speeds[speeds.size() / 2];
        
        double cur_v = ground_flow_speed_mps_.load(std::memory_order_relaxed);
        if (cur_v < 0.0) {
            ground_flow_speed_mps_.store(v_med, std::memory_order_relaxed);
        } else {
            const double alpha = 0.15;
            ground_flow_speed_mps_.store((1.0 - alpha) * cur_v + alpha * v_med, std::memory_order_relaxed);
        }

        LOGI("GROUND_SPEED_DBG raw_mps=%.3f ema_mps=%.3f inliers=%d dt=%.3f coh_rej=%d",
             v_med, ground_flow_speed_mps_.load(), (int)speeds.size(), dt_s, n_coh_reject);
    } else {
        // No forward-COHERENT road flow this frame → forward motion is not being observed. DECAY the
        // EMA toward 0 (same α) instead of HOLDING — a real stop then reads ~0 within ~0.7 s instead of
        // freezing at the last cruise value (owner: "standing still but the speed wasn't 0"). A transient
        // blur/occlusion dips briefly and recovers the moment coherent road flow returns. Only decays a
        // POSITIVE running value; the -1 "uninitialised" sentinel is left untouched.
        double cur_v = ground_flow_speed_mps_.load(std::memory_order_relaxed);
        if (cur_v > 0.0) {
            const double alpha = 0.15;
            ground_flow_speed_mps_.store((1.0 - alpha) * cur_v, std::memory_order_relaxed);
        }
        if (frame_counter_ % 20 == 0)
            LOGI("GROUND_SPEED_DECAY: no coherent road flow (in_road=%d coh_rej=%d) → decay v=%.2f",
                 n_in_road, n_coh_reject, ground_flow_speed_mps_.load());
    }

    // VIZ — publish the road pixels used this frame (or the few candidates when <5) so the camera
    // overlay shows the ground-plane recognition live. Always set (even empty) so stale dots clear.
    {
        std::lock_guard<std::mutex> lk(ipm_viz_mutex_);
        ipm_inlier_px_.swap(inlier_px);
    }
}

// 2026-06-02 — AV-style ground-plane GRID. Projects the IPM ground plane (the flat surface at the known
// mount height, tilted by gravity, oriented by heading) into a perspective mesh of image-pixel line
// segments. Gravity (from the IMU/Madgwick attitude — valid immediately, even static) sets the tilt, so
// the grid lies flat on the real road when the geometry is correct — exactly the "show the road" view AVs
// render (minus the learned road SEGMENTATION, which needs a model we don't have). READ-ONLY: drawn on the
// camera overlay, never feeds the dot. Empty until imu.isInitialized() (gravity settled) — NOT gated on
// full VIO init, so it shows on a bench/video as soon as the accelerometer locks gravity (AUDIT 2026-06-02).
void Tracker::computeGroundGrid(const IMUPreintegrator& imu) {
    std::vector<float> segs;
    auto publish = [&]() {
        std::lock_guard<std::mutex> lk(ground_grid_mutex_);
        ground_grid_segs_.swap(segs);
    };
    const cv::Point3f g_filt = imu.getFilteredGravity();
    const double g_norm = std::sqrt(static_cast<double>(g_filt.x) * g_filt.x +
                                    static_cast<double>(g_filt.y) * g_filt.y +
                                    static_cast<double>(g_filt.z) * g_filt.z);
    if (camera_height_m_ <= 0.0 || fx_ <= 0.0 || fy_ <= 0.0 || g_norm < 1.0) {
        publish();   // clear — no camera height or no gravity sample yet (empty grid = "not locked")
        return;
    }
    // Ground normal straight from filtered gravity (no VIO/Madgwick-init gate) — see updateGroundFlowSpeed.
    const cv::Vec3d n_up = ekf_.getExtrinsicsRotation() * (cv::Vec3d(g_filt.x, g_filt.y, g_filt.z) / g_norm);
    // Only draw when the camera is looking DOWN/forward enough that the GROUND is in front of it. n_up(2) is
    // the optical-axis component of world-up: <0 = camera tilted DOWN (ground ahead), >0 = tilted UP (sky
    // ahead, ground behind). When aimed up the projection degenerates and the grid sprays into the sky
    // (owner-reported), so skip it there — no ground in front to draw. -0.05 ≈ at/just-below the horizon.
    if (n_up(2) > -0.05) { publish(); return; }
    // Forward = camera OPTICAL AXIS projected onto the ground plane (heading-independent — lies on the road
    // the camera actually sees). See updateGroundFlowSpeed for why the world-heading version skewed it.
    cv::Vec3d fwd_g = cv::Vec3d(0.0, 0.0, 1.0) - cv::Vec3d(0.0, 0.0, 1.0).dot(n_up) * n_up;
    const double fn = cv::norm(fwd_g);
    if (fn < 1e-6) { publish(); return; }
    fwd_g /= fn;
    cv::Vec3d right_g = fwd_g.cross(n_up);                                           // right ON the plane
    const double rn = cv::norm(right_g);
    if (rn < 1e-6) { publish(); return; }
    right_g /= rn;
    const double h = camera_height_m_;
    const cv::Vec3d G0 = -h * n_up;                                                  // ground point below cam
    const double img_w = cx_ * 2.0, img_h = cy_ * 2.0;
    auto project = [&](double d, double l, float& px, float& py) -> bool {
        const cv::Vec3d P = G0 + d * fwd_g + l * right_g;
        if (P[2] <= 0.05) return false;                                             // behind / at the camera
        px = static_cast<float>(fx_ * P[0] / P[2] + cx_);
        py = static_cast<float>(fy_ * P[1] / P[2] + cy_);
        if (!std::isfinite(px) || !std::isfinite(py)) return false;
        // keep segments near the frame (allow a margin so lines run to the edges)
        return px > -img_w && px < 2.0 * img_w && py > -img_h && py < 2.0 * img_h;
    };
    auto addSeg = [&](float x0, float y0, float x1, float y1) {
        segs.push_back(x0); segs.push_back(y0); segs.push_back(x1); segs.push_back(y1);
    };
    // Longitudinal rails (constant lateral offset, sweeping forward distance) — the "lane" lines. Capped at
    // 8 m (was 16): far points approach the horizon, which is HIGH in the frame when the camera isn't steeply
    // down (e.g. held near-horizontal at a video) — making the grid look like it sprays at the sky. Keeping it
    // to the near road foreground keeps it visibly ON the road. (The SPEED uses the full road depth band.)
    const double lanes[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    for (double l : lanes) {
        float px0 = 0, py0 = 0; bool have = project(1.0, l, px0, py0);
        for (double d = 2.0; d <= 8.0; d += 1.0) {
            float px1, py1; const bool h1 = project(d, l, px1, py1);
            if (have && h1) addSeg(px0, py0, px1, py1);
            px0 = px1; py0 = py1; have = h1;
        }
    }
    // Lateral rungs (constant forward distance, sweeping lateral) — the perspective "ladder".
    const double rungs[] = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    for (double d : rungs) {
        float px0 = 0, py0 = 0; bool have = project(d, -2.5, px0, py0);
        for (double l = -2.0; l <= 2.5; l += 0.5) {
            float px1, py1; const bool h1 = project(d, l, px1, py1);
            if (have && h1) addSeg(px0, py0, px1, py1);
            px0 = px1; py0 = py1; have = h1;
        }
    }
    publish();
}
