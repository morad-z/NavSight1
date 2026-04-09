#include "Tracker.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

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
    LOGI("setIntrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy);
}

void Tracker::setUserScaleCorrection(double correction) {
    std::lock_guard<std::mutex> lock(mutex_);
    user_scale_correction_ = std::max(0.1, std::min(5.0, correction));
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

    std::vector<float> depth_copy;
    int dw, dh;
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (depth_map_.empty()) return;
        depth_copy = depth_map_;
        dw = depth_width_;
        dh = depth_height_;
    }

    if (pts3d.size() < 15 || pts2d.size() != pts3d.size()) return;
    if (fx_ <= 0 || fy_ <= 0) return;

    // Camera height from user height (phone held ~0.85 * user_height)
    float user_h = imu.getUserHeight();
    double camera_h = static_cast<double>(user_h) * 0.85;
    if (camera_h < 0.8 || camera_h > 2.2) return;

    // Use gravity vector to estimate camera pitch (how much phone tilts down)
    float ax = imu.lastAccelX(), ay = imu.lastAccelY(), az = imu.lastAccelZ();
    double g_mag = std::sqrt(ax*ax + ay*ay + az*az);
    if (g_mag < 5.0) return; // no valid gravity
    // Pitch: angle between phone's Z-axis and horizontal plane
    double pitch = std::asin(std::min(1.0, std::max(-1.0, static_cast<double>(az) / g_mag)));

    // For features in the lower 40% of the image (likely floor/ground),
    // compute: metric_depth = camera_h / (norm_y * cos(pitch) + sin(pitch))
    // Then compare to VIO triangulated depth → scale ratio
    std::vector<double> scale_ratios;
    float fh = static_cast<float>(img_height);
    float fw = static_cast<float>(img_width);

    for (size_t i = 0; i < pts3d.size(); i++) {
        // Only use features in the lower 40% of the image
        if (pts2d[i].y < fh * 0.6f) continue;
        if (pts3d[i].z < 0.3 || pts3d[i].z > 12.0) continue;

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

    if (scale_ratios.size() < 8) return; // not enough confident matches

    // Take median ratio
    std::sort(scale_ratios.begin(), scale_ratios.end());
    double median_ratio = scale_ratios[scale_ratios.size() / 2];

    // Compute target scale: current_scale * median_ratio
    double current;
    {
        std::lock_guard<std::mutex> slock(pose_mutex_);
        current = smooth_scale_;
    }
    double target_scale = current * median_ratio;
    target_scale = std::max(0.01, std::min(10.0, target_scale));

    // Safety gate: only apply if correction is within 3x of current
    if (target_scale > 3.0 * current || target_scale < current / 3.0) {
        LOGI("DEPTH_SCALE: REJECTED target=%.4f current=%.4f ratio=%.4f (too extreme)",
             target_scale, current, median_ratio);
        return;
    }

    // Conservative blend: alpha=0.03, only nudges the scale gently
    double alpha = 0.03;
    {
        std::lock_guard<std::mutex> slock(pose_mutex_);
        smooth_scale_ = (1.0 - alpha) * smooth_scale_ + alpha * target_scale;
        smooth_scale_ = std::max(0.01, std::min(10.0, smooth_scale_));
    }

    if (frame_counter_ % 30 == 0) {
        LOGI("DEPTH_SCALE: applied target=%.4f current=%.4f ratio=%.4f samples=%zu",
             target_scale, current, median_ratio, scale_ratios.size());
    }
}

void Tracker::setInitialHeading(double azimuth_rad) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    double c = std::cos(azimuth_rad), s = std::sin(azimuth_rad);
    global_R_ = (cv::Mat_<double>(3, 3) << c,-s,0, s,c,0, 0,0,1);
    // Set offset so cached heading matches the requested azimuth even though
    // Madgwick owns the physics: heading = madgwick.getHeading() + offset.
    // At the time this is called (app startup), Madgwick is not yet
    // initialized, so its heading is 0 — offset == azimuth_rad.
    // If tracking is re-initialized mid-session, the next processFrame will
    // rebase the offset against the current Madgwick heading (see init path).
    pending_init_heading_ = azimuth_rad;
    pending_init_heading_set_ = true;
    heading_offset_ = azimuth_rad;
    scalar_heading_ = azimuth_rad;
    ekf_.initialize(smooth_scale_);
    LOGI("setInitialHeading: azimuth=%.1f deg (EKF scale initialized, offset=%.1f deg)",
         azimuth_rad * 180.0 / M_PI, heading_offset_ * 180.0 / M_PI);
}

void Tracker::reset() {
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
    smooth_scale_ = 0.20;
    scale_obs_count_ = 0;
    scale_bootstrap_buf_.clear();
    points_3d_current_.clear();
    feature_ages_.clear();
    feature_ids_.clear();
    heading_initialized_ = false;
    scalar_heading_ = 0.0;
    heading_offset_ = 0.0;
    pending_init_heading_set_ = false;
    pending_init_heading_ = 0.0;
    filtered_yaw_rate_ = 0.0;
    heading_fej_set_ = false;
    heading_fej_ = 0.0;
    td_warmup_done_ = false;
    td_warmup_buf_.clear();
    frames_since_keyframe_ = 0;
    last_step_speed_ = 0.0;
    last_step_speed_ns_ = 0;
    ekf_.reset();
    feature_mgr_.reset();
    LOGI("Tracker reset");
}

// ── Helpers ──────────────────────────────────────────────────────────────────

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

    double current_smooth;
    int obs_count;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_smooth = smooth_scale_;
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
    std::lock_guard<std::mutex> lock(pose_mutex_); return smooth_scale_;
}
double Tracker::getHeading() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return scalar_heading_;
}

void Tracker::addImuData(int64_t ts, float ax, float ay, float az, float gx, float gy, float gz) {
    if (!initialized_) {
        initializer_.addImuData(ts, ax, ay, az, gx, gy, gz);
        if (initializer_.isReady()) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            global_R_ = initializer_.getInitialRotation();
            // Gyro bias now managed solely by IMUPreintegrator (unified)
            ekf_.initialize(smooth_scale_);
            initialized_ = true;
            LOGI("Tracker: System initialized via InertialInitializer");
        }
    }
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
    if (!heading_initialized_ && imu.hasMagHeading()) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        float mag_yaw = imu.getMagHeading();
        double cy = std::cos(mag_yaw), sy = std::sin(mag_yaw);
        // Override yaw in global_R_ while keeping identity pitch/roll
        global_R_ = (cv::Mat_<double>(3,3) <<
            cy, -sy, 0,
            sy,  cy, 0,
             0,   0, 1);
        // Rebase offset so cached heading reads mag_yaw right now, regardless
        // of where Madgwick's free-running yaw currently sits.
        heading_offset_ = static_cast<double>(mag_yaw) -
                          static_cast<double>(imu.getHeading());
        scalar_heading_ = mag_yaw;
        heading_initialized_ = true;
        LOGI("Tracker: Initial heading set from magnetometer: %.1f deg "
             "(offset=%.1f deg, madgwick=%.1f deg)",
             mag_yaw * 180.0 / M_PI,
             heading_offset_ * 180.0 / M_PI,
             imu.getHeading() * 180.0 / M_PI);
    }

    // Apply any pending setInitialHeading() request now that Madgwick is live.
    if (pending_init_heading_set_ && imu.isOrientationInitialized()) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        heading_offset_ = pending_init_heading_ -
                          static_cast<double>(imu.getHeading());
        scalar_heading_ = pending_init_heading_;
        pending_init_heading_set_ = false;
        LOGI("Tracker: applied pending init heading=%.1f deg, offset=%.1f deg",
             pending_init_heading_ * 180.0 / M_PI,
             heading_offset_ * 180.0 / M_PI);
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
            LOGI("processFrame: first frame, grid-detected %zu features", prev_pts_.size());
            return out;
        }
        current_prev_gray = prev_gray_;
        current_prev_pts_buf_ = prev_pts_;
        current_prev_ts = prev_timestamp_ns_;
    }

    frame_counter_++;

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
    bool is_pure_rotation = (gyro_norm > GYRO_ROT_ONLY_THRESH);

    // ── 5. Optical flow tracking (TrackKLT) ──────────────────────────────────
    int64_t t_klt_start = now_us();
    std::vector<uchar> status;
    next_pts_buf_.clear();
    klt_.track(current_prev_gray, gray_buf_, current_prev_pts_buf_,
               next_pts_buf_, status, imu_delta.deltaR, K);
    int64_t t_klt_end = now_us();

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
    if (sufficient_motion && has_parallax && !is_static && tracked >= 8) {
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

                // ── Rotation: use gyro directly (bias already subtracted in preintegrator)
                // IMUPreintegrator subtracts gyro_bias_ during integration (line 227-229),
                // so imu_delta.deltaR is already bias-corrected. Do NOT subtract again.
                //
                // CRITICAL: Yaw is UNOBSERVABLE from monocular camera (OpenVINS).
                // Previous code blended Rodrigues vector components rv[0]/rv[2] from
                // camera with rv[1] from gyro — but Rodrigues components are NOT
                // independent Euler angles! Mixing them cross-couples axes and
                // corrupts heading. Use gyro rotation directly for all axes.
                R_fused = imu_delta.deltaR.clone();

                if (frame_counter_ % 90 == 0) {
                    LOGI("POSE: transDegen=%d q=%.2f rotation=gyro_only scale=%.4f",
                         translation_degenerate ? 1 : 0, quality, smooth_scale_);
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
                        if (si.speed_mps > 0.3) {
                            double step_disp = si.speed_mps * dt_sec;
                            double obs_scale = step_disp / vo_dist;

                            // Clamp to sane range
                            obs_scale = std::max(0.005, std::min(10.0, obs_scale));

                            if (scale_obs_count_ < SCALE_BOOTSTRAP_COUNT) {
                                // Phase 1: Bootstrap — collect observations
                                scale_bootstrap_buf_.push_back(obs_scale);
                                scale_obs_count_++;

                                if (scale_obs_count_ >= SCALE_BOOTSTRAP_COUNT) {
                                    // Take median of bootstrap buffer
                                    std::vector<double> sorted = scale_bootstrap_buf_;
                                    std::sort(sorted.begin(), sorted.end());
                                    double median = sorted[sorted.size() / 2];

                                    std::lock_guard<std::mutex> slock(pose_mutex_);
                                    smooth_scale_ = median;
                                    estimatedScale = smooth_scale_;
                                    LOGI("SCALE_BOOTSTRAP: median=%.4f from %d samples (range %.4f-%.4f)",
                                         median, SCALE_BOOTSTRAP_COUNT, sorted.front(), sorted.back());
                                    scale_bootstrap_buf_.clear();
                                    scale_bootstrap_buf_.shrink_to_fit();
                                }
                            } else {
                                // Phase 2: EMA refinement with outlier rejection
                                bool reject = false;
                                {
                                    std::lock_guard<std::mutex> slock(pose_mutex_);
                                    if (obs_scale > 2.5 * smooth_scale_ || obs_scale < smooth_scale_ / 2.5) {
                                        reject = true;
                                    }
                                }

                                if (!reject) {
                                    double alpha = (scale_obs_count_ < 50) ? 0.08 : 0.03;
                                    std::lock_guard<std::mutex> slock(pose_mutex_);
                                    smooth_scale_ = (1.0 - alpha) * smooth_scale_ + alpha * obs_scale;
                                    smooth_scale_ = std::max(0.01, std::min(10.0, smooth_scale_));
                                    scale_obs_count_++;
                                    estimatedScale = smooth_scale_;

                                    if (frame_counter_ % 30 == 0) {
                                        LOGI("SCALE: obs=%.4f smooth=%.4f vo=%.4f step_d=%.3f count=%d",
                                             obs_scale, smooth_scale_, vo_dist, step_disp, scale_obs_count_);
                                    }
                                }
                            }
                        }
                    }
                }

                // Phase 8 (gravity-aided scale) DISABLED:
                // IMU preintegration deltaP is dominated by gravity subtraction
                // errors on phone IMUs, producing wildly wrong scale estimates
                // that fight against the reliable step-based scale.
                // Step detection + stride estimation is the primary scale source.

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

    // Always report current EKF scale (even if no new observation this frame)
    {
        std::lock_guard<std::mutex> slock(pose_mutex_);
        estimatedScale = smooth_scale_;
    }
    double appliedScale = estimatedScale;
    { std::lock_guard<std::mutex> lock(mutex_); appliedScale *= user_scale_correction_; }

    // ── 9. Global pose update (heading-based 2D) ─────────────────────────────
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // ── 9.0 Heading from Madgwick IMU-only attitude filter ─────────────
        // Madgwick runs at full IMU rate inside IMUPreintegrator and returns
        // a yaw that is NOT corrupted by centripetal accel the way the old
        // gravity-projection integrator was. heading_offset_ carries initial
        // mag/GPS bias plus visual keyframe corrections.
        scalar_heading_ = static_cast<double>(imu.getHeading()) + heading_offset_;
        while (scalar_heading_ >  M_PI) scalar_heading_ -= 2.0 * M_PI;
        while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;

        // Keep global_R_ in sync for existing consumers (step 2 of the plan
        // will retire this). During ZUPT we still freeze it so downstream
        // translation-dependent code behaves the same.
        if (!is_static) {
            global_R_ = global_R_ * imu_delta.deltaR;
        }

        double heading = scalar_heading_;

        if (frame_counter_ % 30 == 0) {
            const float m_yaw   = imu.getHeading() * 180.0f / static_cast<float>(M_PI);
            const float m_roll  = imu.getMadgwickRoll()  * 180.0f / static_cast<float>(M_PI);
            const float m_pitch = imu.getMadgwickPitch() * 180.0f / static_cast<float>(M_PI);
            LOGI("HEADING: madgwick_yaw=%.1f° roll=%.1f° pitch=%.1f° "
                 "offset=%.1f° -> heading=%.1f°",
                 m_yaw, m_roll, m_pitch,
                 heading_offset_ * 180.0 / M_PI,
                 heading * 180.0 / M_PI);
        }

        // Phase 7: Lock FEJ heading on first valid pose
        if (!heading_fej_set_ && pose_valid) {
            heading_fej_ = heading;
            heading_fej_set_ = true;
        }

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

            global_t_.at<double>(0) += disp * std::sin(heading);  // +X = East
            global_t_.at<double>(2) += disp * std::cos(heading);  // +Z = North
            if (!t_vo.empty())
                global_t_.at<double>(1) += appliedScale * t_vo.at<double>(1);
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
                global_t_.at<double>(0) += d * std::sin(heading);  // +X = East
                global_t_.at<double>(2) += d * std::cos(heading);  // +Z = North
            }
        }

        // ── 9.1 FEJ & MSCKF: Store Camera Clone ──
        ekf_.addClone(global_R_, global_t_, timestamp_ns);
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

        // ── 11.1 MSCKF Update: DISABLED ──
        // The MSCKF EKF state diverges from Tracker's global pose,
        // causing large position/rotation jumps (seen as 5-11m teleportations).
        // The full error-state EKF (Phase 9) needs proper convergence before
        // its corrections can be trusted. For now, Tracker drives pose directly.
        //
        // Still prune feature observations to prevent memory growth.
        if (ekf_.isFullInitialized() && !feature_ids_.empty()) {
            feature_mgr_.extractLostFeatures(feature_ids_, 3);  // discard result
            if (!ekf_.getWindow().empty()) {
                int min_id = ekf_.getWindow().front().state_id;
                feature_mgr_.pruneObservations(min_id);
            }
        }

        // ── 11.2 Keyframe heading drift correction ──
        // Every keyframe interval, match current frame against the last keyframe.
        // The essential matrix R from that match gives the visual heading change.
        // Compare to gyro-integrated heading change and apply gentle correction.
        //
        // BUG FIX: Only apply during slow movement (gyro_norm < 0.3 rad/s ≈ 17°/s).
        // The visual heading uses atan2(R[1,0], R[0,0]) which extracts rotation
        // around the camera Z-axis (optical axis), NOT around gravity (true yaw).
        // During turns, this systematically under-counts heading change, causing
        // the correction to REMOVE real heading from the gyro integration.
        // This was the root cause of 180° turns registering as only ~134°.
        if (frames_since_keyframe_ >= 14 && pose_valid && tracked >= MIN_INLIERS * 2
            && gyro_norm < 0.3) {
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
                            // Extract yaw from visual rotation (atan2 of 2D rotation component)
                            double visual_delta_heading = std::atan2(
                                R_kf.at<double>(1, 0), R_kf.at<double>(0, 0));

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
                                // 30% correction — applied to heading_offset_
                                // (not Madgwick itself). The next frame's
                                // scalar_heading_ = madgwick + offset will
                                // reflect the correction automatically.
                                heading_offset_ -= 0.30 * drift;
                                while (heading_offset_ >  M_PI) heading_offset_ -= 2.0 * M_PI;
                                while (heading_offset_ < -M_PI) heading_offset_ += 2.0 * M_PI;
                                scalar_heading_ -= 0.30 * drift;
                                while (scalar_heading_ > M_PI) scalar_heading_ -= 2.0 * M_PI;
                                while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;

                                if (frame_counter_ % 30 == 0) {
                                    LOGI("KF_HEADING_CORR: drift=%.2f° correction=%.2f° inliers=%d",
                                         drift * 180.0 / M_PI, 0.30 * drift * 180.0 / M_PI, inl);
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
            frames_since_keyframe_ = 0;
        }

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
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        out.R = global_R_.clone();
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
