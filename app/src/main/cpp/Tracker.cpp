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

void Tracker::setInitialHeading(double azimuth_rad) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    double c = std::cos(azimuth_rad), s = std::sin(azimuth_rad);
    global_R_ = (cv::Mat_<double>(3, 3) << c,-s,0, s,c,0, 0,0,1);
    scalar_heading_ = azimuth_rad;
    ekf_.initialize(smooth_scale_);
    LOGI("setInitialHeading: azimuth=%.1f deg (EKF scale initialized)", azimuth_rad * 180.0 / M_PI);
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
    points_3d_current_.clear();
    feature_ages_.clear();
    feature_ids_.clear();
    heading_initialized_ = false;
    scalar_heading_ = 0.0;
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

// ── Scale / Loop Correction (called by VioEngine) ───────────────────────────

void Tracker::blendScale(double target_scale, double alpha) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    smooth_scale_ = (1.0 - alpha) * smooth_scale_ + alpha * target_scale;
    smooth_scale_ = std::max(0.005, std::min(20.0, smooth_scale_));
    // Also update EKF scale
    ekf_.updateScale(target_scale, alpha);
}

void Tracker::applyLoopCorrection(double tx, double ty, double tz,
                                   double target_heading, double blend,
                                   VisionOutput& out) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    global_t_.at<double>(0) += blend * (tx - global_t_.at<double>(0));
    global_t_.at<double>(1) += blend * (ty - global_t_.at<double>(1));
    global_t_.at<double>(2) += blend * (tz - global_t_.at<double>(2));

    double hdiff = target_heading - out.heading;
    while (hdiff >  M_PI) hdiff -= 2.0 * M_PI;
    while (hdiff < -M_PI) hdiff += 2.0 * M_PI;
    double h = out.heading + blend * hdiff;
    double c = std::cos(h), s = std::sin(h);
    global_R_ = (cv::Mat_<double>(3, 3) << c,-s,0, s,c,0, 0,0,1);
    out.t = global_t_.clone();
    out.heading = h;

    // Propagate loop closure correction to MSCKF sliding window clones
    if (ekf_.isFullInitialized()) {
        // Compute correction delta
        cv::Mat dp = (cv::Mat_<double>(3,1) <<
            blend * (tx - out.t.at<double>(0)),
            blend * (ty - out.t.at<double>(1)),
            blend * (tz - out.t.at<double>(2)));
        double dh = blend * hdiff;

        // Apply same correction to all clones in the window
        // This maintains relative pose consistency within the window
        for (auto& clone : const_cast<std::deque<CameraPose>&>(ekf_.getWindow())) {
            clone.p_G += dp;
            cv::Mat dR_clone = (cv::Mat_<double>(3,3) <<
                std::cos(dh), -std::sin(dh), 0,
                std::sin(dh),  std::cos(dh), 0,
                0, 0, 1);
            clone.R_GtoC = dR_clone * clone.R_GtoC;
        }
    }
}

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
        scalar_heading_ = mag_yaw;
        heading_initialized_ = true;
        LOGI("Tracker: Initial heading set from magnetometer: %.1f deg",
             mag_yaw * 180.0 / M_PI);
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
            feature_mgr_.storeKeyframe(gray_buf_, prev_pts_, timestamp_ns, 0);
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
    if (is_static && imu.getStepInfo().speed_mps > 0.3) is_static = false;
    if (is_static) {
        ekf_.updateZUPT();
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

                // ── Scale estimation from steps (simple EMA) ─
                // Scale = step_displacement / visual_displacement
                // t_vo is a unit vector from recoverPose, so norm(t_vo) ≈ 1.0
                // scale * norm(t_vo) = real_displacement_in_meters
                if (!translation_degenerate) {
                    double vo_dist = cv::norm(t_vo);
                    int64_t dt_ns_frame = timestamp_ns - current_prev_ts;
                    double dt_sec = dt_ns_frame * 1e-9;

                    if (vo_dist > 0.01 && dt_sec > 0.01 && dt_sec < 1.5
                        && !is_pure_rotation && !is_static) {
                        auto si = imu.getStepInfo();
                        // Only update scale when we have active step data
                        if (si.speed_mps > 0.3) {
                            double step_disp = si.speed_mps * dt_sec;
                            double obs_scale = step_disp / vo_dist;

                            // Reject wild outliers (>3x or <1/3 of current)
                            bool reject = false;
                            if (scale_obs_count_ > 10) {
                                std::lock_guard<std::mutex> slock(pose_mutex_);
                                if (obs_scale > 3.0 * smooth_scale_ || obs_scale < smooth_scale_ / 3.0) {
                                    reject = true;
                                }
                            }

                            if (!reject) {
                                // EMA with adaptive learning rate
                                // Fast convergence early (alpha=0.2), slow later (alpha=0.05)
                                double alpha = (scale_obs_count_ < 20) ? 0.15 : 0.05;
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

                // Phase 8 (gravity-aided scale) DISABLED:
                // IMU preintegration deltaP is dominated by gravity subtraction
                // errors on phone IMUs, producing wildly wrong scale estimates
                // that fight against the reliable step-based scale.
                // Step detection + stride estimation is the primary scale source.

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

        // ── 9.0 Heading from gravity-projected yaw rate ──────────────────────
        // The phone's yaw axis is the gravity direction, NOT the Z-axis.
        // Project the gyro angular velocity onto the gravity vector to get
        // the true yaw rate. This correctly handles any phone tilt angle.
        //
        // OpenVINS approach: maintains full gravity-aligned orientation and
        // extracts heading by projecting forward onto ground plane.
        // Our simplified approach: track heading as a scalar, project gyro
        // onto gravity to get yaw rate. Same result for 2D navigation.
        if (!is_static && imu_delta.dt > 0.001) {
            // Get gravity direction in phone body frame (normalized)
            cv::Point3f grav = imu.getFilteredGravity();
            double grav_mag = std::sqrt(grav.x*grav.x + grav.y*grav.y + grav.z*grav.z);
            if (grav_mag > 0.1) {
                double gn_x = grav.x / grav_mag;
                double gn_y = grav.y / grav_mag;
                double gn_z = grav.z / grav_mag;

                // Extract angular velocity from deltaR (bias already removed)
                cv::Mat rv;
                cv::Rodrigues(imu_delta.deltaR, rv);
                // rv = rotation vector in rad over the interval
                double wx = rv.at<double>(0) / imu_delta.dt;
                double wy = rv.at<double>(1) / imu_delta.dt;
                double wz = rv.at<double>(2) / imu_delta.dt;

                // Project angular velocity onto gravity axis = yaw rate
                // Negative sign: rotation around gravity is positive clockwise
                // when viewed from above (navigation convention)
                double yaw_rate = -(wx * gn_x + wy * gn_y + wz * gn_z);
                scalar_heading_ += yaw_rate * imu_delta.dt;

                // Normalize to [-π, π]
                while (scalar_heading_ > M_PI) scalar_heading_ -= 2.0 * M_PI;
                while (scalar_heading_ < -M_PI) scalar_heading_ += 2.0 * M_PI;
            }
        }

        // Update global_R_ for display/output (keep 3D rotation for Euler angles)
        if (is_static) {
            // ZUPT: freeze rotation
        } else {
            global_R_ = global_R_ * imu_delta.deltaR;
        }

        // Use scalar heading for navigation (correctly tracks turns)
        double heading = scalar_heading_;

        if (frame_counter_ % 30 == 0) {
            double old_hdg = std::atan2(global_R_.at<double>(1,0), global_R_.at<double>(0,0));
            LOGI("HEADING: scalar=%.1f° old_R=%.1f° delta=%.1f°",
                 heading * 180.0 / M_PI, old_hdg * 180.0 / M_PI,
                 (heading - old_hdg) * 180.0 / M_PI);
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

            if (speed > 0.1) {
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

        // Keyframe storage
        frames_since_keyframe_++;
        if (frames_since_keyframe_ >= 15 || (tracked < MIN_FEATURES / 2 && frames_since_keyframe_ > 3)) {
            feature_mgr_.storeKeyframe(gray_buf_, next_good_buf_, timestamp_ns, frame_counter_);
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
