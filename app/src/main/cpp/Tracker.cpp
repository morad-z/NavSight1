#include "Tracker.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <android/log.h>
#include <cmath>
#include <algorithm>
#include <limits>

#define TAG "NavSight-Tracker"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ── Constructor ──────────────────────────────────────────────────────────────

Tracker::Tracker() {
    global_R_ = cv::Mat::eye(3, 3, CV_64F);
    global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    accel_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    gyro_bias_  = cv::Mat::zeros(3, 1, CV_64F);

    current_prev_pts_buf_.reserve(MAX_FEATURES);
    next_pts_buf_.reserve(MAX_FEATURES);
    status_buf_.reserve(MAX_FEATURES);
    err_buf_.reserve(MAX_FEATURES);
    prev_good_buf_.reserve(MAX_FEATURES);
    next_good_buf_.reserve(MAX_FEATURES);
    new_pts_buf_.reserve(MAX_FEATURES);
    back_pts_buf_.reserve(MAX_FEATURES);
    back_status_buf_.reserve(MAX_FEATURES);
    back_err_buf_.reserve(MAX_FEATURES);

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
    gyro_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    gyro_bias_count_ = 0;
    smooth_scale_ = 0.20;
    scale_obs_count_ = 0;
    points_3d_current_.clear();
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
}

double Tracker::getSmoothScale() const {
    std::lock_guard<std::mutex> lock(pose_mutex_); return smooth_scale_;
}
double Tracker::getHeading() const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return std::atan2(global_R_.at<double>(1,0), global_R_.at<double>(0,0));
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

    // ── 3. First-frame: grid-based feature detection ─────────────────────────
    cv::Mat current_prev_gray;
    int64_t current_prev_ts = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || prev_pts_.empty()) {
            feature_mgr_.detectGridFeatures(gray_buf_, prev_pts_,
                                             MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            gray_buf_.copyTo(prev_gray_);
            prev_timestamp_ns_ = timestamp_ns;
            initialized_ = true;
            feature_mgr_.storeKeyframe(gray_buf_, prev_pts_, timestamp_ns, 0);
            LOGI("processFrame: first frame, grid-detected %zu features", prev_pts_.size());
            return out;
        }
        current_prev_gray = prev_gray_;
        current_prev_pts_buf_ = prev_pts_;
        current_prev_ts = prev_timestamp_ns_;
    }

    frame_counter_++;

    // ── 4. IMU integration + EKF prediction ──────────────────────────────────
    PreintegratedDelta imu_delta = imu.integrate(current_prev_ts, timestamp_ns);
    double gyro_norm = 0.0;
    {
        cv::Mat rv;
        cv::Rodrigues(imu_delta.deltaR, rv);
        if (imu_delta.dt > 0) gyro_norm = cv::norm(rv) / imu_delta.dt;
    }
    bool is_pure_rotation = (gyro_norm > GYRO_ROT_ONLY_THRESH);
    bool is_static = (gyro_norm < ZUPT_GYRO_THRESH);

    // ── 5. Optical flow with forward-backward check ──────────────────────────
    next_pts_buf_.clear(); status_buf_.clear(); err_buf_.clear();
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(current_prev_gray, gray_buf_, current_prev_pts_buf_,
        next_pts_buf_, status_buf_, err_buf_, cv::Size(31,31), 4, criteria,
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    back_pts_buf_.clear(); back_status_buf_.clear(); back_err_buf_.clear();
    cv::calcOpticalFlowPyrLK(gray_buf_, current_prev_gray, next_pts_buf_,
        back_pts_buf_, back_status_buf_, back_err_buf_, cv::Size(31,31), 4, criteria,
        cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    // ── 6. Filter valid points ───────────────────────────────────────────────
    prev_good_buf_.clear(); next_good_buf_.clear();
    for (size_t i = 0; i < status_buf_.size(); ++i) {
        if (!status_buf_[i] || i >= back_status_buf_.size() || !back_status_buf_[i]) continue;
        float bx = current_prev_pts_buf_[i].x - back_pts_buf_[i].x;
        float by = current_prev_pts_buf_[i].y - back_pts_buf_[i].y;
        if (bx * bx + by * by < FB_CHECK_THRESH) {
            prev_good_buf_.push_back(current_prev_pts_buf_[i]);
            next_good_buf_.push_back(next_pts_buf_[i]);
        }
    }

    int tracked = static_cast<int>(next_good_buf_.size());
    int total   = static_cast<int>(current_prev_pts_buf_.size());
    double quality = (total > 0) ? static_cast<double>(tracked) / total : 0.0;

    // In low light, tracking noise gives false high quality — clamp to zero
    if (is_low_light) quality = 0.0;

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

    // ZUPT: requires BOTH low gyro AND low visual motion
    is_static = (mean_flow < 0.5 && gyro_norm < ZUPT_GYRO_THRESH);
    if (mean_flow > 1.0) is_static = false;
    if (is_static && imu.getStepInfo().speed_mps > 0.1) is_static = false;
    // ZUPT is deferred to section 9 (under pose_mutex_ for thread safety)
    // The is_static flag is used there.

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

    if (sufficient_motion && has_parallax && !is_static && !is_low_light && tracked >= 8) {
        // Undistort matched points before geometric estimation
        std::vector<cv::Point2f> prev_ud = prev_good_buf_;
        std::vector<cv::Point2f> next_ud = next_good_buf_;
        lens_.undistortMatchedPoints(prev_ud, next_ud);

        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(prev_ud, next_ud, K,
                                         cv::RANSAC, RANSAC_CONF, RANSAC_THRESH, mask);

        if (!E.empty() && E.rows == 3 && E.cols == 3) {
            // Check SVD condition: high = forward motion (translation degenerate)
            // Rotation from essential matrix is valid even when translation is not.
            double svd_cond = 0.0;
            {
                cv::SVD svd(E);
                svd_cond = svd.w.at<double>(0) / (svd.w.at<double>(2) + 1e-10);
            }
            bool is_degenerate = (svd_cond > 50000.0);  // truly broken E
            if (svd_cond > 100.0) translation_degenerate = true;

            int inliers = cv::recoverPose(E, prev_ud, next_ud, K, R_vo, t_vo, mask);
            inlier_count_out = inliers;
            double inlier_ratio = static_cast<double>(inliers) / tracked;

            if (!t_vo.empty() && cv::norm(t_vo) < 0.001)
                translation_degenerate = true;

            if (inliers >= MIN_INLIERS && inlier_ratio >= MIN_INLIER_RATIO && !is_degenerate) {
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
                    for (int i = 0; i < pts4d.cols; ++i) {
                        cv::Mat p = pts4d.col(i);
                        double w = p.at<double>(3);
                        if (w > 1e-6)
                            points_3d_current_.emplace_back(
                                p.at<double>(0)/w, p.at<double>(1)/w, p.at<double>(2)/w);
                        else
                            points_3d_current_.emplace_back(0, 0, 0);
                    }
                }

                // ── Rotation fusion (ALWAYS — camera rotation valid even
                //    when translation is degenerate) ────────────────────
                cv::Mat rv_vo, rv_gyro;
                cv::Rodrigues(R_vo, rv_vo);
                cv::Rodrigues(imu_delta.deltaR, rv_gyro);

                if (gyro_bias_count_ < 200) {
                    cv::Mat bias_sample = rv_gyro - rv_vo;
                    double ab = (gyro_bias_count_ < 50) ? 0.05 : 0.01;
                    gyro_bias_ = (1.0 - ab) * gyro_bias_ + ab * bias_sample;
                    double bn = cv::norm(gyro_bias_);
                    if (bn > 0.02) gyro_bias_ *= (0.02 / bn);
                    gyro_bias_count_++;
                }
                cv::Mat rv_gyro_cor = rv_gyro - gyro_bias_;
                double adaptive_alpha;
                if (quality > 0.5) adaptive_alpha = 0.15;
                else if (quality > 0.3) {
                    double t = (quality - 0.3) / 0.2;
                    adaptive_alpha = 0.95 - t * 0.80;
                } else adaptive_alpha = 0.95;

                cv::Mat rv_fused = adaptive_alpha * rv_gyro_cor + (1.0 - adaptive_alpha) * rv_vo;
                cv::Rodrigues(rv_fused, R_fused);

                if (frame_counter_ % 90 == 0) {
                    LOGI("POSE: svdCond=%.0f transDegen=%d  FUSION: q=%.2f gyro=%.0f%% cam=%.0f%%  ekf=%.4f",
                         svd_cond, translation_degenerate ? 1 : 0,
                         quality, adaptive_alpha * 100.0, (1.0 - adaptive_alpha) * 100.0,
                         ekf_.getScale());
                }

                // ── Scale estimation → EKF (only with reliable translation) ─
                if (!translation_degenerate) {
                    double vo_dist = cv::norm(t_vo);
                    int64_t dt_ns_frame = timestamp_ns - current_prev_ts;
                    bool scale_ok = (quality > 0.15) && !is_pure_rotation && !is_static
                                 && (dt_ns_frame > 0) && (dt_ns_frame < 2'000'000'000LL);

                    if (scale_ok) {
                        double candidate = estimateScaleFromSteps(vo_dist, dt_ns_frame, imu);
                        if (candidate > 0.0) {
                            double dt_sec = dt_ns_frame * 1e-9;
                            auto si = imu.getStepInfo();
                            double step_disp = si.speed_mps * dt_sec;
                            double cam_disp = candidate * vo_dist;
                            double consistency = ekf_.checkConsistency(cam_disp, step_disp);

                            double scale_conf = consistency * quality;
                            ekf_.updateScale(candidate, scale_conf);

                            std::lock_guard<std::mutex> slock(pose_mutex_);
                            smooth_scale_ = ekf_.getScale();
                            scale_obs_count_++;
                            estimatedScale = smooth_scale_;
                            if (scale_obs_count_ <= 30) {
                                LOGI("BOOTSTRAP SCALE [%d/30]: cand=%.4f ekf=%.4f consist=%.2f",
                                     scale_obs_count_, candidate, smooth_scale_, consistency);
                            }
                        }
                    }
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

        if (pose_valid) {
            // Camera-fused rotation (includes translation-degenerate frames)
            global_R_ = global_R_ * R_fused;
        } else {
            // Apply Tracker's gyro bias correction to IMU rotation for fallback
            cv::Mat rv_gyro;
            cv::Rodrigues(imu_delta.deltaR, rv_gyro);
            cv::Mat rv_corrected = rv_gyro - gyro_bias_;
            cv::Mat R_corrected;
            cv::Rodrigues(rv_corrected, R_corrected);
            global_R_ = global_R_ * R_corrected;
        }

        double heading = std::atan2(global_R_.at<double>(1,0), global_R_.at<double>(0,0));

        if (is_static) {
            // ZUPT: freeze translation, rotation remains live
        } else if (pose_valid && !is_pure_rotation && !translation_degenerate && quality >= 0.15) {
            double disp = appliedScale * cv::norm(t_vo);

            // IMU cross-check: compare camera displacement direction with IMU deltaP
            if (imu_delta.dt > 0 && !imu_delta.deltaP.empty()) {
                double imu_disp = cv::norm(imu_delta.deltaP);
                // If IMU says minimal motion but camera says large, trust the smaller
                if (imu_disp > 0.001 && disp > 3.0 * imu_disp && imu_disp < 0.05) {
                    disp = std::min(disp, imu_disp * 2.0);
                }
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

        // Heading comes solely from global_R_ accumulation (camera-primary rotation fusion).
        // EKF heading calls (predict/updateHeading/updateZUPT) removed to prevent
        // cross-covariance leakage into the scale estimate.
    }

    // ── 10. Accel bias estimation (diagnostic, when static) ──────────────────
    if (is_static && accel_bias_count_ < ACCEL_BIAS_WARMUP && imu_delta.dt > 0.001) {
        cv::Mat bias_obs = imu_delta.deltaV / imu_delta.dt;
        accel_bias_ = (1.0 - ACCEL_BIAS_ALPHA) * accel_bias_ + ACCEL_BIAS_ALPHA * bias_obs;
        accel_bias_count_++;
    }

    // ── 11. Grid-based feature replenishment + keyframe re-localization ─────
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tracked < MIN_FEATURES) {
            new_pts_buf_.clear();
            feature_mgr_.replenishSparse(gray_buf_, next_good_buf_, new_pts_buf_,
                                          MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            next_good_buf_.insert(next_good_buf_.end(), new_pts_buf_.begin(), new_pts_buf_.end());
        }
        // Keyframe re-localization: if critically low after replenishment,
        // recover features by matching against the most recent keyframe
        if (static_cast<int>(next_good_buf_.size()) < MIN_FEATURES / 2) {
            std::vector<cv::Point2f> kf_matched, cur_matched;
            if (feature_mgr_.matchAgainstKeyframe(gray_buf_, next_good_buf_,
                                                    kf_matched, cur_matched)) {
                next_good_buf_ = cur_matched;
                LOGI("RELOCALIZE: recovered %zu pts from keyframe", cur_matched.size());
            }
        }
        // Keyframe storage (every 15 frames or when features drop significantly)
        // Inside mutex_ to protect feature_mgr_.keyframes_ from concurrent reset()
        frames_since_keyframe_++;
        if (frames_since_keyframe_ >= 15 || (tracked < MIN_FEATURES / 2 && frames_since_keyframe_ > 3)) {
            feature_mgr_.storeKeyframe(gray_buf_, next_good_buf_, timestamp_ns, frame_counter_);
            frames_since_keyframe_ = 0;
        }

        gray_buf_.copyTo(prev_gray_);
        prev_pts_ = next_good_buf_;
        prev_timestamp_ns_ = timestamp_ns;
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
        out.heading = std::atan2(global_R_.at<double>(1,0), global_R_.at<double>(0,0));
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
