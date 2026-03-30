#include "VisionModule.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <android/log.h>
#include <cmath>
#include <algorithm>

#define TAG "NavSight-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ── Constructor / Destructor ──────────────────────────────────────────────────

VisionModule::VisionModule()
    : prev_timestamp_ns_(0),
      initialized_(false),
      fx_(0.), fy_(0.), cx_(0.), cy_(0.)
{
    global_R_ = cv::Mat::eye(3, 3, CV_64F);
    global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    
    // Pre-reserve buffers to minimize re-allocations
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
    accel_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    gyro_bias_ = cv::Mat::zeros(3, 1, CV_64F);

    LOGI("VisionModule created");
}

VisionModule::~VisionModule() {
    LOGI("VisionModule destroyed");
}

// ── addGyroData ───────────────────────────────────────────────────────────────

void VisionModule::addGyroData(int64_t timestamp_ns, float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("addGyroData: NaN/Inf value, dropping");
        return;
    }
    imu_.addGyroReading(timestamp_ns, x, y, z);
}

// ── addAccelData ──────────────────────────────────────────────────────────────

void VisionModule::addAccelData(int64_t timestamp_ns, float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("addAccelData: NaN/Inf value, dropping");
        return;
    }
    imu_.addAccelReading(timestamp_ns, x, y, z);
}

// ── setIntrinsics ─────────────────────────────────────────────────────────────

void VisionModule::setIntrinsics(double fx, double fy, double cx, double cy) {
    std::lock_guard<std::mutex> lock(mutex_);
    fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    LOGI("setIntrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy);
}

// ── reset ─────────────────────────────────────────────────────────────────────

void VisionModule::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    prev_gray_.release();
    prev_pts_.clear();
    prev_timestamp_ns_ = 0;
    initialized_ = false;
    global_R_ = cv::Mat::eye(3, 3, CV_64F);
    global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    accel_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    accel_bias_count_ = 0;
    gyro_bias_ = cv::Mat::zeros(3, 1, CV_64F);
    gyro_bias_count_ = 0;
    last_imu_disp_ = 0.0;
    smooth_scale_ = 0.05;
    scale_obs_count_ = 0;
    frames_since_keyframe_ = 0;
    imu_.reset();
    LOGI("VisionModule reset");
}

// ── evictOldPoints (unused placeholder) ──────────────────────────────────────

void VisionModule::evictOldPoints() {
    // Points are managed directly inside processFrame; nothing to do here.
}

// ── calculateTrackingQuality ──────────────────────────────────────────────────

double VisionModule::calculateTrackingQuality(const std::vector<uchar>& status) {
    if (status.empty()) return 0.0;
    int good = 0;
    for (uchar s : status) {
        if (s) ++good;
    }
    return static_cast<double>(good) / static_cast<double>(status.size());
}

// ── estimateScaleFromAccel ────────────────────────────────────────────────────

double VisionModule::estimateScaleFromAccel(double vision_disp, int64_t dt_ns) {
    // Returns a candidate scale observation, or -1.0 if guards reject it.
    if (vision_disp < 1e-5) return -1.0;

    double dt = dt_ns * 1e-9;
    if (dt <= 0.0 || dt > 0.1) return -1.0; // only trust short intervals (<100ms)

    double imu_disp = last_imu_disp_;
    if (imu_disp < 1e-4) return -1.0;

    double obs_scale = imu_disp / vision_disp;

    // Clamp to physically plausible range first
    obs_scale = std::max(0.001, std::min(20.0, obs_scale));

    // Sanity: reject if more than 3x away from current smooth estimate
    // BUT allow the first 10 observations through unconditionally for bootstrap
    double current_smooth;
    int obs_count;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_smooth = smooth_scale_;
        obs_count = scale_obs_count_;
    }
    if (obs_count >= 10) {
        if (obs_scale > 3.0 * current_smooth || obs_scale < current_smooth / 3.0) {
            LOGD("estimateScaleFromAccel: outlier rejected obs=%.4f smooth=%.4f", obs_scale, current_smooth);
            return -1.0;
        }
    }

    return obs_scale;
}

// ── processFrame ─────────────────────────────────────────────────────────────

VisionOutput VisionModule::processFrame(const uint8_t* yuv_data,
                                        int width, int height,
                                        int64_t timestamp_ns) {
    VisionOutput out{};
    out.quality = 0.0;
    out.trackedCount = 0;
    out.totalCount = 0;
    out.estimatedScale = 1.0;
    out.valid = false;

    // ── 1. Guard invalid inputs ───────────────────────────────────────────────
    if (!yuv_data || width <= 0 || height <= 0) {
        LOGE("processFrame: invalid args (yuv_data=%p, %dx%d)", yuv_data, width, height);
        return out;
    }

    // ── 2. YUV NV21 → grayscale (Reuse gray_buf_) ────────────────────────────
    cv::Mat yuv(height + height / 2, width, CV_8UC1,
                const_cast<uint8_t*>(yuv_data));
    cv::cvtColor(yuv, gray_buf_, cv::COLOR_YUV2GRAY_NV21);

    // ── 3. Compute camera intrinsics ──────────────────────────────────────────
    double fx_use = (fx_ > 0.0) ? fx_ : 0.7 * width;
    double fy_use = (fy_ > 0.0) ? fy_ : fx_use;
    double cx_use = (cx_ > 0.0) ? cx_ : width  / 2.0;
    double cy_use = (cy_ > 0.0) ? cy_ : height / 2.0;

    cv::Mat K = (cv::Mat_<double>(3, 3)
                 << fx_use, 0.0,    cx_use,
                    0.0,    fy_use, cy_use,
                    0.0,    0.0,    1.0);

    // Context for compute (captured under lock then processed outside)
    cv::Mat current_prev_gray;
    int64_t current_prev_ts = 0;
    bool was_initialized = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        was_initialized = initialized_;
        if (!was_initialized || prev_pts_.empty()) {
            cv::goodFeaturesToTrack(gray_buf_, prev_pts_, MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            gray_buf_.copyTo(prev_gray_);
            prev_timestamp_ns_ = timestamp_ns;
            initialized_ = true;
            
            // FR15: Auto-initialize gravity vector if not already done
            if (!imu_.isInitialized()) {
                // Initialize with current static reading if available
                imu_.setGravity(imu_.lastAccelX(), imu_.lastAccelY(), imu_.lastAccelZ());
            }
            
            LOGI("processFrame: first frame, detected %zu features", prev_pts_.size());
            return out;
        }
        current_prev_gray = prev_gray_; // Shallow copy (ref-counted)
        current_prev_pts_buf_  = prev_pts_; // Copy into member buffer (reuses capacity)
        current_prev_ts   = prev_timestamp_ns_;
    }

    // ── 5. Integrate IMU (HEAVY COMPUTE - OUTSIDE MAIN LOCK) ────────────────
    PreintegratedDelta imu_delta = imu_.integrate(current_prev_ts, timestamp_ns);

    // ── Drift-Kill detection ────────────────────────────────────────────────
    double gyro_norm = 0.0;
    {
        // Calculate average gyro magnitude from preintegrated interval
        cv::Mat rot_vec;
        cv::Rodrigues(imu_delta.deltaR, rot_vec);
        if (imu_delta.dt > 0) gyro_norm = cv::norm(rot_vec) / imu_delta.dt;
    }

    bool is_pure_rotation = (gyro_norm > GYRO_ROT_ONLY_THRESH);

    // ZUPT: tentatively static if gyro is very low (overridden by vision below)
    bool is_static = (gyro_norm < ZUPT_GYRO_THRESH);

    // ── 6. Optical Flow with Forward-Backward Check ─────────────────────────
    next_pts_buf_.clear();
    status_buf_.clear();
    err_buf_.clear();

    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(current_prev_gray, gray_buf_, current_prev_pts_buf_, next_pts_buf_, status_buf_, err_buf_,
                             cv::Size(21, 21), 3, criteria);

    // Forward-backward consistency: track back from next to prev
    back_pts_buf_.clear();
    back_status_buf_.clear();
    back_err_buf_.clear();
    cv::calcOpticalFlowPyrLK(gray_buf_, current_prev_gray, next_pts_buf_, back_pts_buf_, back_status_buf_, back_err_buf_,
                             cv::Size(21, 21), 3, criteria);

    // ── 7. Filter valid points with FB check ────────────────────────────────
    prev_good_buf_.clear();
    next_good_buf_.clear();
    for (size_t i = 0; i < status_buf_.size(); ++i) {
        if (status_buf_[i] && i < back_status_buf_.size() && back_status_buf_[i]) {
            float bx = current_prev_pts_buf_[i].x - back_pts_buf_[i].x;
            float by = current_prev_pts_buf_[i].y - back_pts_buf_[i].y;
            if ((bx * bx + by * by) < FB_CHECK_THRESH) {
                prev_good_buf_.push_back(current_prev_pts_buf_[i]);
                next_good_buf_.push_back(next_pts_buf_[i]);
            }
        }
    }

    int tracked = static_cast<int>(next_good_buf_.size());
    int total   = static_cast<int>(current_prev_pts_buf_.size());
    double quality = (total > 0) ? static_cast<double>(tracked) / static_cast<double>(total) : 0.0;

    std::vector<float> tracked_points_flat;
    tracked_points_flat.reserve(next_good_buf_.size() * 2);
    for (const auto& pt : next_good_buf_) {
        tracked_points_flat.push_back(pt.x);
        tracked_points_flat.push_back(pt.y);
    }

    // ── 8. Minimum motion + parallax gate ───────────────────────────────────
    double mean_flow = 0.0;
    if (!prev_good_buf_.empty()) {
        for (size_t i = 0; i < prev_good_buf_.size(); ++i) {
            double fdx = next_good_buf_[i].x - prev_good_buf_[i].x;
            double fdy = next_good_buf_[i].y - prev_good_buf_[i].y;
            mean_flow += std::sqrt(fdx * fdx + fdy * fdy);
        }
        mean_flow /= static_cast<double>(prev_good_buf_.size());
    }
    bool sufficient_motion = (mean_flow >= MIN_FLOW_PX);
    bool has_parallax = (mean_flow >= MIN_PARALLAX_PX);

    // ZUPT requires BOTH low gyro AND low visual motion
    // Walking forward with zero rotation has low gyro but high flow — NOT static
    // Use 0.5px threshold to avoid dead zone between 0.5 and MIN_FLOW_PX
    if (mean_flow >= 0.5) is_static = false;  // any visible flow = not static
    if (mean_flow < 0.5 && gyro_norm < ZUPT_GYRO_THRESH) is_static = true; // sub-pixel flow + low gyro = static

    // ── 9. Essential Matrix with strict validation ──────────────────────────
    bool pose_valid = false;
    cv::Mat R_vo, t_vo, R_fused;
    double estimatedScale = 1.0;

    if (sufficient_motion && has_parallax && !is_static
        && static_cast<int>(prev_good_buf_.size()) >= 8) {
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(prev_good_buf_, next_good_buf_, K,
                                         cv::RANSAC, RANSAC_CONF, RANSAC_THRESH, mask);

        if (!E.empty() && E.rows == 3 && E.cols == 3) {
            int inliers = cv::recoverPose(E, prev_good_buf_, next_good_buf_, K, R_vo, t_vo, mask);
            double inlier_ratio = static_cast<double>(inliers) / static_cast<double>(prev_good_buf_.size());

            if (inliers >= MIN_INLIERS && inlier_ratio >= MIN_INLIER_RATIO) {
                // ── Adaptive rotation fusion ────────────────────────────────
                cv::Mat rot_vec_vo, rot_vec_gyro;
                cv::Rodrigues(R_vo, rot_vec_vo);
                cv::Rodrigues(imu_delta.deltaR, rot_vec_gyro);

                // Gyro bias estimation
                if (gyro_bias_count_ < 200) {
                    cv::Mat bias_sample = rot_vec_gyro - rot_vec_vo;
                    double alpha_bias = (gyro_bias_count_ < 50) ? 0.05 : 0.01;
                    gyro_bias_ = (1.0 - alpha_bias) * gyro_bias_ + alpha_bias * bias_sample;
                    // Clamp bias magnitude to 0.02 rad/s (real phone bias is tiny)
                    double bias_norm = cv::norm(gyro_bias_);
                    if (bias_norm > 0.02) gyro_bias_ *= (0.02 / bias_norm);
                    gyro_bias_count_++;
                }
                cv::Mat rot_vec_gyro_corrected = rot_vec_gyro - gyro_bias_;

                // FR4: Adaptive alpha — camera primary when quality is high
                double adaptive_alpha = 0.98;
                if (quality > 0.7) {
                    adaptive_alpha = 0.85;
                } else if (quality < 0.3) {
                    adaptive_alpha = 0.995;
                } else {
                    double t = (quality - 0.3) / 0.4;
                    adaptive_alpha = 0.995 - t * (0.995 - 0.85);
                }

                cv::Mat rot_vec_fused = adaptive_alpha * rot_vec_gyro_corrected
                                      + (1.0 - adaptive_alpha) * rot_vec_vo;
                cv::Rodrigues(rot_vec_fused, R_fused);

                // ── Scale estimation (GUARDED) ──────────────────────────────
                {
                    std::lock_guard<std::mutex> slock(pose_mutex_);
                    last_imu_disp_ = cv::norm(imu_delta.deltaP);
                }
                double vo_dist = cv::norm(t_vo);
                int64_t dt_ns_frame = timestamp_ns - current_prev_ts;

                bool scale_ok = (quality > 0.5) && (!is_pure_rotation) && (!is_static)
                             && (dt_ns_frame > 0) && (dt_ns_frame < 100'000'000LL);

                if (scale_ok) {
                    double candidate = estimateScaleFromAccel(vo_dist, dt_ns_frame);
                    if (candidate > 0.0) {
                        std::lock_guard<std::mutex> slock(pose_mutex_);
                        smooth_scale_ = 0.95 * smooth_scale_ + 0.05 * candidate;
                        smooth_scale_ = std::max(0.005, std::min(20.0, smooth_scale_));
                        scale_obs_count_++;
                        estimatedScale = smooth_scale_;
                        LOGI("SCALE: candidate=%.4f smooth=%.4f quality=%.2f obs=%d",
                             candidate, smooth_scale_, quality, scale_obs_count_);
                    } else {
                        std::lock_guard<std::mutex> slock(pose_mutex_);
                        estimatedScale = smooth_scale_;
                    }
                } else {
                    std::lock_guard<std::mutex> slock(pose_mutex_);
                    estimatedScale = smooth_scale_;
                }

                pose_valid = true;
            }
        }
    }

    // ── 10. Global pose update — CAMERA PRIMARY, IMU rotation only ──────────
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        if (is_static) {
            // ZUPT: freeze pose completely — no drift
        } else if (pose_valid && !is_pure_rotation) {
            // Full VIO: camera translation * scale, fused rotation
            // NEVER use imu_delta.deltaP for position
            cv::Mat final_t = global_R_ * (estimatedScale * t_vo);
            global_t_ += final_t;
            global_R_  = R_fused * global_R_;
        } else if (pose_valid && is_pure_rotation) {
            // Pure rotation: update rotation only, hold position
            global_R_ = R_fused * global_R_;
        } else {
            // VO failed: rotation from gyro only, HOLD position
            global_R_ = imu_delta.deltaR * global_R_;
        }
    }

    // ── 11. Accel bias estimation (diagnostic, when static) ─────────────────
    if (is_static && accel_bias_count_ < ACCEL_BIAS_WARMUP && imu_delta.dt > 0.001) {
        cv::Mat bias_obs = imu_delta.deltaV / imu_delta.dt;
        accel_bias_ = (1.0 - ACCEL_BIAS_ALPHA) * accel_bias_ + ACCEL_BIAS_ALPHA * bias_obs;
        accel_bias_count_++;
    }

    // ── 18. Update internal state for next frame (SHORT LOCK) ───────────────
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tracked < MIN_FEATURES) {
            new_pts_buf_.clear();
            cv::goodFeaturesToTrack(gray_buf_, new_pts_buf_, MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            next_good_buf_.insert(next_good_buf_.end(), new_pts_buf_.begin(), new_pts_buf_.end());
        }
        gray_buf_.copyTo(prev_gray_);
        prev_pts_  = next_good_buf_; // Vector copy (still necessary if not swapping)
        prev_timestamp_ns_ = timestamp_ns;
    }

    // Read state for output
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        out.R = global_R_.clone(); // Return global orientation
        out.t = global_t_.clone(); // Return global position
    }
    out.quality       = quality;
    out.trackedCount  = tracked;
    out.totalCount    = total;
    out.estimatedScale = estimatedScale;
    out.valid         = true;
    out.trackedPoints = std::move(tracked_points_flat);

    return out;
}
