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

double VisionModule::estimateScaleFromAccel(double /*vision_disp*/, int64_t /*dt_ns*/) {
    // Placeholder: double-integration of accelerometer is noisy.
    // Return baseline 1.0 (clamped to [0.1, 10.0]).
    return 1.0;
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
    
    // ZUPT: If accel is near gravity and gyro is very small, we are static
    bool is_static = false;
    double accel_dev = std::abs(cv::norm(imu_delta.deltaV) / std::max(0.001, imu_delta.dt));
    if (accel_dev < ZUPT_ACCEL_THRESH && gyro_norm < ZUPT_GYRO_THRESH) {
        is_static = true;
        imu_delta.deltaP = cv::Mat::zeros(3, 1, CV_64F);
        imu_delta.deltaV = cv::Mat::zeros(3, 1, CV_64F);
    }

    // ── 6. Optical Flow (HEAVY COMPUTE - OUTSIDE MAIN LOCK) ─────────────────
    // Reuse buffers and set explicit TermCriteria
    next_pts_buf_.clear();
    status_buf_.clear();
    err_buf_.clear();
    
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(current_prev_gray, gray_buf_, current_prev_pts_buf_, next_pts_buf_, status_buf_, err_buf_, 
                             cv::Size(21, 21), 3, criteria);

    // ── 7. Filter valid points ────────────────────────────────────────────────
    prev_good_buf_.clear();
    next_good_buf_.clear();
    for (size_t i = 0; i < status_buf_.size(); ++i) {
        if (status_buf_[i]) {
            prev_good_buf_.push_back(current_prev_pts_buf_[i]);
            next_good_buf_.push_back(next_pts_buf_[i]);
        }
    }

    int tracked = static_cast<int>(next_good_buf_.size());
    int total   = static_cast<int>(current_prev_pts_buf_.size());
    double quality = calculateTrackingQuality(status_buf_);

    std::vector<float> tracked_points_flat;
    tracked_points_flat.reserve(next_good_buf_.size() * 2);
    for (const auto& pt : next_good_buf_) {
        tracked_points_flat.push_back(pt.x);
        tracked_points_flat.push_back(pt.y);
    }

    // ── 9. Essential Matrix Logic (OUTSIDE MAIN LOCK) ──────────────────────
    bool pose_valid = false;
    cv::Mat R_vo, t_vo, R_fused;
    double estimatedScale = 1.0;

    if (static_cast<int>(prev_good_buf_.size()) >= 8) {
        cv::Mat mask;
        cv::Mat E = cv::findEssentialMat(prev_good_buf_, next_good_buf_, K,
                                         cv::RANSAC, RANSAC_CONF, RANSAC_THRESH, mask);

        if (!E.empty() && E.rows == 3 && E.cols == 3) {
            if (cv::recoverPose(E, prev_good_buf_, next_good_buf_, K, R_vo, t_vo, mask) > 0) {
                // ── 14-15. Adaptive fusion based on tracking quality ───────────────────
                cv::Mat rot_vec_vo, rot_vec_gyro;
                cv::Rodrigues(R_vo, rot_vec_vo);
                cv::Rodrigues(imu_delta.deltaR, rot_vec_gyro);

                // FR4: Dynamic Alpha. High quality = camera more main (lower alpha). 
                // Low quality = sensors more main (higher alpha).
                double adaptive_alpha = 0.98; // Default
                if (quality > 0.7) {
                    adaptive_alpha = 0.85; // Camera is strong, let it correct gyro faster
                } else if (quality < 0.3) {
                    adaptive_alpha = 0.995; // Camera is weak (dark/blurry), trust sensors almost entirely
                } else {
                    // Linear interpolation between 0.3 and 0.7 quality
                    double t = (quality - 0.3) / 0.4;
                    adaptive_alpha = 0.995 - t * (0.995 - 0.85);
                }

                LOGI("FUSION: quality=%.2f adaptive_alpha=%.3f", quality, adaptive_alpha);

                cv::Mat rot_vec_fused = adaptive_alpha * rot_vec_gyro
                                      + (1.0 - adaptive_alpha) * rot_vec_vo;
                cv::Rodrigues(rot_vec_fused, R_fused);

                // ── 16. Scale estimation ────────────────────────────────────
                double imu_dist = cv::norm(imu_delta.deltaP);
                double vo_dist = cv::norm(t_vo);
                if (vo_dist > 1e-5) {
                    double obs_scale = imu_dist / vo_dist;
                    // Update class member with smoothing
                    {
                        std::lock_guard<std::mutex> lock(pose_mutex_);
                        smooth_scale_ = 0.9 * smooth_scale_ + 0.1 * obs_scale;
                        smooth_scale_ = std::max(0.1, std::min(10.0, smooth_scale_));
                        estimatedScale = smooth_scale_;
                    }
                }
                pose_valid = true;
            }
        }
    }

    // ── 17. Global pose update (GRANULAR LOCK) ──────────────────────────────
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (pose_valid) {
            // Camera + IMU fusion
            cv::Mat final_t = global_R_ * (estimatedScale * t_vo);
            
            // If we are rotating fast, ignore camera's noisy translation (it's degenerate)
            if (is_pure_rotation) {
                global_t_ += (global_R_ * imu_delta.deltaP);
            } else {
                global_t_ += 0.7 * (global_R_ * imu_delta.deltaP) + 0.3 * final_t;
            }
            global_R_  = R_fused * global_R_;
        } else {
            // Pure IMU dead reckoning if VO fails (or if static - imu_delta is zeroed by ZUPT)
            global_t_ += (global_R_ * imu_delta.deltaP);
            global_R_  = imu_delta.deltaR * global_R_;
        }
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
