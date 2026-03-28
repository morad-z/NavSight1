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

    // ── 2. YUV NV21 → grayscale ───────────────────────────────────────────────
    cv::Mat yuv(height + height / 2, width, CV_8UC1,
                const_cast<uint8_t*>(yuv_data));
    cv::Mat gray;
    cv::cvtColor(yuv, gray, cv::COLOR_YUV2GRAY_NV21);

    // ── 3. Compute camera intrinsics ──────────────────────────────────────────
    double fx_use = (fx_ > 0.0) ? fx_ : 0.7 * width;
    double fy_use = (fy_ > 0.0) ? fy_ : fx_use;
    double cx_use = (cx_ > 0.0) ? cx_ : width  / 2.0;
    double cy_use = (cy_ > 0.0) ? cy_ : height / 2.0;

    cv::Mat K = (cv::Mat_<double>(3, 3)
                 << fx_use, 0.0,    cx_use,
                    0.0,    fy_use, cy_use,
                    0.0,    0.0,    1.0);

    std::lock_guard<std::mutex> lock(mutex_);

    // ── 4. Initialise on first frame (or if no features remain) ───────────────
    if (!initialized_ || prev_pts_.empty()) {
        cv::goodFeaturesToTrack(gray, prev_pts_, MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
        prev_gray_ = gray.clone();
        prev_timestamp_ns_ = timestamp_ns;
        initialized_ = true;
        LOGI("processFrame: first frame, detected %zu features", prev_pts_.size());
        return out; // need two frames
    }

    // ── 5. Integrate gyro between frames ─────────────────────────────────────
    cv::Mat delta_R_gyro = imu_.integrateGyro(prev_timestamp_ns_, timestamp_ns);

    // ── 6. Lucas-Kanade optical flow ──────────────────────────────────────────
    std::vector<cv::Point2f> next_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts_, next_pts, status, err);

    // ── 7. Filter valid points ────────────────────────────────────────────────
    std::vector<cv::Point2f> prev_good, next_good;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            prev_good.push_back(prev_pts_[i]);
            next_good.push_back(next_pts[i]);
        }
    }

    int tracked = static_cast<int>(next_good.size());
    int total   = static_cast<int>(prev_pts_.size());
    double quality = calculateTrackingQuality(status);

    LOGD("VIO Thread: Features tracked: %d / %d", tracked, total);

    // ── 8. Build trackedPoints flat array [x0,y0,x1,y1,...] ─────────────────
    std::vector<float> tracked_points_flat;
    tracked_points_flat.reserve(next_good.size() * 2);
    for (const auto& pt : next_good) {
        tracked_points_flat.push_back(pt.x);
        tracked_points_flat.push_back(pt.y);
    }

    // ── 9. Need ≥ 8 matched pairs for essential matrix ───────────────────────
    // NOTE: replenishment happens AFTER pose estimation so prev/next stay aligned
    if (static_cast<int>(prev_good.size()) < 8) {
        // Replenish before storing next frame's points
        if (tracked < MIN_FEATURES) {
            std::vector<cv::Point2f> new_pts;
            cv::goodFeaturesToTrack(gray, new_pts, MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
            next_good.insert(next_good.end(), new_pts.begin(), new_pts.end());
        }
        prev_gray_ = gray.clone();
        prev_pts_  = next_good;
        prev_timestamp_ns_ = timestamp_ns;
        out.trackedCount  = tracked;
        out.totalCount    = total;
        out.quality       = quality;
        out.trackedPoints = tracked_points_flat;
        return out;
    }

    // ── 11. Essential matrix via RANSAC ───────────────────────────────────────
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(prev_good, next_good, K,
                                     cv::RANSAC, RANSAC_CONF, RANSAC_THRESH, mask);

    // ── 12. Validate E is exactly 3×3 ────────────────────────────────────────
    if (E.empty() || E.rows != 3 || E.cols != 3) {
        LOGE("processFrame: Essential Matrix invalid (empty=%d size=%dx%d)",
             E.empty(), E.rows, E.cols);
        prev_gray_ = gray.clone();
        prev_pts_  = next_good;
        prev_timestamp_ns_ = timestamp_ns;
        out.trackedCount  = tracked;
        out.totalCount    = total;
        out.trackedPoints = tracked_points_flat;
        return out;
    }

    // ── 13. recoverPose ───────────────────────────────────────────────────────
    cv::Mat R_vo, t_vo;
    cv::recoverPose(E, prev_good, next_good, K, R_vo, t_vo, mask);
    LOGI("Essential Matrix found!");

    // ── 14-15. Complementary filter fusion ───────────────────────────────────
    cv::Mat rot_vec_vo, rot_vec_gyro;
    cv::Rodrigues(R_vo, rot_vec_vo);
    cv::Rodrigues(delta_R_gyro, rot_vec_gyro);

    cv::Mat rot_vec_fused = ALPHA_FUSION * rot_vec_gyro
                          + (1.0 - ALPHA_FUSION) * rot_vec_vo;

    cv::Mat R_fused;
    cv::Rodrigues(rot_vec_fused, R_fused);

    // ── 16. Validate dt ───────────────────────────────────────────────────────
    int64_t dt_ns = timestamp_ns - prev_timestamp_ns_;
    double estimatedScale = estimateScaleFromAccel(cv::norm(t_vo), dt_ns);

    if (dt_ns <= 0 || dt_ns >= MAX_DT_NS) {
        LOGE("processFrame: dt_ns out of range (%lld), skipping pose update",
             (long long)dt_ns);
        prev_gray_ = gray.clone();
        prev_pts_  = next_good;
        prev_timestamp_ns_ = timestamp_ns;
        out.trackedCount  = tracked;
        out.totalCount    = total;
        out.quality       = quality;
        out.trackedPoints = tracked_points_flat;
        return out;
    }

    // ── 17. Global pose update ────────────────────────────────────────────────
    global_t_ += estimatedScale * (global_R_ * t_vo);
    global_R_  = R_fused * global_R_;

    LOGD("Pose: x=%.3f y=%.3f z=%.3f",
         global_t_.at<double>(0),
         global_t_.at<double>(1),
         global_t_.at<double>(2));

    // ── 18. Update tracking state for next frame ──────────────────────────────
    // Replenish features for next frame if needed (after pose est — no size mismatch risk)
    if (tracked < MIN_FEATURES) {
        std::vector<cv::Point2f> new_pts;
        cv::goodFeaturesToTrack(gray, new_pts, MAX_FEATURES, QUALITY_LEVEL, MIN_DIST);
        next_good.insert(next_good.end(), new_pts.begin(), new_pts.end());
    }
    prev_gray_ = gray.clone();
    prev_pts_  = next_good;
    prev_timestamp_ns_ = timestamp_ns;

    // ── 19-20. Fill output ────────────────────────────────────────────────────
    out.R             = R_fused;
    out.t             = t_vo;
    out.quality       = quality;
    out.trackedCount  = tracked;
    out.totalCount    = total;
    out.estimatedScale = estimatedScale;
    out.valid         = true;
    out.trackedPoints = tracked_points_flat;

    return out;
}
