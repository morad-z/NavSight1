#include "FeatureManager.h"
#include <opencv2/video/tracking.hpp>
#include <android/log.h>
#include <algorithm>
#include <cmath>

#define TAG "NavSight-Features"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

FeatureManager::FeatureManager() {
    keyframes_.reserve(MAX_KEYFRAMES);
}

void FeatureManager::reset() {
    keyframes_.clear();
}

int FeatureManager::countInCell(const std::vector<cv::Point2f>& points,
                                 int row, int col, int cell_w, int cell_h) const {
    int x0 = col * cell_w, y0 = row * cell_h;
    int x1 = x0 + cell_w, y1 = y0 + cell_h;
    int count = 0;
    for (const auto& p : points) {
        if (p.x >= x0 && p.x < x1 && p.y >= y0 && p.y < y1)
            count++;
    }
    return count;
}

// ── Grid-based feature detection ────────────────────────────────────────────

void FeatureManager::detectGridFeatures(const cv::Mat& gray,
                                         std::vector<cv::Point2f>& out_points,
                                         int max_total, double quality_level,
                                         double min_dist) {
    out_points.clear();
    if (gray.empty()) return;

    int cell_w = gray.cols / GRID_COLS;
    int cell_h = gray.rows / GRID_ROWS;
    int max_per_cell = max_total / (GRID_ROWS * GRID_COLS) + 1;

    std::vector<cv::Point2f> cell_pts;

    for (int r = 0; r < GRID_ROWS; r++) {
        for (int c = 0; c < GRID_COLS; c++) {
            int x0 = c * cell_w;
            int y0 = r * cell_h;
            int w = (c == GRID_COLS - 1) ? (gray.cols - x0) : cell_w;
            int h = (r == GRID_ROWS - 1) ? (gray.rows - y0) : cell_h;

            cv::Rect roi(x0, y0, w, h);
            cv::Mat cell = gray(roi);

            cell_pts.clear();
            cv::goodFeaturesToTrack(cell, cell_pts, max_per_cell,
                                    quality_level, min_dist);

            // Offset to global coordinates
            for (auto& pt : cell_pts) {
                pt.x += static_cast<float>(x0);
                pt.y += static_cast<float>(y0);
            }
            out_points.insert(out_points.end(), cell_pts.begin(), cell_pts.end());
        }
    }

    // Sub-pixel refinement on all detected points
    if (!out_points.empty()) {
        cv::cornerSubPix(gray, out_points, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
    }

    // Trim to max if we got too many
    if (static_cast<int>(out_points.size()) > max_total) {
        out_points.resize(max_total);
    }
}

// ── Replenish sparse cells ──────────────────────────────────────────────────

void FeatureManager::replenishSparse(const cv::Mat& gray,
                                      const std::vector<cv::Point2f>& existing,
                                      std::vector<cv::Point2f>& out_new,
                                      int max_total, double quality_level,
                                      double min_dist) {
    out_new.clear();
    if (gray.empty()) return;

    int cell_w = gray.cols / GRID_COLS;
    int cell_h = gray.rows / GRID_ROWS;
    int target_per_cell = max_total / (GRID_ROWS * GRID_COLS);
    int remaining = max_total - static_cast<int>(existing.size());
    if (remaining <= 0) return;

    std::vector<cv::Point2f> cell_pts;

    for (int r = 0; r < GRID_ROWS && remaining > 0; r++) {
        for (int c = 0; c < GRID_COLS && remaining > 0; c++) {
            int current = countInCell(existing, r, c, cell_w, cell_h);
            int deficit = target_per_cell - current;
            if (deficit <= 0) continue;

            int x0 = c * cell_w;
            int y0 = r * cell_h;
            int w = (c == GRID_COLS - 1) ? (gray.cols - x0) : cell_w;
            int h = (r == GRID_ROWS - 1) ? (gray.rows - y0) : cell_h;

            cv::Rect roi(x0, y0, w, h);
            cv::Mat cell = gray(roi);

            cell_pts.clear();
            cv::goodFeaturesToTrack(cell, cell_pts,
                                    std::min(deficit, remaining),
                                    quality_level, min_dist);

            for (auto& pt : cell_pts) {
                pt.x += static_cast<float>(x0);
                pt.y += static_cast<float>(y0);
            }

            int to_add = std::min(static_cast<int>(cell_pts.size()), remaining);
            out_new.insert(out_new.end(), cell_pts.begin(),
                          cell_pts.begin() + to_add);
            remaining -= to_add;
        }
    }

    if (!out_new.empty()) {
        cv::cornerSubPix(gray, out_new, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
    }
}

// ── Keyframe management ─────────────────────────────────────────────────────

void FeatureManager::storeKeyframe(const cv::Mat& gray,
                                    const std::vector<cv::Point2f>& points,
                                    int64_t timestamp_ns, int frame_id) {
    Keyframe kf;
    kf.gray = gray.clone();
    kf.points = points;
    kf.timestamp_ns = timestamp_ns;
    kf.frame_id = frame_id;

    keyframes_.push_back(std::move(kf));
    if (static_cast<int>(keyframes_.size()) > MAX_KEYFRAMES) {
        keyframes_.erase(keyframes_.begin());
    }
}

bool FeatureManager::matchAgainstKeyframe(
        const cv::Mat& gray,
        const std::vector<cv::Point2f>& current_points,
        std::vector<cv::Point2f>& kf_matched,
        std::vector<cv::Point2f>& cur_matched) const {

    kf_matched.clear();
    cur_matched.clear();

    if (keyframes_.empty() || current_points.empty()) return false;

    // Match against the most recent keyframe using optical flow
    const Keyframe& kf = keyframes_.back();
    if (kf.points.empty()) return false;

    std::vector<cv::Point2f> tracked;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(kf.gray, gray, kf.points, tracked,
                              status, err, cv::Size(31, 31), 4, criteria);

    // Collect valid matches
    for (size_t i = 0; i < status.size(); i++) {
        if (!status[i]) continue;
        float dx = tracked[i].x - kf.points[i].x;
        float dy = tracked[i].y - kf.points[i].y;
        if (std::sqrt(dx * dx + dy * dy) > KF_MATCH_RADIUS * 10) continue;  // reject huge jumps

        kf_matched.push_back(kf.points[i]);
        cur_matched.push_back(tracked[i]);
    }

    return static_cast<int>(kf_matched.size()) >= MIN_KF_MATCHES;
}
