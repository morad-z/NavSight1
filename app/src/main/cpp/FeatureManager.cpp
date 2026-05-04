#include "FeatureManager.h"
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <unordered_set>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Features"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#endif

FeatureManager::FeatureManager() {
    keyframes_.reserve(MAX_KEYFRAMES);
}

void FeatureManager::reset() {
    keyframes_.clear();
    keyframe_descriptors_.clear();
    active_tracks_.clear();
    lifecycle_.clear();
    next_feature_id_ = 0;
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
                                    int64_t timestamp_ns, int frame_id,
                                    double heading, cv::Point3f position) {
    Keyframe kf;
    kf.gray = gray.clone();
    kf.points = points;
    kf.timestamp_ns = timestamp_ns;
    kf.frame_id = frame_id;
    kf.heading = heading;
    kf.position = position;

    keyframes_.push_back(std::move(kf));
    if (static_cast<int>(keyframes_.size()) > MAX_KEYFRAMES) {
        keyframes_.erase(keyframes_.begin());
    }
}

// ── MSCKF Feature Track Management ──────────────────────────────────────────

std::vector<int> FeatureManager::assignIds(int count) {
    std::vector<int> ids(count);
    for (int i = 0; i < count; i++) {
        ids[i] = next_feature_id_++;
    }
    return ids;
}

void FeatureManager::addObservation(int feature_id, int clone_state_id,
                                     const cv::Point2f& pixel_ud) {
    active_tracks_[feature_id].push_back({clone_state_id, pixel_ud});
}

std::vector<LostFeature> FeatureManager::extractLostFeatures(
        const std::vector<int>& current_ids, int min_obs) {
    // Build set of currently tracked IDs
    std::unordered_set<int> current_set(current_ids.begin(), current_ids.end());

    std::vector<LostFeature> lost;
    auto it = active_tracks_.begin();
    while (it != active_tracks_.end()) {
        if (current_set.count(it->first) == 0) {
            // Feature is no longer tracked
            if (static_cast<int>(it->second.size()) >= min_obs) {
                LostFeature lf;
                lf.feature_id = it->first;
                lf.observations = std::move(it->second);
                lost.push_back(std::move(lf));
            }
            it = active_tracks_.erase(it);
        } else {
            ++it;
        }
    }
    return lost;
}

void FeatureManager::pruneObservations(int min_clone_id) {
    for (auto& [fid, obs] : active_tracks_) {
        obs.erase(std::remove_if(obs.begin(), obs.end(),
            [min_clone_id](const FeatureObservation& o) {
                return o.clone_state_id < min_clone_id;
            }), obs.end());
    }
    // Remove features with no remaining observations
    auto it = active_tracks_.begin();
    while (it != active_tracks_.end()) {
        if (it->second.empty()) {
            it = active_tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

// ── Keyframe re-localization ────────────────────────────────────────────────

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

    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.03);
    cv::calcOpticalFlowPyrLK(kf.gray, gray, kf.points, tracked,
                              status, err, cv::Size(21, 21), 3, criteria);

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

// ──────────────────────────────────────────────────────────────────────────────
// Plan Step 3b (ADR-009): SLAM feature lifecycle bookkeeping
// ──────────────────────────────────────────────────────────────────────────────

void FeatureManager::noteObservation(int feature_id, int64_t ts_ns,
                                     bool is_keyframe) {
    auto& lc = lifecycle_[feature_id];
    if (lc.feature_id == -1) {
        lc.feature_id = feature_id;
    }
    lc.age++;
    if (is_keyframe) lc.kf_count++;
    lc.last_obs_ns = ts_ns;
}

void FeatureManager::noteKeyframe(int feature_id) {
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return;
    it->second.kf_count++;
}

void FeatureManager::noteTriangulation(int feature_id,
                                       const cv::Point3f& p_global,
                                       int anchor_clone_id) {
    auto& lc = lifecycle_[feature_id];
    if (lc.feature_id == -1) {
        lc.feature_id = feature_id;
    }
    lc.last_p_global   = p_global;
    lc.has_p_global    = true;
    lc.anchor_clone_id = anchor_clone_id;
}

void FeatureManager::setSlamSlot(int feature_id, int slam_slot) {
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return;
    it->second.slam_slot = slam_slot;
    if (slam_slot < 0) {
        it->second.rms_bad_consecutive = 0;
    }
}

std::vector<int> FeatureManager::getPromotableFeatures(
        int min_obs, int min_kf, double max_init_rms_px) const {
    std::vector<int> out;
    out.reserve(8);
    for (const auto& [fid, lc] : lifecycle_) {
        if (lc.slam_slot >= 0)               continue;     // already promoted
        if (lc.age      < min_obs)           continue;
        if (lc.kf_count < min_kf)            continue;
        if (!lc.has_p_global)                continue;
        if (lc.last_rms_px > max_init_rms_px && lc.last_rms_px > 0.0) continue;
        out.push_back(fid);
    }
    return out;
}

void FeatureManager::markSlamFeatureRMS(int feature_id, double rms_px) {
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return;
    it->second.last_rms_px = rms_px;
    if (rms_px > 3.0) {
        it->second.rms_bad_consecutive++;
    } else {
        it->second.rms_bad_consecutive = 0;
    }
}

std::vector<int> FeatureManager::getDemoteCandidates() const {
    std::vector<int> out;
    for (const auto& [fid, lc] : lifecycle_) {
        if (lc.slam_slot >= 0 && lc.rms_bad_consecutive >= 3) {
            out.push_back(fid);
        }
    }
    return out;
}

std::vector<int> FeatureManager::getLostSlamFeatures(
        int64_t now_ns, int64_t lost_threshold_ns) const {
    std::vector<int> out;
    for (const auto& [fid, lc] : lifecycle_) {
        if (lc.slam_slot < 0)              continue;
        if (lc.last_obs_ns == 0)           continue;  // never observed yet
        if (now_ns - lc.last_obs_ns > lost_threshold_ns) {
            out.push_back(fid);
        }
    }
    return out;
}

const FeatureManager::FeatureLifecycle*
FeatureManager::getLifecycle(int feature_id) const {
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return nullptr;
    return &it->second;
}

const std::vector<FeatureObservation>*
FeatureManager::getObservations(int feature_id) const {
    auto it = active_tracks_.find(feature_id);
    if (it == active_tracks_.end()) return nullptr;
    return &it->second;
}

void FeatureManager::dropLifecycle(int feature_id) {
    lifecycle_.erase(feature_id);
}

std::vector<int> FeatureManager::getAllLifecycleFeatureIds() const {
    std::vector<int> out;
    out.reserve(lifecycle_.size());
    for (const auto& kv : lifecycle_) out.push_back(kv.first);
    return out;
}

// ──────────────────────────────────────────────────────────────────────────────
// Plan Step 4 (ADR-010): ORB descriptors at keyframes
// ──────────────────────────────────────────────────────────────────────────────
//
// Lifecycle: Tracker calls storeKeyframeDescriptors every time it stores a
// keyframe, immediately after the existing storeKeyframe(). The Tracker
// relocalization path reads getKeyframeDescriptors() when its KLT inlier
// counter trips the trigger condition (MIN_INLIERS/2 for ≥ 3 consecutive
// frames). Both paths run inside Tracker::processFrame, which already holds
// the FeatureManager-protecting mutex_.
//
// CPU budget: cv::ORB::create at 250 features on 640×480 lands near
// 5–8 ms/keyframe on a Snapdragon 695 (per visual-production plan §4
// timing measurements). At 1 keyframe per ~15 frames, that's well under
// the 1% CPU envelope target.
void FeatureManager::storeKeyframeDescriptors(
        uint64_t kf_id,
        double ts_ns,
        const cv::Mat& gray,
        const std::vector<cv::Point2f>& corner_pts,
        const std::vector<int>& corner_feature_ids) {

    if (gray.empty()) return;
    if (gray.type() != CV_8UC1) return;

    // Lazy ORB extractor init. Constructed once and reused across keyframes:
    // cv::ORB::create() allocates a FastFeatureDetector + a descriptor
    // pyramid, both of which are non-trivial to build per call. Defaults
    // mirror cv::ORB::create's signature, with the FAST threshold and
    // feature cap pulled from the consumer-side constants per the project
    // rule "constants live next to their consumer".
    if (orb_extractor_.empty()) {
        orb_extractor_ = cv::ORB::create(
            ORB_TARGET_FEATURES,    // nfeatures
            1.2f,                   // scaleFactor (OpenCV default)
            8,                      // nlevels (OpenCV default)
            31,                     // edgeThreshold (OpenCV default)
            0,                      // firstLevel (OpenCV default)
            2,                      // WTA_K (OpenCV default)
            cv::ORB::HARRIS_SCORE,
            31,                     // patchSize (OpenCV default)
            ORB_FAST_THRESHOLD);
    }

    // Gaussian pre-blur σ ≈ 1.0 → ksize=0 lets OpenCV pick the radius from
    // sigma (≈ 6σ + 1 = 7). Reduces FAST corner noise sensitivity without
    // smearing fine detail.
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(0, 0), ORB_PREBLUR_SIGMA);

    KeyframeDescriptors rec;
    rec.keyframe_id  = kf_id;
    rec.timestamp_ns = ts_ns;
    orb_extractor_->detectAndCompute(blurred, cv::noArray(),
                                     rec.keypoints, rec.descriptors);

    // Inherit FeatureManager ids from the supplied tracked KLT corners by
    // spatial proximity (≤ ORB_KLT_MATCH_RADIUS). corner_pts /
    // corner_feature_ids are parallel; if they ever desync we treat all
    // corners as having no id (-1) to keep the rest of the record valid.
    rec.feature_ids.assign(rec.keypoints.size(), -1);
    if (corner_pts.size() == corner_feature_ids.size() && !corner_pts.empty()) {
        const float r2 = ORB_KLT_MATCH_RADIUS * ORB_KLT_MATCH_RADIUS;
        for (size_t k = 0; k < rec.keypoints.size(); ++k) {
            const cv::Point2f& kp = rec.keypoints[k].pt;
            float best_d2 = r2;
            int   best_fid = -1;
            for (size_t c = 0; c < corner_pts.size(); ++c) {
                const float dx = kp.x - corner_pts[c].x;
                const float dy = kp.y - corner_pts[c].y;
                const float d2 = dx * dx + dy * dy;
                if (d2 < best_d2) {
                    best_d2  = d2;
                    best_fid = corner_feature_ids[c];
                }
            }
            rec.feature_ids[k] = best_fid;
        }
    }

    keyframe_descriptors_.push_back(std::move(rec));
    while (keyframe_descriptors_.size() > KEYFRAME_DESC_RING_SIZE) {
        keyframe_descriptors_.pop_front();
    }
}

int FeatureManager::pruneStaleLifecycle(const std::vector<int>& active_ids,
                                        int64_t now_ns) {
    if (lifecycle_.empty()) return 0;

    // Hash-set the active IDs once for O(1) membership tests inside the
    // erase loop. active_ids is typically ≤ 200 — the KLT cap.
    std::unordered_set<int> active_set;
    active_set.reserve(active_ids.size() * 2);
    for (int id : active_ids) {
        if (id >= 0) active_set.insert(id);
    }

    int dropped = 0;
    auto it = lifecycle_.begin();
    while (it != lifecycle_.end()) {
        const int fid = it->first;
        const FeatureLifecycle& lc = it->second;
        // Keep if (a) still tracked this frame, (b) currently a SLAM
        // feature in the EKF state, or (c) was observed within the last
        // LIFECYCLE_KEEP_NS — a one-frame KLT miss during motion blur
        // is common and the track frequently recovers next frame; the
        // grace window prevents silent resets of the promotion gate.
        const bool still_active  = active_set.count(fid) > 0;
        const bool in_slam       = lc.slam_slot >= 0;
        const bool recently_seen = lc.last_obs_ns > 0 &&
                                   (now_ns - lc.last_obs_ns) < LIFECYCLE_KEEP_NS;
        if (!still_active && !in_slam && !recently_seen) {
            // Drop the lifecycle record AND any orphaned observation
            // history in active_tracks_ for the same feature_id. The
            // observation history is otherwise pruned only when a clone
            // gets marginalised below it — for a KLT track that died
            // before the window slid, both grow without bound.
            active_tracks_.erase(fid);
            it = lifecycle_.erase(it);
            dropped++;
        } else {
            ++it;
        }
    }
    return dropped;
}
