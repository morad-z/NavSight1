#include "FeatureManager.h"
#include "EventCounters.h"
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
    // Plan Step 6 (ADR-012): hold snapshot_mutex_ across the active_tracks_
    // and lifecycle_ clear so the BA worker thread can never observe a
    // half-cleared map.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    keyframes_.clear();
    keyframe_descriptors_.clear();
    active_tracks_.clear();
    lifecycle_.clear();
    next_feature_id_ = 0;
    // Plan Step 5 (ADR-011): clear the low-light hysteresis state so a
    // mid-session reset does not carry the prior bootstrap's gate decision
    // forward into the new one.
    low_light_low_streak_    = 0;
    low_light_normal_streak_ = 0;
    low_light_state_         = false;
    low_light_log_counter_   = 0;
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

    // ── Plan Step 5 (ADR-011): adaptive low-light feature replenish ────────
    //
    // Probe centre-crop mean brightness so the goodFeaturesToTrack budget
    // can be relaxed in regimes where strong corners are physically scarce
    // (parking garages, indoor halls under dim LED, night corridors). The
    // crop matches Tracker's blur detector ROI so motion-blur skips and
    // low-light replenish see the same patch. A small two-state hysteresis
    // (BRIGHTNESS_HYSTERESIS_FRAMES) prevents feature-count thrashing when
    // auto-exposure ramps cause the mean to oscillate around the gate.
    //
    // Steady-state Tracker::MAX_FEATURES (200) and Tracker::QUALITY_LEVEL
    // (0.05) are intentionally NOT mutated. Only the local effective
    // targets used inside this call swap to the low-light pair.
    int eff_max_total      = max_total;
    double eff_quality     = quality_level;

    {
        // Centre-crop using the same integer-division pattern Tracker's
        // measureBlur uses (Tracker.cpp ~line 342: cols/2, rows/2). On
        // odd-dimension frames the BRIGHTNESS_CENTRE_FRAC=0.5 multiply
        // could drift by one pixel vs Tracker; integer-divide unifies
        // the two crops bit-for-bit.
        const int cw = gray.cols / 2;
        const int ch = gray.rows / 2;
        const int cx = (gray.cols - cw) / 2;
        const int cy = (gray.rows - ch) / 2;
        const cv::Rect centre(cx, cy, cw, ch);
        const double mean_brightness = cv::mean(gray(centre))[0] / 255.0;

        if (mean_brightness < BRIGHTNESS_LOW_THRESH) {
            low_light_low_streak_    = std::min(low_light_low_streak_ + 1, 1000);
            low_light_normal_streak_ = 0;
        } else {
            low_light_normal_streak_ = std::min(low_light_normal_streak_ + 1, 1000);
            low_light_low_streak_    = 0;
        }
        // Hysteresis flip: only commit the new state after N consecutive
        // observations. Prevents AE-ramp thrashing on the boundary.
        if (!low_light_state_ &&
            low_light_low_streak_ >= BRIGHTNESS_HYSTERESIS_FRAMES) {
            low_light_state_ = true;
        } else if (low_light_state_ &&
                   low_light_normal_streak_ >= BRIGHTNESS_HYSTERESIS_FRAMES) {
            low_light_state_ = false;
        }

        if (low_light_state_) {
            eff_max_total = REPLENISH_TARGET_LOWLIGHT;
            eff_quality   = REPLENISH_QUALITY_LOWLIGHT;
            // Every replenish call while the committed low-light state is
            // active counts as one active frame. This is the per-frame
            // metric, not the per-log-line one.
            navsight::eventCounters().lowlight_state_active_frames.fetch_add(
                1, std::memory_order_relaxed);
            if ((low_light_log_counter_++ % 30) == 0) {
                LOGI("LOWLIGHT: brightness=%.3f target=%d quality=%.2f",
                     mean_brightness, eff_max_total, eff_quality);
                navsight::eventCounters().lowlight_log_lines.fetch_add(
                    1, std::memory_order_relaxed);
            }
        } else {
            // Reset the logging cadence when we leave low-light so the
            // next entry logs immediately rather than at a stale offset.
            low_light_log_counter_ = 0;
        }
    }

    int cell_w = gray.cols / GRID_COLS;
    int cell_h = gray.rows / GRID_ROWS;
    int target_per_cell = eff_max_total / (GRID_ROWS * GRID_COLS);
    int remaining = eff_max_total - static_cast<int>(existing.size());
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
                                    eff_quality, min_dist);

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
    // Plan Step 6 (ADR-012): camera-thread writer of `active_tracks_`. Lock
    // briefly so getLandmarkSnapshot from the BA worker thread cannot see a
    // mid-push_back std::vector. Lock is held only across the O(1) hashmap
    // emplace + vector append.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    active_tracks_[feature_id].push_back({clone_state_id, pixel_ud});
}

std::vector<LostFeature> FeatureManager::extractLostFeatures(
        const std::vector<int>& current_ids, int min_obs) {
    // Plan Step 6 (ADR-012): mutates active_tracks_ (erases lost entries).
    // Lock so the BA snapshot reader cannot iterate a partially-erased map.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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
    // Plan Step 6 (ADR-012): writes through active_tracks_ — lock against
    // BA snapshot reader.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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
    // Plan Step 6 (ADR-012): writes lifecycle_ on the camera thread.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto& lc = lifecycle_[feature_id];
    if (lc.feature_id == -1) {
        lc.feature_id = feature_id;
    }
    lc.age++;
    if (is_keyframe) lc.kf_count++;
    lc.last_obs_ns = ts_ns;
}

void FeatureManager::noteKeyframe(int feature_id) {
    // Plan Step 6 (ADR-012): writes lifecycle_ on the camera thread.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return;
    it->second.kf_count++;
}

void FeatureManager::noteTriangulation(int feature_id,
                                       const cv::Point3f& p_global,
                                       int anchor_clone_id) {
    // Plan Step 6 (ADR-012): writes lifecycle_ on the camera thread.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto& lc = lifecycle_[feature_id];
    if (lc.feature_id == -1) {
        lc.feature_id = feature_id;
    }
    lc.last_p_global   = p_global;
    lc.has_p_global    = true;
    lc.anchor_clone_id = anchor_clone_id;
}

void FeatureManager::setSlamSlot(int feature_id, int slam_slot) {
    // Plan Step 6 (ADR-012): writes lifecycle_ on the camera thread.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return;
    auto& lc = it->second;
    const int prev_slot = lc.slam_slot;
    lc.slam_slot = slam_slot;
    if (slam_slot < 0) {
        lc.rms_bad_consecutive = 0;
    }
    // Step 9 (ADR-014) — SLAM lifetime histogram.
    if (prev_slot < 0 && slam_slot >= 0) {
        // Promotion: snapshot age so the demotion path can compute Δage.
        lc.promoted_age = lc.age;
    } else if (prev_slot >= 0 && slam_slot < 0 && lc.promoted_age >= 0) {
        // Demotion: contribute lifetime in observations, then clear so a
        // re-promotion starts fresh accounting.
        const long long lifetime_obs =
            static_cast<long long>(lc.age) - static_cast<long long>(lc.promoted_age);
        if (lifetime_obs > 0) {
            navsight::eventCounters().slam_lifetime_obs_sum.fetch_add(
                lifetime_obs, std::memory_order_relaxed);
            navsight::eventCounters().slam_lifetime_count.fetch_add(
                1, std::memory_order_relaxed);
        }
        lc.promoted_age = -1;
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
        // NOTE 2026-05-04: removed `has_p_global` requirement here.
        // It was a chicken-and-egg block: has_p_global is only set by
        // noteTriangulation (FeatureManager.cpp:385), which is only
        // called from inside the SLAM-promotion path (Tracker.cpp:1763)
        // AFTER addSlamFeature succeeds. So no fresh feature could ever
        // satisfy the gate without first being promoted, and no feature
        // could be promoted without first satisfying the gate. Result:
        // slam_promotions_total stayed at 0 across two 100 m walks.
        // The promotion path in Tracker.cpp:1684-1721 does its own
        // two-view triangulation + chirality + RMS-gate every time, so
        // the has_p_global field here was redundant defence. Letting
        // un-triangulated candidates through means the promotion path
        // gets a chance to triangulate them itself.
        if (lc.last_rms_px > max_init_rms_px && lc.last_rms_px > 0.0) continue;
        out.push_back(fid);
    }
    return out;
}

void FeatureManager::markSlamFeatureRMS(int feature_id, double rms_px) {
    // Plan Step 6 (ADR-012): writes lifecycle_ on the camera thread.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

// CAMERA THREAD ONLY. Returns a pointer into lifecycle_, which would be
// invalidated by any concurrent writer. Locking snapshot_mutex_ inside
// is insufficient because the pointer outlives the lock; the caller
// would still race with writers after the function returns. Step 6
// (ADR-012) BA worker thread MUST NOT call this — it reads lifecycle
// state via the LandmarkSnapshot path which copies-by-value under the
// mutex. A future caller that needs access from a non-camera thread
// should use a std::optional<FeatureLifecycle> by-value variant.
const FeatureManager::FeatureLifecycle*
FeatureManager::getLifecycle(int feature_id) const {
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return nullptr;
    return &it->second;
}

// CAMERA THREAD ONLY — same caveat as getLifecycle above.
const std::vector<FeatureObservation>*
FeatureManager::getObservations(int feature_id) const {
    auto it = active_tracks_.find(feature_id);
    if (it == active_tracks_.end()) return nullptr;
    return &it->second;
}

void FeatureManager::dropLifecycle(int feature_id) {
    // Plan Step 6 (ADR-012): writes lifecycle_ on the camera thread.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto it = lifecycle_.find(feature_id);
    if (it == lifecycle_.end()) return;
    // Step 9 (ADR-014) — if the feature is dropped while still SLAM-promoted
    // (e.g. EKF removeSlamFeature path that bypasses an explicit setSlamSlot
    // demotion), still account its lifetime contribution so the histogram
    // is monotonic. The promoted_age guard is conservative: zero or negative
    // deltas (which shouldn't happen but could on a re-entrant edge case)
    // are skipped.
    const auto& lc = it->second;
    if (lc.slam_slot >= 0 && lc.promoted_age >= 0) {
        const long long lifetime_obs =
            static_cast<long long>(lc.age) - static_cast<long long>(lc.promoted_age);
        if (lifetime_obs > 0) {
            navsight::eventCounters().slam_lifetime_obs_sum.fetch_add(
                lifetime_obs, std::memory_order_relaxed);
            navsight::eventCounters().slam_lifetime_count.fetch_add(
                1, std::memory_order_relaxed);
        }
    }
    lifecycle_.erase(it);
}

// Returns by VALUE (vector copy under the mutex), so it is safe to call
// from any thread — the snapshot is decoupled from lifecycle_ before the
// lock drops.
std::vector<int> FeatureManager::getAllLifecycleFeatureIds() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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
    // Plan Step 6 (ADR-012): mutates both lifecycle_ and active_tracks_.
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
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

// ── Plan Step 6 (ADR-012): SLAM-landmark snapshot for windowed BA ────────────
//
// Walks the lifecycle map for promoted SLAM landmarks (`slam_slot >= 0`)
// that own a triangulated world point (`has_p_global`), then for each such
// landmark walks its active observation history and emits the entries whose
// `clone_state_id` is in the caller's whitelist. The whitelist is the set
// of clone IDs returned by EKFState::getCloneSnapshot, so the produced
// observation pairs always reference a pose the BA solver will be optimising.
//
// Pixel conversion: active_tracks_ stores normalised (un-distorted) camera-
// frame uv (see Tracker.cpp section 9.1, where addObservation receives
// (px - cx) / fx). The BA worker wants pixel-space residuals so the Huber
// threshold (1.5 px) is meaningful; we multiply back through with the live
// intrinsics.
//
// Filter: a landmark with fewer than `min_obs` observations on the pose
// whitelist is dropped — it cannot constrain the joint pose+landmark solve
// (need at least 2 views to triangulate, 3+ to refine). min_obs=2 matches
// the Step 6 plan's "≥ 2 of 5 KFs" criterion.
//
// Lock: held only for the duration of the read-and-copy. Camera-thread
// writers (addObservation, noteTriangulation, setSlamSlot, etc.) take the
// same mutex briefly when mutating; the snapshot copy is bounded
// O(N_promoted × N_obs_per) which is < 1 ms on phone-class hardware.
std::vector<FeatureManager::LandmarkSnapshot>
FeatureManager::getLandmarkSnapshot(const std::vector<int>& clone_id_whitelist,
                                     double fx, double fy,
                                     double cx, double cy,
                                     int min_obs) const {
    std::vector<LandmarkSnapshot> out;
    if (clone_id_whitelist.empty() || fx <= 1.0 || fy <= 1.0) return out;

    std::lock_guard<std::mutex> lock(snapshot_mutex_);

    if (lifecycle_.empty() || active_tracks_.empty()) return out;

    // Hash-set the whitelist for O(1) membership tests.
    std::unordered_set<int> kf_set;
    kf_set.reserve(clone_id_whitelist.size() * 2);
    for (int id : clone_id_whitelist) {
        if (id >= 0) kf_set.insert(id);
    }
    if (kf_set.empty()) return out;

    out.reserve(8);  // EKFState::MAX_SLAM_FEATURES is 12; reserve upper-ish

    for (const auto& [fid, lc] : lifecycle_) {
        if (lc.slam_slot < 0)    continue;   // not promoted
        if (!lc.has_p_global)    continue;   // no seed point yet

        auto track_it = active_tracks_.find(fid);
        if (track_it == active_tracks_.end()) continue;
        const auto& track = track_it->second;

        LandmarkSnapshot snap;
        snap.feature_id = fid;
        snap.slam_slot  = lc.slam_slot;
        snap.p_world    = cv::Vec3d(static_cast<double>(lc.last_p_global.x),
                                    static_cast<double>(lc.last_p_global.y),
                                    static_cast<double>(lc.last_p_global.z));
        snap.obs.reserve(track.size());

        for (const auto& obs : track) {
            if (kf_set.count(obs.clone_state_id) == 0) continue;
            // active_tracks_ stores normalised camera-frame uv. BA needs
            // pixels for the 1.5-px Huber gate to be meaningful.
            const double u_px = static_cast<double>(obs.pixel_ud.x) * fx + cx;
            const double v_px = static_cast<double>(obs.pixel_ud.y) * fy + cy;
            snap.obs.emplace_back(obs.clone_state_id, cv::Vec2d(u_px, v_px));
        }

        if (static_cast<int>(snap.obs.size()) < min_obs) continue;
        out.push_back(std::move(snap));
    }
    return out;
}
