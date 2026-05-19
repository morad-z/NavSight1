// LandmarkMap — visual-only persistent 3D landmark database (Step 6).
// See LandmarkMap.h for full design rationale + ORB-SLAM3 citations.

#include "LandmarkMap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-LandmarkMap"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGD(...) (void)0
#define LOGW(...) (void)0
#endif

namespace navsight {

namespace {

// Hamming distance between two 1×32 CV_8U ORB descriptors. cv::norm with
// NORM_HAMMING is the OpenCV canonical path (used inside BFMatcher).
int hammingDistance(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty()) return std::numeric_limits<int>::max();
    if (a.cols != 32 || b.cols != 32) return std::numeric_limits<int>::max();
    if (a.type() != CV_8U || b.type() != CV_8U) return std::numeric_limits<int>::max();
    return static_cast<int>(cv::norm(a, b, cv::NORM_HAMMING));
}

inline bool isFiniteVec(const cv::Vec3d& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

}  // namespace

LandmarkMap::LandmarkMap() {
    LOGI("LandmarkMap: constructed (dedup_3d_m=%.2f hamming_max=%d "
         "search_radius_m=%.1f depth_band_m=[%.1f,%.1f])",
         kDedup3DDistanceM, kDescriptorMaxHamming,
         kDefaultSearchRadiusM, kDepthMinM, kDepthMaxM);
}

LandmarkMap::~LandmarkMap() = default;

// ─── Producer API ────────────────────────────────────────────────────────────

// Legacy 4-arg overload. Delegates to the impl with no anchor and no
// observation-pixel seed — preserves the test/legacy behaviour.
int LandmarkMap::addOrMergeLandmark(const cv::Vec3d& p_world,
                                     const cv::Mat&   descriptor,
                                     uint64_t         kf_id,
                                     int64_t          ts_ns) {
    static const cv::Matx33d kIdentity33d(1.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0,
                                          0.0, 0.0, 1.0);
    static const cv::Vec3d   kZero3d(0.0, 0.0, 0.0);
    static const cv::Point2f kNanPixel(
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN());
    return addOrMergeLandmarkImpl(p_world, descriptor, kf_id,
                                   kIdentity33d, kZero3d, kNanPixel, ts_ns,
                                   /*set_anchor=*/false,
                                   /*record_initial_obs=*/false);
}

// 7-arg overload (2026-05-19 Fix #4): anchor data + initial KLT pixel.
int LandmarkMap::addOrMergeLandmark(const cv::Vec3d&   p_world,
                                     const cv::Mat&     descriptor,
                                     uint64_t           kf_id,
                                     const cv::Matx33d& host_R_world_cam,
                                     const cv::Vec3d&   host_t_cam_world,
                                     const cv::Point2f& host_kp_pixel,
                                     int64_t            ts_ns) {
    const bool record = std::isfinite(host_kp_pixel.x) &&
                         std::isfinite(host_kp_pixel.y);
    return addOrMergeLandmarkImpl(p_world, descriptor, kf_id,
                                   host_R_world_cam, host_t_cam_world,
                                   host_kp_pixel, ts_ns,
                                   /*set_anchor=*/true,
                                   /*record_initial_obs=*/record);
}

int LandmarkMap::addOrMergeLandmarkImpl(const cv::Vec3d&   p_world,
                                         const cv::Mat&     descriptor,
                                         uint64_t           kf_id,
                                         const cv::Matx33d& host_R_world_cam,
                                         const cv::Vec3d&   host_t_cam_world,
                                         const cv::Point2f& host_kp_pixel,
                                         int64_t            ts_ns,
                                         bool               set_anchor,
                                         bool               record_initial_obs) {
    // Trust-boundary validation. Invalid inputs return -1; the caller is
    // the camera thread and an out-of-band failure here is a real bug
    // worth catching loud, not a silent skip.
    if (!isFiniteVec(p_world)) {
        LOGW("LandmarkMap.addOrMergeLandmark: REJECT non-finite p_world "
             "(%.3f,%.3f,%.3f) kf=%llu",
             p_world[0], p_world[1], p_world[2],
             static_cast<unsigned long long>(kf_id));
        return -1;
    }
    if (descriptor.empty() || descriptor.cols != 32 ||
        descriptor.type() != CV_8U) {
        LOGW("LandmarkMap.addOrMergeLandmark: REJECT bad descriptor "
             "(empty=%d cols=%d type=%d) kf=%llu",
             descriptor.empty() ? 1 : 0, descriptor.cols, descriptor.type(),
             static_cast<unsigned long long>(kf_id));
        return -1;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Dedup search: find an existing landmark within 3D and descriptor gates.
    int merge_id = findMergeCandidateLocked(p_world, descriptor);
    if (merge_id >= 0) {
        Landmark& l = landmarks_[merge_id];
        l.observed_in_kfs.push_back(kf_id);
        l.last_seen_ts_ns = ts_ns;
        ++l.times_observed;
        ++landmarks_merged_total_;
        // p_world NOT updated here — the existing position is the canonical
        // one. Refinement happens at LC pose-graph back-write.
        // host_clone_id / p_anchor_cam ALSO not updated here — the existing
        // anchor is canonical; this new observation just bumps obs counts.
        //
        // 2026-05-19 Fix #4 — push the host keyframe's keypoint pixel onto
        // the ring buffer (capped at kObsHistoryCap). This gives the
        // windowed BA a fresh observation of this landmark from the host
        // clone for cross-frame refinement, even on merge.
        if (record_initial_obs) {
            l.observation_pixels.emplace_back(kf_id, host_kp_pixel);
            if (l.observation_pixels.size() > kObsHistoryCap) {
                l.observation_pixels.erase(l.observation_pixels.begin());
            }
        }
        LOGD("LandmarkMap.merge: id=%d kf=%llu obs=%d",
             merge_id, static_cast<unsigned long long>(kf_id), l.times_observed);
        return merge_id;
    }

    // Fresh landmark.
    Landmark l;
    l.id                = next_id_++;
    l.p_world           = p_world;
    l.descriptor        = descriptor.clone();          // own a deep copy
    l.observed_in_kfs.push_back(kf_id);
    l.first_seen_ts_ns  = ts_ns;
    l.last_seen_ts_ns   = ts_ns;
    l.times_observed    = 1;

    if (set_anchor && isFiniteVec(host_t_cam_world)) {
        // p_anchor_cam = (world→camera) · (p_world − camera_position_in_world)
        //              = host_R_world_cam.t() · (p_world − host_t_cam_world)
        // Per LandmarkMap::projectIntoCamera at the same convention: R_world_cam
        // is camera→world, so its transpose maps world→camera.
        const cv::Matx33d R_w2c = host_R_world_cam.t();
        const cv::Vec3d   p_anchor = R_w2c * (p_world - host_t_cam_world);
        if (isFiniteVec(p_anchor)) {
            l.host_clone_id = static_cast<int>(kf_id);
            l.p_anchor_cam  = p_anchor;
            l.has_anchor    = true;
        }
    }

    // 2026-05-19 Fix #4 — Seed observation_pixels with the host keyframe's
    // keypoint pixel so the windowed BA has at least one observation to
    // constrain `p_world` against this clone's pose. Cap at kObsHistoryCap
    // (no-op for a fresh landmark but keeps the invariant uniform with the
    // merge path above).
    if (record_initial_obs) {
        l.observation_pixels.emplace_back(kf_id, host_kp_pixel);
        if (l.observation_pixels.size() > kObsHistoryCap) {
            l.observation_pixels.erase(l.observation_pixels.begin());
        }
    }

    const int id = l.id;
    landmarks_.emplace(id, std::move(l));
    ++landmarks_added_total_;
    ++kd_adds_since_rebuild_;
    kd_dirty_ = true;

    if (kd_adds_since_rebuild_ >= kKdRebuildThreshold) {
        rebuildKdTreeLocked();
    }

    LOGD("LandmarkMap.add: id=%d p=(%.2f,%.2f,%.2f) kf=%llu size=%zu anchor=%d",
         id, p_world[0], p_world[1], p_world[2],
         static_cast<unsigned long long>(kf_id), landmarks_.size(),
         set_anchor ? 1 : 0);
    return id;
}

int LandmarkMap::findMergeCandidateLocked(const cv::Vec3d& p_world,
                                            const cv::Mat& descriptor) const {
    // Fast path: when the map is small or the KD index is empty, the
    // linear scan over O(N) candidates is faster than building a query
    // tree. Once size grows past kKdRebuildThreshold, use the spatial
    // index to narrow the candidate set first.
    const double dedup2 = kDedup3DDistanceM * kDedup3DDistanceM;

    if (kd_dirty_ || !kd_index_ || landmarks_.size() < 32) {
        // Linear scan
        int    best_id        = -1;
        double best_dist2     = dedup2;
        int    best_hamming   = kDescriptorMaxHamming + 1;
        for (const auto& [id, l] : landmarks_) {
            const cv::Vec3d d = l.p_world - p_world;
            const double d2 = d.dot(d);
            if (d2 > dedup2) continue;
            const int h = hammingDistance(l.descriptor, descriptor);
            if (h > kDescriptorMaxHamming) continue;
            // Prefer closer Hamming; break ties by 3D distance.
            if (h < best_hamming ||
                (h == best_hamming && d2 < best_dist2)) {
                best_id      = id;
                best_dist2   = d2;
                best_hamming = h;
            }
        }
        return best_id;
    }

    // KD-tree narrowed candidate set.
    cv::Mat query = (cv::Mat_<float>(1, 3) <<
                      static_cast<float>(p_world[0]),
                      static_cast<float>(p_world[1]),
                      static_cast<float>(p_world[2]));
    std::vector<int>   indices;
    std::vector<float> dists;
    cv::flann::SearchParams params(/*checks=*/32);
    // radiusSearch returns up to maxResults; cap is unlikely to be hit
    // for the dedup case (we'd expect 0-2 candidates inside 0.5 m).
    const float radius = static_cast<float>(kDedup3DDistanceM);
    kd_index_->radiusSearch(query, indices, dists, radius * radius,
                             /*maxResults=*/16, params);

    int    best_id      = -1;
    int    best_hamming = kDescriptorMaxHamming + 1;
    double best_dist2   = dedup2;
    for (size_t i = 0; i < indices.size(); ++i) {
        if (indices[i] < 0 ||
            static_cast<size_t>(indices[i]) >= kd_id_map_.size()) continue;
        const int lm_id = kd_id_map_[indices[i]];
        auto it = landmarks_.find(lm_id);
        if (it == landmarks_.end()) continue;
        const int h = hammingDistance(it->second.descriptor, descriptor);
        if (h > kDescriptorMaxHamming) continue;
        const double d2 = static_cast<double>(dists[i]);
        if (h < best_hamming ||
            (h == best_hamming && d2 < best_dist2)) {
            best_id      = lm_id;
            best_hamming = h;
            best_dist2   = d2;
        }
    }
    return best_id;
}

void LandmarkMap::rebuildKdTreeLocked() const {
    const int n = static_cast<int>(landmarks_.size());
    if (n == 0) {
        kd_data_ = cv::Mat();
        kd_id_map_.clear();
        kd_index_.reset();
        kd_dirty_ = false;
        kd_adds_since_rebuild_ = 0;
        return;
    }

    kd_data_.create(n, 3, CV_32F);
    kd_id_map_.resize(n);
    int row = 0;
    for (const auto& [id, l] : landmarks_) {
        kd_data_.at<float>(row, 0) = static_cast<float>(l.p_world[0]);
        kd_data_.at<float>(row, 1) = static_cast<float>(l.p_world[1]);
        kd_data_.at<float>(row, 2) = static_cast<float>(l.p_world[2]);
        kd_id_map_[row] = id;
        ++row;
    }

    kd_index_ = std::make_unique<cv::flann::Index>(
        kd_data_, cv::flann::KDTreeIndexParams(/*trees=*/4));
    kd_dirty_ = false;
    kd_adds_since_rebuild_ = 0;
    ++kd_rebuilds_total_;
    LOGI("LandmarkMap.kd_rebuild: n=%d total_rebuilds=%d",
         n, kd_rebuilds_total_);
}

// ─── Consumer API ────────────────────────────────────────────────────────────

std::vector<int> LandmarkMap::getLandmarksInRadius(const cv::Vec3d& p_center,
                                                     double           radius_m,
                                                     int64_t          now_ns,
                                                     int64_t          max_age_ns) const {
    std::vector<int> result;
    if (!isFiniteVec(p_center) || radius_m <= 0.0) return result;

    std::lock_guard<std::mutex> lock(mutex_);
    if (landmarks_.empty()) return result;

    // Rebuild index lazily if dirty (the consumer also pays for any
    // pending rebuild; this keeps the producer's hot path lighter).
    if (kd_dirty_) rebuildKdTreeLocked();

    // Linear-scan fallback (no index built yet, e.g., < 32 landmarks).
    auto inWindow = [&](const Landmark& l) {
        return max_age_ns < 0 || now_ns < 0 ||
               (now_ns - l.last_seen_ts_ns) <= max_age_ns;
    };

    struct Hit { int id; double d2; };
    std::vector<Hit> hits;

    if (!kd_index_) {
        const double r2 = radius_m * radius_m;
        for (const auto& [id, l] : landmarks_) {
            const cv::Vec3d d = l.p_world - p_center;
            const double d2 = d.dot(d);
            if (d2 > r2) continue;
            if (!inWindow(l)) continue;
            hits.push_back({id, d2});
        }
    } else {
        cv::Mat query = (cv::Mat_<float>(1, 3) <<
                          static_cast<float>(p_center[0]),
                          static_cast<float>(p_center[1]),
                          static_cast<float>(p_center[2]));
        std::vector<int>   idx;
        std::vector<float> dists;
        cv::flann::SearchParams params(/*checks=*/64);
        const float r2f = static_cast<float>(radius_m * radius_m);
        // Cap max-results at a reasonable upper bound. 256 is well over
        // the typical ~10-30 landmarks in a 30m sphere at indoor density.
        kd_index_->radiusSearch(query, idx, dists, r2f, 256, params);
        for (size_t i = 0; i < idx.size(); ++i) {
            if (idx[i] < 0 ||
                static_cast<size_t>(idx[i]) >= kd_id_map_.size()) continue;
            const int lm_id = kd_id_map_[idx[i]];
            auto it = landmarks_.find(lm_id);
            if (it == landmarks_.end()) continue;
            if (!inWindow(it->second)) continue;
            hits.push_back({lm_id, static_cast<double>(dists[i])});
        }
    }

    std::sort(hits.begin(), hits.end(),
              [](const Hit& a, const Hit& b) { return a.d2 < b.d2; });
    result.reserve(hits.size());
    for (const auto& h : hits) result.push_back(h.id);
    return result;
}

// Phase 6.4c — Loop-closure spatial pre-filter. Union of observed_in_kfs
// across all landmarks inside `radius_m` of `p_center`. Read-only.
//
// We deliberately do NOT call getLandmarksInRadius() and then call
// getLandmark() per id — that would double-lock the mutex per landmark.
// Instead, walk the local index inline under a single lock guard.
std::vector<uint64_t> LandmarkMap::getKeyframesNearPosition(
        const cv::Vec3d& p_center,
        double           radius_m,
        int64_t          now_ns,
        int64_t          max_age_ns) const {
    std::vector<uint64_t> result;
    if (!isFiniteVec(p_center) || radius_m <= 0.0) return result;

    std::lock_guard<std::mutex> lock(mutex_);
    if (landmarks_.empty()) return result;

    if (kd_dirty_) rebuildKdTreeLocked();

    auto inWindow = [&](const Landmark& l) {
        return max_age_ns < 0 || now_ns < 0 ||
               (now_ns - l.last_seen_ts_ns) <= max_age_ns;
    };

    // Dedup using set semantics — landmarks observed by the same keyframe
    // would otherwise add duplicates. Reserved capacity sized to the
    // typical 10-30 landmarks-in-30m density times ~3 observations each.
    std::vector<uint64_t> kf_ids;
    kf_ids.reserve(64);

    auto collectFrom = [&](const Landmark& l) {
        for (uint64_t kf : l.observed_in_kfs) kf_ids.push_back(kf);
    };

    if (!kd_index_) {
        const double r2 = radius_m * radius_m;
        for (const auto& [id, l] : landmarks_) {
            const cv::Vec3d d = l.p_world - p_center;
            if (d.dot(d) > r2) continue;
            if (!inWindow(l))   continue;
            collectFrom(l);
        }
    } else {
        cv::Mat query = (cv::Mat_<float>(1, 3) <<
                          static_cast<float>(p_center[0]),
                          static_cast<float>(p_center[1]),
                          static_cast<float>(p_center[2]));
        std::vector<int>   idx;
        std::vector<float> dists;
        cv::flann::SearchParams params(/*checks=*/64);
        const float r2f = static_cast<float>(radius_m * radius_m);
        kd_index_->radiusSearch(query, idx, dists, r2f, 256, params);
        for (size_t i = 0; i < idx.size(); ++i) {
            if (idx[i] < 0 ||
                static_cast<size_t>(idx[i]) >= kd_id_map_.size()) continue;
            const int lm_id = kd_id_map_[idx[i]];
            auto it = landmarks_.find(lm_id);
            if (it == landmarks_.end()) continue;
            if (!inWindow(it->second))  continue;
            collectFrom(it->second);
        }
    }

    if (kf_ids.empty()) return result;
    std::sort(kf_ids.begin(), kf_ids.end());
    kf_ids.erase(std::unique(kf_ids.begin(), kf_ids.end()), kf_ids.end());

    // Cap. Oldest first because sorted ascending — older keyframes are
    // also more likely to be genuine revisit candidates (they survived
    // longer in the LandmarkMap, hence more confirmed observations).
    if (kf_ids.size() > kLcMaxKeyframes) kf_ids.resize(kLcMaxKeyframes);
    result.swap(kf_ids);
    return result;
}

// 2026-05-17 — re-observation API (investigation agent finding #3).
// When local-map tracking matches a stored landmark against a current KLT
// keypoint, this bumps times_observed and updates last_seen_ts_ns so the
// landmark survives the cull policy. Without this, only the initial
// addOrMergeLandmark increments the counter, and landmarks die after the
// grace period even when actively re-observed every walk pass.
bool LandmarkMap::touchLandmark(int landmark_id, uint64_t kf_id, int64_t ts_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = landmarks_.find(landmark_id);
    if (it == landmarks_.end()) return false;
    Landmark& lm = it->second;
    // Append kf_id to observed_in_kfs if not already present. Linear scan
    // is fine — observed_in_kfs is bounded by the number of times the user
    // re-passes this landmark, typically < 10 over a walk.
    bool kf_seen = false;
    for (uint64_t existing_kf : lm.observed_in_kfs) {
        if (existing_kf == kf_id) { kf_seen = true; break; }
    }
    if (!kf_seen) {
        lm.observed_in_kfs.push_back(kf_id);
    }
    ++lm.times_observed;
    lm.last_seen_ts_ns = ts_ns;
    return true;
}

// 2026-05-19 Fix #4 — touchLandmark with current-frame pixel. Adds an entry
// to the observation_pixels ring buffer so the BA can refine `p_world`
// across the matches accumulated over the BA window. Cap enforced at
// kObsHistoryCap (oldest dropped). All other state mutated identically to
// the 3-arg overload.
bool LandmarkMap::touchLandmark(int landmark_id, uint64_t kf_id, int64_t ts_ns,
                                  const cv::Point2f& matched_pixel) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = landmarks_.find(landmark_id);
    if (it == landmarks_.end()) return false;
    Landmark& lm = it->second;
    bool kf_seen = false;
    for (uint64_t existing_kf : lm.observed_in_kfs) {
        if (existing_kf == kf_id) { kf_seen = true; break; }
    }
    if (!kf_seen) {
        lm.observed_in_kfs.push_back(kf_id);
    }
    ++lm.times_observed;
    lm.last_seen_ts_ns = ts_ns;
    // Push the matched pixel onto the ring buffer. Don't dedup-by-kf_id —
    // multiple matches against the same kf_id should overwrite the older
    // entry's pixel with the latest (better-localised) one. Cheap linear
    // scan because the buffer is bounded by kObsHistoryCap (8).
    if (std::isfinite(matched_pixel.x) && std::isfinite(matched_pixel.y)) {
        bool replaced = false;
        for (auto& p : lm.observation_pixels) {
            if (p.first == kf_id) {
                p.second = matched_pixel;
                replaced = true;
                break;
            }
        }
        if (!replaced) {
            lm.observation_pixels.emplace_back(kf_id, matched_pixel);
            if (lm.observation_pixels.size() > kObsHistoryCap) {
                lm.observation_pixels.erase(lm.observation_pixels.begin());
            }
        }
    }
    return true;
}

// 2026-05-19 Fix #4 — gather BA-refinable landmarks. Returns landmarks
// whose observation_pixels intersect `clone_ids_window` in ≥ `min_obs`
// entries. Each result carries the FILTERED observation list (only entries
// whose clone_id is in the window). Caller is the BA worker thread via
// Tracker::kickOffBARound; the lock guarantees a coherent snapshot.
std::vector<LandmarkMap::LandmarkObsForBA>
LandmarkMap::getLandmarksWithObsInClones(
        const std::vector<int>& clone_ids_window,
        int                      min_obs) const {
    std::vector<LandmarkObsForBA> out;
    if (clone_ids_window.empty() || min_obs < 2) return out;
    // Build a small set for O(1) membership tests.
    std::unordered_set<int> window_set(clone_ids_window.begin(),
                                        clone_ids_window.end());
    std::lock_guard<std::mutex> lock(mutex_);
    out.reserve(64);
    for (const auto& [id, lm] : landmarks_) {
        std::vector<std::pair<int, cv::Point2f>> obs_in_window;
        obs_in_window.reserve(lm.observation_pixels.size());
        for (const auto& [kf_id, px] : lm.observation_pixels) {
            const int kf_id_int = static_cast<int>(kf_id);
            if (window_set.count(kf_id_int)) {
                obs_in_window.emplace_back(kf_id_int, px);
            }
        }
        if (static_cast<int>(obs_in_window.size()) < min_obs) continue;
        LandmarkObsForBA entry;
        entry.id      = id;
        entry.p_world = lm.p_world;
        entry.obs     = std::move(obs_in_window);
        out.push_back(std::move(entry));
    }
    return out;
}

bool LandmarkMap::setLandmarkPosition(int landmark_id,
                                       const cv::Vec3d& p_world) {
    if (!isFiniteVec(p_world)) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = landmarks_.find(landmark_id);
    if (it == landmarks_.end()) return false;
    it->second.p_world = p_world;
    // Spatial index now stale relative to the refined position. Mark dirty
    // so the next getLandmarksInRadius rebuilds. Cheap — same flag the
    // add/merge path uses.
    kd_dirty_ = true;
    return true;
}

bool LandmarkMap::getLandmark(int id, Landmark& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = landmarks_.find(id);
    if (it == landmarks_.end()) return false;
    out = it->second;
    out.descriptor = it->second.descriptor.clone();
    return true;
}

bool LandmarkMap::projectIntoCamera(int                 landmark_id,
                                      const cv::Matx33d&  R_world_cam,
                                      const cv::Vec3d&    t_cam_world,
                                      double              fx, double fy,
                                      double              cx, double cy,
                                      int                 img_width,
                                      int                 img_height,
                                      cv::Point2f&        out_pixel) const {
    if (fx <= 0.0 || fy <= 0.0 || img_width <= 0 || img_height <= 0) return false;

    Landmark l;
    if (!getLandmark(landmark_id, l)) return false;

    // World → camera transform:
    //   R_world_cam : cam→world  →  R_cam_world = R_world_cam.t()
    //   t_cam_world : camera position in world
    //   p_cam = R_cam_world · (p_world − t_cam_world)
    const cv::Matx33d R_cam_world = R_world_cam.t();
    const cv::Vec3d   p_cam       = R_cam_world * (l.p_world - t_cam_world);

    const double z = p_cam[2];
    if (!std::isfinite(z))                return false;
    if (z < kDepthMinM || z > kDepthMaxM) return false;

    const double u = fx * (p_cam[0] / z) + cx;
    const double v = fy * (p_cam[1] / z) + cy;
    if (!std::isfinite(u) || !std::isfinite(v))            return false;
    if (u < 0.0 || u >= img_width)                         return false;
    if (v < 0.0 || v >= img_height)                        return false;

    out_pixel = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    return true;
}

// ─── Mutator API ─────────────────────────────────────────────────────────────

int LandmarkMap::applyKeyframePoseCorrection(uint64_t kf_id,
                                              double dx, double dy, double dz,
                                              double dyaw) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Build R_z(Δyaw) for the rotation component. Δyaw applied around
    // world-Z; landmarks anchored to kf_id rotate AND translate with
    // their parent keyframe pose.
    const double c = std::cos(dyaw);
    const double s = std::sin(dyaw);
    const cv::Matx33d Rz(c, -s, 0.0,
                          s,  c, 0.0,
                          0.0, 0.0, 1.0);
    const cv::Vec3d dp(dx, dy, dz);

    int n_shifted = 0;
    for (auto& [id, l] : landmarks_) {
        // A landmark is "anchored to kf_id" if kf_id appears in its
        // observed_in_kfs list. Shift only those landmarks.
        bool anchored = false;
        for (uint64_t kf : l.observed_in_kfs) {
            if (kf == kf_id) { anchored = true; break; }
        }
        if (!anchored) continue;

        // Rotate around world-Z, then translate.
        l.p_world = Rz * l.p_world + dp;
        ++n_shifted;
    }
    if (n_shifted > 0) {
        kd_dirty_ = true;   // spatial index needs rebuild after position shift
        LOGI("LandmarkMap.apply_correction: kf=%llu n_shifted=%d "
             "dxyz=(%.3f,%.3f,%.3f) dyaw_deg=%.2f",
             static_cast<unsigned long long>(kf_id), n_shifted,
             dx, dy, dz, dyaw * 180.0 / M_PI);
    }
    return n_shifted;
}

// ─── Maintenance ─────────────────────────────────────────────────────────────

int LandmarkMap::cullStaleLandmarks(int64_t now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    int n_culled = 0;
    for (auto it = landmarks_.begin(); it != landmarks_.end(); ) {
        const Landmark& l = it->second;
        const bool past_grace =
            (now_ns - l.first_seen_ts_ns) > kGracePeriodNs;
        const bool under_obs =
            l.times_observed < kMinObservationsAfterGrace;
        if (past_grace && under_obs) {
            LOGD("LandmarkMap.cull: id=%d obs=%d age_s=%.1f",
                 l.id, l.times_observed,
                 (now_ns - l.first_seen_ts_ns) * 1e-9);
            it = landmarks_.erase(it);
            ++n_culled;
        } else {
            ++it;
        }
    }
    if (n_culled > 0) {
        landmarks_culled_total_ += n_culled;
        kd_dirty_ = true;
        LOGI("LandmarkMap.cull_total: n_culled=%d remaining=%zu",
             n_culled, landmarks_.size());
    }
    return n_culled;
}

void LandmarkMap::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    landmarks_.clear();
    next_id_                  = 0;
    kd_data_                  = cv::Mat();
    kd_id_map_.clear();
    kd_index_.reset();
    kd_dirty_                 = true;
    kd_adds_since_rebuild_    = 0;
    kd_rebuilds_total_        = 0;
    landmarks_added_total_    = 0;
    landmarks_merged_total_   = 0;
    landmarks_culled_total_   = 0;
    LOGI("LandmarkMap.reset");
}

// ─── Diagnostics ─────────────────────────────────────────────────────────────

size_t LandmarkMap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return landmarks_.size();
}
int LandmarkMap::landmarksAddedTotal()   const { std::lock_guard<std::mutex> l(mutex_); return landmarks_added_total_;   }
int LandmarkMap::landmarksMergedTotal()  const { std::lock_guard<std::mutex> l(mutex_); return landmarks_merged_total_;  }
int LandmarkMap::landmarksCulledTotal()  const { std::lock_guard<std::mutex> l(mutex_); return landmarks_culled_total_;  }
int LandmarkMap::kdRebuildsTotal()       const { std::lock_guard<std::mutex> l(mutex_); return kd_rebuilds_total_;       }

}  // namespace navsight
