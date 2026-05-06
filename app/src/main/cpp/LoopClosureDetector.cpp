// LoopClosureDetector — DBoW2-backed implementation. See header for
// pipeline overview, threading model, and constant citations.

#include "LoopClosureDetector.h"

#include "EventCounters.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <DBoW2/DBoW2.h>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "LoopClosureDetector"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,  TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGD(...) (void)0
#define LOGE(...) (void)0
#define LOGW(...) (void)0
#endif

namespace {

// ─── Magic-number citations ───────────────────────────────────────────────
//
// BoW score gating — adaptive minScore (ORB-SLAM2 LoopClosing.cc::DetectLoop)
//   The previous fixed threshold of 0.05 was a misreading of Galvez-Lopez
//   & Tardos (T-RO 2012 §V): that paper specifies a threshold on the
//   *normalized* score η = score(query, candidate) / score(query, prev),
//   not on the raw L1 score we get back from OrbDatabase::query. With our
//   250 ORB features per keyframe (vs ORB-SLAM2's ~1000), genuine same-
//   place revisits produce raw scores in the 0.003-0.012 band — well below
//   any sensible fixed cutoff. See real-walk evidence in
//   tests/sims/simulation_data_1777985054704.json (114s twice-around-house
//   walk: 150 attempts, 0 accepts at threshold 0.05).
//
//   ORB-SLAM2 fixes this by computing the minimum BoW score the current
//   keyframe achieves against its covisibility neighbors and using that
//   as the per-scene threshold. Quote from raulmur/ORB_SLAM2 LoopClosing.cc:
//       float minScore = 1;
//       for (KeyFrame* pKF : vpConnectedKeyFrames) {
//         float score = mpORBVocabulary->score(CurrentBowVec, pKF->mBowVec);
//         if (score < minScore) minScore = score;
//       }
//       vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(currentKF, minScore);
//
//   We don't have a covisibility graph yet, so we use the K most recent
//   keyframes that fall inside the temporal-exclusion window as a temporal
//   proxy — those are exactly the keyframes the current frame is "still
//   tracking against" in continuous KLT.
//
// kBowScoreFloor
//   Lower bound on the adaptive threshold. Prevents runaway acceptance in
//   low-texture stretches where even the temporal neighbors score near
//   zero against the query — without a floor, an indoor-walls passage
//   could push minScore to 0.0001 and any DB hit would pass. 0.005 sits
//   roughly an order of magnitude below the observed same-place range.
constexpr double kBowScoreFloor   = 0.005;
//
// kCovisibilityK
//   Number of recent keyframes to score against for the adaptive minScore.
//   At our ~2 Hz keyframe rate, K=10 covers ~5s of recent past — a tight
//   covisibility proxy that stays comfortably inside the 30 s temporal
//   exclusion window so the neighbors are themselves excluded from being
//   loop candidates.
constexpr size_t kCovisibilityK   = 10;

// PNP_MIN_INLIERS
//   Originally Step 7 plan §line 723 specified 30. That number assumes a
//   feature-based SLAM where every ORB keypoint owns a 3D MapPoint (so a
//   keyframe with 1000 ORBs naturally yields hundreds of 2D-3D pairs). Our
//   hybrid (KLT-tracked + ORB-at-keyframes + ≤12 SLAM features in EKF) only
//   yields ~5-15 valid pairs even with cross-keyframe ORB triangulation
//   (Tracker.cpp ~L2245), so 30 was structurally unreachable. ORB-SLAM2's
//   loop verification in `LoopClosing.cc::ComputeSim3` accepts at 12-20
//   inliers; we land at 15 as a comfortable middle. Reproj threshold (4 px,
//   below) is the real false-positive guard.
constexpr int kPnpMinInliers = 15;

// LOWE_RATIO
//   Lowe 2004 §7.1. Same value as the Step 4 reloc path.
constexpr float kLoweRatio = 0.75f;

// PNP_REPROJ_THRESH_PX
//   cv::solvePnPRansac default is 4.0 px. Looser than Step 4's 1.5 px
//   because the long temporal gap means the query and match keyframes
//   were captured under noticeably different lighting / viewpoint.
constexpr double kPnpReprojThreshPx = 4.0;
constexpr double kPnpConfidence    = 0.99;
constexpr int    kPnpMaxIters      = 100;

// Top-N candidates pulled from BoW. We only attempt geometric verification
// on the best one (idx 0) — querying a few extras is essentially free
// (inverted index walks the same words once) and helps debug.
constexpr int kBowTopN = 4;

// Convert a CV_8U Nx32 descriptor matrix to DBoW2's preferred
// vector<cv::Mat> (one single-row mat per descriptor).
std::vector<cv::Mat> descriptorMatToVec(const cv::Mat& descriptors) {
    std::vector<cv::Mat> out;
    if (descriptors.empty()) return out;
    if (descriptors.type() != CV_8U || descriptors.cols != 32) {
        // DBoW2 ORB form is strictly CV_8U Nx32. Anything else is a bug
        // upstream; refuse to add rather than crash inside DBoW2.
        return out;
    }
    out.reserve(descriptors.rows);
    for (int r = 0; r < descriptors.rows; ++r) {
        // Clone so each row owns its memory — the OrbDatabase keeps a
        // copy internally anyway, but cloning here protects us if a
        // future DBoW2 version moves to refcounting.
        out.push_back(descriptors.row(r).clone());
    }
    return out;
}

// Single keyframe record kept alongside the BoW database. The OrbDatabase
// only stores BoW vectors / FeatureVectors; we need the original
// descriptors + 3D points + pose to run geometric verification on a hit.
struct KeyframeRecord {
    uint64_t                    kf_id          = 0;
    int64_t                     timestamp_ns   = 0;
    DBoW2::EntryId              db_entry_id    = 0;
    cv::Mat                     descriptors;          // CV_8U Nx32
    std::vector<cv::KeyPoint>   keypoints;
    std::vector<cv::Point3f>    pts3d_world;
    cv::Matx33d                 R_world_cam    = cv::Matx33d::eye();
    cv::Vec3d                   t_cam_world    = cv::Vec3d(0, 0, 0);
    // Camera heading (Tracker::scalar_heading_, radians, north=0 clockwise).
    // Used by the heading gate in tryDetectLoop.
    double                      yaw_rad        = 0.0;
    // BoW vector stored alongside the entry so tryDetectLoop can rescore
    // the query against recent neighbors (adaptive minScore — see comment
    // block above kBowScoreFloor). DBoW2's OrbDatabase::add takes the BoW
    // vector internally but does not expose it back, so we compute it on
    // our side via vocab.transform() before the add() call.
    DBoW2::BowVector            bow_vec;
};

}  // namespace

// ─── LoopClosureDetector::Impl ───────────────────────────────────────────
struct LoopClosureDetector::Impl {
    // Single mutex guards both `db` and `keyframes`. Same pattern as the
    // FeatureManager / EKFState snapshot mutexes used by Step 6.
    mutable std::mutex          mutex;

    OrbVocabulary               vocab;
    // unique_ptr because OrbDatabase is initialised after vocab loads,
    // not at construction time. Default-construction would leave a
    // database with no vocabulary, which DBoW2 cannot use.
    std::unique_ptr<OrbDatabase> db;

    std::vector<KeyframeRecord> keyframes;

    // Map db_entry_id → index into `keyframes` for O(1) lookup after a
    // BoW query returns. EntryIds are dense small integers, but keeping
    // a map decouples us from any future DBoW2 changes that reuse ids.
    std::unordered_map<DBoW2::EntryId, size_t> entry_to_index;

    std::atomic<bool>           ready{false};

    cv::Ptr<cv::BFMatcher>      matcher;

    Impl() {
        // crossCheck=false: knnMatch with k=2 wants both nearest neighbours
        // for Lowe's ratio test, which is incompatible with crossCheck.
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING, /*crossCheck=*/false);
    }
};

// ─── Construction / destruction ──────────────────────────────────────────
LoopClosureDetector::LoopClosureDetector()
    : impl_(std::make_unique<Impl>()) {}

LoopClosureDetector::~LoopClosureDetector() = default;

// ─── Vocabulary loading ──────────────────────────────────────────────────
bool LoopClosureDetector::loadVocabulary(const std::string& vocab_path) {
    if (!impl_) return false;

    std::lock_guard<std::mutex> lk(impl_->mutex);

    // Wipe whatever state was there. loadVocabulary is normally called
    // once at startup, but a re-call must be safe.
    impl_->keyframes.clear();
    impl_->entry_to_index.clear();
    impl_->db.reset();
    impl_->ready.store(false, std::memory_order_release);

    bool ok = false;
    try {
        // ORB-SLAM2 publicly distributes ORBvoc.txt (the .tar.gz on the
        // Android side). loadFromTextFile is our additive method on
        // TemplatedVocabulary that parses that format directly — way
        // smaller and faster than the equivalent YAML.
        ok = impl_->vocab.loadFromTextFile(vocab_path);
    } catch (const std::exception& e) {
        LOGE("Vocabulary load threw: %s", e.what());
        return false;
    } catch (...) {
        LOGE("Vocabulary load threw an unknown exception");
        return false;
    }

    if (!ok) {
        LOGE("Failed to load ORB vocabulary from '%s'", vocab_path.c_str());
        return false;
    }

    if (impl_->vocab.empty()) {
        LOGE("Vocabulary loaded but is empty: '%s'", vocab_path.c_str());
        return false;
    }

    // use_di=false: we do not need the direct index. The direct index
    // lets you look up "which db entries contain word W at level L" — we
    // run our own brute-force descriptor match for geometric verification
    // anyway, so paying the memory + add() cost for it is wasteful.
    // di_levels=0 is the canonical "off" value.
    impl_->db = std::make_unique<OrbDatabase>(impl_->vocab,
                                              /*use_di=*/false,
                                              /*di_levels=*/0);

    impl_->ready.store(true, std::memory_order_release);
    LOGI("ORB vocabulary loaded: %zu words from '%s'",
         static_cast<size_t>(impl_->vocab.size()), vocab_path.c_str());
    return true;
}

// ─── addKeyframe ─────────────────────────────────────────────────────────
void LoopClosureDetector::addKeyframe(
    uint64_t kf_id, double timestamp_ns,
    const cv::Mat& descriptors,
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<cv::Point3f>& pts3d_world,
    const cv::Matx33d& R_world_cam,
    const cv::Vec3d&   t_cam_world,
    double yaw_rad)
{
    if (!impl_ || !impl_->ready.load(std::memory_order_acquire)) {
        return;
    }
    if (descriptors.empty() || descriptors.type() != CV_8U ||
        descriptors.cols != 32) {
        // Silently skip — Tracker should never give us a bad shape, but
        // the only sane recovery is to drop the keyframe.
        return;
    }
    if (static_cast<size_t>(descriptors.rows) != keypoints.size()) {
        LOGW("addKeyframe: descriptors.rows=%d but keypoints.size()=%zu — skipping",
             descriptors.rows, keypoints.size());
        return;
    }
    // QA pass (2026-05-05): the detector's PnP path indexes pts3d_world
    // by trainIdx (which is bounded by keypoints.size()). If a future
    // caller passes a mis-sized pts3d_world (older code did exactly this),
    // every match silently fails the bounds check at the lookup site. Catch
    // the contract violation at the writer rather than letting it manifest
    // as "0 PnP pairs" downstream.
    if (pts3d_world.size() != keypoints.size()) {
        LOGW("addKeyframe: pts3d_world.size()=%zu != keypoints.size()=%zu — "
             "refusing (header contract violation)",
             pts3d_world.size(), keypoints.size());
        return;
    }

    std::vector<cv::Mat> features = descriptorMatToVec(descriptors);
    if (features.empty()) return;

    KeyframeRecord rec;
    rec.kf_id        = kf_id;
    rec.timestamp_ns = static_cast<int64_t>(timestamp_ns);
    rec.descriptors  = descriptors.clone();
    rec.keypoints    = keypoints;
    rec.pts3d_world  = pts3d_world;
    rec.R_world_cam  = R_world_cam;
    rec.t_cam_world  = t_cam_world;
    rec.yaw_rad      = yaw_rad;

    {
        std::lock_guard<std::mutex> lk(impl_->mutex);
        if (!impl_->db) return;  // ready toggled off behind our back
        // Compute BoW vector before add() so we can store it alongside the
        // record. db.add() also computes one internally but does not return
        // it; rather than dig into DBoW2's internals (or pay for a lookup
        // by entry id), recomputing once here is the cleanest option and
        // costs ~50µs per keyframe at 250 features.
        impl_->vocab.transform(features, rec.bow_vec);
        rec.db_entry_id = impl_->db->add(features);

        const size_t idx = impl_->keyframes.size();
        impl_->keyframes.push_back(std::move(rec));
        impl_->entry_to_index[impl_->keyframes.back().db_entry_id] = idx;
    }
    // NOTE: loop_closure_kf_count_in_db is updated at the Tracker call site
    // immediately after this returns (Tracker.cpp ~L2261), to keep both the
    // detector and the Tracker accounting paths self-contained. We do NOT
    // duplicate the store here — that would double-update with no value.
}

// ─── tryDetectLoop ───────────────────────────────────────────────────────
bool LoopClosureDetector::tryDetectLoop(
    uint64_t now_kf_id, int64_t now_ns,
    const cv::Mat& descriptors,
    const std::vector<cv::KeyPoint>& keypoints,
    double fx, double fy, double cx, double cy,
    int64_t temporal_exclusion_ns,
    double current_yaw_rad,
    LoopMatch& out_match)
{
    if (!impl_ || !impl_->ready.load(std::memory_order_acquire)) {
        return false;
    }
    if (descriptors.empty() || descriptors.type() != CV_8U ||
        descriptors.cols != 32) {
        return false;
    }
    if (static_cast<size_t>(descriptors.rows) != keypoints.size()) {
        return false;
    }

    // NOTE: loop_closure_attempts is bumped by Tracker.cpp at the worker
    // tick site (Tracker.cpp ~L3086), not here, so each tick counts as one
    // attempt regardless of whether tryDetectLoop short-circuits early.
    // Same rationale for loop_closure_accepts / loop_closure_rejects_low_score:
    // the Tracker side has the canonical attempt-tick boundary. The
    // detector only owns the *PnP* rejection counter (Tracker explicitly
    // delegates that to us — Tracker.cpp ~L3103).

    // ── (1) BoW query under the lock ─────────────────────────────────────
    DBoW2::QueryResults results;

    // Snapshot of the candidate's stored data, taken under the lock and
    // then used outside the lock for the (slow) PnP step.
    KeyframeRecord candidate;
    bool candidate_valid = false;
    double         best_score = 0.0;
    DBoW2::EntryId best_entry_id = 0;

    {
        std::lock_guard<std::mutex> lk(impl_->mutex);
        if (!impl_->db || impl_->keyframes.empty()) return false;

        std::vector<cv::Mat> features = descriptorMatToVec(descriptors);
        if (features.empty()) return false;

        // Compute excluded_max_id: the highest db entry id whose
        // timestamp is strictly older than `now_ns - temporal_exclusion_ns`.
        // DBoW2's `max_id` parameter means "only consider entries with
        // id <= max_id" — so we want max_id == excluded_max_id, i.e. the
        // last sufficiently-old entry. < 0 means "all entries".
        const int64_t cutoff_ns = now_ns - temporal_exclusion_ns;
        int max_id = -1;
        // Keyframes are appended monotonically, so timestamps are
        // monotonically non-decreasing → linear scan from the end.
        for (auto it = impl_->keyframes.rbegin();
             it != impl_->keyframes.rend(); ++it) {
            if (it->timestamp_ns <= cutoff_ns) {
                max_id = static_cast<int>(it->db_entry_id);
                break;
            }
        }
        if (max_id < 0) {
            // Every keyframe is too recent — nothing to match against yet.
            return false;
        }

        // Compute query BoW vector once — used both for the db query (via
        // the same `features`) and for the adaptive minScore against
        // recent neighbors below.
        DBoW2::BowVector query_bow;
        impl_->vocab.transform(features, query_bow);

        impl_->db->query(features, results, kBowTopN, max_id);

        if (results.empty()) return false;
        // results[0] is highest-score by DBoW2 contract.
        const DBoW2::Result& top = results[0];
        best_score    = top.Score;
        best_entry_id = top.Id;

        // ── Adaptive minScore (ORB-SLAM2 LoopClosing.cc::DetectLoop) ──
        // Score the query against the K most recent keyframes that are
        // themselves *inside* the temporal-exclusion window — those are
        // our covisibility proxy. Take the minimum: a candidate from the
        // DB has to look at least as similar to us as our noisiest
        // recent neighbor does. Floor at kBowScoreFloor to defend against
        // low-texture stretches where every neighbor scores near zero.
        double min_score    = 1.0;
        size_t n_neighbors  = 0;
        const size_t total_kfs = impl_->keyframes.size();
        const size_t start_idx = (total_kfs > kCovisibilityK)
                                 ? total_kfs - kCovisibilityK
                                 : 0;
        for (size_t i = start_idx; i < total_kfs; ++i) {
            const KeyframeRecord& neighbor = impl_->keyframes[i];
            // Only neighbors NEWER than cutoff_ns count — the rest are
            // candidate territory, not covisibility territory.
            if (neighbor.timestamp_ns <= cutoff_ns) continue;
            const double s = impl_->vocab.score(query_bow, neighbor.bow_vec);
            if (s < min_score) min_score = s;
            ++n_neighbors;
        }
        if (n_neighbors == 0) {
            // No recent neighbors yet (early in the walk). Fall back to
            // the floor so the detector can still accept genuinely strong
            // matches but not noise.
            min_score = kBowScoreFloor;
        } else {
            min_score = std::max(min_score, kBowScoreFloor);
        }

        if (best_score < min_score) {
            // Detector owns BOTH rejection counters now (cpp-reviewer 2026-05-04
            // HIGH-1: when Tracker also bumped rejects_low_score on every false
            // return, PnP rejections double-counted because the detector
            // already bumped rejects_pnp before returning false). Worker's
            // accept/attempt counters stay; rejection-reason counters are
            // owned exclusively here.
            navsight::eventCounters().loop_closure_rejects_low_score.fetch_add(
                1, std::memory_order_relaxed);
            LOGD("BoW reject: best_score=%.4f < adaptive_min=%.4f "
                 "(neighbors=%zu, kf %llu)",
                 best_score, min_score, n_neighbors,
                 static_cast<unsigned long long>(now_kf_id));
            return false;
        }

        auto it = impl_->entry_to_index.find(best_entry_id);
        if (it == impl_->entry_to_index.end()) {
            // Should never happen — entry_to_index is updated alongside
            // every db.add(). Defensive guard only.
            return false;
        }
        candidate       = impl_->keyframes[it->second];
        candidate_valid = true;
    }
    // Lock is released. PnP runs lock-free below.

    if (!candidate_valid) return false;

    // Heading gate: reject opposite-direction candidates before the
    // expensive BFMatcher + PnP step. ORB descriptors are ~30°-invariant
    // but not 180°-invariant; reversed-viewpoint 3D-2D pairs are
    // geometrically inconsistent for RANSAC.
    // Source: sim_data_1778078217065 — 587/638 PnP failures from a
    // 600 m walk where the user returned via the opposite direction.
    // π/2 allows ±90° viewpoint variation (forward/sideways) while
    // blocking the 180° reversal case observed in that sim.
    {
        constexpr double kMaxHeadingDiffRad = M_PI / 2.0;
        double hdiff = std::abs(current_yaw_rad - candidate.yaw_rad);
        if (hdiff > M_PI) hdiff = 2.0 * M_PI - hdiff;
        if (hdiff > kMaxHeadingDiffRad) {
            navsight::eventCounters().loop_closure_rejects_heading.fetch_add(
                1, std::memory_order_relaxed);
            LOGD("LC heading reject: now_yaw=%.2f candidate_yaw=%.2f diff=%.2f rad",
                 current_yaw_rad, candidate.yaw_rad, hdiff);
            return false;
        }
    }

    // ── (2) BFMatcher Hamming + Lowe ratio ───────────────────────────────
    std::vector<std::vector<cv::DMatch>> knn;
    impl_->matcher->knnMatch(descriptors, candidate.descriptors, knn, 2);

    std::vector<cv::Point2f> pts2d;
    std::vector<cv::Point3f> pts3d;
    pts2d.reserve(knn.size());
    pts3d.reserve(knn.size());

    for (const auto& pair : knn) {
        if (pair.size() < 2) continue;
        if (pair[0].distance >= kLoweRatio * pair[1].distance) continue;

        const int q_idx = pair[0].queryIdx;
        const int t_idx = pair[0].trainIdx;
        if (q_idx < 0 || q_idx >= static_cast<int>(keypoints.size())) continue;
        if (t_idx < 0 ||
            t_idx >= static_cast<int>(candidate.pts3d_world.size())) continue;

        const cv::Point3f& p3 = candidate.pts3d_world[t_idx];
        // Skip rows whose 3D point isn't valid — Tracker uses NaN to mark
        // keypoints that never got triangulated.
        if (!std::isfinite(p3.x) || !std::isfinite(p3.y) ||
            !std::isfinite(p3.z)) continue;

        pts2d.push_back(keypoints[q_idx].pt);
        pts3d.push_back(p3);
    }

    if (static_cast<int>(pts2d.size()) < kPnpMinInliers) {
        navsight::eventCounters().loop_closure_rejects_pnp.fetch_add(
            1, std::memory_order_relaxed);
        LOGD("PnP reject: only %zu 2D-3D pairs (need %d) for kf %llu vs %llu",
             pts2d.size(), kPnpMinInliers,
             static_cast<unsigned long long>(now_kf_id),
             static_cast<unsigned long long>(candidate.kf_id));
        return false;
    }

    // ── (3) solvePnPRansac ───────────────────────────────────────────────
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 fx,  0, cx,
                  0, fy, cy,
                  0,  0,  1);
    cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);  // already undistorted upstream
    cv::Mat rvec, tvec;
    std::vector<int> inliers;

    bool pnp_ok = false;
    try {
        pnp_ok = cv::solvePnPRansac(
            pts3d, pts2d, K, dist, rvec, tvec,
            /*useExtrinsicGuess=*/false,
            kPnpMaxIters,
            static_cast<float>(kPnpReprojThreshPx),
            kPnpConfidence,
            inliers,
            cv::SOLVEPNP_ITERATIVE);
    } catch (const cv::Exception& e) {
        LOGW("solvePnPRansac threw: %s", e.what());
        navsight::eventCounters().loop_closure_rejects_pnp.fetch_add(
            1, std::memory_order_relaxed);
        return false;
    }

    if (!pnp_ok || static_cast<int>(inliers.size()) < kPnpMinInliers) {
        navsight::eventCounters().loop_closure_rejects_pnp.fetch_add(
            1, std::memory_order_relaxed);
        LOGD("PnP reject: ok=%d inliers=%zu (need %d) for kf %llu vs %llu",
             pnp_ok ? 1 : 0, inliers.size(), kPnpMinInliers,
             static_cast<unsigned long long>(now_kf_id),
             static_cast<unsigned long long>(candidate.kf_id));
        return false;
    }

    // ── (4) Build the LoopMatch payload ──────────────────────────────────
    // PnP returns rvec/tvec mapping a 3D point in the *match camera's
    // world frame* (which == our world frame: same session) into the
    // current camera. So we have:
    //     X_now = R_now_world * X_world + t_now_world
    // Where:
    //     R_now_world = Rodrigues(rvec)
    //     t_now_world = tvec
    // For the LoopMatch caller we want the cam-now → cam-match relative.
    // Let R_match_world / t_match_world be the candidate's stored pose
    // (we have R_world_cam_match — its transpose). Then:
    //     R_now_to_match = R_match_world * R_now_world^T
    //     t_now_to_match = t_match_world - R_now_to_match * t_now_world
    cv::Mat R_now_world_cv;
    cv::Rodrigues(rvec, R_now_world_cv);
    cv::Matx33d R_now_world(R_now_world_cv);

    // R_match_world = R_world_cam_match^T (R_world_cam takes cam→world,
    // its transpose takes world→cam_match).
    const cv::Matx33d R_match_world = candidate.R_world_cam.t();
    // The candidate's t_cam_world is the camera *position in world* — to
    // get the world→cam_match translation we need:
    //     t_match_world = -R_match_world * t_cam_world_match
    const cv::Vec3d t_match_world = -(R_match_world * candidate.t_cam_world);

    const cv::Matx33d R_now_to_match = R_match_world * R_now_world.t();
    const cv::Vec3d   t_now_world(tvec.at<double>(0),
                                  tvec.at<double>(1),
                                  tvec.at<double>(2));
    const cv::Vec3d   t_now_to_match =
        t_match_world - R_now_to_match * t_now_world;

    out_match.matched_kf_id    = candidate.kf_id;
    out_match.bow_score        = best_score;
    out_match.pnp_inliers      = static_cast<int>(inliers.size());
    out_match.R_now_to_match   = R_now_to_match;
    out_match.t_now_to_match   = t_now_to_match;
    out_match.R_world_cam_match = candidate.R_world_cam;
    out_match.t_cam_world_match = candidate.t_cam_world;

    // Tracker bumps loop_closure_accepts itself on the true return — see
    // note above.

    LOGI("LOOP CLOSURE accepted: now_kf=%llu match_kf=%llu score=%.4f inliers=%d",
         static_cast<unsigned long long>(now_kf_id),
         static_cast<unsigned long long>(candidate.kf_id),
         best_score, static_cast<int>(inliers.size()));
    return true;
}

// ─── Trivial accessors ───────────────────────────────────────────────────
size_t LoopClosureDetector::getKeyframeCount() const {
    if (!impl_) return 0;
    std::lock_guard<std::mutex> lk(impl_->mutex);
    return impl_->keyframes.size();
}

bool LoopClosureDetector::isReady() const {
    if (!impl_) return false;
    return impl_->ready.load(std::memory_order_acquire);
}
