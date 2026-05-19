#pragma once

// LandmarkMap — visual-only persistent 3D landmark database.
//
// Phase 1 Step 6 of post_v19_sprint_plan.md §205-298. ORB-SLAM3-inspired
// "Tracking the local map" architecture. Adapted to NavSight per the
// research in docs/AI_HANDOFF_2026_05_16.md and the canonical
// ORB-SLAM3 paper (Campos et al., arXiv:2007.11898).
//
// ARCHITECTURAL DECISIONS (cited)
//
// 1. Visual-only at the state level. Landmarks are world-anchored 3D
//    points stored OUTSIDE the EKF state vector. The IMU is NOT coupled
//    to landmark positions. This matches ORB-SLAM3's design: the paper
//    states explicitly that "the estimated state only includes the
//    current camera pose, in visual-inertial SLAM, additional variables
//    [pose, velocity, biases] need to be computed" — distinct from the
//    map point database.
//
// 2. Landmarks are FIXED during EKF tracking updates. The pose-only
//    optimization pattern (ORB-SLAM3 Tracking thread): "solves a
//    simplified visual-inertial optimization where only the states of
//    the last two frames are optimized, while map points remain fixed."
//    The Jacobian against landmark position is omitted entirely.
//    Landmark p_world is refined only when a loop closure fires (via
//    applyKeyframePoseCorrection).
//
// 3. Deduplication by 3D-distance AND descriptor similarity. Per plan
//    §281: 3D dedup distance 0.5 m (smaller than typical indoor feature
//    spacing) AND ORB-SLAM3 standard Hamming ≤ 50 / 256.
//
// 4. Spatial index = OpenCV cv::flann::Index (KD-tree). Rebuilt lazily
//    after kKdRebuildThreshold adds. Read queries are lock-protected.
//
// 5. Lifecycle: landmarks created via addOrMergeLandmark at keyframe
//    storage; refined only at LC pose-graph back-write; culled when
//    times_observed < kMinObservationsAfterGrace after a grace period.
//
// LIFE CYCLE OF A LANDMARK
//
//      addOrMergeLandmark
//             │
//             ▼
//       active in DB ─────► getLandmarksInRadius (TrackLocalMap)
//             │                       │
//             ▼                       ▼
//      observed_in_kfs += kf_id   projectIntoCamera + match → EKF MSCKF update
//             │                       │
//             ▼                       │
//      cullStaleLandmarks            (landmark p_world UNCHANGED during EKF update)
//             │
//             ▼
//      removed if obs<gate after grace
//
//      LC pose-graph fires:
//      applyKeyframePoseCorrection(kf, Δp, Δyaw)
//      shifts all landmarks observed in that kf
//
// THREADING
//
//   All public methods take an internal std::mutex. The class is used
//   from both the camera thread (addOrMergeLandmark, EKF query) and the
//   1 Hz loop-closure worker (applyKeyframePoseCorrection at LC accept),
//   so a single guard is mandatory. Spatial-index rebuild also happens
//   under the lock.

#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>
#include <unordered_map>
#include <vector>

namespace navsight {

// Per-landmark record. Public so callers can copy for read-only use.
struct Landmark {
    int                       id          = -1;
    cv::Vec3d                 p_world     = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Mat                   descriptor;          // CV_8U, 1×32 ORB
    std::vector<uint64_t>     observed_in_kfs;
    int64_t                   first_seen_ts_ns = 0;
    int64_t                   last_seen_ts_ns  = 0;
    int                       times_observed   = 0;

    // 2026-05-19 Fix #4 — Ring buffer of recent (clone_id, image-pixel)
    // observations of this landmark. Used by the windowed local BA to
    // refine `p_world` across multiple keyframes (ORB-SLAM3 Local BA
    // pattern — MapPoint positions ARE optimization variables, not just
    // residual targets). Populated at:
    //   * addOrMergeLandmark — initial keypoint pixel in the host keyframe.
    //   * touchLandmark      — every local-map-match against a current KF.
    // Capped at kObsHistoryCap entries (oldest dropped) so memory stays
    // bounded for an 8K-landmark map. Parallel index space with
    // observed_in_kfs is NOT enforced — observed_in_kfs may contain entries
    // older than this ring buffer; the BA call only consumes the ring.
    // Mutex-protected by the LandmarkMap mutex_ on all reads and writes.
    std::vector<std::pair<uint64_t, cv::Point2f>> observation_pixels;

    // 2026-05-19 — Orange-dot anchor-relative render fields.
    //
    // Cause this exists: the render path used to project the fixed `p_world`
    // through the LIVE camera pose. Between landmark-add time and render
    // time, the EKF runs thousands of MSCKF / ZRUP / LC measurement updates
    // that shift the camera's world-frame belief; `p_world` stays frozen
    // and the projection drifts off the actual image feature. v54 walk:
    // 5823 MSCKF updates over 118 s + 30 LC corrections = orange dots
    // slide on every motion.
    //
    // Fix: store the landmark's coordinates in its HOST CLONE's camera
    // frame at add time. At render, if the host clone is still alive in
    // EKFState::window_, look up its CURRENT (post-update) pose and
    // recompute world position:
    //     p_world_render = R_host_GtoC_LIVE.t() * p_anchor_cam + p_host_G_LIVE
    // Same mechanism live SLAM features use (EKFState::snapshotForOverlay
    // around line 3450). Once the host clone marginalises out of the
    // window, fall back to the stored `p_world` (now world-fixed; older
    // landmark, less critical).
    //
    // `host_clone_id` is the EKFState clone id (uint64 truncated to int —
    // clone ids fit in int through the lifetime of a process). `has_anchor`
    // gates the lookup so legacy / merge / test paths that don't set the
    // anchor fall straight to the world-fixed path without spurious lookups.
    // On merge, the existing landmark's anchor wins — the new observation
    // only bumps `times_observed` + `observed_in_kfs` (anchor is invariant
    // to subsequent re-observation).
    int       host_clone_id   = -1;
    cv::Vec3d p_anchor_cam    = cv::Vec3d(0.0, 0.0, 0.0);
    bool      has_anchor      = false;
};

class LandmarkMap {
public:
    // ─── Tuning constants ─────────────────────────────────────────────────
    //
    // kDedup3DDistanceM
    //   Two triangulated 3D points within this distance + matching
    //   descriptor (Hamming ≤ kDescriptorMaxHamming) are merged into a
    //   single landmark. Plan §281: smaller than typical inter-feature
    //   spacing in indoor scenes — corner detectors enforce a min-distance
    //   of ~10 px which at typical 5 m feature depth ≈ 0.5 m world spacing.
    static constexpr double   kDedup3DDistanceM         = 0.5;

    // kDescriptorMaxHamming
    //   Maximum ORB binary Hamming distance for two descriptors to be
    //   considered the same feature. ORB-SLAM3 standard. Matches the
    //   threshold used by NavSight's Step 7.1 geometric loop closure
    //   verifier (LoopClosureDetector.cpp kGeomDescriptorMaxDistance) so
    //   the two systems agree on what "the same physical feature" means.
    static constexpr int      kDescriptorMaxHamming     = 50;

    // kDefaultSearchRadiusM
    //   Default radius for getLandmarksInRadius queries. Plan §279:
    //   typical KLT max depth (~10 m at NavSight's intrinsics) × 3.
    static constexpr double   kDefaultSearchRadiusM     = 30.0;

    // kDepthMinM / kDepthMaxM
    //   Camera-frame depth band for projectIntoCamera. Matches the
    //   identical band used in LoopClosureDetector.cpp's geometric path
    //   (kGeomDepthFloorM / kGeomDepthCeilingM). Below 0.5 m the camera
    //   model is unreliable (extreme parallax); above 50 m the projection
    //   is degenerate for the S21 Ultra's pinhole approximation.
    static constexpr double   kDepthMinM                = 0.5;
    static constexpr double   kDepthMaxM                = 50.0;

    // kKdRebuildThreshold
    //   Rebuild the spatial index after this many addOrMergeLandmark
    //   calls. Lazy rebuild avoids O(N) work on every add at the cost of
    //   queries operating on a slightly stale index between rebuilds.
    //   32 is a conservative starting point: a healthy walk adds ~10-15
    //   landmarks/keyframe → rebuild every 2-3 keyframes → ~1 s lag, well
    //   below the 10+ s typical loop-closure cadence.
    static constexpr int      kKdRebuildThreshold       = 32;

    // 2026-05-19 Fix #4 — Cap on per-landmark observation history. The
    // local BA only consumes observations whose clone_id is in the current
    // EKF window (≤ MAX_CLONES ≈ 15), so 8 is plenty: a landmark survives
    // ~0.3 s of keyframe storage at ~15-frame KF cadence × 30 Hz before
    // its oldest obs ages out of the BA-visible window. Memory: 8 ×
    // (8 + 8) bytes per landmark = 128 B + std::vector overhead.
    static constexpr size_t   kObsHistoryCap            = 8;

    // kMinObservationsAfterGrace / kGracePeriodNs
    //   2026-05-17 relaxed per investigation agent (a17a49b).
    //   Original (ORB-SLAM3-inspired): 3 obs / 10 s grace.
    //   That was too aggressive for NavSight's actual walk pattern:
    //   a user walking 1 m/s past a feature observes it across 2-3
    //   keyframes (~3 s), accumulates times_observed = 2, then leaves
    //   it behind. The 10 s grace expires before any return pass, so
    //   the cull kills the landmark before it can be re-recognized.
    //   Combined with the JNI filter (times_observed < 2) at
    //   native-lib.cpp:1345, the persistent landmark overlay never
    //   accumulates anything visible across a walk.
    //
    //   New policy: require 1 observation (any landmark that was ever
    //   triangulated stays), grace 120 s (enough for a 100 m walk +
    //   typical return). Combined with the new touchLandmark API
    //   (bumps times_observed when matched via local-map tracking),
    //   landmarks now naturally accumulate retention from re-visits.
    static constexpr int      kMinObservationsAfterGrace = 1;
    static constexpr int64_t  kGracePeriodNs             = 120'000'000'000LL;

    LandmarkMap();
    ~LandmarkMap();

    // Non-copyable, non-movable (holds a mutex + flann::Index handle).
    LandmarkMap(const LandmarkMap&)            = delete;
    LandmarkMap& operator=(const LandmarkMap&) = delete;

    // ─── Producer API (camera thread, keyframe storage path) ──────────────
    //
    // Add a newly-triangulated 3D point. If a landmark already exists
    // within kDedup3DDistanceM and its descriptor matches within
    // kDescriptorMaxHamming, the existing landmark's observation count
    // is incremented and its id returned. Otherwise a fresh landmark is
    // created and its id returned.
    //
    // Returns -1 on invalid input (empty descriptor, NaN p_world, etc.).
    //
    // Caller MUST hold the camera-thread mutex if they need atomicity
    // with EKF state; this class only protects its own internal state.
    //
    // Legacy 4-arg overload: kept for tests + any caller that doesn't have a
    // host-clone pose handy. Internally calls the 7-arg form with
    // has_anchor=false and an empty initial-pixel so legacy behaviour is
    // preserved (no anchor data, no BA observation).
    int addOrMergeLandmark(const cv::Vec3d& p_world,
                            const cv::Mat&   descriptor,   // 1×32 CV_8U
                            uint64_t         kf_id,
                            int64_t          ts_ns);

    // 2026-05-19 — anchor-relative overload. Pass the host clone's pose
    // (camera→world rotation + camera-position-in-world) at the moment of
    // add so we can compute and store `p_anchor_cam`. Used by the production
    // keyframe-storage call site (Tracker.cpp around the LM_KF block).
    // host_kf_id is reused as host_clone_id — the EKFState clone id matches
    // the kf id used by the loop-closure DB.
    //
    // 2026-05-19 Fix #4 — Extended to also accept the host keyframe's
    // keypoint pixel for this landmark. Pushed onto observation_pixels so
    // the windowed BA has at least one observation to anchor the landmark
    // when its host clone is still in the EKF window. Pass (NaN, NaN) to
    // skip the initial observation entry.
    int addOrMergeLandmark(const cv::Vec3d&   p_world,
                            const cv::Mat&     descriptor,
                            uint64_t           kf_id,
                            const cv::Matx33d& host_R_world_cam,
                            const cv::Vec3d&   host_t_cam_world,
                            const cv::Point2f& host_kp_pixel,
                            int64_t            ts_ns);

    // ─── Consumer API (EKF measurement update, overlay render) ────────────
    //
    // Returns ids of landmarks whose p_world is within `radius_m` of
    // `p_center`. Optionally filters out landmarks not seen in the last
    // `max_age_ns` (pass negative to disable).
    //
    // The returned vector is sorted by ascending squared distance from
    // p_center so the closest landmarks come first (useful when capping
    // the number fed to MSCKF for compute budget).
    std::vector<int> getLandmarksInRadius(const cv::Vec3d& p_center,
                                            double           radius_m,
                                            int64_t          now_ns      = -1,
                                            int64_t          max_age_ns  = -1) const;

    // Copy a single landmark by id. Returns false if id is unknown.
    bool getLandmark(int id, Landmark& out) const;

    // ─── Phase 6.4c — Loop-closure spatial pre-filter ────────────────────
    //
    // Return the SET of keyframe ids that observed any landmark within
    // `radius_m` of `p_center`. Used by LoopClosureDetector to restrict
    // its DBoW2 search to physically-plausible candidates instead of the
    // entire keyframe database.
    //
    // Read-only on the LandmarkMap (no mutation). Reuses the same KD-tree
    // path as getLandmarksInRadius, then unions observed_in_kfs entries.
    //
    // Cap on returned ids: kLcMaxKeyframes (50). Plenty for DBoW2's
    // max_id-style query while bounding worst-case complexity. Sorted by
    // ascending id (so callers iterating monotonically get oldest first).
    //
    // `max_age_ns` filters landmarks not seen recently. Pass < 0 to
    // disable temporal filtering (matches getLandmarksInRadius semantics).
    static constexpr size_t kLcMaxKeyframes = 50;
    std::vector<uint64_t> getKeyframesNearPosition(const cv::Vec3d& p_center,
                                                    double           radius_m,
                                                    int64_t          now_ns      = -1,
                                                    int64_t          max_age_ns  = -1) const;

    // Pinhole projection of landmark id `landmark_id` into the camera
    // defined by (R_world_cam: cam→world, t_cam_world: cam-position in
    // world, fx/fy/cx/cy intrinsics, img_w/img_h bounds). Returns false
    // if the landmark is unknown, behind the camera (z < kDepthMinM), too
    // far (z > kDepthMaxM), or projects outside the image.
    bool projectIntoCamera(int                 landmark_id,
                            const cv::Matx33d&  R_world_cam,
                            const cv::Vec3d&    t_cam_world,
                            double              fx, double fy,
                            double              cx, double cy,
                            int                 img_width,
                            int                 img_height,
                            cv::Point2f&        out_pixel) const;

    // ─── Re-observation API (2026-05-17, investigation agent fix #3) ──────
    //
    // When local-map tracking matches a stored landmark against a current
    // KLT keypoint (Tracker.cpp:3597 area), call touchLandmark to:
    //   * bump times_observed (the landmark is alive — re-confirmed)
    //   * update last_seen_ts_ns (resets the grace timer)
    //   * optionally append the current keyframe id to observed_in_kfs
    //
    // Without this, only the initial addOrMergeLandmark increments
    // times_observed — landmarks die at cull time even when actively
    // re-observed every walk pass. Per investigator finding #3.
    //
    // Returns true if the landmark exists and was touched.
    bool touchLandmark(int landmark_id, uint64_t kf_id, int64_t ts_ns);

    // 2026-05-19 Fix #4 — touchLandmark overload that also records the
    // current-frame KLT-matched pixel into observation_pixels (ring
    // buffer). Required for the windowed BA to refine `p_world`: BA needs
    // ≥ 2 observations across the BA window's clones, and the only place
    // we have per-keyframe pixel-resolution data is at the local-map
    // match site (Tracker.cpp around the LM_TRACK block). Cap enforced via
    // kObsHistoryCap. Returns true if the landmark exists.
    bool touchLandmark(int landmark_id, uint64_t kf_id, int64_t ts_ns,
                        const cv::Point2f& matched_pixel);

    // ─── Mutator API (loop-closure pose-graph back-write) ─────────────────
    //
    // Apply a 4-DOF correction (Δp, Δyaw around world-Z) to every
    // landmark observed in `kf_id`. Mirrors LoopClosureDetector::
    // applyKeyframePoseCorrection — same correction is applied to the
    // keyframe DB pose and to all landmarks anchored to that keyframe.
    //
    // Returns the number of landmarks shifted.
    int applyKeyframePoseCorrection(uint64_t kf_id,
                                      double dx, double dy, double dz,
                                      double dyaw);

    // 2026-05-19 Fix #4 — BA refinement integration. Two APIs:
    //
    // 1) getLandmarksWithObsInClones — gather landmarks the BA can refine.
    //    Returns landmarks whose observation_pixels has ≥ min_obs entries
    //    whose clone_id is in `clone_ids_window`. Each result carries the
    //    filtered observation list (only clones in the window). The
    //    caller (Tracker::kickOffBARound) feeds these into
    //    WindowedBA::FeatureObs alongside the SLAM-feature entries.
    //
    // 2) setLandmarkPosition — BA write-back. Replaces lm.p_world with the
    //    refined value after WindowedBA accepts. Marks the spatial KD-tree
    //    dirty so the next radius query rebuilds. Returns true on success.
    //
    // Both APIs are mutex-protected and may be called from the BA worker
    // thread (after the camera thread has built its read-snapshots).
    struct LandmarkObsForBA {
        int                                          id     = -1;
        cv::Vec3d                                    p_world{0.0, 0.0, 0.0};
        std::vector<std::pair<int, cv::Point2f>>     obs;  // (clone_id, pixel_uv)
    };
    std::vector<LandmarkObsForBA> getLandmarksWithObsInClones(
            const std::vector<int>& clone_ids_window,
            int                      min_obs) const;

    bool setLandmarkPosition(int landmark_id, const cv::Vec3d& p_world);

    // ─── Maintenance ──────────────────────────────────────────────────────
    //
    // Remove landmarks whose times_observed < kMinObservationsAfterGrace
    // and whose first_seen_ts_ns > kGracePeriodNs ago. Call once per
    // keyframe (cheap). Returns the number culled.
    int cullStaleLandmarks(int64_t now_ns);

    void reset();

    // ─── Diagnostics ──────────────────────────────────────────────────────
    size_t size() const;
    int    landmarksAddedTotal()   const;
    int    landmarksMergedTotal()  const;
    int    landmarksCulledTotal()  const;
    int    kdRebuildsTotal()       const;

private:
    void rebuildKdTreeLocked() const;
    int  findMergeCandidateLocked(const cv::Vec3d& p_world,
                                    const cv::Mat& descriptor) const;

    // Shared impl behind both addOrMergeLandmark overloads. set_anchor=false
    // skips the host-pose anchor computation (used by the legacy 4-arg path
    // and tests). 2026-05-19 Fix #4 — also accepts an optional initial
    // keypoint pixel (NaN pair = skip the observation push) for seeding
    // observation_pixels.
    int  addOrMergeLandmarkImpl(const cv::Vec3d&   p_world,
                                  const cv::Mat&     descriptor,
                                  uint64_t           kf_id,
                                  const cv::Matx33d& host_R_world_cam,
                                  const cv::Vec3d&   host_t_cam_world,
                                  const cv::Point2f& host_kp_pixel,
                                  int64_t            ts_ns,
                                  bool               set_anchor,
                                  bool               record_initial_obs);

    mutable std::mutex mutex_;
    std::unordered_map<int, Landmark> landmarks_;
    int next_id_{0};

    // Lazy KD-tree spatial index.
    mutable cv::Mat                          kd_data_;        // Nx3 CV_32F
    mutable std::vector<int>                 kd_id_map_;      // row → landmark id
    mutable std::unique_ptr<cv::flann::Index> kd_index_;
    mutable bool                             kd_dirty_{true};
    mutable int                              kd_adds_since_rebuild_{0};
    mutable int                              kd_rebuilds_total_{0};

    // Counters (mirrored into EventCounters by the caller).
    int landmarks_added_total_{0};
    int landmarks_merged_total_{0};
    int landmarks_culled_total_{0};
};

}  // namespace navsight
