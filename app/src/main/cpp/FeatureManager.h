#pragma once
#include <vector>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "UpdaterMSCKF.h"
#include "KeyframeDescriptors.h"

// Grid-based feature detection, spatial distribution enforcement,
// and multi-clone feature track management for MSCKF.
class FeatureManager {
public:
    FeatureManager();

    // Detect features with grid-based spatial distribution.
    // Divides the image into cells and detects per-cell to ensure coverage.
    void detectGridFeatures(const cv::Mat& gray,
                            std::vector<cv::Point2f>& out_points,
                            int max_total, double quality_level, double min_dist);

    // Replenish features in cells that have gone sparse.
    // existing_points: currently tracked points (will not be duplicated).
    // out_new: newly detected points to add.
    void replenishSparse(const cv::Mat& gray,
                         const std::vector<cv::Point2f>& existing_points,
                         std::vector<cv::Point2f>& out_new,
                         int max_total, double quality_level, double min_dist);

    // Store a keyframe for re-localization and heading correction
    struct Keyframe {
        cv::Mat gray;
        std::vector<cv::Point2f> points;
        int64_t timestamp_ns;
        int frame_id;
        double heading;         // scalar heading at keyframe time
        cv::Point3f position;   // global position at keyframe time
    };

    void storeKeyframe(const cv::Mat& gray,
                       const std::vector<cv::Point2f>& points,
                       int64_t timestamp_ns, int frame_id,
                       double heading = 0.0,
                       cv::Point3f position = cv::Point3f(0,0,0));

    // Try to match current features against the last keyframe.
    // Returns true if enough matches found, fills matched pairs.
    bool matchAgainstKeyframe(const cv::Mat& gray,
                              const std::vector<cv::Point2f>& current_points,
                              std::vector<cv::Point2f>& kf_matched,
                              std::vector<cv::Point2f>& cur_matched) const;

    // Get last keyframe heading/position (for drift correction)
    bool getLastKeyframeInfo(double& heading_out, cv::Point3f& position_out) const {
        if (keyframes_.empty()) return false;
        heading_out = keyframes_.back().heading;
        position_out = keyframes_.back().position;
        return true;
    }

    // ── Plan Step 4 (ADR-010): ORB descriptors at keyframes ────────────────
    //
    // Extract ORB on the keyframe's gray image, attach feature_ids by spatial
    // proximity (≤ 3 px) to the supplied tracked KLT corners, and push the
    // record into a ring buffer capped at the last KEYFRAME_DESC_RING_SIZE
    // entries (50). Used by the relocalization path in Tracker::processFrame.
    //
    //   kf_id            : monotonic keyframe id (current frame_counter_).
    //   ts_ns            : keyframe timestamp (ns) — copied verbatim into the
    //                       record's timestamp_ns (header is double per Agent
    //                       C's struct contract).
    //   gray             : keyframe grayscale image (CV_8UC1).
    //   corner_pts       : KLT corners present at the keyframe (parallel to
    //                       corner_feature_ids).
    //   corner_feature_ids : FeatureManager id per corner; -1 if unassigned.
    void storeKeyframeDescriptors(uint64_t kf_id,
                                  double ts_ns,
                                  const cv::Mat& gray,
                                  const std::vector<cv::Point2f>& corner_pts,
                                  const std::vector<int>& corner_feature_ids);

    // Read accessor for the relocalization scan in Tracker. The caller walks
    // the most recent N entries (Plan Step 4 uses the last 5).
    const std::deque<KeyframeDescriptors>& getKeyframeDescriptors() const {
        return keyframe_descriptors_;
    }

    // ── Plan Step 4 (ADR-010) ORB extractor params (PUBLIC) ────────────────
    // These are intentionally exposed so Tracker's relocalization path can
    // build a matching cv::ORB extractor without duplicating magic numbers.
    // Keeping them PUBLIC and constexpr means there is one source of truth
    // (FeatureManager's storeKeyframeDescriptors used the same values), and
    // a divergence between extraction and matching params is a compile-time
    // impossibility rather than a runtime mismatch.
    static constexpr int    ORB_TARGET_FEATURES = 250;  // density vs CPU
    static constexpr int    ORB_FAST_THRESHOLD  = 10;   // texture-poor floor
    static constexpr double ORB_PREBLUR_SIGMA   = 1.0;  // FAST noise reject

    void reset();

    // ── MSCKF Feature Track Management ──────────────────────────────────────

    // Assign unique IDs to a set of newly detected features.
    // Returns vector of IDs (same size as points).
    std::vector<int> assignIds(int count);

    // Record that feature_id was observed from clone_state_id at pixel_ud.
    void addObservation(int feature_id, int clone_state_id,
                        const cv::Point2f& pixel_ud);

    // Given current tracked feature IDs, find features that are no longer
    // being tracked (lost) and have enough observations for MSCKF update.
    // Removes lost features from internal tracking state.
    std::vector<LostFeature> extractLostFeatures(
        const std::vector<int>& current_ids, int min_obs = 3);

    // Plan Step 3a: lost-feature candidates with ≥ min_obs observations
    // across clones, ready for MSCKF null-space update.
    std::vector<LostFeature> getMSCKFCandidates(
        const std::vector<int>& current_ids, int min_obs = 4) {
        return extractLostFeatures(current_ids, min_obs);
    }

    // Remove observations referencing clones that have been marginalized.
    void pruneObservations(int min_clone_id);

    // ── Plan Step 3b (ADR-009): SLAM feature lifecycle ───────────────────────
    //
    // Per-feature tracking record used to decide promotion to a SLAM slot in
    // EKFState and demotion when reprojection RMS goes bad. The tracking
    // counters are advanced by `noteObservation` / `noteKeyframe` from the
    // Tracker hot loop (cheap O(1) per observation) and queried via the
    // promote / demote / lost helpers below.
    struct FeatureLifecycle {
        int     feature_id{-1};
        int     age{0};                 // # observations
        int     kf_count{0};             // # keyframes feature spans
        int     slam_slot{-1};           // -1 => MSCKF candidate / not promoted
        int     rms_bad_consecutive{0};  // # consecutive frames with rms > 3 px
        double  last_rms_px{0.0};
        int64_t last_obs_ns{0};
        // Last triangulated 3-D point (world frame). Updated when the
        // promotion gate fires; used to seed `EKFState::addSlamFeature`.
        cv::Point3f last_p_global{0.0f, 0.0f, 0.0f};
        bool        has_p_global{false};
        int         anchor_clone_id{-1};
    };

    // Record an observation tick on a feature. Call from Tracker each frame
    // a feature is still tracked (after `addObservation` for the EKF clone
    // observation). `is_keyframe` advances the keyframe counter; pass false
    // and use `noteKeyframe` separately when the keyframe decision is made
    // later in the frame.
    void noteObservation(int feature_id, int64_t ts_ns, bool is_keyframe);

    // Record that a feature was present at a keyframe. Increments kf_count.
    // Call from Tracker in the keyframe-storage block, once per kept ID.
    void noteKeyframe(int feature_id);

    // Record a triangulated 3-D point + anchor for the feature. Call before
    // querying `getPromotableFeatures` so the promotion gate has a position
    // to hand to EKFState::addSlamFeature.
    void noteTriangulation(int feature_id,
                           const cv::Point3f& p_global,
                           int anchor_clone_id);

    // Mark a feature as promoted to slot `slam_slot` (>= 0). Pass -1 to
    // demote it back to MSCKF candidacy (e.g. after Schur removal).
    void setSlamSlot(int feature_id, int slam_slot);

    // Returns feature IDs that meet the promotion gate. Defaults follow the
    // task brief:
    //   - track length ≥ min_obs (12)
    //   - keyframe count ≥ min_kf (2)
    //   - last reprojection RMS ≤ max_init_rms_px (1.5 px)
    //   - has a triangulated p_global
    //   - is not already promoted
    std::vector<int> getPromotableFeatures(int min_obs = 12,
                                            int min_kf = 2,
                                            double max_init_rms_px = 1.5) const;

    // Increment / reset the rms_bad_consecutive counter. > 3 px => bad.
    void markSlamFeatureRMS(int feature_id, double rms_px);

    // SLAM-promoted features that have failed reprojection RMS ≥ 3 frames
    // in a row. Returned IDs should be removed from EKFState.
    std::vector<int> getDemoteCandidates() const;

    // SLAM-promoted features whose last observation timestamp is older
    // than `lost_threshold_ns`. Returned IDs should be removed from
    // EKFState.
    std::vector<int> getLostSlamFeatures(int64_t now_ns,
        int64_t lost_threshold_ns = 1'000'000'000LL) const;

    // Read-only access for the Tracker glue layer; in particular returns
    // (anchor_clone_id, p_global) for promotion and the current slot for
    // demotion lookups. nullptr if `feature_id` is unknown.
    const FeatureLifecycle* getLifecycle(int feature_id) const;

    // Read-only access to a feature's full observation history across
    // clones. Tracker uses this to triangulate at promotion and to build
    // the per-frame SLAM-feature update. Returns nullptr if the feature
    // has no recorded observations.
    const std::vector<FeatureObservation>* getObservations(int feature_id) const;

    // Remove a feature's lifecycle record entirely. Called after
    // EKFState::removeSlamFeature so the demote candidates do not loop.
    void dropLifecycle(int feature_id);

    // All feature IDs that currently have a lifecycle record, regardless
    // of whether they are tracked in the current frame. Used by the
    // Tracker SLAM lifecycle to reconcile slot indices after one or more
    // EKF removals — `feature_ids_` (current-frame tracks) misses
    // SLAM-promoted features that have no current observation, so a walk
    // of just that set leaves stale `slam_slot` values in the lifecycle
    // map and the next removeSlamFeature(stale_slot) silently fails.
    std::vector<int> getAllLifecycleFeatureIds() const;

    // Number of lifecycle records currently held. Used for perf telemetry
    // so we can verify that the per-frame prune (below) is keeping the
    // map bounded. O(1).
    size_t getLifecycleSize() const { return lifecycle_.size(); }

    // ── Plan Step 6 (ADR-012): thread-safe SLAM-landmark snapshot for BA ───
    //
    // Read-only copy of every SLAM-promoted feature (slam_slot >= 0) along
    // with its world-frame seed point (`last_p_global`) and its per-clone
    // pixel observation history. Designed for the BA worker thread.
    //
    // The observations here are returned in PIXEL space. The internal
    // `active_tracks_` stores normalised (un-distorted) camera-frame uv,
    // because the legacy MSCKF path consumes them that way; the BA worker
    // wants pixels so reprojection error is in pixel units. The caller
    // supplies the live intrinsics (fx, fy, cx, cy) so the snapshot is
    // independent of any state-cached intrinsics.
    //
    // The min_obs gate filters out landmarks that would be observed by < 2
    // poses in the BA window — those contribute no information to the
    // joint solve. The caller passes its own clone-id whitelist (the
    // CloneSnapshot result from EKFState::getCloneSnapshot) so we only
    // emit observations that align to a pose actually being optimised.
    //
    // Snapshot lock protects `lifecycle_` and `active_tracks_` against
    // concurrent mutation from the camera thread (addObservation,
    // noteObservation, noteTriangulation, setSlamSlot, markSlamFeatureRMS,
    // pruneStaleLifecycle, dropLifecycle, reset). The lock is taken only
    // for the duration of the read-and-copy.
    struct LandmarkSnapshot {
        int        feature_id = -1;
        int        slam_slot  = -1;
        cv::Vec3d  p_world;                         // world-frame seed point
        // Per-clone pixel observations. The `int` keys correspond to
        // EKFState::CloneSnapshot::clone_id (== CameraPose::state_id).
        std::vector<std::pair<int, cv::Vec2d>> obs;
    };
    std::vector<LandmarkSnapshot>
    getLandmarkSnapshot(const std::vector<int>& clone_id_whitelist,
                         double fx, double fy, double cx, double cy,
                         int min_obs = 2) const;

    // Garbage-collect lifecycle records that are no longer useful. An
    // entry is kept only if (a) its feature_id is in `active_ids` (still
    // tracked this frame), OR (b) it is currently promoted as a SLAM
    // feature (slam_slot >= 0). Anything else is a dead lifecycle
    // record left over from a transient KLT track that never matured —
    // we observed it once, the track died, and the entry would otherwise
    // sit in the map forever, padding every getPromotableFeatures /
    // getDemoteCandidates / getLostSlamFeatures scan with O(N) noise.
    //
    // Without this prune, the lifecycle map grows ~200 entries/sec under
    // a typical KLT churn (200 features × 30 fps × turnover) and the
    // three per-frame scans turn into a multi-millisecond hot path.
    // Empirically: 17 minutes runtime → 50–90 ms total in promote+demote
    // scans alone (logs from the 2026-05-04 profiling pass).
    //
    // Recently-observed entries (last_obs_ns within LIFECYCLE_KEEP_NS of
    // the supplied now_ns) are also kept even when they fell out of
    // active_ids this frame — KLT routinely misses a feature for 1-2
    // frames during motion blur and recovers it next frame. Dropping
    // such an entry would silently reset the SLAM promotion gate (age,
    // kf_count, observation history) for any feature that ever
    // flickered, effectively denying SLAM promotion under realistic
    // motion. Entries whose last_obs_ns is 0 (created but never
    // observed) are dropped normally — they are stillborn records.
    //
    // Returns the number of entries dropped.
    static constexpr int64_t LIFECYCLE_KEEP_NS = 100'000'000LL;  // 100 ms
    int pruneStaleLifecycle(const std::vector<int>& active_ids,
                            int64_t now_ns);

    // DEAD CODE: getNextFeatureId — never called
    // int getNextFeatureId() const { return next_feature_id_; }

private:
    int countInCell(const std::vector<cv::Point2f>& points,
                    int row, int col, int cell_w, int cell_h) const;

    std::vector<Keyframe> keyframes_;

    // Plan Step 4 (ADR-010): ORB descriptor ring buffer for relocalization.
    // Capped at KEYFRAME_DESC_RING_SIZE; oldest evicted on overflow.
    // Lazily-built ORB extractor so the cost (cv::ORB::create allocates a
    // FastFeatureDetector + descriptor extractor) is paid at most once per
    // FeatureManager lifetime instead of per keyframe.
    std::deque<KeyframeDescriptors> keyframe_descriptors_;
    cv::Ptr<cv::ORB>                orb_extractor_;

    // MSCKF: per-feature observation history
    int next_feature_id_{0};
    std::unordered_map<int, std::vector<FeatureObservation>> active_tracks_;

    // Plan Step 3b (ADR-009): per-feature lifecycle record. Keyed by
    // feature_id; populated by noteObservation / noteTriangulation /
    // markSlamFeatureRMS from the Tracker hot loop. O(1) per call.
    std::unordered_map<int, FeatureLifecycle> lifecycle_;

    // ── Plan Step 6 (ADR-012): snapshot mutex ──────────────────────────────
    // Guards `active_tracks_` and `lifecycle_` against torn reads from the
    // BA worker thread. Camera-thread writers (addObservation,
    // noteObservation, noteKeyframe, noteTriangulation, setSlamSlot,
    // markSlamFeatureRMS, pruneObservations, pruneStaleLifecycle,
    // dropLifecycle, extractLostFeatures, reset) take it briefly while
    // mutating; getLandmarkSnapshot takes it for the read-and-copy.
    // mutable so the const snapshot accessor can lock for read.
    mutable std::mutex snapshot_mutex_;

    static constexpr int GRID_ROWS = 4;
    static constexpr int GRID_COLS = 5;
    static constexpr int MAX_KEYFRAMES = 10;
    static constexpr int MIN_KF_MATCHES = 20;
    static constexpr float KF_MATCH_RADIUS = 15.0f;  // pixels

    // ── Plan Step 5 (ADR-011): adaptive low-light feature replenish ────────
    //
    // Brightness gate consumed by replenishSparse. Centre-crop mean of the
    // gray frame, normalised to [0, 1]. Below the threshold the replenish
    // path swaps the steady-state (Tracker::MAX_FEATURES = 200,
    // Tracker::QUALITY_LEVEL = 0.05) targets for the low-light pair below.
    //
    // Empirically below ~0.12 (32/255 in 8-bit) phone cameras enter
    // ISO-pumped territory; corner SNR collapses. In low light, weak
    // corners are dominated by sensor noise; doubling the quality
    // threshold rejects them, and lowering the target acknowledges fewer
    // strong corners are physically available. Net effect: fewer but
    // more reliable features survive into KLT.
    static constexpr double BRIGHTNESS_LOW_THRESH      = 0.12;
    static constexpr int    REPLENISH_TARGET_LOWLIGHT  = 120;   // vs 200
    static constexpr double REPLENISH_QUALITY_LOWLIGHT = 0.10;  // vs 0.05
    // Centre-crop fraction along each axis. 0.5 mirrors the "central half"
    // ROI used by Tracker's blur detector so the brightness probe sees the
    // same patch — keeps the gate consistent with motion-blur skip frames.
    static constexpr double BRIGHTNESS_CENTRE_FRAC     = 0.5;
    // Hysteresis on consecutive-frame transitions: the low-light decision
    // only flips after this many frames sustain the new state. Prevents
    // feature-count thrashing across auto-exposure ramps where the mean
    // brightness oscillates around the 0.12 boundary frame to frame.
    static constexpr int    BRIGHTNESS_HYSTERESIS_FRAMES = 5;

    // Hysteresis state for the low-light gate, owned per-instance so a
    // mid-session FeatureManager::reset() (e.g. recalibration) clears them
    // back to defaults instead of leaking state across the bootstrap.
    int  low_light_low_streak_     = 0;     // consecutive frames below thresh
    int  low_light_normal_streak_  = 0;     // consecutive frames at/above
    bool low_light_state_          = false; // committed state (post-hysteresis)
    int  low_light_log_counter_    = 0;     // rate-limit LOGI to every 30 hits

    // ── Plan Step 4 (ADR-010) ORB descriptor private constants ─────────────
    // (Public ORB extractor params — TARGET_FEATURES / FAST_THRESHOLD /
    //  PREBLUR_SIGMA — are declared in the public section so the Tracker's
    //  relocalization path can build a matching cv::ORB without re-stating
    //  the magic numbers.)
    //
    // Spatial-proximity radius for ORB→KLT id inheritance (px). 3 px chosen
    // because cv::cornerSubPix already tightens KLT corners to ≤ 1 px and
    // ORB's pyramid keypoint coords drift up to ~2 px at coarser octaves.
    static constexpr float ORB_KLT_MATCH_RADIUS  = 3.0f;
    // Ring buffer cap. 50 keyframes × 250 features × 32 bytes (ORB row) ≈
    // 400 KB descriptor payload + cv::KeyPoint metadata; well under the
    // 10 MB RAM headroom budgeted in the visual-production plan.
    static constexpr size_t KEYFRAME_DESC_RING_SIZE = 50;
};
