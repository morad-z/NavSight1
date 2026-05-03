#pragma once
#include <vector>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "UpdaterMSCKF.h"

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

    // DEAD CODE: getNextFeatureId — never called
    // int getNextFeatureId() const { return next_feature_id_; }

private:
    int countInCell(const std::vector<cv::Point2f>& points,
                    int row, int col, int cell_w, int cell_h) const;

    std::vector<Keyframe> keyframes_;

    // MSCKF: per-feature observation history
    int next_feature_id_{0};
    std::unordered_map<int, std::vector<FeatureObservation>> active_tracks_;

    // Plan Step 3b (ADR-009): per-feature lifecycle record. Keyed by
    // feature_id; populated by noteObservation / noteTriangulation /
    // markSlamFeatureRMS from the Tracker hot loop. O(1) per call.
    std::unordered_map<int, FeatureLifecycle> lifecycle_;

    static constexpr int GRID_ROWS = 4;
    static constexpr int GRID_COLS = 5;
    static constexpr int MAX_KEYFRAMES = 10;
    static constexpr int MIN_KF_MATCHES = 20;
    static constexpr float KF_MATCH_RADIUS = 15.0f;  // pixels
};
