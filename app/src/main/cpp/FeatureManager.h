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

    // Store a keyframe for re-localization
    struct Keyframe {
        cv::Mat gray;
        std::vector<cv::Point2f> points;
        int64_t timestamp_ns;
        int frame_id;
    };

    void storeKeyframe(const cv::Mat& gray,
                       const std::vector<cv::Point2f>& points,
                       int64_t timestamp_ns, int frame_id);

    // Try to match current features against the last keyframe.
    // Returns true if enough matches found, fills matched pairs.
    bool matchAgainstKeyframe(const cv::Mat& gray,
                              const std::vector<cv::Point2f>& current_points,
                              std::vector<cv::Point2f>& kf_matched,
                              std::vector<cv::Point2f>& cur_matched) const;

    int getKeyframeCount() const { return static_cast<int>(keyframes_.size()); }
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

    // Remove observations referencing clones that have been marginalized.
    void pruneObservations(int min_clone_id);

    int getNextFeatureId() const { return next_feature_id_; }

private:
    int countInCell(const std::vector<cv::Point2f>& points,
                    int row, int col, int cell_w, int cell_h) const;

    std::vector<Keyframe> keyframes_;

    // MSCKF: per-feature observation history
    int next_feature_id_{0};
    std::unordered_map<int, std::vector<FeatureObservation>> active_tracks_;

    static constexpr int GRID_ROWS = 4;
    static constexpr int GRID_COLS = 5;
    static constexpr int MAX_KEYFRAMES = 10;
    static constexpr int MIN_KF_MATCHES = 20;
    static constexpr float KF_MATCH_RADIUS = 15.0f;  // pixels
};
