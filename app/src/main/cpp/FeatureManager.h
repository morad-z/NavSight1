#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Grid-based feature detection and spatial distribution enforcement.
// Ensures features are spread across the image, not clustered in one area.
// Also manages keyframe storage for re-localization after tracking loss.
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

private:
    int countInCell(const std::vector<cv::Point2f>& points,
                    int row, int col, int cell_w, int cell_h) const;

    std::vector<Keyframe> keyframes_;

    static constexpr int GRID_ROWS = 4;
    static constexpr int GRID_COLS = 5;
    static constexpr int MAX_KEYFRAMES = 10;
    static constexpr int MIN_KF_MATCHES = 20;
    static constexpr float KF_MATCH_RADIUS = 15.0f;  // pixels
};
