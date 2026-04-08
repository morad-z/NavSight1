#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <opencv2/core.hpp>
#include "LoopClosureDetector.h"
#include "PoseGraph.h"
#include "EKFState.h"

struct TrackerFrame;  // forward declaration

// Results returned from each Mapper subsystem
struct GroundPlaneResult {
    bool   is_valid;
    double ground_scale;
    double confidence;
};

struct BAResult {
    bool   is_valid;
    double optimized_scale;
    double alpha;           // blending weight for smooth_scale_ update
};

struct LoopClosureResult {
    bool   detected;
    double target_x, target_y, target_z;
    double target_heading;
    double blend;
};

struct MapperResult {
    GroundPlaneResult  ground_plane;
    BAResult           bundle_adj;
    BAResult           depth_scale;
    LoopClosureResult  loop_closure;
};

class Mapper {
public:
    Mapper();

    // Run all background processing for this frame.
    // current_smooth_scale: snapshot from Tracker for BA temporal constraint.
    // ekf: read-only access to MSCKF sliding window for clone-based BA.
    MapperResult process(const TrackerFrame& frame, double current_smooth_scale,
                         const EKFState* ekf = nullptr);
    void setDepthMap(const float* depth_data, int width, int height);
    void setCameraHeight(double height_m) { user_camera_height_ = height_m; }
    void reset();

private:
    GroundPlaneResult  detectGroundPlane(const TrackerFrame& frame, const EKFState* ekf);
    BAResult           runBundleAdjustment(const TrackerFrame& frame,
                                           double current_smooth_scale,
                                           const EKFState* ekf);
    LoopClosureResult  detectLoopClosure(const TrackerFrame& frame);
    BAResult           constrainScaleWithDepth(const TrackerFrame& frame);

    // ── Depth map state ──────────────────────────────────────────────────────
    std::mutex         depth_mutex_;
    std::vector<float> latest_depth_map_;
    int                depth_width_{0}, depth_height_{0};

    // ── Ground plane state ───────────────────────────────────────────────────
    struct GroundPlaneEstimate {
        cv::Point2f vanishing_point;
        double camera_pitch;
        double ground_distance;
        double confidence;
        bool   is_valid;
    };
    GroundPlaneEstimate ground_plane_estimate_{};
    double user_camera_height_{1.4};  // meters (standing with phone)

    // ── Bundle adjustment state ──────────────────────────────────────────────
    struct Keyframe {
        int frame_id;
        cv::Mat pose;
        std::vector<cv::Point2f> keypoints;
        std::vector<cv::Point3f> points_3d;
        int64_t timestamp_ns;
    };

    class KeyframeWindow {
    public:
        void add(const Keyframe& kf) {
            window_.push_back(kf);
            if (window_.size() > 8) window_.pop_front();
        }
        size_t size() const { return window_.size(); }
        bool has_minimum() const { return window_.size() >= 2; }
        const std::deque<Keyframe>& data() const { return window_; }
    private:
        std::deque<Keyframe> window_;
    };

    KeyframeWindow keyframe_window_;
    double  bundle_adj_scale_{1.0};
    int     bundle_adj_iteration_{0};
    int64_t last_ba_timestamp_ns_{0};

    static constexpr int    BA_INTERVAL = 5;
    static constexpr int    BA_MAX_ITER = 10;
    static constexpr double HUBER_THRESHOLD = 2.0;  // pixels

    // ── Loop closure state ───────────────────────────────────────────────────
    LoopClosureDetector loop_closure_detector_;
    PoseGraph pose_graph_;
    int          frames_since_last_keyframe_{0};
    cv::Point3f  last_keyframe_position_{0, 0, 0};
    int          last_pose_graph_node_id_{-1};
};
