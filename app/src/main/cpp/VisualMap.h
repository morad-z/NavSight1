#pragma once
#include <opencv2/core.hpp>
#include <unordered_map>
#include <mutex>

// 2026-05-12 — visual map module, stage 1 of SLAM-EKF decoupling.
//
// Why this exists:
//   The EKF currently carries SLAM features as inverse-depth (alpha, beta, rho)
//   state coupled to IMU-derived clone poses. Every per-frame Kalman update
//   on this representation is sensitive to IMU bias errors, frame-convention
//   bugs at the body/camera boundary, and the parallax/observability gates
//   needed to keep rho stable. That coupling has been the source of multiple
//   multi-day debugging sessions.
//
//   `VisualMap` is the start of moving persistent landmarks OUT of the EKF
//   state and into a standalone visual map (the architecture ORB-SLAM3
//   uses: pose tracking and SLAM map are decoupled, communicating only
//   through pose hypotheses).
//
// Stage 1 (this commit): cache the world position computed at promotion
// time. Rendering reads from here instead of reconstructing through the
// EKF inverse-depth representation. The EKF SLAM state still exists and
// gets per-frame updates; that's the next stage to remove.
//
// Per Morad's "comment, don't delete" policy: when EKF SLAM coupling is
// removed in stage 2, the existing code paths in EKFState.cpp and
// Tracker.cpp will be commented out, not deleted.
class VisualMap {
public:
    struct MapPoint {
        int     feature_id    = -1;
        cv::Mat p_world;          // 3x1 CV_64F, Z-up world frame
        int     anchor_clone_id = -1;
        int64_t promoted_ts_ns  = 0;
        // Last reprojection RMS at the time the map point was last verified.
        // Used to expire stale map points whose underlying feature dropped.
        double  last_rms_px     = 0.0;
    };

    // Add or overwrite the world position for a feature. If the feature
    // was already in the map, its anchor and timestamp are updated.
    void addOrUpdate(int feature_id,
                     const cv::Mat& p_world,
                     int anchor_clone_id,
                     int64_t ts_ns);

    // Read the cached world position for a feature. Returns false if the
    // feature has not been promoted into the map.
    bool getWorldPosition(int feature_id, cv::Mat& p_world_out) const;

    // Record the latest reprojection RMS for this feature so consumers
    // can decide to demote / expire.
    void noteReprojectionRMS(int feature_id, double rms_px);

    // Remove a feature from the map (e.g. when EKF demotes it).
    void remove(int feature_id);

    // Wipe everything (e.g. on Tracker::reset).
    void clear();

    // Snapshot of the live map for the rendering / sim recorder path.
    // Returns a copy under the mutex so the caller doesn't have to hold a
    // lock while iterating. Map is small (≤ a few dozen entries typically),
    // copy cost is negligible relative to the JNI hop.
    std::vector<MapPoint> snapshot() const;

    size_t size() const;

private:
    mutable std::mutex mutex_;
    std::unordered_map<int, MapPoint> points_;
};
