#pragma once
#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

// Per-keyframe ORB descriptor record stored in FeatureManager's
// ring buffer for relocalization (Plan Step 4, ADR-010).
//
// - keypoints / descriptors / feature_ids are parallel arrays.
// - feature_ids[i] = -1 when an ORB keypoint did not align with any
//   tracked KLT corner at storage time (no FeatureManager id available).
// - descriptors is a CV_8U matrix with 32 columns (ORB binary).
// - R_world_cam / t_cam_world / has_pose: optional camera pose at the
//   moment of capture, populated by Tracker after the keyframe's clone is
//   added to the EKF. Used by Step 7 loop closure to triangulate ORB
//   keypoint pairs across consecutive keyframes (since EKF MAX_SLAM_FEATURES
//   is small — only ~12 of 500 ORB rows would otherwise have a 3D anchor).
//   has_pose stays false if Tracker never gets around to publishing the
//   pose — downstream code must check.
struct KeyframeDescriptors {
    uint64_t                  keyframe_id   = 0;
    double                    timestamp_ns  = 0.0;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;
    std::vector<int>          feature_ids;
    cv::Matx33d               R_world_cam   = cv::Matx33d::eye();
    cv::Vec3d                 t_cam_world   = cv::Vec3d(0, 0, 0);
    bool                      has_pose      = false;
};
