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

    // 2026-05-19 — ORB-distortion root cause fix.
    //
    // Cause: cv::ORB::detectAndCompute runs on the raw camera image, so
    // `keypoints[i].pt` is in DISTORTED pixel coordinates. Every downstream
    // ORB consumer (LandmarkMap triangulation, MSCKF observation residual,
    // local-map match, LC PnP) projects through the LINEAR pinhole intrinsics
    // K (fx, fy, cx, cy) with no distortion correction. Mixing distorted
    // pixels with a linear projection matrix biases the triangulated 3D
    // point by the lens displacement (5-20 px at image edges for the
    // calibrated S21 Ultra coefficients k1=0.263 k2=0.830 k3=12.31), which
    // projects to several cm of 3D error per landmark. That's the dominant
    // source of orange-dot drift the user has been reporting — landmark
    // p_world is BIASED from the moment of creation, and the EKF's
    // measurement model can't reconcile it (62% chi² reject rate in v56).
    //
    // KLT had this exact bug found-and-fixed in v23.4 (see Tracker.cpp
    // comment at line ~2726): KLT pixels are now undistorted before any
    // EKF math touches them. ORB was missed. This field is the parallel
    // fix: populate `keypoints_ud` at extraction time by calling
    // LensCorrector::undistortPoints on the raw keypoint pts. All math
    // consumers read keypoints_ud[i].pt. The raw `keypoints[i].pt` field
    // is preserved for two reasons: (1) display paths (overlay drawing
    // at the matched pixel on the raw camera preview), (2) ORB-detector
    // semantics (the descriptor was computed at the raw position, so the
    // .pt is the descriptor's native anchor for debugging).
    //
    // Falsifier: post-fix the magnitude of landmark drift per-frame at
    // standstill should fall by the distortion-bias-relative-to-depth
    // ratio (typically 50-90% reduction). `landmarks_msckf_rejected_chi2`
    // / `accepted` ratio should improve markedly because predicted and
    // measured pixels now live in the same coordinate space.
    std::vector<cv::KeyPoint> keypoints_ud;
};
