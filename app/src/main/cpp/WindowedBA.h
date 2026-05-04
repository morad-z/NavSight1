// WindowedBA.h — Plan Step 6 / ADR-012
//
// Hand-rolled local windowed bundle adjustment.
// Refines a small window of keyframe poses (SE(3)) and SLAM landmark
// positions jointly by minimising Huber-weighted reprojection error.
// No third-party deps beyond OpenCV (cv::Mat for matrix algebra,
// cv::Rodrigues for SO(3) <-> rotation-vector). Designed for K <= 10
// and N <= 30 — i.e. a 90-DOF problem that fits comfortably on a phone
// without pulling in Ceres / Eigen.
//
// Conventions (consistent across the file):
//   * Pose is (R, t) where R rotates world -> cam (R_cw)
//     and t is the camera centre expressed in the world frame (t_wc).
//     Therefore  p_cam = R * (p_world - t).
//   * Pose perturbation is right-multiplicative on rotation,
//     additive in world-frame on translation:
//        R_new = R_old * Exp(phi_delta)          // 3-DOF rot vec
//        t_new = t_old + delta_t                 // 3-DOF world-frame
//     Local 6-vector ordering: [delta_t (3), phi_delta (3)].
//   * Landmark perturbation is additive in the world frame.
//
// Author: NavSight Plan Step 6 (morad branch).
#pragma once

#include <opencv2/core.hpp>
#include <utility>
#include <vector>

class WindowedBA {
public:
    struct PoseObs {
        int          keyframe_id   = -1;
        cv::Matx33d  R_in;            // input pose, world -> cam
        cv::Vec3d    t_in;            // input camera-in-world translation
        cv::Matx33d  R_out;           // refined output (after solve)
        cv::Vec3d    t_out;           // refined output
        bool         is_anchor = false;  // gauge-fix oldest KF
    };

    struct FeatureObs {
        int        feature_id   = -1;
        cv::Vec3d  p_w_in;            // world-frame landmark, input
        cv::Vec3d  p_w_out;           // refined output
        // Per-keyframe pixel observations. The int must reference a
        // PoseObs::keyframe_id supplied to solve().
        std::vector<std::pair<int, cv::Vec2d>> obs;
    };

    struct Result {
        bool   converged           = false;
        int    iterations          = 0;
        double initial_residual_sq = 0.0;
        double final_residual_sq   = 0.0;
        int    huber_rejects       = 0;   // observations with weight < 1
        int    solve_us            = 0;   // wall-clock microseconds
    };

    // In-out: poses[].R_out / t_out and features[].p_w_out are written.
    // fx/fy/cx/cy are the pinhole intrinsics in pixel space.
    // Returns Result describing convergence; on degenerate inputs
    // (e.g. all features behind camera) returns converged=false and
    // copies inputs straight into the output fields.
    Result solve(std::vector<PoseObs>&    poses,
                 std::vector<FeatureObs>& features,
                 double fx, double fy, double cx, double cy,
                 int    max_iters       = 10,
                 double huber_thresh_px = 1.5);
};
