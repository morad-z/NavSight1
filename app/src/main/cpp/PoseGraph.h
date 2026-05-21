#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <opencv2/core.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// PoseGraph — 4-DOF (x, y, z, yaw) pose-graph optimizer for loop closure.
//
// Phase 1 Step 5 of post_v19_sprint_plan.md (rewritten 2026-05-13).
//
// PURPOSE
//   When DBoW2 detects a revisit and chi² accepts, the existing correction
//   path (Tracker.cpp:4220) only writes the EKF delta into global_t_'s
//   most recent point. Keyframes recorded between K_match and K_now keep
//   their drifted poses. Visually: loop 2's trajectory is offset from
//   loop 1's by the accumulated drift, except for the most recent few
//   seconds which got "snapped" by the LC ramp.
//
//   This module redistributes the loop-closure constraint across ALL
//   keyframes between the two visits via Gauss-Newton on a 4-DOF state
//   (x, y, z, yaw). Roll/pitch are observed by gravity-alignment (LC_GA),
//   so they're held fixed. Z (height) is included because indoor
//   pedestrian walks can include stair flights and Phase 2 scooter mode
//   will exercise inclined routes.
//
// CONVENTION (Z-up world frame post-2026-05-08)
//   x = East, y = North, z = Up
//   yaw = rotation around world-Z, range [-π, π]
//   Same convention as EKFState's R_GtoI extraction. See feedback_branch_workflow
//   memory + project_z_up_fix_2026_05_08.md for the frame migration.
//
// MATH (Gauss-Newton on a product of R³ × S¹)
//   State per node i: (x_i, y_i, z_i, ψ_i)
//   Node 0 is fixed (gauge freedom).
//
//   Edge residual for edge (i → j) with measurement (Δp_meas, Δψ_meas):
//     e_p = R_z(ψ_i)^T · (p_j − p_i) − Δp_meas      (3×1)
//     e_ψ = wrap(ψ_j − ψ_i − Δψ_meas)                (1×1)
//
//   where R_z(ψ) = [[cosψ, -sinψ, 0], [sinψ, cosψ, 0], [0, 0, 1]] is yaw
//   rotation about world-Z. The position residual is expressed in node i's
//   yawed frame (standard convention; reduces to SE(2) for flat ground).
//
//   Jacobian J_i (4×4) of [e_p; e_ψ] w.r.t. (x_i, y_i, z_i, ψ_i):
//     ∂e_p_x/∂x_i = −cosψ_i        ∂e_p_x/∂y_i = −sinψ_i      ∂e_p_x/∂z_i = 0
//     ∂e_p_y/∂x_i = +sinψ_i        ∂e_p_y/∂y_i = −cosψ_i      ∂e_p_y/∂z_i = 0
//     ∂e_p_z/∂x_i = 0              ∂e_p_z/∂y_i = 0            ∂e_p_z/∂z_i = −1
//     ∂e_p_x/∂ψ_i = −sinψ_i·dx + cosψ_i·dy   where (dx, dy, dz) = (p_j − p_i)
//     ∂e_p_y/∂ψ_i = −cosψ_i·dx − sinψ_i·dy
//     ∂e_p_z/∂ψ_i = 0
//     ∂e_ψ/∂ψ_i  = −1
//     ∂e_ψ/∂(x_i, y_i, z_i) = 0
//
//   Jacobian J_j (4×4) of [e_p; e_ψ] w.r.t. (x_j, y_j, z_j, ψ_j):
//     ∂e_p_x/∂x_j = +cosψ_i        ∂e_p_x/∂y_j = +sinψ_i      ∂e_p_x/∂z_j = 0
//     ∂e_p_y/∂x_j = −sinψ_i        ∂e_p_y/∂y_j = +cosψ_i      ∂e_p_y/∂z_j = 0
//     ∂e_p_z/∂x_j = 0              ∂e_p_z/∂y_j = 0            ∂e_p_z/∂z_j = +1
//     ∂e_p / ∂ψ_j = 0  (position residual is independent of ψ_j)
//     ∂e_ψ/∂ψ_j  = +1
//
//   Normal equations:  H · δx = b
//     H = Σ_edges [J_i J_j]^T Ω [J_i J_j]
//     b = −Σ_edges [J_i J_j]^T Ω e
//   where Ω = diag(info_pos, info_pos, info_pos_z, info_yaw) is the
//   information matrix (inverse-variance) for each edge.
//
// LEGACY SE(2) IMPLEMENTATION
//   The pre-2026-05-13 SE(2) version of this class is preserved as a
//   commented block in PoseGraph.cpp under "LEGACY SE(2) implementation".
//   Per feedback_no_deletions, we don't rm code — only comment.
//
// REFERENCES
//   * post_v19_sprint_plan.md §"Step 5 — Pose-graph back-end"
//   * Grisetti et al. 2010 "A Tutorial on Graph-Based SLAM"  (IEEE ITS Magazine)
//   * Kümmerle et al. 2011 "g2o: A General Framework for Graph Optimization" (ICRA)
//     — we use the same residual + Jacobian structure, OpenCV math instead of g2o.
// ─────────────────────────────────────────────────────────────────────────────

class PoseGraph {
public:
    // 4-DOF pose node. (x, y, z) in Z-up world meters, yaw in radians [-π, π].
    // 2026-05-13 Step 5 plan-compliance: per-clone absolute σ² values are
    // snapshotted here at addNode time so the auto-odom edge between this
    // node and the next can derive its information weight from the actual
    // covariance growth instead of a hardcoded default. Per plan line 187:
    // "Σ_odom from EKF clone covariance (per-edge real uncertainty)".
    struct Node {
        int id;
        double x, y, z, yaw;        // Current estimate (mutated by optimize())
        double x0, y0, z0, yaw0;    // Original snapshot (set on addNode, used by
                                    // getLatestCorrection for back-write delta)
        int64_t timestamp_ns;
        // Absolute EKF clone variances at addNode time. Default 0.0 → caller
        // didn't provide covariance, edge falls back to defaults.
        double sigma_xy_sq_abs{0.0};
        double sigma_z_sq_abs{0.0};
        double sigma_yaw_sq_abs{0.0};
    };

    // Edge measurement: relative pose (j w.r.t. i) in node i's yawed frame.
    // info_* are inverse-variance weights (larger = more trusted).
    struct Edge {
        int from_id, to_id;
        double dx, dy, dz, dyaw;
        double info_xy;     // 1/σ² for horizontal position residuals
        double info_z;      // 1/σ² for vertical position residual
        double info_yaw;    // 1/σ² for yaw residual
        bool   is_loop;     // true if this is a loop-closure edge (diagnostic)
    };

    PoseGraph() = default;

    // Add a new pose node. Auto-creates an odometry edge from the previous
    // node if there is one. Returns the assigned node id.
    //
    // 2026-05-13 Step 5 plan-compliance: if sigma_*_sq_abs values are
    // positive AND the previous node also has positive values, the
    // auto-edge's information weights are derived from the per-edge
    // covariance increment (σ²_curr − σ²_prev, floored at SIGMA_FLOOR_SQ
    // to keep info finite). When either side is zero (legacy callers /
    // first node), the edge falls back to the hardcoded defaults in the
    // explicit addOdometryEdge below.
    //
    //   sigma_xy_sq_abs   trace of P_[clone_pos[0..1], clone_pos[0..1]] —
    //                     horizontal-position variance in m²
    //   sigma_z_sq_abs    P_[clone_pos[2], clone_pos[2]] — z variance in m²
    //   sigma_yaw_sq_abs  variance of yaw (world-z rotation) in rad²
    int addNode(double x, double y, double z, double yaw,
                int64_t timestamp_ns,
                double sigma_xy_sq_abs   = 0.0,
                double sigma_z_sq_abs    = 0.0,
                double sigma_yaw_sq_abs  = 0.0);

    // Add an odometry edge (sequential constraint from tracking). Caller
    // supplies the relative measurement in node i's yawed frame. Default
    // info reflects "trust the per-step VIO odometry strongly".
    void addOdometryEdge(int from_id, int to_id,
                         double dx, double dy, double dz, double dyaw,
                         double info_xy   = 100.0,   // σ_xy = 10 cm
                         double info_z    = 100.0,   // σ_z  = 10 cm
                         double info_yaw  = 1000.0); // σ_yaw = ~1.8°

    // Add a loop-closure edge (non-sequential constraint from PnP at revisit).
    // Default info reflects "PnP is noisier than odometry but globally
    // referenced". info_xy = 4 → σ_xy = 50 cm corresponds to the
    // LOOP_CLOSURE_PNP_SIGMA_FLOOR_M = 2 m at full Mahalanobis weighting
    // halved (matches the damping ramp's effective weight on accept).
    void addLoopEdge(int from_id, int to_id,
                     double dx, double dy, double dz, double dyaw,
                     double info_xy   = 4.0,
                     double info_z    = 4.0,
                     double info_yaw  = 100.0);   // σ_yaw_loop ~5.7°

    // Run Gauss-Newton iterations. Returns the L∞ norm of the largest
    // per-node correction applied across iterations.
    //
    // Side effects (per implementor skill instrumentation requirements):
    //   * pose_graph_optimize_calls ← +1
    //   * pose_graph_iters_used_sum ← + (actual iters used)
    //   * pose_graph_residual_norm_pre_mm  ← + ||r||·1000 before first iter
    //   * pose_graph_residual_norm_post_mm ← + ||r||·1000 after last iter
    //   * pose_graph_max_correction_mm   ← + max(Δp)·1000
    //   * pose_graph_max_correction_mrad ← + max(Δψ)·1000
    //   * pose_graph_rejected_singular   ← +1 if Cholesky AND SVD both fail
    //   * LOGI("POSE_GRAPH: ... iters=K nodes=N loops=L pre=XX post=YY ...")
    double optimize(int max_iterations = 10);

    // Read the current (optimized) pose for a node.
    bool getNode(int id, double& x, double& y, double& z, double& yaw) const;

    // Read the delta between current and original pose for a node. Used by
    // Tracker to compute the back-write applied to global_t_ and the
    // LoopClosureDetector keyframe DB. Returns false if id not found.
    bool getCorrection(int id, double& dx, double& dy, double& dz, double& dyaw) const;

    // For Tracker post-optimize loop: iterate all nodes with their corrections.
    // Caller receives a copy under the internal mutex (no external lock needed).
    // Lightweight — typical graph is < 100 nodes, ≤ 32 bytes per Node entry.
    std::vector<Node> snapshotNodes() const;

    // Diagnostic.
    int  getNodeCount()      const { return static_cast<int>(nodes_.size()); }
    int  getOdomEdgeCount()  const { return static_cast<int>(odom_edges_.size()); }
    int  getLoopEdgeCount()  const { return static_cast<int>(loop_edges_.size()); }

    void reset();

    // Cap on stored nodes — bounds memory and Hessian dimension. 500 nodes
    // at typical keyframe cadence (1 KF / 0.5 s) ≈ 4 min of walk. Matches
    // legacy SE(2) limit; revisit when Phase 2 scooter mode arrives (longer
    // routes, faster motion → coarser KF cadence keeps the budget).
    static constexpr int MAX_NODES = 500;

    // Floor on per-edge variance (m² / rad²) when deriving info from
    // covariance increments. Increments can be 0 or negative (after an LC
    // accept the EKF cov shrinks); without a floor we'd divide by 0 and
    // 2026-05-21 BUG-NEW-PG ROOT-CAUSE FIX — raised floors by 3 orders of
    // magnitude (1mm² → 25cm², 1mrad² → 1°²).
    //
    // Cause: PoseGraph derives per-edge info_xy/info_yaw from the EKF's
    // ABSOLUTE covariance INCREMENT (σ²_curr − σ²_prev) at PoseGraph.cpp:66-71.
    // When the EKF runs MSCKF updates between keyframes, σ² often DECREASES
    // (more measurements → tighter covariance) → dvar gets clamped to the
    // SIGMA_POS_FLOOR_SQ = 1e-6 floor → info_xy = 1/1e-6 = **1,000,000**.
    // Meanwhile loop-edge info_xy from PnP is ~0.25 (σ_p ≈ 2 m, var ≈ 4 m²).
    // **4 million× ratio** means loop edges cannot deform the odom chain
    // regardless of how good the loop closure is.
    //
    // Walk evidence (bug02b_walk_2026_05_21.logcat.txt):
    //   POSE_GRAPH: N=110 odom=109 loop=4 iters=2 res_pre=2.614 res_post=2.614
    //   ratio=1.000 max_corr_m=0.004 — Gauss-Newton terminates in 2 iters at
    //   millimeter corrections because the chain is rigid.
    //
    // Threshold derivation (sensor-physics, not arbitrary statistical cuts):
    //   Visual VO position noise:  ~5 mm/frame at decent parallax
    //   × √(15 frames/keyframe at 30 Hz, 0.5 s spacing) ≈ 2 cm σ_xy per edge
    //   IMU process noise over 0.5 s: ~1 cm σ_xy (S21 Ultra Allan analysis)
    //   Combined RSS: ~2.5 cm σ_xy per edge
    //   Safety factor 2× → 5 cm floor → (0.05)² = 2.5e-3 m²
    //
    //   For yaw:
    //   Visual VO yaw: ~0.1°/frame × √15 ≈ 0.4° per edge
    //   Gyro integration over 0.5 s: ~0.1° (Madgwick noise floor)
    //   Combined RSS: ~0.5° per edge
    //   Safety factor 2× → 1° floor → (0.01745)² ≈ 3e-4 rad²
    //
    // Resulting max info_xy = 400 (was 1,000,000); loop info_xy still ~0.25
    // → ratio drops from 4M× to 1600×. Loop edges can now meaningfully
    // deform the chain over multiple iterations.
    //
    // Falsifier (post-fix walk):
    //   POSE_GRAPH log: iters > 2 on at least some solves
    //   POSE_GRAPH log: ratio < 0.9 (residual actually drops)
    //   max_corr_m > 0.05 (corrections at scale of real loop misalignment)
    //   loop_closure_corrections_applied still healthy (LC chain unchanged)
    //   Trajectory loop-bearing delta < 5° (was 11.6°)
    //
    // LEGACY: was 1e-6 each, kept for synthetic-test reference
    // /* legacy SIGMA_POS_FLOOR_SQ = 1e-6;   (1 mm)² */
    // /* legacy SIGMA_YAW_FLOOR_SQ = 1e-6;   (1 mrad)² */
    static constexpr double SIGMA_POS_FLOOR_SQ = 2.5e-3;  // (5 cm)² in m²
    static constexpr double SIGMA_YAW_FLOOR_SQ = 3.0e-4;  // (1°)² in rad²

private:
    // Helper: compute summed squared residual norm across all edges, in
    // metric units (meters for position part, radians for yaw scaled to a
    // common reference). Used pre/post optimize for the diagnostic counter.
    double computeResidualNormSquared() const;

    std::deque<Node> nodes_;
    std::vector<Edge> odom_edges_;
    std::vector<Edge> loop_edges_;
    int next_id_{0};
};
