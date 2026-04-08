#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <opencv2/core.hpp>

/**
 * @brief Pose graph optimizer for loop closure (Gauss-Newton on SE(2)).
 *
 * Stores 2D poses (x, z, heading) as nodes and relative constraints as edges.
 * Optimizes via proper Gauss-Newton: builds sparse Hessian H = J^T Ω J,
 * gradient b = -J^T Ω e, solves H δx = b with Cholesky.
 * Node 0 is fixed (gauge freedom). Replaces naive blend-based loop closure.
 *
 * SE(2) is correct for pedestrian floor-plane navigation; full SE(3) would
 * add elevation/pitch/roll DOFs that are not observable from our 2D heading model.
 */
class PoseGraph {
public:
    struct Node {
        int id;
        double x, z, heading;      // Current estimate
        double x0, z0, heading0;   // Original (before optimization)
        int64_t timestamp_ns;
    };

    struct Edge {
        int from_id, to_id;
        double dx, dz, dheading;   // Relative measurement
        double info_pos;            // Information (inverse variance) for position
        double info_heading;        // Information for heading
    };

    PoseGraph() = default;

    // Add a new pose node. Returns the node ID.
    int addNode(double x, double z, double heading, int64_t timestamp_ns);

    // Add an odometry edge (sequential constraint from tracking).
    void addOdometryEdge(int from_id, int to_id,
                         double dx, double dz, double dheading,
                         double info_pos = 1.0, double info_heading = 1.0);

    // Add a loop closure edge (non-sequential constraint from detection).
    void addLoopEdge(int from_id, int to_id,
                     double dx, double dz, double dheading,
                     double info_pos = 10.0, double info_heading = 5.0);

    // Run pose graph optimization (iterative Gauss-Newton on SE(2)).
    // Returns maximum node correction applied.
    double optimize(int max_iterations = 10);

    // Get optimized pose for a node.
    bool getNode(int id, double& x, double& z, double& heading) const;

    // Get the correction between original and optimized pose for latest node.
    // Used to propagate corrections to the Tracker.
    bool getLatestCorrection(double& dx, double& dz, double& dheading) const;

    // Get number of loop edges (diagnostic).
    int getLoopEdgeCount() const { return static_cast<int>(loop_edges_.size()); }

    void reset();

    static constexpr int MAX_NODES = 500;

private:
    std::deque<Node> nodes_;
    std::vector<Edge> odom_edges_;
    std::vector<Edge> loop_edges_;
    int next_id_{0};
};
