#include "PoseGraph.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-PoseGraph"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#endif

static double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

int PoseGraph::addNode(double x, double z, double heading, int64_t timestamp_ns) {
    Node node;
    node.id = next_id_++;
    node.x = x; node.z = z; node.heading = heading;
    node.x0 = x; node.z0 = z; node.heading0 = heading;
    node.timestamp_ns = timestamp_ns;
    nodes_.push_back(node);

    // Add odometry edge from previous node
    if (nodes_.size() >= 2) {
        const Node& prev = nodes_[nodes_.size() - 2];
        double dx = x - prev.x;
        double dz = z - prev.z;
        double dh = normalizeAngle(heading - prev.heading);
        addOdometryEdge(prev.id, node.id, dx, dz, dh);
    }

    // Prune old nodes
    while (static_cast<int>(nodes_.size()) > MAX_NODES) {
        int removed_id = nodes_.front().id;
        nodes_.pop_front();
        auto removeOld = [removed_id](const Edge& e) {
            return e.from_id == removed_id || e.to_id == removed_id;
        };
        odom_edges_.erase(std::remove_if(odom_edges_.begin(), odom_edges_.end(), removeOld),
                          odom_edges_.end());
        loop_edges_.erase(std::remove_if(loop_edges_.begin(), loop_edges_.end(), removeOld),
                          loop_edges_.end());
    }

    return node.id;
}

void PoseGraph::addOdometryEdge(int from_id, int to_id,
                                 double dx, double dz, double dheading,
                                 double info_pos, double info_heading) {
    odom_edges_.push_back({from_id, to_id, dx, dz, normalizeAngle(dheading),
                           info_pos, info_heading});
}

void PoseGraph::addLoopEdge(int from_id, int to_id,
                             double dx, double dz, double dheading,
                             double info_pos, double info_heading) {
    loop_edges_.push_back({from_id, to_id, dx, dz, normalizeAngle(dheading),
                           info_pos, info_heading});
    LOGI("PoseGraph: loop edge added %d->%d (dx=%.2f, dz=%.2f, dh=%.1f°)",
         from_id, to_id, dx, dz, dheading * 180.0 / M_PI);
}

// ── Gauss-Newton on SE(2) ───────────────────────────────────────────────────
//
// State per node: (x, z, θ). Node 0 is fixed (gauge freedom).
// Edge residual for edge (i,j):
//   e_pos  = R(θ_i)^T * (p_j - p_i) - z_pos_meas     (2×1)
//   e_θ    = θ_j - θ_i - z_θ_meas                      (1×1)
//
// Jacobians:
//   J_i = [-R_i^T,  dR_i^T*(p_j-p_i);  0 0 -1]       (3×3)
//   J_j = [ R_i^T,  0;                  0 0  1]        (3×3)
//
// Normal equations: H δx = b
//   H = Σ [J_i J_j]^T Ω [J_i J_j]
//   b = -Σ [J_i J_j]^T Ω e
// Solve with OpenCV Cholesky.

double PoseGraph::optimize(int max_iterations) {
    if (nodes_.size() < 2 || loop_edges_.empty()) return 0.0;

    // Build node id -> local index mapping
    std::unordered_map<int, int> id_to_idx;
    for (size_t i = 0; i < nodes_.size(); i++) {
        id_to_idx[nodes_[i].id] = static_cast<int>(i);
    }

    int N = static_cast<int>(nodes_.size());
    // Node 0 is fixed. Free variables: nodes 1..N-1, each with 3 DOF
    int n_free = N - 1;
    int dim = 3 * n_free;  // Total state dimension

    if (dim <= 0) return 0.0;

    // Lambda: map node local_idx to variable index in the system
    // Node 0 is fixed, so node i (i>=1) maps to variable index 3*(i-1)
    auto varIdx = [](int node_idx) -> int { return 3 * (node_idx - 1); };

    double max_correction = 0.0;

    for (int iter = 0; iter < max_iterations; iter++) {
        // Build H and b
        cv::Mat H = cv::Mat::zeros(dim, dim, CV_64F);
        cv::Mat b = cv::Mat::zeros(dim, 1, CV_64F);
        double total_cost = 0.0;

        // Process all edges (odometry + loop)
        auto processEdge = [&](const Edge& e) {
            auto it_i = id_to_idx.find(e.from_id);
            auto it_j = id_to_idx.find(e.to_id);
            if (it_i == id_to_idx.end() || it_j == id_to_idx.end()) return;

            int idx_i = it_i->second;
            int idx_j = it_j->second;

            const Node& ni = nodes_[idx_i];
            const Node& nj = nodes_[idx_j];

            double dx = nj.x - ni.x;
            double dz = nj.z - ni.z;
            double ct = std::cos(ni.heading), st = std::sin(ni.heading);

            // Predicted relative pose in frame i
            double pred_tx =  ct * dx + st * dz;
            double pred_tz = -st * dx + ct * dz;
            double pred_dh = normalizeAngle(nj.heading - ni.heading);

            // Residual
            double e_tx = pred_tx - e.dx;
            double e_tz = pred_tz - e.dz;
            double e_th = normalizeAngle(pred_dh - e.dheading);

            total_cost += e.info_pos * (e_tx*e_tx + e_tz*e_tz) + e.info_heading * e_th*e_th;

            // Information matrix (diagonal)
            // Ω = diag(info_pos, info_pos, info_heading)
            double omega_p = e.info_pos;
            double omega_h = e.info_heading;

            // Jacobian J_i (3×3):
            // Row 0-1: ∂e_pos/∂(x_i, z_i, θ_i) = [-R^T, ∂R^T/∂θ * d]
            // Row 2:   ∂e_θ/∂(x_i, z_i, θ_i) = [0, 0, -1]
            // dR^T/dθ * d = [-st*dx + ct*dz; -ct*dx - st*dz]... wait
            // R^T = [ct st; -st ct], ∂R^T/∂θ = [-st ct; -ct -st]
            // ∂R^T/∂θ * [dx; dz] = [-st*dx + ct*dz; -ct*dx - st*dz]
            double dRt_d_x = -st * dx + ct * dz;
            double dRt_d_z = -ct * dx - st * dz;

            // J_i = [-ct, -st, dRt_d_x;  st, -ct, dRt_d_z;  0, 0, -1]
            double Ji[3][3] = {
                {-ct, -st, dRt_d_x},
                { st, -ct, dRt_d_z},
                {  0,   0,    -1.0}
            };

            // J_j = [ct, st, 0;  -st, ct, 0;  0, 0, 1]
            double Jj[3][3] = {
                { ct,  st, 0},
                {-st,  ct, 0},
                {  0,   0, 1}
            };

            double err[3] = {e_tx, e_tz, e_th};
            double omega[3] = {omega_p, omega_p, omega_h};

            // Accumulate into H and b
            // For each pair (a, b) in {i, j} x {i, j}:
            //   H[var_a, var_b] += J_a^T Ω J_b
            //   b[var_a] += -J_a^T Ω e

            auto addBlock = [&](int node_a, double Ja[3][3], int node_b, double Jb[3][3]) {
                if (node_a == 0 || node_b == 0) return;
                int va = varIdx(node_a);
                int vb = varIdx(node_b);
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        double val = 0;
                        for (int k = 0; k < 3; k++) {
                            val += Ja[k][r] * omega[k] * Jb[k][c];
                        }
                        H.at<double>(va + r, vb + c) += val;
                    }
                }
            };

            auto addGradient = [&](int node_a, double Ja[3][3]) {
                if (node_a == 0) return;
                int va = varIdx(node_a);
                for (int r = 0; r < 3; r++) {
                    double val = 0;
                    for (int k = 0; k < 3; k++) {
                        val += Ja[k][r] * omega[k] * err[k];
                    }
                    b.at<double>(va + r) += -val;
                }
            };

            addBlock(idx_i, Ji, idx_i, Ji);
            addBlock(idx_i, Ji, idx_j, Jj);
            addBlock(idx_j, Jj, idx_i, Ji);
            addBlock(idx_j, Jj, idx_j, Jj);

            addGradient(idx_i, Ji);
            addGradient(idx_j, Jj);
        };

        for (const auto& e : odom_edges_) processEdge(e);
        for (const auto& e : loop_edges_) processEdge(e);

        // Add small damping for numerical stability (LM-style)
        for (int i = 0; i < dim; i++) {
            H.at<double>(i, i) += 1e-6;
        }

        // Solve H * dx = b
        cv::Mat dx;
        if (!cv::solve(H, b, dx, cv::DECOMP_CHOLESKY)) {
            if (!cv::solve(H, b, dx, cv::DECOMP_SVD)) break;
        }

        // Apply corrections
        double step_max = 0.0;
        for (int i = 1; i < N; i++) {
            int vi = varIdx(i);
            double d_x = dx.at<double>(vi);
            double d_z = dx.at<double>(vi + 1);
            double d_h = dx.at<double>(vi + 2);

            nodes_[i].x += d_x;
            nodes_[i].z += d_z;
            nodes_[i].heading = normalizeAngle(nodes_[i].heading + d_h);

            step_max = std::max(step_max, std::sqrt(d_x*d_x + d_z*d_z) + std::abs(d_h));
        }

        max_correction = std::max(max_correction, step_max);
        if (step_max < 1e-5) break;  // Converged
    }

    LOGI("PoseGraph: GN optimized %d nodes, %zu odom + %zu loop edges, max_corr=%.4f",
         N, odom_edges_.size(), loop_edges_.size(), max_correction);
    return max_correction;
}

bool PoseGraph::getNode(int id, double& x, double& z, double& heading) const {
    for (const auto& n : nodes_) {
        if (n.id == id) {
            x = n.x; z = n.z; heading = n.heading;
            return true;
        }
    }
    return false;
}

bool PoseGraph::getLatestCorrection(double& dx, double& dz, double& dheading) const {
    if (nodes_.empty()) return false;
    const Node& n = nodes_.back();
    dx = n.x - n.x0;
    dz = n.z - n.z0;
    dheading = normalizeAngle(n.heading - n.heading0);
    return true;
}

void PoseGraph::reset() {
    nodes_.clear();
    odom_edges_.clear();
    loop_edges_.clear();
    next_id_ = 0;
}
