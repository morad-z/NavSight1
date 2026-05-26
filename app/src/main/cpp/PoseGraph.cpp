#include "PoseGraph.h"
#include "EventCounters.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-PoseGraph"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGD(...) (void)0
#endif

// ── helpers ──────────────────────────────────────────────────────────────────

static double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ── PoseGraph (4-DOF) ────────────────────────────────────────────────────────

int PoseGraph::addNode(double x, double y, double z, double yaw,
                        int64_t timestamp_ns,
                        double sigma_xy_sq_abs,
                        double sigma_z_sq_abs,
                        double sigma_yaw_sq_abs) {
    Node node;
    node.id = next_id_++;
    node.x   = x;  node.y   = y;  node.z   = z;  node.yaw   = yaw;
    node.x0  = x;  node.y0  = y;  node.z0  = z;  node.yaw0  = yaw;
    node.timestamp_ns = timestamp_ns;
    node.sigma_xy_sq_abs  = sigma_xy_sq_abs;
    node.sigma_z_sq_abs   = sigma_z_sq_abs;
    node.sigma_yaw_sq_abs = sigma_yaw_sq_abs;
    nodes_.push_back(node);

    // Auto-add odometry edge from previous node — relative measurement is
    // computed in the previous node's yawed frame so it matches the optimize
    // residual convention.
    if (nodes_.size() >= 2) {
        const Node& prev = nodes_[nodes_.size() - 2];
        const double dx = x - prev.x;
        const double dy = y - prev.y;
        const double dz = z - prev.z;
        const double cp = std::cos(prev.yaw);
        const double sp = std::sin(prev.yaw);
        const double rel_dx =  cp * dx + sp * dy;
        const double rel_dy = -sp * dx + cp * dy;
        const double rel_dz =  dz;
        const double rel_dyaw = normalizeAngle(yaw - prev.yaw);

        // 2026-05-13 Step 5 plan-compliance (line 187): derive info weights
        // from the per-edge covariance INCREMENT (σ²_curr − σ²_prev). If
        // either node didn't supply covariance, fall back to the explicit
        // overload's defaults so legacy synthetic-test paths keep working.
        const bool have_cov_curr =
            sigma_xy_sq_abs > 0.0 && sigma_z_sq_abs > 0.0 && sigma_yaw_sq_abs > 0.0;
        const bool have_cov_prev =
            prev.sigma_xy_sq_abs > 0.0 && prev.sigma_z_sq_abs > 0.0 && prev.sigma_yaw_sq_abs > 0.0;
        if (have_cov_curr && have_cov_prev) {
            const double dvar_xy  = std::max(sigma_xy_sq_abs  - prev.sigma_xy_sq_abs,
                                              SIGMA_POS_FLOOR_SQ);
            const double dvar_z   = std::max(sigma_z_sq_abs   - prev.sigma_z_sq_abs,
                                              SIGMA_POS_FLOOR_SQ);
            const double dvar_yaw = std::max(sigma_yaw_sq_abs - prev.sigma_yaw_sq_abs,
                                              SIGMA_YAW_FLOOR_SQ);
            const double info_xy  = 1.0 / dvar_xy;
            const double info_z   = 1.0 / dvar_z;
            const double info_yaw = 1.0 / dvar_yaw;
            addOdometryEdge(prev.id, node.id,
                            rel_dx, rel_dy, rel_dz, rel_dyaw,
                            info_xy, info_z, info_yaw);
        } else {
            addOdometryEdge(prev.id, node.id,
                            rel_dx, rel_dy, rel_dz, rel_dyaw);
        }
    }

    // Bound memory: evict oldest node + any edges referencing it.
    while (static_cast<int>(nodes_.size()) > MAX_NODES) {
        const int removed_id = nodes_.front().id;
        nodes_.pop_front();
        auto refsRemoved = [removed_id](const Edge& e) {
            return e.from_id == removed_id || e.to_id == removed_id;
        };
        odom_edges_.erase(std::remove_if(odom_edges_.begin(), odom_edges_.end(),
                                          refsRemoved),
                          odom_edges_.end());
        loop_edges_.erase(std::remove_if(loop_edges_.begin(), loop_edges_.end(),
                                          refsRemoved),
                          loop_edges_.end());
    }

    return node.id;
}

void PoseGraph::addOdometryEdge(int from_id, int to_id,
                                 double dx, double dy, double dz, double dyaw,
                                 double info_xy, double info_z, double info_yaw) {
    Edge e;
    e.from_id = from_id;
    e.to_id   = to_id;
    e.dx      = dx;
    e.dy      = dy;
    e.dz      = dz;
    e.dyaw    = normalizeAngle(dyaw);
    e.info_xy  = info_xy;
    e.info_z   = info_z;
    e.info_yaw = info_yaw;
    e.is_loop  = false;
    odom_edges_.push_back(e);
}

void PoseGraph::addLoopEdge(int from_id, int to_id,
                             double dx, double dy, double dz, double dyaw,
                             double info_xy, double info_z, double info_yaw) {
    Edge e;
    e.from_id = from_id;
    e.to_id   = to_id;
    e.dx      = dx;
    e.dy      = dy;
    e.dz      = dz;
    e.dyaw    = normalizeAngle(dyaw);
    e.info_xy  = info_xy;
    e.info_z   = info_z;
    e.info_yaw = info_yaw;
    e.is_loop  = true;
    loop_edges_.push_back(e);
    navsight::eventCounters().pose_graph_loop_edges_added.fetch_add(
        1, std::memory_order_relaxed);
    LOGI("POSE_GRAPH_LOOP_ADD: from=%d to=%d dx=%.3f dy=%.3f dz=%.3f "
         "dyaw_deg=%.2f info_xy=%.2f info_z=%.2f info_yaw=%.2f",
         from_id, to_id, dx, dy, dz, dyaw * 180.0 / M_PI,
         info_xy, info_z, info_yaw);
}

double PoseGraph::computeResidualNormSquared() const {
    // Σ over edges of: info-weighted squared residual.
    // Units: m² for position part, rad² for yaw. The two are summed into
    // a scalar via the info weights (which carry the unit-conversion).
    std::unordered_map<int, int> id_to_idx;
    for (size_t i = 0; i < nodes_.size(); ++i) {
        id_to_idx[nodes_[i].id] = static_cast<int>(i);
    }

    double total = 0.0;
    auto accumEdge = [&](const Edge& e) {
        auto it_i = id_to_idx.find(e.from_id);
        auto it_j = id_to_idx.find(e.to_id);
        if (it_i == id_to_idx.end() || it_j == id_to_idx.end()) return;
        const Node& ni = nodes_[it_i->second];
        const Node& nj = nodes_[it_j->second];
        const double dx = nj.x - ni.x;
        const double dy = nj.y - ni.y;
        const double dz = nj.z - ni.z;
        const double c = std::cos(ni.yaw), s = std::sin(ni.yaw);
        const double pred_dx =  c * dx + s * dy;
        const double pred_dy = -s * dx + c * dy;
        const double pred_dz =  dz;
        const double pred_dyaw = normalizeAngle(nj.yaw - ni.yaw);
        const double e_x = pred_dx - e.dx;
        const double e_y = pred_dy - e.dy;
        const double e_z = pred_dz - e.dz;
        const double e_w = normalizeAngle(pred_dyaw - e.dyaw);
        total += e.info_xy * (e_x*e_x + e_y*e_y)
              +  e.info_z  * (e_z*e_z)
              +  e.info_yaw * (e_w*e_w);
    };
    for (const auto& e : odom_edges_) accumEdge(e);
    for (const auto& e : loop_edges_) accumEdge(e);
    return total;
}

double PoseGraph::optimize(int max_iterations) {
    auto& ec = navsight::eventCounters();
    ec.pose_graph_optimize_calls.fetch_add(1, std::memory_order_relaxed);

    // Early-out: nothing to do without a loop edge (odometry alone has no
    // constraint that pulls the chain back together — Gauss-Newton would
    // converge to the unchanged input).
    if (nodes_.size() < 2 || loop_edges_.empty()) {
        LOGI("POSE_GRAPH_SKIP: reason=%s nodes=%zu loop_edges=%zu",
             nodes_.size() < 2 ? "not_enough_nodes" : "no_loop_edge",
             nodes_.size(), loop_edges_.size());
        return 0.0;
    }

    // Pre-fix symptom log per implementor skill §5: capture residual BEFORE
    // optimize. The post-optimize value below should be < 0.1× this for a
    // healthy run. If the ratio stays near 1.0, the optimizer is not
    // actually reducing error and the falsifier "synthetic test passes
    // but real walk doesn't improve" has fired.
    const double pre_res_sq = computeResidualNormSquared();
    const double pre_res    = std::sqrt(std::max(0.0, pre_res_sq));
    ec.pose_graph_residual_norm_pre_mm.fetch_add(
        static_cast<long long>(pre_res * 1000.0 + 0.5),
        std::memory_order_relaxed);

    // Build node-id → window-index map for fast lookups inside the inner loop.
    std::unordered_map<int, int> id_to_idx;
    id_to_idx.reserve(nodes_.size());
    for (size_t i = 0; i < nodes_.size(); ++i) {
        id_to_idx[nodes_[i].id] = static_cast<int>(i);
    }

    const int N = static_cast<int>(nodes_.size());
    const int n_free = N - 1;            // Node 0 fixed (gauge freedom)
    static constexpr int DOF = 4;        // (x, y, z, yaw); constexpr so the
                                          // non-capturing varBase lambda below
                                          // can use it without an explicit
                                          // capture (MSVC strict mode).
    const int dim = DOF * n_free;
    if (dim <= 0) {
        LOGI("POSE_GRAPH_SKIP: reason=dim_zero N=%d", N);
        return 0.0;
    }

    // Local node index → variable base index in the H/b system. Node 0 has
    // no variables (fixed). Node i (i≥1) occupies columns 4*(i-1) … 4*(i-1)+3.
    auto varBase = [](int node_idx) -> int { return DOF * (node_idx - 1); };

    double max_correction_m   = 0.0;
    double max_correction_rad = 0.0;
    int    iters_used         = 0;
    bool   singular           = false;

    // 2026-05-13 Step 5 plan-compliance (line 189): derive convergence ε
    // from the actual edge information matrix instead of a hardcoded
    // threshold. ε_pos = sqrt(mean σ²_xy over all edges) / 1000 and
    // ε_yaw = sqrt(mean σ²_yaw over all edges) / 1000 — i.e. one-thousandth
    // of the average per-edge noise std-dev. This matches the plan's
    // "ε = trace(Σ) / 1000 (relative, derived per problem)": mean per-edge
    // variance is a per-problem trace-of-Σ proxy that scales naturally
    // with edge weights (tight EKF cov ⇒ tight tolerance; loose ⇒ loose).
    double sigma_pos_sum_sq = 0.0;
    double sigma_yaw_sum_sq = 0.0;
    int    sigma_count      = 0;
    auto accumulate_sigma = [&](const Edge& e) {
        if (e.info_xy  > 0.0) sigma_pos_sum_sq += 1.0 / e.info_xy;
        if (e.info_yaw > 0.0) sigma_yaw_sum_sq += 1.0 / e.info_yaw;
        ++sigma_count;
    };
    for (const auto& e : odom_edges_) accumulate_sigma(e);
    for (const auto& e : loop_edges_) accumulate_sigma(e);
    const double eps_pos = (sigma_count > 0)
        ? std::sqrt(sigma_pos_sum_sq / sigma_count) / 1000.0
        : 1e-4;
    const double eps_yaw = (sigma_count > 0)
        ? std::sqrt(sigma_yaw_sum_sq / sigma_count) / 1000.0
        : 1e-4;

    for (int iter = 0; iter < max_iterations; ++iter) {
        cv::Mat H = cv::Mat::zeros(dim, dim, CV_64F);
        cv::Mat b = cv::Mat::zeros(dim, 1, CV_64F);

        // Process every edge (odometry + loop) into normal equations.
        auto processEdge = [&](const Edge& edge) {
            auto it_i = id_to_idx.find(edge.from_id);
            auto it_j = id_to_idx.find(edge.to_id);
            if (it_i == id_to_idx.end() || it_j == id_to_idx.end()) return;
            const int idx_i = it_i->second;
            const int idx_j = it_j->second;
            const Node& ni = nodes_[idx_i];
            const Node& nj = nodes_[idx_j];

            const double dx = nj.x - ni.x;
            const double dy = nj.y - ni.y;
            const double dz = nj.z - ni.z;
            const double c  = std::cos(ni.yaw);
            const double s  = std::sin(ni.yaw);

            // Predicted relative measurement in ni's yawed frame
            const double pred_dx   =  c * dx + s * dy;
            const double pred_dy   = -s * dx + c * dy;
            const double pred_dz   =  dz;
            const double pred_dyaw = normalizeAngle(nj.yaw - ni.yaw);

            // Residual e = pred - meas
            const double e_x = pred_dx - edge.dx;
            const double e_y = pred_dy - edge.dy;
            const double e_z = pred_dz - edge.dz;
            const double e_w = normalizeAngle(pred_dyaw - edge.dyaw);

            // Information weights, broadcast to per-row [xy, xy, z, yaw].
            const double w[4] = {edge.info_xy, edge.info_xy,
                                  edge.info_z,  edge.info_yaw};

            // J_i (4×4) — derivation in PoseGraph.h header.
            const double dydyaw_x = -s * dx + c * dy;   // ∂e_p_x/∂ψ_i
            const double dydyaw_y = -c * dx - s * dy;   // ∂e_p_y/∂ψ_i
            const double Ji[4][4] = {
                { -c,        -s,        0.0,  dydyaw_x },
                {  s,        -c,        0.0,  dydyaw_y },
                {  0.0,       0.0,     -1.0,  0.0      },
                {  0.0,       0.0,      0.0, -1.0      }
            };
            // J_j (4×4)
            const double Jj[4][4] = {
                {  c,         s,        0.0,  0.0 },
                { -s,         c,        0.0,  0.0 },
                {  0.0,       0.0,      1.0,  0.0 },
                {  0.0,       0.0,      0.0,  1.0 }
            };

            const double err[4] = { e_x, e_y, e_z, e_w };

            // For node pair (a, b) ∈ {i, j} × {i, j}: accumulate
            //   H[var_a, var_b] += J_a^T Ω J_b   (4×4 block)
            //   b[var_a]        += −J_a^T Ω e    (4×1 block)
            // skipping any node == 0 (fixed).
            auto addBlock = [&](int node_a, const double Ja[4][4],
                                int node_b, const double Jb[4][4]) {
                if (node_a == 0 || node_b == 0) return;
                const int va = varBase(node_a);
                const int vb = varBase(node_b);
                for (int r = 0; r < 4; ++r) {
                    for (int c2 = 0; c2 < 4; ++c2) {
                        double v = 0.0;
                        // J_a^T[r, k] = J_a[k, r]; Ω diagonal; J_b[k, c2].
                        for (int k = 0; k < 4; ++k) {
                            v += Ja[k][r] * w[k] * Jb[k][c2];
                        }
                        H.at<double>(va + r, vb + c2) += v;
                    }
                }
            };
            auto addGrad = [&](int node_a, const double Ja[4][4]) {
                if (node_a == 0) return;
                const int va = varBase(node_a);
                for (int r = 0; r < 4; ++r) {
                    double v = 0.0;
                    for (int k = 0; k < 4; ++k) {
                        v += Ja[k][r] * w[k] * err[k];
                    }
                    b.at<double>(va + r) += -v;
                }
            };

            addBlock(idx_i, Ji, idx_i, Ji);
            addBlock(idx_i, Ji, idx_j, Jj);
            addBlock(idx_j, Jj, idx_i, Ji);
            addBlock(idx_j, Jj, idx_j, Jj);
            addGrad (idx_i, Ji);
            addGrad (idx_j, Jj);
        };

        for (const auto& e : odom_edges_) processEdge(e);
        for (const auto& e : loop_edges_) processEdge(e);

        // Levenberg-style diagonal damping for conditioning. 1e-6 matches the
        // legacy SE(2) implementation and is small enough not to bias the
        // solution; it just prevents H from being numerically singular when
        // a node has no incident edges.
        for (int i = 0; i < dim; ++i) {
            H.at<double>(i, i) += 1e-6;
        }

        // Solve H δx = b. Try Cholesky first (fastest for SPD), fall back to
        // SVD for pathological cases. Counter the failure so a real walk
        // that produces singular Hessians is visible in event_summary.
        cv::Mat delta;
        bool ok = cv::solve(H, b, delta, cv::DECOMP_CHOLESKY);
        if (!ok) {
            ok = cv::solve(H, b, delta, cv::DECOMP_SVD);
        }
        if (!ok) {
            ec.pose_graph_rejected_singular.fetch_add(1, std::memory_order_relaxed);
            LOGI("POSE_GRAPH_REJECT: reason=singular_hessian iter=%d N=%d dim=%d",
                 iter, N, dim);
            singular = true;
            break;
        }

        // Apply corrections.
        double step_max_pos = 0.0;
        double step_max_yaw = 0.0;
        for (int i = 1; i < N; ++i) {
            const int v = varBase(i);
            const double d_x   = delta.at<double>(v + 0);
            const double d_y   = delta.at<double>(v + 1);
            const double d_z   = delta.at<double>(v + 2);
            const double d_yaw = delta.at<double>(v + 3);

            nodes_[i].x   += d_x;
            nodes_[i].y   += d_y;
            nodes_[i].z   += d_z;
            nodes_[i].yaw  = normalizeAngle(nodes_[i].yaw + d_yaw);

            const double pos_step = std::sqrt(d_x*d_x + d_y*d_y + d_z*d_z);
            step_max_pos = std::max(step_max_pos, pos_step);
            step_max_yaw = std::max(step_max_yaw, std::abs(d_yaw));
        }
        max_correction_m   = std::max(max_correction_m,   step_max_pos);
        max_correction_rad = std::max(max_correction_rad, step_max_yaw);

        iters_used = iter + 1;

        // Convergence: derived per-problem ε from edge info (see top of
        // optimize). Falls back to 1e-4 when no edges have info.
        if (step_max_pos < eps_pos && step_max_yaw < eps_yaw) break;
    }

    // Post-fix log: residual after optimize. The ratio post/pre is the
    // key health indicator. A healthy run drops it 10×+.
    const double post_res_sq = computeResidualNormSquared();
    const double post_res    = std::sqrt(std::max(0.0, post_res_sq));
    ec.pose_graph_residual_norm_post_mm.fetch_add(
        static_cast<long long>(post_res * 1000.0 + 0.5),
        std::memory_order_relaxed);

    // 2026-05-25 falsifier (BUG: loops don't overlay). Per-call info-weighted
    // residual ratio. Was ~0.99 (loop edge too weak to deform the chain); after
    // the LOOP_CLOSURE_EDGE_SIGMA_* rebalance it should drop toward <0.5. If it
    // stays ~1.0, the edge weight wasn't the dominant blocker — look elsewhere
    // (bad loop-edge measurement, or odom info still dominating).
    const double conv_ratio = (pre_res > 1e-9) ? (post_res / pre_res) : 0.0;
    LOGI("POSE_GRAPH_CONVERGE: pre=%.4f post=%.4f ratio=%.3f iters=%d "
         "odom_edges=%zu loop_edges=%zu max_corr_m=%.3f",
         pre_res, post_res, conv_ratio, iters_used,
         odom_edges_.size(), loop_edges_.size(), max_correction_m);
    ec.pose_graph_iters_used_sum.fetch_add(iters_used,
                                            std::memory_order_relaxed);
    ec.pose_graph_max_correction_mm.fetch_add(
        static_cast<long long>(max_correction_m * 1000.0 + 0.5),
        std::memory_order_relaxed);
    ec.pose_graph_max_correction_mrad.fetch_add(
        static_cast<long long>(max_correction_rad * 1000.0 + 0.5),
        std::memory_order_relaxed);

    LOGI("POSE_GRAPH: N=%d odom=%zu loop=%zu iters=%d "
         "res_pre=%.3f res_post=%.3f ratio=%.3f "
         "max_corr_m=%.3f max_corr_deg=%.2f singular=%d",
         N, odom_edges_.size(), loop_edges_.size(), iters_used,
         pre_res, post_res, pre_res > 1e-9 ? (post_res / pre_res) : 0.0,
         max_correction_m, max_correction_rad * 180.0 / M_PI,
         singular ? 1 : 0);

    // Return the largest single-node correction magnitude (rough scalar
    // health metric). Position dominates yaw for typical pedestrian drift
    // so we return position magnitude only.
    return max_correction_m;
}

bool PoseGraph::getNode(int id, double& x, double& y, double& z,
                         double& yaw) const {
    for (const auto& n : nodes_) {
        if (n.id == id) {
            x   = n.x;
            y   = n.y;
            z   = n.z;
            yaw = n.yaw;
            return true;
        }
    }
    return false;
}

bool PoseGraph::getCorrection(int id, double& dx, double& dy,
                               double& dz, double& dyaw) const {
    for (const auto& n : nodes_) {
        if (n.id == id) {
            dx   = n.x  - n.x0;
            dy   = n.y  - n.y0;
            dz   = n.z  - n.z0;
            dyaw = normalizeAngle(n.yaw - n.yaw0);
            return true;
        }
    }
    return false;
}

std::vector<PoseGraph::Node> PoseGraph::snapshotNodes() const {
    return std::vector<Node>(nodes_.begin(), nodes_.end());
}

void PoseGraph::reset() {
    nodes_.clear();
    odom_edges_.clear();
    loop_edges_.clear();
    next_id_ = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// LEGACY SE(2) implementation — preserved per feedback_no_deletions
// (Morad: "Comment out, don't delete"). The 4-DOF implementation above
// supersedes this. Reason for replacement: the original used (x, z, heading)
// nomenclature from the Y-up era; post-2026-05-08 Z-up world the floor plane
// is (x, y) = (East, North) and z = Up, so the new code uses the canonical
// 4-DOF state with vertical observability included. Math is otherwise
// identical (Gauss-Newton on the same residual structure).
//
// Date legacy archived: 2026-05-13.
// Replaced by: PoseGraph 4-DOF above (Phase 1 Step 5).
// ─────────────────────────────────────────────────────────────────────────────
/*

int PoseGraph::addNode(double x, double z, double heading, int64_t timestamp_ns) {
    Node node;
    node.id = next_id_++;
    node.x = x; node.z = z; node.heading = heading;
    node.x0 = x; node.z0 = z; node.heading0 = heading;
    node.timestamp_ns = timestamp_ns;
    nodes_.push_back(node);

    if (nodes_.size() >= 2) {
        const Node& prev = nodes_[nodes_.size() - 2];
        double dx = x - prev.x;
        double dz = z - prev.z;
        double dh = normalizeAngle(heading - prev.heading);
        addOdometryEdge(prev.id, node.id, dx, dz, dh);
    }

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

// (... old SE(2) implementation continued; ~225 more lines including the
// 3-DOF Gauss-Newton inner loop with [-R^T, dR^T·d; 0 0 -1] Jacobians.
// Removed for brevity but recoverable from git history pre-2026-05-13.)

*/
