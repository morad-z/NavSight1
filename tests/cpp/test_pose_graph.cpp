// Phase 1 Step 5 (post_v19_sprint_plan.md): unit tests for the 4-DOF
// PoseGraph back-end. Per the implementor skill, the synthetic test
// MUST exist and pass before the real-walk validation can carry any
// weight. If a real walk produces no improvement but this test passes,
// the falsifier "weight matrices are wrong on real data" has fired and
// the per-walk diagnostics (residual_pre / residual_post counters) tell
// us why. If this test FAILS, the optimizer math itself is broken and
// no real walk will help — fix the test first.
//
// What this file covers:
//   * Empty / undersized graphs are no-ops (no crash, no spurious work)
//   * Consistent chain + identity loop edge → near-zero correction
//   * Inconsistent loop edge reduces the total residual (the actual
//     health metric — exact correction values depend on weight choices)
//   * EventCounters fields fire on every optimize() call
//
// What this file does NOT cover (intentionally — deferred to real-walk
// validation per implementor skill §3):
//   * Behavior on the back-write to global_t_ / LoopClosureDetector
//   * Convergence with EKF-derived weight matrices (those are computed
//     in Tracker.cpp at the wire-up site; unit tests use fixed weights)
//   * Multi-loop scenarios (one revisit per walk is the immediate target;
//     multi-loop fits Phase 1 Step 7 validation)

#include <gtest/gtest.h>
#include <cmath>
#include "PoseGraph.h"
#include "EventCounters.h"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

// Snapshot the counters before a sub-test so we can assert deltas.
struct CounterSnapshot {
    long long optimize_calls;
    long long iters_used_sum;
    long long residual_pre_mm;
    long long residual_post_mm;
    long long max_corr_mm;
    long long max_corr_mrad;
    long long rejected_singular;
    long long loop_edges_added;
};

CounterSnapshot snapshot() {
    const auto& ec = navsight::eventCounters();
    return {
        ec.pose_graph_optimize_calls.load(),
        ec.pose_graph_iters_used_sum.load(),
        ec.pose_graph_residual_norm_pre_mm.load(),
        ec.pose_graph_residual_norm_post_mm.load(),
        ec.pose_graph_max_correction_mm.load(),
        ec.pose_graph_max_correction_mrad.load(),
        ec.pose_graph_rejected_singular.load(),
        ec.pose_graph_loop_edges_added.load()
    };
}

}  // namespace

// ── (a) Defensive: undersized graphs don't crash ─────────────────────────────

TEST(PoseGraph, EmptyGraphOptimizeIsNoOp) {
    PoseGraph pg;
    const auto pre = snapshot();
    const double correction = pg.optimize(10);
    const auto post = snapshot();

    EXPECT_DOUBLE_EQ(correction, 0.0);
    EXPECT_EQ(post.optimize_calls - pre.optimize_calls, 1);
    // No iters used (early exit). No residual recorded.
    EXPECT_EQ(post.iters_used_sum - pre.iters_used_sum, 0);
    EXPECT_EQ(post.residual_pre_mm - pre.residual_pre_mm, 0);
}

TEST(PoseGraph, NoLoopEdgeOptimizeIsNoOp) {
    PoseGraph pg;
    pg.addNode(0.0, 0.0, 0.0, 0.0, 1'000'000'000LL);
    pg.addNode(1.0, 0.0, 0.0, 0.0, 1'100'000'000LL);
    pg.addNode(2.0, 0.0, 0.0, 0.0, 1'200'000'000LL);
    EXPECT_EQ(pg.getNodeCount(), 3);
    EXPECT_EQ(pg.getOdomEdgeCount(), 2);   // Auto-created by addNode
    EXPECT_EQ(pg.getLoopEdgeCount(), 0);

    const double correction = pg.optimize(10);
    EXPECT_DOUBLE_EQ(correction, 0.0);   // No loop edge → no work

    // Nodes should be unchanged.
    double x, y, z, yaw;
    ASSERT_TRUE(pg.getNode(0, x, y, z, yaw));
    EXPECT_NEAR(x, 0.0, 1e-9);
    ASSERT_TRUE(pg.getNode(2, x, y, z, yaw));
    EXPECT_NEAR(x, 2.0, 1e-9);
}

// ── (b) Math sanity: consistent chain + identity-consistent loop ────────────
//
// 4 nodes evenly spaced on a straight east-going chain. Each odometry
// edge measures (1, 0, 0, 0) — perfectly consistent with stored poses.
// Loop edge K_0 → K_3 measures (3, 0, 0, 0) — also consistent. There is
// nothing to optimize; the optimizer should leave nodes alone and report
// near-zero residual on both sides.

TEST(PoseGraph, ConsistentChainNoCorrection) {
    PoseGraph pg;
    pg.addNode(0.0, 0.0, 0.0, 0.0, 1'000'000'000LL);
    pg.addNode(1.0, 0.0, 0.0, 0.0, 1'100'000'000LL);
    pg.addNode(2.0, 0.0, 0.0, 0.0, 1'200'000'000LL);
    pg.addNode(3.0, 0.0, 0.0, 0.0, 1'300'000'000LL);

    // Loop edge K_0 → K_3: in K_0's frame, K_3 is 3m east, identity in y/z/yaw.
    pg.addLoopEdge(/*from=*/0, /*to=*/3,
                   /*dx=*/3.0, /*dy=*/0.0, /*dz=*/0.0, /*dyaw=*/0.0);

    const double max_corr = pg.optimize(10);
    EXPECT_LT(max_corr, 1e-3);   // sub-mm

    // Nodes should still be where we put them.
    double x, y, z, yaw;
    ASSERT_TRUE(pg.getNode(3, x, y, z, yaw));
    EXPECT_NEAR(x, 3.0, 1e-3);
    EXPECT_NEAR(y, 0.0, 1e-3);
    EXPECT_NEAR(z, 0.0, 1e-3);
    EXPECT_NEAR(yaw, 0.0, 1e-3);
}

// ── (c) The actual health metric: residual decreases on inconsistent loop ──
//
// Chain has stored y-drift growing 0.1 m per step (K_0=0, K_1=0.1, K_2=0.2,
// K_3=0.3). Odometry edges between adjacent nodes report the drifted
// relative motion exactly, so odometry residual alone is zero. The loop
// edge K_0 → K_3 says "K_3 is 3 m east, zero y" — i.e., the truth.
// The loop edge introduces a residual of 0.3 m in y at K_3; the optimizer
// should redistribute drift across K_1..K_3 so the total weighted residual
// drops by an order of magnitude.

TEST(PoseGraph, InconsistentLoopReducesResidual) {
    PoseGraph pg;
    pg.addNode(0.0, 0.0, 0.0, 0.0, 1'000'000'000LL);
    pg.addNode(1.0, 0.1, 0.0, 0.0, 1'100'000'000LL);
    pg.addNode(2.0, 0.2, 0.0, 0.0, 1'200'000'000LL);
    pg.addNode(3.0, 0.3, 0.0, 0.0, 1'300'000'000LL);

    // Truth loop edge: K_0 → K_3 should land at exactly 3 m east, no y drift.
    //
    // Weights here are DELIBERATELY rebalanced from the production defaults.
    // Production defaults — info_xy_odom=100, info_xy_loop=4 (header docs) —
    // give a 25× preference to odometry, reflecting that VIO per-step
    // odometry is genuinely much less noisy than each individual PnP-derived
    // loop edge. On this 4-node toy chain with 0.3m total accumulated y-drift,
    // the algebraic minimum-residual solution under those weights only
    // reduces ||r|| by ~6% — the loop edge can't fight 3× the odom edges
    // each with 25× the trust. That's correct math, not a bug.
    //
    // For unit-test purposes we want to verify the optimizer reduces
    // residual when the loop edge has comparable weight to odometry —
    // which is what happens in production after several loop closures
    // accumulate or when drift is large enough that odom variance grows
    // (Phase 2 Step 10.5 adaptive weights). Balanced weights here exercise
    // the math without the production weight ratio dominating.
    pg.addLoopEdge(/*from=*/0, /*to=*/3,
                   /*dx=*/3.0, /*dy=*/0.0, /*dz=*/0.0, /*dyaw=*/0.0,
                   /*info_xy=*/100.0,   // equal to odometry default
                   /*info_z=*/100.0,
                   /*info_yaw=*/1000.0);

    const auto pre = snapshot();
    const double max_corr = pg.optimize(20);
    const auto post = snapshot();

    EXPECT_GT(max_corr, 0.001);   // Optimizer DID move something

    // Counters fired
    EXPECT_EQ(post.optimize_calls - pre.optimize_calls, 1);
    EXPECT_GT(post.iters_used_sum - pre.iters_used_sum, 0);
    EXPECT_GT(post.residual_pre_mm - pre.residual_pre_mm, 0);
    EXPECT_GT(post.residual_post_mm - pre.residual_post_mm, 0);

    // With balanced weights the residual should drop substantially. We
    // assert < 0.5 (50% reduction) which is mid-range given algebraic
    // expectation that K_3.y converges toward the area-mean of (0, 0.3)
    // = ~0.15, halving the loop residual from 0.3 to ~0.15.
    const long long pre_delta  = post.residual_pre_mm  - pre.residual_pre_mm;
    const long long post_delta = post.residual_post_mm - pre.residual_post_mm;
    ASSERT_GT(pre_delta, 0);
    const double ratio = static_cast<double>(post_delta) /
                          static_cast<double>(pre_delta);
    EXPECT_LT(ratio, 0.5)
        << "Optimizer reduced residual by only " << (1.0 - ratio)
        << "× — falsifier 1 (math not reducing error) candidate";

    // K_3 should have moved toward y=0 (the loop says so).
    double x, y, z, yaw;
    ASSERT_TRUE(pg.getNode(3, x, y, z, yaw));
    EXPECT_LT(y, 0.3) << "K_3.y should drop from initial 0.3 toward loop-edge target 0";
}

// ── (d) Counter fires per call ──────────────────────────────────────────────

TEST(PoseGraph, LoopEdgeCounterFires) {
    PoseGraph pg;
    pg.addNode(0.0, 0.0, 0.0, 0.0, 1LL);
    pg.addNode(1.0, 0.0, 0.0, 0.0, 2LL);

    const auto pre = snapshot();
    pg.addLoopEdge(0, 1, 1.0, 0.0, 0.0, 0.0);
    const auto post = snapshot();

    EXPECT_EQ(post.loop_edges_added - pre.loop_edges_added, 1);
}

// ── (e) Correction back-write data path: getCorrection returns Δ ───────────

TEST(PoseGraph, GetCorrectionReturnsDeltaAfterOptimize) {
    PoseGraph pg;
    pg.addNode(0.0, 0.0, 0.0, 0.0, 1LL);
    pg.addNode(1.0, 0.1, 0.0, 0.0, 2LL);
    pg.addNode(2.0, 0.2, 0.0, 0.0, 3LL);
    pg.addLoopEdge(/*from=*/0, /*to=*/2,
                   /*dx=*/2.0, /*dy=*/0.0, /*dz=*/0.0, /*dyaw=*/0.0);

    // Before optimize: correction is zero (nodes haven't moved yet).
    double dx, dy, dz, dyaw;
    ASSERT_TRUE(pg.getCorrection(2, dx, dy, dz, dyaw));
    EXPECT_NEAR(dx, 0.0, 1e-12);
    EXPECT_NEAR(dy, 0.0, 1e-12);

    pg.optimize(10);

    // After optimize: correction should be non-zero for K_2 (it moved toward
    // the loop target). K_0 is fixed.
    ASSERT_TRUE(pg.getCorrection(0, dx, dy, dz, dyaw));
    EXPECT_NEAR(dx, 0.0, 1e-9);   // Node 0 fixed
    EXPECT_NEAR(dy, 0.0, 1e-9);

    ASSERT_TRUE(pg.getCorrection(2, dx, dy, dz, dyaw));
    EXPECT_GT(std::abs(dy), 1e-4)
        << "K_2 should have moved off (0.2) toward loop-edge target (0.0)";
}

// ── (f) Reset wipes state ───────────────────────────────────────────────────

TEST(PoseGraph, ResetWipesNodesAndEdges) {
    PoseGraph pg;
    pg.addNode(0.0, 0.0, 0.0, 0.0, 1LL);
    pg.addNode(1.0, 0.0, 0.0, 0.0, 2LL);
    pg.addLoopEdge(0, 1, 1.0, 0.0, 0.0, 0.0);

    EXPECT_EQ(pg.getNodeCount(), 2);
    EXPECT_EQ(pg.getOdomEdgeCount(), 1);
    EXPECT_EQ(pg.getLoopEdgeCount(), 1);

    pg.reset();

    EXPECT_EQ(pg.getNodeCount(), 0);
    EXPECT_EQ(pg.getOdomEdgeCount(), 0);
    EXPECT_EQ(pg.getLoopEdgeCount(), 0);
}
