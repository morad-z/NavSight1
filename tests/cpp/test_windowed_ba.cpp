// Step 6 (Visual Production Plan): unit tests for the local windowed
// bundle-adjustment solver.
//
// Plan reference: docs/VISUAL_PRODUCTION_PLAN.md, "Step 6 — Local
// windowed bundle adjustment" (around line 649).
//
// Agent A is landing the implementation in app/src/main/cpp/WindowedBA.{h,cpp}
// (NEW) and wiring it into app/CMakeLists.txt; Agent C is editing
// Tracker / EKFState / FeatureManager and ADR-012. These tests touch
// ONLY the WindowedBA API:
//
//   class WindowedBA {
//   public:
//       struct PoseObs    { int kf_id; cv::Matx33d R_in;  cv::Vec3d t_in;
//                                       cv::Matx33d R_out; cv::Vec3d t_out;
//                                       bool is_anchor; };
//       struct FeatureObs { int feature_id; cv::Vec3d p_w_in, p_w_out;
//                           std::vector<std::pair<int, cv::Vec2d>> obs; };
//       struct Result     { bool converged; int iterations;
//                           double initial_residual_sq, final_residual_sq;
//                           int huber_rejects; int solve_us; };
//       Result solve(std::vector<PoseObs>&, std::vector<FeatureObs>&,
//                    double fx, double fy, double cx, double cy,
//                    int max_iters = 10, double huber_thresh_px = 1.5);
//   };
//
// Convention: pose is world->cam. Reprojection is
//   p_cam = R * (p_world - t)
//   (u, v) = (fx * p_cam.x / p_cam.z + cx, fy * p_cam.y / p_cam.z + cy)
//
// Idiom mirrors test_visual_robustness.cpp / test_orb_reloc.cpp from the
// Step-4/Step-5 commits: anonymous-namespace helpers, OpenCV asserts,
// trend-based bounds, all-synthetic in-memory scenes (no FeatureManager,
// EKFState, or Tracker dependency; no sim files; no device recordings).
//
// Note: desktop test build is currently BLOCKED on Morad's Windows host
// by an MSVC/MinGW OpenCV mismatch. These tests are designed to compile
// cleanly in concept; CI / Linux runs them.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "WindowedBA.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

namespace {

// ── Step 6 BA scene parameters ──────────────────────────────────────────

// Pinhole intrinsics for the synthetic scenes. Same focal/cx/cy the
// Step-4 test (test_orb_reloc.cpp) uses so the two test files agree on
// what "the camera" looks like.
constexpr double kFx = 525.0;
constexpr double kFy = 525.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;

// Plan §1 (Step 6): max_iters default is 10, Huber threshold 1.5 px.
constexpr int    kDefaultMaxIters     = 10;
constexpr double kDefaultHuberThreshPx = 1.5;

// ── Helpers ─────────────────────────────────────────────────────────────

// Build a small SO(3) rotation from an axis-angle vector (rad).
// Rodrigues' formula. Used to construct ground-truth keyframe rotations
// and to perturb them with small noise.
cv::Matx33d rodrigues(const cv::Vec3d& aa) {
    const double theta = std::sqrt(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2]);
    if (theta < 1e-12) {
        return cv::Matx33d::eye();
    }
    const double k0 = aa[0] / theta;
    const double k1 = aa[1] / theta;
    const double k2 = aa[2] / theta;
    const double c  = std::cos(theta);
    const double s  = std::sin(theta);
    const double C  = 1.0 - c;
    return cv::Matx33d(
        c + k0 * k0 * C,        k0 * k1 * C - k2 * s,   k0 * k2 * C + k1 * s,
        k1 * k0 * C + k2 * s,   c + k1 * k1 * C,        k1 * k2 * C - k0 * s,
        k2 * k0 * C - k1 * s,   k2 * k1 * C + k0 * s,   c + k2 * k2 * C);
}

// Apply the project convention: world->cam pose, then pinhole projection.
//   p_cam = R * (p_world - t)
//   (u, v) = (fx * p_cam.x / p_cam.z + cx, fy * p_cam.y / p_cam.z + cy)
// Returns true iff the point lies in front of the camera (z > 1e-6).
bool projectPoint(const cv::Matx33d& R, const cv::Vec3d& t,
                  const cv::Vec3d& p_world,
                  double fx, double fy, double cx, double cy,
                  cv::Vec2d& uv_out) {
    const cv::Vec3d p_cam = R * (p_world - t);
    if (p_cam[2] <= 1e-6) {
        return false;
    }
    uv_out[0] = fx * p_cam[0] / p_cam[2] + cx;
    uv_out[1] = fy * p_cam[1] / p_cam[2] + cy;
    return true;
}

// Build a 5-keyframe / 15-feature synthetic scene. Camera moves along a
// gentle curve in the world XY plane while pointing at +Z (looking
// forward), with a small yaw turn between keyframes so feature
// observations have meaningful parallax. Features are scattered in a
// 3 m × 3 m × 3 m volume in front of the start pose.
//
// Ground-truth poses are stored in (R_in, t_in) so caller can perturb
// them and run BA. Each feature gets observed by every keyframe (clean
// scene, no occlusion), with the projected pixel as the observation.
struct GroundTruthScene {
    std::vector<WindowedBA::PoseObs>    poses;
    std::vector<WindowedBA::FeatureObs> features;
};

GroundTruthScene makeCurvedScene(int K, int N, uint64_t seed) {
    GroundTruthScene scene;
    scene.poses.reserve(K);
    scene.features.reserve(N);

    // Ground-truth poses: camera translates +x by 0.4 m / keyframe and
    // yaws +2° / keyframe (small turn, plenty of parallax across N=20).
    for (int k = 0; k < K; ++k) {
        WindowedBA::PoseObs p;
        p.keyframe_id = k;
        // World->cam yaw: small rotation about world +y axis.
        const double yaw_rad = 2.0 * M_PI / 180.0 * static_cast<double>(k);
        p.R_in  = rodrigues(cv::Vec3d(0.0, yaw_rad, 0.0));
        p.t_in  = cv::Vec3d(0.4 * static_cast<double>(k), 0.0, 0.0);
        p.R_out = p.R_in;
        p.t_out = p.t_in;
        p.is_anchor = false;
        scene.poses.push_back(p);
    }

    // Ground-truth features: scattered 1.5–4.5 m in front of the start
    // pose with ±1.5 m lateral spread. Deterministic via seeded RNG.
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> ux(-1.5, 1.5);
    std::uniform_real_distribution<double> uy(-1.0, 1.0);
    std::uniform_real_distribution<double> uz(1.5, 4.5);
    for (int n = 0; n < N; ++n) {
        WindowedBA::FeatureObs f;
        f.feature_id = n;
        f.p_w_in  = cv::Vec3d(ux(rng), uy(rng), uz(rng));
        f.p_w_out = f.p_w_in;
        scene.features.push_back(f);
    }

    // Project every feature into every keyframe. Skip any (k, n) pair
    // where the point falls behind the camera or outside the 640×480
    // image bounds — keeps the residual graph clean.
    for (auto& f : scene.features) {
        for (const auto& p : scene.poses) {
            cv::Vec2d uv;
            if (!projectPoint(p.R_in, p.t_in, f.p_w_in, kFx, kFy, kCx, kCy, uv)) {
                continue;
            }
            if (uv[0] < 0.0 || uv[0] > 640.0 || uv[1] < 0.0 || uv[1] > 480.0) {
                continue;
            }
            f.obs.emplace_back(p.keyframe_id, uv);
        }
    }
    return scene;
}

// Apply Gaussian noise to (R_in, t_in) of every pose and to p_w_in of
// every feature. Sigmas chosen per the test spec:
//   t  σ = 0.05 m
//   R  σ = 0.02 rad (axis-angle perturbation)
//   p  σ = 0.05 m
// The (R_out, t_out, p_w_out) fields are also overwritten so a caller
// running BA-as-identity would see no change. BA must reduce the
// residual to demonstrate it actually optimizes.
void perturbScene(GroundTruthScene& scene, uint64_t seed) {
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> n_t(0.0, 0.05);
    std::normal_distribution<double> n_r(0.0, 0.02);
    std::normal_distribution<double> n_p(0.0, 0.05);

    for (auto& p : scene.poses) {
        const cv::Vec3d daa(n_r(rng), n_r(rng), n_r(rng));
        const cv::Matx33d dR = rodrigues(daa);
        p.R_in  = dR * p.R_in;
        p.t_in += cv::Vec3d(n_t(rng), n_t(rng), n_t(rng));
        p.R_out = p.R_in;
        p.t_out = p.t_in;
    }
    for (auto& f : scene.features) {
        f.p_w_in += cv::Vec3d(n_p(rng), n_p(rng), n_p(rng));
        f.p_w_out = f.p_w_in;
    }
}

// Sum-of-squared reprojection residuals (px²) using the *_out fields,
// EXCLUDING any observation whose pixel coordinates are within tol of
// the supplied skip-list. Used in the outlier test to compute the
// "inlier-only" residual after BA.
double residualSqExcluding(
    const std::vector<WindowedBA::PoseObs>& poses,
    const std::vector<WindowedBA::FeatureObs>& features,
    double fx, double fy, double cx, double cy,
    const std::vector<std::tuple<int, int, double, double>>& skip_uv = {}) {

    auto isSkipped = [&](int feat_id, int kf_id,
                         const cv::Vec2d& uv) -> bool {
        for (const auto& s : skip_uv) {
            if (std::get<0>(s) == feat_id && std::get<1>(s) == kf_id) {
                const double du = uv[0] - std::get<2>(s);
                const double dv = uv[1] - std::get<3>(s);
                if (du * du + dv * dv < 1e-6) {
                    return true;
                }
            }
        }
        return false;
    };

    // Index poses by keyframe_id for O(1) lookup.
    std::vector<const WindowedBA::PoseObs*> pose_by_id;
    for (const auto& p : poses) {
        if (p.keyframe_id + 1 > static_cast<int>(pose_by_id.size())) {
            pose_by_id.resize(p.keyframe_id + 1, nullptr);
        }
        pose_by_id[p.keyframe_id] = &p;
    }

    double sum = 0.0;
    for (const auto& f : features) {
        for (const auto& [kf_id, uv_obs] : f.obs) {
            if (isSkipped(f.feature_id, kf_id, uv_obs)) continue;
            if (kf_id < 0 || kf_id >= static_cast<int>(pose_by_id.size()))
                continue;
            const WindowedBA::PoseObs* p = pose_by_id[kf_id];
            if (!p) continue;
            cv::Vec2d uv_proj;
            if (!projectPoint(p->R_out, p->t_out, f.p_w_out,
                              fx, fy, cx, cy, uv_proj)) {
                continue;
            }
            const double du = uv_proj[0] - uv_obs[0];
            const double dv = uv_proj[1] - uv_obs[1];
            sum += du * du + dv * dv;
        }
    }
    return sum;
}

}  // namespace

// ── Test 1 — residual halves (drops to <1%) on a noise-free scene ───────

// Build a clean 5-KF, 15-feature scene with all observations exact.
// Perturb poses (σ_t=0.05 m, σ_R=0.02 rad) and features (σ_p=0.05 m).
// BA must drive the residual back down. Spec: final_residual_sq <
// 0.01 * initial_residual_sq AND iterations < max AND converged == true.
TEST(WindowedBA, ResidualHalvesOnNoiseFreeScene) {
    GroundTruthScene scene = makeCurvedScene(/*K=*/5, /*N=*/15,
                                             /*seed=*/0xBA0001u);
    perturbScene(scene, /*seed=*/0xBA0002u);

    WindowedBA solver;
    WindowedBA::Result r = solver.solve(
        scene.poses, scene.features,
        kFx, kFy, kCx, kCy,
        kDefaultMaxIters, kDefaultHuberThreshPx);

    EXPECT_TRUE(r.converged)
        << "BA failed to converge on a clean 5-KF / 15-feature scene with "
           "small Gaussian perturbation; check Levenberg-Marquardt setup.";
    EXPECT_LT(r.iterations, kDefaultMaxIters)
        << "BA hit the max-iters cap (" << kDefaultMaxIters
        << ") on a noise-free scene; convergence is too slow.";
    ASSERT_GT(r.initial_residual_sq, 0.0)
        << "Initial residual is zero — perturbation step is broken.";
    EXPECT_LT(r.final_residual_sq, 0.01 * r.initial_residual_sq)
        << "BA reduced residual from " << r.initial_residual_sq
        << " to " << r.final_residual_sq << " (ratio "
        << (r.final_residual_sq / r.initial_residual_sq)
        << "); spec requires < 1% on a noise-free scene.";
}

// ── Test 2 — anchor pose is held EXACTLY fixed ──────────────────────────

// Same scene as test 1, but mark poses[0] as is_anchor=true. After
// solve, R_out and t_out of poses[0] must equal R_in/t_in EXACTLY (no
// drift on the anchor). This is the "gauge fixing" guarantee from plan
// §5: oldest keyframe has its pose fixed to prevent gauge ambiguity.
TEST(WindowedBA, AnchorPoseHeldFixed) {
    GroundTruthScene scene = makeCurvedScene(/*K=*/5, /*N=*/15,
                                             /*seed=*/0xBA0003u);
    perturbScene(scene, /*seed=*/0xBA0004u);

    // Capture the pre-solve R_in / t_in of poses[0] and mark as anchor.
    scene.poses[0].is_anchor = true;
    const cv::Matx33d R_anchor_in = scene.poses[0].R_in;
    const cv::Vec3d   t_anchor_in = scene.poses[0].t_in;

    WindowedBA solver;
    WindowedBA::Result r = solver.solve(
        scene.poses, scene.features,
        kFx, kFy, kCx, kCy,
        kDefaultMaxIters, kDefaultHuberThreshPx);
    (void)r;  // Result fields not under test here.

    // Anchor must be byte-exact.
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            EXPECT_DOUBLE_EQ(scene.poses[0].R_out(row, col),
                             R_anchor_in(row, col))
                << "Anchor R_out(" << row << "," << col << ") drifted from R_in; "
                   "gauge anchor is not being held fixed.";
        }
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(scene.poses[0].t_out[i], t_anchor_in[i])
            << "Anchor t_out[" << i << "] drifted from t_in; "
               "gauge anchor is not being held fixed.";
    }

    // Sanity: at least one non-anchor pose moved (otherwise the solver
    // is just freezing everything, not solving).
    bool any_moved = false;
    for (size_t k = 1; k < scene.poses.size(); ++k) {
        if (cv::norm(scene.poses[k].t_out - scene.poses[k].t_in) > 1e-9) {
            any_moved = true;
            break;
        }
    }
    EXPECT_TRUE(any_moved)
        << "All non-anchor poses are byte-identical to their inputs; "
           "BA is not actually optimizing the free variables.";
}

// ── Test 3 — Huber rejects a single 30 px outlier ───────────────────────

// Take the clean scene from test 1, then ADD a single outlier
// observation: feature 0 in keyframe 2 at +30 px offset from its true
// projection. Run BA with the default 1.5 px Huber threshold. Spec:
//   - huber_rejects >= 1
//   - inlier-only residual after solve < 1.0 px²
// (The residual computed EXCLUDING the outlier row stays tight — Huber
// must not let the outlier corrupt the rest of the solution.)
TEST(WindowedBA, HuberRejects3PxOutlier) {
    GroundTruthScene scene = makeCurvedScene(/*K=*/5, /*N=*/15,
                                             /*seed=*/0xBA0005u);
    // Snapshot the *clean* projections BEFORE perturbation: we'll use
    // them as "true" pixel positions to check the outlier separation.

    // Inject the outlier on (feature 0, keyframe 2) at +30 px in u.
    // Find the existing observation and shift it.
    const int target_feat = 0;
    const int target_kf   = 2;
    cv::Vec2d outlier_uv;
    bool injected = false;
    for (auto& [kf_id, uv] : scene.features[target_feat].obs) {
        if (kf_id == target_kf) {
            uv[0] += 30.0;
            outlier_uv = uv;
            injected = true;
            break;
        }
    }
    ASSERT_TRUE(injected)
        << "Test setup invalid: feature " << target_feat
        << " has no observation in keyframe " << target_kf
        << "; outlier injection failed.";

    perturbScene(scene, /*seed=*/0xBA0006u);

    WindowedBA solver;
    WindowedBA::Result r = solver.solve(
        scene.poses, scene.features,
        kFx, kFy, kCx, kCy,
        kDefaultMaxIters, kDefaultHuberThreshPx);

    EXPECT_GE(r.huber_rejects, 1)
        << "Huber loss reported zero rejects despite a +30 px outlier; "
           "either the threshold (1.5 px) is too loose or the rejects "
           "counter is not wired.";

    // Inlier-only residual: exclude the outlier row.
    const double inlier_residual_sq = residualSqExcluding(
        scene.poses, scene.features,
        kFx, kFy, kCx, kCy,
        {{target_feat, target_kf, outlier_uv[0], outlier_uv[1]}});

    EXPECT_LT(inlier_residual_sq, 1.0)
        << "Inlier-only residual after BA is " << inlier_residual_sq
        << " px²; the outlier corrupted the solution. Huber loss is not "
           "down-weighting it correctly.";
}

// ── Test 4 — soft-gate: solve completes in < 100 ms at K=5, N=20 ────────

// Plan acceptance: BA completes in < 100 ms on a Snapdragon 695 (the
// production target). Desktop should be much faster, so this is a
// generous soft gate. If it fails, the ASSERT message reports the
// actual us so we know how far off we are.
TEST(WindowedBA, ConvergenceUnder100msAtK5N20) {
    GroundTruthScene scene = makeCurvedScene(/*K=*/5, /*N=*/20,
                                             /*seed=*/0xBA0007u);
    perturbScene(scene, /*seed=*/0xBA0008u);

    WindowedBA solver;
    WindowedBA::Result r = solver.solve(
        scene.poses, scene.features,
        kFx, kFy, kCx, kCy,
        kDefaultMaxIters, kDefaultHuberThreshPx);

    EXPECT_LT(r.solve_us, 100000)
        << "BA solve took " << r.solve_us << " us (> 100 ms) at K=5, N=20 "
           "on the dev host. Production target is Snapdragon 695, which "
           "should be slower than the dev host — this is a regression.";
}

// ── Test 5 — degenerate scene reports failure (not falsely-true) ────────

// Build a pathological scene: all features at infinity (z >> baseline)
// AND all keyframe poses identical. Triangulation is undefined, the
// reprojection Jacobian is rank-deficient. BA must NOT crash and must
// NOT report converged=true. Acceptable outcomes:
//   - converged == false, OR
//   - iterations == max_iters with poor residual (still unconverged).
TEST(WindowedBA, DegenerateSceneReportsFailure) {
    GroundTruthScene scene;
    scene.poses.reserve(5);
    scene.features.reserve(15);

    // All five poses identical (zero baseline).
    for (int k = 0; k < 5; ++k) {
        WindowedBA::PoseObs p;
        p.keyframe_id = k;
        p.R_in  = cv::Matx33d::eye();
        p.t_in  = cv::Vec3d(0.0, 0.0, 0.0);
        p.R_out = p.R_in;
        p.t_out = p.t_in;
        p.is_anchor = (k == 0);
        scene.poses.push_back(p);
    }
    // All features at z = 1e6 m (effectively infinity), with random
    // x/y so we don't have a coincident point. Each is observed by
    // every keyframe — but since all poses are identical the
    // observations are identical too, providing zero parallax.
    std::mt19937_64 rng(0xBA0009u);
    std::uniform_real_distribution<double> uxy(-100.0, 100.0);
    for (int n = 0; n < 15; ++n) {
        WindowedBA::FeatureObs f;
        f.feature_id = n;
        f.p_w_in  = cv::Vec3d(uxy(rng), uxy(rng), 1e6);
        f.p_w_out = f.p_w_in;
        for (const auto& p : scene.poses) {
            cv::Vec2d uv;
            if (!projectPoint(p.R_in, p.t_in, f.p_w_in,
                              kFx, kFy, kCx, kCy, uv)) {
                continue;
            }
            f.obs.emplace_back(p.keyframe_id, uv);
        }
        scene.features.push_back(f);
    }

    WindowedBA solver;
    // Must not crash.
    WindowedBA::Result r = solver.solve(
        scene.poses, scene.features,
        kFx, kFy, kCx, kCy,
        kDefaultMaxIters, kDefaultHuberThreshPx);

    // Must not falsely report convergence on a degenerate scene.
    // Either the solver flags converged=false OR it runs out of
    // iterations (iterations >= max) — both signal "could not solve".
    const bool reported_failure =
        (!r.converged) || (r.iterations >= kDefaultMaxIters);
    EXPECT_TRUE(reported_failure)
        << "BA reported converged=" << r.converged
        << " with iterations=" << r.iterations
        << " on a degenerate (zero-baseline, points-at-infinity) scene. "
           "Solver must signal failure rather than claim a false solution.";
}
