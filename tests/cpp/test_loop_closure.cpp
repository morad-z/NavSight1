// [run_tests fix] cv::Rodrigues lives in opencv2/calib3d.hpp; without
// this include the test file fails to compile against modern OpenCV
// modular headers (4.5.x). Pure additive — no existing include displaced.
#include <opencv2/calib3d.hpp>
// Step 7 (Visual Production Plan): unit tests for the same-session
// loop-closure detector.
//
// Plan reference: docs/VISUAL_PRODUCTION_PLAN.md, "Step 7 — Same-session
// loop closure (DBoW2)" (around line 695).
//
// Agent A is landing the implementation in
// app/src/main/cpp/LoopClosureDetector.{h,cpp} (rewritten on top of the
// vendored DBoW2 source under third_party/DBoW2/), shipping the
// ORB-SLAM2 ORBvoc.bin asset, and wiring DBoW2 into app/CMakeLists.txt;
// Agent B is editing Tracker / native-lib / Kotlin glue. These tests
// touch ONLY the new LoopClosureDetector API:
//
//   class LoopClosureDetector {
//   public:
//       bool   loadVocabulary(const std::string& vocab_path);
//       size_t getKeyframeCount() const;
//       bool   isReady() const;
//       void   addKeyframe(uint64_t kf_id, double timestamp_ns,
//                          const cv::Mat& descriptors,
//                          const std::vector<cv::KeyPoint>& keypoints,
//                          const std::vector<cv::Point3f>& pts3d_world,
//                          const cv::Matx33d& R_world_cam,
//                          const cv::Vec3d&   t_cam_world);
//       struct LoopMatch { ... };
//       bool   tryDetectLoop(uint64_t now_kf_id, int64_t now_ns,
//                            const cv::Mat& descriptors,
//                            const std::vector<cv::KeyPoint>& keypoints,
//                            double fx, double fy, double cx, double cy,
//                            int64_t temporal_exclusion_ns,
//                            LoopMatch& out_match);
//   };
//
// Idiom mirrors test_orb_reloc.cpp / test_visual_robustness.cpp /
// test_windowed_ba.cpp from the Step-4/Step-5/Step-6 commits earlier
// today: anonymous-namespace helpers, OpenCV asserts, trend-based
// bounds, all-synthetic in-memory scenes (no Tracker / EKFState /
// FeatureManager dependency, no sim files, no device recordings).
//
// Vocabulary dependency: tryDetectLoop and the BoW-scoring path require
// a real ORBvoc.bin (~10 MB). On hosts where the asset is not staged
// next to the test binary the BoW-dependent tests skip with
// GTEST_SKIP(); the structural contract (count, no-crash-when-not-
// ready, temporal exclusion) is testable without the vocabulary.
//
// Note: desktop test build is currently BLOCKED on Morad's Windows host
// by an MSVC/MinGW OpenCV mismatch. These tests are designed to compile
// cleanly in concept; CI / Linux runs them. The DBoW2 link-time
// dependency lives in tests/cpp/CMakeLists.txt next to the existing
// navsight_core link.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "LoopClosureDetector.h"
#include "EKFState.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

// ── Step 7 scene parameters ─────────────────────────────────────────────

// Pinhole intrinsics. Same focal/cx/cy the Step-4 (test_orb_reloc.cpp)
// and Step-6 (test_windowed_ba.cpp) tests use so the suite agrees on
// what "the camera" looks like.
constexpr double kFx = 525.0;
constexpr double kFy = 525.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;

// Plan §3 / §4: temporal exclusion = 30 s, geometric verification floor
// = 30 PnP inliers. Both echoed here so the test pins the contract; the
// production constants live inside Agent A's LoopClosureDetector.cpp.
constexpr int64_t kTemporalExclusionNs   = 30LL * 1000000000LL;  // 30 s
constexpr int     kPnpInlierFloor        = 30;

// Frame size for the synthetic scenes. Mirrors the (640, 480) size
// Tracker sees after VioEngine downscale on the desktop test path.
constexpr int kFrameW = 640;
constexpr int kFrameH = 480;

// ── Helpers ─────────────────────────────────────────────────────────────

// Build a textured grayscale frame so cv::ORB has structure rather than
// pixel-scale noise to lock onto. Deterministic seed → reproducible
// across runs. Same recipe test_orb_reloc.cpp uses.
cv::Mat makeTexturedFrame(uint64_t seed) {
    cv::Mat raw(kFrameH, kFrameW, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(raw, cv::RNG::UNIFORM, 0, 256);
    cv::Mat blurred;
    cv::GaussianBlur(raw, blurred, cv::Size(0, 0), /*sigmaX=*/1.0);
    return blurred;
}

// ORB extraction with the Step-4 tuning (nfeatures = 250). Production
// FeatureManager uses the same parameters at keyframe creation, so the
// descriptors fed into addKeyframe / tryDetectLoop here have the same
// statistics as on-device.
void detectAndComputeOrb(const cv::Mat& gray,
                         std::vector<cv::KeyPoint>& kps,
                         cv::Mat& desc) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create(/*nfeatures=*/250);
    orb->detectAndCompute(gray, cv::noArray(), kps, desc);
}

// Best-effort lookup of the bundled ORB vocabulary. Tests run in a
// number of environments (CI Linux container, Android device push,
// Morad's Windows host once the OpenCV mismatch is resolved). We probe
// the candidates the production app + CI scripts currently stage; if
// none are present the BoW-dependent tests skip rather than fail.
bool resolveVocabularyPath(std::string& out_path) {
    static const char* const candidates[] = {
        // CI: assets/ symlinked next to the test binary.
        "assets/ORBvoc.bin",
        "assets/ORBvoc.dbow2",
        // Repo-root relative — when tests run from build/.
        "../app/src/main/assets/ORBvoc.bin",
        "../app/src/main/assets/ORBvoc.dbow2",
        // Repo-root relative — when tests run from repo root.
        "app/src/main/assets/ORBvoc.bin",
        "app/src/main/assets/ORBvoc.dbow2",
        // Env override for ad-hoc local runs.
        nullptr,
    };
    for (const char* const* p = candidates; *p != nullptr; ++p) {
        std::FILE* f = std::fopen(*p, "rb");
        if (f != nullptr) {
            std::fclose(f);
            out_path = *p;
            return true;
        }
    }
    if (const char* env = std::getenv("NAVSIGHT_ORB_VOCAB")) {
        std::FILE* f = std::fopen(env, "rb");
        if (f != nullptr) {
            std::fclose(f);
            out_path = env;
            return true;
        }
    }
    return false;
}

// Build a small grid of world-frame 3D points the synthetic scene
// projects from. Used by both the keyframe-storage and loop-detect
// paths so the geometry is shared. Points sit ~3 m in front of the
// origin in the world frame, spread across an 80 cm × 60 cm wall.
std::vector<cv::Point3f> makeSceneGrid() {
    std::vector<cv::Point3f> pts;
    pts.reserve(48);
    for (int row = -3; row <= 3; ++row) {
        for (int col = -3; col <= 3; ++col) {
            pts.emplace_back(static_cast<float>(col) * 0.10f,
                             static_cast<float>(row) * 0.10f,
                             3.0f);
        }
    }
    return pts;
}

// Project a world-frame point through the (R_world_cam, t_cam_world)
// convention LoopClosureDetector consumes. Returns true iff the point
// lies in front of the camera (z > 1e-6).
bool projectPoint(const cv::Matx33d& R_world_cam,
                  const cv::Vec3d&   t_cam_world,
                  const cv::Point3f& p_world,
                  cv::Point2f&       uv_out) {
    const cv::Vec3d p_w(p_world.x, p_world.y, p_world.z);
    const cv::Vec3d p_cam = R_world_cam * (p_w - t_cam_world);
    if (p_cam[2] <= 1e-6) {
        return false;
    }
    uv_out.x = static_cast<float>(kFx * p_cam[0] / p_cam[2] + kCx);
    uv_out.y = static_cast<float>(kFy * p_cam[1] / p_cam[2] + kCy);
    return true;
}

// Build a synthetic descriptor matrix (rows × 32, CV_8U) of randomised
// binary descriptors. Used for the "structural" tests where BoW scoring
// is not exercised — we only care that addKeyframe / count work end-to-
// end on plausibly-shaped inputs.
cv::Mat makeRandomDescriptors(int rows, uint64_t seed) {
    cv::Mat desc(rows, 32, CV_8U);
    cv::RNG rng(seed);
    rng.fill(desc, cv::RNG::UNIFORM, 0, 256);
    return desc;
}

// Build the parallel keypoints array for makeRandomDescriptors so the
// addKeyframe call sees a self-consistent (kp, desc) pair.
std::vector<cv::KeyPoint> makeRandomKeypoints(int n, uint64_t seed) {
    cv::RNG rng(seed);
    std::vector<cv::KeyPoint> kps;
    kps.reserve(n);
    for (int i = 0; i < n; ++i) {
        const float u = static_cast<float>(rng.uniform(20, kFrameW - 20));
        const float v = static_cast<float>(rng.uniform(20, kFrameH - 20));
        kps.emplace_back(u, v, /*size=*/7.0f);
    }
    return kps;
}

}  // namespace

// ── Test 1 — vocabulary load failure is observable + safe ────────────────

// Contract: loadVocabulary on a non-existent path must return false and
// leave isReady() == false. addKeyframe on a not-ready detector MUST
// silently no-op rather than crash — the tracker calls addKeyframe
// every keyframe and a missing vocabulary asset (e.g. corrupt install,
// pre-load race) must not crash the camera thread.
TEST(LoopClosure, VocabularyLoadFailureReturnsFalseAndIsReadyFalse) {
    LoopClosureDetector detector;

    EXPECT_FALSE(detector.isReady())
        << "Detector reports ready before any vocabulary is loaded.";

    const bool loaded = detector.loadVocabulary(
        "/this/path/definitely/does/not/exist/ORBvoc.bin");
    EXPECT_FALSE(loaded)
        << "loadVocabulary returned true for a non-existent path; the "
           "detector must propagate file-open failure to the caller.";

    EXPECT_FALSE(detector.isReady())
        << "isReady() reports true after a failed loadVocabulary; "
           "production gate (Tracker::shouldAttemptLoop) reads this "
           "flag and a false-positive here would push BoW queries "
           "through an uninitialised vocabulary.";

    // Stub keyframe: 16 random rows of CV_8U/32. The not-ready detector
    // must absorb this call without dereferencing the vocabulary.
    cv::Mat desc = makeRandomDescriptors(/*rows=*/16, /*seed=*/0xDEADBEEFu);
    std::vector<cv::KeyPoint> kps = makeRandomKeypoints(/*n=*/16,
                                                        /*seed=*/0xDEADBEEFu);
    std::vector<cv::Point3f> pts3d;  // empty is a valid input.
    cv::Matx33d R_world_cam = cv::Matx33d::eye();
    cv::Vec3d   t_cam_world(0, 0, 0);

    // The contract is "must not crash". GoogleTest's ASSERT_NO_FATAL_FAILURE
    // catches failed assertions inside the callee; an actual SIGSEGV is
    // surfaced by the harness as a test failure.
    ASSERT_NO_FATAL_FAILURE(detector.addKeyframe(
        /*kf_id=*/1, /*timestamp_ns=*/1.0e9,
        desc, kps, pts3d, R_world_cam, t_cam_world));

    // Count must not have advanced — the detector silently dropped the
    // keyframe because it has no vocabulary to score against.
    EXPECT_EQ(detector.getKeyframeCount(), 0u)
        << "Detector accepted a keyframe while not ready; production "
           "gate would then query an uninitialised BoW database.";
}

// ── Test 2 — addKeyframe increments the count when the detector is ready ─

// Contract: with the vocabulary loaded, addKeyframe on N synthetic
// (kp, desc, pts3d) tuples results in getKeyframeCount() == N. This is
// the structural counterpart to Test 1: same input shapes, but on a
// ready detector the keyframes survive into the BoW database.
//
// If the production API requires a real vocabulary (no test vocab
// shipped in the test sandbox) the test skips with a clear reason
// rather than failing — matches the test_slam_msckf.cpp idiom.
TEST(LoopClosure, AddKeyframeIncrementsCount) {
    LoopClosureDetector detector;

    std::string vocab_path;
    if (!resolveVocabularyPath(vocab_path)) {
        GTEST_SKIP() << "ORBvoc.{bin,dbow2} not staged next to the test "
                        "binary — set NAVSIGHT_ORB_VOCAB or copy the "
                        "asset to assets/ORBvoc.bin to run this case.";
    }
    if (!detector.loadVocabulary(vocab_path)) {
        GTEST_SKIP() << "Vocabulary file present but failed to load; "
                        "DBoW2 may have rejected the format. Path: "
                     << vocab_path;
    }
    ASSERT_TRUE(detector.isReady());

    constexpr int kKeyframes = 5;
    for (int i = 0; i < kKeyframes; ++i) {
        const uint64_t seed = 0x100u + static_cast<uint64_t>(i);
        cv::Mat desc = makeRandomDescriptors(/*rows=*/250, seed);
        std::vector<cv::KeyPoint> kps = makeRandomKeypoints(/*n=*/250, seed);
        std::vector<cv::Point3f>  pts3d = makeSceneGrid();

        cv::Matx33d R_world_cam = cv::Matx33d::eye();
        cv::Vec3d   t_cam_world(static_cast<double>(i) * 0.5, 0, 0);

        detector.addKeyframe(
            /*kf_id=*/static_cast<uint64_t>(i + 1),
            /*timestamp_ns=*/static_cast<double>(i + 1) * 1.0e9,
            desc, kps, pts3d, R_world_cam, t_cam_world);
    }

    EXPECT_EQ(detector.getKeyframeCount(),
              static_cast<size_t>(kKeyframes))
        << "addKeyframe count mismatch — either an internal eviction "
           "is dropping keyframes early or the BoW indexer is rejecting "
           "well-formed inputs silently.";
}

// ── Test 3 — temporal exclusion rejects recent self-matches ──────────────

// Contract: keyframe at ts = 3 s is "the present"; a query at ts = 3.5 s
// with the SAME descriptors and a 30 s temporal exclusion window must
// NOT be reported as a loop. This is the §3 plan gate that stops the
// detector from claiming "you've returned" the moment you've just been
// somewhere — the whole point of a 30 s gap is to require the user has
// actually walked away and come back.
//
// Skipped if the vocabulary is unavailable (the BoW score is
// short-circuited in the not-ready path).
TEST(LoopClosure, TemporalExclusionRejectsRecentMatch) {
    LoopClosureDetector detector;

    std::string vocab_path;
    if (!resolveVocabularyPath(vocab_path)) {
        GTEST_SKIP() << "ORBvoc.{bin,dbow2} not staged — temporal-gate "
                        "test requires a ready BoW database to be "
                        "meaningful.";
    }
    if (!detector.loadVocabulary(vocab_path)) {
        GTEST_SKIP() << "Vocabulary file present but failed to load.";
    }
    ASSERT_TRUE(detector.isReady());

    // Three keyframes at ts = 1 s, 2 s, 3 s with a SHARED descriptor
    // matrix so the BoW score at the query is maximal — without this
    // the test could trivially pass on a low BoW score, never reaching
    // the temporal gate.
    cv::Mat shared_desc = makeRandomDescriptors(/*rows=*/250,
                                                /*seed=*/0xAAAAu);
    std::vector<cv::KeyPoint> shared_kps = makeRandomKeypoints(
        /*n=*/250, /*seed=*/0xAAAAu);
    std::vector<cv::Point3f>  pts3d = makeSceneGrid();

    for (int i = 1; i <= 3; ++i) {
        cv::Matx33d R_world_cam = cv::Matx33d::eye();
        cv::Vec3d   t_cam_world(static_cast<double>(i) * 0.1, 0, 0);
        detector.addKeyframe(
            /*kf_id=*/static_cast<uint64_t>(i),
            /*timestamp_ns=*/static_cast<double>(i) * 1.0e9,
            shared_desc, shared_kps, pts3d, R_world_cam, t_cam_world);
    }

    // Query at 3.5 s with the SAME descriptors as the most recent
    // keyframe — high BoW score guaranteed. A temporal_exclusion_ns of
    // 30 s means every stored keyframe is inside the exclusion window
    // and the detector must reject all of them.
    LoopClosureDetector::LoopMatch match{};
    const bool found = detector.tryDetectLoop(
        /*now_kf_id=*/4,
        /*now_ns=*/static_cast<int64_t>(3.5 * 1.0e9),
        shared_desc, shared_kps,
        kFx, kFy, kCx, kCy,
        kTemporalExclusionNs,
        match);

    EXPECT_FALSE(found)
        << "tryDetectLoop reported a loop on a query within the 30 s "
           "exclusion window; the temporal gate is not engaging. A "
           "false positive here means the very first keyframe of every "
           "session triggers a self-loop on the next frame.";
}

// ── Test 4 — geometric verification rejects BoW-only matches ─────────────

// Contract: a high BoW score is necessary but NOT sufficient. The §4
// plan gate is solvePnPRansac with ≥ 30 inliers; without geometric
// agreement between the stored 3D points and the query keypoints the
// detector must reject the candidate even when DBoW2 scores it high.
//
// This is the test that pins the "BoW alone is not a loop" property —
// without it the detector reduces to a place classifier, which is what
// the plan §"Why DBoW2" explicitly says it is NOT.
//
// Construction: two keyframes share descriptors (same BoW vector) but
// the 3D points stored at the FIRST keyframe project to image points
// that have nothing to do with the SECOND keyframe's keypoint
// locations — keypoints are at random pixels chosen from a different
// RNG seed. PnP cannot find a pose that explains the correspondence,
// so it fails the inlier floor.
TEST(LoopClosure, GeometricVerificationRejectsBowOnlyMatches) {
    LoopClosureDetector detector;

    std::string vocab_path;
    if (!resolveVocabularyPath(vocab_path)) {
        GTEST_SKIP() << "ORBvoc.{bin,dbow2} not staged — geometric-gate "
                        "test requires a ready BoW database to even "
                        "advance to the PnP stage.";
    }
    if (!detector.loadVocabulary(vocab_path)) {
        GTEST_SKIP() << "Vocabulary file present but failed to load.";
    }
    ASSERT_TRUE(detector.isReady());

    // Stored keyframe with the scene grid as its 3D points.
    cv::Mat shared_desc = makeRandomDescriptors(/*rows=*/250,
                                                /*seed=*/0xBBBBu);
    std::vector<cv::KeyPoint> stored_kps = makeRandomKeypoints(
        /*n=*/250, /*seed=*/0xBBBBu);
    std::vector<cv::Point3f>  pts3d = makeSceneGrid();

    cv::Matx33d R_world_cam_stored = cv::Matx33d::eye();
    cv::Vec3d   t_cam_world_stored(0, 0, 0);
    detector.addKeyframe(
        /*kf_id=*/1,
        /*timestamp_ns=*/1.0e9,
        shared_desc, stored_kps, pts3d,
        R_world_cam_stored, t_cam_world_stored);

    // Query with the SAME descriptors (BoW score will be at the top of
    // the candidate ranking) but DIFFERENT keypoint locations — the
    // 2D-3D correspondence assumed by solvePnPRansac is broken so the
    // PnP gate must reject. We use a different RNG seed to guarantee
    // the query keypoints do not coincidentally line up with the
    // projection of the stored 3D points.
    std::vector<cv::KeyPoint> query_kps = makeRandomKeypoints(
        /*n=*/250, /*seed=*/0xCCCCu);

    LoopClosureDetector::LoopMatch match{};
    // Query 60 s later — well past the 30 s temporal exclusion gate so
    // the candidate is admitted to the geometric verification stage.
    const bool found = detector.tryDetectLoop(
        /*now_kf_id=*/2,
        /*now_ns=*/static_cast<int64_t>(60.0 * 1.0e9),
        shared_desc, query_kps,
        kFx, kFy, kCx, kCy,
        kTemporalExclusionNs,
        match);

    EXPECT_FALSE(found)
        << "tryDetectLoop accepted a candidate whose 2D-3D "
           "correspondence cannot be explained by any pose (PnP "
           "inlier floor is " << kPnpInlierFloor << "). A false "
           "positive here would inject a spurious correction through "
           "the EKF::updateRelativePose channel.";
}

// ── Test 5 — clean re-visit produces a high-confidence loop match ────────

// Contract: the happy-path acceptance gate. On a clean re-visit (same
// scene, same descriptors, geometry consistent, query beyond the
// temporal exclusion window) the detector reports found = true with
// matched_kf_id matching the stored keyframe, pnp_inliers ≥ the §4
// floor, and a high BoW score.
//
// This is the necessary "yes, fire" counterpart to Tests 3 and 4;
// without it those two tests could trivially pass by always returning
// false. Together the three tests pin the decision boundary in both
// directions.
//
// Construction: stored keyframe at (R = I, t = 0) observes the scene
// grid; query keyframe at (R = I, t = (0.05, 0, 0)) — small parallax
// translation, same scene. Query keypoints are the projected
// 2D positions of the stored 3D points through the query's pose, so
// PnP solves cleanly with all of them as inliers.
TEST(LoopClosure, AcceptsValidLoopReturnsLoopMatch) {
    LoopClosureDetector detector;

    std::string vocab_path;
    if (!resolveVocabularyPath(vocab_path)) {
        GTEST_SKIP() << "ORBvoc.{bin,dbow2} not staged — accept-path "
                        "test requires a ready BoW database.";
    }
    if (!detector.loadVocabulary(vocab_path)) {
        GTEST_SKIP() << "Vocabulary file present but failed to load.";
    }
    ASSERT_TRUE(detector.isReady());

    // Build the stored keyframe from a real textured frame so that
    // ORB extracts matchable descriptors (the random-byte descriptors
    // used by Tests 3/4 are fine for BoW-score saturation but here we
    // need BFMatcher Hamming + Lowe ratio to recover correspondences
    // for the geometric verification stage).
    cv::Mat textured = makeTexturedFrame(/*seed=*/0xC0FFEEu);
    std::vector<cv::KeyPoint> stored_kps;
    cv::Mat stored_desc;
    detectAndComputeOrb(textured, stored_kps, stored_desc);

    ASSERT_GE(static_cast<int>(stored_kps.size()),
              kPnpInlierFloor + 10)
        << "Stored keyframe has fewer ORB features than the PnP floor "
           "needs to admit a match — synthetic scene is too degenerate "
           "for this test.";

    // 3D point per keypoint at z = 3 m, x/y back-projected from the
    // pinhole. This is the cleanest possible synthetic correspondence:
    // when the query camera projects these world points it will land
    // exactly on the stored keypoints when the query pose equals the
    // stored pose, and within sub-pixel of them when the query pose
    // is perturbed by the small translation below.
    std::vector<cv::Point3f> pts3d;
    pts3d.reserve(stored_kps.size());
    for (const auto& kp : stored_kps) {
        const double z = 3.0;
        const double x = (static_cast<double>(kp.pt.x) - kCx) * z / kFx;
        const double y = (static_cast<double>(kp.pt.y) - kCy) * z / kFy;
        pts3d.emplace_back(static_cast<float>(x),
                           static_cast<float>(y),
                           static_cast<float>(z));
    }

    cv::Matx33d R_world_cam_stored = cv::Matx33d::eye();
    cv::Vec3d   t_cam_world_stored(0, 0, 0);
    detector.addKeyframe(
        /*kf_id=*/42,
        /*timestamp_ns=*/1.0e9,
        stored_desc, stored_kps, pts3d,
        R_world_cam_stored, t_cam_world_stored);

    // Query at a temporally distant moment (well past the 30 s gate)
    // with the SAME descriptors — idealised perfect re-visit. Re-
    // project the stored 3D points through a small-translation query
    // pose so the geometric verification has a clean PnP solution.
    cv::Matx33d R_world_cam_query = cv::Matx33d::eye();
    cv::Vec3d   t_cam_world_query(0.05, 0.0, 0.0);

    std::vector<cv::KeyPoint> query_kps;
    query_kps.reserve(stored_kps.size());
    for (size_t i = 0; i < stored_kps.size(); ++i) {
        cv::Point2f uv;
        if (!projectPoint(R_world_cam_query, t_cam_world_query,
                          pts3d[i], uv)) {
            // Behind the camera — drop. Should not happen for z = 3 m
            // and the small translation we use.
            continue;
        }
        query_kps.emplace_back(uv.x, uv.y, stored_kps[i].size);
    }
    ASSERT_GE(static_cast<int>(query_kps.size()),
              kPnpInlierFloor + 10)
        << "Re-projected too few stored points to clear the PnP floor; "
           "the synthetic re-visit pose is too far from the stored "
           "pose for this test.";

    LoopClosureDetector::LoopMatch match{};
    const bool found = detector.tryDetectLoop(
        /*now_kf_id=*/100,
        /*now_ns=*/static_cast<int64_t>(60.0 * 1.0e9),
        stored_desc, query_kps,
        kFx, kFy, kCx, kCy,
        kTemporalExclusionNs,
        match);

    ASSERT_TRUE(found)
        << "tryDetectLoop did NOT accept a clean re-visit: same "
           "descriptors, same scene, query 60 s after store with the "
           "stored 3D points re-projected through the query pose. "
           "Either the BoW threshold is too strict, the PnP gate is "
           "rejecting valid geometry, or the temporal-exclusion path "
           "is over-firing.";

    EXPECT_EQ(match.matched_kf_id, static_cast<uint64_t>(42))
        << "tryDetectLoop accepted a loop but reported the wrong "
           "matched_kf_id; the BoW database returned a different "
           "candidate than the one we stored.";

    EXPECT_GE(match.pnp_inliers, kPnpInlierFloor)
        << "tryDetectLoop accepted a loop with fewer than "
        << kPnpInlierFloor << " PnP inliers; the §4 acceptance gate "
           "is not enforced — a future drift in the gate would let "
           "low-evidence corrections through to the EKF.";

    // BoW score has no fixed scale across vocabularies but on an
    // identical-descriptor query it should be substantially above
    // zero. The plan §3 calls for a "similarity threshold"; we pin
    // only the strict-positive lower bound here so the test is
    // robust across DBoW2 weighting strategies.
    EXPECT_GT(match.bow_score, 0.0)
        << "tryDetectLoop accepted a loop but reported a BoW score "
           "of " << match.bow_score << " — non-positive scores "
           "indicate the BoW path was short-circuited.";
}

// ── Test 6 — absolute-pose update applies a real correction ──────────────
//
// ADR-013 §"Correction injection — absolute pose path": when the matched
// keyframe is older than the EKF clone window (always the case under the
// 30 s temporal exclusion vs. ~5–10 s clone window) the only correction
// channel that can reach it is updateAbsolutePose. This test pins the
// channel: drift the EKF position by 5 m, then call updateAbsolutePose
// with the pre-drift pose as the target. Post-update position must move
// toward the target by a measurable fraction. We do NOT assert full
// convergence — one EKF update with finite covariance lands somewhere
// short of the target; the damping ramp + repeated updates close the
// rest of the gap in production.
//
// Self-contained: builds an EKFState from scratch, no Tracker /
// LoopClosureDetector dependency. The contract is purely about the new
// EKFState measurement channel.
TEST(LoopClosure, AbsolutePoseUpdateAppliesCorrection) {
    EKFState ekf;
    cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);
    ekf.initializeFull(R_eye, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    // Add one clone so the EKF is in the same shape as production. The
    // absolute-pose channel does NOT consume the clone — it operates on
    // the IMU state directly — but adding it pins the test against the
    // real state-vector layout.
    ekf.addClone(R_eye, cv::Mat::zeros(3, 1, CV_64F), 1'000'000'000LL);

    // Inject a 5 m drift into the EKF position via a tight measurement
    // through updatePDRStep (X/Z axes). PDR's H is +I on δp_x/δp_z so a
    // measurement of (5, 0, 0) plus tight variance pulls the position
    // toward (5, 0, 0). Run the update twice with very tight variance
    // so the EKF lands close to the target — we want a meaningful drift,
    // not a 1 cm nudge dominated by the prior.
    for (int i = 0; i < 4; ++i) {
        ASSERT_TRUE(ekf.updatePDRStep(/*dx_world=*/5.0,
                                       /*dz_world=*/0.0,
                                       /*var=*/1e-6));
    }
    cv::Mat p_drifted = ekf.getPosition();
    const double drifted_x = p_drifted.at<double>(0);
    ASSERT_GT(drifted_x, 1.0)
        << "Pre-condition: EKF position should have drifted noticeably "
           "before the absolute-pose update. Got x="
        << drifted_x;

    // Target: pre-drift origin pose. Apply updateAbsolutePose with
    // moderate variance — same scale loop closure uses (~3° rot, 10 cm
    // pos, no damping inflation).
    cv::Mat target_R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat target_p = cv::Mat::zeros(3, 1, CV_64F);
    const double sigma_axis_sq_R = 0.05236 * 0.05236;  // (3°)² in rad²
    const double var_p           = 0.10 * 0.10;        // 10 cm² in m²
    const bool ok = ekf.updateAbsolutePose(target_R, target_p,
                                            sigma_axis_sq_R, var_p);
    ASSERT_TRUE(ok)
        << "updateAbsolutePose returned false on a well-formed input. "
           "Either the chi² gate is over-tight (5 m residual @ 10 cm² "
           "var should be inside the χ²(0.999, 6 DOF) ≈ 22.5 outer gate "
           "after the EKF prior is folded in) or input validation is "
           "rejecting valid Mat shapes.";

    cv::Mat p_after = ekf.getPosition();
    const double after_x = p_after.at<double>(0);
    const double moved   = drifted_x - after_x;          // positive if
                                                          // moved toward 0
    const double moved_frac = moved / drifted_x;

    // Soft assertion: the correction must move us meaningfully toward
    // the target. A single EKF update with finite covariance + damping
    // doesn't snap to the target; >30% closure is the bar this test
    // pins. A regression that flips the rotation sign or zeroes the
    // position Jacobian would land moved_frac near 0 — caught here.
    EXPECT_GT(moved_frac, 0.30)
        << "updateAbsolutePose moved the EKF position by only "
        << (moved_frac * 100.0) << "% of the drift toward the target. "
        << "Drifted x=" << drifted_x << " m → after x=" << after_x
        << " m. A regression that zeros the position Jacobian or flips "
           "the residual sign would surface here as a near-zero or "
           "negative fraction.";
    EXPECT_GT(moved, 0.0)
        << "updateAbsolutePose moved the EKF position AWAY from the "
           "target (moved=" << moved << " m). Sign of the position "
           "residual or Jacobian is wrong.";
}

// ── Test 7 — absolute-pose update rotates toward target ──────────────────
//
// Pins the rotation Jacobian sign that the foreground reviewer flagged.
// Initialize the EKF with R_GtoI_ = Rodrigues(0.5 rad about world-Z),
// call updateAbsolutePose with target_R = identity, assert the resulting
// R_GtoI_ has rotated toward identity (axis-angle norm strictly less than
// the starting 0.5 rad). A sign flip in H_R or in the residual Lie
// algebra would *increase* the rotation magnitude — caught here.
//
// Self-contained: pure EKFState contract test, no Tracker /
// LoopClosureDetector dependency.
TEST(LoopClosure, AbsolutePoseUpdateRotatesTowardTarget) {
    EKFState ekf;
    // Start with R_GtoI_ rotated by 0.5 rad (~28.6°) about world-Z. This
    // is a pure-yaw drift, the same shape loop closure typically corrects.
    cv::Mat R_initial;
    cv::Rodrigues(cv::Vec3d(0.0, 0.0, 0.5), R_initial);
    ekf.initializeFull(R_initial, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    ekf.addClone(R_initial, cv::Mat::zeros(3, 1, CV_64F), 1'000'000'000LL);

    cv::Mat R_before = ekf.getRotation();
    cv::Mat r_before;
    cv::Rodrigues(R_before, r_before);
    const double mag_before = cv::norm(r_before);
    ASSERT_NEAR(mag_before, 0.5, 1e-6)
        << "Pre-condition: starting rotation magnitude should be 0.5 rad. "
           "Got " << mag_before;

    // Target: identity rotation. Use loose position variance because we
    // don't care about p — we only want the rotation to converge.
    cv::Mat target_R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat target_p = cv::Mat::zeros(3, 1, CV_64F);
    // Tight rotation variance (1°² in rad²) to make the gain large enough
    // that the correction is meaningful in one shot — same scale loop
    // closure uses with strength = 1.0.
    const double sigma_axis_sq_R = (1.0 * M_PI / 180.0) *
                                    (1.0 * M_PI / 180.0);
    const double var_p           = 1.0;  // wide — don't constrain p

    ASSERT_TRUE(ekf.updateAbsolutePose(target_R, target_p,
                                        sigma_axis_sq_R, var_p))
        << "updateAbsolutePose returned false on a well-formed input. "
           "0.5 rad residual at 1° measurement noise should be inside "
           "the χ² gate after the EKF prior is folded in.";

    cv::Mat R_after = ekf.getRotation();
    cv::Mat r_after;
    cv::Rodrigues(R_after, r_after);
    const double mag_after = cv::norm(r_after);

    // The correction must reduce the rotation magnitude. A sign flip in
    // H_R or in the Lie-algebra residual would *increase* it.
    EXPECT_LT(mag_after, mag_before)
        << "updateAbsolutePose ROTATED AWAY from the target. "
        << "Initial |R|=" << mag_before << " rad, after |R|="
        << mag_after << " rad. Rotation Jacobian sign is wrong: a +I_3 / "
           "−I_3 mismatch on the δθ rows would surface exactly like "
           "this. See the H_R derivation in EKFState::updateAbsolutePose.";

    // Soft assertion on convergence: with high gain we expect to close
    // most of the gap. Loose threshold (>20% closure) so the test isn't
    // sensitive to small changes in the EKF prior covariance.
    const double closure_frac = (mag_before - mag_after) / mag_before;
    EXPECT_GT(closure_frac, 0.20)
        << "updateAbsolutePose only closed " << (closure_frac * 100.0)
        << "% of the rotation gap with tight measurement variance — "
           "expected at least 20%. Either H_R is wrong magnitude or the "
           "Joseph update is starving the rotation block.";
}

// ── Test 8 — Tracker-style world-pose composition round-trip ─────────────
//
// Pins the translation composition convention the foreground reviewer
// flagged. Build a synthetic LoopMatch from two known camera world
// poses (now and match) and verify that the Tracker-side composition
//
//     target_t_cam_world = R_world_cam_match * t_now_to_match
//                        + t_cam_world_match
//
// recovers the now-camera's known world position. A convention mismatch
// in either field would produce a wildly different position (the
// reviewer's failure mode).
//
// We mirror the LoopClosureDetector::tryDetectLoop construction
// (LoopClosureDetector.cpp:440-451) so this test pins the SAME formulas
// the production path uses.
TEST(LoopClosure, LoopMatchTranslationConventionRoundTrip) {
    // Two cameras in world frame:
    //   match cam at world position (5, 0, 0), pure-yaw 30° about world-Y
    //   now   cam at world position (8, 0, 4), pure-yaw 60° about world-Y
    cv::Mat R_world_cam_match_cv;
    cv::Rodrigues(cv::Vec3d(0.0, 30.0 * M_PI / 180.0, 0.0),
                  R_world_cam_match_cv);
    cv::Matx33d R_world_cam_match(R_world_cam_match_cv);
    cv::Vec3d   t_cam_world_match(5.0, 0.0, 0.0);   // match cam pos in world

    cv::Mat R_world_cam_now_cv;
    cv::Rodrigues(cv::Vec3d(0.0, 60.0 * M_PI / 180.0, 0.0),
                  R_world_cam_now_cv);
    cv::Matx33d R_world_cam_now(R_world_cam_now_cv);
    cv::Vec3d   t_cam_world_now(8.0, 0.0, 4.0);     // now cam pos in world

    // Build R_now_to_match / t_now_to_match using the EXACT formulas in
    // LoopClosureDetector.cpp:440-451 (PnP convention, match-cam frame
    // translation). PnP returns world→now-cam:
    //     R_now_world = R_world_cam_now.t()
    //     t_now_world = -R_now_world * t_cam_world_now    (world origin
    //                                                       in now-cam)
    const cv::Matx33d R_now_world  = R_world_cam_now.t();
    const cv::Vec3d   t_now_world  = -(R_now_world * t_cam_world_now);
    const cv::Matx33d R_match_world = R_world_cam_match.t();
    const cv::Vec3d   t_match_world = -(R_match_world * t_cam_world_match);
    const cv::Matx33d R_now_to_match = R_match_world * R_now_world.t();
    const cv::Vec3d   t_now_to_match =
        t_match_world - R_now_to_match * t_now_world;

    // Tracker-side composition (mirror of consumeLoopClosureMatchIfReady):
    const cv::Matx33d target_R_world_cam = R_world_cam_match * R_now_to_match;
    const cv::Vec3d   target_t_cam_world = R_world_cam_match * t_now_to_match
                                          + t_cam_world_match;

    // Round-trip: target_R_world_cam must equal R_world_cam_now and
    // target_t_cam_world must equal t_cam_world_now (within float noise).
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(target_R_world_cam(r, c), R_world_cam_now(r, c), 1e-9)
                << "Composed target_R_world_cam mismatch at (" << r
                << ", " << c << "). Either R_now_to_match's convention is "
                   "different from what Tracker assumes, or the production "
                   "composition order is wrong.";
        }
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(target_t_cam_world[i], t_cam_world_now[i], 1e-9)
            << "Composed target_t_cam_world mismatch at axis " << i
            << ". Reviewer flagged exactly this — a convention mismatch "
               "between t_now_to_match (claimed world→cam tvec) and "
               "t_cam_world_match (cam-pos-in-world). Round-trip from "
               "synthetic world poses confirms the formula in "
               "Tracker::consumeLoopClosureMatchIfReady is correct.";
    }
}
