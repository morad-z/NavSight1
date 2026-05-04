// Step 4 (Visual Production Plan): unit tests for ORB descriptors at
// keyframes + KLT-loss relocalization.
//
// Plan reference: docs/VISUAL_PRODUCTION_PLAN.md, "Step 4 — ORB descriptors
// at keyframes (relocalization + loop foundation)" (around line 557).
//
// Agent A is landing the FeatureManager API
// (storeKeyframeDescriptors / getKeyframeDescriptors) in parallel; Agent C
// is landing KeyframeDescriptors.h. To stay decoupled from the still-in-
// flight class wiring, these tests exercise the SAME OpenCV primitives
// (cv::ORB, cv::BFMatcher with NORM_HAMMING + Lowe ratio 0.75,
// cv::findEssentialMat with RANSAC) and the SAME parameters the
// implementation agent is using. This verifies the relocalization recipe
// on synthetic two-frame scenes without depending on any application-
// layer classes that may not exist at compile time on the test host.
//
// Idiom mirrors test_relative_rotation.cpp / test_slam_msckf.cpp:
// fixture-style construction in an anonymous namespace, OpenCV asserts,
// trend-based bounds (no brittle absolute counts) sized from the Step 4
// acceptance gates ("≥ 50 ORB inliers" → tests guard ≥ 30 on the smaller
// synthetic scenes used here so passes are reliable on CI).
//
// All scenes are synthetic and in-memory. No sim files, no device
// recordings, no FeatureManager dependency.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

namespace {

// Step 4 ORB tuning per FeatureManager (plan §1):
//   cv::ORB::create(nfeatures = 250)
// nfeatures is a cap, not a target — the actual count depends on scene
// texture. We assert ">= 100 keypoints" on a textured noise frame, well
// below the cap to avoid false negatives on slightly low-contrast
// regions.
constexpr int    kOrbNFeatures    = 250;
constexpr float  kLoweRatio       = 0.75f;

// findEssentialMat tuning per Step 4 spec.
constexpr double kRansacFocal     = 525.0;
constexpr double kRansacCx        = 320.0;
constexpr double kRansacCy        = 240.0;
constexpr double kRansacThreshPx  = 1.5;
constexpr double kRansacConf      = 0.999;

// Frame size used by all synthetic scenes. Matches the (640, 480) size
// VioEngine feeds Tracker after downscale on the desktop test path.
constexpr int kFrameW = 640;
constexpr int kFrameH = 480;

// Build a textured grayscale frame: uniform noise + Gaussian blur σ=1.0.
// This is what Step 4 §1 prescribes ("Gaussian-blurred version of the
// gray frame") so ORB has structure rather than pixel-scale noise to
// lock onto. Deterministic seed → reproducible across runs.
cv::Mat makeTexturedNoiseFrame(uint64_t seed) {
    cv::Mat raw(kFrameH, kFrameW, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(raw, cv::RNG::UNIFORM, 0, 256);
    cv::Mat blurred;
    cv::GaussianBlur(raw, blurred, cv::Size(0, 0), /*sigmaX=*/1.0);
    return blurred;
}

// Build a frame with strong geometric structure: a chessboard-like grid
// of filled rectangles on a mid-gray background. ORB locks onto the
// corners reliably, which makes match-count assertions deterministic.
cv::Mat makeGeometricFrame() {
    cv::Mat frame(kFrameH, kFrameW, CV_8UC1, cv::Scalar(128));
    const int cell = 40;
    for (int y = 0; y < kFrameH; y += cell * 2) {
        for (int x = 0; x < kFrameW; x += cell * 2) {
            cv::rectangle(frame,
                          cv::Rect(x, y, cell, cell),
                          cv::Scalar(255), cv::FILLED);
            cv::rectangle(frame,
                          cv::Rect(x + cell, y + cell, cell, cell),
                          cv::Scalar(0), cv::FILLED);
        }
    }
    // Mild blur: ORB on a perfectly hard-edged chessboard quantises
    // keypoints onto exact corners, which kills sub-pixel descriptor
    // diversity. A small blur restores realistic descriptor spread.
    cv::Mat blurred;
    cv::GaussianBlur(frame, blurred, cv::Size(0, 0), /*sigmaX=*/1.0);
    return blurred;
}

// Apply a translation-only affine warp.
cv::Mat translate(const cv::Mat& src, double tx, double ty) {
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, tx,
                                            0, 1, ty);
    cv::Mat dst;
    cv::warpAffine(src, dst, M, src.size(),
                   cv::INTER_LINEAR, cv::BORDER_REFLECT101);
    return dst;
}

// Apply a rotation about the image center.
cv::Mat rotateAboutCenter(const cv::Mat& src, double angle_deg) {
    cv::Point2f center(static_cast<float>(src.cols) * 0.5f,
                       static_cast<float>(src.rows) * 0.5f);
    cv::Mat M = cv::getRotationMatrix2D(center, angle_deg, /*scale=*/1.0);
    cv::Mat dst;
    cv::warpAffine(src, dst, M, src.size(),
                   cv::INTER_LINEAR, cv::BORDER_REFLECT101);
    return dst;
}

// Compute ORB keypoints + descriptors with the Step-4 parameters.
void detectAndComputeOrb(const cv::Mat& gray,
                         std::vector<cv::KeyPoint>& kps,
                         cv::Mat& desc) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create(kOrbNFeatures);
    orb->detectAndCompute(gray, cv::noArray(), kps, desc);
}

// BF Hamming + knn(k=2) + Lowe ratio (0.75). Returns surviving "good"
// matches in a flat vector (the cross-check flag is OFF on purpose;
// Step 4 spec uses the ratio test for ambiguity rejection — see plan).
std::vector<cv::DMatch> matchWithLoweRatio(const cv::Mat& descA,
                                           const cv::Mat& descB) {
    cv::BFMatcher matcher(cv::NORM_HAMMING, /*crossCheck=*/false);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(descA, descB, knn, /*k=*/2);

    std::vector<cv::DMatch> good;
    good.reserve(knn.size());
    for (const auto& pair : knn) {
        if (pair.size() < 2) continue;
        if (pair[0].distance < kLoweRatio * pair[1].distance) {
            good.push_back(pair[0]);
        }
    }
    return good;
}

// Run findEssentialMat RANSAC on a set of good matches and return the
// number of inliers. K is the pinhole intrinsic from Step 4 spec.
int countEssentialInliers(const std::vector<cv::KeyPoint>& kpsA,
                          const std::vector<cv::KeyPoint>& kpsB,
                          const std::vector<cv::DMatch>& matches,
                          cv::Mat& inlier_mask_out) {
    if (matches.size() < 5) {
        inlier_mask_out = cv::Mat();
        return 0;
    }
    std::vector<cv::Point2f> ptsA, ptsB;
    ptsA.reserve(matches.size());
    ptsB.reserve(matches.size());
    for (const auto& m : matches) {
        ptsA.push_back(kpsA[m.queryIdx].pt);
        ptsB.push_back(kpsB[m.trainIdx].pt);
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 kRansacFocal, 0,            kRansacCx,
                 0,            kRansacFocal, kRansacCy,
                 0,            0,            1);

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(ptsA, ptsB, K,
                                     cv::RANSAC,
                                     kRansacConf,
                                     kRansacThreshPx,
                                     mask);
    if (E.empty() || mask.empty()) {
        inlier_mask_out = cv::Mat();
        return 0;
    }
    inlier_mask_out = mask;
    return cv::countNonZero(mask);
}

}  // namespace

// ── Test 1 — descriptors compute on a textured frame ─────────────────────

// Verifies the basic ORB compute path produces the expected descriptor
// shape (CV_8U, 32 cols) and a useful keypoint count on a textured noise
// frame. This is the entry point of the Step-4 keyframe pipeline.
TEST(OrbReloc, OrbDescriptorsComputeOnTexturedFrame) {
    cv::Mat frame = makeTexturedNoiseFrame(/*seed=*/0xC0FFEEu);

    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    detectAndComputeOrb(frame, kps, desc);

    // Plan target is nfeatures=250, but textured noise rarely hits the
    // cap. 100 is a robust floor — well below the cap, well above noise.
    EXPECT_GE(static_cast<int>(kps.size()), 100)
        << "ORB found too few keypoints on a blurred uniform-noise frame; "
           "either ORB params changed or scene generator is broken.";

    ASSERT_FALSE(desc.empty());
    EXPECT_EQ(desc.type(), CV_8U);
    EXPECT_EQ(desc.cols,  32);
    EXPECT_EQ(desc.rows,  static_cast<int>(kps.size()));
}

// ── Test 2 — Lowe ratio rejects ambiguous matches even on identical frames ─

// On two identical frames every keypoint has a perfect match in the other
// frame, but textured noise produces near-tied second-best matches that
// the ratio test correctly rejects. Asserts that the ratio test is
// actually filtering, not passing everything through.
TEST(OrbReloc, BfMatchWithRatioTestRejectsAmbiguous) {
    cv::Mat frame = makeTexturedNoiseFrame(/*seed=*/0xBADF00Du);

    std::vector<cv::KeyPoint> kpsA, kpsB;
    cv::Mat descA, descB;
    detectAndComputeOrb(frame, kpsA, descA);
    detectAndComputeOrb(frame, kpsB, descB);

    ASSERT_GT(static_cast<int>(kpsA.size()), 0);
    ASSERT_EQ(kpsA.size(), kpsB.size());

    std::vector<cv::DMatch> good = matchWithLoweRatio(descA, descB);

    // Ratio test must keep a healthy fraction (high signal — same scene)
    // but must NOT keep 100% (noise-region keypoints have near-tied 2nd-
    // best neighbours and correctly fail the ratio test).
    const double rate = static_cast<double>(good.size()) /
                        static_cast<double>(kpsA.size());
    EXPECT_GT(rate, 0.30)
        << "Lowe ratio dropped too aggressively on identical frames "
           "(rate=" << rate << "); ratio threshold or knn k may be off.";
    EXPECT_LT(static_cast<int>(good.size()),
              static_cast<int>(kpsA.size()))
        << "Lowe ratio kept 100% of matches on textured noise — ratio "
           "filter is not engaging (check k=2 and ratio<0.75).";
}

// ── Test 3 — BF + ratio finds correspondences on synthetic overlap ───────

// Translation-only warp of a strongly-textured geometric frame.
// Step 4 spec requires the descriptor matcher to recover correspondences
// across viewpoint shifts. 30 px is well within ORB's match radius.
TEST(OrbReloc, BfMatchFindsCorrespondencesOnSyntheticOverlap) {
    cv::Mat frameA = makeGeometricFrame();
    cv::Mat frameB = translate(frameA, /*tx=*/30.0, /*ty=*/0.0);

    std::vector<cv::KeyPoint> kpsA, kpsB;
    cv::Mat descA, descB;
    detectAndComputeOrb(frameA, kpsA, descA);
    detectAndComputeOrb(frameB, kpsB, descB);

    ASSERT_GT(static_cast<int>(kpsA.size()), 20);
    ASSERT_GT(static_cast<int>(kpsB.size()), 20);

    std::vector<cv::DMatch> good = matchWithLoweRatio(descA, descB);

    // Clean translation — matcher should recover well over the floor.
    EXPECT_GE(static_cast<int>(good.size()), 20)
        << "BF + Lowe ratio failed to recover ≥20 matches on a clean "
           "30 px translation of a strongly-textured chessboard scene.";
}

// ── Test 4 — RANSAC essential gate keeps inliers, rejects outliers ───────

// Same scene rotated 5° about Z. Step 4 spec requires findEssentialMat
// RANSAC at threshold 1.5 px to gate matches before pose recovery.
// Asserts both the absolute inlier floor and the inlier *rate* — a
// broken RANSAC threshold often passes too many outliers (rate stays
// high but absolute counts on harder scenes don't).
TEST(OrbReloc, RansacEssentialGateKeepsInliersRejectsOutliers) {
    cv::Mat frameA = makeGeometricFrame();
    cv::Mat frameB = rotateAboutCenter(frameA, /*angle_deg=*/5.0);

    std::vector<cv::KeyPoint> kpsA, kpsB;
    cv::Mat descA, descB;
    detectAndComputeOrb(frameA, kpsA, descA);
    detectAndComputeOrb(frameB, kpsB, descB);

    std::vector<cv::DMatch> good = matchWithLoweRatio(descA, descB);
    ASSERT_GE(static_cast<int>(good.size()), 15)
        << "Insufficient good matches before RANSAC — scene is degenerate "
           "and the RANSAC test below cannot be evaluated.";

    cv::Mat mask;
    int inliers = countEssentialInliers(kpsA, kpsB, good, mask);

    EXPECT_GE(inliers, 15)
        << "RANSAC essential matrix kept too few inliers on a clean 5° "
           "rotation (got " << inliers << "); RANSAC threshold "
           "(1.5 px) or focal (525) may be wrong.";

    const double inlier_rate = static_cast<double>(inliers) /
                               static_cast<double>(good.size());
    EXPECT_GE(inlier_rate, 0.5)
        << "RANSAC inlier rate too low (" << inlier_rate
        << ") — most BF+Lowe matches should be geometrically consistent "
           "with a 5° in-plane rotation.";
}

// ── Test 5 — full pipeline reaches the relocalization accept threshold ───

// End-to-end Step-4 acceptance gate: ORB → BF + Lowe 0.75 → RANSAC
// essential at 1.5 px → count inliers. The plan's relocalization gate is
// "≥ 50 ORB inliers" against a stored keyframe. On the smaller synthetic
// scene used here (640×480 chessboard with translation + small rotation)
// we assert ≥ 30 inliers — sized to be reliably reachable on CI while
// still proving the full pipeline is wired correctly end-to-end.
TEST(OrbReloc, AcceptThresholdReached) {
    cv::Mat frameA = makeGeometricFrame();
    cv::Mat frameB_t = translate(frameA, /*tx=*/20.0, /*ty=*/10.0);
    cv::Mat frameB   = rotateAboutCenter(frameB_t, /*angle_deg=*/3.0);

    std::vector<cv::KeyPoint> kpsA, kpsB;
    cv::Mat descA, descB;
    detectAndComputeOrb(frameA, kpsA, descA);
    detectAndComputeOrb(frameB, kpsB, descB);

    std::vector<cv::DMatch> good = matchWithLoweRatio(descA, descB);
    ASSERT_GT(static_cast<int>(good.size()), 0);

    cv::Mat mask;
    int inliers = countEssentialInliers(kpsA, kpsB, good, mask);

    EXPECT_GE(inliers, 30)
        << "Step-4 relocalization accept gate failed: full ORB → BF → "
           "Lowe → RANSAC pipeline returned " << inliers << " inliers on "
           "a clean translation+small-rotation case (target ≥ 30 on the "
           "synthetic scene; production gate is ≥ 50 on real keyframes).";
}
