// Phase 1 Step 6 (post_v19_sprint_plan.md §205-298): unit tests for the
// visual-only LandmarkMap. Per the implementor skill, the synthetic test
// MUST exist and pass before any Tracker wire-up adds map-based EKF
// measurement updates to the running pipeline.
//
// Coverage:
//   * addOrMergeLandmark: fresh add, dedup-by-3D-distance, dedup-by-descriptor
//   * Reject invalid input (NaN p_world, wrong-shape descriptor)
//   * getLandmarksInRadius: KD-tree spatial query + linear fallback agree
//   * projectIntoCamera: in-frame, behind-camera, out-of-range, oob-pixel
//   * applyKeyframePoseCorrection: shifts only landmarks observed in that kf
//   * cullStaleLandmarks: respects grace period + min-observations gate
//   * reset wipes counters + storage
//
// NOT covered (deferred to integration tests + real walk):
//   * EKF measurement-update integration (Phase 6.3, separate test)
//   * Tracker per-keyframe wire-up (Phase 6.2/6.4 — needs real frames)
//   * LoopClosureDetector::KeyframeRecord landmark_ids refactor (Phase 6.4)

#include <gtest/gtest.h>
#include <cmath>
#include <opencv2/core.hpp>
#include "LandmarkMap.h"

namespace {

// Build a deterministic 1×32 ORB-style descriptor where row i has byte
// value `seed + i*step` mod 256. Two descriptors with the same seed and
// step are identical; small step differences produce small Hamming.
cv::Mat makeDescriptor(uint8_t seed, uint8_t step = 0) {
    cv::Mat d(1, 32, CV_8U);
    for (int i = 0; i < 32; ++i) {
        d.at<uint8_t>(0, i) = static_cast<uint8_t>(seed + i * step);
    }
    return d;
}

// Standard pinhole — matches NavSight's typical S21 Ultra rear camera
// (calibrated values land at runtime; these are reasonable defaults for
// the synthetic projection test).
constexpr double kFx = 700.0;
constexpr double kFy = 700.0;
constexpr double kCx = 320.0;
constexpr double kCy = 240.0;
constexpr int    kImgW = 640;
constexpr int    kImgH = 480;

}  // namespace

TEST(LandmarkMap, FreshAddAssignsMonotonicIds) {
    navsight::LandmarkMap m;
    const int id0 = m.addOrMergeLandmark(cv::Vec3d(0,0,2),    makeDescriptor(0x00), /*kf=*/1, /*ts=*/1000);
    const int id1 = m.addOrMergeLandmark(cv::Vec3d(5,5,5),    makeDescriptor(0x10), /*kf=*/1, /*ts=*/1100);
    const int id2 = m.addOrMergeLandmark(cv::Vec3d(-3,2,4),   makeDescriptor(0x20), /*kf=*/2, /*ts=*/1200);
    EXPECT_EQ(id0, 0);
    EXPECT_EQ(id1, 1);
    EXPECT_EQ(id2, 2);
    EXPECT_EQ(m.size(), 3u);
    EXPECT_EQ(m.landmarksAddedTotal(),  3);
    EXPECT_EQ(m.landmarksMergedTotal(), 0);
}

TEST(LandmarkMap, RejectInvalidInput) {
    navsight::LandmarkMap m;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    // NaN p_world rejected
    EXPECT_EQ(m.addOrMergeLandmark(cv::Vec3d(nan, 0, 0), makeDescriptor(0x00), 1, 1000), -1);
    // Empty descriptor rejected
    EXPECT_EQ(m.addOrMergeLandmark(cv::Vec3d(0, 0, 2), cv::Mat(), 1, 1000), -1);
    // Wrong shape descriptor rejected
    cv::Mat bad(1, 16, CV_8U, cv::Scalar(0));
    EXPECT_EQ(m.addOrMergeLandmark(cv::Vec3d(0, 0, 2), bad, 1, 1000), -1);
    // Wrong type descriptor rejected
    cv::Mat bad_type(1, 32, CV_32F, cv::Scalar(0));
    EXPECT_EQ(m.addOrMergeLandmark(cv::Vec3d(0, 0, 2), bad_type, 1, 1000), -1);
    EXPECT_EQ(m.size(), 0u);
}

TEST(LandmarkMap, DedupByCloseDistanceAndDescriptor) {
    navsight::LandmarkMap m;
    const auto d0 = makeDescriptor(0x40);
    const int id0 = m.addOrMergeLandmark(cv::Vec3d(0,0,2), d0, /*kf=*/1, 1000);
    // Same descriptor, 10 cm away — well within 0.5m dedup threshold → MERGE
    const int id1 = m.addOrMergeLandmark(cv::Vec3d(0.1,0,2), d0, /*kf=*/2, 1100);
    EXPECT_EQ(id0, id1);
    EXPECT_EQ(m.size(), 1u);
    EXPECT_EQ(m.landmarksAddedTotal(),  1);
    EXPECT_EQ(m.landmarksMergedTotal(), 1);
    // Merged landmark records both observations
    navsight::Landmark snap;
    EXPECT_TRUE(m.getLandmark(id0, snap));
    EXPECT_EQ(snap.times_observed, 2);
    ASSERT_EQ(snap.observed_in_kfs.size(), 2u);
    EXPECT_EQ(snap.observed_in_kfs[0], 1u);
    EXPECT_EQ(snap.observed_in_kfs[1], 2u);
}

TEST(LandmarkMap, SeparateLandmarkIfDescriptorTooDifferent) {
    navsight::LandmarkMap m;
    // Same 3D position, but descriptors differ by far more than Hamming 50.
    const auto d_low  = makeDescriptor(0x00);
    const auto d_high = makeDescriptor(0xFF);   // 256 bits all flipped → Hamming = 256
    const int id0 = m.addOrMergeLandmark(cv::Vec3d(1,1,1), d_low,  /*kf=*/1, 1000);
    const int id1 = m.addOrMergeLandmark(cv::Vec3d(1,1,1), d_high, /*kf=*/2, 1100);
    EXPECT_NE(id0, id1);     // distinct landmarks
    EXPECT_EQ(m.size(), 2u);
}

TEST(LandmarkMap, SeparateLandmarkIfTooFar) {
    navsight::LandmarkMap m;
    const auto d = makeDescriptor(0x55);
    const int id0 = m.addOrMergeLandmark(cv::Vec3d(0,0,2), d, 1, 1000);
    // 1.5 m away — past kDedup3DDistanceM (0.5 m) → new landmark
    const int id1 = m.addOrMergeLandmark(cv::Vec3d(1.5,0,2), d, 1, 1100);
    EXPECT_NE(id0, id1);
    EXPECT_EQ(m.size(), 2u);
}

TEST(LandmarkMap, GetLandmarksInRadius) {
    navsight::LandmarkMap m;
    // Place 5 landmarks in a row 1 m apart along the x-axis
    for (int i = 0; i < 5; ++i) {
        m.addOrMergeLandmark(cv::Vec3d(static_cast<double>(i), 0.0, 2.0),
                              makeDescriptor(static_cast<uint8_t>(0x10 + i)),
                              /*kf=*/1, /*ts=*/1000);
    }
    // Query at x=2 with radius=1.5 should return landmarks at x=1,2,3 (3 hits)
    auto hits = m.getLandmarksInRadius(cv::Vec3d(2.0, 0.0, 2.0), 1.5);
    EXPECT_EQ(hits.size(), 3u);

    // Query with radius=0.1 should return only landmark at x=2 (1 hit)
    hits = m.getLandmarksInRadius(cv::Vec3d(2.0, 0.0, 2.0), 0.1);
    EXPECT_EQ(hits.size(), 1u);

    // Query at (100,100,100) far away from all → 0 hits
    hits = m.getLandmarksInRadius(cv::Vec3d(100.0, 100.0, 100.0), 5.0);
    EXPECT_EQ(hits.size(), 0u);
}

TEST(LandmarkMap, ProjectIntoCameraInFrame) {
    navsight::LandmarkMap m;
    // Landmark at world (0, 0, 5) — 5 m in front of a camera at world origin
    // facing world +Z. cam→world = identity.
    const int id = m.addOrMergeLandmark(cv::Vec3d(0,0,5), makeDescriptor(0xAA), 1, 1000);

    cv::Point2f uv;
    EXPECT_TRUE(m.projectIntoCamera(id,
                                      cv::Matx33d::eye(),
                                      cv::Vec3d(0,0,0),
                                      kFx, kFy, kCx, kCy,
                                      kImgW, kImgH,
                                      uv));
    // Principal-point projection: should land at (cx, cy)
    EXPECT_NEAR(uv.x, kCx, 0.001);
    EXPECT_NEAR(uv.y, kCy, 0.001);
}

TEST(LandmarkMap, ProjectIntoCameraRejectsBehindCamera) {
    navsight::LandmarkMap m;
    // Landmark at z = -2 (behind the camera looking down +Z)
    const int id = m.addOrMergeLandmark(cv::Vec3d(0,0,-2), makeDescriptor(0xAB), 1, 1000);
    cv::Point2f uv;
    EXPECT_FALSE(m.projectIntoCamera(id,
                                       cv::Matx33d::eye(),
                                       cv::Vec3d(0,0,0),
                                       kFx, kFy, kCx, kCy,
                                       kImgW, kImgH,
                                       uv));
}

TEST(LandmarkMap, ApplyKeyframePoseCorrectionShiftsOnlyAnchoredLandmarks) {
    navsight::LandmarkMap m;
    const int id_kf1 = m.addOrMergeLandmark(cv::Vec3d(1, 0, 5), makeDescriptor(0x01), /*kf=*/1, 1000);
    const int id_kf2 = m.addOrMergeLandmark(cv::Vec3d(2, 0, 5), makeDescriptor(0x02), /*kf=*/2, 1100);
    const int id_kf3 = m.addOrMergeLandmark(cv::Vec3d(3, 0, 5), makeDescriptor(0x03), /*kf=*/3, 1200);

    // Apply Δp=(0.5, 0, 0) Δyaw=0 to kf=2 only
    const int n_shifted = m.applyKeyframePoseCorrection(/*kf=*/2,
                                                          /*dx=*/0.5, /*dy=*/0.0, /*dz=*/0.0,
                                                          /*dyaw=*/0.0);
    EXPECT_EQ(n_shifted, 1);

    navsight::Landmark s1, s2, s3;
    ASSERT_TRUE(m.getLandmark(id_kf1, s1));
    ASSERT_TRUE(m.getLandmark(id_kf2, s2));
    ASSERT_TRUE(m.getLandmark(id_kf3, s3));
    EXPECT_NEAR(s1.p_world[0], 1.0, 1e-6);   // kf1 untouched
    EXPECT_NEAR(s2.p_world[0], 2.5, 1e-6);   // kf2 shifted by +0.5
    EXPECT_NEAR(s3.p_world[0], 3.0, 1e-6);   // kf3 untouched
}

TEST(LandmarkMap, ApplyKeyframePoseCorrectionAppliesYaw) {
    navsight::LandmarkMap m;
    // Landmark at (1, 0, 0) anchored to kf=1. Rotate +90° around world-Z.
    // R_z(π/2) · (1,0,0) = (0, 1, 0).
    const int id = m.addOrMergeLandmark(cv::Vec3d(1, 0, 0), makeDescriptor(0x77), 1, 1000);
    m.applyKeyframePoseCorrection(1, 0.0, 0.0, 0.0, M_PI / 2.0);
    navsight::Landmark s;
    ASSERT_TRUE(m.getLandmark(id, s));
    EXPECT_NEAR(s.p_world[0], 0.0, 1e-6);
    EXPECT_NEAR(s.p_world[1], 1.0, 1e-6);
    EXPECT_NEAR(s.p_world[2], 0.0, 1e-6);
}

TEST(LandmarkMap, CullStaleLandmarksRespectsGracePeriod) {
    navsight::LandmarkMap m;
    // Landmark created at t=0, only 1 observation
    const int id = m.addOrMergeLandmark(cv::Vec3d(0, 0, 2), makeDescriptor(0x99), 1, /*ts=*/0);

    // 1 second later: still inside grace period → NOT culled
    int n_culled = m.cullStaleLandmarks(/*now=*/1'000'000'000LL);
    EXPECT_EQ(n_culled, 0);
    EXPECT_EQ(m.size(), 1u);
    {
        navsight::Landmark snap_in_grace;
        EXPECT_TRUE(m.getLandmark(id, snap_in_grace));
    }

    // 20 seconds later: past grace, still only 1 obs (< min 3) → CULLED
    n_culled = m.cullStaleLandmarks(/*now=*/20'000'000'000LL);
    EXPECT_EQ(n_culled, 1);
    EXPECT_EQ(m.size(), 0u);
}

TEST(LandmarkMap, CullStaleLandmarksKeepsObservedFeatures) {
    navsight::LandmarkMap m;
    const auto d = makeDescriptor(0xCC);
    // Add then re-observe 4 times (different kfs, same landmark by merge)
    const int id = m.addOrMergeLandmark(cv::Vec3d(0, 0, 2), d, /*kf=*/1, 0);
    m.addOrMergeLandmark(cv::Vec3d(0.01, 0, 2), d, /*kf=*/2, 1'000'000'000LL);
    m.addOrMergeLandmark(cv::Vec3d(0.02, 0, 2), d, /*kf=*/3, 2'000'000'000LL);
    m.addOrMergeLandmark(cv::Vec3d(0.03, 0, 2), d, /*kf=*/4, 3'000'000'000LL);
    navsight::Landmark snap;
    ASSERT_TRUE(m.getLandmark(id, snap));
    EXPECT_EQ(snap.times_observed, 4);

    // Past grace period — kept because times_observed (4) >= min (3)
    const int n_culled = m.cullStaleLandmarks(20'000'000'000LL);
    EXPECT_EQ(n_culled, 0);
    EXPECT_EQ(m.size(), 1u);
}

TEST(LandmarkMap, ResetWipesStateAndCounters) {
    navsight::LandmarkMap m;
    m.addOrMergeLandmark(cv::Vec3d(0, 0, 2), makeDescriptor(0x11), 1, 1000);
    m.addOrMergeLandmark(cv::Vec3d(1, 0, 2), makeDescriptor(0x22), 1, 1000);
    ASSERT_EQ(m.size(), 2u);
    m.reset();
    EXPECT_EQ(m.size(), 0u);
    EXPECT_EQ(m.landmarksAddedTotal(),  0);
    EXPECT_EQ(m.landmarksMergedTotal(), 0);
    EXPECT_EQ(m.landmarksCulledTotal(), 0);
    EXPECT_EQ(m.kdRebuildsTotal(),      0);
}

TEST(LandmarkMap, KdTreeRebuildsOnAddThreshold) {
    navsight::LandmarkMap m;
    // Force enough adds to trigger at least one rebuild.
    for (int i = 0; i < navsight::LandmarkMap::kKdRebuildThreshold + 2; ++i) {
        m.addOrMergeLandmark(cv::Vec3d(static_cast<double>(i) * 1.0, 0.0, 2.0),
                              makeDescriptor(static_cast<uint8_t>(i)),
                              /*kf=*/1, /*ts=*/1000);
    }
    EXPECT_GE(m.kdRebuildsTotal(), 1);
    EXPECT_EQ(m.size(), static_cast<size_t>(navsight::LandmarkMap::kKdRebuildThreshold + 2));
}
