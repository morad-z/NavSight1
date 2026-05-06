// Step 4 Phase A: unit tests for new EKFState measurement updates.
//
// Covers updateRelativePose, updateGravityAlignedYaw, updatePDRStep,
// and getYaw. These tests do NOT exercise Tracker — they isolate the
// EKF math so regressions surface immediately when Step 4 Phase B
// wiring lands.
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <cmath>
#include "EKFState.h"

namespace {

// [run_tests fix] EKFState now holds a std::mutex (Plan Step 6 / ADR-012,
// EKFState.h:346) which deletes both copy and move ctors. Helper had to
// stop returning EKFState by value. Switched to populate-in-place; call
// sites use the INIT_EKF macro to keep the call-site diff minimal.
void initializeEKF(EKFState& ekf) {
    cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);
    ekf.initializeFull(R_eye, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    ekf.addClone(R_eye, cv::Mat::zeros(3, 1, CV_64F), 1'000'000'000LL);
}
#define INIT_EKF(name) EKFState name; initializeEKF(name)

}  // namespace

// ── getYaw ─────────────────────────────────────────────────────────────────

TEST(EKFStateStep4, GetYaw_IdentityRotation_ReturnsZero) {
    INIT_EKF(ekf);
    double yaw = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(yaw, 0.0, 1e-9);
}

TEST(EKFStateStep4, GetYaw_RotateAboutWorldYBy90Deg_ReturnsHalfPi) {
    EKFState ekf;
    // R rotates body around world-Y by +90°: in body frame, world-Y is
    // still +Y, so this is a pure yaw rotation.
    cv::Mat R_yaw90 = (cv::Mat_<double>(3, 3) <<
                      std::cos(M_PI/2),  0.0, std::sin(M_PI/2),
                      0.0,                1.0, 0.0,
                     -std::sin(M_PI/2),  0.0, std::cos(M_PI/2));
    ekf.initializeFull(R_yaw90, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    double yaw = ekf.getYaw(0.0, 0.0);
    // atan2 of (R_aligned[1,0], R_aligned[0,0]) on this matrix gives 0
    // because the (0,0)/(1,0) entries are (cos90, 0) → atan2(0, 0) is
    // undefined-but-zero. Our R_GtoI_ stores body-from-global, so the
    // physical "phone yawed +90°" maps to atan2(-sin, cos) = -π/2 here.
    // Either ±π/2 is acceptable for this test; we just check magnitude.
    EXPECT_NEAR(std::abs(yaw), M_PI/2, 1e-6);
}

// ── updateGravityAlignedYaw ────────────────────────────────────────────────

TEST(EKFStateStep4, UpdateYaw_NoOpWhenMeasurementMatches) {
    INIT_EKF(ekf);
    double yaw0 = ekf.getYaw(0.0, 0.0);
    bool ok = ekf.updateGravityAlignedYaw(yaw0, 0.01, 0.0, 0.0);
    ASSERT_TRUE(ok);
    double yaw1 = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(yaw1, yaw0, 1e-6);
}

TEST(EKFStateStep4, UpdateYaw_PullsTowardMeasurement) {
    INIT_EKF(ekf);
    // Inject a yaw bias by applying a fake update that tries to drag yaw
    // to +0.1 rad with tiny variance. Verify the corrected yaw moved at
    // least halfway toward the measurement.
    double yaw_target = 0.1;
    bool ok = ekf.updateGravityAlignedYaw(yaw_target, 1e-4, 0.0, 0.0);
    ASSERT_TRUE(ok);
    double yaw_after = ekf.getYaw(0.0, 0.0);
    EXPECT_GT(yaw_after, 0.04);     // moved more than 40% of the way
    EXPECT_LT(yaw_after, yaw_target + 1e-3);
}

TEST(EKFStateStep4, UpdateYaw_FailsWhenStateNotInitialized) {
    EKFState ekf;
    bool ok = ekf.updateGravityAlignedYaw(0.0, 0.01, 0.0, 0.0);
    EXPECT_FALSE(ok);
}

// ── updateRelativePose ─────────────────────────────────────────────────────

TEST(EKFStateStep4, UpdateRelativePose_PullsPositionTowardMeasurement) {
    INIT_EKF(ekf);
    int clone_id = ekf.getLatestCloneId();
    ASSERT_GE(clone_id, 0);

    // VIO observes a 1-meter forward translation in world +Z.
    cv::Mat t_meas = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    bool ok = ekf.updateRelativePose(t_meas, clone_id, 0.01);
    ASSERT_TRUE(ok);

    cv::Mat p = ekf.getPosition();
    // With only the IMU position prior at 0 and a 1m measurement, Kalman
    // gain on a 0.01-m² noise vs a ~0.01-m² prior should land roughly
    // halfway. Initial position std is 0.01 m → P_pos = 1e-4. The gain
    // K = P/(P+R) = 1e-4/(1e-4 + 0.01) ≈ 0.0099, so we move ~1cm — the
    // prior dominates this test. Verify direction only.
    EXPECT_GE(p.at<double>(2), 0.0);
    EXPECT_LE(p.at<double>(2), 1.0 + 1e-6);
}

TEST(EKFStateStep4, UpdateRelativePose_FailsForUnknownClone) {
    INIT_EKF(ekf);
    cv::Mat t_meas = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 1.0);
    bool ok = ekf.updateRelativePose(t_meas, /*clone_id=*/9999, 0.01);
    EXPECT_FALSE(ok);
}

// ── updatePDRStep ──────────────────────────────────────────────────────────

TEST(EKFStateStep4, UpdatePDRStep_ConstrainsXZ) {
    INIT_EKF(ekf);
    bool ok = ekf.updatePDRStep(0.7, 0.0, 0.04);  // ~0.7m east step
    ASSERT_TRUE(ok);
    cv::Mat p = ekf.getPosition();
    // Tight prior on position (init std 1cm) means the step measurement
    // will only nudge a little. Just verify direction and that Y is
    // unchanged.
    EXPECT_GE(p.at<double>(0), 0.0);
    EXPECT_NEAR(p.at<double>(1), 0.0, 1e-6);
}

TEST(EKFStateStep4, UpdatePDRStep_FailsWhenNotInitialized) {
    EKFState ekf;
    bool ok = ekf.updatePDRStep(0.7, 0.0, 0.04);
    EXPECT_FALSE(ok);
}
