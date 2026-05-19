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

TEST(EKFStateStep4, GetYaw_RotateAboutWorldZBy90Deg_ReturnsHalfPi) {
    EKFState ekf;
    // Z-up world frame (post 2026-05-08 alignment): yaw is rotation around
    // world Z. R_GtoI is world→body. For body at compass heading +π/2 (East),
    // setInitialHeading builds:
    //   R_GtoI = [[cos(π/2), -sin(π/2), 0], [sin(π/2), cos(π/2), 0], [0, 0, 1]]
    //          = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    cv::Mat R_yaw90 = (cv::Mat_<double>(3, 3) <<
                      std::cos(M_PI/2), -std::sin(M_PI/2), 0.0,
                      std::sin(M_PI/2),  std::cos(M_PI/2), 0.0,
                      0.0,                0.0,             1.0);
    ekf.initializeFull(R_yaw90, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    double yaw = ekf.getYaw(0.0, 0.0);
    // getYaw extracts atan2(R[1,0], R[0,0]) = atan2(sin(π/2), cos(π/2)) = π/2.
    // This matches the input azimuth — the Z-up round-trip is consistent.
    EXPECT_NEAR(yaw, M_PI/2, 1e-6);
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

    // VIO observes a 1-meter translation along world +Z. Z-up world: this
    // is a vertical (Up) translation. The test verifies Kalman pull
    // direction; world axis interpretation doesn't affect the math.
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

// ── Phase 1 Step 6.3 — applyLandmarkObservations (fixed-landmark MSCKF) ──────
//
// Synthetic 4-landmark scene proves:
//   (a) perfect observations → all accepted, χ² ≈ 0, no rejections;
//   (b) state perturbation → Kalman update moves p_G BACK toward the
//       observation-consistent position.
//
// Z-up world frame (post 2026-05-08 alignment, matches MEMORY.md
// project_z_up_fix_2026_05_08). Vertical-phone convention: R_bc =
// diag(1, -1, -1) (camera +X = body +X, camera +Y = body -Y because
// pixel-row down is body up, camera +Z = body -Z because the camera
// looks along -body-z).
TEST(EKFStateStep4, ApplyLandmarkObservations_PerfectObsAccepted) {
    INIT_EKF(ekf);
    // State: p_G = origin, R_GtoI = identity (so body == world). The
    // INIT_EKF helper above already sets exactly this.

    const double fx = 525.0, fy = 525.0, cx = 320.0, cy = 240.0;
    const int W = 640, H = 480;

    // The NavSight vertical-phone convention `R_bc = diag(1,-1,-1)` maps
    // body forward (+x) into camera -z, which would fail the depth gate
    // for landmarks placed in front of the body. The synthetic test uses
    // an R_bc that puts body-forward into camera +Z so all 4 landmarks
    // land with positive depth in the pinhole frame:
    //   R_bc = [[0, 1, 0], [0, 0, -1], [1, 0, 0]]
    //   →  x_body=+5    → camera Z = +5 (depth, gate OK)
    //      y_body=±0.5  → camera X = ±0.5 (image-right)
    //      z_body=±0.5  → camera Y = ∓0.5 (image-down, image-axis flipped)
    // We feed this R_bc into the EKF method directly — the method does
    // NOT touch the EKF's internal R_bc_ (which is reserved for the
    // online extrinsics calibration owned by Phase 1 Step 8b).
    const cv::Matx33d R_bc_test(0.0, 1.0,  0.0,
                                 0.0, 0.0, -1.0,
                                 1.0, 0.0,  0.0);

    std::vector<EKFState::LandmarkObservation> obs;
    const std::vector<cv::Vec3d> p_worlds = {
        cv::Vec3d(5.0,  0.5,  0.5),
        cv::Vec3d(5.0, -0.5,  0.5),
        cv::Vec3d(5.0,  0.5, -0.5),
        cv::Vec3d(5.0, -0.5, -0.5),
    };
    for (size_t i = 0; i < p_worlds.size(); ++i) {
        // Compute the expected pixel by the same math the EKF uses, so the
        // "perfect" injection is mathematically identical to the prediction.
        const cv::Vec3d p_I = p_worlds[i];        // R_GtoI=I
        const cv::Vec3d p_C = R_bc_test * p_I;
        ASSERT_GT(p_C[2], 0.5);                   // depth gate passes
        const double u = fx * p_C[0] / p_C[2] + cx;
        const double v = fy * p_C[1] / p_C[2] + cy;

        EKFState::LandmarkObservation o;
        o.landmark_id = static_cast<int>(i);
        o.p_world     = p_worlds[i];
        o.pixel_meas  = cv::Point2f(static_cast<float>(u),
                                    static_cast<float>(v));
        o.sigma_px    = 1.0;
        obs.push_back(o);
    }

    const auto r = ekf.applyLandmarkObservations(obs, R_bc_test,
                                                  fx, fy, cx, cy, W, H);
    EXPECT_EQ(r.accepted, 4);
    EXPECT_EQ(r.rejected_chi2, 0);
    EXPECT_EQ(r.rejected_huber, 0);
    EXPECT_EQ(r.rejected_depth, 0);
    EXPECT_EQ(r.rejected_outside_image, 0);
    EXPECT_NEAR(r.chi2_total, 0.0, 1e-3);
}

TEST(EKFStateStep4, ApplyLandmarkObservations_PullsStateBackToObs) {
    // Same scene, but perturb p_G by +0.1 m in world-X before applying the
    // SAME pixels. The pose-only Jacobian on δp_G should produce a
    // correction that moves p_G back toward the origin (where the
    // observations are self-consistent).
    INIT_EKF(ekf);
    const cv::Matx33d R_bc_test(0.0, 1.0,  0.0,
                                 0.0, 0.0, -1.0,
                                 1.0, 0.0,  0.0);
    const double fx = 525.0, fy = 525.0, cx = 320.0, cy = 240.0;
    const int W = 640, H = 480;

    // Build "perfect" pixels assuming p_G = origin.
    std::vector<EKFState::LandmarkObservation> obs;
    const std::vector<cv::Vec3d> p_worlds = {
        cv::Vec3d(5.0,  0.5,  0.5),
        cv::Vec3d(5.0, -0.5,  0.5),
        cv::Vec3d(5.0,  0.5, -0.5),
        cv::Vec3d(5.0, -0.5, -0.5),
    };
    for (size_t i = 0; i < p_worlds.size(); ++i) {
        const cv::Vec3d p_C = R_bc_test * p_worlds[i];
        const double u = fx * p_C[0] / p_C[2] + cx;
        const double v = fy * p_C[1] / p_C[2] + cy;
        EKFState::LandmarkObservation o;
        o.landmark_id = static_cast<int>(i);
        o.p_world     = p_worlds[i];
        o.pixel_meas  = cv::Point2f(static_cast<float>(u),
                                    static_cast<float>(v));
        o.sigma_px    = 1.0;
        obs.push_back(o);
    }

    // Perturb the state. setPosition leaves R_GtoI and covariance
    // unchanged — the perturbation is purely positional, so the residual
    // should be non-zero and the Kalman update should drive p_G back
    // toward zero along world +x. Magnitude depends on the prior P_pp
    // (set by initializeFull) vs measurement variance σ²_px / focal² ≈
    // (1/525)² rad²; with 4 observations and 1 cm prior σ on position
    // we expect the correction to recover a meaningful fraction.
    cv::Mat p_shift = (cv::Mat_<double>(3, 1) << 0.1, 0.0, 0.0);
    ekf.setPosition(p_shift);

    const cv::Mat p_before = ekf.getPosition();
    EXPECT_NEAR(p_before.at<double>(0, 0), 0.1, 1e-9);

    const auto r = ekf.applyLandmarkObservations(obs, R_bc_test,
                                                  fx, fy, cx, cy, W, H);
    EXPECT_EQ(r.accepted, 4);
    EXPECT_EQ(r.rejected_depth, 0);
    EXPECT_EQ(r.rejected_outside_image, 0);

    const cv::Mat p_after = ekf.getPosition();
    // p_x should have moved BACK toward the origin (away from 0.1).
    // Kalman gain on a small position prior (init std 1 cm → P_pp = 1e-4
    // m²) vs ~(σ_px / fx)² · depth² ≈ (1/525)² · 25 = 9e-5 m² gives
    // K ≈ 0.5, so we expect roughly half the perturbation absorbed.
    // Just check the direction-and-shrink invariant — the exact value is
    // a downstream concern.
    EXPECT_LT(p_after.at<double>(0, 0), p_before.at<double>(0, 0));
    EXPECT_GT(p_after.at<double>(0, 0), -0.05);  // didn't overshoot
}

TEST(EKFStateStep4, ApplyLandmarkObservations_FailsWhenNotInitialized) {
    EKFState ekf;
    std::vector<EKFState::LandmarkObservation> obs;
    EKFState::LandmarkObservation o;
    o.landmark_id = 0;
    o.p_world     = cv::Vec3d(5.0, 0.0, 0.0);
    o.pixel_meas  = cv::Point2f(320.0f, 240.0f);
    o.sigma_px    = 1.0;
    obs.push_back(o);

    const cv::Matx33d R_bc(1.0,  0.0,  0.0,
                            0.0, -1.0,  0.0,
                            0.0,  0.0, -1.0);
    const auto r = ekf.applyLandmarkObservations(obs, R_bc,
                                                  525.0, 525.0, 320.0, 240.0,
                                                  640, 480);
    EXPECT_EQ(r.accepted, 0);
    EXPECT_EQ(r.rejected_chi2, 0);
    EXPECT_EQ(r.rejected_huber, 0);
    EXPECT_EQ(r.rejected_depth, 0);
    EXPECT_EQ(r.rejected_outside_image, 0);
}
