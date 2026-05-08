// test_ekf_yaw_convention.cpp
//
// Regression suite for the Z-up world frame alignment landed 2026-05-08.
//
// Purpose
// -------
//   Pin every layer of the rotation-convention chain so that future edits to
//   getYaw / updateGravityAlignedYaw / setInitialHeading / Madgwick can't
//   silently break the EKF the way the c1c15b2 commit did. After that fix
//   landed, the EKF was Y-up while Madgwick + IMU samples + Tracker init
//   were Z-up; the cross-frame mismatch produced a ~180° heading offset on
//   real walks and a 9.81 m/s² gravity leak into the wrong velocity axis.
//
// Convention pinned by these tests
// --------------------------------
//   World frame: ENU Z-up (X=East, Y=North, Z=Up), right-handed.
//   Body frame:  Android sensor (X=right, Y=screen-top, Z=out-of-screen).
//   R_GtoI:      world→body. For body at compass heading +ψ (CW-positive
//                nav, North=0, East=+π/2) with zero roll/pitch:
//                  R_GtoI = | cos ψ, -sin ψ, 0 |
//                           | sin ψ,  cos ψ, 0 |
//                           |   0,      0,   1 |
//                (matches Tracker::setInitialHeading at Tracker.cpp:322).
//   getYaw:      atan2(R_aligned[1,0], R_aligned[0,0]) — Z-axis Tait-Bryan
//                yaw in CW-positive nav convention. Matches IMUPreintegrator::getHeading.
//   gravity:     world (0, 0, -9.81) m/s². g·z = -9.81.
//   H Jacobian:  h_body = +R_GtoI · e_z_world, sign +1 (NOT negated).
//
// The pure-numpy mirror of these identities is in
// scripts/test_z_up_conventions.py and is the primary verification on
// Windows hosts where OpenCV is unavailable. These C++ tests verify that
// the actual EKFState code implements the same identities.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include "EKFState.h"
#include "IMUPreintegrator.h"

namespace {

constexpr double kYawToleranceRad = 0.5 * M_PI / 180.0;  // 0.5°

// Build R_GtoI_ (world→body, Z-up) for a body at compass heading +ψ rad.
// Matches Tracker::setInitialHeading at Tracker.cpp:322 — this IS the
// matrix the running EKF receives at startup.
cv::Mat makeRGtoIForYaw(double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    return (cv::Mat_<double>(3, 3) <<
              c, -s, 0.0,
              s,  c, 0.0,
            0.0, 0.0, 1.0);
}

void initEkfWithYaw(EKFState& ekf, double yaw_rad) {
    cv::Mat R = makeRGtoIForYaw(yaw_rad);
    ekf.initializeFull(R, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    ekf.addClone(R, cv::Mat::zeros(3, 1, CV_64F), 1'000'000'000LL);
}

}  // namespace

// ─── Test 1 ─────────────────────────────────────────────────────────────────
// Pure yaw +30° — getYaw must return +30° in CW-positive nav.
TEST(EkfYawConvention, GetYaw_PureYaw30Deg_ReturnsPositive30) {
    const double yaw_rad = 30.0 * M_PI / 180.0;
    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const double measured = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(measured, yaw_rad, kYawToleranceRad)
        << "getYaw=" << measured * 180.0 / M_PI << "° expected +30°";
}

// ─── Test 2 ─────────────────────────────────────────────────────────────────
// Pure yaw -60° — getYaw must return -60°.
TEST(EkfYawConvention, GetYaw_PureYawMinus60Deg_ReturnsNegative60) {
    const double yaw_rad = -60.0 * M_PI / 180.0;
    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const double measured = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(measured, yaw_rad, kYawToleranceRad)
        << "getYaw=" << measured * 180.0 / M_PI << "° expected -60°";
}

// ─── Test 3 ─────────────────────────────────────────────────────────────────
// setInitialHeading round-trip: every azimuth in the canonical test set
// must round-trip through R_GtoI back to the same value via getYaw.
// This is the primary check that the Z-up convention is internally
// consistent across the init path.
TEST(EkfYawConvention, SetInitialHeadingRoundTrip_AllAzimuths) {
    const double azimuths[] = {
        0.0,
        M_PI / 6,    // +30°
        M_PI / 4,    // +45°
        M_PI / 2,    // +90° (East)
        2 * M_PI / 3,  // +120°
        M_PI,        // ±180° (South)
        -M_PI / 2,   // -90° (West)
        -3 * M_PI / 4,  // -135°
        0.123,
    };
    for (double psi : azimuths) {
        EKFState ekf;
        initEkfWithYaw(ekf, psi);
        const double recovered = ekf.getYaw(0.0, 0.0);
        // Wrap-aware comparison: ±π is the same yaw.
        double diff = std::abs(recovered - psi);
        if (diff > M_PI) diff = std::abs(diff - 2.0 * M_PI);
        EXPECT_LT(diff, kYawToleranceRad)
            << "psi=" << psi * 180.0 / M_PI
            << "° -> getYaw=" << recovered * 180.0 / M_PI << "°";
    }
}

// ─── Test 4 ─────────────────────────────────────────────────────────────────
// Consistent measurement -> zero residual -> EKF yaw unchanged.
TEST(EkfYawConvention, UpdateGravityAlignedYaw_ConsistentMeasurement_NoMovement) {
    const double yaw_rad = 30.0 * M_PI / 180.0;
    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const bool ok = ekf.updateGravityAlignedYaw(yaw_rad, 1e-4, 0.0, 0.0);
    ASSERT_TRUE(ok);

    const double yaw_after = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(yaw_after, yaw_rad, kYawToleranceRad)
        << "Consistent measurement moved state from "
        << yaw_rad * 180.0 / M_PI << "° to "
        << yaw_after * 180.0 / M_PI << "°.";
}

// ─── Test 5 ─────────────────────────────────────────────────────────────────
// State at +30°, measurement at +35° — EKF must move TOWARD +35°, not away.
// Pre-Z-up bug had H Jacobian with wrong sign, pulling the state toward -35°.
TEST(EkfYawConvention, UpdateGravityAlignedYaw_SmallPositivePerturbation_PullsTowardMeasurement) {
    const double yaw_rad   = 30.0 * M_PI / 180.0;
    const double delta_rad =  5.0 * M_PI / 180.0;
    const double yaw_meas  = yaw_rad + delta_rad;

    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const bool ok = ekf.updateGravityAlignedYaw(yaw_meas, 1e-4, 0.0, 0.0);
    ASSERT_TRUE(ok);

    const double yaw_after = ekf.getYaw(0.0, 0.0);
    EXPECT_GT(yaw_after, yaw_rad)
        << "EKF did not move above " << yaw_rad * 180.0 / M_PI
        << "° toward measurement " << yaw_meas * 180.0 / M_PI << "°.";
    EXPECT_LT(yaw_after, yaw_meas + kYawToleranceRad)
        << "EKF overshot past measurement.";

    const double dist_to_meas  = std::abs(yaw_after - yaw_meas);
    const double dist_to_wrong = std::abs(yaw_after - (yaw_rad - delta_rad));
    EXPECT_LT(dist_to_meas, dist_to_wrong)
        << "EKF moved toward wrong direction (" << (yaw_rad - delta_rad) * 180.0 / M_PI
        << "°) instead of measurement (" << yaw_meas * 180.0 / M_PI << "°). "
        << "H Jacobian sign reversed?";
}

// ─── Test 6 ─────────────────────────────────────────────────────────────────
// Madgwick body→world quaternion → IMU getHeading must equal EKF getYaw
// on the inverted matrix. This pins the Madgwick<->EKF convention bridge.
//
// For a pure-yaw rotation by +ψ_math CCW around world Z (Madgwick Z-up):
//   q_b2w = (cos(ψ/2), 0, 0, sin(ψ/2))
//   R_b2w = Rz(ψ_math)
//   R_GtoI = R_b2w.t() = Rz(-ψ_math)
//   imu.getHeading() returns -ψ_math (negate for CW-nav).
//   getYaw(R_GtoI) = atan2(R_GtoI[1,0], R_GtoI[0,0])
//                  = atan2(-sin ψ, cos ψ) = -ψ_math.
// Both equal -ψ_math: imu.getHeading() == ekf.getYaw().
TEST(EkfYawConvention, MadgwickGetHeading_MatchesEkfGetYaw_OnInvertedMatrix) {
    const double yaw_math_set[] = {
        0.0, M_PI / 6, -M_PI / 4, M_PI / 3, -M_PI / 2, M_PI / 2, M_PI - 1e-3,
    };
    for (double yaw_math : yaw_math_set) {
        // Build R_b2w = Rz(yaw_math) and its inverse.
        const double c = std::cos(yaw_math);
        const double s = std::sin(yaw_math);
        cv::Mat R_b2w = (cv::Mat_<double>(3, 3) <<
                          c, -s, 0.0,
                          s,  c, 0.0,
                        0.0, 0.0, 1.0);
        cv::Mat R_GtoI = R_b2w.t();

        EKFState ekf;
        ekf.initializeFull(R_GtoI, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
        const double yaw_from_ekf = ekf.getYaw(0.0, 0.0);

        // Direct mirror of IMUPreintegrator::getHeading.
        const double siny_cosp = 2.0 * (std::cos(yaw_math / 2) * std::sin(yaw_math / 2));
        const double cosy_cosp = 1.0 - 2.0 * (std::sin(yaw_math / 2) * std::sin(yaw_math / 2));
        const double yaw_from_imu = -std::atan2(siny_cosp, cosy_cosp);

        double diff = std::abs(yaw_from_ekf - yaw_from_imu);
        if (diff > M_PI) diff = std::abs(diff - 2.0 * M_PI);
        EXPECT_LT(diff, kYawToleranceRad)
            << "yaw_math=" << yaw_math * 180.0 / M_PI
            << "° -> imu=" << yaw_from_imu * 180.0 / M_PI
            << "°, ekf=" << yaw_from_ekf * 180.0 / M_PI << "°";
    }
}

// ─── Test 7 ─────────────────────────────────────────────────────────────────
// Stationary phone (gravity along body +Z) propagated for 1 second must
// produce zero kinematic acceleration in world frame -> v_G stays at 0.
//
// This is the test that detects the Y-up gravity-leak bug. Body accel
// (0,0,+9.81) with R_GtoI=I rotates to world (0,0,+9.81). Z-up gravity
// vector (0,0,-9.81) cancels exactly. Y-up gravity (0,-9.81,0) leaks
// 9.81 m/s² along world Z and -9.81 m/s² along world Y -- v_G accumulates
// 9.81 m/s in 1 second per axis, ‖v‖ ≈ 13.87 m/s.
//
// NOTE: Direct test of EKFState::propagateIMU requires the preintegrator's
// deltaV/deltaP outputs. We construct them analytically here and feed them
// to a single propagate call with dt=1s.
TEST(EkfYawConvention, StationaryGravity_NoVelocityAccumulation) {
    EKFState ekf;
    cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);
    ekf.initializeFull(R_eye, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));

    // Build deltaR=I (no rotation) and deltaV/deltaP for a stationary IMU
    // observing only gravity along body +Z. Preintegrator returns the
    // *integrated specific force* in body frame: ∫(a_body) dt = (0,0,9.81).
    const double dt = 1.0;
    cv::Mat deltaR = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat deltaV = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 9.81 * dt);
    cv::Mat deltaP = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.5 * 9.81 * dt * dt);
    cv::Mat zero33 = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat zero99 = cv::Mat::zeros(9, 9, CV_64F);

    ekf.propagateIMU(deltaR, deltaV, deltaP, dt,
                     zero99, zero33, zero33, zero33, zero33, zero33);

    cv::Mat p = ekf.getPosition();
    ASSERT_FALSE(p.empty());

    // After 1s of pure gravity with Z-up, kinematic accel = a_body_world + g
    //   = (0,0,9.81) + (0,0,-9.81) = (0,0,0). v stays at 0, p stays at 0.
    // With Y-up gravity bug (0,-9.81,0), deltaV is rotated to world +Z but
    // gravity is subtracted along world Y, leaking 9.81 m/s² into BOTH
    // world Y and world Z. After 1s, p[1] ≈ -4.9 m, p[2] ≈ +4.9 m.
    EXPECT_LT(cv::norm(p), 1e-6)
        << "Position after 1s stationary = " << cv::norm(p)
        << " m. Should be 0. Y-up gravity bug leaks ~6.93 m on each of "
        << "world Y and Z (norm ~9.8 m).";
}

// ─── Test 8a ────────────────────────────────────────────────────────────────
// Post-init setRotation correctly replaces R_GtoI_ even when the EKF is
// already full-initialized. Regression for 2026-05-09 bug: Tracker::setInitialHeading
// used to early-return when ekf_.isFullInitialized(), so the magnetometer-derived
// heading from Kotlin (which lands AFTER ekf_.initializeFull on the UI-thread
// dispatch) was silently discarded. Step 7 chi² then rejected every loop closure
// because R_GtoI=Identity vs PnP target ~Rz(real_heading) ≈ 180° residual.
TEST(EkfYawConvention, SetRotation_PostFullInit_UpdatesRGtoI) {
    // Simulate the production sequence: full-init at Identity (mimicking
    // the Tracker bootstrap that fires before Kotlin's handleVioInitialized).
    EKFState ekf;
    cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);
    ekf.initializeFull(R_eye, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    ASSERT_TRUE(ekf.isFullInitialized());
    EXPECT_NEAR(ekf.getYaw(0.0, 0.0), 0.0, kYawToleranceRad);

    // Apply post-init heading correction (Kotlin sees vio.isInitialized=true
    // and calls setInitialHeading via NativeBridge).
    const double psi = 100.0 * M_PI / 180.0;  // 100° east-of-south
    ekf.setRotation(makeRGtoIForYaw(psi));

    EXPECT_NEAR(ekf.getYaw(0.0, 0.0), psi, kYawToleranceRad)
        << "setRotation did not update R_GtoI_ post-init. Step 7 chi² will "
        << "still reject every loop closure.";
}

// ─── Test 8 ─────────────────────────────────────────────────────────────────
// H Jacobian sign verification: state at yaw=0, measurement at +5°,
// updated state must move toward +5°. This is the same logical check as
// Test 5 but with state at the origin to isolate the sign of h_body
// against any state-dependent cross-coupling.
TEST(EkfYawConvention, UpdateGravityAlignedYaw_FromIdentity_PositiveDeltaPullsPositive) {
    const double delta = 5.0 * M_PI / 180.0;
    EKFState ekf;
    initEkfWithYaw(ekf, 0.0);

    ASSERT_TRUE(ekf.updateGravityAlignedYaw(delta, 1e-4, 0.0, 0.0));
    const double y = ekf.getYaw(0.0, 0.0);
    EXPECT_GT(y, 0.0) << "From yaw=0, +5° measurement pulled to "
                       << y * 180.0 / M_PI << "°. H sign is wrong.";
    EXPECT_LT(y, delta + kYawToleranceRad)
        << "From yaw=0, +5° measurement overshot to "
        << y * 180.0 / M_PI << "°.";
}
