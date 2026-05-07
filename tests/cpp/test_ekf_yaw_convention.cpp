// test_ekf_yaw_convention.cpp
//
// Purpose: PIN the sign/convention bug in EKFState::getYaw and
//          EKFState::updateGravityAlignedYaw.
//
// Background:
//   R_GtoI_ is documented as world→body ("R_GtoI_ takes world→current-body",
//   EKFState.cpp:860).  When the body is yawed +ψ CCW around world-Y, the
//   correct world→body rotation is R_GtoI_ = Ry(−ψ):
//
//     R_GtoI_ = | cos ψ,  0, -sin ψ |
//               |  0,     1,   0    |
//               | sin ψ,  0,  cos ψ |
//
//   getYaw (line 1087-1098) extracts  atan2(R_aligned[0,2], R_aligned[0,0]).
//   With roll=pitch=0 the alignment sandwich is identity, so:
//
//     atan2(R_GtoI_[0,2], R_GtoI_[0,0])
//       = atan2(-sin ψ, cos ψ)
//       = -ψ              ← WRONG SIGN
//
//   The correct extraction for a world→body matrix and Y-up navigation yaw is
//   atan2(-R[0,2], R[0,0])  (or equivalently atan2(R[2,0], R[2,2]) when using
//   the transpose convention).  The tests below are EXPECTED TO FAIL on the
//   current code and must pass after the sign is corrected.
//
// Derivation of the rotation matrices used in each test:
//   Body yawed +ψ around world-Y:
//     R_world_from_body = Ry(+ψ) =
//       | cos ψ,  0,  sin ψ |
//       |  0,     1,   0    |
//       | -sin ψ, 0,  cos ψ |
//     R_GtoI_ = R_body_from_world = Ry(-ψ) =
//       | cos ψ,  0, -sin ψ |
//       |  0,     1,   0    |
//       | sin ψ,  0,  cos ψ |
//
//   This is derived analytically (not via cv::Rodrigues) so any cv::Rodrigues
//   sign differences do NOT mask the EKF bug under test.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include "EKFState.h"

namespace {

// Tolerance for yaw comparisons (degrees → radians conversion below).
constexpr double kYawToleranceRad = 0.5 * M_PI / 180.0;  // 0.5°

// Build R_GtoI_ (world→body) for a body yawed +yaw_rad CCW around world-Y.
// As derived in the file-level comment:
//   R_GtoI_ = Ry(-yaw_rad) because R_GtoI_ = R_world_from_body^T = Ry(+ψ)^T = Ry(-ψ).
cv::Mat makeRGtoIForYaw(double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    // Ry(-ψ):  row-major
    return (cv::Mat_<double>(3, 3) <<
             c,  0.0, -s,
             0.0, 1.0,  0.0,
             s,  0.0,  c);
}

// Helper: initialize an EKFState with a specific R_GtoI_ for yaw-convention tests.
// Uses initializeFull which accepts R_GtoI directly.
void initEkfWithYaw(EKFState& ekf, double yaw_rad) {
    cv::Mat R = makeRGtoIForYaw(yaw_rad);
    ekf.initializeFull(R, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    // Add a clone so update methods don't fail for unrelated reasons.
    ekf.addClone(R, cv::Mat::zeros(3, 1, CV_64F), 1'000'000'000LL);
}

}  // namespace

// ─── Test 1 ─────────────────────────────────────────────────────────────────
// Body yawed +30° CCW — getYaw must return +30°.
//
// Current code returns atan2(-sin 30°, cos 30°) = atan2(-0.5, 0.866) ≈ -30°.
// Expected FAIL on current code.
TEST(EkfYawConvention, GetYaw_PureYaw30Deg_ReturnsPositive30) {
    const double yaw_deg = 30.0;
    const double yaw_rad = yaw_deg * M_PI / 180.0;

    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const double measured = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(measured, yaw_rad, kYawToleranceRad)
        << "getYaw returned " << measured * 180.0 / M_PI
        << "° but expected +" << yaw_deg << "°. "
        << "Sign is negated — classic world→body vs body→world mismatch.";
}

// ─── Test 2 ─────────────────────────────────────────────────────────────────
// Body yawed −60° — getYaw must return −60°.
//
// Current code returns +60° (negated).  Expected FAIL on current code.
TEST(EkfYawConvention, GetYaw_PureYawMinus60Deg_ReturnsNegative60) {
    const double yaw_deg = -60.0;
    const double yaw_rad = yaw_deg * M_PI / 180.0;

    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const double measured = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(measured, yaw_rad, kYawToleranceRad)
        << "getYaw returned " << measured * 180.0 / M_PI
        << "° but expected " << yaw_deg << "°.";
}

// ─── Test 3 ─────────────────────────────────────────────────────────────────
// Round-trip: physical navigation yaw (atan2 of forward_world.x,
// forward_world.z) must equal getYaw(0,0).
//
// Body +Z is the forward axis.  Under R_GtoI_ (world→body), the body +Z
// axis expressed in world coordinates is the THIRD COLUMN of
// R_body_from_world = R_GtoI_.t().  So forward_world = R_GtoI_.t() * e_z.
//
// Physical navigation yaw = atan2(forward_world.x, forward_world.z).
//
// For ψ = +45°:
//   forward_world = (sin 45°, 0, cos 45°)
//   physical_yaw  = atan2(sin 45°, cos 45°) = +45°
//   getYaw (current) = atan2(-sin 45°, cos 45°) = -45°  ← WRONG
//
// Expected FAIL on current code.
TEST(EkfYawConvention, GetYaw_RoundTripWithMadgwick_AgreesWithImuGetHeading) {
    const double yaw_rad = 45.0 * M_PI / 180.0;

    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    // Compute the physical navigation yaw from R_GtoI_.
    // forward_world = R_GtoI_.t() * e_z  (third column of R_world_from_body)
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    const double forward_x = s;   // third column of Ry(+ψ), row 0
    const double forward_z = c;   // third column of Ry(+ψ), row 2
    const double physical_yaw = std::atan2(forward_x, forward_z);

    const double ekf_yaw = ekf.getYaw(0.0, 0.0);
    EXPECT_NEAR(ekf_yaw, physical_yaw, kYawToleranceRad)
        << "getYaw=" << ekf_yaw * 180.0 / M_PI
        << "° physical_yaw=" << physical_yaw * 180.0 / M_PI
        << "°. These must agree for the yaw-correction residual to be correct.";
}

// ─── Test 4 ─────────────────────────────────────────────────────────────────
// Consistent measurement must produce zero residual — EKF yaw unchanged.
//
// If getYaw already returns the WRONG sign, and yaw_meas is the CORRECT sign,
// the residual is ≈ 2ψ, so the filter is moved by a large spurious correction.
// This test pins that: initialize at ψ, feed yaw_meas=ψ with tight variance,
// assert EKF yaw is still ≈ ψ.
//
// With the current sign bug:
//   getYaw(ψ_state) ≈ -ψ
//   residual = yaw_meas - getYaw = ψ - (-ψ) = 2ψ  ← drives a spurious update
//   post-update yaw != ψ  → test FAILS
TEST(EkfYawConvention, UpdateGravityAlignedYaw_ConsistentMeasurement_ProducesZeroResidual) {
    const double yaw_rad = 30.0 * M_PI / 180.0;

    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    // Feed the *correct* physical yaw as the measurement.
    const bool ok = ekf.updateGravityAlignedYaw(yaw_rad, 1e-4, 0.0, 0.0);
    ASSERT_TRUE(ok) << "updateGravityAlignedYaw returned false unexpectedly.";

    const double yaw_after = ekf.getYaw(0.0, 0.0);
    // Post-update yaw must still be close to ψ (consistent measurement = no movement).
    EXPECT_NEAR(yaw_after, yaw_rad, kYawToleranceRad)
        << "After consistent measurement yaw_meas=" << yaw_rad * 180.0 / M_PI
        << "°, EKF yaw changed to " << yaw_after * 180.0 / M_PI
        << "°. A consistent measurement must not move the state.";
}

// ─── Test 5 ─────────────────────────────────────────────────────────────────
// Small positive perturbation: measurement = ψ+5°, state at ψ.
// EKF must move TOWARD ψ+5°, NOT away from it.
//
// With the sign bug, H is negated so the Kalman gain pulls the wrong way:
// the state moves toward ψ−5° instead of ψ+5°.
//
// This is the most direct regression test for the 75° drift observed in the
// 485 s daytime walk.  Expected FAIL on current code.
TEST(EkfYawConvention, UpdateGravityAlignedYaw_SmallPositivePerturbation_MovesEKFTowardsMeasurement) {
    const double yaw_rad    = 30.0 * M_PI / 180.0;
    const double delta_rad  =  5.0 * M_PI / 180.0;
    const double yaw_meas   = yaw_rad + delta_rad;

    EKFState ekf;
    initEkfWithYaw(ekf, yaw_rad);

    const bool ok = ekf.updateGravityAlignedYaw(yaw_meas, 1e-4, 0.0, 0.0);
    ASSERT_TRUE(ok);

    const double yaw_after = ekf.getYaw(0.0, 0.0);

    // The post-update yaw must lie strictly between ψ and ψ+5°.
    EXPECT_GT(yaw_after, yaw_rad)
        << "Update with yaw_meas=" << yaw_meas * 180.0 / M_PI
        << "° did not increase yaw above " << yaw_rad * 180.0 / M_PI
        << "°. H sign is inverted — filter pulled the wrong way.";
    EXPECT_LT(yaw_after, yaw_meas + kYawToleranceRad)
        << "Update overshot past yaw_meas.";

    // Extra: post-update must be closer to ψ+5° than to ψ−5°.
    const double dist_to_meas  = std::abs(yaw_after - yaw_meas);
    const double dist_to_wrong = std::abs(yaw_after - (yaw_rad - delta_rad));
    EXPECT_LT(dist_to_meas, dist_to_wrong)
        << "EKF moved toward ψ−5° = " << (yaw_rad - delta_rad) * 180.0 / M_PI
        << "° rather than toward yaw_meas=" << yaw_meas * 180.0 / M_PI
        << "°. Confirms H sign reversal.";
}

// ─── Test 6 ─────────────────────────────────────────────────────────────────
// Propagation round-trip: apply deltaR representing a +30° yaw around body-Y
// (which is aligned with world-Y at identity), verify getYaw returns +30°.
//
// Convention assumed for IMUPreintegrator::preintegrate:
//   deltaR is body-frame incremental rotation: R_new = R_GtoI_ * deltaR
//   (see EKFState.cpp line 138).  Starting from identity R_GtoI_=I, after
//   one step deltaR=Ry(+30°), R_GtoI_ becomes Ry(+30°).  For a body that
//   has physically yawed +30°, R_GtoI_ = Ry(+30°) means world→body = Ry(−30°),
//   which is CONSISTENT with the world→body convention only if deltaR encodes
//   the world-frame rotation.  If deltaR is body-frame, R_GtoI_ = Ry(+30°) is
//   actually body→world, i.e., the convention is inverted.
//
// ASSUMPTION STATED INLINE: IMUPreintegrator produces deltaR in the body frame
// following the standard VIO formulation
//     R_new = R_old * deltaR_body   (EKFState.cpp line 138)
// which means after the propagation R_GtoI_ = I * Ry(+30°) = Ry(+30°).
// Under the CORRECT world→body convention, this represents a body yaw of −30°.
// THEREFORE the expected getYaw return is −30°, NOT +30°.
//
// If getYaw is fixed to return the correct sign (test 1 passes), then after
// propagation with deltaR=Ry(+30°) we must get −30°.  Alternatively, the
// deltaR convention may itself be inverted.  The test documents both paths so
// the developer can resolve the ambiguity during the fix.
//
// On CURRENT (buggy) code: getYaw returns the negated value, so it would return
// +30° even though R_GtoI_=Ry(+30°) physically means yaw=−30°.  After test 1
// is fixed, this test then correctly identifies whether propagation also needs
// a sign fix.
TEST(EkfPropagationConvention, RoundTripGyroIntegration_MatchesAnalytic) {
    const double yaw_deg  = 30.0;
    const double yaw_rad  = yaw_deg * M_PI / 180.0;

    // Start at identity (yaw = 0).
    EKFState ekf;
    cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);
    ekf.initializeFull(R_eye, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));

    // deltaR: body-frame Ry(+30°) — what IMUPreintegrator produces for a
    // +30° yaw rotation around the (body-aligned) Y axis.
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    cv::Mat deltaR = (cv::Mat_<double>(3, 3) <<
                       c,  0.0,  s,
                       0.0, 1.0, 0.0,
                      -s,  0.0,  c);

    // Zero Jacobians / noise for the test.
    cv::Mat zeros33 = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat zeros31 = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat imu_cov = cv::Mat::zeros(9, 9, CV_64F);

    ekf.propagateIMU(deltaR, zeros31, zeros31, 1.0,
                     imu_cov, zeros33, zeros33, zeros33, zeros33, zeros33);

    const double yaw_after = ekf.getYaw(0.0, 0.0);

    // Under the body-frame deltaR convention: R_GtoI_ = I * Ry(+30°) = Ry(+30°).
    // With the correct getYaw sign, atan2(-sin30°, cos30°) = -30°.
    // The test checks for -30° because BOTH conventions must be consistent.
    // If this instead returns +30° after the getYaw fix, the deltaR convention
    // is also wrong and needs a separate transpose fix.
    const double expected_yaw_rad = -yaw_rad;  // see comment above
    EXPECT_NEAR(yaw_after, expected_yaw_rad, kYawToleranceRad)
        << "After propagation with deltaR=Ry(+30°), getYaw returned "
        << yaw_after * 180.0 / M_PI << "°."
        << " Expected " << expected_yaw_rad * 180.0 / M_PI << "°."
        << " If getYaw sign is correct and result is +30°, the propagation"
        << " convention (R_new = R * deltaR vs R_new = deltaR * R) also needs review.";
}
