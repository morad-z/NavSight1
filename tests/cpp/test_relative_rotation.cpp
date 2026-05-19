// Step 2 (Visual Production Plan): unit tests for EKFState::updateRelativeRotation.
//
// Validates the per-frame relative-rotation EKF update that fuses R_vo from
// recoverPose. The implementation lives in EKFState.cpp/.h (parallel impl
// agent). API under test:
//
//     bool updateRelativeRotation(const cv::Mat& R_meas_body,
//                                 double sigma_axis_sq,
//                                 int clone_id);
//
// IMU error-state ordering (per EKFState.cpp): [δθ(0..2), δb_g(3..5),
// δv(6..8), δb_a(9..11), δp(12..14)]. Attitude covariance occupies the
// 3×3 block at P_(0..2, 0..2). Index 1 is body-Y attitude error.
//
// All tests are isolated: they do not exercise Tracker. They cover the
// acceptance criteria from `docs/VISUAL_PRODUCTION_PLAN.md` Step 2 plus the
// guard rails the impl agent will need against regressions.
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include "EKFState.h"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

// Indices of the body-frame attitude error within the IMU error-state.
// Mirrors the layout in EKFState.cpp (init covariance block at rows 0..2).
constexpr int kAttX = 0;
constexpr int kAttY = 1;
constexpr int kAttZ = 2;

// σ²_axis from the visual relative-rotation noise model used in the plan:
//     σ_axis ≈ 1 / (focal · √N_inliers)
// → σ²_axis = 1 / (focal² · N_inliers).
double sigmaAxisSqFromInliers(double focal, double n_inliers) {
    const double sigma = 1.0 / (focal * std::sqrt(n_inliers));
    return sigma * sigma;
}

// Construct a body-frame rotation by Rodrigues angle/axis (radians).
cv::Mat rotBody(double rx, double ry, double rz) {
    cv::Mat R;
    cv::Rodrigues(cv::Vec3d(rx, ry, rz), R);
    return R;
}

// Convert a body-frame Matx33d to cv::Mat for test composition.
cv::Mat matxToMat(const cv::Matx33d& m) {
    cv::Mat out(3, 3, CV_64F);
    for (int rr = 0; rr < 3; ++rr) {
        for (int cc = 0; cc < 3; ++cc) {
            out.at<double>(rr, cc) = m(rr, cc);
        }
    }
    return out;
}

// 2026-05-12 Option-C convention: addClone takes world→camera, NOT world→body.
// Production caller composes R_bc into the clone at Tracker.cpp:1919; mirror
// the same composition here so the tests exercise the same contract.
void addBodyClone(EKFState& ekf, const cv::Mat& R_GtoI_body,
                  const cv::Mat& p, int64_t ts_ns) {
    const cv::Mat R_bc_cv = matxToMat(ekf.getExtrinsicsRotation());
    const cv::Mat R_GtoC = R_bc_cv * R_GtoI_body;
    ekf.addClone(R_GtoC, p, ts_ns);
}

// [run_tests fix] EKFState carries a std::mutex (ADR-012) so it cannot be
// copied or moved. Helper now populates a passed-in reference; call sites
// use INIT_EKF_CLONE(ekf) to keep the diff minimal.
void initializeEKFWithIdentityClone(EKFState& ekf) {
    cv::Mat R_eye = cv::Mat::eye(3, 3, CV_64F);
    ekf.initializeFull(R_eye, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
    addBodyClone(ekf, R_eye, cv::Mat::zeros(3, 1, CV_64F), 1'000'000'000LL);
}
#define INIT_EKF_CLONE(name) EKFState name; initializeEKFWithIdentityClone(name)

}  // namespace

// ── (a) Synthetic 30° camera rotation ─────────────────────────────────────

// Plan acceptance criterion: a 30° body-Y rotation observation with σ²
// derived from focal=500, N_inliers=30 must shrink the body-Y attitude
// covariance, and the shrink factor must match the analytic 1-D Kalman
// scalar update K = P/(P+R), P_new = (1-K)·P, within 5%.
TEST(EKFStateRelativeRotation, Synthetic30DegYAxis_ShrinksAttitudeCovariance) {
    INIT_EKF_CLONE(ekf);
    int clone_id = ekf.getLatestCloneId();
    ASSERT_GE(clone_id, 0);

    cv::Mat R_meas_body = rotBody(0.0, 30.0 * kDeg2Rad, 0.0);

    const double sigma_axis_sq = sigmaAxisSqFromInliers(500.0, 30.0);

    cv::Mat P_before = ekf.getCovariance();
    ASSERT_GE(P_before.rows, EKFState::IMU_STATE_DIM);
    const double p_yaw_before = P_before.at<double>(kAttY, kAttY);
    ASSERT_GT(p_yaw_before, 0.0);

    bool ok = ekf.updateRelativeRotation(R_meas_body, sigma_axis_sq, clone_id);
    ASSERT_TRUE(ok);

    cv::Mat P_after = ekf.getCovariance();
    ASSERT_EQ(P_after.rows, P_before.rows);
    const double p_yaw_after = P_after.at<double>(kAttY, kAttY);

    // Covariance must strictly shrink — a measurement was applied.
    EXPECT_LT(p_yaw_after, p_yaw_before);

    // Joseph-form scalar Kalman update on a 1-D measurement gives:
    //   K = P / (P + R)         shrink_factor = (1 - K) = R / (P + R)
    // The actual update couples additional axes via R_GtoI_, but for an
    // identity clone + identity attitude prior with isotropic per-axis
    // measurement noise, the body-Y axis update is decoupled to a scalar.
    const double expected_shrink =
        sigma_axis_sq / (p_yaw_before + sigma_axis_sq);
    const double actual_shrink  = p_yaw_after / p_yaw_before;
    EXPECT_NEAR(actual_shrink, expected_shrink, 0.05 * expected_shrink)
        << "expected shrink ~" << expected_shrink
        << ", got " << actual_shrink;
}

// ── (b) Returns false on bad inputs ──────────────────────────────────────

TEST(EKFStateRelativeRotation, ReturnsFalse_WhenStateNotFullyInitialized) {
    EKFState ekf;  // no initializeFull → full_initialized_ == false
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    EXPECT_FALSE(ekf.updateRelativeRotation(R, 1e-4, /*clone_id=*/0));
}

TEST(EKFStateRelativeRotation, ReturnsFalse_WhenCloneIdMissing) {
    INIT_EKF_CLONE(ekf);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    EXPECT_FALSE(ekf.updateRelativeRotation(R, 1e-4, /*clone_id=*/9999));
}

TEST(EKFStateRelativeRotation, ReturnsFalse_WhenRotationEmpty) {
    INIT_EKF_CLONE(ekf);
    int clone_id = ekf.getLatestCloneId();
    cv::Mat R_empty;
    EXPECT_FALSE(ekf.updateRelativeRotation(R_empty, 1e-4, clone_id));
}

TEST(EKFStateRelativeRotation, ReturnsFalse_WhenRotationWrongShape) {
    INIT_EKF_CLONE(ekf);
    int clone_id = ekf.getLatestCloneId();
    cv::Mat R_wrong = cv::Mat::eye(4, 4, CV_64F);  // not 3x3
    EXPECT_FALSE(ekf.updateRelativeRotation(R_wrong, 1e-4, clone_id));

    cv::Mat R_2x3 = cv::Mat::zeros(2, 3, CV_64F);
    EXPECT_FALSE(ekf.updateRelativeRotation(R_2x3, 1e-4, clone_id));
}

// ── (c) Multi-axis rotation ──────────────────────────────────────────────

TEST(EKFStateRelativeRotation, MultiAxisRotation_AllAttitudeAxesShrink) {
    INIT_EKF_CLONE(ekf);
    int clone_id = ekf.getLatestCloneId();
    ASSERT_GE(clone_id, 0);

    // 15° about body-X, 10° about body-Y, 5° about body-Z. All three axes
    // should receive a measurement with the same isotropic per-axis variance,
    // so all three diagonal entries must shrink.
    cv::Mat R_meas = rotBody(15.0 * kDeg2Rad,
                             10.0 * kDeg2Rad,
                              5.0 * kDeg2Rad);

    const double sigma_axis_sq = sigmaAxisSqFromInliers(500.0, 30.0);

    cv::Mat P_before = ekf.getCovariance();
    const double p_xx_before = P_before.at<double>(kAttX, kAttX);
    const double p_yy_before = P_before.at<double>(kAttY, kAttY);
    const double p_zz_before = P_before.at<double>(kAttZ, kAttZ);

    ASSERT_TRUE(ekf.updateRelativeRotation(R_meas, sigma_axis_sq, clone_id));

    cv::Mat P_after = ekf.getCovariance();
    EXPECT_LT(P_after.at<double>(kAttX, kAttX), p_xx_before);
    EXPECT_LT(P_after.at<double>(kAttY, kAttY), p_yy_before);
    EXPECT_LT(P_after.at<double>(kAttZ, kAttZ), p_zz_before);

    // Covariance must remain symmetric positive — sanity guard against a
    // broken Joseph form (sign error in I-KH would tank one of these).
    EXPECT_GT(P_after.at<double>(kAttX, kAttX), 0.0);
    EXPECT_GT(P_after.at<double>(kAttY, kAttY), 0.0);
    EXPECT_GT(P_after.at<double>(kAttZ, kAttZ), 0.0);
}

// ── (d) No-op rotation: measurement matches prediction ───────────────────

TEST(EKFStateRelativeRotation, IdentityMeasurement_StateUnchanged_CovShrinks) {
    INIT_EKF_CLONE(ekf);
    int clone_id = ekf.getLatestCloneId();

    cv::Mat R_before_state = ekf.getRotation();
    cv::Mat P_before = ekf.getCovariance();
    const double p_yy_before = P_before.at<double>(kAttY, kAttY);

    // R_meas = I matches the prediction (clone at I, current at I → ΔR = I).
    cv::Mat R_meas = cv::Mat::eye(3, 3, CV_64F);
    const double sigma_axis_sq = sigmaAxisSqFromInliers(500.0, 30.0);
    ASSERT_TRUE(ekf.updateRelativeRotation(R_meas, sigma_axis_sq, clone_id));

    cv::Mat R_after_state = ekf.getRotation();
    cv::Mat P_after = ekf.getCovariance();

    // State should not move (residual is zero → correction is zero).
    cv::Mat dR = R_after_state * R_before_state.t();
    cv::Mat rvec;
    cv::Rodrigues(dR, rvec);
    EXPECT_LT(cv::norm(rvec), 1e-9);

    // Covariance still shrinks because we got information.
    EXPECT_LT(P_after.at<double>(kAttY, kAttY), p_yy_before);
}

// ── (e) Frame-convention sanity ──────────────────────────────────────────

// Set the clone to a known body-yaw of 45°, set the current R_GtoI_ to body-yaw
// 90°, so the predicted relative rotation between clone and current is +45°
// about body-Y. Pass R_meas_body matching that prediction. Residual should be
// ~zero → state must not move appreciably; covariance must still shrink.
TEST(EKFStateRelativeRotation, FrameConvention_PredictionMatch_NoStateMove) {
    EKFState ekf;

    // Current attitude: 90° body-Y rotation.
    cv::Mat R_curr = rotBody(0.0, 90.0 * kDeg2Rad, 0.0);
    ekf.initializeFull(R_curr, cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));

    // Clone at 45° body-Y rotation. The relative rotation from clone to
    // current is (R_curr * R_clone.t()) when expressed as world-frame
    // composition; the *body-frame* relative rotation is R_clone.t() * R_curr,
    // which equals a +45° body-Y rotation either way for pure-yaw cases.
    cv::Mat R_clone_body = rotBody(0.0, 45.0 * kDeg2Rad, 0.0);
    addBodyClone(ekf, R_clone_body, cv::Mat::zeros(3, 1, CV_64F),
                 1'000'000'000LL);

    int clone_id = ekf.getLatestCloneId();
    ASSERT_GE(clone_id, 0);

    // Measurement matches the +45° body-Y prediction.
    cv::Mat R_meas_body = rotBody(0.0, 45.0 * kDeg2Rad, 0.0);
    const double sigma_axis_sq = sigmaAxisSqFromInliers(500.0, 30.0);

    cv::Mat P_before = ekf.getCovariance();
    const double p_yy_before = P_before.at<double>(kAttY, kAttY);
    cv::Mat R_state_before = ekf.getRotation();

    ASSERT_TRUE(ekf.updateRelativeRotation(R_meas_body, sigma_axis_sq, clone_id));

    cv::Mat R_state_after = ekf.getRotation();
    cv::Mat P_after = ekf.getCovariance();

    // State must not appreciably move when residual is zero. Bound is loose
    // (1e-3 rad ≈ 0.057°) to allow tiny couplings through cloned δp covariance.
    cv::Mat dR = R_state_after * R_state_before.t();
    cv::Mat rvec;
    cv::Rodrigues(dR, rvec);
    EXPECT_LT(cv::norm(rvec), 1e-3)
        << "state moved by " << cv::norm(rvec)
        << " rad despite zero-residual measurement — frame convention bug?";

    // Covariance must still shrink (we received information).
    EXPECT_LT(P_after.at<double>(kAttY, kAttY), p_yy_before);
}
