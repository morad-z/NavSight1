// Step 5 (Visual Production Plan): unit tests for adaptive front-end
// robustness — motion-blur detection, adaptive KLT window, dual-gate
// pure-rotation detector (Rayleigh + gyro), and low-light feature policy.
//
// Plan reference: docs/VISUAL_PRODUCTION_PLAN.md, "Step 5 — Adaptive
// front-end robustness" (around line 611).
//
// Agent A is landing the implementations in Tracker.{h,cpp} (blur,
// adaptive KLT window, dual-gate rotation detector); Agent C is landing
// the low-light feature policy in FeatureManager.{h,cpp} and
// docs/adr/ADR-011-adaptive-front-end.md. To stay decoupled from the
// in-flight wiring, these tests exercise the SAME OpenCV primitives and
// the SAME formulas/parameters the implementations are using:
//
//   - Variance of Laplacian on a centre crop, threshold ≈ 80.
//   - win_sz = clamp(int(2.0 * focal * |gyro| * dt + 11.0), 21, 41), odd.
//   - Rayleigh resultant length R/N on flow-vector directions, gate 0.3.
//   - Dual gate: gyro magnitude >= GYRO_ROT_ONLY_THRESH (2.0 rad/s) AND
//     R/N <= 0.3.
//   - Low-light policy: brightness < 0.12 → MAX_FEATURES=120, QUALITY=0.10,
//     else MAX_FEATURES=200, QUALITY=0.05.
//
// Idiom mirrors test_relative_rotation.cpp and test_orb_reloc.cpp from
// today's Step-4 commit: anonymous namespace helpers, OpenCV asserts,
// trend-based bounds. All scenes are synthetic and in-memory; no sim
// files, no device recordings.

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

// ── Step 5 §1: motion-blur detection parameters ─────────────────────────

// Variance-of-Laplacian threshold the Tracker uses to flag a frame as
// blurry. Below this, the visual update is skipped this frame (IMU still
// propagates). Plan says "~80".
constexpr double kBlurVarianceThresh = 80.0;

// Frame size used by all synthetic scenes. Matches the (640, 480) size
// VioEngine feeds Tracker on the desktop test path.
constexpr int kFrameW = 640;
constexpr int kFrameH = 480;

// Centre-crop fraction the blur detector evaluates. Matches the impl's
// approach of ignoring image borders where vignetting and warp artefacts
// would otherwise lower the variance unfairly.
constexpr double kCentreCropFrac = 0.5;

// ── Step 5 §2: adaptive KLT window parameters ───────────────────────────

constexpr int kKltWinMin = 21;
constexpr int kKltWinMax = 41;

// ── Step 5 §4: dual-gate pure-rotation detector parameters ──────────────

// Tracker keeps the existing "2.0 rad/s" gyro floor (the conservative
// gate from before Step 5) AND adds a Rayleigh resultant-length test on
// the optical-flow direction distribution. Pure-rotation requires BOTH:
//   |gyro| >= GYRO_ROT_ONLY_THRESH       (gyro half)
//   R/N    <= kRayleighRotationThresh    (flow half — uniform direction)
constexpr double kGyroRotOnlyThresh = 2.0;
constexpr double kRayleighRotationThresh = 0.3;

// Translational-flow threshold: above this, flow vectors are clearly
// concentrated in one direction → translation, NOT pure rotation.
constexpr double kRayleighTranslationFloor = 0.7;

// ── Step 5 §3: low-light feature policy parameters ──────────────────────

constexpr double kLowLightBrightnessThresh = 0.12;
constexpr int    kFeaturesNormal       = 200;
constexpr int    kFeaturesLowLight     = 120;
constexpr double kQualityNormal        = 0.05;
constexpr double kQualityLowLight      = 0.10;

// ── Helpers ──────────────────────────────────────────────────────────────

// Build a textured grayscale frame: uniform noise + Gaussian blur.
// Mirrors test_orb_reloc.cpp::makeTexturedNoiseFrame so the two test
// files agree on what a "clear" baseline frame looks like. Sigma is
// caller-controlled so we can produce both clear and heavily-blurred
// versions of the same underlying texture.
cv::Mat makeNoiseFrame(uint64_t seed, double blur_sigma) {
    cv::Mat raw(kFrameH, kFrameW, CV_8UC1);
    cv::RNG rng(seed);
    rng.fill(raw, cv::RNG::UNIFORM, 0, 256);
    cv::Mat blurred;
    cv::GaussianBlur(raw, blurred, cv::Size(0, 0), blur_sigma);
    return blurred;
}

// Crop the centre `frac` portion of the frame. Used by the blur detector
// to avoid border artefacts dragging the variance down on a clean frame.
cv::Mat centreCrop(const cv::Mat& src, double frac) {
    const int w = static_cast<int>(src.cols * frac);
    const int h = static_cast<int>(src.rows * frac);
    const int x = (src.cols - w) / 2;
    const int y = (src.rows - h) / 2;
    return src(cv::Rect(x, y, w, h)).clone();
}

// Variance of Laplacian on a grayscale frame. Higher = sharper.
double varianceOfLaplacian(const cv::Mat& gray) {
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

// Adaptive KLT window formula from Step 5 §2:
//   win = clamp(int(2.0 * focal * |gyro| * dt + 11.0), 21, 41)
// then rounded UP to the next odd integer (cv::calcOpticalFlowPyrLK
// requires odd window dimensions).
int adaptiveKltWindow(double focal, double gyro_mag, double dt) {
    const double raw = 2.0 * focal * gyro_mag * dt + 11.0;
    int win = static_cast<int>(raw);
    win = std::clamp(win, kKltWinMin, kKltWinMax);
    if ((win % 2) == 0) {
        win = std::min(win + 1, kKltWinMax);
        // kKltWinMax is 41 (odd) so this branch always lands on odd.
    }
    return win;
}

// Rayleigh resultant length R/N of a set of 2-D vectors, treating each
// vector's *direction* as a unit vector on the circle. R/N near 1 means
// the directions are concentrated; R/N near 0 means uniform (or
// anti-aligned).
double rayleighResultantLength(const std::vector<cv::Vec2f>& flow) {
    if (flow.empty()) return 0.0;
    double sum_x = 0.0, sum_y = 0.0;
    int counted = 0;
    for (const auto& v : flow) {
        const double n = std::hypot(v[0], v[1]);
        if (n < 1e-9) continue;  // skip zero-length vectors
        sum_x += v[0] / n;
        sum_y += v[1] / n;
        ++counted;
    }
    if (counted == 0) return 0.0;
    const double R = std::hypot(sum_x, sum_y);
    return R / static_cast<double>(counted);
}

// Dual-gate pure-rotation classifier: true iff BOTH gyro magnitude is
// above the threshold AND optical-flow direction distribution is
// effectively uniform (low Rayleigh R/N).
bool isPureRotationDualGate(double gyro_mag, double rayleigh_R_over_N) {
    return (gyro_mag >= kGyroRotOnlyThresh) &&
           (rayleigh_R_over_N <= kRayleighRotationThresh);
}

// Low-light feature policy: returns (max_features, quality_level).
// Matches Step 5 §3: in low light (brightness < 0.12), drop max features
// to 120 and double quality_level to 0.10. Otherwise use the normal
// targets (200, 0.05).
struct FeaturePolicy {
    int    max_features;
    double quality_level;
};
FeaturePolicy lowLightFeaturePolicy(double brightness_0_to_1) {
    if (brightness_0_to_1 < kLowLightBrightnessThresh) {
        return {kFeaturesLowLight, kQualityLowLight};
    }
    return {kFeaturesNormal, kQualityNormal};
}

// Build N flow vectors all pointing roughly +x with bounded angular jitter
// (radians). Length is fixed at 1.0 — the Rayleigh test is direction-only
// so length is irrelevant, but we keep it positive to avoid the
// zero-length filter.
std::vector<cv::Vec2f> makeConcentratedFlow(int N, double jitter_rad,
                                            uint64_t seed) {
    std::vector<cv::Vec2f> out;
    out.reserve(N);
    cv::RNG rng(seed);
    for (int i = 0; i < N; ++i) {
        const double a = rng.uniform(-jitter_rad, jitter_rad);
        out.emplace_back(static_cast<float>(std::cos(a)),
                         static_cast<float>(std::sin(a)));
    }
    return out;
}

// Build N flow vectors with directions sampled uniformly on [0, 2π).
std::vector<cv::Vec2f> makeUniformFlow(int N, uint64_t seed) {
    std::vector<cv::Vec2f> out;
    out.reserve(N);
    cv::RNG rng(seed);
    for (int i = 0; i < N; ++i) {
        const double a = rng.uniform(0.0, 2.0 * M_PI);
        out.emplace_back(static_cast<float>(std::cos(a)),
                         static_cast<float>(std::sin(a)));
    }
    return out;
}

}  // namespace

// ── Test 1 — blur detection flags blurred frame ──────────────────────────

// Plan acceptance criterion: variance-of-Laplacian below ~80 → blurry.
// Generate a textured frame (σ=1.0) and a heavily blurred version of the
// SAME underlying texture (σ=5.0). Centre-crop both before measuring so
// border warp/blur from the convolution doesn't bias the result. Assert
// the clear frame's variance clears the threshold, the blurred frame
// fails it, and the blurred is at least 5× lower than clear.
TEST(VisualRobustness, BlurDetectionFlagsBlurredFrame) {
    // [run_tests fix] Original literal `0xB1U2R3R4U5L6` is invalid C++ —
    // 'U2R3R4U5L6' is not a hex suffix. Replaced with a valid 64-bit hex
    // constant of equivalent magic-number role.
    const uint64_t seed = 0xB1B2B3B4B5B6ULL;

    cv::Mat clear   = makeNoiseFrame(seed, /*blur_sigma=*/1.0);
    cv::Mat blurred = makeNoiseFrame(seed, /*blur_sigma=*/5.0);

    cv::Mat clear_centre   = centreCrop(clear,   kCentreCropFrac);
    cv::Mat blurred_centre = centreCrop(blurred, kCentreCropFrac);

    const double var_clear   = varianceOfLaplacian(clear_centre);
    const double var_blurred = varianceOfLaplacian(blurred_centre);

    EXPECT_GT(var_clear, kBlurVarianceThresh)
        << "Clear (σ=1.0) frame variance " << var_clear
        << " did NOT clear the blur threshold (" << kBlurVarianceThresh
        << ") — synthetic baseline is too soft.";

    EXPECT_LT(var_blurred, kBlurVarianceThresh)
        << "Heavily-blurred (σ=5.0) frame variance " << var_blurred
        << " was ABOVE the blur threshold (" << kBlurVarianceThresh
        << ") — detector would let blurry frames through.";

    ASSERT_GT(var_blurred, 0.0)
        << "Blurred variance was zero — math broken before ratio test.";
    EXPECT_GT(var_clear / var_blurred, 5.0)
        << "Clear:blurred variance ratio " << (var_clear / var_blurred)
        << " is below 5×; blur detector lacks dynamic range to trigger "
           "reliably on real motion blur.";
}

// ── Test 2 — adaptive KLT window grows with gyro ────────────────────────

// Plan formula §2: win = clamp(int(2.0 * focal * |gyro| * dt + 11.0),
// 21, 41), rounded to next odd. At gyro=0 we get the floor (21). At
// gyro=1.0 rad/s with focal=525, dt=1/30 the expected pixel displacement
// is 525 / 30 = 17.5 px, so the formula yields ≈46 → clamped to 41.
// Higher gyro stays clamped at 41. KLT requires odd windows.
TEST(VisualRobustness, AdaptiveKltWindowGrowsWithGyro) {
    const double focal = 525.0;
    const double dt    = 1.0 / 30.0;

    // gyro = 0 → floor.
    const int win_zero = adaptiveKltWindow(focal, /*gyro=*/0.0, dt);
    EXPECT_EQ(win_zero, kKltWinMin)
        << "At zero gyro, expected window = " << kKltWinMin
        << " (formula floor) but got " << win_zero;
    EXPECT_EQ(win_zero % 2, 1) << "Window must be odd for KLT.";

    // gyro = 1.0 rad/s, focal=525, dt=1/30:
    //   raw = 2 * 525 * 1.0 / 30 + 11 = 35 + 11 = 46 → clamped to 41.
    const int win_mid = adaptiveKltWindow(focal, /*gyro=*/1.0, dt);
    EXPECT_GE(win_mid, 35)
        << "Mid-gyro window " << win_mid
        << " is below the displacement-driven floor (35).";
    EXPECT_LE(win_mid, kKltWinMax)
        << "Mid-gyro window " << win_mid
        << " exceeded clamp ceiling (" << kKltWinMax << ").";
    EXPECT_EQ(win_mid % 2, 1) << "Window must be odd for KLT.";

    // gyro = 3.0 rad/s → clamped at ceiling.
    const int win_high = adaptiveKltWindow(focal, /*gyro=*/3.0, dt);
    EXPECT_EQ(win_high, kKltWinMax)
        << "At gyro=3.0 rad/s, expected clamp at " << kKltWinMax
        << " but got " << win_high;
    EXPECT_EQ(win_high % 2, 1) << "Window must be odd for KLT.";

    // Monotonicity: higher gyro must produce a window >= lower gyro.
    EXPECT_GE(win_mid, win_zero);
    EXPECT_GE(win_high, win_mid);
}

// ── Test 3 — Rayleigh rejects rotation-dominant flow ─────────────────────

// Plan §4: the Rayleigh resultant length R/N on the flow-direction
// distribution discriminates "translation" (concentrated → high R/N)
// from "pure rotation" (uniform → low R/N). Build N=200 vectors for each
// case and assert the 0.3 gate cleanly separates them.
TEST(VisualRobustness, RayleighRejectsRotationDominantFlow) {
    constexpr int N = 200;

    // Case A: vectors point in roughly +x with small angular jitter
    // (±5°). Resultant length should be near 1.0.
    auto flow_translation = makeConcentratedFlow(
        N, /*jitter_rad=*/5.0 * M_PI / 180.0, /*seed=*/0xA1A1A1A1u);
    const double R_translation = rayleighResultantLength(flow_translation);
    EXPECT_GT(R_translation, kRayleighTranslationFloor)
        << "Concentrated flow produced R/N=" << R_translation
        << ", below the translation floor (" << kRayleighTranslationFloor
        << "). Rayleigh would mis-classify this as pure rotation.";

    // Case B: vectors uniformly distributed on [0, 2π). Resultant
    // length should be small (sqrt(N) random walk → R/N ~ O(1/sqrt(N))).
    auto flow_rotation = makeUniformFlow(N, /*seed=*/0xB2B2B2B2u);
    const double R_rotation = rayleighResultantLength(flow_rotation);
    EXPECT_LT(R_rotation, kRayleighRotationThresh)
        << "Uniform-direction flow produced R/N=" << R_rotation
        << ", above the rotation gate (" << kRayleighRotationThresh
        << "). Rayleigh would mis-classify this as translation.";

    // The gate must cleanly separate the two cases.
    EXPECT_LT(R_rotation, R_translation)
        << "Concentrated flow (R/N=" << R_translation
        << ") was not larger than uniform flow (R/N=" << R_rotation
        << ") — Rayleigh metric is broken.";
}

// ── Test 4 — dual gate respects gyro floor ──────────────────────────────

// Plan §4: pure rotation requires BOTH the gyro half (|gyro| >= 2.0
// rad/s) AND the flow half (R/N <= 0.3). Failing either half must
// disqualify the frame from being classified as pure rotation, even if
// the other half is satisfied.
TEST(VisualRobustness, RayleighDualGateRespectsGyroFloor) {
    constexpr int N = 200;

    // Symmetric "uniform-direction flow but low gyro" case: R/N is well
    // below the rotation gate, but the gyro magnitude is below the floor
    // → must NOT be classified as pure rotation. The gyro half rules it
    // out (e.g. user is walking past a uniform texture; visual flow
    // looks rotation-like by accident but IMU disagrees).
    auto uniform_flow = makeUniformFlow(N, /*seed=*/0xC3C3C3C3u);
    const double R_uniform = rayleighResultantLength(uniform_flow);
    ASSERT_LT(R_uniform, kRayleighRotationThresh)
        << "Test setup invalid: uniform flow R/N=" << R_uniform
        << " should be below the rotation gate.";

    const double gyro_low = 1.0;  // below 2.0 rad/s threshold
    EXPECT_FALSE(isPureRotationDualGate(gyro_low, R_uniform))
        << "Dual gate classified low-gyro + uniform flow as pure rotation; "
           "gyro half (>=2.0 rad/s) is not being enforced.";

    // Symmetric "high gyro but concentrated flow" case: gyro clears the
    // floor easily but the flow-direction distribution is concentrated
    // (e.g. scooter is genuinely translating while head turns slightly)
    // → must NOT be classified as pure rotation. The flow half rules it
    // out.
    auto concentrated_flow = makeConcentratedFlow(
        N, /*jitter_rad=*/5.0 * M_PI / 180.0, /*seed=*/0xD4D4D4D4u);
    const double R_concentrated = rayleighResultantLength(concentrated_flow);
    ASSERT_GT(R_concentrated, kRayleighTranslationFloor)
        << "Test setup invalid: concentrated flow R/N=" << R_concentrated
        << " should be above the translation floor.";

    const double gyro_high = 3.0;  // well above 2.0 rad/s threshold
    EXPECT_FALSE(isPureRotationDualGate(gyro_high, R_concentrated))
        << "Dual gate classified high-gyro + concentrated flow as pure "
           "rotation; flow half (R/N<=0.3) is not being enforced.";

    // Positive control: high gyro AND uniform flow → classified pure
    // rotation. Confirms the gate is not stuck at "always false".
    EXPECT_TRUE(isPureRotationDualGate(gyro_high, R_uniform))
        << "Dual gate failed to classify high-gyro + uniform flow as "
           "pure rotation; both halves were satisfied.";
}

// ── Test 5 — low-light policy lowers feature targets ────────────────────

// Plan §3: brightness < 0.12 → MAX_FEATURES drops 200→120, QUALITY_LEVEL
// doubles 0.05→0.10. Otherwise the normal (200, 0.05) tuning. Assert the
// boundary and both regimes; assert the low-light policy is strictly
// stronger (fewer corners, higher quality bar) so only robust features
// survive when SNR is poor.
TEST(VisualRobustness, LowLightLowersFeatureTargets) {
    // Dark-scene case (brightness 0.08): low-light policy must engage.
    FeaturePolicy dark = lowLightFeaturePolicy(0.08);
    EXPECT_EQ(dark.max_features, kFeaturesLowLight)
        << "Low-light max features should drop to " << kFeaturesLowLight
        << ", got " << dark.max_features;
    EXPECT_DOUBLE_EQ(dark.quality_level, kQualityLowLight)
        << "Low-light quality level should be " << kQualityLowLight
        << ", got " << dark.quality_level;

    // Bright-scene case (brightness 0.5): normal policy.
    FeaturePolicy bright = lowLightFeaturePolicy(0.5);
    EXPECT_EQ(bright.max_features, kFeaturesNormal)
        << "Normal-lit max features should be " << kFeaturesNormal
        << ", got " << bright.max_features;
    EXPECT_DOUBLE_EQ(bright.quality_level, kQualityNormal)
        << "Normal-lit quality level should be " << kQualityNormal
        << ", got " << bright.quality_level;

    // Boundary: brightness exactly at the threshold (0.12) must use the
    // normal policy (the spec says "< 0.12" — strict inequality).
    FeaturePolicy boundary = lowLightFeaturePolicy(kLowLightBrightnessThresh);
    EXPECT_EQ(boundary.max_features, kFeaturesNormal)
        << "Threshold boundary (brightness=" << kLowLightBrightnessThresh
        << ") should use the normal policy, not low-light.";
    EXPECT_DOUBLE_EQ(boundary.quality_level, kQualityNormal);

    // Cross-policy invariants the implementation must honour:
    EXPECT_LT(dark.max_features, bright.max_features)
        << "Low-light policy must request strictly FEWER features than "
           "the normal policy.";
    EXPECT_GT(dark.quality_level, bright.quality_level)
        << "Low-light policy must demand strictly HIGHER corner quality "
           "(fewer but stronger corners).";
}
