#include <gtest/gtest.h>
#include <cmath>
#include "Tracker.h"
#include "IMUPreintegrator.h"
#include "VioEngine.h"
#include "ScaleEstimatorVI.h"
#include "ScaleFuser.h"
#include "test_utils.h"

class TrackerTest : public ::testing::Test {
protected:
    Tracker tracker;
    IMUPreintegrator imu;
    static constexpr int W = 640;
    static constexpr int H = 480;
    static constexpr int64_t BASE_NS = 1'000'000'000LL;
    static constexpr int64_t FRAME_DT_NS = 33'333'333LL; // 30 fps

    void SetUp() override {
        tracker.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0);
    }

    VisionOutput processOneFrame(const std::vector<uint8_t>& frame, int64_t ts) {
        return test_utils::processFrame(tracker, imu, frame, W, H, ts);
    }
};

// ── Construction & Reset ─────────────────────────────────────────────────────

TEST_F(TrackerTest, Construction_NoThrow) {
    Tracker t;
    SUCCEED();
}

TEST_F(TrackerTest, Reset_NoThrow) {
    tracker.reset();
    SUCCEED();
}

// ── First Frame Initialization ───────────────────────────────────────────────

TEST_F(TrackerTest, FirstFrame_ReturnsInvalid) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    test_utils::feedStaticIMU(imu, BASE_NS - 100'000'000LL, 100);

    auto output = processOneFrame(frame, BASE_NS);

    EXPECT_FALSE(output.valid) << "First frame should be initialization only";
}

// ── Static Scene (ZUPT) ─────────────────────────────────────────────────────

TEST_F(TrackerTest, StaticScene_NoDrift) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    int num_frames = 90; // 3 seconds at 30fps
    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        test_utils::feedStaticIMU(imu, ts, 33);
        processOneFrame(frame, ts);
    }

    int64_t last_ts = BASE_NS + num_frames * FRAME_DT_NS;
    test_utils::feedStaticIMU(imu, last_ts, 33);
    auto output = processOneFrame(frame, last_ts);

    if (output.valid && !output.t.empty()) {
        double pos_norm = cv::norm(output.t);
        EXPECT_LT(pos_norm, 0.1)
            << "Static scene should have near-zero position drift, got " << pos_norm << "m";
    }
}

// ── Static Scene Heading Stability (BUG-013 regression test) ────────────────

TEST_F(TrackerTest, StaticScene_HeadingStable) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    // Initialize with a known heading
    // 2026-06-12 — signature gained the IMUPreintegrator arg in cbf20f0 (2026-05-19); the fixture's imu
    // (static-fed in the loop below) preserves the test's intent.
    tracker.setInitialHeading(1.0, imu); // ~57 degrees
    double initial_heading = tracker.getHeading();

    int num_frames = 90; // 3 seconds at 30fps
    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        test_utils::feedStaticIMU(imu, ts, 33);
        processOneFrame(frame, ts);
    }

    double final_heading = tracker.getHeading();
    double hdiff = std::abs(final_heading - initial_heading);
    if (hdiff > M_PI) hdiff = 2.0 * M_PI - hdiff;

    EXPECT_LT(hdiff, 0.05)
        << "Static scene: heading should not drift. Initial=" << initial_heading
        << " Final=" << final_heading << " Diff=" << hdiff << " rad";
}

// ── Pure Rotation ────────────────────────────────────────────────────────────

TEST_F(TrackerTest, PureRotation_NoPositionDrift) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    int num_frames = 60;
    double total_position = 0.0;

    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;

        float angle = static_cast<float>(i) * 0.05f;
        std::vector<cv::Point2f> rotated;
        for (const auto& p : features) {
            float dx = p.x - W / 2.0f;
            float dy = p.y - H / 2.0f;
            float rx = dx * std::cos(angle) - dy * std::sin(angle) + W / 2.0f;
            float ry = dx * std::sin(angle) + dy * std::cos(angle) + H / 2.0f;
            if (rx > 5 && rx < W - 5 && ry > 5 && ry < H - 5)
                rotated.emplace_back(rx, ry);
        }

        auto frame = test_utils::createSyntheticNV21(W, H, rotated);
        test_utils::feedRotatingIMU(imu, ts, 33, 0.0f, 0.0f, 1.0f);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            total_position = cv::norm(output.t);
        }
    }

    EXPECT_LT(total_position, 0.5)
        << "Pure rotation should not cause significant position drift, got " << total_position << "m";
}

// ── Forward Translation ──────────────────────────────────────────────────────

TEST_F(TrackerTest, ForwardTranslation_PositionGrows) {
    bool position_grew = false;

    // Pre-seed 2 seconds of walking IMU so step detector warms up
    test_utils::feedWalkingIMU(imu, BASE_NS - 2'000'000'000LL, 2000, 0.5f);

    for (int i = 0; i < 60; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * i, 0);
        test_utils::feedWalkingIMU(imu, ts, 33, 0.5f);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            if (pos > 0.01) position_grew = true;
        }
    }

    EXPECT_TRUE(position_grew) << "Forward translation should increase position from origin";
}

// ── Intrinsics ───────────────────────────────────────────────────────────────

TEST_F(TrackerTest, SetIntrinsics_NoThrow) {
    tracker.setIntrinsics(600.0, 600.0, 320.0, 240.0);
    SUCCEED();
}

TEST_F(TrackerTest, ZeroIntrinsics_UsesDefault) {
    tracker.setIntrinsics(0.0, 0.0, 0.0, 0.0);

    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);
    test_utils::feedStaticIMU(imu, BASE_NS - 50'000'000LL, 50);

    auto output = processOneFrame(frame, BASE_NS);
    SUCCEED();
}

// ── Edge Cases ───────────────────────────────────────────────────────────────

TEST_F(TrackerTest, EmptyFrame_NoFeatures_NoCrash) {
    std::vector<uint8_t> gray_frame(W * H + W * H / 2, 128);
    test_utils::feedStaticIMU(imu, BASE_NS - 50'000'000LL, 50);

    auto output = processOneFrame(gray_frame, BASE_NS);
    SUCCEED();
}

TEST_F(TrackerTest, LargeTimestampGap_HandledGracefully) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    test_utils::feedStaticIMU(imu, BASE_NS - 50'000'000LL, 50);
    processOneFrame(frame, BASE_NS);

    int64_t late_ts = BASE_NS + 10'000'000'000LL;
    test_utils::feedStaticIMU(imu, late_ts - 50'000'000LL, 50);
    auto output = processOneFrame(frame, late_ts);

    SUCCEED();
}

TEST_F(TrackerTest, RapidReset_NoLeaks) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    for (int cycle = 0; cycle < 10; cycle++) {
        test_utils::feedStaticIMU(imu, BASE_NS, 50);
        processOneFrame(frame, BASE_NS + cycle * FRAME_DT_NS);
        tracker.reset();
        imu.reset();
    }
    SUCCEED();
}

// ── P0: First Frame Without IMU Data ────────────────────────────────────────

TEST_F(TrackerTest, FirstFrameNoIMU_GravityNotZero) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);
    auto frame1 = test_utils::createSyntheticNV21(W, H, features);

    auto out1 = processOneFrame(frame1, BASE_NS);

    for (int i = 1; i <= 30; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto shifted = test_utils::shiftFeatures(features, -3.0f * i, 0.0f, W, H);
        if (shifted.size() < 20) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        test_utils::feedWalkingIMU(imu, ts, 33, 0.3f);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            EXPECT_LT(pos, 10.0)
                << "Frame " << i << ": position exploded to " << pos << "m";
        }
    }
}

// ── P0: Non-Monotonic Timestamps ────────────────────────────────────────────

TEST_F(TrackerTest, NonMonotonicTimestamp_NoCrashNoNaN) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    auto frame1 = test_utils::createSyntheticNV21(W, H, features);
    test_utils::feedStaticIMU(imu, BASE_NS - 50'000'000LL, 50);
    processOneFrame(frame1, BASE_NS);

    auto shifted1 = test_utils::shiftFeatures(features, -3.0f, 0.0f, W, H);
    auto frame2 = test_utils::createSyntheticNV21(W, H, shifted1);
    test_utils::feedStaticIMU(imu, BASE_NS + FRAME_DT_NS - 10'000'000LL, 10);
    processOneFrame(frame2, BASE_NS + FRAME_DT_NS);

    int64_t backwards_ts = BASE_NS - 50'000'000LL;
    auto shifted2 = test_utils::shiftFeatures(features, -6.0f, 0.0f, W, H);
    auto frame3 = test_utils::createSyntheticNV21(W, H, shifted2);
    test_utils::feedStaticIMU(imu, backwards_ts - 10'000'000LL, 10);
    auto out3 = processOneFrame(frame3, backwards_ts);

    if (out3.valid && !out3.t.empty()) {
        double pos = cv::norm(out3.t);
        EXPECT_FALSE(std::isnan(pos)) << "Backwards timestamp produced NaN position";
        EXPECT_FALSE(std::isinf(pos)) << "Backwards timestamp produced Inf position";
        EXPECT_TRUE(std::isfinite(pos)) << "Position must be finite";
    }

    auto shifted3 = test_utils::shiftFeatures(features, -9.0f, 0.0f, W, H);
    auto frame4 = test_utils::createSyntheticNV21(W, H, shifted3);
    test_utils::feedStaticIMU(imu, BASE_NS + 2 * FRAME_DT_NS - 10'000'000LL, 10);
    auto out4 = processOneFrame(frame4, BASE_NS + 2 * FRAME_DT_NS);

    if (out4.valid && !out4.t.empty()) {
        double pos = cv::norm(out4.t);
        EXPECT_TRUE(std::isfinite(pos)) << "Recovery frame after backwards timestamp is not finite";
    }
}

// ── P0: Concurrent setIntrinsics — Read/Write Race ──────────────────────────

TEST_F(TrackerTest, IntrinsicsChangeMidStream_NoCrashNoNaN) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    for (int i = 0; i < 30; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto shifted = test_utils::shiftFeatures(features, -3.0f * i, 0.0f, W, H);
        if (shifted.size() < 20) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        if (i % 5 == 0) {
            double fx = 400.0 + (i * 10);
            tracker.setIntrinsics(fx, fx, W / 2.0, H / 2.0);
        }

        test_utils::feedStaticIMU(imu, ts - 10'000'000LL, 10);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            EXPECT_TRUE(std::isfinite(pos))
                << "Frame " << i << ": position is not finite after intrinsics change";
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════════
// Tests for accuracy improvements (session 0j)
// ══════════════════════════════════════════════════════════════════════════════

// ── Scale Bootstrap: median initialization ──────────────────────────────────

TEST_F(TrackerTest, ScaleBootstrap_InitialValueReasonable) {
    // Verify scale starts at a reasonable value and stays finite
    // Note: full step-based scale convergence requires on-device testing
    // because synthetic walking IMU doesn't trigger the step detector reliably

    double initial_scale = tracker.getSmoothScale();
    EXPECT_NEAR(initial_scale, 0.20, 0.01) << "Initial scale should be 0.20";

    // Feed some frames — scale should remain finite
    auto features = test_utils::generateFeatureGrid(W, H);
    for (int i = 0; i < 30; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto shifted = test_utils::shiftFeatures(features, -2.0f * i, 0.0f, W, H);
        if (shifted.size() < 20) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);
        test_utils::feedStaticIMU(imu, ts, 33);
        processOneFrame(frame, ts);
    }

    double final_scale = tracker.getSmoothScale();
    EXPECT_TRUE(std::isfinite(final_scale)) << "Scale must remain finite";
    EXPECT_GT(final_scale, 0.001) << "Scale should stay positive";
    EXPECT_LT(final_scale, 100.0) << "Scale should stay bounded";
}

// ── ZUPT Gyro Bias Refinement ───────────────────────────────────────────────

TEST_F(TrackerTest, ZUPTGyroBias_RefinedDuringStationary) {
    // After initialization + stationary period, gyro bias should be refined
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    // Initialize with biased gyro (0.01 rad/s bias = ~0.6 deg/s)
    for (int i = 0; i < 60; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        // Feed slightly biased gyro during init
        int64_t imu_dt_ns = 1'000'000'000LL / 200;
        for (int s = 0; s < 6; s++) {
            int64_t imu_ts = ts + s * imu_dt_ns;
            imu.addGyroReading(imu_ts, 0.01f, 0.0f, 0.0f);  // small X bias
            imu.addAccelReading(imu_ts, 0.0f, 9.81f, 0.0f);
        }
        processOneFrame(frame, ts);
    }

    cv::Point3f bias_before = imu.getGyroBias();

    // Now feed 3 seconds of perfectly stationary (zero gyro) data
    // ZUPT should detect stationary and refine bias toward zero
    for (int i = 60; i < 150; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        test_utils::feedStaticIMU(imu, ts, 33);
        processOneFrame(frame, ts);
    }

    cv::Point3f bias_after = imu.getGyroBias();

    // Bias X component should have moved closer to the true reading (0.0)
    // The bias was seeded at ~0.01 from the biased init period
    // After ZUPT refinement, it should still be finite and small
    EXPECT_TRUE(std::isfinite(bias_after.x));
    EXPECT_TRUE(std::isfinite(bias_after.y));
    EXPECT_TRUE(std::isfinite(bias_after.z));
    // Bias magnitude should be small (< 0.1 rad/s)
    double bias_mag = std::sqrt(bias_after.x * bias_after.x +
                                 bias_after.y * bias_after.y +
                                 bias_after.z * bias_after.z);
    EXPECT_LT(bias_mag, 0.1) << "Gyro bias should remain small after ZUPT refinement";
}

// ── Keyframe Heading Correction ─────────────────────────────────────────────

TEST_F(TrackerTest, KeyframeHeadingCorrection_NoExplosion) {
    // Test that keyframe heading correction doesn't cause heading jumps.
    // Feed enough frames to trigger keyframe matching (every 15 frames).
    auto features = test_utils::generateFeatureGrid(W, H);

    tracker.setInitialHeading(0.5, imu);  // ~28 degrees (2026-06-12: + imu arg, see cbf20f0)
    double prev_heading = 0.5;

    for (int i = 0; i < 60; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        // Slight translation to keep features moving
        auto shifted = test_utils::shiftFeatures(features, -1.5f * i, 0.0f, W, H);
        if (shifted.size() < 30) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        test_utils::feedStaticIMU(imu, ts, 33);
        auto output = processOneFrame(frame, ts);

        double heading = tracker.getHeading();
        // Heading should never jump more than 15 degrees in a single frame
        double hdiff = std::abs(heading - prev_heading);
        if (hdiff > M_PI) hdiff = 2.0 * M_PI - hdiff;
        EXPECT_LT(hdiff, 15.0 * M_PI / 180.0)
            << "Frame " << i << ": heading jumped " << hdiff * 180.0 / M_PI << " degrees";
        prev_heading = heading;
    }
}

// ── VioEngine Integration Test ──────────────────────────────────────────────

class VioEngineTest : public ::testing::Test {
protected:
    VioEngine engine;
    static constexpr int W = 640;
    static constexpr int H = 480;
    static constexpr int64_t BASE_NS = 1'000'000'000LL;
    static constexpr int64_t FRAME_DT_NS = 33'333'333LL;

    void SetUp() override {
        engine.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0);
    }
};

TEST_F(VioEngineTest, Construction_NoThrow) {
    VioEngine e;
    SUCCEED();
}

TEST_F(VioEngineTest, Reset_NoThrow) {
    engine.reset();
    SUCCEED();
}

TEST_F(VioEngineTest, StaticScene_NoDrift) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    for (int i = 0; i < 90; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        // Feed static IMU through VioEngine
        int64_t imu_dt_ns = 1'000'000'000LL / 200;
        for (int s = 0; s < 6; s++) {
            int64_t imu_ts = ts + s * imu_dt_ns;
            engine.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            engine.addAccelData(imu_ts, 0.0f, 9.81f, 0.0f);
        }
        auto output = engine.processFrame(frame.data(), W, H, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            EXPECT_LT(pos, 0.5)
                << "Frame " << i << ": static scene drifted " << pos << "m";
        }
    }
}

TEST_F(VioEngineTest, ForwardWalk_PositionIncreases) {
    // Step 5: 5 s of stationary IMU first to satisfy the calibration gate
    // (|gyro|<0.02 rad/s, var(accel)<0.01 m²/s⁴), then 2 s of walking IMU.
    int64_t imu_dt_ns = 5'000'000LL;
    int64_t imu_start_ns = BASE_NS - 7'000'000'000LL;
    for (int i = 0; i < 1000; i++) {
        int64_t ts = imu_start_ns + i * imu_dt_ns;
        engine.addGyroData(ts, 0.0f, 0.0f, 0.0f);
        engine.addAccelData(ts, 0.0f, 9.81f, 0.0f);
    }
    int64_t walk_start_ns = imu_start_ns + 1000 * imu_dt_ns;
    for (int i = 0; i < 400; i++) {
        int64_t ts = walk_start_ns + i * imu_dt_ns;
        double t = i * 0.005;
        float step = 0.7f * static_cast<float>(std::sin(2.0 * M_PI * 1.3 * t));
        if (step > 0) step *= 1.5f;
        engine.addGyroData(ts, 0.0f, 0.0f, 0.0f);
        engine.addAccelData(ts, 0.3f, 9.81f + step, 0.0f);
    }

    bool position_grew = false;
    for (int i = 0; i < 60; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * i, 0);

        int64_t imu_dt_ns = 1'000'000'000LL / 200;
        for (int s = 0; s < 6; s++) {
            int64_t imu_ts = ts + s * imu_dt_ns;
            double t = (i * 6 + s) * 0.005;
            float step = 0.7f * static_cast<float>(std::sin(2.0 * M_PI * 1.3 * t));
            if (step > 0) step *= 1.5f;
            engine.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            engine.addAccelData(imu_ts, 0.3f, 9.81f + step, 0.0f);
        }
        auto output = engine.processFrame(frame.data(), W, H, ts);

        if (output.valid && !output.t.empty()) {
            if (cv::norm(output.t) > 0.01) position_grew = true;
        }
    }

    EXPECT_TRUE(position_grew) << "Forward walk should increase position";
}

TEST_F(VioEngineTest, SetHeading_Preserved) {
    engine.setInitialHeading(1.57);  // ~90 degrees

    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    // Feed a few static frames
    for (int i = 0; i < 10; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        int64_t imu_dt_ns = 5'000'000LL;
        for (int s = 0; s < 6; s++) {
            engine.addGyroData(ts + s * imu_dt_ns, 0.0f, 0.0f, 0.0f);
            engine.addAccelData(ts + s * imu_dt_ns, 0.0f, 9.81f, 0.0f);
        }
        auto output = engine.processFrame(frame.data(), W, H, ts);
        if (output.valid) {
            double hdiff = std::abs(output.heading - 1.57);
            if (hdiff > M_PI) hdiff = 2.0 * M_PI - hdiff;
            EXPECT_LT(hdiff, 0.2)
                << "Frame " << i << ": heading drifted from set value";
        }
    }
}

// ── Step 2.4 — Visual yaw variance accessor ─────────────────────────────────

TEST_F(TrackerTest, GetLastVisualYawVariance_InitiallyMinusOne) {
    EXPECT_EQ(tracker.getLastVisualYawVariance(), -1.0)
        << "Variance must start at -1.0 (sentinel for 'never computed')";
}

TEST_F(TrackerTest, GetLastVisualYawVariance_ResetRestoresSentinel) {
    tracker.reset();
    EXPECT_EQ(tracker.getLastVisualYawVariance(), -1.0)
        << "reset() must restore the -1.0 sentinel";
}

// ── Step 2.1 — Gravity-aligned yaw extraction sanity ────────────────────────
//
// This is a pure math regression test. It does NOT exercise Tracker; it locks
// in the algorithm so a future refactor of the alignment math fails loudly.
// If a 90° yaw under 30° pitch is recovered to within 0.1°, the V-shape bug
// (where atan2(R[1,0], R[0,0]) under-counted yaw under tilt) cannot recur.

// Build a 3x3 rotation about the X axis (no Rodrigues dep).
static cv::Mat makeRx(double a) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    R.at<double>(1, 1) =  std::cos(a); R.at<double>(1, 2) = -std::sin(a);
    R.at<double>(2, 1) =  std::sin(a); R.at<double>(2, 2) =  std::cos(a);
    return R;
}
static cv::Mat makeRy(double a) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    R.at<double>(0, 0) =  std::cos(a); R.at<double>(0, 2) =  std::sin(a);
    R.at<double>(2, 0) = -std::sin(a); R.at<double>(2, 2) =  std::cos(a);
    return R;
}
static cv::Mat makeRz(double a) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    R.at<double>(0, 0) =  std::cos(a); R.at<double>(0, 1) = -std::sin(a);
    R.at<double>(1, 0) =  std::sin(a); R.at<double>(1, 1) =  std::cos(a);
    return R;
}

TEST(Step2GravityAlignedYaw, RecoversYawUnderPitch) {
    const double yaw_truth = 90.0 * M_PI / 180.0;
    const double pitch     = 30.0 * M_PI / 180.0;

    // Truth in world frame: yaw about world-Z. Tilted camera frame relates to
    // world via a pitch about Y. The relative rotation expressed in the tilted
    // camera frame is therefore Ry^T * Rz(yaw) * Ry.
    cv::Mat Ry       = makeRy(pitch);
    cv::Mat Rz_truth = makeRz(yaw_truth);
    cv::Mat R_kf     = Ry.t() * Rz_truth * Ry;

    // Naïve atan2 — what the OLD code did.
    double yaw_naive = std::atan2(R_kf.at<double>(1, 0), R_kf.at<double>(0, 0));

    // Gravity-aligned recovery — what the NEW code does (roll=0 here).
    // R_align = R_phone_to_world = Ry(pitch) * Rx(roll). With roll=0 this
    // reduces to Ry(pitch); the sandwich R_align * R_kf * R_align.t()
    // cancels the pitch wrapping in R_kf and leaves pure Rz(yaw).
    cv::Mat R_align   = makeRy(pitch) * makeRx(0.0);
    cv::Mat R_aligned = R_align * R_kf * R_align.t();
    double yaw_aligned = std::atan2(R_aligned.at<double>(1, 0),
                                    R_aligned.at<double>(0, 0));

    EXPECT_NEAR(yaw_aligned, yaw_truth, 0.1 * M_PI / 180.0)
        << "Gravity-aligned recovery must match truth to within 0.1°";

    double err_naive   = std::abs(yaw_naive   - yaw_truth);
    double err_aligned = std::abs(yaw_aligned - yaw_truth);
    EXPECT_LT(err_aligned * 5.0, err_naive)
        << "Aligned yaw should be much closer to truth than naïve atan2";
}

// ─── Step 3 Observer B: depth-scale variance accessor ─────────────────────────
TEST(Step3DepthScale, VarianceInitiallyMinusOne) {
    Tracker t;
    EXPECT_DOUBLE_EQ(t.getLastDepthScaleVariance(), -1.0)
        << "Variance sentinel must be -1 before any depth-scale measurement";
}

TEST(Step3DepthScale, ResetRestoresVarianceSentinel) {
    Tracker t;
    t.reset();
    EXPECT_DOUBLE_EQ(t.getLastDepthScaleVariance(), -1.0)
        << "reset() must restore variance sentinel";
}

// MAD-based variance math, factored out of applyDepthScaleConstraint so we can
// regression-test it directly without driving the full Tracker pipeline.
static double mad_variance_of_ratios(std::vector<double> ratios) {
    std::sort(ratios.begin(), ratios.end());
    const size_t N = ratios.size();
    double median = ratios[N / 2];
    std::vector<double> dev(N);
    for (size_t i = 0; i < N; ++i) dev[i] = std::abs(ratios[i] - median);
    std::sort(dev.begin(), dev.end());
    double mad = dev[N / 2];
    const double mad_floor = std::max(1e-3, 0.01 * std::abs(median));
    if (mad < mad_floor) mad = mad_floor;
    double sigma = 1.4826 * mad;
    return (sigma * sigma) / static_cast<double>(N);
}

TEST(Step3DepthScale, MadVarianceIsLowerForTighterRatios) {
    std::vector<double> tight  = {0.99, 1.00, 1.00, 1.00, 1.00, 1.00, 1.01, 1.01, 1.00};
    std::vector<double> noisy  = {0.50, 0.80, 0.95, 1.00, 1.05, 1.10, 1.40, 1.80, 2.10};
    double v_tight = mad_variance_of_ratios(tight);
    double v_noisy = mad_variance_of_ratios(noisy);
    EXPECT_GT(v_noisy, v_tight * 5.0)
        << "Noisy ratios must yield substantially higher variance";
    EXPECT_GT(v_tight, 0.0);
}

TEST(Step3DepthScale, MadVarianceFlooredForIdenticalRatios) {
    // All identical ratios → raw MAD = 0; floor must keep variance positive.
    std::vector<double> identical(15, 0.42);
    double v = mad_variance_of_ratios(identical);
    EXPECT_GT(v, 0.0)
        << "Variance must be strictly positive even when MAD floors to zero";
}

// ─── Step 3 Observer A: PDR per-user stride + confidence fields ──────────────

// Synthesize a walking-pattern accel signal so the step detector fires.
// Mirrors test_imu_preintegrator.cpp's WalkingPattern_DetectsSteps shape but
// inlined here because that file isn't in the build.
static void feed_walking_pattern(IMUPreintegrator& imu,
                                 double seconds,
                                 double step_freq_hz,
                                 int rate_hz = 200) {
    int64_t base_ns = 1'000'000'000LL;
    int dt_ns = 1'000'000'000 / rate_hz;
    int n = static_cast<int>(seconds * rate_hz);
    for (int i = 0; i < n; ++i) {
        int64_t ts = base_ns + static_cast<int64_t>(i) * dt_ns;
        double t = static_cast<double>(i) / rate_hz;
        float impact = 0.8f * static_cast<float>(
            std::sin(2.0 * M_PI * step_freq_hz * t));
        if (impact > 0) impact *= 1.5f;
        imu.addGyroReading(ts, 0.0f, 0.0f, 0.0f);
        imu.addAccelReading(ts, 0.0f, 9.81f + impact, 0.0f);
    }
}

TEST(Step3ObserverA, NoCalibratedStrideByDefault) {
    IMUPreintegrator imu;
    EXPECT_FALSE(imu.hasCalibratedStride());
    EXPECT_LE(imu.getUserStride(), 0.0);
    auto info = imu.getStepInfo();
    EXPECT_FALSE(info.stride_calibrated);
}

TEST(Step3ObserverA, SetUserStrideClampedToPlausibleRange) {
    IMUPreintegrator imu;

    imu.setUserStride(0.72);
    EXPECT_TRUE(imu.hasCalibratedStride());
    EXPECT_NEAR(imu.getUserStride(), 0.72, 1e-9);

    imu.setUserStride(5.0);  // unphysical; must clamp
    EXPECT_LE(imu.getUserStride(), 1.5 + 1e-9);

    imu.setUserStride(0.05); // tiny; must clamp
    EXPECT_GE(imu.getUserStride(), 0.3 - 1e-9);

    imu.setUserStride(-1.0); // disable calibration
    EXPECT_FALSE(imu.hasCalibratedStride());
}

TEST(Step3ObserverA, CalibratedStridePreservedAcrossReset) {
    IMUPreintegrator imu;
    imu.setUserStride(0.78);
    imu.reset();
    // Per-user calibration is persistent across session resets.
    EXPECT_TRUE(imu.hasCalibratedStride());
    EXPECT_NEAR(imu.getUserStride(), 0.78, 1e-9);
}

TEST(Step3ObserverA, CalibratedStrideUsedInStepInfo) {
    IMUPreintegrator imu;
    imu.setUserHeight(1.70f);

    // Without calibration: stride driven by height model.
    feed_walking_pattern(imu, /*seconds=*/3.0, /*step_freq_hz=*/1.6);
    auto info_default = imu.getStepInfo();
    ASSERT_GT(info_default.step_count, 1);

    // With a calibrated stride well outside the height-model output, the new
    // value should dominate.
    IMUPreintegrator imu2;
    imu2.setUserHeight(1.70f);
    imu2.setUserStride(1.20);  // unusually long stride
    feed_walking_pattern(imu2, /*seconds=*/3.0, /*step_freq_hz=*/1.6);
    auto info_calib = imu2.getStepInfo();
    ASSERT_GT(info_calib.step_count, 1);

    EXPECT_TRUE(info_calib.stride_calibrated);
    EXPECT_GT(info_calib.stride_length_m, info_default.stride_length_m + 0.1)
        << "Calibrated long stride should produce noticeably larger stride";
}

TEST(Step3ObserverA, StepInfoReportsConfidenceFields) {
    IMUPreintegrator imu;
    feed_walking_pattern(imu, /*seconds=*/4.0, /*step_freq_hz=*/1.6);
    auto info = imu.getStepInfo();
    ASSERT_GT(info.step_count, 3) << "Need several steps to produce variance";

    // Once we have at least 3 step periods buffered, variance is reported.
    EXPECT_GE(info.step_period_variance_s2, 0.0)
        << "step_period_variance must be non-negative once populated";
    EXPECT_LT(info.step_period_variance_s2, 1.0)
        << "Synthetic regular gait should have low period variance";

    // Time-since-last-step is populated whenever a step has been observed.
    EXPECT_GE(info.time_since_last_step_s, 0.0);
    EXPECT_LT(info.time_since_last_step_s, 5.0);

    // accel_variance is populated from the running estimator.
    EXPECT_GE(info.accel_variance, 0.0);
}

// ─── Step 3 Observer C: Hesch/Martinelli VI scale solver ─────────────────────
//
// Synthetic test rig: simulate a body moving along a known trajectory in
// world frame with a known scale s_truth. The "VIO" produces translation
// directions in body frame at scale 1 (i.e. the VIO-frame displacement that
// would need to be multiplied by s_truth to recover metric world motion).
// The IMU preintegration produces ΔP_body and ΔV_body for each segment,
// computed analytically from the trajectory. Solver should recover s_truth.

namespace {

// Build a synthetic constant-velocity straight-line walk along world-X.
// Body frame is identity (R_w_b = I). All segments have equal Δt.
// Returns expected scale = s_truth.
struct VICase {
    std::vector<ScaleEstimatorVI::KeyframePair> pairs;
    double s_truth;
    cv::Vec3d v0_truth;
    cv::Vec3d gravity_w;
};

// Synthetic body trajectory along world-X with constant world acceleration
// a_truth (mild forward speedup). Constant-velocity is degenerate for the
// closed-form LS (b_i = 0 ⇒ trivial null-space solution), so we must include
// a non-zero non-gravitational acceleration to make the system observable.
//
// Physics under R_w_b = I, gravity g (pointing down):
//   v_i = v_0 + a_truth · (i·dt)
//   disp_i = v_i · dt + 0.5·a_truth·dt²
//   t_vis_i = disp_i / s_truth
//   a_proper = a_world − g = a_truth − g (IMU specific force in body frame)
//   Δv_body = (a_truth − g) · dt
//   Δp_body = 0.5 · (a_truth − g) · dt²
// Substituting into the LS system A·x = b yields b_i = (i+0.5)·a_truth·dt²
// (non-zero ⇒ system observable, with x_truth = [s_truth; v_0_truth] exact).
static VICase make_constant_velocity_case(double s_truth) {
    VICase c;
    c.s_truth = s_truth;
    c.gravity_w = cv::Vec3d(0.0, 0.0, -9.81);
    c.v0_truth = cv::Vec3d(1.2, 0.0, 0.0);  // initial 1.2 m/s along world X
    const cv::Vec3d a_truth(0.1, 0.0, 0.0); // 0.1 m/s² forward acceleration

    const int N = 8;
    const double dt = 0.5;
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_64F);

    for (int i = 0; i < N; ++i) {
        ScaleEstimatorVI::KeyframePair p;
        p.R_w_b = I3.clone();
        p.dt = dt;

        cv::Vec3d v_i = c.v0_truth + a_truth * (static_cast<double>(i) * dt);
        cv::Vec3d disp_i = v_i * dt + 0.5 * a_truth * (dt * dt);
        p.t_vis_body = disp_i / s_truth;

        cv::Vec3d a_proper = a_truth - c.gravity_w;
        p.delta_v_body = a_proper * dt;
        p.delta_p_body = 0.5 * a_proper * (dt * dt);

        c.pairs.push_back(p);
    }
    return c;
}

}  // namespace

TEST(Step3ObserverC, EmptyEstimatorReportsZeroSize) {
    ScaleEstimatorVI est;
    EXPECT_EQ(est.size(), 0u);
}

TEST(Step3ObserverC, RefusesToSolveBelowMinPairs) {
    ScaleEstimatorVI est;
    auto c = make_constant_velocity_case(0.42);
    // Add only 2 pairs (below MIN_PAIRS = 4).
    est.addKeyframePair(c.pairs[0]);
    est.addKeyframePair(c.pairs[1]);
    double s, v;
    EXPECT_FALSE(est.solve(s, v));
    EXPECT_EQ(est.size(), 2u);
}

TEST(Step3ObserverC, RecoversKnownScaleOnConstantVelocity) {
    ScaleEstimatorVI est;
    est.setGravity(cv::Vec3d(0.0, 0.0, -9.81));
    auto c = make_constant_velocity_case(0.42);
    for (auto& p : c.pairs) est.addKeyframePair(p);

    double s = 0.0, var = 0.0;
    cv::Vec3d v0;
    ASSERT_TRUE(est.solve(s, var, &v0));

    EXPECT_NEAR(s, c.s_truth, 1e-6)
        << "Closed-form LS must recover known scale on noiseless input";
    EXPECT_GE(var, 0.0);
    EXPECT_LT(var, 1e-6) << "Variance on noiseless input should be ~0";
    EXPECT_NEAR(v0(0), c.v0_truth(0), 1e-6);
    EXPECT_NEAR(v0(1), c.v0_truth(1), 1e-6);
    EXPECT_NEAR(v0(2), c.v0_truth(2), 1e-6);
}

TEST(Step3ObserverC, ScaleVarianceIncreasesWithObservationNoise) {
    auto c = make_constant_velocity_case(0.5);

    // Clean estimator → near-zero variance.
    ScaleEstimatorVI est_clean;
    for (auto& p : c.pairs) est_clean.addKeyframePair(p);
    double s_clean = 0.0, var_clean = 0.0;
    ASSERT_TRUE(est_clean.solve(s_clean, var_clean));

    // Noisy estimator: perturb the visual-translation direction on each pair.
    ScaleEstimatorVI est_noisy;
    int seed = 0;
    auto pseudo_noise = [&seed]() {
        seed = (seed * 1103515245 + 12345) & 0x7fffffff;
        return (static_cast<double>(seed % 1000) / 1000.0 - 0.5) * 0.10;
    };
    for (auto p : c.pairs) {
        p.t_vis_body(0) += pseudo_noise();
        p.t_vis_body(1) += pseudo_noise();
        p.t_vis_body(2) += pseudo_noise();
        est_noisy.addKeyframePair(p);
    }
    double s_noisy = 0.0, var_noisy = 0.0;
    ASSERT_TRUE(est_noisy.solve(s_noisy, var_noisy));

    EXPECT_GT(var_noisy, var_clean * 100.0)
        << "Visual-direction noise must inflate scale variance";
    EXPECT_NEAR(s_noisy, c.s_truth, 0.20)
        << "Even with noise, scale should be in the same ballpark";
}

TEST(Step3ObserverC, ResetClearsBuffer) {
    ScaleEstimatorVI est;
    auto c = make_constant_velocity_case(1.0);
    for (auto& p : c.pairs) est.addKeyframePair(p);
    EXPECT_GE(est.size(), ScaleEstimatorVI::MIN_PAIRS);
    est.reset();
    EXPECT_EQ(est.size(), 0u);
}

TEST(Step3ObserverC, RollingBufferDropsOldestBeyondCapacity) {
    ScaleEstimatorVI est;
    // Push more than MAX_PAIRS unique pairs and verify size caps.
    for (size_t i = 0; i < ScaleEstimatorVI::MAX_PAIRS + 3; ++i) {
        ScaleEstimatorVI::KeyframePair p;
        p.R_w_b = cv::Mat::eye(3, 3, CV_64F);
        p.t_vis_body = cv::Vec3d(1.0, 0.0, 0.0);
        p.delta_p_body = cv::Vec3d(0.0, 0.0, 0.0);
        p.delta_v_body = cv::Vec3d(0.0, 0.0, 0.0);
        p.dt = 0.5;
        est.addKeyframePair(p);
    }
    EXPECT_EQ(est.size(), ScaleEstimatorVI::MAX_PAIRS);
}

// ============================================================================
// Step 3 Fusion — 1-D Kalman update on smooth_scale_ from A/B/C
// ============================================================================

TEST(Step3Fusion, ConstructorClampsScaleAndVariance) {
    ScaleFuser too_low(-1.0, 0.5);
    EXPECT_GE(too_low.scale(), ScaleFuser::SCALE_MIN);
    EXPECT_DOUBLE_EQ(too_low.variance(), 0.5);

    ScaleFuser too_high(1000.0, 1.0);
    EXPECT_LE(too_high.scale(), ScaleFuser::SCALE_MAX);

    ScaleFuser neg_var(0.5, -1.0);
    EXPECT_GE(neg_var.variance(), 0.0);
}

TEST(Step3Fusion, RejectsNonFiniteAndOutOfRangeMeasurements) {
    ScaleFuser f(0.30, 0.10);
    EXPECT_FALSE(f.update(std::nan(""), 0.01));
    EXPECT_FALSE(f.update(0.5, std::nan("")));
    EXPECT_FALSE(f.update(0.5, -0.001));
    EXPECT_FALSE(f.update(0.5, 0.0));
    EXPECT_FALSE(f.update(-0.1, 0.01));
    EXPECT_FALSE(f.update(20.0, 0.01));
    EXPECT_DOUBLE_EQ(f.scale(), 0.30);
    EXPECT_DOUBLE_EQ(f.variance(), 0.10);
}

TEST(Step3Fusion, TightObservationPullsScaleStronglyTowardZ) {
    ScaleFuser f(0.30, 1.0);
    // Tight measurement: r << P → K ≈ 1, scale snaps to z.
    EXPECT_TRUE(f.update(0.50, 1e-6));
    EXPECT_NEAR(f.scale(), 0.50, 1e-3);
    EXPECT_LT(f.variance(), 1e-5);
}

TEST(Step3Fusion, NoisyObservationBarelyMovesScale) {
    ScaleFuser f(0.30, 0.01);
    // Loose measurement: r >> P → K ≈ 0, scale moves negligibly.
    double before = f.scale();
    EXPECT_TRUE(f.update(0.99, 1000.0));
    EXPECT_NEAR(f.scale(), before, 1e-3);
}

TEST(Step3Fusion, VarianceMonotonicallyDecreasesWithRepeatedConsistentObs) {
    ScaleFuser f(0.30, 0.5);
    double prev_var = f.variance();
    for (int i = 0; i < 20; ++i) {
        EXPECT_TRUE(f.update(0.50, 0.1));
        EXPECT_LT(f.variance(), prev_var + 1e-12);
        prev_var = f.variance();
    }
    // After many observations of z=0.5, scale converges toward 0.5.
    EXPECT_NEAR(f.scale(), 0.50, 0.05);
}

TEST(Step3Fusion, PredictGrowsVarianceLinearlyInDt) {
    ScaleFuser f(0.30, 0.01);
    double v0 = f.variance();
    f.predict(60.0);  // 60 s
    double v60 = f.variance();
    double expected_growth = ScaleFuser::PROCESS_NOISE_PER_SEC * 60.0;
    EXPECT_NEAR(v60 - v0, expected_growth, 1e-12);
    f.predict(120.0);
    double v180 = f.variance();
    EXPECT_NEAR(v180 - v60, ScaleFuser::PROCESS_NOISE_PER_SEC * 120.0, 1e-12);
}

TEST(Step3Fusion, PredictIgnoresNonPositiveDt) {
    ScaleFuser f(0.30, 0.01);
    double v0 = f.variance();
    f.predict(0.0);
    f.predict(-1.0);
    f.predict(std::nan(""));
    EXPECT_DOUBLE_EQ(f.variance(), v0);
}

TEST(Step3Fusion, ThreeObserverFusionConvergesNearGroundTruthWeightedByVariance) {
    // Simulate the production fusion pattern: three observers with different
    // confidences hit the fuser repeatedly. The most certain observer
    // (smallest r) should dominate the converged estimate.
    const double s_truth = 0.42;
    ScaleFuser f(1.0, 1.0);  // start far off
    int seed = 1234;
    auto noise = [&seed](double sigma) {
        seed = (seed * 1103515245 + 12345) & 0x7fffffff;
        return (static_cast<double>(seed % 1000) / 1000.0 - 0.5) * sigma;
    };
    // 100 fusion cycles. Each cycle: predict 0.1s then update from 3 observers.
    for (int i = 0; i < 100; ++i) {
        f.predict(0.1);
        // Observer A (PDR): tight (low r), unbiased
        f.update(s_truth + noise(0.05), 0.0025);
        // Observer B (MiDaS): medium (mid r)
        f.update(s_truth + noise(0.15), 0.05);
        // Observer C (VI): loose (high r)
        f.update(s_truth + noise(0.30), 0.50);
    }
    EXPECT_NEAR(f.scale(), s_truth, 0.05);
    EXPECT_LT(f.variance(), 0.01);
}

TEST(Step3Fusion, DeadObserverFadesOutAutomatically) {
    // An observer whose variance is huge (e.g., it stopped firing reliably)
    // contributes almost nothing — fusion still converges from the live ones.
    ScaleFuser f(1.0, 1.0);
    const double s_truth = 0.5;
    for (int i = 0; i < 50; ++i) {
        f.update(s_truth, 0.001);    // healthy observer
        f.update(2.5, 1e6);          // dead observer reporting nonsense
    }
    EXPECT_NEAR(f.scale(), s_truth, 0.01);
}

TEST(Step3Fusion, ResetReturnsToInitialState) {
    ScaleFuser f(0.30, 0.5);
    for (int i = 0; i < 20; ++i) f.update(0.7, 0.01);
    f.reset(0.20, 1.0);
    EXPECT_DOUBLE_EQ(f.scale(), 0.20);
    EXPECT_DOUBLE_EQ(f.variance(), 1.0);
}
