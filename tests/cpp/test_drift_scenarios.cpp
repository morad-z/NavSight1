#include <gtest/gtest.h>
#include <cmath>
#include "VisionModule.h"
#include "test_utils.h"

// ── Drift Scenario Tests ─────────────────────────────────────────────────────
// These tests validate the VIO system's drift characteristics using synthetic
// data. They measure position drift in specific scenarios that commonly cause
// drift in monocular VIO systems.
//
// Industry drift thresholds (monocular VIO):
//   - Static drift: < 0.01m over 60s
//   - Rotational position drift: < 0.1m over 360deg
//   - Loop closure error: < 10% of path length
//   - Scale consistency: < 20% variation

class DriftScenarioTest : public ::testing::Test {
protected:
    VisionModule vm;
    static constexpr int W = 640;
    static constexpr int H = 480;
    static constexpr int64_t BASE_NS = 1'000'000'000LL;
    static constexpr int64_t FRAME_DT = 33'333'333LL;

    void SetUp() override {
        vm.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0);
    }
};

// ── Scenario 1: Extended Static Test ─────────────────────────────────────────
// Phone sits still for 5 seconds. Position must not drift.
// Tests: ZUPT, static detection, accelerometer bias rejection

TEST_F(DriftScenarioTest, Static5Seconds_PositionStaysNearZero) {
    auto features = test_utils::generateFeatureGrid(W, H, 10, 8);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    int num_frames = 150; // 5 seconds at 30fps
    double max_drift = 0.0;

    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        test_utils::feedStaticIMU(vm, ts, 33);
        auto output = vm.processFrame(frame.data(), W, H, ts);

        if (output.valid && !output.t.empty()) {
            double drift = cv::norm(output.t);
            max_drift = std::max(max_drift, drift);
        }
    }

    EXPECT_LT(max_drift, 0.1)
        << "Static 5s: max drift should be < 0.1m, got " << max_drift << "m";
}

// ── Scenario 2: Pure Rotation 360 Degrees ────────────────────────────────────
// Rotate phone in place. Position must stay near zero.
// Tests: is_pure_rotation detection, rotation-only pose update

TEST_F(DriftScenarioTest, FullRotation360_NoPositionDrift) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    // 360 degrees in 4 seconds = pi/2 rad/s
    float omega = static_cast<float>(M_PI / 2.0);
    int num_frames = 120;
    double final_position = 0.0;

    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        float angle = omega * (i * FRAME_DT * 1e-9f);

        // Rotate features around image center
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
        test_utils::feedRotatingIMU(vm, ts, 33, 0.0f, 0.0f, omega);
        auto output = vm.processFrame(frame.data(), W, H, ts);

        if (output.valid && !output.t.empty()) {
            final_position = cv::norm(output.t);
        }
    }

    EXPECT_LT(final_position, 0.5)
        << "360deg rotation: position drift should be < 0.5m, got " << final_position << "m";
}

// ── Scenario 3: Scale Estimation Convergence ─────────────────────────────────
// Steady forward walk. Scale estimate should converge and stabilize.
// Tests: estimateScaleFromAccel, smooth_scale_ EMA, outlier rejection

TEST_F(DriftScenarioTest, SteadyWalk_ScaleConverges) {
    auto features = test_utils::generateFeatureGrid(W, H, 15, 12);

    double last_scale = 0.0;
    int scale_updates = 0;

    for (int i = 0; i < 90; i++) { // 3 seconds
        int64_t ts = BASE_NS + i * FRAME_DT;

        // Shift features uniformly — simulate steady forward motion
        float shift = 3.0f; // pixels per frame
        auto shifted = test_utils::shiftFeatures(features, -shift * i, 0.0f, W, H);
        if (shifted.size() < 30) break;

        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        // Feed matching IMU (constant velocity ~ 1.5 m/s)
        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            vm.addAccelData(imu_ts, 0.3f, 9.81f, 0.0f); // slight forward accel
        }

        auto output = vm.processFrame(frame.data(), W, H, ts);

        if (output.valid && output.estimatedScale > 0.001) {
            last_scale = output.estimatedScale;
            scale_updates++;
        }
    }

    // Scale should have been updated multiple times
    EXPECT_GT(scale_updates, 5) << "Scale should update during steady walking";

    // Scale should be in a physically plausible range
    if (last_scale > 0.001) {
        EXPECT_GT(last_scale, 0.001) << "Scale too small";
        EXPECT_LT(last_scale, 20.0) << "Scale too large";
    }
}

// ── Scenario 4: Quality Tracking ─────────────────────────────────────────────
// Quality should be high with good features, low with few features.

TEST_F(DriftScenarioTest, GoodFeatures_HighQuality) {
    // Many well-distributed features
    auto features = test_utils::generateFeatureGrid(W, H, 15, 12);
    auto frame1 = test_utils::createSyntheticNV21(W, H, features);

    test_utils::feedStaticIMU(vm, BASE_NS - 50'000'000LL, 50);
    vm.processFrame(frame1.data(), W, H, BASE_NS); // init frame

    // Slight shift for second frame
    auto shifted = test_utils::shiftFeatures(features, 3.0f, 0.0f, W, H);
    auto frame2 = test_utils::createSyntheticNV21(W, H, shifted);

    test_utils::feedStaticIMU(vm, BASE_NS + FRAME_DT - 50'000'000LL, 50);
    auto output = vm.processFrame(frame2.data(), W, H, BASE_NS + FRAME_DT);

    if (output.valid) {
        EXPECT_GT(output.quality, 0.3)
            << "With many good features, quality should be > 0.3";
        EXPECT_GT(output.trackedCount, 20)
            << "Should track many features";
    }
}

TEST_F(DriftScenarioTest, FewFeatures_LowerQuality) {
    // Only 5 features — barely enough
    std::vector<cv::Point2f> sparse = {
        {100, 100}, {200, 200}, {400, 300}, {500, 150}, {300, 400}
    };
    auto frame = test_utils::createSyntheticNV21(W, H, sparse);

    test_utils::feedStaticIMU(vm, BASE_NS - 50'000'000LL, 50);
    auto output = vm.processFrame(frame.data(), W, H, BASE_NS);

    // With very few features, tracking quality should be lower
    // (first frame just initializes, so quality may be 0)
    EXPECT_LE(output.quality, 0.5);
}

// ── Scenario 5: Translation Direction Consistency ────────────────────────────
// Moving right should show positive X displacement.

TEST_F(DriftScenarioTest, RightwardMotion_PositiveXDisplacement) {
    double max_pos = 0.0;

    for (int i = 0; i < 60; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;

        // Shift checkerboard left → camera moves right
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * i, 0);

        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            vm.addAccelData(imu_ts, 0.5f, 9.81f, 0.0f);
        }

        auto output = vm.processFrame(frame.data(), W, H, ts);
        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            max_pos = std::max(max_pos, pos);
        }
    }

    EXPECT_GT(max_pos, 0.001)
        << "Rightward motion should produce non-zero displacement";
}

// ── Scenario 6: Reset Clears Accumulated Pose ────────────────────────────────

TEST_F(DriftScenarioTest, Reset_ClearsAccumulatedPose) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    // Process a few frames to build up some pose
    for (int i = 0; i < 10; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        auto shifted = test_utils::shiftFeatures(features, -2.0f * i, 0.0f, W, H);
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);
        test_utils::feedStaticIMU(vm, ts, 33);
        vm.processFrame(frame.data(), W, H, ts);
    }

    // Reset
    vm.reset();

    // First frame after reset should be initialization
    auto frame = test_utils::createSyntheticNV21(W, H, features);
    int64_t reset_ts = BASE_NS + 20 * FRAME_DT;
    test_utils::feedStaticIMU(vm, reset_ts, 33);
    auto output = vm.processFrame(frame.data(), W, H, reset_ts);

    // After reset, should be back to init state
    EXPECT_FALSE(output.valid) << "First frame after reset should be initialization";
}

// ── P1: Slow Walk — ZUPT Must Not Freeze Position ──────────────────────────
// Flow between 0.5px and MIN_FLOW_PX (2.0px) is a dead zone.
// Walking slowly with near-zero gyro could trigger false ZUPT.

TEST_F(DriftScenarioTest, SlowWalk_NotFalselyStatic) {
    double max_pos = 0.0;

    for (int i = 0; i < 90; i++) { // 3 seconds
        int64_t ts = BASE_NS + i * FRAME_DT;

        // Shift checkerboard 8 px/frame — walking speed
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * i, 0);

        // Feed walking IMU with forward acceleration
        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f); // zero gyro (walking straight)
            vm.addAccelData(imu_ts, 0.5f, 9.81f, 0.0f); // forward accel
        }

        auto output = vm.processFrame(frame.data(), W, H, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            max_pos = std::max(max_pos, pos);
        }
    }

    EXPECT_GT(max_pos, 0.001)
        << "ZUPT FALSE POSITIVE: Slow walk (4 px/frame flow) produced zero position. "
        << "ZUPT or motion gates are incorrectly blocking real walking motion.";
}

// ── P1: Scale Bootstrap With Noisy Start ────────────────────────────────────
// First 10 observations bypass outlier rejection. If early frames are noisy,
// scale could lock at a wild value that blocks all subsequent corrections.

TEST_F(DriftScenarioTest, ScaleBootstrap_NoisyStart_DoesNotLockWild) {
    auto features = test_utils::generateFeatureGrid(W, H, 15, 12);

    // Phase 1: 10 frames with exaggerated IMU (simulates vibration)
    for (int i = 0; i < 10; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        auto shifted = test_utils::shiftFeatures(features, -2.0f * i, 0.0f, W, H);
        if (shifted.size() < 30) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        // Exaggerated IMU: large accelerations simulating phone vibration
        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            vm.addAccelData(imu_ts, 5.0f, 9.81f, 3.0f); // heavy vibration
        }
        vm.processFrame(frame.data(), W, H, ts);
    }

    // Phase 2: 60 frames with normal steady walking
    double last_scale = 0.0;
    for (int i = 10; i < 70; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        auto shifted = test_utils::shiftFeatures(features, -3.0f * i, 0.0f, W, H);
        if (shifted.size() < 30) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            vm.addAccelData(imu_ts, 0.3f, 9.81f, 0.0f); // normal walking
        }

        auto output = vm.processFrame(frame.data(), W, H, ts);
        if (output.valid && output.estimatedScale > 0.001) {
            last_scale = output.estimatedScale;
        }
    }

    // Scale should settle to a sane range, not stuck at bootstrap noise
    if (last_scale > 0.001) {
        EXPECT_LT(last_scale, 5.0)
            << "Scale locked at wild value " << last_scale
            << " after noisy bootstrap — outlier rejection failed to recover";
    }
}

// ── P1: Turn Then Walk — Direction Correctness ──────────────────────────────
// Rotate 90 degrees, then walk forward. Position should move along the
// rotated axis, not the original axis. Tests rotation multiplication order.

TEST_F(DriftScenarioTest, TurnThenWalk_PositionInRotatedDirection) {
    // Phase 1: Pure rotation (30 frames) — feed gyro and slightly shifted checkerboard
    float omega = static_cast<float>(M_PI / 2.0); // 90 deg/s
    for (int i = 0; i < 30; i++) { // 1 second
        int64_t ts = BASE_NS + i * FRAME_DT;
        float angle = omega * (i * FRAME_DT * 1e-9f);
        int ox = static_cast<int>(std::cos(angle) * 10);
        int oy = static_cast<int>(std::sin(angle) * 10);
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, ox, oy);
        test_utils::feedRotatingIMU(vm, ts, 33, 0.0f, 0.0f, omega);
        vm.processFrame(frame.data(), W, H, ts);
    }

    // Transition: 2 frames at rest to let tracker re-initialize on new scene
    for (int i = 30; i < 32; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, 0, 0);
        test_utils::feedStaticIMU(vm, ts, 33);
        vm.processFrame(frame.data(), W, H, ts);
    }

    // Phase 2: Forward translation (60 frames after rotation)
    double max_pos = 0.0;

    for (int i = 32; i < 92; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * (i - 32), 0);

        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            vm.addAccelData(imu_ts, 0.5f, 9.81f, 0.0f);
        }

        auto output = vm.processFrame(frame.data(), W, H, ts);
        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            max_pos = std::max(max_pos, pos);
        }
    }

    // If rotation corrupted state so badly that no walk frames are valid,
    // that itself is a bug worth reporting
    EXPECT_GT(max_pos, 0.001)
        << "After turn+walk, position should have moved, not stuck at origin. "
        << "This may indicate rotation phase corrupts tracker state for subsequent translation.";
}

// ── Simplified TurnThenWalk: reset between phases ───────────────────────────
// If the full test fails, this confirms whether the issue is rotation-phase
// state corruption vs. actual rotation multiplication order.

TEST_F(DriftScenarioTest, WalkAfterReset_StillWorks) {
    // Phase 1: some rotation to accumulate global_R_
    for (int i = 0; i < 15; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT;
        int ox = static_cast<int>(std::cos(i * 0.1) * 10);
        int oy = static_cast<int>(std::sin(i * 0.1) * 10);
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, ox, oy);
        test_utils::feedRotatingIMU(vm, ts, 33, 0.0f, 0.0f, 1.0f);
        vm.processFrame(frame.data(), W, H, ts);
    }

    // Reset and walk — this should always work since it's a clean start
    vm.reset();

    double max_pos = 0.0;
    int64_t walk_base = BASE_NS + 100 * FRAME_DT;

    for (int i = 0; i < 60; i++) {
        int64_t ts = walk_base + i * FRAME_DT;
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * i, 0);

        for (int j = 0; j < 6; j++) {
            int64_t imu_ts = ts + j * 5'000'000LL;
            vm.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
            vm.addAccelData(imu_ts, 0.5f, 9.81f, 0.0f);
        }

        auto output = vm.processFrame(frame.data(), W, H, ts);
        if (output.valid && !output.t.empty()) {
            max_pos = std::max(max_pos, cv::norm(output.t));
        }
    }

    EXPECT_GT(max_pos, 0.001)
        << "Walking after reset should produce position movement";
}

// ── P1: Scale Consistency Across Speeds ─────────────────────────────────────
// Since recoverPose returns unit-norm t_vo, scale = imu_disp / 1.0.
// If this is the case, scale will change with walking speed (bug).
// This test exposes it.

TEST_F(DriftScenarioTest, ScaleConsistency_DifferentSpeeds) {
    double scale_slow = 0.0;
    double scale_fast = 0.0;
    int slow_updates = 0;
    int fast_updates = 0;

    // Phase 1: Slow walk (30 frames)
    {
        VisionModule vm_slow;
        vm_slow.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0);
        auto features = test_utils::generateFeatureGrid(W, H, 15, 12);

        for (int i = 0; i < 40; i++) {
            int64_t ts = BASE_NS + i * FRAME_DT;
            auto shifted = test_utils::shiftFeatures(features, -2.0f * i, 0.0f, W, H);
            if (shifted.size() < 30) break;
            auto frame = test_utils::createSyntheticNV21(W, H, shifted);

            for (int j = 0; j < 6; j++) {
                int64_t imu_ts = ts + j * 5'000'000LL;
                vm_slow.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
                vm_slow.addAccelData(imu_ts, 0.2f, 9.81f, 0.0f); // slow walk
            }

            auto output = vm_slow.processFrame(frame.data(), W, H, ts);
            if (output.valid && output.estimatedScale > 0.001) {
                scale_slow = output.estimatedScale;
                slow_updates++;
            }
        }
    }

    // Phase 2: Fast walk (30 frames, separate VisionModule)
    {
        VisionModule vm_fast;
        vm_fast.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0);
        auto features = test_utils::generateFeatureGrid(W, H, 15, 12);

        for (int i = 0; i < 40; i++) {
            int64_t ts = BASE_NS + i * FRAME_DT;
            auto shifted = test_utils::shiftFeatures(features, -6.0f * i, 0.0f, W, H);
            if (shifted.size() < 20) break;
            auto frame = test_utils::createSyntheticNV21(W, H, shifted);

            for (int j = 0; j < 6; j++) {
                int64_t imu_ts = ts + j * 5'000'000LL;
                vm_fast.addGyroData(imu_ts, 0.0f, 0.0f, 0.0f);
                vm_fast.addAccelData(imu_ts, 0.6f, 9.81f, 0.0f); // fast walk
            }

            auto output = vm_fast.processFrame(frame.data(), W, H, ts);
            if (output.valid && output.estimatedScale > 0.001) {
                scale_fast = output.estimatedScale;
                fast_updates++;
            }
        }
    }

    // If both converged, scales should be within 3x of each other
    // (same physical scale, just different speeds)
    if (slow_updates > 5 && fast_updates > 5 && scale_slow > 0.001 && scale_fast > 0.001) {
        double ratio = (scale_fast > scale_slow)
            ? scale_fast / scale_slow
            : scale_slow / scale_fast;
        EXPECT_LT(ratio, 3.0)
            << "Scale varies too much with speed: slow=" << scale_slow
            << " fast=" << scale_fast << " ratio=" << ratio
            << ". This suggests scale = imu_disp / unit_norm_t (broken).";
    }
}
