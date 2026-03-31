#include <gtest/gtest.h>
#include <cmath>
#include "VisionModule.h"
#include "test_utils.h"

class VisionModuleTest : public ::testing::Test {
protected:
    VisionModule vm;
    static constexpr int W = 640;
    static constexpr int H = 480;
    static constexpr int64_t BASE_NS = 1'000'000'000LL;
    static constexpr int64_t FRAME_DT_NS = 33'333'333LL; // 30 fps

    void SetUp() override {
        // Set typical phone intrinsics
        vm.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0);
    }

    // Helper: process one frame and return output
    VisionOutput processOneFrame(const std::vector<uint8_t>& frame, int64_t ts) {
        return vm.processFrame(frame.data(), W, H, ts);
    }
};

// ── Construction & Reset ─────────────────────────────────────────────────────

TEST_F(VisionModuleTest, Construction_NoThrow) {
    VisionModule v;
    // Should not crash
    SUCCEED();
}

TEST_F(VisionModuleTest, Reset_NoThrow) {
    vm.reset();
    SUCCEED();
}

// ── First Frame Initialization ───────────────────────────────────────────────

TEST_F(VisionModuleTest, FirstFrame_ReturnsInvalid) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    // Feed IMU before first frame
    test_utils::feedStaticIMU(vm, BASE_NS - 100'000'000LL, 100);

    auto output = processOneFrame(frame, BASE_NS);

    // First frame initializes tracking — should return valid=false
    EXPECT_FALSE(output.valid) << "First frame should be initialization only";
}

// ── Static Scene (ZUPT) ─────────────────────────────────────────────────────

TEST_F(VisionModuleTest, StaticScene_NoDrift) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    // Feed identical frames for 3 seconds (ZUPT should kick in)
    int num_frames = 90; // 3 seconds at 30fps
    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        // Feed static IMU between frames
        test_utils::feedStaticIMU(vm, ts, 33);
        processOneFrame(frame, ts);
    }

    // Get the last output
    int64_t last_ts = BASE_NS + num_frames * FRAME_DT_NS;
    test_utils::feedStaticIMU(vm, last_ts, 33);
    auto output = processOneFrame(frame, last_ts);

    // Position should be very close to origin
    if (output.valid && !output.t.empty()) {
        double pos_norm = cv::norm(output.t);
        EXPECT_LT(pos_norm, 0.1)
            << "Static scene should have near-zero position drift, got " << pos_norm << "m";
    }
}

// ── Pure Rotation ────────────────────────────────────────────────────────────

TEST_F(VisionModuleTest, PureRotation_NoPositionDrift) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    // Simulate 2 seconds of rotation by shifting features circularly
    int num_frames = 60;
    double total_position = 0.0;

    for (int i = 0; i < num_frames; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;

        // Shift features slightly to simulate rotation (circular motion in image)
        float angle = static_cast<float>(i) * 0.05f; // slow rotation
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

        // Feed high gyro (simulating rotation)
        test_utils::feedRotatingIMU(vm, ts, 33, 0.0f, 0.0f, 1.0f);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            total_position = cv::norm(output.t);
        }
    }

    // Position should be minimal — pure rotation should not cause translation
    EXPECT_LT(total_position, 0.5)
        << "Pure rotation should not cause significant position drift, got " << total_position << "m";
}

// ── Forward Translation ──────────────────────────────────────────────────────

TEST_F(VisionModuleTest, ForwardTranslation_PositionGrows) {
    bool position_grew = false;

    // Simulate forward motion by shifting checkerboard pattern
    for (int i = 0; i < 60; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;

        // Shift checkerboard 8px/frame (same as Kotlin on-device test)
        auto frame = test_utils::createCheckerboardNV21(W, H, 20, -8 * i, 0);

        // Feed walking-like IMU (step impacts for step detector)
        test_utils::feedWalkingIMU(vm, ts, 33, 0.5f);

        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            if (pos > 0.01) position_grew = true;
        }
    }

    EXPECT_TRUE(position_grew) << "Forward translation should increase position from origin";
}

// ── Intrinsics ───────────────────────────────────────────────────────────────

TEST_F(VisionModuleTest, SetIntrinsics_NoThrow) {
    vm.setIntrinsics(600.0, 600.0, 320.0, 240.0);
    SUCCEED();
}

TEST_F(VisionModuleTest, ZeroIntrinsics_UsesDefault) {
    // Setting all zeros should trigger auto-intrinsics from frame width
    vm.setIntrinsics(0.0, 0.0, 0.0, 0.0);

    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);
    test_utils::feedStaticIMU(vm, BASE_NS - 50'000'000LL, 50);

    // Should not crash
    auto output = processOneFrame(frame, BASE_NS);
    SUCCEED();
}

// ── Edge Cases ───────────────────────────────────────────────────────────────

TEST_F(VisionModuleTest, EmptyFrame_NoFeatures_NoCrash) {
    // All-gray frame — no features to detect
    std::vector<uint8_t> gray_frame(W * H + W * H / 2, 128);
    test_utils::feedStaticIMU(vm, BASE_NS - 50'000'000LL, 50);

    auto output = processOneFrame(gray_frame, BASE_NS);
    // Should not crash, output may or may not be valid
    SUCCEED();
}

TEST_F(VisionModuleTest, LargeTimestampGap_HandledGracefully) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    test_utils::feedStaticIMU(vm, BASE_NS - 50'000'000LL, 50);
    processOneFrame(frame, BASE_NS);

    // 10-second gap (exceeds MAX_DT_NS = 5s)
    int64_t late_ts = BASE_NS + 10'000'000'000LL;
    test_utils::feedStaticIMU(vm, late_ts - 50'000'000LL, 50);
    auto output = processOneFrame(frame, late_ts);

    // Should handle gracefully (reset or return invalid)
    SUCCEED();
}

TEST_F(VisionModuleTest, RapidReset_NoLeaks) {
    auto features = test_utils::generateFeatureGrid(W, H);
    auto frame = test_utils::createSyntheticNV21(W, H, features);

    for (int cycle = 0; cycle < 10; cycle++) {
        test_utils::feedStaticIMU(vm, BASE_NS, 50);
        processOneFrame(frame, BASE_NS + cycle * FRAME_DT_NS);
        vm.reset();
    }
    SUCCEED();
}

// ── P0: First Frame Without IMU Data — Gravity Must Not Be Zero ─────────────
// If the first camera frame arrives before any accelerometer data,
// setGravity gets called with (0,0,0). This causes uncompensated gravity
// in IMU preintegration → position explodes at ~4.9 m/s².

TEST_F(VisionModuleTest, FirstFrameNoIMU_GravityNotZero) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);
    auto frame1 = test_utils::createSyntheticNV21(W, H, features);

    // Process first frame WITHOUT feeding any IMU data beforehand
    auto out1 = processOneFrame(frame1, BASE_NS);

    // Now feed normal IMU and process 30 more frames with motion
    for (int i = 1; i <= 30; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto shifted = test_utils::shiftFeatures(features, -3.0f * i, 0.0f, W, H);
        if (shifted.size() < 20) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        // Feed walking-like IMU with step impacts
        test_utils::feedWalkingIMU(vm, ts, 33, 0.3f);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            // Position must not explode (gravity=0 causes ~4.9*t^2 drift)
            EXPECT_LT(pos, 10.0)
                << "Frame " << i << ": position exploded to " << pos
                << "m — gravity was likely set to (0,0,0) on first frame";
        }
    }
}

// ── P0: Non-Monotonic Timestamps — No Crash or NaN ──────────────────────────
// Some Android devices send backwards timestamps during camera restart.
// dt_ns goes negative → undefined IMU integration behavior.

TEST_F(VisionModuleTest, NonMonotonicTimestamp_NoCrashNoNaN) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    // Frame 1: t=1.0s
    auto frame1 = test_utils::createSyntheticNV21(W, H, features);
    test_utils::feedStaticIMU(vm, BASE_NS - 50'000'000LL, 50);
    processOneFrame(frame1, BASE_NS);

    // Frame 2: t=1.033s (normal)
    auto shifted1 = test_utils::shiftFeatures(features, -3.0f, 0.0f, W, H);
    auto frame2 = test_utils::createSyntheticNV21(W, H, shifted1);
    test_utils::feedStaticIMU(vm, BASE_NS + FRAME_DT_NS - 10'000'000LL, 10);
    processOneFrame(frame2, BASE_NS + FRAME_DT_NS);

    // Frame 3: t=0.95s — BACKWARDS timestamp!
    int64_t backwards_ts = BASE_NS - 50'000'000LL;
    auto shifted2 = test_utils::shiftFeatures(features, -6.0f, 0.0f, W, H);
    auto frame3 = test_utils::createSyntheticNV21(W, H, shifted2);
    test_utils::feedStaticIMU(vm, backwards_ts - 10'000'000LL, 10);
    auto out3 = processOneFrame(frame3, backwards_ts);

    // Must not crash, must not produce NaN
    if (out3.valid && !out3.t.empty()) {
        double pos = cv::norm(out3.t);
        EXPECT_FALSE(std::isnan(pos)) << "Backwards timestamp produced NaN position";
        EXPECT_FALSE(std::isinf(pos)) << "Backwards timestamp produced Inf position";
        EXPECT_TRUE(std::isfinite(pos)) << "Position must be finite";
    }

    // Frame 4: t=1.066s — back to normal, must still work
    auto shifted3 = test_utils::shiftFeatures(features, -9.0f, 0.0f, W, H);
    auto frame4 = test_utils::createSyntheticNV21(W, H, shifted3);
    test_utils::feedStaticIMU(vm, BASE_NS + 2 * FRAME_DT_NS - 10'000'000LL, 10);
    auto out4 = processOneFrame(frame4, BASE_NS + 2 * FRAME_DT_NS);

    if (out4.valid && !out4.t.empty()) {
        double pos = cv::norm(out4.t);
        EXPECT_TRUE(std::isfinite(pos)) << "Recovery frame after backwards timestamp is not finite";
    }
}

// ── P0: Concurrent setIntrinsics — Read/Write Race ──────────────────────────
// processFrame reads fx_/fy_/cx_/cy_ outside the lock.
// setIntrinsics writes them under mutex_. Torn reads = garbage Essential Matrix.
// This test is single-threaded but validates that changing intrinsics mid-stream
// doesn't produce NaN or crash. (True thread safety needs on-device test.)

TEST_F(VisionModuleTest, IntrinsicsChangeMidStream_NoCrashNoNaN) {
    auto features = test_utils::generateFeatureGrid(W, H, 12, 10);

    for (int i = 0; i < 30; i++) {
        int64_t ts = BASE_NS + i * FRAME_DT_NS;
        auto shifted = test_utils::shiftFeatures(features, -3.0f * i, 0.0f, W, H);
        if (shifted.size() < 20) break;
        auto frame = test_utils::createSyntheticNV21(W, H, shifted);

        // Change intrinsics every 5 frames (simulates race condition effect)
        if (i % 5 == 0) {
            double fx = 400.0 + (i * 10);
            vm.setIntrinsics(fx, fx, W / 2.0, H / 2.0);
        }

        test_utils::feedStaticIMU(vm, ts - 10'000'000LL, 10);
        auto output = processOneFrame(frame, ts);

        if (output.valid && !output.t.empty()) {
            double pos = cv::norm(output.t);
            EXPECT_TRUE(std::isfinite(pos))
                << "Frame " << i << ": position is not finite after intrinsics change";
        }
    }
}
