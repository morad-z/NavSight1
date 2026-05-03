// Step 5: stationary-gate calibration tests for InertialInitializer.
//
// Plan spec:
//   - 5 s of |gyro| < 0.02 rad/s and var(accel) < 0.01 m²/s⁴ → WAIT_MOTION
//   - >15 s without passing the gate → TIMEOUT_NEEDS_USER
//   - clearTimeout() restarts the gate
//   - loadCalibration() bypasses the gate and lands in WAIT_MOTION
//   - First step after WAIT_MOTION → READY

#include <gtest/gtest.h>
#include <random>
#include "InertialInitializer.h"

namespace {

constexpr double kImuHz = 200.0;
constexpr int64_t kImuDtNs = static_cast<int64_t>(1e9 / kImuHz);

void feed_quiet(InertialInitializer& init, double seconds, int64_t& ts_ns,
                double accel_noise = 0.02, double gyro_noise = 0.005) {
    std::mt19937 rng(123);
    std::normal_distribution<double> a_n(0.0, accel_noise);
    std::normal_distribution<double> g_n(0.0, gyro_noise);
    int n = static_cast<int>(std::round(seconds * kImuHz));
    for (int i = 0; i < n; ++i) {
        init.addImuData(ts_ns,
                        static_cast<float>(a_n(rng)),
                        static_cast<float>(9.81 + a_n(rng)),
                        static_cast<float>(a_n(rng)),
                        static_cast<float>(g_n(rng)),
                        static_cast<float>(g_n(rng)),
                        static_cast<float>(g_n(rng)));
        ts_ns += kImuDtNs;
    }
}

void feed_noisy(InertialInitializer& init, double seconds, int64_t& ts_ns) {
    feed_quiet(init, seconds, ts_ns, /*accel_noise=*/0.5, /*gyro_noise=*/0.3);
}

void feed_step(InertialInitializer& init, int64_t& ts_ns) {
    init.addImuData(ts_ns, 0.0f, 11.0f, 0.0f, 0.01f, 0.01f, 0.01f);
    ts_ns += kImuDtNs;
}

}  // namespace

TEST(InertialInitializerTest, QuietFiveSeconds_PassesGate) {
    InertialInitializer init;
    int64_t ts_ns = 1'000'000'000LL;

    feed_quiet(init, 5.5, ts_ns);

    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_MOTION);
    EXPECT_FALSE(init.needsUserPrompt());
    cv::Point3f gb = init.getGyroBias();
    EXPECT_LT(std::abs(gb.x), 0.02f);
    EXPECT_LT(std::abs(gb.y), 0.02f);
    EXPECT_LT(std::abs(gb.z), 0.02f);
}

TEST(InertialInitializerTest, FirstStep_TransitionsToReady) {
    InertialInitializer init;
    int64_t ts_ns = 1'000'000'000LL;

    feed_quiet(init, 5.5, ts_ns);
    ASSERT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_MOTION);

    feed_step(init, ts_ns);
    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::READY);
    EXPECT_TRUE(init.isReady());
}

TEST(InertialInitializerTest, NoisyFifteenSeconds_TriggersTimeout) {
    InertialInitializer init;
    int64_t ts_ns = 1'000'000'000LL;

    feed_noisy(init, 16.0, ts_ns);

    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::TIMEOUT_NEEDS_USER);
    EXPECT_TRUE(init.needsUserPrompt());
}

TEST(InertialInitializerTest, ClearTimeout_RestartsGate) {
    InertialInitializer init;
    int64_t ts_ns = 1'000'000'000LL;

    feed_noisy(init, 16.0, ts_ns);
    ASSERT_EQ(init.getStatus(), InertialInitializer::Status::TIMEOUT_NEEDS_USER);

    init.clearTimeout();
    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_STATIONARY);

    feed_quiet(init, 5.5, ts_ns);
    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_MOTION);
}

TEST(InertialInitializerTest, LoadCalibration_SkipsGate) {
    InertialInitializer init;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Point3f gb(0.001f, -0.002f, 0.003f);
    cv::Point3f ab(0.0f, 0.0f, 0.0f);

    init.loadCalibration(R, gb, ab);

    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_MOTION);
    cv::Point3f got = init.getGyroBias();
    EXPECT_NEAR(got.x, gb.x, 1e-6f);
    EXPECT_NEAR(got.y, gb.y, 1e-6f);
    EXPECT_NEAR(got.z, gb.z, 1e-6f);
}

TEST(InertialInitializerTest, RotatingGyro_RejectsGate) {
    InertialInitializer init;
    int64_t ts_ns = 1'000'000'000LL;

    // 0.1 rad/s constant rotation — exceeds 0.02 rad/s gate.
    int n = static_cast<int>(std::round(5.5 * kImuHz));
    for (int i = 0; i < n; ++i) {
        init.addImuData(ts_ns, 0.0f, 9.81f, 0.0f, 0.0f, 0.1f, 0.0f);
        ts_ns += kImuDtNs;
    }

    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_STATIONARY);
}

TEST(InertialInitializerTest, Reset_ReturnsToWaitStationary) {
    InertialInitializer init;
    int64_t ts_ns = 1'000'000'000LL;
    feed_quiet(init, 5.5, ts_ns);
    ASSERT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_MOTION);

    init.reset();
    EXPECT_EQ(init.getStatus(), InertialInitializer::Status::WAIT_STATIONARY);
}
