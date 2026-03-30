#include <gtest/gtest.h>
#include <cmath>
#include "IMUPreintegrator.h"
#include "test_utils.h"

class IMUPreintegratorTest : public ::testing::Test {
protected:
    IMUPreintegrator imu;
    static constexpr int64_t BASE_NS = 1'000'000'000LL; // 1 second
    static constexpr int RATE_HZ = 200;
    static constexpr int64_t DT_NS = 1'000'000'000LL / RATE_HZ; // 5ms
};

// ── Zero Input Tests ──────────────────────────────────────────────────────────

TEST_F(IMUPreintegratorTest, ZeroGyro_ProducesIdentityRotation) {
    // Feed 1 second of zero gyro + gravity-only accel
    for (int i = 0; i < RATE_HZ; i++) {
        int64_t ts = BASE_NS + i * DT_NS;
        imu.addGyroReading(ts, 0.0f, 0.0f, 0.0f);
        imu.addAccelReading(ts, 0.0f, 0.0f, 9.81f);
    }

    auto delta = imu.integrate(BASE_NS, BASE_NS + 1'000'000'000LL);

    // deltaR should be identity
    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
    double angle_err = test_utils::rotationAngleError(delta.deltaR, I);
    EXPECT_LT(angle_err, 0.01) << "Zero gyro should produce identity rotation";
    EXPECT_GT(delta.sample_count, 0);
}

TEST_F(IMUPreintegratorTest, ZeroGyro_ZeroDisplacement) {
    imu.setGravity(0.0f, 0.0f, 9.81f);

    for (int i = 0; i < RATE_HZ; i++) {
        int64_t ts = BASE_NS + i * DT_NS;
        imu.addGyroReading(ts, 0.0f, 0.0f, 0.0f);
        imu.addAccelReading(ts, 0.0f, 0.0f, 9.81f); // only gravity
    }

    auto delta = imu.integrate(BASE_NS, BASE_NS + 1'000'000'000LL);

    // After gravity subtraction, deltaP should be near zero
    double pos_norm = cv::norm(delta.deltaP);
    EXPECT_LT(pos_norm, 0.05) << "Static accel (gravity only) should give ~zero deltaP";
}

// ── Constant Rotation Tests ──────────────────────────────────────────────────

TEST_F(IMUPreintegratorTest, ConstantGyroZ_CorrectRotation) {
    // 1 rad/s around Z for 1 second → expect Rz(1.0)
    float omega = 1.0f; // rad/s
    for (int i = 0; i < RATE_HZ; i++) {
        int64_t ts = BASE_NS + i * DT_NS;
        imu.addGyroReading(ts, 0.0f, 0.0f, omega);
        imu.addAccelReading(ts, 0.0f, 0.0f, 9.81f);
    }

    auto delta = imu.integrate(BASE_NS, BASE_NS + 1'000'000'000LL);

    cv::Mat expected_R = test_utils::rotZ(1.0);
    double angle_err = test_utils::rotationAngleError(delta.deltaR, expected_R);
    EXPECT_LT(angle_err, 0.05) << "1 rad/s Z rotation for 1s should give ~1 rad rotation";
}

TEST_F(IMUPreintegratorTest, HalfRotation_CorrectAngle) {
    // pi rad/s around Z for 1 second → 180 degrees
    float omega = static_cast<float>(M_PI);
    for (int i = 0; i < RATE_HZ; i++) {
        int64_t ts = BASE_NS + i * DT_NS;
        imu.addGyroReading(ts, 0.0f, 0.0f, omega);
        imu.addAccelReading(ts, 0.0f, 0.0f, 9.81f);
    }

    auto delta = imu.integrate(BASE_NS, BASE_NS + 1'000'000'000LL);

    cv::Mat expected_R = test_utils::rotZ(M_PI);
    double angle_err = test_utils::rotationAngleError(delta.deltaR, expected_R);
    EXPECT_LT(angle_err, 0.1) << "pi rad/s for 1s should give ~180deg rotation";
}

// ── Gravity Initialization ───────────────────────────────────────────────────

TEST_F(IMUPreintegratorTest, GravityInit_PhoneFlat) {
    std::vector<cv::Point3f> samples;
    for (int i = 0; i < 50; i++) {
        samples.emplace_back(0.0f, 0.0f, 9.81f);
    }
    imu.initializeFromGravity(samples);

    EXPECT_TRUE(imu.isInitialized());
    auto g = imu.getGravityVector();
    float g_norm = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
    EXPECT_NEAR(g_norm, 9.81f, 0.5f) << "Gravity magnitude should be ~9.81";
}

TEST_F(IMUPreintegratorTest, GravityInit_PhoneUpright) {
    // Phone held upright: gravity along Y
    std::vector<cv::Point3f> samples;
    for (int i = 0; i < 50; i++) {
        samples.emplace_back(0.0f, 9.81f, 0.0f);
    }
    imu.initializeFromGravity(samples);

    EXPECT_TRUE(imu.isInitialized());
}

// ── Buffer Management ────────────────────────────────────────────────────────

TEST_F(IMUPreintegratorTest, BufferOverflow_DoesNotCrash) {
    // Add 3000 samples (exceeds MAX_BUF = 2000)
    for (int i = 0; i < 3000; i++) {
        int64_t ts = BASE_NS + i * DT_NS;
        imu.addGyroReading(ts, 0.1f, 0.0f, 0.0f);
        imu.addAccelReading(ts, 0.0f, 0.0f, 9.81f);
    }

    // Should not crash, and integrate should still work
    auto delta = imu.integrate(BASE_NS, BASE_NS + 3000 * DT_NS);
    EXPECT_GT(delta.sample_count, 0);
}

// ── Reset ────────────────────────────────────────────────────────────────────

TEST_F(IMUPreintegratorTest, Reset_ClearsState) {
    for (int i = 0; i < 100; i++) {
        int64_t ts = BASE_NS + i * DT_NS;
        imu.addGyroReading(ts, 1.0f, 0.0f, 0.0f);
        imu.addAccelReading(ts, 0.0f, 0.0f, 9.81f);
    }

    imu.reset();

    // After reset, integrate over the old range should give zero samples
    auto delta = imu.integrate(BASE_NS, BASE_NS + 1'000'000'000LL);
    EXPECT_EQ(delta.sample_count, 0);
}

// ── Edge Cases ───────────────────────────────────────────────────────────────

TEST_F(IMUPreintegratorTest, EmptyRange_ReturnsIdentity) {
    auto delta = imu.integrate(BASE_NS, BASE_NS);
    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
    double angle_err = test_utils::rotationAngleError(delta.deltaR, I);
    EXPECT_LT(angle_err, 0.001);
    EXPECT_EQ(delta.sample_count, 0);
}

TEST_F(IMUPreintegratorTest, LastSensorValues_Updated) {
    imu.addGyroReading(BASE_NS, 1.0f, 2.0f, 3.0f);
    imu.addAccelReading(BASE_NS, 4.0f, 5.0f, 6.0f);

    EXPECT_FLOAT_EQ(imu.lastGyroX(), 1.0f);
    EXPECT_FLOAT_EQ(imu.lastGyroY(), 2.0f);
    EXPECT_FLOAT_EQ(imu.lastGyroZ(), 3.0f);
    EXPECT_FLOAT_EQ(imu.lastAccelX(), 4.0f);
    EXPECT_FLOAT_EQ(imu.lastAccelY(), 5.0f);
    EXPECT_FLOAT_EQ(imu.lastAccelZ(), 6.0f);
}
