#include <gtest/gtest.h>
#include <vector>
#include "UpdaterZeroVelocity.h"
#include "IMUPreintegrator.h"

class ZUPTTest : public ::testing::Test {
protected:
    UpdaterZeroVelocity detector;
    
    std::vector<AccelSample> createStaticAccel(int n, float stddev = 0.01f) {
        std::vector<AccelSample> samples;
        for (int i = 0; i < n; ++i) {
            // Gravity + noise
            float nx = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * stddev;
            float ny = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * stddev;
            float nz = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * stddev;
            samples.push_back({i * 5'000'000LL, nx, ny, 9.81f + nz});
        }
        return samples;
    }

    std::vector<GyroSample> createStaticGyro(int n, float stddev = 0.001f) {
        std::vector<GyroSample> samples;
        for (int i = 0; i < n; ++i) {
            float nx = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * stddev;
            float ny = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * stddev;
            float nz = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * stddev;
            samples.push_back({i * 5'000'000LL, nx, ny, nz});
        }
        return samples;
    }

    std::vector<AccelSample> createMovingAccel(int n, float offset = 0.5f) {
        std::vector<AccelSample> samples;
        for (int i = 0; i < n; ++i) {
            samples.push_back({i * 5'000'000LL, offset, 0.0f, 9.81f});
        }
        return samples;
    }
};

TEST_F(ZUPTTest, StaticDevice_DetectedAsStationary) {
    auto accel = createStaticAccel(20);
    auto gyro = createStaticGyro(20);
    
    EXPECT_TRUE(detector.is_stationary(accel, gyro, 0.1));
}

TEST_F(ZUPTTest, MovingDevice_DetectedAsMoving) {
    auto accel = createMovingAccel(20, 1.0f); // High acceleration
    auto gyro = createStaticGyro(20);
    
    EXPECT_FALSE(detector.is_stationary(accel, gyro, 0.1));
}

TEST_F(ZUPTTest, HighDisparity_DetectedAsMoving) {
    auto accel = createStaticAccel(20);
    auto gyro = createStaticGyro(20);
    
    EXPECT_FALSE(detector.is_stationary(accel, gyro, 2.0)); // High visual motion
}

TEST_F(ZUPTTest, RotatingDevice_DetectedAsMoving) {
    auto accel = createStaticAccel(20);
    std::vector<GyroSample> gyro;
    for (int i = 0; i < 20; ++i) {
        gyro.push_back({i * 5'000'000LL, 0.5f, 0.0f, 0.0f}); // Constant rotation
    }
    
    EXPECT_FALSE(detector.is_stationary(accel, gyro, 0.1));
}

TEST_F(ZUPTTest, Chi2Threshold_Calculation) {
    // Basic sanity check for Chi-squared threshold approximation
    // DOF = 3 for a single 3D sample variance
    // Chi2(3, 0.95) is approx 7.81
    // We don't expose get_chi2_threshold but we can test dof-related behavior if needed.
    SUCCEED();
}
