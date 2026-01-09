#include "IMUPreintegrator.h"
#include <opencv2/calib3d.hpp>
#include <android/log.h>
#include <algorithm>
#include <cstdint>

#define TAG "IMUPreintegrator"

namespace navsight {

// ============================================================================
// Constructor & Destructor
// ============================================================================

IMUPreintegrator::IMUPreintegrator(double gravity_magnitude)
    : gravity_magnitude_(gravity_magnitude)
    , gravity_direction_(0, 0, 1)  // Default: points down in Z
{
    measurements_.reserve(MAX_BUFFER_SIZE);
}

// ============================================================================
// Public Methods
// ============================================================================

void IMUPreintegrator::addMeasurement(long timestamp_ns,
                                     const cv::Vec3d& accel,
                                     const cv::Vec3d& gyro) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Add new measurement
    measurements_.emplace_back(timestamp_ns, accel, gyro);

    // Maintain circular buffer (remove oldest if exceeded)
    if (measurements_.size() > MAX_BUFFER_SIZE) {
        measurements_.erase(measurements_.begin());
    }
}

PreintegratedIMU IMUPreintegrator::integrate(long start_ns, long end_ns) {
    std::lock_guard<std::mutex> lock(mutex_);

    PreintegratedIMU result;
    result.start_timestamp_ns = start_ns;
    result.end_timestamp_ns = end_ns;

    // Check if we have any measurements
    if (measurements_.empty()) {
        __android_log_print(ANDROID_LOG_DEBUG, TAG,
            "No IMU measurements available for integration");
        return result;  // Return identity (no change)
    }

    // State variables for integration
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);     // Current rotation
    cv::Vec3d velocity(0, 0, 0);                 // Current velocity
    cv::Vec3d position(0, 0, 0);                 // Current position
    int64_t last_timestamp = start_ns;           // Use 64-bit to prevent overflow

    // Integrate all measurements in the time range
    for (const auto& m : measurements_) {
        // Skip measurements outside time range
        if (m.timestamp_ns <= start_ns) continue;
        if (m.timestamp_ns > end_ns) break;

        // Calculate time step
        double dt = (m.timestamp_ns - last_timestamp) / 1e9;  // ns to seconds

        // Sanity check on dt
        if (dt <= 0 || dt > 0.5) {  // Reject if > 500ms (likely error)
            __android_log_print(ANDROID_LOG_WARN, TAG,
                "Invalid dt: %.3f seconds, skipping measurement", dt);
            continue;
        }

        // 1. Integrate gyroscope → Update rotation
        integrateGyro(m.gyro, dt, R);

        // 2. Integrate accelerometer → Update velocity and position
        integrateAccel(m.accel, dt, R, velocity, position);

        last_timestamp = m.timestamp_ns;
        result.num_measurements++;
    }

    // Store results
    result.delta_R = R.clone();
    result.delta_v = velocity;
    result.delta_p = position;
    result.dt = (end_ns - start_ns) / 1e9;  // Total time in seconds

    __android_log_print(ANDROID_LOG_DEBUG, TAG,
        "Preintegrated %d measurements over %.3f seconds: "
        "p=(%.3f, %.3f, %.3f), v=(%.3f, %.3f, %.3f)",
        result.num_measurements, result.dt,
        position[0], position[1], position[2],
        velocity[0], velocity[1], velocity[2]);

    return result;
}

void IMUPreintegrator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    measurements_.clear();
    __android_log_print(ANDROID_LOG_INFO, TAG, "IMU preintegrator reset");
}

void IMUPreintegrator::setGravity(const cv::Vec3d& gravity_dir, double gravity_mag) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Normalize direction
    double norm = cv::norm(gravity_dir);
    if (norm > 1e-6) {
        gravity_direction_ = gravity_dir / norm;
    } else {
        __android_log_print(ANDROID_LOG_WARN, TAG,
            "Invalid gravity direction (zero vector), using default");
        gravity_direction_ = cv::Vec3d(0, 0, 1);
    }

    gravity_magnitude_ = gravity_mag;

    __android_log_print(ANDROID_LOG_INFO, TAG,
        "Gravity set: direction=(%.3f, %.3f, %.3f), magnitude=%.3f m/s²",
        gravity_direction_[0], gravity_direction_[1], gravity_direction_[2],
        gravity_magnitude_);
}

cv::Vec3d IMUPreintegrator::getGravity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gravity_direction_ * gravity_magnitude_;
}

size_t IMUPreintegrator::getBufferSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return measurements_.size();
}

// ============================================================================
// Private Helper Methods
// ============================================================================

void IMUPreintegrator::integrateGyro(const cv::Vec3d& gyro, double dt, cv::Mat& R) {
    // Create rotation vector (angle-axis representation)
    // Rotation angle = angular_velocity * time
    cv::Vec3d rotation_vector = gyro * dt;

    // Convert to rotation matrix using Rodrigues formula
    cv::Mat delta_R;
    cv::Rodrigues(rotation_vector, delta_R);

    // Accumulate rotation: R_new = delta_R * R_old
    // (Right-multiply because we're rotating the body frame)
    R = delta_R * R;
}

void IMUPreintegrator::integrateAccel(const cv::Vec3d& accel, double dt,
                                     const cv::Mat& R,
                                     cv::Vec3d& velocity,
                                     cv::Vec3d& position) {
    // Step 1: Rotate acceleration from body frame to world frame
    // a_world = R * a_body
    cv::Mat accel_mat = (cv::Mat_<double>(3, 1) << accel[0], accel[1], accel[2]);
    cv::Mat accel_world_mat = R * accel_mat;
    cv::Vec3d accel_world(accel_world_mat.at<double>(0),
                         accel_world_mat.at<double>(1),
                         accel_world_mat.at<double>(2));

    // Step 2: Remove gravity
    // The accelerometer measures: a_measured = a_true + g
    // So: a_true = a_measured - g
    cv::Vec3d gravity = gravity_direction_ * gravity_magnitude_;
    cv::Vec3d accel_corrected = accel_world - gravity;

    // Step 3: Integrate velocity
    // v(t+dt) = v(t) + a * dt
    velocity = velocity + accel_corrected * dt;

    // Step 4: Integrate position (using midpoint rule for better accuracy)
    // p(t+dt) = p(t) + v(t) * dt + 0.5 * a * dt²
    position = position + velocity * dt + 0.5 * accel_corrected * dt * dt;
}

} // namespace navsight
