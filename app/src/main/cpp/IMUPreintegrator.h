#ifndef NAVSIGHT_IMU_PREINTEGRATOR_H
#define NAVSIGHT_IMU_PREINTEGRATOR_H

#include <opencv2/core.hpp>
#include <vector>
#include <mutex>

namespace navsight {

/**
 * @brief IMU measurement data structure
 *
 * Represents a single IMU reading with timestamp.
 * Accelerometer units: m/s²
 * Gyroscope units: rad/s
 */
struct IMUMeasurement {
    long timestamp_ns;      // Nanosecond timestamp
    cv::Vec3d accel;        // Acceleration (m/s²) in body frame
    cv::Vec3d gyro;         // Angular velocity (rad/s) in body frame

    IMUMeasurement(long ts, const cv::Vec3d& a, const cv::Vec3d& g)
        : timestamp_ns(ts), accel(a), gyro(g) {}
};

/**
 * @brief Preintegrated IMU measurements between two time instants
 *
 * Represents the change in position, velocity, and rotation obtained by
 * integrating IMU measurements between two keyframes. This avoids
 * reintegrating during optimization iterations.
 *
 * Reference: "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
 *            Forster et al., 2016
 */
struct PreintegratedIMU {
    // Preintegrated measurements (all in initial body frame)
    cv::Vec3d delta_p;      // Position change (meters)
    cv::Vec3d delta_v;      // Velocity change (m/s)
    cv::Mat delta_R;        // Rotation change (3x3 rotation matrix)

    // Time interval
    double dt;              // Total integration time (seconds)

    // Number of measurements integrated
    int num_measurements;

    // Timestamps
    long start_timestamp_ns;
    long end_timestamp_ns;

    PreintegratedIMU()
        : delta_p(0, 0, 0)
        , delta_v(0, 0, 0)
        , delta_R(cv::Mat::eye(3, 3, CV_64F))
        , dt(0.0)
        , num_measurements(0)
        , start_timestamp_ns(0)
        , end_timestamp_ns(0) {}
};

/**
 * @brief IMU Preintegrator - Integrates IMU measurements between frames
 *
 * This class handles the preintegration of IMU measurements (accelerometer
 * and gyroscope) between two time instants. The key benefit is that IMU
 * data is integrated once and stored as a single constraint, avoiding
 * expensive reintegration during optimization.
 *
 * THREAD-SAFE: All public methods are protected by mutex.
 *
 * Usage Example:
 * @code
 *   IMUPreintegrator preint;
 *
 *   // Add measurements as they arrive
 *   preint.addMeasurement(timestamp, accel, gyro);
 *
 *   // Get preintegrated result between two times
 *   PreintegratedIMU result = preint.integrate(t_start, t_end);
 *
 *   // Use result.delta_p, result.delta_v, result.delta_R
 * @endcode
 */
class IMUPreintegrator {
public:
    /**
     * @brief Constructor
     * @param gravity_magnitude Magnitude of gravity (default: 9.81 m/s²)
     */
    explicit IMUPreintegrator(double gravity_magnitude = 9.81);

    /**
     * @brief Add a new IMU measurement to the buffer
     * @param timestamp_ns Timestamp in nanoseconds
     * @param accel Acceleration vector (m/s²) in body frame
     * @param gyro Angular velocity vector (rad/s) in body frame
     *
     * Thread-safe. Measurements are stored in a circular buffer.
     */
    void addMeasurement(long timestamp_ns, const cv::Vec3d& accel, const cv::Vec3d& gyro);

    /**
     * @brief Preintegrate IMU measurements between two timestamps
     * @param start_ns Start timestamp (nanoseconds)
     * @param end_ns End timestamp (nanoseconds)
     * @return Preintegrated measurements (position, velocity, rotation changes)
     *
     * Thread-safe. Integrates all measurements in the time range [start_ns, end_ns).
     * If no measurements are available, returns identity (no change).
     *
     * Algorithm:
     * 1. For each gyro sample: Update rotation (delta_R)
     * 2. For each accel sample:
     *    - Rotate acceleration to current orientation
     *    - Remove gravity
     *    - Integrate to velocity (delta_v)
     *    - Integrate to position (delta_p)
     */
    PreintegratedIMU integrate(long start_ns, long end_ns);

    /**
     * @brief Reset the integrator state
     *
     * Clears the measurement buffer. Call this when starting a new trajectory.
     */
    void reset();

    /**
     * @brief Set gravity direction and magnitude
     * @param gravity_dir Normalized gravity direction vector (points down)
     * @param gravity_mag Gravity magnitude (default: 9.81 m/s²)
     *
     * Should be called after gravity initialization. Used to remove gravity
     * from accelerometer readings during integration.
     */
    void setGravity(const cv::Vec3d& gravity_dir, double gravity_mag = 9.81);

    /**
     * @brief Get current gravity vector
     * @return Gravity vector (direction * magnitude)
     */
    cv::Vec3d getGravity() const;

    /**
     * @brief Get number of measurements in buffer
     * @return Number of stored measurements
     */
    size_t getBufferSize() const;

private:
    // Configuration
    double gravity_magnitude_;
    cv::Vec3d gravity_direction_;  // Normalized direction (points down)

    // Measurement buffer (circular buffer with max size)
    std::vector<IMUMeasurement> measurements_;
    static constexpr size_t MAX_BUFFER_SIZE = 200;  // 1 second at 200 Hz

    // Thread safety
    mutable std::mutex mutex_;

    /**
     * @brief Integrate a single gyroscope measurement
     * @param gyro Angular velocity (rad/s)
     * @param dt Time step (seconds)
     * @param R Current rotation matrix (updated in-place)
     *
     * Uses Rodrigues formula to convert angular velocity to rotation matrix.
     */
    void integrateGyro(const cv::Vec3d& gyro, double dt, cv::Mat& R);

    /**
     * @brief Integrate a single accelerometer measurement
     * @param accel Acceleration (m/s²) in body frame
     * @param dt Time step (seconds)
     * @param R Current rotation to world frame
     * @param velocity Current velocity (updated in-place)
     * @param position Current position (updated in-place)
     *
     * Steps:
     * 1. Rotate acceleration to world frame: a_world = R * a_body
     * 2. Remove gravity: a_world = a_world - g
     * 3. Integrate velocity: v = v + a_world * dt
     * 4. Integrate position: p = p + v * dt + 0.5 * a_world * dt²
     */
    void integrateAccel(const cv::Vec3d& accel, double dt, const cv::Mat& R,
                       cv::Vec3d& velocity, cv::Vec3d& position);
};

} // namespace navsight

#endif // NAVSIGHT_IMU_PREINTEGRATOR_H
