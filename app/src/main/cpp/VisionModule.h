/**
 * VisionModule.h
 *
 * Student 1: Vision & Sensor Fusion Module
 *
 * Purpose: Process camera frames and sensor data to extract motion information
 *
 * Responsibilities:
 * - Optical Flow tracking
 * - Feature Detection (multiple algorithms)
 * - Movement vector calculation
 * - Direction and velocity estimation
 * - Noise reduction and stabilization
 *
 * API for Student 2 (Navigation Core):
 * - Provides VisionOutput with rotation matrix, translation vector, and metadata
 */

#ifndef NAVSIGHT_VISION_MODULE_H
#define NAVSIGHT_VISION_MODULE_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <mutex>
#include <atomic>
#include "IMUPreintegrator.h"

namespace navsight {

/**
 * Feature detector types available
 */
enum class FeatureDetectorType {
    GOOD_FEATURES,  // Shi-Tomasi corner detector (default, fast)
    FAST,           // FAST corner detector (very fast)
    ORB,            // ORB features (rotation invariant)
    // SIFT not included - requires OpenCV contrib and patent issues
};

/**
 * Vision output data structure
 * This is the API that Student 2 (Navigation Core) will consume
 */
struct VisionOutput {
    // Rotation matrix (3x3) - camera rotation since last frame
    cv::Mat rotation;

    // Translation vector (3x1) - camera translation since last frame
    cv::Mat translation;

    // Confidence metrics
    int tracked_features;      // Number of features successfully tracked
    int total_features;        // Total number of features attempted
    float tracking_quality;    // 0.0 to 1.0, quality of tracking

    // Timestamp
    long timestamp_ns;         // Nanoseconds

    // For UI visualization (Student 3)
    std::vector<cv::Point2f> tracked_points;

    // Status
    bool is_valid;            // Whether this output is reliable

    VisionOutput() :
        rotation(cv::Mat::eye(3, 3, CV_64F)),
        translation(cv::Mat::zeros(3, 1, CV_64F)),
        tracked_features(0),
        total_features(0),
        tracking_quality(0.0f),
        timestamp_ns(0),
        is_valid(false) {}
};

/**
 * Gyroscope reading for sensor fusion
 */
struct GyroData {
    long timestamp_ns;
    float x, y, z;  // Angular velocity (rad/s)
};

/**
 * Accelerometer reading for scale estimation and initialization
 */
struct AccelData {
    long timestamp_ns;
    float x, y, z;  // Acceleration (m/s^2)
};

/**
 * VisionModule - Main class for Student 1's work
 *
 * Usage by Student 2:
 *
 *   VisionModule vision(FeatureDetectorType::GOOD_FEATURES);
 *
 *   // In processing loop:
 *   VisionOutput output = vision.processFrame(frame, width, height, timestamp);
 *   vision.addGyroData(gyro_reading);
 *
 *   if (output.is_valid) {
 *       // Use output.rotation and output.translation for navigation
 *   }
 */
class VisionModule {
public:
    /**
     * Constructor
     * @param detector_type Type of feature detector to use
     * @param max_features Maximum number of features to track (default: 200)
     */
    explicit VisionModule(
        FeatureDetectorType detector_type = FeatureDetectorType::GOOD_FEATURES,
        int max_features = 200
    );

    ~VisionModule();

    /**
     * Process a camera frame
     *
     * @param yuv_data YUV420 format image data
     * @param width Image width
     * @param height Image height
     * @param timestamp_ns Timestamp in nanoseconds
     * @return VisionOutput containing motion estimate
     */
    VisionOutput processFrame(
        const uint8_t* yuv_data,
        int width,
        int height,
        long timestamp_ns
    );

    /**
     * Add gyroscope data for sensor fusion
     * Call this whenever new gyro data arrives
     *
     * @param gyro Gyroscope reading
     */
    void addGyroData(const GyroData& gyro);

    /**
     * Add accelerometer data for scale estimation and gravity alignment
     * Call this whenever new accelerometer data arrives
     *
     * @param accel Accelerometer reading
     */
    void addAccelData(const AccelData& accel);

    /**
     * Reset the vision module (e.g., when app restarts tracking)
     */
    void reset();

    /**
     * Get current tracking statistics
     */
    struct Statistics {
        int total_frames_processed;
        int successful_tracks;
        float average_features_tracked;
        float average_tracking_quality;
    };

    Statistics getStatistics() const;

    /**
     * Get estimated scale factor
     * @return Estimated scale (meters per vision unit)
     */
    double getEstimatedScale() const { return estimated_scale_; }

    /**
     * Check if module is initialized with gravity
     */
    bool isInitialized() const { return is_initialized_.load(); }

    /**
     * Configuration
     */
    void setFeatureDetectorType(FeatureDetectorType type);
    void setMaxFeatures(int max_features);
    void setGyroFusionWeight(float weight); // 0.0 = pure vision, 1.0 = pure gyro

private:
    // Feature detector
    FeatureDetectorType detector_type_;
    cv::Ptr<cv::Feature2D> feature_detector_;
    int max_features_;

    // Tracking state
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_corners_;
    long prev_timestamp_ns_;

    // Gyro data buffer
    std::vector<GyroData> gyro_buffer_;
    std::mutex gyro_mutex_;

    // Accelerometer data buffer
    std::vector<AccelData> accel_buffer_;
    std::mutex accel_mutex_;

    // IMU Preintegrator (handles sensor fusion)
    IMUPreintegrator imu_preintegrator_;

    // Fusion parameters
    float gyro_fusion_weight_;  // Default: 0.98 (98% gyro, 2% vision)

    // Camera intrinsics (simple model)
    cv::Mat camera_matrix_;

    // Scale estimation
    bool scale_initialized_;
    double estimated_scale_;
    cv::Point3f gravity_direction_;

    // Initialization state (atomic for thread-safe double-checked locking)
    std::atomic<bool> is_initialized_;

    // Statistics
    mutable std::mutex stats_mutex_;
    Statistics stats_;

    // Helper functions
    void detectFeatures(const cv::Mat& gray, std::vector<cv::Point2f>& corners);
    cv::Mat integrateGyro(long start_ns, long end_ns);
    cv::Mat fuseRotations(const cv::Mat& R_vision, const cv::Mat& R_gyro);
    float calculateTrackingQuality(
        const std::vector<cv::Point2f>& prev,
        const std::vector<cv::Point2f>& next,
        const std::vector<uchar>& status
    );
    void updateStatistics(const VisionOutput& output);
    void initializeFeatureDetector();

    // Initialization and scale estimation
    bool initializeFromGravity();
    void estimateScaleFromAccel(const cv::Mat& translation, long dt_ns);
    cv::Point3f getAverageAccel(long start_ns, long end_ns);
    bool checkMotionDegeneracy(const std::vector<cv::Point2f>& prev,
                                const std::vector<cv::Point2f>& next);
};

} // namespace navsight

#endif // NAVSIGHT_VISION_MODULE_H
