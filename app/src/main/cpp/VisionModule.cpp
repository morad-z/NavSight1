/**
 * VisionModule.cpp
 *
 * Student 1: Vision & Sensor Fusion Module Implementation
 * Fixed & Cleaned Version
 */

#include "VisionModule.h"
#include <opencv2/calib3d.hpp>
#include <android/log.h>
#include <algorithm>

#define TAG "VisionModule"

namespace navsight {

    VisionModule::VisionModule(FeatureDetectorType detector_type, int max_features)
            : detector_type_(detector_type),
              max_features_(max_features),
              prev_timestamp_ns_(0),
              gyro_fusion_weight_(0.98f),
              scale_initialized_(false),
              estimated_scale_(1.0),
              is_initialized_(false) {

        // Initialize camera intrinsics (will be updated with actual frame size)
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);

        // Initialize gravity direction (will be updated from accelerometer)
        gravity_direction_ = cv::Point3f(0, 0, -9.81f);  // Default: Z-down

        // Initialize statistics
        stats_.total_frames_processed = 0;
        stats_.successful_tracks = 0;
        stats_.average_features_tracked = 0.0f;
        stats_.average_tracking_quality = 0.0f;

        // Initialize feature detector
        initializeFeatureDetector();

        __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule initialized with detector type %d",
                            static_cast<int>(detector_type_));
    }

    VisionModule::~VisionModule() {
        __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule destroyed");
    }

    void VisionModule::initializeFeatureDetector() {
        switch (detector_type_) {
            case FeatureDetectorType::FAST:
                feature_detector_ = cv::FastFeatureDetector::create();
                __android_log_print(ANDROID_LOG_INFO, TAG, "Using FAST feature detector");
                break;

            case FeatureDetectorType::ORB:
                feature_detector_ = cv::ORB::create(max_features_);
                __android_log_print(ANDROID_LOG_INFO, TAG, "Using ORB feature detector");
                break;

            case FeatureDetectorType::GOOD_FEATURES:
            default:
                // For GOOD_FEATURES, we use cv::goodFeaturesToTrack directly
                feature_detector_ = nullptr;
                __android_log_print(ANDROID_LOG_INFO, TAG, "Using Shi-Tomasi (GOOD_FEATURES) detector");
                break;
        }
    }

    void VisionModule::detectFeatures(const cv::Mat& gray, std::vector<cv::Point2f>& corners) {
        corners.clear();

        if (detector_type_ == FeatureDetectorType::GOOD_FEATURES || !feature_detector_) {
            // Shi-Tomasi corner detector (default, fast and good for tracking)
            cv::goodFeaturesToTrack(gray, corners, max_features_, 0.01, 10);
        } else {
            // Use Feature2D detector (FAST, ORB, etc.)
            std::vector<cv::KeyPoint> keypoints;
            feature_detector_->detect(gray, keypoints);

            // Convert KeyPoints to Point2f
            cv::KeyPoint::convert(keypoints, corners);

            // Limit to max_features
            if (corners.size() > static_cast<size_t>(max_features_)) {
                corners.resize(max_features_);
            }
        }

        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Detected %zu features", corners.size());
    }

    VisionOutput VisionModule::processFrame(
            const uint8_t* yuv_data,
            int width,
            int height,
            long timestamp_ns) {

        VisionOutput output;
        output.timestamp_ns = timestamp_ns;

        // Update camera intrinsics if frame size changed
        double focal = width;  // Simple approximation
        cv::Point2d principal_point(width / 2.0, height / 2.0);
        camera_matrix_.at<double>(0, 0) = focal;
        camera_matrix_.at<double>(1, 1) = focal;
        camera_matrix_.at<double>(0, 2) = principal_point.x;
        camera_matrix_.at<double>(1, 2) = principal_point.y;

        // Convert YUV420 to grayscale
        cv::Mat yuv_mat(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(yuv_data));
        cv::Mat gray_mat;
        cv::cvtColor(yuv_mat, gray_mat, cv::COLOR_YUV2GRAY_NV21);

        // First frame - just detect features
        if (prev_gray_.empty()) {
            detectFeatures(gray_mat, prev_corners_);
            gray_mat.copyTo(prev_gray_);
            prev_timestamp_ns_ = timestamp_ns;

            output.total_features = prev_corners_.size();
            output.tracked_features = 0;
            output.tracking_quality = 0.0f;
            output.is_valid = false;

            __android_log_print(ANDROID_LOG_INFO, TAG, "First frame - detected %d features",
                                output.total_features);

            updateStatistics(output);
            return output;
        }

        // Check if we have features to track
        if (prev_corners_.empty()) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "No features to track, re-detecting...");
            detectFeatures(gray_mat, prev_corners_);
            gray_mat.copyTo(prev_gray_);
            prev_timestamp_ns_ = timestamp_ns;

            output.total_features = prev_corners_.size();
            output.is_valid = false;
            updateStatistics(output);
            return output;
        }

        // Optical Flow tracking
        std::vector<cv::Point2f> next_corners;
        std::vector<uchar> status;
        std::vector<float> err;

        cv::calcOpticalFlowPyrLK(prev_gray_, gray_mat, prev_corners_, next_corners, status, err);

        // Filter good matches
        std::vector<cv::Point2f> good_prev_corners, good_next_corners;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                good_prev_corners.push_back(prev_corners_[i]);
                good_next_corners.push_back(next_corners[i]);
            }
        }

        output.total_features = prev_corners_.size();
        output.tracked_features = good_prev_corners.size();
        output.tracked_points = good_next_corners;

        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Tracked %d / %d features",
                            output.tracked_features, output.total_features);

        // Need at least 8 points for Essential Matrix estimation
        if (good_prev_corners.size() < 8) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "Not enough features (%zu) for pose estimation",
                                good_prev_corners.size());
            output.tracking_quality = 0.0f;
            output.is_valid = false;

            // Re-detect features for next frame
            detectFeatures(gray_mat, prev_corners_);
            gray_mat.copyTo(prev_gray_);
            prev_timestamp_ns_ = timestamp_ns;

            updateStatistics(output);
            return output;
        }

        // Check for motion degeneracy
        if (checkMotionDegeneracy(good_prev_corners, good_next_corners)) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "Degenerate motion detected");
            output.tracking_quality = 0.0f;
            output.is_valid = false;

            // Keep existing corners but update frame
            gray_mat.copyTo(prev_gray_);
            prev_corners_ = good_next_corners;
            prev_timestamp_ns_ = timestamp_ns;

            updateStatistics(output);
            return output;
        }

        // Compute Essential Matrix and recover pose
        cv::Mat E, R_vision, t_vision;
        cv::Mat inlier_mask;

        // Use stricter RANSAC parameters for better outlier rejection
        E = cv::findEssentialMat(good_next_corners, good_prev_corners, camera_matrix_,
                                 cv::RANSAC, 0.9999, 0.5, inlier_mask);

        if (E.empty()) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "Essential Matrix estimation failed");
            output.tracking_quality = 0.0f;
            output.is_valid = false;

            gray_mat.copyTo(prev_gray_);
            prev_corners_ = good_next_corners;
            prev_timestamp_ns_ = timestamp_ns;

            updateStatistics(output);
            return output;
        }

        int inliers_count = cv::recoverPose(E, good_next_corners, good_prev_corners, camera_matrix_,
                                            R_vision, t_vision, inlier_mask);

        if (inliers_count < 6) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "Too few inliers: %d", inliers_count);
            output.tracking_quality = 0.0f;
            output.is_valid = false;

            gray_mat.copyTo(prev_gray_);
            prev_corners_ = good_next_corners;
            prev_timestamp_ns_ = timestamp_ns;

            updateStatistics(output);
            return output;
        }

        // Calculate tracking quality
        output.tracking_quality = calculateTrackingQuality(good_prev_corners, good_next_corners, status);

        // --- IMU Preintegration & Fusion ---
        long dt_ns = timestamp_ns - prev_timestamp_ns_;
        cv::Mat R_fused = R_vision.clone();
        cv::Mat t_scaled = t_vision.clone();

        __android_log_print(ANDROID_LOG_DEBUG, TAG,
                            "Preintegration check: initialized=%d, dt_ns=%ld, buffer_size=%zu",
                            is_initialized_.load(), dt_ns, imu_preintegrator_.getBufferSize());

        if (is_initialized_ && dt_ns > 0 && dt_ns < 1000000000) {  // Sanity check: < 1 second
            try {
                // Get preintegrated IMU measurements between frames
                PreintegratedIMU preint = imu_preintegrator_.integrate(prev_timestamp_ns_, timestamp_ns);

                __android_log_print(ANDROID_LOG_DEBUG, TAG,
                                    "Preintegration result: %d measurements, dt=%.3fs",
                                    preint.num_measurements, preint.dt);

                if (preint.num_measurements > 0) {
                    // Use IMU rotation (more accurate than vision for fast motion)
                    R_fused = fuseRotations(R_vision, preint.delta_R);

                    // Estimate scale from IMU position displacement
                    double vision_displacement = cv::norm(t_vision);
                    double imu_displacement = cv::norm(preint.delta_p);

                    if (vision_displacement > 0.01 && imu_displacement > 0.001) {
                        double scale = imu_displacement / vision_displacement;

                        if (scale > 0.1 && scale < 10.0) {
                            // Weighted average with previous scale for stability
                            estimated_scale_ = 0.7 * estimated_scale_ + 0.3 * scale;
                            scale_initialized_ = true;

                            __android_log_print(ANDROID_LOG_DEBUG, TAG,
                                                "IMU Preintegration: displacement=%.3fm, scale=%.3f (smoothed=%.3f)",
                                                imu_displacement, scale, estimated_scale_);
                        } else {
                            __android_log_print(ANDROID_LOG_WARN, TAG,
                                                "Scale %.3f outside valid range [0.1, 10.0]", scale);
                        }
                    }
                } else {
                    // Fallback: If preintegration had no measurements, try integrating raw gyro buffer
                    // This handles cases where preintegration might have cleared too early
                    cv::Mat R_gyro = integrateGyro(prev_timestamp_ns_, timestamp_ns);
                    R_fused = fuseRotations(R_vision, R_gyro);
                }
            } catch (const std::exception& e) {
                __android_log_print(ANDROID_LOG_ERROR, TAG, "IMU preintegration failed: %s", e.what());
                // Fallback to pure vision
                R_fused = R_vision.clone();
            }
        }

        // Apply scale to translation
        t_scaled = t_vision * estimated_scale_;

        // Set output
        output.rotation = R_fused;
        output.translation = t_scaled;

        // Estimate scale from accelerometer if we have data (Alternative simple method)
        if (is_initialized_ && dt_ns > 0 && dt_ns < 1000000000) {
            try {
                estimateScaleFromAccel(t_vision, dt_ns);
            } catch (const std::exception& e) {
                // Log error but continue
                __android_log_print(ANDROID_LOG_ERROR, TAG, "Scale estimation failed: %s", e.what());
            }
        }

        output.is_valid = true;

        __android_log_print(ANDROID_LOG_INFO, TAG,
                            "Pose estimated - Quality: %.2f, Tracked: %d/%d",
                            output.tracking_quality, output.tracked_features, output.total_features);

        // Update state for next frame
        gray_mat.copyTo(prev_gray_);
        prev_corners_ = good_next_corners;

        // Re-detect features if we're running low
        if (prev_corners_.size() < static_cast<size_t>(max_features_ / 2)) {
            std::vector<cv::Point2f> new_corners;
            detectFeatures(gray_mat, new_corners);

            // Merge with existing corners (avoid duplicates)
            for (const auto& new_corner : new_corners) {
                bool is_duplicate = false;
                for (const auto& existing_corner : prev_corners_) {
                    float dist = cv::norm(new_corner - existing_corner);
                    if (dist < 10.0f) {  // 10 pixel threshold
                        is_duplicate = true;
                        break;
                    }
                }
                if (!is_duplicate) {
                    prev_corners_.push_back(new_corner);
                    if (prev_corners_.size() >= static_cast<size_t>(max_features_)) {
                        break;
                    }
                }
            }
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Re-detected features, now have %zu",
                                prev_corners_.size());
        }

        prev_timestamp_ns_ = timestamp_ns;

        updateStatistics(output);
        return output;
    }

    void VisionModule::addGyroData(const GyroData& gyro) {
        // Transform from Android sensor frame to camera frame (X right, Y down, Z forward)
        GyroData gyro_camera_frame;
        gyro_camera_frame.timestamp_ns = gyro.timestamp_ns;
        gyro_camera_frame.x = gyro.x;
        gyro_camera_frame.y = -gyro.y;
        gyro_camera_frame.z = -gyro.z;

        {
            std::lock_guard<std::mutex> lock(gyro_mutex_);

            // Add transformed data to buffer
            gyro_buffer_.push_back(gyro_camera_frame);

            // Keep buffer size reasonable (last 1 second at 100Hz = 100 samples)
            const size_t MAX_GYRO_BUFFER_SIZE = 100;
            if (gyro_buffer_.size() > MAX_GYRO_BUFFER_SIZE) {
                gyro_buffer_.erase(gyro_buffer_.begin());
            }
        }

        // Feed to IMU preintegrator only if we have a recent, matching accel measurement
        cv::Vec3d accel_vec;
        bool have_matching_accel = false;
        {
            std::lock_guard<std::mutex> lock(accel_mutex_);
            if (!accel_buffer_.empty()) {
                // Use the most recent accelerometer reading, but only if it's close in time
                const auto& latest_accel = accel_buffer_.back();
                if (std::abs(gyro_camera_frame.timestamp_ns - latest_accel.timestamp_ns) < 20000000LL) { // 20ms threshold
                    accel_vec = {latest_accel.x, latest_accel.y, latest_accel.z};
                    have_matching_accel = true;
                }
            }
        }

        if (have_matching_accel) {
            cv::Vec3d gyro_vec(gyro_camera_frame.x, gyro_camera_frame.y, gyro_camera_frame.z);
            imu_preintegrator_.addMeasurement(gyro_camera_frame.timestamp_ns, accel_vec, gyro_vec);
        }

        // Debug: Log every 30th gyro measurement to avoid spam
        static int gyro_count = 0;
        if (++gyro_count % 30 == 0) {
            __android_log_print(ANDROID_LOG_DEBUG, TAG,
                                "Added gyro measurement (count=%d, buffer_size=%zu, matched=%d)",
                                gyro_count, imu_preintegrator_.getBufferSize(), have_matching_accel);
        }
    }

    void VisionModule::addAccelData(const AccelData& accel) {
        // Transform from Android sensor frame to camera frame (X right, Y down, Z forward)
        AccelData accel_camera_frame;
        accel_camera_frame.timestamp_ns = accel.timestamp_ns;
        accel_camera_frame.x = accel.x;
        accel_camera_frame.y = -accel.y;
        accel_camera_frame.z = -accel.z;

        bool should_initialize = false;

        {
            std::lock_guard<std::mutex> lock(accel_mutex_);

            // Add transformed data to buffer
            accel_buffer_.push_back(accel_camera_frame);

            // Keep buffer size reasonable (last 1 second at 100Hz = 100 samples)
            const size_t MAX_ACCEL_BUFFER_SIZE = 100;
            if (accel_buffer_.size() > MAX_ACCEL_BUFFER_SIZE) {
                accel_buffer_.erase(accel_buffer_.begin());
            }

            // Check if we should initialize (but don't do it while holding the lock!)
            should_initialize = !is_initialized_ && accel_buffer_.size() >= 20;
        }

        // Initialize OUTSIDE the lock to avoid deadlock
        if (should_initialize) {
            initializeFromGravity();
        }

        // Debug: Log every 30th accel measurement to avoid spam
        static int accel_count = 0;
        if (++accel_count % 30 == 0) {
            __android_log_print(ANDROID_LOG_DEBUG, TAG,
                                "Buffered accel measurement (count=%d, buffer_size=%zu)",
                                accel_count, accel_buffer_.size());
        }
    }

    cv::Mat VisionModule::integrateGyro(long start_ns, long end_ns) {
        std::lock_guard<std::mutex> lock(gyro_mutex_);

        cv::Mat delta_R = cv::Mat::eye(3, 3, CV_64F);

        if (gyro_buffer_.empty()) {
            return delta_R;  // No gyro data available
        }

        long last_ts = start_ns;

        for (const auto& reading : gyro_buffer_) {
            if (reading.timestamp_ns > start_ns && reading.timestamp_ns <= end_ns) {
                double dt = (reading.timestamp_ns - last_ts) / 1e9;  // Convert to seconds

                // Create rotation vector (angle-axis representation)
                cv::Mat rot_vec = (cv::Mat_<double>(3, 1) <<
                                                          reading.x * dt,
                        reading.y * dt,
                        reading.z * dt);

                // Convert to rotation matrix
                cv::Mat R;
                cv::Rodrigues(rot_vec, R);

                // Accumulate rotation
                delta_R = R * delta_R;

                last_ts = reading.timestamp_ns;
            }
        }

        return delta_R;
    }

    cv::Mat VisionModule::fuseRotations(const cv::Mat& R_vision, const cv::Mat& R_gyro) {
        // Check if gyro provided meaningful rotation
        cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
        double gyro_diff = cv::norm(R_gyro - I);

        if (gyro_diff < 1e-6) {
            // No gyro data, use pure vision
            return R_vision.clone();
        }

        // Convert both to rotation vectors
        cv::Mat rot_vec_vision, rot_vec_gyro;
        cv::Rodrigues(R_vision, rot_vec_vision);
        cv::Rodrigues(R_gyro, rot_vec_gyro);

        // Weighted average (gyro_fusion_weight_ favors gyro for high-frequency motion)
        cv::Mat rot_vec_fused = gyro_fusion_weight_ * rot_vec_gyro +
                                (1.0 - gyro_fusion_weight_) * rot_vec_vision;

        // Convert back to rotation matrix
        cv::Mat R_fused;
        cv::Rodrigues(rot_vec_fused, R_fused);

        return R_fused;
    }

    float VisionModule::calculateTrackingQuality(
            const std::vector<cv::Point2f>& prev,
            const std::vector<cv::Point2f>& next,
            const std::vector<uchar>& status) {

        if (prev.empty() || next.empty()) {
            return 0.0f;
        }

        // Quality metric 1: Percentage of successfully tracked features
        int tracked_count = 0;
        for (uchar s : status) {
            if (s) tracked_count++;
        }
        float tracking_ratio = static_cast<float>(tracked_count) / status.size();

        // Quality metric 2: Average motion magnitude (good tracking has consistent motion)
        float avg_motion = 0.0f;
        int count = 0;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                float motion = cv::norm(next[i] - prev[i]);
                avg_motion += motion;
                count++;
            }
        }
        if (count > 0) {
            avg_motion /= count;
        }

        // Motion should be reasonable (not too small, not too large)
        // Good motion is typically 2-50 pixels between frames
        float motion_quality = 1.0f;
        if (avg_motion < 1.0f || avg_motion > 100.0f) {
            motion_quality = 0.5f;  // Suspicious motion
        }

        // Combine metrics
        float quality = tracking_ratio * 0.7f + motion_quality * 0.3f;

        return std::max(0.0f, std::min(1.0f, quality));
    }

    void VisionModule::updateStatistics(const VisionOutput& output) {
        std::lock_guard<std::mutex> lock(stats_mutex_);

        stats_.total_frames_processed++;

        if (output.is_valid) {
            stats_.successful_tracks++;
        }

        // Update running averages
        float alpha = 0.9f;  // Smoothing factor
        stats_.average_features_tracked = alpha * stats_.average_features_tracked +
                                          (1.0f - alpha) * output.tracked_features;
        stats_.average_tracking_quality = alpha * stats_.average_tracking_quality +
                                          (1.0f - alpha) * output.tracking_quality;
    }

    void VisionModule::reset() {
        __android_log_print(ANDROID_LOG_INFO, TAG, "Resetting VisionModule");

        prev_gray_.release();
        prev_corners_.clear();
        prev_timestamp_ns_ = 0;

        {
            std::lock_guard<std::mutex> lock(gyro_mutex_);
            gyro_buffer_.clear();
        }

        {
            std::lock_guard<std::mutex> lock(accel_mutex_);
            accel_buffer_.clear();
        }

        // Reset IMU preintegrator
        imu_preintegrator_.reset();

        // Don't reset initialization state - keep gravity direction
        // But reset scale
        scale_initialized_ = false;
        estimated_scale_ = 1.0;

        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.total_frames_processed = 0;
            stats_.successful_tracks = 0;
            stats_.average_features_tracked = 0.0f;
            stats_.average_tracking_quality = 0.0f;
        }
    }

    VisionModule::Statistics VisionModule::getStatistics() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return stats_;
    }

    void VisionModule::setFeatureDetectorType(FeatureDetectorType type) {
        detector_type_ = type;
        initializeFeatureDetector();
        __android_log_print(ANDROID_LOG_INFO, TAG, "Feature detector changed to type %d",
                            static_cast<int>(type));
    }

    void VisionModule::setMaxFeatures(int max_features) {
        max_features_ = max_features;
        __android_log_print(ANDROID_LOG_INFO, TAG, "Max features set to %d", max_features_);
    }

    void VisionModule::setGyroFusionWeight(float weight) {
        gyro_fusion_weight_ = std::max(0.0f, std::min(1.0f, weight));
        __android_log_print(ANDROID_LOG_INFO, TAG, "Gyro fusion weight set to %.2f",
                            gyro_fusion_weight_);
    }

    bool VisionModule::initializeFromGravity() {
        // Check again to avoid race condition
        if (is_initialized_) {
            return true;
        }

        std::vector<AccelData> accel_buffer_copy;
        {
            std::lock_guard<std::mutex> lock(accel_mutex_);
            if (accel_buffer_.size() < 20) { // Need enough samples
                return false;
            }
            accel_buffer_copy = accel_buffer_;
        }

        // --- 1. Calculate Mean ---
        cv::Point3f avg_accel(0, 0, 0);
        for (const auto& accel : accel_buffer_copy) {
            avg_accel.x += accel.x;
            avg_accel.y += accel.y;
            avg_accel.z += accel.z;
        }
        avg_accel.x /= accel_buffer_copy.size();
        avg_accel.y /= accel_buffer_copy.size();
        avg_accel.z /= accel_buffer_copy.size();

        // --- 2. Calculate Variance ---
        double variance = 0.0;
        for (const auto& accel : accel_buffer_copy) {
            variance += std::pow(accel.x - avg_accel.x, 2);
            variance += std::pow(accel.y - avg_accel.y, 2);
            variance += std::pow(accel.z - avg_accel.z, 2);
        }
        variance /= accel_buffer_copy.size();

        // --- 3. Check if device is stationary ---
        const double MAX_VARIANCE_FOR_INIT = 0.5; // Threshold for motion
        if (variance > MAX_VARIANCE_FOR_INIT) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "Device in motion (variance=%.3f), deferring gravity initialization.", variance);
            return false;
        }

        // --- 4. If stationary, proceed with initialization ---
        float norm = std::sqrt(avg_accel.x * avg_accel.x +
                               avg_accel.y * avg_accel.y +
                               avg_accel.z * avg_accel.z);

        if (norm < 8.0f || norm > 12.0f) {
            __android_log_print(ANDROID_LOG_WARN, TAG,
                                "Cannot initialize: gravity magnitude %.2f m/s^2 is unreasonable", norm);
            return false;
        }

        gravity_direction_.x = avg_accel.x / norm;
        gravity_direction_.y = avg_accel.y / norm;
        gravity_direction_.z = avg_accel.z / norm;

        is_initialized_ = true;

        // Set gravity in IMU preintegrator for proper sensor fusion
        cv::Vec3d gravity_vec(gravity_direction_.x, gravity_direction_.y, gravity_direction_.z);
        imu_preintegrator_.setGravity(gravity_vec, norm);

        __android_log_print(ANDROID_LOG_INFO, TAG, "VIO Initialized from Gravity: direction=(%.3f, %.3f, %.3f), mag=%.2f, variance=%.3f",
                            gravity_direction_.x, gravity_direction_.y, gravity_direction_.z, norm, variance);

        return true;
    }

    void VisionModule::estimateScaleFromAccel(const cv::Mat& translation, long dt_ns) {
        if (dt_ns <= 0) return;

        double dt = dt_ns / 1e9;  // Convert to seconds

        // Get average acceleration during this frame
        cv::Point3f avg_accel = getAverageAccel(prev_timestamp_ns_, prev_timestamp_ns_ + dt_ns);

        // Check if we got valid accel data
        if (avg_accel.x == 0 && avg_accel.y == 0 && avg_accel.z == 0) {
            return;  // No accel data available
        }

        float gravity_magnitude = std::sqrt(gravity_direction_.x * gravity_direction_.x +
                                            gravity_direction_.y * gravity_direction_.y +
                                            gravity_direction_.z * gravity_direction_.z) * 9.81f;

        // Compute motion acceleration (remove gravity)
        cv::Point3f motion_accel;
        motion_accel.x = avg_accel.x - gravity_direction_.x * gravity_magnitude;
        motion_accel.y = avg_accel.y - gravity_direction_.y * gravity_magnitude;
        motion_accel.z = avg_accel.z - gravity_direction_.z * gravity_magnitude;

        float motion_accel_mag = std::sqrt(motion_accel.x * motion_accel.x +
                                           motion_accel.y * motion_accel.y +
                                           motion_accel.z * motion_accel.z);

        // Estimate displacement from acceleration: s = 0.5 * a * t^2
        double accel_displacement = 0.5 * motion_accel_mag * dt * dt;

        // Get vision-based displacement magnitude
        double vision_displacement = cv::norm(translation);

        if (vision_displacement > 0.001 && accel_displacement > 0.01) {
            // Estimate scale: actual_distance / vision_distance
            double new_scale = accel_displacement / vision_displacement;

            // Filter scale estimate (prevent sudden jumps)
            if (!scale_initialized_) {
                estimated_scale_ = new_scale;
                scale_initialized_ = true;
                __android_log_print(ANDROID_LOG_INFO, TAG,
                                    "Scale initialized: %.3f (accel: %.3f m, vision: %.3f)",
                                    estimated_scale_, accel_displacement, vision_displacement);
            } else {
                // Exponential moving average
                float alpha = 0.1f;
                estimated_scale_ = alpha * new_scale + (1.0f - alpha) * estimated_scale_;

                __android_log_print(ANDROID_LOG_DEBUG, TAG,
                                    "Scale updated: %.3f (accel: %.3f m, vision: %.3f)",
                                    estimated_scale_, accel_displacement, vision_displacement);
            }
        }
    }

    cv::Point3f VisionModule::getAverageAccel(long start_ns, long end_ns) {
        std::lock_guard<std::mutex> lock(accel_mutex_);

        cv::Point3f avg_accel(0, 0, 0);
        int count = 0;

        for (const auto& accel : accel_buffer_) {
            if (accel.timestamp_ns >= start_ns && accel.timestamp_ns <= end_ns) {
                avg_accel.x += accel.x;
                avg_accel.y += accel.y;
                avg_accel.z += accel.z;
                count++;
            }
        }

        if (count > 0) {
            avg_accel.x /= count;
            avg_accel.y /= count;
            avg_accel.z /= count;
        }

        return avg_accel;
    }

    bool VisionModule::checkMotionDegeneracy(
            const std::vector<cv::Point2f>& prev,
            const std::vector<cv::Point2f>& next) {

        if (prev.size() != next.size() || prev.empty()) {
            return true;  // Degenerate
        }

        // Calculate average motion
        float avg_motion = 0.0f;
        for (size_t i = 0; i < prev.size(); i++) {
            float dx = next[i].x - prev[i].x;
            float dy = next[i].y - prev[i].y;
            avg_motion += std::sqrt(dx * dx + dy * dy);
        }
        avg_motion /= prev.size();

        // Check if motion is too small
        if (avg_motion < 1.0f) {
            // __android_log_print(ANDROID_LOG_WARN, TAG, "Motion too small: %.2f pixels", avg_motion);
            return true;  // Degenerate: almost no motion
        }

        // Check if all points move in the same direction (pure rotation or translation)
        // Calculate motion variance
        float motion_variance = 0.0f;
        for (size_t i = 0; i < prev.size(); i++) {
            float dx = next[i].x - prev[i].x;
            float dy = next[i].y - prev[i].y;
            float motion = std::sqrt(dx * dx + dy * dy);
            float diff = motion - avg_motion;
            motion_variance += diff * diff;
        }
        motion_variance /= prev.size();

        if (motion_variance < 0.1f) {
            // __android_log_print(ANDROID_LOG_WARN, TAG, "Motion too uniform: variance = %.2f", motion_variance);
            return true;  // Degenerate: all points move together
        }

        return false;  // Not degenerate
    }

} // namespace navsight