#pragma once
#include <vector>
#include <mutex>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

/**
 * Loop Closure Detector
 * 
 * Detects when a user returns to a previously visited location by comparing
 * visual descriptors and position estimates. When a match is found, forces
 * position and heading consistency to correct accumulated drift.
 * 
 * This solves the drift accumulation problem: over long walks, VIO drift can
 * be 11+ meters. When returning to the start, loop closure detection recognizes
 * the familiar visual environment and snaps the solution back to the original pose.
 */

struct LoopClosureKeyframe {
    int64_t timestamp_ns;
    cv::Point3f position;          // VIO position estimate at this keyframe
    float heading;                 // Heading in radians
    cv::Mat descriptors;           // ORB/BRIEF descriptors (Nx32 uint8)
    std::vector<cv::Point2f> keypoints; // 2D image coordinates
    int keyframe_id;               // Sequential ID for tracking
};

class LoopClosureDetector {
public:
    LoopClosureDetector();
    ~LoopClosureDetector();

    /**
     * Check if the current frame has a loop closure match with previously stored keyframes.
     * 
     * Returns true if:
     * - Visual descriptors match with similarity > threshold (8+ matching features)
     * - Position distance to matched keyframe < threshold (adaptive, ~0.5-1.0m)
     * - Heading difference is reasonable (< 90°)
     * 
     * @param current_descriptors ORB descriptors from current frame (Nx32 uint8)
     * @param current_keypoints 2D points in current frame
     * @param current_position VIO position estimate
     * @param current_heading VIO heading estimate
     * @param matched_keyframe Output: the matched keyframe if true is returned
     * @return true if loop closure detected
     */
    bool detectLoopClosure(
        const cv::Mat& current_descriptors,
        const std::vector<cv::Point2f>& current_keypoints,
        const cv::Point3f& current_position,
        float current_heading,
        LoopClosureKeyframe& matched_keyframe
    );

    /**
     * Store a keyframe for future loop closure matching.
     * Called periodically (e.g., every 10 frames or 0.3m movement).
     */
    void addKeyframe(
        const LoopClosureKeyframe& keyframe
    );

    /**
     * Clear all stored keyframes (e.g., when resetting VIO).
     */
    void reset();

    /**
     * Get the total number of stored keyframes.
     */
    int getKeyframeCount() const;

private:
    mutable std::mutex mutex_;
    std::vector<LoopClosureKeyframe> keyframes_;
    cv::Ptr<cv::ORB> orb_matcher_;
    int next_keyframe_id_{0};

    // Thresholds
    static constexpr int MIN_DESCRIPTOR_MATCHES = 8;      // Min features that must match
    static constexpr float POSITION_MATCH_THRESHOLD_M = 1.0f;  // Must be within 1m
    static constexpr float HEADING_MATCH_THRESHOLD_RAD = M_PI / 2.0f;  // Must be within 90°
    static constexpr int KEYFRAME_SKIP = 10;              // Store every Nth frame

    // Internal state
    int frame_count_{0};

    /**
     * Compute Hamming distance between two ORB descriptor rows.
     * Lower distance = more similar.
     */
    static int hammingDistance(const uint8_t* a, const uint8_t* b, int bytes = 32);

    /**
     * Count matching descriptors between two descriptor sets using Hamming distance.
     * Uses a simple greedy matching (could be improved with FLANN for larger databases).
     */
    static int matchDescriptors(
        const cv::Mat& descriptors1,  // Nx32
        const cv::Mat& descriptors2,  // Mx32
        float max_distance_ratio = 0.7f
    );
};
