#pragma once
#include <vector>
#include <opencv2/core.hpp>

// Shared output struct for the VIO pipeline.
// Matches the JNI VioData constructor in native-lib.cpp.
struct VisionOutput {
    cv::Mat R;               // 3x3 rotation matrix (CV_64F) - FUSED
    cv::Mat t;               // 3x1 translation vector (CV_64F) - FUSED
    cv::Mat rawR;            // 3x3 rotation matrix (CV_64F) - RAW CAMERA
    cv::Mat rawT;            // 3x1 translation vector (CV_64F) - RAW CAMERA
    double quality;
    int trackedCount;
    int totalCount;
    double estimatedScale;
    bool valid;
    std::vector<float> trackedPoints; // [x0,y0, x1,y1, ...]

    // Phase 2 camera overlay: ages (in FRAMES survived) parallel to
    // trackedPoints. trackedPointAges.size() == trackedPoints.size() / 2.
    // Empty when not populated; consumers must check size() before use.
    // At 30 Hz: <30 frames is "new" (≤1 s), 30-89 is "established"
    // (1-3 s), ≥90 is "mature" (≥3 s).
    std::vector<int> trackedPointAges;

    // 2026-06-02 camera overlay — per-point recoverPose inlier flag, parallel to trackedPoints
    // (size == trackedPoints.size()/2): 1 = the VIO actually USED this point (RANSAC inlier),
    // 0 = tracked by KLT but rejected / verification not attempted this frame. Lets the overlay
    // show the REAL points the engine used vs the "pre-calculation" KLT candidates. Empty when
    // not populated; consumers must check size().
    std::vector<unsigned char> trackedPointInlierFlags;

    // Diagnostic fields for simulation recording
    double meanFlow;
    int    inlierCount;
    int    stepCount;
    double stepFreq;
    double strideLength;
    int    poseFlags;        // bit0=static, bit1=pureRot, bit2=poseValid, bit3=fallback
    double heading;
    double td_imu_cam;       // Camera-to-IMU time offset (seconds)
    bool   keyframe_stored;  // Step 9 (ADR-014): true on the frame a keyframe was
                             // committed (frames_since_keyframe_ resets to 0).
                             // Lets the replay scorer compute keyframe-rate metrics
                             // (keyframe_match_count_p95) without keyframe-side logging.
};
