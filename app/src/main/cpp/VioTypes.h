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

    // Diagnostic fields for simulation recording
    double meanFlow;
    int    inlierCount;
    int    stepCount;
    double stepFreq;
    double strideLength;
    int    poseFlags;        // bit0=static, bit1=pureRot, bit2=poseValid, bit3=fallback
    double heading;
};
