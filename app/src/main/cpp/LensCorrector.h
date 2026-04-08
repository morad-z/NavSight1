#pragma once
#include <vector>
#include <opencv2/core.hpp>

// Corrects lens distortion for phone cameras.
// Undistorts feature points before essential matrix estimation
// to eliminate systematic rotation errors from barrel distortion.
class LensCorrector {
public:
    LensCorrector();

    // Set camera intrinsics (must be called before undistort)
    void setIntrinsics(double fx, double fy, double cx, double cy);

    // DEAD CODE: setDistortion — never called
    // void setDistortion(double k1, double k2, double p1, double p2, double k3 = 0.0);

    // DEAD CODE: undistortPoints (single-set) — never called
    // void undistortPoints(std::vector<cv::Point2f>& points) const;

    // Undistort two matched point sets (prev, next) for essential matrix
    void undistortMatchedPoints(std::vector<cv::Point2f>& prev,
                                std::vector<cv::Point2f>& next) const;

    bool isReady() const { return has_intrinsics_; }

private:
    cv::Mat camera_matrix_;    // 3x3
    cv::Mat dist_coeffs_;      // 5x1 [k1, k2, p1, p2, k3]
    bool has_intrinsics_{false};

    // Default: zero distortion (no correction if uncalibrated)
    // Wrong distortion coefficients are WORSE than no correction —
    // they corrupt point positions and cause essential matrix failure.
    static constexpr double DEFAULT_K1 = 0.0;
    static constexpr double DEFAULT_K2 = 0.0;
};
