#pragma once
#include <vector>
#include <opencv2/core.hpp>

// Corrects lens distortion for phone cameras.
// Undistorts feature points before essential matrix estimation
// to eliminate systematic rotation errors from barrel distortion.
//
// Step 1 (Visual Production Plan): re-enabled setDistortion + single-set
// undistortPoints. Wrong distortion coefficients are WORSE than no correction
// — the JSON loader (CameraCalibration.h) gates RMS reprojection error to
// ensure only validated coefficients reach this class.
class LensCorrector {
public:
    LensCorrector();

    // Set camera intrinsics (must be called before undistort)
    void setIntrinsics(double fx, double fy, double cx, double cy);

    // Set distortion coefficients (rational model, 8 coefficients).
    // OpenCV order: [k1, k2, p1, p2, k3, k4, k5, k6].
    // If all coefficients are within 1e-9 of zero OR fx/fy <= 0 (intrinsics
    // not yet set), the corrector falls back to a passthrough (no harm).
    void setDistortion(double k1, double k2, double k3,
                       double k4, double k5, double k6,
                       double p1, double p2);

    // Undistort a single point set (in place). No-op if not ready.
    void undistortPoints(std::vector<cv::Point2f>& points) const;

    // Undistort two matched point sets (prev, next) for essential matrix
    void undistortMatchedPoints(std::vector<cv::Point2f>& prev,
                                std::vector<cv::Point2f>& next) const;

    bool isReady() const { return has_intrinsics_; }
    bool hasDistortion() const { return has_distortion_; }

private:
    cv::Mat camera_matrix_;    // 3x3
    cv::Mat dist_coeffs_;      // 8x1 rational: [k1, k2, p1, p2, k3, k4, k5, k6]
    bool has_intrinsics_{false};
    bool has_distortion_{false};

    // Default: zero distortion (no correction if uncalibrated)
    // Wrong distortion coefficients are WORSE than no correction —
    // they corrupt point positions and cause essential matrix failure.
    static constexpr double DEFAULT_K1 = 0.0;
    static constexpr double DEFAULT_K2 = 0.0;
};
