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

    // Set distortion coefficients [k1, k2, p1, p2, k3]
    // Default: typical phone camera barrel distortion
    void setDistortion(double k1, double k2, double p1, double p2, double k3 = 0.0);

    // Undistort a set of 2D points in-place using the current model
    void undistortPoints(std::vector<cv::Point2f>& points) const;

    // Undistort two matched point sets (prev, next) for essential matrix
    void undistortMatchedPoints(std::vector<cv::Point2f>& prev,
                                std::vector<cv::Point2f>& next) const;

    bool isReady() const { return has_intrinsics_; }

private:
    cv::Mat camera_matrix_;    // 3x3
    cv::Mat dist_coeffs_;      // 5x1 [k1, k2, p1, p2, k3]
    bool has_intrinsics_{false};

    // Default distortion for typical phone cameras (mild barrel)
    static constexpr double DEFAULT_K1 = -0.08;
    static constexpr double DEFAULT_K2 =  0.01;
};
