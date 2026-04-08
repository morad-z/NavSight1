#include "LensCorrector.h"
#include <opencv2/calib3d.hpp>
#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Lens"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#endif

LensCorrector::LensCorrector() {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs_ = (cv::Mat_<double>(5, 1) << DEFAULT_K1, DEFAULT_K2, 0.0, 0.0, 0.0);
}

void LensCorrector::setIntrinsics(double fx, double fy, double cx, double cy) {
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    has_intrinsics_ = true;
}

// DEAD CODE: setDistortion — never called (default zero distortion used)
// void LensCorrector::setDistortion(double k1, double k2, double p1, double p2, double k3) { ... }

// DEAD CODE: undistortPoints (single-set) — never called; only undistortMatchedPoints is used
// void LensCorrector::undistortPoints(std::vector<cv::Point2f>& points) const { ... }

void LensCorrector::undistortMatchedPoints(std::vector<cv::Point2f>& prev,
                                            std::vector<cv::Point2f>& next) const {
    if (!has_intrinsics_ || prev.empty() || next.empty()) return;

    std::vector<cv::Point2f> prev_ud, next_ud;
    cv::undistortPoints(prev, prev_ud, camera_matrix_, dist_coeffs_,
                        cv::noArray(), camera_matrix_);
    cv::undistortPoints(next, next_ud, camera_matrix_, dist_coeffs_,
                        cv::noArray(), camera_matrix_);
    prev = std::move(prev_ud);
    next = std::move(next_ud);
}
