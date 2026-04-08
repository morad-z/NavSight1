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

void LensCorrector::setDistortion(double k1, double k2, double p1, double p2, double k3) {
    dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
    LOGI("Distortion set: k1=%.4f k2=%.4f p1=%.4f p2=%.4f k3=%.4f", k1, k2, p1, p2, k3);
}

void LensCorrector::undistortPoints(std::vector<cv::Point2f>& points) const {
    if (!has_intrinsics_ || points.empty()) return;

    // undistortPoints outputs normalized coords; we re-project to pixel coords
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(points, undistorted, camera_matrix_, dist_coeffs_,
                        cv::noArray(), camera_matrix_);
    points = std::move(undistorted);
}

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
