#include "LensCorrector.h"
#include <opencv2/calib3d.hpp>
#include <cmath>
#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Lens"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#endif

namespace {
constexpr double kDistortionEpsilon = 1e-9;
}

LensCorrector::LensCorrector() {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    // 8-coefficient rational model: [k1, k2, p1, p2, k3, k4, k5, k6].
    dist_coeffs_ = (cv::Mat_<double>(8, 1) <<
                    DEFAULT_K1, DEFAULT_K2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void LensCorrector::setIntrinsics(double fx, double fy, double cx, double cy) {
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    has_intrinsics_ = (fx > 0.0 && fy > 0.0);
}

void LensCorrector::setDistortion(double k1, double k2, double k3,
                                  double k4, double k5, double k6,
                                  double p1, double p2) {
    // Guard: if intrinsics are degenerate, distortion correction would
    // operate on a meaningless camera_matrix_ — refuse and stay passthrough.
    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    if (fx <= 0.0 || fy <= 0.0) {
        has_distortion_ = false;
        LOGI("setDistortion: fx/fy not yet set, staying passthrough");
        return;
    }

    // Guard: if every coefficient is effectively zero, undistortion is a
    // no-op and we save the OpenCV call. Treat as passthrough.
    const bool all_zero =
        std::abs(k1) <= kDistortionEpsilon && std::abs(k2) <= kDistortionEpsilon &&
        std::abs(k3) <= kDistortionEpsilon && std::abs(k4) <= kDistortionEpsilon &&
        std::abs(k5) <= kDistortionEpsilon && std::abs(k6) <= kDistortionEpsilon &&
        std::abs(p1) <= kDistortionEpsilon && std::abs(p2) <= kDistortionEpsilon;
    if (all_zero) {
        has_distortion_ = false;
        LOGI("setDistortion: all coefficients zero, staying passthrough");
        return;
    }

    // OpenCV order is [k1, k2, p1, p2, k3, k4, k5, k6] for the rational model.
    dist_coeffs_ = (cv::Mat_<double>(8, 1) << k1, k2, p1, p2, k3, k4, k5, k6);
    has_distortion_ = true;
    LOGI("setDistortion: k1=%.5f k2=%.5f k3=%.5f p1=%.5f p2=%.5f (rational k4..k6=%.3g,%.3g,%.3g)",
         k1, k2, k3, p1, p2, k4, k5, k6);
}

void LensCorrector::undistortPoints(std::vector<cv::Point2f>& points) const {
    if (!has_intrinsics_ || !has_distortion_ || points.empty()) return;

    std::vector<cv::Point2f> ud;
    cv::undistortPoints(points, ud, camera_matrix_, dist_coeffs_,
                        cv::noArray(), camera_matrix_);
    points = std::move(ud);
}

void LensCorrector::undistortMatchedPoints(std::vector<cv::Point2f>& prev,
                                            std::vector<cv::Point2f>& next) const {
    if (!has_intrinsics_ || !has_distortion_ || prev.empty() || next.empty()) return;

    std::vector<cv::Point2f> prev_ud, next_ud;
    cv::undistortPoints(prev, prev_ud, camera_matrix_, dist_coeffs_,
                        cv::noArray(), camera_matrix_);
    cv::undistortPoints(next, next_ud, camera_matrix_, dist_coeffs_,
                        cv::noArray(), camera_matrix_);
    prev = std::move(prev_ud);
    next = std::move(next_ud);
}
