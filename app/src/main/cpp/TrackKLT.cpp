#include "TrackKLT.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-TrackKLT"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGD(...) (void)0
#endif

TrackKLT::TrackKLT() {}

void TrackKLT::track(const cv::Mat& prev_gray, const cv::Mat& curr_gray,
                      const std::vector<cv::Point2f>& prev_pts,
                      std::vector<cv::Point2f>& curr_pts,
                      std::vector<uchar>& status,
                      const cv::Mat& delta_R, const cv::Mat& K,
                      int win_size) {
    if (prev_pts.empty()) {
        curr_pts.clear();
        status.clear();
        return;
    }

    // ── Step 1: Predict point positions using IMU rotation ────────────────────
    std::vector<cv::Point2f> predicted_pts;
    bool has_rotation = (cv::countNonZero(delta_R - cv::Mat::eye(3, 3, CV_64F)) > 0);

    if (has_rotation) {
        predictPoints(prev_pts, predicted_pts, delta_R, K);
        curr_pts = predicted_pts;
    } else {
        curr_pts = prev_pts; // default guess: stationary
    }

    // Plan Step 5: caller may grow the KLT search window when the IMU predicts
    // large per-frame pixel displacement (rapid head turn / scooter gyro).
    // Passing a non-positive win_size opts back to the class default to keep
    // the steady-state behaviour identical to pre-Step-5 builds.
    int wsz = (win_size > 0) ? win_size : WINDOW_SIZE;
    // KLT requires an odd window — round up if the caller passed an even
    // value (defensive; Tracker already enforces odd, but this keeps the
    // contract local to TrackKLT).
    if ((wsz & 1) == 0) wsz += 1;

    // ── Step 2: Forward KLT tracking ──────────────────────────────────────────
    std::vector<float> err;
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts,
        curr_pts, status, err, cv::Size(wsz, wsz),
        PYRAMID_LEVELS, criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW);

    // ── Step 3: Backward KLT check (FB-Consistency) ───────────────────────────
    std::vector<cv::Point2f> back_pts;
    std::vector<uchar> back_status;
    std::vector<float> back_err;

    cv::calcOpticalFlowPyrLK(curr_gray, prev_gray, curr_pts,
        back_pts, back_status, back_err, cv::Size(wsz, wsz),
        PYRAMID_LEVELS, criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            if (!back_status[i]) {
                status[i] = 0;
            } else {
                float dx = prev_pts[i].x - back_pts[i].x;
                float dy = prev_pts[i].y - back_pts[i].y;
                if (dx * dx + dy * dy > FB_CHECK_THRESH) {
                    status[i] = 0;
                }
            }
        }
    }

    // ── Step 4: Boundary check ───────────────────────────────────────────────
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            if (curr_pts[i].x < 0 || curr_pts[i].x >= curr_gray.cols ||
                curr_pts[i].y < 0 || curr_pts[i].y >= curr_gray.rows) {
                status[i] = 0;
            }
        }
    }
}

bool TrackKLT::geometricVerification(const std::vector<cv::Point2f>& prev_pts,
                                      std::vector<cv::Point2f>& curr_pts,
                                      std::vector<uchar>& status,
                                      const cv::Mat& K,
                                      cv::Mat& R_cam, cv::Mat& t_cam,
                                      int& inliers_count) {
    inliers_count = 0;
    std::vector<cv::Point2f> pts1, pts2;
    std::vector<int> original_indices;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            pts1.push_back(prev_pts[i]);
            pts2.push_back(curr_pts[i]);
            original_indices.push_back(static_cast<int>(i));
        }
    }

    if (pts1.size() < 8) return false;

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 
                                     RANSAC_CONF, RANSAC_THRESH, mask);

    if (E.empty() || E.rows != 3 || E.cols != 3) return false;

    // recoverPose returns the relative rotation and translation (unit vector)
    int inliers = cv::recoverPose(E, pts1, pts2, K, R_cam, t_cam, mask);
    inliers_count = inliers;

    // Update the input status vector using the RANSAC mask
    for (size_t i = 0; i < original_indices.size(); ++i) {
        if (!mask.at<uchar>(static_cast<int>(i))) {
            status[original_indices[i]] = 0;
        }
    }

    return (inliers >= MIN_INLIERS);
}

void TrackKLT::predictPoints(const std::vector<cv::Point2f>& prev_pts,
                              std::vector<cv::Point2f>& predicted_pts,
                              const cv::Mat& delta_R, const cv::Mat& K) {
    predicted_pts.resize(prev_pts.size());
    
    // Homography from rotation: H = K * R * K_inv
    cv::Mat H = K * delta_R * K.inv();
    
    // Ensure H is CV_64F for precision
    if (H.type() != CV_64F) H.convertTo(H, CV_64F);

    double h11 = H.at<double>(0, 0), h12 = H.at<double>(0, 1), h13 = H.at<double>(0, 2);
    double h21 = H.at<double>(1, 0), h22 = H.at<double>(1, 1), h23 = H.at<double>(1, 2);
    double h31 = H.at<double>(2, 0), h32 = H.at<double>(2, 1), h33 = H.at<double>(2, 2);

    for (size_t i = 0; i < prev_pts.size(); ++i) {
        double x = prev_pts[i].x;
        double y = prev_pts[i].y;
        double w = h31 * x + h32 * y + h33;
        if (std::abs(w) > 1e-7) {
            predicted_pts[i].x = static_cast<float>((h11 * x + h12 * y + h13) / w);
            predicted_pts[i].y = static_cast<float>((h21 * x + h22 * y + h23) / w);
        } else {
            predicted_pts[i] = prev_pts[i];
        }
    }
}
