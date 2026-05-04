#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

class TrackKLT {
public:
    TrackKLT();

    /**
     * @brief Performs KLT tracking from prev frame to current frame.
     *
     * @param prev_gray Previous frame (grayscale)
     * @param curr_gray Current frame (grayscale)
     * @param prev_pts Points in the previous frame
     * @param curr_pts Predicted/Initial guess for points in current frame (output)
     * @param status Status of each point (1 if tracked, 0 otherwise)
     * @param delta_R Inter-frame rotation from IMU (optional, eye(3,3) if none)
     * @param K Camera intrinsics matrix
     * @param win_size Optional KLT window size (must be odd). Pass <= 0 to use
     *                 the class default WINDOW_SIZE. Plan Step 5: Tracker uses
     *                 this to grow the window when expected per-frame pixel
     *                 displacement (from gyro) exceeds the static budget.
     */
    void track(const cv::Mat& prev_gray, const cv::Mat& curr_gray,
               const std::vector<cv::Point2f>& prev_pts,
               std::vector<cv::Point2f>& curr_pts,
               std::vector<uchar>& status,
               const cv::Mat& delta_R, const cv::Mat& K,
               int win_size = -1);

    /**
     * @brief Performs geometric outlier rejection using the 5-point algorithm (RANSAC).
     * 
     * @param prev_pts Points in the previous frame (must be same size as curr_pts)
     * @param curr_pts Points in the current frame
     * @param status Input/Output status vector (0 for outliers)
     * @param K Camera intrinsics matrix
     * @param R_cam Output camera rotation (prev to curr)
     * @param t_cam Output camera translation (unit vector)
     * @param inliers_count Number of remaining inliers after RANSAC
     * @return true if verification succeeded with sufficient inliers
     */
    bool geometricVerification(const std::vector<cv::Point2f>& prev_pts,
                                std::vector<cv::Point2f>& curr_pts,
                                std::vector<uchar>& status,
                                const cv::Mat& K,
                                cv::Mat& R_cam, cv::Mat& t_cam,
                                int& inliers_count);

    /**
     * @brief Predicts feature positions in the current frame using IMU rotation.
     * 
     * @param prev_pts Points in the previous frame
     * @param predicted_pts Output predicted points
     * @param delta_R Inter-frame rotation from IMU
     * @param K Camera intrinsics matrix
     */
    void predictPoints(const std::vector<cv::Point2f>& prev_pts,
                       std::vector<cv::Point2f>& predicted_pts,
                       const cv::Mat& delta_R, const cv::Mat& K);

private:
    // Constants matching Tracker.h or OpenVINS defaults
    static constexpr int    PYRAMID_LEVELS     = 4;
    static constexpr int    WINDOW_SIZE        = 21;  // Was 31; 21 is enough for 640x480
    static constexpr double RANSAC_CONF        = 0.999;
    static constexpr double RANSAC_THRESH      = 1.5;  // pixels (0.5 too tight for uncalibrated)
    static constexpr int    MIN_INLIERS        = 8;
    static constexpr double FB_CHECK_THRESH    = 4.0;  // 2px (squared), matching Tracker.h
};
