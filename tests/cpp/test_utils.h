#pragma once
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <opencv2/core.hpp>
#include "Tracker.h"
#include "IMUPreintegrator.h"
#include "VioEngine.h"

namespace test_utils {

// Create a synthetic NV21 frame with bright dots at known positions
// NV21 layout: Y plane (width*height bytes) + interleaved VU plane (width*height/2 bytes)
inline std::vector<uint8_t> createSyntheticNV21(
        int width, int height,
        const std::vector<cv::Point2f>& features,
        uint8_t background = 80) {

    int y_size = width * height;
    int uv_size = width * height / 2;
    std::vector<uint8_t> buffer(y_size + uv_size, 128);

    // Fill Y plane with background
    std::memset(buffer.data(), background, y_size);
    // UV plane stays at 128 (neutral chroma)

    // Draw bright 5x5 squares at feature positions (good for goodFeaturesToTrack)
    for (const auto& pt : features) {
        int px = static_cast<int>(pt.x);
        int py = static_cast<int>(pt.y);
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                int x = px + dx, y = py + dy;
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    buffer[y * width + x] = 255;
                }
            }
        }
    }
    return buffer;
}

// Generate a grid of feature points within the image
inline std::vector<cv::Point2f> generateFeatureGrid(
        int width, int height, int gridCols = 10, int gridRows = 8, int margin = 30) {

    std::vector<cv::Point2f> pts;
    float dx = static_cast<float>(width - 2 * margin) / (gridCols - 1);
    float dy = static_cast<float>(height - 2 * margin) / (gridRows - 1);
    for (int r = 0; r < gridRows; r++) {
        for (int c = 0; c < gridCols; c++) {
            pts.emplace_back(margin + c * dx, margin + r * dy);
        }
    }
    return pts;
}

// Shift all features by (tx, ty) pixels — simulates camera translation
inline std::vector<cv::Point2f> shiftFeatures(
        const std::vector<cv::Point2f>& pts, float tx, float ty,
        int width, int height, int margin = 5) {

    std::vector<cv::Point2f> shifted;
    for (const auto& p : pts) {
        float nx = p.x + tx, ny = p.y + ty;
        if (nx >= margin && nx < width - margin && ny >= margin && ny < height - margin) {
            shifted.emplace_back(nx, ny);
        }
    }
    return shifted;
}

// IMU data dispatch helpers — handle both IMUPreintegrator (addGyroReading)
// and VioEngine (addGyroData) method naming
namespace detail {
    // IMUPreintegrator overloads
    inline void addGyro(IMUPreintegrator& imu, int64_t ts, float x, float y, float z) {
        imu.addGyroReading(ts, x, y, z);
    }
    inline void addAccel(IMUPreintegrator& imu, int64_t ts, float x, float y, float z) {
        imu.addAccelReading(ts, x, y, z);
    }
    // VioEngine overloads
    inline void addGyro(VioEngine& engine, int64_t ts, float x, float y, float z) {
        engine.addGyroData(ts, x, y, z);
    }
    inline void addAccel(VioEngine& engine, int64_t ts, float x, float y, float z) {
        engine.addAccelData(ts, x, y, z);
    }
}

// Feed static IMU data (gravity only, no rotation)
template<typename T>
void feedStaticIMU(T& vm, int64_t start_ns, int duration_ms, int rate_hz = 200) {
    int64_t dt_ns = 1'000'000'000LL / rate_hz;
    int samples = duration_ms * rate_hz / 1000;
    for (int i = 0; i < samples; i++) {
        int64_t ts = start_ns + i * dt_ns;
        detail::addGyro(vm, ts, 0.0f, 0.0f, 0.0f);
        detail::addAccel(vm, ts, 0.0f, 9.81f, 0.0f); // phone upright, Y is up
    }
}

// Feed walking IMU data with realistic step-like accel pattern
template<typename T>
void feedWalkingIMU(T& vm, int64_t start_ns, int duration_ms,
                    float forward_accel = 0.3f, int rate_hz = 200) {
    int64_t dt_ns = 1'000'000'000LL / rate_hz;
    int samples = duration_ms * rate_hz / 1000;
    double step_freq = 1.3;
    for (int i = 0; i < samples; i++) {
        int64_t ts = start_ns + i * dt_ns;
        double t = i * (1.0 / rate_hz);
        float step_component = 0.7f * static_cast<float>(
            std::sin(2.0 * M_PI * step_freq * t));
        if (step_component > 0) step_component *= 1.5f;
        detail::addGyro(vm, ts, 0.0f, 0.0f, 0.0f);
        detail::addAccel(vm, ts, forward_accel, 9.81f + step_component, 0.0f);
    }
}

// Feed constant-rotation IMU data
template<typename T>
void feedRotatingIMU(T& vm, int64_t start_ns, int duration_ms,
                     float gx, float gy, float gz, int rate_hz = 200) {
    int64_t dt_ns = 1'000'000'000LL / rate_hz;
    int samples = duration_ms * rate_hz / 1000;
    for (int i = 0; i < samples; i++) {
        int64_t ts = start_ns + i * dt_ns;
        detail::addGyro(vm, ts, gx, gy, gz);
        detail::addAccel(vm, ts, 0.0f, 9.81f, 0.0f);
    }
}

// Create a checkerboard NV21 frame (same pattern as Kotlin on-device tests).
// Checkerboard produces strong Harris corners at every grid intersection,
// which is reliable for goodFeaturesToTrack + optical flow.
inline std::vector<uint8_t> createCheckerboardNV21(
        int width, int height,
        int cellSize = 20,
        int offsetX = 0,
        int offsetY = 0) {

    int y_size = width * height;
    int uv_size = width * height / 2;
    std::vector<uint8_t> buffer(y_size + uv_size);

    // Y plane: checkerboard
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int cx = (x + offsetX) / cellSize;
            int cy = (y + offsetY) / cellSize;
            bool isWhite = ((cx + cy) % 2 == 0);
            buffer[y * width + x] = isWhite ? 220 : 40;
        }
    }
    // UV plane: neutral chroma
    std::memset(buffer.data() + y_size, 128, uv_size);
    return buffer;
}

// Compare rotation matrix to expected rotation angle around an axis
inline double rotationAngleError(const cv::Mat& R, const cv::Mat& R_expected) {
    cv::Mat dR = R * R_expected.t();
    double trace = dR.at<double>(0, 0) + dR.at<double>(1, 1) + dR.at<double>(2, 2);
    double cos_angle = (trace - 1.0) / 2.0;
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle); // radians
}

// Rotation matrix around Z axis
inline cv::Mat rotZ(double angle_rad) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    R.at<double>(0, 0) = std::cos(angle_rad);
    R.at<double>(0, 1) = -std::sin(angle_rad);
    R.at<double>(1, 0) = std::sin(angle_rad);
    R.at<double>(1, 1) = std::cos(angle_rad);
    return R;
}

// Helper: process a frame through Tracker (wraps the TrackerFrame out-param)
inline VisionOutput processFrame(Tracker& tracker, IMUPreintegrator& imu,
                                  const std::vector<uint8_t>& frame,
                                  int width, int height, int64_t ts) {
    TrackerFrame frame_out;
    return tracker.processFrame(frame.data(), width, height, ts, imu, frame_out);
}

// Helper: process a frame through VioEngine
inline VisionOutput processFrameEngine(VioEngine& engine,
                                        const std::vector<uint8_t>& frame,
                                        int width, int height, int64_t ts) {
    return engine.processFrame(frame.data(), width, height, ts);
}

} // namespace test_utils
