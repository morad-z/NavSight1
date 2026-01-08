# Vision Module API Documentation

**For Student 2 (Navigation Core & Path Estimation)**

This document explains how to use Student 1's Vision & Sensor Fusion module in your navigation core implementation.

## Overview

The VisionModule provides camera-based motion estimation using optical flow and sensor fusion with gyroscope data. It outputs rotation and translation information that you can use for path estimation and navigation.

## File Locations

- **Header**: `app/src/main/cpp/VisionModule.h`
- **Implementation**: `app/src/main/cpp/VisionModule.cpp`
- **Current Integration**: `app/src/main/cpp/native-lib.cpp`

## Quick Start

### 1. Include the Header

```cpp
#include "VisionModule.h"
```

### 2. Create a VisionModule Instance

```cpp
// Create with default settings (Shi-Tomasi detector, 200 features)
navsight::VisionModule vision;

// Or specify detector type and max features
navsight::VisionModule vision(
    navsight::FeatureDetectorType::GOOD_FEATURES,
    200  // max features
);
```

### 3. Process Camera Frames

```cpp
// In your camera callback:
navsight::VisionOutput output = vision.processFrame(
    yuv_data,      // uint8_t* - YUV420 format image data
    width,         // int - image width
    height,        // int - image height
    timestamp_ns   // long - timestamp in nanoseconds
);

if (output.is_valid) {
    // Use output.rotation and output.translation
    // for your navigation calculations
}
```

### 4. Add Gyroscope Data (for better rotation accuracy)

```cpp
// In your gyroscope sensor callback:
navsight::GyroData gyro;
gyro.timestamp_ns = timestamp_ns;
gyro.x = angular_velocity_x;  // rad/s
gyro.y = angular_velocity_y;
gyro.z = angular_velocity_z;

vision.addGyroData(gyro);
```

### 5. Add Accelerometer Data (for automatic scale estimation)

```cpp
// In your accelerometer sensor callback:
navsight::AccelData accel;
accel.timestamp_ns = timestamp_ns;
accel.x = acceleration_x;  // m/s^2
accel.y = acceleration_y;
accel.z = acceleration_z;

vision.addAccelData(accel);
```

**Important**: Adding accelerometer data enables:
- Automatic gravity-based initialization
- Automatic scale estimation (solves monocular scale ambiguity)
- Better overall accuracy

## Data Structures

### VisionOutput

The main output from `processFrame()`:

```cpp
struct VisionOutput {
    cv::Mat rotation;              // 3x3 rotation matrix (relative motion)
    cv::Mat translation;           // 3x1 translation vector (relative motion)

    int tracked_features;          // Number of features successfully tracked
    int total_features;            // Total features attempted
    float tracking_quality;        // 0.0 to 1.0 (quality metric)

    long timestamp_ns;             // Frame timestamp

    std::vector<cv::Point2f> tracked_points;  // For UI visualization

    bool is_valid;                 // Whether output is reliable
};
```

**Important Notes:**
- `rotation` and `translation` represent **relative motion** since the last frame
- To get global pose, you must integrate these over time (see example below)
- `is_valid` will be `false` if tracking failed or not enough features were found
- `tracking_quality` > 0.7 is generally good, < 0.3 indicates poor tracking

### GyroData

Gyroscope input structure:

```cpp
struct GyroData {
    long timestamp_ns;   // Nanoseconds since epoch
    float x, y, z;       // Angular velocity in rad/s
};
```

### AccelData

Accelerometer input structure:

```cpp
struct AccelData {
    long timestamp_ns;   // Nanoseconds since epoch
    float x, y, z;       // Acceleration in m/s^2
};
```

## Feature Detector Types

You can choose different feature detectors based on your needs:

```cpp
enum class FeatureDetectorType {
    GOOD_FEATURES,  // Shi-Tomasi corner detector (default, balanced)
    FAST,           // FAST corner detector (very fast, less accurate)
    ORB             // ORB features (rotation invariant, slower)
};
```

**Recommendations:**
- **GOOD_FEATURES**: Default choice, good balance of speed and accuracy
- **FAST**: Use if you need maximum performance and can tolerate lower accuracy
- **ORB**: Use if the camera rotates significantly

## Integration Example: Building Global Pose

Here's how to use VisionModule output to build a global position estimate:

```cpp
class NavigationCore {
private:
    navsight::VisionModule vision_;

    // Global pose state
    cv::Mat global_R_;  // 3x3 rotation matrix
    cv::Mat global_t_;  // 3x1 translation vector
    double scale_;      // Scale factor (monocular cameras have scale ambiguity)

public:
    NavigationCore()
        : vision_(navsight::FeatureDetectorType::GOOD_FEATURES, 200),
          global_R_(cv::Mat::eye(3, 3, CV_64F)),
          global_t_(cv::Mat::zeros(3, 1, CV_64F)),
          scale_(1.0) {}

    void processFrame(const uint8_t* yuv_data, int width, int height, long timestamp_ns) {
        // Get relative motion from vision module
        navsight::VisionOutput output = vision_.processFrame(
            yuv_data, width, height, timestamp_ns
        );

        if (!output.is_valid) {
            // Tracking failed, handle gracefully
            return;
        }

        // Update global pose
        // Translation: t_global = t_global + scale * (R_global * t_relative)
        global_t_ = global_t_ + (scale_ * (global_R_ * output.translation));

        // Rotation: R_global = R_relative * R_global
        global_R_ = output.rotation * global_R_;

        // Now you have the global camera pose!
        // Use global_t_ for position (x, y, z in camera frame)
        // Use global_R_ for orientation
    }

    void processGyro(long timestamp_ns, float x, float y, float z) {
        navsight::GyroData gyro;
        gyro.timestamp_ns = timestamp_ns;
        gyro.x = x;
        gyro.y = y;
        gyro.z = z;
        vision_.addGyroData(gyro);
    }

    void processAccel(long timestamp_ns, float x, float y, float z) {
        navsight::AccelData accel;
        accel.timestamp_ns = timestamp_ns;
        accel.x = x;
        accel.y = y;
        accel.z = z;
        vision_.addAccelData(accel);
    }

    bool isInitialized() const {
        return vision_.isInitialized();
    }

    double getEstimatedScale() const {
        return vision_.getEstimatedScale();
    }

    void reset() {
        vision_.reset();
        global_R_ = cv::Mat::eye(3, 3, CV_64F);
        global_t_ = cv::Mat::zeros(3, 1, CV_64F);
    }
};
```

## Configuration Options

You can adjust VisionModule behavior:

```cpp
// Change feature detector type
vision.setFeatureDetectorType(navsight::FeatureDetectorType::FAST);

// Adjust maximum features to track
vision.setMaxFeatures(300);  // More features = more accurate but slower

// Adjust gyro fusion weight (0.0 = pure vision, 1.0 = pure gyro)
vision.setGyroFusionWeight(0.98);  // Default: heavily favor gyro
```

## Monitoring Performance

Get statistics about tracking performance:

```cpp
navsight::VisionModule::Statistics stats = vision.getStatistics();

std::cout << "Frames processed: " << stats.total_frames_processed << std::endl;
std::cout << "Successful tracks: " << stats.successful_tracks << std::endl;
std::cout << "Average features: " << stats.average_features_tracked << std::endl;
std::cout << "Average quality: " << stats.average_tracking_quality << std::endl;
```

## Coordinate Frames

**Understanding the output coordinate system:**

- **X-axis**: Right (in camera view)
- **Y-axis**: Down (in camera view)
- **Z-axis**: Forward (camera looking direction)

The rotation matrix `R` transforms points from the previous camera frame to the current camera frame.

The translation vector `t` represents the camera motion in the previous camera frame's coordinate system.

## Automatic Scale Estimation (NEW!)

The VisionModule now includes **automatic scale estimation** using accelerometer data:

### How It Works

1. **Initialization**: The module collects 20+ accelerometer readings to determine gravity direction
2. **Motion Detection**: When the camera moves, it estimates distance from acceleration
3. **Scale Calibration**: Compares accelerometer-based distance to vision-based distance
4. **Continuous Update**: Refines scale estimate over time with exponential smoothing

### Using Automatic Scale

```cpp
// Check if initialized
if (vision.isInitialized()) {
    // Get estimated scale (meters per vision unit)
    double scale = vision.getEstimatedScale();

    // Apply to translation
    cv::Mat real_world_translation = scale * vision_translation;
}
```

### Benefits

- **No manual calibration needed**: Scale is estimated automatically
- **Solves monocular ambiguity**: Provides real-world metric scale
- **Continuously refined**: Gets better over time as more data is collected

### Requirements

- Must call `addAccelData()` regularly (at sensor rate, e.g., 100 Hz)
- Device should be moving (not stationary) for initial scale estimation
- Works best with moderate motion (walking speed)

## Common Issues and Solutions

### Issue: Scale Drift

**Problem**: Monocular vision cannot determine absolute scale.

**Solution** (with automatic scale estimation):
- Ensure accelerometer data is being fed via `addAccelData()`
- Check `isInitialized()` returns true
- Use `getEstimatedScale()` to get current estimate
- For additional accuracy, occasionally fuse with GPS

### Issue: Low Tracking Quality

**Symptoms**: `tracking_quality < 0.3` or frequent `is_valid = false`

**Solutions**:
- Increase `max_features` (more features to track)
- Switch to `ORB` detector for rotation-invariant features
- Check lighting conditions (poor lighting reduces tracking)
- Reduce camera motion speed

### Issue: Rotation Drift

**Symptoms**: Orientation drifts over time

**Solution**: Make sure gyroscope data is being fed correctly via `addGyroData()`

## Threading Considerations

**Thread Safety:**
- VisionModule uses internal mutexes for gyro data buffer
- Multiple calls to `addGyroData()` from different threads are safe
- Do NOT call `processFrame()` from multiple threads simultaneously
- Do NOT call `processFrame()` and `reset()` simultaneously

**Best Practice:**
```cpp
// Create one VisionModule per processing thread
// Process frames sequentially on a single thread
// Add gyro data from sensor thread (safe)
```

## Current Implementation

The current implementation in `native-lib.cpp` shows how the Vision module is integrated:

1. **Thread Creation**: VisionModule is created when VIO thread starts
2. **Frame Processing**: Each camera frame is queued and processed by VisionModule
3. **Gyro Fusion**: Gyro data is passed directly to VisionModule
4. **Pose Integration**: The output rotation/translation are integrated into global pose
5. **Reset Handling**: Reset clears both global pose and VisionModule state

See `vio_thread_loop()` in `native-lib.cpp` for reference.

## Next Steps for Student 2

1. **Understand the output**: Make sure you understand what `rotation` and `translation` represent
2. **Test integration**: Try adjusting `scale_` to match real-world measurements
3. **Add filtering**: Consider adding a Kalman filter to smooth the pose estimates
4. **Implement map building**: Use the pose estimates to build a local map
5. **Add GPS fusion**: Fuse VIO output with GPS for absolute positioning

## Questions or Issues?

If you encounter any issues or have questions about the Vision module:

1. Check the logs with tag "VisionModule" for detailed information
2. Verify `is_valid` before using output
3. Check `tracking_quality` to assess reliability
4. Review statistics to understand long-term performance

## API Summary

| Function | Purpose | Thread-Safe |
|----------|---------|-------------|
| `processFrame()` | Process camera frame, get motion | No (single thread only) |
| `addGyroData()` | Add gyroscope data for fusion | Yes |
| `addAccelData()` | Add accelerometer data for scale & init | Yes |
| `reset()` | Reset all state | No (don't call during processFrame) |
| `getStatistics()` | Get performance metrics | Yes |
| `getEstimatedScale()` | Get auto-estimated scale factor | Yes |
| `isInitialized()` | Check if gravity-initialized | Yes |
| `setFeatureDetectorType()` | Change detector | No |
| `setMaxFeatures()` | Adjust feature count | No |
| `setGyroFusionWeight()` | Adjust sensor fusion | No |

---

**Document Version**: 2.0
**Last Updated**: 2025-11-16
**Author**: Student 1 (Vision & Sensor Fusion Module)
**For**: Student 2 (Navigation Core & Path Estimation)

## Changelog

### Version 2.0 (2025-11-16)
- Added automatic scale estimation using accelerometer
- Added gravity-based initialization
- Improved outlier rejection with stricter RANSAC parameters
- Added motion degeneracy detection
- Added inlier count validation
- Updated documentation with new features
