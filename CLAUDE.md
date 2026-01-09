# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NavSight1 is a production-ready Visual-Inertial Odometry (VIO) navigation application for Android that tracks device position and orientation in real-time using computer vision and IMU sensor fusion. The app is designed for mounting on vehicles (e.g., scooters) to provide GPS-free navigation by analyzing ground-facing camera footage combined with accelerometer/gyroscope data.

**Key Achievement**: Full VIO pipeline with multi-threaded processing and gyroscope-visual sensor fusion (commit 94b0b92).

## Architecture

The application uses a **multi-threaded hybrid architecture** with three main layers:

### 1. UI Layer (Kotlin/Jetpack Compose)
- **MainActivity.kt**: Main entry point, sensor management, JNI bridge
- **Compose UI**: Split-screen camera/map view with real-time overlays
- **State Management**: Reactive mutable state for VIO data and location

### 2. JNI Bridge Layer
- **Threading**: Asynchronous camera frame and sensor data passing
- **Thread Safety**: Mutex-protected queues for cross-thread communication
- **JVM Management**: Proper thread attachment/detachment for callbacks

### 3. VIO Engine (C++/OpenCV) - Multi-threaded
- **Dedicated VIO Thread**: Separate processing thread with condition variables
- **Thread-Safe Queues**: Frame queue, gyroscope queue, accelerometer queue
- **Sensor Fusion**: 98% gyroscope + 2% visual odometry blending
- **Feature Tracking**: Lucas-Kanade optical flow with 200 feature points

### Data Flow Architecture
```
Camera Frames → Thread-Safe Queue → VIO Processing Thread
                                    ↓
IMU Sensors → Gyro/Accel Queues → Sensor Fusion (98:2 ratio)
                                    ↓
                        Essential Matrix + Pose Recovery
                                    ↓
                        Global Pose Update (R, t)
                                    ↓
                    Mutex-Protected State → JNI Return
                                    ↓
                    MainActivity Callback → UI Recomposition
```

## Build System

**Gradle**: 8.13 with Kotlin DSL
**NDK/CMake**: C++17 with multi-threading support
**OpenCV**: 4.5.3.0 via Quickbird Studios Prefab package

### Common Build Commands

```bash
# Build debug APK
./gradlew assembleDebug

# Build and install on connected device
./gradlew installDebug

# Build release APK
./gradlew assembleRelease

# Rebuild native C++ code only (after native changes)
./gradlew :app:externalNativeBuild

# Clean and rebuild everything
./gradlew clean assembleDebug

# Run unit tests
./gradlew test

# Run instrumented tests
./gradlew connectedAndroidTest
```

### Running and Debugging

```bash
# Launch app after installation
adb shell am start -n com.example.navsight1/.MainActivity

# View VIO-specific logs
adb logcat | grep -E "NavSight-Native"

# View all app logs
adb logcat | grep -E "(navsight|VIO|MainActivity)"

# Monitor sensor data flow
adb logcat | grep -E "(Gyro|Accel|Features)"
```

## Native Development

### C++ Source Structure
- **Main VIO engine**: `app/src/main/cpp/native-lib.cpp` (348 lines)
- **Build config**: `app/CMakeLists.txt`
- **Threading**: `std::thread`, `std::mutex`, `std::condition_variable`, `std::queue`

### Multi-Threading Architecture

#### VIO Processing Thread (`vio_thread_loop`)
Runs independently, processing queued camera frames:
1. Wait on condition variable for new frames
2. Pop frame from thread-safe queue
3. Process frame through VIO pipeline
4. Update mutex-protected global state
5. Repeat until stopped

#### Thread Synchronization
```cpp
std::thread vio_thread;                    // Dedicated VIO processing thread
std::mutex vio_mutex;                      // Protects global pose state
std::condition_variable vio_cv;            // Signals new frame availability
bool running;                              // Thread control flag

// Thread-safe queues
std::queue<CamFrame> frame_queue;          // Camera frames
std::queue<GyroReading> gyro_queue;        // Gyroscope readings
std::queue<cv::Point3f> accel_queue;       // Accelerometer readings
```

### VIO Pipeline (native-lib.cpp)

#### 1. Frame Preprocessing (Lines 100-109)
- Convert YUV 420 NV21 to grayscale using `cv::cvtColor`
- First frame: Detect 200 corner features with `goodFeaturesToTrack`
- Parameters: quality=0.01, minDistance=10px

#### 2. Gyroscope Integration (Lines 112-131)
- Consume all queued gyro readings since last frame
- Compute rotation increment via Rodrigues formula: `ω * dt → R`
- Build cumulative rotation matrix: `delta_R_from_gyro`

#### 3. Optical Flow Tracking (Lines 133-154)
- Track features with `cv::calcOpticalFlowPyrLK` (Lucas-Kanade pyramid)
- Filter by status flag (valid tracks only)
- Prepare normalized points for UI visualization overlay

#### 4. Essential Matrix & Pose Estimation (Lines 156-188)
- Compute camera intrinsics (simplified): `K = [focal, 0, cx; 0, focal, cy; 0, 0, 1]`
- Find essential matrix: `cv::findEssentialMat` with RANSAC (confidence=0.999)
- Recover pose: `cv::recoverPose` → rotation R, translation t

#### 5. Sensor Fusion (Lines 166-172)
**Critical Feature**: Blends gyroscope and visual odometry rotations
```cpp
const double alpha = 0.98;  // Fusion weight
cv::Mat rot_vec_fused = alpha * rot_vec_gyro + (1.0 - alpha) * rot_vec_vo;
```
- **98% gyroscope**: High-frequency, short-term accuracy
- **2% visual odometry**: Long-term drift correction
- Result: Stable rotation estimate resistant to camera tracking failures

#### 6. Global Pose Update (Lines 175-178)
```cpp
global_t = global_t + (g_scale * (global_R * t));  // Translation
global_R = R_fused * global_R;                     // Rotation
```
- Concatenates incremental pose to global transformation
- Applies user-adjustable scale factor (g_scale)

#### 7. Feature Management (Lines 191-195)
- Maintain minimum 100 tracked features
- Auto-replenish when count drops below threshold
- Maximum: 200 features per frame

### JNI Interface

All native methods in MainActivity.kt:

| Method | Purpose | Threading |
|--------|---------|-----------|
| `startVIO()` | Launch VIO processing thread | Creates `std::thread` |
| `stopVIO()` | Stop VIO thread gracefully | Joins thread, clears queues |
| `processCameraFrame()` | Enqueue frame, return latest pose | Non-blocking, queues frame |
| `processGyroscope()` | Buffer gyro reading | Thread-safe queue push |
| `processAccelerometer()` | Buffer accel reading (for reset) | Thread-safe queue push |
| `resetVIO()` | Reset pose to origin, compute initial orientation | Uses accel buffer for gravity |
| `setScale()` | Adjust translation scale factor | Mutex-protected write |
| `stringFromJNI()` | Get native version string | Returns "NATIVE CODE VERSION 2" |
| `pingNative()` | Debug connectivity test | Logs to Android logcat |

### CMake Configuration

```cmake
cmake_minimum_required(VERSION 3.22.1)
project("navsight")

# OpenCV SDK path (Windows absolute path)
set(OpenCV_DIR "C:/Users/morad/AndroidStudioProjects/NavSight1/OpenCV-android-sdk/sdk/native/jni")
find_package(OpenCV REQUIRED)

# Build shared library from native-lib.cpp
add_library(navsight SHARED src/main/cpp/native-lib.cpp)

# Link OpenCV and Android log library
target_link_libraries(navsight android log ${OpenCV_LIBS})
```

**Important**: OpenCV path is Windows-specific. Update for other platforms.

## Key Source Files

| File | Lines | Purpose |
|------|-------|---------|
| `MainActivity.kt` | ~339 | Entry point, sensor management, JNI bridge, Compose UI |
| `VioData.kt` | 39 | Data class: position (x,y,z), orientation (roll,pitch,yaw), tracked points |
| `native-lib.cpp` | 348 | Multi-threaded VIO engine with sensor fusion |
| `CMakeLists.txt` | 23 | Native build configuration, OpenCV linking |
| `app/build.gradle.kts` | 94 | Android build config, dependencies, native settings |
| `guides.md` | 196 | Comprehensive 5-phase implementation guide |

## Dependencies

### Android/Kotlin
- **Jetpack Compose + Material3**: Modern declarative UI
- **Google Maps**: `com.google.maps.android:maps-compose:2.11.4`
- **Play Services Maps**: `com.google.android.gms:play-services-maps:18.2.0`
- **Play Services Location**: `com.google.android.gms:play-services-location:21.0.1`
- **CameraView**: `com.otaliastudios:cameraview:2.7.2` (third-party camera wrapper)
- **Accompanist Permissions**: `com.google.accompanist:accompanist-permissions:0.32.0`

### Native
- **OpenCV**: `4.5.3` Android SDK with Prefab
- **C++ Standard**: C++17 (enabled in build.gradle.kts)
- **Threading**: `<thread>`, `<mutex>`, `<condition_variable>`, `<queue>`

## UI Components (Jetpack Compose)

### NavSightApp (Root Composable)
- Permission handling: CAMERA, ACCESS_FINE_LOCATION
- Triggers GPS acquisition on permission grant
- Calls `requestInitialLocation()` to set map origin

### MainScreen (Split-Screen Layout)
```
┌─────────────────────────────────────┐
│  JNI Version    VIO Position (X,Y,Z)│  ← Overlay text
├─────────────────────────────────────┤
│                                     │
│        CameraView (50%)             │  ← Camera feed + feature overlay
│                                     │
├─────────────────────────────────────┤
│                                     │
│        Google Maps (50%)            │  ← Real-time position marker
│                                     │
├─────────────────────────────────────┤
│  [Reset VIO]   Scale: ━━━━●━━━━    │  ← Controls
└─────────────────────────────────────┘
```

### CameraView Component
- **Format**: YUV_420_888 (NV21 in native code)
- **Frame Processor**: Calls `processCameraFrame()` JNI on each frame
- **Lifecycle**: Properly bound to Compose lifecycle
- **Error Handling**: Logs camera errors to console

### MapView Component
- **Google Maps SDK**: Displays position marker in real-time
- **Coordinate Conversion**: Meters to Lat/Lng using spherical Earth model
  ```kotlin
  fun metersToLatLng(start: LatLng, dx: Double, dz: Double): LatLng {
      val latOffset = dz / 111139.0  // ~111139 meters per degree latitude
      val lngOffset = dx / (111139.0 * cos(start.latitude * PI / 180.0))
      return LatLng(start.latitude + latOffset, start.longitude + lngOffset)
  }
  ```
- **Marker Rotation**: Yaw angle displayed on marker orientation

### TrackedPointsOverlay
- **Canvas-based rendering**: Draws green circles for each tracked feature
- **Coordinates**: Pixel coordinates from native code (x, y pairs)
- **Real-time update**: Recomposes on every frame

## VioData Model

```kotlin
data class VioData(
    val x: Double = 0.0,           // Position X (meters)
    val y: Double = 0.0,           // Position Y (meters)
    val z: Double = 0.0,           // Position Z (meters)
    val roll: Double = 0.0,        // Roll angle (radians)
    val pitch: Double = 0.0,       // Pitch angle (radians)
    val yaw: Double = 0.0,         // Yaw angle (radians)
    val trackedPoints: FloatArray  // [x1,y1,x2,y2,...] pixel coords
)
```

## Critical Implementation Details

### 1. Timestamp Synchronization
Camera frames and IMU readings use `System.nanoTime()` (Kotlin) and are correlated in C++. Gyroscope readings between frames are integrated to compute rotation increments.

### 2. Scale Ambiguity Resolution
Monocular visual odometry cannot determine absolute scale. Current approach:
- **User-adjustable scale slider**: 0.0 to 10.0 (default: 1.0)
- Applied as multiplicative factor: `global_t += g_scale * (global_R * t)`
- **Future enhancement**: Auto-calibration using accelerometer double integration

### 3. Sensor Fusion Strategy
**Complementary Filter** approach:
- **Gyroscope (98%)**: High-frequency, short-term accuracy, no drift in rotation
- **Visual Odometry (2%)**: Long-term correction, handles gyro bias
- **Fusion point**: Rotation vectors blended before converting to matrix
- **Why 98:2?**: Gyro is highly accurate for short intervals; VO corrects long-term drift

### 4. Threading Model
**Producer-Consumer Pattern**:
- **Producers**: Camera callback (UI thread), Sensor callbacks (sensor thread)
- **Consumer**: VIO processing thread (dedicated)
- **Synchronization**: Condition variable wakes VIO thread on new frame
- **Benefits**: Non-blocking UI, consistent frame processing time

### 5. Feature Tracking Resilience
- **Minimum threshold**: 100 features (replenish if below)
- **Maximum capacity**: 200 features
- **Detection parameters**: Quality level 0.01, min distance 10px
- **Tracking**: Pyramid Lucas-Kanade (3 levels, 21×21 window)

### 6. Initial Orientation Calibration
On VIO reset (`resetVIO()`):
1. Average last 100 accelerometer readings
2. Normalize to gravity vector
3. Compute initial roll and pitch from gravity
4. Log initial orientation to console
5. Reset global pose to identity

### 7. Map Coordinate Conversion
VIO outputs in local Cartesian frame (meters). Conversion to GPS:
```
Δlat = z / 111139  (meters north → degrees latitude)
Δlng = x / (111139 * cos(start_lat))  (meters east → degrees longitude)
```
Assumes spherical Earth (good approximation for short distances <10km).

## Testing & Debugging

### Hardware Requirements
- **Camera**: Downward-facing, 30-60 FPS capable
- **Mounting**: Fixed height (0.5-1.5m), stable (no vibrations)
- **Surface**: Textured (asphalt, concrete, patterned floors)

### Test Environments
- ✅ **Good**: Asphalt roads, concrete with texture, patterned tiles
- ⚠️ **Moderate**: Indoor floors with some texture, grass (if short)
- ❌ **Poor**: Uniform surfaces (blank concrete), water, very dark/bright areas

### Performance Benchmarks
- **Feature detection**: ~5-15ms per frame (device-dependent)
- **Optical flow**: ~10-25ms per frame
- **Total VIO latency**: ~20-50ms per frame (30-60 FPS achievable)
- **Memory**: ~50-100MB (includes OpenCV libraries)

### Debug Logging
Native code logs extensively. Monitor with:
```bash
adb logcat | grep "NavSight-Native"
```

Key log messages:
- `"VIO Thread: Processing frame <timestamp>"` - Frame received
- `"VIO Thread: Features tracked: X / Y"` - Tracking success rate
- `"Essential Matrix found!"` - Pose estimation succeeded
- `"VIO Reset. Initial orientation: Roll=X, Pitch=Y"` - Reset confirmation

### Common Issues

**1. No features tracked**
- **Cause**: Uniform/textureless surface
- **Solution**: Move to textured surface, increase lighting

**2. Drift/incorrect scale**
- **Cause**: Monocular scale ambiguity
- **Solution**: Adjust scale slider based on known distance

**3. Jerky motion on map**
- **Cause**: Tracking failures, low feature count
- **Solution**: Check surface texture, verify feature count in logs

**4. Camera not starting**
- **Cause**: Permission denied, camera in use
- **Solution**: Grant permissions, close other camera apps

## Permissions (AndroidManifest.xml)

Required permissions:
- `CAMERA` - Camera frame access (critical)
- `ACCESS_FINE_LOCATION` - Initial GPS pin for map origin (critical)
- `INTERNET` - Google Maps tile loading (critical)
- `HIGH_SAMPLING_RATE_SENSORS` - Fast IMU access (recommended)
- `VIBRATE`, `WAKE_LOCK`, `RECORD_AUDIO` - Optional features

## Development Environment

- **Android SDK**: Min API 24, Target/Compile API 34
- **JDK**: Eclipse Adoptium JDK 21 (configured in `gradle.properties`)
- **NDK**: r26b or later (via Android Studio SDK Manager)
- **OpenCV**: Local SDK in `OpenCV-android-sdk/` directory
- **IDE**: Android Studio Hedgehog (2023.1.1) or later

## Known Limitations & Future Work

### Current Limitations
1. **No accelerometer fusion**: Accel data only used for initial orientation
2. **Hardcoded camera intrinsics**: Focal length = frame width (approximation)
3. **Scale ambiguity**: Requires manual adjustment via slider
4. **Single-camera VO**: No stereo vision (depth ambiguity)
5. **No loop closure**: Long-term drift inevitable (no SLAM)

### Planned Enhancements
1. **Full IMU fusion**: Kalman filter with accelerometer integration
2. **Camera calibration**: Per-device intrinsic parameters
3. **Auto-scale**: Use accelerometer double integration or known height
4. **Multi-threading optimization**: Separate threads for detection/tracking
5. **Map features**: Path visualization, waypoints, turn-by-turn
6. **Error recovery**: Automatic tracking failure detection and recovery

## Architecture Decision Records

### Why Multi-Threading?
Camera frame processing (30-60 FPS) blocks for 20-50ms. Running on UI thread causes dropped frames. Dedicated thread ensures:
- Non-blocking UI
- Consistent frame processing
- Better sensor synchronization

### Why 98:2 Fusion Ratio?
Gyroscope:
- ✅ Very accurate for short intervals (<1s)
- ✅ No visual ambiguity
- ❌ Long-term bias drift

Visual Odometry:
- ✅ No long-term drift in rotation
- ❌ Can fail on uniform surfaces
- ❌ Lower frequency than gyro

98:2 ratio leverages gyro's short-term accuracy while allowing VO to correct long-term drift.

### Why Thread-Safe Queues?
Camera/sensor callbacks run on different threads. Direct VIO processing in callbacks would:
- Block camera pipeline (dropped frames)
- Create race conditions in VIO state
- Complicate error handling

Queues decouple producers (sensors) from consumer (VIO thread).

## References

- **guides.md**: 5-phase implementation roadmap (196 lines)
- **OpenCV Docs**: https://docs.opencv.org/4.5.3/
- **CameraView Library**: https://github.com/natario1/CameraView
- **Google Maps Compose**: https://developers.google.com/maps/documentation/android-sdk/maps-compose

## Quick Reference: Common Tasks

### Modify VIO Parameters
Edit `native-lib.cpp`:
- Feature count: Line 104, 193 (`goodFeaturesToTrack` maxCorners)
- Fusion ratio: Line 166 (`alpha = 0.98`)
- Min feature threshold: Line 191 (`< 100`)

### Add New JNI Method
1. Declare in `MainActivity.kt`: `external fun methodName(...)`
2. Implement in `native-lib.cpp`: `Java_com_example_navsight1_MainActivity_methodName`
3. Rebuild: `./gradlew :app:externalNativeBuild`

### Change Camera Resolution
Edit `MainActivity.kt` CameraView component, add:
```kotlin
cameraView.setPreviewStreamSize(SizeSelectors.maxWidth(1280))
```

### Adjust Map Zoom
Edit `MainActivity.kt` MapView, modify:
```kotlin
val cameraPositionState = rememberCameraPositionState {
    position = CameraPosition.fromLatLngZoom(startLocation, 18f) // Increase for closer zoom
}
```

### Enable Advanced Logging
Add to `native-lib.cpp`:
```cpp
__android_log_print(ANDROID_LOG_DEBUG, TAG, "Your message: %f", value);
```

## Contact & Support

See `guides.md` for detailed implementation phases and mathematical background.
