# NavSight VIO Test Suite

## Test Structure

```
tests/
  cpp/                          # C++ Google Test (desktop build)
    CMakeLists.txt              # Desktop CMake build (requires OpenCV)
    test_utils.h                # Synthetic frame/IMU generators
    test_imu_preintegrator.cpp  # IMU preintegration unit tests
    test_vision_module.cpp      # VisionModule unit tests
    test_drift_scenarios.cpp    # Drift detection scenario tests
  android/                      # (reserved for future Android-specific tests)
  README.md                     # This file

app/src/
  test/.../VioDataTest.kt       # JVM unit tests (no device needed)
  androidTest/.../VioNativeTest.kt  # On-device integration tests (JNI)
```

## Running Tests

### JVM Unit Tests (fastest, no device)
```bash
./gradlew app:testDebugUnitTest
```

### On-Device Integration Tests (requires device/emulator)
```bash
./gradlew app:connectedDebugAndroidTest
```
Or in Android Studio: right-click `VioNativeTest.kt` → Run

### C++ Desktop Tests (requires desktop OpenCV + CMake)
```bash
cd tests/cpp
mkdir build && cd build
cmake ..
make
./navsight_tests
```

## Test Scenarios

| Test | Type | What it validates |
|------|------|-------------------|
| Static scene (3s) | Drift | ZUPT prevents position drift when still |
| Pure rotation 360° | Drift | Rotation doesn't cause position drift |
| Forward motion | Integration | Position grows during translation |
| Scale convergence | Scale | smooth_scale_ converges during walk |
| Feature quality | Tracking | High features → high quality |
| Empty frame | Robustness | No crash on featureless frames |
| Timestamp gap | Robustness | Handles 10s gaps gracefully |
| Reset cycles | Memory | No leaks after rapid start/stop |
| Heading fusion | Math | VIO yaw + init azimuth = correct heading |
| Distance accumulation | Math | Total distance vs displacement |

## Drift Thresholds (Industry Standards)

| Metric | Threshold | Source |
|--------|-----------|--------|
| Static drift | < 0.01m/60s | OpenVINS benchmark |
| Rotation position drift | < 0.1m/360° | TUM-VI evaluation |
| Scale error | < 20% | Monocular VIO typical |
| Loop closure error | < 10% path length | Without explicit loop closure |
| ATE (absolute trajectory) | < 5% distance | Consumer VIO acceptable |
