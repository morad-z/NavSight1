# NavSight VIO Test Build
# Supports desktop (find_package) and Android NDK cross-compilation.
#
# Desktop: mkdir build && cd build && cmake .. && make && ./navsight_tests
# Android: see run_tests_device.sh
cmake_minimum_required(VERSION 3.14)
project(navsight_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Google Test via FetchContent
include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

# Step 7 — nlohmann/json for the replay harness sim-file parser
FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# OpenCV: use Android SDK if cross-compiling, desktop find_package otherwise
if(ANDROID)
    set(OPENCV_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../OpenCV-android-sdk/sdk/native")
    set(OpenCV_DIR "${OPENCV_SDK_DIR}/jni")
    find_package(OpenCV REQUIRED)
else()
    find_package(OpenCV REQUIRED)
endif()

# VIO core library (all active source files)
set(VIO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../app/src/main/cpp")
add_library(navsight_core STATIC
    ${VIO_SRC_DIR}/Tracker.cpp
    ${VIO_SRC_DIR}/EKFState.cpp
    ${VIO_SRC_DIR}/IMUPreintegrator.cpp
    ${VIO_SRC_DIR}/FeatureManager.cpp
    ${VIO_SRC_DIR}/LensCorrector.cpp
    ${VIO_SRC_DIR}/CameraCalibration.cpp
    ${VIO_SRC_DIR}/TrackKLT.cpp
    ${VIO_SRC_DIR}/UpdaterZeroVelocity.cpp
    ${VIO_SRC_DIR}/UpdaterMSCKF.cpp
    ${VIO_SRC_DIR}/InertialInitializer.cpp
    ${VIO_SRC_DIR}/VioEngine.cpp
    ${VIO_SRC_DIR}/ScaleEstimatorVI.cpp
    ${VIO_SRC_DIR}/ScaleFuser.cpp
)
target_compile_definitions(navsight_core PUBLIC NAVSIGHT_DESKTOP_TEST=1)
# MSVC's <cmath> doesn't expose M_PI by default; the production build uses
# Clang/GCC where it's available as a POSIX extension. Define it for desktop
# tests on MSVC so the shared source compiles unchanged.
if(MSVC)
    target_compile_definitions(navsight_core PUBLIC _USE_MATH_DEFINES)
endif()
target_include_directories(navsight_core PUBLIC ${VIO_SRC_DIR})
target_link_libraries(navsight_core ${OpenCV_LIBS})

# Test executable
# NOTE: test_imu_preintegrator.cpp, test_drift_scenarios.cpp, test_vision_module.cpp
# reference old APIs (VisionModule, setGravity, getMotionMode) and need updating.
add_executable(navsight_tests
    test_tracker.cpp
    test_zupt.cpp
    test_ekf_state.cpp
    test_inertial_initializer.cpp
    test_relative_rotation.cpp
    # Plan Step 3b (ADR-009): SLAM-features-in-state EKF API.
    # EKFState::{addSlamFeature, removeSlamFeature, updateSlamFeature,
    # applyMSCKFFeature, getSlamFeatureCovIdx} are now landed.
    test_slam_msckf.cpp
    # Plan Step 4: ORB descriptors at keyframes + KLT-loss relocalization.
    # Exercises the OpenCV primitives and parameter tuning the FeatureManager
    # impl uses (ORB nfeatures=250, BF NORM_HAMMING + Lowe 0.75, findEssentialMat
    # RANSAC 1.5 px @ focal=525). Self-contained: no FeatureManager dependency.
    test_orb_reloc.cpp
    # Plan Step 5: adaptive front-end robustness — motion-blur detection,
    # adaptive KLT window from gyro, dual-gate (Rayleigh + gyro floor)
    # pure-rotation detector, and low-light feature policy. Self-contained:
    # exercises the same OpenCV primitives + formulas the Tracker /
    # FeatureManager impls use, no Tracker / FeatureManager dependency.
    test_visual_robustness.cpp
)
target_link_libraries(navsight_tests navsight_core GTest::gtest_main)

if(NOT ANDROID)
    include(GoogleTest)
    gtest_discover_tests(navsight_tests)
endif()

# Step 7 — Replay harness.
# Reads a recorded simulation JSON, drives the VIO pipeline IMU-only at the
# recorded timestamps with synthetic black YUV frames to trigger EKF
# propagation, and emits a CSV trace of pose + horizontal-plane covariance.
# A separate Python scorer (replay_scorer.py) consumes the CSV.
add_executable(replay_harness replay_harness.cpp)
target_link_libraries(replay_harness PRIVATE navsight_core nlohmann_json::nlohmann_json)
