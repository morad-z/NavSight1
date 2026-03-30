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

# OpenCV: use Android SDK if cross-compiling, desktop find_package otherwise
if(ANDROID)
    set(OPENCV_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../OpenCV-android-sdk/sdk/native")
    set(OpenCV_DIR "${OPENCV_SDK_DIR}/jni")
    find_package(OpenCV REQUIRED)
else()
    find_package(OpenCV REQUIRED)
endif()

# VIO core library (no Android deps)
set(VIO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../app/src/main/cpp")
add_library(navsight_core STATIC
    ${VIO_SRC_DIR}/VisionModule.cpp
    ${VIO_SRC_DIR}/IMUPreintegrator.cpp
)
target_compile_definitions(navsight_core PUBLIC NAVSIGHT_DESKTOP_TEST=1)
target_include_directories(navsight_core PUBLIC ${VIO_SRC_DIR})
target_link_libraries(navsight_core ${OpenCV_LIBS})

# Test executable
add_executable(navsight_tests
    test_imu_preintegrator.cpp
    test_vision_module.cpp
    test_drift_scenarios.cpp
)
target_link_libraries(navsight_tests navsight_core GTest::gtest_main)

if(NOT ANDROID)
    include(GoogleTest)
    gtest_discover_tests(navsight_tests)
endif()
