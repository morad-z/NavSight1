cmake_minimum_required(VERSION 3.22.1)

project("navsight")

# Set the path to your OpenCV for Android SDK (relative path for cross-platform compatibility)
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../OpenCV-android-sdk/sdk/native/jni")

# Find the OpenCV package
find_package(OpenCV REQUIRED)

# Add our native library
# This command compiles native-lib.cpp into a shared library named libnavsight.so
add_library(
        navsight
        SHARED
        src/main/cpp/native-lib.cpp
        src/main/cpp/VisionModule.cpp
        src/main/cpp/IMUPreintegrator.cpp)

# Link our library against the Android log library and the OpenCV library
target_link_libraries(
        navsight
        android
        log
        ${OpenCV_LIBS})