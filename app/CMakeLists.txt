cmake_minimum_required(VERSION 3.22.1)

project("navsight")

# Optimization flags for Production VIO
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -ftree-vectorize")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math -ftree-vectorize")

# Resolve OpenCV SDK path (with fallback for relative and absolute paths)
if(DEFINED OpenCV_DIR_OVERRIDE)
    set(OpenCV_DIR "${OpenCV_DIR_OVERRIDE}")
else()
    # Primary: OpenCV subdirectory relative to project root
    set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../OpenCV-android-sdk/sdk/native/jni")
    if(NOT EXISTS "${OpenCV_DIR}")
        message(WARNING "OpenCV directory not found at: ${OpenCV_DIR}")
        message(STATUS "Attempting fallback resolution...")
    endif()
endif()

message(STATUS "Using OpenCV_DIR: ${OpenCV_DIR}")

find_package(OpenCV REQUIRED COMPONENTS core calib3d features2d imgproc video)

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()

add_library(
        navsight
        SHARED
        src/main/cpp/native-lib.cpp
        src/main/cpp/VioEngine.cpp
        src/main/cpp/Tracker.cpp
        # DISABLED: Mapper pipeline — output was discarded (applyMapperResult was a no-op)
        # src/main/cpp/Mapper.cpp
        # src/main/cpp/LoopClosureDetector.cpp
        # src/main/cpp/PoseGraph.cpp
        src/main/cpp/IMUPreintegrator.cpp
        src/main/cpp/EKFState.cpp
        src/main/cpp/FeatureManager.cpp
        src/main/cpp/LensCorrector.cpp
        src/main/cpp/CameraCalibration.cpp
        src/main/cpp/TrackKLT.cpp
        src/main/cpp/UpdaterZeroVelocity.cpp
        src/main/cpp/UpdaterMSCKF.cpp
        src/main/cpp/InertialInitializer.cpp
        src/main/cpp/ScaleEstimatorVI.cpp
        src/main/cpp/ScaleFuser.cpp
)

find_library(
        log-lib
        log
)

# שים לב לסדר הקישור: לפעמים סדר הלינקינג משנה בסטטי.
# שמנו את OpenCV בסוף כדי שהוא ימצא את הסמלים שהוא צריך ב-android/log.
target_link_libraries(
        navsight
        android
        c++_shared
        ${log-lib}
        ${OpenCV_LIBS}
)