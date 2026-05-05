cmake_minimum_required(VERSION 3.22.1)

project("navsight")

# Optimization flags for Production VIO.
#
# WARNING: -ffast-math implies -ffinite-math-only, which constant-folds
# `std::isfinite(x)` to `true` for every call site (the compiler is told to
# assume NaN/Inf never appear). The codebase relies on isfinite checks in 7
# files (LoopClosureDetector.cpp, ScaleFuser.cpp, ScaleEstimatorVI.cpp,
# IMUPreintegrator.cpp, native-lib.cpp, Tracker.cpp, VioEngine.cpp) to filter
# bad triangulations and divergent state. Without -fno-finite-math-only the
# Step 7 PnP path silently received (0,0,0) "3D points" because its NaN
# guard never fired (debugged 2026-05-05 from LC_TRI_DBG: avail=0 despite
# rows being NaN-initialised at fill time). Keep -fno-finite-math-only on
# both compilers — removing it re-introduces the bug.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -fno-finite-math-only -ftree-vectorize")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3 -ffast-math -fno-finite-math-only -ftree-vectorize")

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

# ── Plan Step 7 / ADR-013: vendored DBoW2 for same-session loop closure ──
# Built as a static lib via add_subdirectory(); it inherits OpenCV from the
# CMake variable scope set by find_package() above. The vendored copy has
# its CMakeLists trimmed (no demo, no install rules, NDK-friendly warning
# flags) — see third_party/DBoW2/CMakeLists.txt for the diff vs upstream.
set(DBOW2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/DBoW2")
if(NOT EXISTS "${DBOW2_DIR}/include/DBoW2/DBoW2.h")
    message(FATAL_ERROR "Vendored DBoW2 not found at ${DBOW2_DIR}. "
                        "Loop closure (Plan Step 7) cannot build.")
endif()
# EXCLUDE_FROM_ALL keeps DBoW2's own targets out of the default Android
# build graph — `navsight` reaches in to link the static lib explicitly
# via target_link_libraries(... DBoW2) below.
add_subdirectory("${DBOW2_DIR}" "${CMAKE_BINARY_DIR}/DBoW2" EXCLUDE_FROM_ALL)

add_library(
        navsight
        SHARED
        src/main/cpp/native-lib.cpp
        src/main/cpp/VioEngine.cpp
        src/main/cpp/Tracker.cpp
        # DISABLED: Mapper pipeline — output was discarded (applyMapperResult was a no-op)
        # src/main/cpp/Mapper.cpp
        # src/main/cpp/PoseGraph.cpp
        src/main/cpp/LoopClosureDetector.cpp
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
        src/main/cpp/WindowedBA.cpp
)

target_include_directories(navsight PRIVATE
        "${DBOW2_DIR}/include"
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
        DBoW2
        ${OpenCV_LIBS}
)