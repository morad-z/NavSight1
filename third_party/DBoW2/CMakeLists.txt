cmake_minimum_required(VERSION 3.10)
project(DBoW2)

# NavSight: this is the vendored DBoW2 (https://github.com/dorian3d/DBoW2).
# Used as an add_subdirectory() target from app/CMakeLists.txt during the
# NDK build. Two changes vs upstream:
#   1) The demo target is gated off by default — pulls in image I/O paths
#      we don't need, and the demo binary is host-only anyway.
#   2) We do NOT call find_package(OpenCV) here; the parent project has
#      already done so and exposes OpenCV_LIBS / OpenCV_INCLUDE_DIRS via
#      the CMake variable scope. Calling find_package() a second time
#      under NDK r27 picks up wrong toolchain paths and breaks the build.
#   3) Install/configure_file rules removed — DBoW2 is consumed in-tree.
#   4) -Wno-unused-parameter / -Wno-sign-compare so NDK clang 18 doesn't
#      treat upstream's -Wall -Wextra -Wpedantic warnings as errors.

option(BUILD_DBoW2 "Build DBoW2"            ON)
option(BUILD_Demo  "Build DBoW2 demo target" OFF)

set(HDRS
  include/DBoW2/BowVector.h
  include/DBoW2/FBrief.h
  include/DBoW2/QueryResults.h
  include/DBoW2/TemplatedDatabase.h
  include/DBoW2/FORB.h
  include/DBoW2/DBoW2.h
  include/DBoW2/FClass.h
  include/DBoW2/FeatureVector.h
  include/DBoW2/ScoringObject.h
  include/DBoW2/TemplatedVocabulary.h
)

set(SRCS
  src/BowVector.cpp
  src/FBrief.cpp
  src/FORB.cpp
  src/FeatureVector.cpp
  src/QueryResults.cpp
  src/ScoringObject.cpp
)

if(BUILD_DBoW2)
  # Static lib: simpler ABI story under NDK and we don't share DBoW2
  # with any other consumer.
  add_library(${PROJECT_NAME} STATIC ${SRCS})

  target_include_directories(${PROJECT_NAME}
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/include/DBoW2
  )

  # OpenCV vars are inherited from the parent app/CMakeLists.txt
  # (find_package(OpenCV REQUIRED ...) is called there).
  if(DEFINED OpenCV_INCLUDE_DIRS)
    target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
  endif()
  if(DEFINED OpenCV_LIBS)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
  endif()

  set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
  )

  # Quiet upstream warnings under NDK r27 / clang 18.
  if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
      -Wno-unused-parameter
      -Wno-sign-compare
      -Wno-unused-variable
      -Wno-reorder
      -Wno-deprecated-declarations
    )
  endif()
endif()

if(BUILD_Demo)
  add_executable(demo demo/demo.cpp)
  target_link_libraries(demo ${PROJECT_NAME} ${OpenCV_LIBS})
  set_target_properties(demo PROPERTIES CXX_STANDARD 14)
endif()
