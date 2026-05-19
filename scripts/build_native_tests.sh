#!/usr/bin/env bash
#
# build_native_tests.sh — Cross-compile tests/cpp/ for arm64-v8a via the
# Android NDK toolchain, producing the navsight_tests executable that
# scripts/run_tests_device.sh pushes to the phone.
#
# OpenCV is pulled in statically from the vendored
# OpenCV-android-sdk/sdk/native/staticlibs/arm64-v8a/ so the resulting
# binary needs only libc++_shared.so on-device (NDK-provided), not
# libopencv_java4.so.
#
# Usage: ./scripts/build_native_tests.sh [Release|Debug]
#
set -euo pipefail

# Repo root — script lives in repo/scripts/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_TYPE="${1:-Release}"
ABI="arm64-v8a"
PLATFORM="android-24"

# Locate the NDK. Prefer ANDROID_NDK env, otherwise the same NDK gradle uses.
NDK="${ANDROID_NDK:-C:/Users/morad/AppData/Local/Android/Sdk/ndk/27.0.12077973}"
if [ ! -d "$NDK" ]; then
    echo "ERROR: NDK not found at $NDK" >&2
    echo "Set ANDROID_NDK to override." >&2
    exit 1
fi

TOOLCHAIN="$NDK/build/cmake/android.toolchain.cmake"
if [ ! -f "$TOOLCHAIN" ]; then
    echo "ERROR: toolchain file missing: $TOOLCHAIN" >&2
    exit 1
fi

# Android Studio ships cmake + ninja; reuse them so we don't depend on a
# system cmake.
ANDROID_CMAKE_DIR="C:/Users/morad/AppData/Local/Android/Sdk/cmake/3.22.1/bin"
CMAKE_BIN="${CMAKE_BIN:-$ANDROID_CMAKE_DIR/cmake.exe}"
NINJA_BIN="${NINJA_BIN:-$ANDROID_CMAKE_DIR/ninja.exe}"

if [ ! -x "$CMAKE_BIN" ] && ! command -v cmake >/dev/null 2>&1; then
    echo "ERROR: no cmake found (looked for $CMAKE_BIN and PATH cmake)" >&2
    exit 1
fi
if [ ! -x "$CMAKE_BIN" ]; then
    CMAKE_BIN="$(command -v cmake)"
fi

SRC_DIR="$REPO_ROOT/tests/cpp"
BUILD_DIR="$SRC_DIR/build_android"

echo "[build_native_tests] repo:        $REPO_ROOT"
echo "[build_native_tests] NDK:         $NDK"
echo "[build_native_tests] cmake:       $CMAKE_BIN"
echo "[build_native_tests] ninja:       $NINJA_BIN"
echo "[build_native_tests] ABI:         $ABI"
echo "[build_native_tests] platform:    $PLATFORM"
echo "[build_native_tests] build type:  $BUILD_TYPE"

# Clean prior build_android. Fresh configure each invocation; cmake's
# FetchContent caches under _deps/ so this is cheap after the first run.
echo "[build_native_tests] cleaning $BUILD_DIR"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Configure. ANDROID_STL=c++_shared matches the gradle build (see
# app/build.gradle.kts) so DBoW2 + OpenCV's static libs link against the
# same C++ ABI the device's libc++_shared.so provides.
echo "[build_native_tests] configuring..."
"$CMAKE_BIN" \
    -S "$SRC_DIR" \
    -B "$BUILD_DIR" \
    -G Ninja \
    -DCMAKE_MAKE_PROGRAM="$NINJA_BIN" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DANDROID_ABI="$ABI" \
    -DANDROID_PLATFORM="$PLATFORM" \
    -DANDROID_STL=c++_shared \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# Build navsight_tests (and replay_harness, which is a sibling target).
echo "[build_native_tests] compiling..."
"$CMAKE_BIN" --build "$BUILD_DIR" --target navsight_tests -j

BIN="$BUILD_DIR/navsight_tests"
if [ ! -f "$BIN" ]; then
    echo "ERROR: build succeeded but $BIN missing" >&2
    exit 1
fi

echo ""
echo "[build_native_tests] OK"
echo "  binary:  $BIN"
echo "  size:    $(stat -c%s "$BIN" 2>/dev/null || stat -f%z "$BIN" 2>/dev/null) bytes"
echo ""
echo "Run on the phone with: scripts/run_tests_device.sh"
