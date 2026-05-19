#!/usr/bin/env bash
#
# run_tests_device.sh — Push tests/cpp/build_android/navsight_tests to the
# phone via adb and run it. OpenCV is statically linked into the binary
# (see scripts/build_native_tests.sh), so the only runtime dep is
# libc++_shared.so, which we push from the NDK alongside the binary.
#
# Build first if the binary is missing or stale:
#   ./scripts/build_native_tests.sh
#
# Picks the first attached device by default. Override with ADB_SERIAL=...
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ADB="${ADB:-/c/Users/morad/AppData/Local/Android/Sdk/platform-tools/adb.exe}"
NDK="${ANDROID_NDK:-C:/Users/morad/AppData/Local/Android/Sdk/ndk/27.0.12077973}"

BIN="$REPO_ROOT/tests/cpp/build_android/navsight_tests"
if [ ! -f "$BIN" ]; then
    echo "ERROR: $BIN not found." >&2
    echo "       Run scripts/build_native_tests.sh first." >&2
    exit 1
fi

# libc++_shared.so for android-24 / arm64-v8a. The binary is dynamically
# linked against it (DT_NEEDED libc++_shared.so) so we have to push it
# alongside and prepend LD_LIBRARY_PATH.
LIBCXX="$NDK/toolchains/llvm/prebuilt/windows-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so"
if [ ! -f "$LIBCXX" ]; then
    echo "ERROR: libc++_shared.so not found at $LIBCXX" >&2
    exit 1
fi

# Resolve the device. Both USB and Wi-Fi adb show the same physical phone
# as separate entries; pick the first non-offline one. Honour ADB_SERIAL.
DEVICE_FLAG=()
if [ -n "${ADB_SERIAL:-}" ]; then
    DEVICE_FLAG=(-s "$ADB_SERIAL")
    echo "[run_tests_device] using ADB_SERIAL=$ADB_SERIAL"
else
    SERIAL="$("$ADB" devices | awk 'NR>1 && $2=="device" {print $1; exit}')"
    if [ -z "$SERIAL" ]; then
        echo "ERROR: no adb device in 'device' state. Run: $ADB devices" >&2
        exit 1
    fi
    DEVICE_FLAG=(-s "$SERIAL")
    echo "[run_tests_device] selected device: $SERIAL"
fi

# Choose a writable + executable tmp dir. /data/local/tmp is the standard
# location for shell-side ad-hoc binaries and is exec-allowed.
# Disable MSYS / Git-Bash path translation. Without this, /data/local/tmp/
# gets rewritten to C:/Program Files/Git/data/local/tmp/ before adb sees it,
# and the device push fails with "remote secure_mkdirs failed". Disabling
# also breaks /c/Users/... → C:/Users/... rewrites for host paths, so we
# convert host paths to native Windows form via cygpath up front.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL='*'

# Convert host-side absolute paths to Windows form. cygpath -w turns
# /c/Users/morad/... into C:\Users\morad\..., which adb on Windows accepts.
to_winpath() {
    if command -v cygpath >/dev/null 2>&1; then
        cygpath -w "$1"
    else
        # Fallback: turn /<letter>/... into <letter>:/...
        echo "$1" | sed -E 's|^/([a-zA-Z])/|\1:/|'
    fi
}

BIN_WIN="$(to_winpath "$BIN")"
LIBCXX_WIN="$(to_winpath "$LIBCXX")"

DEV_DIR="/data/local/tmp/navsight"
DEV_BIN="$DEV_DIR/navsight_tests"
DEV_LIB="$DEV_DIR/libc++_shared.so"

echo "[run_tests_device] prepping $DEV_DIR on device..."
"$ADB" "${DEVICE_FLAG[@]}" shell "rm -rf $DEV_DIR && mkdir -p $DEV_DIR"

echo "[run_tests_device] pushing libc++_shared.so..."
"$ADB" "${DEVICE_FLAG[@]}" push "$LIBCXX_WIN" "$DEV_LIB" >/dev/null

echo "[run_tests_device] pushing navsight_tests ($(stat -c%s "$BIN" 2>/dev/null || stat -f%z "$BIN" 2>/dev/null) bytes)..."
"$ADB" "${DEVICE_FLAG[@]}" push "$BIN_WIN" "$DEV_BIN" >/dev/null
"$ADB" "${DEVICE_FLAG[@]}" shell "chmod 755 $DEV_BIN"

# Stage ORBvoc if present so test_loop_closure can load BoW. Skip silently
# otherwise — the affected tests GTEST_SKIP() at runtime when the file is
# missing (test_loop_closure.cpp:46-48).
for VOC_NAME in ORBvoc.bin ORBvoc.dbow2 ORBvoc.txt; do
    VOC_HOST="$REPO_ROOT/app/src/main/assets/$VOC_NAME"
    if [ -f "$VOC_HOST" ]; then
        echo "[run_tests_device] staging $VOC_NAME..."
        "$ADB" "${DEVICE_FLAG[@]}" push "$(to_winpath "$VOC_HOST")" "$DEV_DIR/$VOC_NAME" >/dev/null
    fi
done

echo ""
echo "[run_tests_device] running navsight_tests..."
echo "================================================================"

# We tee through a host-side log so the user keeps the full output even if
# adb's pipe gets choppy on Wi-Fi. The cd ensures any test that drops a
# CWD-relative file lands in DEV_DIR (also where ORBvoc.* are staged).
LOG="$REPO_ROOT/tests/cpp/build_android/last_run.log"
mkdir -p "$(dirname "$LOG")"

set +e
"$ADB" "${DEVICE_FLAG[@]}" shell \
    "cd $DEV_DIR && LD_LIBRARY_PATH=$DEV_DIR ./navsight_tests --gtest_color=yes" \
    | tee "$LOG"
RC="${PIPESTATUS[0]}"
set -e

echo "================================================================"
echo ""

# Summarise. GoogleTest's "[  PASSED  ]" / "[  FAILED  ]" totals appear at
# the very end. Pull them from the saved log so the user has the bottom
# line even if scrollback is long. Strip ANSI colour codes first since
# --gtest_color=yes leaves escape sequences embedded in the file.
strip_ansi() { sed 's/\x1b\[[0-9;]*m//g'; }
PASSED="$(strip_ansi < "$LOG" | grep -E '^\[  PASSED  \] [0-9]+ tests?' | tail -1 || true)"
FAILED="$(strip_ansi < "$LOG" | grep -E '^\[  FAILED  \] [0-9]+ tests?' | tail -1 || true)"
SKIPPED="$(strip_ansi < "$LOG" | grep -E '^\[  SKIPPED \] [0-9]+ tests?' | tail -1 || true)"

echo "[run_tests_device] summary"
[ -n "$PASSED" ]  && echo "  $PASSED"
[ -n "$SKIPPED" ] && echo "  $SKIPPED"
[ -n "$FAILED" ]  && echo "  $FAILED"

if [ "$RC" -ne 0 ]; then
    echo ""
    echo "[run_tests_device] FAILED (exit=$RC). Failing tests:"
    strip_ansi < "$LOG" | grep -E '^\[  FAILED  \] [A-Za-z]' | sed 's/^/  /'
fi

# Leave the binary on-device for fast re-runs; uncomment to clean up:
# "$ADB" "${DEVICE_FLAG[@]}" shell "rm -rf $DEV_DIR"

exit "$RC"
