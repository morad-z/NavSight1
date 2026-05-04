#!/usr/bin/env bash
# Capture adb logcat for an UNTETHERED sim recording so we can verify
# Step 4 (ORB relocalization), Step 5 (BLUR / LOWLIGHT / ROT_GATE), and
# Step 3a/3b (MSCKF + SLAM) actually fired and how they performed.
#
# The phone does NOT need to stay connected during the walk. logcat runs
# on the phone itself (writing to internal storage); it survives USB
# disconnect because we detach it with `nohup ... &` from the adb shell.
#
# Two-phase workflow:
#
#   1. Connect phone via USB.
#      ./scripts/record_with_logcat.sh start
#      → clears the on-phone logcat buffer
#      → spawns a detached `logcat -f /sdcard/...` on the phone
#      → script exits; you can disconnect now
#
#   2. Disconnect, walk around, record sim on phone, stop sim. Reconnect.
#      ./scripts/record_with_logcat.sh stop
#      → kills the on-phone logcat process
#      → pulls the captured logcat into tests/sims/
#      → finds the newest simulation_data_<ts>.json modified since the
#        capture started, renames the logcat to match its <ts>
#      → prints a one-line RELOC / BLUR / LOWLIGHT / ROT_GATE verdict
#
# The on-phone logcat file lives at:
#   /sdcard/Download/navsight_logcat.txt
# A small marker file at /sdcard/Download/navsight_logcat.start_epoch_s
# records the wall-clock start so the host can match it to the sim mtime.

set -euo pipefail

ADB="/c/Users/morad/AppData/Local/Android/Sdk/platform-tools/adb.exe"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SIMS_DIR="$REPO/tests/sims"
# /data/local/tmp is the shell's writable home on Android — unlike
# /sdcard/Download which Samsung One UI's scoped storage maps to an app
# uid (file ends up unreadable by the shell pulling it back).
PHONE_LOGCAT="/data/local/tmp/navsight_logcat.txt"
PHONE_MARKER="/data/local/tmp/navsight_logcat.start_epoch_s"

MODE="${1:-}"
if [ "$MODE" != "start" ] && [ "$MODE" != "stop" ]; then
    echo "Usage: $(basename "$0") {start|stop}" >&2
    echo >&2
    echo "  start   tether phone, kick off on-phone logcat, then disconnect" >&2
    echo "  stop    reconnect phone, pull logcat, pair with newest sim" >&2
    exit 2
fi

if ! "$ADB" devices | grep -qE "device$"; then
    echo "ERROR: no adb device found. Plug the phone in and run 'adb devices'." >&2
    exit 1
fi

# Tags we care about. The app uses NavSight-<Module> tag names (set via
# `#define TAG "NavSight-..."` in each .cpp). RELOC_ORB / BLUR / LOWLIGHT
# / ROT_GATE / PERF are MESSAGE-BODY prefixes inside those tags, NOT
# tags themselves — verdict greps further down filter on those.
# Narrow tag filter so a 10 min walk fits in single-digit MB instead of
# the 100+ MB an unfiltered Samsung logcat would produce.
TAG_FILTER=(
    "NavSight-VioEngine:I"
    "NavSight-Tracker:I"
    "NavSight-TrackKLT:I"
    "NavSight-EKF:I"
    "NavSight-Features:I"
    "NavSight-MSCKF:I"
    "NavSight-ZUPT:I"
    "NavSight-Native:I"
    "NavSight-Lens:I"
    "NavSight-Init:I"
    "NavSight-CamCalib:I"
    "*:S"
)

if [ "$MODE" = "start" ]; then
    echo "Clearing phone logcat buffer..."
    "$ADB" logcat -c

    # Kill any prior on-phone capture so we don't end up with stacked logcats.
    "$ADB" shell "pkill -f 'logcat.*navsight_logcat.txt'" 2>/dev/null || true
    "$ADB" shell "rm -f $PHONE_LOGCAT $PHONE_MARKER" 2>/dev/null || true

    START_EPOCH_S=$(date +%s)
    echo "$START_EPOCH_S" | "$ADB" shell "cat > $PHONE_MARKER"

    # Spawn the on-phone logcat as a detached background process. nohup +
    # & keeps it alive after the adb shell session closes (USB disconnect).
    # We use 'sh -c' so the shell-level redirect happens on the phone.
    "$ADB" shell "nohup logcat -v time ${TAG_FILTER[*]} -f $PHONE_LOGCAT > /dev/null 2>&1 &"
    sleep 1  # give the logcat process a moment to actually start

    # Verify it's running.
    if "$ADB" shell "pgrep -f 'logcat.*navsight_logcat.txt'" >/dev/null 2>&1; then
        echo
        echo "================================================================"
        echo "  On-phone logcat running. PID(s) on device:"
        "$ADB" shell "pgrep -f 'logcat.*navsight_logcat.txt'" | sed 's/^/    /'
        echo "  Writing to: $PHONE_LOGCAT (on phone)"
        echo "  Start epoch: $START_EPOCH_S"
        echo "  ----------------------------------------------------------------"
        echo "  YOU CAN DISCONNECT NOW. Walk around, record the sim, stop it."
        echo "  Reconnect afterward and run:"
        echo "      ./scripts/record_with_logcat.sh stop"
        echo "================================================================"
    else
        echo "ERROR: on-phone logcat did not start. Check phone-side permissions." >&2
        exit 1
    fi
    exit 0
fi

# MODE == stop
if ! "$ADB" shell "test -f $PHONE_MARKER" 2>/dev/null; then
    echo "ERROR: no on-phone capture marker found. Did you run 'start' first?" >&2
    exit 1
fi

START_EPOCH_S=$("$ADB" shell "cat $PHONE_MARKER" | tr -d '\r\n ')
echo "Capture started at epoch $START_EPOCH_S ($(date -d @$START_EPOCH_S 2>/dev/null || echo 'unknown date'))"

# Stop the on-phone logcat so the file is closed cleanly before we pull.
"$ADB" shell "pkill -f 'logcat.*navsight_logcat.txt'" 2>/dev/null || true
sleep 1

# Pull the file into a temp location, then rename based on the matching sim.
TMP_LOG="$(mktemp -t navsight_logcat.XXXXXX.txt)"
"$ADB" pull "$PHONE_LOGCAT" "$TMP_LOG" >/dev/null 2>&1 || {
    echo "ERROR: failed to pull $PHONE_LOGCAT from phone." >&2
    rm -f "$TMP_LOG"
    exit 1
}

# Clean up phone-side artefacts.
"$ADB" shell "rm -f $PHONE_LOGCAT $PHONE_MARKER" 2>/dev/null || true

# Find newest sim file modified since capture started.
NEWEST_SIM=""
NEWEST_MTIME=0
for f in "$SIMS_DIR"/simulation_data_*.json; do
    [ -e "$f" ] || continue
    MTIME=$(stat -c %Y "$f" 2>/dev/null || stat -f %m "$f" 2>/dev/null || echo 0)
    if [ "$MTIME" -gt "$NEWEST_MTIME" ] && [ "$MTIME" -ge "$START_EPOCH_S" ]; then
        NEWEST_MTIME=$MTIME
        NEWEST_SIM=$f
    fi
done

if [ -z "$NEWEST_SIM" ]; then
    FALLBACK_TS=$(date +%s)000
    OUT_PATH="$SIMS_DIR/logcat_${FALLBACK_TS}_unmatched.txt"
    mv "$TMP_LOG" "$OUT_PATH"
    echo
    echo "WARNING: no new simulation_data_*.json found in $SIMS_DIR since"
    echo "         capture started ($START_EPOCH_S). Logcat saved unmatched at:"
    echo "         $OUT_PATH"
    echo "         If the sim file lands later, rename to logcat_<ts>.txt to pair it."
    exit 0
fi

SIM_BASENAME=$(basename "$NEWEST_SIM")
SIM_TS=$(echo "$SIM_BASENAME" | sed -E 's/simulation_data_([0-9]+)\.json/\1/')
OUT_PATH="$SIMS_DIR/logcat_${SIM_TS}.txt"
mv "$TMP_LOG" "$OUT_PATH"

LOG_LINES=$(wc -l < "$OUT_PATH")
echo
echo "Paired with sim:  $SIM_BASENAME"
echo "Logcat saved to:  $(basename "$OUT_PATH")  ($LOG_LINES lines)"
echo

# Quick verdict.
RELOC_ACCEPTS=$(grep -c "RELOC_ORB: kf=" "$OUT_PATH" || true)
RELOC_REJECTS=$(grep -c "RELOC_ORB: no keyframe accepted" "$OUT_PATH" || true)
RELOC_SIZE_SKIP=$(grep -c "RELOC_ORB: id/pts size mismatch" "$OUT_PATH" || true)
BLUR_ENTERS=$(grep -c "BLUR: enter" "$OUT_PATH" || true)
LOWLIGHT_HITS=$(grep -c "LOWLIGHT:" "$OUT_PATH" || true)
ROT_GATE_HITS=$(grep -c "ROT_GATE:" "$OUT_PATH" || true)
MSCKF_HITS=$(grep -cE "MSCKF (Huber|update applied)" "$OUT_PATH" || true)
PERF_SLOW=$(grep -cE "PERF: section=applyMSCKFUpdate" "$OUT_PATH" || true)

echo "── Step 4 / 5 verdict ──"
echo "  RELOC_ORB accepts          : $RELOC_ACCEPTS"
echo "  RELOC_ORB rejects          : $RELOC_REJECTS"
echo "  RELOC_ORB skipped (size)   : $RELOC_SIZE_SKIP"
echo "  BLUR enter events          : $BLUR_ENTERS"
echo "  LOWLIGHT log lines         : $LOWLIGHT_HITS"
echo "  ROT_GATE log lines         : $ROT_GATE_HITS"
echo "  MSCKF update lines         : $MSCKF_HITS"
echo "  PERF: slow MSCKF (>500us)  : $PERF_SLOW"

if [ "$RELOC_ACCEPTS" -gt 0 ]; then
    echo
    echo "  RELOC fired and accepted at least one keyframe. Sample:"
    grep -m 1 "RELOC_ORB: kf=" "$OUT_PATH" | sed 's/^/    /'
elif [ "$RELOC_REJECTS" -gt 0 ]; then
    echo
    echo "  RELOC TRIGGERED but no keyframe survived the >=30-inlier gate. Sample:"
    grep -m 1 "RELOC_ORB: no keyframe accepted" "$OUT_PATH" | sed 's/^/    /'
else
    echo
    echo "  RELOC did NOT fire. Either KLT stayed healthy throughout, or the"
    echo "  occlusion was less than 3 consecutive low-inlier frames."
fi
