#!/usr/bin/env bash
# Capture adb logcat alongside the next on-device sim recording so we can
# verify Step 4 (ORB relocalization) and Step 3a/3b (MSCKF + SLAM) actually
# fired and how they performed. Without this, the simulator JSON gives us
# inlier/quality traces but no per-event signal — and the logcat buffer
# rolls over within a few minutes of the recording ending, evicting
# RELOC_ORB / MSCKF / SLAM lines before we can read them.
#
# Usage:
#   ./scripts/record_with_logcat.sh
#
# Workflow:
#   1. Script clears the logcat buffer and starts streaming filtered
#      NavSight tags to a temp file in the background.
#   2. Run the simulator on the phone (record + walk + stop).
#   3. Pull the resulting simulation_data_<ts>.json into tests/sims/
#      (the existing manual workflow — this script does NOT pull, to
#      avoid clobbering whatever pull mechanism you already use).
#   4. Press Enter here to stop the logcat stream.
#   5. Script auto-detects the newest tests/sims/simulation_data_*.json
#      modified during the capture window and renames the logcat to
#      tests/sims/logcat_<ts>.txt with the SAME <ts> as the sim file.
#   6. Script prints a one-line RELOC_ORB summary so you know
#      immediately whether reloc fired.

set -euo pipefail

ADB="/c/Users/morad/AppData/Local/Android/Sdk/platform-tools/adb.exe"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SIMS_DIR="$REPO/tests/sims"
TMP_LOG="$(mktemp -t navsight_logcat.XXXXXX.txt)"

if ! "$ADB" devices | grep -qE "device$"; then
    echo "ERROR: no adb device found. Run 'adb devices' to check." >&2
    exit 1
fi

# Tags we care about. Add new tags here when new instrumentation lands.
# We deliberately keep this LIST narrow rather than capturing everything —
# full logcat for a 30 s recording can hit 50+ MB on a phone with chatty
# system services.
TAG_FILTER=(
    "VioEngine:I"
    "Tracker:I"
    "EKFState:I"
    "FeatureManager:I"
    "MSCKF:I"
    "PERF:I"
    "RELOC_ORB:I"
    "SLAM:I"
    "ScaleFuser:I"
    "*:S"  # silence everything else
)

echo "Clearing logcat buffer..."
"$ADB" logcat -c

START_EPOCH_S=$(date +%s)
echo "Starting logcat capture → $TMP_LOG"
"$ADB" logcat -v time "${TAG_FILTER[@]}" > "$TMP_LOG" &
LOGCAT_PID=$!
trap 'kill $LOGCAT_PID 2>/dev/null || true; rm -f "$TMP_LOG"' EXIT

echo
echo "================================================================"
echo "  Recording. Run the sim on your phone now."
echo "  (start sim → walk → cover camera if testing reloc → stop sim)"
echo "  Then drop the simulation_data_*.json into tests/sims/"
echo "  Press [Enter] HERE when done to stop the logcat capture."
echo "================================================================"
read -r _

# Stop logcat. Disable trap rm so we can keep the file.
kill $LOGCAT_PID 2>/dev/null || true
wait $LOGCAT_PID 2>/dev/null || true
trap - EXIT

# Find the newest sim file modified after capture started. mtime in
# epoch seconds via stat (-c %Y on GNU, -f %m on BSD; Git Bash is GNU).
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
    echo "         capture started. Logcat saved unmatched at:"
    echo "         $OUT_PATH"
    echo "         If the sim file lands later, rename to logcat_<ts>.txt"
    echo "         using the sim's embedded timestamp."
    exit 0
fi

# Extract embedded timestamp from sim filename: simulation_data_<ts>.json
SIM_BASENAME=$(basename "$NEWEST_SIM")
SIM_TS=$(echo "$SIM_BASENAME" | sed -E 's/simulation_data_([0-9]+)\.json/\1/')
OUT_PATH="$SIMS_DIR/logcat_${SIM_TS}.txt"
mv "$TMP_LOG" "$OUT_PATH"

echo
echo "Paired with sim:  $SIM_BASENAME"
echo "Logcat saved to:  $(basename "$OUT_PATH")"
echo

# Quick verdict on Step 4 events.
RELOC_ACCEPTS=$(grep -c "RELOC_ORB: kf=" "$OUT_PATH" || true)
RELOC_REJECTS=$(grep -c "RELOC_ORB: no keyframe accepted" "$OUT_PATH" || true)
RELOC_SIZE_SKIP=$(grep -c "RELOC_ORB: id/pts size mismatch" "$OUT_PATH" || true)
MSCKF_HITS=$(grep -cE "MSCKF (Huber|update applied)" "$OUT_PATH" || true)
SLAM_PROMOTE=$(grep -cE "SLAM.*[Pp]romote" "$OUT_PATH" || true)
PERF_SLOW=$(grep -cE "PERF: section=applyMSCKFUpdate" "$OUT_PATH" || true)

echo "── Step 4 / 3a / 3b verdict ──"
echo "  RELOC_ORB accepts          : $RELOC_ACCEPTS"
echo "  RELOC_ORB rejects          : $RELOC_REJECTS"
echo "  RELOC_ORB skipped (size)   : $RELOC_SIZE_SKIP"
echo "  MSCKF update lines         : $MSCKF_HITS"
echo "  SLAM promote events        : $SLAM_PROMOTE"
echo "  PERF: slow MSCKF (>500us)  : $PERF_SLOW"

if [ "$RELOC_ACCEPTS" -gt 0 ]; then
    echo
    echo "  ✓ Reloc fired and accepted at least one keyframe."
    echo "    Sample line:"
    grep -m 1 "RELOC_ORB: kf=" "$OUT_PATH" | sed 's/^/    /'
elif [ "$RELOC_REJECTS" -gt 0 ]; then
    echo
    echo "  ! Reloc TRIGGERED but no keyframe survived the ≥30-inlier gate."
    echo "    Sample line:"
    grep -m 1 "RELOC_ORB: no keyframe accepted" "$OUT_PATH" | sed 's/^/    /'
else
    echo
    echo "  - Reloc did NOT fire during this recording."
    echo "    Either KLT stayed healthy throughout, or the camera was"
    echo "    occluded for less than 3 consecutive low-inlier frames."
fi
