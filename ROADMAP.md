# NavSight Fix Roadmap

**Goal:** Connect the C++ VIO pipeline (VisionModule + IMUPreintegrator) to the app and make navigation work end-to-end.

**Root cause summary:** The project was built by multiple students. Student 1 built the C++ engine (`VisionModule.cpp`, `IMUPreintegrator.cpp`). Student 3 built the Kotlin UI. The integration (JNI bridge + MainActivity wiring) was never completed. The app currently runs a simple Kotlin block-matching fallback (`OpticalFlowProcessor`) that has no metric scale and no IMU fusion.

---

## Phase 1 — Wire the C++ engine into the JNI bridge
**File:** `app/src/main/cpp/native-lib.cpp`
**What:** Make `native-lib.cpp` actually call `VisionModule`. Right now it stores zeros and returns them.

- [x] 1.1 Add `#include "VisionModule.h"` and declare a static `navsight::VisionModule* g_vision = nullptr`
- [x] 1.2 `startVIO()` — create the VisionModule instance (`new navsight::VisionModule()`)
- [x] 1.3 `stopVIO()` — delete the instance and null the pointer
- [x] 1.4 `processCameraFrame()` — call `g_vision->processFrame(data, w, h, ts)`, then update `g_x/g_y/g_z/g_roll/g_pitch/g_yaw/g_points/g_quality/g_tracked` from the returned `VisionOutput`
- [x] 1.5 `processGyroscope()` — call `g_vision->addGyroData({ts, x, y, z})`
- [x] 1.6 `processAccelerometer()` — call `g_vision->addAccelData({ts, x, y, z})`
- [x] 1.7 `resetVIO()` — call `g_vision->reset()` before clearing globals
- [x] 1.8 `setScale()` — add `std::lock_guard<std::mutex> lock(state_mutex)` (data race fix)
- [x] 1.9 Update global pose accumulation — `processCameraFrame` must accumulate incremental R/t into a running global pose, not just overwrite with the per-frame delta

**Test:** `./gradlew :app:externalNativeBuildDebug` must compile clean. Then: `adb logcat | grep "VisionModule"` should show feature detection logs on first run.

---

## Phase 2 — Declare JNI methods in Kotlin and call them
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**What:** Add `external fun` declarations and wire them into the camera/sensor callbacks. Currently there are zero JNI calls from Kotlin.

- [x] 2.1 Add `companion object` with `System.loadLibrary("navsight")` and all `external fun` declarations
- [x] 2.2 Call `startVIO()` in `onResume()` and `stopVIO()` in `onPause()`
- [x] 2.3 In `onSensorChanged`: call `processGyroscope` for `TYPE_GYROSCOPE` and `processAccelerometer` for `TYPE_ACCELEROMETER`
- [x] 2.4 In `processCameraFrame(frame: Frame)`: call the JNI `processCameraFrame` and store the returned `VioData`
- [x] 2.5 Expose `VioData` as `vioState: mutableStateOf(VioData())` so the UI can observe it
- [x] 2.6 Replace `virtualX`/`virtualZ` updates with `vio.x` and `vio.z` from JNI (Kotlin OpticalFlow kept for AR direction overlay only)
- [x] 2.7 Updated debug info box: shows x/z position, yaw, tracking quality %, feature count, and VIO init status

**Test:** `adb logcat | grep "NavSight-Native"` should show pose values changing as device moves. Map marker should drift from start position.

---

## Phase 3 — Fix critical bugs in VisionModule
**File:** `app/src/main/cpp/VisionModule.cpp`
**What:** Fix logic bugs that will cause the pipeline to silently reject valid data.

- [ ] 3.1 Fix `checkMotionDegeneracy` — the `motion_variance < 0.1f` check incorrectly rejects straight-line driving (uniform motion is NOT degenerate). Remove the variance check; keep only the `avg_motion < 1.0f` (no motion) check.
- [ ] 3.2 Fix duplicate scale estimation — `estimateScaleFromAccel()` at line ~305 overwrites the scale already computed by IMU preintegration at line ~271. Either remove the `estimateScaleFromAccel` call or only call it when preintegration produced zero measurements.
- [ ] 3.3 Fix static `gyro_count` / `accel_count` debug counters — change from `static int` locals to class member variables so `reset()` can clear them and the log throttle works correctly after reset.
- [ ] 3.4 Replace `gyro_buffer_.erase(gyro_buffer_.begin())` and `accel_buffer_.erase(accel_buffer_.begin())` with `std::deque` to make buffer management O(1) instead of O(n).

**Test:** Logcat should show `"Essential Matrix found!"` and `"Pose estimated"` messages without constant `"Degenerate motion detected"` spam during normal movement.

---

## Phase 4 — Connect scale slider UI
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**What:** Add a scale control so the user can tune position scaling at runtime (important because VIO scale is approximate).

- [ ] 4.1 Add a `var scaleValue by remember { mutableStateOf(1.0) }` state in `MainScreen`
- [ ] 4.2 Add a `Slider` (range 0.1–5.0) to the bottom controls area
- [ ] 4.3 On slider change call `setScale(scaleValue)`
- [ ] 4.4 Display current scale value next to the slider

---

## Phase 5 — Cleanup & polish
**What:** Remove dead code, fix permissions, update docs.

- [ ] 5.1 Remove `RECORD_AUDIO` from `AndroidManifest.xml` — CameraView already has `setAudio(Audio.OFF)`, this permission is unused and will alarm users
- [ ] 5.2 Keep `OpticalFlowProcessor` as a fallback only — move to a separate file and only use it when `VioData.isInitialized == false`; or remove it entirely if C++ pipeline works reliably
- [ ] 5.3 Update `CLAUDE.md` at root and `Navsight/NavSight1/CLAUDE.md` to reflect the actual architecture after wiring is complete
- [ ] 5.4 Replace `pathHistory.removeAt(0)` with `ArrayDeque` to avoid O(n) list shifts on every frame once history is full
- [ ] 5.5 Add null guard in `onSensorChanged` for missing magnetometer — on devices without one, azimuth stays 0° permanently without warning

---

## Quick reference — key files

| File | Role |
|------|------|
| `cpp/native-lib.cpp` | JNI bridge — Phase 1 work goes here |
| `cpp/VisionModule.cpp` | C++ VIO engine — Phase 3 bug fixes go here |
| `cpp/IMUPreintegrator.cpp` | IMU integration — no changes needed |
| `cpp/VisionModule.h` | API contract between engine and bridge |
| `MainActivity.kt` | Kotlin entry point — Phase 2 & 4 work goes here |
| `VioData.kt` | Data model passed from C++ → Kotlin (no changes needed) |
| `OpticalFlowProcessor.kt` | Kotlin fallback — to be demoted in Phase 5 |

## Build & test commands

```bash
# From Navsight/NavSight1/
./gradlew :app:externalNativeBuildDebug   # after any C++ change
./gradlew assembleDebug                   # full build
./gradlew installDebug                    # install on device

adb logcat | grep "VisionModule"          # C++ pipeline logs
adb logcat | grep "NavSight-Native"       # JNI bridge logs
adb logcat | grep "IMUPreintegrator"      # IMU logs
adb logcat | grep -E "(Gyro|Accel|Features)"
```
