---
name: "NavSight Android Specialist"
description: "Deep expertise on the NavSight Kotlin/Android side: NavSightViewModel state holders, SensorRepository sensor pipeline, NativeBridge JNI bridge, CameraX setup, SimulationFrameRecorder, MapScreenUi compose patterns, GPS, permissions, lifecycle. Use for Android UI bugs, sensor permission issues, ViewModel state management, Compose recomposition, JNI-related bugs, frame recorder issues, or reset / lifecycle bugs."
---

# NavSight Android Specialist

## Overview

NavSight's Android layer is the user-facing surface plus the sensor pipeline that feeds the C++ VIO core. This skill carries the architectural knowledge needed to fix UI bugs, sensor pipeline issues, JNI marshaling, the simulation recorder, and the lifecycle (especially the reset path that just got a Stage-3 fix).

## When to use

Trigger on any of:
- "Android UI bug", "Compose", "recomposition", "ViewModel state"
- "sensor permission", "GPS bug", "location not acquired"
- "JNI", "NativeBridge", "processCameraFrameDirect"
- "frame recorder", "SimulationFrameRecorder", "PNG drops"
- "reset button", "ANR on reset", "lifecycle"
- "CameraX", "ImageAnalysis", "preview"
- Kotlin work in `app/src/main/java/com/example/navsight1/`

## Architecture — Kotlin layer

```
MainActivity
  └─ NavSightApp (Compose entry)
       ├─ SplashScreen (3.2 s)
       ├─ PermissionScreen (CAMERA + ACCESS_FINE_LOCATION)
       └─ MainScreen
            ├─ NavSightViewModel (state holder, observes flows)
            │    ├─ SensorRepository (sensor + GPS + frame recorder)
            │    ├─ NavigationManager (out-of-scope per SDD)
            │    ├─ DepthEstimator (MiDaS TFLite, 1 Hz)
            │    └─ DeviceOrientationTracker (mag one-shot only)
            ├─ MapScreenUi (Google Maps + heading marker + path)
            ├─ CameraUi (CameraX preview + ImageAnalysis)
            ├─ DebugPanelUi (REC button, scale calibration)
            ├─ BottomSheetUi (Reset button is here, Rides tab)
            └─ CalibrationScreenUi (camera intrinsic chessboard)
```

The exhaustive companion docs:
- `docs/study/06_android_ui.md` — every Composable, every state holder, every reader/writer
- `docs/study/07_kotlin_support.md` — SensorRepository, DepthEstimator, RoadSnapper, GpxExporter, etc.
- `docs/study/05_vio_engine_jni.md` — the JNI surface from the C++ side

## NavSightViewModel — state model

The VM uses Compose `mutableStateOf` (with `private set`), NOT StateFlow, for UI-facing state. There are 21 observable properties (see `docs/study/06_android_ui.md` §2.1 for the full table).

Key UI state:
- `vioState: VioData` — populated by `handleVioUpdate` (throttled to 200 ms via `UI_UPDATE_THROTTLE_MS`)
- `virtualX, virtualZ` — copied from `vio.x, vio.z` for map plotting
- `pathHistory: List<PathPoint>` — capped at 500, indexed by `pathHistoryVersion` (Compose recomposition key — must bump when mutating)
- `startLocation: LatLng?` — from `SensorRepository.startLocation`. Once set, NEVER overwritten (`SensorRepository.kt:788`) — overwriting after VIO start would shift the entire path.
- `initStatus: SensorRepository.InitStatus` — drives the "Hold steady" AlertDialog modal. `WAIT_STATIONARY` / `WAIT_MOTION` / `READY` / `TIMEOUT_NEEDS_USER`.
- `snappedPosition: LatLng?` — output of `RoadSnapper.snapToRoad` (Google Roads API, 15 m soft-snap gate FR17)

Side-effect coroutines in `init` (lines 196-224): 8 `viewModelScope.launch` adapters that map upstream flows → Compose state.

## SensorRepository — sensor pipeline

| Source | Rate | Status |
|---|---|---|
| `TYPE_ACCELEROMETER` | `SENSOR_DELAY_GAME` (~50 Hz) | always on |
| `TYPE_GYROSCOPE` | `SENSOR_DELAY_GAME` | always on |
| `TYPE_MAGNETIC_FIELD` | `SENSOR_DELAY_GAME` | **unregistered after VIO init** (no-mag-during-tracking rule) |
| `fusedLocationClient` | 1000 ms / 500 ms min | callbacks on `Looper.getMainLooper()` |
| Camera frames | CameraX | forwarded to single-thread `NavSight-VIO` executor |
| Depth (MiDaS) | 1 Hz | `NavSight-Depth` thread |

**NOT subscribed**: barometer, linear-accel, uncalibrated gyro/accel, rotation vector. Confirmed by absence of `getDefaultSensor` calls.

The repositoryScope is `Dispatchers.Main + SupervisorJob()`. `stopSensors()` cancels it; `startSensors()` recreates it (idempotent). This is critical for the reset path.

## JNI bridge

`NativeBridge.kt` is a Kotlin object (singleton) wrapping ~22 `external` functions. Loaded via `System.loadLibrary("navsight")` in init.

Camera frame: `processCameraFrameDirect(yBuffer, uvBuffer, w, h, yStride, uvStride, uvPixelStride, ts, rsSkewNs)` — direct ByteBuffers, zero-copy. Returns `VioData` (28-arg constructor, signature `"(DDDDDDDIIDZ[FDDDDFFFFFFDIIDDIDD)V"` cached at `JNI_OnLoad`).

The state mutex pattern in JNI shims (`native-lib.cpp`):
```cpp
std::shared_ptr<VioEngine> vision;
{ std::lock_guard<std::mutex> lock(state_mutex); vision = g_vision; }
// heavy work outside lock
if (vision) vision->...;
```
shared_ptr ref-count keeps engine alive even if `stopVIO` clears `g_vision` mid-call. **Don't change this pattern without understanding the lifetime invariant.**

Z-up → Y-up swap happens at the JNI boundary at `native-lib.cpp:378-391`:
```cpp
g_x = output.t.at<double>(0);   // East
g_y = output.t.at<double>(2);   // Up — Z-up index 2 → Y exposed
g_z = output.t.at<double>(1);   // North
```

## CameraX setup

`CameraUi.kt:42` (`CameraViewComposable`):
- `PreviewView` with `implementationMode=PERFORMANCE`, `scaleType=FILL_CENTER`
- `ResolutionSelector` strict 4:3 + 640×480 with `FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER`
- `ImageAnalysis.Builder` with `STRATEGY_KEEP_ONLY_LATEST`, `OUTPUT_IMAGE_FORMAT_YUV_420_888`
- **Step 8c rolling-shutter skew**: Camera2Interop attaches a `CameraCaptureSession.CaptureCallback` reading `CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW`, forwarded to `viewModel.updateRollingShutterSkew(skew)`
- Analyzer body MUST gate on `viewModel.isSensorRepositoryActive()` and `image.close()` in finally — RejectedExecutionException / IllegalStateException catch defended

## Reset button — Stage 3 (2026-05-09)

**Problem we just hit**: pressing Reset hung the app (ANR). Cause: `NativeBridge.resetVIO()` joins the BA + loop-closure worker threads, and `SimulationFrameRecorder.stop()` awaits encoder termination up to 10 s. All synchronous on the UI thread.

**Fix**: `NavSightViewModel.resetAll()` does the heavy work on `Dispatchers.IO` via `viewModelScope.launch(Dispatchers.IO)`. UI-visible flags (path history, virtual position, recording state) flip on the main thread immediately so the dot/path clear instantly; the slow work happens behind that.

`SensorRepository.resetAll(hasLocationPermission)` mirrors `onPause → wipe state → onResume`:
1. `stopGpsUpdates()`, `setFrameRecorder(null)`, `stopSensors()`
2. Wipe StateFlows + flags + buffers + native EventCounters
3. `startSensors()`, `startGpsUpdates()`, `requestInitialLocation()`

**Wire-up**: `MapScreenUi.kt:263` `onResetClick = { viewModel.resetAll() }` (the "Rides" button in BottomSheetUi).

## SimulationFrameRecorder — Step 9 / ADR-014

Per-recording-session class that captures Y-plane frames as PNGs in a sibling `<sim_basename>.frames/` directory.

- Bounded queue (capacity 32) + single-thread executor (`NavSight-FrameRec` daemon)
- Drops when queue full (counter); reports stats on `stop()`
- Filename: `<wall_clock_ns>.png` where `wall_clock_ns = System.currentTimeMillis() * 1_000_000` (matches the JSON's `ts` field × 1e6 with a few-ms offset)
- Replay harness uses `--frame-match-tolerance-ms 50` to absorb the offset

The recorder is owned by `NavSightViewModel`, started/stopped from `toggleSimulationRecording`. The repository holds a `@Volatile var frameRecorder: SimulationFrameRecorder?`; the camera analyzer thread reads it lock-free per frame and calls `captureFrame` if active.

## Permissions

Manifest declares INTERNET, CAMERA, ACCESS_FINE_LOCATION, VIBRATE, WAKE_LOCK, HIGH_SAMPLING_RATE_SENSORS. RECORD_AUDIO is currently commented out per the `feedback_no_deletions.md` rule.

Runtime requested via Accompanist: only **CAMERA + ACCESS_FINE_LOCATION** (`MainActivity.kt:44-46`). `HIGH_SAMPLING_RATE_SENSORS` doesn't need runtime UX on Android 12+.

`RECORD_AUDIO` and the audio code path are intentionally inactive — don't re-enable without an explicit ask.

## Implementation Playbooks

Every playbook is a complete, no-shortcuts procedure. Split into named steps when a single shot is too big. Never skip a step. Never leave a `TODO` in shipped code.

### Playbook A — Add a new ViewModel state holder

**Step A1 — Declare the property.**
In `NavSightViewModel`, add `var newState by mutableStateOf<T>(default); private set`. Use Compose `mutableStateOf` (NOT StateFlow) for UI-facing state — every Composable reading it auto-recomposes.

**Step A2 — Wire its source.**
- If derived from a SensorRepository flow: add a `viewModelScope.launch { sensorRepository.x.collect { newState = it } }` in the `init` block (lines 196-224).
- If updated by VIO frames: add a write in `handleVioUpdate` (around line 226-334).
- If user-driven: add a public `fun setNewState(...)` setter that the UI calls.

**Step A3 — Document readers.**
Update `docs/study/06_android_ui.md` §2.1 table with the new property: type, init value, line numbers, writers, readers (which Composables observe it).

**Step A4 — Wire to reset.**
If the state is session-scoped (i.e. should reset on the Rides button), add a clearing line to `NavSightViewModel.resetAll()` BEFORE the `viewModelScope.launch(Dispatchers.IO)` block. Main-thread state clears immediately so the UI updates instantly.

**Step A5 — Verify recomposition.**
For mutable-collection-backed state (e.g. `pathHistory: MutableList`), expose via a version counter (`pathHistoryVersion: Int`) and have readers use `remember(version) { backingList.toList() }`. See `MapScreenUi.kt:44` for the canonical pattern.

### Playbook B — Wire a new sensor

**Step B1 — Pick the sensor type and rate.**
Reference: `SensorManager.SENSOR_DELAY_GAME` ≈ 50 Hz. Document why this rate is appropriate (cite expected signal bandwidth or downstream processing rate).

**Step B2 — Subscribe in `SensorRepository.startSensors`.**
Mirror the existing accel/gyro subscriptions (`SensorRepository.kt:136-146`). Use `sensorManager.registerListener(this, sensor, SENSOR_DELAY_GAME)`.

**Step B3 — Unsubscribe in `stopSensors`.**
Add `sensorManager.unregisterListener(this, sensor)` to `stopSensors` (`SensorRepository.kt:547+`). Idempotent.

**Step B4 — Handle in `onSensorChanged`.**
Add a `Sensor.TYPE_X ->` branch in the switch. Forward to native via `NativeBridge.processX(ts_ns, x, y, z)` if it feeds VIO; otherwise update a StateFlow.

**Step B5 — Add to `resetAll`.**
If session-scoped, the existing `stopSensors → startSensors` cycle handles it. If you maintain a per-sensor accumulator on the Kotlin side, clear it in `resetAll`'s wipe block before `startSensors`.

**Step B6 — Mind the magnetometer rule.**
Magnetometer is unregistered after VIO init (`SensorRepository.kt:743`) per the `feedback_no_magnetometer.md` rule. Don't add new mag-driven paths during tracking.

**Step B7 — Add an EventCounter (telemetry).**
Bump a counter in EventCounters.h on every accepted/rejected sensor sample so the sim's `event_summary` reflects the new pipeline.

### Playbook C — Fix or prevent an ANR on the UI thread

**Step C1 — Identify the blocking call.**
Trace from the UI handler down. Common culprits in NavSight:
- `NativeBridge.resetVIO()` — joins BA + LC worker threads (~1-3 s)
- `SimulationFrameRecorder.stop()` — `awaitTermination` up to 10 s
- `sensorManager.unregisterListener` followed by `registerListener` — synchronous
- `fusedLocationClient.lastLocation.await()` — network/system call

**Step C2 — Decide what must be on the main thread.**
- UI state writes → main thread (Compose `mutableStateOf` requires it)
- StateFlow value updates → any thread (atomic publish)
- Native calls → any thread (NavSight's JNI uses `state_mutex` correctly)

**Step C3 — Split into two phases.**
- Synchronous main-thread phase: clear visible state (path history, virtual position, recording flag), so the user sees an immediate response.
- Asynchronous worker-thread phase: `viewModelScope.launch(Dispatchers.IO) { /* heavy work */ }`.

**Step C4 — Detach long-running resources before stopping them.**
Example: clear `frameRecorder` reference (`sensorRepository.setFrameRecorder(null)`) BEFORE calling `recorder.stop()`. This way no new frames queue while the encoder is draining.

**Step C5 — Verify.**
Press the affected button repeatedly in quick succession. No ANR. Logcat shows the heavy work running on a non-main thread (look for `Dispatcher` in the thread name).

### Playbook D — Add a new JNI binding

**Step D1 — Define the C++ method.**
Add `extern "C"` JNIEXPORT signature in `native-lib.cpp` matching Kotlin's `external fun` shape. Use the established `state_mutex` pattern: copy `g_vision` shared_ptr under the lock, run heavy work outside.

**Step D2 — Define the Kotlin counterpart.**
Add `external fun newCall(...)` in `NativeBridge.kt`. Match the JNI signature exactly. Document the threading model (which thread is allowed to call) in a comment.

**Step D3 — Marshal types correctly.**
- Direct `ByteBuffer` for camera-grade buffers (zero-copy via `GetDirectBufferAddress`)
- `FloatArray` / `DoubleArray` for fixed-size vectors (use `GetFloatArrayElements` + `Release...(JNI_ABORT)` for read, `SetFloatArrayRegion` for write)
- `String` for paths (use `GetStringUTFChars` / `ReleaseStringUTFChars`)

**Step D4 — Cache `jclass` and `jmethodID` in `JNI_OnLoad`.**
If your call returns an object (e.g. `VioData`), cache the class as `NewGlobalRef` and the constructor `jmethodID`. Cleanup in `JNI_OnUnload`.

**Step D5 — Document in `docs/study/05_vio_engine_jni.md`.**
Add to the §8.3 JNI function table with C symbol, Kotlin counterpart, threading model.

**Step D6 — Test.**
Add a unit test if possible. Otherwise verify via a logcat line at both ends.

## Project guardrails (enforced)

### No magic numbers

UI thresholds need a defensible source. Examples in code:
- `MIN_FEATURES/2 = 40` for keyframe-collapse trigger (cited from MAX_FEATURES)
- `BLUR_VAR_THRESH = 80.0` from variance-of-Laplacian baseline
- VioStatusChip σ < 1.5 m for "ACTIVE" — derived from PnP accuracy floor
- Camera blocked: `vio.trackedFeatures < 5` ramping `> 30` consecutive

If a constant has no clear origin, dig until you find it or measure it. Don't guess.

### No shortcuts, no TODOs

If a step in a playbook can't be completed in this shot:
1. STOP at the step. Tell the user.
2. Do NOT leave `// TODO`, `FIXME`, `XXX`, or stubs in shipped code.
3. Do NOT skip ahead and "come back to it later."

### Comment, don't delete

Unused composables, dead Kotlin functions, retired permissions all stay in-tree as `/* … */` blocks with a `LEGACY:` note explaining what currently does the job. Examples:
- `SplashScreen.kt` body wrapped (the active splash is `SplashScreenUi.kt`)
- `ui/theme/{Color,Theme,Type}.kt` bodies wrapped (active theming is `NavSightTheme.kt` + `NavPalette`)
- `AROverlayRenderer.kt` body wrapped — entirely dead
- `RECORD_AUDIO` permission line in AndroidManifest commented

## Gotchas

- **`pathHistory` is a mutable ArrayList exposed as `List<PathPoint>`.** Consumers must use `pathHistoryVersion` as a recomposition key — see `MapScreenUi.kt:44`. Reading the list without the version key skips recomposition.
- **`vioState` is throttled at 200 ms.** Off-throttle code paths (in `handleVioUpdate`) ALSO run for unthrottled state (totalDistanceM, currentSpeedKmh, road snap).
- **`startLocation` is set-once.** Don't call `_startLocation.value = X` after the first GPS fix without explicit reset.
- **Magnetometer is unregistered post-init** (`SensorRepository.kt:743`). DeviceOrientationTracker still calls `getRotationMatrix(...)`; with mag stale, it returns `false` and the fallback path keeps stale smoothed values with `stabilityScore=0`. Don't try to "fix" by re-registering the mag.
- **No `BackHandler` is wired anywhere.** System Back exits the activity even with overlays open. Documented as a known UX issue.
- **Hebrew RTL strings in StatusBadgesUi.kt** (`L27, 28, 57, 58, 76, 93, 106`). Manifest sets `supportsRtl="true"`. Don't translate without checking layout.
- **`MaterialTheme(darkColorScheme())`** at `MainActivity.kt:28` is the active theme path. The `ui/theme/` Compose-template files are dead.

## References

- Code: `app/src/main/java/com/example/navsight1/`
  - `MainActivity.kt`, `NavSightViewModel.kt`, `SensorRepository.kt`, `NativeBridge.kt`, `VioData.kt`
  - UI: `MapScreenUi.kt`, `CameraUi.kt`, `BottomSheetUi.kt`, `DebugPanelUi.kt`, `CalibrationScreenUi.kt`, `StatusBadgesUi.kt`, `SearchBarUi.kt`, `NavInstructionBannerUi.kt`, `SplashScreenUi.kt`, `PermissionScreenUi.kt`, `NavSightTheme.kt`
  - Support: `DepthEstimator.kt`, `DeviceOrientationTracker.kt`, `RoadSnapper.kt`, `CalibrationStore.kt`, `CrashLogger.kt`, `NavSightUtils.kt`, `GpxExporter.kt`, `SimulationFrameRecorder.kt`, `NavigationManager.kt`
- Manifest: `app/src/main/AndroidManifest.xml`
- Studies: `docs/study/06_android_ui.md`, `07_kotlin_support.md`, `05_vio_engine_jni.md`
