# NavSight AI Handoff Protocol

<!-- FORMAT: YAML-like sections for AI parseability. Human-readable but structured. -->
<!-- RULE: Keep total file under 400 lines. Purge COMPLETED items older than 3 sessions. -->
<!-- RULE: Each session entry in CHANGELOG must have: agent, developer, date, branch. -->
<!-- RULE: Before editing, pull latest. After editing, commit immediately. -->

---

## META

```yaml
last_updated: "2026-04-09"
last_agent: "Claude-Opus-4-6"
last_developer: "morad"
branch: "morad"
base_branch: "master"
head_commit: "pending"
```

---

## REFERENCE POINTS

```yaml
# Known-good commits for reverting if something breaks
best_drift_result:
  commit: "10fb69a"
  description: "5.4% drift (1.2m on 22m out-and-back). Heading tracks 180° turns."
  date: "2026-04-08"
  notes: >
    Best on-device verified accuracy. SO(3) fix, accel bias, EKF gravity,
    gyro bias init, ZUPT covariance. All code is ACTIVE (no dead code cleanup yet).
    Revert here if dead code cleanup or map lag fixes cause regressions.
```

---

## DEVELOPERS

```yaml
morad:
  role: "IMU preintegration, sensor fusion, calibration, integration testing"
  branch: "morad"
  agent: "Claude-Opus-4-6"
  last_session: "2026-04-08"

tamir:
  role: "UI/UX, JNI bridge, camera pipeline, feature implementation"
  branch: "tamir-dev / tamir-v2"
  agent: "TBD"
  last_session: "unknown"

roey:
  role: "Security, API key management, infrastructure"
  branch: "feature/ui-redesign"
  agent: "TBD"
  last_session: "unknown"
```

---

## ARCHITECTURE (current — post-cleanup)

```yaml
# Multi-file architecture with Mapper pipeline DISABLED:
#
#   VioEngine (orchestrator — Tracker only, Mapper disabled)
#     ├── Tracker (fast path: optical flow, essential matrix, scalar heading, pose)
#     │   ├── TrackKLT (Lucas-Kanade optical flow + geometric verification)
#     │   ├── UpdaterZeroVelocity (chi-squared ZUPT detector — ACTIVE)
#     │   ├── EKFState (15-DOF error-state EKF with MSCKF sliding window)
#     │   ├── FeatureManager (grid features, sparse replenish, keyframe store/match)
#     │   └── LensCorrector (undistortion for matched points)
#     ├── IMUPreintegrator (SO(3) preintegration, step detection, filtered gravity)
#     └── [DISABLED] Mapper + LoopClosureDetector + PoseGraph (output was discarded)
#
# DISABLED PIPELINE (2026-04-08 session 0i):
#   Mapper thread, LoopClosureDetector, PoseGraph removed from CMakeLists.
#   applyMapperResult was already a no-op — corrections caused teleportation spikes.
#
# RE-ENABLED (2026-04-09 session 0j):
#   DepthEstimator (MiDaS TFLite) re-enabled at 1Hz in SensorRepository.
#   Feeds Tracker::applyDepthScaleConstraint() directly (bypasses disabled Mapper).
#   MiDaS depth → metric scale via camera height + pitch → blends into smooth_scale_.
#
# Camera pipeline: CameraX ImageAnalysis (zero-copy via GetDirectBufferAddress)
#   processCameraFrameDirect only (old ByteArray version commented out)
#
# Heading: scalar_heading_ with gravity-projected yaw rate (filtered LP accel)
# Scale: step detector → stride estimation → EKF updateScale
# Translation: camera t_vo direction + IMU step magnitude
```

---

## CODEBASE STATE

### Build Status

```yaml
android_build: "PASSES"  # Verified 2026-04-08 (post-cleanup)
native_cpp_build: "PASSES"  # Mapper/LoopClosure/PoseGraph removed from CMakeLists
kotlin_compile: "PASSES"
on_device_testing: "TESTED 2026-04-08 — app launches, no crash after cleanup"
```

---

## ACTIVE BUGS

```yaml
# BUG-012: IMPROVED (tracking quality oscillates — acceptable)
# BUG-013: FIXED_VERIFIED (heading stable when stationary)
# BUG-015: RESOLVED — Mapper thread disabled, no more data race
# BUG-016: still exists but mitigated (processCameraFrame ByteArray version commented out)
# BUG-017: RESOLVED — Mapper/LoopClosure disabled
# BUG-018: RESOLVED — Mapper thread disabled entirely

- id: "BUG-014"
  title: "Scale stuck at 0.12-0.20, never calibrates from steps"
  status: "OPEN"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  reported: "2026-04-08"
  description: >
    Scale oscillates 0.12-0.21 despite 67 steps. Position accuracy OK
    (visual odometry compensates). Step-based scale may not feed EKF correctly.

- id: "BUG-016"
  title: "use-after-free risk in native-lib processCameraFrameDirect"
  status: "OPEN"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/native-lib.cpp"
  description: >
    Raw pointer snapshot of g_vision used after releasing lock. If stopVIO()
    deletes it concurrently, use-after-free. Fix: shared_ptr<VioEngine>.
```

---

## PENDING WORK

```yaml
- id: "TASK-033"
  title: "Dead code cleanup + Mapper pipeline disable + map lag fix"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-08"
  notes: >
    (1) Commented out all dead code across 15+ Kotlin and C++ files.
    (2) Disabled Mapper thread, LoopClosureDetector, PoseGraph (output was discarded).
    (3) Disabled DepthEstimator (MiDaS TFLite) — fed disabled Mapper.
    (4) Fixed map lag: mutableStateListOf → plain list + version counter,
    throttled GoogleMap content to 1Hz, cached marker icons.
    (5) Fixed crash: BitmapDescriptorFactory used before Maps SDK init.
    Saves 2 threads + MiDaS GPU inference. Build passes, app launches clean.

- id: "TASK-028"
  title: "MiDaS monocular depth for absolute scale"
  status: "RE-ENABLED"
  owner: "morad"
  priority: "P2"
  notes: >
    Re-enabled 2026-04-09: DepthEstimator runs at 1Hz GPU, feeds Tracker
    applyDepthScaleConstraint() directly (bypasses disabled Mapper).
    Not yet verified firing in real walking simulations.

- id: "TASK-034"
  title: "Map UX overhaul: heading lock, recenter, smooth camera"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-09"
  notes: >
    4Hz camera animation with heading lock. Recenter FAB on user pan.
    Removed unused export arrow button. rememberUpdatedState for fresh values.

- id: "TASK-035"
  title: "Fix V-shape heading on 180° turns"
  status: "DONE (needs device test)"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-09"
  notes: >
    Root cause: keyframe heading correction used atan2(R[1,0],R[0,0]) which
    extracts camera Z-rotation, not yaw. Systematically under-counted heading
    during turns, removing ~30-40° of real heading via 30% correction.
    Fix: gate correction on gyro_norm < 0.3 rad/s (skip during turns).
    Also: magnetic declination correction, ZUPT heading unfreezing.

- id: "TASK-032"
  title: "Fix remaining QA scan issues"
  status: "PARTIAL"
  owner: "morad"
  priority: "P1"
  notes: >
    BUG-015: RESOLVED (Mapper disabled — no more data race).
    BUG-016: OPEN (use-after-free — needs shared_ptr).
    BUG-017: RESOLVED (loop closure disabled).
    BUG-018: RESOLVED (Mapper thread disabled).
```

---

## RECENT CHANGES (last 5 sessions)

```yaml
- session: "0j"
  date: "2026-04-09"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    MAP UX + HEADING FIX + MIDAS RE-ENABLE SESSION.
    (1) Map UX overhaul: 4Hz heading-locked camera, recenter FAB on gesture,
    removed unused export button, rememberUpdatedState for fresh coroutine values.
    (2) MiDaS depth re-enabled at 1Hz: feeds Tracker::applyDepthScaleConstraint()
    directly (metric depth from camera height + pitch → smooth_scale_ blend).
    (3) Magnetic declination correction (~5.5° Haifa) on initial heading.
    (4) ZUPT heading unfreezing: allow turn-in-place detection.
    (5) V-SHAPE BUG FIX (key finding): keyframe heading correction used
    atan2(R[1,0],R[0,0]) = camera Z-rotation, NOT yaw. This removed ~30-40°
    of real heading during 180° turns. Fix: gate on gyro_norm < 0.3 rad/s.
    (6) Synced libs.versions.toml + FeatureManager from master (were diverged).
  files_changed:
    - "app/src/main/java/.../MainActivity.kt (map UX: heading lock, recenter FAB)"
    - "app/src/main/java/.../SensorRepository.kt (MiDaS re-enable, declination)"
    - "app/src/main/cpp/Tracker.cpp (V-shape fix, depth constraint, ZUPT heading)"
    - "app/src/main/cpp/Tracker.h (depth members, filtered_yaw_rate)"
    - "app/src/main/cpp/VioEngine.cpp (setDepthMap forwarding)"
    - "app/src/main/cpp/IMUPreintegrator.cpp/h (getUserHeight uncommented)"
    - "app/src/main/cpp/FeatureManager.h/cpp (heading+position in Keyframe)"
    - "gradle/libs.versions.toml (synced from master)"

- session: "0i"
  date: "2026-04-08"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    CLEANUP SESSION: Dead code audit, Mapper pipeline disable, map lag fix.
    (1) Deep code review: identified all dead code across Kotlin + C++ layers.
    Commented out (not deleted) ~30 dead functions/classes across 15+ files.
    (2) Disabled Mapper pipeline: Mapper thread, LoopClosureDetector, PoseGraph
    removed from CMakeLists + VioEngine. applyMapperResult was already a no-op.
    (3) Disabled DepthEstimator: MiDaS TFLite inference at 5Hz was feeding
    disabled Mapper. Saves GPU + 1 thread + battery.
    (4) Fixed map lag: replaced mutableStateListOf (O(n) state tracking) with
    plain ArrayList + version counter. Throttled GoogleMap Polyline/Marker
    recomposition from 5Hz to 1Hz. Cached BitmapDescriptor icons.
    (5) Fixed crash: BitmapDescriptorFactory.fromBitmap() called before Maps SDK
    init — moved icon caching inside GoogleMap scope.
    REFERENCE: commit 10fb69a = best drift result (5.4%, 1.2m on 22m).
  files_changed:
    - "app/CMakeLists.txt (removed Mapper.cpp, LoopClosureDetector.cpp, PoseGraph.cpp)"
    - "app/src/main/cpp/VioEngine.h/cpp (disabled Mapper thread + all Mapper refs)"
    - "app/src/main/cpp/native-lib.cpp (commented processCameraFrame ByteArray + setMagnetometerHeading JNI)"
    - "app/src/main/cpp/IMUPreintegrator.cpp/h (commented 10 dead functions)"
    - "app/src/main/cpp/EKFState.cpp/h (commented 5 dead functions)"
    - "app/src/main/cpp/Tracker.cpp/h (commented blendScale, applyLoopCorrection)"
    - "app/src/main/cpp/LensCorrector.cpp/h (commented setDistortion, undistortPoints single)"
    - "app/src/main/cpp/FeatureManager.h (commented getKeyframeCount, getNextFeatureId)"
    - "app/src/main/java/.../NavSightViewModel.kt (pathHistory: mutableStateListOf → ArrayList + version)"
    - "app/src/main/java/.../MainActivity.kt (map 1Hz throttle, cached icons, crash fix)"
    - "app/src/main/java/.../SensorRepository.kt (disabled DepthEstimator + depthExecutor)"
    - "app/src/main/java/.../NativeBridge.kt (commented processCameraFrame ByteArray, setMagnetometerHeading)"
    - "app/src/main/java/.../AROverlayRenderer.kt (entire file commented — all composables dead)"
    - "app/src/main/java/.../NavSightUtils.kt (commented computeCalibrationStraightness, nv21ToBitmap)"
    - "app/src/main/java/.../DeviceOrientationTracker.kt (commented isPhoneHorizontal, getCompassHeading)"
    - "app/src/main/java/.../RoadSnapper.kt (commented clearCache)"

- session: "0h"
  date: "2026-04-08"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  commits: "32d8958, 69a3e28, 5709bd2, 10fb69a"
  summary: >
    BREAKTHROUGH SESSION: 27% drift → 5.4% drift. Heading now tracks turns.
    CameraX migration, scalar heading, SO(3) fix, accel bias, EKF gravity,
    gyro bias init, ZUPT covariance. ON-DEVICE: 1.2m on 22m out-and-back.

- session: "0g"
  date: "2026-04-08"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: "VIO accuracy improvement plan (10 phases). OpenVINS gap analysis."

- session: "0f"
  date: "2026-04-05"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: "Major VIO architecture overhaul + 6 critical fixes. Deleted VisionModule.cpp."

```

---

## CONVERSATION CONTEXT

```yaml
current_task: "V-shape heading fix committed. Awaiting real-device test."
stopped_at: "All session 0j changes committed to morad branch. Build passes."
next_action: >
  1. TEST V-SHAPE FIX: walk straight, 180° turn, walk back. Return path should overlap.
  2. VERIFY MiDaS: check logcat for DEPTH_SCALE messages during walking.
  3. FIX BUG-016: use-after-free — switch to shared_ptr<VioEngine>.
  4. Scale calibration: investigate why scale stuck at 0.12-0.20 despite steps.
  5. Places API: enable billing on Google Cloud Project to fix search.
resume_context: >
  Architecture: Tracker-only (Mapper disabled). CameraX with zero-copy JNI.
  Heading: scalar_heading_ with gravity-projected yaw rate + keyframe correction
  (gated on gyro_norm < 0.3 to prevent turn damage).
  MiDaS: re-enabled at 1Hz, feeds depth-based scale constraint in Tracker.
  Declination: added at startup via GeomagneticField (~5.5° Haifa).
  Map: 4Hz heading-locked camera, recenter FAB, 1Hz overlays.
  REFERENCE COMMIT: 10fb69a = best drift (5.4%, 1.2m on 22m).
partial_state: "NONE — all changes committed, build passes"
warnings:
  - "BUG-016 (use-after-free) can crash on VIO stop/restart — fix before release"
  - "V-shape fix not yet tested on device — needs 180° turn simulation"
  - "MiDaS depth constraint hasn't fired in any simulation yet"
  - "Places API search needs Google Cloud billing enabled"
  - "Scale never calibrates (0.12-0.20) — position accuracy comes from VO, not scale"
```

---

## FILE MAP (key files only)

```yaml
# C++ VIO Engine (ACTIVE)
"app/src/main/cpp/VioEngine.h/cpp":        "Orchestrator: Tracker only (Mapper disabled)"
"app/src/main/cpp/Tracker.h/cpp":          "Core VIO: optical flow, essential matrix, scalar heading, pose"
"app/src/main/cpp/EKFState.h/cpp":         "15-DOF error-state EKF + legacy 1-state scale filter"
"app/src/main/cpp/IMUPreintegrator.h/cpp": "SO(3) preintegration, filtered gravity, step detection"
"app/src/main/cpp/UpdaterZeroVelocity.h/cpp": "Chi-squared ZUPT detector (ACTIVE)"
"app/src/main/cpp/TrackKLT.h/cpp":         "Lucas-Kanade optical flow + geometric verification"
"app/src/main/cpp/FeatureManager.h/cpp":   "Grid features, sparse replenish, keyframe store/match"
"app/src/main/cpp/LensCorrector.h/cpp":    "Lens undistortion for matched points"
"app/src/main/cpp/native-lib.cpp":         "JNI bridge: processCameraFrameDirect (zero-copy)"

# C++ DISABLED (compiled but unused, or removed from CMakeLists)
"app/src/main/cpp/Mapper.h/cpp":           "DISABLED — removed from CMakeLists"
"app/src/main/cpp/LoopClosureDetector.h/cpp": "DISABLED — removed from CMakeLists"
"app/src/main/cpp/PoseGraph.h/cpp":        "DISABLED — removed from CMakeLists"
"app/src/main/cpp/UpdaterMSCKF.h/cpp":     "Types used by FeatureManager, processLostFeatures never called"
"app/src/main/cpp/InertialInitializer.h/cpp": "ACTIVE — used by Tracker for system init"

# Kotlin App Layer
"app/src/main/java/.../MainActivity.kt":       "CameraX, Compose UI, 4Hz heading-locked map, recenter FAB"
"app/src/main/java/.../NavSightViewModel.kt":  "MVVM state, pathHistory (ArrayList + version)"
"app/src/main/java/.../SensorRepository.kt":   "ImageProxy dispatch, DepthEstimator at 1Hz, declination"
"app/src/main/java/.../NativeBridge.kt":        "JNI declarations (processCameraFrameDirect)"
"app/src/main/java/.../DepthEstimator.kt":      "MiDaS TFLite depth (RE-ENABLED at 1Hz GPU)"
```

---

## KEY CONSTANTS (current values)

```yaml
MAX_FEATURES:        200    # Tracker.h (was 400 in plan, actual code is 200)
MIN_FEATURES:        80
QUALITY_LEVEL:       0.05
MIN_DIST:            10.0
RANSAC_CONF:         0.9999
RANSAC_THRESH:       1.5
MIN_PARALLAX_PX:     0.8
FB_CHECK_THRESH:     4.0    # squared px (2px threshold)
MIN_FLOW_PX:         0.4
MAX_FLOW_PX:         150.0
MIN_INLIERS:         8
MIN_INLIER_RATIO:    0.25
GYRO_ROT_ONLY_THRESH: 2.0
ZUPT_GYRO_THRESH:    0.04
smooth_scale_init:   0.20
UI_UPDATE_THROTTLE_MS: 200  # 5Hz VIO → UI updates
MAP_UPDATE_THROTTLE:   1000 # 1Hz map camera + polyline + markers
```

---

## PROTOCOL

### For the starting AI agent:
1. Read this entire file first.
2. Check ACTIVE BUGS — do not duplicate work on IN_PROGRESS items.
3. Check CONVERSATION CONTEXT for any partial work to resume.
4. REFERENCE COMMIT 10fb69a = best drift (revert here if regressions).

### For the finishing AI agent:
1. Update ACTIVE BUGS statuses.
2. Move completed PENDING WORK to DONE.
3. Add a session entry to RECENT CHANGES (keep only last 5).
4. Update CONVERSATION CONTEXT with where you stopped.
5. Update META section.
6. Commit this file.

### Purge policy:
- Remove RESOLVED bugs after 2 sessions.
- Remove DONE tasks after 3 sessions.
- Keep only 5 entries in RECENT CHANGES.
