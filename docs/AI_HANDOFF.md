# NavSight AI Handoff Protocol

<!-- FORMAT: YAML-like sections for AI parseability. Human-readable but structured. -->
<!-- RULE: Keep total file under 400 lines. Purge COMPLETED items older than 3 sessions. -->
<!-- RULE: Each session entry in CHANGELOG must have: agent, developer, date, branch. -->
<!-- RULE: Before editing, pull latest. After editing, commit immediately. -->

---

## META

```yaml
last_updated: "2026-04-08"
last_agent: "Claude-Opus-4-6"
last_developer: "morad"
branch: "morad"
base_branch: "master"
head_commit: "10fb69a"
```

---

## DEVELOPERS

```yaml
morad:
  role: "IMU preintegration, sensor fusion, calibration, integration testing"
  branch: "morad"
  agent: "Claude-Opus-4-6"
  last_session: "2026-04-05"

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

## ARCHITECTURE (current — post-refactor)

```yaml
# VisionModule.cpp has been DELETED. Replaced by multi-file architecture:
#
#   VioEngine (orchestrator)
#     ├── Tracker (fast path: optical flow, essential matrix, rotation fusion, pose update)
#     ├── Mapper (background thread: ground plane, bundle adjustment, loop closure)
#     ├── IMUPreintegrator (gyro/accel buffering, preintegration, step detection, motion mode)
#     ├── EKFState (15-DOF error-state EKF with MSCKF sliding window support)
#     ├── UpdaterZeroVelocity (chi-squared ZUPT detector)
#     ├── UpdaterMSCKF (null-space feature marginalization — scaffolded)
#     ├── PoseGraph (pose graph optimization for loop closure — scaffolded)
#     └── InertialInitializer (stationary init — scaffolded)
#
# Camera pipeline: CameraX ImageAnalysis (zero-copy via GetDirectBufferAddress)
#   ImageProxy planes → direct ByteBuffer → JNI processCameraFrameDirect
#   STRATEGY_KEEP_ONLY_LATEST, YUV_420_888, 640x480 target resolution
#
# Tracker fast path runs on camera thread (~10ms).
# Mapper runs on a dedicated background thread via std::condition_variable.
# One-frame-delayed result application (non-blocking).
#
# Heading pipeline (REWRITTEN 2026-04-08):
#   magnetometer → setInitialHeading(azimuth_rad) → scalar_heading_ = azimuth
#   each frame: gravity-projected yaw rate from gyro:
#     grav = getFilteredGravity() (low-pass filtered accel, tracks phone tilt)
#     yaw_rate = -dot(omega, normalize(grav))
#     scalar_heading_ += yaw_rate * dt
#   heading = scalar_heading_ (radians, wraps at ±π)
#   OLD METHOD REMOVED: Rodrigues vector component blending cross-coupled axes
#
# IMU preintegration (FIXED 2026-04-08):
#   Rotation: proper SO(3) via Exp(w*dt) Rodrigues, NOT matrix addition
#   Velocity/Position: midpoint integration (Forster 2017 §IV.A)
#   Gyro bias subtracted before integration (was already done)
#   Accel bias subtracted before integration (was MISSING, now fixed)
#   Covariance: Forster 2017 discrete propagation with bias Jacobians
#
# Scale pipeline:
#   step detection → speed × time → estimateScaleFromSteps → EKF updateScale
#   ground plane (Mapper) → absolute scale from camera height → blendScale
#   bundle adjustment (Mapper) → optimized scale → blendScale
#
# Deleted files: VisionModule.cpp, VisionModule.h, ThreadSafeQueue.h
```

---

## CODEBASE STATE

### Build Status

```yaml
android_build: "PASSES"  # Verified 2026-04-08
native_cpp_build: "PASSES"  # All C++ files including new MSCKF, ZUPT, PoseGraph, InertialInit
kotlin_compile: "PASSES"
desktop_cpp_tests: "36/36 PASS (tests reference old VisionModule — need update)"
on_device_testing: "TESTED 2026-04-08 — 5.4% drift on 22m out-and-back, heading tracks 180° turns"
```

---

## ACTIVE BUGS

```yaml
# BUG-009, BUG-010, BUG-011: RESOLVED (purged — fixed 2026-04-05, verified 2026-04-08)

- id: "BUG-012"
  title: "Tracking quality drops to 0 when moving — falls back to IMU-only"
  status: "IMPROVED"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  reported: "2026-04-05 (on-device test)"
  description: >
    IMPROVED (2026-04-08): CameraX migration with zero-copy frame delivery improved
    FPS significantly. Sim shows quality oscillates between 0.04 and 0.99 during
    walking (was always 0 before). Quality still drops during fast turns (expected).
    Position accuracy 5.4% drift despite quality drops — visual odometry recovers.
  test: "Walk in good light — quality should average >30% during straight walking."

- id: "BUG-013"
  title: "Heading rotates when standing still"
  status: "FIXED_VERIFIED"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  reported: "2026-04-05 (on-device test)"
  description: >
    FIXED (2026-04-08): Heading now uses scalar_heading_ with gravity-projected
    yaw rate instead of global_R_ accumulation. Section 9 skips heading update
    when is_static=true. Sim shows heading stable at ±0.1° during stationary periods.

- id: "BUG-014"
  title: "Scale stuck at 0.12-0.20, never calibrates from steps"
  status: "OPEN"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  reported: "2026-04-08 (simulation analysis)"
  description: >
    Despite 67 steps detected, scale oscillates between 0.12-0.21 and never
    converges to a meaningful value. Step-based scale estimation may not be
    feeding the EKF correctly, or the EKF's scale update is too conservative.
    Position accuracy is OK despite this (visual odometry compensates).
  test: "Walk 20m — scale should converge toward ~0.5-1.0 after 30+ steps."

- id: "BUG-015"
  title: "QA: Data race on EKF between Tracker and Mapper threads"
  status: "OPEN"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/VioEngine.cpp"
  reported: "2026-04-08 (QA scan)"
  description: >
    Mapper thread reads EKF state (getWindow, isFullInitialized) while Tracker
    thread concurrently mutates it (propagateIMU, addClone, updateZUPT).
    No synchronization. UB — can cause crashes or corrupted state.
  fix_hints: "Deep-copy EKF snapshot before passing to Mapper, or add shared_mutex."

- id: "BUG-016"
  title: "QA: use-after-free risk in native-lib processCameraFrame"
  status: "OPEN"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/native-lib.cpp"
  reported: "2026-04-08 (QA scan)"
  description: >
    Raw pointer snapshot of g_vision used after releasing lock. If stopVIO()
    deletes it concurrently, use-after-free crash.
  fix_hints: "Use std::shared_ptr<VioEngine> instead of raw pointer."

- id: "BUG-017"
  title: "QA: Loop closure adds self-edge (same node ID)"
  status: "OPEN"
  severity: "P2"
  owner: "morad"
  file: "app/src/main/cpp/Mapper.cpp:608"
  reported: "2026-04-08 (QA scan)"
  description: >
    pose_graph_.addLoopEdge uses last_pose_graph_node_id_ for both source and
    target. Should use the matched keyframe's node ID as second argument.

- id: "BUG-018"
  title: "QA: Mapper applyMapperResult is a no-op — entire Mapper thread wasted"
  status: "OPEN"
  severity: "P2"
  owner: "morad"
  file: "app/src/main/cpp/VioEngine.cpp:100-113"
  reported: "2026-04-08 (QA scan)"
  description: >
    applyMapperResult discards all results with (void)mr; (void)out.
    Mapper thread runs ORB, RANSAC, BA every frame for nothing.
  fix_hints: "Either re-enable result application or disable heavy computation."
```

---

## PENDING WORK

```yaml
# TASK-018 through TASK-027: DONE (purged — completed 2026-04-05)

- id: "TASK-029"
  title: "CameraX migration + zero-copy frame delivery"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-04-08"
  notes: >
    Replaced CameraView library with CameraX ImageAnalysis. Zero-copy JNI via
    GetDirectBufferAddress on direct ByteBuffers from ImageProxy planes.
    processCameraFrameDirect assembles NV21 from YUV_420_888. Significant FPS improvement.

- id: "TASK-030"
  title: "Fix heading: scalar gravity-projected yaw rate"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-04-08"
  notes: >
    Replaced Rodrigues vector component blending (cross-coupled axes) with
    scalar heading tracker. yaw_rate = -dot(omega, normalize(gravity)).
    Uses filtered gravity (low-pass accel) to track phone tilt during use.
    Heading now tracks 180° turns correctly (verified on-device).

- id: "TASK-031"
  title: "Fix 6 critical/high bugs from QA scan"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-04-08"
  notes: >
    (1) SO(3) integration: replaced RK4 matrix addition with proper Exp(w*dt).
    (2) Accel bias: now subtracted during preintegration (was missing).
    (3) EKF gravity: added g*dt to velocity, 0.5*g*dt² to position propagation.
    (4) Gyro bias init: use most recent 200 samples (was oldest).
    (5) integrateGyro: right-multiplication (consistent with integrate).
    (6) ZUPT covariance: fixed double-dampening to maintain PSD.
    Result: 5.4% drift (1.2m on 22m out-and-back).

- id: "TASK-028"
  title: "MiDaS monocular depth for absolute scale"
  status: "SCAFFOLDED"
  owner: "morad"
  priority: "P2"
  notes: >
    TFLite model bundled in assets/midas_v21_small.tflite. DepthEstimator.kt
    scaffolded. Not yet wired into VIO pipeline.

- id: "TASK-032"
  title: "Fix remaining QA scan issues (threading, dead code)"
  status: "TODO"
  owner: "morad"
  priority: "P1"
  notes: >
    BUG-015: EKF data race (Tracker vs Mapper threads).
    BUG-016: use-after-free in native-lib (raw pointer).
    BUG-017: Loop closure self-edge.
    BUG-018: applyMapperResult is a no-op (Mapper thread wasted).
    Plus: duplicate code in native-lib, O(n) IMU buffer scan, RANSAC 0.9999.
```

---

## RECENT CHANGES (last 5 sessions)

```yaml
- session: "0h"
  date: "2026-04-08"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  commits: "32d8958, 69a3e28, 5709bd2, 10fb69a"
  summary: >
    BREAKTHROUGH SESSION: 27% drift → 5.4% drift. Heading now tracks turns.
    (1) CameraX migration: replaced CameraView with CameraX ImageAnalysis + zero-copy
    JNI via GetDirectBufferAddress. New processCameraFrameDirect endpoint.
    (2) Heading rewrite: scalar_heading_ with gravity-projected yaw rate replaces
    Rodrigues component blending. Filtered gravity (LP accel) tracks phone tilt.
    (3) QA scan (29 issues found): fixed 6 critical/high bugs —
    SO(3) integration (Exp map), accel bias subtraction, EKF gravity term,
    gyro bias init (most recent samples), integrateGyro convention, ZUPT covariance.
    (4) New scaffolded files: PoseGraph, UpdaterMSCKF, UpdaterZeroVelocity,
    InertialInitializer, DepthEstimator.kt. MiDaS TFLite model bundled.
    (5) EKF expanded to 15-DOF error-state with MSCKF sliding window support.
    ON-DEVICE VERIFIED: 1.2m return error on 22m out-and-back (5.4% drift).
  files_changed:
    - "app/build.gradle.kts (CameraView→CameraX deps, TFLite noCompress)"
    - "app/src/main/java/.../MainActivity.kt (CameraX ImageAnalysis setup)"
    - "app/src/main/java/.../SensorRepository.kt (ImageProxy zero-copy dispatch)"
    - "app/src/main/java/.../NativeBridge.kt (processCameraFrameDirect JNI)"
    - "app/src/main/java/.../NavSightViewModel.kt (ImageProxy passthrough)"
    - "app/src/main/cpp/native-lib.cpp (processCameraFrameDirect, NV21 assembly)"
    - "app/src/main/cpp/Tracker.cpp (scalar heading, filtered gravity, remove double bias)"
    - "app/src/main/cpp/Tracker.h (scalar_heading_ field)"
    - "app/src/main/cpp/IMUPreintegrator.cpp (SO(3) fix, accel bias, filtered gravity, gyro bias init)"
    - "app/src/main/cpp/IMUPreintegrator.h (b_a_, filtered_gravity_, getFilteredGravity)"
    - "app/src/main/cpp/EKFState.cpp (gravity in propagation, ZUPT covariance fix)"
    - "app/src/main/cpp/EKFState.h (15-DOF error-state, MSCKF clone support)"
    - "NEW: UpdaterZeroVelocity.cpp/h, UpdaterMSCKF.cpp/h, PoseGraph.cpp/h"
    - "NEW: InertialInitializer.cpp/h, DepthEstimator.kt"
    - "NEW: app/src/main/assets/midas_v21_small.tflite"

- session: "0g"
  date: "2026-04-08"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    VIO accuracy improvement plan created (10 phases). Research into OpenVINS
    gap analysis. Plan stored at .claude/plans/lexical-squishing-stream.md.

- session: "0f"
  date: "2026-04-05"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Major VIO architecture overhaul + 6 critical fixes.
    Parallel VIO, EKF rewrite, degenerate handling, heading fix, FB threshold.
    Deleted VisionModule.cpp/h. Analyzed 8 simulation recordings.

- session: "0e"
  date: "2026-04-01"
  developer: "morad"
  agent: "Gemini CLI"
  branch: "morad"
  summary: >
    Fixed heading freeze, geodesic coordinates, ghost walking, simulation save crash.

- session: "0d"
  date: "2026-03-31"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Removed Kotlin OpticalFlowProcessor for FPS boost. Camera ~2-3fps → expected ~5fps.
```

---

## CONVERSATION CONTEXT

```yaml
current_task: "Continue QA fixes from scan + improve scale calibration"
stopped_at: "6 critical/high QA bugs fixed. On-device verified: 5.4% drift, heading works."
next_action: >
  1. FIX BUG-015: EKF data race — deep-copy EKF snapshot for Mapper thread.
  2. FIX BUG-016: use-after-free — switch to shared_ptr<VioEngine>.
  3. Scale calibration: investigate why scale stuck at 0.12-0.20 despite 67 steps.
  4. Consider remaining QA items: loop closure self-edge, applyMapperResult no-op.
  5. VIO accuracy plan phases 2-5 (feature aging, reprojection gating, ZUPT tuning, mag init).
resume_context: >
  Architecture: Tracker+Mapper+VioEngine. CameraX with zero-copy JNI.
  Heading: scalar_heading_ with gravity-projected yaw rate (filtered gravity LP accel).
  IMU preintegration: proper SO(3) Exp map, accel bias subtracted, midpoint V/P.
  EKF: 15-DOF error-state with MSCKF clone support (legacy 1-state scale still active).
  Key result: 1.2m return error on 22m out-and-back = 5.4% drift.
  Key result: heading tracks 180° turns (156° → -37° = ~193° change, expected 180°).
  QA scan found 29 issues, 6 fixed, 23 remaining (mostly medium/low).
partial_state: "NONE — all code changes done, build passes, on-device verified"
warnings:
  - "BUG-015 (EKF data race) can cause rare crashes — fix before release"
  - "BUG-016 (use-after-free) can crash on VIO stop/restart — fix before release"
  - "Scale never calibrates (0.12-0.20) — position accuracy comes from VO, not scale"
  - "applyMapperResult is no-op — Mapper thread wastes CPU every frame"
  - "C++ tests reference old VisionModule — need updating for Tracker/Mapper"
  - "MiDaS TFLite bundled but not wired into pipeline"
  - "VIO accuracy plan phases 2-10 not yet started"
```

---

## FILE MAP (key files only)

```yaml
# C++ VIO Engine
"app/src/main/cpp/VioEngine.h/cpp":      "Parallel orchestrator: Tracker + Mapper thread"
"app/src/main/cpp/Tracker.h/cpp":        "Core VIO: optical flow, essential matrix, scalar heading, pose"
"app/src/main/cpp/Mapper.h/cpp":         "Background: ground plane, BA, loop closure (results currently discarded)"
"app/src/main/cpp/EKFState.h/cpp":       "15-DOF error-state EKF + legacy 1-state scale filter"
"app/src/main/cpp/IMUPreintegrator.h/cpp": "SO(3) preintegration, filtered gravity, step detection, motion mode"
"app/src/main/cpp/UpdaterZeroVelocity.h/cpp": "Chi-squared ZUPT detector (stationary detection)"
"app/src/main/cpp/UpdaterMSCKF.h/cpp":   "Null-space feature marginalization (scaffolded)"
"app/src/main/cpp/PoseGraph.h/cpp":      "Pose graph optimization for loop closure (scaffolded)"
"app/src/main/cpp/InertialInitializer.h/cpp": "Stationary initialization (scaffolded)"
"app/src/main/cpp/FeatureManager.h/cpp": "Grid features, sparse replenish, keyframe store/match"
"app/src/main/cpp/LensCorrector.h/cpp":  "Lens undistortion for matched points"
"app/src/main/cpp/LoopClosureDetector.h/cpp": "ORB-based loop closure detection"
"app/src/main/cpp/native-lib.cpp":       "JNI bridge: processCameraFrame + processCameraFrameDirect (zero-copy)"

# Kotlin App Layer
"app/src/main/java/.../MainActivity.kt":         "CameraX ImageAnalysis setup, Compose UI, map, radar"
"app/src/main/java/.../NavSightViewModel.kt":    "MVVM state, simulation recording"
"app/src/main/java/.../SensorRepository.kt":     "ImageProxy zero-copy dispatch, sensor registration"
"app/src/main/java/.../NativeBridge.kt":          "JNI declarations (processCameraFrameDirect)"
"app/src/main/java/.../VioData.kt":               "VIO data class: 29 fields + JNI sig"
"app/src/main/java/.../DepthEstimator.kt":        "MiDaS TFLite depth estimation (scaffolded)"

# Assets & Models
"app/src/main/assets/midas_v21_small.tflite":  "MiDaS v2.1 Small depth model (~5MB)"

# Simulation & Analysis
"simulator/simulation_data_*.json":      "Recordings (latest: 1775658313926 = 5.4% drift verified)"
```

---

## KEY CONSTANTS (current values)

```yaml
MAX_FEATURES:        400
MIN_FEATURES:        120
QUALITY_LEVEL:       0.05
MIN_DIST:            10.0
RANSAC_CONF:         0.9999
RANSAC_THRESH:       0.5     # px
MIN_PARALLAX_PX:     0.8     # px
FB_CHECK_THRESH:     9.0     # squared px (3px threshold)
MIN_FLOW_PX:         0.4     # px
MAX_FLOW_PX:         150.0   # px (was 50.0)
MIN_INLIERS:         8
MIN_INLIER_RATIO:    0.25
GYRO_ROT_ONLY_THRESH: 2.0    # rad/s
ZUPT_GYRO_THRESH:    0.04    # rad/s
ACCEL_BIAS_WARMUP:   150
ACCEL_BIAS_ALPHA:    0.005
smooth_scale_init:   0.20
# EKF (EKFState.h)
SIGMA_SCALE_RW:      0.001   # scale random walk per second
SIGMA_SCALE_MEAS:    0.1     # scale observation noise
# Mapper (Mapper.h)
CAMERA_HEIGHT:       1.4     # meters (standing with phone)
BA_INTERVAL:         5       # frames between BA runs
BA_MAX_ITER:         10
HUBER_THRESHOLD:     2.0     # pixels
# Degenerate thresholds (Tracker.cpp)
SVD_TRANS_DEGEN:     100.0   # SVD cond > this = translation degenerate
SVD_FULL_DEGEN:      50000.0 # SVD cond > this = fully degenerate (skip all)
```

---

## PROTOCOL

### For the starting AI agent:
1. Read this entire file first.
2. Check ACTIVE BUGS — do not duplicate work on IN_PROGRESS items.
3. Check CONVERSATION CONTEXT for any partial work to resume.
4. Note: ALL 2026-04-05 changes are UNTESTED. First priority is on-device verification.

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
