# NavSight AI Handoff Protocol

<!-- FORMAT: YAML-like sections for AI parseability. Human-readable but structured. -->
<!-- RULE: Keep total file under 400 lines. Purge COMPLETED items older than 3 sessions. -->
<!-- RULE: Each session entry in CHANGELOG must have: agent, developer, date, branch. -->
<!-- RULE: Before editing, pull latest. After editing, commit immediately. -->

---

## META

```yaml
last_updated: "2026-04-05"
last_agent: "Claude-Opus-4-6"
last_developer: "morad"
branch: "morad"
base_branch: "master"
head_commit: "d90454f"
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
#     ├── IMUPreintegrator (gyro/accel buffering, step detection, motion mode)
#     └── EKFState (1-state scalar Kalman filter for scale only)
#
# Tracker fast path runs on camera thread (~10ms).
# Mapper runs on a dedicated background thread via std::condition_variable.
# One-frame-delayed result application (non-blocking).
#
# Heading pipeline:
#   magnetometer → setInitialHeading(azimuth_rad) → global_R_ = Rz(azimuth)
#   each frame: global_R_ *= R_fused (camera+gyro blend) or R_corrected (gyro fallback)
#   heading = atan2(R[1][0], R[0][0])  ← ZYX yaw extraction (pitch-independent)
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
android_build: "PASSES"  # Verified 2026-04-05
native_cpp_build: "PASSES"  # Tracker, Mapper, VioEngine, EKFState, FeatureManager, LensCorrector
kotlin_compile: "PASSES"
desktop_cpp_tests: "36/36 PASS (tests reference old VisionModule — need update)"
on_device_testing: "UNTESTED — latest changes from 2026-04-05 need on-device verification"
```

---

## ACTIVE BUGS

```yaml
- id: "BUG-009"
  title: "Essential matrix always rejected as degenerate for forward walking"
  status: "FIXED_UNTESTED"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  description: >
    FIXED (2026-04-05): SVD condition number >100 rejected ALL frames during
    forward walking. Restructured: SVD 100-50000 = translation_degenerate
    (rotation still used for heading, step-based displacement). SVD >50000 =
    truly degenerate. Now pose_valid=true for most frames with good inliers.
  test: "Record walking sim — check pflags has bit2 (pose_valid=4) set"

- id: "BUG-010"
  title: "Heading extraction contaminated by phone pitch"
  status: "FIXED_UNTESTED"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  description: >
    FIXED (2026-04-05): atan2(R[1][0], R[1][1]) is pitch-dependent.
    Phone bobbing during walking corrupted heading. Changed all 3 extraction
    points to atan2(R[1][0], R[0][0]) — standard ZYX yaw, pitch-independent.
  test: "Walk straight line — heading should stay constant (not oscillate)"

- id: "BUG-011"
  title: "Tracking quality 90% in darkness, <10% in good light"
  status: "FIXED_UNTESTED"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  description: >
    FIXED (2026-04-05): Two causes: (1) FB_CHECK_THRESH=2.0 (1.41px) too strict
    for real motion — raised to 9.0 (3px). (2) CLAHE amplifies noise in darkness,
    noise tracks with ~0px flow → false 90% quality. Added: quality=0 when is_low_light.
  test: "Cover camera — quality should drop to 0%. Good light should show 50%+."

- id: "BUG-012"
  title: "Tracking quality drops to 0 when moving — falls back to IMU-only"
  status: "OPEN"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  reported: "2026-04-05 (on-device test)"
  description: >
    When walking, tracking quality drops to 0 and system falls to IMU-only mode.
    Likely causes: (1) Section 8 gate at line 297 requires !is_low_light — check
    if brightness threshold 0.12 is too high for indoor lighting. (2) FB check
    (FB_CHECK_THRESH=9.0) may still be too strict for large optical flow during
    walking. (3) is_low_light clamp at line 253 forces quality=0 even if features
    are tracked. (4) Possible: all features genuinely lost during motion (replenishment
    not fast enough). Check GATES log line for diagnostics: flow, blur, motion,
    parallax, static, rot, pts, gyro, lowlight values during walking.
  fix_hints: >
    - Log frame_brightness during walks to verify is_low_light isn't false-triggering.
    - If brightness is fine, FB_CHECK_THRESH may need raising (try 16.0 = 4px).
    - If tracked count is fine but quality=0, the is_low_light clamp is the culprit.
    - Consider removing is_low_light from section 8 gate — let quality handle it.
  test: "Walk in good light — quality should be >30%, pflags should have bit2 (pose_valid=4)."

- id: "BUG-013"
  title: "Heading rotates when standing still"
  status: "OPEN"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/Tracker.cpp"
  reported: "2026-04-05 (on-device test)"
  description: >
    When stationary, heading keeps drifting/rotating. Root cause: Section 9
    (lines 428-439) applies rotation to global_R_ BEFORE the is_static check
    at line 443. ZUPT only freezes translation, not rotation. So gyro noise
    (even bias-corrected) accumulates in heading when standing still. Also:
    if pose_valid=false (due to BUG-012), gyro fallback at line 432-439 applies
    raw gyro rotation with imperfect bias correction, causing drift.
  fix_hints: >
    - When is_static is true, skip the rotation update entirely (both camera
      and gyro paths). Heading should be frozen when standing still.
    - Move the is_static check BEFORE the rotation update block.
    - Alternative: apply a tiny rotation damping factor when is_static detected
      (multiply rv by 0.0 or very small alpha).
    - Also check if ZUPT_GYRO_THRESH (0.04 rad/s) is too low — phone table
      vibrations can exceed this. Try 0.08.
  test: "Place phone on table — heading should stay constant (±0.5° max over 30s)."
```

---

## PENDING WORK

```yaml
- id: "TASK-018"
  title: "Parallel VIO refactor"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-05"
  notes: >
    Phase 2 complete: Tracker on camera thread, Mapper on background thread.
    VioEngine orchestrates with condition_variable. Phase 3 (WorldState 200Hz)
    still TODO.

- id: "TASK-022"
  title: "EKF decoupled to 1-state scale-only filter"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-04-05"
  notes: >
    Old 3-state EKF [heading, gyro_bias, scale] had cross-covariance leakage.
    Rewritten as scalar Kalman filter for scale only. Heading from global_R_
    accumulation. Gyro bias learned separately in Tracker.

- id: "TASK-023"
  title: "Fix essential matrix degenerate rejection (BUG-009)"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-04-05"
  notes: >
    SVD condition split: 100-50000 = translation_degenerate (rotation valid),
    >50000 = truly degenerate. Rotation fusion + gyro bias learning happen for
    all frames with good inliers. Scale/triangulation only with good translation.

- id: "TASK-024"
  title: "Fix heading extraction pitch contamination (BUG-010)"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-04-05"
  notes: "atan2(R[1][0], R[1][1]) → atan2(R[1][0], R[0][0]) at all 3 extraction points."

- id: "TASK-025"
  title: "Fix tracking quality inversion in darkness (BUG-011)"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-05"
  notes: "FB_CHECK_THRESH 2.0→9.0. Low-light quality clamped to 0."

- id: "TASK-026"
  title: "Step speed interpolation for displacement gaps"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-05"
  notes: >
    Fallback displacement had gaps when step detection paused briefly.
    Now maintains last_step_speed_ with 2.5s decay. Also removed motion_blur
    and mean_flow gates from fallback — step detection alone is sufficient.

- id: "TASK-027"
  title: "Ground plane runs without pose_valid"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-04-05"
  notes: >
    Mapper ground plane detection now runs every frame (needs only image +
    features). Provides absolute scale from camera height even during fallback.

- id: "TASK-028"
  title: "MiDaS monocular depth for absolute scale (SUGGESTION)"
  status: "TODO"
  owner: "unassigned"
  priority: "P2"
  notes: >
    SUGGESTED ENHANCEMENT: Integrate MiDaS v2.1 Small (~5MB TFLite model) for
    monocular depth estimation. Would provide depth at tracked features →
    absolute scale without step detection or manual calibration. Works on
    any phone (no ToF sensor needed). Steps: add TFLite dep, bundle .tflite
    in assets, DepthEstimator class runs inference every N frames, depth fed
    to EKF for scale. Alternative to ARCore (which requires camera pipeline
    rewrite). User confirmed interest in AI-based depth.
```

---

## RECENT CHANGES (last 5 sessions)

```yaml
- session: "0f"
  date: "2026-04-05"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Major VIO architecture overhaul + 6 critical fixes. NONE TESTED ON DEVICE YET.
    (1) Parallel VIO: Tracker fast path + Mapper background thread (TASK-018 Phase 2).
    (2) EKF rewritten as 1-state scale-only filter (removed heading/bias cross-covariance).
    (3) Essential matrix degenerate handling restructured — rotation valid even when
    translation is degenerate (SVD 100-50000). Fixes 0% pose_valid in all simulations.
    (4) Heading extraction bug: atan2(R[1][0],R[1][1]) → atan2(R[1][0],R[0][0]) for
    pitch-independent yaw. (5) FB_CHECK_THRESH 2.0→9.0 + low-light quality clamp.
    (6) Step speed interpolation, ground plane without pose_valid, MAX_FLOW_PX 50→150.
    Deleted: VisionModule.cpp/h, ThreadSafeQueue.h (replaced by Tracker/Mapper/VioEngine).
    Analyzed 8 simulation recordings — all showed 0 pose_valid, heading drift, scale=1.0.
  files_changed:
    - "app/src/main/cpp/Tracker.cpp (degenerate restructure, heading fix, FB threshold, speed interp)"
    - "app/src/main/cpp/Tracker.h (MAX_FLOW_PX=150, FB_CHECK_THRESH=9.0, step speed fields)"
    - "app/src/main/cpp/VioEngine.cpp (NEW: parallel orchestrator with background Mapper)"
    - "app/src/main/cpp/VioEngine.h (NEW: thread management, result passing)"
    - "app/src/main/cpp/EKFState.cpp (REWRITTEN: 1-state scalar scale filter)"
    - "app/src/main/cpp/EKFState.h (REWRITTEN: removed heading/bias states)"
    - "app/src/main/cpp/Mapper.cpp (ground plane without pose_valid, LM lambda fix)"
    - "app/src/main/cpp/Mapper.h (KeyframeWindow::data() accessor)"
    - "app/src/main/cpp/IMUPreintegrator.cpp (gyro bias fix in integrate())"
    - "app/CMakeLists.txt (removed VisionModule comment)"
    - "app/src/main/cpp/VisionModule.cpp (DELETED)"
    - "app/src/main/cpp/VisionModule.h (DELETED)"
    - "app/src/main/cpp/ThreadSafeQueue.h (DELETED)"
  impact: "All changes build successfully. UNTESTED ON DEVICE. Must rebuild APK and re-record."

- session: "0e"
  date: "2026-04-01"
  developer: "morad"
  agent: "Gemini CLI"
  branch: "morad"
  summary: >
    Fixed heading freeze, geodesic coordinates, ghost walking, simulation save crash.
  files_changed:
    - "app/src/main/cpp/VisionModule.cpp (relaxed rotation gates)"
    - "app/src/main/cpp/IMUPreintegrator.cpp (2s step timeout)"
    - "app/src/main/java/.../NavSightUtils.kt (geodesic metersToLatLng)"
    - "app/src/main/java/.../NavSightViewModel.kt (thread-safe save)"

- session: "0d"
  date: "2026-03-31"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Removed Kotlin OpticalFlowProcessor for FPS boost. Camera ~2-3fps → expected ~5fps.

- session: "0c"
  date: "2026-03-31"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Fixed VIO direction (magnetometer initial heading) and scale (VIO-ready pitch check).

- session: "0b"
  date: "2026-03-31"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Major VIO accuracy overhaul: 2D heading position, step-based scale, 7 diagnostic fields.
```

---

## CONVERSATION CONTEXT

```yaml
current_task: "Fix BUG-012 (quality=0 when moving) and BUG-013 (heading rotates when still)"
stopped_at: "Bugs documented. On-device test confirms both issues. Code fixes needed."
next_action: >
  1. FIX BUG-012: Check if is_low_light false-triggers indoors. Remove is_low_light
     from section 8 gate (line 297). Possibly raise FB_CHECK_THRESH. Log brightness.
  2. FIX BUG-013: Move is_static check BEFORE rotation update in section 9.
     When static, skip both camera and gyro rotation updates entirely.
  3. Rebuild APK and re-test: walk (quality >30%, pose_valid), stand (heading frozen).
  4. If scale/distance still off: consider MiDaS depth integration (TASK-028).
resume_context: >
  Architecture: VisionModule replaced by Tracker+Mapper+VioEngine.
  VioEngine spawns background thread for Mapper in constructor.
  Tracker handles fast path (optical flow, essential matrix, rotation fusion).
  EKF is now 1-state (scale only). Heading from global_R_ accumulation.
  Key fix: SVD condition 100-50000 = translation_degenerate (rotation valid).
  Key fix: heading = atan2(R[1][0], R[0][0]) not R[1][1] (pitch-independent).
  Key fix: FB_CHECK_THRESH = 9.0 (was 2.0), low-light quality = 0.
partial_state: "NONE — all code changes done, build passes"
warnings:
  - "BUG-012 and BUG-013 confirmed on device — need code fixes"
  - "ALL CHANGES FROM 2026-04-05 session 0f ARE UNTESTED ON DEVICE"
  - "C++ tests reference old VisionModule — need updating for Tracker/Mapper"
  - "Simulation recordings from 2026-04-05 were made with OLD code (pre-fixes)"
  - "MiDaS depth model suggested but not implemented (TASK-028)"
  - "Phase 3 of parallel VIO (WorldState 200Hz) not yet started"
```

---

## FILE MAP (key files only)

```yaml
# C++ VIO Engine (NEW architecture — post-refactor)
"app/src/main/cpp/VioEngine.h":          "Parallel orchestrator: Tracker + Mapper thread (28 lines)"
"app/src/main/cpp/VioEngine.cpp":        "processFrame, mapperThreadFunc, applyMapperResult (175 lines)"
"app/src/main/cpp/Tracker.h":            "Fast path: optical flow, essential matrix, rotation fusion (128 lines)"
"app/src/main/cpp/Tracker.cpp":          "Core VIO tracking: degenerate handling, heading, scale, pose (580 lines)"
"app/src/main/cpp/Mapper.h":             "Background: ground plane, BA, loop closure (97 lines)"
"app/src/main/cpp/Mapper.cpp":           "Ground plane (Canny+Hough), sliding window BA (LM), ORB loop closure (334 lines)"
"app/src/main/cpp/EKFState.h":           "1-state scalar Kalman filter for scale (46 lines)"
"app/src/main/cpp/EKFState.cpp":         "updateScale (Mahalanobis gate), updateZUPT, checkConsistency (79 lines)"
"app/src/main/cpp/IMUPreintegrator.h":   "IMU preintegration + step detection + motion mode (~122 lines)"
"app/src/main/cpp/IMUPreintegrator.cpp": "Gyro/accel buffering, rotation, step detector, driving mode (~465 lines)"
"app/src/main/cpp/FeatureManager.h/cpp": "Grid features, sparse replenish, keyframe store/match (~200 lines)"
"app/src/main/cpp/LensCorrector.h/cpp":  "Lens undistortion for matched points (~100 lines)"
"app/src/main/cpp/LoopClosureDetector.h/cpp": "ORB-based loop closure detection (~150 lines)"
"app/src/main/cpp/native-lib.cpp":       "JNI bridge: VioEngine lifecycle, 29-field VioData (~310 lines)"
"app/CMakeLists.txt":                    "Build: Tracker, Mapper, VioEngine, EKF, FeatureManager, LensCorrector, LoopClosure"

# Kotlin App Layer (unchanged this session)
"app/src/main/java/.../MainActivity.kt":         "Compose UI, map, radar, AR overlay (805 lines)"
"app/src/main/java/.../NavSightViewModel.kt":    "MVVM state, meters-to-LatLng (249 lines)"
"app/src/main/java/.../SensorRepository.kt":     "Sensor registration, camera dispatch, VIO init (394 lines)"
"app/src/main/java/.../NativeBridge.kt":          "JNI declarations (39 lines)"
"app/src/main/java/.../VioData.kt":               "VIO data class: 29 fields + JNI sig (~105 lines)"

# Simulation & Analysis
"simulator/simulation_data_*.json":      "8 recordings (3 pre-fix, 5 post-first-fix, all pre-latest-fix)"
"simulator/analyze_simulation.py":       "Single-file VIO vs GPS analysis + plot"
```

---

## KEY CONSTANTS (current values in Tracker.h)

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
