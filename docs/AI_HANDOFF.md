# NavSight AI Handoff Protocol

<!-- FORMAT: YAML-like sections for AI parseability. Human-readable but structured. -->
<!-- RULE: Keep total file under 400 lines. Purge COMPLETED items older than 3 sessions. -->
<!-- RULE: Each session entry in CHANGELOG must have: agent, developer, date, branch. -->
<!-- RULE: Before editing, pull latest. After editing, commit immediately. -->

---

## META

```yaml
last_updated: "2026-03-30"
last_agent: "Claude-Opus-4-6"
last_developer: "morad"
branch: "morad"
base_branch: "master"
head_commit: "b10009b"
pr_merged: "#9 (morad → master, 2026-03-30)"
```

---

## DEVELOPERS

```yaml
morad:
  role: "IMU preintegration, sensor fusion, calibration, integration testing"
  branch: "morad"
  agent: "Claude Code"
  last_session: "2026-03-30"

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

## CODEBASE STATE

### Build Status

```yaml
android_build: "PASSES"  # Verified 2026-03-30 (gradlew externalNativeBuildDebug)
native_cpp_build: "PASSES"  # VisionModule.cpp, IMUPreintegrator.cpp, native-lib.cpp
kotlin_compile: "PASSES"  # Verified via connectedDebugAndroidTest
desktop_cpp_tests: "33/36 PASS"  # Run on SM-G998B via adb push (NDK cross-compiled)
on_device_kotlin_tests: "13/13 PASS"  # connectedDebugAndroidTest on SM-G998B
failing_tests:
  - "IMUPreintegratorTest.ZeroGyro_ProducesIdentityRotation (sample_count=0)"
  - "IMUPreintegratorTest.BufferOverflow_DoesNotCrash (sample_count=0)"
  - "DriftScenarioTest.TurnThenWalk_PositionInRotatedDirection (post-rotation recovery)"
```

### Key Branches

```yaml
master: "up-to-date, merged morad via PR #9 (2026-03-30)"
morad: "merged to master, HEAD=b10009b, all P0/P1 VIO fixes applied"
tamir-v2: "merged into morad at 746b5e3"
tamir-dev: "stale, superseded by tamir-v2"
feature/ui-redesign: "merged via PR #7"
```

---

## ACTIVE BUGS

<!-- Status: OPEN | IN_PROGRESS:<developer> | FIXED_UNTESTED | RESOLVED -->

```yaml
- id: "BUG-001"
  title: "Position drift from double-integrated accelerometer"
  status: "RESOLVED"
  severity: "P0"
  owner: "morad"
  file: "app/src/main/cpp/VisionModule.cpp"
  description: >
    FIXED: Position update now uses camera-only translation (global_R_ * scale * t_vo).
    imu_delta.deltaP is NEVER used for position. ZUPT freezes pose when static.
    Adaptive alpha fusion adjusts camera/IMU weight based on tracking quality.
  resolved_in: "Committed 660db3f, merged to master via PR #9"

- id: "BUG-002"
  title: "Scale estimation bootstrap broken"
  status: "RESOLVED"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/VisionModule.cpp"
  description: >
    FIXED: smooth_scale_ initial changed from 1.0 to 0.05. scale_obs_count_ tracks
    accepted observations. First 10 observations bypass 3x outlier rejection
    to allow bootstrap convergence. scale_obs_count_ resets in reset().
  resolved_in: "Committed 660db3f, merged to master via PR #9"

- id: "BUG-003"
  title: "ZUPT dead zone between 0.5px and 2.0px flow"
  status: "RESOLVED"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/VisionModule.cpp"
  description: >
    FIXED: ZUPT override now uses 0.5px threshold (was 2.0). Any visible flow
    overrides gyro-only static detection. MIN_FLOW_PX lowered to 1.0,
    MIN_PARALLAX_PX lowered to 2.0.
  resolved_in: "Committed 660db3f, merged to master via PR #9"

- id: "BUG-004"
  title: "Post-rotation translation recovery failure"
  status: "OPEN"
  severity: "P1"
  owner: "UNASSIGNED"
  file: "app/src/main/cpp/VisionModule.cpp"
  description: >
    After a pure rotation period (is_pure_rotation=true), the VIO engine
    cannot transition back to producing translation updates. The TurnThenWalk
    test exposes this: WalkAfterReset passes, TurnThenWalk fails. Likely
    the rotation phase corrupts tracker feature state for subsequent frames.
  test: "tests/cpp/test_drift_scenarios.cpp::TurnThenWalk_PositionInRotatedDirection"

- id: "BUG-005"
  title: "IMU preintegrator sample_count returns 0"
  status: "OPEN"
  severity: "P2"
  owner: "UNASSIGNED"
  file: "app/src/main/cpp/IMUPreintegrator.cpp"
  description: >
    integrate() returns delta.sample_count=0 in certain timestamp ranges.
    Causes 2 pre-existing test failures in test_imu_preintegrator.cpp.
  test: "tests/cpp/test_imu_preintegrator.cpp::ZeroGyro_ProducesIdentityRotation, BufferOverflow_DoesNotCrash"
```

---

## PENDING WORK

<!-- Status: TODO | IN_PROGRESS:<developer> | DONE | BLOCKED:<bug-id> -->

```yaml
- id: "TASK-001"
  title: "Implement camera-primary position update (drift fix)"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-03-30"
  notes: "Position now uses global_R_ * (scale * t_vo). IMU never used for position."

- id: "TASK-002"
  title: "Scale estimation with guards and bootstrap"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-03-30"
  notes: "estimateScaleFromAccel() has quality/dt/outlier guards. Bootstrap bypasses outlier filter for first 10 obs."

- id: "TASK-003"
  title: "Parallax gate + ZUPT dead zone fix"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-03-30"
  notes: "MIN_PARALLAX_PX=2.0, MIN_FLOW_PX=1.0, ZUPT threshold=0.5px"

- id: "TASK-004"
  title: "Enhanced SensorRadar with distance rings and cardinal labels"
  status: "DONE"
  owner: "morad"
  priority: "P2"
  completed: "2026-03-30"
  notes: "150dp radar with 1m/2m/5m rings, N/S/E/W labels, color-coded path, heading arrow"

- id: "TASK-005"
  title: "Debug Panel overlay"
  status: "DONE"
  owner: "morad"
  priority: "P2"
  completed: "2026-03-30"
  notes: "Toggle with 'D' FAB. Shows dist, speed, quality%, mode, scale, heading"

- id: "TASK-006"
  title: "Fix post-rotation translation recovery (BUG-004)"
  status: "TODO"
  owner: "UNASSIGNED"
  priority: "P1"
  files:
    - "app/src/main/cpp/VisionModule.cpp"
  notes: >
    After pure rotation period, VIO can't produce translation. Test:
    tests/cpp/test_drift_scenarios.cpp::TurnThenWalk_PositionInRotatedDirection.
    WalkAfterReset passes, so it's not a fundamental issue. Investigate how
    is_pure_rotation affects feature tracking continuity.

- id: "TASK-007"
  title: "Fix IMU preintegrator sample_count bug (BUG-005)"
  status: "TODO"
  owner: "UNASSIGNED"
  priority: "P2"
  files:
    - "app/src/main/cpp/IMUPreintegrator.cpp"
  notes: "integrate() returns sample_count=0 in certain timestamp ranges. 2 tests failing."

- id: "TASK-008"
  title: "Commit and push all uncommitted VIO changes"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-03-30"
  notes: >
    Committed as 660db3f, merged to master via PR #9 (b10009b).
    All VIO fixes, tests, and AI handoff are now on master.

- id: "TASK-009"
  title: "Simulation engine for AI-driven tuning"
  status: "TODO"
  owner: "UNASSIGNED"
  priority: "P3"
  plan_ref: "docs/simulation_plan.md"
  notes: "Depends on NavSight-Recorder app. Golden dataset not yet captured."
```

---

## RECENT CHANGES (last 5 sessions)

```yaml
- session: 1
  date: "2026-03-30"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  commits: "660db3f, b10009b"
  pr: "#9 (merged to master)"
  summary: >
    VIO engine overhaul: scale bootstrap fix, ZUPT dead zone fix, threshold tuning,
    C++ test suite (8 tests), AI handoff protocol. Resolved merge conflicts with
    tamir-v2. Committed, pushed, and merged to master via PR #9.
  files_changed:
    - "app/src/main/cpp/VisionModule.cpp"
    - "app/src/main/cpp/VisionModule.h"
    - "app/src/main/java/.../MainActivity.kt"
    - "tests/cpp/test_vision_module.cpp (NEW)"
    - "tests/cpp/test_drift_scenarios.cpp (NEW)"
    - "tests/cpp/test_utils.h (UPDATED)"
    - "tests/cpp/CMakeLists.txt (UPDATED)"
    - "docs/AI_HANDOFF.md (NEW)"
  impact: "All P0/P1 VIO fixes on master. 2 known bugs remain (BUG-004, BUG-005)."

- session: 2
  date: "2026-03-30"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  commit: "f3bd9e6"
  summary: "Restored 7-float JNI signature; wrote VIO audit plan (gemini_plan2.md)"
  files_changed:
    - "app/src/main/cpp/native-lib.cpp"
    - "docs/gemini_plan2.md (NEW)"
  impact: "JNI bridge stable. Drift fix plan documented but NOT implemented."

- session: 3
  date: "2026-03-29"
  developer: "morad"
  agent: "Claude-Code"
  branch: "morad"
  commit: "9e5e94e"
  summary: "Implemented adaptive sensor fusion (quality-based alpha weighting)"
  files_changed:
    - "app/src/main/cpp/VisionModule.cpp"
    - "app/src/main/cpp/VisionModule.h"
  impact: "Fusion weight now adapts: quality>0.7 -> alpha=0.85 (camera), quality<0.3 -> alpha=0.995 (IMU)"

- session: 3
  date: "2026-03-29"
  developer: "morad"
  agent: "Claude-Code"
  branch: "morad"
  commit: "176ec40"
  summary: "Applied Gemini Plan P0/P1: timestamp fix, intrinsics scaling, Euler extraction, bearing fusion, gyro bias clamp"
  files_changed:
    - "app/src/main/java/com/example/navsight1/SensorRepository.kt"
    - "app/src/main/cpp/native-lib.cpp"
    - "app/src/main/cpp/VisionModule.cpp"
    - "app/src/main/cpp/VisionModule.h"
    - "app/src/main/java/com/example/navsight1/NavSightViewModel.kt"
    - "app/src/main/java/com/example/navsight1/MainActivity.kt"
  impact: "P0/P1 rotation bugs fixed in code, but NOT verified on device."

- session: 4
  date: "2026-03-29"
  developer: "morad"
  agent: "Claude-Code"
  branch: "morad"
  commit: "9db0a19"
  summary: "MVVM refactor, optimized VIO engine, GPS-denied mode"
  files_changed:
    - "Major refactor across all Kotlin and C++ files"
  impact: "Architecture changed to MVVM. SensorRepository, ViewModel, MainActivity restructured."

```

---

## CONVERSATION CONTEXT

<!-- When an AI stops mid-task, describe exactly where it left off so the next AI can resume. -->

```yaml
current_task: "NONE - session completed"
stopped_at: "All changes committed and merged to master via PR #9"
next_action: >
  Priority order:
  1. TASK-006: Fix post-rotation translation recovery (BUG-004) — a real bug
  2. TASK-007: Fix IMU sample_count bug — 2 pre-existing test failures
  3. TASK-009: Simulation engine (future)
resume_context: >
  All VIO fixes committed (660db3f) and merged to master via PR #9 (2026-03-30).
  Key changes on master:
  - Camera-primary position (no IMU deltaP for position)
  - Scale bootstrap with 10-observation bypass
  - ZUPT dead zone eliminated (0.5px threshold)
  - MIN_FLOW_PX=1.0, MIN_PARALLAX_PX=2.0
  - Enhanced radar with distance rings, cardinal labels, heading arrow
  - Debug panel with toggle FAB
  - Full test suite: 13 Kotlin on-device + 36 C++ tests (cross-compiled via NDK)
  - C++ tests run on device via: adb push build_android/navsight_tests /data/local/tmp/
  Merge conflicts with tamir-v2 resolved: kept morad's VIO overhaul, added tamir's imports.
partial_state: "NONE"
warnings:
  - "MainActivity.kt is 805+ lines — consider splitting"
  - "tests/cpp/build_android/ directory contains cross-compiled binaries, add to .gitignore"
  - "TurnThenWalk test exposes real post-rotation recovery bug (BUG-004)"
```

---

## FILE MAP (key files only)

```yaml
# C++ VIO Engine
"app/src/main/cpp/VisionModule.h":       "VIO constants, thresholds, class definition (106 lines)"
"app/src/main/cpp/VisionModule.cpp":     "Core VIO: optical flow + IMU fusion, global pose (425 lines)"
"app/src/main/cpp/IMUPreintegrator.h":   "IMU preintegration header (73 lines)"
"app/src/main/cpp/IMUPreintegrator.cpp": "Gyro/accel buffering, rotation/position integration (315 lines)"
"app/src/main/cpp/native-lib.cpp":       "JNI bridge: calls VisionModule, extracts Euler angles (287 lines)"

# Kotlin App Layer
"app/src/main/java/.../MainActivity.kt":         "Compose UI, map, radar, AR overlay (805 lines)"
"app/src/main/java/.../NavSightViewModel.kt":    "MVVM state, meters-to-LatLng (249 lines)"
"app/src/main/java/.../SensorRepository.kt":     "Sensor registration, camera dispatch, VIO init (274 lines)"
"app/src/main/java/.../NativeBridge.kt":          "JNI declarations (20 lines)"
"app/src/main/java/.../VioData.kt":               "VIO data class + JNI signature (88 lines)"
"app/src/main/java/.../OpticalFlowProcessor.kt":  "Kotlin-side optical flow (439 lines)"
"app/src/main/java/.../NavigationManager.kt":     "Turn-by-turn nav (495 lines, out-of-scope extension)"
"app/src/main/java/.../DeviceOrientationTracker.kt": "Accel+mag orientation for UI (219 lines)"

# Tests
"tests/cpp/test_imu_preintegrator.cpp":  "10 IMU preintegration tests (8 pass, 2 fail on sample_count)"
"tests/cpp/test_vision_module.cpp":      "14 VisionModule tests (all pass) — includes P0 robustness tests"
"tests/cpp/test_drift_scenarios.cpp":    "12 drift scenario tests (11 pass, TurnThenWalk fails)"
"tests/cpp/test_utils.h":               "Synthetic frame generators (checkerboard NV21, dot patterns, feature grids)"
"tests/cpp/CMakeLists.txt":             "Cross-compile with NDK: cmake -G Ninja -DCMAKE_TOOLCHAIN_FILE=ndk/..."
"app/src/androidTest/.../VioNativeTest.kt": "13 on-device JNI integration tests (all pass on SM-G998B)"
"app/src/test/.../VioDataTest.kt":       "9 JVM unit tests for VioData (all pass)"

# Plans
"docs/gemini_plan.md":     "P0/P1 rotation fix plan (APPLIED, untested)"
"docs/gemini_plan2.md":    "Drift reduction + UI plan (NOT YET IMPLEMENTED)"
"docs/simulation_plan.md": "Sim engine for AI tuning (NOT YET IMPLEMENTED)"
```

---

## KEY CONSTANTS (current values in VisionModule.h)

```yaml
ALPHA_FUSION:        0.98    # gyro weight (overridden by adaptive fusion)
ZUPT_GYRO_THRESH:    0.05    # rad/s, static detection
GYRO_ROT_ONLY_THRESH: 0.5   # rad/s, pure rotation mode
MIN_FLOW_PX:         1.0    # min mean optical flow pixels (was 2.0, lowered 2026-03-30)
MIN_PARALLAX_PX:     2.0    # min parallax for VO acceptance (was 5.0, lowered 2026-03-30)
RANSAC_CONF:         0.9999
RANSAC_THRESH:       0.5    # px
FB_CHECK_THRESH:     1.0    # squared px, forward-backward check
MAX_FEATURES:        200
MIN_FEATURES:        100
smooth_scale_:       0.05   # initial scale estimate (walking)
```

---

## LOCK TABLE

<!-- Prevents two developers from editing the same file simultaneously. -->
<!-- Format: file -> developer:timestamp. Expires after 4 hours. -->
<!-- Before editing a file, check this table. If locked by another dev, coordinate. -->

```yaml
locks: {}
# Example:
# "app/src/main/cpp/VisionModule.cpp": "morad:2026-03-30T14:00:00Z"
```

---

## PROTOCOL

### For the starting AI agent:
1. Read this entire file first.
2. Check ACTIVE BUGS -- do not duplicate work on IN_PROGRESS items.
3. Check LOCK TABLE -- do not edit locked files.
4. Check CONVERSATION CONTEXT for any partial work to resume.
5. Before starting, set your task status to IN_PROGRESS with your developer name.
6. Lock any files you will edit in the LOCK TABLE section.

### For the finishing AI agent:
1. Update ACTIVE BUGS statuses (OPEN -> FIXED_UNTESTED -> RESOLVED).
2. Move completed PENDING WORK to DONE.
3. Add a session entry to RECENT CHANGES (keep only last 5).
4. Update CONVERSATION CONTEXT with where you stopped and what to do next.
5. Update META section (last_updated, last_agent, last_developer, head_commit).
6. Release all LOCK TABLE entries for your developer.
7. Commit this file with message: "handoff: <brief summary>"

### Conflict resolution:
- If two agents edit this file simultaneously, the later commit must manually merge.
- LOCK TABLE is advisory -- check git log to see if someone else committed recently.
- When in doubt, use smaller, more frequent commits to reduce merge conflicts.

### Purge policy:
- Remove RESOLVED bugs after 2 sessions.
- Remove DONE tasks after 3 sessions.
- Keep only 5 entries in RECENT CHANGES.
- Archive old entries to docs/handoff_archive.md if needed.
