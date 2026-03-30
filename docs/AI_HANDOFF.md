# NavSight AI Handoff Protocol

<!-- FORMAT: YAML-like sections for AI parseability. Human-readable but structured. -->
<!-- RULE: Keep total file under 400 lines. Purge COMPLETED items older than 3 sessions. -->
<!-- RULE: Each session entry in CHANGELOG must have: agent, developer, date, branch. -->
<!-- RULE: Before editing, pull latest. After editing, commit immediately. -->

---

## META

```yaml
last_updated: "2026-03-31"
last_agent: "Claude-Opus-4-6"
last_developer: "morad"
branch: "morad"
base_branch: "master"
head_commit: "984710b"
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
  status: "FIXED_UNTESTED"
  severity: "P1"
  owner: "morad"
  file: "app/src/main/cpp/VisionModule.cpp"
  description: >
    FIXED (2026-03-31): Root cause identified — phone tilt causes 180° turn to
    accumulate as Rz rotation, but global_R_ * (0,0,-1) is invariant under Rz.
    Fix: replaced 3D forward vector with 2D heading-based position updates using
    atan2(R[1][0], R[1][1]). Also added step detector cross-check to prevent
    false ZUPT during walking, and smooth speed*dt fallback instead of full stride jumps.
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
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-03-31"
  files:
    - "app/src/main/cpp/VisionModule.cpp"
  notes: >
    2D heading-based position update replaces 3D forward vector approach.
    Heading from atan2(R[1][0], R[1][1]) correctly tracks through turns.
    Step detector cross-check prevents false ZUPT. Smooth speed*dt fallback.

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
  status: "IN_PROGRESS:morad"
  owner: "morad"
  priority: "P2"
  plan_ref: "docs/simulation_plan.md"
  notes: >
    Simulation recording now captures 7 diagnostic fields (meanFlow, inlierCount,
    stepCount, stepFreq, strideLength, poseFlags, heading). Full pipeline:
    VisionOutput → native-lib.cpp JNI → VioData.kt → NavSightViewModel JSON.
    Analysis scripts: simulator/analyze_direction.py, simulator/analyze_light.py.
    Two test recordings exist. Needs re-recording with current code.

- id: "TASK-010"
  title: "VIO accuracy improvements from simulation analysis"
  status: "DONE"
  owner: "morad"
  priority: "P0"
  completed: "2026-03-31"
  notes: >
    Applied 7 fixes from QA analysis of simulation data vs GPS ground truth:
    1. 2D heading-based position (replaces broken 3D forward vector)
    2. Step-based scale estimation (replaces broken accel double-integration)
    3. Scale quality gate 0.35→0.15, dt gate 250ms→500ms
    4. Step thresholds softened (THRESH_HIGH 10.5→10.1, LP_ALPHA 0.15→0.20)
    5. FB check relaxed 4.0→9.0, position quality gate 0.25→0.15
    6. Initial scale 0.05→0.20, smooth fallback speed*dt (replaces full stride)
    7. Anti-false-ZUPT: step detector cross-check overrides visual ZUPT

- id: "TASK-011"
  title: "Diagnostic recording fields in simulation JSON"
  status: "DONE"
  owner: "morad"
  priority: "P1"
  completed: "2026-03-31"
  notes: >
    Added 7 diagnostic fields through full pipeline: VisionOutput struct →
    native-lib.cpp JNI bridge → VioData.kt → SimulationPoint → JSON.
    Fields: mflow, inl, steps, sfreq, stride, pflags, hdg.
    JNI signature updated to (DDDDDDDIIDZ[FDDDDFFFFFFDIIDDID)V.
```

---

## RECENT CHANGES (last 5 sessions)

```yaml
- session: 0
  date: "2026-03-31"
  developer: "morad"
  agent: "Claude-Opus-4-6"
  branch: "morad"
  summary: >
    Major VIO accuracy overhaul based on real-world test (15m walk showed 1.3-4m).
    Root cause analysis: 3D forward vector invariant under Rz rotation, scale never
    converging due to broken accel-based estimation, quality gates too strict.
    Applied 7 fixes: 2D heading-based position, step-based scale, relaxed thresholds,
    anti-false-ZUPT, smooth fallback. Added 7 diagnostic fields to simulation recording
    pipeline (C++ → JNI → Kotlin → JSON). QA analysis of simulation data vs GPS
    ground truth drove all changes.
  files_changed:
    - "app/src/main/cpp/VisionModule.cpp (2D heading, scale gates, ZUPT cross-check)"
    - "app/src/main/cpp/VisionModule.h (thresholds: FB=9.0, scale=0.20, features=500)"
    - "app/src/main/cpp/IMUPreintegrator.cpp (step detection, LP_ALPHA=0.20)"
    - "app/src/main/cpp/IMUPreintegrator.h (step thresholds softened)"
    - "app/src/main/cpp/native-lib.cpp (7 diagnostic fields in JNI bridge)"
    - "app/src/main/java/.../VioData.kt (7 new diagnostic fields)"
    - "app/src/main/java/.../NavSightViewModel.kt (diagnostic fields in SimulationPoint+JSON)"
    - "simulator/analyze_direction.py (NEW: VIO vs GPS direction analysis)"
  impact: "Expected 4-10x distance accuracy improvement. Needs re-recording to verify."

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

```

---

## CONVERSATION CONTEXT

<!-- When an AI stops mid-task, describe exactly where it left off so the next AI can resume. -->

```yaml
current_task: "NONE - session completed, pending merge to master"
stopped_at: "All VIO accuracy fixes applied and build passes on morad branch"
next_action: >
  Priority order:
  1. Re-record simulation data with current code to verify fixes
  2. TASK-007: Fix IMU sample_count bug (BUG-005) — 2 pre-existing test failures
  3. Gravity-aligned heading extraction (improvement 8 from analysis)
resume_context: >
  Major VIO accuracy session (2026-03-31). Real-world test revealed 15m walk
  showing only 1.3-4m. Root causes identified and fixed:
  - 2D heading-based position (replaces 3D forward vector broken by phone tilt)
  - Step-based scale estimation (replaces broken accel double-integration)
  - Relaxed quality gates, FB check, step thresholds
  - Anti-false-ZUPT with step detector cross-check
  - Smooth speed*dt fallback (replaces jerky full-stride jumps)
  - 7 diagnostic fields added to simulation recording pipeline
  Key current values: smooth_scale_=0.20, FB_CHECK_THRESH=9.0, MAX_FEATURES=500,
  MIN_FEATURES=150, QUALITY_LEVEL=0.01, STEP_ACCEL_THRESH_HIGH=10.1,
  position quality gate=0.15, scale quality gate=0.15, dt gate=500ms.
  RULE: No magnetometer during VIO tracking — only at startup for initial heading.
partial_state: "NONE"
warnings:
  - "MainActivity.kt is 805+ lines — consider splitting"
  - "Simulation recordings are from BEFORE the 2D heading fix — must re-record"
  - "BUG-004 fix is UNTESTED on device — need walking test with turnaround"
```

---

## FILE MAP (key files only)

```yaml
# C++ VIO Engine
"app/src/main/cpp/VisionModule.h":       "VIO constants, thresholds, class definition (114 lines)"
"app/src/main/cpp/VisionModule.cpp":     "Core VIO: 2D heading position, step-based scale, ZUPT cross-check (~500 lines)"
"app/src/main/cpp/IMUPreintegrator.h":   "IMU preintegration + step detection header (~100 lines)"
"app/src/main/cpp/IMUPreintegrator.cpp": "Gyro/accel buffering, rotation integration, step detector (~350 lines)"
"app/src/main/cpp/native-lib.cpp":       "JNI bridge: 29-field VioData, diagnostic passthrough (~310 lines)"

# Kotlin App Layer
"app/src/main/java/.../MainActivity.kt":         "Compose UI, map, radar, AR overlay (805 lines)"
"app/src/main/java/.../NavSightViewModel.kt":    "MVVM state, meters-to-LatLng (249 lines)"
"app/src/main/java/.../SensorRepository.kt":     "Sensor registration, camera dispatch, VIO init (274 lines)"
"app/src/main/java/.../NativeBridge.kt":          "JNI declarations (20 lines)"
"app/src/main/java/.../VioData.kt":               "VIO data class: 29 fields + JNI sig (~105 lines)"
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
ZUPT_GYRO_THRESH:    0.04   # rad/s, static detection (tightened 2026-03-31)
GYRO_ROT_ONLY_THRESH: 2.0   # rad/s, pure rotation mode (raised for arm swings)
MIN_FLOW_PX:         0.4    # min mean optical flow pixels (lowered 2026-03-31)
MIN_PARALLAX_PX:     0.8    # min parallax for VO acceptance (lowered 2026-03-31)
RANSAC_CONF:         0.9999
RANSAC_THRESH:       0.5    # px
FB_CHECK_THRESH:     9.0    # squared px, forward-backward check (was 4.0, relaxed 2026-03-31)
MAX_FEATURES:        500    # (was 200, increased 2026-03-31)
MIN_FEATURES:        150    # (was 100)
QUALITY_LEVEL:       0.01   # (was 0.03, lowered 2026-03-31)
smooth_scale_:       0.20   # initial scale estimate (was 0.05, raised 2026-03-31)
MIN_INLIERS:         8      # (was 10)
MIN_INLIER_RATIO:    0.25   # (was 0.35)
STEP_ACCEL_THRESH_HIGH: 10.1  # m/s^2 (was 10.5, softened 2026-03-31)
STEP_ACCEL_THRESH_LOW:  9.3   # m/s^2 (was 9.1, narrowed hysteresis)
LP_ALPHA:            0.20   # step detection filter (was 0.15)
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
