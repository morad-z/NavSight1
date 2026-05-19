# NavSight AI Handoff — 2026-05-16

## Current state in one sentence

**v22 (commit `070a69c`, walked 1.17 m close-loop on 109 m) is still the project HEAD. All of May 12-16's work is uncommitted on the working tree. Multiple critical fixes shipped today; trajectory still produces heading drift + SLAM-dot slide; user has lost confidence in the codebase ("spaghetti, nothing is correct").**

## Decision pending

User asked for one of four paths. Awaiting decision:
- **(A)** Reset to v22, cherry-pick only validated fixes (2-3 hrs work, lose ~4 days of mixed-quality changes)
- **(B)** Refactor `Tracker.cpp` (2700-line monolith) into named modules + kill `global_R_/scalar_heading_/global_t_` (1-2 weeks)
- **(C)** Replace custom VIO with OpenVINS or VINS-Fusion (2-4 weeks)
- **(D)** Empirical "what works" audit before deciding A/B/C (1 day)

My recommendation: **(A) reset to v22, then decide between (B) and (C)**.

---

## Why we got here — chronology of post-v22 work

### v22 (2026-05-11)
Best known walk: 1.17 m close-loop on 109 m, low_score 184→4, corrections 6→49. Plan Steps 0-3 shipped + Z-up frame migration. Committed.

### 2026-05-12 — SLAM anchoring debug
Symptom: orange SLAM dots not anchoring during walks; LC giving 150° rotation residuals. Found `updateRelativeRotation` (EKFState.cpp:946-980) had been missed in the 2026-05-09 Option-C migration. Shipped fix uncommitted. Build green; not real-walk validated.

### 2026-05-13 — Step 5 pose-graph + Step 7.1 fixes
- Added `PoseGraph.{h,cpp}` (4-DOF GN optimizer with R_z(ψ) Jacobian)
- Wired LC accept → addLoopEdge → optimize → back-write to LC DB
- Step 7.1: ORB descriptor verification in `tryDetectLoopGeometric` (Hamming ≤ 50)
- Heading-startup fix: `seedMadgwickYaw` early in `SensorRepository.kt`
- `target_yaw` frame-convention fix at `Tracker.cpp:4435` (atan2(R[0,1], R[0,0]) with nav-CW negation)
- v25 walk validated: close-loop 1.92 m on 114 m (regression from v22's 1.17 m)

### 2026-05-14 / 2026-05-15
v25/v26 walks. v26 catastrophically wrong: 7.36 m close-loop on 109 m, max-radius 14.7 m vs GPS 24 m (40% smaller), heading "overturning" 2×. New `EKF_INIT_TILT` instrumentation showed `up_body=(0,0,1)` at EKF init — EKF was initializing with phone-FLAT assumption while user holds phone VERTICAL.

### 2026-05-16 (today) — ruflo swarm code review + fix sprint

**Phase 1 — 7-agent review** (results in `docs/review_2026_05_16/`):
- `architecture_audit.md` (28 findings: 6 HIGH/9 MED/13 LOW) — 4 yaw extraction formulas, heading in 4 places, `Tracker::processFrame` 2700 lines / 14 sections
- `ekf_audit.md` (15 findings: 2 CRITICAL) — `setPosition(global_t_)` per-frame collapses P_pp; Phi velocity-theta block wrong sign+order per Forster 2017 TRO A22
- `orientation_audit.md` (15 findings: 1 CRITICAL) — pure-yaw `Rz(azimuth)` fallback in EKF init still reachable; 5 HIGH orientation bugs each capable of trajectory orbiting independently
- `domain_model.md` (24 findings) — 8 cross-domain reach-throughs; 5 conflated concepts (`global_R_` has 3 semantics in one variable)
- `silent_failures.md` (18 findings: 2 CRITICAL) — Cholesky+SVD silent return drops MSCKF update; BA singular landmark silently replaced with `1e6*I`
- `bug_patterns.md` (11 families, 8 recurring ≥3 instances) — F1 frame-convention (9+ instances), F2 stale mirror (5), F3 partial migration (3 multi-week events)
- `observability_audit.md` — `MSCKF_DX_BLOCKS` writes 2.76 MB to 256-512 KB Samsung logd buffer per walk; LC tag breaks `adb logcat -s NavSight-*`

**Phase 2 — 4-agent fix sprint** (results in `docs/fix_logs/`):
1. **EKF math (`EKFState.cpp/h`)**:
   - Phi `dv_dtheta` and `dp_dtheta` sign+order fixed per Forster 2017 TRO A22
   - Q preintegration cross-terms restored (was 3 diagonal blocks only; now full 9)
   - P_bc cross-cov zeroed after propagateIMU (decouples disabled δφ_bc consistently)
   - H_td unit fix (was /slam_fx_, now pixels/s)
   - SLAM init σ_uv uses `slam_fx_` not hardcoded 500
   - **IMU σ values REVERTED** to original UNCALIBRATED state. Agent shipped my-recall numbers under fabricated Geneva 2020 §V-B citation; I reverted same session and added explicit TODO comment requiring Allan-variance characterization

2. **Position autonomy + heading SSOT (`Tracker.cpp/h` + `EKFState.h/cpp`)**:
   - Per-frame `ekf_.setPosition(global_t_)` REMOVED before propagateIMU — fixes P_pp covariance collapse (audit Finding 1, the orbiting-origin root cause)
   - Direction reversed: EKF authoritative for p_G, Tracker mirrors after MSCKF+SLAM+LC updates
   - `EKFState::getWorldHeadingRad()` added as single canonical yaw extraction
   - `Tracker.cpp:777` EKF init now defers when `imu.getRotationGtoI()` empty (adds `ekf_init_deferred_madgwick_not_ready` counter), removes pure-yaw fallback
   - `setInitialHeading` post-init queues azimuth into `pending_post_init_azimuth_` instead of silently dropping when Madgwick not ready
   - `global_R_` and `scalar_heading_` marked migration-status but NOT YET DELETED (some legacy reads remain)

3. **Silent-failure counters + `[[nodiscard]]` (multiple files)**:
   - 16 new EventCounters added: `applyMSCKFUpdate_inversion_failed`, `ba_singular_landmarks`, `propagate_skipped` + 3 sub-reasons, `msckf_chi2_rejected`, `msckf_inversion_failed`, `velocity_clamped`, `scale_chi2_rejected`, `time_offset_clamped`, `fej_clone_miss`, `gravity_alignment_rejected`, `relative_rotation_rejected`, `loop_closure_rejects_heading_total`, `loop_closure_rejects_pnp_total`, `ekf_init_deferred_madgwick_not_ready`, `zrup_fired_total`
   - `[[nodiscard]]` on 7 EKF bool-returning update methods
   - `LoopClosureDetector.cpp` TAG renamed `"LoopClosureDetector"` → `"NavSight-LC"`

4. **Android races (Kotlin)**:
   - `madgwickYawSeeded` flags now reset in `resetAll` (fixes regression on second session)
   - `runBlocking` on depth executor replaced with coroutine
   - `@Volatile` on `depthProcessing`
   - `wasVioInitialized` → `AtomicBoolean.compareAndSet` (fixes TOCTOU producing v26's "3 EKF re-inits")
   - `synchronized(simulationDataPoints) { clear() }` (eliminates ConcurrentModificationException risk)
   - KDoc stride fix on `getSlamSnapshot` (4→7)
   - CameraUi analyzer executor → `DisposableEffect` cleanup

**Phase 3 — ZRUP (Zero-Rotation-Rate Update)**:
- User reported heading drifting + SLAM dots sliding top-right when phone held still
- Diagnosed: gyro-bias-driven R_GtoI drift during stationary periods. EKF `b_g_` only updates from MSCKF visual measurements which don't fire when stationary (no parallax). Existing `IMUPreintegrator::refineGyroBiasDuringZUPT` was structurally dead — overwritten by EKF→Madgwick feedback loop every frame.
- Shipped: `EKFState::updateZRUP(mean_gx, mean_gy, mean_gz, sigma, N)` — dual of ZUPT, observes `mean(gyro_window) - b_g_ = 0` with H = -I on rows 3:6
- Wired into Tracker.cpp ZUPT trigger site
- `zrup_fired_total` counter added
- **NOT walk-validated yet**

---

## What's actually shipped tonight, NOT validated

Build green, APK installed, but no validating walk:
- Phi skew sign+order (Forster A22)
- Position autonomy (no more setPosition collapse)
- `getWorldHeadingRad` centralization
- EKF init defer-until-Madgwick-ready
- ZRUP measurement update
- 16 silent-failure counters
- 5 Android race-condition fixes
- target_yaw frame fix (May 13)
- dyaw_loop source change (May 13)
- Step 7.1 descriptor verification
- Step 5 back-write enabled

## What's NOT in tonight's build

- Step 5 plan-spec'd Σ_odom from EKF clone covariance, Σ_loop from var_p_total, ε = trace(Σ)/1000 — math implemented but quality unproven
- IMU σ values (`sigma_g_=0.01, sigma_a_=0.1, sigma_bg_=0.0001, sigma_ba_=0.001`) — UNCALIBRATED, explicitly flagged. Real fix requires 2-4 hr stationary Allan-variance capture + analysis script
- `global_R_` / `scalar_heading_` legacy variables (marked, but reads not all migrated)
- `native-lib.cpp:246/467` `g_yaw` Y-up formula on Z-up matrix (known wrong, flagged, not fixed)

## Files modified since v22 (29 files uncommitted)

```
M app/CMakeLists.txt
M app/src/main/cpp/EKFState.cpp
M app/src/main/cpp/EKFState.h
M app/src/main/cpp/EventCounters.h
M app/src/main/cpp/FeatureManager.cpp
M app/src/main/cpp/FeatureManager.h
M app/src/main/cpp/IMUPreintegrator.cpp
M app/src/main/cpp/IMUPreintegrator.h
M app/src/main/cpp/InertialInitializer.cpp
M app/src/main/cpp/InertialInitializer.h
M app/src/main/cpp/LoopClosureDetector.cpp
M app/src/main/cpp/LoopClosureDetector.h
M app/src/main/cpp/PoseGraph.cpp
M app/src/main/cpp/PoseGraph.h
M app/src/main/cpp/Tracker.cpp                 ← biggest delta, 2700-line god-object
M app/src/main/cpp/Tracker.h
M app/src/main/cpp/UpdaterMSCKF.cpp
M app/src/main/cpp/VioEngine.cpp
M app/src/main/cpp/VioEngine.h
M app/src/main/cpp/WindowedBA.cpp
M app/src/main/cpp/native-lib.cpp
M app/src/main/java/com/example/navsight1/CameraUi.kt
M app/src/main/java/com/example/navsight1/NativeBridge.kt
M app/src/main/java/com/example/navsight1/NavSightViewModel.kt
M app/src/main/java/com/example/navsight1/SensorRepository.kt
M tests/cpp/CMakeLists.txt
M tests/cpp/test_relative_rotation.cpp
+ many untracked files (audit findings, fix logs, sim recordings, scripts)
```

## Symptoms still unresolved

1. **Heading slowly rotating when phone held still** — ZRUP shipped to address; not walk-validated
2. **SLAM orange dots drifting top-right when stationary** — same root cause as #1 (gyro bias → R_GtoI drift → anchor clone drift); ZRUP should fix
3. **Speed display "not correct"** — uninvestigated. Likely related to position-autonomy direction reversal (Tracker now mirrors EKF after updates; speed = derivative of EKF position which may be on a different cadence than before)
4. **Trajectory orbits origin on multi-loop walks** — setPosition fix shipped today; not walk-validated
5. **24% path undershoot vs GPS** — Phi skew fix shipped today; not walk-validated

## What the next AI / next session should know

1. **Read these audit files BEFORE writing any code**:
   - `docs/review_2026_05_16/architecture_audit.md` (top-3 leverage points)
   - `docs/review_2026_05_16/ekf_audit.md` (math errors with citations)
   - `docs/review_2026_05_16/orientation_audit.md` (frame conventions)
   - `docs/review_2026_05_16/domain_model.md` (single-source-of-truth violations)
   - `docs/review_2026_05_16/silent_failures.md` (silent gates)
   - `docs/review_2026_05_16/bug_patterns.md` (11 recurring families + skill amendments)

2. **The plan in `docs/study/post_v19_sprint_plan.md` is correct** — Steps 4-7 implementation just got tangled. The plan itself remains the path forward.

3. **The implementor skill at `.claude/skills/navsight-implementor/SKILL.md` is the binding contract**. The user has repeatedly enforced its anti-patterns (no defensive clamps, no magic numbers, no symptom gates). Honor it from the start.

4. **The user holds the phone vertical with screen facing stomach** (camera forward). NOT phone-flat. Many bugs in this codebase historically assumed phone-flat. Always test rotation formulas at multiple non-identity orientations.

5. **Samsung Galaxy S21 Ultra (SM-G998B), Exynos 2100, Mali-G78 GPU**. Per `project_target_device.md` memory entry.

6. **The user works on `morad` branch, never master.** Commits to morad, then merges to master.

7. **The user has explicit feedback in memory** that should be followed automatically:
   - `feedback_explicit_commit_only` — never auto-commit
   - `feedback_no_magnetometer` — no continuous mag fusion
   - `feedback_no_deletions` — comment out, don't `rm`
   - `feedback_no_disabling` — fix root cause, not behind a flag
   - `feedback_no_metric_celebration` — don't celebrate close-loop drops when symptoms persist
   - `feedback_follow_plan_rules` — code written ≠ step closed
   - `feedback_branch_workflow` — morad first, then merge

## Recommended next action

1. **Preserve audit findings + fix logs to a branch** (so v22 reset doesn't destroy them):
   ```bash
   git checkout -b audit-2026-05-16
   git add docs/review_2026_05_16/ docs/fix_logs/ docs/AI_HANDOFF_2026_05_16.md
   git commit -m "docs: preserve 2026-05-16 audit findings + fix logs + AI handoff"
   git checkout morad
   ```

2. **Then** decide A/B/C/D and proceed. If A (reset to v22):
   ```bash
   git checkout -- .  # discards uncommitted code changes
   # docs/AI_HANDOFF and audit files survive because they're on the audit branch
   ```

3. **Whichever path is chosen, before re-introducing code from this sprint**, walk v22 once to confirm it still matches its prior 1.17 m close-loop. If yes, that's the trusted baseline.
