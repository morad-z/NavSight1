# NavSight Production-Readiness Plan

**Status**: draft
**Owner**: Morad
**Scope**: GPS-denied pedestrian + vehicle (scooter) indoor/outdoor navigation
**Constraint**: GPS must NOT be used for scale, heading, or pose — the entire value proposition is GPS-denied operation. GPS may be used only for (a) global map registration when the user opts into an outdoor "warm-up" phase and (b) ground-truth logging in the replay harness. It must not be inside any state estimator in the hot path.

## Guiding Principles

1. **No shortcuts.** Every step in this plan is a full implementation. No "we'll patch this later," no "good enough for the demo," no stub methods with TODOs. If a step can't be fully implemented now, it is removed from the plan, not downgraded.
2. **One source of truth per quantity.** Orientation lives in exactly one place. Position lives in exactly one place. Scale lives in exactly one place. Duplicate state (`scalar_heading_` vs `global_R_`, Tracker's pose vs EKF's pose) is the root cause of every bug in this repo and is to be eliminated by design, not by careful synchronization.
3. **Covariance is mandatory.** Every estimate emitted to the UI carries an uncertainty. If a subsystem cannot produce a covariance, it is a heuristic, not an estimator, and cannot feed the filter.
4. **Replay before re-flash.** Any change to the estimator is first validated on the full `tests/sims/` corpus via an offline replay harness that runs the native pipeline against recorded sensor data, and only then flashed to a device. "Build and walk around" is debugging, not testing.
5. **Magic numbers are bugs.** The current code has ~40 hand-tuned thresholds. Each one that survives is documented with its physical meaning, its sensitivity, and the noise-density source it derives from. Thresholds that are not derivable from IMU noise density or measured statistics are deleted.
6. **Dead code is deleted.** Mapper, disabled MSCKF blocks, commented-out methods, and "future work" scaffolding are removed. Git preserves history.

---

## Step 1 — Attitude Filter (Madgwick IMU-only)

**Goal**: replace the 1-D `scalar_heading_ += yaw_rate * dt` path with a proper SO(3) attitude filter that fuses gyro (for dynamics) and accel (for roll/pitch drift correction). This is the fix for the V-shape, the rotating-in-place rotation under-count, the 290° gap between `scalar` and `old_R`, and every future bug that would have come from misaligning a scalar state with a rotating phone.

### Why Madgwick specifically

- IMU-only variant needs no magnetometer (satisfies the "no mag during tracking" memory rule).
- O(1) per sample, no matrix inversions, ~150 lines.
- Single tunable (`beta` — gyro-vs-accel trust) derivable from gyro noise density.
- Gyro drives short-term yaw integration directly (no projection onto a noisy gravity estimate), so fast turns are captured exactly.
- Accel corrects *only* roll/pitch over time, never yaw — which is physically correct (you cannot observe yaw from gravity alone).

### Full implementation plan (no placeholders)

1. **State** (`IMUPreintegrator.h`):
   - `double q0_, q1_, q2_, q3_` — unit quaternion, Hamilton convention, body→world, initialized to identity.
   - `int64_t madgwick_last_ns_` — timestamp of last update.
   - `bool madgwick_init_` — false until first accel sample has been used to set initial roll/pitch.
   - `constexpr double MADGWICK_BETA` — gyro drift correction gain. Derived from phone gyro noise density ≈ 0.01 rad/s/√Hz → β ≈ √(3/4) · 0.01 ≈ 0.009 rad/s. Round to 0.033 (classic Madgwick default) and revisit only after replay harness shows heading drift > 2°/min.
2. **Update cycle**:
   - Called from `addGyroReading` under `mutex_` after `last_gx/gy/gz` are stored.
   - First call: initialize quaternion from `last_ax/ay/az` using closed-form roll = atan2(ay, az), pitch = atan2(−ax, √(ay² + az²)), yaw = 0. Mark `madgwick_init_ = true` and return.
   - Subsequent calls: compute `dt` from `madgwick_last_ns_`, clamp to (0, 0.5] s; run the standard Madgwick IMU update with bias-subtracted gyro and unit-normalized accel; integrate `q += qDot * dt`; renormalize.
   - Accel correction is skipped when `|a| < 5 m/s²` (freefall / strong linear accel) or `|a| > 20 m/s²` (impact) — during those windows the filter runs gyro-only for that sample. This is not a shortcut, it's the correct Madgwick recipe.
3. **Outputs**:
   - `getOrientationQuaternion(w, x, y, z)` — const, mutex-locked.
   - `getHeading()` — returns yaw extracted as `atan2(2(q0q3 + q1q2), 1 − 2(q2² + q3²))` and **negated** to match the existing CW-positive navigation convention, so the rest of the pipeline is unchanged.
   - `getRoll()`, `getPitch()` — exposed because the keyframe heading correction in Tracker.cpp will need them to compute the proper gravity-aligned yaw change from an essential-matrix rotation (that fix is Step 2).
4. **Reset**:
   - Extend `IMUPreintegrator::reset()` to zero `q0_=1, q1_=q2_=q3_=0`, clear `madgwick_last_ns_`, set `madgwick_init_ = false`.
5. **Integration into Tracker** (`Tracker.cpp`):
   - Delete the `scalar_heading_ += yaw_rate * imu_delta.dt` block (lines ~740–778).
   - Replace with `double heading = imu.getHeading();`
   - `scalar_heading_` field itself is deleted along with `heading_fej_` (Madgwick's quaternion is the FEJ-stable reference).
   - `global_R_` is no longer updated from `imu_delta.deltaR` either. For display/output it is derived on demand from the Madgwick quaternion converted to a 3×3 matrix. This removes the `scalar` vs `old_R` dual-tracking entirely.
6. **HEADING log line**:
   - Replace with `HEADING: yaw=%.1f° roll=%.1f° pitch=%.1f° (Madgwick)` so that during debugging we see the full attitude, not a projected scalar.
7. **Tests** (`tests/cpp/test_imu_preintegrator.cpp` extension, not a new file):
   - Unit test 1: static phone, 10 s of (0,0,9.81) accel + zero gyro → yaw drift < 0.1°.
   - Unit test 2: 180° rotation about vertical axis over 2 s with simulated gyro only → final yaw = ±180° ± 1°.
   - Unit test 3: 180° rotation with 10% gaussian accel noise (centripetal) → final yaw = ±180° ± 2°. **This is the V-shape regression test.**
   - Unit test 4: freefall window (accel = 0) during rotation → filter continues from gyro only, no divergence.
8. **Acceptance criteria**:
   - Unit tests 1–4 all pass in CI.
   - Replay harness on `tests/sims/*.json` shows heading error on every 180° turn recording < 5° (previously 30–46°).
   - On-device walk test: walk 5 m forward, turn 180°, walk back — the two legs overlap within 0.5 m RMS.

### What Step 1 does NOT do (and why that's OK)

- It does not change how position is integrated. Position still uses `t_vo * scale` and PDR fallback. That's Step 2 and Step 3.
- It does not fix stride-length estimation. Steps 4.
- It does not fix the rotating-in-place phantom steps at the counter level — that was already done in today's fix and is orthogonal.

---

## Step 2 — Visual Odometry without a Scalar Heading

**Goal**: the keyframe heading correction (Tracker.cpp:922) and the essential-matrix pose recovery stop being special-cased for a scalar heading and start working in the full attitude framework.

1. **Keyframe visual heading**: replace `atan2(R_kf[1,0], R_kf[0,0])` with the proper gravity-aligned yaw change. Given `R_kf` from the essential matrix and the current roll/pitch from Madgwick, rotate `R_kf` into the gravity-aligned frame first, then extract yaw. This eliminates the original root cause of the V-shape (scalar correction fighting gyro during turns) *at the correction source*, not via a threshold gate.
2. **Drop the `gyro_norm < 0.3` gate** added in the prior session. With the correction now computing a physically meaningful gravity-aligned yaw, it can fire during turns without subtracting real rotation.
3. **Delete `heading_fej_` and the FEJ heading lock**: Madgwick's quaternion is the reference now. FEJ for position only, not heading.
4. **Expose keyframe visual yaw covariance**: Ceres-style or analytic from RANSAC inlier count. Used in Step 6's ESKF update.

Acceptance: walking a 20 m corridor with four 90° turns returns to within 1 m of start in the replay harness.

---

## Step 3 — Scale Estimation in a GPS-Denied World

**Goal**: produce a metric scale factor with a covariance, without ever touching GPS in the hot path. Three parallel observers, each with an honest variance, fused in the EKF.

### Observer A — Pedestrian Dead Reckoning (PDR)

Current stride model is `height × 0.415 × freq_factor`. Replace with per-user learned stride.

1. **Stride learning mode**: on first app run, the user walks a known distance (configurable, default 10 m indoors marked by two physical landmarks the user points the phone at). The pipeline records step count and computes `stride = distance / steps`. Stored in `SharedPreferences`, exposed via `setUserStride()`.
2. **Per-session stride refinement**: every time VIO triangulated scale is high-confidence (many inliers, stable smooth_scale for >30 frames), record the implied stride and update a running mean with a forgetting factor. Never overwrite the calibration baseline, only add a session offset.
3. **PDR confidence**: derived from step-period variance (uniform gait = high), accel magnitude variance (low vibration = high), and elapsed time since the last confirmed step.
4. **Explicit rejection of PDR during rotation**: done in the prior session fix. Keep that, but move the threshold (0.8 rad/s) to a `constexpr` named `ROTATION_STEP_GATE_RADPS` with a comment linking to the physical justification (46°/s ≫ walking 10°/s arm swing).

### Observer B — Monocular Depth Scale Constraint (MiDaS)

Currently wired but not firing. Full fix:

1. **Log every call**: add `LOGI("DEPTH_SCALE: entry pts3d=%zu pts2d=%zu depth_empty=%d", ...)` at the top of `applyDepthScaleConstraint` so any future regression is visible in logcat.
2. **Log every bailout reason**: separate log lines for "no depth map", "too few points", "camera height out of range", "pitch out of range", "no floor features in lower 40%".
3. **Relax the floor-features filter** from hardcoded "lower 40%" to "lower 40% OR any feature whose reprojected Z (using current gravity) is below the phone height" — handles the scooter case where the camera looks forward but the road is still geometrically below phone height.
4. **Confidence** = median absolute deviation of sampled depth ratios, inversely weighted into the EKF update.
5. **Kalibration step**: first run calibrates MiDaS relative→metric ratio against PDR scale during a stride-learning walk. Stored alongside stride length.

MiDaS is the *only* scale source available on a scooter in GPS-denied areas. It must work. This step stays in the plan at full fidelity.

### Observer C — Gravity-Aided Visual-Inertial Scale (IMU preintegration)

This is the OpenVINS approach and it is the production answer for vehicular motion without GPS.

1. **Between two keyframes**, IMU preintegration produces `ΔP_IMU` in world frame (up to initial velocity). Visual triangulation between the same two keyframes produces `Δp_VIS` up to scale. Stacking several keyframe pairs gives an overdetermined linear system whose unknowns are scale `s`, initial velocity `v0`, and optionally a gravity-direction refinement.
2. **Solve via closed-form least squares** every N keyframes (N = 10). This is Hesch / Martinelli's classic VI bootstrap.
3. **Output**: scale + covariance.
4. **Why this wasn't in the current code**: it was disabled in Phase 8 because "IMU preintegration deltaP is dominated by gravity subtraction errors." That disable is only valid because the attitude was wrong (Step 1 fixes that) and the gravity was noisy. Once Madgwick gives a clean world-frame attitude, the gravity subtraction becomes accurate and this path works.
5. **Scooter requirement**: this is the observer that makes scooter mode actually produce metric scale. No shortcuts — full LS solver, real covariance, re-enabled.

### Fusion

The three observers each emit `(scale_obs, variance)` and feed a 1-D Kalman update on `smooth_scale_`. No cascading fallback (`if (A) else if (B) else if (C)`). Every observer runs every frame; their updates are weighted by variance. Dead observers automatically fade out.

---

## Step 4 — Single State Estimator

**Goal**: delete `global_R_`, `global_t_`, `scalar_heading_` ownership in Tracker. The one estimator is an error-state Kalman filter (ESKF) on `[p, v, q, b_g, b_a]` in world frame, predicted by IMU preintegration, updated by:

- VIO relative pose (from Tracker's essential-matrix path — converted to world-frame pose delta using the current state).
- ZUPT as `v = 0` pseudo-measurement with small variance.
- MiDaS scale as a pseudo-measurement on `‖v‖ · dt`.
- Visual-inertial scale observer as a pseudo-measurement on the integrated `‖Δp‖`.
- PDR step as a pseudo-measurement on displacement magnitude over step_period.

Tracker's remaining responsibility: take a gray frame, produce (relative R, relative t, inlier count, feature observations) and hand it to the EKF. Nothing else. All global pose lookups in the codebase go through `EKFState::getPose()`.

This is the step that kills the duplicate-state family of bugs permanently.

Acceptance: the replay harness runs all of `tests/sims/*.json`, reports heading error, position drift, and final loop-closure gap. Every metric improves over Step 2. No sim regresses.

---

## Step 5 — Calibration & Initialization

**Goal**: every session starts from a known, validated state. No more "it didn't work because the app wasn't warmed up."

1. **Startup gate**: 5 s of stationary sensor data with `|gyro| < 0.02 rad/s` and accel variance < 0.01 m²/s⁴. Used to:
   - Estimate gyro bias.
   - Estimate accel bias.
   - Initialize gravity direction (Madgwick initial quaternion).
   - Read magnetometer once for the initial yaw (per the "mag only at startup" memory rule).
2. **Failure handling**: if the gate does not pass within 15 s, show a user-facing "Place phone flat for 5 s" dialog. No silent fallback.
3. **Stride calibration**: separate one-time flow, see Step 3 Observer A.
4. **MiDaS calibration**: baked into the stride walk, see Step 3 Observer B.
5. **Stored**: all calibration values in `SharedPreferences`, versioned so a future schema change can migrate.

---

## Step 6 — Covariance-Aware UI

**Goal**: the user sees when the system is confident and when it is lost. A V-shape that the user cannot distinguish from a correct path is worse than a V-shape the user knows is wrong.

1. **Pose covariance** exported from EKF to the Kotlin side every 4 Hz.
2. **Map UI**: an uncertainty ellipse around the user marker, scaled from the (x, z) covariance block.
3. **Status chip**: "GPS-DENIED — VIO ACTIVE (σ = 0.8 m)" / "VIO DEGRADED" / "VIO LOST — WALK FORWARD TO RE-ACQUIRE".
4. **Radar trajectory**: points colored by covariance trace. A V-shape from bad tracking would show up red.
5. **Crash/error recording**: every session dumps sensor JSON + covariance trace to `<external-files>/crash_logs/` on unexpected termination. Existing sim recording path is formalized to also fire on every crash, not just on user request.

---

## Step 7 — Replay Harness + CI

**Goal**: no estimator change is merged without replay results on the full sim corpus.

1. **`tests/cpp/replay_harness.cpp`** (new file): reads a simulation JSON (format already used by the Android app), streams it into the C++ pipeline at recorded timestamps, and emits a per-frame pose + covariance CSV.
2. **`tests/cpp/replay_scorer.py`** (new file): reads the CSV, compares to ground truth (embedded in the JSON if recorded with GPS, or against hand-annotated keyframes), emits metrics:
   - Heading RMSE
   - Position drift per meter traveled
   - Loop closure gap (if the path closes)
   - V-shape detector: for recordings labeled as 180° turns, the angle between outbound and return leg bearings — target > 170°.
3. **`.github/workflows/replay.yml`** (new file): builds the native module with tests/cpp CMakeLists, runs `replay_harness` on every JSON in `tests/sims/`, fails the CI if any metric regresses beyond a threshold.
4. **Regression fixtures**: every bug fix adds a new JSON to `tests/sims/regression/`, so the V-shape can never come back silently.

---

## Step 8 — Cleanup (run continuously, not a step at the end)

Each of these happens *as part of* the step that makes them possible:

- `scalar_heading_`, `heading_fej_`, `old_R` logging: deleted in Step 1.
- `global_R_` / `global_t_` Tracker ownership: deleted in Step 4.
- `vehicle_speed_mps_` / `in_vehicle_mode_` hand-rolled integration: deleted in Step 3 (replaced by VI scale observer).
- Disabled Mapper pipeline: deleted in Step 2 (replaced by keyframe feature manager + visual-inertial observer).
- Disabled Phase 9 MSCKF block: deleted in Step 4 (its intent is absorbed into the unified ESKF).
- `is_walking_pattern_` as accel-variance classifier: replaced in Step 3 by step-detector output + explicit rotation gate. The field is deleted.
- All magic numbers that cannot be justified from noise density: deleted or moved to a single `VioConfig` struct with documentation.

---

## Step 9 — Documentation (as code, not separate)

- Every non-obvious constant has a source citation comment (Madgwick thesis section, OpenVINS paper, phone datasheet page).
- `docs/ARCHITECTURE.md` (new, on completion) — a single architecture diagram showing: sensor in → Madgwick → ESKF predict → (VO, ZUPT, MiDaS, VI scale) updates → pose out → UI. No other architecture doc exists. The existing ones in `docs/` are archived.
- ADR-001 through ADR-N under `docs/adr/` — one per decision (why Madgwick, why not EKF-only, why MiDaS is blocking for scooter mode, why no GPS in hot path).

---

## Acceptance Criteria for the Whole Plan

The project is production-ready when **all** of these hold on a clean build, on a real device, on a single session without restart:

1. Walk 50 m outdoors in a straight line → final position within 1 m (2% drift).
2. Walk a 20 m × 20 m square indoors → loop closure gap < 1.5 m.
3. 180° turn-in-place followed by 10 m walk back → outbound and return legs parallel within 5°.
4. Ride a scooter 100 m in a straight line (looking forward, road visible) → final position within 5 m (5% drift), with MiDaS firing.
5. All four scenarios above are recorded as sim JSONs and pass the replay harness in CI.
6. On-device CPU budget < 15% average, < 40% peak on Samsung mid-range. Battery drain < 10%/hour with screen on and VIO running.
7. No crash in a 1-hour continuous session.
8. No magic number in the codebase is unjustified.
9. No dead code, no disabled subsystems, no "TODO future work" comments.

---

## Non-Goals

- Multi-user map sharing. Out of scope for the first production release.
- SLAM loop closure against a persistent map. Mapper is deleted; loop closure is limited to same-session via keyframe matching.
- Indoor Wi-Fi / Bluetooth fingerprinting as a scale source. Out of scope.
- GPS blending of any kind in the hot path. Explicit non-goal per the project's core value proposition.
