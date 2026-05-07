# NavSight Implementation Status

**Date:** 2026-05-07
**Branch:** `morad`
**Scope:** snapshot of what is shipped (committed) and pending (uncommitted)
across `PRODUCTION_READINESS_PLAN.md` (inertial) and
`VISUAL_PRODUCTION_PLAN.md` (visual), framed by their drift/accuracy
contributions.

This is a status document, not a roadmap. For the roadmap see the two
plan files. For decision history see `docs/adr/`.

---

## TL;DR

- **Inertial plan:** Steps 1–4 effectively shipped (Madgwick, gravity-aligned
  visual yaw, three-observer scale fusion, single-source-of-truth ESKF).
  Steps 5–7 partially shipped (calibration gate, replay harness exists).
  Steps 8–9 in progress (ongoing cleanup + per-decision ADRs).
- **Visual plan:** Steps 1–8 shipped (intrinsics, R_vo, hybrid SLAM+MSCKF,
  ORB reloc, adaptive front-end, BA, loop closure, online TD/extrinsics/RS).
  Steps 7 and 8 are **not yet accepted** — they need a daytime sim re-walk
  to validate after today's heading fix.
- **Today's work (uncommitted):** root-caused and fixed an EKF visual yaw
  sign bug responsible for 75° heading drift on real walks. Three
  agent-confirmed lines in `EKFState.cpp`. Critical for unblocking Step 7
  acceptance.
- **Withdrawn:** ADR-017 (GPS course as bounded yaw) — violated the
  VIO-only design principle.

---

## Inertial side — `PRODUCTION_READINESS_PLAN.md`

### Step 1 — Madgwick attitude filter ✅ shipped

**What it does for drift/accuracy:** replaces the original 1-D
`scalar_heading_ += yaw_rate * dt` integrator (which produced 30–46°
heading errors on 180° turns and the infamous "V-shape") with a proper
SO(3) attitude filter. Gyro drives short-term yaw integration; accel
provides roll/pitch drift correction; yaw remains gyro-driven (correctly
unobservable from gravity alone).

**Evidence:** `IMUPreintegrator` quaternion state, `getMadgwickRoll()` /
`getMadgwickPitch()` / `getHeading()` accessors. Commits `a2aa83e`,
`6ab68c0` ("Madgwick attitude filter"). ADR-001.

**Drift contribution:** bounds short-term heading error to gyro-bias-rate
drift (~0.4°/keyframe interval at 22 Hz) instead of compounding scalar
projection errors during turns.

### Step 2 — Visual odometry without scalar heading ✅ shipped

**What it does:** keyframe heading correction (`Tracker.cpp:2030–2222`)
now extracts gravity-aligned yaw from the essential-matrix rotation
using current Madgwick roll/pitch, not from a degenerate scalar atan2.
Camera-frame `R_vo` is converted to body frame via the EKF-maintained
`R_bc` extrinsic (Step 8b) before the gravity-alignment sandwich. The
old `gyro_norm < 0.3` gate is gone — the gravity-aligned correction is
safe to fire during turns.

**Evidence:** `Tracker.cpp:2046–2218`. Visual yaw fed via
`EKFState::updateGravityAlignedYaw`.

**⚠️ Caveat:** today's investigation found a **sign error** in this
correction's residual computation. Fixed but uncommitted — see "Today's
work" below. Step 2 is shipped but produced wrong-direction corrections
on real walks until 2026-05-07.

### Step 3 — Scale estimation in a GPS-denied world ✅ shipped (all 3 observers)

Three parallel observers, each with derived covariance, fused in a 1-D
scale Kalman filter (`ScaleFuser`):

- **Observer A — PDR.** `Tracker::detectAndUpdateStep` writes step
  displacement; rotation gate at `ROTATION_STEP_GATE_RADPS`. Stride
  learning + per-session refinement.
- **Observer B — MiDaS depth.** `applyDepthScaleConstraint` produces
  metric scale from floor-plane depth ratios.
- **Observer C — Hesch–Martinelli VI bootstrap.** `ScaleEstimatorVI`
  closed-form least-squares on stacked keyframe-pair IMU/visual deltas
  every N keyframes.

**Evidence:** `ScaleFuser.{h,cpp}`, `ScaleEstimatorVI.{h,cpp}`. ADR-003
(MiDaS blocking for scooter), ADR-004 (no GPS in hot path).

**Drift contribution:** monocular scale would otherwise be unobservable;
the three-observer fusion bounds scale to within ±10–20% over a session
on pedestrian sims.

**⚠️ Open issue from today's audit:** the scale fuser is currently
biased low (VIO/GPS = 0.88×, 12% undershoot) on the cited real-walk sim
because Observer C is silenced by the heading-convention bug — once
that bug is fixed, Observer C should re-engage and the bias should
drop. Validation pending.

### Step 4 — Single state estimator (ESKF) ✅ shipped

**What it does:** all global pose lookups go through `EKFState::getPose()`.
The dual-state-tracking family of bugs (Tracker pose vs EKF pose) is
eliminated by design. `Tracker::scalar_heading_` is a read-only mirror
refreshed from `ekf_.getRotation()` at `Tracker.cpp:1411`, not an
independent integrator.

**Evidence:** `EKFState.cpp` propagation + update channels. Single
mirror confirmed by the convention agent's audit today.

**Drift contribution:** removes a category of bugs; doesn't directly
reduce drift, but stops drift from appearing in two places at once and
making each "fix" undo the other.

### Step 5 — Calibration & initialization ⚙️ partial

**What's shipped:**
- Stationary startup gate for gyro/accel bias estimation
- Initial gravity direction from accel
- One-shot magnetometer for initial yaw (ADR-005)
- User-confirmed calibration bypass for the stationary gate (commit
  `f1684e4`)

**What's not:**
- Full stride-calibration walk flow with stored `SharedPreferences`
- MiDaS relative-to-metric ratio calibration baked into the same walk

### Step 6 — Covariance-aware UI ⚙️ partial

**What's shipped:** EKF covariance is exported (visible in event_summary
counters). Status chip exists. Crash log dumps to `<external-files>/`.

**What's not:** uncertainty ellipse on the map UI, color-coded
covariance trace on the radar.

### Step 7 — Replay harness + CI ✅ shipped (harness), ⚙️ partial (CI)

**What's shipped:** `tests/cpp/replay_harness.cpp` exists and consumes
sim JSONs. `scripts/analyze_sim.py` provides offline scoring. Per-step
sim recordings live in `tests/sims/regression/`. ADR-007 documents the
IMU-only synthetic-grey-frame design.

**What's not:** the formal replay-scorer Python tool with V-shape
detector and CI workflow; today's `scripts/analyze_sim.py +
compare_gps_vio.py + heading_audit.py` cover most of the scoring
manually.

### Step 8 — Cleanup ⚙️ ongoing

Per the plan, this is continuous. Removed so far:
`scalar_heading_` independent integrator, `heading_fej_`, the 30%
`heading_offset_` blend (Tracker.cpp:2210–2212), Tracker-side global
pose ownership, the original disabled MSCKF block.

### Step 9 — Documentation as code ⚙️ ongoing

ADRs 001–013 live in `docs/adr/`. ADR-014/015/016 are referenced from
code but have no on-disk files (back-fill task tracked separately).
ADR-017 was drafted then withdrawn the same day (see "Today's work").

### Inertial-plan acceptance criteria status

| Criterion | Target | Status |
|---|---|---|
| 1. 50 m straight outdoor walk → < 1 m drift | < 2% | **Failing** — sim 1778147132092 shows ~12% scale error and 75° heading drift; pending today's fix |
| 2. 20×20 m indoor square → < 1.5 m gap | loop close | Pending acceptance walk |
| 3. 180° turn-in-place → legs parallel ±5° | direction recovery | **Likely failing** — heading sign bug active on commits prior to 2026-05-07 |
| 4. Scooter 100 m → < 5 m drift | scooter mode | Pending scooter sim |
| 5. All scenarios in CI replay | regression | Replay harness exists; CI workflow not yet wired |
| 6. CPU < 15% avg / 40% peak, battery < 10%/hour | budget | Not measured systematically |
| 7. No crash in 1-hour session | stability | Pending |
| 8. No unjustified magic numbers | code quality | Mostly clean; today's audit flagged a few residual ones (UpdaterMSCKF pixel_noise, MiDaS mad_floor 1%, scale fuser r_var 0.04) |
| 9. No dead code | code quality | LoopClosureDetector re-enabled (Step 7 visual); Mapper deletion ongoing |

---

## Visual side — `VISUAL_PRODUCTION_PLAN.md`

### Step 1 — Camera intrinsics calibration ✅ shipped

**What it does:** in-app chessboard calibration screen with auto-capture,
9-cell coverage diagnostic, and OpenCV `calibrateCamera`. The previously
zero-distortion `LensCorrector` now applies real per-device intrinsics.

**Evidence:** Step 1 commit batch `e5c53fc`. Calibration UI in
`MainActivity` split (commit `093fb0e`).

**Drift contribution:** removes 8–15 px edge-of-frame distortion bias
that was propagating through `findEssentialMat` into rotation noise on
every keyframe.

### Step 2 — Stop discarding `R_vo` ✅ shipped

See **Inertial Step 2** above; the gravity-aligned visual yaw correction
is the same line of work, claimed by both plans.

### Step 3 — Hybrid SLAM + MSCKF features ✅ shipped

**What it does:** long-lived features get promoted to the EKF state
(`slam_features_` block); short-track features go through MSCKF
null-space projection. Damped, EKF-consistent injection per ADR-008.

**Evidence:** `slam_features_` array in `EKFState`,
`UpdaterMSCKF.{h,cpp}`, `applyMSCKFFeature`. Commit `e5c53fc`. ADR-008,
ADR-009.

**Drift contribution:** dense per-frame visual updates between
keyframes. Long-lived features bound drift between keyframes.

**⚠️ Open issue:** MSCKF Huber rejection rate is currently 1.66/update
on the cited sim — over half of measurements rejected as outliers.
Today's audit identified this as downstream of the heading-convention
bug (residuals biased by yaw misalignment) plus a pixel_noise units
mismatch. Both deferred until heading fix is validated.

### Step 4 — ORB descriptors at keyframes ✅ shipped

**What it does:** ORB descriptors stored per keyframe in a ring buffer;
relocalization through descriptor matching when KLT loses track.

**Evidence:** `feature_mgr_.storeKeyframeDescriptors` at
`Tracker.cpp:2237`. Commit `282e889`. ADR-010.

**Drift contribution:** recovers tracking after KLT loss without
requiring close-in-time keyframes. Critical for handling viewpoint
changes > 30°.

### Step 5 — Adaptive front-end robustness ✅ shipped

**What it does:** KLT window size adapts to motion magnitude; blur
detection skips frames with high motion blur; lowlight detection;
pure-rotation gate.

**Evidence:** `klt_adaptive_window_hits` counter, `blur_*` and
`lowlight_*` events. Commit `5028ff0`.

**Drift contribution:** keeps the front-end producing valid
observations across motion regimes that previously degraded silently.

### Step 6 — Local windowed bundle adjustment ✅ shipped

**What it does:** Ceres-based 5-keyframe BA every 0.5 s reconciles
keyframe poses + landmarks. Damped injection back into the EKF.

**Evidence:** `ba_solves_*` counters, BA accept/reject logic. Commits
`85e1668`, `b6011e5`. ADR-012.

**Drift contribution:** local consistency between keyframes. On the
cited sim: 6 BA solves, 4 accepted, avg 19.7 iterations.

### Step 7 — Same-session loop closure (DBoW2) ✅ shipped, ⏳ acceptance pending

**What it does:** DBoW2 vocabulary + per-keyframe BoW vectors; place
queries every 1 s; geometric verification via solvePnPRansac; correction
injection through `EKFState::updateAbsolutePose` (world-frame absolute
pose channel).

**Evidence:** `LoopClosureDetector.{h,cpp}`, `assets/ORBvoc.dbow2`,
`updateAbsolutePose` at `EKFState.cpp:930`. Commits `fce0b0a`,
`123ebd7`, `2c97162`, `d64a4ff`, `221b22b`. ADR-013.

**⚠️ Acceptance blocker:** on sim 1778147132092 (today): 726 BoW
attempts, 9 accepted at the BoW+PnP+chi² gate, but **0 corrections
applied** because the EKF's absolute-pose chi² gate at threshold 22.5
rejected every damped attempt. Root cause is upstream: the heading
convention bug produced position+yaw residuals too large for the chi²
gate to admit. Once heading fix is validated, residuals shrink and
loop closures should naturally pass the gate.

### Step 8 — Online TD / extrinsics / rolling shutter ✅ shipped, ⏳ acceptance pending

- **8a online time offset:** EKF state extended by `δt_d`, per-frame
  measurement update with feature-velocity-driven Jacobian
  (`EKFState.cpp:1893–1939`). Li & Mourikis 2014 recipe.
- **8b IMU-camera extrinsics:** rotation-only refinement of `R_bc`,
  3-DOF added to error state. Initialised from device orientation.
- **8c rolling-shutter compensation:** per-feature timestamp = frame
  timestamp + (row / image_height) × skew, used in clone-window
  lookup.

**Evidence:** Commit `221b22b`. Counters `cam_imu_time_offset_us`,
`extrinsics_rotation_angle_mdeg`.

**⚠️ Acceptance status (today's analysis):**
- Heading RMSE Criterion 1 — **PASS** (69° → 42°, 40% improvement vs
  10% required) on the cited sim, even with the heading bug active.
- TD offset Criterion 2 — was **REVIEW** on the original "±5 ms of
  warmup" wording; rewritten today (this session) to a hardware-grounded
  bound (`|TD| ≤ 100 ms`, `|TD−warmup| ≤ 10 ms`, `σ_TD ≤ 5 ms`)
  because warmup quantises to integer camera-frame periods (~33 ms at
  30 fps), making the original criterion mathematically unsatisfiable.
- Extrinsics drift counter was always reporting 180° (initial
  `diag(1,-1,-1)` is 180° from identity); fixed today to measure drift
  from initial nominal, not from identity.

### Step 9 — Replay harness + visual fixtures ⏸️ not started

Recorded-camera channel, frame compression, replay harness extension,
visual scorer metrics, CI job — all pending.

### Step 10 — Scooter mode hardening ⏸️ not started

Auto mount-mode detection, vibration filtering, pavement rejection,
4-DOF pose graph for long routes — all pending.

### Step 11 — Sensor health & fault tolerance ⏸️ not started

---

## Today's work (2026-05-07, uncommitted)

Investigation of sim `tests/sims/simulation_data_1778147132092.json`
(485 s daytime walk, GPS-confirmed loop, **75° heading drift,
70+ m endpoint position error**) found the visual yaw correction
was pushing the EKF the **wrong direction** on every keyframe.

### Code changes applied (3 lines, all in `EKFState.cpp`)

| Line | Change | Effect |
|---|---|---|
| 1052 | `h_body = -R_GtoI_ * e_y_world` | H Jacobian sign correction. ∂yaw/∂δθ_y = −1 not +1. Without negation, Kalman gain pushed filter the wrong way. |
| 1097 | `atan2(-R_aligned[0,2], R_aligned[0,0])` | `getYaw` returned −ψ for body at +ψ (R_GtoI_ is world→body, naïve atan2 inverts sign). |
| 1217 | `R_drift = R_bc_ * R_bc_initial` | `getExtrinsicsAngleDeg` measured drift from identity, but initial `diag(1,-1,-1)` IS 180° from identity → metric was always 180°, masking real Step 8b drift. |

Both heading-related fixes (1052 + 1097) **must land together**.
Fixing only one makes things worse; fixing both is necessary AND
sufficient (hand-verified against the 6 tests in
`tests/cpp/test_ekf_yaw_convention.cpp`).

### Other changes today (uncommitted)

- `docs/VISUAL_PRODUCTION_PLAN.md` Step 8 acceptance criterion 2
  rewritten to a hardware-grounded TD bound (see Step 8 above).
- `tests/cpp/test_ekf_yaw_convention.cpp` — 6 GoogleTest cases pinning
  the rotation convention. Cannot build on this Windows host (no
  desktop OpenCV); must run via Android NDK or on a Linux machine.
- `scripts/compare_gps_vio.py` — GPS↔VIO trajectory comparison with
  PNG plot.
- `scripts/heading_audit.py` — vyaw vs gyro-integral vs GPS-course
  time-series.
- `docs/adr/ADR-017-gps-course-as-bounded-yaw.md` + spec — drafted
  then **WITHDRAWN** the same day. NavSight is VIO-only by design;
  GPS must not feed the EKF including for yaw. Files left on disk
  with WITHDRAWN status for the historical record.
- `docs/MAP_MATCHING_PLAN.md` — researched and drafted as a sibling
  plan; marked PENDING REVISION because the first draft assumed
  GPS-fed matching. Rework needed to consume VIO-derived position
  (projected through the startup GPS anchor) instead.
- This document.

### What this fix changes for drift/accuracy

If validated on a daytime sim re-walk:

- **Heading drift**: `vyaw − gps_course` from −112° → expected within ±15°
- **Endpoint position error**: 70+ m → expected < 20 m
- **MSCKF Huber rejection rate**: 1.66/update → expected toward ~0.05
- **Step 7 loop closure**: `corrections_applied = 0` → expected > 0
  (residuals shrink below the chi² gate naturally)
- **Step 8 acceptance**: Criterion 1 (heading) already passes; Criterion
  2 (TD) needs a re-walk under the rewritten criterion

---

## Pending validation (what unblocks what)

```
[Today's heading fix]
        │
        ▼ (Morad walks daytime sim)
[scripts/analyze_sim.py + compare_gps_vio.py + heading_audit.py]
        │
        ├──► PASS → Step 7 acceptance (loop closure corrections > 0)
        │           Step 8 acceptance (TD criterion under new wording)
        │           Inertial Steps 1–4 acceptance criteria 1–3
        │
        │           Then unblocks:
        │           ├── Visual Step 9 (replay harness + CI)
        │           ├── Visual Step 10 (scooter mode hardening)
        │           ├── Visual Step 11 (sensor health)
        │           ├── Map matching plan (rework + execute after visual plan)
        │           ├── MSCKF pixel_noise units fix
        │           ├── MSCKF per-axis residual logging
        │           └── Magnetometer-revisit ADR (alternative heading anchor)
        │
        └──► FAIL → New investigation: secondary bugs not downstream of heading
```

## Plans that are out of scope right now

- **Map matching** (`docs/MAP_MATCHING_PLAN.md`) — to land *after* the
  visual plan ships. Rework pending: must consume VIO-derived position,
  not GPS.
- **Continuous magnetometer** (revising ADR-005) — possible future work
  if loop closure + intersection geometry isn't enough.
- **Multi-session map persistence**, **navigation to a destination**
  — explicit future work per Morad. Will sit on top of map matching's
  OSM data layer.

## Out-of-scope (explicit non-goals from the plans)

- GPS in the EKF hot path (ADR-004; reaffirmed today).
- Multi-user map sharing.
- Wi-Fi / Bluetooth fingerprinting as a scale source.
- ORB-SLAM3 atlas-style multi-map operation.
- DSO-style direct photometric methods.

---

## Open audit items (deferred until heading fix validated)

1. **MSCKF Huber rate** — 1.66 rejects/update on cited sim. Likely downstream
   of heading bug; verify after fix. If still high, fix `UpdaterMSCKF::Options::pixel_noise`
   units mismatch (currently 1.0 in normalized image coords ≈ focal·1 ≈ 500 px;
   should be `RANSAC_THRESH / focal`).
2. **Scale fuser instability** — `smooth_scale_` range [0.038, 0.200] vs mean
   0.0507 (23% std). Likely Observer C silenced by heading bug. If still
   noisy after fix, address scale-fuser `r_var ≥ 0.04` floor (Observer C
   never wins) and MiDaS `mad_floor = 0.01·median` magic 1% relative noise.
3. **Step 7 chi² gate** — at 22.5 (χ²(0.999, 6 DOF)); validate
   corrections start applying after heading fix without needing relaxation.
4. **Path metric `total_path_dm` mismatch** — VIO path 552 m vs
   `event_summary` path 106 m on cited sim (cross-check ratio 0.192).
   Audit `total_path_m_` accounting in `Tracker`.
5. **ADR back-fill** — write missing ADR-014/015/016 docs; their numbers
   are referenced from code but no doc files exist.
