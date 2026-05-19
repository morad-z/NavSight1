---
name: "NavSight VIO Specialist"
description: "Deep expertise on the NavSight C++ VIO core: EKFState (15-DOF error state, propagation, measurement updates), IMUPreintegrator (Madgwick + preintegration), Tracker (visual front-end), MSCKF/SLAM features, frame conventions. Use for any EKF question, VIO bug, p_G drift, rotation matrix, preintegration, chi² gate, gravity alignment, or frame-convention question. Knows the chi² fix history and the lesson 'when chi² rejects, look at residual data first, not parameters'."
---

# NavSight VIO Specialist

## Overview

NavSight is a GPS-denied Android navigation system that fuses IMU + monocular camera through a 15-DOF error-state Kalman filter. This skill carries the deep architectural knowledge of the C++ VIO core so you can answer questions, fix bugs, and avoid the traps that bit the team for three days on the chi² blocker.

## When to use

Trigger on any of:
- "EKF question", "VIO bug", "filter divergence", "state vector"
- "p_G drift", "global_t_ vs p_G", "phantom Z drift"
- "rotation matrix", "R_GtoI", "R_bc", "frame convention", "Z-up", "Y-up"
- "preintegration", "Madgwick", "gyro bias", "accel bias"
- "chi² gate", "loop closure rejected", "updateAbsolutePose"
- "MSCKF", "SLAM feature", "clone pose", "FEJ"
- "gravity alignment", "ZUPT"
- C++ work in `app/src/main/cpp/EKFState*`, `IMUPreintegrator*`, `Tracker*`, `Updater*`, `LoopClosureDetector*`

## Architecture — 4 tiers

```
JNI ↔ Kotlin UI (NativeBridge.kt, NavSightViewModel)
   ▲
   │
VioEngine (orchestrator, owns the two heavyweight modules below)
   ▲
   ├── Tracker (visual front-end: KLT, ORB, keyframes, owns EKF + updaters)
   │   ├── EKFState (15-DOF error-state filter)
   │   ├── FeatureManager (KLT IDs, MSCKF candidates, KF descriptor ring)
   │   ├── LensCorrector
   │   ├── TrackKLT
   │   ├── UpdaterZeroVelocity
   │   ├── UpdaterMSCKF
   │   ├── InertialInitializer
   │   ├── LoopClosureDetector (DBoW2)
   │   ├── ScaleFuser, ScaleEstimatorVI
   │   └── BA worker thread (WindowedBA)
   │
   └── IMUPreintegrator (Madgwick attitude filter, gyro/accel ring,
                         biases, time-offset, mag one-shot init)
```

The exhaustive companion docs:
- `docs/study/01_ekf_core.md` — every state, every propagation step, every measurement update
- `docs/study/02_vio_tracker.md` — Tracker pipeline, KLT, ORB, keyframes, time offset, rolling shutter
- `docs/study/04_updaters_scale.md` — MSCKF, ZUPT, scale fusion math
- `docs/study/05_vio_engine_jni.md` — VioEngine + JNI surface

## State vector layout

15-DOF IMU error state + clones + SLAM features. Order in `P_` is fixed:
`[IMU(19) | Clone_0..N(6 each) | SLAM_0..M(5 each)]` — see `EKFState.cpp:391-471`.

| Rows | Symbol | Frame | Units | Mean state member |
|---|---|---|---|---|
| 0–2 | δθ | **world (Z-up ENU)** | rad | `R_GtoI_` (3×3) |
| 3–5 | δb_g | body | rad/s | `b_g_` |
| 6–8 | δv | world | m/s | `v_G_` |
| 9–11 | δb_a | body | m/s² | `b_a_` |
| 12–14 | δp | world | m | `p_G_` |
| 15 | δt_d | scalar | s | `t_offset_cam_imu_` |
| 16–18 | δφ_bc | so(3) | rad | `R_bc_` (currently SKIPPED in updates — clones bake R_bc) |

Per-clone block: `[δθ_c (3), δp_c (3)]`. Cap `MAX_CLONES = 11`. SLAM block: `[α, β, ρ, pad0, pad1]` per feature, cap `MAX_SLAM_FEATURES = 12`.

## Frame conventions — memorize these

- **World**: ENU, Z-up. `g = (0, 0, -9.81)` at `EKFState.cpp:140`. Yaw nav-conv (CW from north).
- **Body / IMU**: `R_GtoI_` is **world→body**. For body at heading ψ, level: `R_GtoI = Rz(ψ)`.
- **Camera**: `R_bc` is body→camera. Default `diag(1, -1, -1)` for rear camera, vertical phone.
- **Error perturbation**: δθ is **world-frame, left-multiply**: `R_new = exp([δθ_w]_×) · R_old`. ∂yaw/∂δθ is the constant world-Z axis `(0, 0, 1)`. See `EKFState.cpp:1078-1102`.
- Pre-2026-05-08, the project briefly went Y-up. Reverted at commit `ceb8af3`. Anything that says Y-up in old code is stale.

## Two trajectories — critical to understand

NavSight has historically maintained two parallel trajectories:

| | Tracker `global_t_` | EKF `p_G_` |
|---|---|---|
| Updated by | Visual scaled-displacement + PDR fallback (Tracker.cpp section 9) | `propagateIMU` + measurement updates |
| Drifts? | Yes, ~5% bounded by visual scale | Used to drift to **800 m** before Stage 1 fix |
| User-facing | **Yes** — `VioData.t = global_t_`, map dot, GPX, snap | **No** (only covariance ring) |
| Loop-closure anchor | Was used as patch from 2026-05-09 morning–afternoon | **Now the principled source** post Stage 1 fix |

After the gravity-alignment fix (2026-05-09 evening), `p_G` stays bounded by physics, so it's once again the principled anchor. The earlier `global_t_` patch in `consumeLoopClosureMatchIfReady` is reverted.

## Propagation math

Per `EKFState::propagateIMU` (`EKFState.cpp:118-292`):

```
R_new = R_GtoI_ · deltaR              # gyro integration (Rodrigues, Forster midpoint)
v_new = v_G_ + g·dt + R_GtoI_^T · deltaV
p_new = p_G_ + v_G_·dt + ½·g·dt² + R_GtoI_^T · deltaP
```

`deltaR/deltaV/deltaP` come from `IMUPreintegrator::integrate`, which does midpoint integration (Forster 2017 §IV.A). `deltaV/deltaP` are body-frame and INCLUDE specific force (gravity not subtracted yet) — the EKF's `g·dt` and `½·g·dt²` cancel it, given correct `R_GtoI`.

**This cancellation is what failed before Stage 1.** If `R_GtoI` is tilted by θ, world-Z residual acceleration is `(cos(θ) − 1)·g`. A 9.5° tilt gives −0.135 m/s², integrating to **−820 m over 110 s** — exactly what the loop_house_x2 sim showed.

## Measurement updates

| Method | What it observes | DOFs | Where |
|---|---|---|---|
| `applyMSCKFUpdate(H, r, R_noise)` | Generic Joseph-form update with per-row Huber kernel (δ=2.4477) | varies | `EKFState.cpp:584-797` |
| `updateRelativePose(t_world_metric, clone_id, var_t)` | Position-delta vs clone | 3 | `:801-832` |
| `updateRelativeRotation(R_meas_body, σ², clone_id)` | Per-frame rotation from `recoverPose` | 3 | `:834-893` |
| `updateAbsolutePose(target_R, target_p, σ²_R, var_p)` | Loop-closure target world-frame IMU pose. **chi² gate 22.5.** | 6 | `:925-1060` |
| `updateGravityAlignedYaw(yaw_meas, var, roll, pitch)` | Yaw from gravity-aligned visual rotation | 1 | `:1062-1115` |
| `updatePDRStep(dx, dy, var)` | 2-DOF XY position constraint | 2 | `:1117-1135` |
| `updateGravityAlignment(accel_body, var)` | **Stage 1 fix.** 3-DOF roll/pitch from accel direction (yaw unobservable). | 2 effective | `:1137+` |
| `updateZUPT()` | `v_G_ = 0` when stationary | 3 | `:314-345` |

The chi² gate at `EKFState.cpp:1000` is `kChi2Threshold = 22.5` ≈ chi²(0.999, 6).

## Loop closure pipeline (Step 7 / ADR-013)

1. **Camera thread** publishes keyframes via `addKeyframe` after every accepted KF.
2. **Worker thread** (1 Hz, `LOOP_CLOSURE_QUERY_PERIOD_S`) queries DBoW2:
   - Adaptive minScore = max(min over 10 recent neighbors, `kBowScoreFloor = 0.005`) — replaces the misread Galvez-Lopez 2012 fixed 0.05
   - Temporal exclusion 30 s (`LOOP_CLOSURE_TEMPORAL_EXCL_NS`)
   - Heading gate ±π/2 (ORB not 180°-invariant)
   - PnP RANSAC: 100 iters, 4.0 px reproj, 0.99 conf, ≥15 inliers
3. **Camera thread** consumes match, applies damped `updateAbsolutePose` over 10 frames.
4. **Step 7.1 (geometric path)** runs as a fallback when BoW returns false, using `pts3d_world` projection + KLT-corner NN matching. Direction-invariant. See `docs/VISUAL_PLAN_STEP_7_1_GEOMETRIC_LOOP.md`.

## The chi² lesson — read before tuning anything

Three days of struggling were spent tuning `LOOP_CLOSURE_BASE_TRANS_SIGMA_M`, then `LOOP_CLOSURE_DRIFT_RATE`, then dynamic formulas — all to make the chi² gate accept loop-closure injections that kept getting rejected. **It was the wrong layer.** The bug was that EKF `p_G` had drifted to −800 m on Z, so target_p was 800 m off, m² was ~5.8M, no σ_p tuning could close that gap.

The lesson, encoded in `scripts/analyze_chi2_rejections.py`:

> When chi² rejects, dump `m²_R` vs `m²_p` and the actual residual magnitudes BEFORE touching any constant. Per-block diagnostics at `EKFState.cpp:1009-1036` log them. If residuals are physically implausible (>10 m position on a 100 m walk), the bug is upstream — find what corrupted state, don't loosen the gate.

After the data showed 130/130 rejections were position-dominated with median |r_p| = 232 m, root cause was traced in 20 minutes:
- Gyro bias residual ~0.06°/s integrates to 6-10° tilt in 110 s
- Tilt gives `(cos θ − 1)·g ≈ -0.13 m/s²` residual Z-accel
- That integrates to ~800 m phantom drift
- The fix is **gravity-alignment measurement update** (Stage 1), not chi² parameter tuning

## Implementation Playbooks

Every playbook is a complete, no-shortcuts procedure. Split into named steps when a single shot is too big. Never skip a step. Never leave a `TODO` in shipped code — if a step can't be completed, stop and tell the user.

### Playbook A — Add a measurement update to EKFState

**When:** introducing a new sensor observation that constrains the EKF state.

**Step A1 — Derive the observation model on paper (cite physics).**
Write down `z = h(x) + ε` explicitly. State the frame of `z`, the units, and the ε noise model (with variance derived from sensor Allan variance, calibration RMS, or a measured statistic — never "looks right").

**Step A2 — Compute the Jacobian by hand.**
`H = ∂h/∂δx`. Identify which state slots have non-zero rows (most updates touch ≤6 of 19 IMU rows + at most one 6-row clone block). Document any state slot that is structurally unobservable from this measurement (e.g., yaw is unobservable from gravity).

**Step A3 — Add the public method declaration in `EKFState.h`.**
Place near related update methods (around line 196-260). Use the existing comment style: doc-comment with arg list, units, what gets observed, what gets returned. Cite the variance derivation in the comment.

**Step A4 — Implement in `EKFState.cpp`.**
- Validity gate first: state initialised, args well-formed, sensor in valid band (cite the band's derivation).
- Build `H` as `cv::Mat::zeros(rows, dim, CV_64F)`, copy non-zero blocks in.
- Build `R_noise` as `cv::Mat::eye(rows, rows, CV_64F) * var_unit` where `var_unit` is the cited variance.
- Call `applyMSCKFUpdate(H, residual, R_noise)` — NEVER write `P_` directly.
- Return `bool` success.

**Step A5 — Wire it into `Tracker.cpp`.**
Call from the appropriate place in `processFrame`. If the call needs sensor data, pull from `imu` parameter; if it needs EKF state, use the relevant getter.

**Step A6 — Add a logcat line.**
Format: `LC_<name>: <residual> <variance> <result>`. Example template: `EKFState.cpp:1042-1050` (LC_ABS line). This makes debug pulls actionable.

**Step A7 — Document in `docs/study/01_ekf_core.md`.**
Add to the §3 Public Functions table and §8 Interactions section. Include the math, the variance derivation, and the gating conditions.

**Step A8 — Validate against existing fixture(s).**
Before flashing, run the offline harness on a checked-in sim. Confirm metrics aren't worse than baseline.

### Playbook B — Diagnose "p_G is wrong"

**Step B1 — Pull logcat from the phone.**
`adb logcat -d > sim.log` within minutes of the walk; the buffer rolls off. `adb logcat -G 16M` if you need a longer session.

**Step B2 — Run `scripts/analyze_chi2_rejections.py`.**
This dumps block dominance, residual magnitudes, p_G evolution, and required σ_p. Read the output BEFORE forming a hypothesis.

**Step B3 — Categorise the failure.**
- `m²_p` dominates AND `|r_p|` >>> walked-distance × drift-rate → propagation bug. Suspects: gravity cancellation, accel bias, R_GtoI drift.
- `m²_R` dominates AND `|r_R|` > 30° → frame convention, R_bc, or yaw bug.
- Both moderate but cross-correlation explodes m²_total → covariance modeling issue.

**Step B4 — Read p_G evolution over time.**
If p_G[Z] diverges monotonically: gravity cancellation broken (`R_GtoI · g` mis-applied in propagateIMU). If p_G oscillates: corrections being thrashed (false-positive loop closures). If p_G jumps: an injection accepted that shouldn't have been.

**Step B5 — Identify upstream root cause.**
Use the data, not theory. The chi² fix history is in this skill — recognise the pattern: rotation drift → gravity miscancel → position blowup → chi² rejection cascade.

**Step B6 — Propose fix at the upstream layer, not chi² parameter tuning.**
Tuning σ to admit a 232 m residual is not a fix; it's papering over a 232 m state-vector error. Find what corrupted the state.

**Step B7 — Validate the fix offline before flashing.**
Run the harness on the same sim. The fix should bring `m²_p` and `|r_p|` into physically plausible ranges (single-digit metres on a 100 m walk).

### Playbook C — Trace a frame-convention bug

**Step C1 — State the convention you're checking.**
NavSight is Z-up ENU. R_GtoI = world→body. R_bc = body→camera. Gravity = (0, 0, -9.81).

**Step C2 — Read the suspected code path.**
For every matrix operation, write down what frame the result is in. Mark every `R · v` and `R^T · v`. Track units.

**Step C3 — Cross-check against `docs/study/01_ekf_core.md` §7 Frame Conventions.**
Compare to documented conventions. Any deviation is suspect.

**Step C4 — Run the offline test.**
`tests/cpp/test_ekf_yaw_convention.cpp` is the canonical regression for the Z-up yaw extraction. Add a similar test if you're touching frame math.

**Step C5 — If a sign appears flipped, write a 4-line proof on paper.**
"If body is screen-up flat, accel measures (0, 0, +9.81). After bias correction deltaV ≈ (0, 0, +9.81)·dt body. R_GtoI^T · deltaV = world (0, 0, +9.81)·dt. Add g·dt = (0, 0, -9.81)·dt. Net = 0. ✓ or ✗ — which step fails?"

**Step C6 — Fix only the step that failed.**
Don't refactor frame conventions broadly. Surgical edit, comment-cite the derivation.

## Project guardrails (enforced)

### No magic numbers (per `VISUAL_PRODUCTION_PLAN.md` Principle 5)

Every threshold must cite its source: a chi² table entry, a sensor noise model, a calibration RMS, a measured statistic from `tests/sims/`. "0.20 looks right" is not allowed. If you introduce a new constant, the comment must derive it from physics or measurement.

Examples of good citations already in the code:
- `MSCKF_HUBER_DELTA = 2.4477 = √χ²(0.95, 2 dof)` (`EKFState.h:503`)
- `LOOP_CLOSURE_DRIFT_RATE = 0.032 = 0.15/√(22.5−0.84)` (`Tracker.h:524-527`)
- Stage 1 gate band `g ± 0.8 m/s²` derived from `3σ_acc + walking-band`, σ_acc=0.1 from EKFState ctor (`EKFState.cpp:81`)

### No shortcuts, no TODOs

If a step can't be completed in the current shot:
1. STOP at the step.
2. Tell the user what's blocking and exactly which step is incomplete.
3. Do NOT leave `// TODO`, `FIXME`, `XXX`, or stubs in shipped code.
4. Do NOT skip ahead and "come back to it later."

### Comment, don't delete

Unused / superseded code stays in-tree as `/* ... */` blocks with a `LEGACY:` marker explaining why it's commented and what currently does the job. See `Tracker.h:29-37` for the canonical example, plus reverts in `Tracker.cpp` from 2026-05-09 (Stage 2 reverts of the global_t_ patch).

This rule applies to permissions, manifest entries, build flags, and entire files (e.g. `Mapper.cpp` / `PoseGraph.cpp` are commented out in `app/CMakeLists.txt:62-63`).

## Gotchas

- **OpenCV 4.5.3 unary-minus bug** at `EKFState.cpp:1716-1727`: write `mat * -1.0` not `-mat`. Workaround comment is in code; don't "fix" it.
- **Velocity hard-clamp 5 m/s** at `EKFState.cpp:288-291` is a workaround that masks runaway-velocity root causes. Keep it for now; flag if changing.
- **R_bc EKF update is currently SKIPPED** at `EKFState.cpp:699-704` — Option C bakes R_bc into clones at addClone time. Check `KNOWN_ISSUES.md` P0 #1-3.
- **`-fno-finite-math-only`** in `app/CMakeLists.txt:17` is load-bearing — re-enables `isfinite()` filtering across 7 files. Don't remove.
- **Tracker output uses Madgwick yaw, NOT EKF yaw** (`Tracker.cpp:2670-2690`) due to the V-shape bug fix. Don't "unify" without understanding why.

## References

- Code: `app/src/main/cpp/EKFState.{h,cpp}`, `IMUPreintegrator.{h,cpp}`, `Tracker.{h,cpp}`, `UpdaterMSCKF.{h,cpp}`, `UpdaterZeroVelocity.{h,cpp}`, `LoopClosureDetector.{h,cpp}`
- Studies: `docs/study/01_ekf_core.md`, `02_vio_tracker.md`, `03_loop_closure_map.md`, `04_updaters_scale.md`, `05_vio_engine_jni.md`
- Plans: `docs/VISUAL_PRODUCTION_PLAN.md`, `docs/VISUAL_PLAN_STEP_7_1_GEOMETRIC_LOOP.md`, `docs/PRODUCTION_READINESS_PLAN.md`
- ADRs: `docs/adr/ADR-001..014`
- Issues: `docs/KNOWN_ISSUES.md`
- Diagnostic: `scripts/analyze_chi2_rejections.py`
