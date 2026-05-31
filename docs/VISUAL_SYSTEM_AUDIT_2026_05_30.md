# NavSight Visual Front-End Audit — 2026-05-30

**Status:** reference (code-grounded audit). **Method:** 4-lens code read + adversarial verification + live
event counters from the 4 most recent recordings. All `file:line` below were verified against the source.

> **One-line finding:** NavSight *looks* like a full VIO/SLAM system, but the user-facing dot bypasses almost
> all of it and rides a thin, fragile path. Most of the heavy machinery (EKF position, SLAM, loop closure, MSCKF,
> bundle-adjust) is computed every frame, costs CPU/battery, and **does not move the dot, speed, or heading.**

## 1. What actually reaches the user output (the live path)

```
camera pixels (TrackKLT) → recoverPose (motion DIRECTION) → MiDaS depth (relative speed)
  → accel-calibrated scale K (relative→meters) → × Madgwick heading → integrate into the dot (global_t_)
```
- Dot = `global_t_`, advanced by `disp = depth_flow_speed × dt` (`Tracker.cpp:3614-3640, 3737-3739`); exposed as `out.t = global_t_` (`:6253`).
- Speed (speedometer) = same number, `getFusedSpeedMps → trajectory_speed_mps_` (`Tracker.cpp:1000`; Kotlin `NavSightViewModel.kt:429`).
- Scale `K` = calibrated from the accelerometer in a 0.3–2.5 s window after a ZUPT stop (`Tracker.cpp:1202-1239`). Single most load-bearing number; no backup.
- "Looming"/expansion speed = a second forward-speed estimator that blends into the same number, works when recoverPose fails (`Tracker.cpp:8038`). **Uses the EKF *rotation*** (`:8066-8068`) — see §3.
- Heading = Madgwick (gyro + gated compass), `out.heading = scalar_heading_` (`:6287`). **NOT** the EKF yaw.
- **The architecture runs "backwards":** every frame `ekf_.setPosition(global_t_)` forces the EKF to copy the dot (`:1746`); the block that would copy the EKF back into the dot is commented out (`:6214-6232`, `/* SUPERSEDED 2026-05-16 */`). The `:6249` comment "global_t_ reads back from EKF" is **stale/false**.

## 2. Is the SLAM dead weight? — Yes, largely (with caveats)

Computed every frame, reaches the output **not at all** (only writes the overwritten EKF `p_G_`):
- `slam_promotions_total` = 0,0,4,0 across recordings (15k–51k candidates) → orange dots never anchor. Root cause: forward motion gives little sideways parallax → triangulation RMS 654–4866 px vs the ≤1.5 px gate (`Tracker.cpp:4359/4383`).
- `loop_closure_geom_accepts = 0`, `loop_closure_corrections_applied = 0` → loops detected, never close.
- `WindowedBA`: `ba_solves ≈ 0` (starved by SLAM=0). `Mapper`: **not compiled** (`CMakeLists.txt:61-63`).
- EKF position output `getPose()` (`VioEngine.cpp:271`) has **zero Kotlin callers** — computed and thrown away.

## 3. Honest caveats — do NOT blindly cut (verified live dependencies)

1. **Visual→heading nudge is wired** (`Tracker.cpp:5062` `nudgeMadgwickYawAroundWorldZ`, counter `:5065`). Dormant outdoors (mag wins, gated by `!isMagActivelyFusing` `:5061`), but on a **scooter the magnetometer is dirty** (motor/frame/mount) → this visual nudge becomes the only thing bounding gyro-heading drift. **Keep the keyframe storage + this path.**
2. **EKF *rotation* (not position) feeds the live speed** — looming reads `ekf_.getRotation()` (`:8066`). "Cut the EKF" is wrong; only the *position* path is dead. Slimming the EKF requires re-pointing looming at a Madgwick rotation first.
3. **EKF covariance draws the UI uncertainty ring** — `getPositionCovarianceXZ` (`native-lib.cpp:888` → `NavSightViewModel.kt:335`), a real live consumer.
4. **recoverPose is standalone** (`TrackKLT.cpp:91-118`, only pixels + intrinsics) → cutting EKF/SLAM does **not** destabilize it.
5. **Zero-counters are from REPLAY/sims, not real life.** Replay shortcuts WAIT_STATIONARY, grayscale, ~18.7fps, no ZUPT anchor. A scooter has **strong parallax** (the opposite of low-parallax walking) → SLAM/LC may behave very differently on a real ride. **Verify on device before declaring dead.**

## 4. Keep / Fix / Cut (verdicts; "cut" = comment out in CMake, never delete)

| Component | Reaches dot? | Verdict |
|---|---|---|
| TrackKLT | yes | KEEP |
| recoverPose | yes (when verified) | KEEP |
| MiDaS / DepthEstimator | yes (sets speed) | KEEP |
| Accel-K scale + ZUPT | yes (the metric scale) | **KEEP + FIX** (no-stop calibration for scooter) |
| Madgwick heading | yes (direction) | **KEEP — protected, do not touch** |
| Visual→heading nudge + keyframes | heading (dormant→live on scooter) | **KEEP** |
| Looming / expansion speed | yes (fallback) | KEEP + FIX (also rides K) |
| KLT adaptive window | yes (via tracking) | **FIX** — grows from gyro only, not translation (`:2014-2033`) → scooter under-read |
| EKF rotation / gravity-align | indirectly (looming) | KEEP |
| EKF covariance | UI ring | KEEP |
| EKF position (`p_G_`/`v_G_`) | NO (0 callers) | CUT-or-REVIVE (architectural fork, §6) |
| UpdaterMSCKF | NO (corrects discarded p_G_) | gate off for the dot |
| UpdaterSLAM | NO (~0 promotions) | CUT (verify on scooter first) |
| LandmarkMap | NO (orange-dot overlay only) | CUT unless overlay wanted |
| WindowedBA | NO (starved) | CUT |
| LoopClosureDetector + PoseGraph | wired to dot (`:7676`) but `accepts=0` | FIX (BUG-01 SUB-A, ~10 LOC) or cut |

## 5. Does it need improving (scooter)? — Yes
- **`K` can't recalibrate without a stop** (`:1211`) → scooters don't stop → stale scale → speed+distance drift together with no backup. **#1 scooter blocker.** (The principled VI scale estimator that would fix it was proven broken + disabled `:1167-1200`.)
- **KLT window bug** (`:2014`): widens on gyro, not translation → fast straight scooter motion overruns the 21 px window → lost tracks → under-read. Real bug.
- **Parallax helps scooter** → recoverPose `verification_ok` (failed ~67% on slow walks) should pass more on a ride. Core approach sound; verify.
- **No drift bound** today (SLAM/LC dead) → map matching is the intended drift-bound for scooter.

## 6. The architectural fork
- **(A) Revive the EKF** as the real output (fuse visual + speed + future learned-velocity; read the dot *from* it). The heavy machinery stops being dead weight; gives cross-checks + drift-bounding. **Risk: touches heading** (EKF rotation + the yaw nudge interact) — against the "don't touch heading" preference.
- **(B) Commit to the thin path** + map matching. Keep heading exactly as-is, improve only the speed/scale path, retire the dead position machinery for CPU. **Lower risk to heading; recommended near-term for scooter.**

## 7. Device-check BEFORE cutting (read these on a real SCOOTER logcat)
1. `madgwick_visual_yaw_nudges_total` (`:5065`) — if >0 on a ride (dirty mag), the visual→heading path is live; keep it.
2. `loop_closure_geom_accepts` / `loop_closure_corrections_applied` (`EventCounters.h:379`; `:7676`) — if >0 on a real revisit, fix LC, don't cut.
3. `slam_promotions_total` + `slam_promo_rms_milli_max` vs the 1.5 px gate — scooter parallax may promote; verify before retiring.
4. Looming / EKF-rotation health on the speed path (`:8066`) + `depth_flow_updates` / `depth_flow_calib_updates`.
5. Covariance-ring consumer (`native-lib.cpp:888` → `NavSightViewModel.kt:335`) stays sane if MSCKF/EKF updates disabled.

## 8. Reconciliation with the map-matching plan
The plan's `vio_lla` (Step B) must derive from **the dot (`global_t_`)**, NOT EKF `p_G_` (the dead one). Update `MAP_MATCHING_PLAN.md` Step B accordingly. Map matching fixes **cross-track** drift (keeps the dot on the road) but **not along-track** distance error — that still requires the speed/scale fixes (§5).
