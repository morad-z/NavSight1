# VIO Velocity / Speed — research + implementation plan (locomotion-agnostic)

**Goal**: accurate reported SPEED for ALL motion (walking, running, scooter) — **NO mode flag** (user directive). Researched 2026-05-26; implemented + diagnosed through 2026-05-27. **⚠️ The "REVISED PLAN & IMPLEMENTATION STATUS — 2026-05-27" section below SUPERSEDES the original "Lever 0/C/A/B/D" plan** — that was written before we discovered `v_G_` diverges and the real problem is the metric SCALE, not the clamp or a scooter mode. Facts split CODE-VERIFIED vs LITERATURE.

## ⏩ HANDOFF FOR A FRESH SESSION — READ FIRST

**How to start (2026-05-27)**: read the **REVISED PLAN & IMPLEMENTATION STATUS** section below — NOT the original levers. The speed estimator is built (`Tracker::updateDepthFlowSpeed` + `getFusedSpeedMps()`); Step 1 (accel drift removal) is installed and awaiting walk validation; Step 2 (looming/flow-divergence) is researched and ready to build. Invoke skills `navsight-vio-specialist`, `navsight-implementor`, `navsight-sim-debugging`. Cross-ref memory `project_velocity_session_2026_05_26.md` (the running log).

**Git / working tree (2026-05-26)**: branch `morad`, HEAD `68b0b03` (heading fix + pose-graph fix#1 + #2 path-redraw + LC/SLAM diagnostics, all committed). UNCOMMITTED on `Tracker.cpp` + `EventCounters.h`: a parked MiDaS-seed SLAM rescue + clean-RMS instrument (Hidden Bug #3, a harmless no-op, **unrelated to velocity** — leave it). `git status` shows a big pile of pre-existing junk untracked files (`.env`, `%.4e`, `(2)`, build artifacts) — **NEVER `git add -A`/`.`; stage explicit files only; commit ONLY when the user says "commit"** (feedback_explicit_commit_only).

**Build / install / device**: `.\gradlew.bat assembleDebug` (PowerShell, repo root, ~25-40 s). Install `adb -s R5CR70S3NNB install -r app\build\outputs\apk\debug\app-debug.apk` (adb: `C:\Users\morad\AppData\Local\Android\Sdk\platform-tools\adb.exe`; device S21 Ultra `R5CR70S3NNB`). Sims: `/sdcard/Android/data/com.example.navsight1/files/simulation_data_*.json`, pull via PowerShell. **logcat rolls off fast → read `event_summary` in the JSON; `python scripts/analyze_walk.py <sim.json>` summarizes.**

**Implementation gotchas (the "unknowns" a cold session would miss)**:
- **Do NOT add a field to `VioData`** for speed — it's a 28-arg JNI-cached constructor (signature built in `JNI_OnLoad`); adding a field means changing the signature + every construction site. INSTEAD add a standalone JNI getter `getFusedSpeedMps()` mirroring `getLoopCorrectionVersion()` / `getCorrectedTrajectory()` (added this session): Kotlin `external fun` in `NativeBridge.kt` + a `Java_..._` shim in `native-lib.cpp` using the `std::shared_ptr<VioEngine> vision; {lock state_mutex; vision=g_vision;}` pattern → `vision->getTracker()->...`; Tracker getter reads under `pose_mutex_`.
- **The 5 m/s clamp may live in >1 place**: velocity research cited `EKFState.cpp:587-592`; the vio-specialist skill cited `EKFState.cpp:288-291`. **Grep for ALL `5.0` velocity clamps + the `velocity_clamped` counter and fix every site**, gated to scooter mode. Keep each as a raised sanity rail (don't delete — skill gotcha).
- **`in_vehicle_mode_` / `vehicle_speed_mps_` already exist** in `IMUPreintegrator` (a leaky forward-accel integrator + `getVehicleSpeed()`). Lever C must report the EKF `|v_G_|`, NOT resurrect `vehicle_speed_mps_`. Decide whether to reuse the `in_vehicle_mode_` flag or add a clean scooter-mode flag.
- **Reported speed today** = `NavSightViewModel.kt:397-407` (`|Δpos|/Δt` from `vio.x/z` = Tracker `global_t_`). Levers C/D edit this site.
- **Validate `v_G_` is sane first**: velocity divergence diagnostic at `EKFState.cpp:544-584`. `v_G_` accuracy depends on scale (Lever A). The SLAM-feature path is STARVED/deferred (`slam_promotions=0`, clone-pose redirect — see memory `session-2026-05-25-state`); Lever A's MiDaS scale constraint is deliberately INDEPENDENT of it.
- **MiDaS depth is available** via `Tracker::sampleMidasMetricDepth(u_img, v_img, z_out)` (affine fit ~855‰; returns false if not ready). Lever A uses it.
- **Heading is FIXED + committed — do NOT touch it.** Madgwick yaw drives the trajectory; `v_G_` is the separate EKF velocity.

**Validation**: real SCOOTER ride, phone-GPS speed as loose ground truth (keep GPS in app — `project_gps_jamming`). Per-lever falsifiers below. Build + real-ride-validate each lever before the next.

---

## ⏩ REVISED PLAN & IMPLEMENTATION STATUS — 2026-05-27 (supersedes the levers below)

**User directives that reshaped the plan:**
- **NO scooter mode** — speed is locomotion-agnostic (one path, no flag). Original Lever C (scooter-mode scaffold) is DROPPED.
- Keep MiDaS where it's strong (relative depth); fix its broken *metric* calibration.
- Get it RIGHT + VERY ACCURATE; validate on real walk/run before scooter.

**What we learned (diagnosis chain, DATA-VERIFIED this session):**
1. **`v_G_` (EKF velocity) DIVERGES** — ramps to the clamp on a walk (`velocity_clamped`=746, `|v_new|` pinned). Root: gravity-cancellation/attitude error integrates into phantom velocity; velocity is barely observed (SLAM starved, MSCKF rejecting, ZUPT only at stops). ⇒ `v_G_` is NOT a usable speed source ⇒ original **Levers 0 & C (both v_G_-based) are mooted**.
2. **Trajectory `global_t_` = ∫ speed·direction** (`disp = appliedScale·|t_vo|`, capped 2 m/s, projected on heading — Tracker.cpp ~2977/3082). Speed and the map share one scalar ⇒ the magnitude error is a SCALE error.
3. **MiDaS metric depth inherits the weak VIO scale** — the affine fit calibrates to `1/(z_vio·fused_scale)`; the fused scale (ScaleFuser) is structurally too small ⇒ MiDaS depths collapse to ~0.5 m. **ImageNet normalization IS present** (`DepthEstimator.kt:84-88`) — NOT the bug.
4. **Accel gives metric scale but its integrated velocity DRIFTS** (~1.2-1.7 m/s², ramps to 12 m/s on a walk — same root as v_G_). Raw accel can't measure slow walking; high-pass kills cruise. Confirmed on-device + offline (`scripts/analyze_accel_speed.py`).

**ARCHITECTURE (what's actually built — replaces the v_G_/scooter-mode levers):**
`reported speed = K · (visual relative speed)`, in `Tracker::updateDepthFlowSpeed` → `getFusedSpeedMps()` JNI → ViewModel EMA. **Isolated from EKF/trajectory** (zero regression risk).
- **Visual relative speed** = robust median over KLT points of `(per-point flow × its raw MiDaS RELATIVE depth)`. Drift-free, non-zero at cruise. Uses `sampleMidasRawDisparity()` (raw `depth_map_`, NOT the affine metric) — bypasses the affine-valid gate so it works even when the affine fit bails (e.g. the walk). The depth multiply cancels per-point depth so all points agree on the camera motion.
- **K = the one metric scale**, from the ACCELEROMETER in the clean post-ZUPT window: `K = accel_dist / visual_rel_dist` (ratio of ACCUMULATED path lengths since the stop — robust; the per-frame ratio swung 2.4× because MiDaS renormalises per frame). EMA-smoothed; **persisted in `event_summary`** (`midas_scale_k_milli` + min/max).
- **Drift removal (Step 1):** the accel integrator high-passes world linear accel (subtracts a low-pass = the gravity-leak+bias residual, `accel_drift_lp_`, τ=2 s) so accel velocity reflects real motion instead of ramping ⇒ K not inflated.
- **ZUPT** re-zeros at true stops.

**STATUS (uncommitted on `morad`; built+installed on R5CR70S3NNB):**
- ✅ Depth-flow estimator + `getFusedSpeedMps()` + ViewModel display (replaced position-differencing).
- ✅ Raw-disparity (relative) + sky/far masking; accel-K calibration; window-accumulated K (stable ±2.6% on a walk); K in `event_summary`.
- ✅ **Step 1 — accel drift removal** shipped; targets the walk reading 20 km/h (inflated K from drifted accel). **AWAITING walk validation** (expect walk ~5 km/h, `midas_scale_k_milli` walk≈run, was 3174 vs 827).
- ⏳ **Step 2 — looming / flow-divergence** (researched 2026-05-27; full recipe in memory `project_velocity_session_2026_05_26` + session tool-results): per-point `(ṙ/r)·Z_rel·K`; **gyro de-rotation** (Heeger-Jepson); **FOE anchored to EKF heading** when ill-conditioned (pure forward); gates r∈[0.05,0.8], disp>0.01, ≥10 inliers, sky mask. **Fuse with the essential-matrix path weighted by forward-motion fraction** — they're DUALS (E-matrix best lateral, divergence best forward; recoverPose is degenerate for forward = our case). Build after Step 1 validates.

**DEBT before commit:** the original Lever-0 clamp raise (5→15) and a v_G_-era else-branch ZUPT change are now unmotivated (v_G_ not displayed) — revert to committed `68b0b03` or replay-validate. The MAP is still drawn from `global_t_` with the small `appliedScale`; fixing map *size* (feed the clean accel-K into ScaleFuser) is a later validated step.

**Falsifiers (from `event_summary`):** walk no longer 20 km/h; `midas_scale_k_milli` walk≈run; `depth_flow_updates>0` on walk AND run; `depth_flow_total_mm` ≈ real distance; speed tracks GPS/known-distance, ~0 at stops.

---

## Current state (CODE-VERIFIED — 2026-05-26 PRE-IMPLEMENTATION SNAPSHOT, historical)
- EKF carries a proper world-frame velocity `v_G_` (rows 6–8). Propagation `EKFState.cpp:295-296` = `v_new = v_G_ + g·dt + R_GtoI_.t()·deltaV` (Forster) — the standard kinematic velocity. Gravity cancellation depends on correct tilt `R_GtoI_`.
- **BLOCKER #1 — 5 m/s clamp**: `EKFState.cpp:587-592` hard-clamps `|v_G_|` to **5 m/s = 18 km/h** (counter `velocity_clamped`). The specialist skill also flags this (a runaway-masking workaround). A 20–25 km/h scooter is clamped every step.
- **Reported speed ≠ filter velocity**: `NavSightViewModel.kt:397-407` computes `currentSpeedKmh = |Δpos| / Δt` from the Tracker `global_t_` trajectory (raw, ≥200 ms window, no smoothing). `v_G_` is unused for the UI. (`IMUPreintegrator` also has a pedestrian stride model `speed = stride×freq` and a leaky `vehicle_speed_mps_ += accel·dt ×0.997` — neither is the fused velocity either.)
- **ZUPT** (`EKFState.cpp:314-345`) zeros `v_G_` when stationary — a walking-only anchor; rarely fires on a cruising scooter.
- **Scale**: cross-keyframe triangulation + a working MiDaS affine fit (Step 4.2.1, inlier ~855‰) via ScaleFuser/ScaleEstimatorVI. SLAM features starved (`slam_promotions=0`, deferred) → scale leans on short-baseline triangulation + MiDaS.

## Why scooter speed is wrong (root causes)
1. The 5 m/s clamp truncates it outright.
2. Even below the clamp, reported speed is a raw difference of a weakly-scaled position, not the fused velocity.
3. **Scale degeneracy under cruise (LITERATURE)**: monocular metric scale is only observable under acceleration excitation (Fisher info ∝ accel²; Martinelli 2014; Mur-Artal VI-ORB-SLAM 2017). Constant-velocity cruising → scale weakly observable → drifts. Reported straight-line scale error ~9% vs figure-8 ~4.8% (directional; verify exact source). Smoother motion = worse scale = worse speed magnitude.
4. ZUPT (the walking speed anchor) vanishes on a scooter.

## Plan — ranked levers (cause → change → falsifier). Order: 0 → C → A → B → D.

> **⚠️ SUPERSEDED 2026-05-27 — see "REVISED PLAN & IMPLEMENTATION STATUS" above.** These were the original hypotheses. In practice `v_G_` diverges (Levers 0 & C, both v_G_-based, are mooted), and the working approach became **depth-weighted optical flow × accel-calibrated scale + (next) looming**. Lever A's "MiDaS as a scale constraint" survives in spirit (MiDaS *relative* depth + accel scale), and Lever B (non-holonomic) / Lever D (smoothing) remain valid future options. Kept below for the rationale + literature.

**Lever 0 — raise/gate the 5 m/s clamp [HIGH impact, LOW risk, DO FIRST]**
- Cause: `EKFState.cpp:587-592` clamps to 18 km/h.
- Change: in scooter mode raise to a cited scooter ceiling (~12 m/s ≈ 43 km/h); keep as a sanity rail (don't delete — skill gotcha), keep the counter.
- Falsifier: `velocity_clamped ≈ 0`; speed tracks phone-GPS instead of pinning at 18 km/h.

**Lever C — scooter mode scaffold + report smoothed `|v_G_|` [MED-HIGH, LOW-MED]**
- Cause: reported speed doesn't use the filter velocity (§ current state); ZUPT is walking-only.
- Change: a scooter-mode flag (manual toggle or auto from sustained speed) that (a) disables/raises ZUPT, (b) enables Levers A/B, (c) **reports smoothed `|v_G_|`** via a new JNI getter instead of position-differencing.
- Falsifier: `zupt_fired ≈ 0` while cruising; speed = smoothed `|v_G_|`, no position-jitter spikes; ZUPT re-enables at true stops.

**Lever A — MiDaS depth as a velocity/scale constraint + constant-velocity scale hold [HIGHEST impact, MED risk] — this is "MiDaS Phase 2 for speed"**
- Cause: scale degenerate under cruise (Fisher ∝ accel²); speed = scale × visual-rate, so scale error is a multiplicative speed bias.
- Change: inject the affine-fitted MiDaS depth as a **1-DOF constraint on `|v_G_|`/scale** (VI-Depth velocity-magnitude path, Wofk 2023) via `applyMSCKFUpdate` (never write `P_`). Add an excitation monitor (windowed body-accel variance); when low, **freeze/damp** scale adaptation and lean on the MiDaS anchor. Independent of the starved SLAM path.
- Falsifier: on a constant-speed segment, scale stays flat + speed within tolerance of phone-GPS; `midas_fused > 0`, affine inlier ≥ 500‰.

**Lever B — non-holonomic 2-DOF velocity update [HIGH, MED risk]**
- Cause: a scooter rolls forward → lateral + vertical body velocity ≈ 0; ZUPT anchor lost.
- Change: EKF update `z = [e_y, e_z]^T (R_GtoI v_G_) ≈ 0` (2-DOF on rows 6–8, attitude coupling 0–2), modeled on `updateZUPT`, with cited slip/vibration variance. Gate to scooter-mode + moving + not-turning + mount-stable; calibrate the forward axis to the phone's scooter mount. Sources: M2C-GVIO 2023, OpenVINS VIW (NHC part only, no wheel encoder), RINS-W 2019.
- Falsifier: lateral-body-velocity histogram centered at 0; closed-loop drift shrinks vs off; no degradation in real turns.

**Lever D — speed-reporting smoothing [LOW-MED, LOW risk]**
- Cause: raw `|Δpos|/Δt` is jittery + upward-biased.
- Change: low-pass / 3–5-sample median (~1 s) on the reported speed, cited constant, capped lag. `NavSightViewModel` only.
- Falsifier: smooth display, < 1 s lag to a real stop, standstill reads ~0.

## Anti-patterns (do NOT)
- Loosen the chi² gate or add magic clamps to "fix" speed — magnitude error is an upstream SCALE problem (Lever A), per the chi² lesson.
- Swap MiDaS→DA3 (deferred 2026-05-16, 722 ms CPU on S21 vs 100 ms budget).
- Delete the velocity clamp (keep as a raised sanity rail).
- Rely on the starved SLAM path for scale (it's deferred).

## Validation
Each lever on a REAL scooter ride with phone-GPS speed as loose ground truth (keep GPS in the app — `project_gps_jamming`).

## Sources
VINS-Mono (Qin T-RO 2018, arXiv:1708.03852); OpenVINS VIW-Odometry (Lee/Geneva IROS 2020); Martinelli VI-SfM observability (IROS 2013); Mur-Artal & Tardós VI-ORB-SLAM (RA-L 2017, arXiv:1610.05949); VI-Depth (Wofk et al. ICRA 2023, arXiv:2303.12134, github isl-org/VI-Depth); RINS-W (Brossard & Barrau IROS 2019, arXiv:1903.02210); M2C-GVIO (Satellite Navigation 2023). NOTE: the "9.2%/6.4%/4.8% scale-error-by-motion" figures are directionally consistent with observability theory but the surfaced arXiv id looked auto-generated — verify before formal citation.
