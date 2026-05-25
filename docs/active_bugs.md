# NavSight Active Bugs — Status as of 2026-05-25

**Last updated**: 2026-05-25 (heading saga root-caused + fixed; commit `4a9a212` on `morad`)
**Session shipped**: 12 fixes across 3 P0 bugs + 6 supporting changes (2026-05-21); heading fix (2026-05-25)

---

## 🟢 FIXED 2026-05-25 — the multi-session heading saga (root cause found)

### ✅ HEADING — gimbal-locked compass extraction + compass-snap (commit `4a9a212`)
**Symptoms (recurring across many sessions)**: heading drifts while stationary; V-shape / no-overlay on straight-and-back walks; ~95° offset + 200↔0° jumps vs a NOAA-compass reference.
**Root cause (data-proven)**: the rotation-vector heading was extracted with `getOrientation()`'s **flat-phone azimuth** (no `remapCoordinateSystem`), which **gimbal-locks** when the phone is held upright (NavSight's walking pose) → wrong azimuth + jumps. The prior `k=1.0` compass **SNAP** then forced the gyro yaw to follow that bad compass every frame, so disturbance flowed straight into the heading-projected trajectory → V-shape. The earlier "compass disturbed on returns" reading was this frame bug, **not** magnetic interference.
**Fix** (`SensorRepository.kt`, `IMUPreintegrator.cpp/.h`, `native-lib.cpp`, `NativeBridge.kt`, `Tracker.cpp`):
- **Gimbal-free RV extraction**: project the horizontal leading device edge to the ground plane instead of the flat azimuth (tilt-adaptive: upright → camera axis, flat → top edge). Forward axis validated against NOAA ground truth.
- **Gyro-primary + gated compass**: gyro owns the heading (tracks turns cleanly); the compass snaps once at startup for absolute heading, then corrects slow drift only when it agrees with the gyro within 35° (`kMagDisturbRejectRad`); larger disagreement = disturbed field → reject. Standard AHRS / compass-app design.
- Telemetry: `MAG_FUSE[INIT|FUSE|REJECT]`, `RV_SEND mode=FLAT|UPRIGHT`.
**Validation**: user bench-compared app heading to the NOAA mobile compass — matches in the walking hold; stationary heading stable; gimbal jumps gone. **Walk-overlay confirmation pending** on the next straight-and-back.
**Supersedes**: Option B (GPS-bearing init) below + the magnetometer-init-error analysis — the continuous gated compass now sets and maintains absolute heading, so the ~40° mag-init error Option B was designed to one-shot-correct no longer accumulates.
**Note**: temporary `RV_DIAG` candidate-axis logging left in `SensorRepository` (harmless; strip in a follow-up).

---

## 🟢 FIXED 2026-05-21 (validated on real walks)

### ✅ BUG-02 — Madgwick self-injected gyro bias
**File**: `Tracker.cpp:1357` (the REPLACE → ADD-and-ZERO pattern fix)
**Root cause**: `imu.setGyroBias(ekf_.getGyroBias())` was REPLACING the calibrated IMU bias (~0.18 rad/s) with the EKF residual (~0.007 rad/s), leaving ~0.17 rad/s of un-subtracted gyro → +46°/walk phantom drift.
**Fix**: ADD-and-ZERO per `EKFState.h:701-731` documented contract. Read EKF residual, add to IMU bias, write back to IMU, then zero EKF.
**Validation**: `bug02_walk_2026_05_21` — `ekf_bg_absorbed_total = 425` (was 0); trajectory loop-bearing delta dropped 113° → 11.6°.

### ✅ BUG-02b — Bug 5 visual→Madgwick sync unreachable
**File**: `Tracker.cpp:4172-4250` (hoisted Bug 5 nudge out of `is_static||translation_degenerate||is_pure_rotation` gate)
**Root cause**: Bug 5 was nested inside the gate that skips 67% of frames during typical wall-walking. Counter `madgwick_visual_yaw_nudges_total = 0` across all 6 pre-fix walks.
**Fix**: hoisted Bug 5 below the gate; recomputes `yaw_meas_b5` locally; still inside the `if (drift < 3°)` gyro-consistency outer gate.
**Validation**: `bug02b_walk_2026_05_21` — `madgwick_visual_yaw_nudges_total = 27`; user-visible heading drift dropped +40° → +9.7° over 2 loops.

### ✅ BUG-NEW-PG — Pose graph doesn't converge
**File**: `PoseGraph.h:202-203` (raised `SIGMA_POS_FLOOR_SQ` 1e-6 → 2.5e-3, `SIGMA_YAW_FLOOR_SQ` 1e-6 → 3e-4)
**Root cause**: EKF covariance INCREMENT between consecutive keyframes was being clamped to a 1 mm² floor when MSCKF updates decreased covariance, making odom info_xy = 1,000,000. Loop edge info ≈ 0.25. **4 million× ratio** meant loop edges couldn't deform the chain. `iters=2 ratio=1.000 max_corr=4mm` per solve.
**Fix**: raised floors to realistic per-keyframe odometry noise (5cm σ_xy, 1° σ_yaw) derived from visual VO noise + IMU integration over 0.5s keyframe interval.
**Validation**: `bug_newpg_walk_2026_05_21` — pose graph iters 2→2-4, ratio 1.000→0.931-0.999 on early solves, `pose_graph_max_correction_mm = 2838` (was tiny), trajectory loop-bearing delta 11.7°→**2.1°** (loops now overlay).

### ✅ Front-end parallax gate (root cause of recoverPose unit-norm-t bug)
**File**: `Tracker.cpp:2162` — replaced dead `cv::norm(t_vo) < 0.001` (t_vo is always 1.0) with mean-parallax-angle gate
**Threshold**: `kVisualMinParallaxRad = 0.01` (~0.57°), cited OpenVINS UpdaterSLAM `min_parallax_ratio`
**Counter**: `visual_translation_degenerate_total`

### ✅ SLAM promotion parallax gate
**File**: `Tracker.cpp:~3372` — added parallax-angle check before two-view midpoint triangulation
**Threshold**: `kSlamPromoMinParallaxCos = 0.99985` (~1°), cited ORB-SLAM3 Mur-Artal & Tardós 2017 §V.B
**Counter**: `slam_promo_rejected_parallax_total`

### ✅ Bug 3 — Post-PnP rotation residual sanity gate
**File**: `Tracker.cpp:~6484` — gate before `updateAbsolutePose` rejects target_R with |r_R| > π/2
**Catches**: planar-scene essential-matrix π-flip ambiguities before they reach EKF chi²
**Counter**: `loop_closure_rejects_rot_sanity_total`

### ✅ Bug 4 (chi² gate REVERTED)
**File**: `EKFState.cpp` (both `updateRelativeRotation` and `updateGravityAlignedYaw` chi² blocks commented as LEGACY)
**Reason**: chi² gates self-defeat as P grows from rejection→propagation feedback loop. EKF p99 yaw-rate went UP (53→78°/s) when shipped. Replaced by gyro-consistency gates at the sensor-disagreement layer.

### ✅ Gyro-vs-visual consistency gate (per-frame `updateRelativeRotation`)
**File**: `Tracker.cpp:2380` — compares R_vo_body to `imu_delta.deltaR`, rejects if disagreement > 2°
**Counter**: `visual_relative_rotation_gyro_mismatch_total`
**Derivation**: Madgwick yaw-rate p99 × 33ms + corner-turn rate + safety = 2°

### ✅ KF_HEADING_CORR drift gate tightened (20° → 3°)
**File**: `Tracker.cpp:3952` — old 20° gate accepted nearly every visual-yaw measurement
**Derivation**: Madgwick p99 × 0.5s + bias drift + safety 2.7× = 3°

---

## 🟡 P1 OPEN — User-visible, would improve experience

### BUG-01 — Orange dot flicker (descriptor staleness)
**Investigator**: Agent 1 (`docs/active_bugs/agent_01_descriptor_matching.md`)
**Root cause**: `Landmark.descriptor` never updated after first observation. Per-frame matches get 62.9% hits with 0.3-2s old descriptors; Step 7.1 with 30-80s old descriptors gets 0.05% hits.
**Recommended fix**: Multi-descriptor median, updated in `LandmarkMap::touchLandmark` on every match (ORB-SLAM3 `MapPoint::ComputeDistinctiveDescriptors` pattern). HIGH impact, LOW risk.

### BUG-01 SUB-A — Step 7.1 spatial_miss 67-79% (geometric, NOT appearance)
**Investigator**: Agent 1 §3 + §5.2 (`docs/active_bugs/agent_01_descriptor_matching.md`)
**Root cause**: dominant Step 7.1 failure is geometric — projected landmark lands in-image but no FAST corner exists within `kGeomMatchRadiusPx = 15px`. Three mechanisms: (1) EKF pose drift at 30s lookback shifts projections beyond 15px gate, (2) lighting / FAST repeatability between visits, (3) pose-graph back-write may not deliver to matched keyframe. Median min-Hamming-in-radius is 106 / 256 bits (random baseline 128) — the spatial gate is so loose that the matched kp is usually a DIFFERENT physical feature; descriptor verification correctly rejects but there's nothing for it to actually accept.
**Recommended fix**: raise `kGeomMatchRadiusPx` to 30-40 px (derived from EKF pose-error budget at 30s lookback: 17°/loop × 30s × 0.5 m/s ÷ 10m depth × focal ≈ 25px) AND tighten `kGeomDescriptorMaxDistance` from 50 to 35-40 (ORB-SLAM3 "very distinctive" threshold). Loosening one without tightening the other admits false positives. ~10 LOC, MEDIUM risk (PnP false positives — mitigated by existing `kPnpMinInliers=15`).
**Falsifier**: `loop_closure_geom_accepts > 0` AND `hamming_miss / hamming_pairs` ratio drops from ~50:1 to <5.

### BUG-04 F1+F2 — UI render flicker (per-keyframe is_observed refresh)
**Investigator**: Agent 4 (`docs/active_bugs/agent_04_render_pipeline.md`)
**Root causes**:
- F1: `is_observed` boolean refreshes only at keyframes (~1Hz) → dots step orange↔gray
- F2: 5-px proximity refresh yields only 9-15% of observed-dot renders → dots stick at last KF pixel
**Recommended fixes**: per-frame `is_observed` refresh + widen 5px refresh radius
**Note**: This couples with BUG-01 — fixing descriptor matching raises the anchor-hit ratio from 55-61% upward, halving flicker by itself.

### Heading confidence indicator (NEW — UX)
**Context**: today's walks proved visual evidence correctly converges Madgwick to truth even when magnetometer init is off by 30-50° (indoor magnetic anomalies). User can't tell "heading is correcting" from "heading is drifting".
**Recommended fix**: Kotlin UI badge showing heading confidence (orange→green) ramping from session start. Don't change algorithm; just communicate state.
**Cost**: ~50 LOC Kotlin

---

## 🟢 P2 OPEN — System correctness improvements

### BUG-03 — MiDaS Phase 2 live update (designed, not implemented)
**Investigator**: Agent 3 (`docs/active_bugs/agent_03_midas_slam.md`)
**Status**: complete architecture design with H matrix, σ_m derivation, wire-up site. ~140 LOC, 2 new counters.
**Threshold**: chi²(0.95, 1-DOF) = 3.841, σ_m = 0.5m × inverse-inlier-ratio (cited VI-Depth Wofk et al. ICRA 2023 §5.1)
**Wire-up**: `Tracker.cpp:~3670` after `applySlamLiveBatch`, gated on `isParallaxBelowThreshold(slot)` to avoid double-update
**Need**: addresses ρ degeneracy during prolonged axial walks (typical in NavSight's wall-following scenarios)

### BUG-06 Part C — LC soft position nudge
**Investigator**: Agent 6 (`docs/active_bugs/agent_06_lc_soft_correction_and_trajectory.md`)
**Status**: design ready, ~20 LOC. Mostly redundant after Bug 3+5 but addresses 1-in-156 chi² outlier case where mid-ramp p_G shift produces m²_p=31.7 rejection.
**Decision**: low priority; only fires on 0.6% of LC events.

### Hidden Bug #3 — SLAM promotion RMS chokepoint (separate from MiDaS Phase 2)
**Investigator**: Agent 5 §Hidden Bug #3 (`docs/active_bugs/agent_05_cross_walk_analysis.md`)
**Root cause**: across W3-W6, **60-69% of SLAM promo candidates are rejected on the RMS criterion**, NOT the parallax gate. The W4 parallax gate added 18.8% upfront reject but didn't move the dial — RMS was already the chokepoint. Result: promotions per walk = 0-24 out of 100,000+ candidates (≈0.001-0.02% promotion rate). MiDaS depth pipeline starves as downstream consequence (`midas_depth_samples`: 12 → 1 → 0 → 2 for W3→W4→W5→W6).
**Status**: needs investigation before BUG-03 (MiDaS Phase 2) ships — Phase 2 helps ρ refinement but cannot fix the upstream promotion-funnel starvation. Suspect: `slam_promo_rms_milli_p95 = 2904` (the 1.5 px RMS gate is too tight, OR the EKF clone poses being passed to triangulation are themselves drifted, inflating reprojection RMS).
**Action**: dump per-walk RMS distribution and find whether tightening the per-frame visual gates (already shipped) has changed the RMS landscape, or whether the gate constant itself needs re-derivation.

### ⚪ GPS-bearing-aided initial heading (Option B) — SUPERSEDED 2026-05-25
**SUPERSEDED** by the gated-compass heading fix (see FIXED 2026-05-25 at top). The continuous gyro-primary + gated-compass heading now sets the absolute heading from the compass at startup AND corrects drift continuously, so the ~40° mag-init error this option was designed to one-shot-correct no longer accumulates. Kept below for history; revisit only if a future GPS-aided refinement is wanted on top of the compass.
**Scope**: STARTUP ONLY, mirrors existing magnetometer init pattern. NOT continuous use.
**Rationale (UPDATED 2026-05-21 — walks confirmed OUTDOOR)**: today's walks were near-house outdoors with GPS available. Cross-checked GPS-derived bearing against Madgwick on `bug_newpg_walk_2026_05_21` (GPS coverage 98%, median accuracy **4.4 m**):

| t_s | GPS bearing | Madgwick hdg | Δ |
|---|---|---|---|
| 28.7 | +71.6° | +2.7° | +68.9° |
| 33.8 | +53.9° | +5.9° | +48.0° |
| 41.0 | +16.5° | +343.2° | +33.2° |
| 66.2 | +207.9° | +180.8° | +27.2° |
| 86.6 | +42.3° | +0.2° | +42.0° |
| 95.4 | +9.7° | +359.2° | +10.5° |

Consistent positive bias of **30-50°** between GPS direction-of-travel and Madgwick heading. Part of this is user-holding-phone-at-angle relative to walking direction, but the persistent ~40° baseline matches the magnetometer-init error magnitude observed in the heading-convergence analysis. The system's world frame is rotated ~40° from true north because mag-init at session start was off by that much.

**GPS quality across today's 3 walks**:

| Walk | GPS coverage | Median accuracy | Notes |
|---|---|---|---|
| bug02_walk | 80% | 700m | GPS still cold-starting |
| bug02b_walk | 85% | 95m | mixed quality |
| bug_newpg_walk | 98% | **4.4m** | excellent — Option B would have triggered |

**Conclusion**: Option B would have set the system's world frame to true-north-aligned from session start, making Madgwick heading match GPS bearing for the rest of the walk. The +32° heading drift the user observed wouldn't exist.

**Design — fallback hierarchy (CRITICAL: magnetometer is the always-available bedrock)**:

| Priority | Source | Condition | Action |
|---|---|---|---|
| 1 | **Magnetometer (ALWAYS)** | session start, mag reading available | Set initial Madgwick yaw immediately (current behavior, unchanged) |
| 2 | GPS bearing (when available) | fix accuracy < 10m AND speed > 0.5 m/s for ≥ 2 consecutive seconds, fires ONCE | One-shot nudge: replace Madgwick yaw with GPS-derived bearing |
| 3 | Visual yaw (already shipped) | per-keyframe, gyro-consistency drift < 3° | Bug 5 continuous nudges (Tracker.cpp:4172) |

**Critical: magnetometer remains the ALWAYS-AVAILABLE init source**. GPS-bearing is a one-shot REFINEMENT that fires only if GPS warms up and the user starts moving. If GPS never reaches the quality bar (indoor, jammed, urban canyon, cold start that never completes), the system stays on magnetometer-init exactly like today. Zero regression for GPS-denied scenarios.

**Decision flow at runtime**:
```
session start:
  imu.setMagYaw(magnetometer_reading + declination)   ← bedrock, always fires
  mag_init_done = true

every 1s during walk:
  if (gps_fix_acc < 10m && gps_speed > 0.5 m/s for ≥ 2s
      && !gps_bearing_oneshot_fired
      && vio_initialized):
    bearing = compute_gps_velocity_bearing()
    delta = bearing - imu.getHeading()
    imu.nudgeMadgwickYawAroundWorldZ(delta)            ← one-shot refinement
    gps_bearing_oneshot_fired = true

if (gps fails to reach the bar):
  magnetometer-init remains the world-frame reference
  visual evidence continues to refine via Bug 5 (per-keyframe)
  Nothing breaks. NavSight stays GPS-denied-functional.
```

**Implementation locations**:
- In `Tracker::setInitialHeading` (or new `seedHeadingFromGpsBearing`), accept a GPS-derived bearing from Kotlin
- `SensorRepository.kt` tracks GPS history and computes bearing when criteria met
- New `nativeApplyGpsHeadingNudge(double bearing_rad)` JNI call → C++ side calls `imu.nudgeMadgwickYawAroundWorldZ(delta)` once
- Counter: `gps_heading_oneshot_fired` (0 or 1 per session — if 0, mag-only path was used)

**Hard guarantees per `project_gps_jamming` and `feedback_no_magnetometer` memories**:
- ✓ Magnetometer ALWAYS used at session start (no change to existing behavior)
- ✓ GPS is REFINEMENT, never required
- ✓ GPS NEVER used during tracking (only one-shot at startup)
- ✓ GPS-jammed scenarios behave identically to today (mag-init + visual refinement)
- ✓ Indoor scenarios behave identically (no GPS fix → no nudge → mag-only)
- ✓ Cold-start with bad GPS for full walk → no nudge → mag-only

**Cost**: ~80 LOC Kotlin (GPS bearing computation + JNI call + criteria gate) + ~30 LOC C++ (acceptance gate + one-shot nudge + counter)

---

## 🟦 P3 — Latent / cleanup (low priority, low risk)

### F3 — `nearby_ids.empty()` clears ids but not pixels (size-invariant breach)
**Investigator**: Agent 4 §C row F3 (`docs/active_bugs/agent_04_render_pipeline.md`)
**File**: `Tracker.cpp:4717-4720`
**Status**: latent — safe-by-luck today because `getLastObservedLandmarkPixel` iterates ids first, but the `ids.size() == pixels.size()` invariant is temporarily false on every keyframe with `nearby_ids.empty()`. Crash risk if iteration order ever changes.
**Fix**: clear pixels alongside ids, OR add `assert(ids.size() == pixels.size())` at lookup site. ~3 LOC.

### F4 — Per-keyframe race window in `getLastObservedLandmarkIds`
**Investigator**: Agent 4 §C row F4 (`docs/active_bugs/agent_04_render_pipeline.md`)
**Files**: `native-lib.cpp:1306` (reader), `Tracker.cpp:5075` (producer)
**Symptom**: brief 1-frame color flip on keyframe boundary because the overlay snapshot reads `observed_ids` independently of `ensureOverlaySnapshot`, so two consecutive overlay calls 33ms apart may see different orange/gray populations.
**Fix**: move `observed_ids` snapshot inside `ensureOverlaySnapshot` so the entire overlay tick reads a single coherent snapshot. ~10 LOC.

### BUG-01 SUB-B — Pose-graph back-write may not reach matched keyframe
**Investigator**: Agent 1 §3 mechanism #3 (`docs/active_bugs/agent_01_descriptor_matching.md`)
**Suspected mechanism**: `LandmarkMap::applyKeyframePoseCorrection` shifts landmarks observed in `kf_id`, but the current-frame projection uses current EKF pose. The stored keyframe's pose may not have absorbed the corresponding correction → systematic offset → contributes to BUG-01 SUB-A spatial_miss rate.
**Status**: needs verification post-BUG-NEW-PG (which raised PoseGraph floors so it now actually converges). Counter `pose_graph_apply_calls > 0` proves back-write fires; what's not proven is whether the corrected keyframe pose is delivered to LandmarkMap before the next projection.
**Action**: log per-keyframe `R_world_cam` delta pre/post-correction; verify LandmarkMap entries reflect it.

### Agent-2 H3 — `refineGyroBiasDuringZUPT` averages raw `gyro_buf_`, not bias-corrected
**Investigator**: Agent 2 §Fix 3 / H3 (`docs/active_bugs/agent_02_madgwick_bias.md`)
**File**: `IMUPreintegrator.cpp:1068-1074`
**Mechanism**: now that BUG-02 ADD-and-ZERO correctly absorbs EKF residual into `gyro_bias_`, the ZUPT EMA toward raw-mean of `gyro_buf_[i].x` partially undoes the absorption (~1% per ZUPT call). At 0.3 Hz ZUPT vs 5 Hz absorption, the push wins — but the cleanup is principled.
**Fix**: average `(gyro_buf_[i].x − gyro_bias_.x)` instead of raw. ~5 LOC. Defer until BUG-02 has walk-validated for several more sessions.

---

## 🔵 BY-DESIGN, NOT BUGS

### HANDOFF #3 — Trajectory "freezes" at end of walk
**Status**: NOT a bug. By-design `global_t_` freeze during `is_static` intervals at `Tracker.cpp:2845`. User stops moving → ZUPT detects → trajectory stops advancing → recorder writes identical samples → looks like "trajectory peaked and froze".
**Optional cosmetic**: prune trailing identical samples in `NavSightViewModel.kt:336-353` (~5 LOC) for cleaner plot rendering.

---

## 🟦 KNOWN COSMETIC ISSUES

### `extrinsics_rotation_angle_mdeg` always shows 90000
Counter compares current R_bc to OLD initial baseline. R_bc was changed 2026-05-19 to a 90°-rotated value; counter baseline not updated. Reads "90° drift" but R_bc isn't actually drifting. Fix: update baseline in counter computation.

### R_bc sub-degree miscalibration
Per `EKFState.cpp` comment from 2026-05-19, online R_bc calibration (Step 8b) is SKIPPED. Could absorb residual orange-dot sub-degree drift. Separate session.

---

## Today's walks (validation data)

| Walk | Stage | Heading drift | Loop-bearing delta |
|---|---|---|---|
| heading_walk_1_2026_05_20 | pre-any-fix | — | (1D trajectory) |
| heading_walk_2_2026_05_20 | pre-any-fix | — | (1D trajectory) |
| parallax_fix_walk_2026_05_20 | post front-end parallax | — | 15° |
| promo_parallax_walk_2026_05_21 | post SLAM-promo parallax | — | n/a |
| bug3_walk_2026_05_21 | post Bug 3 rot-sanity | — | n/a |
| bug4_walk_2026_05_21 | post Bug 4 chi² (now REVERTED) | +40° | 113° (regression — reverted) |
| **bug02_walk_2026_05_21** | post BUG-02 ADD-and-ZERO | +20.9° / +39.9° | 11.6° |
| **bug02b_walk_2026_05_21** | post BUG-02b Bug-5 unhide | **+9.7°** | 11.7° |
| **bug_newpg_walk_2026_05_21** | post BUG-NEW-PG pose-graph floor | +32° (mag-init correction, see analysis) | **2.1°** |

## Per-agent reports

- [Agent 01 — Descriptor matching](active_bugs/agent_01_descriptor_matching.md)
- [Agent 02 — Madgwick gyro bias](active_bugs/agent_02_madgwick_bias.md)
- [Agent 03 — MiDaS Phase 2 + SLAM sparsity](active_bugs/agent_03_midas_slam.md)
- [Agent 04 — UI render pipeline](active_bugs/agent_04_render_pipeline.md)
- [Agent 05 — Cross-walk analysis](active_bugs/agent_05_cross_walk_analysis.md)
- [Agent 06 — LC soft correction + trajectory](active_bugs/agent_06_lc_soft_correction_and_trajectory.md)

## Recommended next session order (updated 2026-05-25)

- ~~GPS-bearing init (Option B)~~ — **SUPERSEDED** by the 2026-05-25 gated-compass heading fix
- ~~BUG-01 multi-descriptor median~~ — **DONE** in `f86c38d` (verified-only descriptor refresh + recompute)
- ~~Heading~~ — **DONE** in `4a9a212` (gimbal-free + gyro-primary + gated compass)

Latest walk (`rv2_walk_2026_05_25`) counters show the two remaining user-visible systems are both at **zero**:

1. **BUG-01 SUB-A** — widen `kGeomMatchRadiusPx` (15→30-40) + tighten `kGeomDescriptorMaxDistance` (50→35-40). `loop_closure_geom_accepts = 0` → **loops never close → never overlay.** P1, ~10 LOC, derived thresholds, MEDIUM risk. **← top candidate (loops overlay)**
2. **Hidden Bug #3** — SLAM promotion RMS chokepoint. `slam_promotions_total = 0` → **no orange dots are ever created/anchored.** Needs RMS-distribution dump first. **← top candidate (orange dots)**
3. **BUG-04 F1+F2 render fixes** — per-frame `is_observed` + widen refresh radius (companion to dots)
4. **BUG-03 MiDaS Phase 2** — depth observability during axial walks (AFTER Hidden Bug #3)
5. **Heading confidence indicator** (UX) — optional now the heading is solid
6. **P3 cleanups (F3, F4, Agent-2 H3, BUG-01 SUB-B)** when convenient

## Bug count by status (end of 2026-05-21)

| Status | Count |
|---|---|
| ✅ FIXED today | 9 |
| 🟡 P1 OPEN (user-visible) | 5 |
| 🟢 P2 OPEN (correctness) | 3 |
| 🟦 P3 OPEN (latent / cleanup) | 4 |
| 🔵 BY-DESIGN | 1 |
| 🟦 COSMETIC | 2 |
| **Total tracked** | **24** |
