# NavSight Active Bugs — Status as of 2026-05-21 (end of day)

**Last updated**: 2026-05-21 18:48 (after 5 validation walks today)
**Session shipped**: 12 fixes across 3 P0 bugs + 6 supporting changes

---

## 🟢 FIXED today (validated on real walks)

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

### 🟡 GPS-bearing-aided initial heading (Option B — PRIORITY HIGH after GPS analysis)
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

**Design**:
- In `Tracker::setInitialHeading` (or new `seedHeadingFromGpsBearing`), accept a GPS-derived bearing from Kotlin during init
- Use GPS when: fix accuracy < 10m AND user has been moving > 0.5 m/s for ≥ 2 consecutive seconds
- Fall back to magnetometer when no GPS fix (true indoor / jammed scenarios)
- Allow re-arming if mag-init fired but GPS later becomes good — push a one-shot Madgwick nudge to GPS-derived value
- **NEVER use GPS during tracking** — preserves the GPS-denied design philosophy (per `project_gps_jamming` memory)
**Cost**: ~80 LOC Kotlin (GPS bearing computation + JNI call) + ~30 LOC C++ (acceptance gate + one-shot nudge)

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

## Recommended next session order

1. **GPS-bearing-aided initial heading** (Option B) — addresses today's mag-init error, ~110 LOC, low risk, startup-only
2. **Heading confidence indicator** (UX) — communicates the converging-to-truth behavior to the user, ~50 LOC
3. **BUG-01 multi-descriptor median** — fixes orange dot flicker at the source, high impact
4. **BUG-04 F1+F2 render fixes** — companion to BUG-01 for full flicker elimination
5. **BUG-03 MiDaS Phase 2** — depth observability during prolonged axial walks
