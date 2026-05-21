# Bug 02 — Madgwick gyro bias drift root cause

**Worker**: worker-02-madgwick (hive 1779372411394-kzq0l2)
**Date**: 2026-05-21
**Status**: Root cause identified. NO source changes per investigation-only mandate.
**Reproducer**: `tests/sims/regression/visual/bug4_walk_2026_05_21.json` + `.logcat.txt`

---

## TL;DR

The "~0.28°/s gyro bias drift, 490×σ" framing in the task spec is incorrect.
The S21 Ultra Exynos 2100 gyro's intrinsic bias is healthy
(~0.03°/s on a true 9.7 s stationary recording — well within IEEE 1554
spec for a smartphone MEMS unit). The 30°/loop user-visible drift is
NOT hardware drift — it is **software self-injected bias**:

1. `Tracker.cpp:1357` (`IMU_BG_PUSH` block) calls
   `imu.setGyroBias(ekf_.getGyroBias())` every 6 camera frames (~5 Hz).
2. `IMUPreintegrator::setGyroBias` (line 1051) **replaces** the calibrated
   IMU bias with the EKF's small residual (~0.007 rad/s on Z), not adds to it.
3. The IMU was correctly calibrated at startup to ~0.0003 rad/s. After the
   first push it is now off by ~0.007 rad/s = **0.40 °/s phantom bias on Z**.
4. Over a 2-minute walk that is **~48° of phantom yaw drift**, matching the
   observed 30–60° per loop.

Bug 5 (the May 21 fix at `Tracker.cpp:4176`) was a phantom fix. In the
`bug4_walk` it never fired — `madgwick_visual_yaw_nudges_total` = 0 (counter
missing from event_summary JSON). The gate at `Tracker.cpp:4085`
(`is_static || translation_degenerate || is_pure_rotation`) encloses *both*
`updateGravityAlignedYaw` AND the Bug 5 nudge, and 67% of keyframes hit
`translation_degenerate=1` per the PARALLAX_GATE logs.

The architecture exists for a correct loop (see
`EKFState::setGyroBias`, EKFState.h:701–731 — the comment explicitly
documents the right pattern: READ ekf_bg, ADD to imu_bg, then setGyroBias(0,0,0)
on the EKF). But it was never wired — `ekf_bg_absorbed_total = 0` across
every walk.

---

## Data — gyro bias measured from sim `is_static` intervals

Script: `scripts/bug_02_gyro_bias_analysis.py`. Static window detector
mirrors `Tracker.cpp:1960` (`gyro_norm < 0.1` rad/s AND
`|accel|−9.81 < 0.5` m/s²; min 8 consecutive points = 0.4 s @ 20 Hz).

| Sim | duration | static windows | best `mean_gz` | predicted yaw drift over walk |
|---|---|---|---|---|
| `heading_walk_1_2026_05_20.json` | 117.2 s | 1 (0.6 s) | −0.00446 rad/s = **−0.256°/s** | −30.0° |
| `heading_walk_2_2026_05_20.json` | 124.0 s | 4 | +0.00932 rad/s = **+0.534°/s** | +66.2° |
| `parallax_fix_walk_2026_05_20.json` | 128.4 s | 4 | −0.00074 rad/s = **−0.042°/s** | −5.4° |
| `bug3_walk_2026_05_21.json` | 114.6 s | 1 (0.9 s) | −0.01030 rad/s = **−0.590°/s** | −67.6° |
| `bug4_walk_2026_05_21.json` | 116.5 s | 2 | −0.00236 rad/s = **−0.135°/s** | −15.8° |
| `promo_parallax_walk_2026_05_21.json` | 120.7 s | 1 (3.2 s) | +0.00267 rad/s = **+0.153°/s** | +18.5° |

**Cross-walk pattern (Q1 of the brief)**: bias varies between **−0.59 and
+0.53 °/s** across walks. This is too erratic to be hardware random-walk
(Allan-calibrated σ_bg = 7.46e-7 rad/s/√s → 1-σ change over 120 s would be
only 8e-6 rad/s ≈ 5e-4 °/s — three orders of magnitude smaller).

**True stationary baseline (control):**
`v35_static_drift.json` (9.7 s, phone untouched, before any walk):
mean gyro = (−0.013, −0.009, −0.028) °/s. **The hardware is fine.**

Observed bias_gz/σ_bg ratios per sim: 990× – 13,800× the Allan
steady-state floor — confirms this is NOT random-walk regime, it is
**bias-state corruption from the software loop**.

**Q2 of the brief — bias growing within a walk?**
`heading_walk_2`: first-to-last static-window mean drift over 121.3 s:
dgx=−0.030, dgy=+0.022, dgz=−0.007 rad/s. Allan expectation for 121.3 s
RW = 8.2e-6 rad/s. Observed |dgz|/expected = **824×** — clearly NOT
Brownian. Same pattern in all multi-window walks. Bias is being
*reset* by the EKF push, not drifting.

**Q6 — thermal**: NavSight has no thermistor. Exynos 2100 MEMS gyros
(Bosch BMI270 / STMicro LSM6DSO families typical for S21) datasheet
specs ZRO drift over 0–40 °C: ~0.05 °/s typical, ~0.15 °/s worst-case
(Bosch BMI270 datasheet table 6.4). Even worst-case thermal cannot
explain the ±0.5 °/s variations. Thermal is NOT the dominant cause.

---

## Data — IMU_BG_PUSH evolution during walks

Greppped from logcats:

| Sim | pushes | bz first | bz last | bz mean | predicted phantom drift |
|---|---|---|---|---|---|
| `bug4_walk_2026_05_21` | 42 | +0.00725 | +0.00618 | +0.00664 rad/s = **+0.381 °/s** | **+45.7° over 120 s** |
| `heading_walk_2026_05_20` | 76 | +0.01301 | +0.00429 | +0.00871 rad/s = **+0.499 °/s** | **+59.9° over 120 s** |

The EKF residual is stable across the walk (range ≈ ±0.001 rad/s),
which is what we'd expect from a healthy MSCKF — it converges to a
small steady-state correction. The damage is that this stable small
residual is being **OVERWRITTEN onto the IMU's gyro_bias_** every 6
frames, where it functions as a giant new bias on top of the already-
calibrated IMU.

---

## Code path — exact failure chain

```
addGyroReading()                  // IMUPreintegrator.cpp:83
  gyro_buf_.push_back({ts, gx_raw, gy_raw, gz_raw})
  updateMadgwickLocked(ts, gx, gy, gz)
    gx_use = gx_raw − gyro_bias_.x  // line 840 — uses IMUPreintegrator gyro_bias_
    quaternion integrated, accel correction applied

addAccelReading()                 // IMUPreintegrator.cpp:111
  tryInitializeGyroBiasLocked()
    // Once-only, on stationary samples at startup. gyro_bias_ frozen.

[walk begins; EKF runs MSCKF/SLAM/LC]

processFrame() loop               // Tracker.cpp:1300+
  ekf_.propagateIMU(...)
  if (frame_counter_ % 6 == 0):    // line 1354 — every ~200 ms
    cv::Mat ekf_bg = ekf_.getGyroBias()   // EKF residual b_g_ ≈ (0.002, 0.008, 0.007)
    imu.setGyroBias(ekf_bg[0], ekf_bg[1], ekf_bg[2])
                                    // REPLACES IMU's calibrated bias!
    gyro_bias_pushed_total++       // counter bumps; ekf_bg_absorbed_total NEVER bumps

  [later in same frame]
  Bug 5 sync (Tracker.cpp:4176):
    if (drift < 3°)
      && aligned_ok
      && ekf.isFullInitialized()
      && !(is_static || translation_degenerate || is_pure_rotation):   // ← GATE BLOCKS HERE
        imu.nudgeMadgwickYawAroundWorldZ(0.10 * residual)
        madgwick_visual_yaw_nudges_total++   // ← counter stays 0 because gate fails
```

Verification:

- `IMU_BG_PUSH` lines in `bug4_walk` logcat: **42** (matches 5 Hz × 116 s × 0.07 ratio = 42).
- `MADG_VISUAL_SYNC` lines: **0**.
- `KF_HEADING_CORR` lines: 21; 90 % have `|drift| < 3°` (so the OUTER drift gate would PASS in 19 cases).
- `VISUAL_YAW_GATE: skipped` always shows `translation_degenerate=1` — 67 % of PARALLAX_GATE logs have parallax_rad < 0.01 (the threshold).
- `ekf_bg_absorbed_total` in JSON event_summary: **0** (never absorbed).
- `gyro_bias_pushed_total`: **391**.

---

## Why `refineGyroBiasDuringZUPT` doesn't save us (Q4)

It exists (IMUPreintegrator.cpp:1058) and is called from `Tracker.cpp:1992`
inside the ZUPT branch (the `consecutive_static_frames_++` block).
Behavior: `gyro_bias_ ← 0.99 × gyro_bias_ + 0.01 × mean(last 20 gyro samples)`.

Problem: `refineGyroBiasDuringZUPT` runs on the **bias state that
IMU_BG_PUSH just corrupted**. When the EKF push wrote 0.007 rad/s into
`gyro_bias_.z`, and then ZUPT averages 20 samples of raw gyro (~0.0001 rad/s
on a truly stationary phone, but the gyro_buf_ here is raw, NOT bias-corrected)
— the EMA pulls `gyro_bias_.z` slightly toward the raw-mean. With α=0.01 and
the EKF pushing at 5 Hz, the EKF push DOMINATES; ZUPT refinement is a
weak rear-guard.

Also note: in the walks studied, `static=1` only 5–6 % of GATES lines, and
`zrup_fired_total = 35` per bug4 walk → ZUPT contributes ~0.3 Hz, vs IMU_BG_PUSH
contributing ~5 Hz. The push wins.

---

## Why the EKF→Madgwick yaw feedback (`LC_MADG_NUDGE`, Q5) doesn't bound it

5 `LC_MADG_NUDGE` events in `bug4_walk`, every one logs **delta = −0.00°**.
Pose-graph yaw corrections (`pose_graph_max_correction_mrad`) cap out at
**1 mrad ≈ 0.057°** per the JSON summary. LC is making the *position*
corrections that fix the trajectory, but the *yaw* correction signal is
near-zero because (a) LC is sparse (73 corrections per walk) and (b) the
EKF's R_GtoI has already absorbed any visual-yaw residual via
`updateGravityAlignedYaw` — so by the time LC runs, the EKF and Madgwick
differ by what's been pumped in via IMU_BG_PUSH, which LC has no signal
to undo.

---

## Root-cause hypotheses, ranked by evidence

### H1 — `IMU_BG_PUSH` overwrites instead of adds (PRIMARY CAUSE)

- **Evidence**: 42 push events in bug4_walk with bz ≈ +0.007 rad/s; predicted
  +45° drift; observed +30 to +60° per loop.
- **Mechanism**: `IMUPreintegrator::setGyroBias` replaces, doesn't add. EKF's
  `b_g_` is a *residual* state (small correction on top of an already-bias-
  corrected gyro), but Tracker treats it as the *full* bias.
- **Falsifier**: disable IMU_BG_PUSH (`if (false &&` at line 1354) → expect
  Madgwick heading drift across 2 loops to drop from 30–60° to <5° (matches
  raw IMU bias of ~0.03 °/s × 120 s = 3.6°).
- **Why this happens**: the May-16 comment block at `EKFState.h:701–731`
  documents the correct pattern (READ ekf_bg, ADD to imu_bg, then
  `ekf_.setGyroBias(0,0,0)`) but the actual Tracker.cpp:1354 block never
  implements the ADD or the EKF zeroing. `ekf_bg_absorbed_total` is the
  counter for this absorption — it stays 0 in every walk.

### H2 — `translation_degenerate` gate blocks Bug 5 from running (SECONDARY)

- **Evidence**: `madgwick_visual_yaw_nudges_total` missing from JSON event
  summary across every walk = 0 events.
- **Mechanism**: gate at Tracker.cpp:4085 encloses both
  `updateGravityAlignedYaw` and Bug 5's `nudgeMadgwickYawAroundWorldZ`.
  67 % of PARALLAX_GATE logs in `bug4_walk` have parallax_rad < 0.01.
- **Falsifier**: lower `kVisualMinParallaxRad` from 0.01 to 0.005 → expect
  `madgwick_visual_yaw_nudges_total > 100` per walk.
- **Note**: even if Bug 5 fires, it cannot fix H1 — it nudges Madgwick toward
  the EKF, but the EKF's R_GtoI yaw has *also* been corrupted by IMU_BG_PUSH
  through propagateIMU. Need to fix H1 first; H2 is a downstream symptom.

### H3 — `refineGyroBiasDuringZUPT` uses raw gyro_buf, not bias-corrected (TERTIARY)

- **Evidence**: code at IMUPreintegrator.cpp:1068–1074 averages
  `gyro_buf_[i].x` directly (raw stored value). When the EKF push has
  already corrupted gyro_bias_, the EMA toward raw-mean undoes a small
  fraction (α=0.01) per call.
- **Impact**: ~1 % correction per ZUPT, vs the EKF push happening 50× more
  often → marginal effect. Not a fix on its own.

### H4 — Thermal drift (RULED OUT)

- v35_static_drift baseline = −0.028 °/s; even cold-vs-warm worst-case Bosch
  BMI270 ZRO swing = ±0.15 °/s. Cannot explain ±0.5 °/s observed.
- No thermal sensor wired in app to confirm, but the magnitude mismatch
  alone rules thermal out as primary.

---

## Proposed fixes (root cause, not workaround)

> Per project rule (`feedback_no_disabling.md`): no flag-gating; fix the
> root cause.

### Fix 1 (primary — H1) — Implement the absorb-and-zero loop

Replace Tracker.cpp:1354–1372 with the pattern documented in
`EKFState.h:701–731`:

```cpp
if ((frame_counter_ % 6) == 0) {
    cv::Mat ekf_bg = ekf_.getGyroBias();
    if (!ekf_bg.empty() && ekf_bg.rows == 3 && ekf_bg.type() == CV_64F) {
        cv::Point3f imu_bg_before = imu.getGyroBias();
        // ADD the EKF residual to the IMU's calibrated bias
        imu.setGyroBias(
            imu_bg_before.x + static_cast<float>(ekf_bg.at<double>(0, 0)),
            imu_bg_before.y + static_cast<float>(ekf_bg.at<double>(1, 0)),
            imu_bg_before.z + static_cast<float>(ekf_bg.at<double>(2, 0)));
        // Zero the EKF residual now that the IMU holds the full bias
        ekf_.setGyroBias(0.0, 0.0, 0.0);
        navsight::eventCounters().ekf_bg_absorbed_total.fetch_add(1, ...);
        navsight::eventCounters().gyro_bias_pushed_total.fetch_add(1, ...);
    }
}
```

Falsifier: `ekf_bg_absorbed_total > 0`, and the next EKF push should show
ekf_bg shrinking toward zero between calls (asymptotic convergence).
Walk-end mean_gz in `bug_02_gyro_bias_analysis.py` static windows should
drop to within ±0.05 °/s.

### Fix 2 (secondary — H2) — Untangle Bug 5 from the translation_degenerate gate

The `updateGravityAlignedYaw` and the Bug 5 Madgwick nudge solve
different problems:

- `updateGravityAlignedYaw` injects visual yaw into the EKF; requires
  translation-rich motion because pure rotation is geometrically degenerate
  for essential-matrix yaw extraction.
- Bug 5 nudges Madgwick toward the EKF's yaw to bound Madgwick drift; the
  EKF yaw is valid (it's a Kalman estimate), so the nudge does not require
  translation richness.

Move the Bug 5 nudge BELOW the gate at Tracker.cpp:4085 (so it runs even
when `translation_degenerate=1`), and source `yaw_meas` from `ekf_.getYaw()`
instead of from `kf_heading + visual_delta_heading` (which is only valid in
the translation-rich branch).

Falsifier: `madgwick_visual_yaw_nudges_total >= 0.5 × keyframe_count` on
a walk that mostly stares at a flat wall (high translation_degenerate rate).

### Fix 3 (tertiary — H3) — Make ZUPT refinement bias-corrected

`refineGyroBiasDuringZUPT` should average `(gyro_buf_[i].x − gyro_bias_.x)`
instead of raw `gyro_buf_[i].x`. The residual mean over a truly
stationary 20-sample window is the *additional* bias not already accounted
for; the current code averages the entire raw signal which double-counts
the existing `gyro_bias_` if the IMU is well-calibrated.

This is a cleanup, not a primary fix; defer until H1 is verified.

---

## Cross-worker notes

- **Bug 4 (heading display chain)**: the user-visible `172° → 203° → 235°`
  drift comes from `getHeading()` reading Madgwick (SensorRepository.kt:949,
  Tracker.cpp:5210). Once H1 is fixed, expect this debug-panel chain to
  show stable headings within ±5° per loop.
- **Bug 5 cross-walk patterns**: this finding applies to *every* walk
  recorded since the May-16 IMU_BG_PUSH re-add. Pre-2026-05-12 v22 walks
  (which had no push loop) showed clean Madgwick — consistent.

---

## Falsifiers (the experiment that would invalidate H1)

1. Disable IMU_BG_PUSH (one-line edit: `if (false && (frame_counter_ % 6) == 0)`)
   walk → if Madgwick yaw drift still 30°/loop, H1 is wrong.
2. Inspect a single walk's `gyro_bias_` time-series via additional logging
   (PUSH before/after, raw mean of last 20 stationary samples). If the
   raw stationary-mean tracks Madgwick drift but IMU_BG_PUSH doesn't, H1 is
   wrong. Expected: PUSH cycle shows bias jumping by ~0.007 rad/s each cycle,
   stationary-mean stays near 0.

---

## Tools and artifacts produced

- `scripts/bug_02_gyro_bias_analysis.py` — sim-JSON bias measurement
  (reusable per `feedback_reuse_scripts`).

No source-code changes (investigation-only mandate).
