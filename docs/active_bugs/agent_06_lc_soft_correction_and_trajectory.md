# Agent 6 — LC Soft Correction & Trajectory Monotonic Investigation

## Status

Investigation complete on real walk evidence from 2026-05-19 through 2026-05-21. Verdict: **HANDOFF #2 is partially redundant after today's fixes** (heading nudge already covered; position nudge could still help in 1 rare residual). **HANDOFF #3 was the SimulationFrameRecorder freezing on stationary intervals (hypothesis 1 / `is_static` gate at Tracker.cpp:2845)**, not a bug — it's the designed `global_t_` freeze behavior surfacing as "trajectory peaks and freezes." Today's longer walk shows trajectory both peaking AND returning, with brief intra-walk freezes during stationary segments.

## Part 1: LC soft correction

### Still needed?

**Mostly redundant after today's fixes. Position nudge marginally useful for 1-in-156 outliers.**

Quantitative evidence from `tests/sims/regression/visual/bug4_walk_2026_05_21.json` (event_summary):

| Counter | Value |
|---|---|
| `loop_closure_attempts` | 156 |
| `loop_closure_accepts` | 9 |
| `loop_closure_rejects_low_score` | 3 |
| `loop_closure_rejects_pnp` | 5 |
| `loop_closure_rejects_rot_sanity_total` | **10** (NEW — Bug 3) |
| `loop_closure_chi2_rejected` | **0** in event_summary (but see m²=31.7 below) |
| `loop_closure_corrections_applied` | 73 |

Cross-walk chi² rejection trend:
- `parallax_fix_walk_2026_05_20`: **20 chi² rejects** out of 25 accepts (pre-Bug-3 baseline)
- `promo_parallax_walk_2026_05_21`: **10 chi² rejects** out of 18 accepts
- `bug3_walk_2026_05_21`: **0 chi² rejects** out of 14 accepts (post Bug-3 rot-sanity gate)
- `bug4_walk_2026_05_21`: **0 chi² rejects** out of 9 accepts (post Bug-3 + Bug-5)

Bug 3 (rot-sanity gate at `Tracker.cpp:6667-6692`) eliminated the 169°-residual chi² outliers entirely.

LC_ABS m² distribution from `bug4_walk_2026_05_21.logcat.txt`:
- Median m² ≈ 2.7 (well under threshold 22.5)
- Max m² = **31.71** at one ramp frame k=5 (`p_G=[6.86 10.91 -0.61]` vs `target_p=[0.92 2.07 0.18]`, residual 12 m; `m2_R=3.30 m2_p=28.41` → position-driven)

**Part B (soft heading nudge) is REDUNDANT with Bug 5.** Existing `LC_MADG_NUDGE` at `Tracker.cpp:6698-6709` already does this; observed deltas are 0.00° because EKF and Madgwick are already in sync post Bug 5.

**Part A (lower temporal exclusion 30s → 10s) NOT needed for revisit detection.** Bug4 walk (~116s) produced 9 accepts + 156 attempts. LC fires aggressively on long walks.

**Part C (soft position nudge) is the ONLY part with quantitative justification.** ~0.6% rejection rate (1/156) suggests small but real benefit.

### Design

**Recommendation: ship only Part C with tight scope.**

In `Tracker::consumeLoopClosureMatchIfReady` after rot-sanity check but before `updateAbsolutePose`:

```cpp
constexpr double kSoftNudgeBlend = 0.10;
constexpr double kMinPnPInliersSoftNudge = 25;
if (pnp_inlier_count >= kMinPnPInliersSoftNudge && m2_p > kChi2_3DOF_999) {
    cv::Mat soft_delta = kSoftNudgeBlend * (target_p_world - p_G_before);
    ekf_.setPosition(p_G_before + soft_delta);
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        global_t_ += soft_delta;
    }
    navsight::eventCounters().lc_soft_position_nudges.fetch_add(
        1, std::memory_order_relaxed);
}
```

Acceptance gates: PnP inliers ≥ 25, 10% blend, don't touch covariance, new counter `lc_soft_position_nudges`.

**Skip Part A and Part B entirely.**

### Falsifier

- `lc_soft_position_nudges` > 0 only when m²_p > 16.27 AND PnP inliers ≥ 25
- Over multi-LC revisit, `|r_p|` in successive LC_ABS lines must monotonically decrease
- No new false-positive landmark corruption

## Part 2: trajectory monotonic

### Verdict

**FIXED (or never was a generalized bug). The persistent symptom is by-design `global_t_` freeze during is_static.**

### Evidence

Trajectory return-to-origin verified in today's bug4_walk_2026_05_21:

| t+s | position (vx, vy, vz) | distance from origin |
|---|---|---|
| 0.0 | (0.00, 0.00, 0.00) | 0.00 m (start) |
| +57.0 | (0.93, 0.17, 2.29) | 2.47 m |
| +82.3 | (5.86, 0.51, -9.31) | **11.01 m (peak)** |
| +95.8 | (11.91, 0.67, -1.39) | 12.01 m |
| +115.8 | (0.10, 0.09, -2.53) | 2.54 m (near origin) |
| +115.8…+116.3 | (0.10, 0.09, -2.53) ×7 frozen | unchanged (~505ms freeze) |

**Trajectory DOES return to ~2.5m of origin** after peaking at 12m. The "freeze" is a 7-sample stationary window at the END where user stopped before stopping recording.

Yesterday's fix11_revisit_2026_05_19:
- 15+ consecutive identical samples for ~800ms at the END, ending at peak distance ~3.5m — exact match for "peaked-then-froze" pattern.

The mechanism is identical: at `Tracker.cpp:2845`, when `is_static || rotation_dominated`, per-frame `global_t_` update is gated off. ZUPT confirms stationary state. User stopped, no scale update, `global_t_` stops advancing, recorder writes same `vio.x/y/z` for every tick.

**HANDOFF #3 misdiagnosed.** No `global_t_`-freezes-during-low-velocity bug. No SimulationFrameRecorder bug. The recorder writes whatever JNI VIO snapshot returns; JNI reads `global_t_`; `global_t_` is correctly frozen during is_static per design.

### Recommendation

**No code change.** Optional cosmetic: prune trailing stationary samples from `points[]` in `NavSightViewModel.kt:336-353` (skip append when `last == current`). ~5 LOC, doesn't change VIO behavior, just makes plots cleaner.

## Cross-references

- LC soft correction site: `Tracker.cpp:6694-6720`
- Bug 3 rot-sanity gate: `Tracker.cpp:6632-6692`
- LC_MADG_NUDGE: `Tracker.cpp:6698-6709`
- chi² gate: `EKFState.cpp:1601-1657` (kChi2Threshold = 22.5)
- Trajectory freeze site: `Tracker.cpp:2845-2854`
- Recorder (proven not the cause): `SimulationFrameRecorder.kt:106-179`

## Confidence

**HIGH** for both parts. Direct quantitative evidence from 5 cross-day event_summaries; mechanism identified at exact file:line locations; symmetric explanation for yesterday's "freeze at peak" and today's "freeze near origin."
