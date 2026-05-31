# Walk-K Cold Under-Convergence + VEHICLE-Misclass — Fix Plan (2026-05-31)

Source: Opus root-cause workflow on val_2026_05_31_pm (walk+run both real 13m, GPS ground truth).
Along-track 1km/5% lever. All edits camera-thread, speed/scale only — **HEADING UNTOUCHED** (K scales
`depth_flow_speed = K·visual_rel`; gait only selects a K slot; neither path reads Madgwick/RV/EKF rotation).

## ROOT CAUSE (corrected — "566" was a red herring)
- The cold seed was **~745.8**, not 566 (first `ACCEL_K_CALIB[df]` already reads cur_K=745.8). 556.2 = RUN seed
  (897×kRunWalkSeedRatio 0.62); 566396-milli = a session-MIN telemetry gauge; no hardcoded 566 exists.
- K atomics init to **-1.0** (Tracker.h:617/648); `reset()` preserves K (Tracker.cpp:761-765); Kotlin re-seeds from
  `prefs.getFloat(PREF_MIDAS_SCALE_K,-1f)` (NavSightViewModel.kt:274-277/610-613) → `setMidasScaleK`
  (Tracker.cpp:1013-1016) into BOTH midas_scale_K_ and expansion_scale_K_. The persisted value came from a prior
  session's ACTIVE slot via getMidasScaleK (Tracker.cpp:1019-1026). **SELF-POISONING:** a session ending
  under-converged persists that low K → next cold walk starts low. Single un-tagged key (no per-slot persistence).
- **Why the EMA can't self-correct:** alpha = (mode_switch_fast_alpha_frames_>0)?kFastAlpha(0.3):kNormalAlpha(0.05)
  (Tracker.cpp:1325/8532). Fast-α armed ONLY in onModeSwitch (:1103). active_mode_ inits WALK (Tracker.h:582) → a
  PURE walk never switches → never arms fast-α → every update α=0.05. The virgin-slot snap `new_k=(cur_k<=0)?k_obs`
  (:1334/:8540) is DEAD on a seeded device (cur_k>0). Math: 745→1340 at α=0.05 needs ~34 updates for 10%; a 13m/15s
  walk gives ~6 df updates/stop → crawled 745→897 → **0.68× under**. Convergence bug, NOT detection (accel observes
  the right K).
- **VEHICLE misclass:** classifyGait checks VEHICLE first: `time_since_last_step_s>3.0 && accel_speed>0.5`
  (Tracker.cpp:1043). Walk pause/turn freezes last_step_ns_ (detectStep gyro-gates >0.8, IMUPreintegrator.cpp:515)
  → tsls climbs past 3s while accel_speed drifts >0.5 → walking → VEHICLE. Live: 8/17 df calibs at gait=2 (k_obs
  920-968), starving the WALK slot.

## FIX A — cold fast-converge arm (the lever; reuses existing machinery, no new tunable)
CAUSE/CHANGE/FALSIFIER. Add `bool cold_fast_converge_armed_{false};` (Tracker.h ~:594); reset in reset() (~:777).
In BOTH accept blocks (df Tracker.cpp:1320, loom :8528), INSIDE the `!k_outlier` guard, BEFORE alpha is read:
```cpp
if (!cold_fast_converge_armed_ && cur_k > 0.0) {
    mode_switch_fast_alpha_frames_ = kFastConvergeFrames;  // reuse existing fast window
    cold_fast_converge_armed_ = true;
    LOGI("ACCEL_K_COLD_ARM[df]: cur_K=%.1f k_obs=%.1f gait=%d fast_frames=%d", cur_k, k_obs,
         (int)active_mode_, mode_switch_fast_alpha_frames_);
    navsight::eventCounters().cold_fast_converge_armed.fetch_add(1, std::memory_order_relaxed);
}
```
Reuses kFastConvergeFrames(10)/kFastAlpha(0.3). Target = live k_obs (accel_dist/visual_rel), NO hardcoded 1340.
Heading-safe. (Chosen over hard snap-to-first-k_obs because k_obs is high-variance 693-1502; fast-α averages.)
Blast radius: RUN not re-inflated (fast-α pulls to OBSERVED k_obs, RUN's own is lower); pure-walk identical once
converged (reverts to α=0.05; one-shot flag prevents re-arm); >3× blow-up guard intact (arm is inside !k_outlier);
M1 last_fast_alpha_frame_ guard unchanged.
OPTIONAL companion (deferred): persist only the WALK-slot K to break cross-session contamination.

## FIX B — VEHICLE-misclass (ship together; unstarves FIX A's calibration)
Part 1 (IMUPreintegrator.cpp getStepInfo ~:566) — make cadence go STALE (step_period_s_ never time-decays today):
```cpp
const double tsls = (last_step_ns_>0 && last_accel_ts_ns_>0)
    ? std::max(0.0,(last_accel_ts_ns_-last_step_ns_)*1e-9) : 1e9;
info.step_freq_hz = (step_period_s_>0.0 && tsls<=MAX_STEP_PERIOD_S) ? (1.0/step_period_s_) : 0.0;
```
Part 2 (Tracker.cpp:1043) — comment OLD, add cadence veto (reuses 0.6 WALK floor + MAX_STEP_PERIOD_S):
```cpp
// OLD: if (si.time_since_last_step_s > 3.0 && accel_speed > 0.5) cand = GaitMode::VEHICLE;
if (si.time_since_last_step_s > 3.0 && accel_speed > 0.5 && si.step_freq_hz < 0.6)
    cand = GaitMode::VEHICLE;
```
+ GAIT_VEHICLE_SUPPRESS LOGI + gait_vehicle_suppressed counter on the blocked branch.
REAL ride survives: no steps >1.5s → step_freq_hz=0<0.6 → VEHICLE still fires (Part 1 is what makes this safe — a
bare step_freq veto alone would PIN a walk-to-scooter rider in WALK forever).

## INSTRUMENTATION: EventCounters cold_fast_converge_armed + gait_vehicle_suppressed (add ~:521, reset ~:1050,
summary ~:1246/:1423).

## VALIDATION GATE (re-record same 13m walk + loop, + next scooter log):
- exactly 1 ACCEL_K_COLD_ARM, cold_fast_converge_armed=1; K past ~1.1-1.3M milli within first ZUPT window (not
  745→897 crawl); dot net → 12-13m (≥0.9× of 12.3m GPS) vs today 8.9m/0.68×; heading traces UNCHANGED.
- zero GAIT_MODE_SWITCH new=2 on pure walk/loop; gait=2 calib count=0 on walk; gait_vehicle_suppressed>0 on pauses.
- RUN converges to its OWN lower k_obs, no 27km/h spike (>3× guard live).
- real ride: step_freq_hz→0 within ~1.5s, VEHICLE STILL promotes.

## FILES: Tracker.h (1 member), Tracker.cpp (reset + 2 accept blocks + classifyGait), IMUPreintegrator.cpp
(getStepInfo), EventCounters.h (2 counters). No Kotlin change for FIX A.

## SEPARATE (queued after, from the run teleports): looming/depth-flow single-frame SPIKE guard — the run's 3
teleports (1.8-3.7m in ONE frame, ~48 m/s) are spikes the >3× guard misses on blurry fast motion, NOT Fix C
(dot was moving normally before each jump, not frozen). Fix C itself = SKIP (Fix B-looming-fallback already carries
turn-freezes; phantom-drift risk; 1km route is straight+1 U-turn).
