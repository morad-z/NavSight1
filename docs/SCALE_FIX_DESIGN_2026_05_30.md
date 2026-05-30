All cited regions verified against the actual source. The findings are accurate and self-consistent. I have enough to write the executable spec.

# Scale Fix Design — Per-Gait K (Walk / Run / Scooter), 1 km @ ≤5%

## 1. RECOMMENDATION

**Implement per-mode K (Candidate A) + brake-anchored calibration (Candidate C) together, in that order, as one shipment.** A gives independent `K_walk / K_run / K_scooter` slots that cannot poison each other (kills the measured walk-stop spike `k_obs≈4845` → shared-K≈2400 → run-overshoot chain); C extends self-calibration to the scooter, which rarely makes a full stop, by treating a hard deceleration as a ZUPT-equivalent calibration anchor. This pair is the **minimum viable path** for the walk+scooter / 1 km-5% goal because A fixes all three cross-contamination paths and C removes the scooter's stop-dependency, while neither touches heading. **Confidence: high** (every code site and the gait-discriminator signals are verified in-tree; the only UNKNOWNs are scooter classification thresholds and the exact `MIN/MAX_STEP_PERIOD_S` boundary on-device — both resolved by a logcat line shipped *with* the change, not before it).

## 2. WHY NOT THE OTHERS

- **B (un-poison the affine), first:** Correct long-term endgame (removes global K entirely), but it depends on re-seeding `scale_fuser_` from a *good* K — which only exists after A/C calibrate one. It is a follow-on, not a first fix. Its affine target also needs `z_vio` from triangulated points, which the scooter's degenerate forward-motion frames (recoverPose `verification_ok` fails ~67%) cannot reliably supply.
- **D (learned-inertial TLIO/RNIN-VIO):** ~2-week effort + data collection; SenseINS is all-pedestrian (zero scooter); LSTM-on-Mali-G78 flaky. Does not help the immediate 1 km goal.
- **ScaleEstimatorVI (Step B):** Proven dead-end — errors-in-variables OLS attenuates `s→0` under direction noise. Stays commented out.

## 3. EXACT IMPLEMENTATION SPEC

> Invoke the `navsight-implementor` skill before writing. Comment-out (don't delete) replaced blocks per the no-deletions rule. Touch **only** the speed/scale path; never `heading`, `scalar_heading_`, or `Tracker.cpp:3721-3722`.

### Step 0 — Expose the gait signal (`IMUPreintegrator`)
`StepInfo` has no cadence field today (`IMUPreintegrator.h:90-102`, verified — fields stop at `stride_calibrated`).
- **`IMUPreintegrator.h` ~line 102** (after `bool stride_calibrated;`), insert:
  `double step_freq_hz{0.0}; // 1/step_period_s_, 0 if no recent step`
- **`IMUPreintegrator.cpp` ~line 599/614** (after `info.speed_mps = ...`), insert:
  `info.step_freq_hz = (step_period_s_ > 0.0) ? (1.0 / step_period_s_) : 0.0;`
- No ABI/JNI break (`StepInfo` returned by value, not JNI-exposed). **First read** `IMUPreintegrator.cpp` near `MIN_STEP_PERIOD_S`/`MAX_STEP_PERIOD_S` to confirm the constants (header says `MIN=0.25`s=4 Hz, `MAX=1.5`s=0.67 Hz) before locking the run boundary.

### Step 1 — K storage: replace two scalars with per-gait slots (`Tracker.h`)
Currently `std::atomic<double> midas_scale_K_{-1.0}` (Tracker.h:557) and `expansion_scale_K_{-1.0}` (Tracker.h:588), seeded together from one prefs float.
- Add enum + slots in `Tracker.h`:
  ```cpp
  enum class GaitMode { WALK = 0, RUN = 1, VEHICLE = 2 };
  struct KSlot { std::atomic<double> df{-1.0}; std::atomic<double> loom{-1.0}; };
  KSlot k_slots_[3];
  GaitMode active_mode_{GaitMode::WALK};
  int mode_hold_frames_{0};
  int mode_switch_fast_alpha_frames_{0};
  static constexpr int    kModeHoldFrames     = 15;   // 0.5 s @ 30 Hz hysteresis
  static constexpr int    kFastConvergeFrames = 10;
  static constexpr double kFastAlpha          = 0.3;
  static constexpr double kNormalAlpha        = 0.05;
  static constexpr double kRunWalkSeedRatio   = 0.62; // measured 831/1340
  ```
- Comment out the two old atomics; everywhere `midas_scale_K_` was read/written becomes `k_slots_[(int)active_mode_].df`, and `expansion_scale_K_` becomes `k_slots_[(int)active_mode_].loom`. `active_mode_` is camera-thread single-writer/single-reader (it gates `updateDepthFlowSpeed`/`updateExpansionSpeed`, both on the camera thread that already calls `imu.getStepInfo()` at Tracker.cpp:3606) — no new mutex, no atomic on `active_mode_` needed.

### Step 2 — Gait detector (`Tracker.cpp`, camera thread)
Thresholds are **physics-derived, not magic numbers** — cite measured `k_obs` (WALK≈1340, RUN≈831 ⇒ ratio 0.62; UTURN≈1652) and textbook cadence (walk 1.5-2.5 Hz, run 2.7-4 Hz; `MIN_STEP_PERIOD_S=0.25`s=4 Hz cap):
```cpp
GaitMode classifyGait(const IMUPreintegrator& imu, double accel_speed) {
    const auto si = imu.getStepInfo();
    GaitMode cand;
    if (si.time_since_last_step_s > 3.0 && accel_speed > 0.5)        cand = GaitMode::VEHICLE;
    else if (si.step_freq_hz > 2.7)                                  cand = GaitMode::RUN;
    else if (si.step_freq_hz >= 0.6)                                 cand = GaitMode::WALK;
    else                                                             cand = active_mode_; // hold on cold/ambiguous
    // hysteresis: only switch after kModeHoldFrames consecutive candidate frames
    if (cand == active_mode_) { mode_hold_frames_ = 0; return active_mode_; }
    if (++mode_hold_frames_ >= kModeHoldFrames) {
        GaitMode old = active_mode_;
        active_mode_ = cand; mode_hold_frames_ = 0;
        onModeSwitch(old, cand);   // seeding + fast-alpha (Step 3)
    }
    return active_mode_;
}
```
- `accel_speed` is already computed: `std::hypot(accel_vel_w_[0], accel_vel_w_[1])` (Tracker.cpp:1166/2498). Vehicle floor 0.5 m/s derives from `kAccelKMinDistM=1.0 m / 2.2 s`. The 2.7 Hz run threshold leaves a 0.2 Hz hysteresis gap above the 2.5 Hz walk top.
- **Call order is the critical correctness constraint** — in *both* `updateDepthFlowSpeed` (~Tracker.cpp:1202) and `updateExpansionSpeed` (~Tracker.cpp:8299): **(1)** `active_mode_ = classifyGait(imu, accel_speed);` **(2)** load `cur_k` from `k_slots_[(int)active_mode_].df/.loom`; **(3)** existing 3× outlier check against *that* `cur_k`; **(4)** EMA store into the same slot.

### Step 3 — Mode-switch seeding + fast EMA (no stale inheritance)
The run-overshoot root cause was a stale K inherited across the EMA. On switch:
```cpp
void onModeSwitch(GaitMode old, GaitMode neu) {
    mode_switch_fast_alpha_frames_ = kFastConvergeFrames;
    KSlot& s = k_slots_[(int)neu];
    if (s.df.load() < 0.0) { // virgin slot — seed conservatively, never inherit raw
        double w = k_slots_[(int)GaitMode::WALK].df.load();
        if (w > 0.0) {
            double seed = (neu == GaitMode::RUN) ? w * kRunWalkSeedRatio : w; // VEHICLE: walk as start
            s.df.store(seed); s.loom.store(seed);
        }
    }
    LOGI("GAIT_MODE_SWITCH: old=%d new=%d K_new=%.1f", (int)old, (int)neu, s.df.load());
    // DO NOT reset accel_dist_accum_ / visual_rel_dist_* — only ZUPT does that.
}
```
- EMA at Tracker.cpp:1226 and 8327-8328: `double alpha = (mode_switch_fast_alpha_frames_>0)?kFastAlpha:kNormalAlpha; if(mode_switch_fast_alpha_frames_>0) --mode_switch_fast_alpha_frames_; new_k = (cur_k<=0.0)?k_obs:((1-alpha)*cur_k + alpha*k_obs);`
- **Reset only** `mode_hold_frames_` and `mode_switch_fast_alpha_frames_` on switch. The physical accumulators (`accel_dist_accum_`, `visual_rel_dist_accum_`, `visual_rel_dist_loom_`) keep running so calibration fires as soon as the new mode accrues distance.

### Step 4 — Brake-anchored calibration (C): scooter without a stop
The accel integrator already runs world-frame post-gravity (Tracker.cpp:2446-2467); the ZUPT is only a *window selector* (resets `secs_since_zupt_=0` at Tracker.cpp:2474). Add a brake anchor mirroring the init-as-ZUPT precedent (Tracker.cpp:767-777):
- In the accel block (~Tracker.cpp:2445-2468), add brake-detect: track horizontal decel; threshold `kBrakeThreshMps2 = 1.5` (derived: Allan `σ_a=1.63e-3 m/s²/√Hz` → ~0.023 m/s² noise floor at 200 Hz; 1.5 is >>50× floor, far below a hard brake). When `|a_horizontal| > kBrakeThreshMps2` sustained `> kBrakeSustainS = 0.2 s`, set `is_braking=true`, then `secs_since_zupt_ = 0.0; accel_dist_accum_ = 0.0;` (same as the `is_static` reset at 2473-2475) — opening the existing `[0.3,2.5]s` window. Scooter physics check: 20 km/h braking at ~0.3 g ⇒ Δv=5.56 m/s over ~1.9 s ⇒ dist≈5.3 m ≫ `kAccelKMinDistM=1.0` — gates satisfied.
- **Per-mode window**: replace literal `2.5` in *both* gates (Tracker.cpp:1211 and 8311) with `const double window_max = (active_mode_==GaitMode::VEHICLE)?5.0:2.5;` (scooter sustains clean accel/decel longer).

### Step 5 — Turn-suppression (explains UTURN≈1652 inflation)
Turning lowers forward `vis_rel` → inflates `k_obs`. Skip K calibration when `gyro_norm > 0.5 rad/s` (milder than existing `kGyroGateRadS=1.2`) so turns don't pull `K_walk` toward 1652. Add as an extra `&&` on the two calib gates; tag the skip in the `ACCEL_K_CALIB` log. (Confidence: likely — validate the reduction on the next U-turn walk.)

### Step 6 — Persistence (`NavSightViewModel.kt` + JNI)
Today one key `PREF_MIDAS_SCALE_K="midas_scale_k"` (NavSightViewModel.kt:52), read/written at :274-276 / :451-453 / :610-612 / :626-628.
- Add 6 Float keys: `midas_k_walk_df`, `midas_k_walk_loom`, `midas_k_run_df`, `midas_k_run_loom`, `midas_k_veh_df`, `midas_k_veh_loom` (Float precision is fine: ~0.01% rel error at K≈800-1700).
- JNI: add `setMidasScaleKForMode(mode:Int, dfK:Double, loomK:Double)` and `getMidasScaleKForMode(mode:Int):DoubleArray` to `NativeBridge.kt`; matching C++ in `native-lib.cpp` calling new `Tracker::setKSlot/getKSlot`. Init/onResume push all 6; 3 s periodic + onPause pull all 6.
- **Migration**: on first launch where `midas_k_walk_df` is absent but `midas_scale_k` is present, copy the old value into all unset slots (RUN seeded ×0.62), then it's safe to leave the legacy key. `getMidasScaleK()` compat-read returns `k_slots_[WALK].df` (lowest-speed, most-stable). `setMidasScaleK(k)` becomes the "seed all unset slots" bootstrap only (keep the existing non-positive guard), so the Kotlin side needs **zero changes for the first ship** if you defer full 6-slot persistence to a follow-up.
- **Seeds**: `K_walk` and `K_run` may seed from measured 1340 / 831 (or leave −1 to self-calibrate). `K_veh = −1` (unknown — no scooter data exists; self-calibrates on first ride via Step 4).

### Step 7 — New counters / LOGI
- Add `gait=W/R/V` tag to the existing `ACCEL_K_STATE` (Tracker.cpp:2500), `ACCEL_K_CALIB[df]` (1229), `ACCEL_K_CALIB[loom]` (8333), and `DEPTH_FLOW_SPEED` (1253) lines.
- New `GAIT_MODE_SWITCH` (Step 3) and the data-gathering `MOTION_MODE: var=%.3f vehicle_spd=%.2f is_walking=%d in_vehicle=%d` at end of motion classification — **ship this with the change** to resolve the scooter-classification UNKNOWN on the first ride.
- No change at the trajectory site: `disp = df_speed * dt_frame` (Tracker.cpp:3641) reads `depth_flow_speed_mps_`, which already baked K in at update time (1245-1249). Only `updateDepthFlowSpeed`/`updateExpansionSpeed` change.

## 4. TRAP CHECKLIST (one rule per past failure)

1. **1.78 global bias:** Any correction touching `accel_dist_accum_` or `visual_rel_dist_*` MUST be indexed by `GaitMode` or be physics-universal. Make `kAccelKBiasCorrection` (Tracker.cpp:57) a 3-element array, all `1.0`; the UI known-distance calibration writes only `[WALK]`. Never a scalar consumed across gait boundaries.
2. **ScaleEstimatorVI EIV dilution:** Do not reintroduce any joint linear solve using noisy unit-direction `t_vo` as a column. Stays commented out; scooter K uses accel-K ratio-of-distances only.
3. **Single-K contamination:** K is per-gait; walk-stop spike now only ever touches `K_walk` (and is rejected by that slot's own 3× guard once seeded ≈1340, since 4845 > 3×1340).
4. **Looming/depth-flow double-count:** Keep `df` and `loom` slots separate per gait, each from its own basis (`disp_rel` vs `vz_rel`). Shared `accel_dist_accum_` denominator is correct (same physical displacement). **New risk:** on a *moving* gait switch the `visual_rel_dist_*` accumulators carry prior-gait numbers — acceptable because they only feed `k_obs = accel_dist/vis_rel` and both numerator and denominator span the same physical interval since the last ZUPT; do **not** add a mid-motion accumulator reset (it would break the ratio).
5. **Gait misclassification:** 15-frame (0.5 s) hysteresis before any slot switch; freeze the last K during the transition window. Worst case (slow run as walk ⇒ 1.6× over; slow walk as run ⇒ 0.62× under) is still far better than today's shared 0.5×/1.7×.

## 5. SCOOTER NOTE (the honest hard part)

The scooter is the one gait that does **not** reliably self-calibrate, because the accel-K window only opens after `secs_since_zupt_` resets, and a cruising scooter never triggers `is_static`. Three paths, in reliability order:
1. **Brake-anchored (Step 4, ships now):** every hard decel (traffic light, slowdown) becomes a calibration anchor. Works during normal riding. **Risk:** a gentle constant-cruise leg with no braking yields zero calibration events.
2. **Full stop (free, today):** any complete stop fires the existing ZUPT window and calibrates `K_veh` exactly like walk.
3. **Known-distance UI seed / GPS-when-unjammed (not wired, follow-up):** reuse the 5 m/10 m calibration UI, or opportunistically `K_obs = gps_speed / speed_rel` when GPS is fresh.

Because `K_veh` starts at −1 and the dot won't advance until calibrated (`K<=0` bail at Tracker.cpp:1248-1258), the **first scooter ride must include either a deliberate brake-to-stop before departure or a pre-seed** (`K_veh = K_walk` as a coarse "dot at least moves" start). `MOTION_MODE` logcat from that first ride resolves whether `in_vehicle_mode_` even fires on a low-vibration electric scooter (UNKNOWN-needs-data) — if it doesn't, fall back to a locomotion-agnostic gate: `!is_walking_pattern_ && depth_flow_speed > 2.0 m/s` (7.2 km/h, above fast-run).

## 6. VALIDATION (tape-measured; tuning needs real rides)

Goal: 1 km @ ≤5% ⇒ ≤50 m total, ≤25 m per 500 m leg. No GPS (jammed) — use tape-measured course, ideally exact 500 m out + 500 m back.
- **WALK:** out+back; **full 3 s stop at the 500 m mark** (anchors `K_walk` before the return leg — required, since EMA α=0.05 gives only ~1-3 updates over 500 m). Check `ACCEL_K_CALIB[df|loom] gait=W`, `k_obs∈[900,1800]`. Compare `total_path_dm/10` (m) to tape.
- **RUN:** same course; `k_obs∈[600,1100]`, `gait=R`.
- **SCOOTER:** ride course; if no mid-ride stop, confirm `K_veh` was pre-seeded or brake-anchored — else the dot won't advance.
- **Counters/logs to read:** `ACCEL_K_STATE` (every 15 frames: `is_static`, `window_open`, `accel_dist`, `K_df`, `K_loom`, now `gait=`), `ACCEL_K_CALIB[df]/[loom]`, `GAIT_MODE_SWITCH`, `MOTION_MODE`, `midas_scale_k_milli` in `event_summary`, `depth_flow_calib_updates`.
- **Caveat (from memory, separate problem):** per-gait K corrects the *scale* of active frames, but the walk's "93% frozen dot" is the unimplemented looming-fallback (Fix B), not K. The 1 km goal needs **both** per-gait K **and** Fix B active on the walk; validate them together.

## 7. HEADING-SAFETY CONFIRMATION

This design touches **only** the speed/scale path: `updateDepthFlowSpeed`, `updateExpansionSpeed`, the accel-K integration/window, the K slots, gait detection, and persistence. It does **not** modify `heading`, `scalar_heading_`, the heading-projection at `Tracker.cpp:3721-3722` (`dx_world = disp*sin(heading)`), the Madgwick/RV pipeline, or any EKF rotation state. The dot's heading remains exactly as today; only the *length* of each step's displacement changes. Per the binding user decision, heading is protected and untouched.

(Files: `app/src/main/cpp/Tracker.cpp`, `app/src/main/cpp/Tracker.h`, `app/src/main/cpp/IMUPreintegrator.{h,cpp}`, `app/src/main/cpp/native-lib.cpp`, `app/src/main/java/com/example/navsight1/NavSightViewModel.kt`, `app/src/main/java/com/example/navsight1/NativeBridge.kt`. All cited line regions verified in-tree.)