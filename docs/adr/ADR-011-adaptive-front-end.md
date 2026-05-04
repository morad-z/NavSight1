# ADR-011 — Adaptive front-end robustness

**Status:** Accepted
**Date:** 2026-05-04
**Owner:** Morad Zubidat (sensor fusion)
**Companion:** ADR-008 (MSCKF re-enabled with damping + Huber), ADR-009
(SLAM features in EKF state), ADR-010 (ORB descriptors at keyframes for
relocalization).

## Context

The visual front-end shipped through ADR-008 / ADR-009 / ADR-010 (Plan
Steps 3a / 3b / 4) uses fixed thresholds tuned for a "typical" walking
scene: KLT search window 21x21, `MAX_FEATURES = 200`,
`QUALITY_LEVEL = 0.05`, single-gate gyro-only pure-rotation detector at
2.0 rad/s, and no motion-blur skip. Those tunings hold on a flat
sunlit Haifa street; they break on the Haifa scenarios that actually
matter:

- High gyro on scooters and quick head turns push KLT corners out of
  the 21x21 window inside a single inter-frame interval; the FB-check
  drops the track and the next frame seeds with new IDs.
- Low light in parking garages / apartment lobbies (`mean_brightness <
  0.12` in 8-bit) collapses corner SNR. `goodFeaturesToTrack` happily
  returns 200 weak corners that KLT then loses on the next frame; the
  feature-count-stable gate masks the underlying instability.
- Motion blur on rapid head turns yields a frame where every Laplacian
  eigenvalue collapses; `cv::findEssentialMat` accepts a degenerate
  geometry and the resulting pose update is noise.
- Rotation-only "looking around" segments (a scooter rider scanning
  intersections, a user pivoting in place) trip the gyro-only
  pure-rotation gate intermittently — `|ω|` exceeds 2.0 rad/s for
  fractions of a second, then drops, and the front-end ping-pongs
  between rotation-skip and full update without a clean signal.

Step 5 of `docs/VISUAL_PRODUCTION_PLAN.md` (line 611, "Adaptive
front-end robustness") fixes all four with one cross-cutting principle:
**the front-end runtime parameters adapt to the per-frame regime they
observe**, instead of carrying a single static tuning that has to win
every regime by averaging.

## Decision

### Motion blur detection (Laplacian variance gate)

`Tracker::measureBlur` computes the variance of the Laplacian on the
gray frame's centre crop. The threshold is **80** (squared-pixel
units, 8-bit gray); below threshold the frame is flagged blurry.
Variance-of-Laplacian as a blur metric is the standard fast proxy
(Pertuz et al. 2013, *Analysis of focus measure operators in
shape-from-focus*); 80 is calibrated on the Step 4 corpus to flag the
frames a human marks as "blurry" with > 95% recall.

On a blurry frame the Tracker:

- **Skips MSCKF and SLAM updates this frame.** A degenerate geometry
  fed into `findEssentialMat` / `processLostFeatures` produces a
  noise pose update that is strictly worse than no update.
- **Still propagates IMU and runs ZUPT.** This is the one
  non-obvious choice and it is deliberate. `EKFState::propagateIMU`
  is the prediction half of the filter — skipping it does not make
  the filter "wait", it makes the EKF state freeze in time while the
  rest of the system advances. The resulting pose freeze on a
  multi-frame blur burst is exactly the kind of teleport-when-it-
  resumes failure mode ADR-006 documented. Continue propagating;
  drop only the visual update.

Persistent blur (> ~3 consecutive blurry frames) is handled by the
ADR-010 relocalization path, not by Step 5. Step 5 is the per-frame
gate; ADR-010 is the recovery mechanism after the gate has been
firing for a while.

### Adaptive KLT window from gyro

The KLT search window scales with expected inter-frame pixel
displacement, derived directly from the gyro magnitude:

```
expected_disp_px = focal × |ω| × dt
window_px        = clamp(2 × expected_disp_px + 11, 21, 41)
window_px        = round_to_odd(window_px)
```

Rule of thumb (Bouguet 2000, *Pyramidal Implementation of the Lucas
Kanade Feature Tracker*): KLT loses tracks when the window is smaller
than about 1.5x the inter-frame pixel displacement, because the
constant-flow assumption inside the window stops holding. The 2x
multiplier is the same rule with one frame of margin; the +11 bias is
the OpenCV-default lower envelope for KLT on subpixel corners.

The clamp lower bound is the steady-state window (21 px); the upper
bound (41 px) is the largest window that keeps the per-track KLT
solve under the per-frame budget on a Snapdragon 695. Above ~41 px
the constant-flow assumption inside the window starts to fail anyway,
so growing further would not help.

The window must round to an odd integer because OpenCV's
`calcOpticalFlowPyrLK` requires an odd `winSize` (kernel symmetry).

### Adaptive feature replenish for low light

`FeatureManager::replenishSparse` probes the centre crop of the gray
frame with `cv::mean(gray(centre))[0] / 255.0` (centre fraction
`BRIGHTNESS_CENTRE_FRAC = 0.5`, mirroring the blur-detector ROI so
the brightness probe sees the same patch as the blur gate). Below
`BRIGHTNESS_LOW_THRESH = 0.12` the replenish path swaps:

| Parameter         | Steady-state | Low-light |
|-------------------|--------------|-----------|
| Feature target    | 200          | **120**   |
| `QUALITY_LEVEL`   | 0.05         | **0.10**  |

In low light, weak corners are dominated by sensor noise; doubling
the quality threshold rejects them, and lowering the target
acknowledges fewer strong corners are physically available. Net
effect: fewer but more reliable features survive into KLT.

The 0.12 threshold corresponds to a centre-crop mean of 32/255 on
8-bit gray. Empirically below that point phone cameras enter
ISO-pumped territory (auto-exposure has run out of shutter headroom
and is climbing the ISO ladder); at that point the corner SNR
collapses and `goodFeaturesToTrack` starts returning Hessian
responses that are mostly read-noise eigenvalues. The 0.12 figure
matches the existing `is_low_light` gate already used in
`Tracker.cpp` for the CLAHE skip — keeping a single brightness-low
constant across the codebase prevents the "two thresholds, one
concept" drift that ADR-006 §2 called out.

The replenish path uses a **two-state hysteresis** with
`BRIGHTNESS_HYSTERESIS_FRAMES = 5`: the low-light decision only
flips after 5 consecutive frames sustain the new state. This is the
defence against auto-exposure transitions: a sudden frame-darkening
from AE (lights off, walk into a hallway) takes the camera 100–300
ms to settle, during which the centre-crop mean oscillates around
the gate. Without hysteresis the replenish path would thrash between
target=200 and target=120 every 1–2 frames in the AE-settle window,
the new-feature-IDs would churn, and the SLAM-promotion gate
(`age ≥ 12`) would never accumulate.

`Tracker::MAX_FEATURES` and `Tracker::QUALITY_LEVEL` are not
mutated. Only the local effective targets used inside the replenish
call swap. The first-frame `detectGridFeatures` path keeps the
steady-state values — first-frame initialisation is rare enough
(once per session, plus on hard reset) that low-light adaptation
there has no measurable effect on drift.

A rate-limited LOGI fires every 30 replenish calls in the low-light
state with `LOWLIGHT: brightness=%.3f target=%d quality=%.2f` so
on-device sessions can confirm the gate is actually firing without
flooding logcat.

### Stricter pure-rotation detector (dual gate)

The current `Tracker.cpp` pure-rotation detector compares only the
gyro magnitude against 2.0 rad/s. Step 5 adds a second gate on the
**Rayleigh resultant of optical flow direction vectors**:

```
R = | Σ_i (cos θ_i, sin θ_i) |  (resultant length)
N = number of valid flow vectors
R / N → 1 means flow vectors are concentrated around one direction
        (translation or rotation-induced parallax shifts most pixels
         the same way)
R / N → 0 means flow vectors are spread uniformly over directions
        (textbook pure rotation about the optical axis, where each
         pixel rotates around the principal point and so the flow
         direction is a function of pixel position)
```

The pipeline declares pure rotation iff **both**

```
|ω| > 2.0 rad/s   AND   R / N < 0.3
```

The dual gate rejects scooter "looking around" cases the gyro-only
test misses: a rider pivoting at the waist while the scooter coasts
forward generates `|ω| ≈ 2.5 rad/s` *and* a coherent forward-flow
field (`R / N ≈ 0.7`). Gyro-only would skip the visual update; the
dual gate keeps it because forward parallax is still recoverable.

The Rayleigh test for unimodal angular concentration is Mardia 1972,
*Statistics of Directional Data*, §3.4. The 0.3 threshold corresponds
to "weakly concentrated" in the Mardia tabulation; we want the gate
to fire only on near-uniform flow, not on noisy translation.

### What is NOT done in this ADR

- **No automatic blur recovery.** Blurry frames are silently
  skipped at the visual-update level. Multi-frame blur bursts are
  recovered by the ADR-010 relocalization path, not by Step 5. A
  blur recovery primitive (e.g. de-blurring kernel + retrack) was
  considered and rejected as out of the per-frame budget.
- **No exposure control.** We do not poke `Camera2`'s manual
  exposure controls. AE is the OS / camera HAL's job; the front-end
  adapts to whatever AE produces.
- **No HDR fusion.** Multi-frame brightness fusion (e.g. exposure
  bracketing) was considered and rejected as out of scope — it
  doubles the per-frame compute and the AE settle-time problem
  hasn't been measured to justify it.
- **No ML-based blur classifier.** A Laplacian-variance gate is
  cheap (~0.3 ms per frame on a 320x240 centre crop) and the ROC
  is already at 95% recall on the Step 4 corpus. A learned classifier
  would have to clear that bar at the same compute cost; it does
  not, today.
- **No ML-based low-light classifier.** Same argument: the
  centre-crop mean already separates the regimes the on-device data
  shows, and the `is_low_light` gate already has consumers
  elsewhere in the pipeline.

## Consequences

**Positive**

- KLT track-loss events on rapid head turns drop materially
  (target ≥ 40% reduction per the plan's acceptance criterion). The
  adaptive window keeps the inter-frame displacement inside the
  search radius even when the gyro spikes.
- Low-light segments produce stable feature counts instead of
  thrashing 200-noisy / 200-noisy / 200-noisy. The SLAM-promotion
  gate sees coherent track histories on the 120 strong corners
  instead of three different IDs at the same pixel across three
  frames.
- Motion-blur frames stop poisoning the EKF with degenerate
  essential-matrix geometries. The IMU-only propagation across a
  blur burst is strictly better than a blurred-geometry pose
  update; the polyline can drift slightly during the burst but
  does not teleport.
- The dual-gate pure-rotation detector recovers usable visual
  updates on scooter "looking around" scenarios that the gyro-only
  gate previously skipped.
- All four mechanisms are local to the front-end and require no
  changes to `EKFState` / MSCKF / SLAM math. The decision surface
  for a regression bisect stays small.

**Negative / accepted**

- Centre-crop brightness probe adds ~0.05 ms per replenish call
  (sub-millisecond at 320x240). Acceptable; replenish itself was
  flagged at ~30 ms in this morning's perf pass, so the probe is
  noise relative to the dominant cost.
- Centre-crop Laplacian-variance blur probe adds ~0.3 ms per frame.
  Below the per-frame budget; perf-LOGI surfaces it so a regression
  shows up immediately.
- The two-state hysteresis on the brightness gate means a real
  walk-into-a-tunnel transition takes 5 frames (~167 ms at 30 Hz)
  to switch into low-light targets. Acceptable: the AE settle time
  is the same order, and overshooting by 5 frames produces ~5
  noisy-corner-driven KLT misses, which the steady-state pipeline
  already absorbs without drift.
- Adaptive KLT window costs more per-frame KLT compute when gyro is
  high. The 41-px upper bound is the budget guard; the perf-LOGI
  surfaces it so a regression shows up immediately.
- Dual-gate rotation detector is strictly stricter than gyro-only,
  which means a small set of *true* pure-rotation frames now go
  through the full visual update. That is the correct failure
  direction — a rotation-only update on a translation-degenerate
  geometry is bounded by the existing chi² + Huber gates from
  ADR-008; a translation-poisoned update from a missed translation
  case is not.
- Steady-state path is unchanged. A user walking on a sunlit
  flat street sees no behaviour change from this ADR — by design.

## Re-validation criteria

Before flipping any ADR-011 gate (the 80-variance blur threshold,
the 21–41 px window clamp, the 0.12 brightness gate, the
120-feature / 0.10-quality low-light pair, the 5-frame brightness
hysteresis, the 0.3 Rayleigh threshold) from its current default,
the following must hold:

1. **Build succeeds.** `gradlew :app:assembleDebug --offline` runs
   clean with the FeatureManager + Tracker changes (Agent A's
   Tracker work + this ADR's FeatureManager work) on a fresh tree.
2. **Unit contract.** `tests/cpp/test_visual_robustness.cpp`
   passes (Agent B's contract). The test pins:
   - Synthetic Gaussian-blur σ=5 frame → blur detector flags ≥ 95%.
   - Synthetic high-gyro 3 rad/s frame → adaptive KLT window grows.
3. **On-device head-turn corpus.** On the "rapid head turn"
   sub-corpus of `tests/sims/`, KLT track-loss events drop by **≥
   40%** vs the post-Step-4 baseline. This is the plan's stated
   Step-5 acceptance bar.
4. **No baseline regression.** Closed-loop gap on
   `tests/sims/baseline_walk_001.json` stays **≤ 1.79 m**, i.e. the
   Step 3b acceptance bar from ADR-009 must not regress under the
   adaptive front-end. Steady-state drift is the canary; if it
   moves, one of the gates is firing incorrectly on a steady-state
   scene.
