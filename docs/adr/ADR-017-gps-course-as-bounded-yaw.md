# ADR-017 — GPS course-over-ground as a bounded yaw observation

**Status:** **WITHDRAWN 2026-05-07** — supersedes nothing.
**Reason for withdrawal:** NavSight is by design a VIO-only navigation
system. ADR-004 forbids GPS in the EKF hot path, and that decision
covers BOTH position and yaw — fusing GPS course-over-ground would
make the EKF dependent on GPS availability and quality, which violates
the product's core "works without GPS" promise. This ADR was drafted
in response to the 75° heading drift on sim 1778147132092, but the
correct response is to find an absolute heading anchor that does NOT
depend on GPS (continuous magnetometer with proper distortion
handling, better visual loop closure, or map matching using OSM data
without requiring GPS for position).
**Date:** 2026-05-07 (drafted), 2026-05-07 (withdrawn)
**Owner:** Morad Zubidat (sensor fusion)
**Supersedes:** Nothing. ADR-004's position AND yaw clauses remain
fully in force.

> The remaining content below describes the proposal as drafted, kept
> for reference only. **Do not implement.**
**Companion ADRs:** ADR-001 (Madgwick attitude reference),
ADR-005 (magnetometer one-shot only), ADR-008 (MSCKF damping + Huber),
ADR-013 (same-session loop closure).
**Numbering note:** ADR-014, ADR-015, ADR-016 are referenced from code
comments (`EKFState.{h,cpp}`) but no doc files exist. This ADR uses 017
to avoid retroactively claiming a number that production code already
attributes to a different change. Back-filling 014–016 docs is a
separate task.

## Context

A real daytime walk recorded as
`tests/sims/simulation_data_1778147132092.json` exposed that the EKF has
**no absolute heading anchor**:

- ADR-001 Madgwick yaw is gyro-integrated. Yaw is unobservable from
  accelerometer alone, so it drifts at the gyro-bias rate.
- The visual yaw channel (`updateGravityAlignedYaw`) had a
  sign-convention bug fixed in the parallel "heading-convention sign
  fix" change. Even with that fix, visual yaw is a *relative* observer
  (per-keyframe Δyaw on top of Madgwick) — it reduces drift rate, it
  does not re-anchor.
- Magnetometer is one-shot only (ADR-005) — no in-session reads.
- ADR-013 loop closure is the only absolute anchor and chi²-rejects
  when drift gets large, which is exactly the regime where it is most
  needed (the sim above accumulated 75° drift before returning near
  the start; loop closure corrections were rejected at the chi² gate
  because they would teleport).

Result on the cited sim: 75° heading drift over 8 minutes → 70+ m
endpoint position error vs GPS-confirmed return point.

GPS *course-over-ground* (the bearing of the velocity vector inferred
from consecutive GPS samples) is an absolute heading reference that
does not feed GPS *position* into the filter, so ADR-004's jamming
defence is preserved.

## Decision

**GPS course-over-ground IS used as an EKF yaw observation, gated and
variance-derived.** Position fusion remains forbidden per ADR-004.

Implementation lives in a new `EKFState::updateGpsCourseYaw` method
(see `ADR-017-gps-course-spec.md`). Internally it calls the existing
`EKFState::updateGravityAlignedYaw(course_rad, var_yaw, 0.0, 0.0)` to
reuse the same H-Jacobian path — there is no new EKF math.

### Gating thresholds (both derived)

- **`speed_mps > 1.5`.** Course noise scales as
  `(gps_accuracy_m / displacement_per_sec)`. With the worst-tolerated
  `gps_accuracy_m = 10` and a 1 Hz fused-location cadence,
  `v < 1.5 m/s` produces σ_course ≥ 6.7 rad — uninformative. 1.5 m/s
  is also a brisk-walk floor (≈ 5.4 km/h) which excludes standing-still
  GPS jitter that has no real heading signal at all.

- **`gps_accuracy_m < 10.0`.** Beyond this, with the speed gate above,
  `var_yaw = (gacc/(v·Δt))²` exceeds π² so the update carries no
  Fisher information. The hard reject saves CPU compared to a soft
  variance down-weight inside the EKF; the variance term inside the
  method is the second line of defence for marginal `gacc` values
  *below* 10.

Both thresholds are subject to empirical re-tune via the acceptance
criteria below. They are derivations, not gut-feel — the variance
model itself rests on the 1 Hz cadence and the (5°)² floor (next
section), which the re-walk validates or moves.

### Variance model

```
var_yaw_rad²  =  (gps_accuracy_m / speed_mps)²  +  (5° in rad)²
```

The first term is the small-angle propagation: GPS position
uncertainty `gps_accuracy_m` perpendicular to the walk direction
translates to angular uncertainty
`atan(gps_accuracy_m / displacement) ≈ gps_accuracy_m / displacement`.

The (5°)² floor is the residual phone-orientation-vs-walk-direction
uncertainty: even with a perfect GPS, the user does not walk *exactly*
in the direction the phone IMU thinks is forward (pocket carry,
hand-held tilt, sidestepping around obstacles). 5° is a defensible
upper bound for held-phone or pocketed-phone pedestrians; on a re-walk
we expect the empirical floor to land in 3°–8° and acceptance
criterion #6 below pins the test that confirms it.

### Jamming defence (ADR-004's reason for being)

ADR-004 forbids GPS *position* fusion because a jammed fix can
teleport the estimate by tens of metres in a single update. The same
attack is **not** as effective against the yaw channel:

1. **Hard outer gate** at `gps_accuracy_m < 10.0` rejects the most
   obvious jamming (hardcoded "fake fix" payloads typically carry a
   low or default `gacc`, but the speed gate then catches the
   inconsistency between successive fixes).
2. **Variance gate inside**: when `gacc` rises during a jamming event,
   `var_yaw = (gacc/(v·Δt))²` rises quadratically → Kalman gain
   collapses toward zero.
3. **Speed gate** at `v > 1.5 m/s` is a real-physics gate: a stationary
   user with a jammed fix has no course at all, so no observation is
   fed regardless of what the jammer reports.
4. **Course is computed from two consecutive samples**, so a single
   isolated jammed fix between two clean fixes produces course
   observations on both sides that the filter averages.

A jammer that produces *consistent, slowly-evolving* fake fixes
walking the user along a fictitious path *would* pull yaw — but that
is a class of attack ADR-004 itself cannot defend against.

### What is NOT done

- **No GPS *position* fusion.** ADR-004 stands.
- **No GPS *velocity vector* fusion** (only the *direction*).
- **No update during stationary periods** (speed gate is hard).
- **No update before VIO bootstrap** (`!full_initialized_` returns false).
- **No persistence across sessions** (ADR-013's same-session-only rule).

### Observability

Single 1-DOF absolute-yaw constraint at ≤ 1 Hz during sustained
motion. Compared to the per-keyframe gravity-aligned-yaw update
(~3 Hz, relative-via-keyframe), the GPS course channel is rare and
absolute — bounds yaw drift, does not over-constrain. Same 1-DOF rank
as the existing visual yaw path; no new states observed and no
existing observability claims change.

## Consequences

**Positive**

- Long-session yaw drift is bounded by an absolute reference whenever
  the user is walking outdoors with a clean GPS fix — exactly the
  regime where ADR-013 loop-closure χ² gate is *least* able to recover
  from accumulated drift.
- Variance is derived from `gps_accuracy_m` and `speed_mps` plus a
  justified floor — no magic number survives.
- Reuses existing H-Jacobian path → zero new EKF math.
- Failure mode is "no correction" — missing GPS, stationary user, or
  low-accuracy fix simply skips the update; degrades to ADR-001 +
  ADR-013 behaviour gracefully.

**Negative / accepted**

- ADR-004's "no GPS in the hot path" rule loses its yaw clause. We
  consider this acceptable because (a) the position clause carries the
  jamming defence and is preserved, (b) the yaw clause was written
  assuming visual relative-yaw + magnetometer one-shot would suffice,
  and the sim above shows it does not.
- The replay harness gains a dependency on GPS samples in the sim
  JSON. The samples are already recorded (`glat`, `glng`, `gacc`); only
  the replayer's consumption path changes.
- Indoor sessions (no GPS or always above 10 m accuracy) get no
  benefit from this channel. Status quo; nothing regresses.

## Acceptance criteria

Before flipping this channel on by default, all of the following must
hold on a re-walk of the cited sim and at least one fresh outdoor
walk:

1. **EKF yaw tracks GPS course within ±15° during sustained motion**
   (segments where `speed > 1.5 m/s` and `gps_accuracy < 10` for ≥ 5
   consecutive seconds). The 75° drift on the original sim must drop
   below 15° on the re-walk.

2. **Endpoint position error reduces.** The 70+ m endpoint error on
   the cited sim drops to < 20 m on replay with this channel enabled,
   bug-fixed visual yaw active, loop closure otherwise unchanged.

3. **Graceful fallback when GPS degrades mid-walk.** A sim segment
   where `gps_accuracy_m` is forced > 10 m (synthetic injection)
   produces no state shock — yaw drifts continuously under
   gyro+visual until the next clean fix, no discrete jump.

4. **No regression on indoor / GPS-free sessions.** A purely indoor
   sim (no GPS samples that clear the gates) reproduces the
   pre-ADR-017 baseline trajectory bit-identically.

5. **No false yaw push under simulated jamming.** A synthetic test
   that injects "jammed" fixes (small `gacc`, position lying by 30 m
   perpendicular to true track) produces a yaw correction below 5°
   per fix and zero accumulated bias over a 30-fix sequence.

6. **Empirical (5°)² floor validated or re-tuned.** Distribution of
   `(measured course − EKF yaw)` on clean sustained-motion segments
   has RMS in 3°–8°. Outside that range, the floor is re-tuned to the
   measured value and this ADR is revised before merge.

## Dependency

**This ADR must not land before the parallel "visual-yaw heading
convention sign fix" merges.** `updateGpsCourseYaw` reuses the
existing `updateGravityAlignedYaw` H-Jacobian path. If the H-path is
broken, this observer would push yaw the wrong way at 1 Hz with a
derived-low-variance update — converting today's "unanchored drift"
failure into "confidently wrong heading," which is strictly worse.
Merge order: heading sign fix first → this ADR → enable in
`Tracker::onLocationUpdate`.

## Override authority

This ADR supersedes only the yaw clause of ADR-004. Any future PR
that wires GPS *position*, *altitude*, or *velocity-magnitude* into
an EKF update method must reference and supersede ADR-004 explicitly
— this ADR does not provide that authority.
