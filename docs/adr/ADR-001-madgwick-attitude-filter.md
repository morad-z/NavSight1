# ADR-001 — Madgwick as the attitude reference

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

NavSight runs on commodity Android phones with a noisy, biased 6-axis IMU
(MPU-65xx-class) sampled at 100–500 Hz. We need a continuously-updating
attitude reference that:

1. Converges within seconds of stationary startup with no calibration step.
2. Survives the camera dropping out (covered camera, dark stairwell).
3. Has a CPU cost under 1% of one core so it can run alongside the EKF, KLT,
   and Compose UI on a mid-range phone.
4. Is well enough understood that its failure modes are predictable.

The candidates were:

- **Madgwick gradient-descent filter** — quaternion form, single tunable gain,
  ~50 lines of code, published thesis (Madgwick 2010, *AHRS algorithms…*).
- **Mahony complementary filter** — similar cost, two gains, slightly less
  accurate in dynamic motion.
- **Full EKF on attitude alone** — overkill for a 4-state quaternion problem
  and adds maintenance burden of an extra filter beside the main ESKF.
- **Phone built-in `TYPE_ROTATION_VECTOR`** — vendor-specific, opaque, and on
  several test devices fused magnetometer in ways we cannot turn off
  (see ADR-005 for why that disqualifies it).

## Decision

Use **Madgwick** as the attitude reference. Implementation lives in
`IMUPreintegrator` (the gravity / orientation tracker shared with
preintegration), with a single `beta` gain.

The Madgwick output is consumed in three places:

1. As the gravity-alignment frame for `EKFState::updateGravityAlignedYaw`
   (roll/pitch sandwich for the keyframe yaw H matrix).
2. As the initial seed `R_GtoI` handed to `EKFState::initializeFull` after the
   InertialInitializer's stationary gate passes.
3. As the rotation prior fed into `TrackKLT::track` for gyro-aided optical flow.

## Consequences

**Positive**

- One filter, one gain, one paper to cite. Easy to debug and easy to teach.
- Recovers cleanly after a few seconds of stationary motion.
- No magnetometer dependency in the runtime loop (see ADR-005).

**Negative / accepted**

- Yaw is unobservable from accelerometer alone. The Madgwick yaw at startup is
  whatever the gyro integrates from the initial frame, so a separate
  one-shot magnetometer bootstrap (or last-known heading) is needed for the
  *absolute* heading. Tracker handles this seed.
- Under sustained high-G motion (drops, hard scooter braking), the gradient
  step can over-correct. We mitigate by gating the EKF yaw update with
  `last_visual_yaw_variance_` so the visual channel dominates when it is
  healthy.

## Alternatives reconsidered

If we ever add a high-end IMU with a real gyro bias-stability spec, we should
re-evaluate against a 3-state attitude EKF that estimates gyro bias online.
Until then, the simpler filter wins.
