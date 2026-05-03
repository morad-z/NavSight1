# ADR-002 — Error-State Kalman Filter (not full EKF, not factor graph)

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

The core fusion problem is: given Madgwick attitude, preintegrated IMU, KLT
relative motion, MiDaS depth, ZUPT, and PDR step events, produce a metric
horizontal-plane position with calibrated uncertainty at 30 Hz on a phone.

Three architectures were on the table:

1. **Full state-space EKF** with 16-DOF nominal state
   `[q(4), v(3), p(3), b_g(3), b_a(3)]`. Update directly on the nominal
   quaternion.
2. **Error-State Kalman Filter (ESKF)** in OpenVINS / MSCKF style.
   15-DOF error state `[δθ(3), δb_g(3), δv(3), δb_a(3), δp(3)]`, nominal
   state stored separately, error reset after each correction.
3. **Sliding-window factor graph** (GTSAM, Ceres). Higher accuracy, but at
   ~3–5x the per-frame CPU on the phones we tested, plus a dependency
   we don't want to ship.

## Decision

Implement an **ESKF**, with the error-state structure shown above and a
sliding window of 11 camera clones for MSCKF-style relative-pose updates.
Implementation lives in `app/src/main/cpp/EKFState.{h,cpp}`. The error reset
after each update is the textbook injection
`q ← q ⊗ exp(½ δθ)`, performed inside `applyMSCKFUpdate`.

## Consequences

**Why ESKF over full EKF**

- Quaternions are minimally parameterized in error space (3 DOF) but
  over-parameterized in nominal space (4 DOF with a unit-norm constraint).
  Running covariance on 4-DOF quaternion creates a singular block; ESKF
  sidesteps the issue.
- The first-estimate Jacobians (FEJ) trick that maintains observability of
  global yaw and position is well-studied in the OpenVINS literature for
  ESKF, not for full EKF.
- Update-side equations are linear in the error state, so we get to keep
  the Joseph-form covariance update without ad-hoc symmetrization.

**Why ESKF over factor graph**

- 30 Hz on a Snapdragon 695 with ESKF + KLT + MiDaS leaves enough headroom
  for the Compose UI. A GTSAM iSAM2 solve at the same window would push us
  over budget on the phones the SDD targets.
- The sliding-window structure (max 11 clones) gives us most of the
  short-baseline benefit of a graph without an external solver.
- Maintenance: GTSAM is a hard dependency to cross-compile for Android NDK.
  ESKF is plain C++17 with OpenCV linear algebra.

## Cost we accept

- Long-term loop closure is weak. We have *same-session* keyframe-based
  loop matching but no persistent map. The plan's non-goals call this out
  explicitly.
- Yaw observability stays marginal until a strong visual-inertial scale
  observation lands. Below that, gyro bias and yaw drift are coupled.
  ScaleEstimatorVI + the keyframe yaw update are how we resolve this.

## Forward path

If we ever ship a Pixel 9-class device as the minimum target, revisit
GTSAM-based windowed BA. Until then, ESKF is the right tradeoff.
