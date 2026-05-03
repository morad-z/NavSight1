# ADR-008 — MSCKF re-enabled with damping ramp + Huber kernel

**Status:** Accepted (supersedes the MSCKF half of ADR-006)
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

ADR-006 disabled the MSCKF call site at `Tracker.cpp:1272` because
corrections produced visible 5–11 m teleportations. The root cause was
a stale-state mismatch: Tracker held its own `global_R_` / `global_t_`
mirrors that the EKF could not see, so MSCKF residuals were applied
against state the rest of the pipeline had already moved past.

Step 4 of the inertial production plan eliminated those mirrors —
`EKFState` is now the single owner of pose. With the original failure
mode gone, the existing (and already correct) `UpdaterMSCKF` path —
DLT triangulate, FEJ Jacobians, SVD null-space projection, chi² gate,
QR compression, Joseph update — can be safely re-enabled, provided
two residual risks are mitigated:

1. **First-frame impulse.** When the EKF receives an MSCKF correction
   after a quiet period, even a small δp can land as a visible jump
   on the polyline.
2. **Outlier residuals.** `processLostFeatures` already runs a
   per-feature 95 % chi² gate, but it is binary: either the entire
   feature contributes fully or it contributes nothing. A single
   marginal observation inside an otherwise good track can still
   degrade the update.

## Decision

Re-enable MSCKF (Plan Step 3a) at `Tracker.cpp` section 11.1, gated by
two new mechanisms inside `EKFState::applyMSCKFUpdate`:

### 1. Position-correction damping ramp

Linear ramp on the world-frame δp rows (12..14 of the IMU error-state):

| Call after quiet period | Damping factor |
|-------------------------|----------------|
| 0 (first)               | 0.5            |
| 1                       | 0.6            |
| 2                       | 0.7            |
| 3                       | 0.8            |
| 4                       | 0.9            |
| ≥ 5                     | 1.0            |

A "quiet period" is ≥ 5 `propagateIMU` calls without an MSCKF call;
when that elapses the schedule resets to step 0. Velocity (rows 6..8),
attitude (0..2), bias (3..5, 9..11), and clone corrections are not
damped — the polyline-jump risk is local to body δp.

### 2. Per-residual Huber kernel

Replaces the implicit binary chi² accept/reject with a soft, per-row
weighting using `δ = √χ²(0.95, 2 dof) ≈ 2.4477`:

- `m_i ≤ δ`        → weight = 1 (full influence)
- `δ < m_i < 3δ`   → weight = δ / m_i (linear de-weighting)
- `m_i ≥ 3δ`       → weight = 0 (hard reject)

`m_i = |r_i| / √S_ii` is the per-row normalised residual, with
`S = H · P · Hᵀ + R`. Weights fold into `H` and `r` row-scales so the
downstream Joseph-form update is unchanged. A counter
`msckf_huber_rejected_count_` tracks hard-rejects per call and is
LOGI'd alongside the damping factor.

## Validation

- Build: `gradlew :app:assembleDebug` succeeds (Plan Step 3a build
  verify).
- Regression baseline: `tests/sims/regression/baseline_walk_001.json`
  closed-loop return gap stays ≤ 1.93 m (post-Step-2 value).
- LOGI tags `MSCKF Huber: rejected=… dampened=… total_rows=…` and
  `MSCKF update applied: max_correction=… damping=… huber_rejected=…`
  surface activity in logcat for on-device verification.

## Consequences

**Positive**
- MSCKF features now contribute to drift reduction without the
  teleportation failure ADR-006 documented.
- Outlier observations are de-weighted continuously instead of being
  thrown out feature-at-a-time.
- The ramp is short (≤ 5 calls) so steady-state behaviour is
  unchanged; damping only protects the cold-start.

**Negative / accepted**
- Step 3b (SLAM features in EKF state) is now in scope and shipped
  per ADR-009. ADR-008 supersedes the *MSCKF half* of ADR-006; ADR-009
  supersedes the SLAM/Mapper half. The original Mapper / LoopClosure /
  PoseGraph blocks remain disabled — Step 7 (loop closure with DBoW2)
  will be the next supersession.
- Existing `UpdaterMSCKF` outer chi² gate is kept as a safety net;
  the Huber kernel is layered in front of it, not in place of it.

## Note on damping interaction with non-MSCKF measurement updates

`applyMSCKFUpdate` is the shared Joseph-form primitive used by every
EKF measurement channel: `updateRelativePose`, `updateRelativeRotation`,
`updateGravityAlignedYaw`, `updatePDRStep`, `updateZUPT`, the MSCKF
feature update from `UpdaterMSCKF::processLostFeatures`, **and** the
SLAM feature update from `EKFState::updateSlamFeature` (added in Step
3b). Each call advances the damping step counter and resets the
quiet-period counter the same way.

In practice this means:
- On a frame with active visual tracking, the damping ramp saturates
  at 1.0 within the first frame because R_vo + relative-pose + SLAM
  updates all fire. The damping schedule only matters for the
  **first MSCKF call after a quiet period** (e.g. recovery from a
  loss-of-tracking interval), which is exactly the cold-start
  teleportation regime ADR-006 documented.
- The damping is applied only to `dx(12..14)` (world-frame δp),
  never to attitude, velocity, or biases — so SLAM mean corrections
  on `(α, β, ρ)` are unaffected regardless of which channel fired
  the update.

If a future change wants per-channel damping (e.g. damp MSCKF
position corrections more aggressively than SLAM), split the counter
into per-channel state. This is not currently needed.
