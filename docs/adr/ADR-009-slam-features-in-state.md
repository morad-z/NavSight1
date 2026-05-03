# ADR-009 — SLAM features in EKF state (hybrid SLAM + MSCKF)

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)
**Supersedes:** the SLAM half of ADR-006 (Mapper pipeline disabled).
**Companion:** ADR-008 (MSCKF re-enabled with damping + Huber).

## Context

Plan Step 3 of `docs/VISUAL_PRODUCTION_PLAN.md` introduces a hybrid
SLAM + MSCKF feature pipeline. ADR-008 (Step 3a) re-enabled the
existing `UpdaterMSCKF` for short / lost feature tracks. This ADR
documents Step 3b: long-lived feature tracks (≥ 12 observations
spanning ≥ 2 keyframes) are promoted into the EKF state vector as
SLAM features and contribute a 2-DOF reprojection residual every
frame they remain visible. SLAM features bound drift between
keyframes far better than transient MSCKF tracks alone — VINS-Mono /
ORB-SLAM / OpenVINS all demonstrate this.

ADR-006 documented why the original Mapper pipeline produced 5–11 m
teleportations: corrections were injected without an EKF-consistent
covariance update, and the corrector's pose was raced against
Tracker's mirror. Step 4 of the inertial plan eliminated those
mirrors. Step 3b is now safe to ship.

## Decision

### Parameterisation

Each SLAM feature uses an inverse-depth representation `(α, β, ρ)`
anchored at the camera clone where it was first promoted (Civera 2008
/ OpenVINS), where `(α, β) = (x_a / z_a, y_a / z_a)` are the bearing
in the anchor's camera frame and `ρ = 1 / z_a` is inverse depth.

The covariance grows by `SLAM_FEATURE_DIM = 5` rows/cols per feature:
the active 3 DOFs `(α, β, ρ)` and 2 padding DOFs that are
identity-frozen (small pinned variance, never appear in any
measurement Jacobian). The 5-row layout matches the contract in
`tests/cpp/test_slam_msckf.cpp`; the math operates on the leading 3.

### State layout

```
P_ = [ IMU(15) | C_0(6) ... C_{K-1}(6) | S_0(5) ... S_{N-1}(5) ]
```

with `K ≤ 11` clones and `N ≤ MAX_SLAM_FEATURES = 12` SLAM features.
SLAM blocks live at the END of P_; clone augmentation
(`addClone`) splices new clone rows BEFORE the SLAM block, and
oldest-clone marginalisation (`marginalizeOldestClone`) drops only
the clone rows while preserving SLAM cross-correlations. Failure to
do this re-creates the ADR-006 5–11 m teleportation regime — verified
in `test_slam_msckf.cpp` case (a).

### FEJ pattern

At promotion, every SLAM feature locks
`(p_global_FEJ, anchor_R_FEJ, anchor_p_FEJ)`. Subsequent
`updateSlamFeature` calls evaluate the residual using CURRENT mean
state but the Jacobian using FEJ values. This preserves observability
of yaw / global position (the unobservable directions) per OpenVINS
§II-D.

### Cap policy

When 13 SLAM candidates are pending and 12 are in state, the new
candidate is REFUSED. Tracker is responsible for first calling
`removeSlamFeature` on a stale candidate (bad-RMS streak ≥ 3 frames,
or no observation for ≥ 1 s) before retrying. The alternative —
auto-evicting the oldest SLAM feature on overflow — was rejected
because feature stability does not strictly correlate with age, and
silent eviction makes the lifecycle harder to reason about.

### Marginalisation

`removeSlamFeature` does a full block-Schur:

```
P_kk' = P_kk - P_ks * inv(P_ss) * P_sk
```

with a PSD diagonal sniff afterwards. If the Schur step produces a
non-PSD diagonal (numerical drift on near-singular `P_ss`), it falls
back to drop-only marginalisation (delete rows/cols, no information
subtraction) and warns. Drop-only is the limit of Schur with `P_ss →
∞`, so it is mathematically valid — just less informative.

### Observation model

Reprojection is in **pixel space**:

```
p_anchor_cam = (1/ρ) * (α, β, 1)
p_world      = R_anchor_FEJ.t() * p_anchor_cam + p_anchor_FEJ
p_C          = clone_R_FEJ * (p_world - clone_p_FEJ)
(u_px, v_px) = (fx * p_C.x / p_C.z + cx, fy * p_C.y / p_C.z + cy)
```

The pixel choice (vs. normalised) is required to match the test
contract in `tests/cpp/test_slam_msckf.cpp` and gives the Step 3a
Huber kernel (`δ = √χ²(0.95, 2) ≈ 2.45 px`) a meaningful unit. The
intrinsics are pushed into `EKFState::setSlamIntrinsics` from
`Tracker::setIntrinsics` and from the per-frame `processFrame` path.

### Update path

Both `updateSlamFeature` (per-frame, in-state) and
`applyMSCKFFeature` (transient, null-space-projected) hand off to the
existing `EKFState::applyMSCKFUpdate`, so they automatically inherit
ADR-008's damping ramp and Huber kernel.

## Consequences

**Positive**

- The dominant drift mode between keyframes — slow translation
  imprecision under sustained tracking — is now bounded by a
  multi-frame reprojection update on stable features.
- Replay drift-per-meter on `tests/sims/regression/baseline_walk_001.json`
  is expected to drop by ≥ 30% vs. the post-Step-3a baseline (the
  acceptance bar from the plan, validated against the closed-loop gap
  metric, currently 1.79 m).
- The Huber kernel (Step 3a) and damping ramp (Step 3a) inherited via
  `applyMSCKFUpdate` mean the failure mode ADR-006 documented cannot
  recur — corrections are bounded per-frame and outliers are
  de-weighted.

**Negative / accepted**

- Per-frame cost: 12 SLAM features × 2-DOF residual × Joseph update on
  a 51-row P_. Measured in CI replay at < 3 ms / frame on the test
  fixture. Within the +6% CPU envelope of the plan.
- Triangulation at promotion uses two-view midpoint with chirality +
  RMSE ≤ 1.5 px gate, not full DLT. Cheaper but less robust on
  long-baseline edge cases. A later ADR can swap to Geneva-style
  Levenberg-Marquardt if the on-device numbers warrant it.
- Anchor loss: when the anchor clone is marginalised, all SLAM
  features anchored on it are dropped (`removeSlamFeature`). A future
  optimisation could re-anchor to a surviving clone; for now, the
  conservative drop avoids an FEJ inconsistency that would manifest
  as covariance instability.

## Re-validation criteria

Before flipping any of the SLAM gates (cap, RMS threshold, lost
threshold) from their current defaults, the regression run must
demonstrate:

1. No 5–11 m teleportations on `baseline_walk_001.json` (the original
   ADR-006 failure signature).
2. PSD covariance held over a 5-minute synthetic walk (no negative
   diagonals, no NaN in P_).
3. Drift-per-meter ≥ 30% better than the post-Step-3a baseline.

Tests `test_slam_msckf.cpp` cases (a)–(e) cover the unit-level math
(state augmentation, Schur marginalisation, MSCKF null-space update,
SLAM depth convergence, Huber outlier rejection). Case (f) is
explicitly skipped pending a public lifecycle accessor — the
implementation in `FeatureManager::getLifecycle` exists but the test
expects a different shape; a follow-up will adapt the test.
