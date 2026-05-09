# Visual Plan — Step 7.1: Geometric Loop Closure

**Status**: spec
**Owner**: Morad
**Adds to**: `docs/VISUAL_PRODUCTION_PLAN.md` Step 7 (BoW + ORB + PnP)
**Depends on**: nothing new — uses existing keyframe descriptor ring and EKF state
**Companion ADR**: ADR-013 (existing Step 7) — extended, not superseded

---

## Why this exists

The shipped Step 7 path is BoW retrieval → ORB descriptor BFMatch → PnP. ORB
descriptors are **not** rotation-invariant past ~30°, so opposite-direction
revisits (typical U-turn pattern: "walk out 50 m, turn 180°, walk back")
fail at the ORB matching stage. The current code mitigates this with a hard
heading gate at `LoopClosureDetector.cpp:464-475` (`kMaxHeadingDiffRad =
M_PI / 2.0`) that rejects opposite-direction candidates **before** PnP runs.

The result: every U-turn within a single session — exactly the most common
loop a pedestrian produces — gets zero correction even when the same physical
landmarks are visible.

Step 7.1 adds a **second, parallel** detection path that doesn't depend on
appearance descriptors at all. It uses the 3D world-frame points already
triangulated and stored in each keyframe's `pts3d_world` field, and matches
them by *spatial proximity* and *geometric projection* rather than by
descriptor similarity. Direction-invariant by construction.

## Why this works for U-turns specifically

Two properties make the U-turn case much easier than global place recognition:

1. **Drift is bounded.** Round-trip walks of 50–200 m complete in 1–4 minutes
   of VIO. Typical drift is 1–5 % of distance, so the EKF position estimate
   when you return is 0.5–10 m off — small enough that the spatial
   neighbourhood of an old keyframe is still recognisable.
2. **Memory is fresh.** Keyframes outbound are still in the descriptor ring
   (`KEYFRAME_DESC_RING_SIZE = 50` ≈ 50 KFs ≈ ~25 s at 2 Hz keyframe rate, or
   longer with the dynamic keyframe trigger). For most U-turns the very 3D
   points triangulated outbound are still in memory on the return leg.

These two facts mean a position-based retrieval of recent keyframes, followed
by geometric verification using the EKF's predicted current pose to project
the candidate's 3D points into the current image, has a real chance of
producing a high-inlier PnP solve **without ever invoking ORB**.

## Design — purely additive

The shipped BoW path stays exactly as it is. Step 7.1 adds:

1. A new public method `LoopClosureDetector::tryDetectLoopGeometric(...)`
   alongside the existing `tryDetectLoop(...)`. Same `LoopMatch` output type,
   so the EKF-injection path (`Tracker::consumeLoopClosureMatchIfReady` →
   `EKFState::updateAbsolutePose`) is reused without modification.
2. A new EventCounters block — `loop_closure_geom_*` — distinct from the
   existing `loop_closure_*` counters so before/after metrics can be
   attributed cleanly.
3. A new call site in `Tracker::loopClosureWorkerLoop` that runs the
   geometric path **only when the BoW path didn't accept**. Fallback
   semantics, so the worker's result-pending slot stays single-source and
   the metric "marginal benefit of geometric over BoW" is directly
   measurable.

## Algorithm

Inputs (per worker tick):

- Current frame's KLT corners (image px) — already gathered for the BoW
  query.
- Predicted current camera pose (`R_world_cam_pred`, `t_cam_world_pred`)
  from the EKF + extrinsics.
- Camera intrinsics `(fx, fy, cx, cy)`, image dimensions `(W, H)`.
- Position search radius — derived from EKF position covariance trace, with
  floor.
- Temporal exclusion — same as BoW path (`30 s`).

Steps:

1. **Position-based candidate retrieval.** Under the detector's mutex, walk
   the keyframe ring. Filter by:
   - `|kf.t_cam_world − t_cam_world_pred| ≤ position_search_radius_m`, and
   - `kf.timestamp_ns ≤ now_ns − temporal_exclusion_ns`.
   Copy surviving keyframes' `pts3d_world` and pose out under the lock,
   then release. PnP runs lock-free.
2. **Project 3D-world points into the predicted current camera.** For each
   `p_world` in the candidate's `pts3d_world`:
   - `p_cam = R_world_cam_pred^T · (p_world − t_cam_world_pred)`
   - Skip non-finite or `Z ≤ 0.5 m` (behind camera or impossibly close).
   - `(u, v) = (fx·p_cam.x/p_cam.z + cx, fy·p_cam.y/p_cam.z + cy)`
   - Keep only points that fall in `[0, W] × [0, H]`.
3. **Per-candidate gate: minimum in-frame projections.** Reject candidates
   with fewer than `kGeomMinInFrame = 30` projected points landing in the
   image — too few overlap to ground a PnP solve.
4. **NN-match projections to current KLT corners.** For each surviving
   projection `(u_pred, v_pred)`, find the nearest KLT corner within
   `kGeomMatchRadiusPx = 15.0 px`. Build a list of `(p_world, p_image)`
   correspondences.
5. **PnP RANSAC.** Same parameters as the BoW path
   (`SOLVEPNP_ITERATIVE`, 100 iterations, 4.0 px reproj threshold, 0.99
   confidence). Require `≥ kPnpMinInliers = 15` inliers post-RANSAC.
6. **Build `LoopMatch`.** Identical conversion to the BoW path
   (`R_now_to_match`, `t_now_to_match`, `R_world_cam_match`,
   `t_cam_world_match`) so the EKF correction is the same.

Position search radius derivation:

```
sigma_p_xy = sqrt( max(P_xx, 0) + max(P_yy, 0) )
search_radius_m = max(2.0, 3.0 * sigma_p_xy)
```

3σ covers 99.7 % of the position distribution. The 2 m floor mirrors
`LOOP_CLOSURE_PNP_SIGMA_FLOOR_M`.

## Constants (new — none reused, all explicit)

| Name | Value | Rationale |
|---|---|---|
| `kGeomMinInFrame` | 30 | Same lower bound as BoW path's `kPnpMinInliers` × 2 — half of in-frame projections may not match a real KLT corner due to occlusion / different texture sampling. |
| `kGeomMatchRadiusPx` | 15.0 | KLT corner detector's `MIN_DIST = 10` plus margin for projection error. |
| `kGeomDepthFloorM` | 0.5 | Reuses BoW path's behind-camera depth gate. |
| `kGeomDepthCeilingM` | 50.0 | Reuses BoW path's ceiling. |
| `kGeomSearchFloorM` | 2.0 | Floor on `3·σ_p_xy`. |
| `kGeomSearchCeilingM` | 30.0 | Ceiling — beyond this, drift is too large for the U-turn case to be in scope. |

## Telemetry — new EventCounters block

| Counter | Increments when |
|---|---|
| `loop_closure_geom_attempts` | `tryDetectLoopGeometric` is entered with non-empty current KLT corners |
| `loop_closure_geom_accepts` | Returns true (PnP succeeds, ≥ 15 inliers) |
| `loop_closure_geom_rejects_no_position` | EKF position not initialised at query time |
| `loop_closure_geom_rejects_no_candidate` | No keyframe within position radius after temporal exclusion |
| `loop_closure_geom_rejects_few_inframe` | All candidates had < `kGeomMinInFrame` projections in-frame |
| `loop_closure_geom_rejects_pnp` | PnP threw / returned no inliers / < 15 inliers |

## Worker integration

In `Tracker::loopClosureWorkerLoop`:

```
if (have_query) {
  ok = loop_closure_.tryDetectLoop(...)              // existing BoW path
  if (!ok) {
    ok = loop_closure_.tryDetectLoopGeometric(...)   // new path, fallback
  }
  if (ok) publish LoopMatch  // unchanged downstream
}
```

The fallback ordering means BoW gets first crack on every query (preserving
existing behaviour), and the new path only runs when BoW didn't fire. This
gives a clean way to A/B in metrics:

- `loop_closure_corrections_applied` = BoW + Geom successes (all corrections)
- `loop_closure_corrections_applied − loop_closure_geom_accepts` = BoW-only
  corrections (= the pre-Step-7.1 baseline)

## Out of scope for Step 7.1

- **Cross-session persistent maps.** This step only uses the in-memory
  keyframe ring (≤ 50 KFs). Persistent map across app launches stays
  deferred to a future Step 12.
- **Coarse pre-filter for global recognition.** This step's spatial filter
  is correct only when the EKF's position estimate is approximately right
  (within 30 m). For longer-loop / cross-session cases, BoW remains the
  retrieval mechanism.
- **Replacement of the BoW path.** Step 7.1 is purely additive. ADR-013
  stays in force for the BoW path.

## Acceptance criteria

Acceptance proves the U-turn case improves and nothing else regresses:

1. **No regression on BoW-favourable fixtures.** Drift-per-meter and
   loop-closure-gap metrics on existing fixtures unchanged within ±2 %.
   Verified via Step 9 replay.
2. **U-turn fixture lifts.** A new fixture under
   `tests/sims/regression/visual/uturn_*.json` (recorded out-and-back
   walk) shows `loop_closure_geom_accepts ≥ 1` and an end-to-start
   `loop_closure_gap_m` reduction ≥ 30 % vs the same fixture replayed
   without the geometric path enabled.
3. **No spurious corrections.** On an IMU-only / no-vision regression
   fixture, `loop_closure_geom_accepts == 0` (no corrections out of nothing).

## Risk register

| Risk | Mitigation |
|---|---|
| EKF orientation wrong → projections way off → no NN matches → false negative | Self-protecting: bad EKF means no matches means no PnP means no inject. χ² gate at `EKFState.cpp:1000` is the second line of defense. |
| KLT corner thinning during fast motion → too few targets for NN match | Same `kGeomMinInFrame=30` gate that protects against this. |
| Spurious PnP inliers from ambient texture (e.g. asphalt patches) → wrong correction | `EKFState::updateAbsolutePose` runs χ²(0.999, 6) gate before applying. Damping ramp absorbs misfires across 10 frames per ADR-013. |
| Both BoW and geometric paths fire for the same query and disagree | Cannot happen with fallback semantics — geometric only runs if BoW returned false. |
| Worker tick budget exceeded (geometric path runs on every BoW reject) | Time-bound by the same RANSAC iteration cap (100). On a phone this is < 5 ms. Worker is 1 Hz so plenty of headroom. |
