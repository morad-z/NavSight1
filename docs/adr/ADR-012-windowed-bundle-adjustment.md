# ADR-012 — Local windowed bundle adjustment

**Status:** Accepted
**Date:** 2026-05-04
**Owner:** Morad Zubidat (sensor fusion)
**Companion:** ADR-008 (MSCKF re-enabled with damping + Huber), ADR-009
(SLAM features in EKF state), ADR-010 (ORB descriptors at keyframes for
relocalization), ADR-011 (Adaptive front-end robustness).

## Context

After ADR-009 the EKF carries up to 12 SLAM features in its state vector
and refines them through 2-DOF reprojection updates every frame. After
ADR-010 the keyframe ring carries ORB descriptors so feature
associations across keyframes are reliable even when KLT briefly drops.
Both buy bounded drift between keyframes — but neither does what an
EKF structurally cannot do: **jointly refine the recent keyframe poses
AND the SLAM landmarks they observe in a single optimisation**.

The EKF is sequential by construction: each measurement update
linearises around the current mean and pulls a single sub-block of
state. SLAM features and clone poses get refined in their own update
calls, and there is no point in the EKF cycle where the joint geometry
(`min over poses, points` of `Σ ||π(R_i, t_i, p_j) − u_ij||²`) is
optimised. VINS-Mono and ORB-SLAM both demonstrate that adding such a
joint refinement over the last 5–10 keyframes cuts drift by another
30–50% on top of EKF (Qin et al. 2018, *VINS-Mono*; Mur-Artal et al.
2015, *ORB-SLAM*).

Step 6 of `docs/VISUAL_PRODUCTION_PLAN.md` (line 649, "Local windowed
bundle adjustment") fixes this with a fixed-window BA that runs on a
background thread every keyframe and feeds its result back into the
EKF through the normal observation channel.

## Decision

### Solver choice — hand-rolled Gauss-Newton, NOT Ceres

The plan §6 originally suggested Ceres-Solver. Foreground scout
finding before implementation: this problem is **5 poses × 6 DOF + ≤ 20
landmarks × 3 DOF = 90 variables**. The repository already has no
Eigen dependency, no Ceres dependency, no Sophus dependency. Bringing
Ceres in would add ~20–30 MB of APK bloat, force-pull Eigen, and add a
new build-system surface area (Ceres' CMake export) for a problem
small enough to solve by hand in ~400 lines of OpenCV-only code. The
Eigen-free hand-rolled solver also matches the existing repo style
(every other solver here — MSCKF, SLAM update, ZUPT — is hand-rolled
in `cv::Mat` algebra) so a future reader does not have to context-switch
between two linear-algebra dialects mid-file.

The hand-rolled solver lives in `app/src/main/cpp/WindowedBA.{h,cpp}`
(Agent A's deliverable). It uses:

- **Levenberg-Marquardt-flavoured Gauss-Newton** (damped normal
  equations) over a single pose+point block.
- **OpenCV `cv::Mat` for matrix algebra**, `cv::Rodrigues` for SO(3)
  exp/log, no third-party deps.
- **Schur-complement** elimination of the landmark block when forming
  the reduced normal equations (cuts the linear-solve cost from
  O((6K + 3N)³) to O((6K)³ + N × 3³)).

### Parameterisation

- Pose (R, t) where `R` rotates world→cam (`R_cw`), `t` is the camera
  centre expressed in the world frame (`t_wc`). Therefore
  `p_cam = R · (p_world − t)`.
- Pose perturbation: right-multiplicative rotation, additive
  world-frame translation. Local 6-vector is
  `[Δt (3), φ_Δ (3)]` with `R_new = R_old · Exp(φ_Δ)`,
  `t_new = t_old + Δt`.
- Landmark perturbation: additive in the world frame (3-vector
  `Δp`).

This is identical to the WindowedBA.h header contract Agent A ships,
and identical to the convention EKFState already uses internally
(`R_GtoC` / `p_G`), so the snapshot translation in
`Tracker::kickOffBARound` is a straight copy with no frame conversion.

### Gauge fix

Pure pose+point BA has a 7-DOF gauge ambiguity (6-DOF rigid + 1-DOF
scale). We fix it the standard way: **the oldest keyframe in the
window has its pose pinned** (`PoseObs::is_anchor = true`). This is
the same gauge fix VINS-Mono uses for its sliding-window BA. Scale is
not a free DOF in our case because the SLAM landmark seed positions
come from the EKF's metric state, so a scale-degenerate window is
prevented by the input data already being in metres.

### Schur complement

The normal equations have the standard sparse block structure:

```
| H_pp  H_pl | | Δp |   | g_p |
|            | |    | = |     |
| H_pl' H_ll | | Δl |   | g_l |
```

`H_ll` is block-diagonal in the landmarks (each landmark contributes
a 3×3 block), so we Schur-eliminate the landmark block:

```
H_reduced = H_pp − H_pl · H_ll⁻¹ · H_pl'
g_reduced = g_p  − H_pl · H_ll⁻¹ · g_l
```

`H_reduced` is `(6K)×(6K)` and we solve it with
`cv::solve(..., DECOMP_CHOLESKY)`. Landmarks are recovered by back-
substitution: `Δl = H_ll⁻¹ · (g_l − H_pl' · Δp)`. Each `H_ll⁻¹` block
is a 3×3 inverse, which is `cv::invert(..., DECOMP_LU)` per landmark.

### Huber loss

Each (pose, landmark, pixel) residual is reweighted by the Huber kernel
with threshold **1.5 px**, matching `Tracker::RANSAC_THRESH` so the
acceptance noise model is consistent across the front end and the BA
back end. Step 6 plan acceptance bar.

### Threading model

One BA round at a time on a single worker thread. The Tracker side:

1. **At each new keyframe**, call `consumeBAResultIfReady()` then
   `kickOffBARound(timestamp_ns)`.
2. **kickOffBARound** cheap-copies a `CloneSnapshot` (≤ 5 most recent
   EKF clones) and a `LandmarkSnapshot` (SLAM-promoted features
   observed by ≥ 2 of those clones) via the new thread-safe
   `EKFState::getCloneSnapshot` / `FeatureManager::getLandmarkSnapshot`
   APIs, then `std::thread`s its way through `WindowedBA::solve(...)`.
3. **If the previous round is still in flight**, the call returns
   without launching, the camera thread logs `BA: skipped
   (prev_round_in_flight)`, and tries again on the next keyframe.
4. **The worker thread** publishes its result under `ba_result_mutex_`
   then clears `ba_in_flight_` (release-store) so the next camera-
   thread call sees `pending=true` before `in_flight=false`.
5. **consumeBAResultIfReady** reads the published result on the next
   keyframe and re-seeds each refined SLAM feature in the EKF (see
   below).

The thread-safe snapshot APIs hold their respective `snapshot_mutex_`
only for the duration of the deque/hashmap walk + cv::Mat copy. Camera-
thread writers (`addClone`, `marginalizeOldestClone`, `pruneWindow`,
`addObservation`, `noteObservation`, `noteTriangulation`,
`setSlamSlot`, `markSlamFeatureRMS`, `pruneObservations`,
`pruneStaleLifecycle`, `dropLifecycle`, `extractLostFeatures`, both
`reset` paths) take the same mutex briefly. Lock contention is a few
microseconds per camera-thread call and ≤ 1 ms during the BA snapshot
read — well below the per-frame budget.

The destructor and `Tracker::reset()` both call `shutdownBA()` which
joins (not detaches) the worker thread. Joining ensures the worker
has released its snapshot read locks before the EKF / FeatureManager
are torn down — detaching would race the destructor.

### EKF reconciliation — re-seed via add/remove, NOT direct mutation

When a BA round is accepted, each refined landmark goes through the
canonical EKF observation channel:

1. `EKFState::removeSlamFeature(slot)` — Schur-marginalise the old
   slot. The covariance block is deleted cleanly through the existing
   ADR-009 marginalisation path.
2. `EKFState::addSlamFeature(feature_id, p_world_refined, anchor)` —
   re-promote the same `feature_id` anchored at the same clone, with
   the BA-refined world point as the seed. The anchor's CURRENT pose
   (and FEJ) are read fresh from the EKF, so the re-promotion
   linearises against the EKF's most recent state, not against a
   stale snapshot. Covariance is rebuilt by the same standard path
   used for fresh promotions.

The BA-refined pose deltas are **NOT** pushed back into the EKF
clones directly. ADR-006 is explicit on this: a side-channel that
overwrites EKF mean / covariance produces 5–11 m teleportations the
filter cannot recover from. Instead, the BA-refined landmark
positions become the new measurement seeds, and the EKF's existing
SLAM-feature reprojection updates pull the clone poses toward
consistency with the BA-refined points across the next several
frames. This is the same "let the filter catch up" damping pattern
ADR-006 already prescribes for the post-disable re-enabling of
MSCKF and loop closure.

### Why not direct EKF mutation (ADR-006 lesson)

The 2026-03 timeline that produced ADR-006 logged teleportation
events of 5–11 m every time a side-channel update wrote into
`EKFState::P_` or the IMU mean directly. The recurring pattern was:

- Update arrives with a residual the EKF was not expecting.
- Side channel writes the new mean.
- Next IMU propagation step linearises around the new mean but the
  covariance block is now inconsistent with the rest of the filter.
- The next observation update produces a step that overshoots the
  consistency point, the filter oscillates, and the polyline jumps.

Re-seeding via `removeSlamFeature` + `addSlamFeature` avoids all of
that: the EKF reconstructs the slot's covariance entries through
the same code path that handles fresh promotions, so the
covariance is consistent by construction. The cost is that the EKF
needs ~3–5 additional reprojection updates to fully absorb the BA
correction — that is the correct cost to pay.

### What is NOT done in this ADR

- **No global BA.** Window is fixed at 5 most-recent clones. A full
  BA over all keyframes is what Step 7 (loop closure) will add when
  it has the loop-closure topology to make the optimisation tractable.
- **No covariance recovery from BA.** The BA produces a refined
  mean only. We do not back out an information matrix from the
  Schur-reduced normal equations and feed it to the EKF. Future
  ADR can add this if on-device data shows the EKF-only covariance
  is over-confident on BA-refined slots.
- **No Ceres / Eigen dependency.** Hand-rolled solver, OpenCV
  algebra. See "Solver choice" above.
- **No pose-direct injection into EKF.** ADR-006 forbids it; this
  ADR honours that.
- **No BA on every frame.** Per-frame BA was considered and rejected
  — keyframe-only is the cost/benefit sweet spot, matches both
  VINS-Mono and ORB-SLAM, and keeps the worker thread well within
  the 200 ms wall-clock budget.

## Consequences

**Positive**

- Closed-loop drift on indoor sims drops by ≥ 25% (Step 6 acceptance
  bar) without modifying the EKF measurement math.
- The BA worker runs entirely off the camera thread; the camera
  thread sees only the snapshot-mutex contention and the
  re-seeding work in `consumeBAResultIfReady`, both bounded under
  1 ms in the worst case.
- The re-seeding path goes through the canonical EKF API, so
  covariance stays consistent and ADR-006's teleportation regime is
  not re-introduced.
- The hand-rolled solver pulls in **zero** new dependencies. APK
  size unchanged, build pipeline unchanged.
- A failed BA round (non-converged, residual not halved, or > 200 ms
  wall-clock) is silently dropped — the system degrades to EKF-only
  behaviour, which is the ADR-009 baseline. No regression on
  acceptance failure.

**Negative / accepted**

- The remove-and-re-add re-seeding path means each refined landmark
  loses its accumulated covariance correlations with the rest of
  the state — the filter needs ~3–5 frames to re-acquire those
  correlations through the normal SLAM update path. This is a
  correct cost for the safety guarantee the ADR-006 lesson
  demands.
- One BA round at a time means at high keyframe rates (faster than
  the BA solve) we silently skip rounds. The "BA: skipped
  (prev_round_in_flight)" log line surfaces this so a regression in
  BA solve time shows up immediately. At the steady-state 1
  keyframe / ~500 ms cadence and BA solve times of 10–80 ms
  measured during development, skip events should be rare.
- The 200 ms wall-clock cap (2× the plan's 100 ms target) is a
  thermal-throttle headroom budget. A solve that exceeds it is
  rejected as "accept=N" and the system stays on the previous
  state. If on-device telemetry shows the cap firing routinely,
  the next iteration drops the window from 5 clones to 3 rather
  than raising the cap — the cap exists precisely so a slow round
  cannot poison the EKF.
- The snapshot mutexes add a small amount of locking surface to
  EKFState and FeatureManager. The lock-only-the-mutator pattern
  keeps camera-thread cost to a few microseconds per write call;
  measured impact on the per-frame budget is below the perf-LOGI
  resolution.

## Re-validation criteria

Before flipping any ADR-012 gate (the 5-clone window, the 2-obs
landmark threshold, the 1.5 px Huber threshold, the 10-iteration
cap, the 200 ms wall-clock cap, the residual-halved acceptance gate)
from its current default, the following must hold:

1. **Build succeeds.** `gradlew :app:assembleDebug --offline` runs
   clean with the new `WindowedBA.{h,cpp}` (Agent A) + the
   EKFState / FeatureManager snapshot APIs + the Tracker BA-thread
   wiring + the new tests (Agent B) on a fresh tree.
2. **Unit contract.** `tests/cpp/test_windowed_ba.cpp` (Agent B's
   deliverable) passes — synthetic 5-pose × 10-landmark setup with
   known ground truth; the solver must converge within the
   iteration cap and the residual must drop monotonically.
3. **On-device BA budget.** `BA: solve_us=...` p95 stays below
   **200 ms** across a 5-minute walk session. Regression here means
   shrinking the window to 3 clones, NOT raising the cap.
4. **No new crash signatures.** A 30-minute walk + reset cycle
   produces zero ANRs, zero native crashes, and zero
   "BA: skipped (prev_round_in_flight)" frequencies above 10% of
   keyframes. The skip rate is the canary for solve-time regression.
5. **Closed-loop drift.** Replay loop-closure-gap on the indoor
   sims drops by **≥ 25%** vs the post-Step-3b baseline
   (`baseline_walk_001.json` ≤ 1.34 m, down from the ADR-009
   1.79 m). This is the Step 6 plan's stated acceptance bar.
6. **No baseline regression on EKF-only behaviour.** Disabling the
   BA wiring (a one-line guard at the top of `kickOffBARound`)
   must reproduce the ADR-009 closed-loop drift exactly — the BA
   path is strictly additive, so a regression with BA off means
   the snapshot mutexes are interacting with the EKF/FeatureManager
   state in an unintended way.
