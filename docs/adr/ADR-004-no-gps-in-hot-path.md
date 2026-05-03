# ADR-004 — No GPS fusion in the hot path

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** NavSight team

## Context

NavSight's stated value proposition is **GPS-denied pedestrian/scooter
navigation**. Haifa, the deployment target, has experienced ongoing wartime
GPS jamming since late 2023: the jammers are transient, the corruption can be
indistinguishable from a valid fix at the API level, and standard fusion
recipes that down-weight GPS by `gacc` are demonstrably insufficient — a
jammed location can arrive with a small reported accuracy.

Treating GPS as just another EKF observer means that a jammed fix can drag
the estimate by tens of meters in a single update, after which the visual
side has to recover an obviously wrong state.

## Decision

**GPS is not fused into the EKF in the hot path.** Period.

GPS is used in exactly three places, all outside the estimator:

1. **Initial map centering.** The first valid GPS fix sets the start point
   for `metersToLatLng` so the Compose map opens centered on the user.
2. **Sim recording metadata.** Recorded sessions log `glat`, `glng`,
   `gacc` per sample so offline analysis (`scripts/analyze_sim.py`,
   `replay_scorer.py`) can compare VIO output to GPS *post-hoc* without
   ever feeding GPS into the runtime filter.
3. **UI hint only.** A "GPS disagrees by N m" banner is allowed, but it
   does not change the published position.

## Consequences

**Positive**

- A jammed GPS fix cannot teleport the estimate. The product behaves the
  way users in Haifa expect.
- The replay harness (Step 7) is fully deterministic: feeding the same
  sim JSON twice produces the same CSV, because no asynchronous GPS
  callback can perturb the state.
- The architecture diagram has one fewer arrow into the EKF.

**Negative / accepted**

- Long stationary outdoor periods cannot be re-anchored to absolute
  position from GPS. We compensate with same-session loop closure when
  the user returns to a previously-seen keyframe.
- Initial-heading bootstrap from GPS-derived course-over-ground is
  **not** used either — see ADR-005 for the magnetometer one-shot.

## What "hot path" means

"Hot path" = `EKFState::propagate*` and `EKFState::update*`. UI-side
logic, sim recording, and crash-snapshot JSON may freely include GPS so
long as nothing they do leaks back into the EKF.

## Override authority

Any future PR that wires GPS into an EKF update method must reference and
explicitly supersede this ADR.
