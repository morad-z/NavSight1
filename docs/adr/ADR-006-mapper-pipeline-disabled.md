# ADR-006 — Mapper pipeline disabled, kept as commented dead code

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

An earlier version of NavSight ran a `Mapper` background thread alongside
Tracker. Mapper consumed `TrackerFrame` snapshots, ran a `LoopClosureDetector`
plus a `PoseGraph` solver, and was supposed to apply pose corrections back
into the Tracker / EKF state via `applyMapperResult`.

Two problems showed up in real device testing:

1. **`applyMapperResult` was a no-op.** The callsite existed and the thread
   ran; the result was never injected. Conservatively measured CPU on the
   thread was 7–12% of one core for output that was discarded.
2. **When the corrector was enabled in a debug build,** loop-closure
   corrections produced visible "teleportation" — the map polyline
   would jump several meters when the corrector landed, because the
   pose-graph solution was not consistent with the inertial covariance
   of the points already published to the UI.

## Decision

The Mapper pipeline is **disabled** throughout the codebase. The C++
sources (`Mapper.cpp`, `LoopClosureDetector.cpp`, `PoseGraph.cpp`) and
the wiring inside `VioEngine` (background thread, mutex, condition
variable, result mailbox) are kept as commented blocks marked
`// DISABLED: Mapper pipeline …`. They are not deleted because:

- The team plans to revisit windowed bundle adjustment in a later
  release (post-thesis).
- The same-session loop closure goal is currently met by keyframe yaw
  updates inside the EKF — no separate map needed for short loops.
- Per user instruction during Step 8 cleanup ("comment, don't delete"),
  the inactive code stays in place with a clear marker.

`UpdaterMSCKF` is in the same category: still linked, never invoked.
Its update site at `Tracker.cpp:1193` is wrapped in the same DISABLED
block.

## Consequences

**Positive**

- ~10% of one CPU core returned to the rest of the pipeline.
- No more teleportation artifacts on the map view.
- The runtime path through `VioEngine::processFrame` is now strictly
  Tracker → EKF, with no asynchronous corrector mutating shared state
  behind the camera thread.

**Negative / accepted**

- We have no long-baseline loop closure right now. This is recorded in
  the production-readiness plan's non-goals.
- The dead code adds noise to greps. We mitigate by using the consistent
  marker prefix `DISABLED:` so it can be filtered.

## Re-enabling criteria

Before flipping any of this back on, an `applyMapperResult` implementation
must:

1. Inject corrections through an EKF update method (so covariance is
   consistent), not by overwriting `global_R_` / `global_t_` directly.
2. Damp the correction across N frames so the map polyline does not
   jump.
3. Be guarded by a runtime flag so it can be A/B-tested against the
   no-corrector baseline using the Step 7 replay harness.
