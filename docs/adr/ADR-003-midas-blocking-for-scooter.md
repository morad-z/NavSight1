# ADR-003 — MiDaS depth as a blocking observer for scooter mode

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

The metric scale of monocular VIO is observable only when the device
accelerates *non-gravitationally*. Walking provides this through the periodic
heel-strike impulse that the PDR step detector picks up. Scooters do not:
the dominant motion is constant-velocity glide, with little vertical
oscillation, so PDR cannot estimate stride and the visual-inertial scale
estimator (ScaleEstimatorVI) becomes ill-conditioned.

Symptom in early sims: scooter recordings showed a path that drifted to
near-zero scale within 20 m, then stayed there — the EKF locked onto a
small-scale solution that satisfied the visual-only constraint while the
absolute distance reported to the UI was off by 10–30x.

## Decision

In scooter mode, **MiDaS monocular depth is a blocking observer** for the
metric scale. The pipeline is:

1. The TFLite MiDaS small model runs at ~1 Hz on a worker thread.
2. The output relative depth map is sampled at the same pixels Tracker is
   currently using as keyframe features.
3. The ratio `depth_metric / depth_visual` is computed per point. The MAD
   over the sampled population gives the variance of the scale ratio.
4. ScaleFuser receives `(z = scale, var = MAD² · 1.4826²)` and updates the
   shared 1-D scale Kalman state.
5. **Until the first MiDaS measurement has been fused**, scooter-mode
   position output is held at the last-known good and the UI shows
   "initializing" via `VioStatusChip`. This is the "blocking" part of the
   decision: we refuse to publish positions we cannot ground in metric
   scale.

## Consequences

**Positive**

- Eliminates the silent scale collapse on scooter recordings.
- Brings scooter-mode error to ~5% over 100 m (target in the production
  acceptance criteria).
- The UI degradation is honest: rather than show a wrong line, we show
  "waiting for scale".

**Negative / accepted**

- Cold-start latency is ~1 s on scooter — the time for one MiDaS inference
  + one fuse cycle. Walking does not need MiDaS, so cold-start there is
  unaffected.
- MiDaS at 1 Hz costs ~120 ms per inference on a Snapdragon 695. We
  amortize by running on a separate thread and reading the latest result
  best-effort.
- MiDaS is a relative-depth network, not metric. Its "scale" is recovered
  by the ratio against the triangulated visual depth. If the visual side
  is also bad (textureless tunnel), MiDaS will fail along with it. That is
  acceptable because in that case we *should* be refusing to publish.

## Walking mode

In walking mode MiDaS is treated as a non-blocking observer: it can pull
the ScaleFuser, but the PDR observer is enough to bootstrap. This is what
gives us the same-session continuity when the user transitions from
walking to scooter and back.
