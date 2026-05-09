# ADR-014 — Visual replay harness with recorded camera frames

**Status:** Accepted
**Date:** 2026-05-09
**Owner:** Morad Zubidat (sensor fusion)
**Supersedes (in part):** ADR-007 — replay harness streams IMU only, with synthetic camera frames

## Context

ADR-007 established the IMU-only replay harness. That decision was right
for Steps 1–6 of `PRODUCTION_READINESS_PLAN.md` because the inertial half
was the active risk surface and the visual half (MSCKF/SLAM/loop-closure)
was disabled. Steps 7+8 of `VISUAL_PRODUCTION_PLAN.md` (same-session loop
closure via DBoW2; online TD/extrinsic/rolling-shutter estimation) brought
the visual half back into the EKF — and at that point ADR-007's blind spot
on the visual side became load-bearing. ADR-007's "Forward path" section
explicitly anticipated this:

> If we ever add a camera-bytes channel (per-frame PNGs alongside the
> JSON, for example), the harness can grow a second mode that reads them.

Step 9 of the visual plan is that second mode.

The constraint that drove ADR-007 — recording NV21 at 30 Hz blows out the
git LFS budget for a multi-minute walk — has not changed. What changed is
that we no longer need a multi-minute walk to gate visual regressions; a
30 s, 5 fps clip downsampled to 320×240 is ~5 MB compressed PNG, fits in
git LFS, and runs end-to-end in well under the 3 min CI budget specified
by the Step 9 acceptance criteria.

## Decision

Extend the harness, scorer, and CI workflow to consume **recorded camera
frames** alongside the existing IMU stream, when present:

1. **Frame-source resolution** (`replay_harness.cpp`):
   - New CLI flag `--frames-dir <path>`. If provided and not a directory,
     the harness exits non-zero (CI must not silently degrade).
   - If `--frames-dir` is absent, the harness probes a sibling directory
     named `frames/` next to the simulation JSON.
   - If neither resolves, the harness keeps ADR-007's IMU-only synthetic
     mid-grey 320×240 behaviour, byte-for-byte identical to the previous
     build. **Old fixtures keep passing without change.**

2. **Frame format**:
   - One `<ts_ns>.png` per camera tick. Filenames are exact-match,
     nanosecond timestamps; no interpolation, no resampling.
   - 8-bit single-channel (Y plane). 3-channel and 4-channel inputs are
     converted via `cv::COLOR_BGR{,A}2GRAY` so existing tooling that
     records RGB PNGs still works.
   - Decoded into NV21 (`Y=greyscale, U=V=128`) before `processFrame`.
     The runtime's `cv::cvtColor(..., COLOR_YUV2GRAY_NV21)` recovers
     exactly the input Y, so KLT/ORB see the recorded pixels with no
     extra conversion loss.
   - PNG is chosen over JPEG because lossless decode is the only
     guarantee that gives bit-exact replay across CI runs. The size
     penalty (~2× vs equivalent JPEG) is acceptable inside the 5 MB
     fixture target.

3. **Per-frame CSV columns** (append-only extension to ADR-007's CSV
   schema; old scorers that don't read them keep working):
   - `inlier_count, tracked_count, total_count, mean_flow, pose_flags,
     vision_valid, frame_loaded` — sourced directly from `VisionOutput`.
     `frame_loaded` distinguishes recorded vs synthetic ticks for the
     scorer.

4. **Sidecar event-counters JSON**:
   - At end of run the harness writes
     `<output.csv>.event_counters.json` containing
     `EventCounters::serializeAsJsonString()`. This surfaces
     `loop_closure_corrections_applied`, `msckf_huber_rejected_sum`,
     `slam_promotions_total`, etc. for the scorer without per-row
     instrumentation.

5. **Visual scorer metrics** (`replay_scorer.py`):
   - `inlier_ratio_mean` — mean of `inlier_count / max(tracked_count, 1)`
     across `vision_valid AND frame_loaded` rows.
   - `mean_flow_p50` — median optical-flow magnitude (px) on those rows.
   - `loop_closures_detected` — from sidecar.
   - `msckf_huber_rejected_sum` — from sidecar.
   - `slam_promotions_total` — from sidecar.
   - All four return `None` (n/a) when the fixture is IMU-only, so
     ADR-007 fixtures keep passing.
   - New CLI gates: `--min-inlier-ratio`, `--min-loop-closures`. Each
     defaults to None ("don't gate"); CI invokes them per-fixture.

6. **CI job** (`.github/workflows/replay.yml`):
   - New `replay-visual` job runs after the existing `replay` (inertial)
     job in parallel. 5 min timeout (Step 9 acceptance: < 3 min target).
   - Fixtures live under `tests/sims/regression/visual/`; the inertial
     `replay` job's glob is unchanged at the top level so the two corpora
     don't collide.
   - The job is gated on `tests/sims/regression/visual/*.json` existing.
     **Step 9 ships the wiring; the first fixture lands when Steps 7+8
     accept on a real walk.** A merge that just adds the wiring without
     a fixture is intentionally a no-op in CI — no spurious failures,
     no "we have a job that always passes because it has nothing to
     test."
   - LFS pull is enabled on checkout (no-op when LFS isn't configured,
     fetches PNG payloads when it is).

## Why a sidecar JSON instead of widening the CSV

Per-counter values change at irregular cadences (a single update per
loop closure, etc.) — wedging them into a per-row CSV would blow up to
many sparsely-populated columns. The sidecar is one small JSON object,
trivial to parse, and the natural shape of "what happened during this
run" telemetry. The scorer treats sidecar absence as "fixture is too
old, skip event-level metrics" rather than failing.

## Why we didn't add `keyframe_match_count_p95` and `slam_feature_lifetime_p50` yet

The Step 9 plan lists four visual metrics; we shipped two
(`inlier_ratio_mean`, `loop_closures_detected`) plus three event-level
counters and the median flow. The remaining two require richer
instrumentation:

- `keyframe_match_count_p95` needs a per-keyframe sidecar log (not just
  per-frame VisionOutput).
- `slam_feature_lifetime_p50` needs a feature-promotion timeline log
  (when each feature ID was promoted vs demoted vs dropped).

These are not blockers for the first visual fixture — `inlier_ratio_mean`
+ `loop_closures_detected` already gate the failure modes that ADR-007
couldn't catch. The richer instrumentation lands as a follow-up once
Step 9's CI wiring proves out.

## Consequences

**Positive**

- The visual half of the EKF (MSCKF, SLAM-feature reprojection,
  loop-closure injection) becomes regression-testable in CI.
- ADR-007's IMU-only contract is preserved as a strict subset:
  fixtures without frames behave identically to before the change. No
  flag-day migration.
- Determinism is bit-exact: PNG decode + integer-timestamped frames +
  the existing IMU determinism guarantee that two runs of the same
  fixture produce the same CSV.

**Negative / accepted**

- Visual fixtures cost ~5 MB each in git LFS. The Step 9 plan budgets
  for this; the inertial corpus stays free-tier-cheap.
- A regression that only manifests with intrinsics very different from
  the recorded fixture's (e.g. a per-device calibration bug) won't be
  caught. We accept this — calibration regressions are caught by the
  Step 1 calibration RMS gate, not by replay.
- The `replay-visual` CI job exists today as a no-op (no fixture
  shipped) for a window that may extend past this commit, until Steps
  7+8 accept. We accept this; the wiring needs to be in place for the
  first fixture to land cleanly.

## What this ADR does NOT change

- ADR-007's IMU-only contract for fixtures without frames. The synthetic
  mid-grey 320×240 path is preserved verbatim, including the same fill
  byte (`0x40`) and dimensions.
- The CSV schema is append-only — old fixtures' CSVs are still readable
  by the new scorer (visual columns simply default to 0 / NaN).
- `EventCounters` serialisation format (already specified by
  `EventCounters.h:271-375`).

## Forward path

- First visual fixture lands once a clean sim from a real walk validates
  Steps 7+8 (per `KNOWN_ISSUES.md` P0 #1-3). The clip is pruned to ~30 s,
  downsampled to 5 fps, compressed to PNG, and committed under
  `tests/sims/regression/visual/`.
- The `*_loop` naming convention activates `--min-loop-closures 1` per
  the workflow file; future fixtures targeting the curved-road bug
  (KNOWN_ISSUES P1 #5) can use `*_curve` with a custom drift threshold
  added to the workflow.
- `keyframe_match_count_p95` and `slam_feature_lifetime_p50` land as a
  follow-up ADR when their per-frame instrumentation is in place.
