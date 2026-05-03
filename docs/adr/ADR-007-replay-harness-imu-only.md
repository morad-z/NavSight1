# ADR-007 — Replay harness streams IMU only, with synthetic camera frames

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

Step 7 of the production-readiness plan introduces a replay harness
(`tests/cpp/replay_harness.cpp`) plus a Python scorer
(`tests/cpp/replay_scorer.py`) and a CI workflow
(`.github/workflows/replay.yml`) that replays recorded simulator JSONs
through `VioEngine` and emits a per-sample CSV of EKF state. The scorer
turns that CSV into pass/fail metrics (heading RMSE, drift per meter,
loop-closure gap, optional V-shape yaw span) against thresholds wired
into CI.

The recordings the harness reads from (`tests/sims/regression/*.json`,
sourced from real device sessions) contain:

- per-sample IMU (`ax, ay, az, gx, gy, gz`),
- per-sample recorded velocity / yaw rate (the runtime's published
  state, kept for scoring reference),
- per-sample GPS metadata (`glat, glng, gacc`, frequently null),
- timestamps in nanoseconds.

They do **not** contain raw camera frames. Recording NV21 buffers at 30
Hz for a multi-minute walk would be hundreds of MB per sim and would
make the regression set impractical to keep in git, let alone replay in
GitHub Actions.

## Decision

The replay harness drives the engine with **IMU samples only**, plus a
**synthetic uniform-grey 320×240 NV21 frame** fed every camera tick:

```cpp
constexpr int kSyntheticWidth = 320;
constexpr int kSyntheticHeight = 240;
constexpr uint8_t kSyntheticFill = 64;
std::vector<uint8_t> yuv(kSyntheticWidth * kSyntheticHeight * 3 / 2,
                         kSyntheticFill);
engine.processFrame(yuv.data(), kSyntheticWidth, kSyntheticHeight,
                    s.ts_ns);
```

Each sample's IMU values are delivered via `engine.addAccelData` and
`engine.addGyroData` before the synthetic `processFrame` call, so the
EKF's IMU propagation runs exactly as it would on-device.

What this gives us:

- A deterministic regression channel for the **inertial half** of the
  filter — Madgwick attitude, IMU preintegration, EKF propagation, ZUPT
  arming, scale Kalman state, and the bookkeeping that surrounds them.
- A CI run that completes in under a minute on a free Ubuntu runner,
  with no Android emulator and no camera mock.
- A regression that fails loudly when someone breaks IMU
  preintegration, ZUPT, scale fusion, or the EKF update math —
  precisely the layers Morad owns.

What it deliberately does **not** validate:

- KLT tracking quality.
- MSCKF visual updates (no parallax in a flat frame).
- MiDaS scale observations (depth on a grey frame is degenerate).
- Loop-closure detection.

Those subsystems are exercised by on-device walks and the
`scripts/analyze_sim.py` offline pipeline; the replay harness in CI is
not their watchdog.

## Consequences

**Positive**

- The CI gate is reproducible and bit-exact between commits. Two runs
  of the same sim produce the same CSV byte-for-byte, because the only
  inputs are deterministic IMU streams plus a constant-fill buffer.
- We catch the class of regressions that have actually bitten this
  project: `ZUPT_VEL_THRESHOLD` typos, gyro bias estimator off-by-ones,
  scale-update sign flips, EKF covariance asymmetry. None of those need
  visual data to surface.
- New contributors can run the same harness locally on Linux/macOS
  without the Android toolchain, which lowers the barrier to landing
  fixes.

**Negative / accepted**

- A regression that *only* shows up under real KLT (e.g., a feature
  selection thresholding bug) will pass the CI gate. We accept this
  because:
  1. The visual side is simpler than the inertial side and has fewer
     historical regressions.
  2. The on-device sim runner (Settings → "run simulation") still
     exercises the full pipeline, and Morad runs it before merging
     visual-side changes.
  3. Adding a recorded-camera channel is a future-work item; it would
     belong in a separate ADR if/when we take it on.
- Sims recorded with extreme low-light or featureless walls will
  *almost* match the harness's behavior, which means the harness can't
  tell us "the visual side would have struggled here too" — only "the
  IMU side behaved as expected."

## Why a uniform grey frame and not, say, random noise

Random per-frame noise would cause the visual layer to spuriously match
features and inject high-variance MSCKF updates into the EKF — exactly
the kind of non-determinism the harness exists to avoid. A constant
fill drops the visual layer into "no features tracked" cleanly, which
is the controlled state we want for IMU-only replay.

## Forward path

If we ever add a camera-bytes channel (per-frame PNGs alongside the
JSON, for example), the harness can grow a second mode that reads them.
Until then, the design is: **IMU-only, by design, in CI**.
