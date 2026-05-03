# ADR-005 — Magnetometer used only for one-shot bootstrap

**Status:** Accepted
**Date:** 2026-05-03
**Owner:** Morad Zubidat (sensor fusion)

## Context

The phone magnetometer is appealing as a yaw observer because it provides
absolute heading. In practice, on the urban environments NavSight runs
in (Haifa city, parking garages, transit corridors), magnetic field
disturbances from vehicles, rebar, elevators, and laptops produce
30–60° errors that decay over many seconds.

Earlier development versions of this filter fused magnetometer into the
yaw update channel. The result was a heading that *looked good* in
benchmarks (stationary phone on a desk) and broke as soon as the device
walked past a parked car — a class of errors that the visual channel
would otherwise have caught.

A standing rule from the project owner is recorded in the team memory:
"Mag is cheating; only use at startup for initial heading, never during
tracking."

## Decision

The magnetometer is consulted **exactly once per session**, at startup,
to seed `Tracker::global_R_` and (transitively) the initial yaw of
`EKFState::initializeFull`. After that, the magnetometer is never read
by the estimator.

Implementation guardrails:

- `Tracker::heading_initialized_` flag latches `true` after the one-shot
  application; subsequent `setMagnetometerHeading` calls are no-ops.
- The JNI binding `setMagnetometerHeading` is preserved but is dead
  code (Kotlin caller commented out) — kept for the legitimate
  reset-on-relaunch path.
- No update method in `EKFState` accepts a magnetic heading.

## Consequences

**Positive**

- Heading errors are now bounded by visual + gyro performance, not by
  the local magnetic environment.
- We do not need a magnetic-disturbance detector — there is nothing for
  it to defend.
- The "GPS jamming + indoor magnetic clutter" failure mode that Haifa
  imposes simultaneously stops being two correlated failures.

**Negative / accepted**

- If the user starts the session inside an elevator, the bootstrap yaw
  will be wrong by however much the elevator perturbs the field. We
  consider this acceptable because a visual rotation correction lands
  within the first few keyframes and pulls the EKF yaw back.
- We give up the option of a magnetometer-based heading reset during a
  session. Long sessions accumulate yaw drift, bounded by gyro bias
  stability and visual yaw updates; this is what the Step 7 regression
  harness exists to monitor.

## Override authority

This rule is honored by all future PRs unless explicitly superseded.
Any change wiring magnetometer into a runtime update path must
reference and supersede this ADR.
