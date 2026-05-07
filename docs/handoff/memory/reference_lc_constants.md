---
name: Loop-closure tuning constants
description: LOOP_CLOSURE_* constants in Tracker.h with their derivations and chi² gate location
type: reference
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
In `app/src/main/cpp/Tracker.h` (~line 507–538):
- `LOOP_CLOSURE_DAMPING_FRAMES = 10` — frames over which a correction is ramped in
- `LOOP_CLOSURE_PNP_SIGMA_FLOOR_M = 2.0` — minimum position σ (m)
- `LOOP_CLOSURE_DRIFT_RATE = 0.032` — m drift per m walked (3.2 %); position σ = max(floor, drift_rate × total_path_m_)
- `LOOP_CLOSURE_BASE_ROT_SIGMA_RAD = 0.34907` — 20° rotation σ (raised from 3° at commit f1684e4-era for daytime sim)

Chi² gate lives in `EKFState::updateAbsolutePose` at `app/src/main/cpp/EKFState.cpp:1005`:
```
static constexpr double kChi2Threshold = 22.5;   // χ²(0.999, 6 DOF) ≈ 22.458
```
On reject the function bumps `loop_closure_chi2_rejected` and returns false; correction is dropped.

Correction injection path (Tracker.cpp ~3398–3540): `consumeLoopClosureMatchIfReady` pulls a published match, computes target_R_GtoI / target_p_world, calls `ekf_.updateAbsolutePose`, and on success bumps `loop_closure_corrections_applied`. Per accepted match, this runs up to `LOOP_CLOSURE_DAMPING_FRAMES = 10` frames.
