---
name: Heading-convention sign fix applied 2026-05-07
description: Two-line EKF fix + extrinsics-metric fix + Step 8 acceptance criterion fix + ADR-017 GPS-course observer drafted
type: project
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
**Changes applied (uncommitted, branch morad):**

1. **`app/src/main/cpp/EKFState.cpp:1052`** — H Jacobian sign fix.
   `h_body = -R_GtoI_ * e_y_world` (was `R_GtoI_ * e_y_world`).
   For body yawed +ψ around world Y, ∂yaw/∂δθ_y = -1, not +1. Without the negation the Kalman gain pushes the filter in the wrong direction.

2. **`app/src/main/cpp/EKFState.cpp:1097`** — `getYaw` sign fix.
   `atan2(-R_aligned[0,2], R_aligned[0,0])` (was `atan2(R_aligned[0,2], ...)`).
   For R_GtoI_ as world→body, R[0,2] = −sin ψ, so naïve atan2 returns −ψ. Negating recovers +ψ matching `imu.getHeading()` convention.

   **Both fixes must land together.** Fixing only one makes things worse; fixing both is necessary AND sufficient to make a consistent measurement produce zero residual and a +δ measurement pull state toward +δ. Hand-verified against tests 1–6 in `tests/cpp/test_ekf_yaw_convention.cpp`.

3. **`app/src/main/cpp/EKFState.cpp:1217`** — `getExtrinsicsAngleDeg` measures drift from initial nominal (`diag(1,-1,-1)`), not from identity. Was always returning 180° because the initial extrinsic IS 180° from identity. Real Step 8b drift was being silently masked.

4. **`docs/VISUAL_PRODUCTION_PLAN.md` Step 8 acceptance criteria** — replaced unsatisfiable "±5 ms of warmup" with hardware-grounded `|TD| ≤ 100 ms`, `|TD − warmup| ≤ 10 ms` after 1000 updates, `σ_TD ≤ 5 ms` post-convergence. Warmup quantises to integer camera-frame periods (~33 ms at 30 fps), so the old criterion was mathematically impossible.

5. **`docs/adr/ADR-017-gps-course-as-bounded-yaw.md`** + spec — **WITHDRAWN same day**. NavSight is VIO-only; GPS must not be fused into the EKF for any purpose. Files left on disk with WITHDRAWN status for the historical record. Do not implement.

**Tests:**
- `tests/cpp/test_ekf_yaw_convention.cpp` exists (written by tdd-guide agent), 6 tests pinning the convention. Cannot build on this Windows host (no OpenCV); must be run via Android NDK or on a Linux machine with OpenCV.
- Existing `tests/cpp/test_ekf_state.cpp` getYaw/updateYaw tests still pass under the new sign (verified by hand: they test `abs(yaw)` or pull-direction-toward-measurement, both preserved).

**Additional instrumentation landed 2026-05-07 (uncommitted):**
- **MiDaS event_summary counters** in `EventCounters.h` + `Tracker::applyDepthScaleConstraint`: 9 counters covering entry / 6 bailout reasons / extreme-rejected / fused / skipped. Resolves the long-standing AI_HANDOFF concern that "MiDaS hasn't fired in any sim" by making it observable in `event_summary`. Reader added to `scripts/analyze_sim.py` (prints `Step 3 MIDAS entries` and `Step 3 MIDAS bailouts` lines).
- **Per-DOF chi² breakdown in `EKFState::updateAbsolutePose` `LC_ABS` log**: adds `m2_R` (rotation-only Mahalanobis from S(0..2,0..2)) and `m2_p` (position-only from S(3..5,3..5)) alongside the existing combined `m2`. Each has χ²(0.999, 3) ≈ 16.27 budget. Lets us see whether rotation residual or position residual dominates a chi² reject.

**What Morad needs to do next:**
1. Build the app with the three EKFState.cpp edits + MiDaS counter wiring.
2. Walk a daytime out-and-back sim (similar to 1778147132092).
3. Run `python scripts/analyze_sim.py <new>.json`, `python scripts/compare_gps_vio.py <new>.json --plot`, `python scripts/heading_audit.py <new>.json`. Need `$env:PYTHONIOENCODING="utf-8"` in PowerShell.
4. Acceptance: `vyaw − gps_course` end-of-walk drops from −112° to under ±15°. VIO/GPS endpoint divergence drops from 104 m to under 20 m. MSCKF huber rate drops from 1.66/update toward ~0.05.
5. If acceptance passes → unblock Step 7 re-test (`loop_closure_corrections_applied` should naturally be > 0 once residuals are small enough to clear chi² gate at 22.5).

**Deferred (do not start until heading fix is validated):**
- ADR-017 GPS-course observer implementation (would compound with heading fix; need clean validation first).
- Per-axis MSCKF residual logging (diagnostic for Section 1 of MSCKF audit).
- UpdaterMSCKF pixel_noise unit fix (likely partially masked by current sign error).
