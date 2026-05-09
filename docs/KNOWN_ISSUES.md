# NavSight Known Issues

**Last updated:** 2026-05-09
**Branch:** `morad`
**Latest commit:** `ceb8af3` (Z-up world frame + R_bc convention migration)

This file tracks every known bug, regression, or open architectural concern.
For shipped fixes and history, see `docs/IMPLEMENTATION_STATUS.md` and the
session memory at `~/.claude/projects/.../memory/`.

---

## Status legend

- 🔴 **OPEN** — needs investigation or fix
- 🟡 **APPLIED, PENDING VALIDATION** — code committed, awaiting sim walk
- 🟢 **CLOSED** — verified by sim and committed
- ⚫ **DEFERRED** — known but explicitly out-of-scope right now

---

## P0 — Pending validation (next sim walk)

### 1. 🟡 Step 7 loop-closure acceptance — pending sim
**Symptom:** every prior walk shows `loop_closure_corrections_applied = 0`
with 100% chi² rejection.

**Root cause:** clones stored `R_GtoI` (world→body) under the field name
`R_GtoC` (which downstream readers treat as world→camera). MSCKF projection,
SLAM-anchor inverse-depth, and loop-closure target_R_GtoI all used
mixed-frame data → residuals systematically biased → chi² gate rejects every
attempt.

**Fix applied (commit `ceb8af3`):** Option C — addClone caller composes
`R_bc · R_GtoI` so clones store true world→camera; H_bc Jacobian disabled
in `applyMSCKFFeature` + `slamReprojectionJacobian`; R_bc update at
`applyMSCKFUpdate:707` skipped. R_bc is frozen at the physical fixed-mount
value Kotlin reads from `SENSOR_ORIENTATION` (SensorRepository.kt:285-329).

**Validation expected:**
- `loop_closure_corrections_applied > 0`
- `loop_closure_chi2_rejected` drops sharply
- VIO trajectory at least as good as sim 1778270505061

### 2. 🟡 MSCKF Huber rate ~1.0+/update — pending sim
**Symptom:** roughly half of feature observations rejected by the Huber
kernel each update.

**Root cause:** projections in body-frame (per #1) made predicted pixels
systematically off from observed.

**Fix applied:** same as #1 (Option C bakes R_bc so projections are now in
true camera frame). Plus `UpdaterMSCKF::Options::pixel_noise` default
corrected from 1.0 to 0.002 (was treating residuals as if they were in
800px-equivalent pixel units instead of normalized image coords).

**Validation expected:** Huber rate drops below 0.5/update.

### 3. 🟡 Step 8b extrinsics drift to 90° — pending sim
**Symptom:** `event_summary.extrinsics_rotation_angle_mdeg` reports 90,000
(= 90°) on every walk after ~30 seconds.

**Root cause:** R_bc estimator drifting to absorb the missing R_bc factor
in the body-frame projection (per #1). The 90° was R_bc midway between
its physical fixed-mount value (`Rx(180°)` for screen-up phones) and
identity.

**Fix applied:** R_bc frozen at the SENSOR_ORIENTATION-derived nominal
(Option C disables online estimation). Drift metric should now read 0°.

**Validation expected:** drift = 0° throughout the walk.

---

## P1 — Outstanding bugs

### 4. 🔴 Magnetometer initial heading offset in Haifa
**Symptom:** VIO trajectory matches GPS within ~5m drift but is rotated
~30-50° from GPS truth (Procrustes alignment offset).

**Root cause:** Local magnetic anomalies (buildings, electronics, wartime
jammers nearby) corrupt the initial magnetometer heading at startup.
Once Madgwick is seeded with the wrong yaw, all subsequent integration
inherits the offset.

**Mitigations not yet implemented:**
- Average magnetometer over a longer window before locking in the seed
- Fall back to GPS course-over-ground for heading initialization (but
  ADR-004 forbids GPS in the EKF; would need a separate seed path)
- Visual loop closure correcting absolute heading once a known landmark
  is revisited (relies on Step 7 working)
- Continuous magnetometer fusion with disturbance rejection (revisit
  ADR-005)

**Workaround:** none. Position drift is small enough for short walks; long
walks accumulate the rotational offset.

### 5. 🔴 Curved-road underestimation
**Symptom:** on long walks (>200m) the recorded path under-rotates on
curved sections — the trajectory looks straighter than the actual path.

**User observation (2026-05-09):** "the engine detects sharp turns/90 degree
turns well but it fails when there is a curved road... the camera shouldn't
trust the IMU so much."

**Status:** ROOT CAUSE NOT YET DIAGNOSED on a clean post-Z-up-fix sim.
Pre-Z-up fixes had the heading sign bug masking this; with Steps 1-4 +
Option C in place, need a long-walk sim to see whether curved-road tracking
is genuinely degraded or just appeared so because of the heading bug.

**Next step:** record a long (>500m) walk with curved sections after Option C
validates on house-loops. If curved-road drift persists, hypothesize:
- Visual yaw correction is keyframe-discrete; slow continuous curves
  produce sub-pixel parallax that gets filtered out
- Gyro bias slowly absorbs gentle turns as "stationary drift"
- KLT track lifetime too short across curved sections

### 6. ⚫ MiDaS 100% `few_floor` bailout
**User observation (2026-05-09):** "midas isn't firing because the phone is
pointing forward so that's ok."

**Status:** DEFERRED. Not a code bug — phone pose doesn't satisfy the floor-
detection geometry. The MiDaS units bug at Tracker.cpp:214 was fixed (line
214 now uses metric depth, not VIO baseline-units), but in practice with
forward-pointing phones there are still <8 floor matches per frame.

**If revisited:** could relax the 8-feature minimum or broaden the geometric
floor test to accept lower-pitched features. Not a priority.

---

## P2 — Code quality / tech debt

### 7. ⚫ ADR-014 / ADR-015 / ADR-016 docs missing
**Status:** DEFERRED. These ADR numbers are referenced from code comments
(e.g. Step 8a TD offset cites ADR-014) but no doc files exist.

**Next step:** back-fill the three ADRs from the corresponding code comments
+ commit messages. Low priority — code is self-documenting.

### 8. ⚫ Scale fuser `r_var` floor (Observer C never wins)
**Symptom:** `ScaleFuser` has a `r_var ≥ 0.04` floor on the per-observer
variance, which makes the VI-bootstrap observer (Observer C) effectively
ignored — its variance is always at the floor and Observer A (PDR) wins.

**Status:** DEFERRED. PDR scale is empirically working (path length within
1-2% of GPS on most walks). Fixing this would need a re-tune of all three
observer variances.

### 9. ⚫ ADR-005 magnetometer-revisit
**Status:** DEFERRED. ADR-005 currently disables continuous magnetometer
fusion. Long-term, with proper disturbance modeling, it could provide the
absolute heading reference that #4 needs. Open architectural question.

---

## P3 — Visual Plan progression

### 10. 🔴 Visual Plan Step 9 — replay harness + visual fixtures
**Status:** not started. Requires recorded-camera channel, frame compression,
replay harness extension, visual scorer metrics, CI job. Blocked on
Steps 7+8 acceptance.

### 11. 🔴 Visual Plan Step 10 — scooter mode hardening
**Status:** not started. Auto mount-mode detection, vibration filtering,
pavement rejection, 4-DOF pose graph for long routes. Blocked on Step 9.

### 12. 🔴 Visual Plan Step 11 — sensor health & fault tolerance
**Status:** not started. IMU dropout detection, camera-blocked recovery,
graceful degradation. Blocked on Step 10.

---

## Recently fixed (this push, commit `ceb8af3`)

- ✅ Z-up world frame alignment (gravity, getYaw, H Jacobian)
- ✅ Madgwick yaw initialization from magnetometer azimuth
- ✅ Post-init `setRotation` for EKF R_GtoI
- ✅ H Jacobian world-frame convention
- ✅ MiDaS metric-vs-VIO units bug (Tracker.cpp:214)
- ✅ `total_path_dm` undercount (now includes VO branch)
- ✅ `UpdaterMSCKF::Options::pixel_noise` default (0.002 normalized)
- ✅ R_bc Jacobian sign fix (left-mult update)
- ✅ Clone storage convention migration (Option C)

---

## Backlog priorities

1. **Validate Option C** on the next sim walk (P0 #1-3)
2. **Diagnose curved-road behavior** on a clean long walk (P1 #5)
3. **Plan Step 9** (replay harness CI) — only after #1 validates
4. **Address magnetometer initial heading** — only if #5 doesn't auto-resolve

Lower-priority work (back-fill ADRs, scale fuser tuning, magnetometer ADR
revisit) is opportunistic — pick up between sim walks when nothing higher
is blocked.
