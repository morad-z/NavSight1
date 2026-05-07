---
name: Heading-drift root cause investigation (sim 1778147132092)
description: 2026-05-07 finding — visual yaw correction has wrong-sign residual; pushes EKF away from truth instead of toward it
type: project
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
**Root cause (high confidence):** The visual yaw-correction path produces residuals with the **wrong sign**, so every keyframe correction pushes the EKF heading further from truth. Three independent agent investigations + direct code reading converge on this.

**Evidence (data-driven, from `scripts/heading_audit.py` analysis):**
- vyaw drifts from gps_course by 30° within **t = 6.5 s** of walk start — not slow accumulation, immediate sign error
- vyaw is *more* wrong than pure-gyro integration alone (vyaw−gps = -112°, gyro−gps ≈ -80°). Visual correction is hurting, not helping.
- Drift correlates with HIGH visual quality (worst-drift window has more inliers than best-drift window) — rules out "no features" hypothesis
- Loop closure heading-gate rejects 317/726 (43.7%) of candidates because vyaw is so wrong that BoW matches can't validate

**Where the convention breaks (code-level):**
- `EKFState.cpp:860` comment: "R_GtoI_ takes world→current-body" — canonical convention is **world→body**
- `EKFState.cpp:1087-1098` `getYaw`: extracts `atan2(R_aligned[0,2], R_aligned[0,0])`. For R_GtoI_ as world→body, this returns **−ψ** (negative of true yaw). For body→world, returns +ψ.
- `EKFState.cpp:138` propagation `R_new = R_GtoI_ * deltaR` is right-multiply (body→world style), inconsistent with world→body name
- `EKFState.cpp:1052` H Jacobian `R_GtoI_ * e_y_world` (Agent 1 hypothesis: should be `.t()`)
- Tracker.cpp:2184 `yaw_meas = kf_heading + visual_delta_y_up` is the physically-correct yaw (Madgwick + visual delta in same +CCW Y-up convention)

**The mismatch:** if `getYaw` returns −ψ but `yaw_meas` is +ψ, then `res = yaw_meas − yaw_pred = 2ψ` when state matches reality. EKF treats this as a huge error and corrects heading away from truth. Compounds every keyframe.

**Why the system "worked" before this sim:** Step 6/7 unit tests use synthetic poses near identity. Sign error is invisible at small angles and during single-direction walks. A long real walk with significant yaw motion exposes it.

**Why this is NOT a fixable-by-sign-flip-at-a-single-line issue:** the propagation, getYaw, and applyMSCKFUpdate all encode different conventions. Fixing only the H Jacobian could leave propagation inconsistent. **The right path is to (1) write a unit test that pins the convention, (2) fix what the test shows is actually wrong, (3) re-walk.**

**Step 7 status under this finding:** Loop closure isn't the bug — it's correctly refusing to apply 70 m / 75° teleportation. Once heading is fixed upstream, Step 7 corrections will be small enough to pass the chi² gate naturally.

**Step 8 status:** Heading RMSE improvement (Criterion 1: PASS) was a partial mitigation from extrinsic refinement, not a real fix. Re-evaluate Step 8 after heading bug is fixed.
