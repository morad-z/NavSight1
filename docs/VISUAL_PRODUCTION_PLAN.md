# NavSight Visual Production Plan

**Status**: draft
**Owner**: Morad
**Scope**: bring the visual half of the VIO pipeline to the same production
bar that the inertial half reached in `PRODUCTION_READINESS_PLAN.md`.
**Companion to**: `PRODUCTION_READINESS_PLAN.md` (inertial). This document
extends, not replaces, that plan.

---

## Why this plan exists

`docs/old docs/VISUAL_ALGORITHMS.md` documents the visual pipeline as it
stood at end of April 2026. The 9-step inertial plan stabilised attitude
(Madgwick), scale (PDR + MiDaS + VI), and the EKF backbone, but it
deliberately did **not** rebuild the visual front-end. The visual layer
today does only three things for the EKF:

1. Keyframe yaw correction every ~15 frames (`Tracker.cpp:1124–1241`).
2. Triangulated 3-D points feeding the MiDaS depth scale observer.
3. Unit-norm `t_vo` feeding the PDR / Hesch-Martinelli scale paths.

Everything else the visual side could do — dense per-frame EKF updates,
loop closure, descriptor-based relocalization, online intrinsic /
extrinsic / time-offset estimation — is either disabled (MSCKF, Mapper,
LoopClosureDetector) or absent. On long sessions, this is what bounds
drift, because Madgwick + IMU preintegration alone cannot stop position
drift from accumulating linearly with time.

The system **does** rely on the camera at runtime — without it the EKF
falls back to dead-reckoning IMU + PDR, which drifts catastrophically
within seconds. The point of this plan is to make the visual layer
contribute as much per-frame information as it physically can.

---

## Guiding principles (carry over from the inertial plan)

1. **No shortcuts.** Every step in this plan is a full implementation.
   No "we'll patch this later," no "good enough for the demo," no stub
   methods with `TODO` markers, no half-wired code paths behind feature
   flags that ship disabled. If a step can't be fully implemented in
   the milestone it lands in, it is **removed from that milestone**,
   not downgraded. This rule applies equally to the algorithms, the
   tests, the calibration assets, the ADRs, and the CI fixtures.
2. **One source of truth per quantity.** Pose lives in EKF. Visual
   features that contribute to pose live as EKF state (SLAM features)
   or are projected out (MSCKF). No parallel pose mirror in `Tracker`,
   no scalar-heading reincarnation, no "Tracker pose" vs "EKF pose"
   ever again.
3. **Covariance is mandatory.** Every visual measurement going to the
   EKF carries a real, derived `R`. If a subsystem cannot produce a
   covariance — pixel-noise model × inlier-count model, or empirical
   residual statistics — it is a heuristic, not an estimator, and
   does not feed the filter.
4. **Replay before re-flash.** Every step lands with at least one new
   sim recording in `tests/sims/regression/` that demonstrates the
   step's acceptance criteria. The Step 9 replay harness extension is
   the channel; per-step recordings are the gate. "Build and walk
   around" or "build and ride around" is debugging, not testing.
5. **Magic numbers are bugs.** Each visual threshold cites its source:
   a chi-squared table entry, a pixel-noise model, a calibration RMS,
   a measured statistic from `tests/sims/`. Numbers that survive must
   be defensible from physics or measurement, not from "this looked
   right in one walk."
6. **No mock cameras in production.** Synthetic frames are a CI
   construct (ADR-007 for inertial; ADR-010 for the visual extension),
   not a runtime mode. Production code paths must never have a "use
   synthetic frame" branch.
7. **Observability is checked, not assumed.** Every step that adds a
   new EKF state or a new measurement type comes with a documented
   observability analysis: which subspace is observed, which is not,
   and why the unobserved subspace stays bounded. SLAM features and
   loop closures both add states whose observability depends on motion
   profile — Step 11 makes this explicit.
8. **Sensor pathologies are a first-class failure mode.** Gyro
   saturation, accelerometer saturation, dropped frames, exposure
   transitions, and dead-IMU events are not edge cases the plan can
   skip. Step 11 owns this category end-to-end.
9. **Mount mode is a runtime variable.** Pedestrian and scooter
   configurations are different physical regimes. Steps that vary
   their behaviour by mount mode must do so explicitly with a single
   `MountMode` source of truth (Step 10.1), not via scattered
   conditionals.
10. **Dead code is deleted, parked code is documented.** Disabled
    visual code (today: Mapper, LoopClosureDetector, UpdaterMSCKF
    callsites) is either re-enabled with this plan or removed. There
    is no third option of "keep around for later" without an explicit
    re-enabling ADR.

---

## Algorithm-by-algorithm review of the current pipeline

This is the audit that drives the plan.

### Preprocessing

| Component | Status | Verdict |
|---|---|---|
| CLAHE (`Tracker.cpp:413–419`) | Active, brightness-gated | Keep. Good cost/benefit. |
| LensCorrector (`LensCorrector.cpp:27–38`) | Zero-distortion passthrough | **Broken in practice.** No intrinsics calibration is wired in. On a phone wide-angle lens, radial distortion of 5–10% at the edges is a meaningful bias source for `findEssentialMat`. → Step 1. |

### Feature detection & tracking

| Component | Status | Verdict |
|---|---|---|
| Grid-based `goodFeaturesToTrack` (Shi-Tomasi) | Active | Good baseline for KLT. Shi-Tomasi is the right detector when the *next* stage is KLT (it picks corners with two strong eigenvalues — what KLT needs). Keep for KLT seeding. |
| 4-level pyramid KLT, 21×21 window | Active | Good. The IMU warm-start (`H = K·δR·K⁻¹`) is the right approach. Adaptive window size based on motion magnitude could squeeze a few more inliers under fast motion. → Step 5 (minor). |
| Forward-backward error check, threshold 4.0 px² | Active | Good. |
| **No descriptor extraction** (ORB/BRIEF/SuperPoint) | Missing | Big gap. Without descriptors, keyframe matching is KLT-only — meaning *only* keyframes the user is currently tracking can be matched. We cannot recover after KLT loss, cannot loop-close across viewpoint or appearance changes. → Step 4. |

### Geometric estimation

| Component | Status | Verdict |
|---|---|---|
| 5-point essential matrix RANSAC | Active | Standard; keep. Could upgrade to GC-RANSAC / MAGSAC++ for better inlier ratios in cluttered scenes (~5–15% more inliers in our regime), at modest CPU cost. → Step 6. |
| `recoverPose` cheirality check | Active | Standard; keep. `R_vo` deliberately discarded in favour of `imu_delta.deltaR`, which is the right call. |
| `triangulatePoints` (DLT) | Active | Functional. DLT is biased for short baselines. Mid-point triangulation with a chi-squared gate is more robust on phone baselines. → Step 6. |
| Reprojection gate at χ²(2, 0.95)=5.991 | Active | Correct. |

### Motion classification

| Component | Status | Verdict |
|---|---|---|
| Translation-degeneracy detector | Active | Good. The mean-flow + `t_norm` joint test is the right idea. |
| Pure-rotation detector (`gyro > 2.0 rad/s`) | Active | Threshold derived from gyro noise should be checked against actual user-motion stats. → Step 5. |
| ZUPT (chi-squared) | Active | Good (inertial side). |

### Visual updates into the EKF

| Component | Status | Verdict |
|---|---|---|
| Keyframe yaw correction (every 15 frames) | Active | Single most valuable visual update we have. Step 2 of the inertial plan made it gravity-aligned. Currently the **only** visual update touching the EKF state. |
| MSCKF sliding-window update (`UpdaterMSCKF`) | **Disabled** (ADR-006) | Was disabled because corrections caused 5–11 m teleportation when EKF state and Tracker pose disagreed. Now that ESKF is the single source of truth, the original failure mode is gone. The right fix is to re-enable with a damped, EKF-consistent injection. → Step 3. |
| **No SLAM features** (long-lived in EKF state) | Missing | Hybrid MSCKF + SLAM (OpenVINS-style) is the standard production pattern. Long-lived features bound drift between keyframes; null-space-projected MSCKF features handle short tracks. → Step 3. |

### Keyframe management

| Component | Status | Verdict |
|---|---|---|
| Rolling 10-keyframe buffer | Active | Reasonable. |
| KLT-based keyframe matching | Active | Works for *recent* keyframes only. Cannot match after viewpoint change > ~30°. → Step 4. |
| **No descriptor index** | Missing | → Step 4. |

### Scale estimation

| Component | Status | Verdict |
|---|---|---|
| PDR / MiDaS / Hesch-Martinelli VI + 1-D `ScaleFuser` | Active | The cleanest part of the visual stack. Keep. Step 8 of the inertial plan validated it. |

### Disabled / parked

| Component | Status | Verdict |
|---|---|---|
| Loop closure (`LoopClosureDetector`, `Mapper`, `PoseGraph`) | Disabled (ADR-006) | Re-enable with a proper EKF-consistent correction path and a damped injection. → Step 6 (windowed BA), Step 7 (loop closure). |
| Time-offset cross-correlation (`TD warmup`) | Active first 60 frames only | Camera-IMU clock drift is not constant. Online estimation is standard in OpenVINS and VINS-Mono. → Step 8. |
| **No IMU-camera extrinsic refinement** | Missing | We assume a perfect static rotation between IMU and camera. → Step 8. |
| **No rolling-shutter compensation** | Missing | Phone cameras are rolling-shutter. Each row is sampled at a different time; high-speed motion induces a per-row pose offset. → Step 8. |

---

## What other production VIO systems do that we don't

Reference implementations and what each contributes:

- **OpenVINS** (Geneva et al., 2020) — Hybrid MSCKF + SLAM features, FEJ
  Jacobians, online TD/extrinsic/intrinsic estimation, rolling-shutter
  model. Closest in design intent to NavSight; references in our code
  comments confirm the team has read the OpenVINS paper.
- **VINS-Mono / VINS-Fusion** (Qin et al., 2018) — Sliding-window
  optimisation with marginalisation, DBoW2 loop closure, 4-DOF global
  pose graph (yaw + 3-D position). Best-in-class long-session drift.
- **ORB-SLAM3** (Campos et al., 2020) — ORB everywhere (track + reloc +
  loop), atlas of submaps, full BA. Higher accuracy at higher CPU cost;
  not all of this fits a phone.
- **DSO / SVO 2.0** — Direct/semi-direct photometric methods. Faster
  than feature-based for some scenes; very sensitive to exposure
  changes and rolling shutter. Probably wrong tool for Haifa daylight
  + indoor lighting transitions.
- **Kimera-VIO** — Modular VIO + 3-D mesh + semantic. Mesh and semantic
  out of scope for navigation.

What of this is realistic on a Snapdragon 695-class phone:

| Technique | CPU budget | Drift bound | NavSight fit |
|---|---|---|---|
| MSCKF re-enable (Step 3) | +3–5% one core | strong | yes, code already exists |
| Long-lived SLAM features in EKF | +2–4% | strong | yes, ~100 LOC addition |
| ORB descriptors at keyframes | +5 ms / KF | moderate (reloc) | yes, OpenCV native |
| Local windowed BA (5 KF) | +20–40 ms / 0.5 s | strong | borderline; budget-gated |
| DBoW2 loop closure | +10 ms / 0.5 s | very strong long-term | yes, after Step 6 |
| Online TD / extrinsics | negligible | small (eliminates a bias) | yes, EKF state extension |
| Rolling-shutter model | +1 ms / frame | small but real | yes |
| Full ORB-SLAM3 atlas | not realistic | n/a | **no — out of scope** |
| DSO direct method | very high | n/a | **no — out of scope** |

This plan implements the rows marked "yes" / "borderline" and explicitly
excludes the "no" rows.

---

## Mount mode: pedestrian vs scooter

NavSight runs in two physical configurations and the visual stack must
serve both. Most VIO literature implicitly targets a single mount; this
plan calls out the differences explicitly so each step can be designed
for both.

| Property | Pedestrian (hand-held) | Scooter (handlebar mount) |
|---|---|---|
| Speed | 0–1.5 m/s | 3–8 m/s |
| Phone attitude | Swings with gait, varied roll/pitch/yaw | Rigidly clamped, near-fixed roll/pitch |
| IMU dynamic range | Step impulses ±2 m/s² in vertical | Sustained translation + 30–100 Hz wheel/motor vibration |
| Camera scene | Varied (looking around, ground, sky, indoor walls) | Forward-facing, pavement-heavy, repetitive textures |
| Pixel displacement / frame at 1 m depth | 5–15 px | 30–100 px |
| Scale observability | PDR steps work | PDR fails — MiDaS blocking observer (ADR-003) |
| Loop scale | 100–500 m | 1–5 km, route-shaped (out-and-back, city blocks) |
| Lighting | User-controlled framing | Direct sun glare, no framing control |
| Stationary periods | Random, short | Long (traffic lights, parking) |

**Implications across the plan:**

- **Steps 1–2 (calibration, R_vo)**: identical in both modes. Distortion
  and relative rotation are physics-level, mount-independent.
- **Step 3 (MSCKF + SLAM)**: SLAM features need different selection
  policy on scooter — the *upper half* of the frame (buildings, signs)
  is where parallax-rich, long-lived features live; the lower half is
  fast-moving repetitive pavement that loses tracks within 5–10 frames.
- **Step 4 (ORB)**: descriptor matching on pavement is a worst case.
  Vocabulary needs to be biased toward urban features. Step 4 is amended
  below to weight feature distribution by mount mode.
- **Step 5 (adaptive front-end)**: today's adaptive logic is gyro-driven.
  Scooter motion is *translation-driven*. KLT window adaptation must
  also use accel/velocity prediction, not just gyro magnitude.
- **Step 6 (local BA)**: works equally well in both modes; it's the
  reconciliation pass.
- **Step 7 (loop closure)**: scooter routes are long enough that a
  single loop-closure correction is insufficient. Need 4-DOF pose graph
  optimisation (yaw + 3-D position) to distribute the correction across
  the trajectory the way VINS-Fusion does for cars. This becomes
  Step 7.5 inside Step 7.
- **Step 8 (TD / extrinsic / RS)**: rolling-shutter compensation matters
  *more* on scooter (faster motion + bumps amplify per-row offsets).
  Stationary periods at traffic lights are gold for extrinsic
  refinement — handle them explicitly.
- **Step 9 (CI fixtures)**: must include scooter recordings, not only
  walking sims.
- **Step 10 (new)**: dedicated scooter robustness work — mount-mode
  detection, vibration filtering, glare handling, pavement rejection,
  pose graph for long routes.

---

## Step 1 — Camera intrinsics calibration

**Goal**: replace the zero-distortion passthrough in `LensCorrector`
with a real per-device intrinsic + distortion calibration so that
`findEssentialMat` operates on geometrically clean rays.

### Why first

This is the single biggest cheap win in the visual stack. With a
typical phone wide-angle (k₁ ≈ −0.2, k₂ ≈ 0.05), undistortion changes
edge-of-frame point positions by 8–15 px. That noise translates directly
into rotation noise from the essential matrix and propagates to every
downstream observer. Calibration removes that bias for free.

### Full implementation plan

1. **In-app calibration screen** — the primary path users go through
   *before* their first VIO session. Goal: same CameraX preview path
   used by VIO so intrinsics match exactly (no offline-vs-runtime ISP
   mismatch).
   - **Entry point**: `SettingsScreen` has a "Calibrate camera" row.
     If no calibration exists, the main UI shows a banner +
     auto-routes there on first launch (tracking is disabled until
     calibration completes — VIO without distortion correction is
     usable but not what we ship to users).
   - **Camera path**: opens the same CameraX preview NavSight VIO uses
     (480p, 4:3, locked AE/AF where supported via Plan Step 11.3
     stubs). Frames flow through the existing native bridge.
   - **Live UX** (Compose):
     - Full-screen camera preview.
     - Overlaid target outline showing the approximate pose the user
       should hit next (cycles through six poses: head-on, tilt-left,
       tilt-right, tilt-up, tilt-down, close).
     - Real-time `findChessboardCornersSB` running off the preview.
       When corners are detected, draw them in green; when not,
       overlay text "Move closer / point at the checkerboard."
     - **Progress chip**: "12 / 30 captures" with a thin progress bar.
     - **Coverage diagnostic**: a 3×3 grid overlay that fills cells
       green as the user has hit poses with the checkerboard centred
       in each cell — encourages varied angle distribution. The flow
       only allows "complete" when all 9 cells have ≥ 2 captures.
     - **Auto-capture gate**: take a frame when (a) corners detected,
       (b) sharpness > threshold (Laplacian variance), (c) pose
       differs from the last 3 captures by ≥ 8° angular OR ≥ 5 cm
       translation, and (d) at least 0.4 s since last capture.
     - **Manual capture button** as fallback for users who want full
       control.
   - **Solve + verdict** (when ≥ 30 captures collected):
     - Run `cv::calibrateCamera` with the rational distortion model
       (k₁, k₂, k₃, p₁, p₂).
     - Show the result on a "done" screen:
       - **Green checkmark + "Calibration complete"** when RMS < 0.5 px.
       - Yellow + "Usable, retry for better" when 0.5–1.0 px.
       - Red + "Calibration failed — redo" when > 1.0 px (don't save).
     - Display: image count, RMS reprojection error in pixels,
       `(fx, fy, cx, cy)` and `(k₁, k₂, k₃, p₁, p₂)` for the curious /
       for thesis screenshots.
     - Buttons: **"Save"** (writes JSON, returns to main),
       **"Redo"** (clears captures and restarts the flow),
       **"Add more captures"** (keeps existing captures, returns to
       capture mode).
   - **Persistence**: save `(fx, fy, cx, cy, k₁..k₃, p₁, p₂,
     image_size, reprojection_rms, image_count, captured_at_iso8601)`
     to `app-internal/camera_calib.json`. Refuse to save if
     RMS > 1.0 px.
   - **Status indicator on the main screen**: small green checkmark in
     the status bar when calibration is loaded; tap-target opens the
     calibration screen for redo. If no calibration exists, the icon
     is grey with a "Tap to calibrate" tooltip.
   - **First-launch onboarding**: a one-screen explainer with: print
     a 9×6 checkerboard (link to OpenCV's `pattern.png`), tape it flat
     to a wall, follow the live prompts, ~2 minutes total.
2. **Bootstrap path** in `MainActivity` / `VioEngine`:
   - On startup, read `app-internal/camera_calib.json`. If present,
     push intrinsics into `Tracker::setCameraIntrinsics(fx, fy, cx, cy)`
     and `LensCorrector::setDistortion(k₁..k₃, p₁, p₂)`.
   - If absent, log a single warning, keep zero-distortion behaviour,
     and surface the calibration banner described above (VIO still
     works, but degraded).
3. **Re-enable** the currently commented-out `LensCorrector::setDistortion`
   and single-set `undistortPoints` paths. The commented-out warning
   "Wrong distortion coefficients are WORSE than no correction" stays
   true — the calibration RMS gate above is what protects against bad
   distortion coefficients reaching the runtime.
4. **EKF intrinsics state** *(Step 8 prerequisite)*: keep intrinsics
   constant for now. Online refinement is Step 8.
5. **CLI / dev mode** *(✅ shipped — `scripts/calibrate_camera.py` +
   `docs/CALIBRATION.md`)*: reads a directory of checkerboard images
   and produces the same JSON. Used for batch reprocessing and CI
   regression work, not the user-facing path.

### Acceptance criteria

- In-app calibration flow produces RMS < 0.5 px on a real phone with
  a printed checkerboard, with all 9 coverage cells filled.
- The "done" screen renders the green checkmark + RMS verdict; user
  taps Save and the JSON is persisted to internal storage.
- On the next app launch, `LensCorrector` reads the JSON, configures
  distortion coefficients, and the main-screen status icon shows the
  green checkmark.
- Replay harness re-run on `tests/sims/regression/*` shows the same or
  better drift metrics than before (calibration shouldn't make things
  worse on existing recordings, since the recordings used the same
  zero-distortion path; the gain is on *future* recordings).
- A single Android session with calibration applied shows median
  inlier count from `findEssentialMat` ≥ 5% higher than baseline
  on a varied walk.
- The "Redo" button on the calibration done-screen clears state and
  re-runs the flow without an app restart.

---

## Step 2 — Stop discarding `R_vo`

**Goal**: the visual rotation `R_vo` from `recoverPose` is currently
discarded in favour of `imu_delta.deltaR`. After Step 1 calibration,
`R_vo` carries useful relative-rotation information that should be
fused, not thrown away.

### Why

The original reason for discarding `R_vo` was that the essential matrix
gave a noisy `R` at short baselines and the IMU was more reliable on
the order of a single frame. That trade-off changes with calibration
(Step 1) and with proper covariance weighting:

- IMU `δR` has uncertainty proportional to `δt × σ_gyro` plus bias
  drift. On a 30 ms frame interval that's tiny.
- Visual `R_vo` has uncertainty roughly `1 / (focal × √N_inliers)` per
  axis (the same formula used for visual yaw in Step 2.4 of the
  inertial plan).

Over a 200 ms keyframe interval, the IMU uncertainty grows linearly
while the visual uncertainty stays constant. Fusing both gives a tighter
estimate than either alone.

### Full implementation plan

1. **Per-frame relative rotation update** in `EKFState`:
   - Add `updateRelativeRotation(R_meas, sigma_axis_sq)` that injects a
     3-DOF rotation observation between two timestamps in the sliding
     window of clones.
   - Variance scales as `(1 / (focal * sqrt(N_inliers)))^2` per axis,
     same model as the keyframe yaw.
2. **Wiring in `Tracker::processFrame`**:
   - After `geometricVerification` returns `(R_vo, t_vo, inliers)`,
     keep `R_vo` and pass it to `ekf_.updateRelativeRotation`.
   - Gate by `!translation_degenerate && inliers >= 12`.
3. **Drop the comment "Yaw is UNOBSERVABLE from monocular camera"**:
   it's true for *absolute* yaw against the world frame (which is
   indeed only observable through gravity + scale), but *relative* yaw
   between two camera poses is fully observable from `R_vo`.
4. **Tests** (`tests/cpp/test_relative_rotation.cpp`, new):
   - Synthetic: rotate a pinhole camera 30° between two keyframes,
     project a fixed point cloud, run `recoverPose`, verify
     `updateRelativeRotation` reduces yaw covariance by a factor matching
     the inlier count.

### Acceptance criteria

- Keyframe yaw RMSE on `baseline_walk_001.json` drops by ≥ 25% in
  replay vs. the IMU-only path.
- No new failure modes on the V-shape regression sims.

---

## Step 3 — Hybrid SLAM + MSCKF features in the EKF

**Split into 3a (ship now) and 3b (deferred)** after a planning audit on
2026-05-03 found that the SLAM-features-in-state half is genuinely
multi-session work (5-DOF inverse-depth Civera Jacobians, dynamic state
extension across the clone block, mid-block Schur marginalisation, FEJ
on a non-trailing block) and shipping it without a paired test pass +
replay-harness regression run reproduces the exact 5–11 m teleportation
failure ADR-006 documents.

**3a delivers the cheap, safe MSCKF half today**: re-enable the
already-correct `UpdaterMSCKF` path, add ADR-006 damping, add Huber
robustification, expose a clean `FeatureManager::getMSCKFCandidates(min_obs=4)`.
This buys real drift reduction on transient features without touching
the EKF state-extension machinery.

**3b is the SLAM-features-in-state half** and is gated on the
prerequisites listed below. It is NOT done by the same agent as 3a —
it requires a dedicated turn with paired test work.

**Goal (combined 3a + 3b)**: the EKF currently maintains a sliding
window of camera-pose clones (Step 4 of the inertial plan), but no
feature-track observations are ever marginalised against them. This is
the biggest single drift reduction available.

### Why hybrid

OpenVINS and several derivatives demonstrate that a **hybrid** scheme
beats pure-MSCKF or pure-SLAM:

- **MSCKF features** — short tracks (3–11 observations), null-space
  projection eliminates the feature DOF before the update. Cheap; good
  for transient features.
- **SLAM features** — long tracks (the most stable 8–16 features),
  carried in EKF state as `(p_F)` 3-vectors. Direct landmark updates
  bound drift between keyframes far better than transient tracks.

ADR-006 documented why MSCKF was disabled: corrections produced 5–11 m
teleportations because the corrector wasn't EKF-consistent and was
overwriting Tracker's pose mirrors. With Step 4 of the inertial plan
having eliminated those mirrors, the original failure mode is gone.

### Step 3a — MSCKF re-enable (ship now, single session)

This is the cheap, safe half. The existing `UpdaterMSCKF.cpp`
implementation is already correct — DLT triangulate, FEJ-clone
Jacobians, SVD null-space projection, chi-squared gate, QR
compression, Joseph update. It's just disabled at the call site.

1. **Re-enable** `UpdaterMSCKF::processLostFeatures` at
   `Tracker.cpp:1272–1285` (currently `// DISABLED:`). Wire the
   already-populated `feature_mgr_.extractLostFeatures` (or its new
   alias `getMSCKFCandidates(min_obs=4)`) into the call.
2. **ADR-006 damping** in `EKFState::applyMSCKFUpdate`: scale the
   `dx(12..14)` (position-error) component by 0.5 on the first frame
   after fire, ramping linearly to 1.0 over 5 frames. New private
   member `int damping_frames_remaining_`. Reset on each MSCKF call.
3. **Huber kernel** in `applyMSCKFUpdate`: replace the implicit binary
   chi² gate with a Huber-weighted kernel using `δ = √χ²(0.95, 2)` ≈
   2.45 per residual. Hard-reject above 3δ ≈ 7.35.
4. **`FeatureManager::getMSCKFCandidates(min_obs=4)`** — thin alias /
   wrapper for `extractLostFeatures(_, 4)` so call sites read clearly.
5. **ADR-008** ("MSCKF re-enabled with damping + Huber") supersedes
   the MSCKF half of ADR-006.

**Acceptance**:
- APK builds clean.
- `tests/sims/regression/baseline_walk_001.json` re-runs and the
  closed-loop return gap stays ≤ 1.93 m (the post-Step-2 value).
- No 5–11 m teleportations in the on-device sim. If teleportations
  reappear, the damping schedule is too aggressive — tighten and
  re-test before shipping.

### Step 3b — SLAM features in EKF state (deferred, multi-session)

This is the harder half. **Do not start without**:

1. A paired test pass: `tests/cpp/test_slam_msckf.cpp` (already
   written by the test agent — currently parked from CMakeLists with
   a TODO marker, lines 71–76 of `tests/cpp/CMakeLists.txt`).
2. A green run of the test on a Linux/CI environment where OpenCV
   actually links (the Windows MSVC/MinGW mismatch on the dev box
   blocks local execution).
3. A replay-harness run on `baseline_walk_001.json` showing zero
   teleportations and ≥ 30% drift-per-meter improvement vs. Step 3a
   baseline.

Implementation outline (when ready):

1. **Feature lifecycle** (`FeatureManager` extension, ~150 LOC):
   - Each feature ID accumulates `(cam_clone_idx, u_undist, v_undist)`
     observations.
   - On loss (track dies, KLT fails, exits frame), classify:
     - 3 ≤ obs ≤ 11 and feature does not survive any keyframe →
       MSCKF feature, dispatch to `UpdaterMSCKF::processLostFeatures`
       (already implemented; un-park it).
     - obs > 11 or feature in ≥ 2 keyframes → SLAM-feature candidate.
   - Maintain a top-K SLAM feature set (K = 12) selected on track
     length × inlier history.
2. **Re-enable `UpdaterMSCKF`**:
   - Wrap the call in `processFrame` (currently commented at
     `Tracker.cpp:1110–1117` per ADR-006) behind a runtime flag
     `EnableMSCKFUpdates` defaulting to **on after Step 1 lands**.
   - Apply update damping: residual is scaled by 0.5 on first frame,
     ramps to 1.0 over 5 frames. This is the "damp the correction
     across N frames" criterion from ADR-006.
   - Joseph-form covariance update (already in `EKFState`).
3. **SLAM feature representation**:
   - Inverse-depth parameterisation `(α, β, ρ)` anchored to the
     first-observing camera clone, per OpenVINS / Civera 2008.
   - Linear initialisation from the existing `triangulatePoints` output
     plus a chi-squared gate on reprojection.
   - State extension: `EKFState::addSlamFeature(p_F, P_FF)`. Covariance
     dimension grows by 3 per feature; total 15 + 3K = 51 with K = 12.
   - Marginalise oldest SLAM feature when adding a 13ᵗʰ.
4. **Per-frame SLAM update**:
   - Each tracked SLAM feature with a current observation contributes a
     2-DOF reprojection residual.
   - Same chi-squared 95% gate as MSCKF.
5. **ADR-006 supersession**:
   - Add ADR-008 ("MSCKF + SLAM hybrid re-enabled") that supersedes
     ADR-006 once Step 3 ships, citing the EKF-consistent injection
     and the damping scheme.

### Acceptance criteria

- No more 5–11 m teleportations on the existing regression sims (Step 7
  of the inertial plan continues to pass).
- Replay drift-per-meter metric on `baseline_walk_001.json` drops by
  ≥ 30% vs. the IMU-only post-Step-2 baseline.
- CPU envelope stays within +6% of one core (measured with the existing
  CPU profiler the team uses).

---

## Step 4 — ORB descriptors at keyframes (relocalization + loop foundation)

**Goal**: when KLT loses tracking (occlusion, motion blur, dark tunnel),
or when the user comes back to a previously-seen viewpoint, descriptor
matching against keyframes restores tracking. KLT alone cannot do this.

### Why ORB

- OpenCV native, no extra dependency, runs at 5–8 ms / frame for ~300
  features at 320×240 on a Snapdragon 695.
- Rotation- and modest-scale-invariant.
- Binary descriptors → Hamming distance → fast on phone.
- Unlocks DBoW2 loop closure in Step 7.

SuperPoint is more accurate but needs a TFLite inference at every
keyframe (~30 ms). Defer to a future ADR if the data shows ORB is the
bottleneck.

### Full implementation plan

1. **Keyframe descriptor extraction** (`FeatureManager::storeKeyframe`
   extension):
   - When a frame is promoted to keyframe, run `cv::ORB::create(300)`
     over a Gaussian-blurred version of the gray frame.
   - Store `(keypoints, descriptors)` alongside the existing keyframe
     fields.
2. **Descriptor-based keyframe matching**
   (`FeatureManager::matchAgainstKeyframeDescriptors`, new):
   - Use `cv::BFMatcher` with Hamming + cross-check.
   - Lowe's ratio test (0.75) for ambiguity rejection.
   - Returns the same `(prev_pts, curr_pts)` shape as the existing
     KLT-based matcher, so downstream code doesn't change.
3. **Hybrid keyframe matching**:
   - Try KLT-based matching first (fast, dense).
   - If KLT returns < 30 matches, fall back to ORB-descriptor matching.
   - Cache descriptor extraction on the *current* frame (computed at
     most once per keyframe attempt).
4. **Relocalization path** (`Tracker.cpp:1098–1107` extension):
   - When KLT loses > 80% of features in a single frame, search ALL
     keyframes for ≥ 50 ORB inliers. On hit, run `solvePnPRansac`
     against the keyframe's stored 3-D points (from the existing
     triangulation pipeline) to recover absolute pose.
   - Inject the recovered pose as a high-variance EKF update (don't
     teleport; let the filter pull there over a few seconds).

### Acceptance criteria

- Synthetic test: take a sim, blank out 30 frames in the middle, verify
  the system re-acquires within 5 frames of resumption with < 1 m
  position step.
- ORB extraction stays under 8 ms per keyframe at 320×240.

---

## Step 5 — Adaptive front-end robustness

**Goal**: the current pipeline uses fixed thresholds (KLT window,
feature count, FB threshold) that are tuned for a "typical" walking
scene. Real Haifa scenarios — high gyro on scooters, low light in
parking garages, motion blur on rapid head turns — break those tunings.

### Full implementation plan

1. **Motion blur detection** (new, `Tracker::measureBlur`):
   - Variance of Laplacian on the gray frame center crop.
   - Below threshold (~80) → frame is blurry.
   - Action: skip MSCKF/SLAM updates this frame; still propagate IMU.
2. **Adaptive KLT window**:
   - Compute expected pixel displacement from IMU gyro:
     `expected_disp ≈ focal × |gyro| × dt`.
   - Window size = `clamp(2 × expected_disp + 11, 21, 41)`.
3. **Adaptive feature count**:
   - In low light (`brightness < 0.12`), drop `MAX_FEATURES` from 200
     to 120 and double `QUALITY_LEVEL` from 0.05 to 0.10. Fewer but
     stronger corners survive.
4. **Stricter pure-rotation detector**:
   - Current threshold `2.0 rad/s` is conservative. Use a dual gate:
     gyro magnitude AND median optical flow direction concentration
     (Rayleigh test). Rejects scooter "looking around" cases the
     gyro-only test misses.
5. **Tests** (`tests/cpp/test_visual_robustness.cpp`):
   - Synthetic blur (Gaussian σ = 5) → blur detector flags > 95%.
   - Synthetic high-gyro (3 rad/s) frame → adaptive window grows.

### Acceptance criteria

- On the "rapid head turn" sub-corpus of `tests/sims/`, KLT track loss
  events reduce by ≥ 40%.
- No regression on baseline walking sims.

---

## Step 6 — Local windowed bundle adjustment

**Goal**: refine the most recent N keyframes' poses and their SLAM
features by minimising reprojection error jointly. This is the
single-keyframe-window analogue of full BA, kept small enough to fit a
phone.

### Why now

- After Step 3, we have SLAM features in the EKF state. BA refines
  *both* them and the keyframe poses simultaneously, which the EKF
  cannot do because EKF updates are sequential.
- After Step 4, we have descriptors so feature associations across
  keyframes are reliable.
- VINS-Mono and ORB-SLAM both demonstrate that a 5–10 keyframe local BA
  cuts drift by another 30–50% on top of EKF.

### Full implementation plan

1. **BA backend choice**: Ceres-Solver. Cross-compiles for Android NDK
   (the original "no GTSAM" rationale in ADR-002 is specifically about
   GTSAM's Boost dependency; Ceres has been ported to Android by ARCore
   internally and is doable).
2. **Window definition**: 5 most recent keyframes (configurable).
3. **Variables**: 5 × 6-DOF poses + ~20 × 3-DOF SLAM features.
4. **Residuals**: reprojection error per (keyframe, feature)
   observation with Huber loss (threshold 1.5 px).
5. **Anchoring**: oldest keyframe in the window has its pose fixed to
   prevent gauge ambiguity.
6. **Trigger**: every time a new keyframe is created, run BA on a
   background thread. The result updates the keyframe poses and SLAM
   features atomically.
7. **EKF reconciliation**: BA updates do *not* directly mutate EKF
   state — the EKF subsequently sees BA-corrected SLAM features through
   its normal observation channel. (This is the "don't overwrite EKF
   from a side channel" lesson from ADR-006.)

### Acceptance criteria

- BA solve completes in < 100 ms on a Snapdragon 695.
- Replay loop-closure-gap metric drops by ≥ 25% on indoor sims.
- No new memory allocations on the camera thread (BA runs on its own
  thread, exchanges with a double-buffered keyframe buffer).

---

## Step 7 — Same-session loop closure (DBoW2)

**Goal**: long sessions (> 5 min) accumulate yaw drift bounded only by
gyro stability + visual yaw updates. Loop closure detects when the user
has returned to a previously-visited viewpoint and applies a global
correction that snaps the recent trajectory back into agreement.

### Why DBoW2

- Bag-of-Words (DBoW2 by Galvez-Lopez et al.) is the standard for
  feature-based loop closure. ORB descriptors from Step 4 are exactly
  what its vocabulary is trained for.
- Compact: a 100k-word vocabulary fits in ~10 MB and runs queries in
  < 5 ms.
- Battle-tested: ORB-SLAM2/3, VINS-Mono, and many academic
  reproductions all use it.

### Full implementation plan

1. **Vocabulary**: ship a pre-trained ORB vocabulary file
   (`assets/ORBvoc.dbow2`, ~10 MB) bundled with the app.
2. **Per-keyframe BoW vector**: at keyframe creation, transform ORB
   descriptors into a BoW vector, store it.
3. **Place query**: every 1 s, query the BoW database for keyframes
   above a similarity threshold AND temporally older than 30 s
   (ignore self-matches in the recent window).
4. **Geometric verification**: candidate keyframes go through:
   - ORB descriptor matching (Step 4).
   - `solvePnPRansac` with ≥ 30 inliers required.
   - Chi-squared gate on relative-pose covariance.
5. **Correction injection**:
   - Compute the pose delta between the recent keyframe and the
     loop-matched keyframe.
   - Inject through the EKF as a relative-pose constraint with covariance
     derived from the PnP inlier count.
   - Damp across 10 frames (per ADR-006 re-enabling criteria).
6. **Re-enable `LoopClosureDetector`** with the new BoW backend; the
   existing class structure stays as a thin wrapper.
7. **ADR-009** ("Same-session loop closure re-enabled") supersedes the
   loop-closure portion of ADR-006.

### Acceptance criteria

- Synthetic figure-8 sim: 100 m walk back to start point closes within
  < 1 m.
- No false-positive loop closures on a straight 100 m walk (the
  similarity threshold + temporal gate must reject same-direction
  matches).

---

## Step 8 — Online time-offset, IMU-camera extrinsics, rolling-shutter

**Goal**: three small-but-real bias sources that today are either
estimated once at startup (TD warmup) or assumed perfect (extrinsics,
global shutter).

### 8a. Online time offset

- Extend the EKF error state by 1 DOF (`δt_d`).
- Per-frame measurement: `t_camera_corrected = t_camera + t_d`.
- Update with each visual observation: the residual gradient w.r.t.
  `t_d` is `−v_feature × dt`.
- This is the OpenVINS recipe (Mingyang Li & Mourikis 2014); ~80 LOC.
- Startup: initialise from current Step 7 (inertial plan) TD warmup
  result so we don't lose the existing initialisation.

### 8b. IMU-camera extrinsics

- Rotation only for now (translation is sub-cm and not observable on a
  phone form factor). 3 DOF added to error state.
- Initialise from Android's `getCameraOrientation` plus the device
  orientation sensor (these usually give the body-camera rotation to
  within a few degrees).
- Refine through the same visual observations the SLAM updates use.

### 8c. Rolling-shutter compensation

- Phone sensors expose row read-out time via Camera2 metadata
  (`SENSOR_INFO_TIMESTAMP_SOURCE` + `SENSOR_ROLLING_SHUTTER_SKEW`).
- Per-feature timestamp = frame timestamp + (row / image_height) × skew.
- Each SLAM/MSCKF observation uses its per-row timestamp when looking
  up the camera pose in the clone window.
- This is a one-line change in the projection model + a metadata pull
  from the Camera2 layer.

### Acceptance criteria

- Replay scorer's heading RMSE drops a further ≥ 10% over the Step 7
  baseline on long sims.
- Online TD estimate stabilises within ±5 ms of the Step 7 TD warmup
  result on the first 60 frames, then continues to track.

---

## Step 9 — Visual coverage in the replay harness + CI

**Goal**: ADR-007 documents that the Step 7 replay harness streams IMU
only with synthetic grey frames. That's the right design for inertial
regression but means none of the visual changes in this plan can be
regression-tested in CI. Step 9 closes that gap.

### Full implementation plan

1. **Recorded-camera channel**:
   - Extend the simulator JSON schema to optionally point at a
     companion `frames/` directory with one PNG per camera tick
     (timestamps as filenames in nanoseconds).
   - Backward-compatible: sims without frames continue to use synthetic
     grey, harness behaviour unchanged.
2. **Frame compression**:
   - 320×240 PNG @ 30 fps for a 60 s sim ≈ 60 × 30 × 25 KB ≈ 45 MB.
   - For CI fixtures, downsample temporally to 5 fps and clip to 30 s
     → ~5 MB per fixture, manageable in git LFS.
3. **Replay harness extension** (`tests/cpp/replay_harness.cpp`):
   - If `--frames-dir <path>` provided, load `<ts>.png` for each camera
     tick and feed it as the YUV buffer (greyscale → NV21 conversion
     is a 4-line copy).
4. **Visual scorer metrics** (extend `replay_scorer.py`):
   - `inlier_ratio_mean` — average essential-matrix inlier ratio.
   - `keyframe_match_count_p95` — 95th percentile keyframe match count.
   - `slam_feature_lifetime_p50` — median SLAM feature track length.
   - `loop_closures_detected` — count of loop closures (Step 7).
5. **CI workflow** (`.github/workflows/replay.yml` extension):
   - New job `replay-visual` running on the same Ubuntu image, fed
     a small recorded-frames fixture.
   - Thresholds set generously initially; tightened as the corpus
     grows.
6. **ADR-010** ("Visual replay harness with recorded frames")
   documents the design and supersedes ADR-007's "IMU-only by design"
   constraint.

### Acceptance criteria

- New CI job passes on a recorded fixture.
- Total CI replay job time stays under 3 minutes on the free runner.

---

## Step 10 — Scooter mode visual hardening

**Goal**: the steps above improve the visual stack for both pedestrian
and scooter mounts, but several scooter-specific failure modes remain.
Step 10 addresses them as a coherent block so scooter mode reaches the
same production bar as walking.

### 10.1. Auto mount-mode detection

Today the user picks pedestrian vs scooter manually (per ADR-003).
Auto-detection makes the scale observer choice (PDR vs MiDaS-blocking)
driven by data, not a UI toggle.

- **Signal source**: 5 s rolling window of `(|accel| variance,
  step-detector confidence, sustained translation magnitude)`.
- **Scooter signature**: low accel-variance (no heel strikes), zero step
  detections, sustained `|v| > 2 m/s`, gyro yaw rate without
  step-correlated pitch oscillation.
- **Pedestrian signature**: high accel-variance from heel strikes,
  step detector firing at 1.5–2.5 Hz, `|v|` modulated by stride.
- **Hysteresis**: 5 s of consistent signature before switching modes.
- **Output**: `MountMode` enum `{PEDESTRIAN, SCOOTER, UNKNOWN}`. UNKNOWN
  during the first 10 s after launch falls back to whatever the user
  manually selected.
- **Wiring**: `Tracker::current_mount_mode_` published to MiDaS
  blocking-observer logic, KLT adaptive parameters (10.2), and SLAM
  feature-selection policy (10.3).

### 10.2. Vibration filtering for IMU and camera

Scooter wheels at 5–8 m/s on tarmac drive a 30–80 Hz vibration
signature into the handlebar. This:

- Inflates raw gyro noise — Madgwick's `β` looks too low for the
  effective noise floor.
- Causes accelerometer aliasing if the IMU is sampling at 100 Hz
  (Nyquist = 50 Hz).
- Excites micro-rotations in the camera mount that show up as
  sub-pixel KLT jitter.

**Implementation**:

1. **IMU low-pass** (in `IMUPreintegrator::addAccelReading` /
   `addGyroReading`):
   - Detect dominant frequency via short FFT every 1 s on the gyro
     stream.
   - When dominant frequency > 25 Hz **and** `MountMode::SCOOTER`,
     apply a 4th-order Butterworth low-pass at 20 Hz.
   - Pedestrian path: no filter (preserves heel-strike content).
2. **Notch filter for resonance**:
   - If the FFT shows a sharp peak (peak-to-mean > 4× in a 5 Hz band),
     apply a notch at that frequency. Common on bargain scooters with
     specific motor harmonics.
3. **Camera-side robustness**:
   - Already addressed by adaptive KLT window (Step 5) and motion-blur
     gating. Vibration-induced sub-pixel jitter is rejected by the
     reprojection chi-squared gate.

### 10.3. Pavement rejection and feature distribution

Forward-facing handlebar camera puts the lower half of the frame on
pavement: high-frequency texture, repetitive, fast-moving. SLAM
features there die in 5–10 frames and degrade descriptor matching.

**Implementation**:

1. **Region weighting** in `FeatureManager::detectGridFeatures`:
   - In `MountMode::SCOOTER`, allocate 70% of `MAX_FEATURES` to the
     upper 60% of the frame, 30% to the lower 40%.
   - Pedestrian: keep current uniform allocation.
2. **Pavement texture detector**:
   - For each lower-half cell, measure dominant gradient direction.
     If > 75% of strong gradients lie within ±20° of vertical (typical
     for forward-motion-elongated asphalt), mark cell as "pavement",
     don't promote its features to SLAM-feature candidates (Step 3.1).
3. **SLAM feature lifetime**: short-lived pavement features still feed
   MSCKF (good for short-baseline residuals) but never reach the
   long-lived EKF state.

### 10.4. Glare and exposure handling

Direct sun on the handlebar mount routinely blows out highlights or
makes the lens glare. Step 5 covers low-light + blur; Step 10.4 adds
overexposure.

**Implementation**:

1. **Saturation detector** (per-frame, `Tracker::measureExposureQuality`):
   - Fraction of pixels with `Y > 245` (effectively saturated) > 0.15
     → "highlight blowout".
   - Fraction with `Y < 10` > 0.40 → "shadow crush".
2. **Action under blowout**:
   - Halve `MAX_FEATURES`, raise `QUALITY_LEVEL` 2×, and skip MSCKF
     updates this frame (residuals are unreliable when most of the
     scene is clipped).
   - Issue Camera2 exposure compensation request `−1.0 EV` to the
     Android camera control pipeline if available.
3. **CLAHE coupling**:
   - In SCOOTER mode, run CLAHE *unconditionally* (override the 0.55
     brightness gate) since outdoor glare confuses the brightness-skip
     heuristic.

### 10.5. 4-DOF pose graph for long routes

Single loop-closure correction (Step 7) snaps the recent trajectory
back. Scooter routes (1–5 km, city-block loops, out-and-back rides)
benefit from distributing the correction across the trajectory, the
way VINS-Fusion's `pose_graph_optimization` does for cars.

- **State variables**: each keyframe contributes a 4-DOF node
  `(yaw, x, y, z)` — roll/pitch are well-observed from gravity and
  fixed at their estimated values.
- **Edges**: relative-pose constraints from sequential VIO (every
  keyframe) + loop closures (Step 7).
- **Solver**: Ceres-Solver (already needed for Step 6).
- **Trigger**: every successful loop closure runs a graph
  optimisation pass on a background thread; result is integrated back
  into the EKF as a sequence of damped pose corrections (per ADR-006
  re-enabling criteria).
- **Scope limit**: pose graph holds the most recent 200 keyframes
  (≈ 100 s of riding) — older keyframes are marginalised. This bounds
  CPU and memory for hour-long sessions.

### 10.6. Stationary-mount extrinsic refinement

A scooter at a red light with the phone clamped is the cleanest
data we'll ever get for IMU-camera extrinsic estimation: zero user
motion, zero camera motion, hours of small inertial perturbations
(driver shifting weight, traffic vibrations) that excite the
extrinsic rotation observability.

- During ZUPT-confirmed stationary periods longer than 30 s in
  `MountMode::SCOOTER`, accumulate a batch of `(IMU_rate, visual_rate)`
  samples and run a least-squares fit for the IMU-camera rotation.
- Result feeds into the Step 8b extrinsic state with high confidence.

### Full implementation plan

Substeps 10.1 through 10.6 above. Order them: 10.1 (mount detection)
→ 10.2 (vibration) and 10.4 (glare) in parallel → 10.3 (pavement) →
10.5 (pose graph, depends on Step 7) → 10.6 (extrinsic, depends on
Steps 8b + 10.1).

### Acceptance criteria

- On a scooter recording (must be added to `tests/sims/regression/`):
  - Drift per meter ≤ 2.0 (pedestrian target is 1.5; scooter is harder).
  - No false MountMode switches on a 5 min mixed walk-then-scooter sim.
  - Loop closure on a 1 km city-block route within 3 m gap.
- On a pedestrian recording:
  - All Step 10 changes are no-ops (mount detector stays in PEDESTRIAN
    mode, no behavioural change vs Steps 1–9 alone).

---

## Step 11 — Sensor health, fault tolerance, and observability

**Goal**: the steps above assume the camera and IMU produce well-formed
data. Real devices don't. Step 11 makes pathological inputs a
first-class case rather than an undefined-behaviour bug source.

### 11.1. IMU saturation handling

Scooter shocks (potholes, kerbs, hard cornering) routinely saturate
phone gyros (±34 rad/s typical) and accelerometers (±78 m/s²).
Pedestrian dropping the phone produces the same. A saturated sample
is not noise — it is a clipped value with no recoverable information.

- **Detection** in `IMUPreintegrator::addAccelReading` /
  `addGyroReading`: per-axis check `|reading| ≥ 0.95 × full_scale`
  (full-scale read from Android `Sensor.getMaximumRange()` at startup).
- **Propagation behaviour**: a saturated sample does *not* preintegrate
  normally. Bias state held; previous non-saturated value clamped at
  full scale is used; process noise on the affected axis inflated 100×.
- **Visual gate**: frames whose enclosing IMU window contains any
  saturated sample skip MSCKF/SLAM updates and keyframe yaw correction.
- **Telemetry**: `Tracker::saturation_event_count_` to crash logger.
  Persistent saturation (> 1 event/s for 10 s) → UI warning ("phone
  mount may be loose").

### 11.2. Frame drop and late-frame handling

Camera2 occasionally drops frames under load. `processFrame` may also
arrive late w.r.t. its timestamp.

- **Late-frame policy**: if the frame timestamp is older than the
  EKF's most-recent IMU integration time, the frame is dropped (don't
  rewind the EKF).
- **Gap policy**: if the frame timestamp is more than 100 ms newer
  than the previous processed frame, a "frame gap" event is logged,
  KLT track continuity is **broken** (force re-detection), and SLAM
  features are not updated this cycle (avoids extrapolating across
  the gap).
- **Catch-up**: never. We don't run pose updates "for the gap" with
  no measurements.

### 11.3. Auto-exposure and auto-white-balance handling

Camera2 changes exposure/AWB between frames, breaking photometric
consistency assumptions in CLAHE and KLT brightness gradients.

- **Lock during VIO**: request `CONTROL_AE_LOCK = true` and
  `CONTROL_AWB_LOCK = true` after the first 2 s (let auto-exposure
  settle, then freeze).
- **Re-exposure trigger**: if Step 10.4's saturation detector fires
  for > 1 s, briefly unlock AE, let it converge for 10 frames (during
  which MSCKF/SLAM are paused), re-lock.
- **Lock loss handling**: if Camera2 reports lock failure, fall back
  to per-frame brightness compensation in CLAHE.

### 11.4. VIO bootstrap quality gate

The plan's other steps assume the EKF is "initialised". In practice,
on the first 2–10 seconds the user moves, the filter has poor scale
and large covariance — publishing those poses to the UI shows a
jittering map dot.

- **Bootstrap criteria** (all must hold before the first user-facing
  pose is published):
  1. ≥ 2 s of preintegrated IMU since session start.
  2. ScaleFuser variance below `1e-2` (scale settled).
  3. At least one successful keyframe yaw correction with ≥ 30
     inliers OR (in scooter mode) at least one MiDaS scale fuse cycle.
  4. EKF position covariance trace below 1.0 m².
- **UI behaviour during bootstrap**: `VioStatusChip` shows
  "initialising"; map shows a pulsing pin at last GPS fix (if any) or
  a generic centred view.
- **Hard timeout**: 30 s. If the gate doesn't pass, show a blocker
  UI ("move the phone in a small loop to initialise") and continue
  trying.

### 11.5. First-Estimate Jacobians for visual updates

The inertial plan establishes FEJ for IMU propagation (ADR-002 covers
the OpenVINS rationale). When SLAM features are added in Step 3, the
Jacobian of the SLAM-feature reprojection w.r.t. the *anchoring*
camera pose must use the **first** linearisation of that pose, not
the latest one. Without this, yaw and global scale become falsely
"observable" through linearisation drift, which is the original
inconsistency MSCKF was designed to fix.

- **Implementation**: each clone in the EKF state stores both its
  current estimate and its first-estimate copy (8 bytes per pose ×
  11 clones = trivial).
- **Jacobian build sites**: `EKFState::buildSlamFeatureJacobian` and
  `EKFState::buildMsckfJacobian` use the first-estimate pose.
- **Reset behaviour**: on full re-initialisation (e.g. after a long
  KLT loss), first-estimate copies are refreshed.

### 11.6. Robust kernels in EKF residuals

Today's EKF gates residuals with chi-squared (reject if above 95%).
A residual at 90% is fully accepted; at 96%, fully rejected. Borderline
points contribute either too much or nothing at all.

- **Replace** the chi-squared gate with a Huber kernel applied to the
  Mahalanobis distance:
  - For `r̃ < δ`: full weight.
  - For `r̃ ≥ δ`: weight scales as `δ / r̃` (sublinear).
- **Threshold**: `δ = √χ²(0.95)` for the relevant DOF (2 for pixel
  residuals, 1 for scalar yaw).
- **Hard reject**: still reject above `r̃ > 3δ` (5σ).
- **Why now**: Step 3 introduces dense visual residuals. A few
  borderline outliers per frame is normal; binary accept/reject
  destabilises convergence.

### 11.7. SLAM feature lifecycle

Step 3 introduces SLAM features but doesn't fully specify when one
gets demoted to MSCKF or dropped entirely.

- **Promotion** (MSCKF candidate → SLAM): track length ≥ 12 obs AND
  observed in ≥ 2 keyframes AND reprojection RMSE < 1.5 px AND there
  is room (current SLAM count < K=12).
- **Demotion** (SLAM → MSCKF candidate): reprojection RMSE > 3 px for
  3 consecutive frames OR feature occluded for 5 consecutive frames.
- **Drop entirely**: track lost for > 1 s OR feature confirmed
  outside any keyframe FOV for > 10 frames.
- **Replacement policy**: when SLAM count drops below 8, promote the
  best MSCKF candidate (longest track + lowest residual variance).
- **Marginalisation**: dropping a SLAM feature uses Schur complement
  to preserve covariance correctness.

### 11.8. Loop-closure false-positive defenses

Beyond Step 7's geometric verification, repetitive cityscapes (Haifa
has long blocks of similar facades) can produce false matches that
pass single-shot PnP.

- **Temporal consistency**: a loop closure is only accepted if at
  least 2 consecutive query frames (within 1 s) match the same
  candidate keyframe with consistent relative pose (poses agree
  within 0.5 m, 5° rotation).
- **Mahalanobis gate on relative pose**: the relative pose from PnP
  must be within 5σ of the EKF's predicted relative pose between the
  two timestamps.
- **One-shot rule**: at most one loop closure correction per 10 s.
  Distributed correction (Step 10.5 pose graph) handles the slow
  global refinement.

### 11.9. Pixel-noise model

The plan currently assumes σ = 1.0 px for all visual residuals.
Reality: noisy depends on KLT residual magnitude, scene texture,
exposure.

- **Online estimation**: maintain a rolling 200-residual window per
  frame. Robust σ = 1.4826 × MAD of residual magnitudes.
- **Floor and ceiling**: clamp to `[0.5, 3.0]` px. Below 0.5 the
  estimate is over-confident; above 3.0 the front-end is failing
  and we shouldn't be running visual updates anyway (let Step 11.2
  / 10.4 take over).
- **Observation Jacobians use this σ²** as the per-axis pixel noise.

### Acceptance criteria

- Synthetic saturation test: clamp gyro to ±2 rad/s for a 5 s window
  during a sim; verify no EKF divergence and saturation events
  reported.
- Frame-drop test: drop every 5th frame; verify EKF stays consistent
  and bootstrap gate doesn't lock up.
- Bootstrap-gate test: empty-scene start; verify UI stays in
  "initialising" until conditions met or 30 s timeout.
- FEJ consistency test: 360° yaw rotation in place; verify yaw
  covariance does not artificially shrink (the FEJ regression).
- Robust kernel test: synthetic outlier injection at 5% rate; verify
  drift stays within 110% of the no-outlier baseline (vs. 200%+
  with hard reject).

---

## Gap audit — what was missing in the first draft

The earlier draft (Steps 1–10) had real holes. They are now folded
into Step 11 and the strengthened principles. For traceability, the
items that were *added or strengthened* during this audit:

| Gap | Location now |
|---|---|
| "No shortcuts" principle was too soft | Principles §1, full inertial-plan language |
| FEJ for visual updates not specified | Step 11.5 |
| Robust kernels (only chi-squared) | Step 11.6 |
| SLAM feature demotion/drop policy | Step 11.7 |
| Loop-closure temporal consistency | Step 11.8 |
| Online pixel noise model | Step 11.9 |
| IMU saturation | Step 11.1 |
| Frame drop / late frame | Step 11.2 |
| AE/AWB lock | Step 11.3 |
| Bootstrap gate before publishing | Step 11.4 |
| Per-step regression sim required | Principles §4 |
| Observability analysis required | Principles §7 |
| Battery / thermal budget | Cross-step concerns (below) |
| Privacy of recorded CI frames | Cross-step concerns (below) |
| Minimum-device floor | Cross-step concerns (below) |
| Online intrinsic refinement | Step 8 (already in scope, made explicit below) |

**Items deliberately deferred** (called out so they're not silent):

- Full ORB-SLAM3 atlas / multi-map.
- DSO / SVO direct photometric methods.
- Semantic dynamic-object masking (covered partially by Step 10.3
  pavement rejection; full segmentation is future work).
- Dense depth fusion (TSDF / KinectFusion).
- Multi-camera setups.
- Persistent map across sessions.
- Custom DBoW2 vocabulary trained on Haifa imagery (we ship a
  pre-trained ORBvoc; per-city tuning is a future step).
- GPU/NPU offload of ORB or KLT (future optimisation; Step 11
  scope ends at correctness).

---

## Cross-step concerns

### Performance budget

| Phase | CPU added | Cumulative |
|---|---|---|
| Baseline (post-inertial plan) | — | ~70% one core |
| Step 1 (calibration runtime) | +0% | 70% |
| Step 2 (relative rotation update) | +1% | 71% |
| Step 3 (MSCKF + SLAM) | +5% | 76% |
| Step 4 (ORB at KFs) | +2% | 78% |
| Step 5 (adaptive front-end) | +1% | 79% |
| Step 6 (local BA) | +3% (background thread) | 82% |
| Step 7 (loop closure) | +2% | 84% |
| Step 8 (online TD/extrinsic/RS) | +1% | 85% |
| Step 10 (scooter hardening: filter + pavement + 4-DOF pose graph background) | +2% | 87% |

The plan deliberately keeps total CPU under 90% of one core on a
Snapdragon 695, leaving headroom for the Compose UI and OS jitter.
Step 10's pose graph runs on a background thread and is bounded by the
200-keyframe sliding window, so its peak cost is amortised over multiple
seconds.

### Memory budget

- Keyframe descriptors: 300 × 32 bytes × 10 keyframes = 96 KB.
- DBoW2 vocabulary: ~10 MB (one-time, in-app asset).
- BA solver scratch: < 2 MB during solves.

Total marginal RAM: < 15 MB. Fits comfortably.

### Risk register

| Risk | Mitigation |
|---|---|
| Ceres-Solver Android cross-compile pain | Step 6 has a fallback: skip BA, ship Steps 1–5, 7–9. BA is the most isolatable step. |
| MSCKF re-introduction reintroduces teleportation | Damping (Step 3.2) + the EKF-only-source-of-truth invariant established in inertial Step 4. |
| ORB descriptor cost on low-end phones | Adaptive count: 200 on Snapdragon 695, 100 on lower. |
| DBoW2 false positives in repetitive Haifa cityscapes (rebar grids, etc.) | Geometric verification gate (Step 7.4) rejects without sufficient PnP inliers. |
| Camera intrinsics drift with temperature | Step 8b (extrinsic refinement) extends naturally to intrinsic refinement if needed; not in scope this round. |
| Mount-mode misdetection (Step 10.1) flips MountMode mid-ride | Hysteresis (5 s consistent signature) + manual override always wins. |
| Vibration filter (10.2) hides genuine motion content | Filter only engages when FFT confirms 25+ Hz dominant frequency AND MountMode is SCOOTER. Pedestrian path untouched. |
| 4-DOF pose graph (10.5) drifts roll/pitch during long stationary periods | Roll/pitch are not optimisation variables — they remain pinned to gravity-derived values from Madgwick. |

### Order of operations

Steps 1, 2, 3, 4 are sequential dependencies. Steps 5, 6, 7, 8 are
mostly parallel after Step 4 lands. Step 9 should land alongside or
just before each of Steps 3, 4, 6, 7 so each new visual capability has
a regression fixture before merge.

### Things deliberately NOT in this plan

- Full ORB-SLAM3 atlas / multi-map merging.
- DSO / SVO direct photometric tracking.
- Semantic segmentation for dynamic-object masking. (Future work; the
  motion classifier already filters moving cars implicitly through
  reprojection chi-squared.)
- Dense depth fusion (TSDF / KinectFusion-style).
- Multi-camera setups.

---

## Decision log additions

This plan, when executed, produces three new ADRs that supersede or
amend earlier ones:

- **ADR-008** — MSCKF + SLAM hybrid re-enabled (supersedes ADR-006's
  MSCKF clause).
- **ADR-009** — Same-session loop closure re-enabled with DBoW2
  (supersedes ADR-006's loop-closure clause).
- **ADR-010** — Visual replay harness with recorded camera frames
  (extends ADR-007).

ADR-006 itself is amended to "Status: Superseded by ADR-008/009 for
the MSCKF and loop-closure paths; the runtime invariant 'no async
mutator behind the camera thread' continues to hold via Step 3's
EKF-only-source-of-truth + damped injection."

---

## Acceptance criteria for the whole plan

A NavSight build with this plan complete should achieve, on the Step 9
recorded-frame fixtures (split by mount mode):

**Pedestrian:**

- **Drift per meter**: ≤ 1.5 (today: ~3.0 on long sims).
- **Same-session loop-closure gap on 100 m figure-8**: ≤ 1 m
  (today: not measurable, no loop closure).
- **Heading RMSE on 5 min walk**: ≤ 2° (today: 5–10°).
- **Re-acquisition time after KLT loss**: ≤ 0.5 s (today: not
  guaranteed to recover).

**Scooter:**

- **Drift per meter**: ≤ 2.0 on a 1 km ride (scooter is harder than
  walking; pavement and speed reduce inlier counts).
- **Loop-closure gap on 1 km city-block route**: ≤ 3 m after pose
  graph optimisation (Step 10.5).
- **Heading RMSE on 10 min ride**: ≤ 3°.
- **No false MountMode switches** on a 5 min mixed walk-then-ride sim.

**Both:**

- **CI replay-visual job**: < 3 min, all metrics within thresholds.
