# Combined Implementation Plan: Post-v19 Sprint

**Date:** 2026-05-09
**Status:** v19 installed + Phase 1 camera overlay shipped. Six steps to "loop 2 overlays loop 1 within 1 m" with full visual feedback on the camera screen.

## Overview

Sequential path from "v19 installed, second loop offset 2.5 m from first" → "loop 2 overlays loop 1 within 1 m, with full visual feedback on the camera screen". 6 steps. Each step builds → installs → quick walk validation → next step. No magic numbers, no patches.

**Currently in flight:** Phase 2/3 camera overlay agent is running in background (planning + implementing Tasks A/B/C/D). Steps 0 and 1 of this plan are coupled to that agent's output.

---

## Step 0 — Install v20 = v19 + Phase 1 camera overlay (5 min)

**Scope:** Already-built APK contains:
- v19 loop-closure variance fix (sum-of-variances using EKF P)
- Phase 1 camera overlay (teal dots over preview)
- v17 frame-convention audit annotations

**Files touched:** None (build is done).

**Action:** `adb install -r app-debug.apk`, force-stop, `logcat -c`.

**Validation:** Open camera FAB → see teal KLT dots tracking scene corners. No regression on trajectory.

**Why first:** It's free. Gives visual debug aid for the next steps. If KLT is sparse on certain surfaces, we see it directly.

---

## Step 1 — Phase 2/3 camera overlay (currently being implemented in background)

**Scope:** The full camera-screen visualization the user actually wants. Background agent owns this; will return with a completed implementation.

**Sub-tasks (from agent's brief):**

- **Task A — fix Phase 1 lag.** Currently the teal dots trail behind scene movement. Fix by extracting `trackedPoints` into its own StateFlow / using `derivedStateOf` / throttling to ~15 Hz.
- **Task B — Phase 3: world-anchored 3D SLAM points (the main ask).** New JNI surfaces `getSlamSnapshot()` + `getCurrentCameraPose()`. Compose-side projection math (`p_cam = R_world_cam.t() · (p_world − t_world_cam)`, then pixel = K · p_cam normalized). Orange dots, distinct from teal KLT dots. **A point pinned to a doorway corner stays pinned to that corner across viewing angles and revisits.**
- **Task C — Phase 2: KLT age coloring.** Green (< 1 s) → yellow (1–3 s) → red (≥ 3 s). Needs JNI extension for per-feature age.
- **Task D — Loop closure flash overlay.** Brief "LOOP CLOSURE" text on the camera screen when `loop_closure_corrections_applied` increments.

**Files (agent will modify):** `native-lib.cpp` (new JNI getters), `NativeBridge.kt`, `VioData.kt` (new fields), `SensorRepository.kt`, `NavSightViewModel.kt`, `CameraUi.kt`, `MapScreenUi.kt`.

**Validation:** Test by walking around — orange dots stay anchored to physical features in the world; green→yellow→red KLT dots show feature lifetime; LOOP CLOSURE flash on revisits.

**Output:** Plan doc at `docs/study/camera_overlay_phase23_plan.md` first, then implementation.

---

## Step 2 — Quick BoW + frame-rate wins (30 min, before deeper fixes)

**From Agent 2's dependency audit. Three small principled changes:**

### 2a. `kBowScoreFloor` 0.005 → 0.002 (`LoopClosureDetector.h`)

- Genuine same-place BoW scores observed in our sim data: 0.003–0.012
- The 0.005 floor is in the bottom half of the legitimate range, rejecting ~50% of real revisits
- Update the comment to cite the measured range

### 2b. Try all 4 `kBowTopN` candidates instead of just `results[0]` (`LoopClosureDetector::tryDetectLoop`)

- DBoW2 returns top 4 candidates per query
- Currently only `results[0]` gets BFMatcher+PnP; 1, 2, 3 are discarded
- Loop until one accepts or all 4 fail. ~3× BFMatcher cost at 1 Hz = negligible
- Adds substantial PnP-eligible candidates per revisit

### 2c. Lock frame rate at 30 Hz (`CameraUi.kt`)

- Currently `setTargetFrameRate(30, 30)` is NOT called → variable rate
- Rolling-shutter math depends on stable frame timing
- One-line addition

**Validation:** Walk → `loop_closure_attempts` counter same range, but `loop_closure_accepts` (PnP-positive count) should rise by ~50%. `chi2_rejected` may also rise because more candidates reach chi² gate — that's fine, Step 3 handles it.

---

## Step 3 — Fix v19 covariance regression (1 hour)

**Scope:** v19's `var_p = var_pnp + (P[12,12] + P[13,13])` decays to ≈ `var_pnp` because MSCKF visual updates collapse the EKF position covariance to ~3 cm even though `global_t_` has 5–15 m of accumulated drift. chi² then rejects almost everything (m² hovers 23-24 vs threshold 22.5 — `loop_closure_corrections_applied = 6` in v19 walk vs 126 in v18).

**Root cause:** v18 `setPosition(global_t_)` sync overrides `p_G` but doesn't touch `P_pp`. Covariance stops reflecting actual uncertainty in `p_G`.

**Fix:** Track explicit "drift since last accepted loop closure":

```
σ²_p_pnp     = LOOP_CLOSURE_PNP_SIGMA_FLOOR_M²            (PnP measurement noise floor)
σ²_p_ekf     = P[12,12] + P[13,13]                         (EKF horizontal cov, often near 0)
σ²_p_drift   = (LOOP_CLOSURE_DRIFT_RATE × path_since_last_lc_m)²
σ²_p_total   = σ²_p_pnp + max(σ²_p_ekf, σ²_p_drift)
```

**Why principled, not magic:**
- `LOOP_CLOSURE_DRIFT_RATE = 0.032 m/m` already exists at `Tracker.h:564`, derived from sim_data
- `path_since_last_lc_m` is NEW state: starts at 0, increments by `|global_t_(t) − global_t_(t−1)|` per frame, **resets to 0** on accepted loop closure (`ok=1` in `consumeLoopClosureMatchIfReady`)
- `max()` covers both "EKF is genuinely uncertain" AND "EKF is overconfident due to MSCKF collapse"

**Files:**
- `Tracker.h` — add `double path_since_last_lc_m_{0.0};`
- `Tracker.cpp` (section 9 end) — increment after `global_t_` updates
- `Tracker.cpp` (`consumeLoopClosureMatchIfReady`) — replace `var_p` formula, reset `path_since_last_lc_m_` on `ok=1`

**Validation:** v21 walk → `loop_closure_corrections_applied` should be 30–100+ over 2 loops. LC_ABS lines should show `var_p` growing smoothly with path length, not pinned at 4.

---

## Step 4 — Fix SLAM feature anchor churn (the "flashing dots" bug)

**Scope:** v20 walk visualised the actual problem — SLAM 3D points (orange dots) ARE being promoted but **flash and disappear within ~1 second**. Live logcat shows 17 features promoted in 200 ms, then `n_slam=0 dropped=48` 0.5–1 s later. **Promotion counter wasn't 0 — the lifetime is just too short.**

**Root cause:** With `MAX_CLONES = 11` and keyframes added at ~5 Hz (anchor IDs incrementing 5015 → 5026 in 2 s in the v20 trace), each SLAM feature's anchor clone falls off the sliding window in ~2 s. When that happens, `EKFState::marginalizeOldestCloneNoLock` calls `removeSlamFeature` (line 567), and the v8 bridge fires `setSlamSlot(-1)` + `dropLifecycle()`. The feature dies even though the underlying physical 3D point hasn't moved an inch.

This is a fundamental design bug in NavSight's SLAM feature lifecycle: the inverse-depth parameterisation `(α, β, ρ)` is anchored at a SPECIFIC clone pose, and once that clone is gone, the feature can't be expressed in the EKF state anymore. OpenVINS solves this by **re-anchoring** features to a still-alive clone before the original anchor falls off.

### Fix 4a — Re-anchor SLAM features before clone marginalization (principled, 1 day)

In `EKFState::marginalizeOldestCloneNoLock`, BEFORE calling `removeSlamFeature` for features anchored at the dropping clone:

1. Pick a new anchor clone — the most recent surviving clone that has observed this feature, OR fallback to the second-oldest clone in the window
2. Transform the inverse-depth state `(α, β, ρ)` from old anchor frame to new anchor frame:
   - Compute the feature's world-frame position from the OLD anchor pose: `p_world = R_anchor_old.T · (1/ρ_old · [α_old, β_old, 1]) + p_anchor_old`
   - Reproject into the NEW anchor frame: `p_new_anchor = R_new · (p_world - p_new_anchor_pos)`
   - New inverse-depth: `α_new = p_new_anchor[0] / p_new_anchor[2]`, `β_new = p_new_anchor[1] / p_new_anchor[2]`, `ρ_new = 1 / p_new_anchor[2]`
3. Update the feature's `anchor_clone_id` and FEJ pose accordingly
4. Skip the `removeSlamFeature` call for re-anchored features

**Why principled, not a patch:** This is the standard OpenVINS feature persistence pattern (see `OPENVINS_ARCHITECTURAL_LESSONS.md`). The math preserves all information about the feature's world position; we just change which clone "owns" the parameterisation. No covariance gets touched directly — the EKF naturally absorbs any small numerical mismatch in the next measurement update.

**Files:**
- `EKFState.cpp` (`marginalizeOldestCloneNoLock`) — add re-anchor logic before remove
- `EKFState.cpp` (`reanchorSlamFeature(int slot, int new_anchor_id)`) — new private helper
- `EKFState.h` — declare helper
- `tests/cpp/test_slam_msckf.cpp` — synthetic test: promote feature, drop its anchor, verify feature persists with same world position

**Expected outcome:** SLAM dots stay pinned to physical 3D locations across multiple seconds. When the user looks at a doorway corner and pans away then back, the same orange dot appears on the same corner.

### Fix 4b — Bump `MAX_CLONES` 11 → 15 (mitigation, 5 min)

Independent improvement that buys more time for re-anchoring AND aligns with Agent 1's scooter-mode recommendation. With 15-clone window at ~5 Hz, average anchor survives ~3 s instead of 2 s — gives the user noticeably longer-lived SLAM dots even before the re-anchoring fix lands.

**Files:**
- `EKFState.h` — `MAX_CLONES` constant 11 → 15
- Any place that hardcodes `pruneWindow(11)` (probably none; method has default arg)

**Expected outcome:** Quick win. SLAM dots flash 50% less frequently. Stacks with Fix 4a for cumulative benefit.

**Validation (both fixes):** Walk and hold position for 5+ seconds. Orange dots should stay pinned without flashing. v21 sim → `slam_lifetime_obs_mean > 100` observations (currently ~10–20).

---

## Step 5 — Pose-graph back-end (2–3 days, the architectural piece)

**Scope:** When DBoW2 detects a revisit and chi² accepts, redistribute the loop closure constraint across **ALL keyframes between the two visits** instead of snapping just `p_G` and `global_t_`'s most recent point.

### Architecture

```
Inputs:
  keyframes  K_0 ... K_N stored along trajectory  (LoopClosureDetector)
  odometry   ξ_i = SE(3) relative pose K_i → K_{i+1}  (already computed at clone-storage)
  loop edge  ξ_loop = SE(3) relative pose from PnP at revisit moment

Variables: pose_i = (x, y, z, yaw) for each keyframe   (4-DOF — gravity observed)

Cost: Σ_i ||log(odom_i^{-1} · ...)||²_Σ_odom + ||log(loop_edge^{-1} · ...)||²_Σ_loop

Solver: Gauss-Newton, 5–10 iterations with damping
Output: updated pose_i for all i. Apply Δpose_i to global_t_ along the path between K_M and K_N.
```

**4-DOF justification:** Roll/pitch are already observed by gravity-alignment (LC_GA). Only x, y, z, yaw need optimization → sparse Hessian, simple solver.

### Implementation

1. **New file `app/src/main/cpp/PoseGraph.cpp/h`** (~300 lines):
   - `addKeyframe(int id, const cv::Mat& pose)`
   - `addOdometryEdge(int from, int to, const cv::Mat& rel_pose, const cv::Mat& info)`
   - `addLoopEdge(int match_id, int now_id, const cv::Mat& rel_pose, const cv::Mat& info)`
   - `optimize(int max_iters)` — Gauss-Newton on 4-DOF state, OpenCV matrix math (no Ceres/g2o needed — confirmed by Agent 2 audit)
   - `getOptimizedPose(int id)` → cv::Mat

2. **Wire-up in `Tracker.cpp`:**
   - Each clone storage adds keyframe + odometry edge
   - On `consumeLoopClosureMatchIfReady` `ok=1` AND first frame of damping ramp → call `pose_graph_.optimize()`
   - Compute Δpose for all keyframes between match_id and now_id, propagate into `global_t_` and stored DBoW2 keyframe poses

**No magic numbers:**
- Σ_odom from EKF clone covariance (per-edge real uncertainty)
- Σ_loop from `var_p_total` defined in Step 3
- Convergence threshold from residual norm < ε where ε = trace(Σ) / 1000 (relative, derived per problem)

**Files:**
- `app/src/main/cpp/PoseGraph.cpp` (new)
- `app/src/main/cpp/PoseGraph.h` (new)
- `app/CMakeLists.txt` — add new file
- `Tracker.cpp` — wire keyframe/edge creation, invoke `optimize()` on accept
- `Tracker.h` — `PoseGraph pose_graph_;` member
- `tests/cpp/test_pose_graph.cpp` (new) — synthetic test (graph with known answer)

**Expected outcome:** Loop 2 overlays loop 1 within 0.5–1 m (down from 2.5 m in v18). This is what ORB-SLAM3 gets on EuRoC and what makes the difference between "30%/loop drift" and "<1%/loop drift".

**Validation:** Visual — walk two loops, second loop overlays first. Quantitative — `loop_closure_gap_m` from `replay_scorer.py` drops substantially. Synthetic — `test_pose_graph.cpp` passes.

---

## Step 6 — Persistent landmark map (3–5 days, the second architectural piece)

**Scope:** Today every triangulated 3D point lives in exactly one place — either the EKF's `slam_features_` (active, currently observed) or the per-keyframe `pts3d_world` array inside `LoopClosureDetector::KeyframeRecord` (dormant, only touched at loop-closure detection). Drift accumulates between loop closures because the dormant points aren't projected back into current frames and matched.

**ORB-SLAM3 pattern: "Tracking the local map".** A single deduplicated `LandmarkMap` of all triangulated 3D points across the session, queried every frame for "which landmarks should be visible right now", projected into the current camera, matched against current ORB features, fed to the EKF as MSCKF-style measurement updates. Every landmark match is a small drift correction — drift is bounded continuously, not just at loop closures.

### Why this is the drift fix you actually want

- **Pose-graph (Step 5)** corrects drift retroactively at a loop closure event. Useful but only fires when a full revisit happens.
- **Persistent landmark map (Step 6)** corrects drift PROACTIVELY at every frame where any past landmark is currently in view. Walk down a corridor, look at familiar wall corners, every match is a position constraint.

**For the 100 m walk-and-back scenario:** with persistent landmarks, the orange dots seen on the doorway during the outbound walk stay registered in the map. When you come back 100 m later and look at the same doorway, those landmarks (still in the map) get matched against current ORB features → pose constrained immediately, before any DBoW loop closure even has a chance to fire.

### How DBoW2 fits with this — clarification

DBoW2 today already stores per-keyframe 3D points, but they're inert until the moment of a revisit. The persistent map is the **promotion of those dormant points to first-class citizens** — they get projected, matched, and used every frame, not just when DBoW says "hey, I think you're back here".

DBoW2 stays unchanged — it remains the LARGE-SCALE revisit detector (works at any distance, even when local-map tracking has dropped all candidates because we've walked too far). Persistent landmark map is the SMALL-SCALE continuous corrector.

### Architecture

```
LandmarkMap class
  std::vector<Landmark> landmarks_   // all triangulated points across session
                                     // deduplicated across keyframes
  cv::flann::KDTreeIndex spatial_idx // for "landmarks near pose" queries

struct Landmark {
  int id;                            // stable across session
  cv::Vec3d p_world;                 // triangulated world position
  cv::Mat reference_descriptor;      // 32-byte ORB descriptor (median across observations)
  std::vector<int> observed_in_kfs;  // keyframes where this landmark was seen
  int64_t last_seen_ts_ns;
  int times_observed;                // for outlier rejection
};

Per-frame pipeline addition:
  1. Get current pose estimate from EKF
  2. Query landmarks within search_radius (e.g. 30 m sphere)
  3. Project each into current camera; reject behind-camera or out-of-frame
  4. Match projected landmarks against current frame's ORB features (descriptor + pixel proximity)
  5. Build 2D-3D correspondences from matches
  6. Feed to EKF as MSCKF-style measurement update (each match = 2-DOF reprojection residual)

Camera overlay tie-in:
  - SlamFeatureOverlay reads from LandmarkMap (not just slam_features_)
  - Orange dots persist forever; gray-out dots not currently observable
  - Walk past a doorway → dot stays in 3D space, just no longer visible
  - Walk back → dot re-enters frame, re-matches, re-anchored
```

### Implementation

1. **New file `app/src/main/cpp/LandmarkMap.cpp/h`** (~400 lines):
   - `addOrMergeLandmark(p_world, descriptor, kf_id)` — dedup by 3D distance + descriptor similarity
   - `getLandmarksInBoundingBox(p_center, radius)` — KD-tree query
   - `projectIntoCamera(p_world, pose, K)` → pixel coord or null if behind camera
   - `matchAgainstFrame(projected_landmarks, current_orb_features)` → 2D-3D correspondences
   - JSON serialise/deserialise (for diagnostic dumps)

2. **Wire-up in `Tracker.cpp`:**
   - On every keyframe storage: `landmark_map_.addOrMergeLandmark(...)` for each triangulated point
   - On every camera frame: query map → project → match against current ORB → build measurement → call EKF MSCKF-update path
   - On every accepted loop closure: dedupe landmarks across the matched keyframes (multi-view constraints from same physical point)

3. **Wire-up in `LoopClosureDetector.cpp`:**
   - Replace the per-keyframe `pts3d_world` storage with reference-by-id into `LandmarkMap`
   - PnP at revisit time: same code, just pulls 3D points from the map by id

4. **Wire-up in camera overlay:**
   - `SlamFeatureOverlay` reads from `getLandmarkSnapshot()` (new JNI) instead of `getSlamSnapshot()`
   - Returns ALL landmarks within 30 m, with a flag for "currently observed" (orange) vs "dormant" (gray-out)

**No magic numbers:**
- Search radius = 30 m derived from typical KLT max depth × 3
- Descriptor similarity threshold = Hamming distance ≤ 50 / 256 (ORB standard)
- 3D dedup distance = 0.5 m (smaller than typical inter-feature spacing in indoor scenes)
- All cited inline with sim-data references where possible

**Files:**
- `app/src/main/cpp/LandmarkMap.cpp` (new)
- `app/src/main/cpp/LandmarkMap.h` (new)
- `app/CMakeLists.txt` — add new file
- `Tracker.cpp` — keyframe storage, per-frame query/match/update
- `LoopClosureDetector.cpp` — refactor `pts3d_world` to `landmark_ids`
- `LoopClosureDetector.h` — `KeyframeRecord` struct change
- `native-lib.cpp` — `getLandmarkSnapshot` JNI replacing/supplementing `getSlamSnapshot`
- `CameraUi.kt` — `SlamFeatureOverlay` reads landmarks not slam features

**Expected outcome:**
- Drift between loop closures drops from ~30 %/loop to <5 %/loop (matches ORB-SLAM3 numbers)
- 100 m walk-and-back: orange dots SAW from outbound walk re-appear on return
- Loop closure (Step 5 pose-graph) becomes the rare large-scale corrector; everyday drift handled by the local map
- The single biggest accuracy unlock for sustained tracking

**Validation:**
- Two-loop walk: second loop overlays first within 0.5 m (combined with Step 5)
- 100 m straight + back walk: orange dots appear on return at same screen positions as outbound (visual confirmation)
- `landmark_map.json` diagnostic dump shows ~500-2000 landmarks per 100 m of walking
- Per-frame `landmark_matches_used` counter > 5 most frames (proves the map is contributing)

---

## Step 7 — Phase 1 validation & handoff to Phase 2

**Scope:** After Steps 0–6 land, run a structured validation pass to declare "Phase 1: accurate-VIO foundations complete" and hand off to Phase 2 (productization).

### 7.1 Validation walks

Three structured walks recorded under controlled conditions:

1. **Single-loop test** (~50 m, 1 loop, return to start)
   - Target: end position within 1 m of origin
   - `loop_closure_corrections_applied` ≥ 5
   - `slam_lifetime_obs_mean` ≥ 100 frames (proves Step 4 re-anchoring works)
   - Camera overlay: orange dots stay pinned, green→yellow→red age progression visible

2. **Two-loop overlay test** (~100 m, 2 loops same path)
   - Target: loop 2 trajectory overlays loop 1 within 0.5 m
   - Pose-graph (Step 5) optimization fires on revisit
   - Visual confirmation: trajectory polylines visually overlap

3. **Out-and-back test** (~100 m straight, U-turn, return)
   - Target: outbound and return segments overlay within 1 m at the start
   - Persistent landmark map (Step 6) re-projects outbound dots on return (visual confirmation)
   - Note: BoW direction-flip is known-hard; persistent map carries the load here

### 7.2 Metrics dashboard

Pull metrics from `replay_scorer.py` for each walk:
- `loop_closure_corrections_applied`
- `loop_closure_gap_m` (drop substantially vs v19 baseline)
- `slam_promotions_total`
- `slam_lifetime_obs_mean`
- `landmark_matches_used_per_frame_mean` (new metric from Step 6)
- `pose_graph_optimizations_run` (new counter from Step 5)
- `msckf_huber_rejected_sum` (should drop further)

### 7.3 Documentation update

- Update `docs/KNOWN_ISSUES.md` — close out P0 #1, #2, #3 entries
- Update `docs/ARCHITECTURE.md` — reflect single-trajectory architecture, persistent landmark map, pose-graph back-end
- Mark `docs/study/post_v19_sprint_plan.md` (this file) as **completed**
- Open `docs/study/phase2_productization_plan.md` (Phase 2)

### 7.4 Phase 1 success criteria

✅ Two-loop overlay error < 1 m
✅ 100 m walk-and-back trajectory closure < 1.5 m
✅ Camera overlay: orange dots persist while observed, no flashing
✅ No regressions vs v9-v15 baselines on any counter
✅ All 6 architectural pieces (single-trajectory, sign fixes, re-anchoring, pose graph, persistent map, camera overlay) shipped and verified

### 7.5 Handoff trigger

When all 7.4 criteria pass: commit + tag as `phase1-complete`, push to morad branch, merge to master. Open the Phase 2 plan as the next sprint entry point.

**Phase 2 plan location:** `docs/study/phase2_productization_plan.md` (created alongside this step).

Phase 2 covers:
- Scooter MountMode framework (PDR off, MountMode-tuned thresholds)
- Velocity from landmark map (the scooter speed source)
- Bad keyframe filter
- MiDaS final validation
- MSCKF Huber rate tuning
- Map persistence across sessions (save/load LandmarkMap)
- OpenCV 4.5.3 → 4.9.x upgrade
- Direction-flip loop closures (SuperPoint/LoFTR exploration)
- Performance optimization
- Testing infrastructure expansion

---

## Build/Validation Cadence

After each step:
1. **Build green** (`./gradlew assembleDebug`)
2. **Quick install** + force-stop + `logcat -c`
3. **30-second test walk** with you to validate the SPECIFIC behavior the step targets
4. If validated → next step. If broken → fix or revert before stacking more

This avoids the "10 fixes piled up, can't tell which broke what" trap from the v9–v15 chain.

---

## Status

| Step | State |
|---|---|
| 0 | ✅ Build complete, awaiting install |
| 1 | 🔄 Camera overlay Phase 2/3 agent running in background |
| 2 | 🟡 Queued (after Step 1) |
| 3 | 🟡 Queued |
| 4 | 🟡 Queued |
| 5 | 🟡 Queued (pose-graph — retroactive drift correction at loop closure) |
| 6 | 🟡 Queued (persistent landmark map — proactive drift correction every frame) |
| 7 | 🟡 Validation + handoff to Phase 2 (after 0–6 done) |

---

## Reference Documents

- `docs/study/architecture_comparison.md` — Agent 1's VIO architecture comparison vs ORB-SLAM3, VINS-Fusion, OpenVINS, etc.
- `docs/study/dependency_audit.md` — Agent 2's DBoW2 + library audit (MiDaS unit bug, kBowScoreFloor analysis, direction-flip diagnosis)
- `docs/study/camera_overlay_plan.md` — Phase 1 plan (already shipped)
- `docs/study/camera_overlay_phase23_plan.md` — Phase 2/3 plan (Step 1, in progress)
- `docs/study/frame_convention_audit.md` — Z-up internal vs Y-up exposed audit (no bug, just naming)
- `docs/study/v6_vio_investigation.md` — earlier investigation that surfaced sign bugs (now fixed)
- `docs/study/v6_android_investigation.md` — earlier investigation that surfaced reset path bugs (now fixed)

---

## What's NOT in this plan

Things explicitly deferred or out of scope:

- **Direction-flip (U-turn) revisit support** — fundamentally hard with current architecture. Mitigation: route design. Future: rotation-invariant descriptors (SuperPoint/LoFTR).
- **Full bundle adjustment** — pose-graph in Step 5 is enough for loop closure; full BA is overkill for our scale.
- **Cloud sync / multi-session SLAM** — single-session only; same-session revisits are scope.
- **Magnetic field map / GPS fallback** — out of scope; we're pure VIO.
