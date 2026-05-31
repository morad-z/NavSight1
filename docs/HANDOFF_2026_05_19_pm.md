# NavSight AI Handoff — 2026-05-19 (PM session)

**Read me first thing tomorrow.** This handoff covers the afternoon/evening session where we attacked the "orange dots don't reappear on revisit" problem. Five fixes shipped (Fix #8 through #12), two known-needed fixes deferred (Phase 2 MiDaS live update, LC soft correction). Walk validation pending tomorrow's long walk.

---

## TL;DR — for the next agent

1. **Five fixes installed on the S21 Ultra** today (Fix #8, #9, #10, #11/11b, #12 Phase 1). All builds green. None of them are validated by a user-visible walk with the full sequence in place — Morad confirmed Fix #8 fixed the ANR, the rest were diagnosed via event_summary counters + logcat but the user-visible "orange dots reappear on revisit" outcome has NOT been confirmed.
2. **Tomorrow's first move:** Morad will do a LONG walk (> 60 s, real distance). That walk's sim + logcat is the validation dataset for Fix #9 through #12 Phase 1. Pull it, run the scripts, look for the falsifier criteria spelled out per-fix below.
3. **Two known-needed pieces are NOT shipped:** (a) LC soft correction (heading + position partial nudge bypassing chi² gate), (b) Phase 2 MiDaS live-update (per-frame depth-only measurement that constrains ρ when parallax fails). Both are designed; only deferred for scope.
4. **Read the project memory at `~/.claude/projects/C--Users-morad-AndroidStudioProjects-NavSight1/memory/`** before touching anything. The chi²-tuning lesson is real — don't tune the gate, find what corrupted state.

---

## Session story (chronological)

The session opened mid-debugging of the orange-dot anchoring problem from the earlier session. Morad's stated user-visible complaint throughout: "the orange dots don't reappear when I walk back to a spot I was anchored at." Five hours of work follows.

### Phase A — ANR fix (Fix #8)

Earlier in the session, the per-frame SLAM live update (Fix #7 from the prior session) was ANR'ing the app: 30 Hz × 12 SLAM features × O(141³) Joseph form ≈ 1080 ms/sec of camera-thread work, blocking the main thread > 5 s.

**Fix #8 (batched live SLAM update):**
- New `EKFState::buildSlamLiveJacobianRow` (const helper) — extracts the pure Jacobian + chi² + early-out math from the per-feature path.
- New `EKFState::applySlamLiveBatch(observations, sigma_px)` — stacks all N per-feature (H_row, r_row) rows into one (2N × dim) matrix and runs a single Joseph-form update. Kalman is linear so this is mathematically equivalent to N sequential updates.
- `EKFState::updateSlamFeatureLive` refactored to be a thin wrapper over the helper (kept for tests + replay harness).
- `Tracker.cpp` per-frame loop now calls `applySlamLiveBatch` once instead of `updateSlamFeatureLive` N times.
- Throttle removed (Fix #7b's `s_fix7_frame_counter % 3 == 0` gate dropped).
- Counter: `slam_live_batch_calls`.

**Status:** **VALIDATED.** Morad confirmed "looks fine" after install. CPU goes from saturating one core to ~9%. The math is identical to before — just structurally batched. **Not the visible-dots fix; this was the prerequisite to make any other live SLAM work feasible.**

### Phase B — Diagnosing "dots don't reappear"

Morad walked, said dots don't return. I started tracing.

**Architectural finding:** Live SLAM features (orange-with-white-ring) are *transient by design*. When a KLT track dies (occlusion, walk-away, viewpoint change), `removeSlamFeature(slot)` deletes the (α, β, ρ) block from the EKF. There is **no path that re-promotes the same feature** on revisit — new KLT corners get new feature_ids. The system's persistent paths are:

1. **Loop closure** (BoW + PnP) — keyframe-to-keyframe match, corrects pose. Doesn't bring features back.
2. **LandmarkMap** (plain orange dots, NO white ring) — world-fixed points, persist across walk. Designed for revisit.

So "dots reappear on revisit" needs LandmarkMap to work. Morad confirmed: NEITHER variant reappears.

### Phase C — LandmarkMap re-anchor (Fix #9)

**Diagnosis:** LandmarkMap entries store `p_world` as a snapshot at `addOrMergeLandmark` time. When LC ACCEPT subsequently corrects EKF clones (via `updateAbsolutePose` propagating through the covariance), the clones get updated, but LandmarkMap's `p_world` does NOT — it stays at the pre-correction value. Projecting through the corrected camera pose against uncorrected landmark positions misaligns every dot.

**Fix #9:**
- New `LandmarkMap::reanchorLandmarksFromClonePoses(clone_pose_lookup)` — walks all landmarks with `has_anchor=true`, asks caller for each clone's current pose, recomputes `p_world = R_world_cam · p_anchor_cam + t_cam_world`.
- Hook in `Tracker::consumeLoopClosureMatchIfReady` at the `k==0` damp frame: invokes re-anchor with a lambda that wraps `ekf_.getClonePose` (transposing R_GtoC→R_world_cam at the boundary).
- Counter: `landmarks_reanchored_total`. Logcat: `LC_REANCHOR: n_reanchored=N`.

**Status:** Built, installed. **NEVER OBSERVED FIRING** on any subsequent walk — because LC ACCEPT never fired during the recording windows (see Phase F). Falsifier criterion: post-fix walk with same-spot revisit must show `landmarks_reanchored_total > 0` paired with LC ACCEPT, AND orange landmark dots reappear within ~10 px of their feature on the wall after the revisit keyframe.

### Phase D — Diagnosing the deeper LC failure

A walk's sim showed LC ACCEPTs in logcat with absurd targets:

```
target_p = [-56.839, -31.413, 12.359]   ← 65 METERS away
p_G      = [-1.320,  -0.402,  0.245]    ← user is 1.4 m from origin
r_p mag  = 64.7 m
r_R mag  = 3.06 rad ≈ 175°              ← π sign-flip pattern
m² = 1123 (vs threshold 22.5) → REJECTED
```

LC's PnP returned `target_p` 65 m off. With 34 PnP inliers, BoW 0.017 — all green flags, but geometry was poisoned. Diagnosis matches a recurring NavSight bug pattern (project memory `project_slam_dot_anchoring_2026_05_12`): SLAM feature inverse-depth ρ converging to zero → `p_world = (α/ρ, β/ρ, 1/ρ)` → infinity → polluting `pts3d_world` fed to PnP → garbage `target_p`.

**Root cause of ρ corruption:** Pure-axial motion (Morad's walk pattern: phone facing wall, walk backward/forward) gives ZERO depth observability. Both `ray_anchor = p_world - p_anchor` and `ray_now = p_world - p_now` point in the same world direction → SLAM (α, β, ρ) block is unobservable → Fix #7/#8 live updates fire on noise → ρ drifts toward 0 or infinity.

### Phase E — Parallax-baseline gate (Fix #10)

**Fix #10:**
- In `EKFState::buildSlamLiveJacobianRow`, compute parallax angle between `ray_anchor` and `ray_now`. If `cos > kSlamMinParallaxCos = 0.99995` (angle < 0.57°), skip the update silently.
- Threshold derivation: matches OpenVINS UpdaterSLAM `min_parallax_ratio = 0.01` (Geneva et al. 2020 §III.D).
- Counter: `slam_live_skipped_no_parallax`.

**Status:** **VALIDATED partially.** Post-Fix-#10 walk showed:
- `slam_live_skipped_no_parallax = 2183`
- `slam_live_updates_fired = 5` (down from 18 in prior walk)
- ρ no longer being injected with noise during axial motion ✓

But still no user-visible dot reappearance — because LC still didn't fire (Phase F) and Fix #9 still hasn't triggered.

### Phase F — Why LC isn't firing on Morad's walks

All of Morad's test walks: 24–31 seconds. The LC worker thread has a **30-second temporal exclusion** (`LOOP_CLOSURE_TEMPORAL_EXCL_NS`, `LoopClosureDetector.cpp:421`). At most 1 second of LC eligibility at the end of a 31-second walk.

Morad explicitly confirmed: **"LC works on longer walks, tomorrow I will walk a long walk."** So this is a known-pending validation, not a blocker.

### Phase G — Per-frame landmark pixel tracking (Fix #11 / #11b)

**Side-bug discovered:** `landmarks_rendered_anchor_total = 65781` (the count of "observed" landmarks rendered with bright orange) but Morad reported "didn't notice the orange dots." That's 73 observed-dots per frame being painted but invisible to the eye.

**Diagnosis:** The matched-pixel `(obs_u, obs_v)` is set ONCE per keyframe (~1 Hz) and never updated between keyframes. So the dot stays at the keyframe-time pixel while the image moves past it for 28-29 frames — it looks "stuck in screen space."

**Fix #11 (initial attempt):**
- Captured `kf_back.feature_ids[t_idx]` at match time, plumbed feature_id ↔ landmark_id parallel array.
- Per-frame, looked up KLT pixel via the feature_id link.

**Validated FAILED:** Post-Fix-#11 walk: `landmarks_pixel_refreshed_total = 110` over 30k+ observed-dot events. Root cause: most `kf_back.feature_ids[t_idx]` are -1 because ORB keypoints rarely align spatially with KLT tracks at descriptor-storage time (per `KeyframeDescriptors.h:11`).

**Fix #11b (replacement):**
- Per-frame, for each landmark in `last_observed_landmark_pixels_`, search KLT features in `next_good_buf_` within 5 px radius; adopt the nearest's pixel.
- O(N_landmarks × N_KLT) per frame ≈ 5600 ops. Cheap.
- Radius derived from KLT inter-frame motion (~2-5 px walking at 30 Hz) + 1σ ORB localization.

**Status:** Built, installed, **NOT YET WALK-VALIDATED for visual continuity**. Falsifier: post-fix walk should show `landmarks_pixel_refreshed_total ≈ landmarks_rendered_anchor_total` AND visually the orange dots track features as the camera moves instead of being stuck.

### Phase H — MiDaS fusion at SLAM promotion (Fix #12 Phase 1)

Morad asked "can we use MiDaS for depth to aid it?" and "fuse MiDaS for the depth and for the live slam points and whoever might need depth, also previous agents told me MiDaS can help when the phone is mounted on a scooter."

**Background:** NavSight already runs MiDaS via `DepthEstimator.kt` → `Tracker::setDepthMap` → `applyDepthScaleConstraint` (Phase 2 Step 4.2.1 affine fit, shipped 2026-05-17). The affine fit produces `(s, t)` such that `inv_metric_depth = s · disparity + t`. Today's session **caches** that fit and exposes per-pixel sampling.

**Fix #12 Phase 1:**
- `Tracker::sampleMidasMetricDepth(u, v, &depth_m_out)` — public helper. Reads cached `(s, t)` under a new mutex, bilinear-interps disparity at pixel `(u, v)` on `depth_map_`, applies `metric = 1/(s·disp + t)`, returns true if depth is in [0.3, 30 m] band.
- `applyDepthScaleConstraint` caches `(s, t)` into new Tracker members (`midas_affine_s_`, `midas_affine_t_`, `midas_affine_valid_`) after the ≥50% inlier acceptance bar passes.
- SLAM promotion site (`Tracker.cpp:3214+`) now sanity-checks every promoted feature: samples MiDaS at the anchor observation pixel, compares to the triangulated depth in anchor camera frame. If they disagree by > 2× ratio, **replaces** `p_world` with the MiDaS-derived position.
- Math: `p_anchor_cam = (obs.x · z_midas, obs.y · z_midas, z_midas)`, then `p_world = R_anchor.t() · p_anchor_cam + p_anchor`.
- Counters: `midas_depth_samples` (per-pixel samples), `slam_promotions_seeded_with_midas` (count of replacements). Log: `SLAM_PROMOTE_MIDAS_SEED: fid=N z_tri=X z_midas=Y ratio=Z ...`.

**Status:** Built, installed, **NOT YET VALIDATED** with a walk. Falsifier:
- Pure-axial walk should show `slam_promotions_seeded_with_midas > 0` (MiDaS replacing bad triangulations).
- Healthy walking should show `slam_promotions_seeded_with_midas ≈ 0` (triangulation and MiDaS agree).
- Long walk LC should now produce sane `target_p` (within 5 m of `p_G`, not 65 m).

---

## What's NOT shipped (and how to ship it)

### NOT SHIPPED #1 — Phase 2 of MiDaS fusion (live SLAM depth measurement)

**Why it matters:** Phase 1 only seeds depth at feature CREATION. Once promoted, a SLAM feature's ρ evolves only via the existing live-update path (which Fix #10 skips during axial motion). So during a long axial run, ρ stays at its initial value — fine if initial value was correct (Fix #12 Phase 1 ensures this), but doesn't gain any further depth refinement.

**Design:**
- New `EKFState::updateSlamFeatureMidasDepth(slot, depth_metric, sigma_m)` method.
- Builds a 1×state_dim H matrix where the only non-zero columns are the SLAM (α, β, ρ) block + the IMU pose block.
- Predicted depth: `z_pred = (R_bc · R_GtoI · (p_world − p_G))[2]` where `p_world = R_anchor.t() · (α/ρ, β/ρ, 1/ρ) + p_anchor`.
- Residual: `r = z_midas − z_pred`.
- Apply Kalman update via `applyMSCKFUpdate`.
- σ_m derived from MiDaS affine-fit inlier residual std-dev (cite the value from `midas_affine_fit_inlier_ratio_milli`).

**Wire-up:**
- In `Tracker.cpp` SLAM live batch loop, after `applySlamLiveBatch` returns, loop SLAM slots:
  - For each slot where the parallax gate skipped (we'd need to know — could either re-build the check, or have `applySlamLiveBatch` return which slots were skipped),
  - Sample MiDaS depth at the observation pixel,
  - Call `updateSlamFeatureMidasDepth`.

**Cost:** ~150 LOC new EKF method. New counter `slam_live_midas_depth_fired`.

**Risk:** Bad MiDaS depths inject noise into healthy features. Mitigations:
- Per-row chi² gate (1-DOF threshold = 3.84 at 95%).
- Inlier-confidence weighting: σ_m = max(0.5 m, MiDaS_residual_std).
- Only fire when MiDaS affine fit's recent inlier ratio > 70 % (counter `midas_affine_fit_inlier_ratio_milli > 700`).

**Falsifier:** A walk that previously caused ρ-drift (axial motion) should now show ρ converging to MiDaS depth instead of staying frozen. Logcat per-slot ρ trajectory should be bounded.

### NOT SHIPPED #2 — LC soft correction (heading + position nudge bypassing chi²)

**Why it matters:** Morad explicitly asked for this. His framing: "not hard-reset, but at least correct the heading maybe? adjust the position maybe because now I don't think it does much." Even after Fix #10 removes ρ corruption, the chi² gate may still reject LC corrections if the EKF state was already drifted before Fix #10 was active. A soft nudge would converge state to the LC target over multiple LC fires without ever exceeding the chi² threshold per-step.

**Design (3 parts):**

**Part A — lower temporal exclusion 30 s → 10 s:**
- Currently `LoopClosureDetector::queryLoopClosureCandidate` uses a 30 s exclusion to prevent self-matching of recent keyframes.
- 10 s is still well over the typical KF cadence (~1 s) so it doesn't self-match consecutive frames.
- Lets revisit LC fire on short test walks (Morad's 30 s walks).
- Risk: minor false-positive rate increase. Mitigated by adaptive minScore + PnP inlier gate.
- Site: search for `kTemporalExclusionNs` or the constant in `LoopClosureDetector.h`.

**Part B — soft heading nudge:**
- In `consumeLoopClosureMatchIfReady` at the `k==0` damp frame (after `updateAbsolutePose` runs), when PnP inliers ≥ 25:
  - Extract Madgwick yaw from `target_R_GtoI` (use `EKFState::getYaw(roll, pitch)`).
  - Extract current Madgwick yaw the same way.
  - `delta_yaw = target_yaw - current_yaw`, wrapped to [-π, π].
  - Apply 10 % of delta via `imu.nudgeMadgwickYawAroundWorldZ(0.1 * delta_yaw)`.
- The existing `LC_MADG_NUDGE` path already exists for the EKF-to-Madgwick sync; this re-uses it for hard heading correction.
- Multiple LCs over time → convergence.

**Part C — soft position nudge:**
- Same gate (PnP inliers ≥ 25, k==0).
- `delta_p = target_p_world - p_G_world`.
- Apply 10 % of delta directly via the existing global_t_ + EKF state setter. Search for `setPosition` or just mutate `p_G_` under the mutex.
- Counter: `lc_soft_position_nudges`, `lc_soft_heading_nudges`.

**Acceptance criteria:**
- Apply only when PnP inliers ≥ 25 (strong match required).
- 10 % blend factor.
- Bypass chi² gate (this is the whole point).
- Don't touch covariance (small nudge, low risk of false confidence).

**Falsifier:** On a long walk with revisit, LC ACCEPTs fire AND `lc_soft_position_nudges > 0` AND over multiple revisit LCs the EKF p_G converges toward true position (verifiable from logcat LC_ABS lines showing decreasing `|r_p|` over time).

### NOT SHIPPED #3 — Trajectory monotonic-motion bug

Across THREE separate user walks today (fix9_revisit, fix9_revisit_v2, fix11_revisit), the recorded trajectory showed monotonic motion despite Morad confirming he walked back-and-forth. Symptoms:
- Trajectory peaks at maximum distance from start, never returns
- Last N points show IDENTICAL position (recording froze)
- Walks: 24-31 s duration, all peaked-then-froze

This is likely a separate VIO bug or a recorder issue. Two hypotheses:
1. **Tracker `global_t_` freezes** when motion goes below some threshold. The is_static gate or velocity clamp might be locking position updates during walk-back when speed crosses zero.
2. **`SimulationFrameRecorder` stops appending** when some condition fails.

**Not investigated.** Worth a 30-minute dive before tomorrow's long walk. If `global_t_` is freezing, the user-facing trajectory display will misrepresent any walk-back motion.

---

## State of the codebase as of end-of-session

### Files modified

```
app/src/main/cpp/EKFState.h              (Fix #8 helper + batch declarations)
app/src/main/cpp/EKFState.cpp            (Fix #8 helper + batch impl, Fix #10 parallax gate)
app/src/main/cpp/Tracker.h               (Fix #11 parallel array + Fix #12 MiDaS helper decl)
app/src/main/cpp/Tracker.cpp             (Fix #8 batch call, Fix #9 re-anchor hook,
                                          Fix #11/11b proximity refresh, Fix #12 MiDaS sample
                                          + promotion seeding, applyDepthScaleConstraint
                                          caches affine fit)
app/src/main/cpp/LandmarkMap.h           (Fix #9 reanchorLandmarksFromClonePoses decl)
app/src/main/cpp/LandmarkMap.cpp         (Fix #9 reanchor impl)
app/src/main/cpp/EventCounters.h         (8 new counters: slam_live_batch_calls,
                                          slam_live_skipped_no_parallax,
                                          landmarks_reanchored_total,
                                          landmarks_pixel_refreshed_total,
                                          midas_depth_samples,
                                          slam_promotions_seeded_with_midas)
scripts/diagnose_revisit_dots.py         (Fix #9 falsifier check)
scripts/hunt_rgtoi_drift.py              (Rotation drift hunt)
```

### APK on device

`R5CR70S3NNB` (S21 Ultra) has Fix #8+#9+#10+#11b+#12 Phase 1 installed.

### Git state

ALL uncommitted on `morad` branch. Per memory `feedback_explicit_commit_only`, never auto-commit. Wait for explicit "commit" from Morad before staging anything.

---

## Tomorrow's first move (concrete checklist)

1. **Morad walks LONG (> 90 s)** with back-and-forth segments. Records the sim from before the walk starts.
2. **Pull sim + logcat:**
   ```powershell
   $ADB = "C:\Users\morad\AppData\Local\Android\Sdk\platform-tools\adb.exe"
   $DEST = "tests\sims\regression\visual"
   $LATEST = (& $ADB shell "ls -t /sdcard/Android/data/com.example.navsight1/files/simulation_data_*.json | head -1").Trim()
   & $ADB pull $LATEST "$DEST\long_walk_2026_05_20.json"
   & $ADB logcat -d 2>&1 | Out-File -Encoding utf8 "$DEST\long_walk_2026_05_20.logcat.txt"
   ```
3. **Diagnostic battery** (run all three):
   ```bash
   python scripts/diagnose_revisit_dots.py tests/sims/regression/visual/long_walk_2026_05_20.json tests/sims/regression/visual/long_walk_2026_05_20.logcat.txt
   python scripts/hunt_rgtoi_drift.py tests/sims/regression/visual/long_walk_2026_05_20.logcat.txt
   python scripts/analyze_chi2_rejections.py tests/sims/regression/visual/long_walk_2026_05_20.logcat.txt
   ```
4. **Counter check** in `event_summary`:
   - `slam_live_batch_calls` > 0 — Fix #8 active
   - `slam_live_skipped_no_parallax` >> `slam_live_updates_fired` during axial portions — Fix #10 working
   - `landmarks_pixel_refreshed_total` ≈ `landmarks_rendered_anchor_total` — Fix #11b working
   - `slam_promotions_seeded_with_midas` ≥ 0 (positive on axial portions) — Fix #12 Phase 1 active
   - `loop_closure_accepts` > 0 — LC firing on long walk (Morad's hypothesis)
   - `loop_closure_corrections_applied` > 0 — chi² accepting now that Fix #10 keeps state clean
   - `landmarks_reanchored_total` > 0 — Fix #9 finally triggered
5. **User-visible check** (the only criterion that actually matters, per memory `feedback_no_metric_celebration`):
   - On revisit, do plain orange dots reappear near their visible features on the wall?
   - Do white-ring orange dots (live SLAM) track features as the camera moves (not "stuck mid-air")?
6. **If user-visible fails despite counters being healthy:** ship the deferred work.
   - Highest priority: Phase 2 MiDaS live update (closes the per-frame depth observability gap).
   - Second priority: LC soft correction (lets the state converge even when chi² would reject).
   - Third priority: investigate trajectory-monotonic bug.

---

## Tonight's open invariants (DON'T BREAK)

- Camera-overlay dots project through `snap.p_G` (EKF state), not `Tracker.global_t_`. The two diverged by ~4 m today; fixing that divergence is a separate problem.
- The R_bc baking-into-clones convention from 2026-05-09 Option C is still in force. Don't add R_bc updates to the EKF state.
- `-fno-finite-math-only` in `app/CMakeLists.txt:17` is load-bearing.
- All work on `morad` branch only. Never commit to master.
- The chi²-gate threshold (22.5) is NOT to be tuned. Per the project's hard-learned lesson — find what corrupted state, don't loosen the gate.

---

## Conversation context for next agent

Morad's stated experience tonight:
- Fix #8 (ANR): "looks fine" — confirmed.
- After all subsequent fixes: "didn't notice the orange dots, only 2-3 orange-with-white-ring."
- His test pattern: walk forward 2-3 m → walk back → walk forward (phone always facing same wall). This is pure-axial. He explicitly confirmed: "I didn't rotate, I walked in reverse while the phone was still facing the same direction."
- His tolerance: he's been patient through 5 hours of debug; he's not asking for anything unreasonable. The goal — orange dots reappearing on revisit — is reasonable AND aligned with how monocular VIO/SLAM systems work in the literature.
- His next move: longer walk tomorrow to give LC the time it needs.

---

**Good night. Don't tune the chi² gate. Read the memory before you write code. Lead the report with what Morad can see, not the metrics.**
