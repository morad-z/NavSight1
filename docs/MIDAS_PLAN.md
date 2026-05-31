# MiDaS Integration Plan for NavSight

**Last updated:** 2026-05-20
**Owner:** next agent picking up the orange-dot anchoring debug
**Status:** Phase 1 shipped (unvalidated). Phase 2 designed (not started). Phases C-E scoped.

---

## Tier-of-confidence labels used in this document

| Label | Meaning |
|---|---|
| **[RESEARCH]** | Method from published peer-reviewed work. Citation provided. |
| **[PROJECT-VALIDATED]** | Already shipped in NavSight and confirmed on a real walk. |
| **[DESIGNED]** | Code written and built, but NOT walk-validated yet. |
| **[EXTRAPOLATED]** | Logical extension of [RESEARCH] / [PROJECT-VALIDATED]; not measured. |
| **[SPECULATIVE]** | My reasoning from first principles; no evidence yet for or against. |

The next agent should treat [EXTRAPOLATED] and [SPECULATIVE] claims as hypotheses to verify, not facts to act on.

---

## Overview

MiDaS is a monocular depth estimation neural network. Given a single image, it outputs a per-pixel disparity (inverse relative depth) map. NavSight runs the MiDaS v2 small model on-device via TFLite (DepthEstimator.kt), producing a 256×256 disparity grid per frame at ~1 Hz on the S21 Ultra.

**[RESEARCH]** Source: Ranftl et al., "Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer," TPAMI 2020. The training loss is **scale-and-shift invariant**, so the network's raw output is relative — to recover metric depth, an affine transform `(s, t)` must be fit per scene such that `metric_depth = 1 / (s · disparity + t)`.

**[RESEARCH]** Affine-fit method NavSight uses: Wofk et al., "VI-Depth: Visual-Inertial Monocular Depth Estimation," ICRA 2023. NavSight implements VI-Depth Stage 1 (closed-form 2×2 weighted LSQ) per Phase 2 Step 4.2.1 of the project's productization plan.

**Why MiDaS matters for NavSight specifically:** the system is monocular VIO with no stereo or active depth sensor. Triangulation, MSCKF, and SLAM all depend on the camera moving with parallax to recover depth. **MiDaS is the only depth source in the stack that does not require motion.** That makes it the critical depth source whenever the user is stationary, walking purely axially, or moving fast enough that features pass through the FOV before parallax accumulates.

---

## Current State (what's already shipped)

### A1. Affine fit, global scale (Phase 2 Step 4.2.1, 2026-05-17) — [PROJECT-VALIDATED]

`Tracker::applyDepthScaleConstraint`:
- At each keyframe, samples MiDaS disparity at every triangulated feature pixel.
- Fits `inv_metric_depth ≈ s · disparity + t` via closed-form 2×2 LSQ.
- Residual-based inlier classification: threshold = `max(0.01, 3 × MAD)`.
- Acceptance bar: ≥ 50 % inliers.
- Feeds resulting metric/VIO ratio into `scale_fuser_` so visual scale stays metric over the walk.

**Evidence of working:** 2026-05-17 walk's `event_summary.midas_affine_fit_inlier_ratio_milli ≥ 500` and `midas_fused > 0`. Codepath: Tracker.cpp:155-528.

### A2. Per-pixel sampler helper (Fix #12 Phase 1, 2026-05-19) — [DESIGNED]

`Tracker::sampleMidasMetricDepth(u, v, &depth_m_out)`:
- Reads cached affine fit `(s, t)` under `midas_affine_mutex_`.
- Bilinear-interps disparity at the image-space pixel `(u, v)` on `depth_map_`.
- Applies `metric = 1 / (s · disparity + t)`.
- Returns true if depth is in `[kMinMidasDepthM, kMaxMidasDepthM] = [0.3, 30]` m and MiDaS sample ≥ 0.01.

Counter: `midas_depth_samples`. Codepath: Tracker.cpp:530+.

### A3. SLAM promotion sanity check (Fix #12 Phase 1, 2026-05-19) — [DESIGNED]

At each SLAM feature promotion (Tracker.cpp:3214+):
- Compute triangulated depth in anchor camera frame.
- Sample MiDaS at anchor observation pixel.
- If both valid AND disagree by > 2× ratio → REPLACE triangulated `p_world` with MiDaS-derived position.
- Math: `p_anchor_cam = (obs.x · z_midas, obs.y · z_midas, z_midas); p_world = R_anchor.t() · p_anchor_cam + p_anchor`.

Counter: `slam_promotions_seeded_with_midas`. Log: `SLAM_PROMOTE_MIDAS_SEED:` line per replacement.

**Validation pending:** Tomorrow's long walk should show this counter > 0 on the axial-motion portion AND LC `target_p` falling within 5 m of `p_G` (was 65 m pre-fix).

---

## Roadmap

### Phase A — Validate Phase 1 (tomorrow's walk)

**[DESIGNED]** Fix #12 Phase 1 is built and installed but never walked. Tomorrow's > 60 s walk will produce the validation dataset.

**Falsifier criteria (per implementor-skill discipline):**

1. `event_summary.midas_depth_samples > 0` — sampler is being called by consumers.
2. `event_summary.slam_promotions_seeded_with_midas` ≥ 1 during pure-axial segments — MiDaS is replacing bad triangulations as designed.
3. Logcat shows `SLAM_PROMOTE_MIDAS_SEED:` lines with `ratio` outside `[0.5, 2.0]` (the cases we want MiDaS to win on).
4. Post-promotion, LC `LC_ABS` lines should show `|target_p − p_G| < 10 m` (was 65 m before Fix #10 + #12).

**If any of those fail:**
- (1) failing → consumers aren't calling the helper. Check Tracker.cpp wiring.
- (2) failing on axial-motion walks → either MiDaS depth is wrong (affine fit broken), or 2× threshold is wrong, or no SLAM features are being promoted during the axial phase.
- (3) failing → MiDaS and triangulation always agree (might be true; not necessarily a bug).
- (4) failing → there's still ρ-corruption upstream of LC; need Phase 2.

### Phase B — Live SLAM depth update from MiDaS (Phase 2 of Fix #12) — [DESIGNED]

The big one. Current state: Fix #12 Phase 1 only seeds depth at SLAM feature CREATION. After that, ρ evolves via the existing live-update path which Fix #10 SKIPS during axial motion (parallax gate). So during a long axial run, ρ never refines — fine if initial was correct, but no defense against any noise that does slip through.

**Goal:** add a 1D MiDaS depth measurement update that fires when the parallax gate WOULD have skipped a normal update. The feature's ρ converges toward MiDaS-implied depth continuously, even in pure-axial motion.

**[DESIGNED]** Architecture:

```cpp
// New EKFState method:
bool updateSlamFeatureMidasDepth(
    int slot,
    double depth_metric_m,   // From sampleMidasMetricDepth
    double sigma_m);          // From MiDaS affine-fit inlier-residual std

// Math:
//   predicted_depth = p_C_LIVE[2]
//   where p_C_LIVE = R_bc · R_GtoI · (p_world - p_G)
//   and p_world = R_anchor.t() · (α/ρ, β/ρ, 1/ρ) + p_anchor
//
//   residual r = depth_metric - predicted_depth
//   H = ∂(predicted_depth) / ∂(state)  — 1 × state_dim, sparse
//
// Build the 1×dim H row:
//   - IMU θ block (cols 0-2)        : ∂z/∂δθ
//   - IMU p block (cols 12-14)      : ∂z/∂δp_G
//   - Anchor θ block (clone cols)    : ∂z/∂δθ_a (FEJ side)
//   - Anchor p block (clone cols)    : ∂z/∂δp_a
//   - SLAM (α,β,ρ) block             : ∂z/∂(α,β,ρ)
//
// Then applyMSCKFUpdate(H, r, R = sigma²).
```

**Tracker wire-up:**
- In the SLAM live batch loop, AFTER `applySlamLiveBatch`, for each SLAM slot:
  - Re-build the parallax gate check (same math as in `buildSlamLiveJacobianRow`).
  - If the gate would have skipped: sample MiDaS at the slot's current observation pixel.
  - If MiDaS valid: call `updateSlamFeatureMidasDepth`.

**Counters:**
- `slam_live_midas_depth_fired` — successful depth-only updates
- `slam_live_midas_depth_rejected_chi2` — failed per-row chi² gate (1-DOF, threshold 3.84)

**[RESEARCH]** Per-row chi²(95%, 1-DOF) = 3.84 — standard table value. Conservative because a single bad MiDaS sample shouldn't be allowed to poison the state.

**σ_m derivation:**
- Read `midas_affine_fit_inlier_ratio_milli` from EventCounters (current gauge).
- σ_m base = 0.5 m (typical MiDaS metric error per VI-Depth paper §5.1 — [RESEARCH]).
- Scale by inlier-ratio: `σ_m = base × (1.0 / inlier_ratio)`. Low confidence fits get high σ → less weight in the Kalman update.

**[EXTRAPOLATED]** Estimated impact: ρ stays bounded during arbitrary-duration axial motion. SLAM features become usable for the full duration they're tracked, not just until parallax dies.

**Risk:** Bad MiDaS samples (occlusions, reflective surfaces, depth-discontinuity edges) inject noise. Mitigations:
1. Per-row chi² gate at 3.84 (1-DOF, 95%).
2. Only fire when `midas_affine_fit_inlier_ratio_milli > 700` (high-confidence fit).
3. Require MiDaS sample disparity > 0.05 (network's confidence threshold).
4. Cap the per-update Kalman gain to prevent any single update from shifting ρ by > 30 %.

**Scope:** ~150 LOC in EKFState (new method + math). ~50 LOC in Tracker (wiring + gating). 2 new counters.

### Phase C — Cross-cutting consumer integration

After Phase B, four other consumers also benefit from MiDaS-derived depth. Each is a smaller, scoped fix.

**C1. LandmarkMap addOrMerge sanity check** — [DESIGNED]

Same pattern as Fix #12 Phase 1 (SLAM promotion) but applied to LandmarkMap entry creation in Tracker.cpp:4057. When a new landmark's triangulated `p_world` disagrees with MiDaS by > 2× → use MiDaS-derived position.

Counter: `landmarks_added_seeded_with_midas`. Codepath addition: ~20 LOC.

**C2. MSCKF transient feature sanity check** — [SPECULATIVE]

MSCKF features are triangulated and immediately marginalized. Bad triangulation → bad MSCKF residuals → noisy EKF update on pose. Sanity-check `triangulated_p_global` against MiDaS at the feature's last observation pixel before stacking residuals in `UpdaterMSCKF::update`. Reject the feature track if disagreement > 5×.

Counter: `msckf_features_rejected_midas_disagree`. Codepath: UpdaterMSCKF.cpp. ~30 LOC.

**[SPECULATIVE]** because UpdaterMSCKF's quality is dominated by the chi² gate already — MiDaS sanity might not add much over that.

**C3. LC PnP `pts3d_world` filter** — [DESIGNED]

Today's diagnosis: LC PnP's `target_p` came out 65 m off because `pts3d_world` was polluted by SLAM features with diverged ρ. Even with Phase B preventing future divergence, ALREADY-diverged features in flight could still poison PnP until they're demoted.

Filter: at Tracker.cpp:3746 where `pts3d_world` is built from `getSlamFeatureGlobalPosition`, also sample MiDaS at the corresponding ORB keypoint pixel. Drop the entry if `|tri_depth − midas_depth| / midas_depth > 2.0`.

Counter: `lc_pts3d_filtered_midas_disagree`. ~30 LOC.

**C4. Initial scale seeding at first keyframe** — [DESIGNED]

Currently `EKFState::initialize(0.1)` starts scale at 0.1 and `scale_fuser_` converges over the rotational init phase. With MiDaS available at the FIRST keyframe (once `applyDepthScaleConstraint` completes its first fit), we can seed `scale_fuser_` directly from the MiDaS-implied scale. Shaves several seconds off scale convergence.

Counter: `scale_seeded_from_midas` (one-shot flag). ~20 LOC in Tracker init path.

### Phase D — Scooter-specific extensions

**[EXTRAPOLATED]** No scooter test data exists for NavSight. The following are designed assuming MiDaS's known properties from VI-Depth + general scooter VIO challenges. Validate before relying on these claims.

**D1. Pavement plane fit as metric scale anchor**

For a forward-mounted camera with known pitch (from EKF gravity alignment), the bottom-third of the frame is reliably the road surface. Fit a plane to MiDaS depths in that region:
- Use RANSAC over `(u, v, midas_depth)` for the lower-third pixels.
- Plane height = camera mount height (≈ 1.2 m on a scooter handlebar) — assumed constant or set as a calibration parameter.
- The pavement plane gives a continuous, robust metric scale anchor every frame.

Counter: `scooter_pavement_anchor_samples`. ~80 LOC. Depends on a mount-height calibration step.

**[SPECULATIVE]** This works in self-driving research (e.g., Mobileye's road-plane estimation) but I have not seen a published benchmark on a 2-wheel platform at 15 km/h.

**D2. Forward-collision distance (safety feature, not VIO)**

Min depth in the center-column band of the MiDaS map = distance to nearest obstacle ahead. Surface this via JNI to a HUD widget. Independent of the VIO state.

Useful as a stand-alone safety feature even if it doesn't improve navigation accuracy.

**D3. Speed estimation cross-check**

Track a static feature's MiDaS depth shrinking over time → forward speed (since the camera approaches at known velocity per IMU integration). Cross-check against IMU-derived velocity in `EKFState::v_G_` for drift detection.

Counter: `scooter_speed_consistency_milli` (ratio MiDaS-derived vs IMU-derived speed × 1000). Diagnostic only at first; could feed a velocity-correction update later.

**D4. Depth for distant features on scooter**

At 15 km/h, features beyond 5 m have near-zero parallax over 1 sec keyframe interval. MiDaS gives them depth on first sighting. Treat MiDaS depth as the depth source for any feature whose triangulated depth is < 1 cm baseline-implied confidence.

**[SPECULATIVE]** Requires Phase B foundation to be solid first.

### Phase E — Network upgrades (DEFERRED)

**[PROJECT-VALIDATED]** Per memory `project_da3_deferred_2026_05_16`: Phase 2 Step 4.2.2 (MiDaS → DA3 swap) is deferred indefinitely. V2 INT8 measured 722 ms CPU on S21 Ultra vs 100 ms budget. DA3 ≥ V2 cost. MiDaS v2 small stays.

Future option: revisit when on-device inference accelerators improve, OR move depth inference to a paired companion device (e.g., scooter-mounted SBC with a dedicated NPU). Not in any near-term plan.

---

## Validation requirements (gate before declaring any phase "done")

Per the implementor-skill discipline, no fix is shipped until:

1. **Falsifier identified.** Each fix has a named counter or log pattern that distinguishes "working" from "broken." Documented per-phase above.
2. **Build green.** `./gradlew.bat assembleDebug` passes.
3. **Walked on real device.** APK installed on S21 Ultra (`R5CR70S3NNB`), real walk recorded.
4. **Data analyzed.** Counter check + logcat grep confirms falsifier criteria.
5. **User-visible behavior.** Per memory `feedback_no_metric_celebration` — lead with what Morad can SEE, not the metric deltas. For MiDaS work specifically: "do orange dots reappear on revisit?" is the binary outcome that matters.

---

## Cross-references

### Existing project knowledge to read before touching MiDaS

- **`docs/study/04_updaters_scale.md`** — current scale fuser architecture; ScaleEstimatorVI, ScaleFuser API.
- **`docs/study/01_ekf_core.md`** §3 Public Functions table — which measurement update sites exist today.
- **`docs/study/phase2_productization_plan.md`** §4.2.1 — affine fit specification.
- **Memory** `project_midas_affine_fit_2026_05_17` — context on the affine fit's ship.
- **Memory** `project_da3_deferred_2026_05_16` — why DA3 is off the table.

### Papers (in citation order)

1. **MiDaS v2:** Ranftl et al., "Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer," TPAMI 2020. arXiv:1907.01341
2. **DPT (MiDaS v3):** Ranftl et al., "Vision Transformers for Dense Prediction," ICCV 2021. arXiv:2103.13413
3. **VI-Depth:** Wofk et al., "VI-Depth: Visual-Inertial Monocular Depth Estimation," ICRA 2023. arXiv:2303.03446 — **the most important paper for NavSight's affine-fit methodology**.
4. **DROID-SLAM:** Teed and Deng, "DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras," NeurIPS 2021. arXiv:2108.10869 — reference for learned-depth-aided SLAM in general.

### Implementation reference points in NavSight

| Pattern | Reference site | Lines |
|---|---|---|
| Affine fit | `Tracker::applyDepthScaleConstraint` | Tracker.cpp:155-528 |
| Per-pixel sampler | `Tracker::sampleMidasMetricDepth` | Tracker.cpp:530+ |
| Promotion sanity | SLAM promote site | Tracker.cpp:3214-3294 |
| Existing depth-map ingress | `Tracker::setDepthMap` | Tracker.cpp:142 |
| MiDaS counters | EventCounters.h | lines 609-624, plus new entries |

---

## Open questions to resolve before Phase B implementation

1. **Threading.** The MiDaS depth map (`depth_map_`) is updated on the camera thread or a separate worker? If a worker, the synchronization between the depth-map write and the SLAM live update read needs scrutiny. Check `DepthEstimator.kt` → `NativeBridge.setDepthMap` path.
2. **Disparity at distorted vs undistorted pixels.** Current `sampleMidasMetricDepth` takes image-space pixels and uses `2·cx` / `2·cy` as the image dimensions, which is an approximation. For accurate per-feature sampling, the function should accept either raw or undistorted pixels and convert correctly. The MiDaS network operates on the raw camera frame, so distorted pixels are technically the right input — but the SLAM/MSCKF feature positions are in undistorted space. Verify the error magnitude (probably < 5 px at frame edges with NavSight's k1=0.263).
3. **Frame-time staleness.** MiDaS runs at ~1 Hz on the S21 Ultra (722 ms inference per memory `project_session_2026_05_10_v2_bench`). Between MiDaS frames, the depth map is stale. For the live SLAM update, do we accept staleness up to 1 sec, or skip when stale > N ms?
4. **Phase B σ_m calibration.** The 0.5 m base sigma is a guess; the VI-Depth paper reports network error on TUM-RGBD which may not transfer to NavSight's camera. Best path: log the residual std-dev of MiDaS vs converged-VIO depth across a few real walks and pick σ_m from data.

---

**End of plan.** Next actions:
1. Validate Fix #12 Phase 1 on tomorrow's long walk (Phase A).
2. If validation passes, scope Phase B (live SLAM update) and ship.
3. If validation reveals upstream issues, refactor Phase 1 before Phase B.
