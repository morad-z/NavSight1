# SLAM-Lifecycle Bug Hunt — 2026-05-31

**Method:** adversarial multi-agent hunt (26 Opus agents) along the SLAM point lifecycle
(creation → life → map → revisit → impact). Each candidate bug was independently verified by a
skeptic agent that tried to *refute* it against the actual code. No guessing — every claim cites `file:line`
or a measured counter.

**Data:** house-loop `tests/sims/val_2026_05_30b/simulation_data_1780150361149.json` (~100 m loop).

**Result:** 18 candidates verified → **7 REAL-BUG**, 8 BY-DESIGN, 2 DEGENERACY-NOT-BUG, 1 REFUTED.

> **Headline:** 6 of the 7 real bugs are **dormant** because `slam_promotions_total = 0/24084`
> (forward-motion degeneracy — a separately-confirmed *non*-bug). Exactly **one fires on every walk** (H1).
> **None of the 7 is the 1 km/5 % lever** — treat this as a *pre-flight checklist for reviving SLAM promotion*,
> not the next sprint toward the distance goal.

---

## 1. Confirmed real bugs

| # | Sev | Title | Key `file:line` | Mechanism | Impact hop | Fix |
|---|-----|-------|-----------------|-----------|------------|-----|
| **H1** | HIGH | LandmarkMap LC back-write is **dead code**; ~99.7 % of map never corrected | `Tracker.cpp:8229` (calls the LoopClosureDetector overload, not the map fn); `LandmarkMap.cpp:741-778` (real fn, **0 prod callers**); `LandmarkMap.cpp:706-712` (reanchor orphan-skips marginalized hosts) | LC back-write mutates only the keyframe DB record, never landmark `p_world`. `LandmarkMap::applyKeyframePoseCorrection` exists + is unit-tested but is never wired. The only landmark-position writer that runs (`reanchorLandmarksFromClonePoses`) skips every landmark whose host clone marginalized out of the window. Measured `landmarks_reanchored = 5 / 1468` → 99.7 % stay at drifted first-pass coords forever. | **MAP / revisit** — overlay + data-association use stale `p_world`. **Does NOT touch the dot.** | Add `landmark_map_.applyKeyframePoseCorrection(kf_id, pdx,pdy,pdz,pdyaw)` next to the existing LC call at `Tracker.cpp:8229`; add a `landmarks_pg_shifted_total` counter. **Fix L1 first** (the map fn has a pivot bug). |
| **H2** | HIGH | Re-anchor resets SLAM inverse-depth mean to the **promotion-time** point, discarding accumulated depth corrections; covariance desyncs | `EKFState.cpp:2613` (rebuilds state from `p_global_FEJ`); `:2683` (FEJ write-once, never refreshed); `:2575` ("covariance intentionally not transformed") | EKF continuously corrects `f.state (α,β,ρ)`; re-anchor rebuilds the state from the **promotion-time** triangulation, overwriting the accumulated depth correction. Fires ~every marginalize (window_[1] becomes front next frame). Reverted mean paired with a tightened covariance → mis-scaled χ²/gain. Contradicts the impl's own spec (`docs/study/post_v19_sprint_plan.md:120`). | **LIFE (depth refine)** — defeats the SLAM depth update the moment promotion succeeds. | Transfer the world point from the **current corrected** state + the old anchor's FEJ pose (per plan:120), project into the new anchor, refresh `p_global_FEJ`, and transform the 3×3 feature self-block by the reanchor Jacobian. Gate behind a synthetic re-anchor test. |
| **H3** | HIGH | Live-SLAM anchor-clone rotation Jacobian built in **body** frame, written into **camera**-frame δθ_c column without the `R_bc` bridge | `EKFState.cpp:3351` (body-frame J), written at `:3381`; clone cov is camera-frame δθ_c (`:1429-1439`; bridge present at `:1472`) | `J = R_GtoI_a^T·[p_I]_×` is the body-frame Jacobian; the clone covariance is camera-frame. Missing `R_bc.t()` conversion → result carries a trailing `R_bc` (~180° rotation) → mis-rotates the anchor-rotation correction. The keyframe-rate path (`slamReprojectionJacobian`) is correct; bug is unique to the **live** path, which writes the IMU body block. | **LIFE → DOT** — perturbs EKF rotation/cov ⇒ **heading risk**. | Compute directly in the camera frame, mirroring `slamReprojectionJacobian`: `J = R_GtoC_a_FEJ.t() * skewVec(p_anchor_cam)`. Add a numeric-diff unit test for a non-identity `R_bc`. **Do not enable any promotion path until fixed.** |
| **H4** | HIGH | Two divergent SLAM reprojection models consume the **same** promoted-feature obs **twice per frame** | Path A `applySlamLiveBatch` (`Tracker.cpp:2388` / `EKFState.cpp:3203`); Path B `update_skf` (`Tracker.cpp:4796` / `UpdaterSLAM.cpp:74-78`) | Both run per-frame in one `processFrame()`, both call `applyMSCKFUpdate` with **no shared "consumed" bookkeeping**. Path A has IMU body coupling; Path B does not — the same physical measurement is folded twice through inconsistent linearizations. | **LIFE → DOT** — double-counts SLAM evidence. | Keep ONE model (route the Kalman correction solely through `applySlamLiveBatch`, the intended prod path); comment out the `update_skf` call at `4796-4839` (keep lifecycle PROMOTE/DEMOTE + read-back). Validate exactly one fires per frame. |
| **M1** | MED | SLAM live update's `p_G_` correction discarded vs the dot by next-frame `setPosition` overwrite | `EKFState.cpp:3373-3375` (writes δp_G), `:1199` (`p_G_ += dx`); overwrite `Tracker.cpp:1870`; dot reads `global_t_` `:6481` | The live batch genuinely moves `p_G_`, but frame-start `setPosition(global_t_)` hard-overwrites it; `global_t_` is never fed from the SLAM update → zero net effect on the dot. | **DOT** — the p_G_-vs-global_t_ architecture disconnect. | Safe channel = delta-injection (`global_t_ += (p_after − p_before)`) like LC at `7898-7904` — but per the *Feed global_t_?* verdict, SLAM features carry RELATIVE info → likely no-op/redundant. **Do not enable blind.** Precondition: fix degeneracy so promotions > 0, then A/B-measure a guarded inject with an incoherence guard. |
| **M2** | MED | MiDaS depth rescue/seed samples the depth map at the **undistorted** pixel while the map is built on the **distorted** frame; fit/query also disagree on image size | map built raw/distorted `SensorRepository.kt:1119-1128`; fit at distorted pixels `Tracker.cpp:286-287`; query at undistorted pixel `:4557, :4674`; size mismatch `:628-629` vs `:269-270` | The affine fit lands on the distorted grid, the query lands on the undistorted grid → wrong depth-map cell, error grows radially (k1 = 0.263). `2*cx_` size approx disagrees with the true frame size. **Vanishes on default zero-distortion intrinsics; manifests on a calibrated (production) device.** | **CREATION (seed depth)** — biases rescued/seeded `p_world`; this *is* the documented "rescue dies at re-score." Not the zero-promotion cause. | Pass the **raw distorted** pixel to `sampleMidasMetricDepth` (store `pixel_raw` alongside `pixel_ud`); replace `2*cx_/2*cy_` with true `frame_w_/frame_h_` members. Pure-geometry, heading-safe. **Bundle with the PnP-with-depth work.** |
| **L1** | LOW | `LandmarkMap::applyKeyframePoseCorrection` rotates landmarks about the **world origin**, not the keyframe pivot; shielded by a wrong test; currently dead | `LandmarkMap.cpp:767` (`Rz*p_world + dp`, no pivot); wrong test `test_landmark_map.cpp:194-201`; multi-kf double-shift hazard `Tracker.cpp:8216` | Correct form is `Rz·(p_world − t_kf) + t_kf + dp`. Shipped form omits the pivot → spurious translation `~|t_kf|·2sin(dyaw/2)`, growing with distance from start. The fn isn't even passed `t_kf`. | **MAP** — only matters once H1 wires this fn in. | When wiring H1, pass `t_kf` and use the pivot form — **or** prefer the already-correct `reanchorLandmarksFromClonePoses`. Fix the test to use a non-origin keyframe; guard the multi-keyframe double-shift. |
| **L2** | LOW | `n==2` descriptor refresh is a guaranteed no-op (median tie-break always keeps the oldest descriptor) | `LandmarkMap.cpp:570-583` (strict `<` from idx 0), `:587-591` | For `n==2` both rows have median = d; strict `<` keeps `best_idx=0` = oldest. First refresh (ring 1→2) discards the just-matched descriptor; counter never bumps. | **LIFE (match quality)** — overlay/match quality only; does NOT affect the dot. | Break ties toward most-recent: `if (median <= best_median)` at `:580`. (Counter stays ~0 until the refresh path actually fires anyway.) |
| **L3** | LOW | `update_skf` residual comment says "normalised image coords" but the residual is **pixel-space** — latent fx-scale units trap + misnamed `pixel_ud` field | `UpdaterSLAM.cpp:80` (false comment); pred is pixels `EKFState.cpp:2937-2939`; misnamed `pixel_ud` `FeatureManager.h:147-148, 308-309` | The live caller is pixel-consistent, so no active misfire — but a NORMALISED convention coexists under the misnamed `pixel_ud`; a future caller trusting the comment + feeding normalized obs would get residuals ~fx too small that silently pass every χ² gate. | **LIFE (doc/latent)** — zero current numeric impact. | Pure comment/naming fix: correct `:80` to say PIXEL coords + px² noise; rename/annotate `pixel_ud` as a normalized bearing. No runtime change. |

> M2 and L1 were each surfaced by two separate candidate agents; the verified pairs are merged into one row each.

---

## 2. By-design / degeneracy (do **not** chase)

- **BY-DESIGN** — `addSlamFeature` "state-from-current-pose vs FEJ-from-FEJ-pose": textbook FEJ-EKF (live mean for the residual, frozen first-estimate for the Jacobian), documented at `EKFState.cpp:2913-2915`.
- **BY-DESIGN** — SLAM mean from current anchor pose with FEJ stored for the Jacobian: same FEJ contract (the second duplicate FEJ candidate).
- **BY-DESIGN** — `reanchorSlamFeature` doesn't similarity-transform the 5×5 P_ block: deliberate FEJ simplification (`plan §4a`); J ≈ I per step by design (new anchor = window_[1]); dead while promotions = 0. *(One reviewer logged this as REAL-BUG/LOW — same mechanism, near-zero impact; defer until promotion is restored.)*
- **BY-DESIGN** — `cullStaleLandmarks` obs-gate `1 < 1` always false: deliberately relaxed cull policy (commit a17a49b, `LandmarkMap.h:222-240`) to keep ever-triangulated landmarks for re-recognition; the no-op cull does not dirty the KD index.
- **BY-DESIGN** — matched-landmark MSCKF update shifts `p_G_` but not the dot: withholding a **known-bad** self-referential correction (v28: 108 m walked → 417 m `p_G`). Only genuine LOW item here: gate the accepted-id list on Pass-2 success.
- **BY-DESIGN** — accepted landmark EKF correction not mirrored into `global_t_`: adding the mirror would re-expose the dot to the v28 417 m divergence. (Candidate's HIGH framing refuted.)
- **BY-DESIGN** — LC mirrors only the position delta into `global_t_` (drops rotation/clone/SLAM): intentional v22 architecture; heading is Madgwick-sourced (HEADING-OFF-LIMITS); the rotation correction persists in EKF state, it just doesn't steer the dot.
- **DEGENERACY-NOT-BUG** — landmarks anchored to the drifting `global_t_` frame at creation: that *is* the only world frame a no-GPS monocular VIO has; irreducible monocular drift. Fix = external anchor (map-matching / LC), not a code change.
- **REFUTED** — BoW LC target from drifting-frame poses + two-view triangulation: correct single-session monocular loop closure; anchoring to an earlier, less-drifted keyframe is the *purpose* of LC. Gated by χ² + 90° rot-sanity + 1 m rollback + the pose-graph guard.
- **REFUTED** — SLAM "mean-from-current / FEJ-for-Jacobian internal inconsistency": standard OpenVINS FEJ; computing the mean from FEJ would be the actual defect.

---

## 3. The lifecycle bug map

```
CREATION         LIFE (per-frame refine)      MAP (persist)        REVISIT (LC/match)     IMPACT→DOT
  M2  ----------  H2 (depth reset)             H1 (back-write       L2 (descriptor          M1/H3/H4
(MiDaS seed       H3 (rotation J → dot)         dead, 99.7%          tie-break)             route to
 depth biased)    H4 (double-count → dot)       orphaned)                                   global_t_ but
                  L3 (latent units trap)       L1 (origin-rotation                          overwritten/
                  reanchor-P_ (by-design)       once H1 wired)                              double-counted
```

- **CREATION** — M2 biases seed depth (worse at edges). One bug.
- **LIFE** (per-frame depth/pose refine) — heaviest cluster: H2, H3, H4 (+ L3 latent, + by-design reanchor-P_). Promotion-time corruption lives here.
- **MAP** (persistence) — H1 (the only bug that fires today) + L1 (its companion math bug). The map is never corrected after LC.
- **REVISIT** (data association) — L2 (descriptor refresh quality). Matching itself works (~55 %). **Cleanest hop.**
- **IMPACT → DOT** — M1/H3/H4 are the routes by which SLAM evidence reaches `global_t_`; all blocked or double-counted.

---

## 4. Prioritized fixes (for the 1 km / 5 % goal; heading off-limits)

**Fix now — improves what the user sees today, independent of promotion:**
1. **H1 + L1** — wire the LandmarkMap back-write with the pivot form. The **only** confirmed bug that fires on every walk (`landmarks_reanchored = 5/1468`). Fixes persistent-map / revisit overlay correctness. **Does NOT touch the dot** — but aligns with the map-matching direction.

**Bundle *with* the PnP-with-depth promotion fix (block the foot-guns; do not enable promotion without these):**
2. **H3** — body→camera Jacobian; would corrupt the dot the instant promotion succeeds.
3. **H4** — pick one reprojection model; double-counts the moment promotion succeeds.
4. **H2** — re-anchor depth reset; defeats SLAM depth refinement the moment promotion succeeds.
5. **M2** — MiDaS distorted/undistorted sampling; on the promotion-rescue path itself.

**Moot while promotion = 0 (parked):** M1, the reanchor-P_ transform, L2, L3.

**The 1 km/5 % lever is NOT in this bug hunt.** Along-track 5 % error = SPEED (per-mode-K / learned-inertial); cross-track = MAP-MATCHING. No SLAM-lifecycle bug here improves `global_t_` distance/scale.

---

## 5. Honest note

The SLAM live/refinement bugs (H2, H3, H4, M1) are **not worth debugging for their own sake right now** — every
one is latent behind `slam_promotions_total = 0/24084` (forward-motion degeneracy, a confirmed non-bug). They produce
zero measured effect today. They start mattering **only after PnP-with-MiDaS-depth revives promotion** — at which
point H3 and H4 would actively corrupt the dot and H2 would defeat the depth refinement that PnP-with-depth is trying
to add.

Correct sequencing: **H3 + H4 + H2 + M2 are mandatory pre-work to ship bundled *with* the PnP-with-depth promotion
fix** — not standalone tasks, and never enable promotion without them or you ship the corruption live.

Two things worth doing regardless:
- **H1 (+ L1)** fires today and fixes the persistent-map / revisit overlay — the one bug with real current impact.
- Everything else (M1, reanchor-P_, L2, L3) stays parked until promotion is alive.

**Bottom line:** real bugs, but **none is the 1 km lever.** That lever is per-mode-K / learned-inertial (along-track)
+ map-matching (cross-track), per the standing verdicts. This report is a pre-flight checklist for the SLAM revival.

---

*Companion memory: `project_slam_bughunt_2026_05_31`. Related: `docs/MSCKF_PG_WIRING_VERDICT_2026_05_31.md`,
`project_feed_global_t_2026_05_31`, `project_msckf_pg_verdict_2026_05_31`.*
