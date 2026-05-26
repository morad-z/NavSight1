# Geometric LC path (Step 7.1) — root cause + SOTA survey

**Author**: researcher agent (dispatched 2026-05-26) — investigation only, no source changes.
**Question**: why does NavSight's GEOMETRIC loop-closure path (`loop_closure_geom_accepts = 0` in every walk ever) never accept, and how does SOTA do it?
**Builds on**: [agent_01_descriptor_matching.md](agent_01_descriptor_matching.md).
**Data**: `tests/sims/regression/visual/lc_split_2026_05_26.json` (post-heading-fix 2-loop walk): geom_attempts=153, geom_accepts=0, spatial_miss=532 (65%), hamming_miss=290, mean min-Hamming-in-radius=99 (random≈128).

---

## A. Root cause (code-verified)

The geom path establishes its 2D–3D correspondences **the wrong way round vs every SOTA system**: it **projects stored 3D points using the live, drifted EKF pose**, then looks for a current keypoint within 15 px of each projection.

- `Tracker.cpp:5303-5318` — projection pose = **live EKF state** (`R_world_cam_pred = ekf_.getRotation().t()·R_bc.t()`, `t_cam_world_pred = ekf_.getPosition()`) = the drifted `p_G` / `R_GtoI`.
- → `publishLoopClosureQueryKeyframe` (`Tracker.cpp:5345-5354`) → `tryDetectLoopGeometric(predicted_R_world_cam, predicted_t_cam_world)` (`Tracker.cpp:6472-6483`), fired **only when BoW returns false** (`Tracker.cpp:6446`).
- `LoopClosureDetector.cpp:919-960` project; `:1029-1051` match within `kGeomMatchRadiusPx=15`, min-Hamming, reject > `kGeomDescriptorMaxDistance=50`.

**Ranked mechanisms:**
1. **(Dominant) Projection-under-drift → `spatial_miss`.** LC is needed *because* the pose drifted (~17-20°/loop). Using that drifted pose as the projection prior: at f≈640 px, 1° heading err ≈ 11 px, 2° ≈ 22 px, 5° ≈ 56 px (+ parallax from position drift) → the 15 px disc is empty. **532/822 = 65%** (2026-05-26); 67-79% across agent_01's 4 walks. Pure geometry, descriptor-independent. The 15 px radius was derived for *fresh-keyframe* MSCKF noise (`LoopClosureDetector.cpp:139-144`) — wrong budget for a 30-80 s-old keyframe under a drifted pose.
2. **Wrong-feature association → near-random Hamming.** Projection already off → the keypoint inside the disc is usually a *different* corner → mean min-Hamming **99** / 106 ≈ random (128). The Hamming gate is **correctly rejecting** a bad pair; the fault is the upstream pairing. (Secondary: descriptors frozen at first-observation `:381`, never refreshed.)
3. **(Amplifier, not root) Sparse 3D.** `filled_3d≈1/85467`, `slam_promotions=0`. Shrinks the pool before the 65%/95% attrition; fixing alone won't unblock.

**Headline:** #1 and #2 are one flaw — correspondences are established by projecting with a pose that is wrong at LC time *by definition*. NavSight's **BoW path proves the alternative works**: it matches by appearance (`BFMatcher::knnMatch` + Lowe, `LoopClosureDetector.cpp:662-689`) with **no pose prior** and accepts 3-13 loops/walk.

## B. SOTA — appearance establishes correspondences; geometry only verifies

**None of these uses the drifted global odometry pose to *find* LC correspondences.**
- **ORB-SLAM2/3** (Mur-Artal T-RO 2017; Campos T-RO 2021): `ComputeSim3` = (1) `SearchByBoW` (vocab-node Hamming match, **no pose prior**, NNratio 0.6) → >20 matches → (2) Sim3 RANSAC (Horn 3-pt) → (3) `SearchBySim3` projects map points using the **just-solved relative Sim3** (guided refinement), never the global prior. NavSight does only a stage-3-style projection against the drifted global pose.
- **VINS-Mono/Fusion** (Qin T-RO 2018): DBoW2 retrieve → 500 BRIEF corners → **correspondences by BRIEF matching (no pose prior)** → 2D-2D fundamental-matrix RANSAC → 3D-2D PnP RANSAC. PnP *verifies* already-matched features.
- **OpenVINS** (Geneva ICRA 2020): no built-in LC; extensions use DBoW2 / NetVLAD+SuperPoint (ov_hloc).
- **DBoW2** (Gálvez-López T-RO 2012): the direct index groups features by vocab node — the primitive for cheap pose-free feature-to-feature matching. NavSight built `OrbDatabase` with `use_di=false` (`LoopClosureDetector.cpp:328-330`).

## C. Recommendation (cause / change / falsifier). Order: C.3 → C.1 → C.2 → C.4.

- **C.1 [PRIMARY, LOW risk] — make the geom path appearance-first.** *Cause*: pairs via projection-under-drift; SOTA + NavSight's BoW path pair by appearance with no pose prior. *Change*: keep the spatial candidate filter (Stage 1, `:867-905`), but match candidates with the **already-shipped** `BFMatcher::knnMatch` + Lowe 0.75 (reuse `:662-689`) → feed `(pts3d, pts2d)` to the existing PnP (`:1097-1130`); demote projection to an optional guided 2nd pass using the **relative** PnP pose (ORB-SLAM `SearchBySim3` style), never the global prior. *Falsifier*: 2-loop walk shows `loop_closure_geom_accepts > 0` AND accepts pass chi². *Risk*: LOW — reuses validated matcher + PnP gate (`kPnpMinInliers=15`, reproj=4).
- **C.2 [SECONDARY] — descriptor refresh** (median over observations, cap-5 ring) on LandmarkMap + `KeyframeRecord`; recompute `bow_vec` on update (protect DBoW2 tf-idf). (agent_01 §5.1/5.3.)
- **C.3 [DIAGNOSTIC, gate before any radius change] — log nearest-current-keypoint distance regardless of radius** (p50/p90 → event_summary). NOT yet measured. If p50 ≈ 16-25 px → a *derived* widen to ~30 px is justified alongside C.1; if p50 ≫ 40 px → only C.1 helps. Blind radius widen = banned magic-number tweak.
- **C.4 [AMPLIFIER, separate bug]** — SLAM-promotion / 3D density; don't block C.1 on it.
- **C.5 [DO NOT SHIP]** — loosening `kGeomDescriptorMaxDistance` alone (correctly rejecting ≈99-bit wrong-feature pairs → would admit false-positive corrections).

**Nuance**: the geom path fires only when BoW returns false (`Tracker.cpp:6446`), so C.1's marginal value is mainly **low-BoW-score / heading-gate-blocked (opposite-direction) revisits** — exactly the reliability gap (the no-LC walk). Measure `loop_closure_geom_accepts` vs BoW `loop_closure_accepts` to judge.

## D. Confidence
- **HIGH, code-verified**: geom path uses the live drifted EKF pose; Stage-3 15px/Hamming-50 logic; BoW path is appearance-first + accepts; `pts3d_world` source + `filled_3d≈0`.
- **HIGH, inferred** (matches agent_01): projection-under-drift dominant (#1), wrong-feature association (#2).
- **HIGH, literature**: ORB-SLAM SearchByBoW→Sim3→SearchBySim3; VINS-Mono BRIEF→F-RANSAC→PnP.
- **LOW until C.3 runs**: whether widening 15 px is justified (distribution unmeasured).
- **Honesty note**: C.1's accept *quality* can't be confirmed from existing logs (geom path has never matched by appearance) — needs the falsifier walk.

## Sources
ORB-SLAM2 ORBmatcher.h; ORB-SLAM3 (UZ-SLAMLab); ORB-SLAM3 Campos et al. T-RO 2021; VINS-Mono Qin et al. T-RO 2018 (arXiv:1708.03852); OpenVINS docs (Geneva et al. ICRA 2020); ov_hloc; DBoW2 Gálvez-López & Tardós T-RO 28(5) 2012.
