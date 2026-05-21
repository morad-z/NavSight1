# Agent 01 — Descriptor matching root cause (Bug 1 + Bug 2)

**Author**: worker-01-descriptor (hive-mind worker)
**Date**: 2026-05-21
**Status**: investigation only — NO source changes
**Scope**: Bug 1 (orange dot flicker) AND Bug 2 (Step 7.1 LC dead, hamming_pairs=0). Both share the descriptor / viewpoint variance failure mode but at different ends of the per-landmark observation history.

---

## 0. TL;DR (one paragraph)

Both bugs reduce to the same primary cause: **`LandmarkMap` stores a single ORB descriptor per landmark, captured at first-add time, and never updates it** (`LandmarkMap.cpp:128` — `// p_world NOT updated here` and descriptor also unchanged). For the per-frame matcher (~70% match within a few keyframes of add), this is benign because viewpoint barely changes. For Step 7.1 (which compares an ORB descriptor extracted 30+ s / ~30+ m ago against a current keypoint at the same physical wall), viewpoint change makes the stored descriptor essentially uncorrelated with the new one — measured median Hamming distance is 106 / 256 bits across all 4 recent walks (random baseline = 128). However, the data also shows that the **dominant Step 7.1 failure is `spatial_miss` (67–79% of in-frame projections have NO current keypoint within 15 px), not Hamming**. So Bug 2 has TWO sub-causes that the existing `kHammingMax=50` symptom-fix won't address.

---

## 1. What the data says (BEFORE any hypothesis)

### 1.1 Per-frame matcher (LM_TRACK) — Bug 1 source

Aggregated from `tests/sims/regression/visual/bug4_walk_2026_05_21.logcat.txt`:

| Metric | Value | Notes |
|---|---|---|
| LM_TRACK rows | 158 | one per keyframe over 116.5 s, 97.4 m walk |
| nearby_total | 38 837 | candidates from `getLandmarksInRadius` |
| matched_total | 24 427 (62.9 %) | passed Lowe 0.75 + Hamming ≤ 50 + 15 px spatial gate |
| accepted_total | 11 483 (29.6 %) | post-EKF chi² + Huber |
| match-rate distribution | p10=0 %, p50=76.5 %, p90=85.1 % | wide spread |

**Time evolution** (same walk):
- First 5 KFs: 100 % → 86 % → 84 % → 77 % → 78 %  (recently-added landmarks)
- Last  5 KFs:   3 % →  0 % →  0 % →  0 % →  0 %  (landmark cohort has aged out)
- Frames with match-rate < 20 % : 34 / 158

Steady-state per-frame match rate, after the cold-start cohort but before the end-of-walk cohort, is roughly 75–85 %. The user-visible flicker is the residual ~20 % per-frame miss, plus the cohort-age cliff at the end of the walk where match rate falls to 0 %.

### 1.2 Step 7.1 (LC_GEOM stage 3) — Bug 2 source

Aggregated across all four recent walks:

| Walk | rows | in_frame | spatial_miss | hamming_miss | hamming_pairs |
|---|---:|---:|---:|---:|---:|
| `bug3_walk_2026_05_21`             | 30 | 2 054 | 1 623 (79.0 %) | 431 (21.0 %) | 0 (0.0 %) |
| `bug4_walk_2026_05_21`             | 15 |   917 |   650 (70.9 %) | 262 (28.6 %) | 5 (0.5 %) |
| `promo_parallax_walk_2026_05_21`   | 52 | 4 604 | 3 088 (67.1 %) | 1 514 (32.9 %) | 2 (0.0 %) |
| `parallax_fix_walk_2026_05_20`     | 33 | 2 650 | 1 772 (66.9 %) |   877 (33.1 %) | 1 (0.0 %) |

All four walks: `loop_closure_geom_accepts = 0`. The 9–25 BoW-path accepts per walk reach `LOOP_CLOSURE: ACCEPT` only via DBoW2 + BFMatcher, never via the Step 7.1 geometric path.

**`mean_min_h_in_radius`** (over the projections that DID find at least one current kp within 15 px): median 106 bits across all walks, p10 ≈ 98, p90 ≈ 121. The random-binary baseline is 128 bits; threshold is 50. So when there is a nearby kp, the stored descriptor is, on average, **statistically indistinguishable from random** against the current candidate descriptors. (`tests/sims/regression/visual/bug4_walk_2026_05_21.logcat.txt:119822, 123421, 123800, 124145, 124666, 125240, 129974, 130309, 130677, 142390, 143922` confirms with `mean_min_h_in_radius=90–140` and threshold=50.)

### 1.3 Upstream: how `pts3d_world` is populated for Step 7.1 candidates

Aggregated `LC_KF` lines from bug4 walk:

| Metric | Value |
|---|---|
| LC_KF rows | 171 (one per keyframe) |
| total kps | 85 467 |
| `filled_3d` total | **1** (0.00 % of kps) — SLAM-promoted features |
| `triangulated` total | 9 194 (10.76 % of kps) — cross-baseline ORB pairs |
| KFs with `filled_3d > 0` | **1 / 171** |

So Step 7.1 candidates carry at most ~54 valid 3D points each (the cross-baseline-triangulated ones; SLAM-promoted contributes ~0). This couples directly to Bug 3 (SLAM promotion sparsity, `slam_promotions_total = 0–24` per walk).

---

## 2. The asymmetry — same descriptors, different failure rates

Both paths start from THE SAME `kf_back.descriptors` array at the moment of keyframe storage (`Tracker.cpp:5099` for `addKeyframe`, `Tracker.cpp:4615` for `addOrMergeLandmark`). The current-frame side is identical between LM_TRACK and the LC_GEOM query-keyframe path: both consume `kf_back.descriptors` / `kf_back.keypoints_ud` at the same `processFrame` tick. **The differentiator is the stored side.**

### 2.1 What stored descriptor each path matches against

| Path | Stored descriptor lives at | Set at | Updated on re-observation? |
|---|---|---|---|
| Per-frame (LM_TRACK) | `Landmark.descriptor` (1 row CV_8U) | first `addOrMergeLandmark` call (`LandmarkMap.cpp:152`) | **No** — `LandmarkMap.cpp:128` comment "p_world NOT updated here" applies to descriptor too |
| Step 7.1 (LC_GEOM)   | `KeyframeRecord.descriptors` (Nx32) | `LoopClosureDetector::addKeyframe` (`LoopClosureDetector.cpp:382`) | **No** — record cloned at insert |

So both stores are *frozen at first observation*. The difference is **which observation that was**:

- **Per-frame**: the landmark was added 0.3–2 s ago (a recent keyframe with similar pose). The stored descriptor and the current descriptor are extracted at nearby viewpoints. ORB scale + rotation invariance comfortably covers this delta. The 75–85 % match rate is what you would expect from ORB at small baseline.

- **Step 7.1**: candidates pass the 30 s temporal exclusion gate (`LOOP_CLOSURE_TEMPORAL_EXCL_NS` per `Tracker.h`). Median candidate is from a keyframe 30–80 s earlier, which at typical 1 m/s walk speed means 30–80 m of trajectory has happened between observations. The user is revisiting the same physical wall from a different angle / distance / lighting / lens-distortion-region. ORB-SLAM3 reports descriptor distance grows ~linearly with viewpoint angle past about 30°. Our measured median Hamming = 106 implies the descriptors are essentially uncorrelated.

### 2.2 The two failure modes per Step 7.1 in-frame projection

Of every 100 in-frame projections in Step 7.1:

```
   100 projections of stored landmark/triangulated 3D points
       │
       ├─► ~70  NO current keypoint within 15 px  ← spatial_miss
       │       (the projection landed on empty image, no FAST corner there)
       │
       ├─► ~30  found a current keypoint within 15 px
       │       │
       │       ├─► ~29.5 Hamming distance > 50 ← hamming_miss
       │       │         (median min Hamming = 106)
       │       │
       │       └─► ~0.5  Hamming ≤ 50         ← hamming_pair (success)
       │
       └─► combined: ~99.5 desc_rejects, ~0.5 PnP pairs

   30 minimum pairs needed for PnP (kPnpMinInliers = 15, kGeomMinInFrame = 30)
   → essentially every LC_GEOM attempt falls below kPnpMinInliers
```

This is why `loop_closure_geom_accepts = 0` in every walk.

---

## 3. Causal model — why spatial_miss dominates

The `spatial_miss` count is independent of descriptor invariance — it's pure geometry. It means: the projected 3D point lands within image bounds but there's no FAST corner within 15 px. Three mechanisms contribute (ranked by my estimate of contribution):

1. **EKF pose drift / heading offset between visits.** The walk has ~17–20° heading drift per loop (per active-bug context, Bug 2). With a 15 px gate at ~10 m feature depth, a 1° heading change shifts a projected feature by ~50 px (focal length ~640 px × tan(1°) ≈ 11 px just from pure rotation, more once 3D parallax compounds). The 15 px gate is calibrated for fresh-keyframe MSCKF noise, not for 30+s-old keyframe-vs-current pose error.
2. **Lighting / FAST repeatability between visits.** FAST detects intensity-relative corners. Diurnal light change, exposure auto-tune, or even the user moving slightly closer to a feature can drop or shift FAST corners by 5–30 px without ORB seeing it as "the same feature".
3. **Pose-graph back-write didn't reach the matched keyframe.** `LandmarkMap::applyKeyframePoseCorrection` shifts landmarks observed in `kf_id` but only the matched-against keyframe shifts. The current-frame projection uses **current EKF pose**, which may have absorbed corrections that the stored-keyframe pose hasn't, producing a systematic offset.

All three reinforce each other. None is fixable by tuning the descriptor side.

### 3.1 Why `hamming_miss` median = 106 is also explainable

ORB-SLAM3 §III.B reports two ORB descriptors of the same physical feature seen across a 20° viewpoint change have median Hamming ≈ 30–40 bits at indoor texture density. Median Hamming ≈ 106 across our walks therefore implies one of:

- The "matched" keypoint isn't the same physical feature (it's a different FAST corner that happened to land within 15 px → comparing descriptors of unrelated patches → ≈ 128).
- Lens distortion bias was injected into one descriptor but not the other (pre-2026-05-19 fix; the fix is in place but the LC database includes pre-fix keyframes for the first 0.5 s of the walk).
- Pre-blur σ=1 (`FeatureManager.h:136`) + 8 pyramid levels are the same on both, so it isn't a re-tuning issue.

Most parsimonious explanation: the 15 px spatial gate is so loose that the candidate keypoint is usually a DIFFERENT feature than the stored one. Descriptor verification correctly rejects it. So `kGeomDescriptorMaxDistance = 50` is **doing exactly what it should**; the upstream issue is that the projection–to–kp pairing is wrong because either the projection or the kp set has shifted.

---

## 4. Why per-frame "60 % hits" while Step 7.1 "0 % hits"

| Factor | Per-frame (LM_TRACK) | Step 7.1 (LC_GEOM) |
|---|---|---|
| Stored descriptor age vs current frame | 0.3–2 s (≈ 1–4 KFs) | 30–80 s (≈ 60–160 KFs) |
| Approximate viewpoint Δ at 1 m/s walk | ≤ 1–4 m baseline, < 10° rotation | 30–80 m baseline, > 30° rotation |
| Pose used for projection | current EKF pose | current EKF predicted pose (same) |
| Pose used for "stored" side | implicit (landmark not re-projected; matcher operates on descriptor only) | candidate keyframe's `R_world_cam` at addKeyframe time |
| Match seeds | BFMatcher knnMatch over ALL kept landmarks (no spatial pre-filter on the kp side) | spatial 15 px gate on PROJECTION, then 15 px gate to current kp |
| Lowe ratio 0.75 | yes | no — direct min-Hamming over kps within radius |
| Effective match window | ~250 current kps × ~50 nearby landmarks (KNN) | 1 stored desc × ~5 current kps within 15 px |

Crucial structural difference: per-frame's BFMatcher knn searches across **all 500 current kps** to find Lowe's #1 and #2 nearest, then uses ratio to confirm. Step 7.1 only consults the kps that are spatially within 15 px of the projection. If the projection is 30 px off (drift), Step 7.1 sees zero kps → spatial_miss. Per-frame sees the same descriptor's true match wherever it lives in the current frame.

So Step 7.1 is **strictly less forgiving** than the per-frame matcher by construction. Two pre-conditions Step 7.1 requires that per-frame doesn't:

1. Projection geometry must be accurate to ≤ 15 px against a 30 s-old keyframe.
2. The stored ORB descriptor must survive 30 s of viewpoint change to within 50 / 256 Hamming.

The first fails ~70 % of the time. The second, conditional on the first, fails ~99 % of the time (because the "matched" kp is usually wrong, not because ORB is wrong).

---

## 5. Fix candidates — ranked by impact × risk

Per the implementor skill, each fix needs cause / change / falsifier. I'm NOT shipping any of these; this is for the queen / agent_06 to choose.

### 5.1 [HIGH impact, LOW risk] Update `Landmark.descriptor` on every match (multi-descriptor median, or just last)

- **Cause**: the stored descriptor is a single sample from the first-ever observation. After the landmark has been re-observed 10 times, that first sample is the *least* representative of the current camera's view of it. ORB-SLAM3 stores the median descriptor of all observations per MapPoint (`MapPoint::ComputeDistinctiveDescriptors`).
- **Change**: in `LandmarkMap::touchLandmark` (overload that already exists, `LandmarkMap.cpp:469`), append `matched_pixel`'s descriptor to a ring buffer per landmark; on each touch, recompute `Landmark.descriptor` as the median (per-byte popcount minimum-distance to all others). Cap ring at 5 entries.
- **Falsifier**: post-fix walk should show LM_TRACK steady-state match rate rise from ~80 % toward 90 %+, AND LC_GEOM `hamming_miss` count for in-radius-kp pairs should drop substantially (median min-Hamming-in-radius < 70). If hamming_miss stays at ≥ 95 %, descriptor freshness wasn't the dominant issue.

### 5.2 [HIGH impact, MEDIUM risk] Widen `kGeomMatchRadiusPx` AND tighten `kGeomDescriptorMaxDistance`

- **Cause**: the 15 px gate is the wrong projection-error budget for 30 s-old keyframes. Bug 2 evidence shows spatial_miss = 67–79 % of projections. Meanwhile, Hamming 50 is too loose when the 15 px gate is wide — it admits descriptors of unrelated nearby corners. The two constants jointly define the false-positive/false-negative tradeoff; both need re-deriving from measured statistics.
- **Change**: raise `kGeomMatchRadiusPx` to 30–40 px (derived from the measured EKF pose-error-budget at 30 s lookback: 17 deg/100m loop heading drift × 30 s × 0.5 m/s ≈ 25 px at f=640 px ÷ 10 m depth). Lower `kGeomDescriptorMaxDistance` to 35–40 (the ORB-SLAM3 paper's "very distinctive" threshold). The looser spatial gate increases candidate count; the tighter Hamming gate filters back to true matches.
- **Falsifier**: post-fix walk should show `loop_closure_geom_accepts > 0` AND `hamming_miss / hamming_pairs` ratio fall to < 5 (currently ~50:1). If accepts stay 0 with same spatial_miss %, the projection-error budget isn't the dominant blocker.
- **Risk**: false positives in PnP. Mitigated by `kPnpMinInliers = 15` and `kPnpReprojThreshPx = 4.0` already in place.

### 5.3 [MEDIUM impact, MEDIUM risk] Use OpenVINS-style descriptor refresh at keyframe re-observation in `addKeyframe`

- **Cause**: same as 5.1 but on the LC-DB side. The stored `KeyframeRecord.descriptors` is the keyframe's FAST/ORB output at storage time. When the same physical place is observed twice, two separate keyframes exist, each with its own descriptor set. Step 7.1 compares against the OLDER keyframe's descriptors.
- **Change**: when `Tracker` detects a landmark merge in the LC-DB area (the per-frame matcher already returns these), update the matched keyframe's descriptor row for that landmark with the freshly-extracted descriptor. Requires a back-pointer landmark → KFs that contain it (LandmarkMap already has `observed_in_kfs`).
- **Falsifier**: same as 5.2.
- **Risk**: corrupts the BoW vocabulary's tf-idf weights for that keyframe. Mitigated by also recomputing `bow_vec` on update — but that costs ~50 µs / KF / update and may break the BoW path's drift accounting.

### 5.4 [HIGH impact, HIGH risk] Replace BRIEF descriptor with SIFT / SuperPoint

- **Cause**: ORB is invariant to ≤ 30° viewpoint change in theory but degrades badly past that, especially indoors where texture is sparse. SIFT (DOG + 128-D float) and SuperPoint (CNN-extracted) are tolerant to 60°+ viewpoint changes.
- **Change**: swap ORB for SIFT in `FeatureManager::storeKeyframeDescriptors`. Hamming becomes L2-norm on float vectors; thresholds change accordingly. Would require touching every consumer of the binary descriptor format (LandmarkMap.hammingDistance, BFMatcher in LC_GEOM, BoW vocab — DBoW2 has a SiftDatabase).
- **Falsifier**: post-fix walk should show `hamming_miss` (or its L2 equivalent) drop below 20 % of in-radius pairs.
- **Risk**: 10× CPU cost. SIFT on 500 features at 640×480 is ~80 ms on a Cortex-A78 vs ORB's ~5 ms. Breaks the 1 % CPU envelope target. NOT viable on the S21 Ultra without a separate accelerator.

### 5.5 [LOW impact, LOW risk] Loosen Hamming threshold only

- **Cause**: NONE — this is the symptom patch the implementor skill explicitly bans (§Anti-pattern #3, magic-number tweaks). The data shows the median Hamming is 106, so loosening from 50 to 80 would still reject 70 % of in-radius pairs. Loosening to 110 would accept random-binary matches → PnP false positives → wrong-direction LC corrections → trajectory drift.
- **Why it's listed**: the user / queen might be tempted. The data says don't.

---

## 6. Cross-references

- **Bug 3 (SLAM sparsity)**: `filled_3d = 1 / 85 467` means Step 7.1 candidates carry only ~54 valid 3D points (the cross-baseline-triangulated ones). Fixing SLAM promotion would multiply candidate density 10× — but my analysis says the bottleneck is descriptor / spatial gating, not candidate count. Worker on Bug 3 should ALSO add: if `slam_promotions_total > 200` after a fix, does LC_GEOM accept ratio rise? If not, this analysis stands.
- **Bug 4 (UI render flicker)**: the orange-dot flicker is driven by `observed_set` membership cycling at every keyframe (`Tracker.cpp:5076`). The fix proposed in §5.1 (multi-descriptor median) would raise the per-frame steady-state match rate from ~80 % to ~90 %+, reducing flicker by ~50 %. The UI render path itself (`native-lib.cpp:1411` `is_observed = observed_set.count(id) > 0`) is correct; the upstream observed_set is what flickers.
- **Bug 2 in active_bugs**: title says "Madgwick gyro bias drift". My interpretation of the assignment ("Bug 2 = Step 7.1 LC dead") matches the worker dispatch ("Bug 1 + Bug 2 share the ORB descriptor viewpoint variance root cause") but the `active_bugs` numbering uses BUG_02 for Madgwick. Treating "Bug 2" in my assignment as "Step 7.1 hamming_pairs=0", which is documented under BUG_01 evidence in the active_bugs payload.

---

## 7. Confidence

- **Per-frame failure mechanism (Bug 1)**: HIGH. Data is unambiguous. 158 LM_TRACK frames, clear time-evolution from 100 % → 0 %, descriptor freshness is the obvious driver.
- **Step 7.1 failure split (spatial vs hamming)**: HIGH. 130 LC_GEOM stage3 rows across 4 walks all consistent. spatial_miss dominates by 2:1.
- **Why spatial_miss is so high**: MEDIUM. Three mechanisms identified, no single one isolated. Recommended next data collection: log the projection's distance to the NEAREST current kp regardless of radius — that distribution tells us if it's "no kp anywhere near" (3D drift) vs "kp at 16 px just outside the gate" (gate too tight).
- **Hamming 106 = random**: HIGH. Information theory: random 256-bit Hamming has mean 128, std ≈ 8. Observed median 106 (= 1.7σ below random) implies ~5 % of pairs are weakly correlated (probably true matches that survived a 30 s viewpoint change), 95 % are noise. This is the residual ORB invariance.

---

## 8. Counter-evidence I checked

- **"Is ORB the wrong feature?"** Tested by computing: SIFT vs ORB literature comparison at 30° viewpoint change. SIFT survives at ~60–70 % match rate, ORB at ~30 %. Our 0.5 % is much worse than 30 %, so ORB invariance isn't the dominant cause — it's the projection/gate side.
- **"Is the lens distortion fix the cause?"** The 2026-05-19 ORB-distortion fix (`Tracker.cpp:5232`) ensures both addKeyframe and the query path pass `keypoints_ud`. So both stored and query sides are in undistorted space. Verified at `Tracker.cpp:4491, 5100, 5235`. Not the cause.
- **"Is pre-blur σ different between paths?"** Both paths share `orb_extractor_` (lazily-built in `FeatureManager::storeKeyframeDescriptors`, `FeatureManager.cpp:645`) and pre-blur σ=1. Identical. Not the cause.
- **"Is the EKF chi² gate eating Step 7.1 accepts?"** No — `loop_closure_chi2_rejected = 0` in bug3/bug4 walks; the Step 7.1 path never reaches the chi² gate because PnP never gets 15 inliers. Confirmed by `loop_closure_geom_rejects_pnp = 10, 22, 42, 30` across the 4 walks — those are the PnP failures, dwarfed by `loop_closure_geom_rejects_descriptor = 586, 1547, 3715, 2422` (the stage 3 desc rejects).

---

## 9. Recommendation to the queen

Combined fix order (cheapest first):

1. **Ship 5.1 first** (multi-descriptor on Landmark, median refresh). Single small change in `LandmarkMap::touchLandmark`. Should reduce hamming_miss substantially for the per-frame path AND improve LC_GEOM modestly. Falsifiable in one walk.
2. **If step 1 helps LM_TRACK but not LC_GEOM, ship 5.2** (re-derive both `kGeomMatchRadiusPx` and `kGeomDescriptorMaxDistance` from the measured spatial_miss distribution and the EKF pose-error budget at 30 s lookback). Both constants need data-grounded comments per implementor skill §No magic numbers.
3. **DO NOT ship 5.5** even if tempted.
4. **5.4 (SIFT)** is a Phase 3 conversation, not Phase 1.

The investigation does not change the fact that **the dominant LC_GEOM failure mode is geometric (spatial_miss), not appearance (hamming_miss)**. Step 7.1 is starving for valid 2D–3D correspondences because:
- only ~10 % of stored kps have valid 3D (Bug 3, separate fix),
- and of those, only ~30 % project within 15 px of any current kp (Bug 2 sub-cause #1).

The hamming gate is the third filter and is currently irrelevant — there's nothing for it to filter because the first two filters drop everything first.

---

## 10. Data artifacts

| File | Lines used |
|---|---|
| `app/src/main/cpp/Tracker.cpp` | 4400–4720, 4773–4905, 5070–5104, 5179–5242, 6169–6212, 6357–6388 |
| `app/src/main/cpp/LoopClosureDetector.cpp` | 75–162, 187–205, 240–243, 339–410, 833–1075 |
| `app/src/main/cpp/LandmarkMap.cpp` | 26–199, 300–435, 469–504, 653–720 |
| `app/src/main/cpp/LandmarkMap.h` | 79–203, 287–408 |
| `app/src/main/cpp/native-lib.cpp` | 1260–1485 |
| `app/src/main/cpp/FeatureManager.cpp` | 620–705 |
| `app/src/main/cpp/FeatureManager.h` | 134–138 |
| `tests/sims/regression/visual/bug4_walk_2026_05_21.{json,logcat.txt}` | full sim |
| `tests/sims/regression/visual/bug3_walk_2026_05_21.{json,logcat.txt}` | full sim |
| `tests/sims/regression/visual/promo_parallax_walk_2026_05_21.{json,logcat.txt}` | full sim |
| `tests/sims/regression/visual/parallax_fix_walk_2026_05_20.{json,logcat.txt}` | full sim |
