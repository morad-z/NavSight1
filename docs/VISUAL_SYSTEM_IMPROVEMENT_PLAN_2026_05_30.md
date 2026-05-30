The central thesis is confirmed at the source: `Tracker.cpp:1746` (`ekf_.setPosition(global_t_)`) overwrites the EKF position before propagation, and the EKF→dot mirror at `:6266-6274` is commented out (SUPERSEDED 2026-05-16 Tier-1 revert). The reports are accurate. Here is the document.

---

# NavSight Visual System Improvement Plan
**Decision document — drift & accuracy toward 1 km out-and-back @ ≤5% (scooter + walking), GPS jammed**
Date: 2026-05-30 · Status after Fix A (validated) + Fix B (awaiting re-walk) · **HEADING IS OFF-LIMITS throughout**

---

## 1. TL;DR

**The single biggest lever is scale (K), not geometry.** Every metre the dot moves is `disp = K × visual_rel_speed × dt` (`Tracker.cpp:3641`). On the validation run K was **1.70× too high** (run K=1414 vs expected ~831) and on the walk **~0.50× too low** — that alone is a 50–70% distance error, which by itself blows the 5% budget. One shared K cannot serve walk (~1340) / run (~831) / scooter simultaneously, and K can only recalibrate at a stop, which scooter rides rarely provide. **The second, structural truth: almost none of the SLAM/MSCKF/EKF machinery reaches the dot.** Every frame begins with `ekf_.setPosition(global_t_)` (`Tracker.cpp:1746`), and the EKF→dot writeback is commented out (`:6266-6274`, confirmed in-source). So 1,282 accepted MSCKF landmark updates, 14 bundle-adjustment solves, and all SLAM inverse-depth updates correct `p_G_` for exactly one frame and are then discarded — they cost battery and produce **zero** user-visible benefit. Loop closure is the only EKF correction wired through to the dot (`global_t_ += delta_p`, `:7712-7719`), and it fired **0 times** on every validation clip — blocked by a 30 s temporal-exclusion gate against recordings that are all under 28 s, plus empty `pts3d_world` (no SLAM promotions). **Honest framing:** the user-facing system today is essentially KLT → looming/depth-flow speed × K × Madgwick-heading, with no working position-correction mechanism. To hit 1 km/5% you need (a) K right per-mode and fresh without stops, and (b) at least one correction channel — loop closure or MSCKF-via-delta-mirror — actually reaching the dot. Everything below is ranked against those two facts.

**Caveats on the evidence.** These conclusions are grounded in three on-device validation recordings (walk 18.6 s, run 7.9 s, U-turn 27.4 s) plus the replay harness. The recordings are **too short to exercise loop closure** (the 30 s gate is a measurement artifact, not a real-route failure — a 1 km ride lasts ~10 min and *will* escape the gate on the return leg). GPS is jammed so there is **no external ground truth**; absolute-scale claims rest on tape-measured distance and self-consistency. The replay harness runs at ~18.7 fps vs ~23 fps on device (only ~1.2× pessimistic — do not over-discount it, but it is mildly pessimistic). **Treat every "expected gain %" below as a directional estimate, not a measured number.**

---

## 2. Per-feature inventory (grouped by subsystem)

Status legend: **working** = does its job and reaches the dot · **partial** = runs but contribution is limited/capped · **dead-bypassed** = computes correctly but the result never reaches the dot (or is gated off entirely).

### 2A. Visual front-end (KLT / essential matrix / undistortion)

| Feature | What it does / should do | Status | Why limited | Drift role + proof |
|---|---|---|---|---|
| **recoverPose essential matrix** (`TrackKLT.cpp:91-128`, `Tracker.cpp:2742`) | Per-frame (R, t-direction) from 5-pt RANSAC; feeds depth-flow speed + EKF rotation. Should give a usable pose every frame. | **partial** | **Forward motion is the textbook degenerate case** — baseline along the optical axis, epipolar lines converge at image centre, near-zero central parallax. | Walk: `visual_translation_degenerate_total=195/397` (49%); only ~12.6% of frames yield a reliable pose. Run (body sway gives lateral baseline): degeneracy 6.7%. Each degenerate frame is a displacement sample lost from the precise path, falling back to noisier looming. |
| **KLT optical-flow** (`TrackKLT.cpp:16-88`) | IMU-predicted pyramidal LK + forward-backward check; supplies points to *all three* estimators. | **working** | Window grows from **gyro only, not translation** → fast straight scooter (high translation, low gyro) can overrun the 21 px window. FoE region not masked → near-zero-flow points waste budget. | KLT quality gates whether K self-calibrates at all (`depth_flow_calib_updates` 41/25/63). `docs/VISUAL_SYSTEM_AUDIT_2026_05_30.md §5` confirms the window bug. |
| **Parallax / motion gates** (`MIN_PARALLAX_PX=0.8`, `kVisualMinParallaxRad=0.01`, `Tracker.cpp:2893-2902`) | Outer gates skip static/pure-rotation frames; inner gate flags forward-motion degeneracy. | **partial** | Inner gate is a *symptom check* retrofitted from the SLAM triangulation angle — it correctly blocks the worst frames but also blocks frames that PnP-with-depth or homography could serve. | Marginally tight for slow walking (~8.4 px/frame expected vs 10 px gate). The gate is fine; the underlying geometry is the real problem. |
| **Lens undistortion** (`LensCorrector.cpp:61-81`) | 8-coeff rational model before all geometry. Should make points consistent with pinhole K. | **working** | No per-frame residual diagnostic. If the camera *stream* resolution differs from the *calibration* resolution, intrinsics scale but distortion coeffs do not → silent peripheral error at speed (features spread full-width). | `midas_fused=0`, `midas_bailout_few_pts3d=4` on walk — triangulation failing; *may* be partly distortion-related, currently unprovable (no instrumentation). |

### 2B. Metric scale + speed observers — **the headline subsystem for the 5% goal**

| Feature | What it does / should do | Status | Why limited | Drift role + proof |
|---|---|---|---|---|
| **accel-K self-calibration (Fix A)** (`Tracker.cpp:1202-1239`, `:8248-8289`) | After each ZUPT, `k_obs = accel_dist / visual_rel_dist`, EMA α=0.05. Should track absolute scale for every mode. | **working** (validated) | **One shared K can't fit walk/run/scooter**; EMA needs ~20 updates to converge (mode switch keeps stale K for >10 s); recalibration needs a **stop** every ~2.5 s — scooter rarely stops. | THE load-bearing number. Run K=1414 vs ~831 = 1.70× → 70% distance error. Fixing K accuracy + adaptation speed is the precondition for everything else. |
| **updateExpansionSpeed (looming)** (`Tracker.cpp:8080-8395`) | Heeger-Jepson de-rotated radial flow → forward speed for the degenerate forward case. Post-Fix-B it drives the dot when recoverPose fails. | **working** (now dominant) | Same per-mode-K problem; **FoE pinned to EKF heading** (`:8111`) so any heading error biases speed *and* direction (coupling we can't fix without touching heading). | Walk: `loom_used=174/301` (58% of dot advance). Without it the dot freezes on ~65% of walk frames. Scale accuracy = K accuracy. |
| **updateDepthFlowSpeed** (`Tracker.cpp:1028-1297`) | Depth-weighted per-point scale from MiDaS disparity + t_vo; the *more accurate* path when recoverPose works. | **partial** | Only fires when `pose_valid` → frozen ~65% of walk; its K can't update on degenerate frames, so depth-flow K and looming K diverge. `midas_fused=0` (affine path disabled). | Walk `df_updates=245` but `heading_proj=35` (dot advanced from this path only 35×). The right path for scooter/run where parallax is good. |
| **ScaleEstimatorVI** (VINS-style) (`Tracker.cpp:1167-1200`, `:3298-3404`) | Joint (scale, v0) solve from visual-t + IMU across keyframes. Should give stop-free scale for scooter. | **dead-bypassed** | **Proven dead end offline** (`scripts/test_scale_estimator.py`): errors-in-variables dilution — OLS attenuates s→0 (1% direction noise → s drops 2.5×); `relσ` gate can't catch confidently-wrong s. Output goes to dead `scale_fuser_` anyway. | Zero contribution. Two layers of dead (diluted solve + disconnected output). |
| **Legacy ScaleFuser + MiDaS affine** (`Tracker.cpp:184-560`) | 1-D Kalman scale; VI-Depth affine for per-point metric depth. Should remove the single-K problem entirely (scene self-calibrates). | **dead-bypassed** | **Circular anchor poison**: affine target = `1/(z_vio × scale_fuser)` and `scale_fuser` is stuck at the rejected PDR seed 0.10 → targets 5–10× too small. Speed path also hard-disabled (`aff_valid_now=false`). | `midas_fused=0` all 3 recordings. But on run/uturn the affine *fit* converges (inlier 0.96–1.00) — the machinery works, the **anchor** is poisoned. High-value long-term path. |

### 2C. SLAM feature subsystem

| Feature | What it does / should do | Status | Why limited | Drift role + proof |
|---|---|---|---|---|
| **SLAM promotion** (`Tracker.cpp:4195-4561`) | Promote stable KLT features to EKF landmarks; precondition for BA, LC PnP, position anchoring. | **dead-bypassed** | Axial-motion triangulation degeneracy: parallax gate (>1°) rejects nearly every walk candidate (15 cm @ 3 m ≈ 0.3°); RMS gate can't save what parallax missed (degenerate depth lies on all rays); MiDaS rescue not firing. | Walk: `slam_promo_candidates=42962`, `promotions=0`. Gating dependency for the **entire** downstream SLAM chain. |
| **Inverse-depth landmarks / UpdaterSLAM** (`UpdaterSLAM.cpp:13-123`) | Per-frame 2-DOF reprojection update → continuous position anchoring. | **dead-bypassed** | Starved (0 promotions → loop runs 0×); even if it ran, `setPosition` discards the correction next frame. | `slam_live_updates_fired=0` everywhere. More valuable than LC for continuous correction *if* wired. |
| **LandmarkMap** (`LandmarkMap.h`, `Tracker.cpp:5475-6004`) | Persistent metric map; MSCKF local-map tracking → position updates; supplies `pts3d_world` to LC PnP. | **partial** | `pts3d_world` mostly NaN (no promotions + weak inter-KF triangulation). MSCKF updates *do* fire (`landmarks_msckf_accepted=1282`) but `setPosition` discards them. | **Highest-impact component for the return leg if wired** — the MSCKF channel is already computing 1,282 chi²-gated corrections that vanish. |
| **WindowedBA** (`WindowedBA.cpp:151-563`) | Local LM bundle adjustment refining landmark depths. | **partial** | Runs + converges (14/14 accepted, avg 1.3 ms) and writes back to the map, but refined EKF positions are discarded by `setPosition`. | `ba_solves=14`, `landmarks_refined=104`. Second-tier gain behind the re-wiring. |

### 2D. MSCKF backbone

| Feature | What it does / should do | Status | Why limited | Drift role + proof |
|---|---|---|---|---|
| **UpdaterMSCKF (lost-feature)** (`UpdaterMSCKF.cpp:21-322`) | Null-space reprojection updates → bounds clone-to-clone drift to ~1-2 cm/m. | **partial** | Computes + applies to `p_G_` correctly, then discarded next frame. Secondary: chi² gate too tight (`chi2_multiplier=1.5` vs OpenVINS 5.0) → 52% LandmarkMap rejects when scale is uncertain. | `msckf_update_lines=684`, but 0 lasting effect. If wired, **5–15× drift reduction** on good-parallax scooter. |
| **Clone / sliding window** (`EKFState.cpp:801-974`, `MAX_CLONES=15`) | 15-clone window for multi-view geometry; Schur marginalization. | **working** | Sound. Information never escapes to the dot (not a window bug). Keyframe rate ~2 Hz is coarse for scooter baselines. | `fej_clone_miss=0`. Enables MSCKF triangulation accuracy. |

### 2E. Loop closure + keyframes — **the only correction channel wired to the dot**

| Feature | What it does / should do | Status | Why limited | Drift role + proof |
|---|---|---|---|---|
| **DBoW2 appearance LC** (`LoopClosureDetector.cpp:506-519`) | BoW similarity → PnP verify → absolute-pose correction. The drift-killer at the 500 m turnaround. | **dead-bypassed** | **30 s temporal exclusion** (`LOOP_CLOSURE_TEMPORAL_EXCL_NS`, `Tracker.h:936`) eliminates every keyframe in clips <28 s — function exits at `max_id=-1` before scoring. Secondary: empty `pts3d_world` → PnP would fail anyway. | `loop_closure_rejects_low_score=0` AND `rejects_pnp=0` (never reaches scoring/geometry). **Single highest-leverage drift correction for out-and-back.** |
| **Geometric LC (Step 7.1)** (`LoopClosureDetector.cpp:884-911`) | Direction-invariant spatial+descriptor+PnP — built *for* U-turn/out-and-back. | **dead-bypassed** | Same temporal gate. Secondary: search radius pinned at 2 m floor because `setPosition` collapses P → `sigma_p_xy→0`. | `geom_rejects_no_candidate=31/10/40` = 100% of attempts. |
| **Keyframe creation + 3D population** (`Tracker.cpp:4788`) | One KF every ~0.5-1 s with dense `pts3d_world` for PnP. | **partial** | `pose_valid` gate too strict for forward walking (degenerate); `filled_3d≈0` (no promotions); positions anchored to scale-compressed `global_t_`. | `kf_count_in_db=35`; `pts3d_world` ≈ all-NaN. Upstream dependency for **both** LC paths. |
| **Pose-graph / updateAbsolutePose** (`Tracker.cpp:7382-7850`) | Apply LC correction to dot + redistribute across KFs. | **dead-bypassed** | Entirely downstream of LC accept (0 accepts). Wiring is correct (`global_t_ += delta_p` present; Madgwick nudge correctly gated on `!isMagActivelyFusing()` — heading-safe). Back-write to LC DB marked incomplete (`:7795-7801`). | `loop_closure_corrections_applied=0`. The output stage — works *if* LC fires. |

### 2F. EKF backbone + zero-velocity

| Feature | What it does / should do | Status | Why limited | Drift role + proof |
|---|---|---|---|---|
| **15-DOF EKF + "architecture runs backwards"** (`Tracker.cpp:1746`, `:6266-6274`) | EKF should own position; dot should read *from* `p_G_`. | **partial** | Inverted: `setPosition(global_t_)` every frame makes the EKF a downstream passenger. The 2026-05-16 Tier-1 revert did this because making EKF autonomous drifted quadratically (v31: 75 m on 10 m walked). | The reason no MSCKF/BA correction sticks. The documented proper fix is a **P_pp drift-rate floor**, not removing `setPosition`. |
| **ZUPT / ZRUPT** (`Tracker.cpp:2394-2603`, `EKFState.cpp:642-790`) | Zero velocity at stops; keep b_g tight; **open the K-calibration window**. | **working** | None. The one solidly-working drift bounder. | Walk `zrup_fired=101` → the reason `depth_flow_calib_updates=41`. Every stop = one K recalibration. |
| **EKF covariance P_ / UI ring** (`Tracker.cpp:1307-1332`) | Honest confidence ring; grow with distance, shrink on correction. | **partial** | `setPosition` collapses P_pp → ring shows **false confidence** (P[12,12]+P[13,13]≈0.0009 m² while 5-15 m of real drift accumulated). | Documented at `Tracker.cpp:1549`. Same root cause as the architecture inversion. |
| **propagateIMU + gravity alignment** (`EKFState.cpp:238-615`) | High-rate dead-reckoning prediction; gravity anchors roll/pitch. | **working** | Mathematically correct (post Forster + Phi off-diagonal fixes). Position part overwritten each frame so contributes nothing along-track; v_G persists and feeds ZUPT. | `velocity_clamped=0` (walk/run), correct on U-turn. |

---

## 3. The essential-matrix deep-dive (your headline ask)

**The problem in one sentence:** for forward motion the camera translates along its optical axis, the focus-of-expansion sits at image centre, epipolar lines converge there, and the 5-point essential matrix becomes singular — RANSAC finds inliers consistent with *almost any* (R, t). Result: 49% of walk frames are flagged degenerate, only ~12.6% give a reliable pose. This is geometry, not a tuning bug. You cannot fix it by relaxing a threshold; you must change the estimator.

**The four candidate fixes, ranked for our forward-axis case:**

### #1 — PnP-with-MiDaS-depth (3D-2D) — *the single change that most helps forward motion*
Lift each previous-frame KLT point to 3D using its MiDaS disparity (`Z = 1/disp_raw`, exactly what `updateDepthFlowSpeed` already samples), then `cv::solvePnPRansac` against the current 2D positions. **This is the complementary geometry to the essential matrix:** where the essential matrix needs *lateral baseline* (fails forward), PnP needs *depth variation* — and forward motion *creates* depth variation (near points stream past faster than far points). PnP does **not** degenerate under forward motion, returns a **metric** (R, t) directly (no separate scale K step), and needs as few as 4 points so it survives a 0.1 inlier ratio.

- **Why it's #1:** it attacks the exact failure mode (forward axial motion = our dominant walk + scooter-straight case) and every ingredient already exists in-tree (`sampleMidasRawDisparity`, undistorted `prev_ud/next_ud`, intrinsics K). The new function mirrors `updateDepthFlowSpeed`'s depth loop but calls `solvePnPRansac` instead of the linear scalar solve. Expected to convert ~49% degenerate walk frames and ~56% degenerate U-turn frames into usable pose frames.
- **Heading risk: none.** PnP outputs a camera-frame pose; only its translation magnitude would replace `depth_flow_speed_mps_`. The dot advance still projects onto Madgwick heading (`Tracker.cpp:3721`).
- **Effort: medium.**

### #2 — IMU-prior 2-point RANSAC inside geometricVerification
Use the known inter-frame IMU rotation (`imu_delta.deltaR`, already used for KLT prediction) to remove rotation from the essential matrix, reducing it to a pure-translation 2-point problem. Essential-matrix degeneracy is fundamentally about the translation:rotation ratio; removing the rotation greatly improves conditioning for forward motion, and a high-quality IMU constrains rotation far better than noisy pixel matches. This is ORB-SLAM3's standard degeneracy step. Expected +20-30% `verification_ok` on slow walks. **Complements** #1 (improves the rotation R that feeds the EKF; PnP gives the metric t). **Heading risk: none** (stays inside geometricVerification). **Effort: medium.**

### #3 — Homography vs essential-matrix model selection (ORB-SLAM H/E)
Run `findHomography` alongside `findEssentialMat`, compare symmetric-transfer scores (`R_H = score_H/(score_H+score_E)`), pick the better model per frame. When the scene is locally planar (pavement, building facade — common urban walking/scooter), the essential matrix is ill-conditioned but the homography is well-defined. Rescues an estimated 10-20% of degenerate frames with clean planar structure. **Smaller and complementary** to #1 (planar vs depth-layered scenes). **Heading risk: none.** **Effort: medium.**

### #4 — Feature distribution: FoE-exclusion mask + translation-based KLT window
Mask a ~30 px circle around the projected FoE in `goodFeaturesToTrack` (near-FoE points have near-zero, ambiguous flow — maximum essential-matrix ambiguity *and* looming blow-up), and add a translation term to the adaptive KLT window so fast straight scooter motion doesn't overrun the 21 px window. Cleanup that raises effective inlier ratios 5-10 pts and prevents track loss at speed. **Both low-effort, heading-risk none.** Do these alongside #1 — they make #1's input cleaner.

**Verdict:** implement **#1 (PnP-with-monodepth)** first — it is the only option that both eliminates the degeneracy *and* delivers metric scale directly, killing two birds (the recoverPose gap *and* feeding the scale chain). Add **#4's two cheap cleanups** in the same pass. Hold #2/#3 as follow-ups if PnP alone doesn't lift the pose-valid rate enough.

---

## 4. Prioritized roadmap (drift impact ÷ effort)

> Two categories. **Category A** makes a *dead* feature reach the dot (largest potential, requires care). **Category B** improves a *live* feature (lower ceiling, lower risk). All heading-risk tags verified against the source; **nothing here touches `scalar_heading_` or Madgwick.**

### Tier 1 — Do these first (tied directly to 1 km/5%)

| # | Change | Cat | Effort | Expected gain | Heading risk | Ties to goal |
|---|---|---|---|---|---|---|
| **1** | **Per-mode K (walk/run/vehicle) + adaptive EMA** — step-detector gates which K slot to use/calibrate (`imu.getStepInfo()` already distinguishes cadence); α=0.3 on mode-switch, 0.05 stable. | B | medium + low | Kills the **1.70× run / 0.50× walk** error → residual ~5-15% (IMU bias), within reach of 5%. Mode-switch latency ~10 s → ~2 s. | none | **THE distance lever.** Without it the 5% budget is impossible regardless of corrections. |
| **2** | **PnP-with-monodepth** replacing the 2D-2D essential matrix for forward motion (Essential-matrix deep-dive #1) + FoE mask + translation KLT window. | B | medium | Converts ~49% degenerate walk frames to usable metric poses; fills the largest gap in scale continuity on scooter-straight. | none | More verified frames → more K calibration + less reliance on noisier looming. |
| **3** | **Loop closure unblock: cut geometric temporal-exclusion 30 s → 10 s + MiDaS-seed `pts3d_world` at keyframe creation + feed LandmarkMap candidates into the geometric path + drift-inflated search radius.** | A | low ×3 + medium | The **only** position-correction channel wired to the dot. Enables ≥1 LC accept on the return leg → bounds final error from 50-100 m to ~10-30 m. | low (correction nudges Madgwick, already gated on `!isMagActivelyFusing()`) | **Directly delivers the 5%.** One accept at the 500 m turnaround snaps accumulated drift. |

**Why these three:** #1 fixes the dominant error (scale). #2 widens the verified-frame supply that feeds #1's calibration and reduces noisy-looming reliance. #3 is the one channel that can *bound* return-leg drift and is already correctly wired through to the dot — it just needs its upstream (temporal gate + `pts3d_world`) unblocked. Together they attack scale (continuous) + correction (at revisit), which is exactly the two-part requirement from the TL;DR.

### Tier 2 — High value, more involved

| # | Change | Cat | Effort | Expected gain | Heading risk |
|---|---|---|---|---|---|
| 4 | **MSCKF/LandmarkMap delta-mirror to `global_t_`** — after `applyLandmarkObservations`, `dp = p_G_after − global_t_; if(norm<0.5) global_t_ += dp` (mirrors LC `:7712`). Routes the **1,282 already-computed** corrections to the dot. | A | medium | Gradual along-track correction; ~10-30% return-leg drift reduction once landmark depths are decent. | none |
| 5 | **Un-poison the affine: seed `scale_fuser_` from calibrated accel-K on each ZUPT, then re-enable `aff_valid_now` gated on inlier≥0.5.** Removes the single-K problem entirely (scene self-calibrates per-point). | A→B | low ×2 | Eliminates per-mode-K *and* the stop-dependency in one step. Affine fit already proven to converge (inlier 0.96-1.00) — only the anchor is wrong. | none |
| 6 | **Continuous-motion K without stops** — depth-flow K recalibration on hard braking (trusted IMU Δv / depth-flow Δspeed_rel), the scooter-critical path. | B | medium | Breaks the stop-dependency that blocks scooter K freshness; ~2-3× less K staleness over a 1 km ride. | none |
| 7 | **SLAM promotion via MiDaS-only seed for baseline-starved candidates** (extend the existing rescue to fire on *all* low-baseline candidates, not just RMS>1.5). Unblocks BA + LC PnP + UpdaterSLAM. | A | low | Rescues ~41% baseline-rejected candidates; gating dependency for the whole SLAM chain. | none |

### Tier 3 — Structural / long-horizon (do not start before Tier 1-2 land)

| # | Change | Cat | Effort | Note | Heading risk |
|---|---|---|---|---|---|
| 8 | **EKF position re-wiring (P_pp drift-rate floor, remove `setPosition` overwrite)** — the "proper long-term fix" named at `Tracker.cpp:1744`. Makes the EKF authoritative so *all* MSCKF/BA corrections persist. | A | high | Highest ceiling but **highest regression risk** — the 2026-05-16 revert exists because the naive version drifted 75 m/10 m. Do delta-mirror (#4) first as the low-risk version; only attempt this with the replay-harness A/B falsifier (log P[12,12] should grow to O(drift²)). | low |
| 9 | **Learned-inertial velocity fallback (TLIO/RNIN-VIO recipe)** — ~4 M-param net on SenseINS (already downloaded) → metric velocity at 30 Hz, no stop needed. The real scooter scale answer. | B | high | ~2 wk + data. Cheap first step: run pretrained model offline on a recorded walk before any build. | none |
| 10 | **Sim3 pose-graph LC** — absorbs *scale* drift (not just pose) across the loop. The ORB-SLAM3 sub-1% method. | A | high | Best long-term distance fix; large refactor. Only worth it after LC fires at all (#3). | low |

**Do-NOT-touch list (flagged for the heading rule):** the FoE is pinned to EKF heading in looming (`Tracker.cpp:8111`) and the dot projects speed onto Madgwick heading — a heading error appears twice. **Do not** "fix" this by adjusting heading; it is a structural limit we accept. ScaleEstimatorVI's TLS rewrite is **not** recommended near-term (proven dead-end class of problem; spend the effort on #5's affine instead).

---

## 5. What to measure on the upcoming walk (confirm Fix B + decide next step)

Fix B decoupled the dot from `pose_valid` (falls back to looming when recoverPose fails). The validation walk above predates Fix B (it showed the 0.50× undershoot with the dot freezing on degenerate frames). **This walk's job is to confirm Fix B recovers advancement and to pick between Tier-1 #1 (per-mode K) and #2 (PnP) as the immediate next step.**

**Confirm Fix B worked (dot no longer freezes):**
1. **Tape-measure the route** before walking (no GPS — this is your only ground truth). Walk a known distance (e.g., 50 m out, 50 m back).
2. **VIO distance vs tape:** target the dot now reading ≳0.80× of true (was 0.50×). If still ~0.50×, Fix B did not route looming to the dot — check `loom_used` rose and `heading_proj` is no longer ~35.
3. **`depth_flow_frozen` fraction** should drop well below the prior ~72%/65%. Looming should now carry the degenerate frames.

**Decide the next step — pull these counters from logcat:**
4. `depth_flow_calib_updates`, `depth_flow_calib_K_range`, `K_final` — is K stable or swinging? A wide K_range on a single-mode walk argues for #2 (cleaner pose frames → tighter K) before #1.
5. `visual_translation_degenerate_total` / total frames — confirms the ~49% degeneracy persists on device (it's the case for #2/PnP).
6. `loom_used` vs `loom_updates`, and `df_updates` — which speed path actually drove the dot? If looming dominates >55%, PnP (#2) has the most headroom.
7. **U-turn return overlap (proxy for LC readiness):** does the return-leg dot retrace the outbound path, or diverge? Divergence = scale/heading; sideways offset = cross-track (map-matching territory, separate work).

**Honesty checks (do not skip):**
8. **Run the same walk through the replay harness** (`scripts/analyze_replay_csv.py`) and compare to on-device. If they disagree by >1.5×, the harness has a fidelity gap (cadence ~18.7 vs ~23 fps) — trust the device number for absolute scale, the harness for relative A/B.
9. **Walk long enough to test LC later** — these decisions about Tier-1 #3 (loop closure) **cannot** be validated on an <30 s clip. Plan a ≥3-minute out-and-back specifically to exercise the temporal gate, or the LC work stays unvalidated.
10. **Covariance sanity:** log `P[12,12]+P[13,13]`. If it reads ~0.0009 m² after meters of walking, the UI ring is lying (known issue) — confirms the P_pp-floor work (#8) is needed but is *not* a Fix-B regression.

**Bottom line for the walk:** if VIO distance reads ≳0.80× tape and the dot advances through degenerate frames, **Fix B is confirmed** → proceed to Tier-1 #1 (per-mode K) as the highest drift-per-effort lever. If distance is still low *and* `visual_translation_degenerate` is high, prioritize **#2 (PnP-with-monodepth)** first, since looming-only scale is the bottleneck.

---

**Files of record:** `app/src/main/cpp/Tracker.cpp` (`:1746` setPosition overwrite — verified; `:6266-6274` dead mirror — verified; `:7712-7719` LC delta-mirror; `:3641` dot scale; `:8111` FoE-heading coupling), `Tracker.h:936` (30 s LC gate), `LoopClosureDetector.cpp:506-519/884-911`, `UpdaterSLAM.cpp:13-123`, `ScaleEstimatorVI` / `scripts/test_scale_estimator.py` (dead-end proof), `docs/VISUAL_SYSTEM_AUDIT_2026_05_30.md` (§5 KLT window, §7 scooter logcat checklist), `scripts/analyze_walk_run_validation.py` + `tests/cpp/sims/val_2026_05_30/` (the three recordings).