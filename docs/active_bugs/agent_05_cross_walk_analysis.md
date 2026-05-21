# Agent 5 — Cross-Walk Pattern Analysis

## Status

Cross-walk analysis complete across 6 walks captured 2026-05-20..21 (hive worker #5, paired with workers 1/2/3/4/6). Tool: `scripts/cross_walk_analysis.py` + `scripts/hunt_ekf_yaw_jumps.py`.

## TL;DR for the queen

1. **W5 (bug3_walk) has ZERO in-walk stationary windows** — 0/69 ZRUP fires inside recording. Many counters that depend on `is_static` go to 0 in W5. This is NOT regression — discount W5 evidence for static-gated metrics.
2. **Bug 5 (Madgwick visual-yaw sync) FIRED 0 TIMES in ALL 6 walks** despite parent `updateGravityAlignedYaw` firing 8-16 times per walk. Bug 5 was added AFTER walk capture; today's walks do NOT validate Bug 5.
3. **Pose graph residual barely reduces (0.00-0.05% per walk)** even though `pg_max_correction_mm` reports 42-153mm corrections — pose graph isn't actually loop-closing trajectories. Hidden bug in `PoseGraph::optimize` convergence.
4. **EKF yaw-rate is degrading walk-over-walk**: W3 p99=49.94°/s → W5 p99=57.56°/s → W6 p99=78.42°/s. Bug 4 (reverted) made W6 worse.
5. **Loop-bearing drift is unstable and trending wrong**: W6 loop A↔B bearing delta = 113.24° (worst of all 6 walks).
6. **HANDOFF #3 verdict**: confirmed Agent 6. Trajectory freezes are designed `global_t_` freeze during `is_static`. Pattern is walk-dependent (W2/W3/W4 show end freezes; W5/W6 don't because user kept moving).
7. **SLAM is structurally starved**: 99.9-100% of promo candidates rejected across W3-W6. RMS rejection dominates (60-69%). MiDaS pipeline starves as a consequence (`midas_depth_samples`: 12→1→0→2 for W3→W4→W5→W6).

## Walk index

| # | name | stage | duration | path | peak | end | in-walk ZRUPs |
|---|---|---|---|---|---|---|---|
| W1 | heading_walk_1_2026_05_20 | pre-any-fix | 117.2 s | 101 m | 18.40 m | 12.96 m | n/a (shared logcat) |
| W2 | heading_walk_2_2026_05_20 | pre-any-fix | 124.0 s | 96 m | 13.13 m | 2.84 m | n/a |
| W3 | parallax_fix_walk_2026_05_20 | post-front-end-parallax | 128.4 s | 104 m | 15.64 m | 3.60 m | 143 |
| W4 | promo_parallax_walk_2026_05_21 | post-SLAM-promo-parallax | 118.9 s | 100 m | 14.19 m | 2.34 m | 34 |
| W5 | bug3_walk_2026_05_21 | post-Bug-3-rot-sanity | 114.6 s | 96 m | 15.31 m | 3.56 m | **0** |
| W6 | bug4_walk_2026_05_21 | post-Bug-4-REVERTED | 116.5 s | 97 m | 13.33 m | 2.53 m | 34 |

## Counter trend table

Columns = walks W1..W6 (same order as above). Rows only shown when at least one walk is non-zero.

### Yaw / rotation gates

| Counter | W1 | W2 | W3 | W4 | W5 | W6 |
|---|---|---|---|---|---|---|
| Visual yaw chi² rejected | 0 | 0 | 0 | 0 | 0 | **8** |
| Visual yaw gated pure-rot | 0 | 0 | 31 | 24 | 22 | 23 |
| Vis rel-rot chi² rejected | 0 | 0 | 0 | 0 | 0 | **503** |
| Relative rotation rejected | 0 | 0 | 0 | 0 | 0 | **503** |
| Madg visual yaw nudges (Bug 5) | **0** | **0** | **0** | **0** | **0** | **0** |
| Translation heading projection | 967 | 954 | 523 | 460 | 554 | 518 |
| Vis translation degenerate | 0 | 0 | 1099 | 1257 | 1059 | 1047 |

### Loop closure

| Counter | W1 | W2 | W3 | W4 | W5 | W6 |
|---|---|---|---|---|---|---|
| LC attempts | 160 | 137 | 178 | 173 | 155 | 156 |
| LC accepts | 1 | 1 | 25 | 18 | 14 | 9 |
| LC rej rot-sanity (Bug 3) | 0 | 0 | 0 | 0 | **10** | **10** |
| LC rej PnP | 1 | 2 | 17 | 15 | 11 | 8 |
| LC rej low_score | 1 | 3 | 2 | 2 | 2 | 3 |
| LC chi² rejected | 10 | 0 | 20 | 10 | 0 | 0 |
| LC corrections applied | 0 | 10 | 230 | 170 | 130 | 73 |
| LC geom rej descriptor | 1300 | 289 | 2400 | 3700 | 1500 | 586 |

### Pose graph

| Counter | W1 | W2 | W3 | W4 | W5 | W6 |
|---|---|---|---|---|---|---|
| PG optimize calls | 0 | 1 | 23 | 17 | 13 | 8 |
| PG loop edges added | 0 | 1 | 23 | 17 | 13 | 8 |
| PG max correction (mm) | 0 | 1 | **153** | 72 | 49 | 42 |
| PG residual pre (mm) | 0 | 1891 | 153391 | 82178 | 53325 | 36600 |
| PG residual post (mm) | 0 | 1890 | 153377 | 82174 | 53318 | 36595 |
| PG reduce % | n/a | 0.05% | **0.01%** | **0.00%** | **0.01%** | **0.01%** |

### SLAM promo funnel

| Metric | W3 | W4 | W5 | W6 |
|---|---|---|---|---|
| Candidates | 148816 | 139326 | 108175 | 121808 |
| Parallax rej % | 0.0% | 18.8% | 19.1% | 17.6% |
| Baseline rej % | 7.3% | 6.8% | 6.3% | 7.1% |
| Chirality rej % | 18.7% | 12.2% | 14.9% | 13.8% |
| RMS rej % | **68.9%** | **62.2%** | **59.6%** | **61.4%** |
| Promotions | **24** | **1** | **0** | **2** |
| Reject % total | 94.9% | 100.0% | 100.0% | 99.9% |

### Stationary / ZUPT / MiDaS

| Counter | W1 | W2 | W3 | W4 | W5 | W6 |
|---|---|---|---|---|---|---|
| ZRUP fired total | 33 | 78 | 144 | 35 | **0** | 35 |
| SLAM live updates fired | 8 | 6 | 7 | 0 | 0 | 2 |
| SLAM update gated static | 28 | 462 | 1001 | 33 | **0** | 4 |
| SLAM reanchor total | 258 | 452 | 1394 | 48 | **0** | 6 |
| MiDaS depth samples | 0 | 12 | 12 | 1 | **0** | 2 |
| MiDaS affine inlier ratio (milli) | 964 | 978 | 993 | 967 | 900 | 932 |
| MiDaS bailout few-floor | 29 | 34 | 11 | 15 | 15 | 13 |
| MiDaS bailout few-pts3d | 0 | 0 | 14 | 9 | 14 | 11 |

## Yaw stability evolution

Source: `scripts/hunt_ekf_yaw_jumps.py` on (logcat, JSON) per walk.

| Walk | EKF p99 °/s | EKF max °/s | Madg p99 °/s | ≥30°/s jumps | LC_ABS r_R median ° | LC_ABS r_p median m |
|---|---|---|---|---|---|---|
| W3 | 49.94 | 68.86 | 3.72 | 4 | 13.2 | 3.90 |
| W5 | 57.56 | 62.12 | 2.44 | 5 | 18.6 | 2.37 |
| W6 | **78.42** | **83.49** | 1.87 | 5 | **31.4** | 4.62 |

EKF yaw rate is **regressing** walk-over-walk despite supposed fixes. Bug 4 (chi² gate, REVERTED) made W6 the worst.

Madgwick yaw rate is HEALTHY (p99 < 4°/s, expected physiological limit).

### Net rotation per walk (EKF vs Madgwick)

| Walk | vyaw_net ° (EKF) | hdg_net ° (Madgwick) | disagreement |
|---|---|---|---|
| W1 | -510.03 | -549.99 | **+39.96°** |
| W2 | -650.85 | -650.55 | +0.30° |
| W3 | -732.59 | -739.03 | +6.44° |
| W4 | -673.37 | -673.72 | +0.35° |
| W5 | -708.44 | -667.30 | **-41.14°** |
| W6 | -649.55 | -654.90 | +5.35° |

W1 and W5 show EKF/Madgwick disagreeing by ~40°. Bug 5 was supposed to keep them in sync but never fired.

## Trajectory shape consistency (peak bearing per loop)

| Walk | Loop-A peak m | Loop-B peak m | Loop-A bearing ° | Loop-B bearing ° | \|Δ bearing\| ° |
|---|---|---|---|---|---|
| W1 | 18.40 | n/a (1 excursion) | -2.00 | -- | -- |
| W2 | 13.13 | 13.11 | 25.05 | -6.82 | **31.87** |
| W3 | 13.31 | 15.64 | -43.42 | -30.55 | 12.88 |
| W4 | 14.19 | 13.51 | -44.65 | 18.29 | **62.94** |
| W5 | 13.86 | 15.31 | -47.36 | -62.81 | 15.44 |
| W6 | 13.33 | 12.46 | 39.10 | -74.14 | **113.24** |

W6 is far worse than any other walk. Bug 4 broke trajectory shape consistency before it was reverted.

Note: GPS ground truth is **unusable** for these walks. W4=488m, W5=886m, W6=no excursion — all GPS-jamming artifacts (Haifa, per project memory `project_gps_jamming.md`). All trajectory metrics here are VIO self-consistency, not GPS-truth.

## HANDOFF #3 verdict (agreement with Agent 6)

**CONFIRM.** Trajectory freeze is designed `global_t_` freeze during `is_static`, NOT a bug.

| Walk | End-of-walk freeze samples | End-of-walk freeze duration | Max mid-walk freeze duration |
|---|---|---|---|
| W1 | 1 | 0.000 s | 2.44 s |
| W2 | **44** | 2.26 s | 2.44 s |
| W3 | 13 | 0.37 s | 5.86 s (init period, not real freeze) |
| W4 | **30** | 1.66 s | 2.48 s |
| W5 | 1 | 0.000 s | 2.72 s |
| W6 | 1 | 0.000 s | 1.94 s |

End-freeze appears ONLY in walks where the user stopped before stopping recording (W2/W3/W4). Walks where the user kept moving until the end (W5/W6) show no end-freeze. Same per-design `is_static` mechanism — NOT a bug.

## Hidden bugs identified

### HIDDEN BUG #1 — Bug 5 never fires

| What | Where | Evidence |
|---|---|---|
| `madgwick_visual_yaw_nudges_total` = **0** in ALL 6 walks | `Tracker.cpp:4178` | Bug 5 counter consistent across W1-W6 |
| `updateGravityAlignedYaw` IS firing 8-16x per walk | `EKFState.cpp:1748` | LC_YAW_FIRE log lines counted via Python: W3=10, W4=13, W5=16, W6=8 |
| Bug 5 is in the SAME `else {...}` branch | `Tracker.cpp:4095-4190` | Code inspection — single block, no early-return between updateGravityAlignedYaw and the Bug 5 nudge |
| Per-frame LC_YAW_FIRE residual median 1-8°, max 24° | logcat analysis | All under Bug 5's 45° sanity gate |

**Conclusion**: Bug 5 patch was added AFTER the 6 walks were captured today (last git commit 2026-05-19; uncommitted Tracker.cpp diff = 790 lines per `git diff --stat HEAD`). Today's walks do NOT validate Bug 5. **Worker 2 independently identified this as their secondary finding** (slightly different mechanism — they call it "structurally unreachable"; my LC_YAW_FIRE evidence shows it's reachable just not deployed).

### HIDDEN BUG #2 — Pose graph optimize doesn't converge

| What | Evidence |
|---|---|
| PG residual pre ≈ PG residual post in ALL walks | W3: 153391→153377mm, reduction 0.01% (14mm over 23 calls) |
| Yet `pg_max_correction_mm` reports 42-153mm corrections | W3=153mm, W6=42mm |
| `pose_graph_iters_used_sum` very low | W6: 16 iters / 8 calls = 2 iters/solve average |
| 165 keyframes in DB, 8 loop edges (W6) | 2-iter Gauss-Newton can't converge on 165-node problem |

**Conclusion**: pose graph optimize is doing tiny single-keyframe shifts that don't propagate through the graph. LC corrections feel like UI placebos — trajectory loops do not close. Suspect: `PoseGraph::optimize` iteration cap or convergence criterion. **Likely root cause of W6's 113° loop-bearing drift** — LC accepts fire but trajectory doesn't actually correct.

### HIDDEN BUG #3 — SLAM promotion structurally rejects 99.9-100%

Even W3 (before promo-parallax gate) had 24/148816 promotions = 0.016%. RMS rejection dominates (60-69%) — **NOT the parallax gate** added in W4. The W4 parallax gate added 18.8% upfront reject but didn't move the dial; RMS was already the chokepoint.

| Walk | Candidates | RMS rej % | Promotions |
|---|---|---|---|
| W3 (no parallax gate) | 148816 | 68.9% | 24 |
| W4 (with gate) | 139326 | 62.2% | 1 |
| W5 (Bug 3) | 108175 | 59.6% | 0 |
| W6 (Bug 4 rev) | 121808 | 61.4% | 2 |

**Worker 1's finding `slam_filled_3d_per_kf=1/85467=0.001%` is a downstream symptom of this funnel.** Without addressing the RMS rejection chokepoint, fixing the promo-parallax gate just trades one rejection reason for another.

## W5 anomaly characterization (calibration warning for other workers)

W5 (bug3_walk_2026_05_21) had **ZERO in-walk stationary windows**. Verified by counting ZRUP_FIRE log lines and aligning to walk recording window (walk: 15:42:22.690-15:44:17.273; ZRUP fires: 50 pre-walk, 0 in-walk, 19 post-walk).

Counters that go to 0 in W5 but non-zero in other walks (and ARE NOT bugs, just user behavior):
- `zrup_fired_total` (gate: is_static)
- `slam_live_batch_calls` (gate: stationary intervals)
- `slam_live_updates_fired` (gate: stationary intervals)
- `slam_update_gated_static_total` (counts when gated, but only gated when static)
- `slam_reanchor_total` (gate: stationary refresh)
- `slam_lifetime_count`, `slam_lifetime_obs_sum`
- `slam_live_skipped_no_parallax`
- `slam_promotions_seeded_with_midas`
- `midas_depth_samples` (downstream of SLAM seed)

**Other workers should use W3 (143 in-walk stationary windows) as the baseline for static-gated counters, not W5.**

## Cross-references to other worker findings

- **Worker 1** (`navsight.findings.bug_01`): confirms per-frame match rate 62.9% — my counter table shows 56-65% per-walk consistent with this. Worker 1's `slam_filled_3d_per_kf=0.001%` is downstream of my hidden bug #3 (SLAM promotion structurally starves).
- **Worker 2** (`navsight.findings.bug_02`): Worker 2 also flagged Bug 5 as not firing — independent corroboration. Slight disagreement on mechanism (they say "structurally unreachable"; my LC_YAW_FIRE evidence shows it would be reachable in a deployed build but Bug 5 was added after walks were captured). Worker 2's primary fix (`IMU_BG_PUSH` REPLACE→ADD+ZERO at Tracker.cpp:1357) addresses the gyro-bias self-injection root cause, which independent of Bug 5 status would explain my observed EKF↔Madgwick disagreement.
- **Worker 4** (`navsight.findings.bug_04`): worker 4 traces orange-dot flicker to 4 distinct mechanisms in UI render. My counter table corroborates: `landmarks_rendered_anchor_total` 275k-390k vs `landmarks_rendered_world_fixed_total` 195k-276k — large gap consistent with their "40% population drawn via projection fallback" finding. Also confirms their `heading_chain_FAITHFUL` verdict; my net-rotation table shows Madgwick (hdg) and EKF (vyaw) net deltas match within 5° on W2/W3/W4/W6 — so the chain is faithful; root drift is Bug 2.
- **Worker 6** (`docs/active_bugs/agent_06_lc_soft_correction_and_trajectory.md`): I CONFIRM their HANDOFF #3 verdict using the end-freeze pattern across all 6 walks (table above). Their LC chi² evolution table (parallax_fix=20 rej → promo_parallax=10 → bug3=0 → bug4=0) MATCHES my counter table exactly.

## Per-walk verdict

| Walk | Was the fix helpful? | Evidence |
|---|---|---|
| W3 (front-end parallax) | YES for LC | LC accepts 1→25; LC chi² rejects 10→20 (more LC triggered) |
| W4 (SLAM-promo parallax) | NO net benefit | SLAM promotions 24→1; loop-bearing drift 12.88°→62.94° |
| W5 (Bug 3 rot-sanity) | UNCLEAR — improved LC quality | LC chi² 20→0; rot-sanity gate fired 10 times; loop-bearing drift 15.44° |
| W6 (Bug 4 chi² REVERTED) | HARMFUL | EKF yaw p99 57.56→78.42°/s; loop-bearing drift 113.24°; relative-rotation rejected 503× |

## Recommendations (ranked by leverage)

1. **Fix Hidden Bug #2 (Pose graph optimize)** — without convergent pose graph, LC accepts are cosmetic. Falsifier: `PG reduce %` rises from 0.01% to >50% in next walk. Highest leverage: every LC accept becomes useful.
2. **Implement Worker 2's IMU_BG_PUSH fix** (Bug 02 primary) — independent of Bug 5 deployment status, this addresses the self-injected gyro bias that produces EKF↔Madgwick drift.
3. **Verify Bug 5 deployment** — run a fresh walk and confirm `madgwick_visual_yaw_nudges_total > 0`. If still 0, the patch is broken even when deployed.
4. **Investigate SLAM RMS rejection** (Hidden Bug #3) — 60-69% of promo candidates rejected on RMS criterion. Without changing this, MiDaS depth samples stay near zero. May simply need a less strict RMS gate (currently `slam_promo_rms_milli_p95=2904`).
5. **DOWNWEIGHT W5 evidence** in static-gated metric comparisons — W5 had zero in-walk stationary intervals.

## Confidence

- **HIGH** for hidden bug #1 (Bug 5 dead): direct counter+log evidence across 6 walks
- **HIGH** for hidden bug #2 (PG residual): direct counter evidence across 6 walks
- **HIGH** for SLAM promo funnel #3: direct counter evidence across 4 walks
- **HIGH** for W5 calibration warning: ZRUP-fire timestamps cross-correlated with walk recording window
- **MEDIUM** for Bug 4 was harmful: only W6 evidence; would benefit from a second post-Bug-4-revert walk to confirm reversion stuck

## Tools produced / referenced

- `scripts/cross_walk_analysis.py` — counter trend table generator (NEW, written this session)
- `scripts/hunt_ekf_yaw_jumps.py` — existing
- `scripts/analyze_chi2_rejections.py` — existing (referenced, not run in this session)
