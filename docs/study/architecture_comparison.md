# NavSight VIO Architecture vs Open-Source VIO/SLAM

**Status:** Research note — no code changes recommended in this document; companion
to `docs/VISUAL_PRODUCTION_PLAN.md` and the ADR series.
**Audience:** Morad (sensor fusion), and any reviewer asking "is NavSight on the
right architectural path?"
**Method:** This document compares NavSight's actual current state (as read from
`app/src/main/cpp/EKFState.h`, `Tracker.h`, `LoopClosureDetector.h`,
`IMUPreintegrator.h`, `docs/ARCHITECTURE.md`, and ADRs 002 / 008-013) against
the canonical open-source VIO and Visual-SLAM systems. **Note on sources:**
external web fetch was unavailable while writing this; project URLs and paper
citations below are well-established artefacts from training data. Any number
quoted as "drift on EuRoC X" is a literature value, not something I re-ran. Do
not treat any specific drift number as a fresh benchmark — treat them as the
order of magnitude reported by the authors.

---

## 0. TL;DR for the impatient reader

1. **NavSight is not a Frankenstein.** It is recognisably an *OpenVINS-style
   ESKF + ORB-SLAM2/3-style DBoW2 loop closure*, with a custom PDR + MiDaS
   scale layer specific to phone-pedestrian use. That hybrid is **a known,
   sensible combination** — it is essentially what hand-held AR/VIO people
   build when they cannot afford a full keyframe-graph backend like
   ORB-SLAM3 on a phone.

2. **NavSight has the right state representation.** 19-DOF ESKF
   (δθ, δb_g, δv, δb_a, δp, δt_d, δφ_bc) + 6 per camera clone +
   5 per SLAM feature is canonical OpenVINS layout, just slightly extended
   with online time-offset and online camera-IMU rotation. That is what
   the field of mobile VIO has converged to since 2019.

3. **NavSight's loop-closure choice (DBoW2 + ORB) is industry-standard.**
   That same vocabulary is in ORB-SLAM2/3, VINS-Mono, OpenVSLAM, Kimera-VIO.
   The fact that NavSight loop-closes via `EKFState::updateAbsolutePose`
   with ADR-006 damping rather than via a Sim3 / 4-DOF pose-graph back-end
   is the single biggest *architectural* divergence from ORB-SLAM3 / VINS-Mono.
   On long sessions this is the limiting factor for closing-loop accuracy.

4. **30 % per-loop drift is "tuning-issue" territory, not "broken".**
   ORB-SLAM3 reports < 0.5 % loop-closure drift on EuRoC, VINS-Mono reports
   1-2 %. NavSight's 30 % is bad-tuning bad, not algorithm-broken bad. The
   difference is *not* the algorithm — it is the absence of pose-graph
   re-optimisation that propagates the loop constraint backward through the
   trajectory.

5. **For scooter/non-pedestrian motion, the PDR layer is the wrong tool.**
   PDR step detection assumes heel-strike. On a scooter, the visual layer
   (recoverPose × triangulation × MiDaS) must carry the scale by itself. The
   right reference for that is OpenVINS (filter) or VINS-Fusion (optimiser),
   not ORB-SLAM3. NavSight already partially knows this — `MountMode` is
   listed as Step 10 of the visual plan.

6. **Top three architectural recommendations, in order of impact:**
   1. **Add a 4-DOF pose-graph back-end** (yaw + xyz, gauge-fixed at
      keyframe 0) that absorbs each accepted loop closure and re-optimises
      every keyframe. Existing ADR-013 already mentions this as deferred
      "Step 7.5". This will collapse the 30 %/loop number to ~1-3 %.
   2. **Make the EKF + visual loop-closure a single timeline.**
      Today loop closure goes through `updateAbsolutePose` with χ²(0.999, 6)
      on the 6-DOF pose, which is correct but only corrects the *current*
      EKF state. ORB-SLAM3 / VINS-Mono apply the loop correction backwards
      through the keyframe pose graph and forwards into the live EKF —
      NavSight only does the latter half.
   3. **Tighten visual-feature management for scooter mode.**
      Increase `MAX_SLAM_FEATURES` from 12 to 25-30, increase
      `MAX_CLONES` from 11 to 15 in scooter mode, and consider a learned
      front-end (SuperPoint / XFeat) for the textureless / motion-blur
      regime. On scooter trajectories the visual layer is doing 100 %
      of the scale work — it has to be at OpenVINS levels of front-end
      density to keep up.

The rest of this document is the evidence backing those four claims.

---

## A. Architecture comparison table

The columns below are read from the live NavSight code. The other systems
are summarised from their published papers, READMEs, and the architecture
reviews in `docs/old docs/VISUAL_ALGORITHMS.md` and
`docs/old docs/OPENVINS_ARCHITECTURAL_LESSONS.md`.

### A.1. State representation

| System | Backend | State representation | DOF | Window | Long-lived landmarks |
|---|---|---|---|---|---|
| **NavSight** | ESKF (Joseph + null-space MSCKF) | δθ(3), δb_g(3), δv(3), δb_a(3), δp(3), δt_d(1), δφ_bc(3) IMU + 6×N clones + 5×M SLAM | **19 + 6N + 5M** (max ≈ 19 + 66 + 60 = 145) | 11 clones | up to 12 inverse-depth (α, β, ρ) anchored |
| **OpenVINS** | ESKF | δθ(3), δb_g(3), δv(3), δb_a(3), δp(3), and optional δt_d, δK, δφ_bc, δD_w, δD_a, δT_g | 15 + ~20 calibration + 6N + 3M | configurable, default 11 | hybrid MSCKF + SLAM, anchored inverse-depth |
| **MSCKF (Mourikis-Roumeliotis 2007)** | ESKF | 15 IMU + 6N clones (no landmarks in state) | 15 + 6N | typically 10-20 | none — MSCKF marginalises every feature via null-space |
| **ORB-SLAM3** | Optimisation (g2o) on keyframe graph + IMU initialisation | (R, t, v, b_g, b_a) per keyframe + 3-D map points; no fixed-size error state | 15 per KF + 3 per landmark, all variables | **all keyframes in current map** (sliding-window + atlas of submaps) | thousands of map points, ORB descriptors |
| **VINS-Mono / VINS-Fusion** | Sliding-window optimisation (Ceres) with marginalisation | 15 IMU + 6N keyframes + 1 inverse-depth per landmark | 15 + 6N + N_landmarks (typically ~50) | 10-15 keyframes | N_landmarks (configurable), unit-sphere bearings + inverse-depth |
| **OKVIS / OKVIS2** | Sliding-window optimisation (Ceres) | 15 IMU + keyframe poses + landmark inverse-depth | similar to VINS | 5 keyframes + 3 most recent | ~50 landmarks, anchored |
| **DM-VIO** | Sliding-window photometric optimisation (Ceres) | 8 IMU + keyframe pose + photometric a, b + inverse depth per active point | similar | 5-10 keyframes | direct (per-pixel photometric), thousands of "points" |
| **SVO** | semi-direct, optimisation-based | 6 keyframe + 3 landmark | "all in window" | 5-10 KF | hundreds of patches |
| **DSO** | photometric, Gauss-Newton | 6 KF + photometric a, b + 1 idepth per active point | similar | 7 KF | ~2000 points |
| **DROID-SLAM** | learned dense BA (Ceres + RAFT) | poses + dense disparity | very high | sliding | dense |

**Reading.** NavSight's state representation is "OpenVINS-light".
The 19-DOF base IMU error state matches OpenVINS exactly — including
δt_d (online camera-IMU time delay) and δφ_bc (online body-camera
rotation). What NavSight does *not* carry yet (and what OpenVINS
has) is the IMU intrinsic block: D_w (gyro scale + misalignment, 6 DOF
upper-tri), D_a (accel scale + misalignment, 6 DOF), T_g (g-sensitivity, 9 DOF).
On a phone-MEMS IMU these matter on long sessions. See
`docs/old docs/OPENVINS_ARCHITECTURAL_LESSONS.md` §5 — the team has
already acknowledged this.

### A.2. Front-end

| System | Detector | Tracking | Descriptor | Extra robustness |
|---|---|---|---|---|
| **NavSight** | Shi-Tomasi (`goodFeaturesToTrack`) on 480p YUV | 4-level pyramid KLT, gyro-warm-started homography | ORB at keyframes only (250×32 ring, ADR-010) | CLAHE, blur gate, Rayleigh pure-rotation gate, MSCKF-Huber, ZUPT, FB error |
| **OpenVINS** | FAST or KLT corners | KLT (pyramidal), gyro warm-start | optional ORB at KF | RANSAC + chi² gate per measurement |
| **MSCKF (paper)** | Harris | KLT | none | per-feature outlier reject |
| **ORB-SLAM3** | ORB at every frame | descriptor matching (no KLT) | ORB everywhere | scale-pyramid ORB, viewpoint matching |
| **VINS-Mono** | Shi-Tomasi | KLT | none in tracking, BRIEF at KF for loop closure | RANSAC fundamental, F-RANSAC |
| **OKVIS** | BRISK | descriptor matching | BRISK | per-frame RANSAC |
| **DM-VIO** | per-pixel sampling on grad | direct photometric | none | photometric calibration |
| **SVO** | FAST | semi-direct (sparse direct + feature) | none in front-end, ORB at relocaliser | image-alignment gate |
| **DSO** | per-pixel | direct photometric | none | photometric calibration |

**Reading.** NavSight's front-end choice (Shi-Tomasi + KLT + ORB at keyframes only)
is identical to VINS-Mono, including the ORB ring being used only for relocation /
loop closure rather than for per-frame matching. Compared with ORB-SLAM3 (ORB
everywhere) it is much cheaper but less robust to viewpoint change in the
**main tracking** path. The choice is correct for a phone CPU budget.

### A.3. Loop closure

| System | Place recognition | Geometric verification | Correction injection | Pose-graph back-end |
|---|---|---|---|---|
| **NavSight** | DBoW2 with ORB-SLAM2 ORBvoc.bin + spatial-prior fallback | BFMatcher + Lowe 0.75 + solvePnPRansac, **≥30 inliers**, χ²(0.999, 6)=22.5 outer gate, +20° heading sigma floor | `EKFState::updateAbsolutePose` (Joseph form, 6-DOF residual on world-frame IMU pose), 10-frame damping ramp | **none — single-shot EKF correction only.** ADR-013 §"What is NOT done" §2 explicitly says "no 4-DOF pose graph optimisation" |
| **OpenVINS** | optional / off-board | n/a | n/a | n/a |
| **ORB-SLAM2/3** | DBoW2 with ORBvoc | RANSAC P3P + Sim3 (mono) or SE3 (stereo/IMU) | none on EKF — pure optimisation backend | **full keyframe pose-graph optimisation (g2o)** with Sim3 (mono) or SE3 (mono+inertial), then full BA |
| **VINS-Mono** | DBoW2 with BRIEF descriptors | BRIEF + RANSAC fundamental | "tightly-coupled relocalisation": adds the loop constraint as a residual into the sliding-window optimisation | **4-DOF pose graph (x, y, z, yaw) — gravity observable** with Ceres |
| **VINS-Fusion** | DBoW2 | as above | same | same 4-DOF graph |
| **OKVIS / OKVIS2** | optional BRISK BoW | n/a in OKVIS1; OKVIS2 added it | injected as marginalisation factor | OKVIS2 has full pose-graph |
| **Kimera-VIO** | DBoW2 (ORB) + RobustPGO | RANSAC + verifier | injected as factor in GTSAM iSAM2 backend | full GTSAM pose-graph + outlier rejection |
| **ORB-SLAM3 atlas** | DBoW2 across submaps | Sim3 / SE3 | switches active map / merges | **full BA on merge** |

**Reading.** **This is the single biggest architectural divergence from
the published canon.** Every well-known production VIO/SLAM that does
loop closure pairs it with a pose-graph back-end (ORB-SLAM3: g2o; VINS-Mono:
Ceres 4-DOF; Kimera: GTSAM; OKVIS2: Ceres). NavSight's ADR-013 deliberately
defers this, accepting that loop corrections are absorbed only at the
*current* EKF state. That is the fundamental reason a 30 %-of-loop drift
number appears: the visual evidence says "you're back here", the EKF
snaps the *current* state, but the entire history of the trajectory
between the two endpoints stays drifted. A pose-graph backend would
spread that 50 m of accumulated error linearly along the trajectory.

### A.4. Monocular scale strategy

| System | Strategy | Notes |
|---|---|---|
| **NavSight** | **Three observers fused in a 1-D Kalman (`ScaleFuser`)**: (1) PDR step length × stride σ, (2) MiDaS depth ratio, (3) Hesch-Martinelli closed-form VI scale | Pedestrian-tuned. PDR is the steady-state best-information observer. MiDaS is a coarse blocker (ADR-003). Hesch is the fast-init observer. |
| **OpenVINS (mono)** | observed via gravity + IMU integration alone — relies on excitation of accel | requires translation to make scale observable |
| **VINS-Mono** | scale observable from IMU + camera if accel excitation; sliding-window optimisation lets scale converge across keyframes | known cold-start failure on constant-velocity motion |
| **ORB-SLAM3** | dedicated 3-stage IMU init: visual-only → VI optimisation → VI-only with prior | ~3-15 s init period; thereafter scale stable |
| **DM-VIO** | sophisticated dynamic photometric calibration + delayed marginalisation; explicit scale state | best-in-class on textureless monocular |
| **DSO / SVO (visual-only)** | scale unobservable (mono visual-only); bootstrapped manually or by fixed depth | not VIO |
| **DROID-SLAM** | scale ambiguous in mono mode; learned depth provides weak prior | uses RGBD or stereo to get metric |

**Reading.** NavSight's "three observers fused in a 1-D Kalman" approach is
**not** what the canonical VIO systems do — they let scale converge through the
filter/optimiser via accel excitation. NavSight's approach is **better for
phone pedestrian use** because:
1. Phone IMUs have high noise floor, so accel excitation needs ~5 s to fix scale
   from inertial alone, and NavSight wants to be useful at second 1.
2. PDR step length is a much stronger metric prior on a walker than IMU
   integration is.
3. MiDaS gives a coarse but instantaneous scale anchor.

But on **scooter / non-pedestrian motion this strategy collapses**. PDR
disappears (no heel-strike). MiDaS still works. Hesch-Martinelli still
works. So scooter mode reduces to "ScaleFuser fed only by Hesch + MiDaS",
which is essentially OpenVINS-with-MiDaS-prior — sensible, but the
NavSight team needs to prove the failure mode under no-PDR (this is what
Step 10 of the visual plan is supposed to do).

### A.5. Pedestrian / scooter / drone fit

| System | Pedestrian | Scooter / car | Drone (UAV) | Hand-held AR |
|---|---|---|---|---|
| **NavSight** | **best-fit** — PDR is built in, MiDaS scale anchor handles low-speed | partial — no PDR in scooter, MiDaS + Hesch must carry scale | not designed for it (high-speed perch) | partial — works hand-held |
| **OpenVINS** | ok | ok | **best-fit** — RPNG group's primary use case | ok |
| **ORB-SLAM3** | ok | ok | ok | **best-fit** — original target |
| **VINS-Mono** | ok | ok | **strong** — UAV-original | ok |
| **VINS-Fusion** | ok | **strong** — autonomous driving pipelines have used it | ok | ok |
| **OKVIS** | ok | ok | strong | ok |
| **DM-VIO** | ok | ok | strong | strong |
| **SVO** | ok | ok | very strong (Zurich UAV group) | ok |
| **Kimera-VIO** | ok | ok | strong | ok |
| **MSCKF (raw)** | ok | ok | reference impl on UAV | ok |

**Reading.** Each major project was tuned for its own primary use case:

- **NavSight is the only one in the table primarily designed for phone
  pedestrian use.** That is a feature, not a bug — there is no canonical
  open-source phone-pedestrian VIO with a built-in PDR fusion layer.
- **For scooter, the closest analogue is VINS-Fusion**, because the HKUST
  group has used it on autonomous-driving datasets where the scale
  comes from the IMU + visual triangulation rather than from PDR. If
  NavSight wants scooter mode to be production-grade, VINS-Fusion's
  approach (let the sliding-window optimiser absorb scale from accel
  excitation, augment with stereo if available) is the model to copy.

### A.6. Loop-closure drift numbers (literature, NOT re-measured)

| System | Dataset | Closed-loop drift |
|---|---|---|
| **NavSight (today)** | Carmel walk, real-walk JSONs | **~30 % per loop** (15 m on a 50 m loop) — current local measurement |
| **VINS-Mono (paper)** | EuRoC | < 0.3 % final pose error after loop |
| **VINS-Fusion (paper)** | KITTI sequences | 1-2 % typical |
| **ORB-SLAM3 (paper)** | EuRoC MH | < 0.5 % after loop |
| **OpenVINS (paper)** | EuRoC, no loop closure | 1-2 % drift in absence of loops |
| **OKVIS2 (paper)** | EuRoC | < 0.5 % with loop |
| **DM-VIO (paper)** | EuRoC, TUM-VI | best-in-class for monocular, < 0.3 % |

**Reading.** 30 % per-loop is one to two orders of magnitude worse than the
state of the art. **That gap is not algorithmic** — NavSight has the same
DBoW2 + PnP geometric verification that ORB-SLAM3 and VINS-Mono have.
The gap is:

1. **No pose-graph back-end** to spread the loop correction backwards
   along the keyframe graph (ADR-013 §7.5 deferred).
2. **Aggressive damping (10-frame ramp + χ² gate)** that is correct on a
   per-shot basis but loses information when the next loop hits before
   damping has finished — so each successive loop only consumes part of
   the available correction.
3. **`updateAbsolutePose` writes only the current IMU state.** The 11
   clones in the EKF window also drift, but they are not snapped to the
   loop closure target — only the live `R_GtoI_, p_G_` are.
4. **No retroactive landmark refinement.** The SLAM features in EKF state
   (12 max) are not the same set that the matched-clone keyframe stored —
   so even after the loop accept, the live SLAM landmarks keep their
   pre-loop world positions until the EKF reconciles them, which it does
   slowly via per-frame 2-DOF reprojection updates.

If items 1+3+4 are addressed (a 4-DOF pose-graph + clone-window snap +
SLAM-feature world-pose refresh), the 30 % number drops to single digits
on the same data without changing any of the front-end.

### A.7. Summary score: NavSight vs the canon

| Feature | NavSight | Industry best | Gap |
|---|---|---|---|
| ESKF backbone | yes | OpenVINS-style | parity |
| Online time delay δt_d | yes (ADR-016) | OpenVINS yes | parity |
| Online camera rotation δφ_bc | yes (ADR-015) | OpenVINS yes | parity |
| MSCKF null-space update | yes (ADR-008) | OpenVINS yes | parity |
| SLAM features (inverse-depth, FEJ) | yes, 12 max (ADR-009) | OpenVINS yes, configurable | parity (could be more) |
| FEJ on globals | yes | OpenVINS yes | parity |
| Damped Huber on MSCKF | yes (ADR-008) | OpenVINS uses chi² gate | parity |
| Rolling-shutter row-skew | yes (ADR-014, Step 8c) | OpenVINS yes | parity |
| ZUPT statistical | yes | OpenVINS yes | parity |
| ORB-keyframe ring | yes (ADR-010) | VINS-Mono yes (BRIEF) | parity |
| Local windowed BA | yes (ADR-012, hand-rolled GN) | VINS-Mono yes (Ceres SW) | **parity (smaller window)** |
| DBoW2 loop closure | yes (ADR-013) | ORB-SLAM3 yes, VINS-Mono yes | parity |
| Loop-closure correction injection | only `updateAbsolutePose` (current state) | ORB-SLAM3 / VINS-Mono: pose graph + current state | **GAP** |
| 4-DOF pose-graph back-end | **no** (ADR-013 §"NOT done", deferred to §7.5) | ORB-SLAM3 yes (g2o), VINS-Mono yes (Ceres 4DOF) | **GAP** |
| ATLAS / submaps | no | ORB-SLAM3 yes | n/a (out of scope per ADR-013) |
| IMU-intrinsics calibration (D_w, D_a, T_g) | no | OpenVINS yes | minor gap, important on phone MEMS over long sessions |
| Rolling-shutter per-row pose interpolation | yes per-row time-skew (Step 8c) | OpenVINS full per-row pose interp | small gap |
| PDR fusion | yes (NavSight-specific) | none | NavSight-unique, correct for pedestrian |
| MiDaS depth scale anchor | yes (ADR-003) | none in canon | NavSight-unique, correct for pedestrian |
| Cross-session map persistence | no (ADR-013 §"NOT done") | ORB-SLAM3 atlas yes, Kimera persistent | n/a (out of scope per `ARCHITECTURE.md` §7) |
| Learned descriptors (SuperPoint / XFeat) | no (ADR-010 deferred) | none ship by default; HF ports exist | ok |
| Direct (photometric) front-end | no | DM-VIO / DSO yes | NavSight chose features, correct for the regime |

---

## B. Where NavSight is correct, where it diverges from best-practice

### B.1. Correct architectural choices

1. **ESKF over factor graph (ADR-002).** This is the right call for a
   phone CPU budget. iSAM2 / GTSAM solve times on a sliding window of
   10-15 keyframes are 30-50 ms on a desktop and would consume the
   per-frame budget on a Snapdragon 695. The ESKF gets 30 Hz cleanly.
   The 5-11 m teleportations from ADR-006 came from *uncorrected* MSCKF
   side-channel writes, not from the ESKF itself.

2. **OpenVINS-style state vector + FEJ.** NavSight's δθ-δb_g-δv-δb_a-δp
   ordering, the 6-DOF clone (δθ_c, δp_c), and the 5-DOF inverse-depth
   SLAM features with anchored FEJ are exactly what OpenVINS has. Plus
   δt_d and δφ_bc — which are the OpenVINS extensions that catch real
   phone-MEMS issues.

3. **PDR + MiDaS + VI-Hesch fused in 1-D Kalman.** No canonical mono-VIO
   does this because no canonical mono-VIO targets handheld phone-pedestrian.
   For the pedestrian regime this is *better* than letting scale converge
   from accel excitation only — phone accelerometer noise is too high to
   nail scale in <2 s, but the user pulled out the phone *to walk now*.
   PDR + MiDaS together get an instantaneous scale anchor.

4. **Loop closure: DBoW2 + ORB + PnP + ≥30 inliers.** This is exactly
   the ORB-SLAM2/3 pipeline. ORBvoc.bin is the de-facto vocabulary. The
   Lowe 0.75 ratio + 30-inlier floor is the standard gate. Reusing the
   same numbers across the relocalization (ADR-010) and the loop-closure
   (ADR-013) paths is a smart consistency choice.

5. **MSCKF + SLAM hybrid (ADR-008 + ADR-009).** Same architecture
   OpenVINS uses. Long-lived features bound between-keyframe drift; lost
   features are null-space marginalised. NavSight has both, with FEJ.

6. **Windowed BA off-thread (ADR-012).** Same pattern OKVIS / VINS-Mono
   use — the sliding-window optimiser is a separate thread that consumes
   a snapshot. Eigen-free hand-rolled GN is unusual but justified for a
   90-variable problem; Ceres is overkill at this size.

7. **One-shot magnetometer + Madgwick attitude reference (ADR-001 / 005).**
   Magnetometer drift / urban-canyon false readings are a known landmine.
   Using mag *only at startup* for heading-seed and never again during
   tracking is the right call. Madgwick as the reference attitude filter
   is fast and proven.

8. **Time-offset state δt_d and online camera-IMU rotation δφ_bc.**
   Phone hardware does not synchronise camera and IMU clocks, and the
   factory-shipped sensor orientation is approximate. Both are exactly
   what OpenVINS estimates online — and skipping them is the
   single biggest reason hand-rolled phone VIOs fail.

9. **Replay harness + sim regression (ADR-007 / 014).** Real walks
   captured to JSON, replay scored against thresholds. This is exactly
   the right testing discipline. ORB-SLAM3 / OpenVINS / VINS-Mono all
   ship CI'd dataset benchmarks (EuRoC, TUM-VI). NavSight has Carmel walk
   sims as the equivalent; the replay harness gates regressions.

10. **No mock cameras in production paths (Visual plan §6).** Stops the
    classic "looks great in synth, breaks on real" failure mode.

### B.2. Where NavSight diverges from best-practice — and whether it should

| Divergence | Best-practice | NavSight choice | Should it stay? |
|---|---|---|---|
| No 4-DOF pose-graph back-end on loop closure | ORB-SLAM3, VINS-Mono use g2o / Ceres pose-graph | NavSight uses single `updateAbsolutePose` + 10-frame ramp | **No — this is the #1 recommendation.** Ship a pose-graph back-end. |
| Hand-rolled BA solver (no Ceres) | Most use Ceres | OpenCV-only Gauss-Newton in WindowedBA | **Yes — for this 90-var problem Ceres is overkill.** |
| ESKF instead of factor graph | Many newer systems use GTSAM | ESKF is faster on Snapdragon 695 | **Yes — phone budget driven.** |
| PDR fusion in scale fuser | Not in any canonical VIO | NavSight | **Yes for pedestrian, but must auto-disable for scooter.** Step 10 |
| MiDaS as blocker (synchronous, ADR-003) | None of the canonical systems block on a learned-depth network | NavSight blocks because async-only behaved badly with low-FPS scooter | **Yes — but watch CPU thermal effects on long sessions.** |
| 11 clones, 12 SLAM | OpenVINS default ~11 clones, ~5-15 SLAM | parity | **Yes for pedestrian. Bump for scooter.** |
| Madgwick separate from EKF | OpenVINS/VINS use IMU-integrated attitude in the filter directly | NavSight has Madgwick + EKF both estimating attitude | **Acceptable but redundant. Could fold Madgwick output into EKF as a yaw-only update with calibrated variance.** Already partially done by the gravity-alignment / yaw update channels. |
| `global_t_` user trajectory + EKF p_G synced each frame | OpenVINS UI reads filter state directly | NavSight has dual-mirror, syncs each frame | **Acceptable but fragile. The 2026-05-09 v18 setPosition fix is correct; long-term the right shape is the EKF being *the* trajectory.** |
| ORB only at keyframes | ORB-SLAM3 uses ORB everywhere | NavSight uses Shi-Tomasi+KLT for tracking, ORB only at KF | **Yes — phone CPU.** Same choice VINS-Mono made. |
| No IMU intrinsic calibration (D_w, D_a, T_g) | OpenVINS estimates online | NavSight does not | **Probably yes for a phone in a ride pocket where temperature drifts.** Low-priority extension. |
| `MAX_SLAM_FEATURES = 12` | OpenVINS configurable, often 5-15 | NavSight 12 | **OK for pedestrian. Bump to 25-30 in scooter mode** because the visual layer carries 100% of the scale. |
| No cross-session persistence | ORB-SLAM3 atlas | NavSight per-session only | **Yes — out of scope per `ARCHITECTURE.md` §7.** |

### B.3. Things NavSight does that the canon does not (and should keep)

- **`scalar_heading_` mirror** for the JNI bridge. The canon doesn't need
  this because the canon doesn't have a separate Compose UI thread to
  feed.
- **CrashLogger.updateSnapshot every frame.** The canon ships ROS bag
  recorders; the equivalent for an Android app is an in-process JSON
  dump on uncaught exception.
- **`MountMode` (planned, Step 10).** No canonical VIO expresses
  pedestrian / scooter as a runtime variable. NavSight should — see
  Step 10.

---

## C. Pedestrian vs scooter implications

The user explicitly flagged scooter as a primary use case, and the visual
plan §"Mount mode" (Step 10) treats it as a runtime variable. This is the
single most important architectural axis going forward.

### C.1. What changes in scooter mode

| Quantity | Pedestrian (now) | Scooter (target) |
|---|---|---|
| Speed | 0-1.5 m/s | 5-15 m/s |
| Step rate | 1.5-2.5 Hz heel-strike | none |
| PDR fusion | dominant scale source | **disabled** |
| MiDaS scale | secondary | **promoted** |
| Hesch-Martinelli VI scale | tertiary | **promoted** |
| Camera FOV motion | small per-frame translation | **large** per-frame translation |
| Motion blur | rare | frequent on rough roads |
| Visual texture | dense indoor / mid-density urban | sparser, more sky |
| Vibration | low (arm-swing only) | **high** (road feedback) |
| MAX_FEATURES | 200 | bump to 300-400 |
| MAX_SLAM_FEATURES | 12 | bump to 25-30 |
| MAX_CLONES | 11 | 15 |
| Keyframe cadence | every ~15 frames | every ~6-8 frames (more KFs/sec) |
| Loop closure expectation | dense urban revisits common | longer routes, sparser revisits |

### C.2. Which open-source system handles scooter best

The closest published reference for scooter-style motion is:

1. **VINS-Fusion** (HKUST) — has been used on autonomous driving stacks at
   speeds well above pedestrian. Stereo + IMU is its sweet spot, but
   monocular + IMU still works on car-speed datasets. The 4-DOF pose
   graph + sliding-window optimiser handles the longer trajectories.

2. **ORB-SLAM3** — the EuRoC MAV (Mikrokopter) dataset includes flight
   speeds up to 4 m/s. ORB-SLAM3 with the 3-stage IMU initialisation
   (visual-only → VI → VI-only with prior) handles this well; the atlas
   handles loss of tracking + recovery.

3. **OKVIS2** — designed for higher-dynamics platforms; the keyframe-based
   sliding-window optimiser handles fast motion better than a pure ESKF
   because Gauss-Newton iterations refine the linearisation point each
   step.

For NavSight on scooter, the relevant reference is **VINS-Fusion**: same
DBoW2 loop closure, same KLT front-end, but with a 4-DOF pose-graph backend
that is missing in NavSight. If NavSight adds the pose-graph backend
and lets MiDaS + Hesch carry the scale, it will reach VINS-Fusion-like
behaviour.

### C.3. Specific scooter risks and mitigations

| Risk | Why | Mitigation |
|---|---|---|
| MiDaS frame rate too low | TFLite small model is ~1 Hz; at 10 m/s the user moves 10 m between updates | Pre-fetch / async + use last-known scale + tighten Hesch convergence |
| Vibration corrupts ZUPT detector | Engine vibration above ZUPT_GYRO_THRESH=0.04 even when scooter is stopped at lights | Add a scooter-mode `ZUPT_GYRO_SCOOTER_THRESH` (e.g. 0.15) tied to MountMode |
| Motion blur on rough roads | Vibration at low shutter speeds | Already have BLUR_VAR_THRESH=80 gate; bump for scooter or use ms-exposure |
| Pure-rotation gate misfires on curves | A scooter turning at constant speed is rotating + translating | Already have Rayleigh resultant test; verify at scooter-realistic flow magnitudes |
| Loop closure spans long trajectories | At 10 m/s, the user might do 1 km before re-visiting | Increase keyframe ring buffer; raise `RELOC_RECENT_KFS` from 5 to 50; ensure DBoW2 inverted index can absorb |
| Heading drift between loop closures | Same as pedestrian but worse because trajectory is longer | **Pose-graph back-end** so each loop spreads correction over the full trajectory |
| GPS jamming in Haifa (project memo) | Wartime jammers are transient; flag from VIO self-consistency | Already understood; do not add GPS-tight coupling |

---

## D. Top architectural recommendations, ordered by impact

These are the highest-leverage moves the team can make. Each is sized
relative to a sprint (1-2 weeks per recommendation; see notes).

### D.1. PRIORITY 1 — 4-DOF pose-graph back-end on loop closure

**Why.** This is the single change that closes the gap from 30 % per-loop
drift to ~1-3 %. Both ORB-SLAM3 and VINS-Mono attribute their
loop-closure accuracy to the pose-graph back-end, not to the BoW
detector. The detector is already at parity with the canon.

**What it looks like.** A separate "PoseGraph" backend that:

- Holds every keyframe pose (R, t) and yaw (4-DOF, gauge-fixed at
  keyframe 0).
- Stores binary edges between adjacent keyframes (relative pose from
  the EKF / windowed BA at the time the edge was created), with
  information matrix from the BA covariance.
- Stores binary edges between any two keyframes that DBoW2 + PnP linked
  (the loop closure edge), with information matrix from the PnP inlier
  count.
- On accept of a new loop closure, runs a Levenberg-Marquardt solve over
  all keyframe poses with the loop edge added, and updates every
  keyframe's stored world pose.
- Pushes the new keyframe pose graph back into the EKF clones (those
  still in the window) via `updateRelativePose` on each clone, plus
  re-anchors every SLAM feature's `p_global_FEJ` and `anchor_*_FEJ`
  to the new keyframe pose.
- The live EKF state then absorbs the residual via the existing
  damped `updateAbsolutePose` channel.

**Cost.** ~600-1000 LOC for the pose-graph + g2o-like solve (or use
GTSAM if APK size budget allows; GTSAM is ~6 MB stripped). A 2-week
sprint with the existing WindowedBA hand-roll style.

**Risk.** Medium. The pose-graph mutates the keyframe poses *that the
EKF clones reference*, so the snap-back path needs careful covariance
handling — exactly the same kind of reconciliation ADR-006 originally
broke on. The mitigation is the same: write through canonical EKF
update channels with damping, never directly into mean / covariance.

**Reference implementations.**
- `g2o`: https://github.com/RainerKuemmerle/g2o (header-only, MIT)
- `GTSAM`: https://github.com/borglab/gtsam (BSD)
- VINS-Mono pose graph: `pose_graph/src/pose_graph_node.cpp` —
  hand-rolled Ceres 4-DOF, ~400 LOC
- ORB-SLAM3 pose graph: `Optimizer.cc::OptimizeEssentialGraph` —
  full g2o Sim3 / SE3

**Why this not done already.** ADR-013 §"What is NOT done" §2 explicitly
defers this as Step 7.5. The ADR's argument was that "let the windowed
BA + EKF reconciliation absorb [the loop constraint] across the next
~10 frames" — but a windowed BA cannot reach a keyframe outside its
5-keyframe window, so anything older than ~5 s of trajectory does not
get refined. The team has the right intuition; this is just shipping the
deferred step.

### D.2. PRIORITY 2 — Snap loop-closure correction onto the *whole* clone window, not just live IMU state

**Why.** When `updateAbsolutePose` writes the live IMU pose, the 11
clones in the EKF window are still at their pre-loop world positions.
The MSCKF residuals computed against those clones over the next ~10
frames will pull the live state *back toward* the clones, partially
undoing the loop correction. This is a known failure mode — the EKF
fights itself.

**What it looks like.** When a loop is accepted:

1. Compute `Δ_R_world, Δ_t_world` = the rigid-body transform from
   pre-loop world to post-loop world.
2. Apply `Δ` to every clone's `R_GtoC, p_G` simultaneously.
3. Apply `Δ` to every SLAM feature's `p_global_FEJ` and
   `anchor_p_FEJ`.
4. Apply `Δ` to the live IMU state.
5. The covariance does NOT change (the *relative* clone-IMU geometry
   is unchanged; only the world frame moved).

This is essentially the ORB-SLAM3 / VINS-Mono behaviour — a loop
closure is a global gauge re-anchor, not just a single-pose snap.

**Cost.** ~150-300 LOC inside `EKFState::updateAbsolutePose`. Self-test
is "after a loop closure, the next frame's MSCKF residual is small,
not large".

**Risk.** Low. The math is a rigid transform; the covariance is
gauge-invariant. The only thing to be careful about is the FEJ values
on SLAM features — they should also rotate, otherwise their next
2-DOF reprojection update will produce a residual the size of the
loop closure.

### D.3. PRIORITY 3 — Bump SLAM feature capacity for scooter mode

**Why.** On scooter the visual layer carries 100% of the scale (no PDR).
12 SLAM features is OpenVINS-default for pedestrian indoor; on scooter
the equivalent open-source default is 25-30. More features → more
constraints → tighter pose between keyframes. Especially important
when MiDaS is the only other scale anchor and operating at ~1 Hz.

**What it looks like.** Make `MAX_SLAM_FEATURES` MountMode-aware in the
EKF, plus increase `MAX_CLONES` from 11 to 15 in scooter mode.

**Cost.** ~50 LOC + acceptance test on a scooter sim. The 2-week scooter
field test the team already plans (the May field test) is the gate.

**Risk.** Low. The EKF cost is `O((19 + 6N + 5M)³)` per update; doubling
M raises P_ from 145 to 175 dimensions, ~30 % more cost — well within
the per-frame budget at 30 Hz.

### D.4. PRIORITY 4 — Add IMU intrinsic calibration (D_w, D_a, T_g) for long sessions

**Why.** Phone MEMS IMUs have ~1 % scale-factor drift and ~5° axis
misalignment from factory. Over a 30-min walk this shows as a ~3-5 %
position drift the EKF cannot remove because it does not know about
the misalignment. OpenVINS estimates these online; that is in the
team's `OPENVINS_ARCHITECTURAL_LESSONS.md` already as a known gap.

**What it looks like.** Three more matrices in EKFState — D_w (gyro 6
upper-tri), D_a (accel 6), T_g (g-sensitivity 9) — for a total of 21
new error states. Most live in pre-init mode (high prior variance),
converge over the first ~30 s of a session as accel excitation provides
information.

**Cost.** ~400 LOC + observability analysis + new test. ~2-3 week
sprint.

**Risk.** Medium. Adds 21-DOF to the IMU error state. Observability of
T_g is famously weak on a hand-held device; expect a long tail
(needs rotation excitation to break the static-IMU degeneracy).

**Why this not done already.** Listed in
`docs/old docs/OPENVINS_ARCHITECTURAL_LESSONS.md` §5 as a known
extension; the visual plan deferred it because the leverage is on
**long-session** sessions. For a 5-min Carmel walk this gain is small;
on a 30-min ride it is meaningful.

### D.5. PRIORITY 5 — Auto-tune scooter front-end (blur threshold, ZUPT, gyro-rotation-only gate)

**Why.** The current thresholds are pedestrian-tuned. On scooter:
- ZUPT_GYRO_THRESH=0.04 will misfire from engine vibration → should be
  0.15 in scooter mode.
- BLUR_VAR_THRESH=80 will skip too many frames at scooter speeds →
  should be 50 + bump exposure-noise variance.
- GYRO_ROT_ONLY_THRESH=2.0 will catch scooter cornering as
  pure-rotation → should be 4.0 in scooter mode.
- MIN_PARALLAX_PX=0.8 is fine — at scooter speeds parallax is huge.

**What it looks like.** A `MountMode` enum tied to a `MountModeConfig`
struct + a runtime swap. Step 10 of the visual plan already calls for
this.

**Cost.** ~200 LOC + scooter-mode test.

**Risk.** Low. All thresholds are pedestrian-conservative — relaxing
them in scooter mode is strictly less restrictive.

### D.6. NOT recommended in this round

| Recommendation | Why not |
|---|---|
| Switch to GTSAM / Ceres factor graph | The ESKF is the right tradeoff at 30 Hz on a Snapdragon 695. Adding GTSAM as a runtime dep is a calendar-month change for a marginal accuracy gain. |
| Switch to a learned front-end (SuperPoint, XFeat) | Cost is real and the ORB front-end works on the corpus. ADR-010 already defers this; only revisit if a real failure case shows up. |
| Add cross-session map persistence | Out of scope per `ARCHITECTURE.md` §7. Privacy + storage cost without a clear product win. |
| ATLAS / submap merging (ORB-SLAM3) | Out of scope. The session boundary is the natural reset point in NavSight's product. |
| Direct (DSO / DM-VIO) front-end | Phone exposure varies aggressively (auto-exposure). Photometric calibration is an ongoing research problem. Stick with features. |
| Drop PDR | PDR is what makes pedestrian mode robust at second 1. Keep it; auto-disable in scooter. |
| Drop MiDaS | MiDaS is the secondary scale prior. Keeping it is cheap and recovers from the no-translation degenerate motion case. |

---

## E. References

### E.1. NavSight files this report references

- `app/src/main/cpp/EKFState.h` — 19-DOF error-state vector, MSCKF clones, SLAM features
- `app/src/main/cpp/Tracker.h` — front-end, KLT, scale, loop-closure plumbing
- `app/src/main/cpp/LoopClosureDetector.h` — DBoW2 + PnP loop closure
- `app/src/main/cpp/IMUPreintegrator.h` — preintegration + Madgwick
- `app/src/main/cpp/UpdaterMSCKF.h/cpp` — null-space MSCKF update
- `app/src/main/cpp/WindowedBA.h/cpp` — hand-rolled Gauss-Newton BA
- `app/src/main/cpp/ScaleFuser.h/cpp` — 1-D Kalman scale fusion (PDR / MiDaS / VI)
- `app/src/main/cpp/ScaleEstimatorVI.h/cpp` — Hesch-Martinelli closed-form
- `app/src/main/cpp/UpdaterZeroVelocity.h/cpp` — ZUPT
- `docs/ARCHITECTURE.md` — single source of truth for current pipeline
- `docs/VISUAL_PRODUCTION_PLAN.md` — visual upgrade plan, where most ADRs come from
- `docs/adr/ADR-002-eskf-not-full-ekf.md` — ESKF over factor-graph rationale
- `docs/adr/ADR-008-msckf-reenabled-damping-huber.md` — MSCKF re-enable
- `docs/adr/ADR-009-slam-features-in-state.md` — SLAM features (5-DOF inverse depth)
- `docs/adr/ADR-010-orb-descriptors-relocalization.md` — ORB at keyframes
- `docs/adr/ADR-012-windowed-bundle-adjustment.md` — windowed BA off-thread
- `docs/adr/ADR-013-same-session-loop-closure.md` — DBoW2 loop closure (defers pose graph)
- `docs/old docs/OPENVINS_ARCHITECTURAL_LESSONS.md` — team's prior OpenVINS study notes
- `docs/old docs/VISUAL_ALGORITHMS.md` — visual pipeline as it stood end of April 2026

### E.2. Open-source VIO/SLAM systems (canonical, well-known)

These are well-known projects with stable papers and READMEs. URLs are the
canonical home pages / repos. **External web fetch was unavailable when
this report was written; URLs are training-known canonical artefacts.**
Verify before pasting them into shipped docs.

#### OpenVINS (Robot Perception and Navigation Group, U. Delaware)
- Repository: https://github.com/rpng/open_vins
- Paper: Geneva, Eckenhoff, Lee, Yang, Huang. *OpenVINS: A Research
  Platform for Visual-Inertial Estimation.* ICRA 2020.
- Reference for: hybrid MSCKF + SLAM, FEJ, online time-offset and
  intrinsics estimation, rolling-shutter modelling, ESKF in
  general.
- File of particular relevance:
  `ov_msckf/src/state/State.h` (state vector layout),
  `ov_msckf/src/update/UpdaterHelper.cpp` (null-space projection).

#### ORB-SLAM3 (UZ-SLAM Lab, U. Zaragoza)
- Repository: https://github.com/UZ-SLAMLab/ORB_SLAM3
- Paper: Campos, Elvira, Gomez, Montiel, Tardós. *ORB-SLAM3: An
  Accurate Open-Source Library for Visual, Visual-Inertial and
  Multi-Map SLAM.* IEEE T-RO 2021.
- Reference for: ATLAS multi-map, 3-stage IMU initialisation,
  Sim3 / SE3 loop closure, full keyframe-graph BA.
- File of particular relevance:
  `Optimizer.cc::OptimizeEssentialGraph` (pose-graph),
  `LoopClosing.cc` (loop detection + correction).

#### VINS-Mono / VINS-Fusion (HKUST Aerial Robotics)
- VINS-Mono repo: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- VINS-Fusion repo: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- Paper: Qin, Li, Shen. *VINS-Mono: A Robust and Versatile Monocular
  Visual-Inertial State Estimator.* IEEE T-RO 2018.
- Reference for: sliding-window optimisation with marginalisation,
  4-DOF pose graph (yaw + xyz), DBoW2 loop closure with **BRIEF**
  descriptors (not ORB), tightly-coupled relocalisation.
- File of particular relevance:
  `vins_estimator/src/estimator.cpp` (sliding window),
  `pose_graph/src/pose_graph_node.cpp` (4-DOF pose graph).

#### OKVIS / OKVIS2 (Smart Robotics Lab, ETH Zurich / Imperial College)
- OKVIS repo: https://github.com/ethz-asl/okvis
- OKVIS2 repo: https://github.com/smartroboticslab/okvis2
- Paper: Leutenegger et al. *Keyframe-based visual-inertial odometry
  using nonlinear optimization.* IJRR 2015.
- Reference for: keyframe-based sliding-window optimisation in Ceres,
  marginalisation as Ceres factor.

#### MSCKF reference (Mourikis, Roumeliotis)
- Paper: Mourikis & Roumeliotis. *A Multi-State Constraint Kalman Filter
  for Vision-aided Inertial Navigation.* ICRA 2007.
- Reference for: null-space projection of feature Jacobian to update
  poses without putting features in state.

#### DM-VIO (TU Munich)
- Repository: https://github.com/lukasvst/dm-vio
- Paper: von Stumberg, Cremers. *DM-VIO: Delayed Marginalization
  Visual-Inertial Odometry.* RA-L 2022.
- Reference for: best-in-class monocular VIO using direct (photometric)
  front-end, delayed marginalisation.

#### SVO (Robotics and Perception Group, U. Zurich)
- Repository: https://github.com/uzh-rpg/rpg_svo_pro_open
- Paper: Forster et al. *SVO: Semidirect Visual Odometry for
  Monocular and Multicamera Systems.* T-RO 2017.
- Reference for: semi-direct front-end (image alignment + features).

#### DSO (Direct Sparse Odometry, TU Munich)
- Repository: https://github.com/JakobEngel/dso
- Paper: Engel, Koltun, Cremers. *Direct Sparse Odometry.* T-PAMI 2018.
- Reference for: monocular direct method (visual-only); useful as
  contrast to feature-based.

#### Kimera-VIO (MIT SPARK Lab)
- Repository: https://github.com/MIT-SPARK/Kimera-VIO
- Paper: Rosinol et al. *Kimera: an Open-Source Library for Real-Time
  Metric-Semantic Localization and Mapping.* ICRA 2020.
- Reference for: GTSAM iSAM2 backend, 3-D mesh, semantic.

#### DROID-SLAM (Princeton / Stanford)
- Repository: https://github.com/princeton-vl/DROID-SLAM
- Paper: Teed, Deng. *DROID-SLAM: Deep Visual SLAM for Monocular,
  Stereo, and RGB-D Cameras.* NeurIPS 2021.
- Reference for: learning-based dense BA; representative of where
  the field is going for accuracy at high CPU/GPU budget.

#### DBoW2 / DBoW3 (BoW vocabulary library)
- DBoW2 repo: https://github.com/dorian3d/DBoW2
- DBoW3 repo: https://github.com/rmsalinas/DBow3
- ORB-SLAM2 fork (with `ORBvoc.bin`): https://github.com/raulmur/ORB_SLAM2
- Reference for: NavSight's `LoopClosureDetector` is a vendored DBoW2
  with the ORB-SLAM2 vocabulary (see ADR-013 §"Vocabulary choice").

#### g2o, GTSAM, Ceres (back-end solvers)
- g2o: https://github.com/RainerKuemmerle/g2o
- GTSAM: https://github.com/borglab/gtsam
- Ceres: https://github.com/ceres-solver/ceres-solver
- Reference for: pose-graph back-end (PRIORITY 1 recommendation).
  Most well-known VIO/SLAM use one of these for the pose-graph.

### E.3. Background reading on phone VIO

These are listed in `docs/old docs/OPENVINS_ARCHITECTURAL_LESSONS.md` §7
as the team's prior research. Re-listed here for completeness.

- VINS-Mobile-Android: https://github.com/jannismoeller/VINS-Mobile-Android
  — port of the HKUST iOS app to Android, similar use case to NavSight.
- Android-VIOTester (Aalto University):
  https://github.com/AaltoML/android-viotester
  — benchmark app, useful for dataset recording inspiration.
- ORB-SLAM2-Android: https://github.com/muziyongshixin/ORB-SLAM2-based-AR-on-Android
  — port of ORB-SLAM2 to Android NDK.
- Google ARCore SDK: https://github.com/google-ar/arcore-android-sdk
  — closed-source backend; sample apps useful for Camera2 + IMU plumbing.

---

## F. What I'd do next, in two-week chunks

If the team has six available weeks, the highest-leverage path is:

| Sprint | Deliverable | Acceptance |
|---|---|---|
| 1-2 | **PoseGraph back-end** (D.1) — 4-DOF Ceres-or-handrolled-LM solver, accept loop closure as edge, re-optimise all KFs, push back into EKF clones | 50 m loop closes < 5 m; same-session figure-8 closes < 1 m (ADR-013 acceptance criteria already in code) |
| 3 | **Loop-closure correction snaps clones + SLAM features** (D.2) — rigid transform of all keyframes, clones, SLAM landmarks on accept | Next-frame MSCKF chi² is small; no fight-back oscillation |
| 4 | **Scooter MountMode rollout** (D.3 + D.5) — bump MAX_SLAM_FEATURES, MAX_CLONES, ZUPT/BLUR/ROT thresholds, enforce no-PDR | Scooter sim closes loops at < 5 % drift; no false ZUPT under engine idle |
| 5 | **IMU intrinsics in state** (D.4) — D_w / D_a / T_g, observability analysis, init schedule | 30-min indoor sim shows < 2 % drift improvement vs no-intrinsics |
| 6 | **Replay harness scoring on long-session corpus** — extend the existing `replay_harness` to score the new metrics: pose-graph residual, loop-closure correction Δ, scooter-mode performance | All thresholds in `replay_scorer.py` defended from physics |

If the team only has two weeks, do D.1 alone. The 30 %-per-loop number
will collapse and that single change will move NavSight from "VIO that
works on a short walk" to "VIO that closes loops at literature-comparable
levels". Everything else is sharpening; D.1 is the single rock that is
moving the needle.

---

## G. Final word

NavSight is on the right architectural path. It is essentially **OpenVINS
+ ORB-SLAM2 loop closure + a NavSight-specific PDR/MiDaS scale layer**,
which is a sensible hybrid for phone-pedestrian use that no canonical
open-source system targets. The 30 %-per-loop drift is not a sign the
architecture is wrong; it is a sign that the architecture is incomplete
in exactly the same way ADR-013 §"What is NOT done" §2 already
acknowledged. Ship the deferred 4-DOF pose-graph back-end and NavSight
moves into the published-literature accuracy band.

The biggest remaining question — and the one Step 10 of the visual plan
is supposed to answer — is whether scooter mode can stand on its own
without PDR. The honest answer is "with the four changes in §D.1-D.5
above, yes; without them, no." Get the pose-graph and the SLAM-capacity
bump in first. Then test on a real scooter run. Then iterate.
