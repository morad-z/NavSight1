# ADR-013 — Same-session loop closure (DBoW2)

**Status:** Accepted
**Date:** 2026-05-04
**Owner:** Morad Zubidat (sensor fusion)
**Companion:** ADR-008 (MSCKF re-enabled with damping + Huber), ADR-009
(SLAM features in EKF state), ADR-010 (ORB descriptors at keyframes for
relocalization), ADR-011 (Adaptive front-end robustness), ADR-012
(Local windowed bundle adjustment).
**Supersedes:** the loop-closure portion of ADR-006.

## Context

After ADR-010 the front-end carries ORB descriptors at every keyframe
and a brute-force matcher recovers the FeatureManager ID space across
KLT loss events — but the search horizon is hard-capped at the **last
50 keyframes** (~30–90 s of session, depending on keyframe cadence).
After ADR-012 the BA worker refines pose + landmark geometry across the
window of 5 most-recent keyframes. Both buy bounded drift on the short
horizon, but neither can do what an EKF + sliding-window BA structurally
cannot: **detect that the user has returned to a place visited earlier
in the same session, and snap the trajectory closed against that
re-visit**.

A 5-minute walk in the Carmel neighbourhood that loops back to the
starting building accumulates 2–4 m of unbounded yaw + position drift
under the post-ADR-012 baseline. The drift is bounded *between*
keyframes by gyro stability + visual yaw updates; it is bounded *across
the BA window* by the windowed solve; it is **unbounded across the
session** because the BA window slides past the early keyframes long
before the user returns near them. ORB-SLAM2's contribution to the
state of the art (Mur-Artal et al. 2017) is precisely the missing
mechanism: a place-recognition step that reaches across the entire
session keyframe history, gates aggressively on geometric verification,
and injects a single high-confidence correction per detected loop.

Step 7 of `docs/VISUAL_PRODUCTION_PLAN.md` (line 695, "Same-session
loop closure (DBoW2)") fixes this with a 1 Hz background place-query
worker that consumes the ADR-010 keyframe descriptors, scores them
against a pre-trained DBoW2 vocabulary, geometrically verifies the top
candidates with `cv::solvePnPRansac`, and feeds the surviving
correction through the existing `EKFState::updateRelativePose` channel
ADR-008 already uses for the visual-odometry rotation update.

## Decision

### Vocabulary choice — DBoW2 vendored, ORB-SLAM2 ORBvoc.bin shipped

The plan §"Why DBoW2" already commits to bag-of-words against ORB
descriptors. The vendoring decision is between three paths:

- **Path A (chosen):** vendor the upstream DBoW2 C++ source under
  `third_party/DBoW2/`, ship ORB-SLAM2's `ORBvoc.bin` (~10 MB) as an
  Android asset, load at app start.
- Path B: depend on a system-installed DBoW2 via `find_package`. No
  Android tooling, breaks the Android build.
- Path C: re-train a smaller vocabulary from on-device data. Out of
  scope for this iteration — ORB-SLAM2's vocabulary was trained on a
  large diverse corpus and matches Step 4's ORB tuning out of the box;
  re-training would push Step 7 by weeks and risks regressing a
  battle-tested artefact.

The foreground scout looking at GitHub mirrors of DBoW2 picked Path A
on three criteria: (i) the DBoW2 source builds clean against OpenCV
4.x with no Eigen / Boost / non-OpenCV deps, (ii) ORBvoc.bin is the
de-facto vocabulary used by ORB-SLAM2/3, VINS-Mono, and OpenVSLAM, so
its retrieval quality on ORB-from-Step-4 descriptors is well
characterised in the literature, (iii) the binary format is a simple
flat layout that loads in <500 ms even on a Snapdragon 695 cold start
— well below the app-start budget. The license (BSD-3) is compatible.

DBoW3 was considered. It adds binary-vocabulary I/O speedups and a
slightly cleaner API, but the on-device gain is small (vocabulary load
is a one-shot cost), the dependency surface area is larger (its build
expects more recent CMake conventions than Android NDK 25 ships), and
its retrieval quality on ORB descriptors is marginal-equal. Carrying
two BoW libraries in the source tree to test a marginal upgrade is not
the right trade today. We can revisit if on-device telemetry shows the
DBoW2 query is the bottleneck.

### Per-keyframe BoW vector

At keyframe creation the new `LoopClosureDetector::addKeyframe` is
called from the Tracker (Agent B's wiring) with the same 250-row,
32-col CV_8U ORB descriptor matrix the ADR-010 keyframe ring already
carries plus the Step-3b–promoted SLAM 3D points and the EKF's current
world↔cam pose. The detector transforms the descriptors into a BoW
vector and pushes it into DBoW2's inverted index, alongside a parallel
record of `{kf_id, ts_ns, descriptors, keypoints, pts3d_world,
R_world_cam, t_cam_world}`. Cost is ~3–5 ms per keyframe on a
Snapdragon 695 (linear in descriptor count, dominated by the cluster-
assignment tree walk in DBoW2).

### Place query at 1 Hz

A 1 Hz background place-query worker — not the camera thread — pulls
the most recent keyframe's BoW vector and queries the inverted index
for the top-K=3 candidates by tf-idf cosine similarity, with a hard
constraint that the candidate's stored timestamp be **older than the
query's by at least 30 s** (`temporal_exclusion_ns = 30 * 10⁹`). 30 s
is the §3 plan default and corresponds to "long enough that the user
must have left the immediate neighbourhood" at typical walking speed
(~5 km/h ≈ 1.4 m/s, so 30 s ≈ 40 m clear of self).

The 1 Hz cadence (vs per-keyframe) is deliberate: BoW queries cost
~5–8 ms each plus the geometric-verification cost, and a real loop is
a once-per-session event on the timescales we care about. Polling at
1 Hz is a > 100x cost reduction vs per-keyframe with no measurable
recall loss — a 30 s exclusion window already guarantees the same
re-visit will sit in the candidate set across 30 consecutive 1 Hz
queries.

The top-K is 3, not 1, so the geometric verification stage gets a
small fan-out and a high-BoW-score candidate that fails geometry
doesn't block a slightly-lower-score candidate from succeeding.

### Geometric verification — PnP inlier floor of 30

Each top-K candidate goes through:

1. `cv::BFMatcher` with `cv::NORM_HAMMING`, `crossCheck=false`,
   `knnMatch(k=2)` — same matcher Step 4 uses.
2. **Lowe ratio test at 0.75** — same threshold ADR-010 §"Match
   pipeline" pins, identical noise model.
3. `cv::solvePnPRansac` against the candidate keyframe's stored
   `pts3d_world` and the query keyframe's keypoint locations,
   intrinsics `fx, fy, cx, cy` from the camera model.
4. **Accept iff PnP inliers ≥ 30**.

The 30-inlier floor is identical to ADR-010's relocalization gate.
Using one number across the two paths is deliberate: a loop-closure
correction must clear *at least* the same evidence bar that a within-
session reloc clears, and a future regression on this gate is then
caught simultaneously by both `tests/cpp/test_orb_reloc.cpp` and
`tests/cpp/test_loop_closure.cpp`.

If the PnP solve fails or the inlier count is below 30, the candidate
is silently dropped. The next 1 Hz query will pick it up again if it
is still in the BoW top-K — i.e. drops are *not* permanent, they
defer the decision until evidence accumulates.

### Correction injection — through `EKFState::updateRelativePose`

The accept path computes the relative pose between the current
keyframe and the matched keyframe (`R_now_to_match`,
`t_now_to_match`) plus the matched keyframe's stored world↔cam pose
(`R_world_cam_match`, `t_cam_world_match`) and feeds them as a
**relative-pose constraint** into the EKF through the same
`EKFState::updateRelativePose` channel ADR-008 / ADR-009 use for the
per-frame `R_vo` visual-rotation update.

The correction is **damped over 10 frames per ADR-006**: the EKF
applies the constraint with a covariance derived from the PnP inlier
count, and the next ~10 visual-update cycles pull the EKF state
toward consistency. There is **no direct EKF mean / covariance
mutation from the loop-closure thread**. ADR-006 §"Why not direct EKF
mutation" is the load-bearing constraint here, identically to
ADR-012's BA reconciliation: a side-channel write produces 5–11 m
teleportations the filter cannot recover from.

### Threading model — separate 1 Hz worker, atomic + mutex handoff

The same pattern Step 6 uses for the BA worker:

1. **At each new keyframe** the Tracker (Agent B's wiring) calls
   `LoopClosureDetector::addKeyframe` to push the BoW vector into the
   inverted index. This is on the camera thread and bounded under
   ~5 ms.
2. **A separate worker thread** wakes at 1 Hz, copies the most recent
   keyframe descriptor / keypoint cv::Mat under a brief
   `std::mutex` (the same lock-only-the-mutator pattern ADR-012
   describes for the EKF / FeatureManager snapshot APIs), then runs
   the BoW query + geometric verification off-thread.
3. **On accept**, the worker publishes a `LoopMatch` struct under
   `loop_result_mutex_` and sets a `std::atomic<bool> loop_pending_`
   (release-store) so the next camera-thread step sees
   `pending=true`. The Tracker (Agent B's wiring) consumes the
   pending result on the next keyframe creation, before launching
   the next BA round, and feeds it through `updateRelativePose`.
4. **One query at a time.** If the worker is still in flight when
   the next 1 Hz tick arrives, the tick is silently dropped. This
   matches the BA worker's "skip if previous round in flight"
   policy from ADR-012; the canary log line surfaces over-long
   queries the same way.

The worker thread is joined (not detached) by the destructor and
`Tracker::reset()` so the BoW database is not torn down underneath
an in-flight query.

### What is NOT done in this ADR

- **No cross-session persistence.** The BoW database is in-memory,
  populated from the live keyframe stream, and discarded on app
  exit. Cross-session loop closure (load yesterday's keyframes,
  recognise today that you're at the same desk) is explicitly out
  of scope per `docs/ARCHITECTURE.md` §7. A future ADR can add a
  serialised on-disk database when the use case justifies the
  privacy + storage cost.
- **No 4-DOF pose graph optimisation.** A successful loop closure
  in this ADR injects a *single* relative-pose constraint into the
  EKF and lets the windowed BA + EKF reconciliation absorb it across
  the next ~10 frames. A full pose-graph back-end (gauge-fixed
  4-DOF: x, y, z, yaw — roll/pitch are gravity-observed by the IMU)
  that re-optimises every keyframe pose against accumulated loop
  constraints is **Step 7.5**, deferred to the scooter-mode work in
  Step 10 where the longer trajectories make the case for it.
- **No DBoW3.** See "Vocabulary choice" above; marginal upgrade,
  larger dependency surface, deferrable.
- **No SuperPoint / deep descriptors.** ADR-010 already defers
  these; same argument applies here.
- **No vocabulary re-training from on-device data.** ORB-SLAM2's
  ORBvoc.bin is the shipped artefact; on-device training is a
  research project, not a production change.
- **No direct EKF mean / covariance mutation.** ADR-006 forbids it;
  this ADR honours that. Correction injection goes through
  `updateRelativePose` and is damped across 10 frames.

## Consequences

**Positive**

- Long-session yaw + position drift on a closed-loop walk drops to
  the loop-closure-gap acceptance bar (< 1 m on the §"Acceptance
  criteria" synthetic figure-8). Without this ADR the same scenario
  drifts 2–4 m unbounded.
- The geometric verification gate (≥ 30 PnP inliers) is identical
  to the ADR-010 relocalization gate, so a regression on the gate
  is caught simultaneously by `test_orb_reloc.cpp` and
  `test_loop_closure.cpp` — one knob, one decision boundary,
  pinned in two test files.
- The per-keyframe cost of BoW indexing (~3–5 ms) is bounded and
  surfaces in the existing perf-LOGI envelope; the 1 Hz query
  worker runs entirely off the camera thread; the EKF correction
  injection reuses the existing `updateRelativePose` channel —
  zero new EKF math.
- Step 7's keyframe descriptor record is a strict superset of
  ADR-010's `KeyframeDescriptors`. The two features share the
  per-keyframe extraction work, the brute-force matcher, and the
  Lowe-ratio + RANSAC verification stages.
- A failed BoW query (no candidates above similarity threshold,
  no candidate above the 30-inlier PnP floor, query times out)
  is silently dropped and the system degrades to ADR-012-only
  behaviour. The acceptance failure mode is "no correction" — never
  "wrong correction".

**Negative / accepted**

- The vocabulary blob adds ~10 MB to the APK. This is the cost of
  shipping a battle-tested artefact instead of training one
  on-device; cheaper than the months a re-training project would
  cost in calendar time and risk.
- A 1 Hz cadence means the worst-case latency from "user crosses
  the loop point" to "EKF receives correction" is ~1 s of query
  delay + the geometric-verification cost (~10–30 ms) + 10 frames
  of damped EKF reconciliation (~333 ms at 30 Hz). A faster
  cadence would shave the first term but at a > 10x compute cost
  for an event that fires 0–3 times per session; the latency is
  acceptable.
- One query at a time means a slow query (e.g. a textureless place
  with many candidates queueing through PnP) silently skips
  subsequent 1 Hz ticks. The "loop_query: skipped (in_flight)"
  log line surfaces this — same canary pattern ADR-012 uses for
  the BA worker.
- The 30 s temporal exclusion gate is a hard parameter — too short
  and the detector self-loops on the most recent keyframe; too
  long and a tight loop (< 30 s walk-around) is missed. 30 s is
  the §3 plan default; on-device telemetry will tell us if the
  Haifa walk-test corpus needs a shorter window. The Re-validation
  criteria below pin the canaries for the change.
- The vocabulary asset is loaded synchronously at app start. A
  corrupt asset surfaces as `loadVocabulary` returning false and
  `isReady()` staying false; the Tracker (Agent B's wiring) skips
  loop-closure attempts in that state. The system degrades to
  ADR-012-only behaviour gracefully — the vocabulary failure is
  observable but not fatal.

## Re-validation criteria

Before flipping any ADR-013 gate (the vocabulary path, the 30 s
temporal exclusion window, the top-K=3 fan-out, the 30 PnP-inlier
floor, the Lowe 0.75 ratio, the 1 Hz query cadence, the 10-frame EKF
damping window) from its current default, the following must hold:

1. **Build clean.** `gradlew :app:assembleDebug --offline` runs
   clean with the new `LoopClosureDetector.{h,cpp}` (Agent A's
   rewrite), the vendored `third_party/DBoW2/` source, the
   ORBvoc.bin asset, the Tracker / native-lib wiring (Agent B's
   work), and the new tests on a fresh tree.
2. **Unit contract.** `tests/cpp/test_loop_closure.cpp` passes —
   five cases pinning vocabulary-load failure, addKeyframe count
   semantics, the 30 s temporal exclusion gate, geometric
   verification rejection of BoW-only matches, and the happy-path
   accept on a clean re-visit.
3. **Synthetic figure-8 closes < 1 m.** The §"Acceptance criteria"
   replay test on a 100 m figure-8 walk that returns to start
   produces a closed-loop gap **< 1 m**. This is the plan's
   stated Step-7 acceptance bar.
4. **Zero false positives on a straight walk.** A straight 100 m
   walk (no loop) reports **zero** `loop_closure_accept` events
   in the simulation_data event_summary. The similarity threshold
   + 30 s temporal gate + 30 PnP-inlier gate must reject
   same-direction matches; a single false positive on this corpus
   means one of the gates has slipped.
5. **No baseline regression on EKF-only behaviour.** Disabling the
   loop-closure wiring (a one-line guard at the top of the 1 Hz
   worker) must reproduce the ADR-012 closed-loop drift exactly —
   the loop-closure path is strictly additive on top of the BA
   correction. A regression with loop closure off means the
   `addKeyframe` call on the camera thread is interacting with the
   keyframe ring in an unintended way.
6. **event_summary signal.** A real return-to-start walk on the
   on-device test corpus produces `loop_closure_accepts > 0` in the
   simulation_data JSON. A clean walk that visibly closes a loop
   but reports zero accepts means the runtime gates are too tight
   and the parameters need re-tuning before this ADR ships.

## 2026-05-04 (later) — Absolute-pose injection channel landed

Step 7's original integration assumed every accepted loop match would
have its matched clone still in the EKF window. In practice the
temporal exclusion (30 s) is always larger than the clone window
(~5-10 s), so the relative-pose / relative-rotation channels never
fired. `EKFState::updateAbsolutePose` closes the gap: it consumes a
target world-frame IMU pose and corrects IMU-state directly via the
existing `applyMSCKFUpdate` primitive (Joseph form, ADR-008 damping +
Huber inherited). Outer χ² gate at 22.5 (wide) protects against
wildly wrong loop matches before damping fades them in.

Two new event_summary fields: `loop_closure_chi2_rejected` and
`loop_closure_corrections_applied` — completes the accounting from
detection through correction.
