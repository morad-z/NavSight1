# ADR-010 — ORB descriptors at keyframes for relocalization

**Status:** Accepted
**Date:** 2026-05-04
**Owner:** Morad Zubidat (sensor fusion)
**Companion:** ADR-008 (MSCKF re-enabled with damping + Huber), ADR-009
(SLAM features in EKF state).

## Context

The visual front-end shipped through ADR-008 / ADR-009 (Plan Step 3)
relies entirely on KLT optical flow to carry feature identities frame
to frame. KLT is fast and dense, but it is also fragile: a brief
occlusion (a hand crossing the lens, a person walking past in a
corridor), a fast head turn that produces motion blur, or a tunnel /
elevator with poor lighting will lose every track in a single frame.
Once the tracks are gone, the ID space is gone with them — the next
frame seeds new corners with new IDs, FeatureManager has no link back
to the previous keyframe, and the only thing keeping the polyline
alive is dead-reckoning on Madgwick + IMU. Long-session reliability
is capped by how often this happens.

Step 4 of `docs/VISUAL_PRODUCTION_PLAN.md` (line 557, "ORB descriptors
at keyframes — relocalization + loop foundation") fixes this with
descriptor-based matching against recently-stored keyframes. KLT keeps
running as the primary tracker; ORB exists to recover the ID space
after KLT has failed.

## Decision

### ORB extraction at keyframes

`cv::ORB::create` is configured for ~250 features per keyframe
(`nfeatures=250`, default scale pyramid, FAST threshold left at
default). Input is the keyframe gray image after a Gaussian pre-blur
with `σ ≈ 1.0` to suppress single-pixel FAST responses on textured
asphalt. Extraction runs **only on keyframes** — the per-keyframe
budget is ~5–8 ms at 640x480 on Morad's phone, which fits the +6%
CPU envelope inherited from Step 3. Per-frame ORB extraction was
rejected (cost-prohibitive vs. that budget; KLT is the per-frame
tracker by design).

This step is **vocabulary-free**. DBoW2 / bag-of-words clustering is
deferred to Step 7 (loop closure) where it matters; for relocalization
against the last few keyframes a brute-force matcher is sufficient
and avoids carrying a vocabulary blob in the APK.

### Storage

A ring buffer of `KeyframeDescriptors` (defined in
`app/src/main/cpp/KeyframeDescriptors.h`) is held inside
`FeatureManager` and capped at the **last 50 keyframes**. Each record
holds `keyframe_id`, `timestamp_ns`, the parallel arrays
`keypoints` / `descriptors` (CV_8U, 32 cols, ORB binary) /
`feature_ids` (FeatureManager IDs aligned to the closest tracked KLT
corner at storage time, or `-1` when no alignment was possible).

Eviction is FIFO by `keyframe_id`. The 50-keyframe cap gives
~400 KB of actual descriptor payload (250 × 32 B × 50) and ~10 MB
headroom counting keypoint metadata and Mat overhead — well inside
the on-device memory ceiling. Older keyframes are dropped silently;
the existing keyframe history that backs SLAM anchors is unaffected
because that history lives on the EKF clone path, not in this buffer.

### Relocalization trigger

KLT inlier count below `MIN_INLIERS / 2` (i.e. < 4 inliers, with
`MIN_INLIERS = 8` from `Tracker.h:259`) for **≥ 3 consecutive
frames** (~100 ms at 30 Hz). The 3-frame debounce filters out
single-frame noise — a one-off bad RANSAC iteration on a textureless
patch should not invoke a 5-keyframe descriptor search. The
consecutive-frame counter lives on the Tracker side; ADR-010
specifies the gate, not the field.

### Match pipeline

1. `cv::BFMatcher` with `cv::NORM_HAMMING`, `crossCheck=false`,
   `knnMatch(k=2)` against each of the **last 5 keyframes'** stored
   descriptors. Five is a tradeoff: enough horizon to cover a typical
   2–3 second occlusion, few enough to keep the per-trigger cost
   under one frame budget.
2. Lowe ratio test at **0.75** (Lowe 2004, *Distinctive Image
   Features from Scale-Invariant Keypoints*, §7.1) on the two nearest
   neighbours per query descriptor. Matches with `d1 / d2 ≥ 0.75` are
   discarded as ambiguous.
3. `cv::findEssentialMat` RANSAC with threshold **1.5 px**, matching
   the existing `Tracker.h:254 RANSAC_THRESH = 1.5` so the inlier
   geometry uses the same noise model as the per-frame KLT path.
4. Pick the **best keyframe by RANSAC inlier count**. Accept the
   relocalization if and only if that count is **≥ 30 inliers**. The
   30-inlier floor was chosen to be comfortably above
   `MIN_INLIERS = 8` (the per-frame essential-matrix floor) so that a
   reloc decision carries strictly more evidence than a normal frame
   step.

### Acceptance action

On accept:

- Re-adopt the matched keyframe's `feature_ids` for the inlier
  matches so the next KLT step continues with the **same
  FeatureManager IDs** the system had before the loss event. This is
  the whole point — no ID churn, no SLAM-feature anchor invalidation
  cascade, no MSCKF track resets.
- Clear the consecutive-low-inlier counter on the Tracker side.
- Tracker output sources do **not** change. Heading is still owned by
  Madgwick. Position is still owned by `EKFState::global_t_`. ADR-010
  is an ID-recovery mechanism, not a pose source.
- Optional: a yaw injection through the existing
  `EKFState::updateGravityAlignedYaw` channel **only if** a clean
  per-keyframe sigma can be derived (matched keyframe count, RANSAC
  inlier ratio, and the keyframe's stored Madgwick yaw at storage
  time). If a clean sigma cannot be derived, the yaw injection is
  skipped — silently degrading to ID-recovery-only is correct;
  injecting a high-variance yaw with a guessed sigma is the kind of
  thing that re-creates the ADR-006 teleportation regime.

### What is NOT done in this ADR

- **No DBoW2 vocabulary.** Bag-of-words indexing is Step 7 (loop
  closure) where it earns its weight against the full keyframe
  history, not the last 5.
- **No SuperPoint / deep descriptors.** Deferred to a future ADR if
  on-device data shows ORB is the bottleneck. The plan's Step 4
  rationale (§"Why ORB") already explicitly defers SuperPoint.
- **No per-frame ORB extraction.** Per-keyframe only; per-frame is
  cost-prohibitive at the +6% CPU envelope from Step 3.
- **No SLAM-feature re-promotion** from a relocalized keyframe. SLAM
  features that were marginalized during the loss event stay gone;
  re-promotion would require re-running the chirality + RMSE gate
  from ADR-009, which is out of scope here.

## Consequences

**Positive**

- KLT loss events that previously bled into multi-second
  dead-reckoning are now bounded: as soon as the camera sees a recent
  keyframe again, the ID space and the SLAM-anchor graph survive.
- Storage cost is fixed and small (~400 KB descriptor payload), so
  the failure mode is predictable and the cap can be tuned without
  worrying about OOM.
- The acceptance gate (≥ 30 inliers, post-Lowe, post-RANSAC) is
  strictly stronger than the per-frame essential-matrix gate, so a
  successful reloc is a higher-confidence event than the frames
  surrounding it. Combined with no direct EKF mean injection, this
  means a false reloc cannot teleport the polyline.
- Step 7 (loop closure with DBoW2) inherits the keyframe descriptor
  buffer wholesale — the per-keyframe extraction work is amortized
  across both features.

**Negative / accepted**

- Per-keyframe cost: 5–8 ms ORB extraction. Surfaces in the existing
  perf-LOGI added in the perf pass earlier today (Step 4 of the
  visual plan day). Acceptable inside the +6% CPU envelope.
- Recovery latency: the 3-frame debounce + descriptor search means
  the earliest a reloc can fire is ~4 frames (~133 ms at 30 Hz)
  after the camera uncovers. Faster recovery would require running
  ORB on every frame, which the per-frame budget does not permit.
- 50-keyframe horizon is a hard ceiling. A user who walks for >50
  keyframes through a tunnel and emerges into a previously-seen
  scene will not relocalize via this path — that is what Step 7 is
  for. ADR-010 is explicitly the **short-horizon recovery**
  mechanism; long-horizon loop closure is a separate decision.
- Yaw injection is gated and may not fire on every successful reloc.
  This is a deliberate choice — silent ID recovery is the safer
  default, and a future ADR can lift the gate once a clean sigma
  estimator is implemented and validated.

## Re-validation criteria

Before flipping any ADR-010 gate (the 50-keyframe cap, the 3-frame
debounce, the 30-inlier floor, the ratio threshold, the last-5
keyframe match horizon) from its current default, the following must
hold:

1. **Build succeeds.** `gradlew :app:assembleDebug --offline` runs
   clean with the new `KeyframeDescriptors.h` included from the
   FeatureManager / Tracker translation units (Agent A's work).
2. **ORB compute budget.** ORB extraction stays under **8 ms per
   keyframe at 640x480** measured on Morad's phone via the perf-LOGI
   surface added earlier today. A regression here means dropping the
   target feature count from 250 toward 200, not raising the budget.
3. **Unit contract.** `tests/cpp/test_orb_reloc.cpp` (Agent B's
   synthetic two-frame test) passes — the contract there pins the
   per-record shape of `KeyframeDescriptors` and the accept/reject
   semantics of the match pipeline.
4. **On-device occlusion sim.** On the 14 m walk + occlusion sim
   Morad will record on-device, the system must recover KLT tracking
   within **≤ 5 frames** of camera uncovering, and the closed-loop
   gap must stay **≤ baseline_walk_001.json (1.79 m)** — i.e. the
   Step 3b acceptance bar from ADR-009 must not regress.
