# Phase 2: Productization Plan

**Entry trigger:** Phase 1 (`docs/study/post_v19_sprint_plan.md`) complete and tagged `phase1-complete`. All 7 success criteria from Phase 1 §7.4 met.

**Exit trigger:** NavSight runs reliably on both pedestrian AND scooter use cases, with cross-session map persistence, in production-quality form.

## Phase 1 → Phase 2 Architectural Recap

By the time Phase 2 starts, NavSight has:
- ✅ Correct EKF math (sign conventions, propagation, Jacobians all fixed)
- ✅ Single-trajectory architecture (`global_t_` is the user-facing pose, EKF follows)
- ✅ SLAM feature re-anchoring (features survive clone marginalization)
- ✅ Pose-graph back-end (loop closure spreads correction across all keyframes)
- ✅ Persistent landmark map (continuous drift correction every frame)
- ✅ Camera overlay (KLT dots + 3D SLAM points + LOOP CLOSURE flash)

What remains for production: **modal detection (which mount?), scooter-specific tuning, persistence, performance, edge cases.**

---

## Step 1 — Scooter MountMode framework

**Scope:** Detect the mount/use-case (pedestrian / scooter / car / handheld) and adapt the pipeline accordingly. Different motion profiles need different gates, feature counts, and update rates.

### 1.1 MountMode detection

Three signals to discriminate:
- **Step rate** from PDR — non-zero → pedestrian; zero with motion → scooter/car/static
- **Speed magnitude** from landmark-map velocity (Step 2 below) — 0–2 m/s → pedestrian, 2–10 m/s → scooter, 10+ m/s → car
- **IMU vibration spectrum** — pedestrian has 1–2 Hz stride-band peak, scooter has high-frequency wheel-vibration band, car is dampened

Implementation:
- New file `app/src/main/cpp/MountModeDetector.cpp/h`
- Output: `enum class MountMode { Pedestrian, Scooter, Car, Static, Unknown }`
- Hysteresis: require ≥ 3 s of consistent signals before switching mode
- Default: `Unknown` until enough data; behave as Pedestrian (current behavior)

### 1.2 Mode-specific tuning

When `MountMode::Scooter`:
- **PDR off** — disable step counting (no foot strikes; would mis-fire on wheel vibration)
- `MAX_SLAM_FEATURES` 12 → 25–30 — more long-lived 3D points (faster pose travel through scene)
- `MAX_CLONES` 11 → 15 — wider observation window (already done in Phase 1 Step 4b)
- ZUPT threshold tighter — scooter rarely truly stationary
- Blur tolerance higher — scooter vibration causes more motion blur per frame
- Gyro rotation gate higher — scooter handlebars tilt more freely

When `MountMode::Car`:
- All scooter settings + speed-bound velocity gate (no 100+ m/s outliers)
- Larger search radius for landmark map (50 m vs 30 m)

When `MountMode::Static`:
- Hold position; suppress visual updates that would inject jitter
- Drop frame rate to save battery (5 Hz instead of 30)

**Files:**
- `MountModeDetector.cpp/h` (new)
- `Tracker.h` — `MountMode current_mount_;` member, accessor
- `Tracker.cpp` — gates that switch by `current_mount_`
- `EKFState.h` — `MAX_SLAM_FEATURES` and `MAX_CLONES` become run-time configurable instead of `static constexpr`

**Validation:** Walk + ride scooter in same session; mode switches detected within 3 s. No false transitions during transient motion changes.

---

## Step 2 — Velocity from landmark map (scooter speed source)

**Scope:** Replace PDR-derived speed with persistent-landmark-derived velocity for scooter and car modes.

### Math

```
At each frame:
  query landmark map within 30 m radius
  match against current ORB features
  PnP RANSAC → absolute camera pose T_world_cam(t)
  v_world(t) = ( T_world_cam(t) - T_world_cam(t-Δt) ) / Δt
```

EKF wire-up: feed `v_world(t)` as a measurement update for the `v_G_` state with covariance derived from PnP residual covariance + landmark depth uncertainty.

### Speed for the user-facing UI

Replace `vio.stepFreq * vio.strideLength` with `‖v_world‖` (horizontal magnitude). For pedestrian mode keep the PDR-derived value (more accurate at slow walking speeds); for scooter/car mode use the landmark-map velocity.

**Files:**
- `Tracker.cpp` — compute `v_world` from landmark-map PnP, push to EKF as velocity measurement
- `EKFState.cpp` — new `updateVelocityWorld(const cv::Mat& v_world, const cv::Mat& cov)` method
- `VioData.kt` / `native-lib.cpp` — expose mode-aware speed in the UI field
- `NavSightViewModel.kt` — toggle speed-source by `MountMode`

**Validation:**
- Stationary on scooter → speed reads 0 ± 0.1 m/s
- Constant scooter ride at known speed → reading within 5%
- Acceleration profile during a known stop-and-go matches phone GPS reference (when available)

---

## Step 3 — Bad keyframe filter

**Scope:** Prevent corrupted keyframe poses (e.g., the km-range `target_p` we saw in v18) from polluting the DBoW2 database and the landmark map.

### Filter logic

When a new keyframe is about to be added:
- Check distance from previous keyframe — > 100 m on a single-frame transition is impossible at any reasonable speed → reject
- Check rotation delta — > 90° between consecutive keyframes → suspicious, reject
- Check 3D point cluster centroid — if dramatically off vs neighbors, reject

**Files:**
- `LoopClosureDetector.cpp` — sanity check before `addKeyframe`
- New `EventCounters` field: `keyframes_rejected_corruption`

**Validation:** No km-range `target_p` in any LC_ABS line across a 5-loop walk.

---

## Step 4 — MiDaS observability fix (DA3 swap DEFERRED)

> **2026-05-16 update — Step 4.2.2 (DA3 swap) DEFERRED indefinitely.**
> On-device benchmark on Samsung S21 Ultra (Exynos 2100, Mali-G78) ruled out
> the model swap path. See [[project_session_2026_05_10_v2_bench]]:
> V2 INT8 CPU **722 ms**, NNAPI **1907 ms** — both RED vs the 100 ms inference
> budget. V3 is larger than V2, so V3-Small would be ≥ V2 timings.
>
> Step 4 is now **only Step 4.2.1**: the affine-fit observability fix that
> makes the EXISTING MiDaS contribute to the scale anchor instead of bailing
> out 32/32 keyframes. DA3 stays on the shelf until either (a) a smaller
> distillation lands or (b) NavSight targets a higher-end SoC.

**Scope:** Replace the broken floor-plane fusion in `Tracker::applyDepthScaleConstraint` with a VI-Depth affine fit in disparity space (Step 4.2.1 below). Keep MiDaS v2.1 small as the depth model.

**Original Step 4 ambition (preserved for archeology — see deferred Step 4.2.2):** swap MiDaS for Depth-Anything V3 small. Reason for deferral above.

### 4.1 Why the (remaining) change matters

The chain from depth model to scooter speed is direct:

```
scooter speed (Step 2)
  ⇐ Δ(camera position) / Δt
  ⇐ PnP against persistent landmarks with known world positions
  ⇐ landmark world positions = triangulation × scale anchor
  ⇐ scale anchor today: MiDaS relative depth + 5 s PDR calibration loop
                        ↑ broken — floor heuristic bails 32/32 keyframes outdoors
```

If MiDaS keeps bailing out, every landmark falls back to PDR-only scale, every velocity reading inherits PDR's stride-frequency noise — the scooter use case never works. Step 4.2.1 (the affine-fit fix below) makes MiDaS actually contribute to the scale anchor at every keyframe, regardless of camera angle. Expected accuracy gain over today: bailout rate 100% → ~10%, MiDaS scale variance drops to the camera-noise floor instead of being zero (un-contributing).

DA3 was the **bigger** swing that would have eliminated the calibration loop entirely (no PDR dependency, no 5 s warmup), but on-device benchmarks killed it — see the deferral note at the top of this section.

### 4.2 Two-step upgrade

**4.2.1 Drop the floor-plane fusion and align in disparity space (low effort, prerequisite).**

NavSight runs MiDaS today but extracts zero useful information from it. `Tracker::applyDepthScaleConstraint` projects features onto a `(camera_height, ground_plane)` model and computes metric depth purely from geometry; MiDaS output values are only consulted as a `< 0.01` validity gate. With a forward-pointing scooter, chest-mounted, or even handheld-not-pitched-down phone, the floor heuristic fails: 32/32 bailouts in v21 sim, 31/31 in v22 (`midas_fused = 0` in both).

**Critical technical detail (correcting earlier plan version):** `DepthEstimator.kt:67-97` passes MiDaS's **raw network output** to native via `setDepthMap` with no post-processing. MiDaS's raw output is **disparity (relative inverse depth, scale-and-shift ambiguous)**, NOT depth — see MiDaS paper §4 on scale-and-shift-invariant losses. The variable named `rel_depth` in `Tracker.cpp:231` is actually disparity. Any algorithm that treats these values as "depth" gets the math wrong.

**Correct algorithm — VI-Depth Stage 1 (Intel ISL, ICRA 2023):**

For every KLT-tracked feature `i` with VIO triangulated depth `d_vio_i`:

1. Sample MiDaS disparity at the feature's pixel: `m_i = depth_map[u_i, v_i]`
2. Skip if `m_i < 0.01` (very far / unstable) or `d_vio_i` non-finite
3. Build pairs `(m_i, 1 / d_vio_i)` — both are inverse-depth quantities
4. Fit the affine relationship `inv_metric_depth_i ≈ s · m_i + t` by **least squares** (robust loss preferred):
   - `s` is the global scale factor between MiDaS disparity and metric inverse depth
   - `t` is the per-image shift (handles MiDaS's bias on a given scene)
5. The VIO→metric scale recovery comes from the joint fit: for any future VIO depth `d_vio` and its sampled MiDaS disparity `m`, the implied metric depth is `1 / (s · m + t)`. Compute the median or robust mean of `metric_depth_i / d_vio_i` across features to update `scale_fuser_`.
6. Outlier rejection: RANSAC over (s, t) or Huber loss in the LSQ solve. Require inlier ratio ≥ 50 % before accepting.

**Why affine (s, t), not single-parameter ratio.** MiDaS is trained with scale-and-shift-invariant losses by design (paper Eq. 1). Treating it as scale-only (the earlier "median depth ratio" formulation) ignores `t`, which is non-negligible in scenes with extreme depth range (sky + close foreground), causing a systematic bias.

**Why disparity space, not depth space.** Working in disparity avoids `1/m` for near-zero `m` (far points) which would blow up numerically. Outdoor scenes are dominated by far points where MiDaS disparity is near zero; converting to depth first destroys the numerical conditioning of the fit.

**Why this works for any cam angle.** No floor assumption — the algorithm just trusts MiDaS's per-pixel disparity at every triangulated feature, wherever it lands in the image. Forward-pointing scooter cam, downward handheld walk, looking-at-map angled view — all produce a valid `(m_i, 1/d_vio_i)` cloud for the fit.

**Implementation surface:**
- `Tracker.cpp::applyDepthScaleConstraint` — replace floor-plane block (lines ~197-248) with the affine fit. Keep the existing bounds gates (camera_h, intrinsics, finite values).
- `Tracker.cpp` add `static cv::Vec2d fitDisparityAffine(const std::vector<double>& m, const std::vector<double>& inv_z, double huber_delta)` helper.
- No JNI / Kotlin changes needed.
- `EventCounters.h` — add `midas_affine_fit_inlier_ratio_milli` and `midas_affine_fit_shift_milli` counters for offline tuning.

**References:**
- VI-Depth (Intel ISL, ICRA 2023): <https://github.com/isl-org/VI-Depth>
- MiDaS paper "Towards Robust Monocular Depth Estimation" (TPAMI 2022): <http://vladlen.info/papers/midas.pdf>
- Scale-and-shift-invariant loss formulation: §4 of the MiDaS paper.

**4.2.2 ~~Switch the model to Depth-Anything V3 small.~~** — DEFERRED 2026-05-16.

> **Status: DEFERRED.** Closed at 2026-05-10 after on-device benchmarking.
>
> **Benchmark result (S21 Ultra, Exynos 2100, Mali-G78):**
> - DA V2 Metric Outdoor Small ONNX INT8, 518×518:
>   - CPU: **722 ms** / inference
>   - NNAPI: **1907 ms** / inference
> - Budget: 100 ms / inference (1 Hz depth path target).
> - Verdict: V2 @ 518×518 not viable on Mali-G78. V3-Small is architecturally
>   larger than V2-Small → V3 timings would be ≥ V2.
>
> **Why this matters for the plan:** every benefit DA3 promised — metric
> output, no calibration loop, forward-cam works without floor heuristic — is
> recoverable from the existing MiDaS pipeline IF Step 4.2.1 (affine-fit in
> disparity space) is implemented. V3 was the *better-numbers* path, not the
> only-correct path.
>
> **Re-open conditions:** consider revisiting Step 4.2.2 only if (a) a DA3-Mobile
> distillation lands with measured ≤ 80 ms on Mali-G78, OR (b) NavSight targets
> a higher-end SoC (Snapdragon 8 Gen 3+, Apple A17+, Tensor G3+), OR (c) the
> 1 Hz depth-only path becomes a sub-1 Hz path (e.g. depth only at keyframe
> cadence ~0.2 Hz, relaxing the per-inference budget to 500 ms).
>
> **Source benchmark artifacts:**
> - ONNX model: `scripts/da3_benchmark/output/v2_metric_518_int8.onnx` (27 MB)
> - Conversion script: `scripts/da3_benchmark/` (V3→V2 pivot documented)
> - Bench APK: `bench-app/` (build green, NNAPI smoke test OOM-fixed)
> - Memory link: [[project_session_2026_05_10_v2_bench]]
>
> Step 4 of Phase 2 is therefore satisfied by completing Step 4.2.1 alone.

### 4.3 Comparison matrix (post-benchmark, 2026-05-16)

| Property | MiDaS v2.1 small (today + 4.2.1 fix = TARGET) | DA V2 small (DEFERRED) | DA V3 small (DEFERRED) |
|---|---|---|---|
| Output | disparity (post-4.2.1: affine-fit metric) | metric depth | metric + pose + geometry |
| Scale calibration loop | required | none | none |
| Forward-pointing cam (post-4.2.1) | **works** (no floor assumption) | works | works |
| Outdoor accuracy | 3–5 % (post-4.2.1) | 1–2 % | SOTA 2025 |
| **Inference time on S21 Ultra (Mali-G78)** | **~30 ms** ✅ | **722 ms CPU / 1907 ms NNAPI** ❌ | **≥ V2** ❌ |
| Budget | 100 ms | 100 ms | 100 ms |
| Model size | ~30 MB | ~25 MB | ~30 MB |
| License | MIT | Apache 2.0 | Apache 2.0 |
| Status | **ACTIVE — Step 4.2.1 implements** | **DEFERRED** (benchmark miss 7×) | **DEFERRED** (≥ V2 cost) |

### 4.4 Why MiDaS-with-affine-fit is the right destination

- **MiDaS v3.1**: still RELATIVE — same scale-calibration loop, marginal upgrade only. Skip.
- **ZoeDepth**: metric but indoor-trained primarily; weaker outdoors. Skip.
- **Marigold (diffusion)**: ~5 s/frame, unusable in real time. Skip.
- **DA V2 small (metric)**: benchmark-failed on the target SoC (722 ms CPU vs 100 ms budget). Skip.
- **DA V3 small**: same problem, magnified by V3 being larger than V2. Skip.
- **MiDaS v2.1 small + Step 4.2.1 affine fit**: stays within the 30 ms budget we already have, and the affine fit recovers the metric scale that DA3 would have given us — but using the depth model that already runs on-device. **This is the path.**

### 4.5 ~~Bonus opportunities V3 unlocks beyond the depth swap~~ — DEFERRED with 4.2.2

The three bolt-ons (per-pixel confidence map; self-calibrated intrinsics monitor; DA3 camera pose as an EKF measurement) all depended on V3's multi-output transformer. With V3 deferred (Section 4.2.2), they're deferred too.

**One survives in spirit at lower cost:** §4.5.2's "passive intrinsics drift monitor" can be implemented without DA3 — compare per-frame triangulation residuals against the configured intrinsics and flag drift. Logged here as a Phase 2 bolt-on (no DA3 dependency). The actual self-calibrated intrinsics from a model output are gone with V3.

### 4.6 Validation gates (Step 4.2.1 only)

Pull from a 200 m walk in 3 environments × 2 mount modes (handheld + scooter-mounted forward cam):

| Mount × env | Metric | Pass threshold |
|---|---|---|
| Handheld indoor | MiDaS affine-fit inlier ratio ≥ 50%, no bailout | true |
| Handheld outdoor sidewalk | same | true |
| Scooter forward urban | same | true (this is the primary scooter goal) |
| Any mount, stationary | scale-fuser output stable to within ±2% over 10 s | true |

If all four pass, Step 4.2.1 ships. Keep the floor-plane block in tree as a `/* LEGACY ... */` comment per project policy.

### 4.7 Files (Step 4.2.1 only)

- `app/src/main/cpp/Tracker.cpp::applyDepthScaleConstraint` — replace floor-plane block with affine-fit (see Section 4.2.1 above for the exact algorithm)
- `app/src/main/cpp/Tracker.cpp` — add `static cv::Vec2d fitDisparityAffine(...)` helper
- `app/src/main/cpp/EventCounters.h` — add `midas_affine_fit_inlier_ratio_milli`, `midas_affine_fit_shift_milli` counters
- No JNI / Kotlin / asset changes
- `app/src/main/cpp/Tracker.cpp` — `applyDepthScaleConstraint`: drop floor-plane fusion, switch to per-feature ratio aggregation. Add `is_metric` branch that skips the relative-to-metric conversion entirely.
- `app/src/main/cpp/native-lib.cpp` — `setDepthMap` JNI: optional `is_metric` flag.
- `tests/cpp/test_depth_estimator.cpp` (new) — unit tests for the scale-anchor pipeline, using a saved DA V3 inference output as a fixture.

### 4.8 Connection to Phase 1 Step 6 (persistent landmark map)

The persistent landmark map (Phase 1 Step 6) triangulates 3D points using the current depth pipeline. If we ship Phase 1 with MiDaS still in place, the landmark map's metric scale will inherit MiDaS's 3–5 % error and PDR-dependent calibration. **The cleanest order is:**

1. Finish Phase 1 with current MiDaS (drift-bound landmarks at MiDaS accuracy)
2. Phase 2 Step 4a (drop floor assumption) early — fixes immediate breakage for forward cams
3. Phase 2 Step 4b (DA V3 small model swap) — upgrades all landmark scale anchors AND opens the camera-pose-as-EKF-measurement path for Phase 3
4. Phase 2 Step 5 (map persistence across sessions) — this then locks in DA V3-grade accuracy across days

If scooter MountMode (Phase 2 Step 1) is needed before all of Step 4 completes, Step 4a (floor-assumption drop) is the minimum prerequisite.

---

## Step 5 — Map persistence across sessions

**Scope:** Save the persistent landmark map (Step 6 of Phase 1) to disk; load it on next launch. Enables "I've been here before" recognition across days.

### Storage

- Serialise `LandmarkMap` to a binary file under `<filesDir>/maps/session_<timestamp>.bin`
- Format: `[count] [Landmark0_id, p_world(3 doubles), descriptor(32 bytes), times_observed(int)] [Landmark1_...] ...`
- Optional metadata file: GPS lat/lon at save (for matching against next session location)

### Load

- On `startVIO`, scan `<filesDir>/maps/` for sessions within ~100 m of current GPS (if available)
- Pre-load matching session's landmarks into `LandmarkMap`
- DBoW2 keyframe index: also load the keyframe ORB descriptors from the matching session (for cross-session loop closure)

### UI

- Settings screen: "Saved maps" list with size/timestamp/location
- Manual delete option

**Files:**
- `LandmarkMap.cpp` — `saveTo(path)`, `loadFrom(path)` methods
- `Tracker.cpp` — load on init, save on session-end
- `LoopClosureDetector.cpp` — restore keyframe records from disk
- `NavSightViewModel.kt` + new `SavedMapsScreenUi.kt` — UI

**Validation:** Walk a route, close app, reopen later, walk same route — second-session trajectory aligns to first within 2 m at the start.

---

## Step 6 — OpenCV 4.5.3 → 4.9.x upgrade

**Scope:** Agent 2's audit flagged OpenCV 4.5.3 (June 2021). 4.9.x has NEON KLT improvements + bug fixes.

### Migration steps

1. Backup current `OpenCV-android-sdk/`
2. Download 4.9.0 SDK
3. Update `app/CMakeLists.txt` to point at new SDK
4. Compile, fix any API breaks (rare for KLT/PnP/triangulation paths — minor signature changes possible)
5. Run replay-harness regression tests against pre-upgrade fixtures
6. Performance benchmark (KLT frame time) — expect 10–30% improvement on ARM with NEON

### Risk

Low. OpenCV maintains strong backward compat. Most calls (`cv::calcOpticalFlowPyrLK`, `cv::findEssentialMat`, `cv::solvePnPRansac`, `cv::triangulatePoints`) have stable APIs.

**Validation:** All replay-harness tests pass post-upgrade with metrics within 5% of pre-upgrade baseline.

---

## Step 7 — Direction-flip loop closures (SuperPoint / LoFTR)

**Scope:** Agent 2 confirmed BoW-based direction-flip loop closures are FUNDAMENTALLY hard with ORB descriptors (~30° invariance, fails at 180°). Solution: rotation-invariant deep descriptors.

### 7.1 SuperPoint integration

- TFLite-converted SuperPoint model (~5 MB, runs on GPU delegate at 30 ms/frame)
- Replaces ORB extraction in `LoopClosureDetector`
- Train a custom DBoW2-style vocabulary on SuperPoint descriptors (project-specific scenes)
- Or use NetVLAD / LoFTR for descriptor-free matching

### 7.2 Hybrid approach

Run BOTH ORB (current) AND SuperPoint:
- ORB DBoW2: same-direction revisits, fast
- SuperPoint matcher: direction-flip revisits, slower but rotation-invariant
- Falls back to SuperPoint when ORB DBoW2 returns no candidate within heading gate

**Files:**
- New `app/src/main/cpp/SuperPointExtractor.cpp/h`
- TFLite model file in assets
- `LoopClosureDetector.cpp` — alternate query path
- Vocabulary: pre-train offline using a small set of session recordings

**Validation:** U-turn test — walk straight, U-turn, walk back. Trajectory closes within 1 m at the start (currently impossible).

---

## Step 8 — Performance optimization

**Scope:** Battery, frame-rate consistency, latency.

### 8.1 Frame budget

- Profile per-frame time in each component (KLT, MSCKF, SLAM, loop closure, depth)
- Target: ≤ 33 ms total at 30 fps (1 frame budget)
- Identify the slowest 10% of frames and optimize

### 8.2 Battery

- Measure current draw under sustained recording
- Identify high-power components (camera, GPU MiDaS, network if any)
- Reduce GPU MiDaS frequency further if `MountMode::Static`
- Throttle pose graph optimize() to once per minute even if multiple loop closures fire

### 8.3 Thermal

- Add a thermal monitor (`PowerManager.isPowerSaveMode()` + thermal API on Android 11+)
- When throttled: reduce frame rate, disable depth, simplify overlay

**Files:**
- `Tracker.cpp` — per-section perf counters (already present), publish to `EventCounters`
- New `BatteryThermalMonitor.kt`
- `SensorRepository.kt` — gate features by thermal status

---

## Step 9 — Testing infrastructure expansion

**Scope:** Make the project regression-resistant.

### 9.1 Replay fixtures

- Record 10+ canonical walks covering: pedestrian indoor, pedestrian outdoor, scooter, U-turn, multi-loop, low-light, motion-blur
- Each fixture includes JSON + frames + expected metrics (`expected_loop_closure_gap_m`, `expected_drift_per_meter`, etc.)
- CI runs replay-harness against all fixtures, fails if any metric regresses > 10%

### 9.2 Unit tests

- All new C++ classes (`PoseGraph`, `LandmarkMap`, `MountModeDetector`) get `tests/cpp/test_*.cpp` with synthetic inputs
- Use GoogleTest (already a project dep)
- Target: ≥ 80% coverage on new code

### 9.3 Integration tests

- Synthetic trajectory generator (clean ground truth + IMU/camera simulator)
- End-to-end test: feed synthetic data, verify final pose within tolerance
- Catches integration bugs that unit tests miss

**Files:**
- `tests/cpp/test_pose_graph.cpp` (new — already in Phase 1 Step 5)
- `tests/cpp/test_landmark_map.cpp` (new)
- `tests/cpp/test_mount_mode.cpp` (new)
- `tests/synthetic/` — trajectory generator
- `.github/workflows/ci.yml` — auto-run on PRs

---

## Phase 2 Status

| Step | Description | State |
|---|---|---|
| 1 | MountMode framework | 🟡 Queued |
| 2 | Velocity from landmark map | 🟡 Queued |
| 3 | Bad keyframe filter | 🟡 Queued |
| 4 | MiDaS final validation / alternative | 🟡 Queued |
| 5 | Map persistence across sessions | 🟡 Queued |
| 6 | OpenCV 4.5.3 → 4.9.x | 🟡 Queued |
| 7 | Direction-flip loop closures (SuperPoint/LoFTR) | 🟡 Queued |
| 8 | Performance optimization | 🟡 Queued |
| 9 | Testing infrastructure | 🟡 Queued |

## Order Recommendation

Steps are **mostly independent**, but suggested order by impact-per-effort:

1. **Step 1 + 2 first** (MountMode + landmark velocity) — unlocks scooter use case, the project's primary target
2. **Step 5** (map persistence) — high-impact UX, modest effort, leverages Phase 1's landmark map
3. **Step 3 + 4** (bad keyframe filter + MiDaS validation) — quality improvements, small
4. **Step 9** (testing) — anytime — should be ongoing
5. **Step 6** (OpenCV upgrade) — schedule for a quiet sprint, low risk
6. **Step 8** (performance) — when battery/heat complaints surface
7. **Step 7** (SuperPoint) — last; expensive in dev time but high differentiator

## Build / Validation Cadence

Same as Phase 1: build green, install, brief test walk per step, validate, move on. Don't pile up.

---

## Phase 2 Exit Criteria

- ✅ NavSight runs reliably on scooter (5–25 km/h) with < 5%/km drift
- ✅ Cross-session map recognition works (same route, different days, < 2 m alignment)
- ✅ Battery: ≥ 1 hour continuous recording on a typical phone
- ✅ Frame rate: 30 fps sustained, ≤ 5% frames drop
- ✅ Test coverage ≥ 80% on new code
- ✅ Direction-flip loop closures work (SuperPoint or chosen alternative)

When Phase 2 exits: NavSight is **production-ready**.

---

## Reference Documents

- `docs/study/post_v19_sprint_plan.md` — Phase 1 (prerequisite)
- `docs/study/architecture_comparison.md` — VIO architecture comparison (Phase 1 background)
- `docs/study/dependency_audit.md` — DBoW2 + library audit (Phase 1 background)
- `docs/ARCHITECTURE.md` — should be updated at start of Phase 2 to reflect current state
- `docs/KNOWN_ISSUES.md` — should have Phase 1 issues closed before Phase 2 starts

---

## Appendix B — Depth-Anything V3 Mobile Benchmark Plan

**Purpose:** measure whether DA3-Small can replace MiDaS on the actual target phone before we commit Step 4 architecture changes to V3. This benchmark is the **gate** for Phase 2 Step 4 and all of Appendix C (Phase 3 V3 work).

### B.0 The three questions to answer

1. **Latency:** does DA3-Small INT8 fit a < 100 ms per-frame budget on the user's phone at a usable input resolution?
2. **Accuracy:** does INT8 quantization stay within 5% of FP32 depth on real walks?
3. **Stability:** does on-device inference run without thermal throttling or > 15% extra battery drain over 10 minutes?

If all three pass → proceed with DA3METRIC-Small as the Step 4 target. If any fails → fall back to DA V2 small (metric).

### B.1 Step B1 — Conversion pipeline (offline, ~1 day on PC)

1. Pull the DA3-Small PyTorch checkpoint from `depth-anything/DA3-SMALL` on HuggingFace (0.08B params)
2. Convert PyTorch → ONNX via the V3 repo's export tooling
3. Convert ONNX → TFLite via `onnx2tf` (more reliable than `onnx-tf` in 2025/2026)
4. Produce three quantized variants per resolution:
   - **FP32** (accuracy baseline)
   - **FP16** (default mobile-GPU target)
   - **INT8** dynamic-range quant (NNAPI / Hexagon DSP path)
5. Sanity check: file sizes recorded; first inference returns a depth map of correct shape on a known fixture image

**Files (new, all under `scripts/da3_benchmark/`):**
- `scripts/da3_benchmark/convert_da3_small.py` — PyTorch → ONNX → TFLite conversion
- `scripts/da3_benchmark/sanity_inference.py` — FP32 / FP16 / INT8 outputs vs reference
- `app/src/main/assets/da3_small_fp32.tflite`, `da3_small_fp16.tflite`, `da3_small_int8.tflite` (build artefacts; gitignored once we settle)

**Risk:** custom op / unsupported layer in PyTorch DA3-Small that doesn't survive ONNX → TFLite. The AXERA fork ships INT8 configs (`config_u8.json`, `config_u8_base.json`) — proves quantization works in principle on edge silicon, but the conversion path may need fixes.

### B.2 Step B2 — Standalone microbenchmark via adb (~1 hour, no app changes)

Use TensorFlow's `benchmark_model` binary directly on the device. No NavSight app changes needed.

```bash
adb push da3_small_int8.tflite /data/local/tmp/
adb push benchmark_model /data/local/tmp/
adb shell chmod +x /data/local/tmp/benchmark_model

# CPU baseline
adb shell /data/local/tmp/benchmark_model \
  --graph=/data/local/tmp/da3_small_int8.tflite \
  --num_runs=50 --warmup_runs=5 \
  --enable_op_profiling=true \
  --profiling_output_csv_file=/data/local/tmp/profile_cpu.csv

# GPU delegate
adb shell /data/local/tmp/benchmark_model \
  --graph=/data/local/tmp/da3_small_int8.tflite \
  --num_runs=50 --warmup_runs=5 \
  --use_gpu=true

# NNAPI / Hexagon DSP
adb shell /data/local/tmp/benchmark_model \
  --graph=/data/local/tmp/da3_small_int8.tflite \
  --num_runs=50 --warmup_runs=5 \
  --use_nnapi=true
```

**Metrics:**
- Mean / p50 / p95 inference latency at three input resolutions: **504, 384, 256**
- Per-op latency CSV (find the bottleneck ops — usually attention)
- Peak resident memory

**Decision gate B2:**
| p95 inference time | Verdict |
|---|---|
| < 100 ms at ≥ 384 res | Green — proceed to B3 |
| 100–200 ms at 384 | Yellow — V3 only at ≤ 1 Hz scale-anchor rate; not viable as every-frame source |
| > 200 ms at 384 | Red — fall back to DA V2 small (metric); revisit V3 when distilled mobile variant ships |

### B.3 Step B3 — Accuracy regression vs FP32 (~2 hours, offline)

Reuse the existing **sim recording infrastructure** (sims under `tests/sims/`) — saved camera frames + ground truth means we don't need a fresh capture session.

1. Pick 3 representative sims:
   - One indoor walk (e.g., recent v22 sim)
   - One outdoor sidewalk walk (e.g., 1 km field test once captured)
   - One scooter-equivalent (use any sim with sustained 1+ m/s forward motion)
2. Feed each sim's saved frames through FP32 / FP16 / INT8 DA3-Small offline (Python, on PC)
3. Per frame, compute median absolute relative depth error: `|d_INT8 - d_FP32| / d_FP32`
4. Aggregate over the sim: report **median, p95** of per-frame median errors

**Pass criterion:** median rel error < 5% on all three sims; p95 < 10%.

**File:** `scripts/da3_benchmark/regression_vs_fp32.py`

### B.4 Step B4 — Live integration A/B test (~1 day on device)

In `DepthEstimator.kt`, add a debug switch:

```kotlin
private val depthModel = when (BuildConfig.DEPTH_MODEL) {
    "v3" -> DepthAnythingV3Small()
    else -> MidasV21Small()
}
```

Build two variants of the app, each with a different `DEPTH_MODEL` flag. Walk a known route with each (same time of day, same battery start %).

**Compare:**
- VIO trajectory drift end-of-walk (m)
- Loop-closure metrics from `event_summary.json`: `loop_closure_corrections_applied`, `loop_closure_low_score_rejects`, `var_p` distribution
- Power drain: battery % delta over 10 min recording
- Thermal events: `adb shell dumpsys thermalservice` snapshot pre/post

**Pass criterion:** drift no worse than MiDaS baseline; battery drain ≤ 15% over 10 min; no thermal throttling.

### B.5 Step B5 — Confidence-percentile sensitivity (~30 min, offline)

V3's `conf_thresh_percentile` defaults to 40. We want to pick the value that minimizes scale-anchor variance for our specific use case.

Ablate over {20, 40, 60, 80} using a stationary frame from a sim. Compute `Var(depth_ratio)` across N detected features for each setting. Pick the percentile where variance plateaus (over-strict thresholds give too few features for stable mean).

**File:** `scripts/da3_benchmark/conf_percentile_sweep.py`

### B.6 Instrumentation we already have (no new infra needed)

- `DepthEstimator.kt`: TFLite Interpreter machinery already in place from MiDaS — drop-in pattern
- `EventCounters.h`: extend with `depth_inference_ms_mean_milli`, `depth_inference_ms_p95_milli`, `depth_conf_median_milli` — same Welford pattern as `cam_fps_*` already shipped in v22
- `tests/sims/`: saved sim frames are perfect for B3 regression; no new capture needed
- `scripts/`: where the conversion + analysis scripts live (per memory rule "reuse scripts, don't rewrite inline Python")

### B.7 Decision matrix (final)

| B2 result | B3 result | B4 result | Action |
|---|---|---|---|
| Green | Green | Green | Proceed Phase 2 Step 4 with DA3METRIC-Small INT8 |
| Yellow | Green | Green | Use V3 at 1 Hz as scale anchor only; keep MiDaS for high-rate path until distilled DA3 ships |
| Red | — | — | Fall back to DA V2 small (metric); revisit V3 in Q3 |
| — | Fail | — | Investigate quantization config; if irreducible, fall back to FP16 (slower) or DA V2 |
| — | — | Fail | Reduce inference rate; if still bad, fall back |

---

## Appendix C — Phase 3 V3 Architectural Candidates

These are NOT Phase 2 work. They depend on Appendix B benchmark passing and at least one full Phase 2 Step 4 (V3 metric depth) shipping in production. Listed here because (a) the Phase 2 design needs to know about them so we don't paint ourselves into a corner, and (b) they're the items that make V3 a transformative dependency rather than just a depth-model swap.

### C.1 V3 as loop-closure geometry source

**Replaces:** ORB descriptor → PnP 3D-2D → chi² gate path in `LoopClosureDetector.cpp`.

**Why:** the chi² gate has been the binding constraint on loop closure success for over a week. Per `project_session_2026_05_05.md`, accepted m² runs ~5.8M against a threshold of 22.5 — meaning EKF covariance is too tight relative to actual drift, and PnP measurement noise is too small relative to true error. Adjusting either constant masks the underlying problem (which `feedback_no_disabling.md` rules out anyway).

**Mechanism:** when DBoW2 returns a candidate keyframe, instead of running ORB descriptor matching + PnP, do a **2-image V3 inference**:

```python
prediction = model.inference(
    image=[current_frame, candidate_keyframe],
    extrinsics=[ekf_pose_now, saved_pose_then],   # priors
    intrinsics=[K, K],
    align_to_input_ext_scale=True,                # output in our metric scale
    use_ray_pose=False,                           # cam decoder for relative pose
)
# prediction.extrinsics[0] - prediction.extrinsics[1] = relative pose with V3's confidence
```

The output is a learned alternative to PnP that:
- Handles 180° revisits (ORB descriptors aren't direction-invariant; DINOv2 features are) — kills the π/2 heading gate workaround at `LoopClosureDetector.cpp:464-475`
- Returns a continuous confidence per-pixel that we can integrate into the loop-closure measurement covariance (replacing the static `LOOP_CLOSURE_PNP_SIGMA_FLOOR_M=2.0`)
- Bypasses the chi² rejection problem entirely — V3 returns the relative pose; we trust it weighted by V3's confidence

**Effort:** medium. Mostly plumbing if Appendix B passes — 1-2 weeks.
**Impact:** very high. Solves the loop-closure block that's been open for weeks.

### C.2 V3 Gaussians as the persistent landmark map representation

**Replaces:** the ORB-feature-triangulation implementation of Phase 1 Step 6's persistent landmark map.

**Why:** Phase 1 Step 6 stores 3D landmarks as `(xyz, descriptor, observation_count)`. V3's `infer_gs=True` mode outputs **3D Gaussians** — `(xyz, opacity, covariance)` — directly. Each Gaussian carries its own uncertainty ellipsoid, eliminating the ad-hoc covariance-from-observations heuristic.

**Mechanism:**
- Phase 1 Step 6 still ships first as the integration scaffolding (loading, persistence, query-by-position, EKF correction injection). Storage primitive = `xyz + descriptor + obs_count`, source = ORB triangulation.
- Phase 3 swaps the source: `xyz + opacity + covariance`, source = V3 Gaussians from per-keyframe inference.
- Query interface unchanged. Drop-in.

**Effort:** medium — depends on V3 inference rate and Gaussian density. If V3 returns 1k Gaussians per keyframe and we keep 100 keyframes, that's 100k Gaussians = ~10 MB at xyz+rgb+cov. Manageable.
**Impact:** high — better landmark quality, automatic uncertainty, no ad-hoc heuristics.

### C.3 V3 camera pose as a continuous EKF measurement

**Adds:** an MSCKF-style 6-DOF pose measurement to the EKF, sourced from V3, every N frames.

**Why:** drift between loop closures is the residual problem. Even with C.1, loop closures only fire when DBoW2 finds a revisit. Long out-and-back trajectories with no revisits accumulate drift unbounded. V3 pose-as-measurement provides continuous absolute-pose anchoring without requiring revisits.

**Mechanism:** every N frames (probably N=30 = 1 Hz at 30 fps),
- Run V3 single-image inference on the current frame
- Take `extrinsics` output as a 6-DOF measurement
- Innovation = V3 pose − EKF predicted pose (in the appropriate frame)
- Measurement covariance = derived from V3 confidence map (mean confidence → sigma)
- Standard EKF update on the 19-DOF state

**Risk:** V3 pose has its own bias (single-image pose is fundamentally underdetermined up to scale). The metric variant + `align_to_input_ext_scale=True` mitigates but doesn't eliminate. Needs careful covariance tuning.

**Effort:** high — touches the EKF measurement model. 2-3 weeks of careful work.
**Impact:** very high — bounds drift continuously, not just at revisits.

### C.4 Interaction order

```
Phase 2 Step 4   →  Appendix B benchmark   →  ship DA3METRIC-Small   →  C.1   →  C.2   →  C.3
                                                  (quick win)         (loop)   (map)    (drift)
```

Don't start C.1–C.3 until V3 has been stable in production for ≥ 30 days as the Step 4 depth source. We need ground truth on V3's real-world behavior on this phone before adding it to the EKF measurement chain.

### C.5 What this means for Phase 1 Step 6

Phase 1 Step 6 (persistent landmark map) ships with **ORB-triangulation as the storage primitive**. That decision stands — it's the integration scaffolding. C.2 only swaps the *source* of landmarks. So no Phase 1 changes are needed in light of V3.
