# 03 — Loop Closure + Mapping + Windowed BA + Pose Graph

## 0. Build wiring — what is actually shipped

`app/CMakeLists.txt:55-77` defines the `navsight` shared library:

- `LoopClosureDetector.cpp` — **ACTIVE** (`app/CMakeLists.txt:64`).
- `WindowedBA.cpp` — **ACTIVE** (`app/CMakeLists.txt:76`).
- `Mapper.cpp` — **DISABLED** (`app/CMakeLists.txt:62`, comment: _"Mapper pipeline — output was discarded (applyMapperResult was a no-op)"_). ADR-006 ratifies this.
- `PoseGraph.cpp` — **DISABLED** (`app/CMakeLists.txt:63`).

**The shipped loop-closure path is DBoW2 → PnP → `EKFState::updateAbsolutePose`.** It bypasses `Mapper`, `PoseGraph`, and any blend-correction logic.

`Mapper.cpp:31` calls `loop_closure_detector_.reset()` — this method **does not exist** on the current `LoopClosureDetector.h`. `Mapper.cpp:591-594` calls a `detectLoopClosure(descriptors, keypoints, pos, heading_float, matched_kf)` overload and uses a `LoopClosureKeyframe` struct — **neither exists** on the current header. So the dead code cannot be reactivated by flipping the cmake comments alone.

**Compiler flag note** (`app/CMakeLists.txt:5-18`): `-O3 -ffast-math -fno-finite-math-only -ftree-vectorize`. The `-fno-finite-math-only` is load-bearing — `LoopClosureDetector.cpp:499-500` relies on `std::isfinite(...)` to filter NaN-marked rows. Without this flag isfinite always returns `true` and PnP silently receives `(0,0,0)` 3D points (debugged 2026-05-05).

---

## 1. Loop-closure detection pipeline (the active path)

### 1.1 Threading

- **Camera thread** publishes new keyframes via `addKeyframe(...)` (`Tracker.cpp:2603-2611`).
- **Loop-closure worker thread** wakes at most every `LOOP_CLOSURE_QUERY_PERIOD_S = 1.0` s (`Tracker.h:505`, `Tracker.cpp:3416`) and calls `tryDetectLoop(...)` (`Tracker.cpp:3449-3456`).
- Internal mutex `Impl::mutex` (`LoopClosureDetector.cpp:159`) guards `db`, `keyframes`, `entry_to_index`.
- Slow PnP runs **outside** the mutex (`LoopClosureDetector.cpp:449-451`).

### 1.2 Stage-by-stage pipeline (inside `tryDetectLoop`, `LoopClosureDetector.cpp:316-600`)

1. **Input validation** (`L325-334`): detector ready, descriptors `CV_8U Nx32`, rows == keypoints.size().
2. **Temporal exclusion of recent keyframes** (`L362-380`): `cutoff_ns = now_ns − temporal_exclusion_ns`. Walk keyframes deque from back; highest `db_entry_id` whose `timestamp_ns ≤ cutoff_ns` becomes `max_id`. Tracker passes `LOOP_CLOSURE_TEMPORAL_EXCL_NS = 30 × 10⁹` ns (30 s) (`Tracker.h:506`).
3. **BoW query** (`L385-394`): `query_bow = vocab.transform(features)`; `db_.query(features, results, kBowTopN, max_id)`. `kBowTopN = 4` (`L108`). `results[0]` is best.
4. **Adaptive minScore gate (ORB-SLAM2-style)** (`L396-441`): score query against the K most-recent covisibility-proxy neighbors NEWER than `cutoff_ns`. `K = kCovisibilityK = 10` (`L79`). Take `min_score = min(scores)`, floor at `kBowScoreFloor = 0.005` (`L71`):
   ```
   threshold = max(min_over_neighbors(score(query, kf_i)), 0.005)
   ```
   If `n_neighbors == 0` (early walk), fall back to floor 0.005. Reject and bump `loop_closure_rejects_low_score` if `best_score < threshold`. The previous fixed 0.05 was a misreading of Galvez-Lopez & Tardos 2012 (which thresholded *normalised* score η, not raw L1); real-walk evidence: `tests/sims/simulation_data_1777985054704.json` (114 s twice-around-house, 150 attempts, 0 accepts at 0.05).
5. **Candidate snapshot under lock** (`L443-451`): `O(1)` lookup via `entry_to_index`, copy `KeyframeRecord` out, release mutex.
6. **Heading gate** (`L464-475`):
   ```
   hdiff = |current_yaw_rad - candidate.yaw_rad|
   if (hdiff > π) hdiff = 2π - hdiff
   if (hdiff > π/2) reject (bump loop_closure_rejects_heading)
   ```
   `kMaxHeadingDiffRad = M_PI / 2.0` ≈ 1.5708 rad ≈ 90° (`L465`). Source: `sim_data_1778078217065` — 587/638 PnP failures from a 600 m walk where the user returned via the opposite direction.
7. **BFMatcher Hamming + Lowe ratio** (`L478-504`): `cv::BFMatcher(NORM_HAMMING, crossCheck=false)` (`L181`). `knnMatch(query_desc, candidate.descriptors, knn, k=2)`. Lowe test: `pair[0].distance < 0.75 × pair[1].distance`. `kLoweRatio = 0.75f` (`L95`). For each surviving match, look up `candidate.pts3d_world[trainIdx]`. Skip rows where 3D is non-finite (NaN-marked); skip out-of-bounds idx. Collect `pts2d`, `pts3d`.
8. **Pre-PnP minimum-pair gate** (`L506-514`): require `pts2d.size() >= kPnpMinInliers = 15` (`L91`); else bump `loop_closure_rejects_pnp`. Comment `L82-91`: original plan §line 723 specified 30, structurally unreachable here (`MAX_SLAM_FEATURES = 12`). ORB-SLAM2's `LoopClosing.cc::ComputeSim3` accepts at 12-20.
9. **`cv::solvePnPRansac`** (`L516-540`):
   ```
   K = [[fx,0,cx],[0,fy,cy],[0,0,1]]
   distortion = zeros(4,1)               // already undistorted upstream
   useExtrinsicGuess = false
   iterationsCount   = 100               // kPnpMaxIters       (L103)
   reprojectionError = 4.0 px            // kPnpReprojThreshPx (L101)
   confidence        = 0.99              // kPnpConfidence     (L102)
   flags             = cv::SOLVEPNP_ITERATIVE
   ```
   `try/catch (cv::Exception)`; on throw, bump `loop_closure_rejects_pnp` and return false. Looser than reloc's 1.5 px (`L98-100`): "long temporal gap means query and match keyframes captured under noticeably different lighting / viewpoint."
10. **Post-PnP inlier gate** (`L542-550`): require `pnp_ok && inliers.size() >= 15`; else bump `loop_closure_rejects_pnp`.
11. **Build `LoopMatch` payload** (`L552-599`):
    ```
    R_now_world    = Rodrigues(rvec)
    t_now_world    = tvec
    R_match_world  = candidate.R_world_cam.t()
    t_match_world  = -(R_match_world * candidate.t_cam_world)
    R_now_to_match = R_match_world * R_now_world.t()
    t_now_to_match = t_match_world - R_now_to_match * t_now_world
    ```

`LoopMatch` fields and frame conventions (`LoopClosureDetector.h:104-141`):
```
X_match = R_now_to_match * X_now + t_now_to_match
X_world = R_world_cam_match * X_match + t_cam_world_match
```
`t_now_to_match` is OpenCV-tvec-style (NOT a position difference). `t_cam_world_match` is a world-frame point (camera position in world).

### 1.3 Counter ownership

Per comment block at `Tracker.cpp:3458-3466` (cpp-reviewer 2026-05-04 HIGH-1 fix consolidates ownership):
- Tracker owns: `loop_closure_attempts` (worker tick), `loop_closure_accepts` (worker accept), `loop_closure_kf_count_in_db` (after addKeyframe), `loop_closure_corrections_applied` (after EKF accept).
- Detector owns exclusively: `loop_closure_rejects_low_score`, `loop_closure_rejects_pnp`, `loop_closure_rejects_heading`.
- EKFState owns: `loop_closure_chi2_rejected`.

---

## 2. Vocabulary / BoW

### 2.1 Loading (`LoopClosureDetector::loadVocabulary`, `L192-242`)

- Wipes `keyframes`, `entry_to_index`, `db`, `ready`.
- `vocab.loadFromTextFile(vocab_path)` — ORB-SLAM2 plain-text format (`assets/ORBvoc.txt.gz` / `.tar.gz`). Kotlin extracts to FS path; `cv::FileStorage` cannot read AssetManager URIs (header `L67-72`).
- Creates database with **direct index disabled**:
  ```
  db_ = std::make_unique<OrbDatabase>(vocab, use_di=false, di_levels=0);
  ```
  (`L232-236`) — explicit BFMatch makes the direct index unnecessary, saves memory + add() cost.
- Tracker entry: `Tracker::loadLoopClosureVocabulary` (`Tracker.cpp:3339-3360`); on success spawns worker via `std::thread(&Tracker::loopClosureWorkerLoop, this)`.

### 2.2 Scoring

DBoW2 default L1 score on BoW vectors. `KeyframeRecord::bow_vec` (`L150`) caches the `DBoW2::BowVector` because `OrbDatabase::add` doesn't return it; storing costs ~50 µs/keyframe at 250 features (`L300-302`) and avoids re-`transform`ing every neighbor on every query for adaptive minScore.

### 2.3 Similarity threshold

**No fixed threshold.** Per-query adaptive (see §1.2 step 4):
```
threshold = max(min_{i ∈ K_recent}(vocab.score(query, kf_i.bow_vec)), 0.005)
```
- `kBowScoreFloor = 0.005` (`L71`).
- `kCovisibilityK = 10` (`L79`). At ~2 Hz keyframe rate covers ~5 s, comfortably inside 30 s temporal exclusion.

---

## 3. PnP / geometric verification — exact parameters (table)

| Parameter                  | Value                  | Constant                  | File:Line |
|----------------------------|------------------------|---------------------------|-----------|
| Algorithm                  | `cv::SOLVEPNP_ITERATIVE` | hard-coded              | `L534`    |
| RANSAC iterations          | 100                    | `kPnpMaxIters`            | `L103`    |
| Reprojection threshold     | 4.0 px                 | `kPnpReprojThreshPx`      | `L101`    |
| Confidence                 | 0.99                   | `kPnpConfidence`          | `L102`    |
| Min inliers (pre & post)   | 15                     | `kPnpMinInliers`          | `L91`     |
| Lowe ratio                 | 0.75                   | `kLoweRatio`              | `L95`     |
| BoW top-N retrieved        | 4                      | `kBowTopN`                | `L108`    |
| Distortion model           | zeros(4,1)             | (rectified upstream)      | `L521`    |
| `useExtrinsicGuess`        | false                  | hard-coded                | `L529`    |

---

## 4. χ² gating — the actual gate (in `EKFState::updateAbsolutePose`, `EKFState.cpp:925-1060`)

### 4.1 Residual (6×1) — `L947-971`
```
R_err = target_R_world_imu * R_GtoI_.t()
r_R   = Rodrigues(R_err)               // 3×1, axis-angle, wrapped to [-π,π]
r_p   = target_p_world - p_G_          // 3×1, world-frame position
r     = [r_R; r_p]                     // 6×1
```

### 4.2 Jacobian (6×dim) — `L980-984`
IMU error-state layout: `[δθ(0..2), δb_g(3..5), δv(6..8), δb_a(9..11), δp(12..14)]`. Sparse:
```
H[i,    i  ] = 1.0    for i in 0..2    // ∂r_R / ∂δθ
H[3+i, 12+i] = 1.0    for i in 0..2    // ∂r_p / ∂δp
```
ALL clone DOFs get zero Jacobian (absolute-pose, not relative).

### 4.3 Measurement noise (6×6, diagonal) — `L986-993`
```
sR = max(sigma_axis_sq_R, 1e-8)        // rad²
sp = max(var_p,           1e-6)        // m²
R_noise = diag(sR, sR, sR, sp, sp, sp)
```
Computed by Tracker (`Tracker.cpp:3601-3608`):
```
sigma_axis_sq_R = LOOP_CLOSURE_BASE_ROT_SIGMA_RAD² × damping_inv
                = 0.34907² × damping_inv        // 20° base, rad²

sigma_p = max(LOOP_CLOSURE_PNP_SIGMA_FLOOR_M, LOOP_CLOSURE_DRIFT_RATE × total_path_m_)
        = max(2.0, 0.032 × total_path_m_)      // metres
var_p   = sigma_p² × damping_inv
```

| Constant                                | Value             | Reference        |
|-----------------------------------------|-------------------|------------------|
| `LOOP_CLOSURE_BASE_ROT_SIGMA_RAD`       | 0.34907 rad (20°) | `Tracker.h:538`  |
| `LOOP_CLOSURE_PNP_SIGMA_FLOOR_M`        | 2.0 m             | `Tracker.h:528`  |
| `LOOP_CLOSURE_DRIFT_RATE`               | 0.032 m / m walked | `Tracker.h:529` |
| `damping_inv`                           | 1.0…100.0         | `Tracker.cpp:3531-3536` |

### 4.4 The gate — `L995-1056`
```
S      = H × P_ × Hᵀ + R_noise         // 6×6
S_inv  = invert(S, Cholesky → SVD)     // SVD fallback
m²     = rᵀ × S_inv × r

if m² > kChi2Threshold:                // kChi2Threshold = 22.5
    eventCounters().loop_closure_chi2_rejected.fetch_add(1)
    return false

applyMSCKFUpdate(H, r, R_noise)
return true
```
**`kChi2Threshold = 22.5` at `EKFState.cpp:1000`** — χ²(0.999, 6 DOF) ≈ 22.458. Comment `L920-924`: "Loop closures are tolerant by design: damping fades a wrong match across 10 frames; a tight χ² would defeat that."

### 4.5 Per-block diagnostics (no gate effect) — `L1009-1036`
```
S_R   = S[0:3, 0:3]
S_p   = S[3:6, 3:6]
m²_R  = r_Rᵀ × S_R⁻¹ × r_R         // χ²(0.999,3) ≈ 16.27 budget
m²_p  = r_pᵀ × S_p⁻¹ × r_p         // χ²(0.999,3) ≈ 16.27 budget
```
Logged BEFORE the gate (`L1042-1050`) so chi²-rejects are debuggable. They don't sum to `m²` (S has cross terms) but isolate which block dominates.

### 4.6 Important corrections to the project-context numbers

**The constant `LOOP_CLOSURE_BASE_TRANS_SIGMA_M = 6.0` does NOT exist in the current `Tracker.h`.** Commit `d64a4ff` ("step-7: heading gate + dynamic sigma + path-length tracking") replaced it with the dynamic formula. The "derived not magic" framing now applies to:

- `LOOP_CLOSURE_DRIFT_RATE = 0.032`, derived (`Tracker.h:524-527`):
  ```
  0.15 (15 %/100 m drift) ÷ √(22.5 − 0.84) = 0.15 ÷ 4.65 = 0.032
  ```
- `LOOP_CLOSURE_BASE_ROT_SIGMA_RAD = 0.34907` (20°), derived (`Tracker.h:530-537`): with `m²_rot = |r_R|²/σ²_R`, the previous 3° σ allowed only `|r_R| < 14.2°`, but typical VIO heading drift over a 50–100 m loop is 5–20°. Source: `sim_data_1778100250961` — 40 PnP accepts, 392 χ² rejects, 0 corrections with 3° sigma.

**The "m²~5.8M observed vs 22.5 threshold" symptom pre-dates `d64a4ff`.** The `Tracker.h:512-516` comment encodes the analysis: "tight MSCKF updates collapse `P_[pp]` to ~0, so `S ≈ R_noise` and `m² ≈ |r_p|² / var_p`. The χ²(0.999,6) = 22.5 gate then requires sigma ≥ actual_drift." The dynamic-sigma fix is the response — but **whether it actually unblocks corrections has not been validated against a real walk** (no recorded sim with sigma=6.0 either, per the 2026-05-06 session memory note).

---

## 5. Mapper — DEAD CODE, described for archaeological completeness

Per §0, `Mapper.cpp` and `Mapper.h` are not compiled. ADR-006 is authoritative.

### 5.1 State (`Mapper.h:60-112`)
- **Depth-map mirror** (`L61-63`): `depth_mutex_`, `latest_depth_map_` (MiDaS relative depth, `std::vector<float>`), `depth_width_`, `depth_height_`.
- **Ground-plane** (`L66-74`): `GroundPlaneEstimate{vp, camera_pitch, ground_distance, confidence, is_valid}`, `user_camera_height_ = 1.4` m.
- **BA state** (`L77-105`): `Keyframe{frame_id, pose 4×4, keypoints, points_3d, timestamp_ns}`; `KeyframeWindow` is `std::deque<Keyframe>` capped at **8** entries (`L89: if (window_.size() > 8) window_.pop_front()`); `bundle_adj_scale_=1.0`, `bundle_adj_iteration_=0`, `last_ba_timestamp_ns_=0`. Constants: `BA_INTERVAL = 5`, `BA_MAX_ITER = 10`, `HUBER_THRESHOLD = 2.0` px.
- **Loop closure / pose graph** (`L108-112`): `loop_closure_detector_` (uses obsolete API), `pose_graph_`, `frames_since_last_keyframe_=0`, `last_keyframe_position_=(0,0,0)`, `last_pose_graph_node_id_=-1`.

### 5.2 Public API
- `Mapper()` (`L21-24`).
- `void reset()` (`L26-38`) — calls `loop_closure_detector_.reset()` (does not exist in current detector).
- `MapperResult process(const TrackerFrame&, double current_smooth_scale, const EKFState* ekf=nullptr)` (`L42-67`): per-frame tick = (1) `detectGroundPlane`, (2) `constrainScaleWithDepth`, (3) `pose_graph_.addNode(x, z, heading, ts)` (every frame becomes a node), (4) `runBundleAdjustment` if `frame.pose_valid`, (5) `detectLoopClosure`.
- `void setDepthMap(const float*, int, int)` (`L69-80`).
- `void setCameraHeight(double height_m)` (`Mapper.h:49`, inline).

### 5.3 Relationship to EKF
Read-only `const EKFState*`. **Mapper does NOT correct EKF state.** Outputs (`MapperResult`) were intended to flow back through `VioEngine::applyMapperResult`, which `Tracker.cpp:528` confirms is "a no-op, never calls these". ADR-006 ratifies the disabled state.

### 5.4 Per-method math (private)
- **`detectGroundPlane`** (`Mapper.cpp:201-340`): two-stage RANSAC + vanishing-point.
  - RANSAC (`L227-249`): 40 iters, 3-point plane fit, inlier dist threshold **0.04 m**, accept if inliers ≥ 10 AND ≥ 40 % of candidates. Gravity check via EKF (`L256-263`): rejects if plane normal differs from gravity in cam frame by more than 20° (`std::cos(20° × π/180)`). Height gate `0.4 < h < 4.0` m.
  - Horizon (`L274-318`): Canny + HoughLinesP, vote intersection-y, `pitch = |atan2(vp_y - rows/2, focal)|`.
  - Synthesis: prefer RANSAC; horizon is fallback. Confidence `0.7 × inlier_ratio` for RANSAC, `0.4 × min(1, n_vp/10)` for horizon.
- **`constrainScaleWithDepth`** (`Mapper.cpp:82-193`): two-part. (1) Calibrate MiDaS depth via ground-plane homography `Z_metric = h / (img_y × cos(pitch) + sin(pitch))` (`L129-133`). (2) Median ratio between calibrated MiDaS depth and VIO-triangulated depth at tracked features. Returns `optimized_scale = ratio × frame.estimated_scale`, `alpha = 0.12` if calibrated else `0.04`.
- **`runBundleAdjustment`** (`Mapper.cpp:350-569`): 1-DOF Gauss-Newton on global scale with Huber. Two modes:
  - Mode A (MSCKF clone-based): uses `ekf->getWindow().back()` and `[size-2]` clones. `BA_MAX_ITER = 10`, LM damping `λ = 0.01 × H`, step clamp `[-0.1, 0.1]`, scale clamp `[0.005, 20.0]`. Temporal-smoothness rate gate (`max_rate = 0.05` if `quality > 0.7` else `0.08`). Returns `alpha = 0.08`.
  - Mode B (legacy keyframe window): same GN, weight `w = 0.3 + 0.7 × (kf_idx / max(1, n_kfs-1))`, observation cap **200 pairs**.
- **`detectLoopClosure`** (`Mapper.cpp:575-651`): generates ORB(500, 1.2, 8). Calls obsolete `loop_closure_detector_.detectLoopClosure(...)`. On hit: `pose_graph_.addLoopEdge(last_node, last_node, dx, dz, dh, info_pos=10.0, info_heading=5.0)`, `pose_graph_.optimize(15)`, `pose_graph_.getLatestCorrection`, blend weight `0.50`. Stores new keyframe every **0.3 m** of motion or **10** frames (`L632-648`).

---

## 6. PoseGraph — DEAD CODE, described for completeness

`PoseGraph.h`, `PoseGraph.cpp`. SE(2) (`x, z, heading`) Gauss-Newton solver. Used only by dead `Mapper`.

### 6.1 Data layout (`PoseGraph.h:20-72`)
```cpp
struct Node {
    int    id;
    double x, z, heading;          // current estimate
    double x0, z0, heading0;       // pre-optimization snapshot
    int64_t timestamp_ns;
};
struct Edge {
    int    from_id, to_id;
    double dx, dz, dheading;       // measurement (in i's frame)
    double info_pos, info_heading; // diagonal Ω entries
};
std::deque<Node>   nodes_;
std::vector<Edge>  odom_edges_;
std::vector<Edge>  loop_edges_;
int                next_id_ = 0;
static constexpr int MAX_NODES = 500;     // PoseGraph.h:65
```

### 6.2 Public API
- `int addNode(double x, double z, double heading, int64_t timestamp_ns)` (`L20-51`): appends node, copies estimate→snapshot, auto-emits odometry edge `(dx = x − prev.x, dz = z − prev.z, dh = wrap(heading − prev.heading))`. Prunes oldest when `> MAX_NODES`, removing all edges referencing pruned ids.
- `void addOdometryEdge(from, to, dx, dz, dh, info_pos=1.0, info_heading=1.0)` (`L53-58`). **Default information weights: 1.0 / 1.0**.
- `void addLoopEdge(from, to, dx, dz, dh, info_pos=10.0, info_heading=5.0)` (`L60-67`, defaults at `PoseGraph.h:46-47`). **Loop edges weighted 10× / 5× stronger than odometry.**
- `double optimize(int max_iterations=10)` (`L85-249`). See §6.3.
- `bool getNode(int id, double& x, double& z, double& heading) const` (`L251-259`).
- `bool getLatestCorrection(double& dx, double& dz, double& dh) const` (`L261-268`): returns `(latest.x − latest.x0, ..., wrap(latest.heading − latest.heading0))`.
- `int getLoopEdgeCount() const` (`PoseGraph.h:61`, inline).
- `void reset()` (`L270-275`).

### 6.3 Optimization (Gauss-Newton on SE(2), `L85-249`)

- **Gauge fix:** node 0 is anchored. Free vars are nodes 1..N-1, each 3-DOF; `dim = 3 × (N-1)`.
- **Edge residual** (`L130-137`):
  ```
  R(θ_i)^T = [[cos θ_i, sin θ_i], [-sin θ_i, cos θ_i]]
  pred_t = R(θ_i)^T × (p_j − p_i)
  pred_dh = wrap(θ_j − θ_i)
  e_t = pred_t − [dx; dz]                   // 2×1
  e_h = wrap(pred_dh − dheading)            // 1×1
  ```
- **Jacobians** (3×3 each, `L156-167`):
  ```
  J_i = [[ -ct, -st,  -st·dx + ct·dz ],
         [  st, -ct,  -ct·dx − st·dz ],
         [   0,   0,            -1   ]]
  J_j = [[ ct,  st,   0 ],
         [-st,  ct,   0 ],
         [  0,   0,   1 ]]
  ```
- **Information matrix** (diagonal): `Ω = diag(info_pos, info_pos, info_heading)`.
- **Hessian/gradient accumulation** (`L177-211`):
  ```
  for each edge, for each pair (a,b) ∈ {i,j}²:
      H[var_a, var_b] += J_a^T Ω J_b
      b[var_a]        -= J_a^T Ω e
  ```
  `addBlock` and `addGradient` skip the anchor (`node_a == 0`), enforcing gauge fix.
- **Damping** (`L217-219`): `H[i,i] += 1e-6` (constant LM, not adaptive).
- **Solve** (`L222-225`): `cv::solve(H, b, dx, cv::DECOMP_CHOLESKY)`, with `cv::DECOMP_SVD` fallback.
- **Update** (`L228-240`): `x_i += dx_i, z_i += dz_i, heading_i = wrap(heading_i + dh_i)` for `i = 1..N-1`. Anchor untouched.
- **Convergence**: `step_max < 1e-5` early-exits (`L243`).

**Solver is Gauss-Newton with constant LM damping (1e-6). g2o is NOT used.**

### 6.4 SE(2) rationale
`PoseGraph.h:14-17`: SE(2) is correct for pedestrian floor-plane navigation; full SE(3) would add elevation/pitch/roll DOFs that are not observable from NavSight's 2D heading model.

---

## 7. WindowedBA — the active windowed bundle adjustment

`WindowedBA.h`, `WindowedBA.cpp`. Plan Step 6 / ADR-012.

### 7.1 Frame conventions (`WindowedBA.h:12-21`)

- Pose `(R, t)` where `R` is **world → cam** (`R_cw`), `t` is **camera centre in world frame** (`t_wc`).
- Therefore `p_cam = R × (p_world − t)`.
- Pose perturbation: `R_new = R_old × Exp(phi)`, `t_new = t_old + dt`, ordering `[delta_t (3); phi_delta (3)]`.
- Landmark perturbation: additive in world frame.

This matches `EKFState::CameraPose`, so `Tracker.cpp:3057-3067` translates `clone_snap` directly.

### 7.2 Public API (`WindowedBA.h:30-67`)
- `struct PoseObs { keyframe_id, R_in, t_in, R_out, t_out, is_anchor }`
- `struct FeatureObs { feature_id, p_w_in, p_w_out, std::vector<std::pair<int, cv::Vec2d>> obs }` — `(keyframe_id, pixel uv)` pairs.
- `struct Result { converged, iterations, initial_residual_sq, final_residual_sq, huber_rejects, solve_us }`.
- `Result solve(poses, features, fx, fy, cx, cy, max_iters=10, huber_thresh_px=1.5)`. Tracker overrides at the call site to **`max_iters=25`**, **`huber_thresh_px=1.5`** (`Tracker.cpp:3118-3119`).

### 7.3 Sliding-window size and trigger

The window is whatever Tracker passes in. `kickOffBARound(...)` is called on every keyframe (`Tracker.cpp:2639`) using current EKF clone snapshot. `WindowedBA.h:7-9` advertises **`K ≤ 10` and `N ≤ 30`** as design target. Anchor = `poses[0]` (oldest, `is_anchor=true`) per `Tracker.cpp:3065`.

### 7.4 Solver — Levenberg-Marquardt with Schur complement

Variables: `pose_dim = K × 6`, `lm_dim = N × 3`.

**Per-observation Jacobian** (`WindowedBA.cpp:21-34`):
```
p_c = R × (p_w − t),  q = p_w − t
e = (fx·p_c.x/p_c.z + cx,  fy·p_c.y/p_c.z + cy)
r = obs − e

J_proj = [[fx/z,     0,    -fx·x/z² ],
          [   0,  fy/z,    -fy·y/z² ]]      // 2×3

J_pose (2×6) = [ -J_proj·R ,  -J_proj·R·[q]_x ]      // (dt | phi)
J_lm   (2×3) =    J_proj·R
```
Sign convention: `r = obs − pred`, `J = de/dx`, GN form `H = J^T J`, `b = J^T r`.

**Huber IRLS weighting** (`L276-283`):
```
w = 1                        if |r| ≤ delta
w = sqrt(delta / |r|)        if |r| > delta
```
Applied to both J and r; `delta = huber_thresh_px = 1.5`.

**Gauge fixing** (`L174-176, 322-339, 354-363, 371-383`): if no `is_anchor` set, anchor pose 0. Anchor's Hpp / Hpl rows skipped during accumulation; defensively zeroed afterward; identity placed in anchor's 6×6 diagonal block to keep solve non-singular.

**LM damping** (`L230-237`):
```
lambda      = 1e-3   (initial)
lambda_min  = 1e-12
lambda_max  = 1e+12
```
Per-iteration inner try-loop (max **8 attempts**, `L396-526`):
- Damping: `Hpp[i,i] = Hpp_diag[i] × (1 + λ) + 1e-12`; same on Hll diagonals.
- Per-landmark `Hll[j]` 3×3 inverted; if `det < 1e-30` use `1e6 × I` placeholder (`L411-415`) so Schur reduction degrades to pose-only step for that landmark.
- On accept: `λ = max(λ × 0.5, λ_min)`. On reject: `λ = min(λ × 10, λ_max)`, restore Hpp diagonal, retry.

**Schur complement** (`L421-455`):
```
S  = Hpp − Hpl × Hll⁻¹ × Hpl^T            (pose_dim × pose_dim)
bs = bp  − Hpl × Hll⁻¹ × bl               (pose_dim × 1)
```
Implementation: per landmark `j`, extract `Hpl_j = Hpl[:, l_off..l_off+3]`, compute `tmp = Hpl_j × Hll_inv[j]`, then `S -= tmp × Hpl_j^T`, `bs -= tmp × bl_j`. **No `lm_dim × lm_dim` matrix is ever materialised.**

**Solve and back-substitution** (`L450-468`):
```
solve_ok = cv::solve(S, bs, dx_p, cv::DECOMP_CHOLESKY)
dx_l[j]  = Hll_inv[j] × (bl_j − Hpl_j^T × dx_p)
```

**Step application** (`L470-490`):
```
ts_try[i]  = ts[i] + dt          (i ≠ anchor)
Rs_try[i]  = Rs[i] × Exp(dphi)   (i ≠ anchor)
pws_try[j] = pws[j] + dx_l[j]
```

**Acceptance & convergence** (`L502-518`):
```
accept    = (new_r² < prev_r²) AND (new_pruned ≤ initial_pruned)
converged = (rel_change < 1e-4) OR (max_pose_step < 1e-6)
```
Pruned = points with `p_c.z ≤ 1e-3` (behind camera).

**Degeneracy guard** (`L217-228`): if `initial_pruned > obs_table.size() / 2`, bail and copy inputs to outputs.

### 7.5 Tracker integration

`Tracker::kickOffBARound` (`Tracker.cpp:3040-3203`):
- Skips if `lm_snap.size() < 3` (bumps `ba_skipped_too_few_landmarks`).
- Builds `poses` with `poses[0].is_anchor = true`.
- Skips if `features.size() < 3`.
- Marks `ba_in_flight_ = true`, joins prior thread, launches detached thread.

Acceptance gate (`Tracker.cpp:3125-3150`):
```
n_residual_pairs       = max(1, features_in.size())
avg_final_per_residual = result.final_residual_sq / (2 × n_residual_pairs)
residual_improved      = (initial > 0) AND
                         (final < 0.9 × initial OR avg_final_per_residual < 4.0)
fast_enough            = result.solve_us < 200'000     // BA_MAX_SOLVE_US, Tracker.h:384
accept                 = converged AND residual_improved AND fast_enough
```
`max_iters` was bumped 10→25 on 2026-05-04 after sim 1777919741934 showed all solves hitting `max_iters=10` and getting rejected.

On accept: refined landmarks published under `ba_result_mutex_`, consumed on next keyframe by `consumeBAResultIfReady` (`Tracker.cpp:3205+`), which reseeds each SLAM feature via `removeSlamFeature` + `addSlamFeature`. ADR-006 forbids direct EKF mean/covariance writes from a side channel; remove + add is the canonical re-promotion path.

---

## 10. Interactions — when does loop closure fire and what does it do?

### 10.1 Trigger chain (per `Tracker.cpp:3304-3336`)

```
[Camera thread, every accepted keyframe]
   storeKeyframeDescriptors()
       ├──► loop_closure_.addKeyframe(...)          (Tracker.cpp:2603)
       │     — adds to BoW DB + caches descriptors/kp/3D/pose/yaw
       └──► publishLoopClosureQueryKeyframe(...)    (Tracker.cpp:2623)
             — wakes worker via loop_closure_cv_

[Loop-closure worker thread, ≥ 1 Hz]
   loopClosureWorkerLoop()                          (Tracker.cpp:3396-3485)
       ├──► snapshot pending query under loop_closure_query_mutex_
       ├──► loop_closure_.tryDetectLoop(...)         (lock-free during PnP)
       └──► on success: write LoopMatch under loop_closure_result_mutex_

[Camera thread, every frame]                        (Tracker.cpp:2649)
   consumeLoopClosureMatchIfReady()                 (Tracker.cpp:3487-3639)
       ├──► if pending: pull match → active slot, start 10-frame damping ramp
       └──► if active: compose target world-IMU pose,
            call ekf_.updateAbsolutePose(...)
```

### 10.2 What it does to `EKFState` (`Tracker.cpp:3487-3639`)

1. **Pick up fresh match** under `loop_closure_result_mutex_` (`L3492-3501`): `damping_remaining = LOOP_CLOSURE_DAMPING_FRAMES = 10`.
2. **Damping schedule** (`L3525-3536`):
   ```
   k          = 10 − damping_remaining        // 0..9
   strength   = 1 − k/10                       // 1.0 → 0.1
   damping_inv = 1 / max(0.01, strength²)      // 1.0 → 100.0
   ```
3. **Compose target world-IMU pose** (`L3575-3597`):
   ```
   target_R_world_cam = R_world_cam_match × R_now_to_match
   target_t_cam_world = R_world_cam_match × t_now_to_match + t_cam_world_match
   target_R_GtoI      = R_bc^T × target_R_world_cam.t()
   target_p_world     = target_t_cam_world             // body=camera (handheld)
   ```
   `R_bc` is fetched live from `ekf_.getExtrinsicsRotation()` (Step 8b made R_bc EKF-refined instead of hardcoded `diag(1,-1,-1)`).
4. **Inflated variances** (`L3601-3608`):
   ```
   sigma_axis_sq_R = (0.34907)² × damping_inv
   sigma_p         = max(2.0, 0.032 × total_path_m_)
   var_p           = sigma_p² × damping_inv
   ```
5. **EKF update** (`L3610-3611`):
   ```
   ok = ekf_.updateAbsolutePose(target_R_GtoI, target_p_world,
                                sigma_axis_sq_R, var_p)
   ```
   Internally runs χ² gate (`m² > 22.5` ⇒ reject), and if it passes calls `applyMSCKFUpdate(H, r, R_noise)` — the canonical EKF correction path that updates **both the IMU state and all clones** consistently per the EKF covariance.

### 10.3 Why absolute-pose channel (not relative) — `Tracker.cpp:3539-3573`

The matched keyframe's clone is almost always older than the EKF sliding window (temporal exclusion 30 s vs ~5–10 s clone window). `updateRelativePose` / `updateRelativeRotation` both require the matched clone to live in the window — when it doesn't, the correction silently drops. `updateAbsolutePose` consumes a target world-frame IMU pose and applies the correction directly, independent of clone availability.

### 10.4 BA's path back to the EKF — `Tracker::consumeBAResultIfReady`

On accept, each refined landmark is reseeded by removing its existing SLAM slot and re-adding it via `addSlamFeature` anchored at the same clone. ADR-006 forbids overwriting EKF mean / covariance from a side channel; remove + add is the canonical re-promotion path.

### 10.5 Pose-graph correction propagation — N/A

No active pose graph. Correction propagates exclusively via the EKF covariance (cross-correlation `P_` between IMU state and clones). The dead `PoseGraph` would have distributed correction across all 2D nodes via Gauss-Newton (§6.3), but that path is not compiled.

---

## 11. Outstanding issues — flagged

### 11.1 χ² gate vs. dynamic sigma (still load-bearing)

The "5.8 M m² vs 22.5" symptom from project context is addressed by commit `d64a4ff` — `LOOP_CLOSURE_DRIFT_RATE = 0.032` makes `var_p` grow with path length so `m² ≈ |r_p|² / var_p` stays in budget. **But:**

- Fix is **derivation-driven, not measurement-driven** for a given walk. If real drift on a specific walk exceeds 15 %/100 m, m² will still exceed 22.5 and corrections will be rejected.
- Per-block diagnostics (`m²_R`, `m²_p` at `EKFState.cpp:1009-1036`) are logged but not yet plumbed into rejection counters; only aggregate `m² > 22.5` reject is counted.
- The MEMORY index entry "Step 7 detection works (30/168 accepts). Correction blocked by chi² gate (m²~5.8M vs 22.5)" pre-dates `d64a4ff`. **Whether the dynamic-sigma fix actually unblocks corrections has NOT been validated against a real walk** in available simulation data (no recorded sim with sigma=6.0 either, per the 2026-05-06 session memory note).

### 11.2 ORB is not 180°-invariant

Mitigated by heading gate (`LoopClosureDetector.cpp:464-475`, threshold π/2). However:
- Gate uses raw `scalar_heading_` (Tracker's heading). If heading is itself drifting, a genuinely-same-direction revisit can fail the gate.
- The π/2 threshold is tighter than ORB's ~30° invariance band but looser than the 180° reversal it protects against. Forward-vs-sideways-at-90° revisits pass the gate but may produce poor PnP geometry — there's no separate counter for "PnP RANSAC failed despite passing the heading gate" beyond `loop_closure_rejects_pnp`.

### 11.3 Mapper / PoseGraph dead code

`Mapper.cpp:31` calls `loop_closure_detector_.reset()` — the active `LoopClosureDetector` has **no** `reset()` method. `Mapper.cpp:591-594` calls `detectLoopClosure(descriptors, keypoints, pos, heading_float, matched_kf)` — this overload **does not exist**. Same for `LoopClosureKeyframe`. Re-enabling requires a non-trivial refactor; cannot be done by flipping the comments at `app/CMakeLists.txt:62-63`. ADR-006 should remain authoritative.

### 11.4 BA acceptance thresholds

`Tracker.cpp:3145-3148`: residual gate is `final < 0.9 × initial OR avg_final_per_residual < 4.0` (px²). The `4.0 px²` floor corresponds to ≈ 2 px RMS reproj. Comment `L3132-3140` notes it was loosened from "residual halved (50 %)" after real walks regularly missed the tight gate. Bumping `max_iters` 10→25 (`L3110-3119`) was the matching change. **No counter logs how often `residual_improved` triggers via the OR arm vs the `<0.9× initial` arm.**

### 11.5 Documentation — `LOOP_CLOSURE_BASE_TRANS_SIGMA_M` no longer exists

The study brief and `docs/handoff/memory/reference_lc_constants.md` reference `LOOP_CLOSURE_BASE_TRANS_SIGMA_M = 6.0`. **The current `Tracker.h:528-529` has replaced this with `max(LOOP_CLOSURE_PNP_SIGMA_FLOOR_M = 2.0, LOOP_CLOSURE_DRIFT_RATE × total_path_m_)` (commit `d64a4ff`).** Memory note and reference doc are stale.

### 11.6 Loop-closure worker has no time-budget watchdog analogous to `BA_MAX_SOLVE_US`

`tryDetectLoop` is called single-shot from worker. If a dense vocabulary + large keyframe DB makes it exceed the 1 Hz period, queries back up. PnP cost is bounded by `kPnpMaxIters = 100`, but no explicit watchdog.

---

## 12. Cross-references (file:line directory)

- `LoopClosureDetector.h` — public interface (190 lines).
- `LoopClosureDetector.cpp:71` — `kBowScoreFloor = 0.005`.
- `LoopClosureDetector.cpp:79` — `kCovisibilityK = 10`.
- `LoopClosureDetector.cpp:91` — `kPnpMinInliers = 15`.
- `LoopClosureDetector.cpp:95` — `kLoweRatio = 0.75f`.
- `LoopClosureDetector.cpp:101-103` — PnP RANSAC params (4.0 px, 0.99, 100 iters).
- `LoopClosureDetector.cpp:108` — `kBowTopN = 4`.
- `LoopClosureDetector.cpp:316-600` — `tryDetectLoop` body.
- `LoopClosureDetector.cpp:465` — `kMaxHeadingDiffRad = M_PI / 2.0`.
- `Tracker.h:505-507` — query period / temporal excl. / damping frames.
- `Tracker.h:528-538` — dynamic-sigma constants.
- `Tracker.h:384` — `BA_MAX_SOLVE_US = 200'000`.
- `Tracker.cpp:2354-2629` — addKeyframe call site (with cross-keyframe triangulation).
- `Tracker.cpp:3040-3203` — `kickOffBARound`.
- `Tracker.cpp:3205+` — `consumeBAResultIfReady`.
- `Tracker.cpp:3396-3485` — `loopClosureWorkerLoop`.
- `Tracker.cpp:3487-3639` — `consumeLoopClosureMatchIfReady`.
- `EKFState.cpp:925-1060` — `updateAbsolutePose` (the χ² gate).
- `EKFState.cpp:1000` — `kChi2Threshold = 22.5`.
- `WindowedBA.cpp:148-557` — `solve()` (full LM + Schur).
- `app/CMakeLists.txt:62-63` — Mapper/PoseGraph commented out.
- `docs/adr/ADR-006-mapper-pipeline-disabled.md`.
- `docs/adr/ADR-012-windowed-bundle-adjustment.md`.
- `docs/adr/ADR-013-same-session-loop-closure.md`.

---

## Summary of key findings

1. **`Mapper.cpp` and `PoseGraph.cpp` are not compiled** (commented out at `app/CMakeLists.txt:62-63`). They reference an old `LoopClosureDetector` API that no longer exists; not just a flag-flip away from re-enabling.
2. **The active loop-closure path is DBoW2 → PnP → `EKFState::updateAbsolutePose`**, with χ²(0.999,6) ≈ 22.5 gate at `EKFState.cpp:1000`.
3. **`LOOP_CLOSURE_BASE_TRANS_SIGMA_M = 6.0` does not exist in current code.** It was replaced by `sigma_p = max(2.0 m, 0.032 × total_path_m_)` in commit `d64a4ff`. The "0.032 m/m walked" rate is derived: `0.15 (15 %/100 m drift) ÷ √(22.5 − 0.84) = 0.15 ÷ 4.65 = 0.032` (`Tracker.h:524-527`).
4. **Adaptive BoW threshold** replaced fixed 0.05: `max(min over 10 recent neighbors, 0.005)`. Original 0.05 was a misreading of Galvez-Lopez 2012.
5. **Min PnP inliers is 15, not 30** (`LoopClosureDetector.cpp:91`; plan §line 723's 30 was structurally unreachable with `MAX_SLAM_FEATURES = 12`).
6. **Heading gate (π/2)** at `LoopClosureDetector.cpp:464-475` mitigates ORB's lack of 180° invariance; rejects opposite-direction revisits before BFMatcher.
7. **WindowedBA is hand-rolled LM+Schur with Huber IRLS**, K≤10/N≤30 design target, gauge-fixed at oldest pose, accepted via `(converged AND (residual <0.9× OR avg<4 px²) AND solve <200 ms)` at `Tracker.cpp:3145-3150`.
8. **PoseGraph (dead) was Gauss-Newton on SE(2) (x,z,heading)** with constant LM damping `1e-6` and Cholesky→SVD solve, gauge-fixed at node 0; loop edges weighted 10×/5× over odometry.
9. **The memory entry "m²~5.8M vs 22.5" pre-dates the dynamic-sigma fix**; whether `d64a4ff` actually unblocks corrections is unvalidated against a real walk.
