# 02 — NavSight Visual Tracking Pipeline

> Source files studied (full reads):
> - `app/src/main/cpp/Tracker.h` (616 lines)
> - `app/src/main/cpp/Tracker.cpp` (3652 lines)
> - `app/src/main/cpp/TrackKLT.h` (72 lines)
> - `app/src/main/cpp/TrackKLT.cpp` (158 lines)
> - `app/src/main/cpp/FeatureManager.h` (411 lines)
> - `app/src/main/cpp/FeatureManager.cpp` (709 lines)
> - `app/src/main/cpp/KeyframeDescriptors.h` (30 lines)

---

## 1. Frame Ingestion Pipeline

**Entry**: `Tracker::processFrame(yuv_data, w, h, ts_ns, imu, frame_out)` — declared `Tracker.h:65`, implemented `Tracker.cpp:623`. Called once per camera frame with NV21 buffer.

**Step-by-step** (`Tracker.cpp:623`+):

1. Arg validation `Tracker.cpp:629-632`: null/non-positive dims → empty `VisionOutput`.
2. **NV21 → grayscale** `Tracker.cpp:644-646`:
   ```cpp
   cv::Mat yuv(height + height/2, width, CV_8UC1, const_cast<uint8_t*>(yuv_data));
   cv::cvtColor(yuv, gray_buf_, cv::COLOR_YUV2GRAY_NV21);
   ```
   Zero-copy wrap; `gray_buf_` is a reusable member buffer.
3. **Brightness probe + adaptive CLAHE** `Tracker.cpp:648-657`:
   - `frame_brightness = cv::mean(gray_buf_)[0] / 255.0`
   - `is_low_light = (frame_brightness < 0.12)`
   - CLAHE (`clipLimit=2.0`, tile `8×8`, constructed `Tracker.cpp:42`) applied **only if** `frame_brightness < 0.55`.
4. **Motion-blur gate** `Tracker.cpp:659-692`:
   - `blur_var = measureBlur(gray_buf_)` — variance of Laplacian on **centre 50×50% crop** (`Tracker.cpp:473-496`).
   - `frame_is_blurry = (blur_var >= 0.0 && blur_var < BLUR_VAR_THRESH=80.0)` (`Tracker.h:586`).
   - Blurry frames still propagate IMU and run ZUPT, but skip geometric verification, MSCKF, SLAM, and ORB reloc.
5. **Intrinsics + lens corrector** `Tracker.cpp:694-703`. Defaults if unset: `fx_use = 0.7*width`, `fy_use = fx_use`, `cx_use = w/2`, `cy_use = h/2`.
6. **Init gate** `Tracker.cpp:706-708`: early-return if `!initialized_`.
7. **First-frame branch** `Tracker.cpp:744-772`: `detectGridFeatures` → `assignIds` → `prev_gray_/prev_pts_/prev_timestamp_ns_` set → `ekf_.initializeFull` → `storeKeyframe` + `storeKeyframeDescriptors`. Returns empty output (no flow yet).

**Crucial fact**: KLT runs on the **raw (CLAHE-stretched) gray** — there is **no full-image undistortion**. Undistortion is applied only to **matched point pairs** at `Tracker.cpp:1101` via `lens_.undistortMatchedPoints(...)`, immediately before `findEssentialMat`. No image pyramid is pre-built; `cv::calcOpticalFlowPyrLK` builds it internally each call. No ROI cropping.

---

## 2. KLT Pyramid Tracking

**Class**: `TrackKLT` (`TrackKLT.h`, `TrackKLT.cpp`); owned by `Tracker::klt_` (`Tracker.h:308`).

**Constants** (`TrackKLT.h:64-70`):

| Constant | Value |
|---|---|
| `PYRAMID_LEVELS` | 4 |
| `WINDOW_SIZE` | 21 (was 31; reduced for 640×480) |
| `RANSAC_CONF` | 0.999 |
| `RANSAC_THRESH` | 1.5 px |
| `MIN_INLIERS` | 8 |
| `FB_CHECK_THRESH` | 4.0 (= 2 px squared) |

**`TrackKLT::track`** (`TrackKLT.cpp:16-89`):
- **Step 1** (`L:28-37`) IMU rotation prediction: if `delta_R != I`, `predictPoints` warps via homography `H = K * R * K^-1` per point (`TrackKLT.cpp:131-157`).
- **Step 2** (`L:50-55`) Forward LK with `cv::TermCriteria(COUNT+EPS, 30, 0.01)`, flags `OPTFLOW_LK_GET_MIN_EIGENVALS | OPTFLOW_USE_INITIAL_FLOW`.
- **Step 3** (`L:57-78`) Backward LK + FB consistency: zero `status[i]` if `‖prev - back_pt‖² >= 4.0`.
- **Step 4** (`L:81-88`) Boundary check: zero `status[i]` if outside `[0,W)×[0,H)`.

**Adaptive window sizing** (Plan Step 5, `Tracker.cpp:842-854`):
```cpp
expected_disp_px = fx_use * gyro_norm * dt_klt;
win_sz = clamp(int(2.0 * expected_disp_px + 11.0), 21, 41);
if ((win_sz & 1) == 0) win_sz += 1;   // KLT requires odd
```
Steady walk → 21 px; fast head turn → up to 41 px. Counter `klt_adaptive_window_hits` increments when `>21`.

**Geometric verification** (`TrackKLT.cpp:91-129`): `findEssentialMat` with `RANSAC_CONF=0.999`, `RANSAC_THRESH=1.5`; `recoverPose` returns unit-norm `t_cam`. Returns true iff `inliers >= 8`.

**Min-distance grid**: KLT does not enforce spacing. Spacing comes from `goodFeaturesToTrack`'s `minDistance = MIN_DIST = 10.0 px` (`Tracker.h:553`), applied per-cell in `FeatureManager::detectGridFeatures` (`FeatureManager.cpp:77`) and `replenishSparse` (`FeatureManager.cpp:201`).

**Frame-to-frame forwarding** (`Tracker.cpp:868-883` filter; `Tracker.cpp:2651-2653` end of frame):
```cpp
gray_buf_.copyTo(prev_gray_);
prev_pts_ = next_good_buf_;
prev_timestamp_ns_ = timestamp_ns;
```
Surviving tracks keep their `feature_age` (incremented) and `feature_id`. New tracks (replenish) get `age=0` and fresh IDs from `assignIds`.

---

## 3. ORB Descriptor Extraction

**ORB is computed only at keyframes**, never per frame. Two call sites:
1. `FeatureManager::storeKeyframeDescriptors` (`FeatureManager.cpp:518-587`) — called from `Tracker.cpp:766` (first frame) and `Tracker.cpp:2326` (subsequent KFs).
2. `Tracker::tryRelocalizeWithORB` (`Tracker.cpp:2766-2952`) — runs on current `gray` only when `low_inlier_streak_ >= RELOC_TRIGGER_FRAMES=3`.

**`cv::ORB::create` parameters** (`FeatureManager.cpp:534-545`, mirrored at `Tracker.cpp:2780-2784`):

| Param | Value |
|---|---|
| `nfeatures` | `ORB_TARGET_FEATURES=500` (`FeatureManager.h:124`; bumped from 250 for Step 7 BoW discriminability) |
| `scaleFactor` | 1.2f |
| `nlevels` | 8 |
| `edgeThreshold` | 31 |
| `firstLevel` | 0 |
| `WTA_K` | 2 |
| `scoreType` | `cv::ORB::HARRIS_SCORE` |
| `patchSize` | 31 |
| `fastThreshold` | `ORB_FAST_THRESHOLD=10` (`FeatureManager.h:125`) |

**Pre-blur** (`FeatureManager.cpp:550-551`, `Tracker.cpp:2787`): `cv::GaussianBlur(gray, blurred, Size(0,0), ORB_PREBLUR_SIGMA=1.0)`.

**`KeyframeDescriptors`** (`KeyframeDescriptors.h:21-30`):
```cpp
struct KeyframeDescriptors {
    uint64_t                  keyframe_id;
    double                    timestamp_ns;       // double, not int64
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;        // CV_8U, 32 cols
    std::vector<int>          feature_ids;        // parallel; -1 if no KLT match
    cv::Matx33d               R_world_cam = I;    // optional
    cv::Vec3d                 t_cam_world = 0;    // optional
    bool                      has_pose = false;
};
```
`feature_ids` populated by spatial proximity to KLT corners within `ORB_KLT_MATCH_RADIUS=3.0f px` (`FeatureManager.cpp:563-581`).

**Ring buffer**: `keyframe_descriptors_` `std::deque<KeyframeDescriptors>`; cap `KEYFRAME_DESC_RING_SIZE=50` (`FeatureManager.h:409`); `pop_front` on overflow (`FeatureManager.cpp:583-586`). Memory ~1.4 MB at full.

**Two-keyframe ORB triangulation** (`Tracker.cpp:2424-2588`): At each new KF, BFMatch (HAMMING) the latest ORB descriptors against the most recent neighbour KF (look-back up to 10), Lowe ratio 0.75, baseline gate `0.1 m ≤ bn ≤ 5.0 m` (`Tracker.cpp:2479`), `cv::triangulatePoints`, depth in both views `0.5–50 m` (`Tracker.cpp:2567-2570`). Result fills `pts3d_world` for ORB rows that did NOT inherit a SLAM-promoted slot — used by loop-closure PnP.

---

## 4. FeatureManager — Lifecycle

**Three categories of state**:

| Map | Type | Key | Purpose |
|---|---|---|---|
| `active_tracks_` | `unordered_map<int, vector<FeatureObservation>>` | feature_id | per-clone NORMALISED uv (for MSCKF/BA) |
| `lifecycle_` | `unordered_map<int, FeatureLifecycle>` | feature_id | promotion gate counters |
| `keyframe_descriptors_` | `deque<KeyframeDescriptors>` | keyframe_id | ORB ring buffer |

**`FeatureLifecycle`** (`FeatureManager.h:163-176`): `feature_id, age, kf_count, slam_slot (-1=unpromoted), rms_bad_consecutive, last_rms_px, last_obs_ns, last_p_global, has_p_global, anchor_clone_id`.

**Birth**: `assignIds(count)` increments `next_feature_id_` (monotonic, never recycled) — `FeatureManager.cpp:245-251`. Called from `Tracker.cpp:752, 1688, 1700`.

**Tracked**: per frame at `Tracker.cpp:1641-1668`, every surviving KLT track with `feature_id >= 0` produces:
- `feature_mgr_.addObservation(fid, clone_id, normalised_uv)` — `((px-cx)/fx, (py-cy)/fy)`.
- `feature_mgr_.noteObservation(fid, row_ts_ns, false)` — bumps `age`, `last_obs_ns`. `row_ts_ns` is rolling-shutter-corrected (Step 8c).

At keyframe storage (`Tracker.cpp:2333-2335`): `noteKeyframe(fid)` → `kf_count++`.

**Promotion gate** (`FeatureManager::getPromotableFeatures`, `FeatureManager.cpp:400-425`):
- `slam_slot < 0` (not already promoted)
- `age >= min_obs` — Tracker passes `min_obs=8` (`Tracker.cpp:1830`; reduced from 12 on 2026-05-04 because BA never fired)
- `kf_count >= min_kf=2`
- `last_rms_px <= 1.5 px` (or unset)
- (`has_p_global` requirement was removed — chicken-and-egg, see comment `FeatureManager.cpp:408-420`)

**Promotion procedure** (`Tracker.cpp:1859-1937`):
1. Two-view midpoint triangulation in world frame between first/last surviving observation clones.
2. **Chirality gate**: `tA > 0.05 && tB > 0.05`.
3. **Reprojection RMSE** over all surviving observations: `rms <= 1.5 px`.
4. `ekf_.addSlamFeature(fid, p_world, anchor)` returns slot ≥ 0 on success.
5. `feature_mgr_.setSlamSlot(fid, slot)` + `noteTriangulation`. `slam_promotions_total` increments.
- Cap: `EKFState::MAX_SLAM_FEATURES = 12`.

**Per-frame triangulation** (geometric verification path, `Tracker.cpp:1124-1163`): `cv::triangulatePoints` with `P1 = K[I|0]` and `P2 = K[R_vo|t_vo]`. Reprojection chi² gate `err_sq <= 5.991` (chi² 2-DOF 95%).

**Demotion** (`FeatureManager::markSlamFeatureRMS`, `FeatureManager.cpp:427-438`): `rms_bad_consecutive++` if `rms_px > 3.0`, else reset. `getDemoteCandidates`: `rms_bad_consecutive >= 3`. `getLostSlamFeatures(now, 1e9)`: `last_obs_ns` older than 1 s. Both processed in **descending-slot order** (`Tracker.cpp:2018-2089`) to avoid stale-slot bugs.

**Lifecycle prune** (`FeatureManager::pruneStaleLifecycle`, `FeatureManager.cpp:589-631`): Drops entry iff not in current `feature_ids_`, not SLAM-promoted, AND `last_obs_ns > LIFECYCLE_KEEP_NS=100 ms` ago.

**Marginalisation**: when EKF window slides, `Tracker.cpp:1744-1747` calls `pruneObservations(min_clone_id)` (`FeatureManager.cpp:290-309`).

**MSCKF lost-feature extraction** (`FeatureManager.cpp:263-288`): returns features no longer tracked AND `>= min_obs=4` observations. Called via `getMSCKFCandidates(...)` at `Tracker.cpp:1729`. Move-out semantics — feature_id removed from `active_tracks_` after call.

---

## 5. Keyframe Selection

**Trigger** (`Tracker.cpp:2315`):
```cpp
if (frames_since_keyframe_ >= 15
    || (tracked < MIN_FEATURES/2 && frames_since_keyframe_ > 3)) { ... }
```
- 15 frames ≈ 0.5 s @ 30 fps.
- `MIN_FEATURES/2 = 40`.
- **No explicit parallax-px keyframe gate** — parallax (`MIN_PARALLAX_PX=0.8`) gates whether the pose update path runs at all.

**Storage** (`Tracker.cpp:2316-2336`):
1. `feature_mgr_.storeKeyframe(...)` — full grayscale clone (`FeatureManager.cpp:225-241`).
2. `feature_mgr_.storeKeyframeDescriptors(...)` — ORB.
3. `noteKeyframe(fid)` for every tracked id.
4. `frames_since_keyframe_ = 0`.
5. `loop_closure_.addKeyframe(...)` + `publishLoopClosureQueryKeyframe(...)`.
6. `consumeBAResultIfReady()` then `kickOffBARound(timestamp_ns)`.

**KF heading correction** (Step 2.1) at `Tracker.cpp:2129-2311`, fires at `frames_since_keyframe_ >= 14`:
- `matchAgainstKeyframe` requires `>= MIN_KF_MATCHES=20`.
- `findEssentialMat` + `recoverPose` requires `inl >= 20`.
- Gravity-align via Madgwick roll/pitch + `R_bc^T R_kf R_bc` to body frame.
- Z-up yaw: `atan2(R_aligned[1,0], R_aligned[0,0])`.
- Drift gate: `|drift| < 20° = 20*M_PI/180` (`Tracker.cpp:2258`).
- Inject via `ekf_.updateGravityAlignedYaw(yaw_meas, var_yaw, roll, pitch)`.
- Variance: `σ_yaw = RANSAC_THRESH/(focal * √N_inliers)`, floored at `var=1e-4` (~0.6°²).

**Sliding window**: BA snapshot uses `kMaxClones=5` (`Tracker.cpp:3009`). Per-frame `addClone` at `Tracker.cpp:1638`. Keyframe storage cap `MAX_KEYFRAMES=10` (`FeatureManager.h:358`).

---

## 6. Outlier Rejection — Layered Gates

| Gate | Threshold | Where |
|---|---|---|
| KLT FB consistency | `‖Δ‖² < 4.0` | `TrackKLT.cpp:73` |
| Image bounds | inside frame | `TrackKLT.cpp:81-88` |
| Min flow | `mean_flow >= 0.4` | `Tracker.cpp:921` (`MIN_FLOW_PX=0.4`) |
| Max flow | `mean_flow > 150.0` → drop | `Tracker.cpp:920` (`MAX_FLOW_PX=150.0`) |
| Parallax | `mean_flow >= 0.8` | `Tracker.cpp:922` (`MIN_PARALLAX_PX=0.8`) |
| Tracked count | `tracked >= 8` | `Tracker.cpp:1095` |
| Pure-rotation gyro gate | `‖gyro‖ > 2.0 rad/s` | `Tracker.cpp:831` (`GYRO_ROT_ONLY_THRESH=2.0`) |
| Pure-rotation Rayleigh gate | `R/N < 0.3` AND gyro gate | `Tracker.cpp:945-979` (`FLOW_RAYLEIGH_REJECT=0.3`) |
| 5-pt RANSAC (essential) | `conf=0.9999`, `thresh=1.5 px` (Tracker calls); `0.999` (TrackKLT) | constant mismatch |
| Min inliers | 8 | `Tracker.h:560` |
| Min inlier ratio | 0.25 | `Tracker.h:566` |
| Translation degeneracy | `‖t_vo‖ < 0.001` (or `< 0.01` if `mean_flow < 1.5`) | `Tracker.cpp:1115-1118` |
| Reprojection χ² (triangulation) | `err² > 5.991` → drop | `Tracker.cpp:1153` |
| SLAM init RMS | `rms <= 1.5 px` | `Tracker.cpp:1906` |
| Chirality | `tA, tB > 0.05` | `Tracker.cpp:1884` |
| SLAM update χ² | `RANSAC_THRESH²` | `Tracker.cpp:1971` |
| SLAM demote | `rms_bad_consecutive >= 3` (`rms > 3.0`) | `FeatureManager.cpp:443` |
| SLAM expire | `now - last_obs_ns > 1e9 ns` | `FeatureManager.h:222` |
| Disp cap | `disp <= 2.0 m/s × max(dt, 0.03)` | `Tracker.cpp:1490-1494` |
| Loop-closure χ² rotation | base σ = 20° = 0.34907 rad | `Tracker.h:538` |
| Loop-closure χ² translation | dynamic `max(2.0, 0.032×total_path_m)` | `Tracker.h:528-529` |

**Pure-rotation Rayleigh detail** (`Tracker.cpp:945-979`): per-feature unit-flow direction sum yields `R/N`; rotation→0, translation→1. Combined with gyro gate so it can only ADD restrictions.

**MSCKF Huber + damping**: handled inside `UpdaterMSCKF::processLostFeatures` (member `msckf_updater_`, `Tracker.h:313`), called at `Tracker.cpp:1731`. Skipped on blur frames (`Tracker.cpp:1728`).

**Loop-closure dynamic σ derivation** (`Tracker.h:507-538`):
```
sigma_p = max(2.0, 0.032 * total_path_m_)
0.032 = (15%/100m drift rate) / sqrt(chi2(0.999,6) - 0.84) = 0.15/4.65
```
With `chi²(0.999, 6) = 22.5`. Floor 2.0 m matches PnP accuracy floor.

---

## 7. Time-Offset Estimation (Step 8a)

**Two-stage strategy**:

**Stage 1 — Cross-correlation warmup** (`Tracker.cpp:981-1050`):
- Buffer `TdSample{flow_rate, gyro_rate, ts_ns}` for `TD_WARMUP_FRAMES=60` (`Tracker.h:296`) ≈ 2 s @ 30 fps.
- Compute Pearson cross-correlation at lags `[-3, +3]` frames (`Tracker.cpp:1011`) ≈ ±100 ms.
- Pick `best_lag_idx` maximising `corr`.
- Convert: `estimated_td = best_lag_idx * avg_frame_dt`. Clamp to `±50 ms` (`Tracker.cpp:1038`).
- Warm-start `ekf_.setTimeOffset(estimated_td)`.

**Stage 2 — EKF online refinement**: `td = ekf_.getTimeOffset()` read each frame (`Tracker.cpp:792`); used to shift IMU integration: `imu.integrate(prev_ts + td_ns, ts + td_ns)` (`Tracker.cpp:796`). EKF refines via measurement-residual Jacobian (in `EKFState`, out of scope).

Tunables: signal-energy gate `std_f, std_g > 1e-6` (`Tracker.cpp:1008`); buffer cleared after computation (`Tracker.cpp:1047-1048`).

---

## 8. Rolling-Shutter Compensation (Step 8c)

**Setter API**: `Tracker::setRollingShutterSkew(int64_t row_skew_ns)` — `Tracker.cpp:74-79`. Source: Android Camera2 `CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW`. Zero disables.

**Per-row timestamp injection** (`Tracker.cpp:1659-1665`):
```cpp
const int64_t row_ts_ns = (rolling_shutter_row_skew_ns_ > 0)
    ? (timestamp_ns + (int64_t)(next_good_buf_[i].y / (double)height
                                * (double)rolling_shutter_row_skew_ns_))
    : timestamp_ns;
feature_mgr_.noteObservation(feature_ids_[i], row_ts_ns, false);
```
Feature at row `y` of an `H`-row image with skew `S` ns gets `ts + (y/H)*S`. This per-row stamp populates `FeatureLifecycle::last_obs_ns`.

**State**: `rolling_shutter_row_skew_ns_` (`Tracker.h:228`), default 0; reset in `reset()` (`Tracker.cpp:426`); mirrored to `eventCounters().rolling_shutter_skew_ns` for sim JSON.

**What is NOT done**: No image rectification — a single global `timestamp_ns` is still used for `propagateIMU`. Only per-feature observation timestamps vary. Lighter than full RS rectification.

---

## 9. Public API — Every Function

### Constructor / destructor
| Sig | Where | Purpose |
|---|---|---|
| `Tracker()` | `Tracker.cpp:32-44` | init `global_R_=I`, allocate buffers, CLAHE 2.0/8×8 |
| `~Tracker()` | `Tracker.cpp:466-469` | join BA + LC workers |

### Per-frame entry
| Sig | Where |
|---|---|
| `VisionOutput processFrame(yuv,w,h,ts,imu,frame_out)` | `Tracker.cpp:623` |

### Setters
| Sig | Where | Purpose |
|---|---|---|
| `setIntrinsics(fx,fy,cx,cy)` | `Tracker.cpp:48-56` | seed K + LensCorrector + EKF SLAM intrinsics |
| `setDistortion(k1..k6,p1,p2)` | `Tracker.cpp:58-63` | 8-coef rational |
| `setUserScaleCorrection(c)` | `Tracker.cpp:65-68` | clamp `[0.1, 5.0]` |
| `setRollingShutterSkew(ns)` | `Tracker.cpp:74-79` | Step 8c |
| `setExtrinsicsRotation(R_bc[9])` | `Tracker.cpp:84-93` | Step 8b |
| `setInitialHeading(rad)` | `Tracker.cpp:322-358` | bootstrap or post-init R_GtoI correction |
| `setDepthMap(d,w,h)` | `Tracker.cpp:95-103` | MiDaS ~1 Hz |
| `addImuData(ts, ax..gz)` | `Tracker.cpp:575-587` | feeds `InertialInitializer` |
| `loadStoredCalibration(R, gb, ab)` | `Tracker.cpp:597-607` | bypass online init |
| `clearInitTimeout()` | `Tracker.cpp:593-595` | extend init timer |
| `loadLoopClosureVocabulary(path)` | `Tracker.cpp:3339-3366` | DBoW2 vocab + start LC worker |
| `reset()` | `Tracker.cpp:360-462` | full teardown |

### Getters (thread-safe)
| Sig | Where | Returns |
|---|---|---|
| `getInitStatus()` | `Tracker.cpp:589-591` | enum |
| `getInitialRotation()` | `Tracker.cpp:609-611` | 3×3 CV_64F |
| `getCalibratedGyroBias() / getCalibratedAccelBias()` | `Tracker.cpp:613-619` | rad/s, m/s² |
| `getSmoothScale()` | `Tracker.cpp:532-534` | m/VIO unit |
| `getHeading()` | `Tracker.cpp:535-538` | rad CW from North |
| `isInitialized()` | inline `Tracker.h:118` | bool |
| `getEKF()` | inline `Tracker.h:119` | const ptr |
| `getLastVisualYawVariance()` | `Tracker.cpp:539-542` | rad² or -1 |
| `getLastDepthScaleVariance()` | `Tracker.cpp:543-546` | dimensionless² or -1 |
| `getPositionCovarianceXZ(out[3])` | `Tracker.cpp:548-573` | EKF P[12,12], P[12,13], P[13,13] (m²); name retained for ABI/JNI but is X-Y horizontal post-Z-up alignment |
| `getLoopClosureDetector()` | inline `Tracker.h:154` | mutable ref |

### Private helpers
| Sig | Where |
|---|---|
| `estimateScaleFromSteps(disp,dt,imu)` | `Tracker.cpp:498-526` |
| `tryRelocalizeWithORB(gray, current_pts)` | `Tracker.cpp:2766-2952` |
| `measureBlur(gray) const` | `Tracker.cpp:473-496` |
| `applyDepthScaleConstraint(...)` | `Tracker.cpp:108-320` |
| `kickOffBARound(ts) / consumeBAResultIfReady() / shutdownBA()` | `Tracker.cpp:2988+/3205+/3293+` |
| `publishLoopClosureQueryKeyframe(...) / loopClosureWorkerLoop() / consumeLoopClosureMatchIfReady() / shutdownLoopClosure()` | `Tracker.cpp:3368/3396/3487/3641` |

### TrackKLT public
| Sig | Where |
|---|---|
| `TrackKLT()` | `TrackKLT.cpp:14` |
| `track(prev_g, curr_g, prev_pts, curr_pts, status, dR, K, win_size=-1)` | `TrackKLT.cpp:16-89` |
| `geometricVerification(prev, curr, status, K, R, t, inliers)` | `TrackKLT.cpp:91-129` |
| `predictPoints(prev_pts, predicted, dR, K)` | `TrackKLT.cpp:131-157` |

### FeatureManager public
| Sig | Where | Purpose |
|---|---|---|
| `FeatureManager()` | `FeatureManager.cpp:16-18` | reserve KFs |
| `detectGridFeatures(gray,out,max,qual,dist)` | `FeatureManager.cpp:53-99` | 4×5 grid + sub-pixel |
| `replenishSparse(gray,exist,new,max,qual,dist)` | `FeatureManager.cpp:103-221` | low-light adaptive |
| `storeKeyframe(gray,pts,ts,id,heading,position)` | `FeatureManager.cpp:225-241` | full image clone |
| `matchAgainstKeyframe(gray,cur,kf_out,cur_out)` | `FeatureManager.cpp:313-348` | ≥ 20 matches required |
| `getLastKeyframeInfo(...)` | inline `FeatureManager.h:56-61` | scalar |
| `storeKeyframeDescriptors(kf_id,ts,gray,corners,ids)` | `FeatureManager.cpp:518-587` | ORB + KLT id inheritance |
| `getKeyframeDescriptors() const` | inline `FeatureManager.h:86-88` | read deque |
| `setLatestKeyframePose(R,t)` | inline `FeatureManager.h:95-103` | populate pose |
| `reset()` | `FeatureManager.cpp:20-37` | clear |
| `assignIds(count)` | `FeatureManager.cpp:245-251` | monotonic |
| `addObservation(fid,clone_id,uv)` | `FeatureManager.cpp:253-261` | append |
| `extractLostFeatures(cur_ids, min_obs=3)` | `FeatureManager.cpp:263-288` | move-out |
| `getMSCKFCandidates(cur_ids, min_obs=4)` | inline `FeatureManager.h:148-151` | alias |
| `pruneObservations(min_clone_id)` | `FeatureManager.cpp:290-309` | drop marginalised |
| `noteObservation(fid, ts, is_kf)` | `FeatureManager.cpp:354-365` | bump age + last_obs_ns |
| `noteKeyframe(fid)` | `FeatureManager.cpp:367-373` | bump kf_count |
| `noteTriangulation(fid, p, anchor)` | `FeatureManager.cpp:375-387` | seed last_p_global |
| `setSlamSlot(fid, slot)` | `FeatureManager.cpp:389-398` | promote/demote |
| `getPromotableFeatures(min_obs=12,min_kf=2,max_rms=1.5) const` | `FeatureManager.cpp:400-425` | promotion gate |
| `markSlamFeatureRMS(fid, rms)` | `FeatureManager.cpp:427-438` | rms_bad_consecutive |
| `getDemoteCandidates() const` | `FeatureManager.cpp:440-448` | rms_bad_consecutive >= 3 |
| `getLostSlamFeatures(now, threshold=1e9) const` | `FeatureManager.cpp:450-461` | inactivity expiry |
| `getLifecycle(fid) const` | `FeatureManager.cpp:471-476` | **CAMERA THREAD ONLY** ptr |
| `getObservations(fid) const` | `FeatureManager.cpp:479-484` | **CAMERA THREAD ONLY** ptr |
| `dropLifecycle(fid)` | `FeatureManager.cpp:486-490` | erase |
| `getAllLifecycleFeatureIds() const` | `FeatureManager.cpp:495-501` | thread-safe by-value |
| `getLifecycleSize() const` | inline `FeatureManager.h:251` | telemetry |
| `getLandmarkSnapshot(whitelist, fx, fy, cx, cy, min_obs=2) const` | `FeatureManager.cpp:657-708` | thread-safe BA snapshot, returns PIXELS |
| `pruneStaleLifecycle(active_ids, now_ns)` | `FeatureManager.cpp:589-631` | per-frame GC |

---

## 10. Member Variables — Tracker

(Selected — full list in `Tracker.h:189-614`.)

| Variable | Type | Initial | Reader/Writer |
|---|---|---|---|
| `mutex_, pose_mutex_` | mutable mutex | — | state + scalar getters |
| `initializer_` | `InertialInitializer` | — | `addImuData` writes |
| `initialized_` | bool | false | first-frame writes |
| `prev_gray_` | `cv::Mat` | empty | KLT prev |
| `prev_pts_` | `vector<Point2f>` | empty | KLT prev |
| `prev_timestamp_ns_` | int64_t | 0 | dt source |
| `gray_buf_` | `cv::Mat` | reused | YUV target |
| `current_prev_pts_buf_, next_pts_buf_, prev_good_buf_, next_good_buf_, new_pts_buf_` | `vector<Point2f>` | reserved `MAX_FEATURES` | per-frame |
| `global_R_` | 3×3 CV_64F | I | LEGACY mirror (`Tracker.h:207-222`) |
| `global_t_` | 3×1 CV_64F | 0 | Tracker-owned position output |
| `fx_, fy_, cx_, cy_` | double | 0 | from `setIntrinsics` |
| `rolling_shutter_row_skew_ns_` | int64_t | 0 | Step 8c |
| `scale_obs_count_` | int | 0 | bootstrap |
| `user_scale_correction_` | double | 1.0 | clamp [0.1, 5.0] |
| `scale_bootstrap_buf_` | `vector<double>` | empty | first 15 obs |
| `accel_bias_, accel_bias_count_` | mat, int | 0, 0 | diagnostic |
| `points_3d_current_` | `vector<Point3f>` | per-frame | triangulated |
| `feature_ages_` | `vector<int>` | parallel to prev_pts_ | KLT survival counter |
| `frame_counter_` | int | 0 | monotonic |
| `heading_initialized_` | bool | false | mag one-shot |
| `scalar_heading_` | double | 0 | rad CW from North; mirror of Madgwick |
| `total_path_m_` | double | 0 | cumulative metres, drives LC dynamic σ |
| `pending_init_heading_, pending_init_heading_set_` | double, bool | 0, false | applied first frame |
| `filtered_yaw_rate_` | double | 0 | LP filter |
| `last_visual_yaw_variance_` | double | -1 | rad² |
| `last_depth_scale_variance_` | double | -1 | MiDaS |
| `scale_fuser_` | `ScaleFuser{0.20, 1.0}` declared, reset to `(0.10, 4.0)` | m/VIO unit, var | 1-D KF |
| `last_scale_predict_ns_` | int64_t | 0 | predict bookkeeping |
| `scale_estimator_vi_` | `ScaleEstimatorVI` | — | Hesch/Martinelli |
| `observer_c_pair_count_` | int | 0 | solve every 10 |
| `td_warmup_buf_, td_warmup_done_` | vector, bool | empty, false | Step 8a |
| `last_step_speed_, last_step_speed_ns_` | double, int64_t | 0, 0 | PDR interpolation |
| `clahe_` | `cv::Ptr<cv::CLAHE>` | 2.0 / 8×8 | adaptive contrast |
| `ekf_, feature_mgr_, lens_, klt_, zupt_detector_, msckf_updater_` | owned modules | — | core pipeline |
| `feature_ids_` | `vector<int>` | parallel to prev_pts_ | MSCKF id tracking |
| `frames_since_keyframe_` | int | 0 | KF trigger |
| `blur_skipped_streak_` | int | 0 | log cadence |
| `low_inlier_streak_` | int | 0 | reloc trigger |
| `reloc_orb_` | `cv::Ptr<cv::ORB>` | null | lazy |
| `ba_thread_, ba_in_flight_, ba_result_pending_, ba_result_landmarks_, ba_round_counter_, ba_result_mutex_` | thread+flags | — | BA worker plumbing |
| `loop_closure_, loop_closure_thread_, ..., loop_closure_active_match_` | LC plumbing | — | DBoW2 worker |
| `depth_mutex_, depth_map_, depth_width_, depth_height_` | mutex+buffer | — | MiDaS staging |

### Tracker constants (`Tracker.h:550-614, 232, 290, 296, 384, 505-538`)

`MAX_FEATURES=200, MIN_FEATURES=80, QUALITY_LEVEL=0.05, MIN_DIST=10.0, RANSAC_CONF=0.9999, RANSAC_THRESH=1.5, MIN_PARALLAX_PX=0.8, FB_CHECK_THRESH=4.0, MIN_FLOW_PX=0.4, MAX_FLOW_PX=150.0, MIN_INLIERS=8, MIN_INLIER_RATIO=0.25, GYRO_ROT_ONLY_THRESH=2.0, FLOW_RAYLEIGH_REJECT=0.3, BLUR_VAR_THRESH=80.0, ZUPT_GYRO_THRESH=0.04, ACCEL_BIAS_WARMUP=150, ACCEL_BIAS_ALPHA=0.005, RELOC_TRIGGER_FRAMES=3, RELOC_LOW_INLIER_BAR=MIN_INLIERS/2=4, RELOC_RECENT_KFS=5, RELOC_LOWE_RATIO=0.75f, RELOC_MIN_INLIERS=30, RELOC_ID_REATTACH_RADIUS=3.0f, SCALE_BOOTSTRAP_COUNT=15, OBSERVER_C_SOLVE_INTERVAL=10, TD_WARMUP_FRAMES=60, BA_MAX_SOLVE_US=200000, LOOP_CLOSURE_QUERY_PERIOD_S=1.0, LOOP_CLOSURE_TEMPORAL_EXCL_NS=30e9, LOOP_CLOSURE_DAMPING_FRAMES=10, LOOP_CLOSURE_PNP_SIGMA_FLOOR_M=2.0, LOOP_CLOSURE_DRIFT_RATE=0.032, LOOP_CLOSURE_BASE_ROT_SIGMA_RAD=0.34907 (=20°)`.

### TrackKLT constants (`TrackKLT.h:64-70`)

`PYRAMID_LEVELS=4, WINDOW_SIZE=21, RANSAC_CONF=0.999 (note mismatch w/ Tracker 0.9999), RANSAC_THRESH=1.5, MIN_INLIERS=8, FB_CHECK_THRESH=4.0`.

### FeatureManager constants

`ORB_TARGET_FEATURES=500, ORB_FAST_THRESHOLD=10, ORB_PREBLUR_SIGMA=1.0, GRID_ROWS=4, GRID_COLS=5, MAX_KEYFRAMES=10, MIN_KF_MATCHES=20, KF_MATCH_RADIUS=15.0f, BRIGHTNESS_LOW_THRESH=0.12, REPLENISH_TARGET_LOWLIGHT=120, REPLENISH_QUALITY_LOWLIGHT=0.10, BRIGHTNESS_CENTRE_FRAC=0.5, BRIGHTNESS_HYSTERESIS_FRAMES=5, ORB_KLT_MATCH_RADIUS=3.0f, KEYFRAME_DESC_RING_SIZE=50, LIFECYCLE_KEEP_NS=100 ms`.

### FeatureManager private state (`FeatureManager.h:323-394`)

`keyframes_ (vector, reserved 10), keyframe_descriptors_ (deque, empty), orb_extractor_ (cv::Ptr, null lazy), next_feature_id_ (int, 0), active_tracks_ (map, empty), lifecycle_ (map, empty), snapshot_mutex_ (mutex), low_light_low_streak_/normal_streak_ (int, 0), low_light_state_ (bool, false), low_light_log_counter_ (int, 0)`.

---

## 11. Interactions

### Tracker → consumers (VioEngine / JNI)

`VisionOutput` fields:
- `R` — EKF rotation when full-init else `global_R_`
- `t` — `global_t_` (Tracker-owned, **NOT EKF p_G**)
- `rawR, rawT` — recoverPose outputs (per-frame)
- `quality` — track-survival × maturity
- `trackedCount, totalCount, estimatedScale, valid, trackedPoints (flat), meanFlow, inlierCount, stepCount, stepFreq, strideLength`
- `poseFlags` — bit0 is_static, bit1 is_pure_rotation, bit2 pose_valid, bit3 used_fallback
- `heading` — `scalar_heading_` (Madgwick, **NOT EKF yaw**)

**Critical architectural choices** (`Tracker.cpp:2669-2690, 1436-1453, 2711-2718`):
- Rotation output: from EKF when full-init.
- Position output: from `global_t_` (Tracker), NOT EKF — EKF position drifts during standstill before ZUPT fires (residual accel-bias × Δt² + Madgwick tilt-bleed). EKF is still maintained for covariance (`getPositionCovarianceXZ`) but isn't the display source.
- Heading output: from `imu.getHeading()` (Madgwick), NOT `ekf_.getYaw()`. V-shape bug fix 2026-05-03 — EKF yaw under-rotated 180° turns by ~30° because EKF gyro bias diverged from Madgwick's.

`TrackerFrame frame_out` (mutated reference) carries cloned `gray, prev_good, next_good, points_3d, global_position, intrinsics` for downstream Mapper/visualisation.

### Tracker → EKFState writes (per-frame, in order)
1. `propagateIMU` (`Tracker.cpp:800`)
2. `updateZUPT` if static (`L:1061`)
3. `updateRelativeRotation(R_vo_body, var, prev_clone_id)` (`L:1223`)
4. `updateRelativePose(t_world_metric, prev_clone_id, var_t)` (`L:1552`)
5. `updatePDRStep(dx, dy, σ²)` PDR fallback (`L:1610`)
6. `addClone(R = R_bc * R_GtoI, p, ts)` every frame (`L:1638`)
7. `addSlamFeature(fid, p_world, anchor)` per promotion (`L:1923`)
8. `updateSlamFeature(slot, obs, ids, σ²)` (`L:1969`)
9. `removeSlamFeature(slot)` demote/expire (`L:2045`)
10. `updateGravityAlignedYaw(yaw_meas, var, roll, pitch)` (`L:2294`)
11. `updateAbsolutePose(R, p, var_R, var_p)` loop closure (`L:3610`)
12. One-shots: `setExtrinsicsRotation(R_bc)` (`L:92`), `setSlamIntrinsics` (`L:54, 1803`), `setTimeOffset(td)` (`L:1041`).

### Tracker → EKFState reads
`getRotation, getPosition, getCovariance, getTimeOffset, getExtrinsicsRotation, getLatestCloneId, getClonePose, getCloneFEJ, getCloneSnapshot, getWindow, getSlamFeatureCount, getSlamFeatureSlot, getSlamFeatureGlobalPosition, isFullInitialized, getMSCKFHuberRejectedCount, getStateDim`.

### Tracker ← IMUPreintegrator
**Reads**: `integrate(t_start_ns, t_end_ns)` returning `PreintegratedMeasurement{deltaR, deltaV, deltaP, dt, cov, J_R_bg, J_V_bg, J_V_ba, J_P_bg, J_P_ba}`; `getStepInfo, getHeading, getMadgwickRoll/Pitch, hasMagHeading, getMagHeading, isOrientationInitialized, getGyroBias, getAccelBuffer, getGyroBuffer, lastAccel{X,Y,Z}, getUserHeight, getVehicleSpeedEstimate`.

**Writes**: `imu.refineGyroBiasDuringZUPT()` (`Tracker.cpp:1063`).

### Per-frame data flow (~30 Hz)

```
camera frame (NV21, ts) ─┐
  ▼
cvtColor → CLAHE → measureBlur ──→ EKF.propagateIMU(integrate(prev_ts+td, ts+td))
                                        │
                                ZUPT detect → updateZUPT
                                        │
        KLT.track(prev_gray, gray, prev_pts; deltaR, win_sz)
                                        │
        FB-check, boundary filter → next_good_buf_, status
                                        │
        mean_flow, Rayleigh R/N → is_pure_rotation
                                        │
        if (sufficient_motion && parallax && !blurry)
            lens_.undistortMatchedPoints
            klt_.geometricVerification → R_vo, t_vo, inliers
            triangulate → points_3d_current_ (chi² 5.991 gate)
            ekf_.updateRelativeRotation(R_vo_body)
            Step scale obs / Observer C VI scale / MiDaS depth
                                        │
        ORB reloc trigger if low_inlier_streak_ >= 3
                                        │
        ekf_.updateRelativePose(t_world_metric)  OR  updatePDRStep
                                        │
        ekf_.addClone (with R = R_bc * R_GtoI)
        feature_mgr_.addObservation + noteObservation per surviving fid
                                        │
        replenishSparse if tracked < 80
        keyframe re-localize if tracked < 40
                                        │
        MSCKF: getMSCKFCandidates → processLostFeatures → pruneObservations
                                        │
        SLAM lifecycle: prune → promote → update → demote/expire
                                        │
        Keyframe storage every 15 frames (or on track collapse):
            storeKeyframe + storeKeyframeDescriptors
            noteKeyframe(fid) for all
            KF heading correction → updateGravityAlignedYaw
            loop_closure_.addKeyframe + publish to LC worker
            consumeBAResultIfReady → kickOffBARound
                                        │
        consumeLoopClosureMatchIfReady → ekf_.updateAbsolutePose (10-frame damped)
                                        │
        VisionOutput { R=ekf, t=global_t_ (Tracker-owned), heading=Madgwick, ... }
```

---

## 12. Magic Numbers / TODOs / Code Smells Flagged

- `Tracker.cpp:1490` `max_disp = 2.0 * max(dt, 0.03)` — 2 m/s cap, 30 ms floor — inline.
- `Tracker.cpp:1545-1551` visual position variance `5% disp + √scale_var·|t_vo| + 1 cm floor` — inline.
- `Tracker.cpp:1609` PDR sigma `0.05 + 0.10*d` — inline.
- `Tracker.cpp:2287` visual yaw variance floor `1e-4` (~0.6°²) — inline.
- `Tracker.cpp:2467` `min(kf_ring.size(), 10)` look-back limit — inline.
- `Tracker.cpp:2479` ORB triangulation baseline gates `0.1 ≤ bn ≤ 5.0 m` — inline.
- `Tracker.cpp:2567-2570` ORB depth `0.5 ≤ z ≤ 50 m` — inline.
- `Tracker.cpp:223-224` MiDaS `0.3 ≤ z_metric ≤ 12 m` — inline.
- `Tracker.cpp:242` MiDaS secondary `0.3 ≤ z ≤ 10 m` — inline.
- `Tracker.cpp:289` MiDaS scale safety `target ≤ 3× current` and `target ≥ current/3` — inline.
- `Tracker.cpp:1153` chi²(2-DOF, 95%) = 5.991 — well-known.
- `Tracker.cpp:1196-1198` R_vo fusion gate `inliers >= 12` — inline.
- `Tracker.cpp:1257` bootstrap variance `((max-min)/2)²` floored `1e-4` — inline.
- `Tracker.cpp:1306` 10% VO baseline uncertainty in quadrature — inline.
- `Tracker.cpp:1364` Observer C variance floor `0.04` — inline.
- `Tracker.cpp:1820-1830` comment: `min_obs` lowered 12→8 on 2026-05-04.
- `FeatureManager.cpp:341` KF re-localization match radius scaled by 10 (`KF_MATCH_RADIUS * 10` = 150 px).
- `FeatureManager.cpp:408-420` explicit removal of `has_p_global` requirement (chicken-and-egg).
- `FeatureManager.cpp:433` SLAM bad-RMS threshold `3.0 px` — not a named constant.
- **STALE COMMENT**: `Tracker.h:35` says "UpdaterMSCKF member never exercised after Step 4; its update site at Tracker.cpp:1193 is wrapped in DISABLED" — but in current code MSCKF is **re-enabled** at `Tracker.cpp:1729-1742` (Plan Step 3a / ADR-008). The header comment block needs updating.
- **CONSTANT MISMATCH**: `Tracker::RANSAC_CONF=0.9999` vs `TrackKLT::RANSAC_CONF=0.999`. `klt_.geometricVerification` uses 0.999; the inline `findEssentialMat` for KF heading correction (`Tracker.cpp:2141`) and reloc (`Tracker.cpp:2847`) use 0.9999.
- **HARDCODED RATIO**: depth-scale uses `camera_h = user_h * 0.85` (`Tracker.cpp:146`) — phone-held-at-chest assumption.
- **API NAME DRIFT**: `getPositionCovarianceXZ` returns the X-Y horizontal plane post-Z-up alignment; name retained for ABI/JNI stability (`Tracker.cpp:566-568`).
- **DEAD CODE LEFT**: `global_R_` and `global_t_` mirror EKF state per Step 8 cleanup plan but are still load-bearing for bootstrap + position output (`Tracker.h:207-222`). Migration not complete.
- **DEAD CODE comments**: `Tracker.cpp:528-530` references deleted `blendScale/applyLoopCorrection`. `FeatureManager.h:320-321` notes deleted `getNextFeatureId`.
- **CONVENTION**: `KeyframeDescriptors::timestamp_ns` is `double`, not `int64_t` (`KeyframeDescriptors.h:23`) — Agent C contract artifact.
- **SCOPE**: `addImuData` feeds `initializer_`, NOT `imu` — `IMUPreintegrator` is fed elsewhere (out of scope).
