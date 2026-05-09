# 05 — VioEngine Orchestrator + Camera Calibration + Lens Correction + JNI Bridge

> Files studied:
> - `app/src/main/cpp/VioEngine.h` (107 lines)
> - `app/src/main/cpp/VioEngine.cpp` (281 lines)
> - `app/src/main/cpp/CameraCalibration.h` (47 lines)
> - `app/src/main/cpp/CameraCalibration.cpp` (255 lines)
> - `app/src/main/cpp/LensCorrector.h` (49 lines)
> - `app/src/main/cpp/LensCorrector.cpp` (81 lines)
> - `app/src/main/cpp/VioTypes.h` (28 lines)
> - `app/src/main/cpp/native-lib.cpp` (883 lines)
> - `app/src/main/cpp/EventCounters.h` (400 lines)
> - `app/src/main/java/com/example/navsight1/NativeBridge.kt` (103 lines)
> - `app/src/main/java/com/example/navsight1/VioData.kt` (124 lines)
> - `app/src/main/cpp/Tracker.h` (referenced for ownership and constants)

---

## 1. VioEngine class — what owns what

### 1.1 Composition

`VioEngine` is intentionally **thin**. It owns exactly two heavyweight objects and delegates everything else; all algorithmic state lives one level deeper. The Mapper / LoopClosureDetector / PoseGraph subsystem that older revisions ran on a background thread is **disabled** (the code is preserved as comments — see `VioEngine.h:7-9`, `VioEngine.h:81-106`, `VioEngine.cpp:35-70`). The class banner makes it explicit: *"Mapper pipeline DISABLED — was running but output discarded"* (`VioEngine.h:14`).

Direct members (see `VioEngine.h:84-106`):

- `IMUPreintegrator imu_;` — owns Madgwick attitude filter, gyro/accel ring, biases, time-offset, magnetometer one-shot init, user height.
- `Tracker tracker_;` — owns everything visual: `EKFState ekf_`, `FeatureManager`, `LensCorrector lens_`, `TrackKLT klt_`, `UpdaterZeroVelocity zupt_detector_`, `UpdaterMSCKF msckf_updater_`, `InertialInitializer initializer_`, `LoopClosureDetector loop_closure_`, `ScaleFuser scale_fuser_`, `ScaleEstimatorVI scale_estimator_vi_`, `cv::Ptr<cv::CLAHE> clahe_`, `cv::Ptr<cv::ORB> reloc_orb_`, the BA worker thread (`ba_thread_`), and reusable per-frame buffers (`Tracker.h:189-345`).

Disabled members (preserved as comments, never instantiated): `Mapper mapper_`, `std::thread mapper_thread_`, `std::mutex mapper_mutex_`, `std::condition_variable mapper_cv_`, `std::atomic<bool> mapper_stop_`, the `mapper_pending_*` work-queue fields, `latest_depth_map_`, `latest_mapper_result_` pickup (`VioEngine.h:88-106`).

Ownership graph rooted at `VioEngine`:

```
VioEngine
├── IMUPreintegrator imu_                         (per-IMU-sample, Madgwick, biases)
└── Tracker tracker_                              (per-frame, owns the EKF and updaters)
    ├── EKFState ekf_                             (15-DOF + clones + SLAM landmarks + R_bc, td)
    ├── FeatureManager feature_mgr_               (KLT IDs, MSCKF candidates, KF descriptor ring)
    ├── LensCorrector lens_                       (camera_matrix_, dist_coeffs_)
    ├── TrackKLT klt_                             (pyramidal Lucas-Kanade)
    ├── UpdaterZeroVelocity zupt_detector_
    ├── UpdaterMSCKF msckf_updater_
    ├── InertialInitializer initializer_          (Step 5 stationary/motion gate)
    ├── LoopClosureDetector loop_closure_         (DBoW2; Step 7 / ADR-013)
    ├── ScaleFuser scale_fuser_                   (1-D KF over scale observers)
    ├── ScaleEstimatorVI scale_estimator_vi_      (closed-form Hesch/Martinelli)
    ├── cv::Ptr<cv::CLAHE> clahe_
    ├── cv::Ptr<cv::ORB>   reloc_orb_             (Step 4 ORB-relocalize, lazy)
    └── std::thread ba_thread_                    (Step 6 windowed BA worker, detached)
```

### 1.2 State-owning members of `VioEngine` itself

From `VioEngine.h:80-107`:

| Member | Type | Status |
|--------|------|--------|
| `imu_` | `IMUPreintegrator` | live |
| `tracker_` | `Tracker` | live |
| `mapper_` | `Mapper` | **disabled** (commented) |
| `mapper_thread_` | `std::thread` | **disabled** |
| `mapper_mutex_` | `std::mutex` | **disabled** |
| `mapper_cv_` | `std::condition_variable` | **disabled** |
| `mapper_stop_` | `std::atomic<bool>` | **disabled** |
| `mapper_has_work_` | `bool` | **disabled** |
| `mapper_pending_frame_` | `TrackerFrame` | **disabled** |
| `mapper_pending_scale_` | `double` | **disabled** |
| `depth_mutex_` | `std::mutex` | **disabled** |
| `latest_depth_map_` | `std::vector<float>` | **disabled** |
| `depth_width_`, `depth_height_` | `int` | **disabled** |
| `result_mutex_` | `std::mutex` | **disabled** |
| `latest_mapper_result_` | `MapperResult` | **disabled** |
| `has_mapper_result_` | `bool` | **disabled** |

`VioEngine` itself holds **no mutex**. All concurrency lives one level down.

### 1.3 Engine lifecycle

Construction (`VioEngine.cpp:16-20`): default-construct `imu_` and `tracker_`, log creation. No separate `init()` — RAII ctors handle initialization.

Start path (driven from JNI `Java_com_example_navsight1_NativeBridge_startVIO`, `native-lib.cpp:104-114`):
1. JNI takes `state_mutex` and resets `g_vision`.
2. `g_vision = std::make_shared<VioEngine>();`
3. JNI calls `g_vision->setUserScaleCorrection(g_user_scale_correction)` (`native-lib.cpp:111`).
4. `resetPoseState()` zeros all global pose accumulators (`native-lib.cpp:51-60`).
5. Kotlin then separately calls `setIntrinsics`, `nativeLoadCalibration`, `nativeLoadLoopClosureVocabulary`, `nativeSetExtrinsicsRotation`, `setUserHeight`, `setInitialHeading`, optionally `loadStoredCalibration`.

IMU intake (`VioEngine::addGyroData`/`addAccelData`, `VioEngine.cpp:106-126`):
1. NaN/Inf guard — drop on bad sample.
2. Forward to `imu_.addGyroReading` / `imu_.addAccelReading`.
3. **Also** forward to `tracker_.addImuData(ts, ax, ay, az, gx, gy, gz)` for the InertialInitializer's stationary/motion gate. The Tracker call uses the **other** sensor's last cached reading.

Frame intake (`VioEngine::processFrame`, `VioEngine.cpp:75-102`):
1. Allocate `TrackerFrame frame` on the stack (`VioEngine.cpp:77`).
2. Call `tracker_.processFrame(yuv_data, w, h, ts, imu_, frame)` — the **only** call doing work (`VioEngine.cpp:78-79`).
3. Mapper-result pickup and Mapper job submission commented out.
4. Return `out` to the JNI shim.

Reset (`VioEngine::reset`, `VioEngine.cpp:195-210`): Calls `tracker_.reset()`, `imu_.reset()`, logs `"VioEngine reset"`.

Destruction (`~VioEngine`, `VioEngine.cpp:22-33`): Default RAII destruction of `tracker_` and `imu_`.

---

## 2. Frame-rate loop — exact order of operations per frame

`VioEngine::processFrame` itself does almost nothing. The real per-frame algorithm is `Tracker::processFrame`, invoked synchronously on the camera thread. Canonical per-frame ordering:

1. **Frame intake & blur gate** — Convert YUV → grayscale into `gray_buf_`. Run `measureBlur(gray_buf_)`; if variance-of-Laplacian < `BLUR_VAR_THRESH = 80.0`, the EKF prediction (propagateIMU + ZUPT) still runs but visual updates, MSCKF processLostFeatures, and ORB relocalization trigger are **suppressed** for that frame.
2. **IMU pre-integration** — Tracker pulls IMU samples between `prev_timestamp_ns_` and `ts` and accumulates `delta_R`, `delta_v`, `delta_p`, plus their bias Jacobians.
3. **EKF propagation** — `ekf_.propagate(...)` consumes the preintegration block, advances IMU error-state covariance over `dt`, applies bias terms. Time-offset δt_d propagated as a random walk.
4. **ZUPT** — `zupt_detector_` consumes a windowed accel/gyro stationarity check (`ZUPT_GYRO_THRESH = 0.04 rad/s`).
5. **Visual tracking** — KLT/RANSAC/pose path: KLT optical flow, FB-check, undistortion, `findEssentialMat`, `recoverPose`, triangulate, ORB reloc trigger, Rayleigh dual-gate.
6. **EKF measurement update** — visual yaw delta + 2D feature observations via `msckf_updater_`. Huber rejections increment `msckf_huber_rejected_sum`.
7. **Scale fusion** — three observers feed `scale_fuser_`: PDR, MiDaS depth, VI closed-form (every `OBSERVER_C_SOLVE_INTERVAL = 10` keyframe pairs).
8. **Bundle adjustment kickoff** (Step 6) — every keyframe re-seeds previous BA refined landmarks, then `kickOffBARound` snapshots most-recent 5 EKF clones and launches a **detached** worker bounded by `BA_MAX_SOLVE_US = 200_000` µs.
9. **Loop closure** — separate **1 Hz worker thread** owned by Tracker.
10. **Heading mirror & path accounting** — `scalar_heading_` refreshed from `ekf_.getYaw()`, `total_path_m_` incremented.
11. **Publish** — Tracker fills `VisionOutput`.
12. **JNI marshal** — `Java_..._processCameraFrameDirect` swaps Z-up → Y-up axes for `g_x/g_y/g_z`, runs `cv::Rodrigues` on `rawR`, recovers Euler from fused R, constructs Kotlin `VioData` via cached ctor.

---

## 3. IMU-rate loop — exact order per IMU sample

Driven from JNI `Java_..._processGyroscope` and `Java_..._processAccelerometer`.

Per gyro sample (`VioEngine::addGyroData`):
1. NaN/Inf guard.
2. `imu_.addGyroReading(ts, x, y, z)` — bias-correct, push to ring, advance Madgwick.
3. Forward to `tracker_.addImuData(ts, lastAccelX, lastAccelY, lastAccelZ, x, y, z)`.

Per accel sample (`VioEngine::addAccelData`):
1. NaN/Inf guard.
2. `imu_.addAccelReading(ts, x, y, z)` — bias-correct, push to ring; Madgwick uses gravity vector when accel arrives.
3. Forward to `tracker_.addImuData(ts, x, y, z, lastGyroX, lastGyroY, lastGyroZ)`.

Per-IMU-sample net effect: Madgwick attitude updated; sample written to preintegration ring (reset/restarted at every `processFrame` boundary); latest sample stored as `last_ax/ay/az/gx/gy/gz`; InertialInitializer drains samples for stationary window.

JNI thread model: gyro/accel come from Android `SensorManager` event thread; camera frames from CameraX executor. The two paths share `state_mutex` only when reading/writing the `g_vision` shared_ptr.

---

## 4. VioTypes.h — every struct/enum/typedef

One struct only: `VisionOutput` (`VioTypes.h:7-28`). No enums. No typedefs.

| Field | Type | Units / frame | Meaning |
|-------|------|---------------|---------|
| `R` | `cv::Mat` (3×3, `CV_64F`) | rotation matrix, **fused** (EKF gravity-aligned) | Used by JNI to derive roll/pitch/yaw |
| `t` | `cv::Mat` (3×1, `CV_64F`) | meters, internal **Z-up ENU** | World position |
| `rawR` | `cv::Mat` (3×3, `CV_64F`) | **raw camera** rotation, OpenCV camera frame | For sim recording |
| `rawT` | `cv::Mat` (3×1, `CV_64F`) | OpenCV camera frame; **no axis swap** at JNI | Raw VO output |
| `quality` | `double` | dimensionless [0..1] | Tracking quality heuristic |
| `trackedCount` | `int` | count | KLT survivors after FB check |
| `totalCount` | `int` | count | Total slots considered |
| `estimatedScale` | `double` | m per unit-norm visual translation | From `scale_fuser_` |
| `valid` | `bool` | flag | true if pose update consumed |
| `trackedPoints` | `std::vector<float>` | px, interleaved `[x0,y0,x1,y1,…]` | UI overlay |
| `meanFlow` | `double` | px | Mean optical flow magnitude across inliers |
| `inlierCount` | `int` | count | RANSAC inliers from `findEssentialMat` |
| `stepCount` | `int` | count | Cumulative PDR steps |
| `stepFreq` | `double` | Hz | Step cadence |
| `strideLength` | `double` | m | Estimated stride |
| `poseFlags` | `int` | bitmask: bit0=`static`, bit1=`pureRot`, bit2=`poseValid`, bit3=`fallback` | |
| `heading` | `double` | rad, gravity-aligned EKF yaw, CW-positive, North=0 | Yaw scalar |
| `td_imu_cam` | `double` | s | Online IMU↔camera time offset (Step 8a, EKF row 15) |

---

## 5. CameraCalibration

### 5.1 JSON Schema (`CameraCalibration.h:8-18`)

```json
{
  "image_size": [w, h],
  "fx": ..., "fy": ..., "cx": ..., "cy": ...,
  "dist": {"k1":..., "k2":..., "p1":..., "p2":..., "k3":...,
           "k4":0, "k5":0, "k6":0},
  "rms_reprojection_error_px": ...,
  "image_count": N,
  "checkerboard": {"cols":9, "rows":6, "square_mm":25.0},
  "captured_at": "<ISO 8601>",
  "source": "in_app" | "offline_script"
}
```

Runtime app does not link `nlohmann/json`; only test/replay binaries do. Runtime parser is a hand-rolled string scan.

### 5.2 Struct `CameraIntrinsics` (`CameraCalibration.h:26-42`)

| Field | Type | Default | Units |
|-------|------|---------|-------|
| `fx, fy` | `double` | 0.0 | px (focal lengths) |
| `cx, cy` | `double` | 0.0 | px (principal point) |
| `k1, k2, k3, k4, k5, k6` | `double` | 0.0 | dimensionless (rational radial) |
| `p1, p2` | `double` | 0.0 | dimensionless (tangential) |
| `image_w, image_h` | `int` | 0 | px |
| `rms_px` | `double` | 0.0 | px (RMS reprojection) |

### 5.3 Single public function

`bool loadCalibrationFromJson(const std::string& path, CameraIntrinsics& out)` (`CameraCalibration.h:46`, defined `CameraCalibration.cpp:161-255`). Returns true on success; on false, `out` is left untouched.

**Validation gates** (loader returns false on any fail):
- File exists, non-empty.
- `image_size` parses; both dims > 0.
- `fx, fy > 0`.
- `cx ∈ [0, w]`, `cy ∈ [0, h]`.
- All five mandatory distortion children (`k1, k2, p1, p2, k3`) present.
- `k4, k5, k6` are **optional** — silently default to 0.0 when absent.
- `rms_reprojection_error_px ∈ [0.0, 1.0]` px. Above 1.0 px the loader rejects.

### 5.4 How `R_bc` is stored and applied

`R_bc` (body→camera rotation) is **not** handled by `CameraCalibration`. Path:
- Kotlin reads `CameraCharacteristics.SENSOR_ORIENTATION`, builds 3×3 row-major `float[9]`, calls `NativeBridge.nativeSetExtrinsicsRotation(R_bc_flat)`.
- `Java_..._nativeSetExtrinsicsRotation` validates length == 9, calls `vision->setExtrinsicsRotation(...)`.
- `VioEngine::setExtrinsicsRotation` forwards to `tracker_.setExtrinsicsRotation`.
- Tracker stores into `EKFState`'s `R_bc_`. **Default initial value is `diag(1, -1, -1)`**.
- During each MSCKF update the EKF refines `R_bc_` online (Step 8b); residual angle from identity is logged as `extrinsics_rotation_angle_mdeg` (milli-degrees) every update.

### 5.5 Default values vs user calibration

There are no hard-coded fx/fy defaults; struct defaults are 0.0 and the loader rejects zeros. Override order:
1. App startup, no calibration: `tracker_.fx_ = fy_ = cx_ = cy_ = 0.0`; `LensCorrector` falls back to passthrough.
2. Kotlin `setIntrinsics(fx, fy, cx, cy)` → `Java_..._setIntrinsics` → `tracker_.setIntrinsics`. Default path with device-profile values.
3. Kotlin `nativeLoadCalibration(absPath)` → `Java_..._nativeLoadCalibration` → `vision->loadCalibration(path)` → loads + pushes both `setIntrinsics` and `setDistortion`.

**Per `project_vio_rotation_bugs.md` memory**, intrinsics scaling was a known bug. The current loader stores source `image_w/h` but `VioEngine::loadCalibration` does **not** rescale before pushing — calibration JSON must be captured at runtime resolution. The tight `rms_px ≤ 1.0 px` gate fails any mismatched-resolution calibration.

---

## 6. LensCorrector

### 6.1 Class shape (`LensCorrector.h:13-49`)

| Member | Type | Default | Purpose |
|--------|------|---------|---------|
| `camera_matrix_` | `cv::Mat` (3×3 `CV_64F`) | identity | OpenCV K matrix |
| `dist_coeffs_` | `cv::Mat` (8×1 `CV_64F`) | zeros | Rational `[k1,k2,p1,p2,k3,k4,k5,k6]` |
| `has_intrinsics_` | `bool` | `false` | True when `setIntrinsics` sees fx>0 && fy>0 |
| `has_distortion_` | `bool` | `false` | True when `setDistortion` sees any non-zero coefficient |
| `DEFAULT_K1`, `DEFAULT_K2` | `static constexpr double = 0.0` | — | Ctor seed |

### 6.2 Public API

- `LensCorrector()` — `K = I`, dist = zeros.
- `void setIntrinsics(double fx, double fy, double cx, double cy)` — replaces `camera_matrix_` with `[[fx,0,cx],[0,fy,cy],[0,0,1]]`; sets `has_intrinsics_ = (fx > 0 && fy > 0)`.
- `void setDistortion(double k1, k2, k3, k4, k5, k6, p1, p2)`:
  - **Guard 1** — if `camera_matrix_(0,0) <= 0 || camera_matrix_(1,1) <= 0`, refuse with `has_distortion_ = false`.
  - **Guard 2** — if all eight |coefficients| ≤ `kDistortionEpsilon = 1e-9`, refuse with `has_distortion_ = false` (passthrough).
  - Otherwise stores `[k1, k2, p1, p2, k3, k4, k5, k6]` (OpenCV's order) and sets `has_distortion_ = true`.
- `void undistortPoints(std::vector<cv::Point2f>& points) const` — no-op if not ready or empty; calls `cv::undistortPoints(points, ud, K, D, noArray(), K)` (passing `K` as new camera matrix → output in **pixel coords**, not normalized).
- `void undistortMatchedPoints(std::vector<cv::Point2f>& prev, std::vector<cv::Point2f>& next) const` — same but for matched pair; used pre-essential-matrix.
- `bool isReady() const` → `has_intrinsics_`.
- `bool hasDistortion() const` → `has_distortion_`.

### 6.3 Undistortion math

The class **does not roll its own**. Calls `cv::undistortPoints`, which inverts Brown-Conrady **rational** model. OpenCV solves the inverse with a Newton-style fixed-point iteration (default 5 iterations or convergence). **No precomputed lookup table** — every call goes through `cv::undistortPoints`. Per-frame cost ~tens of µs for typical 80–200 features.

**Not** equidistant/fisheye — no `cv::fisheye::*` use. Header explicitly distinguishes 8-coef rational from 5-coef polynomial.

---

## 7. EventCounters

Header-only, lock-free, process-singleton in namespace `navsight` (`EventCounters.h:73-396`). Singleton accessor: `inline EventCounters& navsight::eventCounters()`.

Design: all increments `fetch_add(1, std::memory_order_relaxed)`; all reads `load(std::memory_order_relaxed)`. **No mutex anywhere**. Reset is per-field `store(0, relaxed)`. Single CAS loop is `update_ba_solve_us_max`.

### 7.1 Counter inventory

All `std::atomic<long long>`, default 0.

| Counter | Increment site |
|---------|---------------|
| `reloc_orb_accepts/rejects/size_skipped/slam_guarded` | Tracker ORB reloc |
| `blur_enter_events`, `blur_total_skip_frames` | Blur gate |
| `lowlight_state_active_frames`, `lowlight_log_lines` | FeatureManager replenish |
| `rot_gate_pure_rot_confirmed`, `rot_gate_log_lines` | Rayleigh dual-gate |
| `klt_adaptive_window_hits` | Adaptive KLT window bump |
| `ba_solves_total/accepted/rejected`, `ba_skipped_in_flight`, `ba_solve_us_sum/max`, `ba_iters_sum`, `ba_skipped_no_init/too_few_clones/no_intrinsics/too_few_landmarks` | BA worker |
| `slam_promotions_total` | EKFState `addSlamFeature` |
| `msckf_update_lines`, `msckf_huber_rejected_sum` | MSCKF update |
| `loop_closure_attempts/accepts/rejects_low_score/rejects_pnp/rejects_heading/kf_count_in_db/chi2_rejected/corrections_applied` | Loop closure |
| `total_path_dm` | Tracker (path × 10) |
| `extrinsics_rotation_angle_mdeg` | Step 8b angle telemetry |
| `cam_imu_time_offset_us` | Step 8a |
| `rolling_shutter_skew_ns` | Step 8c |
| `midas_*` | MiDaS scale gating |

### 7.2 Reading for telemetry

Two read paths:
1. **`reset()`** — zero every field. Called from JNI `nativeResetEventCounters` at simulator-recording start.
2. **`serializeAsJsonString()`** — snapshots every field with `load(relaxed)` into local `long long`, then writes single-line `{"key":N,…}` JSON. Returned by JNI `nativeGetEventCountersJson`, embedded into `simulation_data_<ts>.json` under `event_summary`. Within a single field the read is atomic; across fields it is not snapshot-consistent.

---

## 8. JNI bridge (native-lib.cpp)

### 8.1 Module-level state (`native-lib.cpp:17-46`)

| Variable | Type | Purpose |
|----------|------|---------|
| `state_mutex` | `std::mutex` | Guards every access to `g_vision`, `g_*`, `g_user_scale_correction` |
| `g_vision` | `std::shared_ptr<VioEngine>` | Live engine; null when stopped |
| `g_viodata_cls` | `jclass` (GlobalRef) | Cached `com/example/navsight1/VioData` |
| `g_viodata_ctor` | `jmethodID` | Cached `<init>` with sig `"(DDDDDDDIIDZ[FDDDDFFFFFFDIIDDIDD)V"` |
| `g_x, g_y, g_z` | `double` | World position (Y-up exposed) |
| `g_roll, g_pitch, g_yaw` | `double` | Euler from fused R |
| `g_scale` | `double` | Last fused scale |
| `g_user_scale_correction` | `double` | Persists across start/stop |
| `g_raw_x/y/z, g_raw_yaw` | `double` | Raw camera-frame VO |
| `g_ax/ay/az, g_gx/gy/gz` | `float` | Last sensor sample |
| `g_mean_flow, g_inlier_count, g_step_count, g_step_freq, g_stride_length, g_pose_flags, g_heading` | mixed | Diagnostic mirror |

Helper `resetPoseState()` (`native-lib.cpp:51-60`) — must be called with `state_mutex` held; zeroes everything except `g_scale` (kept across resets).

### 8.2 JNI lifecycle

- `JNI_OnLoad` — caches `VioData` `jclass` (NewGlobalRef) and `<init>` `jmethodID`. Returns `JNI_VERSION_1_6`.
- `JNI_OnUnload` — `DeleteGlobalRef` on cached class.

### 8.3 Every JNI function — Kotlin counterpart, threading

All `extern "C"`.

| C symbol | Kotlin counterpart | Threading |
|----------|--------------------|-----------|
| `Java_..._startVIO` | `external fun startVIO()` | Main / VM init |
| `Java_..._setImuNoiseParameters` | (no Kotlin declaration — leftover hook) | n/a |
| `Java_..._stopVIO` | `external fun stopVIO()` | Main |
| `Java_..._processCameraFrameDirect` | `external fun processCameraFrameDirect(...)` | CameraX executor thread |
| `Java_..._processGyroscope` | `external fun processGyroscope(...)` | SensorManager event thread |
| `Java_..._processAccelerometer` | `external fun processAccelerometer(...)` | SensorManager event thread |
| `Java_..._resetVIO` | `external fun resetVIO()` | Main |
| `Java_..._setScale` | `external fun setScale(scale)` | Main / settings |
| `Java_..._setDepthMap` | `external fun setDepthMap(...)` | Background MiDaS thread |
| `Java_..._setIntrinsics` | `external fun setIntrinsics(fx, fy, cx, cy)` | Main / camera open |
| `Java_..._setInitialHeading` | `external fun setInitialHeading(azimuthRad)` | Main |
| `Java_..._setUserHeight` | `external fun setUserHeight(heightM)` | Main / settings |
| `Java_..._getInitStatus` | `external fun getInitStatus(): Int` | UI thread |
| `Java_..._clearInitTimeout` | `external fun clearInitTimeout()` | UI thread |
| `Java_..._loadStoredCalibration` | `external fun loadStoredCalibration(...)` | Main |
| `Java_..._getCalibration` | `external fun getCalibration(...)` | Main |
| `Java_..._getPositionCovariance` | `external fun getPositionCovariance(out): Boolean` | UI thread |
| `Java_..._nativeLoadCalibration` | `external fun nativeLoadCalibration(path): Boolean` | Main |
| `Java_..._nativeLoadLoopClosureVocabulary` | `external fun nativeLoadLoopClosureVocabulary(path): Boolean` | Main |
| `Java_..._nativeGetEventCountersJson` | `external fun nativeGetEventCountersJson(): String` | Main / sim save |
| `Java_..._nativeResetEventCounters` | `external fun nativeResetEventCounters()` | Main / sim start |
| `Java_..._nativeSetExtrinsicsRotation` | `external fun nativeSetExtrinsicsRotation(...)` | Main / camera open |

Dead-code symbols still in source (commented out, no Kotlin caller):
- `Java_..._processCameraFrame` (ByteArray) — superseded by zero-copy direct-buffer path.
- `Java_..._setMagnetometerHeading` — `IMUPreintegrator::setMagnetometerHeading` itself still live.

### 8.4 Marshaling specifics

- **Direct ByteBuffers** (`processCameraFrameDirect`): `env->GetDirectBufferAddress` returns underlying memory pointer, no copy. Y plane handling: dense path single memcpy when `yRowStride == width`, else per-row memcpy. UV plane handling: three branches (NV21 single memcpy, stride-padded per-row, planar fallback fills 128 grayscale). Output is `thread_local std::vector<uint8_t> nv21_buf` resized once and reused.
- **Float arrays**: Read `GetFloatArrayElements` + `ReleaseFloatArrayElements(JNI_ABORT)`. Write `SetFloatArrayRegion`. Length validation always present.
- **Strings**: `GetStringUTFChars(jpath, nullptr)` → copy into `std::string` → `ReleaseStringUTFChars`.
- **Return string**: `env->NewStringUTF(json.c_str())`.
- **Return object** (`processCameraFrameDirect`): `env->NewObject(g_viodata_cls, g_viodata_ctor, …)` with 28 args. Constructor signature `"(DDDDDDDIIDZ[FDDDDFFFFFFDIIDDIDD)V"`.

### 8.5 Coordinate-frame swap at JNI boundary

`processCameraFrameDirect` swaps Z-up → Y-up:
```cpp
g_x = output.t.at<double>(0);   // East — same in both frames
g_y = output.t.at<double>(2);   // Up   — Z-up index 2 -> exposed as Y
g_z = output.t.at<double>(1);   // North — Z-up index 1 -> exposed as Z
```
Same swap mirrored in `VioEngine::getPose`. `output.rawT` is **not** swapped (OpenCV camera frame, not world).

Euler decomposition:
```cpp
g_pitch = asin(-R(1,2));
if (cos(g_pitch) > 1e-6) {
    g_roll = atan2(R(0,2), R(2,2));
    g_yaw  = atan2(R(1,0), R(1,1));
} else {
    g_roll = 0;
    g_yaw  = atan2(-R(2,0), R(0,0));
}
```
YXZ-style with gimbal-lock branch when `pitch ≈ ±π/2`.

---

## 9. NativeBridge.kt — every external function

`NativeBridge` is a Kotlin `object` (singleton). `init` block calls `System.loadLibrary("navsight")` and stores `isLibraryLoaded`. Single accessor: `fun isLoaded(): Boolean = isLibraryLoaded`.

| Kotlin function | Args | Return | Notes |
|-----------------|------|--------|-------|
| `startVIO()` | — | Unit | |
| `stopVIO()` | — | Unit | |
| `processCameraFrameDirect(yBuffer: ByteBuffer, uvBuffer: ByteBuffer, width: Int, height: Int, yRowStride: Int, uvRowStride: Int, uvPixelStride: Int, timestamp: Long, rollingShutterSkewNs: Long)` | 9 args | `VioData` | ByteBuffers must be **direct** |
| `processGyroscope(timestamp, x, y, z)` | 4 args | Unit | |
| `processAccelerometer(timestamp, x, y, z)` | 4 args | Unit | |
| `resetVIO()` | — | Unit | |
| `setScale(scale: Double)` | 1 arg | Unit | Persists across start/stop |
| `setDepthMap(depthData: FloatArray, width: Int, height: Int)` | 3 args | Unit | |
| `setIntrinsics(fx, fy, cx, cy: Double)` | 4 args | Unit | |
| `setInitialHeading(azimuthRad: Double)` | 1 arg | Unit | Also seeds Madgwick yaw |
| `setUserHeight(heightM: Float)` | 1 arg | Unit | Used by PDR stride and MiDaS gating |
| `getInitStatus(): Int` | — | Int | 0=WAIT_STATIONARY, 1=WAIT_MOTION, 2=READY, 3=TIMEOUT_NEEDS_USER |
| `clearInitTimeout()` | — | Unit | |
| `loadStoredCalibration(rotation: FloatArray, gyroBias: FloatArray, accelBias: FloatArray)` | 3 arrays | Unit | rotation = 9 floats, biases = 3 floats |
| `getCalibration(rotation, gyroBias, accelBias): Boolean` | 3 arrays out | Boolean | |
| `getPositionCovariance(out: FloatArray): Boolean` | 1 array (len 3) | Boolean | `[σ_xx, σ_xz, σ_zz]` in m² |
| `nativeLoadCalibration(path: String): Boolean` | 1 arg | Boolean | |
| `nativeLoadLoopClosureVocabulary(path: String): Boolean` | 1 arg | Boolean | ORBvoc.bin path |
| `nativeGetEventCountersJson(): String` | — | String | |
| `nativeResetEventCounters()` | — | Unit | |
| `nativeSetExtrinsicsRotation(R_bc_flat: FloatArray)` | 1 array | Unit | 9 floats row-major |

Commented-dead Kotlin declarations: `processCameraFrame(frameData: ByteArray, …)`, `setMagnetometerHeading(yawRad)`.

**Callback flow**: there are no JNI callbacks. All output is via return value or pre-allocated array filled via `SetFloatArrayRegion`. Threading is one-way push from Kotlin to native. C++-side async threads (BA worker, loop-closure worker) write into shared C++ state, not Kotlin.

---

## 10. Threading and concurrency

### 10.1 Threads in this stack

1. **Camera thread** (CameraX executor) — calls `processCameraFrameDirect`. Runs `Tracker::processFrame` synchronously.
2. **Sensor thread** — calls `processGyroscope`/`processAccelerometer`.
3. **BA worker thread** (`tracker_.ba_thread_`) — detached, launched per keyframe.
4. **Loop-closure worker thread** — owned by Tracker, period 1 Hz.
5. **MiDaS depth thread** (managed by Kotlin) — calls `setDepthMap`.
6. **Main / UI thread** — calls all start/stop/get/set methods.

### 10.2 Locks

| Lock | Owner | Protects |
|------|-------|----------|
| `state_mutex` | JNI module | `g_vision` shared_ptr + all `g_*` accumulators + `g_user_scale_correction` |
| `Tracker::mutex_` | Tracker | All Tracker non-pose state |
| `Tracker::pose_mutex_` | Tracker | `global_R_`, `global_t_`, `scalar_heading_` |
| `Tracker::ba_result_mutex_` | Tracker | `ba_result_landmarks_`, `ba_result_pending_` |
| `IMUPreintegrator::mutex_` | IMU | All IMU ring/bias/Madgwick state |
| `EKFState` internal locks | EKFState | State vector + covariance |

There is **no** mutex inside `VioEngine` itself, `LensCorrector`, `CameraCalibration`, or `EventCounters` (the last is lock-free atomic).

### 10.3 Lock-acquire pattern in JNI

The JNI shims use a consistent pattern:
```cpp
std::shared_ptr<VioEngine> vision;
{
    std::lock_guard<std::mutex> lock(state_mutex);
    vision = g_vision;
}
// ... heavy work outside the lock ...
if (vision) vision->...;
```

This **copies the shared_ptr under the lock** then releases it. The reference count keeps the engine alive even if a concurrent `stopVIO` clears `g_vision`. Heavy work runs without holding `state_mutex`.

---

## 11. Magic numbers — exhaustive list

### 11.1 In `LensCorrector.cpp`

- `kDistortionEpsilon = 1e-9` — passthrough threshold.

### 11.2 In `CameraCalibration.cpp`

- RMS gate: `tmp.rms_px > 1.0` rejects. Hard-coded **1.0 px** ceiling.

### 11.3 In `native-lib.cpp`

- `JNI_VERSION_1_6`.
- VioData ctor signature `"(DDDDDDDIIDZ[FDDDDFFFFFFDIIDDIDD)V"`.
- Gimbal-lock guard `1e-6` on `cos(pitch)`.
- Required array sizes: 9 (rotation), 3 (gyro/accel bias), 3 (covariance out).
- UV passthrough fill value 128.

### 11.4 In `EventCounters.h`

- JSON `reserve(1100)`.
- MiDaS gating thresholds: `pts3d.size() < 15`, `camera_h ∉ [0.8, 2.2]`, `|accel| < 5 m/s²`, floor-matches `< 8`, target_scale > 3× current.

### 11.5 In `Tracker.h` (constants this layer depends on)

- `LOOP_CLOSURE_QUERY_PERIOD_S = 1.0`
- `LOOP_CLOSURE_TEMPORAL_EXCL_NS = 30 s`
- `LOOP_CLOSURE_DAMPING_FRAMES = 10`
- `LOOP_CLOSURE_PNP_SIGMA_FLOOR_M = 2.0 m`
- `LOOP_CLOSURE_DRIFT_RATE = 0.032 m/m walked`
- `LOOP_CLOSURE_BASE_ROT_SIGMA_RAD = 0.34907` (≈20°)
- `MAX_FEATURES = 200`, `MIN_FEATURES = 80`
- `QUALITY_LEVEL = 0.05`, `MIN_DIST = 10.0 px`
- `RANSAC_CONF = 0.9999`, `RANSAC_THRESH = 1.5 px`
- `MIN_PARALLAX_PX = 0.8`, `FB_CHECK_THRESH = 4.0` (sqr px)
- `MIN_FLOW_PX = 0.4`, `MAX_FLOW_PX = 150.0`
- `MIN_INLIERS = 8`
- `MIN_INLIER_RATIO = 0.25`
- `GYRO_ROT_ONLY_THRESH = 2.0 rad/s`
- `FLOW_RAYLEIGH_REJECT = 0.3`
- `BLUR_VAR_THRESH = 80.0`
- `ZUPT_GYRO_THRESH = 0.04 rad/s`
- `ACCEL_BIAS_WARMUP = 150` samples; `ACCEL_BIAS_ALPHA = 0.005`
- `RELOC_TRIGGER_FRAMES = 3`; `RELOC_LOW_INLIER_BAR = MIN_INLIERS / 2 = 4`
- `RELOC_RECENT_KFS = 5`
- `RELOC_LOWE_RATIO = 0.75f`
- `RELOC_MIN_INLIERS = 30`
- `RELOC_ID_REATTACH_RADIUS = 3.0f px`
- `SCALE_BOOTSTRAP_COUNT = 15`
- `OBSERVER_C_SOLVE_INTERVAL = 10`
- `TD_WARMUP_FRAMES = 60` (~2 s @ 30 fps)
- `BA_MAX_SOLVE_US = 200_000`
- `ScaleFuser{0.20, 1.0}` initial scale = 0.20, initial variance = 1.0

---

## Cross-references and notes

- The **Z-up world frame fix** (memory entry `project_z_up_fix_2026_05_08.md`) is the reason for the explicit Z-up→Y-up swap at both `VioEngine.cpp:269-281` and `native-lib.cpp:378-391`. Internal pipeline runs in Z-up ENU; the JNI/Kotlin/sim layer expects Y-up.
- The **VIO Rotation Bugs** memory entry flagged intrinsics scaling as a root cause. The current architecture pushes calibration JSON intrinsics straight into Tracker without rescaling — the **RMS ≤ 1.0 px gate** is the only protection against a mismatched-resolution calibration.
- The **no-magnetometer-in-VIO** memory entry: `setMagnetometerHeading` JNI is dead code. Mag is only used inside `IMUPreintegrator` for a one-shot init seed by `InertialInitializer`.
- `setInitialHeading` does **two** things — it seeds Tracker yaw and seeds Madgwick yaw. Dual-seed was added 2026-05-09 to fix sim 1778258249750 ("at first heading is correct then it instantly rotates 180 degrees").
- `processCameraFrame` (ByteArray) JNI is preserved as commented dead code — live path is the zero-copy `processCameraFrameDirect`.
- `IMUPreintegrator::setNoiseParameters(accel_noise, gyro_noise, accel_rw, gyro_rw)` is exposed via `Java_..._setImuNoiseParameters` but has no Kotlin counterpart. Appears to be a leftover hook for an external configuration/test harness.
