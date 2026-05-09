# 01 — NavSight EKF Core (EKFState + IMUPreintegrator + InertialInitializer)

> Convention note (post commit `ceb8af3`, 2026-05-08): the world frame is **ENU, Z-up** (X = East, Y = North, Z = Up). Gravity is `(0, 0, -9.81) m/s²`. The earlier Y-up "fix" (commit `c1c15b2`) was reverted because it desynchronised the EKF math from the Madgwick filter and the Android sensor body frame. See `EKFState.cpp:140` for the gravity declaration and `EKFState.cpp:1156-1170` for the Y-up → Z-up rationale.

---

## 1. State Vector Layout

The EKF uses an **error-state** formulation. Total dimension = `IMU_STATE_DIM + 6·N_clones + 5·N_slam` = `19 + 6·N_clones + 5·N_slam`. Block order in `P_` is fixed: `[IMU(19) | Clone_0..N(6 each) | SLAM_0..M(5 each)]` (`EKFState.cpp:391-471`).

### 1.1 IMU error-state (19 DOF) — `EKFState.h:33-42, 387-393`; `EKFState.cpp:78-110`

| Rows | Symbol | Name | Frame | Units | Init σ | Mean state member |
|------|--------|------|-------|-------|--------|--------|
| 0–2  | δθ    | attitude error | **world** (ENU, Z-up) | rad | `0.02` (`EKFState.cpp:80`) | `R_GtoI_` (3×3) |
| 3–5  | δb_g  | gyro bias error | body | rad/s | `0.01` (`EKFState.cpp:81`) | `b_g_` (3×1) |
| 6–8  | δv    | velocity error | world | m/s | `0.5` (`EKFState.cpp:82`) | `v_G_` (3×1) |
| 9–11 | δb_a  | accel bias error | body | m/s² | `0.1` (`EKFState.cpp:83`) | `b_a_` (3×1) |
| 12–14 | δp   | position error | world | m | `0.01` (`EKFState.cpp:84`) | `p_G_` (3×1) |
| 15   | δt_d  | camera–IMU time-offset error | scalar | s | √`P_td_` (default `(0.005)²`, `EKFState.h:450`) | `t_offset_cam_imu_` (double, default `0.010` s) |
| 16–18 | δφ_bc | body→camera rotation perturbation | so(3) | rad | `INIT_EXTR_STD = 0.05` rad ≈ 3° (`EKFState.cpp:106`) | `R_bc_` (`cv::Matx33d`, default `diag(1,-1,-1)`) |

**Key conventions**:

- δθ is the **WORLD-frame, left-perturbation**: `R_new = exp([δθ_world]_×) · R_old`. Update applied as `R_GtoI_ = Rodrigues(δθ) · R_GtoI_old` (`EKFState.cpp:670`). This makes ∂yaw/∂δθ the constant world-Z axis `(0, 0, 1)` regardless of body orientation — `EKFState.cpp:1078-1102` documents the 2026-05-09 fix.
- `R_bc` updates by left-multiply: `R_bc = Exp(δφ_bc) · R_bc_hat` (`EKFState.h:383-385`). However, the actual `R_bc_` mean update is currently **SKIPPED** — `EKFState.cpp:699-704` (see § 9.2).

### 1.2 Camera-pose clones — `CLONE_DIM = 6` per clone

`CameraPose` struct (`EKFState.h:10-28`):
- `R_GtoC` (3×3 CV_64F) — world→camera rotation
- `p_G` (3×1 CV_64F) — camera position in world
- `R_FEJ`, `p_FEJ` — first-estimate Jacobian linearisation locks
- `timestamp_ns` (int64), `state_id` (int, monotonic)

Per-clone block: `[δθ_c (3 DOF), δp_c (3 DOF)]`. Cap `MAX_CLONES = 11` (`EKFState.h:395`); auto-marginalised at `EKFState.cpp:478-481`.

### 1.3 SLAM-feature block — `SLAM_FEATURE_DIM = 5` per feature

Inverse-depth, anchored to clone of first promotion (`EKFState.h:79-108, 511-523`):

| Slot | Symbol | Meaning |
|------|--------|---------|
| 0 | α | normalised x in anchor's camera frame |
| 1 | β | normalised y in anchor's camera frame |
| 2 | ρ | inverse depth (1/Z_anchor) |
| 3, 4 | pad0, pad1 | identity-pinned, no measurement coupling |

Reprojection: `p_anchor = (1/ρ)·(α, β, 1)`; `p_C = R_C_anchor · p_anchor + t_C_anchor`; `(u,v) = (p_C.x/p_C.z, p_C.y/p_C.z)`.

Cap `MAX_SLAM_FEATURES = 12` (`EKFState.h:107`). `SLAM_PAD_VARIANCE = 1e-6` (`EKFState.h:108`). Pad cols are pinned identity.

### 1.4 Layout invariant in `P_`

`addClone` splices new clone block **before** the SLAM block (not at the end) to preserve SLAM cross-correlations (`EKFState.cpp:391-471`). This is the ADR-009 fix for the 5–11 m teleportations after clone churn that ADR-006 documented.

---

## 2. Covariance Matrix `P_`

Square `cv::Mat`, `CV_64F`. Total side: `19 + 6·N_clones + 5·N_slam`. Max: `19 + 6·11 + 5·12 = 145`. Empty until `initializeFull` (`EKFState.cpp:78`).

```
                IMU(19)   Clones(6N)   SLAM(5M)
        ┌──────────────────────────────────────┐
IMU(19) │ P_II         P_IC          P_IS      │
Clones  │ P_CI         P_CC          P_CS      │
SLAM    │ P_SI         P_SC          P_SS      │
        └──────────────────────────────────────┘
```

**Conventions**:
- Error-state Jacobians: every `H` linearises residual w.r.t. `δx`, not full state.
- **First-Estimate Jacobians (FEJ)**: clone Jacobians use `R_FEJ`/`p_FEJ` locked at clone insertion; residuals use the current state. Pattern at `EKFState.h:14-17, 102-103`. Same for SLAM (`SlamFeature::p_global_FEJ`, `anchor_R_FEJ`, `anchor_p_FEJ` at `EKFState.h:517-519`).
- Joseph-form update: `P = (I−KH_w)·P·(I−KH_w)^T + K·R·K^T`, then symmetrised `P = (P + P^T)/2` (`EKFState.cpp:743-747`).
- Outer χ² gate (loop closure only): `kChi2Threshold = 22.5` ≈ χ²(0.999, 6 DOF) (`EKFState.cpp:1000`).

**Initial diagonal** (`initializeFull`, `EKFState.cpp:78-110`):

| Block | σ² value |
|-------|---------|
| δθ (0–2) | `(0.02)² = 4e-4` rad² |
| δb_g (3–5) | `(0.01)² = 1e-4` (rad/s)² |
| δv (6–8) | `(0.5)² = 0.25` (m/s)² |
| δb_a (9–11) | `(0.1)² = 0.01` (m/s²)² |
| δp (12–14) | `(0.01)² = 1e-4` m² |
| δt_d (15) | `P_td_` (`(0.005)²` or refined `(0.002)²`) |
| δφ_bc (16–18) | `(0.05)² = 2.5e-3` rad² |

**Process noise Q** (`EKFState.cpp:217-258`) — driven by preintegration `imu_cov` if present, else fallback to constants `sigma_g_=0.01`, `sigma_a_=0.1`, `sigma_bg_=0.0001`, `sigma_ba_=0.001` (`EKFState.h:478-481`); `SIGMA_TD_RW = 1e-7 s/√Hz` (`EKFState.cpp:244`); `SIGMA_EXTR_RW = 1e-8 rad/√Hz` (`EKFState.cpp:254`).

---

## 3. Public functions

### 3.1 `EKFState`

#### Lifecycle

- **`EKFState()`** (`EKFState.cpp:17`): calls `reset()`.
- **`reset()`** (`EKFState.cpp:19-56`): clears every member; locks `snapshot_mutex_` over the whole reset; `msckf_frames_since_call_ = MSCKF_QUIET_PROPAGATION` (cold start ⇒ first MSCKF call uses 0.5× damping); `R_bc_ = diag(1,-1,-1)`.
- **`initialize(double initial_scale)`** (`EKFState.cpp:58-63`): legacy scale-only init.
- **`initializeFull(R_GtoI, gyro_bias, accel_bias)`** (`EKFState.cpp:67-114`): full IMU init, 19×19 `P_`, sets `full_initialized_ = true`.

#### `propagateIMU(deltaR, deltaV, deltaP, dt, imu_cov, J_R_bg, J_V_bg, J_V_ba, J_P_bg, J_P_ba)` — `EKFState.cpp:118-292`

**Mean** (lines 140-143):
- `R_new = R_GtoI_ · deltaR`
- `v_new = v_G_ + g·dt + R_GtoI_^T · deltaV` with `g = (0, 0, −9.81)`
- `p_new = p_G_ + v_G_·dt + ½·g·dt² + R_GtoI_^T · deltaP`

**Φ blocks (19×19)** at `EKFState.cpp:156-214`:
- `∂δθ_new/∂δθ = deltaR^T`; `∂δθ_new/∂δb_g = −J_R_bg`
- `∂δv_new/∂δθ = R^T · [deltaV]_×`; `∂δv_new/∂δb_g = −R^T · J_V_bg`; `∂δv_new/∂δb_a = −R^T · J_V_ba`
- `∂δp_new/∂δθ = R^T · [deltaP]_×`; `∂δp_new/∂δv = I·dt`; `∂δp_new/∂δb_g = −R^T · J_P_bg`; `∂δp_new/∂δb_a = −R^T · J_P_ba`
- δt_d, δφ_bc rows stay identity (eye-init).

**Side effects**: writes `R_GtoI_, v_G_, p_G_`; **clamps `‖v_G_‖ ≤ 5 m/s`** (`EKFState.cpp:288-291`, workaround); updates `P_II`, `P_IC`, `P_CI`; ticks `msckf_frames_since_call_` and resets `msckf_damping_step_` to 0 once a quiet period has elapsed (lines 129-134).

#### Clone management

- **`addClone(R_GtoC, p_G, ts_ns)`** (`EKFState.cpp:355-481`): mints `state_id`, splices `CLONE_DIM` rows/cols **before SLAM block**, builds augmentation Jacobian `J` with `I` on δθ_c-from-δθ and `I` on δp_c-from-δp; computes `J·P_pre·J^T` and cross-correlations; auto-prunes via `marginalizeOldestCloneNoLock` when `window_.size() > MAX_CLONES`. Holds `snapshot_mutex_`.
- **`pruneWindow(max_poses=11)`** (`EKFState.cpp:483-493`): single-lock loop over no-lock body.
- **`marginalizeOldestClone()`** (`EKFState.cpp:495-501`): public entry; locks then delegates.
- **`marginalizeOldestCloneNoLock()`** (`EKFState.cpp:503-571`, private): drops clone rows/cols `[19, 25)`, pops `window_.front()`, **drops every SLAM feature anchored to it** (lines 564-570). Drop-only, not Schur, by design (lines 511-514).
- **`getClonePose / getCloneFEJ / getCloneCovIdx / getCloneTimestamp`** (`EKFState.cpp:1178-1219`): linear scans of `window_`.
- **`getCloneSnapshot(int max_clones = 5)`** (`EKFState.cpp:2077-2114`): thread-safe snapshot for BA worker (Plan Step 6 / ADR-012).
- **`getStateDim() / getCovariance() / getWindow() / getLatestCloneId()`** — accessors.

#### Updates

| Function | Math | Effect |
|---|---|---|
| `applyMSCKFUpdate(H, res, R_noise)` (`EKFState.cpp:584-797`) | `K = P·H_w^T·S^{-1}`, `dx = K·r_w`; Joseph + symmetrisation | Per-row Huber kernel (`MSCKF_HUBER_DELTA = 2.4477 = √χ²(0.95,2)`, hard-reject above 3δ); δp damping ramp 0.5→1.0 over 5 calls; advances damping schedule and resets `msckf_frames_since_call_=0` (line 751). |
| `updateRelativePose(t_world_metric, clone_id, var_t)` (`EKFState.cpp:801-832`) | `r = t_world_metric − (p_G_ − p_clone)`; `H = +I` on δp(curr), `−I` on δp_clone | 3-DOF position-delta constraint. |
| `updateRelativeRotation(R_meas_body, σ², clone_id)` (`EKFState.cpp:834-893`) | `r = log(R_meas · R_pred^T)`, `R_pred = R_GtoI_ · R_clone^T`; `H = +I` on δθ, `−R_meas_body` on δθ_c | Per-frame rotation residual from `recoverPose`. |
| `updateAbsolutePose(target_R, target_p, σ²_R, var_p)` (`EKFState.cpp:925-1060`) | 6×1 `r = [log(R_target·R_GtoI_^T); target_p − p_G_]`; `H = +I` on δθ and δp | Loop-closure injection. χ² gate 22.5; per-block diagnostic m²_R, m²_p (lines 1021-1036). |
| `updateGravityAlignedYaw(yaw_meas, var, roll, pitch)` (`EKFState.cpp:1062-1115`) | `r = wrap(yaw_meas − getYaw(roll,pitch))`; `H = (0, 0, 1)` on δθ | World-Z perturbation (left-perturb convention). |
| `updatePDRStep(dx, dy, var)` (`EKFState.cpp:1117-1135`) | `r = (dx, dy)`; `H` selects δp_x and δp_y | 2-DOF XY position constraint. |
| `updateScale(observed, conf)` (`EKFState.cpp:296-312`) | 1-D Kalman, `R = SIGMA_SCALE_MEAS/(conf+0.01)`, χ² ≤ 9 | Legacy scalar scale. |
| `updateZUPT()` (`EKFState.cpp:314-345`) | `v_G_ = 0`; velocity-block of `P_` × 0.01 (cross-corr × √0.01); `P_scale_ × 0.99`; `P_(15,15) × 0.999` | Zero-velocity update. |
| `getYaw(roll, pitch)` (`EKFState.cpp:1150-1174`) | Tilt-removal sandwich `R_aligned = R_y(pitch)·R_x(roll) · R_GtoI_ · (R_y·R_x)^T`; `yaw = atan2(R_aligned[1,0], R_aligned[0,0])` | Z-up nav yaw extraction. |

#### SLAM features

- **`setSlamIntrinsics(fx,fy,cx,cy)`** (`EKFState.cpp:1300-1305`): cached intrinsics; defaults `500/500/320/240`.
- **`addSlamFeature(feature_id, p_global_init, anchor_clone)`** (`EKFState.cpp:1351-1449`): projects to anchor cam, requires `Z_c ≥ 0.01`, sets `(α, β, ρ) = (Xc/Zc, Yc/Zc, 1/Zc)`. Initial 5×5 cov: `σ_α=σ_β=max(1/500, 1e-3)`, `σ_ρ=max(0.5·ρ, 1e-3)`, pads = `SLAM_PAD_VARIANCE`. **Appends with zero cross-correlation** (workaround, lines 1432-1443).
- **`removeSlamFeature(slot)`** (`EKFState.cpp:1451-1568`): block-Schur `P_kk' = P_kk − P_ks·inv(P_ss)·P_sk`; PSD diagonal sniff falls back to drop-only (lines 1546-1562).
- **`updateSlamFeature(slot, obs, clone_ids, σ²_uv)`** (`EKFState.cpp:1755-1860`): stacks 2K rows → `applyMSCKFUpdate`; tracks `last_obs_rms`.
- **`applyMSCKFFeature(obs, clone_ids, p_global, σ²_uv)`** (`EKFState.cpp:1862-2062`): builds `H_x` (clone), `H_f` (3-DOF feature), residual; **SVD-based left null-space projection** of `H_f` (lines 2042-2053). **Step 8a TD column 15** filled from finite-difference of pixel velocity (`H_td = −v_normalised`, lines 1979-2025).
- Helpers: `getSlamFeatureCovIdx`, `getSlamFeatureCount`, `getSlamFeatureSlot`, `getSlamFeatureGlobalPosition`.
- Internal: `slamBlockStart()` (`EKFState.cpp:1307-1309`), `slamFeatureCovIdxInternal` (`EKFState.cpp:1311-1314`), `slamReprojectionJacobian` (`EKFState.cpp:1570-1753`) — defensive try/catch, **uses `mat * -1.0` instead of `-mat`** to dodge OpenCV 4.5.3 unary-minus bug on empty MatExpr operands (lines 1716-1727).

#### Time-offset / extrinsics

- **`setTimeOffset(td_seconds)`** (`EKFState.cpp:1236-1255`): clamps `t_offset_cam_imu_` to ±0.1 s; sets `P_td_ = (0.002)²` post-warmup; propagates into `P_(15,15)` if full-init.
- **`getTimeOffset()`**, **`getTimeOffsetStd()`** inline (`EKFState.h:326, 330-335`): `sqrt(P_(15,15))` if full-init else `sqrt(P_td_)`.
- **`setExtrinsicsRotation(R_bc)`** (`EKFState.cpp:1262-1269`): replaces nominal R_bc_; does NOT modify covariance.
- **`getExtrinsicsAngleDeg()`** (`EKFState.cpp:1271-1294`): drift from `R_bc_initial = diag(1,-1,-1)` (NOT identity — bug fix from sim 1778147132092).
- **`setRotation(R_GtoI)`** (`EKFState.cpp:1137-1148`): bootstrap-only; replaces only R_GtoI_, leaves position/vel/biases/td/R_bc/cov/window untouched. Used to inject magnetometer initial heading after `initializeFull` already fired (bug from sim 1778260615221, 2026-05-09).

### 3.2 `IMUPreintegrator` — public surface

#### Sample sinks
- `addGyroReading(ts_ns, x, y, z)` (`IMUPreintegrator.cpp:83-107`): drops NaN/Inf; updates `last_g{x,y,z}`; LP filters `gyro_mag_filtered_` with `GYRO_LP_ALPHA = 0.15`; calls `updateMadgwickLocked`.
- `addAccelReading(ts_ns, x, y, z)` (`IMUPreintegrator.cpp:111-141`): drops NaN/Inf; updates `last_a{x,y,z}`; LP filters `filtered_gravity_` with `α = 0.02` (~1.6 s τ at 200 Hz); calls `updateMotionMode`, `detectStep`, `tryInitializeGravityLocked`, `tryInitializeGyroBiasLocked`.

#### Preintegration
- **`PreintegratedMeasurement integrate(start_ns, end_ns)`** (`IMUPreintegrator.cpp:213-353`). Returns `deltaR (3×3)`, `deltaV (3×1)`, `deltaP (3×1)`, `cov (9×9 ordered R/V/P)`, `J_R_bg`, `J_V_bg`, `J_V_ba`, `J_P_bg`, `J_P_ba` (3×3 each — note: no `J_R_ba`), `dt`, `sample_count`.

#### Gravity / biases / mag
- `getFilteredGravity()` — LP-filtered accel; live channel.
- `getGyroBias()` / `setGyroBias(bx,by,bz)` / `refineGyroBiasDuringZUPT()` (`IMUPreintegrator.cpp:964-1002`): blends bias with `α = 0.01` over last 20 gyro samples.
- `setMagnetometerHeading(yaw_rad)` / `hasMagHeading()` / `getMagHeading()` — used **once at startup only**, never blended live (per memory `feedback_no_magnetometer`).
- `setUserStride(m)` / `getUserStride` / `hasCalibratedStride` (`IMUPreintegrator.cpp:586-607`): clamped [0.3, 1.5] m; preserved across `reset()`.
- `setUserHeight(m)` / `getUserHeight()`: clamped [1.0, 2.5] m.
- `getStepInfo()` (`IMUPreintegrator.cpp:515-583`): full `StepInfo` struct.
- `getVehicleSpeedEstimate()` (`IMUPreintegrator.cpp:455-458`): legacy.

#### Madgwick
- `getOrientationQuaternion(q0,q1,q2,q3)`; `getHeading()` returns `yaw_nav = −yaw_math` (CW-positive nav, `IMUPreintegrator.cpp:858-871`); `getMadgwickRoll/Pitch`; `setInitialMadgwickYaw(azimuth_rad_nav)` (bootstrap, `IMUPreintegrator.cpp:734-748`); `resetOrientationFilter()`.

#### Other
- `setNoiseParameters(accel_n, gyro_n, accel_rw, gyro_rw)`; `lastAccelX/Y/Z()`, `lastGyroX/Y/Z()`; `reset()` (`IMUPreintegrator.cpp:621-661`, preserves `user_stride_m_`); `getAccelBuffer / getGyroBuffer`; `isInitialized()`.

### 3.3 `InertialInitializer`

- `addImuData(ts, ax, ay, az, gx, gy, gz)` (`InertialInitializer.cpp:101-152`): drives FSM.
- `getStatus()`, `isReady()`, `needsUserPrompt()`, `getInitialRotation()`, `getGyroBias()`, `getAccelBias()`.
- `loadCalibration(R, b_g, b_a)` (`InertialInitializer.cpp:53-70`): skip the gate, jump to `WAIT_MOTION`.
- `clearTimeout()` (`InertialInitializer.cpp:72-85`): reset to `WAIT_STATIONARY`, set `force_accept_` to bypass variance gates next window.
- `reset()`: full FSM reset.
- Private: `runStationaryCalibration` (`InertialInitializer.cpp:154-243`); `detectMotion(ax,ay,az)` returns `|‖a‖ − 9.81| > 0.5 m/s²` (`InertialInitializer.cpp:245-248`); `trimAccelWindow()`.

---

## 4. Member variables

### 4.1 `EKFState` — `EKFState.h:402-560`

| Member | Type | Default | Read | Write |
|--------|------|---------|------|-------|
| `scale_` | double | `0.20` | `getScale`, `updateScale` | `reset/initialize/updateScale` |
| `scale_fej_` | double | `−1.0` | (unused) | `reset` |
| `P_scale_` | double | `0.25` | `updateScale`, `updateZUPT` | same + `reset/initialize` |
| `R_GtoI_` | cv::Mat 3×3 | identity | every read of rotation | `reset/initializeFull/propagateIMU/applyMSCKFUpdate/setRotation` |
| `b_g_` | cv::Mat 3×1 | zero | (preintegration in IMUPreintegrator subtracts gyro_bias_) | `reset/initializeFull/applyMSCKFUpdate` |
| `v_G_` | cv::Mat 3×1 | zero | `propagateIMU` | `reset/initializeFull/propagateIMU/updateZUPT/applyMSCKFUpdate` |
| `b_a_` | cv::Mat 3×1 | zero | (preintegration) | `reset/initializeFull/applyMSCKFUpdate` |
| `p_G_` | cv::Mat 3×1 | zero | `getPosition`, every position update | `reset/initializeFull/propagateIMU/applyMSCKFUpdate` |
| `P_` | cv::Mat NxN | empty | every update/propagate | `reset/initializeFull/propagateIMU/addClone/marginalize/applyMSCKFUpdate/addSlamFeature/removeSlamFeature/updateZUPT` |
| `window_` | deque<CameraPose> | empty | every clone op | `reset/addClone/marginalizeOldestCloneNoLock` |
| `next_state_id_` | int | `0` | `addClone` | `reset/addClone` |
| `full_initialized_` | bool | `false` | every entry-point gate | `reset/initializeFull` |
| `snapshot_mutex_` | mutable mutex | — | guards `window_` | locked |
| `t_offset_cam_imu_` | double | `0.010` | `getTimeOffset`, MSCKF feature update | `reset/setTimeOffset/applyMSCKFUpdate` |
| `P_td_` | double | `(0.005)²` | `getTimeOffsetStd` | `reset/setTimeOffset/updateZUPT` |
| `R_bc_` | cv::Matx33d | `diag(1,-1,-1)` | `getExtrinsicsRotation`, `getExtrinsicsAngleDeg` | `reset/setExtrinsicsRotation`; **MSCKF update SKIPPED** (`EKFState.cpp:699-704`) |
| `initialized_` | bool | `false` | (legacy gate) | `reset/initialize` |
| `global_first_estimate_R_/p_/initialized_` | (3×3 / 3×1 / bool) | identity / zero / false | (mostly disabled paths) | `reset/addClone` |
| `sigma_g_, sigma_a_, sigma_bg_, sigma_ba_` | double | `0.01, 0.1, 0.0001, 0.001` | `propagateIMU` Q fallback | construction-time only |
| `msckf_frames_since_call_` | int | `MSCKF_QUIET_PROPAGATION` | `propagateIMU/applyMSCKFUpdate` | same |
| `msckf_damping_step_` | int | `0` | same | same |
| `msckf_huber_rejected_count_` | int | `0` | `getMSCKFHuberRejectedCount` | `applyMSCKFUpdate` |
| `extr_log_skip_` | int | `0` | `applyMSCKFUpdate` (throttle) | same |
| `slam_features_` | vector<SlamFeature> | empty | every SLAM op | `reset/addSlam/removeSlam/applyMSCKFUpdate/marginalizeOldest…` |
| `slam_fx_, fy_, cx_, cy_` | double | `500/500/320/240` | `slamReprojectionJacobian/applyMSCKFFeature` | `setSlamIntrinsics` |

`SlamFeature` (`EKFState.h:511-523`): `feature_id`, `anchor_clone_id`, `state` (5×1), `p_global_FEJ` (3×1), `anchor_R_FEJ` (3×3), `anchor_p_FEJ` (3×1), `last_obs_rms`, `rms_bad_run`.

Static constants: `IMU_STATE_DIM=19`, `TD_STATE_OFFSET=15`, `EXTR_STATE_OFFSET=16`, `EXTR_STATE_DIM=3`, `CLONE_DIM=6`, `MAX_CLONES=11`, `SLAM_FEATURE_DIM=5`, `SLAM_FEATURE_ACTIVE_DIM=3`, `MAX_SLAM_FEATURES=12`, `SLAM_PAD_VARIANCE=1e-6`, `SIGMA_SCALE_RW=0.001`, `SIGMA_SCALE_MEAS=0.1`, `MSCKF_QUIET_PROPAGATION=5`, `MSCKF_DAMPING_RAMP_FRAMES=5`, `MSCKF_HUBER_DELTA=2.4477`.

### 4.2 `IMUPreintegrator` — `IMUPreintegrator.h:167-291`

Key fields (defaults / constants):

- `mutex_` (mutable), `gyro_buf_`, `accel_buf_` (`reserve MAX_BUF=2000`; halve when full).
- `gravity_init_samples_` (vector<Point3f>), `GRAVITY_INIT_WINDOW=40`, `GRAVITY_INIT_MAX_VAR=0.08f`, `GRAVITY_INIT_GYRO_MAX=0.12f`.
- `gravity_initialized_` (atomic), `gravity_vec_` default `(0,0,9.81)`, `roll_/pitch_` (DEAD: `getRoll/getPitch` never called).
- `filtered_gravity_` default `(0,0,9.81)` with α=0.02; `filtered_gravity_init_`.
- `last_a{x,y,z}`, `last_g{x,y,z}` floats.
- Step detection: `step_count_=0`, `last_step_ns_=0`, `step_period_s_=0.0`, `accel_mag_filtered_=accel_mag_prev_=accel_mag_slow_=9.81f`, `was_above_thresh_=false`. Constants: `STEP_ACCEL_THRESH_HIGH=10.1f`, `STEP_ACCEL_THRESH_LOW=9.3f`, `MIN_STEP_PERIOD_S=0.25`, `MAX_STEP_PERIOD_S=1.5`, `DEFAULT_STRIDE_M=0.65`. `user_height_m_=1.70f` (clamp [1.0,2.5]), `user_stride_m_=−1.0` (clamp [0.3,1.5]; persistent across reset). `STEP_PERIOD_BUF=8`. `ROTATION_STEP_GATE_RADPS=0.8f` ≈ 46°/s.
- Walking/vehicle: `accel_variance_est_`, `is_walking_pattern_`, `gyro_mag_filtered_`, `vehicle_speed_mps_`, `last_accel_ts_ns_`, `sustained_accel_s_`, `in_vehicle_mode_` — flagged "legacy but live" (`IMUPreintegrator.h:218-241`).
- Madgwick: `q0_=1, q1_=q2_=q3_=0`; `madgwick_last_ns_=0`; `madgwick_init_` (atomic); `pending_madgwick_yaw_nav_=0`; `MADGWICK_BETA=0.033f`; `MADGWICK_ACC_MIN=5.0f`, `MADGWICK_ACC_MAX=20.0f` m/s².
- Bias: `gyro_bias_=0`, `b_a_=0`, `gyro_bias_initialized_` (atomic), `gyro_bias_samples_=0`; `GYRO_BIAS_INIT_SAMPLES=200`.
- Mag: `mag_heading_=0`, `has_mag_heading_` (atomic), `last_mag_update_ns_=0`; `HEADING_CORRECTION_DAMPING=0.95f` (declared but unused — `getCorrectedHeading` is dead code).
- Noise: `accel_noise_sigma_=0.1f` (m/s²/√Hz), `gyro_noise_sigma_=0.01f` (rad/s/√Hz), `accel_rw_sigma_=0.001f`, `gyro_rw_sigma_=0.0001f`.

### 4.3 `InertialInitializer` — `InertialInitializer.h:80-103`

- `options_` (Options), `mutex_` (mutable).
- `state_` (Status, default `WAIT_STATIONARY`).
- `accel_window_`, `gyro_window_` (deques).
- `first_sample_ns_=0`, `has_first_sample_=false`, `force_accept_=false`.
- `R_GtoI_init_` (3×3, identity), `gyro_bias_=0`, `accel_bias_=0`.

`Options` defaults (`InertialInitializer.h:34-54`): `stationary_seconds=5.0`, `max_accel_var=0.05` m²/s⁴ (~0.13 m/s² RMS), `max_gyro_var=0.001` (rad/s)² (~0.018 rad/s RMS) — **variance, NOT mean magnitude** (key bug fix), `timeout_seconds=15.0`, `gravity_mag=9.81`.

---

## 5. IMU preintegration math — `IMUPreintegrator::integrate`, `IMUPreintegrator.cpp:213-353`

**Scheme: midpoint integration** (NOT Euler, NOT RK4). Forster 2017 §IV.A.

Per sub-step, with bias-corrected `ω = ω_raw − b_g` and `a = a_raw − b_a` (lines 276-282; accel-bias subtraction was Bug #2 fix at line 279):

1. **Rotation — full SO(3) exponential** (lines 284-288, 333):
   ```
   dR_step = Rodrigues(ω · dt)
   R_new   = R · dR_step
   ```
   Comment "never add matrices on SO(3)" — Bug #1 fix at line 285.

2. **Velocity / position — rotation at midpoint** (lines 291-298):
   ```
   dR_half = Rodrigues(ω · dt/2)
   R_mid   = R · dR_half
   dV      = R_mid · a · dt
   dP      = V · dt + ½ · R_mid · a · dt²
   ```

3. **Covariance** (lines 300-320), 9×9 ordered (R, V, P):
   ```
   F[0:3,0:3] = dR_step^T
   F[3:6,0:3] = −R · [a]_× · dt
   F[6:9,0:3] = −0.5 · R · [a]_× · dt²
   F[6:9,3:6] = I · dt
   G[0:3,0:3] = I · dt
   G[3:6,3:6] = R · dt
   G[6:9,3:6] = 0.5 · R · dt²
   Q          = blockdiag(σ_g² I, σ_a² I)         (6×6)
   P_new      = F·P·F^T + G·Q·G^T
   ```

4. **Bias Jacobian propagation** (lines 323-330):
   ```
   J_P_bg += J_V_bg · dt − 0.5 · R · [a]_× · dt²
   J_P_ba += J_V_ba · dt + 0.5 · R · dt²
   J_V_bg += −R · [a]_× · dt
   J_V_ba += R · dt
   J_R_bg  = dR_step^T · J_R_bg − Jr_inv · dt          (Jr_inv ≈ I − ½ [ω·dt]_×)
   ```
   Note: **no `J_R_ba`** — rotation does not depend on accel bias. Right-Jacobian inverse approximated to first order at lines 328-329.

**Madgwick filter** (`IMUPreintegrator.cpp:670-850`): IMU-only branch (no magnetometer). `q̇_ω = ½ · q ⊗ (0, gx, gy, gz)`; accel correction via gradient of gravity-alignment objective (eq. 25 of Madgwick 2010), normalised, scaled by `MADGWICK_BETA=0.033`. Gates: only when `|a| ∈ [5, 20]` m/s²; `dt ∈ (0, 0.1]` s; catastrophic-renorm reset at `qNormSq ≤ 1e-9` (line 847-849).

---

## 6. Inertial initialization

### 6.1 `InertialInitializer` FSM

States (`InertialInitializer.h:27-32`): `WAIT_STATIONARY=0` → `WAIT_MOTION=1` → `READY=2`; or `TIMEOUT_NEEDS_USER=3` on timeout.

**`WAIT_STATIONARY`** (`InertialInitializer.cpp:115-145`):
- Buffer accel+gyro samples; trim to `stationary_seconds=5.0` window.
- When `(window.back.ts − window.front.ts) ≥ 5e9 ns`, run `runStationaryCalibration`.
- If accept → `WAIT_MOTION`. If reject → keep sliding.
- If `elapsed > 15.0 s` → `TIMEOUT_NEEDS_USER`.

**`runStationaryCalibration`** (`InertialInitializer.cpp:154-243`):
1. Compute `mean_a, mean_g`.
2. Variances `var_a = (1/N)·Σ‖aᵢ−mean_a‖²`, `var_g = (1/N)·Σ‖gᵢ−mean_g‖²`.
3. Accept if `var_a ≤ 0.05` AND `var_g ≤ 0.001`. Comment at lines 175-179: variance, not mean magnitude — the mean **is** the bias and can exceed 0.02 rad/s on real phones (key bug fix).
4. `force_accept_` (set by `clearTimeout`) bypasses both gates (lines 188-194).
5. **Initial rotation** (lines 208-231): `z_axis = mean_a / ‖mean_a‖`; `x_axis = (1,0,0)` (or `(0,1,0)` if `|z_axis.x| > 0.9`); `y_axis = z×x` orthonormalised; `x_axis = y×z`. `R_ItoG = [x|y|z]`; `R_GtoI_init_ = R_ItoG^T`.
6. `gyro_bias_ = mean_g`. `accel_bias_ = 0` because stationary alone cannot separate accel bias from gravity (lines 233-235).

**`WAIT_MOTION`** (`InertialInitializer.cpp:146-151`): `detectMotion(ax,ay,az) = |‖a‖−9.81| > 0.5` m/s² → `READY`.

### 6.2 Parallel gravity init in `IMUPreintegrator`

`IMUPreintegrator::tryInitializeGravityLocked` (`IMUPreintegrator.cpp:143-206`) runs independently:
- `GRAVITY_INIT_WINDOW = 40` accel samples.
- Gates: `var_mag ≤ GRAVITY_INIT_MAX_VAR = 0.08` AND `‖gyro‖ ≤ GRAVITY_INIT_GYRO_MAX = 0.12`.
- Sets `gravity_vec_`, `roll_ = atan2(gy,gz)`, `pitch_ = atan2(−gx, √(gy²+gz²))`. (DEAD CODE: `getRoll/getPitch/getGravityVector` never called; `getFilteredGravity` is the live channel.)

### 6.3 Parallel gyro-bias init in `IMUPreintegrator`

`tryInitializeGyroBiasLocked` (`IMUPreintegrator.cpp:900-941`):
- Wait until `accel_variance_est_ ≤ 0.05` (stationary).
- Accumulate `GYRO_BIAS_INIT_SAMPLES = 200` quiet samples.
- Average most recent 200 gyro samples → `gyro_bias_`.

`refineGyroBiasDuringZUPT` (`IMUPreintegrator.cpp:976-1002`) blends with `α = 0.01` over last 20 samples.

---

## 7. Frame conventions

**World**: ENU Z-up. `g = (0, 0, -9.81)` at `EKFState.cpp:140`. Yaw is rotation about world-Z, nav convention (CW-positive, North=0, East=+π/2), wraps `[-π, π]`. Madgwick uses same Z-up world (`IMUPreintegrator.cpp:677`).

**Body / IMU**: `R_GtoI_` is **world→body**. For body at heading ψ with zero roll/pitch (`EKFState.h:418-420`):
```
R_GtoI_ = [[cos ψ, -sin ψ, 0],
           [sin ψ,  cos ψ, 0],
           [   0,      0, 1]]
```
`getYaw` extracts ψ via `atan2(R[1,0], R[0,0])` (`EKFState.cpp:1172-1173`). Pre-2026-05-07 form `atan2(-R[0,2], R[0,0])` was Y-up, reverted (`EKFState.cpp:1166-1171`).

**Camera**: `R_bc` is body→camera, `p_cam = R_bc · p_body`. Default `diag(1,-1,-1)` for rear camera, vertical phone (`EKFState.h:466-468`, `EKFState.cpp:53-55`).

**Error-state perturbation**: δθ is **world-frame, left-multiply** (`EKFState.cpp:670, 1078-1102`). ∂yaw/∂δθ is constant `(0,0,1)` — fixed 2026-05-09.

**Madgwick yaw bridge**: `getHeading() = -yaw_math` (`IMUPreintegrator.cpp:858-871`); `setInitialMadgwickYaw(azimuth_nav)` stores `pending_madgwick_yaw_nav_` then re-inits with `yaw_math = -azimuth_nav` (`IMUPreintegrator.cpp:708-713, 734-748`).

**Gravity sign**: world `-Z` direction at 9.81 m/s² (gravity acceleration). Body-frame measured accelerometer reads `+9.81` along the body axis aligned with world `+Z` (i.e. when phone is held screen-up flat, body-Z is up, accel reads `(0,0,+9.81)`; this matches the `filtered_gravity_` initial value in `IMUPreintegrator.h:182`).

---

## 8. Interactions with other subsystems

### 8.1 Direct callers (from `app/src/main/cpp` grep)

| Caller | Usage |
|--------|-------|
| `Tracker.cpp / Tracker.h` | Owns `IMUPreintegrator imu_pre_` and `EKFState ekf_`. Drives every measurement update. |
| `VioEngine.cpp/.h` | Wraps `Tracker`, exposes JNI surface. |
| `native-lib.cpp` | JNI bridge; instantiates `InertialInitializer`; forwards sensor data. |
| `UpdaterMSCKF.cpp/.h` | Legacy/mostly-disabled MSCKF feature path. `EKFState::getFEJ` is DEAD CODE only used by it (`EKFState.h:302-303`). |
| `UpdaterZeroVelocity.cpp/.h` | Calls `ekf_.updateZUPT()`. |
| `ScaleEstimatorVI.cpp/.h` | External scale-fusion logic; in-EKF `scale_/P_scale_` is the legacy path. |
| `FeatureManager.h` | Per-feature observation buffers; pushes into `applyMSCKFFeature` / `updateSlamFeature`. |
| `EventCounters.h` | Telemetry: `msckf_update_lines`, `msckf_huber_rejected_sum`, `loop_closure_chi2_rejected`, `extrinsics_rotation_angle_mdeg`, `cam_imu_time_offset_us` (incremented from `applyMSCKFUpdate`, `EKFState.cpp:776-796`). |

### 8.2 Outward surface

From **`EKFState`**: `getRotation/getPosition/getYaw` (UI / map matching); `getCovariance/getCloneSnapshot` (BA / diagnostics); `getStateDim/getCloneCovIdx/getClonePose/getCloneFEJ/getLatestCloneId/getCloneTimestamp` (UpdaterMSCKF / loop closure / SLAM); `getTimeOffset/getTimeOffsetStd/getExtrinsicsRotation/getExtrinsicsAngleDeg` (telemetry / JNI); `getMSCKFHuberRejectedCount` (logcat); `isFullInitialized` (gate).

From **`IMUPreintegrator`**: `integrate` (single feed for `propagateIMU`); `getFilteredGravity/getGyroBias/getStepInfo/getVehicleSpeedEstimate` (Tracker, UI hint, PDR); `getOrientationQuaternion/getHeading/getMadgwickRoll/Pitch` (UI compass, gravity-aligned yaw); `last{Accel,Gyro}{X,Y,Z}` (VioData passthrough); `setUserHeight/setUserStride/setMagnetometerHeading/setInitialMadgwickYaw` (calibration / bootstrap); `setNoiseParameters`; `refineGyroBiasDuringZUPT` (driven by Tracker's ZUPT detector).

From **`InertialInitializer`**: `addImuData` (sample sink); `getStatus/isReady/needsUserPrompt`; on `READY`, Tracker calls `ekf_.initializeFull(getInitialRotation(), getGyroBias(), getAccelBias())`; `loadCalibration` (startup hook from SharedPreferences); `clearTimeout` (UI dialog dismissal).

### 8.3 Threading

- `EKFState::snapshot_mutex_` (mutable) — guards `window_` against torn reads from BA worker (`EKFState.h:437-442`). All mutating clone ops (`addClone, pruneWindow, marginalizeOldestClone, reset`) hold it; `getCloneSnapshot` locks for the read.
- Other `EKFState` fields are single-threaded (camera thread).
- `IMUPreintegrator::mutex_` (mutable) — guards every buffer / state member.
- `InertialInitializer::mutex_` (mutable) — guards FSM, windows, biases.
- Atomics for lock-free polling: `gravity_initialized_`, `gyro_bias_initialized_`, `madgwick_init_`, `has_mag_heading_`.

---

## 9. Workarounds, magic numbers, red flags

1. **Velocity hard-clamp to 5 m/s** in `propagateIMU` (`EKFState.cpp:288-291`) — pedestrian gating; masks runaway velocity from bad updates rather than fixing root cause.
2. **`R_bc` EKF update is currently SKIPPED** (`EKFState.cpp:699-704`); `H_bc` is also disabled in `applyMSCKFFeature` (`EKFState.cpp:1961-1967`) and `slamReprojectionJacobian` (`EKFState.cpp:1821-1823`). Comment: clones bake `R_bc` at storage time; treating it as a free state would double-count it. Net effect: δφ_bc rows of the state exist but receive only process noise — pure dead weight.
3. **OpenCV 4.5.3 unary-minus bug workaround**: `mat * -1.0` instead of `-mat` (`EKFState.cpp:1716-1727`). Without it every frame after 12 SLAM features were promoted aborted via `__cxa_throw` from `checkOperandsExist`.
4. **MSCKF damping ramp 0.5→1.0 over 5 frames** (`EKFState.cpp:575-582`, ADR-008) — absorbs 5–11 m teleportations ADR-006 saw when MSCKF re-enabled after a quiet period.
5. **Loop-closure χ² threshold 22.5** is intentionally loose (`EKFState.cpp:996-1000, 920-924`) — loop closures fade in via damping, a tight gate would defeat that.
6. **SLAM augmentation has zero cross-correlation to existing state** (`EKFState.cpp:1432-1443`) — workaround that treats depth uncertainty as independent of IMU/clone errors at promotion.
7. **PSD fallback in `removeSlamFeature`** (`EKFState.cpp:1546-1562`) — drop-only if Schur over-corrects.
8. **Double gravity bootstrap**: `InertialInitializer` and `IMUPreintegrator::tryInitializeGravityLocked` run independently with different thresholds (`max_accel_var=0.05` vs `GRAVITY_INIT_MAX_VAR=0.08`) and window sizes (5 s vs 40 samples).
9. **`MotionMode`/walking classifier and vehicle-speed integrator are "legacy but live"** (`IMUPreintegrator.h:218-241`) — Step 8 cleanup target, "comment, don't delete" per user request.
10. **`HEADING_CORRECTION_DAMPING = 0.95f`** (`IMUPreintegrator.h:284`) is declared but `getCorrectedHeading` is dead code — magnetometer is consumed once at startup only.
11. **Window-size discrepancy**: `pruneWindow` default `11` (`EKFState.h:73`) vs `getCloneSnapshot` default `5` (`EKFState.h:300`) — two different "windows" (live MSCKF vs BA snapshot).

---

## 10. Quick reference / call graph

```
InertialInitializer::addImuData  ── once status == READY
        ↓
Tracker::full_init
        ↓
EKFState::initializeFull(R_GtoI, gyro_bias, accel_bias)   # 19×19 P_

per IMU window:
  IMUPreintegrator::integrate(t0, t1)                      # midpoint, SO(3) Rodrigues
        ↓ deltaR/deltaV/deltaP/cov/J_*_b*
EKFState::propagateIMU(...)                                # mean + Φ·P·Φ^T + Q

per camera frame:
EKFState::addClone(R_GtoC, p_G, ts)                        # splice BEFORE SLAM
EKFState::applyMSCKFFeature/updateRelativeRotation/
          updateRelativePose/updateGravityAlignedYaw/
          updatePDRStep                                    # measurement updates
EKFState::updateZUPT                                       # stationary
EKFState::updateAbsolutePose                               # loop closure, χ²=22.5

SLAM lifecycle:
  EKFState::addSlamFeature (inverse-depth promote, FEJ-locked)
  EKFState::updateSlamFeature
  EKFState::removeSlamFeature (Schur, PSD-sniff fallback)

IMUPreintegrator drives in parallel (per IMU sample):
  Madgwick attitude (β=0.033, accel gate [5,20] m/s²)
  Step detector (peak/valley, 0.8 rad/s rotation gate)
  Stationary gravity init (40 samples, var≤0.08, gyro≤0.12)
  Stationary gyro-bias init (200 samples, accel-var≤0.05)
```
