# 04 — EKF Updaters and Scale Estimation

> Read-only study of `UpdaterMSCKF`, `UpdaterZeroVelocity`, `ScaleEstimatorVI`, and `ScaleFuser`.
>
> Files studied:
> - `app/src/main/cpp/UpdaterMSCKF.h`
> - `app/src/main/cpp/UpdaterMSCKF.cpp`
> - `app/src/main/cpp/UpdaterZeroVelocity.h`
> - `app/src/main/cpp/UpdaterZeroVelocity.cpp`
> - `app/src/main/cpp/ScaleEstimatorVI.h`
> - `app/src/main/cpp/ScaleEstimatorVI.cpp`
> - `app/src/main/cpp/ScaleFuser.h`
> - `app/src/main/cpp/ScaleFuser.cpp`
>
> Cross-referenced for the EKF interface contract (`applyMSCKFUpdate`, state layout, Joseph form, Huber, damping):
> - `app/src/main/cpp/EKFState.h`
> - `app/src/main/cpp/EKFState.cpp`

---

## 1. MSCKF residual construction

The MSCKF updater consumes `LostFeature` tracks — feature observations across the sliding window of camera clones — and turns them into linear measurement constraints on the state without ever growing the state with feature parameters. The 3D point is marginalised analytically using the left null-space of `H_f`.

### 1.1 Inputs

`UpdaterMSCKF.h:10-22`:

```cpp
struct FeatureObservation {
    int clone_state_id;    // EKF clone state ID
    cv::Point2f pixel_ud;  // Undistorted normalized coordinates
};
struct LostFeature {
    int feature_id;
    std::vector<FeatureObservation> observations;  // At least 3 for MSCKF
};
```

Observations are already undistorted and in **normalized image coordinates** (`x = X/Z`, `y = Y/Z`). This is critical because:

1. Residuals are built directly as `obs.pixel_ud.x − u_pred` with `u_pred = X/Z` — no `K` ever multiplied in (`UpdaterMSCKF.cpp:107-108`).
2. `pixel_noise` is therefore in normalized units, not pixels. The header (`UpdaterMSCKF.h:41-50`) documents the bug history: default was `1.0` (≈ 800 px equivalent) which made the χ² gate effectively never reject; current default is `0.002` (= 1.5 px ÷ 800 fy).

### 1.2 Per-observation projection

For observation `i` with clone pose `(R_CtoG, p_C)` (world ← camera) the feature in the camera frame is

```
p_f_C = R_GtoC · (p_f − p_C)              (UpdaterMSCKF.cpp:93-94)
R_GtoC = R_CtoG.t()                       (line 93)
```

Predicted normalized projection:

```
u_pred = X / Z                            (line 103)
v_pred = Y / Z                            (line 104)
```

### 1.3 Per-feature residual (2M × 1)

```
res[2i  , 0] = obs.pixel_ud.x − X/Z       (UpdaterMSCKF.cpp:107)
res[2i+1, 0] = obs.pixel_ud.y − Y/Z       (UpdaterMSCKF.cpp:108)
```

Total residual length is `2M`.

### 1.4 Triangulation that produces `p_f`

DLT triangulation (`UpdaterMSCKF.cpp:18-70`). Per observation, build a 3×4 normalized projection matrix:

```
P = [R_GtoC | −R_GtoC · p_C]              (lines 33-37)
```

Stack two rows per observation:

```
A[2i,  :] = u · P[2,:] − P[0,:]           (line 44)
A[2i+1,:] = v · P[2,:] − P[1,:]           (line 45)
```

`cv::SVD(A, FULL_UV)`; solution is the homogenised last column of `V`:

```
pt3d = V[0..2, 3] / V[3, 3]               (lines 51-57)
```

with `|w| < 1e-10` rejecting (line 53), and any depth `Z ≤ 0.1 m` in any camera rejecting (line 66).

### 1.5 Null-space projection forms the actual residual the EKF sees

`H_f` (2M × 3) marginalises the 3D point. Its left null-space is applied to **both** `H_x` and `res`:

```
H_o = Q2ᵀ · H_x         (UpdaterMSCKF.cpp:165)
r_o = Q2ᵀ · res         (UpdaterMSCKF.cpp:166)
```

Resulting dimensions: `(2M − 3) × state_dim` for `H_o`, `(2M − 3) × 1` for `r_o`. With `min_obs = 3` each surviving feature contributes 3 measurement rows.

### 1.6 H_f formula

```
dz/dp_f_C = [ 1/Z   0    −X/Z²
              0    1/Z   −Y/Z² ]              (UpdaterMSCKF.cpp:111-113)

H_f_i = (dz/dp_f_C) · R_GtoC                  (UpdaterMSCKF.cpp:117)
```

(uses `dp_f_C / dp_f_G = R_GtoC`).

### 1.7 Stacked residual returned to EKFState

After per-feature null-space + χ² gate (§3), each `(H_o, r_o)` is pushed (`UpdaterMSCKF.cpp:231-233`) and concatenated into `(total_rows × state_dim)` and `(total_rows × 1)`. If `total_rows > state_dim`, an SVD-based **measurement compression** truncates to `rank = state_dim` rows (`UpdaterMSCKF.cpp:250-257`):

```
SVD(H_all) → u, s, v
Q1 = u[:, 0:rank]
H_all ← Q1ᵀ · H_all
r_all ← Q1ᵀ · r_all
```

That compressed pair is what `applyMSCKFUpdate` finally processes.

---

## 2. MSCKF Jacobians: H_x, H_f, null-space projection

`getFeatureJacobian` (`UpdaterMSCKF.cpp:74-140`) builds for `M` observations of one feature:

```
H_f : 2M × 3                      (line 81)
H_x : 2M × state_dim              (line 82)
res : 2M × 1                      (line 83)
```

### 2.1 FEJ — First-Estimate Jacobians

Jacobian uses **clone pose at first-estimate time** (`UpdaterMSCKF.cpp:90-91`):

```cpp
state.getCloneFEJ(obs.clone_state_id, R_CtoG, p_C);
```

This is the standard observability fix from Hesch / Huang et al. The residual itself is computed against the **live** clone pose geometry (line 107-108 logic) but the linearisation is around the FEJ pose.

> Caveat: in this file `getCloneFEJ` is called without checking its return value (compare `triangulate` at line 29). If the clone has been evicted between feature triangulation and Jacobian construction the matrices may be in an invalid state. Production `EKFState` guards via `getCloneCovIdx` (line 134).

### 2.2 Per-observation H_x rows

Two H_x rows per observation are written into the **clone slot** (`UpdaterMSCKF.cpp:134`):

```cpp
clone_idx = state.getCloneCovIdx(obs.clone_state_id);
```

If `clone_idx < 0` no H_x rows are written for that observation. The clone slot has `CLONE_DIM = 6` columns: `[δθ_c (3), δp_c (3)]` (per `EKFState.h:394`).

| Column block | Formula | Code |
|---|---|---|
| `[clone_idx, clone_idx+3)` (δθ_c) | `H_θ = (dz/dp_f_C) · ⌊p_f_C⌋_×` | `UpdaterMSCKF.cpp:129, 136` |
| `[clone_idx+3, clone_idx+6)` (δp_c) | `H_p = (dz/dp_f_C) · (−R_GtoC)` | `UpdaterMSCKF.cpp:131, 137` |

Skew matrix manually constructed (`UpdaterMSCKF.cpp:123-126`):

```
skew_pfc = [  0  −Z   Y
              Z   0  −X
             −Y   X   0 ]
```

There are **no** rows in H_x for the IMU state itself or for SLAM features. MSCKF is purely a clone-pose constraint here.

### 2.3 Null-space projection (`nullspaceProject`)

`UpdaterMSCKF.cpp:144-170`. The header says "QR" but the code uses SVD (OpenCV does not expose a direct Householder QR):

```
SVD(H_f, FULL_UV) → u, s, vt    (line 155)
Q2 = u[:, 3 : 2M]               (line 162)
H_x ← Q2ᵀ · H_x                 (line 165)
res ← Q2ᵀ · res                 (line 166)
```

`u` is `(2M × 2M)`; columns 0..2 span the column space of `H_f`, columns 3..2M-1 the **left null-space**. Multiplying by `Q2ᵀ` removes the 3 feature DOF.

The early-return guard `if (H_f.rows <= 3) return;` is duplicated at lines 150 and 160. With `min_obs = 3` (`H_f.rows = 6`) projection always runs.

---

## 3. MSCKF chi² gate

Per-feature gate uses the projected residual against its predicted covariance (`UpdaterMSCKF.cpp:214-229`):

```
P_sub = P[0:H_x.cols, 0:H_x.cols]      (line 219)
S = H_x · P_sub · H_xᵀ                 (line 220)
σ²    = pixel_noise²                    (line 221)
S    += σ² · I                          (line 222)
S⁻¹ via Cholesky                        (line 225)
χ² = resᵀ · S⁻¹ · res                   (line 227-228)
reject if χ² > getChi2Threshold(dof)    (line 229)
```

with `dof = H_x.rows = 2M − 3`.

`getChi2Threshold` (`UpdaterMSCKF.cpp:174-185`) is a fixed table of χ² 0.95 critical values for dof 1..20, then a normal-approximation for dof > 20:

```
table[1..20] = { 3.841, 5.991, 7.815, 9.488, 11.070, 12.592,
                 14.067, 15.507, 16.919, 18.307, 19.675, 21.026,
                 22.362, 23.685, 24.996, 26.296, 27.587, 28.869,
                 30.144, 31.410 }
threshold(dof>20) = dof + 1.645 · √(2·dof)
```

The whole threshold is multiplied by `options_.chi2_multiplier` (default `1.5`), so the effective gate is at α ≈ 0.985–0.99 rather than 0.95. Documented rationale (`UpdaterMSCKF.h:53`): give margin for noisy phone features.

What gets rejected:

- Triangulation failure — DLT degenerate, behind-camera (lines 53, 66, 203).
- `H_x.rows == 0` after null-space (line 212) — defensive; cannot happen with `min_obs = 3`.
- Covariance not yet sized for the clones (line 217).
- `cv::invert` Cholesky failure on `S` (line 225) — soft reject.
- χ² exceeds the (multiplied) threshold (line 229).
- Downstream, `applyMSCKFUpdate` runs a **per-row Huber kernel** that can additionally zero out individual measurement rows (`EKFState.cpp:606-635`).

---

## 4. MSCKF update step

`processLostFeatures` does not itself touch covariance; it prepares `(H_all, r_all, R_noise)` and hands them to `EKFState::applyMSCKFUpdate` (`UpdaterMSCKF.cpp:259-267`):

```
σ²      = pixel_noise²                              (line 260)
R_noise = σ² · I_{total_rows}                       (line 261)
state.applyMSCKFUpdate(H_all, r_all, R_noise);      (line 262)
```

The gain / state correction / covariance update live in `EKFState.cpp:584-797`. They are documented here because they are the contract MSCKF relies on.

### 4.1 Pre-Kalman: per-row Huber kernel (ADR-008)

`EKFState.cpp:593-635`: before computing `S`, every row gets an IRLS weight, with `δ = MSCKF_HUBER_DELTA = 2.4477` = √χ²(0.95, 2 dof) (`EKFState.h:503`).

```
S_pre = H · P · Hᵀ + R_noise
m_i   = |r_i| / √S_pre[i,i]
w_i   = { 1            if m_i ≤ δ
        { δ / m_i      if δ < m_i < 3δ
        { 0            if m_i ≥ 3δ        (hard reject)
H_w[i, :] *= w_i ;  res_w[i] *= w_i
```

Counters: `msckf_huber_rejected_count_` for hard rejects, `huber_dampened` for the de-weighted middle band.

### 4.2 Kalman gain (`EKFState.cpp:638-646`)

```
S = H_w · P · H_wᵀ + R_noise
K = P · H_wᵀ · S⁻¹       (Cholesky preferred; SVD fallback)
```

If both inversions fail the update is silently skipped (line 643).

### 4.3 State correction (with damping on δp only)

```
dx = K · res_w               (line 649)
```

Position correction (rows 12..14 = δp) is multiplied by `damping = computeMSCKFDampingFactor()` (`EKFState.cpp:657-662`), ramping 0.5 → 1.0 over `MSCKF_DAMPING_RAMP_FRAMES = 5` calls (`EKFState.h:500`). Other blocks are not damped.

State application (`EKFState.cpp:664-705`):

| `dx` rows | Slot | Update rule |
|---|---|---|
| 0..2 | δθ (world→IMU rot.) | `R_GtoI ← Rodrigues(dθ) · R_GtoI` (left-mult) |
| 3..5 | b_g | `b_g += dx[3..5]` |
| 6..8 | v_G | `v_G += dx[6..8]` |
| 9..11 | b_a | `b_a += dx[9..11]` |
| 12..14 | p_G | `p_G += dx[12..14]` (× damping) |
| 15 | t_offset_cam_imu | `+=`, clamped to ±0.1 s |
| 16..18 | δφ_bc | **skipped** (clones bake R_bc; no measurement coupling) |
| `IMU_STATE_DIM + i·CLONE_DIM` … | clone i | `R_GtoC ← Rodrigues(dθ_c) · R_GtoC`, `p += dp_c` |
| SLAM block | (α, β, ρ, pad0, pad1) | additive |

`IMU_STATE_DIM = 19` (`EKFState.h:387`), `CLONE_DIM = 6` (`EKFState.h:394`), `SLAM_FEATURE_DIM = 5` (`EKFState.h:105`).

### 4.4 Covariance update — Joseph form

`EKFState.cpp:742-744`:

```
I_KH = I − K · H_w
P    = I_KH · P · I_KHᵀ + K · R_noise · Kᵀ
```

This is the symmetric **Joseph form**, numerically robust to small `K` errors. Symmetry is then re-enforced (`EKFState.cpp:747`):

```
P = (P + Pᵀ) · 0.5
```

Damping schedule advances:

```
msckf_frames_since_call_ = 0;                    (line 751)
if (msckf_damping_step_ < MSCKF_DAMPING_RAMP_FRAMES)
    msckf_damping_step_++;                       (lines 752-753)
```

### 4.5 Return value

`processLostFeatures` returns the count of features that survived triangulation, null-space projection, and the χ² gate (`UpdaterMSCKF.cpp:264-266`).

---

## 5. Zero-velocity detection (ZUPT gating)

`UpdaterZeroVelocity` does not produce a residual — it is purely a **gate** that returns `true` when the IMU + visual stream looks stationary. The actual ZUPT measurement update is applied elsewhere when this gate fires.

### 5.1 Inputs (`UpdaterZeroVelocity.h:34-36`)

```cpp
bool is_stationary(const std::vector<AccelSample>& accel_window,
                   const std::vector<GyroSample>& gyro_window,
                   double visual_disparity);
```

`AccelSample` / `GyroSample` come from `IMUPreintegrator.h` (each holds `x, y, z` plus a timestamp).

### 5.2 Length check

Window must be at least `Options.window_size = 20` samples (`UpdaterZeroVelocity.cpp:17-20`). At ≈ 200 Hz IMU, this is ~100 ms of data. Reads the **last** `window_size` samples (`start_idx_a/g = size - N`) → naturally a sliding tail.

### 5.3 Visual gate (early reject)

`UpdaterZeroVelocity.cpp:23-25`:

```
if (visual_disparity > options_.max_disparity) return false;
```

Default `max_disparity = 1.5` px (`UpdaterZeroVelocity.h:25`). History: was `1.0`; raised because KLT is not perfectly stable even when stationary.

### 5.4 Gyro variance test (`UpdaterZeroVelocity.cpp:32-52`)

```
mean_g = (1/N) · Σ g_i
sum_sq_g = Σ ‖g_i − mean_g‖²
χ²_g    = sum_sq_g / σ_g²
```

with `σ_g = options_.sigma_g = 0.025 rad/s`. Was `0.005`; raised for phone-grade gyros.

### 5.5 Accelerometer variance test (`UpdaterZeroVelocity.cpp:55-70`)

```
mean_a = (1/N) · Σ a_i
sum_sq_a = Σ ‖a_i − mean_a‖²
χ²_a    = sum_sq_a / σ_a²
```

with `σ_a = options_.sigma_a = 0.15 m/s²`.

### 5.6 Gravity magnitude check (`UpdaterZeroVelocity.cpp:73-75`)

```
‖mean_a‖ ∈ [g − 3σ_a, g + 3σ_a]
gravity_ok = |‖mean_a‖ − 9.81| < 0.45 m/s²
```

Rejects free-fall, sustained linear acceleration, and impacts even when variance is small.

### 5.7 Combined verdict (`UpdaterZeroVelocity.cpp:78-80`)

```
threshold = chi2(0.95, 3·(N−1)) · chi2_multiplier
stationary = (χ²_g < threshold) ∧ (χ²_a < threshold) ∧ gravity_ok
```

For `N = 20` dof = 57, χ²(0.95, 57) ≈ 75.6 (Wilson-Hilferty), then ×3.0 ≈ 227.

### 5.8 Wilson-Hilferty inverse CDF (`UpdaterZeroVelocity.cpp:90-97`)

```
F⁻¹(0.95; k) = k · (1 − 2/(9k) + 1.645·√(2/(9k)))³
```

Closed form, works for arbitrary dof.

### 5.9 Logging behaviour

Only emits a logcat line when stationary or `χ²_g < 2·threshold` (i.e. close). Silences during normal walking (`UpdaterZeroVelocity.cpp:82-85`).

---

## 6. ZUPT measurement model — what the gate enables

`UpdaterZeroVelocity` is not the EKF update; it returns a boolean. The measurement applied when the gate is `true` is the standard ZUPT:

```
z = 0_{3×1}              (we observe v_G = 0)
H : 3 × state_dim with +I on rows 6..8 (v_G slot)
res = 0 − v_G_estimate = −v_G
R   = σ_v² · I_3
```

That residual is fed into `EKFState::applyMSCKFUpdate(H, res, R)` (or an equivalent Joseph-form helper). The Joseph covariance update shrinks the velocity covariance and — through correlations in `P` — the position and bias blocks. **ZUPT does not zero position** — only velocity — and bias drift is corrected indirectly via cross terms in `P`.

The Updater itself does not pick `σ_v` — the caller does. Empirically the project uses ≈ 0.01 m/s for handheld walking ZUPTs. The actual ZUPT call sites live in tracker / EKF glue code outside the four files in this study.

---

## 7. Visual-inertial scale estimation (Hesch / Martinelli closed form)

`ScaleEstimatorVI` solves the metric scale of a scale-ambiguous visual trajectory in closed form using IMU preintegration. It is the "Observer C" of the Step 3 plan.

### 7.1 Equation (`ScaleEstimatorVI.h:6-20`)

Per consecutive keyframe pair `i`:

```
s · R_w_b_i · t_vis_i  =  R_w_b_i · Δp_i  +  v_i · Δt_i  +  ½ · g · Δt_i²

where v_i = v_0 + γ_i,
      γ_i = Σ_{k<i} (R_w_b_k · Δv_k + g · Δt_k),  γ_0 = 0
```

Rearranged for unknowns `x = [s, v_0_x, v_0_y, v_0_z]ᵀ`:

```
A_i = [ R_w_b_i · t_vis_i   |   −Δt_i · I_3 ]      (3 × 4)
b_i = R_w_b_i · Δp_i  +  γ_i · Δt_i  +  ½ · g · Δt_i²    (3 × 1)
```

Stacking N pairs gives a `3N × 4` ordinary-least-squares system.

### 7.2 Inputs per `KeyframePair` (`ScaleEstimatorVI.h:30-43`)

| Field | Type | Units | Meaning |
|---|---|---|---|
| `R_w_b` | `cv::Mat` (3×3 CV_64F) | unitless | attitude at keyframe i (world←body) |
| `t_vis_body` | `cv::Vec3d` | VIO units (scale-ambiguous) | visual displacement i→i+1 in body frame at i |
| `delta_p_body` | `cv::Vec3d` | m | IMU-preint Δp over `[t_i, t_{i+1}]` (body i) |
| `delta_v_body` | `cv::Vec3d` | m/s | IMU-preint Δv over `[t_i, t_{i+1}]` (body i) |
| `dt` | `double` | s | time gap |

### 7.3 Buffer policy (`ScaleEstimatorVI.cpp:9-16`)

```
addKeyframePair(p):
  pairs_.push_back(p);
  if (pairs_.size() > MAX_PAIRS) drop oldest down to MAX_PAIRS;
```

`MIN_PAIRS = 4`, `MAX_PAIRS = 16` (`ScaleEstimatorVI.h:68-70`). With 4 pairs the system is 12 equations × 4 unknowns — comfortably overdetermined.

### 7.4 Solve algorithm (`ScaleEstimatorVI.cpp:38-155`)

1. **Compute γ_i** for `i = 1..N-1` (forward integration, `cpp:47-53`):

   ```
   γ_0 = 0
   γ_i = γ_{i-1} + R_w_b_{i-1} · Δv_{i-1} + g · Δt_{i-1}
   ```

2. **Build normal equations** AᵀA (4×4) and Aᵀb (4×1) by accumulating per-pair 3×4 blocks (`cpp:65-108`):

   ```
   a_col = R_w_b · t_vis_body
   dpw   = R_w_b · Δp_body
   b_i   = dpw + γ_i · Δt + 0.5 · g · Δt²
   A_i   = [ a_col | −Δt · I_3 ]
   AᵀA  += A_iᵀ A_i
   Aᵀb  += A_iᵀ b_i
   ```

3. **Solve** with `cv::solve(..., DECOMP_SVD)` for robustness near collinear translations (`cpp:110-121`):

   ```
   x  = (AᵀA)⁻¹ Aᵀb
   s  = x[0]
   v0 = x[1..3]
   ```

4. **Sanity bounds** on scale (`cpp:127-129`):

   ```
   if !isfinite(s) || s < 0.01 || s > 10.0  ⇒  return false
   ```

   These match the `ScaleFuser` clamp range exactly.

5. **Residual variance** (`cpp:131-143`):

   ```
   r_i = A_i · x − b_i
   rss = Σ ‖r_i‖²
   dof = 3N − 4
   σ²  = rss / dof
   ```

6. **Variance of s** (`cpp:145-149`):

   ```
   var(s) = σ² · (AᵀA)⁻¹[0,0]
   ```

   Reject if non-finite or negative.

7. **Outputs**: write `scale_out`, `variance_out`, optionally `v0_out`. Return `true` (`cpp:150-154`).

### 7.5 Why this isn't always on

Header (`ScaleEstimatorVI.h:21-25`): disabled in Phase 8 because IMU-preint ΔP was dominated by gravity-subtraction error — i.e. attitude was wrong. With Madgwick (Step 1) the gravity term is clean and the system becomes well-conditioned. The file just provides the math; the gating happens upstream.

### 7.6 Concurrency

A single `mutex_` guards `pairs_` and `gravity_w_` for both writers (`addKeyframePair`, `setGravity`, `reset`) and the reader path (`solve`, `size`, `getGravity`).

---

## 8. ScaleFuser — 1-D Kalman filter on metric scale

`ScaleFuser` is a scalar Kalman filter that fuses `(z, r)` measurements from any number of scale observers — PDR, MiDaS, the VI closed-form.

### 8.1 Model (`ScaleFuser.h:5-26`)

```
State:    s     ∈ [SCALE_MIN, SCALE_MAX] = [0.01, 10.0]
Cov:      P
Predict:  s ← s,   P ← P + q · Δt        with q = PROCESS_NOISE_PER_SEC = 1e-5
Update:
   K = P / (P + r)
   s ← s + K · (z − s)
   P ← (1 − K) · P
```

### 8.2 Predict (`ScaleFuser.cpp:10-14`)

```
predict(dt):
  if !isfinite(dt) || dt <= 0 : return
  P_ += PROCESS_NOISE_PER_SEC · dt
```

`PROCESS_NOISE_PER_SEC = 1e-5` (`ScaleFuser.h:51`). Header note: variance grows ~3.6e-2 per hour, so a long-stationary observer slowly loses weight relative to fresh ones.

### 8.3 Update (`ScaleFuser.cpp:16-26`)

```
update(z, r):
  if !isfinite(z) || !isfinite(r) || r <= 0 : return false
  if z < SCALE_MIN || z > SCALE_MAX        : return false
  K = P / (P + r)
  s = s + K · (z − s)
  P = (1 − K) · P
  s = clamp(s, SCALE_MIN, SCALE_MAX)
  if !isfinite(P) || P < 0 : P = 0
  return true
```

Observers feed `(z, r)` (`ScaleFuser.h:20-23`):

| Observer | z | r |
|---|---|---|
| PDR | `stride_pdr / vo_dist` | from step-period variance |
| MiDaS | `current_scale · median_ratio` | from MAD/N |
| VI (Hesch/Martinelli) | `s` from `ScaleEstimatorVI::solve` | `(AᵀA)⁻¹[0,0] · σ²_resid` |

A measurement with very large `r` produces `K ≈ 0` — that observer contributes nothing without explicit "fallback" logic. Designed dead-observer fade-out.

### 8.4 Initial conditions (`ScaleFuser.h:29-30`, `.cpp:6-8`)

```
ScaleFuser(initial_scale = 0.20, initial_variance = 1.0)
  s_ = clamp(initial_scale, SCALE_MIN, SCALE_MAX)
  P_ = max(0.0, initial_variance)
```

`reset(...)` re-applies the same construction (`ScaleFuser.cpp:38-42`). Default 0.20 corresponds to the typical handheld initialization scale.

### 8.5 Concurrency

`mutable std::mutex mutex_` guards every public method, including `predict` (`.cpp:12`).

---

## 9. Public functions — exact signatures, math, side-effects

### 9.1 `UpdaterMSCKF`

| Function | Signature | Math / Behaviour | Side-effects |
|---|---|---|---|
| `UpdaterMSCKF(const Options&)` | `UpdaterMSCKF.h:57` | stores options | none |
| `processLostFeatures(state, lost_features, fx, fy, cx, cy)` | `UpdaterMSCKF.h:66-68` ; impl `UpdaterMSCKF.cpp:189-267` | per feature: triangulate (DLT), build H_x/H_f/res, null-space project, χ²-gate; stack survivors; SVD-compress if rows > state_dim; call `state.applyMSCKFUpdate(H, r, σ²·I)` | mutates `state` (rotation, biases, velocity, position, time offset, clones, SLAM features) and `state.P_` via Joseph form |
| `triangulate` *(private)* | `UpdaterMSCKF.h:75-76` ; impl `UpdaterMSCKF.cpp:18-70` | DLT SVD; reject if homogeneous w too small or any depth ≤ 0.1 m | output `pt3d`; reads clone poses |
| `getFeatureJacobian` *(private)* | `UpdaterMSCKF.h:81-84` ; impl `UpdaterMSCKF.cpp:74-140` | builds H_f (2M×3), H_x (2M×state_dim), res (2M×1) using FEJ clone pose | writes into output mats |
| `nullspaceProject` *(static private)* | `UpdaterMSCKF.h:90` ; impl `UpdaterMSCKF.cpp:144-170` | SVD of H_f, multiply H_x and res by Q2ᵀ in place | mutates H_x and res |
| `getChi2Threshold(int dof)` *(private)* | `UpdaterMSCKF.h:95` ; impl `UpdaterMSCKF.cpp:174-185` | table for dof≤20, normal-approx otherwise; multiplied by `chi2_multiplier` | pure |

> Note: `processLostFeatures` declares `fx, fy, cx, cy` but the implementation never uses them — residuals are in normalized coordinates and the K matrix is implicit. The parameters are kept on the API for forward compatibility.

### 9.2 `UpdaterZeroVelocity`

| Function | Signature | Math / Behaviour | Side-effects |
|---|---|---|---|
| `UpdaterZeroVelocity(const Options&)` | `UpdaterZeroVelocity.h:29` | stores options | none |
| `is_stationary(accel, gyro, disparity)` | `UpdaterZeroVelocity.h:34-36` ; impl `UpdaterZeroVelocity.cpp:14-88` | length check → disparity gate → χ²_g, χ²_a → gravity check → Wilson-Hilferty threshold | pure (no member writes); emits a logcat line when stationary or near it |
| `get_chi2_threshold(int dof)` *(private)* | `UpdaterZeroVelocity.h:42` ; impl `UpdaterZeroVelocity.cpp:90-97` | Wilson-Hilferty `k·(1 − 2/(9k) + 1.645√(2/(9k)))³` | pure |

### 9.3 `ScaleEstimatorVI`

| Function | Signature | Math / Behaviour | Side-effects |
|---|---|---|---|
| `ScaleEstimatorVI()` | `ScaleEstimatorVI.h:28` ; impl `.cpp:5-7` | reserves capacity | none |
| `addKeyframePair(pair)` | `ScaleEstimatorVI.h:46` ; impl `.cpp:9-16` | append, drop oldest beyond MAX_PAIRS | mutates `pairs_` |
| `solve(scale_out, variance_out, v0_out=nullptr)` | `ScaleEstimatorVI.h:52-53` ; impl `.cpp:38-155` | builds AᵀA / Aᵀb, solves via `cv::solve` SVD, residual variance, variance of s | writes outputs; reads `pairs_`, `gravity_w_` |
| `size()` | `ScaleEstimatorVI.h:56` ; impl `.cpp:18-21` | locked read of `pairs_.size()` | none |
| `setGravity(g)` | `ScaleEstimatorVI.h:59` ; impl `.cpp:23-26` | locked write | mutates `gravity_w_` |
| `getGravity()` | `ScaleEstimatorVI.h:60` ; impl `.cpp:28-31` | locked read | none |
| `reset()` | `ScaleEstimatorVI.h:63` ; impl `.cpp:33-36` | clear pairs | mutates `pairs_` |

### 9.4 `ScaleFuser`

| Function | Signature | Math / Behaviour | Side-effects |
|---|---|---|---|
| `ScaleFuser(initial_scale=0.20, initial_variance=1.0)` | `ScaleFuser.h:29-30` ; impl `.cpp:6-8` | clamp s to range, max(0, P) | sets `s_`, `P_` |
| `predict(dt_seconds)` | `ScaleFuser.h:33` ; impl `.cpp:10-14` | `P += q · dt` if dt finite & > 0 | mutates `P_` |
| `update(z, r)` | `ScaleFuser.h:37` ; impl `.cpp:16-26` | scalar Kalman; rejects on bad z / non-positive r / out-of-range z | mutates `s_`, `P_`; returns true on success |
| `scale()` | `ScaleFuser.h:39` ; impl `.cpp:28-31` | locked read | none |
| `variance()` | `ScaleFuser.h:40` ; impl `.cpp:33-36` | locked read | none |
| `reset(initial_scale=0.20, initial_variance=1.0)` | `ScaleFuser.h:43` ; impl `.cpp:38-42` | re-apply construction values | mutates `s_`, `P_` |

---

## 10. Member variables — type, units, defaults, accessors

### 10.1 `UpdaterMSCKF`

| Member | Type | Units | Default | Reader | Writer |
|---|---|---|---|---|---|
| `options_` | `Options` | — | — | private | ctor |
| `Options::chi2_multiplier` | `double` | unitless | `1.5` | `getChi2Threshold` | ctor (inline init `UpdaterMSCKF.h:53`) |
| `Options::min_obs` | `int` | observations | `3` | `triangulate`, `processLostFeatures` | ctor |
| `Options::max_reproj_px` | `double` | px | `5.0` | *(declared, not read in this file)* | ctor |
| `Options::pixel_noise` | `double` | normalized image units (= px / focal) | `0.002` | `processLostFeatures` (χ² gate, R_noise) | ctor |

### 10.2 `UpdaterZeroVelocity`

| Member | Type | Units | Default | Reader | Writer |
|---|---|---|---|---|---|
| `options_` | `Options` | — | — | private | ctor |
| `Options::window_size` | `int` | samples | `20` | `is_stationary` | ctor |
| `Options::sigma_g` | `double` | rad/s | `0.025` | `is_stationary` | ctor |
| `Options::sigma_a` | `double` | m/s² | `0.15` | `is_stationary` | ctor |
| `Options::chi2_multiplier` | `double` | unitless | `3.0` | `is_stationary` | ctor |
| `Options::max_disparity` | `double` | pixels | `1.5` | `is_stationary` | ctor |
| `Options::gravity_mag` | `double` | m/s² | `9.81` | `is_stationary` | ctor |

(All defaults at `UpdaterZeroVelocity.h:20-26`.)

### 10.3 `ScaleEstimatorVI`

| Member | Type | Units | Default | Reader | Writer |
|---|---|---|---|---|---|
| `mutex_` | `mutable std::mutex` | — | — | every public method | every public method |
| `pairs_` | `std::vector<KeyframePair>` | mixed | empty (cap = `MAX_PAIRS`) | `solve`, `size` | `addKeyframePair`, `reset` |
| `gravity_w_` | `cv::Vec3d` | m/s² | `(0, 0, −9.81)` | `solve`, `getGravity` | `setGravity` |

Static constants:

| Name | Type | Value | Source |
|---|---|---|---|
| `MIN_PAIRS` | `size_t` | `4` | `ScaleEstimatorVI.h:68` |
| `MAX_PAIRS` | `size_t` | `16` | `ScaleEstimatorVI.h:70` |

### 10.4 `ScaleFuser`

| Member | Type | Units | Default | Reader | Writer |
|---|---|---|---|---|---|
| `mutex_` | `mutable std::mutex` | — | — | every public method | every public method |
| `s_` | `double` | unitless ratio (m / VIO-unit) | `0.20` | `scale()`, `update()` | ctor, `update()`, `reset()` |
| `P_` | `double` | (scale)² | `1.0` | `variance()`, `update()`, `predict()` | ctor, `update()`, `predict()`, `reset()` |

Static constants:

| Name | Type | Value | Source |
|---|---|---|---|
| `SCALE_MIN` | `double` | `0.01` | `ScaleFuser.h:46` |
| `SCALE_MAX` | `double` | `10.0` | `ScaleFuser.h:47` |
| `PROCESS_NOISE_PER_SEC` | `double` | `1e-5` | `ScaleFuser.h:51` |

---

## 11. Interactions with `EKFState`

`UpdaterZeroVelocity`, `ScaleEstimatorVI`, and `ScaleFuser` do not touch `EKFState` directly. Only `UpdaterMSCKF` invokes the EKF mutating API.

### 11.1 Read path (`UpdaterMSCKF`)

| Call | Where | What is read |
|---|---|---|
| `state.getClonePose(id, R, p)` | triangulation `UpdaterMSCKF.cpp:29, 62` | live clone pose `(R_CtoG, p_C)` |
| `state.getCloneFEJ(id, R, p)` | Jacobian `UpdaterMSCKF.cpp:91` | first-estimate clone pose for FEJ |
| `state.getStateDim()` | `UpdaterMSCKF.cpp:79, 239` | `19 + 6·n_clones + 5·n_slam` |
| `state.getCloneCovIdx(id)` | `UpdaterMSCKF.cpp:134` | column offset of clone in P / dx |
| `state.getCovariance()` | χ² gate `UpdaterMSCKF.cpp:216` | `P` (cloned). Used to compute `S = H·P·Hᵀ + R` |

### 11.2 Write path (`UpdaterMSCKF`)

A single call writes everything (`UpdaterMSCKF.cpp:262`):

```cpp
state.applyMSCKFUpdate(H_all, r_all, R_noise);
```

Which (per `EKFState.cpp:584-797`) modifies:

| State slot | Cov rows/cols | Mutated? |
|---|---|---|
| `R_GtoI_` (δθ rows 0..2) | 0..2 | yes (`Rodrigues(dθ) · R_GtoI`) |
| `b_g_` | 3..5 | yes |
| `v_G_` | 6..8 | yes |
| `b_a_` | 9..11 | yes |
| `p_G_` | 12..14 | yes (× damping ramp) |
| `t_offset_cam_imu_` | 15 | yes (clamped ±0.1 s) |
| `R_bc_` (δφ_bc) | 16..18 | **no** — defensive skip; clones bake R_bc |
| each clone `(R_GtoC, p_G)` | `19 + 6i .. 19 + 6i+5` | yes |
| each SLAM feature `(α, β, ρ, pad0, pad1)` | tail of P, 5 rows each | yes (additive) |
| Full `P_` | all | yes — Joseph `P = (I − KH_w)·P·(I − KH_w)ᵀ + K·R·Kᵀ`, then symmetrised |

### 11.3 Scale-side state contract

`ScaleFuser` and `ScaleEstimatorVI` are **not** part of the error state. Their outputs feed the PDR/MiDaS/VIO scale fusion that scales visual translations before they reach `EKFState::updateRelativePose` (`EKFState.cpp:801-832`) — itself a wrapper around `applyMSCKFUpdate`. So the scale subsystem indirectly affects the EKF through the metric translations it enables, but does not touch any P block directly.

---

## 12. Magic numbers and thresholds — full enumeration

### 12.1 `UpdaterMSCKF`

| Value | Where | Meaning |
|---|---|---|
| `1.5` | `UpdaterMSCKF.h:53` | default `Options::chi2_multiplier` |
| `3` | `UpdaterMSCKF.h:53` | default `Options::min_obs` |
| `5.0` | `UpdaterMSCKF.h:54` | default `Options::max_reproj_px` (declared, not currently read) |
| `0.002` | `UpdaterMSCKF.h:54` | default `Options::pixel_noise` in normalized units (≈ 1.5 px ÷ 800 fy) |
| `1e-10` | `UpdaterMSCKF.cpp:53` | DLT homogeneous w cutoff |
| `0.1` | `UpdaterMSCKF.cpp:66` | min depth in camera frame (m) — behind-camera reject |
| `H_f.rows ≤ 3` | `UpdaterMSCKF.cpp:150, 160` | null-space skipped if too few observations |
| `1.645` | `UpdaterMSCKF.cpp:184` | 95th-percentile of standard normal (chi² normal-approx) |
| `table[1..20]` | `UpdaterMSCKF.cpp:178-182` | χ²(0.95, dof) for dof=1..20 |

### 12.2 `UpdaterZeroVelocity`

| Value | Where | Meaning |
|---|---|---|
| `20` | `UpdaterZeroVelocity.h:21` | default `window_size` (samples) |
| `0.025` | `UpdaterZeroVelocity.h:22` | default `sigma_g` (rad/s) |
| `0.15` | `UpdaterZeroVelocity.h:23` | default `sigma_a` (m/s²) |
| `3.0` | `UpdaterZeroVelocity.h:24` | default `chi2_multiplier` |
| `1.5` | `UpdaterZeroVelocity.h:25` | default `max_disparity` (px) |
| `9.81` | `UpdaterZeroVelocity.h:26` | default `gravity_mag` (m/s²) |
| `3.0` | `UpdaterZeroVelocity.cpp:75` | gravity ‖a‖ window = ±3·σ_a |
| `2.0` | `UpdaterZeroVelocity.cpp:82` | "near-threshold" multiplier for logging gate |
| `1.645` | `UpdaterZeroVelocity.cpp:93` | Wilson-Hilferty z for p = 0.95 |
| `9.0` | `UpdaterZeroVelocity.cpp:95` | Wilson-Hilferty `9k` denominator |
| `2.0` | `UpdaterZeroVelocity.cpp:95` | Wilson-Hilferty `2/(9k)` numerator |

### 12.3 `ScaleEstimatorVI`

| Value | Where | Meaning |
|---|---|---|
| `−9.81` | `ScaleEstimatorVI.h:75` | default `gravity_w_.z` (m/s², Z-down per header) |
| `4` (`MIN_PAIRS`) | `ScaleEstimatorVI.h:68` | minimum pairs to attempt solve |
| `16` (`MAX_PAIRS`) | `ScaleEstimatorVI.h:70` | rolling buffer cap |
| `0.5` | `ScaleEstimatorVI.cpp:83` | `½ · g · Δt²` factor |
| `0.01` | `ScaleEstimatorVI.cpp:129` | scale lower sanity bound |
| `10.0` | `ScaleEstimatorVI.cpp:129` | scale upper sanity bound |
| `3N − 4` | `ScaleEstimatorVI.cpp:142` | residual variance dof |

### 12.4 `ScaleFuser`

| Value | Where | Meaning |
|---|---|---|
| `0.20` | `ScaleFuser.h:29` | default initial scale |
| `1.0` | `ScaleFuser.h:30` | default initial variance |
| `0.01` (`SCALE_MIN`) | `ScaleFuser.h:46` | lower clamp |
| `10.0` (`SCALE_MAX`) | `ScaleFuser.h:47` | upper clamp |
| `1e-5` (`PROCESS_NOISE_PER_SEC`) | `ScaleFuser.h:51` | scale process noise rate (s⁻¹) |

### 12.5 EKF interface constants used by the MSCKF update path

> Defined in `EKFState.h`; govern how `applyMSCKFUpdate` interprets MSCKF rows.

| Value | Where | Meaning |
|---|---|---|
| `IMU_STATE_DIM = 19` | `EKFState.h:387` | `[δθ(3) δb_g(3) δv(3) δb_a(3) δp(3) δt_d(1) δφ_bc(3)]` |
| `CLONE_DIM = 6` | `EKFState.h:394` | `[δθ_c(3) δp_c(3)]` per clone |
| `SLAM_FEATURE_DIM = 5` | `EKFState.h:105` | `[α, β, ρ, pad0, pad1]` |
| `MSCKF_DAMPING_RAMP_FRAMES = 5` | `EKFState.h:500` | δp damping ramps 0.5 → 1.0 over 5 calls |
| `MSCKF_HUBER_DELTA = 2.4477` | `EKFState.h:503` | √χ²(0.95, 2 dof); robust kernel inner radius |
| `±0.1` | `EKFState.cpp:682` | `t_offset_cam_imu_` clamp (±100 ms) |
| `500 µs` | `EKFState.cpp:766` | gated PERF logging threshold |

---

## Appendix A — Cross-references

- ADR-002 — ESKF, not full EKF (justifies error-state form).
- ADR-008 — MSCKF re-enabled with damping + Huber (matches §4.1, §4.3).
- ADR-009 — SLAM features in state (justifies `SLAM_FEATURE_DIM` block).
- ADR-013 — same-session loop closure (`updateAbsolutePose`, another consumer of `applyMSCKFUpdate`).
- Geneva et al., ICRA 2020 (left-perturbation convention; see `EKFState.cpp:687-695`).
- Mourikis & Roumeliotis, ICRA 2007 (original MSCKF, null-space form).
- Hesch et al. / Martinelli (closed-form VI scale, §7).

---

## Notable findings flagged by the agent

- `UpdaterMSCKF::getFeatureJacobian` calls `state.getCloneFEJ` without checking its bool return (`UpdaterMSCKF.cpp:91`); compare the guarded `getClonePose` calls in `triangulate` (lines 29, 62). Probably benign because `getCloneCovIdx(...)` at line 134 also gates the H_x write, but worth noting.
- `UpdaterMSCKF::processLostFeatures` accepts `fx, fy, cx, cy` but never uses them — residuals are normalized. Dead parameters kept on the API.
- `UpdaterMSCKF.cpp:144-170` is named `nullspaceProject` and the doc comment says QR, but the implementation uses SVD (OpenCV exposes no direct Householder QR). The duplicate guard `if (rows <= 3)` appears at lines 150 and 160.
- The δφ_bc rows 16..18 are intentionally left zero in `applyMSCKFUpdate` (`EKFState.cpp:699-704`) because clones bake R_bc; the MSCKF Jacobian therefore has no 16..18 entries either.
- `pixel_noise = 0.002` is in **normalized** image units, not pixels — was the bug fixed pre-2026-05-09 (header comment `UpdaterMSCKF.h:41-50`).
- `ScaleEstimatorVI` uses gravity `(0, 0, −9.81)` (`ScaleEstimatorVI.h:75`) — Z-down convention, separate from the Z-up world frame fix on `morad`. If callers don't pass the right gravity via `setGravity`, b_i is wrong. **Worth checking the caller.**
- `ScaleFuser` is fully scalar — no cross-coupling with EKF P. Indirect EKF coupling is via the metric scale applied to visual translations before `updateRelativePose`.
