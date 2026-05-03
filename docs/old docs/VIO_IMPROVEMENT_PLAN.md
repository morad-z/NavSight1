# NavSight1 VIO Improvement Plan
## Based on OpenVINS Architecture Study + Gap Analysis

**Date:** 2026-04-06  
**Scope:** Concrete improvements to reduce drift, ranked by impact vs effort

---

## Current System Summary

NavSight1 uses: Pyramid LK optical flow + Essential matrix + gyro-camera fusion + scalar EKF for scale + IMU dead reckoning fallback. Step detector provides the only scale source.

**Measured performance:**
- Outdoors walking: ~2-3% position error / 10 min
- Indoors walking: ~5-8% position error / 10 min  
- Indoors stationary: unbounded drift (no scale source)
- Heading drift: ~140 deg/min (magnetometer fusion exists but is NEVER CALLED)

---

## Phase 1: Quick Wins (1-2 hours, massive impact)

### 1.1 Enable Magnetometer Heading Fusion
**Impact: 7/10 | Effort: 1/10**

`IMUPreintegrator::getCorrectedHeading()` exists (lines 638-662) with damping=0.95 but Tracker.cpp line 577 never calls it.

**Fix:** In `Tracker.cpp` section 9 (global pose update), after computing heading from `global_R_`:
```cpp
// Current (line ~577):
out.heading = std::atan2(global_R_.at<double>(1,0), global_R_.at<double>(0,0));

// Change to:
double raw_heading = std::atan2(global_R_.at<double>(1,0), global_R_.at<double>(0,0));
out.heading = imu.getCorrectedHeading(raw_heading);
```

**Expected:** Heading drift bounded to <5 deg/min instead of ~140 deg/min.

> **Note:** Per project memory, magnetometer should only be used at startup for initial heading. If the user confirms this policy, skip this fix and instead ensure initial heading uses mag, then gyro-only during tracking.

### 1.2 Unify Gyro Bias (Remove Tracker's Redundant Estimate)
**Impact: 8/10 | Effort: 3/10**

Currently TWO separate gyro bias estimates exist:
- `IMUPreintegrator`: 200 stationary samples at startup (lines 584-625)
- `Tracker::gyro_bias_`: vision-gyro fusion (lines 361-368), exponential moving average

These can diverge. OpenVINS uses a single unified bias in the EKF state.

**Fix:** In Tracker.cpp, replace the local `gyro_bias_` update (lines 361-375) with reading the bias from IMUPreintegrator:
```cpp
// Remove self-computed gyro_bias_ update
// Instead, pull from IMU's authoritative bias:
gyro_bias_ = imu.getGyroBias(); // Use IMUPreintegrator's estimate
```

### 1.3 Tighten Scale Rate Limit
**Impact: 5/10 | Effort: 1/10**

Mapper.cpp line 265: max scale change = 15% per 2 seconds. VINS-Mono uses 5%.

**Fix:** Change to 5% per 1 second:
```cpp
// Was:  max_rate = 0.15, window = 2.0s
// Now:
max_rate = 0.05;
window = 1.0;
```

---

## Phase 2: Statistical ZUPT (2-3 hours, high impact)

### Current Problem
NavSight1 uses hardcoded thresholds: `gyro_norm < 0.04 rad/s`. This is device-dependent and brittle.

### OpenVINS Approach
- Buffer last **20 IMU samples** (configurable)
- Compute variance of gyro norms and accel norms
- Check both IMU variance AND visual disparity (< 1.0 pixel)
- Chi-squared gate: `chi2 = r^T * S^{-1} * r` compared to `chi2_table[3]` = **7.815** (95% confidence, 3 DOF)
- Pseudo-measurement: `z = 0` with H pointing to velocity state columns

### Implementation Plan
**File:** `Tracker.cpp` section 3 (static detection)

```cpp
// Replace:
bool is_static = (mean_flow < 0.5) && (gyro_norm < ZUPT_GYRO_THRESH);

// With variance-based detection:
// 1. Buffer last 20 IMU gyro+accel readings
// 2. Compute gyro_variance = var(||w||) over buffer
// 3. Compute accel_variance = var(||a|| - g) over buffer  
// 4. Statistical test:
bool imu_static = (gyro_variance < imu_noise_g * imu_noise_g * chi2_95_3dof)
               && (accel_variance < imu_noise_a * imu_noise_a * chi2_95_3dof);
bool visual_static = (mean_flow < 1.0);  // OpenVINS default
bool is_static = imu_static && visual_static;
```

**Key parameters from OpenVINS:**
| Parameter | OpenVINS Default | NavSight1 Current |
|-----------|-----------------|-------------------|
| IMU buffer size | 20 samples | N/A (single threshold) |
| Gyro threshold | variance-based | 0.04 rad/s norm |
| Accel threshold | 0.1 m/s^2 | N/A |
| Visual disparity | 1.0 pixel | 0.5 pixel mean_flow |
| Chi-squared (3 DOF, 95%) | 7.815 | N/A |

---

## Phase 3: Camera-IMU Time Offset Estimation (4-6 hours)

### Current Problem
NavSight1 assumes perfect camera-IMU sync. Android latency is 5-50ms. A 20ms offset at walking speed (1.4 m/s) = 2.8cm systematic position error per frame, compounding.

### OpenVINS Approach
- State variable `t_d` (scalar) added to EKF
- Jacobian: `dz/dt_d = (dz/dp_cam) * v_cam` (velocity-scaled)
- Initial sigma: 0.01 s
- Converges in 10-30 seconds of motion
- Becomes unobservable at low speeds (should freeze)

### Implementation Plan
**Files:** `EKFState.h/cpp`, `Tracker.cpp`

1. Add `double t_offset_cam_imu = 0.0` to EKFState (with P_td = 0.01^2)
2. In Tracker::processFrame, shift IMU integration target:
   ```cpp
   int64_t adjusted_cam_ts = frame_timestamp_ns + (int64_t)(ekf_.t_offset * 1e9);
   ```
3. Cross-correlate optical flow magnitude with gyro magnitude over 2-second window during warmup to get initial estimate
4. Online refinement: augment scale EKF Jacobian with velocity term

**Simpler alternative (startup-only):** Cross-correlate camera optical flow rate with gyro rate over first 2 seconds. Find lag that maximizes correlation. Set as fixed offset. No online update needed.

---

## Phase 4: Feature Track Aging + Chi-Squared Gating (3-4 hours)

### 4.1 Feature Track Aging
**Current:** All features weighted equally. No track history.
**OpenVINS:** Features tracked across multiple clones. Long-lived features (5+ frames) are more reliable.

**Implementation:**
```cpp
// FeatureManager.h — add:
std::vector<int> feature_ages_;  // Track age per feature

// Tracker.cpp — after optical flow:
// Increment ages for surviving features
// Reset to 0 for newly detected features
// In triangulation: weight features by min(age, 10) / 10.0
```

### 4.2 Reprojection Error Gating
**Current:** RANSAC essential matrix only (one-shot). No per-point gating after pose recovery.
**OpenVINS:** Chi-squared test per feature. `chi2_table[2]` = 5.991 at 95%.

**Implementation:**
After recovering R,t from essential matrix, for each inlier:
```cpp
// Reproject point using recovered pose
cv::Mat pt3d = triangulate(pt_prev, pt_curr, R, t);
cv::Mat reproj = K * (R * pt3d + t);
double err = norm(pt_curr - reproj.xy());
if (err > 2.0) discard;  // Hard threshold, or chi-squared with pixel noise sigma
```

---

## Phase 5: First-Estimate Jacobians (1 day)

### Current Problem
EKFState updates scale with H=[1] Jacobian at current estimate. No FEJ. In multi-state systems this causes filter overconfidence on unobservable directions (yaw, absolute position).

### OpenVINS Approach
Each clone stores TWO values:
- `_pose->value()` — current estimate (updated normally)
- `_pose->fej()` — first estimate (frozen at clone creation)

Jacobians in MSCKF updates use `fej()` not `value()`.

### Implementation for NavSight1
Since NavSight1 has only a scalar scale EKF (not full pose EKF), FEJ is less critical but still useful:

```cpp
// EKFState.h — add:
double scale_fej_ = -1.0;  // First-estimate of scale, set once

// EKFState.cpp — in updateScale():
if (scale_fej_ < 0) scale_fej_ = scale_;  // Lock first estimate
double innov = measurement - scale_fej_;   // Innovation w.r.t. FEJ, not current
// ... rest of Kalman update uses innov
```

For heading: store `initial_yaw_` when global_R_ is first set. Use for gravity-related Jacobians.

---

## Phase 6: Gravity-Based Scale Constraint (1 week, architectural)

### Current Problem
Scale is ONLY observable from step detector. No walking = no scale. Camera+IMU coupling is fundamentally broken because they're decoupled by design.

### OpenVINS Approach (MSCKF)
Scale emerges from tightly-coupling IMU preintegration with visual triangulation:
```
s * p_ij_visual = alpha_ij + R_i * v_0 * dt - 0.5 * g * dt^2
```
Where `alpha_ij` is IMU position preintegration, `g` is gravity. When device rotates, gravity direction changes in camera frame, creating a scale-observable constraint.

### Implementation Plan (Simplified for NavSight1)
This is the hardest change — requires moving from "scale EKF + separate pose" to a coupled state.

**Option A: Gravity constraint as measurement (moderate effort)**
- After triangulating points, compute expected depth from IMU acceleration
- Use as additional scale measurement: `scale_gravity = ||accel_delta|| * dt^2 / (2 * ||visual_disp||)`
- Feed into existing scalar EKF with high uncertainty

**Option B: Full MSCKF (major refactor)**
- Replace scalar EKF with 15-state error-state EKF: [orientation, position, velocity, gyro_bias, accel_bias]
- Add sliding window of camera clones (11 clones per OpenVINS)
- Implement null-space projection for feature marginalization
- **This is essentially rebuilding the estimator from scratch**

**Recommendation:** Option A first (2-3 days), evaluate improvement, then decide if Option B is worth the effort for a final project.

---

## Phase 7: Sliding Window Marginalization (1 week, architectural)

### Current Problem
Keyframes discarded after buffer of 10. No information propagation. Past constraints lost.

### OpenVINS Approach
FIFO marginalization via Schur complement:
```
P_marg = P_rr - P_rm * P_mm^{-1} * P_mr
```
Where `m` = marginalized clone, `r` = remaining states.

### Implementation
Requires dense covariance matrix for all clones. Major architectural change from current frame-to-frame tracking. Defer to Phase 6 Option B if attempted.

---

## Priority Order for Implementation

| Priority | Fix | Time | Heading Drift | Scale Drift | Overall Impact |
|----------|-----|------|--------------|-------------|----------------|
| **P0** | Enable mag heading OR validate startup-only policy | 30 min | -95% | — | Huge |
| **P0** | Unify gyro bias | 1 hr | -40% | — | High |
| **P0** | Tighten scale rate limit | 15 min | — | -30% | Medium |
| **P1** | Statistical ZUPT | 3 hr | -20% | -15% | High |
| **P1** | Feature track aging | 2 hr | — | -20% | Medium |
| **P2** | Time-offset calibration (startup) | 4 hr | -15% | -10% | Medium |
| **P2** | Reprojection chi-squared gating | 2 hr | — | -15% | Medium |
| **P3** | First-Estimate Jacobians | 4 hr | -10% | -10% | Low-Med |
| **P3** | Gravity-based scale (Option A) | 3 days | — | -50% | High |
| **P4** | Full MSCKF rewrite | 2 weeks | -80% | -80% | Transformative |

---

## OpenVINS Key Parameters Reference

| Parameter | OpenVINS Value | NavSight1 Current | Recommended |
|-----------|---------------|-------------------|-------------|
| Sliding window clones | 11 | N/A (frame-to-frame) | 11 if MSCKF |
| Feature count | 200 (mono) | 400 max, 120 min | Keep 200 |
| KLT pyramid levels | 3 | 3 (via OpenCV) | Keep 3 |
| KLT window | 15x15 | 21x21 (default) | Reduce to 15 |
| ZUPT IMU buffer | 20 samples | 1 sample | 20 |
| ZUPT gyro threshold | variance-based | 0.04 rad/s | variance |
| ZUPT visual disparity | 1.0 px | 0.5 px mean_flow | 1.0 px |
| Chi-squared (3DOF, 95%) | 7.815 | N/A | 7.815 |
| Chi-squared (2DOF, 95%) | 5.991 | N/A | 5.991 |
| FB consistency | 1.0 px | 9.0 px^2 (3px) | 1.0-2.0 px |
| Time offset init sigma | 0.01 s | N/A (no td) | 0.01 s |
| RK4 integration | Yes | Rodrigues + Riemann | Keep (simpler) |
| FEJ on clones | Yes | No | Yes if MSCKF |
| IMU-aided KLT prediction | Yes (gyro warp) | No | Yes (P1) |

---

## References

- Geneva et al., "OpenVINS: A Research Platform for Visual-Inertial Estimation," ICRA 2020
- Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation," ICRA 2007 (MSCKF)
- Trawny & Roumeliotis, "Indirect Kalman Filter for 3D Attitude Estimation," 2005
- Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry," T-RO 2017
