# NavSight Visual Algorithms Reference

## How These Fit Together

Each camera frame enters `Tracker::processFrame` as raw YUV NV21 data. The Y plane is extracted and run through **CLAHE** histogram equalization before anything else touches it. **LensCorrector** then undistorts the matched point pairs (using zero-distortion coefficients until calibration is provided). **TrackKLT** runs pyramid Lucas-Kanade optical flow from the previous gray frame to the current one, using an IMU-rotation homography as the initial guess; a **forward-backward error check** immediately filters ambiguous tracks. The surviving point correspondences go to **geometricVerification**, which calls `findEssentialMat` + `recoverPose` (5-point RANSAC) to recover the inter-frame camera rotation and unit-norm translation. If translation is non-degenerate, **triangulatePoints** produces 3-D landmarks and a **reprojection gate** removes those with reprojection error above 5.991 px squared (chi-squared 95%). Rotation from the camera is discarded in favour of the IMU-preintegrated `deltaR`; only the heading is kept, corrected every ~15 frames by a **keyframe essential-matrix heading correction** that gravity-aligns the visual rotation before extracting yaw. Three independent scale observers -- **PDR** (step speed / VO displacement), **MiDaS depth** (floor-plane constraint), and **Hesch/Martinelli VI** (closed-form visual-inertial) -- each push a `(z, variance)` measurement into the **ScaleFuser** 1-D Kalman filter, whose output is `smooth_scale_`. The scaled translation is added to the global position only when the **motion classifiers** (pure-rotation detector, translation-degeneracy detector, ZUPT) all pass. The resulting pose, heading, and 3-D points are emitted in `VisionOutput` and `TrackerFrame` for consumers.

---

## 1. Image Preprocessing

### CLAHE Histogram Equalization

- **Location**: `app/src/main/cpp/Tracker.cpp:40` (constructor), `Tracker.cpp:413-419` (`Tracker::processFrame`)
- **What it does**: Applies Contrast Limited Adaptive Histogram Equalization to the grayscale frame before any feature tracking. Skipped entirely when scene brightness >= 0.55 (well-lit), saving ~2 ms/frame.
- **Why**: Low-contrast frames reduce `goodFeaturesToTrack` response and make KLT lose tracks faster. CLAHE lifts local contrast without over-amplifying noise in bright areas.
- **Parameters**: `clipLimit = 2.0`, `tileSize = 8x8` pixels. Brightness threshold for skipping: `0.55`. Low-light flag at `0.12`; quality scaled by 0.2x in that regime.
- **Status**: Active (conditional on brightness).
- **Consumers**: `gray_buf_` fed directly into `TrackKLT::track` and keyframe storage.

---

### Lens Correction (LensCorrector)

- **Location**: `app/src/main/cpp/LensCorrector.h:27`, `LensCorrector.cpp:27-38` (`LensCorrector::undistortMatchedPoints`)
- **What it does**: Calls `cv::undistortPoints` on both the previous and current matched point sets, re-projecting back into pixel space through the camera matrix. Default distortion coefficients are all zero (passthrough).
- **Why**: Barrel distortion introduces systematic radial displacement that the essential matrix misinterprets as rotation, causing heading drift. Comment in `LensCorrector.h:34`: "Wrong distortion coefficients are WORSE than no correction -- they corrupt point positions and cause essential matrix failure."
- **Parameters**: `k1 = k2 = p1 = p2 = k3 = 0.0` (defaults; no per-device calibration wired in). Dead code: `setDistortion` and single-set `undistortPoints` are commented out.
- **Status**: Active (zero-distortion passthrough until calibration is provided).
- **Consumers**: Output `prev_ud` / `next_ud` passed to `TrackKLT::geometricVerification` and `cv::triangulatePoints`.

---

## 2. Feature Tracking

### Grid-Based Feature Detection and Replenishment (FeatureManager)

- **Location**: `app/src/main/cpp/FeatureManager.cpp:39-85` (`detectGridFeatures`), `FeatureManager.cpp:89-140` (`replenishSparse`)
- **What it does**: Divides the image into a 4x5 grid and runs `cv::goodFeaturesToTrack` independently per cell, followed by `cv::cornerSubPix` refinement. `replenishSparse` only fills cells where the tracked count dropped below the per-cell quota.
- **Why**: Without grid enforcement, features cluster in high-texture regions and leave large image areas empty, degrading essential matrix conditioning and producing biased scale observations.
- **Parameters**: `GRID_ROWS = 4`, `GRID_COLS = 5` (20 cells). `MAX_FEATURES = 200`, `MIN_FEATURES = 80`, `QUALITY_LEVEL = 0.05`, `MIN_DIST = 10.0` px. Sub-pixel window: `5x5`, 20 iterations, `eps = 0.03`.
- **Status**: Active.
- **Consumers**: Detected points become `prev_pts_` fed to `TrackKLT::track`. New IDs from `FeatureManager::assignIds` feed the MSCKF observation log.

---

### KLT Optical Flow (TrackKLT)

- **Location**: `app/src/main/cpp/TrackKLT.cpp:16-78` (`TrackKLT::track`)
- **What it does**: Runs 4-level pyramid Lucas-Kanade optical flow (`cv::calcOpticalFlowPyrLK`) forward (prev to curr). An IMU-rotation homography `H = K * R * K_inv` warm-starts feature positions before the forward pass. A backward pass (curr to prev) is run immediately for the FB check.
- **Why**: Pure KLT with a stationary initial guess fails when the phone rotates quickly -- features jump outside the search window. The IMU warm-start reduces the effective search displacement to the parallax component.
- **Parameters**: `PYRAMID_LEVELS = 4`, `WINDOW_SIZE = 21` px (comment: "Was 31; 21 is enough for 640x480"), `eps = 0.01`, `max_count = 30`. `cv::OPTFLOW_LK_GET_MIN_EIGENVALS` rejects degenerate patches.
- **Status**: Active.
- **Consumers**: `status` vector and `next_pts_buf_` fed to filter step at `Tracker.cpp:552-567`. Surviving pairs become `prev_good_buf_` / `next_good_buf_`.

---

### Forward-Backward Error Check (FB Check)

- **Location**: `app/src/main/cpp/TrackKLT.cpp:46-67` (inside `TrackKLT::track`)
- **What it does**: After the forward KLT pass, tracks each found point backward from current to previous frame. Any point whose squared round-trip error exceeds the threshold is marked failed.
- **Why**: KLT occasionally converges to a wrong local minimum. The backward check verifies the correspondence is geometrically invertible. Threshold tightened from 9.0 to 4.0 to reduce ambiguous tracks reaching the essential matrix.
- **Parameters**: `FB_CHECK_THRESH = 4.0` (squared pixels, 2.0 px Euclidean). Matching constant in `Tracker.h:200`.
- **Status**: Active.
- **Consumers**: Sets `status[i] = 0` for failed points; excluded from surviving-points loop at `Tracker.cpp:552-567`.

---

### IMU-Guided Point Prediction (TrackKLT::predictPoints)

- **Location**: `app/src/main/cpp/TrackKLT.cpp:120-146` (`TrackKLT::predictPoints`)
- **What it does**: Projects each previous-frame point forward through the inter-frame rotation homography `H = K * delta_R * K_inv`. Passed as the initial guess via `cv::OPTFLOW_USE_INITIAL_FLOW`.
- **Why**: Reduces apparent pixel displacement KLT must search at high gyro rates. Falls back to stationary assumption when `delta_R` is identity.
- **Parameters**: `H = K * delta_R * K.inv()` where `delta_R` is `imu_delta.deltaR`. Degenerate `w` check: `|w| > 1e-7`.
- **Status**: Active (called when `delta_R` is non-identity).
- **Consumers**: Predicted points as `curr_pts` initial guess to `cv::calcOpticalFlowPyrLK`.

---

## 3. Geometric Estimation

### Essential Matrix Estimation (findEssentialMat)

- **Location**: `app/src/main/cpp/TrackKLT.cpp:100-117` (`TrackKLT::geometricVerification`); also `Tracker.cpp:1151-1154` (keyframe heading correction path)
- **What it does**: Runs the 5-point algorithm inside RANSAC on undistorted matched pairs to find the essential matrix `E`. The RANSAC inlier mask is propagated back to update `status`.
- **Why**: Removes false tracks that survived KLT and the FB check but are geometrically inconsistent with rigid camera motion.
- **Parameters**: `RANSAC_CONF = 0.999` (TrackKLT), `RANSAC_THRESH = 1.5` px (comment: "0.5 too tight for uncalibrated"). `MIN_INLIERS = 8`. Keyframe path: `RANSAC_CONF = 0.9999`, same threshold.
- **Status**: Active.
- **Consumers**: `E` passed directly to `cv::recoverPose`.

---

### Pose Recovery (recoverPose)

- **Location**: `app/src/main/cpp/TrackKLT.cpp:107` (frame-to-frame), `Tracker.cpp:1154` (keyframe-to-current)
- **What it does**: Decomposes `E` into four (R, t) solutions, selects the one with most in-front triangulated points, and returns camera rotation and a **unit-norm** translation vector.
- **Why**: Gives the sign-corrected rotation and translation direction. Scale is unobservable from unit-norm `t_vo` alone. Comment at `Tracker.cpp:784`: "Yaw is UNOBSERVABLE from monocular camera (OpenVINS)." `R_vo` is discarded in favour of `imu_delta.deltaR`.
- **Parameters**: Returns `inliers_count` used for `MIN_INLIERS = 8` gate (frame) and `inl >= 20` gate (keyframe correction).
- **Status**: Active.
- **Consumers**: `t_vo` (unit-norm) feeds translation-degeneracy check, triangulation, PDR scale observer, and Observer C. `R_vo` intentionally discarded.

---

### Triangulation (cv::triangulatePoints)

- **Location**: `app/src/main/cpp/Tracker.cpp:735-778` (inside `processFrame`)
- **What it does**: Uses the direct linear transform to triangulate 3-D landmarks from undistorted matched pairs and relative camera pose `P1 = K*[I|0]`, `P2 = K*[R_vo|t_vo]`. Points with `w < 1e-6` stored as `(0,0,0)`.
- **Why**: Produces `points_3d_current_` needed for the MiDaS depth-scale constraint and Mapper. Only runs when translation is non-degenerate.
- **Parameters**: Guarded by `!translation_degenerate && !t_vo.empty()`. Invalid threshold: `w > 1e-6`.
- **Status**: Active (guarded).
- **Consumers**: `points_3d_current_` fed to `applyDepthScaleConstraint`. `TrackerFrame::points_3d` exported to Mapper.

---

### Reprojection Outlier Gate

- **Location**: `app/src/main/cpp/Tracker.cpp:745-778` (post-triangulation)
- **What it does**: Re-projects each triangulated point into the second camera frame and computes squared pixel error. Points with error > 5.991 are stored as `(0,0,0)` (marked invalid).
- **Why**: Triangulation amplifies noise at short baselines or near-epipolar features. The gate prevents geometrically-inconsistent 3-D points from biasing the MiDaS depth constraint.
- **Parameters**: Threshold = `5.991` px squared (chi-squared, 2 DOF, 95th percentile).
- **Status**: Active.
- **Consumers**: Filtered `points_3d_current_` consumed by `applyDepthScaleConstraint` and Mapper.

---

## 4. Motion Classification

### Translation-Degeneracy Detector

- **Location**: `app/src/main/cpp/Tracker.cpp:726-729` (inside `processFrame`)
- **What it does**: Checks if `t_vo` has near-zero norm (`< 0.001`) or if both mean flow is low (`< 1.5` px) and `t_norm < 0.01`. Either sets `translation_degenerate = true`.
- **Why**: When the camera is not translating, the essential matrix is degenerate and `recoverPose` returns an arbitrary `t_vo`. Comment: "Do NOT use SVD condition of E -- essential matrix is always rank 2 by definition (sigma, sigma, 0)."
- **Parameters**: `t_norm < 0.001`; `mean_flow < 1.5 && t_norm < 0.01`.
- **Status**: Active.
- **Consumers**: Guards triangulation, PDR scale, Observer C. Reflected in `out.poseFlags` bit 1.

---

### Pure-Rotation Detector

- **Location**: `app/src/main/cpp/Tracker.cpp:542` (inside `processFrame`)
- **What it does**: Computes IMU rotation magnitude over the frame interval via `cv::Rodrigues` + `cv::norm / dt`. If it exceeds `GYRO_ROT_ONLY_THRESH`, the frame is classified as pure-rotation.
- **Why**: During fast turns, unit-norm `t_vo` has high directional uncertainty. Scale observations and Observer C pairs are gated off.
- **Parameters**: `GYRO_ROT_ONLY_THRESH = 2.0` rad/s. Step-fallback also blocked when `gyro_norm >= 0.8` rad/s.
- **Status**: Active.
- **Consumers**: Blocks PDR scale observation, Observer C pair recording, and translation update. Reflected in `out.poseFlags` bit 2.

---

### ZUPT -- Zero Velocity Update (UpdaterZeroVelocity)

- **Location**: `app/src/main/cpp/UpdaterZeroVelocity.h`, called at `Tracker.cpp:680-691`
- **What it does**: Chi-squared test over a sliding IMU window checking accel/gyro variance against stationary noise floors and visual disparity against `max_disparity`. IMU-driven, not visual, but gates visual pipeline steps.
- **Why**: Prevents dead-reckoning drift while the user stands still; freezes translation and triggers gyro bias refinement.
- **Parameters**: `window_size = 20`, `sigma_g = 0.025`, `sigma_a = 0.15`, `chi2_multiplier = 3.0`, `max_disparity = 1.5` px. Override: forced false when `mean_flow > 2.5`.
- **Status**: Active (sensor-side; not strictly visual).
- **Consumers**: `is_static` flag blocks all translation updates; triggers `ekf_.updateZUPT()` and `imu.refineGyroBiasDuringZUPT()`.

---

## 5. Keyframe Management

### Keyframe Storage (FeatureManager::storeKeyframe)

- **Location**: `app/src/main/cpp/FeatureManager.cpp:144-160` (`storeKeyframe`), called from `Tracker.cpp:1246-1254`
- **What it does**: Saves the current gray frame, tracked points, timestamp, frame ID, scalar heading, and global position into a rolling buffer of up to 10 keyframes.
- **Why**: Provides reference frames for heading-drift correction and emergency re-localization.
- **Parameters**: `MAX_KEYFRAMES = 10`. Store interval: every 15 frames, or earlier if `tracked < MIN_FEATURES / 2 && frames_since_keyframe_ > 3`.
- **Status**: Active.
- **Consumers**: `FeatureManager::matchAgainstKeyframe` (re-localization and heading correction paths).

---

### Keyframe Matching (FeatureManager::matchAgainstKeyframe)

- **Location**: `app/src/main/cpp/FeatureManager.cpp:221-256` (`matchAgainstKeyframe`)
- **What it does**: Runs `cv::calcOpticalFlowPyrLK` from the stored keyframe gray image to the current frame using the keyframe tracked points as seeds. Matches jumping > 150 px are rejected. Returns true only if >= 20 matches are found.
- **Why**: Provides wider-baseline correspondences for heading-drift essential matrix estimation and emergency re-localization.
- **Parameters**: `MIN_KF_MATCHES = 20`, `KF_MATCH_RADIUS = 15.0` px (hard rejection at 150 px). LK window `21x21`, 3 pyramid levels, 20 iterations.
- **Status**: Active.
- **Consumers**: Heading correction path (`Tracker.cpp:1144-1156`, requires >= 30 matches). Re-localization path (`Tracker.cpp:1098-1107`).

---

### Keyframe Heading Drift Correction (Steps 2.1 / 2.2)

- **Location**: `app/src/main/cpp/Tracker.cpp:1124-1241` (inside `processFrame`, under mutex)
- **What it does**: Every ~15 frames, matches current frame against the last keyframe, runs `findEssentialMat` + `recoverPose` on wider-baseline pairs, then gravity-aligns the visual rotation `R_kf` via `R_aligned = R_align * R_kf * R_align.t()` where `R_align = Ry(pitch) * Rx(roll)` from Madgwick. Extracts yaw as `atan2(R_aligned[1,0], R_aligned[0,0])`. Applies 30% of the drift between visual and gyro heading change to `heading_offset_`.
- **Why**: Gyroscope bias causes slow heading drift. A 30% partial correction from the keyframe essential matrix catches this drift. Gravity alignment ensures extracted yaw is physically meaningful regardless of phone tilt.
- **Parameters**: `frames_since_keyframe_ >= 14`, `tracked >= 16`, keyframe match count >= 30, RANSAC inliers >= 20. Correction fraction: `0.30`. Drift gate: `|drift| < 20 deg`.
- **Status**: Active (Steps 2.1 + 2.2).
- **Consumers**: `heading_offset_` and `scalar_heading_` updated in place. `last_visual_yaw_variance_` written for Step 2.4 / Step 6 ESKF.

---

### Visual Yaw Covariance (Step 2.4)

- **Location**: `app/src/main/cpp/Tracker.cpp:1187-1201`; accessor `Tracker.h:64-66`
- **What it does**: Computes yaw measurement variance as `sigma_yaw_sq = (1 / (focal * sqrt(N)))^2` where `N` is the RANSAC inlier count. Stored in `last_visual_yaw_variance_`, exposed via `getLastVisualYawVariance()`.
- **Why**: Provides a properly-scaled uncertainty for the ESKF Step 6 heading update. Frames with more inliers receive higher EKF weight.
- **Parameters**: Implicit pixel noise = `1.0` px. Formula: `sigma_yaw = 1.0 / (focal * sqrt(inl))`. Fires only when `inl >= 20`.
- **Status**: Active (computed whenever keyframe heading correction fires with sufficient inliers).
- **Consumers**: `EKFState` Step 6 ESKF heading update (via `Tracker::getLastVisualYawVariance()`).

---

## 6. Scale Estimation

### Observer A -- PDR Scale (Step Speed / VO Displacement)

- **Location**: `app/src/main/cpp/Tracker.cpp:796-890` (inside `processFrame`)
- **What it does**: Divides the IMU pedestrian step displacement (`speed_mps * dt`) by unit-norm VO displacement (`cv::norm(t_vo)`) to get the frame metric scale. Phase 1 (first 15 observations) seeds `ScaleFuser` with the bootstrap median. Phase 2 computes per-observation variance from step-period jitter and passes `(obs_scale, r_var)` to `scale_fuser_.update`.
- **Why**: Resolves monocular scale ambiguity using pedestrian gait. The bootstrap median prevents early noisy frames from corrupting the long-run estimate.
- **Parameters**: `SCALE_BOOTSTRAP_COUNT = 15`. Phase 2 outlier gate: `2.5x` ratio. PDR speed gate: `speed_mps > 0.3`. Observation clamp: `[0.005, 10.0]`. VO baseline uncertainty: 10% CoV in quadrature with step-period CoV.
- **Status**: Active.
- **Consumers**: `scale_fuser_.update(obs_scale, r_var)` -> `smooth_scale_`.

---

### Observer B -- MiDaS Depth Scale Constraint

- **Location**: `app/src/main/cpp/Tracker.cpp:71-267` (`Tracker::applyDepthScaleConstraint`); called at `Tracker.cpp:948-951` every 30 frames
- **What it does**: For each triangulated 3-D point that is either in the lower 40% of the image or gravity-projected below the phone by > 0.3 m, estimates metric depth via the ground-plane formula `metric_z = camera_h / (norm_y * cos(pitch) + sin(pitch))`. Dividing by `pts3d[i].z` gives a scale ratio. The **median** of >= 8 such ratios is the observation; **MAD * 1.4826** gives robust standard deviation; variance of the median is `sigma_sq / N`. Pair `(target_scale, median_variance)` is fed to `scale_fuser_.update`.
- **Why**: Provides an absolute scale anchor from MiDaS relative depth plus known camera height, independent of gait. The gravity-axis floor test extends coverage to scooter mode.
- **Parameters**: `camera_h = user_h * 0.85`. Height validity: `[0.8, 2.2]` m. Gravity gate: `g_mag > 5.0`. Horizon guard: `denom > 0.05`. `metric_z` range: `[0.3, 10.0]` m. VIO depth range: `[0.3, 12.0]`. Scale ratio range: `(0.1, 10.0)`. Min floor matches: 8. Safety gate: `target_scale` within 3x of current. MAD floor: `max(1e-3, 0.01 * median)`.
- **Status**: Active (~1 Hz). Requires `setDepthMap` to have been called.
- **Consumers**: `scale_fuser_.update` -> `smooth_scale_`. `last_depth_scale_variance_` exposed via `getLastDepthScaleVariance()`.

---

### Observer C -- Hesch/Martinelli Closed-Form VI Scale (ScaleEstimatorVI)

- **Location**: `app/src/main/cpp/ScaleEstimatorVI.cpp:38-155` (`ScaleEstimatorVI::solve`); wiring at `Tracker.cpp:902-943`
- **What it does**: Accumulates per-frame `KeyframePair` records. Every 10 pairs (when >= 4 available), stacks normal equations `A * x = b` where `x = [s, v0]` (scale + initial velocity), solves via `cv::DECOMP_SVD`. Scale variance is `(A^T A)^{-1}_{0,0} * sigma_sq_resid`. Valid results fed to `scale_fuser_.update(s_obs, max(var_obs, 0.04))`.
- **Why**: Provides a geometry-derived scale from IMU preintegration and visual translations, independent of gait or depth. Re-enabled after Madgwick (Step 1) made attitude clean enough for the gravity term in `deltaP` to be well-conditioned. Comment: "Phase 8 (gravity-aided scale) was disabled because IMU preintegration deltaP was dominated by gravity-subtraction errors when attitude was wrong."
- **Parameters**: `OBSERVER_C_SOLVE_INTERVAL = 10`. `MIN_PAIRS = 4`, `MAX_PAIRS = 16`. Scale validity: `[0.01, 10.0]`. Variance floor: `0.04`. Gravity: `(0, 0, -9.81)` m/s^2. Guards: `!is_pure_rotation && !is_static && 0.005 < dt < 1.0 && cv::norm(t_vo) > 0.5`.
- **Status**: Recently re-enabled (Step 3 Observer C). Previously disabled as Phase 8.
- **Consumers**: `scale_fuser_.update(s_obs, r_var)` -> `smooth_scale_`.

---

### ScaleFuser -- 1-D Kalman Scale Filter

- **Location**: `app/src/main/cpp/ScaleFuser.h`, `ScaleFuser.cpp`
- **What it does**: Scalar Kalman filter with state `s` (metric scale) and covariance `P`. `predict(dt)` adds process noise; `update(z, r)` performs a standard Kalman update. Scale clamped to `[0.01, 10.0]` after every update.
- **Why**: Merges all three scale observers in a statistically principled way. Observers with large measurement variance contribute almost nothing; the fuser naturally de-weights stale or noisy observers.
- **Parameters**: Initial `s = 0.20`, `P = 1.0`. `SCALE_MIN = 0.01`, `SCALE_MAX = 10.0`. `PROCESS_NOISE_PER_SEC = 1e-5`.
- **Status**: Active. Not strictly visual -- fuses visual outputs with IMU/PDR observers.
- **Consumers**: `smooth_scale_` mirrored from `scale_fuser_.scale()` after each update. Multiplied by `user_scale_correction_` for global position update.

---

## 7. Other / Disabled

### MSCKF Updater (UpdaterMSCKF) -- Dead Code

- **Location**: `app/src/main/cpp/UpdaterMSCKF.h`, `UpdaterMSCKF.cpp`; member `msckf_updater_` at `Tracker.h:175`
- **What it does**: Implements Mourikis/Roumeliotis MSCKF: triangulates lost feature tracks, builds Jacobians, QR null-space-projects out feature DOF, applies chi-squared-gated EKF update on camera pose clones.
- **Why disabled**: Comment at `Tracker.cpp:1110-1114`: "The MSCKF EKF state diverges from Tracker global pose, causing large position/rotation jumps (seen as 5-11m teleportations). The full error-state EKF (Phase 9) needs proper convergence before its corrections can be trusted."
- **Parameters**: `chi2_multiplier = 1.5`, `min_obs = 3`, `max_reproj_px = 5.0`, `pixel_noise = 1.0`.
- **Status**: DEAD CODE. `processLostFeatures` is never called. `extractLostFeatures` result is explicitly discarded at `Tracker.cpp:1117`. Member kept to avoid cascading header changes.
- **Consumers**: None (result discarded).

---

### Time-Offset Cross-Correlation Warmup (Phase 6 / TD Warmup)

- **Location**: `app/src/main/cpp/Tracker.h:157-160` (`TdSample` struct), `Tracker.cpp:608-677`
- **What it does**: During the first 60 frames (~2 s at 30 fps), buffers per-frame optical-flow magnitude and gyro magnitude as two time series. Cross-correlates them at lags `[-3, +3]` frames, finds the peak-correlation lag, converts to seconds, clamps to `[-50ms, +50ms]`, and warm-starts `ekf_.setTimeOffset`. Buffer is freed after warmup completes.
- **Why**: Camera and IMU clocks have a systematic offset on Android. Without correction, shifted IMU windows produce scale observations that conflate camera latency with actual motion.
- **Parameters**: `TD_WARMUP_FRAMES = 60`. Lag search: `[-3, +3]` frames. Clamp: `[-50ms, +50ms]`. Fires once at startup.
- **Status**: Active only during the first 60-frame warmup window, then permanently disabled.
- **Consumers**: `ekf_.setTimeOffset(estimated_td)` shifts the IMU integration window in all subsequent frames.
