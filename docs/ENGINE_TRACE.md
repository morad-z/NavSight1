# NavSight VIO Engine — End-to-End Trace

**Generated:** 2026-05-17
**Branch:** morad
**Scope:** App start to app stop; every major computation annotated with WHERE, WHAT, WHEN, WHY, FRAME CONVENTION, and SUSPICIOUS flags.

---

## Phase Legend

| Tag | Meaning |
|-----|---------|
| `INIT_STATIONARY` | IMU/Madgwick warm-up before motion detected |
| `INIT_MOTION` | EKF not yet full-init; first motion observed |
| `INIT_USER_BYPASS` | User pressed "calibrate" button; skip stationary gate |
| `RUN_STATIC` | Walking stopped; ZUPT/ZRUP active |
| `RUN_ROTATION_ONLY` | Device rotating without translation; rotation_dominated gate active |
| `RUN_WALKING` | Normal walking; MSCKF + PDR + scale fuser active |
| `RUN_LC_ACCEPT` | Loop closure match accepted; correction damping ramp active |
| `RUN_KEYFRAME` | Keyframe stored this frame; BA/LC tasks kicked off |
| `RUN_MARGINALIZATION` | Oldest camera clone marginalised from EKF window |
| `SHUTDOWN` | stopVIO called; all threads joined |

---

## Part 1 — App Start / JNI Initialisation

### 1.1 JNI_OnLoad
**WHERE:** `native-lib.cpp:155-238`
**WHEN:** INIT_STATIONARY
**WHAT:** JNI_OnLoad caches the JVM global reference, locates the `VioData` Java class and its constructor (`<init>(DDDDDDDDDDIIDIZZZ)V`), and stores both as globals. No VioEngine object is created here.
**WHY:** Field IDs are constant for the JVM lifetime; caching them avoids per-frame JNI overhead.
**FRAME CONVENTION:** None; JNI metadata only.

### 1.2 startVIO
**WHERE:** `native-lib.cpp:200-215`
**WHEN:** INIT_STATIONARY
**WHAT:** Constructs `VioEngine` on the native heap and stores the pointer in a `jlong` field on the Java-side `NavSightVio` object. Calls `viewer_server_.start(8765)` (WebSocket debug server).
**WHY:** One engine instance per active session; the Java long is the handle.

### 1.3 VioEngine constructor
**WHERE:** `VioEngine.cpp:1-40`
**WHEN:** INIT_STATIONARY
**WHAT:** Constructs `Tracker` and `IMUProcessor`. Mapper pipeline is DISABLED (commented out). `viewer_server_` is started but Mapper is not wired in.
**WHY:** Mapper was an optional offline refinement path; disabled after it was found to increase drift.
**SUSPICIOUS (LOW):** VioEngine.cpp lines ~20-35 — large commented-out Mapper wiring block. This is #if-0-equivalent dead code.

---

## Part 2 — IMU / Sensor Ingestion

### 2.1 IMU sample delivery (SensorRepository -> JNI)
**WHERE:** `SensorRepository.kt` (navSightExecutor thread) -> `NativeBridge.kt:pushIMUSample` -> `native-lib.cpp:pushIMU`
**WHEN:** All phases
**WHAT:** Android SensorManager delivers accelerometer and gyroscope events at ~200 Hz. `SensorRepository` pushes each sample via JNI. Timestamps are in nanoseconds (monotonic clock). The `rollingShutterSkewNs` value is stored as `@Volatile` and updated from camera metadata on each frame.
**WHY:** Decoupled sensor delivery; the native side batches samples between camera frames.
**FRAME CONVENTION:** Android sensor frame: X=right, Y=up, Z=out-of-screen (device body). EKF world frame is ENU Z-up (X=East, Y=North, Z=Up).

### 2.2 Madgwick attitude filter update
**WHERE:** `IMUProcessor` (internal, called from `pushIMU`)
**WHEN:** INIT_STATIONARY, INIT_MOTION, and all RUN_* phases
**WHAT:** Madgwick AHRS integrates gyro + LP-filtered gravity (alpha=0.02, tau~0.25 s at 200 Hz) to maintain a quaternion attitude estimate. Yaw is seeded once from magnetometer via `madgwickYawSeeded` flag in `SensorRepository.kt`. After seeding, compass is never used again.
**WHY:** Madgwick is the authoritative heading source throughout the session. The EKF yaw is not used for heading output (V-shape fix 2026-05-03).
**FRAME CONVENTION:** Output `R_GtoI` = world->body rotation. World = ENU Z-up.

### 2.3 Forster preintegration
**WHERE:** `IMUProcessor::preintegrate`
**WHEN:** INIT_MOTION, all RUN_* phases
**WHAT:** Midpoint integration accumulates `deltaR`, `deltaV`, `deltaP` and the five Jacobian blocks (`J_R_bg`, `J_V_bg`, `J_V_ba`, `J_P_bg`, `J_P_ba`) across the IMU samples that arrived since the last camera frame. A 9x9 preintegration covariance `Sigma_eta` is also integrated (Q matrix off-diagonal fix shipped 2026-05-16).
**WHY:** Preintegration decouples IMU integration from camera rate; the deltas are consumed once per camera frame.
**FRAME CONVENTION:** Deltas are in the body frame of the start of the preintegration window.

---

## Part 3 — Camera Frame Entry

### 3.1 processCameraFrameDirect (JNI camera callback)
**WHERE:** `native-lib.cpp:400-490`
**WHEN:** All RUN_* phases (and late INIT)
**WHAT:** (1) Assembles the NV21 byte planes from the Java-side `ImageProxy` into a contiguous `cv::Mat`. (2) Pushes rolling-shutter row skew (from `rollingShutterSkewNs @Volatile`) into `VioEngine`. (3) Calls `vision->processFrame(yuv_mat, timestamp_ns)`.
**WHY:** NV21 assembly is kept in JNI to avoid a second copy.
**FRAME CONVENTION:** Raw NV21 is camera-sensor native; Tracker converts to grayscale internally.
**SUSPICIOUS (LOW):** The old `processCameraFrame` entry point (lines 252-402, ByteArray path) remains in the file with a comment warning that it lacks the axis-swap. Dead code reachable if the wrong Java overload is called.

### 3.2 ensureOverlaySnapshot (render-thread reads)
**WHERE:** `native-lib.cpp:89-132`
**WHEN:** All phases (render thread)
**WHAT:** A 20 ms TTL snapshot mutex ensures the UI render thread always reads a consistent `(R, t, heading, trackedPoints)` bundle from the same EKF state version. The snapshot is refreshed by `processFrame` after each frame completes.
**WHY:** EKF state evolves on the VIO thread; the render thread needs a stable copy.
**FRAME CONVENTION:** Snapshot `t` is in Z-up ENU before the JNI axis swap (see Part 14).

---

## Part 4 — Tracker::processFrame — Initialisation Stages

### 4.1 YUV->Gray + CLAHE + blur gate
**WHERE:** `Tracker.cpp:852-900`
**WHEN:** All phases
**WHAT:** Converts NV21 to grayscale. Applies CLAHE for low-light robustness. Computes Laplacian variance as a blur score; if below threshold sets `frame_blurry = true`, which later gates MSCKF and keyframe storage.
**WHY:** Blurry frames produce noisy KLT tracks that corrupt the MSCKF residuals.

### 4.2 Intrinsics fallback
**WHERE:** `Tracker.cpp:900-950`
**WHEN:** All phases
**WHAT:** If camera intrinsics have not been pushed from Java yet, falls back to hard-coded defaults (fx=fy~720, cx=960, cy=540 for a 1920x1080 sensor). Uses `fx_use`, `fy_use`, `cx_use`, `cy_use` locals for all subsequent calculations.
**WHY:** The app can receive camera frames before calibration data is available at startup.
**SUSPICIOUS (MEDIUM):** Hard-coded fallback intrinsics are magic numbers with no device citation. If a user runs on a device with different optics, all projections are silently wrong until real calibration arrives.

### 4.3 Init check + heading bootstrap (Fix C)
**WHERE:** `Tracker.cpp:950-1086`
**WHEN:** INIT_STATIONARY -> INIT_MOTION transition
**WHAT:** Checks `initialized_` flag. If not yet initialised, waits for Madgwick to report a valid `R_GtoI` via `imu.getRotationGtoI()`. If Madgwick is not ready, returns early with `initialized_ = false` — no global_R_ fallback. When Madgwick is ready, calls `ekf_.initializeFull(R_GtoI_seed, ...)` seeding from the Madgwick rotation.
**WHY:** Fix C prevents EKF from starting with a wrong orientation seed when the phone has not yet settled after pick-up. Pre-Fix-C code seeded from `global_R_` = identity, which placed the world Z-up frame in an arbitrary orientation.
**FRAME CONVENTION:** `R_GtoI` = world->body (ENU Z-up -> device body).

---

## Part 5 — Per-Frame IMU Propagation

### 5.1 setPosition(global_t_) — Tier 1 revert
**WHERE:** `Tracker.cpp:1213`
**WHEN:** All RUN_* phases (every frame)
**WHAT:** Writes `global_t_` into the EKF position state block `p_G_` before calling `propagateIMU`. This overwrites whatever position the EKF computed in the previous frame.
**WHY (stated):** This is the v22 "Tier 1 revert" architecture. `global_t_` is the user-facing trajectory updated by the visual-VO + PDR pipeline. The EKF position is reset to this canonical value at the start of each frame so MSCKF and LC corrections in the current frame are applied on top of a position the visual pipeline trusts.
**FRAME CONVENTION:** `global_t_` is ENU Z-up metres.
**SUSPICIOUS (HIGH):** `setPosition` collapses the EKF position covariance block P_pp to zero (or near-zero) every frame. The Kalman gain for any position-dependent update (MSCKF, LC) is always computed against an artificially certain position prior. The acknowledged "P_pp collapse" bug is deferred but never repaired.

### 5.2 propagateIMU
**WHERE:** `EKFState.cpp:153-509`
**WHEN:** All RUN_* phases
**WHAT:**
- `R_new = deltaR.t() * R_GtoI_` (line 209) — applies preintegrated rotation left-multiplied onto current R_GtoI.
- `v_new = v_G_ + g*dt + R_GtoI_.t() * deltaV` — velocity in world frame.
- `p_new = p_G_ + v_G_*dt + 0.5*g*dt^2 + R_GtoI_.t() * deltaP` — position in world frame.
- Computes 19x19 Phi transition matrix and Q noise matrix; propagates `P_ = Phi * P_ * Phi^T + Q`.
- Velocity is clamped to 5 m/s (lines 502-508) after propagation.
**WHY:** Standard EKF error-state propagation using left-perturbation convention.
**FRAME CONVENTION:** World ENU Z-up, g = (0, 0, -9.81). Left-perturbation: `R_new = Exp([delta_theta]x) * R_old`.
**SUSPICIOUS (CRITICAL):** Velocity clamped at 5 m/s hard after propagation (lines 502-508). In v29 walk data 467/622 frames hit the clamp, indicating the clamp is load-bearing rather than a safety margin. This masks root-cause failures (IMU bias runaway, large dt spike, bad preintegration).

### 5.3 Phi matrix blocks — Tier 1 revert
**WHERE:** `EKFState.cpp:224-314`
**WHEN:** All RUN_* phases
**WHAT:** Phi blocks: `dv/dtheta = +Rt * skew(deltaV)` and `dp/dtheta = +Rt * skew(deltaP)`. These were sign-flipped by a swarm agent (2026-05-09) and then reverted. Current code uses the positive-sign form per Tier 1 revert comments.
**WHY:** The swarm's negative-sign form was an incorrect port of a different coordinate convention. The positive form matches the Forster preintegration math in ENU Z-up.

### 5.4 Q matrix assembly (full 9x9 off-diagonal)
**WHERE:** `EKFState.cpp:332-401`
**WHEN:** All RUN_* phases
**WHAT:** Copies the full 9x9 preintegration covariance from `imu_cov` into the Q block (rotation, velocity, position cross-terms included). Before the 2026-05-16 fix, only diagonal 3x3 blocks were copied, silently discarding gyro-accel cross-terms.
**SUSPICIOUS (MEDIUM):** Lines 370-377 — if `imu_cov.rows < 9`, falls back to scalar sigma^2 for diagonal blocks and zeros cross-terms. No EventCounter is emitted on this path. A silent regression in IMUProcessor output format would produce incorrect process noise with no log evidence.

### 5.5 Gyro bias push (every 6 frames)
**WHERE:** `Tracker.cpp:1243`
**WHEN:** All RUN_* phases
**WHAT:** Every 6 frames, the EKF gyro bias estimate `b_g_` is pushed back into `IMUProcessor` via `imu.setGyroBias(ekf_.getGyroBias())`. Keeps Madgwick bias tracking roughly aligned with the EKF estimate.
**WHY:** Prevents the two estimators from drifting apart on long walks.

---

## Part 6 — Gravity Alignment

### 6.1 LP-filtered gravity + gravity-alignment update
**WHERE:** `Tracker.cpp:1307-1405`
**WHEN:** RUN_STATIC (primarily); executed every frame when gyro_norm < 1.2 rad/s
**WHAT:** (1) Accumulates a 200-sample LP-filtered gravity vector in the body frame. (2) Gates on: acceleration magnitude in [9.5, 10.1] m/s^2, gyro norm < 1.2 rad/s, min 100 samples. (3) Computes gravity direction error between LP-filtered gravity and expected (0,0,-g) in body frame. (4) Calls `ekf_.applyMSCKFUpdate` with `apply_huber=false`. (5) Rotation-lag inflation: `var_total = var_acc + (tau_filter * omega * g)^2`.
**WHY:** Gravity alignment keeps EKF roll/pitch honest between MSCKF feature observations. `apply_huber=false` because Huber rejection would prevent needed corrections for large attitude errors — gravity is a hard physical constraint, not a potentially-outlier feature residual.
**FRAME CONVENTION:** Gravity direction measured in body frame; residual is cross-product between measured and expected direction.

---

## Part 7 — KLT Feature Tracking

### 7.1 KLT tracking with adaptive window
**WHERE:** `Tracker.cpp:1407-1593`
**WHEN:** All RUN_* phases
**WHAT:** OpenCV `calcOpticalFlowPyrLK` tracks keypoints from the previous frame. Window size: `win_sz = max(21, min(41, 2*expected_disp_px + 11))`. Feature quality = ratio of successfully tracked points to total.
**WHY:** Adaptive window prevents KLT from losing tracks during fast motion without wasting compute when stationary.

### 7.2 Time-delay (TD) warmup cross-correlation
**WHERE:** `Tracker.cpp:1595-1664`
**WHEN:** INIT_MOTION (early frames)
**WHAT:** Cross-correlates frame-to-frame optical flow magnitude against IMU angular velocity magnitude over a +/-3-frame search window to estimate camera-IMU time offset `t_d`.
**WHY:** A non-zero time delay between camera and IMU causes systematic errors in MSCKF Jacobians.
**SUSPICIOUS (HIGH):** The comment at line ~1600 says +/-50 ms, but the loop iterates +/-3 frames which at 30 fps = +/-100 ms. The stated constraint and the implemented constraint disagree by a factor of 2. If the true t_d is between 50-100 ms, this code will find it but the comment would have led an engineer to believe the range was insufficient.

---

## Part 8 — Static Detection / ZUPT / ZRUP

### 8.1 ZUPT (zero-velocity update)
**WHERE:** `EKFState.cpp:536-566`
**WHEN:** RUN_STATIC
**WHAT:** When the stationary detector fires (15 consecutive frames of low accelerometer variance and low optical flow), applies a zero-velocity measurement: H = [0|I|0...] (velocity columns only), res = 0 - v_G_. Post-update: velocity block scaled by 0.01, cross-terms by sqrt(0.01), scale variance by 0.99.
**WHY:** Prevents velocity drift during standstill; standard VIO technique.
**FRAME CONVENTION:** Residual in world ENU.

### 8.2 ZRUP (zero-rotation-rate update)
**WHERE:** `EKFState.cpp:569-650`
**WHEN:** RUN_STATIC
**WHAT:** Applies zero-rotation-rate measurement using mean of gyro window minus EKF gyro bias: H = -I at b_g columns (3:6 of state), res = mean(gyro_window) - b_g_. Sigma hardcoded at sigma_gyro = 0.01.
**WHY:** Drives gyro bias toward the currently observed gyro output during standstill.
**FRAME CONVENTION:** Gyro in body frame.
**SUSPICIOUS (HIGH):** sigma_gyro = 0.01 is hardcoded (Tracker.cpp:1703 at the R-matrix construction site). This value is not derived from the EKF calibrated gyro noise sigma_g stored in `IMUProcessor`. Wrong hardcoded sigma makes the ZRUP update either too aggressive (shrinks bias too fast) or too conservative.

### 8.3 Rotation-dominated gate (2026-05-17)
**WHERE:** `Tracker.cpp:2157`
**WHEN:** RUN_ROTATION_ONLY
**WHAT:** If `gyro_norm > 0.2 rad/s && step_speed < 0.1 m/s`, freezes `global_t_` — no PDR step, no `updateRelativePose`. Translation-dependent updates are skipped.
**WHY:** When rotating in place, essential-matrix decomposition can produce a spurious translation. Freezing position prevents hallucinated motion.

---

## Part 9 — Geometric Verification & Scale

### 9.1 Essential matrix + triangulation
**WHERE:** `Tracker.cpp:1790-2120`
**WHEN:** RUN_WALKING
**WHAT:** (1) `findEssentialMat` (RANSAC) on matched KLT points computes the essential matrix E between frames t-1 and t. (2) `recoverPose` decomposes E into R_vo (rotation) and t_vo (unit-scale translation direction). (3) Triangulates inlier point pairs via DLT: P = [R_GtoC | t_GtoC] (NOT transposed; 2026-05-12 fix). (4) Each triangulated point filtered by chi^2(0.95,2)=5.991 reprojection gate. (5) R_vo fused into EKF via `updateRelativeRotation(R_bc^T * R_vo * R_bc)`.
**WHY:** Essential matrix + triangulation is the front-end geometry; it feeds scale observers A/B/C.
**FRAME CONVENTION:** R_vo is camera-to-camera; R_bc^T * R_vo * R_bc converts to body-to-body for EKF.

### 9.2 Scale observer A (PDR)
**WHERE:** `Tracker.cpp:~2050-2100`
**WHEN:** RUN_WALKING
**WHAT:** Pedometer step detected -> stride length estimate -> PDR-predicted displacement fed to `ScaleFuser`. Updates `scale_fuser_` with (visual_baseline, pdr_distance) pair.
**WHY:** PDR-to-visual-baseline ratio gives metric scale independent of depth sensors.

### 9.3 Scale observer B (MiDaS / VI-Depth affine fit)
**WHERE:** `Tracker.cpp:43-502` (`fitDisparityAffine` + `applyDepthScaleConstraint`)
**WHEN:** RUN_WALKING (when MiDaS depth frame available)
**WHAT:** VI-Depth-style affine fit in disparity space. For each tracked point with a MiDaS disparity, builds weighted least-squares: [d_i  1] * [a; b]^T = d_vio_inv_i, where d_vio_inv_i = 1 / z_vio_i. Solves 2x2 WLS (closed form). Then computes ratio = metric_z / z_vio_baseline (line ~335) and feeds to `ScaleFuser`.
**WHY:** MiDaS gives relative depth; affine fit converts it to metric using VIO-predicted depths as anchors.
**SUSPICIOUS (MEDIUM):** The `ratio = metric_z / z_vio_baseline` formula biases `ScaleFuser` toward 1.0 — the code comment at line ~335 flags this explicitly. If VIO scale is already wrong, the ratio converges to 1.0 (no correction) rather than to the true affine-fit scale.

### 9.4 Scale observer C (VI gravity)
**WHERE:** `Tracker.cpp:~2100-2120`
**WHEN:** RUN_WALKING
**WHAT:** Compares gravity vector magnitude from accelerometer against nominal 9.81 m/s^2 to derive a weak scale sanity bound fed to `ScaleFuser`.
**WHY:** Third independent scale observer; low weight, mostly a sanity bound.

### 9.5 ScaleFuser chi^2 gate
**WHERE:** `EKFState.cpp:513-567`
**WHEN:** RUN_WALKING
**WHAT:** `updateScale` applies a chi^2-gated measurement update. Gate threshold = 9.0 (chi^2(0.999,1)). Scale clamped to [0.005, 20.0] after update.
**WHY:** Prevents wildly inconsistent scale observations from injecting noise.

---

## Part 10 — Heading & Position Integration

### 10.1 Heading from Madgwick
**WHERE:** `Tracker.cpp:2157-2200`
**WHEN:** All RUN_* phases
**WHAT:** `scalar_heading_ = imu.getHeading()` reads the Madgwick-derived yaw angle. This is the ONLY source of heading output. EKF yaw is never used for `scalar_heading_` or `out.heading` (V-shape bug fix 2026-05-03).
**WHY:** Madgwick tracks fast rotations correctly through gradient-descent; EKF yaw under-rotates quick turns because MSCKF linearisation accumulates small angular increments.
**FRAME CONVENTION:** Heading = yaw in ENU; 0 deg = East, increasing counterclockwise (mathematical convention).

### 10.2 Heading projection to displacement
**WHERE:** `Tracker.cpp:~2200-2260`
**WHEN:** RUN_WALKING
**WHAT:** Converts scalar step displacement delta_d and heading theta to ENU displacement: delta_x = delta_d * sin(heading) (East), delta_y = delta_d * cos(heading) (North). Calls `ekf_.updateRelativePose(delta_p_body)`.
**WHY:** Visual VO gives relative bearing; heading + step distance gives metric ENU displacement.
**FRAME CONVENTION:** ENU Z-up.

### 10.3 updateRelativePose / PDRStep
**WHERE:** `EKFState.cpp` (updateRelativePose, updatePDRStep)
**WHEN:** RUN_WALKING
**WHAT:** Both functions apply a position-measurement Kalman update. H = [0|0|0|0|I|0...] at position columns 12:15. Standard EKF measurement update with K = P*H^T*(H*P*H^T+R)^{-1}.
**WHY:** These are the primary position correction paths during walking.

---

## Part 11 — EKF Clone Management & MSCKF

### 11.1 Clone storage
**WHERE:** `Tracker.cpp:2460-2640`
**WHEN:** RUN_WALKING, RUN_KEYFRAME
**WHAT:** Every frame the current camera pose is stored as a clone in the EKF window (`window_`, max 11). Clone rotation: `clone_R = R_bc_mat * clone_R_GtoI` — bakes extrinsic R_bc into the stored clone so the clone directly represents R_GtoC (world->camera).
**WHY:** MSCKF needs camera-frame poses; baking R_bc at storage time avoids repeated multiplication during update.
**FRAME CONVENTION:** `clone_R` = R_GtoC = R_bc * R_GtoI. World->camera.

### 11.2 SLAM feature undistort + observe
**WHERE:** `Tracker.cpp:~2460-2640`
**WHEN:** RUN_WALKING
**WHAT:** For each active SLAM feature, undistorts its current 2D observation using the radial-tangential distortion model, then calls `ekf_.addSlamObservation(feat_id, undistorted_pt)`. SLAM features use inverse-depth parameterisation (alpha, beta, rho) anchored at the first clone.
**WHY:** Inverse-depth avoids singularities for nearly-parallel features.

### 11.3 SLAM feature promotion
**WHERE:** `Tracker.cpp:~2645-2700`
**WHEN:** RUN_WALKING
**WHAT:** A SLAM feature is promoted to an active EKF slot when it has >=8 observations, >=2 keyframe observations, stereo baseline >=1.5 cm, midpoint triangulation success, and RMSE gate <=1.5 px. After promotion, `UpdaterSLAM` applies a measurement update.
**WHY:** Strict promotion gate prevents noisy tracks from entering the EKF state.

### 11.4 MSCKF update (lost features)
**WHERE:** `EKFState.cpp:applyMSCKFUpdate` + `UpdaterMSCKF.cpp:processLostFeatures`
**WHEN:** RUN_WALKING (gated: !is_static && !frame_blurry)
**WHAT:** (1) For features lost from KLT: triangulate using all clone poses via DLT (UpdaterMSCKF.cpp:21-82). (2) Compute per-clone Jacobians: H_theta = -dz_dpfc * skew(p_f_C) (negative sign, 2026-05-09 fix). (3) Null-space project. (4) QR compress if rows > state_dim. (5) Feed to `applyMSCKFUpdate` with Huber kernel (delta=2.4477, 3*delta hard reject), damping ramp 0.5->1.0 over 5 calls.
**WHY:** MSCKF processes features after loss, projecting out the unobservable depth direction. This is the primary visual-inertial correction path.
**FRAME CONVENTION:** Jacobians in camera frame; residuals are 2D pixel errors.

### 11.5 Null-space projection
**WHERE:** `UpdaterMSCKF.cpp:186-211`
**WHEN:** RUN_WALKING
**WHAT:** Projects the stacked Jacobian matrix onto the left null space of the depth column to eliminate the unobservable feature-depth direction.
**WHY:** Feature depth is unobservable from monocular data; projection removes the corresponding residual direction.
**SUSPICIOUS (MEDIUM):** Function named `nullspaceProject` with comment claiming "Householder QR" but lines 186-198 use `cv::SVD`. Numerically equivalent for this projection, but the documentation mismatch reduces maintainability.

### 11.6 MSCKF damping ramp
**WHERE:** `EKFState.cpp:921-928`
**WHEN:** RUN_WALKING (first 5 MSCKF calls after init)
**WHAT:** Linear scale from 0.5->1.0 over MSCKF_DAMPING_RAMP_FRAMES=5 calls. The K vector is scaled before the state update.
**WHY:** Prevents large initial corrections from destabilising the EKF when few features have been tracked.

### 11.7 phi_bc update SKIPPED
**WHERE:** `EKFState.cpp:1068-1073`
**WHEN:** RUN_WALKING
**WHAT:** The code block that would update the camera-to-IMU rotation error state phi_bc during the MSCKF update is commented out. H_bc columns are zeroed; R_bc corrections are never applied.
**WHY:** H_bc was disabled because the extrinsic calibration was causing instability. Clones bake R_bc at storage time.
**SUSPICIOUS (MEDIUM):** P_bc cross-covariance grows unbounded except for the 1e-8 per-frame process noise regularisation (EKFState.cpp:438-457). If R_bc drifts from the true value mid-session (thermal expansion, re-mount), there is no recovery path; accumulated cross-covariance corrupts correlations with other state blocks.

### 11.8 Marginalization
**WHERE:** `EKFState.cpp:800-909`
**WHEN:** RUN_MARGINALIZATION (when window_ reaches MAX_CLONES=11)
**WHAT:** `marginalizeOldestCloneNoLock` removes the oldest clone. SLAM features anchored to the oldest clone are re-anchored to `window_[1]` (second-oldest remaining clone, not window_.back()). P_ matrix is spliced via Schur complement.
**WHY:** Bounded EKF window; marginalization preserves information from the removed clone.
**FRAME CONVENTION:** Re-anchoring transforms inverse-depth params (alpha, beta, rho) from old anchor's camera frame to new anchor's camera frame.

---

## Part 12 — Keyframe Operations (RUN_KEYFRAME)

### 12.1 Keyframe heading drift correction
**WHERE:** Tracker.cpp:3084-3335
**WHEN:** RUN_KEYFRAME (every >=14 frames, gated: !is_static && !translation_degenerate && !is_pure_rotation)
**WHAT:** (1) findEssentialMat + recoverPose between current frame and last stored keyframe. (2) R_kf_body = R_bc_transpose * R_kf * R_bc converts camera-frame rotation to body-frame. (3) Extracts gravity-aligned yaw. (4) Calls ekf_.updateGravityAlignedYaw(delta_yaw).
**WHY:** Accumulates heading correction from keyframe-to-keyframe geometry; complements per-frame Madgwick heading.

### 12.2 Keyframe storage
**WHERE:** Tracker.cpp:3337-3460
**WHEN:** RUN_KEYFRAME (every 15 frames, gated !frame_blurry)
**WHAT:** storeKeyframe saves grayscale image, ORB descriptors, keypoints, and triangulated 3D points in world frame. storeKeyframeDescriptors adds to the LoopClosureDetector ORB descriptor database.
**WHY:** Keyframes are the spatial anchors for loop closure, relocalization, and SLAM promotion.

### 12.3 Loop closure keyframe rotation convention (v23.15 fix)
**WHERE:** Tracker.cpp:3460-3660
**WHEN:** RUN_KEYFRAME
**WHAT:** When adding to the LoopClosureDetector database, the clone rotation R_GtoC (world->camera) is transposed to get R_CtoG (camera->world) before storage. This is the v23.15 fix; before it, the LC database stored world->camera as camera->world, producing 180-degree rotation residuals in PnP.
**WHY:** PnP RANSAC inside LC query needs camera->world poses.
**FRAME CONVENTION:** LC database stores R_CtoG = camera->world (transposed from stored EKF clone).

### 12.4 LandmarkMap populate (Phase 6.2)
**WHERE:** Tracker.cpp:3660-3745
**WHEN:** RUN_KEYFRAME
**WHAT:** Iterates triangulated pts3d_world and calls addOrMergeLandmark for each finite-valued point. cullStaleLandmarks removes landmarks not observed recently.
**WHY:** LandmarkMap is the spatial index for the track-local-map path (Phase 6.4).

### 12.5 applyLandmarkObservations DISABLED (Phase 6.4)
**WHERE:** Tracker.cpp:3745-4046
**WHEN:** Would be RUN_KEYFRAME; currently DISABLED
**WHAT:** Projects LandmarkMap entries into the current frame. BFMatcher + Lowe ratio 0.8 + Hamming + 15 px gate associates current KLT tracks to map landmarks. The resulting applyLandmarkObservations call is commented out.
**WHY (disabled):** In v28, uncommenting this path caused |p_G| to grow from 108 m to 417 m over a 109 m walk — a self-referential positive feedback loop.
**SUSPICIOUS (CRITICAL):** The root cause of the feedback loop has not been identified or fixed; the function is simply disabled, leaving Phase 6.4 as dead code. Loop-closure-corrected landmark positions cannot constrain the trajectory.

### 12.6 PoseGraph addNode
**WHERE:** Tracker.cpp:4049-4130
**WHEN:** RUN_KEYFRAME
**WHAT:** pose_graph_.addNode(kf_id, global_t_, scalar_heading_, clone_cov) stores the keyframe pose with covariance (sigma^2_xy, sigma^2_z, sigma^2_yaw_body) extracted from the current EKF clone covariance block. Automatically creates an odometry edge to the previous node.
**WHY:** PoseGraph is the backend for global consistency; odometry edges chain keyframes sequentially.

### 12.7 Loop closure query dispatch
**WHERE:** Tracker.cpp:4132-4191
**WHEN:** RUN_KEYFRAME
**WHAT:** Publishes the current keyframe to the LC worker thread. Search radius: search_radius_m = min(30, max(2, 3*sigma_p_xy)) — adaptive to position uncertainty.
**SUSPICIOUS (LOW):** loop_closure_kf_count_in_db is incremented at the call site (Tracker.cpp:4132-4136) rather than inside LoopClosureDetector::addKeyframe. If addKeyframe internally rejects a keyframe, the external counter is still incremented.

### 12.8 BA round kick-off
**WHERE:** Tracker.cpp:4194-4203
**WHEN:** RUN_KEYFRAME
**WHAT:** First consumes any pending BA result (consumeBAResultIfReady), then kicks off a new BA round on the worker thread. Consume-first ensures the new round optimises against already-refined state.
**WHY:** Windowed bundle adjustment refines SLAM features and camera poses on a background thread.

---

## Part 13 — Loop Closure (RUN_LC_ACCEPT)

### 13.1 LoopClosureDetector::tryDetectLoopImpl
**WHERE:** LoopClosureDetector.cpp:456-803
**WHEN:** Background LC worker thread; result consumed in RUN_WALKING / RUN_KEYFRAME
**WHAT:** (1) BoW query under lock; compute DBoW2 score against each candidate. (2) Adaptive minScore: K=10 recent neighbor scores, minScore = max(0.002, mean(K_scores) * 0.9). (3) Spatial filter: candidates must be outside temporal_exclusion_window. (4) Heading gate: candidate heading within +/-pi/2 of current heading — prevents 180-degree false matches from ORB rotation-symmetry limitation. (5) BFMatcher Hamming + Lowe ratio 0.75. (6) PnP RANSAC: 100 iterations, 4 px threshold, confidence=0.99. (7) Chi^2 gate: PnP residual chi^2 < 22.5. (8) Constructs LoopMatch: R_now_to_match = R_match_world * R_now_world_transpose; t_now_to_match = t_match_world - R_now_to_match * t_now_world.
**WHY:** Multi-stage filtering minimises false loop closures; chi^2 gate is the final quantitative quality check.
**FRAME CONVENTION:** R_now_to_match and t_now_to_match express relative pose of current keyframe with respect to matched keyframe, both in ENU world frame.

### 13.2 consumeLoopClosureMatchIfReady
**WHERE:** Tracker.cpp:4205-4212
**WHEN:** Every frame (unconditional)
**WHAT:** Atomic-flag check; if a new LC match is pending, extracts it and applies ekf_.updateAbsolutePose(match.t_world, match.R_world) plus pose_graph_.addLoopEdge(from_id, to_id, match). Triggers damping ramp: lc_damping_step_ = 0 -> ramp 0.5->1.0 over next 10 frames.
**WHY:** Unconditional consumption ensures the correction smoothing ramp runs on every subsequent frame rather than waiting for the next keyframe boundary.

### 13.3 applyKeyframePoseCorrection
**WHERE:** LoopClosureDetector.cpp:248-283
**WHEN:** RUN_LC_ACCEPT
**WHAT:** Applies delta_yaw correction to the stored keyframe pose in the LC database: R_world_cam_corrected = R_z(delta_yaw) * R_world_cam. Left-multiplication in world frame.
**WHY:** Keeps the LC database consistent with the corrected trajectory so future queries use corrected reference poses.
**FRAME CONVENTION:** R_z(delta_yaw) is a rotation about the world Z-axis (ENU up).

---

## Part 14 — Output Assembly & JNI Return

### 14.1 Output assembly
**WHERE:** Tracker.cpp:4252-4325
**WHEN:** End of every RUN_* frame
**WHAT:**
- out.R = EKF rotation if full-init, else global_R_
- out.t = global_t_ (canonical position, ENU Z-up metres)
- out.heading = scalar_heading_ (Madgwick yaw)
- out.poseFlags = bitfield: bit0=is_static, bit1=is_pure_rotation, bit2=pose_valid, bit3=used_fallback
- out.trackedPoints, out.trackedPointAges, out.meanFlow — visual debug overlay data
**WHY:** Single output struct decouples Tracker internals from the JNI boundary.

### 14.2 JNI axis swap (Z-up to Y-up)
**WHERE:** native-lib.cpp:505-507
**WHEN:** Every frame at JNI boundary
**WHAT:** g_y = output.t.at<double>(2); g_z = output.t.at<double>(1) — swaps Y and Z components of global_t_ before building the VioData Java object. Kotlin consumers receive a Y-up coordinate.
**WHY:** Legacy Kotlin rendering code uses a Y-up frame; the swap is done once at the JNI boundary.
**FRAME CONVENTION:** Native side: ENU Z-up. Java side: Y-up (Y=up, Z=north).

### 14.3 Euler angle decomposition for debug
**WHERE:** native-lib.cpp:547-553
**WHEN:** Every frame (debug output only)
**WHAT:** Decomposes out.R (a 3x3 R_GtoI) into roll/pitch/yaw using a fixed Euler sequence. Not used for heading output (that comes from out.heading = scalar_heading_).
**WHY:** Provides roll/pitch/yaw fields in VioData for display in the debug UI.
**SUSPICIOUS (HIGH):** The Euler decomposition sequence applied to a Z-up R_GtoI matrix is not explicitly documented. An incorrect Euler convention for ENU Z-up silently produces wrong yaw in the debug UI, misleading engineers who use debug yaw to diagnose heading regressions.

### 14.4 stopVIO
**WHERE:** native-lib.cpp:220-238 -> VioEngine::~VioEngine -> Tracker::reset
**WHEN:** SHUTDOWN
**WHAT:** Java calls stopVIO. JNI calls VioEngine destructor. VioEngine joins BA and LC worker threads first (ordered shutdown). Tracker::reset clears all state: window_, slam_features_, scale_fuser_.reset(0.10, 4.0), etc. VioEngine pointer set to null.
**WHY:** Ordered shutdown prevents dangling thread references.

---

## Suspicious Findings — Prioritized

| # | File:Line | Description | Severity | Phase | Hypothesis |
|---|-----------|-------------|----------|-------|------------|
| 1 | EKFState.cpp:502-508 | Velocity clamped to 5 m/s hard after propagation; 467/622 frames hit clamp in v29 | CRITICAL | RUN_WALKING | IMU bias drift or dt spike causes integration velocity runaway; the clamp is masking a root cause (bad preintegration, large dt, or uncorrected bias) rather than bounding a safe operating range |
| 2 | Tracker.cpp:3935-3984 | applyLandmarkObservations disabled after v28 caused p_G growth 108 m to 417 m | CRITICAL | RUN_KEYFRAME | Self-referential positive feedback: world-frame landmarks projected into a pose that is itself updated by those observations; root cause not identified or fixed — Phase 6.4 is dead code |
| 3 | Tracker.cpp:1213 | setPosition(global_t_) before propagateIMU collapses P_pp to zero every frame | HIGH | RUN_WALKING | Kalman gain for all position-dependent updates (MSCKF, LC, PDR) is computed against an artificially zero-uncertainty prior; this biases every visual-inertial correction toward maximum boldness |
| 4 | native-lib.cpp:547-553 | Euler decomposition sequence for Z-up R_GtoI not documented; silent wrong yaw in debug UI | HIGH | All | Wrong Euler convention for ENU Z-up; misleads engineers who use the debug yaw value to diagnose heading regressions |
| 5 | Tracker.cpp:1703 | ZRUP sigma_gyro = 0.01 hardcoded; not derived from EKF calibrated gyro noise | HIGH | RUN_STATIC | Wrong sigma makes ZRUP either too aggressive (shrinks bias too fast) or too conservative; the calibrated sigma_g from IMUProcessor should be used |
| 6 | Tracker.cpp:1595-1664 | TD warmup searches +/-3 frames (~100 ms) but comment says +/-50 ms | HIGH | INIT_MOTION | If the true camera-IMU time delay is 50-100 ms, only this code path can find it; the incorrect comment would cause a debugging engineer to believe the search range is inadequate when it is actually correct |
| 7 | UpdaterMSCKF.cpp:186-198 | nullspaceProject claims Householder QR; uses SVD | MEDIUM | RUN_WALKING | No numerical bug (SVD and Householder QR are equivalent for this projection) but the documentation mismatch creates a trap for future maintainers |
| 8 | Tracker.cpp:335 | ratio = metric_z / z_vio_baseline biases ScaleFuser toward 1.0 (code comment flags it) | MEDIUM | RUN_WALKING | The ratio formula is relative to VIO own scale estimate; if VIO is already drifted, the ratio converges to 1.0 (no correction) rather than to the true affine-fit scale |
| 9 | EKFState.cpp:438-457 | phi_bc update disabled; P_bc cross-covariance grows unbounded except for 1e-8 regularisation | MEDIUM | RUN_WALKING | If R_bc drifts from true value mid-session, there is no recovery path; accumulated cross-covariance corrupts correlations with other state blocks |
| 10 | Tracker.cpp:779-803 | getPositionCovarianceXZ misleadingly named — actually returns XY covariance | MEDIUM | All | Any code or log reading this value expecting Z (vertical) uncertainty gets X (East) instead; could produce wrong LC search radii or incorrect uncertainty visualisation |
| 11 | Tracker.cpp:4132-4136 | loop_closure_kf_count_in_db incremented at call site, not inside addKeyframe; miscounts on assertion failure | LOW | RUN_KEYFRAME | If LoopClosureDetector::addKeyframe rejects a keyframe internally, the external counter is still incremented; search-radius computation and debug logs report an inflated keyframe count |
| 12 | EKFState.cpp:370-377 | Q matrix scalar fallback when imu_cov.rows < 9; no EventCounter on this path | LOW | RUN_WALKING | A silent regression in IMUProcessor output format would make the EKF use incorrect (diagonal-only) process noise with no log evidence; impossible to detect from runtime data alone |

---

## Architecture Summary

The engine is a two-trajectory system:
- **global_t_** — user-facing trajectory, updated by visual-VO heading projection + PDR, overwritten into EKF at the start of each frame (Tier 1 revert).
- **EKF p_G_** — position state within the Kalman filter, receives MSCKF + LC + PDR updates, but is reset to global_t_ at the next frame boundary.

**Heading** is always Madgwick (never EKF yaw). **Rotation** (for output.R) is EKF when full-init.

**Frame convention:** ENU Z-up throughout the native stack; Y-up swap at the JNI boundary (native-lib.cpp:505-507).

**Load-bearing flags (never disable):** -fno-finite-math-only in CMakeLists.txt — re-enables all isfinite() checks across 7 files that guard triangulation, MSCKF Jacobians, and scale observers.
