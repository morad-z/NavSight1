# NavSight — How the System Works (Step by Step)

This document explains the full pipeline of NavSight's Visual-Inertial Odometry (VIO) system at a conceptual level: how it estimates position, heading, and speed without GPS.

---

## 1. High-Level Architecture

NavSight fuses two independent sensor streams to estimate the user's position:

1. **Camera** (30 fps) — provides visual information about the environment
2. **IMU** (200 Hz) — gyroscope (rotation rate) + accelerometer (acceleration + gravity)

The camera tells us **which direction** the user moved (by observing how the scene shifts between frames). The IMU tells us **how fast** the user is moving (by detecting footsteps) and **which direction they're facing** (by integrating rotation rate). Combining both gives position estimates in meters.

### Why not just use the camera alone?

A single camera (monocular) can see direction of movement but **cannot determine scale** — it doesn't know if the scene moved 1 meter or 10 meters. A photo of a toy room and a real room look identical. The IMU provides the missing scale through step detection.

### Why not just use the IMU alone?

IMU sensors drift quickly. Integrating acceleration twice to get position accumulates errors exponentially — after 10 seconds, the position estimate can be off by meters. The camera constrains this drift by observing the real world.

---

## 2. System Initialization

Before tracking can begin, the system needs to know three things:

### 2.1 Gravity Direction
The accelerometer measures **gravity + any motion**. When the user is stationary for the first ~1 second, the accelerometer reading IS gravity. The system collects ~40 samples and averages them to determine the gravity vector in the phone's coordinate frame. This tells us which way is "down" — critical for separating horizontal movement from vertical (gravity).

### 2.2 Gyroscope Bias
Every gyroscope has a small constant offset (bias) — it reports a tiny rotation rate even when perfectly still. During the same stationary period, the system averages gyroscope readings to measure this bias (typically 0.001–0.01 rad/s per axis). This bias is subtracted from all future gyro readings.

### 2.3 Initial Heading (Compass)
The magnetometer provides an initial compass bearing (e.g., "facing North-East"). This is used **once at startup only** to align the coordinate system with the real world. A magnetic declination correction (~5.5° in Haifa) is applied because magnetic north differs from true north. After initialization, the magnetometer is never used again — the gyroscope tracks all heading changes.

---

## 3. Feature Detection — "What is a Feature?"

### What is a feature?
A **feature** is a small patch of the image (typically 21×21 pixels) that has strong intensity gradients in at least two directions — meaning it's a **corner** or **textured point** that can be uniquely identified between frames.

Examples of good features: corners of tiles, edges of signs, doorknobs, window corners, texture patterns on walls.

Examples of bad features: blank walls, uniform floors, sky — these have no gradient, so you can't tell if they moved.

### How are features detected?
NavSight uses the **Shi-Tomasi corner detector** (also called "Good Features to Track"):

1. For each pixel, compute the **structure tensor** — a 2×2 matrix that describes how intensity changes in X and Y directions within a small window
2. Compute the two eigenvalues (λ1, λ2) of this matrix
3. If **both eigenvalues are large**, the pixel is a corner — it has strong gradients in two directions
4. The quality score is `min(λ1, λ2)` — corners where both directions are strong score highest
5. Features with quality above a threshold (0.05 × best score) are kept

### Grid-based distribution
The image is divided into a **4×5 grid** (20 cells). Features are detected independently in each cell, ensuring they're spread across the entire image — not clustered in one high-texture area. This is critical because evenly distributed features give more reliable motion estimates.

NavSight tracks up to **200 features** at a time, with a minimum of **80** before triggering replenishment.

### Is a feature a pixel or a point?
A feature starts as a **pixel location** (x, y) in the image. It represents a corner in the 2D image. Later, through triangulation (Section 6), it can become a **3D point** (X, Y, Z) in space. But during tracking, it's always a 2D pixel coordinate.

---

## 4. Feature Tracking — Optical Flow (KLT)

### How does tracking work?
Between consecutive frames (33ms apart at 30fps), the system tracks where each feature moved using the **Lucas-Kanade (KLT) optical flow** algorithm:

1. Take a feature's patch (21×21 pixels) from the previous frame
2. Search for where that same patch appears in the current frame
3. The algorithm assumes the patch moved by (dx, dy) and solves for the displacement that minimizes the intensity difference between the two patches
4. This is done at **3 pyramid levels** — first at a coarse (quarter-resolution) image to handle large movements, then refined at full resolution

### IMU-assisted prediction
Before running KLT, NavSight uses the gyroscope rotation to **predict** where each feature should appear. If the phone rotated 2° to the right, every feature should shift left by a calculable amount. This prediction is computed using the **rotation homography**: `H = K × R × K⁻¹`, where K is the camera matrix and R is the IMU rotation. KLT then starts its search from this predicted position, making it converge faster and more reliably.

### Forward-Backward consistency check
To verify that a tracking result is correct, KLT is run in **reverse** — from the current frame back to the previous frame. If the backward-tracked position doesn't land within 2 pixels of the original position, the match is rejected. This eliminates most false matches.

### Feature ages
Each feature has an **age** — the number of consecutive frames it has been successfully tracked. Features that survive many frames (age ≥ 3, called "mature") are more reliable than freshly detected ones. The system's confidence (quality score) depends partly on the fraction of mature features.

---

## 5. Motion Estimation from Features

### Mean optical flow
The average pixel displacement across all tracked features is called **mean flow**. This single number tells us:
- **< 0.4 px**: the scene barely moved → possible stationary
- **0.4–2.5 px**: normal walking range
- **> 150 px**: motion blur → frames are too blurry to trust

### Essential Matrix and Relative Pose
When there's enough motion (flow ≥ 0.8 px, "parallax"), the system computes the geometric relationship between two frames:

1. **Lens undistortion**: feature coordinates are corrected for camera lens distortion
2. **Essential matrix (E)**: found using the **five-point algorithm** inside a RANSAC loop. The essential matrix encodes the geometric constraint between two views: for any correctly matched pair of points, `p2ᵀ × E × p1 = 0`
3. **RANSAC**: randomly picks 5 point pairs, computes E, counts how many other pairs agree (inliers). Repeats many times and keeps the E with the most inliers. This rejects outlier matches.
4. **Pose recovery**: E is decomposed into a rotation R and a **unit-length** translation direction t. The translation has no scale — we only know the direction of movement, not the distance.

### Why is the translation unit-length?
With a single camera, the essential matrix can only determine the direction of translation up to an unknown scale factor. The scene could be a tiny model 10cm away or a building 100m away — geometrically identical. This is the fundamental **scale ambiguity** of monocular vision.

---

## 6. Triangulation — Features Become 3D Points

When the camera has translated (not just rotated), features seen from two different positions can be **triangulated** into 3D points:

1. Set up two projection matrices: P1 (previous camera position) and P2 (current position, using R and t from the essential matrix)
2. For each matched feature pair, find the 3D point that projects to both 2D locations with minimum error
3. This uses **SVD (Singular Value Decomposition)** to solve the linear system

The resulting 3D points are in an arbitrary scale (because t was unit-length). They're used for:
- **Reprojection error check**: each 3D point is projected back into the image. If the projected pixel is more than ~2.4px from the detected feature, it's rejected as an outlier (chi-squared test with 2 DOF at 95% confidence)
- **Depth-based scale estimation** (with MiDaS, see Section 8)

---

## 7. Scale Estimation — "How Far Did I Walk?"

This is the critical bridge between unit-less visual displacement and real-world meters.

### Step detection (primary scale source)
The accelerometer detects footsteps using **peak detection**:

1. The accelerometer magnitude is low-pass filtered (α = 0.20) to smooth out noise
2. When the filtered magnitude crosses above a **high threshold** (peak of a step), a step is registered
3. When it drops below a **low threshold** (valley), the system resets and waits for the next peak
4. Timing constraints reject false steps: min 0.3s between steps (running), max 1.5s (slow walking)

### Walking pattern recognition
Not every accelerometer spike is a step — car vibrations, phone being placed on a table, etc. The system computes a running **variance** of acceleration magnitude:
- Variance > 0.20 → walking pattern (rhythmic acceleration from footsteps)
- Variance < 0.05 → stationary or in a vehicle
- Hysteresis between these thresholds prevents oscillation

### Speed from step frequency
Walking speed is estimated from step frequency and user height:

```
base_stride = user_height × 0.415    (anthropometric model)
freq_factor = 0.7 + 0.3 × min(2.5, step_frequency)
stride_length = base_stride × freq_factor
speed = stride_length × step_frequency
```

For a 1.75m person walking at 2 Hz (normal pace): stride = 1.75 × 0.415 × 1.3 = 0.94m, speed = 1.88 m/s ≈ 6.8 km/h.

### Scale factor computation
The **scale factor** bridges visual displacement (unit-less) to real meters:

```
scale = step_displacement / visual_displacement
```

Where:
- `step_displacement = speed × dt` (meters, from IMU step detector)
- `visual_displacement = ||t_vo||` (unit-less, from essential matrix)

This is updated every frame with an **exponential moving average** (EMA): fast learning (α=0.15) during the first 20 observations, then slow tracking (α=0.05) to reject noise.

---

## 8. Depth-Based Scale (MiDaS Neural Network)

NavSight also uses a neural network called **MiDaS v2.1 Small** to estimate depth from single images:

1. Every ~1 second, the current camera frame is fed to MiDaS (runs on GPU via TensorFlow Lite, ~75ms)
2. MiDaS outputs a **relative depth map** — it knows that the floor is closer than the wall, but not the actual distance in meters
3. NavSight converts relative depth to metric depth using the **camera height** (user height) and **pitch angle** (from gravity direction): `metric_depth = camera_height / cos(pitch)`
4. For each tracked feature on the ground plane, the predicted metric depth is compared to the triangulated 3D depth from vision
5. The ratio gives an independent scale estimate, blended into the main scale factor

This provides scale even when the user is not walking (e.g., standing still and looking around).

---

## 9. Heading (Direction) Estimation

### Gravity-projected gyro yaw rate
The phone's yaw axis (the direction you turn your body) is aligned with **gravity**, not the phone's Z-axis. For a phone held at an angle, a body turn rotates around the gravity vector, which is some combination of the phone's X, Y, and Z axes.

The system projects the gyroscope angular velocity onto the gravity direction:

```
yaw_rate = -(ωx × gx + ωy × gy + ωz × gz)
```

Where ω is the angular velocity vector from the gyroscope, and g is the normalized gravity vector in the phone's frame (from filtered accelerometer). This correctly measures turning regardless of how the phone is tilted.

The heading is then:
```
heading += yaw_rate × dt
```

This heading is integrated every frame (~33ms), giving a smooth turning signal.

### Why not use the camera for heading?
Yaw (heading) is **unobservable** from a monocular camera in general motion. The essential matrix can decompose into R and t, but the yaw component of R is entangled with the translation direction — you can't distinguish "camera rotated 5° right" from "camera moved slightly left". The gyroscope gives a direct, unambiguous heading measurement.

### Keyframe heading drift correction
During straight-line walking, the gyroscope accumulates small heading errors (~2–4° per step due to walking oscillation). Every 15 frames (~0.5s), NavSight matches the current image against the last stored keyframe using the essential matrix. The rotation from this match provides a visual heading change that's compared to the gyro-integrated heading. If they differ by a small amount (< 20°), 30% of the difference is corrected.

This correction is **disabled during turns** (gyro rotation rate > 17°/s) because the visual heading extraction doesn't work well for the tilted phone geometry during fast turns.

---

## 10. Zero-Velocity Updates (ZUPT)

When the system detects that the user is stationary, it applies a **zero-velocity update**:

### How is "stationary" detected?
Using a chi-squared statistical test (OpenVINS approach):
1. Collect recent accelerometer and gyroscope samples
2. Compute the variance of each sensor
3. If both variances are below a threshold AND the optical flow is < 2.5 pixels AND no steps are detected → the user is stationary

### What happens during ZUPT?
1. **Velocity is set to zero** in the EKF (Extended Kalman Filter) — this prevents position drift when standing still
2. **Translation is frozen** — no position updates
3. **Heading still updates** — the user can turn in place, and the gyroscope captures this
4. **Gyro bias is refined** — when truly stationary, the gyro reading IS the bias. A gentle update (α=0.01) slowly corrects the bias estimate.

---

## 11. Global Position Update — Putting It All Together

Every frame, the global position is updated as:

```
x += displacement × sin(heading)    // East direction
z += displacement × cos(heading)    // North direction
```

Where **displacement** comes from one of two sources:

### When vision is good (pose_valid = true):
```
displacement = scale × ||t_vo||
```
The visual translation direction is already encoded in the heading. The displacement is capped at 2 m/s × dt to reject noise spikes.

### When vision fails (fallback to dead reckoning):
```
displacement = step_speed × dt
```
If the camera is obscured or tracking is lost, the step detector alone provides forward velocity. This keeps navigation working (at reduced accuracy) through brief vision outages.

### Coordinate system
- **Origin**: where the user started
- **+X = East**, **+Z = North** (heading = 0° points North)
- **+Y = Up** (from visual odometry, less accurate)

The initial heading aligns this coordinate system with the compass at startup.

---

## 12. GPS → Map Coordinates

NavSight starts with a known GPS position (latitude/longitude from the phone's last GPS fix). As the VIO estimates displacement in meters, these are converted to lat/lon offsets:

```
Δlatitude  = displacement_north / 111,320           (meters per degree at equator)
Δlongitude = displacement_east / (111,320 × cos(lat))
```

The `cos(lat)` correction accounts for longitude degrees being smaller at higher latitudes (meridians converge at the poles).

---

## 13. Camera-IMU Time Synchronization

The camera and IMU have slightly different clocks. A 5–20ms offset between them causes heading errors (the gyro rotation is applied at the wrong time). NavSight estimates this offset during startup:

1. For the first 60 frames (~2 seconds), record both the optical flow rate and gyro rate
2. Compute the **cross-correlation** at different time lags (-100ms to +100ms)
3. The lag with the highest correlation is the time offset
4. This offset is applied to all subsequent IMU integration windows

---

## 14. EKF (Extended Kalman Filter)

NavSight uses a **15-DOF error-state EKF** that maintains:
- 3D position, velocity, rotation (9 states)
- Gyroscope bias (3 states)
- Accelerometer bias (3 states)

The EKF serves two roles:
1. **Prediction**: when IMU data arrives, propagate the state forward using the physics model (rotation, velocity, position integration)
2. **Update**: when visual measurements arrive (scale, ZUPT), correct the state and reduce uncertainty

The covariance matrix tracks how uncertain each state is. After a ZUPT, velocity uncertainty drops to near zero. After many steps with good vision, scale uncertainty decreases.

---

## 15. Summary: One Frame, Start to Finish

Here's what happens every ~33ms when a new camera frame arrives:

1. **Convert to grayscale** and apply CLAHE (adaptive histogram equalization) for better contrast
2. **IMU preintegration**: integrate all gyro/accel samples between this frame and the last into a single rotation/velocity/position change
3. **Feature tracking**: use KLT optical flow (with IMU-predicted starting positions) to find where each feature moved
4. **Forward-backward check**: verify each match by tracking backwards
5. **Quality assessment**: count tracked features, compute mean flow, check for stationary/blur
6. **Essential matrix + RANSAC**: compute relative camera pose (R, t) from matched features
7. **Triangulation**: convert 2D feature matches into 3D points
8. **Scale update**: compare visual displacement to step displacement
9. **Heading update**: project gyro rotation onto gravity axis
10. **Position update**: advance global position by `scale × visual_displacement` in the heading direction
11. **Feature replenishment**: if too few features remain, detect new ones in sparse grid cells
12. **Output**: send position (lat, lon), heading, speed, and quality to the UI

---

## 16. Frequently Asked Questions

### Q: How accurate is the system?
Best measured result: **5.4% drift** (1.2m error on a 22m out-and-back walk). Typical accuracy is 5–10% of distance traveled.

### Q: What happens when the camera can't see anything?
The system falls back to **dead reckoning** using only the step detector and gyroscope. This works for short periods (~5-10 seconds) before drift becomes significant.

### Q: What's the difference between VIO and SLAM?
VIO (Visual-Inertial Odometry) estimates relative motion — "I moved 2 meters north since last frame." SLAM (Simultaneous Localization and Mapping) also builds a map and recognizes previously visited places (loop closure). NavSight currently uses VIO only.

### Q: Why not use two cameras (stereo)?
Stereo cameras can determine scale directly from the disparity between left/right images. But phone hardware only has one main camera. NavSight achieves scale through the IMU step detector instead.

### Q: How does the system handle different walking speeds?
The step detector measures step frequency in real-time. The stride model scales with frequency — faster stepping produces longer strides (biomechanical relationship). The system adapts from slow walking (~0.8 m/s) to jogging (~3 m/s).

### Q: What happens during a 180° turn?
The gyroscope detects the turning angular velocity, projects it onto gravity to get the yaw rate, and integrates it into the heading. During the turn, the keyframe heading correction is disabled (it would incorrectly reduce the measured turn angle). After the turn, the heading has changed by ~180° and subsequent steps move in the opposite direction.

### Q: Does the phone orientation matter?
No — because heading is computed by projecting gyro rotation onto the gravity vector, not the phone's Z-axis. The system works whether the phone is held upright (portrait), tilted (looking at screen), or nearly horizontal. The gravity vector is continuously tracked via a low-pass filtered accelerometer.
