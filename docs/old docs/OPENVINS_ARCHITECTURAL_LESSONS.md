# Advanced OpenVINS Architectural Lessons for NavSight1

## 1. Executive Summary
[OpenVINS](https://github.com/rpng/open_vins) is an industry-leading, filter-based Visual-Inertial Odometry (VIO) framework developed by the Robot Perception and Navigation Group (RPNG). While NavSight1 currently utilizes a custom EKF paired with explicit feature triangulation (relying on GPS for scale correction), OpenVINS achieves drift-free, robotics-grade state estimation purely through advanced math and rigorous sensor modeling.

This document serves as an **implementation blueprint for Claude AI** to upgrade the `NavSight1` mobile VIO engine. By systematically applying the five core paradigms from OpenVINS outlined below, the NavSight1 system can mathematically eliminate the root causes of drift, allowing it to reach 98%+ accuracy even during GPS-denied indoor navigation or rapid smartphone movement.

---

## 2. Online Spatiotemporal Calibration (Time-Offset Estimation)

### The Problem in Mobile VIO
Android smartphones do not hardware-synchronize their camera exposures with their IMU sampling. The camera frame timestamp and IMU timestamp pass through different OS layers, creating a variable latency (typically 5–20 ms). If the EKF assumes they are aligned, it pairs visual motion with the wrong inertial forces, causing massive scale and heading drift during rotation or fast movement.

### The OpenVINS Solution
OpenVINS treats the camera-to-IMU time offset ($t_{offset}$) as a calibratable state variable within the EKF itself. It dynamically shifts the IMU integration window to align with the visual data.

**OpenVINS Code Example:**
```cpp
// From UpdaterZeroVelocity.cpp / Propagator.cpp
// Retrieve the dynamically estimated time offset from the state
double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

// Shift the IMU integration window so the visual and inertial data align temporally
double time0 = state->_timestamp + last_prop_time_offset;
double time1 = current_cam_timestamp + t_off_new;

// Select IMU readings matching the exact exposure time of the camera
std::vector<ov_core::ImuData> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);
```

### Claude Implementation Guide
**File to Modify:** `app/src/main/cpp/EKFState.h`, `EKFState.cpp`, `IMUPreintegrator.cpp`
1. **State Expansion:** Add a `double t_offset_cam_imu;` to the `EKFState` vector. Initialize it to 0.0 (or a known average Android latency like 0.010s).
2. **Preintegration Adjustment:** In `Tracker::processFrame`, before calling the `IMUPreintegrator`, adjust the target integration time: `target_time = frame_timestamp + ekf.t_offset_cam_imu`.
3. **Jacobian Update:** During the visual measurement update step, compute the derivative of the reprojection error with respect to the time offset (chain rule: error w.r.t pose * pose w.r.t time (velocity)). This allows the EKF to converge on the true hardware latency.

---

## 3. First-Estimate Jacobians (FEJ) for Observability

### The Problem in Mobile VIO
A standard EKF linearizes its equations around the *latest* state estimate. In VIO, global yaw and absolute position are "unobservable" (you don't know where North is just from a camera/IMU). Re-linearizing around changing estimates injects false information into the covariance matrix along these unobservable directions, tricking the EKF into thinking it is certain of its heading, which inevitably leads to drift.

### The OpenVINS Solution
OpenVINS strictly enforces **First-Estimate Jacobians (FEJ)**. Once a pose or feature is initialized in the filter, the system permanently locks the linearization point for unobservable dimensions.

**OpenVINS Conceptual Example:**
```cpp
// In StateHelper::EKFUpdate
// When computing the Jacobian (H matrix) for a measurement, we do NOT use the current yaw.
// Instead, we use the yaw that was estimated the first time this state was added to the filter.
Eigen::Matrix3d R_GtoI_FEJ = get_first_estimate_rotation(state_id);

// The Jacobian w.r.t gravity or position uses the locked FEJ rotation
H.block(row, col, 3, 3) = -skew_x(R_GtoI_FEJ * _gravity); 
```

### Claude Implementation Guide
**File to Modify:** `app/src/main/cpp/EKFState.cpp`
1. **Cache Linearization Points:** When a new state is added (e.g., a new cloned camera pose), save its `initial_yaw` and `initial_position` alongside the current estimate.
2. **Force FEJ:** When constructing the $H$ matrix for the visual measurement update or IMU propagation, explicitly substitute the `current_yaw` with the `initial_yaw` for all derivatives related to global orientation and gravity. This mathematically prevents the covariance from artificially shrinking.

---

## 4. Multi-State Constraint Kalman Filter (MSCKF)

### The Problem in Mobile VIO
According to `COMPLETE_IMPLEMENTATION_SUMMARY.md`, NavSight1 triangulates 3D points explicitly. Putting 3D map points into an EKF state vector is computationally crushing ($O(N^3)$ complexity). To run in real-time, the system has to limit the number of features tracked, drastically reducing accuracy.

### The OpenVINS Solution
Instead of putting 3D points into the state vector, OpenVINS keeps a "sliding window" of the last $N$ camera poses (the MSCKF formulation). When a feature is lost, it is triangulated, the residuals are computed across all poses that saw it, and the feature is mathematically marginalized using null-space projection. The update is applied strictly to the camera poses.

**OpenVINS Code Example:**
```cpp
// From UpdaterHelper::measurement_compress_inplace
// Instead of a massive H matrix containing 3D point states, MSCKF projects the 
// visual residual onto the left null-space of the feature Jacobian.

// 1. Compute QR decomposition of the feature Jacobian (H_f)
Eigen::HouseholderQR<Eigen::MatrixXd> qr(H_f);
Eigen::MatrixXd Q = qr.householderQ();

// 2. Extract the null-space basis (Q_0)
Eigen::MatrixXd Q_0 = Q.rightCols(Q.cols() - 3);

// 3. Project the pose Jacobian (H_x) and residual (res) to remove the feature state dependency
H_x_projected = Q_0.transpose() * H_x;
res_projected = Q_0.transpose() * res;

// 4. Apply standard EKF update using the much smaller, feature-free matrices
StateHelper::EKFUpdate(state, H_x_projected, res_projected);
```

### Claude Implementation Guide
**File to Modify:** `app/src/main/cpp/Tracker.cpp`, `EKFState.cpp`
1. **Sliding Window:** Modify `EKFState` to hold an array/deque of the last 15 camera poses (position + quaternion).
2. **Feature Marginalization:** When `FeatureManager` detects a feature has been lost, triangulate it using all historical observations. Construct the Jacobian matrix `H_x` w.r.t the sliding window poses.
3. **Null-Space Projection:** Apply the QR decomposition technique shown above. Update the EKF state using only the projected `H_x` and `res`. Do NOT store the 3D point in the EKF state vector.

---

## 5. Advanced IMU Intrinsic Calibration

### The Problem in Mobile VIO
Smartphone MEMS IMUs suffer from poor orthogonality (misalignment), temperature-dependent scale factors, and g-sensitivity (accelerometers responding to linear acceleration). NavSight1 only estimates basic IMU bias (`bias_a`, `bias_g`), which is insufficient for long-duration pedestrian tracking.

### The OpenVINS Solution
OpenVINS includes full intrinsic calibration states in the EKF, continuously estimating the scale factors and misalignment matrices while the user moves.

**OpenVINS Code Example:**
```cpp
// Applying estimated intrinsic calibration to raw IMU data
// Dw = Gyroscope Scale/Misalignment Matrix
// Da = Accelerometer Scale/Misalignment Matrix
// Tg = G-sensitivity Matrix

Eigen::Vector3d a_true = Da * (imu.am - state->_imu->bias_a());
Eigen::Vector3d w_true = Dw * (imu.wm - state->_imu->bias_g() - Tg * a_true);
```

### Claude Implementation Guide
**File to Modify:** `app/src/main/cpp/IMUPreintegrator.cpp`, `EKFState.cpp`
1. **State Expansion:** Add a $3 \times 3$ upper-triangular matrix for `Da` and `Dw` to `EKFState`.
2. **Apply Calibration:** In `IMUPreintegrator::integrate()`, multiply the raw IMU readings by these matrices (as shown in the code above) *before* performing numeric integration.
3. **Filter Update:** Allow the visual measurement update to correct `Da` and `Dw` over time.

---

## 6. Statistical Zero-Velocity Detection (ZUPT)

### The Problem in Mobile VIO
In `Tracker.cpp`, NavSight1 relies on hardcoded thresholds (`gyro_norm < ZUPT_GYRO_THRESH`). These thresholds are incredibly brittle because every Android device has a different IMU noise floor. A threshold that works on a Pixel 8 might trigger false stops on a Galaxy S21.

### The OpenVINS Solution
OpenVINS uses statistical variance testing against the calibrated noise density of the specific device. It computes the measurement residuals and checks if they fall within a mathematically sound $\chi^2$ (Chi-squared) distribution limit.

**OpenVINS Code Example:**
```cpp
// From UpdaterZeroVelocity.cpp
// Construct a residual based on the assumption that true angular velocity and acceleration are zero/gravity.
res.block(6 * i + 0, 0, 3, 1) = -w_omega * w_hat;
res.block(6 * i + 3, 0, 3, 1) = -w_accel * (a_hat - state->_imu->Rot() * _gravity);

// Chi-Squared Gating Test
double chi2 = res.transpose() * S_inv * res;
if (chi2 < chi2_threshold_95_percent) {
    // Statistically confirmed stationary! Apply EKF Update to freeze velocity and scale.
    StateHelper::EKFUpdate(state, H, res, R);
}
```

### Claude Implementation Guide
**File to Modify:** `app/src/main/cpp/Tracker.cpp`, `EKFState.cpp`
1. **Statistical Variance Check:** Instead of `gyro_norm < 0.04`, buffer the last 15 IMU readings. Calculate their variance.
2. **Chi-Squared Gating:** Compare the variance to the IMU noise density parameter (`imu_noise_g`, `imu_noise_a`). If the variance is statistically indistinguishable from the sensor's baseline noise (using a $\chi^2$ distribution threshold), trigger `EKFState::updateZUPT()`.
3. **Update Application:** Apply a pseudo-measurement to the EKF where velocity = 0, clamping the scale and position drift instantly.

---

## Final Directive for Claude
Your objective is to integrate these five pillars into the `NavSight1` codebase. Begin with **Online Spatiotemporal Calibration (Time-Offset)** and **Statistical ZUPT**, as these two fixes require the least architectural refactoring while providing the most massive immediate reduction in drift for smartphone-based systems. Proceed to **First-Estimate Jacobians** to ensure long-term filter stability.

---

## 7. Similar Android VIO Projects for Research

If further architectural references are needed beyond OpenVINS, the following GitHub projects are specifically built for Android smartphones and utilize C++, the Android NDK, and OpenCV (similar to NavSight1):

1.  **[VINS-Mobile-Android](https://github.com/jannismoeller/VINS-Mobile-Android)**
    *   **Description:** An optimization-based sliding window estimator (port of the famous HKUST iOS app).
    *   **Why study it:** It represents the gold standard for handling Android camera-IMU synchronization, loop closure, and relocalization in a mobile environment.

2.  **[Android-VIOTester](https://github.com/AaltoML/android-viotester)**
    *   **Description:** A benchmark application developed by Aalto University for testing VIO algorithms on Android hardware.
    *   **Why study it:** Excellent for reviewing dataset recording frameworks (Images + IMU + Ground Truth) and real-time UI debugging of feature tracks.

3.  **[ORB_SLAM2-Android](https://github.com/muziyongshixin/ORB-SLAM2-based-AR-on-Android)**
    *   **Description:** An Android port of the industry-standard ORB-SLAM2 visual SLAM system.
    *   **Why study it:** Provides architectural best practices for managing massive C++ maps (Graph SLAM) in a background thread without blocking the Android UI thread.

4.  **[LSD-SLAM on Android](https://github.com/omair18/LSD-SLAM-Android)**
    *   **Description:** A port of Large-Scale Direct Monocular SLAM (LSD-SLAM).
    *   **Why study it:** Uses direct pixel intensities instead of feature extraction. Useful reference if `FeatureManager` performance drops in low-texture environments like white hallways.

5.  **[Google ARCore SDK](https://github.com/google-ar/arcore-android-sdk)**
    *   **Description:** Google's official (closed-source backend) AR library.
    *   **Why study it:** The sample apps provide the definitive guide on safely passing Camera2 API frames and high-rate IMU events from Android Java/Kotlin down to the C++ NDK layer without dropping frames or corrupting lifecycle state.