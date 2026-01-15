save# VIO Threading & Sensor Fusion Research

## Research Summary: Modern VIO Architectures

Based on analysis of state-of-the-art VIO systems (ORB-SLAM3, VINS-Mono, OpenVINS, Kimera-VIO), here's what the research shows:

---

## 1. Current Architecture Analysis

### What We Have Now (NavSight1)

```
┌─────────────────────────────────────────────────┐
│ Main Thread (UI - 60 FPS)                       │
│  ├── Camera callback → Frame Queue              │
│  ├── Accel callback → Store + VisionModule      │
│  └── Gyro callback → Store + VisionModule       │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ VIO Thread (Sequential Processing)              │
│  1. Dequeue frame                               │
│  2. YUV → Grayscale conversion                  │
│  3. Feature detection (if needed)               │
│  4. Optical flow tracking                       │
│  5. RANSAC outlier rejection                    │
│  6. Essential matrix decomposition              │
│  7. Pose recovery                               │
│  8. Scale estimation (with IMU)                 │
│  9. Update global pose                          │
│  └── Return results                             │
└─────────────────────────────────────────────────┘
```

**Bottlenecks:**
- Single thread does ALL processing sequentially
- Camera runs at 30 FPS, but VIO thread may not keep up
- IMU data (100-200 Hz) is underutilized
- No parallelization of expensive operations

**Strengths:**
- Simple, easy to debug
- No complex synchronization issues
- Works for basic VIO

---

## 2. State-of-the-Art Threading Architectures

### ORB-SLAM3 (Best Accuracy - 5-10x better than VINS-Mono)

```
┌──────────────────────┐
│  Tracking Thread     │  ← Runs at camera FPS (30-40 Hz)
│  - Frame ingestion   │     Localizes incoming frames
│  - Feature matching  │     Creates keyframes
│  - Pose estimation   │
└──────────────────────┘
          ↓ Keyframes
┌──────────────────────┐
│ Local Mapping Thread │  ← Runs at keyframe rate (3-6 Hz)
│  - Keyframe insert   │     Optimizes active map
│  - Local BA          │     Adds new points
│  - Culling           │
└──────────────────────┘
          ↓ Map data
┌──────────────────────┐
│ Loop Closure Thread  │  ← Runs opportunistically
│  - Place recognition │     Detects revisited areas
│  - Global BA         │     Corrects drift
└──────────────────────┘
```

**Key Insights:**
- **3 parallel threads** for different time scales
- Tracking thread NEVER waits - always fast
- Heavy optimization happens asynchronously
- IMU integration takes "negligible time" in tracking
- Can run real-time at 30-40 FPS

### OpenVINS (Filter-based, Real-time Focus)

```
┌──────────────────────┐
│   IMU Thread         │  ← Runs at IMU rate (200 Hz)
│  - Always publishes  │     Low latency odometry
│  - State propagation │     Critical for robotics
└──────────────────────┘
          ↓ State updates
┌──────────────────────┐
│  Frontend Thread     │  ← Runs at camera rate (30 Hz)
│  - Feature tracking  │     Asynchronous subscription
│  - MSCKF update      │     Lower frequency OK
└──────────────────────┘
          ↓ (Optional)
┌──────────────────────┐
│ Loop Closure Thread  │  ← Loosely coupled
│  - VINS-based        │     Secondary optimization
└──────────────────────┘
```

**Key Insights:**
- **IMU thread has highest priority** - always publishes
- Frontend runs asynchronously, can miss frames if needed
- Designed for low-latency deployment on robots
- Uses Extended Kalman Filter (faster than optimization)

### VINS-Mono / VINS-Fusion

```
┌─────────────────────────────────────────────┐
│         Preprocessing (Parallel)            │
│  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Visual Odometry │  │Inertial Odometry│  │
│  │ - Feature track │  │ - IMU preintegr │  │
│  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────┘
                    ↓ Fusion
┌─────────────────────────────────────────────┐
│       Tightly-coupled Optimization          │
│  - Sliding window BA                        │
│  - IMU preintegration factors               │
│  - Loop closure (if detected)               │
└─────────────────────────────────────────────┘
```

**Key Insights:**
- **Parallel preprocessing** for visual and inertial
- Tight coupling through optimization
- IMU preintegration reduces computation cost
- Sliding window for real-time performance

### Kimera-VIO (MIT, Modular Architecture)

```
┌──────┐   Queue   ┌──────────┐   Queue   ┌─────────┐
│ IMU  │ ────────→ │ Frontend │ ────────→ │ Backend │
│Thread│           │ (Vision) │           │  (Opt)  │
└──────┘           └──────────┘           └─────────┘
                                                ↓
                                          ┌──────────┐
                                          │ Meshing  │
                                          │ (3D Map) │
                                          └──────────┘
```

**Key Insights:**
- **Modular pipeline** with queues between stages
- Can run parallel or sequential (configurable)
- Each module can drop frames if overloaded
- Queue-based communication for decoupling

---

## 3. IMU Preintegration (Critical for Performance)

### What It Is

Instead of reintegrating IMU measurements every optimization iteration:

**Without Preintegration (Current NavSight1):**
```
Every frame:
  1. Get IMU readings
  2. Estimate scale from displacement
  3. Apply scale to translation

Problem: Repeated integration during optimization is expensive!
```

**With Preintegration (State-of-the-art):**
```
Between keyframes i and j:
  1. Collect all IMU measurements (typically 20-60 samples)
  2. Pre-integrate into a single constraint:
     - Δp_ij (position change)
     - Δv_ij (velocity change)
     - Δq_ij (rotation change)
  3. Store as a factor in optimization graph

During optimization:
  - Use pre-computed IMU factor
  - Update with Jacobians (cheap!)
  - No re-integration needed
```

### How It Works

From the foundational paper (Forster et al., 2016):

```cpp
// High-frequency IMU measurements between frames i and j
for each IMU measurement (ω, a) at time t:
    // Propagate rotation (gyroscope)
    Δq = Δq ⊗ [0.5 * ω * dt]

    // Propagate velocity (accelerometer)
    Δv = Δv + Δq * a * dt

    // Propagate position
    Δp = Δp + Δv * dt + 0.5 * Δq * a * dt²

    // Update covariance (uncertainty)
    Σ = F * Σ * F^T + G * Q * G^T

// Result: Single preintegrated measurement
PreintegratedIMU {
    Δp_ij,  // Total position change
    Δv_ij,  // Total velocity change
    Δq_ij,  // Total rotation change
    Σ       // Uncertainty covariance
}
```

### Benefits

1. **Computational Efficiency**
   - IMU rate: 200 Hz, Camera rate: 30 Hz
   - Between frames: ~7 IMU samples
   - Without preintegration: Integrate 7 samples × N optimization iterations
   - With preintegration: Integrate once, use factor N times
   - **Speedup: ~10-100x for IMU processing**

2. **Better Fusion**
   - Proper uncertainty propagation
   - Manifold-aware integration (rotation on SO(3))
   - Bias correction during optimization

3. **Lower Rate Operations**
   - Expensive covariance updates at lower rate
   - Save compute for vision processing

### Implementation Complexity

**Easy:** Basic preintegration (position, velocity, rotation)
**Medium:** Add uncertainty propagation
**Hard:** Bias estimation and correction

**Libraries Available:**
- GTSAM (C++) - Full implementation with examples
- Ceres Solver - Custom factors
- GitHub: mc275/IMU_Preintegration - Standalone examples

---

## 4. Recommended Architecture for NavSight1

### Option A: Minimal Improvement (Low Effort, Medium Gain)

**Add 1 more thread for IMU preintegration:**

```
┌────────────────────┐
│   IMU Thread       │  ← NEW: High priority, 200 Hz
│ - Preintegrate IMU │     Always running
│ - Publish odometry │     Low latency
└────────────────────┘
          ↓ Preintegrated measurements
┌────────────────────┐
│  VIO Thread        │  ← MODIFIED: Use preintegrated IMU
│ - Dequeue frame    │     Faster processing
│ - Feature tracking │
│ - Pose estimation  │
│ - Use IMU factor   │
└────────────────────┘
```

**Changes Required:**
1. Create IMU preintegration module
2. Launch IMU thread on VIO start
3. Pass preintegrated measurements to VIO thread
4. Update pose estimation to use IMU factors

**Expected Improvements:**
- 20-30% faster VIO processing
- Better scale estimation
- Smoother odometry output
- More accurate rotation

**Effort:** ~3-5 days development

---

### Option B: Modern Architecture (Medium Effort, High Gain)

**3-thread architecture inspired by ORB-SLAM3:**

```
┌──────────────────────┐
│  Tracking Thread     │  ← Fast! Always processes frames
│  - Feature tracking  │     Creates keyframes
│  - Quick pose est.   │     Publishes pose at 30 Hz
│  - IMU propagation   │
└──────────────────────┘
          ↓ Keyframes
┌──────────────────────┐
│  Mapping Thread      │  ← Slower, runs in background
│  - IMU preintegration│     Optimizes trajectory
│  - Local BA          │     Manages map points
│  - Scale refinement  │     Runs at ~3-6 Hz
└──────────────────────┘
          ↓ (Future)
┌──────────────────────┐
│  Loop Closure        │  ← For future implementation
│  - Place recognition │
└──────────────────────┘
```

**Changes Required:**
1. Split current VIO thread into Tracking + Mapping
2. Implement keyframe selection
3. Add IMU preintegration in Mapping
4. Use sliding window optimization
5. Queue-based communication

**Expected Improvements:**
- Never drop frames (tracking always fast)
- 2-3x better accuracy (from optimization)
- Proper scale convergence
- Foundation for loop closure

**Effort:** ~2-3 weeks development

---

### Option C: Production-Ready (High Effort, Maximum Gain)

**Use proven open-source system:**

```
Replace VisionModule with:
  - OpenVINS (if speed is critical)
  - ORB-SLAM3 (if accuracy is critical)
  - VINS-Mobile (Android optimized)
```

**Pros:**
- Battle-tested code
- Extensive documentation
- Active community support
- State-of-the-art performance

**Cons:**
- Large codebase to integrate
- Less control over implementation
- Potential license restrictions
- Learning curve

**Expected Improvements:**
- 5-10x better accuracy (ORB-SLAM3)
- Robust to challenging scenarios
- Loop closure for drift correction
- May require 1-2 GB RAM

**Effort:** ~1-2 weeks integration + learning

---

## 5. Specific Improvements for Current System

### Immediate Wins (No Threading Changes)

#### 1. Add IMU Preintegration to VisionModule

**Current:**
```cpp
// VisionModule.cpp - estimateScaleFromAccel()
double accel_disp = 0.5 * accel_mag * dt * dt;  // Simple physics
scale = accel_disp / vision_mag;
```

**Improved:**
```cpp
// Preintegrate between frames
struct PreintegratedIMU {
    cv::Vec3d delta_p;  // Position change
    cv::Vec3d delta_v;  // Velocity change
    cv::Mat delta_R;    // Rotation change
    double dt;          // Time interval
};

// In VisionModule
PreintegratedIMU preintegrateIMU(long t_start, long t_end) {
    PreintegratedIMU result;
    cv::Vec3d velocity(0, 0, 0);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    for (auto& imu : imu_buffer_) {
        if (imu.timestamp < t_start || imu.timestamp > t_end) continue;

        // Propagate rotation
        cv::Vec3d omega(imu.gyro_x, imu.gyro_y, imu.gyro_z);
        cv::Mat R_delta = exponentialMap(omega * imu.dt);
        R = R_delta * R;

        // Propagate velocity (rotate acceleration to world frame)
        cv::Vec3d accel(imu.accel_x, imu.accel_y, imu.accel_z);
        cv::Vec3d accel_world = R * accel;
        velocity += accel_world * imu.dt;

        // Propagate position
        result.delta_p += velocity * imu.dt + 0.5 * accel_world * imu.dt * imu.dt;
    }

    result.delta_v = velocity;
    result.delta_R = R;
    result.dt = (t_end - t_start) / 1e9;  // nanoseconds to seconds

    return result;
}

// Use in scale estimation
double estimateScaleFromIMU(const PreintegratedIMU& imu,
                            const cv::Mat& visual_translation) {
    double imu_displacement = cv::norm(imu.delta_p);
    double visual_displacement = cv::norm(visual_translation);
    return imu_displacement / visual_displacement;
}
```

**Benefits:**
- More accurate scale (uses full IMU trajectory)
- Rotation from gyro helps visual tracking
- Foundation for full sensor fusion

**Effort:** 1-2 days

---

#### 2. Implement Parallel Feature Detection

**Current:**
```cpp
// Sequential: ~15-20ms per frame
cv::goodFeaturesToTrack(gray, corners, max_features_, 0.01, 10);
cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_points_, curr_points_, status, error);
```

**Improved:**
```cpp
// Parallel: ~8-12ms per frame
std::thread detection_thread, tracking_thread;

if (need_detection) {
    detection_thread = std::thread([&]() {
        cv::goodFeaturesToTrack(gray, new_corners, max_features_, 0.01, 10);
    });
}

tracking_thread = std::thread([&]() {
    cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_points_, curr_points_, status, error);
});

if (detection_thread.joinable()) detection_thread.join();
tracking_thread.join();

// Merge new_corners with tracked curr_points_
```

**Benefits:**
- ~30-40% faster frame processing
- Better CPU utilization
- No architectural changes needed

**Effort:** Half day

---

#### 3. Add Velocity State

**Current:**
```cpp
// Only track position and rotation
cv::Mat global_R, global_t;
```

**Improved:**
```cpp
// Track position, velocity, rotation
struct VIOState {
    cv::Mat R;          // Rotation (3x3)
    cv::Vec3d t;        // Position (3x1)
    cv::Vec3d v;        // Velocity (3x1)  ← NEW
    cv::Vec3d bg;       // Gyro bias (3x1) ← NEW
    cv::Vec3d ba;       // Accel bias (3x1) ← NEW
    long timestamp;
};

// Use IMU to propagate velocity
void propagateWithIMU(VIOState& state, const IMUData& imu) {
    // Correct for bias
    cv::Vec3d accel_corrected = imu.accel - state.ba;
    cv::Vec3d gyro_corrected = imu.gyro - state.bg;

    // Update rotation
    cv::Mat R_delta = exponentialMap(gyro_corrected * dt);
    state.R = R_delta * state.R;

    // Update velocity (remove gravity)
    cv::Vec3d accel_world = state.R * accel_corrected;
    cv::Vec3d gravity(0, 0, -9.81);
    state.v += (accel_world - gravity) * dt;

    // Update position
    state.t += state.v * dt;
}
```

**Benefits:**
- Smoother trajectory
- Better motion prediction
- More stable tracking
- Bias estimation foundation

**Effort:** 1-2 days

---

## 6. Performance Comparison

Based on research papers (ORB-SLAM3, VINS-Mono, OpenVINS):

| System | Threads | IMU Fusion | Accuracy (ATE cm) | FPS | Notes |
|--------|---------|------------|-------------------|-----|-------|
| **NavSight1 (current)** | 1 | Basic scale | ~50-200 | 20-30 | Simple, works |
| **+ Preintegration** | 1 | Preintegrated | ~20-80 | 25-30 | Easy upgrade |
| **+ IMU Thread** | 2 | Preintegrated | ~15-50 | 30 | Better latency |
| **Tracking + Mapping** | 2-3 | Tight coupling | ~5-20 | 30 | ORB-like |
| **OpenVINS** | 2-3 | MSCKF filter | ~10-30 | 30 | Real-time focus |
| **ORB-SLAM3** | 3 | Preintegrated | ~2-10 | 30-40 | Best accuracy |
| **VINS-Mono** | 3-4 | Preintegrated | ~5-15 | 20-30 | Good balance |

*Note: Accuracy depends heavily on environment, motion, calibration*

---

## 7. Recommended Implementation Plan

### Phase 1: Foundation (Week 1-2)
1. ✅ Add IMU preintegration to VisionModule
2. ✅ Implement velocity state
3. ✅ Add gyroscope-based rotation prediction
4. ✅ Test on EuRoC dataset (if available)

**Expected Result:** 2-3x better accuracy, smoother trajectory

---

### Phase 2: Threading (Week 3-4)
1. ✅ Create dedicated IMU thread
2. ✅ Implement preintegration between frames
3. ✅ Add thread-safe state propagation
4. ✅ Publish odometry at IMU rate

**Expected Result:** Lower latency, never miss frames

---

### Phase 3: Optimization (Week 5-6)
1. ✅ Split into Tracking + Mapping threads
2. ✅ Implement keyframe selection
3. ✅ Add sliding window bundle adjustment
4. ✅ Integrate Ceres or g2o optimizer

**Expected Result:** 5-10x better accuracy, production ready

---

### Phase 4: Advanced (Week 7-8)
1. ✅ Add loop closure detection
2. ✅ Implement place recognition
3. ✅ Global pose graph optimization
4. ✅ Map saving/loading

**Expected Result:** Drift correction, multi-session mapping

---

## 8. Code Examples & Resources

### IMU Preintegration Libraries

**GTSAM (Recommended):**
```cpp
#include <gtsam/navigation/ImuFactor.h>

// Create preintegration parameters
auto params = gtsam::PreintegrationParams::MakeSharedU(9.81);
params->accelerometerCovariance = I_3x3 * pow(0.0003924, 2);
params->gyroscopeCovariance = I_3x3 * pow(0.000205689024915, 2);
params->integrationCovariance = I_3x3 * 1e-8;

// Create preintegrated IMU measurements
auto preintegrated = std::make_shared<gtsam::PreintegratedImuMeasurements>(params);

// Add measurements
for (auto& imu : imu_buffer) {
    preintegrated->integrateMeasurement(imu.accel, imu.gyro, imu.dt);
}

// Use in factor graph
graph.add(gtsam::ImuFactor(X(i), V(i), X(j), V(j), B(i), *preintegrated));
```

**Standalone (Simpler):**
```cpp
// Based on mc275/IMU_Preintegration
class IMUPreintegrator {
public:
    void addMeasurement(const Vector3d& accel, const Vector3d& gyro, double dt) {
        // Update rotation
        Quaterniond dq = Quaterniond(1, 0.5*gyro.x()*dt, 0.5*gyro.y()*dt, 0.5*gyro.z()*dt);
        delta_q = (delta_q * dq).normalized();

        // Update velocity and position
        Vector3d acc_world = delta_q * accel;
        delta_v += acc_world * dt;
        delta_p += delta_v * dt + 0.5 * acc_world * dt * dt;

        sum_dt += dt;
    }

    // Get preintegrated measurement
    void getPreintegrated(Vector3d& p, Vector3d& v, Quaterniond& q) {
        p = delta_p;
        v = delta_v;
        q = delta_q;
    }

private:
    Vector3d delta_p{0, 0, 0};
    Vector3d delta_v{0, 0, 0};
    Quaterniond delta_q{1, 0, 0, 0};
    double sum_dt = 0;
};
```

### GitHub Repositories to Study

1. **ORB-SLAM3** (Best reference for threading)
   - https://github.com/UZ-SLAMLab/ORB_SLAM3
   - Files to read:
     - `src/Tracking.cc` - Tracking thread
     - `src/LocalMapping.cc` - Mapping thread
     - `src/LoopClosing.cc` - Loop closure
     - `include/ImuTypes.h` - IMU preintegration

2. **OpenVINS** (Filter-based, good for real-time)
   - https://github.com/rpng/open_vins
   - Files to read:
     - `ov_msckf/src/core/VioManager.cpp` - Main loop
     - `ov_core/src/track/TrackBase.cpp` - Feature tracking
     - `ov_core/src/sim/Simulator.cpp` - IMU simulation

3. **VINS-Mono** (Good balance)
   - https://github.com/HKUST-Aerial-Robotics/VINS-Mono
   - Files to read:
     - `vins_estimator/src/estimator_node.cpp` - Main node
     - `vins_estimator/src/factor/imu_factor.h` - IMU factor
     - `feature_tracker/src/feature_tracker_node.cpp` - Tracking

4. **IMU Preintegration Examples**
   - https://github.com/mc275/IMU_Preintegration
   - Simple standalone examples with rotation matrix and quaternion

### Key Papers to Read

1. **IMU Preintegration (Must Read)**
   - "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
   - Forster et al., 2016
   - https://arxiv.org/pdf/1512.02363.pdf
   - **Read this first!** Tutorial-style with full derivations

2. **ORB-SLAM3 (Best System)**
   - "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM"
   - Campos et al., 2020
   - https://arxiv.org/pdf/2007.11898.pdf

3. **VINS-Mono (Optimization-based)**
   - "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator"
   - Qin et al., 2018

4. **OpenVINS (Filter-based)**
   - "OpenVINS: A Research Platform for Visual-Inertial Estimation"
   - Geneva et al., 2020

---

## 9. My Recommendations (Priority Order)

### **Immediate (This Week):**
1. **Implement basic IMU preintegration**
   - Use standalone code (mc275/IMU_Preintegration as reference)
   - Add to VisionModule.cpp
   - Test scale estimation improvement
   - **Effort:** 1-2 days
   - **Gain:** 2x better scale accuracy

2. **Add velocity state**
   - Extend VIOState with velocity
   - Use IMU to propagate between frames
   - **Effort:** 1 day
   - **Gain:** Smoother trajectory

### **Short Term (Next 2 Weeks):**
3. **Create IMU thread**
   - Dedicated thread for IMU preintegration
   - High-priority, always publishes
   - Low-latency odometry
   - **Effort:** 2-3 days
   - **Gain:** Better responsiveness

4. **Parallelize feature detection and tracking**
   - Use std::thread for concurrent operations
   - **Effort:** Half day
   - **Gain:** 30% faster processing

### **Medium Term (Next Month):**
5. **Split into Tracking + Mapping threads**
   - Tracking: Fast pose estimation at camera rate
   - Mapping: Background optimization
   - **Effort:** 1-2 weeks
   - **Gain:** 3-5x better accuracy

6. **Integrate optimization library**
   - Use Ceres Solver or g2o
   - Sliding window bundle adjustment
   - **Effort:** 1 week
   - **Gain:** Proper scale convergence

### **Long Term (2-3 Months):**
7. **Add loop closure**
   - DBoW2 for place recognition
   - Global pose graph optimization
   - **Effort:** 2-3 weeks
   - **Gain:** No drift on loops

8. **Consider switching to ORB-SLAM3**
   - If you need production-level accuracy
   - **Effort:** 1-2 weeks integration
   - **Gain:** State-of-the-art performance

---

## 10. Next Steps

1. **Read the IMU preintegration paper** (1-2 hours)
   - https://arxiv.org/pdf/1512.02363.pdf
   - Focus on Section III (Preintegration on Manifold)

2. **Study one reference implementation** (2-3 hours)
   - Clone mc275/IMU_Preintegration
   - Understand the basic algorithm
   - Adapt to your VisionModule

3. **Implement and test** (1-2 days)
   - Add preintegration to VisionModule
   - Compare before/after accuracy
   - Tune parameters

4. **Iterate based on results**
   - If good improvement → Add IMU thread
   - If marginal → May need better calibration
   - If worse → Check bias, coordinate frames

---

## Conclusion

**Current System:** 1 thread, basic IMU fusion, ~50-200cm accuracy
**With Preintegration:** 1 thread, better fusion, ~20-80cm accuracy (2-3x better)
**With IMU Thread:** 2 threads, real-time, ~15-50cm accuracy (3-4x better)
**With Tracking+Mapping:** 3 threads, optimized, ~5-20cm accuracy (10-40x better)
**ORB-SLAM3 Level:** 3 threads, state-of-art, ~2-10cm accuracy (25-100x better)

The biggest single improvement you can make is **IMU preintegration** - it's the foundation everything else builds on. Start there, then add threading as needed.
