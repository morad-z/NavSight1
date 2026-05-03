# NavSight Parallel VIO Refactor Plan

## 1. Goal

Transform the current sequential VIO pipeline into a three-thread parallel architecture (like NVIDIA cuVSLAM), then add drift reduction on top. The result is faster tracking, smoother UI, and reduced position drift.

**Current (sequential):** Everything blocks on the camera thread. One frame can't start until the previous finishes.

```
Camera frame → [YUV→gray + IMU + optical flow + essential matrix + feature detect + pose] → next frame
               ←————————————————————— 25-30ms blocked ———————————————————————→
```

**Target (parallel):**

```
IMU/Sensor:   |||||||||||||||||||||||||||||||||||||||  (200Hz, never waits for camera)
Tracker:      [flow+pose]      [flow+pose]            (30-60fps, fast path only)
Mapper:           [detect+BA]         [detect+BA]      (2-5fps, heavy work in background)
```

---

## 2. Current Bottlenecks

- **Sequential core:** `processFrame()` is a single blocking call (~25-30ms) that does everything
- **Feature detection on hot path:** `goodFeaturesToTrack` (~5ms) runs inline when features drop below 150
- **30Hz pose output:** UI only gets position updates at camera framerate (jerky on fast motion)
- **Frame-to-frame only:** Each frame matches against the previous frame only — errors compound across every frame
- **No loop closure:** Walking back to the start shows meters of drift with no correction

---

## 3. Architecture Overview

### Three Threads

| Thread | Rate | Priority | Responsibility |
|--------|------|----------|----------------|
| **IMU / Sensor** | 200-500 Hz | Highest (Android sensor thread) | Dead reckoning, smooth pose for UI |
| **Tracker** | 30-60 fps | High (camera thread) | Optical flow, essential matrix, rotation fusion, pose update |
| **Mapper** | 2-5 fps | Background | Feature detection, keyframe management, local BA, loop closure |

### Data Flow

```
Android Sensors (200Hz) ──→ IMUPreintegrator (buffer)
                         ──→ WorldState::propagate() (200Hz dead reckoning)

Android Camera (30fps)  ──→ Tracker::processFrame()
                              ├── reads IMU buffer via imu_.integrate()
                              ├── optical flow + essential matrix + pose
                              ├── WorldState::applyVisualCorrection()
                              └── pushes KeyframeData to queue (when features low)

Mapper thread (2-5fps)  ──→ pops KeyframeData from queue
                              ├── goodFeaturesToTrack
                              ├── Tracker::injectFeatures()
                              ├── local bundle adjustment (Phase 4)
                              └── loop closure detection (Phase 4)

UI thread (60-120Hz)    ──→ WorldState::getLatestPose() (smooth interpolation)
```

---

## 4. Implementation Phases

### Phase 2: Tracking / Mapping Thread Split

**Goal:** Remove feature detection from the camera thread. Stabilize framerate to 40-60fps.

#### What stays in Tracker (fast path)
- YUV-to-gray conversion
- Camera matrix K construction
- IMU integrate for the inter-frame interval
- Forward + backward optical flow (LK 31x31, 4 pyramids)
- Essential matrix RANSAC + recoverPose
- Adaptive rotation fusion (gyro + camera)
- Step-based scale estimation
- Global pose update (2D heading-based)

#### What moves to Mapper (background)
- `goodFeaturesToTrack` (currently VisionModule.cpp line 520)
- Duplicate point filtering
- Keyframe promotion decisions

#### New Files

| File | ~Lines | Purpose |
|------|--------|---------|
| `app/src/main/cpp/ThreadSafeQueue.h` | 60 | Bounded SPSC queue (size 4) between Tracker and Mapper |
| `app/src/main/cpp/Tracker.h` | 80 | Tracker class declaration |
| `app/src/main/cpp/Tracker.cpp` | 400 | Extracted from VisionModule::processFrame(), adds feature injection |
| `app/src/main/cpp/Mapper.h` | 50 | Mapper class declaration |
| `app/src/main/cpp/Mapper.cpp` | 120 | Background thread: feature detection + injection |
| `app/src/main/cpp/VioEngine.h` | 60 | Top-level orchestrator replacing VisionModule |
| `app/src/main/cpp/VioEngine.cpp` | 80 | Wires Tracker + Mapper + IMUPreintegrator |

#### Modified Files

| File | Change |
|------|--------|
| `native-lib.cpp` | Replace `VisionModule*` with `VioEngine*` |
| `app/CMakeLists.txt` | Add new .cpp files to build |

#### Key Design Decisions

**Feature injection pattern:** Mapper detects features on a keyframe and sends them to Tracker via a small `inject_mutex_` critical section. Tracker merges injected features into its tracking set between frames.

**Emergency fallback:** Tracker keeps a `goodFeaturesToTrack` fallback at half-threshold (75 instead of 150). This ensures tracking never fully fails even if Mapper stalls. In practice, Mapper replenishes before features drop this low.

**Keyframe queue:** Bounded at 4 entries. If Mapper falls behind, old keyframes are dropped (not queued indefinitely). Tracker never blocks on the queue.

#### Threading + Mutex Strategy

| Mutex | Protects | Contention |
|-------|----------|------------|
| `Tracker::mutex_` | `prev_gray_`, `prev_pts_` | Camera thread only |
| `Tracker::pose_mutex_` | `global_R_`, `global_t_`, `smooth_scale_` | Camera writes, JNI reads |
| `Tracker::inject_mutex_` | `injected_pts_` (NEW) | Mapper writes (2-5Hz), Tracker reads (30Hz) |
| `IMUPreintegrator::mutex_` | sensor buffers, step state | Sensor writes (200Hz), Tracker reads (30Hz) |
| `ThreadSafeQueue::mutex_` | internal queue | Tracker pushes (occasional), Mapper blocks |

Lock ordering (always acquire in this order): `pose_mutex_` > `mutex_` > `inject_mutex_` > queue > imu

#### Tests

| Test | What it validates |
|------|-------------------|
| TrackerBasicFlow | Tracker alone produces output identical to current VisionModule |
| MapperFeatureInjection | Mapper detects features and injects into Tracker |
| TrackerMapperIntegration | Full VioEngine, 90 frames of walking, position grows |
| MapperFallsBehind | 10 rapid keyframes, Tracker still outputs at 30fps |
| ConcurrentResetSafety | reset() during Mapper processing: no crash/deadlock |
| FeatureCountRecovery | Features drop to 0 (blank wall), Mapper replenishes, tracking recovers |

---

### Phase 3: IMU Dead Reckoning (200Hz Smooth Pose)

**Goal:** Smooth pose output between camera frames. Map pointer never stutters.

#### New: WorldState class

```cpp
class WorldState {
    void propagate(timestamp_ns, gx, gy, gz, ax, ay, az);  // called at 200Hz from sensor thread
    void applyVisualCorrection(R, t, heading, timestamp_ns); // called at 30Hz from Tracker
    Pose3D getLatestPose() const;                            // called at 60-120Hz from UI
};
```

**Critical design decisions:**

1. **No accelerometer double-integration for position.** Consumer phone accelerometers have ~0.01 m/s^2 bias causing 0.5m drift in 10 seconds. Instead, between visual corrections, WorldState propagates at constant velocity (last velocity from Tracker).

2. **No separate C++ thread.** Piggybacks on Android's sensor callback thread (already 200Hz, already high priority). `propagate()` is ~5 microseconds.

3. **Snap correction, not Kalman filter.** When Tracker produces a valid pose, WorldState snaps to it and estimates velocity from position difference. Preserves existing step-scale behavior.

4. **Drift cap:** 2m max dead-reckoning displacement without visual correction. After that, velocity zeroed.

#### New Files

| File | ~Lines | Purpose |
|------|--------|---------|
| `app/src/main/cpp/WorldState.h` | 90 | Pose state: position, velocity, rotation, heading |
| `app/src/main/cpp/WorldState.cpp` | 150 | propagate(), applyVisualCorrection(), getLatestPose() |
| `app/src/main/java/.../PoseData.kt` | 10 | Lightweight 5-field data class for UI (x, y, z, heading, timestamp) |

#### Modified Files

| File | Change |
|------|--------|
| `VioEngine.h/cpp` | Add WorldState member, wire sensor callbacks to propagate() |
| `native-lib.cpp` | Add `getLatestPose()` JNI function |
| `NativeBridge.kt` | Add `external fun getLatestPose(): PoseData` |
| `app/CMakeLists.txt` | Add WorldState.cpp |

#### How sensor callbacks change in VioEngine

```cpp
void VioEngine::addGyroData(int64_t ts, float x, float y, float z) {
    cached_gx_ = x; cached_gy_ = y; cached_gz_ = z;  // atomic cache
    imu_.addGyroReading(ts, x, y, z);                  // buffer for Tracker (unchanged)
}

void VioEngine::addAccelData(int64_t ts, float x, float y, float z) {
    imu_.addAccelReading(ts, x, y, z);                  // buffer for Tracker (unchanged)
    world_state_.propagate(ts, cached_gx_, cached_gy_, cached_gz_, x, y, z);  // NEW: 200Hz
}
```

#### Tests

| Test | What it validates |
|------|-------------------|
| PropagateConstantVelocity | Set velocity, propagate 1s, check position |
| PropagateWithRotation | Constant gyro, verify heading changes |
| VisualCorrectionSnaps | Propagate 0.5s, apply correction, verify snap |
| VelocityEstimation | Two corrections 33ms apart, verify velocity |
| DriftCap | 10s without correction, displacement capped at 2m |
| ConcurrentReadWrite | 4 threads (propagate + correct + 2x read), no crash/NaN |

---

### Phase 4: Drift Reduction

**Goal:** Reduce position drift by tracking against keyframes, optimizing recent poses, and detecting loop closures. Runs entirely on the Mapper thread (from Phase 2).

**Prerequisites:** Phase 2 and Phase 3 must be complete first.

#### Existing drift mitigation (DO NOT duplicate)

These already work and must remain untouched:

| Mechanism | File:Line | Status |
|-----------|-----------|--------|
| ZUPT (freeze when static) | VisionModule.cpp:322-332 | Working |
| Step-based scale estimation | VisionModule.cpp:131-176 | Working |
| Gyro bias online estimation | VisionModule.cpp:368-376 | Working |
| Forward-backward flow check | VisionModule.cpp:273-292 | Working |
| Heading freeze on bad frames | VisionModule.cpp:453-458 | Working |
| Adaptive rotation fusion | VisionModule.cpp:380-392 | Working |
| Initial heading from magnetometer | VisionModule.cpp:77-91 | Working (one-time) |
| Accel bias estimation | VisionModule.cpp:509-513 | Working (diagnostic) |

#### 4A: Keyframe Map + Multi-Frame Tracking

**Problem:** Current system matches features only against the previous frame. Errors compound: frame 1->2->3->...->30 accumulates ~30x the per-frame error.

**Fix:** Mapper maintains a ring of keyframes (max 10). Tracker matches against the nearest keyframe instead of the previous frame. Error is measured once (keyframe->current) instead of compounding across 30 intermediate frames.

```
Current:  frame1 → frame2 → frame3 → ... → frame30  (error compounds 30x)
New:      keyframe ──────────────────────→ frame30    (error measured once)
```

**What changes in Mapper.cpp** (~60 lines added):
```cpp
struct Keyframe {
    cv::Mat gray;
    std::vector<cv::Point2f> points;
    cv::Mat pose_R, pose_t;           // world pose at keyframe time
    int64_t timestamp_ns;
    int id;
};
std::vector<Keyframe> keyframes_;     // ring buffer, max 10
```

**What changes in Tracker:**
- New method: `setReferenceKeyframe(const cv::Mat& gray, const std::vector<cv::Point2f>& pts)`
- When set, optical flow runs against the keyframe instead of `prev_gray_`

**Keyframe promotion criteria:** mean flow > 20px from last keyframe OR > 30 frames elapsed.

#### 4B: Local Bundle Adjustment (Sliding Window)

**Problem:** Each frame estimates R,t independently. Errors in one frame propagate forward forever.

**Fix:** Every time Mapper processes a keyframe, it runs a sliding-window optimization over the last 3-5 keyframes. This jointly refines their poses to minimize total reprojection error.

This is NOT full SLAM BA. It's a small local window at 2-5fps on the Mapper thread, never blocking the Tracker.

**New files:**

| File | ~Lines | Purpose |
|------|--------|---------|
| `app/src/main/cpp/LocalBA.h` | 40 | Sliding-window pose optimizer interface |
| `app/src/main/cpp/LocalBA.cpp` | 200 | Gauss-Newton over 3-5 keyframe poses |

**How it connects:**
```cpp
// In Mapper, after inserting a keyframe:
PoseCorrection correction = local_ba_.optimize(keyframes_, last_5);
if (correction.valid) {
    tracker_.applyPoseCorrection(correction);
}
```

**New in Tracker:**
```cpp
struct PoseCorrection {
    cv::Mat R_corrected, t_corrected;
    double heading_corrected;
};
void Tracker::applyPoseCorrection(const PoseCorrection& correction);
```

#### 4C: Loop Closure (Place Recognition)

**Problem:** Walking back to the start position shows meters of drift. The system doesn't know you've returned to a previously visited location.

**Fix:** When Mapper receives a new keyframe, it compares ORB descriptors against all stored keyframes. If a match is found, it computes the relative pose and distributes the error correction across all intermediate keyframes.

**New files:**

| File | ~Lines | Purpose |
|------|--------|---------|
| `app/src/main/cpp/LoopDetector.h` | 40 | ORB-based place recognition interface |
| `app/src/main/cpp/LoopDetector.cpp` | 250 | Loop detection + pose-graph relaxation |

**How loop correction works:**
```
Walk path: A → B → C → D → E → A'   (A' should be A but drifted 3m)

Loop detected: A' matches keyframe A, error = 3m
Correction: distribute 3m proportionally across B, C, D, E
Result: each intermediate pose shifts ~0.6m, closing the loop
```

**Runs entirely on Mapper thread.** Tracker sees corrected pose via `applyPoseCorrection()` which feeds through `WorldState::applyVisualCorrection()`.

#### Phase 4 Tests

| Test | What it validates |
|------|-------------------|
| KeyframePromotion | Keyframe created when mean flow > 20px |
| KeyframeTracking | Tracker uses keyframe ref, error lower than frame-to-frame |
| LocalBAConvergence | 5 keyframes with known error, BA reduces reprojection error |
| LoopDetectionORB | Same scene image detected as loop candidate |
| LoopCorrectionDistribution | 3m drift at loop point distributed across intermediate poses |
| NoFalseLoops | Different scenes don't trigger false loop closure |

---

## 5. Expected Results

| Metric | Current | After Phase 2+3 | After Phase 4 |
|--------|---------|-----------------|---------------|
| **Frame rate** | ~30fps (drops to 25 during feature detect) | 40-60fps | 40-60fps (unchanged) |
| **UI update rate** | 30Hz (jerky) | 200Hz (smooth) | 200Hz (unchanged) |
| **Tracking during occlusion** | No pose updates | IMU dead reckoning for ~2m | Same |
| **100m straight walk drift** | ~5m | ~5m (same math) | ~2-3m (local BA) |
| **Walk a loop back to start** | ~5m gap | ~5m gap | ~0.5m gap (loop closure) |
| **Feature loss recovery** | Blocks tracker for 5ms | Background replenishment | Same |

**Phase 2+3 = faster and smoother, same accuracy.**
**Phase 4 = actually reduces drift.**

---

## 6. File Inventory

### New Files (all phases)

| File | Phase | ~Lines |
|------|-------|--------|
| `app/src/main/cpp/ThreadSafeQueue.h` | 2 | 60 |
| `app/src/main/cpp/Tracker.h` | 2 | 80 |
| `app/src/main/cpp/Tracker.cpp` | 2 | 400 |
| `app/src/main/cpp/Mapper.h` | 2 | 50 |
| `app/src/main/cpp/Mapper.cpp` | 2+4 | 120+60 |
| `app/src/main/cpp/VioEngine.h` | 2 | 60 |
| `app/src/main/cpp/VioEngine.cpp` | 2+3 | 80+40 |
| `app/src/main/cpp/WorldState.h` | 3 | 90 |
| `app/src/main/cpp/WorldState.cpp` | 3 | 150 |
| `app/src/main/cpp/LocalBA.h` | 4 | 40 |
| `app/src/main/cpp/LocalBA.cpp` | 4 | 200 |
| `app/src/main/cpp/LoopDetector.h` | 4 | 40 |
| `app/src/main/cpp/LoopDetector.cpp` | 4 | 250 |
| `app/src/main/java/.../PoseData.kt` | 3 | 10 |

### Modified Files

| File | Phase | Change |
|------|-------|--------|
| `native-lib.cpp` | 2+3 | VisionModule* → VioEngine*, add getLatestPose JNI |
| `NativeBridge.kt` | 3 | Add getLatestPose() |
| `app/CMakeLists.txt` | 2+3+4 | Add all new .cpp files |

### Preserved Files (NO changes in any phase)

| File | Why |
|------|-----|
| `IMUPreintegrator.h/cpp` | Step detection, ring buffers, motion mode — all untouched |
| `VioData.kt` | All 27 fields preserved, JNI signature unchanged |
| `SensorRepository.kt` | Sensor registration unchanged |
| `DeviceOrientationTracker.kt` | VIO-ready pitch check unchanged |

---

## 7. Implementation Order

```
Phase 2 (3-4 days)
  Step 1: ThreadSafeQueue.h (header-only, test in isolation)
  Step 2: Tracker.h/cpp (extract from VisionModule.cpp)
  Step 3: Mapper.h/cpp (background feature detection thread)
  Step 4: VioEngine.h/cpp (wire together)
  Step 5: Update native-lib.cpp (swap VisionModule* for VioEngine*)
  Step 6: Update CMakeLists.txt
  Step 7: Tests + on-device validation

Phase 3 (2-3 days)
  Step 1: WorldState.h/cpp (propagate, correct, read)
  Step 2: Wire into VioEngine (sensor callbacks + processFrame)
  Step 3: Add getLatestPose JNI + PoseData.kt
  Step 4: Tests + on-device validation

Phase 4 (4-5 days)
  Step 1: Keyframe storage in Mapper + setReferenceKeyframe in Tracker
  Step 2: LocalBA.h/cpp (sliding window optimizer)
  Step 3: LoopDetector.h/cpp (ORB matching + pose-graph correction)
  Step 4: Wire into Mapper pipeline
  Step 5: Tests + on-device validation
```

---

## 8. Risk Areas

| Risk | Impact | Mitigation |
|------|--------|------------|
| Stale features from Mapper | Injected features may not track | Existing FB check naturally rejects stale points |
| Emergency feature detect defeats the split | Tracker still blocks sometimes | Threshold set to 75 (half of 150), rarely hit |
| WorldState snap causes visual jump | Map pointer jumps at 30Hz boundaries | Add optional 15ms blend (polish after validation) |
| Local BA too slow for phone | Mapper falls behind | Limit to 3 keyframes, cap at 10ms, skip if overdue |
| False loop closures | Incorrect pose correction | Require 30+ ORB matches + geometric verification |
| Step detector disrupted by refactor | Scale estimation breaks | IMUPreintegrator is completely untouched |

---

## 9. Constraints

- **Do NOT break the pedestrian step/stride model** — it's the ground truth scale prior
- **Do NOT use magnetometer during tracking** — only at startup for initial heading (FR17)
- **Preserve the JNI interface** — VioData with all fields stays identical
- **Keep files under 500 lines**
- **pthreads only** — no C++20 jthread (Android NDK compatibility)
