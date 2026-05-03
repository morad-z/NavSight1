# NavSight — Architecture

This is the **single source of truth** for how the NavSight VIO pipeline is
wired. All other architecture documents have been moved to `docs/old docs/`
(e.g. `VISUAL_ALGORITHMS.md`, `OPENVINS_ARCHITECTURAL_LESSONS.md`,
`parallel_vio_refactor_plan.md`, `HOW_THE_SYSTEM_WORKS.md`,
`VIO_IMPROVEMENT_PLAN.md`) and are **archival** — they may disagree with
current code. When they do, this file wins.

---

## 1. One-screen pipeline

```
                        ┌─────────────────────────────────────────────────────┐
                        │                  ANDROID PROCESS                    │
                        └─────────────────────────────────────────────────────┘
                                                │
        ┌───────────────────────┬───────────────┴───────────────┬─────────────────────────┐
        │                       │                               │                         │
   ┌────▼─────┐          ┌──────▼──────┐                ┌───────▼────────┐         ┌──────▼──────┐
   │  Camera  │          │ IMU         │                │ MiDaS depth    │         │ GPS / Mag   │
   │  YUV @30 │          │ Acc + Gyro  │                │ (TFLite, ~1Hz) │         │ (one-shot   │
   │  Hz      │          │ ~100–500 Hz │                │                │         │  bootstrap) │
   └────┬─────┘          └──────┬──────┘                └────────┬───────┘         └──────┬──────┘
        │                       │                                │                        │
        │                       ▼                                │                        │
        │              ┌──────────────────┐                      │                        │
        │              │  Madgwick        │                      │                        │
        │              │  attitude filter │ ◄─── ADR-001          │                        │
        │              │  (roll, pitch,   │                      │                        │
        │              │   yaw_seed)      │                      │                        │
        │              └────────┬─────────┘                      │                        │
        │                       │                                │                        │
        │                       ▼                                │                        │
        │             ┌─────────────────────┐                    │                        │
        │             │  IMUPreintegrator   │                    │                        │
        │             │  (preintegrated     │                    │                        │
        │             │   ΔR, ΔV, ΔP, J*)   │                    │                        │
        │             └────────┬────────────┘                    │                        │
        │                      │                                 │                        │
        ▼                      ▼                                 │                        │
  ┌──────────────────────────────────────┐                       │                        │
  │                Tracker                │                      │                        │
  │  • TrackKLT (gyro-aided optical flow)│                      │                        │
  │  • RANSAC essential matrix           │                      │                        │
  │  • FeatureManager + keyframe gating  │                      │                        │
  │  • ScaleEstimatorVI (closed-form     │                      │                        │
  │    Hesch/Martinelli)                 │                      │                        │
  │  • UpdaterZeroVelocity (ZUPT)        │                      │                        │
  └──────────────────┬───────────────────┘                      │                        │
                     │                                          │                        │
                     ▼                                          │                        │
        ┌───────────────────────────────────────────────────────▼────────────────────────▼──┐
        │                                EKFState (15-DOF)                                 │
        │     state = [δθ(3), δb_g(3), δv(3), δb_a(3), δp(3)]   (+ 6 per camera clone)     │
        │                                                                                   │
        │   propagateIMU (predict)  ◄── IMUPreintegrator                                    │
        │   updateRelativePose     ◄── Tracker (vision)                                     │
        │   updateGravityAlignedYaw ◄── Tracker (keyframe yaw)                              │
        │   updatePDRStep          ◄── IMUPreintegrator (step events)                       │
        │   updateZUPT             ◄── UpdaterZeroVelocity                                  │
        │   ScaleFuser ↔ updateScale ◄── (PDR | MiDaS | VI scale observers)                 │
        │                                                                                   │
        │   Output: R_GtoI_, p_G_, v_G_, b_g_, b_a_, P_  (15×15)                            │
        └───────────────────────────────────────┬───────────────────────────────────────────┘
                                                │
                  ┌─────────────────────────────┼─────────────────────────────┐
                  ▼                             ▼                             ▼
          ┌──────────────┐             ┌──────────────────┐           ┌──────────────────┐
          │ JNI bridge   │             │ getPositionCov-  │           │ getCalibration / │
          │ (NativeBridge│             │ arianceXZ        │           │ getInitStatus    │
          │  → VioData)  │             │ (Step 6)         │           │ (Step 5)         │
          └──────┬───────┘             └────────┬─────────┘           └─────────┬────────┘
                 │                              │                               │
                 ▼                              ▼                               ▼
        ┌──────────────────────────────────────────────────────────────────────────────┐
        │                            NavSightViewModel (Kotlin)                       │
        │  • pathHistory : List<PathPoint(x, z, σ)>                                    │
        │  • positionSigmaM, positionCovValid                                          │
        │  • CrashLogger.updateSnapshot(...) every frame                               │
        └────────────────────────────────────┬─────────────────────────────────────────┘
                                             │
                                             ▼
        ┌──────────────────────────────────────────────────────────────────────────────┐
        │                         Compose UI (MapScreenUi.kt)                         │
        │  • Google Maps polyline (color = σ bin)                                      │
        │  • Uncertainty Circle (radius = σ, color = σ bin)                            │
        │  • VioStatusChip (lost / init / active / degraded)                           │
        │  • SensorRadarWaze fallback (no map)                                         │
        └──────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Threads and ownership

| Thread                    | Owns                                       | Reads                         |
| ------------------------- | ------------------------------------------ | ----------------------------- |
| Camera (CameraX worker)   | Tracker hot path, EKF predict + updates    | IMU buffer (lock-free read)   |
| Sensor (IMU listener)     | IMUPreintegrator append-only buffer        | —                             |
| MiDaS (TFLite worker)     | depth-map pubsub                           | latest YUV (best-effort)      |
| GPS / Mag listener        | one-shot bootstrap heading                 | —                             |
| Compose UI                | NavSightViewModel.pathHistory snapshot     | EKF pose via JNI mirror       |
| `Thread.UncaughtException`| `CrashLogger` JSON dump                    | last published snapshot       |

`EKFState` is touched only from the camera thread. The JNI accessors
(`getPose`, `getPositionCovarianceXZ`, `getCalibration`) take internal mutexes
and clone matrices before returning, so the UI thread sees consistent
snapshots.

---

## 3. Bootstrap ordering

1. **Permissions** — Camera + ACCESS_FINE_LOCATION (`MainActivity.kt:44`).
2. **CrashLogger.install** — before any UI work, so an early crash still dumps
   `<external-files>/crash_logs/crash_<ts>.json` (`MainActivity.kt:25`).
3. **InertialInitializer**:
   - Stationary gate — wait for accel/gyro variance to drop below threshold.
   - Motion gate — wait for the user to start walking.
   - On READY: `EKFState::initializeFull(R_GtoI, gyro_bias, accel_bias)`.
4. **Optional fast-path**: a stored calibration (gravity rotation + biases)
   from a prior session may be loaded via
   `VioEngine::loadStoredCalibration` to skip the stationary gate.
5. **Magnetometer one-shot**: at startup only, the magnetometer can seed
   `global_R_` so the first heading is correct. After init it is **never**
   consulted again — see ADR-005.

---

## 4. Update channels into the EKF

| Update                        | Source                                | Variance source                       | Frequency  |
| ----------------------------- | ------------------------------------- | ------------------------------------- | ---------- |
| `propagateIMU`                | IMUPreintegrator.integrate(t0,t1)     | `imu_delta.cov`                       | per frame  |
| `updateRelativePose`          | Tracker recoverPose × ScaleFuser      | `var_t` (scaled by fuser variance)    | per frame  |
| `updateGravityAlignedYaw`     | Tracker keyframe yaw                  | `last_visual_yaw_variance_`           | keyframes  |
| `updatePDRStep`               | IMUPreintegrator step event           | stride² · σ_stride²                   | per step   |
| `updateZUPT`                  | UpdaterZeroVelocity                   | hard-coded ZUPT noise                 | quasi-static |
| ScaleFuser predict / update   | ScaleEstimatorVI, MiDaS, PDR          | per-observer (MAD-based for MiDaS)    | mixed      |

---

## 5. Module map (C++)

```
app/src/main/cpp/
├── native-lib.cpp           ─ JNI glue
├── VioEngine.{h,cpp}        ─ top-level orchestrator
├── Tracker.{h,cpp}          ─ camera-thread hot path
├── EKFState.{h,cpp}         ─ 15-DOF ESKF + clone window
├── IMUPreintegrator.{h,cpp} ─ Madgwick + preintegration + steps
├── InertialInitializer.{h,cpp}
├── FeatureManager.{h,cpp}   ─ KLT keyframe bookkeeping
├── TrackKLT.{h,cpp}         ─ gyro-aided KLT
├── LensCorrector.{h,cpp}
├── ScaleFuser.{h,cpp}       ─ 1-D Kalman over observers
├── ScaleEstimatorVI.{h,cpp} ─ closed-form Hesch/Martinelli
├── UpdaterZeroVelocity.{h,cpp}
├── UpdaterMSCKF.{h,cpp}     ─ DEAD (kept; see ADR-006)
└── VioTypes.h               ─ shared structs
```

```
app/src/main/java/com/example/navsight1/
├── MainActivity.kt          ─ permissions, splash, CrashLogger install
├── NavSightViewModel.kt     ─ pathHistory, positionSigmaM, snapshot publish
├── NativeBridge.kt          ─ JNI binding
├── SensorRepository.kt      ─ camera + IMU plumbing
├── CrashLogger.kt           ─ uncaught exception JSON sink
├── MapScreenUi.kt           ─ Maps, radar, VioStatusChip
└── CameraUi.kt              ─ camera preview overlay
```

```
tests/
├── cpp/
│   ├── CMakeLists.txt       ─ desktop + Android test build
│   ├── test_*.cpp           ─ GoogleTest unit suites
│   ├── replay_harness.cpp   ─ Step 7 IMU-only replay
│   └── replay_scorer.py     ─ Step 7 metrics gate
└── sims/
    ├── *.json               ─ recorded sessions (analysis)
    └── regression/          ─ fixtures gated by .github/workflows/replay.yml
```

---

## 6. Configuration knobs

There is intentionally **no** central `VioConfig` struct. Every constant lives
next to its consumer with a citation comment per Step 9. To find them:

```bash
grep -rn "static constexpr" app/src/main/cpp/
```

The notable ones are:

| Constant                          | Where                | Justification                                  |
| --------------------------------- | -------------------- | ---------------------------------------------- |
| `ROTATION_STEP_GATE_RADPS = 0.8`  | `IMUPreintegrator.h` | 46°/s — above arm swing, below in-place turn. |
| `MIN_PARALLAX_PX = 0.8`           | `Tracker.h`          | Below sub-px noise floor for KLT.              |
| `RANSAC_THRESH = 1.5`             | `Tracker.h`          | OpenCV recommended for 480p.                   |
| `MAX_CLONES = 11`                 | `EKFState.h`         | OpenVINS default for monocular VIO.            |
| `SCALE_BOOTSTRAP_COUNT = 15`      | `Tracker.h`          | First N PDR observations → median seed.        |

---

## 7. What is **out of scope**

(see also `docs/PRODUCTION_READINESS_PLAN.md` §Non-Goals)

- Multi-user map sharing.
- Persistent SLAM map / loop closure across sessions.
- Wi-Fi / Bluetooth fingerprinting.
- GPS blending in the hot path (ADR-004).
- Magnetometer used during tracking (ADR-005).
- Turn-by-turn voice navigation. The SDD scope is "where am I", not "how do I get there".

---

## 8. Decision log

See `docs/adr/`. Each decision is one numbered file. Index:

| ADR  | Title                                              |
| ---- | -------------------------------------------------- |
| 001  | Madgwick as the attitude reference                 |
| 002  | ESKF (not full EKF or factor graph)                |
| 003  | MiDaS depth as a blocking observer for scooter mode |
| 004  | No GPS fusion in the hot path                      |
| 005  | Magnetometer used only for one-shot bootstrap      |
| 006  | Mapper pipeline disabled, kept as commented dead   |
| 007  | Replay harness is IMU-only by design               |
