# NavSight — 4-Tier System Architecture (v1.0-osm)

>_SUPERSEDED: the deck uses the single full-system diagram diagrams/pro/png/00-system-architecture-full.png; this sub-diagram is reference-only._

```mermaid
flowchart TB
    subgraph T1["Tier 1 — Presentation (Kotlin / Jetpack Compose)"]
        UI["NavSightViewModel (state)<br/>MainScreen (Google Maps base) · CameraUi overlay · DebugPanel"]
        CAM["CameraX (rear camera)"]
        SENS["SensorRepository<br/>accel + gyro ~50 Hz · GPS health-check"]
    end

    subgraph T1b["JNI Bridge (C++/JNI)"]
        JNI["NativeBridge (JNI)<br/>zero-copy ByteBuffer frames"]
    end

    subgraph T2["Tier 2 — VioEngine (orchestrator)"]
        VIO["VioEngine<br/>owns the two heavyweight native modules"]
    end

    subgraph T3["Tier 3 — Tracker (visual front-end)"]
        TRK["KLT optical flow · ORB relocalization · keyframes"]
        EKF["EKF (15-DOF error state)<br/>+ measurement updaters"]
    end

    subgraph T4["Tier 4 — IMUPreintegrator"]
        IMU["Madgwick AHRS attitude<br/>gyro / accel preintegration"]
    end

    subgraph OSM["On-device OSM Road-Matching (Kotlin, GPS-free)"]
        MATCH["LocalMatcher (Viterbi HMM)<br/>bundled Haifa OSM assets in APK"]
        BALL["Graph-rail snaps the 'ball' onto the matched OSM road<br/>(displayed over Google Maps base tiles)"]
    end

    CAM --> JNI
    SENS --> JNI
    JNI <--> VIO
    VIO --> TRK
    VIO --> IMU
    TRK --> EKF
    IMU --> EKF
    EKF -->|pose / position| MATCH
    MATCH --> BALL
    BALL -->|matched position<br/>(speed derived Kotlin-side)| UI
```

**Presenter caption:** NavSight is a 4-tier system: a Compose UI feeds camera frames and IMU through a zero-copy JNI bridge into the native `VioEngine`, which owns the `Tracker` (KLT front-end + 15-DOF error-state EKF) and the `IMUPreintegrator` (Madgwick). The EKF's pose is snapped to the on-device OSM road graph by a Viterbi matcher, producing the live "ball on the road" (speed is derived Kotlin-side from successive positions). Navigation is computed on-device and GPS-free — camera + IMU + offline OSM road-matching need no live GPS fix and no network in the navigation hot path; the base map shown under the ball uses Google Maps tiles.
