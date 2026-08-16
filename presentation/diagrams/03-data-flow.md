# NavSight — End-to-End Data Flow (sensors -> EKF -> map ball -> UI)

>_SUPERSEDED: the deck uses the single full-system diagram diagrams/pro/png/00-system-architecture-full.png; this sub-diagram is reference-only._

```mermaid
flowchart LR
    subgraph IN["Sensors"]
        C["Rear camera<br/>640x480, 30 fps capture<br/>(~23 fps processed after frame-drop)"]
        I["IMU<br/>accel + gyro ~50 Hz"]
        G["GPS<br/>(health-check only,<br/>not in hot path)"]
    end

    subgraph CORE["Native VIO core"]
        T["Tracker<br/>KLT + ORB relocalization"]
        E["EKF (15-DOF error state)<br/>IMU propagation + updates"]
    end

    subgraph OUT["Estimates"]
        SP["Speed<br/>(IPM + inertial bridge)"]
        PO["Pose / heading"]
    end

    subgraph MM["Map matching"]
        V["Viterbi HMM matcher<br/>(offline OSM)"]
        B["Graph-rail ball<br/>advanced by speed"]
    end

    UI["Compose UI<br/>ball snapped to road + speed<br/>(base map: Google Maps tiles)"]

    C --> T
    I --> E
    T --> E
    G -.->|verify only| MM
    E --> SP
    E --> PO
    SP -->|advances the ball| B
    PO -->|junction steering| V
    V --> B
    B --> UI
```

**Presenter caption:** Camera and IMU feed the native EKF/Tracker; estimated speed and pose flow into the on-device offline Viterbi OSM road-matcher. Speed is what advances the graph-rail ball along the locked road, and heading steers it at junctions. GPS is used only as an out-of-loop health check — it never drives the navigation hot path. The navigation is GPS-free and computed on-device (offline OSM road-matching, no network in the navigation hot path); the base map shown under the ball uses Google Maps tiles.
