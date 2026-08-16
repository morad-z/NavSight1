>_SUPERSEDED: the deck uses the single full-system diagram diagrams/pro/png/00-system-architecture-full.png; this sub-diagram is reference-only._

# NavSight — Single Camera Frame Lifecycle (30 fps capture, ~23 fps processed, 15.4 ms median work)

```mermaid
sequenceDiagram
    participant CX as CameraX (30 fps capture)
    participant JNI as NativeBridge (JNI)
    participant TRK as Tracker
    participant IPM as IPM Speed + EKF
    participant VM as NavSightViewModel
    participant UI as Compose UI

    Note over CX: Capture locked 30 fps / 33 ms;<br/>processed ~23 fps / ~43 ms after frame-drop
    CX->>JNI: ByteBuffer frame (zero-copy)
    JNI->>TRK: processCameraFrameDirect (9 args)
    Note over TRK: per-frame work — median 15.4 ms, max ~78 ms<br/>(vs 200 ms SDD budget: 13x under at median;<br/>a worst-case ~78 ms frame is absorbed by frame-dropping)
    TRK->>TRK: lens-correct -> normalized coords
    TRK->>TRK: KLT pyramidal optical flow
    TRK->>TRK: forward-backward consistency check
    TRK->>TRK: de-rotate flow (gyro / Madgwick)
    TRK->>IPM: translation-only flow f_i
    IPM->>IPM: ground-plane speed (vote / zero-witness)
    IPM->>IPM: MSCKF measurement update<br/>(Mahalanobis chi^2 gate + per-row Huber, Joseph form)
    IPM-->>JNI: VioData (30-field return; speed derived Kotlin-side)
    JNI-->>VM: state update
    VM->>UI: throttled (~200 ms) recompose
    UI-->>UI: ball snapped to OSM road on Google Maps base
```

**Presenter caption:** The camera captures at a locked 30 fps (33 ms); after KEEP_ONLY_LATEST frame-dropping the effective processed VIO rate is ~23 fps (~43 ms). The `Tracker` lens-corrects, runs KLT optical flow, applies a forward-backward consistency check, and de-rotates the flow before IPM derives ground-plane speed and the MSCKF update refines pose (Mahalanobis chi^2 gate with per-row Huber, Joseph-form covariance). Median per-frame work is 15.4 ms (max ~78 ms) against the 200 ms SDD budget — 13x under at the median, 2.5x worst-case margin; a worst-case ~78 ms frame exceeds the inter-frame interval and is absorbed by frame-dropping. The ball is snapped to the on-device OSM road graph and drawn over a Google Maps base.
