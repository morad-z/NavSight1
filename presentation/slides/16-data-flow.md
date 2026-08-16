# Slide 16: End-to-End Data Flow — Sensor to Screen

**Section:** Deep Dive / Subsystems · **Slide:** 16 of 23 · **Estimated Time:** 0.5 minutes

## On-Slide Content
- **Sensors in:** IMU (accel + gyro @ ~50 Hz via `SensorRepository`), rear camera frames (CameraX, capture locked at 30 fps / 33 ms; ~23 fps / ~43 ms effective after `KEEP_ONLY_LATEST` frame-dropping), all on-device.
- **Native core (C++17 / OpenCV 4.5.3):** `VioEngine` orchestrates → `IMUPreintegrator` (Madgwick + Forster midpoint preintegration) and `Tracker` (KLT front-end, owns the **15-DOF EKF** error-state + measurement updaters).
- **JNI bridge:** `NativeBridge` marshals zero-copy `ByteBuffer` camera frames in; returns a `VioData` 30-field struct (pose + status; speed is derived Kotlin-side from successive positions) out.
- **Kotlin UI:** `NavSightViewModel` holds state (Compose `mutableStateOf`, ~200 ms throttle) → `MainScreen` (`MapScreenUi`) + `CameraUi` overlay.
- **The ball advances:** estimated **speed** moves the map-matched ball along the locked OSM road; the **gyro-relative heading** steers it at junctions.
- **Map layers:** the displayed base map is Google Maps tiles; the on-device `LocalMatcher` over the bundled Haifa OSM road graph snaps the ball — the GPS-free navigation hot path runs offline (no GPS, no network).
- [Diagram: diagrams/03-data-flow.md]

## Talking Points (what the presenter SAYS)
- "This slide is the whole pipeline on one page — from raw sensors to the ball moving on the screen. Two streams come in: the IMU at about 50 hertz and camera frames captured at a locked 30 fps — about 23 fps effective after frame-dropping keeps only the latest — both entirely on-device."
- "They cross into the native C++ core through the JNI bridge. `VioEngine` is a thin orchestrator owning two heavyweight modules — the `IMUPreintegrator` for attitude and motion preintegration, and the `Tracker`, which runs the KLT visual front-end and owns the 15-DOF Extended Kalman Filter error-state. The camera frame goes across as a zero-copy ByteBuffer (the `processCameraFrameDirect` entry takes 9 arguments), and the result comes back as a single `VioData` 30-field return struct — speed isn't a field, it's derived Kotlin-side from successive positions."
- "On the Kotlin side, `NavSightViewModel` is the single source of UI truth. It throttles updates to roughly 200 milliseconds — fast enough to look live, slow enough not to thrash recomposition — and drives the `MainScreen` map and `CameraUi` Compose surfaces."
- "The last link is the important one: the estimated **speed** advances the map-matched ball along whichever OSM road it's locked to, and the **gyro-relative heading offset** picks the branch at junctions. The road graph comes from the on-device `LocalMatcher` over the bundled Haifa OSM assets, so the navigation computation needs no GPS and no network — only the Google Maps base map underneath is fetched (cached or blank in airplane mode)."

## Potential Questions (Defense)
**Q:** Where exactly does the displayed position come from — the EKF, or the map matcher?
**A:** Both, in a split role. The EKF/Tracker produces motion — speed and heading change. The map matcher (`LocalMatcher`) owns the *displayed* position by advancing a ball along the road graph: the EKF's speed moves it along the locked way, and the gyro-relative heading steers it at junctions. So position is graph-constrained, while distance and turn decisions come from the VIO core. That decoupling is why the ball never leaves the road yet still travels the correct distance.
**Follow-up Q:** Doesn't a 200 ms UI throttle make the ball look laggy?
**Follow-up A:** No — the native core still processes frames at ~23 fps (~43 ms effective interval, captured at a locked 30 fps) and integrates continuously; only the *display* is sampled at ~200 ms. The underlying state is current; we just don't recompose Compose faster than the eye needs, which keeps the UI thread light.

**Q:** The JNI boundary is a classic performance and correctness hazard — how do you keep it safe and fast?
**A:** Speed: camera frames cross as a direct `ByteBuffer` with no copy, and results come back as one batched `VioData` struct rather than many small calls. Correctness: a `state_mutex` plus a `shared_ptr` lifetime pattern keeps the native engine alive across start/stop, and there's an explicit Z-up to Y-up frame conversion at the boundary so the world-frame convention is handled in exactly one place. Per-frame tracking time was 15.4 ms median against a 200 ms budget, so the boundary is not the bottleneck.
**Follow-up Q:** What happens to this flow when vision fails — blur, darkness, feature starvation?
**Follow-up A:** The flow degrades gracefully via the inertial bridge: `IMUPreintegrator` keeps feeding motion and the speed estimator predicts from forward acceleration for up to ~6 s, so the ball keeps moving down the road. On the 2026-06-04 ride we logged 36 looming (essential-matrix-degenerate) fallbacks and 49 ORB relocalization events — the pipeline recovered every time without a network call.

## Speaker Notes
- This is a ~0.5-minute "connect the dots" slide — keep it as a narration over the diagram, not a deep dive. The deep dives are Slides 13 (KLT), 14 (map matching), 15 (speed).
- Key class names to land: `SensorRepository`, `NativeBridge`, `VioEngine`, `IMUPreintegrator`, `Tracker`, `EKF` (15-DOF error-state), `VioData`, `NavSightViewModel`, `MainScreen`/`CameraUi`, `LocalMatcher`.
- The one sentence that must land: "speed advances the ball, gyro-relative heading steers it, the road graph constrains it." That is the user-visible behaviour the whole stack exists to produce.
- Numbers to have ready if pressed: camera captured at a locked 30 fps (~23 fps / ~43 ms effective after frame-dropping), ~50 Hz IMU, ~200 ms UI throttle, 15.4 ms median tracking time vs the 200 ms SDD budget, and the bundled Haifa OSM assets.
- Pitfall to avoid: don't re-explain the EKF math or IPM here — defer to the dedicated slides; this slide is purely about *how the pieces connect and hand off*.
