# Slide 07: Application Architecture — Modules & Ownership

**Section:** Architecture · **Slide:** 7 of 23 · **Estimated Time:** 1.5 minutes

## On-Slide Content
- **Kotlin side:** `NavSightViewModel` (state holder) ← `SensorRepository` (accel/gyro ~50 Hz, GPS 1 Hz, CameraX frames) ← `NativeBridge` (JNI, ~47 `external` fns) → `MainScreen` (`MapScreenUi`) + `CameraUi` overlay (Compose).
- **Native side — `VioEngine`:** thin orchestrator owning **exactly two** heavyweight modules.
- **`Tracker` (visual front-end):** KLT optical flow (forward-backward check, de-rotation), ORB relocalization, keyframes; **owns the 15-DOF EKF** and all measurement updaters (MSCKF, ZUPT/ZRUP, gravity-alignment, map).
- **`IMUPreintegrator`:** Madgwick AHRS attitude + gyro/accel preintegration (Forster midpoint).
- **EKF state:** 15-DOF navigation error state `[attitude, gyro bias, velocity, accel bias, position]`, Z-up ENU world; Joseph-form updates, per-row Huber (δ=2.4477), live MSCKF Mahalanobis χ² gate.
- [Diagram: diagrams/01-system-architecture.md]

## Talking Points (what the presenter SAYS)
- "On the Kotlin side, the flow is simple and one-directional. `SensorRepository` collects accelerometer and gyroscope at roughly 50 Hz, GPS at 1 Hz, and CameraX frames; it pushes them through `NativeBridge` — about 47 JNI functions — into the native core, and the resulting state lands in `NavSightViewModel`, which the Compose `MainScreen` and `CameraUi` overlay observe."
- "Inside native code, `VioEngine` is deliberately thin. It owns just two heavyweight things. The `IMUPreintegrator` runs the Madgwick attitude filter and preintegrates inertial measurements between frames. The `Tracker` is the visual front-end — KLT optical flow with a forward-backward check, ORB relocalization, keyframes — and crucially it *owns* the Extended Kalman Filter and every measurement updater."
- "The EKF is a 15-DOF error-state filter in a Z-up ENU world frame. Updates use the numerically stable Joseph form with per-row Huber robustness and a live MSCKF Mahalanobis χ² gate — which keeps a bad measurement from corrupting the state."
- "The navigation pipeline is GPS-free and computed on-device: camera plus IMU plus on-device OSM road-matching, with no GPS and no network in the navigation hot path. The displayed base map uses Google Maps tiles; OSM is only the road graph that snaps the ball."
- "We kept what we verified is used: KLT optical flow, ORB relocalization, the MSCKF update, ZUPT and gravity-alignment, IMU preintegration, the IPM ground-plane speed path, and offline OSM map-matching."

## Why We Chose This Approach
- **Error-state EKF (tightly-coupled) vs loosely-coupled blend.** A loosely-coupled filter would estimate a visual pose and an inertial pose separately, then average them — losing the cross-constraints. We chose a tightly-coupled, MSCKF-style error-state filter so the IMU can directly constrain visual scale and rotation, and so the filter degrades gracefully: when vision fails, the inertial bridge carries dead reckoning for ~6 s before decaying. We also considered optimization-based sliding-window VIO (VINS-Mono style) — heavier and unnecessary once map-matching took over global drift control.
- **Tradeoffs accepted:** filter consistency is genuinely hard — gravity must cancel exactly during propagation, and a ~6–10° tilt once leaked into ~800 m of phantom Z drift. We pay for tight coupling with this fragility, mitigated by a dedicated gravity-alignment update.
- **Module boundaries — why `VioEngine` owns only two modules.** A single mega-class would bury the EKF inside frame I/O. By making `VioEngine` a thin orchestrator and giving the `Tracker` clear ownership of the EKF + updaters, each updater (MSCKF, ZUPT, gravity, map) is an isolated, independently testable unit — which is exactly what the deterministic replay harness exercises.
- **Benefit gained:** low CPU, a principled covariance, a clean dead-reckoning fallback, and a data path where each updater is an independently testable unit driven by the deterministic replay harness.

## Potential Questions (Defense)
**Q:** Why does the `Tracker` own the EKF rather than having a separate fusion module?
**A:** Because every EKF measurement update is fundamentally a *visual* event — a KLT track, an MSCKF feature, a zero-velocity detection, a map snap. Keeping the EKF inside the `Tracker` means the state and its updaters live next to the measurements that drive them, with no extra marshaling layer. The SDD called this conceptual unit `SensorFusionEngine`; in code it's `EKFState` owned by `Tracker`. The `IMUPreintegrator` stays separate because it runs on a different clock (~50 Hz IMU vs the ~23 fps processed frame rate) and only hands over preintegrated `delta_R/v/p` deltas.

**Follow-up Q:** If the `Tracker` owns the EKF, how do you unit-test the filter in isolation?
**A:** Through the deterministic offline replay harness: the unmodified native engine re-runs recorded frames + IMU on identical input, so we can A/B individual updaters and gates. That's how we measured the inertial bridge contributes +24% integrated distance (62→77 m) on a turn-heavy ride — a stored deterministic offline A/B replay result — and it's backed by a 95-case Kotlin unit suite plus a C++ unit suite and CI replay scoring.

**Q:** How do you control drift, since you rely on the map matcher outdoors?
**A:** The offline map matcher is the drift constraint outdoors: it snaps the position to the road graph every step, with wrong-fork recovery bounded by design to 25–60 m at confidence ≥ 0.55. The ball cannot teleport across town, so on a road network the matcher keeps it honest without any global re-localization machinery.

## Speaker Notes
- Numbers to have ready: ~47 JNI `external` functions; MAX_CLONES = 15; Huber δ = 2.4477; live per-frame gate is the MSCKF Mahalanobis χ² gate (5× χ²₀.₉₅(2K); landmark per-obs gate 5.991), Joseph-form covariance update.
- Implementation nuance if pressed: the C++ state carries 19 DOF (adds camera-IMU time offset `δt_d` and extrinsic `δφ_bc`), but the *navigation* state is the 15-DOF block; the extra 4 are calibration nuisances. Slides keep "15-DOF."
- Frame the gravity-drift lesson if pressed: a ~6–10° tilt mis-cancelled gravity and leaked ~800 m of phantom Z drift; we found it by *reading the residual data*, not by tuning constants for days, and fixed it with the live gravity-alignment update. Lesson: read the data before tuning constants.
- The speed-path metric depth comes from the calibrated ground plane (h = 1.05 m), not learned depth.
- Build name on slide: **v1.0-osm**. Never a git hash.
