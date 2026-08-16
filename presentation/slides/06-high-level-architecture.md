# Slide 06: High-Level Architecture — Four Tiers, One Pipeline

**Section:** Architecture · **Slide:** 6 of 23 · **Estimated Time:** 1.5 minutes

## On-Slide Content
- NavSight is a **4-tier system** that fuses one rear camera + IMU into a road-locked position — **GPS-free navigation computed on-device**, with no live GPS fix in the navigation hot path.
- **Tier 1 — Presentation (Kotlin / Jetpack Compose):** `NavSightViewModel`, `MainScreen` (`MapScreenUi`) + `CameraUi` overlay, CameraX, `SensorRepository`.
- **Tier 2 — JNI Bridge:** `NativeBridge` marshals zero-copy `ByteBuffer` camera frames into the native core (entry `processCameraFrameDirect`, 9 arguments) and returns the `VioData` 30-field return type. (~47 external JNI functions overall.)
- **Tier 3 — Native Core (C++17 / NDK / OpenCV 4.5.3):** `VioEngine` orchestrator owns `Tracker` (visual front-end + 15-DOF navigation error-state EKF) and `IMUPreintegrator` (Madgwick AHRS + Forster midpoint preintegration).
- **Tier 4 — Offline OSM map-matching:** on-device Viterbi `LocalMatcher` snaps position to a road graph; the bundled Haifa OSM assets ship in the APK with no network in the navigation hot path.
- Output: a live **"ball on the road" + speed**; the displayed base map is Google Maps tiles, with the ball snapped to the on-device OSM road graph.
- [Diagram: diagrams/01-system-architecture.md]

## Talking Points (what the presenter SAYS)
- "Everything you'll see today flows top-to-bottom through four tiers. Sensors come in at the top in Kotlin, the heavy math happens in a native C++ core, and the result is snapped to an on-device OSM road graph at the bottom — all the navigation runs on the phone."
- "Tier 1 is pure UI and sensor capture in Jetpack Compose. Tier 2 is a thin JNI bridge that hands raw camera frames to native code with zero copying. Tier 3 is where the real work lives: a `VioEngine` orchestrator that owns exactly two heavyweight modules — the `Tracker`, which runs the visual front-end and a 15-DOF navigation error-state Extended Kalman Filter, and the `IMUPreintegrator`, which handles attitude and inertial preintegration."
- "Tier 4 is what makes the output trustworthy: an on-device OSM map matcher that constrains the estimated position to an actual road graph, so the dot can never float off into a field."
- "The single most important architectural property: the navigation has no backend, no server, and no live GPS in the hot path. Haifa's road-matching graph ships inside the APK. The base map shown to the user is Google Maps tiles, but the navigation itself is self-sufficient — which is exactly the point in a GPS-denied environment."

## Why We Chose This Approach
- **Native C++ core vs pure-Kotlin/JVM.** The visual front-end + EKF must finish per frame inside our budget. The camera captures at a locked **30 fps (33 ms interval)**; after KEEP_ONLY_LATEST frame-dropping, the effective processed VIO rate is **~23 fps (~43 ms)** and a worst-case ~78 ms frame is absorbed by frame-dropping rather than stalling the pipeline. OpenCV and the Kalman filter are CPU-bound matrix work — the kind the JVM penalises with allocation and GC pressure. We accepted real costs for this: JNI marshaling complexity, and dual-language object-lifetime/mutex bugs across the boundary. The payoff is decisive: measured **median 15.4 ms** per tracking frame against a **200 ms** SDD budget — **13× headroom** — which a managed-runtime implementation could not reliably guarantee on a mid-range phone.
- **On-device vs cloud.** A cloud architecture would offload compute but reintroduce exactly the dependency we are trying to eliminate: a network. Our test region is Haifa under real regional GNSS jamming, where connectivity is unreliable and latency-sensitive navigation cannot tolerate a round-trip. By keeping the navigation on-device — including the bundled offline OSM road-matching graph — we get no network latency, no rate limits, full privacy, and **jamming-resilient / GPS-denied-capable** operation. (The displayed base map uses Google Maps tiles; in airplane-mode the navigation still runs offline while the base map shows cached or blank tiles.)
- **Tradeoffs accepted:** APK carries regional map assets (coverage limited to bundled regions); accuracy ceiling is bounded by the VIO trajectory quality, not by an external service we can blame.
- **Benefit gained:** the whole pipeline is reproducible offline via a deterministic replay harness, which is why every result on later slides is regression-tested, not anecdotal.

## Potential Questions (Defense)
**Q:** Why split into four tiers instead of putting everything in the native layer for speed?
**A:** Separation of concerns and lifecycle safety. Tier 1 must live in Kotlin/Compose because Android owns the UI thread, camera, sensors, and permissions. Tier 2 (the JNI bridge) is deliberately thin — it only marshals zero-copy `ByteBuffer` frames and returns the `VioData` 30-field return type, and it carries a `state_mutex` + `shared_ptr` pattern so the native engine survives a `stopVIO`/restart without a crash. Pushing UI into native code would gain nothing and would make us fight the Android lifecycle in C++. The boundary is where it belongs.

**Follow-up Q:** Doesn't the JNI boundary itself become a performance bottleneck?
**A:** No — that's why frames cross as a direct `ByteBuffer` (zero-copy), not a copied byte array. The marshaling cost is negligible next to the 15.4 ms median we spend inside the Tracker. The bridge moves a pointer, not pixels.

**Q:** You say "no GPS in the hot path" — but the app still requests location permission. Isn't that a contradiction?
**A:** GPS is captured at 1 Hz purely as a *secondary reference* and for an optional one-shot startup alignment — it never feeds the position estimate that draws the ball. NavSight is GPS-denied-*capable*: the visual-inertial pipeline runs entirely without a fix. We keep GPS available because removing it would discard a free validation signal on the days it happens to be healthy, but the navigation output does not depend on it.

## Speaker Notes
- Anchor numbers to have ready: native core is **C++17 / Android NDK / OpenCV 4.5.3**; target is the **Samsung Galaxy S21 Ultra (SM-G998B), Exynos 2100, Mali-G78**.
- If asked about the discrepancy with the SDD: the signed SDD describes a three-tier *layered* architecture (VisionModule / IMUPreintegrator / SensorFusionEngine); the live code realised this as four tiers with the JNI bridge as a first-class layer and the modules renamed `Tracker` / `IMUPreintegrator` / `EKFState`. Same design, more precise vocabulary.
- Emphasise the "ball on the road" framing — it's the mental model for the entire talk and ties directly to Tier 4.
- Refer to the build as **v1.0-osm**. The app targets **compileSdk/targetSdk 34 (Android 14)**; Android 15 / API 35 is the test-device runtime OS, not the build target. Do **not** quote a git hash on the slide.
- Pitfall to avoid: don't claim the system "doesn't use GPS at all" — that overstates and invites a gotcha. Use the precise framing: no GPS in the *hot path*.
