# Slide 11: Runtime Overview — How a Live Session Runs

**Section:** Runtime Architecture · **Slide:** 11 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- One unbroken pipeline, GPS-free navigation end to end: **Startup → Calibration → Acquire → Process → Track → Map-Match → Speed → Ball on the road.**
- **Startup:** load `libnavsight` native core; grant CAMERA + ACCESS_FINE_LOCATION; load the bundled Haifa OSM road-matching assets (no network in the navigation hot path).
- **Calibration:** gravity-alignment init + one-shot compass heading at start.
- **Acquire:** rear camera captured at a locked 30 fps (33 ms interval); after KEEP_ONLY_LATEST frame-dropping the VIO processes ~23 fps (~43 ms effective interval); IMU (accel + gyro) at ~50 Hz.
- **Process → Track:** 15.4 ms median per-frame visual work (KLT + MSCKF EKF) — 13× under the 200 ms SDD budget.
- **Map-Match → Speed:** Viterbi HMM snaps the position to the OSM road graph; IPM ground-plane speed + inertial/accel bridge estimate metric speed.
- **Output:** the navigation snaps a graph-constrained "ball on the road" with live km/h on a Google Maps base map, with **zero dependence on a GPS fix.**
- [Diagram: diagrams/03-data-flow.md]

## Talking Points (what the presenter SAYS)
- "Before we go module by module, here is the whole system running in one breath — this is what happens every second a user is navigating."
- "On startup the app loads the native C++ core, requests camera and location permissions, and pulls in the Haifa OSM road-matching graph that already lives inside the APK — so from this point on the navigation computation never touches the network; only the Google Maps base tiles do."
- "It calibrates: it finds 'down' from gravity and takes a single compass reading to know which way the user is facing. Then it opens two sensor streams — the camera captures at a locked 30 frames a second, of which the VIO processes about 23 after frame-dropping, and the IMU at roughly 50 Hz."
- "Each camera frame is processed and tracked in about 15 milliseconds median — comfortably inside our 200-millisecond budget — fed into the Kalman filter, snapped onto the road by the map matcher, and combined with the speed estimate to advance the ball on the map."
- "The single most important property of this loop: there is no live GPS fix anywhere in the navigation. GPS is not in the hot path — which is the entire point of the project."

## Why We Chose This Approach
- **Alternatives considered:** a GPS-assisted hybrid that blends a live fix when available. We rejected GPS from the hot path because the test region (Haifa) suffers real regional GNSS jamming — on Route A jammed GPS over-reported the path by +33% (1,705 m vs a true 1,280 m), and on other jammed rides we observed multi-second GPS position freezes while moving. A loop that depends on GPS would inherit those failures.
- **Tradeoff accepted:** the whole burden of accuracy falls on the camera + IMU + map graph, so the visual pipeline must be both fast and robust. We pay for that with a dedicated native core and a tight per-frame budget.
- **Benefit gained:** the navigation is computed on-device (GPS-free, offline OSM road-matching) and jamming-resilient. Every navigation stage — matching, speed — runs on-device, so there is no network latency and no privacy exposure in the navigation path; only the Google Maps base tiles use the network.
- **Engineering reasoning:** keeping the pipeline linear and single-purpose (acquire → process → match → display) makes it auditable and lets us validate each stage independently on recorded fixtures via the deterministic replay harness.

## Potential Questions (Defense)
**Q:** If GPS is available, why not use it to improve accuracy?
**A:** Because in our actual operating environment GPS is the unreliable signal, not the trustworthy one. Under real jamming on Route A, GPS over-reported the path by +33% (1,705 m on a route that truly measures 1,280 m), and on other jammed rides we observed multi-second GPS position freezes while moving. Map-measured routes are our jamming-resilient ground truth. We keep GPS in the app — we never removed it — but it is not in the navigation hot path.
**Follow-up Q:** So GPS is completely unused at runtime?
**Follow-up A:** It is sampled at about 1 Hz and used only as a secondary reference and, on clean days, for validation. The displayed position is driven entirely by the camera, IMU, and the OSM road graph.

**Q:** What is the slowest part of this runtime loop, and does it bottleneck the frame rate?
**A:** The per-frame visual tracking is the heaviest stage, with a median of 15.4 ms and a worst case of about 78 ms. The camera captures at a locked 30 fps (a frame every 33 ms); after KEEP_ONLY_LATEST frame-dropping the VIO processes ~23 fps. A worst-case ~78 ms frame exceeds the inter-frame interval, so it is simply absorbed by frame-dropping rather than stalling the pipeline. Against the 200 ms SDD budget we are 13× under at the median and still have a 2.5× margin at worst case.
**Follow-up Q:** What happens to a frame that does exceed the budget?
**Follow-up A:** Heavy work runs on a dedicated native VIO executor thread, so a slow frame does not stall the UI, and frame-dropping keeps only the latest frame; if vision genuinely fails on a frame, the inertial/accel bridge carries speed forward and the ball keeps moving on the road.

## Speaker Notes
- Numbers to have ready: 30 fps / 33 ms camera capture (locked) vs ~23 fps / ~43 ms effective processed VIO rate after frame-dropping; 15.4 ms median / ~78 ms max tracking; 200 ms SDD budget; the bundled Haifa OSM assets; IMU ~50 Hz, GPS ~1 Hz.
- Emphasise the headline: "no GPS in the hot path." This is the thesis of the whole project and frames every later slide.
- Pitfall to avoid: do not let the audience think this is a GPS-correction system. It is GPS-denied by design; GPS is only a validation reference on verified-clean days.
- This is the "map of the deck" slide — keep it fast (1 minute) and point forward: "the next five slides open each of these boxes."
