# Slide 08: Infrastructure Architecture — GPS-Free Navigation On-Device, No Backend

**Section:** Architecture · **Slide:** 8 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **Platform:** Android NDK (native C++17) + OpenCV 4.5.3; Kotlin + Jetpack Compose UI.
- **No backend for navigation:** no server, no cloud processing — the navigation (camera + IMU + on-device OSM road-matching) computes entirely on the phone with no GPS and no network in the hot path. The base map uses Google Maps tiles.
- **Offline OSM road-matching graph:** the bundled Haifa OSM road graph + geocoder drive an on-device Viterbi `LocalMatcher` that replaces Google Roads / Directions / OSRM `/match` for snapping. (OSM is the matching graph only — the displayed base map is Google Maps.)
- **Quality infrastructure:** deterministic C++ **replay harness** (re-runs recorded frames + IMU on identical input) + **CI scoring** on recorded fixtures; a **95-case Kotlin unit suite** (95 `@Test` across 14 files) + C++ unit suite.
- **Target device:** Samsung Galaxy S21 Ultra (SM-G998B), **Exynos 2100, Mali-G78** GPU.
- **Real-time on this hardware:** per-frame tracking **median 15.4 ms / max ~78 ms** vs the **200 ms** SDD budget; camera capture is locked at 30 fps (33 ms), and the effective processed VIO rate is ~23 fps (~43 ms) after frame-dropping.
- [Diagram: diagrams/03-data-flow.md]

## Talking Points (what the presenter SAYS)
- "The infrastructure story is short and deliberate: there's no navigation backend. NavSight has no server and no cloud for navigation. Capture, fusion, and map-matching all run inside one Android app, with no GPS and no network in the navigation hot path. The base map is drawn with Google Maps tiles."
- "The road-matching graph ships in the APK. Haifa's OpenStreetMap road graph and geocoder are bundled in the binary, and an on-device Viterbi matcher does the road snapping that we used to ask Google Roads or OSRM to do over the network. The OSM graph snaps the ball; the map you see behind it is Google Maps."
- "Our quality infrastructure is the part we're proudest of as engineers. We have a deterministic replay harness that re-runs recorded camera frames and IMU through the unmodified native engine, so every result is reproducible bit-for-bit — plus CI scoring on recorded fixtures and a 95-case Kotlin unit suite alongside a C++ unit suite."
- "And it all has to fit the hardware. The target is a Samsung S21 Ultra with the Exynos 2100 and a Mali-G78 GPU — not a Snapdragon. On that device, per-frame tracking runs at a median of 15.4 milliseconds against a 200-millisecond budget. The infrastructure is the phone, and the phone has room to spare."

## Why We Chose This Approach
- **Offline OSM road-matching vs Google Roads / OSRM cloud.** The online services are excellent — but every one of them is a network dependency, and our entire premise is operating where the network and GNSS can't be trusted. Haifa is tested under real regional GNSS jamming; a cloud map-matching service adds latency, rate limits, a privacy surface, and a single point of failure precisely when navigation matters most.
- **Alternatives considered:** Google Roads/Directions for snapping, OSRM `/match` for HMM map-matching. Both require connectivity and send the user's trajectory off-device.
- **Tradeoffs accepted:** we carry the road-matching graph in the APK (the bundled Haifa OSM assets), so map-matching coverage is limited to bundled regions, and the matcher's accuracy ceiling is the VIO trajectory quality rather than a remote service's. We own those tradeoffs deliberately.
- **Benefits gained:** no network latency, no rate limits, on-device privacy for the matching path, and — the headline — **jamming-resilient, GPS-denied-capable** operation. The on-device Viterbi `LocalMatcher` is a drop-in `MatchResult` replacement for the OSRM URL, so the architecture above it never knew the service went offline.
- **Why no navigation backend at all:** a thin server would have given us nothing the device can't do for navigation, while adding deployment, availability, and trust costs. Keeping the navigation on-device made the system testable end-to-end via the replay harness — which is why our validation is regression-gated, not hand-waved.

## Potential Questions (Defense)
**Q:** The bundled assets only cover Haifa — how does this scale to a real product across regions?
**A:** It scales by tiling. The bundled Haifa road graph plus geocoder is compact; OSM data compresses well and is region-partitionable, so a production build would download or bundle regional tiles on demand — the same model offline navigation apps already use. The architecture doesn't change: the `LocalMatcher` consumes whatever `OsmDataLayer` loads. We scoped the validated build to our actual test region (Haifa) by design, since that's where we could gather real jammed-environment rides.

**Follow-up Q:** Why not just download the road graph once and cache it instead of bundling?
**A:** Bundling guarantees the road-matching graph is present the first time the app runs in a dead zone — you can't assume the user had connectivity to pre-cache. For a GPS-denied use case, "snaps to roads on first launch, offline" is a requirement, not an optimization. A production version could layer on-demand tile downloads on top, but the bundled core stays.

**Q:** You stress this is an Exynos/Mali-G78 device — does that matter for your timing claims?
**A:** It matters a lot, and it's why we lock it in. The S21 Ultra's Mali-G78 is meaningfully less capable for ML than a Snapdragon Adreno, and we proved it bites: a DA3/V2 INT8 metric-depth model benchmarked at 722 ms on this hardware against a 100 ms budget, so the speed-path metric depth comes from the calibrated ground plane (camera height h = 1.05 m) instead of learned depth. Our 15.4 ms median is measured *on the actual target device*, not on a faster reference phone — so the headroom is real, not aspirational.

**Follow-up Q:** What happens if a frame does blow the budget — your max was ~78 ms?
**A:** Even the worst case sits at ~78 ms against the 200 ms SDD budget — a 2.5× margin. Camera capture is locked at 30 fps (33 ms); a ~78 ms frame exceeds that interval, but it's absorbed by the KEEP_ONLY_LATEST frame-dropping, which is why the effective processed rate settles around ~23 fps (~43 ms) rather than stalling the pipeline. When tracking genuinely can't produce a measurement, the inertial bridge carries dead reckoning rather than stalling, so a slow frame never freezes the dot.

## Speaker Notes
- Exact figures: median 15.4 ms, max ~78 ms, budget 200 ms; camera capture locked 30 fps (33 ms); effective processed VIO rate ~23 fps (~43 ms) after frame-dropping; 13× under-median and 2.5× worst-case margin are vs the 200 ms SDD budget (not vs the inter-frame interval).
- The replay harness builds under MSYS2/MinGW; CI scoring runs on recorded fixtures (`.github/workflows/replay.yml`). Mention "deterministic" — same input, same output — it's the credibility anchor for the results section.
- Stress that the target is **Exynos 2100 / Mali-G78**, explicitly *not* Snapdragon — it's in project memory because it dictated the depth choice: a DA3/V2 INT8 model at 722 ms ruled out learned metric depth, so the speed path uses the calibrated ground plane (h = 1.05 m).
- Build name on slide: **v1.0-osm**. The app targets compileSdk/targetSdk 34 (Android 14); Android 15 / API 35 is the test device's runtime OS, not the build target. No git hash.
- Pitfall: don't say "fully offline app." Precise claim: GPS-free navigation runs on-device (offline OSM road-matching, no network in the hot path); the base map uses Google Maps tiles. In an airplane-mode demo the navigation still runs offline; the base map shows cached/blank tiles.
