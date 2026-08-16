# NavSight — Beyond GPS · Defense Preparation Q&A Bank

**Project:** NavSight — Beyond GPS (Precision Navigation in GPS-Denied Environments)
**Degree:** B.Sc. Software Engineering — Final Project
**Team:** Roey Ben Harush · Tamir Sobuh · Morad Zubidat · **Supervisor:** Mr. Amit Dunsky
**Validated build:** v1.0-osm (compileSdk/targetSdk 34, Android 14) · **Test device runtime:** Android 15 / API 35 · **Device:** Samsung Galaxy S21 Ultra (SM-G998B), Exynos 2100, Mali-G78

This document aggregates **every "Potential Questions" block** from the 23 defense slides, organised by theme, followed by a **Hardest Questions** section of cross-cutting challenges with strong model answers.

> **Stage discipline reminders (apply to every answer):**
> - Refer to the build as **"v1.0-osm"** — never quote a git hash on stage.
> - GPS is removed only from the **navigation hot path** — never claim "no GPS at all". The displayed base map is **Google Maps tiles**; OSM is the road-matching graph only.
> - Map-matching bounds **lateral** drift; the **speed estimator** owns **along-track distance** — keep them decoupled.
> - The 8.9 km/h speed RMSE sits **at the reference's own noise floor** — it is not pure system error.

---

## THEME 1 — Architecture (Slides 06, 07, 08, 16)

### Q: Why split into four tiers instead of putting everything in the native layer for speed? *(Slide 06)*
**A:** Separation of concerns and lifecycle safety. Tier 1 must live in Kotlin/Compose because Android owns the UI thread, camera, sensors, and permissions. Tier 2 (the JNI bridge) is deliberately thin — it only marshals zero-copy `ByteBuffer` frames (the `processCameraFrameDirect` entry takes 9 arguments) and returns a `VioData` (a 30-field JNI return type carrying pose and status; speed is derived Kotlin-side from successive positions plus heading/scale, not a struct field), and it carries a `state_mutex` + `shared_ptr` pattern so the native engine survives a `stopVIO`/restart without a crash. Pushing UI into native code would gain nothing and would make us fight the Android lifecycle in C++. The boundary is where it belongs.

### Follow-up: Doesn't the JNI boundary itself become a performance bottleneck? *(Slide 06)*
**A:** No — that's why frames cross as a direct `ByteBuffer` (zero-copy), not a copied byte array. The marshaling cost is negligible next to the 15.4 ms median we spend inside the Tracker. The bridge moves a pointer, not pixels — and it stays thin even though there are ~47 external JNI functions across the boundary.

### Q: You say "no GPS in the hot path" — but the app still requests location permission. Isn't that a contradiction? *(Slide 06)*
**A:** GPS is captured at 1 Hz purely as a *secondary reference* and for an optional one-shot startup alignment — it never feeds the position estimate that draws the ball. NavSight is GPS-denied-*capable*: the visual-inertial pipeline runs entirely without a fix. We keep GPS available because removing it would discard a free validation signal on the days it happens to be healthy, but the navigation output does not depend on it.

### Q: Why does the `Tracker` own the EKF rather than having a separate fusion module? *(Slide 07)*
**A:** Because every EKF measurement update is fundamentally a *visual* event — a KLT track, an MSCKF feature, a zero-velocity detection, a map snap. Keeping the EKF inside the `Tracker` means the state and its updaters live next to the measurements that drive them, with no extra marshaling layer. The SDD called this conceptual unit `SensorFusionEngine`; in code it's `EKFState` owned by `Tracker`. The `IMUPreintegrator` stays separate because it runs on a different clock (50–200 Hz vs the ~23 fps effective processed-VIO rate) and only hands over preintegrated `delta_R/v/p` deltas.

### Follow-up: If the `Tracker` owns the EKF, how do you unit-test the filter in isolation? *(Slide 07)*
**A:** Through the deterministic offline replay harness: the unmodified native engine re-runs recorded frames + IMU on identical input, so we can A/B individual updaters and gates. That's how we measured the inertial bridge contributes +24% integrated distance on a turn-heavy ride (a stored deterministic offline A/B replay result), and it's backed by a 95-case Kotlin unit suite (95 `@Test` across 14 files) plus a C++ unit suite and CI replay scoring.

### Q: How do you control drift outdoors? *(Slide 07)*
**A:** The offline OSM map matcher is the drift constraint outdoors: it snaps the position to the road graph every step, with wrong-fork recovery bounded by design to 25–60 m at confidence ≥ 0.55. The ball cannot teleport across town, so the position stays honest on a road network.

### Q: How does the bundled-map approach scale to a real product across regions? *(Slide 08)*
**A:** It scales by tiling. The bundled Haifa OSM road graph plus geocoder is shipped in the APK; OSM data compresses well and is region-partitionable, so a production build would download or bundle regional tiles on demand — the same model offline navigation apps already use. The architecture doesn't change: the `LocalMatcher` consumes whatever `OsmDataLayer` loads. We scoped the validated build to our actual test region (Haifa) by design, since that's where we could gather real jammed-environment rides.

### Follow-up: Why not just download the map once and cache it instead of bundling? *(Slide 08)*
**A:** Bundling guarantees the map is present the first time the app runs in a dead zone — you can't assume the user had connectivity to pre-cache. For a GPS-denied use case, "works on first launch, offline" is a requirement, not an optimization. A production version could layer on-demand tile downloads on top, but the bundled core stays.

### Q: You stress this is an Exynos/Mali-G78 device — does that matter for your timing claims? *(Slide 08)*
**A:** It matters a lot, and it's why we lock it in. The S21 Ultra's Mali-G78 is meaningfully less capable for ML than a Snapdragon Adreno, and we proved it bites: a DA3/V2 INT8 metric-depth model benchmarked at 722 ms on this hardware against a 100 ms budget, so the speed-path metric depth comes from the calibrated ground plane (mount height h = 1.05 m) rather than a learned-depth model. Our 15.4 ms median is measured *on the actual target device*, not on a faster reference phone — so the headroom is real, not aspirational.

### Follow-up: What happens if a frame does blow the budget — your max was ~78 ms? *(Slide 08)*
**A:** Even the worst case sits at ~78 ms against the 200 ms SDD budget — a 2.5× margin. The camera captures at a locked 30 fps (33 ms interval); a ~78 ms frame exceeds that interval, but the `KEEP_ONLY_LATEST` frame-drop policy absorbs it by skipping the next frames, which is exactly what makes the *processed* VIO rate settle at ~23 fps (~43 ms effective). When tracking genuinely can't produce a measurement, the inertial bridge carries dead reckoning rather than stalling, so a slow frame never freezes the dot.

### Q: Where exactly does the displayed position come from — the EKF, or the map matcher? *(Slide 16)*
**A:** Both, in a split role. The EKF/Tracker produces motion — speed and heading change. The map matcher (`LocalMatcher`) owns the *displayed* position by advancing a ball along the road graph: the EKF's speed moves it along the locked way, and the gyro-relative heading steers it at junctions. So position is graph-constrained, while distance and turn decisions come from the VIO core. That decoupling is why the ball never leaves the road yet still travels the correct distance.

### Follow-up: Doesn't a 200 ms UI throttle make the ball look laggy? *(Slide 16)*
**A:** No — the native core still runs at the ~23 fps processed-VIO rate (~43 ms effective) and integrates continuously; only the *display* is sampled at ~200 ms. The underlying state is current; we just don't recompose Compose faster than the eye needs, which keeps the UI thread light.

### Q: The JNI boundary is a classic performance and correctness hazard — how do you keep it safe and fast? *(Slide 16)*
**A:** Speed: camera frames cross as a direct `ByteBuffer` with no copy, and results come back as one batched `VioData` return type (30 fields, pose + status) rather than many small calls. Correctness: a `state_mutex` plus a `shared_ptr` lifetime pattern keeps the native engine alive across start/stop, and there's an explicit Z-up to Y-up frame conversion at the boundary so the world-frame convention is handled in exactly one place. Per-frame tracking time was 15.4 ms median against a 200 ms budget, so the boundary is not the bottleneck.

### Follow-up: What happens to this flow when vision fails — blur, darkness, feature starvation? *(Slide 16)*
**A:** The flow degrades gracefully via the inertial bridge: `IMUPreintegrator` keeps feeding motion and the speed estimator predicts from forward acceleration for up to ~6 s, so the ball keeps moving down the road. On the 2026-06-04 ride we logged 36 looming (essential-matrix-degenerate) optical-flow fallbacks plus 49 ORB relocalization events — the pipeline recovered every time without a network call.

---

## THEME 2 — Algorithms & Math (Slides 12, 13, 14, 15)

### Q: Why is the forward-backward consistency check necessary — doesn't KLT already report a status flag? *(Slide 12)*
**A:** KLT's own status flag is not enough: on a moving scooter, motion blur and parallel lane lines produce points that KLT marks as "tracked" but whose flow is meaningless. The forward-backward check re-tracks each point in reverse and discards it unless it returns near its origin within a pixel threshold. This is what lets us trust the flow — in our camera frame figure the green arrows are the FB-verified flow that survives, versus the raw red KLT.

### Follow-up: Doesn't running KLT twice blow the frame budget? *(Slide 12)*
**A:** No — even with the backward track the median frame is 15.4 ms and the worst case is ~78 ms, both far under the 200 ms SDD budget (the camera itself captures at a locked 30 fps / 33 ms, with `KEEP_ONLY_LATEST` absorbing any heavy frame).

### Q: Why de-rotate the optical flow instead of letting the EKF handle rotation? *(Slide 12)*
**A:** Because the IPM ground-plane speed estimator needs pure translational flow to solve its least-squares speed — any residual rotation would bias the speed. We already have a high-rate gyro/Madgwick attitude estimate, so subtracting the rotational flow component is cheap and removes the coupling before either consumer sees the frame. The EKF still maintains its own attitude state; de-rotation just keeps the visual measurement clean.

### Follow-up: What if the gyro estimate is wrong — won't de-rotation introduce error? *(Slide 12)*
**A:** Gyro attitude is very accurate over a single ~43 ms processed-frame interval — bias drift is negligible at that timescale — so per-frame de-rotation is reliable. Longer-term gyro bias is separately estimated and corrected inside the EKF's 15-DOF navigation error state (the full IMU error-state is 19-DOF, adding the camera-IMU time offset and body→camera extrinsic).

### Q: Why use KLT optical flow instead of matching ORB (or other) descriptors on every frame? *(Slide 13)*
**A:** Cost and cadence. KLT tracks existing points with a pyramidal search and finishes a frame in a 15.4 ms median; matching descriptors on every frame means detecting and describing hundreds of features per image, which would not fit the 200 ms SDD budget on the Exynos 2100 / Mali-G78. We instead reserve ORB for relocalization only — we recorded 49 ORB relocalization events on the 2026-06-04 ride, versus tracking running on every single frame.

### Follow-up: What is the cost of KLT being wrong — doesn't optical flow drift? *(Slide 13)*
**A:** Raw KLT does produce bad tracks under blur and occlusion, which is exactly why we never use raw flow. The forward-backward consistency check discards any point that doesn't re-track back to its origin, and on top of that the IPM speed stage applies a statistical vote/zero-witness gate, so a few bad tracks cannot move the estimate. When tracking degrades wholesale, ORB relocalization re-anchors us.

### Q: What happens when KLT loses tracking entirely — does the system just stop? *(Slide 13)*
**A:** No. Two things happen. First, ORB relocalization re-anchors the visual state to a previously seen keyframe — we observed 49 such relocalization events on the 2026-06-04 ride. Second, in the meantime the complementary inertial bridge keeps predicting forward speed from the IMU (trusted for up to ~6 seconds), and the map matcher keeps the ball constrained to the road graph. So a tracking loss degrades gracefully rather than freezing the display.

### Follow-up: How do you know the relocalizations were genuine recoveries and not false matches? *(Slide 13)*
**A:** Each relocalization is gated before it is accepted, and the recorded behaviour on that ride shows tracking resuming cleanly after the 49 relocalization events. The same ride also logged 36 looming (essential-matrix-degenerate) optical-flow fallbacks, which shows the visual front-end and the recovery path were actively covering the gaps — the mechanisms work together rather than masking failures.

### Q: Isn't map-matching just hiding VIO error — if the road graph were wrong, you'd be lost? *(Slide 14)*
**A:** It constrains rather than hides. The matcher's *along-track* position still comes from our estimated speed, so distance accuracy is independent of the snap — Route A measured 1,195 m against a Google-measured 1,280 m, which is 93.4%, with the ball staying on-road the whole time. The graph removes *lateral* drift, which is exactly the drift VIO is worst at. And we bound any mismatch to a 25–60 m re-acquire at confidence ≥ 0.55, so a graph error degrades gracefully rather than catastrophically.

### Follow-up: Why a 20-metre emission sigma and not tighter? *(Slide 14)*
**A:** σz = 20 m is the Newson-Krumm working value and it has to cover the combined budget of VIO lateral drift plus road-centerline-versus-lane offset. A tighter sigma would over-trust an estimate that genuinely has tens of metres of lateral uncertainty and would cause the matcher to thrash between adjacent candidates; 20 m keeps the emission soft enough that the transition term decides at forks.

### Q: How does the ball pick the correct road at a junction without GPS? *(Slide 14)*
**A:** By the gyro-relative heading offset. The ball advances by speed along the locked way; at a junction we compare each outgoing branch's tangent against our heading change since the anchor, `θ_anchor + Δψ_gyro`, and take the branch with the smallest angular error. The gyro is reliable for the few seconds a junction takes, so the turn we physically made selects the branch — no GPS fix needed.

### Follow-up: What stops it from getting stuck circling a roundabout? *(Slide 14)*
**A:** A RING EJECT mechanism. On the validation ride we started the app *on* a roundabout, detected it at start, and exited cleanly at ~9 s (observed) with no spurious re-entry. The exit re-acquires the ball onto the suggested exit segment with re-entry hysteresis (RING EJECT fires after 460 ticks, conf 0.80 on the ring), so a circling trace can't keep matching its own ring indefinitely.

### Q: The flat-road assumption seems fragile — what happens on a hill or a banked curve? *(Slide 15)*
**A:** The plane model is only the *vote source*, not the final speed. When the geometry breaks (slope, blur, feature starvation), the per-point estimates fail the 3-sigma vote gate and we fall through to the inertial bridge, which integrates forward acceleration for up to ~6 seconds — bounded because a 0.3 m/s² accel-bias integrates to only ~1.8 m/s over that window — before decaying. So a hill doesn't produce a wrong speed; it produces a *bridged* speed that re-anchors to vision as soon as flat, textured road returns.

### Follow-up: How accurate is the resulting speed in practice? *(Slide 15)*
**A:** Against a conditioned GPS reference over 5-second windows, RMSE was 8.9 km/h with a −1.9 km/h bias, and median 36.3 km/h versus 35.1 km/h — within 3.5%. That RMSE actually sits *at* the GPS reference's own noise floor of about 4–6 km/h at 5 s, so we're at the resolution limit of the thing we're comparing against, not above it.

### Q: Why does standstill read *exactly* zero instead of a small filtered value? *(Slide 15)*
**A:** The zero-witness branch is an explicit decision, not a clamp. When ≥5 points each test statistically indistinguishable-from-zero (|v_i| < 3σ with a sub-1-m/s floor), we hard-lock the output to 0.0 rather than taking a median of small noisy numbers. On the validation ride the displayed speed read exactly 0.0 km/h throughout the stop (~27 frames at exactly 0.0), which never exceeded ~0.9 km/h. A magnitude-only filter cannot do this because it has no way to distinguish "small motion" from "rectified noise."

### Follow-up: How does the inertial bridge blend with vision, and what is α = 0.15? *(Slide 15)*
**A:** The mechanism is: every frame we predict `v += a_fwd·dt`; once ≥5 votes return we correct by an EMA toward the vote-median (and hard-lock to 0 on ≥5 zero-witnesses). α = 0.15 (`kGpBridgeDecayAlpha`) is not the correction gain — it is the *post-budget decay rate*: once the bridge passes its ~6 s budget without a vision fix, the predicted speed decays at 0.15 rather than free-running. So prediction carries continuity, vision re-anchors the moment votes are stable, and a stale bridge winds down gently instead of asserting false confidence.

---

## THEME 3 — Validity of Results vs GPS (Slides 04, 11, 17, 18, 19, 23)

### Q: GPS jamming sounds like an edge case — why build an entire system around it? *(Slide 04)*
**A:** It is not an edge case in our test region: we observed real, repeatable jamming on multiple Haifa rides. On Route A, jammed GPS over-reported +33% (1,705 m vs the true 1,280 m); on other jammed rides we observed multi-second GPS position freezes while moving. And the same denial happens routinely in tunnels, parking garages, urban canyons, and indoors. The motivation is general; the jamming just made the failure mode unmissable.

### Follow-up: If GPS is so unreliable here, why does the app still include GPS at all? *(Slide 04)*
**A:** Because GPS is genuinely useful when it is healthy, and removing it would make NavSight worse in the common case. We keep it but demote it: it is an optional, untrusted reference, never ground truth. The visual-inertial hot path runs entirely without a fix, so a corrupted GPS reading can never derail the trajectory.

### Q: How do you know it was jamming and not just a buggy sensor or your own pipeline? *(Slide 04)*
**A:** We cross-checked against map-measured ground truth, which is jamming-resilient. A healthy GPS day on the same device gave a median fix accuracy of ~4 m with no freezes, whereas the jammed rides showed multi-second position freezes and, on Route A, the +33% inflation. We also separately ruled out a sim-pipeline bug (re-sampling the same fix each tick) that had once mimicked a freeze — the real rides were genuinely jammed.

### Follow-up: Couldn't the user just notice the freeze and ignore it? *(Slide 04)*
**A:** No — the failure is silent. The app reports a plausible-looking frozen position with no error flag, so a human has no way to distinguish a real stop from a jammed freeze. That is exactly why an independent, GPS-free estimate of position and speed is necessary.

### Q: If GPS is available, why not use it to improve accuracy? *(Slide 11)*
**A:** Because in our actual operating environment GPS is the unreliable signal, not the trustworthy one. Under real jamming we observed multi-second position freezes while moving, and on Route A a path inflated by +33% (GPS reported 1,705 m on a route that truly measures 1,280 m). Map-measured routes are our jamming-resilient ground truth. We keep GPS in the app — we never removed it — but it is not in the navigation hot path.

### Follow-up: So GPS is completely unused at runtime? *(Slide 11)*
**A:** It is sampled at about 1 Hz and used only as a secondary reference and, on clean days, for validation. The displayed position is driven entirely by the camera, IMU, and the OSM road graph.

### Q: What is the slowest part of this runtime loop, and does it bottleneck the frame rate? *(Slide 11)*
**A:** The per-frame visual tracking is the heaviest stage, with a median of 15.4 ms and a worst case of about 78 ms. The camera captures at a locked 30 fps (33 ms interval); the processed VIO rate settles at ~23 fps (~43 ms) after `KEEP_ONLY_LATEST` frame-dropping, which absorbs the occasional ~78 ms frame. We are 13× under the 200 ms SDD budget at the median and still have a 2.5× margin at worst case — both margins are against the 200 ms budget, not the inter-frame interval.

### Follow-up: What happens to a frame that does exceed the budget? *(Slide 11)*
**A:** Heavy work runs on a dedicated native VIO executor thread, so a slow frame does not stall the UI; if vision genuinely fails on a frame, the inertial bridge carries speed forward and the ball keeps moving on the road.

### Q: Why don't you just use GPS as ground truth like every other navigation paper? *(Slide 17)*
**A:** Because GPS is precisely the signal we are replacing. Under the real Haifa jamming we observed multi-second position freezes while moving, and on Route A a +33% inflated path length — GPS reported 1,705 m on a route that truly measures 1,280 m. Using a corrupted signal as truth would let the corruption flatter or unfairly penalise us. Map-measured route distance is resilient to jamming, so it is the honest baseline.

### Follow-up: Then how can you trust GPS even as a secondary speed reference? *(Slide 17)*
**A:** Only conditionally. We use it for speed only on rides whose GPS health we verified independently — median fix accuracy 4 m, no freezes or jumps — and even then we run a plausibility filter (|dv/dt| > 3 m/s² rejected) and 5-second window averaging, which pushes residual reference noise down to about 4–6 km/h. We never use GPS from a ride that shows jamming artifacts.

### Q: How do you know your A/B feature comparisons aren't just run-to-run variation? *(Slide 17)*
**A:** They aren't, because we use a deterministic offline replay. The unmodified native engine re-runs the exact same recorded camera frames and IMU samples, so the only difference between A and B is the feature flag. That is how we cleanly measured the inertial bridge contributing +24% integrated distance (62 m → 77 m) on identical input — no walk-to-walk noise involved (a stored deterministic offline A/B replay result).

### Follow-up: Is the replay engine the same code that runs on the phone? *(Slide 17)*
**A:** Yes — it is the unmodified native C++ engine; replay only changes how frames and IMU are fed in, not the algorithm. We also back it with a 95-case Kotlin unit suite (95 `@Test` across 14 files), a C++ unit suite, and CI scoring on recorded fixtures so the replayed code is the shipped code.

### Q: Your distance accuracy is 91–93%. Where does the remaining 7–9% go? *(Slide 18)*
**A:** It is overwhelmingly turn under-read before the inertial bridge engages — the camera loses near-road texture in tight turns. Our own A/B proves this: enabling the bridge recovers +24% of integrated distance on a turn-heavy ride (a stored deterministic offline A/B replay result). The residual after that is consistent under-read, which is why both numbers land just under the route length rather than scattering above and below — a stable, bounded, *understood* error, not random drift.

### Follow-up: Why does NavSight read 1,195 m while GPS read 1,705 m on the same 1,280 m route? *(Slide 18)*
**A:** That is the headline of the project. NavSight is within 7% of the true 1,280 m; jammed GPS inflated the path by +33% to 1,705 m. We hold accuracy on exactly the route where GPS — the incumbent — fails catastrophically. The cumulative-distance figure makes this visually obvious.

### Q: An 8.9 km/h speed RMSE sounds large for a navigation system. *(Slide 18)*
**A:** It sits at the reference's own resolution limit. The conditioned GPS speed reference has a noise floor of about 4–6 km/h at 5 s windows, and raw 1 s differencing is around 20 km/h. So 8.9 km/h is essentially "as good as the yardstick can measure," and the medians confirm it — 36.3 vs 35.1 km/h, within 3.5%.

### Follow-up: Could the small −1.9 km/h bias accumulate into long-route error? *(Slide 18)*
**A:** It is bounded, not accumulating, because speed is corrected against the median of voting road pixels whenever ≥5 votes return (with α = 0.15 only as the post-budget decay rate, not the correction gain), and the map matcher constrains position to the road graph. The bias shows up as the consistent slight under-read in distance (91–93%), which we already account for — it does not free-run.

### Q: Isn't it convenient to blame the GPS reference for your RMSE? How do you prove the error is the reference's and not yours? *(Slide 19)*
**A:** Two independent checks. First, the medians nearly coincide — 36.3 vs 35.1 km/h, within 3.5% — so there is no systematic scale error on our side; a system error would shift the median, not just inflate variance. Second, the reference noise floor is derived from first principles: σ_v,ref ≈ √2 · σ_pos / τ ≈ 5.7 m/s ≈ 20 km/h per 1 s sample, which 5 s averaging reduces to 4–6 km/h. Our 8.9 km/h is the convolution of our true error with that floor — it is bounded below by the reference, not invented.

### Follow-up: If you had a perfect reference, what RMSE would you expect? *(Slide 19)*
**A:** Lower than 8.9 km/h, because a large share of the measured RMSE is the reference's own 4–6 km/h noise added in quadrature. We can't quote an exact figure without a survey-grade reference — and we won't fabricate one — but the coinciding medians indicate the underlying systematic error is small.

### Q: You admit turns under-read and blur degrades the camera. Why should we accept the system as validated? *(Slide 19)*
**A:** Because both limitations are root-caused and *mitigated*, not hidden. Turn under-read is handled by the complementary inertial bridge — a stored deterministic offline A/B replay shows it recovers +24% (62→77 m). Blur is handled by the per-point σ-floor and the two-sided vote/zero-witness taxonomy, so blurred or texture-starved frames are rejected rather than producing wrong speed, and the accel bridge carries those frames. The end-to-end result after these mitigations is still 91–93% distance accuracy.

### Follow-up: Why is standstill *exactly* zero and not "near zero"? *(Slide 19)*
**A:** By design. We use a two-sided taxonomy: a pixel only votes a non-zero speed if it clears 3σ and is forward-coherent; otherwise, if its noise floor is ≤1 m/s, it is a zero-witness. Five or more zero-witnesses force a hard zero-lock. A one-sided magnitude gate would rectify up to ~34% (analytical estimate) of random-direction tracker noise into a positive speed — that's exactly the phantom-creep failure our taxonomy eliminates. The data shows it: the displayed speed reads exactly 0.0 km/h throughout the stop.

### Q: If you had to defend one number as your strongest result, which is it and why? *(Slide 23)*
**A:** The speed result, because RMSE 8.9 km/h sits at the conditioned GPS reference's own ~4–6 km/h noise floor at a 5 s window — we are accurate to the limit of what the reference can measure, with a median of 36.3 vs 35.1 km/h, within 3.5%. It is the result hardest to dismiss as luck, because it is bounded by the reference's resolution, not ours.

### Follow-up: Why report distance as a range, 91–93%? *(Slide 23)*
**A:** Because it is two independent rides with two ground-truth methods: Route A against Google-measured distance gave 93.4% (1,195 m of 1,280 m), and ride 18:02 against verified-healthy GPS gave 91% with ρ = 0.91. Reporting both, rather than the better one, is the honest summary.

---

## THEME 4 — Engineering Decisions (Slides 03, 09, 10, 22)

### Q: Which parts did *you personally* build and own? *(Slide 03)*
**A:** My confirmed ownership is the inertial half of the pipeline: the IMU pre-integration module (Madgwick attitude plus gyro/accel preintegration), the sensor-fusion integration into the 15-DOF error-state EKF, sensor synchronization and calibration (including the Allan-calibrated noise model and timestamp alignment), and the integration-testing and documentation effort — notably the deterministic replay harness used to validate every fix before walking.

### Follow-up: How did you keep three people from breaking each other's code on a tightly-coupled VIO system? *(Slide 03)*
**A:** Two mechanisms. First, ownership followed the 4-tier architecture, so each member worked behind a stable interface — the Kotlin↔C++ boundary is the JNI contract (the `processCameraFrameDirect` entry takes 9 arguments and returns the 30-field `VioData`), which let the UI and the native core change independently. Second, every behavior-changing edit had to pass the deterministic C++ replay harness and CI scoring on recorded fixtures before an on-device walk — so regressions surfaced on identical recorded input, not in the field.

### Q: You spent three days chasing a drift bug before reading the data. Isn't that a process failure you should hide, not present? *(Slide 09)*
**A:** We present it deliberately because it produced our most valuable engineering rule. The real fault lived upstream of any filter constant: a ~6–10° tilt mis-cancelled gravity, integrating into roughly 800 m of phantom Z drift over the run. We spent days suspecting filter parameters when the fix belonged at the gravity-alignment layer (the illustrative residual band is g ± 0.8 m/s²). One run of our residual-analysis script showed the residuals were position-dominated and physically impossible, ending the issue in about 20 minutes — fixed by the live gravity-alignment update. The takeaway — "read the data before tuning constants" — is now a binding team convention.

### Follow-up: How do you guarantee this never recurs? *(Slide 09)*
**A:** Two safeguards. First, every state transition and convention boundary now emits LOGI plus event counters, so the data needed to diagnose is captured by default rather than reconstructed under pressure. Second, the gravity-alignment update keeps p_G physically bounded by observing 2-DOF roll/pitch from the accelerometer direction; the ~6–10° tilt that caused the 800 m drift can no longer accumulate unobserved.

### Q: What did you keep in the VIO core, and how do you know it earns its cost? *(Slide 09)*
**A:** The live core is a tightly-coupled error-state EKF with KLT optical flow (forward-backward checked, de-rotated), MSCKF covariance updates, ORB relocalization, IMU preintegration (Madgwick + Forster midpoint), ZUPT/gravity-alignment/stationary-accel updates, the IPM ground-plane speed path with its vote/zero-witness taxonomy and inertial bridge, and the offline OSM Viterbi map-matcher. We verified the recovery path fires on the 2026-06-04 ride: 49 ORB relocalization events and 36 looming (essential-matrix-degenerate) optical-flow fallbacks were logged, confirming it matters.

### Q: Making the map-matcher the source of truth feels like a crutch — aren't you just hiding poor VIO behind the road graph? *(Slide 10)*
**A:** No — the matcher constrains position but cannot invent motion. The ball advances along the locked way *by the VIO-estimated speed* (s_{k+1} = s_k + v_k·Δt) and is steered at junctions by the gyro-relative heading offset. If the VIO speed is wrong, the matched distance is wrong — which is exactly what we report: Route A measured 1,280 m on Google Maps, NavSight produced 1,195 m (93.4%), and ride 18:02 produced 91% (ρ = 0.91). The matcher bounds *lateral* drift to the road; it does not fabricate the *along-track* distance, so it cannot mask a poor speed estimate.

### Follow-up: What is the failure mode if the matcher locks onto the wrong road? *(Slide 10)*
**A:** It is bounded by design. Wrong-fork recovery is constrained to 25–60 m and only acts at confidence ≥ 0.55, with a measured split between bad yanks (0.40–0.42) and healthy matches (0.63–0.86). It can correct a fork mistake within a block but can never teleport across town — the graph constraint guarantees the position is always on a real road, and the distance band caps how far a correction can jump.

### Q: With a road graph driving position, is this "really a VIO system"? *(Slide 10)*
**A:** It is fully a tightly-coupled VIO system: a 15-DOF error-state EKF fusing camera and IMU with MSCKF covariance updates, ORB relocalization, IMU preintegration, and the IPM ground-plane speed path — all verified used (36 looming optical-flow fallbacks, 49 ORB relocalization events on the 2026-06-04 ride). The offline OSM map-matcher supplies the *global* position constraint outdoors, where a road graph exists; the VIO core supplies the motion (speed and heading) that moves the ball along it. The two are decoupled by design, not a substitute for one another.

### Follow-up: How did you keep the build lean for the outdoor target? *(Slide 10)*
**A:** We scoped the pipeline to what the GPS-denied *outdoor* target actually needs — camera + IMU + on-device OSM road-matching — and validated it with the 95-case Kotlin unit suite and the deterministic replay harness, with no regression in distance or standstill behaviour.

### Q: Is learned-inertial velocity realistic on this hardware? *(Slide 22)*
**A:** Yes — we treat it as a TFLite path that reuses an existing, proven on-device inference seam rather than introducing a new runtime. We also kept learned metric-depth out of the speed hot path because a DA3/V2 INT8 bench cost 722 ms vs our budget, so we know how to keep model choices honest to the Mali-G78.

### Q: How hard is it to add a new region or a new vehicle? *(Slide 22)*
**A:** A new region is additional bundled OSM assets, the same form as the bundled Haifa pack. A new vehicle is a new calibrated per-mode K slot alongside the existing walk/run/vehicle slots — for example a measured scooter K from a stop/brake calibration. Both extend existing mechanisms rather than changing the pipeline.

---

## THEME 5 — Scope, Limitations & Impact (Slides 01, 02, 05, 20, 21)

### Q: Why call it "Beyond GPS" — does the app remove GPS entirely? *(Slide 01)*
**A:** No. The navigation *hot path* (camera + IMU fusion through the EKF, then offline OSM road-snapping) is fully GPS-free and produces the live "ball on the road" with speed. We deliberately keep GPS in the app as an optional reference, because removing it would reduce capability; the point is that NavSight does not *depend* on a live GPS fix. We validated this under real regional GNSS jamming in Haifa.

### Follow-up: What exactly does "v1.0-osm" mean? *(Slide 01)*
**A:** It is our validated release build that ships the offline OpenStreetMap *road-matching* layer — the on-device Viterbi LocalMatcher that snaps the ball to the road graph (it replaced online Roads/Directions/OSRM calls). The matching graph runs with no network in the navigation hot path; the displayed base map is Google Maps tiles. It bundles the Haifa OSM assets in the APK. The build targets compileSdk/targetSdk 34 (Android 14); the test device runtime is Android 15 / API 35. All the field results in this deck come from this exact build.

### Q: In one sentence, what is the core contribution here? *(Slide 02)*
**A:** GPS-free navigation computed on-device — a GPS-independent pipeline that achieves road-level position and within-3.5% speed accuracy on a commodity Android phone by tightly fusing a single rear camera with the IMU and constraining the result to an offline OSM road-matching graph (no GPS, no network in the navigation hot path; the base map uses Google Maps tiles) — validated under real GNSS jamming, where GPS itself fails.

### Follow-up: Why is "offline" such a central design point rather than a nice-to-have? *(Slide 02)*
**A:** Because the threat model is jamming and denial. An online map-matching service (Roads, OSRM) needs a network and a position fix to query — exactly the things that are unreliable in our environment. By bundling the Haifa road graph in the APK and running a Viterbi matcher on-device, the navigation has no network latency, no rate limits, and is jamming-resilient where GPS breaks. (The displayed base map still uses Google Maps tiles, which fall back to cached or blank tiles offline.)

### Q: Why is "relative to a start point" acceptable — don't users need their absolute location? *(Slide 05)*
**A:** For the GPS-denied use case, what the user needs is to follow a route correctly from where they are, and a road-snapped relative track delivers exactly that: the ball stays on the right road and reaches junctions on time. We anchor the start with a one-shot compass alignment and constrain everything to the OSM graph, so the trajectory is meaningful on a real map without needing a continuous absolute fix.

### Follow-up: What happens if the relative track drifts over a long ride? *(Slide 05)*
**A:** Drift is bounded by the map graph rather than left to grow: the position is a ball constrained to the road network, and wrong-fork recovery is bounded by design to 25–60 m at confidence ≥ 0.55 — it can never teleport across town. On a 1,280 m route we still measured 93.4% distance accuracy, so the bounded drift stays well inside the road geometry. This is why we chose a map-constrained estimator over free dead reckoning.

### Q: Why exclude turn-by-turn voice navigation — wouldn't that make it a real product? *(Slide 05)*
**A:** It is a legitimate feature, but it is a separate concern layered on top of positioning, and the signed SDD scoped it out so we could fully solve the hard core: producing a trustworthy GPS-free position and speed. Voice guidance, full 3D SLAM, and cloud processing are all explicit non-goals; keeping them out is what let us hit the real-time budget and validate the core on real rides.

### Follow-up: Is the system architecturally able to add voice guidance later? *(Slide 05)*
**A:** Yes — the output is a road-snapped position with heading and speed on the OSM graph, which is exactly the input a routing/guidance layer consumes. The scope decision was about project focus, not an architectural dead end.

### Q: Who actually benefits from this, and in what real situation? *(Slide 20)*
**A:** Anyone moving through a GPS-denied environment with only a phone — commuters and riders in jammed regions like wartime Haifa, indoor or tunnel navigation, and privacy-sensitive users who do not want their location streamed to a cloud service. We demonstrated the core case on a scooter under real regional GNSS jamming and held 91–93% distance accuracy where GPS itself was wrong by +33%.

### Follow-up: Is this just a research demo, or could it ship? *(Slide 20)*
**A:** It runs in real time on a stock S21 Ultra with 13× median headroom under the 200 ms frame budget, the road-matching graph is bundled as a Haifa OSM asset in the APK, and the navigation needs no network — those are shipping-grade constraints, not lab constraints. The honest limitation is region coverage (we ship Haifa) and low-light speed, which are on the future-work slide.

### Q: GPS already works almost everywhere — why invest here? *(Slide 20)*
**A:** "Almost everywhere" is exactly the gap. Our own measurements show GPS failing precisely when it is needed: multi-second frozen positions while moving and, on Route A, a +33% path-length inflation under jamming. NavSight is designed for the cases where GPS is the unreliable signal, and it treats jamming-resilient map-measured distance — not GPS — as the truth.

### Q: How do we know the live demo isn't quietly using GPS? *(Slide 21)*
**A:** We run it in airplane mode with no fix: the navigation runs offline (camera + IMU + on-device OSM road-matching) and the base map shows cached or blank Google Maps tiles. The VIO chip plus debug panel show the position is map-matched from the camera+IMU trajectory, not from a location provider. The whole point of v1.0-osm is that the navigation hot path is GPS-free and the OSM road-matching graph is a bundled on-device asset — there is no network to fall back to.

### Follow-up: What if the live ride drifts badly in front of the committee? *(Slide 21)*
**A:** That is exactly why we carry the recorded Route A fallback — same engine via deterministic replay, validated to 93.4% distance with the U-turn retrace. We show routeA_matched_traj.png and ui3.png and narrate the identical behaviors, so the evidence is unchanged.

### Q: Why Route A specifically and not the 18:02 ride? *(Slide 21)*
**A:** Route A is our best-documented case: Google-measured ground truth of 1,280 m, NavSight 1,195 m at 93.4%, a clean U-turn that retraces the same road, and a full Fig 9–11 case study. It demonstrates the on-road constraint and distance accuracy together on a single route, which is the most convincing single artifact we have.

### Q: What is the single biggest limitation you want us to know? *(Slide 23)*
**A:** Low-light speed and region coverage. Motion blur degrades the near-band optical flow, which is why a camera exposure cap is our named next lever, and we currently ship only the Haifa OSM region. Neither affects the validated results, and both are extensions of existing mechanisms rather than redesigns.

---

## HARDEST QUESTIONS — Cross-Cutting Challenges & Model Answers

These are the toughest questions a sharp examiner can ask. They cut across multiple slides; the answers below combine the strongest evidence from the whole deck.

### H1. Monocular scale ambiguity — how do you get *metric* speed from a single camera with no inherent scale?
**A:** Three independent anchors, with the strongest one carrying the speed path. (1) The **IPM ground plane** gives a *direct geometric* metric speed from a single calibrated constant — mount height h = 1.05 m below the camera. For each road pixel we compute its plane depth `Z_i = -h/(n̂·r_i)`, the flow-per-unit-speed `a_i`, and a least-squares per-point speed `v_i = -(f_i·a_i)/(a_i·a_i)·(1/Δt)`. No learned depth, no network — one physical measurement turns pixels into metres-per-second. (2) The IMU resolves scale through `s = d_imu/d_vis` (EMA-smoothed) for the EKF velocity state. We deliberately keep learned depth *out* of the speed hot path because a DA3/V2 INT8 metric-depth bench cost 722 ms on the Mali-G78 against a 100 ms budget — the speed-path metric depth comes from the calibrated ground plane, not a learned-depth model. The proof that the metric scale is right: the speed medians nearly coincide with GPS (36.3 vs 35.1 km/h, within 3.5%), so there is no systematic scale error.

### H2. Why should we trust your *speed* without GPS, when GPS is the obvious speedometer?
**A:** Because in our environment GPS is the *less* trustworthy speedometer — on Route A jammed GPS inflated a 1,280 m route to 1,705 m (+33%), and on other jammed rides we observed multi-second position freezes while moving. We validate speed only against *conditioned, health-verified* GPS (median fix 4 m, plausibility-filtered at |dv/dt| > 3 m/s², 5 s window-averaged). Even then the reference's own noise floor is ~4–6 km/h at 5 s (raw 1 s differencing is ~20 km/h, derived as σ_v,ref ≈ √2·σ_pos/τ ≈ 5.7 m/s). Our measured RMSE of 8.9 km/h sits *at* that floor — we are as accurate as the yardstick can resolve — and the coinciding medians prove the error is noise-limited, not bias-limited. Distance, our primary metric, is validated against jamming-resilient map-measured ground truth, not GPS at all.

### H4. How is the gravity-drift lesson generalisable beyond this one bug? Isn't "we spent three days then read the data" just an anecdote?
**A:** It generalises into a falsifiable engineering discipline, and that is why we present it rather than hide it. The pattern was: we chased a fault at the wrong layer — a ~6–10° tilt mis-cancelled gravity into ~800 m of phantom Z drift, while we suspected filter parameters. Three days of tuning was tuning the wrong layer; one run of the residual-analysis script showed the residuals were position-dominated and physically impossible, ending it in 20 minutes (fixed by the live gravity-alignment update). The generalisable rule — **read the data before tuning a constant** — is now enforced structurally: every state transition and convention boundary emits LOGI + event counters by default, so the diagnostic data exists *before* the next mystery, not reconstructed under pressure. The same discipline produced the IPM root-cause (gradient energy 10.2→3.5 under motion, 77% of cruise frames feature-starved) and the map-matcher root-cause (49/49 curve steps were 2–7°, not turns) — each was a data finding, not a guess.

### H5. What if the road simply isn't in your OSM map — a new road, a parking lot, a footpath, or a region you didn't bundle?
**A:** Two layered answers. First, *coverage*: the validated build ships the Haifa region; the architecture is region-partitioned, so a production build tiles additional OSM regions on demand — the `LocalMatcher` consumes whatever `OsmDataLayer` loads, no pipeline change. Second, *graceful degradation off-graph*: when there is no nearby candidate within the 30 m radius, the system is doing pure visual-inertial dead reckoning, which the inertial bridge carries for ~6 s and ORB relocalization recovers. So an off-map excursion degrades to bounded VIO rather than failing — the map is a constraint when present, not a hard dependency.

### H6. The map-matcher makes your position look perfect because it can't leave the road — aren't you just hiding poor VIO behind the road graph?
**A:** The matcher constrains *lateral* position but cannot fabricate *along-track* motion — those are deliberately decoupled. The ball advances along the locked way purely by the VIO-estimated speed (`s_{k+1} = s_k + v_k·Δt`); if the speed is wrong, the matched distance is wrong, and that is precisely what we report honestly: 93.4% on Route A, 91% on ride 18:02. A perfect-looking on-road line with a 9% distance under-read is exactly the signature of "lateral constrained, along-track not faked." If the matcher were hiding poor VIO, our distance would be arbitrary; instead it is a stable, bounded, understood under-read that the inertial bridge recovers +24% of.

### H7. The flat-ground IPM assumption and the gyro-relative junction steering both look fragile. What is the real-world failure envelope?
**A:** We scope each assumption to its valid window and cover the gap. IPM's flat-plane model is only the *vote source*: on a slope, banked curve, blur, or feature-starved frame the per-point estimates fail the 3σ vote gate, and we fall through to the inertial bridge (forward-accel integration, bounded — 0.3 m/s² bias → 1.8 m/s over the ~6 s horizon) until flat, textured road returns. Gyro junction steering only has to be reliable for the *few seconds* a junction takes, where gyro bias drift is negligible; longer-term bias is estimated inside the EKF's 15-DOF error state. The roundabout case — historically the worst — is now handled by RING EJECT hysteresis: started on a ring, detected at start, exited cleanly at ~9 s (observed; RING EJECT after 460 ticks, conf 0.80 on the ring), no re-entry. So the failure envelope is "sustained non-planar road under sustained vision loss," and even there the system bridges rather than fabricates.

### H8. Why an EKF at all? Optimization-based VIO (VINS-Mono) and ORB-SLAM are the state of the art — did you pick the easy option?
**A:** We picked the *appropriate* option for a real-time phone with a map constraint. A tightly-coupled MSCKF-style error-state EKF lets the IMU directly constrain visual scale and rotation, has principled covariance, low CPU, and a clean dead-reckoning fallback when vision fails — all of which fit the ~43 ms processed-frame budget on a Mali-G78. We explicitly considered sliding-window optimization (VINS-Mono) and pure feature-SLAM (ORB-SLAM) and rejected them as heavier and *unnecessary once map-matching took over global drift control* — the very thing those heavier methods spend their compute on (global consistency) is what the offline road graph already provides for free. The honest cost we pay is filter-consistency fragility (the gravity-cancellation episode), which we mitigate with a dedicated gravity-alignment update and the live MSCKF Mahalanobis chi-squared gate (5× χ²₀.₉₅(2K), per-row Huber δ = 2.4477, Joseph-form covariance update). So it is a defensible engineering match to the constraints, not a shortcut.

### H9. Your two strongest distance numbers come from only two rides. Isn't your validation sample too small to claim 91–93%?
**A:** The headline distance numbers are two independent rides with two independent ground-truth methods (Route A vs Google-measured 1,280 m; ride 18:02 vs verified-healthy GPS 869 m), which is why we report a *range* rather than a single flattering figure. But the validation is broader than two rides: it rests on a **deterministic replay harness** that re-runs recorded frames+IMU bit-for-bit (removing walk-to-walk noise entirely), a **95-case Kotlin unit suite** (95 `@Test` across 14 files), a **C++ unit suite**, and **CI scoring on recorded fixtures**. The A/B results (inertial bridge +24%, 62→77 m — a stored deterministic offline A/B replay) and behavioural results (displayed speed exactly 0.0 throughout the stop; roundabout exit at ~9 s; 36 looming optical-flow fallbacks + 49 ORB relocalization events on the 2026-06-04 ride) are all reproducible on identical input. The small-sample concern applies to a *field* metric; we counter it with reproducibility, not volume — every claim can be re-run.

### H10. If a jammer can fake GPS, what stops the same adversary from defeating NavSight?
**A:** NavSight's inputs are physically un-spoofable from a distance in the way an RF signal is. GPS is a received radio signal an adversary can overpower or forge over the air; NavSight's position comes from the **device's own camera and IMU** observing the physical world, plus a **road graph already inside the APK**. There is no signal to jam and no network round-trip to intercept — that is the entire architectural point of "no GPS in the hot path, no backend, offline map." An adversary would have to physically obscure the camera or move the phone, not transmit anything. Jamming corrupts at most NavSight's *optional* secondary GPS reference, never the trajectory that draws the ball.

### H11. Why exactly 0.0 km/h at standstill? A real speedometer never reads a perfect zero — isn't that a cosmetic clamp that could hide real slow motion?
**A:** It is a principled classification, not a clamp, and it is *more* honest than a filtered small number. The two-sided taxonomy treats a stationary scene as positive evidence: a pixel is a **zero-witness** when its speed is statistically indistinguishable from zero (|v_i| < 3σ) *and* its noise floor is ≤ 1 m/s. Five or more zero-witnesses force an exact zero-lock. The alternative — a one-sided magnitude gate — is what produces phantom creep, because random-direction tracker noise rectifies into a positive magnitude (an analytical estimate puts this up to ~34%). So "exactly 0.0" is the *correct* reading; a creeping 1–5 km/h would be the bug. And it is genuinely two-sided: real slow motion above the 3σ floor with forward-coherent direction (inside the 45° cone) clears the vote gate and registers — we do not clamp real motion, we reject directionless noise. The data backs it: the displayed speed reads exactly 0.0 km/h throughout the stop, peak ~0.9 km/h.

### H12. The SDD describes a three-tier architecture and a 15-DOF filter; your code is four tiers and the C++ IMU error-state is 19-DOF. Doesn't that mean you didn't follow your own design?
**A:** It means the design was *refined* during implementation, and we are transparent about both deltas. (1) The SDD's three-tier *layered* model (VisionModule / IMUPreintegrator / SensorFusionEngine) is realised as four tiers because the JNI bridge earned first-class-layer status — it carries the zero-copy frame marshaling, the `state_mutex`+`shared_ptr` lifetime contract, and the single Z-up↔Y-up frame conversion. Same design intent, more precise vocabulary (`Tracker`/`IMUPreintegrator`/`EKFState`). (2) The *navigation* state is genuinely the 15-DOF error block `[attitude, gyro bias, velocity, accel bias, position]`; the C++ `IMU_STATE_DIM = 19` adds two *calibration nuisances* — camera-IMU time offset `δt_d` and extrinsic `δφ_bc` (the latter receives only process noise, MSCKF-skipped). We keep "15-DOF" on slides because that is the navigation state that does the work; the extra four are bookkeeping. Both are documented, neither changes the validated behaviour.

### H13. The inertial bridge "trusts" prediction for ~6 seconds. What stops it from drifting catastrophically in a long tunnel or sustained vision blackout?
**A:** Two bounds and a constraint. First, the bridge is *bounded by physics*: a 0.3 m/s² accel-bias integrates to only ~1.8 m/s of error over the ~6 s (138-frame) horizon, then the prediction *decays* rather than free-running — it does not assert false confidence indefinitely. Second, it is *corrected every time vision returns* — once ≥5 votes return we EMA toward the vote-median (α = 0.15 is the post-budget decay rate, not the correction gain), so even brief texture recovery re-anchors it. Third, the **map graph** keeps the *position* on a real road throughout — speed error becomes a bounded along-track error, not a lateral excursion into a building. So a vision blackout degrades to bounded, decaying dead-reckoning on a constrained road — not catastrophic drift.

### H14. Your whole speed result leans on the claim that the GPS reference is the noisy one. If a committee member simply doesn't accept that, what is your fallback evidence?
**A:** Three independent readings that don't depend on accepting the noise-floor argument. (1) **Median agreement**: 36.3 vs 35.1 km/h, within 3.5% — a systematic scale error would shift the median, and it doesn't, so our speed scale is correct regardless of the reference's noise. (2) **First-principles derivation**: the GPS-differenced noise floor σ_v,ref ≈ √2·σ_pos/τ ≈ 5.7 m/s ≈ 20 km/h per 1 s sample is standard error propagation, not our assertion — it's arithmetic on the GPS position uncertainty. (3) **The jamming contrast**: on Route A, jammed GPS inflated distance +33% (1,705 m vs true 1,280 m) while NavSight held within 7% — direct, undeniable evidence that GPS is the corrupted signal in this environment. We scope the speed claim precisely: "noise-limited, no systematic scale bias." We do *not* claim a sub-floor precision we can't prove without a survey-grade reference, and we won't invent one.

### H15. If you started over today, what is the one architectural decision you would defend most strongly, and the one you'd reconsider?
**A:** **Defend most strongly:** making the offline map-matcher the source of truth for position while keeping speed/distance decoupled from it. That single decision converts unbounded VIO drift into bounded along-graph error, makes wrong-road excursions impossible by construction (the ball physically rides the graph), and is what lets a fragile monocular pipeline produce a trustworthy, on-road, jamming-resilient result — all with no network in the navigation hot path. Every strong number traces back to it. **Reconsider:** the depth-model placement. We correctly kept learned metric-depth out of the speed hot path (DA3/V2 at 722 ms vs budget) and lean on the calibrated IPM ground plane, but a learned-inertial velocity model (RNIN-VIO via the existing TFLite seam) would strengthen the inertial bridge in exactly our weakest regime — low-light/turn under-read — and we'd prioritise that earlier. It's already our top future-work item precisely because we now know it's the highest-value lever; the architecture's existing on-device inference seam means it's an addition, not a redesign.

---

## QUICK-REFERENCE FACT SHEET (for rapid recall under pressure)

**Headline results:** Distance 91–93% (Route A 1,195/1,280 m = 93.4%, ρ=0.93; ride 18:02 ~792/869 m = 91%, ρ=0.91) · Speed RMSE 8.9 km/h, bias −1.9 km/h, median 36.3 vs 35.1 km/h (within 3.5%) · Standstill exact 0.0 km/h throughout the stop (~27 frames at exactly 0.0, peak ~0.9 km/h) · Timing median 15.4 ms / max ~78 ms vs 200 ms SDD budget (13× / 2.5×, both vs the budget) · Inertial bridge A/B +24% (62→77 m, stored deterministic offline A/B replay) · Roundabout exit ~9 s, no re-entry.

**Camera/rate:** capture locked 30 fps / 33 ms (CameraUi Range(30,30)); ~23 fps / ~43 ms is the effective processed-VIO rate after `KEEP_ONLY_LATEST` frame-dropping.

**The jamming contrast:** On Route A, jammed GPS over-reported +33% (1,705 m on a true 1,280 m route); on other jammed rides, multi-second GPS position freezes while moving.

**Recovery evidence (2026-06-04 ride):** 36 looming (essential-matrix-degenerate) optical-flow fallbacks · 49 ORB relocalization events.

**Live VIO core (on):** KLT optical flow + forward-backward check + de-rotation · ORB relocalization · MSCKF update · ZUPT/ZRUP/gravity-alignment/stationary-accel · IMU preintegration (Madgwick + Forster midpoint) · IPM ground-plane speed + vote/zero-witness taxonomy + inertial/accel bridge · offline OSM map-matching (LocalMatcher / Viterbi HMM).

**Key constants:** h = 1.05 m · fx ≈ 451 · 640×480 · σpx = 0.5 px · 3σ vote gate · 45° cone (cos θ < −1/√2) · ≥5 votes / ≥5 zero-witnesses · α = 0.15 (post-budget bridge decay rate, not correction gain) · bridge horizon ~6 s (138 frames at the ~23 fps processed rate) · σz = 20 m · β = 6 · b_way = 0.7 · b_rail = 1.2 · candidate radius 30 m · recovery band 25–60 m at conf ≥ 0.55 · live gate: MSCKF Mahalanobis chi-squared (5× χ²₀.₉₅(2K), landmark per-obs 5.991) + per-row Huber δ = 2.4477, Joseph form · MAX_CLONES = 15 · R_bc = diag(1,−1,−1) · GPS ref noise floor ~4–6 km/h @ 5 s.

**Build:** v1.0-osm — compileSdk/targetSdk 34 (Android 14); test device runtime Android 15 / API 35 — never quote a git hash on stage. Displayed base map = Google Maps tiles; OSM = road-matching graph only.

**Tests:** 95-case Kotlin unit suite (95 `@Test` across 14 files) · C++ unit suite · deterministic replay harness · CI scoring on recorded fixtures.
