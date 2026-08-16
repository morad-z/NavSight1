# Slide 10: Technical Decisions

**Section:** Architecture & Engineering · **Slide:** 10 of 23 · **Estimated Time:** 1.5 minutes (deck total ~30 minutes)

## On-Slide Content
- **Decision table — Decision · Alternative rejected · Why · Tradeoff accepted:**
- **Monocular camera + IMU** — vs stereo / LIDAR — runs on any commodity phone, GPS-free navigation computed on-device, no extra hardware — scale ambiguity, solved by IMU + map-matcher.
- **Map-matcher = source of truth** — vs raw VIO trajectory — graph constraint bounds drift; wrong-fork recovery bounded **25–60 m** at conf ≥ **0.55**, never teleports across town — ceiling = bundled OSM coverage (Haifa).
- **Two-sided zero-witness taxonomy** — vs one-sided magnitude gate — standstill reads **exact 0**; a one-sided gate rectifies **~34%** of noise into positive speed — needs ≥5 votes or ≥5 zero-witnesses to resolve.
- **Complementary inertial bridge** — vs pure visual speed — carries blur/starved frames; **+24%** integrated distance in offline A/B replay (62 → 77 m) — trusted ≤ ~6 s then decays at 0.15.
- [Diagram: diagrams/01-system-architecture.md — the 4-tier stack these decisions shape]

## Talking Points (what the presenter SAYS)
- "Four decisions define this system. Each one is a deliberate tradeoff, and I can defend every one with a measured number rather than a preference."
- "We chose a single monocular camera plus the phone's IMU — no stereo rig, no LIDAR. That gives us an app that runs on any commodity Android phone, with GPS-free navigation computed entirely on-device. The price is monocular scale ambiguity, which we resolve with the IMU and the map graph."
- "The most important architectural decision: the *map-matcher is the source of truth for position*, not the raw VIO trajectory. The ball is snapped to the OpenStreetMap road-matching graph, so it can never drift into a building — even though the base map you see is Google Maps tiles. Wrong-fork recovery is bounded by design to 25–60 m at confidence above 0.55 — it can correct a fork mistake but can never teleport across town."
- "Two smaller decisions punch above their weight: the two-sided zero-witness taxonomy, which is the only reason standstill reads an exact zero; and the inertial bridge, which buys +24% integrated distance through blur and feature-starved frames in a stored deterministic offline A/B replay."

## Why We Chose This Approach
- **Monocular vs stereo/LIDAR.** Stereo or depth sensors resolve scale directly but require hardware no commodity phone carries and break the "any Android phone, GPS-free on-device navigation" goal. We accepted monocular scale ambiguity and resolved it two ways: IMU-derived scale (s = d_imu / d_vis, EMA) plus the calibrated ground plane (camera height h = 1.05 m) for the speed-path metric depth, and ultimately the map graph constraining absolute position. Benefit: zero extra hardware, navigation private and computed on-device.
- **Map-matcher as source of truth vs raw VIO.** Raw VIO drifts unbounded over distance; even a good filter accumulates along-track error. Snapping to the OSM road-matching graph (HMM emission σ_z = 20 m, transition β = 6, rail bonus 1.2) converts unbounded drift into bounded along-graph error and makes wrong-road excursions impossible by construction. The displayed base map is Google Maps tiles; OSM is only the road-matching graph that snaps the ball. Tradeoff: position quality is capped by both VIO trajectory quality and bundled OSM coverage; benefit: drift control with no network in the navigation hot path, no rate limits, GPS-denied-capable / jamming-resilient.
- **Two-sided zero-witness taxonomy vs one-sided gate.** A simple magnitude gate is one line of code but structurally rectifies ~34% of random-direction tracker noise into a positive speed, making a true 0 km/h impossible. The two-sided design treats a stationary scene as positive evidence (zero-witnesses), so ≥5 of them force an exact zero-lock — the correct behaviour, not a clamp.
- **Inertial bridge vs pure visual speed.** Pure visual speed drops out whenever blur or feature starvation kills the vote count. The complementary bridge predicts v⁻ = v + a_fwd·Δt every frame, then corrects by EMA toward the vote-median once ≥5 votes return (hard-0 on ≥5 zero-witnesses); past the ~6 s budget it decays at α = 0.15 (kGpBridgeDecayAlpha). The A/B gain is a stored deterministic offline replay result on identical input: +24% integrated distance.

## Potential Questions (Defense)
**Q:** Making the map-matcher the source of truth feels like a crutch — aren't you just hiding poor VIO behind the road graph?
**A:** No — the matcher constrains position but cannot invent motion. The ball advances along the locked way *by the VIO-estimated speed* (s_{k+1} = s_k + v_k·Δt) and is steered at junctions by the gyro-relative heading offset. If the VIO speed is wrong, the matched distance is wrong — which is exactly what we report: Route A measured 1,280 m on Google Maps, NavSight produced 1,195 m (93.4%), and ride 18:02 produced 91% (ρ = 0.91). The matcher bounds *lateral* drift to the road; it does not fabricate the *along-track* distance, so it cannot mask a poor speed estimate.

**Follow-up Q:** What is the failure mode if the matcher locks onto the wrong road?
**Follow-up A:** It is bounded by design. Wrong-fork recovery is constrained to 25–60 m and only acts at confidence ≥ 0.55, with a measured split between bad yanks (0.40–0.42) and healthy matches (0.63–0.86). It can correct a fork mistake within a block but can never teleport across town — the graph constraint guarantees the position is always on a real road, and the distance band caps how far a correction can jump.

**Q:** Is this "really a VIO system" — what actually runs on each frame?
**A:** It is a tightly-coupled VIO system: a 15-DOF error-state EKF (the full IMU error-state is 19-DOF, adding the camera–IMU time offset and the body→camera extrinsic) fusing camera and IMU with an MSCKF Mahalanobis chi-squared gate plus per-row Huber and a Joseph-form covariance update. The visual front-end is KLT optical flow with a forward–backward check and de-rotation, plus ORB relocalization. On the 2026-06-04 ride we observed 36 looming (essential-matrix-degenerate) optical-flow fallbacks and 49 ORB relocalization events — all live, all verified used. Speed comes from the IPM ground-plane estimate with the vote/zero-witness taxonomy and the inertial/accel bridge, and position is bounded by the offline OSM map-matcher (LocalMatcher / Viterbi HMM).

**Follow-up Q:** How was the build validated before this defense?
**Follow-up A:** The build is v1.0-osm. It carries a 95-case Kotlin unit suite (95 @Test across 14 files), and the real-walk validation shows the headline results: Route A distance 1,195 m vs true 1,280 m (93.4%), ride 18:02 at 91%, exact-0 standstill, and per-frame tracking median 15.4 ms against the 200 ms SDD budget. For the GPS-denied outdoor target, that is the relevant evidence.

## Speaker Notes
- Lead with the map-matcher decision — it is the single most architecturally consequential choice and the strongest answer to "why won't it drift into a building."
- Numbers to keep loaded: HMM params σ_z = 20 m, β = 6, rail bonus 1.2, same-way bonus 0.7 (the 0.7 appears in the report figure caption, not the canonical algorithm line — only cite it if asked); recovery band 25–60 m at conf ≥ 0.55; zero-witness one-sided-gate rectification ~34%; inertial bridge decay α = 0.15, trusted ≤ ~6 s (138 frames at the ~23 fps processed VIO rate), +24% stored offline A/B (62 → 77 m).
- Scale-resolution detail if pressed: s = d_imu / d_vis with EMA; the speed-path metric depth comes from the calibrated ground plane (h = 1.05 m), not learned depth; the map graph provides the final absolute-position anchor.
- **Pitfalls to avoid:** do not claim the matcher improves speed accuracy — it bounds lateral drift only. Do not call the Google Maps base map "OSM"; OSM is only the road-matching graph that snaps the ball. Do not quote a git hash; refer to the build as "v1.0-osm."
- Tie-back line for the next section (Runtime/Frame Lifecycle slides 11–16): "Every one of these decisions shows up concretely in the per-frame pipeline you're about to see."
