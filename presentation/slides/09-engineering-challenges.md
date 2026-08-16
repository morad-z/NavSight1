# Slide 09: Engineering Challenges

**Section:** Architecture & Engineering · **Slide:** 9 of 23 · **Estimated Time:** 2 minutes

> Renders as two Canva slides (9a: the navigation/perception challenges 1–4; 9b: the systems/discipline challenges 5–7).

## On-Slide Content
- **7 root-caused challenges** — every one fixed at the *cause*, not patched at the symptom.
- **(1) IPM speed under-read + standstill creep** → two-sided vote taxonomy + accel bridge → zero-lock holds **exact 0**, cruise IPM/GPS ≈ **0.91**.
- **(2) Map-matcher teleport ("into the woods", 77–242 m)** → re-anchor only at real junctions + distance-banded supervision → teleports eliminated.
- **(3) Roundabout orbit (460 ticks ≈ 4 min) + backwards start** → RING EJECT + deferred bearing acquire → clean exit at **~9 s** (observed), no re-entry.
- **(4) Per-mode scale K under-convergence** → physics-derived gait detector, per-mode K slots → built green, on-device installed.
- **(5) The gravity-drift debugging lesson** → a ~6–10° tilt mis-cancelled gravity (≈800 m phantom Z drift); found by reading residual data, fixed by the gravity-alignment update. Lesson: **read the data before tuning constants.**
- **(6) Real-time budget on Mali-G78** → calibrated ground-plane (h=1.05 m) metric speed path → median **15.4 ms**/frame vs **200 ms** budget.
- **(7) GPS jamming defeats GPS as ground truth** → on Route A jammed GPS over-reported **+33%** (1,705 m vs true 1,280 m) → map-measured routes are ground truth; VIO scale uncorrupted.
- [Diagram: diagrams/06-ekf-pipeline.md — where the MSCKF gate and gravity-alignment update sit]
- [Screenshot: tests/sims/val_2026_06_03b/probe_cruise_0.jpg — amber sampling mask, green verified flow, red production KLT]

## Talking Points (what the presenter SAYS)
- "I want to be honest about how this system was actually built — not the clean version, the real version. Every number on this slide started as a bug we could see in the data."
- "Take the IPM speedometer: it saturated at 13–19 km/h on a scooter and crept while standing still. The root cause was three measured faults — motion blur killing the near band, lane lines running parallel to the flow, and feature starvation on 77% of cruise frames. We fixed it with a two-sided vote taxonomy plus an inertial accel-bridge, and standstill now reads an exact zero."
- "The map-matcher once 'took us into the woods' — 77 to 242 metres onto forest paths. The cause was a per-vertex re-anchor firing at every 2–7° curve step, which zeroed the user's turn. We made re-anchoring fire only at real junctions and gated supervision behind a 25–60 m distance band and confidence ≥ 0.55."
- "My favourite lesson is the gravity-drift debugging story. We were chasing a runaway position error, and the temptation was to tune constants for days. The real cause was a ~6–10° tilt mis-cancelling gravity, which integrated into roughly 800 metres of phantom Z drift. One run of an analysis script on the residual data showed it immediately, and the fix belonged at the gravity-alignment update, not in any constant. That became our cardinal rule: read the data before you tune a constant."
- "We kept only what the counters proved earns its cost: KLT optical flow with a forward-backward check, ORB relocalization, MSCKF covariance updates, IMU preintegration, ZUPT and gravity alignment, the IPM ground-plane speed path, and the offline OSM map-matcher — each verified actually used on the validation rides."

## Why We Chose This Approach
- **Root-cause over symptom-patch (team discipline).** Alternatives we rejected at each turn: a flow cap to hide IPM over-reads, a clamp on standstill creep, a hard-coded heading on backwards starts. Every one of these is a defensive patch that masks a fault while corrupting the state elsewhere. We chose to instrument each state transition (LOGI + counters), reproduce on deterministic replay, then fix the layer where the fault originates.
- **Two-sided vote taxonomy vs one-sided magnitude gate (challenge 1).** A one-sided magnitude gate rectifies ~34% (analytical estimate) of random-direction tracker noise into a *positive* speed, so standstill could never read true zero. The VOTE / ZERO-WITNESS / reject taxonomy is the only design where a stationary scene produces an exact 0 km/h — zeros are evidence, not noise to be discarded.
- **Junction-only re-anchor vs per-vertex re-anchor (challenge 2).** Re-anchoring at every curve vertex is computationally cheap but semantically wrong — it discards real rotation. Re-anchoring only where there are ≥2 eligible continuations, plus a quiet-straight re-sync to the *external* matched tangent, preserves the user's turn and still corrects slow curve drift.
- **Calibrated ground plane vs learned metric depth (challenge 6).** For metric Z in the speed path we chose plane + mount-height geometry (camera height h = 1.05 m), which is deterministic and cheap, over learned depth. Benefit: real-time on the actual Mali-G78 hardware, with the speed-path metric depth coming from the calibrated ground plane rather than a network in the hot loop.

## Potential Questions (Defense)
**Q:** You chased a large position error for a while before reading the data. Isn't that a process failure you should hide, not present?
**A:** We present it deliberately because it produced our most valuable engineering rule. The runaway error came from an upstream gravity mis-cancellation — a ~6–10° tilt integrating into roughly 800 m of phantom Z drift over the run. The fix belonged at the gravity-alignment layer (the residual band g ± 0.8 m/s² is illustrative), not in any tuned constant. One run of our residual-analysis script showed the residuals were position-dominated and physically impossible, ending the issue quickly. The takeaway — "read the data before tuning constants" — is now a binding team convention.

**Follow-up Q:** How do you guarantee this never recurs?
**Follow-up A:** Two safeguards. First, every state transition and convention boundary now emits LOGI plus event counters, so the data needed to diagnose is captured by default rather than reconstructed under pressure. Second, the gravity-alignment update keeps the position physically bounded by observing 2-DOF roll/pitch from the accelerometer direction; the tilt that caused the 800 m drift can no longer accumulate unobserved.

**Q:** What runs in the shipped v1.0-osm pipeline, and how do you know each piece earns its cost?
**A:** The live pipeline is KLT optical flow with a forward-backward check and de-rotation, ORB relocalization, MSCKF covariance updates, IMU preintegration (Madgwick + Forster midpoint), ZUPT / ZRUP / gravity-alignment / stationary-accel updates, the IPM ground-plane speed path with its vote / zero-witness taxonomy and inertial accel bridge, and the offline OSM map-matcher (LocalMatcher / Viterbi HMM). On the 2026-06-04 ride the counters logged 36 looming (essential-matrix-degenerate) fallbacks and 49 ORB relocalization events, confirming the recovery path fires and matters. We present only what runs in v1.0-osm.

## Speaker Notes
- Concrete numbers to have ready per challenge: IPM gradient energy 10.2 → 3.5 when moving; standstill far-point noise 0.57 px; 77% of cruise frames starved the ≥5-vote gate; gp/gps ≈ 0.92 when locked; offline replay standstill 11/12 pairs zero-lock.
- Map-matcher specifics: 49/49 curve steps were 2–7° (i.e. genuine curves, not turns); measured supervision confidence split — bad yanks 0.40–0.42 vs healthy 0.63–0.86; recovery band 25–60 m at confidence ≥ 0.55.
- Roundabout specifics: ride3 logged 460 consecutive ON_ROUNDABOUT ticks (~4 min); matcher confidence stayed 0.80 because a circling trace matches its own ring; backwards start = seed bearing falling back to 0.0 (NORTH) on E–W streets; `maybeFixBackwardsHeading` one-shot re-snap near ±180°. The ~9 s exit time is an observed caption value, not a recomputed metric.
- Per-mode K: walk ~1340, run ~831, u-turn ~1652; walk-stop K spike 4845 bled into shared K 2400; run seeds at walk × 0.62; scooter K stays honest-hard (−1) until a stop/brake calibrates it.
- Build: shipped as v1.0-osm (do NOT quote a git hash on the slide); 95-case Kotlin unit suite (95 @Test across 14 files).
- **Pitfalls to avoid on stage:** do not over-claim the gravity-drift story as a heroic save — frame it as the lesson it taught. Do not quote a git hash; refer to the build as "v1.0-osm." Do not call the displayed base map "OSM" — the base map is Google Maps tiles; OSM is only the road-matching graph that snaps the ball.
- If pressed for time, present 9a (challenges 1–4) in full and compress 9b to the gravity-drift lesson + the GPS-jamming ground-truth contrast, since those two carry the strongest engineering narrative.
