# Slide 14: Map Matching — Snapping the Position to the Road

**Section:** Deep Dive / Subsystems · **Slide:** 14 of 23 · **Estimated Time:** 2 minutes

## On-Slide Content
- **Problem:** raw VIO drifts; we constrain it to a real road graph using on-device OSM road-matching (no GPS, no network in the navigation hot path).
- **HMM map matcher** (Newson & Krumm 2009), Viterbi-decoded over candidate road segments within 30 m.
- **Emission** (how well a position fits a road): `log p_e = -d_perp² / (2·σz²)`, **σz = 20 m**.
- **Transition** (how plausible a road-to-road move is): `log p_t = -|d_gc - d_route| / β + b_way + b_rail`, **β = 6**, **same-way bonus b_way = 0.7**, **rail bonus b_rail = 1.2**.
- **Graph-rail "ball":** displayed position is snapped onto the OSM road-matching graph — advanced by estimated speed, steered at junctions by the **gyro-relative heading offset**. (The visible base map is Google Maps tiles; OSM is the road-matching graph that snaps the ball.)
- **Bounded recovery:** wrong-fork re-acquire is bounded to **25–60 m at confidence ≥ 0.55** — can never teleport across town.
- **On-device OSM matching:** on-device Viterbi `LocalMatcher` replaced Google Roads / Directions / OSRM; the bundled Haifa OSM assets ship inside the APK — road-matching needs no network.
- [Diagram: diagrams/04-map-matching-hmm.md]
- [Screenshot: tests/sims/val_2026_06_03b/routeA_matched_traj.png — NavSight matched trajectory, colour = time; stays on road and the U-turn retraces the same road]

## Talking Points (what the presenter SAYS)
- "Any dead-reckoning system drifts. Our insight is that a vehicle is almost always on a known road, so we treat the offline road graph as a constraint — not a decoration. The matcher, not the raw trajectory, is the thing we trust for position."
- "We use a Hidden Markov Model from Newson and Krumm's 2009 paper. Each candidate road segment within 30 metres is a hidden state. The **emission** term rewards segments close to our estimate — a Gaussian on perpendicular distance with a 20-metre sigma. The **transition** term rewards road-to-road moves whose graph distance matches how far we actually travelled, with a same-way bonus of 0.7 and a rail bonus of 1.2 that keep us latched to the road we're already on. Viterbi finds the single most likely sequence of roads over the whole ride."
- "What the user sees is a 'ball on the road'. It is snapped to the OSM road-matching graph — it physically cannot leave the network — and that ball is drawn over a Google Maps base map. Speed advances it along the locked way; at a junction the gyro-relative heading offset picks the branch whose tangent best matches where we actually turned."
- "Recovery is bounded by design. If we ever match the wrong fork, we can only re-acquire within a 25-to-60-metre band, and only when confidence is at least 0.55. There is no scenario where the ball teleports across the city — that bound is the safety property."
- "Crucially the navigation is GPS-free and computed on-device. We replaced the Google Roads and OSRM web calls with an on-device matcher and bundled the Haifa OSM graph inside the APK, so road-matching needs no network. The base map is still Google Maps tiles, but under GPS jamming — even with no network — the navigation itself keeps working (the base map just shows cached or blank tiles)."

## Why We Chose This Approach
- **Matcher as truth vs raw trajectory.** Alternative: display the raw integrated VIO path. Tradeoff rejected because VIO accumulates drift, and our V&V showed the trajectory is the *ceiling* on accuracy — so the matcher does double duty: it both displays a road-clean line and absorbs lateral drift. The validated trajectory image shows a U-turn retracing the *same* road rather than smearing across two — direct evidence the constraint is working.
- **HMM/Viterbi vs nearest-road snap.** A naive nearest-road snap is memoryless: at a fork it flips to whichever segment is momentarily closest, causing teleports between parallel roads. The HMM adds the **transition** term, so the chosen road must be both close (emission) AND reachable by the distance we actually moved (transition). Viterbi optimises the whole sequence jointly, so a single noisy frame cannot derail the path. This is the difference between "snap to nearest line" and "decode the most likely route."
- **On-device OSM matching vs Google Roads / OSRM online.** Alternatives considered: Google Roads API, Directions API, OSRM `/match`. Rejected because the deployment environment is GPS-jammed Haifa with unreliable connectivity, plus rate limits, latency, and privacy concerns. Benefit gained: no network latency, no rate limits, jamming-resilient (GPS-denied-capable), and private road-matching. Tradeoff accepted: the APK carries the region's OSM assets and coverage is limited to bundled regions. (The base map remains Google Maps tiles; only the road-matching graph is on-device.)

## Potential Questions (Defense)
**Q:** Isn't map-matching just hiding VIO error — if the road graph were wrong, you'd be lost?
**A:** It constrains rather than hides. The matcher's *along-track* position still comes from our estimated speed, so distance accuracy is independent of the snap — Route A measured 1,195 m against a Google-measured 1,280 m, which is 93.4%, with the ball staying on-road the whole time. The graph removes *lateral* drift, which is exactly the drift VIO is worst at. And we bound any mismatch to a 25–60 m re-acquire at confidence ≥ 0.55, so a graph error degrades gracefully rather than catastrophically.
**Follow-up Q:** Why a 20-metre emission sigma and not tighter?
**Follow-up A:** σz = 20 m is the Newson-Krumm working value and it has to cover the combined budget of VIO lateral drift plus road-centerline-versus-lane offset. A tighter sigma would over-trust an estimate that genuinely has tens of metres of lateral uncertainty and would cause the matcher to thrash between adjacent candidates; 20 m keeps the emission soft enough that the transition term decides at forks.

**Q:** How does the ball pick the correct road at a junction without GPS?
**A:** By the gyro-relative heading offset. The ball advances by speed along the locked way; at a junction we compare each outgoing branch's tangent against our heading change since the anchor, `θ_anchor + Δψ_gyro`, and take the branch with the smallest angular error. The gyro is reliable for the few seconds a junction takes, so the turn we physically made selects the branch — no GPS fix needed.
**Follow-up Q:** What stops it from getting stuck circling a roundabout?
**Follow-up A:** A ring-eject mechanism. On the validation ride we started the app *on* a roundabout, detected it, and exited cleanly at roughly t≈9 s (observed) with no spurious re-entry. The exit re-acquires the ball onto the suggested exit segment with re-entry hysteresis (RING EJECT after ~460 ticks, confidence 0.80), so a circling trace can't keep matching its own ring indefinitely.

## Speaker Notes
- Numbers to have ready: σz = 20 m, β = 6, b_way = 0.7, b_rail = 1.2, candidate radius 30 m, recovery band 25–60 m, confidence gate ≥ 0.55, bundled Haifa OSM assets (do not assert a specific MB figure).
- Emphasise the engineering reframe: "the matcher is the source of truth for *position*; speed is the source of truth for *distance*." They are decoupled, which is why distance accuracy survives even when the snap is doing heavy lifting.
- If asked about the same-way / rail bonus: these are additive log-prior terms that bias the decode toward staying on the road already locked (`lockedWayId`), which kills parallel-road teleport — the original failure mode that motivated the rail-follower design.
- Pitfall to avoid: do NOT claim the matcher *corrects* along-track distance — it does not; distance comes from the speed estimator (Slide 15). Be precise that map-matching fixes lateral/topological error.
- The screenshot is the strongest visual proof — point at the U-turn retracing the same road, then at colour=time to show temporal consistency. Real GNSS-jamming context: GPS on this region inflated path by +33% (1,705 m on a true 1,280 m route), so map-measured distance — not GPS — is the ground truth here.
