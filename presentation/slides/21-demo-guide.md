# Slide 21: Live Demo Guide — Route A Drive on the Offline OSM Road Graph

**Section:** Impact & Closing · **Slide:** 21 of 23 · **Estimated Time:** 2 minutes

## On-Slide Content
- **Strongest workflow:** start the app with **airplane mode / no GPS fix**, drive Route A, watch the **ball stay snapped to the road via on-device OSM road-matching** (base map = Google Maps tiles) with a live speed readout.
- **What to watch:** ball never leaves the road network · U-turn retraces the **same** road · speed tracks the ride · exact **0 km/h** at a stop (zero-witness lock).
- **Expected outputs:** live map (ui3.png-style), live camera overlay (amber mask + green flow), green VIO chip = healthy.
- **Headline to call out:** Route A = Google-measured **1,280 m** → NavSight **1,195 m (93.4%)**, GPS would have said 1,705 m.
- **Safe fallback if live fails:** play the recorded Route A ride / show the two stills below.
- [Screenshot: tests/sims/val_2026_06_12/ui3.png — live map (Google Maps base), 0 km/h, ball snapped by on-device OSM road-matching]
- [Screenshot: tests/sims/val_2026_06_03b/routeA_matched_traj.png — matched trajectory, U-turn retraces same road]

## Talking Points (what the presenter SAYS)
- The one thing to demonstrate is the whole thesis in motion: with no GPS fix, the dot stays on the road and the speed is right — navigation computed on-device (camera + IMU + on-device OSM road-matching). Everything else is supporting evidence for this.
- Required setup is deliberately minimal — a charged S21 Ultra, the v1.0-osm build with the bundled Haifa OSM assets, airplane mode on to prove the navigation hot path needs no network (the Google Maps base map will fall back to cached/blank tiles), camera and motion permissions granted, and the phone mounted with a clear forward view of the road.
- As we drive Route A, watch three behaviors: the ball is graph-constrained so it physically cannot drift off into a building; when we U-turn it retraces the same road rather than jumping to a parallel street; and when we stop, the speed reads exactly 0 km/h, not a creeping 1–2.
- If the live ride is not possible in the room, we fall back to the recorded Route A: the matched-trajectory image shows the dot staying on the road through the U-turn, and the live UI still shows the road-matched ball and the green VIO chip — same evidence, deterministic playback.

## Why We Chose This Approach
- **Alternatives considered:** demoing speed estimation alone, the camera optical-flow overlay alone, or the standstill zero-lock alone. Each shows one subsystem; none shows the integrated claim.
- **Tradeoffs accepted:** a live ride is the highest-risk demo (jamming, mounting, room logistics), so we pair it with a deterministic recorded fallback rather than a different, weaker live demo.
- **Benefits gained:** Route A is our strongest validated case (93.4% distance, U-turn retrace, on-road constraint, all on one route) and it visually carries the entire value proposition — offline, on-road, correct speed — in under a minute.
- **Engineering reasoning:** the recorded fallback uses the same unmodified engine via deterministic offline replay, so the fallback is not a mock — it is the identical pipeline on identical input.

## Potential Questions (Defense)
**Q:** How do we know the live demo isn't quietly using GPS?
**A:** We run it in airplane mode with no fix, and the VIO chip plus debug panel show the position is map-matched from the camera+IMU trajectory, not from a location provider. The whole point of v1.0-osm is that the navigation hot path is GPS-free — the position is computed on-device from camera + IMU + the bundled Haifa OSM road graph, with no GPS and no network in the navigation path. (The base map underneath is Google Maps tiles, which simply fall back to cached/blank tiles when offline; the snapped ball does not depend on them.)

**Follow-up Q:** What if the live ride drifts badly in front of the committee?
**Follow-up A:** That is exactly why we carry the recorded Route A fallback — same engine via deterministic replay, validated to 93.4% distance with the U-turn retrace. We show routeA_matched_traj.png and ui3.png and narrate the identical behaviors, so the evidence is unchanged.

**Q:** Why Route A specifically and not the 18:02 ride?
**A:** Route A is our best-documented case: Google-measured ground truth of 1,280 m, NavSight 1,195 m at 93.4%, a clean U-turn that retraces the same road, and a full Fig 9–11 case study. It demonstrates the on-road constraint and distance accuracy together on a single route, which is the most convincing single artifact we have.

## Speaker Notes
- Pre-flight checklist to state confidently: charged device, v1.0-osm installed, Haifa assets present, airplane mode ON, camera + fine-location permissions granted, phone mounted with clear forward road view, screen mirroring tested.
- Keep the narration to the three watch-points (on-road, U-turn retrace, exact-zero stop) — do not narrate internals during the ride.
- If asked for numbers mid-demo: Route A 1,280 m truth → 1,195 m NavSight (93.4%); GPS would have inflated it to 1,705 m.
- Fallback trigger discipline: if the ball misbehaves for more than a few seconds, cut to the recorded ride immediately rather than debugging live. The recorded path is the same engine on the same frames.
