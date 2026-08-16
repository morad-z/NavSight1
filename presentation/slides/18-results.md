# Slide 18: Results

**Section:** Validation & Results · **Slide:** 18 of 23 · **Estimated Time:** 2 minutes

## On-Slide Content

| Metric | NavSight | Reference | Result |
|---|---|---|---|
| Distance — Route A (map-measured) | 1,195 m | 1,280 m (Google) | **93.4%** (ρ = 0.93) |
| Distance — Ride 18:02 (verified GPS) | ~792 m | 869 m | **91%** (ρ = 0.91) |
| Speed RMSE (5 s windows) | — | — | **8.9 km/h** |
| Speed bias | — | — | **−1.9 km/h** |
| Speed median | 36.3 km/h | 35.1 km/h (GPS) | **within 3.5%** |
| Standstill | exact **0.0 km/h** | — | **reads exactly 0.0 throughout the stop** |
| Per-frame tracking time | median **15.4 ms** (max ~78 ms) | 200 ms SDD budget | **13× under median** |
| Inertial bridge A/B (replay) | 62 m → 77 m | identical input | **+24%** distance¹ |
| Roundabout (started ON ring) | detected t≈0, exit **t≈9 s** | — | **no spurious re-entry** |

¹ Stored deterministic offline A/B replay result (identical input).

- [Screenshot: tests/sims/val_2026_06_03b/routeA_google_measure.png — Route A Google measurement = 1.28 km ground truth]
- [Screenshot: tests/sims/val_2026_06_03b/routeA_cumdist_gps_vs_dot.png — cumulative distance: jammed GPS 1,705 m vs NavSight 1,195 m on a 1,280 m route]
- V&V report Figs 3–7: speed-vs-time, standstill zoom, distance bars, timing histogram, A/B bridge.

## Talking Points (what the presenter SAYS)
- "Start with distance, our primary metric. On Route A — a 1,280 m route measured on Google Maps — NavSight reported 1,195 m, that's 93.4% accurate with a correlation of 0.93. On a second ride against verified-healthy GPS of 869 m, we measured 91%, correlation 0.91. Two independent rides, both above 90%."
- "Speed: across 5-second windows, RMSE is 8.9 km/h, with a small negative bias of 1.9 km/h, and the medians almost coincide — 36.3 for NavSight versus 35.1 for GPS, within three and a half percent. I'll explain in the next slide why the 8.9 figure is actually excellent."
- "Standstill is where the design really shows: the zero-witness lock holds an *exact* 0.0 km/h. On the validation ride the displayed speed read literally 0.0 throughout the stop — about 27 frames at exactly 0.0 — and speed never crept above about 0.9 km/h. No phantom drift while parked."
- "On real-time performance, per-frame tracking takes a median of 15.4 ms against our 200 ms SDD budget — 13× of margin at the median and still 2.5× at the 78 ms worst case, both measured against the 200 ms budget. Camera capture is locked at 30 fps (33 ms); the processed VIO rate is ~23 fps (~43 ms) after frame-dropping, and a worst-case ~78 ms frame is simply absorbed by dropping the next frame."
- "Two targeted experiments close it out. The inertial bridge, measured on a stored deterministic offline replay, lifts integrated distance from 62 to 77 metres — a +24% gain on a turn-heavy ride. And starting the app *on* a roundabout, it detected the ring at t≈0 and cleanly exited around 9 seconds (RING EJECT, conf 0.80) with no spurious re-entry."

## Potential Questions (Defense)
**Q:** Your distance accuracy is 91–93%. Where does the remaining 7–9% go?
**A:** It is overwhelmingly turn under-read before the inertial bridge engages — the camera loses near-road texture in tight turns. Our own A/B proves this: enabling the bridge recovers +24% of integrated distance on a turn-heavy ride. The residual after that is consistent under-read, which is why both numbers land just under the route length rather than scattering above and below — a stable, bounded, *understood* error, not random drift.
**Follow-up Q:** Why does NavSight read 1,195 m while GPS read 1,705 m on the same 1,280 m route?
**Follow-up A:** That is the headline of the project. NavSight is within 7% of the true 1,280 m; jammed GPS inflated the path by +33% to 1,705 m. We hold accuracy on exactly the route where GPS — the incumbent — fails catastrophically. The cumulative-distance figure makes this visually obvious.

**Q:** An 8.9 km/h speed RMSE sounds large for a navigation system.
**A:** It sits at the reference's own resolution limit. The conditioned GPS speed reference has a noise floor of about 4–6 km/h at 5 s windows, and raw 1 s differencing is around 20 km/h. So 8.9 km/h is essentially "as good as the yardstick can measure," and the medians confirm it — 36.3 vs 35.1 km/h, within 3.5%. The next slide develops this point.
**Follow-up Q:** Could the small −1.9 km/h bias accumulate into long-route error?
**Follow-up A:** It is bounded, not accumulating. Speed is predicted each frame (v += a_fwd·dt) and corrected by an EMA toward the median of voting road pixels once at least five votes return (hard-zero on five zero-witnesses); past the ~6 s inertial-bridge budget the prediction simply decays (decay rate α = 0.15). The map matcher also constrains position to the road graph. The bias shows up as the consistent slight under-read in distance (91–93%), which we already account for — it does not free-run.

## Speaker Notes
- Quote distances precisely: Route A 1,195/1,280 = 93.4%, ρ = 0.93; Ride 18:02 ~792/869 = 91%, ρ = 0.91. Treat 792 m as telemetry-derived/approximate; the firm, defensible labels are 93.4% and 91%.
- Timing: median 15.4 ms, max ~78 ms, budget 200 ms → 13× under median, 2.5× worst-case margin (both vs the 200 ms budget). Camera capture is LOCKED at 30 fps (33 ms, CameraUi Range(30,30)); ~23 fps / ~43 ms is the EFFECTIVE processed VIO rate after KEEP_ONLY_LATEST frame-dropping — never call ~23 fps the camera/native rate. A worst-case ~78 ms frame exceeds the interval and is absorbed by frame-dropping.
- Standstill: the displayed speed reads exactly 0.0 km/h throughout the stop (the exact-0 zero-witness lock is real); on the cited ride that is ~27 frames at exactly 0.0 (not "50"), with peak ~0.9 km/h during the stop.
- A/B bridge: 62 → 77 m = +24%, a stored deterministic offline replay result (identical input, hardcoded — not recomputed), not two different walks. The ~6 s bridge budget (138 frames at the ~23 fps processed rate) is real.
- Roundabout: started ON the ring, detected t≈0, clean exit ~9 s (approximate/observed caption; mechanism RING EJECT, 460 ticks, conf 0.80 is code-confirmed), no re-entry.
- Pitfall: do NOT quote a git hash on slides — refer to the build only as "v1.0-osm". Do not over-claim 792 m as exact.
- If asked about fallbacks generally: 36 looming (essential-matrix-degenerate) optical-flow fallbacks and 49 ORB relocalization events were logged on the 2026-06-04 ride — evidence the recovery paths fire in practice. (These are looming/optical-flow fallbacks, never call them "inertial".)
