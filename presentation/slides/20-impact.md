# Slide 20: Impact — Why This Matters

**Section:** Impact & Closing · **Slide:** 20 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- Resilient navigation **without a live GPS fix** — works under the real GNSS jamming we measured in Haifa.
- **GPS-free navigation computed on-device & private**: rear camera + IMU + on-device OpenStreetMap road-matching (LocalMatcher / Viterbi HMM) — no GPS and no network in the navigation hot path; base map uses Google Maps tiles.
- Runs on a **commodity phone** (Samsung S21 Ultra, Exynos 2100 / Mali-G78) — no stereo rig, no LIDAR, no extra hardware.
- Proven accuracy: **91–93% of true distance**, speed **within 3.5%** of a clean GPS reference, **13× real-time headroom**.
- Solves a concrete failure mode: on Route A jammed GPS inflated path length by **+33%** (1,705 m vs true 1,280 m) — NavSight stayed on the road at 1,195 m.
- [Screenshot: tests/sims/val_2026_06_12/ui3.png — live "ball snapped to the road" at 0 km/h, green VIO chip, GPS-free navigation]

## Talking Points (what the presenter SAYS)
- The practical problem we solve is simple to state: when GPS lies or disappears, you still need to know where you are and how fast you are going. In Haifa under wartime jamming this is not hypothetical — on Route A jammed GPS inflated a 1,280 m route to 1,705 m, and on other jammed rides we observed multi-second GPS position freezes while moving.
- NavSight delivers a live position and speed using only the rear camera, the IMU, and on-device OSM road-matching — no GPS fix required, and the navigation computation stays on the device.
- Because it needs no special sensors and no connectivity in the navigation path, the value generalizes: anywhere GPS is denied, jammed, or simply unavailable — tunnels, dense urban canyons, indoors, contested regions — a normal Android phone becomes a navigation aid.
- The research value is equally concrete: we built a GPS-free, deterministic-replay VIO pipeline that is validated against jamming-resilient, map-measured ground truth rather than the GPS that the environment was actively corrupting.

## Potential Questions (Defense)
**Q:** Who actually benefits from this, and in what real situation?
**A:** Anyone moving through a GPS-denied environment with only a phone — commuters and riders in jammed regions like wartime Haifa, indoor or tunnel navigation, and privacy-sensitive users who do not want their location streamed to a cloud service. We demonstrated the core case on a scooter under real regional GNSS jamming and held 91–93% distance accuracy where GPS itself was wrong by +33% on Route A.

**Follow-up Q:** Is this just a research demo, or could it ship?
**Follow-up A:** It runs in real time on a stock S21 Ultra with 13× median headroom under the 200 ms frame budget, the bundled Haifa OSM assets ship inside the APK, and the navigation path needs no network — those are shipping-grade constraints, not lab constraints. The honest limitation is region coverage (we ship Haifa) and low-light speed, which are on the future-work slide.

**Q:** GPS already works almost everywhere — why invest here?
**A:** "Almost everywhere" is exactly the gap. Our own measurements show GPS failing precisely when it is needed: a +33% path-length inflation under jamming on Route A, and multi-second GPS position freezes while moving on other jammed rides. NavSight is designed for the cases where GPS is the unreliable signal, and it treats jamming-resilient map-measured distance — not GPS — as the truth.

## Speaker Notes
- Lead with the user benefit, not the algorithm — this slide is about value, not math.
- Have the three headline numbers ready in one breath: 91–93% distance, speed within 3.5% (RMSE 8.9 km/h sitting at the GPS reference's own 4–6 km/h noise floor), 13× real-time headroom.
- The strongest single proof point is the GPS-vs-truth contrast: on Route A GPS reported 1,705 m on a route that truly measures 1,280 m; NavSight reported 1,195 m (within 7% of truth). Use this to pre-empt "why not just trust GPS."
- Emphasize "GPS-free + commodity + private" together — that triple is what makes the impact broad rather than niche.
