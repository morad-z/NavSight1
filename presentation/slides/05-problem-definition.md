# Slide 05: Problem Definition — Position & Speed Without a Fix

**Section:** Problem & Context · **Slide:** 5 of 23 · **Estimated Time:** 1.5 minutes

## On-Slide Content
- **The problem, stated precisely:** track **position and speed relative to a known start point**, with **no GPS fix**, **in real time on a commodity phone** — accurate enough to drive a map-matched "ball on the road" to the right intersection at the right moment.
- Inputs: rear camera + IMU (gyro/accel) + an **offline** OpenStreetMap road graph used for road-matching. No GPS, no network in the navigation hot path.
- Hard constraints: real-time per-frame budget (SDD **200 ms**), runs on a standard Android phone; **GPS-free navigation computed on-device** (offline OSM road-matching) — the base map uses Google Maps tiles.
- Success criterion: the displayed ball stays **on the road**, advances at the **right speed**, and arrives at junctions **at the right time** — so the user can act on it like a normal map.
- **In scope:** relative position, live speed, road-snapped trajectory, standstill detection, GPS-free on-device navigation.
- **Out of scope (explicit non-goals):** turn-by-turn voice guidance, full 3D SLAM/reconstruction, cloud processing, external/online map services, absolute geodetic re-localization without a start anchor.
- [Diagram: diagrams/01-system-architecture.md] · [Screenshot: tests/sims/val_2026_06_12/ui3.png — live map: 0 km/h at standstill, green VIO chip, ball snapped to the OSM road-matching graph over the Google Maps base map]

## Talking Points (what the presenter SAYS)
- "Let us define the problem sharply, because the scope is what makes it tractable. We are not rebuilding GPS. We are tracking position and speed *relative to where you started* — and we must do it in real time, on a phone, with no live fix."
- "Our navigation inputs are the rear camera, the inertial sensors, and an offline OpenStreetMap road-matching graph bundled in the app. The navigation runs GPS-free and entirely on-device — no GPS lock, no network in the navigation path. The base map itself is rendered with Google Maps tiles."
- "The accuracy bar is defined by the user experience, not by an abstract metre count: the position is shown as a ball constrained to the road network. Success means that ball stays on the correct road, moves at the correct speed, and reaches each intersection at the correct time — so you can navigate by it exactly like a normal map."
- "There are firm constraints. Each processed frame's visual work must complete inside the SDD's 200 ms budget — the camera captures at a locked 30 fps and excess frames are dropped to keep up — and the navigation must run GPS-free, on-device, on a standard Samsung Galaxy S21 Ultra."
- "Equally important is what we deliberately left out: no turn-by-turn voice guidance, no full 3D SLAM, no cloud, no online map services. Those are real features, but they are out of scope for this project — naming them keeps our claims honest."

## Why We Chose This Approach
- **Alternatives considered:** (1) solve for *absolute* global position (full geodetic localization) — rejected because it requires either a live fix or a pre-built global map, neither available in a GPS-denied, offline setting; (2) full 3D SLAM with dense reconstruction — rejected as far beyond a real-time phone budget and beyond what a road-following app needs; (3) free-running visual-inertial dead reckoning with no map — rejected because unbounded drift eventually walks the ball off the road.
- **Tradeoffs accepted:** by defining the target as *relative* position anchored to a start point and *snapped to a known road graph*, we trade absolute global accuracy for bounded, road-constrained accuracy — and we accept that coverage is limited to bundled map regions (the bundled Haifa OSM assets).
- **Benefits gained:** the map graph turns an unbounded drift problem into a bounded one (the ball physically cannot leave the network and wrong-fork recovery is bounded to 25–60 m), and the relative-to-start framing makes the problem solvable with commodity camera + IMU alone.
- **Engineering reasoning:** scope is a design tool. By excluding voice nav, SLAM, and cloud, we focused all the engineering on the one thing the user sees — a correct, well-timed ball on the road — and made the accuracy requirement measurable.

## Potential Questions (Defense)
**Q:** Why is "relative to a start point" acceptable — don't users need their absolute location?
**A:** For the GPS-denied use case, what the user needs is to follow a route correctly from where they are, and a road-snapped relative track delivers exactly that: the ball stays on the right road and reaches junctions on time. We anchor the start with a one-shot compass alignment and constrain everything to the OSM graph, so the trajectory is meaningful on a real map without needing a continuous absolute fix.
**Follow-up Q:** What happens if the relative track drifts over a long ride?
**Follow-up A:** Drift is bounded by the map graph rather than left to grow: the position is a ball constrained to the road network, and wrong-fork recovery is bounded by design to 25–60 m at confidence ≥ 0.55 — it can never teleport across town. On a 1,280 m route we still measured 93.4% distance accuracy, so the bounded drift stays well inside the road geometry.
**Follow-up A continues:** This is why we chose a map-constrained estimator over free dead reckoning.

**Q:** Why exclude turn-by-turn voice navigation — wouldn't that make it a real product?
**A:** It is a legitimate feature, but it is a separate concern layered on top of positioning, and the signed SDD scoped it out so we could fully solve the hard core: producing a trustworthy GPS-free position and speed. Voice guidance, full 3D SLAM, and cloud processing are all explicit non-goals; keeping them out is what let us hit the real-time budget and validate the core on real rides.
**Follow-up Q:** Is the system architecturally able to add voice guidance later?
**Follow-up A:** Yes — the output is a road-snapped position with heading and speed on the OSM graph, which is exactly the input a routing/guidance layer consumes. The scope decision was about project focus, not an architectural dead end.

## Speaker Notes
- The one-sentence problem statement is the slide's spine: *relative position + speed, no GPS fix, real time, on a phone, accurate enough to advance a map-matched ball to reach intersections on time.* Say it cleanly.
- Tie the accuracy requirement to something concrete and visible: the ball on the road (ui3.png) and the validated 93.4% distance on the 1,280 m Route A.
- Constraint numbers to have ready: 200 ms SDD per-frame budget; GPS-free on-device navigation (offline OSM road-matching; base map = Google Maps tiles); bundled Haifa OSM assets in the APK; target device Samsung Galaxy S21 Ultra (Exynos 2100, Mali-G78).
- Non-goals to state without hedging: turn-by-turn voice navigation, full 3D SLAM, cloud processing, external/online map services. Naming them pre-empts "did you do X?" questions.
- Pitfall to avoid: do not imply absolute global localization — the contract is relative-to-start, road-constrained. Stay disciplined on that wording.
