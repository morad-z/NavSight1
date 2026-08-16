# NavSight — Live Demo Runbook (Slide 21 Expansion)

**Build:** NavSight v1.0-osm (compileSdk/targetSdk 34 / Android 14; test device runs Android 15 / API 35) · **Device:** Samsung Galaxy S21 Ultra (SM-G998B, Exynos 2100, Mali-G78)
**Chosen workflow:** *Route A drive with GPS-free navigation (on-device OSM road-matching), Google Maps base map.*
**Total demo budget:** ~2 minutes live (≤90 s of driving narrated), or ~90 s on the recorded fallback.

---

## 0. Why This Is the One Demo

Route A is the single strongest validated artifact in the project. It carries the entire thesis in one continuous take:

- **GPS-free navigation** — the navigation is computed on-device (camera + IMU + on-device OSM road-matching) with no GPS and no network in the hot path. The base map shown on screen is Google Maps tiles; in airplane mode it falls back to cached/blank tiles while navigation keeps running offline.
- **On-road constraint** — the displayed "ball" is snapped to the on-device OSM road graph and physically cannot drift into a building.
- **Correct distance** — Route A is Google-measured at **1,280 m**; NavSight reported **1,195 m = 93.4%** (ρ = 0.93). Jammed GPS on the same route inflated the path to **1,705 m (+33%)**.
- **Exact-zero standstill** — the zero-witness lock holds a literal **0.0 km/h** at a stop, not a creeping 1–2 km/h.

Every other slide is supporting evidence for what this one ride shows in motion.

---

## 1. Pre-Demo Checklist & Setup

Run this list out loud (briefly) so the committee sees the setup is honest, then stop narrating internals once driving begins.

### Hardware / device
- [ ] **Device:** Samsung Galaxy S21 Ultra (SM-G998B), **charged ≥ 80%**, thermals cool (no prior heavy GPU load).
- [ ] **Build installed:** NavSight **v1.0-osm** — confirm the version string in-app, do **not** quote a git hash.
- [ ] **Haifa OSM assets present** — the bundled Haifa OSM road-matching assets ship in the APK; confirm the app opens a Haifa map and navigation runs with no network.
- [ ] **Phone mount** secured with a **clear, unobstructed forward view of the road** (rear camera sees the road surface ahead, not sky or dashboard). The IPM speed path depends on near-road texture.
- [ ] **Screen mirroring tested** to the projector/room display *before* the session (cable or wireless, whichever was rehearsed).

### Software / permissions
- [ ] **Airplane mode ON** — this is the headline proof that navigation runs with "no GPS, no network." Show the airplane icon on the status bar to the room. (The Google Maps base map will show cached/blank tiles in airplane mode; navigation keeps computing offline.)
- [ ] **Camera permission** granted.
- [ ] **Fine-location permission** granted (so the audience cannot claim the demo "hid" GPS — it is allowed but unused for the hot path, and there is no fix under airplane mode anyway).
- [ ] **No other camera/GPU app** running (close background apps to protect the frame budget).

### Calibration / state
- [ ] App **freshly launched** at the Route A start point (do not resume a stale session).
- [ ] **Mount/camera height ≈ 1.05 m** matches the calibrated value (the ground-plane model assumes the road is a plane at this height below the camera).
- [ ] **Gravity-alignment init done** — hold the device still for the first ~1–2 s so the filter levels and the heading seeds from travel direction (avoids a backwards-start).
- [ ] **VIO chip showing healthy (green)** before pulling away.

### Fallback assets staged (open in tabs/finder *before* the demo)
- [ ] `tests/sims/val_2026_06_03b/routeA_matched_traj.png` — matched trajectory, colour = time, U-turn retraces the same road.
- [ ] `tests/sims/val_2026_06_12/ui3.png` — live map, 0 km/h at standstill, green VIO chip, ball snapped to the OSM road graph (base map is Google Maps).
- [ ] `tests/sims/val_2026_06_03b/routeA_cumdist_gps_vs_dot.png` — cumulative distance: jammed GPS 1,705 m vs NavSight 1,195 m on a 1,280 m route.
- [ ] `tests/sims/val_2026_06_03b/routeA_google_measure.png` — Google-measured 1.28 km ground truth.
- [ ] `tests/sims/val_2026_06_12/ui_cam.png` and `tests/sims/val_2026_06_03b/probe_cruise_0.jpg` — camera overlay (amber mask + green flow), for the "is it really using the camera?" question.
- [ ] **V&V report:** `Final-Project/SDD/דוח אימות.pdf` (signed validation report; Figs 1–11). Per-figure page renders are in `tests/sims/val_2026_06_12/report_render/` (`p1.png`–`p11.png` map to Fig 1–11): Fig 3 speed-vs-time (`p3.png`), Fig 4 standstill (`p4.png`), Fig 5 distance bars (`p5.png`), Fig 6 timing histogram (`p6.png`), Fig 7 A/B bridge (`p7.png`).

---

## 2. The Scenario — Step by Step

> **Route A:** a Haifa urban loop, Google-measured at **1,280 m**, including a U-turn. Drive it (or play the recording) while narrating only the three watch-points.

| # | Action | Expected on-screen output | Talking point (what you SAY) |
|---|--------|---------------------------|------------------------------|
| **S0** | App open at start, device still, airplane mode visible. | Live map (ui3-style): ball snapped to the OSM road graph, **green VIO chip**, speed **0 km/h**; live camera overlay shows **amber sampling mask + green optical-flow arrows**. | "We're in airplane mode — no GPS fix, no network in the navigation. The ball you see is placed by the camera and IMU, snapped to the on-device Haifa OSM road graph. The base map is Google Maps." |
| **S1** | Pull away and reach cruising speed. | Ball advances **along the road**; speed readout climbs and tracks the ride; camera overlay keeps green flow on the road surface. | "As we move, the ball is graph-constrained — it advances along the locked road by our estimated speed. It physically cannot leave the network." |
| **S2** | Continue straight through one or two segments. | Ball stays **centered on the road line**, no lateral wander into adjacent/parallel streets. | "Watch the ball hold the road through these segments — no drift into the parallel street next door. That's the map-matcher absorbing lateral VIO drift." |
| **S3** | Execute the **U-turn**. | Ball **retraces the same road** it came in on — it does **not** jump to a parallel street or cut a chord. | "Here's the U-turn — the ball retraces the *same* road. A naive nearest-road snap would flip to whichever line is momentarily closest; the HMM decode keeps us on the road we actually drove." |
| **S4** | Come to a **full stop**. | Speed snaps to **exactly 0.0 km/h** and holds — no creeping 1–2 km/h. | "And at the stop — exact zero. The zero-witness lock means standstill reads a literal 0.0, not phantom creep — the displayed speed reads exactly 0.0 km/h throughout the stop on the validation ride." |
| **S5** | Resume and finish the loop. | Speed climbs cleanly off zero; ball continues on-road to the finish. | "Speed comes straight off zero and we finish on-road. Total route: 1,195 metres against a Google-measured 1,280 — 93.4%." |

### The three watch-points (the only things to narrate while driving)
1. **On-road** — the ball cannot leave the road network.
2. **U-turn retrace** — it returns on the **same** road, not a parallel one.
3. **Exact-zero stop** — standstill reads a literal **0 km/h**.

### Numbers to have on the tip of your tongue (only if asked mid-demo)
- Route A: **1,280 m** true → **1,195 m** NavSight = **93.4%**, ρ = 0.93.
- Same route, jammed GPS: **1,705 m (+33% inflated)** — i.e. GPS fails exactly where we hold.
- Standstill: displayed speed reads exactly **0.0 km/h** throughout the stop (~27 frames at exactly 0.0 on the cited ride).

---

## 3. Failure-Safe Fallback

**Trigger discipline:** if the ball misbehaves (drifts off-road, freezes, or the VIO chip goes non-green) for **more than a few seconds**, **cut to the recorded ride immediately** — do **not** debug live in front of the committee. The recorded path is the *same unmodified engine* on the *same recorded frames* via deterministic offline replay, so the fallback is not a mock; it is the identical pipeline on identical input.

### Fallback sequence (≈90 s)
1. **Open `routeA_matched_traj.png`.** Say: "Same engine, deterministic replay of the recorded Route A. Colour is time. Point one — the ball stays on the road end to end. Point two — here is the U-turn retracing the *same* road, not smearing across two."
2. **Open `ui3.png`.** Say: "And this is the live UI from the validation session — ball snapped to the on-device OSM road graph (base map is Google Maps), green VIO chip, and the speed reading **0 km/h** at a stop. Same three behaviors as the live ride."
3. **(If distance is questioned) open `routeA_cumdist_gps_vs_dot.png`.** Say: "Cumulative distance on the same 1,280 m route — NavSight tracks to 1,195 m; jammed GPS inflates to 1,705 m. We're within 7% of truth where the incumbent is off by a third."
4. **(If 'is it really the camera?' is questioned) open `ui_cam.png` or `probe_cruise_0.jpg`.** Say: "Live camera overlay — amber is the ground-plane sampling mask, green arrows are verified optical flow. That flow is what drives the speed."
5. **(If the panel wants the signed evidence) open the V&V report `Final-Project/SDD/דוח אימות.pdf`** and turn to the relevant figure (Fig 5 distance bars, Fig 4 standstill, Fig 3 speed-vs-time). Per-figure renders are pre-staged as `report_render/p3.png`, `p4.png`, `p5.png`.

> **Key line for the fallback:** "This is the same code that runs on the phone — replay only changes how frames and IMU are fed in, not the algorithm. Backed by a 95-case Kotlin unit suite and CI scoring on recorded fixtures."

---

## 4. What NOT to Demo

- **Do NOT narrate internals while driving.** Stick to the three watch-points. Explaining the EKF, HMM, or IPM math live invites questions you can't pause the road for — that material lives in Slides 12–16.
- **Do NOT demo subsystems in isolation as the main act.** Speed-only, optical-flow-overlay-only, or standstill-only each show a single module and *fail to show the integrated claim*. They are backup explainers, not the headline.
- **Do NOT attempt a live ride on a fresh/unverified route.** Route A is the validated case; an improvised route has no ground truth and risks an unbounded, unexplained result on stage.
- **Do NOT turn GPS off as the "trick."** The honest framing is airplane mode (no fix, no network). Avoid claiming "GPS is wrong everywhere" — jamming is *intermittent*; be precise.
- **Do NOT quote a git hash** for the build — refer to it only as **v1.0-osm**.
- **Do NOT over-claim 792 m** for the second ride if asked — quote it as **~792 m / 91% / ρ = 0.91** (telemetry-derived, approximate); Route A's **93.4%** is the firm number.
- **Do NOT debug a misbehaving ball live.** After a few seconds, cut to the recorded fallback — every second spent debugging on stage erodes confidence more than the fallback ever would.
- **Do NOT demo indoors expecting outdoor behavior.** The validated workflow is the outdoor scooter drive; the road-graph constraint is the whole story outdoors.

---

## 5. One-Line Recovery Cheat Sheet (tape to the lectern)

> Ball drifts/freezes > 3 s → **stop driving, open `routeA_matched_traj.png` then `ui3.png`** → narrate on-road + U-turn retrace + exact-zero stop → "same engine, deterministic replay, 93.4% on Route A."
