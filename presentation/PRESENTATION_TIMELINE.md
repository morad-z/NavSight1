# NavSight — Beyond GPS · Defense Presentation Timeline

**Build:** v1.0-osm (compileSdk/targetSdk 34, Android 14; test device runs Android 15 / API 35) · **Target talk length:** ~30 minutes + Q&A
**Team:** Roey Ben Harush · Tamir Sobuh · Morad Zubidat · **Supervisor:** Mr. Amit Dunsky

This guide is the top-level run-of-show for the 23-slide defense deck in `presentation/slides/`.
It is built from the **Estimated Time** declared in each slide file. It gives the per-slide budget,
the running total, who presents what, and where to spend or save time on stage.

---

## 1. Per-Slide Timing Table (as authored in the slide files)

| #  | Slide                          | Section                       | Est. (min) | Running total |
|----|--------------------------------|-------------------------------|-----------:|--------------:|
| 01 | Title                          | Opening                       | 0.5        | 0.5           |
| 02 | Project Overview               | Opening                       | 1.0        | 1.5           |
| 03 | Team & Collaboration           | Opening                       | 1.0        | 2.5           |
| 04 | Motivation                     | Problem & Context             | 1.5        | 4.0           |
| 05 | Problem Definition             | Problem & Context             | 1.5        | 5.5           |
| 06 | High-Level Architecture        | Architecture                  | 1.5        | 7.0           |
| 07 | Application Architecture       | Architecture                  | 1.5        | 8.5           |
| 08 | Infrastructure Architecture    | Architecture                  | 1.0        | 9.5           |
| 09 | Engineering Challenges (9a+9b) | Architecture & Engineering    | 2.0        | 11.5          |
| 10 | Technical Decisions            | Architecture & Engineering    | 1.5        | 13.0          |
| 11 | Runtime Overview               | Runtime Architecture          | 1.0        | 14.0          |
| 12 | Frame Lifecycle                | Runtime Architecture          | 1.0        | 15.0          |
| 13 | KLT Tracking                   | Subsystems                    | 1.0        | 16.0          |
| 14 | Map Matching                   | Deep Dive / Subsystems        | 2.0        | 18.0          |
| 15 | Speed Estimation               | Deep Dive / Subsystems        | 2.0        | 20.0          |
| 16 | Data Flow                      | Deep Dive / Subsystems        | 0.5        | 20.5          |
| 17 | Validation Methodology         | Validation & Results          | 1.0        | 21.5          |
| 18 | Results                        | Validation & Results          | 2.0        | 23.5          |
| 19 | Analysis                       | Validation & Results          | 1.0        | 24.5          |
| 20 | Impact                         | Impact & Closing              | 1.0        | 25.5          |
| 21 | Live Demo Guide                | Impact & Closing              | 2.0        | 27.5          |
| 22 | Future Work                    | Impact & Closing              | 1.0        | 28.5          |
| 23 | Closing & Questions            | Impact & Closing              | 1.0        | 29.5          |

**Authored total: 29.5 minutes** — i.e. **~30 minutes**, with ~0.5 min under the ceiling
and Q&A budgeted separately. Rehearse to this; the plans below are only for recovering time on stage.

---

## 2. On-Stage Time Management

The deck is authored at **29.5 min**, so it is compliant as written. Section 6 has the section budgets.
You do **not** need to cut anything — but a defense rarely runs to plan (a long Q&A interruption, a slow
demo start). Keep these **release valves** ready to recover 2–4 min without dropping a slide:

**Compress to ~27 min (recover ~2.5 min) — first valves, lowest information loss:**

| Slide | Authored | Compressed | Saved | Why this slide can shrink on the fly |
|-------|---------:|-----------:|------:|---------------------------------------|
| 06 High-Level Architecture | 1.5 | 1.0 | 0.5 | The 4 tiers are re-walked in 07/08/16 — give the overview once, fast. |
| 12 Frame Lifecycle         | 1.0 | 0.5 | 0.5 | One clean pass over the 6 stages; the FB-check detail repeats on 13. |
| 16 Data Flow               | 0.5 | (skip narration) | 0.5 | Pure "connect the dots" over the diagram — let one sentence land, move on. |
| 19 Analysis                | 1.0 | 0.5 | 0.5 | Land the three-reading framing; skip the derivation unless asked. |
| 20 Impact                  | 1.0 | 0.5 | 0.5 | Three headline numbers in one breath; no math. |

**Compress to ~25 min (recover ~4.5 min) — if the slot is hard-capped:**
Add to the above: **08 Infrastructure 1.0 → 0.5** (the "there is no backend" point lands in one line)
and **17 Validation Methodology 1.0 → 0.5** (state the two references and the conditioning rule, then
go to Results).

> **Do NOT trim these, ever:** **14 Map Matching (2.0)**, **15 Speed Estimation (2.0)**,
> **18 Results (2.0)**, and the **09 gravity-drift debugging story**. They are the technical core, the
> strongest defense material, and the most likely to draw questions.

> **Buffer policy:** rehearse to ~29.5 and hold the demo's natural slack in reserve. Slides 01, 03, 16,
> and 20 are the release valves — if you are running long by minute 20, take them at pace and recover
> time before Results.

---

## 3. Presenter Assignments (mapped to confirmed/declared ownership)

Ownership follows the 4-tier architecture, the same seam the team used to divide the work.
Morad's ownership is **confirmed** (inertial / fusion / calibration / validation harness);
Roey (Android/Compose UI + JNI bridge) and Tamir (C++ native VIO core / vision front-end) are
the **declared, to-confirm** splits per slide 03 — assign final speakers once the team confirms.

| Slides | Block | Presenter | Rationale (ownership match) |
|--------|-------|-----------|------------------------------|
| 01–03 | Opening (title, overview, team) | **Roey** (or whoever opens) | Sets identity; slide 03 is delivered first-person by each member for their own line. |
| 04–05 | Motivation + Problem Definition | **Tamir** | Framing the GPS-denied problem and scope; sets up the architecture. |
| 06–08 | Architecture (high-level, app, infra) | **Roey** | Tier 1 (Compose UI) + Tier 2 (JNI bridge) owner; speaks to module ownership and on-device infra. |
| 09–10 | Engineering Challenges + Technical Decisions | **Morad** | Carries the gravity-drift debugging story (a ~6–10° tilt mis-cancelled gravity → ~800 m phantom Z drift, fixed by the gravity-alignment update), the "read the data before tuning" rule, and the IMU/fusion challenges — all in his confirmed lane. |
| 11–13 | Runtime Overview, Frame Lifecycle, KLT | **Tamir** | Visual front-end / native VIO core owner — frame pipeline and KLT optical flow. |
| 14    | Map Matching | **Roey** | On-device OSM road-matching / `LocalMatcher` map layer (Kotlin side, declared). |
| 15    | Speed Estimation (IPM + inertial bridge) | **Morad** | Inertial bridge + sensor fusion is confirmed Morad; IPM ground-plane geometry pairs with it. |
| 16    | Data Flow | **Tamir** | "Connect the dots" across the native core he owns. |
| 17–19 | Validation Methodology, Results, Analysis | **Morad** | Integration testing + deterministic replay harness is confirmed Morad's; he owns the V&V narrative. |
| 20    | Impact | **Tamir** | Value framing, broad audience. |
| 21    | Live Demo | **Roey** drives the device, **Tamir** narrates | UI owner runs the app; core owner explains on-road / U-turn / zero-lock behavior. |
| 22–23 | Future Work + Closing | **Morad** | Closes on the three headline results he validated; whole team named on screen for thanks. |

> Keep handoffs to clean section boundaries (Opening → Problem → Architecture → Engineering →
> Runtime/Subsystems → Validation → Impact/Closing). Avoid mid-section speaker swaps.

---

## 4. Pacing Advice

**Go FAST (narration, not deep dive):**
- **01 Title (0.5 min)** — identity only; ~30 s. Do not preview numbers; let the talk build to evidence.
- **11 Runtime Overview (1.0 min)** — this is the "map of the deck" slide; point forward, don't explain.
- **16 Data Flow (0.5 min)** — one sentence must land: *"speed advances the ball, gyro-relative heading steers it, the road graph constrains it."* Then move on.
- **20 Impact (1.0 min)** — user value, not math; three headline numbers in one breath.
- **23 Closing (1.0 min)** — restate the three headlines, thank, open the floor. No new material.

**Go SLOW (highest-value, most defensible, most likely to draw questions):**
- **09 Engineering Challenges (2.0 min)** — the gravity-drift debugging story ("a ~6–10° tilt mis-cancelled gravity → ~800 m phantom Z drift; reading the residual data ended a multi-day debugging effort instead of tuning constants") is the strongest engineering narrative. Land "read the data before tuning constants."
- **14 Map Matching (2.0 min — hold full)** — the architectural thesis: *the matcher is the source of truth for position*; bounded 25–60 m recovery at conf ≥ 0.55; on-device OSM road-matching. Use the U-turn-retrace screenshot.
- **15 Speed Estimation (2.0 min — hold full)** — the two-sided VOTE / ZERO-WITNESS taxonomy is why standstill reads **exact 0**; the +24% inertial-bridge A/B (62 → 77 m, stored deterministic offline replay result) is the cleanest single-variable evidence in the deck.
- **18 Results (2.0 min — hold full)** — lead with the decisive contrast: NavSight 1,195 m on a 1,280 m route (within 7%) while jammed GPS read 1,705 m (+33%). This is the whole thesis in one chart.

**Critical framing discipline (recurring in every slide's notes):**
- Always say the build is **"v1.0-osm"** — **never quote a git hash** on stage.
- Say **"GPS-free navigation computed on-device"**: the navigation hot path (camera + IMU + on-device OSM road-matching) uses no GPS and no network. The base map is Google Maps tiles; GPS is an optional, untrusted secondary reference only.
- Distance is validated against **map-measured** ground truth; speed against **conditioned GPS** — never conflate the two references.
- Treat **792 m (ride 18:02)** as approximate/telemetry-derived; the firm labels are **93.4%** and **91% (ρ = 0.91)**.

---

## 5. The 2-Minute Live Demo (Slide 21)

The demo is the single highest-risk, highest-reward moment. Budget the slide at **2 min**: ~1 min live ride + ~1 min narration/fallback. **Do not spend stage time on setup** — complete the pre-flight before the talk.

**Pre-flight checklist (done BEFORE you present):**
- Charged Samsung S21 Ultra with the **v1.0-osm** build installed and the **bundled Haifa OSM assets present**.
- **Airplane mode ON** — navigation runs offline (no network / no live GPS fix); the Google Maps base map shows cached/blank tiles. Camera + fine-location permissions granted.
- Phone mounted with a clear forward view of the road; screen mirroring tested.

**Three watch-points to narrate (and only these):**
1. The ball is **graph-constrained** — it physically cannot drift off the road into a building.
2. On the **U-turn it retraces the same road**, not a parallel street.
3. At a stop, speed reads **exactly 0 km/h** (zero-witness lock), not a creeping 1–2.

**Headline to call out:** Route A = Google-measured **1,280 m** → NavSight **1,195 m (93.4%)**; jammed GPS would have said **1,705 m**.

**Fallback discipline:** if the live ball misbehaves for more than a few seconds, **cut immediately** to the recorded Route A — same unmodified engine via deterministic replay, validated to 93.4%. Show `routeA_matched_traj.png` (U-turn retrace) and `ui3.png` (GPS-free ball, green VIO chip) and narrate the identical three watch-points. The fallback is not a mock — it is the same pipeline on the same frames. The signed V&V report (`Final-Project/SDD/דוח אימות.pdf`) and its per-figure renders (`tests/sims/val_2026_06_12/report_render/p1.png`–`p11.png`) are the last-resort static evidence.

---

## 6. Quick Reference — Section Budgets

| Section | Slides | Authored (min) | Compressed (~27) |
|---------|--------|---------------:|-----------------:|
| Opening | 01–03 | 2.5 | 2.5 |
| Problem & Context | 04–05 | 3.0 | 3.0 |
| Architecture | 06–08 | 4.0 | 3.5 |
| Architecture & Engineering | 09–10 | 3.5 | 3.5 |
| Runtime & Subsystems | 11–16 | 7.5 | 6.5 |
| Validation & Results | 17–19 | 4.0 | 3.5 |
| Impact & Closing | 20–23 | 5.0 | 4.5 |
| **Total** |  | **29.5** | **~27.0** |

Rehearse against a stopwatch at least twice; the under-ceiling margin plus the release valves in §2 are
your insurance against a long Q&A interruption or a slow demo start.
