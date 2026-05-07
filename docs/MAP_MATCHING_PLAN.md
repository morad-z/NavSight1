# NavSight Map Matching Plan

**Status**: draft
**Owner**: Morad
**Scope**: add an OSM-backed map-matching layer that consumes
**VIO-tracked position** (never raw GPS) and (a) eliminates visible
off-road drift on the Compose map, (b) supplies a *bounded* absolute-
position correction back into the EKF when the matcher is confident,
(c) anchors heading at OSM intersection nodes (replacing the withdrawn
ADR-017 GPS-course-as-yaw mechanism), and (d) exposes the OSM data
layer that downstream destination-routing features (Step K) build on.

**Companion to**: `VISUAL_PRODUCTION_PLAN.md` (visual VIO, 11 steps),
`PRODUCTION_READINESS_PLAN.md` (inertial). Sibling to the visual plan,
not an insertion.

**Blocking dependency.** This plan does not start until the visual plan
has landed at least through Step 7 (loop closure) **and** today's
heading-convention fix (`EKFState.cpp` lines 1052/1097/1217, see
`docs/IMPLEMENTATION_STATUS.md` §"Today's work") has been validated on a
daytime sim re-walk. Map matching is the *last* line of defence against
drift, not the first. If the upstream estimator is wrong by 50 m the
matcher will confidently lock onto the wrong road; see Risks.

**What changed from the prior draft (2026-05-07 rework).** The original
draft assumed GPS samples (1 Hz, lat/lng + `gacc`) feed the HMM matcher
in classical Newson & Krumm 2009 fashion, and that ADR-017 (GPS course
as yaw) had landed. Both assumptions were wrong: NavSight is VIO-only
by design (ADR-004 + the GPS usage model documented in
`memory/reference_gps_usage_model.md`), and ADR-017 was withdrawn the
day it was drafted (`memory/feedback_no_gps_in_ekf.md`). The
**structural** decisions of the prior draft survive (HMM, GraphHopper,
intersection-anchored heading, off-road silent-disable, mount-mode
awareness, OSM tag analysis, library landscape). The **math** is rebuilt
from scratch around a windowed shape-similarity emission probability
that is robust to VIO's correlated-drift noise model.

---

## 1. Status / scope / dependencies

### Status
Draft. No code lands until visual Step 7 acceptance is signed off
against a re-walked daytime sim.

### Scope
- New on-device OSM data layer (Step A).
- New VIO-position-to-lat/lng projection in `Tracker` (Step B).
- New `MapMatcher` Kotlin/native component with a snap-to-nearest
  baseline (Step C) and a windowed-shape HMM matcher (Step D).
- New confidence model + off-road silent-disable (Step E).
- New `EKFState::updateMapPosition` channel for soft position
  observation (Step F).
- New `EKFState::updateMapIntersectionYaw` channel for intersection-
  anchored heading (Step G), replacing the withdrawn ADR-017.
- Mount-mode-aware routing-graph profiles (Step H).
- Replay harness extension + CI fixtures (Step I).
- ADR-018 (map matching as drift bound) + ADR-019 (intersection-
  anchored yaw) + ADR-020 (on-device OSM lifecycle) (Step J).
- Downstream: navigation-to-destination feature on top of the same OSM
  layer (Step K, deferred behind A–J acceptance).

### Out of scope (explicit non-goals)
- Raw GPS in the EKF hot path (forbidden by ADR-004; this plan does
  not erode that rule).
- Cross-session persistent matcher state (matcher is in-memory; the
  routing data on disk is the only persistence).
- Free-space movement graph for plazas/parks (Yan et al. 2024 — silent-
  disable is the cheaper answer for v1).
- Multi-user / crowd-sourced map corrections.
- Indoor map data (no source of truth available; matcher silent-
  disables indoors).

### Dependencies
- **Visual plan Step 7** (loop closure) must be accepted with
  `loop_closure_corrections_applied > 0` on a real walk.
- **Today's heading fix** (`EKFState.cpp` lines 1052/1097/1217) must
  be validated — see `docs/IMPLEMENTATION_STATUS.md` §"Today's work".
- **Inertial plan acceptance criteria 1, 2, 3** (50 m straight walk
  drift < 1 m, 20×20 m indoor square loop, 180° turn-in-place) must be
  cleanly passing.
- `Tracker::current_vio_lla()` accessor (new — Step B).
- OSM PBF for the deployment region available offline.

---

## 2. Why this plan exists

A real daytime walk recorded as
`tests/sims/simulation_data_1778147132092.json` accumulated **75°
heading drift over 8 minutes → 70+ m endpoint position error vs the
GPS-confirmed return point**, with the trajectory visibly cutting
through buildings on the rendered map. The proximate cause was the EKF
heading-convention sign bug (fixed today, validation pending) compounded
by the absence of an absolute heading anchor.

Even after today's heading fix and the visual plan's loop-closure layer
land, residual drift will produce visible off-road artefacts, especially:

- Indoor / GPS-denied segments where neither GPS bootstrap nor visual
  loop closure helps for the first traversal.
- Long sustained-motion segments between loop-closure events
  (loop closures fire 0–3 times per session per ADR-013).
- Scooter routes (3–8 m/s × ~5 km) where sub-degree heading drift
  compounded over kilometres puts the dot on the wrong street.

OSM road geometry is a free, dense prior on **where the user can
physically be**. On sidewalks/roads/cycleways the matcher snaps the
displayed dot to the right segment and — when confidence is high —
feeds back a soft position observation into the EKF. At intersections,
the angle between incoming and outgoing road segments is a strong yaw
observation that fires whenever the user actually turns — and it
functions at low speed where the withdrawn GPS-course observer would
have failed.

This plan scopes the map-matching layer end-to-end, with explicit guards
against the failure mode of "confidently wrong" map matches that ADR-004
was originally written to prevent for raw GPS.

---

## 3. Guiding principles (carry over from the visual plan)

These mirror `VISUAL_PRODUCTION_PLAN.md` §"Guiding principles" lines
38–91. Where a principle needs map-matching-specific phrasing, the
change is called out.

1. **No shortcuts.** Every step lands as a full implementation, with
   tests, sim recordings, and an ADR if a runtime invariant changes.
   No feature flags shipping disabled, no half-wired matchers behind
   "experimental" toggles.
2. **One source of truth per quantity.** Pose lives in EKF. The map
   matcher publishes a **display-layer** snapped polyline by default
   and a **soft EKF observation** only when its confidence clears the
   gate (Step E). It never directly writes EKF mean/covariance,
   identical to the ADR-006 invariant the loop closer respects.
3. **Covariance is mandatory.** Every map-matched position fed to the
   EKF carries a derived `R` from (matcher confidence × OSM accuracy ×
   EKF position covariance trace). A matcher that cannot produce a
   variance is a snap-to-nearest UI helper, not an estimator.
4. **Replay before re-flash.** Every step lands with at least one new
   labelled sim recording in `tests/sims/regression/`. CI runs the
   matcher offline against the same recording and pins the output.
   "Build and walk around" is debugging, not testing.
5. **Magic numbers are bugs.** Every threshold (window length,
   shape-distance scale, confidence cutoff, intersection proximity
   radius, chi-squared gate) cites its source: a literature value, an
   OSM-wiki accuracy figure, an empirical statistic from
   `tests/sims/`. Numbers that survive must be defensible.
6. **No mock-OSM in production.** Synthetic OSM tiles are a CI
   construct only (Step I). Production code must never have a "use
   synthetic road network" branch — same rule as ADR-007's "no mock
   camera in production" applied to map data.
7. **Observability is checked, not assumed.** The map-matched position
   observation introduces a new measurement type (Step F). It comes
   with a documented observability claim — what subspace it
   constrains, when it is rank-deficient (e.g. on a long straight
   road, only the *across-track* component is informative), and how
   the existing FEJ machinery from visual plan Step 11.5 carries
   through.
8. **Sensor pathologies are first-class.** Wrong-snap, no-snap-
   available (off-road), no-OSM-coverage (uncharted area), stale OSM
   data (newly built road), and dense-parallel-road ambiguity (urban
   canyon with multiple parallel streets within VIO drift) are
   handled explicitly, not skipped.
9. **Mount mode is a runtime variable.** `MountMode::PEDESTRIAN`
   includes sidewalks + footways + paths in the route graph;
   `MountMode::SCOOTER` prioritises cycleways + roads and de-prioritises
   sidewalks (riding sidewalks is illegal in many places but happens —
   the matcher tolerates it but never *prefers* it).
10. **Dead code is deleted; parked code is documented.** If a future
    iteration disables the matcher path, it lands an ADR explaining the
    rollback. There is no "leave it commented out" path.

**Two additional principles, map-matching-specific:**

11. **The matcher can always say "I don't know."** Every layer (snap,
    HMM, EKF feedback) emits a confidence in `[0, 1]`. Below threshold
    the layer publishes nothing — no snapped polyline, no EKF update.
    Silent-disable is the default. This is the load-bearing defence
    against "confidently wrong" failures: the EKF + visual loop closure
    + intersection-anchored yaw form a complete navigation stack on
    their own; the matcher is strictly additive.
12. **VIO is not GPS.** The matcher's input is `(t, vio_lat, vio_lng)`
    derived from `(startup_anchor + EKF.position) → haversine
    projection`, sampled at the EKF's ~22 Hz cadence. Standard GPS-
    input emission models (Newson & Krumm: Gaussian on per-sample
    perpendicular distance) do **not apply** because VIO noise is
    heavily correlated across samples — a single drift offset can move
    a 30 s window of samples uniformly off-road. Step D's emission
    probability is rebuilt around **trajectory-shape similarity** for
    that reason.

---

## 4. Algorithm landscape

One paragraph each on the four most relevant approaches, with citations
and a verdict.

### 4.1 Snap-to-nearest road (geometric baseline)

For each VIO sample, find the OSM way segment with minimum perpendicular
distance and project the sample onto it. With clean GPS this is bounded
by GPS noise plus segment density; with VIO input the cross-track error
can be 10s of metres of slow drift, and snap-to-nearest will silently
follow the drift onto the wrong parallel road. It cannot recover from
mis-snaps because each sample is independent. Cost: O(log N) per sample
with an R-tree spatial index. **Verdict**: ship as Step C (display-only
baseline) — useful as a fallback when the HMM's confidence is below
threshold and as the visual UX before HMM is ready. Never feeds the
EKF.

### 4.2 Hidden Markov Model with per-sample Gaussian emission (Newson & Krumm 2009)

The classical reference. Hidden states are road segments; emission
probability is a Gaussian on perpendicular GPS distance with σ ≈
4.07 m (Newson & Krumm's calibrated value); transition probability is
exponential on `|d_route − d_great_circle|`. Viterbi decoding picks the
most-likely state sequence. Reports 99.9% accuracy at 30 s sampling on
vehicular GPS traces. **Why it's wrong for our input:** the per-sample
Gaussian assumes radial-symmetric independent noise. VIO noise is
correlated across neighbouring samples (long-baseline drift offsets the
entire local trajectory by a roughly constant amount over tens of
seconds). A 5 m constant offset on a 30 s window of VIO points produces
30 individually low-likelihood observations relative to the true road,
yet the **shape** of those points perfectly matches the road — the
classical HMM throws that signal away. **Verdict**: cite as the
algorithmic ancestor; do not adopt the per-sample emission unmodified.
**Citation**: Newson, P., & Krumm, J. (2009). Hidden Markov Map
Matching Through Noise and Sparseness. *Proceedings of the 17th ACM
SIGSPATIAL GIS*.
https://www.microsoft.com/en-us/research/publication/hidden-markov-map-matching-noise-sparseness/

### 4.3 Online HMM with sliding-window decode (Goh et al. 2012)

Forward-only Viterbi with a sliding window. Trades the global optimality
of batch decoding for bounded latency. Used by Barefoot's online
matcher and by Valhalla's Meili. The sliding-window structure is exactly
what we need — but with a different emission probability per Step D.
**Verdict**: this is the decoder structure we adopt in Step D, with the
emission probability replaced by a windowed shape-similarity score.
**Citation**: Goh, C., Dauwels, J., Mitrovic, N., Asif, M., Oran, A., &
Jaillet, P. (2012). Online map-matching based on Hidden Markov model
for real-time traffic sensing applications. *IEEE ITSC 2012*.
https://web.mit.edu/jaillet/www/general/map_matching_itsc2012-final.pdf

### 4.4 Particle-filter / continuous-trajectory map matchers

A second family of map matchers that consume continuous trajectories
rather than discrete fixes — used historically for GPS-trace simplification,
LiDAR-input matching, and inertial-only dead reckoning. Lafarge & Mallet
(2012) particle-filter the trajectory against the road graph; more
recent work in pedestrian DR matches continuous PDR-derived position
windows against the graph topology. The structural insight relevant to
NavSight: **trajectory shape over a window** is robust to per-sample
correlated noise in a way that per-sample Gaussian emission is not.
**Verdict**: adopt the *shape-similarity* idea; use a windowed
Fréchet-style distance as the emission probability inside the
Goh-style online HMM decoder, rather than rebuild a full particle filter.
This composes the best of both lines of work.
**Citations**:
- Lafarge, F., & Mallet, C. (2012). Building large urban environments
  from unstructured point data. *ICCV 2011*.
- Eiter, T., & Mannila, H. (1994). *Computing discrete Fréchet
  distance*. Tech. Report CD-TR 94/64, TU Wien.
  https://www.kr.tuwien.ac.at/staff/eiter/et-archive/cdtr9464.pdf
- Müller, M. (2007). *Information Retrieval for Music and Motion*,
  Ch. 4 ("Dynamic Time Warping").
  https://www.springer.com/gp/book/9783540740476

### 4.5 Pedestrian-aware HMM extensions (open-field, low-speed)

Standard HMM breaks down for pedestrians in two specific regimes: open
fields (parks, plazas, parking lots) where there is no road candidate
within emission radius, and low-speed walking near intersections where
direction information is noisy. Recent papers extend the basic HMM with
free-space movement nodes derived from `landuse=*` polygons (Yan et al.
2024) or heuristic look-ahead during open-field traversal. **Verdict**:
adopt the *off-road silent-disable* idea (Step E — the matcher emits
low confidence in open-field areas, which silently disables the EKF
feedback) but defer the explicit free-space graph as future work.
Confidence-gated silent-disable is the cheaper, more robust answer.
**Citation**: Yan, et al. (2024). An Enhanced Hidden Markov Model for
Map-Matching in Pedestrian Navigation. *Electronics 13(9), 1685*.
https://www.mdpi.com/2079-9292/13/9/1685

---

## 5. Library options ranked (on-device feasibility)

### 5.1 GraphHopper (Java, Apache 2.0) — recommended baseline, with caveat

Routing engine with a built-in map-matching module that wraps the same
`hmm-lib` used by Newson & Krumm. Officially supports Android (F-Droid
build `com.graphhopper.maps`); routing data file size scales with map
size, ~30 MB for a small region. Apache-2.0 license is APK-clean.

**Pros**: native Java (no NDK build), supports both online and offline
matching, HMM implementation is the canonical reference, F-Droid ships
an actual Android binary so the path is proven, the pre-processed
routing graph and R-tree it produces are reusable for both the matcher
*and* the destination-routing feature in Step K.

**Cons (and the load-bearing one for this plan)**: GraphHopper's
matcher exposes the canonical Gaussian-on-perpendicular-distance
emission and does **not** publicly support injecting a custom emission
function. Verified by reading
https://docs.graphhopper.com/openapi/map-matching and the
`graphhopper/map-matching` repository
(https://github.com/graphhopper/graphhopper/tree/master/map-matching).
This plan therefore uses GraphHopper for **the data layer**
(PBF preprocessing, routing graph, R-tree spatial index, Dijkstra/CH
routing for Step K) and replaces its matcher with a custom HMM decoder
on top of GraphHopper's graph (Step D). The custom decoder is small
(~300 LOC of Kotlin/native) and reuses GraphHopper's data structures
through its public API. **APK binary impact remains Unknown — needs
spike**: GraphHopper's docs warn about size and recommend `autojar`
stripping. Step A's spike confirms ≤ 30 MB compressed delta.

**Verdict**: ship GraphHopper for data + routing; build the matcher
on top. If the size spike fails, fall back to a hand-rolled spatial
index over a vendored OSM PBF parser (next option down).
**Source**: https://github.com/graphhopper/graphhopper ;
https://f-droid.org/packages/com.graphhopper.maps/

### 5.2 Valhalla / Meili (C++, MIT) — viable but heavy, fallback for routing

Mapbox's HMM matcher, C++ throughout, designed for tile-based serving.
Tiles for a country-sized region are 100 MB to several GB. A
community-driven mobile build exists at `Rallista/valhalla-mobile`.
Like GraphHopper its matcher is hard-coded around per-sample Gaussian
emission, so we'd still build a custom decoder on top. NDK build of a
third-party C++ project of this size is real engineering cost.
**Verdict**: documented fallback if GraphHopper fails the spike. Do
not start integration speculatively.
**Source**: https://valhalla.github.io/valhalla/meili/ ;
https://github.com/Rallista/valhalla-mobile

### 5.3 Barefoot (Java, Apache 2.0) — server-only, rejected for runtime

BMW Carit's HMM matcher implementing Newson & Krumm offline + Goh et
al. online. Excellent algorithm reference. But the project assumes
PostgreSQL/PostGIS and a Docker-based map import pipeline; no mobile
target, dependency graph would balloon the APK. **Verdict**:
reference-only — use the Java source as a correctness oracle for the
custom decoder's unit tests in Step D.
**Source**: https://github.com/bmwcarit/barefoot

### 5.4 OSRM (C++) — server-side primarily, rejected

Routing engine with map-matching API. Designed for server deployment,
no mobile build path documented, matching algorithm is geometry-only
(snap-to-nearest with shortest-path consistency check), weaker than
HMM on noisy input. Ruled out.

### 5.5 osmdroid (Java) — display-only, complementary

Pure Android map renderer. No matching capability. We may use osmdroid
for the map-display layer regardless of which matcher we pick, but it
contributes zero algorithmically.
**Source**: https://github.com/osmdroid/osmdroid

### 5.6 fmm / LeuvenMapMatching — desktop-only correctness oracles

Both excellent academic implementations, both batch-only desktop
targets. Useful as offline replay-scorer references (Step I) — same
role as Barefoot. Not shipped on device.
**Sources**: https://github.com/cyang-kth/fmm ;
https://github.com/wannesm/LeuvenMapMatching

### Summary

GraphHopper for data layer + routing; **custom HMM decoder built on
top of GraphHopper's graph and R-tree**, with a windowed shape-
similarity emission per Step D; Barefoot + LeuvenMapMatching as offline
correctness oracles; osmdroid as the display-layer renderer. Valhalla
is the documented bailout if GraphHopper's APK-size spike fails.

---

## 6. Data layer

### 6.1 OSM data acquisition

Geofabrik publishes daily-updated PBF extracts. The relevant extract
for NavSight's primary deployment is **Israel and Palestine combined**
(~140–200 MB PBF as of 2026, no per-country split available). Country-
sized extracts for European cities are 200 MB – 4 GB.

**Strategy**:

- **Default ship**: pre-built GraphHopper routing data for Israel +
  Palestine, embedded in the APK as a raw asset. Size budget: 50 MB
  compressed (PBF → GraphHopper conversion typically reduces footprint
  vs raw PBF for the routing-only subset).
- **On-demand download**: when the user travels outside the cached
  region, the app prompts to download the relevant Geofabrik regional
  extract and preprocess it on first launch. UX: progress bar,
  estimated time, must be on Wi-Fi. Storage budget: up to 500 MB per
  extra region; user can delete regions in Settings.
- **Update cadence**: 30 days default re-download prompt; user-
  configurable 7/30/90 days or "manual only". Re-download is opt-in
  (Wi-Fi confirmation) — never silent.

**Source**:
https://download.geofabrik.de/asia/israel-and-palestine.html

### 6.2 On-device storage budget

| Item | Budget |
|---|---|
| Default (Israel) routing graph | 50 MB compressed |
| Per extra region | ≤ 500 MB |
| GraphHopper jar + transitive deps (post-`autojar`) | ≤ 30 MB compressed |
| ORBvoc.dbow2 (already shipped, ADR-013) | 10 MB |
| **Total APK delta from this plan** | **≤ 30 MB compressed** (Step A gate) |
| **Total per-region writable storage** | **≤ 200 MB** (Step A gate) |

### 6.3 Pedestrian / scooter tag coverage

OSM tagging for pedestrian-relevant ways:

- `highway=footway` — explicit pedestrian footway.
- `highway=path` — generic path, often outside urban areas.
- `highway=pedestrian` — pedestrianised street/square.
- `highway=steps` — staircases.
- `sidewalk=both | left | right | none` — implicit sidewalk attribute
  on a parent road.
- `footway=sidewalk` vs `footway=crossing` — sub-classifications.

**Coverage variance**: OSM US's 2024 pedestrian-data report documents
US cities adding ~10,000 km of footways in a single year, with city-
by-city growth ranging from baseline to >50% YoY. Sidewalk/footway
coverage is **highly city-dependent and improving fast** but not
universal. NavSight cannot assume a footway exists for every walked
path. Off-road handling (Step E) is mandatory, not nice-to-have.
**Sources**:
https://openstreetmap.us/news/2025/03/pedestrian-data-trends/ ;
https://wiki.openstreetmap.org/wiki/Tag:highway=footway ;
https://wiki.openstreetmap.org/wiki/Sidewalks

Scooter-relevant tags:
- `highway=cycleway` — dedicated cycleway.
- `cycleway=lane | track | shared_lane` — bike-lane attribute.
- `bicycle=yes | no | designated | dismount` — access tag.
- `highway=residential | tertiary | unclassified` — shared road
  network.

Cycleway coverage in Israel is incomplete in many neighbourhoods; the
matcher must tolerate "scooter on a regular road" as a valid state
(Step H).
**Source**: https://wiki.openstreetmap.org/wiki/Tag:highway=cycleway

### 6.4 Off-road pedestrian time

We could not find a definitive published statistic for "fraction of
pedestrian travel time spent on `highway=*` ways." Anecdotally Haifa
walks regularly cross plazas, car parks, university courtyards, and
apartment-block gardens that are not mapped as ways. Step I's CI
fixtures must include at least one walk that is >30% off-road by
ground-truth label, so the off-road detector (Step E) has a regression
test.

---

## 7. Mount-mode implications

The pedestrian/scooter split from `VISUAL_PRODUCTION_PLAN.md` §"Mount
mode" carries through to map matching. Implications per step:

| Property | Pedestrian | Scooter |
|---|---|---|
| Speed | 0–1.5 m/s | 3–8 m/s |
| VIO drift rate | ~5 m / 5 min on a clean walk after today's heading fix; higher indoors | ~5–10 m / km on a clean ride |
| Window length needed for shape-match | Longer (slower distance accumulation per second) → 30 s default | Shorter — same 30 s default covers more geometric variation, sufficient |
| Routable surface | Sidewalks + footways + paths + roads with `foot != no` | Cycleways (priority) + roads with `bicycle != no`; sidewalks tolerated, never preferred |
| Off-road frequency | Common (parks, plazas, indoors) | Rare on a typical ride |
| Loop length | 100–500 m typical | 1–5 km typical |
| Wrong-snap penalty | A 5 m mis-snap onto a parallel road shows visibly | A 5 m mis-snap onto the wrong street confidently mis-routes the dot for hundreds of metres |
| Intersection-yaw value | High — a 90° turn at a corner is a clean yaw observation, especially at low speed | Higher — scooter turns are crisper and faster |

**Per-step implications:**

- **Step A**: same data acquisition path; runtime profile filter differs
  (Step H).
- **Step B**: VIO-position projection is mount-independent.
- **Step C**: snap-to-nearest weights ways by mount-mode access in
  candidate selection.
- **Step D**: emission window length may differ slightly per mode.
- **Step E**: pedestrian off-road detection is the load-bearing
  difference. Scooter mode rarely needs off-road handling.
- **Step F**: EKF position-observation variance is mode-dependent.
- **Step G**: intersection-yaw works in both modes; proximity threshold
  is mode-dependent (slower walker takes longer to clear the node).
- **Step H**: dedicated mount-mode-aware routing-graph profile.

---

## 8. Steps

### Step A — OSM data acquisition + on-device storage

**Goal**: the app ships with a usable GraphHopper routing graph for the
primary deployment region and can download extras on demand. End state:
first city downloaded and indexed in < 60 s on a mid-range phone, total
storage < 200 MB for the default region.

#### Full implementation plan

1. **GraphHopper Android spike** (gate for the rest of the plan):
   - Add GraphHopper to `app/build.gradle.kts` with `autojar`-stripped
     dependencies. Build a debug APK and measure size delta.
   - Pre-process Israel+Palestine PBF
     (`download.geofabrik.de/asia/israel-and-palestine-latest.osm.pbf`)
     into GraphHopper routing format on a developer machine.
   - Embed as `app/src/main/assets/osm/israel.gh/`.
   - On first launch, copy assets to internal storage (GraphHopper needs
     a writable directory).
   - **Acceptance gate**: APK size delta ≤ 30 MB compressed; on-device
     routing data ≤ 200 MB; first-launch copy + index time ≤ 60 s on a
     Snapdragon 695-class device.
   - **If gate fails**: fall back to a hand-rolled R-tree over a
     vendored OSM PBF parser. ADR required documenting the choice.
2. **Region manager**:
   - Settings screen lists installed regions with size; user can delete.
   - "Add region" lists Geofabrik regional extracts with download size
     estimates.
   - Download → preprocess pipeline runs in a foreground service
     (notification: "preparing map for navigation in <region>").
   - Preprocessing must run off the UI thread and survive process death
     (resume on next app launch).
3. **Update cadence**:
   - Per-region timestamp recorded at preprocess time.
   - Settings shows "Last updated: X days ago" per region.
   - Default re-download prompt at 30 days; user-configurable.
   - Re-download is opt-in (Wi-Fi prompt); never silent (data plans).

#### Acceptance criteria

- APK size delta ≤ 30 MB compressed (GraphHopper + Israel routing
  data).
- First-launch index time ≤ 60 s on a Snapdragon 695-class device.
- Total per-region storage ≤ 200 MB for Israel+Palestine.
- Add-region flow downloads + preprocesses an EU country extract end-to-
  end with the user able to use the app during preprocess (existing
  region remains available).
- ADR-020 documenting the GraphHopper-vs-fallback outcome and the on-
  device storage strategy lands with the code.

---

### Step B — VIO-position projection (`Tracker::current_vio_lla`)

**Goal**: produce `(t, vio_lat, vio_lng)` as the matcher's input
stream, derived from `(startup_anchor_lla + EKF.position) → haversine
projection`. **No raw GPS sample ever feeds the matcher — only
VIO-derived position.**

#### Full implementation plan

1. **`SessionAnchor` struct**: stored once at session bootstrap from the
   first valid GPS fix (per `memory/reference_gps_usage_model.md`).
   Fields: `anchor_lat_rad`, `anchor_lng_rad`, `anchor_t_ns`. Frozen
   after Tracker::start; never mutated by GPS callbacks afterward.
2. **`Tracker::current_vio_lla()` accessor** (new public method):
   - Reads `EKFState::getPosition()` (local-frame metres).
   - Applies inverse haversine to get
     `(vio_lat = anchor_lat + dy/EARTH_RADIUS,
       vio_lng = anchor_lng + dx/(EARTH_RADIUS·cos(anchor_lat)))`.
   - Returns `VioLla{lat_rad, lng_rad, t_ns, var_xy_m2}` where
     `var_xy_m2` is the EKF's current position covariance trace
     (publishes uncertainty alongside position).
3. **UI display**: Compose map shows two dots:
   - Raw EKF position (small grey dot, in metres relative to anchor).
   - VIO-projected lat/lng (matched to OSM tile coordinates, larger
     blue dot — pre-Step C this dot is the projection itself; from
     Step C onward it becomes the snapped position).
4. **Sim recording**: `simulation_data_*.json` gains a per-frame
   `vio_lla` field alongside the existing `glat/glng/gacc`. Used by
   `scripts/compare_gps_vio.py` for offline comparison.
5. **No EKF coupling.** This step is read-only on EKF state.

#### Acceptance criteria

- On a stationary sim (phone-on-a-table), the `vio_lla` stream is
  within haversine distance equal to the EKF's reported position
  standard deviation of the recorded GPS lat/lng (i.e. the projection
  doesn't introduce additional bias beyond the EKF's stated
  uncertainty).
- On the cited 8-min Haifa walk after today's heading fix, the
  `vio_lla` track plotted on a map has the expected `~75°` reduction
  compared to the pre-fix track — confirms projection correctness end-
  to-end.
- No regression on EKF behaviour (this step is read-only on the
  estimator).

---

### Step C — Snap-to-nearest baseline (display-only)

**Goal**: ship the simplest possible map-matched display layer first,
to get the UX path running before the windowed-HMM is ready. **Never**
fed to the EKF.

#### Full implementation plan

1. **R-tree spatial index** built from the GraphHopper routing graph
   on first launch, kept in memory. Indexes way segments (not nodes)
   for per-segment perpendicular-distance queries.
2. **Snapping logic** in `MapMatcher::snap`:
   - Input: `(vio_lat, vio_lng, vio_var_xy_m2)`.
   - Output: `SnapResult(snapped_lat, snapped_lng, segment_id,
     distance_m, confidence)`.
   - Confidence (baseline only; replaced by Step D HMM normalised
     probability):
     `confidence = exp(-distance_m / sigma)` where `sigma = max(5 m,
     sqrt(vio_var_xy_m2))`. The 5 m floor cites OSM way-geometry
     accuracy from the OSM Wiki Accuracy page
     (https://wiki.openstreetmap.org/wiki/Accuracy).
   - Returns `null` if no segment within `5 × sigma` (off-road).
3. **Display layer wiring**:
   - The Compose map renders both the raw VIO-projected position
     (small grey dot) and the snapped position (larger blue dot) when
     snap confidence > 0.3 (heuristic for the display-only path; Step
     D replaces it).
   - Debug toggle in Settings shows raw + snapped + recorded GPS as
     three separate dots for replay analysis.
4. **No EKF coupling.** Display only.

#### Acceptance criteria

- On `tests/sims/simulation_data_1778147132092.json` (after today's
  heading fix lands), the snapped polyline visually follows roads on
  the rendered map for ≥ 80% of sustained-motion samples.
- The snapped dot disappears when the user enters a building (visible
  in the test fixture) — no false snapping.
- No regression on EKF behaviour.

---

### Step D — Windowed shape-match HMM (display-only)

**Goal**: replace the per-sample snap-to-nearest with an online HMM
whose **emission probability is computed on a sliding 30 s window of
VIO positions vs each candidate road segment's geometry**, robust to
the constant-offset drift mode that breaks per-sample emission. Still
display-only; EKF integration comes in Step F.

#### 8.D.1 The emission probability model (new math — load-bearing)

Standard Newson & Krumm assumes a Gaussian on perpendicular distance
from a single GPS sample to a candidate road segment. With VIO input
this fails because a single drift offset moves the entire local
trajectory uniformly off-road, and per-sample distance is no longer an
independent draw from a known noise distribution — neighbouring samples
are correlated.

The fix is to evaluate **trajectory shape** over a window. The key
observation: VIO short-term shape (first/second derivatives of the
position trajectory) is accurate (~1 m local consistency over 30 s,
per `memory/reference_gps_usage_model.md`); only the absolute offset
drifts. A windowed shape-distance metric is invariant or weakly
sensitive to offset translation, exactly the property we want.

**Definitions:**

- Sliding window: `W(t) = {(t_i, vio_lat_i, vio_lng_i) : t_i ∈ [t-30s, t]}`
  at the EKF's ~22 Hz cadence (≈ 660 samples per window).
- Candidate window: each candidate road segment's geometry, sampled at
  matching arc-length spacing to produce `C_k = {(s_j, lat_j, lng_j)}`.
- Pose-aligned shape distance:
  `D_shape(W, C_k) = discreteFréchet(W − offset(W, C_k), C_k)`
  where `offset(W, C_k)` is the mean position offset between `W` and
  `C_k` over the window.

  The offset subtraction is what makes this robust to constant-offset
  VIO drift: a 5 m drift offset moves all VIO samples uniformly, which
  the offset subtraction removes before measuring shape mismatch.
  Pure-shape errors (turning where the road doesn't, going straight
  where the road bends) survive the subtraction and produce real shape
  distance.

- Shape-DTW alternative: `D_shape(W, C_k) = shapeDTW(W − offset(W, C_k), C_k)`
  with the local-shape descriptors of Zhao & Itti (2018,
  https://arxiv.org/abs/1606.01601). Empirically preferred when window
  endpoints are noisy.

**Emission probability:**

```
p_emit(W | state = segment_k) ∝ exp(-D_shape(W, C_k) / lambda)
```

with `lambda = 5 m` derived from OSM way-geometry accuracy
(https://wiki.openstreetmap.org/wiki/Accuracy) plus typical 30 s VIO
shape-noise from `tests/sims/` (to be empirically calibrated in Step
I; treat 5 m as a starting prior, refine with data).

**Normalised softmin emission across all candidates:**

```
p(state_k | W) = exp(-D_shape(W, C_k) / lambda) /
                 Σ_j exp(-D_shape(W, C_j) / lambda)
```

This is the HMM emission term; the candidate set is pruned to within
`3 × sqrt(vio_var_xy_m2)` of the current VIO position (standard HMM
practice, scaled here to VIO uncertainty rather than constant GPS-σ).

#### 8.D.2 Transition probability

Goh-style transition probability on routing distance:

```
p_trans(k → k') ∝ exp(-|d_route(k, k') - d_vio(W)| / beta)
```

where `d_route(k, k')` is the GraphHopper routing distance between the
two segments and `d_vio(W)` is the cumulative VIO arc-length over the
window. `beta = 2.0` per Newson & Krumm; re-derive empirically in Step I.

#### 8.D.3 Online Viterbi decoder

Forward-only Viterbi with the sliding window. Publishes a snap with
~1 s decode lag — acceptable for real-time navigation per Card et al.
1991 (sub-2 s display delay imperceptible for navigation). The decoder
is ~300 LOC of Kotlin built on top of GraphHopper's R-tree + routing
graph; no GraphHopper matcher code reused.

#### 8.D.4 Per-mode parameter selection

- `MountMode::PEDESTRIAN`: include footways/paths/pedestrian/steps;
  window length 30 s.
- `MountMode::SCOOTER`: cycleways + roads with `bicycle != no`;
  exclude footways/pedestrian; window length 30 s (covers ~150 m at
  5 m/s — sufficient for shape).

#### Acceptance criteria

- On a hand-labelled outdoor walk fixture (Step I delivers labelling
  tooling), the HMM correctly identifies the road segment on **≥ 95%
  of sustained-motion samples** (`speed > 1.0 m/s`, `not OFF_ROAD`).
- Discrete Fréchet computation per candidate: ≤ 3 ms p95 on a
  Snapdragon 695-class device. Total decoder cost per sample
  (candidate set ~10) ≤ 50 ms p95. **Unknown — needs spike**: if
  exceeded, switch to a coarser-grain window (resample VIO at 5 Hz
  inside the window, ~150 samples per 30 s window).
- No false off-road declarations on a clean outdoor walk
  (false-off-road rate < 1% of samples).
- The Step C snap-to-nearest path remains the fallback when the HMM's
  normalised top-state probability < 0.5 AND the snap distance < 10 m.

---

### Step E — Confidence model + off-road detection

**Goal**: build the explicit decision logic for "should the matcher be
active right now?" and the silent-disable gate.

#### Full implementation plan

1. **`MapMatchConfidence` struct** with fields:
   - `hmm_score ∈ [0, 1]` — Step D normalised top-state probability.
   - `shape_distance_m` — `D_shape(W, top_candidate)`.
   - `transition_consistency ∈ [0, 1]` — agreement between routing
     distance over consecutive snapped states and VIO-cumulative arc-
     length.
   - `vio_var_xy_m2` — EKF position covariance trace.
   - `loop_closure_recency_s` — seconds since last accepted loop
     closure correction (proxy for absolute-frame freshness).
   - `osm_data_age_days` — region-level metadata from Step A.
   - `final ∈ [0, 1]` — combined.

2. **Combination rule (each factor in `[0, 1]`, conservative AND):**
   ```
   final = hmm_score
         * sigmoid((lambda - shape_distance_m) / (lambda/2))
         * transition_consistency
         * sigmoid((25 - sqrt(vio_var_xy_m2)) / 10)
         * sigmoid((300 - loop_closure_recency_s) / 60)
   ```
   The VIO-variance factor down-weights the matcher when the EKF is
   already uncertain (don't double-count drift). The loop-closure-
   recency factor up-weights right after a loop closure, when the
   absolute frame is freshest. Each threshold cites a derivation in
   the ADR.

3. **Off-road state machine**:
   - `STATE_ON_ROAD`: `final > 0.5` for ≥ 3 consecutive samples (~140
     ms at 22 Hz).
   - `STATE_UNCERTAIN`: ≤ 2 of last 3 samples passed.
   - `STATE_OFF_ROAD`: ≥ 3 of last 3 failed AND nearest road > 25 m.
   - Hysteresis: 5 s of consistent state before transitioning out of
     `STATE_OFF_ROAD` (prevents flicker on plaza crossings).

4. **Indoor detection bridging**: when the EKF position covariance
   trace exceeds 30 m² for ≥ 5 s OR the visual front-end reports
   `low_texture` for ≥ 5 s, treat as indoor → `STATE_OFF_ROAD`. (No
   `gps_accuracy` here — the bridging signal is purely VIO/visual
   health.)

5. **Telemetry**: per-sim `event_summary` gains
   `map_match_state_transitions`, `map_match_off_road_seconds`,
   `map_match_low_confidence_seconds`.

#### Acceptance criteria

- The matcher emits a confidence in `[0, 1]` for every input sample.
- On an indoor-segment sim, `STATE_OFF_ROAD` engages within 5 s of
  entering a building and disengages within 5 s of exiting — no
  display flicker, no false EKF feedback during indoor segments.
- On a deliberate plaza-crossing test (a sim labelled
  `landuse=plaza`), the matcher emits `STATE_OFF_ROAD` and re-engages
  cleanly on the far side.
- Bit-identical EKF behaviour with-and-without the matcher when state
  is `OFF_ROAD` (proves silent-disable correctness).

---

### Step F — Soft EKF position observation (`updateMapPosition`)

**Goal**: when Step E emits `STATE_ON_ROAD` with high confidence,
inject the snapped position as a soft EKF observation through a new
`EKFState::updateMapPosition(p_world, var_xy)`. This is the *only*
step where the matcher writes back into the estimator. Lands with the
same care as ADR-013's loop-closure injection: damped, χ²-gated, never
direct mutation.

#### 8.F.1 Why a soft observation, not a snap

A hard snap would violate "covariance is mandatory," break ADR-006's
"no direct EKF mutation," and produce 5–11 m teleportations the way the
original disabled MSCKF path did. Soft observation through `update*`
lets the EKF reconcile new information with existing covariance, the
way `updateAbsolutePose` does for loop closures (ADR-013).

#### 8.F.2 Variance derivation

```
var_xy_m² = max(
    lambda²,                              // shape-emission noise floor (~25)
    vio_var_xy_m2 * (1 - hmm_score)       // soft trust on HMM consensus
) + osm_position_accuracy²                 // OSM way-geometry uncertainty (~9)
```

The first term floors variance at the shape-emission noise lambda² (~25
m²; can never claim more accuracy than the matcher's own noise model).
The second inflates variance when HMM is uncertain (and crucially: when
EKF is *certain* but matcher isn't, we don't let the matcher dominate).
The third adds OSM way-geometry uncertainty, ~3 m sigma for well-
mapped areas (https://wiki.openstreetmap.org/wiki/Accuracy).

#### 8.F.3 Full implementation plan

1. **New EKF method** `EKFState::updateMapPosition(p_world, var_xy)`
   in `EKFState.{h,cpp}`:
   - 2-DOF position observation in world XY (Z is not observed —
     keeps gravity vertical untouched).
   - Reuses `updateAbsolutePose`'s position-update Jacobian path for
     FEJ consistency.
   - Inner χ² gate at χ²(0.95, 2) = 5.99; outer hard gate at 22.5
     (matches ADR-013 §"Absolute-pose injection channel").
   - Damping schedule: scale `dx_position` by 0.5 on the first frame
     after a state-change-on event, ramping to 1.0 over 5 frames
     (matches ADR-008 MSCKF damping).

2. **Wiring in `Tracker.cpp`**:
   - On each EKF update tick, after Step D + E produce a confidence and
     state, if `STATE_ON_ROAD` and `final > 0.7`, call
     `ekf_.updateMapPosition(snapped_xy, var_xy)`.
   - Rate-limited to 1 Hz (one update per second max) to avoid
     overwhelming the filter with correlated observations.

3. **Mount-mode-aware threshold**:
   - Pedestrian: `final > 0.7` to engage feedback.
   - Scooter: `final > 0.6` (scooters have stronger transition evidence
     per second due to higher displacement; we can be more trusting).

4. **ADR-018** documents the rationale, variance model, χ² gates, and
   relationship to ADR-004. The matcher feeds a *VIO-derived* signal
   that has already been laundered through HMM consensus + confidence
   gating — distinct from raw GPS, which ADR-004 forbids. ADR-004's
   spirit is preserved (jamming defence: a jammer cannot affect VIO,
   so cannot affect this channel).

#### Acceptance criteria

- On `tests/sims/simulation_data_1778147132092.json` after today's
  heading fix + visual Step 7 land, position drift bounded to ≤ 5 m
  RMS on outdoor segments where `STATE_ON_ROAD` is active.
- No degradation on indoor / off-road segments: trajectory bit-
  identical with-and-without the matcher when `STATE != ON_ROAD`.
- No teleportation events (>2 m position jump in a single update).
- Synthetic wrong-snap test: inject a fake snap 30 m off the true
  trajectory; the χ² gate rejects.
- ADR-018 lands with the code.

---

### Step G — Intersection-anchored heading observer (`updateMapIntersectionYaw`)

**Goal**: when the user passes through an OSM intersection node and
the gyro confirms a turn, fuse the OSM intersection's geometric
incoming-vs-outgoing road angle as a 1-DOF yaw observation. **This is
the heading-anchor mechanism that replaces the withdrawn ADR-017
GPS-course-as-yaw observer** — see `memory/feedback_no_gps_in_ekf.md`
for the design rationale.

#### 8.G.1 Why this is its own step

Intersection nodes are **better** yaw anchors than GPS course (the
withdrawn ADR-017 channel) and than any other VIO-derivable heading
signal:
- They provide a measurement at low speed (where GPS course fails
  structurally — small per-second displacement → noisy course).
- They have low variance (OSM way bearings ~1° for surveyed roads vs
  the 5° floor on course-from-displacement).
- They fire at events the user actually does (turning a corner), not
  continuously — minimal correlation with VIO drift mode.
- They are entirely VIO-fed: the matcher detects the intersection from
  the snapped polyline and consumes EKF yaw rate to confirm a turn.
  No GPS in the path.

#### 8.G.2 Full implementation plan

1. **Intersection-node index** — at Step A preprocessing, extract OSM
   nodes with degree ≥ 3 (≥ 3 incident ways) into a separate spatial
   index. Bearing-pairs at each intersection are precomputed: for each
   pair of incident ways, the (bearing_in, bearing_out) tuple in
   `[0, 2π)`.

2. **Detection logic** in `MapMatcher.kt`:
   - When the matcher is `STATE_ON_ROAD` AND the snapped position is
     within 5 m of an intersection node AND the recent EKF yaw rate
     `|ω_z|` ≥ 0.3 rad/s for ≥ 1 s (genuinely turning, not
     straight-through), AND the EKF yaw delta over the last 2 s
     matches one of the precomputed `(bearing_in, bearing_out)`
     deltas within ±15°:
   - Emit a yaw observation `yaw_meas = bearing_out_world` (the
     world-frame heading the user is now travelling along).

3. **Variance model** —
   `var_yaw = (1° in rad)² + (yaw_drift_2s_rad)²`. The first term is
   OSM way-bearing accuracy
   (https://wiki.openstreetmap.org/wiki/Accuracy); the second accounts
   for the 2 s window of yaw integration noise.

4. **Wiring** — fuse via the existing
   `EKFState::updateGravityAlignedYaw` (same channel the visual yaw
   correction uses, so all the FEJ/Huber/damping machinery proven on
   that path is inherited). New thin wrapper
   `EKFState::updateMapIntersectionYaw(yaw_meas, var_yaw)` enforces
   the additional `STATE_ON_ROAD` precondition and increments a
   dedicated counter for telemetry (`map_yaw_corrections_applied`).

5. **One observation per intersection passage** — the matcher remembers
   the last-fired intersection node id and refuses to re-fire for the
   same node within 30 s.

#### 8.G.3 Citation note

The closest published work to this framing is "turning-point-based map
matching" (Bian et al. 2021), which uses intersections to *segment*
trajectories rather than as direct yaw observations. Treating the
intersection geometry as a discrete EKF yaw measurement appears to be a
novel framing for NavSight; ADR-019 calls that out explicitly so
reviewers can verify it isn't an obvious bad idea missed in the
literature search.
**Source**: Bian, et al. (2021). A turning point-based offline map
matching algorithm for urban road networks. *Information Sciences*.
https://www.sciencedirect.com/science/article/abs/pii/S0020025521001948

#### Acceptance criteria

- On a labelled multi-intersection sim (the cited Haifa walk has ~10
  intersections), heading drift between consecutive intersection-yaw
  fires is bounded to ≤ 5°.
- Zero false-positive intersection fires on a straight-walk sim (no
  turn → no fire).
- Synthetic wrong-bearing test: inject an OSM intersection with a
  bogus bearing pair; the ±15° gate rejects.
- ADR-019 lands documenting this novel-to-NavSight observer.

---

### Step H — Mount-mode-aware routing graph

**Goal**: the routing graph the matcher uses must reflect the user's
physical mode. Pedestrian mode includes sidewalks/footways; scooter
mode prioritises cycleways and excludes pedestrian-only ways.

#### Full implementation plan

1. **Two pre-built routing profiles** in the GraphHopper preprocessing
   step (Step A):
   - `pedestrian`: `highway in {footway, path, pedestrian, steps,
     residential, tertiary, ...}` with `foot != no`.
   - `scooter`: `highway=cycleway` (highest priority) +
     `highway in {residential, tertiary, primary, ...}` with
     `bicycle != no`. Excludes `highway=footway` and
     `highway=pedestrian`.
2. **Runtime profile switch**: when `MountMode` (visual plan Step 10.1)
   changes, the matcher switches profiles. 500 ms warm-up while the
   new profile's R-tree is loaded; during warm-up the matcher emits
   `STATE_UNCERTAIN`.
3. **Sidewalks-on-roads handling**: the pedestrian profile snaps to
   the parent road and applies a geometric offset (1 m, side dependent
   on country drive-side) for **display only**. The EKF observation
   (Step F) goes to the road centerline — that's the metric the OSM
   geometry warrants.
4. **Scooter on sidewalk**: matcher tolerates but never prefers. A
   scooter rider on a sidewalk produces low scooter-profile confidence;
   if pedestrian-profile confidence on the same trajectory is high,
   log a `mount_mode_disagreement` event but stay with the user-
   selected mode (ADR-006: never silently override user choice).

#### Acceptance criteria

- On a labelled scooter sim recorded on a bike path parallel to a
  major road (`highway=trunk`), the matcher snaps to
  `highway=cycleway`, not the trunk road.
- On a labelled pedestrian sim crossing a public square
  (`highway=pedestrian`), the matcher snaps to the pedestrianised
  area, not the surrounding road network.
- Mount-mode switch (PEDESTRIAN ↔ SCOOTER) on a mid-sim event triggers
  profile swap within 500 ms with no EKF state shock.

---

### Step I — Replay harness extension + CI fixtures

**Goal**: build the labelling tool, replay-scorer extension, and CI
fixtures so the acceptance criteria of Steps C–H are mechanically
enforceable.

#### Full implementation plan

1. **Sim labelling tool** (`scripts/label_sim_road_truth.py`):
   - Reads sim JSON, renders VIO+GPS trajectory on a Folium map with
     OSM tiles, presents click-to-label UI for each VIO sample → "this
     sample is on way <id>" or "off-road".
   - Outputs `<sim>.road_truth.json` sidecar with per-sample road IDs.
   - Initial corpus: label `simulation_data_1778147132092.json` plus 2
     fresh outdoor walks and 1 indoor-mixed walk.
2. **Replay-scorer extension** (`scripts/replay_scorer.py`):
   - New metrics: `map_match_accuracy`, `map_match_off_road_recall`,
     `map_match_off_road_precision`, `map_match_state_transitions`,
     `map_match_position_drift_with_feedback_p95`,
     `map_yaw_corrections_applied`.
3. **C++ replay harness** (`tests/cpp/replay_harness.cpp`):
   - Add `--map-matching-enabled` flag.
   - When enabled, replays the sim through the matcher path the same
     way camera/IMU paths are replayed.
   - Outputs map-match decisions per sample to a CSV alongside the
     existing trajectory CSV.
4. **CI fixture set** in `tests/sims/regression/`:
   - 1× clean outdoor walk (>95% on-road).
   - 1× plaza-crossing walk (significant `landuse=plaza` traversal).
   - 1× indoor-outdoor mixed walk (building entry+exit).
   - 1× scooter ride on a labelled cycleway.
   - 1× scooter ride on a road with no cycleway.
   - Each fixture under git LFS to keep repo size bounded.
5. **CI workflow extension** (`.github/workflows/replay.yml`):
   - New job `replay-map-matching` runs the matcher offline on each
     fixture and asserts metrics from C–H acceptance criteria.

#### Acceptance criteria

- Labelling tool produces road-truth JSON for ≥ 5 sims.
- Replay scorer reports the new metrics.
- CI job passes within the existing 3-minute total CI replay budget.
- Per-step acceptance criteria from Steps C–H all pass mechanically
  against the fixtures.

---

### Step J — ADRs

This plan, when executed, produces three new ADRs:

- **ADR-018 — Map matching as VIO drift bound (soft EKF position
  observation)** (Step F). Documents the windowed shape-similarity
  emission, the variance model, the χ² gates, and the relationship to
  ADR-004. Additive — does **not** supersede ADR-004 (raw-GPS-into-EKF
  prohibition is intact); the matcher's input is laundered VIO, not
  GPS.
- **ADR-019 — Intersection-node bearing as a yaw observation** (Step
  G). Documents the apparently-novel-to-NavSight framing and the
  variance model. Replaces the **withdrawn** ADR-017 as the absolute-
  heading anchor mechanism. Companions ADR-001, ADR-005.
- **ADR-020 — On-device OSM data lifecycle** (Step A): regional
  extracts, preprocessing, update cadence. Documents the privacy
  posture (no telemetry of which regions a user downloads), storage
  caps, and user-controlled re-download cadence.

---

### Step K — Navigation to a specific destination (downstream feature)

**Goal**: per Morad 2026-05-07, NavSight will eventually let the user
say "navigate to a place I tell it." This sits on top of the same OSM
data layer (Step A), routing graph (Step A), and matcher (Steps B–H).

#### 8.K.1 What this step adds

1. **Destination input UX**:
   - Search bar (place name, address) backed by a local Nominatim-style
     index built from OSM `name`/`addr:*` tags during Step A
     preprocessing. On-device search; no server round-trip.
   - Tap-on-map alternative (any lat/lng).
   - Recent-destinations list (local, no cloud sync).
2. **Local-frame conversion**: destination lat/lng → local-frame
   metres relative to `SessionAnchor` via inverse haversine (mirror of
   Step B's projection).
3. **Path planning**: GraphHopper's existing Dijkstra/CH routing
   (`com.graphhopper.routing`) on the active mount-mode profile from
   Step H. Output: list of way segments + turn-by-turn instructions.
4. **Route adherence**: at each EKF tick, compute distance from the
   matched VIO position to the planned route polyline. If within
   threshold (mode-dependent: 10 m pedestrian, 15 m scooter), continue;
   if persistently off-route for ≥ 10 s, trigger re-route.
5. **Turn-by-turn cues**: voice + on-screen instruction at upcoming
   maneuvers ("Turn left in 50 m onto Ben Yehuda Street"). Distance-
   to-turn driven by EKF position projected onto the route polyline.
6. **Re-route**: when off-route for ≥ 10 s, call GraphHopper routing
   again with the current matched position as the new origin. Single
   new route published; old route geometry discarded.

#### 8.K.2 Constraints

- The route geometry is **display + cue only**; it does **not** feed
  the EKF. Step F's `updateMapPosition` continues to use the matcher's
  snapped point, not the route.
- Destination input must work entirely offline within the cached region.
- Step K depends on Step F + Step G being shipped and accepted (the
  matcher and intersection-yaw must be solid before turn-by-turn cues
  ride on top).

#### Acceptance criteria

- A destination search by name (e.g. "Tel Aviv University") resolves
  to a lat/lng on-device within 500 ms p95 over the Israel routing
  graph.
- A planned route from (Haifa Tel HaShomer) to (HaCarmel Market)
  produces a sensible turn-by-turn list within 2 s p95.
- On a re-walked sim where the user takes a different turn than
  planned, the re-route fires within 10 s of going off-route and
  produces a new valid route.
- The route polyline is rendered on the Compose map in a distinct
  colour from the matched-position polyline.

---

## 9. Acceptance criteria for the entire plan

A NavSight build with Steps A–J complete should achieve, on the Step I
fixtures:

**Pedestrian (cited Haifa walk + outdoor + plaza fixtures):**
- **Visible off-road drift**: zero on-screen artefacts — the rendered
  blue dot follows roads/footways for ≥ 95% of sustained-motion
  samples; hides cleanly during indoor/plaza segments.
- **Endpoint position error vs GPS-confirmed return**: ≤ 10 m on the
  cited 8-min Haifa walk (today: 70+ m; with today's heading fix
  alone, expected < 30 m; with this plan: ≤ 10 m).
- **Heading RMSE through intersection-rich routes**: ≤ 3° (today:
  5–10°; visual plan + heading fix: ~2°; this plan adds the
  intersection-anchor bound).
- **No state shock**: bit-identical EKF behaviour when matcher is
  `STATE_OFF_ROAD`.

**Scooter (cycleway + road fixtures):**
- **Mode-correct snapping**: scooter rides on labelled cycleways snap
  to `highway=cycleway`, not the parallel `highway=trunk`.
- **Drift per kilometre on bike-path ride**: ≤ 5 m/km of position
  drift over a 1 km city-block route.

**Both:**
- **CI replay-map-matching job**: passes within the existing 3-minute
  budget with all acceptance metrics within thresholds.
- **APK size delta**: ≤ 30 MB compressed (Step A acceptance gate).
- **Cold-start time impact**: ≤ 200 ms added to first-launch (Step A
  index loading).

Step K (navigation) acceptance is independent and listed under §8.K.

---

## 10. Risks and mitigations

| Risk | Severity | Mitigation |
|---|---|---|
| **Confidently-wrong snap when initial drift is large (>30 m)**: matcher locks onto wrong road, then EKF feedback pushes the filter toward that road, which makes the snap "more confident" on subsequent samples. Same class of failure ADR-004 was written against, now possible via the matcher channel. | **CRITICAL** | Step E χ² gate on EKF observation rejects implausible feedback. Step G intersection-anchor cross-checks the matcher's road claim against geometric turn evidence. Hysteresis on `STATE_ON_ROAD` (3 consecutive samples) prevents single-sample lock-in. Visual loop closure (ADR-013) provides an independent corrector. The shape-similarity emission is offset-invariant, which structurally reduces the lock-in pressure: a constant drift offset that would lock a per-sample matcher leaves shape similarity unchanged. |
| **Shape-similarity collapses on long straight roads** (any 30 s window of straight motion has near-zero shape information; many parallel candidates will look equally good) | **HIGH** | Step D's transition probability disambiguates across the full session, not just within the window — a long-straight ambiguity will collapse the moment the user turns. Until then, Step E's confidence stays low (multiple candidates with near-equal emission) and the matcher publishes `STATE_UNCERTAIN`, no EKF feedback. Acceptance is about not being *wrong*, not about always being *right*. |
| **GraphHopper APK size > 30 MB** | HIGH | Step A's spike is the gate. Fall-back path (hand-rolled R-tree on a vendored OSM PBF parser) is documented up front. |
| **Discrete Fréchet computation > 50 ms / sample** | HIGH | Resample VIO inside the window at 5 Hz (~150 samples per 30 s); use approximate Fréchet (https://arxiv.org/abs/1404.1448). Fallback: switch to shape-DTW with local descriptors. |
| **OSM data stale (newly built road not in OSM)** | MEDIUM | `osm_data_age_days` factor in Step E confidence. User-facing "Last updated" indicator in Step A region manager is the manual escape hatch. |
| **Dense urban canyon with parallel roads inside VIO drift envelope** | MEDIUM | Shape-similarity over a 30 s window is strictly better than per-sample for this case (parallel roads have different shapes when one curves and the other doesn't). Step E hysteresis prevents committing on the first 2-3 ambiguous samples. |
| **Pedestrian off-road > 30% of session time** | MEDIUM | Acceptable degradation. Off-road segments fall back to visual loop closure + EKF — the same stack as today. Matcher is strictly additive (Principle 11). |
| **Intersection-bearing observer (Step G) is novel framing** | MEDIUM | Step I's labelled multi-intersection sims are the regression. ADR-019 review must explicitly walk through "what if a 4-way intersection has bearings 0°/89°/180°/271°?" — small bearing perturbations should not flip which pair gets matched. |
| **Privacy: which OSM regions the user downloads is sensitive (reveals travel)** | LOW | Step A: no telemetry of region downloads. Local-only. ADR-020 documents the rule. |
| **Scooter on a sidewalk + scooter profile** → matcher silent-disables for the entire ride | LOW | `mount_mode_disagreement` event logged; user can switch profile. Auto-switch on sustained disagreement is future work (open question 7). |
| **VIO scale bias** moves the entire VIO stream uniformly slow/fast → window arc-length disagrees with route arc-length → transition probability collapses | LOW–MEDIUM | The scale fuser (inertial Step 3) bounds scale to ±10–20%. Step D's transition `beta = 2.0 m` accommodates that range. Larger biases trip the confidence gate (`transition_consistency` low) → silent-disable. |

---

## 11. Open questions for Morad

Before this plan kicks off, please answer:

1. **GraphHopper-vs-spike-vs-bailout decision authority.** Step A's
   acceptance gate (≤ 30 MB APK delta) decides whether we go
   GraphHopper (Java, easy) or have to hand-roll a spatial index
   (longer, riskier). Are you OK with the spike outcome being the
   binding decision, or do you want a review checkpoint before the
   Step A team starts integration?

2. **Default region.** Israel+Palestine combined extract is the
   obvious default for shipping. Is that right, or should the first
   build be Haifa-only (smaller PBF subset, faster preprocess, but
   the user becomes "trapped" the moment they leave the city)?

3. **EKF feedback default-on or default-off for v1?** Step F's
   `updateMapPosition` is the most consequential single change. We
   can ship Steps A–E as display-only (matcher visible on the map,
   never feeds the EKF) for one release, then enable Step F in a
   follow-up after on-device replay confirms no surprise failure
   modes. Alternatively, we can ship A–F together gated behind a
   strict `final > 0.7` threshold from day one. Which risk profile do
   you prefer?

4. **Intersection-anchored heading (Step G): novel framing — green
   light or literature spike first?** We could not find a paper that
   does *exactly* this. Either it's a real gap (cool — first to ship
   it on a phone) or we missed something. Want a one-week dedicated
   spike to either find prior art or convince ourselves there isn't
   any, before Step G kicks off?

5. **Window length for the shape-similarity emission (Step D).** 30 s
   is the proposed default — long enough for a 30-sample-per-second
   trajectory to have non-trivial shape, short enough to bound decode
   lag. Pedestrian at 1 m/s covers 30 m in a window; scooter at 5 m/s
   covers 150 m. Are those window sizes the right unit? (Alternative:
   distance-based window of 50 m for both modes, which inverts the
   relationship.)

6. **Off-road CI fixture coverage.** Step I proposes 5 fixtures.
   Realistically the off-road / plaza / indoor regimes are where this
   plan's risk concentrates. Are you OK with a target of 10 labelled
   sims (5 outdoor pedestrian + 2 scooter + 3 mixed/indoor) by Step I
   completion, with new sims added as Steps F/G roll out?

7. **Mount-mode auto-switch on map-matcher disagreement.** Step H logs
   `mount_mode_disagreement` but does not auto-switch (per the ADR-006
   user-choice invariant). Down the line, if telemetry shows users
   routinely forget to switch from pedestrian to scooter mode, do we
   add an auto-switch suggestion (UI prompt) or auto-switch silently?
   Open product decision.

8. **Step K (navigation) sequencing.** Step K is downstream and
   substantial. Do you want it scoped as part of this plan (executed
   after Step J) or split out as `docs/NAVIGATION_PLAN.md` once A–J
   are accepted? My preference: keep it here as Step K so the OSM
   data layer's design is informed by both consumers, but execute it
   strictly after A–J ship.

---

## 12. Things deliberately NOT in this plan

- **Cross-session persistent matcher state** (caching matcher state
  across app launches beyond the routing data itself).
- **Free-space movement graph for plazas/parks** (Yan et al. 2024 —
  off-road silent-disable is the cheaper answer for v1).
- **Custom ORB-style descriptor for road appearance** (camera-frame
  cross-check of OSM geometry; massive scope, defer).
- **Indoor map matching** (no indoor map data; matcher silent-
  disables. Indoor positioning is a separate research project).
- **Crowd-sourced sidewalk corrections** (users cannot edit OSM from
  the app; ADR-020 may or may not allow telemetry-driven map quality
  signalling — separate decision).
- **GNN / transformer matchers** (deferred until standard HMM path
  proves insufficient).
- **Raw GPS in the EKF** (forbidden by ADR-004; this plan does not
  erode that rule).
- **Multi-session map / crowdsourced geometry refinement** (privacy +
  scope, defer).