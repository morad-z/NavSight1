# NavSight Map Matching Plan

**Status**: draft — **REVISED 2026-05-29** (scale-first refocus + Google→free migration; see §0)
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

## 0. Revision 2026-05-29 — scale-first refocus + Google→free migration

This revision does two things: **(A)** refocuses the plan from *heading drift* (the problem when
the plan was drafted — now fixed, commit `4a9a212`) to the *metric-scale collapse* that now
dominates; and **(B)** reframes the plan as the **migration that rips out the paid Google APIs
and replaces them with the free, offline OSM stack this plan already designs.** §1–§12 below stand
**except where corrected here** — read §0 first; it overrides the body on every point it touches.
This was reviewed by a multi-lens + adversarial pass; the corrections below already incorporate the
adversary's pushback (notably: do NOT use per-candidate free-scale Procrustes — see §0.3 M2).

### 0.0 Branch & merge strategy — READ FIRST if you're on the `osm-migration` worktree

This migration is developed on a **separate branch + worktree** so it never tangles with the in-flight
speed work. Both branches are on GitHub (`origin`).

| Branch | Folder | Owns |
|---|---|---|
| `morad` (trunk) | `C:\Users\morad\AndroidStudioProjects\NavSight1` | the SPEED work (Fix A/B/C) + the VIO **C++ core** |
| `osm-migration` | `C:\Users\morad\AndroidStudioProjects\NavSight1-mapmatch` (worktree off `morad`) | **THIS migration** (Kotlin / data-layer / build) |

The two touch nearly **disjoint files** — migration = `RoadSnapper.kt`, `NavigationManager.kt`, `SearchBarUi.kt`,
`build.gradle.kts`, new OSM files; speed = `Tracker.cpp` / `EKFState.cpp`. **Rules for the migration session:**

1. **Do the speed-INDEPENDENT migration half** (§8M Steps A\*/B/C/K-routing/K-search, + L if wanted): replace
   the 3 paid Google APIs (Roads/Directions/Places) with free OSM. Needs no speed fix.
2. **Do NOT edit the C++ speed core** (`Tracker.cpp`/`Tracker.h`/`EKFState.cpp`) on this branch — those are
   changing on `morad` (Fix A/B/C). The ONLY allowed engine touch is the read-only Step B `current_vio_lla()`
   accessor; keep it minimal + additive so the later merge is conflict-free.
3. The **scale-dependent matcher** (Step D HMM, K̂ estimator, Step E confidence, Step F EKF feedback) stays
   **DEFERRED** until the speed (Fix A/B/C) is validated on `morad` and merged — a scale-collapsed dot can't be
   arc-length-matched (§0.3 M1, §0.5). Build those AFTER the merge, on the correct-scale base.
4. **Commit/push to `osm-migration` only** — plain `git commit` / `git push` in this folder; never name `morad`.
5. **Merge order (later, from the `morad` folder):** validate speed on `morad` → `git checkout morad &&
   git merge osm-migration` → then build the deferred scale-dependent matcher on the merged base →
   `git worktree remove ../NavSight1-mapmatch`.
6. **Memory note:** this worktree is a different folder, so it does **not** auto-load the NavSight project
   memory at `~/.claude/projects/C--Users-morad-AndroidStudioProjects-NavSight1/`. **This plan (§0 + §8M) is the
   authoritative context — read both before starting.**

### 0.1 Corrected headline

The plan's bones are sound and still 2026-correct: the Goh online sliding-window HMM (§4.3),
offset-invariant emission to survive correlated VIO drift (§8.D.1), display-only-before-EKF-feedback
phasing (§11 Q3), off-road silent-disable (Principle 11), covariance-mandatory soft observation (§8.F).
**Keep all of it.** But the live failure is no longer heading — it is **metric scale**: the user-facing
dot (`Tracker::global_t_`, advanced by `disp = K·speed_rel`) reads **~0.12× cold / ~0.30× warm-K** of
true distance (3–8× under-read). **Honesty caveat (the user's own steer):** that magnitude is from the
offline replay harness only (grayscale frames, ~18.7 fps, harness skips `reset()`/`WAIT_STATIONARY`, cold
K=−1) and is **not yet on-device-confirmed** — the *direction* (scale under-reads) is well-supported by
months of walk data; the *magnitude* awaits an on-device logcat. Turn **angles/shape are correct**
(heading ~2–3°); only metric scale is broken.

### 0.2 Google → free migration (NEW — this is the cost driver)

Found in the current code. Google **Maps SDK map loads are free on Android**; the money is **Roads API +
Directions API + Places API**. We migrate ALL of it for $0 + offline + jamming-immunity (no key, no network).

| Google component | File(s) | Google API | Cost | Free replacement | Maps to |
|---|---|---|---|---|---|
| Map display | `MapScreenUi.kt` (`GoogleMap`, `CameraPosition`) | Maps SDK | free* but online + key-coupled | **MapLibre Native** (vector, offline MBTiles/PMTiles) or **osmdroid** (raster, simplest) | NEW **Step L** |
| Road snapping | `RoadSnapper.kt` (`RoadsApi.snapToRoads`, ~2 Hz @ `NavSightViewModel:487`) | Roads API | **$$ / req** | the **OSM HMM matcher** — `RoadSnapper.snapToRoad()` body becomes the local snap; delete the `GeoApiContext`/Roads call | **Steps C/D** |
| Routing | `NavigationManager.kt` (`DirectionsApi`) | Directions API | **$$ / req** (now disabled → straight-line fallback @ `:216`) | offline router: **BRouter** (FOSS, purpose-built offline-Android) or **Valhalla-mobile** | **Step K** |
| Dest. search | `SearchBarUi.kt` + `places-client` | Places API | **$$ / req** | offline **Nominatim-style index** over OSM `name`/`addr:*` | **Step K.1** |
| `LatLng` data type | 7 files (MapScreenUi, NavigationManager, NavSightUtils, NavSightViewModel, RoadSnapper, SearchBarUi, SensorRepository) | `gms.maps.model.LatLng` | coupling | neutral `data class GeoPoint(lat, lng)` (or typealias) | **Step 0.x / L** |

**Recommended v1 = HYBRID: keep the Google display, swap only the 3 PAID APIs.** The display is free and
decoupled, so keeping `GoogleMap` (and the `gms LatLng` type) is the leanest path — it avoids both the display
rewrite **and** the `GeoPoint` refactor across 7 files, and removes the entire bill (Roads + Directions +
Places are the only paid surfaces). Caveats of keeping the Google display: (1) still needs a Maps key with a
billing account attached (loads are free, but the account must exist); (2) the **basemap tiles need network** —
in a jammed/offline area the background goes blank/stale while the VIO dot + OSM snap + offline router keep
working on embedded data; (3) cosmetic: the OSM-snapped dot may sit ~1–2 m off Google's rendered road (different
surveys). **Therefore Step L (display → MapLibre/osmdroid) is OPTIONAL / deferred** — do it only when a fully
offline, no-key, jam-proof basemap is wanted. v1 keeps Google display + free OSM matcher/router/search.

**"Make routing work again"** = NavigationManager currently falls back to a straight line because the
Directions key is blank (`:57` "routing will be disabled", `:216` `calculateFallbackRoute`). The fix is
**not** to restore the paid key — it is to wire an **offline router** (BRouter/Valhalla) so routing works
with **$0, offline, and jam-proof**. Remove `places-client`, `google-maps-services`, and the
`com.google.android.geo.API_KEY` manifest placeholder once L/C/D/K land. Keep `play-services-location`
only if the **one** bootstrap GPS fix (SessionAnchor, §8.B) still uses it (see SU8 — add a no-fix fallback).

### 0.3 MUST-FIX corrections to the body (these override §1–§12)

- **M1 — §10 risk "VIO scale bias": LOW–MEDIUM → HIGH / precondition.** The text "scale fuser bounds to
  ±10–20%; β=2.0 accommodates" is false: the affine/scale-fuser path is forced **off** the speed path
  (the "5× poison"), and the error is multiples, not ±20%. At collapsed scale the transition term is
  ~10σ-rejected and the emission loses its discriminative *margin*. (Magnitude replay-only — confirm
  on-device before fixing β.) **It is a precondition, not a tolerated nuisance.**
- **M2/M3 — §8.D.1/§8.D.2 scale handling.** Make the matcher scale-robust with a **single, bounded,
  shared `K̂`** applied uniformly to all candidates, via a **log-ratio transition kernel**
  `exp(−|log(d_route /(K̂·d_vio))| / β_log)`. **Do NOT** use per-candidate free-scale Procrustes/Umeyama —
  it discards the arc-length prior and *amplifies* confidently-wrong snaps (a curvy side-street rescales
  itself to fit). Keep the existing mean-offset subtraction (it's correct; compose it under the shared K̂).
- **M4 — §1 dependencies.** Heading-fix dep = **MET** (commit `4a9a212`: gimbal-free RV + gyro-primary
  gated compass — not the cited `EKFState.cpp:1052/1097/1217`). Step-7 loop-closure dep = **PARTIAL**
  (`loop_closure_corrections_applied>0` but `loop_closure_geom_accepts=0`, `slam_promotions_total=0`).
  **ADD** a dependency: *"dot metric scale validated within ~±20% of truth on a real walk (Fix A/B landed)."*
- **M5 — §10 "confidently-wrong" mitigation** must not lean on loop closure as the independent corrector
  (geom closures = 0 today). Lean on χ² gating + hysteresis + Step-G cross-check instead.
- **M6/M7 — §2 motivation + the primary fixture.** Rewrite §2 around scale; the fixture
  `simulation_data_1778147132092.json` is the **pre-fix 75° walk** and its acceptance criteria (notably
  Step B's "~75° reduction", §9 baselines) are dead. Re-record the primary fixture on the post-`4a9a212`
  engine; the `1779643969824` 134 m loop is a candidate.
- **M8 — §8.I** the replay harness **already exists** (`tests/cpp/replay_harness.cpp`, mingw+Ninja
  `build_mingw/`; extended with traj/speed CSV cols + `--depth-dir` + `--seed-k`). "Add the flag," not
  "build it" — and note its fidelity caveats so map-match acceptance isn't over-trusted vs replay.
- **M9 — §5.1 GraphHopper offline-Android is DEPRECATED** (Android demo removed v2.0; offline routing
  deprecated; F-Droid app is now online-only). For routing prefer **BRouter** or **Valhalla-mobile**
  (Step K); for the matcher data layer, a **hand-rolled R-tree over a Haifa OSM extract** is the lean v1.
- **M10 — drop the PEDESTRIAN/SCOOTER mount-mode split** (§7, §8.D.4, §8.H) — contradicts the
  locomotion-agnostic product decision. Single v1 profile (foot ∪ cycleway ∪ road).

### 0.4 NEW opportunity — Step D.5: matcher-as-scale-estimator (`K̂_map`)

The single highest-leverage upgrade. Over a matched window the matcher yields `d_route` (matched road
arc-length) and `d_vio` (VIO arc-length); their ratio **`K̂_map = d_route / d_vio` is a direct, GPS-free,
ZUPT-free observation of the VIO scale K** — exactly the anchor accel-K fails to produce
(`depth_flow_calib_updates=0`) and that ScaleEstimatorVI was abandoned for. Emit `K̂_map` (log-EMA,
gated on top-state prob > 0.5 + not-OFF_ROAD + > 10 m motion + **hard road-identity confidence**), feed it
into `Tracker::updateDepthFlowSpeed` above accel-K with the existing >3× blow-up guard; counters
`map_scale_k_milli`, `map_scale_calib_updates`. **Caveats (load-bearing):** (i) it is **downstream of
Fix B** — `d_vio` is corrupted by the same verification-gating bug (only ~28% of frames advance), so the
ratio mixes scale error with sampling error until Fix B lands; (ii) a mis-identified parallel road poisons
K and **diverges** (the ADR-004 loop) → identity-confidence gating is mandatory, not optional; (iii) it is
most reliable on **turn-rich** windows and weakest on long straights (where the transition term most needs
it). So: a strong *opportunistic* anchor, complementary to Fix A (fuse in log space) and orthogonal to
Fix B — **not** a closed-loop guarantee or a substitute for fixing scale in the engine.

### 0.5 Resequenced lean v1 ("a dot that stays on the road at roughly-right distance")

```
PREREQ  Fix A (accel-K self-calibration) + Fix B (per-frame dot advance)  — NOT in this plan; the real blocker.
        Gate: a real walk shows dot metric scale within ~±20% of truth.  (Confirm via on-device logcat.)
        Fix C (TURNS, see §0.7) — narrow the `rotation_dominated` freeze (Tracker.cpp:3562) to gate on a real
        translation signal, not PDR step_speed. Required before any turn (roundabout/U-turn) renders as an arc.
 0   Re-baseline this doc (M1/M4/M5/M6/M7 edits).
 B   Tracker::current_vio_lla  (read-only, ~50 LOC).  Acceptance rewritten (drop the 75° claim).
     (v1 keeps the Google display + gms LatLng — no GeoPoint refactor needed for the hybrid.)
 A*  LEAN data layer — Haifa-only OSM extract + hand-rolled R-tree over ways.  Defer GraphHopper/Valhalla/BRouter
     decision and the region manager until Step K actually needs routing.
 C   Snap-to-nearest, display-only — REPLACES RoadSnapper.kt's Google Roads call.   ← v1 user-visible payload.
 E*  LITE confidence + off-road silent-disable (snap-distance + VIO-variance + LC-recency; keep silent-disable).
 I*  PARTIAL — extend the EXISTING harness; labelling tool; 1–2 fixtures (not the 5-corpus).
        ── v1 ships here: free display + free road-snapping, $0, offline. ──
 D + D.5   Windowed shape HMM with shared-K̂ scale handling (M2) + the K̂_map scale estimator (§0.4).
           Only after scale is fixed AND if Step C mis-snaps on parallel roads in practice.  Distance-based 50 m window.
 F   Soft EKF position observation — only after on-device replay shows no self-reinforcing lock-in; default-OFF.
 K   ROUTING + SEARCH MIGRATION — offline router (BRouter/Valhalla) replaces Directions API ("routing works again");
     offline Nominatim-style index replaces Places API.   [the other half of the Google→free migration]
 L*  OPTIONAL/DEFERRED — DISPLAY swap GoogleMap → MapLibre/osmdroid + neutral GeoPoint. Only for a fully-offline,
     no-key, jam-proof basemap. v1 KEEPS the Google display (free; the bill was Roads/Directions/Places, now gone).
 —   CUT from v1: Step G (intersection-yaw) — heading is fixed; cutting on PRIORITY grounds (not "heading solved
     forever": 2.5° over 1 km ≈ 44 m cross-track, but the matcher absorbs it). Keep G named as the heading-re-drift
     backstop, since the geometric LC that would otherwise catch re-drift is currently dead (geom_accepts=0).
 —   SPLIT OUT: the rest of Step K's turn-by-turn navigation → docs/NAVIGATION_PLAN.md once a matched dot ships.
```

### 0.6 Updated answers to §11 open questions

1. **GraphHopper authority** — moot for v1 (defer routing to Step K); GraphHopper offline-Android is
   deprecated (M9) — choose BRouter/Valhalla when routing is actually on the table.
2. **Default region** — **Haifa-only** for v1 (smaller/faster; it's the test+deploy city).
3. **EKF feedback default on/off** — **display-only (A–E), Step F OFF** — reinforced by scale collapse
   (don't feed a scale-inconsistent snap into the EKF).
4. **Step G** — **CUT from v1** on priority grounds; keep as the heading-re-drift backstop.
5. **Window length** — deferred with Step D; prefer the **distance-based 50 m** window (arc-length is what
   scale distorts, so a distance window is more robust to residual scale error).
6. **Off-road fixtures** — **2 for v1** (1 clean outdoor loop + 1 indoor/plaza), grow later.
7. **Mount-mode auto-switch** — moot (locomotion-agnostic; no mount mode in v1).
8. **Step K sequencing** — turn-by-turn navigation splits into `docs/NAVIGATION_PLAN.md`; but the
   **offline routing + search engines** (the Google Directions/Places replacement) stay here as Step K
   because the user wants routing working again as part of this migration.

### 0.7 Turn handling — roundabouts & mid-road U-turns (code-verified 2026-05-29)

Split **DETECTION** (did the user turn / which way) from **RENDERING** (the shape drawn on the map):

- **DETECTION works for both.** Gyro-primary heading (`Tracker.cpp:3491`) tracks a 270–360° roundabout sweep and
  a 180° U-turn flip. *Caveat:* gyro **scale-factor** error (~1–3% of swept angle ≈ 4–11° on a full roundabout)
  is now **un-anchored** since Step G (intersection-yaw) is cut — bias drift over the sweep is fine, scale-factor
  is the small residual heading risk; the matcher absorbs it on entry/exit.
- **RENDERING is NOT correct yet** for either maneuver — gated on **three** fixes, in order:
  1. **Fix C — the `rotation_dominated` translation freeze (the binding bug, code-verified).**
     `rotation_dominated = gyro_norm > 0.2 rad/s && PDR step_speed < 0.1 m/s` (`Tracker.cpp:3549-3550`);
     `if (is_static || rotation_dominated)` (`:3562`) skips the **sole** per-frame advance site (`:3737-3739`).
     A turn sweeps at 30–90°/s and PDR `step_speed` is structurally ~0 for **non-pedestrian/scooter** motion →
     the gate is true for the **whole** maneuver → the dot **freezes position, pivots in place, then teleports to
     the exit**. The matcher then sees *two clusters + a teleport, not an arc* — so Fix A/B and Step D cannot
     render the turn until this is fixed. The gate exists for a real reason (phone-in-chair "looking around" =
     50 m phantom, 2026-05-17), so fix the **root cause**: re-gate on a real translation signal (accel speed /
     looming forward-fraction), NOT PDR `step_speed`. Do **not** remove it.
  2. **Fix A/B** (scale + advance every frame) — else the roundabout radius / U-turn legs are ~⅓ size.
  3. **Step D, not Step C** — snap-to-nearest **corner-cuts** a roundabout to the chord (and is CW/CCW
     ambiguous); the HMM + routing-distance transition follows the ring, but only once scale + Fix C give it a
     real arc to match.
- **ROUNDABOUT: NO today → YES after {Fix C + Fix A/B + Step D}.** Step C alone never renders it. Even with Step D
  the entry/exit transition is numerically twitchy on the multi-short-arc one-way ring (CW/CCW flip risk).
- **MID-ROAD U-TURN: the genuine blind spot.** Detection = VIO/gyro (yes). The matcher does **not** detect the
  reversal — it inherits whatever VIO drew, and can make it **worse**: Step C flickers the dot between the two
  lane/segment directions (bearing-blind, per-sample); Step D can **node-snap the reversal to the nearest
  junction** (fabricating a detour to an intersection and back). v1 has no reverse-candidate / heading-aware
  emission to localize the reversal. So for U-turns: **trust VIO**; the matcher's best case is "keep it on the
  road," worst case it relocates the turnaround. (Step G being cut has no effect — a mid-block U-turn has no
  intersection node anyway.)

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

> ⚠ **REVISED 2026-05-29 (see §0.1):** this section's heading-drift motivation is **obsolete** — the
> heading sign bug below was fixed and committed (`4a9a212`); real-walk heading drift is now ~2–3°. The
> live reason this plan exists today is **metric-scale collapse** (the dot reads ~0.12–0.30× of true
> distance) plus replacing the paid Google Roads/Directions/Places APIs with a free offline OSM stack.
> The `…1778147132092` walk below is the *pre-fix* 75° recording — keep it only as a heading-regression
> archive, not as the acceptance fixture.

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

> ⚠ **REVISED 2026-05-29 (see §0.3 M9):** GraphHopper's **offline-Android path is deprecated** — the
> Android demo was removed in v2.0 and offline routing is no longer officially supported; the F-Droid
> `com.graphhopper.maps` app is now an **online** planner. Do **not** assume the "F-Droid ships an Android
> binary so the path is proven" claim below. For **routing** (Step K) prefer **BRouter** (FOSS, offline-
> Android) or **Valhalla-mobile**; for the **matcher data layer** (lean v1) use a hand-rolled R-tree over a
> Haifa OSM extract (§0.5 Step A*). Re-spike before committing to GraphHopper.

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
| **VIO metric-scale collapse** — the dot reads ~0.12–0.30× of true distance (3–8× under-read; replay-measured, on-device magnitude unconfirmed). The window is the right *shape/angles* at ~⅓ size. **REVISED 2026-05-29** (was rated LOW–MEDIUM "±10–20%, β=2.0 accommodates" — both false: the affine/scale-fuser path is OFF the speed path, so even ±20% no longer holds). | **HIGH / precondition** | This is **not** a tolerated nuisance — at collapsed scale the offset-only Fréchet emission loses its discriminative *margin* (→ confidently-wrong snaps on parallel roads, the §10 row-1 CRITICAL failure) and the `β=2.0 m` transition term is ~10σ-rejected. **Fix:** (1) gate the arc-length-dependent parts (transition, straight-road disambiguation) on **Fix A/B** restoring scale to ~±20%; (2) make the matcher scale-robust with a single shared bounded `K̂` via a log-ratio transition kernel (§0.3 M2 — NOT per-candidate free Procrustes); (3) turn-topology *identity* is scale-free, so display-only snapping (Step C) can run before scale is fixed. Bonus: `K̂_map = d_route/d_vio` becomes a scale *estimator* (§0.4). |

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

---

## 8M — Migration step bodies (execution-ready 2026-05-29)

> These step bodies are the **execution-ready** rewrite of the migration half of §8 (the
> Google→free OSM swap). They **supersede the original §8 A / C / K where they differ** — the
> original §8 is left intact above as the historical superset (it assumed GraphHopper + per-mode
> mount profiles + EKF feedback, all of which §0 corrected). When the two conflict, **8M wins**.
> Scope of 8M = the **speed-independent, cost-saving** half only: replace the 3 PAID Google APIs
> (Roads, Directions, Places) + build the OSM data layer they need. It does **NOT** include the
> scale-dependent matcher (Step D HMM, K̂ estimator, Step E confidence, Step F EKF feedback) — those
> stay in the original §8, DEFERRED behind the speed fixes (Fix A/B/C, §0.7) and out of scope here.
> Fixed decisions (do not re-litigate): HYBRID v1 keeps the Google display + the `gms LatLng` type
> (no `GeoPoint` refactor, no display rewrite); Step L is OPTIONAL/DEFERRED; GraphHopper offline-Android
> is OUT (M9); region is **Haifa-only**; snap-to-nearest (Step C) is a like-for-like free replacement of
> the current Google Roads snapper (per-point, corner-cutting accepted for v1).

### Step A* — OSM data layer (lean, Haifa-only)

**Goal**: produce, from a Haifa-only OSM extract, the three in-memory structures the migration
consumes — (1) a **road-segment R-tree** for snapping (Step C), (2) a **routing graph** for Step
K-routing, (3) a **name/addr geocode index** for Step K-search — built by a **dev-machine
preprocessing pipeline** whose compact output ships as an **APK asset** and loads into RAM on first
launch. **No GraphHopper** (M9); a **hand-rolled PBF→structures preprocessor** plus a **hand-rolled
R-tree + adjacency graph** for snap/geocode, with **BRouter** (`brouter-core`, MIT) as the embedded
offline router fed its own `rd5` segment file. End state: Haifa data is embedded, the three
structures are live, and first-launch load is bounded and measured. **This step is purely additive
and read-only on the EKF** — it produces data and a query API; it does not touch `Tracker`/`EKFState`.

#### A*.0 Decisions locked for v1 (no re-deciding downstream)

- **Extract source**: **Geofabrik `israel-and-palestine-latest.osm.pbf`** (~115 MB, daily) is the
  upstream; there is **no Haifa sub-region on Geofabrik**, so the dev pipeline **clips to a Haifa
  bbox locally**. Clip via **`osmium extract`** with a polygon/bbox (preferred — reproducible, scriptable)
  or **BBBike** (`extract.bbbike.org`, custom rectangle/polygon) as the manual fallback. Haifa bbox v1:
  `lon 34.94…35.10, lat 32.74…32.86` (Haifa municipality + Carmel + downtown + the test loops; widen later
  if a walk leaves it). Record the exact bbox + the upstream PBF date in the asset manifest (A*.4).
- **PBF parse (dev machine, NOT on device)**: **`osmpbf`** (Java, `org.openstreetmap.pbf:osmpbf`,
  low-level protobuf block reader) wrapped by our own way/node walker — OR **OSMonaut** (Java, returns
  complete entities, low-memory mode) if we want complete-entity convenience. **Pick `osmpbf`** for v1:
  the clipped Haifa PBF is small (≈3–6 MB), we only need a single forward pass emitting our own compact
  binary, and `osmpbf` has the smallest dependency surface. OSMonaut is the documented fallback if
  two-pass node-resolution becomes annoying. **Neither library ships in the APK** — they are
  build-time only (the preprocessor is a desktop Gradle/`:tools` task), so they add **0 bytes** to the APK.
- **R-tree (on device + at preprocess)**: **hand-rolled**, ~250 LOC. A static, **bulk-loaded
  packed Hilbert R-tree** (STR / sort-tile-recursive packing) over way-segment AABBs — read-only after
  build, no insert/delete needed, so packing beats a dynamic R-tree on both build time and query speed.
  Rationale for hand-rolling vs vendoring (e.g. JTS `STRtree`): JTS is a ~1.5 MB jar dragging in a full
  geometry suite we do not need, and we want the leaf payload to be our `segment_id` + precomputed segment
  endpoints (for the perpendicular-distance projection in Step C) rather than generic `Geometry`. Hand-roll.
- **Routing engine**: **BRouter `brouter-core`** (MIT, Java, offline-Android-native, embeddable
  in-process — modules `brouter-core` / `brouter-codec` / `brouter-mapaccess` / `brouter-expressions`
  are separable from the Android-Service app). **Chosen over** (a) **Valhalla-mobile** — C++/NDK build of
  a large third-party project, tile data 100 MB+ even for a city, real engineering cost (§5.2); and
  (b) **hand-rolled A*/Dijkstra over our own adjacency graph** — viable and ~150 LOC but re-implements
  turn restrictions, one-way handling, foot/bike access rules, and weighting that BRouter's `.brf`
  profiles already encode and that we'd otherwise hand-maintain. **BRouter wins**: $0, offline, MIT,
  proven on Android, profile-driven (one `foot`-∪-`bike`-∪-`road` profile per M10, no mount split), and
  its `rd5` data is produced from the **same** clipped Haifa PBF by **`brouter-map-creator`** on the dev
  machine. **Cost note**: `brouter-core` + `brouter-codec` + `brouter-mapaccess` + `brouter-expressions`
  jars ≈ **2–4 MB** added to the APK (measure in A*.1 gate); the Haifa `rd5` segment slice ≈ **2–8 MB**
  (BRouter ships 5×5° world tiles at tens of MB each, but a Haifa-clipped single-tile slice produced by
  `brouter-map-creator` from our bbox is small). If the jar+data gate (A*.1) is blown, **fall back to the
  hand-rolled A\*/Dijkstra over the routing graph** (structure (2) already exists for that purpose) and
  drop BRouter — an ADR records the choice.
- **Geocode index**: **hand-rolled** inverted index (token → list of `place_id`) over OSM
  `name` / `addr:street` / `addr:housenumber` / `addr:city` tags on nodes + way centroids, plus a flat
  `place_id → (lat, lng, display_name)` table. No Lucene (multi-MB jar, overkill for one city). Substring +
  prefix match with simple ranking (exact-name > prefix > token-contains, ties broken by closer-to-anchor).
- **Asset shipping**: **dev-machine preprocess → embed compact binary as APK asset** (NOT on-device
  PBF parse). On-device parsing of even a clipped PBF means shipping a parser, holding the protobuf
  blocks + the full node coordinate map in RAM during a cold start, and a multi-second first-launch
  stall — all avoidable. The dev pipeline emits **mmap-friendly little-endian binary blobs**; first
  launch memory-maps / streams them straight into the three structures. **No `.osm.pbf` is shipped in
  the APK.**

#### A*.1 Dev-machine preprocessing pipeline (`:tools` desktop module + `scripts/`)

A new Gradle desktop module `tools/osm-preprocess/` (JVM `application`, NOT shipped) plus a thin shell
wrapper `scripts/build_haifa_assets.sh`. Stages, each idempotent and logged:

1. **Acquire + clip** — `scripts/build_haifa_assets.sh`:
   - `curl` the Geofabrik `israel-and-palestine-latest.osm.pbf` (cache locally; record SHA + date).
   - `osmium extract --bbox 34.94,32.74,35.10,32.86 israel-and-palestine-latest.osm.pbf -o haifa.osm.pbf`
     (fallback: BBBike polygon export). Assert `haifa.osm.pbf` size in `[1 MB, 12 MB]` (sanity bound).
2. **Parse + build R-tree blob** — `OsmPreprocessor.kt` in `:tools` (uses `osmpbf`):
   - **Pass 1**: collect every node `id → (lat, lng)` into a primitive `LongObjectMap` (Haifa ≈ a few
     hundred k nodes → fits in dev-machine RAM easily).
   - **Pass 2**: for each `way` whose `highway ∈ {footway, path, pedestrian, steps, living_street,
     residential, unclassified, tertiary, secondary, primary, service, cycleway, track}` AND
     `access != private/no` (single locomotion-agnostic profile, M10): materialise its node coordinates,
     split into consecutive **segments** `(a→b)`, assign a stable `segment_id` (monotonic), record
     `way_id`, `highway` class, the segment endpoints, and the parent way's `name` (for cue text).
   - Bulk-load all segment AABBs into the **STR-packed R-tree**; serialise to `haifa_segments.rtree`
     (header: count, node fanout, bbox; then packed node array + leaf payload array).
3. **Build routing graph blob** — same Pass-2 walk:
   - Emit an **adjacency graph**: dedup endpoints into graph vertices (snap-to-grid at ~1e-7 deg to merge
     shared endpoints), edges = segments with length (haversine) + `highway` class + `oneway` flag.
     Serialise to `haifa_graph.bin` (vertex table `id→lat,lng`; edge table `from,to,len_m,class,oneway`).
   - This is structure (2) and is the **fallback router's** input; BRouter uses its own `rd5` (stage 5).
4. **Build geocode index blob**:
   - For every node/way with `name` or `addr:*`, emit a `place` record `(place_id, lat, lng, display_name,
     normalised_tokens)`; build the inverted index `token → [place_id]`.
   - Serialise `haifa_geocode.bin` (places table + token postings). Lowercase/strip-diacritics tokens;
     keep Hebrew + Latin (Haifa names are bilingual) — store raw display, index normalised.
5. **Build BRouter `rd5` slice** — invoke `brouter-map-creator` on `haifa.osm.pbf` to produce the Haifa
   `rd5` segment file(s); copy into assets. (Profiles `.brf` are tiny text — ship the chosen
   foot∪bike∪road profile alongside.)
6. **Emit asset manifest** — `haifa_assets.json`: `{ schema_version, bbox, upstream_pbf_date,
   upstream_pbf_sha, built_at, segment_count, vertex_count, edge_count, place_count, blob_sizes{...},
   brouter_rd5_name, profile_name }`. This is what the region manager (deferred) and the
   `osm_data_age_days` confidence factor (Step E, deferred) read.
7. **APK-size gate (binding)**: build a debug APK with the assets + the BRouter jars and **measure the
   delta**. **Gate: total APK delta ≤ 12 MB compressed** (BRouter jars ~2–4 MB + `rd5` ~2–8 MB +
   our three blobs ~1–3 MB for Haifa). If blown, drop BRouter for the hand-rolled A* router (removes the
   jars + `rd5`, leaving only our blobs ≈ ≤ 4 MB) and re-measure. ADR records the outcome.

Output assets, committed under `app/src/main/assets/osm/haifa/`:
`haifa_segments.rtree`, `haifa_graph.bin`, `haifa_geocode.bin`, `haifa_assets.json`,
`haifa.rd5` (+ `*.brf` profile). **No `.pbf`.** Add `noCompress` for `.rd5`/`.rtree`/`.bin`
in `app/build.gradle.kts` `androidResources` (alongside the existing `tflite`) so they memory-map.

#### A*.2 On-device data layer (`OsmDataLayer.kt`, new — the runtime API)

A single Kotlin object/class `OsmDataLayer` constructed once at app start (off the UI thread,
`Dispatchers.Default`), holding the three loaded structures. It is the **only** thing Steps C / K-routing /
K-search talk to — they never see the asset format. Public API (typed, immutable returns):

- `suspend fun load(context: Context): Result<Unit>` — copies/opens the four blobs from assets,
  mmaps `.rtree`/`.bin`, hands `.rd5` path to BRouter; populates structures; logs load time + counts.
  Idempotent; safe to call on every `onCreate`. Returns `Result.failure` (not throw) on a corrupt/missing
  blob so the app degrades to "no snap, straight-line route" rather than crashing.
- `data class RoadSegment(val segmentId: Long, val wayId: Long, val highwayClass: String,
   val aLat: Double, val aLng: Double, val bLat: Double, val bLng: Double, val name: String?)`
- `fun nearestSegments(lat: Double, lng: Double, radiusM: Double, maxResults: Int): List<RoadSegment>`
  — R-tree window query (expand the AABB by `radiusM` converted to deg) returning candidates **sorted by
  true perpendicular distance** to the `(a→b)` segment. This is exactly what Step C's hand-rolled
  `RoadSnapper.snapToRoad` body calls.
- `data class GeoHit(val placeId: Long, val lat: Double, val lng: Double, val displayName: String)`
- `fun geocode(query: String, anchorLat: Double, anchorLng: Double, limit: Int): List<GeoHit>`
  — token-normalise the query, intersect postings, rank, return top-`limit`. Step K-search calls this.
- `fun manifest(): OsmAssetManifest` — exposes bbox + ages for the (deferred) region manager / Step E.
- The **routing graph** is held internally; routing is exposed through a separate `OsmRouter`
  (A*.3) so the data layer stays a pure data/query surface.

All queries are pure, allocation-light, and main-thread-safe for the read path (mmap + binary search /
tree descent — no I/O after `load`).

#### A*.3 Router seam (`OsmRouter.kt`, new — feeds Step K-routing only)

Thin interface so Step K-routing is decoupled from BRouter-vs-fallback:
`interface OsmRouter { suspend fun route(start: GeoPt, dest: GeoPt): List<GeoPt> /* polyline, empty = no route */ }`.
Default impl `BRouterRouter` wraps `brouter-core` in-process against the loaded `rd5` + profile;
fallback impl `DijkstraRouter` runs A* over `haifa_graph.bin`. Step K-routing's `NavigationManager`
builds its `NavigationRoute` from the returned polyline (the existing straight-line `calculateFallbackRoute`
stays as the **no-route** fallback when the polyline is empty / out-of-bbox). `GeoPt` here is an internal
`(lat,lng)` pair; the public `NavigationRoute`/`RouteStep`/`ManeuverType` contract is unchanged (Step K body).

#### A*.4 First-launch load + budgets (Haifa-only)

- **APK delta budget**: **≤ 12 MB compressed** (A*.1 gate). For comparison the original §8 budget was
  ≤ 30 MB for the whole country with GraphHopper; Haifa-only + BRouter is far under.
- **Per-region writable storage**: **≤ 25 MB** (Haifa blobs + `rd5` copied to internal storage if
  BRouter needs a writable dir; the `.rtree`/`.bin` blobs are mmapped read-only from assets where the
  platform allows, else copied once).
- **First-launch load time**: **≤ 300 ms** to mmap the three blobs + hand BRouter the `rd5`
  (no parsing on device — the cost the original §8's ≤ 60 s "index time" paid is moved to the dev machine).
  Cold-start impact ≤ 200 ms on the warm path (Step 9 plan-wide budget); load runs off the UI thread so it
  never blocks first frame.
- **No network, no key, no Geofabrik call on device** — Haifa data is fully embedded (jamming-immune,
  per §0.2). On-demand extra regions + the region manager are **DEFERRED** (original §8 Step A.2/A.3);
  v1 is Haifa-only and the user is intentionally "trapped" in-city (§0.6 Q2 — accepted for v1).

#### A*.5 Instrumentation (mandatory per navsight-implementor)

`OsmDataLayer.load` logs (LOGI) + records counters: `osm_load_ms`, `osm_segment_count`,
`osm_vertex_count`, `osm_edge_count`, `osm_place_count`, `osm_load_failed` (1 on corrupt blob),
`osm_rtree_bytes`, `osm_rd5_present` (0/1). `nearestSegments` increments `osm_snap_query_count` and
records `osm_snap_query_p95_us`; `geocode` increments `osm_geocode_query_count`. These are the
falsifiers the Step C / K bodies assert against.

#### Full implementation plan

1. **Asset pipeline** — create `tools/osm-preprocess/` (`:tools` desktop JVM module, not in the app
   classpath) with `OsmPreprocessor.kt` (osmpbf parse → R-tree blob + graph blob + geocode blob +
   manifest) and `scripts/build_haifa_assets.sh` (acquire/clip via `osmium`, invoke the preprocessor,
   invoke `brouter-map-creator`, drop assets into `app/src/main/assets/osm/haifa/`). Commit the
   generated assets (they are reproducible from the manifest's recorded bbox + PBF date).
2. **Build config** — add `noCompress` for `.rd5`/`.rtree`/`.bin` in `app/build.gradle.kts`; add the
   BRouter jars (`brouter-core`, `brouter-codec`, `brouter-mapaccess`, `brouter-expressions`) to
   `libs.versions.toml` + `app/build.gradle.kts` (the Google `google-maps-services` + `places-client`
   removals happen in the Step C / K bodies, not here — keep `play-services-maps` + the Maps key for the
   hybrid display).
3. **Runtime data layer** — implement `OsmDataLayer.kt` (load + `nearestSegments` + `geocode` +
   `manifest`) and the binary readers (`SegmentRTreeReader`, `RoutingGraphReader`, `GeocodeIndexReader`)
   as small focused files (< 300 LOC each per coding-style). Hand-roll the **STR-packed R-tree** reader
   + perpendicular-projection helper.
4. **Router seam** — implement `OsmRouter` + `BRouterRouter` (in-process `brouter-core`) and the
   `DijkstraRouter` fallback over `haifa_graph.bin`.
5. **Wire load** — call `OsmDataLayer.load(context)` once during app init (e.g. in
   `NavSightViewModel` init or `Application.onCreate`), off the main thread, before Step C's snapper or
   Step K's search/route are first invoked. Degrade gracefully on `Result.failure`.
6. **Instrumentation** — emit the A*.5 counters + LOGI lines.
7. **Offline preprocess verification** — a `:tools` unit test asserts the round-trip: a tiny synthetic
   PBF (CI construct only — §3 Principle 6: no synthetic OSM in *production*; test-only is fine)
   produces blobs whose `nearestSegments`/`geocode` return the planted segment/place. This is the
   "replay before re-flash" guard (§3 Principle 4) for the data layer.

#### Acceptance criteria

- The dev pipeline (`scripts/build_haifa_assets.sh`) runs end-to-end from the Geofabrik
  `israel-and-palestine` PBF to `app/src/main/assets/osm/haifa/{haifa_segments.rtree, haifa_graph.bin,
  haifa_geocode.bin, haifa_assets.json, haifa.rd5, *.brf}`, deterministically (same PBF date + bbox →
  byte-identical blobs), and writes a manifest recording the upstream PBF date, SHA, and exact bbox.
- **APK size delta ≤ 12 MB compressed** with the BRouter jars + Haifa assets (the A*.1 gate); if the
  gate is blown, the build switches to the `DijkstraRouter` fallback (drops BRouter jars + `rd5`) and an
  ADR records it.
- `OsmDataLayer.load(context)` completes in **≤ 300 ms** on the target S21 Ultra, off the UI thread,
  with no network access (airplane mode), logging `osm_load_ms` + the four `*_count` counters > 0 for Haifa.
- `nearestSegments(lat, lng, radiusM=30, maxResults=8)` for a point on a known Haifa road returns that
  road's segment first (sorted by true perpendicular distance), with `osm_snap_query_p95_us` ≤ 1000 µs.
- `geocode("…known Haifa place…", anchor, limit=5)` resolves to a lat/lng inside the Haifa bbox; an
  out-of-corpus query returns an empty list (no crash, no false hit).
- `OsmRouter.route(start, dest)` for two in-bbox Haifa points returns a non-empty polyline that stays on
  mapped ways; an out-of-bbox destination returns empty (caller falls back to straight line).
- **Read-only on the EKF**: trajectory output is bit-identical with and without `OsmDataLayer` loaded
  (this step adds data + queries only; it does not call `Tracker`/`EKFState`).
- The `:tools` synthetic-PBF round-trip test passes in CI (no on-device, no production synthetic-OSM branch).

---

### Step B* — Read-only VIO→lat/lng accessor (`Tracker::current_vio_lla`)

**Goal**: expose the EKF's current position as `(lat, lng)` for the Kotlin map-matching layer, derived
from `(SessionAnchor + EKF position) → inverse-haversine`. **Read-only on the engine — the only C++ touch
in the whole migration.** One bootstrap GPS fix sets the anchor (allowed by ADR-004); if no fix is ever
obtained, the accessor reports "unanchored" and the matcher silent-disables. **No raw GPS sample feeds
the EKF; no GPS feeds the matcher — only VIO-derived position.** (Supersedes original §8 Step B where it
assumes a matcher consumer; v1 the only consumer is the read path the harness records.)

#### Full implementation plan

1. **`SessionAnchor` struct in `Tracker.h`** (new, private member `session_anchor_`):
   - Fields: `double anchor_lat_rad`, `double anchor_lng_rad`, `int64_t anchor_t_ns`, `bool valid`.
   - Default-constructed `valid = false`. Set exactly once, never mutated after.
2. **`Tracker::setSessionAnchor(double lat_deg, double lng_deg, int64_t t_ns)`** (new public method):
   - Called from Kotlin via a new `NativeBridge` JNI entry the **first** time a valid GPS fix arrives during
     bootstrap (mirrors the existing `memory/reference_gps_usage_model.md` one-fix contract).
   - Stores `anchor_lat_rad = lat_deg·π/180`, `anchor_lng_rad = lng_deg·π/180`, `anchor_t_ns = t_ns`,
     `valid = true`. Idempotent: if `session_anchor_.valid` is already true, log
     `LOGI("[VIO_LLA] anchor already set, ignoring re-anchor")` and return — never re-anchor mid-session
     (would teleport the whole track).
   - `EventCounters`: increment `vio_lla_anchor_set` on first set; `vio_lla_anchor_reanchor_ignored` on any
     later call.
3. **`Tracker::current_vio_lla()`** (new public method), returns a small POD:
   ```cpp
   struct VioLla {
       double lat_rad;     // VIO-projected latitude
       double lng_rad;     // VIO-projected longitude
       int64_t t_ns;       // EKF state timestamp
       double var_xy_m2;   // EKF position-covariance trace (xy), published uncertainty
       bool   valid;       // false => no anchor yet; matcher must silent-disable
   };
   ```
   - If `!session_anchor_.valid`: return `VioLla{0,0,last_state_t_ns,0,false}` and (rate-limited, once per
     ~5 s) `LOGI("[VIO_LLA] no SessionAnchor — matcher disabled")`; increment `vio_lla_unanchored_reads`.
   - Else read `EKFState::getPosition()` → local-frame metres `(p_x, p_y)` in the **same world convention the
     engine already uses** (Z-up world per `project_z_up_fix_2026_05_08`; XY are the horizontal components —
     assert/log the convention at the boundary per the implementor contract).
   - Inverse-haversine (small-angle, equirectangular — adequate at city scale, the inverse of Step K-search's
     forward projection so they round-trip):
     ```
     lat_rad = anchor_lat_rad + p_y / EARTH_RADIUS_M
     lng_rad = anchor_lng_rad + p_x / (EARTH_RADIUS_M · cos(anchor_lat_rad))
     ```
     `EARTH_RADIUS_M = 6371000.0` (matches `SnappedLatLng.distanceTo` in `RoadSnapper.kt:238`, so distance
     math is consistent across the JNI boundary — no magic-number drift).
   - `var_xy_m2` = trace of the EKF position covariance block (`P[0,0] + P[1,1]`), already available where
     loop-closure variance is read.
   - **No write to EKF state.** This method only reads.
4. **`NativeBridge.kt` JNI**: add `external fun nativeSetSessionAnchor(latDeg, lngDeg, tNs)` and
   `external fun nativeCurrentVioLla(): DoubleArray?` (returns `[latDeg, lngDeg, tNs, varXyM2]` or `null` when
   `valid == false`). Convert `lat_rad/lng_rad` → degrees on the C++ side so Kotlin gets a gms-compatible
   `LatLng(latDeg, lngDeg)` directly. Keep the bridge thin (no logic).
5. **No-fix fallback (ADR-004 compliance)**: `nativeCurrentVioLla()` returns `null` until an anchor exists.
   The Kotlin caller treats `null` as "matcher unavailable" and falls through to the raw-VIO display path —
   exactly the `isSnapped = false` behavior `RoadSnapper` already has. GPS jamming in Haifa means a fix may
   never arrive; the app must remain fully functional (VIO dot only, no snap) in that case.
6. **Sim recording**: `SimulationFrameRecorder` gains a per-frame `vio_lla` field (`[latDeg, lngDeg,
   varXyM2]`) alongside the existing `glat/glng/gacc`, so the replay harness (§8.I / `tests/cpp/
   replay_harness.cpp`) and `scripts/analyze_replay_csv.py` can compare VIO-projected position vs recorded
   GPS offline.

#### Acceptance criteria

- With a SessionAnchor set, `current_vio_lla()` round-trips: feeding its `(lat,lng)` back through Step
  K-search's forward `(lat,lng)→local-metres` projection reproduces `EKFState::getPosition().xy` to < 0.5 m
  at Haifa latitude (proves the inverse-haversine pair is consistent).
- On a stationary sim (phone on a table) the `vio_lla` stream stays within the EKF's own reported position σ
  (`sqrt(var_xy_m2)`) of the recorded GPS lat/lng — the projection adds no bias beyond the EKF's stated
  uncertainty. (No "75° reduction" claim — that was the dead pre-`4a9a212` fixture, §0 M6/M7.)
- With **no** GPS fix for the entire session, `nativeCurrentVioLla()` returns `null` for every call, the app
  shows the raw VIO dot, no crash, and `vio_lla_unanchored_reads > 0`.
- EKF behaviour is bit-identical with and without this accessor wired (read-only proof): the same sim produces
  an identical `traj_x/z` CSV from the replay harness.
- `vio_lla_anchor_set == 1` and `vio_lla_anchor_reanchor_ignored == 0` on a normal single-anchor walk.

---

### Step C* — SNAP: replace `RoadSnapper.kt`'s Google Roads call with a local OSM R-tree snap

**Goal**: make `RoadSnapper.snapToRoad()` snap against the local Haifa OSM road segments (Step A*'s
`OsmDataLayer.nearestSegments` R-tree) instead of `RoadsApi.snapToRoads`, while keeping the class name, the
`snapToRoad(vioPosition: LatLng, recentPath: List<LatLng>): SnappedLatLng` signature, the `SnappedLatLng`
shape, the LRU cache, the >15 m soft-snap "trust raw VIO", and the throttle **byte-identical at the call
sites** — so `NavSightViewModel:63` (construction), `:487` (call), and `:825` (`shutdown()`) are untouched.
This is a like-for-like $0/offline replacement of the *current* per-point Google Roads snapper; its
snap-to-nearest corner-cut limitation is **accepted for v1** and fixed later by the deferred Step D HMM (it
is not a regression — the current Google snapper also snaps per-point). (Supersedes original §8 Step C; the
R-tree it queries is the one built in Step A*, not a separate `RoadGraph`.)

#### Full implementation plan

1. **Source = Step A*'s `OsmDataLayer`, not a new graph.** Step A* already builds the STR-packed segment
   R-tree and exposes `fun nearestSegments(lat, lng, radiusM, maxResults): List<RoadSegment>` (sorted by true
   perpendicular distance) and `RoadSegment(segmentId, wayId, highwayClass, aLat, aLng, bLat, bLng, name)`.
   Step C consumes that seam; it does **not** define its own R-tree. `RoadSnapper` reaches the loaded
   `OsmDataLayer` through a process-wide holder set by Step A* on first launch (see step 4) — it never loads
   OSM data itself.
2. **Per-point snap math** — a top-level `internal` function `projectPointOntoSegment(...)` (in a new small
   file `app/src/main/java/com/example/navsight1/RoadSnapMath.kt`, unit-testable, no Android deps). For the
   single nearest `RoadSegment` returned by `nearestSegments`, recompute the foot-of-perpendicular + metric
   distance (the R-tree already sorted by it, but Step C needs the projected *point*, not just the distance):
   - Work in a **local east-north metric tangent plane** anchored at the query point:
     `mPerDegLat = 111_320.0`, `mPerDegLng = 111_320.0 * cos(toRadians(pLat))`.
     Convert A, B, P to metres relative to P: `ax = (aLng - pLng) * mPerDegLng; ay = (aLat - pLat) * mPerDegLat`
     (B likewise; P is the origin).
   - Project origin onto A→B: `dx = bx - ax; dy = by - ay; len2 = dx*dx + dy*dy`.
     `t = if (len2 < 1e-9) 0.0 else clamp(-(ax*dx + ay*dy) / len2, 0.0, 1.0)` (clamp keeps the snap on the
     finite segment — this is what makes it snap to nearest *road*, and is exactly the corner-cutting source
     that Step D later removes).
     `fx = ax + t*dx; fy = ay + t*dy` → foot in local metres; distance `= hypot(fx, fy)`.
   - Convert the foot back to lat/lng: `snappedLat = pLat + fy / mPerDegLat; snappedLng = pLng + fx /
     mPerDegLng`. The returned `distanceM` is reused directly (already metric, do **not** re-run haversine for
     the gate; keep `SnappedLatLng.distanceTo` only for the existing >15 m soft-snap comparison so that path
     stays byte-identical).
3. **Rip out Google inside `RoadSnapper.kt`, keep the control flow**:
   - **Remove** imports `com.google.maps.GeoApiContext`, `com.google.maps.RoadsApi`,
     `com.google.maps.model.SnappedPoint`. **Keep** `com.google.android.gms.maps.model.LatLng` (display still
     uses gms LatLng — no GeoPoint refactor in v1).
   - **Comment-out** (not delete, per the project's comment-out rule) the `geoApiContext` lazy property and the
     `RoadsApi.snapToRoads(...).await()` call in a `/* LEGACY (Google Roads API, removed in OSM migration
     2026-05-29) ... */` brace so the diff shows what was replaced.
   - **Preserve verbatim**: the `withContext(Dispatchers.IO)` wrapper; the cache-first check
     (`snapCache.get(cacheKey)`); the `getCacheKey` 6-decimal key; the LRU `CACHE_SIZE = 50`; the `>15.0`
     soft-snap branch (`distToRoad > 15.0 → return rawSnapped`); the `try/catch` with `consecutiveFailures`
     log-throttling (now wraps the local query — a missing/empty index falls into the same graceful "return
     unsnapped" path); and the `recentPath` parameter (stays in the signature; v1 snap-to-nearest only needs
     the last point, so `recentPath` is accepted-but-unused — documented inline, used again by Step D).
     Keeping it unused preserves the `:487` call byte-for-byte.
   - **`shutdown()` stays a method** with the same name: replace the body with a no-op log
     (`Log.d(TAG, "RoadSnapper shutdown")`) since there is no longer a `GeoApiContext` to close. `:825` is
     untouched.
4. **Constructor — keep `:63` byte-identical via a provider, not a ctor-arg change.** The current ctor is
   `RoadSnapper(private val apiKey: String)` and `NavSightViewModel:63` constructs it. Since `:63` must stay
   byte-identical and `RoadSnapper` must reach the data layer, keep `RoadSnapper(private val apiKey: String)`
   **unchanged** and have `RoadSnapper` pull the already-loaded `OsmDataLayer` from the process-wide holder
   Step A* populates (`OsmDataLayer` singleton / a `@Volatile` provider). The construction line at `:63` is
   **literally unchanged**; `apiKey` becomes a benign ignored field (comment `// MIGRATION: apiKey ignored —
   OSM snap needs no key`). If the data layer is `null` (not yet loaded), `snapToRoad` returns the unsnapped
   `SnappedLatLng(isSnapped=false)` — same shape as today's "API disabled" path, so the dot trusts raw VIO
   until the layer is ready. **Decision (no re-deciding downstream): ship the provider route** — the
   ignored `apiKey` field costs nothing and the provider is the only zero-touch way to inject the layer.
5. **Local confidence + graceful field mapping** (replace Google's `placeId`/`originalIndex`):
   - Keep the `SnappedLatLng(latitude, longitude, placeId: String?, originalIndex: Int?, isSnapped: Boolean)`
     data class **unchanged** (read elsewhere via `toLatLng()`/`distanceTo` and possibly by recordings — do
     not alter its shape).
   - On a successful snap: `placeId = "osm:way:$wayId"` (the OSM way id, namespaced so any consumer that only
     checks non-null still works and a human can tell it's OSM). `originalIndex = null` (Google's batch index
     has no analogue in a single-point snap; nothing downstream depends on its value).
   - **Confidence is computed locally** so Step D can later reuse it: the current `snapToRoad` signature has
     **no** variance argument and the task forbids changing it, so v1 uses a constant floor: `sigma = 5.0`
     (OSM way-geometry accuracy, https://wiki.openstreetmap.org/wiki/Accuracy), `confidence = exp(-distanceM /
     sigma)`. (When Step F/D add a variance-carrying overload, the `max(5, sqrt(var))` form drops in.) The
     result is **not** stored on `SnappedLatLng` (no field exists and the task forbids adding one) — it is
     logged (`Log.d(TAG, "OSM snap %.1fm conf %.2f way %d".format(distanceM, confidence, wayId))`) and used
     internally only.
   - **Off-road / no-snap**: call `OsmDataLayer.nearestSegments(lat, lng, radiusM = 5.0 * sigma /*=25 m*/,
     maxResults = 1)`. If it returns empty (no segment within 25 m), return the unsnapped
     `SnappedLatLng(vioLat, vioLng, placeId=null, originalIndex=null, isSnapped=false)` — identical to today's
     empty-result and API-disabled paths, so the existing "trust raw VIO when not snapped" display is unchanged.
6. **Throttle**: the original code's "throttled to 2 Hz" is enforced at the **call site**
   (`NavSightViewModel:487`, ~2 Hz cadence), not inside `RoadSnapper`. Do **not** add a throttle (keeps `:487`
   byte-identical); the local R-tree query is O(log N) and cheap enough at the existing cadence. Note this in
   a comment so the doc-claim/behavior gap is intentional.
7. **gradle / keys** (cross-reference the Build-changes body; do not duplicate the edit): once C lands,
   `google-maps-services` is no longer referenced by `RoadSnapper`. Its removal happens in the Build-changes
   pass (it may still be transitively referenced until Directions/Places are migrated). Keep
   `play-services-maps` and the Maps display key (display still needs them — hybrid v1).
8. **Tests** (`tests/` — never root): JVM unit tests on the pure-Kotlin math, no Android runtime.
   - `projectPointOntoSegment`: point beside a horizontal segment → foot at perpendicular, distance == offset;
     beyond endpoint A → clamps to A (`t==0`); beyond B → clamps to B (`t==1`); degenerate zero-length segment
     → distance to the single node, no NaN.
   - `RoadSnapper.snapToRoad`: with a fake `OsmDataLayer` returning a 3 m snap → `isSnapped=true`,
     `placeId=="osm:way:<id>"`, lat/lng == projected; with a 40 m snap (>15 m soft-snap) → returns raw VIO
     unchanged (`isSnapped=false`); with the data-layer provider `null` → returns raw VIO unsnapped; cache-hit
     on a repeated 6-decimal key returns the cached snap without re-querying.

#### Acceptance criteria

- `NavSightViewModel.kt` lines **63 (construction), 487 (call), 825 (`shutdown()`)** are **byte-identical**
  before and after — verified by `git diff -- app/src/main/java/com/example/navsight1/NavSightViewModel.kt`
  showing **no change** at those lines.
- The public surface of `RoadSnapper` is preserved: class name `RoadSnapper`, method
  `suspend fun snapToRoad(vioPosition: LatLng, recentPath: List<LatLng>): SnappedLatLng`, method
  `fun shutdown()`, and the `SnappedLatLng(latitude, longitude, placeId, originalIndex, isSnapped)` shape are
  all unchanged (compile-checked by the untouched call sites).
- No `com.google.maps.*` (Roads API) import remains in `RoadSnapper.kt`; `com.google.android.gms.maps.model.
  LatLng` is retained. The old Google block is commented-out, not deleted.
- On a replayed Haifa walk (the post-`4a9a212` fixture, e.g. the `1779643969824` 134 m loop), the snapped dot
  follows the OSM road for ≥ 80% of sustained-motion samples (`speed > 1.0 m/s`) and returns raw-VIO unsnapped
  when the user is > 25 m from any road (building/plaza), matching the original Step C off-road behavior — and
  **never** produces a snap further than the >15 m soft-snap allows.
- The snap runs fully **offline with no Maps/Roads key and no network** (jamming-immune); with the data layer
  not yet loaded the dot trusts raw VIO and never crashes.
- JVM unit tests pass; build is green (`./gradlew :app:assembleDebug`).
- Documented limitation: snap-to-nearest **corner-cuts** and is **bearing-blind** — **accepted for v1**, fixed
  by the deferred Step D HMM (per §0.7). Like-for-like replacement of the current per-point Google snapper,
  **not a regression**.

---

### Step K-routing* — offline router (replaces Google Directions API)

**Goal**: "make routing work again" with **$0, offline, and jamming-immune**: replace the `DirectionsApi`
call inside `NavigationManager.calculateRoute(start, destination)` with the embedded offline router
(**BRouter `brouter-core`**, MIT, pure-Java) chosen in Step A* / exposed via the `OsmRouter` seam, while
**keeping the `NavigationRoute` / `RouteStep` / `ManeuverType` public types and the `calculateFallbackRoute`
straight-line fallback unchanged** so `NavigationManager`'s callers and the navigation UI are untouched.
Turn-by-turn maneuvers and street names come from OSM way data (BRouter voice hints + way `name` tags)
instead of Google's HTML instruction strings. (Supersedes original §8 Step K routing portion §8.K.1 items
3/5/6 where they assumed GraphHopper/Directions.)

#### Library decision (justified, binding for v1): BRouter, not Valhalla-mobile, not hand-rolled A*

| Option | Verdict | Reason |
|---|---|---|
| **BRouter `brouter-core`** (MIT, pure-Java + Android) | **CHOSEN** | No NDK — drops into `app/build.gradle.kts` as a JVM dependency/vendored module, same slot as the removed `google-maps-services`. Purpose-built for **offline Android**. **Its own `.rd5` data** (a single Haifa-clipped 5×5° segment slice, a few hundred KB–low MB), built by `brouter-map-creator` from the same clipped PBF as Step A*. Emits **voice hints with turn types** (left / slight-left / sharp-left / right / … / roundabout-exit / u-turn / continue) that map 1:1 onto the existing `ManeuverType` enum; street names from OSM way `name` tags — so the UI's instruction/maneuver/icon path is preserved. One locomotion-agnostic profile (`.brf`, foot∪cycleway∪road) per M10, no mount split. |
| **Valhalla-mobile** (MIT, C++/NDK) | Rejected v1 | Real NDK build of a large C++ project (the cost flagged in §5.2); tiles heavier than a single `.rd5`; community mobile wrapper exposes only `route` via JNI. Documented bailout only if BRouter fails the spike. |
| **Hand-rolled A*/Dijkstra over the Step A* graph** | Rejected v1 (kept as fallback) | Reuses the Step A* `haifa_graph.bin` (zero new data, the `DijkstraRouter` seam already exists), but the entire turn-instruction generator (junction detection, turn-angle → `ManeuverType`, street-name attribution, voice-hint distances) would be net-new code that BRouter already ships and field-tests. Named fallback if BRouter's APK-delta or routing-latency gate (A*.1) is blown. |

#### Full implementation plan

1. **Dependencies** — handled in the Build-changes body: remove `google-maps-services` + `places-client`;
   keep `play-services-maps` + the Maps display key; add the BRouter jars. Not duplicated here.
2. **Embed Haifa routing data + profiles** — handled in Step A* (stage 5): `app/src/main/assets/osm/haifa/
   haifa.rd5` + the chosen `.brf` profile; copied to a writable `segmentDir` on first launch by the Step A*
   asset-copy pass.
3. **Router class = Step A*'s seam.** `OfflineRouter` is the `OsmRouter`/`BRouterRouter` from A*.3, wrapping
   `brouter-core` **in-process** (no `Bundle`/AIDL service — we link the jar). It runs on `Dispatchers.IO`:
   ```
   val rc = RoutingContext().apply {
       localFunction = profilePath.absolutePath   // .brf profile
       turnInstructionMode = 2                     // locus-style hints (street names + turn cmds)
   }
   val waypoints = listOf(
       OsmNodeNamed().apply { name = "from"; ilon = toIlon(start.lng);  ilat = toIlat(start.lat) },
       OsmNodeNamed().apply { name = "to";   ilon = toIlon(destination.lng); ilat = toIlat(destination.lat) }
   )
   val engine = RoutingEngine(null, null, segmentDir.absolutePath, waypoints, rc, /*engineMode=*/0)
   engine.doRun(MAX_RUNNING_TIME_MS)               // bounded value, not 0
   val track: OsmTrack? = engine.foundTrack        // null + engine.errorMessage on failure
   ```
   (`ilon = (lon + 180) * 1e6`, `ilat = (lat + 90) * 1e6` — BRouter's fixed-point integer coords.) Returns
   `null` (point off-graph / `.rd5` missing) so the caller falls back to the straight line.
4. **Rewrite `NavigationManager.calculateRoute(start, destination)` body — signature/return type unchanged.**
   - **Comment-out** the `geoApiContext` field, the `GeoApiContext` import, the
     `DirectionsApi`/`PolylineEncoding`/`TravelMode` usage. Keep `apiKey` as an ignored constructor param so
     the construction site is untouched (pass it nowhere).
   - New body:
     1. If `OsmRouter` is unavailable (assets not yet copied / no `.rd5`), `Log.w` and
        `return calculateFallbackRoute(start, destination)` — the straight line stays as the no-route fallback.
     2. `val polyline = osmRouter.route(start, destination); if (polyline.size < 2) return
        calculateFallbackRoute(start, destination)`.
     3. Map `OsmTrack` → `NavigationRoute`:
        - `polyline`: `track.nodes` → `List<LatLng>` (each `OsmPathElement`'s `ilon/ilat` → `LatLng`).
        - `totalDistanceMeters`: `track.distance.toDouble()`.
        - `estimatedTimeSeconds`: prefer a profile-derived time if exposed; else the existing
          `distance / averageSpeed` (10 m/s) estimate `calculateFallbackRoute` uses — documented v1
          approximation (turn-by-turn timing split to `NAVIGATION_PLAN.md`).
        - `steps`: build `List<RouteStep>` from BRouter's **voice hints** (`track.getVoiceHints()` when
          `turnInstructionMode != 0`). Per hint: `startLocation`/`endLocation` = hint track position + next
          hint position; `maneuver` = `toManeuverType(hint.cmd)` (step 5); `streetName` = OSM way `name` at/
          after the junction ("Unknown road" when untagged — matches today's `extractStreetName` default);
          `instruction` = synthesized (`"Turn left onto <streetName>"`) — **no HTML parsing**
          (`parseManeuver`/`extractStreetName`'s `htmlInstructions` regex is now dead — comment out, don't
          delete); `distance` = arc-length between consecutive hints.
        - Zero hints (short straight route) → a single `STRAIGHT` `RouteStep` spanning start→destination (same
          shape as `calculateFallbackRoute`).
   - Keep the whole `withContext(Dispatchers.IO) { … }` + `try/catch` shape; on any exception, `Log.e` and
     `calculateFallbackRoute`.
5. **`ManeuverType` mapping** (replaces `parseManeuver` HTML matching). Add
   `OsmRouter.toManeuverType(cmd: Int): ManeuverType` keyed on BRouter `VoiceHint` command codes:
   left/right families → `TURN_LEFT/RIGHT`, `TURN_SLIGHT_*`, `TURN_SHARP_*`; u-turn → `UTURN_LEFT`/`UTURN_RIGHT`;
   roundabout-exit → `ROUNDABOUT_LEFT`/`ROUNDABOUT_RIGHT` (by exit-turn sign); keep-left/right → `FORK_LEFT`/
   `FORK_RIGHT`; continue / no-hint → `STRAIGHT`. `MERGE`/`RAMP_*` (no foot/cycleway analogue) fold to
   `STRAIGHT`/`FORK_*`; left in the enum (used by `getManeuverIcon`), no UI change. Pure code→enum table —
   **no magic thresholds** (BRouter already classified the turn angle).
6. **No EKF coupling, no display rewrite.** Routing output is display + cue only (matches original §8.K.2):
   `polyline` renders on the Google `GoogleMap` as a route overlay; the matched/snapped dot still comes from
   Step C* (`RoadSnapper`). `updateVioPosition(snappedPosition: LatLng)` and the rest of `NavigationManager`
   are untouched.
7. **Re-route + search are out of this body.** Off-route re-routing, route-adherence thresholds, voice
   playback, and the destination-search migration (Step K-search*) belong to the broader split (§0.5, §0.6
   Q8). This body only restores `calculateRoute` to working offline.
8. **ADR (Step J)**: BRouter chosen over Valhalla-mobile (no NDK) and hand-rolled A* (reuses shipped
   turn-instruction code), MIT license, `.rd5` data lifecycle (single Haifa segment, update cadence shared
   with §6 / ADR-020 region story), and the hand-rolled-A* bailout trigger (spike fails APK-delta or routing-
   latency gate).

#### Acceptance criteria

- **Works offline, $0, no Google key for routing.** With data disabled and `com.google.maps:*` deps removed,
  `calculateRoute(start, destination)` returns a `NavigationRoute` with `polyline.size > 2` and
  `steps.isNotEmpty()` for a Haifa start→destination pair on the embedded `.rd5` — no `GeoApiContext`, no
  `DirectionsApi`, no billing.
- **Public contract unchanged**: `NavigationRoute`, `RouteStep`, `ManeuverType`,
  `NavigationManager.calculateRoute` signature, `updateVioPosition(LatLng)`, and `calculateFallbackRoute` all
  compile and run **without any change to `NavSightViewModel` or the navigation UI** (no `.kt` outside
  `NavigationManager.kt` + the new `OsmRouter`/`OfflineRouter.kt` is edited).
- **Maneuvers + street names come from OSM**: a planned Haifa route produces `RouteStep`s whose `maneuver`
  matches the real turns (left/right/roundabout) and whose `streetName` matches the OSM way `name` for ≥ 80%
  of named ways along the route — with **zero HTML parsing** in the code path.
- **Fallback preserved**: when the router cannot produce a track (point off-graph, `.rd5` not yet copied, or
  exception), `calculateRoute` returns the straight-line `calculateFallbackRoute` (single `STRAIGHT` step) —
  no crash, no empty route.
- **Latency gate (spike)**: route computation for a typical Haifa origin→destination ≤ 2 s p95 on a Samsung
  Galaxy S21 Ultra; if exceeded, profile-simplify or fall back to the hand-rolled A* per the ADR.
- **APK/data gate (spike)**: `brouter-core` jar + Haifa `.rd5` + `.brf` delta is measured against the §6.2 /
  A*.1 storage budget; if blown, the hand-rolled-A*-over-Step-A*-graph (`DijkstraRouter`) bailout is taken.
- **ADR lands with the code** documenting BRouter-vs-Valhalla-vs-hand-rolled and the `.rd5` lifecycle.

---

### Step K-search* — offline destination geocoder (replaces Places API)

**Goal**: replace the paid Google **Places API** autocomplete in `SearchBarUi.kt` with the on-device
geocoder built in Step A* (`OsmDataLayer.geocode`) over the Haifa OSM `name`/`addr:*` index. Destination
search works fully offline, **$0**, jamming-immune. The search UX (the `WazeSearchBar` look, the predictions
dropdown, the route-preview card) is preserved unchanged — only the prediction/resolve source swaps.

#### Library decision (WebSearch'd)

There is **no battle-tested truly-offline embedded geocoder for Android.** The closest hits — OSMBonusPack
`GeocoderNominatim` and `hdk24/nominatim-osm` — are **online** wrappers around the public Nominatim HTTP
service (network + rate-limited + not jam-proof), failing the offline mandate; a full local Nominatim is a
Postgres/PostGIS server, not embeddable. **Decision: the hand-rolled inverted index from Step A*** (token →
`place_id` postings + a flat `place_id → (lat,lng,display_name)` table). Small (~150 LOC, already built in
A*.1 stage 4), reuses Step A*'s parsed POI list, needs no extra dependency or APK weight, and is the only
option satisfying offline + no-key + jam-proof.

#### Full implementation plan

1. **Index = Step A* output.** The geocode blob `haifa_geocode.bin` and the runtime
   `OsmDataLayer.geocode(query, anchorLat, anchorLng, limit): List<GeoHit>` (with
   `GeoHit(placeId, lat, lng, displayName)`) already exist from Step A*. Step K-search consumes them; it does
   not parse OSM itself. `name`/`name:en`/`name:he` are all indexed (Hebrew + Latin both hit), `addr:street` +
   `addr:housenumber` populate `displayName`.
2. **Thin adapter `OfflineGeocoder`** (new, `app/src/main/java/com/example/navsight1/OfflineGeocoder.kt`)
   — wraps `OsmDataLayer.geocode` to **mirror the existing free-function signatures** in `SearchBarUi.kt` so
   the composable wiring is untouched:
   ```kotlin
   suspend fun predictions(query: String, anchor: LatLng?, limit: Int = 6): List<PlacePrediction>
   suspend fun resolve(placeId: String): LatLng?     // placeId == GeoHit.placeId encoded as String
   ```
   - `predictions` = `OsmDataLayer.geocode(query, anchor?.latitude ?: bboxCenterLat, anchor?.longitude ?:
     bboxCenterLng, limit).map { PlacePrediction(placeId = it.placeId.toString(), primaryText =
     it.displayName.substringBefore(','), secondaryText = it.displayName.substringAfter(',', "")) }` — the
     **existing** `PlacePrediction` data class (`SearchBarUi.kt:32`), so the dropdown renders identically.
     A*'s `geocode` already ranks exact-name > prefix > token-contains, ties broken by closer-to-anchor.
   - `resolve` re-queries by id (or caches the last `predictions` list) and returns the gms
     `LatLng(lat, lng)` — the **existing** `onResult: (LatLng?) -> Unit` contract.
3. **Swap in `SearchBarUi.kt`** (preserve the composable, change only the source):
   - Replace the `placesClient: PlacesClient` parameter on `SearchBarCard`/`WazeSearchBar`/
     `fetchPlacePredictions`/`fetchPlaceLatLng` with `geocoder: OfflineGeocoder`. The composable body, the
     `predictions` state, the `isSearching` flag, the dropdown `Surface`, and the route-preview card are
     unchanged.
   - `fetchPlacePredictions(query, …)` → `scope.launch { predictions = geocoder.predictions(q, anchor) }`
     (drop the `AutocompleteSessionToken` — no billing session).
   - `fetchPlaceLatLng(placeId, …)` → `geocoder.resolve(placeId)`.
   - Keep the `q.length >= 2` debounce gate (`SearchBarUi.kt:72`) and the `try/catch → Log.e`.
   - `fetchDirectionsRoute(...)` (the Google Directions call at `SearchBarUi.kt:213`) is **out of this body** —
     replaced by the offline router in Step K-routing*. Until that lands, wire the call site to the `OsmRouter`
     interface; do not restore the Google key.
4. **Construction**: build `OfflineGeocoder` once (lazy singleton or in the ViewModel, alongside the existing
   `RoadSnapper` construction at `NavSightViewModel:63`) and pass it down; **comment-out** the `PlacesClient`
   creation and the `Places.initialize(...)` call.
5. **No network, no key**: `OfflineGeocoder`/`OsmDataLayer.geocode` make zero HTTP calls. Works in airplane
   mode / jammed Haifa.

#### Acceptance criteria

- A name query (e.g. "Technion", typed as "tech") returns a Haifa POI prediction list within 100 ms p95 on an
  S21 Ultra, fully offline (airplane mode on).
- Selecting a prediction resolves to a `LatLng` within ~20 m of the OSM POI node, and the existing
  route-preview card opens with that destination — UX visually identical to the Places path.
- Hebrew (`name:he`) and English (`name:en`) queries both resolve the same landmark.
- Removing the device's network connection causes **no** change in search behavior (proves no hidden online
  dependency).
- `SearchBarUi.kt` no longer imports `com.google.android.libraries.places.*`; a build with `places-client`
  removed from gradle compiles.
- Empty/blank handling matches the old path: no predictions for queries < 2 chars; graceful empty list (no
  crash) when the index has no match.

---

### Build changes (8M) — gradle edits

**Goal**: remove the two paid Google client libraries (Roads via `google-maps-services`, Places via
`places-client`); **keep** `play-services-maps` (the Google display stays in HYBRID v1) and **keep** the Maps
key (display still needs a key + billing account, even though loads are free); add the chosen free libs for
the OSM data layer/router. No display rewrite.

#### Full implementation plan

1. **`gradle/libs.versions.toml`** — remove the two paid entries and their version refs:
   - Delete library `google-maps-services` (line 39) and version `googleMapsServices` (line 17). (Roads API.)
   - Delete library `places-client` (line 42) and version `places` (line 18). (Places API.)
   - Optionally delete `slf4j-simple` (line 43) + `slf4j` (line 19) — exists only for `google-maps-services`
     logging (`build.gradle.kts:117–118 "SLF4J for Roads API logging"`); confirm no other consumer first.
   - **Keep**: `maps-services-client` = `play-services-maps` (line 41 — note the misnomer: this alias is the
     *display* SDK, not Roads), `maps-compose` (line 40), `google-location-services` (line 44 — still used for
     the one bootstrap GPS fix feeding `setSessionAnchor`; keep with a no-fix fallback per Step B*),
     `google-tasks` (line 45).
   - **Add** the BRouter router jars (`brouter-core`, `brouter-codec`, `brouter-mapaccess`,
     `brouter-expressions`) per Step A* / K-routing* (vendored `libs/*.jar` or a Gradle subproject — BRouter is
     not on Maven Central). The matcher data layer (Step A*) hand-rolled R-tree + the dev-machine PBF parser
     need **no runtime dependency** (parsing is build-time in `:tools`/`scripts`; the asset ships).
2. **`app/build.gradle.kts`** dependencies block:
   - Remove `implementation(libs.google.maps.services)` (line 106, Roads).
   - Remove `implementation(libs.places.client)` (line 115, Places).
   - Remove `implementation(libs.slf4j.simple)` (line 118) if its version ref was removed.
   - **Keep** `implementation(libs.maps.services.client)` (line 112 — display SDK),
     `implementation(libs.maps.compose)` (line 109), `implementation(libs.google.location.services)`
     (line 121), `implementation(libs.google.tasks)` (line 122).
   - **Keep** the Maps key plumbing unchanged (`build.gradle.kts:25–28`):
     `manifestPlaceholders["GOOGLE_MAPS_API_KEY"]` + `buildConfigField("String","GOOGLE_MAPS_API_KEY",…)` stay
     — the `GoogleMap` compose display needs the key + an attached billing account (loads free; the account
     must exist). Note `BuildConfig.GOOGLE_MAPS_API_KEY` is also read at `SearchBarUi.kt:58` for the Directions
     call — once Step K-routing* lands and `fetchDirectionsRoute` is gone, that read disappears, but the key
     itself stays for display.
   - Add the BRouter jars `implementation(libs.brouter.core)` etc. Ship the Haifa OSM assets under
     `app/src/main/assets/osm/haifa/` (Step A* output); `noCompress` for `.rd5`/`.rtree`/`.bin` is added in
     the Step A* build-config pass (mirrors the existing `noCompress += "tflite"` at `build.gradle.kts:42`).
3. **Manifest**: keep `com.google.android.geo.API_KEY` (display). Comment-out any
   `com.google.android.libraries.places` meta-data / `Places.initialize` (Places gone). Confirm no leftover
   Roads/Places permissions.
4. **ProGuard**: no new keep rules for removals; add keep rules only if BRouter uses reflection.

#### Acceptance criteria

- A clean build succeeds with `google-maps-services` and `places-client` fully removed from both
  `libs.versions.toml` and `build.gradle.kts`.
- The app still renders the Google `GoogleMap` display (HYBRID v1) — `play-services-maps` + `maps-compose` +
  the Maps key are intact.
- No source file references `com.google.maps.*` (Roads) or `com.google.android.libraries.places.*` (Places)
  after the swap (grep clean; commented-out legacy blocks excepted).
- APK has no Roads/Places client code; the size delta is a reduction net of the added BRouter lib + Haifa
  asset (which are budgeted in Step A*.1's ≤ 12 MB gate).
- The bootstrap GPS fix still reaches `setSessionAnchor` via `play-services-location`, and the app is fully
  functional with location denied/jammed (no-fix fallback, Step B*).

---

### Step L (8M) — OSM basemap display swap — **OPTIONAL / DEFERRED**

**Goal (deferred)**: replace the Google `GoogleMap` compose display (`MapScreenUi.kt`) with a fully-offline
OSM renderer so the basemap works with **no key, no billing account, and no network** — closing the last
Google coupling. **Not in v1.** HYBRID v1 deliberately keeps the Google display because it is free, decoupled,
and avoids both the display rewrite and the `GeoPoint` refactor across 7 files. Do Step L only when a jam-proof
offline basemap is explicitly wanted (the v1 caveat: in jammed/offline Haifa the Google basemap tiles go
blank/stale while the VIO dot + OSM snap + offline router keep working on embedded data).

#### Short plan (when undeferred)

1. Pick renderer: **osmdroid** (raster tiles, simplest drop-in `MapView`, smallest change) vs **MapLibre
   Native** (vector, MBTiles/PMTiles, nicer offline basemap, heavier integration). Recommendation: osmdroid
   for the first cut.
2. Introduce a neutral `data class GeoPoint(lat, lng)` (or a typealias) and migrate the gms `LatLng` usages
   across the 7 coupled files (MapScreenUi, NavigationManager, NavSightUtils, NavSightViewModel, RoadSnapper,
   SearchBarUi, SensorRepository). This is the refactor v1 intentionally skips.
3. Ship offline MBTiles/raster tiles for Haifa as an asset; remove the Maps key, the manifest
   `com.google.android.geo.API_KEY`, and `play-services-maps` + `maps-compose`.

#### Acceptance criteria (when undeferred)

- The basemap renders fully offline (airplane mode, no Maps key in `local.properties`) over the Haifa tile
  asset.
- The VIO dot, OSM-snapped dot, and route polyline render on the new map with the same UX as the Google
  display.
- No source references `com.google.android.gms.maps.*`; the Maps key and `play-services-maps` are removed.

---

## 8M.adv — Adversarial feasibility check (top risks + mitigations)

> Skeptic pass over the 8M bodies. Where they will fail in practice, with a mitigation each so the plan is
> not over-optimistic. Numbers are estimates to be confirmed by the A*.1 APK gate spike, not promises.

1. **APK/asset size for Haifa (R-tree + routing graph + geocode + BRouter jars + `.rd5`).** Realistic v1
   estimate: our three hand-rolled blobs ≈ **1–3 MB** (Haifa is one city: tens of k road segments, a few k
   POIs); BRouter jars ≈ **2–4 MB**; the Haifa-clipped `.rd5` ≈ **2–8 MB** (BRouter's stock world tiles are
   tens of MB each, but a single-bbox slice from `brouter-map-creator` is much smaller). **Total ≈ 5–15 MB**
   — the A*.1 **≤ 12 MB** gate is *plausible but not guaranteed*; the `.rd5` slice is the swing factor.
   **Mitigation**: the A*.1 gate is binding and measured before any commit; if blown, drop BRouter for the
   `DijkstraRouter` over `haifa_graph.bin` (removes jars + `.rd5`, leaving ≈ ≤ 4 MB) and take the hand-rolled
   turn-instruction cost. ADR records the outcome. Tighten the bbox if even the blobs run large.

2. **The router needs its OWN data format, doubling data + preprocessing (the biggest hidden cost).** TRUE —
   BRouter does **not** read our R-tree or `haifa_graph.bin`; it needs its own `.rd5`, built by a *separate*
   `brouter-map-creator` pass over the same PBF. So v1 carries **two** routing-capable structures (the R-tree/
   graph for snap + the `.rd5` for routing) and **two** preprocess stages. The hand-rolled A*/Dijkstra is the
   only option that reuses `haifa_graph.bin` (one structure, one preprocess) — but it costs the entire
   turn-instruction generator BRouter ships for free. **Net "less work" verdict**: BRouter is less *code* (no
   turn-instruction engine to write/test) at the price of more *data + a second preprocess stage*; hand-rolled
   A* is less *data* at the price of significant net-new code. **Mitigation**: BRouter for v1 (code risk >
   data risk when the data is one city and the gate has headroom); `DijkstraRouter` is the pre-built fallback
   if the A*.1 size gate fails — and `haifa_graph.bin` is built regardless (Step A* stage 3), so the fallback
   is always available with zero extra preprocessing.

3. **Google display + OSM snap = visible dot-vs-road offset, and a possible licensing issue.** Two real
   risks. (a) **Geometric offset**: Google's basemap tiles and OSM road geometry are independently surveyed
   and can disagree by **5–15 m** in places; the OSM-snapped dot will sit *beside* the Google-drawn road where
   they diverge, looking wrong even when the snap is correct. **Mitigation**: accept for v1 (the dot is
   VIO-truth snapped to OSM, not to Google's raster — documented); Step L (offline OSM basemap) removes the
   mismatch by drawing the same OSM geometry the snap uses. (b) **Licensing**: Google Maps Platform ToS
   restrict using Google Maps *content* alongside non-Google maps/derived data in the same view. Rendering the
   Google basemap while overlaying an OSM-derived polyline/dot is a **gray area** — overlaying your own data on
   a Google map is normal, but mixing OSM-*derived* routing geometry on a Google basemap may trip the
   no-other-maps clause. **Mitigation**: flag for a ToS read before shipping HYBRID v1 publicly; the clean exit
   is Step L (all-OSM display). For internal Haifa testing this is moot.

4. **Hand-rolled OSM PBF parse + R-tree on-device — weekend or a month?** Note: 8M moved the **parse off the
   device** (dev-machine `:tools` preprocess → shipped blob), which removes the hardest part (protobuf block
   decoding + the full node-coordinate map in RAM during cold start). What remains on-device is the **blob
   reader + STR-packed R-tree query + perpendicular projection** ≈ a few hundred LOC, realistically a
   **few days**, not a month. The dev-side `osmpbf` two-pass walk (node map → way segmentation) is the larger
   chunk, ~**1 week** with edge cases (relations ignored for v1, multipolygon ways, `oneway` parsing).
   **Mitigation**: scope to ways-only (skip relations/turn-restrictions in v1 — the deferred Step D/G handle
   those); OSMonaut is the documented fallback if the two-pass node resolution gets fiddly; the `:tools`
   synthetic-PBF round-trip test (A* step 7) catches reader/writer drift early.

5. **Turn-by-turn maneuvers + street names from raw OSM (no Google HTML) — harder than it looks?** Moderate
   risk, **mostly mitigated by choosing BRouter** (which already emits classified turn commands + carries the
   way `name`). The residual hard parts: (a) **roundabout exit numbering** and (b) **lane/exit guidance** are
   weaker in BRouter than Google; (c) **`name` coverage** — many Haifa service roads/footways are untagged, so
   `streetName` falls back to "Unknown road" more often than Google's. **Mitigation**: v1 maps BRouter's
   already-classified `VoiceHint.cmd` straight to `ManeuverType` (a pure table, no angle math); accept
   "Unknown road" for untagged ways (matches the existing default); the ≥ 80%-named-ways acceptance criterion
   *measures* coverage rather than assuming it. If hand-rolled A* is taken instead (gate failure), the
   turn-angle → maneuver classifier becomes net-new and **this risk jumps** — another reason BRouter is the v1
   pick.

6. **The one bootstrap GPS fix under jamming — what if it never arrives?** Very likely in jammed Haifa: the
   `SessionAnchor` may never be set. Then `current_vio_lla()` returns `valid=false` / `null` forever, so
   **map-matching, snap, and any lat/lng-anchored display are silent-disabled for the whole session** — the
   app runs on raw VIO local-frame only. **Mitigation (already in Step B* step 5)**: this is the explicit
   no-fix fallback — the app stays fully functional (VIO dot, no snap, no absolute geo), never crashes, and
   `vio_lla_unanchored_reads` proves the path was taken. Secondary mitigation for later: a **manual "I am
   here" anchor** (long-press the map to set `SessionAnchor` from a user tap) gives the matcher an anchor
   without GPS — *deferred*, noted here as the escape hatch if jamming makes the no-anchor session the common
   case rather than the exception.

7. **(bonus) Haifa-bbox trap + stale OSM data.** Two smaller risks. (a) A walk/route that **leaves the
   `34.94,32.74,35.10,32.86` bbox** gets no snap and no route (off-graph) — the user is "trapped" in-city.
   **Mitigation**: accepted for v1 (§0.6 Q2); the bbox is widened in the manifest if a test walk leaves it;
   the region manager (on-demand extra regions) is the deferred general fix. (b) **Stale embedded OSM** — a new
   road or a renamed street won't appear until the asset is rebuilt. **Mitigation**: the manifest records
   `upstream_pbf_date`; the (deferred) `osm_data_age_days` confidence factor + a rebuild cadence (ADR-020)
   handle freshness; for v1 a manual asset rebuild from a fresh Geofabrik PBF is the update path.