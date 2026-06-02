# Road-Follower Matcher + Heading Leg — Design (2026-05-31)

Source: Opus root-cause workflow on the 3 real scooter sims (tests/sims/val_2026_05_31_scooter/, GPS ground truth).
Confirms the owner's diagnosis: **the matcher follows the VIO trajectory, not the road.** All findings `verified`
against code. (Workflow synthesis step hit the session limit; this doc is reconstructed from its completed reports.)

## ROOT CAUSE — why it snaps to parallel roads + inflates MM path 1.2–2.1× GPS
1. **Emission is perpendicular-distance-only, no bearing.** `LocalMatcher.kt:68 emit=-0.5*(offsetM/SIGMA_Z)^2`,
   SIGMA_Z=20m, SEARCH_RADIUS_M=30m (bearings were removed app-wide because they collapsed confidence). A VIO window
   drifted 15–30° but with the right arc-LENGTH lands inside a *parallel* road's Gaussian.
2. **Transition is identity-blind.** `LocalMatcher.kt:78-79 trans=-|dGc-dRoute|/BETA`, BETA=6. A parallel road of
   equal local geometry costs ~0 to jump to. (This is why K_map≈1 while the road IDENTITY is wrong — misleading.)
3. **No committed-road state.** `SAME_WAY_BONUS=0.7` exists but `match()` starts every ~500ms tick with `prev=null`
   and rebuilds the trellis from scratch (`LocalMatcher.kt:61`, `NavSightViewModel.kt:642-650`). So it re-decodes
   the whole window every tick and **follows the VIO drift** → the Viterbi terminal jumps between non-connected roads
   (the TELEPORT) → arc concatenation inflates `routeDistanceM` to 1.2–2.1× GPS.
4. **Roundabout latch (2nd inflation source).** ON_ROUNDABOUT fires 48–55% of samples — NOT density (only 5/3/1
   episodes, 20–39s each). Enter gate tests distance-to-CENTER (disc R+12m, `ManeuverStateMachine.kt:101/200`,
   `RoundaboutModel.kt:32-40`), so a straight pass near a circle latches; exit needs distToCenter>R+15 but the Fix-C
   freeze holds the dot near center so it never exits; no sweep-completion/dwell exit. RoadSnapper then pins the dot
   to the ring (`RoadSnapper.kt:137-146`), adding ring-arc.

ROOT CHAIN: Madgwick yaw drifts 16–66° → dot leaves road (`Tracker.cpp:3990-3991 dx=disp*sin(hdg),dy=disp*cos(hdg)`,
hdg=Madgwick `scalar_heading_`, NOT EKF yaw) → drifted-but-same-length window re-matches a parallel road (1,2) →
no rail state → teleport (3) → MM inflates. Roundabout latch (4) compounds it.

## THE FIX — two independently-falsifiable steps

### STEP 1 — RAIL-LOCK (Kotlin only; does NOT touch heading) — the owner's "follow the road"
Add the missing primitive: **committed-road state carried across `match()` calls.**
- Add `railWayId` + along-track position (session-scoped) in/around LocalMatcher; change `match()` to take the prior
  lock + measured along-track distance + the ManeuverStateMachine result.
- While locked + confident: **advance the matched point ALONG railWayId's polyline by the measured along-track
  distance** (fused_speed·dt projected onto the road tangent via `RoadSnapMath.projectPointOntoSegment`,
  `RoadSnapper.kt:158`), constraining candidates to the rail. Do NOT re-pick the nearest road each window.
- **Change road ONLY on a maneuver** from the heading-delta classifier (below). Recovery: if confidence collapses
  for N ticks, drop the lock and re-decode freely (re-acquire).
- Reuses `MIN_MATCH_CONFIDENCE` (`NavSightViewModel.kt:61`) for initial lock; a strong (>>0.7) same-as-rail bonus
  while locked. No magic numbers beyond what's tuned on the sims.
- **Maneuver-from-heading-delta classifier** (runs on the existing ~500ms SM tick, `headingDeg` already passed in):
  - FREE_ROAD/rail = |sweep rate| below a straight-road noise floor (few °/0.5s; arm jitter ~10°/s per Tracker.h:241).
  - ~90° accumulated **then settles** + new heading matches a connecting way (±EXIT_BEARING_TOL_DEG=35) → INTERSECTION
    turn → switch to the crossing way.
  - ~180° with small displacement → MID-ROAD U-TURN → keep rail, flip TravelDirection.REVERSED (existing
    `detectMidRoadUTurn`: UTURN_HEADING_THRESH_DEG=160, UTURN_MAX_DISPLACEMENT_M=20, UTURN_WINDOW_TICKS=6).
  - continuous same-sign sweep that doesn't settle → ROUNDABOUT → follow ring; exit on sweep→0 + arm match (existing
    pickExit).
- **Roundabout-latch fix (same step, pure-Kotlin):** (a) enter on distance-to-ANNULUS `|dist-radius|<=ENTER_MARGIN`
  not center-disc; (b) add sweep-completion/dwell exit; (c) gate exit on snapped wayId≠ring.wayId (robust to the
  freeze).
- **FALSIFIER (offline, before any device build):** re-run the 3 scooter sims' recorded VIO track (vlat/vlng) through
  the modified matcher + SM via a JVM test (extend LocalMatcherTest / ManeuverStateMachineTest). Assert: MM path-vs-GPS
  drops from 2.09×/1.19×/1.28× toward ~1.0×; ON_ROUNDABOUT fraction drops from ~0.47 to a few %; no teleports between
  non-connected roads.

### STEP 2 — HEADING LEG (C++; heading now UNLOCKED by owner) — fixes the dot-drift at the source
Road-bearing → Madgwick-yaw nudge via the EXISTING gated channel `IMUPreintegrator::nudgeMadgwickYawAroundWorldZ`
(the SAME one the visual nudge `Tracker.cpp:5404` kBug5SyncStrength=0.10 / residual gate π/4 and loop-closure
`Tracker.cpp:8005` use). When confidently railed on a LONG STRAIGHT road of known bearing B and !imu.isMagActivelyFusing():
nudge `scalar_heading_` a fractional step toward B. Gated HARD (high-conf + long + straight + forward-progress +
single dominant candidate + |hdg−B|≤35° so it can't re-aim onto a crossing road), rate-limited, **default-OFF**,
validated ALONE on the replay harness. The 35° disagreement gate is the safety property (a 90°-off crossing road is
rejected → only slow-drift correction, never re-aiming). Do NOT use a snap (k≠1.0). Comment-out-not-delete.

## SEQUENCING + CAVEATS
- **Step 1 (rail-lock) first** — Kotlin-only, no heading risk, directly kills the teleport/inflation; falsifiable
  offline on the 3 sims. **Step 2 (heading leg) second**, behind its own flag, default-OFF, validated alone.
- **GPS:** owner asked *if* it would help (course→heading, speed→scale) — it would, but it is NOT authorized; the
  chosen path is the road-follower, not GPS. Keep ADR-004 intact.
- **⚠ SPEED CAVEAT (does NOT fix the 1km/5% along-track goal):** rail + heading fix CROSS-track (keeps the dot on the
  road). They do NOT fix the along-track distance error (sim1 VIO 1068m vs GPS 727m = 1.47× over; sims 0.79× under).
  A rail that "advances at the measured speed" INHERITS the speed error → overshoots/undershoots the real exit even
  with perfect heading. The speed under-read (15→3-4 km/h, median 0.42-0.70× of GPS, + 300-600 km/h spikes) is the
  SEPARATE per-mode-VEHICLE-K / spike-guard lever and must be fixed too for 1km/5%.
