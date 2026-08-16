# NavSight — Master Architecture Guide

> **Audience:** the NavSight team, to study before the defense.
> **Project:** *NavSight — Beyond GPS* (Precision Navigation in GPS-Denied Environments) · Software Engineering B.Sc. final project.
> **Team:** Roey Ben Harush, Tamir Sobuh, Morad Zubidat · **Supervisor:** Mr. Amit Dunsky.
> **Validated build:** **v1.0-osm** (compileSdk/targetSdk 34, Android 14). *Android 15 / API 35 is the test-device runtime OS, not the build target.* *Refer to the build as "v1.0-osm" on slides — never quote a git hash.*
> **Target device:** Samsung Galaxy S21 Ultra (SM-G998B), **Exynos 2100** SoC, **Mali-G78** GPU (global variant — explicitly *not* Snapdragon).
> **Test region:** Haifa urban roads (scooter rides + indoor walks), under real regional GNSS jamming.

This document is the single, thorough reference for how NavSight is built. It covers every tier and every major subsystem, the *why* behind each decision (alternatives, tradeoffs, benefits, reasoning), the frame conventions, the end-to-end data flow, and an index mapping each diagram to what it illustrates. The slide deck (`presentation/slides/`) and diagrams (`presentation/diagrams/`) are the presentation surface; this guide is the study material behind them.

---

## 1. What NavSight Is (the one-paragraph thesis)

NavSight is a GPS-denied Android navigation app. It fuses the **rear camera + IMU** through a **15-DOF error-state Extended Kalman Filter**, snaps the resulting position to an **offline OpenStreetMap road-matching graph**, and shows a live **"ball on the road" with speed** — with **zero dependence on a live GPS fix**. The native core is **C++17 / Android NDK / OpenCV 4.5.3**; the UI is **Kotlin + Jetpack Compose**. The navigation is GPS-free and computed on-device: capture, fusion, and on-device OSM road-matching run on the phone with no GPS and no network in the navigation hot path. The displayed base map uses Google Maps tiles (a `GoogleMap` composable); OSM is only the road-matching graph that snaps the ball.

The mental model for the whole talk is the **"ball on the road"**:
- **Speed** advances the ball along the locked road.
- **Gyro-relative heading** steers it at junctions.
- **The road-matching graph** constrains it so it can never float into a field.

The single most important architectural property: **no GPS in the hot path.** GPS is sampled at ~1 Hz only as a secondary reference (and an optional one-shot startup alignment) — it never drives the displayed position. Do not over-claim "no GPS at all"; the precise framing is *no GPS in the hot path*.

---

## 2. Frame Conventions (memorize these — they get probed)

These conventions are the contract that keeps every subsystem consistent. Errors here are the source of the project's worst bugs (e.g. the ~800 m phantom Z-drift).

| Convention | Value / definition | Where it lives |
|---|---|---|
| **World frame** | **ENU, Z-up**, gravity `g = (0, 0, −9.81)` | EKF state, propagation |
| **Body→camera rotation** | `R_bc = diag(1, −1, −1)` | extrinsic, fixed |
| **World→body rotation** | `R_GtoI` (used in propagation `R_new = R_GtoI · ΔR`) | EKF propagation |
| **Working camera resolution** | 640 × 480 | KLT front-end |
| **Camera intrinsics** | `fx ≈ 451` | lens-correction, IPM |
| **Mount / camera height** | `h = 1.05 m` (camera above ground plane) | IPM speed |
| **Camera capture rate** | **locked 30 fps** = **33 ms** capture interval (`CameraUi.kt` `Range(30,30)`) | capture/timing |
| **Processed VIO rate** | **~23 fps** = **~43 ms** *effective* interval after `KEEP_ONLY_LATEST` frame-drop | VIO processing |
| **IMU rate** | accel + gyro **~50 Hz** (up to ~200 Hz at the preintegrator) | SensorRepository / IMUPreintegrator |
| **GPS rate** | ~1 Hz, **health-check / reference only** | SensorRepository |

**Z-up→Y-up boundary.** The native world is ENU **Z-up**; an explicit Z-up→Y-up frame conversion happens at the JNI boundary (`native-lib.cpp`, index 2→Y exposed) so the world-frame convention is handled in exactly **one place** before it reaches the Android map. If asked "where do you handle the frame swap," the answer is: at the JNI boundary, deliberately centralized.

**State dimension nuance (have ready, but keep "15-DOF" on slides).** The *navigation* state is the 15-DOF block `[attitude, gyro bias, velocity, accel bias, position]`. The C++ implementation actually carries **19 DOF** — it adds the camera-IMU time offset `δt_d` and the camera extrinsic `δφ_bc` (the latter receives only process noise and is MSCKF-skipped). Slides say **15-DOF**; the extra 4 are calibration nuisances. State carries MSCKF clones (`MAX_CLONES = 15`).

---

## 3. The Four Tiers

> **Diagram:** `diagrams/01-system-architecture.md` is the canonical picture of all four tiers. Study it first.
> **Note on vocabulary:** the signed SDD describes a three-tier *layered* architecture (`VisionModule` / `IMUPreintegrator` / `SensorFusionEngine`). The live code realizes this as **four tiers** with the JNI bridge as a first-class layer and the modules renamed `Tracker` / `IMUPreintegrator` / `EKFState`. Same design, more precise vocabulary. If challenged on the discrepancy, use that exact framing.

### Tier 1 — Presentation (Kotlin / Jetpack Compose)

**What it is.** Pure UI and sensor capture. `MainActivity → NavSightApp → MainScreen` (`MapScreenUi.kt`) with the `CameraUi` overlay, driven by `NavSightViewModel` (state holder using Compose `mutableStateOf`, ~21 observable props, **~200 ms throttle**). Composables: `MainScreen` (Google Maps `GoogleMap` base map + heading marker + path), `CameraUi` (CameraX preview + `ImageAnalysis`), `DebugPanelUi`, `BottomSheetUi`, `CalibrationScreenUi`, `StatusBadgesUi`. `SensorRepository` collects accel + gyro (~50 Hz, `SENSOR_DELAY_GAME`), fusedLocation (~1 Hz, health-check), and CameraX frames. Permissions: CAMERA + ACCESS_FINE_LOCATION. Also: `DeviceOrientationTracker`, `SimulationFrameRecorder`.

**Data direction.** One-directional and simple: `SensorRepository` → `NativeBridge` → native core → state lands back in `NavSightViewModel` → the `MainScreen` + `CameraUi` composables observe it.

**Why we chose this approach.**
- *Alternative considered:* pushing UI logic into the native layer for "speed."
- *Why not:* Android owns the UI thread, camera, sensors, and permissions. There is nothing to gain by fighting the Android lifecycle in C++, and much to lose. The boundary belongs exactly where it is.
- *Tradeoff accepted:* a JNI boundary to cross (addressed in Tier 2).
- *Benefit:* the UI stays light. The ~200 ms display throttle is fast enough to look live, slow enough not to thrash Compose recomposition — while the native core still processes every kept frame at the ~23 fps processed rate underneath. The state is always current; only the *display* is sampled at ~200 ms, so the ball is not laggy.

### Tier 2 — JNI Bridge (C++/JNI)

**What it is.** `NativeBridge.kt` (Kotlin singleton, ~47 `external` JNI functions — 48 declared, 1 commented — `System.loadLibrary("navsight")`) ↔ `native-lib.cpp`. It marshals **zero-copy `ByteBuffer`** camera frames *into* the native core (the `processCameraFrameDirect` entry takes 9 arguments) and returns a single batched **`VioData`** return type *out*. `VioData` is the 30-field JNI return type carrying pose/heading/status — it has **no `speed` field**; speed is derived Kotlin-side from successive positions plus heading/scale. A `state_mutex` + `shared_ptr` lifetime pattern keeps the native engine alive across `stopVIO`/restart without a crash. Heavy work runs on a dedicated `NavSight-VIO` executor thread.

**Why we chose this approach.**
- *Alternative considered:* copying frames across JNI as byte arrays; many small JNI calls per frame.
- *Why not:* at the ~23 fps processed rate, copying pixels every frame is wasteful, and chatty JNI is a correctness and performance hazard.
- *Tradeoff accepted:* dual-language object-lifetime / mutex bugs are real and were a genuine source of pain.
- *Benefit:* the bridge **moves a pointer, not pixels** (direct `ByteBuffer`), and returns one batched struct. Marshaling cost is negligible next to the 15.4 ms median spent inside the `Tracker` — so the JNI boundary is *not* the bottleneck. The `state_mutex` + `shared_ptr` pattern makes start/stop safe.
- *Reasoning:* a deliberately thin bridge keeps the "where does the frame swap happen" answer to a single place (Z-up→Y-up at the boundary) and keeps the two languages decoupled.

### Tier 3 — Native Processing Core (C++17 / OpenCV 4.5.3)

**`VioEngine`** is a thin orchestrator that owns **exactly two** heavyweight members. That deliberate minimalism is the architectural point.

1. **`Tracker` — the visual front-end.** Runs KLT optical flow, ORB relocalization, keyframes — and **owns the 15-DOF EKF (`EKFState`)** and all measurement updaters (MSCKF, ZUPT, gravity-alignment, map). It also owns `FeatureManager`, `TrackKLT`, `LensCorrector`, `UpdaterMSCKF`, `UpdaterZeroVelocity`, `InertialInitializer`, and `ScaleFuser`.
2. **`IMUPreintegrator`.** Madgwick AHRS attitude + gyro/accel preintegration (Forster midpoint), producing preintegrated `ΔR / Δv / Δp` deltas between frames.

**Why the `Tracker` owns the EKF (and not a separate fusion module).**
- *Alternative considered:* a standalone `SensorFusionEngine` separate from vision (this is the SDD's conceptual name for it).
- *Why not:* every EKF measurement update is fundamentally a *visual* event — a KLT track, an MSCKF feature, a zero-velocity detection, a map snap. Keeping the EKF inside the `Tracker` puts the state and its updaters next to the measurements that drive them, with no extra marshaling layer.
- *Why `IMUPreintegrator` stays separate:* it runs on a different clock (50–200 Hz vs ~23 fps) and only hands over preintegrated deltas.
- *Benefit:* each updater (MSCKF, ZUPT, gravity, map) is an isolated, independently testable unit — exactly what the deterministic replay harness exercises.

**Why `VioEngine` owns only two modules.**
- *Alternative:* a single mega-class that buries the EKF inside frame I/O.
- *Benefit:* a thin orchestrator + clear `Tracker` ownership of the EKF keeps each updater an isolated, independently testable unit and keeps the data path simple and modular.

### Tier 4 — Offline OSM Map-Matching (Kotlin)

**What it is.** `OsmDataLayer` loads the bundled Haifa OSM assets (geocode + segment R-tree) shipped in the APK. `LocalMatcher` is a fully on-device Newson-Krumm 2009 HMM/Viterbi matcher — the offline replacement for the OSRM `/match` URL. It produces a drop-in `MatchResult`. Companions: `LiveMatcher`, `GraphRailDot`, `OfflineGeocoder`, `OsmRouter`, `RoadRegionManager`. The displayed position is a graph-constrained "ball" — drawn over the Google Maps base map — advanced by estimated speed and steered at junctions by the gyro-relative heading offset.

**Why this is its own tier, and why offline.**
- *Alternatives considered:* Google Roads API, Directions API, OSRM `/match` (online).
- *Why not:* the deployment environment is GPS-jammed Haifa with unreliable connectivity. A cloud road-matching service adds latency, rate limits, a privacy surface, and a single point of failure exactly when navigation matters most — and sends the user's trajectory off-device.
- *Tradeoff accepted:* the APK carries the region's road-matching graph, so road-matching coverage is limited to bundled regions; the matcher's accuracy ceiling is the VIO trajectory quality rather than a remote service's.
- *Benefit:* no network latency, no rate limits, full privacy, and the headline — **jamming-resilient / GPS-denied-capable** road-matching. `LocalMatcher` is a drop-in `MatchResult` replacement, so the architecture above it "never knew the service went offline." (The displayed base map still uses Google Maps tiles; in airplane mode the navigation runs offline while the base map shows cached/blank tiles.)
- *Why bundle, not cache:* bundling guarantees the road-matching graph is present on first launch in a dead zone. For a GPS-denied use case, on-device road-matching that "works on first launch, offline" is a requirement, not an optimization. Production would download regional graph tiles on demand on top of the bundled core; the architecture doesn't change because `LocalMatcher` consumes whatever `OsmDataLayer` loads.

---

## 4. Subsystem Deep Dives (each with "Why We Chose This Approach")

### 4.1 KLT Optical-Flow Front-End

> **Diagram:** `diagrams/02-frame-lifecycle.md` (the frame's journey). **Reference image:** `tests/sims/val_2026_06_03b/probe_cruise_0.jpg` — amber = ground-plane sampling mask, green = FB-verified flow, red = raw production KLT.

**What it is.** Pyramidal Lucas-Kanade tracks the **same** corner points across consecutive frames — no per-frame re-detection, no per-frame descriptor matching. A **forward-backward consistency check** filters every track: track forward, track back, keep only points that return near origin. **ORB relocalization** is the recovery path when tracking is lost. The de-rotated, FB-verified flow is what IPM and the EKF consume.

**Why KLT, not per-frame descriptor matching.**
- *Alternative:* descriptor-matching every frame (ORB-to-ORB / ORB-SLAM-style).
- *Why not:* detecting, describing, and matching hundreds of features every frame does not fit a ~43 ms processed-frame interval with a 200 ms hard budget on a Mali-G78.
- *Tradeoff accepted:* KLT assumes small inter-frame motion + brightness constancy, so it degrades under large jumps/blur and can lose tracks.
- *Benefit:* continuous, dense, cheap frame-to-frame motion at the ~23 fps processed rate. We pay descriptor cost on demand (ORB relocalization on lost tracks) instead of ~23 times a second.
- *Reasoning:* "cheap continuous tracker + expensive on-demand recovery" is the standard, proven VIO front-end pattern.

**Why the forward-backward check.** KLT's own status flag is not enough — on a moving scooter, motion blur and parallel lane lines produce points KLT marks "tracked" but whose flow is meaningless. FB re-tracks each point in reverse and discards it unless it returns within a pixel threshold. Cost: roughly doubles KLT work (an extra backward track per point). Accepted because unfiltered tracks are the single biggest source of phantom speed. The green arrows in `probe_cruise_0.jpg` are the verified survivors; red is raw.

**Recovery evidence.** On the 2026-06-04 ride: **49 ORB relocalization events** (clean recoveries) and **36 looming (essential-matrix-degenerate) fallbacks** (the optical-flow looming path covering frames where the essential-matrix estimate is degenerate). The two mechanisms work together rather than masking failures. ORB relocalization and MSCKF covariance updates are the kept, verified-used recovery and update paths.

### 4.2 The Frame Lifecycle (six stages, ~15 ms)

> **Diagram:** `diagrams/02-frame-lifecycle.md`.

**Capture → Lens-Correct → KLT Track → Forward-Backward Check → De-Rotate Flow → Feed IPM + EKF → Publish.**

1. **Capture:** rear camera, 640×480, locked 30 fps / 33 ms capture; frames pass through `KEEP_ONLY_LATEST` to a ~23 fps / ~43 ms processed rate; zero-copy `ByteBuffer` into native (`processCameraFrameDirect`, 9 args).
2. **Lens-correct:** raw pixels → undistorted **normalized** coords (`x = X/Z, y = Y/Z`, via `LensCorrector`). All downstream residuals are normalized — `K` is never re-multiplied.
3. **KLT track:** pyramidal LK, raw flow `f_raw,i = p_i(t) − p_i(t−Δt)`.
4. **Forward-backward check:** keep iff `‖p_i − FB(p_i)‖ < ε`.
5. **De-rotate flow:** subtract the rotational component using gyro/Madgwick attitude → residual flow is translation-only: `f_i = f_raw,i − f_rot(ω, r_i)`.
6. **Feed + Publish:** clean flow → IPM speed + EKF update → publish `VioData` to the UI.

**Why this ordering (clean first, then fuse).**
- *Reasoning:* every cleaning step (undistort → FB-check → de-rotate) happens **before** the expensive fusion math, so the EKF only ever ingests trusted, translation-only measurements.
- *Why de-rotate before the estimators:* the IPM least-squares speed needs **pure translational flow** — any residual rotation biases the speed. We already have a high-rate gyro/Madgwick attitude, so subtracting rotational flow is cheap and removes the coupling before either consumer sees the frame. Gyro attitude is very accurate over a single ~43 ms processed interval (bias drift negligible at that timescale); longer-term gyro bias is separately estimated inside the EKF's 15-DOF error state.
- *Budget:* **15.4 ms median** processing vs the **200 ms** SDD budget → 13× under at median (2.5× even against a worst-case ~78 ms frame). The 13× / 2.5× margins are against the 200 ms SDD budget, not the inter-frame interval; a worst-case ~78 ms frame *exceeds* the ~43 ms processed interval and is absorbed by `KEEP_ONLY_LATEST` frame-dropping. KLT runs on **CPU** in OpenCV (the Mali-G78 matters for depth-model choices, *not* KLT).

### 4.3 IPM Ground-Plane Speed + Inertial Bridge

> **Diagram:** `diagrams/05-speed-estimation-ipm.md`. **Reference image:** `probe_cruise_0.jpg`.

**The geometry.** Model the road as a plane at calibrated mount height **h = 1.05 m** below the camera. Per road-pixel `i`:
```
Z_i      = -h / (n̂ · r_i)                        depth of road pixel i
a_i      = (u_fwd - r_i · u_fwd,z) / Z_i          flow per unit forward speed
v_i      = -(f_i · a_i)/(a_i · a_i) · (1/Δt)      per-point least-squares speed
σ_v,i    = (σ_px / fx) / (|a_i| · Δt),  σ_px = 0.5 px   per-point noise floor
```

**The two-sided vote taxonomy (the key idea).**
- **VOTE** iff `v_i > 3·σ_v,i` **AND** forward-coherent `cos θ_i < −1/√2` (a 45° cone).
- **ZERO-WITNESS** iff `|v_i| < 3·σ_v,i` **AND** floor `3·σ_v,i ≤ 1 m/s`.
- **Resolve:** ≥5 votes → `v = median{v_i}`; else ≥5 zero-witnesses → **EXACT 0** (zero-lock standstill); else **inertial bridge**.

**The complementary inertial bridge.** `a_fwd` = body-forward linear accel (specific force minus gravity projected on the horizontalized optical axis):
```
v_k⁻ = v_(k-1) + a_fwd · Δt                       predict (every frame)
v_k  = EMA(v_k⁻, median{v_i})                      correct toward vote-median (≥5 votes)
```
Mechanism: predict `v += a_fwd·Δt` every frame; once ≥5 votes return, correct by EMA toward the vote-median (hard-0 on ≥5 zero-witnesses). Prediction trusted up to the **~6 s** budget (138 frames at the ~23 fps processed rate; accel bias ≤ 0.3 m/s² → ≤ 1.8 m/s); past the budget it decays at rate `α = 0.15` (`kGpBridgeDecayAlpha`, the post-budget **decay** rate — not the correction gain).

**Why IPM ground-plane, not raw VIO scale.**
- *Alternative:* read speed directly from the EKF velocity state.
- *Why not:* monocular VIO scale is fragile and was the project's hardest-fought problem — a single shared scale chronically under-read on the scooter.
- *Benefit:* a **direct geometric** speed from one calibrated constant (h = 1.05 m) instead of a drifting estimated scale.
- *Tradeoff:* assumes locally-planar road, degrades on slopes/curbs — which is exactly when the inertial bridge takes over.
- *Why no learned depth in the speed path:* the speed-path metric depth comes from the calibrated ground plane (h = 1.05 m), **not** learned depth. (Learned metric-depth benches were non-viable on the Mali-G78 anyway.)

**Why two-sided, not a one-sided magnitude gate.** A one-sided "is the speed big enough" gate rectifies up to **~34%** (analytical estimate, `Tracker.cpp:9302`) of pure random-direction tracker noise into a positive speed → phantom 1–5 km/h while standing still. Adding the forward-coherence cone (direction must agree, not just magnitude) and a zero-witness branch classifies noise as zero instead of motion. **This is the single decision that makes standstill read *exactly* zero** — on the validation-ride stop the displayed speed reads exactly 0.0 km/h throughout (the exact-0 zero-witness lock held for ~27 frames), never exceeding ~0.9 km/h.

**Why the complementary bridge.** Vision-only would have *no* speed during blur/feature-starvation (77% of cruise frames showed starvation). The bridge predicts through those gaps and re-anchors when votes return. A/B replay on identical input: **62 m → 77 m (+24%)** integrated distance on a turn-heavy ride — the missing distance the gaps were eating (stored deterministic offline A/B replay result; hardcoded, not recomputed live). The ~6 s bridge budget (138 frames at the ~23 fps processed rate) is real. The EMA correction is a deliberately gentle blend so a single noisy median can't jerk the speed.

**Framing to land:** *geometry gives the number, the two-sided gate gives the trust, the bridge gives continuity — three separate jobs.*

### 4.4 The 15-DOF Error-State EKF

> **Diagram:** `diagrams/06-ekf-pipeline.md`.

**State** (error-state; ENU **Z-up**, `g = (0,0,−9.81)`): the 15-DOF navigation block `[δθ, δb_g, δv, δb_a, δp]`, plus MSCKF clones (`MAX_CLONES = 15`). Rows: 0–2 `δθ` (world), 3–5 `δb_g` (body), 6–8 `δv` (world), 9–11 `δb_a` (body), 12–14 `δp` (world). (For precision: the full IMU error-state is 19-DOF, adding the camera-IMU time offset and body→camera extrinsic; slides say 15-DOF.)

**Propagation** (Forster midpoint, preintegrated deltas):
```
R_new = R_GtoI · ΔR
v_new = v_G + g·Δt + R_GtoI^T · Δv
p_new = p_G + v_G·Δt + ½·g·Δt² + R_GtoI^T · Δp
P     = Φ·P·Φ^T + Q
```

**Updates** via Joseph form `P = (I−KH)·P·(I−KH)^T + K·R·K^T` with per-row **Huber** (δ = 2.4477 = √χ²(0.95,2)). Updaters: **MSCKF** visual, **gravity-alignment** (keeps `p_G` physically bounded — observes 2-DOF roll/pitch from accel direction; yaw unobservable; illustrative residual band `g ± 0.8 m/s²`), and **ZUPT** (sets `v_G = 0` when stationary). The live per-frame gate is the **MSCKF Mahalanobis chi-squared gate** (5× χ²₀.₉₅(2K); landmark per-obs gate 5.991) with per-row Huber and the Joseph-form covariance update — it rejects physically impossible residuals.

**Why a tightly-coupled error-state EKF.**
- *Alternatives:* loosely-coupled blend; optimization-based sliding-window VIO (VINS-Mono); pure feature-SLAM (ORB-SLAM).
- *Why not loosely-coupled:* averaging a visual pose and an inertial pose separately loses the cross-constraints.
- *Why tight:* the IMU can directly constrain visual scale and rotation, and the filter degrades gracefully (inertial bridge carries dead-reckoning ~6 s).
- *Why not VINS-Mono / ORB-SLAM:* both are heavier and were unnecessary once map-matching took over global drift control.
- *Tradeoff accepted:* filter consistency is genuinely hard — gravity must cancel exactly during propagation; a ~6–10° tilt once leaked into ~800 m of phantom Z drift. Mitigated by the dedicated gravity-alignment update.
- *Benefit:* low CPU, a principled covariance, a clean dead-reckoning fallback.

**The gravity-drift lesson (cautionary tale).** A ~6–10° `R_GtoI` tilt mis-cancelled gravity during propagation, leaking into ~800 m of phantom Z drift (`EKFState.h:420-423`). The team's instinct was to spend days tuning constants — but the real fix came from **reading the residual data**, which pointed straight at the tilt; the fix landed at the gravity-alignment layer (a live updater), not by loosening a gate. **Project cardinal rule: read the data before tuning the constants.**

### 4.5 HMM / Viterbi Map-Matching + Graph-Rail Ball

> **Diagram:** `diagrams/04-map-matching-hmm.md`. **Reference image:** `tests/sims/val_2026_06_03b/routeA_matched_traj.png` (matched trajectory, colour = time; U-turn retraces the same road).

**The model.** Candidate road segments within **30 m** are HMM states (Newson & Krumm 2009), Viterbi-decoded:
```
log p_e = -d_perp² / (2·σ_z²),                  σ_z = 20 m            (emission)
log p_t = -|d_gc - d_route| / β + b_way + b_rail, β = 6, b_way = 0.7, b_rail = 1.2  (transition)
s*_{1:N} = argmax_s Σ_k (log p_e + log p_t)                          (Viterbi)
```
**Graph-rail ball:** `s_{k+1} = s_k + v_k·Δt` (advance along the locked way by speed); at a junction `b* = argmin_b |θ_b − (θ_anchor + Δψ_gyro)|` (branch by gyro-relative heading). **Wrong-fork recovery bounded 25–60 m at confidence ≥ 0.55** — never teleports across town.

**Why the matcher is the source of truth for position (not the raw trajectory).**
- *Alternative:* display the raw integrated VIO path.
- *Why not:* VIO accumulates drift; the trajectory is the *ceiling* on accuracy.
- *Benefit:* the matcher does double duty — displays a road-clean line and absorbs lateral drift. Evidence: the U-turn retraces the *same* road rather than smearing across two.

**Why HMM/Viterbi, not nearest-road snap.**
- *Why not nearest-snap:* it is memoryless — at a fork it flips to whichever segment is momentarily closest, causing teleports between parallel roads.
- *Benefit:* the transition term forces the chosen road to be both close (emission) AND reachable by the distance actually moved (transition). Viterbi optimizes the whole sequence jointly, so a single noisy frame can't derail the path. The same-way (0.7) and rail (1.2) bonuses bias the decode toward staying on the locked way (`lockedWayId`) — this kills the parallel-road teleport that motivated the rail-follower design.

**Critical decoupling to land.** *The matcher is the source of truth for position; speed is the source of truth for distance.* They are decoupled — which is why distance accuracy survives even when the snap is doing heavy lifting. **Do not** claim the matcher corrects along-track distance; it fixes lateral/topological error only. Distance comes from the IPM speed estimator (4.3).

**Why σ_z = 20 m.** It is the Newson-Krumm working value and must cover VIO lateral drift + road-centerline-vs-lane offset. Tighter would over-trust an estimate with tens of metres of genuine lateral uncertainty and cause the matcher to thrash between candidates; 20 m keeps the emission soft so the transition term decides at forks.

**Roundabout safety.** A ring-eject mechanism: on the validation ride the app started *on* a roundabout, detected at t=0, and exited cleanly at **~9 s** (observed, approximate) with no spurious re-entry. The mechanism is code-confirmed (RING EJECT, 460 ticks, conf 0.80): the exit re-acquires the ball onto the suggested exit segment with re-entry hysteresis, so a circling trace can't keep matching its own ring.

### 4.6 IMU Preintegration + Heading (Madgwick AHRS)

**What it is.** `IMUPreintegrator` runs the Madgwick attitude filter and Forster midpoint preintegration, handing the EKF preintegrated `ΔR / Δv / Δp` between frames. It runs at 50–200 Hz, decoupled from the ~23 fps processed visual clock.

**Heading.** Gyro-primary Madgwick with a **one-shot compass alignment at start**; nudged toward the matched road tangent **only** on straight, non-circular roads: `ψ ← ψ + κ·δ iff |δ| < 35°`, with the ±180° ambiguity resolved travel-aligned. A crossing ~90° road is rejected.

**Why gyro-primary with a one-shot compass.**
- *Reasoning:* the gyro is reliable over the few seconds a junction takes (where the road-tangent nudge does its branch-selection work) and free of magnetic distortion; the one-shot compass only seeds the initial absolute heading.
- *Tradeoff:* a bad >90° startup compass snap was once unrecoverable — addressed by a `maybeFixBackwardsHeading` cluster detector near ±180° and deferring acquire until a travel bearing is established (~2 m).

---

## 5. End-to-End Data Flow (sensor → screen)

> **Diagram:** `diagrams/03-data-flow.md`.

```
Rear camera (640×480, 30 fps capture → ~23 fps processed) ─┐
IMU (accel+gyro, ~50 Hz) ───────┤
GPS (~1 Hz, health-check only) ─┘ (out of loop, "verify only")
        │
        ▼  (zero-copy ByteBuffer across JNI)
  ┌─────────────────────────────────────────────┐
  │  Native VIO core (C++17 / OpenCV 4.5.3)       │
  │  VioEngine orchestrates:                      │
  │   • IMUPreintegrator (Madgwick + preint.)     │
  │   • Tracker (KLT + ORB) → owns 15-DOF EKF      │
  └─────────────────────────────────────────────┘
        │ Speed (IPM + inertial bridge)   │ Pose / heading
        ▼                                  ▼
  ┌───────────────────────────────────────────────┐
  │  Offline OSM map matcher (Kotlin)              │
  │  Viterbi HMM (LocalMatcher) →                  │
  │  Graph-rail ball: speed advances it,           │
  │  gyro-relative heading steers it at junctions  │
  └───────────────────────────────────────────────┘
        │ matched position + heading (back across JNI as VioData; speed derived Kotlin-side)
        ▼
  NavSightViewModel (Compose mutableStateOf, ~200 ms throttle)
        ▼
  MainScreen (Google Maps base map) + CameraUi — "ball on the road" + live km/h
```

**The one sentence that must land:** *speed advances the ball, gyro-relative heading steers it, the road-matching graph constrains it.* Position is graph-constrained; distance + turn decisions come from the VIO core. That decoupling is why the ball never leaves the road yet still travels the correct distance.

**Graceful degradation.** When vision fails (blur, darkness, feature starvation), `IMUPreintegrator` keeps feeding motion and the speed estimator predicts from forward acceleration for up to ~6 s, so the ball keeps moving. On the 2026-06-04 ride: 36 looming (essential-matrix-degenerate) fallbacks + 49 ORB relocalization events — every recovery without a network call.

---

## 6. The Live Pipeline (v1.0-osm) — What Actually Runs

The validated build ships a deliberately lean VIO pipeline. The subsystems that run, end to end, are:

- **KLT optical flow** + **forward-backward check** + **de-rotation**
- **ORB relocalization** (on lost tracks) and the **MSCKF covariance update**
- **ZUPT / ZRUP / gravity-alignment / stationary-accel** detectors
- **IMU preintegration** (Madgwick + Forster midpoint)
- **IPM ground-plane speed** + the vote / zero-witness taxonomy + the inertial/accel bridge
- **Offline OSM road-matching** (`LocalMatcher` / Viterbi HMM)

Each updater is an isolated, independently testable unit — exactly what the deterministic replay harness exercises. The speed-path metric depth comes from the calibrated ground plane (h = 1.05 m), not learned depth.

---

## 7. Why GPS Is Not Ground Truth (and Why We Keep It Anyway)

Under real GNSS jamming in Haifa, the Route-A measurement shows jammed GPS **over-reported by +33%** (GPS 1,705 m on a route that truly measures 1,280 m; NavSight reported 1,195 m). On other jammed rides we also observed multi-second GPS position freezes while moving. Therefore **map-measured routes are the jamming-resilient ground truth**, and GPS is only a *secondary* reference on rides whose GPS health is verified (median fix accuracy ~4 m, no freeze/jump artifacts).

We never remove GPS: it is a free validation signal on clean days, and removing it would discard that signal. But it is **not in the navigation hot path** — the displayed position is driven entirely by camera + IMU + the OSM road graph. (A separate sim-pipeline bug — resampling the same fix every tick — was once mistaken for jamming; GPS was actually healthy on those rides.)

---

## 8. Diagram Index — Which Diagram Illustrates What

| Diagram file | Illustrates | Use it on slides | Key labels to point at |
|---|---|---|---|
| `diagrams/01-system-architecture.md` | The full **4-tier system** (Tier 1 Compose UI → JNI bridge → `VioEngine` owning `Tracker`+EKF and `IMUPreintegrator` → offline OSM matcher → ball) | **06** High-Level Architecture, **07** Application Architecture | Zero-copy `ByteBuffer`; `VioEngine` owns exactly two modules; EKF speed+pose → matcher → ball; no network, no live GPS |
| `diagrams/02-frame-lifecycle.md` | The **single-frame journey** as a sequence diagram (Capture → lens-correct → KLT → FB-check → de-rotate → IPM+EKF → throttled UI) | **12** Frame Lifecycle, **13** KLT Tracking | 30 fps capture → ~23 fps / ~43 ms processed; 15.4 ms median vs 200 ms budget (~78 ms worst frame absorbed by frame-drop); MSCKF Mahalanobis chi-squared gate; ~200 ms UI recompose |
| `diagrams/03-data-flow.md` | **End-to-end data flow** sensors → native core → estimates → map matching → UI; GPS shown out-of-loop ("verify only") | **08** Infrastructure, **11** Runtime Overview, **16** Data Flow | Speed advances the ball; heading steers at junctions; GPS dotted "verify only" |
| `diagrams/04-map-matching-hmm.md` | **HMM map-matching** — candidate segments (30 m) as states, emission/transition terms, Viterbi, junction steering, bounded recovery | **14** Map Matching | σ_z = 20 m; β=6, b_way=0.7, b_rail=1.2; recovery 25–60 m @ conf ≥ 0.55 |
| `diagrams/05-speed-estimation-ipm.md` | **IPM speed** — ray → ground plane (h=1.05 m) → depth → flow-per-speed → per-point speed + noise floor → two-sided VOTE/ZERO-WITNESS taxonomy → median / exact-0 / bridge | **15** Speed Estimation | h=1.05 m; σ_px=0.5 px; 3σ gate + 45° cone; ≥5 votes / ≥5 zero-witnesses; α=0.15; ~6 s bridge |
| `diagrams/06-ekf-pipeline.md` | **15-DOF error-state EKF** — state, IMU propagation (Forster), Joseph-form updates (MSCKF / gravity / ZUPT), MSCKF Mahalanobis chi-squared gate, bounded output | (supports **07** Application Architecture / EKF Q&A) | ENU Z-up, g=(0,0,−9.81); R_new = R_GtoI·ΔR; MSCKF Mahalanobis chi-squared gate + per-row Huber, Joseph form; gravity alignment keeps p_G bounded |

**Companion reference images** (real validation assets, relative to repo root):
- `tests/sims/val_2026_06_03b/probe_cruise_0.jpg` — rear-camera frame: amber = ground-plane sampling mask, green = FB-verified flow, red = raw KLT (use for slides 12/13/15).
- `tests/sims/val_2026_06_03b/routeA_matched_traj.png` — matched trajectory, colour = time, U-turn retraces same road (slide 14).
- `tests/sims/val_2026_06_03b/routeA_google_measure.png` — Google Maps 1.28 km ground truth.
- `tests/sims/val_2026_06_03b/routeA_cumdist_gps_vs_dot.png` — cumulative distance: GPS 1,705 m inflated vs NavSight 1,195 m on a 1,280 m route.
- `tests/sims/val_2026_06_12/ui3.png` — live map UI: 0 km/h at standstill, green VIO chip, ball on the road (Google Maps base map; ball snapped by the offline OSM road-matching graph).
- `tests/sims/val_2026_06_12/ui_cam.png` — camera screen with live overlay.
- `Final-Project/SDD/דוח אימות.pdf` — the signed V&V report (Figs 1–11; per-figure page renders in `tests/sims/val_2026_06_12/report_render/p1.png`–`p11.png`).

---

## 9. Numbers Cheat-Sheet (have these ready)

| Quantity | Value |
|---|---|
| Camera | 640×480; locked 30 fps / 33 ms capture → ~23 fps / ~43 ms processed (after `KEEP_ONLY_LATEST`); `fx ≈ 451` |
| IMU / GPS | ~50 Hz / ~1 Hz (health-check only) |
| Per-frame tracking | median **15.4 ms** vs **200 ms** SDD budget (13×; ~78 ms worst frame still 2.5× under, absorbed by frame-drop) |
| EKF | 15-DOF (impl. 19); MSCKF clones cap 15; Huber δ = 2.4477; **MSCKF Mahalanobis chi-squared gate** (5× χ²₀.₉₅(2K); per-obs 5.991), Joseph form |
| IPM | h = 1.05 m; σ_px = 0.5 px; 3σ vote + 45° cone (cos θ < −1/√2); ≥5 votes / ≥5 zero-witnesses |
| Inertial bridge | predict v += a_fwd·Δt; EMA correct on ≥5 votes; trusted ~6 s (138 frames @ ~23 fps); post-budget decay α = 0.15; bias ≤0.3 m/s² → ≤1.8 m/s |
| Map matching | candidate radius 30 m; σ_z = 20 m; β = 6; b_way = 0.7; b_rail = 1.2; recovery 25–60 m @ conf ≥ 0.55 |
| Distance accuracy | Route A: 1,195 / 1,280 m = **93.4%** (ρ 0.93); Ride 18:02: ~792 / 869 m = **91%** (ρ 0.91) |
| Speed accuracy | RMSE **8.9 km/h**, bias **−1.9 km/h**, median 36.3 vs 35.1 km/h (within 3.5%); ref noise floor ~4–6 km/h @ 5 s |
| Standstill | displayed speed exactly 0.0 km/h throughout the stop (exact-0 zero-witness lock; ~27 frames); never exceeds ~0.9 km/h |
| Inertial bridge A/B | 62 → 77 m = **+24%** (stored deterministic offline A/B replay result) |
| Fallback/recovery | 36 looming (essential-matrix-degenerate) fallbacks, 49 ORB relocalization events (2026-06-04) |
| Roundabout | started on ring, detected t=0, clean exit ~9 s (observed); RING EJECT, 460 ticks, conf 0.80 |
| GNSS jamming | Route A: jammed GPS +33% inflated (1,705 m vs true 1,280 m → NavSight 1,195 m); multi-second GPS freezes observed on other jammed rides |
| Tests | 95-case Kotlin unit suite (95 `@Test` across 14 files); C++ unit suite + deterministic replay + CI scoring |
| Offline assets | bundled Haifa OSM assets in APK |

---

## 10. Defense Pitfalls — Things NOT to Say

- **Don't** say "the system doesn't use GPS at all." Say: *no GPS in the **hot path*** (GPS = ~1 Hz secondary reference / validation only).
- **Don't** claim KLT runs on the GPU. KLT runs on **CPU** in OpenCV; the Mali-G78 matters for **depth-model** choices, not KLT.
- **Don't** claim the map matcher *corrects along-track distance*. It fixes **lateral/topological** error; **distance comes from the IPM speed estimator**.
- **Don't** say learned depth feeds the speed path. The IPM speed path uses **plane + mount height only** (h = 1.05 m) — no learned depth.
- **Don't** say the displayed base map is OSM. The base map is **Google Maps tiles**; OSM is only the road-matching graph that snaps the ball.
- **Don't** over-claim "fully offline app." The precise framing is *GPS-free navigation computed on-device (offline OSM road-matching); base map uses Google Maps tiles.* In airplane mode the navigation runs offline while the base map shows cached/blank tiles.
- **Don't** quote a git hash on slides. The build is **v1.0-osm** (compileSdk/targetSdk 34, Android 14; API 35 is the test-device OS).
- **Don't** call the camera rate "native ~23 fps." Camera capture is **locked 30 fps / 33 ms**; ~23 fps / ~43 ms is the **processed** VIO rate after frame-dropping.
