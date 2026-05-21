# BUG_04 — UI Render Pipeline (orange-dot flicker + heading display chain)

Worker #4 (worker-04-render), hive-1779372411394-kzq0l2, navsight-bug-queen.
Walks consulted: heading_walk_1/2_2026_05_20, parallax_fix_walk_2026_05_20,
promo_parallax_walk_2026_05_21, bug3_walk_2026_05_21.

---

## A. HEADING DISPLAY CHAIN — VERDICT: KOTLIN-SIDE CHAIN IS FAITHFUL

### A.1 End-to-end trace

| Layer | Site | Type / Range |
|---|---|---|
| Madgwick yaw | `IMUPreintegrator::getHeading()` | radians, signed `[-π,π]` |
| Tracker copy | `Tracker.cpp:2774` `scalar_heading_ = imu.getHeading()` then `Tracker.cpp:2775-2776` wrap to `[-π,π]` | radians |
| VisionOutput.heading | `Tracker.cpp:5357` `out.heading = scalar_heading_` | radians |
| JNI global | `native-lib.cpp:529` `g_heading = output.heading` | radians |
| JNI return | `native-lib.cpp:605` `ret_heading = g_heading` returned via `VioData` ctor | `Double`, radians |
| Kotlin model | `VioData.kt:52` `val heading: Double = 0.0` (KDoc unit-comment missing, but C++ caller writes radians) | radians |
| Repository | `SensorRepository.kt:949` `_vioState.value = vio` (raw passthrough) | radians |
| ViewModel publish | `NavSightViewModel.kt:296` `vioState = vio` — gated by 200 ms `UI_UPDATE_THROTTLE_MS` (`NavSightViewModel.kt:220`) | radians, 5 Hz |
| MapScreen conversion | `MapScreenUi.kt:40-42` `fusedHeading = ((toDegrees(vio.heading).toFloat() % 360f) + 360f) % 360f` | float deg, `[0, 360)` |
| Debug panel render | `DebugPanelUi.kt:73` `"${"%.1f".format(headingDeg)}°"` | one decimal degree |

Bench-tested the wrap math at all boundary cases (rad=-π, +π, ±0.1, ±1): signed-radians convert to wrapped-degrees correctly. No double-wrap, no off-by-360. (`scripts`-free check via inline Python, see investigation notes.)

### A.2 Empirical validation against `heading_walk_1_2026_05_20.json` (n=2355 samples)

- Start: `hdg=3.008 rad` → `172.33°` displayed. Matches user's first observation **exactly**.
- First 30 samples (~1 s of walking): heading drifts smoothly `172.33° → 174.51°` with per-frame deltas of **+0.05 to +0.36°/frame** (median ~0.1°). No discontinuities, no spikes.
- 200-ms-throttled sampling (every 6th sample, mimicking debug panel cadence at `UI_UPDATE_THROTTLE_MS=200`): values `172.33, 173.11, 173.67, 174.10, 174.22, 174.58, ...`. Smooth. **No flicker, no jumps introduced by throttle.**
- The user's reported sequence `172° → 203° → 235°` exists in the data, but **not at adjacent timestamps**: 235° is at t≈51.0 s (sample idx 1527), 203° at t≈53.4 s (idx 1601), 172° at t≈56.7 s (idx 1702). And the start sample is 172.3°. These are **isolated snapshots from disparate moments** during a 78 s walk that traverses the full `[0, 360°)` compass range while the user makes corner turns.

### A.3 Sensor-radar arrow vs debug-panel number — IN PHASE

Both consume the same `fusedHeading` derived at `MapScreenUi.kt:40-42`:
- Debug panel: `MapScreenUi.kt:263` passes `fusedHeading` to `DebugPanel`.
- Radar arrow: `MapScreenUi.kt:227-228` `val radarHeading = (Math.round(fusedHeading / 2f) * 2f)` then `SensorRadarWaze(history, radarHeading, pal)`. The radar quantizes to 2° steps for visual stability; debug panel shows 0.1° resolution. Same source, different display granularity — by design.

Inside `SensorRadarWaze` (`MapScreenUi.kt:527-529`): arrow tip computed as `tipX = cx + sin(toRadians(currentAzimuth)) * aLen` / `tipY = cy - cos(currentAzimuth) * aLen`. Standard compass convention (0°=North up, +90°=East right). Confirmed consistent.

### A.4 Heading verdict

**The Kotlin/JNI heading chain is FAITHFUL. The user-reported `172° → 203° → 235°` over 2 loops is NOT a render bug — it is BUG_02 (Madgwick gyro-bias drift ~0.28°/s) manifesting as cumulative heading drift across a multi-turn walk.** No fix belongs to BUG_04 here.

---

## B. ORANGE-DOT FLICKER — VERDICT: FOUR DISTINCT ROOT CAUSES

User report: "orange dots appear and disappear randomly."

### B.1 Per-walk counter snapshot (5 walks)

| walk | anchor (a) | world-fixed (w) | pixel-refresh (r) | snapshots | a/(a+w) | r/a |
|---|---:|---:|---:|---:|---:|---:|
| heading_walk_1 | 327,305 | 267,639 | 46,253 | 2,484 | 0.550 | 0.141 |
| heading_walk_2 | 275,301 | 207,515 | 25,847 | 2,374 | 0.570 | 0.094 |
| parallax_fix | 390,296 | 245,096 | 56,601 | 2,798 | 0.614 | 0.145 |
| promo_parallax | 350,323 | 276,109 | 53,870 | 2,600 | 0.559 | 0.154 |
| bug3 | 314,094 | 249,618 | 37,005 | 2,425 | 0.557 | 0.118 |

Per-overlay-snapshot (one frame): ~130 anchor + ~95 fallback dots drawn. Anchor-hit ratio steady at **55–61 %**, meaning ~40 % of every overlay frame relies on the projection-fallback path — those dots are subject to EKF pose drift and ARE the population that "moves around" on the screen.

### B.2 Root cause #1 — "Observed" boolean toggles per keyframe (Tracker.cpp:5076-5085 vs 4717-4720)

`native-lib.cpp:1411` `is_observed = observed_set.count(id) > 0` derives the orange-vs-gray color from `last_observed_landmark_ids_`. That ids vector is rewritten at **keyframe rate (~1 Hz)** by the producer (Tracker.cpp:5076-5077):

- Path A — keyframe with matches: ids vector replaced with `accepted_ids` (typically ~10-20 entries).
- Path B — keyframe with `nearby_ids.empty()` (Tracker.cpp:4705): `ids.clear()` only, **pixels NOT cleared** (Tracker.cpp:4717-4720). Sets the size invariant `ids.size() == pixels.size()` temporarily false. `getLastObservedLandmarkPixel` iterates ids first so this is safe-by-luck, but it means every orange dot turns gray for one keyframe period when the local map yields zero matches.

Between consecutive keyframes, dot color is **frozen** — observed_set has the same content for all ~30 frames. Then on the next keyframe, landmarks not re-observed transition orange → gray, fresh hits transition the other way. **This is the primary visible "flicker" cadence: a step transition at ~1 Hz, not random noise.**

### B.3 Root cause #2 — Per-frame proximity refresh has 9-15 % yield (Tracker.cpp:1702-1736)

Fix #11b's 5-px proximity refresh is meant to slide observed dots with the live KLT feature between keyframes (`Tracker.cpp:1702` `kRefreshRadiusPx = 5.0f`). Measured yield: `refresh/anchor` ratio is **0.094 – 0.154** across all walks — i.e. only 1 in 7–10 anchor-dot renders actually gets refreshed.

Why so low: at 30 Hz walking, KLT inter-frame motion is 2-5 px median but the tail exceeds 5 px easily during turns or fast translation. When the KLT feature drifts > 5 px from the previously stored landmark pixel, the refresh loop finds no nearest-neighbor and **the dot stays at the last keyframe's pixel** — visually "stuck", while the image content slides past it. To the user, the dot appears to drift away from its physical anchor between keyframes, then snap back at the next keyframe. **This is the second visible flicker mechanism: ~1 Hz position snap on stuck observed dots.**

The comment at `Tracker.cpp:1698-1701` claims the falsifier should be `r ≈ a`. **Actual measured `r/a` is ~10 %** — the fix is underperforming its own acceptance criterion. The 5-px radius is too tight for hand-held walking.

### B.4 Root cause #3 — "World-fixed" fallback dots drift with EKF pose (native-lib.cpp:1389-1437, CameraUi.kt:548-557)

For the ~40 % of dots where `observed=false` OR the pixel lookup raced (`native-lib.cpp:1428-1437`), the Kotlin `LandmarkOverlay` falls back to projecting `p_world` through the cached overlay pose (`CameraUi.kt:540-557`):

```
p_cam = R_world_cam.t() · (p_world - t_world)
u = fx · xc / zc + cx
v = fy · yc / zc + cy
```

`R_world_cam` and `t_world_cam` come from the OverlaySnapshot cached at `native-lib.cpp:90-133` (TTL 20 ms). **The pose IS coherent within one overlay tick** — the single-snapshot fix at `native-lib.cpp:1297` ensures both `getCurrentCameraPose` and `getLandmarkSnapshot` read the same state-version. But the cached snapshot's `p_G` evolves over time as the EKF updates, so a "world-fixed" dot drawn by projection slides in screen space at the EKF correction rate — which **is** the visual drift the user sees on the gray (~40 %) population. This is correct behavior (the dots represent fixed world points, the camera pose is what's moving), but combined with B.2 / B.3 it explains why a dot near the orange/gray transition boundary appears to "flicker on and off" — when it loses its observed flag it ALSO loses its KLT anchor, and projection drift snaps it to a noticeably different pixel.

### B.5 Root cause #4 — Per-snapshot `LandmarkOverlay` recomposition allocates a fresh JNI float array

`CameraUi.kt:464` `val landmarks = NativeBridge.getLandmarkSnapshot()` runs on every recomposition triggered by `viewModel.overlaySnapshot` updates (~30 Hz on the VIO executor → main thread). Each call walks up to 500 landmarks under `LandmarkMap`'s mutex, packs into a stride-7 float buffer, returns via JNI as a fresh `FloatArray`. The composable then computes the projection per dot.

Cost is bounded (500 dots × ~10 floats of math), but the relevant issue for flicker: when the `getLastObservedLandmarkIds()` mutex read at `native-lib.cpp:1306` happens to interleave with the producer's keyframe-write at Tracker.cpp:5076, the reader can get either the OLD or the NEW ids vector. There is no torn-read (atomic vector replace under the lock), but two consecutive overlay calls 33 ms apart may show different orange/gray populations. This is a small per-keyframe race window (~1 ms of producer hold time) but it manifests as a brief 1-frame color flip on the keyframe boundary.

### B.6 Render-cadence vs capture-cadence

- Capture: CameraX analyzer → `SensorRepository.processCameraFrame` → `vioExecutor.execute { processCameraFrameDirect }` (`SensorRepository.kt:929-957`). VIO frame rate is logged every 30 frames at `SensorRepository.kt:947` `"VIO_FPS: jni=Xms total=Yms"`.
- Overlay publish: `SensorRepository.kt:950` `publishOverlaySnapshot(vio)` runs IMMEDIATELY after `processCameraFrameDirect` returns, same thread, same frame. So `OverlaySnapshot` is push-per-frame at native rate (~30 Hz, modulo VIO duration which is currently ~80-100 ms causing under-30 effective fps).
- Render: Compose recompose triggered by `viewModel.overlaySnapshot = snap` (`NavSightViewModel.kt:262`). Compose batches at 60 Hz (or the display refresh rate); since the producer pushes ~10-12 Hz effective (VIO duration bound), every push produces one recompose.
- Display: SLAM dots, Landmark dots, KLT dots all gated on the SAME `overlaySnapshot` push so they recompose in lock-step. No phase mismatch BETWEEN overlays.

**Thread-coherency**: the overlay snapshot itself is single-frame coherent (the cached `g_overlay_snapshot` at `native-lib.cpp:79-82` + TTL guarantees `getCurrentCameraPose` and `getSlamSnapshot` read the SAME `OverlaySnapshot`). The OUTLIER is `getLastObservedLandmarkIds()` which reads its own mutex independently of the overlay snapshot cache — see B.5.

### B.7 Fallback projection uses CURRENT EKF pose, not stale snapshot pose

Verified at `native-lib.cpp:1181` and `native-lib.cpp:1297`: `getCurrentCameraPose` and `getLandmarkSnapshot` both call `ensureOverlaySnapshot` under `state_mutex`. The snapshot's `p_G` and `R_GtoI` and `R_bc` come from `EKFState::snapshotForOverlay()` at `EKFState.cpp:3787` — that snapshot is **CURRENT** at the moment of the JNI call (or up to 20 ms stale via the TTL cache). The fallback projection at `CameraUi.kt:548-557` uses the snapshot pose, which is bounded-fresh; not stale across overlay ticks.

---

## C. SUMMARY OF USER-VISIBLE BUGS

| # | Mechanism | File:line | Severity | Action proposal |
|---|---|---|---|---|
| F1 | Observed→gray transitions at ~1 Hz keyframe rate are perceived as flicker | Tracker.cpp:5076-5085, native-lib.cpp:1411 | HIGH (primary user complaint) | Add hysteresis: keep `is_observed=true` for N frames (e.g. 30) after last keyframe match, decay alpha gradually instead of step transition |
| F2 | 5-px proximity refresh underperforms; observed dots stuck between keyframes | Tracker.cpp:1702 | HIGH | Either widen radius adaptively to mean_flow (e.g. `max(5, 2*mean_flow)`) or use the recorded `accepted_feature_ids_match` link directly when present |
| F3 | `nearby_ids.empty()` clears ids but not pixels (size invariant breach, latent bug) | Tracker.cpp:4717-4720 | MEDIUM (latent) | Clear pixels alongside ids; or assert size equality in `getLastObservedLandmarkPixel` |
| F4 | Per-keyframe race window in `getLastObservedLandmarkIds` read interleaving | native-lib.cpp:1306, Tracker.cpp:5075 | LOW | Move the `observed_ids` snapshot inside `ensureOverlaySnapshot` so the entire overlay tick reads a single coherent snapshot |
| H1 | Debug-panel heading drift is from BUG_02 (Madgwick bias), NOT render | (no render bug) | INFO | No fix in render scope; defer to Worker #2 |

---

## D. DELIVERABLES FOR OTHER WORKERS

- Worker #1 (descriptor matching, BUG_01): F1 and F2 will visibly improve dot stability EVEN BEFORE descriptor-match yield improves, because anchor-hit ratio at 55-61 % is not the bottleneck — the bottleneck is the cadence at which the orange→gray flips happen. Recommend both fixes ship together.
- Worker #2 (BUG_02 Madgwick bias): the user's "heading display 172/203/235" report is YOUR symptom, NOT a render artifact. Verified faithful chain on the Kotlin side. Counter-evidence: walk-1 sample idx 0 IS 172.33° (start of walk), so the "172" is just the user's initial orientation read at the start of the walk.
- Worker #3 (MiDaS/SLAM sparsity, BUG_03): SLAM dots and Landmark dots share the same overlay-snapshot timing, so any cadence improvements there will manifest in the rendered dot population.

---

## E. EVIDENCE INDEX

- Counter arithmetic: derived from `event_summary` of all 5 walks in `tests/sims/regression/visual/`.
- Heading wrap math: validated against `heading_walk_1_2026_05_20.json` `points[].hdg` (radians) at every 200 ms cadence.
- Code citations: every file:line referenced above is verified against the repository at HEAD on branch `morad` as of 2026-05-21.
- No source code changes made.
