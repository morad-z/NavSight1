# 06 — Android UI Layer (Activities + ViewModel + Compose Screens)

## 1. App lifecycle

### 1.1 AndroidManifest.xml — `app/src/main/AndroidManifest.xml`

Permissions (lines 5–11): `INTERNET`, `CAMERA`, `ACCESS_FINE_LOCATION`, `VIBRATE`, `WAKE_LOCK`, `RECORD_AUDIO`, `HIGH_SAMPLING_RATE_SENSORS`.

Application (13–38):
- `supportsRtl="true"` (20) — important: warning labels are Hebrew (e.g. `StatusBadgesUi.kt:27, 57, 76, 93, 106`).
- Theme `@style/Theme.NavSight1` (21) is **overridden** at runtime by `MaterialTheme(colorScheme = darkColorScheme())` (`MainActivity.kt:28`).
- `meta-data com.google.android.geo.API_KEY` = `${GOOGLE_MAPS_API_KEY}` (23–25), same value read in code via `BuildConfig.GOOGLE_MAPS_API_KEY` (`NavSightViewModel.kt:51`, `SearchBarUi.kt:58`).
- Single launcher `MainActivity` (27–37); `exported="true"` because of `MAIN`/`LAUNCHER`.

### 1.2 `MainActivity.onCreate` flow — `MainActivity.kt`

1. `CrashLogger.install(applicationContext)` (line 25) — JSON crash sink installed before any UI work.
2. `enableEdgeToEdge()` (26).
3. `window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)` (27).
4. `setContent { MaterialTheme(colorScheme = darkColorScheme()) { NavSightApp() } }` (28).

Lifecycle (31–32):
- `onResume()` → `viewModel.onResume()` → `sensorRepositoryActive=true` then `sensorRepository.startSensors()` (`NavSightViewModel.kt:396–399`).
- `onPause()` → `viewModel.onPause()` → flag false then `stopSensors()` (401–405). Flag flipped **before** sensors so in-flight CameraX frames drop at the boundary (comments at 397, 402).

### 1.3 Splash → Permissions → Calibration → Main

`NavSightApp()` (`MainActivity.kt:36–52`) is the entire navigation graph:
- `showSplash: Boolean` initial `true` (37). `SplashScreen(pal) { showSplash = false }` (42).
- `isNight: Boolean` initial `isNightTime()` (38). `LaunchedEffect(Unit) { while (true) { delay(60_000L); isNight = isNightTime() } }` (39). Manually toggled from the bottom sheet (51, 265).
- `pal = remember(isNight) { buildNavPalette(isNight) }` (40).
- Permissions (44–46): `rememberMultiplePermissionsState(listOf(CAMERA, ACCESS_FINE_LOCATION))`.
- `LaunchedEffect(perms.allPermissionsGranted) { if (granted) viewModel.requestInitialLocation(true) }` (47–49).
- If not granted → `PermissionScreen(pal) { perms.launchMultiplePermissionRequest() }` (50). Else → `MainScreen(viewModel, pal, isNight) { isNight = !isNight }` (51).

End-to-end:

```
onCreate → CrashLogger.install → KEEP_SCREEN_ON → setContent → NavSightApp
  ├─ SplashScreen(pal)                                     // ~3.2 s, animated
  ├─ PermissionScreen(pal)                                 // until both granted
  └─ MainScreen
       ├─ CalibrationScreen overlay (calibrationVisible)
       ├─ AlertDialog "Hold steady"                        // when initStatus == TIMEOUT_NEEDS_USER  ← f1684e4 user-confirmed bypass
       └─ CameraOverlay full-screen (cameraVisible)
```

---

## 2. NavSightViewModel — every state holder, writer, reader

`app/src/main/java/com/example/navsight1/NavSightViewModel.kt`

### 2.1 Public observable state (Compose `mutableStateOf`, `private set`)

There are **no** exposed StateFlow/LiveData.

| Property | Type | Init | Line | Writer(s) | Reader(s) |
|---|---|---|---|---|---|
| `orientationState` | `DeviceOrientationTracker.OrientationResult` | sentinel zeros | 67–71 | init coroutine, `sensorRepository.orientationState.sample(200L)` (200–202) | `MapScreenUi.kt:38, 124–127` |
| `vioState` | `VioData` | `VioData()` | 73–74 | `handleVioUpdate:233` (throttled) | `MapScreenUi.kt:39, 79–82, 125`; `DebugPanelUi.kt:36, 107` |
| `virtualX` | `Double` | `0.0` | 76–77 | `handleVioUpdate:235`, `resetPath:426` | `MapScreenUi.kt:221, 315` |
| `virtualZ` | `Double` | `0.0` | 78–79 | `handleVioUpdate:236`, `resetPath:427` | same |
| `pathHistoryVersion` | `Int` | `0` | 83–84 | `handleVioUpdate:249`, `resetPath:425` | `MapScreenUi.kt:44` (snapshot key) |
| `pathHistory` (mutable list backing) | `List<PathPoint>` | `ArrayList(512)` | 81, 85 | `handleVioUpdate:247–248`, `resetPath:424`, `exportPath:502` | via `historySnapshot` |
| `positionSigmaM` | `Float` | `Float.NaN` | 90–91 | `handleVioUpdate:246` | `MapScreenUi.kt:166, 387, 392`; radar 466 |
| `positionCovValid` | `Boolean` | `false` | 93–94 | `handleVioUpdate:245` | `MapScreenUi.kt:165, 388` |
| `startLocation` | `LatLng?` | `null` | 98–99 | init coroutine 207 | `MainScreen.kt:47, 187`; `NavigationMapWrapper:309, 404` |
| `navigationState` | `NavigationState` | `Idle` | 101–102 | init coroutine 219 | `MainScreen.kt:45, 81, 179, 186`; `BottomSheet.navState` |
| `currentInstruction` | `NavigationInstruction?` | `null` | 104–105 | init coroutine 222 | `MainScreen.kt:46, 179–185` |
| `snappedPosition` | `LatLng?` | `null` | 107–108 | `handleVioUpdate:326` | `MapScreenUi.kt:220, 314` |
| `currentSpeedKmh` | `Float` | `0f` | 110–111 | `handleVioUpdate:307`, `resetPath:428` | hero header, speed badge, debug panel, sheet |
| `totalDistanceM` | `Double` | `0.0` | 113–114 | `handleVioUpdate:296`, `resetPath:429` | `MainScreen.kt:81, 134, 250`; `DebugPanel:23` |
| `showCameraBlocked` | `Boolean` | `false` | 116–117 | init coroutine 210 | `MapScreenUi.kt:122` (CameraOverlay) |
| `initStatus` | `SensorRepository.InitStatus` | `WAIT_STATIONARY` | 118–119 | init coroutine 213 | `MapScreenUi.kt:289` (AlertDialog gate) |
| `navigationStartMessage` | `String?` | `null` | 120–121 | `startNavigation:439`, `clearNavigationStartMessage:446` | `MapScreenUi.kt:189–191` |
| `scaleCalibrationFactor` | `Double` | prefs default `1.0` | 122–125 | `saveScaleCalibrationFactor:526` | `DebugPanelUi.kt:35, 97` |
| `scaleCalibrationMessage` | `String?` | `null` | 126–127 | start/finish/cancel/reset paths | `DebugPanelUi.kt:34, 145` |
| `scaleCalibrationSession` | `ScaleCalibrationSession?` | `null` | 128–129 | `startScaleCalibration:454`, mutated `handleVioUpdate:263–270`, cleared 479, 496 | `DebugPanelUi.kt:33, 107–115` |
| `userHeight` | `Float` | prefs default `1.70f` | 130–131 | `updateUserHeight:191` | `DebugPanelUi.kt:80, 83, 86` |
| `calibrationLoaded` | `Boolean` | `calibrationExists(application)` | 138–139 | `refreshCalibrationLoaded:143` (called from `MapScreenUi.kt:282`, `CalibrationScreenUi.kt:202`) | `MapScreenUi.kt:169, 174` |
| `isRecordingSimulation` | `Boolean` | `false` | 147–148 | `toggleSimulationRecording:345, 347` | `DebugPanelUi.kt:55, 59, 61` |

Computed read-only:
- `vioInitAzimuth: Float` get from `sensorRepository` (166).
- `pathHistory` exposes the underlying mutable ArrayList; readers must use `pathHistoryVersion` keying.

Companion constants: `PREFS_NAME="navsight_prefs"` (45), `PREF_SCALE_CALIBRATION_FACTOR="scale_calibration_factor"` (46), `PREF_USER_HEIGHT="user_height_m"` (47).

### 2.2 Side-effect coroutines (init block, lines 196–224)

Eight `viewModelScope.launch { ... .collect { ... } }` adapters that map upstream flows to Compose state:
1. `orientationState.sample(200L) → orientationState =` (200–202)
2. `vioState → handleVioUpdate(vio)` (203–205)
3. `startLocation → startLocation =` (206–208)
4. `showCameraBlocked → showCameraBlocked =` (209–211)
5. `initStatus → initStatus =` (212–214)
6. `currentLocation → currentGpsLocation =` (215–217)
7. `navigationManager.navigationState → navigationState =` (218–220)
8. `navigationManager.currentInstruction → currentInstruction =` (221–223)

Init also pushes prefs into native (197–198): `NativeBridge.setScale(scaleCalibrationFactor)` and `NativeBridge.setUserHeight(userHeight)`.

### 2.3 `handleVioUpdate(vio)` walk (lines 226–334)

- `latestVioState = vio` always (227).
- UI throttle gate `UI_UPDATE_THROTTLE_MS = 200L` (173, 229). Inside:
  - `vioState = vio` (233). If initialized: copy `virtualX/Z`, pull horizontal-plane covariance via `NativeBridge.getPositionCovariance(covBuf)` and `sigma = sqrt(σ_xx + σ_zz)` (239–246). Append `PathPoint(x, z, sigma)` (247–248). Cap `_pathHistory` at 500 (248). Bump `pathHistoryVersion` (249). Push `CrashLogger.updateSnapshot(buildCrashSnapshotJson(...))` (252; impl 536–559).
- Always (off-throttle), if VIO initialized: scale-calib session metrics integration (257–271); simulation `SimulationPoint` append (273–290); `totalDistanceM` integration (292–298); `currentSpeedKmh` only when `dtMs >= 200` (300–314); every 500 ms, `Dispatchers.IO`, road-snap and `withContext(Main) { snappedPosition = ...; navigationManager.updateVioPosition(...) }` (316–332).

### 2.4 Public functions

`isSensorRepositoryActive()` (63), `refreshCalibrationLoaded()` (142), `updateUserHeight(Float)` (189) — coerce 1.0–2.5 + persist + native, `toggleSimulationRecording(...)` (336) — resets native EventCounters, GPS on/off, JSON write, `onResume()` (396), `onPause()` (401), `processCameraFrame(ImageProxy)` (407), `updateRollingShutterSkew(Long)` (413) — Step 8c relay, `requestInitialLocation(Boolean)` (417), `resetPath()` (422), `startNavigation(LatLng)` (434), `clearNavigationStartMessage()` (446), `clearInitTimeout()` (449) — **user-confirmed bypass entry point**, `startScaleCalibration(Double)` (451), `finishScaleCalibration()` (466), `cancelScaleCalibration()` (496), `clearScaleCalibrationMessage()` (497), `resetScaleCalibration()` (498), `stopNavigation()` (499), `exportPath(...)` (501), `onCleared()` (523), private `saveScaleCalibrationFactor(Double)` (525), private `buildCrashSnapshotJson(VioData,Float,Boolean)` (536).

DTOs declared here: `PathPoint(x,z,sigmaM)` (28), `ScaleCalibrationSession(...)` (31), `SimulationPoint(...)` (152).

---

## 3. Navigation graph

There is **no `NavController`, no sealed `Route` hierarchy, no `NavHost`**. The graph is the if/else in `MainActivity.NavSightApp` (`MainActivity.kt:36–52`). Inside `MainScreen`, overlays are toggled by booleans:

| Boolean | File:Line | Behavior |
|---|---|---|
| `cameraVisible` | `MapScreenUi.kt:50` | Crossfade map ↔ `CameraOverlay`; preview always alive (1.dp invisible) |
| `calibrationVisible` | `MapScreenUi.kt:55` | `AnimatedVisibility` swap to `CalibrationScreen` |
| `debugVisible` | `MapScreenUi.kt:54` | Slide-in `DebugPanel` |
| `bottomSheetExpanded` | `MapScreenUi.kt:56` (`rememberSaveable`) | Expands sheet |
| `isRecordingGpx`/`gpxPoints`/`gpxMessage` | 51–53 | GPX state |

Implications: system Back exits the activity even when overlays are open — no `BackHandler` is wired anywhere in the inspected files. Only `bottomSheetExpanded` survives process death.

---

## 4. MapScreenUi — composables

`app/src/main/java/com/example/navsight1/MapScreenUi.kt`

### 4.1 `MainScreen(viewModel, pal, isNight, onToggleNight)` (37)

State derived:
- `fusedHeading` (40–42): VIO `heading` (radians) wrapped to `[0,360)` if initialized, else `orientation.azimuth`.
- `historySnapshot = remember(viewModel.pathHistoryVersion) { viewModel.pathHistory.toList() }` (44).
- `isMoving = vio.isInitialized && vio.meanFlow > 1.0` (48).
- `compassLabel` 8-way bins (61–70).
- `fusionMode`: `INIT` / `IMU<0.3` / `CAMERA>0.7` / else `HYBRID` (71–76).
- `qualityLevel` 0/1/2 (77).
- `lowerOverlayPadding`: `260.dp` if expanded else `88.dp` (59).

Layout layers (each top-level box stacked, line numbers):
1. Camera background (90–96), wrapped `key(calibrationVisible)` so analyzer rebinds after `CalibrationScreenUi.kt:328–335` `provider.unbindAll()`.
2. Map area (99–115). If `mapStart == null`, "Acquiring location…" spinner. Else `NavigationMapWrapper`.
3. Camera full-screen overlay (118–141).
4. Map overlays (144–239) — only when `!cameraVisible`:
   - 82.dp purple gradient (145–151).
   - Status column (153–193): `HeroHeader` (157), row of `VioStatusChip` (162) + `CalibrationStatusPill` (168), conditional `CalibrationFirstLaunchBanner` (174–177), `NavigationInstructionBanner` if active (179–185), `SearchBarCard` if idle (186–188), `ErrorCard` for `navigationStartMessage` (189–192).
   - Right column (195–210): `SensorRadarWaze` only when idle, `MapActionStack`.
   - `SpeedLimitBadge` bottom-left (212–216).
   - GPX recording pill + auto-toast (218–230).
   - `DebugPanel` `AnimatedVisibility` (232–238).
5. `BottomSheet` always (241–267).
6. `CalibrationScreen` overlay `AnimatedVisibility(calibrationVisible)` (270–286). `onClose` flips visible off and calls `viewModel.refreshCalibrationLoaded()`.
7. **`AlertDialog "Hold steady"`** (289–303): visible iff `initStatus == TIMEOUT_NEEDS_USER`. `onDismissRequest = {}` (modal). Confirm calls `viewModel.clearInitTimeout()` — this is the surface for commit `f1684e4`'s **user-confirmed bypass**.

### 4.2 `NavigationMapWrapper(start, azimuth, history, pal, viewModel, modifier)` (308)

Map provider: **Google Maps via `com.google.maps.android:maps-compose`** (`com.google.maps.android.compose.GoogleMap`, line 369).

Camera state (320–322): initial `CameraPosition` based on `displayPos = snappedPosition ?: metersToLatLng(start, virtualX, virtualZ)`. Zoom/tilt: active=`19f/60f`, idle=`18f/30f` (316–317).

Follow loop:
- `isFollowing` flips false on user gesture (329–333).
- Animation loop `LaunchedEffect(Unit)` (334–352): when following, animate full pose every 350 ms then `delay(400L)`; otherwise rotate-only when bearing delta > 1.5° in 180 ms then `delay(160L)`.

Render throttle (354–366): map snapshot interval 800 ms when active, 1200 ms otherwise.

`GoogleMap` content (369–405):
- `MapProperties(mapType=NORMAL, isMyLocationEnabled=false)` (372) — blue dot off, VIO drives the marker.
- `MapUiSettings(zoomControls=false, compass=false, myLocationButton=false, zoom/scroll/tilt/rotation=true)` (373–375).
- Active route polyline (teal, width 14, zIndex 10) + destination marker (379–382).
- Walked-path polyline (orange 0.8, width 7, zIndex 5) (383).
- **Heading marker** (line 403): `Marker(state = MarkerState(mapPos), rotation = mapAzi, flat = true, anchor = Offset(0.5f, 0.5f), icon = arrowIcon)` — `arrowIcon` = `NavSightUtils.vectorToBitmap(context, R.drawable.navigation_arrow)` (377, remember-cached).
- Start anchor: `BitmapDescriptorFactory.defaultMarker(HUE_AZURE)` (404).
- **1σ uncertainty ring** (Step 6/Task #29) (387–402): drawn when `positionCovValid && sigma.isFinite() && sigma > 0`. Bins: `<0.5 Teal500`, `<1.5 Orange400`, else red `0xFFEF5350`. Radius `sigma.coerceIn(0.5f, 25f).toDouble()`. Stroke 4f at 0.85 alpha; fill 0.12 alpha.
- Recenter FAB (406–413), only when `!isFollowing`.

### 4.3 `SensorRadarWaze(history, currentAzimuth, pal)` (418)

118.dp Waze-style mini-radar:
- Range circles 1m/2m/5m, compass labels N/S/E/W.
- Trail rotated by `-currentAzimuth`; **Step 6/Task #30 sigma coloring** (460–476): per-segment color binned (NaN treated as red).
- Heading arrow with two 145°-offset wings (484–489).
- Distance label `"%.1f m"`, "RADAR" topline.

### 4.4 Other composables in this file

| Composable | Line | Notes |
|---|---|---|
| `HeroHeader(speedKmh, compassLabel, fusionMode, qualityPct, pal)` | 500 | Purple bar with quality %, speed, compass + mode |
| `HeroMiniBadge(icon, text, bg)` | 525 | small pill |
| `VioStatusChip(isInitialized, covValid, sigmaM, pal)` | 543 | 4-state EKF chip — see table below |
| `SpeedLimitCore(speedKmh)` | 577 | 42.dp circle, `max(30, ceil(speedKmh/10)*10)` |
| `SpeedLimitBadge(modifier, speedKmh, pal)` | 587 | bottom-left wrapped variant |
| `MapActionStack(pal, onCameraClick, onDebugClick, onExpandSheet, onCalibrateClick)` | 599 | 4 round FABs (camera/layers/calibrate/debug) |
| `CalibrationStatusPill(loaded, pal, onClick)` | 620 | Green "Calibrated" or grey "Tap to calibrate" |
| `CalibrationFirstLaunchBanner(pal, onClick)` | 648 | Orange warning banner when `!calibrationLoaded` |
| `FloatingMapButton(icon, onClick, containerColor, contentColor)` | 674 | 50.dp circular FAB |

`VioStatusChip` mapping (550–555):

| Predicate | Label | Color |
|---|---|---|
| `!isInitialized` | "VIO LOST — WALK FORWARD TO RE-ACQUIRE" | `0xFFEF5350` red |
| `!covValid \|\| sigmaM.isNaN()` | "VIO INITIALIZING" | `0xFF9E9E9E` grey |
| `sigmaM < 1.5f` | "GPS-DENIED — VIO ACTIVE (σ = X.X m)" | `Teal500` |
| else | "VIO DEGRADED (σ = X.X m)" | `Orange400` |

---

## 5. CameraUi

`app/src/main/java/com/example/navsight1/CameraUi.kt`

### 5.1 `CameraViewComposable(viewModel)` (42)

- `AndroidView` factory builds `PreviewView` with `implementationMode = PERFORMANCE`, `scaleType = FILL_CENTER` (46–49).
- `ResolutionSelector` strict 4:3 + 640×480 with `FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER` (59–67) — must match calibration screen.
- `ImageAnalysis.Builder` with `STRATEGY_KEEP_ONLY_LATEST`, `OUTPUT_IMAGE_FORMAT_YUV_420_888` (72–75).
- **Step 8c** Camera2Interop attaches a `CameraCaptureSession.CaptureCallback` whose `onCaptureCompleted` reads `CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW` (ns) and forwards to `viewModel.updateRollingShutterSkew(skew)` (78–95).
- Analyzer registration: `Executors.newSingleThreadExecutor()` (98). The body does only `if (viewModel.isSensorRepositoryActive()) viewModel.processCameraFrame(img) else img.close()` and explicitly catches `RejectedExecutionException`, `IllegalStateException`, generic `Exception` (98–110).
- Bind: `provider.unbindAll(); provider.bindToLifecycle(lifecycleOwner, DEFAULT_BACK_CAMERA, preview, analysis)` (113–114).

### 5.2 `CameraOverlay(...)` (124, 16 parameters)

- Vertical gradient (147–150).
- Top-left `DirectionBadge(isMoving, vioTrackingQuality, pal)` (152–154).
- Top-right column (156–165): close button + `SensorRadarWaze`.
- Center conditional warnings: `VioInitializingBadge` if `!isVioInitialized` (167); `CameraBlockedWarning` if `showCameraBlocked` (169); `NoTextureWarning` if `vioTrackedFeatures < 30` (171); `PhoneOrientationWarning(orientationDeviation)` if `!orientationIsHorizontal` (173); `StabilityIndicator(stabilityScore, vioTrackingQuality)` (175).
- Bottom HUD strip (179–211): white surface with 5 columns separated by `CamHudDiv`: `m/s`, mode, quality %, distance, debug toggle.
- `AnimatedVisibility(debugVisible)` shows `DebugPanel` (213–220).

### 5.3 `PipCameraCard(modifier, pal, vioInitialized, onClick)` (225)

Defined but **not currently called** from any inspected file.

### 5.4 Private helpers

`CamHudStat(value, label, color)` (254), `CamHudDiv()` (262).

---

## 6. CalibrationScreenUi — VIO **camera intrinsic** calibration (Visual Plan Step 1)

`app/src/main/java/com/example/navsight1/CalibrationScreenUi.kt`

> This file is the **camera intrinsic calibration wizard** (chessboard + OpenCV `calibrateCamera`), not IMU stationary-init. The IMU "place phone flat 5 s" flow is the `AlertDialog` at `MapScreenUi.kt:289–303`, gated on `initStatus == TIMEOUT_NEEDS_USER`. The user-confirmed bypass (commit `f1684e4`) flips that timeout via `viewModel.clearInitTimeout()` (line 300, defined `NavSightViewModel.kt:449`).

### 6.1 Constants (69–80)

| Const | Value |
|---|---|
| `CHECKER_COLS` | 9 |
| `CHECKER_ROWS` | 6 |
| `MIN_CAPTURES` | 30 |
| `SHARPNESS_THRESHOLD` | 80.0 (variance-of-Laplacian) |
| `MIN_ANGULAR_DELTA_DEG` | 8.0 |
| `MIN_TRANS_DELTA_M` | 0.05 |
| `MIN_INTERVAL_MS` | 400L |
| `DETECT_INTERVAL_MS` | 100L |
| `GRID_DIM` | 3 |
| `GRID_CELL_NEEDED` | 2 |
| `CALIB_FILE` | `"camera_calib.json"` |

### 6.2 Step machine (82): `enum class CalibrationStep { Onboarding, Capturing, Solving, Verdict }`

### 6.3 `CalibrationScreen(viewModel, pal, onClose)` (136)

Holds `captures`, `coverage`, `imageSize`, `lastResult`, `solveError`, `openCvOk = ensureOpenCV()`. Branches by `step`:
- Onboarding (155–162) — info, square-mm input, Start button.
- Capturing (163–188) — live capture; `onSolve` kicks `Dispatchers.Default` solver and routes to Verdict.
- Solving (189) — spinner + "calibrateCamera with rational distortion model".
- Verdict (190–221) — colored verdict; Save only if `rms <= 1.0`. On success: `viewModel.refreshCalibrationLoaded(); onClose()` (202–203).

### 6.4 `CalibrationCapturing` (297) flow

1. CameraX preview with **identical** ResolutionSelector to runtime (4:3, 640×480) — critical for intrinsics 1:1 match (396–404).
2. ImageAnalysis runs corner detection ≤10 Hz (`DETECT_INTERVAL_MS`).
3. `processFrame` (993–1117) returns a `FrameOutcome`. Results posted to main via `mainHandler.post`.
4. Auto-capture if `sharpOk && timeOk && poseFar && pnpOk` or manual (line 569).
5. **Auto-solve trigger** (347–354): `LaunchedEffect(canSolveAuto) { if (canSolveAuto) onSolve() }` where `canSolveAuto = captures.size >= MIN_CAPTURES && coverage.all { it >= GRID_CELL_NEEDED }`.

UI:
- `CalibrationOverlay` (500–508): 3×3 coverage grid + corner dots + arrow to target cell + tap-to-focus indicator.
- Top status row (512–522): back button + `CaptureProgressChip(count, MIN_CAPTURES)`.
- Center directional prompt (527–548) using `directionPrompt(currentCell, targetCell)` (966–985) — emoji arrows.
- Bottom slim controls (555–596): Capture (only if `cornersDetected`), Redo, status line.

Tap-to-focus (358–377): `LaunchedEffect(focusTapPx)` builds a `FocusMeteringAction` and calls `cam.cameraControl.startFocusAndMetering(action)`. Auto-clear after 1 s.

`DisposableEffect` releases camera + OpenCV resources on dispose (328–340) — and this is exactly **why `MainScreen` re-keys the regular `CameraViewComposable` on `calibrationVisible`** (`MapScreenUi.kt:90`).

### 6.5 IMU-stationary feedback surfaces (live elsewhere)

- `DirectionBadge` (`StatusBadgesUi.kt:23`): LOW QUALITY / "בתנועה" / "עומד".
- `VioInitializingBadge` (`StatusBadgesUi.kt:65`): pulsing orange "מכייל…" ("calibrating…").
- `PhoneOrientationWarning` (`StatusBadgesUi.kt:44`): pulsing red border + Hebrew "הטה את הטלפון קדימה" + "סטייה: X°".
- `StabilityIndicator` (`StatusBadgesUi.kt:113`): bar binned at 0.7/0.4 + confidence %.
- `MapScreenUi.kt:289–303` `AlertDialog "Hold steady"` — modal until user taps OK; OK calls `viewModel.clearInitTimeout()` ⇒ the **f1684e4 user-confirmed bypass**.

---

## 7. SearchBar / NavInstruction / StatusBadges / DebugPanel / BottomSheet

### 7.1 SearchBarUi.kt
- DTOs: `PlacePrediction(placeId, primaryText, secondaryText)` (32); `RoutePreview(destination, destinationName, polyline)` (34).
- `SearchBarCard(pal, startLocation, placesClient, onDestinationSelected)` (41) → wraps `WazeSearchBar` (46).
- Internal state: `searchText` (52), `predictions` (53), `isSearching` (54), `routePreview` (55), `sessionToken` (56), `apiKey = BuildConfig.GOOGLE_MAPS_API_KEY` (58).
- Predictions trigger after `q.length >= 2` (72) via `fetchPlacePredictions`.
- On prediction tap → `fetchPlaceLatLng` → `fetchDirectionsRoute(origin, ll, apiKey)` → populate route preview (100–119).
- `Start` button emits `onDestinationSelected(preview.destination)` and clears state (171–180).
- `fetchPlacePredictions(query, sessionToken, placesClient, onResult)` (187), `fetchPlaceLatLng(placeId, placesClient, onResult)` (204), `suspend fetchDirectionsRoute(origin, dest, apiKey)` (213) — direct HTTPS GET against `maps.googleapis.com/maps/api/directions/json` with `mode=walking`, 8000 ms timeouts (221).
- Helpers: `decodeDirectionsJson(json)` (231), `decodePolyline(encoded)` (238).

### 7.2 NavInstructionBannerUi.kt
- `NavigationInstructionBanner(instruction, remainingDistanceMeters, remainingTimeSeconds, pal, modifier)` (20). Renders maneuver icon (`instruction.getManeuverIcon()`), street name (or "Continue"), `"in <formatDistance(distanceMeters)>"`, ETA `formatTime(remainingTimeSeconds)` and total remaining distance.
- Per the SDD this is part of the **out-of-scope** turn-by-turn extension.

### 7.3 StatusBadgesUi.kt — 10 composables

| Composable | Line | Visual |
|---|---|---|
| `DirectionBadge(isMoving, trackingQuality, pal)` | 23 | LOW QUALITY (red) / "בתנועה" (teal) / "עומד" (orange) |
| `PhoneOrientationWarning(deviation, pal)` | 44 | Pulsing red border, icon rotated by `deviation.coerceIn(-30, 30)` |
| `VioInitializingBadge(pal)` | 65 | Pulsing orange "מכייל…" |
| `CameraBlockedWarning(pal)` | 82 | Pulsing red "מצלמה חסומה" |
| `NoTextureWarning(pal)` | 99 | Orange "אין מרקם — עבור לאזור מרוצף" |
| `StabilityIndicator(stability, confidence, pal)` | 113 | 36×4 dp bar; thresholds 0.7/0.4 |
| `RecordingPill(pal)` | 129 | Pulsing dot + "REC GPX" |
| `WazeToast(text, pal)` | 142 | Generic toast pill |
| `WazeChip(pal, content)` | 156 | Generic content chip |
| `ErrorCard(message, pal, onDismiss)` | 172 | Orange-icon row, dismiss on tap |

### 7.4 DebugPanelUi.kt
- `DebugPanel(totalDistanceM, speedMs, qualityPct, fusionMode, scaleFactor, headingDeg, pal, viewModel)` (22), 192.dp wide.
- Reads `viewModel.scaleCalibrationSession` (33), `scaleCalibrationMessage` (34), `scaleCalibrationFactor` (35), `vioState` (36), `isRecordingSimulation` (55, 59, 61), `userHeight` (80, 83, 86).
- Sim record button toggles `viewModel.toggleSimulationRecording` (49–63).
- Stats rows: Dist, Speed, Quality (color-binned at 70%/30%), Mode, Scale, Heading (66–73).
- Height stepper ±0.05 m, calls `viewModel.updateUserHeight` (76–90).
- Calibration accordion (93–146) using local `calibExpanded`:
  - In session: Path / Out / Back / Avg Q rows (110–114) + Finish + Cancel.
  - Idle: 5m / 10m / Reset buttons (119–132) → `startScaleCalibration(5.0/10.0)` / `resetScaleCalibration()`.
- `DebugRow(label, value, valueColor, pal)` (152).

### 7.5 BottomSheetUi.kt
- `BottomSheet(...)` (25) — 18 parameters.
- ETA banner (`AnimatedVisibility(navState is Active)`, 52–85): purple surface, `formatTime(remainingTimeSeconds)`, `formatDistance(remainingDistanceMeters)`, "End" button → `onStopNavClick`.
- Expandable panel (`AnimatedVisibility(expanded)`, 88–157):
  - `SheetHandleRow(expanded, speedKmh, compassLabel, onToggleExpanded)` (100, defined 178).
  - "Nearest incidents" header + `IncidentCard` per item from `incidents: List<IncidentCardModel>` (defined 271).
  - Stat-pill row: km/h, meters, compass heading, mode/state.
  - Tab row: Camera/Map toggle, Day/Night, Debug, End-Nav or Live/Waiting.
- Bottom nav bar (159–173): four `BottomNavItem`s — Map, Warnings, Rides (`onResetClick`), Records (`onGpxClick`).
- Helpers: `BottomNavItem` (204), private `RecordingDot` (223), `TabBtn` (230, 9 params), `IncidentCard` (271), `StatPill` (288).
- `buildIncidentCards(navState, isMoving, speedKmh, totalDistanceM, trackingQuality, vioInitialized): List<IncidentCardModel>` (295) — primary card adapts (Active vs On-the-move/Standing); Quality at 0.75/0.4 thresholds; "Debug tools" card.

---

## 8. Theme & typography

### 8.1 NavSightTheme.kt — the **active** theme
- `isNightTime()` (7): `LocalTime.now().hour < 6 || hour >= 20` (night between 20:00 and 06:00 local). Refreshed every 60 s in `MainActivity.kt:39`.
- Color tokens (10–30):
  - Teals: `Teal400=0xFF26C6DA`, `Teal500=0xFF00BCD4`, `Teal700=0xFF0097A7`, `Teal900=0xFF006064`.
  - Oranges: `Orange400=0xFFFF9800`, `Orange600=0xFFF57C00`, `OrangeAcc=0xFFFF6D00`.
  - Dark: `DarkBg=0xFF0E1621`, `DarkCard=0xFF162130`, `DarkBorder=0xFF1E3045`.
  - Text: `TextWhite=0xFFECF0F1`, `TextGrey=0xFF90A4AE`.
  - Light: `LightBg=0xFFF0F7FA`, `LightCard=0xFFFFFFFF`, `LightText=0xFF1A2B3C`.
  - Hero: `HeroPurple=0xFF5847B8`, `HeroPurpleDark=0xFF4B3DA7`.
  - Soft: `SoftSurface=0xFFF7F7FB`, `SoftBorder=0xFFE8E8F0`.
  - Whites: `White12=0x1FFFFFFF`, `White30=0x4DFFFFFF`.
- `data class NavPalette(bg, card, cardBorder, teal, tealDark, orange, orangeAcc, textPrimary, textSecondary, isNight)` (33).
- `buildNavPalette(night)` (49):
  - Night → `(DarkBg, DarkCard, DarkBorder, Teal500, Teal700, Orange400, OrangeAcc, TextWhite, TextGrey, true)`.
  - Day → `(LightBg, LightCard, 0xFFB2EBF2, Teal700, Teal900, Orange600, OrangeAcc, LightText, 0xFF546E7A, false)`.
- `data class IncidentCardModel(title, subtitle, eta, color, icon)` (41).

### 8.2 ui/theme/{Color,Theme,Type}.kt — declared but **dead at runtime**
- `NavSight1Theme(darkTheme, dynamicColor, content)` defines Material3 `lightColorScheme`/`darkColorScheme` based on Purple40/PurpleGrey40/Pink40 (and `_80` variants), supports Android 12+ `dynamicLight/DarkColorScheme`. **Never invoked**: `MainActivity.kt:28` calls `MaterialTheme(colorScheme = darkColorScheme())` directly.
- `Type.kt` defines a single `bodyLarge` style; not reached at runtime. Typography is **inline** in every composable (e.g. `fontSize = 11.sp`, `fontWeight = FontWeight.ExtraBold`); there is no use of `MaterialTheme.typography.*` outside the unused `Type.kt`.

### 8.3 Theming approach summary

| Layer | Role | Where |
|---|---|---|
| `MaterialTheme(darkColorScheme())` | Forces dark M3 scheme | `MainActivity.kt:28` |
| `NavPalette` + `buildNavPalette(isNight)` | App palette, threaded explicitly through every composable | `NavSightTheme.kt`, every screen |
| `isNightTime()` | Auto-night by clock; manual toggle from sheet | `NavSightTheme.kt:7`, `MainActivity.kt:39`, `MapScreenUi.kt:51` |
| `ui/theme/*` | Unused | n/a at runtime |

---

## 9. Public functions / composables — full signature index

- **MainActivity.kt** — `class MainActivity : ComponentActivity` (17), `onCreate(Bundle?)` (21), `onResume()` (31), `onPause()` (32), `@Composable fun NavSightApp()` (36).
- **NavSightViewModel.kt** — see §2 for every state + every public/private function with file:line. DTOs: `PathPoint`, `ScaleCalibrationSession`, `SimulationPoint`.
- **SplashScreen.kt** — `@Composable fun SplashScreen(onSplashFinished: () -> Unit)` (25). White background, Spring stiffness 40 (60), `tween(2500)` progress (79). **Not invoked at runtime.**
- **SplashScreenUi.kt** — `@Composable fun SplashScreen(pal: NavPalette, onDone: () -> Unit)` (32). Active splash. 3200 ms total (33). Three infinite transitions (orbit 3600, orbitReverse 5000, glow 1600). Progress `0→1` in 2800 ms (41).
- **PermissionScreenUi.kt** — `@Composable fun PermissionScreen(pal: NavPalette, onRequest: () -> Unit)` (22). Single card: 76.dp icon, "Camera & Location\nRequired", "NavSight needs sensors to track your path", button → `onRequest`.
- **CalibrationScreenUi.kt** — see §6.
- **MapScreenUi.kt** — see §4.
- **CameraUi.kt** — see §5.
- **BottomSheetUi.kt** — `BottomSheet(...)` (25), `SheetHandleRow(...)` (178), `BottomNavItem(...)` (204), private `RecordingDot()` (223), `TabBtn(...)` (230), `IncidentCard(incident)` (271), `StatPill(title, subtitle, accent)` (288), `buildIncidentCards(...)` (295).
- **SearchBarUi.kt** — see §7.1.
- **StatusBadgesUi.kt** — see §7.3.
- **DebugPanelUi.kt** — `DebugPanel(...)` (22), `DebugRow(...)` (152).
- **NavInstructionBannerUi.kt** — `NavigationInstructionBanner(...)` (20).
- **NavSightTheme.kt** — `isNightTime()` (7), all color `val`s (10–30), `data class NavPalette` (33), `data class IncidentCardModel` (41), `buildNavPalette(night)` (49).

---

## 10. State holder → reader/writer matrix

See §2.1 — every property has writer line(s) and reader file:line list. Highlights:

- `pathHistory` is a mutable ArrayList exposed as `List<PathPoint>`; consumers must use `pathHistoryVersion` as a recomposition key (`MapScreenUi.kt:44`) to avoid reading stale list contents.
- `snappedPosition` is the **displayed** position; if absent, `MapScreenUi.kt:220, 314` falls back to `metersToLatLng(start, virtualX, virtualZ)`.
- `initStatus` is the only writer that triggers an `AlertDialog` (the user-confirmed bypass).

---

## 11. Permissions runtime flow

- Manifest: `INTERNET`, `CAMERA`, `ACCESS_FINE_LOCATION`, `VIBRATE`, `WAKE_LOCK`, `RECORD_AUDIO`, `HIGH_SAMPLING_RATE_SENSORS`.
- Runtime requested via Accompanist: only **`CAMERA` + `ACCESS_FINE_LOCATION`** (`MainActivity.kt:44–46`).
- `RECORD_AUDIO` is declared but **not** runtime-requested in the inspected files.
- `HIGH_SAMPLING_RATE_SENSORS` is `signature|privileged|appop` style on Android 12+ and needs no runtime UX.
- Sequence:
  1. SplashScreen ~3.2 s.
  2. `rememberMultiplePermissionsState` evaluated; if any not granted, `PermissionScreen` shown.
  3. Tap "Enable Sensors" → `perms.launchMultiplePermissionRequest()` (`MainActivity.kt:50`).
  4. On all-granted, `LaunchedEffect(perms.allPermissionsGranted)` fires `viewModel.requestInitialLocation(true)` (47–49) → sets `hasLocationPermission = true` and asks `SensorRepository` to acquire the start fix.
  5. `MainScreen` is then composed.
- There is no per-permission rationale UI before the system dialog.

---

## 12. Magic numbers and UI tunables (curated)

### Lifecycle / animation
- `delay(60_000L)` night auto-refresh cadence (`MainActivity.kt:39`).
- Active splash 3200 ms (`SplashScreenUi.kt:33`); inner timings: orbit 3600 / orbitReverse 5000 / glow 1600 / progress 2800 / enter 700 ms (36–47).
- Alternate (unused) splash: `tween(1500)`, `Spring(stiffness=40f)`, `tween(2500)` (`SplashScreen.kt:42–84`).

### ViewModel
- `UI_UPDATE_THROTTLE_MS = 200L` (173); `sample(200L)` orientation downsample (201); `dtMs >= 200` speed compute floor (303); `nowMs - lastSnapTimeMs > 500` road-snap cadence (316); `_pathHistory` cap = 500 (248); `userHeight.coerceIn(1.0f, 2.5f)` (190); `pathHistory` initial capacity 512 (81).

### Map / camera follow
- Active route zoom 19f / tilt 60f; idle 18f / 30f (`MapScreenUi.kt:316–317`).
- Animate 350 ms + delay 400 ms (follow); 180 ms + delay 160 ms (rotate-only, threshold > 1.5°) (337–349).
- Map snapshot interval: 800 ms active, 1200 ms otherwise (361).
- Radar bearing rounded to 2°: `Math.round(fusedHeading / 2f) * 2f` (201).

### Status thresholds
- `fusionMode`: `<0.3 IMU`, `>0.7 CAMERA`, else `HYBRID` (`MapScreenUi.kt:73–76`).
- `qualityLevel`: `<0.3, <0.7, else` (77).
- `isMoving = vio.meanFlow > 1.0` (48).
- `VioStatusChip` sigma threshold `< 1.5f` ACTIVE vs DEGRADED (553–554).
- Uncertainty ring: `<0.5 teal`, `<1.5 orange`, else red; radius clamped to `[0.5, 25] m` (389–397).
- `RecordingDot` blink: `tween(500), Reverse` (`BottomSheetUi.kt:225`).
- `gpxMessage` toast auto-dismiss: `delay(3000)` (`MapScreenUi.kt:228`).
- `lowerOverlayPadding`: 260.dp expanded vs 88.dp collapsed (59).

### Debug / scale calibration thresholds (`finishScaleCalibration` 480–486)
- `< 10 samples` "Too short"; `maxDist < 0.7*leg` "Did not walk far enough"; `avgQuality < 0.35 || lowQualityRatio > 0.35` "Quality too low"; `pathLength < roundTripTarget * 0.6` "Round trip too short"; `closure > max(2.0, leg * 0.4)` "Return drift too large".

### Camera intrinsic calibration (`CalibrationScreenUi.kt:72–79`)
- `MIN_CAPTURES=30`, `SHARPNESS_THRESHOLD=80.0`, `MIN_ANGULAR_DELTA_DEG=8.0`, `MIN_TRANS_DELTA_M=0.05`, `MIN_INTERVAL_MS=400`, `DETECT_INTERVAL_MS=100`, `GRID_DIM=3`, `GRID_CELL_NEEDED=2`.
- Verdict tiers: `rms<0.5 green`, `<1.0 orange`, else red. Save enabled only when `rms <= 1.0` (825–828, 886–887, 1188).

### Camera HUD
- HUD quality color: `>70 teal`, `>30 orange`, else red (`CameraUi.kt:194–196`).
- "No texture" warning shows when `vioTrackedFeatures < 30` (171).
- `PhoneOrientationWarning` icon rotation clamped to `[-30, 30]°` (`StatusBadgesUi.kt:54`).
- `StabilityIndicator`: `>0.7 teal`, `>0.4 orange`, else red (119).

### Speed badge
- `SpeedLimitCore`: `max(30, ceil(speedKmh / 10) * 10).toInt()` (`MapScreenUi.kt:580`).

### Search
- Predictions trigger after `q.length >= 2` (`SearchBarUi.kt:72`).
- Directions HTTP timeouts: 8000 ms each (221).
- `&mode=walking` hardcoded (219).

### BottomSheet animations
- ETA banner: `slideInVertically + fadeIn(220)` / out (`BottomSheetUi.kt:54–55`).
- Expandable: in 90 ms, out 70 ms (90–91).

---

## Notes / loose ends

- **Two `SplashScreen` definitions exist.** `SplashScreen.kt:25` (`onSplashFinished`) is unused; `SplashScreenUi.kt:32` (`pal, onDone`) is the active one called by `MainActivity.NavSightApp`.
- **`PipCameraCard`** (`CameraUi.kt:225`) and **`CoverageStatusRow`** (`CalibrationScreenUi.kt:783`) are defined but unreferenced.
- **Theming dichotomy.** `ui/theme/{Color,Theme,Type}.kt` define a Material3 `NavSight1Theme` that is **not** used at runtime. The runtime path is `MaterialTheme(darkColorScheme())` plus the explicit `NavPalette` plumbed through every composable.
- **No `NavController`.** Routing is `if/else` plus `AnimatedVisibility` overlays. System Back is not intercepted.
- **Hebrew RTL strings** in `StatusBadgesUi.kt:27, 28, 57, 58, 76, 93, 106` (and surfaces using these badges). `supportsRtl="true"` is set in the manifest.
- **Turn-by-turn UI is present but the SDD scopes it as an extension.** `NavigationInstructionBanner`, the active-route polyline, and the destination marker are wired through `navigationManager` in the VM, but the GPS-denied VIO path is the proof-of-concept core.
- The **camera intrinsic calibration screen** (`CalibrationScreenUi.kt`) is the Visual-Plan Step-1 surface; the **IMU stationary calibration** is **not a screen** but the `AlertDialog` at `MapScreenUi.kt:289–303` and the `clearInitTimeout()` user-confirmed bypass added in commit `f1684e4`.
