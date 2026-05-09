# 07 — Kotlin Support Layer

> All paths/lines refer to `app/src/main/java/com/example/navsight1/`.

## 1. SensorRepository (`SensorRepository.kt`, 828 lines)

The single Kotlin owner of every Android sensor source feeding native VIO.

### 1.1 Sensor inventory (constructor `L46–L50`)

| Field | Sensor | Rate | Status |
|---|---|---|---|
| `accelerometer` (`L47`) | `TYPE_ACCELEROMETER` | `SENSOR_DELAY_GAME` (~50 Hz), registered `L136` | Always on |
| `magnetometer` (`L48`) | `TYPE_MAGNETIC_FIELD` | `SENSOR_DELAY_GAME`, registered `L141` | **Unregistered at `L743` once VIO initializes** (per no-mag-during-tracking rule) |
| `gyroscope` (`L49`) | `TYPE_GYROSCOPE` | `SENSOR_DELAY_GAME`, registered `L146` | Always on |
| `fusedLocationClient` (`L50`) | Google Play Fused | 1000 ms / 500 ms min (`L441–L443`) | Callbacks on `Looper.getMainLooper()` |
| Camera | via `processCameraFrame` (`L611`) | CameraX | Forwarded to `vioExecutor` thread `NavSight-VIO` (`L58–L60`) |
| Depth | from same Y plane | `DEPTH_THROTTLE_MS = 1000L` (1 Hz, `L81`) | `NavSight-Depth` thread (`L76–L78`) |
| Rolling-shutter skew | `CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW` | per frame, written into `rollingShutterSkewNs` (`L68`) via `updateRollingShutterSkew` (`L71–L73`) | Step 8c |

**NOT subscribed**: barometer (`TYPE_PRESSURE`), linear-accel, uncalibrated gyro/accel, rotation vector — confirmed by absence of any `getDefaultSensor` calls for those types.

### 1.2 IMU forwarding to native (`onSensorChanged`, `L568–L607`)

- Accel → `orientationTracker.updateAccelerometer(values)` (`L572`) AND `NativeBridge.processAccelerometer(ts, x, y, z)` (`L573`). Also runs shake detector: 20-sample magnitude history (`L580–L582`); if variance > 30.0 over ≥10 samples, calls `resetPath()` (`L589–L591`); throttled to 500 ms (`L583`).
- Mag → `orientationTracker.updateMagnetometer(values)` (`L595`). **Never sent to native** — only used by Kotlin `DeviceOrientationTracker` for UI gating + initial-heading capture.
- Gyro → `NativeBridge.processGyroscope(ts, x, y, z)` (`L598`).
- Orientation StateFlow refreshed every `50_000_000L` ns (50 ms = 20 Hz, `L602–L606`).

### 1.3 Camera frame plumbing (`processCameraFrame`, `L611–L720`)

1. Drop frame if `vioProcessing` (`L613`).
2. Lazy intrinsics: `getCameraIntrinsics(w,h)` (`L494–L536`) reads `LENS_INFO_AVAILABLE_FOCAL_LENGTHS`, `SENSOR_INFO_PHYSICAL_SIZE`, `SENSOR_INFO_PIXEL_ARRAY_SIZE`, computes `fx = (focalMm/sensorWidthMm) * fullPixelWidth`, scales to preview, pushes via `NativeBridge.setIntrinsics` (`L624`). Suppressed if `calibratedIntrinsicsLoaded` (`L492`).
3. Y plane = `image.planes[0]`, UV = `image.planes[2]` (V-plane / NV21, `L632–L633`). Direct ByteBuffers passed to JNI.
4. Optional Y-only copy for depth (`L647–L661`) — honors row stride.
5. VIO branch: submit to `vioExecutor`. Calls `NativeBridge.processCameraFrameDirect(yBuffer, uvBuffer, w, h, yRowStride, uvRowStride, uvPixelStride, timestampNs, rollingShutterSkewNs)` returning `VioData` (`L669–L679`). Logs `"VIO_FPS: jni=Xms total=Yms fc=N pts=K"` every 30 frames (`L683–L685`). `image.close()` always in `finally`.
6. Depth branch: build grayscale ARGB Bitmap, `runBlocking { depthEstimator.estimateDepth(bitmap) }`, push via `NativeBridge.setDepthMap(depthMap, 256, 256)` (`L710`).

### 1.4 Initialization gate (Step 5)

- `enum InitStatus { WAIT_STATIONARY=0, WAIT_MOTION=1, READY=2, TIMEOUT_NEEDS_USER=3 }` (`L44`) — order MUST match native enum.
- `startInitStatusPoller` (`L354–L379`): coroutine polls `NativeBridge.getInitStatus()` every **250 ms** (`L376`); persists via `persistNativeCalibration` (`L382–L402`) when status enters `WAIT_MOTION`/`READY`.
- `pushStoredCalibrationToNative` (`L337–L351`): on startup loads via `CalibrationStore.load(context)` and bypasses gate via `NativeBridge.loadStoredCalibration(rotation, gyroBias, accelBias)`.
- `clearInitTimeout` (`L416–L431`): UI bypass; injects identity rotation + zero biases if no stored calibration.

### 1.5 Camera calibration / extrinsics push at startup (`startSensors`, `L129–L163`)

In order after `NativeBridge.startVIO()`:
1. `pushStoredCalibrationToNative` (IMU calibration)
2. `pushCameraIntrinsicsToNative` (`L177–L195`): if `<filesDir>/camera_calib.json` exists, `NativeBridge.nativeLoadCalibration(file.absolutePath)`; on success sets `calibratedIntrinsicsLoaded = true` and `intrinsicsInitialized = true`.
3. `pushLoopClosureVocabularyToNative` (`L213–L249`, Step 7 / ADR-013): `ORB_VOCAB_ASSET = "ORBvoc.txt"` (`L35`), `ORB_VOCAB_FILE = "ORBvoc.txt"` (`L36`). One-time copy from assets to `<filesDir>` (AGP auto-decompresses .gz so APK ships plain text — verified `L26–L28`). Pushes via `NativeBridge.nativeLoadLoopClosureVocabulary`.
4. `pushExtrinsicsRotationToNative` (`L274–L334`, Step 8b): reads `SENSOR_ORIENTATION` from back camera (typically 90°), computes `R_bc(θ) = R_bc_default × Rz(−θ)` with `R_bc_default = diag(1,−1,−1)` (`L312–L317`), pushes via `NativeBridge.nativeSetExtrinsicsRotation(R_bc)`.
5. `startInitStatusPoller`.

### 1.6 GPS (`L434–L484`, `L759–L811`)

`startGpsUpdates(granted)`: `LocationRequest.Builder(PRIORITY_HIGH_ACCURACY, 1000L).setMinUpdateIntervalMillis(500L)`. Permission-gated; API-S branching for `MissingPermission` suppression. `requestInitialLocation`: `lastLocation` then `getCurrentLocation(PRIORITY_HIGH_ACCURACY, token)`. **Never overwrites `_startLocation` once set** (`L788`) — overwriting after VIO start would shift the entire path.

### 1.7 VIO-init heading capture (`handleVioInitialized`, `L722–L747`)

When `vio.isInitialized` flips true: capture `vioInitAzimuth = orientationState.value.azimuth`, apply `GeomagneticField(lat, lon, 0f, currentTimeMs).declination` correction if GPS available, push `Math.toRadians(correctedAzimuth)` to `NativeBridge.setInitialHeading`, then **unregister magnetometer** (`L743`).

### 1.8 Camera-blocked detector (`checkCameraBlocked`, `L749–L757`)

If `vio.trackedFeatures < 5` increment counter; if `> 30` consecutive low-feature frames set `_showCameraBlocked.value = true`.

### 1.9 Public StateFlows

- `orientationState: StateFlow<OrientationResult>` (`L83–L87`)
- `vioState: StateFlow<VioData>` (`L89–L90`)
- `startLocation: StateFlow<LatLng?>` (`L92–L93`)
- `showCameraBlocked: StateFlow<Boolean>` (`L110–L111`)
- `initStatus: StateFlow<InitStatus>` (`L114–L115`)
- `currentLocation: StateFlow<Location?>` (`L122–L123`)

### 1.10 Magic numbers

| Value | Where | Meaning |
|---|---|---|
| `1000L` ms | `L81` | Depth at 1 Hz |
| `20` | `L96` | Shake-detect window |
| `500` ms | `L583` | Shake-check throttle |
| `30.0` | `L589` | Shake variance threshold |
| `50_000_000L` ns | `L602` | Orientation update period |
| `5` / `30` | `L750`, `L752` | Camera-blocked thresholds |
| `250L` ms | `L376` | Init-status poll period |
| `1000L` / `500L` ms | `L441–L443` | GPS interval / min |

---

## 2. NavigationManager (`NavigationManager.kt`, 504 lines)

**Per the SDD this is an out-of-scope extension** — the core thesis is GPS-denied positioning, not turn-by-turn UX. Wired but not load-bearing.

Companion constants (`L46–L51`):
- `INSTRUCTION_TRIGGER_DISTANCE_METERS = 100.0`
- `ARRIVAL_THRESHOLD_METERS = 20.0`
- `ROUTE_RECALC_DISTANCE_METERS = 50.0` (declared, unused)

### 2.1 State machine (`L411–L421`)

`sealed class NavigationState`: `Idle`, `Routing(destination)`, `Active(route, remainingDistanceMeters, remainingTimeSeconds, currentStepIndex)`, `Arrived`.

### 2.2 Public API

- `suspend fun startNavigation(currentPosition, destination)` (`L87`)
- `fun updateVioPosition(snappedPosition: LatLng)` (`L122`) — re-checks state under `if (_navigationState.value !is Active) return` after computation to avoid race with `cancelNavigation` (`L153`)
- `fun cancelNavigation()` (`L170`)

### 2.3 Routing

`calculateRoute` (`L216–L278`) on `Dispatchers.IO`: `DirectionsApi.newRequest(ctx).origin().destination().mode(TravelMode.DRIVING).await()`. Decodes `overviewPolyline` via `PolylineEncoding.decode`. Per step: `parseManeuver` + `extractStreetName` + strip HTML via `Regex("<[^>]*>")`. Falls back to `calculateFallbackRoute` (`L283–L306`) — straight-line, `estimatedTime = distance / 10.0` (10 m/s).

`parseManeuver` (`L311–L332`): case-insensitive; uturn → sharp → slight/bear → turn → merge → ramp → fork → roundabout → STRAIGHT.

`extractStreetName`: `<b>(.*?)</b>` regex, fallback `"Unknown road"`.

### 2.4 Distance helpers

- `calculateDistance` (`L396–L405`): Haversine, `earthRadius = 6371000.0`
- `findCurrentStep` (`L346–L360`): linear scan from `currentStepIndex` forward; never decreases
- `estimateRemainingTime`: `remainingDistance / 10.0`

### 2.5 Data classes

- `NavigationRoute(start, destination, steps, totalDistanceMeters, estimatedTimeSeconds, polyline)` (`L426–L433`)
- `RouteStep(startLocation, endLocation, distance, maneuver, streetName, instruction)` (`L438–L445`)
- `NavigationInstruction(type, streetName, distanceMeters, instruction)` (`L450–L481`) with `getManeuverIcon(): ImageVector`
- `enum ManeuverType` — 16 values (`L486–L503`): TURN_LEFT/RIGHT, TURN_SLIGHT_LEFT/RIGHT, TURN_SHARP_LEFT/RIGHT, UTURN_LEFT/RIGHT, MERGE, FORK_LEFT/RIGHT, ROUNDABOUT_LEFT/RIGHT, STRAIGHT, RAMP_LEFT/RIGHT

---

## 3. VioData (`VioData.kt`, 124 lines)

Immutable data class. Manual `equals`/`hashCode` (`L55–L123`) needed because `data class` auto-equality fails on `FloatArray` (`trackedPoints`).

| Field | Type | Default | Units / frame |
|---|---|---|---|
| `x, y, z` | `Double` | `0.0` | m, world (Z-up post-2026-05-08 fix) |
| `roll, pitch, yaw` | `Double` | `0.0` | rad |
| `trackingQuality` | `Double` | `0.0` | 0–1 |
| `trackedFeatures, totalFeatures` | `Int` | `0` | counts |
| `estimatedScale` | `Double` | `1.0` | unitless |
| `isInitialized` | `Boolean` | `false` | flag |
| `trackedPoints` | `FloatArray` | empty | `[u₀,v₀,...]` pixels |
| `rawX, rawY, rawZ, rawYaw` | `Double` | `0.0` | pre-EKF VO (sim only) |
| `accelX/Y/Z` | `Float` | `0f` | m/s², body |
| `gyroX/Y/Z` | `Float` | `0f` | rad/s, body |
| `meanFlow` | `Double` | `0.0` | px |
| `inlierCount` | `Int` | `0` | RANSAC inliers |
| `stepCount` | `Int` | `0` | pedometer |
| `stepFreq` | `Double` | `0.0` | Hz |
| `strideLength` | `Double` | `0.0` | m |
| `poseFlags` | `Int` | `0` | bitmask |
| `heading` | `Double` | `0.0` | rad |
| `tdImuCam` | `Double` | `0.0` | s (Step 8a) |

Populated by `NativeBridge.processCameraFrameDirect` JNI return.

---

## 4. DepthEstimator (`DepthEstimator.kt`, 110 lines)

Wraps **MiDaS v2.1 Small** (`MODEL_FILE = "midas_v21_small.tflite"`, `L27`). Implements `AutoCloseable`.

Constants (`L26–L30`): `INPUT_SIZE = 256`, `NUM_THREADS = 4`.

### 4.1 Init (`L42–L62`)

`FileUtil.loadMappedFile(context, MODEL_FILE)`. Try `GpuDelegate()`; fall back to `setNumThreads(4)` on any exception (`L48–L55`). Construct `Interpreter(modelBuffer, options)`.

### 4.2 `estimateDepth(bitmap): FloatArray?` (`L67–L102`)

`suspend` on dedicated single-thread `inferenceDispatcher` (`L36`). Pipeline:
1. `ImageProcessor`: `ResizeOp(256, 256, BILINEAR)` + `NormalizeOp(0f, 255f)` (divides by 255 → [0,1]) + `TensorImage(FLOAT32)`.
2. Output: `ByteBuffer.allocateDirect(256 * 256 * 4)` native byte order.
3. `interpreter.run(processedImage.buffer, outputBuffer)`. Logs `"Inference time: X ms"` at `Log.v`.
4. Read 65536 floats into `FloatArray`.

### 4.3 Fusion contract

Per `SensorRepository.kt:53`: "MiDaS depth feeds Tracker scale constraint (bypasses Mapper)". Pushed via `NativeBridge.setDepthMap(depthMap, 256, 256)` (`SensorRepository.kt:710`). Output is **inverse-depth, unitless** — does NOT enter EKF state vector; serves as scale prior in native Tracker.

### 4.4 `close()` (`L104–L109`)

Closes dispatcher, interpreter, gpuDelegate (each may be null).

---

## 5. DeviceOrientationTracker (`DeviceOrientationTracker.kt`, 222 lines)

Pure Kotlin orientation fuser. **Used only for UI gating + one-shot initial heading.** Hebrew doc-comments throughout.

Companion constants (`L14–L25`):
- `VIO_IDEAL_PITCH_MIN = -75f`
- `VIO_IDEAL_PITCH_MAX = -30f`
- `HORIZONTAL_TOLERANCE_DEGREES = 25f`
- `SMOOTHING_ALPHA = 0.15f`

Android pitch convention (documented `L15–L21`): `-90° = upright`, `0° = flat/floor`. VIO ideal range = phone tilted 30–60° forward from vertical.

### 5.1 Public API

- `data class OrientationResult(pitch, roll, azimuth: Float, isHorizontal: Boolean, deviationFromHorizontal, stabilityScore: Float)` (`L45–L52`)
- `updateAccelerometer(values: FloatArray)` (`L57`)
- `updateMagnetometer(values: FloatArray)` (`L64`)
- `getOrientation(): OrientationResult` (`L71–L125`)
- `reset()` (`L216–L221`)

### 5.2 Fusion math (`L71–L125`)

1. `SensorManager.getRotationMatrix(rotationMatrix, null, accel, mag)`. If `success == false` (mag missing or gimbal lock), fall back to cached smoothed values; return `stabilityScore = 0f`.
2. `SensorManager.getOrientation(rotationMatrix, orientationAngles)`.
3. Convert rad → deg.
4. EMA smooth: `smoothedPitch = smoothedPitch * (1 - α) + pitch * α` with α = 0.15.
5. Azimuth uses `smoothAngle` (`L180–L191`) handling 360° wraparound (shortest delta in [-180, +180]).
6. Push smoothed pitch into `pitchHistory` (cap 10 from `historySize = 10`, `L40`).

### 5.3 `calculateVioDeviation` (`L139–L155`)

Returns 0 if pitch ∈ [−75°, −30°]. Else signed distance outside band. `isHorizontal = deviation <= 25°`.

### 5.4 `calculateStability` (`L160–L175`)

Variance of `pitchHistory`. Returns `1f - clamp(variance / 100f, 0, 1)`.

### 5.5 Magnetometer timeline (per no-mag-during-tracking memo)

- **Pre-init**: registered, feeds `updateMagnetometer`, produces fused azimuth.
- **At VIO init**: azimuth captured, declination-corrected, pushed to native via `setInitialHeading`. Magnetometer **unregistered** (`SensorRepository.kt:743`).
- **Post-init**: `getRotationMatrix` returns false (stale all-zero mag); accel-only fallback path.

---

## 6. RoadSnapper (`RoadSnapper.kt`, 248 lines)

Snaps VIO `LatLng` to nearest road via **Google Roads API** (per-point HTTP, not local map graph). No offline road graph.

Constants (`L31–L35`): `CACHE_SIZE = 50`, `MAX_BATCH_SIZE = 100`.

### 6.1 Members

- `geoApiContext: GeoApiContext?` lazy (`L39–L48`) — null if `apiKey.isBlank()`
- `snapCache: LruCache<String, SnappedLatLng>(50)` (`L51`) — key `"%.6f,%.6f"` (~0.1 m)
- `consecutiveFailures, lastErrorMessage, apiDisabledLogged` — log throttling

### 6.2 `snapToRoad` algorithm (`L65–L187`)

Runs on `Dispatchers.IO`:
1. Cache hit check.
2. Build path: if `recentPath.size >= 2`, take last 5 + current; else just current. Cap at 100.
3. If no API key: return raw `isSnapped = false`.
4. `RoadsApi.snapToRoads(ctx, true, *geoPoints).await()` — `true` is `interpolate`.
5. **Soft-snap gate (FR17, `L131–L141`)**: if Haversine distance from raw VIO to snapped > **15.0 m**, **trust raw VIO** (likely sidewalk, park, or indoor) — return raw with `isSnapped = false`. **This is the snap-vs-free threshold.**
6. Cache + return.
7. Failure handling (`L165–L176`): full stack on first error / on changed message; once-per-20 warn otherwise.

### 6.3 `SnappedLatLng` (`L222–L247`)

Fields: `latitude, longitude: Double`, `placeId: String?`, `originalIndex: Int?`, `isSnapped: Boolean`. Methods: `toLatLng(): LatLng`, `distanceTo(other): Double` (Haversine, R=6371000.0).

---

## 7. AROverlayRenderer (`AROverlayRenderer.kt`, 188 lines)

**Status: entirely dead code.** Header at `L3` reads "DEAD FILE: All composables in AROverlayRenderer are unused." Body wrapped in `/* … */` from `L7` to `L187`. Nothing compiled.

If revived, would have used **Jetpack Compose `Canvas`** (`androidx.compose.foundation.Canvas`) — not OpenGL, not classic `View.Canvas`. Components: `DirectionArrow`, `TiltWarning`, `SpeedIndicator`, `ConfidenceIndicator`. Color constants: `ArrowGreen 0xFF00E676`, `ArrowRed 0xFFFF5252`, `ArrowYellow 0xFFFFEB3B`, `ArrowCyan 0xFF00E5FF`. Was previously called from `AROverlay()` in `MainActivity` (also dead).

---

## 8. GpxExporter (`GpxExporter.kt`, 20 lines)

Single top-level function — no class.

`fun saveGpxFile(context: Context, points: List<Pair<Double, Double>>): String` (`L6–L20`):
- Returns `"Not enough points for GPX"` if `points.size < 2`.
- File: `context.getExternalFilesDir(null)/navsight_${currentTimeMillis()}.gpx`.
- Format: GPX 1.1, single `<trk>` with one `<trkseg>` containing `<trkpt lat="…" lon="…"/>`. **No elevation, no time, no segmentation.**
- Returns `"GPX saved ✓"` on success / `"GPX save failed: …"` on exception.
- Trigger: UI/ViewModel-initiated (user tap), not periodic.

---

## 9. CalibrationStore (`CalibrationStore.kt`, 103 lines)

`object` (singleton). Versioned IMU calibration persistence.

### 9.1 Schema

`SCHEMA_VERSION = 1` (`L21`). Backend: `SharedPreferences` named `"navsight_calibration"` (`L23`), `MODE_PRIVATE`.

`data class CalibrationData` (`L30–L41`):
- `rotation: FloatArray` size **9** (row-major 3×3) — `R_GtoI` (gravity → IMU)
- `gyroBias: FloatArray` size **3** — rad/s, IMU frame
- `accelBias: FloatArray` size **3** — m/s², IMU frame
- `savedAtMs: Long` — epoch ms

`init` block validates sizes (`L36–L40`).

Encoded as comma-separated float strings (SharedPreferences has no native float-array support). Keys: `"version"`, `"rotation"`, `"gyro_bias"`, `"accel_bias"`, `"saved_at_ms"` (`L24–L28`).

**NOT persisted here**: camera intrinsics (live in `<filesDir>/camera_calib.json`); `R_bc` extrinsics (re-derived from `SENSOR_ORIENTATION` every startup); MiDaS model (TFLite asset).

### 9.2 Public API

- `save(context, data)` (`L46–L59`) — writes all 5 keys via `apply()`
- `load(context): CalibrationData?` (`L62–L83`) — returns null if no entry, version mismatch, or parse error. Drops on version mismatch (`L68`); drops on parse error (`L80`). Migration policy: nuke and rebuild.
- `clear(context)` (`L85–L87`) — `prefs.edit().clear().apply()`

---

## 10. CrashLogger (`CrashLogger.kt`, 108 lines)

`object` singleton. Step 6 / Task #31 — uncaught-exception sink.

### 10.1 Output

Directory: `<external-files>/crash_logs/` (or `<files>/crash_logs/` fallback) (`L58–L63`). Filename: `crash_{yyyyMMdd_HHmmss}_{epoch_ms}.json` (`L67–L68`).

JSON keys (`L74–L84`): `timestamp_ms`, `timestamp_iso`, `thread`, `exception_class`, `exception_message`, `stack_trace`, `snapshot`. Manual JSON construction with `escape()` helper (`L88–L107`) handling `\`, `"`, `\n`, `\r`, `\t`, `\b`, `\f`, and `\uXXXX` for control chars (threshold `0x20`, `L99`).

### 10.2 Severity / rotation

**No severity levels** — only fatal uncaught crashes are written. `info/warn/error` go to `android.util.Log`. **No rotation policy** — files accumulate; external cleanup responsibility.

### 10.3 Public API

- `updateSnapshot(json: String)` (`L33–L35`): atomically replaces `AtomicReference<String>` (`L29`); called from ViewModel every VIO frame.
- `@Synchronized install(context)` (`L42–L56`): idempotent; captures previous `UncaughtExceptionHandler` and **chains to it after writing** so platform crash dialog still fires.

Threading: `snapshotRef` is `AtomicReference<String>` — lock-free reads/writes from any thread. Handler runs on crashing thread, top-level `try/catch (Throwable)` (`L48–L52`).

---

## 11. NavSightUtils (`NavSightUtils.kt`, 106 lines)

`object` of stateless helpers. Six live functions, three commented-out dead.

| Function | Signature | Line | Purpose |
|---|---|---|---|
| `resolveNavigationStart` | `(snappedPosition: LatLng?, startLocation: LatLng?): LatLng?` | `L14–L16` | Returns `snappedPosition ?: startLocation` |
| `computeUpdatedScaleCalibrationFactor` | `(currentFactor, knownDistance, measuredDistance: Double): Double?` | `L24–L34` | Multiplies by `known/measured`; null if any ≤ 0; clamps result to `[0.25, 4.0]` |
| `metersToLatLng` | `(start: LatLng, dx: Double, dz: Double): LatLng` | `L36–L58` | Geodesic destination point. `bearing = atan2(dx, dz)` (z=north, x=east). Earth `R = 6378137.0` (WGS84 equatorial). Returns `start` if `distance < 0.001`. |
| `formatDistance` | `(meters: Int): String` | `L60–L65` | `<1000` → `"{m}m"`; else `"{km}km"` 1 decimal |
| `formatTime` | `(seconds: Int): String` | `L67–L73` | `<60min` → `"{m}min"`; else `"{h}h {m}min"` |
| `vectorToBitmap` | `(context: Context, drawableId: Int): BitmapDescriptor?` | `L88–L104` | Vector drawable → ARGB_8888 Bitmap → `BitmapDescriptorFactory.fromBitmap`. For Maps custom markers. Null on exception. |

Dead code (commented):
- `computeCalibrationStraightness` (`L19–L22`)
- `nv21ToBitmap` (`L76–L86`) — superseded by `processCameraFrameDirect`

---

## 12. Cross-cutting

### 12.1 Threading map

| Thread | Owner | Work |
|---|---|---|
| Android main | `repositoryScope: Dispatchers.Main + SupervisorJob()` (`SensorRepository.kt:127`) | StateFlow updates, init poller |
| Android sensor HAL | `onSensorChanged` callback | Forward IMU to native (low-latency) |
| `NavSight-VIO` (single, daemon) | `vioExecutor` (`L58`) | `processCameraFrameDirect` JNI |
| `NavSight-Depth` (single, daemon) | `depthExecutor` (`L76`) | Bitmap conversion + `runBlocking { estimateDepth }` |
| TFLite single-thread | `DepthEstimator.inferenceDispatcher` (`L36`) | `interpreter.run` |
| `Dispatchers.IO` | `RoadsApi`, `DirectionsApi`, `RoadSnapper.snapToRoad` | HTTP |
| Crashing thread | `CrashLogger` handler (`L46`) | Best-effort JSON write |

### 12.2 NativeBridge surface used from this layer

`startVIO`, `stopVIO`, `resetVIO`, `processAccelerometer(ts, x, y, z)`, `processGyroscope(ts, x, y, z)`, `processCameraFrameDirect(...)` (returns `VioData`), `setIntrinsics(fx, fy, cx, cy)` (fallback), `nativeLoadCalibration(path): Boolean` (preferred), `nativeLoadLoopClosureVocabulary(path): Boolean`, `nativeSetExtrinsicsRotation(R_bc: FloatArray)` (Step 8b), `setDepthMap(depth, w, h)` (256×256 push), `setInitialHeading(azimuthRad: Double)`, `getInitStatus(): Int` (4 Hz poll), `getCalibration(rot, gyroBias, accelBias): Boolean`, `loadStoredCalibration(rot, gyroBias, accelBias)`, `clearInitTimeout()`, `isLoaded(): Boolean`.

### 12.3 Aggregated magic-number index

| Domain | Constant | Value |
|---|---|---|
| Depth throttle | `DEPTH_THROTTLE_MS` | `1000L` ms |
| Shake reset | accel-magnitude variance | `30.0` |
| Shake check | window / period / min | 20 / 500 ms / 10 |
| Camera blocked | features < / fails > | `5` / `30` |
| Init poll | period | `250L` ms |
| Orientation update | period | `50_000_000L` ns |
| GPS | interval / min | `1000L` / `500L` ms |
| Nav | trigger / arrival / recalc | 100 / 20 / 50 m |
| Nav | avg speed | 10 m/s |
| Nav | earth R | 6371000.0 m |
| Depth | input / threads | 256 / 4 |
| Depth | normalize | 0f, 255f |
| Orient | pitch range / tol / α / hist | [−75°, −30°] / 25° / 0.15 / 10 |
| Orient | stability normalizer | 100f |
| Snap | cache / batch / context | 50 / 100 / 5 |
| Snap | soft-snap gate | **15.0 m** |
| Snap | cache key precision | 6 decimals |
| Snap | log throttle | every 20 |
| GPX | min points | 2 |
| Calib | schema | 1 |
| Calib | array sizes | 9 / 3 / 3 |
| Utils | scale clamp | [0.25, 4.0] |
| Utils | WGS84 equatorial R | 6378137.0 m |
| Utils | min step | 0.001 m |

### 12.4 Frame-of-reference summary

| Quantity | Frame | Unit |
|---|---|---|
| `VioData.x/y/z` | World, **Z-up** post-2026-05-08 fix | m |
| `VioData.roll/pitch/yaw, heading` | World | rad |
| `VioData.accelX/Y/Z` | Body (IMU) | m/s² |
| `VioData.gyroX/Y/Z` | Body (IMU) | rad/s |
| `VioData.tdImuCam` | — | s |
| `VioData.trackedPoints` | Image | px (u, v interleaved) |
| `OrientationResult.pitch/roll/azimuth` | Device | degrees |
| `CalibrationData.rotation` | `R_GtoI` (gravity → IMU) | row-major 3×3 |
| `R_bc` extrinsics | body ← camera | row-major 3×3 |
| MiDaS depth (256×256) | image | inverse-depth, unitless |
| Roads API positions | WGS84 | deg lat/lon |
| `metersToLatLng` `dx, dz` | local ENU (z=north, x=east) | m |

---

## Key findings

1. **Sensor budget is intentionally minimal**: only accel + gyro feed native after init. Mag is one-shot (initial heading only). No barometer, no rotation vector, no uncalibrated IMU types subscribed.
2. **Magnetometer deregistration at `SensorRepository.kt:743`** is the concrete enforcement of the no-mag-during-tracking rule.
3. **MiDaS depth runs at 1 Hz** (`DEPTH_THROTTLE_MS = 1000L`) and is informational/scale-prior for the native Tracker — it does NOT enter the EKF state vector. Output is 256×256 inverse-depth floats pushed via `NativeBridge.setDepthMap`.
4. **Camera intrinsics path is dual**: preferred is `<filesDir>/camera_calib.json` loaded via `nativeLoadCalibration` (sets `calibratedIntrinsicsLoaded = true` to suppress fallback); fallback is on-the-fly derivation from `CameraCharacteristics` pushed lazily on first frame via `setIntrinsics`.
5. **`R_bc` extrinsics are seeded fresh each session** from back-camera `SENSOR_ORIENTATION` via `pushExtrinsicsRotationToNative` — not persisted.
6. **CalibrationStore persists ONLY IMU calibration**: `R_GtoI` rotation (9 floats), gyro bias, accel bias, with `SCHEMA_VERSION = 1` in SharedPreferences `"navsight_calibration"`. Migration policy is "nuke and rebuild" on version mismatch.
7. **RoadSnapper has a 15 m soft-snap gate (FR17)**: beyond 15 m from snapped road, raw VIO is trusted (sidewalk/park/indoor heuristic). Cache key is 6-decimal lat/lon (~0.1 m).
8. **AROverlayRenderer is entirely dead** — file body is one big `/* … */` block.
9. **NavigationManager is wired but out-of-scope** per the SDD; recall route via Google Directions API with PolylineEncoding decode and a Material-Icons-only maneuver mapping.
10. **Threading**: dedicated single-thread daemon executors `NavSight-VIO` and `NavSight-Depth`; TFLite has its own dedicated dispatcher; `repositoryScope` is `Dispatchers.Main + SupervisorJob()` and is recreated if cancelled.
11. **CrashLogger has no rotation policy** — files accumulate at `<external-files>/crash_logs/`; only fatal uncaught crashes are captured (the `snapshot` field carries the latest VIO state via `updateSnapshot` from the ViewModel).
12. **Dead code identified**: `SensorRepository.setScale` (`L825–L827`), `DeviceOrientationTracker.isPhoneHorizontal` / `getCompassHeading`, `RoadSnapper.clearCache`, `NavSightUtils.computeCalibrationStraightness` / `nv21ToBitmap`, the entire `AROverlayRenderer.kt` body.
