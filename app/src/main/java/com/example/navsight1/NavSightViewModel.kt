package com.example.navsight1

import android.app.Application
import android.location.Location
import android.util.Log
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.getValue
import androidx.compose.runtime.setValue
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.google.android.gms.maps.model.LatLng
import androidx.camera.core.ImageProxy
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.sample
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlin.math.*
import java.util.concurrent.atomic.AtomicInteger

/**
 * One sample on the VIO trajectory in local (x, z) meters with the
 * 1-sigma horizontal-plane position uncertainty at that sample, in meters.
 * `sigmaM` is `Float.NaN` when the EKF has not yet exposed a valid covariance
 * (e.g. before full initialization).
 */
data class PathPoint(val x: Float, val z: Float, val sigmaM: Float)

class NavSightViewModel(application: Application) : AndroidViewModel(application) {
    data class ScaleCalibrationSession(
        val legDistanceMeters: Double,
        val startX: Double,
        val startZ: Double,
        val lastX: Double,
        val lastZ: Double,
        val pathLengthMeters: Double,
        val sampleCount: Int,
        val maxDistanceFromStartMeters: Double,
        val sumQuality: Double,
        val lowQualityFrames: Int
    )

    companion object {
        private const val PREFS_NAME = "navsight_prefs"
        private const val PREF_SCALE_CALIBRATION_FACTOR = "scale_calibration_factor"
        private const val PREF_USER_HEIGHT = "user_height_m"
        // 2026-05-28 — persisted MiDaS scale K across app launches. See NativeBridge
        // setMidasScaleK comment for why this is necessary (first-launch walks fail
        // to calibrate K because essential-matrix verification fails → looming
        // bails → UI sits at 0). Float because SharedPreferences has no Double.
        private const val PREF_MIDAS_SCALE_K = "midas_scale_k"
        // Save K to prefs every N milliseconds while VIO is running. Dropped
        // from 10 s to 3 s on 2026-05-28: short recordings (10 s run) finished
        // before the 10 s persist tick ever fired, so K=1499 from the run was
        // lost on the next startVIO. 3 s gives ~3 persists per 10 s recording.
        private const val MIDAS_K_PERSIST_INTERVAL_MS = 3_000L

        // Tier-1 #4 — minimum OSRM /match HMM confidence to trust the matched dot.
        // Below this (or on a broken/split trace) we fall back to the per-point snapper.
        private const val MIN_MATCH_CONFIDENCE = 0.30
        // Tier-1 #2 — read-only K̂_map scale-observation gates. Higher confidence bar
        // than the dot (scale is sensitive), enough VIO baseline to be meaningful, and
        // a sane ratio band so a wrong match never logs a garbage scale.
        private const val SCALE_OBS_MIN_CONFIDENCE = 0.60
        private const val SCALE_OBS_MIN_VIO_M = 15.0
        private const val SCALE_OBS_MIN_RATIO = 0.2
        private const val SCALE_OBS_MAX_RATIO = 5.0
        private const val SCALE_OBS_LOG_INTERVAL_MS = 3_000L
    }

    private val sensorRepository = SensorRepository(application)
    private val apiKey = BuildConfig.GOOGLE_MAPS_API_KEY
    private val navigationManager = NavigationManager(application, apiKey)
    private val roadSnapper = RoadSnapper(apiKey = apiKey)
    private val prefs = application.getSharedPreferences(PREFS_NAME, Application.MODE_PRIVATE)

    // ── SENSOR ACTIVE FLAG ────────────────────────────────────────────────────
    // Used by CameraViewComposable to guard against sending frames to a terminated
    // ThreadPool (RejectedExecutionException) or a closed buffer (IllegalStateException).
    @Volatile
    private var sensorRepositoryActive = false

    /** Returns true only when SensorRepository's ThreadPool is running and accepting frames. */
    fun isSensorRepositoryActive(): Boolean = sensorRepositoryActive
    // ──────────────────────────────────────────────────────────────────────────

    // UI States
    var orientationState by mutableStateOf(DeviceOrientationTracker.OrientationResult(
        pitch = 0f, roll = 0f, azimuth = 0f,
        isHorizontal = false, deviationFromHorizontal = 90f, stabilityScore = 0f
    ))
        private set

    var vioState by mutableStateOf(VioData())
        private set

    /**
     * Phase 1 camera-overlay plumbing: latest analyzer-frame geometry
     * (width, height, CCW rotation degrees applied by CameraX). Read by
     * `CameraFeatureOverlay` to map analyzer-space KLT points onto the
     * Compose Canvas. Null until the first camera frame lands.
     */
    var cameraFrameGeometry by mutableStateOf<SensorRepository.CameraFrameGeometry?>(null)
        private set

    /**
     * Camera overlay Phase 2/3/4 state (camera_overlay_phase23_plan.md).
     * Updated on EVERY native VIO frame (~30 Hz), bypassing the 200 ms
     * UI throttle in [handleVioUpdate]. Heavy [vioState] consumers
     * (path history, sigma read, GPS snap, crash logger) stay throttled
     * for performance; the overlay reads these per-frame fields so KLT
     * dots track the live preview with no perceptible lag.
     *
     * The fix for the user's "Phase-1 dots are laggy" report is exactly
     * this: lift the overlay's data sources off the throttled vioState
     * onto a parallel un-throttled flow.
     */
    var overlaySnapshot by mutableStateOf(
        SensorRepository.OverlaySnapshot(
            floatArrayOf(), intArrayOf(), floatArrayOf(), floatArrayOf(), 0L
        )
    )
        private set

    /**
     * Loop-closure flash window. The overlay reads this every frame and
     * if `System.currentTimeMillis() < loopClosureFlashUntilMs` it draws
     * the "LOOP CLOSURE" banner with alpha decaying linearly to 0 across
     * the remaining time. Set to one second in the future whenever the
     * native EventCounters.loop_closure_corrections_applied increments.
     */
    var loopClosureFlashUntilMs by mutableStateOf(0L)
        private set
    private var lastLoopClosureCount = 0L

    var virtualX by mutableStateOf(0.0)
        private set
    var virtualZ by mutableStateOf(0.0)
        private set

    private val _pathHistory = ArrayList<PathPoint>(512)
    private val _pathHistoryVersion = AtomicInteger(0)
    var pathHistoryVersion by mutableStateOf(0)
        private set
    val pathHistory: List<PathPoint> get() = _pathHistory
    // 2026-05-26 — #2 loop-overlay path redraw. On a loop_correction_version bump
    // (a loop closure re-optimized the pose graph), pathHistory is rebuilt from the
    // CORRECTED node polyline so the two loops snap onto each other.
    private var lastLoopCorrectionVersion = 0
    private val correctedTrajBuf = FloatArray(1000)   // 500 (x,z) pairs = PoseGraph MAX_NODES
    private val correctedPathSigmaM = 0.5f            // confident sigma for redrawn points (map color)

    // Step 6: current 1-sigma horizontal-plane position uncertainty (meters).
    // NaN until the EKF has finished its full init. Backed by a per-frame read
    // of NativeBridge.getPositionCovariance(), with sigma = sqrt(σ_xx + σ_zz).
    var positionSigmaM by mutableStateOf(Float.NaN)
        private set
    // True while the EKF is reporting a valid covariance block (full init).
    var positionCovValid by mutableStateOf(false)
        private set
    // Reusable buffer to avoid per-frame allocation when polling covariance.
    private val covBuf = FloatArray(3)

    var startLocation by mutableStateOf<LatLng?>(null)
        private set
    // When the user long-presses the map to relocate the start pin, GPS fixes must not
    // clobber the chosen anchor (the set-once GPS normally wins).
    private var startLocationOverridden = false

    var navigationState by mutableStateOf<NavigationState>(NavigationState.Idle)
        private set

    var currentInstruction by mutableStateOf<NavigationInstruction?>(null)
        private set

    var snappedPosition by mutableStateOf<LatLng?>(null)
        private set

    var currentSpeedKmh by mutableStateOf(0f)
        private set

    var totalDistanceM by mutableStateOf(0.0)
        private set

    var showCameraBlocked by mutableStateOf(false)
        private set
    var initStatus by mutableStateOf(SensorRepository.InitStatus.WAIT_STATIONARY)
        private set
    var navigationStartMessage by mutableStateOf<String?>(null)
        private set
    var scaleCalibrationFactor by mutableStateOf(
        prefs.getFloat(PREF_SCALE_CALIBRATION_FACTOR, 1.0f).toDouble()
    )
        private set
    var scaleCalibrationMessage by mutableStateOf<String?>(null)
        private set
    var scaleCalibrationSession by mutableStateOf<ScaleCalibrationSession?>(null)
        private set
    var userHeight by mutableStateOf(prefs.getFloat(PREF_USER_HEIGHT, 1.70f))
        private set

    // Step 1 (Visual plan): camera-calibration JSON existence.
    // True when filesDir/camera_calib.json is present at startup or after the
    // in-app calibration screen saves a fresh result. Drives the main-screen
    // status pill and the first-launch banner. Updated synchronously from the
    // UI layer via [refreshCalibrationLoaded].
    var calibrationLoaded by mutableStateOf(calibrationExists(application))
        private set

    /** Re-check whether camera_calib.json exists on disk. */
    fun refreshCalibrationLoaded() {
        calibrationLoaded = calibrationExists(getApplication())
    }

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    var isRecordingSimulation by mutableStateOf(false)
        private set
    private val simulationDataPoints = mutableListOf<SimulationPoint>()

    // Step 9 / ADR-014 — frame recorder lifecycle. Created in
    // toggleSimulationRecording when the user starts a sim, drained in
    // saveSimulationData when they stop. Stats are embedded into the
    // simulator JSON so the replay-harness fixture can self-describe.
    private var simFrameRecorder: SimulationFrameRecorder? = null
    private var simFrameStats: SimulationFrameRecorder.Stats? = null
    private var simFrameStartTimeMs: Long = 0L
    private var currentGpsLocation: Location? = null

    data class SimulationPoint(
        val timestamp: Long,
        val vioX: Double, val vioY: Double, val vioZ: Double,
        val vioYaw: Double, val vioScale: Double, val vioQuality: Double,
        val rawX: Double, val rawY: Double, val rawZ: Double, val rawYaw: Double,
        val accelX: Float, val accelY: Float, val accelZ: Float,
        val gyroX: Float, val gyroY: Float, val gyroZ: Float,
        val gpsLat: Double?, val gpsLng: Double?, val gpsAlt: Double?, val gpsAcc: Float?,
        val meanFlow: Double, val inlierCount: Int,
        val stepCount: Int, val stepFreq: Double, val strideLength: Double,
        val poseFlags: Int, val heading: Double,
        // Map-matching Step B* (§8M) — VIO position projected to geographic
        // (lat,lng) via the SessionAnchor, with the EKF's xy-covariance trace.
        // null until a SessionAnchor exists (no GPS fix / jammed).
        val vioLat: Double?, val vioLng: Double?, val vioVarXy: Double?,
        // §0.8 — the map-matched (OSM-snapped) track, same geographic frame as
        // glat/glng. null on samples where the matcher produced nothing.
        val mmLat: Double?, val mmLng: Double?, val mmConf: Double?,
        // Matcher diagnostics persisted for offline analysis (logcat is ephemeral):
        //   mmSrc    — "osrm:match" | "snap" | "raw" | null (which matcher produced the dot)
        //   kMap     — per-sample K̂_map = d_route/d_vio (scale observation; null when unmatched)
        //   maneuver — ManeuverState ("FREE_ROAD" | "ON_ROUNDABOUT" | "MID_ROAD_UTURN")
        val mmSrc: String?, val kMap: Double?, val maneuver: String?
    )
    // ──────────────────────────────────────────────────────────────────────────

    val vioInitAzimuth: Float get() = sensorRepository.vioInitAzimuth

    private var lastVioForSpeed: VioData? = null
    // 2026-05-26 — EMA state for the locomotion-agnostic fused-speed display. Smooths
    // NativeBridge.getFusedSpeedMps() (|v_G_|) into a stable speedometer reading. See
    // the speed block in the VIO update loop; reset in resetPath()/resetAll().
    private var speedEmaMps = 0f
    private var lastVioForDist: VioData? = null
    private var lastSpeedTimeMs = 0L
    private var lastSnapTimeMs = 0L
    // Heading-aware map-matcher (§0.7): detects roundabout traversal/exit + U-turns
    // from the VIO heading. Session-scoped; reset in resetAll(). maneuverState is
    // exposed for the UI (e.g. a roundabout indicator).
    private val maneuverStateMachine = ManeuverStateMachine()
    var maneuverState by mutableStateOf(ManeuverState.FREE_ROAD)
        private set

    // Step D — live trajectory map-matcher (OSRM /match HMM). Primary dot source:
    // matches the recent VIO PATH to the road so the dot follows curves/roundabout
    // rings instead of "letting go" past the per-point soft-snap cap. Falls back to
    // [roadSnapper] (per-point) when offline / no match. Source logged on transition.
    // Local on-device Viterbi map-matcher (the offline replacement for the OSRM /match URL —
    // no network/rate-limits/coordinate-cap). LiveMatcher (OSRM) is kept for reference but no
    // longer the dot source.
    private val localMatcher = LocalMatcher()
    @Suppress("unused") private val liveMatcher = LiveMatcher()
    @Volatile private var lastSnapSource = ""
    // Guards against overlapping snap coroutines while a /match HTTP call is in flight
    // (set/cleared on the Main collector thread + the IO finally).
    @Volatile private var snapInFlight = false
    // Tier-1 #3: the matched ROAD polyline for the current window (on-road geometry the
    // dot sits on). Empty when on the per-point fallback / unmatched. Rendered by the map.
    var matchedRoadPath by mutableStateOf<List<LatLng>>(emptyList())
        private set
    // Tier-1 #2: throttle for the read-only K̂_map scale observation log (ms).
    @Volatile private var lastScaleObsMs = 0L
    // Latest per-sample matcher diagnostics, stashed by the snap coroutine and recorded
    // into the sim JSON (logcat is ephemeral; these must survive in the recording).
    @Volatile private var lastKMap: Double? = null

    // §0.8 / Step C*: the latest MAP-MATCHED position (the OSM-snapped dot), set
    // only when the snap actually landed on a road (isSnapped). null otherwise.
    // Recorded per sample as mm_lat/mm_lng for the analyzer's magenta track.
    @Volatile private var lastMapMatched: LatLng? = null
    // §0.8 — per-sample map-match confidence = exp(-snapDistanceM / 5.0), mirroring
    // RoadSnapper's SNAP_SIGMA_M=5 m. null when not snapped (no map-matched point).
    @Volatile private var lastMapMatchedConf: Double? = null
    private var lastUiUpdateTimeMs = 0L
    private val UI_UPDATE_THROTTLE_MS = 200L

    private var latestVioState: VioData = VioData()
    private var hasLocationPermission = false

    // MIGRATION 2026-05-30 (MAP_MATCHING_PLAN.md §8M Step K-search*): offline OSM
    // destination search replaces the Google Places API. Reaches the loaded
    // OsmDataLayer through its process-wide holder; $0, no key, fully offline.
    val offlineGeocoder = OfflineGeocoder()

    /* LEGACY (Google Places API, removed in OSM migration 2026-05-30). Commented
       out per the comment-out rule.
    val placesClient by lazy {
        if (apiKey.isNotBlank() && !com.google.android.libraries.places.api.Places.isInitialized()) {
            com.google.android.libraries.places.api.Places.initialize(getApplication(), apiKey)
        }
        if (com.google.android.libraries.places.api.Places.isInitialized()) {
            com.google.android.libraries.places.api.Places.createClient(getApplication())
        } else {
            throw IllegalStateException("Places API not initialized. Check your GOOGLE_MAPS_API_KEY.")
        }
    }
    */

    // Map-matching Step B* (§8M): true once the bootstrap GPS fix has set the
    // native SessionAnchor. Guards against re-anchoring (native is also idempotent).
    @Volatile
    private var sessionAnchorSet = false

    fun updateUserHeight(height: Float) {
        val h = height.coerceIn(1.0f, 2.5f)
        userHeight = h
        prefs.edit().putFloat(PREF_USER_HEIGHT, h).apply()
        NativeBridge.setUserHeight(h)
    }

    private var lastMidasKPersistMs = 0L

    init {
        NativeBridge.setScale(scaleCalibrationFactor)
        NativeBridge.setUserHeight(userHeight)
        // 2026-05-28 — push persisted MiDaS scale K to native at startup.
        // -1 = never calibrated; native ignores non-positive values so this is a
        // safe no-op on first launch. Once any session calibrates K (run, walk+run,
        // or a fast-enough walk that passes essential-matrix verification), the
        // value persists and future cold-start walks inherit it immediately so
        // the looming path (which gates on K>0) fires from frame 1.
        val persistedK = prefs.getFloat(PREF_MIDAS_SCALE_K, -1f).toDouble()
        if (persistedK > 0.0) {
            NativeBridge.setMidasScaleK(persistedK)
        }

        // Map-matching §8M: load the embedded Israel-wide SEARCH index off the main
        // thread, and init the dynamic ROAD-region manager (roads are fetched on
        // demand around the user's location — RoadRegionManager). Both degrade
        // gracefully (raw-VIO display, empty search) if assets/network are absent.
        viewModelScope.launch(Dispatchers.Default) {
            OsmDataLayer.load(getApplication())
            RoadRegionManager.init(getApplication())
        }

        viewModelScope.launch {
            sensorRepository.orientationState.sample(200L).collect { orientationState = it }
        }
        viewModelScope.launch {
            sensorRepository.vioState.collect { vio -> handleVioUpdate(vio) }
        }
        viewModelScope.launch {
            sensorRepository.cameraFrameGeometry.collect { cameraFrameGeometry = it }
        }
        viewModelScope.launch {
            sensorRepository.overlaySnapshot.collect { snap ->
                // Camera-overlay Phase 2/3/4: per-frame snapshot, NOT subject
                // to the 200 ms UI throttle. Detect loop-closure increments
                // here so the flash banner reacts within one frame of the
                // native correction.
                overlaySnapshot = snap
                if (snap.loopClosureCount > lastLoopClosureCount) {
                    lastLoopClosureCount = snap.loopClosureCount
                    loopClosureFlashUntilMs = System.currentTimeMillis() + 1000L
                }
            }
        }
        viewModelScope.launch {
            sensorRepository.startLocation.collect {
                // Respect a user long-press override; GPS otherwise sets the anchor once.
                if (!startLocationOverridden) startLocation = it
                // Dynamic roads: begin loading the road region around the user the
                // instant the first GPS fix lands (before VIO produces a position).
                if (it != null) RoadRegionManager.ensureRegion(it.latitude, it.longitude)
                // Map-matching Step B* (§8M): the FIRST bootstrap GPS fix anchors the
                // VIO local frame to geographic coordinates (ADR-004 — one fix, never
                // feeds the EKF). Idempotent on the native side; guarded here too.
                if (!sessionAnchorSet && it != null) {
                    sessionAnchorSet = true
                    runCatching {
                        NativeBridge.nativeSetSessionAnchor(
                            it.latitude, it.longitude, System.currentTimeMillis() * 1_000_000L
                        )
                    }.onFailure { e -> Log.w("NavSightVM", "setSessionAnchor failed: ${e.message}") }
                }
            }
        }
        viewModelScope.launch {
            sensorRepository.showCameraBlocked.collect { showCameraBlocked = it }
        }
        viewModelScope.launch {
            sensorRepository.initStatus.collect { initStatus = it }
        }
        viewModelScope.launch {
            sensorRepository.currentLocation.collect { currentGpsLocation = it }
        }
        viewModelScope.launch {
            navigationManager.navigationState.collect { navigationState = it }
        }
        viewModelScope.launch {
            navigationManager.currentInstruction.collect { currentInstruction = it }
        }
    }

    private fun handleVioUpdate(vio: VioData) {
        latestVioState = vio
        val nowMs = System.currentTimeMillis()
        val shouldUpdateUI = (nowMs - lastUiUpdateTimeMs) >= UI_UPDATE_THROTTLE_MS

        if (shouldUpdateUI) {
            lastUiUpdateTimeMs = nowMs
            vioState = vio
            if (vio.isInitialized) {
                virtualX = vio.x
                virtualZ = vio.z
                // Step 6: pull horizontal-plane position covariance from EKF and
                // derive sigma_h = sqrt(σ_xx + σ_zz). NaN before full init.
                val covOk = NativeBridge.isLoaded() &&
                    NativeBridge.getPositionCovariance(covBuf)
                val sigma = if (covOk) {
                    val trace = covBuf[0] + covBuf[2]
                    if (trace > 0f) sqrt(trace) else 0f
                } else Float.NaN
                positionCovValid = covOk
                positionSigmaM = sigma
                // 2026-05-26 — #2 loop-overlay path redraw. When a loop closure
                // re-optimized the pose graph (version bumped), rebuild the
                // (drifted) pathHistory from the CORRECTED node polyline so the
                // two loops snap together; the live per-frame point below then
                // appends to the corrected history.
                if (NativeBridge.isLoaded()) {
                    val lcVer = NativeBridge.getLoopCorrectionVersion()
                    if (lcVer != lastLoopCorrectionVersion) {
                        lastLoopCorrectionVersion = lcVer
                        val nPairs = NativeBridge.getCorrectedTrajectory(correctedTrajBuf)
                        if (nPairs > 0) {
                            _pathHistory.clear()
                            for (i in 0 until nPairs) {
                                _pathHistory.add(
                                    PathPoint(
                                        correctedTrajBuf[2 * i],
                                        correctedTrajBuf[2 * i + 1],
                                        correctedPathSigmaM
                                    )
                                )
                            }
                            android.util.Log.i(
                                "NavSight-VM",
                                "PATH_REDRAW: loop_correction_version=$lcVer corrected_nodes=$nPairs"
                            )
                        }
                    }
                }
                _pathHistory.add(PathPoint(vio.x.toFloat(), vio.z.toFloat(), sigma))
                if (_pathHistory.size > 500) _pathHistory.removeAt(0)
                pathHistoryVersion = _pathHistoryVersion.incrementAndGet()
                // Step 6 (Task #31): publish a compact snapshot for the crash
                // logger so the report includes the last good VIO state.
                CrashLogger.updateSnapshot(buildCrashSnapshotJson(vio, sigma, covOk))
            }
        }

        if (vio.isInitialized) {
            scaleCalibrationSession?.let { session ->
                val dx = vio.x - session.lastX
                val dz = vio.z - session.lastZ
                val startDx = vio.x - session.startX
                val startDz = vio.z - session.startZ
                val distanceFromStart = sqrt(startDx * startDx + startDz * startDz)
                scaleCalibrationSession = session.copy(
                    lastX = vio.x, lastZ = vio.z,
                    pathLengthMeters = session.pathLengthMeters + sqrt(dx * dx + dz * dz),
                    sampleCount = session.sampleCount + 1,
                    maxDistanceFromStartMeters = max(session.maxDistanceFromStartMeters, distanceFromStart),
                    sumQuality = session.sumQuality + vio.trackingQuality,
                    lowQualityFrames = session.lowQualityFrames + if (vio.trackingQuality < 0.2) 1 else 0
                )
            }

            if (isRecordingSimulation) {
                val gps = currentGpsLocation
                // Step B* (§8M): VIO position projected to geographic (lat,lng) — the
                // user-facing dot (global_t_) via the SessionAnchor. null when no anchor.
                val vioLla = runCatching { NativeBridge.nativeCurrentVioLla() }.getOrNull()
                val mm = lastMapMatched   // §0.8 — latest OSM-snapped position (or null)
                val mmC = lastMapMatchedConf   // §0.8 — its confidence (or null)
                val mmS = lastSnapSource.ifEmpty { null }   // matcher source (osrm:match/snap/raw)
                val kM = lastKMap   // per-sample scale observation (null when unmatched)
                val mvr = maneuverState.name   // current maneuver state (Main thread read)
                synchronized(simulationDataPoints) {
                    simulationDataPoints.add(SimulationPoint(
                        timestamp = System.currentTimeMillis(),
                        vioX = vio.x, vioY = vio.y, vioZ = vio.z,
                        vioYaw = vio.yaw, vioScale = vio.estimatedScale, vioQuality = vio.trackingQuality,
                        rawX = vio.rawX, rawY = vio.rawY, rawZ = vio.rawZ, rawYaw = vio.rawYaw,
                        accelX = vio.accelX, accelY = vio.accelY, accelZ = vio.accelZ,
                        gyroX = vio.gyroX, gyroY = vio.gyroY, gyroZ = vio.gyroZ,
                        gpsLat = gps?.latitude, gpsLng = gps?.longitude,
                        gpsAlt = gps?.altitude, gpsAcc = gps?.accuracy,
                        meanFlow = vio.meanFlow, inlierCount = vio.inlierCount,
                        stepCount = vio.stepCount, stepFreq = vio.stepFreq,
                        strideLength = vio.strideLength, poseFlags = vio.poseFlags, heading = vio.heading,
                        vioLat = vioLla?.getOrNull(0), vioLng = vioLla?.getOrNull(1),
                        vioVarXy = vioLla?.getOrNull(3),
                        mmLat = mm?.latitude, mmLng = mm?.longitude, mmConf = mmC,
                        mmSrc = mmS, kMap = kM, maneuver = mvr
                    ))
                }
            }

            val prevDist = lastVioForDist
            if (prevDist != null) {
                val ddx = vio.x - prevDist.x
                val ddz = vio.z - prevDist.z
                totalDistanceM += sqrt(ddx * ddx + ddz * ddz)
            }
            lastVioForDist = vio

            // 2026-05-26 — locomotion-agnostic speed. Report the smoothed
            // depth-weighted metric speed (getFusedSpeedMps: recoverPose translation
            // scaled by tracked-point MiDaS depths) for ALL motion types instead of
            // differencing a stride-scaled position. Returns -1.0 before the first
            // estimate; until then show 0.
            if (NativeBridge.isLoaded()) {
                val fusedMps = NativeBridge.getFusedSpeedMps()
                if (fusedMps >= 0f) {
                    // EMA low-pass for a stable speedometer. τ ≈ 0.7 s; at the UI
                    // tick cadence α = dt/(τ+dt). Cited, not magic: 0.7 s trades a
                    // little display lag for jitter rejection on a moving vehicle
                    // (Lever D may swap this for a median window later).
                    val dtS = (nowMs - lastSpeedTimeMs).coerceAtLeast(1L).toDouble() / 1000.0
                    val tauS = 0.7
                    val alpha = (dtS / (tauS + dtS)).toFloat()
                    speedEmaMps += alpha * (fusedMps - speedEmaMps)
                    currentSpeedKmh = speedEmaMps * 3.6f
                } else {
                    speedEmaMps = 0f
                    currentSpeedKmh = 0f
                }
                lastSpeedTimeMs = nowMs

                // 2026-05-28 — every 10 s, persist the calibrated MiDaS scale K
                // to SharedPreferences so the next cold start inherits it. Only
                // writes positive values (native returns -1 before first calib).
                if (nowMs - lastMidasKPersistMs >= MIDAS_K_PERSIST_INTERVAL_MS) {
                    lastMidasKPersistMs = nowMs
                    val k = NativeBridge.getMidasScaleK()
                    if (k > 0.0) {
                        prefs.edit().putFloat(PREF_MIDAS_SCALE_K, k.toFloat()).apply()
                    }
                }
            }
            /* SUPERSEDED 2026-05-26 — position-differencing speed. Computed
               |Δpos|/Δt from vio.x/z (= global_t_), whose scale falls back to the
               pedestrian stride model → wrong for non-walking motion, and was
               capped by the old 5 m/s (18 km/h) velocity clamp. Replaced by the
               fused |v_G_| getter above (locomotion-agnostic, no stride model).
            val prev = lastVioForSpeed
            if (prev != null) {
                val dtMs = nowMs - lastSpeedTimeMs
                if (dtMs >= 200) {
                    val dx = vio.x - prev.x
                    val dz = vio.z - prev.z
                    val distM = sqrt(dx * dx + dz * dz)
                    currentSpeedKmh = (distM / (dtMs / 1000.0) * 3.6).toFloat()
                    lastVioForSpeed = vio
                    lastSpeedTimeMs = nowMs
                }
            } else {
                lastVioForSpeed = vio
                lastSpeedTimeMs = nowMs
            }
            */

            // The ManeuverStateMachine is STATEFUL (per-tick heading sweep + tick-based U-turn
            // window), so it MUST tick every ~500 ms on the (Main, serial) collector — even
            // while a /match HTTP call is still in flight. Only the network snap is gated by
            // snapInFlight. The timer is consumed only once we actually have an anchor.
            if (nowMs - lastSnapTimeMs > 500) {
                val start = startLocation
                if (start != null) {
                    lastSnapTimeMs = nowMs
                    val currentLatLng = NavSightUtils.metersToLatLng(start, vio.x, vio.z)
                    // Dynamic roads: ensure the region around the user's CURRENT position
                    // is loaded (re-fetches when they move into a new tile). Non-blocking.
                    RoadRegionManager.ensureRegion(currentLatLng.latitude, currentLatLng.longitude)
                    val region = RoadRegionManager.regionFor(currentLatLng.latitude, currentLatLng.longitude)
                    // Heading-aware matcher (§0.7): detect roundabout traversal/exit + U-turns
                    // from the VIO heading (radians → degrees; MapScreenUi convention).
                    val headingDeg = Math.toDegrees(vio.heading)
                    val maneuver = maneuverStateMachine.tick(
                        currentLatLng.latitude, currentLatLng.longitude, headingDeg, region?.roundabouts
                    )
                    maneuver.event?.let { ev ->
                        Log.i("ManeuverSM", "MANEUVER event=$ev state=${maneuver.state} " +
                            "sweep=${"%.0f".format(maneuver.accumulatedSweepDeg)} " +
                            "exitBearing=${maneuver.suggestedExit?.bearingDeg?.let { "%.0f".format(it) } ?: "-"} " +
                            "dir=${maneuver.travelDirection}")
                    }
                    maneuverState = maneuver.state  // Compose state; collector is on Main
                    // Gate ONLY the network snap so /match coroutines never stack; the SM has
                    // already advanced this cycle regardless.
                    if (!snapInFlight) launchNetworkSnap(start, currentLatLng, region, maneuver, nowMs)
                }
            }
        }
    }

    /**
     * Launch the gated network snap: OSRM /match on a subsampled trajectory window, with the
     * per-point [roadSnapper] as the offline/low-confidence fallback. Holds [snapInFlight] for
     * its lifetime (cleared in finally). [maneuver] is the latest SM result (ring-pin for the
     * fallback); [nowMs] stamps the read-only K̂_map observation.
     */
    private fun launchNetworkSnap(start: LatLng, currentLatLng: LatLng, region: DynamicRoadRegion?, maneuver: ManeuverResult, nowMs: Long) {
        val recentPath = pathHistory.takeLast(10).map { p ->
            NavSightUtils.metersToLatLng(start, p.x.toDouble(), p.z.toDouble())
        }
        // Step D — primary: the LOCAL Viterbi matcher on a subsampled trajectory window (last
        // ~150 frames ≈ 6 s of path). It map-matches the PATH (not the point) to the road, so
        // the dot follows curves/roundabouts and does not "let go" the way per-point snap does.
        // ~15 points (the on-device matcher has no coordinate cap — that was an OSRM-URL limit).
        val recent = pathHistory.takeLast(150)
        val stride = maxOf(1, recent.size / 14)
        val matchWindow = recent.filterIndexed { i, _ -> i % stride == 0 }.takeLast(15)
            .map { p -> NavSightUtils.metersToLatLng(start, p.x.toDouble(), p.z.toDouble()) } + currentLatLng
        snapInFlight = true
        viewModelScope.launch(Dispatchers.IO) {
            try {
                // Primary: on-device Viterbi map-match (geometry + confidence). No network.
                val matched = localMatcher.match(matchWindow, region)
                // #4 — confidence gating + break detection: only trust /match when it is
                // confident and the trace was not split. Otherwise fall back to per-point snap.
                val accept = matched != null &&
                    matched.confidence >= MIN_MATCH_CONFIDENCE && !matched.broken
                val snapped: SnappedLatLng
                val matchedPath: List<LatLng>
                if (accept && matched != null) {
                    snapped = SnappedLatLng(matched.matched.latitude, matched.matched.longitude, "local:match", null, true)
                    matchedPath = matched.matchedPath        // #3 — on-road geometry
                } else {
                    snapped = roadSnapper.snapToRoad(currentLatLng, recentPath, maneuver)
                    // #10 — keep the last matched road on a TRANSIENT /match miss (no green-line
                    // flicker); only drop it when we are truly off any road.
                    matchedPath = if (snapped.isSnapped) matchedRoadPath else emptyList()
                }
                val srcLabel = if (accept) "local:match" else if (snapped.isSnapped) "snap" else "raw"
                if (srcLabel != lastSnapSource) {
                    lastSnapSource = srcLabel
                    Log.i("LiveMatcher", "SNAP_SOURCE source=$srcLabel " +
                        "conf=${matched?.confidence?.let { "%.2f".format(it) } ?: "-"} " +
                        "broken=${matched?.broken ?: "-"} windowPts=${matchWindow.size}")
                }
                // #2 — READ-ONLY scale observation K̂_map (logged + recorded only; NOT fed to scale).
                if (accept && matched != null) recordScaleObservation(matched, matchWindow, nowMs)
                else lastKMap = null
                // §0.8 — record the map-matched position only when snapped to a road.
                if (snapped.isSnapped) {
                    lastMapMatched = snapped.toLatLng()
                    lastMapMatchedConf = if (accept && matched != null) matched.confidence else {
                        val rawSL = SnappedLatLng(currentLatLng.latitude, currentLatLng.longitude, null, null, false)
                        exp(-snapped.distanceTo(rawSL) / 5.0)   // σ=5 m (RoadSnapper.SNAP_SIGMA_M)
                    }
                } else {
                    lastMapMatched = null
                    lastMapMatchedConf = null
                }
                // #8 — when navigating, project onto the planned route (route-pinned) + detect
                // off-route → reroute. Returns the display position (pinned within the corridor).
                val displayPos = if (navigationState is NavigationState.Active) {
                    navigationManager.updateVioPosition(snapped.toLatLng())
                } else {
                    snapped.toLatLng()
                }
                withContext(Dispatchers.Main) {
                    snappedPosition = displayPos
                    matchedRoadPath = matchedPath
                }
            } finally {
                snapInFlight = false
            }
        }
    }

    /**
     * Tier-1 #2 — READ-ONLY map-derived scale observation. d_route = the matched road
     * length over the window (treated as truth); d_vio = the raw VIO path length over the
     * SAME (subsampled) window; K̂_map = d_route / d_vio is how much our VIO under/over-reads
     * distance. Two outputs: (a) STASH the raw per-sample value in [lastKMap] for the sim
     * recording (recorded un-gated/un-throttled; the analyzer filters it with mm_conf);
     * (b) LOG a hard-gated, throttled MAP_SCALE_OBS line for live debugging. READ-ONLY —
     * deliberately NOT applied to the dot's scale/speed: that path is protected (morad
     * branch, gated on Fix A/B). The subsampled d_vio slightly under-reads on tight curves.
     */
    private fun recordScaleObservation(matched: LiveMatcher.MatchResult, window: List<LatLng>, nowMs: Long) {
        if (matched.routeDistanceM <= 0.0) { lastKMap = null; return }
        var dVio = 0.0
        for (i in 1 until window.size) {
            dVio += RoadSnapMath.haversineM(
                window[i - 1].latitude, window[i - 1].longitude,
                window[i].latitude, window[i].longitude
            )
        }
        if (dVio < 1.0) { lastKMap = null; return }   // avoid div-by-~0 on a stationary window
        val kMap = matched.routeDistanceM / dVio
        lastKMap = kMap   // (a) raw per-sample → recording

        // (b) hard-gated + throttled live log
        if (matched.confidence < SCALE_OBS_MIN_CONFIDENCE) return
        if (dVio < SCALE_OBS_MIN_VIO_M) return
        if (kMap < SCALE_OBS_MIN_RATIO || kMap > SCALE_OBS_MAX_RATIO) return
        if (nowMs - lastScaleObsMs < SCALE_OBS_LOG_INTERVAL_MS) return
        lastScaleObsMs = nowMs
        Log.i("LiveMatcher", "MAP_SCALE_OBS k_map=%.3f d_route=%.1f d_vio=%.1f conf=%.2f".format(
            kMap, matched.routeDistanceM, dVio, matched.confidence))
    }

    /**
     * User override (long-press on the map): relocate the start/origin pin to [latLng].
     * The VIO origin (vio 0,0) is re-anchored here, so the dot + path + matcher all
     * reproject from this point (NavSightUtils.metersToLatLng uses startLocation as the
     * origin). Pre-loads the road region around it. The C++ SessionAnchor (vlat/vlng in
     * recordings) is intentionally left on the original GPS fix — this is a display/match
     * anchor override, not a re-anchor of the EKF.
     */
    fun overrideStartLocation(latLng: LatLng) {
        startLocationOverridden = true
        startLocation = latLng
        RoadRegionManager.ensureRegion(latLng.latitude, latLng.longitude)
        Log.i("NavSightVM", "START_OVERRIDE lat=${latLng.latitude} lng=${latLng.longitude}")
    }

    fun toggleSimulationRecording(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        if (!isRecordingSimulation) {
            // Use same monitor as saveSimulationData for consistency (Finding 11 fix).
            synchronized(simulationDataPoints) { simulationDataPoints.clear() }
            simFrameStats = null
            // Zero the native EventCounters so this recording's
            // event_summary reflects only what happened during the walk.
            // The native singleton survives process lifetime, so leftover
            // counts from a prior recording would otherwise carry over.
            runCatching { NativeBridge.nativeResetEventCounters() }
            sensorRepository.startGpsUpdates(hasLocationPermission)
            // Step 9 / ADR-014 — start the camera-frame recorder. Output
            // directory is named for the same wall-clock millisecond the
            // simulation_data_<ts>.json file will use, so a fixture pair
            // is trivially co-located.
            simFrameStartTimeMs = System.currentTimeMillis()
            val dir = getExternalFilesDir(null) ?: filesDir
            val framesDir = java.io.File(dir, "simulation_data_${simFrameStartTimeMs}.frames")
            val recorder = SimulationFrameRecorder(framesDir)
            recorder.start()
            simFrameRecorder = recorder
            sensorRepository.setFrameRecorder(recorder)
            isRecordingSimulation = true
        } else {
            isRecordingSimulation = false
            // Detach the recorder from the camera analyzer first so no new
            // frames are submitted, THEN stop (which drains in-flight encodes).
            sensorRepository.setFrameRecorder(null)
            simFrameStats = simFrameRecorder?.stop()
            simFrameRecorder = null
            sensorRepository.stopGpsUpdates()
            saveSimulationData(getExternalFilesDir, filesDir)
        }
    }

    private fun saveSimulationData(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        if (simulationDataPoints.isEmpty()) return
        val snapshot = synchronized(simulationDataPoints) { simulationDataPoints.toList() }
        // Snapshot the native EventCounters now (recording-stop time) so the
        // embedded event_summary covers exactly the same window as the
        // points array. The native call is lock-free; failure (library not
        // loaded) falls back to an empty object so the rest of the JSON is
        // still well-formed.
        val eventSummaryJson: String = runCatching {
            NativeBridge.nativeGetEventCountersJson()
        }.getOrNull()?.takeIf { it.isNotBlank() } ?: "{}"
        val frameStats = simFrameStats
        val frameStartMs = simFrameStartTimeMs
        viewModelScope.launch(Dispatchers.IO) {
            val startTime = snapshot.firstOrNull()?.timestamp ?: System.currentTimeMillis()
            val sb = StringBuilder()
            sb.append("{\"startTime\":$startTime,\"event_summary\":")
            sb.append(eventSummaryJson)
            // Step 9 / ADR-014 — frames_meta block. Present iff a frame
            // recorder was active and wrote ≥ 1 PNG. The replay harness
            // ignores unknown keys, so older harnesses that don't read
            // frames_meta still load this JSON unchanged.
            if (frameStats != null && frameStats.written > 0) {
                sb.append(",\"frames_meta\":{")
                sb.append("\"dir\":\"simulation_data_${frameStartMs}.frames\",")
                sb.append("\"written\":${frameStats.written},")
                sb.append("\"dropped\":${frameStats.dropped},")
                sb.append("\"first_ts_ns\":${frameStats.firstFrameNs},")
                sb.append("\"last_ts_ns\":${frameStats.lastFrameNs}")
                sb.append("}")
            }
            sb.append(",\"points\":[")
            snapshot.forEachIndexed { index, p ->
                if (index > 0) sb.append(",")
                sb.append("{\"ts\":${p.timestamp},")
                sb.append("\"vx\":${p.vioX},\"vy\":${p.vioY},\"vz\":${p.vioZ},")
                sb.append("\"vyaw\":${p.vioYaw},\"vsc\":${p.vioScale},\"vql\":${p.vioQuality},")
                sb.append("\"rx\":${p.rawX},\"ry\":${p.rawY},\"rz\":${p.rawZ},\"ryaw\":${p.rawYaw},")
                sb.append("\"ax\":${p.accelX},\"ay\":${p.accelY},\"az\":${p.accelZ},")
                sb.append("\"gx\":${p.gyroX},\"gy\":${p.gyroY},\"gz\":${p.gyroZ},")
                sb.append("\"glat\":${p.gpsLat ?: "null"},\"glng\":${p.gpsLng ?: "null"},")
                sb.append("\"galt\":${p.gpsAlt ?: "null"},\"gacc\":${p.gpsAcc ?: "null"},")
                // Step B* / §0.8 — VIO-projected (lat,lng) + map-matched track.
                sb.append("\"vlat\":${p.vioLat ?: "null"},\"vlng\":${p.vioLng ?: "null"},\"vvar\":${p.vioVarXy ?: "null"},")
                sb.append("\"mm_lat\":${p.mmLat ?: "null"},\"mm_lng\":${p.mmLng ?: "null"},\"mm_conf\":${p.mmConf ?: "null"},")
                // Matcher diagnostics (logcat-equivalent, persisted): source, K̂_map, maneuver state.
                sb.append("\"mm_src\":${p.mmSrc?.let { "\"$it\"" } ?: "null"},\"k_map\":${p.kMap ?: "null"},")
                sb.append("\"maneuver\":${p.maneuver?.let { "\"$it\"" } ?: "null"},")
                sb.append("\"mflow\":${p.meanFlow},\"inl\":${p.inlierCount},")
                sb.append("\"steps\":${p.stepCount},\"sfreq\":${p.stepFreq},")
                sb.append("\"stride\":${p.strideLength},\"pflags\":${p.poseFlags},")
                sb.append("\"hdg\":${p.heading}}")
            }
            sb.append("]}")
            try {
                val dir = getExternalFilesDir(null) ?: filesDir
                // Use the same wall-clock millisecond the recorder used for
                // its frames-dir so the JSON and the frames sit beside each
                // other under matching basenames. Falls back to the now-time
                // when no recorder was active (Step 9 frames-disabled path).
                val basenameMs =
                    if (frameStartMs > 0L) frameStartMs else System.currentTimeMillis()
                val file = java.io.File(dir, "simulation_data_${basenameMs}.json")
                file.writeText(sb.toString())
            } catch (e: Exception) {
                Log.e("SIMULATION", "Failed to save: ${e.message}")
            }
        }
    }

    fun onResume() {
        sensorRepositoryActive = true   // ← mark BEFORE starting sensors
        sensorRepository.startSensors()
        // 2026-05-28 — re-seed the MiDaS scale K AFTER startSensors(). startSensors
        // calls NativeBridge.startVIO() which destroys and recreates the VioEngine
        // (native-lib.cpp:199-202), wiping the in-native midas_scale_K_ atomic
        // back to its -1.0 init value. Without this re-push, switching apps and
        // coming back would force a fresh K calibration on the next recording —
        // exactly the regression we saw between today's run (K=1499) and the
        // subsequent walk+run (K=0). Doing it here in onResume covers every
        // lifecycle path: app open, return-from-background, configuration change.
        if (NativeBridge.isLoaded()) {
            val persistedK = prefs.getFloat(PREF_MIDAS_SCALE_K, -1f).toDouble()
            if (persistedK > 0.0) {
                NativeBridge.setMidasScaleK(persistedK)
            }
        }
    }

    fun onPause() {
        sensorRepositoryActive = false  // ← mark BEFORE stopping sensors so in-flight frames are dropped
        // 2026-05-28 — persist the latest K BEFORE stopSensors. stopSensors does
        // not directly recreate the Tracker, but the next onResume->startVIO
        // will. If we let onResume's re-seed run with a stale prefs value
        // (from before this session's calibration), all of this session's
        // calibration is lost. Save what's in native NOW so the next onResume
        // pushes the latest value back.
        if (NativeBridge.isLoaded()) {
            val k = NativeBridge.getMidasScaleK()
            if (k > 0.0) {
                prefs.edit().putFloat(PREF_MIDAS_SCALE_K, k.toFloat()).apply()
            }
        }
        if (isRecordingSimulation) sensorRepository.stopGpsUpdates()
        sensorRepository.stopSensors()
    }

    fun processCameraFrame(image: ImageProxy) {
        sensorRepository.processCameraFrame(image)
    }

    // Step 8c: relay rolling-shutter skew from CameraUi Camera2Interop callback
    // into SensorRepository where the JNI call reads it per frame.
    fun updateRollingShutterSkew(skewNs: Long) {
        sensorRepository.updateRollingShutterSkew(skewNs)
    }

    fun requestInitialLocation(granted: Boolean = false) {
        hasLocationPermission = granted
        sensorRepository.requestInitialLocation(granted)
    }

    fun resetPath() {
        sensorRepository.resetPath()
        _pathHistory.clear()
        pathHistoryVersion = _pathHistoryVersion.incrementAndGet()
        virtualX = 0.0
        virtualZ = 0.0
        currentSpeedKmh = 0f
        totalDistanceM = 0.0
        lastVioForSpeed = null
        speedEmaMps = 0f
        lastVioForDist = null
    }

    /**
     * Stage 3 (2026-05-09) — full reset that mirrors a fresh app launch.
     * Wired to the "Rides" button in BottomSheetUi.
     *
     * Performs the heavy work on Dispatchers.IO because the call chain
     * blocks for up to several seconds (NativeBridge.resetVIO() joins
     * the BA + loop-closure worker threads; SimulationFrameRecorder.stop()
     * awaits encoder thread termination; sensor unregister is synchronous).
     * Doing all of that on the UI thread caused an ANR. The UI-visible
     * pieces (state flags, path history) flip immediately on the main
     * thread so the dot/path clear instantly; the slow work happens
     * behind that.
     */
    fun resetAll() {
        // ── Main-thread immediate UI clear ─────────────────────────────
        // Clear visible state synchronously so the user sees the dot/path
        // disappear right away. The rest happens off-thread below.
        _pathHistory.clear()
        pathHistoryVersion = _pathHistoryVersion.incrementAndGet()
        virtualX = 0.0
        virtualZ = 0.0
        currentSpeedKmh = 0f
        totalDistanceM = 0.0
        positionSigmaM = Float.NaN
        positionCovValid = false
        lastVioForSpeed = null
        speedEmaMps = 0f
        // Heading-aware matcher: drop any in-progress roundabout/U-turn state.
        maneuverStateMachine.reset()
        maneuverState = ManeuverState.FREE_ROAD
        lastSnapSource = ""
        snapInFlight = false
        matchedRoadPath = emptyList()
        lastScaleObsMs = 0L
        lastKMap = null
        startLocationOverridden = false   // a fresh session re-acquires the GPS anchor
        lastMapMatched = null
        lastMapMatchedConf = null
        lastVioForDist = null
        snappedPosition = null
        navigationStartMessage = null
        // Camera overlay Phase 2/3/4: clear the bundled snapshot so KLT
        // dots / SLAM dots / loop-closure flash all disappear instantly
        // on reset. SensorRepository will republish on the next frame
        // after startSensors().
        overlaySnapshot = SensorRepository.OverlaySnapshot(
            floatArrayOf(), intArrayOf(), floatArrayOf(), floatArrayOf(), 0L
        )
        loopClosureFlashUntilMs = 0L
        lastLoopClosureCount = 0L
        runCatching { navigationManager.cancelNavigation() }
        if (scaleCalibrationSession != null) {
            scaleCalibrationSession = null
            scaleCalibrationMessage = "Scale calibration cancelled by reset."
        }
        // Mark recording stopped IMMEDIATELY so any in-flight frame
        // callbacks on the camera thread see isRecordingSimulation=false
        // and skip captureFrame. The recorder itself is drained off-thread.
        val recorderToStop = if (isRecordingSimulation) {
            sensorRepository.setFrameRecorder(null)
            isRecordingSimulation = false
            // Audit Finding 11 (2026-05-16): simulationDataPoints.clear() was
            // called on the main thread without synchronization. saveSimulationData
            // on the IO thread uses synchronized(simulationDataPoints) { toList() }.
            // Two threads mutating the same ArrayList concurrently → ConcurrentModificationException.
            // Fix: use the same monitor object as the read site.
            synchronized(simulationDataPoints) { simulationDataPoints.clear() }
            Log.i("NavSightViewModel", "resetAll: simulationDataPoints cleared under lock (Finding 11 fix)")
            simFrameRecorder.also {
                simFrameRecorder = null
                simFrameStats = null
            }
        } else null

        // ── Off-thread heavy work ─────────────────────────────────────
        // stopSensors → resetVIO (joins worker threads) → startSensors
        // → GPS re-acquire. ~1-3 s on this device; must not block UI.
        viewModelScope.launch(Dispatchers.IO) {
            // Drain the recorder's encoder thread off-UI.
            recorderToStop?.stop()
            sensorRepository.resetAll(hasLocationPermission)
        }
    }

    fun startNavigation(destination: LatLng) {
        viewModelScope.launch {
            val currentPos = NavSightUtils.resolveNavigationStart(snappedPosition, startLocation)
            if (currentPos == null) {
                navigationStartMessage = "Current location not ready yet."
                return@launch
            }
            navigationStartMessage = null
            navigationManager.startNavigation(currentPos, destination)
        }
    }

    fun clearNavigationStartMessage() { navigationStartMessage = null }

    /** Step 5: invoked by the "Place phone flat for 5 s" dialog OK button. */
    fun clearInitTimeout() { sensorRepository.clearInitTimeout() }

    fun startScaleCalibration(targetDistanceMeters: Double) {
        val current = latestVioState
        if (!current.isInitialized) { scaleCalibrationMessage = "Tracking is not ready yet."; return }
        scaleCalibrationSession = ScaleCalibrationSession(
            legDistanceMeters = targetDistanceMeters,
            startX = current.x, startZ = current.z,
            lastX = current.x, lastZ = current.z,
            pathLengthMeters = 0.0, sampleCount = 0,
            maxDistanceFromStartMeters = 0.0,
            sumQuality = current.trackingQuality,
            lowQualityFrames = if (current.trackingQuality < 0.2) 1 else 0
        )
        scaleCalibrationMessage = "Walk ${targetDistanceMeters.toInt()} m out, return to start, then tap Finish."
    }

    fun finishScaleCalibration() {
        val session = scaleCalibrationSession
        val current = latestVioState
        if (session == null || !current.isInitialized) {
            scaleCalibrationMessage = "Calibration session is not ready to finish."; return
        }
        val dx = current.x - session.startX
        val dz = current.z - session.startZ
        val closureErrorMeters = sqrt(dx * dx + dz * dz)
        val pathLengthMeters = session.pathLengthMeters
        val roundTripTargetMeters = session.legDistanceMeters * 2.0
        val avgQuality = session.sumQuality / max(1, session.sampleCount)
        val lowQualityRatio = session.lowQualityFrames.toDouble() / max(1, session.sampleCount)
        scaleCalibrationSession = null
        when {
            session.sampleCount < 10 -> { scaleCalibrationMessage = "Too short. Please try again."; return }
            session.maxDistanceFromStartMeters < session.legDistanceMeters * 0.7 -> { scaleCalibrationMessage = "Did not walk far enough."; return }
            avgQuality < 0.35 || lowQualityRatio > 0.35 -> { scaleCalibrationMessage = "Quality too low. Avg ${"%.2f".format(avgQuality)}."; return }
            pathLengthMeters < roundTripTargetMeters * 0.6 -> { scaleCalibrationMessage = "Round trip too short. Retry."; return }
            closureErrorMeters > max(2.0, session.legDistanceMeters * 0.4) -> { scaleCalibrationMessage = "Return drift too large (${"%.2f".format(closureErrorMeters)} m)."; return }
        }
        val newFactor = NavSightUtils.computeUpdatedScaleCalibrationFactor(
            currentFactor = scaleCalibrationFactor,
            knownDistanceMeters = roundTripTargetMeters,
            measuredDistanceMeters = pathLengthMeters
        ) ?: run { scaleCalibrationMessage = "Factor could not be computed."; return }
        saveScaleCalibrationFactor(newFactor)
        scaleCalibrationMessage = "Saved ${"%.2f".format(newFactor)}x. Measured ${"%.1f".format(pathLengthMeters)} m, closure ${"%.2f".format(closureErrorMeters)} m."
    }

    fun cancelScaleCalibration() { scaleCalibrationSession = null; scaleCalibrationMessage = "Calibration cancelled." }
    fun clearScaleCalibrationMessage() { scaleCalibrationMessage = null }
    fun resetScaleCalibration() { scaleCalibrationSession = null; saveScaleCalibrationFactor(1.0); scaleCalibrationMessage = "Scale reset to 1.00x." }
    fun stopNavigation() { navigationManager.cancelNavigation() }

    fun exportPath(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        val path = pathHistory.toList()
        if (path.isEmpty()) return
        val start = startLocation
        val sb = StringBuilder()
        sb.append("{\"type\":\"navsight_path\",\"points\":[")
        path.forEachIndexed { idx, p ->
            if (idx > 0) sb.append(",")
            val sigmaJson = if (p.sigmaM.isNaN()) "null" else p.sigmaM.toString()
            if (start != null) {
                val latLng = NavSightUtils.metersToLatLng(start, p.x.toDouble(), p.z.toDouble())
                sb.append("{\"lat\":${latLng.latitude},\"lng\":${latLng.longitude},\"x\":${p.x},\"z\":${p.z},\"sigma_m\":$sigmaJson}")
            } else sb.append("{\"x\":${p.x},\"z\":${p.z},\"sigma_m\":$sigmaJson}")
        }
        sb.append("]}")
        try {
            val dir = getExternalFilesDir(null) ?: filesDir
            val file = java.io.File(dir, "navsight_path_${System.currentTimeMillis()}.json")
            file.writeText(sb.toString())
        } catch (e: Exception) { Log.e("NavSight", "Export failed: ${e.message}") }
    }

    override fun onCleared() { super.onCleared(); roadSnapper.shutdown() }

    private fun saveScaleCalibrationFactor(factor: Double) {
        scaleCalibrationFactor = factor
        prefs.edit().putFloat(PREF_SCALE_CALIBRATION_FACTOR, factor.toFloat()).apply()
        NativeBridge.setScale(factor)
    }

    /**
     * Step 6 (Task #31): compact JSON snapshot of the latest VIO state plus
     * sigma. Embedded into [CrashLogger]'s crash files. Kept allocation-light
     * because it is called per UI tick on the main thread.
     */
    private fun buildCrashSnapshotJson(vio: VioData, sigmaM: Float, covValid: Boolean): String {
        val sigmaJson = if (sigmaM.isFinite()) sigmaM.toString() else "null"
        val gps = currentGpsLocation
        val gpsJson = if (gps != null) {
            "{\"lat\":${gps.latitude},\"lng\":${gps.longitude},\"alt\":${gps.altitude},\"acc\":${gps.accuracy}}"
        } else "null"
        return "{" +
            "\"now_ms\":${System.currentTimeMillis()}," +
            "\"vio_initialized\":${vio.isInitialized}," +
            "\"vio_x\":${vio.x},\"vio_y\":${vio.y},\"vio_z\":${vio.z}," +
            "\"vio_yaw\":${vio.yaw},\"vio_heading\":${vio.heading}," +
            "\"vio_quality\":${vio.trackingQuality}," +
            "\"vio_scale\":${vio.estimatedScale}," +
            "\"vio_features\":${vio.trackedFeatures}," +
            "\"vio_inliers\":${vio.inlierCount}," +
            "\"vio_mean_flow\":${vio.meanFlow}," +
            "\"sigma_m\":$sigmaJson," +
            "\"cov_valid\":$covValid," +
            "\"path_len_m\":$totalDistanceM," +
            "\"speed_kmh\":$currentSpeedKmh," +
            "\"init_status\":\"${initStatus.name}\"," +
            "\"gps\":$gpsJson" +
            "}"
    }
}