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
        // 2026-05-31 advance-along-rail — cap a single tick's graph advance to a plausible
        // speed (54 km/h) so a VIO teleport-spike can't race the ball down the road.
        private const val RAIL_MAX_SPEED_MPS = 15.0
        // map→VIO POSITION leg gates (review HIGH — wrong-road runaway). Push only when the matcher is
        // high-confidence (≥0.6 = the rail-lock bar) and has stayed so for MAP_POS_MIN_STREAK ticks.
        private const val MAP_POS_MIN_CONFIDENCE = 0.6
        private const val MAP_POS_MIN_STREAK = 3
        // Matcher-supervised wrong-turn recovery: re-acquire the ball onto the matcher's road only after
        // it has stayed >25 m from the confident matcher dot for 5 consecutive ticks (~2.5 s) — long
        // enough that a transient parallel-road matcher flicker can't cause a spurious teleport.
        private const val RAIL_MATCHER_REACQUIRE_M = 25.0
        private const val RAIL_MATCHER_REACQUIRE_TICKS = 5
        // DEFAULT-OFF: matcher-supervised wrong-turn recovery. OFFLINE on sim A (a heading-disaster ride
        // where the matcher is itself unreliable) it REGRESSES the dot — 968 m/1.33×/3 teleports vs pure
        // dead-reckoning's 599 m/0.82×/0. Its benefit needs a device re-record where the (on) heading leg
        // makes the matcher trustworthy. Enable together with the position leg after that validation.
        private const val MATCHER_SUPERVISION_ENABLED = false
        private const val RAIL_TRAIL_MAX = 600   // ~5 min of on-road breadcrumb at the 0.5 s snap tick
        // EMA rate for the gyro↔road heading offset. Slow (0.1) so the offset is a stable calibration of
        // the gyro's absolute error and brief junction transients don't shift it.
        private const val HEADING_OFFSET_ALPHA = 0.1
        // Sustained ticks of "heading clearly opposite the ball's facing" before flipping (a real U-turn /
        // wrong facing). ~1.5 s at the 0.5 s snap tick — long enough that noise never triggers a flip.
        private const val RAIL_REVERSE_TICKS = 3
        // INSTRUMENTATION-ONLY noise floor for the RAIL_JUNCTION log: a junction switch is a discrete
        // graph-edge tangent JUMP (many degrees); curvature-following along one road is sub-degree per
        // tick. This separates "took a junction" from "followed the curve" for the log; it is NOT a
        // steering threshold (steering is the anchor model) so it changes no behaviour.
        private const val RAIL_JUNCTION_LOG_MIN_TURN_DEG = 1.0
        // Scooter camera-to-road mount height (m) for the read-only ground-plane estimator. Default guess;
        // a wrong value scales the recovered speed linearly. Auto-calibrated from GPS once per mount (below).
        private const val SCOOTER_CAMERA_HEIGHT_M = 1.05
        // Persisted GPS-calibrated mount height. Key PRESENCE = "already calibrated" (don't re-calibrate).
        private const val PREF_CAMERA_HEIGHT = "camera_height_m"
        // Bumped whenever the IPM geometry changes — a height auto-calibrated against an OLD (e.g. under-
        // reading) IPM is STALE and must be discarded so it re-calibrates against the corrected estimator.
        // v2 = the 2026-06-02 geometric road-point fix (a prior calib like 1.70 = the under-read compensation).
        // v3 = the 2026-06-12 IPM front-end fix (FB check + road-mask top-up + σ-floor taxonomy): any
        //      height calibrated against the pre-fix UNDER-reading IPM is inflated (ratio = gps/ipm) and
        //      must be discarded so it re-solves against the corrected estimator.
        private const val PREF_CAMERA_HEIGHT_VERSION = "camera_height_calib_version"
        private const val CAMERA_HEIGHT_CALIB_VERSION = 3
        // Samples of a clean GPS+IPM pair before committing the height (~5 s at the ~5 Hz UI tick).
        private const val HEIGHT_CALIB_SAMPLES = 25
        // Min GPS accuracy (m) and min speed (m/s) for a calibration sample, and the plausible-correction
        // band on a single ratio (the final height is hard-clamped to 0.3–2.5 m regardless).
        private const val HEIGHT_CALIB_MAX_GPS_ACC_M = 12f
        private const val HEIGHT_CALIB_MIN_MPS = 3.0f
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
    // 2026-06-12ui — calibration sheet visibility (opened by the header VIO chip; the UI
    // both opens and dismisses it, so the setter is public).
    var showCalibrationSheet by mutableStateOf(false)
    // 2026-06-12ui — read-only matcher readouts for the debug panel (set on the IO snap tick;
    // Compose snapshot state is thread-safe to write off-main).
    var mmSrcDebug by mutableStateOf("--")
        private set
    var mmConfDebug by mutableStateOf(-1f)
        private set

    /** 2026-06-12b power-trim — camera-screen overlay polling on/off (see SensorRepository). */
    fun setOverlayPolling(enabled: Boolean) { sensorRepository.overlayPollingEnabled = enabled }

    // 2026-06-02 — READ-ONLY live readout of the IPM ground-plane speed (km/h) for on-device
    // eyeballing vs the speedometer (the candidate scooter speed-fix). null = not firing yet
    // (the IPM needs ≥5 road-pixel inliers). It NEVER drives the dot/speedometer — display only.
    var groundFlowSpeedKmh by mutableStateOf<Float?>(null)
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

    // 2026-06-02 — GPS-auto-calibrated camera mount height (m). The IPM speed is LINEAR in this height,
    // so one trusted GPS speed solves it: h_true = h × (gps/ipm). Loaded from prefs (key presence =
    // already calibrated → don't redo); default = the scooter guess. GPS is used ONLY for this physical
    // constant — the dot/position stays pure VIO. Exposed read-only for the camera HUD.
    private var cameraHeightM = prefs.getFloat(PREF_CAMERA_HEIGHT, SCOOTER_CAMERA_HEIGHT_M.toFloat()).toDouble()
    private var heightCalibrated = prefs.contains(PREF_CAMERA_HEIGHT)
    private val heightCalibRatios = mutableListOf<Double>()
    // 2026-06-12 — previous GPS fix the height calibrator accepted (jam cross-check + per-fix dedup).
    private var lastHeightCalibFix: Location? = null
    val mountHeightM: Double get() = cameraHeightM
    var heightCalibStatus by mutableStateOf<String?>(null)
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
        val mmSrc: String?, val kMap: Double?, val maneuver: String?,
        // 2026-06-02 — READ-ONLY speed channels logged for offline scoring vs GPS (the IPM
        // ground-plane speed-fix validation). gpFlow = Tracker::updateGroundFlowSpeed (the
        // candidate); fused = the currently-DISPLAYED speed (getFusedSpeedMps). Both m/s,
        // null when the native estimate is the -1.0 "not yet" sentinel. Compare both to the
        // glat/glng-derived GPS speed to decide whether the IPM corrects the under-read.
        val gpFlowSpeed: Double?, val fusedSpeed: Double?,
        // 2026-06-04 — per-DEPTH-BAND IPM diagnostic (near/mid/far × n, flow_px, vi_kmh, cos_fa, survived =
        // 15 floats), persisted in the sim JSON because logcat rolls off before a long ride returns. Lets
        // offline analysis tell DROPPED vs UNDER-MEASURED vs GATED vs DILUTED. null when the IPM hasn't run.
        val ipmBand: FloatArray? = null
    )
    // ──────────────────────────────────────────────────────────────────────────

    val vioInitAzimuth: Float get() = sensorRepository.vioInitAzimuth

    private var lastVioForSpeed: VioData? = null
    // 2026-05-26 — EMA state for the locomotion-agnostic fused-speed display. Smooths
    // NativeBridge.getFusedSpeedMps() (|v_G_|) into a stable speedometer reading. See
    // the speed block in the VIO update loop; reset in resetPath()/resetAll().
    private var speedEmaMps = 0f
    private var lastVioForDist: VioData? = null
    // 2026-06-03 — the displayed in-app distance follows the DOT (snapped on-road position), not the raw
    // VIO. These track the last displayed-dot position + time for the gated (teleport-excluding) accumulation.
    private var lastDotForDist: LatLng? = null
    private var lastDotDistMs: Long = 0L
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
    // 2026-05-31 advance-along-rail — the displayed dot is a BALL constrained to the routing graph
    // (DynamicRoadRegion.graph): it advances ALONG its current road by the distance travelled and can
    // only cross to a CONNECTED edge at a junction, so it never teleports to a parallel road ("ball in
    // a maze", owner). Rebuilt when the region changes. lastSnapVio carries the previous tick's VIO
    // position so we can derive the per-tick distance + travel bearing the ball advances by.
    @Volatile private var graphRail: GraphRailDot? = null
    @Volatile private var graphRailRegion: DynamicRoadRegion? = null
    @Volatile private var lastSnapVio: LatLng? = null
    // Consecutive confident-railed ticks; the map→VIO position push only fires once this reaches
    // MAP_POS_MIN_STREAK (review HIGH — a single wrong-road Viterbi win must not start the feedback).
    @Volatile private var mapPosPushStreak = 0
    // Consecutive ticks the ball has stayed far from the matcher's confident dot ON THE SAME wayId;
    // once it reaches RAIL_MATCHER_REACQUIRE_TICKS the ball re-acquires onto the matcher's road
    // (wrong-turn recovery). railMatcherWayId is the way that streak is accumulating on.
    @Volatile private var railMatcherDivergeTicks = 0
    @Volatile private var railMatcherWayId: Long? = null
    // 2026-06-02 — last STABLE travel direction along the road (net bearing over the matched polyline);
    // held across no-match ticks so the ball's orientation + the arrow never flip on per-tick VIO noise.
    @Volatile private var lastTravelBearing: Double = Double.NaN
    // 2026-06-02 (owner's heading-lock model): the gyro/Madgwick heading (deg), and the running offset
    // between it and the road tangent. While on a road the gyro's ABSOLUTE value may be wrong but its
    // RELATIVE change is reliable, so we calibrate offset = gyro − road_tangent; at a junction the road
    // is picked by gyro − offset (the user's real heading), giving "rotate −50 → take the 130 road".
    @Volatile private var lastVioHeadingDeg: Double = Double.NaN
    @Volatile private var headingOffsetDeg: Double = Double.NaN
    // 2026-06-02 (owner: "on a straight road with a slight fork it takes the OTHER exit even though I
    // didn't rotate") — the drift-free GYRO-RELATIVE junction anchor. We snapshot (road bearing, gyro)
    // whenever the ball is confidently STRAIGHT on a road; the STEERING heading that decides the next exit
    // is then railRoadBearingAnchorDeg + (gyroNow − railHeadingAnchorGyroDeg). This uses the EXACT live
    // road geometry + only the SMALL recent gyro delta, so it is free of the absolute-heading error that
    // the EMA-based steering carried at the decision instant. That error has two independent sources: (1)
    // the rail offset EMA (α=0.1) LAGS road-direction changes (magnetometer-independent — the dominant
    // one); (2) the magnetometer yaw correction is gyro-primary, SLOW (τ=5s complementary, IMUPreintegrator
    // ~958) and GATED OFF in magnetic disturbance (>35° mag/gyro disagreement — common on a scooter mount),
    // so it does not null gyro drift instantly. At a SHALLOW fork the decision boundary is half the (small)
    // fork angle, so a few degrees of that error was enough to flip onto the branch when the user hadn't
    // turned. The gyro's RELATIVE rotation over a few seconds is reliable regardless, so we use it instead.
    // With the anchor: no rotation → delta ≈ 0 → the straight continuation wins; a real turn moves the
    // delta → the matching exit wins ("stayed → 180, rotated −50 → 130", the owner's spec). Held (NOT
    // updated) through curves/turns where straightRoad is false, so the delta accumulates the real turn.
    @Volatile private var railHeadingAnchorGyroDeg: Double = Double.NaN
    @Volatile private var railRoadBearingAnchorDeg: Double = Double.NaN
    // Consecutive ticks the corrected heading has been clearly opposite the ball's facing → a confirmed
    // U-turn / wrong-facing once it reaches RAIL_REVERSE_TICKS (debounced so noise never flips the dot).
    @Volatile private var railReverseTicks = 0
    // 2026-06-03 — wall time of the last rail-ball advance, so the IPM-speed→distance integration uses the
    // real (gated, variable) tick interval instead of a fixed 0.5 s.
    @Volatile private var lastRailAdvanceMs: Long = 0L
    @Volatile private var lastSnapSource = ""
    // Guards against overlapping snap coroutines while a /match HTTP call is in flight
    // (set/cleared on the Main collector thread + the IO finally).
    @Volatile private var snapInFlight = false
    // Tier-1 #3: the matched ROAD polyline for the current window (on-road geometry the
    // dot sits on). Empty when on the per-point fallback / unmatched. Rendered by the map.
    var matchedRoadPath by mutableStateOf<List<LatLng>>(emptyList())
        private set
    // 2026-06-02 — on-road breadcrumb of the advance-along-rail ball positions. The clean trail of
    // where the user actually went, ON the roads, that never teleports (the ball is graph-constrained).
    // Rendered as the green trail in place of the raw Viterbi matchedRoadPath (which still teleports).
    val railTrail = androidx.compose.runtime.mutableStateListOf<LatLng>()
    // 2026-06-02 — last position the dot sat ON a road; the dot HOLDS here when the VIO drifts past any
    // road (never renders the raw off-road VIO). Owner: "i dont want it to go offroad, never."
    @Volatile private var lastOnRoadPos: LatLng? = null
    // 2026-06-02 — the map arrow's heading = the road's tangent at the ball (so it points ALONG the road
    // and turns only at junctions), not the VIO heading. null until the ball has acquired.
    var railBearingDeg by mutableStateOf<Float?>(null)
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
        // Discard a mount height auto-calibrated against an OLD IPM (the geometry was fixed 2026-06-02 — a
        // prior calib of ~1.70 m was just the under-read compensation). Reset to the default so GPS
        // re-calibrates against the corrected estimator.
        if (prefs.getInt(PREF_CAMERA_HEIGHT_VERSION, 0) != CAMERA_HEIGHT_CALIB_VERSION) {
            prefs.edit().remove(PREF_CAMERA_HEIGHT)
                .putInt(PREF_CAMERA_HEIGHT_VERSION, CAMERA_HEIGHT_CALIB_VERSION).apply()
            cameraHeightM = SCOOTER_CAMERA_HEIGHT_M
            heightCalibrated = false
        }
        // 2026-06-02 — activate the read-only ground-plane metric-scale estimator with the scooter
        // camera-to-road mount height (default ~1.05 m; the camera debug overlay draws what it finds).
        // READ-ONLY: it never feeds the dot — it's the on-device validation tool for the camera-height
        // scooter-speed idea (gpt_speed_suggestion.md; the offline harness can't fire on sparse frames).
        NativeBridge.setCameraHeight(cameraHeightM)   // persisted GPS-calibrated value, else the default guess
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
                // READ-ONLY speed channels for offline GPS scoring (IPM ground-plane validation).
                // Cheap native atomic reads; -1.0 sentinel → null so the analyzer skips "not yet".
                val gpFlow = runCatching { NativeBridge.getGroundFlowSpeedMps() }.getOrNull()
                    ?.takeIf { it >= 0f }?.toDouble()
                val fusedSp = runCatching { NativeBridge.getFusedSpeedMps() }.getOrNull()
                    ?.takeIf { it >= 0f }?.toDouble()
                // 2026-06-04 — per-frame IPM depth-band diagnostic, persisted in the sim JSON (logcat rolls off).
                val ipmBandArr = runCatching {
                    val buf = FloatArray(15)
                    if (NativeBridge.getIpmBandDiag(buf) >= 15) buf else null
                }.getOrNull()
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
                        mmSrc = mmS, kMap = kM, maneuver = mvr,
                        gpFlowSpeed = gpFlow, fusedSpeed = fusedSp,
                        ipmBand = ipmBandArr
                    ))
                }
            }

            // 2026-06-03 (owner: "show the dot's distance, not the raw VIO"): the displayed totalDistanceM
            // now follows the DISPLAYED DOT (the snapped on-road position) — accumulated in launchNetworkSnap
            // below — instead of the raw VIO global_t_ (vio.x/vio.z), the under-scaled trajectory (e.g. 786 m
            // vs the dot's ~1195 m on the same ride). Kept (commented) per the no-delete rule.
            /* val prevDist = lastVioForDist
            if (prevDist != null) {
                val ddx = vio.x - prevDist.x
                val ddz = vio.z - prevDist.z
                totalDistanceM += sqrt(ddx * ddx + ddz * ddz)
            }
            lastVioForDist = vio */

            // 2026-05-26 — locomotion-agnostic speed. Report the smoothed
            // depth-weighted metric speed (getFusedSpeedMps: recoverPose translation
            // scaled by tracked-point MiDaS depths) for ALL motion types instead of
            // differencing a stride-scaled position. Returns -1.0 before the first
            // estimate; until then show 0.
            if (NativeBridge.isLoaded()) {
                val fusedMps = NativeBridge.getFusedSpeedMps()   // LEGACY displayed speed — kept only for the diag log
                // IPM ground-plane speed. 2026-06-03 (owner): the IPM now DRIVES the displayed speedometer (and the
                // rail ball, below), REPLACING the frozen getFusedSpeedMps path. With Fix A it is continuous and
                // decays to ~0 at a stop (no separate true-stop snap needed — the EMA follows it down). NOTE: while
                // MOVING it is currently front-end-saturated → decorrelated from true speed (Fix B not done); it
                // will track once B restores the fast near-ground flow. The B probe measures whether that's possible.
                val gpMps = NativeBridge.getGroundFlowSpeedMps()
                groundFlowSpeedKmh = if (gpMps >= 0f) gpMps * 3.6f else null
                maybeAutoCalibrateHeight(gpMps)   // GPS one-time mount-height solve (read-only of position)
                if (gpMps < 0f) {
                    // -1.0 sentinel: no IPM estimate yet → show 0.
                    speedEmaMps = 0f
                    currentSpeedKmh = 0f
                } else {
                    // EMA low-pass for a stable speedometer. τ ≈ 0.7 s; at the UI tick cadence α = dt/(τ+dt).
                    // Cited, not magic: 0.7 s trades a little display lag for jitter rejection (the raw IPM
                    // jitters 4-48 km/h). Fix A's decay already takes the IPM to ~0 at a stop, so the EMA
                    // follows it to 0-2 km/h — the standstill behaviour the owner confirmed in-room.
                    val dtS = (nowMs - lastSpeedTimeMs).coerceAtLeast(1L).toDouble() / 1000.0
                    val tauS = 0.7
                    val alpha = (dtS / (tauS + dtS)).toFloat()
                    speedEmaMps += alpha * (gpMps - speedEmaMps)
                    currentSpeedKmh = speedEmaMps * 3.6f
                }
                lastSpeedTimeMs = nowMs
                if (nowMs % 3000L < 60L)
                    Log.i("NavSightVM", "SPEED_SRC: ipm=%.1f disp=%.1f legacy_fused=%.1f km/h".format(
                        (groundFlowSpeedKmh ?: -1f), currentSpeedKmh, fusedMps * 3.6f))

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
                    lastVioHeadingDeg = headingDeg   // gyro/Madgwick heading → ball junction choice (relative turn)
                    val maneuver = maneuverStateMachine.tick(
                        currentLatLng.latitude, currentLatLng.longitude, headingDeg, region?.roundabouts
                    )
                    maneuver.event?.let { ev ->
                        // RAIL-LOCK (2026-05-31): a maneuver — roundabout entry/exit, U-turn — is the
                        // sanctioned reason to leave the current road. Release the matcher's rail so it
                        // re-locks to the NEW road, instead of being dragged back onto the old one.
                        // (Plain intersection turns with no event are caught by LocalMatcher's range
                        // recovery; a dedicated ~90° classifier is the next refinement.)
                        localMatcher.releaseRail()
                        Log.i("ManeuverSM", "MANEUVER event=$ev state=${maneuver.state} " +
                            "sweep=${"%.0f".format(maneuver.accumulatedSweepDeg)} " +
                            "exitBearing=${maneuver.suggestedExit?.bearingDeg?.let { "%.0f".format(it) } ?: "-"} " +
                            "dir=${maneuver.travelDirection} railReleased=1")
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
                // ── advance-along-rail (ball-in-maze, 2026-05-31) — the displayed dot is a ball
                // CONSTRAINED to the routing graph: it advances ALONG its road by the VIO step and can
                // only cross to a CONNECTED edge at a junction, so it never teleports to a parallel road
                // ("it stays inside the maze... it can't jump through the wall"). Offline-validated on
                // sim A: 0 teleports, 0.82× GPS (vs the re-projecting matcher's 8 teleports / 1.80×). ──
                if (region != null && region !== graphRailRegion) {
                    graphRail = GraphRailDot(region.graph); graphRailRegion = region; lastSnapVio = null
                }
                val rail = graphRail
                val prevVio = lastSnapVio
                val dVioRaw = if (prevVio != null) RoadSnapMath.haversineM(
                    prevVio.latitude, prevVio.longitude, currentLatLng.latitude, currentLatLng.longitude) else 0.0
                // BALL SPEED = IPM (2026-06-03, owner): advance the ball by the IPM-measured distance (the
                // displayed, EMA-smoothed speed × the real tick dt) instead of the VIO position delta, so the
                // ball moves at the SHOWN speed. Falls back to the VIO delta before the first IPM estimate
                // (groundFlowSpeedKmh == null). Capped at RAIL_MAX_SPEED_MPS × dt to absorb a spike. NOTE: while
                // the IPM is front-end-saturated (pre-Fix-B) the ball advances at a wrongish ~constant pace —
                // expected until B; the matcher supervision still corrects the road choice.
                val railDtS = if (lastRailAdvanceMs > 0L)
                    (nowMs - lastRailAdvanceMs).coerceIn(50L, 1500L).toDouble() / 1000.0 else 0.5
                lastRailAdvanceMs = nowMs
                val ipmBallMps = currentSpeedKmh / 3.6f   // the displayed (EMA-smoothed) IPM speed
                val advanceM = if (groundFlowSpeedKmh != null)
                    minOf(ipmBallMps.toDouble() * railDtS, RAIL_MAX_SPEED_MPS * railDtS)
                    else minOf(dVioRaw, RAIL_MAX_SPEED_MPS * railDtS)
                // TRAVEL DIRECTION along the road = net bearing over the MATCHED polyline (snapped
                // positions, oldest→newest). Road-aligned, time-ordered, HEADING-INDEPENDENT → stable even
                // at crawl speed. The per-tick VIO delta we used before is pure noise at 3 km/h and made
                // the ball/arrow flip 180° every tick. Held across a no-match tick so it never goes NaN.
                val mpw = matched?.matchedPath
                val travelBearing: Double = if (accept && mpw != null && mpw.size >= 2 &&
                        RoadSnapMath.haversineM(mpw.first().latitude, mpw.first().longitude,
                                                mpw.last().latitude, mpw.last().longitude) > 2.0) {
                    val tb = ManeuverMath.bearingDeg(mpw.first().latitude, mpw.first().longitude,
                                                     mpw.last().latitude, mpw.last().longitude)
                    lastTravelBearing = tb; tb
                } else lastTravelBearing
                val seedBearing = if (travelBearing.isNaN()) 0.0 else travelBearing
                var railPos: GeoPt? = null
                if (rail != null) {
                    if (!rail.acquired) {
                        val sLat = if (accept && matched != null) matched.matched.latitude else currentLatLng.latitude
                        val sLng = if (accept && matched != null) matched.matched.longitude else currentLatLng.longitude
                        if (rail.acquire(sLat, sLng, seedBearing)) {
                            // Seed the junction anchor AT acquire time (not only on the first straight tick)
                            // so steerHeading uses the drift-free gyro-relative model from tick 0. Without
                            // this, the first ticks after an acquire / matcher re-acquire (a teleport onto a
                            // new road, when the ball is most likely right next to a junction) steer on the
                            // stale travelBearing fallback → a transient wrong-exit. The acquired edge tangent
                            // is a drift-free reference; pair it with the current gyro (NaN-guarded downstream).
                            railRoadBearingAnchorDeg = rail.currentBearingDeg() ?: seedBearing
                            railHeadingAnchorGyroDeg = lastVioHeadingDeg
                        }
                    } else {
                        // OWNER'S HEADING-LOCK MODEL: the heading IS the road tangent; at a junction the
                        // road is chosen by the gyro's RELATIVE rotation. While on a road, calibrate the
                        // offset between the (possibly-wrong-absolute) gyro heading and the road tangent;
                        // the ball then turns at a junction toward (gyro − offset) = the user's real
                        // heading, so "rotate −50 → take the 130 road", etc. The gyro change is reliable
                        // even when its absolute value is off, so this is stable across the intersection.
                        // Calibrate offset = gyro − road on a confident STRAIGHT stretch only, from the
                        // EXTERNAL matched-road direction (NOT the ball edge — that would be circular and
                        // re-oscillate). Held through junctions/curves where the matched direction is noisy.
                        // LOCAL road tangent = bearing of the LAST matched segment (the road direction AT the
                        // ball's current position). Calibrate the gyro offset against THIS, not the net
                        // first→last travelBearing. WHY (owner-reported U-turn flip-back, 2026-06-02): right
                        // after a U-turn the matched window still holds the OLD-direction half, so the NET
                        // bearing points ~180° the wrong way while the last segment already points the NEW way.
                        // The straightRoad gate (last-2-segs colinear) goes true the moment you're straight
                        // again, so calibrating off the stale NET would rewrite a correct held offset with one
                        // corrupted by ~180° → correctedHeading snaps back the old way → reverse() re-fires →
                        // the arrow rotates back. The gyro offset (sensor bias vs true heading) is
                        // direction-INDEPENDENT, so the held value is already correct for the new heading; we
                        // must just stop poisoning it with the lagging net. The last segment is the gate's own
                        // validated, transition-safe road direction.
                        val roadTangentDeg = if (mpw != null && mpw.size >= 2)
                            ManeuverMath.bearingDeg(mpw[mpw.size - 2].latitude, mpw[mpw.size - 2].longitude,
                                                    mpw[mpw.size - 1].latitude, mpw[mpw.size - 1].longitude)
                            else Double.NaN
                        val straightRoad = mpw != null && mpw.size >= 3 && !roadTangentDeg.isNaN() && kotlin.math.abs(
                            ManeuverMath.angularDifferenceDeg(roadTangentDeg,
                                ManeuverMath.bearingDeg(mpw[mpw.size - 3].latitude, mpw[mpw.size - 3].longitude,
                                                        mpw[mpw.size - 2].latitude, mpw[mpw.size - 2].longitude))) < 30.0
                        if (accept && straightRoad && !roadTangentDeg.isNaN() && !lastVioHeadingDeg.isNaN() &&
                                maneuver.state != ManeuverState.ON_ROUNDABOUT) {
                            val inst = ManeuverMath.angularDifferenceDeg(lastVioHeadingDeg, roadTangentDeg)
                            headingOffsetDeg = if (headingOffsetDeg.isNaN()) inst
                                else headingOffsetDeg + HEADING_OFFSET_ALPHA *
                                    ManeuverMath.angularDifferenceDeg(inst, headingOffsetDeg)
                            // 2026-06-02 (owner's offset model: "have the road's heading but only turn when
                            // the offset is met — if the road is 100 and there's a turn to 50 and a turn to
                            // 150, offset +50 → take 150, −50 → take 50"). The gyro-reference re-anchor MOVED
                            // OUT of this every-straight-tick block down to an EDGE-CHANGE gate (after
                            // advance/reverse). Re-anchoring here on every confident-straight tick wiped the
                            // offset each tick — gyroDelta only ever held ONE tick's rotation (≈0) → steerHeading
                            // ≈ the road → the ball ONLY went straight and never took a junction or U-turn
                            // (owner: "i cant do a u turn or chose a road on a junction... it only goes
                            // straight"). headingOffsetDeg (the EMA above) is KEPT as the correctedHeading
                            // fallback for the earliest ticks before the anchor is first seeded on acquire.
                            /* MOVED to the edge-change re-anchor below (2026-06-02) — see the EDGE-CHANGE block:
                            railRoadBearingAnchorDeg = rail.currentBearingDeg() ?: roadTangentDeg
                            railHeadingAnchorGyroDeg = lastVioHeadingDeg */
                        }
                        // EMA correctedHeading (gyro − smoothed offset) — kept as the FALLBACK only, used
                        // until the drift-free anchor below has been set on the first confident-straight tick.
                        val correctedHeading = if (!lastVioHeadingDeg.isNaN() && !headingOffsetDeg.isNaN())
                            ((lastVioHeadingDeg - headingOffsetDeg) % 360.0 + 360.0) % 360.0
                            else travelBearing
                        // STEERING heading = the drift-free anchor model: the road bearing the ball is on +
                        // the gyro rotation since the last confident-straight tick. THIS is what picks the
                        // exit at a junction. It is robust to the EMA-offset lag that was making the ball
                        // take a slight fork when the user hadn't turned: no rotation → gyroDelta ≈ 0 →
                        // steer = the road the ball is on → the straight continuation wins. Falls back to the
                        // EMA correctedHeading until the anchor is first set (early ticks after acquire).
                        val gyroDelta = if (!railHeadingAnchorGyroDeg.isNaN() && !lastVioHeadingDeg.isNaN())
                            ManeuverMath.angularDifferenceDeg(lastVioHeadingDeg, railHeadingAnchorGyroDeg)
                            else Double.NaN
                        // Final fallback is seedBearing (coerced non-NaN), NOT correctedHeading — which is
                        // itself NaN on the earliest ticks (offset + travelBearing both unset) — so steerHeading
                        // is never NaN and no silent bad value reaches advance()/the reversal test.
                        val steerHeading = if (!railRoadBearingAnchorDeg.isNaN() && !gyroDelta.isNaN())
                            ((railRoadBearingAnchorDeg + gyroDelta) % 360.0 + 360.0) % 360.0
                            else if (!correctedHeading.isNaN()) correctedHeading
                            else seedBearing
                        val facingBefore = rail.currentBearingDeg()
                        // U-TURN DEBOUNCE BACK-HOP GUARD: when the steering heading is CLEARLY opposite the
                        // ball's facing (>120°), a U-turn (or wrong facing) is being debounced. Do NOT let
                        // advance() junction-hop onto a backward-pointing connected edge during the debounce —
                        // that is an ungated pseudo-U-turn before the clean, latched reverse() flip (a brief
                        // wrong-edge excursion at a multi-way junction). Keep advancing along the CURRENT
                        // facing; only reverse() (after RAIL_REVERSE_TICKS) changes direction. Same 120° gate,
                        // computed once and reused for both the advance guard and the reversal counter.
                        val opposed = facingBefore != null && !steerHeading.isNaN() &&
                            kotlin.math.abs(ManeuverMath.angularDifferenceDeg(steerHeading, facingBefore)) > 120.0
                        rail.advance(advanceM, if (opposed) (facingBefore ?: steerHeading) else steerHeading)
                        // INSTRUMENTATION: log ONLY when the ball actually changed road direction this tick
                        // (a junction was taken) — the exact event being debugged, not per-tick spam. A
                        // wrong-exit shows as a large facing jump with gyroD≈0 (took a branch without turning);
                        // a swallowed turn shows as NO RAIL_JUNCTION line on the tick the user turned at a
                        // shallow fork (anchor re-absorbed the rotation) — both diagnosable from a logcat pull.
                        val facing = rail.currentBearingDeg()
                        if (facingBefore != null && facing != null &&
                            kotlin.math.abs(ManeuverMath.angularDifferenceDeg(facing, facingBefore)) > RAIL_JUNCTION_LOG_MIN_TURN_DEG) {
                            Log.i("RailDot", "RAIL_JUNCTION: %.0f->%.0f steer=%.0f gyroD=%.0f anchorRoad=%.0f".format(
                                facingBefore, facing, steerHeading,
                                if (gyroDelta.isNaN()) 0.0 else gyroDelta,
                                railRoadBearingAnchorDeg))
                        }
                        // DEBOUNCED REVERSAL (owner: "if it thinks I'm facing the wrong direction, how do
                        // we make it rotate? what if I did a U-turn?"). Sustained "steering clearly opposite
                        // the facing" (the `opposed` flag above) for RAIL_REVERSE_TICKS → a real U-turn /
                        // wrong facing → flip the ball 180°. Debounced so per-tick noise never flips it.
                        if (opposed) {
                            railReverseTicks++
                            if (railReverseTicks >= RAIL_REVERSE_TICKS) {
                                if (rail.reverse()) Log.i("RailDot", "RAIL_REVERSE: sustained heading opposite facing")
                                railReverseTicks = 0
                            }
                        } else railReverseTicks = 0
                        // OFFSET-PRESERVING EDGE-CHANGE RE-ANCHOR (2026-06-02 — owner's "road heading + offset"
                        // model; the fix for "it only goes straight, can't U-turn or choose a junction"). Reset
                        // the gyro reference to offset=0 ONLY when the ball actually crossed to a NEW-bearing
                        // edge THIS tick — a junction taken (facing changed during advance) or a reverse (facing
                        // flipped). Between edge crossings the anchor stays FROZEN, so gyroDelta = the user's
                        // REAL rotation since they got on the current road → steerHeading = road + offset picks
                        // the matching exit (road 100 + gyro +50 → the 150 exit) and a sustained ~180° offset
                        // trips reverse(). Reuses RAIL_JUNCTION_LOG_MIN_TURN_DEG (1°) — the same exact-graph-
                        // geometry "the ball changed road direction" threshold the junction log just above uses
                        // — so no new constant. On a STRAIGHT road facing is unchanged → no re-anchor → the
                        // offset survives to the next junction; on a CURVE each segment re-anchors so following
                        // the road's own bend is never mistaken for the user's offset.
                        val facingAfter = rail.currentBearingDeg()
                        if (facingBefore != null && facingAfter != null && !lastVioHeadingDeg.isNaN() &&
                            kotlin.math.abs(ManeuverMath.angularDifferenceDeg(facingAfter, facingBefore)) >
                                RAIL_JUNCTION_LOG_MIN_TURN_DEG) {
                            railRoadBearingAnchorDeg = facingAfter
                            railHeadingAnchorGyroDeg = lastVioHeadingDeg
                            Log.i("RailDot", "RAIL_REANCHOR: road=%.0f offset->0 (edge %.0f->%.0f)".format(
                                facingAfter, facingBefore, facingAfter))
                        }
                        // WIRE THE MATCHER (2026-06-02) — supervise the ball's road choice. The junction
                        // CHOICE stays VIO-bearing-driven (the user's actual motion is the true signal for
                        // which exit they took), but if the ball then stays far from the matcher's CONFIDENT
                        // window-Viterbi dot for several consecutive ticks, it took a WRONG exit / wrong road
                        // → re-acquire onto the matcher's road. This is the multi-exit / wrong-turn recovery
                        // (and the root mitigation for the position-leg wrong-road runaway). Sustained +
                        // high-confidence gating means a transient parallel-road matcher flicker (which flips
                        // back within a tick) can't trigger a spurious teleport.
                        val rp = rail.position()
                        val mWayId = matched?.matchedWayId
                        val mOff = if (matched != null && rp != null) RoadSnapMath.haversineM(
                            rp.lat, rp.lng, matched.matched.latitude, matched.matched.longitude) else 0.0
                        if (MATCHER_SUPERVISION_ENABLED &&
                            accept && matched != null && matched.confidence >= MAP_POS_MIN_CONFIDENCE &&
                            mWayId != null && rp != null && mOff > RAIL_MATCHER_REACQUIRE_M) {
                            // Diverged from a confident matcher THIS tick. Only count it when the matcher
                            // is STABLE on the SAME wayId — a flickering matcher (jumping between parallel
                            // roads, the unreliable-heading signature) keeps resetting the streak and never
                            // triggers a spurious re-acquire; a rock-solid disagreement (a real wrong road)
                            // accumulates and recovers the ball onto the matcher's road.
                            if (mWayId == railMatcherWayId) railMatcherDivergeTicks++
                            else { railMatcherWayId = mWayId; railMatcherDivergeTicks = 1 }
                            if (railMatcherDivergeTicks >= RAIL_MATCHER_REACQUIRE_TICKS) {
                                rail.acquire(matched.matched.latitude, matched.matched.longitude, seedBearing)
                                railMatcherDivergeTicks = 0; railMatcherWayId = null
                                Log.i("RailDot", "RAIL_REACQUIRE matcherOff=%.0fm conf=%.2f way=%d".format(mOff, matched.confidence, mWayId))
                            }
                        } else { railMatcherDivergeTicks = 0; railMatcherWayId = null }
                    }
                    railPos = rail.position()
                }
                lastSnapVio = currentLatLng

                val snapped: SnappedLatLng
                val matchedPath: List<LatLng>
                if (railPos != null && rail?.acquired == true) {
                    // Ball is on a road — it IS the dot whether or not THIS tick's matcher accepted
                    // (dead-reckoning along the graph keeps us on the road between confident matches).
                    snapped = SnappedLatLng(railPos.lat, railPos.lng, if (accept) "rail" else "rail-dr", null, true)
                    matchedPath = if (accept && matched != null) matched.matchedPath else matchedRoadPath
                } else if (accept && matched != null) {
                    snapped = SnappedLatLng(matched.matched.latitude, matched.matched.longitude, "local:match", null, true)
                    matchedPath = matched.matchedPath        // #3 — on-road geometry
                } else {
                    snapped = roadSnapper.snapToRoad(currentLatLng, recentPath, maneuver)
                    // #10 — keep the last matched road on a TRANSIENT /match miss (no green-line flicker).
                    matchedPath = if (snapped.isSnapped) matchedRoadPath else emptyList()
                }
                // 2026-05-31 (map-as-sensor HEADING leg) — when confidently RAILED on a STRAIGHT road
                // in FREE_ROAD, push that road's bearing to native so it nudges the VIO heading onto the
                // road (stops the off-road drift; the user's "fix the heading so we stay on the road").
                // Native gates on |heading-road|<90° (kRoadMaxResidualRad = π/2, the crossing-road
                // backstop) + magnetometer-not-fusing. Sentinel -1000 clears the hint when not applicable.
                // 2026-06-02 — road→heading correction TARGET = the ball-on-rail's CURRENT edge bearing
                // (the road's LOCAL TANGENT where you are). Per-road by construction (the ball is
                // graph-constrained), and it follows CURVATURE as the ball advances (owner: "when a road
                // is curved the heading doesn't match the curvature, update simultaneously") — a turn onto
                // a different road just transitions the ball to that road's tangent. TRUST = the ball is
                // acquired on a road AND the matcher confidently agrees (a wrong/crossing road won't be
                // confidently matched). No averaged segment, no sharp-turn skip (a curve IS the road);
                // only a roundabout is skipped (its tangent is meaningless while circulating). Native
                // picks the travel-aligned direction (no 180° flip) + the 90° crossing-road backstop.
                val roadHint: Double = if (accept && matched != null &&
                        matched.confidence >= MAP_POS_MIN_CONFIDENCE && rail?.acquired == true &&
                        maneuver.state != ManeuverState.ON_ROUNDABOUT) {
                    rail.currentBearingDeg() ?: -1000.0
                } else -1000.0
                NativeBridge.setRoadHeadingHint(roadHint)
                // map-as-sensor POSITION leg — push the world-frame (ball − VIO) error so native bleeds
                // its CROSS-TRACK part into global_t_, pulling the drifting VIO trajectory onto the road
                // (owner: "fix the drifting vio based on the map matcher"). HARD-GATED (review HIGH —
                // wrong-road runaway / ADR-004): only when the SAME condition the heading hint requires
                // holds (roadHint != -1000 ⇒ accept + railed + FREE_ROAD + straight), AND the match is
                // high-confidence, AND it has held for several consecutive ticks — so a transient or a
                // wrong-parallel-road lock cannot start dragging the VIO. Native applies it while the
                // position leg is enabled (default-ON since 2026-06-02, Tracker.h map_pos_correction_enabled_).
                val posConfident = accept && matched != null &&
                    matched.confidence >= MAP_POS_MIN_CONFIDENCE && roadHint != -1000.0
                mapPosPushStreak = if (posConfident) mapPosPushStreak + 1 else 0
                if (railPos != null && rail?.acquired == true && posConfident &&
                    mapPosPushStreak >= MAP_POS_MIN_STREAK) {
                    val mLat = RoadSnapMath.M_PER_DEG_LAT
                    val mLng = RoadSnapMath.M_PER_DEG_LAT * kotlin.math.cos(Math.toRadians(currentLatLng.latitude))
                    val dEast = (railPos.lng - currentLatLng.longitude) * mLng
                    val dNorth = (railPos.lat - currentLatLng.latitude) * mLat
                    NativeBridge.setMapPositionCorrection(dEast, dNorth)
                }
                val srcLabel = snapped.placeId ?: if (snapped.isSnapped) "snap" else "raw"
                // 2026-06-12ui — feed the debug panel's MATCHER group.
                mmSrcDebug = srcLabel
                mmConfDebug = matched?.confidence?.toFloat() ?: -1f
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
                    val dotNow = snapped.toLatLng()
                    // IN-APP DISTANCE = the DISPLAYED DOT's path (2026-06-03, owner). Accumulate the on-road
                    // dot movement; a step faster than the rail speed cap (RAIL_MAX_SPEED_MPS) is a matcher
                    // reacquire/teleport — a snap correction, not travel — so it is excluded. This is what
                    // replaces the raw-VIO totalDistanceM (commented out above) so the counter matches the dot.
                    lastDotForDist?.let { prev ->
                        val d = RoadSnapMath.haversineM(prev.latitude, prev.longitude, dotNow.latitude, dotNow.longitude)
                        val ddt = (nowMs - lastDotDistMs).coerceIn(1L, 3000L).toDouble() / 1000.0
                        if (d / ddt <= RAIL_MAX_SPEED_MPS) totalDistanceM += d
                    }
                    lastDotForDist = dotNow; lastDotDistMs = nowMs
                    lastMapMatched = dotNow
                    lastMapMatchedConf = if (accept && matched != null) matched.confidence else {
                        val rawSL = SnappedLatLng(currentLatLng.latitude, currentLatLng.longitude, null, null, false)
                        exp(-snapped.distanceTo(rawSL) / 5.0)   // σ=5 m (RoadSnapper.SNAP_SIGMA_M)
                    }
                } else {
                    lastMapMatched = null
                    lastMapMatchedConf = null
                    lastDotForDist = null   // off-road gap → re-snap starts fresh, don't count the gap as travel
                }
                // #8 — when navigating, project onto the planned route (route-pinned) + detect
                // off-route → reroute. Returns the display position (pinned within the corridor).
                // NEVER OFF-ROAD (owner: "i dont want it to go offroad, never"): the dot must always sit
                // on a road. When this tick is on a road (snapped) remember it; when it is NOT (the VIO
                // drifted past the search radius and there's no road within reach), HOLD the last on-road
                // position instead of rendering the raw off-road VIO. The dot freezes on the road until the
                // VIO comes back into reach — never visibly leaves it.
                if (snapped.isSnapped) lastOnRoadPos = snapped.toLatLng()
                val dotLatLng = if (snapped.isSnapped) snapped.toLatLng()
                                else (lastOnRoadPos ?: snapped.toLatLng())
                val displayPos = if (navigationState is NavigationState.Active) {
                    navigationManager.updateVioPosition(dotLatLng)
                } else {
                    dotLatLng
                }
                val railBreadcrumb = if (railPos != null && rail?.acquired == true)
                    LatLng(railPos.lat, railPos.lng) else null
                // Arrow direction = the ball's road TANGENT (the road's direction where you are), NOT the
                // VIO Madgwick heading (which can be 90° off when the heading nudge is gated out). The
                // arrow then points ALONG the road and rotates only at turns/junctions, where the ball
                // crosses to the next edge (owner: "the arrow should point wherever the roads direction
                // is and only rotate on turns/roundabouts"). Held across a transient miss so it never
                // snaps back to the wrong VIO heading.
                val railBearing: Float? = (if (rail?.acquired == true) rail.currentBearingDeg() else null)?.toFloat()
                withContext(Dispatchers.Main) {
                    snappedPosition = displayPos
                    matchedRoadPath = matchedPath
                    if (railBearing != null) railBearingDeg = railBearing
                    if (railBreadcrumb != null) {
                        railTrail.add(railBreadcrumb)
                        while (railTrail.size > RAIL_TRAIL_MAX) railTrail.removeAt(0)
                    }
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
        // A long-press relocation is a DELIBERATE teleport of the origin. The ball-on-rail ignores VIO
        // position jumps by design (it advances along the road by capped steps), so it would otherwise
        // stay stuck on its old edge and the dot wouldn't move. Drop it → the next snap re-acquires the
        // ball at the new location (regression: "when i hold to change location it doesnt change").
        resetRailState()
        // Anchor the never-off-road HOLD at the point you tapped, and show the dot there immediately.
        // If that area's roads aren't downloaded/cached yet, the dot STAYS here (on the road you placed)
        // until the region loads and the ball acquires — instead of drifting off-road from the raw VIO
        // (owner: "if i hold on a place and it isn't downloaded/cached it still goes offroad").
        lastOnRoadPos = latLng
        snappedPosition = latLng
        RoadRegionManager.ensureRegion(latLng.latitude, latLng.longitude)
        Log.i("NavSightVM", "START_OVERRIDE lat=${latLng.latitude} lng=${latLng.longitude}")
    }

    /** Drop the advance-along-rail ball state so the next snap re-acquires from scratch. Used on a full
     *  reset and on a deliberate start-location relocation (both are origin teleports the ball must follow). */
    /**
     * 2026-06-02 — GPS one-time mount-height auto-calibration. The IPM ground-plane speed is LINEAR in the
     * camera mount height, so a single trusted GPS speed solves the height: h_true = h × (gps_mps / ipm_mps).
     * Runs ONLY until calibrated (key persisted), needs a clean GPS fix + real motion + the IPM firing, and
     * the committed height is hard-clamped to a physically sane 0.3–2.5 m — so a still-wrong IPM (e.g. a bad
     * direction frame producing a 13 m height) is rejected and the calibration self-activates only once the
     * IPM is actually correct. GPS is used ONLY to set this physical constant; the dot/position stays VIO.
     */
    private fun maybeAutoCalibrateHeight(ipmMps: Float) {
        if (heightCalibrated || ipmMps < 0.8f) return
        val g = currentGpsLocation ?: return
        if (!g.hasSpeed() || !g.hasAccuracy() || g.accuracy > HEIGHT_CALIB_MAX_GPS_ACC_M ||
            g.speed < HEIGHT_CALIB_MIN_MPS) return
        // 2026-06-12 — GPS-JAM hardening (Haifa). The ride val_2026_06_03b shows jammed segments where
        // the position FREEZES (position-derived speed ≈ 0) or JUMPS (>60 km/h spikes) while fixes still
        // report good accuracy — a height solved from such fixes is poison and LATCHES via prefs. A
        // healthy receiver's Doppler speed agrees with its position-derived speed (~10% at >3 m/s); jam
        // failure modes disagree at ×-level, so require agreement within 30% (or 0.7 m/s absolute), and
        // trust the chip's own speed-accuracy estimate when it exists. Also: one sample per NEW fix —
        // the old path re-sampled the SAME ~1 Hz fix on every ~43 ms UI tick, filling the 25-sample
        // window from ~2 distinct fixes (a median of near-duplicates has no robustness).
        if (android.os.Build.VERSION.SDK_INT >= 26 && g.hasSpeedAccuracy() &&
            g.speedAccuracyMetersPerSecond > 1.5f) return
        val prevFix = lastHeightCalibFix
        if (prevFix != null && g.time <= prevFix.time) return   // same fix re-delivered by the UI tick
        lastHeightCalibFix = g
        if (prevFix == null) return                              // need two fixes for the cross-check
        val dtFixS = (g.time - prevFix.time) / 1000.0
        if (dtFixS < 0.2 || dtFixS > 5.0) return
        val posMps = g.distanceTo(prevFix) / dtFixS
        if (kotlin.math.abs(posMps - g.speed) > kotlin.math.max(0.3 * g.speed, 0.7)) return
        val ratio = g.speed.toDouble() / ipmMps.toDouble()   // = h_true / h_current
        if (!ratio.isFinite() || ratio < 0.2 || ratio > 5.0) return
        heightCalibRatios.add(ratio)
        if (heightCalibRatios.size < HEIGHT_CALIB_SAMPLES) return
        val hNew = (cameraHeightM * heightCalibRatios.sorted()[heightCalibRatios.size / 2]).coerceIn(0.3, 2.5)
        cameraHeightM = hNew
        NativeBridge.setCameraHeight(hNew)
        prefs.edit().putFloat(PREF_CAMERA_HEIGHT, hNew.toFloat()).apply()
        heightCalibrated = true
        heightCalibStatus = "h=%.2fm ✓".format(hNew)
        Log.i("NavSightVM", "HEIGHT_CALIB: h=$hNew n=${heightCalibRatios.size}")
    }

    /** Manual mount-height set (debug-panel fallback when GPS auto-calibration can't run, e.g. GPS jammed).
     *  Clamps to the physical range, applies + persists, and marks calibrated so the GPS auto-calibration
     *  won't override a deliberate manual value. */
    fun setMountHeight(h: Double) {
        val hc = h.coerceIn(0.3, 2.5)
        cameraHeightM = hc
        heightCalibrated = true
        NativeBridge.setCameraHeight(hc)
        prefs.edit().putFloat(PREF_CAMERA_HEIGHT, hc.toFloat()).apply()
        heightCalibStatus = "h=%.2fm (manual)".format(hc)
    }

    private fun resetRailState() {
        graphRail = null
        graphRailRegion = null
        lastSnapVio = null
        mapPosPushStreak = 0
        railMatcherDivergeTicks = 0
        railMatcherWayId = null
        railTrail.clear()
        lastOnRoadPos = null
        railBearingDeg = null
        lastTravelBearing = Double.NaN
        headingOffsetDeg = Double.NaN
        railHeadingAnchorGyroDeg = Double.NaN
        railRoadBearingAnchorDeg = Double.NaN
        railReverseTicks = 0
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
                // READ-ONLY speed channels for offline GPS scoring (IPM ground-plane validation).
                sb.append("\"gp_flow_speed\":${p.gpFlowSpeed ?: "null"},\"fused_speed\":${p.fusedSpeed ?: "null"},")
                // 2026-06-04 — per-DEPTH-BAND IPM diagnostic [near n,flow,vi,cos,surv, mid…, far…] = 15 floats.
                sb.append("\"ipm_band\":${p.ipmBand?.joinToString(",", "[", "]") ?: "null"},")
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
        lastVioForDist = null; lastDotForDist = null
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
        // Drop the advance-along-rail state on the MAIN thread (before the IO block) so a restart in the
        // SAME road region can't reuse a stale acquired ball / cross-session lastSnapVio jump (review HIGH).
        resetRailState()
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
        sessionAnchorSet = false          // 2026-06-19 — re-anchor to the NEW start (was: stale anchor reprojected the dot to the old start after a reset)
        lastMapMatched = null
        lastMapMatchedConf = null
        lastVioForDist = null; lastDotForDist = null
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