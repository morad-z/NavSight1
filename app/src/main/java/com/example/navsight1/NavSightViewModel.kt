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

    var virtualX by mutableStateOf(0.0)
        private set
    var virtualZ by mutableStateOf(0.0)
        private set

    private val _pathHistory = ArrayList<Pair<Float, Float>>(512)
    private val _pathHistoryVersion = AtomicInteger(0)
    var pathHistoryVersion by mutableStateOf(0)
        private set
    val pathHistory: List<Pair<Float, Float>> get() = _pathHistory

    var startLocation by mutableStateOf<LatLng?>(null)
        private set

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

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    var isRecordingSimulation by mutableStateOf(false)
        private set
    private val simulationDataPoints = mutableListOf<SimulationPoint>()
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
        val poseFlags: Int, val heading: Double
    )
    // ──────────────────────────────────────────────────────────────────────────

    val vioInitAzimuth: Float get() = sensorRepository.vioInitAzimuth

    private var lastVioForSpeed: VioData? = null
    private var lastVioForDist: VioData? = null
    private var lastSpeedTimeMs = 0L
    private var lastSnapTimeMs = 0L
    private var lastUiUpdateTimeMs = 0L
    private val UI_UPDATE_THROTTLE_MS = 200L

    private var latestVioState: VioData = VioData()
    private var hasLocationPermission = false

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

    fun updateUserHeight(height: Float) {
        val h = height.coerceIn(1.0f, 2.5f)
        userHeight = h
        prefs.edit().putFloat(PREF_USER_HEIGHT, h).apply()
        NativeBridge.setUserHeight(h)
    }

    init {
        NativeBridge.setScale(scaleCalibrationFactor)
        NativeBridge.setUserHeight(userHeight)

        viewModelScope.launch {
            sensorRepository.orientationState.sample(200L).collect { orientationState = it }
        }
        viewModelScope.launch {
            sensorRepository.vioState.collect { vio -> handleVioUpdate(vio) }
        }
        viewModelScope.launch {
            sensorRepository.startLocation.collect { startLocation = it }
        }
        viewModelScope.launch {
            sensorRepository.showCameraBlocked.collect { showCameraBlocked = it }
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
                _pathHistory.add(Pair(vio.x.toFloat(), vio.z.toFloat()))
                if (_pathHistory.size > 500) _pathHistory.removeAt(0)
                pathHistoryVersion = _pathHistoryVersion.incrementAndGet()
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
                        strideLength = vio.strideLength, poseFlags = vio.poseFlags, heading = vio.heading
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

            if (nowMs - lastSnapTimeMs > 500) {
                lastSnapTimeMs = nowMs
                viewModelScope.launch(Dispatchers.IO) {
                    val start = startLocation ?: return@launch
                    val currentLatLng = NavSightUtils.metersToLatLng(start, vio.x, vio.z)
                    val recentPath = pathHistory.takeLast(10).map { (x, z) ->
                        NavSightUtils.metersToLatLng(start, x.toDouble(), z.toDouble())
                    }
                    val snapped = roadSnapper.snapToRoad(currentLatLng, recentPath)
                    withContext(Dispatchers.Main) {
                        snappedPosition = snapped.toLatLng()
                        if (navigationState is NavigationState.Active) {
                            navigationManager.updateVioPosition(snapped.toLatLng())
                        }
                    }
                }
            }
        }
    }

    fun toggleSimulationRecording(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        if (!isRecordingSimulation) {
            simulationDataPoints.clear()
            sensorRepository.startGpsUpdates(hasLocationPermission)
            isRecordingSimulation = true
        } else {
            isRecordingSimulation = false
            sensorRepository.stopGpsUpdates()
            saveSimulationData(getExternalFilesDir, filesDir)
        }
    }

    private fun saveSimulationData(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        if (simulationDataPoints.isEmpty()) return
        val snapshot = synchronized(simulationDataPoints) { simulationDataPoints.toList() }
        viewModelScope.launch(Dispatchers.IO) {
            val startTime = snapshot.firstOrNull()?.timestamp ?: System.currentTimeMillis()
            val sb = StringBuilder()
            sb.append("{\"startTime\":$startTime,\"points\":[")
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
                sb.append("\"mflow\":${p.meanFlow},\"inl\":${p.inlierCount},")
                sb.append("\"steps\":${p.stepCount},\"sfreq\":${p.stepFreq},")
                sb.append("\"stride\":${p.strideLength},\"pflags\":${p.poseFlags},")
                sb.append("\"hdg\":${p.heading}}")
            }
            sb.append("]}")
            try {
                val dir = getExternalFilesDir(null) ?: filesDir
                val file = java.io.File(dir, "simulation_data_${System.currentTimeMillis()}.json")
                file.writeText(sb.toString())
            } catch (e: Exception) {
                Log.e("SIMULATION", "Failed to save: ${e.message}")
            }
        }
    }

    fun onResume() {
        sensorRepositoryActive = true   // ← mark BEFORE starting sensors
        sensorRepository.startSensors()
    }

    fun onPause() {
        sensorRepositoryActive = false  // ← mark BEFORE stopping sensors so in-flight frames are dropped
        if (isRecordingSimulation) sensorRepository.stopGpsUpdates()
        sensorRepository.stopSensors()
    }

    fun processCameraFrame(image: ImageProxy) {
        sensorRepository.processCameraFrame(image)
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
        lastVioForDist = null
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
        path.forEachIndexed { idx, (x, z) ->
            if (idx > 0) sb.append(",")
            if (start != null) {
                val latLng = NavSightUtils.metersToLatLng(start, x.toDouble(), z.toDouble())
                sb.append("{\"lat\":${latLng.latitude},\"lng\":${latLng.longitude},\"x\":$x,\"z\":$z}")
            } else sb.append("{\"x\":$x,\"z\":$z}")
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
}