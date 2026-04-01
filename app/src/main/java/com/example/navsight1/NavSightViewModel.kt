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
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.Priority
import com.google.android.gms.maps.model.LatLng
import com.otaliastudios.cameraview.frame.Frame
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlin.math.*

class NavSightViewModel(application: Application) : AndroidViewModel(application) {

    private val sensorRepository = SensorRepository(application)
    private val apiKey = BuildConfig.GOOGLE_MAPS_API_KEY
    private val navigationManager = NavigationManager(application, apiKey)
    private val roadSnapper = RoadSnapper(apiKey = apiKey)

    // UI States (exposed as Compose State)
    var orientationState by mutableStateOf(DeviceOrientationTracker.OrientationResult(
        pitch = 0f, roll = 0f, azimuth = 0f,
        isHorizontal = false, deviationFromHorizontal = 90f, stabilityScore = 0f
    ))
        private set

    // VIO-derived motion state (replaces removed Kotlin OpticalFlowProcessor)
    var isMoving by mutableStateOf(false)
        private set
    var isDriving by mutableStateOf(false)
        private set

    var vioState by mutableStateOf(VioData())
        private set

    var virtualX by mutableStateOf(0.0)
        private set
    var virtualZ by mutableStateOf(0.0)
        private set
    val pathHistory = mutableStateListOf<Pair<Float, Float>>()

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

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    var isRecordingSimulation by mutableStateOf(false)
        private set
    private val simulationDataPoints = mutableListOf<SimulationPoint>()
    private var currentGpsLocation: Location? = null

    data class SimulationPoint(
        val timestamp: Long,
        val vioX: Double,
        val vioY: Double,
        val vioZ: Double,
        val vioYaw: Double,
        val vioScale: Double,
        val vioQuality: Double,
        val rawX: Double,
        val rawY: Double,
        val rawZ: Double,
        val rawYaw: Double,
        val accelX: Float,
        val accelY: Float,
        val accelZ: Float,
        val gyroX: Float,
        val gyroY: Float,
        val gyroZ: Float,
        val gpsLat: Double?,
        val gpsLng: Double?,
        val gpsAlt: Double?,
        val gpsAcc: Float?,
        // Diagnostics
        val meanFlow: Double,
        val inlierCount: Int,
        val stepCount: Int,
        val stepFreq: Double,
        val strideLength: Double,
        val poseFlags: Int,
        val heading: Double
    )

    data class SimulationData(
        val startTime: Long,
        val points: List<SimulationPoint>
    )
    // ──────────────────────────────────────────────────────────────────────────

    /** Initial heading in degrees from SensorRepository (captured at VIO init from magnetometer) */
    val vioInitAzimuth: Float
        get() = sensorRepository.vioInitAzimuth

    private var lastVioForSpeed: VioData? = null
    private var lastVioForDist: VioData? = null
    private var lastSpeedTimeMs = 0L
    private var lastSnapTimeMs = 0L
    
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

    init {
        // Observe Repository states
        viewModelScope.launch {
            sensorRepository.orientationState.collect { orientationState = it }
        }
        viewModelScope.launch {
            sensorRepository.vioState.collect { vio ->
                handleVioUpdate(vio)
            }
        }
        viewModelScope.launch {
            sensorRepository.startLocation.collect { startLocation = it }
        }
        viewModelScope.launch {
            sensorRepository.showCameraBlocked.collect { showCameraBlocked = it }
        }

        // ── FOR SIMULATION ────────────────────────────────────────────────────────
        viewModelScope.launch {
            sensorRepository.currentLocation.collect { currentGpsLocation = it }
        }
        // ──────────────────────────────────────────────────────────────────────────

        // Observe Navigation states
        viewModelScope.launch {
            navigationManager.navigationState.collect { navigationState = it }
        }
        viewModelScope.launch {
            navigationManager.currentInstruction.collect { currentInstruction = it }
        }
    }

    private fun handleVioUpdate(vio: VioData) {
        vioState = vio
        if (vio.isInitialized) {
            virtualX = vio.x
            virtualZ = vio.z
            pathHistory.add(Pair(vio.x.toFloat(), vio.z.toFloat()))
            if (pathHistory.size > 500) pathHistory.removeAt(0)

            // ── FOR SIMULATION ────────────────────────────────────────────────────────
            if (isRecordingSimulation) {
                val gps = currentGpsLocation
                simulationDataPoints.add(SimulationPoint(
                    timestamp = System.currentTimeMillis(),
                    vioX = vio.x, vioY = vio.y, vioZ = vio.z,
                    vioYaw = vio.yaw,
                    vioScale = vio.estimatedScale,
                    vioQuality = vio.trackingQuality,
                    rawX = vio.rawX, rawY = vio.rawY, rawZ = vio.rawZ,
                    rawYaw = vio.rawYaw,
                    accelX = vio.accelX, accelY = vio.accelY, accelZ = vio.accelZ,
                    gyroX = vio.gyroX, gyroY = vio.gyroY, gyroZ = vio.gyroZ,
                    gpsLat = gps?.latitude,
                    gpsLng = gps?.longitude,
                    gpsAlt = gps?.altitude,
                    gpsAcc = gps?.accuracy,
                    meanFlow = vio.meanFlow,
                    inlierCount = vio.inlierCount,
                    stepCount = vio.stepCount,
                    stepFreq = vio.stepFreq,
                    strideLength = vio.strideLength,
                    poseFlags = vio.poseFlags,
                    heading = vio.heading
                ))
            }
            // ──────────────────────────────────────────────────────────────────────────

            // Distance accumulation (every frame)
            val prevDist = lastVioForDist
            if (prevDist != null) {
                val ddx = vio.x - prevDist.x
                val ddz = vio.z - prevDist.z
                totalDistanceM += sqrt(ddx * ddx + ddz * ddz)
            }
            lastVioForDist = vio

            // Speed computation (throttled to 200ms)
            val nowMs = System.currentTimeMillis()
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

            // Road snapping
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

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    fun toggleSimulationRecording(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        if (!isRecordingSimulation) {
            // Start recording
            simulationDataPoints.clear()
            sensorRepository.startGpsUpdates()
            isRecordingSimulation = true
            Log.d("SIMULATION", "Started recording simulation")
        } else {
            // Stop and save
            isRecordingSimulation = false
            sensorRepository.stopGpsUpdates()
            saveSimulationData(getExternalFilesDir, filesDir)
            Log.d("SIMULATION", "Stopped recording simulation, points: ${simulationDataPoints.size}")
        }
    }

    private fun saveSimulationData(getExternalFilesDir: (String?) -> java.io.File?, filesDir: java.io.File) {
        if (simulationDataPoints.isEmpty()) return
        
        // Take a snapshot of the data immediately on the current thread (likely Main)
        // to avoid ConcurrentModificationException while the camera thread is still active.
        val snapshot = synchronized(simulationDataPoints) {
            simulationDataPoints.toList()
        }
        
        viewModelScope.launch(Dispatchers.IO) {
            val startTime = snapshot.firstOrNull()?.timestamp ?: System.currentTimeMillis()
            
            // Simple manual JSON conversion to avoid adding heavy libraries
            val sb = StringBuilder()
            sb.append("{\"startTime\":$startTime,\"points\":[")
            snapshot.forEachIndexed { index, p ->
                if (index > 0) sb.append(",")
                sb.append("{")
                sb.append("\"ts\":${p.timestamp},")
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
                sb.append("\"hdg\":${p.heading}")
                sb.append("}")
            }
            sb.append("]}")

            try {
                val dir = getExternalFilesDir(null) ?: filesDir
                val file = java.io.File(dir, "simulation_data_${System.currentTimeMillis()}.json")
                file.writeText(sb.toString())
                Log.d("SIMULATION", "Saved simulation data to: ${file.absolutePath}")
            } catch (e: Exception) {
                Log.e("SIMULATION", "Failed to save simulation data: ${e.message}")
            }
        }
    }
    // ──────────────────────────────────────────────────────────────────────────

    fun onResume() {
        sensorRepository.startSensors()
    }

    fun onPause() {
        if (isRecordingSimulation) {
            // Stop GPS updates if we leave the app
            sensorRepository.stopGpsUpdates()
        }
        sensorRepository.stopSensors()
    }

    fun processCameraFrame(frame: Frame) {
        sensorRepository.processCameraFrame(frame)
    }

    fun requestInitialLocation() {
        sensorRepository.requestInitialLocation()
    }

    fun resetPath() {
        sensorRepository.resetPath()
        pathHistory.clear()
        virtualX = 0.0
        virtualZ = 0.0
        currentSpeedKmh = 0f
        totalDistanceM = 0.0
        lastVioForSpeed = null
        lastVioForDist = null
    }

    fun startNavigation(destination: LatLng) {
        viewModelScope.launch {
            val currentPos = snappedPosition ?: startLocation ?: LatLng(32.0853, 34.7818)
            navigationManager.startNavigation(currentPos, destination)
        }
    }

    fun stopNavigation() {
        navigationManager.cancelNavigation()
    }

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
            } else {
                sb.append("{\"x\":$x,\"z\":$z}")
            }
        }
        sb.append("]}")
        try {
            val dir = getExternalFilesDir(null) ?: filesDir
            val file = java.io.File(dir, "navsight_path_${System.currentTimeMillis()}.json")
            file.writeText(sb.toString())
        } catch (e: Exception) {
            // Log error
        }
    }

    override fun onCleared() {
        super.onCleared()
        roadSnapper.shutdown()
    }
}
