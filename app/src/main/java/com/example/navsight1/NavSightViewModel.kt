package com.example.navsight1

import android.app.Application
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

    var flowResultState by mutableStateOf(OpticalFlowProcessor.FlowResult(
        dx = 0f, dy = 0f, magnitude = 0f,
        direction = OpticalFlowProcessor.MovementDirection.STOPPED,
        confidence = 0f,
        mode = OpticalFlowProcessor.MovementMode.WALKING
    ))
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

    var showCameraBlocked by mutableStateOf(false)
        private set

    /** Initial heading in degrees from SensorRepository (captured at VIO init from magnetometer) */
    val vioInitAzimuth: Float
        get() = sensorRepository.vioInitAzimuth

    private var lastVioForSpeed: VioData? = null
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
            sensorRepository.flowResultState.collect { flowResultState = it }
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

            // Speed computation
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

    fun onResume() {
        sensorRepository.startSensors()
    }

    fun onPause() {
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
        lastVioForSpeed = null
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
                val lat = start.latitude + z / 111111.0
                val cosLat = cos(Math.toRadians(start.latitude))
                val lng = start.longitude + if (cosLat > 1e-10) x / (111111.0 * cosLat) else 0.0
                sb.append("{\"lat\":$lat,\"lng\":$lng,\"x\":$x,\"z\":$z}")
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
