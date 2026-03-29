package com.example.navsight1

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.util.Log
import com.google.android.gms.location.*
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.tasks.CancellationTokenSource
import com.otaliastudios.cameraview.frame.Frame
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.sqrt

class SensorRepository(private val context: Context) : SensorEventListener {

    private val TAG = "SensorRepository"

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val fusedLocationClient = LocationServices.getFusedLocationProviderClient(context)

    private val orientationTracker = DeviceOrientationTracker()
    private val opticalFlowProcessor = OpticalFlowProcessor()

    private val _orientationState = MutableStateFlow(DeviceOrientationTracker.OrientationResult(
        pitch = 0f, roll = 0f, azimuth = 0f,
        isHorizontal = false, deviationFromHorizontal = 90f, stabilityScore = 0f
    ))
    val orientationState = _orientationState.asStateFlow()

    private val _flowResultState = MutableStateFlow(OpticalFlowProcessor.FlowResult(
        dx = 0f, dy = 0f, magnitude = 0f,
        direction = OpticalFlowProcessor.MovementDirection.STOPPED,
        confidence = 0f,
        mode = OpticalFlowProcessor.MovementMode.WALKING
    ))
    val flowResultState = _flowResultState.asStateFlow()

    private val _vioState = MutableStateFlow(VioData())
    val vioState = _vioState.asStateFlow()

    private val _startLocation = MutableStateFlow<LatLng?>(null)
    val startLocation = _startLocation.asStateFlow()

    private var lastOrientationUpdateNs = 0L
    private val accelMagnitudeHistory = ArrayDeque<Float>(20)
    private var lastShakeCheckMs = 0L

    private var locationTokenSource: CancellationTokenSource? = null

    // VIO Init state
    var wasVioInitialized = false
        private set
    var vioInitAzimuth = 0f
        private set

    // Camera Blocked
    private var consecutiveVioFailures = 0
    private val _showCameraBlocked = MutableStateFlow(false)
    val showCameraBlocked = _showCameraBlocked.asStateFlow()

    private val repositoryScope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    fun startSensors() {
        accelerometer?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        magnetometer?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        gyroscope?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        NativeBridge.startVIO()
        NativeBridge.setIntrinsics(0.0, 0.0, 0.0, 0.0)
    }

    fun stopSensors() {
        sensorManager.unregisterListener(this)
        NativeBridge.stopVIO()
        locationTokenSource?.cancel()
    }

    override fun onSensorChanged(event: SensorEvent) {
        val ts = event.timestamp
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                orientationTracker.updateAccelerometer(event.values)
                NativeBridge.processAccelerometer(ts, event.values[0], event.values[1], event.values[2])

                val mag = sqrt(
                    event.values[0] * event.values[0] +
                            event.values[1] * event.values[1] +
                            event.values[2] * event.values[2]
                )
                if (accelMagnitudeHistory.size >= 20) accelMagnitudeHistory.removeFirst()
                accelMagnitudeHistory.addLast(mag)
                val nowMs = System.currentTimeMillis()
                if (nowMs - lastShakeCheckMs > 500 && accelMagnitudeHistory.size >= 10) {
                    lastShakeCheckMs = nowMs
                    val mean = accelMagnitudeHistory.average()
                    val variance = accelMagnitudeHistory.sumOf { v ->
                        (v - mean) * (v - mean)
                    } / accelMagnitudeHistory.size
                    if (variance > 30.0) {
                        resetPath()
                    }
                }
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                orientationTracker.updateMagnetometer(event.values)
            }
            Sensor.TYPE_GYROSCOPE -> {
                NativeBridge.processGyroscope(ts, event.values[0], event.values[1], event.values[2])
            }
        }

        if (ts - lastOrientationUpdateNs >= 50_000_000L) {
            lastOrientationUpdateNs = ts
            _orientationState.value = orientationTracker.getOrientation()
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    fun processCameraFrame(frame: Frame) {
        val data = frame.getData<ByteArray>() ?: return

        val vio = NativeBridge.processCameraFrame(
            frameData = data,
            width = frame.size.width,
            height = frame.size.height,
            timestamp = frame.time
        )

        val flowResult = opticalFlowProcessor.processFrame(
            frame = data,
            width = frame.size.width,
            height = frame.size.height,
            isYuv = true
        )

        _vioState.value = vio
        _flowResultState.value = flowResult

        if (vio.isInitialized && !wasVioInitialized) {
            wasVioInitialized = true
            vioInitAzimuth = _orientationState.value.azimuth
            
            // FR17: Magnetometer is only needed for initial direction. 
            // Unregister it now to prevent interference and save battery.
            magnetometer?.let {
                sensorManager.unregisterListener(this, it)
                Log.d(TAG, "Magnetometer unregistered - initial heading captured.")
            }
        }

        if (vio.isInitialized && vio.trackedFeatures < 5) {
            consecutiveVioFailures++
            if (consecutiveVioFailures > 30) _showCameraBlocked.value = true
        } else {
            consecutiveVioFailures = 0
            _showCameraBlocked.value = false
        }
    }

    @SuppressLint("MissingPermission")
    fun requestInitialLocation() {
        fusedLocationClient.lastLocation.addOnSuccessListener { last ->
            if (last != null && _startLocation.value == null) {
                _startLocation.value = LatLng(last.latitude, last.longitude)
            }
        }

        locationTokenSource?.cancel()
        val tokenSource = CancellationTokenSource()
        locationTokenSource = tokenSource
        fusedLocationClient.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token)
            .addOnSuccessListener { location ->
                if (location != null) {
                    _startLocation.value = LatLng(location.latitude, location.longitude)
                }
                locationTokenSource = null
            }
            .addOnFailureListener {
                locationTokenSource = null
            }
    }

    fun resetPath() {
        NativeBridge.resetVIO()
        _vioState.value = VioData()
        opticalFlowProcessor.reset()
        orientationTracker.reset()
        wasVioInitialized = false
        vioInitAzimuth = 0f
        consecutiveVioFailures = 0
        _showCameraBlocked.value = false
        accelMagnitudeHistory.clear()
    }

    fun setScale(scale: Double) {
        NativeBridge.setScale(scale)
    }
}
