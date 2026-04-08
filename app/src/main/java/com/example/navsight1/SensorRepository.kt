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
import androidx.camera.core.ImageProxy
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
    // DISABLED: DepthEstimator — MiDaS depth feeds Mapper pipeline which is disabled (output discarded)
    // private val depthEstimator = DepthEstimator(context)

    // Dedicated VIO processing thread — decouples camera preview from VIO computation.
    // Frame dropping: if VIO is still busy when the next frame arrives, we skip it.
    private val vioExecutor = java.util.concurrent.Executors.newSingleThreadExecutor { r ->
        Thread(r, "NavSight-VIO").apply { isDaemon = true }
    }
    @Volatile private var vioProcessing = false
    @Volatile private var vioFrameCount = 0

    // DISABLED: Depth estimation thread + state — DepthEstimator/Mapper pipeline disabled
    // private val depthExecutor = java.util.concurrent.Executors.newSingleThreadExecutor { r ->
    //     Thread(r, "NavSight-Depth").apply { isDaemon = true }
    // }
    // @Volatile private var depthProcessing = false
    // private var lastDepthTimeMs = 0L
    // private val DEPTH_THROTTLE_MS = 200L // 5Hz depth updates

    private val _orientationState = MutableStateFlow(DeviceOrientationTracker.OrientationResult(
        pitch = 0f, roll = 0f, azimuth = 0f,
        isHorizontal = false, deviationFromHorizontal = 90f, stabilityScore = 0f
    ))
    val orientationState = _orientationState.asStateFlow()

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

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    private val _currentLocation = MutableStateFlow<Location?>(null)
    val currentLocation = _currentLocation.asStateFlow()
    private var locationCallback: LocationCallback? = null
    // ──────────────────────────────────────────────────────────────────────────

    private val repositoryScope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    fun startSensors() {
        try {
            accelerometer?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
                Log.d(TAG, "Accelerometer registered")
            } ?: Log.w(TAG, "Accelerometer not available on this device")
            
            magnetometer?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
                Log.d(TAG, "Magnetometer registered")
            } ?: Log.w(TAG, "Magnetometer not available on this device")
            
            gyroscope?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
                Log.d(TAG, "Gyroscope registered")
            } ?: Log.w(TAG, "Gyroscope not available on this device")
            
            if (NativeBridge.isLoaded()) {
                NativeBridge.startVIO()
            } else {
                Log.e(TAG, "Cannot start VIO: native library not loaded")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error starting sensors: ${e.message}", e)
        }
    }

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    fun startGpsUpdates(granted: Boolean = false) {
        if (!granted) {
            Log.w(TAG, "Cannot start GPS updates: location permission not granted")
            return
        }
        
        try {
            val request = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 1000L)
                .setMinUpdateIntervalMillis(500L)
                .build()
            
            locationCallback = object : LocationCallback() {
                override fun onLocationResult(result: LocationResult) {
                    result.lastLocation?.let {
                        _currentLocation.value = it
                        // Also update start location if not set
                        if (_startLocation.value == null) {
                            _startLocation.value = LatLng(it.latitude, it.longitude)
                        }
                    }
                }
            }
            
            locationCallback?.let {
                // Use safe location request with proper permission checks
                if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                    fusedLocationClient.requestLocationUpdates(
                        request, it, android.os.Looper.getMainLooper()
                    )
                } else {
                    @Suppress("MissingPermission")
                    fusedLocationClient.requestLocationUpdates(
                        request, it, android.os.Looper.getMainLooper()
                    )
                }
            }
            Log.d(TAG, "GPS updates started for simulation")
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException starting GPS updates: ${e.message}")
        } catch (e: Exception) {
            Log.e(TAG, "Error starting GPS updates: ${e.message}", e)
        }
    }

    fun stopGpsUpdates() {
        locationCallback?.let {
            fusedLocationClient.removeLocationUpdates(it)
        }
        locationCallback = null
        Log.d(TAG, "GPS updates stopped")
    }
    // ──────────────────────────────────────────────────────────────────────────

    private var intrinsicsInitialized = false

    private fun getCameraIntrinsics(targetWidth: Int, targetHeight: Int): FloatArray {
        try {
            val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as android.hardware.camera2.CameraManager
            val cameraId = cameraManager.cameraIdList.firstOrNull { id ->
                val chars = cameraManager.getCameraCharacteristics(id)
                chars.get(android.hardware.camera2.CameraCharacteristics.LENS_FACING) == android.hardware.camera2.CameraCharacteristics.LENS_FACING_BACK
            } ?: cameraManager.cameraIdList[0]

            val characteristics = cameraManager.getCameraCharacteristics(cameraId)
            val focalLengths = characteristics.get(android.hardware.camera2.CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)
            val sensorSize = characteristics.get(android.hardware.camera2.CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
            val pixelArraySize = characteristics.get(android.hardware.camera2.CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)

            if (focalLengths != null && focalLengths.isNotEmpty() && sensorSize != null && pixelArraySize != null) {
                val focalLengthMm = focalLengths[0]
                val sensorWidthMm = sensorSize.width
                val sensorHeightMm = sensorSize.height
                val fullPixelWidth = pixelArraySize.width.toFloat()
                val fullPixelHeight = pixelArraySize.height.toFloat()

                // Calculate intrinsics for full sensor resolution
                val fxFull = (focalLengthMm / sensorWidthMm) * fullPixelWidth
                val fyFull = (focalLengthMm / sensorHeightMm) * fullPixelHeight
                val cxFull = fullPixelWidth / 2f
                val cyFull = fullPixelHeight / 2f

                // Scale to target (preview) resolution
                val scaleX = targetWidth.toFloat() / fullPixelWidth
                val scaleY = targetHeight.toFloat() / fullPixelHeight

                val fx = fxFull * scaleX
                val fy = fyFull * scaleY
                val cx = cxFull * scaleX
                val cy = cyFull * scaleY

                Log.d(TAG, "Camera intrinsics scaled: fx=$fx fy=$fy cx=$cx cy=$cy (target=${targetWidth}x${targetHeight})")
                return floatArrayOf(fx, fy, cx, cy)
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to get camera intrinsics: ${e.message}")
        }
        return floatArrayOf(0f, 0f, 0f, 0f)
    }

    fun stopSensors() {
        try {
            sensorManager.unregisterListener(this)
            Log.d(TAG, "All sensors unregistered")
        } catch (e: Exception) {
            Log.e(TAG, "Error unregistering listeners: ${e.message}")
        }
        
        try {
            if (NativeBridge.isLoaded()) {
                NativeBridge.stopVIO()
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping VIO: ${e.message}")
        }
        
        locationTokenSource?.cancel()
        stopGpsUpdates()
        vioExecutor.shutdown()
        // DISABLED: depth cleanup — DepthEstimator/Mapper pipeline disabled
        // depthExecutor.shutdown()
        // depthEstimator.close()
        intrinsicsInitialized = false
        repositoryScope.cancel()
        Log.d(TAG, "Repository cleaned up")
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
            val orientation = orientationTracker.getOrientation()
            _orientationState.value = orientation
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    fun processCameraFrame(image: ImageProxy) {
        // Drop frame if VIO is still processing the previous one.
        if (vioProcessing) {
            image.close()
            return
        }

        val w = image.width
        val h = image.height

        // Initialize intrinsics once
        if (!intrinsicsInitialized) {
            val intrinsics = getCameraIntrinsics(w, h)
            NativeBridge.setIntrinsics(
                intrinsics[0].toDouble(), intrinsics[1].toDouble(),
                intrinsics[2].toDouble(), intrinsics[3].toDouble()
            )
            intrinsicsInitialized = true
        }

        // Get direct ByteBuffers from ImageProxy planes (zero-copy)
        val yPlane = image.planes[0]
        val uvPlane = image.planes[2] // V plane — for NV21, this is the interleaved VU plane
        val yBuffer = yPlane.buffer
        val uvBuffer = uvPlane.buffer
        val yRowStride = yPlane.rowStride
        val uvRowStride = uvPlane.rowStride
        val uvPixelStride = uvPlane.pixelStride

        val timestampNs = image.imageInfo.timestamp
        val nowMs = System.currentTimeMillis()

        // 1. VIO processing — pass direct buffers to JNI (near zero-copy)
        vioProcessing = true
        val frameStartMs = nowMs
        vioExecutor.execute {
            try {
                val jniStartMs = System.currentTimeMillis()
                val vio = NativeBridge.processCameraFrameDirect(
                    yBuffer = yBuffer,
                    uvBuffer = uvBuffer,
                    width = w,
                    height = h,
                    yRowStride = yRowStride,
                    uvRowStride = uvRowStride,
                    uvPixelStride = uvPixelStride,
                    timestamp = timestampNs
                )
                val jniMs = System.currentTimeMillis() - jniStartMs
                val totalMs = System.currentTimeMillis() - frameStartMs
                vioFrameCount++
                if (vioFrameCount % 30 == 0) {
                    Log.i(TAG, "VIO_FPS: jni=${jniMs}ms total=${totalMs}ms fc=$vioFrameCount pts=${vio.trackedFeatures}")
                }
                _vioState.value = vio
                handleVioInitialized(vio)
                checkCameraBlocked(vio)
            } finally {
                image.close()  // Release ImageProxy AFTER JNI is done
                vioProcessing = false
            }
        }

        // DISABLED: Depth estimation — Mapper pipeline disabled, MiDaS output was never applied
        // if (!depthProcessing && (nowMs - lastDepthTimeMs >= DEPTH_THROTTLE_MS)) {
        //     val yBytes = ByteArray(w * h)
        //     yBuffer.rewind()
        //     if (yRowStride == w) {
        //         yBuffer.get(yBytes, 0, w * h)
        //     } else {
        //         for (row in 0 until h) {
        //             yBuffer.position(row * yRowStride)
        //             yBuffer.get(yBytes, row * w, w)
        //         }
        //     }
        //     depthProcessing = true
        //     lastDepthTimeMs = nowMs
        //     depthExecutor.execute {
        //         try {
        //             val bitmap = android.graphics.Bitmap.createBitmap(w, h, android.graphics.Bitmap.Config.ARGB_8888)
        //             val pixels = IntArray(w * h)
        //             for (i in yBytes.indices) {
        //                 val lum = yBytes[i].toInt() and 0xFF
        //                 pixels[i] = (0xFF shl 24) or (lum shl 16) or (lum shl 8) or lum
        //             }
        //             bitmap.setPixels(pixels, 0, w, 0, 0, w, h)
        //             repositoryScope.launch {
        //                 val depthMap = depthEstimator.estimateDepth(bitmap)
        //                 if (depthMap != null) {
        //                     NativeBridge.setDepthMap(depthMap, 256, 256)
        //                 }
        //             }
        //         } catch (e: Exception) {
        //             Log.e(TAG, "Depth processing error: ${e.message}")
        //         } finally {
        //             depthProcessing = false
        //         }
        //     }
        // }
    }

    private fun handleVioInitialized(vio: VioData) {
        if (vio.isInitialized && !wasVioInitialized) {
            wasVioInitialized = true
            vioInitAzimuth = _orientationState.value.azimuth
            val azimuthRad = Math.toRadians(vioInitAzimuth.toDouble())
            NativeBridge.setInitialHeading(azimuthRad)
            Log.i(TAG, "Initial heading set: ${vioInitAzimuth}° (${azimuthRad} rad)")

            magnetometer?.let {
                sensorManager.unregisterListener(this, it)
                Log.i(TAG, "Magnetometer unregistered - initial heading captured.")
            }
        }
    }

    private fun checkCameraBlocked(vio: VioData) {
        if (vio.isInitialized && vio.trackedFeatures < 5) {
            consecutiveVioFailures++
            if (consecutiveVioFailures > 30) _showCameraBlocked.value = true
        } else {
            consecutiveVioFailures = 0
            _showCameraBlocked.value = false
        }
    }

    fun requestInitialLocation(granted: Boolean = false) {
        if (!granted) {
            Log.w(TAG, "Cannot request location: permission not granted")
            return
        }
        
        try {
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                fusedLocationClient.lastLocation.addOnSuccessListener { last ->
                    if (last != null && _startLocation.value == null) {
                        _startLocation.value = LatLng(last.latitude, last.longitude)
                    }
                }
            } else {
                @Suppress("MissingPermission")
                fusedLocationClient.lastLocation.addOnSuccessListener { last ->
                    if (last != null && _startLocation.value == null) {
                        _startLocation.value = LatLng(last.latitude, last.longitude)
                    }
                }
            }

            locationTokenSource?.cancel()
            val tokenSource = CancellationTokenSource()
            locationTokenSource = tokenSource
            
            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                fusedLocationClient.getCurrentLocation(
                    Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token
                )
                    .addOnSuccessListener { location ->
                        if (location != null) {
                            _startLocation.value = LatLng(location.latitude, location.longitude)
                        }
                        locationTokenSource = null
                    }
                    .addOnFailureListener {
                        Log.w(TAG, "Failed to get current location")
                        locationTokenSource = null
                    }
            } else {
                @Suppress("MissingPermission")
                fusedLocationClient.getCurrentLocation(
                    Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token
                )
                    .addOnSuccessListener { location ->
                        if (location != null) {
                            _startLocation.value = LatLng(location.latitude, location.longitude)
                        }
                        locationTokenSource = null
                    }
                    .addOnFailureListener {
                        Log.w(TAG, "Failed to get current location")
                        locationTokenSource = null
                    }
            }
        } catch (e: SecurityException) {
            Log.e(TAG, "SecurityException requesting location: ${e.message}")
        } catch (e: Exception) {
            Log.e(TAG, "Error requesting location: ${e.message}", e)
        }
    }

    fun resetPath() {
        NativeBridge.resetVIO()
        _vioState.value = VioData()
        orientationTracker.reset()
        wasVioInitialized = false
        vioInitAzimuth = 0f
        consecutiveVioFailures = 0
        _showCameraBlocked.value = false
        accelMagnitudeHistory.clear()
    }

    // DEAD CODE: never called — ViewModel calls NativeBridge.setScale() directly
    // fun setScale(scale: Double) {
    //     NativeBridge.setScale(scale)
    // }
}
