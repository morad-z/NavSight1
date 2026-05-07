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

// Plan Step 7 (ADR-013): on-device filename for the ORB DBoW2 vocabulary.
// SensorRepository.pushLoopClosureVocabularyToNative copies the ORB
// vocabulary asset on first launch and points native at the file.
//
// We ship ORBvoc.txt.gz in app/src/main/assets/, but the Android
// Gradle Plugin auto-decompresses .gz assets at packaging time and
// stores them in the APK as plain text (verified 2026-05-04: APK
// contains `assets/ORBvoc.txt` at 145 MB, despite the source-tree
// asset being ORBvoc.txt.gz at 41 MB). The on-disk APK is still
// zip-deflated so the install size stays reasonable; only the
// runtime read sees plain text.
//
// At runtime we therefore open `ORBvoc.txt` directly from assets and
// stream it to <filesDir>/ORBvoc.txt. No GZIPInputStream needed.
// DBoW2's loadFromTextFile reads the result.
const val ORB_VOCAB_ASSET = "ORBvoc.txt"
const val ORB_VOCAB_FILE  = "ORBvoc.txt"

class SensorRepository(private val context: Context) : SensorEventListener {

    private val TAG = "SensorRepository"

    // Step 5: VIO initialization status (mirrors C++ InertialInitializer::Status).
    // Order MUST match native enum: WAIT_STATIONARY=0, WAIT_MOTION=1, READY=2, TIMEOUT_NEEDS_USER=3.
    enum class InitStatus { WAIT_STATIONARY, WAIT_MOTION, READY, TIMEOUT_NEEDS_USER }

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val fusedLocationClient = LocationServices.getFusedLocationProviderClient(context)

    private val orientationTracker = DeviceOrientationTracker()
    // MiDaS depth feeds Tracker scale constraint (bypasses Mapper)
    private val depthEstimator = DepthEstimator(context)

    // Dedicated VIO processing thread — decouples camera preview from VIO computation.
    // Frame dropping: if VIO is still busy when the next frame arrives, we skip it.
    private val vioExecutor = java.util.concurrent.Executors.newSingleThreadExecutor { r ->
        Thread(r, "NavSight-VIO").apply { isDaemon = true }
    }
    @Volatile private var vioProcessing = false
    @Volatile private var vioFrameCount = 0

    // Step 8c: rolling-shutter row read-out time from Camera2.
    // Updated by CameraUi.kt Camera2Interop CaptureCallback on every frame.
    // Camera2 API: CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW (API level 21+) —
    // nanoseconds from first-row to last-row read-out. 0 = skew unavailable.
    @Volatile var rollingShutterSkewNs: Long = 0L
        private set

    fun updateRollingShutterSkew(skewNs: Long) {
        rollingShutterSkewNs = skewNs
    }

    // Depth estimation at ~1Hz for scale constraint
    private val depthExecutor = java.util.concurrent.Executors.newSingleThreadExecutor { r ->
        Thread(r, "NavSight-Depth").apply { isDaemon = true }
    }
    @Volatile private var depthProcessing = false
    private var lastDepthTimeMs = 0L
    private val DEPTH_THROTTLE_MS = 1000L // 1Hz depth — conservative for battery

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

    // Step 5: Initialization-gate status. Polled from native InertialInitializer.
    private val _initStatus = MutableStateFlow(InitStatus.WAIT_STATIONARY)
    val initStatus = _initStatus.asStateFlow()
    private var initStatusPollerJob: Job? = null
    // True once we have persisted a fresh calibration during this run, so we don't
    // repeatedly re-write the same values to SharedPreferences.
    private var calibrationPersistedThisRun = false

    // ── FOR SIMULATION ────────────────────────────────────────────────────────
    private val _currentLocation = MutableStateFlow<Location?>(null)
    val currentLocation = _currentLocation.asStateFlow()
    private var locationCallback: LocationCallback? = null
    // ──────────────────────────────────────────────────────────────────────────

    private var repositoryScope = CoroutineScope(Dispatchers.Main + SupervisorJob())

    fun startSensors() {
        // Recreate scope if a previous stopSensors() cancelled it. Idempotent.
        if (!repositoryScope.isActive) {
            repositoryScope = CoroutineScope(Dispatchers.Main + SupervisorJob())
        }
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
                pushStoredCalibrationToNative()
                pushCameraIntrinsicsToNative()
                pushLoopClosureVocabularyToNative()
                pushExtrinsicsRotationToNative()   // Step 8b
                startInitStatusPoller()
            } else {
                Log.e(TAG, "Cannot start VIO: native library not loaded")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error starting sensors: ${e.message}", e)
        }
    }

    /**
     * Plan Step 1b/8b: load `<filesDir>/camera_calib.json` (written by the
     * in-app calibration screen) and push fx/fy/cx/cy + distortion into
     * the native runtime via NativeBridge.nativeLoadCalibration.
     *
     * Called immediately after startVIO() so the loader runs before the
     * first camera frame arrives and before SensorRepository's lazy
     * setIntrinsics fallback kicks in. Without this, the saved JSON sits
     * on disk unused and LensCorrector stays at zero-distortion.
     *
     * No-op if the file doesn't exist (user hasn't calibrated yet).
     */
    private fun pushCameraIntrinsicsToNative() {
        val file = java.io.File(context.filesDir, CALIB_FILE)
        if (!file.exists()) {
            Log.i(TAG, "No camera_calib.json — running with default zero-distortion intrinsics")
            return
        }
        try {
            val ok = NativeBridge.nativeLoadCalibration(file.absolutePath)
            if (ok) {
                calibratedIntrinsicsLoaded = true
                intrinsicsInitialized = true   // suppress first-frame default push
                Log.i(TAG, "Camera intrinsics loaded from ${file.absolutePath}")
            } else {
                Log.w(TAG, "nativeLoadCalibration returned false for ${file.absolutePath}")
            }
        } catch (e: Throwable) {
            Log.e(TAG, "Failed to load camera intrinsics: ${e.message}", e)
        }
    }

    /**
     * Plan Step 7 (ADR-013): copy the ORB DBoW2 vocabulary out of
     * `assets/ORBvoc.bin` into `<filesDir>/ORBvoc.bin` if not already
     * present, then push its absolute path into the native runtime via
     * NativeBridge.nativeLoadLoopClosureVocabulary.
     *
     * Why the copy: Android AssetManager does not give a real filesystem
     * path. DBoW2 / cv::FileStorage need a path they can fopen, so we
     * stream the asset to internal storage exactly once per install.
     * Subsequent app launches see the file already present and skip
     * the copy.
     *
     * No-op if the asset doesn't ship with this build (returns silently;
     * loop closure stays disabled). Failures here MUST NOT block the
     * camera path — caller catches Throwable and logs.
     */
    private fun pushLoopClosureVocabularyToNative() {
        try {
            val outFile = java.io.File(context.filesDir, ORB_VOCAB_FILE)

            // One-time copy. AGP auto-decompresses .gz assets at
            // packaging time, so the APK ships plain ORBvoc.txt — read
            // it straight through to <filesDir>/ORBvoc.txt. No
            // decompression layer at runtime. If the asset is missing
            // entirely we silently skip — loop closure is an optional
            // accuracy boost, not a hard requirement for the rest of
            // the VIO pipeline.
            if (!outFile.exists() || outFile.length() == 0L) {
                val assetExists = try {
                    context.assets.list("")?.contains(ORB_VOCAB_ASSET) == true
                } catch (_: Throwable) { false }
                if (!assetExists) {
                    Log.i(TAG, "No assets/$ORB_VOCAB_ASSET — loop closure disabled")
                    return
                }
                context.assets.open(ORB_VOCAB_ASSET).use { input ->
                    java.io.FileOutputStream(outFile).use { output ->
                        input.copyTo(output)
                    }
                }
                Log.i(TAG, "Copied $ORB_VOCAB_ASSET to ${outFile.absolutePath} (${outFile.length()} bytes)")
            }

            val ok = NativeBridge.nativeLoadLoopClosureVocabulary(outFile.absolutePath)
            if (ok) {
                Log.i(TAG, "Loop-closure vocabulary loaded from ${outFile.absolutePath}")
            } else {
                Log.w(TAG, "nativeLoadLoopClosureVocabulary returned false for ${outFile.absolutePath} — loop closure stays disabled")
            }
        } catch (e: Throwable) {
            Log.e(TAG, "Failed to push loop-closure vocabulary: ${e.message}", e)
        }
    }

    /**
     * Step 8b: read the back-camera SENSOR_ORIENTATION from Android
     * CameraCharacteristics and seed the EKF's R_bc with the corresponding
     * rotation matrix.
     *
     * Android convention: SENSOR_ORIENTATION is the angle (0, 90, 180, 270)
     * that the camera sensor image must be rotated CW to align with the device's
     * natural portrait orientation.  For most rear cameras this is 90°.
     *
     * Our body frame has +X forward (device top), +Y left, +Z up (screen face).
     * The default R_bc = diag(1,-1,-1) encodes the camera-to-body flip assumed
     * when SENSOR_ORIENTATION = 0.  For a 90° rotated sensor we compose an
     * additional Rz(-90°) on the right (rotating in the camera's own frame):
     *
     *   R_bc(θ) = R_bc_default  ×  Rz(−θ)
     *
     * where θ = SENSOR_ORIENTATION in radians.  The minus sign comes from the
     * fact that SENSOR_ORIENTATION is a CW rotation of the image, which
     * corresponds to a CCW rotation of the coordinate frame.
     *
     * If CameraCharacteristics is unavailable the method logs a warning and
     * returns without calling native — the EKF keeps its compile-time default.
     */
    private fun pushExtrinsicsRotationToNative() {
        try {
            val cameraManager = context.getSystemService(Context.CAMERA_SERVICE)
                as android.hardware.camera2.CameraManager
            val cameraId = cameraManager.cameraIdList.firstOrNull { id ->
                val chars = cameraManager.getCameraCharacteristics(id)
                chars.get(android.hardware.camera2.CameraCharacteristics.LENS_FACING) ==
                    android.hardware.camera2.CameraCharacteristics.LENS_FACING_BACK
            } ?: run {
                Log.w(TAG, "pushExtrinsicsRotationToNative: no back camera found — keeping default R_bc")
                return
            }

            val chars = cameraManager.getCameraCharacteristics(cameraId)
            val sensorOrientation = chars.get(
                android.hardware.camera2.CameraCharacteristics.SENSOR_ORIENTATION
            ) ?: run {
                Log.w(TAG, "pushExtrinsicsRotationToNative: SENSOR_ORIENTATION null — keeping default R_bc")
                return
            }

            // R_bc_default = diag(1,-1,-1)  (matches the C++ compile-time init)
            // Rz(-θ) where θ = sensorOrientation degrees (CW sensor rotation → CCW frame rotation)
            val thetaRad = Math.toRadians(-sensorOrientation.toDouble())
            val cosT = Math.cos(thetaRad).toFloat()
            val sinT = Math.sin(thetaRad).toFloat()

            // Rz(-θ), row-major:
            //  [ cos  -sin  0 ]
            //  [ sin   cos  0 ]
            //  [  0     0   1 ]
            val rz = floatArrayOf(
                 cosT, -sinT, 0f,
                 sinT,  cosT, 0f,
                 0f,    0f,   1f
            )

            // R_bc_default row-major: [1,0,0, 0,-1,0, 0,0,-1]
            // R_bc = R_bc_default * Rz(-θ)  (matrix multiply 3×3)
            val def = floatArrayOf(
                1f,  0f,  0f,
                0f, -1f,  0f,
                0f,  0f, -1f
            )
            val R_bc = FloatArray(9)
            for (row in 0..2) {
                for (col in 0..2) {
                    var sum = 0f
                    for (k in 0..2) {
                        sum += def[row * 3 + k] * rz[k * 3 + col]
                    }
                    R_bc[row * 3 + col] = sum
                }
            }

            NativeBridge.nativeSetExtrinsicsRotation(R_bc)
            Log.i(TAG, "Extrinsics R_bc seeded from SENSOR_ORIENTATION=${sensorOrientation}°")
        } catch (e: Throwable) {
            Log.e(TAG, "pushExtrinsicsRotationToNative failed: ${e.message}", e)
        }
    }

    /** Step 5: load any previously-saved calibration and push it into the native gate. */
    private fun pushStoredCalibrationToNative() {
        val stored = CalibrationStore.load(context) ?: return
        try {
            NativeBridge.loadStoredCalibration(
                stored.rotation, stored.gyroBias, stored.accelBias
            )
            Log.i(
                TAG,
                "Loaded stored calibration (saved ${System.currentTimeMillis() - stored.savedAtMs}ms ago); skipping stationary gate"
            )
            calibrationPersistedThisRun = true
        } catch (e: Exception) {
            Log.e(TAG, "Failed to push stored calibration to native: ${e.message}", e)
        }
    }

    /** Step 5: poll native init-status at 4 Hz and persist calibration when the gate passes. */
    private fun startInitStatusPoller() {
        initStatusPollerJob?.cancel()
        initStatusPollerJob = repositoryScope.launch {
            while (isActive) {
                val raw = try { NativeBridge.getInitStatus() } catch (e: Throwable) {
                    Log.e(TAG, "getInitStatus failed: ${e.message}"); -1
                }
                val status = when (raw) {
                    0 -> InitStatus.WAIT_STATIONARY
                    1 -> InitStatus.WAIT_MOTION
                    2 -> InitStatus.READY
                    3 -> InitStatus.TIMEOUT_NEEDS_USER
                    else -> _initStatus.value
                }
                if (status != _initStatus.value) {
                    Log.i(TAG, "InitStatus: ${_initStatus.value} -> $status")
                    _initStatus.value = status
                }
                if (!calibrationPersistedThisRun &&
                    (status == InitStatus.WAIT_MOTION || status == InitStatus.READY)) {
                    persistNativeCalibration()
                }
                delay(250L)
            }
        }
    }

    /** Step 5: pull calibration from native and write it to SharedPreferences. */
    private fun persistNativeCalibration() {
        val rotation = FloatArray(9)
        val gyroBias = FloatArray(3)
        val accelBias = FloatArray(3)
        val ok = try {
            NativeBridge.getCalibration(rotation, gyroBias, accelBias)
        } catch (e: Throwable) {
            Log.e(TAG, "getCalibration failed: ${e.message}", e); false
        }
        if (!ok) return
        CalibrationStore.save(
            context,
            CalibrationStore.CalibrationData(
                rotation = rotation,
                gyroBias = gyroBias,
                accelBias = accelBias,
                savedAtMs = System.currentTimeMillis(),
            )
        )
        calibrationPersistedThisRun = true
    }

    /** Step 5: invoked by UI when the user dismisses the timeout dialog.
     *
     * Bypasses the stationary variance gate entirely by injecting calibration
     * via loadStoredCalibration — which unconditionally transitions the native
     * state to WAIT_MOTION. The gate was gating on sensor noise thresholds that
     * vary widely across devices; when the user explicitly says "it's flat",
     * we trust them. The EKF refines gyro bias online once walking starts.
     *
     * Priority: use persisted calibration from a prior session if available
     * (best accuracy), otherwise inject identity rotation + zero biases
     * (VIO starts immediately; EKF converges within seconds of motion).
     */
    fun clearInitTimeout() {
        val stored = CalibrationStore.load(context)
        val rotation  = stored?.rotation  ?: floatArrayOf(1f,0f,0f, 0f,1f,0f, 0f,0f,1f)
        val gyroBias  = stored?.gyroBias  ?: floatArrayOf(0f, 0f, 0f)
        val accelBias = stored?.accelBias ?: floatArrayOf(0f, 0f, 0f)
        try {
            NativeBridge.loadStoredCalibration(rotation, gyroBias, accelBias)
            Log.i(TAG, "clearInitTimeout: bypassed gate via loadStoredCalibration " +
                "(stored=${stored != null})")
        } catch (e: Throwable) {
            Log.e(TAG, "clearInitTimeout: loadStoredCalibration failed, falling back", e)
            try { NativeBridge.clearInitTimeout() } catch (e2: Throwable) {
                Log.e(TAG, "clearInitTimeout native fallback also failed", e2)
            }
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
    // Plan Step 1b/8b: set true by pushCameraIntrinsicsToNative() when a
    // valid camera_calib.json is loaded at startup. Suppresses the
    // first-frame default-intrinsics push so the calibration values
    // (fx/fy/cx/cy + distortion) survive to the runtime tracker.
    private var calibratedIntrinsicsLoaded = false

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

        initStatusPollerJob?.cancel()
        initStatusPollerJob = null
        locationTokenSource?.cancel()
        stopGpsUpdates()
        vioExecutor.shutdown()
        depthExecutor.shutdown()
        depthEstimator.close()
        intrinsicsInitialized = false
        calibratedIntrinsicsLoaded = false
        calibrationPersistedThisRun = false
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

        // Copy yBytes for depth estimation BEFORE submitting to vioExecutor.
        // This eliminates any ordering dependency between the VIO executor thread
        // (which reads yBuffer via GetDirectBufferAddress) and the camera thread
        // reading yBuffer here for depth.
        val needsDepth = !depthProcessing && (nowMs - lastDepthTimeMs >= DEPTH_THROTTLE_MS)
        val yBytesForDepth: ByteArray? = if (needsDepth) {
            ByteArray(w * h).also { bytes ->
                yBuffer.rewind()
                if (yRowStride == w) {
                    yBuffer.get(bytes, 0, w * h)
                } else {
                    for (row in 0 until h) {
                        yBuffer.position(row * yRowStride)
                        yBuffer.get(bytes, row * w, w)
                    }
                }
                yBuffer.rewind()  // restore position for JNI (GetDirectBufferAddress ignores it, but be explicit)
            }
        } else null

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
                    timestamp = timestampNs,
                    rollingShutterSkewNs = rollingShutterSkewNs  // Step 8c
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

        // Depth estimation at 1Hz — MiDaS feeds Tracker scale constraint
        if (yBytesForDepth != null) {
            depthProcessing = true
            lastDepthTimeMs = nowMs
            depthExecutor.execute {
                try {
                    val bitmap = android.graphics.Bitmap.createBitmap(w, h, android.graphics.Bitmap.Config.ARGB_8888)
                    val pixels = IntArray(w * h)
                    for (i in yBytesForDepth.indices) {
                        val lum = yBytesForDepth[i].toInt() and 0xFF
                        pixels[i] = (0xFF shl 24) or (lum shl 16) or (lum shl 8) or lum
                    }
                    bitmap.setPixels(pixels, 0, w, 0, 0, w, h)
                    val depthMap = kotlinx.coroutines.runBlocking { depthEstimator.estimateDepth(bitmap) }
                    if (depthMap != null) {
                        NativeBridge.setDepthMap(depthMap, 256, 256)
                    }
                    bitmap.recycle()
                } catch (e: Exception) {
                    Log.e(TAG, "Depth processing error: ${e.message}")
                } finally {
                    depthProcessing = false
                }
            }
        }
    }

    private fun handleVioInitialized(vio: VioData) {
        if (vio.isInitialized && !wasVioInitialized) {
            wasVioInitialized = true
            vioInitAzimuth = _orientationState.value.azimuth

            // Apply magnetic declination correction if we have a GPS fix
            var declinationDeg = 0f
            val loc = _startLocation.value
            if (loc != null) {
                val geoField = android.hardware.GeomagneticField(
                    loc.latitude.toFloat(), loc.longitude.toFloat(), 0f,
                    System.currentTimeMillis()
                )
                declinationDeg = geoField.declination
            }
            val correctedAzimuth = vioInitAzimuth + declinationDeg
            val azimuthRad = Math.toRadians(correctedAzimuth.toDouble())
            NativeBridge.setInitialHeading(azimuthRad)
            Log.i(TAG, "Initial heading set: ${vioInitAzimuth}° + declination ${declinationDeg}° = ${correctedAzimuth}° (${azimuthRad} rad)")

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
            
            val onLocation = com.google.android.gms.tasks.OnSuccessListener<android.location.Location?> { location ->
                // Only set startLocation if it hasn't been set yet — never overwrite
                // after VIO has started tracking, as that would shift the entire path
                if (location != null && _startLocation.value == null) {
                    _startLocation.value = LatLng(location.latitude, location.longitude)
                }
                locationTokenSource = null
            }
            val onFail = com.google.android.gms.tasks.OnFailureListener {
                Log.w(TAG, "Failed to get current location")
                locationTokenSource = null
            }

            if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.S) {
                fusedLocationClient.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token)
                    .addOnSuccessListener(onLocation).addOnFailureListener(onFail)
            } else {
                @Suppress("MissingPermission")
                fusedLocationClient.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token)
                    .addOnSuccessListener(onLocation).addOnFailureListener(onFail)
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
