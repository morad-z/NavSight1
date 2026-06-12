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
import java.util.concurrent.atomic.AtomicBoolean

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
    // 2026-05-25 continuous compass: latest OS-reported magnetometer accuracy,
    // updated in onAccuracyChanged. Gates mag->VIO heading fusion to HIGH/MEDIUM
    // ("relevant locations"). Default UNRELIABLE so we don't fuse until the OS
    // confirms a trustworthy field.
    private var magAccuracy: Int = SensorManager.SENSOR_STATUS_UNRELIABLE
    // 2026-05-25 field-magnitude gate for continuous compass: total |B| (uT)
    // from the raw magnetometer. Clean Earth field ~25-65 uT; out-of-band =
    // local ferromagnetic anomaly. Used instead of OS accuracy (which stayed
    // below MEDIUM the whole 2026-05-25 walk, so MAG_FUSE never fired).
    private var magFieldMagnitude: Float = 0f
    private var lastMagLogMs: Long = 0L
    private val gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    // 2026-05-25 PRIMARY heading: Android's fused, auto-calibrating orientation
    // (gyro+accel+mag) — what compass apps use. Its yaw feeds the VIO heading
    // (the raw magnetometer was uncalibrated: |B| 9-59uT, osAcc=LOW, wild azimuth
    // -> heading flips/V-shapes). Gated on its own estimated heading accuracy.
    private val rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    // Cached magnetic declination (deg, location-dependent ~static) so the
    // rotation-vector branch converts magnetic-north yaw to true north without
    // recomputing GeomagneticField at sensor rate.
    private var cachedDeclinationDeg: Float = 0f
    private val fusedLocationClient = LocationServices.getFusedLocationProviderClient(context)

    private val orientationTracker = DeviceOrientationTracker()
    // MiDaS depth feeds Tracker scale constraint (bypasses Mapper).
    //
    // 2026-05-09 reset-fix: same pattern as vioExecutor — stopSensors()
    // calls depthEstimator.close() which permanently releases the TFLite
    // interpreter, GPU delegate, and inferenceDispatcher. With a `val`
    // field, resetAll() leaves MiDaS dead for the rest of the session:
    // estimateDepth returns null, setDepthMap never fires, scale drift
    // goes unconstrained. Make it `var` and recreate from
    // ensureExecutorsAlive() on the next startSensors().
    private var depthEstimator: DepthEstimator = newDepthEstimator()
    @Volatile private var depthEstimatorClosed = false

    private fun newDepthEstimator(): DepthEstimator {
        depthEstimatorClosed = false
        return DepthEstimator(context)
    }

    // Dedicated VIO processing thread — decouples camera preview from VIO computation.
    // Frame dropping: if VIO is still busy when the next frame arrives, we skip it.
    //
    // 2026-05-09 reset-fix: the executor is `var` and re-created by
    // ensureExecutorsAlive() because stopSensors() shuts them down and
    // resetAll() (Stage 3) wires through stopSensors → startSensors
    // expecting the pipeline to come back. Without re-creation the second
    // start would submit to a dead executor and frames would silently drop.
    private var vioExecutor: java.util.concurrent.ExecutorService = newVioExecutor()
    @Volatile private var vioProcessing = false
    @Volatile private var vioFrameCount = 0

    private fun newVioExecutor() = java.util.concurrent.Executors.newSingleThreadExecutor { r ->
        Thread(r, "NavSight-VIO").apply { isDaemon = true }
    }

    // Main-Looper handler reused for SensorManager.registerListener so
    // sensor callbacks always dispatch on the main thread regardless of
    // which thread called startSensors().
    //
    // 2026-05-09 reset-fix: resetAll() runs stopSensors → startSensors on
    // Dispatchers.IO. The 3-arg registerListener overload uses
    // Looper.myLooper() of the caller, which is null on IO threads. The
    // spec says fall back to the main Looper, but Samsung OneUI variants
    // have been observed to silently drop — onSensorChanged is never
    // delivered, EKF receives no IMU, VIO stays forever at WAIT_MOTION
    // ("VIO LOST" red banner) until app restart. Passing an explicit
    // main handler makes registration thread-agnostic.
    private val mainHandler = android.os.Handler(android.os.Looper.getMainLooper())

    // Step 8c: rolling-shutter row read-out time from Camera2.
    // Updated by CameraUi.kt Camera2Interop CaptureCallback on every frame.
    // Camera2 API: CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW (API level 21+) —
    // nanoseconds from first-row to last-row read-out. 0 = skew unavailable.
    @Volatile var rollingShutterSkewNs: Long = 0L
        private set

    fun updateRollingShutterSkew(skewNs: Long) {
        rollingShutterSkewNs = skewNs
    }

    // Step 9 / ADR-014 — optional camera-frame recorder for replay-harness
    // fixtures. Set when the user starts a sim recording from the debug panel,
    // cleared when the recording stops. Volatile because writes happen from
    // the VM main thread and reads happen from the camera analyzer thread.
    @Volatile private var frameRecorder: SimulationFrameRecorder? = null
    fun setFrameRecorder(recorder: SimulationFrameRecorder?) {
        frameRecorder = recorder
    }

    // Depth estimation at ~1Hz for scale constraint
    private var depthExecutor: java.util.concurrent.ExecutorService = newDepthExecutor()
    // Audit Finding 5 (2026-05-16): @Volatile required for ARM memory-model
    // correctness. CameraX analyzer thread reads at the needsDepth gate;
    // depthExecutor (or repositoryScope coroutine after Fix 2) writes in finally.
    // Without @Volatile, the read and write may never synchronize on ARM's
    // weak memory model — identical pattern to vioProcessing at line 79.
    @Volatile private var depthProcessing = false
    private var lastDepthTimeMs = 0L

    private fun newDepthExecutor() = java.util.concurrent.Executors.newSingleThreadExecutor { r ->
        Thread(r, "NavSight-Depth").apply { isDaemon = true }
    }

    /**
     * Re-create per-frame executors and the depth estimator if
     * stopSensors() shut them down. Called from startSensors() so the
     * second start after a resetAll() restores the VIO frame-processing
     * pipeline AND the MiDaS scale constraint. Idempotent — leaves
     * still-alive resources untouched.
     */
    private fun ensureExecutorsAlive() {
        if (vioExecutor.isShutdown) vioExecutor = newVioExecutor()
        if (depthExecutor.isShutdown) depthExecutor = newDepthExecutor()
        if (depthEstimatorClosed) depthEstimator = newDepthEstimator()
    }
    private val DEPTH_THROTTLE_MS = 1000L // 1Hz depth — conservative for battery

    // 2026-05-13 heading-startup fix: track whether Madgwick has been seeded
    // with the compass yaw. Set on first valid compass reading (with no
    // declination); flipped to "with declination" once GPS lands and the
    // re-seed fires. Prevents repeated re-init calls flooding Madgwick.
    private var madgwickYawSeeded = false
    private var madgwickYawSeededWithDeclination = false

    private val _orientationState = MutableStateFlow(DeviceOrientationTracker.OrientationResult(
        pitch = 0f, roll = 0f, azimuth = 0f,
        isHorizontal = false, deviationFromHorizontal = 90f, stabilityScore = 0f
    ))
    val orientationState = _orientationState.asStateFlow()

    private val _vioState = MutableStateFlow(VioData())
    val vioState = _vioState.asStateFlow()

    /**
     * Phase 1 camera-overlay plumbing: the analyzer image dimensions and the
     * CCW rotation CameraX applies to make the analyzer upright on the
     * preview surface. Both are needed by `CameraFeatureOverlay` to map
     * analyzer-space KLT pixel coords onto the Compose Canvas. Updated on
     * every camera frame in `processCameraFrame`; the flow only emits when
     * the geometry actually changes (typically once per session). Cleared
     * back to null in `resetAll` so the overlay hides until the first new
     * frame after reset re-publishes.
     */
    data class CameraFrameGeometry(
        val analyzerWidth: Int,
        val analyzerHeight: Int,
        val rotationDegrees: Int,
    )
    private val _cameraFrameGeometry = MutableStateFlow<CameraFrameGeometry?>(null)
    val cameraFrameGeometry = _cameraFrameGeometry.asStateFlow()

    // ── Camera overlay Phase 2/3/4 plumbing ────────────────────────────────
    // These flows publish at the FULL native frame rate (~30 Hz) so the
    // overlay tracks the live preview with no lag. They are SEPARATE from
    // _vioState (which the ViewModel deliberately throttles to 5 Hz for
    // heavier consumers like the path history and crash logger). The
    // overlay collects ONLY this flow, so heavy consumers stay throttled.
    //
    // OverlaySnapshot bundles every per-frame piece of overlay state into
    // a single value class so the ViewModel collects ONE flow and the
    // Canvas recomposes at most once per native frame even when multiple
    // fields change together.
    data class OverlaySnapshot(
        val trackedPoints: FloatArray,    // [x0,y0, x1,y1, ...] analyzer px
        val trackedAges: IntArray,        // parallel, frames survived
        val slamSnapshot: FloatArray,     // [fid,x,y,z, ...] Y-up world
        val cameraPose: FloatArray,       // 16 floats; empty if EKF not ready
        val loopClosureCount: Long,       // cumulative
        // 2026-06-02 — per-point recoverPose inlier flags (1=VIO used it, 0=rejected/unverified),
        // parallel to trackedPoints; empty when unavailable.
        val trackedInlierFlags: ByteArray = ByteArray(0),
        // 2026-06-02 — ground-plane snapshot [is_valid, horizon_v_px, confidence, scale, cand, inl];
        // empty when the estimator is off (no camera height set).
        val groundPlane: FloatArray = FloatArray(0),
        // 2026-06-02 — IPM road-inlier pixel positions [x0,y0, x1,y1, ...] (analyzer coords) the
        // ground-plane speed used this frame; drawn as amber dots so the rider sees ground recognition.
        val ipmInlierPoints: FloatArray = FloatArray(0),
        // 2026-06-02 — AV-style ground-plane grid line segments [x0,y0,x1,y1, ...] (analyzer coords):
        // the IPM ground plane projected onto the image as a perspective road mesh.
        val groundGridSegs: FloatArray = FloatArray(0),
        // 2026-06-04 — IPM per-point DIAGNOSTIC: ALL road candidates [x,y,vi_kmh,survived, ...] (4/point),
        // drawn as speed-graded dots (rejects red) so the rider sees the per-point speed live.
        val ipmCandidates: FloatArray = FloatArray(0),
    ) {
        // Compose reads-per-frame; equality on identity is enough.
        override fun equals(other: Any?): Boolean = this === other
        override fun hashCode(): Int = System.identityHashCode(this)
    }
    private val _overlaySnapshot = MutableStateFlow(
        OverlaySnapshot(floatArrayOf(), intArrayOf(), floatArrayOf(), floatArrayOf(), 0L)
    )
    val overlaySnapshot = _overlaySnapshot.asStateFlow()

    // Reusable JNI getter buffers — preallocated so the hot camera path
    // does not allocate. The agesBuf trims via copyOf when fewer features
    // are returned. SLAM is bounded by MAX_SLAM_FEATURES = 12 (ADR-009).
    private val agesBuf = IntArray(512)
    private val inlierFlagsBuf = ByteArray(512)   // 2026-06-02 — recoverPose inlier flags (parallel to points)
    private val groundPlaneBuf = FloatArray(6)    // 2026-06-02 — [valid, horizon_v, conf, scale, cand, inl]
    private val ipmInlierBuf = FloatArray(512)    // 2026-06-02 — IPM road-inlier pixel positions (x,y pairs)
    private val ipmCandBuf = FloatArray(1024)     // 2026-06-04 — IPM candidates (x,y,vi_kmh,survived) = 4/point, ≤256 pts
    private val groundGridBuf = FloatArray(1024)  // 2026-06-02 — ground-plane grid segments (x0,y0,x1,y1 each)
    // v23.11: stride increased from 4 → 7 floats per feature
    // (fid, wx, wy, wz, obs_u, obs_v, has_obs).
    private val slamBuf = FloatArray(7 * 12)
    private val cameraPoseBuf = FloatArray(16)
    private val emptyFloats = FloatArray(0)
    private val emptyInts = IntArray(0)
    private val emptyBytes = ByteArray(0)

    private val _startLocation = MutableStateFlow<LatLng?>(null)
    val startLocation = _startLocation.asStateFlow()

    private var lastOrientationUpdateNs = 0L
    private val accelMagnitudeHistory = ArrayDeque<Float>(20)
    private var lastShakeCheckMs = 0L

    private var locationTokenSource: CancellationTokenSource? = null

    // VIO Init state
    // Audit Finding 7 (2026-05-16): TOCTOU race — handleVioInitialized runs on
    // NavSight-VIO thread and checks-then-sets wasVioInitialized. Plain var with
    // @Volatile does NOT make check-then-set atomic; two rapid VIO-init callbacks
    // can both pass the `!wasVioInitialized` guard and call setInitialHeading twice
    // (explains "3 EKF re-inits" in v26 logcat). AtomicBoolean.compareAndSet(false,
    // true) is the only correct fix for this TOCTOU.
    private val _wasVioInitialized = AtomicBoolean(false)
    val wasVioInitialized: Boolean get() = _wasVioInitialized.get()
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
        // 2026-05-09 reset-fix: stopSensors() shuts the per-frame executors
        // down. Re-create them now or every camera-thread submit will throw
        // RejectedExecutionException and frames will silently drop —
        // visible to the user as VIO never advancing past WAIT_MOTION
        // ("VIO LOST" red banner) after pressing the reset button.
        ensureExecutorsAlive()
        try {
            accelerometer?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME, mainHandler)
                Log.d(TAG, "Accelerometer registered")
            } ?: Log.w(TAG, "Accelerometer not available on this device")

            magnetometer?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME, mainHandler)
                Log.d(TAG, "Magnetometer registered")
            } ?: Log.w(TAG, "Magnetometer not available on this device")

            gyroscope?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME, mainHandler)
                Log.d(TAG, "Gyroscope registered")
            } ?: Log.w(TAG, "Gyroscope not available on this device")

            // 2026-05-25 rotation-vector heading source (compass-app approach).
            rotationVector?.let {
                sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME, mainHandler)
                Log.d(TAG, "RotationVector registered")
            } ?: Log.w(TAG, "RotationVector not available on this device")

            if (NativeBridge.isLoaded()) {
                NativeBridge.startVIO()
                pushStoredCalibrationToNative()
                pushCameraIntrinsicsToNative()
                pushLoopClosureVocabularyToNative()
                // 2026-05-18 DISABLED: Kotlin Rz(-θ)*R_bc_default math produces
                // R_bc that's 90° from correct diag(1,-1,-1) for SENSOR_ORIENTATION=90
                // (S21 Ultra back camera). v40 walk confirmed extrinsics_rotation_angle
                // _mdeg = 90000, MSCKF chi² rejection rate 44%, LC rotation residual
                // up to 57° late in the walk. The C++ compile-time default
                // diag(1,-1,-1) at EKFState.cpp:63 is the correct value for this
                // device; the Kotlin push was a 2026-05-11 (v23.8) "fix" that
                // moved the bug instead of fixing it. Re-enable only after
                // re-deriving the math against a non-90°-orientation device.
                // pushExtrinsicsRotationToNative()   // Step 8b
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
            //
            // v23.8 (2026-05-11): switched composition from RIGHT-multiply
            // (R_bc = R_bc_default * Rz(-θ)) to LEFT-multiply
            // (R_bc = Rz(-θ) * R_bc_default).
            //
            // The sensor rotation is a transformation IN THE CAMERA FRAME
            // (sensor mounted rotated θ° in the phone). The correct
            // composition is therefore left-multiplication: first map
            // body → un-rotated camera via R_bc_default, then apply the
            // camera-frame rotation Rz(-θ).
            //
            // Right-multiply (the previous code) instead rotated the BODY
            // input by -θ before the default mapping, which produces a
            // mirror-image R_bc with opposite signs on the off-diagonal
            // entries. Diagnostic: v23.6 had the sensor-rotation applied
            // via right-multiply and user reported phone-L/R → dots-U/D
            // (90° axis swap). v23.7 force-defaulted R_bc and the symptom
            // persisted (because sensor IS rotated 90° physically). v23.8
            // applies the rotation in the correct frame.
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
                        // LEFT-multiply: R_bc = Rz(-θ) * R_bc_default
                        // out[r,c] = sum_k rz[r,k] * def[k,c]
                        sum += rz[row * 3 + k] * def[k * 3 + col]
                    }
                    R_bc[row * 3 + col] = sum
                }
            }

            NativeBridge.nativeSetExtrinsicsRotation(R_bc)
            Log.i(TAG, "Extrinsics R_bc seeded from SENSOR_ORIENTATION=${sensorOrientation}° via LEFT-multiply (v23.8)")
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
                // Audit Finding 4 (2026-05-16): both API-31+ and pre-31 branches
                // made identical requestLocationUpdates calls — the version split
                // served no purpose and the @Suppress hid a real SecurityException
                // risk if permission is revoked between check and call. Collapsed
                // to one call; SecurityException is already caught by the outer
                // try/catch and logged.
                @Suppress("MissingPermission")  // guarded by granted param check above
                fusedLocationClient.requestLocationUpdates(
                    request, it, android.os.Looper.getMainLooper()
                )
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
        // Guard against double-close if stopSensors fires twice (e.g. a
        // pause arrives mid-resetAll). Some TFLite native heap sanitizers
        // crash on a re-close of an already-released interpreter.
        if (!depthEstimatorClosed) {
            depthEstimator.close()
            depthEstimatorClosed = true
        }
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
                // 2026-05-25 field-magnitude gate: total field strength (uT).
                // Clean Earth field ~25-65 uT; far outside that band = local
                // ferromagnetic anomaly (a bad-field "location" to skip).
                val mx = event.values[0]; val my = event.values[1]; val mz = event.values[2]
                magFieldMagnitude = kotlin.math.sqrt(mx * mx + my * my + mz * mz)
            }
            Sensor.TYPE_GYROSCOPE -> {
                NativeBridge.processGyroscope(ts, event.values[0], event.values[1], event.values[2])
            }
            Sensor.TYPE_ROTATION_VECTOR -> {
                // 2026-05-25 ONLY heading source: Android's fused, auto-
                // calibrating orientation (what compass apps use). Extract yaw,
                // gate on the device's OWN estimated heading accuracy
                // (values[4], rad; API 18+), apply declination, feed it to the
                // native heading filter which SNAPS Madgwick yaw to it (not
                // fused) and gates the gyro/visual/LC heading nudges off.
                if (cachedDeclinationDeg == 0f) {
                    _startLocation.value?.let { loc ->
                        cachedDeclinationDeg = android.hardware.GeomagneticField(
                            loc.latitude.toFloat(), loc.longitude.toFloat(), 0f,
                            System.currentTimeMillis()).declination
                    }
                }
                val rv = if (event.values.size > 4) event.values.copyOfRange(0, 4)
                         else event.values
                val rvR = FloatArray(9)
                SensorManager.getRotationMatrixFromVector(rvR, rv)
                // 2026-05-25 (rev 3) — TILT-ADAPTIVE heading, the way compass apps
                // work in ANY hold. getOrientation()'s azimuth assumes a FLAT phone
                // (it reads the +Y/top edge); held upright camera-forward (NavSight's
                // walking pose) +Y points at the sky -> gimbal lock -> ~95 deg offset
                // + 200<->0 jumps (the bench test vs NOAA). Fix: pick the device axis
                // that is currently HORIZONTAL and read its ground-plane bearing.
                // rvR is device->world ENU (x=East, y=North, z=Up), row-major; a
                // device axis in world = a COLUMN of rvR (world = rvR * axis):
                //   - flat (screen up/down): forward = TOP edge (+Y) = column 1
                //   - upright (camera fwd):  forward = CAMERA (-Z)   = -column 2
                val screenUpComp = rvR[8]            // world-Up component of device +Z (screen normal)
                val rvUpright = kotlin.math.abs(screenUpComp) <= 0.5f
                val fwdEast: Float
                val fwdNorth: Float
                if (rvUpright) {
                    fwdEast = -rvR[2]; fwdNorth = -rvR[5]   // camera (-Z) projected to ground
                } else {
                    fwdEast = rvR[1];  fwdNorth = rvR[4]    // top edge (+Y) projected to ground
                }
                // Azimuth CW from North (N=0, E=90) — matches Android/NOAA convention.
                val rvAzimuthDeg = Math.toDegrees(
                    kotlin.math.atan2(fwdEast.toDouble(), fwdNorth.toDouble())).toFloat()
                // Estimated heading accuracy (rad) if reported; NaN if not.
                val rvHeadingAccRad = if (event.values.size >= 5) event.values[4] else Float.NaN
                // 2026-05-25 the S21 reports values[4] = -1 = heading accuracy
                // UNAVAILABLE (NOT "bad"). Treating -1 as untrustworthy made the
                // RV never engage -> heading fell back to the drifting gyro
                // ("moves while stationary"). Trust the RV unless it EXPLICITLY
                // reports poor accuracy (>= 0.5 rad). -1 / NaN (unavailable) ->
                // trust: the RV is the OS-fused orientation and is stable while
                // stationary, whereas the gyro fallback drifts.
                val rvTrustworthy = rvHeadingAccRad.isNaN() || rvHeadingAccRad < 0.5f
                if (rvAzimuthDeg.isFinite() && rvTrustworthy) {
                    val yawRad = Math.toRadians(
                        (rvAzimuthDeg + cachedDeclinationDeg).toDouble()).toFloat()
                    NativeBridge.setMagnetometerHeading(yawRad)
                }
                val nowRvMs = System.currentTimeMillis()
                if (nowRvMs - lastMagLogMs > 1000L) {
                    lastMagLogMs = nowRvMs
                    Log.i(TAG, "RV_SEND: az=${"%.1f".format(rvAzimuthDeg)} " +
                            "mode=${if (rvUpright) "UPRIGHT" else "FLAT"} " +
                            "hdgAccRad=${"%.2f".format(rvHeadingAccRad)} trust=$rvTrustworthy " +
                            "decl=${"%.1f".format(cachedDeclinationDeg)} " +
                            "sent=${rvAzimuthDeg.isFinite() && rvTrustworthy}")
                    // 2026-05-25 DIAGNOSTIC (temporary): every candidate device-axis
                    // bearing as a TRUE heading (declination folded in, 0-360). Hold
                    // the phone exactly as you walk, point it at a known direction,
                    // and whichever value matches the NOAA reading is the axis to use.
                    fun trueAz(e: Float, n: Float): Int {
                        var d = Math.toDegrees(kotlin.math.atan2(e.toDouble(), n.toDouble())) +
                                cachedDeclinationDeg
                        d = ((d % 360) + 360) % 360
                        return d.toInt()
                    }
                    Log.i(TAG, "RV_DIAG (true hdg vs NOAA): " +
                            "camZ=${trueAz(-rvR[2], -rvR[5])} " +
                            "topY=${trueAz(rvR[1], rvR[4])} " +
                            "rightX=${trueAz(rvR[0], rvR[3])} " +
                            "leftNegX=${trueAz(-rvR[0], -rvR[3])} " +
                            "screenUp=${"%.2f".format(screenUpComp)}")
                }
            }
        }

        if (ts - lastOrientationUpdateNs >= 50_000_000L) {
            lastOrientationUpdateNs = ts
            val orientation = orientationTracker.getOrientation()
            _orientationState.value = orientation

            // 2026-05-13 heading-startup fix: seed Madgwick's yaw the moment
            // the compass is stable, DECOUPLED from vio.isInitialized.
            //
            // Before this fix, the only path that pushed compass heading to
            // Madgwick was handleVioInitialized → setInitialHeading, gated on
            // the EKF static-init period (1-3 s). Until then Madgwick ran with
            // its default yaw seed of 0 (North), so scalar_heading_ (which
            // Tracker.cpp:1748 reads from imu.getHeading()) showed 0 regardless
            // of actual orientation. The radar rotated in place when phone was
            // still because gyro integration accumulates from yaw=0 with bias
            // noise. User symptom: "after a few seconds heading starts tracking
            // correctly" — the few seconds = VIO init duration.
            //
            // Fires twice at most: once on first valid compass reading
            // (declination=0 if GPS not yet fixed), again with declination
            // applied if GPS lands later. Both fires call into Madgwick's
            // idempotent re-init path.
            val az = orientation.azimuth
            if (az.isFinite() && !madgwickYawSeededWithDeclination) {
                val haveLoc = _startLocation.value != null
                if (haveLoc || !madgwickYawSeeded) {
                    var declinationDeg = 0f
                    val loc = _startLocation.value
                    if (loc != null) {
                        val geoField = android.hardware.GeomagneticField(
                            loc.latitude.toFloat(), loc.longitude.toFloat(), 0f,
                            System.currentTimeMillis()
                        )
                        declinationDeg = geoField.declination
                    }
                    val correctedAz = az + declinationDeg
                    val yawRad = Math.toRadians(correctedAz.toDouble())
                    NativeBridge.seedMadgwickYaw(yawRad)
                    Log.i(TAG, "Madgwick yaw seeded early: az=${az}° + decl=${declinationDeg}° = ${correctedAz}° (haveLoc=$haveLoc)")
                    madgwickYawSeeded = true
                    if (haveLoc) madgwickYawSeededWithDeclination = true
                }
            }

            // 2026-05-25 raw-magnetometer VIO heading send REMOVED. The compass
            // was uncalibrated (|B| 9-59uT, osAcc=LOW, wild azimuth -> 90deg
            // heading flips + V-shapes). The VIO heading now comes ONLY from the
            // fused, auto-calibrating TYPE_ROTATION_VECTOR (onSensorChanged
            // branch above). Raw mag still drives the on-screen compass arrow via
            // orientationTracker; it just no longer touches the VIO heading.
            // LEGACY raw-mag send:
            //   val magFieldClean = magFieldMagnitude in 25f..70f
            //   if (az.isFinite() && magFieldClean) { NativeBridge.setMagnetometerHeading(az+decl) }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // 2026-05-25 continuous compass gating: remember the OS-reported
        // magnetometer accuracy so the orientation tick only forwards the
        // compass heading to VIO when the field is trustworthy (HIGH/MEDIUM).
        if (sensor?.type == Sensor.TYPE_MAGNETIC_FIELD) {
            magAccuracy = accuracy
        }
    }

    fun processCameraFrame(image: ImageProxy) {
        // Drop frame if VIO is still processing the previous one.
        if (vioProcessing) {
            image.close()
            return
        }

        val w = image.width
        val h = image.height

        // Phase 1 camera-overlay plumbing: publish analyzer geometry so the
        // Compose overlay can map analyzer-space KLT coords → Canvas pixels.
        // The MutableStateFlow update only fires downstream collectors when
        // the value actually changes, so this is a near-no-op after frame 1.
        run {
            val rot = image.imageInfo.rotationDegrees
            val cur = _cameraFrameGeometry.value
            if (cur == null ||
                cur.analyzerWidth != w ||
                cur.analyzerHeight != h ||
                cur.rotationDegrees != rot) {
                _cameraFrameGeometry.value = CameraFrameGeometry(w, h, rot)
                // Tell native the analyzer rotation so the ground-plane estimator searches the region
                // that maps to the bottom of the upright (portrait) view (where the road is).
                NativeBridge.setAnalyzerRotation(rot)
            }
        }

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

        // Step 9 / ADR-014 — submit the Y plane to the frame recorder before
        // the JNI consumes it. The recorder copies bytes synchronously then
        // returns; encode runs on its own thread. Filename uses wall-clock
        // ms × 1e6 so the replay harness can match against the simulator
        // JSON `ts` field (also wall-clock ms). The few-ms offset between
        // this capture stamp and the matching SimulationPoint timestamp is
        // absorbed by the harness's --frame-match-tolerance-ms flag.
        run {
            val recorder = frameRecorder
            if (recorder != null && recorder.isActive()) {
                yBuffer.rewind()
                recorder.captureFrame(
                    yBuffer = yBuffer,
                    width = w,
                    height = h,
                    yRowStride = yRowStride,
                    timestampNs = nowMs * 1_000_000L,
                )
                yBuffer.rewind()  // restore for downstream JNI / depth path
            }
        }

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

        // 1. VIO processing — pass direct buffers to JNI (near zero-copy).
        //
        // 2026-05-09 reset-fix: vioProcessing is set BEFORE submit so a
        // second camera frame arriving before the runnable starts gets
        // dropped at the line ~649 early-out guard. The catch below is
        // critical: if execute() throws RejectedExecutionException
        // (e.g. between stopSensors() and ensureExecutorsAlive() during
        // resetAll), the runnable's `finally` never runs and
        // vioProcessing would latch `true` forever — every subsequent
        // frame would be silently dropped at the early-out and VIO
        // would die for the rest of the session. Resetting the flag in
        // the catch closes that latch.
        vioProcessing = true
        val frameStartMs = nowMs
        try {
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
                    publishOverlaySnapshot(vio)
                    handleVioInitialized(vio)
                    checkCameraBlocked(vio)
                } finally {
                    image.close()  // Release ImageProxy AFTER JNI is done
                    vioProcessing = false
                }
            }
        } catch (e: java.util.concurrent.RejectedExecutionException) {
            Log.w(TAG, "vioExecutor rejected frame (executor shut down); dropping")
            image.close()
            vioProcessing = false
        }

        // Depth estimation at 1Hz — MiDaS feeds Tracker scale constraint.
        //
        // 2026-05-09 reset-fix: same RejectedExecutionException pattern
        // as the VIO submit above. Without the outer try/catch,
        // depthProcessing would latch `true` after a shutdown executor
        // rejected a submit, and depth would never run again for the
        // rest of the session — silent MiDaS death after any reset.
        if (yBytesForDepth != null) {
            depthProcessing = true
            lastDepthTimeMs = nowMs
            // Audit Finding 2 (2026-05-16): the prior code bridged from the
            // depthExecutor thread into coroutines via runBlocking, which:
            //   1. Swallows CancellationException (coroutine cancellation is lost).
            //   2. Burns a thread-pool thread while waiting on inferenceDispatcher.
            //   3. Serially couples two executor contexts unnecessarily.
            // Fix: launch a coroutine directly in repositoryScope.
            // DepthEstimator.estimateDepth is already a suspend fun that dispatches
            // to its own inferenceDispatcher internally — no bridge needed.
            // depthExecutor still exists for backwards-compat (ensureExecutorsAlive
            // recreates it) but is no longer used for the depth pipeline.
            // Cancellation propagates normally; a cancelled scope simply does not
            // call setDepthMap, leaving the previous depth map in effect (safe).
            repositoryScope.launch(Dispatchers.Default) {
                try {
                    val bitmap = android.graphics.Bitmap.createBitmap(w, h, android.graphics.Bitmap.Config.ARGB_8888)
                    val pixels = IntArray(w * h)
                    for (i in yBytesForDepth.indices) {
                        val lum = yBytesForDepth[i].toInt() and 0xFF
                        pixels[i] = (0xFF shl 24) or (lum shl 16) or (lum shl 8) or lum
                    }
                    bitmap.setPixels(pixels, 0, w, 0, 0, w, h)
                    // estimateDepth is a suspend fun; it handles its own
                    // inferenceDispatcher internally — no runBlocking needed.
                    val depthMap = depthEstimator.estimateDepth(bitmap)
                    if (depthMap != null) {
                        NativeBridge.setDepthMap(depthMap, 256, 256)
                    }
                    bitmap.recycle()
                } catch (e: CancellationException) {
                    // Coroutine was cancelled (e.g. stopSensors) — rethrow so
                    // structured concurrency propagates cancellation correctly.
                    throw e
                } catch (e: Exception) {
                    Log.e(TAG, "Depth processing error: ${e.message}")
                } finally {
                    depthProcessing = false
                }
            }
        }
    }

    private fun handleVioInitialized(vio: VioData) {
        // Audit Finding 7 (2026-05-16): compareAndSet(false, true) is atomic —
        // only the thread that wins the CAS proceeds into the heading-set block.
        // Prevents double setInitialHeading + double magnetometer unregister.
        if (vio.isInitialized && _wasVioInitialized.compareAndSet(false, true)) {
            Log.i(TAG, "VIO initialized: setting initial heading (atomic CAS, no re-init possible)")
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

            // 2026-05-25 continuous compass (professor approved): do NOT
            // unregister the magnetometer after init — the compass heading is
            // now fused continuously (gated to HIGH/MEDIUM OS accuracy), so it
            // must keep streaming. LEGACY (no-mag-during-tracking rule):
            // magnetometer?.let {
            //     sensorManager.unregisterListener(this, it)
            //     Log.i(TAG, "Magnetometer unregistered - initial heading captured.")
            // }
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

    /**
     * Camera overlay Phase 2/3/4: publish a fresh per-frame snapshot to the
     * overlay flow. Runs on the VIO executor immediately after the native
     * frame returned. Allocations on the steady-state path are bounded to
     * one OverlaySnapshot object plus copyOf calls only when each field is
     * non-empty (the buffers themselves are reused).
     *
     * Why a single bundled flow rather than four parallel ones: a single
     * MutableStateFlow.value set fires one Compose invalidation per
     * subscriber. Bundling means the Canvas recomposes once per native
     * frame even when all four fields change. Compare to four flows: four
     * invalidations per frame, four StateFlow writes, four equality checks.
     */
    // 2026-06-12b power-trim — overlay polling runs ONLY while the camera screen is open
    // (MapScreen sets this via the ViewModel on cameraVisible changes). The native side
    // lazy-computes its overlay products off these polls, so map-view riding stops paying
    // ~7 JNI array copies + a Compose snapshot publish per frame. Camera screen unchanged.
    @Volatile var overlayPollingEnabled = false

    private fun publishOverlaySnapshot(vio: VioData) {
        if (!overlayPollingEnabled) {
            // One EMPTY publish on the transition so stale overlays clear, then no churn.
            if (_overlaySnapshot.value.trackedAges.isNotEmpty() ||
                _overlaySnapshot.value.groundGridSegs.isNotEmpty() ||
                _overlaySnapshot.value.ipmCandidates.isNotEmpty()) {
                _overlaySnapshot.value = OverlaySnapshot(
                    trackedPoints = emptyFloats, trackedAges = emptyInts,
                    slamSnapshot = emptyFloats, cameraPose = emptyFloats,
                    loopClosureCount = 0, trackedInlierFlags = emptyBytes,
                    groundPlane = emptyFloats, ipmInlierPoints = emptyFloats,
                    groundGridSegs = emptyFloats, ipmCandidates = emptyFloats,
                )
            }
            return
        }
        // Phase 2: ages parallel to vio.trackedPoints.
        val expectedAgeCount = vio.trackedPoints.size / 2
        val agesArr: IntArray = if (expectedAgeCount <= 0) emptyInts
        else {
            val n = NativeBridge.getLastTrackedPointAges(agesBuf)
            // Trim if VIO published a different count than the JNI getter
            // returned (the two sources share state_mutex on the writer
            // side; reads here are sequential so a mismatch would only
            // happen if the buffer was undersized — agesBuf is sized 512,
            // well above any plausible KLT count).
            if (n == expectedAgeCount) agesBuf.copyOf(n) else emptyInts
        }

        // 2026-06-02: per-point recoverPose inlier flags, parallel to trackedPoints. Read under the
        // same coherent (ages, flags) writer lock; trim/blank on a count mismatch (treated as unknown).
        val inlierArr: ByteArray = if (expectedAgeCount <= 0) emptyBytes
        else {
            val n = NativeBridge.getLastTrackedPointInlierFlags(inlierFlagsBuf)
            if (n == expectedAgeCount) inlierFlagsBuf.copyOf(n) else emptyBytes
        }

        // 2026-06-02: ground-plane snapshot [valid, horizon_v, conf, scale, cand, inl] (empty when off).
        val gpN = NativeBridge.getGroundPlaneSnapshot(groundPlaneBuf)
        val gpArr: FloatArray = if (gpN >= 6) groundPlaneBuf.copyOf(6) else emptyFloats

        // 2026-06-02: IPM road-inlier pixel positions (x,y pairs) — the pixels the ground-plane speed
        // used this frame. Empty when the IPM found no road pixels (then the overlay shows none).
        val ipmN = NativeBridge.getIpmInlierPoints(ipmInlierBuf)
        val ipmArr: FloatArray = if (ipmN >= 2) ipmInlierBuf.copyOf(ipmN) else emptyFloats

        // 2026-06-02: AV-style ground-plane grid segments (x0,y0,x1,y1 each). Empty until gravity-aligned.
        val gridN = NativeBridge.getGroundGridSegments(groundGridBuf)
        val gridArr: FloatArray = if (gridN >= 4) groundGridBuf.copyOf(gridN) else emptyFloats

        // 2026-06-04: IPM per-point diagnostic candidates (x,y,vi_kmh,survived = 4 floats/point).
        val candN = NativeBridge.getIpmCandidates(ipmCandBuf)
        val candArr: FloatArray = if (candN >= 4) ipmCandBuf.copyOf(candN) else emptyFloats

        // Phase 3: SLAM positions (Y-up world). Bounded by MAX_SLAM_FEATURES.
        val slamCount = NativeBridge.getSlamSnapshot(slamBuf)
        // v23.11: stride 7 (fid, wx, wy, wz, obs_u, obs_v, has_obs).
        val slamArr: FloatArray = if (slamCount > 0) slamBuf.copyOf(slamCount * 7)
        else emptyFloats

        // Phase 3: camera pose (Y-up world). Empty when EKF not ready.
        val poseValid = NativeBridge.getCurrentCameraPose(cameraPoseBuf)
        val poseArr: FloatArray = if (poseValid) cameraPoseBuf.copyOf() else emptyFloats

        // Phase 4: loop closure correction count.
        val lcCount = NativeBridge.getLoopClosureCorrectionsApplied()

        _overlaySnapshot.value = OverlaySnapshot(
            trackedPoints = vio.trackedPoints,   // already a fresh JNI alloc per frame
            trackedAges = agesArr,
            slamSnapshot = slamArr,
            cameraPose = poseArr,
            loopClosureCount = lcCount,
            trackedInlierFlags = inlierArr,
            groundPlane = gpArr,
            ipmInlierPoints = ipmArr,
            groundGridSegs = gridArr,
            ipmCandidates = candArr,
        )
    }

    fun resetPath() {
        NativeBridge.resetVIO()
        _vioState.value = VioData()
        orientationTracker.reset()
        _wasVioInitialized.set(false)
        vioInitAzimuth = 0f
        consecutiveVioFailures = 0
        _showCameraBlocked.value = false
        accelMagnitudeHistory.clear()
    }

    /**
     * Stage 3 (2026-05-09) — full reset that mirrors a fresh app launch.
     *
     * Equivalent to: onPause → wipe persistent state → onResume →
     * requestInitialLocation. Everything that gets seeded from
     * `startSensors()` on a cold start is re-seeded. After this returns
     * the system is in the same state as immediately after MainActivity
     * acquires permissions and runs the first frame.
     *
     * Called from the UI's "Rides" reset button. Idempotent.
     *
     * @param hasLocationPermission caller-owned permission state (matches
     *        the ViewModel's flag set in requestInitialLocation()). Passed
     *        explicitly so the repository doesn't shadow the activity's
     *        permission tracking.
     */
    fun resetAll(hasLocationPermission: Boolean) {
        // 1. Tear down active subscriptions and any in-flight recorder
        //    so callbacks during the reset window can't write to stale
        //    state. Mirrors the lifecycle's onPause path.
        stopGpsUpdates()
        setFrameRecorder(null)
        stopSensors()

        // 2. Wipe Kotlin-side state that crosses session boundaries.
        _startLocation.value = null
        _currentLocation.value = null
        _showCameraBlocked.value = false
        _initStatus.value = InitStatus.WAIT_STATIONARY
        _wasVioInitialized.set(false)
        vioInitAzimuth = 0f
        consecutiveVioFailures = 0
        accelMagnitudeHistory.clear()
        intrinsicsInitialized = false
        calibratedIntrinsicsLoaded = false
        rollingShutterSkewNs = 0L
        // Audit Finding 1 (2026-05-16): madgwickYawSeeded flags were NOT reset here,
        // so after a "Rides" reset the 2026-05-13 heading-startup fix silently
        // re-introduced itself: both flags stayed true, the early-seed block at
        // onSensorChanged:780-797 exited immediately on every orientation update,
        // and Madgwick was never re-seeded on the second session.
        // Falsifier: after reset, expect "Madgwick yaw seeded early:" in logcat
        // within the first compass reading. If absent (but present on cold-start),
        // the bug is confirmed.
        madgwickYawSeeded = false
        madgwickYawSeededWithDeclination = false
        Log.i(TAG, "resetAll: Madgwick yaw seed flags cleared (Finding 1 fix)")
        // Defensive: if a frame slipped between stopSensors's executor
        // shutdown and ensureExecutorsAlive's recreate, the per-frame
        // guards may have latched true. Reset here so the first new
        // camera frame after startSensors() is not dropped at the
        // early-out gate at the top of processCameraFrame.
        vioProcessing = false
        depthProcessing = false

        // 3. Clear native + filter state. resetVIO() flows through to
        //    VioEngine::reset → Tracker::reset → EKFState::reset, plus
        //    nativeResetEventCounters() to zero the telemetry block so
        //    fresh diagnostics start from 0.
        NativeBridge.resetVIO()
        runCatching { NativeBridge.nativeResetEventCounters() }
        orientationTracker.reset()
        _vioState.value = VioData()
        // Phase 1 camera-overlay plumbing: drop cached analyzer geometry so
        // the overlay hides until the first new frame after reset republishes
        // it. The Composable returns early when this is null.
        _cameraFrameGeometry.value = null
        // Phase 2/3/4 overlay plumbing: clear the bundled snapshot so the
        // overlay's KLT dots, SLAM dots, and any in-flight loop-closure
        // flash all hide instantly on reset. Republished by the next
        // native frame after startSensors() completes.
        _overlaySnapshot.value = OverlaySnapshot(
            floatArrayOf(), intArrayOf(), floatArrayOf(), floatArrayOf(), 0L
        )

        // 4. Restart everything in the same order startSensors() does
        //    at app launch.
        startSensors()
        // GPS depends on the permission flag the activity sets; preserve it.
        startGpsUpdates(hasLocationPermission)
        requestInitialLocation(hasLocationPermission)
    }

    // DEAD CODE: never called — ViewModel calls NativeBridge.setScale() directly
    // fun setScale(scale: Double) {
    //     NativeBridge.setScale(scale)
    // }
}
