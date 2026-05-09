package com.example.navsight1

import android.util.Log
import com.example.navsight1.VioData
import java.nio.ByteBuffer

object NativeBridge {
    private const val TAG = "NativeBridge"
    private var isLibraryLoaded = false

    init {
        try {
            System.loadLibrary("navsight")
            isLibraryLoaded = true
            Log.d(TAG, "Native library 'navsight' loaded successfully")
        } catch (e: UnsatisfiedLinkError) {
            Log.e(TAG, "Failed to load native library 'navsight': ${e.message}", e)
            isLibraryLoaded = false
        } catch (e: Exception) {
            Log.e(TAG, "Unexpected error loading native library: ${e.message}", e)
            isLibraryLoaded = false
        }
    }

    fun isLoaded(): Boolean = isLibraryLoaded

    external fun startVIO()
    external fun stopVIO()
    // DEAD CODE: old ByteArray version — superseded by processCameraFrameDirect (zero-copy ByteBuffer)
    // external fun processCameraFrame(
    //     frameData: ByteArray, width: Int, height: Int, timestamp: Long
    // ): VioData
    // Zero-copy: accepts direct ByteBuffer from CameraX ImageProxy.
    // rollingShutterSkewNs: Camera2 CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW
    // (API level 21+) — nanoseconds from first-row to last-row read-out.
    // Pass 0 if the key is absent (global-shutter device or unavailable).
    external fun processCameraFrameDirect(
        yBuffer: ByteBuffer, uvBuffer: ByteBuffer,
        width: Int, height: Int,
        yRowStride: Int, uvRowStride: Int, uvPixelStride: Int,
        timestamp: Long,
        rollingShutterSkewNs: Long
    ): VioData
    external fun processGyroscope(timestamp: Long, x: Float, y: Float, z: Float)
    external fun processAccelerometer(timestamp: Long, x: Float, y: Float, z: Float)
    external fun resetVIO()
    external fun setScale(scale: Double)
    external fun setDepthMap(depthData: FloatArray, width: Int, height: Int)
    external fun setIntrinsics(fx: Double, fy: Double, cx: Double, cy: Double)
    external fun setInitialHeading(azimuthRad: Double)
    external fun setUserHeight(heightM: Float)
    // DEAD CODE: never called from Kotlin — mag heading is captured via orientationTracker at init, not via this JNI
    // external fun setMagnetometerHeading(yawRad: Float)

    // Step 5: Calibration & Initialization
    // Returns 0=WAIT_STATIONARY, 1=WAIT_MOTION, 2=READY, 3=TIMEOUT_NEEDS_USER
    external fun getInitStatus(): Int
    external fun clearInitTimeout()
    // rotation: 9 floats row-major; gyroBias: 3 floats; accelBias: 3 floats
    external fun loadStoredCalibration(
        rotation: FloatArray, gyroBias: FloatArray, accelBias: FloatArray
    )
    // Returns true if calibration was filled (gate has passed); false otherwise
    external fun getCalibration(
        rotation: FloatArray, gyroBias: FloatArray, accelBias: FloatArray
    ): Boolean

    // Step 6: Horizontal-plane position covariance in m².
    // out = [σ_xx, σ_xz, σ_zz] (length 3). Returns true once the EKF has
    // finished its full init; false (zeros) before that.
    external fun getPositionCovariance(out: FloatArray): Boolean

    // Step 1 (Visual Production Plan): pass the absolute filesystem path of
    // the in-app camera calibration JSON. Native side parses and validates,
    // then pushes intrinsics + distortion into the VIO pipeline. Returns
    // true on success; false if the file is missing or fails validation
    // (engine then keeps the zero-distortion passthrough).
    external fun nativeLoadCalibration(path: String): Boolean

    // Step 7 (Visual Production Plan, ADR-013): pass the absolute filesystem
    // path of the ORB DBoW2 vocabulary (copied at app startup from
    // assets/ORBvoc.bin into <filesDir>/ORBvoc.bin — AssetManager paths are
    // not real filesystem paths and the DBoW2 reader needs to fopen). On
    // success the native LoopClosureDetector becomes ready and the Tracker
    // launches its 1 Hz query worker thread; on false the rest of the VIO
    // pipeline keeps running with loop closure disabled.
    external fun nativeLoadLoopClosureVocabulary(path: String): Boolean

    // EventCounters bridge — used by NavSightViewModel.saveSimulationData to
    // embed a per-event verdict (RELOC_ORB / BLUR / LOWLIGHT / ROT_GATE /
    // KLT / BA / MSCKF counts) into simulation_data_<ts>.json. Replaces the
    // old adb-logcat capture pipeline so untethered walks still produce a
    // verifiable per-step breakdown. See app/src/main/cpp/EventCounters.h
    // for the JSON schema.
    external fun nativeGetEventCountersJson(): String
    external fun nativeResetEventCounters()

    // Step 8b (Visual Production Plan): seed the EKF body→camera extrinsics
    // rotation R_bc from Android CameraCharacteristics.SENSOR_ORIENTATION.
    // R_bc_flat: 9 floats in row-major order.  Called once after camera open,
    // before the first frame arrives.  No-op if VIO is not yet started.
    external fun nativeSetExtrinsicsRotation(R_bc_flat: FloatArray)

    // ── Camera overlay Phase 2 / 3 / 4 (see camera_overlay_phase23_plan.md) ──

    // Phase 2: per-feature ages (frames survived) for the most recent VIO
    // frame. Pairs 1:1 with VioData.trackedPoints (length =
    // trackedPoints.size / 2). Returns the number of ages written into
    // `out`. Caller preallocates IntArray(MAX_FEATURES) once and reuses.
    // At 30 Hz: <30 frames is "new" (≤1 s), 30-89 is "established"
    // (1-3 s), ≥90 is "mature" (≥3 s) for the overlay color ramp.
    external fun getLastTrackedPointAges(out: IntArray): Int

    // Phase 3: flat snapshot of currently-active SLAM features.
    // Layout per feature (4 floats): [feature_id, world_x, world_y, world_z]
    // in Y-up world frame (X=East, Y=Up, Z=North) — same convention as the
    // existing Kotlin pose pipeline. Returns the number of features written
    // (≤ out.size / 4). Caller preallocates FloatArray(48) (4 ×
    // MAX_SLAM_FEATURES = 12). Returns 0 if EKF has no SLAM features yet.
    external fun getSlamSnapshot(out: FloatArray): Int

    // Phase 3: current camera pose for SLAM-point reprojection. 16 floats:
    //   [0..8]   R_world_cam row-major (camera→world rotation, Y-up world)
    //   [9..11]  t_world_cam (camera position in Y-up world)
    //   [12..15] fx, fy, cx, cy
    // Returns true on success, false if the EKF has not yet reached full
    // initialisation. Caller preallocates FloatArray(16) once and reuses.
    external fun getCurrentCameraPose(out: FloatArray): Boolean

    // Phase 4: cumulative loop-closure correction count from
    // EventCounters.loop_closure_corrections_applied. Polled per VIO
    // frame by the overlay; an increment triggers the 1-second
    // "LOOP CLOSURE" flash banner. Lock-free atomic read — cheap to
    // call every frame.
    external fun getLoopClosureCorrectionsApplied(): Long
}
