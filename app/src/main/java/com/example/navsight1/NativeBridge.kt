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
    // Zero-copy: accepts direct ByteBuffer from CameraX ImageProxy
    external fun processCameraFrameDirect(
        yBuffer: ByteBuffer, uvBuffer: ByteBuffer,
        width: Int, height: Int,
        yRowStride: Int, uvRowStride: Int, uvPixelStride: Int,
        timestamp: Long
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

    // EventCounters bridge — used by NavSightViewModel.saveSimulationData to
    // embed a per-event verdict (RELOC_ORB / BLUR / LOWLIGHT / ROT_GATE /
    // KLT / BA / MSCKF counts) into simulation_data_<ts>.json. Replaces the
    // old adb-logcat capture pipeline so untethered walks still produce a
    // verifiable per-step breakdown. See app/src/main/cpp/EventCounters.h
    // for the JSON schema.
    external fun nativeGetEventCountersJson(): String
    external fun nativeResetEventCounters()
}
