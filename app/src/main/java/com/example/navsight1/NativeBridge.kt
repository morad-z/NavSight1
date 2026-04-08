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
}
