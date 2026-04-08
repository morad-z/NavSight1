package com.example.navsight1

import android.util.Log
import com.example.navsight1.VioData

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
    external fun processCameraFrame(
        frameData: ByteArray, width: Int, height: Int, timestamp: Long
    ): VioData
    external fun processGyroscope(timestamp: Long, x: Float, y: Float, z: Float)
    external fun processAccelerometer(timestamp: Long, x: Float, y: Float, z: Float)
    external fun resetVIO()
    external fun setScale(scale: Double)
    external fun setIntrinsics(fx: Double, fy: Double, cx: Double, cy: Double)
    external fun setInitialHeading(azimuthRad: Double)
    external fun setUserHeight(heightM: Float)
    external fun setMagnetometerHeading(yawRad: Float)
    external fun updateDepthScale(scale: Double, confidence: Double)
}
