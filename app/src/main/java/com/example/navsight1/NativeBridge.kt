package com.example.navsight1

import com.example.navsight1.VioData

object NativeBridge {
    init {
        System.loadLibrary("navsight")
    }

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
}
