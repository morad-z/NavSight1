package com.example.navsight1

data class VioData(
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0,
    val roll: Double = 0.0,
    val pitch: Double = 0.0,
    val yaw: Double = 0.0,
    val trackedPoints: FloatArray = floatArrayOf(),
    val trackingQuality: Double = 0.0,
    val trackedFeatures: Int = 0,
    val totalFeatures: Int = 0,
    val estimatedScale: Double = 1.0,
    val isInitialized: Boolean = false,
    // Raw IMU data
    val accelX: Float = 0.0f,
    val accelY: Float = 0.0f,
    val accelZ: Float = 0.0f,
    val gyroX: Float = 0.0f,
    val gyroY: Float = 0.0f,
    val gyroZ: Float = 0.0f
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as VioData

        if (x != other.x) return false
        if (y != other.y) return false
        if (z != other.z) return false
        if (roll != other.roll) return false
        if (pitch != other.pitch) return false
        if (yaw != other.yaw) return false
        if (!trackedPoints.contentEquals(other.trackedPoints)) return false

        return true
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + z.hashCode()
        result = 31 * result + roll.hashCode()
        result = 31 * result + pitch.hashCode()
        result = 31 * result + yaw.hashCode()
        result = 31 * result + trackedPoints.contentHashCode()
        return result
    }
}