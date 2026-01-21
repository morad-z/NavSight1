package com.example.navsight1

/**
 * VIO Data class - מחזיק את כל המידע מה-Visual Inertial Odometry
 * 
 * משמש בעיקר אם אתה רוצה לחזור להשתמש ב-C++ VIO בעתיד
 */
data class VioData(
    // Position (meters)
    val x: Double = 0.0,
    val y: Double = 0.0,
    val z: Double = 0.0,
    
    // Rotation (radians)
    val roll: Double = 0.0,
    val pitch: Double = 0.0,
    val yaw: Double = 0.0,
    
    // Tracking info
    val trackingQuality: Double = 0.0,
    val trackedFeatures: Int = 0,
    val totalFeatures: Int = 0,
    val estimatedScale: Double = 1.0,
    
    // Status
    val isInitialized: Boolean = false,
    
    // Tracked points for visualization
    val trackedPoints: FloatArray = floatArrayOf(),
    
    // IMU data
    val accelX: Float = 0f,
    val accelY: Float = 0f,
    val accelZ: Float = 0f,
    val gyroX: Float = 0f,
    val gyroY: Float = 0f,
    val gyroZ: Float = 0f
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
        if (trackingQuality != other.trackingQuality) return false
        if (trackedFeatures != other.trackedFeatures) return false
        if (totalFeatures != other.totalFeatures) return false
        if (estimatedScale != other.estimatedScale) return false
        if (isInitialized != other.isInitialized) return false
        if (!trackedPoints.contentEquals(other.trackedPoints)) return false
        if (accelX != other.accelX) return false
        if (accelY != other.accelY) return false
        if (accelZ != other.accelZ) return false
        if (gyroX != other.gyroX) return false
        if (gyroY != other.gyroY) return false
        if (gyroZ != other.gyroZ) return false

        return true
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        result = 31 * result + z.hashCode()
        result = 31 * result + roll.hashCode()
        result = 31 * result + pitch.hashCode()
        result = 31 * result + yaw.hashCode()
        result = 31 * result + trackingQuality.hashCode()
        result = 31 * result + trackedFeatures
        result = 31 * result + totalFeatures
        result = 31 * result + estimatedScale.hashCode()
        result = 31 * result + isInitialized.hashCode()
        result = 31 * result + trackedPoints.contentHashCode()
        result = 31 * result + accelX.hashCode()
        result = 31 * result + accelY.hashCode()
        result = 31 * result + accelZ.hashCode()
        result = 31 * result + gyroX.hashCode()
        result = 31 * result + gyroY.hashCode()
        result = 31 * result + gyroZ.hashCode()
        return result
    }
}
