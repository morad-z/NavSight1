package com.example.navsight1

import android.hardware.SensorManager
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Device Orientation Tracker - מעקב מדויק אחר זווית המכשיר
 * מזהה אם הטלפון מוחזק במצב VIO אופטימלי (מוטה קדימה ~30-60° מהאנך)
 * כך שהמצלמה רואה את הסצנה קדימה ולא את הרצפה
 */
class DeviceOrientationTracker {

    companion object {
        private const val TAG = "OrientationTracker"

        // VIO-optimal tilt range: phone held upright, tilted 30-60° forward from vertical.
        // Android pitch: -90° = upright, 0° = horizontal (floor). Ideal range: -75° to -30°.
        private const val VIO_IDEAL_PITCH_MIN = -75f  // most upright acceptable
        private const val VIO_IDEAL_PITCH_MAX = -30f  // most tilted acceptable
        private const val HORIZONTAL_TOLERANCE_DEGREES = 25f  // used for deviation calc

        // Smoothing
        private const val SMOOTHING_ALPHA = 0.15f
    }
    
    // ערכי חיישנים
    private val accelerometerReading = FloatArray(3)
    private val magnetometerReading = FloatArray(3)
    private val rotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3)
    
    // זוויות מוחלקות
    private var smoothedPitch = 0f
    private var smoothedRoll = 0f
    private var smoothedAzimuth = 0f
    
    // היסטוריה לייצוב
    private val pitchHistory = mutableListOf<Float>()
    private val historySize = 10
    
    /**
     * תוצאת מעקב אחר אוריינטציה
     */
    data class OrientationResult(
        val pitch: Float,           // זווית קדימה/אחורה (מעלות)
        val roll: Float,            // זווית צד (מעלות)
        val azimuth: Float,         // כיוון מצפן (מעלות)
        val isHorizontal: Boolean,  // האם הטלפון במצב VIO תקין (מוטה קדימה, מצלמה לסצנה)
        val deviationFromHorizontal: Float, // כמה רחוק מהמצב האופטימלי
        val stabilityScore: Float   // ציון יציבות (0-1)
    )
    
    /**
     * עדכון ערכי אקסלרומטר
     */
    fun updateAccelerometer(values: FloatArray) {
        System.arraycopy(values, 0, accelerometerReading, 0, 3)
    }
    
    /**
     * עדכון ערכי מגנטומטר
     */
    fun updateMagnetometer(values: FloatArray) {
        System.arraycopy(values, 0, magnetometerReading, 0, 3)
    }
    
    /**
     * חישוב אוריינטציה נוכחית
     */
    fun getOrientation(): OrientationResult {
        // חישוב מטריצת סיבוב
        val success = SensorManager.getRotationMatrix(
            rotationMatrix, null, 
            accelerometerReading, magnetometerReading
        )
        
        if (!success) {
            // getRotationMatrix fails when magnetometer is unavailable (all zeros) or at gimbal lock.
            // Either way, use the raw accelerometer to determine tilt — it works without magnetometer.
            val deviation = calculateVioDeviation()
            return OrientationResult(
                pitch = smoothedPitch,
                roll = smoothedRoll,
                azimuth = smoothedAzimuth,
                isHorizontal = deviation <= HORIZONTAL_TOLERANCE_DEGREES,
                deviationFromHorizontal = deviation,
                stabilityScore = 0f
            )
        }
        
        // חישוב זוויות
        SensorManager.getOrientation(rotationMatrix, orientationAngles)
        
        val azimuth = Math.toDegrees(orientationAngles[0].toDouble()).toFloat()
        val pitch = Math.toDegrees(orientationAngles[1].toDouble()).toFloat()
        val roll = Math.toDegrees(orientationAngles[2].toDouble()).toFloat()
        
        // החלקה
        smoothedAzimuth = smoothAngle(smoothedAzimuth, azimuth)
        smoothedPitch = smoothedPitch * (1 - SMOOTHING_ALPHA) + pitch * SMOOTHING_ALPHA
        smoothedRoll = smoothedRoll * (1 - SMOOTHING_ALPHA) + roll * SMOOTHING_ALPHA
        
        // הוספה להיסטוריה לחישוב יציבות
        pitchHistory.add(smoothedPitch)
        if (pitchHistory.size > historySize) {
            pitchHistory.removeAt(0)
        }
        
        // חישוב יציבות
        val stability = calculateStability()
        
        // בדיקה אם הטלפון במצב VIO אופטימלי (מוטה קדימה)
        val deviationFromHorizontal = calculateVioDeviation()
        val isHorizontal = deviationFromHorizontal <= HORIZONTAL_TOLERANCE_DEGREES
        
        return OrientationResult(
            pitch = smoothedPitch,
            roll = smoothedRoll,
            azimuth = normalizeAzimuth(smoothedAzimuth),
            isHorizontal = isHorizontal,
            deviationFromHorizontal = deviationFromHorizontal,
            stabilityScore = stability
        )
    }
    
    /**
     * חישוב סטייה מהמצב האופטימלי ל-VIO
     * מצב אופטימלי = הטלפון מוחזק כמו בשימוש רגיל, מוטה קדימה 30-60° מהאנך
     * כך שהמצלמה רואה את הסצנה קדימה (לא את הרצפה)
     *
     * Android pitch values:
     *   -90° = phone fully upright (screen facing user)
     *     0° = phone flat/horizontal (screen facing ceiling, camera at floor)
     *
     * VIO ideal range: pitch between -75° and -30°
     * (phone tilted 30-60° forward from vertical = camera sees forward scene)
     */
    private fun calculateVioDeviation(): Float {
        val pitch = smoothedPitch

        // If pitch is in the ideal VIO range, deviation = 0
        if (pitch in VIO_IDEAL_PITCH_MIN..VIO_IDEAL_PITCH_MAX) {
            return 0f
        }

        // How far outside the ideal range
        return if (pitch > VIO_IDEAL_PITCH_MAX) {
            // Phone too tilted toward horizontal (camera at floor)
            pitch - VIO_IDEAL_PITCH_MAX
        } else {
            // Phone too upright (camera at sky/face)
            VIO_IDEAL_PITCH_MIN - pitch
        }
    }
    
    /**
     * חישוב יציבות המכשיר
     */
    private fun calculateStability(): Float {
        if (pitchHistory.size < 3) return 0f
        
        var totalVariance = 0f
        val avg = pitchHistory.average().toFloat()
        
        for (value in pitchHistory) {
            totalVariance += (value - avg) * (value - avg)
        }
        
        val variance = totalVariance / pitchHistory.size
        
        // ממיר variance לציון יציבות (0-1)
        // variance נמוך = יציבות גבוהה
        return (1f - (variance / 100f).coerceIn(0f, 1f))
    }
    
    /**
     * החלקת זווית (מטפל ב-wraparound של 360 מעלות)
     */
    private fun smoothAngle(current: Float, target: Float): Float {
        var delta = target - current
        
        if (delta < -180) delta += 360
        if (delta > 180) delta -= 360
        
        var result = current + delta * SMOOTHING_ALPHA
        if (result < 0) result += 360
        if (result >= 360) result -= 360
        
        return result
    }
    
    /**
     * נורמליזציה של אזימוט ל-0-360
     */
    private fun normalizeAzimuth(azimuth: Float): Float {
        var normalized = azimuth
        while (normalized < 0) normalized += 360
        while (normalized >= 360) normalized -= 360
        return normalized
    }
    
    /**
     * בדיקה מהירה אם הטלפון במצב VIO תקין
     */
    fun isPhoneHorizontal(): Boolean {
        return getOrientation().isHorizontal
    }
    
    /**
     * קבלת כיוון מצפן (0-360)
     */
    fun getCompassHeading(): Float {
        return normalizeAzimuth(smoothedAzimuth)
    }
    
    /**
     * איפוס
     */
    fun reset() {
        smoothedPitch = 0f
        smoothedRoll = 0f
        smoothedAzimuth = 0f
        pitchHistory.clear()
    }
}