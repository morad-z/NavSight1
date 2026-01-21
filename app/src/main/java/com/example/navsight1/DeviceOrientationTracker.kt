package com.example.navsight1

import android.hardware.SensorManager
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Device Orientation Tracker - מעקב מדויק אחר זווית המכשיר
 * מזהה אם הטלפון במצב אופקי (מקביל לרצפה, מצלמה למטה) עם tolerance מתאים
 */
class DeviceOrientationTracker {
    
    companion object {
        private const val TAG = "OrientationTracker"
        
        // Tolerance להחזקת הטלפון אופקית (מקביל לרצפה)
        private const val HORIZONTAL_TOLERANCE_DEGREES = 25f  // סובלנות של ±25 מעלות
        
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
        val isHorizontal: Boolean,  // האם הטלפון אופקי (מקביל לרצפה)
        val deviationFromHorizontal: Float, // כמה רחוק ממצב אופקי
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
            return OrientationResult(
                pitch = smoothedPitch,
                roll = smoothedRoll,
                azimuth = smoothedAzimuth,
                isHorizontal = false,
                deviationFromHorizontal = 90f,
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
        
        // בדיקה אם הטלפון אופקי (מקביל לרצפה)
        val deviationFromHorizontal = calculateHorizontalDeviation()
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
     * חישוב סטייה ממצב אופקי
     * מצב אופקי = הטלפון מקביל לרצפה, המצלמה מכוונת למטה
     */
    private fun calculateHorizontalDeviation(): Float {
        val ax = accelerometerReading[0]
        val ay = accelerometerReading[1]
        val az = accelerometerReading[2]
        
        val totalAccel = sqrt(ax * ax + ay * ay + az * az)
        
        if (totalAccel < 0.1f) return 90f
        
        // נורמליזציה
        val normZ = az / totalAccel
        
        // כשהטלפון אופקי לחלוטין (מקביל לרצפה, מסך למעלה):
        // az צריך להיות קרוב ל-g (~9.8) כלומר normZ קרוב ל-1
        // כשהטלפון אופקי עם המסך למטה (מצלמה לרצפה):
        // az צריך להיות קרוב ל--g (~-9.8) כלומר normZ קרוב ל--1
        
        // אנחנו רוצים לזהות את שני המצבים - מסך למעלה או מסך למטה
        val absNormZ = abs(normZ)
        
        // חישוב הזווית מהמצב האופקי
        // acos(absNormZ) נותן את הזווית בין ציר Z לכיוון הגרביטציה
        val angleFromHorizontal = Math.toDegrees(kotlin.math.acos(absNormZ.coerceIn(0f, 1f)).toDouble()).toFloat()
        
        return angleFromHorizontal
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
     * בדיקה מהירה אם הטלפון אופקי
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