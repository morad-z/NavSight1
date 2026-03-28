package com.example.navsight1

import android.util.Log
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Optical Flow Processor - מזהה תנועה מתמונות המצלמה
 * תומך בהליכה (איטי) ונסיעה (מהיר)
 */
class OpticalFlowProcessor {
    
    companion object {
        private const val TAG = "OpticalFlow"
        
        // הגדרות גריד
        private const val GRID_SIZE = 5
        
        // סף תנועה - נמוך מאוד כדי לתפוס גם תנועות קטנות
        private const val MIN_MOVEMENT_WALK = 0.8f    // סף להליכה
        private const val MIN_MOVEMENT_DRIVE = 2.0f   // סף לנסיעה
        
        // החלקה
        private const val SMOOTHING_WALK = 0.3f       // החלקה חזקה להליכה
        private const val SMOOTHING_DRIVE = 0.5f      // החלקה מהירה לנסיעה
        
        // סף מהירות להבחנה בין הליכה לנסיעה
        private const val DRIVE_SPEED_THRESHOLD = 15f
    }
    
    // תמונה קודמת
    private var previousFrame: IntArray? = null
    private var previousWidth = 0
    private var previousHeight = 0

    // Reusable grayscale buffer — allocated once per resolution to avoid per-frame GC pressure
    private var grayscaleBuffer: IntArray? = null
    
    // תנועה מוחלקת
    private var smoothedDx = 0f
    private var smoothedDy = 0f
    
    // היסטוריה
    private val dxHistory = mutableListOf<Float>()
    private val dyHistory = mutableListOf<Float>()
    private val magnitudeHistory = mutableListOf<Float>()
    private val historySize = 10
    
    // מצב תנועה נוכחי
    private var currentMode = MovementMode.WALKING
    
    // ספירת פריימים
    private var frameCount = 0
    private var lastLogTime = 0L
    
    /**
     * מצב תנועה
     */
    enum class MovementMode {
        WALKING,  // הליכה
        DRIVING   // נסיעה
    }
    
    /**
     * תוצאת Flow
     */
    data class FlowResult(
        val dx: Float,
        val dy: Float,
        val magnitude: Float,
        val direction: MovementDirection,
        val confidence: Float,
        val mode: MovementMode = MovementMode.WALKING,
        val rawPoints: List<Pair<Float, Float>> = emptyList()
    )
    
    enum class MovementDirection {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        STOPPED
    }
    
    /**
     * עיבוד פריים
     */
    fun processFrame(frame: ByteArray, width: Int, height: Int, isYuv: Boolean = true): FlowResult {
        frameCount++
        
        // המרה לגריי-סקייל
        val grayscale = if (isYuv) {
            yuvToGrayscale(frame, width, height)
        } else {
            rgbToGrayscale(frame, width, height)
        }
        
        // אם אין פריים קודם
        if (previousFrame == null || previousWidth != width || previousHeight != height) {
            previousFrame = grayscale
            previousWidth = width
            previousHeight = height
            return FlowResult(0f, 0f, 0f, MovementDirection.STOPPED, 0f)
        }
        
        // חישוב Optical Flow
        val flowVectors = calculateOpticalFlow(previousFrame!!, grayscale, width, height)
        
        // עדכון פריים קודם
        previousFrame = grayscale
        
        if (flowVectors.isEmpty()) {
            decaySmoothedValues()
            return FlowResult(smoothedDx, smoothedDy, 0f, MovementDirection.STOPPED, 0f, currentMode)
        }
        
        // חישוב ממוצע עם סינון
        val (avgDx, avgDy, validCount) = calculateRobustAverage(flowVectors)
        
        if (validCount < 2) {
            decaySmoothedValues()
            return FlowResult(smoothedDx, smoothedDy, 0f, MovementDirection.STOPPED, 0f, currentMode)
        }
        
        // הוספה להיסטוריה
        dxHistory.add(avgDx)
        dyHistory.add(avgDy)
        if (dxHistory.size > historySize) dxHistory.removeAt(0)
        if (dyHistory.size > historySize) dyHistory.removeAt(0)
        
        // ממוצע נע
        val histAvgDx = dxHistory.average().toFloat()
        val histAvgDy = dyHistory.average().toFloat()
        
        // חישוב מהירות גולמית לזיהוי מצב
        val rawMagnitude = sqrt(histAvgDx * histAvgDx + histAvgDy * histAvgDy)
        
        // עדכון היסטוריית מהירות
        magnitudeHistory.add(rawMagnitude)
        if (magnitudeHistory.size > historySize) magnitudeHistory.removeAt(0)
        
        // זיהוי מצב תנועה (הליכה או נסיעה)
        val avgMagnitude = magnitudeHistory.average().toFloat()
        currentMode = if (avgMagnitude > DRIVE_SPEED_THRESHOLD) {
            MovementMode.DRIVING
        } else {
            MovementMode.WALKING
        }
        
        // בחירת פרמטרים לפי מצב
        val smoothingFactor = if (currentMode == MovementMode.DRIVING) SMOOTHING_DRIVE else SMOOTHING_WALK
        val minMovement = if (currentMode == MovementMode.DRIVING) MIN_MOVEMENT_DRIVE else MIN_MOVEMENT_WALK
        
        // החלקה
        smoothedDx = smoothedDx * (1 - smoothingFactor) + histAvgDx * smoothingFactor
        smoothedDy = smoothedDy * (1 - smoothingFactor) + histAvgDy * smoothingFactor
        
        val magnitude = sqrt(smoothedDx * smoothedDx + smoothedDy * smoothedDy)
        
        // קביעת כיוון
        val direction = determineDirection(smoothedDx, smoothedDy, magnitude, minMovement)
        
        // ביטחון
        val confidence = (validCount.toFloat() / flowVectors.size).coerceIn(0f, 1f)
        
        // לוג
        val now = System.currentTimeMillis()
        if (now - lastLogTime > 500) {
            lastLogTime = now
            Log.d(TAG, "Mode: $currentMode, Dir: $direction, Mag: ${"%.1f".format(magnitude)}, dx: ${"%.1f".format(smoothedDx)}, dy: ${"%.1f".format(smoothedDy)}")
        }
        
        return FlowResult(
            dx = smoothedDx,
            dy = smoothedDy,
            magnitude = magnitude,
            direction = direction,
            confidence = confidence,
            mode = currentMode,
            rawPoints = flowVectors
        )
    }
    
    /**
     * דעיכה הדרגתית של ערכים כשאין תנועה
     */
    private fun decaySmoothedValues() {
        smoothedDx *= 0.9f
        smoothedDy *= 0.9f
    }
    
    /**
     * קביעת כיוון התנועה
     */
    private fun determineDirection(dx: Float, dy: Float, magnitude: Float, minMovement: Float): MovementDirection {
        if (magnitude < minMovement) {
            return MovementDirection.STOPPED
        }
        
        val absDx = abs(dx)
        val absDy = abs(dy)
        
        // כשהמצלמה מכוונת לרצפה:
        // dy חיובי (רצפה זזה למטה) = קדימה
        // dy שלילי (רצפה זזה למעלה) = אחורה
        // dx חיובי (רצפה זזה ימינה) = שמאלה
        // dx שלילי (רצפה זזה שמאלה) = ימינה
        
        return if (absDy > absDx * 0.7f) {
            // תנועה אנכית דומיננטית
            if (dy > 0) MovementDirection.FORWARD else MovementDirection.BACKWARD
        } else if (absDx > absDy * 0.7f) {
            // תנועה אופקית דומיננטית
            if (dx > 0) MovementDirection.LEFT else MovementDirection.RIGHT
        } else {
            // אלכסון - בוחרים את הגדול יותר
            if (absDy >= absDx) {
                if (dy > 0) MovementDirection.FORWARD else MovementDirection.BACKWARD
            } else {
                if (dx > 0) MovementDirection.LEFT else MovementDirection.RIGHT
            }
        }
    }
    
    /**
     * חישוב ממוצע עם סינון outliers
     */
    private fun calculateRobustAverage(flowVectors: List<Pair<Float, Float>>): Triple<Float, Float, Int> {
        if (flowVectors.isEmpty()) return Triple(0f, 0f, 0)

        // Need at least 4 points for meaningful IQR; fall back to plain median otherwise
        if (flowVectors.size < 4) {
            val dx = flowVectors.map { it.first }.sorted()[flowVectors.size / 2]
            val dy = flowVectors.map { it.second }.sorted()[flowVectors.size / 2]
            return Triple(dx, dy, flowVectors.size)
        }

        // מיון לחישוב median
        val sortedDx = flowVectors.map { it.first }.sorted()
        val sortedDy = flowVectors.map { it.second }.sorted()

        val medianDx = sortedDx[sortedDx.size / 2]
        val medianDy = sortedDy[sortedDy.size / 2]

        // IQR לסינון
        val q1Dx = sortedDx[sortedDx.size / 4]
        val q3Dx = sortedDx[3 * sortedDx.size / 4]
        val iqrDx = (q3Dx - q1Dx) * 1.5f + 10f
        
        val q1Dy = sortedDy[sortedDy.size / 4]
        val q3Dy = sortedDy[3 * sortedDy.size / 4]
        val iqrDy = (q3Dy - q1Dy) * 1.5f + 10f
        
        var totalDx = 0f
        var totalDy = 0f
        var validPoints = 0
        
        for ((dx, dy) in flowVectors) {
            if (abs(dx - medianDx) <= iqrDx && abs(dy - medianDy) <= iqrDy) {
                totalDx += dx
                totalDy += dy
                validPoints++
            }
        }
        
        return if (validPoints > 0) {
            Triple(totalDx / validPoints, totalDy / validPoints, validPoints)
        } else {
            Triple(medianDx, medianDy, 1)
        }
    }
    
    /**
     * YUV לגריי-סקייל
     * Reuses a pre-allocated buffer to avoid per-frame GC pressure.
     */
    private fun yuvToGrayscale(yuv: ByteArray, width: Int, height: Int): IntArray {
        val size = width * height
        val grayscale = if (grayscaleBuffer?.size == size) {
            grayscaleBuffer!!.fill(0)  // zero-fill so truncated frames leave unused pixels at 0
            grayscaleBuffer!!
        } else {
            IntArray(size).also { grayscaleBuffer = it }
        }
        val limit = minOf(size, yuv.size)
        for (i in 0 until limit) {
            grayscale[i] = yuv[i].toInt() and 0xFF
        }
        return grayscale
    }
    
    /**
     * RGB לגריי-סקייל
     */
    private fun rgbToGrayscale(rgb: ByteArray, width: Int, height: Int): IntArray {
        val grayscale = IntArray(width * height)
        for (i in 0 until width * height) {
            val idx = i * 3
            if (idx + 2 < rgb.size) {
                val r = rgb[idx].toInt() and 0xFF
                val g = rgb[idx + 1].toInt() and 0xFF
                val b = rgb[idx + 2].toInt() and 0xFF
                grayscale[i] = (0.299 * r + 0.587 * g + 0.114 * b).toInt()
            }
        }
        return grayscale
    }
    
    /**
     * Optical Flow עם Block Matching
     */
    private fun calculateOpticalFlow(
        prev: IntArray,
        curr: IntArray,
        width: Int,
        height: Int
    ): List<Pair<Float, Float>> {
        val flowVectors = mutableListOf<Pair<Float, Float>>()
        
        // פרמטרים מותאמים לתנועה מהירה
        val blockSize = 12
        val searchRadius = 32  // רדיוס גדול לתנועה מהירה
        
        // אזור עניין - חלק תחתון-מרכזי (הרצפה/כביש)
        val marginX = width / 8
        val startY = height / 4
        val endY = height - blockSize - 10
        val startX = marginX
        val endX = width - marginX - blockSize
        
        val stepX = (endX - startX) / GRID_SIZE
        val stepY = (endY - startY) / GRID_SIZE

        // Image too small to form a valid grid
        if (stepX <= 0 || stepY <= 0) return flowVectors

        for (gridY in 0 until GRID_SIZE) {
            for (gridX in 0 until GRID_SIZE) {
                val x = startX + gridX * stepX
                val y = startY + gridY * stepY
                
                // בדיקת גבולות
                if (x < searchRadius || x >= width - blockSize - searchRadius ||
                    y < searchRadius || y >= height - blockSize - searchRadius) {
                    continue
                }
                
                // חיפוש התאמה
                var bestDx = 0
                var bestDy = 0
                var bestSad = Int.MAX_VALUE
                
                // חיפוש בצעדים - קודם גס ואז עדין
                // שלב 1: חיפוש גס
                for (dy in -searchRadius..searchRadius step 4) {
                    for (dx in -searchRadius..searchRadius step 4) {
                        val sad = calculateSAD(prev, curr, width, x, y, x + dx, y + dy, blockSize)
                        if (sad < bestSad) {
                            bestSad = sad
                            bestDx = dx
                            bestDy = dy
                        }
                    }
                }
                
                // שלב 2: חיפוש עדין סביב הנקודה הטובה ביותר
                val fineDx = bestDx
                val fineDy = bestDy
                for (dy in (fineDy - 4)..(fineDy + 4) step 1) {
                    for (dx in (fineDx - 4)..(fineDx + 4) step 1) {
                        if (x + dx < 0 || x + dx >= width - blockSize ||
                            y + dy < 0 || y + dy >= height - blockSize) continue
                        
                        val sad = calculateSAD(prev, curr, width, x, y, x + dx, y + dy, blockSize)
                        if (sad < bestSad) {
                            bestSad = sad
                            bestDx = dx
                            bestDy = dy
                        }
                    }
                }
                
                // הוספה אם יש התאמה טובה ותנועה
                val threshold = blockSize * blockSize * 25
                if (bestSad < threshold && (bestDx != 0 || bestDy != 0)) {
                    flowVectors.add(Pair(bestDx.toFloat(), bestDy.toFloat()))
                }
            }
        }
        
        return flowVectors
    }
    
    /**
     * Sum of Absolute Differences
     */
    private fun calculateSAD(
        prev: IntArray,
        curr: IntArray,
        width: Int,
        x1: Int, y1: Int,
        x2: Int, y2: Int,
        blockSize: Int
    ): Int {
        var sad = 0
        for (by in 0 until blockSize step 2) {
            for (bx in 0 until blockSize step 2) {
                val idx1 = (y1 + by) * width + (x1 + bx)
                val idx2 = (y2 + by) * width + (x2 + bx)
                
                if (idx1 in prev.indices && idx2 in curr.indices) {
                    sad += abs(prev[idx1] - curr[idx2])
                } else {
                    sad += 128
                }
            }
        }
        return sad
    }
    
    /**
     * איפוס
     */
    fun reset() {
        previousFrame = null
        grayscaleBuffer = null
        smoothedDx = 0f
        smoothedDy = 0f
        dxHistory.clear()
        dyHistory.clear()
        magnitudeHistory.clear()
        currentMode = MovementMode.WALKING
        frameCount = 0
    }
    
    /**
     * קבלת מצב נוכחי
     */
    fun getCurrentMode(): MovementMode = currentMode
}