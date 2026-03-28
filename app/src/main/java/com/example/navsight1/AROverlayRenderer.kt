package com.example.navsight1

import androidx.compose.animation.core.*
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.*
import androidx.compose.ui.unit.dp
import kotlin.math.*

/**
 * AR Overlay Renderer - רנדור שכבת AR על גבי המצלמה
 * מציג כיוון תנועה, מהירות, ואינדיקטורים ויזואליים
 */
object AROverlayRenderer {
    
    // צבעים
    val ArrowGreen = Color(0xFF00E676)
    val ArrowRed = Color(0xFFFF5252)
    val ArrowYellow = Color(0xFFFFEB3B)
    val ArrowCyan = Color(0xFF00E5FF)
    val GlowColor = Color(0xFF00E676).copy(alpha = 0.3f)
    
    /**
     * Composable לרנדור חץ כיוון התנועה
     */
    @Composable
    fun DirectionArrow(
        direction: OpticalFlowProcessor.MovementDirection,
        magnitude: Float,
        modifier: Modifier = Modifier
    ) {
        // אנימציה של פעימה כשיש תנועה
        val infiniteTransition = rememberInfiniteTransition(label = "pulse")
        val pulseScale by infiniteTransition.animateFloat(
            initialValue = 1f,
            targetValue = if (direction != OpticalFlowProcessor.MovementDirection.STOPPED) 1.2f else 1f,
            animationSpec = infiniteRepeatable(
                animation = tween(500, easing = FastOutSlowInEasing),
                repeatMode = RepeatMode.Reverse
            ),
            label = "pulse"
        )
        
        // אנימציה של סיבוב לכיוון
        val targetRotation = when (direction) {
            OpticalFlowProcessor.MovementDirection.FORWARD -> 0f
            OpticalFlowProcessor.MovementDirection.BACKWARD -> 180f
            OpticalFlowProcessor.MovementDirection.LEFT -> -90f
            OpticalFlowProcessor.MovementDirection.RIGHT -> 90f
            OpticalFlowProcessor.MovementDirection.STOPPED -> 0f
        }
        
        val animatedRotation by animateFloatAsState(
            targetValue = targetRotation,
            animationSpec = spring(dampingRatio = 0.7f),
            label = "rotation"
        )
        
        val arrowColor = when (direction) {
            OpticalFlowProcessor.MovementDirection.FORWARD -> ArrowGreen
            OpticalFlowProcessor.MovementDirection.BACKWARD -> ArrowRed
            OpticalFlowProcessor.MovementDirection.LEFT, 
            OpticalFlowProcessor.MovementDirection.RIGHT -> ArrowCyan
            OpticalFlowProcessor.MovementDirection.STOPPED -> ArrowYellow.copy(alpha = 0.5f)
        }
        
        Canvas(modifier = modifier.size(120.dp)) {
            val centerX = size.width / 2
            val centerY = size.height / 2
            val arrowSize = size.minDimension / 2 * pulseScale
            
            // ציור Glow
            if (direction != OpticalFlowProcessor.MovementDirection.STOPPED) {
                drawCircle(
                    brush = Brush.radialGradient(
                        colors = listOf(arrowColor.copy(alpha = 0.3f), Color.Transparent),
                        center = Offset(centerX, centerY),
                        radius = arrowSize * 1.5f
                    ),
                    radius = arrowSize * 1.5f,
                    center = Offset(centerX, centerY)
                )
            }
            
            // ציור החץ
            rotate(animatedRotation, pivot = Offset(centerX, centerY)) {
                val path = Path().apply {
                    // ראש החץ
                    moveTo(centerX, centerY - arrowSize * 0.8f)
                    lineTo(centerX - arrowSize * 0.4f, centerY)
                    lineTo(centerX - arrowSize * 0.15f, centerY)
                    lineTo(centerX - arrowSize * 0.15f, centerY + arrowSize * 0.5f)
                    lineTo(centerX + arrowSize * 0.15f, centerY + arrowSize * 0.5f)
                    lineTo(centerX + arrowSize * 0.15f, centerY)
                    lineTo(centerX + arrowSize * 0.4f, centerY)
                    close()
                }
                
                // מילוי עם גרדיאנט
                drawPath(
                    path = path,
                    brush = Brush.verticalGradient(
                        colors = listOf(arrowColor, arrowColor.copy(alpha = 0.6f))
                    )
                )
                
                // קו מתאר
                drawPath(
                    path = path,
                    color = Color.White.copy(alpha = 0.8f),
                    style = Stroke(width = 2f)
                )
            }
            
            // מעגל מרכזי
            drawCircle(
                color = Color.White,
                radius = 8f,
                center = Offset(centerX, centerY)
            )
        }
    }
    
    /**
     * Grid AR על הרצפה
     */
    @Composable
    fun FloorGrid(
        flowResult: OpticalFlowProcessor.FlowResult,
        modifier: Modifier = Modifier
    ) {
        val infiniteTransition = rememberInfiniteTransition(label = "grid")
        val gridOffset by infiniteTransition.animateFloat(
            initialValue = 0f,
            targetValue = 50f,
            animationSpec = infiniteRepeatable(
                animation = tween(1000, easing = LinearEasing),
                repeatMode = RepeatMode.Restart
            ),
            label = "gridOffset"
        )
        
        Canvas(modifier = modifier.fillMaxSize()) {
            val gridSpacing = 80f
            val lineCount = (size.width / gridSpacing).toInt() + 2
            
            // אופסט לפי התנועה
            val offsetX = flowResult.dx * 2
            val offsetY = flowResult.dy * 2
            
            // קווים אנכיים
            for (i in 0..lineCount) {
                val x = (i * gridSpacing + offsetX + gridOffset) % size.width
                val alpha = (0.3f * (1 - abs(x - size.width / 2) / (size.width / 2))).coerceIn(0f, 0.3f)
                
                drawLine(
                    color = ArrowCyan.copy(alpha = alpha),
                    start = Offset(x, size.height * 0.5f),
                    end = Offset(x, size.height),
                    strokeWidth = 1f
                )
            }
            
            // קווים אופקיים (פרספקטיבה)
            val horizontalLines = 10
            for (i in 0..horizontalLines) {
                val progress = i.toFloat() / horizontalLines
                val y = size.height * (0.5f + progress * 0.5f)
                val perspective = progress
                val alpha = 0.2f * perspective
                
                drawLine(
                    color = ArrowCyan.copy(alpha = alpha),
                    start = Offset(size.width * (0.5f - 0.5f * perspective), y + offsetY),
                    end = Offset(size.width * (0.5f + 0.5f * perspective), y + offsetY),
                    strokeWidth = 1f
                )
            }
        }
    }
    
    /**
     * HUD מלא עם כל האינדיקטורים
     */
    @Composable
    fun FullARHud(
        flowResult: OpticalFlowProcessor.FlowResult,
        orientation: DeviceOrientationTracker.OrientationResult,
        modifier: Modifier = Modifier
    ) {
        Box(modifier = modifier.fillMaxSize()) {
            // Grid על הרצפה
            FloorGrid(
                flowResult = flowResult,
                modifier = Modifier
                    .fillMaxWidth()
                    .fillMaxHeight(0.6f)
                    .align(Alignment.BottomCenter)
            )
            
            // חץ כיוון באמצע
            DirectionArrow(
                direction = flowResult.direction,
                magnitude = flowResult.magnitude,
                modifier = Modifier.align(Alignment.Center)
            )
            
            // אינדיקטור אזהרה אם הטלפון לא אופקי
            if (!orientation.isHorizontal) {
                TiltWarning(
                    deviation = orientation.deviationFromHorizontal,
                    modifier = Modifier
                        .align(Alignment.TopCenter)
                        .padding(top = 60.dp)
                )
            }
            
            // מד מהירות
            SpeedIndicator(
                speed = flowResult.magnitude,
                modifier = Modifier
                    .align(Alignment.BottomStart)
                    .padding(16.dp)
            )
            
            // רמת ביטחון
            ConfidenceIndicator(
                confidence = flowResult.confidence,
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .padding(16.dp)
            )
        }
    }
    
    /**
     * אזהרת הטיה
     */
    @Composable
    fun TiltWarning(
        deviation: Float,
        modifier: Modifier = Modifier
    ) {
        val infiniteTransition = rememberInfiniteTransition(label = "blink")
        val alpha by infiniteTransition.animateFloat(
            initialValue = 0.5f,
            targetValue = 1f,
            animationSpec = infiniteRepeatable(
                animation = tween(300),
                repeatMode = RepeatMode.Reverse
            ),
            label = "blink"
        )
        
        Canvas(modifier = modifier.size(200.dp, 60.dp)) {
            // רקע
            drawRoundRect(
                color = ArrowRed.copy(alpha = alpha * 0.3f),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(12f)
            )
            
            // מסגרת
            drawRoundRect(
                color = ArrowRed.copy(alpha = alpha),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(12f),
                style = Stroke(width = 2f)
            )
        }
    }
    
    /**
     * מד מהירות
     */
    @Composable
    fun SpeedIndicator(
        speed: Float,
        modifier: Modifier = Modifier
    ) {
        val normalizedSpeed = (speed / 20f).coerceIn(0f, 1f)
        
        Canvas(modifier = modifier.size(60.dp, 100.dp)) {
            val barHeight = size.height * 0.8f
            val barWidth = size.width * 0.3f
            val startY = size.height * 0.1f
            
            // רקע
            drawRoundRect(
                color = Color.White.copy(alpha = 0.2f),
                topLeft = Offset((size.width - barWidth) / 2, startY),
                size = androidx.compose.ui.geometry.Size(barWidth, barHeight),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(4f)
            )
            
            // מילוי
            val fillHeight = barHeight * normalizedSpeed
            drawRoundRect(
                brush = Brush.verticalGradient(
                    colors = listOf(ArrowGreen, ArrowCyan),
                    startY = startY + barHeight - fillHeight,
                    endY = startY + barHeight
                ),
                topLeft = Offset((size.width - barWidth) / 2, startY + barHeight - fillHeight),
                size = androidx.compose.ui.geometry.Size(barWidth, fillHeight),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(4f)
            )
        }
    }
    
    /**
     * אינדיקטור ביטחון
     */
    @Composable
    fun ConfidenceIndicator(
        confidence: Float,
        modifier: Modifier = Modifier
    ) {
        Canvas(modifier = modifier.size(50.dp)) {
            val radius = size.minDimension / 2 - 4
            val strokeWidth = 6f
            
            // רקע
            drawCircle(
                color = Color.White.copy(alpha = 0.2f),
                radius = radius,
                style = Stroke(width = strokeWidth)
            )
            
            // קשת הביטחון
            val sweepAngle = 360f * confidence
            drawArc(
                color = when {
                    confidence > 0.7f -> ArrowGreen
                    confidence > 0.4f -> ArrowYellow
                    else -> ArrowRed
                },
                startAngle = -90f,
                sweepAngle = sweepAngle,
                useCenter = false,
                style = Stroke(width = strokeWidth, cap = StrokeCap.Round)
            )
        }
    }
}