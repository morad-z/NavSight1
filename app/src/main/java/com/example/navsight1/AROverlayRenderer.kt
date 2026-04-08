package com.example.navsight1

// DEAD FILE: All composables in AROverlayRenderer are unused.
// DirectionArrow was only called from the dead AROverlay() in MainActivity.
// TiltWarning, SpeedIndicator, ConfidenceIndicator were never called.

/*
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

object AROverlayRenderer {

    val ArrowGreen = Color(0xFF00E676)
    val ArrowRed = Color(0xFFFF5252)
    val ArrowYellow = Color(0xFFFFEB3B)
    val ArrowCyan = Color(0xFF00E5FF)

    @Composable
    fun DirectionArrow(
        isMoving: Boolean,
        magnitude: Float,
        modifier: Modifier = Modifier
    ) {
        val infiniteTransition = rememberInfiniteTransition(label = "pulse")
        val pulseScale by infiniteTransition.animateFloat(
            initialValue = 1f,
            targetValue = if (isMoving) 1.2f else 1f,
            animationSpec = infiniteRepeatable(
                animation = tween(500, easing = FastOutSlowInEasing),
                repeatMode = RepeatMode.Reverse
            ),
            label = "pulse"
        )

        val arrowColor = if (isMoving) ArrowGreen else ArrowYellow.copy(alpha = 0.5f)

        Canvas(modifier = modifier.size(120.dp)) {
            val centerX = size.width / 2
            val centerY = size.height / 2
            val arrowSize = size.minDimension / 2 * pulseScale

            if (isMoving) {
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

            // Forward arrow (rotation 0 = up)
            val path = Path().apply {
                moveTo(centerX, centerY - arrowSize * 0.8f)
                lineTo(centerX - arrowSize * 0.4f, centerY)
                lineTo(centerX - arrowSize * 0.15f, centerY)
                lineTo(centerX - arrowSize * 0.15f, centerY + arrowSize * 0.5f)
                lineTo(centerX + arrowSize * 0.15f, centerY + arrowSize * 0.5f)
                lineTo(centerX + arrowSize * 0.15f, centerY)
                lineTo(centerX + arrowSize * 0.4f, centerY)
                close()
            }

            drawPath(
                path = path,
                brush = Brush.verticalGradient(
                    colors = listOf(arrowColor, arrowColor.copy(alpha = 0.6f))
                )
            )
            drawPath(
                path = path,
                color = Color.White.copy(alpha = 0.8f),
                style = Stroke(width = 2f)
            )

            drawCircle(
                color = Color.White,
                radius = 8f,
                center = Offset(centerX, centerY)
            )
        }
    }

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
            drawRoundRect(
                color = ArrowRed.copy(alpha = alpha * 0.3f),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(12f)
            )
            drawRoundRect(
                color = ArrowRed.copy(alpha = alpha),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(12f),
                style = Stroke(width = 2f)
            )
        }
    }

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

            drawRoundRect(
                color = Color.White.copy(alpha = 0.2f),
                topLeft = Offset((size.width - barWidth) / 2, startY),
                size = androidx.compose.ui.geometry.Size(barWidth, barHeight),
                cornerRadius = androidx.compose.ui.geometry.CornerRadius(4f)
            )

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

    @Composable
    fun ConfidenceIndicator(
        confidence: Float,
        modifier: Modifier = Modifier
    ) {
        Canvas(modifier = modifier.size(50.dp)) {
            val radius = size.minDimension / 2 - 4
            val strokeWidth = 6f

            drawCircle(
                color = Color.White.copy(alpha = 0.2f),
                radius = radius,
                style = Stroke(width = strokeWidth)
            )

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
*/
