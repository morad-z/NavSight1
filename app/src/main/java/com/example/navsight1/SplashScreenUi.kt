package com.example.navsight1

import androidx.compose.animation.core.*
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.delay
import kotlin.math.cos
import kotlin.math.sin

@Composable
fun SplashScreen(pal: NavPalette, onDone: () -> Unit) {
    LaunchedEffect(Unit) { delay(3200); onDone() }

    val inf = rememberInfiniteTransition(label = "splash")
    val orbit        by inf.animateFloat(0f, 360f,   infiniteRepeatable(tween(3600, easing = LinearEasing)), label = "orbit")
    val orbitReverse by inf.animateFloat(360f, 0f,   infiniteRepeatable(tween(5000, easing = LinearEasing)), label = "orbitR")
    val glow         by inf.animateFloat(0.18f, 0.55f, infiniteRepeatable(tween(1600, easing = FastOutSlowInEasing), RepeatMode.Reverse), label = "glow")

    var progress by remember { mutableStateOf(0f) }
    val animProgress by animateFloatAsState(progress, tween(2800, easing = FastOutSlowInEasing), label = "progress")
    LaunchedEffect(Unit) { progress = 1f }

    var visible by remember { mutableStateOf(false) }
    LaunchedEffect(Unit) { visible = true }
    val enterAlpha by animateFloatAsState(if (visible) 1f else 0f, tween(700), label = "enter")
    val enterSlide by animateFloatAsState(if (visible) 0f else 40f, tween(700, easing = FastOutSlowInEasing), label = "slide")

    Box(
        Modifier.fillMaxSize().background(
            Brush.radialGradient(listOf(Color(0xFF3D2FA0), Color(0xFF1E1560), Color(0xFF0A0A1A)), radius = 1800f)
        )
    ) {
        Canvas(Modifier.fillMaxSize()) {
            drawCircle(Brush.radialGradient(listOf(Color(0xFF5847B8).copy(0.35f), Color.Transparent),
                radius = size.minDimension * 0.7f, center = Offset(size.width * 0.1f, size.height * 0.12f)),
                radius = size.minDimension * 0.7f, center = Offset(size.width * 0.1f, size.height * 0.12f))
            drawCircle(Brush.radialGradient(listOf(Teal500.copy(0.20f), Color.Transparent),
                radius = size.minDimension * 0.55f, center = Offset(size.width * 0.88f, size.height * 0.78f)),
                radius = size.minDimension * 0.55f, center = Offset(size.width * 0.88f, size.height * 0.78f))
            drawCircle(Brush.radialGradient(listOf(Orange400.copy(0.10f), Color.Transparent),
                radius = size.minDimension * 0.4f, center = Offset(size.width * 0.75f, size.height * 0.15f)),
                radius = size.minDimension * 0.4f, center = Offset(size.width * 0.75f, size.height * 0.15f))
        }

        Column(
            Modifier.fillMaxSize().alpha(enterAlpha).offset(y = enterSlide.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            // Logo ring stack
            Box(Modifier.size(260.dp), contentAlignment = Alignment.Center) {
                Canvas(Modifier.fillMaxSize()) {
                    drawCircle(Brush.radialGradient(listOf(HeroPurple.copy(glow * 0.6f), Color.Transparent)),
                        radius = size.minDimension * 0.52f)
                }
                Canvas(Modifier.size(240.dp)) {
                    val cx = size.width / 2f; val cy = size.height / 2f
                    val r  = size.minDimension / 2f - 4.dp.toPx()
                    for (i in 0 until 24) {
                        val angle = Math.toRadians((orbit + i * 15.0))
                        val len   = if (i % 3 == 0) 14.dp.toPx() else 6.dp.toPx()
                        val alpha = if (i % 3 == 0) 0.65f else 0.22f
                        drawLine(color = Teal400.copy(alpha),
                            start = Offset(cx + (cos(angle) * (r - len)).toFloat(), cy + (sin(angle) * (r - len)).toFloat()),
                            end   = Offset(cx + (cos(angle) * r).toFloat(),          cy + (sin(angle) * r).toFloat()),
                            strokeWidth = 2.5.dp.toPx(), cap = StrokeCap.Round)
                    }
                }
                Canvas(Modifier.size(200.dp)) {
                    val inset = Offset(6.dp.toPx(), 6.dp.toPx())
                    val sz    = Size(size.width - 12.dp.toPx(), size.height - 12.dp.toPx())
                    drawArc(Brush.sweepGradient(listOf(Color.Transparent, Teal400.copy(0.9f), Color.Transparent)),
                        orbitReverse, 120f, false, inset, sz, style = Stroke(3.dp.toPx(), cap = StrokeCap.Round))
                    drawArc(Brush.sweepGradient(listOf(Color.Transparent, Orange400.copy(0.7f), Color.Transparent)),
                        orbitReverse + 180f, 60f, false, inset, sz, style = Stroke(2.dp.toPx(), cap = StrokeCap.Round))
                }
                Box(
                    Modifier.size(158.dp).clip(CircleShape)
                        .background(Brush.radialGradient(listOf(Color(0xFF7060E0), Color(0xFF3D2FA0), Color(0xFF1A1060))))
                        .border(BorderStroke(1.5.dp, Brush.linearGradient(
                            listOf(Color.White.copy(0.45f), Teal400.copy(0.3f), Color.White.copy(0.1f)))), CircleShape),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(painterResource(id = R.drawable.logo), "NavSight Logo",
                        tint = Color.Unspecified, modifier = Modifier.size(118.dp))
                }
            }

            Spacer(Modifier.height(36.dp))
            Text("NavSight", color = Color.White, fontSize = 42.sp, fontWeight = FontWeight.ExtraBold, letterSpacing = 1.sp)
            Spacer(Modifier.height(6.dp))
            Text("Visual · IMU · GPS Navigation", color = Color.White.copy(0.52f), fontSize = 13.sp, letterSpacing = 0.5.sp)
            Spacer(Modifier.height(48.dp))

            // Feature pills
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp), verticalAlignment = Alignment.CenterVertically) {
                listOf(Icons.Default.Videocam to "Camera", Icons.Default.Explore to "IMU", Icons.Default.Map to "GPS")
                    .forEach { (icon, label) ->
                        Surface(color = Color.White.copy(0.08f), shape = RoundedCornerShape(20.dp),
                            border = BorderStroke(1.dp, Color.White.copy(0.13f))) {
                            Row(Modifier.padding(horizontal = 14.dp, vertical = 8.dp),
                                horizontalArrangement = Arrangement.spacedBy(6.dp),
                                verticalAlignment = Alignment.CenterVertically) {
                                Icon(icon, null, tint = Teal400, modifier = Modifier.size(14.dp))
                                Text(label, color = Color.White.copy(0.85f), fontSize = 12.sp)
                            }
                        }
                    }
            }

            Spacer(Modifier.height(44.dp))

            // Progress bar
            Column(horizontalAlignment = Alignment.CenterHorizontally, modifier = Modifier.padding(horizontal = 52.dp)) {
                Box(Modifier.fillMaxWidth().height(3.dp).clip(RoundedCornerShape(2.dp)).background(Color.White.copy(0.10f))) {
                    Box(Modifier.fillMaxHeight().fillMaxWidth(animProgress)
                        .background(Brush.horizontalGradient(listOf(Teal400, HeroPurple.copy(0.9f))), RoundedCornerShape(2.dp)))
                }
                Spacer(Modifier.height(14.dp))
                Text("Initializing sensors…", color = Color.White.copy(0.40f), fontSize = 11.sp, letterSpacing = 0.3.sp)
            }
        }
    }
}
