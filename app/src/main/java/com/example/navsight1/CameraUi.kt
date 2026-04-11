package com.example.navsight1

import android.util.Log
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.animation.*
import androidx.compose.animation.core.tween
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat

private const val TAG = "NavSight"

@Composable
fun CameraViewComposable(viewModel: NavSightViewModel) {
    val lifecycleOwner = LocalLifecycleOwner.current
    AndroidView(
        factory = { ctx ->
            val pv = PreviewView(ctx).apply {
                implementationMode = PreviewView.ImplementationMode.PERFORMANCE
                scaleType          = PreviewView.ScaleType.FILL_CENTER
            }
            val future = ProcessCameraProvider.getInstance(ctx)
            future.addListener({
                val provider = future.get()
                val preview  = Preview.Builder().build().also { it.setSurfaceProvider(pv.surfaceProvider) }
                @Suppress("DEPRECATION")
                val analysis = ImageAnalysis.Builder()
                    .setTargetResolution(android.util.Size(640, 480))
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                    .build().also { ia ->
                        ia.setAnalyzer(java.util.concurrent.Executors.newSingleThreadExecutor()) { img ->
                            try {
                                if (viewModel.isSensorRepositoryActive()) viewModel.processCameraFrame(img)
                                else img.close()
                            } catch (e: java.util.concurrent.RejectedExecutionException) {
                                try { img.close() } catch (_: Exception) {}
                            } catch (e: IllegalStateException) {
                                try { img.close() } catch (_: Exception) {}
                            } catch (e: Exception) {
                                Log.w(TAG, "Camera frame dropped: ${e.message}")
                                try { img.close() } catch (_: Exception) {}
                            }
                        }
                    }
                try {
                    provider.unbindAll()
                    provider.bindToLifecycle(lifecycleOwner, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis)
                } catch (e: Exception) { Log.e(TAG, "CameraX bind failed", e) }
            }, ContextCompat.getMainExecutor(ctx))
            pv
        },
        modifier = Modifier.fillMaxSize()
    )
}

@Composable
fun CameraOverlay(
    pal: NavPalette,
    isMoving: Boolean,
    showCameraBlocked: Boolean,
    isVioInitialized: Boolean,
    vioTrackingQuality: Float,
    vioTrackedFeatures: Int,
    orientationIsHorizontal: Boolean,
    orientationDeviation: Float,
    stabilityScore: Float,
    fusedHeading: Float,
    historySnapshot: List<Pair<Float, Float>>,
    onClose: () -> Unit,
    onDebug: () -> Unit,
    debugVisible: Boolean,
    totalDistanceM: Double,
    speedMs: Float,
    qualityPct: Float,
    fusionMode: String,
    scaleFactor: Double,
    viewModel: NavSightViewModel
) {
    Box(Modifier.fillMaxSize()) {
        Box(Modifier.fillMaxSize().background(Brush.verticalGradient(
            0f to Color.Black.copy(0.52f), 0.22f to Color.Transparent,
            0.78f to Color.Transparent,    1f    to Color.Black.copy(0.60f)
        )))

        Box(Modifier.align(Alignment.TopStart).statusBarsPadding().padding(14.dp)) {
            DirectionBadge(isMoving, vioTrackingQuality, pal)
        }

        Column(
            Modifier.align(Alignment.TopEnd).statusBarsPadding().padding(14.dp),
            verticalArrangement   = Arrangement.spacedBy(10.dp),
            horizontalAlignment   = Alignment.End
        ) {
            Surface(onClick = onClose, color = Color.White.copy(0.90f), shape = CircleShape, shadowElevation = 4.dp) {
                Icon(Icons.Default.Close, "Close", tint = LightText, modifier = Modifier.padding(10.dp).size(20.dp))
            }
            SensorRadarWaze(historySnapshot, fusedHeading, pal)
        }

        if (!isVioInitialized)
            Box(Modifier.align(Alignment.Center)) { VioInitializingBadge(pal) }
        if (showCameraBlocked)
            Box(Modifier.align(Alignment.Center)) { CameraBlockedWarning(pal) }
        if (isVioInitialized && !showCameraBlocked && vioTrackedFeatures < 30)
            Box(Modifier.align(Alignment.BottomStart).padding(start = 14.dp, bottom = 115.dp)) { NoTextureWarning(pal) }
        if (!orientationIsHorizontal)
            Box(Modifier.align(Alignment.BottomCenter).padding(bottom = 115.dp)) { PhoneOrientationWarning(orientationDeviation, pal) }
        Box(Modifier.align(Alignment.BottomEnd).padding(end = 14.dp, bottom = 115.dp)) {
            StabilityIndicator(stabilityScore, vioTrackingQuality, pal)
        }

        Surface(
            modifier        = Modifier.align(Alignment.BottomCenter).fillMaxWidth().navigationBarsPadding(),
            color           = Color.White,
            shape           = RoundedCornerShape(topStart = 22.dp, topEnd = 22.dp),
            shadowElevation = 14.dp
        ) {
            Row(
                Modifier.fillMaxWidth().padding(horizontal = 20.dp, vertical = 14.dp),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment     = Alignment.CenterVertically
            ) {
                CamHudStat("${"%.1f".format(speedMs)}", "m/s", pal.teal)
                CamHudDiv()
                CamHudStat(fusionMode, "mode", when (fusionMode) {
                    "CAMERA" -> pal.teal; "HYBRID" -> pal.orange; else -> Color(0xFF90A4AE) })
                CamHudDiv()
                CamHudStat("${"%.0f".format(qualityPct)}%", "quality", when {
                    qualityPct > 70f -> pal.teal; qualityPct > 30f -> pal.orange; else -> Color(0xFFEF5350) })
                CamHudDiv()
                CamHudStat("${"%.0f".format(totalDistanceM)}", "m", Orange400)
                CamHudDiv()
                Surface(
                    onClick = onDebug,
                    color  = if (debugVisible) pal.teal.copy(0.10f) else Color(0xFFF5F5F5),
                    shape  = RoundedCornerShape(12.dp),
                    border = BorderStroke(1.dp, if (debugVisible) pal.teal.copy(0.4f) else Color(0xFFE0E0E0))
                ) {
                    Icon(Icons.Default.Settings, "Debug",
                        tint     = if (debugVisible) pal.teal else Color(0xFF90A4AE),
                        modifier = Modifier.padding(9.dp).size(20.dp))
                }
            }
        }

        AnimatedVisibility(
            debugVisible,
            enter = fadeIn() + expandVertically(expandFrom = Alignment.Bottom),
            exit  = fadeOut() + shrinkVertically(shrinkTowards = Alignment.Bottom),
            modifier = Modifier.align(Alignment.BottomStart).padding(start = 14.dp, bottom = 115.dp)
        ) {
            DebugPanel(totalDistanceM, speedMs, qualityPct, fusionMode, scaleFactor, fusedHeading, pal, viewModel)
        }
    }
}

@Composable
fun PipCameraCard(modifier: Modifier, pal: NavPalette, vioInitialized: Boolean, onClick: () -> Unit) {
    Surface(
        modifier        = modifier.clickable { onClick() },
        color           = Color.Black.copy(alpha = 0.82f),
        shape           = RoundedCornerShape(18.dp),
        border          = BorderStroke(1.5.dp, Color.White.copy(alpha = 0.24f)),
        shadowElevation = 10.dp
    ) {
        Box(Modifier.size(width = 92.dp, height = 68.dp)) {
            Column(Modifier.fillMaxSize().padding(10.dp),
                horizontalAlignment = Alignment.Start, verticalArrangement = Arrangement.SpaceBetween) {
                Box(Modifier.size(28.dp).clip(CircleShape).background(Color.White.copy(0.14f)),
                    contentAlignment = Alignment.Center) {
                    Icon(Icons.Default.PhotoCamera, null, tint = Color.White, modifier = Modifier.size(15.dp))
                }
                Column {
                    Text("Camera", color = Color.White, fontSize = 10.sp, fontWeight = FontWeight.Bold)
                    Text(if (vioInitialized) "Live preview" else "Standby", color = Color.White.copy(0.68f), fontSize = 8.sp)
                }
            }
            Box(Modifier.align(Alignment.TopEnd).padding(7.dp).size(8.dp).clip(CircleShape)
                .background(if (vioInitialized) Color(0xFF53D34D) else pal.orange))
            Icon(Icons.Default.KeyboardArrowRight, null, tint = Color.White.copy(0.62f),
                modifier = Modifier.align(Alignment.BottomEnd).padding(6.dp).size(10.dp))
        }
    }
}

@Composable
private fun CamHudStat(value: String, label: String, color: Color) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(value, color = color, fontSize = 15.sp, fontWeight = FontWeight.ExtraBold, lineHeight = 17.sp)
        Text(label, color = Color(0xFF90A4AE), fontSize = 8.sp, letterSpacing = 0.5.sp)
    }
}

@Composable
private fun CamHudDiv() {
    Box(Modifier.width(1.dp).height(26.dp).background(Color(0xFFEEEEEE)))
}
