package com.example.navsight1

import android.Manifest
import android.os.Bundle
import android.util.Log
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.*
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.ArrowForward
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.KeyboardArrowDown
import androidx.compose.material.icons.filled.KeyboardArrowUp
import androidx.compose.material.icons.filled.Phone
import androidx.compose.material.icons.filled.Place
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.rememberMultiplePermissionsState
import com.google.android.gms.maps.model.*
import com.google.maps.android.compose.*
import com.otaliastudios.cameraview.CameraView
import com.otaliastudios.cameraview.controls.Audio
import com.otaliastudios.cameraview.frame.Frame
import com.otaliastudios.cameraview.frame.FrameProcessor
import androidx.annotation.MainThread
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.*

/* ===================== COLORS ===================== */
private val LuxuryBlack = Color(0xFF0A0A0F)
private val LuxuryDarkGrey = Color(0xFF1C1C1E)
private val LuxuryGreen = Color(0xFF00E676)
private val LuxuryRed = Color(0xFFFF5252)
private val LuxuryCyan = Color(0xFF00E5FF)
private val LuxuryYellow = Color(0xFFFFEB3B)
private val LuxuryTextGrey = Color(0xFF8E8E93)

class MainActivity : ComponentActivity() {

    private val TAG = "NavSight"
    private val viewModel: NavSightViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        setContent {
            MaterialTheme(colorScheme = darkColorScheme()) {
                NavSightApp()
            }
        }
    }

    override fun onResume() {
        super.onResume()
        viewModel.onResume()
    }

    override fun onPause() {
        super.onPause()
        viewModel.onPause()
    }

    @OptIn(ExperimentalPermissionsApi::class)
    @Composable
    fun NavSightApp() {
        var showSplash by remember { mutableStateOf(true) }

        if (showSplash) {
            SplashScreen(onSplashFinished = { showSplash = false })
        } else {
            val perms = rememberMultiplePermissionsState(
                listOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION)
            )

            LaunchedEffect(perms.allPermissionsGranted) {
                if (perms.allPermissionsGranted) {
                    viewModel.requestInitialLocation(true)
                }
            }

            when {
                !perms.allPermissionsGranted -> PermissionScreen { perms.launchMultiplePermissionRequest() }
                else -> MainScreen()
            }
        }
    }

    @Composable
    fun PermissionScreen(onRequest: () -> Unit) {
        Box(Modifier.fillMaxSize().background(LuxuryBlack), contentAlignment = Alignment.Center) {
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Icon(
                    Icons.Default.Place,
                    contentDescription = null,
                    tint = LuxuryGreen,
                    modifier = Modifier.size(64.dp)
                )
                Spacer(Modifier.height(16.dp))
                Text(
                    "Camera & Location Required",
                    color = Color.White,
                    fontSize = 18.sp
                )
                Spacer(Modifier.height(24.dp))
                Button(
                    onClick = onRequest,
                    colors = ButtonDefaults.buttonColors(containerColor = LuxuryGreen)
                ) {
                    Text("Enable Sensors", color = LuxuryBlack)
                }
            }
        }
    }

    @Composable
    fun MainScreen() {
        val orientation = viewModel.orientationState
        val vio = viewModel.vioState
        val isMoving = vio.isInitialized && vio.meanFlow > 1.0
        val azimuth = orientation.azimuth

        // Heading: vio.heading is absolute compass heading (initial azimuth baked into global_R_)
        val fusedHeading: Float = if (vio.isInitialized) {
            ((Math.toDegrees(vio.heading).toFloat() % 360f) + 360f) % 360f
        } else {
            azimuth
        }

        var debugPanelVisible by remember { mutableStateOf(false) }

        Box(Modifier.fillMaxSize().background(LuxuryBlack)) {
            Column(Modifier.fillMaxSize()) {
                Box(
                    modifier = Modifier
                        .weight(0.65f)
                        .fillMaxWidth()
                        .padding(12.dp)
                        .clip(RoundedCornerShape(20.dp))
                        .border(
                            width = 2.dp,
                            color = if (orientation.isHorizontal) LuxuryGreen else LuxuryRed,
                            shape = RoundedCornerShape(20.dp)
                        )
                ) {
                    CameraViewComposable()
                    AROverlay(isMoving = isMoving, orientation = orientation)

                    Box(Modifier.align(Alignment.TopEnd).padding(12.dp)) {
                        SensorRadar(
                            history = viewModel.pathHistory.toList(),
                            currentAzimuth = fusedHeading
                        )
                    }

                    Box(Modifier.align(Alignment.TopStart).padding(12.dp)) {
                        DirectionBadge(
                            isMoving = isMoving,
                            trackingQuality = vio.trackingQuality.toFloat()
                        )
                    }

                    if (!vio.isInitialized) {
                        Box(Modifier.align(Alignment.Center).padding(8.dp)) {
                            VioInitializingBadge()
                        }
                    }

                    if (viewModel.showCameraBlocked) {
                        Box(Modifier.align(Alignment.Center).padding(8.dp)) {
                            CameraBlockedWarning()
                        }
                    }

                    if (vio.isInitialized && !viewModel.showCameraBlocked && vio.trackedFeatures < 30) {
                        Box(Modifier.align(Alignment.BottomStart).padding(12.dp)) {
                            NoTextureWarning()
                        }
                    }

                    if (!orientation.isHorizontal) {
                        Box(Modifier.align(Alignment.BottomCenter).padding(12.dp)) {
                            PhoneOrientationWarning(deviation = orientation.deviationFromHorizontal)
                        }
                    }

                    Box(Modifier.align(Alignment.BottomEnd).padding(12.dp)) {
                        StabilityIndicator(
                            stability = orientation.stabilityScore,
                            confidence = vio.trackingQuality.toFloat()
                        )
                    }

                    // Debug panel + toggle
                    Box(Modifier.align(Alignment.BottomStart).padding(start = 10.dp, bottom = 40.dp)) {
                        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                            DebugPanel(
                                isVisible = debugPanelVisible,
                                totalDistanceM = viewModel.totalDistanceM,
                                speedMs = viewModel.currentSpeedKmh / 3.6f,
                                qualityPct = (vio.trackingQuality * 100).toFloat(),
                                fusionMode = when {
                                    !vio.isInitialized -> "INIT"
                                    vio.trackingQuality < 0.3 -> "IMU"
                                    vio.trackingQuality > 0.7 -> "CAMERA"
                                    else -> "HYBRID"
                                },
                                scaleFactor = vio.estimatedScale,
                                headingDeg = fusedHeading
                            )
                            SmallFloatingActionButton(
                                onClick = { debugPanelVisible = !debugPanelVisible },
                                containerColor = if (debugPanelVisible) LuxuryCyan else LuxuryDarkGrey,
                                modifier = Modifier.size(32.dp)
                            ) {
                                Text(
                                    if (debugPanelVisible) "x" else "D",
                                    color = if (debugPanelVisible) LuxuryBlack else LuxuryTextGrey,
                                    fontSize = 12.sp,
                                    fontWeight = FontWeight.Bold
                                )
                            }
                        }
                    }
                }

                Box(
                    modifier = Modifier
                        .weight(0.35f)
                        .fillMaxWidth()
                        .padding(start = 7.dp, end = 7.dp, top = 4.dp, bottom = 50.dp)
                        .clip(RoundedCornerShape(20.dp))
                        .background(LuxuryDarkGrey)
                ) {
                    val mapStartLocation = viewModel.startLocation

                    if (mapStartLocation == null) {
                        // FR32: Locating State - prevents Tel Aviv fallback
                        Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                                CircularProgressIndicator(color = LuxuryGreen)
                                Spacer(Modifier.height(8.dp))
                                Text("משיג מיקום התחלתי...", color = Color.White, fontSize = 12.sp)
                            }
                        }
                    } else {
                        NavigationMapWrapper(
                            start = mapStartLocation,
                            azimuth = fusedHeading,
                            history = viewModel.pathHistory.toList()
                        )
                    }

                    val navState = viewModel.navigationState
                    val instruction = viewModel.currentInstruction
                    val navigationStartMessage = viewModel.navigationStartMessage
                    if (navState is NavigationState.Active && instruction != null) {
                        Box(Modifier.align(Alignment.TopCenter).padding(16.dp).fillMaxWidth(0.95f)) {
                            NavigationInstructionBanner(
                                instruction = instruction,
                                remainingDistanceMeters = navState.remainingDistanceMeters,
                                remainingTimeSeconds = navState.remainingTimeSeconds
                            )
                        }
                    }

                    if (navState is NavigationState.Idle) {
                        Box(Modifier.align(Alignment.TopCenter).padding(horizontal = 12.dp, vertical = 8.dp).fillMaxWidth(0.95f)) {
                            Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
                                DestinationSearchBar(
                                    onDestinationSelected = { destination ->
                                        viewModel.startNavigation(destination)
                                    }
                                )
                                if (navigationStartMessage != null) {
                                    Surface(
                                        onClick = { viewModel.clearNavigationStartMessage() },
                                        color = LuxuryRed.copy(alpha = 0.92f),
                                        shape = RoundedCornerShape(12.dp)
                                    ) {
                                        Text(
                                            text = navigationStartMessage,
                                            color = Color.White,
                                            fontSize = 13.sp,
                                            modifier = Modifier.padding(horizontal = 14.dp, vertical = 10.dp)
                                        )
                                    }
                                }
                            }
                        }
                    }

                    if (navState is NavigationState.Active) {
                        FloatingActionButton(
                            onClick = { viewModel.stopNavigation() },
                            containerColor = LuxuryRed,
                            modifier = Modifier.align(Alignment.TopEnd).padding(16.dp).size(48.dp)
                        ) {
                            Icon(Icons.Default.Close, "Cancel Navigation", tint = Color.White)
                        }
                    }

                    Column(
                        Modifier.align(Alignment.BottomEnd).padding(16.dp),
                        verticalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        SmallFloatingActionButton(
                            onClick = { viewModel.exportPath(::getExternalFilesDir, filesDir) },
                            containerColor = LuxuryCyan
                        ) {
                            Icon(Icons.Default.ArrowForward, "Export", tint = LuxuryBlack)
                        }
                        FloatingActionButton(
                            onClick = { viewModel.resetPath() },
                            containerColor = LuxuryGreen
                        ) {
                            Icon(Icons.Default.Refresh, "Reset", tint = LuxuryBlack)
                        }
                    }

                    Box(
                        Modifier.align(Alignment.TopEnd).padding(8.dp)
                            .background(Color.Black.copy(alpha = 0.7f), RoundedCornerShape(8.dp))
                            .padding(horizontal = 10.dp, vertical = 6.dp)
                    ) {
                        Column(horizontalAlignment = Alignment.End) {
                            Text("${"%.1f".format(viewModel.currentSpeedKmh)} km/h", color = LuxuryCyan, fontWeight = FontWeight.Bold, fontSize = 16.sp)
                            Text("${"%.0f".format(viewModel.totalDistanceM)} m", color = LuxuryGreen, fontSize = 12.sp)
                        }
                    }
                }
            }
        }
    }

    @Composable
    fun CameraViewComposable() {
        val lifecycle = LocalLifecycleOwner.current
        AndroidView(
            factory = { ctx ->
                CameraView(ctx).apply {
                    setLifecycleOwner(lifecycle)
                    setAudio(Audio.OFF)
                    setFrameProcessingFormat(android.graphics.ImageFormat.YUV_420_888)
                    addFrameProcessor(object : FrameProcessor {
                        override fun process(frame: Frame) {
                            viewModel.processCameraFrame(frame)
                        }
                    })
                }
            },
            modifier = Modifier.fillMaxSize()
        )
    }

    @Composable
    fun AROverlay(
        isMoving: Boolean,
        orientation: DeviceOrientationTracker.OrientationResult
    ) {
        if (isMoving && orientation.isHorizontal) {
            val infiniteTransition = rememberInfiniteTransition(label = "grid")
            val gridOffset by infiniteTransition.animateFloat(
                initialValue = 0f, targetValue = 60f,
                animationSpec = infiniteRepeatable(tween(800, easing = LinearEasing), RepeatMode.Restart),
                label = "gridOffset"
            )
            Canvas(Modifier.fillMaxSize()) {
                val startY = size.height * 0.4f
                val centerX = size.width / 2
                for (i in 0..10) {
                    val y = startY + (gridOffset + i * 40f) % (size.height * 0.6f)
                    val alpha = ((y - startY) / (size.height * 0.6f) * 0.5f).coerceIn(0f, 0.5f)
                    drawLine(color = LuxuryGreen.copy(alpha = alpha), start = Offset(centerX, y), end = Offset(centerX, y + 30f), strokeWidth = 3f, cap = StrokeCap.Round)
                }
            }
        }
        Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
            AROverlayRenderer.DirectionArrow(isMoving = isMoving, magnitude = if (isMoving) 5f else 0f)
        }
    }

    @Composable
    fun DirectionBadge(isMoving: Boolean, trackingQuality: Float) {
        val lowQuality = trackingQuality < 0.3f && trackingQuality > 0f
        val (text, color, icon) = when {
            lowQuality -> Triple("LOW QUALITY", LuxuryRed, Icons.Default.Place)
            isMoving -> Triple("בתנועה", LuxuryGreen, Icons.Default.KeyboardArrowUp)
            else -> Triple("עומד", LuxuryYellow, Icons.Default.Place)
        }
        Surface(color = Color.Black.copy(0.8f), shape = RoundedCornerShape(12.dp), border = BorderStroke(1.dp, color)) {
            Row(modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp), verticalAlignment = Alignment.CenterVertically) {
                Icon(icon, null, tint = color, modifier = Modifier.size(20.dp))
                Spacer(Modifier.width(6.dp))
                Text(text, color = color, fontWeight = FontWeight.Bold, fontSize = 14.sp)
            }
        }
    }

    @Composable
    fun PhoneOrientationWarning(deviation: Float) {
        val infiniteTransition = rememberInfiniteTransition(label = "blink")
        val alpha by infiniteTransition.animateFloat(initialValue = 0.6f, targetValue = 1f, animationSpec = infiniteRepeatable(tween(400), RepeatMode.Reverse), label = "blink")
        Surface(color = LuxuryRed.copy(alpha = alpha * 0.3f), shape = RoundedCornerShape(12.dp), border = BorderStroke(2.dp, LuxuryRed.copy(alpha = alpha))) {
            Row(modifier = Modifier.padding(horizontal = 16.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.Phone, null, tint = LuxuryRed, modifier = Modifier.size(24.dp).rotate(deviation.coerceIn(-30f, 30f)))
                Spacer(Modifier.width(8.dp))
                Column {
                    Text("הטה את הטלפון קדימה — מצלמה לסצנה", color = LuxuryRed, fontWeight = FontWeight.Bold, fontSize = 14.sp)
                    Text("סטייה: ${"%.0f".format(deviation)}°", color = LuxuryRed.copy(alpha = 0.8f), fontSize = 12.sp)
                }
            }
        }
    }

    @Composable
    fun VioInitializingBadge() {
        val infiniteTransition = rememberInfiniteTransition(label = "init_pulse")
        val alpha by infiniteTransition.animateFloat(initialValue = 0.5f, targetValue = 1f, animationSpec = infiniteRepeatable(tween(600), RepeatMode.Reverse), label = "init_alpha")
        Surface(color = Color.Black.copy(0.85f), shape = RoundedCornerShape(12.dp), border = BorderStroke(1.dp, LuxuryYellow.copy(alpha = alpha))) {
            Row(Modifier.padding(horizontal = 14.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.Refresh, null, tint = LuxuryYellow, modifier = Modifier.size(18.dp))
                Spacer(Modifier.width(8.dp))
                Text("מכייל…", color = LuxuryYellow, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            }
        }
    }

    @Composable
    fun CameraBlockedWarning() {
        val infiniteTransition = rememberInfiniteTransition(label = "blocked_blink")
        val alpha by infiniteTransition.animateFloat(initialValue = 0.6f, targetValue = 1f, animationSpec = infiniteRepeatable(tween(400), RepeatMode.Reverse), label = "blocked_alpha")
        Surface(color = LuxuryRed.copy(0.2f), shape = RoundedCornerShape(12.dp), border = BorderStroke(2.dp, LuxuryRed.copy(alpha = alpha))) {
            Row(Modifier.padding(horizontal = 14.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.Place, null, tint = LuxuryRed, modifier = Modifier.size(20.dp))
                Spacer(Modifier.width(8.dp))
                Text("מצלמה חסומה", color = LuxuryRed, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            }
        }
    }

    @Composable
    fun NoTextureWarning() {
        Surface(color = Color.Black.copy(0.75f), shape = RoundedCornerShape(10.dp), border = BorderStroke(1.dp, LuxuryYellow)) {
            Text("אין מרקם — עבור לאזור מרוצף", color = LuxuryYellow, fontSize = 11.sp, modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp))
        }
    }

    @Composable
    fun StabilityIndicator(stability: Float, confidence: Float) {
        Surface(color = Color.Black.copy(0.7f), shape = RoundedCornerShape(8.dp)) {
            Column(modifier = Modifier.padding(8.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                Box(Modifier.width(40.dp).height(4.dp).clip(RoundedCornerShape(2.dp)).background(Color.White.copy(0.2f))) {
                    Box(Modifier.fillMaxHeight().fillMaxWidth(stability).background(if (stability > 0.7f) LuxuryGreen else if (stability > 0.4f) LuxuryYellow else LuxuryRed))
                }
                Spacer(Modifier.height(4.dp))
                Text("${"%.0f".format(confidence * 100)}%", color = LuxuryTextGrey, fontSize = 10.sp)
            }
        }
    }

    @Composable
    fun SensorRadar(history: List<Pair<Float, Float>>, currentAzimuth: Float) {
        // Straight-line distance from origin (0,0) — VIO always starts at origin
        val distFromStart = if (history.isEmpty()) 0f
        else {
            val lx = history.last().first.toDouble()
            val lz = history.last().second.toDouble()
            sqrt(lx * lx + lz * lz).toFloat()
        }

        Card(
            shape = RoundedCornerShape(12.dp),
            colors = CardDefaults.cardColors(containerColor = Color.Black.copy(0.88f)),
            border = BorderStroke(1.dp, LuxuryGreen),
            modifier = Modifier.size(150.dp)
        ) {
            Box(Modifier.fillMaxSize()) {
                Canvas(Modifier.fillMaxSize().padding(8.dp)) {
                    val cx = size.width / 2f
                    val cy = size.height / 2f
                    val maxRadius = minOf(cx, cy)
                    val metersToPixels = maxRadius / 5f

                    // Distance rings (1m, 2m, 5m)
                    listOf(1f to "1m", 2f to "2m", 5f to "5m").forEach { (distM, label) ->
                        val ringR = distM * metersToPixels
                        drawCircle(
                            color = LuxuryGreen.copy(alpha = 0.15f),
                            radius = ringR,
                            center = Offset(cx, cy),
                            style = Stroke(width = 1f)
                        )
                        drawContext.canvas.nativeCanvas.drawText(
                            label,
                            cx + ringR * 0.72f,
                            cy - ringR * 0.72f + 10f,
                            android.graphics.Paint().apply {
                                color = android.graphics.Color.argb(150, 0, 230, 118)
                                textSize = 16f
                                textAlign = android.graphics.Paint.Align.CENTER
                            }
                        )
                    }

                    // Cardinal labels (N/S/E/W)
                    val cardPaint = android.graphics.Paint().apply {
                        color = android.graphics.Color.argb(130, 142, 142, 147)
                        textSize = 20f
                        textAlign = android.graphics.Paint.Align.CENTER
                        isFakeBoldText = true
                    }
                    val cr = maxRadius + 1f
                    drawContext.canvas.nativeCanvas.apply {
                        drawText("N", cx, cy - cr + 14f, cardPaint)
                        drawText("S", cx, cy + cr, cardPaint)
                        drawText("E", cx + cr, cy + 5f, cardPaint)
                        drawText("W", cx - cr, cy + 5f, cardPaint)
                    }

                    // Crosshair
                    drawLine(LuxuryGreen.copy(0.12f), Offset(cx, cy - maxRadius), Offset(cx, cy + maxRadius), 1f)
                    drawLine(LuxuryGreen.copy(0.12f), Offset(cx - maxRadius, cy), Offset(cx + maxRadius, cy), 1f)

                    // Path history (color-coded by movement speed)
                    if (history.size >= 2) {
                        val curX = history.last().first
                        val curZ = history.last().second
                        val rad = Math.toRadians((-currentAzimuth).toDouble())
                        val cosA = cos(rad).toFloat()
                        val sinA = sin(rad).toFloat()

                        fun toCanvas(pX: Float, pZ: Float): Offset {
                            val dx = pX - curX
                            val dz = pZ - curZ
                            val rx = dx * cosA - dz * sinA
                            val rz = dx * sinA + dz * cosA
                            return Offset(
                                (cx + rx * metersToPixels).coerceIn(0f, size.width),
                                (cy - rz * metersToPixels).coerceIn(0f, size.height)
                            )
                        }

                        val n = history.size
                        for (i in 1 until n) {
                            val segDx = history[i].first - history[i - 1].first
                            val segDz = history[i].second - history[i - 1].second
                            val mov = sqrt((segDx * segDx + segDz * segDz).toDouble()).toFloat()
                            val segColor = when {
                                mov > 0.03f -> LuxuryGreen
                                mov > 0.008f -> LuxuryYellow
                                else -> LuxuryRed
                            }
                            val ageFade = (0.25f + 0.75f * (i.toFloat() / n)).coerceIn(0.2f, 1f)
                            drawLine(
                                color = segColor.copy(alpha = ageFade),
                                start = toCanvas(history[i - 1].first, history[i - 1].second),
                                end = toCanvas(history[i].first, history[i].second),
                                strokeWidth = 2.5f,
                                cap = StrokeCap.Round
                            )
                        }
                        // Start marker
                        drawCircle(LuxuryYellow, 4f, toCanvas(history.first().first, history.first().second))
                    }

                    // Current position dot
                    drawCircle(Color.White, 5f, Offset(cx, cy))

                    // Heading arrow
                    val arrowLen = maxRadius * 0.38f
                    val aRad = Math.toRadians(currentAzimuth.toDouble())
                    val tipX = cx + (sin(aRad) * arrowLen).toFloat()
                    val tipY = cy - (cos(aRad) * arrowLen).toFloat()
                    drawLine(LuxuryCyan, Offset(cx, cy), Offset(tipX, tipY), strokeWidth = 3f, cap = StrokeCap.Round)
                    listOf(150.0, -150.0).forEach { offset ->
                        val hr = Math.toRadians(currentAzimuth + offset)
                        drawLine(
                            LuxuryCyan,
                            Offset(tipX, tipY),
                            Offset(tipX + (sin(hr) * 8f).toFloat(), tipY - (cos(hr) * 8f).toFloat()),
                            strokeWidth = 2.5f, cap = StrokeCap.Round
                        )
                    }
                }

                // Distance from start label
                Text(
                    text = "${"%.1f".format(distFromStart)}m",
                    color = LuxuryCyan,
                    fontSize = 9.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 3.dp)
                )
                Text(
                    "RADAR",
                    color = LuxuryGreen.copy(0.45f),
                    fontSize = 7.sp,
                    letterSpacing = 1.sp,
                    modifier = Modifier.align(Alignment.TopCenter).padding(top = 3.dp)
                )
            }
        }
    }

    @Composable
    private fun DebugRow(label: String, value: String, valueColor: Color) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(label, color = LuxuryTextGrey, fontSize = 10.sp)
            Text(value, color = valueColor, fontSize = 10.sp, fontWeight = FontWeight.Bold)
        }
    }

    @Composable
    fun DebugPanel(
        isVisible: Boolean,
        totalDistanceM: Double,
        speedMs: Float,
        qualityPct: Float,
        fusionMode: String,
        scaleFactor: Double,
        headingDeg: Float
    ) {
        if (!isVisible) return

        var calibrationExpanded by remember { mutableStateOf(false) }
        val session = viewModel.scaleCalibrationSession
        val calibMessage = viewModel.scaleCalibrationMessage
        val calibFactor = viewModel.scaleCalibrationFactor
        val currentVio = viewModel.vioState

        Surface(
            color = Color.Black.copy(alpha = 0.85f),
            shape = RoundedCornerShape(10.dp),
            border = BorderStroke(1.dp, LuxuryCyan.copy(alpha = 0.35f)),
            modifier = Modifier.width(185.dp)
        ) {
            Column(
                modifier = Modifier.padding(horizontal = 10.dp, vertical = 7.dp),
                verticalArrangement = Arrangement.spacedBy(3.dp)
            ) {
                Text(
                    "DEBUG",
                    color = LuxuryCyan,
                    fontSize = 9.sp,
                    fontWeight = FontWeight.Bold,
                    letterSpacing = 1.5.sp
                )
                Divider(color = LuxuryCyan.copy(0.25f), thickness = 0.5.dp)

                // ── FOR SIMULATION ────────────────────────────────────────────────
                Button(
                    onClick = { viewModel.toggleSimulationRecording(::getExternalFilesDir, filesDir) },
                    modifier = Modifier.fillMaxWidth().height(24.dp),
                    contentPadding = PaddingValues(0.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (viewModel.isRecordingSimulation) LuxuryRed else LuxuryGreen.copy(alpha = 0.2f)
                    ),
                    shape = RoundedCornerShape(4.dp)
                ) {
                    Text(
                        if (viewModel.isRecordingSimulation) "STOP RECORDING" else "START RECORDING",
                        fontSize = 8.sp,
                        fontWeight = FontWeight.Bold,
                        color = if (viewModel.isRecordingSimulation) Color.White else LuxuryGreen
                    )
                }
                Divider(color = LuxuryCyan.copy(0.1f), thickness = 0.5.dp)
                // ──────────────────────────────────────────────────────────────────

                DebugRow("Dist", "${"%.2f".format(totalDistanceM)} m", LuxuryGreen)
                DebugRow("Speed", "${"%.2f".format(speedMs)} m/s", LuxuryGreen)
                DebugRow("Quality", "${"%.0f".format(qualityPct)}%",
                    when {
                        qualityPct > 70f -> LuxuryGreen
                        qualityPct > 30f -> LuxuryYellow
                        else -> LuxuryRed
                    })
                DebugRow("Mode", fusionMode,
                    when (fusionMode) {
                        "CAMERA" -> LuxuryGreen
                        "HYBRID" -> LuxuryCyan
                        "IMU" -> LuxuryYellow
                        else -> LuxuryTextGrey
                    })
                DebugRow("Scale", "${"%.4f".format(scaleFactor)}", LuxuryCyan)
                DebugRow("Heading", "${"%.1f".format(headingDeg)}\u00B0", LuxuryTextGrey)

                // ── USER HEIGHT ───────────────────────────────────────────────────
                Divider(color = LuxuryCyan.copy(0.2f), thickness = 0.5.dp)
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Height", color = LuxuryTextGrey, fontSize = 10.sp)
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        SmallFloatingActionButton(
                            onClick = { viewModel.updateUserHeight(viewModel.userHeight - 0.05f) },
                            containerColor = LuxuryDarkGrey,
                            modifier = Modifier.size(20.dp)
                        ) { Text("-", color = Color.White, fontSize = 10.sp) }
                        Text(
                            "${"%.2f".format(viewModel.userHeight)}m",
                            color = LuxuryGreen,
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold,
                            modifier = Modifier.padding(horizontal = 4.dp)
                        )
                        SmallFloatingActionButton(
                            onClick = { viewModel.updateUserHeight(viewModel.userHeight + 0.05f) },
                            containerColor = LuxuryDarkGrey,
                            modifier = Modifier.size(20.dp)
                        ) { Text("+", color = Color.White, fontSize = 10.sp) }
                    }
                }

                // ── SCALE CALIBRATION ─────────────────────────────────────────────
                Divider(color = LuxuryCyan.copy(0.2f), thickness = 0.5.dp)
                Surface(
                    onClick = { calibrationExpanded = !calibrationExpanded },
                    color = Color.Transparent,
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth().padding(vertical = 2.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text("Calibration", color = LuxuryCyan, fontSize = 10.sp, fontWeight = FontWeight.Bold)
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Text("${"%.2f".format(calibFactor)}x", color = LuxuryGreen, fontSize = 10.sp, fontWeight = FontWeight.Bold)
                            Icon(
                                if (calibrationExpanded) Icons.Default.KeyboardArrowUp else Icons.Default.KeyboardArrowDown,
                                contentDescription = null,
                                tint = LuxuryTextGrey,
                                modifier = Modifier.size(14.dp)
                            )
                        }
                    }
                }

                if (calibrationExpanded) {
                    if (session != null && currentVio.isInitialized) {
                        val closure = sqrt(
                            (currentVio.x - session.startX).pow(2) +
                                (currentVio.z - session.startZ).pow(2)
                        )
                        val avgQuality = session.sumQuality / max(1, session.sampleCount)
                        Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
                            DebugRow("Path", "${"%.1f".format(session.pathLengthMeters)} m", Color.White)
                            DebugRow("Out", "${"%.1f".format(session.maxDistanceFromStartMeters)} m", LuxuryGreen)
                            DebugRow("Back", "${"%.1f".format(closure)} m", LuxuryYellow)
                            DebugRow("Avg Q", "${"%.2f".format(avgQuality)}", LuxuryTextGrey)
                        }
                    }

                    if (session == null) {
                        Text("Walk out & back to calibrate scale", color = LuxuryTextGrey, fontSize = 9.sp)
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.spacedBy(4.dp)
                        ) {
                            Button(
                                onClick = { viewModel.startScaleCalibration(5.0) },
                                modifier = Modifier.weight(1f).height(26.dp),
                                contentPadding = PaddingValues(0.dp),
                                colors = ButtonDefaults.buttonColors(containerColor = LuxuryGreen),
                                shape = RoundedCornerShape(4.dp)
                            ) {
                                Text("5m", color = LuxuryBlack, fontSize = 9.sp, fontWeight = FontWeight.Bold)
                            }
                            Button(
                                onClick = { viewModel.startScaleCalibration(10.0) },
                                modifier = Modifier.weight(1f).height(26.dp),
                                contentPadding = PaddingValues(0.dp),
                                colors = ButtonDefaults.buttonColors(containerColor = LuxuryCyan),
                                shape = RoundedCornerShape(4.dp)
                            ) {
                                Text("10m", color = LuxuryBlack, fontSize = 9.sp, fontWeight = FontWeight.Bold)
                            }
                            OutlinedButton(
                                onClick = { viewModel.resetScaleCalibration() },
                                modifier = Modifier.weight(1f).height(26.dp),
                                contentPadding = PaddingValues(0.dp),
                                border = BorderStroke(1.dp, LuxuryTextGrey),
                                shape = RoundedCornerShape(4.dp)
                            ) {
                                Text("Reset", color = Color.White, fontSize = 9.sp)
                            }
                        }
                    } else {
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.spacedBy(4.dp)
                        ) {
                            Button(
                                onClick = { viewModel.finishScaleCalibration() },
                                modifier = Modifier.weight(1f).height(26.dp),
                                contentPadding = PaddingValues(0.dp),
                                colors = ButtonDefaults.buttonColors(containerColor = LuxuryGreen),
                                shape = RoundedCornerShape(4.dp)
                            ) {
                                Text("Finish", color = LuxuryBlack, fontSize = 9.sp, fontWeight = FontWeight.Bold)
                            }
                            OutlinedButton(
                                onClick = { viewModel.cancelScaleCalibration() },
                                modifier = Modifier.weight(1f).height(26.dp),
                                contentPadding = PaddingValues(0.dp),
                                border = BorderStroke(1.dp, LuxuryTextGrey),
                                shape = RoundedCornerShape(4.dp)
                            ) {
                                Text("Cancel", color = Color.White, fontSize = 9.sp)
                            }
                        }
                    }

                    if (calibMessage != null) {
                        Text(
                            calibMessage,
                            color = LuxuryYellow,
                            fontSize = 9.sp,
                            modifier = Modifier.padding(top = 2.dp)
                        )
                    }
                }
            }
        }
    }


    @Composable
    fun NavigationMapWrapper(start: LatLng, azimuth: Float, history: List<Pair<Float, Float>>) {
        val navState = viewModel.navigationState
        val displayPosition = viewModel.snappedPosition ?: NavSightUtils.metersToLatLng(start, viewModel.virtualX, viewModel.virtualZ)
        val displayAzimuth by animateFloatAsState(targetValue = azimuth, animationSpec = spring(stiffness = Spring.StiffnessLow), label = "azimuth")
        val targetZoom = if (navState is NavigationState.Active) 19f else 18f
        val targetTilt = if (navState is NavigationState.Active) 60f else 30f

        val cameraState = rememberCameraPositionState {
            position = CameraPosition.Builder().target(displayPosition).zoom(targetZoom).bearing(azimuth).tilt(targetTilt).build()
        }

        // FR31: Throttle map camera updates to 1Hz to fix lag
        var lastMapUpdateTime by remember { mutableStateOf(0L) }
        LaunchedEffect(displayPosition, azimuth, navState) {
            val now = System.currentTimeMillis()
            // Re-center if enough time passed OR if navigation just became active
            if (now - lastMapUpdateTime >= 1000L || navState is NavigationState.Routing) {
                lastMapUpdateTime = now
                cameraState.animate(com.google.android.gms.maps.CameraUpdateFactory.newCameraPosition(
                    CameraPosition.Builder().target(displayPosition).zoom(targetZoom).bearing(azimuth).tilt(targetTilt).build()
                ), durationMs = 800)
            }
        }
        GoogleMap(modifier = Modifier.fillMaxSize(), cameraPositionState = cameraState, uiSettings = MapUiSettings(zoomControlsEnabled = false, compassEnabled = false, myLocationButtonEnabled = false)) {
            if (navState is NavigationState.Active) {
                Polyline(points = navState.route.polyline, color = LuxuryCyan, width = 12f, zIndex = 10f)
                Marker(state = MarkerState(navState.route.destination), title = "Destination")
            }
            if (history.isNotEmpty()) {
                Polyline(points = history.map { NavSightUtils.metersToLatLng(start, it.first.toDouble(), it.second.toDouble()) }, color = LuxuryGreen, width = 6f, zIndex = 5f)
            }
            Marker(state = MarkerState(displayPosition), rotation = displayAzimuth, flat = true, anchor = Offset(0.5f, 0.5f), icon = NavSightUtils.vectorToBitmap(this@MainActivity, R.drawable.navigation_arrow))
            Marker(state = MarkerState(start), title = "Start", icon = BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN))
        }
    }

    @Composable
    fun DestinationSearchBar(onDestinationSelected: (LatLng) -> Unit) {
        var searchText by remember { mutableStateOf("") }
        var predictions by remember { mutableStateOf<List<PlacePrediction>>(emptyList()) }
        var isSearching by remember { mutableStateOf(false) }
        val sessionToken = remember { com.google.android.libraries.places.api.model.AutocompleteSessionToken.newInstance() }
        val scope = rememberCoroutineScope()
        Column(modifier = Modifier.fillMaxWidth()) {
            Surface(color = Color.White, shape = RoundedCornerShape(12.dp), shadowElevation = 8.dp) {
                Row(modifier = Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 12.dp), verticalAlignment = Alignment.CenterVertically) {
                    Icon(Icons.Default.Place, null, tint = Color(0xFFB0B0B0), modifier = Modifier.size(24.dp))
                    Spacer(Modifier.width(12.dp))
                    BasicTextField(value = searchText, onValueChange = { query ->
                        searchText = query
                        if (query.length >= 2) {
                            scope.launch { fetchPlacePredictions(query, sessionToken) { predictions = it } }
                        } else predictions = emptyList()
                    }, modifier = Modifier.weight(1f), textStyle = TextStyle(color = Color.Black, fontSize = 16.sp), singleLine = true, decorationBox = { if (searchText.isEmpty()) Text("Where to?", color = Color(0xFFB0B0B0), fontSize = 16.sp); it() })
                    if (searchText.isNotEmpty()) IconButton(onClick = { searchText = ""; predictions = emptyList() }, modifier = Modifier.size(32.dp)) { Icon(Icons.Default.Close, null, tint = Color.Gray) }
                }
            }
            if (predictions.isNotEmpty()) {
                Spacer(Modifier.height(4.dp))
                Surface(color = Color.White, shape = RoundedCornerShape(12.dp), shadowElevation = 4.dp) {
                    Column {
                        predictions.forEach { prediction ->
                            Surface(onClick = {
                                isSearching = true
                                fetchPlaceLatLng(prediction.placeId) { latLng ->
                                    isSearching = false
                                    if (latLng != null) { onDestinationSelected(latLng); searchText = ""; predictions = emptyList() }
                                }
                            }, color = Color.Transparent) {
                                Column(modifier = Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 10.dp)) {
                                    Text(prediction.primaryText, color = Color.Black, fontWeight = FontWeight.Medium, fontSize = 14.sp)
                                    Text(prediction.secondaryText, color = Color(0xFF888888), fontSize = 12.sp)
                                }
                            }
                        }
                    }
                }
            }
            if (isSearching) { Spacer(Modifier.height(4.dp)); LinearProgressIndicator(modifier = Modifier.fillMaxWidth(), color = LuxuryCyan) }
        }
    }

    private fun fetchPlacePredictions(query: String, sessionToken: com.google.android.libraries.places.api.model.AutocompleteSessionToken, onResult: (List<PlacePrediction>) -> Unit) {
        val request = com.google.android.libraries.places.api.net.FindAutocompletePredictionsRequest.builder().setSessionToken(sessionToken).setQuery(query).build()
        viewModel.placesClient.findAutocompletePredictions(request).addOnSuccessListener { response ->
            onResult(response.autocompletePredictions.map { PlacePrediction(it.placeId, it.getPrimaryText(null).toString(), it.getSecondaryText(null).toString()) })
        }.addOnFailureListener { onResult(emptyList()) }
    }

    private fun fetchPlaceLatLng(placeId: String, onResult: (LatLng?) -> Unit) {
        val fields = listOf(com.google.android.libraries.places.api.model.Place.Field.LAT_LNG)
        val request = com.google.android.libraries.places.api.net.FetchPlaceRequest.newInstance(placeId, fields)
        viewModel.placesClient.fetchPlace(request).addOnSuccessListener { onResult(it.place.latLng) }.addOnFailureListener { onResult(null) }
    }

    data class PlacePrediction(val placeId: String, val primaryText: String, val secondaryText: String)

    @Composable
    fun NavigationInstructionBanner(instruction: NavigationInstruction, remainingDistanceMeters: Double, remainingTimeSeconds: Int) {
        Surface(color = Color.Black.copy(0.9f), shape = RoundedCornerShape(16.dp), border = BorderStroke(2.dp, LuxuryCyan)) {
            Row(modifier = Modifier.padding(16.dp).fillMaxWidth(), verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.SpaceBetween) {
                Icon(imageVector = instruction.getManeuverIcon(), contentDescription = null, tint = LuxuryCyan, modifier = Modifier.size(48.dp))
                Spacer(Modifier.width(16.dp))
                Column(modifier = Modifier.weight(1f)) {
                    Text(instruction.streetName ?: "Continue", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 18.sp)
                    Text("in ${NavSightUtils.formatDistance(instruction.distanceMeters)}", color = Color(0xFFB0B0B0), fontSize = 14.sp)
                }
                Column(horizontalAlignment = Alignment.End) {
                    Text(NavSightUtils.formatTime(remainingTimeSeconds), color = LuxuryGreen, fontWeight = FontWeight.Bold, fontSize = 16.sp)
                    Text(NavSightUtils.formatDistance(remainingDistanceMeters.toInt()), color = Color(0xFFB0B0B0), fontSize = 12.sp)
                }
            }
        }
    }
}
