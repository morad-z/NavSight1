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
                    viewModel.requestInitialLocation()
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
        val flowResult = viewModel.flowResultState
        val vio = viewModel.vioState
        val azimuth = orientation.azimuth

        // Heading fusion
        val fusedHeading: Float = if (vio.isInitialized) {
            // This logic is simplified for the example; normally you'd use the offset captured at init
            val vioYawDeg = Math.toDegrees(-vio.yaw).toDouble()
            ((azimuth + vioYawDeg).toFloat() % 360 + 360) % 360
        } else {
            azimuth
        }

        val totalDistanceM = remember(viewModel.pathHistory.size) {
            val h = viewModel.pathHistory.toList()
            if (h.size < 2) 0.0
            else h.zipWithNext().sumOf { (a, b) ->
                val dx = (b.first - a.first).toDouble()
                val dz = (b.second - a.second).toDouble()
                sqrt(dx * dx + dz * dz)
            }
        }

        Box(Modifier.fillMaxSize().background(LuxuryBlack)) {
            Column(Modifier.fillMaxSize()) {
                Box(
                    modifier = Modifier
                        .weight(0.55f)
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
                    AROverlay(flowResult = flowResult, orientation = orientation)

                    Box(Modifier.align(Alignment.TopEnd).padding(12.dp)) {
                        SensorRadar(
                            history = viewModel.pathHistory.toList(),
                            currentAzimuth = fusedHeading
                        )
                    }

                    Box(Modifier.align(Alignment.TopStart).padding(12.dp)) {
                        DirectionBadge(
                            direction = flowResult.direction,
                            mode = flowResult.mode,
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
                            confidence = flowResult.confidence
                        )
                    }
                }

                Box(
                    modifier = Modifier
                        .weight(0.45f)
                        .fillMaxWidth()
                        .padding(start = 7.dp, end = 7.dp, top = 7.dp, bottom = 50.dp)
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
                        Box(Modifier.align(Alignment.TopCenter).padding(16.dp).fillMaxWidth(0.95f)) {
                            DestinationSearchBar(
                                onDestinationSelected = { destination ->
                                    viewModel.startNavigation(destination)
                                }
                            )
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
                            Text("${"%.0f".format(totalDistanceM)} m", color = LuxuryGreen, fontSize = 12.sp)
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
        flowResult: OpticalFlowProcessor.FlowResult,
        orientation: DeviceOrientationTracker.OrientationResult
    ) {
        if (flowResult.direction != OpticalFlowProcessor.MovementDirection.STOPPED && orientation.isHorizontal) {
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
            AROverlayRenderer.DirectionArrow(direction = flowResult.direction, magnitude = flowResult.magnitude)
        }
    }

    @Composable
    fun DirectionBadge(direction: OpticalFlowProcessor.MovementDirection, mode: OpticalFlowProcessor.MovementMode, trackingQuality: Float) {
        val lowQuality = trackingQuality < 0.3f && trackingQuality > 0f
        val (text, color, icon) = if (lowQuality) Triple("LOW QUALITY", LuxuryRed, Icons.Default.Place)
        else when (direction) {
            OpticalFlowProcessor.MovementDirection.FORWARD -> Triple("קדימה", LuxuryGreen, Icons.Default.KeyboardArrowUp)
            OpticalFlowProcessor.MovementDirection.BACKWARD -> Triple("אחורה", LuxuryRed, Icons.Default.KeyboardArrowDown)
            OpticalFlowProcessor.MovementDirection.LEFT -> Triple("שמאלה", LuxuryCyan, Icons.Default.ArrowBack)
            OpticalFlowProcessor.MovementDirection.RIGHT -> Triple("ימינה", LuxuryCyan, Icons.Default.ArrowForward)
            OpticalFlowProcessor.MovementDirection.STOPPED -> Triple("עומד", LuxuryYellow, Icons.Default.Place)
        }
        Surface(color = Color.Black.copy(0.8f), shape = RoundedCornerShape(12.dp), border = BorderStroke(1.dp, color)) {
            Row(modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp), verticalAlignment = Alignment.CenterVertically) {
                Text(if (mode == OpticalFlowProcessor.MovementMode.WALKING) "🚶" else "🚗", fontSize = 16.sp)
                Spacer(Modifier.width(6.dp))
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
                    Text("החזק את הטלפון אופקית", color = LuxuryRed, fontWeight = FontWeight.Bold, fontSize = 14.sp)
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
        Card(shape = RoundedCornerShape(12.dp), colors = CardDefaults.cardColors(containerColor = Color.Black.copy(0.85f)), border = BorderStroke(1.dp, LuxuryGreen), modifier = Modifier.size(110.dp)) {
            Box(Modifier.fillMaxSize()) {
                Canvas(Modifier.fillMaxSize().padding(8.dp)) {
                    val cx = size.width / 2
                    val cy = size.height / 2
                    val radius = minOf(cx, cy)
                    for (i in 1..3) drawCircle(color = LuxuryGreen.copy(0.15f), radius = radius * i / 3, style = Stroke(1f))
                    drawLine(LuxuryGreen.copy(0.2f), Offset(cx, 0f), Offset(cx, size.height))
                    drawLine(LuxuryGreen.copy(0.2f), Offset(0f, cy), Offset(size.width, cy))
                    if (history.isNotEmpty()) {
                        val path = Path()
                        val scale = if (history.size > 1) {
                            val span = maxOf(history.maxOf { it.first } - history.minOf { it.first }, history.maxOf { it.second } - history.minOf { it.second }, 1f)
                            (radius * 0.8f) / (span / 2f)
                        } else 3f
                        val rad = Math.toRadians((-currentAzimuth).toDouble())
                        val cosA = cos(rad).toFloat()
                        val sinA = sin(rad).toFloat()
                        val (curX, curZ) = history.last()
                        fun transform(pX: Float, pZ: Float): Offset {
                            val dx = pX - curX
                            val dz = pZ - curZ
                            val rotX = dx * cosA - dz * sinA
                            val rotZ = dx * sinA + dz * cosA
                            return Offset((cx + rotX * scale).coerceIn(0f, size.width), (cy - rotZ * scale).coerceIn(0f, size.height))
                        }
                        val first = transform(history.first().first, history.first().second)
                        path.moveTo(first.x, first.y)
                        for (i in 1 until history.size) {
                            val p = transform(history[i].first, history[i].second)
                            path.lineTo(p.x, p.y)
                        }
                        drawPath(path, LuxuryGreen, style = Stroke(2f))
                        drawCircle(LuxuryYellow, 4f, first)
                    }
                    drawCircle(Color.White, 5f, Offset(cx, cy))
                    drawLine(LuxuryCyan, Offset(cx, cy), Offset(cx, cy - 15f), strokeWidth = 3f, cap = StrokeCap.Round)
                }
                Text("RADAR", color = LuxuryGreen.copy(0.6f), fontSize = 8.sp, modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 4.dp))
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
