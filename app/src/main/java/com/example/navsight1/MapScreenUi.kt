package com.example.navsight1

import androidx.compose.animation.*
import androidx.compose.animation.core.tween
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.google.android.gms.maps.model.BitmapDescriptorFactory
import com.google.android.gms.maps.model.CameraPosition
import com.google.android.gms.maps.model.LatLng
import com.google.maps.android.compose.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.*

@Composable
fun MainScreen(viewModel: NavSightViewModel, pal: NavPalette, isNight: Boolean, onToggleNight: () -> Unit) {
    val orientation  = viewModel.orientationState
    val vio          = viewModel.vioState
    val fusedHeading = if (vio.isInitialized)
        ((Math.toDegrees(vio.heading).toFloat() % 360f) + 360f) % 360f
    else orientation.azimuth

    val historySnapshot = remember(viewModel.pathHistoryVersion) { viewModel.pathHistory.toList() }
    val navState        = viewModel.navigationState
    val instruction     = viewModel.currentInstruction
    val mapStart        = viewModel.startLocation
    val isMoving        = vio.isInitialized && vio.meanFlow > 1.0

    var cameraVisible       by remember { mutableStateOf(false) }
    var isRecordingGpx      by remember { mutableStateOf(false) }
    var gpxPoints           by remember { mutableStateOf<List<Pair<Double, Double>>>(emptyList()) }
    var gpxMessage          by remember { mutableStateOf<String?>(null) }
    var debugVisible        by remember { mutableStateOf(false) }
    var bottomSheetExpanded by rememberSaveable { mutableStateOf(false) }
    val scope               = rememberCoroutineScope()
    val context             = LocalContext.current
    val lowerOverlayPadding = if (bottomSheetExpanded) 260.dp else 88.dp

    val compassLabel = when {
        fusedHeading < 22.5f || fusedHeading >= 337.5f -> "N"
        fusedHeading < 67.5f  -> "NE"
        fusedHeading < 112.5f -> "E"
        fusedHeading < 157.5f -> "SE"
        fusedHeading < 202.5f -> "S"
        fusedHeading < 247.5f -> "SW"
        fusedHeading < 292.5f -> "W"
        else                  -> "NW"
    }
    val fusionMode = when {
        !vio.isInitialized        -> "INIT"
        vio.trackingQuality < 0.3 -> "IMU"
        vio.trackingQuality > 0.7 -> "CAMERA"
        else                      -> "HYBRID"
    }
    val qualityLevel = when { vio.trackingQuality < 0.3 -> 0; vio.trackingQuality < 0.7 -> 1; else -> 2 }
    val speedBucket  = (viewModel.currentSpeedKmh / 5f).toInt()
    val incidentItems = remember(navState, vio.isInitialized, qualityLevel, isMoving, speedBucket) {
        buildIncidentCards(navState, isMoving, viewModel.currentSpeedKmh,
            viewModel.totalDistanceM, vio.trackingQuality.toFloat(), vio.isInitialized)
    }

    Box(Modifier.fillMaxSize().background(pal.bg)) {

        // Camera — always alive, invisible when not shown
        Box(modifier = if (cameraVisible) Modifier.fillMaxSize() else Modifier.size(1.dp).alpha(0f)) {
            CameraViewComposable(viewModel)
        }

        // Map
        AnimatedVisibility(!cameraVisible, enter = fadeIn(tween(200)), exit = fadeOut(tween(200)),
            modifier = Modifier.fillMaxSize()) {
            if (mapStart == null) {
                Box(Modifier.fillMaxSize(), Alignment.Center) {
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        CircularProgressIndicator(color = pal.teal, strokeWidth = 2.5.dp, modifier = Modifier.size(38.dp))
                        Spacer(Modifier.height(14.dp))
                        Text("Acquiring location…", color = pal.textSecondary, fontSize = 13.sp)
                    }
                }
            } else {
                NavigationMapWrapper(mapStart, fusedHeading, historySnapshot, pal, viewModel, Modifier.fillMaxSize())
            }
        }

        // Camera full-screen overlay
        AnimatedVisibility(cameraVisible, enter = fadeIn(tween(200)), exit = fadeOut(tween(200)),
            modifier = Modifier.fillMaxSize()) {
            CameraOverlay(
                pal = pal, isMoving = isMoving,
                showCameraBlocked        = viewModel.showCameraBlocked,
                isVioInitialized         = vio.isInitialized,
                vioTrackingQuality       = vio.trackingQuality.toFloat(),
                vioTrackedFeatures       = vio.trackedFeatures,
                orientationIsHorizontal  = orientation.isHorizontal,
                orientationDeviation     = orientation.deviationFromHorizontal,
                stabilityScore           = orientation.stabilityScore,
                fusedHeading             = fusedHeading,
                historySnapshot          = historySnapshot,
                onClose                  = { cameraVisible = false },
                onDebug                  = { debugVisible = !debugVisible },
                debugVisible             = debugVisible,
                totalDistanceM           = viewModel.totalDistanceM,
                speedMs                  = viewModel.currentSpeedKmh / 3.6f,
                qualityPct               = (vio.trackingQuality * 100).toFloat(),
                fusionMode               = fusionMode,
                scaleFactor              = vio.estimatedScale,
                viewModel                = viewModel
            )
        }

        // Map overlays
        if (!cameraVisible) {
            Box(
                Modifier.fillMaxWidth().height(82.dp)
                    .background(Brush.verticalGradient(listOf(
                        HeroPurple.copy(alpha = 0.84f), HeroPurpleDark.copy(alpha = 0.58f),
                        HeroPurpleDark.copy(alpha = 0.10f), Color.Transparent)))
                    .align(Alignment.TopCenter)
            )

            Column(
                Modifier.align(Alignment.TopCenter).fillMaxWidth()
                    .statusBarsPadding().padding(horizontal = 10.dp, vertical = 6.dp)
            ) {
                HeroHeader(viewModel.currentSpeedKmh, compassLabel, fusionMode,
                    (vio.trackingQuality * 100).toFloat(), pal)
                Spacer(Modifier.height(4.dp))
                AnimatedVisibility(navState is NavigationState.Active && instruction != null,
                    enter = slideInVertically { -it } + fadeIn(), exit = slideOutVertically { -it } + fadeOut()) {
                    if (navState is NavigationState.Active && instruction != null) {
                        NavigationInstructionBanner(instruction, navState.remainingDistanceMeters,
                            navState.remainingTimeSeconds, pal)
                    }
                }
                AnimatedVisibility(navState is NavigationState.Idle, enter = fadeIn(), exit = fadeOut()) {
                    SearchBarCard(pal, viewModel.startLocation, viewModel.placesClient) { dest -> viewModel.startNavigation(dest) }
                }
                viewModel.navigationStartMessage?.let { msg ->
                    Spacer(Modifier.height(8.dp))
                    ErrorCard(msg, pal) { viewModel.clearNavigationStartMessage() }
                }
            }

            Column(
                modifier = Modifier.align(Alignment.CenterEnd).padding(end = 12.dp, bottom = lowerOverlayPadding),
                verticalArrangement   = Arrangement.spacedBy(10.dp),
                horizontalAlignment   = Alignment.End
            ) {
                AnimatedVisibility(navState is NavigationState.Idle, enter = fadeIn(), exit = fadeOut()) {
                    val radarHeading = remember(fusedHeading) { (Math.round(fusedHeading / 2f) * 2f) }
                    SensorRadarWaze(historySnapshot, radarHeading, pal)
                }
                MapActionStack(pal,
                    onCameraClick  = { cameraVisible = true },
                    onDebugClick   = { debugVisible = !debugVisible },
                    onExpandSheet  = { bottomSheetExpanded = !bottomSheetExpanded }
                )
            }

            SpeedLimitBadge(
                modifier = Modifier.align(Alignment.BottomStart)
                    .padding(start = 10.dp, bottom = lowerOverlayPadding + 42.dp),
                speedKmh = viewModel.currentSpeedKmh, pal = pal
            )

            if (isRecordingGpx) {
                Box(Modifier.align(Alignment.TopCenter).statusBarsPadding().padding(top = 80.dp)) { RecordingPill(pal) }
                val displayPos = viewModel.snappedPosition
                    ?: mapStart?.let { NavSightUtils.metersToLatLng(it, viewModel.virtualX, viewModel.virtualZ) }
                displayPos?.let { pos ->
                    LaunchedEffect(pos) { gpxPoints = gpxPoints + Pair(pos.latitude, pos.longitude) }
                }
            }

            gpxMessage?.let { msg ->
                LaunchedEffect(msg) { delay(3000); gpxMessage = null }
                Box(Modifier.align(Alignment.TopCenter).statusBarsPadding().padding(top = 80.dp)) { WazeToast(msg, pal) }
            }

            AnimatedVisibility(debugVisible,
                enter = fadeIn() + expandVertically(expandFrom = Alignment.Bottom),
                exit  = fadeOut() + shrinkVertically(shrinkTowards = Alignment.Bottom),
                modifier = Modifier.align(Alignment.BottomStart).padding(start = 10.dp, bottom = lowerOverlayPadding)) {
                DebugPanel(viewModel.totalDistanceM, viewModel.currentSpeedKmh / 3.6f,
                    (vio.trackingQuality * 100).toFloat(), fusionMode, vio.estimatedScale, fusedHeading, pal, viewModel)
            }
        }

        BottomSheet(
            modifier        = Modifier.align(Alignment.BottomCenter),
            pal             = pal,
            navState        = navState,
            isNight         = isNight,
            cameraVisible   = cameraVisible,
            isRecordingGpx  = isRecordingGpx,
            debugVisible    = debugVisible,
            speedKmh        = viewModel.currentSpeedKmh,
            totalM          = viewModel.totalDistanceM,
            compassLabel    = compassLabel,
            isMoving        = isMoving,
            fusionMode      = fusionMode,
            vioInitialized  = vio.isInitialized,
            incidents       = incidentItems,
            expanded        = bottomSheetExpanded,
            onCameraClick   = { cameraVisible = !cameraVisible },
            onGpxClick      = {
                if (!isRecordingGpx) { isRecordingGpx = true; gpxPoints = emptyList(); gpxMessage = "Recording…" }
                else { isRecordingGpx = false; scope.launch(Dispatchers.IO) { gpxMessage = saveGpxFile(context, gpxPoints) } }
            },
            onDebugClick    = { debugVisible = !debugVisible },
            onResetClick    = { viewModel.resetPath() },
            onStopNavClick  = { viewModel.stopNavigation() },
            onNightToggle   = onToggleNight,
            onToggleExpanded = { bottomSheetExpanded = !bottomSheetExpanded }
        )
    }
}

@Composable
fun NavigationMapWrapper(
    start: LatLng, azimuth: Float,
    history: List<Pair<Float, Float>>, pal: NavPalette,
    viewModel: NavSightViewModel, modifier: Modifier = Modifier
) {
    val navState   = viewModel.navigationState
    val displayPos = viewModel.snappedPosition
        ?: NavSightUtils.metersToLatLng(start, viewModel.virtualX, viewModel.virtualZ)
    val targetZoom = if (navState is NavigationState.Active) 19f else 18f
    val targetTilt = if (navState is NavigationState.Active) 60f else 30f
    val context    = LocalContext.current

    val cameraState = rememberCameraPositionState {
        position = CameraPosition.Builder().target(displayPos).zoom(targetZoom).bearing(azimuth).tilt(targetTilt).build()
    }
    var isFollowing by remember { mutableStateOf(true) }
    val curPos  by rememberUpdatedState(displayPos)
    val curAzi  by rememberUpdatedState(azimuth)
    val curZoom by rememberUpdatedState(targetZoom)
    val curTilt by rememberUpdatedState(targetTilt)

    LaunchedEffect(cameraState.isMoving) {
        if (cameraState.isMoving &&
            cameraState.cameraMoveStartedReason == CameraMoveStartedReason.GESTURE)
            isFollowing = false
    }
    LaunchedEffect(Unit) {
        var lastBearing = azimuth
        while (true) {
            val bearingChanged = kotlin.math.abs(((curAzi - lastBearing + 540f) % 360f) - 180f) > 1.5f
            if (isFollowing) {
                cameraState.animate(com.google.android.gms.maps.CameraUpdateFactory.newCameraPosition(
                    CameraPosition.Builder().target(curPos).zoom(curZoom).bearing(curAzi).tilt(curTilt).build()), 350)
                lastBearing = curAzi; delay(400L)
            } else {
                if (bearingChanged) {
                    cameraState.animate(com.google.android.gms.maps.CameraUpdateFactory.newCameraPosition(
                        CameraPosition.Builder().target(cameraState.position.target)
                            .zoom(cameraState.position.zoom).bearing(curAzi).tilt(cameraState.position.tilt).build()), 180)
                    lastBearing = curAzi
                }
                delay(160L)
            }
        }
    }

    var mapPos      by remember { mutableStateOf(displayPos) }
    var mapAzi      by remember { mutableStateOf(azimuth) }
    var mapPath     by remember { mutableStateOf<List<LatLng>>(emptyList()) }
    var lastOverlay by remember { mutableStateOf(0L) }

    LaunchedEffect(displayPos, azimuth, history.size) {
        val now = System.currentTimeMillis()
        val interval = if (navState is NavigationState.Active) 800L else 1200L
        if (now - lastOverlay >= interval || navState is NavigationState.Routing) {
            lastOverlay = now; mapPos = displayPos; mapAzi = azimuth
            mapPath = history.map { NavSightUtils.metersToLatLng(start, it.first.toDouble(), it.second.toDouble()) }
        }
    }

    Box(modifier) {
        GoogleMap(
            modifier            = Modifier.fillMaxSize(),
            cameraPositionState = cameraState,
            properties          = MapProperties(mapType = MapType.NORMAL, isMyLocationEnabled = false),
            uiSettings          = MapUiSettings(zoomControlsEnabled = false, compassEnabled = false,
                myLocationButtonEnabled = false, zoomGesturesEnabled = true, scrollGesturesEnabled = true,
                tiltGesturesEnabled = true, rotationGesturesEnabled = true)
        ) {
            val arrowIcon = remember { NavSightUtils.vectorToBitmap(context, R.drawable.navigation_arrow) }
            val startIcon = remember { BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_AZURE) }
            if (navState is NavigationState.Active) {
                Polyline(points = navState.route.polyline, color = pal.teal, width = 14f, zIndex = 10f)
                Marker(state = MarkerState(navState.route.destination), title = "Destination")
            }
            if (mapPath.isNotEmpty()) Polyline(points = mapPath, color = pal.orange.copy(0.8f), width = 7f, zIndex = 5f)
            Marker(state = MarkerState(mapPos), rotation = mapAzi, flat = true, anchor = Offset(0.5f, 0.5f), icon = arrowIcon)
            Marker(state = MarkerState(start), title = "Start", icon = startIcon)
        }
        if (!isFollowing) {
            FloatingActionButton(
                onClick        = { isFollowing = true },
                modifier       = Modifier.align(Alignment.BottomEnd).padding(end = 14.dp, bottom = 105.dp).size(52.dp),
                containerColor = pal.card, contentColor = pal.teal,
                elevation      = FloatingActionButtonDefaults.elevation(6.dp)
            ) { Icon(Icons.Default.Place, "Recenter", modifier = Modifier.size(26.dp)) }
        }
    }
}

@Composable
fun SensorRadarWaze(history: List<Pair<Float, Float>>, currentAzimuth: Float, pal: NavPalette) {
    val dist = if (history.isEmpty()) 0f else sqrt(
        history.last().first.toDouble().pow(2) + history.last().second.toDouble().pow(2)).toFloat()

    val rPaint = remember { android.graphics.Paint().apply {
        color = android.graphics.Color.argb(200, 0, 188, 212)
        textSize = 15f; textAlign = android.graphics.Paint.Align.CENTER
    }}
    val cPaint = remember { android.graphics.Paint().apply {
        color = android.graphics.Color.argb(230, 255, 255, 255)
        textSize = 18f; textAlign = android.graphics.Paint.Align.CENTER; isFakeBoldText = true
    }}

    Surface(shape = RoundedCornerShape(20.dp),
        color    = if (pal.isNight) Color(0xCC0A1628) else Color(0xE6FFFFFF),
        border   = BorderStroke(1.5.dp, pal.teal.copy(0.6f)),
        modifier = Modifier.size(118.dp).shadow(8.dp, RoundedCornerShape(20.dp))) {
        Box(Modifier.fillMaxSize()) {
            Canvas(Modifier.fillMaxSize().padding(8.dp)) {
                val cx = size.width / 2f; val cy = size.height / 2f
                val maxR = minOf(cx, cy); val mToPx = maxR / 5f
                listOf(1f to "1m", 2f to "2m", 5f to "5m").forEach { (d, lbl) ->
                    val rr = d * mToPx
                    drawCircle(Teal500.copy(0.20f), rr, Offset(cx, cy), style = Stroke(1.2f))
                    drawContext.canvas.nativeCanvas.drawText(lbl, cx + rr * 0.72f, cy - rr * 0.72f + 10f, rPaint)
                }
                drawContext.canvas.nativeCanvas.apply {
                    drawText("N", cx, cy - maxR + 15f, cPaint); drawText("S", cx, cy + maxR + 1f, cPaint)
                    drawText("E", cx + maxR + 1f, cy + 6f, cPaint); drawText("W", cx - maxR - 1f, cy + 6f, cPaint)
                }
                drawLine(Teal500.copy(0.18f), Offset(cx, cy - maxR), Offset(cx, cy + maxR), 1f)
                drawLine(Teal500.copy(0.18f), Offset(cx - maxR, cy), Offset(cx + maxR, cy), 1f)
                drawCircle(Teal500.copy(0.35f), maxR, Offset(cx, cy), style = Stroke(1.5f))
                if (history.size >= 2) {
                    val curX = history.last().first; val curZ = history.last().second
                    val rad  = Math.toRadians((-currentAzimuth).toDouble())
                    val cosA = cos(rad).toFloat(); val sinA = sin(rad).toFloat()
                    fun toC(px: Float, pz: Float): Offset {
                        val dx = px - curX; val dz = pz - curZ
                        return Offset((cx + (dx * cosA - dz * sinA) * mToPx).coerceIn(0f, size.width),
                            (cy - (dx * sinA + dz * cosA) * mToPx).coerceIn(0f, size.height))
                    }
                    val n = history.size; val si = maxOf(1, n - 120)
                    for (i in si until n) {
                        val dx = history[i].first - history[i-1].first; val dz = history[i].second - history[i-1].second
                        val mov = sqrt((dx*dx + dz*dz).toDouble()).toFloat()
                        val col = when { mov > 0.03f -> Teal500; mov > 0.008f -> Orange400; else -> Color(0xFFEF5350) }
                        val fade = (0.3f + 0.7f * (i.toFloat() / n)).coerceIn(0.25f, 1f)
                        drawLine(col.copy(fade), toC(history[i-1].first, history[i-1].second),
                            toC(history[i].first, history[i].second), strokeWidth = 2.5f, cap = StrokeCap.Round)
                    }
                    drawCircle(Orange400, 4.5f, toC(history.first().first, history.first().second))
                }
                drawCircle(Color.White, 5.5f, Offset(cx, cy))
                drawCircle(Teal500.copy(0.55f), 12f, Offset(cx, cy), style = Stroke(2.5f))
                val aLen = maxR * 0.42f; val aRad = Math.toRadians(currentAzimuth.toDouble())
                val tipX = cx + sin(aRad).toFloat() * aLen; val tipY = cy - cos(aRad).toFloat() * aLen
                drawLine(Teal500, Offset(cx, cy), Offset(tipX, tipY), strokeWidth = 3.5f, cap = StrokeCap.Round)
                for (off in listOf(145.0, -145.0)) {
                    val hr = Math.toRadians(currentAzimuth + off)
                    drawLine(Teal500, Offset(tipX, tipY),
                        Offset(tipX + sin(hr).toFloat() * 9f, tipY - cos(hr).toFloat() * 9f),
                        strokeWidth = 2.5f, cap = StrokeCap.Round)
                }
            }
            Text("${"%.1f".format(dist)}m", color = Orange400, fontSize = 10.sp, fontWeight = FontWeight.Bold,
                modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 4.dp))
            Text("RADAR", color = Teal500.copy(0.55f), fontSize = 7.sp, letterSpacing = 1.5.sp,
                modifier = Modifier.align(Alignment.TopCenter).padding(top = 4.dp))
        }
    }
}

@Composable
fun HeroHeader(speedKmh: Float, compassLabel: String, fusionMode: String, qualityPct: Float, pal: NavPalette) {
    Surface(color = Color.Transparent, shape = RoundedCornerShape(18.dp)) {
        Box(
            Modifier.fillMaxWidth()
                .background(Brush.verticalGradient(listOf(HeroPurple, HeroPurpleDark)), RoundedCornerShape(18.dp))
                .padding(horizontal = 10.dp, vertical = 5.dp)
        ) {
            Row(Modifier.fillMaxWidth(), Arrangement.SpaceBetween, Alignment.CenterVertically) {
                HeroMiniBadge(Icons.Default.Speed, "${qualityPct.toInt()}%", Color.White.copy(0.14f))
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    SpeedLimitCore(speedKmh)
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text("Live drive", color = Color.White.copy(0.72f), fontSize = 7.sp)
                        Text("${"%.0f".format(speedKmh)}", color = Color.White, fontSize = 18.sp,
                            fontWeight = FontWeight.ExtraBold, lineHeight = 18.sp)
                        Text("km/h", color = Color.White.copy(0.92f), fontSize = 7.sp)
                    }
                }
                HeroMiniBadge(Icons.Default.Explore, "$compassLabel • $fusionMode", Color.White.copy(0.14f))
            }
        }
    }
}

@Composable
fun HeroMiniBadge(icon: androidx.compose.ui.graphics.vector.ImageVector, text: String, bg: Color) {
    Surface(color = bg, shape = CircleShape) {
        Row(Modifier.padding(horizontal = 5.dp, vertical = 3.dp),
            verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(2.dp)) {
            Icon(icon, null, tint = Color.White, modifier = Modifier.size(9.dp))
            Text(text, color = Color.White, fontSize = 7.sp, fontWeight = FontWeight.Bold)
        }
    }
}

@Composable
fun SpeedLimitCore(speedKmh: Float) {
    Surface(color = Color.White, shape = CircleShape, border = BorderStroke(4.dp, Color(0xFFFF6A5E))) {
        Box(Modifier.size(42.dp), contentAlignment = Alignment.Center) {
            Text("${max(30, (ceil(speedKmh / 10f) * 10).toInt())}", color = Color.Black,
                fontSize = 14.sp, fontWeight = FontWeight.ExtraBold)
        }
    }
}

@Composable
fun SpeedLimitBadge(modifier: Modifier, speedKmh: Float, pal: NavPalette) {
    Surface(modifier = modifier, color = Color.White, shape = RoundedCornerShape(28.dp), shadowElevation = 10.dp) {
        Column(Modifier.padding(horizontal = 8.dp, vertical = 8.dp), horizontalAlignment = Alignment.CenterHorizontally) {
            SpeedLimitCore(speedKmh)
            Spacer(Modifier.height(4.dp))
            Text("${"%.0f".format(speedKmh.coerceAtLeast(0f))}", color = pal.textPrimary,
                fontSize = 16.sp, fontWeight = FontWeight.ExtraBold)
        }
    }
}

@Composable
fun MapActionStack(pal: NavPalette, onCameraClick: () -> Unit, onDebugClick: () -> Unit, onExpandSheet: () -> Unit) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp), horizontalAlignment = Alignment.CenterHorizontally) {
        FloatingMapButton(Icons.Default.PhotoCamera, onCameraClick)
        FloatingMapButton(Icons.Default.Layers, onExpandSheet)
        FloatingMapButton(Icons.Default.Tune, onDebugClick, Color.Black, Color.White)
    }
}

@Composable
fun FloatingMapButton(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    onClick: () -> Unit,
    containerColor: Color = Color.White,
    contentColor: Color   = Color.Black
) {
    Surface(onClick = onClick, color = containerColor, shape = CircleShape, shadowElevation = 8.dp) {
        Box(Modifier.size(50.dp), contentAlignment = Alignment.Center) {
            Icon(icon, null, tint = contentColor, modifier = Modifier.size(20.dp))
        }
    }
}
