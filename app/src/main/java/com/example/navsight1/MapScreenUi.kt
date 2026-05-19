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
    var calibrationVisible  by remember { mutableStateOf(false) }
    // 2026-05-17 — Allan-variance IMU recorder overlay. Standalone, does not
    // run VIO/camera/GPS. Long-running (1-2 hours) for noise characterization.
    var imuCalibrationVisible by remember { mutableStateOf(false) }
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

        // Camera — always alive, invisible when not shown.
        // Step 1 (Visual plan): re-key on `calibrationVisible` so the camera
        // analyzer rebinds to the activity lifecycle after the calibration
        // screen unbinds the back camera on dispose.
        key(calibrationVisible) {
            Box(modifier = if (cameraVisible && !calibrationVisible) Modifier.fillMaxSize() else Modifier.size(1.dp).alpha(0f)) {
                if (!calibrationVisible) {
                    CameraViewComposable(viewModel)
                    // Phase 1 camera overlay: KLT tracked-feature dots.
                    // Gated on `cameraVisible` so the always-alive offscreen
                    // camera (1-dp invisible box) does not pay the per-frame
                    // recomposition cost. The composable itself early-returns
                    // when geometry/points aren't available yet.
                    if (cameraVisible) {
                        CameraFeatureOverlay(viewModel, pal)
                        // Phase 6.5 (post_v19_sprint_plan.md §205-298):
                        // persistent LandmarkMap dots (orange = observed
                        // this frame, gray = dormant). Drawn BEFORE
                        // SlamFeatureOverlay so the live EKF SLAM dots
                        // sit on top of the static landmark layer.
                        LandmarkOverlay(viewModel, pal)
                        // Phase 3: world-anchored 3D SLAM points (orange
                        // dots with white ring outline). Reprojected each
                        // frame from the EKF state, so they stick to
                        // physical 3D points across pan-and-return motions.
                        SlamFeatureOverlay(viewModel, pal)
                        // Phase 4: 1-second flash banner on every accepted
                        // loop closure correction. Triggers from EventCounters
                        // increments; reads viewModel.loopClosureFlashUntilMs.
                        LoopClosureFlash(viewModel)
                    }
                }
            }
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
                NavigationMapWrapper(
                    mapStart, fusedHeading, historySnapshot, pal, viewModel,
                    Modifier.fillMaxSize()
                )
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
                Row(verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    VioStatusChip(
                        isInitialized = vio.isInitialized,
                        covValid      = viewModel.positionCovValid,
                        sigmaM        = viewModel.positionSigmaM,
                        pal           = pal
                    )
                    CalibrationStatusPill(
                        loaded = viewModel.calibrationLoaded,
                        pal = pal,
                        onClick = { calibrationVisible = true },
                    )
                }
                if (!viewModel.calibrationLoaded) {
                    Spacer(Modifier.height(6.dp))
                    CalibrationFirstLaunchBanner(pal) { calibrationVisible = true }
                }
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
                    onExpandSheet  = { bottomSheetExpanded = !bottomSheetExpanded },
                    onCalibrateClick = { calibrationVisible = true }
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
            onResetClick    = { viewModel.resetAll() },
            onStopNavClick  = { viewModel.stopNavigation() },
            onNightToggle   = onToggleNight,
            onToggleExpanded = { bottomSheetExpanded = !bottomSheetExpanded }
        )

        // Step 1 (Visual plan): full-screen camera calibration overlay.
        AnimatedVisibility(
            calibrationVisible,
            enter = fadeIn(tween(200)),
            exit  = fadeOut(tween(200)),
            modifier = Modifier.fillMaxSize(),
        ) {
            Box(Modifier.fillMaxSize().background(pal.bg)) {
                CalibrationScreen(
                    viewModel = viewModel,
                    pal = pal,
                    onClose = {
                        calibrationVisible = false
                        viewModel.refreshCalibrationLoaded()
                    },
                    onOpenImuCalibration = {
                        calibrationVisible = false
                        imuCalibrationVisible = true
                    },
                )
            }
        }

        // 2026-05-17 — Allan-variance IMU calibration overlay. Independent
        // of camera calibration; opened via a button on the CalibrationScreen.
        AnimatedVisibility(
            imuCalibrationVisible,
            enter = fadeIn(tween(200)),
            exit  = fadeOut(tween(200)),
            modifier = Modifier.fillMaxSize(),
        ) {
            Box(Modifier.fillMaxSize().background(pal.bg)) {
                ImuCalibrationScreen(
                    pal = pal,
                    onClose = { imuCalibrationVisible = false },
                )
            }
        }

        // Step 5: prompt user to place phone flat when stationary calibration timed out.
        if (viewModel.initStatus == SensorRepository.InitStatus.TIMEOUT_NEEDS_USER) {
            AlertDialog(
                onDismissRequest = { /* keep modal until user confirms */ },
                title = { Text("Hold steady") },
                text = {
                    Text(
                        "Place the phone flat on a stable surface for 5 seconds, " +
                        "then tap OK to calibrate."
                    )
                },
                confirmButton = {
                    TextButton(onClick = { viewModel.clearInitTimeout() }) { Text("OK") }
                }
            )
        }
    }
}

@Composable
fun NavigationMapWrapper(
    start: LatLng, azimuth: Float,
    history: List<PathPoint>, pal: NavPalette,
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
            mapPath = history.map { NavSightUtils.metersToLatLng(start, it.x.toDouble(), it.z.toDouble()) }
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
            // Step 6 (Task #29): 1σ horizontal-plane uncertainty ring around the user.
            // Radius is the sigma reported by the EKF, clamped to [0.5 m, 25 m] so the
            // overlay stays visible without dominating the screen during a degraded run.
            val sigma = viewModel.positionSigmaM
            if (viewModel.positionCovValid && sigma.isFinite() && sigma > 0f) {
                val ringColor = when {
                    sigma < 0.5f -> Teal500
                    sigma < 1.5f -> Orange400
                    else         -> Color(0xFFEF5350)
                }
                Circle(
                    center      = mapPos,
                    radius      = sigma.coerceIn(0.5f, 25f).toDouble(),
                    strokeColor = ringColor.copy(alpha = 0.85f),
                    strokeWidth = 4f,
                    fillColor   = ringColor.copy(alpha = 0.12f),
                    zIndex      = 4f
                )
            }
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
fun SensorRadarWaze(history: List<PathPoint>, currentAzimuth: Float, pal: NavPalette) {
    val dist = if (history.isEmpty()) 0f else sqrt(
        history.last().x.toDouble().pow(2) + history.last().z.toDouble().pow(2)).toFloat()

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
                    val curX = history.last().x; val curZ = history.last().z
                    val rad  = Math.toRadians((-currentAzimuth).toDouble())
                    val cosA = cos(rad).toFloat(); val sinA = sin(rad).toFloat()
                    fun toC(px: Float, pz: Float): Offset {
                        val dx = px - curX; val dz = pz - curZ
                        return Offset((cx + (dx * cosA - dz * sinA) * mToPx).coerceIn(0f, size.width),
                            (cy - (dx * sinA + dz * cosA) * mToPx).coerceIn(0f, size.height))
                    }
                    // Step 6 (Task #30): trajectory points colored by EKF position
                    // uncertainty σ. Bins: σ<0.5m (good)=teal, 0.5–1.5m (caution)=orange,
                    // ≥1.5m or NaN (degraded/lost)=red. Pre-init samples (NaN) treated
                    // as the worst class so the user can see the calibration tail.
                    val n = history.size; val si = maxOf(1, n - 120)
                    for (i in si until n) {
                        val sig = history[i].sigmaM
                        val col = when {
                            sig.isNaN() -> Color(0xFFEF5350)
                            sig < 0.5f  -> Teal500
                            sig < 1.5f  -> Orange400
                            else        -> Color(0xFFEF5350)
                        }
                        val fade = (0.3f + 0.7f * (i.toFloat() / n)).coerceIn(0.25f, 1f)
                        drawLine(col.copy(fade), toC(history[i-1].x, history[i-1].z),
                            toC(history[i].x, history[i].z), strokeWidth = 2.5f, cap = StrokeCap.Round)
                    }
                    drawCircle(Orange400, 4.5f, toC(history.first().x, history.first().z))
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

/**
 * Step 6: VIO health chip driven by EKF position covariance.
 *
 * - !isInitialized                → red   "VIO LOST — WALK FORWARD TO RE-ACQUIRE"
 * - covValid && σ <  1.5 m        → teal  "GPS-DENIED — VIO ACTIVE (σ = X.X m)"
 * - covValid && σ >= 1.5 m        → orange "VIO DEGRADED (σ = X.X m)"
 * - initialized && !covValid      → gray  "VIO INITIALIZING"
 */
@Composable
fun VioStatusChip(
    isInitialized: Boolean,
    covValid: Boolean,
    sigmaM: Float,
    pal: NavPalette
) {
    val (label, bg) = when {
        !isInitialized -> "VIO LOST — WALK FORWARD TO RE-ACQUIRE" to Color(0xFFEF5350)
        !covValid || sigmaM.isNaN() -> "VIO INITIALIZING" to Color(0xFF9E9E9E)
        sigmaM < 1.5f -> "GPS-DENIED — VIO ACTIVE (σ = ${"%.1f".format(sigmaM)} m)" to Teal500
        else -> "VIO DEGRADED (σ = ${"%.1f".format(sigmaM)} m)" to Orange400
    }
    Surface(
        color = bg.copy(alpha = 0.92f),
        shape = RoundedCornerShape(10.dp),
        shadowElevation = 4.dp
    ) {
        Row(
            Modifier.padding(horizontal = 10.dp, vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                label,
                color = Color.White,
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                letterSpacing = 0.4.sp
            )
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
fun MapActionStack(
    pal: NavPalette,
    onCameraClick: () -> Unit,
    onDebugClick: () -> Unit,
    onExpandSheet: () -> Unit,
    onCalibrateClick: () -> Unit = {},
) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp), horizontalAlignment = Alignment.CenterHorizontally) {
        FloatingMapButton(Icons.Default.PhotoCamera, onCameraClick)
        FloatingMapButton(Icons.Default.Layers, onExpandSheet)
        FloatingMapButton(Icons.Default.CenterFocusStrong, onCalibrateClick)
        FloatingMapButton(Icons.Default.Tune, onDebugClick, Color.Black, Color.White)
    }
}

/**
 * Step 1 (Visual plan): main-screen calibration status pill. Shows a green
 * checkmark + "Calibrated" when camera_calib.json exists at startup, or a grey
 * "Tap to calibrate" pill when missing — tapping opens the calibration screen.
 */
@Composable
fun CalibrationStatusPill(loaded: Boolean, pal: NavPalette, onClick: () -> Unit) {
    val bg = if (loaded) Teal500 else Color(0xFF9E9E9E)
    val label = if (loaded) "Calibrated" else "Tap to calibrate"
    val icon = if (loaded) Icons.Default.CheckCircle else Icons.Default.CenterFocusStrong
    Surface(
        onClick = onClick,
        color = bg.copy(alpha = 0.92f),
        shape = RoundedCornerShape(10.dp),
        shadowElevation = 4.dp,
    ) {
        Row(
            Modifier.padding(horizontal = 8.dp, vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Icon(icon, null, tint = Color.White, modifier = Modifier.size(12.dp))
            Spacer(Modifier.width(4.dp))
            Text(label, color = Color.White, fontSize = 10.sp, fontWeight = FontWeight.Bold,
                letterSpacing = 0.4.sp)
        }
    }
}

/**
 * Step 1 (Visual plan): non-dismissable banner shown above the map when
 * camera_calib.json is missing. Tapping opens the calibration flow. The
 * banner disappears once [NavSightViewModel.calibrationLoaded] flips to true.
 */
@Composable
fun CalibrationFirstLaunchBanner(pal: NavPalette, onClick: () -> Unit) {
    Surface(
        onClick = onClick,
        color = Color(0xFFFFB300).copy(0.96f),
        shape = RoundedCornerShape(12.dp),
        shadowElevation = 4.dp,
        modifier = Modifier.fillMaxWidth(),
    ) {
        Row(
            Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Icon(Icons.Default.Warning, null, tint = Color.White, modifier = Modifier.size(18.dp))
            Spacer(Modifier.width(8.dp))
            Column(Modifier.weight(1f)) {
                Text("Camera not calibrated — tap here to set up (~2 min)",
                    color = Color.White, fontSize = 12.sp, fontWeight = FontWeight.Bold)
                Text("VIO accuracy reduced until done.",
                    color = Color.White.copy(0.85f), fontSize = 10.sp)
            }
            Icon(Icons.Default.KeyboardArrowRight, null, tint = Color.White, modifier = Modifier.size(18.dp))
        }
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
