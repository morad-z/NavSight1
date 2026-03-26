package com.example.navsight1

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.util.Log
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.animation.core.*
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.ArrowDropDown
import androidx.compose.material.icons.filled.ArrowForward
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
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.lifecycleScope
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.rememberMultiplePermissionsState
import com.google.android.gms.location.*
import com.google.android.gms.maps.model.*
import com.google.android.gms.tasks.CancellationTokenSource
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

/* ===================== MAIN ACTIVITY ===================== */

class MainActivity : ComponentActivity(), SensorEventListener {

    companion object {
        private const val TAG = "NavSight"
    }

    // Sensor Manager
    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var magnetometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private lateinit var fusedLocationClient: FusedLocationProviderClient

    // מנועים
    private val orientationTracker = DeviceOrientationTracker()
    private val opticalFlowProcessor = OpticalFlowProcessor()

    // UI States
    val orientationState = mutableStateOf(DeviceOrientationTracker.OrientationResult(
        pitch = 0f, roll = 0f, azimuth = 0f,
        isHorizontal = false, deviationFromHorizontal = 90f, stabilityScore = 0f
    ))
    
    val flowResultState = mutableStateOf(OpticalFlowProcessor.FlowResult(
        dx = 0f, dy = 0f, magnitude = 0f,
        direction = OpticalFlowProcessor.MovementDirection.STOPPED,
        confidence = 0f,
        mode = OpticalFlowProcessor.MovementMode.WALKING
    ))
    
    // מיקום וירטואלי (במטרים) עבור הרדאר
    private var virtualX = 0.0
    private var virtualZ = 0.0
    val pathHistory = mutableStateListOf<Pair<Float, Float>>()
    
    // GPS Start Location
    val startLocation = mutableStateOf<LatLng?>(null)
    
    // מהירות לפי Optical Flow
    private var lastFlowTime = 0L
    private val velocityScale = 0.01f // קנה מידה להמרת pixels למטרים

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Initialize location client
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)
        
        // Initialize sensors
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        Log.d(TAG, "Sensors initialized - Accel: ${accelerometer != null}, Mag: ${magnetometer != null}, Gyro: ${gyroscope != null}")

        setContent {
            MaterialTheme(colorScheme = darkColorScheme()) {
                NavSightApp()
            }
        }
    }

    override fun onResume() {
        super.onResume()
        // רישום לחיישנים
        accelerometer?.let { 
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) 
        }
        magnetometer?.let { 
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) 
        }
        gyroscope?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                orientationTracker.updateAccelerometer(event.values)
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                orientationTracker.updateMagnetometer(event.values)
            }
        }
        
        // עדכון אוריינטציה
        orientationState.value = orientationTracker.getOrientation()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    /**
     * עיבוד פריים מהמצלמה
     */
    fun processCameraFrame(frame: Frame) {
        val data = frame.getData<ByteArray>() ?: return
        
        // עיבוד Optical Flow
        val flowResult = opticalFlowProcessor.processFrame(
            frame = data,
            width = frame.size.width,
            height = frame.size.height,
            isYuv = true
        )
        
        // עדכון state
        flowResultState.value = flowResult
        
        // עדכון מיקום וירטואלי אם יש תנועה ואם הטלפון אופקי
        if (flowResult.direction != OpticalFlowProcessor.MovementDirection.STOPPED &&
            orientationState.value.isHorizontal) {
            
            val currentTime = System.currentTimeMillis()
            val deltaTime = if (lastFlowTime > 0) (currentTime - lastFlowTime) / 1000.0 else 0.0
            lastFlowTime = currentTime
            
            if (deltaTime > 0 && deltaTime < 0.5) {
                // חישוב תזוזה בהתאם לכיוון המצפן
                val azimuthRad = Math.toRadians(orientationState.value.azimuth.toDouble())
                
                // המרת תנועת Optical Flow לתנועה בעולם
                // שים לב: dy חיובי = רצפה זזה למטה = אנחנו זזים קדימה
                val forwardSpeed = flowResult.dy * velocityScale
                val lateralSpeed = flowResult.dx * velocityScale
                
                // סיבוב לפי המצפן
                val dx = forwardSpeed * sin(azimuthRad) + lateralSpeed * cos(azimuthRad)
                val dz = forwardSpeed * cos(azimuthRad) - lateralSpeed * sin(azimuthRad)
                
                virtualX += dx
                virtualZ += dz
                
                // הוספה להיסטוריה
                pathHistory.add(Pair(virtualX.toFloat(), virtualZ.toFloat()))
                if (pathHistory.size > 500) pathHistory.removeAt(0)
            }
        }
    }

    @SuppressLint("MissingPermission")
    fun requestInitialLocation() {
        val tokenSource = CancellationTokenSource()
        fusedLocationClient.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token)
            .addOnSuccessListener { location ->
                if (location != null) {
                    startLocation.value = LatLng(location.latitude, location.longitude)
                    Log.d(TAG, "Got initial location: ${location.latitude}, ${location.longitude}")
                }
            }
    }

    fun resetPath() {
        pathHistory.clear()
        virtualX = 0.0
        virtualZ = 0.0
        opticalFlowProcessor.reset()
        orientationTracker.reset()
    }

    /* ===================== UI COMPOSABLES ===================== */

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
                    requestInitialLocation()
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
        val orientation = orientationState.value
        val flowResult = flowResultState.value
        val azimuth = orientation.azimuth
        
        Box(Modifier.fillMaxSize().background(LuxuryBlack)) {
            Column(Modifier.fillMaxSize()) {
                
                // === CAMERA + AR AREA (Top) ===
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
                    // Camera View
                    CameraViewComposable()
                    
                    // AR Overlay
                    AROverlay(flowResult = flowResult, orientation = orientation)
                    
                    // Radar at top right
                    Box(Modifier.align(Alignment.TopEnd).padding(12.dp)) {
                        SensorRadar(
                            history = pathHistory.toList(),
                            currentAzimuth = azimuth
                        )
                    }
                    
                    // Direction indicator at top left
                    Box(Modifier.align(Alignment.TopStart).padding(12.dp)) {
                        DirectionBadge(direction = flowResult.direction, mode = flowResult.mode)
                    }
                    
                    // Phone orientation warning - show when NOT horizontal
                    if (!orientation.isHorizontal) {
                        Box(
                            Modifier
                                .align(Alignment.BottomCenter)
                                .padding(12.dp)
                        ) {
                            PhoneOrientationWarning(deviation = orientation.deviationFromHorizontal)
                        }
                    }
                    
                    // Stability indicator
                    Box(Modifier.align(Alignment.BottomEnd).padding(12.dp)) {
                        StabilityIndicator(
                            stability = orientation.stabilityScore,
                            confidence = flowResult.confidence
                        )
                    }
                }

                // === MAP AREA (Bottom) ===
                Box(
                    modifier = Modifier
                        .weight(0.45f)
                        .fillMaxWidth()
                        .padding(start = 7.dp, end = 7.dp, top = 7.dp, bottom = 50.dp)
                        .clip(RoundedCornerShape(20.dp))
                        .background(LuxuryDarkGrey)
                ) {
                    // מיקום ברירת מחדל אם אין GPS (תל אביב)
                    val mapStartLocation = startLocation.value ?: LatLng(32.0853, 34.7818)
                    
                    GoogleMapWrapper(
                        start = mapStartLocation,
                        azimuth = azimuth,
                        pathHistory = pathHistory.toList()
                    )
                    
                    // Reset button
                    FloatingActionButton(
                        onClick = { resetPath() },
                        containerColor = LuxuryGreen,
                        modifier = Modifier
                            .align(Alignment.BottomEnd)
                            .padding(16.dp)
                    ) {
                        Icon(Icons.Default.Refresh, "Reset", tint = LuxuryBlack)
                    }
                    
                    // Debug info
                    Box(
                        Modifier
                            .align(Alignment.TopStart)
                            .padding(8.dp)
                            .background(Color.Black.copy(alpha = 0.7f), RoundedCornerShape(8.dp))
                            .padding(8.dp)
                    ) {
                        Column {
                            Text(
                                "X: ${"%.1f".format(virtualX)}m  Z: ${"%.1f".format(virtualZ)}m",
                                color = LuxuryCyan,
                                fontSize = 12.sp
                            )
                            Text(
                                "Flow: ${"%.1f".format(flowResult.magnitude)}",
                                color = LuxuryGreen,
                                fontSize = 10.sp
                            )
                        }
                    }
                }
            }
        }
    }

    @Composable
    fun CameraViewComposable() {
        val lifecycle = LocalLifecycleOwner.current
        val mainActivity = this
        
        AndroidView(
            factory = { ctx ->
                CameraView(ctx).apply {
                    setLifecycleOwner(lifecycle)
                    setAudio(Audio.OFF)
                    setFrameProcessingFormat(android.graphics.ImageFormat.YUV_420_888)
                    
                    addFrameProcessor(object : FrameProcessor {
                        override fun process(frame: Frame) {
                            mainActivity.processCameraFrame(frame)
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
        // Grid effect when moving
        if (flowResult.direction != OpticalFlowProcessor.MovementDirection.STOPPED &&
            orientation.isHorizontal) {
            
            val infiniteTransition = rememberInfiniteTransition(label = "grid")
            val gridOffset by infiniteTransition.animateFloat(
                initialValue = 0f,
                targetValue = 60f,
                animationSpec = infiniteRepeatable(
                    animation = tween(800, easing = LinearEasing),
                    repeatMode = RepeatMode.Restart
                ),
                label = "gridOffset"
            )
            
            Canvas(Modifier.fillMaxSize()) {
                val gridSpacing = 80f
                
                // Flow lines from center going down
                val centerX = size.width / 2
                val startY = size.height * 0.4f
                
                // אפקט של קווים זורמים כלפי מטה
                for (i in 0..10) {
                    val y = startY + (gridOffset + i * 40f) % (size.height * 0.6f)
                    val alpha = ((y - startY) / (size.height * 0.6f) * 0.5f).coerceIn(0f, 0.5f)
                    
                    // קו מרכזי
                    drawLine(
                        color = LuxuryGreen.copy(alpha = alpha),
                        start = Offset(centerX, y),
                        end = Offset(centerX, y + 30f),
                        strokeWidth = 3f,
                        cap = StrokeCap.Round
                    )
                    
                    // קווים צדדיים
                    val spread = (y - startY) * 0.5f
                    drawLine(
                        color = LuxuryCyan.copy(alpha = alpha * 0.7f),
                        start = Offset(centerX - spread, y),
                        end = Offset(centerX - spread - 20f, y + 30f),
                        strokeWidth = 2f
                    )
                    drawLine(
                        color = LuxuryCyan.copy(alpha = alpha * 0.7f),
                        start = Offset(centerX + spread, y),
                        end = Offset(centerX + spread + 20f, y + 30f),
                        strokeWidth = 2f
                    )
                }
            }
        }
        
        // Direction Arrow Overlay
        Box(
            Modifier.fillMaxSize(),
            contentAlignment = Alignment.Center
        ) {
            AROverlayRenderer.DirectionArrow(
                direction = flowResult.direction,
                magnitude = flowResult.magnitude
            )
        }
    }

    @Composable
    fun DirectionBadge(direction: OpticalFlowProcessor.MovementDirection, mode: OpticalFlowProcessor.MovementMode = OpticalFlowProcessor.MovementMode.WALKING) {
        val (text, color, icon) = when (direction) {
            OpticalFlowProcessor.MovementDirection.FORWARD -> Triple("שמאלה", LuxuryGreen, Icons.Default.KeyboardArrowUp)
            OpticalFlowProcessor.MovementDirection.BACKWARD -> Triple("ימינה", LuxuryRed, Icons.Default.KeyboardArrowDown)
            OpticalFlowProcessor.MovementDirection.LEFT -> Triple("קדימה", LuxuryCyan, Icons.Default.ArrowBack)
            OpticalFlowProcessor.MovementDirection.RIGHT -> Triple("אחורה", LuxuryCyan, Icons.Default.ArrowForward)
            OpticalFlowProcessor.MovementDirection.STOPPED -> Triple("עומד", LuxuryYellow, Icons.Default.Place)
        }
        
        val modeText = when (mode) {
            OpticalFlowProcessor.MovementMode.WALKING -> "🚶"
            OpticalFlowProcessor.MovementMode.DRIVING -> "🚗"
        }
        
        Surface(
            color = Color.Black.copy(0.8f),
            shape = RoundedCornerShape(12.dp),
            border = BorderStroke(1.dp, color)
        ) {
            Row(
                modifier = Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(modeText, fontSize = 16.sp)
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
        val alpha by infiniteTransition.animateFloat(
            initialValue = 0.6f,
            targetValue = 1f,
            animationSpec = infiniteRepeatable(
                animation = tween(400),
                repeatMode = RepeatMode.Reverse
            ),
            label = "blink"
        )
        
        Surface(
            color = LuxuryRed.copy(alpha = alpha * 0.3f),
            shape = RoundedCornerShape(12.dp),
            border = BorderStroke(2.dp, LuxuryRed.copy(alpha = alpha))
        ) {
            Row(
                modifier = Modifier.padding(horizontal = 16.dp, vertical = 10.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    Icons.Default.Phone,
                    null,
                    tint = LuxuryRed,
                    modifier = Modifier
                        .size(24.dp)
                        .rotate(deviation.coerceIn(-30f, 30f))
                )
                Spacer(Modifier.width(8.dp))
                Column {
                    Text(
                        "החזק את הטלפון אופקית",
                        color = LuxuryRed,
                        fontWeight = FontWeight.Bold,
                        fontSize = 14.sp
                    )
                    Text(
                        "סטייה: ${"%.0f".format(deviation)}°",
                        color = LuxuryRed.copy(alpha = 0.8f),
                        fontSize = 12.sp
                    )
                }
            }
        }
    }

    @Composable
    fun StabilityIndicator(stability: Float, confidence: Float) {
        Surface(
            color = Color.Black.copy(0.7f),
            shape = RoundedCornerShape(8.dp)
        ) {
            Column(
                modifier = Modifier.padding(8.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                // Stability bar
                Box(
                    Modifier
                        .width(40.dp)
                        .height(4.dp)
                        .clip(RoundedCornerShape(2.dp))
                        .background(Color.White.copy(0.2f))
                ) {
                    Box(
                        Modifier
                            .fillMaxHeight()
                            .fillMaxWidth(stability)
                            .background(
                                when {
                                    stability > 0.7f -> LuxuryGreen
                                    stability > 0.4f -> LuxuryYellow
                                    else -> LuxuryRed
                                }
                            )
                    )
                }
                Spacer(Modifier.height(4.dp))
                Text(
                    "${"%.0f".format(confidence * 100)}%",
                    color = LuxuryTextGrey,
                    fontSize = 10.sp
                )
            }
        }
    }

    @Composable
    fun SensorRadar(history: List<Pair<Float, Float>>, currentAzimuth: Float) {
        Card(
            shape = RoundedCornerShape(12.dp),
            colors = CardDefaults.cardColors(containerColor = Color.Black.copy(0.85f)),
            border = BorderStroke(1.dp, LuxuryGreen),
            modifier = Modifier.size(110.dp)
        ) {
            Box(Modifier.fillMaxSize()) {
                Canvas(Modifier.fillMaxSize().padding(8.dp)) {
                    val cx = size.width / 2
                    val cy = size.height / 2
                    val radius = minOf(cx, cy)
                    
                    // Grid circles
                    for (i in 1..3) {
                        drawCircle(
                            color = LuxuryGreen.copy(0.15f),
                            radius = radius * i / 3,
                            style = Stroke(1f)
                        )
                    }
                    
                    // Cross lines
                    drawLine(LuxuryGreen.copy(0.2f), Offset(cx, 0f), Offset(cx, size.height))
                    drawLine(LuxuryGreen.copy(0.2f), Offset(0f, cy), Offset(size.width, cy))

                    // Draw path
                    if (history.isNotEmpty()) {
                        val path = Path()
                        val scale = 3f  // 3 pixels per meter
                        
                        val rad = Math.toRadians((-currentAzimuth).toDouble())
                        val cosA = cos(rad).toFloat()
                        val sinA = sin(rad).toFloat()

                        val currentX = history.last().first
                        val currentZ = history.last().second

                        fun transform(pX: Float, pZ: Float): Offset {
                            val dx = pX - currentX
                            val dz = pZ - currentZ
                            // Rotate to align with compass heading
                            val rotX = dx * cosA - dz * sinA
                            val rotZ = dx * sinA + dz * cosA
                            return Offset(
                                (cx + rotX * scale).coerceIn(0f, size.width),
                                (cy - rotZ * scale).coerceIn(0f, size.height)
                            )
                        }

                        val start = transform(history.first().first, history.first().second)
                        path.moveTo(start.x, start.y)
                        
                        for (i in 1 until history.size) {
                            val p = transform(history[i].first, history[i].second)
                            path.lineTo(p.x, p.y)
                        }
                        
                        drawPath(path, LuxuryGreen, style = Stroke(2f))
                        
                        // Start point
                        val startPoint = transform(history.first().first, history.first().second)
                        drawCircle(LuxuryYellow, 4f, startPoint)
                    }

                    // Current position (center)
                    drawCircle(Color.White, 5f, Offset(cx, cy))
                    
                    // Direction indicator
                    drawLine(
                        LuxuryCyan,
                        Offset(cx, cy),
                        Offset(cx, cy - 15f),
                        strokeWidth = 3f,
                        cap = StrokeCap.Round
                    )
                }
                
                // Label
                Text(
                    "RADAR",
                    color = LuxuryGreen.copy(0.6f),
                    fontSize = 8.sp,
                    modifier = Modifier
                        .align(Alignment.BottomCenter)
                        .padding(bottom = 4.dp)
                )
            }
        }
    }

    @Composable
    fun GoogleMapWrapper(
        start: LatLng,
        azimuth: Float,
        pathHistory: List<Pair<Float, Float>>
    ) {
        // Calculate current position based on virtual position
        val currentPos = remember(virtualX, virtualZ, start) {
            metersToLatLng(start, virtualX, virtualZ)
        }
        
        val cameraState = rememberCameraPositionState {
            position = CameraPosition.Builder()
                .target(currentPos)
                .zoom(18f)
                .bearing(azimuth)
                .tilt(0f)
                .build()
        }
        
        // Update camera to follow position
        LaunchedEffect(currentPos, azimuth) {
            cameraState.animate(
                com.google.android.gms.maps.CameraUpdateFactory.newCameraPosition(
                    CameraPosition.Builder()
                        .target(currentPos)
                        .zoom(18f)
                        .bearing(azimuth)
                        .tilt(30f)
                        .build()
                ),
                durationMs = 500
            )
        }
        
        GoogleMap(
            modifier = Modifier.fillMaxSize(),
            cameraPositionState = cameraState,
            uiSettings = MapUiSettings(
                zoomControlsEnabled = false,
                compassEnabled = false,
                myLocationButtonEnabled = false
            )
        ) {
            // Current position marker
            Marker(
                state = MarkerState(currentPos),
                title = "You",
                rotation = azimuth,
                flat = true,
                icon = BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_AZURE)
            )
            
            // Draw path on map if we have history
            if (pathHistory.isNotEmpty()) {
                val mapPath = pathHistory.map { (x, z) ->
                    metersToLatLng(start, x.toDouble(), z.toDouble())
                }
                
                Polyline(
                    points = mapPath,
                    color = LuxuryGreen,
                    width = 8f
                )
            }
            
            // Start point marker
            Marker(
                state = MarkerState(start),
                title = "Start",
                icon = BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN)
            )
        }
    }

    /**
     * המרת מטרים ל-LatLng
     */
    private fun metersToLatLng(start: LatLng, dx: Double, dz: Double): LatLng {
        val metersPerDegree = 111111.0
        val lat = start.latitude + (dz / metersPerDegree)
        val lng = start.longitude + (dx / (metersPerDegree * cos(Math.toRadians(start.latitude))))
        return LatLng(lat, lng)
    }
}