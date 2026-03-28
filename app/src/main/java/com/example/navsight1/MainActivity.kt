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

/* ===================== MAIN ACTIVITY ===================== */

class MainActivity : ComponentActivity(), SensorEventListener {

    companion object {
        private const val TAG = "NavSight"

        init {
            System.loadLibrary("navsight")
        }
    }

    // ── JNI declarations ──────────────────────────────────────────────────────
    external fun startVIO()
    external fun stopVIO()
    external fun processCameraFrame(
        frameData: ByteArray, width: Int, height: Int, timestamp: Long
    ): VioData
    external fun processGyroscope(timestamp: Long, x: Float, y: Float, z: Float)
    external fun processAccelerometer(timestamp: Long, x: Float, y: Float, z: Float)
    external fun resetVIO()
    external fun setScale(scale: Double)
    external fun setIntrinsics(fx: Double, fy: Double, cx: Double, cy: Double)

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
    // mutableStateOf makes writes from the camera thread visible to Compose on the UI thread
    private var virtualX by mutableStateOf(0.0)
    private var virtualZ by mutableStateOf(0.0)
    val pathHistory = mutableStateListOf<Pair<Float, Float>>()
    
    // GPS Start Location
    val startLocation = mutableStateOf<LatLng?>(null)

    // C++ VIO output state (updated on every camera frame)
    val vioState = mutableStateOf(VioData())

    // Kept so we can cancel the in-flight location request on destroy
    private var locationTokenSource: CancellationTokenSource? = null

    // Heading fusion (FR17): azimuth at the moment VIO first initializes
    private var vioInitAzimuth = 0f
    private var wasVioInitialized = false

    // Speed display (FR19)
    private var currentSpeedKmh by mutableStateOf(0f)
    private var lastVioForSpeed: VioData? = null
    private var lastSpeedTimeMs = 0L

    // Camera-blocked detection (FR26): count consecutive frames with no tracking
    private var consecutiveVioFailures = 0
    private var showCameraBlocked by mutableStateOf(false)

    // Shake detection for auto-reset (FR28)
    private val accelMagnitudeHistory = ArrayDeque<Float>(20)
    private var lastShakeCheckMs = 0L

    // Scale slider (FR15): user-adjustable translation scale, 0.1–5.0, default 1.0
    private var scaleValue by mutableStateOf(1.0f)

    // מהירות לפי Optical Flow
    private var lastFlowTime = 0L
    private val velocityScale = 0.01f // קנה מידה להמרת pixels למטרים

    // Orientation throttle: update at most 20Hz (50ms between updates)
    private var lastOrientationUpdateNs = 0L

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
        startVIO()
        // Pass 0s to use auto focal-length (0.7*width); replace with measured values for accuracy
        setIntrinsics(0.0, 0.0, 0.0, 0.0)
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
        stopVIO()
    }

    override fun onSensorChanged(event: SensorEvent) {
        val ts = event.timestamp  // nanoseconds from Android sensor
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                orientationTracker.updateAccelerometer(event.values)
                processAccelerometer(ts, event.values[0], event.values[1], event.values[2])

                // FR28: detect high IMU variance (shaking) and auto-reset VIO
                val mag = sqrt(
                    event.values[0] * event.values[0] +
                    event.values[1] * event.values[1] +
                    event.values[2] * event.values[2]
                )
                if (accelMagnitudeHistory.size >= 20) accelMagnitudeHistory.removeFirst()
                accelMagnitudeHistory.addLast(mag)
                val nowMs2 = System.currentTimeMillis()
                if (nowMs2 - lastShakeCheckMs > 500 && accelMagnitudeHistory.size >= 10) {
                    lastShakeCheckMs = nowMs2
                    val mean = accelMagnitudeHistory.average()
                    val variance = accelMagnitudeHistory.sumOf { v ->
                        (v - mean) * (v - mean)
                    } / accelMagnitudeHistory.size
                    if (variance > 30.0) {
                        lifecycleScope.launch(Dispatchers.Main) { resetPath() }
                    }
                }
            }
            Sensor.TYPE_MAGNETIC_FIELD -> {
                orientationTracker.updateMagnetometer(event.values)
            }
            Sensor.TYPE_GYROSCOPE -> {
                processGyroscope(ts, event.values[0], event.values[1], event.values[2])
            }
        }

        // עדכון אוריינטציה — throttled to 20Hz
        val nowNs = event.timestamp
        if (nowNs - lastOrientationUpdateNs >= 50_000_000L) {
            lastOrientationUpdateNs = nowNs
            orientationState.value = orientationTracker.getOrientation()
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    /**
     * עיבוד פריים מהמצלמה
     */
    fun processCameraFrame(frame: Frame) {
        val data = frame.getData<ByteArray>() ?: return

        // ── C++ VIO pipeline (primary) ────────────────────────────────────────
        val vio = processCameraFrame(
            frameData = data,
            width     = frame.size.width,
            height    = frame.size.height,
            timestamp = frame.time
        )

        // ── Kotlin Optical Flow (drives AR direction overlay only) ────────────
        val flowResult = opticalFlowProcessor.processFrame(
            frame  = data,
            width  = frame.size.width,
            height = frame.size.height,
            isYuv  = true
        )

        // Dispatch all UI state writes to the main thread (camera callback is on a bg thread)
        lifecycleScope.launch(Dispatchers.Main) {
            vioState.value = vio
            // Capture the compass azimuth at the moment VIO first initializes (for heading fusion)
            if (vio.isInitialized && !wasVioInitialized) {
                wasVioInitialized = true
                vioInitAzimuth = orientationState.value.azimuth
            }
            if (vio.isInitialized) {
                virtualX = vio.x
                virtualZ = vio.z
                pathHistory.add(Pair(vio.x.toFloat(), vio.z.toFloat()))
                if (pathHistory.size > 500) pathHistory.removeAt(0)
            }
            flowResultState.value = flowResult

            // FR19: Speed computation from consecutive VIO positions
            val nowMs = System.currentTimeMillis()
            val prev = lastVioForSpeed
            if (prev != null && vio.isInitialized) {
                val dtMs = nowMs - lastSpeedTimeMs
                if (dtMs >= 200) {
                    val dx = vio.x - prev.x
                    val dz = vio.z - prev.z
                    val distM = sqrt(dx * dx + dz * dz)
                    currentSpeedKmh = (distM / (dtMs / 1000.0) * 3.6).toFloat()
                    lastVioForSpeed = vio
                    lastSpeedTimeMs = nowMs
                }
            } else {
                lastVioForSpeed = vio
                lastSpeedTimeMs = nowMs
            }

            // FR26: Camera-blocked detection (30 consecutive frames with < 5 tracked points)
            if (vio.isInitialized && vio.trackedFeatures < 5) {
                consecutiveVioFailures++
                if (consecutiveVioFailures > 30) showCameraBlocked = true
            } else {
                consecutiveVioFailures = 0
                showCameraBlocked = false
            }
        }
    }

    @SuppressLint("MissingPermission")
    fun requestInitialLocation() {
        locationTokenSource?.cancel()
        val tokenSource = CancellationTokenSource()
        locationTokenSource = tokenSource
        fusedLocationClient.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, tokenSource.token)
            .addOnSuccessListener { location ->
                if (location != null) {
                    startLocation.value = LatLng(location.latitude, location.longitude)
                    Log.d(TAG, "Got initial location: ${location.latitude}, ${location.longitude}")
                }
                locationTokenSource = null
            }
    }

    override fun onDestroy() {
        super.onDestroy()
        locationTokenSource?.cancel()
        locationTokenSource = null
    }

    // FR21: Export tracked path as JSON to the app's external files directory
    fun exportPath() {
        val path = pathHistory.toList()
        if (path.isEmpty()) return
        val start = startLocation.value
        val sb = StringBuilder()
        sb.append("{\"type\":\"navsight_path\",\"points\":[")
        path.forEachIndexed { idx, (x, z) ->
            if (idx > 0) sb.append(",")
            if (start != null) {
                // Convert to lat/lng if we have a GPS anchor
                val lat = start.latitude + z / 111111.0
                val cosLat = cos(Math.toRadians(start.latitude))
                val lng = start.longitude + if (cosLat > 1e-10) x / (111111.0 * cosLat) else 0.0
                sb.append("{\"lat\":$lat,\"lng\":$lng,\"x\":$x,\"z\":$z}")
            } else {
                sb.append("{\"x\":$x,\"z\":$z}")
            }
        }
        sb.append("]}")
        try {
            val dir = getExternalFilesDir(null) ?: filesDir
            val file = java.io.File(dir, "navsight_path_${System.currentTimeMillis()}.json")
            file.writeText(sb.toString())
            Log.d(TAG, "Path exported to ${file.absolutePath}")
        } catch (e: Exception) {
            Log.e(TAG, "Export failed: ${e.message}")
        }
    }

    fun resetPath() {
        resetVIO()
        pathHistory.clear()
        virtualX = 0.0
        virtualZ = 0.0
        vioState.value = VioData()
        opticalFlowProcessor.reset()
        orientationTracker.reset()
        wasVioInitialized = false
        vioInitAzimuth = 0f
        currentSpeedKmh = 0f
        lastVioForSpeed = null
        consecutiveVioFailures = 0
        showCameraBlocked = false
        accelMagnitudeHistory.clear()
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
        val vio = vioState.value
        val azimuth = orientation.azimuth

        // FR17: Fuse VIO yaw (80%) with magnetometer azimuth (20%) once VIO is initialized.
        // VIO yaw is cumulative radians CCW+; compass is degrees CW+.
        // We compute an absolute VIO heading by adding the yaw delta to the azimuth captured
        // at initialization, then blend the two.
        val fusedHeading: Float = if (vio.isInitialized && wasVioInitialized) {
            val vioYawDeg = Math.toDegrees(-vio.yaw).toFloat()   // negate: CCW+ → CW+
            val vioAbsDeg = ((vioInitAzimuth + vioYawDeg) % 360 + 360) % 360
            // Circular weighted blend (handles the 0°/360° wraparound)
            val diff = ((azimuth - vioAbsDeg + 540) % 360) - 180
            ((vioAbsDeg + 0.2f * diff) % 360 + 360) % 360
        } else {
            azimuth
        }

        // FR20: Total distance traveled (sum of consecutive pathHistory segment lengths)
        val totalDistanceM = remember(pathHistory.size) {
            val h = pathHistory.toList()
            if (h.size < 2) 0.0
            else h.zipWithNext().sumOf { (a, b) ->
                val dx = (b.first - a.first).toDouble()
                val dz = (b.second - a.second).toDouble()
                sqrt(dx * dx + dz * dz)
            }
        }
        
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
                            currentAzimuth = fusedHeading
                        )
                    }
                    
                    // Direction indicator at top left (FR23: shows LOW QUALITY when quality < 0.3)
                    Box(Modifier.align(Alignment.TopStart).padding(12.dp)) {
                        DirectionBadge(
                            direction = flowResult.direction,
                            mode = flowResult.mode,
                            trackingQuality = vio.trackingQuality.toFloat()
                        )
                    }

                    // FR24: Initialization progress — shown until VIO is initialized
                    if (!vio.isInitialized) {
                        Box(Modifier.align(Alignment.Center).padding(8.dp)) {
                            VioInitializingBadge()
                        }
                    }

                    // FR26: Camera blocked warning
                    if (showCameraBlocked) {
                        Box(Modifier.align(Alignment.Center).padding(8.dp)) {
                            CameraBlockedWarning()
                        }
                    }

                    // FR29: No-texture warning (initialized but very few features)
                    if (vio.isInitialized && !showCameraBlocked && vio.trackedFeatures < 30) {
                        Box(Modifier.align(Alignment.BottomStart).padding(12.dp)) {
                            NoTextureWarning()
                        }
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
                        azimuth = fusedHeading,
                        pathHistory = pathHistory.toList()
                    )
                    
                    // Buttons: Export (FR21) and Reset
                    Column(
                        Modifier
                            .align(Alignment.BottomEnd)
                            .padding(16.dp),
                        verticalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        SmallFloatingActionButton(
                            onClick = { exportPath() },
                            containerColor = LuxuryCyan
                        ) {
                            Icon(Icons.Default.ArrowForward, "Export", tint = LuxuryBlack)
                        }
                        FloatingActionButton(
                            onClick = { resetPath() },
                            containerColor = LuxuryGreen
                        ) {
                            Icon(Icons.Default.Refresh, "Reset", tint = LuxuryBlack)
                        }
                    }
                    
                    // FR19/FR20: Speed and distance display
                    Box(
                        Modifier
                            .align(Alignment.TopEnd)
                            .padding(8.dp)
                            .background(Color.Black.copy(alpha = 0.7f), RoundedCornerShape(8.dp))
                            .padding(horizontal = 10.dp, vertical = 6.dp)
                    ) {
                        Column(horizontalAlignment = Alignment.End) {
                            Text(
                                "${"%.1f".format(currentSpeedKmh)} km/h",
                                color = LuxuryCyan,
                                fontWeight = FontWeight.Bold,
                                fontSize = 16.sp
                            )
                            Text(
                                "${"%.0f".format(totalDistanceM)} m",
                                color = LuxuryGreen,
                                fontSize = 12.sp
                            )
                        }
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
                                "X: ${"%.1f".format(vio.x)}m  Z: ${"%.1f".format(vio.z)}m",
                                color = LuxuryCyan,
                                fontSize = 12.sp
                            )
                            Text(
                                "Yaw: ${"%.1f".format(Math.toDegrees(vio.yaw))}°  " +
                                "Q: ${"%.0f".format(vio.trackingQuality * 100)}%",
                                color = LuxuryGreen,
                                fontSize = 10.sp
                            )
                            Text(
                                "Pts: ${vio.trackedFeatures}/${vio.totalFeatures}  " +
                                if (vio.isInitialized) "VIO ✓" else "Initializing…",
                                color = if (vio.isInitialized) LuxuryGreen else LuxuryYellow,
                                fontSize = 10.sp
                            )
                        }
                    }

                    // FR15: Scale slider — adjusts VIO translation scale (0.1–5.0)
                    Row(
                        Modifier
                            .align(Alignment.BottomStart)
                            .fillMaxWidth()
                            .padding(horizontal = 12.dp, vertical = 4.dp)
                            .background(Color.Black.copy(alpha = 0.6f), RoundedCornerShape(8.dp))
                            .padding(horizontal = 10.dp, vertical = 2.dp),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            "Scale: ${"%.1f".format(scaleValue)}×",
                            color = LuxuryCyan,
                            fontSize = 11.sp,
                            modifier = Modifier.width(72.dp)
                        )
                        Slider(
                            value = scaleValue,
                            onValueChange = { v ->
                                scaleValue = v
                                setScale(v.toDouble())
                            },
                            valueRange = 0.1f..5.0f,
                            modifier = Modifier.weight(1f),
                            colors = SliderDefaults.colors(
                                thumbColor = LuxuryCyan,
                                activeTrackColor = LuxuryCyan
                            )
                        )
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
    fun DirectionBadge(
        direction: OpticalFlowProcessor.MovementDirection,
        mode: OpticalFlowProcessor.MovementMode = OpticalFlowProcessor.MovementMode.WALKING,
        trackingQuality: Float = 1f
    ) {
        // FR23/UX-DR1: Replace direction with LOW QUALITY warning when tracking quality < 0.3
        val lowQuality = trackingQuality < 0.3f && trackingQuality > 0f

        val (text, color, icon) = if (lowQuality) {
            Triple("LOW QUALITY", LuxuryRed, Icons.Default.Place)
        } else when (direction) {
            OpticalFlowProcessor.MovementDirection.FORWARD -> Triple("קדימה", LuxuryGreen, Icons.Default.KeyboardArrowUp)
            OpticalFlowProcessor.MovementDirection.BACKWARD -> Triple("אחורה", LuxuryRed, Icons.Default.KeyboardArrowDown)
            OpticalFlowProcessor.MovementDirection.LEFT -> Triple("שמאלה", LuxuryCyan, Icons.Default.ArrowBack)
            OpticalFlowProcessor.MovementDirection.RIGHT -> Triple("ימינה", LuxuryCyan, Icons.Default.ArrowForward)
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

    // FR24/UX-DR2: Shown while VIO is collecting gravity samples before initialization
    @Composable
    fun VioInitializingBadge() {
        val infiniteTransition = rememberInfiniteTransition(label = "init_pulse")
        val alpha by infiniteTransition.animateFloat(
            initialValue = 0.5f, targetValue = 1f,
            animationSpec = infiniteRepeatable(tween(600), RepeatMode.Reverse),
            label = "init_alpha"
        )
        Surface(
            color = Color.Black.copy(0.85f),
            shape = RoundedCornerShape(12.dp),
            border = BorderStroke(1.dp, LuxuryYellow.copy(alpha = alpha))
        ) {
            Row(
                Modifier.padding(horizontal = 14.dp, vertical = 10.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(Icons.Default.Refresh, null, tint = LuxuryYellow, modifier = Modifier.size(18.dp))
                Spacer(Modifier.width(8.dp))
                Column {
                    Text("מכייל…", color = LuxuryYellow, fontWeight = FontWeight.Bold, fontSize = 13.sp)
                    Text("החזק את הטלפון שטוח", color = LuxuryYellow.copy(0.8f), fontSize = 11.sp)
                }
            }
        }
    }

    // FR26: Shown when VIO has produced no features for 30+ consecutive frames
    @Composable
    fun CameraBlockedWarning() {
        val infiniteTransition = rememberInfiniteTransition(label = "blocked_blink")
        val alpha by infiniteTransition.animateFloat(
            initialValue = 0.6f, targetValue = 1f,
            animationSpec = infiniteRepeatable(tween(400), RepeatMode.Reverse),
            label = "blocked_alpha"
        )
        Surface(
            color = LuxuryRed.copy(0.2f),
            shape = RoundedCornerShape(12.dp),
            border = BorderStroke(2.dp, LuxuryRed.copy(alpha = alpha))
        ) {
            Row(
                Modifier.padding(horizontal = 14.dp, vertical = 10.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(Icons.Default.Place, null, tint = LuxuryRed, modifier = Modifier.size(20.dp))
                Spacer(Modifier.width(8.dp))
                Text("מצלמה חסומה", color = LuxuryRed, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            }
        }
    }

    // FR29: Shown when active but too few texture features are found
    @Composable
    fun NoTextureWarning() {
        Surface(
            color = Color.Black.copy(0.75f),
            shape = RoundedCornerShape(10.dp),
            border = BorderStroke(1.dp, LuxuryYellow)
        ) {
            Text(
                "אין מרקם — עבור לאזור מרוצף",
                color = LuxuryYellow,
                fontSize = 11.sp,
                modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp)
            )
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
                        // FR22/UX-DR3: Auto-scale to fit the entire path bounding box inside the radar circle
                        val scale = if (history.size > 1) {
                            val minX = history.minOf { it.first }
                            val maxX = history.maxOf { it.first }
                            val minZ = history.minOf { it.second }
                            val maxZ = history.maxOf { it.second }
                            val span = maxOf(maxX - minX, maxZ - minZ, 1f)
                            (radius * 0.8f) / (span / 2f)
                        } else 3f
                        
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
        
        // Update camera to follow position — debounced to max 2 updates/sec
        LaunchedEffect(currentPos, azimuth) {
            delay(500)
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
        val cosLat = cos(Math.toRadians(start.latitude))
        val lngOffset = if (cosLat > 1e-10) dx / (metersPerDegree * cosLat) else 0.0
        return LatLng(lat, start.longitude + lngOffset)
    }
}