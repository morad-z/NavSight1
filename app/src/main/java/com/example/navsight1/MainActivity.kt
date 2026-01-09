package com.example.navsight1

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.net.Uri
import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.unit.dp
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.viewinterop.AndroidView
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.rememberMultiplePermissionsState
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.google.android.gms.maps.model.CameraPosition
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.tasks.CancellationTokenSource
import com.google.maps.android.compose.*
import androidx.compose.runtime.LaunchedEffect
import com.google.android.gms.maps.CameraUpdateFactory
import com.otaliastudios.cameraview.CameraListener
import com.otaliastudios.cameraview.CameraOptions
import com.otaliastudios.cameraview.CameraView
import org.opencv.android.OpenCVLoader
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.coroutines.launch
import androidx.compose.runtime.rememberCoroutineScope
import androidx.lifecycle.lifecycleScope
import com.google.android.libraries.places.api.Places
import com.google.android.libraries.places.api.model.Place
import com.google.android.libraries.places.api.net.FindAutocompletePredictionsRequest
import com.google.android.libraries.places.api.net.FetchPlaceRequest
import com.google.android.libraries.places.api.net.PlacesClient
import org.json.JSONObject
import java.net.URL
import java.net.URLEncoder

class MainActivity : ComponentActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private var fusedLocationClient: FusedLocationProviderClient? = null
    private var placesClient: PlacesClient? = null
    internal var isNativeLibraryLoaded = false
    private var isVioStarted = false

    // State to hold the latest VIO data
    val vioDataState = mutableStateOf(VioData())
    val startLocation = mutableStateOf<LatLng?>(null)
    val destinationLocation = mutableStateOf<LatLng?>(null)
    val routePoints = mutableStateOf<List<LatLng>>(emptyList())
    val isNavigating = mutableStateOf(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Set up global exception handler
        val defaultExceptionHandler = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
            Log.e(TAG, "UNCAUGHT EXCEPTION in thread ${thread.name}", throwable)
            Log.e(TAG, "Stack trace: ${throwable.stackTraceToString()}")
            // Let the system handle it after logging
            defaultExceptionHandler?.uncaughtException(thread, throwable)
        }

        Log.d(TAG, "onCreate started")

        try {
            enableEdgeToEdge()
            Log.d(TAG, "Edge-to-edge enabled")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to enable edge-to-edge: ${e.message}", e)
        }

        // Native library loaded - ready to use

        try {
            fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)
            Log.d(TAG, "Location client initialized")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize location client: ${e.message}", e)
        }

        try {
            setContent {
                NavSightApp(
                    vioDataState.value,
                    startLocation.value,
                    destinationLocation.value,
                    routePoints.value,
                    isNavigating.value
                )
            }
            Log.d(TAG, "Content set successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to set content: ${e.message}", e)
            throw e // Re-throw to prevent app from continuing in bad state
        }

        try {
            sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
            accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
            gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
            Log.d(TAG, "Sensors initialized - Accel: ${accelerometer != null}, Gyro: ${gyroscope != null}")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize sensors: ${e.message}", e)
        }

        // Initialize Google Places API
        try {
            val apiKey = packageManager
                .getApplicationInfo(packageName, android.content.pm.PackageManager.GET_META_DATA)
                .metaData
                .getString("com.google.android.geo.API_KEY") ?: ""

            if (!Places.isInitialized()) {
                Places.initialize(applicationContext, apiKey)
            }
            placesClient = Places.createClient(this)
            Log.d(TAG, "Places API initialized")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize Places API: ${e.message}", e)
        }

        Log.d(TAG, "onCreate completed successfully")
    }

    @SuppressLint("MissingPermission")
    fun requestInitialLocation() {
        fusedLocationClient?.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, CancellationTokenSource().token)
            ?.addOnSuccessListener { location ->
                if (location != null) {
                    startLocation.value = LatLng(location.latitude, location.longitude)
                    resetVIO() // Reset VIO origin to this new location
                }
            } ?: run {
            Log.e(TAG, "Location client not initialized")
        }
    }

    override fun onResume() {
        super.onResume()
        accelerometer?.also { accel ->
            sensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST)
        }
        gyroscope?.also { gyro ->
            sensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST)
        }

        // Only start VIO if native library is loaded and not already started
        if (isNativeLibraryLoaded && !isVioStarted) {
            try {
                startVIO()
                isVioStarted = true
                Log.d(TAG, "VIO started successfully")
            } catch (e: Exception) {
                Log.e(TAG, "Failed to start VIO: ${e.message}", e)
            }
        }
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)

        if (isNativeLibraryLoaded && isVioStarted) {
            try {
                stopVIO()
                isVioStarted = false
                Log.d(TAG, "VIO stopped successfully")
            } catch (e: Exception) {
                Log.e(TAG, "Failed to stop VIO: ${e.message}", e)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        // Cleanup if needed
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Do nothing
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event != null && isNativeLibraryLoaded) {
            try {
                when (event.sensor.type) {
                    Sensor.TYPE_ACCELEROMETER -> {
                        processAccelerometer(event.timestamp, event.values[0], event.values[1], event.values[2])
                    }
                    Sensor.TYPE_GYROSCOPE -> {
                        processGyroscope(event.timestamp, event.values[0], event.values[1], event.values[2])
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error processing sensor data: ${e.message}")
            }
        }
    }

    suspend fun fetchRoute(origin: LatLng, destination: LatLng): List<LatLng> {
        return withContext(Dispatchers.IO) {
            try {
                val apiKey = packageManager
                    .getApplicationInfo(packageName, android.content.pm.PackageManager.GET_META_DATA)
                    .metaData
                    .getString("com.google.android.geo.API_KEY") ?: ""

                val url = "https://maps.googleapis.com/maps/api/directions/json?" +
                        "origin=${origin.latitude},${origin.longitude}" +
                        "&destination=${destination.latitude},${destination.longitude}" +
                        "&mode=driving" +
                        "&key=$apiKey"

                val response = URL(url).readText()
                val json = JSONObject(response)

                val routes = json.getJSONArray("routes")
                if (routes.length() > 0) {
                    val route = routes.getJSONObject(0)
                    val overviewPolyline = route.getJSONObject("overview_polyline")
                    val points = overviewPolyline.getString("points")
                    decodePolyline(points)
                } else {
                    emptyList()
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error fetching route: ${e.message}", e)
                emptyList()
            }
        }
    }

    private fun decodePolyline(encoded: String): List<LatLng> {
        val poly = ArrayList<LatLng>()
        var index = 0
        val len = encoded.length
        var lat = 0
        var lng = 0

        while (index < len) {
            var b: Int
            var shift = 0
            var result = 0
            do {
                if (index >= len) {
                    Log.e(TAG, "Malformed polyline encoding at index $index")
                    break
                }
                b = encoded[index++].code - 63
                result = result or (b and 0x1f shl shift)
                shift += 5
            } while (b >= 0x20 && index < len)
            val dlat = if (result and 1 != 0) (result shr 1).inv() else result shr 1
            lat += dlat

            shift = 0
            result = 0
            do {
                if (index >= len) {
                    Log.e(TAG, "Malformed polyline encoding at index $index")
                    break
                }
                b = encoded[index++].code - 63
                result = result or (b and 0x1f shl shift)
                shift += 5
            } while (b >= 0x20 && index < len)
            val dlng = if (result and 1 != 0) (result shr 1).inv() else result shr 1
            lng += dlng

            val p = LatLng(lat.toDouble() / 1E5, lng.toDouble() / 1E5)
            poly.add(p)
        }

        return poly
    }

    fun startNavigation(destination: LatLng) {
        lifecycleScope.launch {
            val origin = startLocation.value
            if (origin != null) {
                val route = fetchRoute(origin, destination)
                routePoints.value = route
                isNavigating.value = route.isNotEmpty()
                destinationLocation.value = destination
                if (route.isNotEmpty()) {
                    Toast.makeText(this@MainActivity, "Route loaded!", Toast.LENGTH_SHORT).show()
                } else {
                    Toast.makeText(this@MainActivity, "Failed to load route", Toast.LENGTH_SHORT).show()
                }
            }
        }
    }

    fun updateNavigationRoute() {
        lifecycleScope.launch {
            val destination = destinationLocation.value
            val origin = startLocation.value
            if (destination != null && origin != null && isNavigating.value) {
                // Get current VIO position in LatLng
                val currentVioPos = metersToLatLng(origin, vioDataState.value.x, vioDataState.value.z)
                // Re-fetch route from current position to destination
                val route = fetchRoute(currentVioPos, destination)
                if (route.isNotEmpty()) {
                    routePoints.value = route
                }
            }
        }
    }

    fun stopNavigation() {
        routePoints.value = emptyList()
        destinationLocation.value = null
        isNavigating.value = false
        Toast.makeText(this, "Navigation stopped", Toast.LENGTH_SHORT).show()
    }

    suspend fun searchPlace(query: String): LatLng? {
        return withContext(Dispatchers.IO) {
            try {
                val apiKey = packageManager
                    .getApplicationInfo(packageName, android.content.pm.PackageManager.GET_META_DATA)
                    .metaData
                    .getString("com.google.android.geo.API_KEY") ?: ""

                val encodedQuery = URLEncoder.encode(query, "UTF-8")
                val url = "https://maps.googleapis.com/maps/api/geocode/json?address=$encodedQuery&key=$apiKey"

                val response = URL(url).readText()
                val json = JSONObject(response)

                val results = json.getJSONArray("results")
                if (results.length() > 0) {
                    val location = results.getJSONObject(0)
                        .getJSONObject("geometry")
                        .getJSONObject("location")

                    val lat = location.getDouble("lat")
                    val lng = location.getDouble("lng")

                    Log.d(TAG, "Found place: $query at $lat, $lng")
                    LatLng(lat, lng)
                } else {
                    Log.w(TAG, "No results found for: $query")
                    null
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error searching place: ${e.message}", e)
                null
            }
        }
    }

    external fun processCameraFrame(
        frameData: ByteArray,
        width: Int, height: Int, timestamp: Long
    ): VioData?
    external fun processAccelerometer(timestamp: Long, x: Float, y: Float, z: Float)
    external fun processGyroscope(timestamp: Long, x: Float, y: Float, z: Float)
    external fun resetVIO()
    external fun setScale(scale: Double)
    external fun startVIO()
    external fun stopVIO()
    external fun pingNative()


    companion object {
        const val TAG = "NavSight"
        internal const val METERS_PER_DEGREE = 111139.0  // Meters per degree latitude (standard Earth)
        private var isOpenCVInitialized = false
        private var isLibraryLoaded = false

        init {
            Log.d(TAG, "Companion object init block starting...")
            try {
                Log.d(TAG, "Attempting to initialize OpenCV...")
                isOpenCVInitialized = if (BuildConfig.DEBUG) {
                    Log.d(TAG, "Initializing OpenCV in DEBUG mode")
                    OpenCVLoader.initDebug()
                } else {
                    Log.d(TAG, "Initializing OpenCV in RELEASE mode")
                    OpenCVLoader.initDebug()
                }
                if (isOpenCVInitialized) {
                    Log.d(TAG, "OpenCV is initialized successfully.")
                } else {
                    Log.e(TAG, "OpenCV initialization failed!")
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error initializing OpenCV: ${e.message}", e)
                isOpenCVInitialized = false
            } catch (e: Error) {
                Log.e(TAG, "Critical error initializing OpenCV: ${e.message}", e)
                isOpenCVInitialized = false
            }

            try {
                Log.d(TAG, "Attempting to load native library 'navsight'...")
                System.loadLibrary("navsight")
                isLibraryLoaded = true
                Log.d(TAG, "Native library loaded successfully.")
            } catch (e: UnsatisfiedLinkError) {
                Log.e(TAG, "Failed to load native library: ${e.message}", e)
                isLibraryLoaded = false
            } catch (e: Exception) {
                Log.e(TAG, "Unexpected error loading native library: ${e.message}", e)
                isLibraryLoaded = false
            }

            Log.d(TAG, "Companion init complete - OpenCV: $isOpenCVInitialized, Library: $isLibraryLoaded")
        }
    }

    init {
        try {
            isNativeLibraryLoaded = isLibraryLoaded && isOpenCVInitialized
            Log.d(TAG, "Instance init - Native library loaded: $isNativeLibraryLoaded")
        } catch (e: Exception) {
            Log.e(TAG, "Error in instance init: ${e.message}", e)
            isNativeLibraryLoaded = false
        }
    }
}

@OptIn(ExperimentalPermissionsApi::class)
@Composable
fun NavSightApp(
    vioData: VioData,
    startLocation: LatLng?,
    destinationLocation: LatLng?,
    routePoints: List<LatLng>,
    isNavigating: Boolean
) {
    val context = LocalContext.current
    val mainActivity = context as MainActivity
    var isLibraryReady by remember { mutableStateOf(mainActivity.isNativeLibraryLoaded) }
    var retryCount by remember { mutableStateOf(0) }
    val maxRetries = 10

    // Check library loading status with retry mechanism
    LaunchedEffect(Unit) {
        while (!isLibraryReady && retryCount < maxRetries) {
            kotlinx.coroutines.delay(500) // Wait 500ms between checks
            isLibraryReady = mainActivity.isNativeLibraryLoaded
            if (!isLibraryReady) {
                retryCount++
                Log.d(MainActivity.TAG, "Waiting for native library... Attempt ${retryCount}/${maxRetries}")
            }
        }
        if (isLibraryReady) {
            Log.d(MainActivity.TAG, "Native library loaded successfully!")
        } else {
            Log.e(MainActivity.TAG, "Failed to load native library after $maxRetries attempts")
        }
    }

    if (!isLibraryReady) {
        // Loading screen
        Column(
            modifier = Modifier.fillMaxSize(),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            androidx.compose.material3.CircularProgressIndicator(
                modifier = Modifier.padding(16.dp)
            )
            Text(
                text = if (retryCount < maxRetries) {
                    "Initializing NavSight...\nLoading native libraries..."
                } else {
                    "Failed to load native libraries.\nPlease restart the app."
                },
                modifier = Modifier.padding(16.dp),
                textAlign = TextAlign.Center
            )
            if (retryCount >= maxRetries) {
                Button(
                    onClick = {
                        // Restart activity
                        (context as ComponentActivity).recreate()
                    },
                    modifier = Modifier.padding(16.dp)
                ) {
                    Text("Retry")
                }
            }
        }
    } else {
        // Library loaded, proceed with normal flow
        val permissionsState = rememberMultiplePermissionsState(
            permissions = listOf(
                Manifest.permission.CAMERA,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        )

        LaunchedEffect(permissionsState.allPermissionsGranted) {
            if (permissionsState.allPermissionsGranted) {
                mainActivity.requestInitialLocation()
            }
        }

        if (permissionsState.allPermissionsGranted) {
            MainScreen(context, vioData, startLocation, destinationLocation, routePoints, isNavigating)
        } else {
            Column(
                modifier = Modifier.fillMaxSize(),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center
            ) {
                Text(
                    text = "Camera and Location permissions are required to use this app.",
                    modifier = Modifier.padding(16.dp),
                    textAlign = TextAlign.Center
                )
                Button(
                    onClick = { permissionsState.launchMultiplePermissionRequest() },
                    modifier = Modifier.padding(16.dp)
                ) {
                    Text("Grant Permissions")
                }
            }
        }
    }
}

@Composable
fun TrackedPointsOverlay(points: FloatArray) {
    Canvas(modifier = Modifier.fillMaxSize()) {
        val pointCount = points.size / 2
        for (i in 0 until pointCount) {
            val x = points[i * 2]
            val y = points[i * 2 + 1]
            drawCircle(
                color = Color.Green,
                radius = 8f,
                center = Offset(x, y),
                style = Stroke(width = 3f)
            )
        }
    }
}

@Composable
fun DebugOverlay(vioData: VioData) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp),
        verticalArrangement = Arrangement.Top,
        horizontalAlignment = Alignment.Start
    ) {
        // Status card at top
        Card(
            colors = CardDefaults.cardColors(
                containerColor = Color.Black.copy(alpha = 0.7f)
            ),
            modifier = Modifier.padding(bottom = 8.dp)
        ) {
            Column(modifier = Modifier.padding(12.dp)) {
                // Initialization status
                Text(
                    text = if (vioData.isInitialized) "✅ Initialized" else "⏳ Initializing...",
                    color = if (vioData.isInitialized) Color.Green else Color.Yellow,
                    style = MaterialTheme.typography.labelLarge
                )

                Spacer(modifier = Modifier.height(8.dp))

                // Position
                Text(
                    text = "Position (m):",
                    color = Color.White,
                    style = MaterialTheme.typography.labelMedium
                )
                Text(
                    text = "X: ${"%.2f".format(vioData.x)}  Y: ${"%.2f".format(vioData.y)}  Z: ${"%.2f".format(vioData.z)}",
                    color = Color.Cyan,
                    style = MaterialTheme.typography.bodySmall
                )

                Spacer(modifier = Modifier.height(4.dp))

                // Rotation
                Text(
                    text = "Rotation (deg):",
                    color = Color.White,
                    style = MaterialTheme.typography.labelMedium
                )
                Text(
                    text = "R: ${"%.1f".format(Math.toDegrees(vioData.roll))}  P: ${"%.1f".format(Math.toDegrees(vioData.pitch))}  Y: ${"%.1f".format(Math.toDegrees(vioData.yaw))}",
                    color = Color.Cyan,
                    style = MaterialTheme.typography.bodySmall
                )

                Spacer(modifier = Modifier.height(4.dp))

                // Tracking quality
                val qualityColor = when {
                    vioData.trackingQuality > 0.7 -> Color.Green
                    vioData.trackingQuality > 0.3 -> Color.Yellow
                    else -> Color.Red
                }
                Text(
                    text = "Tracking Quality: ${"%.1f%%".format(vioData.trackingQuality * 100)}",
                    color = qualityColor,
                    style = MaterialTheme.typography.labelMedium
                )

                // Features
                Text(
                    text = "Features: ${vioData.trackedFeatures} / ${vioData.totalFeatures}",
                    color = Color.White,
                    style = MaterialTheme.typography.bodySmall
                )

                // Scale
                Text(
                    text = "Scale: ${"%.3f".format(vioData.estimatedScale)} m/unit",
                    color = if (vioData.estimatedScale > 0.01) Color.Green else Color.Gray,
                    style = MaterialTheme.typography.bodySmall
                )

                // Point count
                val pointCount = vioData.trackedPoints.size / 2
                Text(
                    text = "Tracked Points: $pointCount 🟢",
                    color = Color.White,
                    style = MaterialTheme.typography.bodySmall
                )

                Spacer(modifier = Modifier.height(8.dp))

                // IMU Data section
                Text(
                    text = "IMU Sensors:",
                    color = Color.Yellow,
                    style = MaterialTheme.typography.labelMedium
                )

                // Accelerometer
                Text(
                    text = "Accel (m/s²): X:${"%.2f".format(vioData.accelX)} Y:${"%.2f".format(vioData.accelY)} Z:${"%.2f".format(vioData.accelZ)}",
                    color = Color.Cyan,
                    style = MaterialTheme.typography.bodySmall
                )

                // Gyroscope
                Text(
                    text = "Gyro (rad/s): X:${"%.2f".format(vioData.gyroX)} Y:${"%.2f".format(vioData.gyroY)} Z:${"%.2f".format(vioData.gyroZ)}",
                    color = Color.Cyan,
                    style = MaterialTheme.typography.bodySmall
                )
            }
        }
    }
}

@Composable
fun MainScreen(
    context: Context,
    vioData: VioData,
    startLocation: LatLng?,
    destinationLocation: LatLng?,
    routePoints: List<LatLng>,
    isNavigating: Boolean
) {
    var showNavigationDialog by remember { mutableStateOf(false) }
    val mainActivity = context as MainActivity

    Box(modifier = Modifier.fillMaxSize()) {
        Column(modifier = Modifier.fillMaxSize()) {
            Box(modifier = Modifier.fillMaxWidth().weight(1f)) {
                CameraView(
                    modifier = Modifier.fillMaxSize(),
                    context = context
                )
                TrackedPointsOverlay(points = vioData.trackedPoints)
                DebugOverlay(vioData = vioData)
            }
            if (startLocation != null) {
                MapView(
                    modifier = Modifier
                        .fillMaxWidth()
                        .weight(1f),
                    startLocation = startLocation,
                    vioData = vioData,
                    destinationLocation = destinationLocation,
                    routePoints = routePoints
                )
            } else {
                Box(modifier = Modifier.fillMaxWidth().weight(1f), contentAlignment = Alignment.Center) {
                    Text("Getting initial location...")
                }
            }
        }
        // Top center: Navigation info or VIO position
        if (isNavigating && destinationLocation != null && startLocation != null) {
            // Navigation mode: Show distance to destination
            val currentPos = remember(vioData, startLocation) {
                metersToLatLng(startLocation!!, vioData.x, vioData.z)
            }
            val distance = remember(currentPos, destinationLocation) {
                calculateDistance(currentPos, destinationLocation!!)
            }
            Column(
                modifier = Modifier
                    .align(Alignment.TopCenter)
                    .padding(16.dp)
                    .background(Color(0xFF1E88E5).copy(alpha = 0.9f))
                    .padding(horizontal = 16.dp, vertical = 12.dp),
                horizontalAlignment = Alignment.CenterHorizontally
            ) {
                Text(
                    text = if (distance >= 1000) "%.1f km".format(distance / 1000) else "%.0f m".format(distance),
                    style = MaterialTheme.typography.headlineMedium,
                    color = Color.White
                )
                Text(
                    text = "to destination",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color.White.copy(alpha = 0.9f)
                )
            }
        } else {
            // Normal mode: Show VIO position
            Text(
                text = "X: %.2f, Y: %.2f, Z: %.2f".format(vioData.x, vioData.y, vioData.z),
                modifier = Modifier
                    .align(Alignment.TopCenter)
                    .padding(16.dp)
                    .background(Color.Black.copy(alpha = 0.5f))
                    .padding(8.dp),
                color = Color.White
            )
        }

        // Bottom controls
        Column(
            modifier = Modifier.align(Alignment.BottomCenter),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Row(
                modifier = Modifier.padding(8.dp),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Button(
                    onClick = { mainActivity.resetVIO() }
                ) {
                    Text("Reset VIO")
                }
                if (!isNavigating) {
                    Button(
                        onClick = { showNavigationDialog = true }
                    ) {
                        Text("Navigate")
                    }
                } else {
                    Button(
                        onClick = { mainActivity.stopNavigation() },
                        colors = ButtonDefaults.buttonColors(containerColor = Color.Red)
                    ) {
                        Text("Stop Nav")
                    }
                }
            }
        }
    }

    // Navigation dialog
    if (showNavigationDialog) {
        NavigationDialog(
            onDismiss = { showNavigationDialog = false },
            onNavigate = { destination ->
                (context as MainActivity).destinationLocation.value = destination
                showNavigationDialog = false
            },
            currentLocation = startLocation
        )
    }
}

@Composable
fun CameraView(modifier: Modifier = Modifier, context: Context) {
    val lifecycleOwner = LocalLifecycleOwner.current
    val cameraView = remember { com.otaliastudios.cameraview.CameraView(context) }
    val mainActivity = context as MainActivity

    AndroidView(
        factory = {
            cameraView.apply {
                Log.i(MainActivity.TAG, "Configuring CameraView...")
                setLifecycleOwner(lifecycleOwner)
                setAudio(com.otaliastudios.cameraview.controls.Audio.OFF)
                setFrameProcessingFormat(android.graphics.ImageFormat.YUV_420_888) // Corrected constant

                addFrameProcessor(object : com.otaliastudios.cameraview.frame.FrameProcessor { // Corrected API usage
                    override fun process(frame: com.otaliastudios.cameraview.frame.Frame) {
                        // Log.d(MainActivity.TAG, "Frame processor called. Timestamp: ${frame.time}")
                        if (!mainActivity.isNativeLibraryLoaded) {
                            return
                        }
                        val data = frame.getData<ByteArray>()
                        if (data != null) {
                            try {
                                val result = mainActivity.processCameraFrame(
                                    data,
                                    frame.size.width,
                                    frame.size.height,
                                    frame.time
                                )
                                if (result != null) {
                                    mainActivity.vioDataState.value = result
                                }
                            } catch (e: Throwable) {
                                Log.e(MainActivity.TAG, "Error processing camera frame: ${e.message}", e)
                                // Re-throw fatal errors that indicate unrecoverable app state
                                if (e is OutOfMemoryError || e is VirtualMachineError) {
                                    throw e
                                }
                                // Continue processing for recoverable exceptions
                            }
                        }
                    }
                })
                addCameraListener(object : CameraListener() {
                    override fun onCameraOpened(options: CameraOptions) {
                        super.onCameraOpened(options)
                        Log.i(MainActivity.TAG, "Camera opened!")
                    }

                    override fun onCameraError(exception: com.otaliastudios.cameraview.CameraException) {
                        super.onCameraError(exception)
                        Log.e(MainActivity.TAG, "Camera Error: ", exception)
                    }
                })
            }
        },
        modifier = modifier
    )
}

@Composable
fun MapView(
    modifier: Modifier = Modifier,
    startLocation: LatLng,
    vioData: VioData,
    destinationLocation: LatLng?,
    routePoints: List<LatLng>
) {
    val currentVioPosition = remember(vioData, startLocation) {
        metersToLatLng(startLocation, vioData.x, vioData.z)
    }

    val isNavigating = routePoints.isNotEmpty()

    // Calculate heading in degrees (0-360)
    val heading = remember(vioData.yaw) {
        val degrees = (vioData.yaw * 180.0 / Math.PI).toFloat()
        // Normalize to 0-360
        val normalized = (degrees + 360) % 360
        normalized
    }

    val cameraPositionState = rememberCameraPositionState {
        position = CameraPosition.fromLatLngZoom(startLocation, 16f)
    }

    // Auto-follow camera during navigation
    LaunchedEffect(currentVioPosition, isNavigating, heading) {
        if (isNavigating) {
            // Navigation mode: Follow position with heading orientation
            cameraPositionState.animate(
                update = com.google.android.gms.maps.CameraUpdateFactory.newCameraPosition(
                    CameraPosition.Builder()
                        .target(currentVioPosition)
                        .zoom(18f)
                        .bearing(heading) // Rotate map to match heading
                        .tilt(45f) // 3D perspective
                        .build()
                ),
                durationMs = 1000
            )
        }
    }

    // Update route periodically during navigation
    val context = LocalContext.current
    var lastUpdateDistance by remember { mutableStateOf(0.0) }
    LaunchedEffect(vioData.x, vioData.z, isNavigating) {
        if (isNavigating) {
            // Calculate distance traveled since last update
            val distanceTraveled = kotlin.math.sqrt(vioData.x * vioData.x + vioData.z * vioData.z)
            // Update route every 50 meters
            if (kotlin.math.abs(distanceTraveled - lastUpdateDistance) > 50) {
                lastUpdateDistance = distanceTraveled
                (context as MainActivity).updateNavigationRoute()
            }
        }
    }

    GoogleMap(
        modifier = modifier,
        cameraPositionState = cameraPositionState,
        uiSettings = com.google.maps.android.compose.MapUiSettings(
            zoomControlsEnabled = !isNavigating,
            rotationGesturesEnabled = !isNavigating,
            scrollGesturesEnabled = !isNavigating,
            tiltGesturesEnabled = !isNavigating,
            myLocationButtonEnabled = false
        )
    ) {
        // Current position marker (VIO-tracked position)
        Marker(
            state = MarkerState(position = currentVioPosition),
            title = "My Position",
            snippet = "VIO: %.1fm, %.1fm".format(vioData.x, vioData.z),
            rotation = heading,
            flat = true,
            anchor = Offset(0.5f, 0.5f)
        )

        // Destination marker
        if (destinationLocation != null) {
            Marker(
                state = MarkerState(position = destinationLocation),
                title = "Destination"
            )
        }

        // Route polyline
        if (routePoints.isNotEmpty()) {
            Polyline(
                points = routePoints,
                color = androidx.compose.ui.graphics.Color(0xFF4285F4), // Google Maps blue
                width = 12f
            )
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun NavigationDialog(
    onDismiss: () -> Unit,
    onNavigate: (LatLng) -> Unit,
    currentLocation: LatLng?
) {
    var searchText by remember { mutableStateOf("") }
    var selectedPlace by remember { mutableStateOf<LatLng?>(null) }
    var errorMessage by remember { mutableStateOf("") }
    var isSearching by remember { mutableStateOf(false) }
    val context = LocalContext.current
    val mainActivity = context as MainActivity
    val coroutineScope = rememberCoroutineScope()

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Navigate to Destination") },
        text = {
            Column {
                OutlinedTextField(
                    value = searchText,
                    onValueChange = {
                        searchText = it
                        selectedPlace = null
                        errorMessage = ""
                    },
                    label = { Text("Enter address or place") },
                    placeholder = { Text("e.g., Times Square, New York") },
                    modifier = Modifier.fillMaxWidth(),
                    singleLine = true,
                    trailingIcon = {
                        if (isSearching) {
                            CircularProgressIndicator(
                                modifier = Modifier.size(24.dp),
                                strokeWidth = 2.dp
                            )
                        }
                    }
                )

                if (errorMessage.isNotEmpty()) {
                    Text(
                        text = errorMessage,
                        color = Color.Red,
                        modifier = Modifier.padding(top = 8.dp),
                        style = MaterialTheme.typography.bodySmall
                    )
                }

                if (selectedPlace != null) {
                    Text(
                        text = "✓ Location found: ${selectedPlace!!.latitude}, ${selectedPlace!!.longitude}",
                        color = Color(0xFF4CAF50),
                        modifier = Modifier.padding(top = 8.dp),
                        style = MaterialTheme.typography.bodySmall
                    )
                }

                Spacer(modifier = Modifier.height(16.dp))
                Text("Quick test destinations:", style = MaterialTheme.typography.bodySmall)
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Button(
                        onClick = {
                            currentLocation?.let {
                                selectedPlace = LatLng(it.latitude + 0.0045, it.longitude)
                                searchText = "500m North"
                                errorMessage = ""
                            }
                        },
                        modifier = Modifier.weight(1f)
                    ) {
                        Text("500m N")
                    }
                    Button(
                        onClick = {
                            currentLocation?.let {
                                selectedPlace = LatLng(it.latitude, it.longitude + 0.0045)
                                searchText = "500m East"
                                errorMessage = ""
                            }
                        },
                        modifier = Modifier.weight(1f)
                    ) {
                        Text("500m E")
                    }
                }

                Spacer(modifier = Modifier.height(8.dp))
                Text(
                    text = "VIO will track your position. Route shows on map.",
                    style = MaterialTheme.typography.bodySmall,
                    color = Color.Gray
                )
            }
        },
        confirmButton = {
            Button(
                onClick = {
                    // First check if it's already a selected place
                    if (selectedPlace != null) {
                        mainActivity.destinationLocation.value = selectedPlace
                        mainActivity.startNavigation(selectedPlace!!)
                        onNavigate(selectedPlace!!)
                        onDismiss()
                        return@Button
                    }

                    // Try to parse as coordinates
                    val coordDestination = parseLatLng(searchText)
                    if (coordDestination != null) {
                        mainActivity.destinationLocation.value = coordDestination
                        mainActivity.startNavigation(coordDestination)
                        onNavigate(coordDestination)
                        onDismiss()
                        return@Button
                    }

                    // Search as place name
                    if (searchText.isNotBlank()) {
                        isSearching = true
                        errorMessage = ""
                        coroutineScope.launch {
                            val placeResult = mainActivity.searchPlace(searchText)
                            isSearching = false
                            if (placeResult != null) {
                                selectedPlace = placeResult
                                mainActivity.destinationLocation.value = placeResult
                                mainActivity.startNavigation(placeResult)
                                onNavigate(placeResult)
                                onDismiss()
                            } else {
                                errorMessage = "Place not found. Try: 'City, Country' or 'lat,lng'"
                            }
                        }
                    } else {
                        errorMessage = "Please enter a destination"
                    }
                },
                enabled = !isSearching
            ) {
                Text(if (isSearching) "Searching..." else "Show Route")
            }
        },
        dismissButton = {
            TextButton(
                onClick = onDismiss,
                enabled = !isSearching
            ) {
                Text("Cancel")
            }
        }
    )
}

fun parseLatLng(text: String): LatLng? {
    return try {
        val parts = text.split(",")
        if (parts.size == 2) {
            val lat = parts[0].trim().toDouble()
            val lng = parts[1].trim().toDouble()
            LatLng(lat, lng)
        } else {
            null
        }
    } catch (e: Exception) {
        null
    }
}

fun metersToLatLng(start: LatLng, dx: Double, dz: Double): LatLng {
    val lat = start.latitude + (dz / MainActivity.METERS_PER_DEGREE)
    val lng = start.longitude + (dx / (MainActivity.METERS_PER_DEGREE * Math.cos(Math.toRadians(start.latitude))))
    return LatLng(lat, lng)
}

fun calculateDistance(from: LatLng, to: LatLng): Double {
    val earthRadius = 6371000.0 // meters
    val dLat = Math.toRadians(to.latitude - from.latitude)
    val dLng = Math.toRadians(to.longitude - from.longitude)
    val a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(Math.toRadians(from.latitude)) * Math.cos(Math.toRadians(to.latitude)) *
            Math.sin(dLng / 2) * Math.sin(dLng / 2)
    val c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
    return earthRadius * c
}
