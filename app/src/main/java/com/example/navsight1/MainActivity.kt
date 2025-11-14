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
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.Button
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.rememberMultiplePermissionsState
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.google.android.gms.maps.model.CameraPosition
import com.google.android.gms.maps.model.LatLng
import com.google.android.gms.tasks.CancellationTokenSource
import com.google.maps.android.compose.GoogleMap
import com.google.maps.android.compose.Marker
import com.google.maps.android.compose.MarkerState
import com.google.maps.android.compose.rememberCameraPositionState
import com.otaliastudios.cameraview.CameraListener
import com.otaliastudios.cameraview.CameraOptions
import com.otaliastudios.cameraview.CameraView
import org.opencv.android.OpenCVLoader
import java.nio.ByteBuffer

class MainActivity : ComponentActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null
    private lateinit var fusedLocationClient: FusedLocationProviderClient
    lateinit var jniVersionString: String

    // State to hold the latest VIO data
    val vioDataState = mutableStateOf(VioData())
    val startLocation = mutableStateOf<LatLng?>(null)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        jniVersionString = stringFromJNI()
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)

        setContent {
            NavSightApp(vioDataState.value, startLocation.value, jniVersionString)
        }

        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    }

    @SuppressLint("MissingPermission")
    fun requestInitialLocation() {
        fusedLocationClient.getCurrentLocation(Priority.PRIORITY_HIGH_ACCURACY, CancellationTokenSource().token)
            .addOnSuccessListener { location ->
                if (location != null) {
                    startLocation.value = LatLng(location.latitude, location.longitude)
                    resetVIO() // Reset VIO origin to this new location
                }
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
        startVIO()
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
        stopVIO()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Do nothing
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event != null) {
            when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    processAccelerometer(event.timestamp, event.values[0], event.values[1], event.values[2])
                }
                Sensor.TYPE_GYROSCOPE -> {
                    processGyroscope(event.timestamp, event.values[0], event.values[1], event.values[2])
                }
            }
        }
    }

    external fun stringFromJNI(): String
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
        init {
            if (OpenCVLoader.initDebug()) {
                Log.d(TAG, "OpenCV is initialized.")
            } else {
                Log.d(TAG, "OpenCV is not initialized.")
            }
            System.loadLibrary("navsight")
        }
    }
}

@OptIn(ExperimentalPermissionsApi::class)
@Composable
fun NavSightApp(vioData: VioData, startLocation: LatLng?, jniVersion: String) {
    val context = LocalContext.current
    val permissionsState = rememberMultiplePermissionsState(
        permissions = listOf(
            Manifest.permission.CAMERA,
            Manifest.permission.ACCESS_FINE_LOCATION
        )
    )

    LaunchedEffect(permissionsState.allPermissionsGranted) {
        if (permissionsState.allPermissionsGranted) {
            (context as MainActivity).requestInitialLocation()
        }
    }

    if (permissionsState.allPermissionsGranted) {
        MainScreen(context, vioData, startLocation, jniVersion)
    } else {
        Column(modifier = Modifier.fillMaxSize(), horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = "Camera and Location permissions are required to use this app.",
                modifier = Modifier.padding(16.dp),
                textAlign = TextAlign.Center
            )
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
fun MainScreen(context: Context, vioData: VioData, startLocation: LatLng?, jniVersion: String) {
    var sliderPosition by remember { mutableStateOf(1.0f) }

    Box(modifier = Modifier.fillMaxSize()) {
        Column(modifier = Modifier.fillMaxSize()) {
            Box(modifier = Modifier.fillMaxWidth().weight(1f)) {
                CameraView(
                    modifier = Modifier.fillMaxSize(),
                    context = context
                )
                TrackedPointsOverlay(points = vioData.trackedPoints)
            }
            if (startLocation != null) {
                MapView(
                    modifier = Modifier
                        .fillMaxWidth()
                        .weight(1f),
                    startLocation = startLocation,
                    vioData = vioData
                )
            } else {
                Box(modifier = Modifier.fillMaxWidth().weight(1f), contentAlignment = Alignment.Center) {
                    Text("Getting initial location...")
                }
            }
        }
        Text(
            text = "JNI: $jniVersion",
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(16.dp)
                .background(Color.Black.copy(alpha = 0.5f))
                .padding(8.dp),
            color = Color.White
        )
        Text(
            text = "X: %.2f, Y: %.2f, Z: %.2f".format(vioData.x, vioData.y, vioData.z),
            modifier = Modifier
                .align(Alignment.TopCenter)
                .padding(16.dp)
                .background(Color.Black.copy(alpha = 0.5f))
                .padding(8.dp),
            color = Color.White
        )
        Column(
            modifier = Modifier.align(Alignment.BottomCenter),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Slider(
                value = sliderPosition,
                onValueChange = {
                    sliderPosition = it
                    (context as MainActivity).setScale(it.toDouble())
                },
                valueRange = 0.0f..10.0f,
                modifier = Modifier.padding(horizontal = 32.dp)
            )
            Button(
                onClick = { (context as MainActivity).resetVIO() },
                modifier = Modifier.padding(16.dp)
            ) {
                Text("Reset VIO")
            }
        }
    }
}

@Composable
fun CameraView(modifier: Modifier = Modifier, context: Context) {
    val lifecycleOwner = LocalLifecycleOwner.current
    val cameraView = remember { CameraView(context) }
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
                        val data = frame.getData<ByteArray>()
                        if (data != null) {
                            val result = mainActivity.processCameraFrame(
                                data,
                                frame.size.width,
                                frame.size.height,
                                frame.time
                            )
                            if (result != null) {
                                mainActivity.vioDataState.value = result
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
fun MapView(modifier: Modifier = Modifier, startLocation: LatLng, vioData: VioData) {
    val currentVioPosition = remember(vioData, startLocation) {
        metersToLatLng(startLocation, vioData.x, vioData.z)
    }

    val cameraPositionState = rememberCameraPositionState {
        position = CameraPosition.fromLatLngZoom(startLocation, 18f)
    }

    GoogleMap(
        modifier = modifier,
        cameraPositionState = cameraPositionState
    ) {
        Marker(
            state = MarkerState(position = currentVioPosition),
            title = "My Position",
            rotation = vioData.yaw.toFloat() * (180f / Math.PI.toFloat()) // Convert rad to deg
        )
    }
}

fun metersToLatLng(start: LatLng, dx: Double, dz: Double): LatLng {
    val lat = start.latitude + (dz / 111111.0)
    val lng = start.longitude + (dx / (111111.0 * Math.cos(Math.toRadians(start.latitude))))
    return LatLng(lat, lng)
}
