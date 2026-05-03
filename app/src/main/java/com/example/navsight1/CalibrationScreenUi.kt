package com.example.navsight1

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.util.Size
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.FocusMeteringAction
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.AspectRatioStrategy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.ui.input.pointer.pointerInput
import java.util.concurrent.TimeUnit
import androidx.compose.animation.core.animateFloat
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.opencv.android.OpenCVLoader
import org.opencv.calib3d.Calib3d
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point3
import java.nio.ByteBuffer
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.min
import kotlin.math.sqrt

private const val CTAG = "CalibrationScreen"
private const val CHECKER_COLS = 9
private const val CHECKER_ROWS = 6
private const val MIN_CAPTURES = 30
private const val SHARPNESS_THRESHOLD = 80.0
private const val MIN_ANGULAR_DELTA_DEG = 8.0
private const val MIN_TRANS_DELTA_M = 0.05
private const val MIN_INTERVAL_MS = 400L
private const val DETECT_INTERVAL_MS = 100L
private const val GRID_DIM = 3
private const val GRID_CELL_NEEDED = 2
const val CALIB_FILE = "camera_calib.json"

enum class CalibrationStep { Onboarding, Capturing, Solving, Verdict }

data class CalibrationCapture(
    val corners: MatOfPoint2f,
    val cellIndex: Int,
    val rvec: DoubleArray,
    val tvec: DoubleArray,
    val centroidX: Double,
    val centroidY: Double,
)

data class CalibrationResult(
    val rms: Double,
    val fx: Double,
    val fy: Double,
    val cx: Double,
    val cy: Double,
    val dist: DoubleArray,   // length 8 (k1,k2,p1,p2,k3,k4,k5,k6)
    val imageWidth: Int,
    val imageHeight: Int,
    val imageCount: Int,
)

private fun ensureOpenCV(): Boolean = try { OpenCVLoader.initDebug() } catch (e: Throwable) {
    Log.e(CTAG, "OpenCV init failed", e); false
}

private fun buildObjectPointsTemplate(squareMm: Float): MatOfPoint3f {
    val pts = ArrayList<Point3>(CHECKER_COLS * CHECKER_ROWS)
    for (r in 0 until CHECKER_ROWS) {
        for (c in 0 until CHECKER_COLS) {
            pts.add(Point3((c * squareMm).toDouble(), (r * squareMm).toDouble(), 0.0))
        }
    }
    return MatOfPoint3f().apply { fromList(pts) }
}

private fun rotationDeltaDeg(a: DoubleArray, b: DoubleArray): Double {
    // Treat rvec as axis-angle (Rodrigues vector). Approximate angular distance via
    // the magnitude of rvec_a - rvec_b — exact for small differences and a good
    // proxy for our gating threshold.
    val dx = a[0] - b[0]; val dy = a[1] - b[1]; val dz = a[2] - b[2]
    return Math.toDegrees(sqrt(dx * dx + dy * dy + dz * dz))
}

private fun translationDeltaM(a: DoubleArray, b: DoubleArray): Double {
    // tvec is in millimetres (squareMm units); convert to metres.
    val dx = (a[0] - b[0]) / 1000.0
    val dy = (a[1] - b[1]) / 1000.0
    val dz = (a[2] - b[2]) / 1000.0
    return sqrt(dx * dx + dy * dy + dz * dz)
}

@Composable
fun CalibrationScreen(
    viewModel: NavSightViewModel,
    pal: NavPalette,
    onClose: () -> Unit,
) {
    val context = LocalContext.current
    var step by remember { mutableStateOf(CalibrationStep.Onboarding) }
    var squareMmText by remember { mutableStateOf("25.0") }
    val captures = remember { mutableStateListOf<CalibrationCapture>() }
    val coverage = remember { mutableStateListOf<Int>().apply { repeat(GRID_DIM * GRID_DIM) { add(0) } } }
    var imageSize by remember { mutableStateOf(android.util.Size(640, 480)) }
    var lastResult by remember { mutableStateOf<CalibrationResult?>(null) }
    var solveError by remember { mutableStateOf<String?>(null) }
    val openCvOk = remember { ensureOpenCV() }

    val scope = rememberCoroutineScope()

    Box(Modifier.fillMaxSize().background(pal.bg)) {
        when (step) {
            CalibrationStep.Onboarding -> CalibrationOnboarding(
                pal = pal,
                openCvOk = openCvOk,
                squareMmText = squareMmText,
                onSquareMmChange = { squareMmText = it },
                onStart = { if (openCvOk) step = CalibrationStep.Capturing },
                onClose = onClose,
            )
            CalibrationStep.Capturing -> CalibrationCapturing(
                pal = pal,
                captures = captures,
                coverage = coverage,
                squareMm = squareMmText.toFloatOrNull() ?: 25.0f,
                onImageSize = { imageSize = it },
                onSolve = {
                    step = CalibrationStep.Solving
                    scope.launch(Dispatchers.Default) {
                        val res = runCatching {
                            solveCalibration(captures, imageSize, squareMmText.toFloatOrNull() ?: 25.0f)
                        }
                        withContext(Dispatchers.Main) {
                            res.onSuccess { lastResult = it; solveError = null }
                                .onFailure { solveError = it.message ?: "calibration failed"; lastResult = null }
                            step = CalibrationStep.Verdict
                        }
                    }
                },
                onCancel = onClose,
                onRedo = {
                    captures.forEach { it.corners.release() }
                    captures.clear()
                    for (i in coverage.indices) coverage[i] = 0
                },
            )
            CalibrationStep.Solving -> CalibrationSolving(pal = pal)
            CalibrationStep.Verdict -> CalibrationVerdict(
                pal = pal,
                result = lastResult,
                error = solveError,
                squareMm = squareMmText.toFloatOrNull() ?: 25.0f,
                onSave = {
                    val r = lastResult ?: return@CalibrationVerdict
                    if (r.rms <= 1.0) {
                        scope.launch(Dispatchers.IO) {
                            val ok = saveCalibrationJson(context, r, squareMmText.toFloatOrNull() ?: 25.0f)
                            withContext(Dispatchers.Main) {
                                if (ok) {
                                    viewModel.refreshCalibrationLoaded()
                                    onClose()
                                }
                            }
                        }
                    }
                },
                onRedo = {
                    captures.forEach { it.corners.release() }
                    captures.clear()
                    for (i in coverage.indices) coverage[i] = 0
                    lastResult = null; solveError = null
                    step = CalibrationStep.Capturing
                },
                onAddMore = {
                    lastResult = null; solveError = null
                    step = CalibrationStep.Capturing
                },
                onClose = onClose,
            )
        }
    }
}

// ── Onboarding ────────────────────────────────────────────────────────────────
@Composable
private fun CalibrationOnboarding(
    pal: NavPalette,
    openCvOk: Boolean,
    squareMmText: String,
    onSquareMmChange: (String) -> Unit,
    onStart: () -> Unit,
    onClose: () -> Unit,
) {
    Column(
        Modifier.fillMaxSize().padding(20.dp).statusBarsPadding(),
        verticalArrangement = Arrangement.spacedBy(14.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Surface(onClick = onClose, color = pal.card, shape = CircleShape) {
                Icon(Icons.Default.Close, "Close", tint = pal.textPrimary,
                    modifier = Modifier.padding(10.dp).size(20.dp))
            }
            Spacer(Modifier.width(10.dp))
            Text("Camera calibration", color = pal.textPrimary, fontSize = 20.sp, fontWeight = FontWeight.Bold)
        }
        Spacer(Modifier.height(4.dp))
        Surface(color = pal.card, shape = RoundedCornerShape(20.dp), border = BorderStroke(1.dp, pal.cardBorder)) {
            Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
                Text("Why this matters", color = pal.teal, fontSize = 12.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
                Text("VIO accuracy depends on knowing your phone's camera lens. A 2-minute calibration replaces the zero-distortion default with a real per-device intrinsic + distortion fit.",
                    color = pal.textPrimary, fontSize = 14.sp, lineHeight = 19.sp)
                Spacer(Modifier.height(2.dp))
                Text("How", color = pal.teal, fontSize = 12.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
                Text("1. Print OpenCV's 9x6 checkerboard at 100% scale.\n2. Tape it flat to a wall.\n3. Follow the live prompts: head-on, tilt, close, far.\n4. Save when the verdict is green.",
                    color = pal.textPrimary, fontSize = 14.sp, lineHeight = 19.sp)
            }
        }
        Surface(color = pal.card, shape = RoundedCornerShape(20.dp), border = BorderStroke(1.dp, pal.cardBorder)) {
            Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Square edge size", color = pal.teal, fontSize = 12.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
                OutlinedTextField(
                    value = squareMmText, onValueChange = onSquareMmChange,
                    label = { Text("millimetres") },
                    singleLine = true,
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
                    modifier = Modifier.fillMaxWidth(),
                )
                Text("Default 25 mm matches OpenCV's pattern.png at 100% on A4. Measure with a ruler if unsure — square size affects translation only, not the final intrinsics.",
                    color = pal.textSecondary, fontSize = 11.sp)
            }
        }
        if (!openCvOk) {
            Surface(color = Color(0xFFEF5350).copy(0.18f), shape = RoundedCornerShape(14.dp)) {
                Text("OpenCV native library failed to initialise. Calibration unavailable on this device.",
                    color = Color(0xFFEF5350), fontSize = 12.sp, modifier = Modifier.padding(12.dp))
            }
        }
        Spacer(Modifier.weight(1f))
        Button(
            onClick = onStart,
            enabled = openCvOk && (squareMmText.toFloatOrNull() ?: 0f) > 1f,
            colors = ButtonDefaults.buttonColors(containerColor = pal.teal),
            modifier = Modifier.fillMaxWidth().height(56.dp),
            shape = RoundedCornerShape(16.dp),
        ) {
            Icon(Icons.Default.PlayArrow, null, tint = Color.White)
            Spacer(Modifier.width(8.dp))
            Text("Start calibration", color = Color.White, fontSize = 16.sp, fontWeight = FontWeight.Bold)
        }
    }
}

// ── Capturing ─────────────────────────────────────────────────────────────────
@Composable
private fun CalibrationCapturing(
    pal: NavPalette,
    captures: MutableList<CalibrationCapture>,
    coverage: MutableList<Int>,
    squareMm: Float,
    onImageSize: (android.util.Size) -> Unit,
    onSolve: () -> Unit,
    onCancel: () -> Unit,
    onRedo: () -> Unit,
) {
    val lifecycleOwner = LocalLifecycleOwner.current
    var cornersDetected by remember { mutableStateOf(false) }
    var lastCornersUi by remember { mutableStateOf<List<Offset>>(emptyList()) }
    var lastCentroidAnalyzer by remember { mutableStateOf(Offset(-1f, -1f)) }
    var lastRotation by remember { mutableStateOf(0) }
    var frameW by remember { mutableStateOf(640) }
    var frameH by remember { mutableStateOf(480) }
    var sharpness by remember { mutableStateOf(0.0) }
    var manualCaptureRequested by remember { mutableStateOf(false) }
    var lastTelemetryMs by remember { mutableStateOf(0L) }

    val analyzerExec = remember { Executors.newSingleThreadExecutor() }
    val mainHandler = remember { Handler(Looper.getMainLooper()) }
    val processingFlag = remember { AtomicBoolean(false) }
    val objTemplate = remember(squareMm) { buildObjectPointsTemplate(squareMm) }
    val intrinsicsGuess = remember { Mat.eye(3, 3, CvType.CV_64F) }
    // Plain refs for analyzer-thread-only mutation; not Compose state.
    val lastDetectMsRef = remember { java.util.concurrent.atomic.AtomicLong(0L) }
    val lastCaptureMsRef = remember { java.util.concurrent.atomic.AtomicLong(0L) }
    val context = LocalContext.current

    DisposableEffect(Unit) {
        onDispose {
            // Release the back camera so the regular CameraViewComposable can re-bind
            // when the calibration screen closes.
            try {
                val provider = ProcessCameraProvider.getInstance(context).get()
                provider.unbindAll()
            } catch (e: Throwable) { Log.w(CTAG, "unbind on dispose failed", e) }
            analyzerExec.shutdown()
            objTemplate.release()
            intrinsicsGuess.release()
        }
    }

    // Auto-trigger solve as soon as the conditions are met. Without this, the
    // user has to spot the bottom "Solve calibration" button — which is small,
    // and the directional prompt above suggested "solving…" was already in
    // progress. Now the moment we have 30+ captures AND every cell has ≥ 2
    // captures, kick off the solve. The Solve button stays as a manual override.
    val coverFullState = coverage.all { it >= GRID_CELL_NEEDED }
    val canSolveAuto = captures.size >= MIN_CAPTURES && coverFullState
    LaunchedEffect(canSolveAuto) {
        if (canSolveAuto) {
            Log.i(CTAG, "Auto-solve triggered (captures=${captures.size}, coverage=full)")
            onSolve()
        }
    }

    // Hold the bound Camera + PreviewView so the tap-to-focus gesture
    // can call into the camera control + meteringPointFactory.
    val cameraRef = remember { java.util.concurrent.atomic.AtomicReference<Camera?>(null) }
    val previewViewRef = remember { java.util.concurrent.atomic.AtomicReference<PreviewView?>(null) }
    var focusTapPx by remember { mutableStateOf<Offset?>(null) }
    LaunchedEffect(focusTapPx) {
        val tap = focusTapPx ?: return@LaunchedEffect
        val cam = cameraRef.get() ?: return@LaunchedEffect
        val pv = previewViewRef.get() ?: return@LaunchedEffect
        runCatching {
            val factory = pv.meteringPointFactory
            val point = factory.createPoint(tap.x, tap.y)
            val action = FocusMeteringAction.Builder(point, FocusMeteringAction.FLAG_AF)
                .setAutoCancelDuration(3, TimeUnit.SECONDS)
                .build()
            cam.cameraControl.startFocusAndMetering(action)
            Log.i(CTAG, "Tap-to-focus at (${tap.x.toInt()}, ${tap.y.toInt()})")
        }.onFailure { Log.w(CTAG, "tap-to-focus failed", it) }
        // Auto-clear after 1s so the focus indicator can fade.
        kotlinx.coroutines.delay(1000)
        focusTapPx = null
    }

    Box(Modifier.fillMaxSize()) {
        AndroidView(
            factory = { ctx ->
                val pv = PreviewView(ctx).apply {
                    implementationMode = PreviewView.ImplementationMode.PERFORMANCE
                    scaleType = PreviewView.ScaleType.FILL_CENTER
                }
                previewViewRef.set(pv)
                val future = ProcessCameraProvider.getInstance(ctx)
                future.addListener({
                    val provider = future.get()
                    val preview = Preview.Builder().build().also { it.setSurfaceProvider(pv.surfaceProvider) }
                    // ResolutionSelector with strict 4:3 + 640×480 target —
                    // matches VIO's runtime camera path so calibration
                    // intrinsics apply 1:1 at runtime. Replaces the
                    // deprecated setTargetResolution which let some devices
                    // (e.g. Samsung S22) deliver a square 1088×1088 crop.
                    val resSelector = ResolutionSelector.Builder()
                        .setAspectRatioStrategy(AspectRatioStrategy.RATIO_4_3_FALLBACK_AUTO_STRATEGY)
                        .setResolutionStrategy(
                            ResolutionStrategy(
                                Size(640, 480),
                                ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER,
                            )
                        )
                        .build()
                    val analysis = ImageAnalysis.Builder()
                        .setResolutionSelector(resSelector)
                        .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                        .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                        .build()
                        .also { ia ->
                            ia.setAnalyzer(analyzerExec) { img ->
                                val nowMs = System.currentTimeMillis()
                                if (nowMs - lastDetectMsRef.get() < DETECT_INTERVAL_MS) {
                                    img.close(); return@setAnalyzer
                                }
                                if (!processingFlag.compareAndSet(false, true)) {
                                    img.close(); return@setAnalyzer
                                }
                                lastDetectMsRef.set(nowMs)
                                try {
                                    val outcome = processFrame(
                                        img = img,
                                        objTemplate = objTemplate,
                                        intrinsicsGuess = intrinsicsGuess,
                                        capturesSnapshot = captures.toList(),
                                        nowMs = nowMs,
                                        lastCaptureMs = lastCaptureMsRef.get(),
                                        manualCaptureRequested = manualCaptureRequested,
                                    )
                                    mainHandler.post {
                                        if (frameW != outcome.frameW || frameH != outcome.frameH) {
                                            frameW = outcome.frameW
                                            frameH = outcome.frameH
                                            onImageSize(android.util.Size(outcome.frameW, outcome.frameH))
                                        }
                                        cornersDetected = outcome.detected
                                        lastCornersUi = outcome.cornersUi
                                        lastCentroidAnalyzer = outcome.centroidAnalyzer
                                        lastRotation = outcome.rotationDegrees
                                        sharpness = outcome.sharpness
                                        if (outcome.capture != null) {
                                            captures.add(outcome.capture)
                                            val cell = outcome.capture.cellIndex
                                            if (cell in coverage.indices) coverage[cell] = coverage[cell] + 1
                                            lastCaptureMsRef.set(nowMs)
                                            manualCaptureRequested = false
                                        }
                                        // Telemetry once per second so we can debug the
                                        // rotation/cell math from logcat.
                                        if (nowMs - lastTelemetryMs > 1000) {
                                            lastTelemetryMs = nowMs
                                            val (pw, ph) = rotatedDims(outcome.frameW, outcome.frameH,
                                                                        outcome.rotationDegrees)
                                            val curCell = if (outcome.detected) {
                                                val pc = rotatePoint(
                                                    outcome.centroidAnalyzer.x,
                                                    outcome.centroidAnalyzer.y,
                                                    outcome.frameW, outcome.frameH,
                                                    outcome.rotationDegrees,
                                                )
                                                previewGridCell(pc.x, pc.y, pw, ph)
                                            } else -1
                                            val targetCell = findTargetCell(coverage)
                                            val filled = coverage.count { it >= GRID_CELL_NEEDED }
                                            Log.i(CTAG,
                                                "CALIB: w=${outcome.frameW} h=${outcome.frameH} " +
                                                "rot=${outcome.rotationDegrees} " +
                                                "centroid=(${outcome.centroidAnalyzer.x.toInt()}," +
                                                "${outcome.centroidAnalyzer.y.toInt()}) " +
                                                "rotW=${pw} rotH=${ph} " +
                                                "cell=$curCell target=$targetCell coverage=$filled/9")
                                        }
                                    }
                                } catch (e: Throwable) {
                                    Log.w(CTAG, "frame proc failed", e)
                                } finally {
                                    processingFlag.set(false)
                                    img.close()
                                }
                            }
                        }
                    try {
                        provider.unbindAll()
                        val cam = provider.bindToLifecycle(
                            lifecycleOwner, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis,
                        )
                        cameraRef.set(cam)
                    } catch (e: Exception) { Log.e(CTAG, "bind failed", e) }
                }, ContextCompat.getMainExecutor(ctx))
                pv
            },
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(Unit) {
                    detectTapGestures(onTap = { tapOffset -> focusTapPx = tapOffset })
                },
        )

        // Coverage grid + corner overlay (with rotation + active target guidance)
        CalibrationOverlay(
            cornersDetected = cornersDetected,
            cornersUi = lastCornersUi,
            centroidAnalyzer = lastCentroidAnalyzer,
            rotationDegrees = lastRotation,
            frameW = frameW,
            frameH = frameH,
            coverage = coverage,
            focusTapPx = focusTapPx,
        )

        // Top status row
        Row(
            modifier = Modifier.align(Alignment.TopStart).fillMaxWidth().statusBarsPadding().padding(12.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Surface(onClick = onCancel, color = Color.Black.copy(0.5f), shape = CircleShape) {
                Icon(Icons.Default.ArrowBack, "Back", tint = Color.White,
                    modifier = Modifier.padding(10.dp).size(20.dp))
            }
            CaptureProgressChip(captures.size, MIN_CAPTURES, pal)
        }

        // Active directional prompt: tells the user exactly what to do based
        // on detected centroid + nearest under-covered cell. Replaces the
        // previous static "Aim at the checkerboard" message.
        run {
            val (prevW, prevH) = rotatedDims(frameW, frameH, lastRotation)
            val curCell = if (cornersDetected && lastCentroidAnalyzer.x >= 0f) {
                val pc = rotatePoint(
                    lastCentroidAnalyzer.x, lastCentroidAnalyzer.y,
                    frameW, frameH, lastRotation,
                )
                previewGridCell(pc.x, pc.y, prevW.coerceAtLeast(1), prevH.coerceAtLeast(1))
            } else -1
            val tgt = findTargetCell(coverage)
            val msg = directionPrompt(curCell, tgt)
            Surface(
                color = Color.Black.copy(0.65f), shape = RoundedCornerShape(14.dp),
                modifier = Modifier.align(Alignment.Center).padding(20.dp),
            ) {
                Text(
                    msg,
                    color = Color.White, fontSize = 16.sp, fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(horizontal = 18.dp, vertical = 12.dp),
                )
            }
        }

        // Bottom controls — slim + translucent so the bottom-row grid
        // cells underneath remain visible. The Solve button was removed:
        // solve auto-triggers via LaunchedEffect when 30+ captures with
        // full coverage are reached. Only the manual override (Capture +
        // Redo) and a one-line status remain here.
        Surface(
            color = Color.Black.copy(alpha = 0.55f),
            shape = RoundedCornerShape(topStart = 18.dp, topEnd = 18.dp),
            modifier = Modifier.align(Alignment.BottomCenter).fillMaxWidth().navigationBarsPadding(),
        ) {
            Column(
                Modifier.padding(horizontal = 14.dp, vertical = 10.dp),
                verticalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                Row(
                    horizontalArrangement = Arrangement.spacedBy(8.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Button(
                        onClick = { manualCaptureRequested = true },
                        enabled = cornersDetected,
                        modifier = Modifier.weight(1f).height(42.dp),
                        colors = ButtonDefaults.buttonColors(containerColor = pal.teal),
                        shape = RoundedCornerShape(12.dp),
                    ) {
                        Icon(Icons.Default.PhotoCamera, null, tint = Color.White,
                             modifier = Modifier.size(16.dp))
                        Spacer(Modifier.width(4.dp))
                        Text("Capture", color = Color.White, fontWeight = FontWeight.Bold,
                             fontSize = 13.sp)
                    }
                    OutlinedButton(
                        onClick = onRedo,
                        modifier = Modifier.height(42.dp),
                        shape = RoundedCornerShape(12.dp),
                    ) { Text("Redo", color = Color.White, fontSize = 13.sp) }
                }
                val coverFull = coverage.all { it >= GRID_CELL_NEEDED }
                val cellsFilled = coverage.count { it >= GRID_CELL_NEEDED }
                val statusLine = when {
                    coverFull && captures.size >= MIN_CAPTURES -> "Solving…"
                    coverFull -> "Full coverage — ${MIN_CAPTURES - captures.size} more captures · tap preview to focus"
                    else -> "$cellsFilled/9 cells  •  ${captures.size}/$MIN_CAPTURES caps  •  sharp ${sharpness.toInt()}  •  tap to focus"
                }
                Text(statusLine, color = Color.White.copy(alpha = 0.85f), fontSize = 11.sp)
            }
        }
    }
}

@Composable
private fun CalibrationOverlay(
    cornersDetected: Boolean,
    cornersUi: List<Offset>,
    centroidAnalyzer: Offset,
    rotationDegrees: Int,
    frameW: Int,
    frameH: Int,
    coverage: MutableList<Int>,
    focusTapPx: Offset?,
) {
    // Pulse for target-cell highlight.
    val infiniteTransition = androidx.compose.animation.core.rememberInfiniteTransition(label = "pulse")
    val pulse by infiniteTransition.animateFloat(
        initialValue = 0.45f, targetValue = 1.0f,
        animationSpec = androidx.compose.animation.core.infiniteRepeatable(
            animation = androidx.compose.animation.core.tween(
                durationMillis = 700,
                easing = androidx.compose.animation.core.LinearEasing,
            ),
            repeatMode = androidx.compose.animation.core.RepeatMode.Reverse,
        ),
        label = "pulseAlpha",
    )

    Canvas(Modifier.fillMaxSize()) {
        val w = size.width; val h = size.height
        if (frameW <= 0 || frameH <= 0) return@Canvas

        // Grid fills the entire canvas. The bottom UI (controls bar) is
        // intentionally translucent + slim so the bottom-row cells remain
        // visible underneath — see the Surface in CalibrationCapturing.
        val cw = w / GRID_DIM
        val ch = h / GRID_DIM

        // Map analyzer (x,y) → canvas (x,y). Since the canvas is portrait
        // and the analyzer can come in either orientation, we try both
        // possible mappings (rotated or unrotated) and pick whichever
        // lands the centroid on-screen. This sidesteps the dim-swap
        // ambiguity some devices introduce.
        fun mapping(rotForMap: Int): (Float, Float) -> Offset {
            val (rotW, rotH) = rotatedDims(frameW, frameH, rotForMap)
            val sx = w / rotW.toFloat()
            val sy = h / rotH.toFloat()
            return { x, y ->
                val r = rotatePoint(x, y, frameW, frameH, rotForMap)
                Offset(r.x * sx, r.y * sy)
            }
        }

        // Pick the mapping that puts the centroid (when present) in-bounds.
        val mapPrimary = mapping(rotationDegrees)
        val toCanvas: (Float, Float) -> Offset = if (
            centroidAnalyzer.x >= 0f
        ) {
            val cPrimary = mapPrimary(centroidAnalyzer.x, centroidAnalyzer.y)
            if (cPrimary.x in 0f..w && cPrimary.y in 0f..h) {
                mapPrimary
            } else {
                // Try the other orientation. Some devices report rot=90
                // while delivering an already-portrait analyzer, in which
                // case rotation needs to be 0 (or vice versa).
                val alt = if (rotationDegrees == 90 || rotationDegrees == 270) 0 else 90
                mapping(alt)
            }
        } else mapPrimary

        // ── 3×3 coverage grid (always on, always visible) ────────────
        val targetCell = findTargetCell(coverage)
        for (i in 0 until GRID_DIM) {
            for (j in 0 until GRID_DIM) {
                val idx = i * GRID_DIM + j
                val filled = (coverage.getOrNull(idx) ?: 0) >= GRID_CELL_NEEDED
                val isTarget = (idx == targetCell)
                val tl = Offset(j * cw, i * ch)
                val cellSize = androidx.compose.ui.geometry.Size(cw, ch)
                // Filled cells: translucent green wash + darker border so
                // they're visible against any background.
                if (filled) {
                    drawRect(
                        color = Color(0x6634C759),  // ~40% alpha green
                        topLeft = tl,
                        size = cellSize,
                    )
                }
                // All cells: dark border, thick enough to read.
                drawRect(
                    color = Color(0xCC000000),  // ~80% alpha black
                    topLeft = tl,
                    size = cellSize,
                    style = Stroke(width = 3f),
                )
                if (isTarget && !filled) {
                    // Pulsing yellow border + faint yellow fill so the
                    // user can't miss the target cell.
                    drawRect(
                        color = Color(0xFFFFD60A).copy(alpha = pulse * 0.25f),
                        topLeft = tl,
                        size = cellSize,
                    )
                    drawRect(
                        color = Color(0xFFFFD60A).copy(alpha = pulse),
                        topLeft = tl,
                        size = cellSize,
                        style = Stroke(width = 8f),
                    )
                }
            }
        }

        // ── Corners — green dots on the actual checkerboard corners ──
        if (cornersDetected && cornersUi.isNotEmpty()) {
            cornersUi.forEach { p ->
                drawCircle(
                    color = Color(0xFF34C759),
                    radius = 5f,
                    center = toCanvas(p.x, p.y),
                )
            }
        }

        // ── Arrow: centroid → target cell centre (canvas-pixel coords) ──
        if (cornersDetected && centroidAnalyzer.x >= 0f && targetCell != null) {
            val from = toCanvas(centroidAnalyzer.x, centroidAnalyzer.y)
            val tr = targetCell / GRID_DIM
            val tc = targetCell % GRID_DIM
            val to = Offset((tc + 0.5f) * cw, (tr + 0.5f) * ch)
            drawLine(
                color = Color(0xFFFFD60A).copy(alpha = pulse.coerceAtLeast(0.7f)),
                start = from,
                end = to,
                strokeWidth = 10f,
            )
            val dx = to.x - from.x; val dy = to.y - from.y
            val len = sqrt(dx * dx + dy * dy)
            if (len > 1f) {
                val ux = dx / len; val uy = dy / len
                val ah = 28f
                val aw = 16f
                val baseX = to.x - ux * ah; val baseY = to.y - uy * ah
                val px = -uy; val py = ux
                val left = Offset(baseX + px * aw, baseY + py * aw)
                val right = Offset(baseX - px * aw, baseY - py * aw)
                drawLine(Color(0xFFFFD60A), to, left, strokeWidth = 10f)
                drawLine(Color(0xFFFFD60A), to, right, strokeWidth = 10f)
            }
        }

        // Tap-to-focus indicator: a fading white ring at the last tap point.
        if (focusTapPx != null) {
            drawCircle(
                color = Color.White.copy(alpha = 0.9f),
                radius = 48f,
                center = focusTapPx,
                style = Stroke(width = 4f),
            )
            drawCircle(
                color = Color.White.copy(alpha = 0.4f),
                radius = 30f,
                center = focusTapPx,
                style = Stroke(width = 2f),
            )
        }
    }
}

@Composable
private fun CaptureProgressChip(count: Int, target: Int, pal: NavPalette) {
    Surface(color = Color.Black.copy(0.55f), shape = RoundedCornerShape(14.dp)) {
        Column(Modifier.padding(horizontal = 12.dp, vertical = 6.dp)) {
            Text("$count / $target captures", color = Color.White, fontSize = 12.sp,
                fontWeight = FontWeight.Bold)
            val pct = (count.toFloat() / target).coerceIn(0f, 1f)
            LinearProgressIndicator(
                progress = { pct },
                modifier = Modifier.width(120.dp).height(3.dp).padding(top = 3.dp),
                color = pal.teal, trackColor = Color.White.copy(0.25f),
            )
        }
    }
}

@Composable
private fun CoverageStatusRow(coverage: List<Int>, pal: NavPalette) {
    val filled = coverage.count { it >= GRID_CELL_NEEDED }
    Row(verticalAlignment = Alignment.CenterVertically) {
        Icon(Icons.Default.GridOn, null, tint = pal.teal, modifier = Modifier.size(16.dp))
        Spacer(Modifier.width(6.dp))
        Text("Coverage $filled/${GRID_DIM * GRID_DIM} cells",
            color = pal.textPrimary, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
        Spacer(Modifier.width(8.dp))
        Text("(need ≥ $GRID_CELL_NEEDED captures per cell)",
            color = pal.textSecondary, fontSize = 10.sp)
    }
}

// ── Solving ───────────────────────────────────────────────────────────────────
@Composable
private fun CalibrationSolving(pal: NavPalette) {
    Box(Modifier.fillMaxSize(), Alignment.Center) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            CircularProgressIndicator(color = pal.teal, strokeWidth = 4.dp, modifier = Modifier.size(64.dp))
            Spacer(Modifier.height(20.dp))
            Text("Solving intrinsics...", color = pal.textPrimary, fontSize = 16.sp, fontWeight = FontWeight.Bold)
            Spacer(Modifier.height(6.dp))
            Text("calibrateCamera with rational distortion model",
                color = pal.textSecondary, fontSize = 11.sp)
        }
    }
}

// ── Verdict ───────────────────────────────────────────────────────────────────
@Composable
private fun CalibrationVerdict(
    pal: NavPalette,
    result: CalibrationResult?,
    error: String?,
    squareMm: Float,
    onSave: () -> Unit,
    onRedo: () -> Unit,
    onAddMore: () -> Unit,
    onClose: () -> Unit,
) {
    val tier = when {
        result == null -> -1
        result.rms < 0.5 -> 0
        result.rms < 1.0 -> 1
        else -> 2
    }
    val tint = when (tier) { 0 -> Color(0xFF34C759); 1 -> Color(0xFFFFB300); 2 -> Color(0xFFEF5350); else -> Color(0xFF9E9E9E) }
    val verdictText = when (tier) {
        0 -> "Calibration complete"
        1 -> "Usable, redo for better"
        2 -> "Calibration failed — redo"
        else -> "Solver error"
    }
    val icon = when (tier) {
        0 -> Icons.Default.CheckCircle
        1 -> Icons.Default.Warning
        else -> Icons.Default.Error
    }

    Column(Modifier.fillMaxSize().padding(20.dp).statusBarsPadding(),
        verticalArrangement = Arrangement.spacedBy(14.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Surface(onClick = onClose, color = pal.card, shape = CircleShape) {
                Icon(Icons.Default.Close, "Close", tint = pal.textPrimary,
                    modifier = Modifier.padding(10.dp).size(20.dp))
            }
            Spacer(Modifier.width(10.dp))
            Text("Verdict", color = pal.textPrimary, fontSize = 20.sp, fontWeight = FontWeight.Bold)
        }
        Surface(color = tint.copy(0.15f), shape = RoundedCornerShape(20.dp), border = BorderStroke(1.dp, tint)) {
            Row(Modifier.fillMaxWidth().padding(16.dp), verticalAlignment = Alignment.CenterVertically) {
                Icon(icon, null, tint = tint, modifier = Modifier.size(36.dp))
                Spacer(Modifier.width(12.dp))
                Column {
                    Text(verdictText, color = tint, fontSize = 18.sp, fontWeight = FontWeight.Bold)
                    if (result != null) {
                        Text("RMS reprojection ${"%.3f".format(result.rms)} px",
                            color = pal.textPrimary, fontSize = 12.sp)
                    } else if (error != null) {
                        Text(error, color = pal.textSecondary, fontSize = 12.sp)
                    }
                }
            }
        }
        if (result != null) {
            Surface(color = pal.card, shape = RoundedCornerShape(18.dp), border = BorderStroke(1.dp, pal.cardBorder)) {
                Column(Modifier.padding(14.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    StatLine("Captures", "${result.imageCount}", pal)
                    StatLine("Image size", "${result.imageWidth} x ${result.imageHeight}", pal)
                    StatLine("fx, fy", "${"%.1f".format(result.fx)}, ${"%.1f".format(result.fy)}", pal)
                    StatLine("cx, cy", "${"%.1f".format(result.cx)}, ${"%.1f".format(result.cy)}", pal)
                    StatLine("k1, k2, k3",
                        "${"%.4f".format(result.dist[0])}, ${"%.4f".format(result.dist[1])}, ${"%.4f".format(result.dist[4])}",
                        pal)
                    StatLine("p1, p2",
                        "${"%.4f".format(result.dist[2])}, ${"%.4f".format(result.dist[3])}", pal)
                    StatLine("Square size", "%.1f mm".format(squareMm), pal)
                }
            }
        }
        Spacer(Modifier.weight(1f))
        Button(
            onClick = onSave,
            enabled = result != null && result.rms <= 1.0,
            colors = ButtonDefaults.buttonColors(containerColor = pal.teal),
            modifier = Modifier.fillMaxWidth().height(54.dp),
            shape = RoundedCornerShape(14.dp),
        ) {
            Icon(Icons.Default.Save, null, tint = Color.White)
            Spacer(Modifier.width(8.dp))
            Text("Save", color = Color.White, fontWeight = FontWeight.Bold)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            OutlinedButton(onClick = onRedo, modifier = Modifier.weight(1f).height(48.dp),
                shape = RoundedCornerShape(14.dp)) { Text("Redo from scratch", color = pal.textPrimary) }
            OutlinedButton(onClick = onAddMore, modifier = Modifier.weight(1f).height(48.dp),
                shape = RoundedCornerShape(14.dp)) { Text("Add more captures", color = pal.textPrimary) }
        }
    }
}

@Composable
private fun StatLine(label: String, value: String, pal: NavPalette) {
    Row(Modifier.fillMaxWidth(), Arrangement.SpaceBetween, Alignment.CenterVertically) {
        Text(label, color = pal.textSecondary, fontSize = 11.sp)
        Text(value, color = pal.textPrimary, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
    }
}

// ── Frame processing & math ──────────────────────────────────────────────────

private data class FrameOutcome(
    val frameW: Int,           // analyzer-frame width (pre-rotation)
    val frameH: Int,           // analyzer-frame height (pre-rotation)
    val rotationDegrees: Int,  // CCW degrees to rotate analyzer → preview
    val detected: Boolean,
    val cornersUi: List<Offset>,         // analyzer-frame coords
    val centroidAnalyzer: Offset,        // analyzer-frame centroid (or (-1,-1) if !detected)
    val sharpness: Double,
    val capture: CalibrationCapture?,
)

// Rotate a point from analyzer space to preview space.
// rot is the value from ImageProxy.imageInfo.rotationDegrees (0/90/180/270),
// the CCW rotation applied by CameraX to make the analyzer image upright on
// the preview surface. The analyzer image is in sensor-native landscape
// (e.g. 640×480); the preview is in (rotW, rotH) where:
//   rot ∈ {0, 180}: rotW=w, rotH=h    (no swap)
//   rot ∈ {90, 270}: rotW=h, rotH=w   (dimensions swap)
private fun rotatePoint(x: Float, y: Float, w: Int, h: Int, rot: Int): Offset =
    when (rot) {
        90 -> Offset(h - y, x)
        180 -> Offset(w - x, h - y)
        270 -> Offset(y, w - x)
        else -> Offset(x, y)
    }

private fun rotatedDims(w: Int, h: Int, rot: Int): Pair<Int, Int> =
    if (rot == 90 || rot == 270) Pair(h, w) else Pair(w, h)

// Find the lowest-index cell whose coverage count is below the threshold.
// Row-major order means the user gets walked through cells deterministically.
private fun findTargetCell(coverage: List<Int>): Int? {
    for (i in coverage.indices) if (coverage[i] < GRID_CELL_NEEDED) return i
    return null
}

// Cell index from a preview-space point (already rotated). Same math as
// gridCell but operates on (preview-w, preview-h).
private fun previewGridCell(px: Float, py: Float, prevW: Int, prevH: Int): Int {
    val col = ((px / prevW) * GRID_DIM).toInt().coerceIn(0, GRID_DIM - 1)
    val row = ((py / prevH) * GRID_DIM).toInt().coerceIn(0, GRID_DIM - 1)
    return row * GRID_DIM + col
}

// Direction prompt for "move from currentCell toward targetCell".
// currentCell may be -1 (no detection); targetCell may be null (all done).
//
// The text is intentionally tied to the visual cues on screen: a yellow
// pulsing cell highlights the destination and an arrow draws the path
// from the current centroid to it. The words just reinforce what the
// eye already sees — they never prescribe a specific physical motion
// (move-the-phone vs move-the-board both work, depends on user).
private fun directionPrompt(currentCell: Int, targetCell: Int?): String {
    if (targetCell == null) return "✓ Full coverage — solving once captures hit ${MIN_CAPTURES}"
    if (currentCell < 0) return "📷 Point camera at the checkerboard"
    if (currentCell == targetCell) return "✓ In target cell — vary angle slightly to capture"
    val cr = currentCell / GRID_DIM; val cc = currentCell % GRID_DIM
    val tr = targetCell / GRID_DIM; val tc = targetCell % GRID_DIM
    val dr = tr - cr; val dc = tc - cc
    val arrow = when {
        dr < 0 && dc == 0 -> "⬆"
        dr < 0 && dc > 0 -> "↗"
        dr == 0 && dc > 0 -> "➡"
        dr > 0 && dc > 0 -> "↘"
        dr > 0 && dc == 0 -> "⬇"
        dr > 0 && dc < 0 -> "↙"
        dr == 0 && dc < 0 -> "⬅"
        dr < 0 && dc < 0 -> "↖"
        else -> "•"
    }
    return "$arrow Get the board into the yellow cell"
}

/**
 * Run corner detection + sharpness + auto-capture gating on a single ImageProxy.
 *
 * Pure compute: returns a [FrameOutcome] for the caller to apply to UI state
 * on the main thread. No Compose state is touched from the analyzer thread.
 */
private fun processFrame(
    img: ImageProxy,
    objTemplate: MatOfPoint3f,
    intrinsicsGuess: Mat,
    capturesSnapshot: List<CalibrationCapture>,
    nowMs: Long,
    lastCaptureMs: Long,
    manualCaptureRequested: Boolean,
): FrameOutcome {
    val w = img.width
    val h = img.height
    val yPlane = img.planes[0]
    val rowStride = yPlane.rowStride
    val yBuffer: ByteBuffer = yPlane.buffer

    // Copy Y plane to a packed byte array so OpenCV Mat sees contiguous data.
    val yBytes = ByteArray(w * h)
    yBuffer.rewind()
    if (rowStride == w) {
        yBuffer.get(yBytes, 0, w * h)
    } else {
        for (row in 0 until h) {
            yBuffer.position(row * rowStride)
            yBuffer.get(yBytes, row * w, w)
        }
    }

    val gray = Mat(h, w, CvType.CV_8UC1)
    val corners = MatOfPoint2f()
    var detected = false
    var capture: CalibrationCapture? = null
    var sharp = 0.0
    val cornersUi = ArrayList<Offset>(CHECKER_COLS * CHECKER_ROWS)
    try {
        gray.put(0, 0, yBytes)
        val flags = Calib3d.CALIB_CB_EXHAUSTIVE or Calib3d.CALIB_CB_ACCURACY
        detected = Calib3d.findChessboardCornersSB(
            gray,
            org.opencv.core.Size(CHECKER_COLS.toDouble(), CHECKER_ROWS.toDouble()),
            corners,
            flags,
        )

        if (detected && !corners.empty()) {
            // Sharpness via variance of Laplacian on the gray frame.
            val lap = Mat()
            org.opencv.imgproc.Imgproc.Laplacian(gray, lap, CvType.CV_64F)
            val mean = MatOfDouble(); val std = MatOfDouble()
            Core.meanStdDev(lap, mean, std)
            val s = std.toArray().firstOrNull() ?: 0.0
            sharp = s * s
            lap.release(); mean.release(); std.release()

            // Build UI corner list.
            val arr = corners.toArray()
            arr.forEach { cornersUi.add(Offset(it.x.toFloat(), it.y.toFloat())) }
            val centroidX = arr.sumOf { it.x } / arr.size
            val centroidY = arr.sumOf { it.y } / arr.size

            // Pose estimate via solvePnP using a scratch K (fx≈w, cx=w/2). Square_mm
            // is in millimetres; tvec inherits the same unit.
            val k = intrinsicsGuess
            k.put(0, 0, w.toDouble()); k.put(0, 1, 0.0); k.put(0, 2, w / 2.0)
            k.put(1, 0, 0.0); k.put(1, 1, w.toDouble()); k.put(1, 2, h / 2.0)
            k.put(2, 0, 0.0); k.put(2, 1, 0.0); k.put(2, 2, 1.0)
            val rvec = Mat(); val tvec = Mat()
            val zeroDist = MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)
            val pnpOk = Calib3d.solvePnP(objTemplate, corners, k, zeroDist, rvec, tvec)
            zeroDist.release()

            val rArr = if (pnpOk) DoubleArray(3).also { rvec.get(0, 0, it) } else doubleArrayOf(0.0, 0.0, 0.0)
            val tArr = if (pnpOk) DoubleArray(3).also { tvec.get(0, 0, it) } else doubleArrayOf(0.0, 0.0, 0.0)
            rvec.release(); tvec.release()

            // Auto-capture gating + manual override.
            val sinceLast = nowMs - lastCaptureMs
            val sharpOk = sharp > SHARPNESS_THRESHOLD
            val timeOk = sinceLast >= MIN_INTERVAL_MS
            val poseFar = if (capturesSnapshot.size < 3) true else {
                val last3 = capturesSnapshot.takeLast(3)
                last3.all { c ->
                    rotationDeltaDeg(rArr, c.rvec) >= MIN_ANGULAR_DELTA_DEG ||
                            translationDeltaM(tArr, c.tvec) >= MIN_TRANS_DELTA_M
                }
            }
            val shouldCapture = (manualCaptureRequested && sharpOk) ||
                (sharpOk && timeOk && poseFar && pnpOk)

            if (shouldCapture) {
                // Cell index in PREVIEW space (what the user sees) — rotate
                // the analyzer-frame centroid before bucketing. Without
                // this, "left" in the analyzer maps to a different cell
                // than the user perceives, and the coverage-grid prompts
                // disagree with what the eye sees.
                val rotForCell = img.imageInfo.rotationDegrees
                val rotatedC = rotatePoint(
                    centroidX.toFloat(), centroidY.toFloat(),
                    w, h, rotForCell,
                )
                val (rotW, rotH) = rotatedDims(w, h, rotForCell)
                val cell = previewGridCell(rotatedC.x, rotatedC.y, rotW, rotH)
                val ownedCorners = MatOfPoint2f()
                corners.copyTo(ownedCorners)
                capture = CalibrationCapture(
                    corners = ownedCorners,
                    cellIndex = cell,
                    rvec = rArr,
                    tvec = tArr,
                    centroidX = centroidX,
                    centroidY = centroidY,
                )
            }
        }
    } finally {
        gray.release()
        corners.release()
    }
    val rot = img.imageInfo.rotationDegrees
    val centroidAnalyzer = if (detected && cornersUi.isNotEmpty()) {
        var sx = 0f; var sy = 0f
        cornersUi.forEach { sx += it.x; sy += it.y }
        Offset(sx / cornersUi.size, sy / cornersUi.size)
    } else Offset(-1f, -1f)
    return FrameOutcome(w, h, rot, detected, cornersUi, centroidAnalyzer, sharp, capture)
}

private fun gridCell(cx: Double, cy: Double, w: Int, h: Int): Int {
    val col = ((cx / w) * GRID_DIM).toInt().coerceIn(0, GRID_DIM - 1)
    val row = ((cy / h) * GRID_DIM).toInt().coerceIn(0, GRID_DIM - 1)
    return row * GRID_DIM + col
}

// ── Solver ───────────────────────────────────────────────────────────────────

@Throws(Exception::class)
private fun solveCalibration(
    captures: List<CalibrationCapture>,
    imageSize: android.util.Size,
    squareMm: Float,
): CalibrationResult {
    require(captures.isNotEmpty()) { "no captures" }
    val template = buildObjectPointsTemplate(squareMm)
    val objList = ArrayList<Mat>(captures.size)
    val imgList = ArrayList<Mat>(captures.size)
    try {
        repeat(captures.size) {
            // calibrateCamera mutates per-image state; pass independent copies so we
            // never alias shared OpenCV memory across iterations.
            val o = MatOfPoint3f()
            template.copyTo(o)
            objList.add(o)
            imgList.add(captures[it].corners)
        }
        val K = Mat()
        val dist = Mat()
        val rvecs = ArrayList<Mat>()
        val tvecs = ArrayList<Mat>()
        val sz = org.opencv.core.Size(imageSize.width.toDouble(), imageSize.height.toDouble())
        val rms = Calib3d.calibrateCamera(
            objList, imgList, sz, K, dist, rvecs, tvecs, Calib3d.CALIB_RATIONAL_MODEL
        )
        val fx = K.get(0, 0)?.firstOrNull() ?: 0.0
        val fy = K.get(1, 1)?.firstOrNull() ?: 0.0
        val cx = K.get(0, 2)?.firstOrNull() ?: 0.0
        val cy = K.get(1, 2)?.firstOrNull() ?: 0.0
        val coeffs = DoubleArray(8)
        val totalCoeffs = min(8, (dist.total() * dist.channels()).toInt())
        val flat = DoubleArray(totalCoeffs)
        // CALIB_RATIONAL_MODEL returns 8 contiguous coefficients regardless of
        // whether the Mat is row- or column-shaped, so a single get() suffices.
        dist.get(0, 0, flat)
        for (i in 0 until totalCoeffs) coeffs[i] = flat[i]
        rvecs.forEach { it.release() }
        tvecs.forEach { it.release() }
        K.release(); dist.release()
        return CalibrationResult(
            rms = rms,
            fx = fx, fy = fy, cx = cx, cy = cy,
            dist = coeffs,
            imageWidth = imageSize.width, imageHeight = imageSize.height,
            imageCount = captures.size,
        )
    } finally {
        objList.forEach { it.release() }
        template.release()
    }
}

// ── Persistence ──────────────────────────────────────────────────────────────

fun calibrationFile(context: Context) = java.io.File(context.filesDir, CALIB_FILE)

fun calibrationExists(context: Context): Boolean = calibrationFile(context).exists()

private fun saveCalibrationJson(context: Context, r: CalibrationResult, squareMm: Float): Boolean {
    if (r.rms > 1.0) return false
    return try {
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US).apply {
            timeZone = TimeZone.getTimeZone("UTC")
        }.format(Date())
        val sb = StringBuilder()
        sb.append("{\n")
        sb.append("  \"image_size\": [${r.imageWidth}, ${r.imageHeight}],\n")
        sb.append("  \"fx\": ${r.fx},\n  \"fy\": ${r.fy},\n  \"cx\": ${r.cx},\n  \"cy\": ${r.cy},\n")
        sb.append("  \"dist\": {\n")
        sb.append("    \"k1\": ${r.dist[0]},\n    \"k2\": ${r.dist[1]},\n")
        sb.append("    \"p1\": ${r.dist[2]},\n    \"p2\": ${r.dist[3]},\n")
        sb.append("    \"k3\": ${r.dist[4]},\n    \"k4\": ${r.dist[5]},\n")
        sb.append("    \"k5\": ${r.dist[6]},\n    \"k6\": ${r.dist[7]}\n")
        sb.append("  },\n")
        sb.append("  \"rms_reprojection_error_px\": ${r.rms},\n")
        sb.append("  \"image_count\": ${r.imageCount},\n")
        sb.append("  \"checkerboard\": {\"cols\": $CHECKER_COLS, \"rows\": $CHECKER_ROWS, \"square_mm\": $squareMm},\n")
        sb.append("  \"captured_at\": \"$iso\",\n")
        sb.append("  \"source\": \"in_app\"\n")
        sb.append("}\n")
        calibrationFile(context).writeText(sb.toString())
        true
    } catch (e: Exception) {
        Log.e(CTAG, "save failed", e); false
    }
}
