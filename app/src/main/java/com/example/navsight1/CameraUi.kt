package com.example.navsight1

import android.util.Log
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.AspectRatioStrategy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.compose.animation.*
import androidx.compose.animation.core.tween
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
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
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.camera.camera2.interop.Camera2Interop
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import android.hardware.camera2.CaptureResult

private const val TAG = "NavSight"

@Composable
fun CameraViewComposable(viewModel: NavSightViewModel) {
    val lifecycleOwner = LocalLifecycleOwner.current
    // Audit Finding 13 (2026-05-16): Executors.newSingleThreadExecutor() created
    // inline in the AndroidView factory was never shut down. The thread is
    // non-daemon and keeps a reference to the captured lambda (including viewModel),
    // blocking GC after the composable is removed.
    // Fix: remember the executor so it survives recomposition, and use
    // DisposableEffect to shut it down when the composable leaves composition.
    val analyzerExecutor = remember {
        java.util.concurrent.Executors.newSingleThreadExecutor { r ->
            Thread(r, "NavSight-CameraAnalyzer").apply { isDaemon = true }
        }
    }
    DisposableEffect(Unit) {
        onDispose {
            Log.d(TAG, "CameraViewComposable: shutting down analyzerExecutor (Finding 13 fix)")
            analyzerExecutor.shutdown()
        }
    }
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
                // ResolutionSelector with strict 4:3 + 640×480 target. Must
                // match the calibration screen's selector so intrinsics
                // saved during calibration apply 1:1 at runtime. Replaces
                // the deprecated setTargetResolution which let some
                // devices deliver a square 1088×1088 crop.
                val resSelector = ResolutionSelector.Builder()
                    .setAspectRatioStrategy(AspectRatioStrategy.RATIO_4_3_FALLBACK_AUTO_STRATEGY)
                    .setResolutionStrategy(
                        ResolutionStrategy(
                            android.util.Size(640, 480),
                            ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER,
                        )
                    )
                    .build()
                // Step 8c: build ImageAnalysis with Camera2Interop so we can
                // attach a CaptureCallback and read SENSOR_ROLLING_SHUTTER_SKEW
                // (Camera2 API, API level 21+) on every captured frame.
                @OptIn(ExperimentalCamera2Interop::class)
                val analysisBuilder = ImageAnalysis.Builder()
                    .setResolutionSelector(resSelector)
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)

                @OptIn(ExperimentalCamera2Interop::class)
                Camera2Interop.Extender(analysisBuilder)
                    // 2026-05-09 v21 (Phase 1 Step 2c): lock the camera AE
                    // algorithm to a stable [30, 30] fps range. Without this
                    // CameraX defaults to a variable rate (commonly [15, 30]
                    // or [10, 60] depending on lighting) which complicates
                    // rolling-shutter row-time interpolation (Step 8c) and
                    // makes the IMU preintegrator's per-frame Δt more
                    // variable than the 1/fps it expects. 30 fps = the
                    // analysis target rate; locking both ends prevents
                    // the AE algorithm from dropping it under low light.
                    // Source: Agent 2 dependency audit, docs/study/dependency_audit.md §A7.
                    .setCaptureRequestOption(
                        android.hardware.camera2.CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
                        android.util.Range(30, 30)
                    )
                    .setSessionCaptureCallback(
                        object : android.hardware.camera2.CameraCaptureSession.CaptureCallback() {
                            override fun onCaptureCompleted(
                                session: android.hardware.camera2.CameraCaptureSession,
                                request: android.hardware.camera2.CaptureRequest,
                                result: android.hardware.camera2.TotalCaptureResult
                            ) {
                                // CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW: nanoseconds
                                // from first-row to last-row read-out. Null on global-
                                // shutter devices or when the key is not supported.
                                val skew = result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW)
                                if (skew != null) {
                                    viewModel.updateRollingShutterSkew(skew)
                                }
                            }
                        }
                    )

                val analysis = analysisBuilder.build().also { ia ->
                    ia.setAnalyzer(analyzerExecutor) { img ->
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
    historySnapshot: List<PathPoint>,
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

// ── Phase 1 camera overlay: KLT tracked feature points ─────────────────────
//
// `VioData.trackedPoints` is a flat float array `[x0,y0, x1,y1, ...]` in
// analyzer-native pixel coords (typically 640×480 landscape, sourced from
// `next_good_buf_` at Tracker.cpp:1026-1031). To draw them on top of the
// PreviewView we must:
//
//   1. Rotate analyzer coords → upright preview coords by
//      `image.imageInfo.rotationDegrees` (CameraX makes the analyzer image
//      upright on the preview surface; we replicate the same rotation).
//   2. Replicate `PreviewView.ScaleType.FILL_CENTER` (declared at
//      `CameraViewComposable` line 48) — uniform scale by max(sx, sy) so the
//      smaller dimension fills the view exactly while the larger overflows
//      symmetrically and is cropped.
//   3. Draw filled circles in Compose Canvas pixels.
//
// The same math lives in `CalibrationScreenUi.kt` for the chessboard
// overlay; we keep file-local copies (10 lines total) to avoid a refactor
// for two callers.

// Phase 2 KLT age palette (camera_overlay_phase23_plan.md §C.4).
// At 30 Hz, 30 frames = 1 s, 90 frames = 3 s.
private const val OVERLAY_AGE_NEW_FRAMES        = 30        // < 1 s
private const val OVERLAY_AGE_ESTABLISHED_FRAMES = 90       // 1-3 s
private val OVERLAY_AGE_NEW_COLOR         = Color(0xD953D34D)   // green @85%
private val OVERLAY_AGE_ESTABLISHED_COLOR = Color(0xD9FFEB3B)   // yellow @85%
private val OVERLAY_AGE_MATURE_COLOR      = Color(0xD9EF5350)   // red @85%

// Phase 3 SLAM dot styling.
private val OVERLAY_SLAM_FILL    = Color(0xD9FF6D00)   // orange @85%
private val OVERLAY_SLAM_RING    = Color(0xCCFFFFFF)   // white  @80%
private const val OVERLAY_SLAM_RADIUS_PX = 8f
private const val OVERLAY_SLAM_RING_RADIUS_PX = 9f
private const val OVERLAY_SLAM_RING_STROKE_PX = 1.5f
// Minimum projected depth (m) for a stable dot. Derivation:
// at fx ~500 px, the focal/depth gain is 500/0.05 = 10 000 px/m.
// 1° EKF rotation noise ≈ 0.0175 rad pivots a 5 cm-deep point by
// ~5 cm × tan(1°) = 0.87 mm, projecting to ~17 px on screen. Anything
// closer than 5 cm flickers visibly; anything farther stays sub-pixel.
private const val OVERLAY_SLAM_MIN_DEPTH_M = 0.05f

// ── Phase 1 Step 6 (post_v19_sprint_plan.md §205-298): LandmarkMap overlay ───
// Static landmark dots rendered BEHIND the live SLAM dots. Two colour states:
//   * observed_this_frame == 1 → bright orange filled circle (recently matched)
//   * observed_this_frame == 0 → dim gray   filled circle (in DB, not in view)
// Spec: orange (255,160,0) @ alpha 0.95 with 5 px radius for observed;
// gray (128,128,128) @ alpha 0.50 with 4 px radius for dormant. These are the
// fixed Phase 6.5 colours — they intentionally differ from the live SLAM
// orange (OVERLAY_SLAM_FILL above, ~FF6D00 darker) so the user can distinguish
// "persistent map point" (this overlay) from "EKF SLAM feature" (the layer
// drawn on top of us).
private val OVERLAY_LANDMARK_OBSERVED_COLOR = Color(0xF2FFA000)  // RGB 255,160,0 @95%
private val OVERLAY_LANDMARK_DORMANT_COLOR  = Color(0x80808080)  // RGB 128,128,128 @50%
private const val OVERLAY_LANDMARK_OBSERVED_RADIUS_PX = 5f
private const val OVERLAY_LANDMARK_DORMANT_RADIUS_PX  = 4f
// Stride of the JNI float array returned by NativeBridge.getLandmarkSnapshot.
// See native-lib.cpp Java_..._getLandmarkSnapshot for the layout:
//   [0]=id, [1]=x_yup, [2]=y_yup, [3]=z_yup, [4]=observed_this_frame,
//   [5]=obs_u, [6]=obs_v.
// 2026-05-19 Fix #3 (orange-dot anchor): added obs_u/obs_v so observed
// landmarks render AT the actual KLT-matched image pixel instead of
// projecting through a possibly-drifted EKF camera pose. obs_u = obs_v = 0
// when the landmark was not observed this frame OR the Tracker pixel
// lookup raced; the projection path remains the fallback in that case.
private const val OVERLAY_LANDMARK_STRIDE = 7
// Minimum depth (m) — match OVERLAY_SLAM_MIN_DEPTH_M so the two layers
// disappear together as features pass behind the camera.

private fun ageToColor(ageFrames: Int): Color = when {
    ageFrames < OVERLAY_AGE_NEW_FRAMES         -> OVERLAY_AGE_NEW_COLOR
    ageFrames < OVERLAY_AGE_ESTABLISHED_FRAMES -> OVERLAY_AGE_ESTABLISHED_COLOR
    else                                        -> OVERLAY_AGE_MATURE_COLOR
}

@Composable
fun CameraFeatureOverlay(viewModel: NavSightViewModel, pal: NavPalette) {
    // Camera overlay Phase 1 LAG FIX (camera_overlay_phase23_plan.md §A):
    // read from the per-frame `overlaySnapshot` instead of the throttled
    // `vioState`. The snapshot updates at the full ~30 Hz native frame
    // rate so dots track the live preview with no perceptible lag.
    val snap = viewModel.overlaySnapshot
    val geom = viewModel.cameraFrameGeometry
    if (geom == null) return
    val pts = snap.trackedPoints
    if (pts.isEmpty()) return
    val ages = snap.trackedAges
    val n = pts.size / 2
    // Phase 2 age colouring is enabled iff ages parallel-pair with pts.
    // Anytime they don't (e.g. the JNI getter race lost a frame) we
    // fall back to the original teal so dots stay visible.
    val useAgeColors = ages.size == n
    val fallbackColor = pal.teal.copy(alpha = 0.85f)
    val dotRadius = 6f

    Canvas(Modifier.fillMaxSize()) {
        val (prevW, prevH) = overlayRotatedDims(
            geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
        )
        if (prevW <= 0 || prevH <= 0) return@Canvas

        val viewW = size.width
        val viewH = size.height
        val sx = viewW / prevW.toFloat()
        val sy = viewH / prevH.toFloat()
        val s = maxOf(sx, sy)                       // FILL_CENTER
        val ox = (viewW - prevW * s) / 2f
        val oy = (viewH - prevH * s) / 2f

        for (i in 0 until n) {
            val ax = pts[2 * i]
            val ay = pts[2 * i + 1]
            val r = overlayRotatePoint(
                ax, ay, geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
            )
            val cx = ox + r.x * s
            val cy = oy + r.y * s
            val color = if (useAgeColors) ageToColor(ages[i]) else fallbackColor
            drawCircle(color = color, radius = dotRadius, center = Offset(cx, cy))
        }
    }
}

/**
 * Camera overlay Phase 6.5 (post_v19_sprint_plan.md §205-298): persistent
 * 3D LandmarkMap overlay. Renders the visual-only Landmark database
 * (populated each keyframe by `Tracker::addOrMergeLandmark`, persisted
 * across loop closures via `LandmarkMap::applyKeyframePoseCorrection`).
 *
 * Two colour states per landmark:
 *   * observed_this_frame == 1.0 → bright orange filled circle (the EKF
 *     just matched this landmark in the most recent keyframe — confirming
 *     the map at this physical location).
 *   * observed_this_frame == 0.0 → dim gray filled circle (in the DB but
 *     not seen this frame — the map remembers it from earlier passes).
 *
 * Draw order (MapScreenUi): LandmarkOverlay → SlamFeatureOverlay so the
 * live EKF features render ABOVE the static landmark layer. This is the
 * design contract — flipping the order makes orange landmarks occlude the
 * live SLAM-orange dots and breaks the "live > static" hierarchy.
 *
 * Projection math: identical to [SlamFeatureOverlay] (snapshot.cameraPose
 * carries R_world_cam + t_world_cam + intrinsics in Y-up world). The JNI
 * returns landmark world coords in the SAME Y-up frame as SlamSnapshot —
 * verified at the JNI boundary via the `JNI_LM_BOUNDARY:` log line on the
 * first frame.
 */
@Composable
fun LandmarkOverlay(viewModel: NavSightViewModel, pal: NavPalette) {
    // Gate on the camera-pose snapshot (16 floats) — same gate
    // [SlamFeatureOverlay] uses. When the EKF is not yet full-initialised
    // the pose array is empty and we have no way to project the
    // world-anchored landmarks onto the camera plane. This also implicitly
    // requires VIO to be initialised, mirroring how
    // `SensorRepository.wasVioInitialized` gates the rest of the overlay.
    val snap = viewModel.overlaySnapshot
    val geom = viewModel.cameraFrameGeometry
    if (geom == null) return
    val pose = snap.cameraPose
    if (pose.size < 16) return

    // Pull a fresh landmark snapshot from JNI. The buffer size scales with
    // map population (capped at 500 landmarks × 5 floats = 2500 entries in
    // the JNI getter — see native-lib.cpp::getLandmarkSnapshot). The Kotlin-
    // side allocation is unavoidable given the JNI contract returns a new
    // FloatArray each call; the 500-cap keeps it bounded.
    val landmarks = NativeBridge.getLandmarkSnapshot()

    // Diagnostic: when the snapshot is empty for too long after VIO init we
    // expect the user to want to see why. Throttle to ≤ 1 line/sec to keep
    // logcat readable.
    val throttle = remember { LandmarkOverlayThrottle() }
    if (landmarks.isEmpty()) {
        val now = System.currentTimeMillis()
        if (now - throttle.lastWarnMs >= 1000L) {
            throttle.lastWarnMs = now
            Log.w(TAG, "LM_OVERLAY: snapshot empty (camera_pose_valid=${pose.size >= 16})")
        }
        return
    }
    val n = landmarks.size / OVERLAY_LANDMARK_STRIDE
    if (n <= 0) return

    // Decode the pose matrix once outside the per-feature loop — identical
    // to [SlamFeatureOverlay]. Layout (native-lib.cpp getCurrentCameraPose):
    //   [0..8]   R_world_cam row-major (camera→world)
    //   [9..11]  t_world_cam (camera position in Y-up world)
    //   [12..15] fx, fy, cx, cy
    val r0 = pose[0]; val r1 = pose[1]; val r2 = pose[2]
    val r3 = pose[3]; val r4 = pose[4]; val r5 = pose[5]
    val r6 = pose[6]; val r7 = pose[7]; val r8 = pose[8]
    val tx = pose[9]; val ty = pose[10]; val tz = pose[11]
    val fx = pose[12]; val fy = pose[13]
    val cxIntr = pose[14]; val cyIntr = pose[15]

    Canvas(Modifier.fillMaxSize()) {
        val (prevW, prevH) = overlayRotatedDims(
            geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
        )
        if (prevW <= 0 || prevH <= 0) return@Canvas

        val viewW = size.width
        val viewH = size.height
        val sx = viewW / prevW.toFloat()
        val sy = viewH / prevH.toFloat()
        // FILL_CENTER scale — identical pipeline as the SLAM overlay so a
        // landmark at the same scene location as an EKF SLAM feature draws
        // at the same canvas pixel.
        val s = maxOf(sx, sy)
        val ox = (viewW - prevW * s) / 2f
        val oy = (viewH - prevH * s) / 2f

        for (i in 0 until n) {
            // Stride 7: [id, wx, wy, wz, observed_flag, obs_u, obs_v]
            // val id = landmarks[OVERLAY_LANDMARK_STRIDE * i]    // unused in draw
            val wx = landmarks[OVERLAY_LANDMARK_STRIDE * i + 1]
            val wy = landmarks[OVERLAY_LANDMARK_STRIDE * i + 2]
            val wz = landmarks[OVERLAY_LANDMARK_STRIDE * i + 3]
            val observed = landmarks[OVERLAY_LANDMARK_STRIDE * i + 4] > 0.5f
            val obsU = landmarks[OVERLAY_LANDMARK_STRIDE * i + 5]
            val obsV = landmarks[OVERLAY_LANDMARK_STRIDE * i + 6]

            // 2026-05-19 Fix #3 — observed-dot direct-pixel render.
            //
            // If the landmark was matched against a current-frame KLT
            // keypoint, `obsU/obsV` IS that keypoint's pixel — the actual
            // image-feature location THIS frame. Drawing there anchors the
            // dot by construction (the dot IS the image feature). The
            // projection chain (R_world_cam, t, fx/fy/cx/cy) is skipped
            // entirely because projection through a drifted EKF camera
            // pose produces the wrong screen position; we already know the
            // right pixel from the matcher.
            //
            // (obsU == 0 && obsV == 0) is the JNI sentinel for "no pixel
            // available" (race between Tracker's last-observed snapshot
            // and our lookup, OR the lookup function returned false).
            // Treat as unobserved and fall through to projection.
            val haveObsPixel =
                observed && (obsU != 0f || obsV != 0f)

            val u: Float
            val v: Float
            if (haveObsPixel) {
                u = obsU
                v = obsV
            } else {
                // Fall-back: project p_world through current camera pose.
                // p_cam = R_world_cam.t() · (p_world - t_world)
                // Row-major R_world_cam transpose-times-vector:
                //   out[r] = sum_c R[c, r] * v[c]
                val dx = wx - tx
                val dy = wy - ty
                val dz = wz - tz
                val xc = r0 * dx + r3 * dy + r6 * dz
                val yc = r1 * dx + r4 * dy + r7 * dz
                val zc = r2 * dx + r5 * dy + r8 * dz
                if (zc <= OVERLAY_SLAM_MIN_DEPTH_M) continue
                u = fx * xc / zc + cxIntr
                v = fy * yc / zc + cyIntr
            }

            // Apply identical analyzer→canvas transform as KLT / SLAM overlays.
            val rot = overlayRotatePoint(
                u, v, geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
            )
            val cx = ox + rot.x * s
            val cy = oy + rot.y * s

            // Cull obviously off-screen draws. Compose's drawCircle will
            // happily clip, but skipping early avoids per-pixel raster cost
            // when the map carries hundreds of landmarks behind the camera.
            if (cx < -16f || cx > viewW + 16f || cy < -16f || cy > viewH + 16f) continue

            if (observed) {
                drawCircle(
                    color = OVERLAY_LANDMARK_OBSERVED_COLOR,
                    radius = OVERLAY_LANDMARK_OBSERVED_RADIUS_PX,
                    center = Offset(cx, cy)
                )
            } else {
                drawCircle(
                    color = OVERLAY_LANDMARK_DORMANT_COLOR,
                    radius = OVERLAY_LANDMARK_DORMANT_RADIUS_PX,
                    center = Offset(cx, cy)
                )
            }
        }
    }
}

// Per-Composable throttle holder for the "snapshot empty" warning. Kept as
// a plain class (not a data class) so identity-equality lets the
// `remember {}` survive across recompositions without unintentional resets.
private class LandmarkOverlayThrottle {
    var lastWarnMs: Long = 0L
}

/**
 * Camera overlay Phase 3 (camera_overlay_phase23_plan.md §B):
 * world-anchored 3D SLAM points reprojected through the current camera
 * pose. Unlike KLT teal/coloured dots — which die when KLT loses the
 * feature — these dots stay locked to the same physical 3D point in the
 * scene. Pan away and back: the same orange-with-white-ring dot reappears
 * at the same spot.
 *
 * Math (per feature):
 *   p_cam = R_world_cam.t() · (p_world - t_world_cam)
 *   if (p_cam.z <= MIN_DEPTH_M) skip   // behind / too close to camera
 *   u = fx · p_cam.x / p_cam.z + cx
 *   v = fy · p_cam.y / p_cam.z + cy
 *   then apply analyzer→canvas rotation + FILL_CENTER scale-and-crop
 *
 * The FILL_CENTER math is identical to [CameraFeatureOverlay] so a SLAM
 * point at the same scene location as a KLT dot draws at the same canvas
 * pixel. Verifying that visually is the easiest acceptance test.
 */
@Composable
fun SlamFeatureOverlay(viewModel: NavSightViewModel, pal: NavPalette) {
    val snap = viewModel.overlaySnapshot
    val geom = viewModel.cameraFrameGeometry
    if (geom == null) return
    val slam = snap.slamSnapshot
    if (slam.size < 4) return                    // need at least 1 feature
    val pose = snap.cameraPose
    if (pose.size < 16) return                   // EKF not yet full-init

    // Decode pose matrix once outside the per-feature loop.
    // Layout (see native-lib.cpp Java_..._getCurrentCameraPose):
    //   [0..8]   R_world_cam row-major (camera→world)
    //   [9..11]  t_world_cam (camera position in Y-up world)
    //   [12..15] fx, fy, cx, cy
    val r0 = pose[0]; val r1 = pose[1]; val r2 = pose[2]
    val r3 = pose[3]; val r4 = pose[4]; val r5 = pose[5]
    val r6 = pose[6]; val r7 = pose[7]; val r8 = pose[8]
    val tx = pose[9]; val ty = pose[10]; val tz = pose[11]
    val fx = pose[12]; val fy = pose[13]
    val cxIntr = pose[14]; val cyIntr = pose[15]

    Canvas(Modifier.fillMaxSize()) {
        val (prevW, prevH) = overlayRotatedDims(
            geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
        )
        if (prevW <= 0 || prevH <= 0) return@Canvas

        val viewW = size.width
        val viewH = size.height
        val sx = viewW / prevW.toFloat()
        val sy = viewH / prevH.toFloat()
        val s = maxOf(sx, sy)                       // FILL_CENTER
        val ox = (viewW - prevW * s) / 2f
        val oy = (viewH - prevH * s) / 2f

        // v23.11: stride 7 (fid, wx, wy, wz, obs_u, obs_v, has_obs).
        val n = slam.size / 7
        for (i in 0 until n) {
            // val fid = slam[7 * i]    // feature_id — currently unused in draw
            val wx = slam[7 * i + 1]
            val wy = slam[7 * i + 2]
            val wz = slam[7 * i + 3]
            val obsU = slam[7 * i + 4]
            val obsV = slam[7 * i + 5]
            val hasObs = slam[7 * i + 6] > 0.5f

            // p_cam = R_world_cam.t() · (p_world - t_world)
            // For a row-major R_world_cam, the transpose times a vector
            // is column-wise: out[r] = sum_c R[c, r] * v[c].
            val dx = wx - tx
            val dy = wy - ty
            val dz = wz - tz
            val xc = r0 * dx + r3 * dy + r6 * dz
            val yc = r1 * dx + r4 * dy + r7 * dz
            val zc = r2 * dx + r5 * dy + r8 * dz
            if (zc <= OVERLAY_SLAM_MIN_DEPTH_M) continue

            val u = fx * xc / zc + cxIntr
            val v = fy * yc / zc + cyIntr

            // Apply identical analyzer→canvas transform as KLT overlay.
            val rotPred = overlayRotatePoint(
                u, v, geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
            )
            val cPredX = ox + rotPred.x * s
            val cPredY = oy + rotPred.y * s

            // v23.11 DIAGNOSTIC overlay: draw the obs (where KLT thinks the
            // feature actually is) as a small cyan crosshair, plus a red line
            // from obs to the EKF projection. Line length = the EKF's
            // disagreement in screen pixels. Short red line / cyan crosshair
            // ON the orange dot = healthy SLAM. Long red line / cyan offset
            // from orange = EKF projection is drifting from reality.
            // 2026-05-19 Fix #6 — pull cObsX/cObsY out of the if-block so
            // the dot-render below can reference them. Defaults to the
            // pred position when there's no obs (KLT lost the track).
            var cObsX = cPredX
            var cObsY = cPredY
            if (hasObs) {
                val rotObs = overlayRotatePoint(
                    obsU, obsV, geom.analyzerWidth, geom.analyzerHeight,
                    geom.rotationDegrees
                )
                cObsX = ox + rotObs.x * s
                cObsY = oy + rotObs.y * s
                // Red residual line (pred → obs).
                drawLine(
                    color = androidx.compose.ui.graphics.Color(0xFFFF3030),
                    start = Offset(cPredX, cPredY),
                    end = Offset(cObsX, cObsY),
                    strokeWidth = 2f
                )
                // Cyan crosshair at the obs (KLT) position.
                val crossLen = 8f
                drawLine(
                    color = androidx.compose.ui.graphics.Color(0xFF00E5FF),
                    start = Offset(cObsX - crossLen, cObsY),
                    end = Offset(cObsX + crossLen, cObsY),
                    strokeWidth = 2f
                )
                drawLine(
                    color = androidx.compose.ui.graphics.Color(0xFF00E5FF),
                    start = Offset(cObsX, cObsY - crossLen),
                    end = Offset(cObsX, cObsY + crossLen),
                    strokeWidth = 2f
                )
            }

            // 2026-05-19 Fix #6 — render the SLAM dot AT the observation
            // pixel (where KLT is currently tracking the feature in this
            // frame) rather than at the EKF projection. The projection
            // through (R_world_cam, p_G) reflects the EKF's belief, which
            // only refreshes via UpdaterSLAM at keyframe rate (~2 Hz). KLT
            // tracks the feature in real time at 30 Hz, so drawing at the
            // KLT pixel makes the dot anchor to the actual image feature
            // every frame instead of "jumping" each keyframe when the EKF
            // snaps to the live KLT obs. The red residual line (above)
            // still shows EKF projection drift as a diagnostic — short
            // line = healthy, long line = EKF state drifting between
            // keyframes (separate concern).
            //
            // hasObs=false (KLT lost the feature this frame) falls back
            // to the projection draw — there's no current observation,
            // so the EKF prediction is the best we have.
            val drawX = if (hasObs) cObsX else cPredX
            val drawY = if (hasObs) cObsY else cPredY
            drawCircle(
                color = OVERLAY_SLAM_FILL,
                radius = OVERLAY_SLAM_RADIUS_PX,
                center = Offset(drawX, drawY)
            )
            drawCircle(
                color = OVERLAY_SLAM_RING,
                radius = OVERLAY_SLAM_RING_RADIUS_PX,
                center = Offset(drawX, drawY),
                style = androidx.compose.ui.graphics.drawscope.Stroke(
                    width = OVERLAY_SLAM_RING_STROKE_PX
                )
            )
        }
    }
}

/**
 * Camera overlay Phase 4 (camera_overlay_phase23_plan.md §D): a one-second
 * "LOOP CLOSURE" banner that fades in/out whenever the EKF accepts a
 * loop-closure correction. The flash window is set in the ViewModel by
 * detecting EventCounters.loop_closure_corrections_applied increments.
 *
 * The composable triggers a recomposition loop using `LaunchedEffect` +
 * `delay()` to drive the alpha decay smoothly even though the
 * `loopClosureFlashUntilMs` field itself only changes once per increment.
 */
@Composable
fun LoopClosureFlash(viewModel: NavSightViewModel) {
    val until = viewModel.loopClosureFlashUntilMs
    if (until == 0L) return

    var nowMs by remember { mutableStateOf(System.currentTimeMillis()) }
    LaunchedEffect(until) {
        while (true) {
            val cur = System.currentTimeMillis()
            if (cur >= until) {
                nowMs = cur
                break
            }
            nowMs = cur
            kotlinx.coroutines.delay(33L)        // ~30 Hz refresh
        }
    }

    val remaining = until - nowMs
    if (remaining <= 0L) return
    val alpha = (remaining / 1000f).coerceIn(0f, 1f)

    Box(
        modifier = Modifier.fillMaxSize().statusBarsPadding(),
        contentAlignment = Alignment.TopCenter
    ) {
        Surface(
            modifier = Modifier.padding(top = 60.dp),
            color = Color(0xFFFFEB3B).copy(alpha = 0.85f * alpha),
            shape = RoundedCornerShape(12.dp),
            shadowElevation = 6.dp
        ) {
            Text(
                "LOOP CLOSURE",
                modifier = Modifier.padding(horizontal = 18.dp, vertical = 10.dp),
                color = Color(0xFF1A1A1A).copy(alpha = alpha),
                fontSize = 18.sp,
                fontWeight = FontWeight.ExtraBold,
                letterSpacing = 1.5.sp
            )
        }
    }
}

/**
 * Rotate a point from analyzer space (sensor-native landscape) to upright
 * preview space. `rot` is `ImageProxy.imageInfo.rotationDegrees`. Mirrors the
 * helper at `CalibrationScreenUi.kt:932`.
 */
private fun overlayRotatePoint(x: Float, y: Float, w: Int, h: Int, rot: Int): Offset =
    when (rot) {
        90 -> Offset(h - y, x)
        180 -> Offset(w - x, h - y)
        270 -> Offset(y, w - x)
        else -> Offset(x, y)
    }

/**
 * Analyzer dimensions rotated to upright preview orientation. For
 * `rot ∈ {90, 270}` the dims swap (sensor-native landscape → portrait).
 * Mirrors `CalibrationScreenUi.kt:940`.
 */
private fun overlayRotatedDims(w: Int, h: Int, rot: Int): Pair<Int, Int> =
    if (rot == 90 || rot == 270) Pair(h, w) else Pair(w, h)
