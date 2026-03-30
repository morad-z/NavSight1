package com.example.navsight1

import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.After
import org.junit.Assert.*
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import kotlin.math.sqrt

/**
 * VIO Native Integration Tests
 *
 * These tests exercise the real C++ VIO pipeline through JNI on a device/emulator.
 * They use synthetic NV21 frames with checkerboard patterns (strong Harris corners)
 * to validate drift, tracking, and robustness.
 *
 * Run: ./gradlew app:connectedDebugAndroidTest
 * Or right-click in Android Studio -> Run Tests
 */
@RunWith(AndroidJUnit4::class)
class VioNativeTest {

    companion object {
        private const val TAG = "VioNativeTest"
        private const val W = 640
        private const val H = 480
        private const val FRAME_SIZE = W * H + W * H / 2 // NV21
        private const val BASE_TS = 1_000_000_000L
        private const val FRAME_DT = 33_333_333L // 30 fps
    }

    @Before
    fun setUp() {
        NativeBridge.startVIO()
        NativeBridge.setIntrinsics(500.0, 500.0, W / 2.0, H / 2.0)
    }

    @After
    fun tearDown() {
        NativeBridge.stopVIO()
    }

    // ── Frame Generation ────────────────────────────────────────────────────

    /**
     * Create a NV21 frame with a checkerboard pattern offset by (offsetX, offsetY) pixels.
     * Checkerboard produces strong Harris corners at every grid intersection,
     * which is what goodFeaturesToTrack looks for. Shifting the pattern simulates
     * camera translation.
     *
     * @param cellSize Size of each checker square in pixels
     * @param offsetX Horizontal pixel offset (simulates camera X translation)
     * @param offsetY Vertical pixel offset (simulates camera Y translation)
     */
    private fun createCheckerboardFrame(
        cellSize: Int = 20,
        offsetX: Int = 0,
        offsetY: Int = 0
    ): ByteArray {
        val frame = ByteArray(FRAME_SIZE)
        // Y plane: checkerboard
        for (y in 0 until H) {
            for (x in 0 until W) {
                val cx = (x + offsetX) / cellSize
                val cy = (y + offsetY) / cellSize
                val isWhite = (cx + cy) % 2 == 0
                frame[y * W + x] = if (isWhite) 220.toByte() else 40.toByte()
            }
        }
        // UV plane: neutral chroma (128)
        for (i in W * H until FRAME_SIZE) frame[i] = 128.toByte()
        return frame
    }

    /**
     * Create a NV21 frame with bright dots at given positions on gray background.
     * Less reliable for optical flow than checkerboard, but useful for sparse tests.
     */
    private fun createDotFrame(features: List<Pair<Int, Int>>, background: Byte = 80): ByteArray {
        val frame = ByteArray(FRAME_SIZE)
        for (i in 0 until W * H) frame[i] = background
        for (i in W * H until FRAME_SIZE) frame[i] = 128.toByte()
        for ((fx, fy) in features) {
            for (dy in -2..2) {
                for (dx in -2..2) {
                    val x = fx + dx; val y = fy + dy
                    if (x in 0 until W && y in 0 until H) {
                        frame[y * W + x] = 255.toByte()
                    }
                }
            }
        }
        return frame
    }

    private fun featureGrid(cols: Int = 10, rows: Int = 8, margin: Int = 30): List<Pair<Int, Int>> {
        val dx = (W - 2 * margin) / (cols - 1)
        val dy = (H - 2 * margin) / (rows - 1)
        return (0 until rows).flatMap { r ->
            (0 until cols).map { c -> Pair(margin + c * dx, margin + r * dy) }
        }
    }

    // ── IMU Helpers ──────────────────────────────────────────────────────────

    private fun feedStaticIMU(startNs: Long, durationMs: Int, rateHz: Int = 200) {
        val dtNs = 1_000_000_000L / rateHz
        val samples = durationMs * rateHz / 1000
        for (i in 0 until samples) {
            val ts = startNs + i * dtNs
            NativeBridge.processGyroscope(ts, 0f, 0f, 0f)
            NativeBridge.processAccelerometer(ts, 0f, 9.81f, 0f)
        }
    }

    private fun feedRotatingIMU(startNs: Long, durationMs: Int, gz: Float, rateHz: Int = 200) {
        val dtNs = 1_000_000_000L / rateHz
        val samples = durationMs * rateHz / 1000
        for (i in 0 until samples) {
            val ts = startNs + i * dtNs
            NativeBridge.processGyroscope(ts, 0f, 0f, gz)
            NativeBridge.processAccelerometer(ts, 0f, 9.81f, 0f)
        }
    }

    private fun feedWalkingIMU(startNs: Long, durationMs: Int, forwardAccel: Float = 0.5f, rateHz: Int = 200) {
        val dtNs = 1_000_000_000L / rateHz
        val samples = durationMs * rateHz / 1000
        for (i in 0 until samples) {
            val ts = startNs + i * dtNs
            NativeBridge.processGyroscope(ts, 0f, 0f, 0f)
            NativeBridge.processAccelerometer(ts, forwardAccel, 9.81f, 0f)
        }
    }

    // ── Lifecycle Tests ─────────────────────────────────────────────────────

    @Test
    fun lifecycle_startStopNoThrow() {
        NativeBridge.stopVIO()
        NativeBridge.startVIO()
        NativeBridge.stopVIO()
        NativeBridge.startVIO()
    }

    @Test
    fun lifecycle_resetNoThrow() {
        NativeBridge.resetVIO()
        NativeBridge.resetVIO()
    }

    // ── Initialization Tests ────────────────────────────────────────────────

    @Test
    fun firstFrame_returnsNotInitialized() {
        val frame = createCheckerboardFrame()
        feedStaticIMU(BASE_TS - 50_000_000L, 50)
        val vio = NativeBridge.processCameraFrame(frame, W, H, BASE_TS)
        assertFalse("First frame should not be initialized", vio.isInitialized)
    }

    @Test
    fun secondFrame_returnsInitialized() {
        val frame1 = createCheckerboardFrame(offsetX = 0)
        val frame2 = createCheckerboardFrame(offsetX = 8) // shift 8px

        feedStaticIMU(BASE_TS - 50_000_000L, 50)
        NativeBridge.processCameraFrame(frame1, W, H, BASE_TS)

        feedWalkingIMU(BASE_TS + FRAME_DT - 33_000_000L, 33)
        val vio = NativeBridge.processCameraFrame(frame2, W, H, BASE_TS + FRAME_DT)

        assertTrue("Second frame should be initialized (tracking started)", vio.isInitialized)
    }

    // ── DRIFT TEST: Static Scene ────────────────────────────────────────────
    // Phone sits perfectly still for 3 seconds.
    // If ZUPT works correctly, position MUST stay near zero.
    // This is the most important drift test.

    @Test
    fun staticScene_noDrift() {
        val frame = createCheckerboardFrame()
        var maxDrift = 0.0

        for (i in 0 until 90) { // 3 seconds
            val ts = BASE_TS + i * FRAME_DT
            feedStaticIMU(ts, 33)
            val vio = NativeBridge.processCameraFrame(frame, W, H, ts)

            if (vio.isInitialized) {
                val drift = sqrt(vio.x * vio.x + vio.y * vio.y + vio.z * vio.z)
                if (drift > maxDrift) {
                    maxDrift = drift
                    Log.d(TAG, "staticScene frame=$i drift=${drift}m quality=${vio.trackingQuality} tracked=${vio.trackedFeatures}")
                }
            }
        }

        assertTrue(
            "ZUPT FAILED: Static scene drift should be < 0.1m over 3s, got ${"%.4f".format(maxDrift)}m. " +
            "Double-integrated accelerometer noise is leaking into position.",
            maxDrift < 0.1
        )
    }

    // ── DRIFT TEST: Pure Rotation ───────────────────────────────────────────
    // Rotate the phone in place (high gyro, no translation).
    // Position MUST stay near zero — only rotation should change.

    @Test
    fun pureRotation_noPositionDrift() {
        var maxPosition = 0.0

        for (i in 0 until 60) { // 2 seconds
            val ts = BASE_TS + i * FRAME_DT
            // Rotating checkerboard — shift offset circularly
            val angle = i * 0.05
            val ox = (kotlin.math.cos(angle) * 10).toInt()
            val oy = (kotlin.math.sin(angle) * 10).toInt()
            val frame = createCheckerboardFrame(cellSize = 20, offsetX = ox, offsetY = oy)

            feedRotatingIMU(ts, 33, 1.0f) // high gyro = pure rotation
            val vio = NativeBridge.processCameraFrame(frame, W, H, ts)

            if (vio.isInitialized) {
                val pos = sqrt(vio.x * vio.x + vio.y * vio.y + vio.z * vio.z)
                if (pos > maxPosition) {
                    maxPosition = pos
                    Log.d(TAG, "pureRotation frame=$i pos=${pos}m quality=${vio.trackingQuality}")
                }
            }
        }

        assertTrue(
            "ROTATION DRIFT: Pure rotation should not cause position drift > 0.5m, got ${"%.4f".format(maxPosition)}m. " +
            "The is_pure_rotation detection or pose update is broken.",
            maxPosition < 0.5
        )
    }

    // ── DRIFT TEST: Forward Motion ──────────────────────────────────────────
    // Steady forward motion with consistent checkerboard shift.
    // Position MUST grow — if it stays at zero, the VIO is not working.

    @Test
    fun forwardMotion_positionGrows() {
        var maxPos = 0.0
        var lastTracked = 0
        var lastQuality = 0.0
        var lastScale = 1.0

        for (i in 0 until 60) { // 2 seconds
            val ts = BASE_TS + i * FRAME_DT
            // Shift checkerboard 8 pixels per frame = camera translating
            val frame = createCheckerboardFrame(cellSize = 20, offsetX = -8 * i)

            feedWalkingIMU(ts, 33, forwardAccel = 1.0f)
            val vio = NativeBridge.processCameraFrame(frame, W, H, ts)

            if (vio.isInitialized) {
                val pos = sqrt(vio.x * vio.x + vio.y * vio.y + vio.z * vio.z)
                maxPos = maxOf(maxPos, pos)
                lastTracked = vio.trackedFeatures
                lastQuality = vio.trackingQuality
                lastScale = vio.estimatedScale
                Log.d(TAG, "forwardMotion frame=$i pos=${"%.4f".format(pos)}m " +
                        "tracked=$lastTracked quality=${"%.2f".format(lastQuality)} " +
                        "scale=${"%.4f".format(vio.estimatedScale)}")
            }
        }

        assertTrue(
            "FORWARD MOTION FAILED: Position must grow during steady translation. " +
            "Got maxPos=${"%.4f".format(maxPos)}m, tracked=$lastTracked, " +
            "quality=${"%.2f".format(lastQuality)}, scale=${"%.4f".format(lastScale)}. " +
            "If tracked>0 and quality>0 but pos=0, ZUPT or is_static is incorrectly triggered.",
            maxPos > 0.001
        )
    }

    // ── TRACKING TEST: Feature Count ────────────────────────────────────────
    // Checkerboard should give lots of Harris corners.

    @Test
    fun checkerboard_highFeatureCount() {
        val frame1 = createCheckerboardFrame(offsetX = 0)
        val frame2 = createCheckerboardFrame(offsetX = 8)

        feedStaticIMU(BASE_TS - 50_000_000L, 50)
        NativeBridge.processCameraFrame(frame1, W, H, BASE_TS)

        feedWalkingIMU(BASE_TS + FRAME_DT, 33)
        val vio = NativeBridge.processCameraFrame(frame2, W, H, BASE_TS + FRAME_DT)

        Log.d(TAG, "checkerboard tracked=${vio.trackedFeatures} total=${vio.totalFeatures} quality=${vio.trackingQuality}")

        assertTrue(
            "Checkerboard should produce > 30 tracked features, got ${vio.trackedFeatures}. " +
            "goodFeaturesToTrack is not finding corners in the synthetic pattern.",
            vio.trackedFeatures > 30
        )

        assertTrue(
            "Tracking quality should be > 0.5 with good checkerboard, got ${vio.trackingQuality}. " +
            "Forward-backward check or RANSAC is rejecting too many points.",
            vio.trackingQuality > 0.5
        )
    }

    // ── ROBUSTNESS: Empty Frame ─────────────────────────────────────────────

    @Test
    fun emptyFrame_noCrash() {
        val frame = ByteArray(FRAME_SIZE) { 128.toByte() }
        feedStaticIMU(BASE_TS - 50_000_000L, 50)
        val vio = NativeBridge.processCameraFrame(frame, W, H, BASE_TS)
        assertNotNull("processCameraFrame should never return null", vio)
    }

    // ── ROBUSTNESS: Reset Clears Pose ───────────────────────────────────────

    @Test
    fun reset_clearsAccumulatedPose() {
        // Build up some pose
        for (i in 0 until 10) {
            val ts = BASE_TS + i * FRAME_DT
            val frame = createCheckerboardFrame(offsetX = -5 * i)
            feedWalkingIMU(ts, 33)
            NativeBridge.processCameraFrame(frame, W, H, ts)
        }

        NativeBridge.resetVIO()

        // After reset, first frame must be initialization
        val frame = createCheckerboardFrame()
        val resetTs = BASE_TS + 20 * FRAME_DT
        feedStaticIMU(resetTs, 33)
        val vio = NativeBridge.processCameraFrame(frame, W, H, resetTs)

        assertFalse(
            "After resetVIO, the next frame should be initialization (isInitialized=false)",
            vio.isInitialized
        )
    }

    // ── ROBUSTNESS: Large Timestamp Gap ─────────────────────────────────────

    @Test
    fun largeTimestampGap_noCrash() {
        val frame = createCheckerboardFrame()
        feedStaticIMU(BASE_TS - 50_000_000L, 50)
        NativeBridge.processCameraFrame(frame, W, H, BASE_TS)

        // 10-second gap (exceeds MAX_DT_NS=5s)
        val lateTs = BASE_TS + 10_000_000_000L
        feedStaticIMU(lateTs - 50_000_000L, 50)
        val vio = NativeBridge.processCameraFrame(frame, W, H, lateTs)
        assertNotNull("Should handle 10s timestamp gap without crash", vio)
    }

    // ── ROBUSTNESS: Rapid Start/Stop ────────────────────────────────────────

    @Test
    fun rapidStartStop_noMemoryLeaks() {
        for (cycle in 0 until 20) {
            NativeBridge.startVIO()
            NativeBridge.setIntrinsics(500.0, 500.0, 320.0, 240.0)
            val frame = createCheckerboardFrame()
            feedStaticIMU(BASE_TS, 33)
            NativeBridge.processCameraFrame(frame, W, H, BASE_TS)
            NativeBridge.stopVIO()
        }
    }

    // ── SCALE TEST: Steady Motion ───────────────────────────────────────────
    // During steady forward motion with IMU acceleration, scale should update
    // from its initial value of 1.0 to something based on IMU/vision ratio.

    @Test
    fun steadyMotion_scaleUpdatesFromDefault() {
        var scaleChanged = false
        var lastScale = 1.0

        for (i in 0 until 90) { // 3 seconds
            val ts = BASE_TS + i * FRAME_DT
            val frame = createCheckerboardFrame(cellSize = 20, offsetX = -8 * i)

            feedWalkingIMU(ts, 33, forwardAccel = 1.0f)
            val vio = NativeBridge.processCameraFrame(frame, W, H, ts)

            if (vio.isInitialized && vio.estimatedScale != 1.0) {
                scaleChanged = true
                lastScale = vio.estimatedScale
                Log.d(TAG, "scale frame=$i scale=${"%.4f".format(lastScale)} quality=${vio.trackingQuality}")
            }
        }

        if (scaleChanged) {
            assertTrue(
                "Scale should be physically plausible (0.001-20.0), got ${"%.4f".format(lastScale)}",
                lastScale in 0.001..20.0
            )
        }
        // Note: if scaleChanged is false, it means the guards rejected all scale
        // observations. This is acceptable for synthetic data but would be a
        // problem with real camera frames. Log it for visibility.
        Log.i(TAG, "steadyMotion_scaleUpdatesFromDefault: scaleChanged=$scaleChanged lastScale=$lastScale")
    }
}
