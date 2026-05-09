package com.example.navsight1

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Log
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.RejectedExecutionException
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong

/**
 * Step 9 / ADR-014 — captures camera frames during a simulator recording so
 * the replay harness can drive the visual half of the EKF off the recorded
 * pixels.
 *
 * Lifetime: created per recording session by [NavSightViewModel] when the
 * user taps "REC" in the debug panel; destroyed via [stop] when recording
 * ends. The output directory is `<sim_json_dir>/<basename>.frames/`, which
 * matches the convention the C++ harness probes for (`replay_harness.cpp:resolveFramesDir`).
 *
 * Threading model:
 *   * [captureFrame] is called on the CameraX analyzer thread. It must
 *     return quickly — the camera supplies frames at ~30 Hz and a slow
 *     recorder would back-pressure the entire VIO pipeline.
 *   * The Y plane bytes are copied immediately (the source ByteBuffer's
 *     ownership ends when CameraX recycles the ImageProxy on return).
 *   * The PNG encode runs on a dedicated single-thread executor so the
 *     camera thread is never blocked on `Bitmap.compress`.
 *
 * Backpressure:
 *   * A bounded queue caps in-flight encode jobs at [QUEUE_CAPACITY]. When
 *     the encoder can't keep up, [captureFrame] drops the oldest job and
 *     bumps [droppedCount]. On stop, the dropped count is logged so you
 *     can see whether the recording was lossy.
 *
 * Timestamp convention:
 *   * The filename is `<ts_ns>.png` where `ts_ns = System.currentTimeMillis() * 1_000_000`.
 *     This matches the wall-clock-ns convention the simulator JSON uses
 *     for its `ts` field (see [NavSightViewModel.SimulationPoint.timestamp]),
 *     so the harness can do an exact lookup of `<ts_ns>.png` per sample.
 *   * Note this is the WALL clock, not the monotonic Camera2 clock. The
 *     two clocks diverge by reboot drift but agree well within a single
 *     session.
 */
class SimulationFrameRecorder(
    private val outputDir: File,
    private val pngQuality: Int = PNG_QUALITY_DEFAULT,
) {

    @Volatile private var active: Boolean = false
    private val pendingCount = AtomicInteger(0)
    private val writtenCount = AtomicInteger(0)
    private val droppedCount = AtomicInteger(0)
    private val firstFrameNs = AtomicLong(-1L)
    private val lastFrameNs = AtomicLong(-1L)

    // Lazily created so a recorder that's never started doesn't burn a
    // thread. Single-thread executor — PNG encode is the bottleneck and
    // additional threads would just contend on the disk.
    private var encodeExecutor: ExecutorService? = null

    /** Begin recording. Idempotent — calling twice is a no-op (logs and ignores). */
    @Synchronized
    fun start() {
        if (active) {
            Log.w(TAG, "start() called while already active; ignoring")
            return
        }
        if (!outputDir.exists()) {
            if (!outputDir.mkdirs()) {
                Log.e(TAG, "start() failed to create $outputDir; recorder will silently drop")
                return
            }
        }
        encodeExecutor = Executors.newSingleThreadExecutor { r ->
            Thread(r, THREAD_NAME).apply { isDaemon = true }
        }
        pendingCount.set(0)
        writtenCount.set(0)
        droppedCount.set(0)
        firstFrameNs.set(-1L)
        lastFrameNs.set(-1L)
        active = true
        Log.i(TAG, "started, outputDir=${outputDir.absolutePath}")
    }

    /**
     * Submit a frame for encoding. Returns immediately. The Y-plane bytes
     * are copied before this function returns, so the caller may recycle
     * its [ImageProxy] on return.
     *
     * @param yBuffer  direct ByteBuffer with the Y plane (camera analyzer's plane[0])
     * @param width    image width in pixels
     * @param height   image height in pixels
     * @param yRowStride number of bytes between rows in [yBuffer]; ≥ width
     * @param timestampNs filename timestamp in nanoseconds (use
     *                    `System.currentTimeMillis() * 1_000_000` to align
     *                    with the simulator JSON `ts` field)
     */
    fun captureFrame(
        yBuffer: ByteBuffer,
        width: Int,
        height: Int,
        yRowStride: Int,
        timestampNs: Long,
    ) {
        if (!active) return
        if (width <= 0 || height <= 0) return

        // Capacity gate. The pendingCount IS the queue depth; we increment
        // before the executor accepts. If we can't fit, drop and bump.
        val cur = pendingCount.incrementAndGet()
        if (cur > QUEUE_CAPACITY) {
            pendingCount.decrementAndGet()
            droppedCount.incrementAndGet()
            return
        }

        // Copy Y plane to a heap byte[]. Account for row stride padding.
        val yCopy = ByteArray(width * height)
        if (yRowStride == width) {
            // Dense Y plane — single bulk copy.
            val savedPosition = yBuffer.position()
            try {
                yBuffer.position(0)
                yBuffer.get(yCopy, 0, width * height)
            } finally {
                yBuffer.position(savedPosition)
            }
        } else {
            // Stride-padded — copy row by row. ByteBuffer.get(byte[], offset, len)
            // doesn't accept arbitrary stride, so we walk each row.
            val savedPosition = yBuffer.position()
            try {
                for (row in 0 until height) {
                    yBuffer.position(row * yRowStride)
                    yBuffer.get(yCopy, row * width, width)
                }
            } finally {
                yBuffer.position(savedPosition)
            }
        }

        val executor = encodeExecutor
        if (executor == null) {
            pendingCount.decrementAndGet()
            droppedCount.incrementAndGet()
            return
        }
        try {
            executor.execute {
                try {
                    encodeAndWrite(yCopy, width, height, timestampNs)
                    writtenCount.incrementAndGet()
                    if (firstFrameNs.compareAndSet(-1L, timestampNs)) {
                        // first-frame stamp set
                    }
                    lastFrameNs.set(timestampNs)
                } catch (t: Throwable) {
                    Log.w(TAG, "encode failed for ts=$timestampNs: ${t.message}")
                    droppedCount.incrementAndGet()
                } finally {
                    pendingCount.decrementAndGet()
                }
            }
        } catch (e: RejectedExecutionException) {
            // Executor was shut down between the active check and the
            // execute() call. Roll back the pending counter so a later
            // start() sees a clean slate.
            pendingCount.decrementAndGet()
            droppedCount.incrementAndGet()
        }
    }

    /**
     * Stop recording. Drains any in-flight encodes (up to [SHUTDOWN_TIMEOUT_S] s)
     * and returns final stats. Safe to call multiple times.
     */
    @Synchronized
    fun stop(): Stats {
        if (!active) {
            return Stats(
                written = writtenCount.get(),
                dropped = droppedCount.get(),
                firstFrameNs = firstFrameNs.get(),
                lastFrameNs = lastFrameNs.get(),
                outputDir = outputDir,
            )
        }
        active = false
        val executor = encodeExecutor
        encodeExecutor = null
        if (executor != null) {
            executor.shutdown()
            val drained = executor.awaitTermination(SHUTDOWN_TIMEOUT_S, TimeUnit.SECONDS)
            if (!drained) {
                Log.w(TAG, "encoder failed to drain within ${SHUTDOWN_TIMEOUT_S}s; forcing shutdown")
                executor.shutdownNow()
                droppedCount.addAndGet(pendingCount.getAndSet(0))
            }
        }
        val stats = Stats(
            written = writtenCount.get(),
            dropped = droppedCount.get(),
            firstFrameNs = firstFrameNs.get(),
            lastFrameNs = lastFrameNs.get(),
            outputDir = outputDir,
        )
        Log.i(TAG, "stopped, $stats")
        return stats
    }

    /** Whether [captureFrame] will currently accept frames. */
    fun isActive(): Boolean = active

    private fun encodeAndWrite(yBytes: ByteArray, width: Int, height: Int, ts_ns: Long) {
        // Y → ARGB grayscale via int[]. For 640×480 this is ~300 K iterations,
        // sub-millisecond on any phone we care about. Avoids
        // Bitmap.copyPixelsFromBuffer + extra format conversion.
        val pixels = IntArray(width * height)
        for (i in 0 until width * height) {
            val y = yBytes[i].toInt() and 0xFF
            pixels[i] = (0xFF shl 24) or (y shl 16) or (y shl 8) or y
        }
        val bitmap = Bitmap.createBitmap(pixels, width, height, Bitmap.Config.ARGB_8888)
        try {
            val target = File(outputDir, "$ts_ns.png")
            FileOutputStream(target).use { fos ->
                if (!bitmap.compress(Bitmap.CompressFormat.PNG, pngQuality, fos)) {
                    throw IllegalStateException("Bitmap.compress returned false")
                }
            }
        } finally {
            bitmap.recycle()
        }
    }

    /** Lightweight value for telemetry / JSON metadata at recording end. */
    data class Stats(
        val written: Int,
        val dropped: Int,
        val firstFrameNs: Long,
        val lastFrameNs: Long,
        val outputDir: File,
    ) {
        override fun toString(): String =
            "Stats(written=$written, dropped=$dropped, " +
                "first=$firstFrameNs, last=$lastFrameNs, dir=${outputDir.name})"
    }

    companion object {
        private const val TAG = "SimFrameRecorder"
        private const val THREAD_NAME = "NavSight-FrameRec"
        // Bounded in-flight queue. Sized so a brief encoder hiccup (e.g. a
        // GC pause) doesn't immediately start dropping at 30 Hz. Each slot
        // is one outstanding ~80 KB encode, so the cap is also a hard RAM
        // ceiling of ~2.5 MB peak in flight.
        private const val QUEUE_CAPACITY = 32
        // PNG quality is unused by Android's PNG encoder (PNG is lossless;
        // the int is for API symmetry with JPEG). Kept as a parameter so a
        // future JPEG fallback for slower devices doesn't change the API.
        private const val PNG_QUALITY_DEFAULT = 100
        private const val SHUTDOWN_TIMEOUT_S = 10L
    }
}
