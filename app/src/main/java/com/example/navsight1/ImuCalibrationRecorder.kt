package com.example.navsight1

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Dedicated IMU-only recorder for Allan-variance calibration.
 *
 * Unlike the regular sim recorder, this does NOT run VIO, NOT record frames,
 * and NOT activate the camera. It registers gyro + accel listeners at
 * SENSOR_DELAY_FASTEST (~200 Hz on S21 Ultra) on a dedicated HandlerThread
 * and writes a plain CSV. Low CPU = no thermal drift = honest noise data.
 *
 * Output: `<app-files>/imu_calibration_<timestamp>.csv`
 *   header: `ts_ns,gx,gy,gz,ax,ay,az`
 *   rows: one per gyro event, with the most recent accel reading pinned
 *
 * Why pair on gyro ticks: gyro and accel callbacks fire independently; sample
 * rates can differ slightly. For Allan variance, what matters is regular
 * sampling at the sensor's native rate. Pinning the last accel to each gyro
 * tick gives us a clean grid; the accel "held" value is a small bias on its
 * own Allan curve but doesn't dominate (the held interval is <5 ms at
 * fastest rate).
 *
 * Lifecycle:
 *   1. `start(context)` — opens output file, registers listeners, starts
 *      sample counter. Idempotent (returns false if already running).
 *   2. UI polls `state` for elapsed seconds + sample count.
 *   3. `stop()` — flushes, closes file, unregisters listeners. Idempotent.
 *
 * Safe to leave running for hours. Battery-efficient (no GPU, no camera).
 */
class ImuCalibrationRecorder {

    data class State(
        val isRecording: Boolean = false,
        val outputPath: String? = null,
        val elapsedSeconds: Long = 0L,
        val sampleCount: Long = 0L,
        val droppedAccel: Long = 0L,
        val errorMessage: String? = null,
        // 2026-05-17 — auto-stop support. targetDurationSec=null means
        // user-controlled stop only. Otherwise the ticker calls stop()
        // when elapsed >= target and sets autoStopped=true so the UI
        // can confirm the recording finished cleanly.
        val targetDurationSec: Long? = null,
        val autoStopped: Boolean = false,
    )

    companion object {
        private const val TAG = "ImuCalRec"
        private const val FLUSH_EVERY_SAMPLES = 1000
    }

    private val _state = MutableStateFlow(State())
    val state: StateFlow<State> = _state.asStateFlow()

    private var sensorManager: SensorManager? = null
    private var gyro: Sensor? = null
    private var accel: Sensor? = null
    private var listener: SensorEventListener? = null
    private var writer: BufferedWriter? = null
    private var outputFile: File? = null
    private var startNs: Long = 0L
    private var sampleCount: Long = 0L
    private var droppedAccel: Long = 0L
    private var bufferedRows: Int = 0
    private var targetDurationSec: Long? = null

    // Most recent accel reading, written on each gyro tick
    @Volatile private var lastAx: Float = 0f
    @Volatile private var lastAy: Float = 0f
    @Volatile private var lastAz: Float = 0f
    @Volatile private var hasAccel: Boolean = false

    private var handlerThread: HandlerThread? = null
    private var handler: Handler? = null

    /**
     * Start recording. Returns false if already running or if the device
     * lacks the required sensors.
     *
     * @param targetDurationSec optional auto-stop target in seconds. When set,
     *        the recorder calls stop() automatically once elapsed >= target.
     *        Pass null (default) for user-controlled stop only.
     */
    fun start(context: Context, targetDurationSec: Long? = null): Boolean {
        if (_state.value.isRecording) {
            Log.w(TAG, "start() called while already recording — ignored")
            return false
        }

        val sm = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
        if (sm == null) {
            _state.value = State(errorMessage = "SensorManager unavailable")
            return false
        }
        val g = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val a = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        if (g == null || a == null) {
            _state.value = State(errorMessage = "Required sensors missing (gyro=$g accel=$a)")
            return false
        }
        sensorManager = sm
        gyro = g
        accel = a

        // Output file in app-private external storage (same path as sim JSONs;
        // adb-pullable). Filename includes wall-clock so multiple sessions
        // don't collide.
        val ts = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        val baseDir = context.getExternalFilesDir(null) ?: context.filesDir
        val f = File(baseDir, "imu_calibration_$ts.csv")
        try {
            writer = BufferedWriter(FileWriter(f, false))
            writer?.write("ts_ns,gx,gy,gz,ax,ay,az\n")
            outputFile = f
        } catch (e: Exception) {
            Log.e(TAG, "Failed to open output file ${f.absolutePath}: ${e.message}", e)
            _state.value = State(errorMessage = "Failed to open file: ${e.message}")
            return false
        }

        // Dedicated thread so we don't share the main looper with the rest
        // of the app (UI redraws, sim recorder, etc.). Allan variance needs
        // regular dispatch — being preempted distorts the time-base.
        handlerThread = HandlerThread("ImuCalRecorder").also { it.start() }
        handler = Handler(handlerThread!!.looper)

        sampleCount = 0L
        droppedAccel = 0L
        bufferedRows = 0
        hasAccel = false
        startNs = SystemClock.elapsedRealtimeNanos()
        this.targetDurationSec = targetDurationSec

        listener = object : SensorEventListener {
            override fun onSensorChanged(event: SensorEvent) {
                when (event.sensor.type) {
                    Sensor.TYPE_ACCELEROMETER -> {
                        lastAx = event.values[0]
                        lastAy = event.values[1]
                        lastAz = event.values[2]
                        hasAccel = true
                    }
                    Sensor.TYPE_GYROSCOPE -> {
                        if (!hasAccel) {
                            // Drop early gyro samples before any accel arrived;
                            // pinned-accel would be (0,0,0) which is garbage.
                            droppedAccel++
                            return
                        }
                        val w = writer ?: return
                        try {
                            // Write row. event.timestamp is in elapsedRealtimeNanos
                            // domain (the same clock IMUPreintegrator uses), so
                            // downstream analysis aligns with VIO timestamps if
                            // you ever cross-reference.
                            w.write(
                                "${event.timestamp},${event.values[0]},${event.values[1]},${event.values[2]}," +
                                        "$lastAx,$lastAy,$lastAz\n"
                            )
                            sampleCount++
                            bufferedRows++
                            if (bufferedRows >= FLUSH_EVERY_SAMPLES) {
                                w.flush()
                                bufferedRows = 0
                            }
                        } catch (e: Exception) {
                            Log.e(TAG, "Write failed at sample $sampleCount: ${e.message}", e)
                            _state.value = _state.value.copy(errorMessage = "Write error: ${e.message}")
                        }
                    }
                }
            }
            override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) { /* no-op */ }
        }

        // SENSOR_DELAY_FASTEST asks for the sensor's max native rate. On
        // S21 Ultra both gyro and accel deliver ~200 Hz; older devices
        // return lower. The actual rate is whatever the driver gives us.
        val gyroOk = sm.registerListener(listener, g, SensorManager.SENSOR_DELAY_FASTEST, handler)
        val accelOk = sm.registerListener(listener, a, SensorManager.SENSOR_DELAY_FASTEST, handler)
        if (!gyroOk || !accelOk) {
            Log.e(TAG, "registerListener failed (gyro=$gyroOk accel=$accelOk)")
            stopInternal(errorMessage = "registerListener returned false")
            return false
        }

        _state.value = State(
            isRecording = true,
            outputPath = f.absolutePath,
            elapsedSeconds = 0L,
            sampleCount = 0L,
            targetDurationSec = targetDurationSec,
            autoStopped = false,
        )

        // Tick a state update once per second so the UI shows live elapsed
        // time + sample count without us repainting on every sample.
        startTicker()

        Log.i(TAG, "Started recording to ${f.absolutePath}")
        return true
    }

    /**
     * Stop recording, flush, close. Returns the final output File or null.
     */
    fun stop(): File? {
        if (!_state.value.isRecording) return outputFile
        return stopInternal(errorMessage = null)
    }

    private fun stopInternal(errorMessage: String?, autoStopped: Boolean = false): File? {
        try {
            val sm = sensorManager
            val l = listener
            if (sm != null && l != null) sm.unregisterListener(l)
            writer?.flush()
            writer?.close()
        } catch (e: Exception) {
            Log.e(TAG, "Error during stop: ${e.message}", e)
        }
        handlerThread?.quitSafely()
        handlerThread = null
        handler = null
        listener = null
        sensorManager = null
        writer = null
        val f = outputFile
        Log.i(TAG, "Stopped. samples=$sampleCount dropped=$droppedAccel file=${f?.absolutePath}")
        _state.value = State(
            isRecording = false,
            outputPath = f?.absolutePath,
            elapsedSeconds = (SystemClock.elapsedRealtimeNanos() - startNs) / 1_000_000_000L,
            sampleCount = sampleCount,
            droppedAccel = droppedAccel,
            errorMessage = errorMessage,
            targetDurationSec = targetDurationSec,
            autoStopped = autoStopped,
        )
        return f
    }

    private fun startTicker() {
        val h = handler ?: return
        h.postDelayed(object : Runnable {
            override fun run() {
                if (!_state.value.isRecording) return
                val elapsed = (SystemClock.elapsedRealtimeNanos() - startNs) / 1_000_000_000L
                val target = targetDurationSec
                // Auto-stop check. Once elapsed reaches the target the recorder
                // calls stop() with autoStopped=true so the UI can confirm a
                // clean finish. Without a target the ticker just keeps ticking.
                if (target != null && elapsed >= target) {
                    Log.i(TAG, "Auto-stop fired at elapsed=${elapsed}s (target=${target}s)")
                    stopInternal(errorMessage = null, autoStopped = true)
                    return
                }
                _state.value = _state.value.copy(
                    elapsedSeconds = elapsed,
                    sampleCount = sampleCount,
                    droppedAccel = droppedAccel,
                )
                handler?.postDelayed(this, 1000L)
            }
        }, 1000L)
    }
}
