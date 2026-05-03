package com.example.navsight1

import android.content.Context
import android.util.Log
import java.io.File
import java.io.PrintWriter
import java.io.StringWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.atomic.AtomicReference

/**
 * Step 6 (Task #31) — uncaught-exception sink.
 *
 * Writes a JSON crash report to `<external-files>/crash_logs/crash_<ts>.json`
 * on any unexpected termination, then chains to the previous handler so the
 * platform default behavior (process death, system crash dialog) is preserved.
 *
 * The latest VIO/sensor snapshot is published by [updateSnapshot] from the
 * ViewModel on every frame. When the JVM is in a poisoned state the dump may
 * still fail — this is best-effort, not a crash-recovery contract.
 */
object CrashLogger {

    private const val TAG = "CrashLogger"
    private const val DIR_NAME = "crash_logs"

    private val snapshotRef = AtomicReference<String>("{}")
    private var installed = false

    /** Replace the latest snapshot. JSON-shaped string; not validated. */
    fun updateSnapshot(json: String) {
        snapshotRef.set(json)
    }

    /**
     * Install the handler. Idempotent — safe to call multiple times.
     * Captures the previous default handler and chains to it after writing.
     */
    @Synchronized
    fun install(context: Context) {
        if (installed) return
        val appContext = context.applicationContext
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
            try {
                writeCrashFile(appContext, thread, throwable)
            } catch (t: Throwable) {
                Log.e(TAG, "Failed to write crash log", t)
            }
            previous?.uncaughtException(thread, throwable)
        }
        installed = true
        Log.i(TAG, "Crash logger installed; output dir = ${crashDir(appContext).absolutePath}")
    }

    private fun crashDir(context: Context): File {
        val parent = context.getExternalFilesDir(null) ?: context.filesDir
        val dir = File(parent, DIR_NAME)
        if (!dir.exists()) dir.mkdirs()
        return dir
    }

    private fun writeCrashFile(context: Context, thread: Thread, throwable: Throwable) {
        val timestamp = System.currentTimeMillis()
        val tsLabel = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date(timestamp))
        val file = File(crashDir(context), "crash_${tsLabel}_${timestamp}.json")

        val stackWriter = StringWriter()
        throwable.printStackTrace(PrintWriter(stackWriter))
        val stack = stackWriter.toString()

        val json = buildString {
            append("{")
            append("\"timestamp_ms\":").append(timestamp).append(",")
            append("\"timestamp_iso\":\"").append(escape(tsLabel)).append("\",")
            append("\"thread\":\"").append(escape(thread.name)).append("\",")
            append("\"exception_class\":\"").append(escape(throwable.javaClass.name)).append("\",")
            append("\"exception_message\":\"").append(escape(throwable.message ?: "")).append("\",")
            append("\"stack_trace\":\"").append(escape(stack)).append("\",")
            append("\"snapshot\":").append(snapshotRef.get() ?: "{}")
            append("}")
        }
        file.writeText(json, Charsets.UTF_8)
    }

    private fun escape(s: String): String {
        val sb = StringBuilder(s.length + 16)
        for (c in s) {
            when (c) {
                '\\' -> sb.append("\\\\")
                '"' -> sb.append("\\\"")
                '\n' -> sb.append("\\n")
                '\r' -> sb.append("\\r")
                '\t' -> sb.append("\\t")
                '\b' -> sb.append("\\b")
                '\u000C' -> sb.append("\\f")
                else -> if (c.code < 0x20) {
                    sb.append("\\u").append(String.format(Locale.US, "%04x", c.code))
                } else {
                    sb.append(c)
                }
            }
        }
        return sb.toString()
    }
}
