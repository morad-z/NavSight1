package com.example.navsight1

import android.content.Context
import android.content.SharedPreferences
import android.util.Log

/**
 * Versioned persistence of IMU initial calibration:
 *   - R_GtoI: 3x3 rotation (gravity-frame to IMU-frame), 9 floats row-major
 *   - gyro bias:  3 floats
 *   - accel bias: 3 floats
 *
 * Stored as comma-separated strings under a versioned key prefix so a future
 * schema change can migrate (or invalidate) old entries cleanly.
 */
object CalibrationStore {
    private const val TAG = "CalibrationStore"

    // Bump this whenever the stored schema changes. Old entries with a different
    // version are dropped on read.
    //
    // v2 (2026-05-09): world frame switched from Y-up to Z-up ENU
    // (project_z_up_fix_2026_05_08). The stored R_GtoI matrix encodes
    // body-from-world; switching the world convention rotates R_GtoI by
    // 90° about world-X. Old v1 entries seed R_GtoI with a 50°+ residual
    // against gravity from the first frame (verified in v9 walk:
    // simulation_data_1778343884100, LC_GA r_norm starts at 0.86 and
    // climbs to 1.97 — gravity-alignment can't recover from a seed
    // that's off by half the unit-vector range). Forcing a re-calibration
    // is the only correct response.
    const val SCHEMA_VERSION = 2

    private const val PREFS_NAME = "navsight_calibration"
    private const val KEY_VERSION = "version"
    private const val KEY_ROTATION = "rotation"
    private const val KEY_GYRO_BIAS = "gyro_bias"
    private const val KEY_ACCEL_BIAS = "accel_bias"
    private const val KEY_SAVED_AT_MS = "saved_at_ms"

    data class CalibrationData(
        val rotation: FloatArray,    // length 9, row-major R_GtoI
        val gyroBias: FloatArray,    // length 3
        val accelBias: FloatArray,   // length 3
        val savedAtMs: Long,
    ) {
        init {
            require(rotation.size == 9) { "rotation must be 9 floats, got ${rotation.size}" }
            require(gyroBias.size == 3) { "gyroBias must be 3 floats, got ${gyroBias.size}" }
            require(accelBias.size == 3) { "accelBias must be 3 floats, got ${accelBias.size}" }
        }
    }

    private fun prefs(context: Context): SharedPreferences =
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

    fun save(context: Context, data: CalibrationData) {
        try {
            prefs(context).edit()
                .putInt(KEY_VERSION, SCHEMA_VERSION)
                .putString(KEY_ROTATION, encode(data.rotation))
                .putString(KEY_GYRO_BIAS, encode(data.gyroBias))
                .putString(KEY_ACCEL_BIAS, encode(data.accelBias))
                .putLong(KEY_SAVED_AT_MS, data.savedAtMs)
                .apply()
            Log.i(TAG, "Calibration saved (v$SCHEMA_VERSION) at ${data.savedAtMs}")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to save calibration: ${e.message}", e)
        }
    }

    /** Returns null if no entry exists, version mismatches, or parse fails. */
    fun load(context: Context): CalibrationData? {
        val p = prefs(context)
        val storedVersion = p.getInt(KEY_VERSION, -1)
        if (storedVersion != SCHEMA_VERSION) {
            if (storedVersion != -1) {
                Log.w(TAG, "Stored calibration version=$storedVersion != current=$SCHEMA_VERSION; dropping")
                clear(context)
            }
            return null
        }
        return try {
            val rotation = decode(p.getString(KEY_ROTATION, null), 9) ?: return null
            val gyroBias = decode(p.getString(KEY_GYRO_BIAS, null), 3) ?: return null
            val accelBias = decode(p.getString(KEY_ACCEL_BIAS, null), 3) ?: return null
            val savedAtMs = p.getLong(KEY_SAVED_AT_MS, 0L)
            CalibrationData(rotation, gyroBias, accelBias, savedAtMs)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to parse stored calibration: ${e.message}", e)
            clear(context)
            null
        }
    }

    fun clear(context: Context) {
        prefs(context).edit().clear().apply()
    }

    private fun encode(arr: FloatArray): String =
        arr.joinToString(",") { it.toString() }

    private fun decode(s: String?, expectedLen: Int): FloatArray? {
        if (s.isNullOrBlank()) return null
        val parts = s.split(",")
        if (parts.size != expectedLen) {
            Log.w(TAG, "decode: expected $expectedLen floats, got ${parts.size}")
            return null
        }
        return FloatArray(expectedLen) { i ->
            parts[i].toFloatOrNull() ?: return null
        }
    }
}
