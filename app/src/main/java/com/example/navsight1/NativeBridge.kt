package com.example.navsight1

import android.util.Log
import com.example.navsight1.VioData
import java.nio.ByteBuffer

object NativeBridge {
    private const val TAG = "NativeBridge"
    private var isLibraryLoaded = false

    init {
        try {
            System.loadLibrary("navsight")
            isLibraryLoaded = true
            Log.d(TAG, "Native library 'navsight' loaded successfully")
        } catch (e: UnsatisfiedLinkError) {
            Log.e(TAG, "Failed to load native library 'navsight': ${e.message}", e)
            isLibraryLoaded = false
        } catch (e: Exception) {
            Log.e(TAG, "Unexpected error loading native library: ${e.message}", e)
            isLibraryLoaded = false
        }
    }

    fun isLoaded(): Boolean = isLibraryLoaded

    external fun startVIO()
    external fun stopVIO()
    // DEAD CODE: old ByteArray version — superseded by processCameraFrameDirect (zero-copy ByteBuffer)
    // external fun processCameraFrame(
    //     frameData: ByteArray, width: Int, height: Int, timestamp: Long
    // ): VioData
    // Zero-copy: accepts direct ByteBuffer from CameraX ImageProxy.
    // rollingShutterSkewNs: Camera2 CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW
    // (API level 21+) — nanoseconds from first-row to last-row read-out.
    // Pass 0 if the key is absent (global-shutter device or unavailable).
    external fun processCameraFrameDirect(
        yBuffer: ByteBuffer, uvBuffer: ByteBuffer,
        width: Int, height: Int,
        yRowStride: Int, uvRowStride: Int, uvPixelStride: Int,
        timestamp: Long,
        rollingShutterSkewNs: Long
    ): VioData
    external fun processGyroscope(timestamp: Long, x: Float, y: Float, z: Float)
    external fun processAccelerometer(timestamp: Long, x: Float, y: Float, z: Float)
    external fun resetVIO()
    external fun setScale(scale: Double)
    external fun setDepthMap(depthData: FloatArray, width: Int, height: Int)
    external fun setIntrinsics(fx: Double, fy: Double, cx: Double, cy: Double)
    external fun setInitialHeading(azimuthRad: Double)
    // 2026-05-13 heading-startup fix: seed Madgwick yaw without waiting
    // for VIO init. Safe to call multiple times (re-init on each call).
    external fun seedMadgwickYaw(yawRad: Double)
    external fun setUserHeight(heightM: Float)
    // 2026-05-25 re-enabled (professor approved continuous compass). Called
    // every orientation tick when OS compass accuracy is HIGH/MEDIUM; native
    // shim (native-lib.cpp Java_..._setMagnetometerHeading) forwards to
    // IMUPreintegrator::setMagnetometerHeading for gentle gated yaw fusion.
    external fun setMagnetometerHeading(yawRad: Float)

    // 2026-05-26 — #2 loop-overlay path redraw. getLoopCorrectionVersion()
    // increments each time a loop closure re-optimizes the pose graph; the
    // ViewModel polls it and, on a change, calls getCorrectedTrajectory(out) to
    // rebuild pathHistory from the CORRECTED pose-graph node polyline so the two
    // loops visually overlay (only the now-node delta reaches the live pose).
    // outXz is filled [x0,z0,x1,z1,...] (x=East, z=North; same frame as VioData);
    // returns the number of (x,z) pairs written (<= outXz.size/2).
    external fun getLoopCorrectionVersion(): Int
    external fun getCorrectedTrajectory(outXz: FloatArray): Int

    // 2026-05-26 — locomotion-agnostic reported speed (m/s) for ALL motion types
    // (walking, scooter, bike). Depth-weighted metric speed: the recoverPose
    // translation scaled by the tracked feature points' MiDaS depths — NOT the
    // pedestrian step model, independent of the EKF velocity. Returns -1.0 before
    // the first estimate; the ViewModel then shows 0 and smooths the value.
    external fun getFusedSpeedMps(): Float

    // 2026-05-28 — MiDaS scale K (relative-disparity → metric m). Calibrated
    // inside the C++ depth-flow path from accumulated accel distance vs visual
    // relative distance. Persisting it across app launches is required because
    // the first recording after a cold start may not pass essential-matrix
    // verification (slow walk + close scene → low inlier ratio); without a
    // seeded K the looming path bails (K<=0) and the UI sits at 0 forever.
    // ViewModel loads from SharedPreferences on init, pushes here via
    // setMidasScaleK; periodically reads via getMidasScaleK and writes back.
    external fun setMidasScaleK(k: Double)
    external fun getMidasScaleK(): Double

    // Step 5: Calibration & Initialization
    // Returns 0=WAIT_STATIONARY, 1=WAIT_MOTION, 2=READY, 3=TIMEOUT_NEEDS_USER
    external fun getInitStatus(): Int
    external fun clearInitTimeout()
    // rotation: 9 floats row-major; gyroBias: 3 floats; accelBias: 3 floats
    external fun loadStoredCalibration(
        rotation: FloatArray, gyroBias: FloatArray, accelBias: FloatArray
    )
    // Returns true if calibration was filled (gate has passed); false otherwise
    external fun getCalibration(
        rotation: FloatArray, gyroBias: FloatArray, accelBias: FloatArray
    ): Boolean

    // Step 6: Horizontal-plane position covariance in m².
    // out = [σ_xx, σ_xz, σ_zz] (length 3). Returns true once the EKF has
    // finished its full init; false (zeros) before that.
    external fun getPositionCovariance(out: FloatArray): Boolean

    // Step 1 (Visual Production Plan): pass the absolute filesystem path of
    // the in-app camera calibration JSON. Native side parses and validates,
    // then pushes intrinsics + distortion into the VIO pipeline. Returns
    // true on success; false if the file is missing or fails validation
    // (engine then keeps the zero-distortion passthrough).
    external fun nativeLoadCalibration(path: String): Boolean

    // Step 7 (Visual Production Plan, ADR-013): pass the absolute filesystem
    // path of the ORB DBoW2 vocabulary (copied at app startup from
    // assets/ORBvoc.bin into <filesDir>/ORBvoc.bin — AssetManager paths are
    // not real filesystem paths and the DBoW2 reader needs to fopen). On
    // success the native LoopClosureDetector becomes ready and the Tracker
    // launches its 1 Hz query worker thread; on false the rest of the VIO
    // pipeline keeps running with loop closure disabled.
    external fun nativeLoadLoopClosureVocabulary(path: String): Boolean

    // EventCounters bridge — used by NavSightViewModel.saveSimulationData to
    // embed a per-event verdict (RELOC_ORB / BLUR / LOWLIGHT / ROT_GATE /
    // KLT / BA / MSCKF counts) into simulation_data_<ts>.json. Replaces the
    // old adb-logcat capture pipeline so untethered walks still produce a
    // verifiable per-step breakdown. See app/src/main/cpp/EventCounters.h
    // for the JSON schema.
    external fun nativeGetEventCountersJson(): String
    external fun nativeResetEventCounters()

    // Map-matching Step B* (MAP_MATCHING_PLAN.md §8M) — VIO→lat/lng bridge.
    // nativeSetSessionAnchor: push the ONE bootstrap GPS fix that anchors the
    // VIO local frame to geographic coordinates (ADR-004 — never feeds the EKF).
    // Idempotent on the native side; a second call is logged + ignored.
    external fun nativeSetSessionAnchor(latDeg: Double, lngDeg: Double, tNs: Long)
    // Returns [latDeg, lngDeg, tNs, varXyM2] for the current user-facing dot
    // (global_t_), or null when no SessionAnchor exists yet (matcher disabled /
    // GPS jammed). Read-only on the engine.
    external fun nativeCurrentVioLla(): DoubleArray?

    // Step 8b (Visual Production Plan): seed the EKF body→camera extrinsics
    // rotation R_bc from Android CameraCharacteristics.SENSOR_ORIENTATION.
    // R_bc_flat: 9 floats in row-major order.  Called once after camera open,
    // before the first frame arrives.  No-op if VIO is not yet started.
    external fun nativeSetExtrinsicsRotation(R_bc_flat: FloatArray)

    // ── Camera overlay Phase 2 / 3 / 4 (see camera_overlay_phase23_plan.md) ──

    // Phase 2: per-feature ages (frames survived) for the most recent VIO
    // frame. Pairs 1:1 with VioData.trackedPoints (length =
    // trackedPoints.size / 2). Returns the number of ages written into
    // `out`. Caller preallocates IntArray(MAX_FEATURES) once and reuses.
    // At 30 Hz: <30 frames is "new" (≤1 s), 30-89 is "established"
    // (1-3 s), ≥90 is "mature" (≥3 s) for the overlay color ramp.
    external fun getLastTrackedPointAges(out: IntArray): Int

    // Phase 3: flat snapshot of currently-active SLAM features.
    // Audit Finding 8 (2026-05-16): KDoc previously said stride=4 / FloatArray(48)
    // which was stale after v23.11 increased the stride from 4 → 7.
    // Layout per feature (7 floats):
    //   [feature_id, world_x, world_y, world_z, obs_u, obs_v, has_obs]
    // in Y-up world frame (X=East, Y=Up, Z=North).
    // Returns the number of features written (≤ out.size / 7).
    // Caller preallocates FloatArray(7 * MAX_SLAM_FEATURES) = FloatArray(84).
    // Returns 0 if EKF has no SLAM features yet.
    external fun getSlamSnapshot(out: FloatArray): Int

    // Phase 3: current camera pose for SLAM-point reprojection. 16 floats:
    //   [0..8]   R_world_cam row-major (camera→world rotation, Y-up world)
    //   [9..11]  t_world_cam (camera position in Y-up world)
    //   [12..15] fx, fy, cx, cy
    // Returns true on success, false if the EKF has not yet reached full
    // initialisation. Caller preallocates FloatArray(16) once and reuses.
    external fun getCurrentCameraPose(out: FloatArray): Boolean

    // Phase 4: cumulative loop-closure correction count from
    // EventCounters.loop_closure_corrections_applied. Polled per VIO
    // frame by the overlay; an increment triggers the 1-second
    // "LOOP CLOSURE" flash banner. Lock-free atomic read — cheap to
    // call every frame.
    external fun getLoopClosureCorrectionsApplied(): Long

    // ── Phase 1 Step 6 (post_v19_sprint_plan.md §205-298): LandmarkMap snapshot
    //
    /**
     * Returns the persistent LandmarkMap snapshot. Float array with stride 5:
     *   [0]=id, [1]=x (Y-up), [2]=y (Y-up), [3]=z (Y-up),
     *   [4]=observed_this_frame (1.0 if matched in the most recent keyframe, 0.0 otherwise).
     *
     * Empty array if VIO is not initialized or the map is empty.
     *
     * @see app/src/main/cpp/native-lib.cpp getLandmarkSnapshot
     */
    external fun getLandmarkSnapshot(): FloatArray
}
