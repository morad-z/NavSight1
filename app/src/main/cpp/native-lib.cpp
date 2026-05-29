#include <jni.h>
#include <android/log.h>
#include <string>
#include <mutex>
#include <memory>
#include <cstdint>
#include <cmath>
#include <chrono>  // 2026-05-16 single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <opencv2/calib3d.hpp>
#include "VioEngine.h"
#include "EventCounters.h"

#define TAG "NavSight-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ── Global state ──────────────────────────────────────────────────────────────

static std::mutex  state_mutex;
static std::shared_ptr<VioEngine> g_vision = nullptr;

// Cached JNI class/method IDs — avoid per-frame FindClass/GetMethodID reflection
static jclass    g_viodata_cls = nullptr;  // GlobalRef
static jmethodID g_viodata_ctor = nullptr;

// Accumulated pose
static double g_x = 0, g_y = 0, g_z = 0;
static double g_roll = 0, g_pitch = 0, g_yaw = 0;
static double g_scale = 1.0;
static double g_user_scale_correction = 1.0;

// RAW VO for simulation
static double g_raw_x = 0, g_raw_y = 0, g_raw_z = 0, g_raw_yaw = 0;

// Last sensor values
static float g_ax = 0, g_ay = 0, g_az = 0;
static float g_gx = 0, g_gy = 0, g_gz = 0;

// Diagnostic fields from VisionOutput
static double g_mean_flow = 0;
static int    g_inlier_count = 0;
static int    g_step_count = 0;
static double g_step_freq = 0;
static double g_stride_length = 0;
static int    g_pose_flags = 0;
static double g_heading = 0;

// Camera overlay Phase 2 (Task C): the latest frame's per-feature ages
// (in frames survived). Updated atomically with the trackedPoints array
// inside processCameraFrameDirect under state_mutex. Read by the
// getLastTrackedPointAges JNI accessor for the overlay's age-coloring.
// Cleared in resetPoseState() so a fresh VIO session starts empty.
static std::vector<int> g_tracked_point_ages;

// 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
//
// Cached EKF overlay snapshot. The Kotlin overlay path makes TWO sequential
// JNI calls per render frame (getCurrentCameraPose + getSlamSnapshot). Without
// this cache each call would re-read EKFState independently, exposing a
// multi-millisecond window in which processFrame on the camera thread can
// mutate R_GtoI_ / p_G_ / window_ / slam_features_ → the camera pose used to
// project feature dots ends up from a DIFFERENT state-version than the world
// positions themselves → orange dots slide ~1-2 px even when phone is still.
//
// Strategy: the FIRST JNI getter on each overlay tick refreshes the cache
// from EKFState::snapshotForOverlay() (one mutex acquisition, consistent
// state); the SECOND getter (within kOverlaySnapshotTtlMs of the first) reads
// from the cache instead of touching the EKF. This guarantees both getters
// project from the SAME state-version per overlay tick.
//
// TTL chosen to span one Kotlin overlay frame (~16 ms at 60 fps recompose,
// ~33 ms at 30 fps) without bridging across frames where the user would
// actually expect a fresh pose. 20 ms is the midpoint of the practical range.
static constexpr long long kOverlaySnapshotTtlMs = 20;
static EKFState::OverlaySnapshot g_overlay_snapshot;
static long long                 g_overlay_snapshot_ts_ms = 0;
static long long                 g_overlay_log_last_ms = 0;
static bool                      g_overlay_snapshot_valid = false;

// Refresh-or-reuse the cached overlay snapshot. Caller MUST hold state_mutex.
// Returns a const reference into g_overlay_snapshot. When the cache is
// refreshed: increments overlay_snapshots_taken_total and (once per second
// of wall time) emits LOGI with R_GtoI_yaw, p_G, slam_feature_count so
// post-fix walks can grep the falsifier — if dots still slide visibly while
// these LOGI lines remain stable, the root cause is elsewhere.
static const EKFState::OverlaySnapshot& ensureOverlaySnapshot(VioEngine* vision) {
    const long long now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    const bool fresh = g_overlay_snapshot_valid &&
                       (now_ms - g_overlay_snapshot_ts_ms) <= kOverlaySnapshotTtlMs;
    if (fresh) return g_overlay_snapshot;

    if (!vision) {
        g_overlay_snapshot = EKFState::OverlaySnapshot{};
        g_overlay_snapshot_valid = false;
        g_overlay_snapshot_ts_ms = now_ms;
        return g_overlay_snapshot;
    }
    const EKFState* ekf = vision->getEKFState();
    if (!ekf) {
        g_overlay_snapshot = EKFState::OverlaySnapshot{};
        g_overlay_snapshot_valid = false;
        g_overlay_snapshot_ts_ms = now_ms;
        return g_overlay_snapshot;
    }

    g_overlay_snapshot       = ekf->snapshotForOverlay();
    g_overlay_snapshot_valid = true;
    g_overlay_snapshot_ts_ms = now_ms;
    navsight::eventCounters().overlay_snapshots_taken_total.fetch_add(
        1, std::memory_order_relaxed);

    // Falsifier log: emit at most once per second to keep logcat readable.
    if ((now_ms - g_overlay_log_last_ms) >= 1000) {
        g_overlay_log_last_ms = now_ms;
        // R_GtoI_yaw: nav-CW yaw extracted from R_GtoI row 1 (body-x in world).
        // Matches getWorldHeadingRad() convention (atan2(R[1,0], R[0,0])).
        const double yaw_rad = std::atan2(g_overlay_snapshot.R_GtoI(1, 0),
                                          g_overlay_snapshot.R_GtoI(0, 0));
        const double yaw_deg = yaw_rad * 180.0 / M_PI;
        LOGI("OVERLAY_SNAPSHOT: init=%d R_GtoI_yaw_deg=%.2f p_G=(%.3f,%.3f,%.3f) n_slam=%zu",
             g_overlay_snapshot.full_initialized ? 1 : 0,
             yaw_deg,
             g_overlay_snapshot.p_G(0), g_overlay_snapshot.p_G(1), g_overlay_snapshot.p_G(2),
             g_overlay_snapshot.slam_features.size());
    }
    return g_overlay_snapshot;
}

// ── resetPoseState ────────────────────────────────────────────────────────────
// Must be called with state_mutex held.

static void resetPoseState() {
    g_x = g_y = g_z = 0;
    g_roll = g_pitch = g_yaw = 0;
    g_raw_x = g_raw_y = g_raw_z = g_raw_yaw = 0;
    g_ax = g_ay = g_az = 0;
    g_gx = g_gy = g_gz = 0;
    g_mean_flow = 0; g_inlier_count = 0; g_step_count = 0;
    g_step_freq = 0; g_stride_length = 0; g_pose_flags = 0; g_heading = 0;
    g_tracked_point_ages.clear();
    // 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
    g_overlay_snapshot = EKFState::OverlaySnapshot{};
    g_overlay_snapshot_valid = false;
    g_overlay_snapshot_ts_ms = 0;
    // keep g_scale
}

// ── JNI_OnLoad: cache VioData class + constructor ────────────────────────────
// Called once when the library is loaded. Avoids per-frame FindClass overhead.

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* /*reserved*/) {
    JNIEnv* env = nullptr;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        return JNI_ERR;
    }

    jclass local = env->FindClass("com/example/navsight1/VioData");
    if (local) {
        g_viodata_cls = reinterpret_cast<jclass>(env->NewGlobalRef(local));
        env->DeleteLocalRef(local);

        const char* sig = "(DDDDDDDIIDZ[FDDDDFFFFFFDIIDDIDD)V";
        g_viodata_ctor = env->GetMethodID(g_viodata_cls, "<init>", sig);
        if (!g_viodata_ctor) {
            LOGE("JNI_OnLoad: VioData constructor not found");
        }
    } else {
        LOGE("JNI_OnLoad: VioData class not found");
    }

    LOGI("JNI_OnLoad: native library loaded, VioData cached");
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void* /*reserved*/) {
    JNIEnv* env = nullptr;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) return;
    if (g_viodata_cls) {
        env->DeleteGlobalRef(g_viodata_cls);
        g_viodata_cls  = nullptr;
        g_viodata_ctor = nullptr;
    }
    LOGI("JNI_OnUnload: GlobalRef released");
}

extern "C" {

// ── startVIO ──────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_startVIO(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (g_vision) {
        g_vision.reset();
    }
    g_vision = std::make_shared<VioEngine>();
    g_vision->setUserScaleCorrection(g_user_scale_correction);
    resetPoseState();
    LOGI("VIO started");
}

// ── stopVIO ───────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setImuNoiseParameters(JNIEnv*, jobject,
                                                            jfloat accel_noise, jfloat gyro_noise,
                                                            jfloat accel_rw, jfloat gyro_rw) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->getIMU().setNoiseParameters(accel_noise, gyro_noise, accel_rw, gyro_rw);
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_stopVIO(JNIEnv*, jobject) {
    std::shared_ptr<VioEngine> to_delete;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        to_delete = g_vision;
        g_vision.reset();
    }
    // shared_ptr will delete when to_delete goes out of scope
    // (outside the lock to avoid holding it during destruction)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        resetPoseState();
    }
    LOGI("VIO stopped");
}

// DEAD CODE: processCameraFrame (old ByteArray version) — Kotlin caller commented out,
// superseded by processCameraFrameDirect (zero-copy CameraX ByteBuffer path).
//
// 2026-05-09 v19 frame-convention audit warning:
// THIS DEAD BLOCK IS MISSING THE Z-up → Y-up AXIS SWAP that the live
// processCameraFrameDirect applies at lines 388-390. If this code is ever
// re-enabled without ALSO adding the swap (`g_y = output.t[2]; g_z = output.t[1]`),
// every downstream Kotlin/UI/sim consumer will silently get scrambled
// trajectories: g_y will receive North (Z-up world Y) instead of Up. All
// distance/speed math (uses x and z only) will mix North into "altitude" and
// vice versa. Existing sim fixture files would be corrupted retroactively.
// DO NOT uncomment without applying the same swap from processCameraFrameDirect.
/*
JNIEXPORT jobject JNICALL
Java_com_example_navsight1_NativeBridge_processCameraFrame(
        JNIEnv* env,
        jobject,
        jbyteArray frameData,
        jint width,
        jint height,
        jlong timestamp) {

    VioEngine* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }

    VisionOutput output{};
    output.valid = false;

    if (vision && width > 0 && height > 0) {
        jbyte* bytes = env->GetByteArrayElements(frameData, nullptr);
        if (bytes) {
            output = vision->processFrame(
                    reinterpret_cast<const uint8_t*>(bytes),
                    static_cast<int>(width),
                    static_cast<int>(height),
                    static_cast<int64_t>(timestamp));
            env->ReleaseByteArrayElements(frameData, bytes, JNI_ABORT);
        }
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (output.valid && !output.R.empty() && !output.t.empty()) {
            g_x = output.t.at<double>(0);
            g_y = output.t.at<double>(1);
            g_z = output.t.at<double>(2);
            g_scale = output.estimatedScale;

            g_raw_x = output.rawT.empty() ? 0.0 : output.rawT.at<double>(0);
            g_raw_y = output.rawT.empty() ? 0.0 : output.rawT.at<double>(1);
            g_raw_z = output.rawT.empty() ? 0.0 : output.rawT.at<double>(2);

            if (!output.rawR.empty()) {
                cv::Mat rv;
                cv::Rodrigues(output.rawR, rv);
                g_raw_yaw = rv.at<double>(1);
            }

            g_mean_flow = output.meanFlow;
            g_inlier_count = output.inlierCount;
            g_step_count = output.stepCount;
            g_step_freq = output.stepFreq;
            g_stride_length = output.strideLength;
            g_pose_flags = output.poseFlags;
            g_heading = output.heading;

            // 2026-05-09 v19 frame-convention audit annotation:
            // These Euler-angle formulas decompose R assuming a Y-up camera
            // rotation matrix, but `output.R` is R_GtoI_ — a Z-up world-to-body
            // rotation. The g_roll/g_pitch/g_yaw values written here are NOT
            // correct intrinsic Euler angles for R_GtoI; they're numerical
            // artifacts of applying the wrong decomposition.
            //
            // VioData.roll/pitch/yaw flows only into Kotlin diagnostics
            // (crash snapshot, debug logging). They are NEVER re-ingested by
            // C++ and never used in motion / position math. The user-visible
            // heading on the map comes from g_heading (line above) which is
            // the EKF's gravity-aligned scalar yaw — that one is correct.
            //
            // Do NOT consume VioData.roll/pitch/yaw for any feature beyond
            // diagnostics without first rewriting this decomposition for the
            // Z-up world / body convention (see EKFState.h:516).
            const cv::Mat& R = output.R;
            g_pitch = std::asin(-R.at<double>(1, 2));
            if (std::abs(std::cos(g_pitch)) > 1e-6) {
                g_roll = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2));
                g_yaw  = std::atan2(R.at<double>(1, 0), R.at<double>(1, 1));
            } else {
                g_roll = 0;
                g_yaw  = std::atan2(-R.at<double>(2, 0), R.at<double>(0, 0));
            }
        }
    }

    if (!g_viodata_cls || !g_viodata_ctor) {
        LOGE("processCameraFrame: VioData JNI cache not initialized");
        return nullptr;
    }

    jfloatArray pointsArray = env->NewFloatArray(
            static_cast<jsize>(output.trackedPoints.size()));
    if (!pointsArray) {
        LOGE("processCameraFrame: NewFloatArray failed (OOM)");
        return nullptr;
    }
    if (!output.trackedPoints.empty()) {
        env->SetFloatArrayRegion(pointsArray, 0,
                                 static_cast<jsize>(output.trackedPoints.size()),
                                 output.trackedPoints.data());
    }

    double ret_x, ret_y, ret_z, ret_roll, ret_pitch, ret_yaw;
    double ret_quality, ret_scale;
    double ret_rx, ret_ry, ret_rz, ret_ryaw;
    int    ret_tracked, ret_total;
    jboolean ret_initialized;
    float  ret_ax, ret_ay, ret_az, ret_gx, ret_gy, ret_gz;
    double ret_mean_flow, ret_step_freq, ret_stride_length, ret_heading, ret_td_imu_cam;
    int    ret_inlier_count, ret_step_count, ret_pose_flags;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        ret_x = g_x; ret_y = g_y; ret_z = g_z;
        ret_roll = g_roll; ret_pitch = g_pitch; ret_yaw = g_yaw;
        ret_scale = g_scale;
        ret_rx = g_raw_x; ret_ry = g_raw_y; ret_rz = g_raw_z; ret_ryaw = g_raw_yaw;
        ret_ax = g_ax; ret_ay = g_ay; ret_az = g_az;
        ret_gx = g_gx; ret_gy = g_gy; ret_gz = g_gz;
        ret_mean_flow = g_mean_flow;
        ret_inlier_count = g_inlier_count;
        ret_step_count = g_step_count;
        ret_step_freq = g_step_freq;
        ret_stride_length = g_stride_length;
        ret_pose_flags = g_pose_flags;
        ret_heading = g_heading;
        ret_td_imu_cam = output.td_imu_cam;
    }
    ret_quality = output.quality;
    ret_tracked = output.trackedCount;
    ret_total = output.totalCount;
    ret_initialized = static_cast<jboolean>(output.valid);

    return env->NewObject(
            g_viodata_cls, g_viodata_ctor,
            ret_x, ret_y, ret_z,
            ret_roll, ret_pitch, ret_yaw,
            ret_quality,
            ret_tracked, ret_total,
            ret_scale,
            ret_initialized,
            pointsArray,
            ret_rx, ret_ry, ret_rz, ret_ryaw,
            ret_ax, ret_ay, ret_az,
            ret_gx, ret_gy, ret_gz,
            ret_mean_flow,
            ret_inlier_count, ret_step_count,
            ret_step_freq, ret_stride_length,
            ret_pose_flags, ret_heading,
            ret_td_imu_cam);
}
*/

// ── processCameraFrameDirect (zero-copy CameraX) ────────────────────────────
// Accepts direct ByteBuffers from CameraX ImageProxy planes.
// Y plane is always dense (rowStride == width for 640x480).
// UV plane is interleaved (NV21: VU pairs) — we assemble NV21 in-place.

JNIEXPORT jobject JNICALL
Java_com_example_navsight1_NativeBridge_processCameraFrameDirect(
        JNIEnv* env,
        jobject /* thiz */,
        jobject yBuffer,
        jobject uvBuffer,
        jint width,
        jint height,
        jint yRowStride,
        jint uvRowStride,
        jint uvPixelStride,
        jlong timestamp,
        jlong rollingShutterSkewNs) {

    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }

    VisionOutput output{};
    output.valid = false;

    if (vision && width > 0 && height > 0 && yBuffer && uvBuffer) {
        // Step 8c: push rolling-shutter skew into the Tracker once per frame,
        // before frame data is consumed. Camera2 API:
        // CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW (API level 21+).
        // 0 = global-shutter device or key absent — correction disabled.
        vision->setRollingShutterSkew(static_cast<int64_t>(rollingShutterSkewNs));

        auto* yData = static_cast<uint8_t*>(env->GetDirectBufferAddress(yBuffer));
        auto* uvData = static_cast<uint8_t*>(env->GetDirectBufferAddress(uvBuffer));

        if (yData && uvData) {
            int ySize = width * height;
            int uvSize = (width * height) / 2;

            // Fast path: if Y is dense and UV is interleaved NV21-style,
            // we can build NV21 with a single memcpy per plane.
            // NV21 layout: [Y plane (w*h)] [VU interleaved (w*h/2)]
            thread_local std::vector<uint8_t> nv21_buf;
            int totalSize = ySize + uvSize;
            if ((int)nv21_buf.size() != totalSize) nv21_buf.resize(totalSize);

            // Copy Y plane (handle stride if needed)
            if (yRowStride == width) {
                memcpy(nv21_buf.data(), yData, ySize);
            } else {
                for (int row = 0; row < height; row++) {
                    memcpy(nv21_buf.data() + row * width,
                           yData + row * yRowStride, width);
                }
            }

            // Copy UV plane
            // CameraX YUV_420_888: plane[1]=U, plane[2]=V
            // We receive plane[2] (V) which for NV21 is interleaved VU with pixelStride=2
            if (uvPixelStride == 2 && uvRowStride == width) {
                // Already interleaved VU (NV21 format) — direct copy
                memcpy(nv21_buf.data() + ySize, uvData, uvSize);
            } else if (uvPixelStride == 2) {
                // Interleaved but with stride padding
                int uvHeight = height / 2;
                for (int row = 0; row < uvHeight; row++) {
                    memcpy(nv21_buf.data() + ySize + row * width,
                           uvData + row * uvRowStride, width);
                }
            } else {
                // Planar UV — need to interleave (rare on modern devices)
                // Fall back to just using Y plane as grayscale
                // (VIO only uses grayscale anyway, UV doesn't matter)
                memset(nv21_buf.data() + ySize, 128, uvSize);
            }

            output = vision->processFrame(
                    nv21_buf.data(),
                    static_cast<int>(width),
                    static_cast<int>(height),
                    static_cast<int64_t>(timestamp));
        }
    }

    // Update accumulated pose (same as processCameraFrame)
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (output.valid && !output.R.empty() && !output.t.empty()) {
            // Coordinate-frame boundary swap (Z-up internal -> Y-up exposed).
            //
            // The C++ pipeline runs in Z-up ENU world (X=East, Y=North, Z=Up)
            // matching the Madgwick filter and Android sensor body frame.
            // The Kotlin/UI/sim-format layer historically expects Y-up
            // (X=East, Y=Up, Z=North). Swap output.t[1] (North) and
            // output.t[2] (Up) here so legacy Kotlin code, sim recordings,
            // and analyzer scripts continue to work without changes.
            // See scripts/test_z_up_conventions.py and the 2026-05-07
            // Z-up surgical fix.
            g_x = output.t.at<double>(0);   // East — same in both frames
            g_y = output.t.at<double>(2);   // Up   — Z-up index 2 -> exposed as Y
            g_z = output.t.at<double>(1);   // North — Z-up index 1 -> exposed as Z
            g_scale = output.estimatedScale;

            // rawT is in OpenCV camera frame (X=right, Y=down, Z=forward),
            // not world frame, so no axis swap.
            g_raw_x = output.rawT.empty() ? 0.0 : output.rawT.at<double>(0);
            g_raw_y = output.rawT.empty() ? 0.0 : output.rawT.at<double>(1);
            g_raw_z = output.rawT.empty() ? 0.0 : output.rawT.at<double>(2);

            if (!output.rawR.empty()) {
                cv::Mat rv;
                cv::Rodrigues(output.rawR, rv);
                g_raw_yaw = rv.at<double>(1);
            }

            g_mean_flow = output.meanFlow;
            g_inlier_count = output.inlierCount;
            g_step_count = output.stepCount;
            g_step_freq = output.stepFreq;
            g_stride_length = output.strideLength;
            g_pose_flags = output.poseFlags;
            g_heading = output.heading;

            // 2026-05-09 v19 frame-convention audit annotation:
            // These Euler-angle formulas decompose R assuming a Y-up camera
            // rotation matrix, but `output.R` is R_GtoI_ — a Z-up world-to-body
            // rotation. The g_roll/g_pitch/g_yaw values written here are NOT
            // correct intrinsic Euler angles for R_GtoI; they're numerical
            // artifacts of applying the wrong decomposition.
            //
            // VioData.roll/pitch/yaw flows only into Kotlin diagnostics
            // (crash snapshot, debug logging). They are NEVER re-ingested by
            // C++ and never used in motion / position math. The user-visible
            // heading on the map comes from g_heading (line above) which is
            // the EKF's gravity-aligned scalar yaw — that one is correct.
            //
            // Do NOT consume VioData.roll/pitch/yaw for any feature beyond
            // diagnostics without first rewriting this decomposition for the
            // Z-up world / body convention (see EKFState.h:516).
            const cv::Mat& R = output.R;
            g_pitch = std::asin(-R.at<double>(1, 2));
            if (std::abs(std::cos(g_pitch)) > 1e-6) {
                g_roll = std::atan2(R.at<double>(0, 2), R.at<double>(2, 2));
                g_yaw  = std::atan2(R.at<double>(1, 0), R.at<double>(1, 1));
            } else {
                g_roll = 0;
                g_yaw  = std::atan2(-R.at<double>(2, 0), R.at<double>(0, 0));
            }
        }
    }

    if (!g_viodata_cls || !g_viodata_ctor) {
        LOGE("processCameraFrameDirect: VioData JNI cache not initialized");
        return nullptr;
    }

    jfloatArray pointsArray = env->NewFloatArray(
            static_cast<jsize>(output.trackedPoints.size()));
    if (!pointsArray) return nullptr;
    if (!output.trackedPoints.empty()) {
        env->SetFloatArrayRegion(pointsArray, 0,
                                 static_cast<jsize>(output.trackedPoints.size()),
                                 output.trackedPoints.data());
    }

    // Phase 2 camera overlay (Task C): publish the per-feature ages
    // (frames survived) for getLastTrackedPointAges. Done under
    // state_mutex so the JNI getter sees a coherent snapshot. The
    // overlay calls getLastTrackedPointAges right after this method
    // returns on the same camera thread.
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        g_tracked_point_ages = output.trackedPointAges;
    }

    double ret_x, ret_y, ret_z, ret_roll, ret_pitch, ret_yaw;
    double ret_quality, ret_scale;
    double ret_rx, ret_ry, ret_rz, ret_ryaw;
    int    ret_tracked, ret_total;
    jboolean ret_initialized;
    float  ret_ax, ret_ay, ret_az, ret_gx, ret_gy, ret_gz;
    double ret_mean_flow, ret_step_freq, ret_stride_length, ret_heading, ret_td_imu_cam;
    int    ret_inlier_count, ret_step_count, ret_pose_flags;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        ret_x = g_x; ret_y = g_y; ret_z = g_z;
        ret_roll = g_roll; ret_pitch = g_pitch; ret_yaw = g_yaw;
        ret_scale = g_scale;
        ret_rx = g_raw_x; ret_ry = g_raw_y; ret_rz = g_raw_z; ret_ryaw = g_raw_yaw;
        ret_ax = g_ax; ret_ay = g_ay; ret_az = g_az;
        ret_gx = g_gx; ret_gy = g_gy; ret_gz = g_gz;
        ret_mean_flow = g_mean_flow;
        ret_inlier_count = g_inlier_count;
        ret_step_count = g_step_count;
        ret_step_freq = g_step_freq;
        ret_stride_length = g_stride_length;
        ret_pose_flags = g_pose_flags;
        ret_heading = g_heading;
        ret_td_imu_cam = output.td_imu_cam;
    }
    ret_quality = output.quality;
    ret_tracked = output.trackedCount;
    ret_total = output.totalCount;
    ret_initialized = static_cast<jboolean>(output.valid);

    return env->NewObject(
            g_viodata_cls, g_viodata_ctor,
            ret_x, ret_y, ret_z,
            ret_roll, ret_pitch, ret_yaw,
            ret_quality,
            ret_tracked, ret_total,
            ret_scale,
            ret_initialized,
            pointsArray,
            ret_rx, ret_ry, ret_rz, ret_ryaw,
            ret_ax, ret_ay, ret_az,
            ret_gx, ret_gy, ret_gz,
            ret_mean_flow,
            ret_inlier_count, ret_step_count,
            ret_step_freq, ret_stride_length,
            ret_pose_flags, ret_heading,
            ret_td_imu_cam);
}

// ── processGyroscope ──────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_processGyroscope(
        JNIEnv*, jobject /* thiz */,
        jlong timestamp, jfloat x, jfloat y, jfloat z) {

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("processGyroscope: NaN/Inf value, dropping");
        return;
    }

    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        g_gx = x; g_gy = y; g_gz = z;
        vision = g_vision;
    }
    if (vision) {
        vision->addGyroData(static_cast<int64_t>(timestamp), x, y, z);
    }
}

// ── processAccelerometer ─────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_processAccelerometer(
        JNIEnv*, jobject /* thiz */,
        jlong timestamp, jfloat x, jfloat y, jfloat z) {

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("processAccelerometer: NaN/Inf value, dropping");
        return;
    }

    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        g_ax = x; g_ay = y; g_az = z;
        vision = g_vision;
    }
    if (vision) {
        vision->addAccelData(static_cast<int64_t>(timestamp), x, y, z);
    }
}

// ── resetVIO ──────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_resetVIO(JNIEnv*, jobject) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
        resetPoseState();
    }
    if (vision) {
        vision->reset();
    }
    LOGI("VIO reset");
}

// ── setScale ──────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setScale(
        JNIEnv*, jobject /* thiz */, jdouble scale) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        g_user_scale_correction = scale;
        vision = g_vision;
    }
    if (vision) {
        vision->setUserScaleCorrection(static_cast<double>(scale));
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setDepthMap(
        JNIEnv* env, jobject /* thiz */, jfloatArray depthData, jint width, jint height) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision && depthData) {
        jfloat* data = env->GetFloatArrayElements(depthData, nullptr);
        if (data) {
            vision->setDepthMap(reinterpret_cast<const float*>(data), width, height);
            env->ReleaseFloatArrayElements(depthData, data, JNI_ABORT);
        }
    }
}

// ── setIntrinsics ─────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setIntrinsics(
        JNIEnv*, jobject /* thiz */,
        jdouble fx, jdouble fy, jdouble cx, jdouble cy) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setIntrinsics(fx, fy, cx, cy);
    }
}

// ── setInitialHeading ────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setInitialHeading(
        JNIEnv*, jobject /* thiz */, jdouble azimuthRad) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setInitialHeading(static_cast<double>(azimuthRad));
    }
}

// ── seedMadgwickYaw (2026-05-13 heading-startup fix) ─────────────────────────
// Decoupled from setInitialHeading so SensorRepository can seed Madgwick's
// yaw the moment the compass is stable, without waiting for VIO init. The
// existing setInitialHeading path stays for EKF rotation seeding.

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_seedMadgwickYaw(
        JNIEnv*, jobject /* thiz */, jdouble yawRad) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->seedMadgwickYaw(static_cast<double>(yawRad));
    }
}

// ── setUserHeight ────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setUserHeight(
        JNIEnv*, jobject /* thiz */, jfloat heightM) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setUserHeight(static_cast<float>(heightM));
    }
}

// ── Step 5: Calibration & Initialization JNI ────────────────────────────────

JNIEXPORT jint JNICALL
Java_com_example_navsight1_NativeBridge_getInitStatus(JNIEnv*, jobject) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    return vision ? vision->getInitStatus() : 0;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_clearInitTimeout(JNIEnv*, jobject) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) vision->clearInitTimeout();
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_loadStoredCalibration(
        JNIEnv* env, jobject,
        jfloatArray rotation,
        jfloatArray gyroBias,
        jfloatArray accelBias) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision || !rotation || !gyroBias || !accelBias) return;

    if (env->GetArrayLength(rotation) != 9
        || env->GetArrayLength(gyroBias) != 3
        || env->GetArrayLength(accelBias) != 3) {
        LOGE("loadStoredCalibration: bad array sizes");
        return;
    }

    jfloat* rot_ptr = env->GetFloatArrayElements(rotation, nullptr);
    jfloat* gb_ptr  = env->GetFloatArrayElements(gyroBias, nullptr);
    jfloat* ab_ptr  = env->GetFloatArrayElements(accelBias, nullptr);

    vision->loadStoredCalibration(rot_ptr, gb_ptr, ab_ptr);

    env->ReleaseFloatArrayElements(rotation, rot_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(gyroBias, gb_ptr, JNI_ABORT);
    env->ReleaseFloatArrayElements(accelBias, ab_ptr, JNI_ABORT);
}

JNIEXPORT jboolean JNICALL
Java_com_example_navsight1_NativeBridge_getCalibration(
        JNIEnv* env, jobject,
        jfloatArray rotation,
        jfloatArray gyroBias,
        jfloatArray accelBias) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision || !rotation || !gyroBias || !accelBias) return JNI_FALSE;

    if (env->GetArrayLength(rotation) != 9
        || env->GetArrayLength(gyroBias) != 3
        || env->GetArrayLength(accelBias) != 3) {
        return JNI_FALSE;
    }

    float R[9], gb[3], ab[3];
    bool ok = vision->getCalibration(R, gb, ab);
    if (!ok) return JNI_FALSE;

    env->SetFloatArrayRegion(rotation, 0, 9, R);
    env->SetFloatArrayRegion(gyroBias, 0, 3, gb);
    env->SetFloatArrayRegion(accelBias, 0, 3, ab);
    return JNI_TRUE;
}

// Step 6: Horizontal-plane position covariance [σ_xx, σ_xz, σ_zz] in m².
// Returns true when the EKF has finished its full init and the covariance is
// valid; false (zeros) otherwise. Caller passes a length-3 FloatArray.
JNIEXPORT jboolean JNICALL
Java_com_example_navsight1_NativeBridge_getPositionCovariance(
        JNIEnv* env, jobject, jfloatArray out) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision || !out) return JNI_FALSE;
    if (env->GetArrayLength(out) != 3) return JNI_FALSE;

    double cov[3] = {0.0, 0.0, 0.0};
    bool ok = vision->getPositionCovarianceXZ(cov);
    float cov_f[3] = {
        static_cast<float>(cov[0]),
        static_cast<float>(cov[1]),
        static_cast<float>(cov[2])
    };
    env->SetFloatArrayRegion(out, 0, 3, cov_f);
    return ok ? JNI_TRUE : JNI_FALSE;
}

// ── nativeLoadCalibration (Step 1, Visual Production Plan) ──────────────────
// Pushes the in-app calibration JSON into the live VioEngine. Returns true on
// success, false on any failure (file missing, malformed, validation fails).
// On false the engine continues with zero-distortion passthrough — caller
// must NOT block the camera path on this.

JNIEXPORT jboolean JNICALL
Java_com_example_navsight1_NativeBridge_nativeLoadCalibration(
        JNIEnv* env, jobject /* thiz */, jstring jpath) {
    if (!jpath) {
        LOGE("nativeLoadCalibration: null path");
        return JNI_FALSE;
    }
    const char* c_path = env->GetStringUTFChars(jpath, nullptr);
    if (!c_path) {
        LOGE("nativeLoadCalibration: GetStringUTFChars returned null");
        return JNI_FALSE;
    }
    std::string path(c_path);
    env->ReleaseStringUTFChars(jpath, c_path);

    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) {
        LOGI("nativeLoadCalibration: VIO engine not yet started, skipping");
        return JNI_FALSE;
    }
    return vision->loadCalibration(path) ? JNI_TRUE : JNI_FALSE;
}

// ── nativeLoadLoopClosureVocabulary (Step 7, Visual Production Plan) ─────────
// Pushes the on-device path of the ORB DBoW2 vocabulary into the live
// VioEngine's Tracker. The Android side copies assets/ORBvoc.bin →
// <filesDir>/ORBvoc.bin once at app startup (AssetManager paths are not
// real filesystem paths) and passes the absolute path here. Returns true
// on success; false if the file is missing or the vocabulary fails to
// parse — the rest of the pipeline keeps running with loop closure
// disabled (the Tracker's worker thread is never started in that case).

JNIEXPORT jboolean JNICALL
Java_com_example_navsight1_NativeBridge_nativeLoadLoopClosureVocabulary(
        JNIEnv* env, jobject /* thiz */, jstring jpath) {
    if (!jpath) {
        LOGE("nativeLoadLoopClosureVocabulary: null path");
        return JNI_FALSE;
    }
    const char* c_path = env->GetStringUTFChars(jpath, nullptr);
    if (!c_path) {
        LOGE("nativeLoadLoopClosureVocabulary: GetStringUTFChars returned null");
        return JNI_FALSE;
    }
    std::string path(c_path);
    env->ReleaseStringUTFChars(jpath, c_path);

    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) {
        LOGI("nativeLoadLoopClosureVocabulary: VIO engine not yet started, "
             "skipping (caller should retry after startVIO)");
        return JNI_FALSE;
    }
    return vision->loadLoopClosureVocabulary(path) ? JNI_TRUE : JNI_FALSE;
}

// ── EventCounters JNI bridge ──────────────────────────────────────────────────
// Exports two symbols for the simulator-recording path:
//
//   nativeGetEventCountersJson() : returns the EventCounters snapshot as a
//       single-line JSON object string. Called from
//       NavSightViewModel.saveSimulationData just before the
//       simulation_data_<ts>.json file is serialised, so the recording
//       can embed an "event_summary" key alongside "startTime" / "points".
//
//   nativeResetEventCounters()   : zeroes every counter. Called from
//       NavSightViewModel.toggleSimulationRecording on the START leg, so
//       each recording begins with a clean slate.
//
// No mutex needed: EventCounters is a struct of std::atomic<> fields and
// the singleton has process lifetime.

JNIEXPORT jstring JNICALL
Java_com_example_navsight1_NativeBridge_nativeGetEventCountersJson(
        JNIEnv* env, jobject) {
    const std::string json = navsight::eventCounters().serializeAsJsonString();
    return env->NewStringUTF(json.c_str());
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_nativeResetEventCounters(
        JNIEnv*, jobject) {
    navsight::eventCounters().reset();
    LOGI("EventCounters reset");
}

// ── Step 8b: seed body→camera extrinsics rotation ────────────────────────────
// Called once from Kotlin after camera open, using the rotation matrix derived
// from CameraCharacteristics.SENSOR_ORIENTATION (degrees) converted by the
// Kotlin layer into a 3×3 row-major float array.
// R_bc_flat: 9 floats (float[9]).
// No-op if VIO is not yet started or the array has the wrong length.

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_nativeSetExtrinsicsRotation(
        JNIEnv* env, jobject, jfloatArray R_bc_flat) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision || !R_bc_flat) return;
    if (env->GetArrayLength(R_bc_flat) != 9) {
        LOGE("nativeSetExtrinsicsRotation: expected 9-element array, got %d",
             env->GetArrayLength(R_bc_flat));
        return;
    }
    jfloat* ptr = env->GetFloatArrayElements(R_bc_flat, nullptr);
    if (ptr) {
        vision->setExtrinsicsRotation(reinterpret_cast<const float*>(ptr));
        env->ReleaseFloatArrayElements(R_bc_flat, ptr, JNI_ABORT);
        LOGI("nativeSetExtrinsicsRotation: R_bc seeded from Kotlin camera metadata");
    }
}

// 2026-05-25 RE-ENABLED (professor approved continuous compass). The Kotlin
// caller (NativeBridge.setMagnetometerHeading) is live again; this shim MUST be
// compiled + linked or the call hits UnsatisfiedLinkError -> SIGABRT on the
// first mag reading (regression seen 2026-05-25 — symbol was comment-wrapped).
// Uses the shared_ptr pattern (matches setInitialHeading) so the engine stays
// alive if stopVIO races mid-call.
JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setMagnetometerHeading(
        JNIEnv*, jobject /* thiz */, jfloat yawRad) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setMagnetometerHeading(static_cast<float>(yawRad));
    }
}

// 2026-05-26 — #2 loop-overlay path redraw. The UI polls getLoopCorrectionVersion()
// (cheap atomic) on each VIO update; on a change it calls getCorrectedTrajectory(out)
// to rebuild its pathHistory from the CORRECTED pose-graph node polyline so the two
// loops visually overlay (only the now-node delta reaches global_t_). shared_ptr
// pattern so the engine stays alive if stopVIO races mid-call.
JNIEXPORT jint JNICALL
Java_com_example_navsight1_NativeBridge_getLoopCorrectionVersion(
        JNIEnv*, jobject /* thiz */) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) return 0;
    const Tracker* tracker = vision->getTracker();
    return tracker ? static_cast<jint>(tracker->getLoopCorrectionVersion()) : 0;
}

JNIEXPORT jint JNICALL
Java_com_example_navsight1_NativeBridge_getCorrectedTrajectory(
        JNIEnv* env, jobject /* thiz */, jfloatArray out_xz) {
    if (out_xz == nullptr) return 0;
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) return 0;
    const Tracker* tracker = vision->getTracker();
    if (!tracker) return 0;
    const jsize cap = env->GetArrayLength(out_xz);
    const int max_pairs = static_cast<int>(cap / 2);
    if (max_pairs <= 0) return 0;
    std::vector<float> buf(static_cast<size_t>(max_pairs) * 2);
    const int n = tracker->getCorrectedTrajectory(buf.data(), max_pairs);
    if (n > 0) {
        env->SetFloatArrayRegion(out_xz, 0, n * 2, buf.data());
    }
    return static_cast<jint>(n);
}

// 2026-05-26 — locomotion-agnostic reported speed (m/s) for ALL motion types
// (walking, scooter, bike). Depth-weighted metric speed from Tracker::getFusedSpeedMps
// (recoverPose translation scaled by the tracked points' MiDaS depths) — NOT the
// pedestrian step model, independent of the EKF v_G_. -1.0 before the first estimate.
// Same shared_ptr pattern as getLoopCorrectionVersion so the engine stays alive if
// stopVIO races mid-call.
JNIEXPORT jfloat JNICALL
Java_com_example_navsight1_NativeBridge_getFusedSpeedMps(
        JNIEnv*, jobject /* thiz */) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) return -1.0f;
    const Tracker* tracker = vision->getTracker();
    return tracker ? static_cast<jfloat>(tracker->getFusedSpeedMps()) : -1.0f;
}

// 2026-05-28 — cross-app-launch persistence of the MiDaS scale K.
// Without this, every cold start re-pays the K calibration tax; on the
// first recording of a session if essential-matrix verification fails
// (slow walk + close scene), K never calibrates → looming bails → UI
// speed sits at 0 forever. Kotlin (NavSightViewModel) loads K from
// SharedPreferences on init and pushes it here; periodically pulls
// the latest K back to write to prefs.
extern "C" JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setMidasScaleK(
        JNIEnv*, jobject /* thiz */, jdouble k) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) return;
    Tracker* tracker = vision->getTracker();
    if (tracker) tracker->setMidasScaleK(static_cast<double>(k));
}

extern "C" JNIEXPORT jdouble JNICALL
Java_com_example_navsight1_NativeBridge_getMidasScaleK(
        JNIEnv*, jobject /* thiz */) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision) return -1.0;
    const Tracker* tracker = vision->getTracker();
    return tracker ? static_cast<jdouble>(tracker->getMidasScaleK()) : -1.0;
}

// ── Camera overlay Phase 2 / 3 (camera_overlay_phase23_plan.md) ──────────────
//
// Four read-only accessors for the live camera overlay. None of them mutate
// EKF state; they snapshot existing accumulators / EKF state into a flat
// FloatArray (or jlong) the Kotlin overlay can consume cheaply on the camera
// thread before publishing to a Compose state field.
//
// All return zero / false if the VIO engine has not yet been started or the
// EKF has not yet reached full initialisation. Callers must check the return
// value before drawing.

// Phase 2: per-feature ages (frames survived) for the most recent frame.
// Pairs 1:1 with VioData.trackedPoints (length = trackedPoints.length / 2).
// Returns the number of ages written into `out` (≤ out.length). Callers
// preallocate IntArray(MAX_FEATURES) once and reuse.
JNIEXPORT jint JNICALL
Java_com_example_navsight1_NativeBridge_getLastTrackedPointAges(
        JNIEnv* env, jobject, jintArray out) {
    if (!out) return 0;
    const jsize cap = env->GetArrayLength(out);
    if (cap <= 0) return 0;
    std::vector<int> ages_copy;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        ages_copy = g_tracked_point_ages;
    }
    const jsize n = std::min<jsize>(cap, static_cast<jsize>(ages_copy.size()));
    if (n > 0) env->SetIntArrayRegion(out, 0, n, ages_copy.data());
    return static_cast<jint>(n);
}

// Phase 3: flat snapshot of currently-active SLAM features in the EKF.
// Layout per feature (4 floats):
//   [feature_id, world_x, world_y, world_z]
// Coordinates are exposed in Y-up world frame to match the rest of the
// Kotlin pipeline (X=East, Y=Up, Z=North) — same swap as in
// processCameraFrameDirect at line ~414. Returns the number of features
// written (≤ out.length / 4). The EKF caps SLAM features at
// MAX_SLAM_FEATURES = 12, so a 48-float buffer is always sufficient.
// Returns 0 if the EKF has not promoted any SLAM features yet.
JNIEXPORT jint JNICALL
Java_com_example_navsight1_NativeBridge_getSlamSnapshot(
        JNIEnv* env, jobject, jfloatArray out) {
    if (!out) return 0;
    // v23.11: stride increased from 4 → 7 floats per feature to carry
    // the last observed pixel (obs_u, obs_v, has_obs) for the visual
    // debug overlay's residual line.
    const jsize cap = env->GetArrayLength(out) / 7;
    if (cap <= 0) return 0;

    // 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
    // Read the cached EKF snapshot under state_mutex so the SLAM feature world
    // positions come from the SAME EKF state-version that getCurrentCameraPose
    // saw on this overlay tick. The FeatureManager last-obs lookup stays as a
    // separate per-feature read — its data is independent of EKF state and
    // FeatureManager has its own internal mutex.
    std::lock_guard<std::mutex> lock(state_mutex);
    VioEngine* vision_raw = g_vision.get();
    if (!vision_raw) return 0;
    const EKFState::OverlaySnapshot& snap = ensureOverlaySnapshot(vision_raw);
    const Tracker* tracker = vision_raw->getTracker();
    const FeatureManager* fmgr = tracker ? &tracker->getFeatureManager() : nullptr;
    const int n = static_cast<int>(snap.slam_features.size());
    if (n <= 0) return 0;

    /* SUPERSEDED 2026-05-16: single-snapshot consistency fix replaced these
     * per-feature ekf-> accessor calls with one consolidated snapshot read.
     * Math (world-pos derivation) is unchanged — see EKFState::snapshotForOverlay
     * which inlines the same formula getSlamFeatureGlobalPosition uses.
     *
     *   const int n = ekf->getSlamFeatureCount();
     *   for (int i = 0; i < writeCap; ++i) {
     *       if (!ekf->getSlamFeatureGlobalPosition(i, p_global)) continue;
     *       const int fid = ekf->getSlamFeatureId(i);
     *       ...
     *   }
     */

    const jsize writeCap = std::min<jsize>(cap, static_cast<jsize>(n));
    std::vector<float> buf(writeCap * 7, 0.0f);
    int wrote = 0;
    for (int i = 0; i < writeCap; ++i) {
        const auto& f = snap.slam_features[static_cast<size_t>(i)];
        const double zup_x = f.p_world(0);   // East
        const double zup_y = f.p_world(1);   // North (Z-up index 1)
        const double zup_z = f.p_world(2);   // Up    (Z-up index 2)
        const int fid = f.feature_id;
        float obs_u = 0.0f, obs_v = 0.0f;
        bool has_obs = false;
        if (fmgr) {
            has_obs = fmgr->getSlamFeatureLastObs(fid, obs_u, obs_v);
        }
        buf[wrote * 7 + 0] = static_cast<float>(fid);
        buf[wrote * 7 + 1] = static_cast<float>(zup_x);   // East unchanged
        buf[wrote * 7 + 2] = static_cast<float>(zup_z);   // Up   (Z-up Z → Y-up Y)
        buf[wrote * 7 + 3] = static_cast<float>(zup_y);   // North(Z-up Y → Y-up Z)
        buf[wrote * 7 + 4] = obs_u;
        buf[wrote * 7 + 5] = obs_v;
        buf[wrote * 7 + 6] = has_obs ? 1.0f : 0.0f;
        ++wrote;
    }
    if (wrote > 0) env->SetFloatArrayRegion(out, 0, wrote * 7, buf.data());
    return static_cast<jint>(wrote);
    // 2026-05-12: VisualMap-sourced render path reverted — that belongs to
    // Phase 1 Step 6 (persistent landmark map), not a mid-week scaffold.
    // Scaffolding files VisualMap.{h,cpp} retained on disk uncompiled.
}

// Phase 3: current camera pose for SLAM-point reprojection.
// Layout (16 floats):
//   [0..8]   R_world_cam row-major (camera→world rotation, Y-up world)
//   [9..11]  t_world_cam (camera position in Y-up world; ≈ body position —
//            camera-to-body lever arm assumed negligible at typical 1-10 m
//            feature depths)
//   [12..15] fx, fy, cx, cy — same intrinsics the EKF uses for SLAM updates
// Returns true on success, false if EKF not yet full-initialised.
//
// Frame derivation:
//   R_GtoI                        : world Z-up → body
//   R_bc                          : body → camera (OpenCV conv: X=right, Y=down, Z=forward)
//   R_world(Zup)_to_cam = R_bc · R_GtoI
//   R_cam_to_world(Zup) = transpose(R_world(Zup)_to_cam)
// Then permute world rows from Z-up (X,Y,Z = E,N,U) to Y-up (X,Y,Z = E,U,N):
//   row1 ↔ row2 in the 3×3 above so the columns of R_cam_to_world_Yup
//   represent camera unit-axes expressed in the Y-up world frame.
JNIEXPORT jboolean JNICALL
Java_com_example_navsight1_NativeBridge_getCurrentCameraPose(
        JNIEnv* env, jobject, jfloatArray out) {
    if (!out) return JNI_FALSE;
    if (env->GetArrayLength(out) != 16) return JNI_FALSE;

    // 2026-05-16: single-snapshot consistency fix (per ekf-inv-dot-investigation agent)
    // Pull the cached EKF snapshot under state_mutex so both this call and a
    // subsequent getSlamSnapshot within the same overlay tick project from the
    // SAME state-version. See ensureOverlaySnapshot above for the cause/change.
    std::lock_guard<std::mutex> lock(state_mutex);
    VioEngine* vision_raw = g_vision.get();
    const EKFState::OverlaySnapshot& snap = ensureOverlaySnapshot(vision_raw);
    if (!snap.full_initialized) return JNI_FALSE;

    /* SUPERSEDED 2026-05-16: single-snapshot consistency fix replaced these
     * separate ekf-> accessor calls with a single snapshotForOverlay() read.
     * Behaviour unchanged when the snapshot's full_initialized matches the
     * old !isFullInitialized() gate; the only delta is consistency across the
     * subsequent JNI hop. Math below is preserved verbatim but reads from
     * `snap.R_GtoI` / `snap.p_G` / `snap.R_bc` / `snap.slam_*` instead.
     *
     *   const cv::Mat R_GtoI = ekf->getRotation();   // 3×3 CV_64F, world Z-up → body
     *   const cv::Mat p_zup  = ekf->getPosition();   // 3×1 CV_64F, body in world Z-up
     *   if (R_GtoI.rows != 3 || R_GtoI.cols != 3 || R_GtoI.type() != CV_64F) return JNI_FALSE;
     *   if (p_zup.rows != 3 || p_zup.cols != 1 || p_zup.type() != CV_64F) return JNI_FALSE;
     *   const cv::Matx33d R_bc = ekf->getExtrinsicsRotation();
     *   cv::Matx33d R_GtoI_x;
     *   for (int r=0;r<3;++r) for (int c=0;c<3;++c) R_GtoI_x(r,c) = R_GtoI.at<double>(r,c);
     *   buf[9..11] = p_zup.at<double>(0/2/1);
     *   buf[12..15] = ekf->getSlamFx()/Fy/Cx/Cy;
     */

    // R_world(Zup) → camera   = R_bc · R_GtoI
    const cv::Matx33d R_world_to_cam_zup = snap.R_bc * snap.R_GtoI;
    const cv::Matx33d R_cam_to_world_zup = R_world_to_cam_zup.t();

    // Permute world axes Z-up → Y-up by swapping rows 1 and 2.
    // R_cam_to_world rows correspond to world-frame components of the
    // camera basis vectors; row 0 = East (unchanged), row 1 = Up
    // (was row 2 in Z-up), row 2 = North (was row 1 in Z-up).
    cv::Matx33d R_cam_to_world_yup;
    for (int c = 0; c < 3; ++c) {
        R_cam_to_world_yup(0, c) = R_cam_to_world_zup(0, c);   // East
        R_cam_to_world_yup(1, c) = R_cam_to_world_zup(2, c);   // Up   (Z-up row 2)
        R_cam_to_world_yup(2, c) = R_cam_to_world_zup(1, c);   // North(Z-up row 1)
    }

    float buf[16];
    // [0..8] R row-major
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            buf[r * 3 + c] = static_cast<float>(R_cam_to_world_yup(r, c));
        }
    }
    // [9..11] camera position in Y-up world (using body position as
    // approximation; lever arm ≪ feature depth)
    buf[9]  = static_cast<float>(snap.p_G(0));   // East
    buf[10] = static_cast<float>(snap.p_G(2));   // Up    (Z-up Z)
    buf[11] = static_cast<float>(snap.p_G(1));   // North (Z-up Y)
    // [12..15] intrinsics
    buf[12] = static_cast<float>(snap.slam_fx);
    buf[13] = static_cast<float>(snap.slam_fy);
    buf[14] = static_cast<float>(snap.slam_cx);
    buf[15] = static_cast<float>(snap.slam_cy);

    env->SetFloatArrayRegion(out, 0, 16, buf);
    return JNI_TRUE;
}

// Phase 4: cumulative loop-closure correction count from EventCounters.
// Polled per frame by the overlay; an increment triggers the
// "LOOP CLOSURE" 1-second flash banner. Atomic relaxed read — no mutex.
JNIEXPORT jlong JNICALL
Java_com_example_navsight1_NativeBridge_getLoopClosureCorrectionsApplied(
        JNIEnv*, jobject) {
    return static_cast<jlong>(
        navsight::eventCounters().loop_closure_corrections_applied
            .load(std::memory_order_relaxed));
}

// ── Phase 1 Step 6 (post_v19_sprint_plan.md §205-298): LandmarkMap snapshot ───
//
// Returns the persistent visual-only LandmarkMap as a flat float array with
// stride 5 per landmark:
//   [0] = id           (cast from int → float; fits exactly for our id range)
//   [1] = x_yup        (Y-up world X = Z-up world X — East, no swap)
//   [2] = y_yup        (Y-up world Y = Z-up world Z — Up; swap Z-up index 2)
//   [3] = z_yup        (Y-up world Z = Z-up world Y — North; swap Z-up index 1)
//   [4] = observed_this_frame  (1.0f iff id ∈ tracker.getLastObservedLandmarkIds(),
//                               else 0.0f)
//
// Axis swap MUST match `getSlamSnapshot` above (lines 1039-1043) byte-for-byte.
// Camera-overlay reprojection (CameraUi.kt SlamFeatureOverlay) is calibrated to
// that exact swap; any divergence makes the new landmark dots misalign with
// the live SLAM dots that share the same projection pipeline.
//
// Filters:
//   * Skip landmarks with times_observed < 2 (single-observation entries are
//     not yet re-confirmed and would render as ephemeral noise).
//   * Cap output at kMaxOut = 500 landmarks. When the map exceeds this we
//     take the closest 500 to the current camera position via
//     LandmarkMap::getLandmarksInRadius (already sorted ascending squared-
//     distance), bisecting the radius downward until we hit ≤ kMaxOut. This
//     keeps the JNI payload bounded so the overlay's per-frame Kotlin
//     allocation never balloons under loop-closure stress.
//
// Threading: LandmarkMap is internally mutex-protected. The vision pointer is
// snapshotted under state_mutex, then released so the LandmarkMap calls run
// outside our mutex (they take their own lock).
JNIEXPORT jfloatArray JNICALL
Java_com_example_navsight1_NativeBridge_getLandmarkSnapshot(
        JNIEnv* env, jobject) {
    // 2026-05-19 — orange-dot anchor-relative render fix.
    //
    // Take an EKF overlay snapshot under state_mutex BEFORE releasing it, so
    // the clones list we use to look up each landmark's host pose is from
    // the SAME state-version as the camera pose getCurrentCameraPose returns
    // on this overlay tick. Without this single-snapshot guarantee a
    // marginalizeOldestClone() running on the camera thread between our two
    // JNI hops could shift the host clone's R_GtoC / p_G out from under us
    // and the resulting projection would tear.
    std::shared_ptr<VioEngine> vision;
    EKFState::OverlaySnapshot snap;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
        if (!vision) return env->NewFloatArray(0);
        snap = ensureOverlaySnapshot(vision.get());
    }
    const Tracker* tracker = vision->getTracker();
    if (!tracker) {
        return env->NewFloatArray(0);
    }
    // Phase 6.4 deps — if these methods are missing the file fails to compile,
    // which is the deliberate signal that Phase 6.4 has not landed yet.
    const navsight::LandmarkMap& lm = tracker->getLandmarkMap();
    const std::vector<int> observed_ids = tracker->getLastObservedLandmarkIds();

    if (lm.size() == 0) {
        return env->NewFloatArray(0);
    }

    // Build a small unordered_set from observed_ids for O(1) membership tests.
    // Typical size is the per-frame KLT count (≤ ~250), so this is cheap.
    std::unordered_set<int> observed_set;
    observed_set.reserve(observed_ids.size() * 2);
    for (int id : observed_ids) observed_set.insert(id);

    /* SUPERSEDED 2026-05-19 Fix #3 — anchor-relative clone lookup reverted.
     * Reason: the anchor-relative render path correctly compensates for the
     * host clone's pose drift, but the LANDMARK's stored p_anchor_cam is
     * frozen at add time — there is no UpdaterSLAM-equivalent that refines
     * it from current observations. So a clone-pose update by `dx_clone`
     * shifts the rendered position by ~dx_clone with no compensation,
     * giving the same drift pattern as world-fixed. v55 walk: 81162 anchor
     * hits / 35830 world-fixed, anchor ratio 69.4%, user-visible dots
     * still drift. The fix that actually works is to draw OBSERVED dots
     * directly at their KLT-matched pixel — see the obs_u/obs_v pass-
     * through below. Code kept commented per feedback_no_deletions.
     *
     * std::unordered_map<int, const EKFState::OverlaySnapshot::CloneEntry*> clone_lut;
     * clone_lut.reserve(snap.clones.size() * 2);
     * for (const auto& ce : snap.clones) {
     *     clone_lut[ce.state_id] = &ce;
     * }
     */

    // Cap output at kMaxOut. Use a generous initial radius (50 m covers any
    // plausible indoor walk well past the kDepthMaxM = 50.0 LandmarkMap depth
    // band), then truncate to the nearest kMaxOut. LandmarkMap::
    // getLandmarksInRadius already returns ids sorted by ascending squared
    // distance, so the first kMaxOut ARE the closest.
    constexpr int kMaxOut = 500;
    constexpr double kInitialRadiusM = 50.0;

    // Camera position: the same Y-up→Z-up convention used in getCurrentCameraPose
    // (lines 1087-1094). Use the snapshot's p_G so the radius filter is
    // consistent with the camera pose we just snapshotted.
    const cv::Vec3d cam_zup = snap.p_G;

    std::vector<int> ids = lm.getLandmarksInRadius(cam_zup, kInitialRadiusM);
    if (static_cast<int>(ids.size()) > kMaxOut) {
        ids.resize(kMaxOut);
    }

    // 2026-05-19 Fix #3 — stride 5 → 7 floats per landmark to carry the
    // matched-in-this-frame KLT pixel (obs_u, obs_v) for the overlay. When
    // observed_flag == 1 the Kotlin overlay draws the dot AT (obs_u, obs_v)
    // instead of projecting p_world through the live camera pose. This
    // bypasses the stale-stored-p_world drift entirely for the dots the
    // user is most attentive to. obs_u / obs_v default to 0 / 0 when the
    // landmark is not observed this frame (Kotlin treats that as
    // "fall back to projection").
    std::vector<float> buf;
    buf.reserve(ids.size() * 7);
    // One-shot static boundary log: fires once for the entire process
    // lifetime so we can verify the Y-up swap matches getSlamSnapshot on
    // real walks WITHOUT flooding logcat. `static bool` is safe here:
    // the only race is two threads racing the false→true transition,
    // which at worst logs the boundary twice — benign.
    static bool s_logged_boundary = false;
    long long n_anchor_hits = 0;   // 2026-05-19 Fix #3 reuses counters: anchor_hits → obs-pixel-render dots
    long long n_world_fixed = 0;   //                                   world_fixed → fall-back-projection dots
    for (int id : ids) {
        navsight::Landmark lm_rec;
        if (!lm.getLandmark(id, lm_rec)) continue;
        // 2026-05-17: relaxed from times_observed < 2 to allow any landmark
        // that was ever triangulated (investigation agent finding #2).
        // Combined with new touchLandmark API (Tracker.cpp), retention is
        // now driven by re-observation events, not the initial-counter filter.
        if (lm_rec.times_observed < 1) continue;

        // Fix #3 (2026-05-19) — world-fixed render is restored as the
        // baseline path because anchor-relative doesn't actually fix the
        // user-visible drift (see the long comment block above on the
        // commented-out clone_lut). The overlay path that DOES anchor the
        // dot is the obs_u/obs_v pass-through below: observed landmarks
        // draw at their actual KLT-matched pixel, bypassing any projection
        // staleness.
        const cv::Vec3d p_world_render = lm_rec.p_world;

        const double zup_x = p_world_render[0];   // East
        const double zup_y = p_world_render[1];   // North (Z-up index 1)
        const double zup_z = p_world_render[2];   // Up    (Z-up index 2)

        // Y-up swap, byte-for-byte identical to getSlamSnapshot lines 1041-1043:
        //   x_yup ← zup_x  (East unchanged)
        //   y_yup ← zup_z  (Up   : Z-up Z → Y-up Y)
        //   z_yup ← zup_y  (North: Z-up Y → Y-up Z)
        const float x_yup = static_cast<float>(zup_x);
        const float y_yup = static_cast<float>(zup_z);
        const float z_yup = static_cast<float>(zup_y);

        if (!s_logged_boundary) {
            LOGD("JNI_LM_BOUNDARY: id=%d zup=(%.3f,%.3f,%.3f) yup=(%.3f,%.3f,%.3f)",
                 id, zup_x, zup_y, zup_z,
                 static_cast<double>(x_yup), static_cast<double>(y_yup),
                 static_cast<double>(z_yup));
            s_logged_boundary = true;
        }

        const bool is_observed = observed_set.count(id) > 0;
        const float observed_flag = is_observed ? 1.0f : 0.0f;

        // 2026-05-19 Fix #3 — pull matched pixel from Tracker for observed
        // landmarks. When present, the Kotlin overlay uses this pixel
        // directly as the dot's screen position (after the same image→
        // screen transform the SLAM-feature overlay applies). Cause: the
        // projected position via the EKF camera pose drifts by integrated
        // measurement-update shifts; the matched KLT pixel IS the actual
        // image-feature location THIS frame, so drawing there is by-
        // definition anchored. Unobserved dots leave obs_u = obs_v = 0
        // and the Kotlin side falls back to projecting p_world.
        float obs_u = 0.0f;
        float obs_v = 0.0f;
        if (is_observed) {
            if (tracker->getLastObservedLandmarkPixel(id, obs_u, obs_v)) {
                ++n_anchor_hits;   // counted as "obs-pixel render" in the post-walk JSON
            } else {
                // observed_flag set but pixel lookup failed — race or stale
                // observed_set vs. published pixels. Fall back to projection.
                obs_u = 0.0f;
                obs_v = 0.0f;
                ++n_world_fixed;
            }
        } else {
            ++n_world_fixed;
        }

        buf.push_back(static_cast<float>(id));
        buf.push_back(x_yup);
        buf.push_back(y_yup);
        buf.push_back(z_yup);
        buf.push_back(observed_flag);
        buf.push_back(obs_u);
        buf.push_back(obs_v);
    }

    // 2026-05-19 — Publish anchor-vs-world-fixed attribution into the
    // event_summary so post-walk analysis can quantify the fix's effect.
    // Counters are CUMULATIVE across all render calls in a sim, which is
    // what's wanted: the ratio anchor_hits / (anchor_hits + world_fixed)
    // across a whole walk tells us what fraction of orange dot renders
    // benefited from the host-clone lookup. Expect this ratio to be high
    // (most renders go via anchor) once landmarks are routinely produced
    // with has_anchor=true and host clones live long enough to be looked
    // up. If the ratio is low even on a fresh walk, host clones are
    // marginalising out before the overlay paints — that's the signal to
    // add reanchor at margin time (Fix #2b).
    auto& ec_lm_render = navsight::eventCounters();
    if (n_anchor_hits > 0) {
        ec_lm_render.landmarks_rendered_anchor_total.fetch_add(
            n_anchor_hits, std::memory_order_relaxed);
    }
    if (n_world_fixed > 0) {
        ec_lm_render.landmarks_rendered_world_fixed_total.fetch_add(
            n_world_fixed, std::memory_order_relaxed);
    }
    // One-shot anchor-boundary log so a real walk's logcat confirms the
    // anchor path actually fired at least once with sane numbers. Static
    // bool race is benign (worst case: logs twice).
    static bool s_logged_anchor = false;
    if (!s_logged_anchor && n_anchor_hits > 0) {
        LOGI("JNI_LM_ANCHOR_BOUNDARY: anchor_hits=%lld world_fixed=%lld "
             "n_clones_in_snap=%zu",
             n_anchor_hits, n_world_fixed, snap.clones.size());
        s_logged_anchor = true;
    }

    jfloatArray out = env->NewFloatArray(static_cast<jsize>(buf.size()));
    if (!out) return nullptr;
    if (!buf.empty()) {
        env->SetFloatArrayRegion(out, 0, static_cast<jsize>(buf.size()), buf.data());
    }
    return out;
}

} // extern "C"

