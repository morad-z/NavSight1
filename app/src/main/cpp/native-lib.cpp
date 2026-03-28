#include <jni.h>
#include <android/log.h>
#include <string>
#include <mutex>
#include <cstdint>
#include <cmath>
#include "VisionModule.h"

#define TAG "NavSight-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ── Global state ──────────────────────────────────────────────────────────────

static std::mutex  state_mutex;
static VisionModule* g_vision = nullptr;

// Accumulated pose
static double g_x = 0, g_y = 0, g_z = 0;
static double g_roll = 0, g_pitch = 0, g_yaw = 0;
static double g_scale = 1.0;

// Last sensor values
static float g_ax = 0, g_ay = 0, g_az = 0;
static float g_gx = 0, g_gy = 0, g_gz = 0;

// ── resetPoseState ────────────────────────────────────────────────────────────
// Must be called with state_mutex held.

static void resetPoseState() {
    g_x = g_y = g_z = 0;
    g_roll = g_pitch = g_yaw = 0;
    g_ax = g_ay = g_az = 0;
    g_gx = g_gy = g_gz = 0;
    // keep g_scale
}

extern "C" {

// ── startVIO ──────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_startVIO(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (g_vision) {
        delete g_vision;
        g_vision = nullptr;
    }
    g_vision = new VisionModule();
    resetPoseState();
    LOGI("VIO started");
}

// ── stopVIO ───────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_stopVIO(JNIEnv*, jobject) {
    VisionModule* to_delete = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        to_delete = g_vision;
        g_vision  = nullptr;
    }
    // Delete outside the lock to avoid holding it during destruction
    delete to_delete;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        resetPoseState();
    }
    LOGI("VIO stopped");
}

// ── processCameraFrame ────────────────────────────────────────────────────────

JNIEXPORT jobject JNICALL
Java_com_example_navsight1_MainActivity_processCameraFrame(
        JNIEnv* env,
        jobject /* thiz */,
        jbyteArray frameData,
        jint width,
        jint height,
        jlong timestamp) {

    // Snapshot g_vision pointer under lock to prevent use-after-free with stopVIO
    VisionModule* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }

    // Run VIO pipeline outside the state lock (VisionModule has its own locks)
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

    // Update accumulated pose under lock
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        if (output.valid && !output.R.empty() && !output.t.empty()) {
            g_x += output.t.at<double>(0);
            g_y += output.t.at<double>(1);
            g_z += output.t.at<double>(2);

            // Derive Euler angles from the incremental rotation for display
            const cv::Mat& R = output.R;
            double pitch = std::asin(-R.at<double>(2, 0));
            double roll, yaw;
            if (std::abs(std::cos(pitch)) > 1e-6) {
                roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
                yaw  = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
            } else {
                roll = 0.0;
                yaw  = std::atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
            }
            g_roll  = roll;
            g_pitch = pitch;
            g_yaw   = yaw;
        }
    }

    // ── Build VioData Java object ──────────────────────────────────────────────
    jclass cls = env->FindClass("com/example/navsight1/VioData");
    if (!cls) {
        LOGE("processCameraFrame: VioData class not found");
        return nullptr;
    }

    // Signature must match VioData primary constructor field order exactly:
    // x(D) y(D) z(D) roll(D) pitch(D) yaw(D) trackingQuality(D)
    // trackedFeatures(I) totalFeatures(I) estimatedScale(D)
    // isInitialized(Z) trackedPoints([F)
    // accelX(F) accelY(F) accelZ(F) gyroX(F) gyroY(F) gyroZ(F)
    jmethodID ctor = env->GetMethodID(cls, "<init>", "(DDDDDDDIIDZ[FFFFFF)V");
    if (!ctor) {
        LOGE("processCameraFrame: VioData constructor not found");
        return nullptr;
    }

    // Build the tracked-points float array — null-check before use
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

    // Read latest accumulated state under lock for the return value
    double ret_x, ret_y, ret_z, ret_roll, ret_pitch, ret_yaw;
    double ret_quality, ret_scale;
    int    ret_tracked, ret_total;
    jboolean ret_initialized;
    float  ret_ax, ret_ay, ret_az, ret_gx, ret_gy, ret_gz;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        ret_x    = g_x;   ret_y    = g_y;   ret_z    = g_z;
        ret_roll = g_roll; ret_pitch = g_pitch; ret_yaw = g_yaw;
        ret_scale = g_scale;
        ret_ax = g_ax; ret_ay = g_ay; ret_az = g_az;
        ret_gx = g_gx; ret_gy = g_gy; ret_gz = g_gz;
    }
    ret_quality     = output.quality;
    ret_tracked     = output.trackedCount;
    ret_total       = output.totalCount > 0 ? output.totalCount : 200;
    ret_initialized = static_cast<jboolean>(output.valid);

    LOGD("Pose: x=%.3f y=%.3f z=%.3f yaw=%.3f tracked=%d quality=%.2f",
         ret_x, ret_y, ret_z, ret_yaw, ret_tracked, ret_quality);

    return env->NewObject(
            cls, ctor,
            (jdouble)ret_x, (jdouble)ret_y, (jdouble)ret_z,
            (jdouble)ret_roll, (jdouble)ret_pitch, (jdouble)ret_yaw,
            (jdouble)ret_quality,
            (jint)ret_tracked, (jint)ret_total,
            (jdouble)ret_scale,
            ret_initialized,
            pointsArray,
            (jfloat)ret_ax, (jfloat)ret_ay, (jfloat)ret_az,
            (jfloat)ret_gx, (jfloat)ret_gy, (jfloat)ret_gz);
}

// ── processGyroscope ──────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processGyroscope(
        JNIEnv*, jobject /* thiz */,
        jlong timestamp, jfloat x, jfloat y, jfloat z) {

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("processGyroscope: NaN/Inf value, dropping");
        return;
    }

    VisionModule* vision = nullptr;
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
Java_com_example_navsight1_MainActivity_processAccelerometer(
        JNIEnv*, jobject /* thiz */,
        jlong timestamp, jfloat x, jfloat y, jfloat z) {

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        LOGE("processAccelerometer: NaN/Inf value, dropping");
        return;
    }

    VisionModule* vision = nullptr;
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
Java_com_example_navsight1_MainActivity_resetVIO(JNIEnv*, jobject) {
    VisionModule* vision = nullptr;
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
Java_com_example_navsight1_MainActivity_setScale(
        JNIEnv*, jobject /* thiz */, jdouble scale) {
    std::lock_guard<std::mutex> lock(state_mutex);
    g_scale = scale;
}

// ── setIntrinsics ─────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_setIntrinsics(
        JNIEnv*, jobject /* thiz */,
        jdouble fx, jdouble fy, jdouble cx, jdouble cy) {
    VisionModule* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setIntrinsics(fx, fy, cx, cy);
    }
}

} // extern "C"
