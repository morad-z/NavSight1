#include <jni.h>
#include <android/log.h>
#include <string>
#include <mutex>
#include <cstdint>
#include <cmath>
#include <opencv2/calib3d.hpp>
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
    // keep g_scale
}

extern "C" {

// ── startVIO ──────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_startVIO(JNIEnv*, jobject) {
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
Java_com_example_navsight1_NativeBridge_stopVIO(JNIEnv*, jobject) {
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
Java_com_example_navsight1_NativeBridge_processCameraFrame(
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
            g_x = output.t.at<double>(0);
            g_y = output.t.at<double>(1);
            g_z = output.t.at<double>(2);
            g_scale = output.estimatedScale; 
            
            // RAW VO
            g_raw_x = output.rawT.empty() ? 0.0 : output.rawT.at<double>(0);
            g_raw_y = output.rawT.empty() ? 0.0 : output.rawT.at<double>(1);
            g_raw_z = output.rawT.empty() ? 0.0 : output.rawT.at<double>(2);
            
            if (!output.rawR.empty()) {
                cv::Mat rv;
                cv::Rodrigues(output.rawR, rv);
                g_raw_yaw = rv.at<double>(1);
            }

            // Diagnostic fields
            g_mean_flow = output.meanFlow;
            g_inlier_count = output.inlierCount;
            g_step_count = output.stepCount;
            g_step_freq = output.stepFreq;
            g_stride_length = output.strideLength;
            g_pose_flags = output.poseFlags;
            g_heading = output.heading;

            // Derive Euler angles from the GLOBAL rotation for display
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


    // ── Build VioData Java object ──────────────────────────────────────────────
    jclass cls = env->FindClass("com/example/navsight1/VioData");
    if (!cls) {
        LOGE("processCameraFrame: VioData class not found");
        return nullptr;
    }

    // Signature: base fields + RAW VO + IMU + diagnostics (meanFlow,inlierCount,stepCount,stepFreq,strideLength,poseFlags,heading)
    const char* vio_sig = "(DDDDDDDIIDZ[FDDDDFFFFFFDIIDDID)V";
    jmethodID ctor = env->GetMethodID(cls, "<init>", vio_sig);
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

    // Snapshot state for return
    double ret_x, ret_y, ret_z, ret_roll, ret_pitch, ret_yaw;
    double ret_quality, ret_scale;
    double ret_rx, ret_ry, ret_rz, ret_ryaw;
    int    ret_tracked, ret_total;
    jboolean ret_initialized;
    float  ret_ax, ret_ay, ret_az, ret_gx, ret_gy, ret_gz;
    double ret_mean_flow, ret_step_freq, ret_stride_length, ret_heading;
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
    }
    ret_quality = output.quality;
    ret_tracked = output.trackedCount;
    ret_total = output.totalCount;
    ret_initialized = static_cast<jboolean>(output.valid);

    return env->NewObject(
            cls, ctor,
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
            ret_pose_flags, ret_heading);
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
Java_com_example_navsight1_NativeBridge_processAccelerometer(
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
Java_com_example_navsight1_NativeBridge_resetVIO(JNIEnv*, jobject) {
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
Java_com_example_navsight1_NativeBridge_setScale(
        JNIEnv*, jobject /* thiz */, jdouble scale) {
    std::lock_guard<std::mutex> lock(state_mutex);
    g_scale = scale;
}

// ── setIntrinsics ─────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setIntrinsics(
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

// ── setInitialHeading ────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setInitialHeading(
        JNIEnv*, jobject /* thiz */, jdouble azimuthRad) {
    VisionModule* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setInitialHeading(static_cast<double>(azimuthRad));
    }
}

} // extern "C"

