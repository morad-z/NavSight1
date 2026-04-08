#include <jni.h>
#include <android/log.h>
#include <string>
#include <mutex>
#include <cstdint>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include "VioEngine.h"

#define TAG "NavSight-Native"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ── Global state ──────────────────────────────────────────────────────────────

static std::mutex  state_mutex;
static VioEngine* g_vision = nullptr;

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

extern "C" {

// ── startVIO ──────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_startVIO(JNIEnv*, jobject) {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (g_vision) {
        delete g_vision;
        g_vision = nullptr;
    }
    g_vision = new VioEngine();
    g_vision->setUserScaleCorrection(g_user_scale_correction);
    resetPoseState();
    LOGI("VIO started");
}

// ── stopVIO ───────────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setImuNoiseParameters(JNIEnv*, jobject, 
                                                            jfloat accel_noise, jfloat gyro_noise,
                                                            jfloat accel_rw, jfloat gyro_rw) {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (g_vision) {
        g_vision->getIMU().setNoiseParameters(accel_noise, gyro_noise, accel_rw, gyro_rw);
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_stopVIO(JNIEnv*, jobject) {
    VioEngine* to_delete = nullptr;
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
    VioEngine* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }

    // Run VIO pipeline outside the state lock (VioEngine has its own locks)
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


    // ── Build VioData Java object (using cached class/ctor from JNI_OnLoad) ───
    if (!g_viodata_cls || !g_viodata_ctor) {
        LOGE("processCameraFrame: VioData JNI cache not initialized");
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
        jlong timestamp) {

    VioEngine* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }

    VisionOutput output{};
    output.valid = false;

    if (vision && width > 0 && height > 0 && yBuffer && uvBuffer) {
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

    VioEngine* vision = nullptr;
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

    VioEngine* vision = nullptr;
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
    VioEngine* vision = nullptr;
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
    VioEngine* vision = nullptr;
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
    VioEngine* vision = nullptr;
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
    VioEngine* vision = nullptr;
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
    VioEngine* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setInitialHeading(static_cast<double>(azimuthRad));
    }
}

// ── setUserHeight ────────────────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setUserHeight(
        JNIEnv*, jobject /* thiz */, jfloat heightM) {
    VioEngine* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setUserHeight(static_cast<float>(heightM));
    }
}

// ── setMagnetometerHeading ───────────────────────────────────────────────────

JNIEXPORT void JNICALL
Java_com_example_navsight1_NativeBridge_setMagnetometerHeading(
        JNIEnv*, jobject /* thiz */, jfloat yawRad) {
    VioEngine* vision = nullptr;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (vision) {
        vision->setMagnetometerHeading(static_cast<float>(yawRad));
    }
}

} // extern "C"

