#include <jni.h>
#include <android/log.h>
#include <vector>
#include <mutex>
#include <cmath>

#define TAG "NavSight-Native"

static std::mutex state_mutex;

// --- Pose ---
static double g_x = 0.0;
static double g_y = 0.0;
static double g_z = 0.0;
static double g_roll = 0.0;
static double g_pitch = 0.0;
static double g_yaw = 0.0;

// --- Tracking ---
static std::vector<float> g_points;
static double g_quality = 0.0;
static int g_tracked = 0;
static int g_total = 200;
static double g_scale = 1.0;
static bool g_initialized = false;

// --- IMU ---
static float g_ax = 0, g_ay = 0, g_az = 0;
static float g_gx = 0, g_gy = 0, g_gz = 0;

extern "C" {

// ================= CAMERA =================

JNIEXPORT jobject JNICALL
Java_com_example_navsight1_MainActivity_processCameraFrame(
        JNIEnv *env,
        jobject /* this */,
        jbyteArray /* frameData */,
        jint /* width */,
        jint /* height */,
        jlong /* timestamp */) {

    std::lock_guard<std::mutex> lock(state_mutex);

    jclass cls = env->FindClass("com/example/navsight1/VioData");
    if (!cls) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "VioData class not found");
        return nullptr;
    }

    // ⚠️ MUST MATCH Kotlin constructor EXACTLY
    jmethodID ctor = env->GetMethodID(
            cls,
            "<init>",
            "(DDDDDD[FDIIDZFFFFFF)V"
    );

    if (!ctor) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "VioData constructor not found");
        return nullptr;
    }

    jfloatArray pointsArray = env->NewFloatArray(g_points.size());
    if (pointsArray && !g_points.empty()) {
        env->SetFloatArrayRegion(pointsArray, 0, g_points.size(), g_points.data());
    }

    return env->NewObject(
            cls,
            ctor,
            g_x,
            g_y,
            g_z,
            g_roll,
            g_pitch,
            g_yaw,
            pointsArray,
            g_quality,
            g_tracked,
            g_total,
            g_scale,
            g_initialized,
            g_ax,
            g_ay,
            g_az,
            g_gx,
            g_gy,
            g_gz
    );
}

// ================= IMU =================

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processAccelerometer(
        JNIEnv *, jobject, jlong, jfloat x, jfloat y, jfloat z) {

    std::lock_guard<std::mutex> lock(state_mutex);
    g_ax = x;
    g_ay = y;
    g_az = z;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processGyroscope(
        JNIEnv *, jobject, jlong, jfloat x, jfloat y, jfloat z) {

    std::lock_guard<std::mutex> lock(state_mutex);
    g_gx = x;
    g_gy = y;
    g_gz = z;
}

// ================= CONTROL =================

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_resetVIO(JNIEnv *, jobject) {
    std::lock_guard<std::mutex> lock(state_mutex);

    g_x = g_y = g_z = 0.0;
    g_roll = g_pitch = g_yaw = 0.0;
    g_points.clear();
    g_quality = 0.0;
    g_tracked = 0;
    g_initialized = false;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_startVIO(JNIEnv *, jobject) {}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_stopVIO(JNIEnv *, jobject) {}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_setScale(JNIEnv *, jobject, jdouble scale) {
    g_scale = scale;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_pingNative(JNIEnv *, jobject) {
    __android_log_print(ANDROID_LOG_INFO, TAG, "Ping native OK");
}

}
