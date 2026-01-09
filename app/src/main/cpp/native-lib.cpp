#include <jni.h>
#include <string>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <android/log.h>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include "VisionModule.h"

JavaVM* g_jvm = nullptr;

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    g_jvm = vm;
    return JNI_VERSION_1_6;
}

#define TAG "NavSight-Native"

// RAII wrapper for JNI thread attachment/detachment to prevent memory leaks
struct JNIThreadGuard {
    JNIEnv* env;
    bool attached;
    JavaVM* jvm;

    JNIThreadGuard(JavaVM* vm, JNIEnv* current_env) : jvm(vm), env(current_env), attached(false) {
        if (!jvm) {
            env = nullptr;
            return;
        }
        if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
            if (jvm->AttachCurrentThread(&env, nullptr) == JNI_OK) {
                attached = true;
            } else {
                env = nullptr;
            }
        }
    }

    ~JNIThreadGuard() {
        if (attached && jvm) {
            jvm->DetachCurrentThread();
        }
    }

    JNIEnv* getEnv() const { return env; }
};

// --- Threading and Data Synchronization ---
std::thread vio_thread;

// Protects: vision_module pointer, latest_global_R, latest_global_t, latest_points
// Acquire order: Always acquire vio_mutex before accessing vision_module or global pose state
std::mutex vio_mutex;

// --- Threading and Data Synchronization ---
std::thread vio_thread;
std::mutex vio_mutex;
std::condition_variable vio_cv;
bool running = false;

struct CamFrame {
    jlong timestamp;
    cv::Mat yuvMat;
    int width;
    int height;
};

// Thread-safe queues for sensor data
// VIO systems need low latency for IMU-visual synchronization
// Queue size of 2 provides double-buffering while minimizing staleness
const size_t MAX_FRAME_QUEUE_SIZE = 2;  // Optimized for real-time VIO (not 5)
std::queue<CamFrame> frame_queue;
std::mutex frame_queue_mutex;

std::mutex accel_queue_mutex;

// Latest calculated pose (protected by vio_mutex)
cv::Mat latest_global_R = cv::Mat::eye(3, 3, CV_64F);
cv::Mat latest_global_t = cv::Mat::zeros(3, 1, CV_64F);
std::vector<float> latest_points;

// Latest IMU readings (protected by their respective mutexes)
std::mutex latest_imu_mutex;
float latest_accel_x = 0.0f, latest_accel_y = 0.0f, latest_accel_z = 0.0f;
float latest_gyro_x = 0.0f, latest_gyro_y = 0.0f, latest_gyro_z = 0.0f;


// --- VIO State Variables (shared_ptr for thread-safe lifecycle management) ---
static std::shared_ptr<navsight::VisionModule> vision_module;
// --- VIO State Variables (used only by VIO thread) ---
static navsight::VisionModule* vision_module = nullptr;
static cv::Mat global_R = cv::Mat::eye(3, 3, CV_64F);
static cv::Mat global_t = cv::Mat::zeros(3, 1, CV_64F);
static double g_scale = 1.0;

// Buffer for accel data for reset
const int ACCEL_BUFFER_SIZE = 100;
static std::vector<cv::Point3f> accel_buffer_for_reset;

// Helper function to convert rotation matrix to Euler angles (roll, pitch, yaw)
cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat &R) {
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
}

void vio_thread_loop() {
    // Initialize VisionModule with thread-safe shared_ptr
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        if (!vision_module) {
            try {
                vision_module = std::make_shared<navsight::VisionModule>(
                    navsight::FeatureDetectorType::GOOD_FEATURES, 200);
                __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule created successfully");
            } catch (const std::exception& e) {
                __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to create VisionModule: %s", e.what());
                return;
            } catch (...) {
                __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to create VisionModule: unknown error");
                return;
            }
    // Initialize VisionModule
    if (!vision_module) {
        try {
            vision_module = new navsight::VisionModule(navsight::FeatureDetectorType::GOOD_FEATURES, 200);
            __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule created successfully");
        } catch (const std::exception& e) {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to create VisionModule: %s", e.what());
            return;
        } catch (...) {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to create VisionModule: unknown error");
            return;
        }
    }

    while (running) {
        CamFrame frame;
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex);
            vio_cv.wait(lock, [] { return !frame_queue.empty() || !running; });
            if (!running) break;
            frame = frame_queue.front();
            frame_queue.pop();
        }

        __android_log_print(ANDROID_LOG_INFO, TAG, "VIO Thread: Processing frame %ld", (long)frame.timestamp);

        // Safety check
        if (!vision_module) {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "VisionModule is null!");
            continue;
        }

        // Process frame using VisionModule
        navsight::VisionOutput vision_output;
        try {
            vision_output = vision_module->processFrame(
                frame.yuvMat.data,
                frame.width,
                frame.height,
                frame.timestamp
            );
        } catch (const std::exception& e) {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "processFrame failed: %s", e.what());
            continue;
        }

        // Prepare points for UI
        std::vector<float> points_for_ui;
        points_for_ui.reserve(vision_output.tracked_points.size() * 2);
        for(const auto& p : vision_output.tracked_points) {
            points_for_ui.push_back(p.x);
            points_for_ui.push_back(p.y);
        }

        // Update global pose if vision output is valid
        if (vision_output.is_valid) {
            std::lock_guard<std::mutex> lock(vio_mutex);

            // Get estimated scale from VisionModule (or use manual scale if not estimated yet)
            double scale_to_use = vision_module->getEstimatedScale();
            if (scale_to_use < 0.01) {
                scale_to_use = g_scale;  // Fall back to manual scale if auto-scale not ready
            }

            // Update global translation: t_global = t_global + scale * (R_global * t_relative)
            global_t = global_t + (scale_to_use * (global_R * vision_output.translation));

            // Update global rotation: R_global = R_relative * R_global
            global_R = vision_output.rotation * global_R;

            latest_global_t = global_t.clone();
            latest_global_R = global_R.clone();
            latest_points = points_for_ui;

            __android_log_print(ANDROID_LOG_DEBUG, TAG,
                "Pose updated - Position: (%.2f, %.2f, %.2f), Quality: %.2f, Scale: %.3f",
                global_t.at<double>(0), global_t.at<double>(1), global_t.at<double>(2),
                vision_output.tracking_quality, scale_to_use);
        } else {
            std::lock_guard<std::mutex> lock(vio_mutex);
            latest_points = points_for_ui;

            __android_log_print(ANDROID_LOG_DEBUG, TAG,
                "Vision output not valid - Tracked: %d/%d",
                vision_output.tracked_features, vision_output.total_features);
        }
    }

    // Log statistics when thread exits
    if (vision_module) {
        auto stats = vision_module->getStatistics();
        __android_log_print(ANDROID_LOG_INFO, TAG,
            "VisionModule Stats - Frames: %d, Success: %d, Avg Features: %.1f, Avg Quality: %.2f",
            stats.total_frames_processed, stats.successful_tracks,
            stats.average_features_tracked, stats.average_tracking_quality);
    }
}

extern "C" {

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_startVIO(JNIEnv *env, jobject /* this */) {
    if (running) return;
    running = true;
    vio_thread = std::thread(vio_thread_loop);
    __android_log_print(ANDROID_LOG_INFO, TAG, "VIO thread started.");
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_stopVIO(JNIEnv *env, jobject /* this */) {
    if (!running) return;
    running = false;
    vio_cv.notify_one();
    if (vio_thread.joinable()) {
        vio_thread.join();
    }

    // Clean up VisionModule (thread-safe with shared_ptr)
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        if (vision_module) {
            vision_module.reset();
            __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule destroyed");
        }
    }

    // Clear frame queue to prevent memory leaks from unprocessed frames
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex);
        while (!frame_queue.empty()) {
            frame_queue.pop();
        }
        __android_log_print(ANDROID_LOG_INFO, TAG, "Frame queue cleared");
    }

    __android_log_print(ANDROID_LOG_INFO, TAG, "VIO thread stopped.");
}

JNIEXPORT jobject JNICALL
Java_com_example_navsight1_MainActivity_processCameraFrame(
        JNIEnv* env, jobject /* this */, jbyteArray frameData, jint width, jint height, jlong timestamp) {

    // Use RAII guard to ensure thread detachment on all exit paths
    JNIThreadGuard jni_guard(g_jvm, env);
    JNIEnv* jni_env = jni_guard.getEnv();
    if (!jni_env) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to get JNI environment");
        return nullptr;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_stopVIO(JNIEnv *env, jobject /* this */) {
    if (!running) return;
    running = false;
    vio_cv.notify_one();
    if (vio_thread.joinable()) {
        vio_thread.join();
    }

    // Clean up VisionModule
    if (vision_module) {
        delete vision_module;
        vision_module = nullptr;
        __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule destroyed");
    }

    __android_log_print(ANDROID_LOG_INFO, TAG, "VIO thread stopped.");
}

JNIEXPORT jobject JNICALL
Java_com_example_navsight1_MainActivity_processCameraFrame(
        JNIEnv* env, jobject /* this */, jbyteArray frameData, jint width, jint height, jlong timestamp) {
    
    JNIEnv* jni_env = nullptr;
    bool attached = false;
    if (g_jvm->GetEnv(reinterpret_cast<void**>(&jni_env), JNI_VERSION_1_6) != JNI_OK) {
        if (g_jvm->AttachCurrentThread(&jni_env, nullptr) == JNI_OK) {
            attached = true;
        } else {
            return nullptr;
        }
    } else {
        jni_env = env;
    }

    jbyte* buffer = jni_env->GetByteArrayElements(frameData, nullptr);
    if (buffer != nullptr) {
        cv::Mat yuvMat(height + height / 2, width, CV_8UC1, (unsigned char*)buffer);
        {
            std::lock_guard<std::mutex> lock(frame_queue_mutex);
            // Drop oldest frame if queue is full to prevent memory exhaustion
            if (frame_queue.size() >= MAX_FRAME_QUEUE_SIZE) {
                frame_queue.pop();
                __android_log_print(ANDROID_LOG_WARN, TAG,
                    "Frame queue full (%zu frames), dropping oldest", MAX_FRAME_QUEUE_SIZE);
            }
            frame_queue.push({timestamp, yuvMat.clone(), width, height});
            vio_cv.notify_one();
        }
        std::lock_guard<std::mutex> lock(frame_queue_mutex);
        frame_queue.push({timestamp, yuvMat.clone(), width, height});
        vio_cv.notify_one();
        jni_env->ReleaseByteArrayElements(frameData, buffer, JNI_ABORT);
    }

    jclass vioDataClass = jni_env->FindClass("com/example/navsight1/VioData");
    jmethodID vioDataConstructor = jni_env->GetMethodID(vioDataClass, "<init>", "(DDDDDD[FDIIDZFFFFFF)V");

    cv::Mat current_R, current_t;
    std::vector<float> current_points;
    double tracking_quality = 0.0;
    int tracked_features = 0;
    int total_features = 0;
    double estimated_scale = 1.0;
    bool is_initialized = false;

    // Get a local copy of vision_module to safely access outside lock
    std::shared_ptr<navsight::VisionModule> local_module;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        current_R = latest_global_R.clone();
        current_t = latest_global_t.clone();
        current_points = latest_points;
        local_module = vision_module;
    }

    // Access VisionModule stats safely using local copy
    if (local_module) {
        auto stats = local_module->getStatistics();
        tracking_quality = stats.average_tracking_quality;
        tracked_features = (int)stats.average_features_tracked;
        total_features = 200; // max_features
        estimated_scale = local_module->getEstimatedScale();
        is_initialized = local_module->isInitialized();

        // Get VisionModule stats if available
        if (vision_module) {
            auto stats = vision_module->getStatistics();
            tracking_quality = stats.average_tracking_quality;
            tracked_features = (int)stats.average_features_tracked;
            total_features = 200; // max_features
            estimated_scale = vision_module->getEstimatedScale();
            is_initialized = vision_module->isInitialized();
        }
    }
    cv::Vec3d eulerAngles = rotationMatrixToEulerAngles(current_R);

    // Get latest IMU readings
    float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    {
        std::lock_guard<std::mutex> lock(latest_imu_mutex);
        accel_x = latest_accel_x;
        accel_y = latest_accel_y;
        accel_z = latest_accel_z;
        gyro_x = latest_gyro_x;
        gyro_y = latest_gyro_y;
        gyro_z = latest_gyro_z;
    }

    jfloatArray pointsArray = jni_env->NewFloatArray(current_points.size());
    if (!pointsArray) {
        __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to allocate float array for tracked points");
        // Return default VioData with empty points array
        jfloatArray emptyArray = jni_env->NewFloatArray(0);
        if (!emptyArray) {
            return nullptr; // Critical failure
        }
        return jni_env->NewObject(vioDataClass, vioDataConstructor,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, emptyArray,
            0.0, 0, 0, 0.0, false,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    if (!current_points.empty()) {
    if (pointsArray != nullptr && !current_points.empty()) {
        jni_env->SetFloatArrayRegion(pointsArray, 0, current_points.size(), current_points.data());
    }

    jobject result = jni_env->NewObject(vioDataClass, vioDataConstructor,
                          current_t.at<double>(0), current_t.at<double>(1), current_t.at<double>(2),
                          eulerAngles[0], eulerAngles[1], eulerAngles[2], pointsArray,
                          tracking_quality, tracked_features, total_features, estimated_scale, is_initialized,
                          accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

    // JNIThreadGuard destructor will automatically detach thread if needed
    if (attached) {
        g_jvm->DetachCurrentThread();
    }

    return result;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processAccelerometer(
        JNIEnv* env, jobject /* this */, jlong timestamp, jfloat x, jfloat y, jfloat z) {

    // Store latest reading for display
    {
        std::lock_guard<std::mutex> lock(latest_imu_mutex);
        latest_accel_x = x;
        latest_accel_y = y;
        latest_accel_z = z;
    }

    // Store for reset functionality
    {
        std::lock_guard<std::mutex> lock(accel_queue_mutex);
        accel_buffer_for_reset.push_back(cv::Point3f(x, y, z));
        if (accel_buffer_for_reset.size() > ACCEL_BUFFER_SIZE) {
            accel_buffer_for_reset.erase(accel_buffer_for_reset.begin());
        }
    }

    // Pass to VisionModule for scale estimation and initialization (thread-safe)
    std::shared_ptr<navsight::VisionModule> local_module;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        local_module = vision_module;
    }
    if (running && local_module) {
    // Pass to VisionModule for scale estimation and initialization
    // Check if VIO is running before accessing vision_module
    if (running && vision_module) {
        navsight::AccelData accel;
        accel.timestamp_ns = timestamp;
        accel.x = x;
        accel.y = y;
        accel.z = z;
        local_module->addAccelData(accel);
        vision_module->addAccelData(accel);
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_resetVIO(JNIEnv *env, jobject /* this */) {
    std::lock_guard<std::mutex> lock(vio_mutex);
    if (accel_buffer_for_reset.empty()) {
        __android_log_print(ANDROID_LOG_WARN, TAG, "Reset VIO called, but accel buffer is empty.");
        return;
    }

    cv::Point3f avg_accel(0, 0, 0);
    {
        std::lock_guard<std::mutex> accel_lock(accel_queue_mutex);
        for (const auto& reading : accel_buffer_for_reset) {
            avg_accel += reading;
        }
        avg_accel.x /= accel_buffer_for_reset.size();
        avg_accel.y /= accel_buffer_for_reset.size();
        avg_accel.z /= accel_buffer_for_reset.size();
    }

    double norm = cv::norm(avg_accel);
    avg_accel.x /= norm;
    avg_accel.y /= norm;
    avg_accel.z /= norm;
    double roll = atan2(avg_accel.y, avg_accel.z) * 180.0 / M_PI;
    double pitch = atan2(-avg_accel.x, sqrt(avg_accel.y * avg_accel.y + avg_accel.z * avg_accel.z)) * 180.0 / M_PI;
    __android_log_print(ANDROID_LOG_INFO, TAG, "VIO Reset. Initial orientation: Roll=%.2f, Pitch=%.2f", roll, pitch);

    // Reset global pose
    global_R = cv::Mat::eye(3, 3, CV_64F);
    global_t = cv::Mat::zeros(3, 1, CV_64F);
    latest_global_R = global_R.clone();
    latest_global_t = global_t.clone();
    latest_points.clear();

    // Reset VisionModule
    if (vision_module) {
        vision_module->reset();
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processGyroscope(
        JNIEnv* env, jobject /* this */, jlong timestamp, jfloat x, jfloat y, jfloat z) {
    // Store latest reading for display
    {
        std::lock_guard<std::mutex> lock(latest_imu_mutex);
        latest_gyro_x = x;
        latest_gyro_y = y;
        latest_gyro_z = z;
    }

    // Pass gyro data to VisionModule for sensor fusion (thread-safe)
    std::shared_ptr<navsight::VisionModule> local_module;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        local_module = vision_module;
    }
    if (running && local_module) {
    // Pass gyro data to VisionModule for sensor fusion
    // Check if VIO is running before accessing vision_module
    if (running && vision_module) {
        navsight::GyroData gyro;
        gyro.timestamp_ns = timestamp;
        gyro.x = x;
        gyro.y = y;
        gyro.z = z;
        local_module->addGyroData(gyro);
        vision_module->addGyroData(gyro);
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_setScale(JNIEnv *env, jobject /* this */, jdouble scale) {
    std::lock_guard<std::mutex> lock(vio_mutex);
    g_scale = scale;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_pingNative(JNIEnv *env, jobject thiz) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "--- PING NATIVE SUCCESSFUL ---");
}

} // extern "C"