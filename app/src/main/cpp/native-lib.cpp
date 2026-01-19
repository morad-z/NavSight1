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
#include <atomic>
#include <cmath>
#include "VisionModule.h"

#define TAG "NavSight-Native"

JavaVM* g_jvm = nullptr;

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    g_jvm = vm;
    return JNI_VERSION_1_6;
}

// --- Data Structures ---

struct CamFrame {
    jlong timestamp;
    cv::Mat yuvMat;
    int width;
    int height;
};

// --- Global State & Synchronization ---

// VIO Thread controls
std::thread vio_thread;
std::mutex vio_mutex; // Protects vision_module and global pose data
std::condition_variable vio_cv;
std::atomic<bool> running{false};

// Vision Module (Managed via shared_ptr for safety)
std::shared_ptr<navsight::VisionModule> vision_module;

// Frame Queue
const size_t MAX_FRAME_QUEUE_SIZE = 2;
std::queue<CamFrame> frame_queue;
std::mutex frame_queue_mutex;

// IMU Buffers
std::mutex accel_queue_mutex;
const int ACCEL_BUFFER_SIZE = 100;
std::vector<cv::Point3f> accel_buffer_for_reset;

// Latest Global State (For UI rendering)
cv::Mat latest_global_R = cv::Mat::eye(3, 3, CV_64F);
cv::Mat latest_global_t = cv::Mat::zeros(3, 1, CV_64F);
std::vector<float> latest_points;
double g_scale = 1.0;

// Internal Tracking State (Used only inside VIO thread)
cv::Mat global_R = cv::Mat::eye(3, 3, CV_64F);
cv::Mat global_t = cv::Mat::zeros(3, 1, CV_64F);

// Latest IMU readings (For UI display only)
std::mutex latest_imu_mutex;
float latest_accel_x = 0.0f, latest_accel_y = 0.0f, latest_accel_z = 0.0f;
float latest_gyro_x = 0.0f, latest_gyro_y = 0.0f, latest_gyro_z = 0.0f;

// --- Helper Functions ---

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
    return cv::Vec3d(x, y, z); // Returns in Radians
}

// --- VIO Worker Thread ---

void vio_thread_loop() {
    // Initialize VisionModule
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        try {
            vision_module = std::make_shared<navsight::VisionModule>(
                    navsight::FeatureDetectorType::GOOD_FEATURES, 200);
            __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule created successfully");
        } catch (const std::exception& e) {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to create VisionModule: %s", e.what());
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

        // Create a local shared_ptr to ensure module stays alive during processing
        std::shared_ptr<navsight::VisionModule> local_module;
        {
            std::lock_guard<std::mutex> lock(vio_mutex);
            local_module = vision_module;
        }

        if (!local_module) {
            __android_log_print(ANDROID_LOG_ERROR, TAG, "VisionModule is null!");
            continue;
        }

        // Process frame
        navsight::VisionOutput vision_output;
        try {
            vision_output = local_module->processFrame(
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

        // Update global pose
        if (vision_output.is_valid) {
            std::lock_guard<std::mutex> lock(vio_mutex);

            // Get scale
            double scale_to_use = local_module->getEstimatedScale();
            // Fallback to manual scale if auto-scale is not ready or too small
            if (scale_to_use < 0.01) {
                scale_to_use = g_scale;
            }

            // Update global position: t_global += scale * (R_global * t_local)
            global_t = global_t + (scale_to_use * (global_R * vision_output.translation));

            // Update global rotation: R_global = R_local * R_global
            // Note: Order depends on coordinate system. Typically R_global_new = R_local * R_global_old
            global_R = vision_output.rotation * global_R;

            // Update shared state for UI
            latest_global_t = global_t.clone();
            latest_global_R = global_R.clone();
            latest_points = points_for_ui;
        } else {
            // Just update points for UI even if pose failed
            std::lock_guard<std::mutex> lock(vio_mutex);
            latest_points = points_for_ui;
        }
    }

    // Thread exit cleanup
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        if (vision_module) {
            auto stats = vision_module->getStatistics();
            __android_log_print(ANDROID_LOG_INFO, TAG,
                                "Stats - Frames: %d, Success: %d", stats.total_frames_processed, stats.successful_tracks);
        }
    }
}


// --- JNI Exported Functions ---

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

    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        if (vision_module) {
            vision_module.reset(); // Release the shared_ptr
            __android_log_print(ANDROID_LOG_INFO, TAG, "VisionModule destroyed");
        }
    }

    // Clear frame queue
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex);
        std::queue<CamFrame> empty;
        std::swap(frame_queue, empty);
    }

    __android_log_print(ANDROID_LOG_INFO, TAG, "VIO thread stopped.");
}

JNIEXPORT jobject JNICALL
Java_com_example_navsight1_MainActivity_processCameraFrame(
        JNIEnv* env, jobject /* this */, jbyteArray frameData, jint width, jint height, jlong timestamp) {

    // 1. Push Frame to Queue
    jbyte* buffer = env->GetByteArrayElements(frameData, nullptr);
    if (buffer != nullptr) {
        // Need to clone the data because JNI buffer will be released
        cv::Mat yuvMat(height + height / 2, width, CV_8UC1, (unsigned char*)buffer);

        {
            std::lock_guard<std::mutex> lock(frame_queue_mutex);
            if (frame_queue.size() >= MAX_FRAME_QUEUE_SIZE) {
                frame_queue.pop(); // Drop oldest
            }
            // Clone is essential here as 'buffer' is temporary
            frame_queue.push({timestamp, yuvMat.clone(), width, height});
            vio_cv.notify_one();
        }

        env->ReleaseByteArrayElements(frameData, buffer, JNI_ABORT);
    }

    // 2. Prepare Data for Java Return
    // We grab the LATEST state processed by the thread, not the frame we just pushed.

    jclass vioDataClass = env->FindClass("com/example/navsight1/VioData");
    if (!vioDataClass) return nullptr;

    // Signature must match your VioData.java constructor exactly!
    // double x, y, z, roll, pitch, yaw, float[] points, double quality, int tracked, int total, double scale, boolean init, double ax, ay, az, gx, gy, gz
    jmethodID vioDataConstructor = env->GetMethodID(vioDataClass, "<init>", "(DDDDDD[FDIIDZFFFFFF)V");
    if (!vioDataConstructor) return nullptr;

    cv::Mat current_R, current_t;
    std::vector<float> current_points;
    double tracking_quality = 0.0;
    int tracked_features = 0;
    int total_features = 200;
    double estimated_scale = 1.0;
    bool is_initialized = false;

    // Use local shared_ptr for safety
    std::shared_ptr<navsight::VisionModule> local_module;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        current_R = latest_global_R.clone();
        current_t = latest_global_t.clone();
        current_points = latest_points;
        local_module = vision_module;
    }

    if (local_module) {
        auto stats = local_module->getStatistics();
        tracking_quality = stats.average_tracking_quality;
        tracked_features = (int)stats.average_features_tracked;
        estimated_scale = local_module->getEstimatedScale();
        is_initialized = local_module->isInitialized();
    }

    cv::Vec3d euler = rotationMatrixToEulerAngles(current_R);

    // IMU Data for UI
    float ax, ay, az, gx, gy, gz;
    {
        std::lock_guard<std::mutex> lock(latest_imu_mutex);
        ax = latest_accel_x; ay = latest_accel_y; az = latest_accel_z;
        gx = latest_gyro_x; gy = latest_gyro_y; gz = latest_gyro_z;
    }

    // Create Float Array for Points
    jfloatArray pointsArray = env->NewFloatArray(current_points.size());
    if (pointsArray != nullptr && !current_points.empty()) {
        env->SetFloatArrayRegion(pointsArray, 0, current_points.size(), current_points.data());
    }

    // Create Java Object
    return env->NewObject(vioDataClass, vioDataConstructor,
                          current_t.at<double>(0), current_t.at<double>(1), current_t.at<double>(2), // Position
                          euler[0], euler[1], euler[2], // Rotation (Radians)
                          pointsArray,
                          tracking_quality, tracked_features, total_features,
                          estimated_scale, is_initialized,
                          ax, ay, az, gx, gy, gz);
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processAccelerometer(
        JNIEnv* env, jobject /* this */, jlong timestamp, jfloat x, jfloat y, jfloat z) {

    // 1. Update UI display variables
    {
        std::lock_guard<std::mutex> lock(latest_imu_mutex);
        latest_accel_x = x; latest_accel_y = y; latest_accel_z = z;
    }

    // 2. Buffer for Reset Calculation
    {
        std::lock_guard<std::mutex> lock(accel_queue_mutex);
        accel_buffer_for_reset.push_back(cv::Point3f(x, y, z));
        if (accel_buffer_for_reset.size() > ACCEL_BUFFER_SIZE) {
            accel_buffer_for_reset.erase(accel_buffer_for_reset.begin());
        }
    }

    // 3. Pass to VisionModule safely
    std::shared_ptr<navsight::VisionModule> local_module;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        local_module = vision_module;
    }

    if (running && local_module) {
        navsight::AccelData accel;
        accel.timestamp_ns = timestamp;
        accel.x = x; accel.y = y; accel.z = z;
        local_module->addAccelData(accel);
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processGyroscope(
        JNIEnv* env, jobject /* this */, jlong timestamp, jfloat x, jfloat y, jfloat z) {

    // 1. Update UI display
    {
        std::lock_guard<std::mutex> lock(latest_imu_mutex);
        latest_gyro_x = x; latest_gyro_y = y; latest_gyro_z = z;
    }

    // 2. Pass to VisionModule
    std::shared_ptr<navsight::VisionModule> local_module;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        local_module = vision_module;
    }

    if (running && local_module) {
        navsight::GyroData gyro;
        gyro.timestamp_ns = timestamp;
        gyro.x = x; gyro.y = y; gyro.z = z;
        local_module->addGyroData(gyro);
    }
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_resetVIO(JNIEnv *env, jobject /* this */) {
    std::lock_guard<std::mutex> lock(vio_mutex);

    // Logic: Reset based on Gravity from buffered Accel data
    cv::Point3f avg_accel(0, 0, 0);
    {
        std::lock_guard<std::mutex> accel_lock(accel_queue_mutex);
        if (accel_buffer_for_reset.empty()) {
            __android_log_print(ANDROID_LOG_WARN, TAG, "Reset called but accel buffer empty");
            // Perform basic reset without gravity alignment
        } else {
            for (const auto& reading : accel_buffer_for_reset) {
                avg_accel += reading;
            }
            avg_accel.x /= accel_buffer_for_reset.size();
            avg_accel.y /= accel_buffer_for_reset.size();
            avg_accel.z /= accel_buffer_for_reset.size();
        }
    }

    // Reset Globals
    global_R = cv::Mat::eye(3, 3, CV_64F);
    global_t = cv::Mat::zeros(3, 1, CV_64F);
    latest_global_R = global_R.clone();
    latest_global_t = global_t.clone();
    latest_points.clear();

    if (vision_module) {
        vision_module->reset();
        // Force initialization from calculated gravity if possible?
        // Usually VisionModule handles this internally via addAccelData,
        // but we just reset it, so it needs fresh data.
    }

    __android_log_print(ANDROID_LOG_INFO, TAG, "VIO Reset Complete");
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_setScale(JNIEnv *env, jobject /* this */, jdouble scale) {
    std::lock_guard<std::mutex> lock(vio_mutex);
    g_scale = scale;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_pingNative(JNIEnv *env, jobject thiz) {
    __android_log_print(ANDROID_LOG_INFO, TAG, "--- PING NATIVE SUCCESSFUL ---");
}

} // extern "C"