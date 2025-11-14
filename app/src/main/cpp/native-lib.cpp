#include <jni.h>
#include <string>
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

JavaVM* g_jvm = nullptr;

JNIEXPORT jint JNI_OnLoad(JavaVM* vm, void* reserved) {
    g_jvm = vm;
    return JNI_VERSION_1_6;
}

#define TAG "NavSight-Native"

struct GyroReading {
    long timestamp;
    float x, y, z;
};

// --- Threading and Data Synchronization ---
std::thread vio_thread;
std::mutex vio_mutex;
std::condition_variable vio_cv;
bool running = false;

struct CamFrame {
    long timestamp;
    cv::Mat yuvMat;
    int width;
    int height;
};

// Thread-safe queues for sensor data
std::queue<CamFrame> frame_queue;
std::mutex frame_queue_mutex;

std::queue<GyroReading> gyro_queue;
std::mutex gyro_queue_mutex;

std::queue<cv::Point3f> accel_queue;
std::mutex accel_queue_mutex;

// Latest calculated pose (protected by vio_mutex)
cv::Mat latest_global_R = cv::Mat::eye(3, 3, CV_64F);
cv::Mat latest_global_t = cv::Mat::zeros(3, 1, CV_64F);
std::vector<float> latest_points;


// --- VIO State Variables (used only by VIO thread) ---
static cv::Mat prevGray;
static std::vector<cv::Point2f> prevCorners;
static cv::Mat global_R = cv::Mat::eye(3, 3, CV_64F);
static cv::Mat global_t = cv::Mat::zeros(3, 1, CV_64F);
static long prev_timestamp = 0;
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
    while (running) {
        CamFrame frame;
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex);
            vio_cv.wait(lock, [] { return !frame_queue.empty() || !running; });
            if (!running) break;
            frame = frame_queue.front();
            frame_queue.pop();
        }

        __android_log_print(ANDROID_LOG_INFO, TAG, "VIO Thread: Processing frame %ld", frame.timestamp);

        // --- Main VIO Logic ---
        cv::Mat grayMat;
        cv::cvtColor(frame.yuvMat, grayMat, cv::COLOR_YUV2GRAY_NV21);

        if (prevGray.empty()) {
            cv::goodFeaturesToTrack(grayMat, prevCorners, 200, 0.01, 10);
            grayMat.copyTo(prevGray);
            prev_timestamp = frame.timestamp;
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "VIO Thread: First frame, detected %zu features.", prevCorners.size());
            continue;
        }

        // Gyro Integration
        cv::Mat delta_R_from_gyro = cv::Mat::eye(3, 3, CV_64F);
        long last_gyro_ts = prev_timestamp;
        std::queue<GyroReading> temp_gyro_queue;
        {
            std::lock_guard<std::mutex> lock(gyro_queue_mutex);
            temp_gyro_queue = gyro_queue;
            while(!gyro_queue.empty()) gyro_queue.pop();
        }
        while(!temp_gyro_queue.empty()) {
            GyroReading reading = temp_gyro_queue.front();
            temp_gyro_queue.pop();
            if (reading.timestamp > last_gyro_ts) {
                double dt = (reading.timestamp - last_gyro_ts) / 1e9;
                cv::Mat rot_vec = (cv::Mat_<double>(3, 1) << reading.x * dt, reading.y * dt, reading.z * dt);
                cv::Mat delta_R;
                cv::Rodrigues(rot_vec, delta_R);
                delta_R_from_gyro = delta_R * delta_R_from_gyro;
                last_gyro_ts = reading.timestamp;
            }
        }

        // Optical Flow
        std::vector<cv::Point2f> nextCorners;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prevGray, grayMat, prevCorners, nextCorners, status, err);

        std::vector<cv::Point2f> good_prev_corners, good_next_corners;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                good_prev_corners.push_back(prevCorners[i]);
                good_next_corners.push_back(nextCorners[i]);
            }
        }
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "VIO Thread: Features tracked: %zu / %zu", good_prev_corners.size(), prevCorners.size());

        // Prepare points for UI
        std::vector<float> points_for_ui;
        points_for_ui.reserve(good_next_corners.size() * 2);
        for(const auto& p : good_next_corners) {
            points_for_ui.push_back(p.x);
            points_for_ui.push_back(p.y);
        }

        if (good_prev_corners.size() > 5) {
            cv::Mat E, R, t;
            double focal = frame.width;
            cv::Point2d principal_point(frame.width / 2.0, frame.height / 2.0);
            cv::Mat K = (cv::Mat_<double>(3, 3) << focal, 0, principal_point.x, 0, focal, principal_point.y, 0, 0, 1);
            E = cv::findEssentialMat(good_next_corners, good_prev_corners, K, cv::RANSAC, 0.999, 1.0, cv::noArray());

            if (!E.empty()) {
                __android_log_print(ANDROID_LOG_INFO, TAG, "Essential Matrix found!");
                cv::recoverPose(E, good_next_corners, good_prev_corners, K, R, t, cv::noArray());
                const double alpha = 0.98;
                cv::Mat rot_vec_vo, rot_vec_gyro;
                cv::Rodrigues(R, rot_vec_vo);
                cv::Rodrigues(delta_R_from_gyro, rot_vec_gyro);
                cv::Mat rot_vec_fused = alpha * rot_vec_gyro + (1.0 - alpha) * rot_vec_vo;
                cv::Mat R_fused;
                cv::Rodrigues(rot_vec_fused, R_fused);

                std::lock_guard<std::mutex> lock(vio_mutex);
                global_t = global_t + (g_scale * (global_R * t));
                global_R = R_fused * global_R;
                latest_global_t = global_t.clone();
                latest_global_R = global_R.clone();
                latest_points = points_for_ui;
            } else {
                __android_log_print(ANDROID_LOG_WARN, TAG, "Essential Matrix not found.");
                std::lock_guard<std::mutex> lock(vio_mutex);
                latest_points = points_for_ui;
            }
        } else {
            std::lock_guard<std::mutex> lock(vio_mutex);
            latest_points = points_for_ui;
        }
        grayMat.copyTo(prevGray);
        prevCorners = good_next_corners;
        if (prevCorners.size() < 100) {
            std::vector<cv::Point2f> new_corners;
            cv::goodFeaturesToTrack(grayMat, new_corners, 200 - prevCorners.size(), 0.01, 10);
            prevCorners.insert(prevCorners.end(), new_corners.begin(), new_corners.end());
        }
        prev_timestamp = frame.timestamp;
    }
}

extern "C" {

JNIEXPORT jstring JNICALL
Java_com_example_navsight1_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "NATIVE CODE VERSION 2";
    return env->NewStringUTF(hello.c_str());
}

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
        std::lock_guard<std::mutex> lock(frame_queue_mutex);
        frame_queue.push({timestamp, yuvMat.clone(), width, height});
        vio_cv.notify_one();
        jni_env->ReleaseByteArrayElements(frameData, buffer, JNI_ABORT);
    }

    jclass vioDataClass = jni_env->FindClass("com/example/navsight1/VioData");
    jmethodID vioDataConstructor = jni_env->GetMethodID(vioDataClass, "<init>", "(DDDDDD[F)V");
    
    cv::Mat current_R, current_t;
    std::vector<float> current_points;
    {
        std::lock_guard<std::mutex> lock(vio_mutex);
        current_R = latest_global_R.clone();
        current_t = latest_global_t.clone();
        current_points = latest_points;
    }
    cv::Vec3d eulerAngles = rotationMatrixToEulerAngles(current_R);

    jfloatArray pointsArray = jni_env->NewFloatArray(current_points.size());
    if (pointsArray != nullptr && !current_points.empty()) {
        jni_env->SetFloatArrayRegion(pointsArray, 0, current_points.size(), current_points.data());
    }

    jobject result = jni_env->NewObject(vioDataClass, vioDataConstructor,
                          current_t.at<double>(0), current_t.at<double>(1), current_t.at<double>(2),
                          eulerAngles[0], eulerAngles[1], eulerAngles[2], pointsArray);

    if (attached) {
        g_jvm->DetachCurrentThread();
    }

    return result;
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processAccelerometer(
        JNIEnv* env, jobject /* this */, jlong timestamp, jfloat x, jfloat y, jfloat z) {
    
    std::lock_guard<std::mutex> lock(accel_queue_mutex);
    accel_buffer_for_reset.push_back(cv::Point3f(x, y, z));
    if (accel_buffer_for_reset.size() > ACCEL_BUFFER_SIZE) {
        accel_buffer_for_reset.erase(accel_buffer_for_reset.begin());
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

    global_R = cv::Mat::eye(3, 3, CV_64F);
    global_t = cv::Mat::zeros(3, 1, CV_64F);
    latest_global_R = global_R.clone();
    latest_global_t = global_t.clone();
    latest_points.clear();
    prevGray.release();
    prevCorners.clear();
}

JNIEXPORT void JNICALL
Java_com_example_navsight1_MainActivity_processGyroscope(
        JNIEnv* env, jobject /* this */, jlong timestamp, jfloat x, jfloat y, jfloat z) {
    std::lock_guard<std::mutex> lock(gyro_queue_mutex);
    gyro_queue.push({timestamp, x, y, z});
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