#include <jni.h>
#include <string>
#include <opencv2/core.hpp>

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_navsight1_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string cv_version = cv::getVersionString();
    std::string hello = "Hello from C++. OpenCV version: " + cv_version;
    return env->NewStringUTF(hello.c_str());
}
