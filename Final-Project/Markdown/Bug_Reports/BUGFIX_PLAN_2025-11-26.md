# NavSight1 Bug Fix Plan - 2025-11-26

## Overview
This document details all planned fixes for the 23 issues found in the NavSight1 VIO codebase.
**Created:** 2025-11-26
**Scope:** Critical, High, and Medium severity bug fixes

## Revert Instructions
If issues occur after applying these fixes:
```bash
git diff HEAD > bugfix_changes.patch
git checkout HEAD -- .
# Review bugfix_changes.patch to see what was changed
```

---

## CRITICAL FIXES (Priority 1)

### Critical #1: JNI Thread Attachment Memory Leak
**File:** `app/src/main/cpp/native-lib.cpp`
**Lines:** 213-288
**Problem:** Thread detachment skipped on exceptions

**Current Code:**
```cpp
if (g_jvm->GetEnv(reinterpret_cast<void**>(&jni_env), JNI_VERSION_1_6) != JNI_OK) {
    if (g_jvm->AttachCurrentThread(&jni_env, nullptr) == JNI_OK) {
        attached = true;
    }
}
// ... 70 lines of code ...
if (attached) {
    g_jvm->DetachCurrentThread();
}
```

**Fix:** Add RAII wrapper class at top of native-lib.cpp:
```cpp
// Add after includes, before any functions
struct JNIThreadGuard {
    JNIEnv* env;
    bool attached;
    JavaVM* jvm;

    JNIThreadGuard(JavaVM* vm, JNIEnv* current_env) : jvm(vm), env(current_env), attached(false) {
        if (jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
            if (jvm->AttachCurrentThread(&env, nullptr) == JNI_OK) {
                attached = true;
            }
        }
    }

    ~JNIThreadGuard() {
        if (attached && jvm) {
            jvm->DetachCurrentThread();
        }
    }

    JNIEnv* getEnv() { return env; }
};
```

**Usage in processCameraFrame():**
Replace lines 213-223 with:
```cpp
JNIThreadGuard jni_guard(g_jvm, env);
JNIEnv* jni_env = jni_guard.getEnv();
if (!jni_env) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to get JNI environment");
    return nullptr;
}
```
Remove detachment code at line 287.

---

### Critical #2: VisionModule Race Condition
**File:** `app/src/main/cpp/native-lib.cpp`
**Lines:** 252-260, 316-323, 379-386

**Problem:** Unprotected vision_module access while VIO thread could delete it

**Fix Strategy:** Change vision_module to shared_ptr and protect all access

**Changes:**
1. Change global declaration (line ~60):
```cpp
std::shared_ptr<navsight::VisionModule> vision_module;
```

2. Update startVIO() creation (line ~145):
```cpp
std::lock_guard<std::mutex> lock(vio_mutex);
vision_module = std::make_shared<navsight::VisionModule>();
```

3. Update stopVIO() deletion (line ~200):
```cpp
std::shared_ptr<navsight::VisionModule> module_copy;
{
    std::lock_guard<std::mutex> lock(vio_mutex);
    module_copy = vision_module;
    vision_module.reset();
}
// module_copy destructs here safely outside lock
```

4. Protect all access in processCameraFrame() (line ~252):
```cpp
std::shared_ptr<navsight::VisionModule> local_module;
{
    std::lock_guard<std::mutex> lock(vio_mutex);
    local_module = vision_module;
}
if (local_module) {
    auto stats = local_module->getStatistics();
    // ... use local_module instead of vision_module
}
```

5. Same pattern for processAccelerometer() and processGyroscope()

---

### Critical #3: Exception Handler Recursion
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Lines:** 84-89

**Current Code:**
```kotlin
Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
    Log.e(TAG, "UNCAUGHT EXCEPTION in thread ${thread.name}", throwable)
    Thread.getDefaultUncaughtExceptionHandler()?.uncaughtException(thread, throwable)
}
```

**Fix:** Capture default handler before setting:
```kotlin
val defaultExceptionHandler = Thread.getDefaultUncaughtExceptionHandler()
Thread.setDefaultUncaughtExceptionHandler { thread, throwable ->
    Log.e(TAG, "UNCAUGHT EXCEPTION in thread ${thread.name}", throwable)
    defaultExceptionHandler?.uncaughtException(thread, throwable)
}
```

---

### Critical #4: Double-Checked Locking Violation
**File:** `app/src/main/cpp/VisionModule.h`
**Line:** ~50 (class member declaration)

**Current:**
```cpp
bool is_initialized_;
```

**Fix:** Use atomic for thread-safe flag:
```cpp
std::atomic<bool> is_initialized_;
```

**File:** `app/src/main/cpp/VisionModule.cpp`
Constructor initialization:
```cpp
is_initialized_(false)  // Atomic initialization
```

All reads/writes remain the same (atomic handles memory barriers automatically).

---

### Critical #5: Missing Null Check for JNI Array
**File:** `app/src/main/cpp/native-lib.cpp`
**Lines:** 275-278

**Current:**
```cpp
jfloatArray pointsArray = jni_env->NewFloatArray(current_points.size());
if (pointsArray != nullptr && !current_points.empty()) {
    jni_env->SetFloatArrayRegion(pointsArray, 0, current_points.size(), current_points.data());
}
```

**Fix:** Return early if allocation fails:
```cpp
jfloatArray pointsArray = jni_env->NewFloatArray(current_points.size());
if (!pointsArray) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "Failed to allocate float array for tracked points");
    // Return default VioData with empty points
    jfloatArray emptyArray = jni_env->NewFloatArray(0);
    return jni_env->NewObject(vioDataClass, vioDataConstructor,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0, 0, 0.0, false,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        emptyArray);
}
if (!current_points.empty()) {
    jni_env->SetFloatArrayRegion(pointsArray, 0, current_points.size(), current_points.data());
}
```

---

## HIGH PRIORITY FIXES (Priority 2)

### High #1: Duplicate Import
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Lines:** 23, 32
**Fix:** Remove duplicate `import androidx.compose.ui.unit.dp` at line 32

---

### High #2: Unbounded Queue Growth
**File:** `app/src/main/cpp/native-lib.cpp`
**Lines:** 228-230

**Add constant at top:**
```cpp
const size_t MAX_FRAME_QUEUE_SIZE = 5;
```

**Fix frame queue push:**
```cpp
{
    std::lock_guard<std::mutex> lock(frame_queue_mutex);
    if (frame_queue.size() >= MAX_FRAME_QUEUE_SIZE) {
        frame_queue.pop();
        __android_log_print(ANDROID_LOG_WARN, TAG, "Frame queue full (%zu frames), dropping oldest",
            MAX_FRAME_QUEUE_SIZE);
    }
    frame_queue.push({timestamp, yuvMat.clone(), width, height});
    vio_cv.notify_one();
}
```

---

### High #3: Exception Handling Only Catches Exception
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Lines:** 823-836

**Current:**
```kotlin
try {
    val result = mainActivity.processCameraFrame(...)
} catch (e: Exception) {
    Log.e(MainActivity.TAG, "Error processing camera frame: ${e.message}")
}
```

**Fix:**
```kotlin
try {
    val result = mainActivity.processCameraFrame(...)
} catch (e: Throwable) {
    Log.e(MainActivity.TAG, "Error processing camera frame: ${e.message}", e)
}
```

---

### High #4: VioData equals/hashCode Missing Fields
**File:** `app/src/main/java/com/example/navsight1/VioData.kt`
**Lines:** 24-50

**Fix:** Remove custom equals/hashCode entirely to use data class defaults:
Delete lines 24-50 (custom equals, hashCode, and companion object).
The data class will auto-generate correct implementations.

---

### High #5: Hardcoded Windows Path in CMakeLists.txt
**File:** `app/CMakeLists.txt`
**Line:** 6

**Current:**
```cmake
set(OpenCV_DIR "C:/Users/morad/AndroidStudioProjects/NavSight1/OpenCV-android-sdk/sdk/native/jni")
```

**Fix:**
```cmake
# Use relative path from project root
set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../OpenCV-android-sdk/sdk/native/jni")
```

---

### High #6: Timestamp Type Mismatch
**File:** `app/src/main/cpp/IMUPreintegrator.cpp`
**Lines:** 66, 81

**Current:**
```cpp
long last_timestamp = start_ns;
```

**Fix:**
```cpp
int64_t last_timestamp = start_ns;
```
Also update line 81 similarly.

---

### High #7: Memory Leak in Frame Queue
**File:** `app/src/main/cpp/native-lib.cpp`
**Function:** stopVIO()

**Add queue cleanup after thread join:**
```cpp
// Clear frame queue to prevent memory leaks
{
    std::lock_guard<std::mutex> lock(frame_queue_mutex);
    while (!frame_queue.empty()) {
        frame_queue.pop();
    }
    __android_log_print(ANDROID_LOG_INFO, TAG, "Frame queue cleared");
}
```

---

## MEDIUM PRIORITY FIXES (Priority 3)

### Medium #1: OpenCV Debug Mode in Production
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Line:** 395

**Current:**
```kotlin
isOpenCVInitialized = OpenCVLoader.initDebug()
```

**Fix:**
```kotlin
isOpenCVInitialized = if (BuildConfig.DEBUG) {
    Log.d(TAG, "Initializing OpenCV in DEBUG mode")
    OpenCVLoader.initDebug()
} else {
    Log.d(TAG, "Initializing OpenCV in RELEASE mode")
    OpenCVLoader.initLocal()
}
```

---

### Medium #2: String Formatting in Hot Path
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Lines:** 589, 603, 621, 628, 635, 659, 667

**Current (example):**
```kotlin
text = String.format("X: %.2f  Y: %.2f  Z: %.2f", vioData.x, vioData.y, vioData.z)
```

**Fix:** Use Kotlin string templates:
```kotlin
text = "X: ${"%.2f".format(vioData.x)}  Y: ${"%.2f".format(vioData.y)}  Z: ${"%.2f".format(vioData.z)}"
```

Apply to all 7 instances in DebugOverlay().

---

### Medium #3: Missing Boundary Check in Polyline Decoder
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Lines:** 273, 283

**Current:**
```kotlin
do {
    b = encoded[index++].code - 63
    result = result or (b and 0x1f shl shift)
    shift += 5
} while (b >= 0x20)
```

**Fix:**
```kotlin
do {
    if (index >= len) {
        Log.e(TAG, "Malformed polyline encoding at index $index")
        break
    }
    b = encoded[index++].code - 63
    result = result or (b and 0x1f shl shift)
    shift += 5
} while (b >= 0x20 && index <= len)
```

---

### Medium #4: Mutex Documentation
**File:** `app/src/main/cpp/native-lib.cpp`
**Line:** ~80 (global variables section)

**Add documentation:**
```cpp
// Protects: latest_global_R, latest_global_t, latest_points, vision_module pointer
// Acquire order: Always acquire vio_mutex before accessing vision_module or global pose
std::mutex vio_mutex;
```

---

### Medium #5: IMU Buffer Size Consistency
**File:** `app/src/main/cpp/VisionModule.h`
**Lines:** Define constants

**Current:**
```cpp
const size_t MAX_GYRO_BUFFER_SIZE = 100;
const size_t MAX_ACCEL_BUFFER_SIZE = 100;
```

**Fix:** Align with IMUPreintegrator:
```cpp
const size_t MAX_GYRO_BUFFER_SIZE = 200;
const size_t MAX_ACCEL_BUFFER_SIZE = 200;
```

---

### Medium #6: Redundant Assignment
**File:** `app/src/main/cpp/native-lib.cpp`
**Line:** 222

**Remove:**
```cpp
} else {
    jni_env = env;  // DELETE THIS LINE
}
```

---

### Medium #7: Magic Numbers
**File:** `app/src/main/java/com/example/navsight1/MainActivity.kt`
**Lines:** 1129, 1130

**Add constant:**
```kotlin
companion object {
    private const val METERS_PER_DEGREE = 111139.0  // Standard Earth radius conversion
    // ... existing constants
}
```

**Fix usage:**
```kotlin
val lat = start.latitude + (dz / METERS_PER_DEGREE)
val lng = start.longitude + (dx / (METERS_PER_DEGREE * Math.cos(start.latitude * Math.PI / 180.0)))
```

---

## Implementation Order

1. **Phase 1 - Critical (Do First):**
   - Critical #3 (Exception handler) - Kotlin only, safest
   - Critical #5 (JNI null check) - Small change
   - Critical #1 (JNI RAII wrapper) - Requires careful testing
   - Critical #4 (Atomic bool) - Header + cpp change
   - Critical #2 (shared_ptr) - Most complex, do last in critical phase

2. **Phase 2 - High Priority:**
   - High #1 (Duplicate import) - Trivial
   - High #5 (CMakeLists path) - Important for portability
   - High #3 (Exception catch Throwable) - Simple change
   - High #4 (VioData equals/hashCode) - Delete code
   - High #2 (Queue size limit) - Add bounds checking
   - High #7 (Queue cleanup) - Add cleanup code
   - High #6 (Timestamp type) - Type change

3. **Phase 3 - Medium Priority:**
   - Medium #1 (OpenCV debug/release)
   - Medium #7 (Magic numbers constant)
   - Medium #3 (Polyline boundary check)
   - Medium #4 (Mutex documentation)
   - Medium #6 (Remove redundant assignment)
   - Medium #2 (String formatting) - Many changes
   - Medium #5 (Buffer sizes)

## Testing After Each Phase

**After Phase 1:**
```bash
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n com.example.navsight1/.MainActivity
adb logcat | grep -E "(NavSight|VIO|FATAL)"
```

**After Phase 2:**
Run for 10 minutes, monitor memory usage:
```bash
adb shell dumpsys meminfo com.example.navsight1
```

**After Phase 3:**
Full regression test, check performance metrics.

---

## Rollback Plan

If critical issues arise:
1. Check git status: `git status`
2. View changes: `git diff`
3. Revert specific file: `git checkout HEAD -- <file>`
4. Revert all: `git reset --hard HEAD`

## Files Modified

- `app/src/main/cpp/native-lib.cpp` (15 changes)
- `app/src/main/cpp/VisionModule.h` (2 changes)
- `app/src/main/cpp/VisionModule.cpp` (1 change)
- `app/src/main/cpp/IMUPreintegrator.cpp` (2 changes)
- `app/src/main/java/com/example/navsight1/MainActivity.kt` (12 changes)
- `app/src/main/java/com/example/navsight1/VioData.kt` (1 change)
- `app/CMakeLists.txt` (1 change)

**Total:** 7 files, 34 changes

---

**Status:** Ready to implement
**Estimated Time:** 2-3 hours
**Risk Level:** Medium (most changes are localized, but Critical #2 touches multiple areas)
