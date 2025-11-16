# Quick Rollback Instructions

If the app isn't working, follow these steps to go back to the working version:

## Option 1: Use Git (if you have version control)
```bash
git status
git diff
git checkout HEAD -- app/src/main/cpp/
```

## Option 2: Manual Rollback

The changes that might be causing issues:

### 1. CMakeLists.txt
Remove this line:
```cmake
src/main/cpp/VisionModule.cpp
```

Should look like:
```cmake
add_library(
        navsight
        SHARED
        src/main/cpp/native-lib.cpp)
```

### 2. Delete New Files (temporary)
- `app/src/main/cpp/VisionModule.h`
- `app/src/main/cpp/VisionModule.cpp`

### 3. Restore native-lib.cpp

The main changes were in `vio_thread_loop()` and sensor callbacks.

## Minimal Working Version

If you want to keep using VisionModule but disable problematic features, I can create a simplified version.

## What to Share for Debugging

Please share:
1. Build output (Success or Failed?)
2. If failed: error messages
3. If successful: Logcat crash output
   ```bash
   adb logcat -d *:E | tail -50
   ```
