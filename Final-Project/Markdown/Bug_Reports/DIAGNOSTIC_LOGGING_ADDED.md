# Diagnostic Logging Added for IMU Preintegration

## Summary

I've added comprehensive diagnostic logging to help identify why the IMU preintegration isn't activating (scale stuck at 1.000).

## What Was Changed

### File: `VisionModule.cpp`

**Added logging at 6 key decision points:**

1. **Before preintegration check** (line 246-248)
   - Logs: initialization status, time delta, buffer size
   - Identifies if system is initialized and has IMU data

2. **After integrate() call** (line 255-257)
   - Logs: number of measurements integrated, time interval
   - Shows if IMU measurements are being found in time window

3. **After displacement calculation** (line 267-269)
   - Logs: vision displacement, IMU displacement
   - Shows if motion is large enough to estimate scale

4. **After scale computation** (line 274-276)
   - Logs: computed scale value, valid range
   - Shows if scale passes sanity checks

5. **When scale update succeeds** (line 284-286)
   - Logs: IMU displacement, scale, smoothed scale
   - Confirms preintegration is working

6. **When preintegration fails** (line 288-289, 292-294, 297-298)
   - Logs specific failure reasons
   - Helps identify exact issue

**Added periodic logging for IMU data collection:**

7. **In addGyroData()** (line 391-397)
   - Logs every 30th gyro measurement
   - Shows buffer size growing

8. **In addAccelData()** (line 438-444)
   - Logs every 30th accel measurement
   - Shows buffer size growing

## How to Use the Diagnostic Logs

### Step 1: Build the App

**Option A: Android Studio**
1. Open project in Android Studio
2. Click "Build" → "Make Project"
3. If it fails with "Java 8" error, set JAVA_HOME to Java 11+
4. Install to device: Run → Run 'app'

**Option B: Command Line** (if you have Java 11+)
```bash
cd C:\Users\morad\AndroidStudioProjects\NavSight1
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

### Step 2: Run Diagnostic Logcat

Open a terminal and run:

```bash
adb logcat -s NavSight-Native:D IMUPreintegrator:D
```

Or for cleaner output:

```bash
adb logcat -s NavSight-Native:D IMUPreintegrator:D | grep -E "Preintegration|Added gyro|Added accel|Displacement|Scale computed"
```

### Step 3: Launch the App

Start NavSight1 on your phone and watch the logcat output.

### Step 4: Interpret the Logs

**See the comprehensive guide in `TEST_IMU_PREINTEGRATION.md`** - it has a complete troubleshooting decision tree.

## Expected Output (If Working Correctly)

```
NavSight-Native D  Added gyro measurement (count=30, buffer_size=25)
NavSight-Native D  Added accel measurement (count=30, buffer_size=50)
NavSight-Native D  Preintegration check: initialized=1, dt_ns=33333333, buffer_size=50
NavSight-Native D  Preintegration result: 8 measurements, dt=0.033s
IMUPreintegrator D  Preintegrated 8 measurements over 0.033 seconds: p=(0.012, -0.003, 0.005), v=(...)
NavSight-Native D  Displacement check: vision=0.0523m, imu=0.0147m
NavSight-Native D  Scale computed: 0.281 (range check: 0.1 to 10.0)
NavSight-Native D  IMU Preintegration: displacement=0.015m, scale=0.281 (smoothed=0.450)
```

## Common Issues and Fixes

### Issue 1: "initialized=0" keeps appearing
**Cause**: System waiting for gravity estimation
**Fix**: Wait 1-2 seconds after app launch. If it persists, check accel sensor.

### Issue 2: "Preintegration result: 0 measurements"
**Cause**: IMU buffer empty or timestamp mismatch
**Fix**:
1. Check if "Added gyro/accel" logs appear
2. If not, check sensor registration in MainActivity
3. If yes, check timestamp alignment (IMU vs camera)

### Issue 3: "Displacement too small"
**Cause**: Phone not moving enough
**Fix**: Move phone at least 5-10cm between frames

### Issue 4: "Scale X.XX outside valid range [0.1, 10.0]"
**Cause**:
- Vision displacement too small (lost tracking)
- IMU displacement too large (acceleration spike)
**Fix**: Improve lighting, move slower

### Issue 5: No "Added gyro/accel" logs at all
**Cause**: IMU sensors not sending data
**Fix**:
1. Check sensor permissions in AndroidManifest.xml
2. Verify sensor registration in MainActivity.kt
3. Check JNI calls in native-lib.cpp

## Next Steps After Diagnosis

Once you identify the issue from the logs:

1. **If preintegration IS working** → Test accuracy improvements, tune parameters
2. **If buffer is empty** → Fix IMU data collection (sensor registration)
3. **If timestamps misaligned** → Fix timestamp synchronization
4. **If displacements too small** → Improve feature tracking or increase thresholds
5. **If scale unrealistic** → Add more robust outlier rejection

## Files Modified

- `app/src/main/cpp/VisionModule.cpp` - Added 8 new diagnostic log points
- `TEST_IMU_PREINTEGRATION.md` - Added diagnostic mode section

## No Functionality Changes

This update adds **logging only** - no algorithm changes. The app should behave identically, just with more verbose output for debugging.

## Revert Instructions

If you want to remove the diagnostic logs later (after fixing the issue):

```bash
git diff HEAD app/src/main/cpp/VisionModule.cpp
```

Then manually remove the `__android_log_print` calls added in this update.

---

**Ready to test!** Follow the steps above and share the logcat output to diagnose the preintegration issue.
