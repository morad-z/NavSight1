# Testing IMU Preintegration

## 🔍 DIAGNOSTIC MODE (START HERE!)

**NEW**: Enhanced diagnostic logging has been added to help identify exactly why preintegration might not be working.

### Quick Diagnostic Test

1. **Build and install the app** (or have it built in Android Studio)

2. **Run this command** to see diagnostic output:
```bash
adb logcat -s NavSight-Native:D IMUPreintegrator:D | grep -E "Preintegration|Added gyro|Added accel|Displacement|Scale computed"
```

3. **Launch the app** and watch the output

### What the Diagnostic Logs Tell You

**Log 1: IMU Data Collection**
```
NavSight-Native D  Added gyro measurement (count=30, buffer_size=25)
NavSight-Native D  Added accel measurement (count=30, buffer_size=50)
```
- Shows IMU data is flowing into the preintegrator
- `buffer_size` shows how many measurements are stored
- If you DON'T see these → IMU data not being collected

**Log 2: Preintegration Status**
```
NavSight-Native D  Preintegration check: initialized=1, dt_ns=33333333, buffer_size=50
```
- `initialized=1` → System initialized from gravity ✅
- `initialized=0` → Still initializing, need more accel samples ❌
- `dt_ns` → Time between frames (should be ~33ms for 30 FPS)
- `buffer_size` → Number of IMU measurements available

**Log 3: Integration Result**
```
NavSight-Native D  Preintegration result: 8 measurements, dt=0.033s
IMUPreintegrator D  Preintegrated 8 measurements over 0.033 seconds: p=(0.012, -0.003, 0.005), v=(...)
```
- Shows how many IMU measurements were integrated
- If `0 measurements` → IMU buffer empty or timestamps misaligned
- `p=` shows position change from IMU

**Log 4: Displacement Check**
```
NavSight-Native D  Displacement check: vision=0.0523m, imu=0.0147m
```
- `vision` = camera-based movement (unitless, converted to meters by scale)
- `imu` = IMU-based movement in meters
- If both are too small → phone not moving enough
- Thresholds: vision > 0.01m, imu > 0.001m

**Log 5: Scale Computation**
```
NavSight-Native D  Scale computed: 0.281 (range check: 0.1 to 10.0)
NavSight-Native D  IMU Preintegration: displacement=0.015m, scale=0.281 (smoothed=0.450)
```
- Shows computed scale and whether it passed validation
- If you see "Scale X.XX outside valid range" → scale unrealistic
- `smoothed` = exponentially smoothed scale value

**Log 6: Common Failure Messages**
```
NavSight-Native W  Displacement too small: vision=0.0045, imu=0.0002 (thresholds: 0.01, 0.001)
```
→ Phone not moving enough, or feature tracking lost

```
NavSight-Native W  Scale 23.456 outside valid range [0.1, 10.0]
```
→ IMU displacement too large or vision displacement too small

```
NavSight-Native D  No IMU measurements in time window, using fallback
```
→ IMU buffer empty or timestamps don't overlap with frame times

### Troubleshooting Decision Tree

**If you see "Added gyro/accel measurement" logs:**
✅ IMU data flowing → Check next step

**If you DON'T see IMU measurement logs:**
❌ IMU sensors not working → Check:
- Sensor permissions in AndroidManifest.xml
- JNI calls in native-lib.cpp
- Sensor registration in MainActivity

---

**If you see "initialized=0":**
❌ System not initialized → Wait for gravity estimation
- Needs 20+ accel samples (~0.2 seconds)
- Look for "Initialized from gravity" log

**If you see "initialized=1":**
✅ System initialized → Check next step

---

**If you see "Preintegration result: 0 measurements":**
❌ No IMU in time window → Likely causes:
1. IMU buffer cleared too aggressively
2. Timestamp mismatch between IMU and camera
3. Camera frames arriving faster than IMU samples

**Fix**: Check timestamp alignment and buffer sizes

**If you see "Preintegration result: 5-15 measurements":**
✅ IMU integration working → Check next step

---

**If you see "Displacement too small":**
❌ Not enough motion → Try:
- Moving phone more (>5cm)
- Rotating phone
- Walking with phone

**If you see displacement values:**
✅ Motion detected → Check next step

---

**If you see "Scale X.XX outside valid range":**
❌ Scale unrealistic → Possible causes:
1. Vision tracking failed (very small vision_displacement)
2. IMU acceleration spike (very large imu_displacement)
3. Feature tracking on far-away objects

**If you see "IMU Preintegration: displacement=..., scale=..., smoothed=...":**
✅ **PREINTEGRATION IS WORKING!** 🎉

---

## Method 1: Logcat Analysis (No Movement Required)

### Step 1: Launch app and open logcat

```bash
# Open logcat filtered for our tags
adb logcat -s NavSight-Native IMUPreintegrator VisionModule
```

### Step 2: What to look for

**✅ Successful Initialization:**
```
IMUPreintegrator: Gravity set: direction=(0.000, 0.000, 1.000), magnitude=9.810 m/s²
VisionModule: Initialized from gravity: direction = (0.000, 0.000, 1.000), magnitude = 9.81 m/s²
```

**✅ IMU Data Being Collected:**
```
IMUPreintegrator: Preintegrated X measurements over 0.033 seconds: p=(0.012, -0.003, 0.005), v=(...)
```
- `X measurements` should be > 0 (typically 5-10 between frames at 30 FPS)
- `p=` values should be small but non-zero when moving

**✅ Scale Estimation Working:**
```
VisionModule: IMU Preintegration: displacement=0.014m, scale=1.234 (smoothed=1.156)
```
- `displacement` = IMU-based movement
- `scale` = current frame's scale estimate
- `smoothed` = averaged scale (should stabilize over time)

**❌ Problems to Watch For:**
```
IMUPreintegrator: No IMU measurements available for integration
```
→ IMU data not reaching preintegrator

```
VisionModule: IMU preintegration failed: [error message]
```
→ Check error details

---

## Method 2: Indoor Movement Test (5 minutes)

### Test 1: Stationary Baseline
1. Place phone on table
2. Let it sit for 10 seconds
3. **Expected**: Position should drift very little (<5cm)
4. **Check**: Scale should stay around 1.0 (±0.2)

### Test 2: Simple Forward Motion
1. Hold phone steady, pointing forward
2. Walk forward 1 meter slowly (count steps)
3. Stop and hold steady
4. **Expected**:
   - Position Z should show ~1 meter movement
   - Scale should be reasonable (0.5 - 2.0)
   - Green dots should track well

### Test 3: Side-to-Side Test
1. Hold phone in front of you
2. Move left 0.5m, then right 0.5m (slowly)
3. **Expected**:
   - Position X should show oscillation
   - Should return close to start position
   - Less drift than without preintegration

### Test 4: Rotation Test
1. Hold phone steady
2. Rotate 90° left, then 90° right
3. **Expected**:
   - Rotation values should show ~90° changes
   - Position should barely change (pure rotation)
   - Green dots should track corners/edges well

---

## Method 3: Debug Overlay Verification

### What to watch in the debug overlay:

**Initialization Status:**
```
⏳ Initializing...  → Should change to:
✅ Initialized      (within 1-2 seconds)
```

**IMU Sensors Section:**
```
Accel (m/s²): X:0.12 Y:-0.05 Z:9.81
Gyro (rad/s): X:0.00 Y:0.00 Z:0.00
```

**When stationary:**
- Accel Z should be ~9.8 (gravity)
- Accel X, Y should be small (~0)
- Gyro should be near zero

**When moving:**
- Accel values should change
- Gyro should show rotation rates
- Values should be smooth, not jumping wildly

**Scale:**
```
Scale: 1.234 m/unit  (should be green when > 0.01)
```
- Should stabilize after 5-10 seconds of movement
- Typical range: 0.5 - 2.0
- Should not jump wildly (±0.1 per frame max)

**Position:**
```
Position (m): X: 0.45  Y: -0.12  Z: 1.23
```
- Should accumulate as you move
- Should be relatively smooth
- Drift should be low when stationary

---

## Method 4: Comparison Test (Before vs After)

### Create a test scenario:

**Test Route** (can do indoors):
1. Start at a marked position (e.g., door)
2. Walk to corner (count steps)
3. Turn 90°
4. Walk back to door
5. Check final position error

**Expected Improvement:**
- **Before preintegration**: 20-50% position error
- **After preintegration**: 5-15% position error

**Example:**
- Walked 3m forward, 3m back
- Before: Shows 2.4m or 3.8m (20-27% error)
- After: Shows 2.7m or 3.2m (5-10% error)

---

## Method 5: Logcat Commands for Analysis

### Real-time monitoring:
```bash
# Watch only preintegration logs
adb logcat -s IMUPreintegrator:D

# Watch scale estimation
adb logcat | grep "scale="

# Watch for errors
adb logcat -s NavSight-Native:E VisionModule:E IMUPreintegrator:E

# Full debug output
adb logcat -s NavSight-Native:D VisionModule:D IMUPreintegrator:D
```

### Save logs for analysis:
```bash
# Capture 1 minute of testing
adb logcat -d > vio_test_$(date +%Y%m%d_%H%M%S).log

# Then search the file
grep "Preintegrated" vio_test_*.log
grep "scale=" vio_test_*.log
```

---

## Quick Verification Checklist

Run the app and check these items:

- [ ] App builds without errors
- [ ] App launches successfully
- [ ] Debug overlay appears
- [ ] "✅ Initialized" appears within 1-2 seconds
- [ ] IMU sensors show reasonable values (accel Z ≈ 9.8)
- [ ] Scale value is green (> 0.01)
- [ ] Logcat shows "Preintegrated X measurements" (X > 0)
- [ ] Moving phone changes position values
- [ ] Position doesn't drift wildly when stationary
- [ ] Green dots track features smoothly
- [ ] No errors in logcat

**If all checked ✅ → Preintegration is working!**

---

## Expected Performance Metrics

### Good Performance Indicators:

**IMU Collection:**
- 5-10 measurements between camera frames (30 FPS)
- Consistent timestamps (no huge gaps)

**Scale Estimation:**
- Converges within 5-10 seconds
- Stable (±0.1 variation)
- Reasonable value (0.5 - 2.0)

**Position Tracking:**
- Smooth accumulation
- Low drift when stationary (<5cm/minute)
- Returns near origin on closed loops

**Rotation:**
- Matches phone orientation
- Smooth changes
- No sudden jumps

### Red Flags:

❌ **"No IMU measurements available"** → Sensor data not flowing
❌ **Scale jumps wildly** (0.1 → 5.0 → 0.3) → Need more smoothing
❌ **Position explodes** (suddenly 100m) → Scale estimation issue
❌ **Never initializes** → Gravity estimation failing
❌ **Frequent crashes** → Check deadlocks, memory issues

---

## Sample Test Session (5 minutes)

### Minute 1: Launch & Initialize
```bash
adb logcat -s NavSight-Native IMUPreintegrator VisionModule &
# Launch app, wait for "Initialized"
```

### Minute 2: Stationary Test
- Place phone on table
- Watch position - should barely move
- Note scale value

### Minute 3: Forward Motion
- Walk 1 meter forward
- Check position Z increased by ~1m
- Check scale is reasonable

### Minute 4: Rotation Test
- Rotate phone 90°
- Check rotation values changed
- Check position barely changed

### Minute 5: Analysis
- Save logcat output
- Check for errors
- Verify scale stabilized

---

## Automated Testing (Future)

For more rigorous testing, you could:

1. **Record IMU + Camera data**
   - Capture a test sequence
   - Replay it repeatedly
   - Compare results

2. **Use public datasets**
   - EuRoC MAV dataset
   - TUM VI dataset
   - Compare against ground truth

3. **Synthetic data generator**
   - Generate known trajectory
   - Feed to preintegrator
   - Check if output matches

But for now, **indoor movement + logcat analysis** is sufficient to verify it works!

---

## Next Steps After Verification

Once you confirm preintegration is working:

1. ✅ **Tune parameters** (smoothing weights, thresholds)
2. ✅ **Add velocity state** (easy next step)
3. ✅ **Parallelize feature detection** (performance boost)
4. ✅ **Create IMU thread** (better real-time performance)

The foundation is now solid - time to build on it!
