# Allan-Variance IMU Calibration

Per-device noise characterization for the S21 Ultra's gyro + accel. Output
plugs directly into the EKF process-noise constants in `EKFState.cpp`,
replacing the ad-hoc values that are currently miscalibrating every chi²
gate and bias-estimation rate.

## Why we need this

Today the EKF uses hand-picked values (e.g. `σ_acc = 0.1 m/s²`,
`σ_gyro = 0.005 rad/s`) that we never measured. They're probably wrong by a
factor of 2-5 for our specific device. Symptoms:
- Gyro bias estimate converges too slowly → yaw drift dominates open-loop error
- chi² gates either over-reject good visual updates or admit garbage
- ZUPT damping is mis-scaled

Allan variance is the standard (IEEE 1554-2005) way to measure these per
device. ARKit / ARCore use factory-calibrated values; we have to do it
ourselves.

## Quick start (existing sim data — preview only)

If you want to see the workflow before doing a proper 2-hour recording:

```bash
python scripts/allan_variance/calibrate.py \
    tests/sims/regression/visual/v33_two_loops.json
```

This will run on a walking-route sim, which is **not stationary** — the
numbers will be inflated by motion. Use this only to verify the analyzer
runs; do NOT bake the output into `EKFState.cpp`.

## Proper procedure

### 1. Phone setup
- Place phone **flat on a stable surface** that is not vibrating (NOT on a
  laptop or near a fan). A bookshelf or wooden table is ideal.
- Plug into power so the battery doesn't drain.
- Disable auto-rotate and screen-off (calibration screen handles this; if
  you use a recording-mode workaround, enable airplane mode + max screen
  timeout).
- Wait 10 minutes after placing — let the phone reach thermal equilibrium.
  Cold IMU has a different bias from hot IMU, and the gyro bias drift you
  measure should reflect steady-state behavior.

### 2. Record
**Goal**: 1-2 hours of stationary IMU data.

Today, NavSight only logs IMU at the VIO output rate (~19 Hz observed). For
a first pass:
- Start a normal sim recording from the calibration / main screen
- Leave the phone **untouched** for 2 hours
- Stop recording, pull the JSON via adb

A dedicated high-rate (`SENSOR_DELAY_FASTEST`) recording mode is a planned
addition (see Task #63) — it bypasses VIO and writes a raw 7-column CSV at
~200 Hz, which is the correct sample rate for Allan analysis.

### 3. Pull & analyze

```bash
$ADB="C:\Users\morad\AppData\Local\Android\Sdk\platform-tools\adb.exe"
$SRC="/sdcard/Android/data/com.example.navsight1/files"

# Pull the long-stationary sim
& $ADB pull "$SRC/simulation_data_<TS>.json" "tests/sims/allan/stationary_2h.json"

# Run the analyzer
python scripts/allan_variance/calibrate.py tests/sims/allan/stationary_2h.json
```

### 4. Interpret outputs

The analyzer writes three artifacts to `scripts/allan_variance/output/`:

- **`allan_dev_gyro.png`** — log-log Allan deviation curve for gx/gy/gz
- **`allan_dev_accel.png`** — log-log Allan deviation curve for ax/ay/az
- **`ekf_constants_snippet.txt`** — C++ block with the four values

Sanity-check the plots:
- **Short tau** (left side) should show a `-1/2` slope — that's white noise
- **Long tau** (right side) should curve up toward `+1/2` slope — that's bias
  random walk
- A flat region in between is normal (1/f flicker)
- If the whole curve is flat or wandering, the recording was too noisy
  (vibration, accidental motion, thermal transient) — re-record

### 5. Bake into EKFState.cpp

The snippet looks like:

```cpp
constexpr double SIGMA_G_N  = 1.234e-04;  // gyro white noise
constexpr double SIGMA_BG_N = 5.678e-06;  // gyro bias RW
constexpr double SIGMA_A_N  = 9.012e-03;  // accel white noise
constexpr double SIGMA_BA_N = 3.456e-04;  // accel bias RW
```

Locate the existing process-noise constants in `EKFState.cpp` (search for
`sigma_g`, `sigma_a`, `sigma_bg`, `sigma_ba`, or the Q-matrix construction)
and replace them. Keep the *old* values in a `/* SUPERSEDED <date> */`
comment block per `feedback_no_deletions`.

Rebuild, install, walk. Compare close-loop on a fixture walk before and
after — the bias-related drift should drop noticeably.

## References

- IEEE Std 1554-2005, *Standard for Inertial Sensor Terminology*
- Maybeck, *Stochastic Models, Estimation, and Control*, vol 1, ch 4
- `allantools` Python library (we use a hand-rolled equivalent for zero deps)

## Caveats

- Sample rate matters. At 19 Hz, the shortest tau is ~0.1 s, which barely
  resolves white noise. At 200 Hz we'd resolve it cleanly. Until the
  high-rate recording mode lands, treat the white-noise number as an
  upper bound.
- Temperature matters. The S21 Ultra warms up under heavy CPU/screen load
  → gyro bias shifts. Allan variance assumes stationarity; thermal drift
  shows up as a spurious bias-RW slope. Keep the phone idle and cool.
- Two-axis runs. If you can, repeat the measurement with the phone face-up
  AND face-down. Gravity loads the accelerometer differently in each pose
  and helps separate sensor noise from cradle vibration.
