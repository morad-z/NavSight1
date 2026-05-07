# ADR-017 — Implementation spec: GPS course as bounded yaw

**Companion to:** [ADR-017-gps-course-as-bounded-yaw.md](./ADR-017-gps-course-as-bounded-yaw.md)
**Status:** **WITHDRAWN 2026-05-07** — see the parent ADR-017
withdrawal notice. NavSight is VIO-only; the EKF must not fuse GPS,
including GPS course-over-ground for yaw. **Do not implement.**
**Owner:** Morad Zubidat (sensor fusion)

## Dependency

This spec must not be implemented until the parallel
"visual-yaw heading convention sign fix" lands. Reuses
`EKFState::updateGravityAlignedYaw`'s H-Jacobian — if that path is
sign-broken at the time of this work, the GPS course channel will
push yaw the wrong way at 1 Hz, strictly worse than today's
unanchored drift.

## EKF method

### Header (proposed addition to `EKFState.h`, after `updatePDRStep`)

```cpp
// ── ADR-017: GPS course-over-ground as a bounded yaw observation ────────
//
// 1-DOF absolute-yaw update derived from GPS course. Reuses
// updateGravityAlignedYaw's H-Jacobian internally; no new EKF math.
//
//   course_rad_y_up : world-frame heading in y-up navigation convention
//                     (North = 0, East = +π/2). Caller computes
//                     atan2(dE, dN) from consecutive GPS samples.
//   speed_mps       : magnitude of inter-sample displacement vector
//                     (m/s). Caller filters out sub-0.5 s, sub-0.5 m
//                     pairs.
//   gps_accuracy_m  : phone-reported horizontal accuracy (1-σ), m.
//
// Returns true iff the update was applied. Returns false (silently)
// when:
//   - speed_mps      ≤ 1.5      (speed gate; ADR-017 §"Gating")
//   - gps_accuracy_m ≥ 10.0     (outer accuracy gate; same)
//   - !full_initialized_         (EKF not bootstrapped)
//   - !std::isfinite() of any input
//
// Variance:
//   var_yaw = (gps_accuracy_m / speed_mps)²  +  (5° in rad)²
bool updateGpsCourseYaw(double course_rad_y_up,
                        double speed_mps,
                        double gps_accuracy_m);
```

### Implementation (proposed addition to `EKFState.cpp`)

```cpp
bool EKFState::updateGpsCourseYaw(double course_rad_y_up,
                                  double speed_mps,
                                  double gps_accuracy_m) {
    // Hard gates (ADR-017).
    if (!full_initialized_) return false;
    if (!std::isfinite(course_rad_y_up)) return false;
    if (!std::isfinite(speed_mps) || speed_mps <= 1.5) return false;
    if (!std::isfinite(gps_accuracy_m) || gps_accuracy_m >= 10.0) {
        return false;
    }

    // Variance model: derived noise from (gacc / speed) plus a (5°)²
    // floor for residual phone-orientation-vs-walk-direction
    // uncertainty (ADR-017 §"Variance model"). Empirical floor is
    // pinned to the re-walk acceptance criterion #6.
    constexpr double kFloorRad = 5.0 * M_PI / 180.0;          // 0.0873 rad
    constexpr double kFloorVarRad2 = kFloorRad * kFloorRad;   // ≈ 7.6e-3 rad²
    const double sigma_from_gps = gps_accuracy_m / speed_mps;
    const double var_yaw =
        sigma_from_gps * sigma_from_gps + kFloorVarRad2;

    // Reuse the existing 1-DOF gravity-aligned yaw update path.
    // GPS course is already an absolute world-frame heading; no
    // body-frame alignment sandwich needed → roll = pitch = 0.
    return updateGravityAlignedYaw(course_rad_y_up, var_yaw, 0.0, 0.0);
}
```

## Caller contract: `Tracker::onLocationUpdate` (new)

There is no `Tracker::onLocationUpdate` today. The Java/Kotlin GPS
callback at
`app/src/main/java/com/example/navsight1/SensorRepository.kt:445`
(`locationCallback.onLocationResult`) writes only to
`_currentLocation`/`_startLocation` MutableStateFlows; no JNI bridge
to native.

This spec proposes adding:

1. **JNI binding** in `app/src/main/cpp/native-lib.cpp`:
   ```cpp
   extern "C" JNIEXPORT void JNICALL
   Java_com_example_navsight1_NavSightJNI_onLocationUpdate(
       JNIEnv*, jclass,
       jdouble lat, jdouble lng, jdouble accuracy_m,
       jlong timestamp_ns) {
       g_tracker.onLocationUpdate(lat, lng, accuracy_m, timestamp_ns);
   }
   ```
2. **Kotlin wrapper** in `NavSightJNI` and a delegate call from
   `SensorRepository.locationCallback.onLocationResult` after the
   existing `_currentLocation.value = it` line.
3. **`Tracker::onLocationUpdate`**:
   ```cpp
   // Tracker.cpp — sketch.
   void Tracker::onLocationUpdate(double lat, double lng,
                                  double accuracy_m,
                                  int64_t timestamp_ns) {
       Vec2 enu_now = latLngToLocalEnu(lat, lng);

       // Caller-side filter: skip if previous accepted sample is
       // < 0.5 s old or < 0.5 m away. These thresholds are sample-
       // cadence properties — keep EKFState ignorant of them.
       if (last_gps_sample_.valid) {
           const double dt = (timestamp_ns - last_gps_sample_.t_ns) * 1e-9;
           const Vec2 d = enu_now - last_gps_sample_.enu;
           const double disp = std::hypot(d.x, d.y);
           if (dt >= 0.5 && disp >= 0.5) {
               // Y-up navigation: North = 0, East = +π/2.
               // Matches EKFState::getYaw / updateGravityAlignedYaw
               // convention.
               const double course = std::atan2(d.x /*east*/, d.y /*north*/);
               const double speed_mps = disp / dt;
               ekf_.updateGpsCourseYaw(course, speed_mps, accuracy_m);
               last_gps_sample_ = { enu_now, timestamp_ns, true };
           }
           // else: too close in time/space, keep the previous anchor.
       } else {
           last_gps_sample_ = { enu_now, timestamp_ns, true };
       }
   }
   ```

### Caller-side thresholds (cadence-pinned, NOT in EKFState)

- **Inter-sample dt floor: 0.5 s.** Below this, the fused-location
  callback at 1 Hz with `setMinUpdateIntervalMillis(500L)`
  (`SensorRepository.kt:442`) cannot deliver two distinct samples;
  the floor is defensive belt-and-braces.
- **Inter-sample displacement floor: 0.5 m.** Below this, `atan2(dE, dN)`
  is dominated by intra-fix noise (1 m `gacc` plus 0.3 m real
  displacement is mostly noise). The EKFState speed gate would catch
  it (0.3 m / 0.5 s = 0.6 m/s < 1.5), but the displacement floor
  saves the CPU of computing course + invoking the EKF method.

These belong in `Tracker`, not `EKFState`, because they are
properties of the GPS sample stream cadence. The EKF method itself
is cadence-agnostic — it consumes (course, speed, accuracy) and
applies physics-derived gates only.

## Test plan

`tests/cpp/test_gps_course_yaw.cpp`, mirroring
`tests/cpp/test_loop_closure.cpp` structure:

1. **Speed-gate rejection.** `updateGpsCourseYaw(0.0, 1.0, 5.0)` on
   full-init EKF returns `false`, EKF state unchanged.
2. **Accuracy-gate rejection.** `updateGpsCourseYaw(0.0, 2.0, 15.0)`
   returns `false`, EKF state unchanged.
3. **Variance floor enforced.** With `speed = 100` and `gacc = 0.01`,
   effective `var_yaw` reflects the (5°)² floor, not the near-zero
   gps-derived term.
4. **Sign-convention sanity.** Initial EKF at yaw = 0 (North), call
   with `course = +π/2` (East), `speed = 2`, `gacc = 1`: post-update
   yaw moves *toward* +π/2, not −π/2. (This depends on the visual-yaw
   sign fix being in.)
5. **Happy-path: covariance reduces.** After a clean update, world-Y
   attitude variance trace strictly decreases.
6. **Replay determinism.** Two consecutive replays of the same sim
   segment produce identical post-update covariance and mean.

## Files touched (proposed; not yet written)

- `app/src/main/cpp/EKFState.h` — declaration.
- `app/src/main/cpp/EKFState.cpp` — definition.
- `app/src/main/cpp/Tracker.h` / `.cpp` — `onLocationUpdate` method
  and `last_gps_sample_` member.
- `app/src/main/cpp/native-lib.cpp` — JNI binding.
- `app/src/main/java/com/example/navsight1/NavSightJNI.kt` — Kotlin
  wrapper for the JNI binding.
- `app/src/main/java/com/example/navsight1/SensorRepository.kt:445`
  — extend the existing `onLocationResult` to call the JNI binding.
- `tests/cpp/test_gps_course_yaw.cpp` — new test file.
- `scripts/analyze_sim.py` — add `gps_course_yaw_accepted_count`
  reader, mirroring the loop-closure accept-count fields in
  `event_summary`.
