# Camera Overlay — Phase 2 + Phase 3 Plan (plus Phase-1 lag fix)

> Sequel to `camera_overlay_plan.md`. Phase 1 (KLT teal dots) shipped and works,
> but the user reports the dots LAG behind the live preview. This plan adds:
>
> - Task A: kill the Phase-1 lag (root cause: `VioData` is a single
>   MutableStateFlow throttled by the VM to 5 Hz; the overlay only sees a new
>   frame every 200 ms even though native produces ~30 frames/s)
> - Task B: world-anchored 3D SLAM points (the user's main ask)
> - Task C: KLT feature age coloring (green / yellow / red)
> - Task D: loop-closure flash overlay
>
> All four are additive, surgical, reversible. **No EKF math changes** — only
> read-only JNI accessors.

---

## TASK A — Phase 1 lag root cause + fix

### Diagnosis

`SensorRepository._vioState` emits a fresh `VioData` per native frame (~30 Hz).
`NavSightViewModel.handleVioUpdate` then **throttles** the Compose-state field
`vioState` to 200 ms (`UI_UPDATE_THROTTLE_MS = 200L`):

```kotlin
// NavSightViewModel.kt:189, 246-253
private val UI_UPDATE_THROTTLE_MS = 200L
…
val shouldUpdateUI = (nowMs - lastUiUpdateTimeMs) >= UI_UPDATE_THROTTLE_MS
if (shouldUpdateUI) {
    lastUiUpdateTimeMs = nowMs
    vioState = vio   // ← only here does the overlay see a new frame
    …
}
```

Effect: KLT dots update at most 5 Hz. At a 1 m/s walking pan that's ~6 cm of
real-world drift per visible frame; on a 19.5:9 phone with FILL_CENTER scaling
it shows up as a several-dot-radius lag on screen edges.

### Fix

Bypass the 200 ms gate **for overlay-only fields**. Keep the gate for
heavier consumers (path history, sigma reads, GPS snap, crash snapshot)
because those make IO calls or allocate. Use a **separate Compose-state
field** updated on every native frame (cheap — float-array reference swap).

Three new pieces of state on `NavSightViewModel`:

```kotlin
/** Latest KLT pixels [x0,y0,x1,y1,...]. Updated on every native frame
 *  (~30 Hz), bypassing the 200 ms UI throttle so the overlay tracks
 *  the live preview. */
var trackedPointsLive by mutableStateOf<FloatArray>(floatArrayOf())
    private set

/** Phase 2 — parallel ages in milliseconds. Length = trackedPointsLive.size / 2. */
var trackedPointAgesLive by mutableStateOf<IntArray>(intArrayOf())
    private set

/** Phase 3 — flat SLAM snapshot [fid,x,y,z,fid,x,y,z,...] in Y-up world coords.
 *  Updated on every native frame; ≤ 4 × MAX_SLAM_FEATURES = 48 floats. */
var slamSnapshotLive by mutableStateOf<FloatArray>(floatArrayOf())
    private set

/** Phase 3 — current camera pose for projection. 16 floats:
 *  - [0..8] R_world_cam row-major (camera→world rotation)
 *  - [9..11] t_world_cam (camera position in world, ≈ p_G)
 *  - [12..15] fx, fy, cx, cy
 *  Length-0 when EKF not yet full-initialized. */
var cameraPoseLive by mutableStateOf<FloatArray>(floatArrayOf())
    private set
```

These are written from the SAME `handleVioUpdate` block but **outside** the
`shouldUpdateUI` gate. Compose only invalidates the Canvas overlay (which
reads these fields and nothing else). All other `vioState` consumers stay on
the 5 Hz path — no regression for them.

#### Why a separate `mutableStateOf<FloatArray>` instead of `derivedStateOf`?

`derivedStateOf` would still depend on `vioState`, which changes only every
200 ms — derived state can't fire faster than its dependency.

#### Why per-frame and not throttled to 15 Hz?

The cost per frame is:
- 1 `FloatArray` reference swap (no copy — JNI already owns the array)
- 1 Compose snapshot invalidation
- 1 Canvas redraw with at most ~150 circles (Phase 1) or ~12 circles (Phase 3)

That's well under 1 ms on a Pixel-class GPU. Throttling is unnecessary
complexity — the real cost was being yoked to the unrelated 200 ms throttle.

---

## TASK B — Phase 3: World-anchored SLAM 3D points

### B.1 — JNI: `getSlamSnapshot(out: FloatArray): Int`

Returns the count of SLAM features written into `out` (callers preallocate
`FloatArray(4 * MAX_SLAM_FEATURES)` once and reuse it). Layout per feature:

```
[ feature_id, world_x, world_y, world_z, ...]
```

**Coordinate convention**: same Y-up swap the rest of `processCameraFrameDirect`
applies (native-lib.cpp:414-416):

```cpp
out[4*i + 0] = (float) feature_id;
out[4*i + 1] = (float) p_world_zup[0];   // East — same in both
out[4*i + 2] = (float) p_world_zup[2];   // Up   — Z-up index 2 → Y-up index 1
out[4*i + 3] = (float) p_world_zup[1];   // North — Z-up index 1 → Y-up index 2
```

Native impl skeleton:

```cpp
JNIEXPORT jint JNICALL
Java_com_example_navsight1_NativeBridge_getSlamSnapshot(
        JNIEnv* env, jobject, jfloatArray out) {
    std::shared_ptr<VioEngine> vision;
    {
        std::lock_guard<std::mutex> lock(state_mutex);
        vision = g_vision;
    }
    if (!vision || !out) return 0;
    const EKFState* ekf = vision->getEKFState();   // new accessor
    if (!ekf) return 0;
    const int n = ekf->getSlamFeatureCount();
    if (n <= 0) return 0;
    const int cap = env->GetArrayLength(out) / 4;
    const int writeN = std::min(n, cap);
    if (writeN <= 0) return 0;
    std::vector<float> buf(writeN * 4);
    int wrote = 0;
    cv::Mat p_global;
    for (int i = 0; i < writeN; ++i) {
        if (!ekf->getSlamFeatureGlobalPosition(i, p_global)) continue;
        const int fid = ekf->getSlamFeatureId(i);   // need new helper or peek slam_features_
        buf[wrote*4 + 0] = static_cast<float>(fid);
        buf[wrote*4 + 1] = static_cast<float>(p_global.at<double>(0));   // East
        buf[wrote*4 + 2] = static_cast<float>(p_global.at<double>(2));   // Up   (Z→Y)
        buf[wrote*4 + 3] = static_cast<float>(p_global.at<double>(1));   // North(Y→Z)
        ++wrote;
    }
    if (wrote > 0) env->SetFloatArrayRegion(out, 0, wrote * 4, buf.data());
    return wrote;
}
```

`VioEngine::getEKFState()` is a new const accessor returning
`tracker_.getEKF()` (already exists at Tracker.h:120).

`EKFState::getSlamFeatureId(int slot)` is a new tiny helper that returns
`slam_features_[slot].feature_id` after a bounds check (already used inside
the class — just need a public read accessor).

### B.2 — JNI: `getCurrentCameraPose(out: FloatArray): Boolean`

Returns true and fills `out[16]` when EKF is full-initialized. Otherwise
returns false and leaves `out` untouched.

`out[0..8]` = R_world_cam in Y-up world frame, row-major:
- world frame is the SAME Y-up frame the rest of Kotlin uses (so the SLAM
  positions from B.1 and the camera pose are in the same frame)
- camera frame is the OpenCV camera frame (X=right, Y=down, Z=forward)

Computation:
```
R_GtoI         (world Z-up → body):   from ekf->getRotation()
R_bc           (body → camera OpenCV): from ekf->getExtrinsicsRotation()
R_GtoC         = R_bc * R_GtoI                   // world Z-up → camera
R_world_cam_zup = R_GtoC.t()                     // camera → world Z-up
```

Then permute the world-frame ROWS (= the columns of R because rotation rows are
unit vectors in world frame) to expose Y-up:

```
Z-up world axes (X=East, Y=North, Z=Up) → Y-up world axes (X=East, Y=Up, Z=North)
permutation P = ((1,0,0), (0,0,1), (0,1,0))   // swaps row 1 ↔ row 2
R_world_cam_yup = P * R_world_cam_zup
```

`out[9..11]` = camera position in Y-up world frame:
```
p_zup = ekf->getPosition()   // 3×1, world Z-up
out[9]  = p_zup(0);          // East
out[10] = p_zup(2);          // Up
out[11] = p_zup(1);          // North
```

(This is the BODY position. We approximate camera position ≈ body position —
the lever arm offset is 1-2 cm and the user does NOT see a difference at
typical 1-10 m feature depths. Documenting this so future readers know it's
a deliberate simplification, not a bug.)

`out[12..15]` = fx, fy, cx, cy from `ekf->getSlamFx/Fy/Cx/Cy()`.

### B.3 — Kotlin overlay: `SlamFeatureOverlay`

New composable in CameraUi.kt, mirrors the structure of CameraFeatureOverlay
but reads `slamSnapshotLive`, `cameraPoseLive`, and `cameraFrameGeometry`.

Per feature:
1. Read `(fid, x, y, z)` from the flat snapshot (Y-up world).
2. Compute camera-frame point:
   ```kotlin
   val dx = x - tx; val dy = y - ty; val dz = z - tz
   val xc = R[0]*dx + R[3]*dy + R[6]*dz   // R is row-major, so column 0 is R[0,3,6]
   val yc = R[1]*dx + R[4]*dy + R[7]*dz   // for transpose multiply
   val zc = R[2]*dx + R[5]*dy + R[8]*dz
   ```
   Wait — R is row-major camera→world, so to go world→camera we use the
   transpose: world→camera = R.t() * (p_world - t_world). For a row-major R,
   the transpose-multiply by columns gives:
   ```kotlin
   val xc = R[0]*dx + R[3]*dy + R[6]*dz
   val yc = R[1]*dx + R[4]*dy + R[7]*dz
   val zc = R[2]*dx + R[5]*dy + R[8]*dz
   ```
3. Skip if `zc <= MIN_DEPTH_M`. Derivation of the 0.05 m threshold:
   - 1 px reprojection noise at fx≈500 → ~1/500 rad direction noise.
   - 1° EKF rotation noise → ~0.0175 rad.
   - At 5 cm depth, a 1° rotation pivots the projected pixel by
     5 cm × tan(1°) ≈ 0.87 mm in world, but reprojection mag = focal/depth =
     500/0.05 = 10000 px/m. The noise budget collapses; below 5 cm a 1°
     rotation moves the dot ~17 px on screen which is unstable. Above 5 cm
     the projection stabilizes.
4. Compute analyzer pixel:
   ```kotlin
   val u = fx * xc / zc + cx
   val v = fy * yc / zc + cy
   ```
5. Apply the SAME `overlayRotatePoint` + FILL_CENTER scale-and-crop as KLT
   so SLAM dots align with KLT dots when both observe the same scene point.
6. Draw orange filled circle (radius 8 px, alpha 0.85) plus a 1-px white ring
   outline at radius 9 px to make the SLAM points visually distinct from KLT.

### B.4 — Performance

- ≤ 12 SLAM features (`MAX_SLAM_FEATURES`), so per-frame cost is 12 dot draws.
- JNI calls: 2 per frame (`getSlamSnapshot`, `getCurrentCameraPose`).
- Both run on the camera thread inside `SensorRepository.processCameraFrame`,
  immediately after the existing `processCameraFrameDirect` call. The values
  are stuffed into the new MutableStateFlows on the same path that publishes
  `_vioState`, so no extra threading is introduced.

---

## TASK C — Phase 2: KLT feature age coloring

### C.1 — Native: expose ages parallel to tracked points

Tracker already maintains `feature_ages_` (vector<int>, frames-survived) at
`Tracker.h:240`. We add a parallel int vector to `VisionOutput`:

```cpp
// VioTypes.h
std::vector<float> trackedPoints;       // existing
std::vector<int>   trackedPointAges;    // NEW — ages in FRAMES, parallel
```

In Tracker.cpp around line 1026, fill alongside the existing flat array:
```cpp
std::vector<int> tracked_point_ages;
tracked_point_ages.reserve(next_good_buf_.size());
for (size_t i = 0; i < next_good_buf_.size(); ++i) {
    int age = (i < feature_ages_.size()) ? feature_ages_[i] : 0;
    tracked_point_ages.push_back(age);
}
…
out.trackedPointAges = std::move(tracked_point_ages);
```

### C.2 — JNI: `getLastTrackedPointAges(out: IntArray): Int`

Returns count, fills `out` with the last frame's ages. The call must run
right after `processCameraFrameDirect` so it reads the *same* frame's data.

Implementation note: stash the latest `trackedPointAges` in a new global
`g_tracked_point_ages` (vector<int>, mutex-guarded) inside
`processCameraFrameDirect`. Then `getLastTrackedPointAges` snapshots it.
Same pattern as the existing globals (g_x, g_y, …).

### C.3 — Kotlin: pass ages through to overlay

`SensorRepository.processCameraFrame` calls the new JNI method right after
`processCameraFrameDirect` returns, while still on the VIO executor:

```kotlin
val agesBuf = IntArray(vio.trackedPoints.size / 2)
val ageCount = NativeBridge.getLastTrackedPointAges(agesBuf)
// (ageCount may be < buf.size if VIO published fewer this tick; trim)
val ages = if (ageCount == agesBuf.size) agesBuf else agesBuf.copyOf(ageCount)
```

Then publish `ages` alongside `vio.trackedPoints` in a NEW
`MutableStateFlow<Pair<FloatArray, IntArray>>` so they are always consistent.

The ViewModel collects from this flow and writes `trackedPointsLive` /
`trackedPointAgesLive` together, atomically.

### C.4 — Overlay: color by age

Frames at 30 Hz. 1 second = 30 frames; 3 seconds = 90 frames.
- age < 30 → **green** (Color(0xFF53D34D))
- 30 ≤ age < 90 → **yellow** (Color(0xFFFFEB3B))
- age ≥ 90 → **red** (Color(0xFFEF5350))

Edge case: when `trackedPointAgesLive.size != trackedPointsLive.size / 2`
(should never happen but be safe), fall back to the Phase-1 single-color teal
draw.

---

## TASK D — Loop closure flash

### D.1 — JNI: `getLoopClosureCorrectionsApplied(): Long`

Reads `navsight::eventCounters().loop_closure_corrections_applied`. No mutex
needed (already an atomic). Already-imported `<EventCounters.h>`.

```cpp
JNIEXPORT jlong JNICALL
Java_com_example_navsight1_NativeBridge_getLoopClosureCorrectionsApplied(
        JNIEnv*, jobject) {
    return static_cast<jlong>(
        navsight::eventCounters().loop_closure_corrections_applied
            .load(std::memory_order_relaxed));
}
```

### D.2 — ViewModel: detect counter increments

Poll the counter on every native frame (cheap — atomic relaxed read) and
compare to last value. On increase, set a `flashUntilMs` timestamp 1000 ms
in the future. The overlay reads `flashUntilMs - System.currentTimeMillis()`
and if positive, draws the banner with alpha decaying from 1.0 to 0.0 over
the second.

```kotlin
private var lastLoopClosureCount = 0L
var loopClosureFlashUntilMs by mutableStateOf(0L)
    private set

// In handleVioUpdate, before the throttle gate:
val cur = NativeBridge.getLoopClosureCorrectionsApplied()
if (cur > lastLoopClosureCount) {
    lastLoopClosureCount = cur
    loopClosureFlashUntilMs = System.currentTimeMillis() + 1000L
}
```

### D.3 — Compose: `LoopClosureFlash`

Top-of-screen banner positioned in `MapScreenUi.kt` next to the camera box.
Text: "LOOP CLOSURE", semi-transparent yellow (`Color(0x99FFEB3B)`), 24 sp
bold, fades over 1 s using `withFrameMillis` (smoother than recompose-driven).

---

## File-by-file edit list

### C++

| File | Change |
|---|---|
| `VioTypes.h` | + `std::vector<int> trackedPointAges;` field |
| `EKFState.h` | + `int getSlamFeatureId(int slot) const` (public read accessor) |
| `EKFState.cpp` | + `getSlamFeatureId` impl (3 lines) |
| `VioEngine.h` | + `const EKFState* getEKFState() const { return tracker_.getEKF(); }` |
| `Tracker.cpp` | + populate `out.trackedPointAges` next to `out.trackedPoints` |
| `native-lib.cpp` | + 4 JNI methods: `getSlamSnapshot`, `getCurrentCameraPose`, `getLastTrackedPointAges`, `getLoopClosureCorrectionsApplied`. Plus a `g_tracked_point_ages` global stashed in `processCameraFrameDirect` |

### Kotlin

| File | Change |
|---|---|
| `NativeBridge.kt` | + 4 `external fun` declarations matching JNI signatures |
| `SensorRepository.kt` | + `_trackedSnapshot` flow emitting `(FloatArray pts, IntArray ages)`; + `_slamSnapshot` flow emitting flat `FloatArray`; + `_cameraPose` flow emitting flat `FloatArray`; + `_loopClosureFlashUntilMs` flow emitting `Long`. Reads JNI getters once per frame after `processCameraFrameDirect`. |
| `NavSightViewModel.kt` | + 4 Compose-state fields (`trackedPointsLive`, `trackedPointAgesLive`, `slamSnapshotLive`, `cameraPoseLive`, `loopClosureFlashUntilMs`); + collect each from SensorRepository in `init` |
| `CameraUi.kt` | + `SlamFeatureOverlay` composable; + age-coloring in `CameraFeatureOverlay`; + `LoopClosureFlash` composable; + `worldYupToCamera` helper |
| `MapScreenUi.kt` | + 2 composable invocations inside the camera box (SlamFeatureOverlay, LoopClosureFlash) |

### No changes

- EKF math (no new measurements, no new constants in error state)
- Tracker.cpp loop closure code path (Tracker.cpp:tryRelocalizeWithORB stays untouched)
- VioData.kt — keep the existing constructor signature, no breaking change

---

## Acceptance criteria

1. `./gradlew assembleDebug` succeeds.
2. **Lag fix** (Task A): on phone, KLT teal/colored dots track the live preview
   with no perceptible delay — moving the phone briskly and stopping should
   show the dots stop within 1 frame, not after 200 ms.
3. **Age coloring** (Task C): freshly-detected dots appear green; over ~3 s
   they progress to yellow, then red. Reset clears them all back to green.
4. **SLAM points** (Task B): once the EKF has promoted ≥ 1 SLAM feature
   (visible in event_summary as `slam_promotions > 0` after a short walk),
   orange dots with a white outline appear, and they STICK to the same
   real-world point even when the user pans away and back.
5. **Loop closure flash** (Task D): on a closed loop walk, when
   `loop_closure_corrections_applied` increments, "LOOP CLOSURE" appears
   at the top of the camera view for ~1 s and fades.
6. No new ANRs, no new logcat warnings, no measurable VIO FPS drop
   (`VIO_FPS:` logcat line should stay near 30 Hz).

---

## Out of scope

- Color-by-feature-type (KLT vs SLAM is the only categorisation we surface)
- Persistent dot trails / fade-out for lost features
- Lever-arm correction in `getCurrentCameraPose` (camera ≈ body — see B.2)
- Distortion correction for SLAM reprojection (we use the rectified intrinsics
  the EKF already cached; if the LensCorrector is active the cached intrinsics
  are post-undistort which is what we want — the pixel space matches the
  rectified Y-channel KLT operates on)
