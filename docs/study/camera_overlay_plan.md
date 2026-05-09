# Camera Overlay — KLT Feature Visualization Plan

## Goal

Render KLT-tracked feature points as small dots overlaid on the live camera
preview, so the developer/user can visually verify what the C++ VIO front-end
is currently tracking. This is the simplest, highest-information overlay we
can ship — Phase 1 of a wider plan that later adds 3D-projected SLAM points
and loop-closure flashes.

## Scope of this plan

**Phase 1 (this PR)** — KLT tracked feature dots only.
**Phase 2 (later)** — colored by feature age (new/mature/lost).
**Phase 3 (later)** — projected 3D SLAM points + loop-closure flash.

Phase 1 deliberately does NOT colorize by age — `VioData.trackedPoints` is a
flat snapshot and there's no age info plumbed yet. We render a single style
(small teal dot) and call it a day. Adding age requires a JNI change which we
defer.

## Existing infrastructure

| Piece | Location | Status |
|---|---|---|
| `VioData.trackedPoints: FloatArray` flat `[x0,y0, x1,y1, ...]` | `VioData.kt:29` | Already plumbed |
| C++ source: `out.trackedPoints = std::move(tracked_pts_flat)` | `Tracker.cpp:2961` (populated at `Tracker.cpp:1026-1031`) | Pixel coords from `next_good_buf_` (analyzer-native space, 640×480 landscape) |
| JNI marshal | `native-lib.cpp:472-479` | `SetFloatArrayRegion` into a `jfloatArray` |
| `_vioState.value = vio` published per-frame | `SensorRepository.kt:792` | ~30 Hz |
| `viewModel.vioState` (Compose state) | `NavSightViewModel.kt:73, 241` | Throttled to 200 ms (5 Hz) — fine for overlay |
| Camera preview composable | `CameraUi.kt:42 CameraViewComposable` | `AndroidView { PreviewView }`, `FILL_CENTER` |
| Existing rotation helper | `CalibrationScreenUi.kt:932 rotatePoint(x,y,w,h,rot)` + `rotatedDims` | Reusable; analyzer→preview math already solved |

## Coordinate spaces — the four-stage transform

The KLT points start in C++ analyzer pixel space and end in Compose Canvas
pixel space. The chain is:

1. **Analyzer space** (e.g. `640×480`, sensor-native landscape).
   - Source: `VioData.trackedPoints` from `next_good_buf_` (Tracker.cpp:1028-1030).
2. **Rotated/preview space** — analyzer rotated by `image.imageInfo.rotationDegrees`
   (typically `90` on portrait-held phones), giving e.g. `480×640` upright.
3. **PreviewView fit space** — `PreviewView.scaleType = FILL_CENTER` performs
   a center-crop scale to fill the view bounds, preserving aspect.
4. **Canvas/screen space** — pixel coordinates inside the Compose `Canvas`
   composable that overlays the `AndroidView`.

`PreviewView` resolves the FILL_CENTER fit internally — Compose's `Canvas` on
top of `AndroidView` shares the same bounds (`fillMaxSize`), so we can do the
math against the Canvas size as if it were the view bounds. We must replicate
the FILL_CENTER fit ourselves on the dot positions, otherwise dots will be
wrong on devices where the analyzer aspect doesn't match the view aspect
(very common — 4:3 analyzer vs the device's 19.5:9 screen).

### FILL_CENTER fit math

Given preview dimensions `(prevW, prevH)` and view dimensions `(viewW, viewH)`:

```
sx = viewW / prevW
sy = viewH / prevH
s  = max(sx, sy)            // FILL_CENTER: scale so smaller dim fills exactly,
                            // larger dim overflows symmetrically (cropped).
ox = (viewW - prevW * s) / 2  // negative when this axis is the cropped one
oy = (viewH - prevH * s) / 2
canvasX = ox + previewX * s
canvasY = oy + previewY * s
```

If we used `FIT_CENTER` (letterboxing), the only change is `s = min(sx, sy)`.
Since `CameraUi.kt:48` declares `FILL_CENTER`, we use `max`.

## What needs plumbing (minimal)

Three small additions:

### 1. Expose analyzer dims + rotation from the camera path

`SensorRepository` already has `image.width`, `image.height`, and analyzer
rotation can be read via `image.imageInfo.rotationDegrees`. Today none of
these reach the UI. We add a single Compose-state field on `NavSightViewModel`:

```kotlin
data class CameraFrameGeometry(
    val analyzerWidth: Int,
    val analyzerHeight: Int,
    val rotationDegrees: Int,
)

// In NavSightViewModel:
var cameraFrameGeometry by mutableStateOf<CameraFrameGeometry?>(null); private set

fun updateCameraFrameGeometry(w: Int, h: Int, rot: Int) {
    val cur = cameraFrameGeometry
    if (cur == null || cur.analyzerWidth != w || cur.analyzerHeight != h ||
        cur.rotationDegrees != rot) {
        cameraFrameGeometry = CameraFrameGeometry(w, h, rot)
    }
}
```

Updated from `SensorRepository.processCameraFrame` (it already has `w`, `h`):

```kotlin
val rot = image.imageInfo.rotationDegrees
viewModel.updateCameraFrameGeometry(w, h, rot)
```

Wait — `SensorRepository` doesn't reference `viewModel`. We need to plumb
through the existing pattern. Cleanest path: expose a StateFlow from
`SensorRepository` and have the ViewModel collect it. But this is overkill
for one tiny piece of geometry that only changes once per session. Simpler:

The repository already has a `private val _vioState = MutableStateFlow(...)`.
We add a sibling `_cameraFrameGeometry = MutableStateFlow<CameraFrameGeometry?>(null)`,
expose `cameraFrameGeometry: StateFlow<CameraFrameGeometry?>`, and have the
ViewModel adapt it to a Compose state field in its `init` block — same pattern
as the 8 other adapters at `NavSightViewModel.kt:196-224`.

### 2. New composable: `CameraFeatureOverlay`

A small `Canvas` that reads `viewModel.vioState.trackedPoints` plus
`viewModel.cameraFrameGeometry`, replicates the analyzer→canvas transform,
and draws filled circles. Lives in `CameraUi.kt`.

```kotlin
@Composable
fun CameraFeatureOverlay(viewModel: NavSightViewModel, pal: NavPalette) {
    val vio = viewModel.vioState
    val geom = viewModel.cameraFrameGeometry
    if (geom == null || vio.trackedPoints.isEmpty()) return

    Canvas(Modifier.fillMaxSize()) {
        val (prevW, prevH) = rotatedDimsForOverlay(
            geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
        )
        if (prevW == 0 || prevH == 0) return@Canvas

        val viewW = size.width
        val viewH = size.height
        val sx = viewW / prevW.toFloat()
        val sy = viewH / prevH.toFloat()
        val s = maxOf(sx, sy)         // FILL_CENTER
        val ox = (viewW - prevW * s) / 2f
        val oy = (viewH - prevH * s) / 2f

        val pts = vio.trackedPoints
        val n = pts.size / 2
        val color = pal.teal.copy(alpha = 0.85f)
        for (i in 0 until n) {
            val ax = pts[2 * i]
            val ay = pts[2 * i + 1]
            val r = rotatePointForOverlay(
                ax, ay, geom.analyzerWidth, geom.analyzerHeight, geom.rotationDegrees
            )
            val cx = ox + r.x * s
            val cy = oy + r.y * s
            drawCircle(color = color, radius = 6f, center = Offset(cx, cy))
        }
    }
}
```

Where `rotatePointForOverlay` and `rotatedDimsForOverlay` mirror the helpers
already living in `CalibrationScreenUi.kt:932,940`. We don't reach across
files for `private fun`; we add file-level helpers in `CameraUi.kt` (a
~10-line copy is cheaper than a refactor for two callers).

### 3. Wire the overlay into `MapScreenUi`

The full-screen camera lives at `MapScreenUi.kt:90-96`:

```kotlin
key(calibrationVisible) {
    Box(modifier = if (cameraVisible && !calibrationVisible) Modifier.fillMaxSize() else Modifier.size(1.dp).alpha(0f)) {
        if (!calibrationVisible) {
            CameraViewComposable(viewModel)
        }
    }
}
```

We add a sibling `CameraFeatureOverlay(viewModel, pal)` in the same `Box`,
positioned after `CameraViewComposable` so it draws on top:

```kotlin
key(calibrationVisible) {
    Box(modifier = if (cameraVisible && !calibrationVisible) Modifier.fillMaxSize() else Modifier.size(1.dp).alpha(0f)) {
        if (!calibrationVisible) {
            CameraViewComposable(viewModel)
            if (cameraVisible) CameraFeatureOverlay(viewModel, pal)
        }
    }
}
```

Gating on `cameraVisible` skips the overlay when the camera is in the offscreen
"always alive" state (where the camera is rendered as a 1-dp invisible box for
keep-alive purposes). The overlay itself is cheap, but reading `vioState` and
walking up to ~150 points 5×/s for nothing is wasteful.

## Files to edit (minimum)

| File | Change |
|---|---|
| `NavSightViewModel.kt` | + `cameraFrameGeometry` Compose state, + adapter in `init`, + private setter (or expose flow on SensorRepository) |
| `SensorRepository.kt` | + `_cameraFrameGeometry` StateFlow, + write site in `processCameraFrame` near line 690-692, + reset to null in `resetAll` wipe block |
| `CameraUi.kt` | + `CameraFeatureOverlay` composable, + private `rotatePointFor*` helpers |
| `MapScreenUi.kt` | + 1 line: invoke `CameraFeatureOverlay(viewModel, pal)` inside the camera Box |

## Performance considerations

- **Allocation**: the only per-frame allocation in the overlay is a single
  `Offset(...)` per circle (Compose's `drawCircle` accepts `Offset` directly).
  For 100 features that's ~100 small heap allocations every 200 ms = ~500/s
  of small short-lived objects. Acceptable. We do NOT cache an array because
  positions change every frame anyway.
- **Recomposition**: `vioState` is throttled at 200 ms in the VM; the Canvas
  recomposes at most 5 Hz. `cameraFrameGeometry` only changes when analyzer
  geometry changes — typically once per session.
- **Color**: `pal.teal` is a `Color` value class; `.copy(alpha=...)` produces
  a new `Color` per draw. We can lift it out once but it's effectively
  zero-cost (long-packed primitive). Skip optimization unless profile says.
- **Off-camera path**: gated by `cameraVisible` in `MapScreenUi`, so the
  overlay only draws while the camera is actually visible.

## Edge cases

| Case | Behavior |
|---|---|
| `trackedPoints.isEmpty()` | overlay returns early — no draw |
| `cameraFrameGeometry == null` | overlay returns early — happens for the first ~33 ms before the first analyzer frame lands |
| Calibration screen open | `MapScreenUi.kt:92` already gates `CameraViewComposable` on `!calibrationVisible`; we honor the same gate |
| App reset (`resetAll`) | `_vioState.value = VioData()` clears `trackedPoints`; overlay shows no dots until VIO re-initializes — correct |
| Device rotation mid-session | `cameraFrameGeometry.rotationDegrees` updates the next analyzer frame — overlay reflects new orientation within ~33 ms |
| Analyzer aspect != view aspect | `FILL_CENTER` math handles it; dots near the cropped edges may be off-screen, which matches the camera image |

## Acceptance criteria

1. Build succeeds: `./gradlew assembleDebug`.
2. On phone, opening the camera (camera FAB on map) shows small teal dots
   following texture-rich regions of the live preview.
3. Dots move with the scene as the user pans the phone.
4. Closing the camera (X button) hides the overlay; opening again restores it.
5. Reset (Rides → reset) clears the dots; they reappear when VIO re-initializes.
6. No new ANRs, no new logcat warnings, no measurable FPS impact on the
   preview (still 30 Hz).

## Out of scope for Phase 1

- Color by age (need JNI extension for per-feature age).
- 3D projected SLAM points (need SLAM-state JNI extension).
- Loop-closure flash (need event-counter delta detection).
- Lost-feature fade (need previous-frame point retention).

These are tracked as Phase 2 and Phase 3 in the originating task.
