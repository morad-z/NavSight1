# NavSight UI Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the approved "Refined Current" redesign (spec: `docs/superpowers/specs/2026-06-12-ui-redesign-design.md`): disciplined purple theme, slim centered-speed header with tappable VIO status chip, calibration sheet, organized debug panel, restyled camera chrome, 3-state night mode.

**Architecture:** Pure UI layer. Theme tokens carry most of the restyle (legacy `pal.teal/orange` fields re-pointed to disciplined colors so all consumers shift at once); the header/chip/sheet are rebuilt; everything else adopts tokens. No VIO/matcher/native logic changes; calibration actions call only existing functions.

**Tech Stack:** Jetpack Compose (Material3), existing `NavPalette` pattern, Google Maps Compose markers.

**Repo rules:** comment out replaced code with `LEGACY 2026-06-12ui` markers (never delete); NO git commits unless the owner says "commit"; build gate = `gradlew assembleDebug` + 74 unit tests green; final gate = on-device screenshots vs approved mocks (`.superpowers/brainstorm/1582-1781284535/content/header-final.html`).

---

### Task 1: Theme tokens — `NavSightTheme.kt` (rewrite, file is 55 lines)

**Files:** Modify: `app/src/main/java/com/example/navsight1/NavSightTheme.kt`

- [ ] **Step 1.1** Replace the color tokens + `NavPalette` with the disciplined set. Keep legacy field NAMES as aliases (every consumer keeps compiling, instantly restyled):

```kotlin
// ── Brand + semantic tokens (2026-06-12 redesign — one accent, three status colors) ──
val BrandPurple     = Color(0xFF6A4CF0)
val BrandPurpleAlt  = Color(0xFF7E5CFF)   // header gradient end
val StatusGood      = Color(0xFF2ED589)
val StatusBad       = Color(0xFFFF5252)
val StatusWarn      = Color(0xFFFFB300)
// dark palette
val DarkBg     = Color(0xFF15121F)
val DarkCard   = Color(0xFF1E1930)
val DarkBorder = Color(0xFF2C2545)
val TextWhite  = Color(0xFFECEAF4)
val TextGrey   = Color(0xFF9A93B5)
// light palette
val LightBg    = Color(0xFFF4F2FA)
val LightCard  = Color(0xFFFFFFFF)
val LightText  = Color(0xFF2D2356)
val SoftBorder = Color(0xFFE2DCF2)
// header gradient consts (already referenced at MapScreenUi:178,561)
val HeroPurple     = BrandPurple
val HeroPurpleDark = BrandPurpleAlt
```

- [ ] **Step 1.2** `NavPalette` gains `brand`, `onBrand`, `statusGood`, `statusBad`, `statusWarn`; legacy `teal/tealDark/orange/orangeAcc` stay as fields but map to brand/warn (comment `// LEGACY aliases — retired hues now map to the disciplined palette`). `buildNavPalette(night)` fills both palettes accordingly. Keep `isNightTime()`, `IncidentCardModel`, `White12/White30` (used by chips).

- [ ] **Step 1.3** Build: `./gradlew.bat assembleDebug` → BUILD SUCCESSFUL.

### Task 2: Slim header + VIO chip — `MapScreenUi.kt`

**Files:** Modify: `app/src/main/java/com/example/navsight1/MapScreenUi.kt` (header call site :187-207, `HeroHeader` :548-615, `VioStatusChip` :636-667, `SpeedLimitCore` use :586, radar :230-233)

- [ ] **Step 2.1** Rewrite `HeroHeader` as the approved slim bar (~48dp): N-circle left, centered speed, VIO chip right. New signature carries chip state + click:

```kotlin
@Composable
fun HeroHeader(speedKmh: Float, compassLabel: String, chip: VioChipState,
               onChipClick: () -> Unit, pal: NavPalette) {
    Box(Modifier.fillMaxWidth().clip(RoundedCornerShape(14.dp))
        .background(Brush.linearGradient(listOf(HeroPurple, HeroPurpleDark)))
        .padding(horizontal = 14.dp, vertical = 9.dp)) {
        Box(Modifier.size(30.dp).align(Alignment.CenterStart)
            .clip(CircleShape).background(Color.White.copy(0.18f)),
            contentAlignment = Alignment.Center) {
            Text(compassLabel, color = Color.White, fontSize = 12.sp, fontWeight = FontWeight.Bold)
        }
        Row(Modifier.align(Alignment.Center), verticalAlignment = Alignment.Bottom) {
            Text("%.0f".format(speedKmh), color = Color.White, fontSize = 26.sp,
                fontWeight = FontWeight.ExtraBold, lineHeight = 26.sp)
            Text(" km/h", color = Color.White.copy(0.8f), fontSize = 11.sp,
                fontWeight = FontWeight.SemiBold, modifier = Modifier.padding(bottom = 3.dp))
        }
        VioChip(chip, onChipClick, Modifier.align(Alignment.CenterEnd))
    }
}
```

- [ ] **Step 2.2** Add `VioChipState` enum + `VioChip` (green ✓ / red ⚠ pulsing / amber degraded / grey init), 10sp bold, tappable:

```kotlin
enum class VioChipState { GOOD, INIT, DEGRADED, BAD }
@Composable
fun VioChip(state: VioChipState, onClick: () -> Unit, modifier: Modifier = Modifier) {
    val (label, color) = when (state) {
        VioChipState.GOOD -> "VIO ✓" to StatusGood
        VioChipState.INIT -> "VIO …" to Color(0xFF9E9E9E)
        VioChipState.DEGRADED -> "VIO ~" to StatusWarn
        VioChipState.BAD -> "VIO ⚠" to StatusBad
    }
    val pulse = rememberInfiniteTransition(label = "vio").animateFloat(
        0.55f, 1f, infiniteRepeatable(tween(900), RepeatMode.Reverse), label = "vioA")
    val alpha = if (state == VioChipState.BAD) pulse.value else 1f
    Surface(onClick = onClick, modifier = modifier, shape = RoundedCornerShape(10.dp),
        color = color.copy(alpha = 0.25f * alpha),
        border = BorderStroke(1.dp, color.copy(alpha = alpha))) {
        Text(label, color = Color.White, fontSize = 10.sp, fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 4.dp))
    }
}
```

- [ ] **Step 2.3** Chip-state derivation at the call site (existing signals only): `!vio.isInitialized → BAD`; `!viewModel.calibrationLoaded → BAD`; `!covValid → INIT`; `sigmaM ≥ 1.5 → DEGRADED`; else `GOOD`. Click → `showCalibrationSheet = true`.

- [ ] **Step 2.4** Call-site cleanup (:187-207): header gets new args; LEGACY-comment the `VioStatusChip(...)` + `CalibrationStatusPill(...)` row (replaced by the chip; σ readout moves to debug panel Task 6); keep `CalibrationFirstLaunchBanner`. LEGACY-comment `SpeedLimitCore`/`SpeedLimitBadge` composables + uses (the fabricated limit). LEGACY-comment the in-header `IPM --` subtext (IPM *is* the speedometer now; raw value joins debug panel).

- [ ] **Step 2.5** Radar card (:230-233): wrap `SensorRadarWaze(...)` in `if (debugVisible)` — it's a dev visual; this removes the overlap for normal use.

- [ ] **Step 2.6** Build green.

### Task 3: Calibration sheet — new `CalibrationSheetUi.kt` + ViewModel state

**Files:** Create: `app/src/main/java/com/example/navsight1/CalibrationSheetUi.kt`; Modify: `NavSightViewModel.kt` (add `showCalibrationSheet by mutableStateOf(false)`, expose `mountHeightM`, `heightCalibStatus` already exists), `MapScreenUi.kt` (host the sheet)

- [ ] **Step 3.1** Sheet with the three approved rows (status caption + outlined action):

```kotlin
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun CalibrationSheet(viewModel: NavSightViewModel, pal: NavPalette,
                     compassAzimuthDeg: Float, onOpenCameraCalibration: () -> Unit,
                     onDismiss: () -> Unit) {
    var showHeightDialog by remember { mutableStateOf(false) }
    ModalBottomSheet(onDismissRequest = onDismiss, containerColor = pal.card) {
        Column(Modifier.padding(horizontal = 18.dp).padding(bottom = 18.dp)) {
            Text("Calibration", color = pal.textPrimary, fontSize = 15.sp, fontWeight = FontWeight.ExtraBold)
            CalibRow("Camera",
                if (viewModel.calibrationLoaded) "calibrated · OK" else "not calibrated",
                "Recalibrate", pal, enabled = true) { onOpenCameraCalibration(); onDismiss() }
            CalibRow("Mount height",
                "%.2f m · %s".format(viewModel.mountHeightM,
                    viewModel.heightCalibStatus ?: "auto-tuning from GPS"),
                "Set…", pal, enabled = true) { showHeightDialog = true }
            CalibRow("Heading",
                if (compassAzimuthDeg.isFinite()) "compass %.0f°".format(compassAzimuthDeg) else "no compass reading",
                "Re-snap", pal, enabled = compassAzimuthDeg.isFinite()) {
                NativeBridge.seedMadgwickYaw(Math.toRadians(compassAzimuthDeg.toDouble()))
                onDismiss()
            }
        }
    }
    if (showHeightDialog) MountHeightDialog(viewModel.mountHeightM, pal,
        onSet = { viewModel.setMountHeight(it); showHeightDialog = false },
        onCancel = { showHeightDialog = false })
}
```

`CalibRow` = title 13sp/700 + caption 10sp `textSecondary` + `OutlinedButton` in `pal.brand` (greyed when `!enabled`, caption shows the reason). `MountHeightDialog` = `AlertDialog` with a numeric `OutlinedTextField` (validates 0.3–2.5 m, matching `setMountHeight`'s clamp).

- [ ] **Step 3.2** Host in `MapScreenUi`: `if (viewModel.showCalibrationSheet) CalibrationSheet(... onOpenCameraCalibration = { calibrationVisible = true }, onDismiss = { viewModel.showCalibrationSheet = false })`. Compass azimuth from `viewModel.orientationState.azimuth`.

- [ ] **Step 3.3** Build green; sheet smoke-checked via screenshot in Task 9.

### Task 4: Position markers — `MapScreenUi.kt` :420-460

- [ ] **Step 4.1** Read the marker block; restyle: the user marker (the arrow at :457) becomes the single prominent brand marker (purple arrow icon); the Start marker (:458) becomes a small subtle outline pin (it is legitimate — start point — but must stop competing with the user marker). Any additional raw-VIO dot/trace rendering becomes `if (debugVisible)`.
- [ ] **Step 4.2** Build green.

### Task 5: FABs, footer reset-confirm, 3-state night

**Files:** Modify: `MapScreenUi.kt` (`MapActionStack` :692-703), `BottomSheetUi.kt` (footer :78-105), `MainActivity.kt` (:35-52)

- [ ] **Step 5.1** `MapActionStack`: camera (white) + tune (dark `DarkCard`) only; LEGACY-comment the middle calibrate FAB (calibration now lives on the chip). Unify size 44dp.
- [ ] **Step 5.2** Footer Reset → confirm flow in `BottomSheetUi.kt`:

```kotlin
var confirmReset by remember { mutableStateOf(false) }
LaunchedEffect(confirmReset) { if (confirmReset) { delay(3000); confirmReset = false } }
FooterAction(
    icon  = if (confirmReset) Icons.Default.Warning else Icons.Default.Refresh,
    label = if (confirmReset) "Confirm reset" else "Reset",
    tint  = if (confirmReset) StatusBad else pal.brand
) { if (confirmReset) { onResetClick(); confirmReset = false } else confirmReset = true }
```

- [ ] **Step 5.3** `MainActivity` night: 3-state pref `night_mode` ∈ {auto, day, night} (SharedPreferences). Auto = `isNightTime()` refreshed every 60 s (existing loop, now gated on mode==auto); toggle cycles auto→day→night→auto and persists; pass the resolved `isNight` down unchanged.
- [ ] **Step 5.4** Build green.

### Task 6: Debug panel reorganization — `DebugPanelUi.kt`

- [ ] **Step 6.1** Read the file; regroup content under `.label`-style 9sp headers: **SPEED** (displayed, IPM raw `groundFlowSpeedKmh`, fused legacy, bridge state), **MATCHER** (conf, src, rail way, maneuver), **VIO** (σ `positionSigmaM`, K, h `mountHeightM`, gait/fusion mode, track %), **COUNTERS** (existing block + the Reset-counters button it already has). Panel surface `DarkBg.copy(alpha=.92f)`, 10sp mono-ish rows. All previously-shown values preserved (σ, mode dot info, track % land here from the old header/chips).
- [ ] **Step 6.2** Build green.

### Task 7: Camera screen chrome — `CameraUi.kt`

- [ ] **Step 7.1** Carry the slim `HeroHeader` over the camera view (same args; replaces any duplicated speed text), keep `onClose` X top-right restyled (`DarkCard` circle, white icon).
- [ ] **Step 7.2** One amber banner slot under the header: a single composable that shows the highest-priority current message (existing guidance strings unchanged — incl. the Hebrew ones and "LOW QUALITY") with `StatusWarn` border style; LEGACY-comment the scattered toast/pill renderings it replaces.
- [ ] **Step 7.3** The raw debug text line (`rot= KLT= ...`) moves behind `debugVisible` (it already feeds DebugPanel on this screen — keep only the panel). Overlay graphics (mesh, KLT dots, IPM candidates) untouched.
- [ ] **Step 7.4** Build green.

### Task 8: Token adoption — `BottomSheetUi.kt`, `NavInstructionBannerUi.kt`

- [ ] **Step 8.1** Sweep both files for raw `Teal*/Orange*` references → `pal.brand`/`pal.statusWarn` equivalents (mostly automatic via Task 1 aliases; fix any hardcoded `Color(0xFF26C6DA)`-style literals).
- [ ] **Step 8.2** Build green + run unit tests: `./gradlew.bat assembleDebug testDebugUnitTest` → BUILD SUCCESSFUL, 74 tests pass.

### Task 9: On-device visual validation

- [ ] **Step 9.1** Install: `adb install -r app/build/outputs/apk/debug/app-debug.apk`.
- [ ] **Step 9.2** Screenshot ride screen (day), toggle night → screenshot, open camera screen → screenshot, tap VIO chip → calibration sheet screenshot, tune FAB → debug panel screenshot (`adb shell screencap` + pull).
- [ ] **Step 9.3** Compare against approved mocks; fix deviations; show the owner the screenshots for acceptance. (No commit — owner commits explicitly.)
