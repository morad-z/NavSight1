# NavSight UI/UX Redesign — Design Spec (2026-06-12)

Approved by owner in brainstorming session 2026-06-12 (visual companion mockups in
`.superpowers/brainstorm/1582-1781284535/content/`, selections: direction `refined-current`,
header `compact-bar` with centered speed, debug access `debug-fab`, header-final approved).

## Goal

Restyle the existing app — "Refined Current" direction: keep the purple identity, impose
discipline. Strictly the UI layer: **no navigation-flow changes, no VIO/matcher/heading logic
changes.** Calibration actions wire only to natively existing functions.

## 1. Theme — `NavSightTheme.kt`

- **Brand accent (one):** purple `#6A4CF0`, header gradient `#6A4CF0 → #7E5CFF`.
- **Semantic colors (only these besides brand):**
  - `statusGood` green `#2ED589` — calibrated / healthy
  - `statusBad` red `#FF5252` — needs attention (pulses gently where used as a chip)
  - `statusWarn` amber `#FFB300` — transient warnings (camera guidance, low quality)
- Retire: teal chips, orange day-toggle accent, decorative red. Day/night icon uses
  `textSecondary` tint, not orange.
- **Light + dark palettes** with identical token structure: `surface`, `surfaceCard`,
  `textPrimary`, `textSecondary`, `outline`, `brand`, `onBrand`, the three status colors.
  Dark surfaces near-black w/ purple cast (`#15121F` base, `#1E1930` cards).
- **Type scale:** speed numerals 26sp / 800 weight; section titles 15sp / 800; body 13sp;
  captions & chips 10–11sp. Single type family (system default).

## 2. Ride screen — `MapScreenUi.kt`

- **Slim header bar (~48dp), full width, brand gradient, rounded 14dp:**
  - Left: compass circle (30dp, translucent white bg) showing cardinal letter (existing
    `compassLabel`).
  - Center: speed `27 km/h` — 26sp bold numerals + 11sp unit, centered in the bar.
  - Right: **VIO status chip** — green `VIO ✓` when calibrated, red `VIO ⚠` (gentle pulse)
    when not. **Tap → Calibration sheet (§3).** Chip replaces the old
    "GPS-DENIED — VIO ACTIVE (σ = 0.1 m)" and "Calibrated" chips entirely; σ moves to the
    debug panel.
- **Remove the radar/compass overlay card** (it overlapped the search bar). Heading is
  conveyed by the header compass letter + the map marker's heading arrow.
- **Search bar:** white/`surfaceCard` rounded field directly under the header; unchanged
  behavior.
- **One position marker:** the rail ball (brand purple, white ring, heading arrow). The raw
  VIO dot renders only while the debug panel is open (it is a dev signal).
- **FAB stack (right edge):** camera and recenter as matching white circles (44dp, same icon
  style, theme outline); tune button keeps its place as the **debug toggle**, styled dark
  (`#1E1930`) to read as a dev control.
- **Footer (slim, `surfaceCard`):** left = distance tracked (13sp bold + 10sp caption);
  right = day/night toggle. **Reset requires a confirm:** first tap morphs the button into
  red "Confirm reset" for 3 s; second tap resets (prevents glove mis-taps).
- **Delete the fake speed-limit badge** (`SpeedLimitCore` rendered `max(30, ceil(speed/10)*10)`
  — a fabricated number). Per repo rules the composable is commented out, not deleted.

## 3. Calibration sheet — new small composable (`CalibrationSheetUi.kt`)

Modal bottom sheet opened from the VIO chip. Three rows, each: title + status caption +
outlined action button. Wire ONLY to existing entry points:

| Row | Status caption | Action | Existing function |
|---|---|---|---|
| Camera | last calibration state | Recalibrate | the existing camera-calibration flow trigger |
| Mount height | current `h` + auto-tune state (e.g. "1.05 m · auto-tuning from GPS") | Set… (numeric dialog) | `NavSightViewModel.setMountHeight` |
| Heading | compass snap state | Re-snap | `NativeBridge.seedMadgwickYaw` with current compass azimuth |

No new native logic. The sheet shows, it never auto-runs anything.

## 4. Debug panel — reorganized, content preserved

- Toggled by the tune FAB (kept). Dark panel (`#15121F` at 92% opacity) sliding from the
  bottom above the footer.
- Groups with `.label` headers: **Speed** (IPM, fused, bridge state), **Matcher** (conf,
  rail way id, src, maneuver state), **VIO** (σ, K, h, gait mode), **Counters** (the
  event-counter snapshot lines currently shown).
- The camera screen's raw text line (`rot=90 KLT=5 …`) moves into this panel (visible on
  both screens when toggled). Nothing currently displayed is deleted.

## 5. Camera screen — `CameraUi.kt`

- Same slim header carried over (continuity + glanceable speed while in camera view).
- **One banner slot** under the header for guidance/warnings (replaces stacked toasts):
  one message at a time, amber `statusWarn` styling, auto-dismiss on condition clear.
  Existing message strings unchanged (language untouched in this pass).
- "LOW QUALITY" pill restyled into the same banner slot.
- Overlay graphics (ground mesh, KLT dots, IPM candidates) keep their functional colors —
  they are diagnostics; only the close button and chrome adopt theme styling.
- Footer identical to the ride screen's.

## 6. Night mode

- Theme follows the system dark setting by default; the existing manual Day/Night footer
  toggle remains as an override (3-state: auto / day / night, persisted).
- Dark map style applied when dark (existing map night style hook); surfaces/chips use the
  dark palette.

## Component & file map

| Surface | File | Change type |
|---|---|---|
| Theme tokens | `NavSightTheme.kt` | rewrite (small file) |
| Header, marker, FABs, footer, debug panel host | `MapScreenUi.kt` | restructure sections |
| Calibration sheet | `CalibrationSheetUi.kt` | new |
| Camera chrome + banner | `CameraUi.kt` | restyle sections |
| Sheet handle/footer rows | `BottomSheetUi.kt` | token adoption only |
| Instruction banner | `NavInstructionBannerUi.kt` | token adoption only |
| State for chip/sheet | `NavSightViewModel.kt` | expose calibrated-state + sheet visibility |

## Error handling

- Calibration sheet actions disabled (greyed) when their precondition is absent (e.g.
  heading re-snap disabled until a compass reading exists; shows the reason as caption).
- VIO chip state derives from existing calibrated/height state — no new polling.

## Testing

- Build + existing 74 Kotlin unit tests must stay green (no logic-layer edits expected).
- Visual validation on device: screenshots of ride screen (day + night), camera screen,
  calibration sheet, debug panel open — compared against the approved mocks.
- Functional checks: chip tap opens sheet; mount-height set round-trips to native; reset
  confirm flow; debug toggle shows raw dot + panels on both screens.

## Non-goals

Navigation flow, bottom-sheet features, VIO/matcher/heading logic, string language
unification, exposure cap — all unchanged in this pass.
