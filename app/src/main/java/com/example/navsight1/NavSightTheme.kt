package com.example.navsight1

import androidx.compose.ui.graphics.Color
import java.time.LocalTime

// ── Night detection ───────────────────────────────────────────────────────────
fun isNightTime(): Boolean { val h = LocalTime.now().hour; return h < 6 || h >= 20 }

// ── Color tokens (2026-06-12 redesign — ONE brand accent + three semantic colors) ──
// Spec: docs/superpowers/specs/2026-06-12-ui-redesign-design.md §1. The old palette ran
// 6+ accent hues (teal chips, orange toggles, decorative red, purple header); everything
// now derives from the brand purple + statusGood/Bad/Warn.
val BrandPurple    = Color(0xFF6A4CF0)
val BrandPurpleAlt = Color(0xFF7E5CFF)   // header-gradient end
val BrandPurpleDeep= Color(0xFF4B3DA7)   // pressed / dark-emphasis variant
val StatusGood     = Color(0xFF2ED589)   // calibrated / healthy
val StatusBad      = Color(0xFFFF5252)   // needs attention
val StatusWarn     = Color(0xFFFFB300)   // transient warnings (camera guidance, low quality)

// dark palette (near-black with a purple cast)
val DarkBg     = Color(0xFF15121F)
val DarkCard   = Color(0xFF1E1930)
val DarkBorder = Color(0xFF2C2545)
val TextWhite  = Color(0xFFECEAF4)
val TextGrey   = Color(0xFF9A93B5)
val White12    = Color(0x1FFFFFFF)
val White30    = Color(0x4DFFFFFF)

// light palette
val LightBg    = Color(0xFFF4F2FA)
val LightCard  = Color(0xFFFFFFFF)
val LightText  = Color(0xFF2D2356)
val SoftSurface    = Color(0xFFF7F6FB)
val SoftBorder     = Color(0xFFE2DCF2)

// Header gradient constants (referenced across MapScreenUi/BottomSheetUi).
val HeroPurple     = BrandPurple
val HeroPurpleDark = BrandPurpleAlt

// LEGACY aliases (2026-06-12) — the teal/orange hues are RETIRED; these names remain so
// every existing consumer compiles and is instantly restyled to the disciplined palette.
// New code should use Brand*/Status* directly.
val Teal400    = BrandPurpleAlt
val Teal500    = BrandPurple
val Teal700    = BrandPurple
val Teal900    = BrandPurpleDeep
val Orange400  = StatusWarn
val Orange600  = StatusWarn
val OrangeAcc  = StatusWarn

// ── Data models ───────────────────────────────────────────────────────────────
data class NavPalette(
    val bg: Color, val card: Color, val cardBorder: Color,
    // LEGACY field names kept for source compatibility — now mapped to brand/warn.
    val teal: Color, val tealDark: Color,
    val orange: Color, val orangeAcc: Color,
    val textPrimary: Color, val textSecondary: Color,
    val isNight: Boolean,
    // 2026-06-12 redesign tokens
    val brand: Color = BrandPurple,
    val onBrand: Color = Color.White,
    val statusGood: Color = StatusGood,
    val statusBad: Color = StatusBad,
    val statusWarn: Color = StatusWarn,
)

data class IncidentCardModel(
    val title: String,
    val subtitle: String,
    val eta: String,
    val color: Color,
    val icon: androidx.compose.ui.graphics.vector.ImageVector
)

fun buildNavPalette(night: Boolean) = if (night) NavPalette(
    DarkBg, DarkCard, DarkBorder, BrandPurpleAlt, BrandPurple, StatusWarn, StatusWarn,
    TextWhite, TextGrey, true
) else NavPalette(
    LightBg, LightCard, SoftBorder, BrandPurple, BrandPurpleDeep, StatusWarn, StatusWarn,
    LightText, Color(0xFF6F6792), false
)
