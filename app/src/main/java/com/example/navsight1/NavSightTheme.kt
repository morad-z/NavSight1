package com.example.navsight1

import androidx.compose.ui.graphics.Color
import java.time.LocalTime

// ── Night detection ───────────────────────────────────────────────────────────
fun isNightTime(): Boolean { val h = LocalTime.now().hour; return h < 6 || h >= 20 }

// ── Color tokens ─────────────────────────────────────────────────────────────
val Teal400    = Color(0xFF26C6DA)
val Teal500    = Color(0xFF00BCD4)
val Teal700    = Color(0xFF0097A7)
val Teal900    = Color(0xFF006064)
val Orange400  = Color(0xFFFF9800)
val Orange600  = Color(0xFFF57C00)
val OrangeAcc  = Color(0xFFFF6D00)
val DarkBg     = Color(0xFF0E1621)
val DarkCard   = Color(0xFF162130)
val DarkBorder = Color(0xFF1E3045)
val TextWhite  = Color(0xFFECF0F1)
val TextGrey   = Color(0xFF90A4AE)
val White12    = Color(0x1FFFFFFF)
val White30    = Color(0x4DFFFFFF)
val LightBg    = Color(0xFFF0F7FA)
val LightCard  = Color(0xFFFFFFFF)
val LightText  = Color(0xFF1A2B3C)
val HeroPurple     = Color(0xFF5847B8)
val HeroPurpleDark = Color(0xFF4B3DA7)
val SoftSurface    = Color(0xFFF7F7FB)
val SoftBorder     = Color(0xFFE8E8F0)

// ── Data models ───────────────────────────────────────────────────────────────
data class NavPalette(
    val bg: Color, val card: Color, val cardBorder: Color,
    val teal: Color, val tealDark: Color,
    val orange: Color, val orangeAcc: Color,
    val textPrimary: Color, val textSecondary: Color,
    val isNight: Boolean
)

data class IncidentCardModel(
    val title: String,
    val subtitle: String,
    val eta: String,
    val color: Color,
    val icon: androidx.compose.ui.graphics.vector.ImageVector
)

fun buildNavPalette(night: Boolean) = if (night) NavPalette(
    DarkBg, DarkCard, DarkBorder, Teal500, Teal700, Orange400, OrangeAcc,
    TextWhite, TextGrey, true
) else NavPalette(
    LightBg, LightCard, Color(0xFFB2EBF2), Teal700, Teal900, Orange600, OrangeAcc,
    LightText, Color(0xFF546E7A), false
)
