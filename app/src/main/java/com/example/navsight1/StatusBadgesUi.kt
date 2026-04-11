package com.example.navsight1

import androidx.compose.animation.core.*
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun DirectionBadge(isMoving: Boolean, trackingQuality: Float, pal: NavPalette) {
    val lowQ = trackingQuality < 0.3f && trackingQuality > 0f
    val (text, color, icon) = when {
        lowQ     -> Triple("LOW QUALITY", Color(0xFFEF5350), Icons.Default.Place)
        isMoving -> Triple("בתנועה", pal.teal, Icons.Default.KeyboardArrowUp)
        else     -> Triple("עומד", pal.orange, Icons.Default.Place)
    }
    Surface(
        color  = Color.Black.copy(0.58f),
        shape  = RoundedCornerShape(14.dp),
        border = BorderStroke(1.dp, color.copy(0.6f))
    ) {
        Row(Modifier.padding(horizontal = 12.dp, vertical = 8.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(icon, null, tint = color, modifier = Modifier.size(18.dp))
            Spacer(Modifier.width(6.dp))
            Text(text, color = color, fontWeight = FontWeight.Bold, fontSize = 13.sp)
        }
    }
}

@Composable
fun PhoneOrientationWarning(deviation: Float, pal: NavPalette) {
    val inf = rememberInfiniteTransition(label = "blink")
    val a by inf.animateFloat(0.5f, 1f, infiniteRepeatable(tween(400), RepeatMode.Reverse), label = "a")
    Surface(
        color  = Color(0xFFEF5350).copy(0.2f),
        shape  = RoundedCornerShape(14.dp),
        border = BorderStroke(1.5.dp, Color(0xFFEF5350).copy(a))
    ) {
        Row(Modifier.padding(horizontal = 14.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(Icons.Default.Phone, null, tint = Color(0xFFEF5350),
                modifier = Modifier.size(22.dp).rotate(deviation.coerceIn(-30f, 30f)))
            Spacer(Modifier.width(8.dp))
            Column {
                Text("הטה את הטלפון קדימה", color = Color(0xFFEF5350), fontWeight = FontWeight.Bold, fontSize = 13.sp)
                Text("סטייה: ${"%.0f".format(deviation)}°", color = Color(0xFFEF5350).copy(0.75f), fontSize = 11.sp)
            }
        }
    }
}

@Composable
fun VioInitializingBadge(pal: NavPalette) {
    val inf = rememberInfiniteTransition(label = "init")
    val a by inf.animateFloat(0.4f, 1f, infiniteRepeatable(tween(600), RepeatMode.Reverse), label = "ia")
    Surface(
        color  = Color.Black.copy(0.65f),
        shape  = RoundedCornerShape(14.dp),
        border = BorderStroke(1.dp, pal.orange.copy(a))
    ) {
        Row(Modifier.padding(horizontal = 14.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(Icons.Default.Refresh, null, tint = pal.orange, modifier = Modifier.size(18.dp))
            Spacer(Modifier.width(8.dp))
            Text("מכייל…", color = pal.orange, fontWeight = FontWeight.Bold, fontSize = 13.sp)
        }
    }
}

@Composable
fun CameraBlockedWarning(pal: NavPalette) {
    val inf = rememberInfiniteTransition(label = "blk")
    val a by inf.animateFloat(0.5f, 1f, infiniteRepeatable(tween(400), RepeatMode.Reverse), label = "ba")
    Surface(
        color  = Color(0xFFEF5350).copy(0.2f),
        shape  = RoundedCornerShape(14.dp),
        border = BorderStroke(1.5.dp, Color(0xFFEF5350).copy(a))
    ) {
        Row(Modifier.padding(horizontal = 14.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
            Icon(Icons.Default.Place, null, tint = Color(0xFFEF5350), modifier = Modifier.size(20.dp))
            Spacer(Modifier.width(8.dp))
            Text("מצלמה חסומה", color = Color(0xFFEF5350), fontWeight = FontWeight.Bold, fontSize = 13.sp)
        }
    }
}

@Composable
fun NoTextureWarning(pal: NavPalette) {
    Surface(
        color  = Color.Black.copy(0.6f),
        shape  = RoundedCornerShape(10.dp),
        border = BorderStroke(1.dp, pal.orange.copy(0.7f))
    ) {
        Text(
            "אין מרקם — עבור לאזור מרוצף", color = pal.orange, fontSize = 11.sp,
            modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp)
        )
    }
}

@Composable
fun StabilityIndicator(stability: Float, confidence: Float, pal: NavPalette) {
    Surface(color = Color.Black.copy(0.55f), shape = RoundedCornerShape(10.dp),
        border = BorderStroke(1.dp, White12)) {
        Column(Modifier.padding(8.dp), horizontalAlignment = Alignment.CenterHorizontally) {
            Box(Modifier.width(36.dp).height(4.dp).clip(RoundedCornerShape(2.dp)).background(White12)) {
                Box(Modifier.fillMaxHeight().fillMaxWidth(stability).background(
                    when { stability > 0.7f -> pal.teal; stability > 0.4f -> pal.orange; else -> Color(0xFFEF5350) }
                ))
            }
            Spacer(Modifier.height(3.dp))
            Text("${"%.0f".format(confidence * 100)}%", color = TextGrey, fontSize = 10.sp)
        }
    }
}

@Composable
fun RecordingPill(pal: NavPalette) {
    val inf = rememberInfiniteTransition(label = "pill")
    val a by inf.animateFloat(0.4f, 1f, infiniteRepeatable(tween(500), RepeatMode.Reverse), label = "pa")
    Surface(color = pal.orange.copy(0.9f), shape = RoundedCornerShape(20.dp)) {
        Row(Modifier.padding(horizontal = 14.dp, vertical = 6.dp), verticalAlignment = Alignment.CenterVertically) {
            Box(Modifier.size(7.dp).clip(CircleShape).background(Color.White.copy(a)))
            Spacer(Modifier.width(7.dp))
            Text("REC GPX", color = Color.White, fontSize = 11.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
        }
    }
}

@Composable
fun WazeToast(text: String, pal: NavPalette) {
    Surface(
        color  = pal.card.copy(0.95f),
        shape  = RoundedCornerShape(22.dp),
        border = BorderStroke(1.dp, pal.teal.copy(0.4f))
    ) {
        Text(
            text, color = pal.textPrimary, fontSize = 12.sp,
            modifier = Modifier.padding(horizontal = 18.dp, vertical = 9.dp)
        )
    }
}

@Composable
fun WazeChip(pal: NavPalette, content: @Composable RowScope.() -> Unit) {
    Surface(
        color           = pal.card,
        shape           = RoundedCornerShape(16.dp),
        border          = BorderStroke(1.dp, pal.cardBorder),
        shadowElevation = 4.dp
    ) {
        Row(
            Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            content = content
        )
    }
}

@Composable
fun ErrorCard(message: String, pal: NavPalette, onDismiss: () -> Unit) {
    Surface(onClick = onDismiss, color = LightCard, shape = RoundedCornerShape(14.dp), shadowElevation = 6.dp) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 10.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Box(
                Modifier.size(30.dp).clip(CircleShape).background(Orange400.copy(0.12f)),
                contentAlignment = Alignment.Center
            ) {
                Icon(Icons.Default.Warning, null, tint = Orange400, modifier = Modifier.size(16.dp))
            }
            Spacer(Modifier.width(10.dp))
            Text(message, color = LightText, fontSize = 13.sp, modifier = Modifier.weight(1f))
            Icon(Icons.Default.Close, null, tint = Color(0xFFB0BEC5), modifier = Modifier.size(16.dp))
        }
    }
}
