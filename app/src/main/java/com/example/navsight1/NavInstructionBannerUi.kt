package com.example.navsight1

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.BorderStroke
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun NavigationInstructionBanner(
    instruction: NavigationInstruction,
    remainingDistanceMeters: Double,
    remainingTimeSeconds: Int,
    pal: NavPalette,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier        = modifier,
        color           = pal.card,
        shape           = RoundedCornerShape(20.dp),
        border          = BorderStroke(1.5.dp, pal.teal.copy(0.45f)),
        shadowElevation = 12.dp
    ) {
        Row(
            Modifier.padding(14.dp).fillMaxWidth(),
            verticalAlignment         = Alignment.CenterVertically,
            horizontalArrangement     = Arrangement.SpaceBetween
        ) {
            Box(
                Modifier.size(52.dp).clip(CircleShape).background(
                    Brush.radialGradient(listOf(pal.teal.copy(0.25f), pal.teal.copy(0.05f)))
                ),
                contentAlignment = Alignment.Center
            ) {
                Icon(instruction.getManeuverIcon(), null, tint = pal.teal, modifier = Modifier.size(30.dp))
            }
            Spacer(Modifier.width(12.dp))
            Column(Modifier.weight(1f)) {
                Text(
                    instruction.streetName ?: "Continue", color = pal.textPrimary,
                    fontWeight = FontWeight.Bold, fontSize = 17.sp,
                    maxLines = 1, overflow = TextOverflow.Ellipsis
                )
                Text(
                    "in ${NavSightUtils.formatDistance(instruction.distanceMeters)}",
                    color = pal.textSecondary, fontSize = 13.sp
                )
            }
            Column(horizontalAlignment = Alignment.End) {
                Text(
                    NavSightUtils.formatTime(remainingTimeSeconds),
                    color = pal.orange, fontWeight = FontWeight.ExtraBold, fontSize = 20.sp
                )
                Text(
                    NavSightUtils.formatDistance(remainingDistanceMeters.toInt()),
                    color = pal.textSecondary, fontSize = 12.sp
                )
            }
        }
    }
}
