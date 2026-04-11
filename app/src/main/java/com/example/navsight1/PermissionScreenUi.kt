package com.example.navsight1

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Place
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun PermissionScreen(pal: NavPalette, onRequest: () -> Unit) {
    Box(Modifier.fillMaxSize().background(pal.bg), contentAlignment = Alignment.Center) {
        Surface(
            modifier        = Modifier.padding(32.dp),
            color           = pal.card,
            shape           = RoundedCornerShape(24.dp),
            border          = BorderStroke(1.dp, pal.cardBorder),
            shadowElevation = 10.dp
        ) {
            Column(Modifier.padding(32.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                Box(
                    Modifier.size(76.dp).clip(CircleShape).background(pal.teal.copy(0.12f)),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(Icons.Default.Place, null, tint = pal.teal, modifier = Modifier.size(42.dp))
                }
                Spacer(Modifier.height(22.dp))
                Text("Camera & Location\nRequired", color = pal.textPrimary,
                    fontSize = 20.sp, fontWeight = FontWeight.Bold, textAlign = TextAlign.Center)
                Spacer(Modifier.height(8.dp))
                Text("NavSight needs sensors to track your path",
                    color = pal.textSecondary, fontSize = 13.sp, textAlign = TextAlign.Center)
                Spacer(Modifier.height(28.dp))
                Button(
                    onClick = onRequest,
                    colors  = ButtonDefaults.buttonColors(containerColor = pal.teal),
                    shape   = RoundedCornerShape(16.dp),
                    modifier = Modifier.fillMaxWidth().height(52.dp)
                ) {
                    Text("Enable Sensors", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 15.sp)
                }
            }
        }
    }
}
