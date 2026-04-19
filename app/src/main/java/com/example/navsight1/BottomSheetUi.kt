package com.example.navsight1

import androidx.compose.animation.*
import androidx.compose.animation.core.*
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
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
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun BottomSheet(
    modifier: Modifier,
    pal: NavPalette,
    navState: NavigationState,
    isNight: Boolean,
    cameraVisible: Boolean,
    isRecordingGpx: Boolean,
    debugVisible: Boolean,
    speedKmh: Float,
    totalM: Double,
    compassLabel: String,
    isMoving: Boolean,
    fusionMode: String,
    vioInitialized: Boolean,
    incidents: List<IncidentCardModel>,
    expanded: Boolean,
    onCameraClick: () -> Unit,
    onGpxClick: () -> Unit,
    onDebugClick: () -> Unit,
    onResetClick: () -> Unit,
    onStopNavClick: () -> Unit,
    onNightToggle: () -> Unit,
    onToggleExpanded: () -> Unit
) {
    Column(modifier.fillMaxWidth().navigationBarsPadding()) {

        // ETA banner — slides up when navigating
        AnimatedVisibility(
            navState is NavigationState.Active,
            enter = slideInVertically { it } + fadeIn(tween(220)),
            exit  = slideOutVertically { it } + fadeOut(tween(220))
        ) {
            if (navState is NavigationState.Active) {
                Surface(color = HeroPurple, shape = RoundedCornerShape(topStart = 28.dp, topEnd = 28.dp)) {
                    Row(
                        Modifier.fillMaxWidth().padding(horizontal = 22.dp, vertical = 14.dp),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment     = Alignment.CenterVertically
                    ) {
                        Column {
                            Text(NavSightUtils.formatTime(navState.remainingTimeSeconds),
                                color = Color.White, fontSize = 30.sp,
                                fontWeight = FontWeight.ExtraBold, lineHeight = 32.sp)
                            Text("ETA", color = Color.White.copy(0.75f), fontSize = 10.sp, letterSpacing = 1.5.sp)
                        }
                        Column(horizontalAlignment = Alignment.CenterHorizontally) {
                            Text(NavSightUtils.formatDistance(navState.remainingDistanceMeters.toInt()),
                                color = Color.White, fontSize = 22.sp, fontWeight = FontWeight.Bold)
                            Text("remaining", color = Color.White.copy(0.75f), fontSize = 10.sp)
                        }
                        Surface(onClick = onStopNavClick, color = Color.White.copy(0.20f), shape = RoundedCornerShape(14.dp)) {
                            Row(Modifier.padding(horizontal = 14.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
                                Icon(Icons.Default.Close, null, tint = Color.White, modifier = Modifier.size(16.dp))
                                Spacer(Modifier.width(5.dp))
                                Text("End", color = Color.White, fontSize = 13.sp, fontWeight = FontWeight.Bold)
                            }
                        }
                    }
                }
            }
        }

        // Expandable panel
        AnimatedVisibility(
            visible = expanded,
            enter = slideInVertically(tween(90)) { it / 3 } + fadeIn(tween(90)),
            exit  = slideOutVertically(tween(70)) { it / 3 } + fadeOut(tween(70))
        ) {
                Surface(
                    modifier        = Modifier.fillMaxWidth().padding(horizontal = 10.dp, vertical = 6.dp),
                    color           = Color.White,
                    shape           = RoundedCornerShape(30.dp),
                    shadowElevation = 20.dp
                ) {
                    Column(Modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 12.dp)) {
                        SheetHandleRow(expanded, speedKmh, compassLabel, onToggleExpanded)
                        Spacer(Modifier.height(10.dp))
                        Text("Nearest incidents", color = LightText, fontSize = 18.sp, fontWeight = FontWeight.Bold)
                        Spacer(Modifier.height(12.dp))
                        incidents.forEachIndexed { idx, incident ->
                            IncidentCard(incident)
                            if (idx != incidents.lastIndex) Spacer(Modifier.height(10.dp))
                        }
                        Spacer(Modifier.height(12.dp))
                        Surface(color = SoftSurface, shape = RoundedCornerShape(20.dp), border = BorderStroke(1.dp, SoftBorder)) {
                            Column(Modifier.padding(horizontal = 12.dp, vertical = 10.dp)) {
                                Row(Modifier.fillMaxWidth(), Arrangement.SpaceBetween, Alignment.CenterVertically) {
                                    StatPill("${"%.0f".format(speedKmh)}", "km/h", HeroPurple)
                                    StatPill("${"%.0f".format(totalM)}", "meters", pal.orange)
                                    StatPill(compassLabel, "heading", Color(0xFF4B7BEC))
                                    StatPill(
                                        if (vioInitialized) fusionMode else "INIT",
                                        if (isMoving) "tracking" else "standby",
                                        if (vioInitialized) pal.teal else Color(0xFF9E9E9E)
                                    )
                                }
                                Spacer(Modifier.height(8.dp))
                                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                                    TabBtn(
                                        icon = Icons.Default.PhotoCamera,
                                        label = if (cameraVisible) "Map" else "Camera",
                                        tint = Color.White, bgColor = HeroPurple, isPrimary = true,
                                        modifier = Modifier.weight(1f), onClick = onCameraClick
                                    )
                                    TabBtn(
                                        icon  = if (isNight) Icons.Default.LightMode else Icons.Default.DarkMode,
                                        label = if (isNight) "Day" else "Night",
                                        tint  = if (isNight) Orange400 else pal.textSecondary,
                                        modifier = Modifier.weight(1f), onClick = onNightToggle
                                    )
                                    TabBtn(
                                        icon = Icons.Default.Tune, label = "Debug",
                                        tint = if (debugVisible) pal.teal else pal.textSecondary,
                                        isActive = debugVisible, modifier = Modifier.weight(1f), onClick = onDebugClick
                                    )
                                    if (navState is NavigationState.Active) {
                                        TabBtn(icon = Icons.Default.Close, label = "End",
                                            tint = Color.White, bgColor = Color(0xFFEF5350),
                                            modifier = Modifier.weight(1f), onClick = onStopNavClick)
                                    } else {
                                        TabBtn(
                                            icon  = if (vioInitialized) Icons.Default.Sensors else Icons.Default.HourglassEmpty,
                                            label = if (vioInitialized) "Live" else "Waiting",
                                            tint  = if (vioInitialized) pal.teal else pal.textSecondary,
                                            modifier = Modifier.weight(1f), onClick = onDebugClick
                                        )
                                    }
                                }
                            }
                        }
                    }
                }
        }
        // Bottom nav bar
        Surface(color = Color.White, shadowElevation = 14.dp) {
            Row(
                Modifier.fillMaxWidth().padding(horizontal = 6.dp, vertical = 6.dp),
                horizontalArrangement = Arrangement.SpaceAround,
                verticalAlignment     = Alignment.CenterVertically
            ) {
                BottomNavItem(Icons.Default.Map, "Map", !expanded) {
                    if (expanded) onToggleExpanded()
                    if (cameraVisible) onCameraClick()
                }
                BottomNavItem(Icons.Default.Warning, "Warnings", expanded, onToggleExpanded)
                BottomNavItem(Icons.Default.Refresh, "Rides", false, onResetClick)
                BottomNavItem(Icons.Default.FiberManualRecord, "Records", isRecordingGpx, onGpxClick)
            }
        }
    }
}

@Composable
fun SheetHandleRow(expanded: Boolean, speedKmh: Float, compassLabel: String, onToggleExpanded: () -> Unit) {
    Row(
        Modifier.fillMaxWidth().clickable { onToggleExpanded() },
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment     = Alignment.CenterVertically
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Box(Modifier.width(46.dp).height(3.dp).clip(RoundedCornerShape(3.dp)).background(Color(0xFFD7D7DE)))
            Spacer(Modifier.width(8.dp))
            Text(if (expanded) "Hide panel" else "Show panel",
                color = Color(0xFF7E7E8B), fontSize = 10.sp, fontWeight = FontWeight.Medium)
        }
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("${"%.0f".format(speedKmh)} km/h • $compassLabel",
                color = LightText, fontSize = 10.sp, fontWeight = FontWeight.SemiBold)
            Surface(color = SoftSurface, shape = CircleShape) {
                Icon(
                    if (expanded) Icons.Default.KeyboardArrowDown else Icons.Default.KeyboardArrowUp,
                    null, tint = LightText, modifier = Modifier.padding(3.dp).size(16.dp)
                )
            }
        }
    }
}

@Composable
fun BottomNavItem(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    label: String,
    selected: Boolean,
    onClick: () -> Unit
) {
    Column(
        modifier = Modifier.clip(RoundedCornerShape(16.dp)).clickable { onClick() }
            .padding(horizontal = 5.dp, vertical = 2.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Icon(icon, label, tint = if (selected) Color.Black else Color(0xFF9E9EAC), modifier = Modifier.size(18.dp))
        Spacer(Modifier.height(2.dp))
        Text(label, color = if (selected) Color.Black else Color(0xFF9E9EAC), fontSize = 8.sp,
            fontWeight = if (selected) FontWeight.Bold else FontWeight.Medium)
    }
}

@Composable
private fun RecordingDot() {
    val inf = rememberInfiniteTransition(label = "rec")
    val a by inf.animateFloat(0.35f, 1f, infiniteRepeatable(tween(500), RepeatMode.Reverse), label = "ra")
    Box(Modifier.padding(top = 3.dp, end = 3.dp).size(7.dp).clip(CircleShape).background(Color(0xFFEF5350).copy(a)))
}

@Composable
fun TabBtn(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    label: String,
    tint: Color,
    bgColor: Color? = null,
    isPrimary: Boolean = false,
    isRecording: Boolean = false,
    isActive: Boolean = false,
    modifier: Modifier = Modifier,
    onClick: () -> Unit
) {
    Box(contentAlignment = Alignment.TopEnd) {
        Column(
            modifier
                .clip(RoundedCornerShape(if (isPrimary) 20.dp else 14.dp))
                .then(when {
                    bgColor != null -> Modifier.background(bgColor)
                    isActive        -> Modifier.background(tint.copy(0.10f))
                    else            -> Modifier.background(Color.White)
                })
                .border(
                    width = if (bgColor != null || isActive) 0.dp else 1.dp,
                    color = SoftBorder,
                    shape = RoundedCornerShape(if (isPrimary) 20.dp else 14.dp)
                )
                .clickable { onClick() }
                .padding(horizontal = if (isPrimary) 10.dp else 8.dp, vertical = if (isPrimary) 8.dp else 6.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Icon(icon, null,
                tint     = if (bgColor != null) Color.White else tint,
                modifier = Modifier.size(if (isPrimary) 18.dp else 16.dp))
            Spacer(Modifier.height(2.dp))
            Text(label, color = if (bgColor != null) Color.White else tint,
                fontSize = 7.sp, fontWeight = if (isPrimary) FontWeight.Bold else FontWeight.Medium)
        }
        if (isRecording) RecordingDot()
    }
}

@Composable
fun IncidentCard(incident: IncidentCardModel) {
    Surface(color = SoftSurface, shape = RoundedCornerShape(20.dp), border = BorderStroke(1.dp, SoftBorder)) {
        Row(Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 10.dp), verticalAlignment = Alignment.CenterVertically) {
            Box(Modifier.size(46.dp).clip(CircleShape).background(incident.color), contentAlignment = Alignment.Center) {
                Icon(incident.icon, null, tint = Color.White, modifier = Modifier.size(22.dp))
            }
            Spacer(Modifier.width(10.dp))
            Column(Modifier.weight(1f)) {
                Text(incident.title, color = LightText, fontSize = 14.sp, fontWeight = FontWeight.Bold)
                Text(incident.subtitle, color = Color(0xFF9E9EAC), fontSize = 11.sp)
            }
            Text(incident.eta, color = LightText, fontSize = 14.sp, fontWeight = FontWeight.ExtraBold)
        }
    }
}

@Composable
fun StatPill(title: String, subtitle: String, accent: Color) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(title, color = accent, fontSize = 16.sp, fontWeight = FontWeight.ExtraBold)
        Text(subtitle, color = Color(0xFF9E9EAC), fontSize = 9.sp)
    }
}

fun buildIncidentCards(
    navState: NavigationState,
    isMoving: Boolean,
    speedKmh: Float,
    totalDistanceM: Double,
    trackingQuality: Float,
    vioInitialized: Boolean
): List<IncidentCardModel> {
    val primary = when (navState) {
        is NavigationState.Active -> IncidentCardModel(
            title    = "Navigation active",
            subtitle = "${NavSightUtils.formatDistance(navState.remainingDistanceMeters.toInt())} remaining",
            eta      = NavSightUtils.formatTime(navState.remainingTimeSeconds),
            color    = Color(0xFFFF6A5E),
            icon     = Icons.Default.Directions
        )
        else -> IncidentCardModel(
            title    = if (isMoving) "On the move" else "Vehicle standing",
            subtitle = "${"%.0f".format(speedKmh)} km/h • ${"%.0f".format(totalDistanceM)} m tracked",
            eta      = if (isMoving) "Live" else "Idle",
            color    = Color(0xFF53D34D),
            icon     = Icons.Default.DirectionsCar
        )
    }
    val qualityEta = when {
        trackingQuality >= 0.75f -> "High"
        trackingQuality >= 0.4f  -> "Medium"
        else                     -> "Low"
    }
    return listOf(
        primary,
        IncidentCardModel("Tracking quality",
            if (vioInitialized) "Visual + IMU fusion" else "Waiting for VIO initialization",
            qualityEta, Color(0xFF4B7BEC), Icons.Default.Security),
        IncidentCardModel("Debug tools",
            "Camera, GPX and live diagnostics are ready",
            "Tools", HeroPurple, Icons.Default.Tune)
    )
}
