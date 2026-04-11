package com.example.navsight1

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

@Composable
fun DebugPanel(
    totalDistanceM: Double,
    speedMs: Float,
    qualityPct: Float,
    fusionMode: String,
    scaleFactor: Double,
    headingDeg: Float,
    pal: NavPalette,
    viewModel: NavSightViewModel
) {
    var calibExpanded by remember { mutableStateOf(false) }
    val session  = viewModel.scaleCalibrationSession
    val calibMsg = viewModel.scaleCalibrationMessage
    val calibF   = viewModel.scaleCalibrationFactor
    val vio      = viewModel.vioState
    val context  = LocalContext.current

    Surface(
        color           = pal.card.copy(0.96f),
        shape           = RoundedCornerShape(14.dp),
        border          = BorderStroke(1.dp, pal.teal.copy(0.3f)),
        modifier        = Modifier.width(192.dp)
    ) {
        Column(Modifier.padding(horizontal = 10.dp, vertical = 8.dp), verticalArrangement = Arrangement.spacedBy(3.dp)) {
            Text("DEBUG", color = pal.teal, fontSize = 9.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.5.sp)
            HorizontalDivider(color = pal.teal.copy(0.2f), thickness = 0.5.dp)

            Button(
                onClick  = { viewModel.toggleSimulationRecording(
                    { type -> context.getExternalFilesDir(type) }, context.filesDir) },
                modifier = Modifier.fillMaxWidth().height(26.dp),
                contentPadding = PaddingValues(0.dp),
                colors   = ButtonDefaults.buttonColors(
                    containerColor = if (viewModel.isRecordingSimulation) Color(0xFFEF5350) else pal.teal.copy(0.2f)),
                shape    = RoundedCornerShape(6.dp)
            ) {
                Text(
                    if (viewModel.isRecordingSimulation) "■ STOP SIM REC" else "▶ START SIM REC",
                    fontSize = 8.sp, fontWeight = FontWeight.Bold,
                    color    = if (viewModel.isRecordingSimulation) Color.White else pal.teal
                )
            }

            HorizontalDivider(color = pal.teal.copy(0.1f), thickness = 0.5.dp)
            DebugRow("Dist",    "${"%.2f".format(totalDistanceM)} m", pal.teal, pal)
            DebugRow("Speed",   "${"%.2f".format(speedMs)} m/s",       pal.teal, pal)
            DebugRow("Quality", "${"%.0f".format(qualityPct)}%",
                when { qualityPct > 70f -> pal.teal; qualityPct > 30f -> pal.orange; else -> Color(0xFFEF5350) }, pal)
            DebugRow("Mode",    fusionMode,
                when (fusionMode) { "CAMERA" -> pal.teal; "HYBRID" -> pal.orange; else -> pal.textSecondary }, pal)
            DebugRow("Scale",   "${"%.4f".format(scaleFactor)}", Teal400, pal)
            DebugRow("Heading", "${"%.1f".format(headingDeg)}°", pal.textSecondary, pal)

            HorizontalDivider(color = pal.teal.copy(0.2f), thickness = 0.5.dp)
            Row(Modifier.fillMaxWidth(), Arrangement.SpaceBetween, Alignment.CenterVertically) {
                Text("Height", color = pal.textSecondary, fontSize = 10.sp)
                Row(verticalAlignment = Alignment.CenterVertically) {
                    SmallFloatingActionButton(
                        onClick        = { viewModel.updateUserHeight(viewModel.userHeight - 0.05f) },
                        containerColor = pal.cardBorder, modifier = Modifier.size(20.dp)
                    ) { Text("-", color = Color.White, fontSize = 10.sp) }
                    Text("${"%.2f".format(viewModel.userHeight)}m", color = pal.teal,
                        fontSize = 10.sp, fontWeight = FontWeight.Bold, modifier = Modifier.padding(horizontal = 4.dp))
                    SmallFloatingActionButton(
                        onClick        = { viewModel.updateUserHeight(viewModel.userHeight + 0.05f) },
                        containerColor = pal.cardBorder, modifier = Modifier.size(20.dp)
                    ) { Text("+", color = Color.White, fontSize = 10.sp) }
                }
            }

            HorizontalDivider(color = pal.teal.copy(0.2f), thickness = 0.5.dp)
            Surface(onClick = { calibExpanded = !calibExpanded }, color = Color.Transparent, modifier = Modifier.fillMaxWidth()) {
                Row(Modifier.fillMaxWidth().padding(vertical = 2.dp), Arrangement.SpaceBetween, Alignment.CenterVertically) {
                    Text("Calibration", color = pal.teal, fontSize = 10.sp, fontWeight = FontWeight.Bold)
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text("${"%.2f".format(calibF)}x", color = pal.orange, fontSize = 10.sp, fontWeight = FontWeight.Bold)
                        Icon(
                            if (calibExpanded) Icons.Default.KeyboardArrowUp else Icons.Default.KeyboardArrowDown,
                            null, tint = pal.textSecondary, modifier = Modifier.size(14.dp)
                        )
                    }
                }
            }

            if (calibExpanded) {
                if (session != null && vio.isInitialized) {
                    val closure = sqrt((vio.x - session.startX).pow(2) + (vio.z - session.startZ).pow(2))
                    val avgQ    = session.sumQuality / max(1, session.sampleCount)
                    Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
                        DebugRow("Path",  "${"%.1f".format(session.pathLengthMeters)} m",          Color.White,          pal)
                        DebugRow("Out",   "${"%.1f".format(session.maxDistanceFromStartMeters)} m", pal.teal,             pal)
                        DebugRow("Back",  "${"%.1f".format(closure)} m",                            pal.orange,           pal)
                        DebugRow("Avg Q", "${"%.2f".format(avgQ)}",                                 pal.textSecondary,    pal)
                    }
                }
                if (session == null) {
                    Text("Walk out & back to calibrate", color = pal.textSecondary, fontSize = 9.sp)
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                        Button(onClick = { viewModel.startScaleCalibration(5.0) },
                            modifier = Modifier.weight(1f).height(26.dp), contentPadding = PaddingValues(0.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = pal.teal), shape = RoundedCornerShape(4.dp)
                        ) { Text("5m", color = Color.White, fontSize = 9.sp, fontWeight = FontWeight.Bold) }
                        Button(onClick = { viewModel.startScaleCalibration(10.0) },
                            modifier = Modifier.weight(1f).height(26.dp), contentPadding = PaddingValues(0.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = pal.orange), shape = RoundedCornerShape(4.dp)
                        ) { Text("10m", color = Color.White, fontSize = 9.sp, fontWeight = FontWeight.Bold) }
                        OutlinedButton(onClick = { viewModel.resetScaleCalibration() },
                            modifier = Modifier.weight(1f).height(26.dp), contentPadding = PaddingValues(0.dp),
                            border = BorderStroke(1.dp, pal.textSecondary), shape = RoundedCornerShape(4.dp)
                        ) { Text("Reset", color = pal.textSecondary, fontSize = 9.sp) }
                    }
                } else {
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                        Button(onClick = { viewModel.finishScaleCalibration() },
                            modifier = Modifier.weight(1f).height(26.dp), contentPadding = PaddingValues(0.dp),
                            colors = ButtonDefaults.buttonColors(containerColor = pal.teal), shape = RoundedCornerShape(4.dp)
                        ) { Text("Finish", color = Color.White, fontSize = 9.sp, fontWeight = FontWeight.Bold) }
                        OutlinedButton(onClick = { viewModel.cancelScaleCalibration() },
                            modifier = Modifier.weight(1f).height(26.dp), contentPadding = PaddingValues(0.dp),
                            border = BorderStroke(1.dp, pal.textSecondary), shape = RoundedCornerShape(4.dp)
                        ) { Text("Cancel", color = pal.textSecondary, fontSize = 9.sp) }
                    }
                }
                calibMsg?.let { Text(it, color = pal.orange, fontSize = 9.sp, modifier = Modifier.padding(top = 2.dp)) }
            }
        }
    }
}

@Composable
fun DebugRow(label: String, value: String, valueColor: Color, pal: NavPalette) {
    Row(Modifier.fillMaxWidth(), Arrangement.SpaceBetween, Alignment.CenterVertically) {
        Text(label, color = pal.textSecondary, fontSize = 10.sp)
        Text(value, color = valueColor,        fontSize = 10.sp, fontWeight = FontWeight.Bold)
    }
}
