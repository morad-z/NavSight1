package com.example.navsight1

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.collectAsState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * Allan-variance IMU calibration recording screen.
 *
 * Standalone overlay. Does not run VIO, does not open camera, does not consume
 * GPS. Just samples gyro + accel at SENSOR_DELAY_FASTEST and writes a CSV.
 *
 * Goal usage: place phone flat on a stable non-vibrating surface, wait 10
 * minutes for thermal equilibrium, START, leave 1-2 hours untouched, STOP.
 * Pull the CSV with adb and feed it to
 * `scripts/allan_variance/calibrate.py`.
 *
 * UI is deliberately minimal — fewer fingers on the phone = less mechanical
 * coupling = cleaner noise data.
 */
// Preset durations for auto-stop. null = "manual" (no auto-stop).
private data class DurationPreset(val label: String, val seconds: Long?)
private val DURATION_PRESETS = listOf(
    DurationPreset("Manual", null),
    DurationPreset("30 min", 30L * 60),
    DurationPreset("1 hour", 60L * 60),
    DurationPreset("2 hours", 2L * 60 * 60),
    DurationPreset("4 hours", 4L * 60 * 60),
)

@Composable
fun ImuCalibrationScreen(
    pal: NavPalette,
    onClose: () -> Unit,
) {
    val context = LocalContext.current
    val recorder = remember { ImuCalibrationRecorder() }
    val state by recorder.state.collectAsState()

    // Default 2 hours per the recording procedure recommendation.
    var selectedPreset by remember { mutableStateOf(DURATION_PRESETS[3]) }

    DisposableEffect(Unit) {
        onDispose { recorder.stop() }
    }

    // Box parent so the action buttons stay pinned to the bottom regardless
    // of scroll position. The scrolling content reserves bottom padding so it
    // can't slide under the sticky button stack.
    Box(Modifier.fillMaxSize().background(pal.bg)) {

        Column(
            Modifier
                .fillMaxSize()
                .padding(24.dp)
                .padding(bottom = 140.dp)  // reserve room for sticky buttons
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(16.dp),
        ) {
            Text(
                "IMU Allan-Variance Calibration",
                color = pal.textPrimary,
                fontWeight = FontWeight.Bold,
                fontSize = 22.sp,
            )
            Text(
                "Records raw gyroscope + accelerometer at the sensor's fastest rate. " +
                    "No camera, no VIO, no GPS. Use for per-device noise characterization.",
                color = pal.textSecondary,
                fontSize = 13.sp,
            )

            // Duration selector — promoted ABOVE status so it's the first thing
            // a fresh user sees. Disabled while recording.
            Surface(
                color = pal.card,
                shape = RoundedCornerShape(12.dp),
                modifier = Modifier.fillMaxWidth(),
            ) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text(
                        "Auto-stop after",
                        color = pal.textPrimary,
                        fontWeight = FontWeight.SemiBold,
                        fontSize = 15.sp,
                    )
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        DURATION_PRESETS.forEach { preset ->
                            val selected = preset.seconds == selectedPreset.seconds &&
                                preset.label == selectedPreset.label
                            FilterChip(
                                selected = selected,
                                onClick = { if (!state.isRecording) selectedPreset = preset },
                                enabled = !state.isRecording,
                                label = { Text(preset.label, fontSize = 12.sp) },
                                colors = FilterChipDefaults.filterChipColors(
                                    selectedContainerColor = pal.teal,
                                    selectedLabelColor = pal.bg,
                                ),
                            )
                        }
                    }
                    Text(
                        if (selectedPreset.seconds == null)
                            "No auto-stop — you'll need to come back and tap STOP."
                        else
                            "Recorder will stop automatically after the selected time.",
                        color = pal.textSecondary,
                        fontSize = 11.sp,
                    )
                }
            }

            // Status card
            Surface(
                color = pal.card,
                shape = RoundedCornerShape(12.dp),
                modifier = Modifier.fillMaxWidth(),
            ) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    val statusText = when {
                        state.isRecording -> "RECORDING"
                        state.autoStopped -> "AUTO-STOPPED ✓"
                        else -> "Ready"
                    }
                    StatRow("Status", statusText, pal)
                    StatRow("Elapsed", formatElapsed(state.elapsedSeconds), pal)
                    state.targetDurationSec?.let { target ->
                        val remaining = (target - state.elapsedSeconds).coerceAtLeast(0L)
                        StatRow("Remaining", formatElapsed(remaining), pal)
                    }
                    StatRow("Samples", state.sampleCount.toString(), pal)
                    if (state.elapsedSeconds > 0 && state.sampleCount > 0) {
                        val rate = state.sampleCount.toDouble() / state.elapsedSeconds.toDouble()
                        StatRow("Rate", "${"%.1f".format(rate)} Hz", pal)
                    }
                    if (state.droppedAccel > 0) {
                        StatRow("Dropped (warmup)", state.droppedAccel.toString(), pal)
                    }
                    state.outputPath?.let { StatRow("File", it.substringAfterLast('/'), pal, small = true) }
                    state.errorMessage?.let {
                        Text("Error: $it", color = pal.orangeAcc, fontSize = 12.sp)
                    }
                }
            }

            // Procedure block — reference text, last so it doesn't gate the action.
            Surface(
                color = pal.card,
                shape = RoundedCornerShape(12.dp),
                modifier = Modifier.fillMaxWidth(),
            ) {
                Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("Procedure", color = pal.textPrimary, fontWeight = FontWeight.SemiBold, fontSize = 15.sp)
                    ProcStep("1. Phone flat on a stable, non-vibrating surface (NOT a laptop).", pal)
                    ProcStep("2. Plug into power. Don't touch the phone after START.", pal)
                    ProcStep("3. Wait ≥ 10 min for thermal equilibrium before pressing START.", pal)
                    ProcStep("4. START — leave for the selected duration untouched.", pal)
                    ProcStep("5. STOP / auto-stop. Pull the CSV via adb. Run calibrate.py.", pal)
                }
            }
        }

        // Sticky bottom action stack — always visible regardless of scroll.
        Column(
            Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .background(pal.bg)
                .padding(horizontal = 24.dp, vertical = 16.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            if (state.isRecording) {
                Button(
                    onClick = { recorder.stop() },
                    colors = ButtonDefaults.buttonColors(containerColor = pal.orangeAcc),
                    modifier = Modifier.fillMaxWidth().height(56.dp),
                ) { Text("STOP", color = pal.bg, fontWeight = FontWeight.Bold) }
            } else {
                Button(
                    onClick = { recorder.start(context, selectedPreset.seconds) },
                    colors = ButtonDefaults.buttonColors(containerColor = pal.teal),
                    modifier = Modifier.fillMaxWidth().height(56.dp),
                ) {
                    val label = if (selectedPreset.seconds == null) "START (manual stop)"
                                else "START — auto-stop after ${selectedPreset.label}"
                    Text(label, color = pal.bg, fontWeight = FontWeight.Bold)
                }
            }

            TextButton(
                onClick = onClose,
                modifier = Modifier.fillMaxWidth(),
                enabled = !state.isRecording,
            ) {
                Text(
                    if (state.isRecording) "Stop before closing" else "Close",
                    color = if (state.isRecording) pal.textSecondary else pal.textPrimary,
                )
            }
        }
    }
}

@Composable
private fun StatRow(label: String, value: String, pal: NavPalette, small: Boolean = false) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, color = pal.textSecondary, fontSize = 13.sp)
        Text(
            value,
            color = pal.textPrimary,
            fontSize = if (small) 11.sp else 15.sp,
            fontFamily = FontFamily.Monospace,
        )
    }
}

@Composable
private fun ProcStep(text: String, pal: NavPalette) {
    Text(text, color = pal.textSecondary, fontSize = 13.sp)
}

private fun formatElapsed(secs: Long): String {
    val h = secs / 3600
    val m = (secs % 3600) / 60
    val s = secs % 60
    return "%02d:%02d:%02d".format(h, m, s)
}
