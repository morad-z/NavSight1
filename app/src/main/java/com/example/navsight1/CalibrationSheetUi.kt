package com.example.navsight1

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.ModalBottomSheet
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * 2026-06-12ui — Calibration sheet (spec §3, approved mock header-final.html). Opened from
 * the header VIO chip. Three rows, each wired ONLY to functions that already exist:
 *   Camera        → the existing camera-calibration screen (caller opens it)
 *   Mount height  → NavSightViewModel.setMountHeight (numeric dialog, clamps like the setter)
 *   Heading       → NativeBridge.seedMadgwickYaw with the current compass azimuth
 * The sheet shows state; it never auto-runs anything.
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun CalibrationSheet(
    viewModel: NavSightViewModel,
    pal: NavPalette,
    compassAzimuthDeg: Float,
    onOpenCameraCalibration: () -> Unit,
    onDismiss: () -> Unit,
) {
    var showHeightDialog by remember { mutableStateOf(false) }
    ModalBottomSheet(onDismissRequest = onDismiss, containerColor = pal.card) {
        Column(Modifier.padding(horizontal = 18.dp).padding(bottom = 22.dp)) {
            Text("Calibration", color = pal.textPrimary, fontSize = 15.sp,
                fontWeight = FontWeight.ExtraBold)
            Spacer(Modifier.width(0.dp))
            CalibRow(
                title = "Camera",
                caption = if (viewModel.calibrationLoaded) "calibrated · OK" else "not calibrated",
                action = "Recalibrate", pal = pal, enabled = true,
            ) { onOpenCameraCalibration(); onDismiss() }
            HorizontalDivider(color = pal.cardBorder)
            CalibRow(
                title = "Mount height",
                caption = "%.2f m · %s".format(viewModel.mountHeightM,
                    viewModel.heightCalibStatus ?: "auto-tuning from GPS"),
                action = "Set…", pal = pal, enabled = true,
            ) { showHeightDialog = true }
            HorizontalDivider(color = pal.cardBorder)
            CalibRow(
                title = "Heading",
                caption = if (compassAzimuthDeg.isFinite())
                    "compass %.0f°".format(compassAzimuthDeg) else "no compass reading yet",
                action = "Re-snap", pal = pal, enabled = compassAzimuthDeg.isFinite(),
            ) {
                runCatching { NativeBridge.seedMadgwickYaw(Math.toRadians(compassAzimuthDeg.toDouble())) }
                onDismiss()
            }
        }
    }
    if (showHeightDialog) {
        MountHeightDialog(
            currentM = viewModel.mountHeightM, pal = pal,
            onSet = { viewModel.setMountHeight(it); showHeightDialog = false },
            onCancel = { showHeightDialog = false },
        )
    }
}

@Composable
private fun CalibRow(title: String, caption: String, action: String, pal: NavPalette,
                     enabled: Boolean, onClick: () -> Unit) {
    Row(Modifier.fillMaxWidth().padding(vertical = 10.dp),
        verticalAlignment = Alignment.CenterVertically) {
        Column(Modifier.weight(1f)) {
            Text(title, color = pal.textPrimary, fontSize = 13.sp, fontWeight = FontWeight.Bold)
            Text(caption, color = pal.textSecondary, fontSize = 10.sp)
        }
        OutlinedButton(onClick = onClick, enabled = enabled) {
            Text(action, color = if (enabled) pal.brand else pal.textSecondary, fontSize = 11.sp,
                fontWeight = FontWeight.Bold)
        }
    }
}

/** Numeric mount-height entry; validates the same 0.3–2.5 m band setMountHeight clamps to. */
@Composable
private fun MountHeightDialog(currentM: Double, pal: NavPalette,
                              onSet: (Double) -> Unit, onCancel: () -> Unit) {
    var text by remember { mutableStateOf("%.2f".format(currentM)) }
    val parsed = text.replace(',', '.').toDoubleOrNull()
    val valid = parsed != null && parsed in 0.3..2.5
    AlertDialog(
        onDismissRequest = onCancel,
        containerColor = pal.card,
        title = { Text("Mount height (m)", color = pal.textPrimary, fontSize = 14.sp,
            fontWeight = FontWeight.Bold) },
        text = {
            Column {
                OutlinedTextField(value = text, onValueChange = { text = it }, singleLine = true)
                Text(if (valid) "camera-to-road distance" else "enter 0.30 – 2.50 m",
                    color = if (valid) pal.textSecondary else pal.statusBad, fontSize = 10.sp,
                    modifier = Modifier.padding(top = 4.dp))
            }
        },
        confirmButton = {
            TextButton(onClick = { parsed?.let(onSet) }, enabled = valid) {
                Text("Set", color = if (valid) pal.brand else pal.textSecondary,
                    fontWeight = FontWeight.Bold)
            }
        },
        dismissButton = { TextButton(onClick = onCancel) { Text("Cancel", color = pal.textSecondary) } },
    )
}
