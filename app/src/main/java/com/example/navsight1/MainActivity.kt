package com.example.navsight1

import android.Manifest
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.*
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.rememberMultiplePermissionsState
import kotlinx.coroutines.delay

class MainActivity : ComponentActivity() {

    private val viewModel: NavSightViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // Step 6 (Task #31): install the JSON crash sink before any UI work so
        // that an early-startup failure still produces a report.
        CrashLogger.install(applicationContext)
        enableEdgeToEdge()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContent { MaterialTheme(colorScheme = darkColorScheme()) { NavSightApp() } }
    }

    override fun onResume() { super.onResume(); viewModel.onResume() }
    override fun onPause()  { super.onPause();  viewModel.onPause() }

    @OptIn(ExperimentalPermissionsApi::class)
    @Composable
    fun NavSightApp() {
        var showSplash by remember { mutableStateOf(true) }
        // 2026-06-12ui — 3-state night mode (spec §6): auto / day / night, persisted. The old
        // code re-evaluated isNightTime() every 60 s and silently OVERRODE a manual toggle.
        val uiPrefs = remember { getSharedPreferences("navsight_ui", MODE_PRIVATE) }
        var nightMode by remember { mutableStateOf(uiPrefs.getString("night_mode", "auto") ?: "auto") }
        var autoNight by remember { mutableStateOf(isNightTime()) }
        LaunchedEffect(Unit) { while (true) { delay(60_000L); autoNight = isNightTime() } }
        val isNight = when (nightMode) { "day" -> false; "night" -> true; else -> autoNight }
        val pal = remember(isNight) { buildNavPalette(isNight) }

        if (showSplash) { SplashScreen(pal) { showSplash = false }; return }

        val perms = rememberMultiplePermissionsState(
            listOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION)
        )
        LaunchedEffect(perms.allPermissionsGranted) {
            if (perms.allPermissionsGranted) viewModel.requestInitialLocation(true)
        }
        if (!perms.allPermissionsGranted) PermissionScreen(pal) { perms.launchMultiplePermissionRequest() }
        else MainScreen(viewModel, pal, isNight,
            nightModeLabel = when (nightMode) { "auto" -> "Auto"; "day" -> "Day"; else -> "Night" }) {
            nightMode = when (nightMode) { "auto" -> "day"; "day" -> "night"; else -> "auto" }
            uiPrefs.edit().putString("night_mode", nightMode).apply()
        }
    }
}
