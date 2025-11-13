package com.example.navsight1

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Text
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import org.opencv.android.OpenCVLoader

class MainActivity : ComponentActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        setContent {
            Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                Text(text = stringFromJNI())
            }
        }
    }

    external fun stringFromJNI(): String

    companion object {
        private const val TAG = "NavSight"
        init {
            if (OpenCVLoader.initDebug()) {
                Log.d(TAG, "OpenCV is initialized.")
            } else {
                Log.d(TAG, "OpenCV is not initialized.")
            }
            System.loadLibrary("navsight")
        }
    }
}
