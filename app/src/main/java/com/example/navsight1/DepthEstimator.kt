package com.example.navsight1

import android.content.Context
import android.graphics.Bitmap
import android.util.Log
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.support.common.FileUtil
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * MiDaS v2.1 Small depth estimator for automatic VIO scale calibration.
 *
 * Uses TensorFlow Lite to run monocular depth estimation on camera frames.
 * The estimated depth is used to provide absolute scale information to the VIO system.
 */
class DepthEstimator(context: Context) {
    private val TAG = "DepthEstimator"

    private var interpreter: Interpreter? = null
    private val inputSize = 256  // MiDaS small input size
    private val inputBuffer: ByteBuffer
    private val outputBuffer: ByteBuffer

    // Reference depth for scale estimation (ground plane assumption)
    // Typical mounted camera height: 0.3-0.5m for scooter/bike
    private var referenceHeight = 0.4f  // meters

    init {
        try {
            val model = FileUtil.loadMappedFile(context, "midas_v2_small.tflite")
            val options = Interpreter.Options().apply {
                setNumThreads(2)
            }
            interpreter = Interpreter(model, options)
            Log.d(TAG, "DepthEstimator initialized successfully")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initialize DepthEstimator: ${e.message}", e)
            interpreter = null
        }

        // Pre-allocate input buffer: 1 x 256 x 256 x 3 x 4 bytes (float32)
        inputBuffer = ByteBuffer.allocateDirect(1 * inputSize * inputSize * 3 * 4).apply {
            order(ByteOrder.nativeOrder())
        }

        // Pre-allocate output buffer: 1 x 256 x 256 x 1 x 4 bytes (float32)
        outputBuffer = ByteBuffer.allocateDirect(1 * inputSize * inputSize * 4).apply {
            order(ByteOrder.nativeOrder())
        }
    }

    /**
     * Estimate depth from a camera frame bitmap.
     * Returns the median depth value in the image.
     */
    fun estimateDepth(bitmap: Bitmap): Float {
        val interp = interpreter ?: return 0f

        try {
            // Resize to model input size
            val resized = Bitmap.createScaledBitmap(bitmap, inputSize, inputSize, true)

            // Preprocess: convert to float and normalize to [0, 1]
            preprocessBitmap(resized, inputBuffer)

            // Run inference
            inputBuffer.rewind()
            outputBuffer.rewind()
            interp.run(inputBuffer, outputBuffer)

            // Get median depth from output
            outputBuffer.rewind()
            val depths = FloatArray(inputSize * inputSize)
            outputBuffer.asFloatBuffer().get(depths)

            // MiDaS outputs inverse depth (disparity), larger = closer
            // Sort and get median
            depths.sort()
            val medianDepth = depths[depths.size / 2]

            // Recycle the scaled bitmap if it's different from input
            if (resized != bitmap) {
                resized.recycle()
            }

            return medianDepth
        } catch (e: Exception) {
            Log.e(TAG, "Depth estimation failed: ${e.message}", e)
            return 0f
        }
    }

    /**
     * Estimate scale factor from depth.
     * Uses the ground plane assumption: if we know the camera height,
     * we can derive absolute scale from the depth of the ground.
     *
     * @param depth The median depth value from MiDaS (inverse depth/disparity)
     * @return Scale factor to apply to VIO translation, or null if invalid
     */
    fun estimateScaleFromDepth(depth: Float): Float? {
        if (depth <= 0f) return null

        // MiDaS outputs inverse depth (disparity)
        // Convert to relative depth scale
        // The scale factor relates MiDaS disparity to real-world meters

        // Ground plane assumption:
        // If camera is at height h, ground appears at depth ~h
        // MiDaS disparity d ~ 1/h_midas
        // Real height h_real = referenceHeight
        // Scale = h_real * d (approximately)

        // Use a calibration factor based on typical MiDaS output range
        // MiDaS small typically outputs disparity in range [0.1, 10+]
        // Higher disparity = closer object

        val calibrationFactor = 0.1f  // Empirical calibration factor
        val scale = (referenceHeight * depth * calibrationFactor).coerceIn(0.1f, 10f)

        return scale
    }

    /**
     * Set the reference camera mount height for scale calibration.
     */
    fun setReferenceHeight(heightM: Float) {
        referenceHeight = heightM.coerceIn(0.1f, 2.0f)
        Log.d(TAG, "Reference height set to ${referenceHeight}m")
    }

    private fun preprocessBitmap(bitmap: Bitmap, buffer: ByteBuffer) {
        buffer.rewind()
        val pixels = IntArray(inputSize * inputSize)
        bitmap.getPixels(pixels, 0, inputSize, 0, 0, inputSize, inputSize)

        for (pixel in pixels) {
            // Extract RGB and normalize to [0, 1]
            val r = ((pixel shr 16) and 0xFF) / 255.0f
            val g = ((pixel shr 8) and 0xFF) / 255.0f
            val b = (pixel and 0xFF) / 255.0f

            buffer.putFloat(r)
            buffer.putFloat(g)
            buffer.putFloat(b)
        }
    }

    fun isInitialized(): Boolean = interpreter != null

    fun close() {
        interpreter?.close()
        interpreter = null
        Log.d(TAG, "DepthEstimator closed")
    }
}
