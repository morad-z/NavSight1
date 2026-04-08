package com.example.navsight1

import android.content.Context
import android.graphics.Bitmap
import android.util.Log
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.withContext
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.GpuDelegate
import org.tensorflow.lite.support.common.FileUtil
import org.tensorflow.lite.support.common.ops.NormalizeOp
import org.tensorflow.lite.support.image.ImageProcessor
import org.tensorflow.lite.support.image.TensorImage
import org.tensorflow.lite.support.image.ops.ResizeOp
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.Executors

/**
 * High-performance monocular depth estimation using TFLite.
 * Optimized for Android 2024 (GPU/NNAPI delegates + background threading).
 */
class DepthEstimator(private val context: Context) : AutoCloseable {

    companion object {
        private const val TAG = "DepthEstimator"
        private const val MODEL_FILE = "midas_v21_small.tflite"
        private const val INPUT_SIZE = 256
        private const val NUM_THREADS = 4
    }

    private var interpreter: Interpreter? = null
    private var gpuDelegate: GpuDelegate? = null
    
    // Dedicated dispatcher for TFLite inference to avoid thread-safety issues
    private val inferenceDispatcher = Executors.newSingleThreadExecutor().asCoroutineDispatcher()

    init {
        initializeInterpreter()
    }

    private fun initializeInterpreter() {
        try {
            val modelBuffer = FileUtil.loadMappedFile(context, MODEL_FILE)
            val options = Interpreter.Options()

            // Try GPU delegate, fall back to CPU if unsupported
            try {
                gpuDelegate = GpuDelegate()
                options.addDelegate(gpuDelegate)
                Log.d(TAG, "GPU Delegate enabled")
            } catch (e: Exception) {
                Log.d(TAG, "GPU Delegate not available, using CPU with $NUM_THREADS threads: ${e.message}")
                options.setNumThreads(NUM_THREADS)
            }

            interpreter = Interpreter(modelBuffer, options)
            Log.d(TAG, "TFLite Interpreter initialized with ${MODEL_FILE}")
        } catch (e: Exception) {
            Log.e(TAG, "Error initializing TFLite Interpreter: ${e.message}", e)
        }
    }

    /**
     * Estimates depth for a given bitmap. Returns a FloatArray of depth values (normalized).
     */
    suspend fun estimateDepth(bitmap: Bitmap): FloatArray? = withContext(inferenceDispatcher) {
        val interpreter = interpreter ?: return@withContext null

        try {
            // 1. Pre-processing
            val imageProcessor = ImageProcessor.Builder()
                .add(ResizeOp(INPUT_SIZE, INPUT_SIZE, ResizeOp.ResizeMethod.BILINEAR))
                .add(NormalizeOp(0f, 255f)) // Normalize to [0, 1]
                .build()

            val tensorImage = TensorImage(org.tensorflow.lite.DataType.FLOAT32)
            tensorImage.load(bitmap)
            val processedImage = imageProcessor.process(tensorImage)

            // 2. Prepare Output Buffer (MiDaS v2.1 Small output is 256x256x1)
            val outputBuffer = ByteBuffer.allocateDirect(INPUT_SIZE * INPUT_SIZE * 4)
            outputBuffer.order(ByteOrder.nativeOrder())
            outputBuffer.rewind()

            // 3. Inference
            val startTime = System.currentTimeMillis()
            interpreter.run(processedImage.buffer, outputBuffer)
            val endTime = System.currentTimeMillis()
            Log.v(TAG, "Inference time: ${endTime - startTime} ms")

            // 4. Post-processing
            outputBuffer.rewind()
            val depthArray = FloatArray(INPUT_SIZE * INPUT_SIZE)
            outputBuffer.asFloatBuffer().get(depthArray)

            return@withContext depthArray
        } catch (e: Exception) {
            Log.e(TAG, "Error during inference: ${e.message}", e)
            null
        }
    }

    override fun close() {
        inferenceDispatcher.close()
        interpreter?.close()
        gpuDelegate?.close()
        Log.d(TAG, "DepthEstimator resources closed.")
    }
}
