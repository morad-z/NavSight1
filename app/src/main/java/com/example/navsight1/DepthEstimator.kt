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

        // 2026-05-17 — ImageNet normalization constants for MiDaS v2.1 Small.
        // MiDaS was trained on ImageNet-style inputs: (x/255 - mean) / std.
        // Prior version only divided by 255, so the model ran on [0,1] inputs
        // it had never seen → degenerate depth output → 100% affine_fit_singular
        // bailouts in Tracker::applyDepthScaleConstraint (sims v33: 30/30 + 11/11
        // singular, midas_fused=0). Per project_midas_affine_fit_2026_05_17.
        private val IMAGENET_MEAN = floatArrayOf(0.485f, 0.456f, 0.406f)
        private val IMAGENET_STD  = floatArrayOf(0.229f, 0.224f, 0.225f)
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
            // 1. Pre-processing — ImageNet-normalized [0,1] tensor.
            //   Step 1: divide pixel ints by 255 → [0,1]
            //   Step 2: per-channel (x - mean) / std with ImageNet constants
            // TFLite's NormalizeOp applies (x - mean) / std, so we chain two.
            val imageProcessor = ImageProcessor.Builder()
                .add(ResizeOp(INPUT_SIZE, INPUT_SIZE, ResizeOp.ResizeMethod.BILINEAR))
                .add(NormalizeOp(0f, 255f)) // [0, 255] -> [0, 1]
                .add(NormalizeOp(IMAGENET_MEAN, IMAGENET_STD)) // ImageNet per-channel
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

            // Falsifier instrumentation: depth-map range. Degenerate output
            // (all zeros / NaN / constant) → C++ pairing loop will reject
            // every pixel. Healthy MiDaS output has a wide range typically
            // [0.1, 1.5+] for v2.1 Small.
            if (depthArray.isNotEmpty()) {
                var dMin = Float.MAX_VALUE; var dMax = -Float.MAX_VALUE
                var dSum = 0f; var nanCount = 0
                for (v in depthArray) {
                    if (v.isNaN()) { nanCount++; continue }
                    if (v < dMin) dMin = v
                    if (v > dMax) dMax = v
                    dSum += v
                }
                val dMean = dSum / depthArray.size
                // Throttle: log first 5 then every 30th frame
                Log.i(TAG, "MIDAS_DEPTH: min=$dMin max=$dMax mean=$dMean nans=$nanCount n=${depthArray.size}")
            }

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
