package com.example.navsight1

import org.junit.Assert.*
import org.junit.Test
import com.google.android.gms.maps.model.LatLng
import kotlin.math.sqrt

/**
 * Unit tests for VioData and distance computation logic.
 * These run on the host JVM (no device needed).
 *
 * Run: ./gradlew app:testDebugUnitTest
 */
class VioDataTest {

    @Test
    fun vioData_defaultValues() {
        val vio = VioData()
        assertEquals(0.0, vio.x, 0.001)
        assertEquals(0.0, vio.y, 0.001)
        assertEquals(0.0, vio.z, 0.001)
        assertEquals(0.0, vio.trackingQuality, 0.001)
        assertFalse(vio.isInitialized)
        assertEquals(1.0, vio.estimatedScale, 0.001)
    }

    @Test
    fun vioData_equality() {
        val a = VioData(x = 1.0, y = 2.0, z = 3.0)
        val b = VioData(x = 1.0, y = 2.0, z = 3.0)
        assertEquals(a, b)
    }

    @Test
    fun vioData_inequality() {
        val a = VioData(x = 1.0)
        val b = VioData(x = 2.0)
        assertNotEquals(a, b)
    }

    @Test
    fun distanceFromOrigin_calculation() {
        val vio = VioData(x = 3.0, y = 0.0, z = 4.0)
        val dist = sqrt(vio.x * vio.x + vio.z * vio.z)
        assertEquals(5.0, dist, 0.001) // 3-4-5 triangle
    }

    @Test
    fun totalDistance_accumulation() {
        // Simulate the ViewModel distance accumulation logic
        val positions = listOf(
            VioData(x = 0.0, z = 0.0),
            VioData(x = 1.0, z = 0.0),  // moved 1m east
            VioData(x = 1.0, z = 1.0),  // moved 1m north
            VioData(x = 0.0, z = 1.0),  // moved 1m west
            VioData(x = 0.0, z = 0.0),  // moved 1m south (back to start)
        )

        var totalDist = 0.0
        for (i in 1 until positions.size) {
            val dx = positions[i].x - positions[i - 1].x
            val dz = positions[i].z - positions[i - 1].z
            totalDist += sqrt(dx * dx + dz * dz)
        }

        assertEquals("Total path around 1m square should be 4m", 4.0, totalDist, 0.001)

        // Displacement from start
        val displacement = sqrt(
            positions.last().x * positions.last().x +
            positions.last().z * positions.last().z
        )
        assertEquals("Back at start, displacement should be 0", 0.0, displacement, 0.001)
    }

    @Test
    fun qualityClassification() {
        // Test the fusion mode classification used in DebugPanel
        fun fusionMode(quality: Double, initialized: Boolean): String = when {
            !initialized -> "INIT"
            quality < 0.3 -> "IMU"
            quality > 0.7 -> "CAMERA"
            else -> "HYBRID"
        }

        assertEquals("INIT", fusionMode(0.5, false))
        assertEquals("IMU", fusionMode(0.1, true))
        assertEquals("HYBRID", fusionMode(0.5, true))
        assertEquals("CAMERA", fusionMode(0.9, true))
    }

    @Test
    fun trackedPoints_evenLength() {
        // trackedPoints should always have even length (x,y pairs)
        val vio = VioData(trackedPoints = floatArrayOf(1f, 2f, 3f, 4f))
        assertEquals(0, vio.trackedPoints.size % 2)
    }

    @Test
    fun headingFusion_calculation() {
        // Test the heading fusion formula from MainScreen
        val vioInitAzimuth = 90f  // Started facing East
        val vioYawRad = Math.PI / 2  // Turned 90 degrees left (CCW)
        val vioYawDeg = Math.toDegrees(vioYawRad).toFloat()

        val fusedHeading = (vioInitAzimuth - vioYawDeg + 360f) % 360f

        // Started at 90 (East), turned 90 left → should face 0 (North)
        assertEquals(0f, fusedHeading, 1f)
    }

    @Test
    fun headingFusion_wraparound() {
        val vioInitAzimuth = 10f  // Started facing ~North
        val vioYawRad = Math.PI / 4  // Turned 45 degrees left
        val vioYawDeg = Math.toDegrees(vioYawRad).toFloat()

        val fusedHeading = (vioInitAzimuth - vioYawDeg + 360f) % 360f

        // 10 - 45 + 360 = 325 degrees (NNW)
        assertEquals(325f, fusedHeading, 1f)
    }

    @Test
    fun resolveNavigationStart_prefersSnappedPosition() {
        val snapped = LatLng(1.0, 2.0)
        val start = LatLng(3.0, 4.0)

        val resolved = NavSightUtils.resolveNavigationStart(snapped, start)

        assertEquals(snapped, resolved)
    }

    @Test
    fun resolveNavigationStart_returnsNullWhenNoLocationAvailable() {
        val resolved = NavSightUtils.resolveNavigationStart(null, null)

        assertNull(resolved)
    }

    @Test
    fun computeCalibrationStraightness_capsAtOne() {
        val straightness = NavSightUtils.computeCalibrationStraightness(
            displacementMeters = 9.0,
            pathLengthMeters = 8.0
        )

        assertEquals(1.0, straightness, 0.001)
    }

    @Test
    fun computeUpdatedScaleCalibrationFactor_multipliesCurrentFactor() {
        val updated = NavSightUtils.computeUpdatedScaleCalibrationFactor(
            currentFactor = 1.2,
            knownDistanceMeters = 10.0,
            measuredDistanceMeters = 8.0
        )

        assertEquals(1.5, updated!!, 0.001)
    }
}
