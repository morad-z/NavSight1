package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Tests the #8 route-pinned projection ([RoutePinMath]): cross-track distance for
 * off-route detection + the pinned point. JVM-pure (gms LatLng works in unit tests).
 */
class RoutePinMathTest {

    // A short straight route heading east at lat 32.80.
    private val a = LatLng(32.80, 35.000)
    private val b = LatLng(32.80, 35.002)
    private val route = listOf(a, b)

    @Test
    fun `point on the route has near-zero cross-track`() {
        val onRoute = LatLng(32.80, 35.0015)
        val proj = RoutePinMath.nearestOnPolyline(onRoute, route)
        assertTrue("cross-track ${proj.distanceM} ≈ 0", proj.distanceM < 0.5)
        assertEquals(35.0015, proj.point.longitude, 1e-5)
    }

    @Test
    fun `point offset from the route reports the perpendicular distance`() {
        // 10 m north of the midpoint.
        val off = LatLng(32.80 + 10.0 / 111_320.0, 35.001)
        val proj = RoutePinMath.nearestOnPolyline(off, route)
        assertEquals("cross-track ≈ 10 m", 10.0, proj.distanceM, 0.5)
        // Projected back onto the line (lat ≈ route lat, lng ≈ midpoint).
        assertEquals(32.80, proj.point.latitude, 1e-5)
        assertEquals(35.001, proj.point.longitude, 1e-4)
    }

    @Test
    fun `point beyond the segment clamps to the endpoint`() {
        // East of b → nearest point is b itself.
        val beyond = LatLng(32.80, 35.003)
        val proj = RoutePinMath.nearestOnPolyline(beyond, route)
        assertEquals(b.longitude, proj.point.longitude, 1e-6)
        assertEquals(b.latitude, proj.point.latitude, 1e-6)
        // ~0.001° lng east of b at this latitude ≈ 93 m.
        assertTrue("distance ${proj.distanceM} ~90 m", proj.distanceM in 80.0..105.0)
    }

    @Test
    fun `empty polyline is treated as infinitely far (off-route)`() {
        val proj = RoutePinMath.nearestOnPolyline(a, emptyList())
        assertTrue(proj.distanceM > 1e6)
    }
}
