package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Pure tests for [LiveMatcher.parse] (OSRM `/match` response → dot + matched geometry
 * + route distance + broken flag) and [LiveMatcher.buildBearings] (#1, per-point travel
 * direction). JVM-pure (org.json + gms LatLng work in unit tests). The HTTP call is not
 * unit-tested (network).
 */
class LiveMatcherParseTest {

    private val m = LiveMatcher()

    @Test
    fun `parses dot, confidence, geometry and route distance`() {
        val json = """
            {"code":"Ok",
             "matchings":[{"confidence":0.91,"distance":42.0,
               "geometry":{"type":"LineString","coordinates":[
                 [35.0010,32.8000],[35.0020,32.8005],[35.0030,32.8010]]}}],
             "tracepoints":[
               {"location":[35.0010,32.8000]},
               {"location":[35.0020,32.8005]},
               {"location":[35.0030,32.8010]}
             ]}
        """.trimIndent()
        val r = m.parse(json)
        assertNotNull(r)
        // dot = latest tracepoint, [lon,lat] → LatLng(lat,lon)
        assertEquals(32.8010, r!!.matched.latitude, 1e-9)
        assertEquals(35.0030, r.matched.longitude, 1e-9)
        assertEquals(0.91, r.confidence, 1e-9)
        assertEquals(42.0, r.routeDistanceM, 1e-9)
        assertEquals(3, r.matchedPath.size)
        assertEquals(32.8000, r.matchedPath.first().latitude, 1e-9)
        assertFalse(r.broken)
    }

    @Test
    fun `broken trace concatenates geometry and sums distance`() {
        val json = """
            {"code":"Ok",
             "matchings":[
               {"confidence":0.40,"distance":50.0,"geometry":{"type":"LineString",
                 "coordinates":[[35.001,32.800],[35.002,32.801]]}},
               {"confidence":0.30,"distance":30.0,"geometry":{"type":"LineString",
                 "coordinates":[[35.003,32.802],[35.004,32.803]]}}
             ],
             "tracepoints":[{"location":[35.001,32.800]},{"location":[35.004,32.803]}]}
        """.trimIndent()
        val r = m.parse(json)
        assertNotNull(r)
        assertTrue("two matchings → broken", r!!.broken)
        assertEquals(80.0, r.routeDistanceM, 1e-9)   // 50 + 30
        assertEquals(4, r.matchedPath.size)          // 2 + 2 coords
        assertEquals(0.40, r.confidence, 1e-9)       // matchings[0]
    }

    @Test
    fun `skips trailing null tracepoints (unmatched latest points)`() {
        val json = """
            {"code":"Ok","matchings":[{"confidence":0.5,"distance":0,"geometry":{"coordinates":[]}}],
             "tracepoints":[{"location":[35.0010,32.8000]},{"location":[35.0020,32.8005]},null,null]}
        """.trimIndent()
        val r = m.parse(json)
        assertNotNull(r)
        assertEquals(32.8005, r!!.matched.latitude, 1e-9)
    }

    @Test
    fun `returns null when OSRM reports no match`() {
        assertNull(m.parse("""{"code":"NoMatch","message":"Could not match the trace."}"""))
    }

    @Test
    fun `returns null when all tracepoints are null`() {
        assertNull(m.parse("""{"code":"Ok","matchings":[],"tracepoints":[null,null]}"""))
    }

    @Test
    fun `returns null when tracepoints array is absent`() {
        assertNull(m.parse("""{"code":"Ok","matchings":[{"confidence":0.7}]}"""))
    }

    @Test
    fun `buildBearings gives empty first entry then incoming segment bearings`() {
        val mPerDegLat = 111_320.0
        val p0 = LatLng(32.80, 35.00)
        val p1 = LatLng(32.80 + 10.0 / mPerDegLat, 35.00)                                  // 10 m north → 0°
        val p2 = LatLng(p1.latitude, p1.longitude + 10.0 / (mPerDegLat * Math.cos(Math.toRadians(32.80)))) // 10 m east → 90°
        assertEquals(";0,60;90,60", m.buildBearings(listOf(p0, p1, p2)))
    }

    @Test
    fun `buildBearings skips near-stationary segments`() {
        val p0 = LatLng(32.80, 35.00)
        val p1 = LatLng(32.80 + 1.0 / 111_320.0, 35.00)   // ~1 m < 3 m → no bearing
        assertEquals(";", m.buildBearings(listOf(p0, p1)))
    }
}
