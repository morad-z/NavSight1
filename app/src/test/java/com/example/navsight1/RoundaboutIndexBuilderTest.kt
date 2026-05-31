package com.example.navsight1

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

/**
 * Validates [RoundaboutIndexBuilder] end-to-end from an Overpass `out geom` JSON
 * (a junction=roundabout ring + 4 exit ways) through [RoadsJsonParser]. JVM-pure
 * (RoadsJsonParser uses org.json, provided to unit tests).
 */
class RoundaboutIndexBuilderTest {

    private val cLat = 32.800
    private val cLng = 35.000

    private fun coord(angleDeg: Double, distM: Double): Pair<Double, Double> {
        val th = Math.toRadians(angleDeg)
        val north = distM * cos(th); val east = distM * sin(th)
        val lat = cLat + north / 111_320.0
        val lng = cLng + east / (111_320.0 * cos(Math.toRadians(cLat)))
        return lat to lng
    }

    private fun node(p: Pair<Double, Double>) = """{"lat":${p.first},"lon":${p.second}}"""

    private fun overpassJson(): String {
        // 8-node ring at radius 20 m (closed), + 4 exit stubs at N/E/S/W.
        val ringNodes = (0 until 8).map { coord(it * 45.0, 20.0) }
        val ringGeom = (ringNodes + ringNodes.first()).joinToString(",") { node(it) }
        val ringWay = """{"type":"way","id":1,"tags":{"highway":"residential","junction":"roundabout"},"geometry":[$ringGeom]}"""
        val exits = listOf(0.0, 90.0, 180.0, 270.0).mapIndexed { i, a ->
            val ring = coord(a, 20.0); val far = coord(a, 50.0)
            """{"type":"way","id":${10 + i},"tags":{"highway":"residential"},"geometry":[${node(ring)},${node(far)}]}"""
        }
        return """{"elements":[$ringWay,${exits.joinToString(",")}]}"""
    }

    @Test
    fun `builds one ring with centre, radius and four exit bearings`() {
        val arrays = RoadsJsonParser.parse(overpassJson())
        val idx = RoundaboutIndexBuilder.build(arrays)
        assertEquals("one roundabout", 1, idx.size)
        val ring = idx.rings[0]
        assertEquals(1L, ring.wayId)
        assertEquals(cLat, ring.centerLat, 1e-4)
        assertEquals(cLng, ring.centerLng, 1e-4)
        assertEquals("radius ≈ 20 m", 20.0, ring.radiusM, 2.0)
        assertEquals("four exits", 4, ring.exits.size)
        val bearings = ring.exits.map { it.bearingDeg }.sorted()
        // ≈ {0, 90, 180, 270}
        listOf(0.0, 90.0, 180.0, 270.0).forEachIndexed { i, expected ->
            assertTrue("exit bearing ${bearings[i]} ≈ $expected", kotlin.math.abs(bearings[i] - expected) < 5.0)
        }
    }

    @Test
    fun `ring with fewer than two exits is not emitted`() {
        val ringNodes = (0 until 8).map { coord(it * 45.0, 20.0) }
        val ringGeom = (ringNodes + ringNodes.first()).joinToString(",") { node(it) }
        val ringWay = """{"type":"way","id":1,"tags":{"highway":"residential","junction":"roundabout"},"geometry":[$ringGeom]}"""
        val oneExit = run {
            val ring = coord(0.0, 20.0); val far = coord(0.0, 50.0)
            """{"type":"way","id":10,"tags":{"highway":"residential"},"geometry":[${node(ring)},${node(far)}]}"""
        }
        val json = """{"elements":[$ringWay,$oneExit]}"""
        val idx = RoundaboutIndexBuilder.build(RoadsJsonParser.parse(json))
        assertEquals("incomplete roundabout dropped", 0, idx.size)
    }
}
