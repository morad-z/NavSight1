package com.example.navsight1

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Validates the runtime dynamic-road path (RoadRegionManager's building blocks):
 * an Overpass `out geom` JSON → [RoadsJsonParser] → [SegmentRTreeReader.buildSegmentStore]
 * (snap) + [RoutingGraphReader.buildGraphStore] (route). Confirms the Kotlin port
 * reproduces scripts/build_haifa_assets.py `build_roads()`: highway-class filter,
 * access skip, way segmentation, and vertex dedup.
 */
class DynamicRoadsParseTest {

    // 3 ways: one indexed residential road; one skipped (access=private); one skipped
    // (highway=motorway is not in the v1 class table).
    private val overpassJson = """
        {"elements":[
          {"type":"way","id":100,
           "tags":{"highway":"residential","name":"Test St","oneway":"no"},
           "geometry":[{"lat":32.80,"lon":34.98},{"lat":32.80,"lon":34.99},{"lat":32.80,"lon":35.00}]},
          {"type":"way","id":200,
           "tags":{"highway":"service","access":"private"},
           "geometry":[{"lat":32.81,"lon":34.98},{"lat":32.81,"lon":34.99}]},
          {"type":"way","id":300,
           "tags":{"highway":"motorway"},
           "geometry":[{"lat":32.79,"lon":34.98},{"lat":32.79,"lon":34.99}]}
        ]}
    """.trimIndent()

    @Test
    fun `parse applies highway and access filters and segments the way`() {
        val a = RoadsJsonParser.parse(overpassJson)
        // Only way 100 survives: 3 nodes → 2 segments, 3 unique vertices, 2 edges.
        assertEquals("private + motorway ways skipped", 2, a.segCount)
        assertEquals(3, a.vertexCount)
        assertEquals(2, a.edgeCount)
        assertEquals(100L, a.wayId[0])
        assertEquals("Test St", a.segName[0])
    }

    @Test
    fun `buildSegmentStore snaps a point onto the parsed road`() {
        val store = SegmentRTreeReader.buildSegmentStore(RoadsJsonParser.parse(overpassJson))
        val hits = store.nearestSegments(32.801, 34.99, radiusM = 200.0, maxResults = 4)
        assertTrue("a segment should be found near the road", hits.isNotEmpty())
        assertEquals(100L, hits[0].wayId)
        assertEquals("Test St", hits[0].name)
    }

    @Test
    fun `buildGraphStore routes along the parsed road`() {
        val store = RoutingGraphReader.buildGraphStore(RoadsJsonParser.parse(overpassJson))
        assertEquals(3, store.vertexCount)
        val poly = store.route(32.80, 34.980, 32.80, 35.000)
        assertEquals("route visits all 3 nodes of the road", 3, poly.size)
        assertEquals(34.98, poly.first().lng, 1e-9)
        assertEquals(35.00, poly.last().lng, 1e-9)
    }

    @Test
    fun `empty or roadless input yields an empty store`() {
        val a = RoadsJsonParser.parse("""{"elements":[]}""")
        assertEquals(0, a.segCount)
        val store = SegmentRTreeReader.buildSegmentStore(a)
        assertTrue(store.nearestSegments(32.8, 35.0, 50.0, 4).isEmpty())
    }
}
