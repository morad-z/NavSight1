package com.example.navsight1

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Pure-JVM round-trip test for the offline OSM data layer (MAP_MATCHING_PLAN.md
 * §8M Step A* step 7: "the replay-before-reflash guard for the data layer").
 *
 * It builds tiny SYNTHETIC blobs in the exact on-disk format (matching
 * OsmAssetFormat / the readers — NO osmpbf, NO external tools, runs on the JVM),
 * parses them with the real readers, and asserts the snap / route / geocode
 * queries return the planted data. This is the "make no mistakes" falsifier for
 * the binary format: writer and readers must agree byte-for-byte.
 *
 * (§3 Principle 6: synthetic OSM is allowed in TESTS only, never in production.)
 */
class OsmDataLayerRoundTripTest {

    // ── RoadSnapMath geometry ───────────────────────────────────────────────────
    @Test
    fun `projectPointOntoSegment foot is perpendicular for a point beside the segment`() {
        // Horizontal segment along a constant latitude; point offset due north.
        val aLat = 32.80; val aLng = 34.98
        val bLat = 32.80; val bLng = 35.00
        val pLat = 32.801; val pLng = 34.99 // ~111 m north of the segment midpoint
        val proj = RoadSnapMath.projectPointOntoSegment(pLat, pLng, aLat, aLng, bLat, bLng)
        assertTrue("t should be mid-segment", proj.t in 0.4..0.6)
        assertEquals("snapped lat == segment lat", aLat, proj.snappedLat, 1e-6)
        // distance ≈ 0.001 deg lat * 111320 ≈ 111.3 m
        assertEquals(111.3, proj.distanceM, 2.0)
    }

    @Test
    fun `projectPointOntoSegment clamps beyond endpoint A`() {
        val proj = RoadSnapMath.projectPointOntoSegment(
            32.80, 34.97, /*P west of A*/
            32.80, 34.98, 32.80, 35.00
        )
        assertEquals(0.0, proj.t, 1e-9)
    }

    @Test
    fun `projectPointOntoSegment clamps beyond endpoint B`() {
        val proj = RoadSnapMath.projectPointOntoSegment(
            32.80, 35.01, /*P east of B*/
            32.80, 34.98, 32.80, 35.00
        )
        assertEquals(1.0, proj.t, 1e-9)
    }

    @Test
    fun `projectPointOntoSegment degenerate zero-length segment yields finite distance`() {
        val proj = RoadSnapMath.projectPointOntoSegment(
            32.801, 34.99, 32.80, 34.99, 32.80, 34.99
        )
        assertTrue(proj.distanceM.isFinite())
        assertEquals(0.0, proj.t, 1e-9)
    }

    // ── Segments blob → R-tree → nearestSegments ─────────────────────────────────
    @Test
    fun `nearestSegments returns the planted road nearest first`() {
        val bytes = writeSegments(
            listOf(
                Seg(1L, 100L, 6, 32.80, 34.98, 32.80, 35.00, "Herzl Street"),
                Seg(2L, 200L, 6, 32.81, 34.98, 32.81, 35.00, "Far Street")
            )
        )
        val store = SegmentRTreeReader.parse(bytes)
        assertEquals(2, store.count)
        val hits = store.nearestSegments(32.8005, 34.99, radiusM = 100.0, maxResults = 4)
        assertTrue("expected a hit within 100 m", hits.isNotEmpty())
        assertEquals("nearest is way 100", 100L, hits[0].wayId)
        assertEquals("Herzl Street", hits[0].name)
    }

    @Test
    fun `nearestSegments returns empty when no road is within radius`() {
        val bytes = writeSegments(
            listOf(Seg(1L, 100L, 6, 32.80, 34.98, 32.80, 35.00, "Herzl Street"))
        )
        val store = SegmentRTreeReader.parse(bytes)
        // ~1 km north → outside a 25 m radius.
        val hits = store.nearestSegments(32.809, 34.99, radiusM = 25.0, maxResults = 1)
        assertTrue(hits.isEmpty())
    }

    // ── Graph blob → Dijkstra route ──────────────────────────────────────────────
    @Test
    fun `route returns a connected polyline between two vertices`() {
        // 3 vertices in a line: 0 - 1 - 2 (two bidirectional edges).
        val bytes = writeGraph(
            vertices = listOf(
                doubleArrayOf(32.80, 34.98),
                doubleArrayOf(32.80, 34.99),
                doubleArrayOf(32.80, 35.00)
            ),
            edges = listOf(
                Edge(0, 1, 900.0, 6, oneway = false),
                Edge(1, 2, 900.0, 6, oneway = false)
            )
        )
        val store = RoutingGraphReader.parse(bytes)
        val poly = store.route(32.80, 34.980, 32.80, 35.000)
        assertEquals("polyline visits all 3 vertices", 3, poly.size)
        assertEquals(34.98, poly.first().lng, 1e-9)
        assertEquals(35.00, poly.last().lng, 1e-9)
    }

    @Test
    fun `route respects oneway (no reverse traversal)`() {
        // 0 -> 1 (oneway). A request from 1 to 0 must find no path.
        val bytes = writeGraph(
            vertices = listOf(doubleArrayOf(32.80, 34.98), doubleArrayOf(32.80, 34.99)),
            edges = listOf(Edge(0, 1, 900.0, 6, oneway = true))
        )
        val store = RoutingGraphReader.parse(bytes)
        val forward = store.route(32.80, 34.98, 32.80, 34.99)
        assertEquals(2, forward.size)
        val backward = store.route(32.80, 34.99, 32.80, 34.98)
        assertTrue("oneway: reverse route must be empty", backward.isEmpty())
    }

    // ── Geocode blob → ranked search ─────────────────────────────────────────────
    @Test
    fun `geocode resolves a prefix query and ranks an exact-token match first`() {
        val bytes = writeGeocode(
            places = listOf(
                Place(10L, 32.80, 34.99, "Technion, Haifa", listOf("technion", "haifa")),
                Place(11L, 32.79, 34.97, "Technical College", listOf("technical", "college"))
            )
        )
        val store = GeocodeIndexReader.parse(bytes)
        val hits = store.geocode("tech", 32.80, 34.99, limit = 5)
        assertTrue("prefix 'tech' matches both", hits.size >= 1)
        // Both are prefix matches; the one nearer the anchor (Technion) wins the tie.
        assertEquals(10L, hits[0].placeId)

        val exact = store.geocode("technion", 32.80, 34.99, limit = 5)
        assertEquals("exact token match", 10L, exact[0].placeId)

        val none = store.geocode("zzznotacity", 32.80, 34.99, limit = 5)
        assertTrue("out-of-corpus query is empty", none.isEmpty())
    }

    @Test
    fun `tokenize keeps hebrew and latin, strips punctuation`() {
        val t = GeocodeIndexReader.tokenize("Ben-Gurion Blvd., חיפה")
        assertTrue(t.contains("ben"))
        assertTrue(t.contains("gurion"))
        assertTrue(t.contains("blvd"))
        assertTrue("hebrew token survives", t.contains("חיפה"))
        assertFalse(t.contains(""))
    }

    // ── Synthetic blob writers (mirror OsmAssetFormat / the readers exactly) ──────

    private data class Seg(
        val segmentId: Long, val wayId: Long, val classCode: Int,
        val aLat: Double, val aLng: Double, val bLat: Double, val bLng: Double,
        val name: String?
    )

    private fun writeSegments(segs: List<Seg>): ByteArray {
        val pool = ArrayList<String>()
        fun nameIdx(n: String?): Int {
            if (n == null) return -1
            val i = pool.indexOf(n); if (i >= 0) return i
            pool.add(n); return pool.size - 1
        }
        val nameIdxs = segs.map { nameIdx(it.name) }
        val buf = newBuf()
        buf.putInt(OsmAssetFormat.MAGIC_SEGMENTS)
        buf.putInt(OsmAssetFormat.SCHEMA_VERSION)
        buf.putInt(segs.size)
        segs.forEachIndexed { i, s ->
            buf.putLong(s.segmentId); buf.putLong(s.wayId); buf.putInt(s.classCode)
            buf.putDouble(s.aLat); buf.putDouble(s.aLng); buf.putDouble(s.bLat); buf.putDouble(s.bLng)
            buf.putInt(nameIdxs[i])
        }
        writePool(buf, pool)
        return buf.array().copyOf(buf.position())
    }

    private data class Edge(val from: Int, val to: Int, val len: Double, val classCode: Int, val oneway: Boolean)

    private fun writeGraph(vertices: List<DoubleArray>, edges: List<Edge>): ByteArray {
        val buf = newBuf()
        buf.putInt(OsmAssetFormat.MAGIC_GRAPH)
        buf.putInt(OsmAssetFormat.SCHEMA_VERSION)
        buf.putInt(vertices.size)
        buf.putInt(edges.size)
        for (v in vertices) { buf.putDouble(v[0]); buf.putDouble(v[1]) }
        for (e in edges) {
            buf.putInt(e.from); buf.putInt(e.to); buf.putDouble(e.len)
            buf.putInt(e.classCode); buf.put(if (e.oneway) 1 else 0)
        }
        return buf.array().copyOf(buf.position())
    }

    private data class Place(
        val placeId: Long, val lat: Double, val lng: Double,
        val displayName: String, val tokens: List<String>
    )

    private fun writeGeocode(places: List<Place>): ByteArray {
        val pool = ArrayList<String>()
        fun nameIdx(n: String): Int { val i = pool.indexOf(n); if (i >= 0) return i; pool.add(n); return pool.size - 1 }
        val nameIdxs = places.map { nameIdx(it.displayName) }
        // token -> list of place indices
        val postings = LinkedHashMap<String, MutableList<Int>>()
        places.forEachIndexed { i, p -> for (tok in p.tokens) postings.getOrPut(tok) { ArrayList() }.add(i) }

        val buf = newBuf()
        buf.putInt(OsmAssetFormat.MAGIC_GEOCODE)
        buf.putInt(OsmAssetFormat.SCHEMA_VERSION)
        buf.putInt(places.size)
        places.forEachIndexed { i, p ->
            buf.putLong(p.placeId); buf.putDouble(p.lat); buf.putDouble(p.lng); buf.putInt(nameIdxs[i])
        }
        writePool(buf, pool)
        buf.putInt(postings.size)
        for ((tok, list) in postings) {
            val tb = tok.toByteArray(Charsets.UTF_8)
            buf.putInt(tb.size); buf.put(tb)
            buf.putInt(list.size); for (idx in list) buf.putInt(idx)
        }
        return buf.array().copyOf(buf.position())
    }

    private fun writePool(buf: ByteBuffer, pool: List<String>) {
        buf.putInt(pool.size)
        for (s in pool) { val b = s.toByteArray(Charsets.UTF_8); buf.putInt(b.size); buf.put(b) }
    }

    private fun newBuf(): ByteBuffer = ByteBuffer.allocate(1 shl 16).order(ByteOrder.LITTLE_ENDIAN)
}
