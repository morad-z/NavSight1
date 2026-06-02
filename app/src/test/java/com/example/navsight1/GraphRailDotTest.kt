package com.example.navsight1

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

/**
 * Unit tests for [GraphRailDot] — the "ball in a maze" display constraint. These pin the invariants
 * the owner demanded: the ball ALWAYS stays on a road (graph edge), it advances ALONG its road, it
 * only ever moves to a CONNECTED edge at a junction, and it NEVER jumps to a disconnected parallel
 * road ("can't jump through the wall"). Pure JVM (synthetic GraphStore, no Android / road region).
 */
class GraphRailDotTest {

    // ~10 m east-west spacing and ~30 m north offset, in degrees near Haifa.
    private val lat0 = 32.79
    private val lng0 = 35.00
    private val dLng = 10.0 / (RoadSnapMath.M_PER_DEG_LAT * Math.cos(Math.toRadians(lat0)))  // ~10 m east
    private val dLatB = 30.0 / RoadSnapMath.M_PER_DEG_LAT                                     // ~30 m north

    /** Build a GraphStore from vertices + undirected/oneway edges (lengths via haversine). */
    private fun buildGraph(
        verts: List<Pair<Double, Double>>,
        edges: List<Triple<Int, Int, Boolean>>   // (from, to, oneway)
    ): GraphStore {
        val n = verts.size
        val vLat = DoubleArray(n) { verts[it].first }
        val vLng = DoubleArray(n) { verts[it].second }
        val m = edges.size
        val eFrom = IntArray(m) { edges[it].first }
        val eTo = IntArray(m) { edges[it].second }
        val eOneway = BooleanArray(m) { edges[it].third }
        val eLen = DoubleArray(m) { RoadSnapMath.haversineM(vLat[eFrom[it]], vLng[eFrom[it]], vLat[eTo[it]], vLng[eTo[it]]) }
        val degree = IntArray(n)
        for (e in 0 until m) { degree[eFrom[e]]++; if (!eOneway[e]) degree[eTo[e]]++ }
        return GraphStore(n, vLat, vLng, eFrom, eTo, eLen, eOneway, degree)
    }

    /** Road A (0-1-2) going east at lat0; a DISCONNECTED parallel Road B (3-4-5) 30 m north. */
    private fun twoParallelRoads(): GraphStore = buildGraph(
        verts = listOf(
            lat0 to lng0, lat0 to lng0 + dLng, lat0 to lng0 + 2 * dLng,          // Road A
            lat0 + dLatB to lng0, lat0 + dLatB to lng0 + dLng, lat0 + dLatB to lng0 + 2 * dLng  // Road B
        ),
        edges = listOf(
            Triple(0, 1, false), Triple(1, 2, false),   // Road A (bidirectional)
            Triple(3, 4, false), Triple(4, 5, false)    // Road B (bidirectional, NOT linked to A)
        )
    )

    @Test
    fun `ball never jumps to a disconnected parallel road even when pushed toward it`() {
        val rail = GraphRailDot(twoParallelRoads())
        assertTrue(rail.acquire(lat0, lng0 + 0.5 * dLng, preferBearingDeg = 90.0))  // on Road A, heading east

        // Drive forward repeatedly with a bearing pointing NE (45°, toward Road B). Road B is not
        // connected, so the ball must stay on Road A — its latitude can never climb toward B.
        repeat(40) {
            val p = rail.advance(distM = 8.0, bearingDeg = 45.0)
            assertNotNull(p)
            // Stays on Road A's latitude (within 2 m), nowhere near Road B (30 m north).
            val northM = abs(p!!.lat - lat0) * RoadSnapMath.M_PER_DEG_LAT
            assertTrue("ball drifted $northM m north toward the wall (Road B)", northM < 2.0)
        }
    }

    /** Straight road 0-1-2 (east) with a north branch 1-3 at the middle junction. */
    private fun tJunction(): GraphStore = buildGraph(
        verts = listOf(
            lat0 to lng0, lat0 to lng0 + dLng, lat0 to lng0 + 2 * dLng,   // straight east
            lat0 + dLatB to lng0 + dLng                                   // north branch off vertex 1
        ),
        edges = listOf(Triple(0, 1, false), Triple(1, 2, false), Triple(1, 3, false))
    )

    @Test
    fun `ball continues straight through a junction when travel bearing is straight`() {
        val rail = GraphRailDot(tJunction())
        assertTrue(rail.acquire(lat0, lng0 + 0.1 * dLng, preferBearingDeg = 90.0))
        // Advance ~15 m east through the junction at vertex 1 with an EAST bearing → must reach the
        // far straight vertex 2 (lng increases, lat unchanged), NOT turn north onto the branch.
        var p = rail.advance(distM = 15.0, bearingDeg = 90.0)!!
        assertEquals("should stay on the straight (east) road", lat0, p.lat, dLatB * 0.25)
        assertTrue("should have advanced east past the junction", p.lng > lng0 + dLng)
    }

    @Test
    fun `ball turns onto the connected branch when the travel bearing indicates the turn`() {
        val rail = GraphRailDot(tJunction())
        assertTrue(rail.acquire(lat0, lng0 + 0.1 * dLng, preferBearingDeg = 90.0))
        // A SINGLE advance that crosses the junction at vertex 1 carrying a NORTH (0°) bearing: the
        // ball reaches v1 (mid-edge motion ignores bearing), then at the junction turns onto the
        // connected north branch 1-3. (Junction choice uses the bearing of the advance that reaches it.)
        val p = rail.advance(distM = 15.0, bearingDeg = 0.0)!!
        val northM = (p.lat - lat0) * RoadSnapMath.M_PER_DEG_LAT
        assertTrue("ball should have turned north onto the branch (got ${northM} m)", northM > 5.0)
    }

    @Test
    fun `reverse() flips the facing so the ball rides the other way (U-turn or wrong facing)`() {
        // Acquire facing WEST, then reverse() (the caller does this on a confirmed/debounced U-turn or a
        // wrong initial facing). After reverse() the ball must advance EAST. Replaces the old per-tick
        // auto-flip, which flipped on noise; the caller now debounces and calls reverse() explicitly.
        val rail = GraphRailDot(buildGraph(
            verts = listOf(lat0 to lng0, lat0 to lng0 + dLng, lat0 to lng0 + 2 * dLng),
            edges = listOf(Triple(0, 1, false), Triple(1, 2, false))   // bidirectional east-west
        ))
        assertTrue(rail.acquire(lat0, lng0 + dLng, preferBearingDeg = 270.0))   // acquired facing WEST
        // Facing west: advancing moves WEST (no auto-flip anymore).
        val startLng = rail.position()!!.lng
        rail.advance(distM = 3.0, bearingDeg = 90.0)
        assertTrue("without reverse(), a west-facing ball moves west", rail.position()!!.lng <= startLng + 1e-9)
        // reverse() flips it east (the road is two-way), then it advances east.
        assertTrue("reverse() succeeds on a two-way road", rail.reverse())
        val afterRevLng = rail.position()!!.lng
        repeat(5) { rail.advance(distM = 3.0, bearingDeg = 90.0) }
        assertTrue("after reverse() the ball rides EAST", rail.position()!!.lng > afterRevLng)
    }

    @Test
    fun `dead-end reverses along the road instead of leaving the graph`() {
        // Single edge 0-1 (east). Advancing well past the end must keep the ball ON the edge.
        val rail = GraphRailDot(buildGraph(
            verts = listOf(lat0 to lng0, lat0 to lng0 + dLng),
            edges = listOf(Triple(0, 1, false))
        ))
        assertTrue(rail.acquire(lat0, lng0, preferBearingDeg = 90.0))
        val p = rail.advance(distM = 100.0, bearingDeg = 90.0)   // 10× the edge length
        assertNotNull("ball must stay on the road, not fall off the graph", p)
        // Still exactly on the (constant-latitude) edge, within numerical noise.
        assertTrue(abs(p!!.lat - lat0) * RoadSnapMath.M_PER_DEG_LAT < 0.5)
        assertTrue("ball must stay within the single edge's lng span",
            p.lng >= lng0 - 1e-9 && p.lng <= lng0 + dLng + 1e-9)
    }

    @Test
    fun `oneway against travel direction advances on the legal road instead of stranding`() {
        // One-way chain 0→1→2 going east. The user is travelling WEST (against it). acquire must NOT
        // reverse into the forbidden direction (which would strand + oscillate); it keeps the legal
        // east direction so the ball still advances along the real road (review HIGH).
        val rail = GraphRailDot(buildGraph(
            verts = listOf(lat0 to lng0, lat0 to lng0 + dLng, lat0 to lng0 + 2 * dLng),
            edges = listOf(Triple(0, 1, true), Triple(1, 2, true))   // oneway east
        ))
        assertTrue(rail.acquire(lat0, lng0 + 0.5 * dLng, preferBearingDeg = 270.0))  // user heading west
        val startLng = rail.position()!!.lng
        val p = rail.advance(distM = 15.0, bearingDeg = 270.0)
        assertNotNull("ball must stay on the legal road, not strand", p)
        assertTrue("ball must advance along the legal (east) direction, not oscillate in place",
            p!!.lng > startLng + 0.3 * dLng)
        assertTrue(abs(p.lat - lat0) * RoadSnapMath.M_PER_DEG_LAT < 1.0)
    }

    @Test
    fun `degenerate zero-length cycle does not hang and releases the lock`() {
        // Two distinct vertices at the SAME coordinate (zero-length bidirectional edge) — a malformed-OSM
        // case. advance must not spin forever; the hop backstop drops the lock so the next tick re-acquires.
        val rail = GraphRailDot(buildGraph(
            verts = listOf(lat0 to lng0, lat0 to lng0),   // coincident
            edges = listOf(Triple(0, 1, false))
        ))
        assertTrue(rail.acquire(lat0, lng0, preferBearingDeg = 90.0))
        val p = rail.advance(distM = 50.0, bearingDeg = 90.0)   // must return (not hang)
        assertNull("stuck on a degenerate component → lock released, position null", p)
        assertFalse(rail.acquired)
    }

    @Test
    fun `acquire fails and stays unacquired when no road is within range`() {
        val rail = GraphRailDot(twoParallelRoads())
        // 500 m north of either road → beyond MAX_ACQUIRE_M (30 m): must refuse to snap.
        val far = lat0 + 500.0 / RoadSnapMath.M_PER_DEG_LAT
        assertFalse(rail.acquire(far, lng0, preferBearingDeg = 0.0))
        assertFalse(rail.acquired)
        assertNull(rail.position())
        assertNull("advance on an unacquired ball returns null", rail.advance(5.0, 90.0))
    }
}
