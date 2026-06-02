package com.example.navsight1

import java.util.PriorityQueue

/**
 * RoutingGraphReader — parses the `haifa_graph.bin` blob into a [GraphStore] and
 * provides an A* / Dijkstra shortest path over it (MAP_MATCHING_PLAN.md §8M Step
 * A* / K-routing*). This is the build-green, dependency-free v1 router (the
 * `DijkstraRouter` fallback the plan names): BRouter needs vendored jars that are
 * not on Maven Central, so it stays a deferred seam (see OsmRouter.kt).
 *
 * Blob layout (little-endian):
 *   [magic:int=NSGR][version:int][vertexCount:int][edgeCount:int]
 *   vertexCount × { lat:double, lng:double }
 *   edgeCount   × { fromIdx:int, toIdx:int, lenM:double, classCode:int, oneway:byte }
 */
internal object RoutingGraphReader {

    fun parse(bytes: ByteArray): GraphStore {
        val buf = OsmAssetFormat.reader(bytes)
        val magic = buf.int
        require(magic == OsmAssetFormat.MAGIC_GRAPH) {
            "graph blob: bad magic $magic (expected ${OsmAssetFormat.MAGIC_GRAPH})"
        }
        val version = buf.int
        require(version == OsmAssetFormat.SCHEMA_VERSION) {
            "graph blob: schema version $version != ${OsmAssetFormat.SCHEMA_VERSION}"
        }
        val vertexCount = buf.int
        val edgeCount = buf.int
        require(vertexCount in 0..MAX_VERTICES) { "graph blob: vertexCount out of range $vertexCount" }
        require(edgeCount in 0..MAX_EDGES) { "graph blob: edgeCount out of range $edgeCount" }

        val vLat = DoubleArray(vertexCount)
        val vLng = DoubleArray(vertexCount)
        for (i in 0 until vertexCount) {
            vLat[i] = buf.double
            vLng[i] = buf.double
        }

        // CSR-style adjacency: first count out-degree, then fill.
        val degree = IntArray(vertexCount)
        val eFrom = IntArray(edgeCount)
        val eTo = IntArray(edgeCount)
        val eLen = DoubleArray(edgeCount)
        val eOneway = BooleanArray(edgeCount)
        for (e in 0 until edgeCount) {
            val from = buf.int
            val to = buf.int
            val len = buf.double
            buf.int // classCode — unused by the v1 router (no per-class weighting yet)
            val oneway = buf.get().toInt() != 0
            require(from in 0 until vertexCount && to in 0 until vertexCount) {
                "graph blob: edge endpoint out of range ($from,$to)"
            }
            eFrom[e] = from; eTo[e] = to; eLen[e] = len; eOneway[e] = oneway
            degree[from]++
            if (!oneway) degree[to]++
        }

        return GraphStore(vertexCount, vLat, vLng, eFrom, eTo, eLen, eOneway, degree)
    }

    /**
     * Runtime factory: build a [GraphStore] from in-memory [RoadArrays] parsed from
     * a live Overpass response (RoadRegionManager). Computes the out-degrees then
     * reuses the GraphStore constructor's CSR build verbatim.
     */
    fun buildGraphStore(a: RoadArrays): GraphStore {
        val degree = IntArray(a.vertexCount)
        for (e in 0 until a.edgeCount) {
            degree[a.eFrom[e]]++
            if (!a.eOneway[e]) degree[a.eTo[e]]++
        }
        return GraphStore(a.vertexCount, a.vLat, a.vLng, a.eFrom, a.eTo, a.eLen, a.eOneway, degree)
    }

    private const val MAX_VERTICES = 50_000_000
    private const val MAX_EDGES = 200_000_000
}

/**
 * Immutable routing graph with a Dijkstra shortest-path query. Adjacency is built
 * once at construction as a flat CSR array for cache-friendly relaxation.
 */
internal class GraphStore(
    val vertexCount: Int,
    private val vLat: DoubleArray,
    private val vLng: DoubleArray,
    eFrom: IntArray,
    eTo: IntArray,
    eLen: DoubleArray,
    eOneway: BooleanArray,
    degree: IntArray
) {
    // CSR adjacency: adjStart[v]..adjStart[v+1] index into adjTo/adjLen.
    private val adjStart: IntArray = IntArray(vertexCount + 1)
    private val adjTo: IntArray
    private val adjLen: DoubleArray

    init {
        for (v in 0 until vertexCount) adjStart[v + 1] = adjStart[v] + degree[v]
        val total = adjStart[vertexCount]
        adjTo = IntArray(total)
        adjLen = DoubleArray(total)
        val cursor = adjStart.copyOf()
        for (e in eFrom.indices) {
            val a = eFrom[e]; val b = eTo[e]; val len = eLen[e]
            adjTo[cursor[a]] = b; adjLen[cursor[a]] = len; cursor[a]++
            if (!eOneway[e]) {
                adjTo[cursor[b]] = a; adjLen[cursor[b]] = len; cursor[b]++
            }
        }
    }

    // ---- Map-matching accessors (GraphRailDot / advance-along-rail, 2026-05-31) -------------
    // The ball-on-rail lives on a graph EDGE and walks the adjacency, so it needs read access
    // to vertex coordinates + per-vertex out-neighbours + a nearest-edge query. All additive,
    // no behaviour change to the router above.

    /** Vertex count (alias of [vertexCount] for the matcher). */
    val vCount: Int get() = vertexCount

    fun vLatAt(i: Int): Double = vLat[i]
    fun vLngAt(i: Int): Double = vLng[i]

    /** Out-degree of vertex [v] (directed; a oneway edge contributes only its allowed direction). */
    fun degreeOf(v: Int): Int = adjStart[v + 1] - adjStart[v]

    /** The k-th out-neighbour vertex of [v] (0 ≤ k < [degreeOf]). */
    fun neighborAt(v: Int, k: Int): Int = adjTo[adjStart[v] + k]

    /** Foot of the nearest graph edge to (lat,lng): the directed edge u→w, the along-edge
     *  fraction [t] ∈ [0,1], the foot lat/lng, and the metric offset. */
    class EdgeHit(
        val u: Int, val w: Int, val t: Double,
        val footLat: Double, val footLng: Double, val offsetM: Double
    )

    /** Nearest graph edge to (lat,lng). Scans the directed CSR (undirected edges are visited
     *  twice — harmless for a min) and projects the point onto each edge segment. Returns null
     *  only for an empty graph. O(E); called at (re)acquire, which is rare, not per frame. */
    fun nearestEdge(lat: Double, lng: Double): EdgeHit? {
        if (vertexCount == 0) return null
        var bU = -1; var bW = -1; var bT = 0.0; var bFLat = 0.0; var bFLng = 0.0
        var bestD = Double.POSITIVE_INFINITY
        for (u in 0 until vertexCount) {
            var k = adjStart[u]
            val end = adjStart[u + 1]
            while (k < end) {
                val w = adjTo[k]; k++
                val pr = RoadSnapMath.projectPointOntoSegment(
                    lat, lng, vLat[u], vLng[u], vLat[w], vLng[w]
                )
                if (pr.distanceM < bestD) {
                    bestD = pr.distanceM; bU = u; bW = w; bT = pr.t
                    bFLat = pr.snappedLat; bFLng = pr.snappedLng
                }
            }
        }
        if (bU < 0) return null
        return EdgeHit(bU, bW, bT, bFLat, bFLng, bestD)
    }

    /** Index of the graph vertex nearest to (lat,lng), or -1 if the graph is empty. */
    fun nearestVertex(lat: Double, lng: Double): Int {
        var best = -1
        var bestD = Double.POSITIVE_INFINITY
        for (v in 0 until vertexCount) {
            val d = RoadSnapMath.perpendicularDistanceM(lat, lng, vLat[v], vLng[v], vLat[v], vLng[v])
            if (d < bestD) { bestD = d; best = v }
        }
        return best
    }

    /**
     * Dijkstra shortest path from the vertex nearest `start` to the vertex
     * nearest `dest`. Returns the polyline of vertex coordinates (start vertex →
     * dest vertex), or an empty list if either endpoint is off-graph / unreachable.
     */
    fun route(startLat: Double, startLng: Double, destLat: Double, destLng: Double): List<GeoPt> {
        if (vertexCount == 0) return emptyList()
        val src = nearestVertex(startLat, startLng)
        val dst = nearestVertex(destLat, destLng)
        if (src < 0 || dst < 0) return emptyList()
        if (src == dst) return listOf(GeoPt(vLat[src], vLng[src]))

        val dist = DoubleArray(vertexCount) { Double.POSITIVE_INFINITY }
        val prev = IntArray(vertexCount) { -1 }
        dist[src] = 0.0
        val pq = PriorityQueue<LongArray>(compareBy { java.lang.Double.longBitsToDouble(it[1]) })
        pq.add(longArrayOf(src.toLong(), java.lang.Double.doubleToLongBits(0.0)))

        while (pq.isNotEmpty()) {
            val top = pq.poll()
            val u = top[0].toInt()
            val d = java.lang.Double.longBitsToDouble(top[1])
            if (d > dist[u]) continue
            if (u == dst) break
            var k = adjStart[u]
            val end = adjStart[u + 1]
            while (k < end) {
                val v = adjTo[k]
                val nd = d + adjLen[k]
                if (nd < dist[v]) {
                    dist[v] = nd
                    prev[v] = u
                    pq.add(longArrayOf(v.toLong(), java.lang.Double.doubleToLongBits(nd)))
                }
                k++
            }
        }

        if (dist[dst].isInfinite()) return emptyList()
        // Reconstruct path src..dst.
        val rev = ArrayList<Int>()
        var cur = dst
        while (cur != -1) { rev.add(cur); if (cur == src) break; cur = prev[cur] }
        if (rev.isEmpty() || rev.last() != src) return emptyList()
        val out = ArrayList<GeoPt>(rev.size)
        for (i in rev.indices.reversed()) {
            val v = rev[i]
            out.add(GeoPt(vLat[v], vLng[v]))
        }
        return out
    }
}
