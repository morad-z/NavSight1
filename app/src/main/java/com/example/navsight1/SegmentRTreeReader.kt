package com.example.navsight1

import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.sqrt

/**
 * SegmentRTreeReader — parses the flat `haifa_segments.bin` blob into a
 * column-oriented [SegmentStore] and builds an in-memory STR-packed R-tree over
 * the segment AABBs for window queries (MAP_MATCHING_PLAN.md §8M Step A* / C*).
 *
 * Blob layout (little-endian, see OsmAssetFormat):
 *   [magic:int=NSSG][version:int][count:int]
 *   count × { segmentId:long, wayId:long, classCode:int,
 *             aLat:double, aLng:double, bLat:double, bLng:double, nameIdx:int }
 *   <string pool>  (way names; nameIdx references it, -1 = unnamed)
 */
internal object SegmentRTreeReader {

    private const val NODE_CAP = 16

    fun parse(bytes: ByteArray): SegmentStore {
        val buf = OsmAssetFormat.reader(bytes)
        val magic = buf.int
        require(magic == OsmAssetFormat.MAGIC_SEGMENTS) {
            "segments blob: bad magic $magic (expected ${OsmAssetFormat.MAGIC_SEGMENTS})"
        }
        val version = buf.int
        require(version == OsmAssetFormat.SCHEMA_VERSION) {
            "segments blob: schema version $version != ${OsmAssetFormat.SCHEMA_VERSION}"
        }
        val count = buf.int
        require(count in 0..MAX_SEGMENTS) { "segments blob: count out of range $count" }

        val segmentId = LongArray(count)
        val wayId = LongArray(count)
        val classCode = IntArray(count)
        val aLat = DoubleArray(count)
        val aLng = DoubleArray(count)
        val bLat = DoubleArray(count)
        val bLng = DoubleArray(count)
        val nameIdx = IntArray(count)

        for (i in 0 until count) {
            segmentId[i] = buf.long
            wayId[i] = buf.long
            classCode[i] = buf.int
            aLat[i] = buf.double
            aLng[i] = buf.double
            bLat[i] = buf.double
            bLng[i] = buf.double
            nameIdx[i] = buf.int
        }
        val pool = StringPool.read(buf)

        val store = SegmentStore(
            count, segmentId, wayId, classCode, aLat, aLng, bLat, bLng, nameIdx, pool
        )
        // Build the tree from the LOCAL arrays (not the store's private fields).
        store.rtreeRoot = if (count == 0) null else buildStrTree(count, aLat, aLng, bLat, bLng)
        return store
    }

    /**
     * Runtime factory: build a [SegmentStore] (with STR R-tree) from in-memory
     * [RoadArrays] parsed from a live Overpass response (RoadRegionManager). Same
     * tree-build path as [parse]; names are kept inline (no string pool).
     */
    fun buildSegmentStore(a: RoadArrays): SegmentStore {
        val store = SegmentStore(
            a.segCount, a.segmentId, a.wayId, a.classCode,
            a.aLat, a.aLng, a.bLat, a.bLng,
            IntArray(0), emptyArray(), a.segName
        )
        store.rtreeRoot =
            if (a.segCount == 0) null else buildStrTree(a.segCount, a.aLat, a.aLng, a.bLat, a.bLng)
        return store
    }

    // ── STR (Sort-Tile-Recursive) bulk-load of a static, read-only R-tree ───────
    private fun buildStrTree(
        count: Int, aLat: DoubleArray, aLng: DoubleArray, bLat: DoubleArray, bLng: DoubleArray
    ): RNode {
        // Leaf entries: one per segment with its AABB + centroid.
        var nodes: List<RNode> = (0 until count).map { i ->
            val minLat = minOf(aLat[i], bLat[i])
            val maxLat = maxOf(aLat[i], bLat[i])
            val minLng = minOf(aLng[i], bLng[i])
            val maxLng = maxOf(aLng[i], bLng[i])
            RNode(minLat, minLng, maxLat, maxLng, children = null, segIndices = intArrayOf(i))
        }
        // Pack repeatedly until a single root remains.
        while (nodes.size > 1) {
            nodes = packLevel(nodes)
        }
        return nodes[0]
    }

    private fun packLevel(input: List<RNode>): List<RNode> {
        val n = input.size
        val cap = NODE_CAP
        val leafCount = ceil(n.toDouble() / cap).toInt()
        val slices = ceil(sqrt(leafCount.toDouble())).toInt().coerceAtLeast(1)
        val perSlice = slices * cap

        // Sort by centroid longitude (x).
        val byX = input.sortedBy { it.cLng() }
        val out = ArrayList<RNode>(leafCount)
        var s = 0
        while (s < n) {
            val sliceEnd = minOf(s + perSlice, n)
            val slice = byX.subList(s, sliceEnd).sortedBy { it.cLat() } // sort slice by y
            var i = 0
            while (i < slice.size) {
                val end = minOf(i + cap, slice.size)
                out.add(group(slice.subList(i, end)))
                i = end
            }
            s = sliceEnd
        }
        return out
    }

    private fun group(children: List<RNode>): RNode {
        var minLat = Double.POSITIVE_INFINITY
        var minLng = Double.POSITIVE_INFINITY
        var maxLat = Double.NEGATIVE_INFINITY
        var maxLng = Double.NEGATIVE_INFINITY
        for (c in children) {
            if (c.minLat < minLat) minLat = c.minLat
            if (c.minLng < minLng) minLng = c.minLng
            if (c.maxLat > maxLat) maxLat = c.maxLat
            if (c.maxLng > maxLng) maxLng = c.maxLng
        }
        return RNode(minLat, minLng, maxLat, maxLng, children = children, segIndices = null)
    }

    private const val MAX_SEGMENTS = 50_000_000

    internal class RNode(
        val minLat: Double, val minLng: Double, val maxLat: Double, val maxLng: Double,
        val children: List<RNode>?,
        val segIndices: IntArray?
    ) {
        fun cLat() = (minLat + maxLat) * 0.5
        fun cLng() = (minLng + maxLng) * 0.5
    }
}

/**
 * Column-oriented store of parsed segments plus the STR-packed R-tree. Holds the
 * runtime query [nearestSegments]; produced once by [SegmentRTreeReader.parse].
 */
internal class SegmentStore(
    val count: Int,
    private val segmentId: LongArray,
    private val wayId: LongArray,
    private val classCode: IntArray,
    private val aLat: DoubleArray,
    private val aLng: DoubleArray,
    private val bLat: DoubleArray,
    private val bLng: DoubleArray,
    private val nameIdx: IntArray,
    private val namePool: Array<String>,
    // Runtime dynamic-region path (RoadRegionManager) keeps way names inline instead
    // of via the (nameIdx, namePool) blob pool. Null on the blob path. See buildFrom.
    private val nameInline: Array<String?>? = null
) {
    internal var rtreeRoot: SegmentRTreeReader.RNode? = null

    /**
     * R-tree window query: all segments whose AABB (expanded by [radiusM]) could
     * contain a foot within [radiusM] of (lat,lng), re-ranked by TRUE
     * perpendicular distance and truncated to [maxResults]. Returns nearest-first.
     */
    fun nearestSegments(lat: Double, lng: Double, radiusM: Double, maxResults: Int): List<RoadSegment> {
        val root = rtreeRoot ?: return emptyList()
        if (maxResults <= 0 || radiusM <= 0.0) return emptyList()

        // Convert the metric radius to a degree window (latitude + longitude scale).
        val dLat = radiusM / RoadSnapMath.M_PER_DEG_LAT
        val mPerDegLng = RoadSnapMath.M_PER_DEG_LAT * cos(Math.toRadians(lat))
        val dLng = if (mPerDegLng > 1e-6) radiusM / mPerDegLng else radiusM / 1e-6
        val qMinLat = lat - dLat; val qMaxLat = lat + dLat
        val qMinLng = lng - dLng; val qMaxLng = lng + dLng

        // Collect candidate segment indices whose AABB intersects the query window.
        val candidates = ArrayList<Int>(64)
        collect(root, qMinLat, qMinLng, qMaxLat, qMaxLng, candidates)

        // Re-rank by TRUE perpendicular distance; filter to radius.
        val scored = ArrayList<Pair<Int, Double>>(candidates.size)
        for (i in candidates) {
            val d = RoadSnapMath.perpendicularDistanceM(
                lat, lng, aLat[i], aLng[i], bLat[i], bLng[i]
            )
            if (d <= radiusM) scored.add(i to d)
        }
        scored.sortBy { it.second }

        val limit = minOf(maxResults, scored.size)
        val out = ArrayList<RoadSegment>(limit)
        for (k in 0 until limit) {
            val i = scored[k].first
            out.add(
                RoadSegment(
                    segmentId = segmentId[i],
                    wayId = wayId[i],
                    highwayClass = OsmAssetFormat.highwayClassName(classCode[i]),
                    aLat = aLat[i], aLng = aLng[i], bLat = bLat[i], bLng = bLng[i],
                    name = if (nameInline != null) nameInline.getOrNull(i)
                           else StringPool.resolve(namePool, nameIdx[i])
                )
            )
        }
        return out
    }

    private fun collect(
        node: SegmentRTreeReader.RNode,
        qMinLat: Double, qMinLng: Double, qMaxLat: Double, qMaxLng: Double,
        out: ArrayList<Int>
    ) {
        // AABB-vs-query intersection test.
        if (node.maxLat < qMinLat || node.minLat > qMaxLat ||
            node.maxLng < qMinLng || node.minLng > qMaxLng
        ) return

        val segs = node.segIndices
        if (segs != null) {
            for (i in segs) out.add(i)
            return
        }
        val children = node.children ?: return
        for (c in children) collect(c, qMinLat, qMinLng, qMaxLat, qMaxLng, out)
    }
}
