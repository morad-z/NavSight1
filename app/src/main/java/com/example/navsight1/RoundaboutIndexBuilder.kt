package com.example.navsight1

/**
 * Builds a [RoundaboutIndex] from the parsed road region ([RoadArrays]). Groups
 * `junction=roundabout` ring ways, computes each ring's centre + radius, and finds
 * the exit roads (segments of OTHER ways that touch a ring node) with their
 * outbound bearings — the candidates [ManeuverStateMachine] picks "which exit" from.
 */
internal object RoundaboutIndexBuilder {

    private const val MIN_RING_RADIUS_M = 3.0   // mini-roundabout floor
    private const val MAX_RING_RADIUS_M = 80.0  // reject giant loop roads / tagging errors

    fun build(a: RoadArrays): RoundaboutIndex {
        if (a.segCount == 0) return RoundaboutIndex(emptyList())

        // node key (quantized lat,lng — same scheme as the parser's vertexOf) → segment endpoints.
        // endpoint encoded as segIndex*2 + (0 = endpoint A, 1 = endpoint B).
        fun key(lat: Double, lng: Double): Long {
            val la = Math.round(lat * 1e7); val ln = Math.round(lng * 1e7)
            return (la shl 32) xor (ln and 0xffffffffL)
        }
        val nodeToSeg = HashMap<Long, MutableList<Int>>()
        for (i in 0 until a.segCount) {
            nodeToSeg.getOrPut(key(a.aLat[i], a.aLng[i])) { ArrayList() }.add(i * 2)
            nodeToSeg.getOrPut(key(a.bLat[i], a.bLng[i])) { ArrayList() }.add(i * 2 + 1)
        }

        // ring segments grouped by way id
        val ringWays = HashMap<Long, MutableList<Int>>()
        for (i in 0 until a.segCount) if (a.junction[i] != null) ringWays.getOrPut(a.wayId[i]) { ArrayList() }.add(i)

        val rings = ArrayList<RoundaboutRing>()
        for ((wayId, segs) in ringWays) {
            val nodeKeys = LinkedHashSet<Long>()
            val nodeCoords = HashMap<Long, DoubleArray>()
            for (i in segs) {
                val ka = key(a.aLat[i], a.aLng[i]); nodeKeys.add(ka); nodeCoords[ka] = doubleArrayOf(a.aLat[i], a.aLng[i])
                val kb = key(a.bLat[i], a.bLng[i]); nodeKeys.add(kb); nodeCoords[kb] = doubleArrayOf(a.bLat[i], a.bLng[i])
            }
            val coords = nodeKeys.mapNotNull { nodeCoords[it] }
            if (coords.isEmpty()) continue
            val cLat = coords.map { it[0] }.average()
            val cLng = coords.map { it[1] }.average()
            val radius = coords.map { RoadSnapMath.haversineM(cLat, cLng, it[0], it[1]) }.average()
            if (radius < MIN_RING_RADIUS_M || radius > MAX_RING_RADIUS_M) continue

            // exit roads: segments of OTHER (non-ring) ways touching a ring node.
            val exitByWay = LinkedHashMap<Long, ExitRoad>()
            for (nk in nodeKeys) {
                val touching = nodeToSeg[nk] ?: continue
                val nc = nodeCoords[nk] ?: continue
                for (enc in touching) {
                    val si = enc / 2
                    val aIsNode = (enc % 2 == 0)
                    if (a.wayId[si] == wayId || a.junction[si] != null) continue // own ring / another ring
                    val oLat = if (aIsNode) a.bLat[si] else a.aLat[si]
                    val oLng = if (aIsNode) a.bLng[si] else a.aLng[si]
                    val bearing = ManeuverMath.bearingDeg(nc[0], nc[1], oLat, oLng)
                    exitByWay.putIfAbsent(a.wayId[si], ExitRoad(a.wayId[si], nc[0], nc[1], bearing))
                }
            }
            if (exitByWay.size < 2) continue // incomplete OSM data → don't emit; SM degrades to FREE_ROAD
            rings.add(RoundaboutRing(wayId, cLat, cLng, radius, exitByWay.values.toList(), coords))
        }
        return RoundaboutIndex(rings)
    }
}
