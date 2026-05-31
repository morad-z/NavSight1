package com.example.navsight1

/**
 * Roundabout model for the heading-aware map-matcher (MAP_MATCHING_PLAN.md §0.7).
 * Built from the live OSM road region (junction=roundabout ways) by
 * [RoundaboutIndexBuilder]; consumed by [ManeuverStateMachine].
 */

/** A road that connects to a roundabout ring, with the OUTBOUND bearing (0=N, CW) of
 *  its first off-ring segment — used to pick "which exit" by heading. */
data class ExitRoad(
    val wayId: Long,
    val nodeLat: Double, val nodeLng: Double,
    val bearingDeg: Double
)

/** A single roundabout: the ring way + centre + radius + its exit roads. */
data class RoundaboutRing(
    val wayId: Long,
    val centerLat: Double, val centerLng: Double,
    val radiusM: Double,
    val exits: List<ExitRoad>,
    val ringNodes: List<DoubleArray> // each = [lat, lng]
)

/** All roundabouts in the current region. Linear scan — a region has at most a
 *  handful of roundabouts, so no spatial index is needed. */
internal class RoundaboutIndex(val rings: List<RoundaboutRing>) {

    /** The roundabout whose centre is nearest the point AND within radius+[marginM],
     *  or null if the point is on no roundabout. */
    fun nearestRing(lat: Double, lng: Double, marginM: Double): RoundaboutRing? {
        var best: RoundaboutRing? = null
        var bestD = Double.POSITIVE_INFINITY
        for (r in rings) {
            val d = RoadSnapMath.haversineM(lat, lng, r.centerLat, r.centerLng)
            if (d <= r.radiusM + marginM && d < bestD) { bestD = d; best = r }
        }
        return best
    }

    fun contains(wayId: Long): Boolean = rings.any { it.wayId == wayId }

    val size: Int get() = rings.size
}
