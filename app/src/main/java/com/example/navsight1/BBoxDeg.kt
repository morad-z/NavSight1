package com.example.navsight1

/**
 * Geographic AABB in degrees, [minLon, minLat, maxLon, maxLat] order (matches the
 * manifest bbox + the Overpass query box). No Android/gms dependency.
 * Used by the dynamic road-region loader (RoadRegionManager).
 */
internal data class BBoxDeg(
    val minLon: Double, val minLat: Double,
    val maxLon: Double, val maxLat: Double
) {
    fun contains(lat: Double, lng: Double): Boolean =
        lng in minLon..maxLon && lat in minLat..maxLat

    /** Shrunk on every side by [marginDeg] — the hysteresis "inner zone". */
    fun containsWithMargin(lat: Double, lng: Double, marginDeg: Double): Boolean =
        lng in (minLon + marginDeg)..(maxLon - marginDeg) &&
            lat in (minLat + marginDeg)..(maxLat - marginDeg)

    companion object {
        /** A box of half-side [halfDeg] centred on (lat,lng). */
        fun centred(lat: Double, lng: Double, halfDeg: Double): BBoxDeg =
            BBoxDeg(lng - halfDeg, lat - halfDeg, lng + halfDeg, lat + halfDeg)
    }
}
