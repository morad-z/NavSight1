package com.example.navsight1

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt

/**
 * RoadSnapMath — pure, Android-free geometry for snapping a point onto a road
 * segment (MAP_MATCHING_PLAN.md §8M Step C*, step 2). No Android deps so it is
 * unit-testable on the JVM.
 *
 * All work is done in a LOCAL east-north metric tangent plane anchored at the
 * query point, so the perpendicular projection and distance are true metres at
 * city scale (the small-angle equirectangular approximation is exact enough for
 * Haifa — the inverse of Step B*'s VIO→lat/lng projection, same EARTH model).
 */
internal object RoadSnapMath {

    /** Metres per degree of latitude (WGS84 mean). Matches the §8M Step C* spec. */
    const val M_PER_DEG_LAT = 111_320.0

    data class Projection(
        /** Foot-of-perpendicular latitude (degrees). */
        val snappedLat: Double,
        /** Foot-of-perpendicular longitude (degrees). */
        val snappedLng: Double,
        /** True metric distance from the query point to the foot, in metres. */
        val distanceM: Double,
        /** Parameter along A→B in [0,1] of the foot (0 = at A, 1 = at B). */
        val t: Double
    )

    /**
     * Project query point P = (pLat, pLng) onto the finite segment A→B, all in
     * degrees. Returns the foot-of-perpendicular (clamped to the segment, which
     * is exactly the corner-cutting source the deferred Step D HMM later removes),
     * its metric distance from P, and the along-segment parameter t.
     */
    fun projectPointOntoSegment(
        pLat: Double, pLng: Double,
        aLat: Double, aLng: Double,
        bLat: Double, bLng: Double
    ): Projection {
        val mPerDegLng = M_PER_DEG_LAT * cos(Math.toRadians(pLat))

        // A, B relative to P (the origin), in local metres (east = x, north = y).
        val ax = (aLng - pLng) * mPerDegLng
        val ay = (aLat - pLat) * M_PER_DEG_LAT
        val bx = (bLng - pLng) * mPerDegLng
        val by = (bLat - pLat) * M_PER_DEG_LAT

        val dx = bx - ax
        val dy = by - ay
        val len2 = dx * dx + dy * dy

        // Clamp t to [0,1] so the foot stays on the finite segment.
        val t = if (len2 < 1e-9) 0.0 else clamp(-(ax * dx + ay * dy) / len2, 0.0, 1.0)

        val fx = ax + t * dx
        val fy = ay + t * dy
        val distanceM = hypot(fx, fy)

        // Convert the foot back to lat/lng.
        val snappedLat = pLat + fy / M_PER_DEG_LAT
        val snappedLng = pLng + fx / mPerDegLng

        return Projection(snappedLat, snappedLng, distanceM, t)
    }

    /**
     * Perpendicular metric distance from P to segment A→B only (no foot). Used by
     * the R-tree candidate sort where the projected point is not yet needed.
     */
    fun perpendicularDistanceM(
        pLat: Double, pLng: Double,
        aLat: Double, aLng: Double,
        bLat: Double, bLng: Double
    ): Double {
        val mPerDegLng = M_PER_DEG_LAT * cos(Math.toRadians(pLat))
        val ax = (aLng - pLng) * mPerDegLng
        val ay = (aLat - pLat) * M_PER_DEG_LAT
        val bx = (bLng - pLng) * mPerDegLng
        val by = (bLat - pLat) * M_PER_DEG_LAT
        val dx = bx - ax
        val dy = by - ay
        val len2 = dx * dx + dy * dy
        val t = if (len2 < 1e-9) 0.0 else clamp(-(ax * dx + ay * dy) / len2, 0.0, 1.0)
        val fx = ax + t * dx
        val fy = ay + t * dy
        return hypot(fx, fy)
    }

    /** Haversine great-circle distance in metres. Mirrors SnappedLatLng.distanceTo. */
    fun haversineM(lat1: Double, lng1: Double, lat2: Double, lng2: Double): Double {
        val earthRadius = 6371000.0
        val dLat = Math.toRadians(lat2 - lat1)
        val dLng = Math.toRadians(lng2 - lng1)
        val a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
            Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
            Math.sin(dLng / 2) * Math.sin(dLng / 2)
        val c = 2 * Math.atan2(sqrt(a), sqrt(1 - a))
        return earthRadius * c
    }

    private fun clamp(v: Double, lo: Double, hi: Double): Double =
        if (v < lo) lo else if (v > hi) hi else v
}
