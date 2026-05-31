package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import kotlin.math.abs
import kotlin.math.exp

/**
 * LocalMatcher — fully on-device trajectory map-matcher (Newson & Krumm 2009 HMM /
 * Viterbi), the OFFLINE replacement for the OSRM `/match` URL. Runs against the locally
 * loaded OSM road graph ([DynamicRoadRegion.segments] R-tree). No network, no rate
 * limits, no 10-coordinate "TooBig" cap, no latency, jamming-immune — and it returns the
 * same [LiveMatcher.MatchResult] shape so it is a drop-in for the dot source.
 *
 * Per window point we take the nearby road segments as candidates (R-tree) and project
 * the point onto each. Viterbi then picks the sequence of candidates that is both:
 *   - close to the trace        — EMISSION: Gaussian on the perpendicular distance, and
 *   - geometrically consistent  — TRANSITION: Newson-Krumm |straight-line move − along-road
 *                                 move|, plus a small bonus for staying on the same OSM way.
 *
 * For the dense VIO window (points ~0.5 s / a few metres apart) the along-road move is
 * well-approximated by the straight-line distance between consecutive projections, so no
 * per-pair graph routing is needed — the emission keeps the dot on the nearest road and the
 * transition stops it flickering between parallel roads or corner-cutting curves.
 *
 * The Viterbi core ([matchCandidates]) is a pure function (candidates in → result out), so
 * it is unit-testable without building a road region.
 */
class LocalMatcher {

    /** One projected road candidate for a trace point. */
    data class Candidate(
        val lat: Double, val lng: Double,
        val offsetM: Double,   // perpendicular distance from the trace point to this road
        val wayId: Long
    )

    /** Extract candidates from the loaded region and map-match [window] (oldest→newest).
     *  internal: DynamicRoadRegion is an internal type (same-module callers only). */
    internal fun match(window: List<LatLng>, region: DynamicRoadRegion?): LiveMatcher.MatchResult? {
        if (region == null || window.size < MIN_POINTS) return null
        val candsPerPoint = window.map { p ->
            region.segments.nearestSegments(p.latitude, p.longitude, SEARCH_RADIUS_M, MAX_CANDIDATES)
                .map { seg ->
                    val pr = RoadSnapMath.projectPointOntoSegment(
                        p.latitude, p.longitude, seg.aLat, seg.aLng, seg.bLat, seg.bLng
                    )
                    Candidate(pr.snappedLat, pr.snappedLng, pr.distanceM, seg.wayId)
                }
        }
        return matchCandidates(window, candsPerPoint)
    }

    /**
     * Pure Viterbi over per-point [candsPerPoint] (same length as [window]). Returns the
     * matched location of the LATEST matchable point (the dot) + the matched-road polyline,
     * or null if no point had a candidate. A point with no candidate is an off-road gap that
     * restarts the trellis.
     */
    fun matchCandidates(window: List<LatLng>, candsPerPoint: List<List<Candidate>>): LiveMatcher.MatchResult? {
        if (window.size < MIN_POINTS) return null
        var prev: List<Node>? = null
        for (i in window.indices) {
            val layer = candsPerPoint[i]
            if (layer.isEmpty()) { prev = null; continue }   // off-road gap → restart trellis
            val nodes = ArrayList<Node>(layer.size)
            val prevLayer = prev
            for (c in layer) {
                val emit = -0.5 * (c.offsetM / SIGMA_Z) * (c.offsetM / SIGMA_Z)   // log Gaussian emission
                if (prevLayer == null) {
                    nodes.add(Node(c, emit, null))
                } else {
                    val dGc = RoadSnapMath.haversineM(
                        window[i - 1].latitude, window[i - 1].longitude, window[i].latitude, window[i].longitude
                    )
                    var bestScore = Double.NEGATIVE_INFINITY
                    var bestBack: Node? = null
                    for (pn in prevLayer) {
                        val dRoute = RoadSnapMath.haversineM(pn.cand.lat, pn.cand.lng, c.lat, c.lng)
                        val trans = -abs(dGc - dRoute) / BETA
                        val sameWay = if (pn.cand.wayId == c.wayId) SAME_WAY_BONUS else 0.0
                        val s = pn.score + trans + sameWay
                        if (s > bestScore) { bestScore = s; bestBack = pn }
                    }
                    nodes.add(Node(c, bestScore + emit, bestBack))
                }
            }
            prev = nodes
        }
        val terminal = prev ?: return null
        var node: Node? = terminal.maxByOrNull { it.score } ?: return null
        val chosen = ArrayList<Candidate>()
        while (node != null) { chosen.add(node.cand); node = node.back }
        chosen.reverse()
        if (chosen.isEmpty()) return null

        val path = chosen.map { LatLng(it.lat, it.lng) }
        val routeDistanceM = (1 until chosen.size).sumOf {
            RoadSnapMath.haversineM(chosen[it - 1].lat, chosen[it - 1].lng, chosen[it].lat, chosen[it].lng)
        }
        // Confidence ~ how tightly the chosen path hugs the roads (mirrors OSRM's [0,1]).
        val meanOffsetM = chosen.sumOf { it.offsetM } / chosen.size
        val confidence = exp(-meanOffsetM / SIGMA_Z).coerceIn(0.0, 1.0)
        val dot = LatLng(chosen.last().lat, chosen.last().lng)
        return LiveMatcher.MatchResult(dot, confidence, path, routeDistanceM, broken = false)
    }

    private class Node(val cand: Candidate, val score: Double, val back: Node?)

    companion object {
        private const val MIN_POINTS = 4
        private const val MAX_CANDIDATES = 5
        private const val SEARCH_RADIUS_M = 30.0  // candidate search radius (a bit wider than per-point snap)
        private const val SIGMA_Z = 20.0          // emission stdev: VIO position noise / OSM way accuracy
        private const val BETA = 6.0              // transition scale (Newson-Krumm distance consistency)
        private const val SAME_WAY_BONUS = 0.7    // log-prob bonus for staying on the same OSM way
    }
}
