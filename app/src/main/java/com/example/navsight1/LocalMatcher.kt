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

    // 2026-05-31 (RAIL-LOCK) — the OSM way the matcher is currently "railed" to, carried
    // ACROSS match() calls. Before this, every ~500ms tick re-decoded the window from scratch
    // (prev=null) so a VIO window that had drifted 15-66deg off-heading but kept its arc-LENGTH
    // re-matched onto a PARALLEL road — the teleport that inflated the matched path to 2.09x GPS
    // (val_2026_05_31_scooter). With a lock, the matcher FOLLOWS THE ROAD, not the drifting trace:
    // it only leaves the railed way when the caller's ManeuverStateMachine fires a real maneuver
    // (releaseRail) or when the railed way is no longer in range (match() recovery below).
    @Volatile private var lockedWayId: Long? = null   // read on the IO match() coroutine, cleared from the SM tick

    /** Current rail (the locked OSM way), or null when unlocked. */
    fun railWayId(): Long? = lockedWayId

    /** Drop the rail lock so the next match() may switch roads. The caller invokes this on a
     *  ManeuverStateMachine event (intersection turn / mid-road U-turn / roundabout) — i.e. the
     *  ONLY sanctioned reasons to leave the current road. */
    fun releaseRail() { lockedWayId = null }

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
        // Rail recovery: if the locked way is no longer a candidate anywhere in the window, we
        // have physically left that road (not via a detected maneuver) — drop the stale lock and
        // re-decode freely so we can re-acquire, rather than dragging the dot back onto it.
        val lock = lockedWayId
        if (lock != null && candsPerPoint.none { layer -> layer.any { it.wayId == lock } }) {
            lockedWayId = null
        }
        val r = matchCandidates(window, candsPerPoint, lockedWayId) ?: return null
        // Engage/refresh the lock once the match confidently hugs a road (mean offset within ~half
        // the emission sigma => confidence >= exp(-0.5) ~= 0.61). Below that we stay unlocked so we
        // never lock onto a weak/ambiguous match.
        if (r.confidence >= LOCK_MIN_CONFIDENCE && r.matchedWayId != null) lockedWayId = r.matchedWayId
        return r
    }

    /**
     * Pure Viterbi over per-point [candsPerPoint] (same length as [window]). Returns the
     * matched location of the LATEST matchable point (the dot) + the matched-road polyline,
     * or null if no point had a candidate. A point with no candidate is an off-road gap that
     * restarts the trellis.
     */
    fun matchCandidates(
        window: List<LatLng>,
        candsPerPoint: List<List<Candidate>>,
        railWayId: Long? = null
    ): LiveMatcher.MatchResult? {
        if (window.size < MIN_POINTS) return null
        var prev: List<Node>? = null
        for (i in window.indices) {
            val layer = candsPerPoint[i]
            if (layer.isEmpty()) { prev = null; continue }   // off-road gap → restart trellis
            val nodes = ArrayList<Node>(layer.size)
            val prevLayer = prev
            for (c in layer) {
                val emit = -0.5 * (c.offsetM / SIGMA_Z) * (c.offsetM / SIGMA_Z)   // log Gaussian emission
                // RAIL-LOCK: a candidate ON the currently-railed way gets a fixed bonus so the
                // matcher stays on the road it committed to instead of re-snapping to a parallel
                // road when the trace drifts. railWayId==null (unlocked) => 0 => unchanged behavior.
                val railB = if (railWayId != null && c.wayId == railWayId) RAIL_BONUS else 0.0
                if (prevLayer == null) {
                    nodes.add(Node(c, emit + railB, null))
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
                    nodes.add(Node(c, bestScore + emit + railB, bestBack))
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
        return LiveMatcher.MatchResult(
            dot, confidence, path, routeDistanceM, broken = false,
            matchedWayId = chosen.last().wayId   // the way the dot sits on → the rail for next tick
        )
    }

    private class Node(val cand: Candidate, val score: Double, val back: Node?)

    companion object {
        private const val MIN_POINTS = 4
        private const val MAX_CANDIDATES = 5
        private const val SEARCH_RADIUS_M = 30.0  // candidate search radius (a bit wider than per-point snap)
        private const val SIGMA_Z = 20.0          // emission stdev: VIO position noise / OSM way accuracy
        private const val BETA = 6.0              // transition scale (Newson-Krumm distance consistency)
        private const val SAME_WAY_BONUS = 0.7    // log-prob bonus for staying on the same OSM way
        // RAIL-LOCK bonus for a candidate on the currently-railed way. Derived to sit just above
        // the maximum in-search-radius emission span 0.5*(SEARCH_RADIUS_M/SIGMA_Z)^2 =
        // 0.5*(30/20)^2 = 1.125, so while railed a candidate ON the road beats any off-rail
        // candidate within the search radius — the matcher follows the road, not the drifting
        // trace. It still leaves on a maneuver (releaseRail) or when the way exits range.
        private const val RAIL_BONUS = 1.2
        // Engage the rail lock only when the match hugs the road within ~half the emission sigma
        // (confidence = exp(-meanOffsetM/SIGMA_Z) >= exp(-0.5) ≈ 0.61) — never lock a weak match.
        private const val LOCK_MIN_CONFIDENCE = 0.6
    }
}
