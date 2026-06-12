package com.example.navsight1

import kotlin.math.abs

/**
 * GraphRailDot — the "ball in a maze" display constraint (MAP_MATCHING_PLAN.md §8M, the
 * advance-along-rail leg that was deferred in the rail-LOCK step, 2026-05-31).
 *
 * THE PROBLEM IT FIXES (owner, 2026-05-31, with screenshot): the displayed dot was a per-tick
 * perpendicular re-projection of the drifting VIO point onto the nearest road. The moment the
 * VIO trace drifts past the matcher's search radius, the "nearest road" becomes a DIFFERENT,
 * often disconnected parallel road, so the dot teleports across the block ("snapped into
 * nowhere" / "jumped through the wall to another part of the maze"). No matcher-side emission
 * or transition tweak fixes that — they all re-project the same drifting point (measured:
 * rail-LOCK bonus capped at 1.80× GPS with the 8 teleports UNCHANGED).
 *
 * THE FIX (the owner's analogy, literally): the dot is a BALL that lives ON a routing-graph
 * EDGE. Each tick it is MOVED FORWARD along that edge by the distance the user actually
 * travelled. When it reaches a junction (a graph vertex) it can only continue onto a CONNECTED
 * edge — never onto a disconnected parallel road. It therefore physically cannot leave the road
 * network or teleport through a wall. It RE-ACQUIRES (a sanctioned, rare jump to a new edge)
 * only when the raw VIO point has stayed far off the rail for several consecutive ticks = a
 * confirmed wrong road / missed turn, never on per-tick jitter.
 *
 * Per-tick inputs: forward distance travelled (m) and the VIO direction-of-travel (compass
 * bearing, 0=N, CW+). Junction choice uses that bearing and naturally prefers going STRAIGHT
 * (a straight-through pass has VIO-bearing ≈ road-bearing, so the colinear edge wins), which
 * keeps it robust to the slowly-varying absolute-heading drift — only a genuine turn, where the
 * VIO bearing swings, makes it take a different connected edge.
 *
 * CAVEAT (documented, not hidden): the along-rail POSITION advances by the VIO distance, which
 * still under-reads on a scooter (the separate per-mode-K / K̂_map speed lever). So the ball can
 * LAG along an otherwise-correct road. This component fixes the CROSS-track failure the owner
 * showed (off-road + teleport); it does not fix along-track speed. Once the ball locks the right
 * road, K̂_map = d_route/d_vio becomes a clean scale observation that can later drive the advance.
 *
 * Pure / Android-free (GeoPt, GraphStore, RoadSnapMath, ManeuverMath only) so it is unit-testable
 * on the JVM and replayable offline in RailReplayTest.
 */
internal class GraphRailDot(private val graph: GraphStore) {

    private var u = -1      // current edge tail vertex
    private var w = -1      // current edge head vertex (travel runs u → w)
    private var t = 0.0     // fraction along u → w in [0,1]
    var acquired = false
        private set
    // 2026-06-12 — true when THIS advance() crossed a vertex offering ≥2 eligible continuations (a real
    // junction DECISION, not a curve shape-point). The caller re-anchors its gyro-relative steering
    // reference ONLY then. Re-anchoring on every >1° facing change (the old rule) zeroed the user's
    // accumulated rotation at every curve vertex (06-04 ride logcat: 49/49 re-anchors were 2-7° curve
    // steps) — that glued the steering to the road's own tangent, which (a) made the ball prefer the
    // chord-straight branch at forks on curves and (b) made a roundabout ring inescapable (the exit arm
    // could never beat the ring tangent).
    var lastAdvanceTookDecision = false
        private set
    private var choiceHadAlternatives = false

    /** Current on-graph position, or null when not acquired (off-road / no graph). */
    fun position(): GeoPt? = if (acquired) interp(u, w, t) else null

    /**
     * (Re)snap the ball to the nearest graph edge to (lat,lng), oriented so travel runs roughly
     * along [preferBearingDeg]. Returns false — and leaves the ball UNACQUIRED — when the nearest
     * edge is farther than [maxAcquireM] (off-road, or no routing graph, e.g. the indoor house
     * loop), so the caller can fall back to its raw snapper instead of force-snapping to a far road.
     */
    fun acquire(lat: Double, lng: Double, preferBearingDeg: Double, maxAcquireM: Double = MAX_ACQUIRE_M): Boolean {
        val hit = graph.nearestEdge(lat, lng)
        if (hit == null || hit.offsetM > maxAcquireM) { acquired = false; return false }
        // Orient u → w so the edge's forward bearing is the one closer to the desired travel
        // bearing (within 90°); otherwise traverse the edge the other way.
        val fwd = ManeuverMath.bearingDeg(graph.vLatAt(hit.u), graph.vLngAt(hit.u),
                                          graph.vLatAt(hit.w), graph.vLngAt(hit.w))
        // Reverse u↔w only when the travel bearing disagrees by >90° AND the reverse arc is LEGAL
        // (the edge is bidirectional — w has an arc back to u). On a ONE-WAY edge the CSR stores only
        // the permitted direction, so reversing would point the ball the forbidden way into a vertex
        // with no forward neighbour → it would oscillate forever (review HIGH). When the user travels
        // against a one-way, we keep the legal direction (the ball lags but advances on the real road).
        if (preferBearingDeg.isFinite() &&
            abs(ManeuverMath.angularDifferenceDeg(fwd, preferBearingDeg)) > 90.0 &&
            hasArc(hit.w, hit.u)) {
            u = hit.w; w = hit.u; t = 1.0 - hit.t
        } else {
            u = hit.u; w = hit.w; t = hit.t
        }
        acquired = true
        return true
    }

    /** Reverse the ball's facing along the road (a confirmed U-turn or a wrong initial facing). Flips the
     *  current edge to its reverse so the ball faces + advances the other way and the arrow turns 180°.
     *  Returns true if it could reverse (the reverse arc exists = a two-way road). On a one-way road the
     *  reverse is illegal so this is a no-op (you can't drive a one-way backward). */
    fun reverse(): Boolean {
        if (!acquired) return false
        if (!hasArc(w, u)) return false
        val tmp = u; u = w; w = tmp; t = 1.0 - t
        return true
    }

    /** True if the graph has a directed arc [from]→[to] (used to test edge bidirectionality). */
    private fun hasArc(from: Int, to: Int): Boolean {
        val d = graph.degreeOf(from)
        for (k in 0 until d) if (graph.neighborAt(from, k) == to) return true
        return false
    }

    /**
     * Advance the ball [distM] forward along the graph, turning at each junction it reaches
     * toward [bearingDeg] (the VIO travel direction this tick). Returns the new on-graph
     * position, or null if not acquired. A non-positive / non-finite [distM] leaves the ball
     * in place (returns the current position).
     */
    fun advance(distM: Double, bearingDeg: Double): GeoPt? {
        lastAdvanceTookDecision = false   // 2026-06-12 — set below when a junction choice happens
        if (!acquired) return null
        // NOTE: the facing (forward vs backward along the road) is NOT decided here per-tick — that was
        // fragile and flipped on noise. The caller detects a SUSTAINED reversal (a real U-turn or a
        // wrong initial facing) and calls reverse(). advance() just moves forward along the current
        // facing and turns at junctions toward [bearingDeg] (the user's corrected heading).
        var move = if (distM.isFinite() && distM > 0.0) distM else 0.0
        var hops = 0
        while (hops < MAX_HOPS_PER_TICK) {
            val edgeLen = edgeLenM(u, w)
            if (edgeLen < MIN_EDGE_M) {
                // Degenerate / zero-length edge — step straight to the head and keep going so we
                // never divide by ~0 or stall. If the only "exit" is the same vertex (a self-loop),
                // bail out instead of spinning the backstop 64× every tick (review HIGH).
                val next = chooseNextNeighbor(w, prevVertex = u, desiredBearingDeg = effectiveBearing(bearingDeg))
                if (choiceHadAlternatives) lastAdvanceTookDecision = true
                if (next == w) break
                u = w; w = next; t = 0.0; hops++
                continue
            }
            val edgeRemaining = (1.0 - t) * edgeLen
            if (move <= edgeRemaining) {
                t = (t + move / edgeLen).coerceAtMost(1.0)
                break
            }
            // Consume the rest of this edge and turn at the junction onto a CONNECTED edge.
            move -= edgeRemaining
            val next = chooseNextNeighbor(w, prevVertex = u, desiredBearingDeg = effectiveBearing(bearingDeg))
            if (choiceHadAlternatives) lastAdvanceTookDecision = true
            u = w; w = next; t = 0.0; hops++
        }
        // If we exhausted the hop backstop we are stuck on a degenerate/tiny disconnected component —
        // drop the lock so the next tick re-acquires cleanly instead of spinning here forever (review HIGH).
        if (hops >= MAX_HOPS_PER_TICK) acquired = false
        return if (acquired) interp(u, w, t) else null
    }

    /** Bearing (compass deg, 0=N CW) of the ball's CURRENT edge in the travel direction (u→w) — i.e.
     *  the road's local TANGENT at the ball's position. It updates as the ball advances, so it follows
     *  the road's curvature (and a turn onto a connected road yields that road's tangent). null when
     *  unacquired. Used to align the VIO heading to the road continuously, including on curves. */
    fun currentBearingDeg(): Double? {
        if (!acquired) return null
        return ManeuverMath.bearingDeg(graph.vLatAt(u), graph.vLngAt(u), graph.vLatAt(w), graph.vLngAt(w))
    }

    /** Great-circle metres from (lat,lng) to the current ball position (∞ when unacquired).
     *  The caller uses this to detect a CONFIRMED wrong road before allowing a re-acquire. */
    fun offsetFromM(lat: Double, lng: Double): Double {
        val p = position() ?: return Double.POSITIVE_INFINITY
        return RoadSnapMath.haversineM(lat, lng, p.lat, p.lng)
    }

    // ---- internals -----------------------------------------------------------------------

    /** Pick the next connected edge at [junction], preferring the neighbour whose bearing best
     *  matches [desiredBearingDeg]. Excludes an immediate U-turn back to [prevVertex] unless the
     *  junction is a dead-end (then reversing is the only legal move). */
    private fun chooseNextNeighbor(junction: Int, prevVertex: Int, desiredBearingDeg: Double): Int {
        val deg = graph.degreeOf(junction)
        var best = -1
        var bestScore = Double.POSITIVE_INFINITY
        var eligible = 0   // 2026-06-12 — ≥2 eligible continuations = a real junction decision
        for (k in 0 until deg) {
            val nb = graph.neighborAt(junction, k)
            if (nb == prevVertex && deg > 1) continue   // no instant U-turn unless dead-end
            eligible++
            val b = ManeuverMath.bearingDeg(graph.vLatAt(junction), graph.vLngAt(junction),
                                            graph.vLatAt(nb), graph.vLngAt(nb))
            val score = abs(ManeuverMath.angularDifferenceDeg(b, desiredBearingDeg))
            if (score < bestScore) { bestScore = score; best = nb }
        }
        choiceHadAlternatives = eligible >= 2
        return if (best >= 0) best else prevVertex   // dead-end → reverse
    }

    /** VIO travel bearing if finite, else the current edge's own bearing (= continue straight). */
    private fun effectiveBearing(bearingDeg: Double): Double =
        if (bearingDeg.isFinite()) bearingDeg
        else ManeuverMath.bearingDeg(graph.vLatAt(u), graph.vLngAt(u), graph.vLatAt(w), graph.vLngAt(w))

    private fun edgeLenM(a: Int, b: Int): Double =
        RoadSnapMath.haversineM(graph.vLatAt(a), graph.vLngAt(a), graph.vLatAt(b), graph.vLngAt(b))

    private fun interp(a: Int, b: Int, frac: Double): GeoPt {
        val f = frac.coerceIn(0.0, 1.0)
        return GeoPt(
            graph.vLatAt(a) + (graph.vLatAt(b) - graph.vLatAt(a)) * f,
            graph.vLngAt(a) + (graph.vLngAt(b) - graph.vLngAt(a)) * f
        )
    }

    companion object {
        /** Don't acquire to a road farther than the matcher's search radius (SEARCH_RADIUS_M=30) —
         *  beyond that we are genuinely off-road and the caller should use its raw snapper. */
        private const val MAX_ACQUIRE_M = 30.0
        /** Treat an edge shorter than this as degenerate (step over it) to avoid div-by-~0. */
        private const val MIN_EDGE_M = 0.5
        /** Per-tick junction-hop backstop: bounds work if VIO reports a huge jump or the ball lands
         *  on a tiny disconnected component. 64 hops ≫ any plausible 0.5 s of real travel. */
        private const val MAX_HOPS_PER_TICK = 64
    }
}
