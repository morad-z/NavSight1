package com.example.navsight1

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos

/**
 * Heading-aware maneuver detection (MAP_MATCHING_PLAN.md §0.7). A pure, session-
 * scoped state machine fed (position, VIO heading, roundabout index) each ~500 ms
 * snap tick. It DETECTS maneuvers from the reliable heading signal:
 *   - you are ON a specific roundabout, accumulating the heading sweep;
 *   - WHICH EXIT you take (the exit road whose bearing matches your heading);
 *   - a U-TURN VIA the roundabout (you leave ~opposite to how you entered);
 *   - a MID-ROAD U-TURN (heading reverses ~180° without moving far).
 *
 * It outputs the maneuver state + the snap candidate constraint (pin to the ring).
 * It does NOT render the arc shape at true size — that needs the metric-scale fix
 * (Fix A/B) + the rotation-freeze fix (Fix C) on the speed branch (§0.7, §10).
 *
 * Pure Kotlin (no Android) so it is unit-testable on the JVM. Logging is done by
 * the caller (NavSightViewModel) on each returned [ManeuverResult.event].
 */
// Public value types (the ViewModel exposes maneuverState; RoadSnapper takes a
// ManeuverResult). The machinery (state machine, index) stays internal.
enum class ManeuverState { FREE_ROAD, ON_ROUNDABOUT, MID_ROAD_UTURN }
enum class TravelDirection { FORWARD, REVERSED }
enum class ManeuverEvent {
    ROUNDABOUT_ENTERED, ROUNDABOUT_EXIT, UTURN_VIA_ROUNDABOUT, UTURN_DETECTED, UTURN_COMPLETE
}

data class ManeuverResult(
    val state: ManeuverState,
    val activeRing: RoundaboutRing?,
    val suggestedExit: ExitRoad?,
    val travelDirection: TravelDirection,
    val accumulatedSweepDeg: Double,
    val signedSweepDeg: Double,
    val event: ManeuverEvent?
)

internal object ManeuverMath {
    /** Signed shortest-arc difference a−b, in (−180, +180]. */
    fun angularDifferenceDeg(a: Double, b: Double): Double = ((a - b + 540.0) % 360.0) - 180.0

    /** Compass bearing from→to: 0=N, CW-positive (matches the VIO world heading). */
    fun bearingDeg(fromLat: Double, fromLng: Double, toLat: Double, toLng: Double): Double {
        val mLng = RoadSnapMath.M_PER_DEG_LAT * cos(Math.toRadians(fromLat))
        val east = (toLng - fromLng) * mLng
        val north = (toLat - fromLat) * RoadSnapMath.M_PER_DEG_LAT
        return (Math.toDegrees(atan2(east, north)) + 360.0) % 360.0
    }
}

internal class ManeuverStateMachine {

    private var state = ManeuverState.FREE_ROAD
    private var activeRing: RoundaboutRing? = null
    private var suggestedExit: ExitRoad? = null
    private var travelDirection = TravelDirection.FORWARD

    private var entryBearingDeg = Double.NaN
    private var lastHeadingDeg = Double.NaN
    private var accumulatedSweepDeg = 0.0
    private var signedSweepDeg = 0.0
    private var ticksOnRing = 0   // 2026-05-31 — dwell-tick safety cap to bail the frozen-dot latch

    private var lastLat = Double.NaN
    private var lastLng = Double.NaN
    private var uturnAnchorLat = Double.NaN
    private var uturnAnchorLng = Double.NaN

    // rolling window for mid-road U-turn detection: each entry = [heading, lat, lng]
    private val window = ArrayDeque<DoubleArray>()

    fun reset() {
        state = ManeuverState.FREE_ROAD
        activeRing = null; suggestedExit = null; travelDirection = TravelDirection.FORWARD
        entryBearingDeg = Double.NaN; lastHeadingDeg = Double.NaN
        accumulatedSweepDeg = 0.0; signedSweepDeg = 0.0; ticksOnRing = 0
        lastLat = Double.NaN; lastLng = Double.NaN
        uturnAnchorLat = Double.NaN; uturnAnchorLng = Double.NaN
        window.clear()
    }

    /**
     * Advance one tick. [headingDeg] is the VIO world heading in degrees (0=N, CW).
     * [roundabouts] is the current region's index (null when no region loaded).
     */
    fun tick(lat: Double, lng: Double, headingDeg: Double, roundabouts: RoundaboutIndex?): ManeuverResult {
        if (headingDeg.isNaN() || roundabouts == null) {
            reset()
            return result(null)
        }
        // Region swap guard: the active ring vanished → drop to FREE_ROAD.
        activeRing?.let { if (!roundabouts.contains(it.wayId)) toFreeRoad() }

        val moved = if (lastLat.isNaN()) 0.0 else RoadSnapMath.haversineM(lastLat, lastLng, lat, lng)
        var event: ManeuverEvent? = null

        when (state) {
            ManeuverState.FREE_ROAD -> {
                val ring = roundabouts.nearestRing(lat, lng, ENTER_MARGIN_M)
                if (ring != null) {
                    enterRoundabout(ring, headingDeg)
                    event = ManeuverEvent.ROUNDABOUT_ENTERED
                } else {
                    pushWindow(headingDeg, lat, lng)
                    if (moved >= MOVE_GATE_M && detectMidRoadUTurn(headingDeg, lat, lng)) {
                        state = ManeuverState.MID_ROAD_UTURN
                        travelDirection = TravelDirection.REVERSED
                        uturnAnchorLat = lat; uturnAnchorLng = lng
                        window.clear()
                        event = ManeuverEvent.UTURN_DETECTED
                    }
                }
            }

            ManeuverState.ON_ROUNDABOUT -> {
                val ring = activeRing!!
                ticksOnRing++
                // 2026-05-31 — accumulate the heading sweep on ANY heading change, NOT gated on
                // translation. The Fix-C turn-freeze holds the dot in place while heading sweeps, so
                // a motion-gated sweep stays 0 and the sweep/dwell exits below could never fire — the
                // exact 20-39s ON_ROUNDABOUT latch seen on the scooter sims (val_2026_05_31_scooter).
                if (!lastHeadingDeg.isNaN()) {
                    val delta = ManeuverMath.angularDifferenceDeg(headingDeg, lastHeadingDeg)
                    signedSweepDeg += delta
                    accumulatedSweepDeg += abs(delta)
                }
                lastHeadingDeg = headingDeg
                val distToCenter = RoadSnapMath.haversineM(lat, lng, ring.centerLat, ring.centerLng)
                // EXIT on ANY of: (a) we left the ring outward (the real geometric exit); (b) we have
                // swept more than a full turn (a real traversal exits well before ~360°, so >MAX means
                // stuck/spinning); (c) a dwell-tick safety cap (a real pass is bounded — this bails the
                // frozen-dot latch the distToCenter test alone cannot catch).
                val leftRing = distToCenter > ring.radiusM + EXIT_MARGIN_M
                if (leftRing || accumulatedSweepDeg > MAX_RBT_SWEEP_DEG || ticksOnRing > MAX_RING_DWELL_TICKS) {
                    suggestedExit = pickExit(ring, headingDeg, lat, lng)
                    // U-turn via roundabout: we leave heading ~opposite how we entered.
                    val reversed = !entryBearingDeg.isNaN() &&
                        abs(ManeuverMath.angularDifferenceDeg(headingDeg, entryBearingDeg)) > UTURN_VIA_RBT_THRESH_DEG
                    event = if (reversed) ManeuverEvent.UTURN_VIA_ROUNDABOUT else ManeuverEvent.ROUNDABOUT_EXIT
                    toFreeRoad()
                }
            }

            ManeuverState.MID_ROAD_UTURN -> {
                val dist = RoadSnapMath.haversineM(uturnAnchorLat, uturnAnchorLng, lat, lng)
                if (dist > UTURN_COMPLETE_DIST_M) {
                    state = ManeuverState.FREE_ROAD
                    travelDirection = TravelDirection.FORWARD
                    event = ManeuverEvent.UTURN_COMPLETE
                }
            }
        }

        lastLat = lat; lastLng = lng
        return result(event)
    }

    private fun enterRoundabout(ring: RoundaboutRing, headingDeg: Double) {
        state = ManeuverState.ON_ROUNDABOUT
        activeRing = ring
        entryBearingDeg = headingDeg
        lastHeadingDeg = headingDeg
        accumulatedSweepDeg = 0.0
        signedSweepDeg = 0.0
        ticksOnRing = 0
        suggestedExit = null
        window.clear()
    }

    private fun toFreeRoad() {
        state = ManeuverState.FREE_ROAD
        activeRing = null
        entryBearingDeg = Double.NaN
        lastHeadingDeg = Double.NaN
    }

    private fun pickExit(ring: RoundaboutRing, headingDeg: Double, lat: Double, lng: Double): ExitRoad? {
        if (ring.exits.isEmpty()) return null
        var best: ExitRoad? = null
        var bestDiff = Double.POSITIVE_INFINITY
        for (e in ring.exits) {
            val d = abs(ManeuverMath.angularDifferenceDeg(headingDeg, e.bearingDeg))
            if (d < bestDiff) { bestDiff = d; best = e }
        }
        if (bestDiff <= EXIT_BEARING_TOL_DEG) return best
        // Ambiguous (heading doesn't clearly match an arm) → nearest exit node.
        return ring.exits.minByOrNull { RoadSnapMath.haversineM(lat, lng, it.nodeLat, it.nodeLng) }
    }

    private fun pushWindow(h: Double, lat: Double, lng: Double) {
        window.addLast(doubleArrayOf(h, lat, lng))
        while (window.size > UTURN_WINDOW_TICKS) window.removeFirst()
    }

    private fun detectMidRoadUTurn(curH: Double, curLat: Double, curLng: Double): Boolean {
        if (window.size < UTURN_WINDOW_TICKS) return false
        val old = window.first()
        val flipped = abs(ManeuverMath.angularDifferenceDeg(curH, old[0])) > UTURN_HEADING_THRESH_DEG
        val disp = RoadSnapMath.haversineM(old[1], old[2], curLat, curLng)
        return flipped && disp < UTURN_MAX_DISPLACEMENT_M
    }

    private fun result(event: ManeuverEvent?) = ManeuverResult(
        state, activeRing, suggestedExit, travelDirection,
        accumulatedSweepDeg, signedSweepDeg, event
    )

    companion object {
        private const val ENTER_MARGIN_M = 12.0
        private const val EXIT_MARGIN_M = 15.0
        private const val EXIT_BEARING_TOL_DEG = 35.0
        private const val UTURN_HEADING_THRESH_DEG = 160.0
        private const val UTURN_WINDOW_TICKS = 6      // ~3 s at 500 ms ticks
        private const val UTURN_COMPLETE_DIST_M = 8.0
        private const val UTURN_MAX_DISPLACEMENT_M = 20.0
        private const val MOVE_GATE_M = 0.5
        private const val UTURN_VIA_RBT_THRESH_DEG = 160.0
        // 2026-05-31 roundabout-latch exits: a real traversal exits well before a full turn, so a
        // sweep past ~one revolution = stuck/spinning; the dwell cap (~30 s at 500 ms ticks) is a
        // hard safety bail for the frozen-dot latch the centre-distance exit can't catch.
        private const val MAX_RBT_SWEEP_DEG = 400.0
        private const val MAX_RING_DWELL_TICKS = 60
    }
}
