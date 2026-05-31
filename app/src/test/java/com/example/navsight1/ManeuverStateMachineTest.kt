package com.example.navsight1

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos

/**
 * Synthetic-trace tests for the heading-aware [ManeuverStateMachine] (§0.7).
 * Proves roundabout entry, exit-by-heading, and mid-road U-turn detection on the
 * JVM (no device). All pure doubles — no Android/gms.
 */
class ManeuverStateMachineTest {

    private val cLat = 32.800
    private val cLng = 35.000

    /** offset (lat,lng) by metres north/east. */
    private fun off(north: Double, east: Double): Pair<Double, Double> {
        val lat = cLat + north / 111_320.0
        val lng = cLng + east / (111_320.0 * cos(Math.toRadians(cLat)))
        return lat to lng
    }

    /** A 20 m ring at the centre with N/E/S/W exits (bearings 0/90/180/270). */
    private fun ringIndex(): RoundaboutIndex {
        val nodes = listOf(off(20.0, 0.0), off(0.0, 20.0), off(-20.0, 0.0), off(0.0, -20.0))
            .map { doubleArrayOf(it.first, it.second) }
        val exits = listOf(
            ExitRoad(10, nodes[0][0], nodes[0][1], 0.0),
            ExitRoad(11, nodes[1][0], nodes[1][1], 90.0),
            ExitRoad(12, nodes[2][0], nodes[2][1], 180.0),
            ExitRoad(13, nodes[3][0], nodes[3][1], 270.0)
        )
        return RoundaboutIndex(listOf(RoundaboutRing(1, cLat, cLng, 20.0, exits, nodes)))
    }

    @Test
    fun `enters roundabout from approach`() {
        val sm = ManeuverStateMachine()
        val idx = ringIndex()
        // 45 m south, heading north → outside the ring (45 > 20+12)
        var p = off(-45.0, 0.0)
        var r = sm.tick(p.first, p.second, 0.0, idx)
        assertEquals(ManeuverState.FREE_ROAD, r.state)
        // 25 m south → within radius+margin → enter
        p = off(-25.0, 0.0)
        r = sm.tick(p.first, p.second, 0.0, idx)
        assertEquals(ManeuverState.ON_ROUNDABOUT, r.state)
        assertEquals(ManeuverEvent.ROUNDABOUT_ENTERED, r.event)
        assertNotNull(r.activeRing)
        assertEquals(1L, r.activeRing!!.wayId)
    }

    @Test
    fun `picks the east exit by heading`() {
        val sm = ManeuverStateMachine()
        val idx = ringIndex()
        sm.tick(off(-45.0, 0.0).first, off(-45.0, 0.0).second, 0.0, idx)   // approach
        sm.tick(off(-25.0, 0.0).first, off(-25.0, 0.0).second, 0.0, idx)   // enter
        sm.tick(off(0.0, 10.0).first, off(0.0, 10.0).second, 45.0, idx)    // inside, turning
        // 40 m east, heading 90 → exit east
        val ex = off(0.0, 40.0)
        val r = sm.tick(ex.first, ex.second, 90.0, idx)
        assertEquals(ManeuverState.FREE_ROAD, r.state)
        assertEquals(ManeuverEvent.ROUNDABOUT_EXIT, r.event)
        assertNotNull(r.suggestedExit)
        assertEquals(90.0, r.suggestedExit!!.bearingDeg, 1.0)
    }

    @Test
    fun `detects a mid-road U-turn from heading reversal`() {
        val sm = ManeuverStateMachine()
        val idx = ringIndex()
        // far from the ring so nearestRing == null; drive north ~1 m/tick heading 0
        var lat = 32.850; val lng = 35.050
        repeat(6) { lat += 1.0 / 111_320.0; sm.tick(lat, lng, 0.0, idx) }
        // flip heading to 175° (≈U-turn) at ~same spot
        lat += 1.0 / 111_320.0
        val r = sm.tick(lat, lng, 175.0, idx)
        assertEquals(ManeuverState.MID_ROAD_UTURN, r.state)
        assertEquals(ManeuverEvent.UTURN_DETECTED, r.event)
        assertEquals(TravelDirection.REVERSED, r.travelDirection)
        // drive back >8 m → U-turn complete
        lat -= 12.0 / 111_320.0
        val r2 = sm.tick(lat, lng, 175.0, idx)
        assertEquals(ManeuverState.FREE_ROAD, r2.state)
        assertEquals(ManeuverEvent.UTURN_COMPLETE, r2.event)
        assertEquals(TravelDirection.FORWARD, r2.travelDirection)
    }

    @Test
    fun `a gentle 90 degree curve is not a U-turn`() {
        val sm = ManeuverStateMachine()
        val idx = ringIndex()
        var lat = 32.850; val lng = 35.050
        // heading drifts 0→90 over 8 ticks, moving — never exceeds 160° vs window start
        for (h in listOf(0.0, 10.0, 25.0, 40.0, 55.0, 70.0, 85.0, 90.0)) {
            lat += 1.0 / 111_320.0
            val r = sm.tick(lat, lng, h, idx)
            assertEquals(ManeuverState.FREE_ROAD, r.state)
            assertNull(r.event)
        }
    }

    @Test
    fun `null roundabout index degrades to free road`() {
        val sm = ManeuverStateMachine()
        val r = sm.tick(cLat, cLng, 90.0, null)
        assertEquals(ManeuverState.FREE_ROAD, r.state)
        assertNull(r.event)
    }

    @Test
    fun `does not enter when passing near but outside the ring`() {
        val sm = ManeuverStateMachine()
        val idx = ringIndex()
        // travel east 60 m offset 40 m north of centre → always > radius+margin
        for (e in -60..60 step 10) {
            val p = off(40.0, e.toDouble())
            val r = sm.tick(p.first, p.second, 90.0, idx)
            assertEquals(ManeuverState.FREE_ROAD, r.state)
        }
    }
}
