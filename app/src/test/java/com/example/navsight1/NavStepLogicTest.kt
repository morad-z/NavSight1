package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Tests the turn-by-turn leg advance ([NavStepLogic.advanceLeg]) — the fix for the
 * off-by-one that made the banner show the maneuver already taken. The "current leg"
 * is `idx`; the upcoming maneuver is `idx+1`. JVM-pure (gms LatLng OK in unit tests).
 */
class NavStepLogicTest {

    // Maneuver points ~187 m apart heading east at lat 32.80 (depart, turn, turn, arrive).
    private val pts = listOf(
        LatLng(32.80, 35.000),
        LatLng(32.80, 35.002),
        LatLng(32.80, 35.004),
        LatLng(32.80, 35.006),
    )
    private val reach = 18.0

    @Test
    fun `stays on the first leg at the start`() {
        assertEquals(0, NavStepLogic.advanceLeg(0, pts, pts[0], reach))
    }

    @Test
    fun `advances when within reach of the next maneuver point`() {
        // ~14 m before maneuver point 1 (within the 18 m reach radius).
        val pos = LatLng(32.80, 35.002 + 14.0 / (111_320.0 * Math.cos(Math.toRadians(32.80))))
        assertEquals(1, NavStepLogic.advanceLeg(0, pts, pos, reach))
    }

    @Test
    fun `advances incrementally through each maneuver as it is reached`() {
        // Called every tick as the user reaches each maneuver point in turn.
        assertEquals(1, NavStepLogic.advanceLeg(0, pts, pts[1], reach))
        assertEquals(2, NavStepLogic.advanceLeg(1, pts, pts[2], reach))
        assertEquals(3, NavStepLogic.advanceLeg(2, pts, pts[3], reach))
    }

    @Test
    fun `does not skip the turnaround on an out-and-back route`() {
        // Maneuvers: depart, turnaround (far east), end (back near the start). On the OUTBOUND
        // leg the end point is far closer in straight-line metres than the turnaround we have
        // not yet reached — the removed "already past" shortcut would have skipped to leg 1.
        val mLng = 111_320.0 * Math.cos(Math.toRadians(32.80))
        val depart     = LatLng(32.80, 35.000)
        val turnaround = LatLng(32.80, 35.000 + 400.0 / mLng)   // 400 m east
        val end        = LatLng(32.80, 35.000 + 9.0 / mLng)     // 9 m east (the return point, near start)
        val route = listOf(depart, turnaround, end)
        val outboundPos = LatLng(32.80, 35.000 + 10.0 / mLng)   // 10 m east, still heading OUT
        // Must NOT skip past the turnaround while still on the outbound leg.
        assertEquals(0, NavStepLogic.advanceLeg(0, route, outboundPos, reach))
    }

    @Test
    fun `never moves backward`() {
        assertEquals(2, NavStepLogic.advanceLeg(2, pts, pts[0], reach))
    }
}
