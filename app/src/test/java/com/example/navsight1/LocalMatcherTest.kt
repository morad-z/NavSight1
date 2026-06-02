package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

/**
 * Tests the pure Viterbi core ([LocalMatcher.matchCandidates]) — the offline equivalent of
 * OSRM /match. JVM-pure (gms LatLng OK in unit tests); no road region needed.
 */
class LocalMatcherTest {

    private val m = LocalMatcher()
    private val mPerDegLat = 111_320.0
    private fun mPerDegLng(lat: Double) = mPerDegLat * Math.cos(Math.toRadians(lat))

    // 6 trace points heading east at lat 32.80, ~20 m apart.
    private val window: List<LatLng> = (0 until 6).map {
        LatLng(32.80, 35.000 + it * 20.0 / mPerDegLng(32.80))
    }

    @Test
    fun `picks the nearer road and never flickers to the parallel road`() {
        // At every point: a TRUE road ~5 m away (wayId 1) and a PARALLEL road ~25 m away (wayId 2).
        val cands = window.map { p ->
            listOf(
                LocalMatcher.Candidate(p.latitude, p.longitude, 5.0, 1L),                       // true road
                LocalMatcher.Candidate(p.latitude + 25.0 / mPerDegLat, p.longitude, 25.0, 2L)   // parallel road
            )
        }
        val r = m.matchCandidates(window, cands)
        assertNotNull(r)
        // The dot (last matched point) is on the TRUE road, not 25 m north on the parallel one.
        assertEquals(window.last().latitude, r!!.matched.latitude, 1e-6)
        assertEquals(6, r.matchedPath.size)
        assertTrue("confidence ${r.confidence} should be high (~0.78)", r.confidence > 0.7)
    }

    @Test
    fun `confidence drops as the trace sits farther from the road`() {
        val cands = window.map { p -> listOf(LocalMatcher.Candidate(p.latitude, p.longitude, 20.0, 1L)) }
        val r = m.matchCandidates(window, cands)
        assertNotNull(r)
        // mean offset 20 m, σ=20 → confidence ≈ exp(-1) ≈ 0.37.
        assertEquals(0.37, r!!.confidence, 0.05)
    }

    @Test
    fun `too few points returns null`() {
        val short = window.take(3)
        val cands = short.map { p -> listOf(LocalMatcher.Candidate(p.latitude, p.longitude, 5.0, 1L)) }
        assertNull(m.matchCandidates(short, cands))
    }

    @Test
    fun `an off-road gap restarts the trellis without crashing`() {
        // Middle point has NO candidate (off-road) — must still match the rest.
        val cands = window.mapIndexed { i, p ->
            if (i == 2) emptyList() else listOf(LocalMatcher.Candidate(p.latitude, p.longitude, 6.0, 1L))
        }
        val r = m.matchCandidates(window, cands)
        assertNotNull(r)
        assertEquals(window.last().latitude, r!!.matched.latitude, 1e-6)
    }

    // ── RAIL-LOCK (2026-05-31): follow the road, not the drifting trace ──────────
    // Trace drifts NORTH along its length so the LATER points sit closer to a PARALLEL road
    // (the exact correlated-drift case that teleported the dot onto a parallel road and inflated
    // the matched path to 2.09x GPS on the scooter sims). True road = way 1 (lat 32.80), parallel
    // road = way 2, 25 m north.
    private val driftM = doubleArrayOf(0.0, 6.0, 12.0, 18.0, 24.0, 30.0)
    private val driftWindow: List<LatLng> = (0 until 6).map {
        LatLng(32.80 + driftM[it] / mPerDegLat, 35.000 + it * 20.0 / mPerDegLng(32.80))
    }
    private val driftCands: List<List<LocalMatcher.Candidate>> = (0 until 6).map { i ->
        listOf(
            LocalMatcher.Candidate(32.80, driftWindow[i].longitude, driftM[i], 1L),                          // true road
            LocalMatcher.Candidate(32.80 + 25.0 / mPerDegLat, driftWindow[i].longitude, abs(25.0 - driftM[i]), 2L) // parallel
        )
    }

    @Test
    fun `without a rail the drifting trace jumps to the parallel road (the bug)`() {
        val r = m.matchCandidates(driftWindow, driftCands)   // railWayId = null (today's behavior)
        assertNotNull(r)
        assertEquals("free matcher follows the drift onto the parallel road", 2L, r!!.matchedWayId)
    }

    @Test
    fun `rail-lock keeps the dot on the committed road despite the drift`() {
        val r = m.matchCandidates(driftWindow, driftCands, railWayId = 1L)
        assertNotNull(r)
        assertEquals("railed matcher stays on the road it committed to", 1L, r!!.matchedWayId)
        // every chosen point stays on the railed way → no lateral teleport to the parallel road
        assertTrue("railed path should hug the true road (north of it by <=drift)",
            r.matchedPath.last().latitude <= 32.80 + 1e-6)
    }

    @Test
    fun `matchedWayId is reported on a normal match`() {
        val cands = window.map { p -> listOf(LocalMatcher.Candidate(p.latitude, p.longitude, 5.0, 7L)) }
        val r = m.matchCandidates(window, cands)
        assertNotNull(r)
        assertEquals(7L, r!!.matchedWayId)
    }
}
