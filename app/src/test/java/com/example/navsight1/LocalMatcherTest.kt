package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

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
}
