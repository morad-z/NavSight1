package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.json.JSONObject
import org.junit.Test
import java.io.File
import kotlin.math.asin
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * DIAGNOSTIC harness (NOT a CI gate) — replays a RECORDED scooter sim's VIO track (vlat/vlng)
 * through the REAL [LocalMatcher] (with the 2026-05-31 rail-lock) + [ManeuverStateMachine], against
 * OSM roads fetched offline for the ride bbox (roads_*.json), and PRINTS the matched-path length vs
 * GPS vs the OLD recorded matcher output (mm_lat/mm_lng in the sim). Proves whether the rail-lock
 * cuts the parallel-road teleport / path inflation on REAL data. Skips (passes) if the data files
 * are absent so it is CI-safe. Run: gradlew :app:testDebugUnitTest --tests "*RailReplayTest*".
 */
class RailReplayTest {

    private val dir = "C:/Users/morad/AndroidStudioProjects/NavSight1/tests/sims/val_2026_05_31_scooter"

    private fun hav(aLat: Double, aLng: Double, bLat: Double, bLng: Double): Double {
        val R = 6371000.0
        val la1 = Math.toRadians(aLat); val la2 = Math.toRadians(bLat)
        val dla = Math.toRadians(bLat - aLat); val dlo = Math.toRadians(bLng - aLng)
        val h = sin(dla / 2) * sin(dla / 2) + cos(la1) * cos(la2) * sin(dlo / 2) * sin(dlo / 2)
        return 2 * R * asin(min(1.0, sqrt(h)))
    }
    private fun pathLen(p: List<DoubleArray>) =
        (1 until p.size).sumOf { hav(p[it - 1][0], p[it - 1][1], p[it][0], p[it][1]) }
    private fun teleports(p: List<DoubleArray>, thr: Double = 30.0) =
        (1 until p.size).count { hav(p[it - 1][0], p[it - 1][1], p[it][0], p[it][1]) > thr }

    private fun region(roadsFile: File): DynamicRoadRegion {
        val arrays = RoadsJsonParser.parse(roadsFile.readText())
        return DynamicRoadRegion(
            SegmentRTreeReader.buildSegmentStore(arrays),
            RoutingGraphReader.buildGraphStore(arrays),
            BBoxDeg.centred(32.7334, 35.1447, 0.05), 0, 0, 0L,
            RoundaboutIndexBuilder.build(arrays)
        )
    }

    /** Replay one sim and print GPS / OLD-mm(recorded) / NEW-mm(rail-lock) path lengths + teleports. */
    private fun replay(simTs: String) {
        val sf = File("$dir/simulation_data_$simTs.json")
        val rf = File("$dir/roads_$simTs.json")
        if (!sf.exists() || !rf.exists()) { println("RAIL_REPLAY $simTs: data absent — skipped"); return }
        val region = region(rf)

        val arr = JSONObject(sf.readText()).getJSONArray("points")
        data class P(val ts: Long, val vlat: Double, val vlng: Double,
                     val glat: Double?, val glng: Double?, val mm: DoubleArray?, val hdgRad: Double)
        val pts = ArrayList<P>()
        for (i in 0 until arr.length()) {
            val o = arr.getJSONObject(i)
            fun d(k: String): Double? = if (o.has(k) && !o.isNull(k)) o.getDouble(k) else null
            val vlat = d("vlat"); val vlng = d("vlng") ?: continue
            if (vlat == null) continue
            val mmLat = d("mm_lat"); val mmLng = d("mm_lng")
            pts.add(P(o.optLong("ts"), vlat, vlng, d("glat"), d("glng"),
                if (mmLat != null && mmLng != null) doubleArrayOf(mmLat, mmLng) else null,
                o.optDouble("hdg", 0.0)))
        }
        if (pts.size < 10) { println("RAIL_REPLAY $simTs: too few points"); return }

        // Replay the app's matcher loop: a snap tick every ~500ms; window = last 150 VIO points
        // subsampled to ~15 + the current point (mirrors NavSightViewModel.launchNetworkSnap).
        val matcher = LocalMatcher(); val sm = ManeuverStateMachine()
        val rail = GraphRailDot(region.graph)   // 2026-05-31 advance-along-rail (ball-in-maze)
        val history = ArrayList<LatLng>()
        val newMm = ArrayList<DoubleArray>()
        val railMm = ArrayList<DoubleArray>()   // the ball-on-rail dot (graph-constrained)
        var lastSnapMs = pts.first().ts - SNAP_MS   // so the first point ticks (avoid MIN_VALUE overflow)
        var prevVio: LatLng? = null
        var divergeTicks = 0; var reacquires = 0; var reacqWayId: Long? = null
        var releases = 0; var ticks = 0; var onRbt = 0
        for (p in pts) {
            history.add(LatLng(p.vlat, p.vlng))
            if (p.ts - lastSnapMs < SNAP_MS) continue   // ts is MILLISECONDS for these sims
            val dtMs = (p.ts - lastSnapMs).coerceAtLeast(1L)
            lastSnapMs = p.ts; ticks++
            val recent = history.takeLast(150)
            val stride = maxOf(1, recent.size / 14)
            val window = recent.filterIndexed { i, _ -> i % stride == 0 }.takeLast(15) + LatLng(p.vlat, p.vlng)
            val man = sm.tick(p.vlat, p.vlng, Math.toDegrees(p.hdgRad), region.roundabouts)
            if (man.state == ManeuverState.ON_ROUNDABOUT) onRbt++
            if (man.event != null) { matcher.releaseRail(); releases++ }
            val r = matcher.match(window, region)
            val accept = r != null && r.confidence >= 0.30 && !r.broken
            if (accept) newMm.add(doubleArrayOf(r!!.matched.latitude, r.matched.longitude))

            // --- ball-on-rail (advance-along-rail): move the ball forward along the graph by the
            // VIO step (CAPPED to a plausible speed×dt so a VIO teleport-spike can't race it), and
            // turn at junctions by the VIO bearing. Divergence is measured against the MATCHER's
            // road choice (window-shape robust), NEVER the raw VIO point (which is itself off-road).
            // Re-acquire (a sanctioned jump) only after the matcher confidently insists on a road
            // far from the ball for several consecutive ticks = a real missed turn / wrong road. ---
            val dVioRaw = prevVio?.let { hav(it.latitude, it.longitude, p.vlat, p.vlng) } ?: 0.0
            val advanceM = minOf(dVioRaw, MAX_SPEED_MPS * dtMs / 1000.0)
            val bearing = if (prevVio != null && dVioRaw > 0.01)
                ManeuverMath.bearingDeg(prevVio!!.latitude, prevVio!!.longitude, p.vlat, p.vlng) else Double.NaN
            val seedBearing = if (bearing.isNaN()) 0.0 else bearing
            if (!rail.acquired) {
                val sLat = if (accept) r!!.matched.latitude else p.vlat
                val sLng = if (accept) r!!.matched.longitude else p.vlng
                rail.acquire(sLat, sLng, seedBearing)
            } else {
                rail.advance(advanceM, bearing)
                // WIRE THE MATCHER (2026-06-02): supervise the ball — re-acquire onto the matcher's road
                // ONLY after sustained (5-tick) high-confidence (≥0.6) divergence (>25 m) from the matcher
                // dot = a confirmed wrong exit / wrong road. Mirrors NavSightViewModel. The long+confident
                // window keeps a transient parallel-road matcher flicker from causing a spurious teleport.
                // NOTE: sim A is a HEADING-DISASTER ride (≤230 m drift, pre-heading-leg) so the matcher is
                // itself unreliable here; the recovery's BENEFIT needs a device re-record where the (now-on)
                // heading leg makes the matcher trustworthy. This offline check only proves it stays BOUNDED.
                val rp = rail.position()
                val mWayId = r?.matchedWayId
                val mOff = if (r != null && rp != null) hav(rp.lat, rp.lng, r.matched.latitude, r.matched.longitude) else 0.0
                if (SUPERVISE && accept && r!!.confidence >= MAP_POS_MIN_CONFIDENCE && mWayId != null && rp != null &&
                    mOff > RAIL_MATCHER_REACQUIRE_M) {
                    // wayId-stability gate: only a STABLE confident disagreement (same way) recovers.
                    if (mWayId == reacqWayId) divergeTicks++ else { reacqWayId = mWayId; divergeTicks = 1 }
                    if (divergeTicks >= RAIL_MATCHER_REACQUIRE_TICKS) {
                        rail.acquire(r.matched.latitude, r.matched.longitude, seedBearing)
                        divergeTicks = 0; reacqWayId = null; reacquires++
                    }
                } else { divergeTicks = 0; reacqWayId = null }
            }
            rail.position()?.let { railMm.add(doubleArrayOf(it.lat, it.lng)) }
            prevVio = LatLng(p.vlat, p.vlng)
        }
        val gps = pts.mapNotNull { if (it.glat != null && it.glng != null) doubleArrayOf(it.glat, it.glng) else null }
        val oldMm = pts.mapNotNull { it.mm }
        val g = pathLen(gps)
        println("RAIL_REPLAY $simTs:  GPS=%.0fm | OLD_mm=%.0fm (%.2fx, teleports=%d) | NEW_mm=%.0fm (%.2fx, teleports=%d)  [ticks=%d, railReleases=%d]"
            .format(g,
                pathLen(oldMm), pathLen(oldMm) / g, teleports(oldMm),
                pathLen(newMm), pathLen(newMm) / g, teleports(newMm),
                newMm.size, releases) + "  onRoundabout=%d/%d (%.0f%%)".format(onRbt, ticks, 100.0 * onRbt / maxOf(1, ticks)))
        println("RAIL_REPLAY $simTs:  BALL-on-rail=%.0fm (%.2fx, teleports=%d, reacquires=%d)  <- graph-constrained, should NOT teleport off-road"
            .format(pathLen(railMm), pathLen(railMm) / g, teleports(railMm), reacquires))
    }

    @Test
    fun `replay scooter sims through the rail-lock matcher vs GPS`() {
        // sim A (085336) is the 727m / 2.09x-inflation ride; roads fetched offline for its bbox.
        replay("1780235085336")
    }

    companion object {
        private const val SNAP_MS = 500L
        private const val MAX_SPEED_MPS = 15.0            // advance cap (54 km/h) — kills VIO teleport-spike racing
        private const val MAP_POS_MIN_CONFIDENCE = 0.6    // matcher-supervision: require a confident match to re-acquire
        private const val RAIL_MATCHER_REACQUIRE_M = 25.0 // ball >25 m off the matcher dot = on a different road
        private const val RAIL_MATCHER_REACQUIRE_TICKS = 5 // sustained ~2.5 s before a wrong-turn re-acquire
        // Mirrors NavSightViewModel.MATCHER_SUPERVISION_ENABLED (shipped OFF). With SUPERVISE=true this sim
        // measured 968 m / 1.33× / 3 teleports (a REGRESSION vs the OFF floor of 599 m / 0.82× / 0) because
        // sim A's matcher is unreliable (bad heading). Kept here so the trade-off is re-measurable.
        private const val SUPERVISE = false
    }
}
