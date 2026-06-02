package com.example.navsight1

import android.util.Log
import com.google.android.gms.maps.model.LatLng
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONArray
import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL
import kotlin.math.roundToInt

/**
 * LiveMatcher — trajectory map-matching via OSRM's `/match` HMM service (free, no
 * key; the Newson-Krumm 2009 algorithm). Instead of snapping each point independently
 * it matches the recent VIO PATH to the road network, so the dot follows curves and
 * roundabout rings instead of "letting go" past the per-point soft-snap cap.
 *
 * Industry-standard usage (Tier 1):
 *   #1 BEARINGS — we hand OSRM the per-point direction of travel (our strongest VIO
 *      signal). The HMM folds heading agreement into the emission probability, which
 *      disambiguates parallel roads / divided highways / roundabout exit-vs-through.
 *   #3 GEOMETRY — overview=full returns the matched ROAD polyline (not just the dot),
 *      so the trail can be rendered on-road, and its length is a free distance read.
 *   #4 CONFIDENCE — OSRM reports a per-trace confidence + splits broken traces into
 *      multiple matchings; the caller gates on both.
 *
 * Complementary to [ManeuverStateMachine] (heading-based maneuver detection): this
 * only decides the dot POSITION + matched geometry. Returns null (→ per-point
 * [RoadSnapper] fallback) when offline / no match.
 */
class LiveMatcher {

    /**
     * Map-match the trajectory [window] (oldest→newest geographic points) and return
     * the matched road position of the LATEST point + the matched geometry, or null on
     * failure / no match.
     */
    suspend fun match(window: List<LatLng>): MatchResult? = withContext(Dispatchers.IO) {
        if (window.size < MIN_POINTS) return@withContext null
        try {
            val coords = window.joinToString(";") { "${it.longitude},${it.latitude}" }
            val radiuses = window.joinToString(";") { RADIUS_M.toString() }
            // NOTE (2026-05-31): `bearings` DISABLED — it was destroying the match. Measured
            // live on the OSRM demo server: the SAME window scored confidence 0.96 with
            // radiuses-only vs 0.06 WITH bearings. Our per-point bearings are geometric
            // directions between dense (2-3 m), noisy, scale-distorted VIO points; OSRM's HMM
            // emission punishes the heading mismatch and collapses the match below our gate,
            // so every match was rejected → permanent per-point fallback → dot off-street.
            // buildBearings() is kept (tested) for a future clean per-point heading source.
            // val bearings = buildBearings(window)
            val url = "https://router.project-osrm.org/match/v1/driving/$coords" +
                "?geometries=geojson&overview=full&tidy=true&gaps=ignore&radiuses=$radiuses"
            val conn = (URL(url).openConnection() as HttpURLConnection).apply {
                connectTimeout = TIMEOUT_MS; readTimeout = TIMEOUT_MS
                setRequestProperty("User-Agent", "navsight-android/1.0")
            }
            val text = try { conn.inputStream.bufferedReader().use { it.readText() } } finally { conn.disconnect() }
            parse(text)
        } catch (e: Exception) {
            Log.w(TAG, "match failed (${e.message}) — caller falls back to per-point snap")
            null
        }
    }

    /** Pure parse of an OSRM /match response → latest matched point + geometry + stats. */
    fun parse(json: String): MatchResult? {
        val root = JSONObject(json)
        if (root.optString("code") != "Ok") return null

        val tps: JSONArray = root.optJSONArray("tracepoints") ?: return null
        // The last non-null tracepoint is the matched location of the most-recent
        // (matchable) input point — that's the dot.
        var dot: LatLng? = null
        for (i in tps.length() - 1 downTo 0) {
            val tp = tps.optJSONObject(i) ?: continue
            val loc = tp.optJSONArray("location") ?: continue
            if (loc.length() < 2) continue
            dot = LatLng(loc.getDouble(1), loc.getDouble(0)); break
        }
        if (dot == null) return null

        // Matched road geometry + distance. A broken trace splits into >1 matchings;
        // we concatenate their geometries and sum their distances, and flag `broken`.
        val matchings = root.optJSONArray("matchings")
        val broken = (matchings?.length() ?: 0) > 1
        val confidence = matchings?.optJSONObject(0)?.optDouble("confidence", 0.0) ?: 0.0
        var routeDistanceM = 0.0
        val path = ArrayList<LatLng>()
        if (matchings != null) {
            for (mi in 0 until matchings.length()) {
                val m = matchings.optJSONObject(mi) ?: continue
                routeDistanceM += m.optDouble("distance", 0.0)
                val coords = m.optJSONObject("geometry")?.optJSONArray("coordinates") ?: continue
                for (ci in 0 until coords.length()) {
                    val c = coords.optJSONArray(ci) ?: continue
                    if (c.length() >= 2) path.add(LatLng(c.getDouble(1), c.getDouble(0)))
                }
            }
        }
        return MatchResult(dot, confidence, path, routeDistanceM, broken)
    }

    /**
     * OSRM `bearings=b,range;…` — the per-point direction of travel (#1). For each
     * point we use the incoming segment bearing (prev→cur), skipped (empty entry) when
     * the segment is too short to give a reliable direction (stationary / first point).
     * Pure + side-effect-free so it is unit-testable.
     */
    fun buildBearings(window: List<LatLng>): String = window.indices.joinToString(";") { i ->
        if (i == 0) return@joinToString ""
        val a = window[i - 1]; val b = window[i]
        if (RoadSnapMath.haversineM(a.latitude, a.longitude, b.latitude, b.longitude) < MIN_BEARING_SEG_M) return@joinToString ""
        val brg = ManeuverMath.bearingDeg(a.latitude, a.longitude, b.latitude, b.longitude).roundToInt() % 360
        "$brg,$BEARING_RANGE_DEG"
    }

    data class MatchResult(
        val matched: LatLng,
        val confidence: Double,
        val matchedPath: List<LatLng>,
        val routeDistanceM: Double,
        val broken: Boolean,
        // 2026-05-31 — the OSM way the dot was matched onto (LocalMatcher rail-lock).
        // Null for matchers that don't track it (LiveMatcher). Lets the caller keep the
        // matcher "railed" to the same road across ticks instead of re-snapping per window.
        val matchedWayId: Long? = null
    )

    companion object {
        private const val TAG = "LiveMatcher"
        private const val MIN_POINTS = 4      // need a short trajectory to disambiguate
        private const val RADIUS_M = 25       // per-point search radius (~VIO/GPS precision)
        private const val MIN_BEARING_SEG_M = 3.0  // below this the segment bearing is noise
        private const val BEARING_RANGE_DEG = 60   // ± cone OSRM allows around our heading
        private const val TIMEOUT_MS = 6000
    }
}
