package com.example.navsight1

import android.util.Log
import com.google.android.gms.maps.model.LatLng
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL

/**
 * OnlineRouter — full road routing to ANY destination via the free no-key OSM
 * routing service (OSRM, car profile). Provides both the route geometry and
 * turn-by-turn maneuvers, including roundabout exit numbers and U-turns.
 *
 * Two entry points:
 *  - [route]          → polyline only (OsmRouter interface; used for the search preview).
 *  - [routeWithSteps] → full [NavigationRoute] with turn-by-turn steps (navigation).
 *
 * Falls back to the local dynamic-region [DijkstraRouter] when offline (nearby
 * destinations still route from the loaded region). $0, no key — the free OSM
 * "Directions API".
 */
class OnlineRouter : OsmRouter {

    override suspend fun route(start: LatLng, dest: LatLng): List<LatLng> =
        withContext(Dispatchers.IO) {
            try {
                val poly = parseOsrmRoute(fetch(start, dest, steps = false), start, dest)?.polyline
                if (poly != null && poly.size >= 2) return@withContext poly
            } catch (e: Exception) {
                Log.w(TAG, "online route failed (${e.message}) — local region fallback")
            }
            DijkstraRouter().route(start, dest)
        }

    /** Full route with turn-by-turn steps (roundabouts/U-turns included). null on failure. */
    suspend fun routeWithSteps(start: LatLng, dest: LatLng): NavigationRoute? =
        withContext(Dispatchers.IO) {
            try {
                val r = parseOsrmRoute(fetch(start, dest, steps = true), start, dest)
                if (r != null && r.polyline.size >= 2) return@withContext r
                Log.w(TAG, "online route empty")
            } catch (e: Exception) {
                Log.w(TAG, "online route failed (${e.message})")
            }
            // Offline fallback: a road polyline from the local region (no turn steps).
            val poly = DijkstraRouter().route(start, dest)
            if (poly.size >= 2) {
                NavigationRoute(start, dest, listOf(straightStep(start, dest, poly)),
                    polylineLengthM(poly), (polylineLengthM(poly) / 10.0).toInt(), poly)
            } else null
        }

    private fun fetch(start: LatLng, dest: LatLng, steps: Boolean): String {
        val url = "https://router.project-osrm.org/route/v1/driving/" +
            "${start.longitude},${start.latitude};${dest.longitude},${dest.latitude}" +
            "?overview=full&geometries=geojson&steps=${if (steps) "true" else "false"}"
        val conn = (URL(url).openConnection() as HttpURLConnection).apply {
            connectTimeout = 8000; readTimeout = 8000
            setRequestProperty("User-Agent", "navsight-android/1.0")
        }
        return try { conn.inputStream.bufferedReader().use { it.readText() } } finally { conn.disconnect() }
    }

    // ── pure parsing (unit-tested) ──────────────────────────────────────────────
    /** Parse an OSRM JSON response into a [NavigationRoute]. Pure (no HTTP). */
    fun parseOsrmRoute(json: String, start: LatLng, dest: LatLng): NavigationRoute? {
        val root = JSONObject(json)
        if (root.optString("code") != "Ok") return null
        val routes = root.optJSONArray("routes") ?: return null
        if (routes.length() == 0) return null
        val r0 = routes.getJSONObject(0)
        val polyline = coordsToLatLng(r0.optJSONObject("geometry")?.optJSONArray("coordinates"))
        if (polyline.size < 2) return null
        val distanceM = r0.optDouble("distance", 0.0)
        val durationS = r0.optDouble("duration", distanceM / 10.0).toInt()

        val steps = ArrayList<RouteStep>()
        val legs = r0.optJSONArray("legs")
        if (legs != null && legs.length() > 0) {
            val osrmSteps = legs.getJSONObject(0).optJSONArray("steps")
            if (osrmSteps != null) {
                for (i in 0 until osrmSteps.length()) {
                    val s = osrmSteps.optJSONObject(i) ?: continue
                    val man = s.optJSONObject("maneuver") ?: continue
                    val type = man.optString("type")
                    val modifier = man.optString("modifier")
                    val exit = man.optInt("exit", -1)
                    // Many OSM roads (residential/service/unclassified) have no `name` tag —
                    // keep it blank and let buildInstruction / the banner omit "onto …" rather
                    // than fabricate "Unknown road" on every such step.
                    val name = s.optString("name")
                    val loc = arrToLatLng(man.optJSONArray("location")) ?: continue
                    val geom = coordsToLatLng(s.optJSONObject("geometry")?.optJSONArray("coordinates"))
                    val end = geom.lastOrNull() ?: loc
                    if (type == "exit roundabout" || type == "exit rotary") continue // folded into the roundabout step
                    steps.add(
                        RouteStep(
                            startLocation = loc, endLocation = end,
                            distance = s.optDouble("distance", 0.0),
                            maneuver = mapManeuver(type, modifier),
                            streetName = name,
                            instruction = buildInstruction(type, modifier, exit, name)
                        )
                    )
                }
            }
        }
        if (steps.isEmpty()) steps.add(straightStep(start, dest, polyline))
        return NavigationRoute(start, dest, steps, distanceM, durationS, polyline)
    }

    private fun mapManeuver(type: String, modifier: String): ManeuverType {
        if (modifier == "uturn") return ManeuverType.UTURN_LEFT // drive-right: U-turns go left
        return when (type) {
            "roundabout", "rotary", "roundabout turn" -> when {
                // OSRM encodes the EXIT direction in the modifier. "straight" (continue
                // through) must NOT render as a right turn — that was the bug.
                modifier.contains("left") -> ManeuverType.ROUNDABOUT_LEFT
                modifier.contains("right") -> ManeuverType.ROUNDABOUT_RIGHT
                else -> ManeuverType.ROUNDABOUT_STRAIGHT
            }
            "merge" -> ManeuverType.MERGE
            "on ramp", "off ramp" ->
                if (modifier.contains("left")) ManeuverType.RAMP_LEFT else ManeuverType.RAMP_RIGHT
            "fork" -> if (modifier.contains("left")) ManeuverType.FORK_LEFT else ManeuverType.FORK_RIGHT
            else -> when (modifier) {
                "sharp left" -> ManeuverType.TURN_SHARP_LEFT
                "sharp right" -> ManeuverType.TURN_SHARP_RIGHT
                "slight left" -> ManeuverType.TURN_SLIGHT_LEFT
                "slight right" -> ManeuverType.TURN_SLIGHT_RIGHT
                "left" -> ManeuverType.TURN_LEFT
                "right" -> ManeuverType.TURN_RIGHT
                else -> ManeuverType.STRAIGHT
            }
        }
    }

    private fun buildInstruction(type: String, modifier: String, exit: Int, name: String): String {
        // " onto <road>" only when the road actually has a name (most OSM roads don't).
        val onto = if (name.isBlank()) "" else " onto $name"
        return when (type) {
            "roundabout", "rotary", "roundabout turn" ->
                if (exit > 0) "At the roundabout, take exit $exit$onto" else "At the roundabout, continue$onto"
            "depart" -> if (name.isBlank()) "Head out" else "Head onto $name"
            "arrive" -> "Arrive at your destination"
            "merge" -> "Merge$onto"
            "on ramp", "off ramp" -> "Take the ramp$onto"
            "fork" -> "Keep ${modifier.ifBlank { "straight" }}$onto"
            else -> when {
                modifier == "uturn" -> "Make a U-turn$onto"
                modifier.isBlank() || modifier == "straight" -> if (name.isBlank()) "Continue" else "Continue onto $name"
                else -> "Turn $modifier$onto"
            }
        }
    }

    private fun coordsToLatLng(arr: org.json.JSONArray?): List<LatLng> {
        if (arr == null) return emptyList()
        val out = ArrayList<LatLng>(arr.length())
        for (i in 0 until arr.length()) arrToLatLng(arr.optJSONArray(i))?.let { out.add(it) }
        return out
    }

    private fun arrToLatLng(c: org.json.JSONArray?): LatLng? {
        if (c == null || c.length() < 2) return null
        return LatLng(c.getDouble(1), c.getDouble(0)) // [lon,lat] → LatLng(lat,lon)
    }

    private fun straightStep(start: LatLng, dest: LatLng, poly: List<LatLng>) = RouteStep(
        startLocation = start, endLocation = dest,
        distance = polylineLengthM(poly), maneuver = ManeuverType.STRAIGHT,
        streetName = "Route", instruction = "Follow the route"
    )

    private fun polylineLengthM(poly: List<LatLng>): Double {
        var d = 0.0
        for (i in 1 until poly.size)
            d += RoadSnapMath.haversineM(poly[i - 1].latitude, poly[i - 1].longitude, poly[i].latitude, poly[i].longitude)
        return d
    }

    companion object { private const val TAG = "OnlineRouter" }
}
