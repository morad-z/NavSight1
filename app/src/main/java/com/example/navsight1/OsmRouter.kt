package com.example.navsight1

import android.util.Log
import com.google.android.gms.maps.model.LatLng
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/** Internal graph coordinate (lat,lng). Distinct from gms LatLng so the graph
 *  layer carries no Android dependency. */
internal data class GeoPt(val lat: Double, val lng: Double)

/**
 * OsmRouter — the routing seam (MAP_MATCHING_PLAN.md §8M Step A*.3 / K-routing*).
 * Decouples NavigationManager from the concrete router so the BRouter-vs-fallback
 * choice is swappable. Returns the route polyline as gms LatLng (hybrid v1 keeps
 * the Google display + gms LatLng); an empty list means "no route" and the caller
 * falls back to the straight line.
 */
interface OsmRouter {
    suspend fun route(start: LatLng, dest: LatLng): List<LatLng>
}

/**
 * DijkstraRouter — the build-green v1 router: A* / Dijkstra over the Step A*
 * `haifa_graph.bin` graph held by [OsmDataLayer]. The plan's first choice was
 * BRouter, but `brouter-core` is not on Maven Central and no jar is vendored in
 * this tree, so the plan's named DijkstraRouter fallback is the v1 default
 * (§8M Step A*: "haifa_graph.bin is built regardless… so the fallback is always
 * available with zero extra preprocessing"). It reaches the loaded data layer
 * through the process-wide holder [OsmDataLayer.instance] — no construction wiring.
 */
class DijkstraRouter : OsmRouter {
    override suspend fun route(start: LatLng, dest: LatLng): List<LatLng> =
        withContext(Dispatchers.Default) {
            // Roads come purely from the on-demand region around the start point.
            val region = RoadRegionManager.regionFor(start.latitude, start.longitude) ?: run {
                Log.w(TAG, "route: no road region for start yet — caller falls back to straight line")
                return@withContext emptyList<LatLng>()
            }
            region.graph.route(start.latitude, start.longitude, dest.latitude, dest.longitude)
                .map { LatLng(it.lat, it.lng) }
        }

    companion object { private const val TAG = "DijkstraRouter" }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * DEFERRED: BRouterRouter — the plan's preferred offline router (BRouter
 * `brouter-core`, MIT). NOT enabled in this branch because BRouter is not on
 * Maven Central and no jar is vendored under libs/. To enable later:
 *   1. Vendor brouter-core/codec/mapaccess/expressions jars under app/libs/.
 *   2. Add them to libs.versions.toml + build.gradle.kts (measure the A*.1 ≤12MB gate).
 *   3. Ship the Haifa `haifa.rd5` + `.brf` profile produced by brouter-map-creator.
 *   4. Implement the body below against brouter-core's RoutingEngine (see §8M
 *      Step K-routing* step 3 for the call shape) and switch NavSightViewModel's
 *      OsmRouter wiring from DijkstraRouter() to BRouterRouter(segmentDir, profile).
 *
 * class BRouterRouter(...) : OsmRouter {
 *     override suspend fun route(start: LatLng, dest: LatLng): List<LatLng> = ...
 * }
 * ──────────────────────────────────────────────────────────────────────────── */
