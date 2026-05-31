package com.example.navsight1

import android.util.Log
import android.util.LruCache
import androidx.annotation.Nullable
import androidx.annotation.NonNull
import com.google.android.gms.maps.model.LatLng
// MIGRATION 2026-05-30 (MAP_MATCHING_PLAN.md §8M Step C*): Google Roads API
// removed; snapping now runs fully offline against the local Haifa OSM R-tree
// (OsmDataLayer). The gms LatLng import is RETAINED — hybrid v1 keeps the Google
// display + gms LatLng (no GeoPoint refactor).
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlin.math.exp

/**
 * RoadSnapper
 *
 * Snaps VIO-derived positions to the road network using the local, offline OSM
 * road graph (Haifa OSM R-tree in [OsmDataLayer]). $0, no key, no network,
 * jamming-immune (MAP_MATCHING_PLAN.md §8M Step C*).
 *
 * This is a like-for-like replacement of the old per-point Google Roads snapper.
 * The public surface is UNCHANGED so the call sites stay byte-identical:
 *   - ctor `RoadSnapper(apiKey: String)`  (apiKey is now ignored — no key needed)
 *   - `suspend fun snapToRoad(vioPosition, recentPath): SnappedLatLng`
 *   - `fun shutdown()`
 *   - the `SnappedLatLng` shape
 *
 * Features:
 * - LRU cache to reduce queries (50 recent points)
 * - Graceful fallback when the data layer isn't loaded yet (trust raw VIO)
 * - >15 m soft-snap: trust raw VIO when far from any road
 *
 * Documented limitation (accepted for v1, per §0.7): snap-to-nearest corner-cuts
 * and is bearing-blind — fixed later by the deferred Step D HMM. NOT a regression
 * (the old Google snapper was also per-point).
 *
 * Usage:
 *   val snapper = RoadSnapper(apiKey = "")   // apiKey ignored
 *   val snapped = snapper.snapToRoad(currentPosition, recentPath)
 */
class RoadSnapper(private val apiKey: String) {
    // MIGRATION: apiKey ignored — OSM snap needs no key. Kept in the ctor so
    // NavSightViewModel:63 (`RoadSnapper(apiKey = apiKey)`) stays byte-identical.

    companion object {
        private const val TAG = "RoadSnapper"
        private const val CACHE_SIZE = 50
        // OSM way-geometry accuracy floor (m) — https://wiki.openstreetmap.org/wiki/Accuracy.
        // confidence = exp(-distanceM / SNAP_SIGMA_M); also sets the search radius.
        private const val SNAP_SIGMA_M = 5.0
        // Search radius = 5σ = 25 m. Beyond this there is no candidate road → unsnapped.
        private const val SNAP_SEARCH_RADIUS_M = 5.0 * SNAP_SIGMA_M
        // Soft-snap threshold (m): if the nearest road is farther than this we are
        // likely on a sidewalk/plaza/indoors → trust raw VIO. Derived from (== ) the
        // search radius: candidates are already bounded at SNAP_SEARCH_RADIUS_M, so any
        // candidate the search returns is snappable. The previous tighter 15 m second-
        // gate was Google-GPS-tuned and made the VIO dot "let go" on curves once raw
        // drift exceeded 15 m (user obs 2026-05-30). This path is now the OFFLINE
        // FALLBACK only — the primary dot source is the OSRM /match trajectory matcher.
        private const val SOFT_SNAP_MAX_M = SNAP_SEARCH_RADIUS_M

        // Dynamic-region swap epoch: RoadRegionManager bumps this on every region
        // swap; each snapper instance evicts its LRU when it observes a new epoch so
        // a stale snap from a previous region never leaks in.
        @Volatile
        private var swapEpoch = 0
        fun onRegionSwapped() { swapEpoch++ }
    }

    private var seenEpoch = 0

    /* LEGACY (Google Roads API, removed in OSM migration 2026-05-30, §8M Step C*).
       Kept commented (project rule: comment out, don't delete) so the diff shows
       exactly what the local snap replaced.

    private val geoApiContext: GeoApiContext? by lazy {
        if (apiKey.isBlank()) {
            Log.e(TAG, "Roads API key is blank — road snapping will be disabled")
            null
        } else {
            GeoApiContext.Builder().apiKey(apiKey).build()
        }
    }
    private val snappedPoints = RoadsApi.snapToRoads(ctx, true, *geoPoints).await()
    */

    // LRU cache to store recent snapped positions
    private val snapCache = LruCache<String, SnappedLatLng>(CACHE_SIZE)

    // Track query-failure state to reduce log spam (now wraps the local R-tree query).
    private var consecutiveFailures = 0
    private var lastErrorMessage: String? = null
    private var dataLayerMissingLogged = false

    /**
     * Snaps a VIO position to the nearest OSM road segment.
     *
     * @param vioPosition Current VIO-derived position
     * @param recentPath Recent path history — accepted-but-unused in v1 snap-to-nearest
     *        (kept in the signature so NavSightViewModel:487 stays byte-identical;
     *        used again by the deferred Step D HMM).
     * @return Snapped position, or the raw VIO position if snapping is unavailable/far.
     */
    suspend fun snapToRoad(
        vioPosition: LatLng,
        recentPath: List<LatLng> = emptyList(),
        maneuver: ManeuverResult? = null
    ): SnappedLatLng = withContext(Dispatchers.IO) {

        // Drop cached snaps from a previous dynamic road region (cross-region staleness).
        if (seenEpoch != swapEpoch) { snapCache.evictAll(); seenEpoch = swapEpoch }

        // Check cache first
        val cacheKey = getCacheKey(vioPosition)
        snapCache.get(cacheKey)?.let { cached ->
            Log.d(TAG, "Cache hit for position: $vioPosition")
            return@withContext cached
        }

        try {
            // Roads are loaded purely on-demand around the user's location
            // (RoadRegionManager) — no embedded region. If no region covers this
            // point yet (still fetching, or off-grid), trust raw VIO.
            val region = RoadRegionManager.regionFor(vioPosition.latitude, vioPosition.longitude)
            if (region == null) {
                if (!dataLayerMissingLogged) {
                    Log.w(TAG, "no road region loaded for this location yet — using raw VIO")
                    dataLayerMissingLogged = true
                }
                return@withContext rawUnsnapped(vioPosition)
            }
            // Heading-aware matcher (§0.7): when the state machine says we're ON a
            // roundabout, pin the snap candidates to that ring's way so the dot
            // follows the ring geometry instead of corner-cutting to a crossing road.
            val onRing = maneuver?.state == ManeuverState.ON_ROUNDABOUT && maneuver.activeRing != null
            val rawCandidates = region.segments.nearestSegments(
                vioPosition.latitude, vioPosition.longitude,
                radiusM = if (onRing) SNAP_SEARCH_RADIUS_M * 2.0 else SNAP_SEARCH_RADIUS_M,
                maxResults = if (onRing) 8 else 1
            )
            val candidates: List<RoadSegment> = if (onRing) {
                val ringWay = maneuver!!.activeRing!!.wayId
                rawCandidates.filter { it.wayId == ringWay }.ifEmpty { rawCandidates.take(1) }
            } else rawCandidates
            if (consecutiveFailures > 0) {
                Log.i(TAG, "OSM snap recovered after $consecutiveFailures failures")
                consecutiveFailures = 0
                lastErrorMessage = null
            }
            if (candidates.isEmpty()) {
                // No road within 25 m (building/plaza/indoors) → trust raw VIO.
                return@withContext rawUnsnapped(vioPosition)
            }

            val seg = candidates[0]
            val proj = RoadSnapMath.projectPointOntoSegment(
                vioPosition.latitude, vioPosition.longitude,
                seg.aLat, seg.aLng, seg.bLat, seg.bLng
            )

            // >15 m soft-snap: too far from the road → trust raw VIO.
            if (proj.distanceM > SOFT_SNAP_MAX_M) {
                Log.d(TAG, "Soft-Snap: nearest road %.1fm (>%.0fm). Trusting raw VIO."
                    .format(proj.distanceM, SOFT_SNAP_MAX_M))
                return@withContext rawUnsnapped(vioPosition)
            }

            val confidence = exp(-proj.distanceM / SNAP_SIGMA_M)
            val snappedLatLng = SnappedLatLng(
                latitude = proj.snappedLat,
                longitude = proj.snappedLng,
                // OSM way id, namespaced so any non-null check still works and a
                // human can tell it's an OSM snap. No analogue for Google's batch index.
                placeId = "osm:way:${seg.wayId}",
                originalIndex = null,
                isSnapped = true
            )
            snapCache.put(cacheKey, snappedLatLng)
            Log.d(TAG, "OSM snap %.1fm conf %.2f way %d".format(proj.distanceM, confidence, seg.wayId))
            return@withContext snappedLatLng

        } catch (e: Exception) {
            consecutiveFailures++
            val errorMsg = e.message ?: e.javaClass.simpleName
            when {
                consecutiveFailures == 1 ->
                    Log.e(TAG, "OSM snap error (will retry): $errorMsg", e)
                errorMsg != lastErrorMessage -> {
                    Log.e(TAG, "OSM snap error changed: $errorMsg ($consecutiveFailures consecutive)", e)
                    lastErrorMessage = errorMsg
                }
                consecutiveFailures % 20 == 0 ->
                    Log.w(TAG, "OSM snap still failing: $errorMsg ($consecutiveFailures consecutive)")
            }
            // Graceful fallback: return unsnapped position
            return@withContext rawUnsnapped(vioPosition)
        }
    }

    /** Raw VIO position as an unsnapped result (identical shape to the old paths). */
    private fun rawUnsnapped(vioPosition: LatLng): SnappedLatLng = SnappedLatLng(
        latitude = vioPosition.latitude,
        longitude = vioPosition.longitude,
        placeId = null,
        originalIndex = null,
        isSnapped = false
    )

    /**
     * Generates a cache key from a LatLng position.
     * Uses 6 decimal places (~0.1m precision) for reasonable cache hits.
     */
    private fun getCacheKey(latLng: LatLng): String {
        return "%.6f,%.6f".format(latLng.latitude, latLng.longitude)
    }

    // DEAD CODE: never called from anywhere
    // fun clearCache() {
    //     snapCache.evictAll()
    //     Log.d(TAG, "Snap cache cleared")
    // }

    /**
     * No-op now that there is no GeoApiContext to close. Kept as a method with the
     * same name so NavSightViewModel:825 (`roadSnapper.shutdown()`) is untouched.
     */
    fun shutdown() {
        Log.d(TAG, "RoadSnapper shutdown")
    }
}

/**
 * Data class representing a snapped position.
 *
 * @property latitude Latitude of snapped position
 * @property longitude Longitude of snapped position
 * @property placeId Google Place ID of the road segment (if snapped)
 * @property originalIndex Original index in the batch (if snapped)
 * @property isSnapped Whether the position was successfully snapped to a road
 */
data class SnappedLatLng(
    val latitude: Double,
    val longitude: Double,
    val placeId: String?,
    val originalIndex: Int?,
    val isSnapped: Boolean
) {
    /**
     * Converts to Google Maps LatLng.
     */
    fun toLatLng(): LatLng = LatLng(latitude, longitude)

    /**
     * Calculates distance to another position in meters.
     */
    fun distanceTo(other: SnappedLatLng): Double {
        val earthRadius = 6371000.0 // meters
        val dLat = Math.toRadians(other.latitude - latitude)
        val dLng = Math.toRadians(other.longitude - longitude)
        val a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(Math.toRadians(latitude)) * Math.cos(Math.toRadians(other.latitude)) *
                Math.sin(dLng / 2) * Math.sin(dLng / 2)
        val c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
        return earthRadius * c
    }
}
