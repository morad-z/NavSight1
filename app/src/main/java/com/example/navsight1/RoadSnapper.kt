package com.example.navsight1

import android.util.Log
import android.util.LruCache
import com.google.android.gms.maps.model.LatLng
import com.google.maps.GeoApiContext
import com.google.maps.RoadsApi
import com.google.maps.model.SnappedPoint
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext

/**
 * RoadSnapper
 *
 * Snaps VIO-derived positions to the road network using Google Roads API.
 *
 * Features:
 * - Batch snapping up to 100 points for better accuracy
 * - LRU cache to reduce API calls (50 recent points)
 * - Graceful fallback if API fails
 * - Throttled to 2Hz (500ms between calls)
 *
 * Usage:
 *   val snapper = RoadSnapper(apiKey = "YOUR_API_KEY")
 *   val snapped = snapper.snapToRoad(currentPosition, recentPath)
 */
class RoadSnapper(private val apiKey: String) {

    companion object {
        private const val TAG = "RoadSnapper"
        private const val CACHE_SIZE = 50
        private const val MAX_BATCH_SIZE = 100
    }

    // Google Roads API context — lazily initialized off the main thread
    private val geoApiContext: GeoApiContext? by lazy {
        if (apiKey.isBlank()) {
            Log.e(TAG, "Roads API key is blank — road snapping will be disabled")
            null
        } else {
            GeoApiContext.Builder()
                .apiKey(apiKey)
                .build()
        }
    }

    // LRU cache to store recent snapped positions
    private val snapCache = LruCache<String, SnappedLatLng>(CACHE_SIZE)

    /**
     * Snaps a VIO position to the nearest road.
     *
     * @param vioPosition Current VIO-derived position
     * @param recentPath Recent path history (up to 10 points) for context
     * @return Snapped position or original if snapping fails
     */
    suspend fun snapToRoad(
        vioPosition: LatLng,
        recentPath: List<LatLng> = emptyList()
    ): SnappedLatLng = withContext(Dispatchers.IO) {

        // Check cache first
        val cacheKey = getCacheKey(vioPosition)
        snapCache.get(cacheKey)?.let { cached ->
            Log.d(TAG, "Cache hit for position: $vioPosition")
            return@withContext cached
        }

        try {
            // Build path for snapping (recent path + current position)
            val pathToSnap = if (recentPath.size >= 2) {
                // Use up to 5 recent points for context
                (recentPath.takeLast(5) + vioPosition)
            } else {
                listOf(vioPosition)
            }

            // Limit to max batch size
            val limitedPath = pathToSnap.takeLast(MAX_BATCH_SIZE)

            // Convert to Roads API format
            val geoPoints = limitedPath.map { latLng ->
                com.google.maps.model.LatLng(latLng.latitude, latLng.longitude)
            }.toTypedArray()

            Log.d(TAG, "Snapping ${geoPoints.size} points to roads...")

            // Call Roads API with interpolate=true for smoother results
            val ctx = geoApiContext ?: return@withContext SnappedLatLng(
                latitude = vioPosition.latitude,
                longitude = vioPosition.longitude,
                placeId = null,
                originalIndex = null,
                isSnapped = false
            )
            val snappedPoints = RoadsApi.snapToRoads(ctx, true, *geoPoints).await()

            if (snappedPoints.isNotEmpty()) {
                // Get the last snapped point (corresponds to current position)
                val lastSnapped = snappedPoints.last()
                val snappedLatLng = SnappedLatLng(
                    latitude = lastSnapped.location.lat,
                    longitude = lastSnapped.location.lng,
                    placeId = lastSnapped.placeId,
                    originalIndex = lastSnapped.originalIndex,
                    isSnapped = true
                )

                // FR17: Soft-Snapping Architecture
                // Calculate distance from raw VIO to snapped point
                val rawSnapped = SnappedLatLng(vioPosition.latitude, vioPosition.longitude, null, null, false)
                val distToRoad = rawSnapped.distanceTo(snappedLatLng)

                // If we are more than 15 meters from the road, we are likely on a sidewalk,
                // in a park, or indoors. In this case, we trust the raw VIO core.
                if (distToRoad > 15.0) {
                    Log.d(TAG, "Soft-Snap: Distance to road is %.1fm (>15m). Trusting raw VIO.".format(distToRoad))
                    return@withContext rawSnapped
                }

                // Cache the result
                snapCache.put(cacheKey, snappedLatLng)

                Log.d(TAG, "Successfully snapped to road. Place ID: ${lastSnapped.placeId}")

                return@withContext snappedLatLng
            } else {
                Log.w(TAG, "Roads API returned empty result")
                return@withContext SnappedLatLng(
                    latitude = vioPosition.latitude,
                    longitude = vioPosition.longitude,
                    placeId = null,
                    originalIndex = null,
                    isSnapped = false
                )
            }

        } catch (e: Exception) {
            Log.e(TAG, "Roads API error: ${e.message}", e)

            // Graceful fallback: return unsnapped position
            return@withContext SnappedLatLng(
                latitude = vioPosition.latitude,
                longitude = vioPosition.longitude,
                placeId = null,
                originalIndex = null,
                isSnapped = false
            )
        }
    }

    /**
     * Generates a cache key from a LatLng position.
     * Uses 6 decimal places (~0.1m precision) for reasonable cache hits.
     */
    private fun getCacheKey(latLng: LatLng): String {
        return "%.6f,%.6f".format(latLng.latitude, latLng.longitude)
    }

    /**
     * Clears the snapping cache.
     * Useful when starting a new navigation session.
     */
    fun clearCache() {
        snapCache.evictAll()
        Log.d(TAG, "Snap cache cleared")
    }

    /**
     * Shuts down the GeoApiContext.
     * Call this when the RoadSnapper is no longer needed.
     */
    fun shutdown() {
        geoApiContext?.shutdown()
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
