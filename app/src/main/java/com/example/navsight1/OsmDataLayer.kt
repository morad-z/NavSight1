package com.example.navsight1

import android.content.Context
import android.util.Log
import org.json.JSONObject

/** A single OSM road segment (a→b) returned by the snap query (§8M Step A* / C*). */
data class RoadSegment(
    val segmentId: Long,
    val wayId: Long,
    val highwayClass: String,
    val aLat: Double, val aLng: Double,
    val bLat: Double, val bLng: Double,
    val name: String?
)

/** A single geocode hit returned by the offline search (§8M Step A* / K-search*). */
data class GeoHit(
    val placeId: Long,
    val lat: Double,
    val lng: Double,
    val displayName: String
)

/** Asset manifest (bbox + provenance). Best-effort; null when absent/malformed. */
data class OsmAssetManifest(
    val schemaVersion: Int,
    val bbox: DoubleArray?,          // [minLon, minLat, maxLon, maxLat]
    val upstreamPbfDate: String?,
    val builtAt: String?,
    val segmentCount: Int,
    val vertexCount: Int,
    val edgeCount: Int,
    val placeCount: Int
)

/**
 * OsmDataLayer — the single runtime surface Steps C* / K-routing* / K-search* talk
 * to (MAP_MATCHING_PLAN.md §8M Step A*.2). It loads the three Haifa blobs from APK
 * assets, builds the in-memory structures (STR R-tree, routing graph, geocode
 * index), and exposes pure, allocation-light query methods. It NEVER touches
 * Tracker/EKFState — read-only/additive.
 *
 * GRACEFUL DEGRADATION: until the assets exist (the dev-machine preprocessor is a
 * separate step the user runs — see tools/osm-preprocess + scripts/build_haifa_assets.sh),
 * [load] returns Result.failure and the app keeps working with NO snap / NO offline
 * route / NO offline search (the consumers all fall back to raw VIO / straight line /
 * empty predictions). This is the same degradation as the old "API key blank" path.
 */
class OsmDataLayer private constructor(
    private val geocodeIndex: GeocodeStore,
    private val manifest: OsmAssetManifest?
) {

    /** Offline geocode: ranked place hits for [query] near the anchor (Israel-wide). */
    fun geocode(query: String, anchorLat: Double, anchorLng: Double, limit: Int): List<GeoHit> =
        geocodeIndex.geocode(query, anchorLat, anchorLng, limit)

    fun manifest(): OsmAssetManifest? = manifest

    val placeCount: Int get() = geocodeIndex.placeCount

    companion object {
        private const val TAG = "OsmDataLayer"

        /**
         * Process-wide holder. Set on the first successful [load]; consumers that
         * cannot take a constructor argument (RoadSnapper at NavSightViewModel:63,
         * which must stay byte-identical) read it through here. `null` => not loaded
         * yet => degrade gracefully.
         */
        @Volatile
        var instance: OsmDataLayer? = null
            private set

        /**
         * Load the Haifa assets from APK assets, build the structures, and publish
         * [instance]. Idempotent (a second successful call replaces the instance).
         * Returns Result.failure — never throws — on a missing/corrupt blob so the
         * app degrades instead of crashing. Run OFF the UI thread (Dispatchers.Default).
         */
        fun load(context: Context): Result<Unit> {
            val t0 = System.nanoTime()
            return try {
                val am = context.assets
                // Only the SEARCH (geocode) index is embedded (Israel-wide). Roads are
                // loaded on demand around the user's location by RoadRegionManager.
                val geoBytes = am.open("${OsmAssetFormat.ASSET_DIR}/${OsmAssetFormat.FILE_GEOCODE}").use { it.readBytes() }

                val geoStore = GeocodeIndexReader.parse(geoBytes)
                val manifest = loadManifest(context)

                val layer = OsmDataLayer(geoStore, manifest)
                instance = layer

                val loadMs = (System.nanoTime() - t0) / 1_000_000
                Log.i(
                    TAG,
                    "OSM_LOAD ok osm_load_ms=$loadMs osm_place_count=${geoStore.placeCount} " +
                        "osm_geocode_bytes=${geoBytes.size} osm_load_failed=0 (search only; roads dynamic)"
                )
                Result.success(Unit)
            } catch (e: java.io.FileNotFoundException) {
                // Assets not built yet — expected until the dev pipeline runs. Not an error.
                Log.i(TAG, "OSM_LOAD osm_load_failed=1 reason=assets_absent (${e.message}); map-match disabled, raw-VIO only")
                Result.failure(e)
            } catch (e: Exception) {
                Log.e(TAG, "OSM_LOAD osm_load_failed=1 reason=corrupt_or_parse_error", e)
                Result.failure(e)
            }
        }

        private fun loadManifest(context: Context): OsmAssetManifest? = try {
            val text = context.assets
                .open("${OsmAssetFormat.ASSET_DIR}/${OsmAssetFormat.FILE_MANIFEST}")
                .use { it.readBytes() }
                .toString(Charsets.UTF_8)
            val o = JSONObject(text)
            val bboxArr = o.optJSONArray("bbox")
            val bbox = if (bboxArr != null && bboxArr.length() == 4) {
                DoubleArray(4) { bboxArr.getDouble(it) }
            } else null
            OsmAssetManifest(
                schemaVersion = o.optInt("schema_version", OsmAssetFormat.SCHEMA_VERSION),
                bbox = bbox,
                upstreamPbfDate = o.optString("upstream_pbf_date", null),
                builtAt = o.optString("built_at", null),
                segmentCount = o.optInt("segment_count", 0),
                vertexCount = o.optInt("vertex_count", 0),
                edgeCount = o.optInt("edge_count", 0),
                placeCount = o.optInt("place_count", 0)
            )
        } catch (e: Exception) {
            // Manifest is non-critical (only the deferred region manager / Step E read it).
            Log.i(TAG, "OSM_LOAD manifest absent/unreadable (non-fatal): ${e.message}")
            null
        }
    }
}
