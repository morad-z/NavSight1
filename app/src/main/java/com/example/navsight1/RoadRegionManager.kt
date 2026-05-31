package com.example.navsight1

import android.content.Context
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.launch
import java.util.concurrent.atomic.AtomicBoolean

/**
 * RoadRegionManager — on-demand, location-based road loader so snapping + routing
 * work EVERYWHERE without embedding the whole country. Fetches an ~8.8 km Overpass
 * box around the user's current tile, builds SegmentStore + GraphStore off-thread,
 * and atomically publishes an immutable [DynamicRoadRegion]. Embedded Haifa stays
 * the instant offline bootstrap; fetched regions are disk-cached (offline on revisit).
 *
 * Trigger (kills boundary thrash): quantized tile + inner-margin hysteresis + a
 * 60 s min-fetch floor + single-fetch-in-flight flag. Reads are lock-free: a reader
 * captures the @Volatile region once and the stores are immutable, so a concurrent
 * swap is invisible mid-call. See the design doc for the full failure-mode analysis.
 */
internal object RoadRegionManager {
    private const val TAG = "RoadRegionManager"

    private const val TILE_DEG = 0.04            // ~4.4 km quantization grid
    private const val FETCH_HALF_DEG = 0.04      // ~8.8 km fetch box (≈2 tiles wide)
    private const val INNER_MARGIN_DEG = 0.012   // ~1.3 km hysteresis inside the box
    private const val CACHE_TTL_MS = 30L * 24 * 3600 * 1000  // 30 days
    private const val MIN_FETCH_INTERVAL_MS = 60_000L        // thrash floor
    private val TILE_KEY = Regex("""tile_(-?\d+)_(-?\d+)""")

    @Volatile
    var activeRegion: DynamicRoadRegion? = null
        private set

    private val fetchInFlight = AtomicBoolean(false)
    @Volatile private var lastFetchEndMs = 0L
    @Volatile private var loadedTileLat = Int.MIN_VALUE
    @Volatile private var loadedTileLng = Int.MIN_VALUE

    private var cache: RegionDiskCache? = null
    private var scope: CoroutineScope? = null
    @Volatile private var initialized = false

    // Falsifier counters (grep logcat "OSM_REGION").
    @Volatile var fetchOk = 0
    @Volatile var fetchFailed = 0
    @Volatile var swaps = 0
    @Volatile var cacheHits = 0
    @Volatile var emptyRegions = 0
    @Volatile var staleAccepted = 0

    fun init(context: Context) {
        if (initialized) return
        cache = RegionDiskCache(context.applicationContext)
        scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)
        initialized = true
        Log.i(TAG, "OSM_REGION init ok")
        scope?.launch { bootstrapFromCache() } // instant snap on reopen in a visited area
    }

    /** The road region to query for a point: the active region IF it covers the
     *  point, else null (so consumers fall back to the embedded OsmDataLayer). */
    fun regionFor(lat: Double, lng: Double): DynamicRoadRegion? =
        activeRegion?.takeIf { it.bbox.contains(lat, lng) }

    /** Non-blocking. Cheap when the current region already covers the point. */
    fun ensureRegion(lat: Double, lng: Double) {
        if (!initialized) return
        val r = activeRegion
        if (r != null && r.bbox.containsWithMargin(lat, lng, INNER_MARGIN_DEG)) return // already loaded

        val tLat = tileOf(lat); val tLng = tileOf(lng)
        // Hysteresis: still in the loaded tile + box → don't refetch at the edge.
        if (r != null && tLat == loadedTileLat && tLng == loadedTileLng && r.bbox.contains(lat, lng)) return

        if (System.currentTimeMillis() - lastFetchEndMs < MIN_FETCH_INTERVAL_MS) return
        if (!fetchInFlight.compareAndSet(false, true)) return // one fetch at a time
        scope?.launch { fetchAndSwap(tLat, tLng) }
    }

    private fun tileOf(v: Double) = Math.floor(v / TILE_DEG).toInt()

    /** Instant bootstrap: load the most-recently-cached region from disk on launch
     *  (before any GPS fix), so reopening the app in a previously-visited area snaps
     *  immediately. A later [ensureRegion] refreshes it if the user has moved. */
    private fun bootstrapFromCache() {
        val c = cache ?: return
        val key = c.newestKey() ?: return
        val m = TILE_KEY.matchEntire(key) ?: return
        val tLat = m.groupValues[1].toIntOrNull() ?: return
        val tLng = m.groupValues[2].toIntOrNull() ?: return
        val json = c.read(key) ?: return
        try {
            val arrays = RoadsJsonParser.parse(json)
            if (arrays.segCount == 0) return
            val cLat = (tLat + 0.5) * TILE_DEG; val cLng = (tLng + 0.5) * TILE_DEG
            val box = BBoxDeg.centred(cLat, cLng, FETCH_HALF_DEG)
            activeRegion = DynamicRoadRegion(
                SegmentRTreeReader.buildSegmentStore(arrays),
                RoutingGraphReader.buildGraphStore(arrays),
                box, tLat, tLng, System.currentTimeMillis(),
                RoundaboutIndexBuilder.build(arrays)
            )
            loadedTileLat = tLat; loadedTileLng = tLng
            swaps++
            Log.i(TAG, "OSM_REGION bootstrap-from-cache tile=${tLat}_${tLng} segs=${arrays.segCount}")
        } catch (e: Exception) {
            Log.w(TAG, "OSM_REGION bootstrap parse failed: ${e.message}")
        }
    }

    private fun fetchAndSwap(tLat: Int, tLng: Int) {
        try {
            // Box centred on the TILE CENTRE so every position in a tile maps to one
            // cache key (no duplicate fetches for the same tile).
            val cLat = (tLat + 0.5) * TILE_DEG
            val cLng = (tLng + 0.5) * TILE_DEG
            val box = BBoxDeg.centred(cLat, cLng, FETCH_HALF_DEG)
            val key = "tile_${tLat}_${tLng}"
            val c = cache ?: return

            var json: String? = null
            if (c.ageMs(key) < CACHE_TTL_MS) {
                json = c.read(key)
                if (json != null) cacheHits++
            }
            if (json == null) {
                val res = OverpassRoadsClient.fetchRoadsJson(box)
                if (res.isSuccess) {
                    json = res.getOrThrow(); c.write(key, json); fetchOk++
                } else {
                    val stale = c.read(key) // offline → accept stale (jam-safe)
                    if (stale != null) { json = stale; staleAccepted++ }
                    else {
                        fetchFailed++
                        Log.w(TAG, "OSM_REGION fetch_failed tile=${tLat}_${tLng} fetch_failed=$fetchFailed (region unchanged)")
                        return
                    }
                }
            }

            val arrays = try {
                RoadsJsonParser.parse(json!!)
            } catch (e: Exception) {
                c.delete(key); fetchFailed++
                Log.w(TAG, "OSM_REGION parse_failed tile=${tLat}_${tLng}: ${e.message}")
                return
            }
            if (arrays.segCount == 0) emptyRegions++

            val seg = SegmentRTreeReader.buildSegmentStore(arrays) // STR tree built off-thread
            val graph = RoutingGraphReader.buildGraphStore(arrays) // CSR built off-thread
            val roundabouts = RoundaboutIndexBuilder.build(arrays) // heading-aware matcher
            val region = DynamicRoadRegion(seg, graph, box, tLat, tLng, System.currentTimeMillis(), roundabouts)

            activeRegion = region // single volatile publish of a fully-built region
            loadedTileLat = tLat; loadedTileLng = tLng
            swaps++
            RoadSnapper.onRegionSwapped() // drop cross-region stale snap LRU
            Log.i(
                TAG,
                "OSM_REGION swap ok tile=${tLat}_${tLng} segs=${arrays.segCount} verts=${arrays.vertexCount} " +
                    "roundabouts=${roundabouts.size} swaps=$swaps cache_hits=$cacheHits stale_accepted=$staleAccepted empty=$emptyRegions"
            )
        } finally {
            lastFetchEndMs = System.currentTimeMillis()
            fetchInFlight.set(false)
        }
    }
}
