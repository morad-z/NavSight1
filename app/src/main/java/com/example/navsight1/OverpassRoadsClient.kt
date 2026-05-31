package com.example.navsight1

import android.util.Log
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder

/**
 * Fetches the highway ways for a bbox from the Overpass API. HTTP only — never
 * throws (returns Result). Rotates mirrors. Used by RoadRegionManager to load a
 * road region around the user's current location on demand.
 */
internal object OverpassRoadsClient {
    private const val TAG = "OverpassRoadsClient"
    private val MIRRORS = listOf(
        "https://overpass-api.de/api/interpreter",
        "https://overpass.kumi.systems/api/interpreter",
        "https://maps.mail.ru/osm/tools/overpass/api/interpreter"
    )
    private const val TIMEOUT_MS = 45_000

    /** Returns Result.success(json) or Result.failure — never throws. Overpass bbox
     *  order is (S,W,N,E). */
    fun fetchRoadsJson(box: BBoxDeg): Result<String> {
        val bbox = "${box.minLat},${box.minLon},${box.maxLat},${box.maxLon}"
        val query = "[out:json][timeout:40];(way[\"highway\"]($bbox););out geom;"
        val body = "data=" + URLEncoder.encode(query, "UTF-8")
        var last: Exception? = null
        for (url in MIRRORS) {
            var conn: HttpURLConnection? = null
            try {
                conn = (URL(url).openConnection() as HttpURLConnection).apply {
                    requestMethod = "POST"; doOutput = true
                    connectTimeout = TIMEOUT_MS; readTimeout = TIMEOUT_MS
                    setRequestProperty("User-Agent", "navsight-android/1.0")
                    setRequestProperty("Content-Type", "application/x-www-form-urlencoded")
                }
                conn.outputStream.use { it.write(body.toByteArray()) }
                val code = conn.responseCode
                if (code != 200) {
                    Log.w(TAG, "OSM_REGION mirror=$url http=$code")
                    last = Exception("http $code"); continue
                }
                val text = conn.inputStream.bufferedReader().use { it.readText() }
                return Result.success(text)
            } catch (e: Exception) {
                Log.w(TAG, "OSM_REGION mirror=$url failed: ${e.message}"); last = e
            } finally {
                conn?.disconnect()
            }
        }
        return Result.failure(last ?: Exception("all mirrors failed"))
    }
}
