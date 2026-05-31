package com.example.navsight1

import org.json.JSONObject
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Parsed road arrays — column layout consumed by the [SegmentRTreeReader.buildSegmentStore]
 * and [RoutingGraphReader.buildGraphStore] runtime factories. Segment names are kept
 * inline (no string pool) since the dynamic path has a single consumer.
 */
internal class RoadArrays(
    val segCount: Int,
    val segmentId: LongArray, val wayId: LongArray, val classCode: IntArray,
    val aLat: DoubleArray, val aLng: DoubleArray, val bLat: DoubleArray, val bLng: DoubleArray,
    val segName: Array<String?>,
    val vertexCount: Int, val vLat: DoubleArray, val vLng: DoubleArray,
    val edgeCount: Int, val eFrom: IntArray, val eTo: IntArray,
    val eLen: DoubleArray, val eOneway: BooleanArray,
    // per-segment junction tag: "roundabout" / "mini_roundabout" / null (heading-aware matcher)
    val junction: Array<String?>
)

/**
 * Runtime Kotlin port of scripts/build_haifa_assets.py `build_roads()`: turns an
 * Overpass `way["highway"](bbox); out geom;` JSON response into [RoadArrays]
 * (segments + dedup'd routing graph). Replicates the same highway-class filter,
 * access skip, oneway set, vertex quantization, and haversine, so a dynamically
 * fetched region snaps/routes identically to the embedded blobs.
 */
internal object RoadsJsonParser {

    private fun haversineM(lat1: Double, lng1: Double, lat2: Double, lng2: Double): Double {
        val r = 6371000.0
        val dLat = Math.toRadians(lat2 - lat1); val dLng = Math.toRadians(lng2 - lng1)
        val h = sin(dLat / 2) * sin(dLat / 2) +
            cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLng / 2) * sin(dLng / 2)
        return r * 2 * atan2(sqrt(h), sqrt(1 - h))
    }

    /** Throws JSONException on malformed input — the caller (RoadRegionManager) catches. */
    fun parse(json: String): RoadArrays {
        val root = JSONObject(json)
        val elements = root.optJSONArray("elements") ?: return empty()

        val segId = ArrayList<Long>(); val wId = ArrayList<Long>(); val cc = ArrayList<Int>()
        val aLa = ArrayList<Double>(); val aLn = ArrayList<Double>()
        val bLa = ArrayList<Double>(); val bLn = ArrayList<Double>()
        val sName = ArrayList<String?>(); val sJn = ArrayList<String?>()
        val vLa = ArrayList<Double>(); val vLn = ArrayList<Double>()
        val vIndex = HashMap<Long, Int>()
        val eF = ArrayList<Int>(); val eT = ArrayList<Int>()
        val eL = ArrayList<Double>(); val eO = ArrayList<Boolean>()
        var nextSeg = 0L

        fun vertexOf(lat: Double, lng: Double): Int {
            val latQ = Math.round(lat * 1e7); val lngQ = Math.round(lng * 1e7)
            val key = (latQ shl 32) xor (lngQ and 0xffffffffL)
            vIndex[key]?.let { return it }
            val v = vLa.size; vLa.add(lat); vLn.add(lng); vIndex[key] = v; return v
        }

        for (i in 0 until elements.length()) {
            val el = elements.optJSONObject(i) ?: continue
            if (el.optString("type") != "way") continue
            val tags = el.optJSONObject("tags") ?: JSONObject()
            val hw = tags.optString("highway", "")
            val geom = el.optJSONArray("geometry") ?: continue
            if (hw.isEmpty() || OsmAssetFormat.highwayClassCode(hw) == 0) continue
            val access = tags.optString("access", "")
            if (access == "private" || access == "no") continue
            val code = OsmAssetFormat.highwayClassCode(hw)
            val ow = tags.optString("oneway", "").let { it == "yes" || it == "true" || it == "1" }
            val name = if (tags.has("name")) tags.optString("name") else null
            val jn = tags.optString("junction", "").let {
                if (it == "roundabout" || it == "mini_roundabout") it else null
            }
            var prevLat = Double.NaN; var prevLng = Double.NaN; var prevV = -1
            for (g in 0 until geom.length()) {
                val p = geom.optJSONObject(g) ?: continue
                if (!p.has("lat") || !p.has("lon")) continue
                val lat = p.getDouble("lat"); val lng = p.getDouble("lon")
                val v = vertexOf(lat, lng)
                if (prevV >= 0) {
                    segId.add(nextSeg); wId.add(el.optLong("id", 0)); cc.add(code)
                    aLa.add(prevLat); aLn.add(prevLng); bLa.add(lat); bLn.add(lng)
                    sName.add(name); sJn.add(jn)
                    eF.add(prevV); eT.add(v); eL.add(haversineM(prevLat, prevLng, lat, lng)); eO.add(ow)
                    nextSeg++
                }
                prevLat = lat; prevLng = lng; prevV = v
            }
        }
        return RoadArrays(
            segId.size, segId.toLongArray(), wId.toLongArray(), cc.toIntArray(),
            aLa.toDoubleArray(), aLn.toDoubleArray(), bLa.toDoubleArray(), bLn.toDoubleArray(),
            sName.toTypedArray(),
            vLa.size, vLa.toDoubleArray(), vLn.toDoubleArray(),
            eF.size, eF.toIntArray(), eT.toIntArray(), eL.toDoubleArray(), eO.toBooleanArray(),
            sJn.toTypedArray()
        )
    }

    private fun empty() = RoadArrays(
        0, LongArray(0), LongArray(0), IntArray(0),
        DoubleArray(0), DoubleArray(0), DoubleArray(0), DoubleArray(0), arrayOfNulls(0),
        0, DoubleArray(0), DoubleArray(0), 0, IntArray(0), IntArray(0), DoubleArray(0), BooleanArray(0),
        arrayOfNulls(0)
    )
}
