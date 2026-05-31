package com.example.navsight1

import java.text.Normalizer

/**
 * GeocodeIndexReader — parses `haifa_geocode.bin` into a [GeocodeStore] and runs
 * the offline destination search (MAP_MATCHING_PLAN.md §8M Step A* / K-search*).
 * Hand-rolled inverted index (token → place postings) over OSM name/addr tags —
 * the only option satisfying offline + no-key + jam-proof.
 *
 * Blob layout (little-endian):
 *   [magic:int=NSGC][version:int][placeCount:int]
 *   placeCount × { placeId:long, lat:double, lng:double, displayNameIdx:int }
 *   <string pool>  (display names)
 *   [tokenCount:int]  ×{ [tokenLen:int][utf8 token][postingCount:int][postings:int[]] }
 */
internal object GeocodeIndexReader {

    fun parse(bytes: ByteArray): GeocodeStore {
        val buf = OsmAssetFormat.reader(bytes)
        val magic = buf.int
        require(magic == OsmAssetFormat.MAGIC_GEOCODE) {
            "geocode blob: bad magic $magic (expected ${OsmAssetFormat.MAGIC_GEOCODE})"
        }
        val version = buf.int
        require(version == OsmAssetFormat.SCHEMA_VERSION) {
            "geocode blob: schema version $version != ${OsmAssetFormat.SCHEMA_VERSION}"
        }
        val placeCount = buf.int
        require(placeCount in 0..MAX_PLACES) { "geocode blob: placeCount out of range $placeCount" }

        val placeId = LongArray(placeCount)
        val lat = DoubleArray(placeCount)
        val lng = DoubleArray(placeCount)
        val nameIdx = IntArray(placeCount)
        for (i in 0 until placeCount) {
            placeId[i] = buf.long
            lat[i] = buf.double
            lng[i] = buf.double
            nameIdx[i] = buf.int
        }
        val pool = StringPool.read(buf)

        val tokenCount = buf.int
        require(tokenCount in 0..MAX_TOKENS) { "geocode blob: tokenCount out of range $tokenCount" }
        val postings = HashMap<String, IntArray>(tokenCount * 2)
        for (t in 0 until tokenCount) {
            val len = buf.int
            require(len in 0..4096) { "geocode blob: token length out of range $len" }
            val tb = ByteArray(len); buf.get(tb)
            val token = String(tb, Charsets.UTF_8)
            val pc = buf.int
            require(pc in 0..placeCount) { "geocode blob: posting count out of range $pc" }
            val arr = IntArray(pc) { buf.int }
            postings[token] = arr
        }

        return GeocodeStore(placeCount, placeId, lat, lng, nameIdx, pool, postings)
    }

    /**
     * Normalize a string to lowercase, diacritic-stripped tokens. Keeps Unicode
     * letters (Hebrew + Latin both survive) and digits; splits on everything else.
     * Used at BOTH preprocess (to build postings) and query time so they agree.
     */
    fun tokenize(s: String): List<String> {
        val nfd = Normalizer.normalize(s.lowercase(), Normalizer.Form.NFD)
        val stripped = nfd.replace(COMBINING_MARKS, "")
        return stripped.split(NON_TOKEN).filter { it.isNotBlank() }
    }

    private val COMBINING_MARKS = Regex("\\p{Mn}+")
    private val NON_TOKEN = Regex("[^\\p{L}\\p{Nd}]+")

    private const val MAX_PLACES = 50_000_000
    private const val MAX_TOKENS = 50_000_000
}

/**
 * Immutable geocode index with a ranked [geocode] query. Ranking, per §8M Step
 * A*: exact-token > prefix-token, more query tokens matched is better, ties broken
 * by proximity to the search anchor.
 */
internal class GeocodeStore(
    val placeCount: Int,
    private val placeId: LongArray,
    private val lat: DoubleArray,
    private val lng: DoubleArray,
    private val nameIdx: IntArray,
    private val namePool: Array<String>,
    private val postings: Map<String, IntArray>
) {
    fun geocode(query: String, anchorLat: Double, anchorLng: Double, limit: Int): List<GeoHit> {
        if (limit <= 0) return emptyList()
        val qTokens = GeocodeIndexReader.tokenize(query)
        if (qTokens.isEmpty()) return emptyList()

        // candidate place index -> (matchedTokens, bestQuality)
        val matched = HashMap<Int, IntArray>()  // [matchedCount, bestQuality]
        for (qt in qTokens) {
            // exact token postings
            postings[qt]?.let { arr ->
                for (p in arr) bump(matched, p, quality = 2)
            }
            // prefix token postings (token keys that start with qt). Linear scan
            // over the key set — fine for one city; avoids a trie in v1.
            if (qt.length >= 2) {
                for ((key, arr) in postings) {
                    if (key.length > qt.length && key.startsWith(qt)) {
                        for (p in arr) bump(matched, p, quality = 1)
                    }
                }
            }
        }
        if (matched.isEmpty()) return emptyList()

        // Rank: more matched tokens, then higher quality, then closer to anchor.
        val ranked = matched.entries.map { (p, agg) ->
            val dist = RoadSnapMath.haversineM(anchorLat, anchorLng, lat[p], lng[p])
            ScoredPlace(p, agg[0], agg[1], dist)
        }.sortedWith(
            compareByDescending<ScoredPlace> { it.matchedTokens }
                .thenByDescending { it.bestQuality }
                .thenBy { it.distanceM }
        )

        val out = ArrayList<GeoHit>(minOf(limit, ranked.size))
        for (k in 0 until minOf(limit, ranked.size)) {
            val p = ranked[k].place
            out.add(
                GeoHit(
                    placeId = placeId[p],
                    lat = lat[p],
                    lng = lng[p],
                    displayName = StringPool.resolve(namePool, nameIdx[p]) ?: ""
                )
            )
        }
        return out
    }

    private fun bump(map: HashMap<Int, IntArray>, place: Int, quality: Int) {
        val cur = map[place]
        if (cur == null) {
            map[place] = intArrayOf(1, quality)
        } else {
            cur[0] = cur[0] + 1
            if (quality > cur[1]) cur[1] = quality
        }
    }

    private data class ScoredPlace(
        val place: Int,
        val matchedTokens: Int,
        val bestQuality: Int,
        val distanceM: Double
    )
}
