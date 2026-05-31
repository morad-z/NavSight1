package com.example.navsight1

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * OsmAssetFormat — authoritative little-endian binary format for the Haifa OSM
 * assets consumed by the offline map-matching / routing / geocoding stack
 * (MAP_MATCHING_PLAN.md §8M Step A*).
 *
 * Three blobs are produced by the dev-machine preprocessor (`tools/osm-preprocess`)
 * from a Haifa-clipped OSM extract and shipped as APK assets under
 * `assets/osm/haifa/`:
 *   - haifa_segments.bin  — flat road-segment records (snapping, Step C*)
 *   - haifa_graph.bin     — routing graph: vertices + edges (routing, Step K-routing*)
 *   - haifa_geocode.bin   — place table + token postings (search, Step K-search*)
 *
 * DESIGN DECISIONS (deliberate, documented — "make no mistakes"):
 *   1. Blobs are flat record arrays, NOT serialized tree/index structures. The
 *      STR-packed R-tree (segments) and the inverted-index lookup (geocode) are
 *      built IN MEMORY at load from the flat records. The plan named a serialized
 *      packed R-tree; building it in-RAM from a flat blob is far less bug-prone
 *      (no packed-node pointer arithmetic to get wrong) and still meets the
 *      ≤300 ms load / ≤1 ms query budgets for one city (Haifa ≈ tens of k segments).
 *   2. Everything is little-endian, length-prefixed, with a 4-byte magic + version
 *      header so a schema mismatch fails fast (Result.failure) instead of mis-reading.
 *   3. Determinism: the preprocessor emits records in a stable sort order, so the
 *      same PBF date + bbox produce byte-identical blobs (A* acceptance).
 *
 * Versioning: bump [SCHEMA_VERSION] on any layout change; the readers reject a
 * blob whose version differs.
 */
internal object OsmAssetFormat {

    const val SCHEMA_VERSION: Int = 1

    // 4-byte ASCII magics (read/written as little-endian Int for a single compare).
    // "NSSG" segments, "NSGR" graph, "NSGC" geocode.
    val MAGIC_SEGMENTS: Int = magic("NSSG")
    val MAGIC_GRAPH: Int = magic("NSGR")
    val MAGIC_GEOCODE: Int = magic("NSGC")

    // Asset directory + filenames (relative to AssetManager root).
    const val ASSET_DIR = "osm/haifa"
    const val FILE_SEGMENTS = "haifa_segments.bin"
    const val FILE_GRAPH = "haifa_graph.bin"
    const val FILE_GEOCODE = "haifa_geocode.bin"
    const val FILE_MANIFEST = "haifa_assets.json"

    /**
     * Single locomotion-agnostic highway-class table (M10: no mount-mode split).
     * The preprocessor only emits ways whose `highway` tag is in this set; the
     * class code stored in each segment/edge is the index here. Index 0 is a
     * defensive "unknown" so an out-of-range code never crashes a reader.
     */
    val HIGHWAY_CLASSES: List<String> = listOf(
        "unknown",        // 0 — defensive default
        "footway",        // 1
        "path",           // 2
        "pedestrian",     // 3
        "steps",          // 4
        "living_street",  // 5
        "residential",    // 6
        "unclassified",   // 7
        "tertiary",       // 8
        "secondary",      // 9
        "primary",        // 10
        "service",        // 11
        "cycleway",       // 12
        "track"           // 13
    )

    fun highwayClassName(code: Int): String =
        HIGHWAY_CLASSES.getOrElse(code) { HIGHWAY_CLASSES[0] }

    fun highwayClassCode(name: String?): Int {
        if (name == null) return 0
        val idx = HIGHWAY_CLASSES.indexOf(name)
        return if (idx >= 0) idx else 0
    }

    private fun magic(s: String): Int {
        require(s.length == 4) { "magic must be 4 chars" }
        // little-endian: first char is least-significant byte
        return (s[0].code and 0xFF) or
            ((s[1].code and 0xFF) shl 8) or
            ((s[2].code and 0xFF) shl 16) or
            ((s[3].code and 0xFF) shl 24)
    }

    /** Wrap a blob's bytes as a little-endian read cursor. */
    fun reader(bytes: ByteArray): ByteBuffer =
        ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
}

/**
 * Length-prefixed UTF-8 string pool reader/format helper shared by the segment
 * and geocode blobs. Pool layout: [count:int]{ [len:int][utf8 bytes] } repeated.
 * Strings are referenced by their index into the pool; index -1 means "no string".
 */
internal object StringPool {
    const val NO_STRING_INDEX = -1

    fun read(buf: ByteBuffer): Array<String> {
        val count = buf.int
        require(count >= 0 && count <= MAX_POOL) { "string pool count out of range: $count" }
        return Array(count) {
            val len = buf.int
            require(len in 0..MAX_STRING_BYTES) { "string length out of range: $len" }
            val b = ByteArray(len)
            buf.get(b)
            String(b, Charsets.UTF_8)
        }
    }

    fun resolve(pool: Array<String>, idx: Int): String? =
        if (idx == NO_STRING_INDEX || idx < 0 || idx >= pool.size) null else pool[idx]

    private const val MAX_POOL = 5_000_000
    private const val MAX_STRING_BYTES = 1 shl 20  // 1 MB sanity bound per string
}
