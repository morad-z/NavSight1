package com.example.navsight1

import android.content.Context
import android.util.Log
import java.io.File

/**
 * Bounded LRU disk cache for fetched road-region Overpass JSON (RoadRegionManager).
 * Lives in cacheDir (OS-evictable under storage pressure — embedded Haifa is the
 * floor, so eviction is always safe) and is self-capped at [MAX_FILES] so it never
 * grows unbounded between OS sweeps. Atomic .tmp-rename writes survive a mid-write kill.
 */
internal class RegionDiskCache(context: Context) {
    private val dir = File(context.cacheDir, "osm_regions").apply { mkdirs() }

    fun read(key: String): String? = try {
        val f = File(dir, "$key.json")
        if (f.exists()) {
            f.setLastModified(System.currentTimeMillis()) // touch (LRU)
            f.readText()
        } else null
    } catch (e: Exception) {
        Log.w(TAG, "cache read $key: ${e.message}"); null
    }

    /** Age of the cache entry in ms, or Long.MAX_VALUE if absent. */
    fun ageMs(key: String): Long {
        val f = File(dir, "$key.json")
        return if (f.exists()) System.currentTimeMillis() - f.lastModified() else Long.MAX_VALUE
    }

    fun write(key: String, json: String) {
        try {
            val tmp = File(dir, "$key.json.tmp")
            tmp.writeText(json)
            if (!tmp.renameTo(File(dir, "$key.json"))) tmp.delete()
            evict()
        } catch (e: Exception) {
            Log.w(TAG, "cache write $key: ${e.message}")
        }
    }

    fun delete(key: String) {
        runCatching { File(dir, "$key.json").delete() }
    }

    /** Key of the most-recently-touched cached region, or null if the cache is empty.
     *  Used for the instant launch bootstrap (load where the user last was). */
    fun newestKey(): String? {
        val files = dir.listFiles { f -> f.name.endsWith(".json") } ?: return null
        return files.maxByOrNull { it.lastModified() }?.name?.removeSuffix(".json")
    }

    private fun evict() {
        val files = dir.listFiles { f -> f.name.endsWith(".json") } ?: return
        if (files.size <= MAX_FILES) return
        files.sortedBy { it.lastModified() }
            .take(files.size - MAX_FILES)
            .forEach { runCatching { it.delete() } }
    }

    companion object {
        private const val TAG = "RegionDiskCache"
        private const val MAX_FILES = 8 // ~8 × ~2 MB ≈ 16 MB ceiling
    }
}
