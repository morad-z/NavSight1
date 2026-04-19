package com.example.navsight1

import android.content.Context
import java.io.File

fun saveGpxFile(context: Context, points: List<Pair<Double, Double>>): String {
    if (points.size < 2) return "Not enough points for GPX"
    return try {
        val ts   = System.currentTimeMillis()
        val file = File(context.getExternalFilesDir(null), "navsight_$ts.gpx")
        file.writeText(buildString {
            appendLine("""<?xml version="1.0" encoding="UTF-8"?>""")
            appendLine("""<gpx version="1.1" creator="NavSight" xmlns="http://www.topografix.com/GPX/1/1">""")
            appendLine("""  <trk><name>NavSight Track $ts</name><trkseg>""")
            points.forEach { (lat, lon) -> appendLine("""    <trkpt lat="$lat" lon="$lon"/>""") }
            appendLine("""  </trkseg></trk></gpx>""")
        })
        "GPX saved ✓"
    } catch (e: Exception) { "GPX save failed: ${e.message}" }
}
