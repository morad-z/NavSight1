package com.example.navsight1

import android.graphics.Bitmap
import android.graphics.Canvas
import android.content.Context
import androidx.core.content.ContextCompat
import com.google.android.gms.maps.model.BitmapDescriptor
import com.google.android.gms.maps.model.BitmapDescriptorFactory
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

object NavSightUtils {
    fun metersToLatLng(start: LatLng, dx: Double, dz: Double): LatLng {
        val metersPerDegree = 111111.0
        val lat = start.latitude + (dz / metersPerDegree)
        val cosLat = cos(Math.toRadians(start.latitude))
        val lngOffset = if (cosLat > 1e-10) dx / (metersPerDegree * cosLat) else 0.0
        return LatLng(lat, start.longitude + lngOffset)
    }

    fun formatDistance(meters: Int): String {
        return when {
            meters < 1000 -> "${meters}m"
            else -> "${"%.1f".format(meters / 1000.0)}km"
        }
    }

    fun formatTime(seconds: Int): String {
        val minutes = seconds / 60
        return when {
            minutes < 60 -> "${minutes}min"
            else -> "${minutes / 60}h ${minutes % 60}min"
        }
    }

    fun vectorToBitmap(context: Context, drawableId: Int): BitmapDescriptor {
        val vectorDrawable = ContextCompat.getDrawable(context, drawableId)
        vectorDrawable!!.setBounds(0, 0, vectorDrawable.intrinsicWidth, vectorDrawable.intrinsicHeight)
        val bitmap = Bitmap.createBitmap(
            vectorDrawable.intrinsicWidth,
            vectorDrawable.intrinsicHeight,
            Bitmap.Config.ARGB_8888
        )
        val canvas = Canvas(bitmap)
        vectorDrawable.draw(canvas)
        return BitmapDescriptorFactory.fromBitmap(bitmap)
    }
}
