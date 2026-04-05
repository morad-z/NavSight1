package com.example.navsight1

import android.graphics.Bitmap
import android.graphics.Canvas
import android.content.Context
import android.util.Log
import androidx.core.content.ContextCompat
import com.google.android.gms.maps.model.BitmapDescriptor
import com.google.android.gms.maps.model.BitmapDescriptorFactory
import com.google.android.gms.maps.model.LatLng
import kotlin.math.*

object NavSightUtils {
    fun resolveNavigationStart(snappedPosition: LatLng?, startLocation: LatLng?): LatLng? {
        return snappedPosition ?: startLocation
    }

    fun computeCalibrationStraightness(displacementMeters: Double, pathLengthMeters: Double): Double {
        if (pathLengthMeters <= 0.0) return 0.0
        return (displacementMeters / pathLengthMeters).coerceIn(0.0, 1.0)
    }

    fun computeUpdatedScaleCalibrationFactor(
        currentFactor: Double,
        knownDistanceMeters: Double,
        measuredDistanceMeters: Double
    ): Double? {
        if (currentFactor <= 0.0 || knownDistanceMeters <= 0.0 || measuredDistanceMeters <= 0.0) {
            return null
        }
        val ratio = knownDistanceMeters / measuredDistanceMeters
        return (currentFactor * ratio).coerceIn(0.25, 4.0)
    }

    fun metersToLatLng(start: LatLng, dx: Double, dz: Double): LatLng {
        // Use a more accurate geodesic calculation (Destination point given distance and bearing)
        // Earth's radius in meters
        val R = 6378137.0
        
        // Bearing in radians (atan2(dx, dz) where z is north, x is east)
        // In our coordinate system: z is north (forward), x is east (right)
        val bearing = atan2(dx, dz)
        val distance = sqrt(dx * dx + dz * dz)
        
        if (distance < 0.001) return start

        val lat1 = Math.toRadians(start.latitude)
        val lon1 = Math.toRadians(start.longitude)

        val lat2 = asin(sin(lat1) * cos(distance / R) +
                cos(lat1) * sin(distance / R) * cos(bearing))
        
        val lon2 = lon1 + atan2(sin(bearing) * sin(distance / R) * cos(lat1),
                cos(distance / R) - sin(lat1) * sin(lat2))

        return LatLng(Math.toDegrees(lat2), Math.toDegrees(lon2))
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

    fun vectorToBitmap(context: Context, drawableId: Int): BitmapDescriptor? {
        return try {
            val vectorDrawable = ContextCompat.getDrawable(context, drawableId) ?: return null
            vectorDrawable.setBounds(0, 0, vectorDrawable.intrinsicWidth, vectorDrawable.intrinsicHeight)
            val bitmap = Bitmap.createBitmap(
                vectorDrawable.intrinsicWidth,
                vectorDrawable.intrinsicHeight,
                Bitmap.Config.ARGB_8888
            )
            val canvas = Canvas(bitmap)
            vectorDrawable.draw(canvas)
            BitmapDescriptorFactory.fromBitmap(bitmap)
        } catch (e: Exception) {
            Log.e("NavSightUtils", "Failed to convert vector to bitmap", e)
            null
        }
    }
}
