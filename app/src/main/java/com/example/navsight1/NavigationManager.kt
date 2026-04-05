package com.example.navsight1

import android.content.Context
import android.util.Log
import androidx.annotation.Nullable
import androidx.annotation.NonNull
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.ArrowForward
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.ui.graphics.vector.ImageVector
import com.google.android.gms.maps.model.LatLng
import com.google.maps.DirectionsApi
import com.google.maps.GeoApiContext
import com.google.maps.internal.PolylineEncoding
import com.google.maps.model.TravelMode
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withContext
import kotlin.math.*

/**
 * NavigationManager
 *
 * Manages Navigation SDK routing and turn-by-turn instructions.
 *
 * Architecture:
 * - Uses Navigation SDK for route calculation
 * - Tracks progress using VIO snapped positions (not GPS)
 * - Emits turn-by-turn instructions based on distance to maneuver
 * - Computes ETA and remaining distance
 *
 * State Flow:
 * Idle → Routing → Active → Arrived
 *
 * Usage:
 *   val manager = NavigationManager(context)
 *   manager.startNavigation(destination)
 *   manager.updateVioPosition(snappedLatLng)
 *   manager.navigationState.collect { state -> ... }
 */
class NavigationManager(private val context: Context, private val apiKey: String) {

    companion object {
        private const val TAG = "NavigationManager"
        private const val INSTRUCTION_TRIGGER_DISTANCE_METERS = 100.0
        private const val ARRIVAL_THRESHOLD_METERS = 20.0
        private const val ROUTE_RECALC_DISTANCE_METERS = 50.0
    }

    // Google Directions API context — lazily initialized (may be null if API key missing)
    // Note: @Nullable cannot be applied to delegated properties, type system handles nullability
    private val geoApiContext: GeoApiContext? by lazy {
        if (apiKey.isBlank()) {
            Log.e(TAG, "API key is blank — Directions API routing will be disabled")
            null
        } else {
            GeoApiContext.Builder()
                .apiKey(apiKey)
                .build()
        }
    }

    // Observable navigation state
    private val _navigationState = MutableStateFlow<NavigationState>(NavigationState.Idle)
    val navigationState: StateFlow<NavigationState> = _navigationState.asStateFlow()

    // Observable current instruction
    private val _currentInstruction = MutableStateFlow<NavigationInstruction?>(null)
    val currentInstruction: StateFlow<NavigationInstruction?> = _currentInstruction.asStateFlow()

    // Internal state
    @Nullable
    private var currentRoute: NavigationRoute? = null
    private var currentStepIndex = 0
    @Nullable
    private var lastPosition: LatLng? = null

    /**
     * Starts navigation to a destination.
     *
     * @param currentPosition Current user position (GPS or VIO-derived)
     * @param destination Target location (LatLng)
     */
    suspend fun startNavigation(currentPosition: LatLng, destination: LatLng) {
        Log.i(TAG, "Starting navigation from: $currentPosition to: $destination")

        try {
            _navigationState.value = NavigationState.Routing(destination)

            lastPosition = currentPosition
            val route = calculateRoute(currentPosition, destination)

            currentRoute = route
            currentStepIndex = 0

            _navigationState.value = NavigationState.Active(
                route = route,
                remainingDistanceMeters = route.totalDistanceMeters,
                remainingTimeSeconds = route.estimatedTimeSeconds,
                currentStepIndex = 0
            )

            // Emit first instruction
            updateInstruction()

            Log.i(TAG, "Navigation started. Route distance: ${route.totalDistanceMeters}m")

        } catch (e: Exception) {
            Log.e(TAG, "Failed to start navigation: ${e.message}", e)
            _navigationState.value = NavigationState.Idle
        }
    }

    /**
     * Updates current VIO snapped position and recalculates navigation state.
     *
     * @param snappedPosition Current snapped position from RoadSnapper
     */
    fun updateVioPosition(snappedPosition: LatLng) {
        lastPosition = snappedPosition

        val state = _navigationState.value
        if (state !is NavigationState.Active) return

        val route = state.route

        // Calculate distance to destination
        val distanceToDestination = calculateDistance(snappedPosition, route.destination)

        // Check if arrived
        if (distanceToDestination < ARRIVAL_THRESHOLD_METERS) {
            Log.i(TAG, "Arrived at destination!")
            _navigationState.value = NavigationState.Arrived
            _currentInstruction.value = null
            return
        }

        // Find current step based on position
        val newStepIndex = findCurrentStep(snappedPosition, route)
        if (newStepIndex != currentStepIndex) {
            currentStepIndex = newStepIndex
            updateInstruction()
        }

        // Calculate remaining distance and time
        val remainingDistance = calculateRemainingDistance(snappedPosition, route, currentStepIndex)
        val remainingTime = estimateRemainingTime(remainingDistance, route)

        // Update navigation state
        _navigationState.value = NavigationState.Active(
            route = route,
            remainingDistanceMeters = remainingDistance,
            remainingTimeSeconds = remainingTime,
            currentStepIndex = currentStepIndex
        )

        // Update instruction if close to maneuver
        updateInstruction(snappedPosition)
    }

    /**
     * Cancels active navigation.
     */
    fun cancelNavigation() {
        Log.i(TAG, "Navigation cancelled")
        _navigationState.value = NavigationState.Idle
        _currentInstruction.value = null
        currentRoute = null
        currentStepIndex = 0
    }

    /**
     * Updates the current instruction based on distance to next maneuver.
     */
    private fun updateInstruction(currentPosition: LatLng? = lastPosition) {
        val state = _navigationState.value
        if (state !is NavigationState.Active) return

        val route = state.route
        if (currentStepIndex >= route.steps.size) return

        val currentStep = route.steps[currentStepIndex]
        val distanceToManeuver = currentPosition?.let {
            calculateDistance(it, currentStep.startLocation)
        } ?: Double.MAX_VALUE

        // Only show instruction if within trigger distance
        if (distanceToManeuver < INSTRUCTION_TRIGGER_DISTANCE_METERS) {
            _currentInstruction.value = NavigationInstruction(
                type = currentStep.maneuver,
                streetName = currentStep.streetName,
                distanceMeters = distanceToManeuver.toInt(),
                instruction = currentStep.instruction
            )
        } else {
            // Far from maneuver - show general "continue" instruction
            _currentInstruction.value = NavigationInstruction(
                type = ManeuverType.STRAIGHT,
                streetName = currentStep.streetName,
                distanceMeters = distanceToManeuver.toInt(),
                instruction = "Continue on ${currentStep.streetName}"
            )
        }
    }

    /**
     * Calculates a route using the Google Directions API.
     * Falls back to a straight-line route if the API call fails.
     */
    private suspend fun calculateRoute(start: LatLng, destination: LatLng): NavigationRoute {
        val ctx = geoApiContext
        if (ctx == null) {
            Log.w(TAG, "GeoApiContext unavailable, falling back to straight-line route")
            return calculateFallbackRoute(start, destination)
        }

        return try {
            withContext(Dispatchers.IO) {
                val result = DirectionsApi.newRequest(ctx)
                    .origin(com.google.maps.model.LatLng(start.latitude, start.longitude))
                    .destination(com.google.maps.model.LatLng(destination.latitude, destination.longitude))
                    .mode(TravelMode.DRIVING)
                    .await()

                if (result.routes.isNullOrEmpty()) {
                    Log.w(TAG, "Directions API returned no routes, falling back")
                    return@withContext calculateFallbackRoute(start, destination)
                }

                val route = result.routes[0]
                val leg = route.legs[0]

                // Decode the overview polyline into LatLng points
                val decodedPath = PolylineEncoding.decode(route.overviewPolyline.encodedPath)
                val polylinePoints = decodedPath.map { LatLng(it.lat, it.lng) }

                // Parse steps from the response
                val steps = leg.steps.map { step ->
                    val maneuver = parseManeuver(step.htmlInstructions)
                    val streetName = extractStreetName(step.htmlInstructions)
                    val instruction = step.htmlInstructions
                        .replace(Regex("<[^>]*>"), "") // Strip HTML tags

                    RouteStep(
                        startLocation = LatLng(step.startLocation.lat, step.startLocation.lng),
                        endLocation = LatLng(step.endLocation.lat, step.endLocation.lng),
                        distance = step.distance.inMeters.toDouble(),
                        maneuver = maneuver,
                        streetName = streetName,
                        instruction = instruction
                    )
                }

                val totalDistanceMeters = leg.distance.inMeters.toDouble()
                val totalTimeSeconds = leg.duration.inSeconds.toInt()

                Log.i(TAG, "Directions API route: ${totalDistanceMeters}m, ${totalTimeSeconds}s, ${steps.size} steps, ${polylinePoints.size} polyline points")

                NavigationRoute(
                    start = start,
                    destination = destination,
                    steps = steps,
                    totalDistanceMeters = totalDistanceMeters,
                    estimatedTimeSeconds = totalTimeSeconds,
                    polyline = polylinePoints
                )
            }
        } catch (e: Exception) {
            Log.e(TAG, "Directions API error: ${e.message}", e)
            calculateFallbackRoute(start, destination)
        }
    }

    /**
     * Creates a straight-line fallback route when the Directions API is unavailable.
     */
    private fun calculateFallbackRoute(start: LatLng, destination: LatLng): NavigationRoute {
        val distance = calculateDistance(start, destination)
        val estimatedTime = (distance / 10.0).toInt() // Assume 10 m/s average speed

        val steps = listOf(
            RouteStep(
                startLocation = start,
                endLocation = destination,
                distance = distance,
                maneuver = ManeuverType.STRAIGHT,
                streetName = "Route",
                instruction = "Head to destination"
            )
        )

        return NavigationRoute(
            start = start,
            destination = destination,
            steps = steps,
            totalDistanceMeters = distance,
            estimatedTimeSeconds = estimatedTime,
            polyline = listOf(start, destination)
        )
    }

    /**
     * Parses a ManeuverType from Directions API HTML instructions.
     */
    private fun parseManeuver(htmlInstructions: String): ManeuverType {
        val lower = htmlInstructions.lowercase()
        return when {
            "uturn" in lower || "u-turn" in lower -> {
                if ("left" in lower) ManeuverType.UTURN_LEFT else ManeuverType.UTURN_RIGHT
            }
            "sharp left" in lower -> ManeuverType.TURN_SHARP_LEFT
            "sharp right" in lower -> ManeuverType.TURN_SHARP_RIGHT
            "slight left" in lower || "bear left" in lower -> ManeuverType.TURN_SLIGHT_LEFT
            "slight right" in lower || "bear right" in lower -> ManeuverType.TURN_SLIGHT_RIGHT
            "turn left" in lower -> ManeuverType.TURN_LEFT
            "turn right" in lower -> ManeuverType.TURN_RIGHT
            "merge" in lower -> ManeuverType.MERGE
            "ramp" in lower && "left" in lower -> ManeuverType.RAMP_LEFT
            "ramp" in lower && "right" in lower -> ManeuverType.RAMP_RIGHT
            "fork" in lower && "left" in lower -> ManeuverType.FORK_LEFT
            "fork" in lower && "right" in lower -> ManeuverType.FORK_RIGHT
            "roundabout" in lower && "left" in lower -> ManeuverType.ROUNDABOUT_LEFT
            "roundabout" in lower -> ManeuverType.ROUNDABOUT_RIGHT
            else -> ManeuverType.STRAIGHT
        }
    }

    /**
     * Extracts a street name from Directions API HTML instructions.
     */
    private fun extractStreetName(htmlInstructions: String): String {
        // The street name is typically inside a <b> tag
        val match = Regex("<b>(.*?)</b>").find(htmlInstructions)
        return match?.groupValues?.get(1) ?: "Unknown road"
    }

    /**
     * Finds the current route step based on position.
     */
    private fun findCurrentStep(position: LatLng, route: NavigationRoute): Int {
        var closestStepIndex = currentStepIndex
        var minDistance = Double.MAX_VALUE

        for (i in currentStepIndex until route.steps.size) {
            val step = route.steps[i]
            val distance = calculateDistance(position, step.startLocation)
            if (distance < minDistance) {
                minDistance = distance
                closestStepIndex = i
            }
        }

        return closestStepIndex
    }

    /**
     * Calculates remaining distance from current position.
     */
    private fun calculateRemainingDistance(
        position: LatLng,
        route: NavigationRoute,
        stepIndex: Int
    ): Double {
        var remaining = 0.0

        // Distance to end of current step
        if (stepIndex < route.steps.size) {
            remaining += calculateDistance(position, route.steps[stepIndex].endLocation)

            // Distance of all remaining steps
            for (i in stepIndex + 1 until route.steps.size) {
                remaining += route.steps[i].distance
            }
        }

        return remaining
    }

    /**
     * Estimates remaining time based on distance and average speed.
     */
    private fun estimateRemainingTime(remainingDistance: Double, route: NavigationRoute): Int {
        val averageSpeed = 10.0 // meters per second (36 km/h)
        return (remainingDistance / averageSpeed).toInt()
    }

    /**
     * Calculates distance between two LatLng points using Haversine formula.
     */
    private fun calculateDistance(from: LatLng, to: LatLng): Double {
        val earthRadius = 6371000.0 // meters
        val dLat = Math.toRadians(to.latitude - from.latitude)
        val dLng = Math.toRadians(to.longitude - from.longitude)
        val a = sin(dLat / 2) * sin(dLat / 2) +
                cos(Math.toRadians(from.latitude)) * cos(Math.toRadians(to.latitude)) *
                        sin(dLng / 2) * sin(dLng / 2)
        val c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return earthRadius * c
    }
}

/**
 * Navigation state sealed class.
 */
sealed class NavigationState {
    object Idle : NavigationState()
    data class Routing(val destination: LatLng) : NavigationState()
    data class Active(
        val route: NavigationRoute,
        val remainingDistanceMeters: Double,
        val remainingTimeSeconds: Int,
        val currentStepIndex: Int
    ) : NavigationState()
    object Arrived : NavigationState()
}

/**
 * Navigation route data class.
 */
data class NavigationRoute(
    val start: LatLng,
    val destination: LatLng,
    val steps: List<RouteStep>,
    val totalDistanceMeters: Double,
    val estimatedTimeSeconds: Int,
    val polyline: List<LatLng>
)

/**
 * Route step data class.
 */
data class RouteStep(
    val startLocation: LatLng,
    val endLocation: LatLng,
    val distance: Double,
    val maneuver: ManeuverType,
    val streetName: String,
    val instruction: String
)

/**
 * Turn-by-turn instruction data class.
 */
data class NavigationInstruction(
    val type: ManeuverType,
    val streetName: String?,
    val distanceMeters: Int,
    val instruction: String
) {
    /**
     * Gets the appropriate icon for this maneuver type.
     */
    fun getManeuverIcon(): ImageVector {
        // Using available Material Icons as placeholders
        // TODO: Add custom navigation icons for better turn visualization
        return when (type) {
            ManeuverType.TURN_LEFT -> Icons.Default.ArrowBack
            ManeuverType.TURN_RIGHT -> Icons.Default.ArrowForward
            ManeuverType.TURN_SLIGHT_LEFT -> Icons.Default.ArrowBack
            ManeuverType.TURN_SLIGHT_RIGHT -> Icons.Default.ArrowForward
            ManeuverType.TURN_SHARP_LEFT -> Icons.Default.ArrowBack
            ManeuverType.TURN_SHARP_RIGHT -> Icons.Default.ArrowForward
            ManeuverType.UTURN_LEFT -> Icons.Default.ArrowBack
            ManeuverType.UTURN_RIGHT -> Icons.Default.ArrowForward
            ManeuverType.MERGE -> Icons.Default.ArrowForward
            ManeuverType.FORK_LEFT -> Icons.Default.ArrowBack
            ManeuverType.FORK_RIGHT -> Icons.Default.ArrowForward
            ManeuverType.ROUNDABOUT_LEFT -> Icons.Default.Refresh
            ManeuverType.ROUNDABOUT_RIGHT -> Icons.Default.Refresh
            ManeuverType.STRAIGHT -> Icons.Default.ArrowForward
            ManeuverType.RAMP_LEFT -> Icons.Default.ArrowBack
            ManeuverType.RAMP_RIGHT -> Icons.Default.ArrowForward
        }
    }
}

/**
 * Maneuver types enum.
 */
enum class ManeuverType {
    TURN_LEFT,
    TURN_RIGHT,
    TURN_SLIGHT_LEFT,
    TURN_SLIGHT_RIGHT,
    TURN_SHARP_LEFT,
    TURN_SHARP_RIGHT,
    UTURN_LEFT,
    UTURN_RIGHT,
    MERGE,
    FORK_LEFT,
    FORK_RIGHT,
    ROUNDABOUT_LEFT,
    ROUNDABOUT_RIGHT,
    STRAIGHT,
    RAMP_LEFT,
    RAMP_RIGHT
}
