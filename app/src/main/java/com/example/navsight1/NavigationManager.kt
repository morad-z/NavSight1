package com.example.navsight1

import android.content.Context
import android.util.Log
import androidx.annotation.Nullable
import androidx.annotation.NonNull
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ForkLeft
import androidx.compose.material.icons.filled.ForkRight
import androidx.compose.material.icons.filled.Merge
import androidx.compose.material.icons.filled.RampLeft
import androidx.compose.material.icons.filled.RampRight
import androidx.compose.material.icons.filled.RoundaboutLeft
import androidx.compose.material.icons.filled.RoundaboutRight
import androidx.compose.material.icons.filled.Straight
import androidx.compose.material.icons.filled.TurnLeft
import androidx.compose.material.icons.filled.TurnRight
import androidx.compose.material.icons.filled.TurnSharpLeft
import androidx.compose.material.icons.filled.TurnSharpRight
import androidx.compose.material.icons.filled.TurnSlightLeft
import androidx.compose.material.icons.filled.TurnSlightRight
import androidx.compose.material.icons.filled.UTurnLeft
import androidx.compose.material.icons.filled.UTurnRight
import androidx.compose.ui.graphics.vector.ImageVector
import com.google.android.gms.maps.model.LatLng
// MIGRATION 2026-05-30 (MAP_MATCHING_PLAN.md §8M Step K-routing*): Google
// Directions API removed; routing now runs offline via the local OSM router
// (OsmRouter / DijkstraRouter over the Haifa graph). gms LatLng retained (hybrid v1).
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
        // #8 route-pinned matching: pin the dot to the planned route when within this
        // cross-track corridor; beyond ROUTE_RECALC_DISTANCE_METERS for OFF_ROUTE_TICKS
        // consecutive updates we declare off-route and recompute the route from here.
        private const val ROUTE_CORRIDOR_METERS = 30.0
        private const val ROUTE_RECALC_DISTANCE_METERS = 50.0
        private const val OFF_ROUTE_TICKS = 4   // ~2 s at the 500 ms snap cadence
        // Advance to the next leg once we're within this of the upcoming maneuver point.
        private const val STEP_ADVANCE_RADIUS_METERS = 18.0
    }

    // OSM router (Step K-routing*). OnlineRouter routes to ANY destination via the
    // free no-key OSRM service with turn-by-turn steps (roundabout exits, U-turns);
    // it falls back to the local-region DijkstraRouter when offline. `apiKey` is an
    // ignored ctor param (no Google key for routing).
    private val onlineRouter = OnlineRouter()

    /* LEGACY (Google Directions API, removed in OSM migration 2026-05-30, §8M
       Step K-routing*). Commented out per the project's comment-out rule.

    private val geoApiContext: GeoApiContext? by lazy {
        if (apiKey.isBlank()) {
            Log.e(TAG, "API key is blank — Directions API routing will be disabled")
            null
        } else {
            GeoApiContext.Builder().apiKey(apiKey).build()
        }
    }
    */

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
    // #8 off-route detection: consecutive updates outside the route corridor, and a
    // guard so we never fire two reroutes at once.
    private var offRouteCount = 0
    @Volatile private var isRerouting = false

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
            offRouteCount = 0

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
     * Updates navigation against the current matched position and returns the position
     * to DISPLAY (#8 route-pinned matching). While navigating, the planned route is the
     * strongest prior: we project the dot onto the route polyline and, when it is within
     * the corridor, return that pinned point so the dot rides the planned route instead
     * of a parallel street. When the dot stays beyond ROUTE_RECALC_DISTANCE_METERS for
     * OFF_ROUTE_TICKS consecutive updates we declare off-route and recompute the route
     * from here (rerouting). suspend because the reroute is an HTTP call.
     *
     * @return the display position (route-pinned on-corridor, else the raw [snappedPosition]).
     */
    suspend fun updateVioPosition(snappedPosition: LatLng): LatLng {
        lastPosition = snappedPosition

        val state = _navigationState.value
        if (state !is NavigationState.Active) return snappedPosition

        val route = state.route

        // #8 — project onto the planned route + measure cross-track distance.
        val proj = RoutePinMath.nearestOnPolyline(snappedPosition, route.polyline)

        // Off-route detection + reroute.
        if (proj.distanceM > ROUTE_RECALC_DISTANCE_METERS) {
            offRouteCount++
            Log.i(TAG, "OFF_ROUTE_CHECK crossTrack=${"%.1f".format(proj.distanceM)}m " +
                "count=$offRouteCount/$OFF_ROUTE_TICKS rerouting=$isRerouting")
            if (offRouteCount >= OFF_ROUTE_TICKS && !isRerouting) {
                reroute(snappedPosition, route.destination)
                return snappedPosition   // route changed under us; show the raw matched dot
            }
        } else {
            offRouteCount = 0
        }

        // Arrival.
        val distanceToDestination = calculateDistance(snappedPosition, route.destination)
        if (distanceToDestination < ARRIVAL_THRESHOLD_METERS) {
            Log.i(TAG, "Arrived at destination!")
            _navigationState.value = NavigationState.Arrived
            _currentInstruction.value = null
            return snappedPosition
        }

        // Advance the LEG we're travelling on once we reach/pass the next maneuver point.
        // currentStepIndex = the step we're on; the UPCOMING maneuver is currentStepIndex+1.
        currentStepIndex = NavStepLogic.advanceLeg(
            currentStepIndex, route.steps.map { it.startLocation }, snappedPosition, STEP_ADVANCE_RADIUS_METERS
        )

        val remainingDistance = calculateRemainingDistance(snappedPosition, route, currentStepIndex)
        val remainingTime = estimateRemainingTime(remainingDistance, route)

        // Re-check state before writing — cancelNavigation() may have fired since we read it.
        if (_navigationState.value !is NavigationState.Active) return snappedPosition

        _navigationState.value = NavigationState.Active(
            route = route,
            remainingDistanceMeters = remainingDistance,
            remainingTimeSeconds = remainingTime,
            currentStepIndex = currentStepIndex
        )
        updateInstruction(snappedPosition)

        // Display: pin to the route within the corridor, else show the raw matched dot.
        return if (proj.distanceM <= ROUTE_CORRIDOR_METERS) proj.point else snappedPosition
    }

    /**
     * #8 — recompute the route from [from] to [destination] after going off-route.
     * Guarded by [isRerouting] so overlapping updates can't fire concurrent reroutes.
     */
    private suspend fun reroute(from: LatLng, destination: LatLng) {
        if (isRerouting) return
        isRerouting = true
        try {
            Log.i(TAG, "REROUTE: off-route → recomputing from $from to $destination")
            _navigationState.value = NavigationState.Routing(destination)
            val route = calculateRoute(from, destination)
            currentRoute = route
            currentStepIndex = 0
            offRouteCount = 0
            _navigationState.value = NavigationState.Active(
                route = route,
                remainingDistanceMeters = route.totalDistanceMeters,
                remainingTimeSeconds = route.estimatedTimeSeconds,
                currentStepIndex = 0
            )
            updateInstruction(from)
            Log.i(TAG, "REROUTE done: ${route.totalDistanceMeters}m, ${route.polyline.size} pts")
        } catch (e: Exception) {
            Log.e(TAG, "Reroute failed: ${e.message}", e)
        } finally {
            isRerouting = false
        }
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
        offRouteCount = 0
    }

    /**
     * Updates the current instruction based on distance to next maneuver.
     */
    private fun updateInstruction(currentPosition: LatLng? = lastPosition) {
        val state = _navigationState.value
        if (state !is NavigationState.Active) return
        val route = state.route

        // The UPCOMING maneuver is the START of the NEXT step (currentStepIndex+1) — the
        // maneuver of the current step is the turn we've ALREADY taken to get onto this leg,
        // so showing it (the old bug) lagged the banner by one turn with a growing distance.
        val nextIdx = currentStepIndex + 1
        if (nextIdx >= route.steps.size) {
            // On the final leg → head to the destination.
            val dist = currentPosition?.let { calculateDistance(it, route.destination) } ?: 0.0
            _currentInstruction.value = NavigationInstruction(
                type = ManeuverType.STRAIGHT,
                streetName = route.steps.getOrNull(currentStepIndex)?.streetName ?: "",
                distanceMeters = dist.toInt(),
                instruction = "Arrive at your destination"
            )
            return
        }

        val next = route.steps[nextIdx]
        val distanceToManeuver = currentPosition?.let { calculateDistance(it, next.startLocation) } ?: Double.MAX_VALUE
        _currentInstruction.value = NavigationInstruction(
            type = next.maneuver,
            streetName = next.streetName,            // the road we turn ONTO
            distanceMeters = distanceToManeuver.toInt(),
            instruction = next.instruction
        )
    }

    /**
     * Calculates a route using the offline OSM router (Step K-routing*). Falls back
     * to a straight-line route when the data layer isn't loaded, the points are
     * off-graph, or the router errors — so "routing works again" with $0, offline,
     * jamming-immune, and never crashes / never returns an empty route.
     *
     * v1 (DijkstraRouter) returns a road-following polyline + total distance. Full
     * turn-by-turn maneuvers/street-names (BRouter voice hints → ManeuverType) are
     * deferred to docs/NAVIGATION_PLAN.md (§0.5); v1 emits a single STRAIGHT step
     * spanning the route, same shape as calculateFallbackRoute.
     */
    private suspend fun calculateRoute(start: LatLng, destination: LatLng): NavigationRoute {
        return try {
            val route = onlineRouter.routeWithSteps(start, destination)
            if (route == null || route.polyline.size < 2) {
                Log.w(TAG, "router returned no track, falling back to straight line")
                calculateFallbackRoute(start, destination)
            } else {
                Log.i(TAG, "OSM route: ${route.totalDistanceMeters}m, ${route.estimatedTimeSeconds}s, " +
                    "${route.steps.size} steps, ${route.polyline.size} polyline points")
                route
            }
        } catch (e: Exception) {
            Log.e(TAG, "OSM routing error: ${e.message}", e)
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

    /* LEGACY (Google Directions HTML parsers, removed in OSM migration 2026-05-30,
       §8M Step K-routing*). The v1 DijkstraRouter emits no turn instructions, so
       these are dead. When BRouter lands, its already-classified VoiceHint.cmd maps
       1:1 onto ManeuverType via a pure code→enum table (toManeuverType) — no HTML
       parsing. Kept commented per the comment-out rule.

    private fun parseManeuver(htmlInstructions: String): ManeuverType { ... when { "turn left" in lower -> ... } }
    private fun extractStreetName(htmlInstructions: String): String { ... Regex("<b>(.*?)</b>") ... }
    */

    /* LEGACY (replaced 2026-05-31 by NavStepLogic.advanceLeg). findCurrentStep picked the
       nearest step START from currentStepIndex forward, which made the banner show the
       maneuver at the start of the leg you're ON (the turn already taken) for the first
       half of every leg — the off-by-one that lagged the instruction. Kept per the
       comment-out rule.

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
    */

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
 * #8 — pure point-to-polyline projection for route-pinned matching. Equirectangular
 * local metres around the query point (good to <0.1 % over a city). JVM-unit-testable.
 */
object RoutePinMath {
    private const val M_PER_DEG_LAT = 111_320.0

    data class Proj(val point: LatLng, val distanceM: Double)

    /** Nearest point on [polyline] to [p] + the cross-track distance in metres. */
    fun nearestOnPolyline(p: LatLng, polyline: List<LatLng>): Proj {
        if (polyline.isEmpty()) return Proj(p, Double.MAX_VALUE)
        if (polyline.size == 1) return nearestOnSegment(p, polyline[0], polyline[0])
        var best = Proj(polyline[0], Double.MAX_VALUE)
        for (i in 0 until polyline.size - 1) {
            val seg = nearestOnSegment(p, polyline[i], polyline[i + 1])
            if (seg.distanceM < best.distanceM) best = seg
        }
        return best
    }

    /** Nearest point on segment a→b to [p] + distance (m). p is treated as the origin. */
    fun nearestOnSegment(p: LatLng, a: LatLng, b: LatLng): Proj {
        val mPerDegLng = M_PER_DEG_LAT * cos(Math.toRadians(p.latitude))
        val ax = (a.longitude - p.longitude) * mPerDegLng; val ay = (a.latitude - p.latitude) * M_PER_DEG_LAT
        val bx = (b.longitude - p.longitude) * mPerDegLng; val by = (b.latitude - p.latitude) * M_PER_DEG_LAT
        val dx = bx - ax; val dy = by - ay
        val len2 = dx * dx + dy * dy
        // t = projection of the origin onto a→b, clamped to the segment.
        val t = if (len2 < 1e-9) 0.0 else (-(ax * dx + ay * dy) / len2).coerceIn(0.0, 1.0)
        val projX = ax + t * dx; val projY = ay + t * dy
        val dist = sqrt(projX * projX + projY * projY)
        val pt = LatLng(p.latitude + projY / M_PER_DEG_LAT, p.longitude + projX / mPerDegLng)
        return Proj(pt, dist)
    }
}

/**
 * Pure leg-advance logic for turn-by-turn. The next maneuver is at the START of the
 * NEXT step; [advanceLeg] moves the current-leg index forward once we have reached
 * (within [reachRadiusM]) or already passed that maneuver point. JVM-unit-testable.
 */
object NavStepLogic {
    fun advanceLeg(currentIdx: Int, stepStarts: List<LatLng>, pos: LatLng, reachRadiusM: Double): Int {
        var idx = currentIdx.coerceIn(0, maxOf(0, stepStarts.size - 1))
        // Advance ONLY when we have actually reached (within reachRadiusM of) the next maneuver
        // point. An earlier "idx+2 is closer than idx+1 ⇒ already passed" shortcut was REMOVED:
        // on an out-and-back / doubling-back route the return-leg maneuver point can be closer in
        // straight-line metres than the turnaround we have NOT yet reached, which silently skipped
        // (and never announced) the upcoming turn. reachRadius alone is monotone + skip-free and
        // catches every maneuver at realistic speeds (a 500 ms tick advances ≤14 m at 100 km/h,
        // well inside an 18 m radius).
        while (idx + 1 < stepStarts.size &&
            RoadSnapMath.haversineM(pos.latitude, pos.longitude,
                stepStarts[idx + 1].latitude, stepStarts[idx + 1].longitude) <= reachRadiusM) {
            idx++
        }
        return idx
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
    /** Banner title: the road we turn onto, or a short maneuver label when it's unnamed. */
    fun title(): String = if (!streetName.isNullOrBlank()) streetName else when (type) {
        ManeuverType.TURN_LEFT, ManeuverType.TURN_SLIGHT_LEFT, ManeuverType.TURN_SHARP_LEFT -> "Turn left"
        ManeuverType.TURN_RIGHT, ManeuverType.TURN_SLIGHT_RIGHT, ManeuverType.TURN_SHARP_RIGHT -> "Turn right"
        ManeuverType.UTURN_LEFT, ManeuverType.UTURN_RIGHT -> "Make a U-turn"
        ManeuverType.ROUNDABOUT_LEFT, ManeuverType.ROUNDABOUT_RIGHT, ManeuverType.ROUNDABOUT_STRAIGHT -> "Roundabout"
        ManeuverType.MERGE -> "Merge"
        ManeuverType.FORK_LEFT -> "Keep left"
        ManeuverType.FORK_RIGHT -> "Keep right"
        ManeuverType.RAMP_LEFT, ManeuverType.RAMP_RIGHT -> "Take the ramp"
        ManeuverType.STRAIGHT -> "Continue"
    }

    fun getManeuverIcon(): ImageVector = when (type) {
        ManeuverType.TURN_LEFT -> Icons.Filled.TurnLeft
        ManeuverType.TURN_RIGHT -> Icons.Filled.TurnRight
        ManeuverType.TURN_SLIGHT_LEFT -> Icons.Filled.TurnSlightLeft
        ManeuverType.TURN_SLIGHT_RIGHT -> Icons.Filled.TurnSlightRight
        ManeuverType.TURN_SHARP_LEFT -> Icons.Filled.TurnSharpLeft
        ManeuverType.TURN_SHARP_RIGHT -> Icons.Filled.TurnSharpRight
        ManeuverType.UTURN_LEFT -> Icons.Filled.UTurnLeft
        ManeuverType.UTURN_RIGHT -> Icons.Filled.UTurnRight
        ManeuverType.MERGE -> Icons.Filled.Merge
        ManeuverType.FORK_LEFT -> Icons.Filled.ForkLeft
        ManeuverType.FORK_RIGHT -> Icons.Filled.ForkRight
        ManeuverType.ROUNDABOUT_LEFT -> Icons.Filled.RoundaboutLeft
        ManeuverType.ROUNDABOUT_RIGHT -> Icons.Filled.RoundaboutRight
        // No straight-through roundabout glyph in this icon set → up arrow (correct
        // direction); the "Roundabout" title + "take exit N" text carry the context.
        ManeuverType.ROUNDABOUT_STRAIGHT -> Icons.Filled.Straight
        ManeuverType.STRAIGHT -> Icons.Filled.Straight
        ManeuverType.RAMP_LEFT -> Icons.Filled.RampLeft
        ManeuverType.RAMP_RIGHT -> Icons.Filled.RampRight
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
    ROUNDABOUT_STRAIGHT,
    STRAIGHT,
    RAMP_LEFT,
    RAMP_RIGHT
}
