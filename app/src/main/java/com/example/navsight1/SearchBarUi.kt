package com.example.navsight1

import android.util.Log
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.google.android.gms.maps.model.LatLng
// MIGRATION 2026-05-30 (MAP_MATCHING_PLAN.md §8M Step K-search*): Places API
// removed; destination search now runs offline against the local Haifa OSM
// geocode index (OfflineGeocoder). gms LatLng retained (hybrid v1).
import kotlinx.coroutines.launch

private const val TAG = "NavSight"

data class PlacePrediction(val placeId: String, val primaryText: String, val secondaryText: String)

data class RoutePreview(
    val destination: LatLng,
    val destinationName: String,
    val polyline: List<LatLng>
)

@Composable
fun SearchBarCard(pal: NavPalette, startLocation: LatLng?, geocoder: OfflineGeocoder, onDestinationSelected: (LatLng) -> Unit) {
    WazeSearchBar(pal, startLocation, geocoder, onDestinationSelected)
}

@Composable
fun WazeSearchBar(
    pal: NavPalette,
    startLocation: LatLng?,
    geocoder: OfflineGeocoder,
    onDestinationSelected: (LatLng) -> Unit
) {
    var searchText   by remember { mutableStateOf("") }
    var predictions  by remember { mutableStateOf<List<PlacePrediction>>(emptyList()) }
    var isSearching  by remember { mutableStateOf(false) }
    var routePreview by remember { mutableStateOf<RoutePreview?>(null) }
    val scope        = rememberCoroutineScope()

    Column(Modifier.fillMaxWidth()) {
        Surface(color = pal.card, shape = RoundedCornerShape(18.dp),
            border = BorderStroke(1.5.dp, pal.teal.copy(0.4f)), shadowElevation = 10.dp) {
            Row(Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 13.dp),
                verticalAlignment = Alignment.CenterVertically) {
                Icon(Icons.Default.Search, null, tint = pal.teal, modifier = Modifier.size(22.dp))
                Spacer(Modifier.width(10.dp))
                BasicTextField(
                    value       = searchText,
                    onValueChange = { q ->
                        searchText   = q
                        routePreview = null
                        if (q.length >= 2) scope.launch {
                            try { predictions = geocoder.predictions(q, startLocation) }
                            catch (e: Exception) { Log.e(TAG, "Offline geocode crashed", e) }
                        } else predictions = emptyList()
                    },
                    modifier   = Modifier.weight(1f),
                    textStyle  = TextStyle(color = pal.textPrimary, fontSize = 15.sp),
                    singleLine = true,
                    decorationBox = { inner ->
                        if (searchText.isEmpty()) Text("Where to?", color = pal.textSecondary, fontSize = 15.sp)
                        inner()
                    }
                )
                if (searchText.isNotEmpty()) {
                    IconButton(onClick = { searchText = ""; predictions = emptyList(); routePreview = null },
                        modifier = Modifier.size(30.dp)) {
                        Icon(Icons.Default.Close, null, tint = pal.textSecondary, modifier = Modifier.size(18.dp))
                    }
                }
            }
        }

        if (predictions.isNotEmpty()) {
            Spacer(Modifier.height(4.dp))
            Surface(color = pal.card, shape = RoundedCornerShape(16.dp),
                border = BorderStroke(1.dp, pal.cardBorder), shadowElevation = 6.dp) {
                Column {
                    predictions.forEach { pred ->
                        Surface(onClick = {
                            isSearching = true
                            predictions = emptyList()
                            scope.launch {
                                val ll = geocoder.resolve(pred.placeId)
                                if (ll != null) {
                                    searchText = pred.primaryText
                                    val origin = startLocation
                                    if (origin != null) {
                                        val polyline = fetchOfflineRoute(origin, ll)
                                        isSearching  = false
                                        routePreview = RoutePreview(ll, pred.primaryText, polyline)
                                    } else {
                                        isSearching = false
                                        onDestinationSelected(ll)
                                        searchText = ""
                                    }
                                } else { isSearching = false }
                            }
                        }, color = Color.Transparent) {
                            Row(Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 11.dp),
                                verticalAlignment = Alignment.CenterVertically) {
                                Icon(Icons.Default.Place, null, tint = pal.teal.copy(0.55f), modifier = Modifier.size(16.dp))
                                Spacer(Modifier.width(10.dp))
                                Column {
                                    Text(pred.primaryText, color = pal.textPrimary,
                                        fontWeight = FontWeight.Medium, fontSize = 14.sp,
                                        maxLines = 1, overflow = TextOverflow.Ellipsis)
                                    Text(pred.secondaryText, color = pal.textSecondary, fontSize = 11.sp)
                                }
                            }
                        }
                        if (pred != predictions.last()) HorizontalDivider(color = pal.cardBorder, thickness = 0.5.dp)
                    }
                }
            }
        }

        if (isSearching) {
            Spacer(Modifier.height(4.dp))
            LinearProgressIndicator(modifier = Modifier.fillMaxWidth().clip(RoundedCornerShape(4.dp)), color = pal.teal)
        }

        routePreview?.let { preview ->
            Spacer(Modifier.height(6.dp))
            Surface(color = pal.card, shape = RoundedCornerShape(18.dp),
                border = BorderStroke(1.5.dp, pal.teal.copy(0.5f)), shadowElevation = 8.dp) {
                Column(Modifier.padding(14.dp)) {
                    Row(Modifier.fillMaxWidth(), Arrangement.SpaceBetween, Alignment.CenterVertically) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Box(Modifier.size(38.dp).clip(CircleShape).background(pal.teal.copy(0.14f)),
                                contentAlignment = Alignment.Center) {
                                Icon(Icons.Default.Place, null, tint = pal.teal, modifier = Modifier.size(20.dp))
                            }
                            Spacer(Modifier.width(10.dp))
                            Column {
                                Text(preview.destinationName, color = pal.textPrimary,
                                    fontWeight = FontWeight.Bold, fontSize = 14.sp,
                                    maxLines = 1, overflow = TextOverflow.Ellipsis)
                                Text(
                                    if (preview.polyline.isNotEmpty()) "${preview.polyline.size} waypoints" else "Route ready",
                                    color = pal.textSecondary, fontSize = 11.sp
                                )
                            }
                        }
                        IconButton(onClick = { routePreview = null; searchText = "" }, modifier = Modifier.size(28.dp)) {
                            Icon(Icons.Default.Close, null, tint = pal.textSecondary, modifier = Modifier.size(16.dp))
                        }
                    }
                    Spacer(Modifier.height(12.dp))
                    Button(
                        onClick  = { onDestinationSelected(preview.destination); routePreview = null; searchText = "" },
                        modifier = Modifier.fillMaxWidth().height(44.dp),
                        colors   = ButtonDefaults.buttonColors(containerColor = pal.teal),
                        shape    = RoundedCornerShape(14.dp)
                    ) {
                        Icon(Icons.Default.KeyboardArrowRight, null, tint = Color.White, modifier = Modifier.size(18.dp))
                        Spacer(Modifier.width(6.dp))
                        Text("Start", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 14.sp)
                    }
                }
            }
        }
    }
}

/**
 * Route polyline (preview) via the OSM router. OnlineRouter routes to ANY
 * destination (free no-key OSRM), falling back to the local dynamic region when
 * offline. Empty → the route-preview card just shows "Route ready".
 */
suspend fun fetchOfflineRoute(origin: LatLng, destination: LatLng): List<LatLng> =
    OnlineRouter().route(origin, destination)

/* LEGACY (Google Places + Directions API, removed in OSM migration 2026-05-30,
   §8M Step K-search* / K-routing*). Replaced by OfflineGeocoder (predictions/
   resolve) + fetchOfflineRoute above. Commented out per the project's
   comment-out-don't-delete rule.

fun fetchPlacePredictions(query, sessionToken, placesClient, onResult) { ... PlacesClient.findAutocompletePredictions ... }
fun fetchPlaceLatLng(placeId, placesClient, onResult) { ... placesClient.fetchPlace ... }
suspend fun fetchDirectionsRoute(origin, destination, apiKey) { ... maps.googleapis.com/maps/api/directions ... }
private fun decodeDirectionsJson(json) { ... overview_polyline ... }
fun decodePolyline(encoded) { ... Google encoded-polyline decode ... }
*/
