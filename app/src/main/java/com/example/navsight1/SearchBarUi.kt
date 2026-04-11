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
import com.google.android.libraries.places.api.net.PlacesClient
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject

private const val TAG = "NavSight"

data class PlacePrediction(val placeId: String, val primaryText: String, val secondaryText: String)

data class RoutePreview(
    val destination: LatLng,
    val destinationName: String,
    val polyline: List<LatLng>
)

@Composable
fun SearchBarCard(pal: NavPalette, startLocation: LatLng?, placesClient: PlacesClient, onDestinationSelected: (LatLng) -> Unit) {
    WazeSearchBar(pal, startLocation, placesClient, onDestinationSelected)
}

@Composable
fun WazeSearchBar(
    pal: NavPalette,
    startLocation: LatLng?,
    placesClient: PlacesClient,
    onDestinationSelected: (LatLng) -> Unit
) {
    var searchText   by remember { mutableStateOf("") }
    var predictions  by remember { mutableStateOf<List<PlacePrediction>>(emptyList()) }
    var isSearching  by remember { mutableStateOf(false) }
    var routePreview by remember { mutableStateOf<RoutePreview?>(null) }
    val sessionToken = remember { com.google.android.libraries.places.api.model.AutocompleteSessionToken.newInstance() }
    val scope        = rememberCoroutineScope()
    val apiKey       = remember { BuildConfig.GOOGLE_MAPS_API_KEY }

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
                            try { fetchPlacePredictions(q, sessionToken, placesClient) { predictions = it } }
                            catch (e: Exception) { Log.e(TAG, "Places crashed", e) }
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
                            fetchPlaceLatLng(pred.placeId, placesClient) { ll ->
                                if (ll != null) {
                                    searchText = pred.primaryText
                                    val origin = startLocation
                                    if (origin != null) {
                                        scope.launch {
                                            val polyline = fetchDirectionsRoute(origin, ll, apiKey)
                                            isSearching  = false
                                            routePreview = RoutePreview(ll, pred.primaryText, polyline)
                                        }
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

fun fetchPlacePredictions(
    query: String,
    sessionToken: com.google.android.libraries.places.api.model.AutocompleteSessionToken,
    placesClient: PlacesClient,
    onResult: (List<PlacePrediction>) -> Unit
) {
    val req = com.google.android.libraries.places.api.net.FindAutocompletePredictionsRequest
        .builder().setSessionToken(sessionToken).setQuery(query).build()
    placesClient.findAutocompletePredictions(req)
        .addOnSuccessListener { r ->
            onResult(r.autocompletePredictions.map {
                PlacePrediction(it.placeId, it.getPrimaryText(null).toString(), it.getSecondaryText(null).toString())
            })
        }
        .addOnFailureListener { e -> Log.e(TAG, "Autocomplete failed", e); onResult(emptyList()) }
}

fun fetchPlaceLatLng(placeId: String, placesClient: PlacesClient, onResult: (LatLng?) -> Unit) {
    val fields = listOf(com.google.android.libraries.places.api.model.Place.Field.LAT_LNG)
    placesClient.fetchPlace(
        com.google.android.libraries.places.api.net.FetchPlaceRequest.newInstance(placeId, fields)
    )
        .addOnSuccessListener { onResult(it.place.latLng) }
        .addOnFailureListener { e -> Log.e(TAG, "Place fetch failed", e); onResult(null) }
}

suspend fun fetchDirectionsRoute(origin: LatLng, destination: LatLng, apiKey: String): List<LatLng> =
    withContext(Dispatchers.IO) {
        try {
            val url  = "https://maps.googleapis.com/maps/api/directions/json" +
                "?origin=${origin.latitude},${origin.longitude}" +
                "&destination=${destination.latitude},${destination.longitude}" +
                "&mode=walking&key=$apiKey"
            val conn = java.net.URL(url).openConnection() as java.net.HttpURLConnection
            conn.connectTimeout = 8000; conn.readTimeout = 8000
            val json = conn.inputStream.bufferedReader().readText()
            conn.disconnect()
            decodeDirectionsJson(json)
        } catch (e: Exception) {
            Log.e(TAG, "Directions API error: ${e.message}", e)
            emptyList()
        }
    }

private fun decodeDirectionsJson(json: String): List<LatLng> = try {
    val obj    = JSONObject(json)
    val routes = obj.getJSONArray("routes")
    if (routes.length() == 0) emptyList()
    else decodePolyline(routes.getJSONObject(0).getJSONObject("overview_polyline").getString("points"))
} catch (e: Exception) { Log.e(TAG, "Directions JSON parse error", e); emptyList() }

fun decodePolyline(encoded: String): List<LatLng> {
    val poly  = mutableListOf<LatLng>()
    var index = 0; val len = encoded.length
    var lat = 0; var lng = 0
    while (index < len) {
        var b: Int; var shift = 0; var result = 0
        do { b = encoded[index++].code - 63; result = result or ((b and 0x1f) shl shift); shift += 5 } while (b >= 0x20)
        val dLat = if (result and 1 != 0) (result shr 1).inv() else result shr 1; lat += dLat
        shift = 0; result = 0
        do { b = encoded[index++].code - 63; result = result or ((b and 0x1f) shl shift); shift += 5 } while (b >= 0x20)
        val dLng = if (result and 1 != 0) (result shr 1).inv() else result shr 1; lng += dLng
        poly.add(LatLng(lat / 1E5, lng / 1E5))
    }
    return poly
}
