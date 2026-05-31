package com.example.navsight1

import com.google.android.gms.maps.model.LatLng
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Validates OnlineRouter's OSRM response parsing — in particular that roundabout
 * (with exit number), U-turn, and ordinary turn maneuvers map to the right
 * [ManeuverType] and instruction text. Pure JVM (no HTTP); the OSRM service itself
 * was verified live to emit these maneuvers.
 */
class OnlineRouterParseTest {

    private val router = OnlineRouter()
    private val start = LatLng(32.80, 34.98)
    private val dest = LatLng(32.80, 35.00)

    // A minimal OSRM-shaped response: depart → turn-left → roundabout(exit 2) →
    // U-turn → arrive. (OSRM emits exactly these maneuver types; verified live.)
    private val osrmJson = """
        {"code":"Ok","routes":[{
          "distance":1000.0,"duration":120.0,
          "geometry":{"coordinates":[[34.98,32.80],[34.99,32.80],[35.00,32.80]]},
          "legs":[{"steps":[
            {"name":"Herzl","distance":300,"geometry":{"coordinates":[[34.98,32.80],[34.985,32.80]]},
             "maneuver":{"location":[34.98,32.80],"type":"depart","modifier":""}},
            {"name":"Allenby","distance":200,"geometry":{"coordinates":[[34.985,32.80],[34.99,32.80]]},
             "maneuver":{"location":[34.985,32.80],"type":"turn","modifier":"left"}},
            {"name":"Kikar","distance":100,"geometry":{"coordinates":[[34.99,32.80],[34.992,32.80]]},
             "maneuver":{"location":[34.99,32.80],"type":"roundabout","modifier":"right","exit":2}},
            {"name":"Back St","distance":150,"geometry":{"coordinates":[[34.992,32.80],[34.995,32.80]]},
             "maneuver":{"location":[34.992,32.80],"type":"turn","modifier":"uturn"}},
            {"name":"","distance":0,"geometry":{"coordinates":[[35.00,32.80]]},
             "maneuver":{"location":[35.00,32.80],"type":"arrive","modifier":""}}
          ]}]
        }]}
    """.trimIndent()

    @Test
    fun `parses geometry, distance and turn-by-turn steps`() {
        val r = router.parseOsrmRoute(osrmJson, start, dest)!!
        assertEquals("route polyline", 3, r.polyline.size)
        assertEquals(1000.0, r.totalDistanceMeters, 0.001)
        assertEquals(120, r.estimatedTimeSeconds)
        assertEquals("depart→left→roundabout→uturn→arrive", 5, r.steps.size)
    }

    @Test
    fun `turn-left maps correctly`() {
        val s = router.parseOsrmRoute(osrmJson, start, dest)!!.steps[1]
        assertEquals(ManeuverType.TURN_LEFT, s.maneuver)
        assertEquals("Allenby", s.streetName)
        assertTrue(s.instruction.contains("left") && s.instruction.contains("Allenby"))
    }

    @Test
    fun `roundabout maps to ROUNDABOUT with exit number in the instruction`() {
        val s = router.parseOsrmRoute(osrmJson, start, dest)!!.steps[2]
        assertEquals(ManeuverType.ROUNDABOUT_RIGHT, s.maneuver)
        assertTrue("instruction names the exit: ${s.instruction}", s.instruction.contains("exit 2"))
    }

    @Test
    fun `roundabout straight-through does NOT map to a right turn`() {
        // OSRM modifier "straight" through a roundabout must be ROUNDABOUT_STRAIGHT,
        // not ROUNDABOUT_RIGHT (the reported bug).
        val json = """
            {"code":"Ok","routes":[{"distance":50,"duration":10,
              "geometry":{"type":"LineString","coordinates":[[34.98,32.80],[34.99,32.80]]},
              "legs":[{"steps":[
                {"name":"A","distance":50,"geometry":{"type":"LineString","coordinates":[[34.98,32.80],[34.99,32.80]]},
                 "maneuver":{"location":[34.99,32.80],"type":"roundabout","modifier":"straight","exit":2}}
              ]}]}]}
        """.trimIndent()
        val s = router.parseOsrmRoute(json, start, dest)!!.steps[0]
        assertEquals(ManeuverType.ROUNDABOUT_STRAIGHT, s.maneuver)
    }

    @Test
    fun `u-turn maps to UTURN with a U-turn instruction`() {
        val s = router.parseOsrmRoute(osrmJson, start, dest)!!.steps[3]
        assertEquals(ManeuverType.UTURN_LEFT, s.maneuver)
        assertTrue("instruction: ${s.instruction}", s.instruction.contains("U-turn"))
    }

    @Test
    fun `non-Ok or empty response returns null`() {
        assertNull(router.parseOsrmRoute("""{"code":"NoRoute","routes":[]}""", start, dest))
        assertNull(router.parseOsrmRoute("""{"code":"Ok","routes":[]}""", start, dest))
    }
}
