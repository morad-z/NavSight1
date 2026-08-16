# Slide 01: Title

**Section:** Opening · **Slide:** 1 of 23 · **Estimated Time:** 0.5 minutes

## On-Slide Content
- **NavSight — Beyond GPS** (large, primary headline)
- *Precision Navigation in GPS-Denied Environments* (subtitle)
- B.Sc. Software Engineering — Final Project
- **Team:** Roey Ben Harush · Tamir Sobuh · Morad Zubidat
- **Supervisor:** Mr. Amit Dunsky
- Validated build: **v1.0-osm** (compileSdk/targetSdk 34 / Android 14; tested on an Android 15 / API 35 device)
- [Screenshot: tests/sims/val_2026_06_12/ui3.png — live map UI: 0 km/h at standstill, green VIO chip, ball snapped to the OSM road graph over the Google Maps base map (subtle background visual)]

## Talking Points (what the presenter SAYS)
- "Good morning. We are presenting NavSight — Beyond GPS: a precision navigation system that works when GPS does not."
- "This is our B.Sc. Software Engineering final project, supervised by Mr. Amit Dunsky."
- "Over the next ~30 minutes we will show how we navigate on a phone using only the camera and the inertial sensors — without depending on a live GPS fix — and we will back every claim with measured field data from Haifa."
- "The name says the goal: position, road, and speed, beyond GPS."

## Potential Questions (Defense)
**Q:** Why call it "Beyond GPS" — does the app remove GPS entirely?
**A:** No. The navigation *hot path* (camera + IMU fusion through the EKF, then offline OSM road-snapping) is fully GPS-free and produces the live "ball on the road" with speed. We deliberately keep GPS in the app as an optional reference, because removing it would reduce capability; the point is that NavSight does not *depend* on a live GPS fix. We validated this under real regional GNSS jamming in Haifa.

**Follow-up Q:** What exactly does "v1.0-osm" mean?
**Follow-up A:** It is our validated release build that ships the offline OpenStreetMap road-matching layer — the on-device Viterbi LocalMatcher that replaced Google Roads/Directions/OSRM. It bundles the Haifa OSM assets in the APK, so the navigation hot path (road-matching) needs no network; the displayed base map still uses Google Maps tiles. All the field results in this deck come from this exact build, captured on an Android 15 / API 35 device.

## Speaker Notes
- Keep this slide to ~30 seconds — it sets identity, not detail.
- Have ready if asked the device: Samsung Galaxy S21 Ultra (SM-G998B), Exynos 2100 SoC, Mali-G78 GPU.
- Do NOT quote a git hash on slides — refer to the build only as "v1.0-osm". (The signed V&V report PDF text references a commit internally; on stage say "v1.0-osm".)
- Pitfall: do not over-promise on this slide. Save the numbers (93.4% distance, 8.9 km/h speed RMSE) for the results section so the talk builds toward evidence.
- Pronounce the supervisor's title: "Mr. Amit Dunsky".
