# Slide 02: Project Overview

**Section:** Opening · **Slide:** 2 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **The problem (one line):** When GPS is jammed or denied, the phone no longer knows where it is or how fast it is moving.
- **What NavSight is:** A GPS-denied Android navigation app that fuses the **rear camera + IMU** through a **15-DOF error-state Extended Kalman Filter**, snaps the result to an **offline OpenStreetMap road-matching graph**, and shows a live **"ball on the road" + speed** — with zero dependence on a live GPS fix.
- **Pipeline:** Camera + IMU → EKF (visual-inertial fusion) → offline OSM road-matching → ball-on-road + speed
- **GPS-free navigation, computed on-device:** the navigation hot path (camera + IMU + on-device OSM road-matching) needs no GPS and no network; the bundled Haifa OSM assets ship in the APK. The displayed base map uses Google Maps tiles (cached/blank in airplane mode).
- **Stack:** Native C++17 / Android NDK core, OpenCV 4.5.3; Kotlin + Jetpack Compose UI
- [Screenshot: tests/sims/val_2026_06_12/ui3.png — live map UI: 0 km/h at standstill, green VIO chip, ball snapped to the OSM road graph over the Google Maps base map]

## Talking Points (what the presenter SAYS)
- "Navigation today assumes a working GPS fix. In Haifa, under real regional GNSS jamming, that assumption breaks — on Route A, jammed GPS over-reported path length by 33% (1,705 m vs a true 1,280 m), and on other jammed rides we observed multi-second GPS position freezes while moving."
- "NavSight removes that single point of failure. It fuses the rear camera and the inertial sensors through a 15-degree-of-freedom error-state Kalman filter, then snaps the trajectory onto an offline OpenStreetMap road-matching graph."
- "The user sees exactly what's on this screen: a ball travelling along the real road, with a live speed readout — and the navigation never touches a live GPS fix to do it."
- "The navigation runs entirely on the device. The Haifa OSM assets are bundled in the APK, so the GPS-free navigation has no network latency, no rate limits, and nothing for a jammer to attack. The base map you see is drawn from Google Maps tiles."
- "The heavy lifting is a native C++17 core on OpenCV; the interface is Kotlin and Jetpack Compose."

## Potential Questions (Defense)
**Q:** In one sentence, what is the core contribution here?
**A:** A GPS-free, on-device navigation pipeline that achieves road-level position and within-3.5% speed accuracy on a commodity Android phone by tightly fusing a single rear camera with the IMU and constraining the result to an offline OSM road-matching graph — validated under real GNSS jamming, where GPS itself fails. (The navigation hot path needs no GPS and no network; the displayed base map uses Google Maps tiles.)

**Follow-up Q:** Why is "offline" such a central design point rather than a nice-to-have?
**Follow-up A:** Because the threat model is jamming and denial. An online map-matching service (Google Roads, OSRM) needs a network and a position fix to query — exactly the things that are unreliable in our environment. By bundling the Haifa road graph in the APK and running a Viterbi matcher on-device, the navigation has no network latency, no rate limits, and is jamming-resilient — GPS-denied-capable where GPS itself fails. (The base map still uses Google Maps tiles, so it falls back to cached/blank tiles offline; the navigation does not depend on it.)

## Speaker Notes
- This slide is the 60-second elevator pitch — establish problem, solution, and "it's real and the navigation is GPS-free / on-device", then move on.
- Numbers to have ready if pressed: 15-DOF error-state EKF; OpenCV 4.5.3; the bundled Haifa OSM assets; native C++17 core with Kotlin/Compose UI. (If asked the exact asset size, say "on the order of low-tens of megabytes including the geocoder" rather than quoting a precise figure.)
- The screenshot is the proof-of-life: point at the "0 km/h" reading and the green VIO chip to show standstill detection and a healthy tracker before you've even explained how it works.
- Pitfall: don't dive into the EKF math here — slides 12–16 own that. Keep it conceptual: camera+IMU in, ball-on-road+speed out.
- If asked "where does GPS come in at all?" — defer to slide 17 (validation methodology): GPS is only a *secondary* reference on rides with verified GPS health, never the ground truth.
