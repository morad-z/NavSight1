# Slide 23: Closing & Questions

**Section:** Impact & Closing · **Slide:** 23 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **NavSight — Beyond GPS**: precision navigation in GPS-denied environments, validated on real Haifa rides under live GNSS jamming.
- **Three headline results:**
  - **Distance: 91–93%** of true distance (Route A 1,195 m / 1,280 m = 93.4%; ride 18:02 = 91%, ρ = 0.91).
  - **Speed at the GPS resolution limit** — RMSE 8.9 km/h sits at the reference's own ~4–6 km/h noise floor; median 36.3 vs 35.1 km/h (within 3.5%).
  - **13× real-time headroom** — 15.4 ms median per frame vs the 200 ms budget (2.5× worst-case margin).
- GPS-free navigation computed on-device (offline OSM road-matching), on a commodity phone, with no GPS fix required; the base map uses Google Maps tiles.
- **Thank you** — Roey Ben Harush · Tamir Sobuh · Morad Zubidat · Supervisor: Mr. Amit Dunsky.
- **Questions?** — the team is ready.

## Talking Points (what the presenter SAYS)
- To close: NavSight shows that a normal Android phone, with no GPS fix, can navigate precisely on-device in an environment where GPS itself was actively failing — the navigation runs offline (camera + IMU + on-device OSM road-matching, no GPS in the hot path), while the displayed base map uses Google Maps tiles.
- We validated three things that matter together. Distance: we held 91 to 93 percent of true, map-measured distance, on a route where GPS was inflated by a third. Speed: our error sits right at the noise floor of a clean GPS reference — within 3.5 percent of the median — meaning we are as accurate as the reference can even measure. And timing: we run at 15.4 milliseconds per frame against a 200 millisecond budget, so there is 13× headroom on the target device.
- We did this on the camera and IMU alone, snapped to an offline OpenStreetMap road-matching graph in the bundled Haifa OSM assets — private, GPS-free, and jamming-resilient.
- Thank you to our supervisor Mr. Amit Dunsky and to the committee for your time. We would be glad to take your questions.

## Potential Questions (Defense)
**Q:** If you had to defend one number as your strongest result, which is it and why?
**A:** The speed result, because RMSE 8.9 km/h sits at the conditioned GPS reference's own ~4–6 km/h noise floor at a 5 s window — we are accurate to the limit of what the reference can measure, with a median of 36.3 vs 35.1 km/h, within 3.5%. It is the result hardest to dismiss as luck, because it is bounded by the reference's resolution, not ours.

**Follow-up Q:** Why report distance as a range, 91–93%?
**Follow-up A:** Because it is two independent rides with two ground-truth methods: Route A against Google-measured distance gave 93.4% (1,195 m of 1,280 m), and ride 18:02 against verified-healthy GPS gave 91% with ρ = 0.91. Reporting both, rather than the better one, is the honest summary.

**Q:** What is the single biggest limitation you want us to know?
**A:** Low-light speed and region coverage. Motion blur degrades the near-band optical flow, which is why a camera exposure cap is our named next lever, and we currently ship only the Haifa OSM region. Neither affects the validated results, and both are extensions of existing mechanisms rather than redesigns.

## Speaker Notes
- Keep this to one minute — restate the three numbers cleanly, thank, and open the floor; do not introduce new material here.
- Memorize the three headlines verbatim: 91–93% distance, speed RMSE 8.9 km/h at the GPS reference's ~4–6 km/h noise floor (within 3.5%), 13× real-time headroom (15.4 ms vs 200 ms).
- Refer to the build as "v1.0-osm" — never quote a git hash on slides.
- Names and supervisor must be on screen and spoken: Roey Ben Harush, Tamir Sobuh, Morad Zubidat; supervisor Mr. Amit Dunsky.
- Have the deeper backup ready in case the first question lands hard: standstill exact-0 zero-lock (displayed speed reads exactly 0.0 km/h throughout the stop), inertial-bridge A/B +24% (62→77 m, stored deterministic offline A/B replay result), and the GPS-jamming contrast (Route A jammed GPS 1,705 m vs 1,280 m truth, +33%).
