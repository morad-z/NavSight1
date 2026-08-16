# Slide 04: Motivation — When GPS Lies, You Lose the Map

**Section:** Problem & Context · **Slide:** 4 of 23 · **Estimated Time:** 1.5 minutes

## On-Slide Content
- **GPS is not a guarantee — it is a signal that can be jammed, spoofed, or denied.**
- Real evidence from Haifa, under regional GNSS jamming:
  - **+33% inflated path length (Route A)** — jammed GPS measured **1,705 m** on a route that truly measures **1,280 m**, while NavSight estimated **1,195 m**.
  - **Multi-second position freezes** — on other jammed rides the GPS position froze for seconds at a time while the phone was physically moving.
- Existing nav apps (Google/Waze) assume a live fix — when the fix is corrupted, they silently follow the lie.
- NavSight answers: *navigate with zero dependence on a live GPS fix.*
- Who benefits: civilians in jammed regions, emergency/field responders, indoor & tunnel users, privacy-sensitive users.
- [Screenshot: tests/sims/val_2026_06_03b/routeA_cumdist_gps_vs_dot.png — GPS 1,705 m inflated vs NavSight 1,195 m on a 1,280 m route]

## Talking Points (what the presenter SAYS)
- "Every navigation app on your phone today rests on one silent assumption: that the GPS fix is true. In Haifa, that assumption breaks. We tested under real regional GNSS jamming — not a simulation — and we measured what jamming actually does."
- "On Route A, GPS reported 1,705 metres for a road that we measured on the map at exactly 1,280 metres — a 33% inflation — while NavSight estimated 1,195 metres. On other jammed rides we also saw the GPS position freeze for several seconds at a time while we were physically moving. The fix wasn't missing; it was confidently wrong."
- "That is the real-world pain point. When GPS degrades, today's apps don't warn you and they don't recover — they just plot a fictional position. The user is stranded with a map that no longer matches the world."
- "This matters far beyond wartime jamming: urban canyons, tunnels, parking garages, and indoor spaces all deny GPS the same way. And there is a research angle too — most academic visual-inertial work assumes GPS as ground truth, which is exactly the assumption we cannot make here."
- "NavSight's motivation is therefore concrete: deliver a live position and speed that survive when the GPS fix is jammed, frozen, or simply absent — GPS-free navigation computed on-device."

## Why We Chose This Approach
- **Alternatives considered:** (1) keep trusting GPS and add smoothing/outlier rejection — rejected because the failure is not noise, it is a *confidently frozen or inflated* fix that smoothing cannot detect; (2) cellular / Wi-Fi positioning — rejected for coarse accuracy (tens to hundreds of metres) and dependence on infrastructure that is also unreliable in denied environments; (3) extra hardware (LiDAR, UWB beacons) — rejected as non-commodity and not deployable on a standard phone.
- **Tradeoffs accepted:** building a self-contained camera+IMU pipeline is harder than calling a GPS API, and it requires its own ground truth (we use map-measured route distances, which are independent of GPS and therefore unaffected by jamming).
- **Benefits gained:** the system degrades gracefully precisely where commercial apps fail; the navigation is GPS-free and computed on-device (offline OSM road-matching, no GPS or network in the navigation hot path — the base map still uses Google Maps tiles); and because the hot path never needs a live fix, jamming can corrupt at most the display layer, not the trajectory itself.
- **Engineering reasoning:** we treat GPS as an *untrusted, optional* reference — never as ground truth — which is the only honest stance once you have measured a +33% inflation on real hardware, alongside multi-second GPS freezes on other jammed rides.

## Potential Questions (Defense)
**Q:** GPS jamming sounds like an edge case — why build an entire system around it?
**A:** It is not an edge case in our test region: we observed real, repeatable jamming on multiple Haifa rides, including a +33% path inflation on Route A (GPS 1,705 m vs the true 1,280 m) and multi-second GPS position freezes while moving on other jammed rides. And the same denial happens routinely in tunnels, parking garages, urban canyons, and indoors. The motivation is general; the jamming just made the failure mode unmissable.
**Follow-up Q:** If GPS is so unreliable here, why does the app still include GPS at all?
**Follow-up A:** Because GPS is genuinely useful when it is healthy, and removing it would make NavSight worse in the common case. We keep it but demote it: it is an optional, untrusted reference, never ground truth. The visual-inertial hot path runs entirely without a fix, so a corrupted GPS reading can never derail the trajectory.

**Q:** How do you know it was jamming and not just a buggy sensor or your own pipeline?
**A:** We cross-checked against map-measured ground truth, which is independent of GPS and therefore unaffected by jamming. A healthy GPS day on the same device gave a median fix accuracy of ~4 m with no freezes, whereas the jammed rides showed the +33% inflation on Route A and multi-second position freezes on others. We also separately ruled out a sim-pipeline bug (re-sampling the same fix each tick) that had once mimicked a freeze — the real rides were genuinely jammed.
**Follow-up Q:** Couldn't the user just notice the freeze and ignore it?
**Follow-up A:** No — the failure is silent. The app reports a plausible-looking frozen position with no error flag, so a human has no way to distinguish a real stop from a jammed freeze. That is exactly why an independent, GPS-free estimate of position and speed is necessary.

## Speaker Notes
- Lead with the hard, computed number — the +33% inflation on Route A (jammed GPS 1,705 m vs the true 1,280 m, NavSight 1,195 m) — plus the qualitative fact that other jammed rides showed multi-second GPS position freezes while moving. These are the emotional and technical anchor of the whole talk.
- Emphasise these are from real ambient GNSS jamming on the Samsung Galaxy S21 Ultra in Haifa, not synthetic.
- Have ready: the contrast that a *healthy* GPS day on the same hardware was ~4 m accurate — this proves the device and pipeline are fine; the jamming is external.
- Pitfall to avoid: do not over-claim that NavSight "replaces" GPS. The framing is precise — NavSight removes the *dependence* on a live fix; GPS stays as an optional reference.
- If asked about scale/region: ground truth is always map-measured distance (Google Maps), so even when both GPS and our estimate are imperfect we have an external yardstick that is independent of GPS and therefore unaffected by jamming.
