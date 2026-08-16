# Slide 17: Validation Methodology

**Section:** Validation & Results · **Slide:** 17 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **Ground truth = map-measured distance**, not GPS. Routes measured on Google Maps; GPS is jamming-corrupted in Haifa.
- **GPS used only as a *secondary* speed reference** — and only on rides whose GPS health was verified (median fix accuracy 4 m, no freezes/jumps).
- **Conditioned-reference protocol** before every speed comparison:
  - Plausibility filter rejects samples with |dv/dt| > 3 m/s² vs *both* neighbours (position jitter).
  - Both signals averaged over the **same non-overlapping 5 s windows** → residual reference noise ~4–6 km/h.
- **Deterministic offline replay** — unmodified native engine re-runs recorded frames + IMU for exact A/B comparison on identical input.
- **Automated suites:** 95-case Kotlin unit suite (95 @Test across 14 files), C++ unit suite, CI replay scoring on recorded fixtures.
- [Screenshot: tests/sims/val_2026_06_03b/routeA_google_measure.png — Google Maps 1.28 km measurement = the jamming-resilient ground truth]

## Talking Points (what the presenter SAYS)
- "Before any number, the first question a fair examiner asks is: what is your ground truth? Ours is deliberately *not* GPS — because in Haifa, under real GNSS jamming, GPS is the thing we are trying to replace. We measure each route's true length on Google Maps and treat that as truth."
- "We still use GPS, but only as a *secondary* speed reference, and only on rides where we independently verified GPS health — median fix accuracy of 4 metres, no frozen positions, no jumps. On jammed rides we don't trust GPS at all."
- "Even healthy GPS speed is noisy, so we condition it: first a plausibility filter throws out any sample whose implied acceleration exceeds physical capability — over 3 m/s² against both neighbours — then we average both NavSight and GPS over the same 5-second windows. That drops the reference noise to about 4 to 6 km/h."
- "Crucially, our A/B experiments run on a deterministic offline replay: the *unmodified* native engine re-plays the recorded camera frames and IMU, so a feature's effect is measured on byte-identical input. On top of that, a 95-case Kotlin unit suite runs, plus a C++ suite and CI scoring on recorded fixtures."

## Why We Chose This Approach
- **Alternatives considered:** (a) Use raw GPS as ground truth — rejected, because the very jamming we target corrupts it (on Route A, jammed GPS over-reported +33% path length; on other jammed rides we observed multi-second GPS position freezes). (b) Compare against a survey-grade external tracker — out of scope/cost for a B.Sc. project and still jammed in the same airspace. (c) Trust on-device GPS speed directly — rejected, raw 1 s differencing has a ~20 km/h noise floor.
- **Tradeoffs accepted:** Map-measured distance gives total length but not an instantaneous reference, so speed truth needs the conditioned GPS path; we accept that GPS is only usable on verified-healthy rides, which limits how many rides qualify for speed validation.
- **Benefits gained:** A ground truth that is *unaffected by the failure mode under test*, a reproducible offline harness that removes "it worked on my walk" ambiguity, and a continuous-integration safety net so regressions are caught before a field walk.
- **Engineering reasoning:** Validation must be falsifiable and independent of the system's own weakest input. We separate "is it accurate?" (map-measured distance) from "how noisy is the comparison?" (the GPS reference's own 4–6 km/h floor), so a defender can see exactly which error is ours and which is the reference's.

## Potential Questions (Defense)
**Q:** Why don't you just use GPS as ground truth like every other navigation paper?
**A:** Because GPS is precisely the signal we are replacing. Under the real Haifa jamming, on Route A jammed GPS over-reported a +33% inflated path length — GPS reported 1,705 m on a route that truly measures 1,280 m; on other jammed rides we observed multi-second GPS position freezes while moving. Using a corrupted signal as truth would let the corruption flatter or unfairly penalise us. Map-measured route distance is unaffected by jamming, so it is the honest baseline.
**Follow-up Q:** Then how can you trust GPS even as a secondary speed reference?
**Follow-up A:** Only conditionally. We use it for speed only on rides whose GPS health we verified independently — median fix accuracy 4 m, no freezes or jumps — and even then we run a plausibility filter (|dv/dt| > 3 m/s² rejected) and 5-second window averaging, which pushes residual reference noise down to about 4–6 km/h. We never use GPS from a ride that shows jamming artifacts.

**Q:** How do you know your A/B feature comparisons aren't just run-to-run variation?
**A:** They aren't, because we use a deterministic offline replay. The unmodified native engine re-runs the exact same recorded camera frames and IMU samples, so the only difference between A and B is the feature flag. That is how we cleanly measured the inertial bridge contributing +24% integrated distance (62 m → 77 m) on identical input — no walk-to-walk noise involved.[^abnote]

[^abnote]: The +24% (62 m → 77 m) figure is a stored deterministic offline A/B replay result (recorded, not recomputed live).
**Follow-up Q:** Is the replay engine the same code that runs on the phone?
**Follow-up A:** Yes — it is the unmodified native C++ engine; replay only changes how frames and IMU are fed in, not the algorithm. We also back it with a 95-case Kotlin unit suite, a C++ unit suite, and CI scoring on recorded fixtures so the replayed code is the shipped code.

## Speaker Notes
- Have the noise-floor derivation ready: GPS-differenced reference noise σ_v,ref ≈ √2 · σ_pos / τ ≈ 5.7 m/s ≈ 20 km/h on raw 1 s samples; the 5 s windowing is what brings it to 4–6 km/h.
- Window-selection detail (in case pressed): a window is kept only if it has ≥3 GPS samples and ≥10 NavSight samples, with a moving threshold of speed > 0.28 m/s.
- Emphasise the *separation of concerns*: distance accuracy is validated against map measurement; speed accuracy is validated against conditioned GPS. Don't let the panel conflate the two references.
- Pitfall to avoid: do not claim GPS is "wrong everywhere" — be precise that jamming is intermittent, which is exactly why some rides are GPS-healthy and usable and others are discarded.
- One sim-pipeline caveat worth knowing internally (don't volunteer unless asked): an early apparent "jamming" artifact was once traced to resampling the same fix every tick; on those rides GPS was actually healthy at 4 m. We dedupe consecutive identical fixes before differencing.
