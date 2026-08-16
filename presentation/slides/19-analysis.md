# Slide 19: Analysis

**Section:** Validation & Results · **Slide:** 19 of 23 · **Estimated Time:** 1 minute

## On-Slide Content
- **8.9 km/h RMSE sits AT the reference's own resolution limit** — the conditioned GPS speed reference is itself noisy at ~4–6 km/h (5 s windows); raw 1 s differencing ~20 km/h.
- **Medians nearly coincide:** 36.3 (NavSight) vs 35.1 km/h (GPS) = within 3.5% → no systematic speed scale error.
- **The decisive result:** NavSight 1,195 m (within 7% of true 1,280 m) while jammed GPS inflated to 1,705 m (+33%). **We hold accuracy where GPS fails.**
- **Known limitation 1 — turn under-read (pre-bridge):** camera loses near-road texture in tight turns → addressed by the complementary inertial bridge (+24% recovered, 62→77 m; stored deterministic offline A/B replay result).
- **Known limitation 2 — low-light / motion blur:** near-band flow degrades → addressed by per-point σ-floor + vote/zero-witness taxonomy (blur frames don't fabricate speed) and the accel bridge carrying blurred/starved frames.
- **Standstill is provably correct:** two-sided taxonomy yields exact 0.0; a one-sided gate would rectify ~34% (analytical estimate) of random noise into phantom positive speed.

## Talking Points (what the presenter SAYS)
- "The most important interpretation point: our 8.9 km/h speed RMSE is not a weakness — it sits *at* the resolution limit of the reference itself. The conditioned GPS reference is noisy at 4 to 6 km/h over 5-second windows, and raw 1-second GPS differencing is around 20 km/h. We can't measure ourselves more precisely than the yardstick allows, and we're at that floor."
- "That the medians nearly coincide — 36.3 versus 35.1 km/h, within 3.5% — tells us there is no systematic scale error in our speed. The error is noise-limited, not bias-limited."
- "The result we want the panel to remember: on the same physical route, NavSight stayed within 7% of the true 1,280 metres while jammed GPS over-reported by 33% to 1,705 metres. That is the entire thesis demonstrated in one chart — we hold accuracy in exactly the conditions where the incumbent technology collapses."
- "We are honest about two limitations. First, tight turns under-read before the bridge engages, because the camera momentarily loses near-road texture — the inertial bridge directly addresses this, recovering 24% of integrated distance on our stored deterministic offline A/B replay. Second, low light and motion blur degrade the near band — our per-point noise floor and the two-sided vote/zero-witness taxonomy mean blurred frames are *discarded*, never fabricated into speed, and the accel bridge carries us across them."
- "And standstill is provably correct by construction: the two-sided taxonomy reads an exact 0.0, whereas a naive one-sided magnitude gate would rectify roughly a third (~34%, analytical estimate) of random tracker noise into a phantom positive speed."

## Why We Chose This Approach
- **Alternatives considered for interpreting speed error:** report raw RMSE as an absolute pass/fail vs. contextualise it against the reference's own noise floor. We chose the latter — comparing an estimator to a noisy reference and reading the RMSE as pure system error would unfairly attribute the reference's ~4–6 km/h noise to NavSight.
- **Tradeoffs accepted:** we cannot prove sub-reference-floor speed accuracy without a better yardstick, so we deliberately scope the claim to "noise-limited, no scale bias" rather than overstating precision.
- **Benefits gained:** the distance metric (map-measured) carries the strong accuracy claim, while the speed metric carries the "no systematic bias" claim — each is defended on the reference appropriate to it.
- **Engineering reasoning:** good analysis separates *our* error from the *measurement's* error. The medians (scale), the RMSE-vs-floor (noise), and the GPS-inflation contrast (robustness) are three independent readings that together make a coherent, falsifiable story instead of one headline number.

## Potential Questions (Defense)
**Q:** Isn't it convenient to blame the GPS reference for your RMSE? How do you prove the error is the reference's and not yours?
**A:** Two independent checks. First, the medians nearly coincide — 36.3 vs 35.1 km/h, within 3.5% — so there is no systematic scale error on our side; a system error would shift the median, not just inflate variance. Second, the reference noise floor is derived from first principles: σ_v,ref ≈ √2 · σ_pos / τ ≈ 5.7 m/s ≈ 20 km/h per 1 s sample, which 5 s averaging reduces to 4–6 km/h. Our 8.9 km/h is the convolution of our true error with that floor — it is bounded below by the reference, not invented.
**Follow-up Q:** If you had a perfect reference, what RMSE would you expect?
**Follow-up A:** Lower than 8.9 km/h, because a large share of the measured RMSE is the reference's own 4–6 km/h noise added in quadrature. We can't quote an exact figure without a survey-grade reference — and we won't fabricate one — but the coinciding medians indicate the underlying systematic error is small.

**Q:** You admit turns under-read and blur degrades the camera. Why should we accept the system as validated?
**A:** Because both limitations are root-caused and *mitigated*, not hidden. Turn under-read is handled by the complementary inertial bridge — a stored deterministic offline A/B replay recovers +24% (62→77 m). Blur is handled by the per-point σ-floor and the two-sided vote/zero-witness taxonomy, so blurred or texture-starved frames are rejected rather than producing wrong speed, and the accel bridge carries those frames. The end-to-end result after these mitigations is still 91–93% distance accuracy.
**Follow-up Q:** Why is standstill *exactly* zero and not "near zero"?
**Follow-up A:** By design. We use a two-sided taxonomy: a pixel only votes a non-zero speed if it clears 3σ and is forward-coherent; otherwise, if its noise floor is ≤1 m/s, it is a zero-witness. Five or more zero-witnesses force a hard zero-lock. A one-sided magnitude gate would rectify ~34% (analytical estimate) of random-direction tracker noise into a positive speed — that's exactly the phantom-creep failure our taxonomy eliminates. The data shows it: the displayed speed reads exactly 0.0 km/h throughout the stop (~27 frames at exactly 0.0 on the cited ride).

## Speaker Notes
- Reference noise-floor derivation to have memorised: σ_v,ref ≈ √2 · σ_pos / τ ≈ 5.7 m/s ≈ 20 km/h per 1 s sample → ~4–6 km/h after 5 s windowing.
- The three-reading framing is the strongest defense: median (scale, within 3.5%) + RMSE-vs-floor (noise-limited) + GPS +33% inflation contrast (robustness). Lead with whichever the examiner challenges.
- Tie back to methodology: the GPS inflation (1,705 vs true 1,280) is *why* we don't use GPS as ground truth — limitation and justification reinforce each other.
- Bridge A/B is the cleanest evidence for the turn-under-read mitigation: +24%, 62→77 m, stored deterministic offline A/B replay, identical input.
- Pitfall: don't claim NavSight is more accurate than GPS in *all* conditions — the claim is robustness *under jamming* plus accuracy within 7–9%. Stay scoped.
- Pitfall: don't promise a hypothetical "perfect-reference RMSE" number — state that it would be lower and that we won't invent a figure without a better yardstick.
