# Slide 15: Speed Estimation — Ground-Plane IPM + Inertial Bridge

**Section:** Deep Dive / Subsystems · **Slide:** 15 of 23 · **Estimated Time:** 2 minutes

## On-Slide Content
- **IPM (Inverse Perspective Mapping):** model the road as a plane at calibrated mount height **h = 1.05 m** below the camera; recover metric speed from optical flow — no learned depth in the speed path.
- Per road-pixel *i*: depth `Z_i = -h / (n̂·r_i)`; flow-per-speed `a_i = (u_fwd - r_i·u_fwd,z)/Z_i`; per-point speed `v_i = -(f_i·a_i)/(a_i·a_i)·(1/Δt)`.
- **Per-point noise floor:** `σv_i = (σpx / fx) / (|a_i|·Δt)`, with **σpx = 0.5 px**, fx ≈ 451.
- **Two-sided taxonomy** — a point **VOTES** iff `v_i > 3·σv_i` AND forward-coherent (`cos θ < -1/√2`, a 45° cone); a point **ZERO-WITNESSES** iff `|v_i| < 3·σ` and floor ≤ 1 m/s.
- **Resolve:** ≥5 votes → median speed; else ≥5 zero-witnesses → **EXACT 0** (zero-lock standstill); else **inertial bridge**.
- **Complementary inertial bridge:** predict `v_k⁻ = v_(k-1) + a_fwd·Δt` every frame; once ≥5 votes return, correct by EMA toward the vote-median (hard-0 on ≥5 zero-witnesses); trusted up to **~6 s** budget, then **decays at α = 0.15** (the post-budget decay rate, not the correction gain).
- [Diagram: diagrams/05-speed-estimation-ipm.md]
- [Screenshot: tests/sims/val_2026_06_03b/probe_cruise_0.jpg — rear-camera frame; amber = ground-plane sampling mask, green arrows = verified optical flow, red = production KLT]

## Talking Points (what the presenter SAYS)
- "Speed is what advances the ball down the road, so it has to be metric and trustworthy. A monocular camera has no inherent scale, so instead of asking the network 'how far is that?', we make one physical assumption: the road is a flat plane a known 1.05 metres below the camera. That single calibrated height turns a pixel into a metric depth."
- "For each road pixel we compute its depth from the plane, then how much optical flow one metre-per-second of forward motion *should* produce. A least-squares fit against the flow we actually measured gives a per-point speed — and critically, a per-point noise floor that tells us how much to trust it."
- "Then the key idea: a **two-sided** decision. A point only votes a speed if it's both above its own 3-sigma noise floor *and* pointing the right way — flow has to be forward-coherent inside a 45-degree cone. Separately, a point can be a 'zero-witness' if its speed is statistically indistinguishable from zero. Five votes give us a median speed; five zero-witnesses give us an *exact* zero."
- "That exact zero is why we don't creep at a stop. On the validation ride the displayed speed read exactly 0.0 km/h throughout the stop — the zero-witness lock holding the output hard at zero rather than smearing noise into a small filtered value."
- "When the road is blurred or starved of features — which happens whenever you're moving fast — we fall back to a complementary inertial bridge: integrate forward acceleration to predict speed, and correct it toward the vision vote-median whenever votes return. We trust that prediction for about six seconds, then it decays."

## Why We Chose This Approach
- **IPM ground-plane vs raw VIO scale.** Alternative: read speed directly from the EKF's velocity state. Rejected because monocular VIO scale is fragile and was the project's hardest-fought problem — a single shared scale factor chronically under-read on the scooter. The ground-plane gives a *direct geometric* speed from one calibrated constant (h = 1.05 m) instead of a drifting estimated scale. Tradeoff accepted: it assumes the road is locally planar and degrades on slopes/curbs — which is exactly when the inertial bridge takes over. The speed-path metric depth comes from the calibrated ground plane (h = 1.05 m), not from learned depth.
- **Two-sided gate vs one-sided magnitude gate.** A one-sided "is the speed big enough" gate is the obvious design — and it is the bug. Random tracker noise has random *magnitude*; a one-sided gate rectifies ~34% of pure noise (analytical estimate) into a positive speed, so the device reads a phantom 1–5 km/h while standing still. The two-sided taxonomy adds the forward-coherence cone (direction must agree, not just magnitude) and a zero-witness branch, so noise is classified as zero rather than smeared into motion. This is the single decision that makes standstill read *exactly* zero.
- **Complementary bridge vs vision-only.** Vision-only would simply have no speed during blur/starvation (77% of cruise frames showed feature starvation). The bridge predicts through those gaps from forward acceleration and re-anchors to vision the moment votes return. Benefit in stored A/B replay: integrated distance on a turn-heavy ride rose from 62 m to 77 m (+24%) on identical input — the missing distance the gaps were eating. (This is a stored deterministic offline A/B replay result, not a live recompute; the ~6 s / 138-frame bridge budget at the ~23 fps processed rate is real.)

## Potential Questions (Defense)
**Q:** The flat-road assumption seems fragile — what happens on a hill or a banked curve?
**A:** The plane model is only the *vote source*, not the final speed. When the geometry breaks (slope, blur, feature starvation), the per-point estimates fail the 3-sigma vote gate and we fall through to the inertial bridge, which integrates forward acceleration for up to ~6 seconds — bounded because a 0.3 m/s² accel-bias integrates to only ~1.8 m/s over that window — before decaying. So a hill doesn't produce a wrong speed; it produces a *bridged* speed that re-anchors to vision as soon as flat, textured road returns.
**Follow-up Q:** How accurate is the resulting speed in practice?
**Follow-up A:** Against a conditioned GPS reference over 5-second windows, RMSE was 8.9 km/h with a −1.9 km/h bias, and median 36.3 km/h versus 35.1 km/h — within 3.5%. That RMSE actually sits *at* the GPS reference's own noise floor of about 4–6 km/h at 5 s, so we're at the resolution limit of the thing we're comparing against, not above it.

**Q:** Why does standstill read *exactly* zero instead of a small filtered value?
**A:** The zero-witness branch is an explicit decision, not a clamp. When ≥5 points each test statistically indistinguishable-from-zero (|v_i| < 3σ with a sub-1-m/s floor), we hard-lock the output to 0.0 rather than taking a median of small noisy numbers. On the validation ride the displayed speed read exactly 0.0 km/h throughout the stop (the exact-0 zero-witness lock is real; the cited ride shows ~27 such frames at exactly 0.0). A magnitude-only filter cannot do this because it has no way to distinguish "small motion" from "rectified noise."
**Follow-up Q:** What is α = 0.15 in the bridge?
**Follow-up A:** α = 0.15 is the *post-budget decay rate*, not the vision-correction gain. The bridge predicts forward every frame (`v += a_fwd·Δt`) and, once ≥5 votes return, corrects by an EMA toward the vote-median (hard-zero on ≥5 zero-witnesses). Past the ~6 s budget, with no votes returning, the bridged speed decays at 0.15 so a stale prediction can't run away. The correction itself is driven by the returning votes, not by α.

## Speaker Notes
- Numbers to have ready: h = 1.05 m, fx ≈ 451, working res 640×480, σpx = 0.5 px, 3σ vote gate, 45° forward cone (cos θ < −1/√2), ≥5 votes / ≥5 zero-witnesses, bridge horizon ~6 s = 138 frames (at the ~23 fps processed VIO rate), post-budget decay α = 0.15, accel-bias ≤0.3 m/s² → ≤1.8 m/s.
- The screenshot is the money shot: amber mask = where we sample the road plane, green arrows = flow that passed the forward-coherence + 3σ gate, red = raw production KLT. Use it to make the vote/zero-witness distinction concrete.
- Emphasise the framing: geometry gives the *number*, the two-sided gate gives the *trust*, the bridge gives *continuity*. Three separate jobs.
- Pitfall to avoid: don't say learned depth feeds speed — it does NOT; the speed-path metric depth comes from the calibrated ground plane + mount height (h = 1.05 m) only.
- Connect forward to Slide 14: this speed is exactly what advances the matched ball along the road; the two slides are the position/distance split — map-matching owns lateral position, IPM owns along-track speed/distance.
- A/B proof: stored deterministic offline replay on identical input, 62 m → 77 m (+24%) with the bridge enabled (a stored result, not a live recompute) — the cleanest single-variable evidence the bridge is load-bearing.
