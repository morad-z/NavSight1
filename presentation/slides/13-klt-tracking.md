# Slide 13: KLT Tracking — The Visual Front-End

**Section:** Subsystems · **Slide:** 13 of 23 · **Estimated Time:** 1.5 minutes

## On-Slide Content
- **KLT optical flow** is the front-end: pyramidal Lucas-Kanade tracks the *same* corner points across consecutive frames — no per-frame re-detection, no per-frame descriptor matching.
- **Forward-backward consistency** filters every track: track forward, track back, keep only points that return to origin — rejects blur and occlusion outliers.
- **ORB relocalization** is the recovery path: when tracking is lost, ORB descriptors re-anchor the system to a known keyframe. **49 relocalization events observed** on the 2026-06-04 ride.
- **De-rotated, FB-verified flow** is what the speed estimator (IPM) and the EKF consume.
- The amber mask + green arrows in the camera figure are the FB-verified flow that survives; red is raw production KLT.
- [Screenshot: tests/sims/val_2026_06_03b/probe_cruise_0.jpg — rear-camera frame: amber = ground-plane sampling mask, green = verified optical flow, red = production KLT]

## Talking Points (what the presenter SAYS)
- "The visual front-end is built on KLT optical flow. The key idea is that we *track the same points* from frame to frame rather than re-detecting and re-describing features on every frame — that's what makes it cheap enough to keep up with the ~23-frames-a-second processed VIO rate on a phone (the camera captures at a locked 30 fps; frame-dropping keeps only the latest)."
- "On top of raw KLT we layer the forward-backward consistency check we just saw: track each point forward, then backward, and discard anything that doesn't return to where it started. That single filter is what turns noisy raw tracks — the red ones in this image — into the trustworthy green flow we actually use."
- "KLT alone has one failure mode: if tracking is lost — a sharp turn, a tunnel, sudden blur — the chain breaks. That's where ORB relocalization comes in. ORB extracts distinctive descriptors and re-anchors the system to a keyframe it has seen before. On one validation ride we recorded 49 such relocalization events, each recovering tracking cleanly."
- "So the division of labour is deliberate: KLT does the cheap, continuous frame-to-frame work, and ORB is the expensive recovery tool we only pay for when we genuinely need to relocalize."
- "Everything the green arrows represent — de-rotated, FB-verified flow over the road region — is the clean signal handed to the speed estimator and the Kalman filter."

## Why We Chose This Approach
- **Alternatives considered:** descriptor-matching every frame (ORB-to-ORB or ORB-SLAM-style feature matching on every image). We rejected per-frame descriptor matching as the *primary* tracker because it is far more expensive: it requires detecting, describing, and matching hundreds of features each frame, which does not fit the ~43 ms processed-frame interval with a 200 ms hard SDD budget on a Mali-G78 device.
- **Tradeoff accepted:** KLT assumes small inter-frame motion and brightness constancy, so it degrades under large jumps and blur and can lose tracks. We accept that and explicitly cover its failure mode with two safeguards: the forward-backward consistency filter (rejects bad tracks immediately) and ORB relocalization (recovers when the chain breaks).
- **Benefit gained:** KLT gives us continuous, dense, cheap frame-to-frame motion at the ~23 fps processed rate, while ORB is invoked only on loss — so we pay full descriptor cost only on relocalization events instead of ~23 times a second. This is the difference between a real-time phone app and an offline batch processor.
- **Engineering reasoning:** "cheap continuous tracker + expensive on-demand recovery" is the standard, proven VIO front-end pattern. It matches the per-frame budget while still being robust to the loss events that inevitably happen in real driving.

## Potential Questions (Defense)
**Q:** Why use KLT optical flow instead of matching ORB (or other) descriptors on every frame?
**A:** Cost and cadence. KLT tracks existing points with a pyramidal search and finishes a frame in a 15.4 ms median; matching descriptors on every frame means detecting and describing hundreds of features per image, which would not fit the ~43 ms processed-frame interval or the 200 ms SDD budget on the Exynos 2100 / Mali-G78. We instead reserve ORB for relocalization only — we recorded 49 ORB relocalization events on the 2026-06-04 ride, versus tracking running on every single frame.
**Follow-up Q:** What is the cost of KLT being wrong — doesn't optical flow drift?
**Follow-up A:** Raw KLT does produce bad tracks under blur and occlusion, which is exactly why we never use raw flow. The forward-backward consistency check discards any point that doesn't re-track back to its origin, and on top of that the IPM speed stage applies a statistical vote/zero-witness gate, so a few bad tracks cannot move the estimate. When tracking degrades wholesale, ORB relocalization re-anchors us.

**Q:** What happens when KLT loses tracking entirely — does the system just stop?
**A:** No. Two things happen. First, ORB relocalization re-anchors the visual state to a previously seen keyframe — we observed 49 such clean recovery events on the 2026-06-04 ride. Second, in the meantime the complementary inertial bridge keeps predicting forward speed from the IMU (trusted for up to ~6 seconds), and the map matcher keeps the ball constrained to the road graph. So a tracking loss degrades gracefully rather than freezing the display.
**Follow-up Q:** How do you know the relocalizations were genuine recoveries and not false matches?
**Follow-up A:** Each relocalization is gated before it is accepted, and the recorded behaviour on that ride shows tracking resuming cleanly after each of the 49 events. The same ride also logged 36 looming (essential-matrix-degenerate) fallbacks, where the optical-flow front-end fell back to the looming cue when the essential-matrix solve was degenerate — showing the front-end was actively handling hard frames rather than masking failures.

## Speaker Notes
- Numbers to have ready: 49 ORB relocalization events and 36 looming (essential-matrix-degenerate) fallbacks on the 2026-06-04 ride; 15.4 ms median / ~78 ms max tracking; ~23 fps / ~43 ms processed-frame interval (camera captures at a locked 30 fps / 33 ms; a worst-case ~78 ms frame is absorbed by frame-dropping); 200 ms SDD budget.
- Reference asset is `tests/sims/val_2026_06_03b/probe_cruise_0.jpg`: amber = ground-plane sampling mask, green arrows = FB-verified optical flow, red = raw production KLT. Use it to make the "raw vs verified" point concretely.
- Emphasise the architectural division: KLT = continuous + cheap; ORB = recovery + occasional. This framing pre-empts the "why not ORB-SLAM" question.
- Pitfall to avoid: do not claim the system runs full SLAM. The validated v1.0-osm build presents only what runs on roads: KLT optical flow with the forward-backward check and de-rotation, ORB relocalization, and the MSCKF covariance update. Keep the framing to the live pipeline.
- 1.5 minutes — lead with the screenshot, land the KLT-vs-descriptor decision, then the ORB recovery story.
