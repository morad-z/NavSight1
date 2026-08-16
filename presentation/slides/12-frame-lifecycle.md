# Slide 12: Frame Lifecycle — The Journey of One Camera Frame

**Section:** Runtime Architecture · **Slide:** 12 of 23 · **Estimated Time:** 1.5 minutes

## On-Slide Content
- One frame, six stages, ~15 ms: **Capture → Lens-Correct → KLT Track → Forward-Backward Check → De-Rotate Flow → Feed IPM + EKF → Publish.**
- **Capture:** rear camera, 640×480 working resolution, zero-copy `ByteBuffer` into the native core; camera capture is locked at 30 fps / 33 ms, and after `KEEP_ONLY_LATEST` frame-dropping the effective processed VIO rate is ~23 fps / ~43 ms.
- **Lens-correct:** raw pixels → undistorted normalized coordinates (x = X/Z, y = Y/Z); intrinsics fx ≈ 451.
- **KLT track:** pyramidal Lucas-Kanade tracks corner points frame→frame for raw flow.
- **Forward-backward check:** track forward then back; keep a point only if it returns near its origin (rejects blur/occlusion outliers).
- **De-rotate flow:** subtract the gyro/Madgwick rotational component so residual flow is translation-only.
- **Feed + Publish:** clean flow → IPM speed + EKF update → publish the `VioData` return struct to the UI.
- **Budget:** 15.4 ms median, ~78 ms max vs the 200 ms SDD budget — 13× under at median, 2.5× margin at worst case (a ~78 ms frame exceeds the inter-frame interval and is absorbed by `KEEP_ONLY_LATEST` frame-dropping).
- [Diagram: diagrams/02-frame-lifecycle.md]

## Talking Points (what the presenter SAYS)
- "Every camera frame takes the same six-step journey, and the whole thing finishes in about 15 milliseconds — so let me walk you through one frame."
- "It arrives at 640×480 and is handed to the native C++ core as a zero-copy buffer — we don't copy the pixels across the JNI boundary, which matters at 23 frames a second."
- "First we undistort it into normalized camera coordinates, so every measurement downstream is in clean geometry with no lens warp."
- "Then pyramidal KLT optical flow tracks the same corner points from the previous frame to this one. But raw tracks lie — blur and occlusion produce confident-looking garbage — so we run a forward-backward consistency check: track each point forward, then track it back, and keep it only if it lands where it started."
- "The surviving flow still contains the device's own rotation, so we subtract the rotational component using the gyro-driven attitude. What's left is pure translational flow — exactly what the speed estimator and the Kalman filter need."
- "That clean flow feeds two consumers at once: the ground-plane speed estimator and the EKF measurement update. The result is packaged into the VioData return struct and published to the UI — and the next frame is already arriving."

## Why We Chose This Approach
- **Alternatives considered:** processing at full sensor resolution and copying frames across JNI. We chose a fixed 640×480 working resolution and a zero-copy `ByteBuffer` because the per-frame budget is dominated by tracking, not capture — shrinking resolution and avoiding the copy buys headroom directly.
- **Tradeoff accepted:** the forward-backward consistency check costs an extra backward track per point, roughly doubling KLT work. We accept that cost because unfiltered tracks are the single biggest source of phantom speed — verified flow (the green arrows in our camera figure) is dramatically cleaner than raw production KLT (red).
- **Benefit gained:** de-rotating the flow before it reaches the estimators means the speed and EKF stages never have to disentangle rotation from translation — each stage sees only the signal it is designed for, which is both faster and more numerically stable.
- **Engineering reasoning:** the lifecycle is deliberately ordered so that every cleaning step (undistort, FB-check, de-rotate) happens before the expensive fusion math, so the EKF only ever ingests trusted, translation-only measurements.

## Potential Questions (Defense)
**Q:** Why is the forward-backward consistency check necessary — doesn't KLT already report a status flag?
**A:** KLT's own status flag is not enough: on a moving scooter, motion blur and parallel lane lines produce points that KLT marks as "tracked" but whose flow is meaningless. The forward-backward check re-tracks each point in reverse and discards it unless it returns near its origin within a pixel threshold. This is what lets us trust the flow — in our camera frame figure the green arrows are the FB-verified flow that survives, versus the raw red KLT.
**Follow-up Q:** Doesn't running KLT twice blow the frame budget?
**Follow-up A:** No — even with the backward track the median frame is 15.4 ms, far under the 200 ms SDD budget (13× margin). The worst case is ~78 ms; that does exceed the ~43 ms processed inter-frame interval, but `KEEP_ONLY_LATEST` frame-dropping absorbs it, and even ~78 ms is still 2.5× under the 200 ms SDD budget.

**Q:** Why de-rotate the optical flow instead of letting the EKF handle rotation?
**A:** Because the IPM ground-plane speed estimator needs pure translational flow to solve its least-squares speed — any residual rotation would bias the speed. We already have a high-rate gyro/Madgwick attitude estimate, so subtracting the rotational flow component is cheap and removes the coupling before either consumer sees the frame. The EKF still maintains its own attitude state; de-rotation just keeps the visual measurement clean.
**Follow-up Q:** What if the gyro estimate is wrong — won't de-rotation introduce error?
**Follow-up A:** Gyro attitude is very accurate over a single 43 ms frame interval — bias drift is negligible at that timescale — so per-frame de-rotation is reliable. Longer-term gyro bias is separately estimated and corrected inside the EKF's 15-DOF error state.

## Speaker Notes
- Numbers to have ready: 640×480 working resolution; fx ≈ 451; 30 fps / 33 ms camera capture, ~23 fps / ~43 ms effective processed VIO rate after frame-dropping; 15.4 ms median, ~78 ms max, 200 ms SDD budget.
- The reference asset for "verified vs raw flow" is `tests/sims/val_2026_06_03b/probe_cruise_0.jpg` — amber is the ground-plane sampling mask, green arrows are FB-verified flow, red is production KLT. Mention it if asked to show evidence.
- Emphasise the ordering principle: clean first (undistort → FB-check → de-rotate), then fuse. This is why the EKF stays consistent.
- Pitfall to avoid: do not claim KLT runs on the GPU — it runs on CPU in OpenCV.
- Keep this tightly choreographed to the diagram; it is the visual heart of the deck. 1.5 minutes — one clean pass through the six stages.
