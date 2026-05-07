---
name: NavSight is VIO-only — never fuse GPS into the EKF
description: GPS recorded in sims is for offline analysis only; runtime EKF must not consume GPS for position or yaw
type: feedback
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
NavSight is a VIO navigation app. The EKF must not fuse GPS at runtime, **including for yaw**. ADR-004 forbids GPS in the hot path; that prohibition is total, not just for position.

**Why:** NavSight's product premise is "works without GPS" (indoor navigation, GPS-denied environments, jamming defence per ADR-004). Adding any GPS dependency to the EKF — even as a derived course-over-ground yaw observation — breaks that premise and would degrade behaviour wherever GPS is unavailable, jammed, or noisy. Morad's correction on 2026-05-07 was explicit: "thats not allowed in the app since its a vio app."

**How to apply:**
- GPS *recorded* in sim JSON files (`glat`, `glng`, `gacc`) is fine — it's a forensic tool for offline analysis (see `scripts/compare_gps_vio.py` ground-truth comparisons).
- GPS *at session startup* is allowed: as the world-frame anchor (lat/lng of local-frame origin) and as the scale fuser's initial seed. After bootstrap completes, no further GPS samples enter the EKF or any subsequent estimator. See `reference_gps_usage_model.md`.
- GPS *fused into the EKF after startup* is **never** acceptable. Do not draft another `updateGpsCourseYaw`, `updateGpsPosition`, or any sibling method that runs in the per-frame hot path.
- Absolute heading anchors must come from non-GPS sources: revisited continuous magnetometer (with distortion modelling), better visual loop closure, or OSM map-matching consuming VIO-derived position (projected through the startup anchor) — not raw GPS.

**Consequence:** ADR-017 (GPS course as bounded yaw) was drafted on 2026-05-07 and withdrawn the same day. Do not revive without explicit reconsideration of the VIO-only product principle.
