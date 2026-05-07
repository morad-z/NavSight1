---
name: NavSight GPS usage model — bootstrap only, never EKF runtime
description: GPS feeds the startup world anchor and scale initializer once; after bootstrap the EKF is pure VIO. Future navigation will need lat/lng destination resolution.
type: reference
originSessionId: 79610daf-47c4-4b65-8f87-6ada8b7fecc8
---
NavSight uses GPS in exactly **two** places, both at session startup:

1. **World-frame anchor**: GPS lat/lng at session start is recorded as the origin of the local navigation frame. All VIO positions are relative to this anchor. The conversion VIO → real-world coordinates uses (`startup_lat`, `startup_lng`) plus haversine.

2. **Scale initializer**: GPS feeds an initial estimate to the scale fuser at startup. After enough VIO observations accumulate the fuser converges on a VIO-derived scale; the GPS seed exists only to give the fuser something better than zero on frame 1.

After startup, **the EKF reads no GPS samples**. ADR-004's "no GPS in hot path" rule covers everything past frame ~N where N is the bootstrap window. Position drift, heading drift, and all per-frame state evolution are pure VIO + IMU.

**Consequence for map matching:** the matcher CAN consume VIO position (projected through the startup anchor into lat/lng-space) without violating the VIO-only design. The matcher's input stream is `(timestamp, vio_lat, vio_lng, vio_yaw)` derived from `(startup_anchor + EKF.position, EKF.yaw)`, never raw GPS.

**Consequence for the emission-probability model:** standard map matching (Newson & Krumm) assumes Gaussian noise on perpendicular GPS distance. VIO has different noise: tight short-term local consistency (~1 m over 30 s) but unbounded long-term drift (10-100 m over many minutes). A VIO-driven matcher needs a different emission model — likely a *trajectory-shape* match over a sliding window rather than a per-sample radial Gaussian.

**Future feature (per Morad 2026-05-07):** navigation to a specific destination. This will require:
- Lat/lng destination input (search, address, or click on map)
- Conversion to local frame relative to startup anchor
- Path planning over OSM road network (separate from map matching)
- Turn-by-turn cues based on EKF position vs route geometry

The destination resolution and routing are downstream of map matching but use the same OSM road network data. Plan for them together.
