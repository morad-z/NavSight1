#!/usr/bin/env python3
"""Score the read-only IPM ground-plane speed (gp_flow_speed) vs GPS, per sim recording.

The IPM speed estimator (Tracker::updateGroundFlowSpeed) is wired READ-ONLY — it never
feeds the dot. To decide whether it corrects the scooter speed under-read (0.18-0.62x GPS)
we record it per-sample next to GPS and the currently-displayed speed (fused_speed), then
compare here. Run this on a sim recorded with the IPM build, on a ride with a clean GPS fix.

Usage:
    python scripts/score_ground_flow.py <sim.json> [--gps-acc-max 15] [--min-gps-mps 0.8]

Reports, for BOTH gp_flow_speed (the IPM candidate) and fused_speed (what the user sees now):
  - coverage  : % of moving samples where the channel produced a value (IPM needs >=5 road inliers)
  - ratio     : median(channel / gps_speed)  -> 1.0 = perfect, <1 = under-read, >1 = over-read
  - MAE       : mean abs error in km/h
so you can see at a glance whether IPM beats the current speed and by how much.
"""
import json
import math
import sys
import argparse

R_EARTH_M = 6371000.0


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))


def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return float("nan")
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim", help="sim recording .json")
    ap.add_argument("--gps-acc-max", type=float, default=15.0,
                    help="reject GPS samples worse than this accuracy (m)")
    ap.add_argument("--min-gps-mps", type=float, default=0.8,
                    help="only score samples where GPS speed exceeds this (skip stops)")
    args = ap.parse_args()

    with open(args.sim, encoding="utf-8") as f:
        data = json.load(f)
    pts = data.get("points", [])
    if not pts:
        print("no points in", args.sim)
        return

    # Per-sample GPS speed from consecutive fixes (m/s), gated on accuracy + dt.
    rows = []   # (gps_mps, gp_flow_mps_or_None, fused_mps_or_None)
    prev = None
    for p in pts:
        glat, glng = p.get("glat"), p.get("glng")
        gacc = p.get("gacc")
        ts = p.get("ts")
        ok = (glat is not None and glng is not None and ts is not None
              and (gacc is None or gacc <= args.gps_acc_max))
        if ok and prev is not None:
            dt = (ts - prev[2]) / 1000.0
            if 0.05 < dt < 3.0:
                d = haversine_m(prev[0], prev[1], glat, glng)
                gps_mps = d / dt
                rows.append((gps_mps, p.get("gp_flow_speed"), p.get("fused_speed")))
        if ok:
            prev = (glat, glng, ts)

    moving = [r for r in rows if r[0] >= args.min_gps_mps]
    print(f"\n=== {args.sim} ===")
    print(f"samples: {len(pts)}  scored-with-GPS: {len(rows)}  moving(>={args.min_gps_mps} m/s): {len(moving)}")
    if not moving:
        print("no moving samples with a clean GPS fix — can't score (GPS jammed? indoor? too short?)")
        return

    gps_kmh = [r[0] * 3.6 for r in moving]
    print(f"GPS speed over moving samples: median {median(gps_kmh):.1f} km/h  "
          f"[{min(gps_kmh):.1f}..{max(gps_kmh):.1f}]")

    for name, idx in (("gp_flow (IPM, candidate)", 1), ("fused   (displayed now)", 2)):
        fired = [r for r in moving if r[idx] is not None and r[idx] >= 0]
        cov = 100.0 * len(fired) / len(moving)
        if not fired:
            print(f"  {name}: coverage 0% — never produced a value on this ride")
            continue
        ratios = [r[idx] / r[0] for r in fired if r[0] > 1e-6]
        errs_kmh = [abs(r[idx] - r[0]) * 3.6 for r in fired]
        print(f"  {name}: coverage {cov:4.0f}%  "
              f"ratio(med) {median(ratios):.2f}x  MAE {sum(errs_kmh)/len(errs_kmh):.1f} km/h  "
              f"med {median([r[idx]*3.6 for r in fired]):.1f} km/h")
    print("\nratio 1.0 = perfect; <1 under-read, >1 over-read. IPM 'wins' if its ratio is closer "
          "to 1.0 than fused AND its coverage is high on the road.\n")


if __name__ == "__main__":
    main()
