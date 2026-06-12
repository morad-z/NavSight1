#!/usr/bin/env python3
"""Per-ride speed medians + integrated distances (2026-06-12).

Compares, per sim JSON:
  - gp_flow_speed  (IPM — what the speedometer/ball use)
  - fused_speed    (legacy VIO trajectory speed; pre-2026-06-12b builds cap at 8 m/s)
  - GPS speed      (position-derived, jam-aware-clipped — UNRELIABLE under jamming)
and the matching distances: integral gp dt, VIO path length (vx/vz deltas), GPS path.

Usage: py -3.13 scripts/speed_medians_2026_06_12.py <ride.json> [...]
"""
import json
import math
import sys

import numpy as np

R = 6371000.0


def hav(a, b, c, d):
    p1, p2 = math.radians(a), math.radians(c)
    h = (math.sin((p2 - p1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(d - b) / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(h))


def main():
    for path in sys.argv[1:]:
        d = json.load(open(path))
        pts = d["points"]
        t = np.array([p["ts"] for p in pts], dtype=float) / 1000.0
        gp = np.array([p["gp_flow_speed"] if p["gp_flow_speed"] is not None else np.nan for p in pts])
        fu = np.array([p["fused_speed"] if p["fused_speed"] is not None else np.nan for p in pts])
        vx = np.array([p["vx"] for p in pts]); vz = np.array([p["vz"] for p in pts])
        dt = np.diff(t, prepend=t[0])
        dt[0] = 0.0
        dur = t[-1] - t[0]

        # distances
        d_gp = float(np.nansum(np.where(np.isfinite(gp), gp, 0.0) * dt))
        d_vio = float(np.sum(np.hypot(np.diff(vx), np.diff(vz))))
        # 2026-06-12c — DEDUP consecutive identical fixes FIRST: the sim resamples the same GPS fix
        # every ~43 ms tick, and advancing the "previous fix" on every duplicate skipped nearly all
        # real fix-to-fix segments (the bug that made GPS look frozen/short on the 06-12 rides).
        uniq = []
        for p in pts:
            if p.get("glat") is None or (p.get("gacc") or 99) > 25:
                continue
            if not uniq or p["glat"] != uniq[-1]["glat"] or p["glng"] != uniq[-1]["glng"]:
                uniq.append(p)
        d_gps = 0.0
        gps_v = []
        for i in range(1, len(uniq)):
            ddt = (uniq[i]["ts"] - uniq[i - 1]["ts"]) / 1000.0
            if not (0.2 <= ddt <= 10.0):
                continue
            seg = hav(uniq[i - 1]["glat"], uniq[i - 1]["glng"], uniq[i]["glat"], uniq[i]["glng"])
            if seg / ddt <= 25.0:                  # spike clip (90 km/h)
                d_gps += seg
                gps_v.append(seg / ddt)

        moving = np.isfinite(gp) & (gp > 0.28)      # >1 km/h
        med = lambda a: float(np.nanmedian(a)) * 3.6 if np.size(a) and np.any(np.isfinite(a)) else float("nan")
        print(f"\n{path.split('/')[-1]}  dur={dur:.0f}s")
        print(f"  median speed (moving): IPM/displayed={med(gp[moving]):5.1f} km/h"
              f"   VIO-fused={med(fu[moving]):5.1f} km/h"
              f"   GPS={np.median(gps_v)*3.6 if gps_v else float('nan'):5.1f} km/h (jam-suspect)")
        print(f"  distance: ∫IPM dt={d_gp:6.0f} m   VIO path={d_vio:6.0f} m   GPS path={d_gps:6.0f} m")


if __name__ == "__main__":
    main()
