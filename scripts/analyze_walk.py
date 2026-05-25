#!/usr/bin/env python3
"""analyze_walk.py - summarize a NavSight sim JSON.

Reports heading-over-time, trajectory loop-closure gap, per-loop bearing
drift, and key event_summary counters. Reusable across walks (do not
rewrite inline per navsight feedback_reuse_scripts).

Usage:
    python scripts/analyze_walk.py <sim.json>

JSON schema (per navsight-sim-debugging skill):
    points[].vx/vy/vz : Tracker output, Y-up (vy = UP). Ground plane = (vx, vz).
    points[].hdg      : displayed heading estimate (deg, nav-CW from north).
    points[].vyaw     : Tracker yaw (rad or deg depending on build).
    event_summary     : EventCounters dump at recording stop.
"""
import json
import sys
import math


def wrap180(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def fmt_num(v):
    return f"{v}" if v is not None else "?"


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_walk.py <sim.json>")
        sys.exit(1)
    path = sys.argv[1]
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        d = json.load(fh)

    pts = d.get("points", [])
    es = d.get("event_summary", {}) or {}
    n = len(pts)
    print(f"=== {path} ===")
    if n == 0:
        print("NO POINTS")
        return

    t0 = pts[0].get("ts", 0)
    t1 = pts[-1].get("ts", 0)
    dur = (t1 - t0) / 1000.0
    print(f"points={n}  duration={dur:.1f}s  total_path_dm={fmt_num(es.get('total_path_dm'))}")

    # ---- heading over time ----
    print("\n-- heading hdg(deg) / vyaw over time --")
    have_hdg = any(p.get("hdg") is not None for p in pts)
    for frac in (0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0):
        i = min(n - 1, int(frac * (n - 1)))
        p = pts[i]
        ts = (p.get("ts", t0) - t0) / 1000.0
        print(f"  t={ts:6.1f}s  hdg={fmt_num(p.get('hdg')):>8}  vyaw={fmt_num(p.get('vyaw')):>8}")
    if have_hdg:
        hdgs = [p.get("hdg") for p in pts if p.get("hdg") is not None]
        net = 0.0
        wraps = 0.0
        for k in range(1, len(hdgs)):
            step = wrap180(hdgs[k] - hdgs[k - 1])
            net += step
            wraps += abs(step)
        print(f"  net wrapped heading change: {net:+.1f} deg   total |turning|: {wraps:.0f} deg")

    # ---- trajectory (ground plane vx,vz) ----
    print("\n-- trajectory (ground plane vx,vz in m) --")
    def gp(p):
        return (p.get("vx", 0.0), p.get("vz", 0.0))
    x0, z0 = gp(pts[0])
    xe, ze = gp(pts[-1])
    gap = math.hypot(xe - x0, ze - z0)
    maxr = max(math.hypot(gp(p)[0] - x0, gp(p)[1] - z0) for p in pts)
    print(f"  start=({x0:.2f},{z0:.2f})  end=({xe:.2f},{ze:.2f})")
    print(f"  loop-closure gap (end vs start): {gap:.2f} m")
    print(f"  max excursion from start: {maxr:.2f} m")

    # ---- jump clusters (>0.4m consecutive) = LC correction ramps ----
    jumps = 0
    maxjump = 0.0
    for i in range(1, n):
        a, b = pts[i - 1], pts[i]
        dd = math.sqrt(sum((b.get(k, 0.0) - a.get(k, 0.0)) ** 2 for k in ("vx", "vy", "vz")))
        if dd > 0.4:
            jumps += 1
            maxjump = max(maxjump, dd)
    print(f"  frames with >0.4m jump (LC ramp/teleport): {jumps}  max single jump: {maxjump:.2f} m")

    # ---- coarse path trace: per-leg position + heading ----
    # Lets us see whether heading reverses ~180 on each turn (then a non-
    # retracing path is a translation bug) or stays flat (heading not reversing).
    print("\n-- path trace (t, x=vx, z=vz, hdg_deg, vyaw_deg) --")
    step = max(1, n // 28)
    for i in range(0, n, step):
        p = pts[i]
        ts = (p.get("ts", t0) - t0) / 1000.0
        hd = p.get("hdg")
        vy = p.get("vyaw")
        hd_d = math.degrees(hd) if hd is not None else float("nan")
        vy_d = math.degrees(vy) if vy is not None else float("nan")
        print(f"  t={ts:5.1f}s  x={p.get('vx', 0.0):7.2f}  z={p.get('vz', 0.0):7.2f}"
              f"  hdg={hd_d:7.1f}  vyaw={vy_d:7.1f}")

    # ---- key counters ----
    print("\n-- key event_summary counters --")
    keys = [
        "landmarks_observed_total", "landmarks_matched_total",
        "landmarks_descriptor_refreshed_total", "landmarks_added_total",
        "landmark_map_size", "slam_promotions_total",
        "loop_closure_attempts", "loop_closure_accepts",
        "loop_closure_geom_attempts", "loop_closure_geom_accepts",
        "loop_closure_corrections_applied", "loop_closure_chi2_rejected",
    ]
    for k in keys:
        if k in es:
            print(f"  {k} = {es[k]}")


if __name__ == "__main__":
    main()
