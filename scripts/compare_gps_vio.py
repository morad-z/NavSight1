#!/usr/bin/env python3
"""
Compare VIO trajectory to GPS trajectory in a common local-ENU frame.

GPS is shown as reference (it can be noisy / jammed in Haifa, but on a long
walk along a road it should be the closest thing to ground truth we have).

Outputs:
  - Per-window divergence (VIO-from-GPS distance over time)
  - The moment VIO first diverges by >5 m, >10 m, >20 m
  - Best-fit yaw rotation between VIO XZ and GPS ENU (Procrustes)
  - VIO scale ratio vs GPS path
  - PNG plot if matplotlib is available
"""

import argparse
import json
import math
import os
import sys


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def lla_to_enu(lat, lon, lat0, lon0):
    """Local east-north (m) anchored at (lat0, lon0). Flat-earth approx OK at < 5km."""
    R = 6371000.0
    e = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * R
    return e, n


def best_fit_yaw(vio_xy, gps_en):
    """Find yaw R such that R * vio_xy ≈ gps_en (Procrustes, no scale).
       Returns (yaw_rad, residual_rms_m)."""
    assert len(vio_xy) == len(gps_en)
    n = len(vio_xy)
    if n < 3:
        return 0.0, float("inf")
    # Centre both
    cv_x = sum(p[0] for p in vio_xy) / n
    cv_y = sum(p[1] for p in vio_xy) / n
    cg_e = sum(p[0] for p in gps_en) / n
    cg_n = sum(p[1] for p in gps_en) / n
    # Cross-correlation (no scale)
    sxx = sum((p[0] - cv_x) * (q[0] - cg_e) for p, q in zip(vio_xy, gps_en))
    sxy = sum((p[0] - cv_x) * (q[1] - cg_n) for p, q in zip(vio_xy, gps_en))
    syx = sum((p[1] - cv_y) * (q[0] - cg_e) for p, q in zip(vio_xy, gps_en))
    syy = sum((p[1] - cv_y) * (q[1] - cg_n) for p, q in zip(vio_xy, gps_en))
    # Optimal rotation angle
    yaw = math.atan2(sxy - syx, sxx + syy)
    # Apply
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    rss = 0.0
    for (vx, vy), (ge, gn) in zip(vio_xy, gps_en):
        rx = cos_y * (vx - cv_x) - sin_y * (vy - cv_y) + cg_e
        ry = sin_y * (vx - cv_x) + cos_y * (vy - cv_y) + cg_n
        rss += (rx - ge) ** 2 + (ry - gn) ** 2
    return yaw, math.sqrt(rss / n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim", help="simulation_data_*.json")
    ap.add_argument("--plot", action="store_true",
                    help="save PNG (requires matplotlib)")
    args = ap.parse_args()

    with open(args.sim, "r", encoding="utf-8") as fp:
        d = json.load(fp)
    pts = d["points"]
    n = len(pts)

    # Build paired (VIO XZ, GPS EN) at every sample where both exist
    gps_pts = [p for p in pts if p.get("glat") is not None and p.get("glng") is not None]
    if len(gps_pts) < 10:
        print("Not enough GPS samples for a useful comparison.")
        return

    lat0 = gps_pts[0]["glat"]
    lon0 = gps_pts[0]["glng"]
    t0 = pts[0]["ts"]

    paired = []  # (t_s, vio_x, vio_z, gps_e, gps_n, gps_acc)
    for p in pts:
        if p.get("glat") is None or p.get("glng") is None:
            continue
        t_s = (p["ts"] - t0) / 1000.0
        ge, gn = lla_to_enu(p["glat"], p["glng"], lat0, lon0)
        # VIO frame: X horizontal, Y up, Z horizontal. Use (X, Z) as 2-D.
        # Anchor VIO at (vx[0], vz[0]) = first paired sample.
        paired.append((t_s, p["vx"], p["vz"], ge, gn, p.get("gacc")))

    # Anchor VIO so the first paired sample sits at origin (matches GPS anchor).
    vx0, vz0 = paired[0][1], paired[0][2]
    vio_xy = [(t[1] - vx0, t[2] - vz0) for t in paired]
    gps_en = [(t[3], t[4]) for t in paired]
    times  = [t[0] for t in paired]
    accs   = [t[5] for t in paired]

    # Best-fit yaw to align VIO frame with GPS frame
    yaw, fit_rms = best_fit_yaw(vio_xy, gps_en)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    vio_aligned = [
        (cos_y * vx - sin_y * vz, sin_y * vx + cos_y * vz)
        for (vx, vz) in vio_xy
    ]

    # Distance of VIO-aligned to GPS at each sample
    dists = [
        math.sqrt((vx - ge) ** 2 + (vy - gn) ** 2)
        for (vx, vy), (ge, gn) in zip(vio_aligned, gps_en)
    ]

    # Cumulative path lengths
    vio_path = 0.0
    gps_path = 0.0
    for i in range(1, len(paired)):
        v0, v1 = vio_aligned[i - 1], vio_aligned[i]
        g0, g1 = gps_en[i - 1], gps_en[i]
        vio_path += math.sqrt((v1[0] - v0[0]) ** 2 + (v1[1] - v0[1]) ** 2)
        gps_path += math.sqrt((g1[0] - g0[0]) ** 2 + (g1[1] - g0[1]) ** 2)

    # First-divergence times
    def first_exceed(threshold):
        for t, dv in zip(times, dists):
            if dv > threshold:
                return t, dv
        return None, None

    t5, d5    = first_exceed(5.0)
    t10, d10  = first_exceed(10.0)
    t20, d20  = first_exceed(20.0)
    t50, d50  = first_exceed(50.0)

    # Endpoint analysis
    end_dist = dists[-1]
    end_gps = gps_en[-1]
    end_vio = vio_aligned[-1]

    # Stats
    avg_acc = sum(a for a in accs if a is not None) / max(1, sum(1 for a in accs if a is not None))
    max_dist = max(dists)
    t_max = times[dists.index(max_dist)]

    print("=" * 72)
    print("GPS vs VIO trajectory comparison")
    print(f"  file     : {os.path.basename(args.sim)}")
    print(f"  duration : {times[-1]:.1f} s   ({len(paired)} paired samples)")
    print("=" * 72)
    print()
    print("ALIGNMENT (best-fit yaw, no scale, both anchored at first GPS sample):")
    print(f"  yaw rotation needed to align VIO→GPS : {math.degrees(yaw):+.1f}°")
    print(f"  Procrustes residual RMS              : {fit_rms:.2f} m")
    print(f"  → If residual is small the trajectories agree in shape; large residual")
    print(f"    means VIO drifted from GPS over the walk.")
    print()
    print("PATH LENGTHS (XZ-plane only):")
    print(f"  GPS path     : {gps_path:.1f} m")
    print(f"  VIO path     : {vio_path:.1f} m")
    if gps_path > 0:
        print(f"  VIO/GPS      : {vio_path / gps_path:.3f}x   "
              f"({'VIO undershoots' if vio_path < gps_path else 'VIO overshoots'})")

    # Map-matched track (§8M Step F output, logged per sample as mm_lat/mm_lng once map matching is
    # implemented). It's already in the same geographic frame as GPS, so NO yaw-alignment is needed —
    # convert straight to ENU. Optional: only reported/plotted when the field is present.
    mm_en = [lla_to_enu(p["mm_lat"], p["mm_lng"], lat0, lon0)
             for p in pts if p.get("mm_lat") is not None and p.get("mm_lng") is not None]
    mm_path = sum(math.hypot(mm_en[i][0] - mm_en[i - 1][0], mm_en[i][1] - mm_en[i - 1][1])
                  for i in range(1, len(mm_en)))
    if mm_en:
        print(f"  map-matched  : {mm_path:.1f} m   ({len(mm_en)} pts)"
              + (f"   {mm_path / gps_path:.3f}x" if gps_path > 0 else ""))
    print()

    # OUT-AND-BACK / 1 km GOAL: 500 m forward + back = ~1 km, target ≤5% drift. On an out-and-back the
    # cleanest drift metric is "endpoint return" — how far the end point lands from the start (ideally 0).
    # 5% of a 1 km ride = 50 m. Denominator = GPS path (the measured truth).
    def _endpoint_return(track):
        return None if len(track) < 2 else math.hypot(track[-1][0] - track[0][0],
                                                       track[-1][1] - track[0][1])
    if gps_path > 0:
        print("OUT-AND-BACK / 1 km GOAL (target ≤5% drift = ≤50 m endpoint return on a 1 km ride):")
        for name, track in (("GPS", gps_en), ("VIO", vio_aligned), ("map-matched", mm_en)):
            er = _endpoint_return(track)
            if er is None:
                continue
            pct = 100 * er / gps_path
            flag = "PASS" if pct <= 5.0 else "OVER"
            print(f"  {name:<12}: end returns {er:6.1f} m from start  "
                  f"({pct:4.1f}% of {gps_path:.0f} m)  [{flag} vs 5%]")
        print()

    print("DIVERGENCE (VIO-aligned vs GPS, at each paired sample):")
    print(f"  mean dist    : {sum(dists) / len(dists):.2f} m")
    print(f"  max dist     : {max_dist:.2f} m   at t = {t_max:.1f} s")
    print(f"  end dist     : {end_dist:.2f} m   "
          f"(VIO end {end_vio[0]:+.1f}, {end_vio[1]:+.1f}; "
          f"GPS end {end_gps[0]:+.1f}, {end_gps[1]:+.1f})")
    print()
    print("FIRST-DIVERGENCE TIMES:")
    for thr, t, dv in [(5.0, t5, d5), (10.0, t10, d10),
                       (20.0, t20, d20), (50.0, t50, d50)]:
        if t is None:
            print(f"  > {thr:>4.0f} m   : never reached")
        else:
            print(f"  > {thr:>4.0f} m   : at t = {t:>6.1f} s  (dist = {dv:.1f} m)")
    print()
    print("GPS QUALITY (caveat — GPS can be jammed in Haifa):")
    print(f"  mean accuracy reported by phone: {avg_acc:.1f} m")
    print()

    # Drift accumulation slope (m of error per m of GPS path)
    if gps_path > 0:
        # Sample drift at quarter, half, three-quarter, and end
        quarters = [0.25, 0.5, 0.75, 1.0]
        cum_gps = [0.0]
        for i in range(1, len(paired)):
            g0, g1 = gps_en[i - 1], gps_en[i]
            cum_gps.append(cum_gps[-1] + math.sqrt(
                (g1[0] - g0[0]) ** 2 + (g1[1] - g0[1]) ** 2))
        print("DRIFT ACCUMULATION (VIO error vs GPS distance walked):")
        for q in quarters:
            target = q * gps_path
            for i, c in enumerate(cum_gps):
                if c >= target:
                    print(f"  at {q*100:>3.0f}% of walk ({target:>5.1f} m walked, "
                          f"t={times[i]:>6.1f} s): drift = {dists[i]:>5.1f} m  "
                          f"({100 * dists[i] / max(target, 1):.1f}% of distance)")
                    break
        print()

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed; skipping plot", file=sys.stderr)
            return
        fig, axes = plt.subplots(1, 2, figsize=(14, 7))
        axes[0].plot([p[0] for p in gps_en], [p[1] for p in gps_en],
                     "-g", linewidth=1.5, label="GPS (E,N)")
        axes[0].plot([p[0] for p in vio_aligned], [p[1] for p in vio_aligned],
                     "-b", linewidth=1.5, label=f"VIO aligned (yaw={math.degrees(yaw):+.1f}°)")
        axes[0].plot(0, 0, "go", markersize=10, label="start")
        axes[0].plot(end_gps[0], end_gps[1], "g^", markersize=12, label="GPS end")
        axes[0].plot(end_vio[0], end_vio[1], "b^", markersize=12, label="VIO end")
        if mm_en:
            axes[0].plot([p[0] for p in mm_en], [p[1] for p in mm_en],
                         "-m", linewidth=1.5, label=f"map-matched ({mm_path:.0f} m)")
            axes[0].plot(mm_en[-1][0], mm_en[-1][1], "m^", markersize=12, label="map-matched end")
        axes[0].set_aspect("equal")
        axes[0].set_xlabel("East (m)")
        axes[0].set_ylabel("North (m)")
        axes[0].set_title(f"GPS vs VIO — {os.path.basename(args.sim)}")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(times, dists, "-r", linewidth=1.2, label="VIO-from-GPS distance")
        axes[1].axhline(5, color="orange", linestyle="--", linewidth=0.8, label="5 m")
        axes[1].axhline(20, color="red", linestyle="--", linewidth=0.8, label="20 m")
        axes[1].set_xlabel("time (s)")
        axes[1].set_ylabel("VIO-vs-GPS distance (m)")
        axes[1].set_title("Drift accumulation over time")
        axes[1].legend()
        axes[1].grid(True)

        out = os.path.splitext(args.sim)[0] + "_gps_vio.png"
        fig.tight_layout()
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(f"plot saved: {out}")


if __name__ == "__main__":
    main()
