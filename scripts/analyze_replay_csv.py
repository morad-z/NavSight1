#!/usr/bin/env python3
"""Score a replay_harness CSV (current engine on real frames) vs recorded GPS.

Unlike compare_gps_vio.py (which reads the OLD recording's vx/vz from the sim
JSON), this reads the replay CSV that the harness emits when run on real frames,
so it scores the CURRENT engine. Three trajectories are compared to GPS:
  - dot   : traj_x/traj_z  (Tracker::global_t_ — the user-facing map dot, driven
            by the depth-flow speed work)
  - ekf   : ekf_x/ekf_z    (EKF p_G_ dead-reckoning — a different trajectory)
GPS fixes are held constant between ~1 Hz updates in the recording, so we
subsample to unique fixes for a clean pairing. Reuses the Procrustes / ENU
helpers from compare_gps_vio.py per the reuse-scripts rule.
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_gps_vio import haversine_m, lla_to_enu, best_fit_yaw  # noqa: E402


def fnum(x):
    try:
        v = float(x)
        return v if v == v else None  # reject NaN
    except (TypeError, ValueError):
        return None


def best_fit_yaw_scale(vio_xy, gps_en):
    """Procrustes WITH uniform scale: find s, yaw minimizing |s*R*v - g|.
    Returns (scale, yaw_rad, rms_m)."""
    n = len(vio_xy)
    if n < 3:
        return 1.0, 0.0, float("inf")
    cvx = sum(p[0] for p in vio_xy) / n
    cvy = sum(p[1] for p in vio_xy) / n
    cge = sum(p[0] for p in gps_en) / n
    cgn = sum(p[1] for p in gps_en) / n
    sxx = sum((p[0] - cvx) * (q[0] - cge) for p, q in zip(vio_xy, gps_en))
    sxy = sum((p[0] - cvx) * (q[1] - cgn) for p, q in zip(vio_xy, gps_en))
    syx = sum((p[1] - cvy) * (q[0] - cge) for p, q in zip(vio_xy, gps_en))
    syy = sum((p[1] - cvy) * (q[1] - cgn) for p, q in zip(vio_xy, gps_en))
    yaw = math.atan2(sxy - syx, sxx + syy)
    var_v = sum((p[0] - cvx) ** 2 + (p[1] - cvy) ** 2 for p in vio_xy)
    num = math.cos(yaw) * (sxx + syy) + math.sin(yaw) * (sxy - syx)
    scale = num / var_v if var_v > 1e-9 else 1.0
    cy, sy = math.cos(yaw), math.sin(yaw)
    rss = 0.0
    for (vx, vy), (ge, gn) in zip(vio_xy, gps_en):
        rx = scale * (cy * (vx - cvx) - sy * (vy - cvy)) + cge
        ry = scale * (sy * (vx - cvx) + cy * (vy - cvy)) + cgn
        rss += (rx - ge) ** 2 + (ry - gn) ** 2
    return scale, yaw, math.sqrt(rss / n)


def path_len(track):
    return sum(math.hypot(track[i][0] - track[i - 1][0],
                          track[i][1] - track[i - 1][1])
               for i in range(1, len(track)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="replay_harness output CSV")
    ap.add_argument("--gacc-max", type=float, default=15.0,
                    help="drop GPS fixes with accuracy worse than this (m)")
    ap.add_argument("--plot", metavar="PNG", default=None,
                    help="save a GPS-vs-dot-vs-EKF overlay PNG")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    if not rows:
        print("empty CSV")
        return

    # Forward-fill dot/ekf so every row has a position (global_t_ holds between
    # visual frames; EKF p_G updates every row once initialised).
    last_dot = last_ekf = None
    recs = []  # (ts_s, dot_xz, ekf_xz, glat, glng, gacc, speed)
    t0 = fnum(rows[0]["ts_ns"])
    for r in rows:
        ts = fnum(r["ts_ns"])
        tx, tz = fnum(r["traj_x"]), fnum(r["traj_z"])
        if tx is not None and tz is not None:
            last_dot = (tx, tz)
        ex, ez = fnum(r["ekf_x"]), fnum(r["ekf_z"])
        if ex is not None and ez is not None:
            last_ekf = (ex, ez)
        glat, glng, gacc = fnum(r["glat"]), fnum(r["glng"]), fnum(r["gacc_m"])
        spd = fnum(r["fused_speed_mps"])
        recs.append(((ts - t0) / 1e9, last_dot, last_ekf,
                     glat, glng, gacc, spd))

    # Unique GPS fixes (recording holds last fix between ~1 Hz updates).
    gps_fixes = []  # (ts_s, glat, glng, gacc, dot, ekf)
    prev_ll = None
    for ts, dot, ekf, glat, glng, gacc, _ in recs:
        if glat is None or glng is None:
            continue
        if args.gacc_max and gacc is not None and gacc > args.gacc_max:
            continue
        if prev_ll == (glat, glng):
            continue
        prev_ll = (glat, glng)
        if dot is None or ekf is None:
            continue
        gps_fixes.append((ts, glat, glng, gacc, dot, ekf))

    if len(gps_fixes) < 5:
        print(f"only {len(gps_fixes)} usable GPS fixes (gacc<={args.gacc_max}) — "
              f"loosen --gacc-max")
        return

    lat0, lon0 = gps_fixes[0][1], gps_fixes[0][2]
    gps_en = [lla_to_enu(g[1], g[2], lat0, lon0) for g in gps_fixes]
    dot_xz = [(g[4][0], g[4][1]) for g in gps_fixes]
    ekf_xz = [(g[5][0], g[5][1]) for g in gps_fixes]
    # anchor each track at its first paired sample
    dot0, ekf0 = dot_xz[0], ekf_xz[0]
    dot_xz = [(p[0] - dot0[0], p[1] - dot0[1]) for p in dot_xz]
    ekf_xz = [(p[0] - ekf0[0], p[1] - ekf0[1]) for p in ekf_xz]

    gps_path = path_len(gps_en)
    dur = gps_fixes[-1][0] - gps_fixes[0][0]

    print("=" * 74)
    print(f"REPLAY (current engine on real frames) vs GPS — {os.path.basename(args.csv)}")
    print(f"  duration {dur:.0f} s   paired GPS fixes (gacc<={args.gacc_max}m): "
          f"{len(gps_fixes)}   GPS path {gps_path:.1f} m")
    print(f"  full-rate dot path (all frames): {path_len([r[1] for r in recs if r[1]]):.1f} m   "
          f"ekf path: {path_len([r[2] for r in recs if r[2]]):.1f} m")
    print("=" * 74)

    for name, track in (("DOT (global_t_, user-facing)", dot_xz),
                        ("EKF (p_G_ dead-reckon)", ekf_xz)):
        yaw_ns, rms_ns = best_fit_yaw(track, gps_en)
        scale, yaw_s, rms_s = best_fit_yaw_scale(track, gps_en)
        vpath = path_len(track)
        print(f"\n{name}")
        print(f"  path @GPS-cadence : {vpath:.1f} m   (GPS {gps_path:.1f} m -> "
              f"{vpath / gps_path:.3f}x {'UNDER' if vpath < gps_path else 'over'})")
        print(f"  Procrustes no-scale : residual {rms_ns:6.2f} m  (yaw {math.degrees(yaw_ns):+.1f} deg)")
        print(f"  Procrustes w/ scale : residual {rms_s:6.2f} m  (yaw {math.degrees(yaw_s):+.1f} deg, "
              f"scale {scale:.3f}  -> engine is {1/scale:.2f}x too {'small' if scale > 1 else 'big'})")

    # Speed: fused vs GPS instantaneous (between unique fixes).
    print("\nSPEED (fused_speed_mps vs GPS):")
    gps_spd = []
    for i in range(1, len(gps_fixes)):
        dt = gps_fixes[i][0] - gps_fixes[i - 1][0]
        if dt <= 0:
            continue
        d = haversine_m(gps_fixes[i - 1][1], gps_fixes[i - 1][2],
                        gps_fixes[i][1], gps_fixes[i][2])
        gps_spd.append(d / dt)
    spds = [r[6] for r in recs if r[6] is not None]
    if spds:
        spds_sorted = sorted(spds)
        print(f"  fused: median {spds_sorted[len(spds)//2]:.2f}  "
              f"p90 {spds_sorted[int(len(spds)*0.9)]:.2f}  max {max(spds):.2f} m/s")
    if gps_spd:
        gs = sorted(gps_spd)
        print(f"  gps  : median {gs[len(gs)//2]:.2f}  p90 {gs[int(len(gs)*0.9)]:.2f}  "
              f"max {max(gs):.2f} m/s   (mean {sum(gps_spd)/len(gps_spd):.2f})")
    print()

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed; skipping plot", file=sys.stderr)
            return

        def align(track):  # Procrustes no-scale onto GPS, for shape overlay
            yaw, _ = best_fit_yaw(track, gps_en)
            cvx = sum(p[0] for p in track) / len(track)
            cvy = sum(p[1] for p in track) / len(track)
            cge = sum(p[0] for p in gps_en) / len(gps_en)
            cgn = sum(p[1] for p in gps_en) / len(gps_en)
            c, s = math.cos(yaw), math.sin(yaw)
            return [(c * (x - cvx) - s * (y - cvy) + cge,
                     s * (x - cvx) + c * (y - cvy) + cgn) for x, y in track]

        dot_a, ekf_a = align(dot_xz), align(ekf_xz)
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.plot([p[0] for p in gps_en], [p[1] for p in gps_en],
                "-g", lw=2.2, label=f"GPS ({gps_path:.0f} m)")
        ax.plot([p[0] for p in dot_a], [p[1] for p in dot_a],
                "-b", lw=1.6, label=f"DOT global_t_ ({path_len(dot_xz):.0f} m)")
        ax.plot([p[0] for p in ekf_a], [p[1] for p in ekf_a],
                "-r", lw=1.0, alpha=0.7, label=f"EKF p_G ({path_len(ekf_xz):.0f} m)")
        ax.plot(gps_en[0][0], gps_en[0][1], "ko", ms=9, label="start")
        ax.set_aspect("equal"); ax.grid(True)
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
        ax.set_title(f"Current engine vs GPS — {os.path.basename(args.csv)}\n"
                     f"(Procrustes-aligned, no scale)")
        ax.legend()
        fig.tight_layout(); fig.savefig(args.plot, dpi=110); plt.close(fig)
        print(f"plot saved: {args.plot}")


if __name__ == "__main__":
    main()
