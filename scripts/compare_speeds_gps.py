#!/usr/bin/env python3
"""3-way speed comparison vs GPS for a NavSight sim recording.

Compares, per recording:
  - fused_speed   : the speed shown to the user now (Tracker trajectory_speed_mps_)
  - gp_flow_speed : the read-only IPM ground-plane candidate (Tracker::updateGroundFlowSpeed)
  - GPS speed     : ground truth, derived from consecutive DISTINCT GPS fixes (m/s)

It answers the two owner complaints directly:
  (A) "displayed speed kept going to 0" -> FREEZE: % of moving samples where fused ~= 0.
  (B) "standing still but speed was not 0" -> LEAK: what fused/IPM read while GPS says stopped.

plus the moving-accuracy scoreboard (ratio, MAE, bias) so we can say which channel
tracks GPS best.

GPS speed is taken between DISTINCT fixes only: the recorder writes a point every camera
frame (~15 Hz) but the GPS fix updates ~1 Hz, so consecutive points repeat the same
lat/lng. Differencing every point would inject fake 0 speeds. We difference unique fixes
and hold each fix's speed across the points until the next fix.

Usage:
    python scripts/compare_speeds_gps.py <sim.json> [<sim2.json> ...] \
        [--gps-acc-max 20] [--stop-mps 0.5] [--move-mps 1.5] [--freeze-mps 0.3]
"""
import json
import math
import sys
import argparse

R_EARTH_M = 6371000.0
MS_TO_KMH = 3.6


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


def pct(num, den):
    return 100.0 * num / den if den else float("nan")


def gps_speed_track(pts, gps_acc_max):
    """Return a list of (ts, gps_mps) at each DISTINCT GPS fix, gated on accuracy.

    Distinct = the fix moved (>0.3 m) or enough time passed; this avoids the repeated
    same-fix points the recorder writes between 1 Hz GPS updates.
    """
    fixes = []  # (ts, lat, lng)
    for p in pts:
        glat, glng, gacc, ts = p.get("glat"), p.get("glng"), p.get("gacc"), p.get("ts")
        if glat is None or glng is None or ts is None:
            continue
        if gacc is not None and gacc > gps_acc_max:
            continue
        if fixes:
            _, plat, plng = fixes[-1]
            if haversine_m(plat, plng, glat, glng) < 0.05:   # identical repeated fix
                continue
        fixes.append((ts, glat, glng))
    track = []
    for i in range(1, len(fixes)):
        ts0, la0, lo0 = fixes[i - 1]
        ts1, la1, lo1 = fixes[i]
        dt = (ts1 - ts0) / 1000.0
        if 0.2 < dt < 5.0:
            track.append((ts1, haversine_m(la0, lo0, la1, lo1) / dt))
    return track


def gps_at(track, ts, max_gap_ms=2500):
    """Hold the GPS-fix speed nearest in time to ts (within max_gap_ms), else None."""
    best, bestdt = None, max_gap_ms
    for tts, spd in track:
        d = abs(tts - ts)
        if d <= bestdt:
            best, bestdt = spd, d
    return best


def analyze(path, args):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    pts = data.get("points", [])
    name = path.split("/")[-1].split("\\")[-1]
    print(f"\n{'='*78}\n{name}  ({len(pts)} points)")
    if not pts:
        print("  no points"); return

    dur_s = (pts[-1]["ts"] - pts[0]["ts"]) / 1000.0 if len(pts) > 1 else 0.0
    track = gps_speed_track(pts, args.gps_acc_max)
    if not track:
        print("  no usable GPS track (jammed/indoor/too short) — cannot compare to GPS"); return
    gps_kmh_all = [s * MS_TO_KMH for _, s in track]
    print(f"  duration {dur_s:.0f}s   GPS fixes used {len(track)}   "
          f"GPS speed: median {median(gps_kmh_all):.1f}  max {max(gps_kmh_all):.1f} km/h")

    # Build aligned rows: (gps_mps, fused_mps, gp_mps) for points that have a GPS speed.
    rows = []
    for p in pts:
        g = gps_at(track, p["ts"])
        if g is None:
            continue
        rows.append((g, p.get("fused_speed"), p.get("gp_flow_speed")))
    if not rows:
        print("  no points aligned to GPS"); return

    stop = [r for r in rows if r[0] < args.stop_mps]
    move = [r for r in rows if r[0] >= args.move_mps]
    print(f"  aligned {len(rows)}   stopped(<{args.stop_mps} m/s) {len(stop)}   "
          f"moving(>={args.move_mps} m/s) {len(move)}")

    for label, idx in (("fused (DISPLAYED)", 1), ("gp_flow (IPM)   ", 2)):
        vals_move = [r for r in move if r[idx] is not None]
        vals_stop = [r for r in stop if r[idx] is not None]
        print(f"\n  --- {label} ---")
        if vals_move:
            ratios = [r[idx] / r[0] for r in vals_move if r[0] > 1e-6]
            mae = sum(abs(r[idx] - r[0]) for r in vals_move) / len(vals_move) * MS_TO_KMH
            bias = sum(r[idx] - r[0] for r in vals_move) / len(vals_move) * MS_TO_KMH
            frozen = [r for r in vals_move if r[idx] < args.freeze_mps]
            print(f"    MOVING : med {median([r[idx]*MS_TO_KMH for r in vals_move]):5.1f} km/h "
                  f"vs GPS {median([r[0]*MS_TO_KMH for r in vals_move]):5.1f}   "
                  f"ratio {median(ratios):.2f}x  MAE {mae:.1f}  bias {bias:+.1f} km/h")
            print(f"    FREEZE : {pct(len(frozen), len(vals_move)):4.0f}% of moving samples read "
                  f"<{args.freeze_mps*MS_TO_KMH:.1f} km/h (≈0) while GPS says moving")
        else:
            print("    MOVING : no values")
        if vals_stop:
            leak = [r for r in vals_stop if r[idx] * MS_TO_KMH > 1.0]
            print(f"    STOPPED: med {median([r[idx]*MS_TO_KMH for r in vals_stop]):5.1f} km/h  "
                  f"max {max(r[idx]*MS_TO_KMH for r in vals_stop):5.1f}   "
                  f"LEAK {pct(len(leak), len(vals_stop)):4.0f}% read >1 km/h while GPS stopped")
        else:
            print("    STOPPED: no stationary GPS samples detected")

    es = data.get("event_summary")
    if isinstance(es, dict):
        keys = ["is_static_true", "is_static_false", "zupt_applied", "zupt_rotinplace_fired",
                "depth_flow_calib_updates", "depth_flow_updates", "accel_k_calib",
                "trans_fallback", "ground_flow_updates", "ground_flow_funnel_reject"]
        hits = {k: es[k] for k in keys if k in es}
        if hits:
            print("\n  event_summary:", "  ".join(f"{k}={v}" for k, v in hits.items()))

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            t0 = pts[0]["ts"]
            tx = [(p["ts"] - t0) / 1000.0 for p in pts]
            fu = [(p.get("fused_speed") or 0) * MS_TO_KMH for p in pts]
            gp = [(p.get("gp_flow_speed") or 0) * MS_TO_KMH for p in pts]
            gt = [(t - t0) / 1000.0 for t, _ in track]
            gv = [s * MS_TO_KMH for _, s in track]
            plt.figure(figsize=(14, 5))
            plt.plot(tx, fu, color="#d33", lw=1.0, label="fused (displayed)")
            plt.plot(tx, gp, color="#28a", lw=1.0, label="gp_flow (IPM)")
            plt.plot(gt, gv, color="#2a2", lw=2.0, marker="o", ms=3, label="GPS (truth)")
            plt.xlabel("time (s)"); plt.ylabel("speed (km/h)")
            plt.title(name); plt.legend(); plt.grid(alpha=0.3); plt.ylim(bottom=0)
            out = path.rsplit(".", 1)[0] + "_speedcmp.png"
            plt.savefig(out, dpi=110, bbox_inches="tight"); plt.close()
            print(f"  plot -> {out}")
        except Exception as e:
            print(f"  (plot skipped: {e})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sims", nargs="+")
    ap.add_argument("--gps-acc-max", type=float, default=20.0)
    ap.add_argument("--stop-mps", type=float, default=0.5, help="GPS below this = stopped")
    ap.add_argument("--move-mps", type=float, default=1.5, help="GPS above this = clearly moving")
    ap.add_argument("--freeze-mps", type=float, default=0.3, help="displayed below this = frozen/0")
    ap.add_argument("--plot", action="store_true", help="save a <sim>_speedcmp.png time-series")
    args = ap.parse_args()
    for s in args.sims:
        try:
            analyze(s, args)
        except Exception as e:
            print(f"\n{s}: ERROR {e}")
    print()


if __name__ == "__main__":
    main()
