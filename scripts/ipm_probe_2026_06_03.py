#!/usr/bin/env python3
"""IPM ground-flow under-read probe (ROBUST lens, 2026-06-03).

Correlates the recorded read-only IPM speed (gp_flow_speed) against GPS truth and
against every per-point covariate the recorder logs (inlier count `inl`, mean flow
`mflow`, gyro magnitude, dt between frames, heading). Goal: decide whether the
consistent ~0.65x under-read is a SYSTEMATIC scale bug (present even on clean,
high-inlier, low-gyro frames) or a robustness/median artifact (only on noisy frames).

It reuses the GPS-track machinery from compare_speeds_gps.py.
"""
import json, math, sys, argparse, statistics

R_EARTH_M = 6371000.0
MS_TO_KMH = 3.6


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb/2)**2
    return 2 * R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))


def gps_track(pts, acc_max=20.0):
    fixes = []
    for p in pts:
        glat, glng, gacc, ts = p.get("glat"), p.get("glng"), p.get("gacc"), p.get("ts")
        if glat is None or glng is None or ts is None:
            continue
        if gacc is not None and gacc > acc_max:
            continue
        if fixes:
            _, pl, pn = fixes[-1]
            if haversine_m(pl, pn, glat, glng) < 0.05:
                continue
        fixes.append((ts, glat, glng))
    track = []
    for i in range(1, len(fixes)):
        t0, la0, lo0 = fixes[i-1]
        t1, la1, lo1 = fixes[i]
        dt = (t1 - t0) / 1000.0
        if 0.2 < dt < 5.0:
            track.append((t1, haversine_m(la0, lo0, la1, lo1)/dt))
    return track


def gps_at(track, ts, max_gap=2500):
    best, bd = None, max_gap
    for tt, sp in track:
        d = abs(tt - ts)
        if d <= bd:
            best, bd = sp, d
    return best


def med(xs):
    return statistics.median(xs) if xs else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim")
    ap.add_argument("--move-mps", type=float, default=2.0)
    args = ap.parse_args()
    d = json.load(open(args.sim, encoding="utf-8"))
    pts = d["points"]
    track = gps_track(pts)
    print(f"{args.sim}\n  {len(pts)} pts, {len(track)} GPS fixes")

    rows = []  # dict per moving point
    prev_ts = None
    for p in pts:
        ts = p["ts"]
        dt = (ts - prev_ts) / 1000.0 if prev_ts else None
        prev_ts = ts
        g = gps_at(track, ts)
        gp = p.get("gp_flow_speed")
        if g is None or gp is None or g < args.move_mps:
            continue
        gx, gy, gz = p.get("gx", 0) or 0, p.get("gy", 0) or 0, p.get("gz", 0) or 0
        gyro_mag = math.degrees(math.sqrt(gx*gx + gy*gy + gz*gz))  # deg/s
        rows.append(dict(g=g, gp=gp, ratio=gp/g if g > 1e-6 else None,
                         inl=p.get("inl"), mflow=p.get("mflow"),
                         gyro=gyro_mag, dt=dt, hdg=p.get("hdg")))
    if not rows:
        print("  no moving rows"); return

    ratios = [r["ratio"] for r in rows if r["ratio"] is not None]
    print(f"  moving samples: {len(rows)}   overall ratio median {med(ratios):.3f}")

    # 1. Does the under-read survive on CLEAN frames? Filter to high-inlier, low-gyro, normal-dt.
    def subset(name, pred):
        sub = [r for r in rows if pred(r) and r["ratio"] is not None]
        if not sub:
            print(f"  {name:38s}: (0 samples)"); return
        rr = [r["ratio"] for r in sub]
        print(f"  {name:38s}: n={len(sub):4d}  ratio_med={med(rr):.3f}  "
              f"ratio_p25={sorted(rr)[len(rr)//4]:.3f}  ratio_p75={sorted(rr)[3*len(rr)//4]:.3f}  "
              f"GPS_med={med([r['g'] for r in sub])*MS_TO_KMH:.1f}")

    inls = [r["inl"] for r in rows if r["inl"] is not None]
    gyros = [r["gyro"] for r in rows]
    print(f"  inl: min={min(inls)} med={med(inls):.0f} max={max(inls)}   "
          f"gyro deg/s: med={med(gyros):.1f} p90={sorted(gyros)[int(0.9*len(gyros))]:.1f} max={max(gyros):.1f}")
    print("  --- under-read by subset (is 0.65x present on CLEAN frames?) ---")
    subset("ALL moving", lambda r: True)
    subset("high inlier (inl>=20)", lambda r: (r["inl"] or 0) >= 20)
    subset("very high inlier (inl>=30)", lambda r: (r["inl"] or 0) >= 30)
    subset("low gyro (<10 deg/s)", lambda r: r["gyro"] < 10)
    subset("very low gyro (<5 deg/s)", lambda r: r["gyro"] < 5)
    subset("CLEAN (inl>=20 & gyro<10)", lambda r: (r["inl"] or 0) >= 20 and r["gyro"] < 10)
    subset("CLEANEST (inl>=25 & gyro<5)", lambda r: (r["inl"] or 0) >= 25 and r["gyro"] < 5)
    subset("normal dt (0.04-0.09s)", lambda r: r["dt"] and 0.04 < r["dt"] < 0.09)
    subset("CLEAN+normaldt", lambda r: (r["inl"] or 0) >= 20 and r["gyro"] < 10 and r["dt"] and 0.04 < r["dt"] < 0.09)

    # 2. Speed-bucketed ratio: is the under-read a constant SCALE (ratio flat vs speed) or
    #    a fixed offset (ratio rising with speed) or saturating (ratio falling at high speed)?
    print("  --- ratio vs GPS speed bucket (flat => pure scale bug) ---")
    buckets = [(0, 3), (3, 5), (5, 7), (7, 10), (10, 30)]
    for lo, hi in buckets:
        sub = [r for r in rows if lo <= r["g"] < hi and r["ratio"] is not None]
        if sub:
            rr = [r["ratio"] for r in sub]
            print(f"    GPS {lo*MS_TO_KMH:4.0f}-{hi*MS_TO_KMH:4.0f} km/h: n={len(sub):4d} "
                  f"ratio_med={med(rr):.3f}  IPM_med={med([r['gp'] for r in sub])*MS_TO_KMH:5.1f}")

    # 3. Spikes: IPM >> GPS. How many, and on what kind of frame?
    spikes = [r for r in rows if r["gp"] > r["g"] * 1.5]
    print(f"  --- spikes (IPM > 1.5x GPS): {len(spikes)} of {len(rows)} ({100*len(spikes)/len(rows):.1f}%) ---")
    if spikes:
        print(f"    spike inl med={med([s['inl'] for s in spikes if s['inl'] is not None]):.0f}  "
              f"gyro med={med([s['gyro'] for s in spikes]):.1f}  dt med={med([s['dt'] for s in spikes if s['dt']]):.3f}  "
              f"IPM med={med([s['gp'] for s in spikes])*MS_TO_KMH:.1f} km/h")


if __name__ == "__main__":
    main()
