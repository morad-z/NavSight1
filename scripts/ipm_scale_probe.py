#!/usr/bin/env python3
"""IPM under-read diagnosis: is GPS/IPM a CLEAN CONSTANT (=> one scalar bug)
or does it VARY with geometry (=> directional / de-rotation bug)?

For every point with both a held GPS speed and a non-zero gp_flow_speed we compute
the per-frame implied scale  k = GPS / IPM. Then we look at:
  - the distribution of k (median, IQR, CV). A tight distribution => scalar bug.
  - correlation of k with |gyro| (turning), GPS speed, and heading-rate. A trend
    with |gyro| => the de-rotation is leaking rotation into translation.
  - whether the spikes (IPM >> GPS, k<<1) line up with high |gyro| (de-rotation
    over/under-shoot) rather than random.
"""
import json
import math
import sys

MS_TO_KMH = 3.6
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


def pct(xs, q):
    s = sorted(xs)
    if not s:
        return float("nan")
    i = int(q * (len(s) - 1))
    return s[i]


def gps_track(pts, acc_max=20.0):
    fixes = []
    for p in pts:
        glat, glng, gacc, ts = p.get("glat"), p.get("glng"), p.get("gacc"), p.get("ts")
        if glat is None or glng is None or ts is None:
            continue
        if gacc is not None and gacc > acc_max:
            continue
        if fixes:
            _, plat, plng = fixes[-1]
            if haversine_m(plat, plng, glat, glng) < 0.05:
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
    best, bestdt = None, max_gap_ms
    for tts, spd in track:
        d = abs(tts - ts)
        if d <= bestdt:
            best, bestdt = spd, d
    return best


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return float("nan")
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx == 0 or sy == 0:
        return float("nan")
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


def main(path):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    pts = data.get("points", [])
    track = gps_track(pts)
    rows = []  # (ts, gps_mps, ipm_mps, gyromag, gpsspeed)
    for i, p in enumerate(pts):
        g = gps_at(track, p["ts"])
        ipm = p.get("gp_flow_speed")
        if g is None or ipm is None or ipm <= 0.05 or g < 1.5:
            continue
        gx, gy, gz = p.get("gx", 0), p.get("gy", 0), p.get("gz", 0)
        gyromag = math.sqrt(gx * gx + gy * gy + gz * gz)  # rad/s
        rows.append((p["ts"], g, ipm, gyromag, g))
    if not rows:
        print("no rows"); return

    ks = [r[1] / r[2] for r in rows]  # GPS / IPM
    gyros = [r[3] for r in rows]
    gps_sp = [r[4] for r in rows]

    print(f"{path.split(chr(92))[-1]}   N={len(rows)}")
    print(f"  k = GPS/IPM  median={median(ks):.3f}  mean={sum(ks)/len(ks):.3f}")
    print(f"     p10={pct(ks,0.10):.3f}  p25={pct(ks,0.25):.3f}  p50={pct(ks,0.50):.3f}  "
          f"p75={pct(ks,0.75):.3f}  p90={pct(ks,0.90):.3f}")
    mk = median(ks)
    cv = math.sqrt(sum((k - mk) ** 2 for k in ks) / len(ks)) / mk
    iqr = pct(ks, 0.75) - pct(ks, 0.25)
    print(f"     CV(std/med)={cv:.2f}   IQR={iqr:.3f}  (IQR/med={iqr/mk:.2f})")
    print(f"  |gyro| rad/s: median={median(gyros):.3f}  p90={pct(gyros,0.90):.3f}  max={max(gyros):.3f}")

    print("\n  --- is the under-read geometry-dependent? ---")
    print(f"  corr(k, |gyro|)   = {pearson(ks, gyros):+.3f}   (≠0 => de-rotation leaks into k)")
    print(f"  corr(k, GPS_speed)= {pearson(ks, gps_sp):+.3f}   (≠0 => scale depends on speed)")
    print(f"  corr(IPM, GPS)    = {pearson([r[2] for r in rows],[r[1] for r in rows]):+.3f}")

    # Bin k by gyro magnitude to see the trend explicitly.
    print("\n  k median by |gyro| band:")
    bands = [(0.0, 0.1), (0.1, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, 99)]
    for lo, hi in bands:
        bk = [r[1] / r[2] for r in rows if lo <= r[3] < hi]
        bipm = [r[2] * MS_TO_KMH for r in rows if lo <= r[3] < hi]
        bgps = [r[1] * MS_TO_KMH for r in rows if lo <= r[3] < hi]
        if bk:
            print(f"    |gyro| {lo:.1f}-{hi:<4.1f} rad/s  N={len(bk):4d}  "
                  f"k_med={median(bk):.2f}  IPM_med={median(bipm):5.1f}  GPS_med={median(bgps):5.1f} km/h")

    # Spikes: IPM > GPS (k<1 strongly). Are they at high gyro?
    spikes = [r for r in rows if r[2] > r[1] * 1.3]
    if spikes:
        sg = [r[3] for r in spikes]
        print(f"\n  SPIKES (IPM>1.3*GPS): N={len(spikes)} ({100*len(spikes)/len(rows):.0f}%)  "
              f"|gyro| median={median(sg):.3f} vs overall {median(gyros):.3f}")

    # Bin k by GPS speed band — does scale drift with speed (=> depth/scale issue)?
    print("\n  k median by GPS speed band:")
    sbands = [(1.5, 3), (3, 5), (5, 7), (7, 9), (9, 99)]
    for lo, hi in sbands:
        bk = [r[1] / r[2] for r in rows if lo <= r[1] < hi]
        if bk:
            print(f"    GPS {lo*MS_TO_KMH:4.0f}-{hi*MS_TO_KMH:<4.0f} km/h  N={len(bk):4d}  k_med={median(bk):.2f}")


if __name__ == "__main__":
    for a in sys.argv[1:]:
        main(a)
        print()
