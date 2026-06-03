#!/usr/bin/env python3
"""Probe: can we detect a TRUE STOP from the recorded signals without false-firing at cruise?

The IPM speed leaks a ~2-10 km/h floor when stopped (camera shake on the ground points).
To zero it we need a stationarity gate that is NOT fooled by constant cruise (a cruising
scooter has near-zero LINEAR accel too). This prints, per recording and bucketed by GPS
speed, the candidate gate signals so we can see which one separates STOP from GO:

  - accel_std : rolling std of |accel| over ~0.7 s  (road vibration proxy)
  - gyro_mag  : |gyro| mean                          (turning / wobble)
  - ipm_kmh   : the IPM speed itself                 (its own flow magnitude)

If a signal's STOP distribution is cleanly below its CRUISE distribution, that's the gate.
"""
import json, math, sys, statistics as st

R = 6371000.0
def hav(a, b, c, d):
    p1, p2 = math.radians(a), math.radians(c)
    dp, dl = math.radians(c - a), math.radians(d - b)
    x = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(min(1, math.sqrt(x)))

def gps_track(pts, acc_max=20.0):
    fx = []
    for p in pts:
        la, lo, ac, ts = p.get("glat"), p.get("glng"), p.get("gacc"), p.get("ts")
        if la is None or lo is None or ts is None: continue
        if ac is not None and ac > acc_max: continue
        if fx and hav(fx[-1][1], fx[-1][2], la, lo) < 0.05: continue
        fx.append((ts, la, lo))
    tr = []
    for i in range(1, len(fx)):
        dt = (fx[i][0]-fx[i-1][0])/1000.0
        if 0.2 < dt < 5.0:
            tr.append((fx[i][0], hav(fx[i-1][1], fx[i-1][2], fx[i][1], fx[i][2])/dt))
    return tr

def gps_at(tr, ts, gap=2500):
    best, bd = None, gap
    for t, s in tr:
        d = abs(t-ts)
        if d <= bd: best, bd = s, d
    return best

def dist(name, xs):
    if not xs: return f"{name}: (none)"
    s = sorted(xs)
    q = lambda f: s[min(len(s)-1, int(f*len(s)))]
    return f"{name}: n={len(s)} med={st.median(s):.3f} p10={q(.1):.3f} p90={q(.9):.3f} max={max(s):.3f}"

def main():
    for path in sys.argv[1:]:
        d = json.load(open(path, encoding="utf-8"))
        pts = d.get("points", [])
        name = path.split("/")[-1].split("\\")[-1]
        tr = gps_track(pts)
        amag = [math.sqrt((p.get("ax") or 0)**2+(p.get("ay") or 0)**2+(p.get("az") or 0)**2) for p in pts]
        gmag = [math.sqrt((p.get("gx") or 0)**2+(p.get("gy") or 0)**2+(p.get("gz") or 0)**2) for p in pts]
        W = 10
        astd = [0.0]*len(pts)
        for i in range(len(pts)):
            lo = max(0, i-W); seg = amag[lo:i+1]
            astd[i] = st.pstdev(seg) if len(seg) > 1 else 0.0
        # VIO displacement speed over the same ~0.7 s window (the dot's own motion).
        viosp = [0.0]*len(pts)
        for i in range(len(pts)):
            j = max(0, i-W)
            dx = (pts[i].get("vx") or 0) - (pts[j].get("vx") or 0)
            dz = (pts[i].get("vz") or 0) - (pts[j].get("vz") or 0)
            dt = (pts[i]["ts"] - pts[j]["ts"])/1000.0
            viosp[i] = (math.sqrt(dx*dx+dz*dz)/dt*3.6) if dt > 1e-3 else 0.0
        buckets = {"STOP(<0.5)": [], "CRAWL(0.5-1.5)": [], "CRUISE(>4)": []}
        pflag = {"STOP(<0.5)": {}, "CRAWL(0.5-1.5)": {}, "CRUISE(>4)": {}}
        for i, p in enumerate(pts):
            g = gps_at(tr, p["ts"])
            if g is None: continue
            row = (astd[i], gmag[i], (p.get("gp_flow_speed") or 0)*3.6, (p.get("fused_speed") or 0)*3.6, viosp[i])
            key = "STOP(<0.5)" if g < 0.5 else "CRAWL(0.5-1.5)" if g < 1.5 else "CRUISE(>4)" if g > 4.0 else None
            if key:
                buckets[key].append(row)
                pf = p.get("pflags")
                pflag[key][pf] = pflag[key].get(pf, 0) + 1
        print(f"\n=== {name} (gps fixes {len(tr)}) ===")
        for b, rows in buckets.items():
            if not rows: continue
            print(f"  [{b}] n={len(rows)}")
            print("    ", dist("accel_std", [r[0] for r in rows]))
            print("    ", dist("gyro_mag ", [r[1] for r in rows]))
            print("    ", dist("ipm_kmh  ", [r[2] for r in rows]))
            print("    ", dist("fused_kmh", [r[3] for r in rows]))
            print("    ", dist("vio_kmh  ", [r[4] for r in rows]))
            print("     pflags:", dict(sorted(pflag[b].items(), key=lambda kv: -kv[1])))

if __name__ == "__main__":
    main()
