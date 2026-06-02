#!/usr/bin/env python3
"""Review recorded NavSight sims vs GPS: heading offset, 180° flips, speed under-read, trajectory.

Per-sample fields: ts(ms), vlat/vlng(dot), glat/glng(GPS), hdg(radians, compass 0=N CW),
mm_lat/mm_lng(map-matched), gacc(GPS acc m), maneuver. Usage: python review_sims_heading_speed.py <dir>
"""
import json, glob, math, os, sys, statistics as st

R = 6371000.0
def hav(a, b, c, d):
    la1, la2 = math.radians(a), math.radians(c)
    dla, dlo = math.radians(c - a), math.radians(d - b)
    h = math.sin(dla/2)**2 + math.cos(la1)*math.cos(la2)*math.sin(dlo/2)**2
    return 2*R*math.asin(min(1.0, math.sqrt(h)))
def bearing(a, b, c, d):  # compass deg 0=N CW
    mlng = 111320.0*math.cos(math.radians(a))
    east = (d - b)*mlng; north = (c - a)*111320.0
    return (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
def wrap180(x): return ((x + 540.0) % 360.0) - 180.0
def plen(pts): return sum(hav(pts[i-1][0], pts[i-1][1], pts[i][0], pts[i][1]) for i in range(1, len(pts)))

def review(path):
    d = json.load(open(path))
    pts = d.get("points", [])
    es = d.get("event_summary", {}) or {}
    rows = []
    for p in pts:
        def g(k):
            v = p.get(k)
            return v if isinstance(v, (int, float)) else None
        rows.append(dict(ts=p.get("ts"), vlat=g("vlat"), vlng=g("vlng"), glat=g("glat"),
                         glng=g("glng"), hdg=g("hdg"), gacc=g("gacc"), mmlat=g("mm_lat"), mmlng=g("mm_lng")))
    name = os.path.basename(path).replace("simulation_data_", "").replace(".json", "")
    n = len(rows)

    # GPS fixes (good acc), de-duplicated by movement
    gps = [(r["ts"], r["glat"], r["glng"]) for r in rows
           if r["glat"] and r["glng"] and (r["gacc"] is None or r["gacc"] <= 20)]
    gfix = []
    for t, la, lo in gps:
        if not gfix or hav(gfix[-1][1], gfix[-1][2], la, lo) > 3.0:
            gfix.append((t, la, lo))

    # VIO heading series (rad→deg)
    hser = [(r["ts"], math.degrees(r["hdg"])) for r in rows if r["hdg"] is not None and r["ts"]]
    def hdg_at(t):
        best = min(hser, key=lambda x: abs(x[0]-t), default=None)
        return best[1] if best else None

    # Heading error vs GPS course per GPS segment (need ≥8 m to trust the course)
    herr = []
    for i in range(1, len(gfix)):
        t0, la0, lo0 = gfix[i-1]; t1, la1, lo1 = gfix[i]
        if hav(la0, lo0, la1, lo1) < 8.0: continue
        crs = bearing(la0, lo0, la1, lo1)
        h = hdg_at((t0+t1)//2)
        if h is None: continue
        herr.append(wrap180(h - crs))
    # 180° flips in the raw VIO heading series (consecutive jump > 150°)
    flips = sum(1 for i in range(1, len(hser)) if abs(wrap180(hser[i][1]-hser[i-1][1])) > 150.0)

    # Speed: GPS vs VIO per GPS segment
    gsp, vsp, cruising_but_slow = [], [], 0
    vd = {r["ts"]: (r["vlat"], r["vlng"]) for r in rows if r["vlat"] and r["vlng"] and r["ts"]}
    vts = sorted(vd)
    def vio_pos_at(t):
        if not vts: return None
        k = min(vts, key=lambda x: abs(x-t)); return vd[k]
    for i in range(1, len(gfix)):
        t0, la0, lo0 = gfix[i-1]; t1, la1, lo1 = gfix[i]
        dt = (t1-t0)/1000.0
        if dt < 0.5: continue
        gs = hav(la0, lo0, la1, lo1)/dt
        p0 = vio_pos_at(t0); p1 = vio_pos_at(t1)
        if not p0 or not p1: continue
        vs = hav(p0[0], p0[1], p1[0], p1[1])/dt
        gsp.append(gs); vsp.append(vs)
        if gs > 3.0 and vs < 1.5: cruising_but_slow += 1  # GPS cruising (>10.8km/h), VIO <5.4km/h

    vpath = plen([(r["vlat"], r["vlng"]) for r in rows if r["vlat"] and r["vlng"]])
    gpath = plen([(t, la, lo)[1:] for t, la, lo in gfix])
    mpath = plen([(r["mmlat"], r["mmlng"]) for r in rows if r["mmlat"] and r["mmlng"]])

    print(f"\n=== {name}  ({n} samples, {len(gfix)} GPS fixes) ===")
    print(f"  PATH:   GPS={gpath:.0f}m  VIO={vpath:.0f}m ({vpath/max(1,gpath):.2f}x)  MM={mpath:.0f}m ({mpath/max(1,gpath):.2f}x)")
    if herr:
        print(f"  HEADING err (VIO-GPScourse): median={st.median(herr):+.0f}°  mean={st.mean(herr):+.0f}°  "
              f"|err|>30°: {sum(1 for e in herr if abs(e)>30)}/{len(herr)}  |err|>120°(flipped): {sum(1 for e in herr if abs(e)>120)}/{len(herr)}")
    print(f"  HEADING 180°-flips in raw series: {flips}")
    if gsp:
        print(f"  SPEED:  GPS median={st.median(gsp):.1f} m/s ({st.median(gsp)*3.6:.0f}km/h)  "
              f"VIO median={st.median(vsp):.1f} m/s ({st.median(vsp)*3.6:.0f}km/h)  ratio={st.median(vsp)/max(0.01,st.median(gsp)):.2f}")
        print(f"  CRUISING-BUT-SLOW (GPS>10.8 & VIO<5.4 km/h): {cruising_but_slow}/{len(gsp)} segments")
    for k in ("madgwick_road_yaw_nudges_total", "map_position_corrections_total", "depth_flow_calib_updates"):
        if k in es: print(f"  counter {k} = {es[k]}")

if __name__ == "__main__":
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    for f in sorted(glob.glob(os.path.join(d, "simulation_data_*.json"))):
        try: review(f)
        except Exception as e: print(f"\n{os.path.basename(f)}: ERROR {e}")
