#!/usr/bin/env python3
"""Probe: does IPM under-read / spike correlate with per-frame |gyro|?

For the de-rotation lens. We have per-point gx/gy/gz (rad/s, body frame),
gp_flow_speed (IPM, m/s), GPS truth. Question: do IPM spikes coincide with
high |gyro| (turn-time over-/under-subtraction), and is the 0.65x under-read
present even on the LOW-gyro (straight) frames where derotation is ~a no-op?

If the under-read persists at near-zero gyro, the de-rotation is NOT the cause
of the 0.65x bias (it would be a translational/geometry bug). If spikes track
high gyro, that's a derotation interval/sign/units problem.
"""
import json, math, sys

MS_TO_KMH = 3.6
R_EARTH_M = 6371000.0

def haversine_m(a, b, c, d):
    p1, p2 = math.radians(a), math.radians(c)
    dphi = math.radians(c - a); dl = math.radians(d - b)
    x = math.sin(dphi/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R_EARTH_M*math.asin(min(1.0, math.sqrt(x)))

def gps_track(pts, accmax=20.0):
    fixes = []
    for p in pts:
        glat,glng,gacc,ts = p.get("glat"),p.get("glng"),p.get("gacc"),p.get("ts")
        if glat is None or glng is None or ts is None: continue
        if gacc is not None and gacc > accmax: continue
        if fixes:
            _,pa,po = fixes[-1]
            if haversine_m(pa,po,glat,glng) < 0.05: continue
        fixes.append((ts,glat,glng))
    tr=[]
    for i in range(1,len(fixes)):
        t0,a0,o0=fixes[i-1]; t1,a1,o1=fixes[i]
        dt=(t1-t0)/1000.0
        if 0.2<dt<5.0: tr.append((t1, haversine_m(a0,o0,a1,o1)/dt))
    return tr

def gps_at(tr, ts, gap=2500):
    best,bd=None,gap
    for t,s in tr:
        d=abs(t-ts)
        if d<=bd: best,bd=s,d
    return best

def med(xs):
    s=sorted(xs); n=len(s)
    return float("nan") if n==0 else (s[n//2] if n%2 else 0.5*(s[n//2-1]+s[n//2]))

def main(path):
    data=json.load(open(path, encoding="utf-8"))
    pts=data.get("points",[])
    tr=gps_track(pts)
    rows=[]  # (ts, gps_mps, gp_mps, gyro_mag_dps, dt_s)
    prev_ts=None
    for p in pts:
        ts=p.get("ts")
        gp=p.get("gp_flow_speed")
        g=gps_at(tr,ts) if tr else None
        gx,gy,gz=p.get("gx"),p.get("gy"),p.get("gz")
        gmag=None
        if None not in (gx,gy,gz):
            gmag=math.degrees(math.sqrt(gx*gx+gy*gy+gz*gz))
        dt=(ts-prev_ts)/1000.0 if prev_ts else None
        prev_ts=ts
        if g is not None and gp is not None and gmag is not None:
            rows.append((ts,g,gp,gmag,dt))
    print(f"{path.split(chr(92))[-1]}: {len(rows)} aligned rows with gyro+gps+gp")
    if not rows: return

    # dt distribution (is the flow interval stable ~1/15s, or are there 10-frame gaps?)
    dts=[r[4] for r in rows if r[4] and 0<r[4]<2]
    print(f"  dt (flow interval): median {med(dts)*1000:.1f} ms  "
          f"p90 {sorted(dts)[int(0.9*len(dts))]*1000:.1f} ms  max {max(dts)*1000:.1f} ms  (n={len(dts)})")

    # Bucket by gyro magnitude and report ratio in each bucket.
    buckets=[(0,5),(5,10),(10,20),(20,40),(40,80),(80,1e9)]
    print("  gyro_dps_bin   n   GPS_kmh  IPM_kmh  ratio   IPM_MAE_kmh")
    for lo,hi in buckets:
        sel=[r for r in rows if lo<=r[3]<hi]
        if len(sel)<5:
            print(f"   [{lo:4.0f},{hi:4.0f})  {len(sel):4d}   (too few)")
            continue
        gps_k=med([r[1]*MS_TO_KMH for r in sel])
        ipm_k=med([r[2]*MS_TO_KMH for r in sel])
        ratios=[r[2]/r[1] for r in sel if r[1]>1e-6]
        mae=sum(abs(r[2]-r[1]) for r in sel)/len(sel)*MS_TO_KMH
        print(f"   [{lo:4.0f},{hi:4.0f})  {len(sel):4d}   {gps_k:6.1f}  {ipm_k:6.1f}  {med(ratios):.2f}x   {mae:5.1f}")

    # Spikes: IPM frames where IPM >> GPS. Do they coincide with high gyro?
    spikes=[r for r in rows if r[2]*MS_TO_KMH > r[1]*MS_TO_KMH + 8 and r[2]*MS_TO_KMH>20]
    if spikes:
        print(f"  SPIKES (IPM>GPS+8 and IPM>20km/h): n={len(spikes)}  "
              f"median |gyro|={med([r[3] for r in spikes]):.1f} dps  "
              f"vs all-frames median |gyro|={med([r[3] for r in rows]):.1f} dps")
    else:
        print("  no spikes by the IPM>GPS+8 threshold")

    # Low-gyro straight-line subset: is the 0.65x under-read STILL there when derotation≈no-op?
    straight=[r for r in rows if r[3]<5 and r[1]>=1.5]
    if straight:
        ratios=[r[2]/r[1] for r in straight if r[1]>1e-6]
        print(f"  STRAIGHT (|gyro|<5dps, GPS>=5.4km/h): n={len(straight)}  ratio {med(ratios):.2f}x  "
              f"(under-read here => NOT a derotation bug)")

if __name__=="__main__":
    for p in sys.argv[1:]:
        main(p); print()
