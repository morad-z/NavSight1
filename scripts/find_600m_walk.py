"""Scan candidate recordings to find the ~600 m walk with accurate GPS.
Reports duration, GPS path length (haversine), VIO path, GPS accuracy stats."""
import json, math, glob, os, statistics as st

def hav(a, b, c, d):
    R = 6371000.0
    p1, p2 = math.radians(a), math.radians(c)
    dp = math.radians(c - a); dl = math.radians(d - b)
    x = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(x))

paths = (glob.glob('cand_*.json') + glob.glob('tests/sims/*.json')
         + glob.glob('good simulations/*.json') + glob.glob('tests/sims/180/*.json'))
rows = []
for fn in paths:
    try:
        d = json.load(open(fn))
    except Exception as e:
        continue
    pts = d.get('points') or []
    if not pts or 'glat' not in pts[0]:
        continue
    tk = 'ts' if 'ts' in pts[0] else 'timestamp'
    dur = (pts[-1][tk]-pts[0][tk])/1000.0
    gps = [(p[tk], p['glat'], p['glng'], p.get('gacc')) for p in pts
           if p.get('glat') is not None and p.get('glng') is not None]
    if len(gps) < 2:
        continue
    accs = [g[3] for g in gps if g[3] is not None]
    gp = [g for g in gps if g[3] is None or g[3] <= 12]
    gpath = sum(hav(gp[i-1][1], gp[i-1][2], gp[i][1], gp[i][2]) for i in range(1, len(gp)))
    vp, prev = 0.0, None
    for p in pts:
        x, z = p.get('vx'), p.get('vz')
        if x is None: continue
        if prev: vp += math.hypot(x-prev[0], z-prev[1])
        prev = (x, z)
    accmed = st.median(accs) if accs else 999
    rows.append((gpath, fn, dur, len(gps), accmed, min(accs) if accs else 999, vp))

rows.sort(reverse=True)  # longest GPS path first
print(f"{'GPS_path':>9} {'dur':>5} {'fixes':>6} {'accMed':>7} {'accMin':>6} {'VIO_path':>8}  file")
for gpath, fn, dur, nf, accmed, accmin, vp in rows[:20]:
    print(f"{gpath:9.0f} {dur:5.0f} {nf:6d} {accmed:7.1f} {accmin:6.1f} {vp:8.0f}  {fn}")
