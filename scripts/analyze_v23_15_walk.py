"""Quick analysis of the v23.15 post-fix walk. Compare to v22 baseline / v23.14 broken walk."""
import json, math, re, sys
from collections import Counter

WALK_JSON = 'tests/sims/regression/visual/v23_15_walk_1778586148344.json'
WALK_LOG = 'tests/sims/regression/visual/v23_15_walk_1778586148344.logcat.txt'

# ── JSON ───────────────────────────────────────────────────────────────────
with open(WALK_JSON) as f:
    d = json.load(f)
es = d['event_summary']
pts = d.get('points') or []
print(f"=== JSON: {len(pts)} points, {len(es)} event_summary keys ===")

# Trajectory close-loop & path
def vio_pos(pt):
    return pt.get('vx'), pt.get('vy'), pt.get('vz')

def gps_pos(pt):
    return pt.get('gx'), pt.get('gy'), pt.get('gz')

if pts:
    vp_first = vio_pos(pts[0])
    vp_last = vio_pos(pts[-1])
    gp_first = gps_pos(pts[0])
    gp_last = gps_pos(pts[-1])
    if all(v is not None for v in vp_first + vp_last):
        d_vio = math.sqrt(sum((a-b)**2 for a,b in zip(vp_first,vp_last)))
        print(f"VIO close-loop (rx,ry,rz):  start={vp_first} end={vp_last}  Δ={d_vio:.3f} m")
    if all(v is not None for v in gp_first + gp_last):
        d_gps = math.sqrt(sum((a-b)**2 for a,b in zip(gp_first,gp_last)))
        print(f"GPS close-loop (gx,gy,gz):  start={gp_first} end={gp_last}  Δ={d_gps:.3f} m")

    # VIO path length
    pl_v = 0.0
    pl_g = 0.0
    for i in range(1,len(pts)):
        vp1 = vio_pos(pts[i-1]); vp2 = vio_pos(pts[i])
        gp1 = gps_pos(pts[i-1]); gp2 = gps_pos(pts[i])
        if all(v is not None for v in vp1+vp2):
            pl_v += math.sqrt(sum((a-b)**2 for a,b in zip(vp1,vp2)))
        if all(v is not None for v in gp1+gp2):
            pl_g += math.sqrt(sum((a-b)**2 for a,b in zip(gp1,gp2)))
    print(f"VIO path length: {pl_v:.1f} m")
    print(f"GPS path length: {pl_g:.1f} m")

print(f"event_summary total_path_dm: {es.get('total_path_dm')}  ({es.get('total_path_dm',0)/10:.1f} m)")

# ── LC_ABS analysis ────────────────────────────────────────────────────────
LC_RE = re.compile(r"LC_ABS: r_R=\[([^\]]+)\] r_p=\[([^\]]+)\] p_G=\[([^\]]+)\] target_p=\[([^\]]+)\].*m2=([\d.]+) m2_R=([\d.]+) m2_p=([\d.]+)")
lc_lines = []
with open(WALK_LOG, encoding='utf-8', errors='replace') as f:
    for line in f:
        m = LC_RE.search(line)
        if m:
            rR = [float(x) for x in m.group(1).split()]
            rp = [float(x) for x in m.group(2).split()]
            pG = [float(x) for x in m.group(3).split()]
            tp = [float(x) for x in m.group(4).split()]
            m2 = float(m.group(5)); m2R = float(m.group(6)); m2p = float(m.group(7))
            lc_lines.append((rR, rp, pG, tp, m2, m2R, m2p))
print(f"\n=== LC_ABS: {len(lc_lines)} lines ===")
if lc_lines:
    rR_norms = [math.sqrt(sum(x*x for x in r[0])) for r in lc_lines]
    rR_z = [r[0][2] for r in lc_lines]
    rp_norms = [math.sqrt(sum(x*x for x in r[1])) for r in lc_lines]
    m2s = [r[4] for r in lc_lines]
    m2Rs = [r[5] for r in lc_lines]
    m2ps = [r[6] for r in lc_lines]

    def percentiles(xs, ps=(50,90,99)):
        s = sorted(xs)
        return {p: s[min(len(s)-1, int(len(s)*p/100))] for p in ps}

    def deg(r): return r * 180.0/math.pi

    p = percentiles(rR_norms)
    print(f"|r_R| median={p[50]:.3f} rad ({deg(p[50]):.1f}°)  P90={p[90]:.3f} rad ({deg(p[90]):.1f}°)  P99={p[99]:.3f} rad ({deg(p[99]):.1f}°)")
    pos = sum(1 for z in rR_z if z>1)
    neg = sum(1 for z in rR_z if z<-1)
    print(f"r_R.z sign skew: |z|>1: neg={neg} pos={pos}  ratio={neg/max(1,pos):.1f}:1")
    p = percentiles(rp_norms)
    print(f"|r_p| median={p[50]:.3f} m  P90={p[90]:.3f} m  P99={p[99]:.3f} m")
    p = percentiles(m2s)
    print(f"m2   median={p[50]:.2f}  P90={p[90]:.2f}  thresh=22.5")
    p = percentiles(m2Rs)
    print(f"m2_R median={p[50]:.2f}  P90={p[90]:.2f}")
    p = percentiles(m2ps)
    print(f"m2_p median={p[50]:.2f}  P90={p[90]:.2f}")
    n_pass = sum(1 for m in m2s if m < 22.5)
    print(f"chi² PASS rate: {n_pass}/{len(m2s)} = {100*n_pass/len(m2s):.1f}%")
    rot_dom = sum(1 for r in lc_lines if r[5] > r[6])
    print(f"m2_R-dominated: {rot_dom}/{len(lc_lines)} = {100*rot_dom/len(lc_lines):.1f}%")

# ── ok=1 vs ok=0 ───────────────────────────────────────────────────────────
DAMP_RE = re.compile(r"LOOP_CLOSURE: damp.*ok=(\d)")
ok1 = 0; ok0 = 0
with open(WALK_LOG, encoding='utf-8', errors='replace') as f:
    for line in f:
        m = DAMP_RE.search(line)
        if m:
            if m.group(1) == '1': ok1 += 1
            else: ok0 += 1
print(f"\n=== Loop closure damp application ===")
print(f"ok=1 (applied): {ok1}    ok=0 (skipped): {ok0}    apply rate: {100*ok1/max(1,ok1+ok0):.1f}%")

# ── SLAM promote details ───────────────────────────────────────────────────
PROM_RE = re.compile(r"SLAM promote fid=(\d+) slot=(\d+) rms=([\d.]+)")
proms = []
with open(WALK_LOG, encoding='utf-8', errors='replace') as f:
    for line in f:
        m = PROM_RE.search(line)
        if m:
            ts_match = re.match(r"^(\S+ \S+)", line)
            proms.append((ts_match.group(1) if ts_match else '?', int(m.group(1)), int(m.group(2)), float(m.group(3))))
print(f"\n=== SLAM promotions ===")
print(f"count: {len(proms)}")
for t, fid, slot, rms in proms[:15]:
    print(f"  {t}  fid={fid} slot={slot} rms={rms:.2f}px")

# ── SLAM_RMS distribution ───────────────────────────────────────────────────
RMS_RE = re.compile(r"SLAM_RMS .* rms=([\d.]+)px .* du=([-\d.]+) dv=([-\d.]+)")
rms_values = []
with open(WALK_LOG, encoding='utf-8', errors='replace') as f:
    for line in f:
        m = RMS_RE.search(line)
        if m:
            rms_values.append(float(m.group(1)))
print(f"\n=== SLAM_RMS ===")
print(f"count: {len(rms_values)}")
if rms_values:
    print(f"median: {sorted(rms_values)[len(rms_values)//2]:.2f} px")
    print(f"max:    {max(rms_values):.2f} px")

# ── Where in time did SLAM activity happen? ───────────────────────────────
TS_RE = re.compile(r"^(\d{2}-\d{2} \d{2}:\d{2}:\d{2})")
def first_last_match(pat):
    first=None; last=None
    with open(WALK_LOG, encoding='utf-8', errors='replace') as f:
        for line in f:
            if pat in line:
                ts_m = TS_RE.match(line)
                if ts_m:
                    if first is None: first = ts_m.group(1)
                    last = ts_m.group(1)
    return first, last
print("\n=== Event time windows ===")
print(f"SLAM_RMS first/last: {first_last_match('SLAM_RMS')}")
print(f"LC_ABS first/last:   {first_last_match('LC_ABS:')}")
print(f"VIO frame first/last:{first_last_match('NavSight-Tracker:')}")
