"""Playbook B: cluster trajectory jumps + cross-reference with LC accepts.
Per navsight-sim-debugging skill: characterise corrections as toward-origin
(good) vs away-from-origin (thrashing)."""
import json, math, re

JSON = 'tests/sims/regression/visual/v24_walk_1778683492977.json'
LOG  = 'tests/sims/regression/visual/v24_walk_1778683492977.logcat.txt'

with open(JSON) as f:
    d = json.load(f)
pts = d['points']
N = len(pts)

# Playbook B Step 1+2: find jump clusters
clusters = []
cur = []
for i in range(1, N):
    a, b = pts[i-1], pts[i]
    dx = b['vx']-a['vx']; dy = b['vy']-a['vy']; dz = b['vz']-a['vz']
    d3 = math.sqrt(dx*dx + dy*dy + dz*dz)
    if d3 > 0.15:   # lowered: damped corrections are ~0.05–0.5m per frame
        cur.append({'i': i, 'd': d3, 'ts': b['ts']})
    else:
        if len(cur) >= 2:
            clusters.append(cur)
        cur = []
if len(cur) >= 2:
    clusters.append(cur)

print(f'Trajectory clusters (>0.4m jumps, ≥2 consecutive frames): {len(clusters)}')
print()
print(f'{"cluster":>7} {"ts_s":>7} {"|pre|":>7} {"|post|":>7} {"delta":>8} {"verdict":<20}')
toward = away = 0
for ci, cluster in enumerate(clusters):
    pre  = pts[cluster[0]['i'] - 1]
    post = pts[cluster[-1]['i']]
    pre_mag  = math.sqrt(pre['vx']**2 + pre['vy']**2 + pre['vz']**2)
    post_mag = math.sqrt(post['vx']**2 + post['vy']**2 + post['vz']**2)
    delta = post_mag - pre_mag
    verdict = 'TOWARD origin (good)' if delta < 0 else 'AWAY from origin (bad)'
    if delta < 0:
        toward += 1
    else:
        away += 1
    t_s = (post['ts'] - pts[0]['ts']) / 1000.0
    print(f'{ci:>7} {t_s:>7.1f} {pre_mag:>7.2f} {post_mag:>7.2f} {delta:>+8.2f} {verdict:<20}')
print()
print(f'Summary: toward={toward}  away={away}  thrashing_ratio={away/max(1,toward+away):.0%}')

# Step B4: cross-reference with LOOP_CLOSURE: ACCEPT lines
print()
print('=== All LOOP_CLOSURE: ACCEPT events ===')
accept_re = re.compile(r'(\d\d:\d\d:\d\d\.\d\d\d).*LOOP_CLOSURE: ACCEPT(?:\s+\((geom)\))?\s+now_kf=(\d+)\s+match_kf=(\d+).*?(?:bow=([\d.]+))?.*?(?:pnp_inl|inl)=(\d+)')
accepts = []
with open(LOG, encoding='utf-16-le', errors='replace') as f:
    for line in f:
        m = accept_re.search(line)
        if m:
            accepts.append({
                't': m.group(1),
                'path': m.group(2) or 'BoW',
                'now_kf': int(m.group(3)),
                'match_kf': int(m.group(4)),
                'bow': m.group(5),
                'inl': int(m.group(6)),
            })
for a in accepts:
    print(f'  {a["t"]}  {a["path"]:>4}  now_kf={a["now_kf"]}  match_kf={a["match_kf"]}  inl={a["inl"]}  bow={a["bow"]}')

# Aggregate LC_ABS by accept event
print()
print('=== LC_ABS per accept (first 3 lines of each ramp) ===')
abs_re = re.compile(r'(\d\d:\d\d:\d\d\.\d\d\d).*LC_ABS: r_R=\[([^\]]+)\] r_p=\[([^\]]+)\] p_G=\[([^\]]+)\] target_p=\[([^\]]+)\].*?m2=([\d.]+) m2_R=([\d.]+) m2_p=([\d.]+)')
abs_lines = []
with open(LOG, encoding='utf-16-le', errors='replace') as f:
    for line in f:
        m = abs_re.search(line)
        if m:
            abs_lines.append({
                't': m.group(1),
                'r_R': [float(x) for x in m.group(2).split()],
                'r_p': [float(x) for x in m.group(3).split()],
                'p_G': [float(x) for x in m.group(4).split()],
                'target_p': [float(x) for x in m.group(5).split()],
                'm2': float(m.group(6)),
                'm2_R': float(m.group(7)),
                'm2_p': float(m.group(8)),
            })

# Group LC_ABS lines into ramps (gaps > 0.5s = new ramp)
ramps = []
cur_r = []
prev_t = None
def to_ms(t):
    hh, mm, sec = t.split(':')
    return int(hh)*3600000 + int(mm)*60000 + int(float(sec)*1000)
for a in abs_lines:
    t = to_ms(a['t'])
    if prev_t is None or t - prev_t < 500:
        cur_r.append(a)
    else:
        if cur_r: ramps.append(cur_r)
        cur_r = [a]
    prev_t = t
if cur_r: ramps.append(cur_r)

for i, r in enumerate(ramps):
    first = r[0]
    rR_norm = math.sqrt(sum(x*x for x in first['r_R']))
    rp_norm = math.sqrt(sum(x*x for x in first['r_p']))
    n_pass = sum(1 for x in r if x['m2'] < 22.5)
    print(f'  ramp {i}: t={first["t"]}  N={len(r):>2}  passes_chi2={n_pass:>2}/{len(r):>2}  '
          f'|r_R|={rR_norm*180/math.pi:>5.1f}°  |r_p|={rp_norm:>5.2f}m  '
          f'm2={first["m2"]:>6.1f} m2_R={first["m2_R"]:>5.2f} m2_p={first["m2_p"]:>5.2f}')

# Per-keyframe pts3d_world fill statistics
print()
print('=== Stored keyframe 3D-point fill quality (LC_KF lines) ===')
lckf_re = re.compile(r'LC_KF: kp=(\d+) filled_3d=(\d+) triangulated=(\d+) tri_baseline=([\d.]+)m')
lckf = []
with open(LOG, encoding='utf-16-le', errors='replace') as f:
    for line in f:
        m = lckf_re.search(line)
        if m:
            lckf.append({
                'kp': int(m.group(1)),
                'filled_3d': int(m.group(2)),
                'triangulated': int(m.group(3)),
                'baseline_m': float(m.group(4)),
            })
print(f'  total keyframes stored: {len(lckf)}')
if lckf:
    filled = [k['filled_3d'] for k in lckf]
    tri    = [k['triangulated'] for k in lckf]
    bsl    = [k['baseline_m'] for k in lckf]
    print(f'  filled_3d (SLAM-feature points): med={sorted(filled)[len(filled)//2]}  '
          f'p95={sorted(filled)[int(0.95*len(filled))]}  max={max(filled)}')
    print(f'  triangulated (LC-internal tri):  med={sorted(tri)[len(tri)//2]}  '
          f'p95={sorted(tri)[int(0.95*len(tri))]}  max={max(tri)}')
    print(f'  tri_baseline_m: med={sorted(bsl)[len(bsl)//2]:.2f}  '
          f'p95={sorted(bsl)[int(0.95*len(bsl))]:.2f}  max={max(bsl):.2f}')
    zero_f = sum(1 for k in lckf if k['filled_3d'] == 0)
    zero_t = sum(1 for k in lckf if k['triangulated'] == 0)
    print(f'  keyframes with filled_3d=0: {zero_f}/{len(lckf)}  ({100*zero_f/len(lckf):.0f}%)')
    print(f'  keyframes with triangulated=0: {zero_t}/{len(lckf)}  ({100*zero_t/len(lckf):.0f}%)')
