"""ASCII trajectory plot of v25 + loop-overlay analysis.

Detects loop closures via temporal clustering of "self-revisits" (later
points that come close to earlier points), then quantifies how well the
second pass overlays the first.
"""
import json, math, sys
from collections import defaultdict

PATH = 'tests/sims/regression/visual/v25_walk_1778687893540.json'

with open(PATH) as f:
    d = json.load(f)
pts = d['points']

# vx = East, vz = North (Y-up JSON post-JNI swap; vy is up)
xs = [p['vx'] for p in pts]
zs = [p['vz'] for p in pts]
ts = [p['ts'] for p in pts]
t0 = ts[0]

# ── ASCII plot (60×30) ──────────────────────────────────────────────────────
W, H = 80, 36
xmin, xmax = min(xs), max(xs)
zmin, zmax = min(zs), max(zs)
pad = 0.5
xmin -= pad; xmax += pad; zmin -= pad; zmax += pad
sx = (W - 1) / max(1e-6, (xmax - xmin))
sz = (H - 1) / max(1e-6, (zmax - zmin))

grid = [[' '] * W for _ in range(H)]

# Plot trajectory with time-coded markers
N = len(pts)
for i, (x, z) in enumerate(zip(xs, zs)):
    col = int((x - xmin) * sx)
    row = H - 1 - int((z - zmin) * sz)
    if 0 <= row < H and 0 <= col < W:
        frac = i / max(1, N - 1)
        # 5 time buckets
        if frac < 0.2:   ch = '1'
        elif frac < 0.4: ch = '2'
        elif frac < 0.6: ch = '3'
        elif frac < 0.8: ch = '4'
        else:            ch = '5'
        # Don't overwrite a denser char with a sparser one
        if grid[row][col] in (' ', '.'):
            grid[row][col] = ch

# Mark start (S) and end (E)
sx_col = int((xs[0]  - xmin) * sx); sx_row = H - 1 - int((zs[0]  - zmin) * sz)
ex_col = int((xs[-1] - xmin) * sx); ex_row = H - 1 - int((zs[-1] - zmin) * sz)
if 0 <= sx_row < H and 0 <= sx_col < W: grid[sx_row][sx_col] = 'S'
if 0 <= ex_row < H and 0 <= ex_col < W: grid[ex_row][ex_col] = 'E'

print(f'v25 walk — x range [{xmin:+.1f},{xmax:+.1f}] m, z range [{zmin:+.1f},{zmax:+.1f}] m')
print(f'time bins: 1=first 20% of walk, 5=last 20%. S=start, E=end')
print('+' + '-'*W + '+')
for row in grid:
    print('|' + ''.join(row) + '|')
print('+' + '-'*W + '+')

# ── Loop-overlap analysis ───────────────────────────────────────────────────
# For each point in the SECOND half of the walk, find its closest distance to
# any point in the FIRST half (after >30s temporal exclusion). Sub-1m means
# loops are overlaying; multi-m means they're offset.
print()
print('=== Loop-overlap analysis (2nd-half points vs 1st-half points) ===')
mid = N // 2
overlaps = []
for i in range(mid, N):
    t_i_s = (ts[i] - t0) / 1000.0
    best = (1e9, -1)
    for j in range(0, mid):
        t_j_s = (ts[j] - t0) / 1000.0
        if t_i_s - t_j_s < 30: continue  # temporal exclusion
        dxy = math.sqrt((xs[i]-xs[j])**2 + (zs[i]-zs[j])**2)
        if dxy < best[0]: best = (dxy, j)
    overlaps.append(best[0])

# Stats
import statistics
print(f'  median nearest-overlap distance : {statistics.median(overlaps):.2f} m')
print(f'  p25 / p75                       : {statistics.quantiles(overlaps, n=4)[0]:.2f} / {statistics.quantiles(overlaps, n=4)[2]:.2f} m')
print(f'  count < 0.5m (good overlap)     : {sum(1 for d in overlaps if d < 0.5)} / {len(overlaps)}')
print(f'  count < 1.0m                    : {sum(1 for d in overlaps if d < 1.0)} / {len(overlaps)}')
print(f'  count < 2.0m                    : {sum(1 for d in overlaps if d < 2.0)} / {len(overlaps)}')

# ── First-pass vs second-pass — find the centroid and "loop count" ──────────
# Compute distance from origin trajectory
print()
print('=== Excursion pattern (radial distance vs time) ===')
last = -1
for i, p in enumerate(pts):
    s = (ts[i] - t0) / 1000.0
    if s - last >= 5:
        r = math.sqrt(p['vx']**2 + p['vz']**2)
        bar = '#' * int(r)
        print(f'  t={s:5.1f}s  r={r:5.2f}m  {bar}')
        last = s
