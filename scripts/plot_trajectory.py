#!/usr/bin/env python3
"""Plot a NavSight sim's user-facing dot ground-track (vx vs vz; vy is up) with
teleport jumps marked. Reusable for trajectory-blow-up investigations.

Usage: python scripts/plot_trajectory.py <sim.json> [--out path.png] [--jump 0.5]
"""
import json, sys, math, argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument('sim')
ap.add_argument('--out', default=None)
ap.add_argument('--jump', type=float, default=0.5)
a = ap.parse_args()

d = json.load(open(a.sim, encoding='utf-8'))
pts = d['points']
x = [p['vx'] for p in pts]; y = [p['vy'] for p in pts]; z = [p['vz'] for p in pts]
ts = [p['ts'] for p in pts]
n = len(pts)
# ground plane = (x, z) per the Y-up recording convention; vy = up
segs = [math.dist((x[i], y[i], z[i]), (x[i-1], y[i-1], z[i-1])) for i in range(1, n)]
jumps = [i for i in range(1, n) if segs[i-1] > a.jump]

fig, ax = plt.subplots(1, 2, figsize=(15, 7))
# left: ground track colored by frame index
sc = ax[0].scatter(x, z, c=range(n), cmap='viridis', s=8)
ax[0].plot(x, z, '-', lw=0.4, alpha=0.4, color='gray')
ax[0].scatter([x[0]], [z[0]], c='lime', s=120, marker='o', label='start', zorder=5, edgecolor='k')
ax[0].scatter([x[-1]], [z[-1]], c='red', s=120, marker='X', label='end', zorder=5, edgecolor='k')
for i in jumps:
    ax[0].plot([x[i-1], x[i]], [z[i-1], z[i]], 'r-', lw=1.5, alpha=0.7)
    ax[0].annotate(f'{segs[i-1]:.1f}m', (x[i], z[i]), fontsize=6, color='red')
ax[0].set_title(f'Ground track (vx vs vz)  n={n}  path={sum(segs):.1f}m  net={math.dist((x[-1],y[-1],z[-1]),(x[0],y[0],z[0])):.1f}m\n{len(jumps)} teleports >{a.jump}m (red)')
ax[0].set_xlabel('vx (m, East)'); ax[0].set_ylabel('vz (m, fwd/North)'); ax[0].axis('equal'); ax[0].legend()
plt.colorbar(sc, ax=ax[0], label='frame')
# right: per-frame segment size over time (the teleport timeline)
ax[1].plot([(t-ts[0])/1000 for t in ts[1:]], segs, '-', lw=0.6)
ax[1].axhline(a.jump, color='r', ls='--', lw=0.8, label=f'{a.jump}m teleport thresh')
ax[1].set_title('Per-frame displacement over time (spikes = teleports)')
ax[1].set_xlabel('t (s)'); ax[1].set_ylabel('segment (m)'); ax[1].legend()

out = a.out or a.sim.rsplit('.', 1)[0] + '_traj.png'
plt.tight_layout(); plt.savefig(out, dpi=110)
print('wrote', out)
print(f'path={sum(segs):.1f}m teleports={len(jumps)} max_seg={max(segs):.2f}m '
      f'vy(up)_range={max(y)-min(y):.2f}m')
