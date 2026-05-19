"""ASCII trajectory plot + shape metrics for v22 / v23.16 / v24."""
import json, math

WALKS = [
    ('v22',    'tests/sims/regression/visual/v22_verification_walk.json'),
    ('v23.16', 'tests/sims/regression/visual/v23_16_walk_1778599109970.json'),
    ('v24',    'tests/sims/regression/visual/v24_walk_1778683492977.json'),
]

def load_xz(path):
    with open(path) as f:
        d = json.load(f)
    # vx = East, vz = North (Y-up convention in JSON post-JNI swap)
    return [(p['vx'], p['vz'], p.get('hdg', 0.0), p['ts']) for p in d['points']]


def shape_metrics(pts):
    N = len(pts)
    if N < 2:
        return {}
    # Per-segment displacement and time delta
    seg_lens = []
    seg_dts  = []
    speeds   = []
    head_changes = []
    for i in range(1, N):
        dx = pts[i][0] - pts[i-1][0]
        dz = pts[i][1] - pts[i-1][1]
        seg = math.sqrt(dx*dx + dz*dz)
        dt  = (pts[i][3] - pts[i-1][3]) / 1000.0
        seg_lens.append(seg)
        seg_dts.append(dt)
        if dt > 1e-6:
            speeds.append(seg / dt)
        # Heading change, wrapped
        dh = pts[i][2] - pts[i-1][2]
        while dh >  math.pi: dh -= 2*math.pi
        while dh < -math.pi: dh += 2*math.pi
        head_changes.append(abs(dh))

    path = sum(seg_lens)
    cl   = math.sqrt((pts[-1][0]-pts[0][0])**2 + (pts[-1][1]-pts[0][1])**2)
    max_r = max(math.sqrt(x*x + z*z) for x, z, _, _ in pts)

    # Per-frame jitter: count "direction reversals" (consecutive segments
    # whose dot product is negative — i.e., camera went one way then back).
    reversals = 0
    for i in range(1, len(seg_lens)):
        if seg_lens[i] < 1e-3 or seg_lens[i-1] < 1e-3:
            continue
        # Two consecutive segment vectors
        ax = pts[i][0] - pts[i-1][0]; az = pts[i][1] - pts[i-1][1]
        bx = pts[i+1][0] - pts[i][0] if i+1 < N else 0
        bz = pts[i+1][1] - pts[i][1] if i+1 < N else 0
        if i+1 >= N:
            break
        dot = ax*bx + az*bz
        if dot < 0:
            reversals += 1

    speeds_sorted = sorted(speeds) if speeds else [0]
    return {
        'n_points': N,
        'duration_s': (pts[-1][3] - pts[0][3]) / 1000.0,
        'path_m': path,
        'close_loop_m': cl,
        'max_radius_m': max_r,
        # Tortuosity: path / (2 * max_radius). For a perfect circular loop
        # ratio ≈ π. For random walk inside a bounded area, much higher.
        'tortuosity': path / max(2 * max_r, 1e-3),
        # Median speed (m/s). Walking is 1-1.5 m/s.
        'median_speed_mps': speeds_sorted[len(speeds_sorted)//2],
        'p95_speed_mps':    speeds_sorted[int(0.95 * len(speeds_sorted))],
        'max_speed_mps':    max(speeds) if speeds else 0,
        # Number of frames where speed > 3 m/s (impossible for walking,
        # indicates state snap)
        'snap_count': sum(1 for s in speeds if s > 3.0),
        # Per-frame heading change rate (median rad/frame)
        'median_dhdg_rad': sorted(head_changes)[len(head_changes)//2],
        # Total heading change (unwrapped)
        'total_hdg_change_revs': sum(head_changes) / (2 * math.pi),
        'direction_reversals': reversals,
    }


def ascii_plot(name, pts, width=60, height=20):
    """Render trajectory as ASCII over a width × height grid."""
    if not pts:
        return ['<empty>']
    xs = [p[0] for p in pts]
    zs = [p[1] for p in pts]
    minx, maxx = min(xs), max(xs)
    minz, maxz = min(zs), max(zs)
    rangex = max(maxx - minx, 0.1)
    rangez = max(maxz - minz, 0.1)
    # Square aspect — pick the larger range
    r = max(rangex, rangez) * 1.05
    cx = (minx + maxx) / 2
    cz = (minz + maxz) / 2
    grid = [[' '] * width for _ in range(height)]
    for i, (x, z, _, _) in enumerate(pts):
        # Map (x, z) → (col, row).  East increases right; North up.
        col = int((x - (cx - r/2)) / r * (width - 1))
        row = int(((cz + r/2) - z) / r * (height - 1))
        if 0 <= col < width and 0 <= row < height:
            # Markers: . = generic point, S = start, E = end
            if i == 0:
                grid[row][col] = 'S'
            elif i == len(pts) - 1:
                grid[row][col] = 'E'
            elif grid[row][col] == ' ':
                grid[row][col] = '.'
    out = [f'{name} — range East {minx:.1f}..{maxx:.1f}m  North {minz:.1f}..{maxz:.1f}m']
    out.append('+' + '-' * width + '+')
    for r in grid:
        out.append('|' + ''.join(r) + '|')
    out.append('+' + '-' * width + '+')
    return out


# Run all three
for name, path in WALKS:
    try:
        pts = load_xz(path)
    except FileNotFoundError:
        print(f'{name}: not found')
        continue
    m = shape_metrics(pts)
    print(f'\n=== {name} ===')
    print(f'  duration={m["duration_s"]:.1f}s   path={m["path_m"]:.1f}m   close_loop={m["close_loop_m"]:.2f}m')
    print(f'  max_radius={m["max_radius_m"]:.2f}m   tortuosity(path/2R)={m["tortuosity"]:.2f}  (π≈3.14 for clean loop)')
    print(f'  speed   median={m["median_speed_mps"]:.2f}m/s   P95={m["p95_speed_mps"]:.2f}m/s   max={m["max_speed_mps"]:.2f}m/s')
    print(f'  snap_count (speed>3m/s): {m["snap_count"]}')
    print(f'  heading total_rotations={m["total_hdg_change_revs"]:.2f}   direction_reversals={m["direction_reversals"]}')

print('\n')
for name, path in WALKS:
    try:
        pts = load_xz(path)
    except FileNotFoundError:
        continue
    for line in ascii_plot(name, pts):
        print(line)
    print()
