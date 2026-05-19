"""Side-by-side trajectory + counter comparison: v23.16 vs v24."""
import json, math

WALKS = [
    ('v22_baseline', 'tests/sims/regression/visual/v22_verification_walk.json'),
    ('v23.16',       'tests/sims/regression/visual/v23_16_walk_1778599109970.json'),
    ('v24',          'tests/sims/regression/visual/v24_walk_1778683492977.json'),
]

def load(name, path):
    try:
        with open(path) as f:
            return name, json.load(f)
    except FileNotFoundError:
        return name, None

walks = [load(n, p) for n, p in WALKS]

def trajectory_stats(d):
    pts = d['points']
    es = d['event_summary']
    # JSON convention: vx=East, vy=Up, vz=North (Y-up post-JNI swap)
    # GPS gx,gy,gz same Y-up swap.
    xs = [p['vx'] for p in pts]
    ys = [p['vy'] for p in pts]
    zs = [p['vz'] for p in pts]
    gxs = [p['gx'] for p in pts]
    gzs = [p['gz'] for p in pts]
    # Total horizontal path
    path = sum(math.sqrt((pts[i+1]['vx']-pts[i]['vx'])**2 +
                          (pts[i+1]['vz']-pts[i]['vz'])**2)
                for i in range(len(pts)-1))
    # End-to-start close-loop (horizontal)
    cl = math.sqrt((pts[-1]['vx']-pts[0]['vx'])**2 +
                    (pts[-1]['vz']-pts[0]['vz'])**2)
    # Max radius from origin
    max_r = max(math.sqrt(p['vx']**2 + p['vz']**2) for p in pts)
    duration = (pts[-1]['ts'] - pts[0]['ts']) / 1000.0
    return {
        'n_points': len(pts),
        'duration_s': duration,
        'vio_extent_east_m': max(xs) - min(xs),
        'vio_extent_up_m':   max(ys) - min(ys),
        'vio_extent_north_m': max(zs) - min(zs),
        'gps_extent_east_m':  max(gxs) - min(gxs),
        'gps_extent_north_m': max(gzs) - min(gzs),
        'vio_path_m': path,
        'vio_close_loop_m': cl,
        'vio_max_radius_m': max_r,
        'drift_pct': 100.0 * cl / max(path, 1e-3),
        # Counters
        'lc_attempts': es.get('loop_closure_attempts', 0),
        'lc_accepts_bow': es.get('loop_closure_accepts', 0),
        'lc_accepts_geom': es.get('loop_closure_geom_accepts', 0),
        'lc_corrections': es.get('loop_closure_corrections_applied', 0),
        'lc_chi2_rejected': es.get('loop_closure_chi2_rejected', 0),
        'slam_promotions': es.get('slam_promotions_total', 0),
        'slam_reanchor': es.get('slam_reanchor_total', 0),
        'msckf_lines': es.get('msckf_update_lines', 0),
        'msckf_huber_rejected': es.get('msckf_huber_rejected_sum', 0),
        'pose_graph_optimize': es.get('pose_graph_optimize_calls', 0),
        'pose_graph_apply': es.get('pose_graph_apply_calls', 0),
        'pose_graph_pre_mm': es.get('pose_graph_residual_norm_pre_mm', 0),
        'pose_graph_post_mm': es.get('pose_graph_residual_norm_post_mm', 0),
        'pose_graph_max_corr_mm': es.get('pose_graph_max_correction_mm', 0),
    }

stats = []
for name, d in walks:
    if d is None:
        print(f'{name}: file not found')
        continue
    s = trajectory_stats(d)
    s['_name'] = name
    stats.append(s)

# Print side-by-side
metrics = [
    ('n_points', '%d'),
    ('duration_s', '%.1f s'),
    ('vio_path_m', '%.1f m'),
    ('vio_close_loop_m', '%.2f m'),
    ('drift_pct', '%.2f%%'),
    ('vio_max_radius_m', '%.1f m'),
    ('vio_extent_east_m', '%.1f m'),
    ('vio_extent_north_m', '%.1f m'),
    ('vio_extent_up_m', '%.2f m'),
    ('gps_extent_east_m', '%.2f m'),
    ('gps_extent_north_m', '%.2f m'),
    ('---', ''),
    ('lc_attempts', '%d'),
    ('lc_accepts_bow', '%d'),
    ('lc_accepts_geom', '%d'),
    ('lc_corrections', '%d'),
    ('lc_chi2_rejected', '%d'),
    ('---', ''),
    ('slam_promotions', '%d'),
    ('slam_reanchor', '%d'),
    ('msckf_lines', '%d'),
    ('msckf_huber_rejected', '%d'),
    ('---', ''),
    ('pose_graph_optimize', '%d'),
    ('pose_graph_apply', '%d'),
    ('pose_graph_pre_mm', '%d'),
    ('pose_graph_post_mm', '%d'),
    ('pose_graph_max_corr_mm', '%d'),
]

# Header
names = [s['_name'] for s in stats]
print(f'{"metric":<28}' + ''.join(f'{n:>18}' for n in names))
print('-' * (28 + 18 * len(names)))
for key, fmt in metrics:
    if key == '---':
        print('-' * (28 + 18 * len(names)))
        continue
    label = key
    vals = []
    for s in stats:
        v = s.get(key)
        if v is None:
            vals.append('n/a')
        else:
            try:
                vals.append(fmt % v)
            except TypeError:
                vals.append(str(v))
    print(f'{label:<28}' + ''.join(f'{v:>18}' for v in vals))
