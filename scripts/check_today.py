import json, os
files = ['sim_walk_today.json', 'sim_run_today.json', 'sim_walkrun_today.json']
look_keys = [
    'depth_flow_updates', 'depth_flow_calib_updates', 'depth_flow_skipped_few_pts',
    'depth_flow_outlier_rejected', 'midas_scale_k_milli',
    'depth_flow_looming_updates', 'depth_flow_looming_used', 'depth_flow_looming_skipped',
    'global_t_gated_rotation_dominated_total',
    'velocity_clamped', 'total_path_dm',
    'is_static_total', 'rotation_dominated_total',
    'pose_valid_total', 'translation_degenerate_total', 'pure_rotation_total',
    'midas_fused', 'midas_skipped', 'midas_inferred', 'midas_affine_fit_attempted',
    'scale_observer_updates', 'scale_a_updates', 'scale_b_updates',
    'cnt_klt_tracked_points_total',
]
for fn in files:
    if not os.path.exists(fn): continue
    with open(fn) as f:
        d = json.load(f)
    es = d.get('event_summary') or {}
    pts = d.get('data_points') or d.get('dataPoints') or d.get('simulationData') or []
    print(f'=== {fn} ===')
    print(f'  top-level keys = {list(d.keys())[:20]}')
    print(f'  frames = {len(pts)}')
    print('  event_summary subset:')
    for k in look_keys:
        if k in es:
            print(f'    {k} = {es[k]}')
    extras = sorted(k for k in es.keys() if 'depth' in k.lower() or 'static' in k.lower() or 'rotation' in k.lower() or 'gated' in k.lower())
    print('  related keys present:')
    for k in extras:
        if k not in look_keys:
            print(f'    {k} = {es[k]}')
    print()
