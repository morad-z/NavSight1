#!/usr/bin/env python3
"""
Walk/Run/U-turn validation analyzer for NavSight sim recordings.

Reads a simulation_data_*.json (schema: top-level 'points' array with
vx,vy,vz = user-facing dot/global_t_, hdg = heading deg, a*,g* = imu, glat/glng = GPS)
and emits a deterministic validation report:

  - motion profile (speed percentiles from dot deltas) -> walk vs run classification
  - trajectory path length / net displacement / endpoint-return ratio (out-and-back)
  - heading profile + U-turn detection (sustained ~180 deg reversal)
  - Fix C symptom: freeze-then-teleport detection (low-motion+high-gyro -> jump)
  - speed-spike census (the reverted 1.78 hardcode produced >8 m/s spikes)
  - GPS availability (jamming check)
  - event_summary counter dump

Usage:
  python scripts/analyze_walk_run_validation.py <sim.json> [--label NAME] [--json-out]

Reusable across future walk/run validations. Pure stdlib.
"""
import json, sys, math, argparse

WALK_MAX_MPS = 2.2          # > this median => not a casual walk
RUN_MIN_MPS  = 2.4          # >= this median => run/jog
SPIKE_MPS    = 8.0          # 28.8 km/h: the old 1.78-hardcode bug signature
FREEZE_MPS   = 0.15         # below this = effectively frozen dot
HIGH_GYRO    = 0.20         # rad/s: matches rotation_dominated gate (gyro_norm>0.2)
JUMP_FACTOR  = 4.0          # a "teleport" = segment > JUMP_FACTOR x median segment


def pctl(xs, p):
    if not xs:
        return float('nan')
    s = sorted(xs)
    k = (len(s) - 1) * p
    lo = int(math.floor(k)); hi = int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi - k) + s[hi] * (k - lo)


def ang_diff(a, b):
    """smallest signed deg difference a-b in (-180,180]"""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d if d != -180.0 else 180.0


def analyze(path, label=None):
    d = json.load(open(path, encoding='utf-8'))
    pts = d.get('points', [])
    es = d.get('event_summary', {})
    fm = d.get('frames_meta', {})
    R = {'file': path, 'label': label or path, 'n_points': len(pts)}
    if len(pts) < 3:
        R['error'] = 'too few points'
        return R

    # --- time ---
    ts = [p['ts'] for p in pts]
    dur_s = (ts[-1] - ts[0]) / 1000.0
    dts = [(ts[i] - ts[i-1]) / 1000.0 for i in range(1, len(ts))]
    dts_pos = [x for x in dts if x > 0]
    R['duration_s'] = round(dur_s, 2)
    R['fps_mean'] = round((len(pts) - 1) / dur_s, 2) if dur_s > 0 else None
    R['dt_ms_median'] = round(pctl(dts_pos, 0.5) * 1000.0, 1) if dts_pos else None

    # --- dot trajectory (vx,vy,vz = global_t_, world Z-up ENU) ---
    P = [(p.get('vx', 0.0), p.get('vy', 0.0), p.get('vz', 0.0)) for p in pts]
    def seg3(i):
        return math.dist(P[i], P[i-1])
    def seg_h(i):  # horizontal (x,y)
        return math.hypot(P[i][0]-P[i-1][0], P[i][1]-P[i-1][1])

    segs3 = [seg3(i) for i in range(1, len(P))]
    path3 = sum(segs3)
    path_h = sum(seg_h(i) for i in range(1, len(P)))
    net3 = math.dist(P[-1], P[0])
    # per-axis travel + extent
    for ax, idx in (('x', 0), ('y', 1), ('z', 2)):
        vals = [pp[idx] for pp in P]
        R[f'axis_{ax}_range_m'] = round(max(vals) - min(vals), 2)
        R[f'axis_{ax}_net_m'] = round(P[-1][idx] - P[0][idx], 2)
    R['path_len_3d_m'] = round(path3, 2)
    R['path_len_horiz_m'] = round(path_h, 2)
    R['net_disp_3d_m'] = round(net3, 2)
    R['endpoint_return_ratio'] = round(net3 / path3, 3) if path3 > 1e-6 else None

    # --- speed from dot deltas ---
    spd = []
    for i in range(1, len(P)):
        dt = (ts[i] - ts[i-1]) / 1000.0
        if dt > 1e-3:
            spd.append(seg3(i) / dt)
    R['speed_mps'] = {
        'mean': round(sum(spd)/len(spd), 3) if spd else None,
        'median': round(pctl(spd, 0.5), 3),
        'p90': round(pctl(spd, 0.9), 3),
        'p95': round(pctl(spd, 0.95), 3),
        'max': round(max(spd), 3) if spd else None,
    }
    R['speed_kmh_median'] = round(pctl(spd, 0.5) * 3.6, 2)
    R['speed_kmh_max'] = round(max(spd) * 3.6, 2) if spd else None
    R['spike_frames_gt_8mps'] = sum(1 for s in spd if s > SPIKE_MPS)
    med_spd = pctl(spd, 0.5)
    if med_spd < 0.3:
        R['motion_class'] = 'near-stationary/frozen'
    elif med_spd <= WALK_MAX_MPS:
        R['motion_class'] = 'walk'
    elif med_spd >= RUN_MIN_MPS:
        R['motion_class'] = 'run/jog'
    else:
        R['motion_class'] = 'brisk-walk/ambiguous'

    # --- heading + U-turn ---
    # NOTE: the recorded 'hdg' field is in RADIANS (range +/-pi). Normalize to
    # degrees up front so the deg-based ang_diff() below is correct. (Treating
    # radians as degrees made a 180deg U-turn look like an 11deg wiggle.)
    hdg = [p.get('hdg') for p in pts]
    hdg = [h for h in hdg if h is not None]
    if hdg and max(abs(h) for h in hdg) <= math.pi + 0.05:
        hdg = [math.degrees(h) for h in hdg]
    if len(hdg) >= 3:
        # cumulative absolute turning + net heading swing via running unwrap
        cum = 0.0
        unwrapped = [hdg[0]]
        for i in range(1, len(hdg)):
            dd = ang_diff(hdg[i], hdg[i-1])
            cum += abs(dd)
            unwrapped.append(unwrapped[-1] + dd)
        R['heading_total_turn_deg'] = round(cum, 1)
        R['heading_net_swing_deg'] = round(max(unwrapped) - min(unwrapped), 1)
        # U-turn = a window where heading reverses by >=150 deg and stays reversed
        h0 = hdg[0]
        max_rev = 0.0; rev_at = None
        for i in range(len(hdg)):
            rev = abs(ang_diff(hdg[i], h0))
            if rev > max_rev:
                max_rev = rev; rev_at = i
        R['heading_max_reversal_vs_start_deg'] = round(max_rev, 1)
        R['heading_max_reversal_at_frac'] = round(rev_at / (len(hdg)-1), 2) if rev_at else None
        R['uturn_detected'] = bool(max_rev >= 150.0)
    else:
        R['heading_total_turn_deg'] = None
        R['uturn_detected'] = None

    # --- Fix C: freeze-then-teleport (low motion + high gyro -> jump next frames) ---
    gnorm = [math.sqrt(p.get('gx',0)**2 + p.get('gy',0)**2 + p.get('gz',0)**2) for p in pts]
    med_seg = pctl([s for s in segs3 if s > 0], 0.5) if any(s>0 for s in segs3) else 0.0
    jump_thr = max(JUMP_FACTOR * med_seg, 0.5)
    teleports = []
    frozen_hi_gyro = 0
    for i in range(1, len(P)):
        if gnorm[i] > HIGH_GYRO and (spd[i-1] if i-1 < len(spd) else 0) < FREEZE_MPS:
            frozen_hi_gyro += 1
        if segs3[i-1] > jump_thr:
            teleports.append({'frame': i, 'jump_m': round(segs3[i-1], 2),
                              'gyro': round(gnorm[i], 2),
                              'frac': round(i/(len(P)-1), 2)})
    R['median_segment_m'] = round(med_seg, 3)
    R['teleport_jump_thresh_m'] = round(jump_thr, 2)
    R['teleport_count'] = len(teleports)
    R['teleports'] = teleports[:10]
    R['frozen_while_turning_frames'] = frozen_hi_gyro
    R['max_gyro_rads'] = round(max(gnorm), 3)

    # --- GPS availability (jamming check) ---
    gps_ok = sum(1 for p in pts if p.get('glat') is not None and p.get('glng') is not None)
    R['gps_fix_frac'] = round(gps_ok / len(pts), 3)

    # --- event summary (key counters) ---
    keep = ['ba_solves_total', 'rot_gate_pure_rot_confirmed', 'klt_adaptive_window_hits',
            'blur_total_skip_frames', 'lowlight_state_active_frames',
            'reloc_orb_accepts', 'reloc_orb_rejects']
    R['event_summary'] = {k: es.get(k) for k in es}
    R['frames_meta'] = fm
    return R


def fmt(R):
    L = []
    L.append(f"\n{'='*70}\n{R['label']}  ({R['file'].split('/')[-1]})\n{'='*70}")
    if R.get('error'):
        L.append("  ERROR: " + R['error']); return '\n'.join(L)
    L.append(f"  points={R['n_points']}  dur={R['duration_s']}s  fps={R['fps_mean']}  dt_med={R['dt_ms_median']}ms  gps_fix={R['gps_fix_frac']}")
    sp = R['speed_mps']
    L.append(f"  SPEED  median={sp['median']} m/s ({R['speed_kmh_median']} km/h)  p95={sp['p95']}  max={sp['max']} ({R['speed_kmh_max']} km/h)")
    L.append(f"         class={R['motion_class']}   spikes>8m/s={R['spike_frames_gt_8mps']}")
    L.append(f"  PATH   3d={R['path_len_3d_m']}m  horiz={R['path_len_horiz_m']}m  net={R['net_disp_3d_m']}m  return_ratio={R['endpoint_return_ratio']}")
    L.append(f"  AXES   x_range={R['axis_x_range_m']} y_range={R['axis_y_range_m']} z_range={R['axis_z_range_m']}  (net x={R['axis_x_net_m']} y={R['axis_y_net_m']} z={R['axis_z_net_m']})")
    L.append(f"  HEAD   total_turn={R['heading_total_turn_deg']}deg  max_reversal_vs_start={R.get('heading_max_reversal_vs_start_deg')}deg @frac={R.get('heading_max_reversal_at_frac')}  UTURN={R['uturn_detected']}")
    L.append(f"  FIXC   teleports(>{R['teleport_jump_thresh_m']}m)={R['teleport_count']}  frozen_while_turning={R['frozen_while_turning_frames']}  max_gyro={R['max_gyro_rads']}rad/s  med_seg={R['median_segment_m']}m")
    if R['teleports']:
        L.append("         jumps: " + ', '.join(f"f{t['frame']}({t['jump_m']}m,g{t['gyro']},@{t['frac']})" for t in R['teleports']))
    es = R['event_summary']
    nz = {k: v for k, v in es.items() if v not in (0, None)}
    L.append(f"  EVENTS nonzero: {nz}")
    return '\n'.join(L)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('sims', nargs='+')
    ap.add_argument('--label', action='append', default=[])
    ap.add_argument('--json-out', action='store_true')
    a = ap.parse_args()
    out = []
    for i, s in enumerate(a.sims):
        lbl = a.label[i] if i < len(a.label) else None
        R = analyze(s, lbl)
        out.append(R)
        print(fmt(R))
    if a.json_out:
        print("\n===JSON===")
        print(json.dumps(out))
