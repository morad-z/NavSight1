"""Step A validation analysis for the 3 v5 recordings (2026-05-29).

Per the handoff checklist:
  - vsc (= scale_fuser) per recording: ~0.2 => Observer C broken; ~1.0 => fix affine target
  - VIO dist vs GPS dist
  - event_summary: K, depth_flow_updates, looming, midas_affine_fit_inlier_ratio + shift
  - VIO speed series (1 s windows) vs GPS speed
"""

import json
import math
import os

FILES = [
    ("v7_walk (19:36)", "sim_v7_walk.json"),
    ("v7_run (19:37)", "sim_v7_run.json"),
    ("v7_walkrun(19:38)", "sim_v7_walkrun.json"),
]


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def analyze(label, fn):
    if not os.path.exists(fn):
        print(f"{label}: MISSING ({fn})")
        return
    with open(fn) as f:
        d = json.load(f)
    pts = d.get("points") or []
    es = d.get("event_summary") or {}
    if not pts:
        print(f"{label}: no points")
        return

    ts_key = "ts" if "ts" in pts[0] else "timestamp"
    t0 = pts[0][ts_key]
    dur = (pts[-1][ts_key] - t0) / 1000.0

    # VIO distance (cumulative + net)
    vio_dist, prev = 0.0, None
    first_xz = last_xz = None
    vsc_vals = []
    for p in pts:
        x, z = p.get("vx"), p.get("vz")
        if "vsc" in p and p["vsc"] is not None:
            vsc_vals.append(float(p["vsc"]))
        if x is None or z is None:
            continue
        if first_xz is None:
            first_xz = (x, z)
        last_xz = (x, z)
        if prev is not None:
            vio_dist += math.hypot(x - prev[0], z - prev[1])
        prev = (x, z)
    vio_net = math.hypot(last_xz[0] - first_xz[0], last_xz[1] - first_xz[1]) if first_xz else 0.0

    # GPS
    gps = [(p[ts_key], p["glat"], p["glng"], p.get("gacc"))
           for p in pts if p.get("glat") is not None and p.get("glng") is not None
           and (p.get("gacc") is None or p.get("gacc") <= 30)]
    gps_dist = sum(haversine_m(gps[i-1][1], gps[i-1][2], gps[i][1], gps[i][2])
                   for i in range(1, len(gps)))
    gps_net = haversine_m(gps[0][1], gps[0][2], gps[-1][1], gps[-1][2]) if len(gps) >= 2 else 0.0

    # VIO 1 s speed windows
    win_ms, cur_t0, cur_d, cur_prev, wins = 1000, pts[0][ts_key], 0.0, None, []
    for p in pts:
        x, z = p.get("vx"), p.get("vz")
        if x is None or z is None:
            continue
        if cur_prev is not None:
            cur_d += math.hypot(x - cur_prev[0], z - cur_prev[1])
        cur_prev = (x, z)
        if p[ts_key] - cur_t0 >= win_ms:
            dt = (p[ts_key] - cur_t0) / 1000.0
            if dt > 0:
                wins.append(cur_d / dt * 3.6)
            cur_t0, cur_d = p[ts_key], 0.0
    nz = [s for s in wins if s > 0.1]

    # GPS speed
    gps_sp = []
    for i in range(1, len(gps)):
        dt = (gps[i][0] - gps[i-1][0]) / 1000.0
        if dt > 0:
            gps_sp.append(haversine_m(gps[i-1][1], gps[i-1][2], gps[i][1], gps[i][2]) / dt * 3.6)
    gps_nz = [s for s in gps_sp if s > 0.1]

    vsc_med = sorted(vsc_vals)[len(vsc_vals)//2] if vsc_vals else float("nan")
    vsc_min = min(vsc_vals) if vsc_vals else float("nan")
    vsc_max = max(vsc_vals) if vsc_vals else float("nan")

    print(f"=== {label} ===  ({len(pts)} frames, {dur:.1f}s, {len(gps)} GPS fixes)")
    print(f"  vsc (scale_fuser)  med={vsc_med:.3f}  range=[{vsc_min:.3f}, {vsc_max:.3f}]   <-- ~0.2=ObsC broken, ~1.0=fix affine target")
    print(f"  VIO dist  {vio_dist:6.2f} m   net {vio_net:6.2f} m")
    print(f"  GPS dist  {gps_dist:6.2f} m   net {gps_net:6.2f} m"
          + (f"   ratio {vio_dist/gps_dist:.2f}x" if gps_dist > 0.5 else "   (GPS unusable)"))
    if nz:
        print(f"  VIO 1s spd  mean {sum(nz)/len(nz):5.2f}  p50 {sorted(nz)[len(nz)//2]:5.2f}  max {max(wins):6.2f} km/h  ({len(nz)*100//len(wins)}% nz)")
    if gps_nz:
        print(f"  GPS spd     mean {sum(gps_nz)/len(gps_nz):5.2f}  p50 {sorted(gps_nz)[len(gps_nz)//2]:5.2f}  max {max(gps_sp):6.2f} km/h")
    print(f"  --- event_summary ---")
    print(f"  K          {es.get('midas_scale_k_milli',0)/1000:.1f}  (min {es.get('midas_scale_k_min_milli',0)/1000:.1f}, max {es.get('midas_scale_k_max_milli',0)/1000:.1f})")
    print(f"  affine     shift_t={es.get('midas_affine_fit_shift_milli',0)/1000:.4f}  inlier_ratio={es.get('midas_affine_fit_inlier_ratio_milli',0)/1000:.3f}  singular={es.get('midas_affine_fit_singular',0)}  low_inliers={es.get('midas_affine_fit_low_inliers',0)}")
    print(f"  depth_flow updates={es.get('depth_flow_updates',0)} calib={es.get('depth_flow_calib_updates',0)} outlier_rej={es.get('depth_flow_outlier_rejected',0)} total_mm={es.get('depth_flow_total_mm',0)}")
    print(f"  looming    updates={es.get('depth_flow_looming_updates',0)} used={es.get('depth_flow_looming_used',0)} skipped={es.get('depth_flow_looming_skipped',0)}")
    print(f"  midas      entries={es.get('midas_entries',0)} fused={es.get('midas_fused',0)} bailout_no_depth={es.get('midas_bailout_no_depth',0)} bailout_few_pts3d={es.get('midas_bailout_few_pts3d',0)}")
    print(f"  total_path_dm={es.get('total_path_dm',0)/10:.1f} m   velocity_clamped={es.get('velocity_clamped',0)}")
    print()


for label, fn in FILES:
    analyze(label, fn)
