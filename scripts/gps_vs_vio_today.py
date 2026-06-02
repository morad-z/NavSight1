"""Compare GPS vs VIO distance + speed for today's 3 recordings.

GPS distance: cumulative haversine of consecutive valid GPS fixes (acc <= 30 m).
VIO distance: cumulative |Delta(vioX, vioZ)|.
Speed series: stored `speed_kmh` per point (= currentSpeedKmh = getFusedSpeedMps EMA).
"""

import json
import math
import os

FILES = [
    ("v4_a (17:14)      ", "sim_v4_a.json"),
    ("v4_b (17:14)      ", "sim_v4_b.json"),
    ("v4_c (17:16)      ", "sim_v4_c.json"),
]


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
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

    # Find timestamp field (different recordings may use different names)
    ts_key = next((k for k in ("timestamp", "ts") if k in pts[0]), None)
    if ts_key is None:
        print(f"{label}: no timestamp field; keys={list(pts[0])[:15]}")
        return

    t0 = pts[0][ts_key]
    duration_s = (pts[-1][ts_key] - t0) / 1000.0

    # Cumulative VIO distance and start->end straight-line
    vio_dist = 0.0
    prev = None
    first_xz, last_xz = None, None
    for p in pts:
        x, z = p.get("vx"), p.get("vz")
        if x is None or z is None:
            continue
        if first_xz is None:
            first_xz = (x, z)
        last_xz = (x, z)
        if prev is not None:
            vio_dist += math.hypot(x - prev[0], z - prev[1])
        prev = (x, z)
    vio_net = (math.hypot(last_xz[0] - first_xz[0], last_xz[1] - first_xz[1])
               if first_xz and last_xz else 0.0)

    # GPS: cumulative + start->end
    gps_fixes = []
    for p in pts:
        lat, lng, acc = p.get("glat"), p.get("glng"), p.get("gacc")
        if lat is not None and lng is not None and (acc is None or acc <= 30):
            gps_fixes.append((p[ts_key], lat, lng, acc))
    gps_dist = 0.0
    for i in range(1, len(gps_fixes)):
        _, la0, lo0, _ = gps_fixes[i - 1]
        _, la1, lo1, _ = gps_fixes[i]
        gps_dist += haversine_m(la0, lo0, la1, lo1)
    gps_net = (haversine_m(gps_fixes[0][1], gps_fixes[0][2],
                           gps_fixes[-1][1], gps_fixes[-1][2])
               if len(gps_fixes) >= 2 else 0.0)

    # VIO speed from position deltas (cumulative_distance / total_time as avg)
    vio_avg_kmh = (vio_dist / duration_s * 3.6) if duration_s > 0 else 0.0
    # Per-second VIO speed series — sum |dvx,dvz| within each ~1s window
    win_ms = 1000
    cur_t0 = pts[0][ts_key]
    cur_d = 0.0
    cur_prev = None
    win_speeds = []
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
                win_speeds.append(cur_d / dt * 3.6)
            cur_t0 = p[ts_key]
            cur_d = 0.0
    nz = [s for s in win_speeds if s > 0.1]
    spds = win_speeds

    # GPS speed = haversine between consecutive fixes / dt
    gps_spds_kmh = []
    for i in range(1, len(gps_fixes)):
        dt_s = (gps_fixes[i][0] - gps_fixes[i - 1][0]) / 1000.0
        if dt_s <= 0:
            continue
        d_m = haversine_m(gps_fixes[i - 1][1], gps_fixes[i - 1][2],
                          gps_fixes[i][1], gps_fixes[i][2])
        gps_spds_kmh.append(d_m / dt_s * 3.6)
    gps_nz = [s for s in gps_spds_kmh if s > 0.1]

    print(f"=== {label} ===")
    print(f"  duration_s  = {duration_s:.1f}  ({len(pts)} frames, {len(gps_fixes)} GPS fixes)")
    print(f"  VIO  dist   = {vio_dist:7.2f} m   net = {vio_net:6.2f} m")
    print(f"  GPS  dist   = {gps_dist:7.2f} m   net = {gps_net:6.2f} m")
    if gps_dist > 0.5:
        print(f"  ratio       = {vio_dist / gps_dist:.2f}x cumulative, "
              f"{vio_net/gps_net if gps_net > 0.5 else float('nan'):.2f}x net")
    print(f"  VIO avg     = {vio_avg_kmh:5.2f} km/h (cum_dist/duration)")
    if nz:
        print(f"  VIO 1s win  = mean {sum(nz)/len(nz):5.2f}  "
              f"p50 {sorted(nz)[len(nz)//2]:5.2f}  max {max(spds):5.2f} km/h  "
              f"({len(nz)*100//len(spds)}% non-zero of {len(spds)} windows)")
    else:
        print(f"  VIO 1s win  = ALL ZERO across {len(spds)} windows")
    if gps_nz:
        print(f"  GPS speed   = mean {sum(gps_nz)/len(gps_nz):5.2f}  "
              f"p50 {sorted(gps_nz)[len(gps_nz)//2]:5.2f}  max {max(gps_spds_kmh):5.2f} km/h")
    print(f"  K           = {es.get('midas_scale_k_milli', 0)/1000:.0f}  "
          f"(min {es.get('midas_scale_k_min_milli',0)/1000:.0f}, "
          f"max {es.get('midas_scale_k_max_milli',0)/1000:.0f})")
    print(f"  depth_flow_updates = {es.get('depth_flow_updates', 0)}   "
          f"looming: updates={es.get('depth_flow_looming_updates', 0)} "
          f"used={es.get('depth_flow_looming_used', 0)} "
          f"skipped={es.get('depth_flow_looming_skipped', 0)}")
    print(f"  total_path_dm (counter) = {es.get('total_path_dm', 0)/10:.1f} m")
    print()


for label, fn in FILES:
    analyze(label, fn)
