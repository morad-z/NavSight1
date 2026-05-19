"""
diagnose_heading_speed.py — Why doesn't VIO see turns, and why is speed off?

Reads a NavSight sim JSON and answers three questions with numbers:
  1. Raw gyro integration vs published `hdg`. If raw integration sees N turns
     and `hdg` sees zero, the JNI-published heading is decoupled from gyro.
  2. Raw accel-derived speed vs GPS-derived speed vs published VIO speed
     (computed from (vx, vy) tick deltas).
  3. Gyro bias drift over the walk. If bias drifts by > 0.01 rad/s the EKF
     bias estimate isn't converging — heading drifts at the bias rate.

Usage:
    python scripts/diagnose_heading_speed.py <sim.json>
"""
from __future__ import annotations
import json, math, sys
from pathlib import Path


EARTH_R_M = 6_371_000.0


def haversine(la1, lo1, la2, lo2):
    if None in (la1, lo1, la2, lo2): return 0.0
    a = math.radians(la2 - la1); b = math.radians(lo2 - lo1)
    s = (math.sin(a/2)**2 + math.cos(math.radians(la1))
         * math.cos(math.radians(la2)) * math.sin(b/2)**2)
    return 2 * EARTH_R_M * math.asin(min(1.0, math.sqrt(s)))


def bearing(la1, lo1, la2, lo2):
    if None in (la1, lo1, la2, lo2): return None
    y = math.sin(math.radians(lo2-lo1)) * math.cos(math.radians(la2))
    x = (math.cos(math.radians(la1))*math.sin(math.radians(la2))
         - math.sin(math.radians(la1))*math.cos(math.radians(la2))
         * math.cos(math.radians(lo2-lo1)))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def wrap180(a):
    while a >  180: a -= 360
    while a < -180: a += 360
    return a


def main(p):
    sim = json.load(open(p))
    pts = sim["points"]
    if len(pts) < 50:
        print("too few points"); return 1

    # Body-frame yaw rate is whichever gyro axis is most active during turning.
    # Vertical phone, screen facing stomach: camera-forward = body +x;
    # phone-up   = body +y (because screen faces back); world-up = -gravity.
    # In NavSight's R_bc = diag(1,-1,-1) convention (camera Z = body X forward),
    # yaw-around-world-Z corresponds primarily to gyro_y when the phone is held
    # vertical (cross-check empirically below).

    # Integrate each gyro axis independently and see which one tracks turns.
    t0 = pts[0]["ts"] / 1000.0
    yaw_int_x = 0.0; yaw_int_y = 0.0; yaw_int_z = 0.0
    gx_sum = 0.0; gy_sum = 0.0; gz_sum = 0.0
    n = 0
    # Static-detection: build candidate static windows (low accel variance, low gyro mag)
    accel_buf = []; gyro_buf = []
    bias_estimates = []
    samples_per_s = 0
    last_print_t = 0

    print(f"File: {p}")
    print(f"Duration: {(pts[-1]['ts'] - pts[0]['ts'])/1000:.1f}s, n_points={len(pts)}")
    print()

    # First, print the field schema we found.
    p0 = pts[0]
    print(f"Sample point: ts={p0['ts']}, ax={p0.get('ax')}, ay={p0.get('ay')}, az={p0.get('az')}, "
          f"gx={p0.get('gx')}, gy={p0.get('gy')}, gz={p0.get('gz')}, hdg={p0.get('hdg')}")
    print()

    # Walk through, integrate gyro on each axis, track hdg, find static periods.
    static_windows = []   # list of (t_start_s, t_end_s, mean_gx, mean_gy, mean_gz)
    cur_static_start = None
    cur_static_gx = []
    cur_static_gy = []
    cur_static_gz = []

    # Per-second summary: gyro mag (yaw rate), accel mag, hdg.
    per_sec = {}  # int seconds → list of dicts

    for i in range(1, len(pts)):
        a = pts[i-1]; b = pts[i]
        dt = (b["ts"] - a["ts"]) / 1000.0
        if dt <= 0 or dt > 0.5: continue
        gx = b.get("gx", 0.0); gy = b.get("gy", 0.0); gz = b.get("gz", 0.0)
        ax = b.get("ax", 0.0); ay = b.get("ay", 0.0); az = b.get("az", 0.0)
        hdg = b.get("hdg", 0.0)
        t_s = (b["ts"] - pts[0]["ts"]) / 1000.0

        yaw_int_x += gx * dt
        yaw_int_y += gy * dt
        yaw_int_z += gz * dt

        # Static window detection: |gyro| < 0.02 rad/s and accel near gravity.
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        is_static = gyro_mag < 0.03 and abs(accel_mag - 9.81) < 0.5
        if is_static:
            if cur_static_start is None:
                cur_static_start = t_s
                cur_static_gx = []; cur_static_gy = []; cur_static_gz = []
            cur_static_gx.append(gx)
            cur_static_gy.append(gy)
            cur_static_gz.append(gz)
        else:
            if cur_static_start is not None and len(cur_static_gx) > 20:
                t_end = t_s
                static_windows.append((
                    cur_static_start, t_end,
                    sum(cur_static_gx) / len(cur_static_gx),
                    sum(cur_static_gy) / len(cur_static_gy),
                    sum(cur_static_gz) / len(cur_static_gz),
                    len(cur_static_gx),
                ))
            cur_static_start = None

        sec = int(t_s)
        if sec not in per_sec:
            per_sec[sec] = {"gyro_mag": [], "hdg": [], "ax": [], "ay": [], "az": []}
        per_sec[sec]["gyro_mag"].append(gyro_mag)
        per_sec[sec]["hdg"].append(hdg)
        per_sec[sec]["ax"].append(ax)
        per_sec[sec]["ay"].append(ay)
        per_sec[sec]["az"].append(az)

    # Gyro integration totals — which axis tracks turns best?
    print(f"=== GYRO YAW INTEGRATION (raw, no bias correction) ===")
    print(f"  ∫gx dt = {math.degrees(yaw_int_x):+8.1f}° over the walk")
    print(f"  ∫gy dt = {math.degrees(yaw_int_y):+8.1f}° over the walk")
    print(f"  ∫gz dt = {math.degrees(yaw_int_z):+8.1f}° over the walk")
    print(f"  (For vertical phone, the dominant yaw axis should be the one whose integral matches the user's walked turns.)")
    print()

    # Published hdg field range
    hdgs = [p.get("hdg", 0.0) for p in pts]
    hdg_min = min(hdgs); hdg_max = max(hdgs)
    # circular hdg, so wrap-aware range:
    hdg_unwrapped = [hdgs[0]]
    for i in range(1, len(hdgs)):
        d = wrap180(hdgs[i] - hdgs[i-1])
        hdg_unwrapped.append(hdg_unwrapped[-1] + d)
    hdg_excursion = max(hdg_unwrapped) - min(hdg_unwrapped)
    print(f"=== PUBLISHED hdg FIELD ===")
    print(f"  hdg first = {hdgs[0]:7.1f}° last = {hdgs[-1]:7.1f}°")
    print(f"  hdg circular range (min..max wrapped): {hdg_min:.1f}°..{hdg_max:.1f}°")
    print(f"  hdg unwrapped total excursion: {hdg_excursion:.1f}°")
    print()

    # Static periods → gyro bias estimate evolution
    print(f"=== STATIC WINDOW GYRO BIAS ESTIMATES ===")
    print(f"  Static windows detected: {len(static_windows)}")
    print(f"  (Each is a window where the phone was held still — mean gyro = bias estimate)")
    for j, w in enumerate(static_windows[:10]):
        t_start, t_end, mgx, mgy, mgz, ns = w
        print(f"    [{j+1:2d}] t={t_start:6.1f}..{t_end:6.1f}s  n={ns:4d}  "
              f"bias=(gx={mgx:+.5f}, gy={mgy:+.5f}, gz={mgz:+.5f}) rad/s")
    if len(static_windows) > 1:
        # Compare first vs last bias estimate to see drift
        first = static_windows[0]
        last = static_windows[-1]
        drift_gx = last[2] - first[2]
        drift_gy = last[3] - first[3]
        drift_gz = last[4] - first[4]
        print(f"  Bias drift across the walk:")
        print(f"    Δbias_gx = {drift_gx:+.5f} rad/s ({math.degrees(drift_gx)*60:+.2f}°/min)")
        print(f"    Δbias_gy = {drift_gy:+.5f} rad/s ({math.degrees(drift_gy)*60:+.2f}°/min)")
        print(f"    Δbias_gz = {drift_gz:+.5f} rad/s ({math.degrees(drift_gz)*60:+.2f}°/min)")
        # For consumer MEMS, bias instability is ~0.005 rad/s (Allan deviation).
        # If our observed drift over a 2-min walk is much bigger, EKF bias estimate not converging.
        print(f"  (Consumer MEMS Allan bias instability ≈ 0.005 rad/s = 17°/min)")
    print()

    # GPS-derived path + bearing for ground truth comparison
    gps_path = 0.0; gps_speed_max = 0.0
    last_gps = None
    gps_pts_for_bearing = []
    for p in pts:
        la = p.get("glat"); lo = p.get("glng")
        if not la or not lo or (la == 0.0 and lo == 0.0): continue
        if last_gps is not None:
            d = haversine(*last_gps[:2], la, lo)
            if d < 20.0:
                gps_path += d
                dt = (p["ts"] - last_gps[2]) / 1000.0
                if dt > 0 and d > 0.1:
                    spd = d / dt
                    if spd < 8.0:  # walking speed ceiling 8 m/s — anything more is GPS jump
                        gps_speed_max = max(gps_speed_max, spd)
        last_gps = (la, lo, p["ts"])
        gps_pts_for_bearing.append((p["ts"], la, lo))

    # GPS turn count derived from successive bearings (>2 m apart, low-pass)
    gps_bearings = []
    for i in range(1, len(gps_pts_for_bearing)):
        t1, la1, lo1 = gps_pts_for_bearing[i-1]
        t2, la2, lo2 = gps_pts_for_bearing[i]
        if haversine(la1, lo1, la2, lo2) >= 2.0:
            br = bearing(la1, lo1, la2, lo2)
            if br is not None:
                gps_bearings.append((t1, br))
    # count >45° heading swings in a 5-step rolling window
    turn_n = 0
    for i in range(5, len(gps_bearings)):
        if abs(wrap180(gps_bearings[i][1] - gps_bearings[i-5][1])) > 45:
            turn_n += 1
    print(f"=== GPS GROUND TRUTH ===")
    print(f"  GPS path summed (Haversine, filtered): {gps_path:.1f} m")
    print(f"  GPS peak walking speed:                {gps_speed_max:.2f} m/s")
    print(f"  GPS detected turns (>45° in 5 fixes):  {turn_n}")
    print()

    # Speed comparison via VIO position tick deltas
    vio_speeds = []
    for i in range(1, len(pts)):
        a = pts[i-1]; b = pts[i]
        dt = (b["ts"] - a["ts"]) / 1000.0
        if dt <= 0: continue
        d = math.sqrt((b["vx"]-a["vx"])**2 + (b["vy"]-a["vy"])**2 + (b["vz"]-a["vz"])**2)
        if d < 5.0:
            vio_speeds.append(d/dt)
    if vio_speeds:
        # Mean of moving samples
        moving = [s for s in vio_speeds if 0.2 < s < 5.0]
        print(f"=== VIO SPEED (from vx/vy/vz tick deltas) ===")
        print(f"  Mean moving speed: {sum(moving)/len(moving):.2f} m/s  (typical walk = 1.2 m/s)")
        print(f"  Max moving speed:  {max(moving):.2f} m/s")
        print(f"  Fraction of ticks moving (>0.2 m/s): {len(moving)/len(vio_speeds):.1%}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "tests/sims/regression/visual/v28_phase6_walk1.json"))
