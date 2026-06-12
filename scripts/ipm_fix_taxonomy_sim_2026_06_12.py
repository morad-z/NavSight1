#!/usr/bin/env python3
"""Predictive validation of the 2026-06-12 IPM fix, on real ride frames.

Replicates the PROPOSED estimator on the same frame pairs the probe used:

  front-end : road-mask goodFeatures top-up + LK(61,5) + forward-backward
              consistency check (the production tracker has FB at
              TrackKLT.cpp:57; the Fix-B IPM flow at Tracker.cpp:3096 does NOT)
  taxonomy  : per-point noise floor sigma_v_i = (SIGMA_PX/fx) * Z / dt / |a|
              vote  := coherent (cos<-1/sqrt2) & v_i >= 3 sigma_v_i & v_i < 50
              zero  := |v_i| < 3 sigma_v_i & 3 sigma_v_i <= 1.0 m/s (witness-able)
              reject otherwise
  aggregate : >=5 votes -> median(votes); else >=5 zeros -> 0; else decay(old)

and prints OLD-logic vs NEW-logic speed per pair class (stop / slow / fast).

SIGMA_PX = 0.5 px: measured on this ride's standstill pairs (FixB-KLT flow
p50 = 0.57 px, probe 2026-06-12) ~ Bouguet KLT subpixel accuracy bound.

Usage: py -3.13 scripts/ipm_fix_taxonomy_sim_2026_06_12.py [ride.json]
"""
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs opencv-python")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ipm_frames_flow_probe_2026_06_12 import (  # noqa: E402
    CAMERA_H_M, COH_MIN, CX, CY, DEFAULT_JSON, FX, KMAX_Z, KMIN_Z, R_BC,
    gravity_at, load_ride, nearest_idx, road_band_mask)

SIGMA_PX = 0.5
ZERO_WITNESS_MAX_FLOOR = 1.0   # m/s: a zero-witness must be able to exclude >1 m/s motion
FB_THRESH_PX = 1.5


def run_pair(frames_dir, fa, fb, ts_a, ts_b, t_ms, acc, gyr):
    ga = cv2.imread(os.path.join(frames_dir, fa), cv2.IMREAD_GRAYSCALE)
    gb = cv2.imread(os.path.join(frames_dir, fb), cv2.IMREAD_GRAYSCALE)
    if ga is None or gb is None:
        return None
    dt_s = (ts_b - ts_a) / 1000.0
    g_body = gravity_at(t_ms, acc, ts_a)
    n_up_c = R_BC @ g_body
    if n_up_c[2] > -0.05:
        return None
    w_body = gyr[nearest_idx(t_ms, (ts_a + ts_b) / 2)] * dt_s
    wx, wy, wz = R_BC @ w_body

    # front-end: road-mask corner top-up + LK61 + FB check
    mask = road_band_mask(ga.shape, n_up_c, KMIN_Z, KMAX_Z)
    corners = cv2.goodFeaturesToTrack(ga, maxCorners=100, qualityLevel=0.01,
                                      minDistance=8, mask=mask)
    if corners is None or len(corners) < 5:
        return None
    p0 = corners.astype(np.float32)
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 30, 0.01)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(ga, gb, p0, None, winSize=(61, 61),
                                         maxLevel=5, criteria=crit)
    pb, st2, _ = cv2.calcOpticalFlowPyrLK(gb, ga, p1, None, winSize=(61, 61),
                                          maxLevel=5, criteria=crit)
    st = (st.reshape(-1) > 0) & (st2.reshape(-1) > 0)
    fb_err = np.linalg.norm((pb - p0).reshape(-1, 2), axis=1)
    keep = st & (fb_err < FB_THRESH_PX)

    u_fwd = np.array([0.0, 0.0, 1.0]) - np.array([0.0, 0.0, 1.0]).dot(n_up_c) * n_up_c
    u_fwd /= (np.linalg.norm(u_fwd) + 1e-12)

    votes, zeros, old_speeds = [], 0, []
    n_keep = 0
    for k in range(len(p0)):
        if not keep[k]:
            continue
        n_keep += 1
        px, py = p0[k, 0]
        nx, ny = p1[k, 0]
        xp, yp = (px - CX) / FX, (py - CY) / FX
        D2 = n_up_c[0] * xp + n_up_c[1] * yp + n_up_c[2]
        inv_z = -D2 / CAMERA_H_M
        if inv_z <= 0:
            continue
        Z = 1.0 / inv_z
        if not (KMIN_Z <= Z <= KMAX_Z):
            continue
        u_rot = xp * yp * wx - (1 + xp * xp) * wy + yp * wz
        v_rot = (1 + yp * yp) * wx - xp * yp * wy - xp * wz
        fx_tr = (nx - px) / FX - u_rot
        fy_tr = (ny - py) / FX - v_rot
        a_x = (u_fwd[0] - xp * u_fwd[2]) * inv_z
        a_y = (u_fwd[1] - yp * u_fwd[2]) * inv_z
        aa = a_x * a_x + a_y * a_y
        if aa <= 1e-8:
            continue
        v_i = -((fx_tr * a_x + fy_tr * a_y) / aa) / dt_s
        fmag = math.hypot(fx_tr, fy_tr)
        cos_fa = (fx_tr * a_x + fy_tr * a_y) / (fmag * math.sqrt(aa) + 1e-12)
        # OLD logic for comparison
        if v_i < 50.0 and cos_fa < -COH_MIN:
            old_speeds.append(v_i)
        # NEW taxonomy
        sigma_v = (SIGMA_PX / FX) / (math.sqrt(aa) * dt_s)
        if abs(v_i) < 3.0 * sigma_v:
            if 3.0 * sigma_v <= ZERO_WITNESS_MAX_FLOOR:
                zeros += 1
        elif v_i < 50.0 and cos_fa < -COH_MIN and v_i >= 3.0 * sigma_v:
            votes.append(v_i)

    old = float(np.median(old_speeds)) if len(old_speeds) >= 5 else None
    if len(votes) >= 5:
        new = ("vote", float(np.median(votes)))
    elif zeros >= 5:
        new = ("ZERO", 0.0)
    else:
        new = ("decay", None)
    return dict(n_tracked=n_keep, old=old, new=new,
                n_votes=len(votes), n_zeros=zeros)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    frames_dir = path.replace(".json", ".frames")
    t_ms, acc, gyr, gps_v, gp = load_ride(path)
    files = sorted(os.listdir(frames_dir))
    ts_ms = np.array([int(f.split(".")[0]) for f in files], dtype=np.float64) / 1e6
    pairs = [(i, i + 1) for i in range(len(files) - 1)
             if 0.025 <= (ts_ms[i + 1] - ts_ms[i]) / 1000.0 <= 0.055]
    t0 = t_ms[0]
    classes = {"stop": [], "slow": [], "fast": []}
    for i, j in pairs:
        g = gps_v[nearest_idx(t_ms, ts_ms[i])]
        tt = (ts_ms[i] - t0) / 1000.0
        if tt < 10.0:
            classes["stop"].append((i, j))
        elif np.isfinite(g) and 3.0 <= g <= 8.0:
            classes["slow"].append((i, j))
        elif np.isfinite(g) and g >= 10.0:
            classes["fast"].append((i, j))

    for name, lst in classes.items():
        sel = lst[:: max(1, len(lst) // 12)][:12]
        rows = [r for r in (run_pair(frames_dir, files[i], files[j], ts_ms[i],
                                     ts_ms[j], t_ms, acc, gyr) for i, j in sel) if r]
        if not rows:
            print(f"{name}: no analysable pairs")
            continue
        olds = [r["old"] for r in rows if r["old"] is not None]
        news = [r["new"][1] for r in rows if r["new"][1] is not None]
        zero_locks = sum(1 for r in rows if r["new"][0] == "ZERO")
        decays = sum(1 for r in rows if r["new"][0] == "decay")
        gps_here = [gps_v[nearest_idx(t_ms, ts_ms[i])] for i, j in sel]
        gps_med = np.nanmedian(gps_here) * 3.6
        print(f"\n{name} ({len(rows)} pairs, GPS p50={gps_med:.0f} km/h):")
        print(f"  tracked/pair p50 = {np.median([r['n_tracked'] for r in rows]):.0f}  "
              f"votes p50 = {np.median([r['n_votes'] for r in rows]):.0f}  "
              f"zeros p50 = {np.median([r['n_zeros'] for r in rows]):.0f}")
        print(f"  OLD speed p50 = {np.median(olds)*3.6 if olds else float('nan'):6.1f} km/h "
              f"(emitted on {len(olds)}/{len(rows)} pairs)")
        print(f"  NEW speed p50 = {np.median(news)*3.6 if news else float('nan'):6.1f} km/h "
              f"(votes on {len(news)}/{len(rows)}, ZERO-lock on {zero_locks}, decay on {decays})")


if __name__ == "__main__":
    main()
