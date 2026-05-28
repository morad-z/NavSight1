#!/usr/bin/env python3
"""
Offline proof: can ZUPT-anchored accelerometer integration give an accurate
metric SPEED on a NavSight sim, independent of MiDaS / visual scale?

Approach (matches the planned on-device estimator so the offline result transfers):
  1. Detect the stationary start (gyro small + |accel| ~ g, stable).
  2. Bootstrap body orientation R_WB(0) from the mean stationary specific force
     (gravity direction). Yaw is arbitrary (horizontal SPEED magnitude is
     yaw-invariant).
  3. Propagate R_WB via gyro integration (Rodrigues), rotate body specific force
     to world, subtract gravity -> linear world acceleration.
  4. Integrate linear accel -> world velocity (from v=0 at the stationary start).
  5. ZUPT: re-zero velocity whenever stationary is detected again.

We report the moving-segment speed stats and the FINAL speed (should be ~0 after
the user's full stop) -> that final value is the accumulated drift. We also run a
no-ZUPT and a high-pass variant to characterise the bias.

Body accel = specific force (includes gravity reaction; |.| ~ 9.81 at rest).
World is Z-up. Usage: python analyze_accel_speed.py <sim.json> [<sim2.json> ...]
"""
import json
import math
import sys
import numpy as np

GYRO_STAT = 0.08      # rad/s  - below this (with accel check) = stationary
ACC_STAT_TOL = 0.7    # m/s2   - |accel| within this of g => gravity-only
STAT_WIN_S = 1.0      # s      - stationary window used to estimate initial gravity


def rodrigues(w, dt):
    theta = np.linalg.norm(w) * dt
    if theta < 1e-9:
        return np.eye(3)
    k = w / np.linalg.norm(w)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def align_up_to_z(up_b):
    """Rotation R s.t. R @ up_b = world +Z (0,0,1)."""
    up_b = up_b / np.linalg.norm(up_b)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(up_b, z)
    s = np.linalg.norm(v)
    c = float(np.dot(up_b, z))
    if s < 1e-8:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def integrate(ts, a, g, gn, am, gmag, mode):
    """mode: 'zupt', 'raw', 'hp' (high-pass accel bias)."""
    n = len(ts)
    # bootstrap orientation from initial gravity direction
    # mean stationary accel was used to set gmag; reuse a[i0] region for up_b
    R_WB = align_up_to_z(a_init_dir)
    vel = np.zeros(3)
    speeds = np.zeros(n)
    a_lp = np.zeros(3)  # low-pass accel estimate (bias) for 'hp' mode
    LP_TAU = 1.5
    for i in range(n):
        dt = ts[i] - ts[i - 1] if i > 0 else 0.0
        if dt <= 0 or dt > 0.2:
            dt = 0.033
        R_WB = R_WB @ rodrigues(g[i], dt)
        f_world = R_WB @ a[i]
        a_lin = f_world - np.array([0.0, 0.0, gmag])
        if mode == 'hp':
            alpha = dt / (LP_TAU + dt)
            a_lp = a_lp + alpha * (a_lin - a_lp)
            a_use = a_lin - a_lp
        else:
            a_use = a_lin
        vel = vel + a_use * dt
        if mode == 'zupt' and gn[i] < GYRO_STAT and abs(am[i] - gmag) < ACC_STAT_TOL:
            vel = np.zeros(3)
        speeds[i] = math.hypot(vel[0], vel[1])
    return speeds


def analyze(path):
    global a_init_dir
    with open(path) as fh:
        pts = json.load(fh)["points"]
    n = len(pts)
    ts = np.array([p["ts"] for p in pts], float) / 1000.0
    a = np.array([[p["ax"], p["ay"], p["az"]] for p in pts], float)
    g = np.array([[p["gx"], p["gy"], p["gz"]] for p in pts], float)
    gn = np.linalg.norm(g, axis=1)
    am = np.linalg.norm(a, axis=1)
    stat = (gn < GYRO_STAT) & (np.abs(am - 9.81) < ACC_STAT_TOL)

    i0 = 0
    while i0 < n and not stat[i0]:
        i0 += 1
    gsamp = []
    j = i0
    while j < n and stat[j] and (ts[j] - ts[i0]) < STAT_WIN_S:
        gsamp.append(a[j]); j += 1
    g_body0 = np.mean(gsamp, axis=0) if len(gsamp) >= 3 else (a[i0] if i0 < n else np.array([0.0, 0.0, 9.81]))
    gmag = float(np.linalg.norm(g_body0))
    a_init_dir = g_body0.copy()

    dur = ts[-1] - ts[0]
    moving = ~stat
    mv_idx = np.where(moving)[0]
    print(f"\n=== {path.split(chr(92))[-1]} ===")
    print(f"  n={n} dur={dur:.1f}s  stationary_start@idx{i0}({ts[i0]-ts[0]:.1f}s)  gmag={gmag:.2f}  "
          f"moving_frac={moving.mean():.2f}")

    for mode in ('zupt', 'raw', 'hp'):
        sp = integrate(ts, a, g, gn, am, gmag, mode)
        sp_mv = sp[mv_idx] if len(mv_idx) else sp
        kmh = 3.6
        print(f"  [{mode:4s}] moving speed m/s: med={np.median(sp_mv):.2f} "
              f"p90={np.percentile(sp_mv,90):.2f} max={sp_mv.max():.2f}  "
              f"({np.median(sp_mv)*kmh:.1f}/{np.percentile(sp_mv,90)*kmh:.1f}/{sp_mv.max()*kmh:.1f} km/h)  "
              f"FINAL={sp[-1]:.2f} m/s (drift; want ~0)")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        analyze(p)
