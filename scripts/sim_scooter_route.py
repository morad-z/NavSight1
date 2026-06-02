"""1 km scooter-route simulator for the NavSight speed/scale/trajectory pipeline.

Production teams test VIO by replaying recorded datasets WITH ground truth, or by
synthesizing sensors from a known path. We can't render real camera frames here (that
needs a 3D engine and wouldn't capture real MiDaS/recoverPose noise anyway), but the
part that is FAILING is the scale+speed+trajectory math — and that we CAN simulate
faithfully against a known ground-truth route.

This script:
  1. Builds a ground-truth ~1 km city scooter route (straights, turns, stops, accel/brake to ~25 km/h).
  2. Synthesizes per-frame sensors from the GT: accelerometer (gravity + bias + noise + the
     dominant GRAVITY-LEAK from attitude error), heading (gyro drift, mag-bounded), and the
     visual relative speed (depth-flow: GT_speed / K_true + recoverPose-style noise), ZUPT at stops.
  3. Runs a FAITHFUL port of the current on-device pipeline:
       raw-accel integration + ZUPT re-zero  ->  accel-K calibration (post-ZUPT window, EMA,
       >3x outlier guard, dist cap)           ->  depth_flow_speed = K * speed_rel
                                              ->  trajectory: pos += speed*dt*[sin h, cos h] (8 m/s cap)
  4. Scores: total-distance error %, speed RMSE, endpoint drift, and decomposes scale-error vs
     heading-error. Runs 3 noise scenarios (optimistic / realistic / pessimistic) so you see a
     RANGE, not one rosy number. Saves trajectory + speed plots.

Noise grounded in the real v5/v6/v7 recordings: K ~1000-1900 (so ~±20% per-frame relative-speed
noise); accel zupt reference roughly matched run speed; Madgwick heading ~0.3 deg/s drift (mag-bounded).

Run: python scripts/sim_scooter_route.py
"""
import math
import os
import numpy as np

FPS = 30.0
DT = 1.0 / FPS
G = 9.81
K_TRUE = 1200.0           # true relative->metric scale (scene constant; from real v7 K range)
DISP_CAP_MPS = 8.0        # trajectory per-frame speed cap (matches Tracker disp cap)
ACCEL_K_MIN_DIST_M = 1.0  # matches kAccelKMinDistM
ZUPT_WIN = (0.3, 2.5)     # secs_since_zupt window for K calibration


def build_route():
    """Return list of segments: (turn_deg_into_segment, length_m, cruise_kmh, stop_after).
    ~1 km city ride: straights with turns and a few intersection stops."""
    return [
        # turn(+=left/right rel), length m, cruise km/h, stop at end?
        (0,   180, 22, True),   # start straight, stop at light
        (90,  220, 25, False),  # right turn, long straight
        (-90, 120, 18, True),   # left, slow zone, stop
        (45,  260, 27, False),  # diagonal fast straight
        (-30, 140, 20, True),   # ease left, stop
        (60,  90,  15, False),  # turn into slow lane
    ]  # total ~1010 m


def gen_ground_truth(route):
    """Time-step the route at FPS. Returns arrays: t, x, y, heading(rad), speed(m/s), is_stop(bool)."""
    ts, xs, ys, hs, sps, stops = [], [], [], [], [], []
    x, y, t = 0.0, 0.0, 0.0
    heading = 0.0  # rad, 0 = +Y (north)
    A_ACC, A_DEC = 2.0, 3.0  # m/s^2 accel / brake
    # 2026-05-29 — model the real start: the user stands STILL before riding (the app's
    # init requires a stationary period). This opens the ZUPT K-calibration window so K
    # calibrates during the first acceleration — i.e. the first segment is NOT zero. The
    # earlier 20% cold-start drift was largely an artifact of starting in motion.
    for _ in range(int(2.0 * FPS)):
        t += DT
        ts.append(t); xs.append(0.0); ys.append(0.0); hs.append(0.0)
        sps.append(0.0); stops.append(True)
    for (turn_deg, length, cruise_kmh, stop_after) in route:
        heading += math.radians(turn_deg)
        cruise = cruise_kmh / 3.6
        # distance-based speed profile within the segment: ramp up, cruise, (ramp down if stop)
        dist = 0.0
        v = sps[-1] if sps else 0.0
        # precompute brake distance if stopping
        brake_d = (cruise ** 2) / (2 * A_DEC) if stop_after else 0.0
        while dist < length:
            remaining = length - dist
            if stop_after and remaining <= brake_d:
                v = max(0.0, v - A_DEC * DT)
            elif v < cruise:
                v = min(cruise, v + A_ACC * DT)
            else:
                v = cruise
            x += v * math.sin(heading) * DT
            y += v * math.cos(heading) * DT
            dist += v * DT
            t += DT
            ts.append(t); xs.append(x); ys.append(y); hs.append(heading)
            sps.append(v); stops.append(False)
            if v < 1e-6 and stop_after:
                break
        if stop_after:
            # dwell stopped ~1.5 s (ZUPT window)
            for _ in range(int(1.5 * FPS)):
                t += DT
                ts.append(t); xs.append(x); ys.append(y); hs.append(heading)
                sps.append(0.0); stops.append(True)
    return (np.array(ts), np.array(xs), np.array(ys), np.array(hs),
            np.array(sps), np.array(stops))


def simulate(gt, noise, rng, init_K=-1.0):
    """Run the faithful pipeline port over the GT with the given noise dict.
    init_K: starting K. -1 = COLD start (no persisted K, calibrates after first ZUPT).
            >0 = WARM start (persisted K from a prior session, as the app actually runs).
    noise keys: grav_leak_deg (attitude err -> gravity leak), accel_white, flow_rel (relative-speed
    multiplicative noise frac), head_drift_dps (heading drift deg/s)."""
    ts, xs, ys, hs, sps, stops = gt
    n = len(ts)
    # --- attitude error (slow random walk, bounded) -> gravity leak into horizontal accel ---
    att_err = 0.0
    # --- pipeline state ---
    accel_vel = np.zeros(2)         # horizontal world accel-velocity (raw integ, ZUPT rezero)
    secs_since_zupt = -1.0
    accel_dist = 0.0
    vis_rel_dist = 0.0
    K = init_K
    est_x = est_y = 0.0
    est_heading_err = 0.0
    est_pos = [(0.0, 0.0)]
    est_speeds = np.zeros(n)
    K_hist = []
    prev_v_world = np.zeros(2)
    for i in range(n):
        v = sps[i]; h = hs[i]; is_static = stops[i]
        v_world = np.array([v * math.sin(h), v * math.cos(h)])
        a_world = (v_world - prev_v_world) / DT
        prev_v_world = v_world
        # attitude error random walk (bounded), produces gravity leak
        att_err += rng.normal(0, math.radians(noise['grav_leak_deg']) * 0.1)
        att_err = float(np.clip(att_err, -math.radians(noise['grav_leak_deg']) * 3,
                                math.radians(noise['grav_leak_deg']) * 3))
        grav_leak = G * math.sin(att_err)   # leaks into a horizontal axis
        # measured horizontal linear accel (what the integrator sees): true + leak + white
        a_meas = a_world.copy()
        a_meas[0] += grav_leak + rng.normal(0, noise['accel_white'])
        a_meas[1] += rng.normal(0, noise['accel_white'])
        # --- accel integration with ZUPT rezero (matches fixed on-device: raw, no HP) ---
        if not is_static:
            accel_vel += a_meas * DT
            if secs_since_zupt >= 0.0:
                secs_since_zupt += DT
            if 0.0 <= secs_since_zupt <= ZUPT_WIN[1]:
                accel_dist += math.hypot(accel_vel[0], accel_vel[1]) * DT
        else:
            accel_vel[:] = 0.0
            secs_since_zupt = 0.0
            accel_dist = 0.0
            vis_rel_dist = 0.0
        # --- visual relative speed (depth-flow): GT speed / K_true + recoverPose noise ---
        speed_rel = (v / K_TRUE) * (1.0 + rng.normal(0, noise['flow_rel'])) if v > 1e-6 else 0.0
        if not is_static:
            vis_rel_dist += speed_rel * DT
        # --- accel-K calibration (post-ZUPT window, EMA 0.95/0.05, >3x outlier guard) ---
        if (ZUPT_WIN[0] <= secs_since_zupt <= ZUPT_WIN[1]
                and accel_dist > ACCEL_K_MIN_DIST_M and vis_rel_dist > 1e-6):
            k_obs = accel_dist / vis_rel_dist
            outlier = (K > 0.0) and (k_obs > 3.0 * K or k_obs < K / 3.0)
            if math.isfinite(k_obs) and k_obs > 0.0 and not outlier:
                K = k_obs if K <= 0.0 else (0.95 * K + 0.05 * k_obs)
                K_hist.append((ts[i], K))
        # --- depth-flow metric speed ---
        if K > 0.0:
            est_speed = K * speed_rel
            est_speed = min(est_speed, DISP_CAP_MPS)
        else:
            est_speed = 0.0
        est_speeds[i] = est_speed
        # --- heading estimate (gyro drift, mag-bounded random walk) ---
        est_heading_err += rng.normal(0, math.radians(noise['head_drift_dps']) * DT)
        est_heading_err = float(np.clip(est_heading_err, -math.radians(8), math.radians(8)))
        est_h = h + est_heading_err
        # --- trajectory integration (matches app: pos += speed*dt*[sin,cos]) ---
        est_x += est_speed * math.sin(est_h) * DT
        est_y += est_speed * math.cos(est_h) * DT
        est_pos.append((est_x, est_y))
    return np.array(est_pos[1:]), est_speeds, K_hist


def score(gt, est_pos, est_speeds):
    ts, xs, ys, hs, sps, stops = gt
    # path lengths
    gt_path = np.sum(np.hypot(np.diff(xs), np.diff(ys)))
    est_path = np.sum(np.hypot(np.diff(est_pos[:, 0]), np.diff(est_pos[:, 1])))
    # endpoint
    endpoint_err = math.hypot(est_pos[-1, 0] - xs[-1], est_pos[-1, 1] - ys[-1])
    # speed RMSE over moving frames
    mv = sps > 0.1
    spd_rmse = math.sqrt(np.mean((est_speeds[mv] - sps[mv]) ** 2)) if mv.any() else float('nan')
    return dict(gt_path=gt_path, est_path=est_path,
                dist_err_pct=100.0 * (est_path - gt_path) / gt_path,
                endpoint_err=endpoint_err,
                endpoint_pct=100.0 * endpoint_err / gt_path,
                spd_rmse_kmh=spd_rmse * 3.6,
                gt_mean_kmh=np.mean(sps[mv]) * 3.6 if mv.any() else 0.0,
                est_mean_kmh=np.mean(est_speeds[mv]) * 3.6 if mv.any() else 0.0)


SCENARIOS = {
    'optimistic': dict(grav_leak_deg=0.2, accel_white=0.02, flow_rel=0.08, head_drift_dps=0.1),
    'realistic':  dict(grav_leak_deg=0.6, accel_white=0.05, flow_rel=0.18, head_drift_dps=0.3),
    'pessimistic':dict(grav_leak_deg=1.2, accel_white=0.10, flow_rel=0.30, head_drift_dps=0.6),
}

def main():
    route = build_route()
    gt = gen_ground_truth(route)
    ts = gt[0]
    print(f"=== Scooter route simulation ===")
    print(f"  GT path length = {np.sum(np.hypot(np.diff(gt[1]), np.diff(gt[2]))):.1f} m, "
          f"duration = {ts[-1]:.0f} s, frames = {len(ts)}")
    print(f"  K_true = {K_TRUE:.0f}, {len([s for s in route if s[3]])} stops\n")
    # WARM start models the app's real steady-state operation: K persisted from a prior
    # session (SharedPreferences). Seed with a small persistence error (+/-5%) so it is not
    # cheating. COLD start (init_K=-1) shows the one-time first-install penalty (first
    # segment reads ~0 until the first ZUPT calibrates K).
    results = {}
    for start_name, init_K_fn in [('WARM (persisted K, steady-state)', lambda r: K_TRUE * (1 + r.normal(0, 0.05))),
                                  ('COLD (first install, no persisted K)', lambda r: -1.0)]:
        print(f"  --- {start_name} ---")
        for name, noise in SCENARIOS.items():
            accs, last = [], None
            for seed in range(5):
                rng = np.random.default_rng(seed + 100)
                est_pos, est_speeds, K_hist = simulate(gt, noise, rng, init_K=init_K_fn(rng))
                accs.append(score(gt, est_pos, est_speeds))
                last = (est_pos, est_speeds, K_hist)
            agg = {k: np.mean([a[k] for a in accs]) for k in accs[0]}
            if start_name.startswith('WARM') and name == 'realistic':
                results['plot'] = (agg, last)
            print(f"    [{name:11s}] dist_err={agg['dist_err_pct']:+5.1f}%  "
                  f"endpoint_err={agg['endpoint_err']:5.1f} m ({agg['endpoint_pct']:.1f}%)  "
                  f"speed GT {agg['gt_mean_kmh']:.1f}/est {agg['est_mean_kmh']:.1f} km/h  "
                  f"RMSE {agg['spd_rmse_kmh']:.1f}")
        print()

    # ---- plots for the realistic scenario ----
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        est_pos, est_speeds, K_hist = results['plot'][1]
        outdir = os.path.join(os.path.dirname(__file__), 'output')
        os.makedirs(outdir, exist_ok=True)
        fig, ax = plt.subplots(1, 2, figsize=(14, 6))
        ax[0].plot(gt[1], gt[2], 'g-', lw=2, label='ground truth')
        ax[0].plot(est_pos[:, 0], est_pos[:, 1], 'r--', lw=1.5, label='estimated (realistic)')
        ax[0].plot(gt[1][0], gt[2][0], 'ko', label='start')
        ax[0].plot(gt[1][-1], gt[2][-1], 'gs', ms=8, label='GT end')
        ax[0].plot(est_pos[-1, 0], est_pos[-1, 1], 'r^', ms=8, label='est end')
        ax[0].set_aspect('equal'); ax[0].legend(); ax[0].grid(alpha=0.3)
        ax[0].set_title('Trajectory (top-down)'); ax[0].set_xlabel('East (m)'); ax[0].set_ylabel('North (m)')
        ax[1].plot(gt[0], gt[4] * 3.6, 'g-', lw=2, label='GT speed')
        ax[1].plot(gt[0], est_speeds * 3.6, 'r-', lw=0.8, alpha=0.8, label='est speed')
        ax[1].legend(); ax[1].grid(alpha=0.3)
        ax[1].set_title('Speed vs time'); ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('km/h')
        png = os.path.join(outdir, 'sim_scooter_route.png')
        fig.tight_layout(); fig.savefig(png, dpi=110)
        print(f"\n  Plot saved: {png}")
    except Exception as e:
        print(f"\n  (plot skipped: {e})")


if __name__ == '__main__':
    main()
