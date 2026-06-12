#!/usr/bin/env python3
"""IPM under-read + standstill-creep analyzer (2026-06-12).

Tests, against a recorded ride JSON, the competing explanations for the IPM
ground-flow speed misbehaviour:

  H-rot   : dips at cruise coincide with high gyro (large_frame_rotation bail
            at Tracker.cpp:9137 -> 0.85x decay per IPM frame)
  H-klt   : dips coincide with low/failed production flow (mflow) i.e. the
            fast near-ground flow exceeds the tracker range -> <5 coherent
            survivors -> decay branch at Tracker.cpp:9308
  H-decay : the dips themselves follow the exact 0.85^k geometric decay
            signature (signal LOSS) rather than low-but-fresh measurements
  Creep   : at standstill the IPM keeps emitting 1-5 km/h. Quantify it, and
            check whether the system's own is_static bit (pflags bit0) was
            asserted there (i.e. whether an is_static gate would have fixed it)

Usage:
  python scripts/analyze_ipm_underread_2026_06_12.py [ride.json]
Defaults to tests/sims/val_2026_06_03b/simulation_data_1780505700912.json
"""
import json
import math
import sys

import numpy as np

DEFAULT_JSON = "tests/sims/val_2026_06_03b/simulation_data_1780505700912.json"
R_EARTH_M = 6371000.0
DECAY_ALPHA = 0.85          # Tracker.cpp:9139 / :9316-9317  (1 - 0.15)
GPS_MAX_ACC_M = 25.0        # ignore fixes worse than this
GPS_MAX_SPEED_MPS = 25.0    # jam-spike clip (90 km/h on a scooter = artifact)
CRUISE_MIN_MPS = 4.0        # >= 14.4 km/h sustained = cruise
STOP_MAX_MPS = 0.6          # GPS-speed below this = candidate stop


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH_M * math.asin(math.sqrt(a))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    with open(path) as fh:
        d = json.load(fh)
    pts = d["points"]
    n = len(pts)

    t = np.array([p["ts"] for p in pts], dtype=np.float64) / 1000.0
    t -= t[0]
    gp = np.array([p["gp_flow_speed"] if p["gp_flow_speed"] is not None else np.nan for p in pts])
    fused = np.array([p["fused_speed"] if p["fused_speed"] is not None else np.nan for p in pts])
    gyro = np.array([[p["gx"], p["gy"], p["gz"]] for p in pts])
    acc = np.array([[p["ax"], p["ay"], p["az"]] for p in pts])
    mflow = np.array([p["mflow"] for p in pts], dtype=np.float64)
    inl = np.array([p["inl"] for p in pts], dtype=np.float64)
    pflags = np.array([p["pflags"] for p in pts], dtype=np.int64)
    glat = np.array([p["glat"] if p["glat"] is not None else np.nan for p in pts])
    glng = np.array([p["glng"] if p["glng"] is not None else np.nan for p in pts])
    gacc = np.array([p["gacc"] if p["gacc"] is not None else np.nan for p in pts])

    gyro_norm = np.linalg.norm(gyro, axis=1)
    acc_norm = np.linalg.norm(acc, axis=1)
    static_bit = (pflags & 1).astype(bool)

    dt_med = float(np.median(np.diff(t)))
    print(f"file: {path}")
    print(f"points={n} duration={t[-1]:.1f}s sample_dt={dt_med*1000:.0f}ms")

    # ── GPS speed (jam-aware) ─────────────────────────────────────────────
    gps_v = np.full(n, np.nan)
    last_i = None
    for i in range(n):
        if not np.isfinite(glat[i]) or (np.isfinite(gacc[i]) and gacc[i] > GPS_MAX_ACC_M):
            continue
        if last_i is not None:
            ddt = t[i] - t[last_i]
            if 0.2 <= ddt <= 5.0:
                v = haversine_m(glat[last_i], glng[last_i], glat[i], glng[i]) / ddt
                if v <= GPS_MAX_SPEED_MPS:
                    gps_v[i] = v
        last_i = i
    # forward-fill GPS speed up to 1.5 s so per-frame masks line up with ~1 Hz GPS
    filled = np.full(n, np.nan)
    last_v, last_t = np.nan, -10.0
    for i in range(n):
        if np.isfinite(gps_v[i]):
            last_v, last_t = gps_v[i], t[i]
        if t[i] - last_t <= 1.5:
            filled[i] = last_v
    gps_v = filled
    ok_gps = np.isfinite(gps_v)
    print(f"GPS usable on {ok_gps.mean()*100:.0f}% of frames "
          f"(median acc={np.nanmedian(gacc):.1f}m)")

    # ── cruise / dip / good masks ─────────────────────────────────────────
    cruise = ok_gps & (gps_v >= CRUISE_MIN_MPS) & np.isfinite(gp)
    dip = cruise & (gp < 0.5 * gps_v)
    good = cruise & (gp >= 0.75 * gps_v) & (gp <= 1.30 * gps_v)
    over = cruise & (gp > 1.30 * gps_v)
    print(f"\n── cruise (GPS>= {CRUISE_MIN_MPS*3.6:.0f} km/h): {cruise.sum()} frames "
          f"({cruise.sum()*dt_med:.0f}s)")
    if cruise.sum() > 50:
        ratio = gp[cruise] / gps_v[cruise]
        print(f"  gp/gps ratio: p10={np.percentile(ratio,10):.2f} p50={np.percentile(ratio,50):.2f} "
              f"p90={np.percentile(ratio,90):.2f}")
        print(f"  frames: dip(<0.5x)={dip.sum()} ({dip.sum()/cruise.sum()*100:.0f}%)  "
              f"good(0.75-1.3x)={good.sum()} ({good.sum()/cruise.sum()*100:.0f}%)  "
              f"over(>1.3x)={over.sum()} ({over.sum()/cruise.sum()*100:.0f}%)")
        if good.sum() > 20:
            print(f"  WHEN MEASURING (good frames): gp/gps p50="
                  f"{np.percentile(gp[good]/gps_v[good],50):.2f} -> residual scale error when locked")

    # ── H-rot: gyro in dip vs good ────────────────────────────────────────
    print("\n── H-rot (rotation-bail decay, threshold ~0.2 rad/frame ≈ 3-4.6 rad/s):")
    for name, m in (("dip ", dip), ("good", good)):
        if m.sum() > 10:
            g = gyro_norm[m]
            print(f"  {name}: |gyro| p50={np.percentile(g,50):.2f} p90={np.percentile(g,90):.2f} "
                  f"p99={np.percentile(g,99):.2f} rad/s   frac>3rad/s={np.mean(g>3.0)*100:.1f}%")
    print("  (UI-tick sampled gyro UNDER-samples vibration spikes; treat >3rad/s frac as lower bound)")

    # ── H-klt: production flow in dip vs good ─────────────────────────────
    print("\n── H-klt (production-tracker flow & inliers):")
    for name, m in (("dip ", dip), ("good", good)):
        if m.sum() > 10:
            print(f"  {name}: mflow p50={np.nanpercentile(mflow[m],50):.1f}px "
                  f"p90={np.nanpercentile(mflow[m],90):.1f}px   inl p50={np.nanpercentile(inl[m],50):.0f}")

    # ── H-decay: geometric 0.85^k signature inside dips ──────────────────
    print("\n── H-decay (0.85^k geometric-decay signature between consecutive samples):")
    decay_flag = np.zeros(n, dtype=bool)
    la = math.log(DECAY_ALPHA)
    for i in range(1, n):
        if gp[i - 1] is None or not (np.isfinite(gp[i]) and np.isfinite(gp[i - 1])):
            continue
        if gp[i] > 0.05 and gp[i - 1] > 0.05 and gp[i] < gp[i - 1] * 0.995:
            k = math.log(gp[i] / gp[i - 1]) / la
            if k >= 0.7 and abs(k - round(k)) < 0.12:
                decay_flag[i] = True
    print(f"  decay-signature frames: total={decay_flag.sum()} "
          f"({decay_flag.sum()/n*100:.0f}% of ride)")
    if dip.sum() > 10:
        print(f"  within cruise DIPS: {decay_flag[dip].mean()*100:.0f}% carry the exact decay signature")
    if good.sum() > 10:
        print(f"  within good frames: {decay_flag[good].mean()*100:.0f}%")

    # ── dip episodes ──────────────────────────────────────────────────────
    eps, cur = [], 0
    for i in range(n):
        if dip[i]:
            cur += 1
        elif cur:
            eps.append(cur)
            cur = 0
    if cur:
        eps.append(cur)
    if eps:
        eps_s = np.array(eps) * dt_med
        print(f"  dip episodes: {len(eps)}  median={np.median(eps_s):.2f}s  p90={np.percentile(eps_s,90):.2f}s")

    # ── Standstill creep ─────────────────────────────────────────────────
    print("\n── Standstill creep:")
    acc_var = np.full(n, np.nan)
    half = max(1, int(0.5 / dt_med))
    for i in range(half, n - half):
        acc_var[i] = np.std(acc_norm[i - half:i + half])
    stop = ok_gps & (gps_v < STOP_MAX_MPS) & np.isfinite(gp)
    stop_imu = (acc_var < 0.35) & np.isfinite(gp)   # quiet IMU (rough walking-free band)
    for name, m in (("GPS-stop ", stop), ("IMU-quiet", stop_imu)):
        if m.sum() > 10:
            g = gp[m] * 3.6
            print(f"  {name}: {m.sum()} frames ({m.sum()*dt_med:.0f}s)  gp p50={np.percentile(g,50):.1f} "
                  f"p90={np.percentile(g,90):.1f} max={g.max():.1f} km/h  "
                  f">1km/h={np.mean(g>1)*100:.0f}%  >3km/h={np.mean(g>3)*100:.0f}%")
            print(f"            is_static asserted on {static_bit[m].mean()*100:.0f}% of these frames")
    # fresh-votes vs pure-decay during stops: rising gp at a stop = fresh (noise) votes
    if stop.sum() > 10:
        idx = np.where(stop)[0]
        rises = sum(1 for i in idx if i > 0 and np.isfinite(gp[i - 1]) and gp[i] > gp[i - 1] + 1e-6)
        print(f"  during GPS-stops gp ROSE on {rises}/{len(idx)} frames "
              f"({rises/len(idx)*100:.0f}%) -> fresh (noise) votes, not just decay tail")

    # ── timeline plot ────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 5))
        ax.plot(t, gps_v * 3.6, c="g", lw=1.2, label="GPS")
        ax.plot(t, gp * 3.6, c="tab:blue", lw=0.7, label="gp_flow (IPM)")
        ax.fill_between(t, 0, 80, where=dip, color="red", alpha=0.15, label="dip(<0.5x GPS)")
        ax.fill_between(t, 0, 80, where=decay_flag, color="orange", alpha=0.25, label="0.85^k decay")
        ax.fill_between(t, 0, 80, where=stop, color="gray", alpha=0.25, label="GPS stop")
        ax.set_xlabel("t (s)"); ax.set_ylabel("km/h"); ax.set_ylim(0, 80)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title(path.split("/")[-1])
        out = path.replace(".json", "_ipm_diag.png")
        fig.tight_layout(); fig.savefig(out, dpi=110)
        print(f"\nplot -> {out}")
    except Exception as e:  # matplotlib optional
        print(f"(plot skipped: {e})")


if __name__ == "__main__":
    main()
