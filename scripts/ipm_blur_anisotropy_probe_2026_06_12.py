#!/usr/bin/env python3
"""Discriminate WHY the near-band road flow signal is dead at cruise (2026-06-12).

Three candidate killers, three signatures:

  motion blur   -> gradient energy ALONG the predicted flow direction collapses
                   at cruise vs standstill (blur smears along flow); effect
                   grows with speed.
  lane-aperture -> points ON the lane lines under-read; isolated-blob points fine.
                   (gradient orientation ~ perpendicular to flow -> no info)
  handlebar     -> dead votes spatially CONCENTRATED in the attached-foreground
                   region; rest of the band reads true flow.

Measures, per pair, in the near band (Z<6m) and mid band (6-12m):
  - along/across gradient-energy ratio (Scharr, projected on predicted flow dir)
  - TM-truth flow vs the flow EXPECTED from GPS speed at each point's Z
  - the same split by gradient orientation class (aperture test)

Usage: py -3.13 scripts/ipm_blur_anisotropy_probe_2026_06_12.py [ride.json]
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
from ipm_frames_flow_probe_2026_06_12 import (  # noqa: E402  reuse, don't rewrite
    CAMERA_H_M, CX, CY, FX, R_BC, DEFAULT_JSON,
    gravity_at, load_ride, nearest_idx, road_band_mask, tm_displacement)


def flow_direction_field(shape, n_up_c):
    """Per-pixel predicted ground-flow unit direction (for forward motion)."""
    h, w = shape
    u_fwd = np.array([0.0, 0.0, 1.0]) - np.array([0.0, 0.0, 1.0]).dot(n_up_c) * n_up_c
    u_fwd /= (np.linalg.norm(u_fwd) + 1e-12)
    xs = (np.arange(w) - CX) / FX
    ys = (np.arange(h) - CY) / FX
    XP, YP = np.meshgrid(xs, ys)
    ax = (u_fwd[0] - XP * u_fwd[2])
    ay = (u_fwd[1] - YP * u_fwd[2])
    n = np.sqrt(ax * ax + ay * ay) + 1e-12
    return ax / n, ay / n


def pair_metrics(frames_dir, fa, fb, ts_a, ts_b, t_ms, acc, gps_v, label):
    ga = cv2.imread(os.path.join(frames_dir, fa), cv2.IMREAD_GRAYSCALE)
    gb = cv2.imread(os.path.join(frames_dir, fb), cv2.IMREAD_GRAYSCALE)
    if ga is None or gb is None:
        return None
    dt_s = (ts_b - ts_a) / 1000.0
    g_body = gravity_at(t_ms, acc, ts_a)
    n_up_c = R_BC @ g_body
    if n_up_c[2] > -0.05:
        return None
    dirx, diry = flow_direction_field(ga.shape, n_up_c)
    gx = cv2.Scharr(ga, cv2.CV_32F, 1, 0) / 16.0
    gy = cv2.Scharr(ga, cv2.CV_32F, 0, 1) / 16.0
    g_along = np.abs(gx * dirx + gy * diry)
    g_across = np.abs(-gx * diry + gy * dirx)

    out = dict(label=label, dt=dt_s,
               gps=float(gps_v[nearest_idx(t_ms, ts_a)]))
    for band, zlo, zhi in (("near", 0.5, 6.0), ("mid", 6.0, 12.0)):
        m = road_band_mask(ga.shape, n_up_c, zlo, zhi) > 0
        if m.sum() < 500:
            continue
        out[f"{band}_aniso"] = float(np.mean(g_along[m]) / (np.mean(g_across[m]) + 1e-9))
        out[f"{band}_grad"] = float(np.mean(g_along[m]))

        # TM truth vs expected flow at sampled corners in this band
        corners = cv2.goodFeaturesToTrack(ga, maxCorners=60, qualityLevel=0.01,
                                          minDistance=8,
                                          mask=(m.astype(np.uint8) * 255))
        ratios, on_line, off_line = [], [], []
        if corners is not None and np.isfinite(out["gps"]) and out["gps"] > 2.0:
            for p in corners.reshape(-1, 2):
                r = tm_displacement(ga, gb, p)
                if r is None:
                    continue
                xp, yp = (p[0] - CX) / FX, (p[1] - CY) / FX
                D2 = n_up_c[0] * xp + n_up_c[1] * yp + n_up_c[2]
                inv_z = -D2 / CAMERA_H_M
                if inv_z <= 0:
                    continue
                ax = (abs(dirx[int(p[1]), int(p[0])]) * 0 + 1)  # placeholder keep simple
                # expected flow magnitude px for forward motion at GPS speed
                u_fwd = np.array([0.0, 0.0, 1.0]) - np.array([0.0, 0.0, 1.0]).dot(n_up_c) * n_up_c
                u_fwd /= (np.linalg.norm(u_fwd) + 1e-12)
                a_x = (u_fwd[0] - xp * u_fwd[2]) * inv_z
                a_y = (u_fwd[1] - yp * u_fwd[2]) * inv_z
                exp_px = out["gps"] * math.hypot(a_x, a_y) * dt_s * FX
                if exp_px < 3.0:
                    continue
                meas = math.hypot(r[0], r[1])
                ratios.append(meas / exp_px)
                # aperture class: local gradient orientation vs flow direction
                yy, xx = int(p[1]), int(p[0])
                al, ac = g_along[yy, xx], g_across[yy, xx]
                (on_line if al < 0.5 * ac else off_line).append(meas / exp_px)
        if ratios:
            out[f"{band}_meas_over_exp"] = float(np.median(ratios))
            out[f"{band}_n"] = len(ratios)
        if on_line:
            out[f"{band}_ratio_lineline"] = float(np.median(on_line))
            out[f"{band}_n_line"] = len(on_line)
        if off_line:
            out[f"{band}_ratio_blob"] = float(np.median(off_line))
            out[f"{band}_n_blob"] = len(off_line)
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    frames_dir = path.replace(".json", ".frames")
    t_ms, acc, gyr, gps_v, gp = load_ride(path)
    files = sorted(os.listdir(frames_dir))
    ts_ms = np.array([int(f.split(".")[0]) for f in files], dtype=np.float64) / 1e6
    pairs = [(i, i + 1) for i in range(len(files) - 1)
             if 0.025 <= (ts_ms[i + 1] - ts_ms[i]) / 1000.0 <= 0.055]

    t0 = t_ms[0]
    rows = []
    slow, fast, stop = [], [], []
    for i, j in pairs:
        g = gps_v[nearest_idx(t_ms, ts_ms[i])]
        tt = (ts_ms[i] - t0) / 1000.0
        if tt < 10.0:
            stop.append((i, j))
        elif np.isfinite(g) and 3.0 <= g <= 8.0:
            slow.append((i, j))
        elif np.isfinite(g) and g >= 10.0:
            fast.append((i, j))
    print(f"pairs: stop={len(stop)} slow(11-29km/h)={len(slow)} fast(>36km/h)={len(fast)}")

    for name, lst, cap in (("stop", stop, 8), ("slow", slow, 14), ("fast", fast, 14)):
        sel = lst[:: max(1, len(lst) // cap)][:cap]
        for i, j in sel:
            r = pair_metrics(frames_dir, files[i], files[j], ts_ms[i], ts_ms[j],
                             t_ms, acc, gps_v, name)
            if r:
                rows.append(r)

    def agg(label, key):
        v = [r[key] for r in rows if r["label"] == label and np.isfinite(r.get(key, np.nan))]
        return (float(np.median(v)), len(v)) if v else (float("nan"), 0)

    print(f"\n{'':8s}{'along/across grad':>20s}{'along grad-energy':>20s}")
    for lbl in ("stop", "slow", "fast"):
        a, na = agg(lbl, "near_aniso")
        g, _ = agg(lbl, "near_grad")
        am, _ = agg(lbl, "mid_aniso")
        gm, _ = agg(lbl, "mid_grad")
        print(f"{lbl:8s} near {a:6.2f} mid {am:6.2f}   near {g:6.1f} mid {gm:6.1f}   (n={na})")
    print("  -> motion blur signature: along/across and along-energy FALL with speed")

    print(f"\n{'':8s}{'measured/expected flow (TM truth vs GPS)':>45s}")
    for lbl in ("slow", "fast"):
        for band in ("near", "mid"):
            r, n = agg(lbl, f"{band}_meas_over_exp")
            rl, nl = agg(lbl, f"{band}_ratio_lineline")
            rb, nb = agg(lbl, f"{band}_ratio_blob")
            print(f"{lbl:8s} {band}: all={r:5.2f} (n={n})  on-line={rl:5.2f} (n={nl})  "
                  f"blob={rb:5.2f} (n={nb})")
    print("  -> aperture signature: on-line << blob.  blur: both low at fast.")
    print("  -> note: GPS may be jam-inflated; ratios are lower bounds on truth/GPS mismatch")


if __name__ == "__main__":
    main()
