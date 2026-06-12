#!/usr/bin/env python3
"""Offline IPM flow probe on real ride frames (2026-06-12).

For adjacent stored frame pairs (dt 25-55 ms) from a recorded ride, on
NEAR-band road points (Z < 6 m, where the speed signal lives):

  1. TRUE displacement via exhaustive template matching (TM, +/-150 px search
     -> no tracker range limit, blur-tolerant since both frames share blur)
  2. what production KLT measures   (win 25, 3 levels, fwd-bwd check)
  3. what Fix-B KLT measures        (win 61, 5 levels, forward-only
                                     = Tracker.cpp:3096)
  4. the IPM per-point speed v_i + forward-coherence gate, replicated exactly
     (Tracker.cpp:9156-9213), fed by each flow source

Cruise pairs answer:  is the near-ground flow DROPPED, UNDER-MEASURED
(aliased), or correctly measured but GATED/diluted?
Standstill pairs answer: where does the 1-5 km/h creep come from
(noise through the one-sided coherence gate, amplified ~Z)?

The road mask is rebuilt PER PAIR from the JSON accel (gravity) — not a fixed
median attitude — avoiding the 2026-06-04 floating-mask trap.

Usage:
  python scripts/ipm_frames_flow_probe_2026_06_12.py [ride.json]
"""
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs opencv-python (pip install opencv-python)")

DEFAULT_JSON = "tests/sims/val_2026_06_03b/simulation_data_1780505700912.json"

# Intrinsics at the 640x480 analyzer (fx from the 2026-06-04 code-comment
# derivation Z=fx*v*dt/41: fx = 5.9*41/(8*0.067) = 451). cx,cy = centre.
FX = 451.0
CX, CY = 320.0, 240.0
CAMERA_H_M = 1.05
# R_bc body->camera, EKFState.cpp:129
R_BC = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
KMIN_Z, KMAX_Z = 0.5, 50.0          # Tracker.cpp:9116-9117
COH_MIN = 1.0 / math.sqrt(2.0)      # Tracker.cpp:9126
PAIR_DT = (0.025, 0.055)
N_CRUISE_PAIRS = 24
N_STOP_PAIRS = 10
TM_PATCH = 31                        # template side
TM_SEARCH_X, TM_SEARCH_Y = 150, 60   # search half-extents (px)
TM_MIN_SCORE = 0.55


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6371000.0 * math.asin(math.sqrt(a))


def load_ride(path):
    with open(path) as fh:
        d = json.load(fh)
    pts = d["points"]
    t_ms = np.array([p["ts"] for p in pts], dtype=np.float64)
    acc = np.array([[p["ax"], p["ay"], p["az"]] for p in pts])
    gyr = np.array([[p["gx"], p["gy"], p["gz"]] for p in pts])
    glat = np.array([p["glat"] if p["glat"] is not None else np.nan for p in pts])
    glng = np.array([p["glng"] if p["glng"] is not None else np.nan for p in pts])
    gacc = np.array([p["gacc"] if p["gacc"] is not None else np.nan for p in pts])
    gp = np.array([p["gp_flow_speed"] if p["gp_flow_speed"] is not None else np.nan for p in pts])
    # jam-aware GPS speed, forward-filled 1.5 s
    n = len(pts)
    t = t_ms / 1000.0
    gps_v = np.full(n, np.nan)
    last = None
    for i in range(n):
        if not np.isfinite(glat[i]) or (np.isfinite(gacc[i]) and gacc[i] > 25.0):
            continue
        if last is not None and 0.2 <= t[i] - t[last] <= 5.0:
            v = haversine_m(glat[last], glng[last], glat[i], glng[i]) / (t[i] - t[last])
            if v <= 25.0:
                gps_v[i] = v
        last = i
    filled = np.full(n, np.nan)
    lv, lt = np.nan, -10.0
    for i in range(n):
        if np.isfinite(gps_v[i]):
            lv, lt = gps_v[i], t[i]
        if t[i] - lt <= 1.5:
            filled[i] = lv
    return t_ms, acc, gyr, filled, gp


def nearest_idx(t_ms, ts):
    return int(np.clip(np.searchsorted(t_ms, ts), 1, len(t_ms) - 1))


def gravity_at(t_ms, acc, ts_ms, half_s=0.3):
    m = (t_ms >= ts_ms - half_s * 1000) & (t_ms <= ts_ms + half_s * 1000)
    g = acc[m].mean(axis=0) if m.sum() else acc[nearest_idx(t_ms, ts_ms)]
    return g / (np.linalg.norm(g) + 1e-12)


def ipm_speeds(prev_pts, next_pts, dt_s, n_up_c, w_cam):
    """Exact replication of Tracker.cpp:9156-9213 (returns per-point dicts)."""
    u_fwd = np.array([0.0, 0.0, 1.0]) - np.array([0.0, 0.0, 1.0]).dot(n_up_c) * n_up_c
    u_fwd /= (np.linalg.norm(u_fwd) + 1e-12)
    wx, wy, wz = w_cam
    out = []
    for (px, py), (nx, ny) in zip(prev_pts, next_pts):
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
        out.append(dict(Z=Z, v_i=v_i, fmag_px=fmag * FX, cos=cos_fa,
                        survived=(v_i < 50.0 and cos_fa < -COH_MIN)))
    return out


def road_band_mask(shape, n_up_c, zlo, zhi):
    h, w = shape
    xs = (np.arange(w) - CX) / FX
    ys = (np.arange(h) - CY) / FX
    XP, YP = np.meshgrid(xs, ys)
    D2 = n_up_c[0] * XP + n_up_c[1] * YP + n_up_c[2]
    inv_z = -D2 / CAMERA_H_M
    with np.errstate(divide="ignore"):
        Z = np.where(inv_z > 0, 1.0 / inv_z, np.inf)
    return ((Z >= zlo) & (Z < zhi)).astype(np.uint8) * 255


def tm_displacement(gray_a, gray_b, pt):
    """Exhaustive template match of a patch around pt; returns (dx, dy, score)."""
    h, w = gray_a.shape
    half = TM_PATCH // 2
    x, y = int(round(pt[0])), int(round(pt[1]))
    if not (half <= x < w - half and half <= y < h - half):
        return None
    tpl = gray_a[y - half:y + half + 1, x - half:x + half + 1]
    if tpl.std() < 4.0:          # flat patch -> TM meaningless
        return None
    x0, x1 = max(0, x - half - TM_SEARCH_X), min(w, x + half + 1 + TM_SEARCH_X)
    y0, y1 = max(0, y - half - TM_SEARCH_Y), min(h, y + half + 1 + TM_SEARCH_Y)
    region = gray_b[y0:y1, x0:x1]
    if region.shape[0] < TM_PATCH or region.shape[1] < TM_PATCH:
        return None
    res = cv2.matchTemplate(region, tpl, cv2.TM_CCOEFF_NORMED)
    _, score, _, loc = cv2.minMaxLoc(res)
    if score < TM_MIN_SCORE:
        return None
    return (x0 + loc[0] + half - x, y0 + loc[1] + half - y, score)


def lk(gray_a, gray_b, pts, win, levels, fb_check):
    p0 = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(
        gray_a, gray_b, p0, None, winSize=(win, win), maxLevel=levels,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 30, 0.01))
    st = st.reshape(-1).astype(bool)
    if fb_check:
        pb, st2, _ = cv2.calcOpticalFlowPyrLK(
            gray_b, gray_a, p1, None, winSize=(win, win), maxLevel=levels,
            criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 30, 0.01))
        d = np.linalg.norm((pb - p0).reshape(-1, 2), axis=1)
        st &= st2.reshape(-1).astype(bool) & (d < 1.5)
    return p1.reshape(-1, 2), st


def analyse_pair(fa, fb, ts_a_ms, ts_b_ms, t_ms, acc, gyr, label, dump_img=None):
    gray_a = cv2.imread(fa, cv2.IMREAD_GRAYSCALE)
    gray_b = cv2.imread(fb, cv2.IMREAD_GRAYSCALE)
    if gray_a is None or gray_b is None:
        return None
    dt_s = (ts_b_ms - ts_a_ms) / 1000.0
    g_body = gravity_at(t_ms, acc, ts_a_ms)
    n_up_c = R_BC @ g_body
    if n_up_c[2] > -0.05:
        return None
    w_body = gyr[nearest_idx(t_ms, (ts_a_ms + ts_b_ms) / 2)] * dt_s
    w_cam = R_BC @ w_body

    near = road_band_mask(gray_a.shape, n_up_c, 0.5, 6.0)
    corners = cv2.goodFeaturesToTrack(gray_a, maxCorners=80, qualityLevel=0.01,
                                      minDistance=8, mask=near)
    if corners is None or len(corners) < 6:
        return None
    pts = corners.reshape(-1, 2)

    # 1. truth (TM)
    tm = [tm_displacement(gray_a, gray_b, p) for p in pts]
    tm_ok = [i for i, r in enumerate(tm) if r is not None]
    if len(tm_ok) < 5:
        return None
    tm_next = {i: (pts[i][0] + tm[i][0], pts[i][1] + tm[i][1]) for i in tm_ok}
    tm_mag = {i: math.hypot(tm[i][0], tm[i][1]) for i in tm_ok}

    # 2/3. trackers
    p_prod, st_prod = lk(gray_a, gray_b, pts, 25, 3, fb_check=True)
    p_fixb, st_fixb = lk(gray_a, gray_b, pts, 61, 5, fb_check=False)

    res = dict(label=label, dt=dt_s, n_pts=len(pts), n_tm=len(tm_ok),
               tm_flow=float(np.median([tm_mag[i] for i in tm_ok])))

    # what each source implies through the exact IPM math (on TM-valid points)
    for name, nxt, st in (("tm", tm_next, None),
                          ("prod", p_prod, st_prod), ("fixb", p_fixb, st_fixb)):
        sel_prev, sel_next = [], []
        for i in tm_ok:
            if st is None:
                sel_prev.append(pts[i]); sel_next.append(nxt[i])
            elif st[i]:
                sel_prev.append(pts[i]); sel_next.append(nxt[i])
        rows = ipm_speeds(sel_prev, sel_next, dt_s, n_up_c, w_cam) if sel_prev else []
        surv = [r["v_i"] for r in rows if r["survived"]]
        res[f"{name}_n"] = len(rows)
        res[f"{name}_surv"] = len(surv)
        res[f"{name}_vmed"] = float(np.median(surv)) if len(surv) >= 5 else float("nan")
        res[f"{name}_flow"] = (float(np.median([r["fmag_px"] for r in rows]))
                               if rows else float("nan"))
    # per-point agreement fixb vs tm
    agree, undr, miss = 0, 0, 0
    for i in tm_ok:
        if not st_fixb[i]:
            miss += 1
            continue
        lk_mag = math.hypot(*(p_fixb[i] - pts[i]))
        if tm_mag[i] > 3.0:
            if lk_mag < 0.7 * tm_mag[i]:
                undr += 1
            elif lk_mag < 1.3 * tm_mag[i]:
                agree += 1
    res["fixb_miss"] = miss
    res["fixb_under"] = undr
    res["fixb_agree"] = agree

    if dump_img:
        vis = cv2.cvtColor(gray_a, cv2.COLOR_GRAY2BGR)
        cnt, _ = cv2.findContours(near, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnt, -1, (0, 200, 255), 1)
        for i in tm_ok:
            p = tuple(np.int32(pts[i]))
            q = tuple(np.int32(tm_next[i]))
            cv2.arrowedLine(vis, p, q, (0, 255, 0), 1, tipLength=0.25)
            if st_fixb[i]:
                r = tuple(np.int32(p_fixb[i]))
                cv2.arrowedLine(vis, p, r, (0, 0, 255), 1, tipLength=0.25)
        cv2.imwrite(dump_img, vis)
    return res


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_JSON
    frames_dir = path.replace(".json", ".frames")
    t_ms, acc, gyr, gps_v, gp = load_ride(path)
    files = sorted(os.listdir(frames_dir))
    ts_ns = np.array([int(f.split(".")[0]) for f in files], dtype=np.int64)
    ts_ms = ts_ns / 1e6

    pairs = [(i, i + 1) for i in range(len(files) - 1)
             if PAIR_DT[0] <= (ts_ms[i + 1] - ts_ms[i]) / 1000.0 <= PAIR_DT[1]]
    print(f"{len(files)} frames, {len(pairs)} adjacent pairs with dt in {PAIR_DT}")

    t0 = t_ms[0]
    cruise_pairs = [p for p in pairs
                    if np.isfinite(gps_v[nearest_idx(t_ms, ts_ms[p[0]])])
                    and gps_v[nearest_idx(t_ms, ts_ms[p[0]])] >= 4.0]
    stop_pairs = [p for p in pairs if (ts_ms[p[0]] - t0) / 1000.0 < 10.0]
    print(f"cruise pairs available={len(cruise_pairs)} stop(first-10s) pairs={len(stop_pairs)}")

    sel_c = cruise_pairs[:: max(1, len(cruise_pairs) // N_CRUISE_PAIRS)][:N_CRUISE_PAIRS]
    sel_s = stop_pairs[:: max(1, len(stop_pairs) // N_STOP_PAIRS)][:N_STOP_PAIRS]

    out_dir = os.path.dirname(path)
    rows_c, rows_s = [], []
    for k, (i, j) in enumerate(sel_c):
        dump = (os.path.join(out_dir, f"probe_cruise_{k}.jpg") if k in (0, len(sel_c) // 2) else None)
        r = analyse_pair(os.path.join(frames_dir, files[i]), os.path.join(frames_dir, files[j]),
                         ts_ms[i], ts_ms[j], t_ms, acc, gyr, "cruise", dump)
        if r:
            r["gps"] = float(gps_v[nearest_idx(t_ms, ts_ms[i])])
            r["gp_live"] = float(gp[nearest_idx(t_ms, ts_ms[i])])
            rows_c.append(r)
    for k, (i, j) in enumerate(sel_s):
        dump = os.path.join(out_dir, "probe_stop_0.jpg") if k == 0 else None
        r = analyse_pair(os.path.join(frames_dir, files[i]), os.path.join(frames_dir, files[j]),
                         ts_ms[i], ts_ms[j], t_ms, acc, gyr, "stop", dump)
        if r:
            rows_s.append(r)

    def agg(rows, key):
        v = [r[key] for r in rows if np.isfinite(r.get(key, np.nan))]
        return float(np.median(v)) if v else float("nan")

    print(f"\n══ CRUISE ({len(rows_c)} pairs analysed) ══")
    if rows_c:
        print(f"  GPS speed                 p50 = {agg(rows_c,'gps')*3.6:6.1f} km/h")
        print(f"  live gp_flow at pair time p50 = {agg(rows_c,'gp_live')*3.6:6.1f} km/h")
        print(f"  TRUE (TM) near-band flow  p50 = {agg(rows_c,'tm_flow'):6.1f} px"
              f"   -> IPM speed {agg(rows_c,'tm_vmed')*3.6:6.1f} km/h")
        print(f"  prod-KLT (25px,3lvl,fb)   flow {agg(rows_c,'prod_flow'):6.1f} px"
              f"   -> IPM speed {agg(rows_c,'prod_vmed')*3.6:6.1f} km/h")
        print(f"  FixB-KLT (61px,5lvl)      flow {agg(rows_c,'fixb_flow'):6.1f} px"
              f"   -> IPM speed {agg(rows_c,'fixb_vmed')*3.6:6.1f} km/h")
        n_tm = sum(r["n_tm"] for r in rows_c)
        miss = sum(r["fixb_miss"] for r in rows_c)
        undr = sum(r["fixb_under"] for r in rows_c)
        agree = sum(r["fixb_agree"] for r in rows_c)
        print(f"  FixB per-point vs truth: tracked {n_tm-miss}/{n_tm}, "
              f"agree(±30%)={agree}, UNDER(>30% low)={undr}")
        surv_tm = sum(r["tm_surv"] for r in rows_c)
        n_ipm_tm = sum(r["tm_n"] for r in rows_c)
        surv_fb = sum(r["fixb_surv"] for r in rows_c)
        n_ipm_fb = sum(r["fixb_n"] for r in rows_c)
        print(f"  coherence-gate pass: truth-flow {surv_tm}/{n_ipm_tm}"
              f"  fixb-flow {surv_fb}/{n_ipm_fb}")

    print(f"\n══ STANDSTILL ({len(rows_s)} pairs analysed) ══")
    if rows_s:
        print(f"  TRUE (TM) flow            p50 = {agg(rows_s,'tm_flow'):6.2f} px (expect ~0)")
        print(f"  FixB-KLT flow             p50 = {agg(rows_s,'fixb_flow'):6.2f} px")
        surv = sum(r["fixb_surv"] for r in rows_s)
        n_all = sum(r["fixb_n"] for r in rows_s)
        vmeds = [r["fixb_vmed"] for r in rows_s if np.isfinite(r["fixb_vmed"])]
        print(f"  coherence-gate pass rate on standstill noise: {surv}/{n_all} "
              f"({(surv/max(1,n_all))*100:.0f}%)  [random direction would give ~25%]")
        print(f"  frames with >=5 survivors (fresh speed vote): "
              f"{sum(1 for r in rows_s if r['fixb_surv']>=5)}/{len(rows_s)}")
        if vmeds:
            print(f"  median fabricated speed when voting: "
                  f"{np.median(vmeds)*3.6:.1f} km/h")
    print("\noverlay JPGs (mask + green=TM truth, red=FixB-KLT): probe_cruise_*.jpg, probe_stop_0.jpg")


if __name__ == "__main__":
    main()
