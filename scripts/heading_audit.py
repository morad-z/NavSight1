#!/usr/bin/env python3
"""
Forensic heading audit: compare EKF heading (vyaw) against pure-gyro
integration and GPS course over a single sim recording.

Output:
  - PNG plot at <sim>_heading_audit.png
  - print() report to stdout

Usage:
  python scripts/heading_audit.py tests/sims/simulation_data_*.json
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ----- helpers ---------------------------------------------------------------

def lla_to_enu(lat, lon, lat0, lon0):
    R = 6371000.0
    e = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * R
    return e, n


def norm_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def unwrap(seq):
    """Continuous angle (radians) from wrapped input."""
    out = [seq[0]]
    for v in seq[1:]:
        d = v - out[-1]
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        out.append(out[-1] + d)
    return out


# ----- main analysis ---------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sim")
    args = ap.parse_args()

    with open(args.sim, "r", encoding="utf-8") as fp:
        d = json.load(fp)

    pts = d["points"]
    evt = d.get("event_summary", {})

    t0 = pts[0]["ts"]
    t = [(p["ts"] - t0) / 1000.0 for p in pts]
    vyaw = [p["vyaw"] for p in pts]
    gz = [p["gz"] for p in pts]   # rad/s; assumed body-frame Z. For a hand-held
                                  # phone walked roughly upright, body-Z and
                                  # world-Z are close enough to drive trend
                                  # comparison. This is forensic, not a
                                  # calibration exercise — see header.
    mflow = [p.get("mflow") or 0.0 for p in pts]
    inl = [p.get("inl") or 0 for p in pts]

    # 1) EKF heading: unwrap vyaw to remove +-pi jumps
    vyaw_u = unwrap(vyaw)

    # 2) Pure-gyro heading: trapezoidal integration of gz, anchored at vyaw[0]
    gyro_u = [vyaw_u[0]]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 1.0:
            # bad gap; reuse prev to avoid garbage spikes
            gyro_u.append(gyro_u[-1])
            continue
        gyro_u.append(gyro_u[-1] + 0.5 * (gz[i] + gz[i - 1]) * dt)

    # 3) GPS course heading: pick GPS samples spaced >=3s and >=3m apart, then
    # compute course = atan2(dE, dN) per pair. Snap each course to the time of
    # the *second* sample of the pair.
    gps_pts = []
    for p in pts:
        if p.get("glat") is None or p.get("glng") is None:
            continue
        gps_pts.append(((p["ts"] - t0) / 1000.0, p["glat"], p["glng"]))

    course_t = []
    course_h = []
    if len(gps_pts) >= 2:
        lat0, lon0 = gps_pts[0][1], gps_pts[0][2]
        # Build ENU once
        gps_enu = [(ts, *lla_to_enu(la, lo, lat0, lon0)) for (ts, la, lo) in gps_pts]
        last_idx = 0
        for j in range(1, len(gps_enu)):
            t_j, e_j, n_j = gps_enu[j]
            t_i, e_i, n_i = gps_enu[last_idx]
            dt_seg = t_j - t_i
            de, dn = e_j - e_i, n_j - n_i
            dist = math.hypot(de, dn)
            if dt_seg >= 3.0 and dist >= 3.0:
                course_t.append(t_j)
                # atan2(dE, dN) -> heading from north, east-positive
                course_h.append(math.atan2(de, dn))
                last_idx = j
        if course_h:
            course_h = unwrap(course_h)

    # ---- align course_h offset with vyaw_u so we are comparing *drift*, not
    # absolute frame. The convention difference between VIO yaw (radians, frame
    # = first-pose) and GPS course (radians from north) is a constant; we
    # subtract the mean of (vyaw - course) over the first 60s window with
    # course samples to align them.
    course_aligned = list(course_h)
    if course_h:
        # find vyaw values at the same timestamps (nearest neighbour)
        idx_v = 0
        vy_at_course = []
        for ct in course_t:
            while idx_v + 1 < len(t) and t[idx_v + 1] <= ct:
                idx_v += 1
            vy_at_course.append(vyaw_u[idx_v])
        # use samples in the first 60s of course data for offset
        ct0 = course_t[0]
        offsets = [vy - c for vy, c, ct in zip(vy_at_course, course_h, course_t) if ct - ct0 <= 60.0]
        if not offsets:
            offsets = [vy_at_course[0] - course_h[0]]
        offset = sum(offsets) / len(offsets)
        course_aligned = [c + offset for c in course_h]
    else:
        vy_at_course = []

    # ---- build per-sample (vyaw - gyro) and (vyaw - gps_course) series ----
    diff_visual = [vy - gy for vy, gy in zip(vyaw_u, gyro_u)]  # rad

    # vy at course samples vs aligned course
    diff_gps = [vy - ca for vy, ca in zip(vy_at_course, course_aligned)]  # rad

    # ---- numeric report --------------------------------------------------

    rad2deg = 180.0 / math.pi
    drift_visual_end_deg = diff_visual[-1] * rad2deg
    drift_gps_end_deg = diff_gps[-1] * rad2deg if diff_gps else float("nan")
    # The unwrapped diff can over/undercount full revolutions if the GPS
    # course unwrapping picked the wrong branch. Report the wrapped version
    # too for sanity.
    drift_gps_end_wrapped_deg = (
        norm_pi(diff_gps[-1]) * rad2deg if diff_gps else float("nan")
    )
    drift_visual_end_wrapped_deg = norm_pi(diff_visual[-1]) * rad2deg

    # First time |vyaw - gps_course| crosses 10 deg, 30 deg
    def first_cross(thr_deg):
        thr = thr_deg / rad2deg
        for ct, dv in zip(course_t, diff_gps):
            if abs(dv) >= thr:
                return ct
        return None

    t_cross_10 = first_cross(10.0)
    t_cross_30 = first_cross(30.0)

    # Per 60s window, rate of (vyaw - gps_course) in deg/s using samples that
    # fall inside the window. We use linear regression on (course_t, diff_gps).
    duration = t[-1]
    win_size = 60.0
    n_wins = int(math.ceil(duration / win_size))
    win_rates = []  # (start_s, end_s, slope_deg_per_s, n_samples)
    for w in range(n_wins):
        ws = w * win_size
        we = ws + win_size
        xs = []
        ys = []
        for ct, dv in zip(course_t, diff_gps):
            if ws <= ct < we:
                xs.append(ct)
                ys.append(dv * rad2deg)
        if len(xs) >= 2:
            mx = sum(xs) / len(xs)
            my = sum(ys) / len(ys)
            num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
            den = sum((x - mx) ** 2 for x in xs)
            slope = num / den if den > 0 else 0.0
        else:
            slope = float("nan")
        win_rates.append((ws, we, slope, len(xs)))

    # worst / best windows by |slope|
    valid = [w for w in win_rates if not math.isnan(w[2]) and w[3] >= 2]
    worst = max(valid, key=lambda w: abs(w[2])) if valid else None
    best = min(valid, key=lambda w: abs(w[2])) if valid else None

    def slice_stats(start, end):
        ms = [m for ti, m in zip(t, mflow) if start <= ti < end]
        ns = [v for ti, v in zip(t, inl) if start <= ti < end]
        def msd(xs):
            if not xs:
                return float("nan"), float("nan")
            mu = sum(xs) / len(xs)
            if len(xs) < 2:
                return mu, 0.0
            var = sum((x - mu) ** 2 for x in xs) / (len(xs) - 1)
            return mu, math.sqrt(var)
        mfm, mfs = msd(ms)
        inm, ins = msd(ns)
        return mfm, mfs, inm, ins, len(ms)

    if worst:
        ws_m, ws_s, in_m, in_s, n = slice_stats(worst[0], worst[1])
    if best:
        bs_m, bs_s, bin_m, bin_s, bn = slice_stats(best[0], best[1])

    # event_summary zero/near-zero counters
    zero_counters = []
    for k, v in evt.items():
        if isinstance(v, (int, float)) and v == 0:
            zero_counters.append((k, v))

    # ---- print report ----------------------------------------------------
    print("=" * 72)
    print("HEADING AUDIT  -  " + os.path.basename(args.sim))
    print("=" * 72)
    print(f"duration:                     {duration:.1f} s")
    print(f"samples:                      {len(pts)}")
    print(f"GPS course samples (>=3s,>=3m): {len(course_t)}")
    print()
    print("--- end-of-walk drift -------------------------------------------------")
    print(f"vyaw - pure_gyro    (visual correction applied) : "
          f"{drift_visual_end_deg:+8.2f} deg  (wrapped: {drift_visual_end_wrapped_deg:+7.2f} deg)")
    print(f"vyaw - gps_course   (EKF error vs GPS truth)    : "
          f"{drift_gps_end_deg:+8.2f} deg  (wrapped: {drift_gps_end_wrapped_deg:+7.2f} deg)")
    print()
    print("--- divergence vs gps_course ------------------------------------------")
    print(f"first |vyaw - gps_course| >= 10 deg : "
          f"{('t=%.1fs' % t_cross_10) if t_cross_10 is not None else 'NEVER'}")
    print(f"first |vyaw - gps_course| >= 30 deg : "
          f"{('t=%.1fs' % t_cross_30) if t_cross_30 is not None else 'NEVER'}")
    print()
    print("--- 60s windowed rate of (vyaw - gps_course) [deg/s] -----------------")
    print("    win_start  win_end   slope_deg/s   n_course_samples")
    for ws_, we_, sl, ns in win_rates:
        sl_str = ("%+8.4f" % sl) if not math.isnan(sl) else "    nan "
        print(f"    {ws_:7.1f}  {we_:7.1f}    {sl_str}        {ns}")
    print()
    if worst and best and worst is not best:
        print("--- worst vs best 60s segment ----------------------------------------")
        print(f"WORST window  [{worst[0]:.1f}, {worst[1]:.1f}) s : "
              f"slope = {worst[2]:+.4f} deg/s  ({worst[3]} samples)")
        print(f"  mflow  mean={ws_m:.3f}  std={ws_s:.3f}")
        print(f"  inl    mean={in_m:.2f}  std={in_s:.2f}  ({n} sim samples in window)")
        print(f"BEST  window  [{best[0]:.1f}, {best[1]:.1f}) s : "
              f"slope = {best[2]:+.4f} deg/s  ({best[3]} samples)")
        print(f"  mflow  mean={bs_m:.3f}  std={bs_s:.3f}")
        print(f"  inl    mean={bin_m:.2f}  std={bin_s:.2f}  ({bn} sim samples in window)")
    else:
        print("Could not isolate worst/best windows (insufficient course samples).")
    print()
    print("--- event_summary counters at ZERO -----------------------------------")
    for k, v in zero_counters:
        print(f"    {k:40s} = {v}")
    print()
    print("--- event_summary (selected, non-zero) -------------------------------")
    for k in (
        "loop_closure_attempts", "loop_closure_accepts",
        "loop_closure_rejects_heading", "loop_closure_rejects_pnp",
        "loop_closure_corrections_applied",
        "ba_solves_accepted", "ba_solves_rejected",
        "msckf_huber_rejected_sum", "msckf_update_lines",
        "slam_promotions_total", "klt_adaptive_window_hits",
    ):
        if k in evt:
            print(f"    {k:40s} = {evt[k]}")
    print()

    # ---- plot ------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)

    ax1.plot(t, [v * rad2deg for v in vyaw_u], label="EKF vyaw", linewidth=1.2)
    ax1.plot(t, [v * rad2deg for v in gyro_u], label="pure gyro integral", linewidth=1.0, alpha=0.85)
    if course_t:
        ax1.plot(course_t, [v * rad2deg for v in course_aligned],
                 label="GPS course (offset-aligned)", linestyle="--", marker=".", markersize=3)
    ax1.set_ylabel("heading (deg)")
    ax1.set_title(f"Heading sources -- {os.path.basename(args.sim)}")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best")

    ax2.plot(t, [v * rad2deg for v in diff_visual],
             label="vyaw - gyro  (visual correction)", linewidth=1.0)
    if course_t:
        ax2.plot(course_t, [v * rad2deg for v in diff_gps],
                 label="vyaw - gps_course (EKF error)", linestyle="--", marker=".", markersize=3)
    ax2.axhline(10, color="orange", linestyle=":", alpha=0.6, label="+/-10 deg")
    ax2.axhline(-10, color="orange", linestyle=":", alpha=0.6)
    ax2.axhline(30, color="red", linestyle=":", alpha=0.6, label="+/-30 deg")
    ax2.axhline(-30, color="red", linestyle=":", alpha=0.6)
    if worst:
        ax2.axvspan(worst[0], worst[1], color="red", alpha=0.12, label="worst 60s")
    if best and best is not worst:
        ax2.axvspan(best[0], best[1], color="green", alpha=0.12, label="best 60s")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("delta heading (deg)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="best", fontsize=8)

    plt.tight_layout()
    out_png = os.path.splitext(args.sim)[0] + "_heading_audit.png"
    plt.savefig(out_png, dpi=130)
    print(f"plot saved: {out_png}")


if __name__ == "__main__":
    main()
