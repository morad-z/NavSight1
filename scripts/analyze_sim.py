#!/usr/bin/env python3
"""
Reusable analyzer for NavSight simulation recordings.

Input: one or more simulation_data_*.json files produced by the simulator
  format: {"startTime": <epoch_ms>, "points": [ {ts, vx, vy, vz, vyaw, vsc,
  vql, rx, ry, rz, ryaw, ax, ay, az, gx, gy, gz, glat, glng, galt, gacc,
  mflow, inl, steps, sfreq, stride, pflags, hdg}, ... ]}

Meanings:
  vx/vy/vz   — VIO position in meters (world frame)
  vyaw       — VIO yaw (heading) in radians
  vsc        — EKF scale (smooth_scale_)
  vql        — VIO quality [0..1]
  rx/ry/rz   — reference position (may be zero if no GT source)
  ryaw       — reference yaw
  ax/ay/az   — accelerometer m/s^2
  gx/gy/gz   — gyro rad/s
  glat/glng  — GPS lat/lng (may be null or jammed in Haifa)
  gacc       — GPS accuracy in meters
  mflow      — average optical flow magnitude
  inl        — feature inliers
  steps      — PDR step count
  sfreq      — step frequency Hz
  stride     — current stride length estimate
  hdg        — fused heading
  pflags     — pose flags bitfield

What this script reports (self-consistency first, GPS only as reference
because GPS can be jammed in the deployment area):
  - duration, sample count, sample rate
  - total VIO path length (sum of frame-to-frame displacements)
  - straight-line VIO displacement (first-to-last)
  - closed-loop gap (if start ~= end, report return error)
  - heading range, turn count, heading gap at loop return
  - ryaw RMSE vs vyaw (only when ryaw has real content)
  - step count vs VIO forward motion — implied stride
  - scale stability (mean, std, min, max)
  - inlier rate, stationary fraction
  - GPS (if present) shown as secondary reference with caveat
  - VIO-vs-GPS endpoint displacement, clearly labeled as DELTA not ERROR
  - Step 8 event counters (TD offset, extrinsics angle, path_dm)

Usage:
  python scripts/analyze_sim.py simulator/simulation_data_*.json
  python scripts/analyze_sim.py --plot simulator/simulation_data_1775734294613.json
  python scripts/analyze_sim.py --compare-baseline baseline.json new.json
"""

import argparse
import json
import math
import os
import sys
from statistics import mean, stdev


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def analyze(path, quiet=False):
    """Analyze a sim JSON. Returns a dict of key metrics for --compare-baseline."""
    with open(path, "r", encoding="utf-8") as fp:
        d = json.load(fp)

    pts = d["points"]
    n = len(pts)
    if n < 2:
        if not quiet:
            print(f"[{os.path.basename(path)}] only {n} points, skipping")
        return {}

    t0 = pts[0]["ts"]
    t1 = pts[-1]["ts"]
    duration_s = (t1 - t0) / 1000.0
    rate_hz = n / duration_s if duration_s > 0 else 0.0

    # --- VIO path length and endpoint displacement ---
    path_len = 0.0
    for i in range(1, n):
        dx = pts[i]["vx"] - pts[i - 1]["vx"]
        dy = pts[i]["vy"] - pts[i - 1]["vy"]
        dz = pts[i]["vz"] - pts[i - 1]["vz"]
        path_len += math.sqrt(dx * dx + dy * dy + dz * dz)

    dx = pts[-1]["vx"] - pts[0]["vx"]
    dy = pts[-1]["vy"] - pts[0]["vy"]
    dz = pts[-1]["vz"] - pts[0]["vz"]
    straight_line = math.sqrt(dx * dx + dy * dy + dz * dz)
    horiz_line = math.sqrt(dx * dx + dz * dz)  # 2D (Y is usually up)

    # --- Closed-loop detection ---
    loop = path_len > 5.0 * straight_line and path_len > 5.0
    loop_gap = straight_line if loop else None

    # --- Heading analysis ---
    headings = [p["vyaw"] for p in pts]
    hdg_min = min(headings)
    hdg_max = max(headings)
    # Unwrap and compute total turning
    unwrapped = [headings[0]]
    for i in range(1, n):
        delta = norm_angle(headings[i] - headings[i - 1])
        unwrapped.append(unwrapped[-1] + delta)
    total_turn_rad = unwrapped[-1] - unwrapped[0]
    # Count direction changes (sign flip of unwrapped derivative over 5°)
    TURN_EPS = math.radians(5.0)
    turns = 0
    last_sign = 0
    for i in range(1, n):
        dh = unwrapped[i] - unwrapped[i - 1]
        if abs(dh) < TURN_EPS / 100:
            continue
        sign = 1 if dh > 0 else -1
        if sign != last_sign and last_sign != 0:
            turns += 1
        last_sign = sign

    # Heading gap at loop return (vyaw[end] - vyaw[start], angle-wrapped)
    # For a perfect loop this should be 0; drift shows here.
    heading_gap_deg = math.degrees(norm_angle(headings[-1] - headings[0]))

    # ryaw RMSE vs vyaw: only meaningful when ryaw has actual content
    ryaws = [p.get("ryaw", 0.0) or 0.0 for p in pts]
    ryaw_range = max(ryaws) - min(ryaws)
    heading_rmse_deg = None
    if ryaw_range > math.radians(10.0):  # real reference, not all-zero
        sq_sum = sum(norm_angle(v - r) ** 2 for v, r in zip(headings, ryaws))
        heading_rmse_deg = math.degrees(math.sqrt(sq_sum / n))

    # --- Step analysis ---
    step_start = pts[0]["steps"]
    step_end = pts[-1]["steps"]
    steps = step_end - step_start
    stride_end = pts[-1]["stride"]
    sfreq_last = pts[-1]["sfreq"]
    implied_pdr_distance = steps * stride_end
    implied_stride = (path_len / steps) if steps > 0 else None

    # --- Scale stability ---
    scales = [p["vsc"] for p in pts if p.get("vsc") is not None]
    sc_mean = mean(scales) if scales else 0.0
    sc_std = stdev(scales) if len(scales) > 1 else 0.0
    sc_min = min(scales) if scales else 0.0
    sc_max = max(scales) if scales else 0.0

    # --- Quality and inliers ---
    quals = [p["vql"] for p in pts]
    qual_mean = mean(quals)
    inliers = [p["inl"] for p in pts if p.get("inl") is not None]
    inl_mean = mean(inliers) if inliers else 0.0
    inl_frames_with_matches = sum(1 for i in inliers if i >= 20)

    # --- Stationary fraction (low accel variance + low gyro) ---
    stationary = 0
    for p in pts:
        gmag = math.sqrt(p["gx"] ** 2 + p["gy"] ** 2 + p["gz"] ** 2)
        if gmag < 0.05:
            stationary += 1
    stat_frac = stationary / n

    # --- GPS reference (with caveat) ---
    gps_pts = [p for p in pts if p.get("glat") is not None and p.get("glng") is not None]
    gps_dist_m = None
    gps_accuracy_mean = None
    gps_endpoint_to_start = None
    if len(gps_pts) >= 2:
        gps_path = 0.0
        for i in range(1, len(gps_pts)):
            gps_path += haversine_m(
                gps_pts[i - 1]["glat"], gps_pts[i - 1]["glng"],
                gps_pts[i]["glat"], gps_pts[i]["glng"],
            )
        gps_dist_m = gps_path
        accs = [p["gacc"] for p in gps_pts if p.get("gacc") is not None]
        gps_accuracy_mean = mean(accs) if accs else None
        gps_endpoint_to_start = haversine_m(
            gps_pts[0]["glat"], gps_pts[0]["glng"],
            gps_pts[-1]["glat"], gps_pts[-1]["glng"],
        )

    # --- Event summary fields ---
    es = d.get("event_summary") or {}

    def g(k):
        v = es.get(k)
        return int(v) if isinstance(v, (int, float)) else 0

    ba_total = g("ba_solves_total")
    ba_iters = g("ba_iters_sum")
    avg_iters = (ba_iters / ba_total) if ba_total > 0 else 0.0

    # Step 8 fields (may be 0 in pre-Step-8 sims)
    td_offset_us  = g("cam_imu_time_offset_us")   # µs, signed
    td_offset_ms  = td_offset_us / 1000.0
    extr_mdeg     = g("extrinsics_rotation_angle_mdeg")
    extr_deg      = extr_mdeg / 1000.0
    path_dm       = g("total_path_dm")             # decimetres
    path_from_ev  = path_dm / 10.0                 # metres

    lc_attempts = g('loop_closure_attempts')
    lc_accepts  = g('loop_closure_accepts')
    lc_low      = g('loop_closure_rejects_low_score')
    lc_pnp      = g('loop_closure_rejects_pnp')
    lc_chi2     = g('loop_closure_chi2_rejected')
    lc_kf_db    = g('loop_closure_kf_count_in_db')
    lc_corr     = g('loop_closure_corrections_applied')

    # Step 3 Observer B — MiDaS depth (added 2026-05-07; will be 0 in
    # pre-counter sims).
    midas_entries     = g('midas_entries')
    midas_no_depth    = g('midas_bailout_no_depth')
    midas_few_pts3d   = g('midas_bailout_few_pts3d')
    midas_invalid_int = g('midas_bailout_invalid_intrinsics')
    midas_camera_h    = g('midas_bailout_camera_h')
    midas_low_g       = g('midas_bailout_low_g')
    midas_few_floor   = g('midas_bailout_few_floor_matches')
    midas_extreme     = g('midas_rejected_extreme')
    midas_fused       = g('midas_fused')
    midas_skipped     = g('midas_skipped')

    # --- Report ---
    if not quiet:
        print("=" * 70)
        print(f"FILE: {os.path.basename(path)}")
        print("=" * 70)
        print(f"  samples        : {n}")
        print(f"  duration       : {duration_s:.1f} s")
        print(f"  rate           : {rate_hz:.1f} Hz")
        print()
        print("VIO TRAJECTORY (self-consistency):")
        print(f"  path length    : {path_len:.2f} m")
        print(f"  straight-line  : {straight_line:.2f} m (3D)")
        print(f"  horiz-line     : {horiz_line:.2f} m (XZ)")
        if loop:
            print(f"  closed loop?   : YES — return gap = {loop_gap:.2f} m "
                  f"({100 * loop_gap / path_len:.1f}% of path)")
        else:
            print("  closed loop?   : NO (open trajectory)")
        if path_from_ev > 0:
            print(f"  path (event_summary): {path_from_ev:.2f} m  "
                  f"(cross-check ratio = {path_from_ev / path_len:.3f})")
        print()
        print("HEADING:")
        print(f"  range          : [{math.degrees(hdg_min):+.1f}°, {math.degrees(hdg_max):+.1f}°]")
        print(f"  total turning  : {math.degrees(total_turn_rad):+.1f}° cumulative")
        print(f"  direction flips: {turns}")
        print(f"  loop hdg gap   : {heading_gap_deg:+.2f}°  "
              f"(vyaw[end]-vyaw[start]; 0° = perfect heading closure)")
        if heading_rmse_deg is not None:
            print(f"  ryaw RMSE      : {heading_rmse_deg:.2f}°  "
                  f"(vs ryaw reference, {math.degrees(ryaw_range):.0f}° range)")
        else:
            print("  ryaw RMSE      : N/A (ryaw all-zero / no reference)")
        print()
        print("STEPS / PDR:")
        print(f"  steps detected : {steps}")
        print(f"  stride (final) : {stride_end:.3f} m")
        print(f"  step freq      : {sfreq_last:.2f} Hz")
        print(f"  PDR distance   : {implied_pdr_distance:.2f} m  (steps × stride)")
        if implied_stride is not None:
            print(f"  implied stride : {implied_stride:.3f} m  "
                  f"(VIO path / steps) → ratio vs configured = {implied_stride / stride_end:.2f}x")
        print()
        print("SCALE (EKF smooth_scale_):")
        print(f"  mean±std       : {sc_mean:.4f} ± {sc_std:.4f}")
        print(f"  range          : [{sc_min:.4f}, {sc_max:.4f}]")
        print()
        print("QUALITY:")
        print(f"  quality mean   : {qual_mean:.2f}")
        print(f"  inliers mean   : {inl_mean:.1f}")
        print(f"  frames w/ ≥20 inliers: {inl_frames_with_matches}/{n} "
              f"({100 * inl_frames_with_matches / n:.0f}%)")
        print(f"  stationary frac: {100 * stat_frac:.0f}%  (|gyro| < 0.05)")
        print()
        if gps_dist_m is not None:
            print("GPS (reference only — not ground truth; may be jammed):")
            print(f"  GPS points     : {len(gps_pts)}")
            print(f"  GPS path len   : {gps_dist_m:.2f} m")
            print(f"  GPS endpoint gap: {gps_endpoint_to_start:.2f} m")
            if gps_accuracy_mean is not None:
                print(f"  GPS acc (mean) : {gps_accuracy_mean:.1f} m")
            if path_len > 0 and gps_dist_m > 0:
                ratio = path_len / gps_dist_m
                print(f"  VIO/GPS path ratio: {ratio:.2f}x  "
                      f"({'VIO longer' if ratio > 1.05 else 'GPS longer' if ratio < 0.95 else 'agree'})")
        else:
            print("GPS: no GPS fixes in this recording")
        print()

        if isinstance(es, dict) and es:
            print("EVENT SUMMARY (from event_summary in JSON):")
            print(f"  Step 4 RELOC_ORB        : accepts={g('reloc_orb_accepts')} "
                  f"rejects={g('reloc_orb_rejects')} "
                  f"(size_skip={g('reloc_orb_size_skipped')} "
                  f"slam_guarded={g('reloc_orb_slam_guarded')})")
            print(f"  Step 5 BLUR             : enter={g('blur_enter_events')} "
                  f"total_skip_frames={g('blur_total_skip_frames')}")
            print(f"  Step 5 LOWLIGHT         : active_frames={g('lowlight_state_active_frames')} "
                  f"log_lines={g('lowlight_log_lines')}")
            print(f"  Step 5 ROT_GATE         : pure_rot={g('rot_gate_pure_rot_confirmed')} "
                  f"log_lines={g('rot_gate_log_lines')}")
            print(f"  Step 5 KLT adaptive     : hits={g('klt_adaptive_window_hits')}")
            print(f"  Step 6 BA               : total={ba_total} "
                  f"accepted={g('ba_solves_accepted')} "
                  f"rejected={g('ba_solves_rejected')} "
                  f"skipped_inflight={g('ba_skipped_in_flight')}")
            print(f"  Step 6 BA skip reasons  : no_init={g('ba_skipped_no_init')} "
                  f"few_clones={g('ba_skipped_too_few_clones')} "
                  f"no_intrinsics={g('ba_skipped_no_intrinsics')} "
                  f"few_landmarks={g('ba_skipped_too_few_landmarks')}")
            print(f"  Step 6 BA timing        : sum_us={g('ba_solve_us_sum')} "
                  f"max_us={g('ba_solve_us_max')} avg_iters={avg_iters:.2f}")
            print(f"  Step 3b SLAM promotions : total={g('slam_promotions_total')}")
            print(f"  Step 3 MSCKF            : updates={g('msckf_update_lines')} "
                  f"huber_rej={g('msckf_huber_rejected_sum')}")
            print(f"  Step 7 LOOP CLOSURE     : attempts={lc_attempts} accepts={lc_accepts} "
                  f"corrections_applied={lc_corr}")
            print(f"  Step 7 LC reject reasons: low_score={lc_low} pnp_failed={lc_pnp} "
                  f"chi2_rejected={lc_chi2} (kf_in_db={lc_kf_db})")
            # Step 3 Observer B — MiDaS counters (added 2026-05-07).
            # If midas_entries == 0, MiDaS isn't being CALLED — check
            # Tracker.cpp:1332 callsite or DepthEstimator wiring. If
            # entries > 0 but midas_fused == 0, one of the bailout
            # gates is firing every time — read the breakdown below.
            print(f"  Step 3 MIDAS entries    : {midas_entries}  "
                  f"fused={midas_fused} skipped={midas_skipped}")
            print(f"  Step 3 MIDAS bailouts   : no_depth={midas_no_depth} "
                  f"few_pts3d={midas_few_pts3d} invalid_intr={midas_invalid_int} "
                  f"camera_h={midas_camera_h} low_g={midas_low_g} "
                  f"few_floor={midas_few_floor} extreme={midas_extreme}")
            # Step 8 fields — will be 0 in pre-Step-8 sims
            print(f"  Step 8a TD offset       : {td_offset_ms:+.2f} ms  "
                  f"(warmup ~2 ms; converged value = hw pipeline latency)")
            print(f"  Step 8b extrinsics drift: {extr_deg:.3f}°  "
                  f"(angle from R_bc identity; 180° expected for diag(1,-1,-1))")
            print()
        else:
            print("EVENT SUMMARY: (no event_summary — pre-counter sim)")
            print()

    return {
        "file": os.path.basename(path),
        "duration_s": duration_s,
        "path_len_m": path_len,
        "loop_gap_m": loop_gap,
        "heading_gap_deg": heading_gap_deg,
        "heading_rmse_deg": heading_rmse_deg,
        "td_offset_ms": td_offset_ms,
        "extr_deg": extr_deg,
        "lc_corrections": lc_corr,
        "lc_chi2": lc_chi2,
        "qual_mean": qual_mean,
        "inl_mean": inl_mean,
        "sc_mean": sc_mean,
        "sc_std": sc_std,
        "midas_entries": midas_entries,
        "midas_fused": midas_fused,
    }


def compare_baseline(base_path, new_path):
    """Print side-by-side comparison for Step 8 acceptance criteria."""
    print("=" * 70)
    print("STEP 8 ACCEPTANCE COMPARISON")
    print(f"  BASELINE : {os.path.basename(base_path)}")
    print(f"  NEW      : {os.path.basename(new_path)}")
    print("=" * 70)
    b = analyze(base_path, quiet=True)
    n = analyze(new_path,  quiet=True)
    if not b or not n:
        print("ERROR: could not parse one or both files")
        return

    def row(label, bval, nval, fmt=".2f", unit="", better="lower"):
        bstr = format(bval, fmt) if bval is not None else "N/A"
        nstr = format(nval, fmt) if nval is not None else "N/A"
        if bval is not None and nval is not None:
            delta = nval - bval
            pct = 100.0 * delta / abs(bval) if bval != 0 else float("nan")
            good = (delta < 0) if better == "lower" else (delta > 0)
            arrow = ("↓ BETTER" if good else "↑ WORSE") if not math.isnan(pct) else ""
            note = f"  Δ={delta:+{fmt}} ({pct:+.1f}%)  {arrow}"
        else:
            note = ""
        print(f"  {label:<30}: {bstr:>10}{unit}  →  {nstr:>10}{unit}{note}")

    print()
    print("CRITERION 1 — Heading accuracy:")
    row("loop heading gap",
        b["heading_gap_deg"], n["heading_gap_deg"], fmt=".2f", unit="°", better="lower")
    if b["heading_rmse_deg"] is not None or n["heading_rmse_deg"] is not None:
        row("ryaw RMSE",
            b["heading_rmse_deg"], n["heading_rmse_deg"], fmt=".2f", unit="°", better="lower")

    # Verdict for criterion 1: need ≥10% improvement in heading gap OR ryaw RMSE
    c1_metric = "heading_gap_deg"
    bv = b[c1_metric]
    nv = n[c1_metric]
    if bv is not None and nv is not None and bv != 0:
        pct_improvement = 100.0 * (bv - nv) / abs(bv)
        verdict = "PASS" if pct_improvement >= 10.0 else "FAIL"
        print(f"  Criterion 1 verdict      : {verdict}  "
              f"(improvement = {pct_improvement:.1f}%, need ≥10%)")
    else:
        print("  Criterion 1 verdict      : INCONCLUSIVE (no loop or both zero)")

    print()
    print("CRITERION 2 — TD offset convergence:")
    row("cam-IMU TD offset",
        b["td_offset_ms"], n["td_offset_ms"], fmt=".2f", unit=" ms")
    WARMUP_MS = 2.0
    if n["td_offset_ms"] != 0.0:
        diff_from_warmup = abs(n["td_offset_ms"] - WARMUP_MS)
        verdict2 = "PASS" if diff_from_warmup <= 5.0 else "REVIEW"
        print(f"  Criterion 2 verdict      : {verdict2}  "
              f"(|TD - warmup| = {diff_from_warmup:.1f} ms, need ≤5 ms)")
        if diff_from_warmup > 5.0:
            print("  NOTE: large offset may indicate real hardware latency, "
                  "not a convergence failure — check logcat LC_ABS lines")
    else:
        print("  Criterion 2 verdict      : INCONCLUSIVE (TD=0 in new sim)")

    print()
    print("OTHER METRICS:")
    row("loop position gap",
        b["loop_gap_m"], n["loop_gap_m"], fmt=".2f", unit=" m", better="lower")
    row("loop-closure corrections",
        b["lc_corrections"], n["lc_corrections"], fmt="d", unit="", better="higher")
    row("chi2 rejects",
        b["lc_chi2"], n["lc_chi2"], fmt="d", unit="", better="lower")
    row("extrinsics drift",
        b["extr_deg"], n["extr_deg"], fmt=".3f", unit="°")
    row("quality mean",
        b["qual_mean"], n["qual_mean"], fmt=".2f", better="higher")
    row("inliers mean",
        b["inl_mean"], n["inl_mean"], fmt=".1f", better="higher")
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*", help="simulation_data_*.json files")
    ap.add_argument("--plot", action="store_true",
                    help="matplotlib plot of VIO vs GPS (if available)")
    ap.add_argument("--compare-baseline", nargs=2, metavar=("BASELINE", "NEW"),
                    help="compare two sims for Step 8 acceptance criteria")
    args = ap.parse_args()

    if args.compare_baseline:
        compare_baseline(args.compare_baseline[0], args.compare_baseline[1])
        return

    if not args.files:
        ap.print_help()
        sys.exit(1)

    for f in args.files:
        try:
            analyze(f)
        except Exception as e:
            print(f"[{f}] ERROR: {e}", file=sys.stderr)

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed; skipping --plot", file=sys.stderr)
            return
        for f in args.files:
            with open(f) as fp:
                d = json.load(fp)
            pts = d["points"]
            fig, ax = plt.subplots(figsize=(8, 8))
            xs = [p["vx"] for p in pts]
            zs = [p["vz"] for p in pts]
            ax.plot(xs, zs, "-b", label="VIO (XZ)")
            ax.plot(xs[0], zs[0], "go", markersize=10, label="start")
            ax.plot(xs[-1], zs[-1], "ro", markersize=10, label="end")
            ax.set_aspect("equal")
            ax.set_title(os.path.basename(f))
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Z (m)")
            ax.legend()
            ax.grid(True)
            out = os.path.splitext(f)[0] + "_plot.png"
            fig.savefig(out, dpi=120)
            print(f"  plot saved: {out}")
            plt.close(fig)


if __name__ == "__main__":
    main()
