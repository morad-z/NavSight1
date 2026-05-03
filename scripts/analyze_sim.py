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
  - heading range and turn count (zero-crossings of dyaw/dt)
  - step count vs VIO forward motion — implied stride
  - scale stability (mean, std, min, max)
  - inlier rate, stationary fraction
  - GPS (if present) shown as secondary reference with caveat
  - VIO-vs-GPS endpoint displacement, clearly labeled as DELTA not ERROR

Usage:
  python scripts/analyze_sim.py simulator/simulation_data_*.json
  python scripts/analyze_sim.py --plot simulator/simulation_data_1775734294613.json
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


def analyze(path):
    with open(path, "r", encoding="utf-8") as fp:
        d = json.load(fp)

    pts = d["points"]
    n = len(pts)
    if n < 2:
        print(f"[{os.path.basename(path)}] only {n} points, skipping")
        return

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
    # If the path is > 5x the endpoint displacement it's probably a loop
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
        d = unwrapped[i] - unwrapped[i - 1]
        if abs(d) < TURN_EPS / 100:
            continue
        sign = 1 if d > 0 else -1
        if sign != last_sign and last_sign != 0:
            turns += 1
        last_sign = sign

    # --- Step analysis ---
    step_start = pts[0]["steps"]
    step_end = pts[-1]["steps"]
    steps = step_end - step_start
    stride_end = pts[-1]["stride"]
    sfreq_last = pts[-1]["sfreq"]
    implied_pdr_distance = steps * stride_end
    # Implied stride from VIO distance / steps
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

    # --- Report ---
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
    print()
    print("HEADING:")
    print(f"  range          : [{math.degrees(hdg_min):+.1f}°, {math.degrees(hdg_max):+.1f}°]")
    print(f"  total turning  : {math.degrees(total_turn_rad):+.1f}° cumulative")
    print(f"  direction flips: {turns}")
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+", help="simulation_data_*.json files")
    ap.add_argument("--plot", action="store_true", help="matplotlib plot of VIO vs GPS (if available)")
    args = ap.parse_args()

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
