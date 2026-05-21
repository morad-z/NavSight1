"""Cross-walk pattern analysis for NavSight bug investigation hive-mind.

Usage:
    python scripts/cross_walk_analysis.py

Compares 6 walks captured 2026-05-20..21 across counter, yaw-stability,
trajectory-shape, MiDaS, and SLAM dimensions. Generates tables suitable
for the cross-walk findings report.

CRITICAL UNITS: `vyaw` and `hdg` in points[] are in RADIANS. Multiply by
180/pi when reporting in degrees.
"""
from __future__ import annotations

import json
import math
import os
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parent.parent
SIM_DIR = REPO / "tests" / "sims" / "regression" / "visual"

WALKS: List[Dict[str, str]] = [
    {"name": "heading_walk_1_2026_05_20", "stage": "pre-fix",
     "json": "heading_walk_1_2026_05_20.json",
     "logcat": "heading_walk_2026_05_20.logcat.txt"},
    {"name": "heading_walk_2_2026_05_20", "stage": "pre-fix",
     "json": "heading_walk_2_2026_05_20.json",
     "logcat": "heading_walk_2026_05_20.logcat.txt"},
    {"name": "parallax_fix_walk_2026_05_20", "stage": "post-front-end-parallax",
     "json": "parallax_fix_walk_2026_05_20.json",
     "logcat": "parallax_fix_walk_2026_05_20.logcat.txt"},
    {"name": "promo_parallax_walk_2026_05_21", "stage": "post-SLAM-promo-parallax",
     "json": "promo_parallax_walk_2026_05_21.json",
     "logcat": "promo_parallax_walk_2026_05_21.logcat.txt"},
    {"name": "bug3_walk_2026_05_21", "stage": "post-Bug-3-rot-sanity",
     "json": "bug3_walk_2026_05_21.json",
     "logcat": "bug3_walk_2026_05_21.logcat.txt"},
    {"name": "bug4_walk_2026_05_21", "stage": "post-Bug-4-REVERTED",
     "json": "bug4_walk_2026_05_21.json",
     "logcat": "bug4_walk_2026_05_21.logcat.txt"},
]


def load_walk(walk_meta: Dict[str, str]) -> Dict[str, Any]:
    path = SIM_DIR / walk_meta["json"]
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def es_get(data: Dict[str, Any], key: str, default: float = 0.0) -> float:
    es = data.get("event_summary", {})
    v = es.get(key, default)
    if v is None:
        return default
    return float(v)


def percentile(values: List[float], p: float) -> float:
    if not values:
        return float("nan")
    values = sorted(values)
    n = len(values)
    if n == 1:
        return values[0]
    k = (n - 1) * p
    f = int(math.floor(k))
    c = int(math.ceil(k))
    if f == c:
        return values[f]
    return values[f] * (c - k) + values[c] * (k - f)


def compute_yaw_stats(data: Dict[str, Any]) -> Dict[str, float]:
    """Yaw stability stats from points[].

    Returns dict with p50/p95/p99 of |vyaw rate deg/s| and |hdg rate deg/s|,
    and total angular variance. vyaw and hdg are RADIANS.
    """
    points = data.get("points", [])
    if len(points) < 2:
        return {}

    vyaw_rates = []  # deg/s of EKF yaw
    hdg_rates = []   # deg/s of compass/Madgwick heading
    vyaw_abs = []
    hdg_abs = []
    dt_sum = 0.0

    for i in range(1, len(points)):
        p0 = points[i - 1]
        p1 = points[i]
        dt = (p1["ts"] - p0["ts"]) / 1000.0  # ms to s
        if dt <= 0 or dt > 1.0:  # skip gaps
            continue
        dt_sum += dt
        # unwrap circular: assume small dt so wrap unlikely
        def unwrap(a):
            while a > math.pi:
                a -= 2 * math.pi
            while a < -math.pi:
                a += 2 * math.pi
            return a
        dyaw = unwrap(p1.get("vyaw", 0.0) - p0.get("vyaw", 0.0))
        dhdg = unwrap(p1.get("hdg", 0.0) - p0.get("hdg", 0.0))
        vyaw_rates.append(abs(math.degrees(dyaw) / dt))
        hdg_rates.append(abs(math.degrees(dhdg) / dt))
        vyaw_abs.append(abs(math.degrees(p1.get("vyaw", 0.0))))
        hdg_abs.append(abs(math.degrees(p1.get("hdg", 0.0))))

    return {
        "ekf_yaw_p50_deg_s": percentile(vyaw_rates, 0.50),
        "ekf_yaw_p95_deg_s": percentile(vyaw_rates, 0.95),
        "ekf_yaw_p99_deg_s": percentile(vyaw_rates, 0.99),
        "ekf_yaw_max_deg_s": max(vyaw_rates) if vyaw_rates else float("nan"),
        "hdg_rate_p50_deg_s": percentile(hdg_rates, 0.50),
        "hdg_rate_p95_deg_s": percentile(hdg_rates, 0.95),
        "hdg_rate_p99_deg_s": percentile(hdg_rates, 0.99),
        "hdg_rate_max_deg_s": max(hdg_rates) if hdg_rates else float("nan"),
        "duration_s": dt_sum,
        "n_samples": len(points),
    }


def compute_trajectory_stats(data: Dict[str, Any]) -> Dict[str, Any]:
    """Trajectory shape stats.

    Returns peak distance, path length, return-to-origin, peak bearing per
    loop. NOTE: x=East,z=North in vio frame on Android; we report planar peak.
    """
    points = data.get("points", [])
    if not points:
        return {}

    xs = [float(p.get("vx", 0.0)) for p in points]
    ys = [float(p.get("vy", 0.0)) for p in points]
    zs = [float(p.get("vz", 0.0)) for p in points]
    # planar distance from origin (use vx,vz on Android per Z-up convention,
    # since vy is up). Compute both x,z plane and x,y plane to be safe.
    planar = [math.hypot(xs[i], zs[i]) for i in range(len(points))]
    if not planar:
        return {}

    peak_dist = max(planar)
    peak_idx = planar.index(peak_dist)
    end_dist = planar[-1]
    end_to_origin_ratio = end_dist / peak_dist if peak_dist > 0 else float("nan")

    # Path length (cumulative)
    path_len = 0.0
    for i in range(1, len(points)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        dz = zs[i] - zs[i - 1]
        path_len += math.sqrt(dx * dx + dy * dy + dz * dz)

    # Heading at peak (atan2 of position)
    peak_bearing_deg = math.degrees(math.atan2(zs[peak_idx], xs[peak_idx]))

    # Crude loop split: find two halves and compute peak bearing in each
    half = len(points) // 2
    p1_xs, p1_zs = xs[:half], zs[:half]
    p2_xs, p2_zs = xs[half:], zs[half:]
    p1_planar = [math.hypot(p1_xs[i], p1_zs[i]) for i in range(len(p1_xs))]
    p2_planar = [math.hypot(p2_xs[i], p2_zs[i]) for i in range(len(p2_xs))]
    p1_peak_idx = p1_planar.index(max(p1_planar)) if p1_planar else 0
    p2_peak_idx = p2_planar.index(max(p2_planar)) if p2_planar else 0
    loop1_peak_dist = max(p1_planar) if p1_planar else 0.0
    loop2_peak_dist = max(p2_planar) if p2_planar else 0.0
    loop1_peak_bearing_deg = (math.degrees(math.atan2(p1_zs[p1_peak_idx], p1_xs[p1_peak_idx]))
                              if p1_xs else float("nan"))
    loop2_peak_bearing_deg = (math.degrees(math.atan2(p2_zs[p2_peak_idx], p2_xs[p2_peak_idx]))
                              if p2_xs else float("nan"))
    bearing_drift_deg = ((loop2_peak_bearing_deg - loop1_peak_bearing_deg + 540) % 360) - 180

    return {
        "peak_dist_m": peak_dist,
        "end_dist_m": end_dist,
        "end_to_peak_ratio": end_to_origin_ratio,
        "path_len_m": path_len,
        "peak_bearing_deg": peak_bearing_deg,
        "loop1_peak_dist_m": loop1_peak_dist,
        "loop2_peak_dist_m": loop2_peak_dist,
        "loop1_peak_bearing_deg": loop1_peak_bearing_deg,
        "loop2_peak_bearing_deg": loop2_peak_bearing_deg,
        "loop_bearing_drift_deg": bearing_drift_deg,
        "n_points": len(points),
    }


# Counters of interest (rows of the cross-walk table)
ROW_COUNTERS = [
    # SLAM / promo
    ("slam_promo_candidates_total", "SLAM promo candidates"),
    ("slam_promo_rejected_parallax_total", "SLAM promo rejected parallax"),
    ("slam_promo_rejected_baseline_total", "SLAM promo rejected baseline"),
    ("slam_promo_rejected_chirality_total", "SLAM promo rejected chirality"),
    ("slam_promo_rejected_rms_total", "SLAM promo rejected RMS"),
    ("slam_promotions_total", "SLAM promotions"),
    ("slam_promotions_seeded_with_midas", "SLAM promo seeded with MiDaS"),
    ("slam_live_updates_fired", "SLAM live updates fired"),
    ("slam_live_updates_skipped", "SLAM live updates skipped"),
    # Landmark / SLAM observation
    ("landmark_map_size", "Landmark map size"),
    ("landmarks_added_total", "Landmarks added"),
    ("landmarks_observed_total", "Landmarks observed"),
    ("landmarks_matched_total", "Landmarks matched"),
    ("landmarks_merged_total", "Landmarks merged"),
    ("landmarks_reanchored_total", "Landmarks re-anchored"),
    ("landmarks_msckf_accepted_total", "MSCKF accepted"),
    ("landmarks_msckf_rejected_chi2_total", "MSCKF chi2 rejects"),
    ("landmarks_rendered_anchor_total", "Rendered anchor (UI)"),
    ("landmarks_rendered_world_fixed_total", "Rendered world-fixed (UI)"),
    # Loop closure
    ("loop_closure_attempts", "LC attempts"),
    ("loop_closure_accepts", "LC accepts"),
    ("loop_closure_rejects_low_score", "LC rej low_score"),
    ("loop_closure_rejects_pnp_total", "LC rej PnP"),
    ("loop_closure_rejects_rot_sanity_total", "LC rej rot_sanity (Bug3)"),
    ("loop_closure_rejects_heading_total", "LC rej heading"),
    ("loop_closure_geom_rejects_descriptor", "LC geom rej descriptor"),
    ("loop_closure_corrections_applied", "LC corrections applied"),
    ("loop_closure_chi2_rejected", "LC chi2 rejected"),
    # MiDaS
    ("midas_depth_samples", "MiDaS depth samples"),
    ("midas_entries", "MiDaS entries"),
    ("midas_affine_fit_inlier_ratio_milli", "MiDaS affine inlier ratio (milli)"),
    ("midas_affine_fit_shift_milli", "MiDaS affine shift (milli)"),
    ("midas_fused", "MiDaS fused"),
    ("midas_bailout_few_floor_matches", "MiDaS bailout few-floor"),
    ("midas_bailout_few_pts3d", "MiDaS bailout few-pts3d"),
    ("midas_skipped_no_disparity", "MiDaS skipped no-disparity"),
    # Yaw / rotation / pure-rot
    ("visual_yaw_chi2_rejected_total", "Visual yaw chi2 rejected"),
    ("visual_yaw_gated_pure_rotation_total", "Visual yaw gated pure-rot"),
    ("visual_relative_rotation_chi2_rejected_total", "Vis rel-rot chi2 rejected"),
    ("relative_rotation_rejected", "Rel rotation rejected"),
    ("visual_relative_rotation_gyro_mismatch_total", "Vis rel-rot gyro mismatch"),
    ("rot_gate_pure_rot_confirmed", "Rot gate pure-rot confirmed"),
    ("madgwick_visual_yaw_nudges_total", "Madg visual yaw nudges (Bug5)"),
    # Stationary / static
    ("zrup_fired_total", "ZUPT fired"),
    ("velocity_clamped", "Velocity clamped"),
    ("global_t_gated_rotation_dominated_total", "global_t gated rot_dom"),
    ("global_t_frozen_static_total", "global_t frozen is_static"),
    ("slam_update_gated_static_total", "SLAM update gated static"),
    ("msckf_gated_static_total", "MSCKF gated static"),
    # Reloc / ORB
    ("reloc_orb_accepts", "Reloc ORB accepts"),
    ("reloc_orb_rejects", "Reloc ORB rejects"),
    ("reloc_orb_size_skipped", "Reloc ORB size skipped"),
    # Pose graph
    ("pose_graph_apply_calls", "PG apply calls"),
    ("pose_graph_optimize_calls", "PG optimize calls"),
    ("pose_graph_loop_edges_added", "PG loop edges added"),
    ("pose_graph_max_correction_mm", "PG max correction (mm)"),
    ("pose_graph_max_correction_mrad", "PG max correction (mrad)"),
    ("pose_graph_residual_norm_pre_mm", "PG residual pre (mm)"),
    ("pose_graph_residual_norm_post_mm", "PG residual post (mm)"),
    # BA
    ("ba_solves_accepted", "BA solves accepted"),
    ("ba_solves_rejected", "BA solves rejected"),
    ("ba_skipped_too_few_landmarks", "BA skipped few landmarks"),
    # Health
    ("gyro_bias_pushed_total", "Gyro bias pushed"),
    ("extrinsics_rotation_angle_mdeg", "Extrinsics rot angle (mdeg)"),
    ("translation_heading_projection_total", "Trans heading projection"),
    ("visual_translation_degenerate_total", "Vis trans degenerate"),
    # Path
    ("total_path_dm", "Total path (dm)"),
]


def main() -> int:
    walks_data = []
    for w in WALKS:
        try:
            d = load_walk(w)
        except Exception as e:
            print(f"FAILED to load {w['name']}: {e}", file=sys.stderr)
            continue
        traj = compute_trajectory_stats(d)
        yaw = compute_yaw_stats(d)
        walks_data.append({"meta": w, "data": d, "traj": traj, "yaw": yaw})

    print("=" * 80)
    print("CROSS-WALK ANALYSIS — 6 walks, 2026-05-20..21")
    print("=" * 80)

    # Header
    print("\nWalk index:")
    for i, w in enumerate(walks_data):
        m = w["meta"]
        traj = w["traj"]
        yaw = w["yaw"]
        print(f"  [{i + 1}] {m['name']}")
        print(f"      stage:    {m['stage']}")
        print(f"      duration: {yaw.get('duration_s', float('nan')):.1f}s, "
              f"{yaw.get('n_samples', 0)} points")
        print(f"      path:     {traj.get('path_len_m', 0):.1f} m, "
              f"peak {traj.get('peak_dist_m', 0):.2f} m, "
              f"end {traj.get('end_dist_m', 0):.2f} m")

    # Counter table
    print("\n" + "=" * 80)
    print("COUNTER TREND TABLE  (W1..W6 columns = walks 1..6 above)")
    print("=" * 80)
    header = f"{'Counter':<42s} " + " ".join(f"{'W' + str(i+1):>10s}" for i in range(len(walks_data)))
    print(header)
    print("-" * len(header))
    for key, label in ROW_COUNTERS:
        vals = [es_get(w["data"], key) for w in walks_data]
        # skip rows where all zero
        if all(v == 0 for v in vals):
            continue
        cells = []
        for v in vals:
            if v >= 1e6:
                cells.append(f"{v / 1e6:>8.2f}M")
            elif v >= 1e3:
                cells.append(f"{v / 1e3:>8.1f}k")
            else:
                cells.append(f"{v:>10.0f}")
        print(f"{label:<42s} " + " ".join(cells))

    # Yaw stability table
    print("\n" + "=" * 80)
    print("YAW STABILITY  (deg/s — vyaw = EKF, hdg = compass/Madgwick blend)")
    print("=" * 80)
    yaw_rows = [
        ("ekf_yaw_p50_deg_s",  "EKF yaw rate p50 deg/s"),
        ("ekf_yaw_p95_deg_s",  "EKF yaw rate p95 deg/s"),
        ("ekf_yaw_p99_deg_s",  "EKF yaw rate p99 deg/s"),
        ("ekf_yaw_max_deg_s",  "EKF yaw rate MAX deg/s"),
        ("hdg_rate_p50_deg_s", "hdg rate p50 deg/s"),
        ("hdg_rate_p95_deg_s", "hdg rate p95 deg/s"),
        ("hdg_rate_p99_deg_s", "hdg rate p99 deg/s"),
        ("hdg_rate_max_deg_s", "hdg rate MAX deg/s"),
    ]
    header = f"{'Metric':<32s} " + " ".join(f"{'W' + str(i+1):>10s}" for i in range(len(walks_data)))
    print(header)
    print("-" * len(header))
    for key, label in yaw_rows:
        vals = [w["yaw"].get(key, float("nan")) for w in walks_data]
        cells = " ".join(f"{v:>10.2f}" for v in vals)
        print(f"{label:<32s} {cells}")

    # Trajectory shape table
    print("\n" + "=" * 80)
    print("TRAJECTORY SHAPE")
    print("=" * 80)
    traj_rows = [
        ("peak_dist_m",          "Peak distance from origin (m)"),
        ("end_dist_m",           "End distance from origin (m)"),
        ("end_to_peak_ratio",    "End/peak ratio"),
        ("path_len_m",           "Path length (m)"),
        ("loop1_peak_dist_m",    "Loop-1 (first half) peak dist (m)"),
        ("loop2_peak_dist_m",    "Loop-2 (second half) peak dist (m)"),
        ("loop1_peak_bearing_deg", "Loop-1 peak bearing (deg)"),
        ("loop2_peak_bearing_deg", "Loop-2 peak bearing (deg)"),
        ("loop_bearing_drift_deg", "Loop bearing drift L2-L1 (deg)"),
    ]
    print(header)
    print("-" * len(header))
    for key, label in traj_rows:
        vals = [w["traj"].get(key, float("nan")) for w in walks_data]
        cells = " ".join(f"{v:>10.2f}" for v in vals)
        print(f"{label:<32s} {cells}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
