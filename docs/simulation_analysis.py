"""
NavSight Simulation Analysis — VIO vs GPS Ground Truth
=======================================================
Reads all simulation JSON files from simulator/, produces per-simulation
comparison graphs and a summary report of VIO accuracy.

Usage:
    pip install matplotlib numpy
    python docs/simulation_analysis.py

Output:
    docs/simulation_report_<timestamp>.png  (one multi-panel figure per simulation)
    Console summary with per-simulation metrics and code improvement notes.
"""

import json
import os
import sys
import math
import glob
from datetime import datetime
from pathlib import Path

try:
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")  # non-interactive backend
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
except ImportError:
    print("Install dependencies:  pip install matplotlib numpy")
    sys.exit(1)


# ── Helpers ──────────────────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in meters between two GPS coordinates."""
    R = 6_371_000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def gps_bearing(lat1, lon1, lat2, lon2):
    """Bearing in radians from point 1 to point 2 (0 = North, CW)."""
    dlon = math.radians(lon2 - lon1)
    lat1r, lat2r = math.radians(lat1), math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2r)
    y = (math.cos(lat1r) * math.sin(lat2r) -
         math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon))
    return math.atan2(x, y)


def gps_to_local_meters(points, ref_lat, ref_lon):
    """Convert GPS lat/lon to local ENU meters relative to ref point."""
    xs, ys = [], []
    for p in points:
        dy = haversine_m(ref_lat, ref_lon, p["glat"], ref_lon)
        if p["glat"] < ref_lat:
            dy = -dy
        dx = haversine_m(ref_lat, ref_lon, ref_lat, p["glng"])
        if p["glng"] < ref_lon:
            dx = -dx
        xs.append(dx)
        ys.append(dy)
    return xs, ys


def classify_simulation(points):
    """Classify as 'walking', 'driving', or 'unknown' based on data."""
    has_extended = "steps" in points[0] if points else False

    if not has_extended:
        # Old-format files — check GPS speed
        gps_pts = [p for p in points if p.get("glat") is not None]
        if len(gps_pts) >= 2:
            total_dist = sum(
                haversine_m(gps_pts[i]["glat"], gps_pts[i]["glng"],
                            gps_pts[i+1]["glat"], gps_pts[i+1]["glng"])
                for i in range(len(gps_pts) - 1)
            )
            duration_s = (gps_pts[-1]["ts"] - gps_pts[0]["ts"]) / 1000
            if duration_s > 0:
                avg_speed_kmh = (total_dist / duration_s) * 3.6
                return "driving" if avg_speed_kmh > 6 else "walking"
        return "unknown"

    # Extended format — use step frequency and accel variance
    gps_pts = [p for p in points if p.get("glat") is not None]
    if len(gps_pts) >= 2:
        total_dist = sum(
            haversine_m(gps_pts[i]["glat"], gps_pts[i]["glng"],
                        gps_pts[i+1]["glat"], gps_pts[i+1]["glng"])
            for i in range(len(gps_pts) - 1)
        )
        duration_s = (gps_pts[-1]["ts"] - gps_pts[0]["ts"]) / 1000
        if duration_s > 0:
            avg_speed_kmh = (total_dist / duration_s) * 3.6
            if avg_speed_kmh > 8:
                return "driving"

    last_pts = points[-10:] if len(points) >= 10 else points
    avg_sfreq = np.mean([p.get("sfreq", 0) for p in last_pts])
    if avg_sfreq > 0.8:
        return "walking"
    return "driving" if any(p.get("sfreq", 0) < 0.3 for p in last_pts[-5:]) else "unknown"


# ── Per-Simulation Analysis ─────────────────────────────────────────────────

def analyze_simulation(filepath):
    """Analyze one simulation file. Returns metrics dict."""
    with open(filepath, "r") as f:
        data = json.load(f)

    points = data["points"]
    start_time = data.get("startTime", points[0]["ts"] if points else 0)
    sim_name = Path(filepath).stem
    has_extended = "steps" in points[0] if points else False
    mode = classify_simulation(points)

    # ── Time axis ────────────────────────────────────────────────────────
    t0 = points[0]["ts"]
    times_s = [(p["ts"] - t0) / 1000 for p in points]
    duration_s = times_s[-1] if times_s else 0

    # ── Frame rate ───────────────────────────────────────────────────────
    dts_ms = [points[i+1]["ts"] - points[i]["ts"] for i in range(len(points)-1)]
    avg_dt_ms = np.mean(dts_ms) if dts_ms else 0
    avg_fps = 1000 / avg_dt_ms if avg_dt_ms > 0 else 0

    # ── GPS ground truth ─────────────────────────────────────────────────
    gps_pts = [p for p in points if p.get("glat") is not None and p.get("glng") is not None]
    gps_total_dist = 0
    gps_segments = []
    if len(gps_pts) >= 2:
        ref_lat, ref_lon = gps_pts[0]["glat"], gps_pts[0]["glng"]
        gps_xs, gps_ys = gps_to_local_meters(gps_pts, ref_lat, ref_lon)
        gps_times = [(p["ts"] - t0) / 1000 for p in gps_pts]
        for i in range(len(gps_pts) - 1):
            d = haversine_m(gps_pts[i]["glat"], gps_pts[i]["glng"],
                            gps_pts[i+1]["glat"], gps_pts[i+1]["glng"])
            gps_total_dist += d
            gps_segments.append(d)
    else:
        ref_lat, ref_lon = 0, 0
        gps_xs, gps_ys, gps_times = [], [], []

    gps_straight_line = 0
    if len(gps_pts) >= 2:
        gps_straight_line = haversine_m(
            gps_pts[0]["glat"], gps_pts[0]["glng"],
            gps_pts[-1]["glat"], gps_pts[-1]["glng"]
        )

    # ── VIO trajectory ───────────────────────────────────────────────────
    vio_x = [p["vx"] for p in points]
    vio_z = [p["vz"] for p in points]  # Z is forward in our coordinate system
    vio_dist_from_origin = [math.sqrt(p["vx"]**2 + p["vz"]**2) for p in points]
    vio_total_dist = sum(
        math.sqrt((points[i+1]["vx"] - points[i]["vx"])**2 +
                  (points[i+1]["vz"] - points[i]["vz"])**2)
        for i in range(len(points)-1)
    )
    vio_straight_line = math.sqrt(vio_x[-1]**2 + vio_z[-1]**2) if points else 0

    # ── Quality & scale ──────────────────────────────────────────────────
    qualities = [p["vql"] for p in points]
    scales = [p["vsc"] for p in points]
    yaws = [p["vyaw"] for p in points]

    # ── Extended diagnostics ─────────────────────────────────────────────
    if has_extended:
        mean_flows = [p.get("mflow", 0) for p in points]
        inliers = [p.get("inl", 0) for p in points]
        step_counts = [p.get("steps", 0) for p in points]
        step_freqs = [p.get("sfreq", 0) for p in points]
        strides = [p.get("stride", 0) for p in points]
        pflags = [p.get("pflags", 0) for p in points]
        headings = [p.get("hdg", 0) for p in points]
        accels = [math.sqrt(p.get("ax", 0)**2 + p.get("ay", 0)**2 + p.get("az", 0)**2)
                  for p in points]
    else:
        mean_flows = inliers = step_counts = step_freqs = strides = pflags = headings = accels = None

    # ── Drift metrics ────────────────────────────────────────────────────
    distance_ratio = vio_total_dist / gps_total_dist if gps_total_dist > 0 else float("nan")
    straight_line_ratio = vio_straight_line / gps_straight_line if gps_straight_line > 0 else float("nan")
    drift_error_m = abs(vio_straight_line - gps_straight_line) if gps_straight_line > 0 else float("nan")
    drift_pct = (drift_error_m / gps_straight_line * 100) if gps_straight_line > 0 else float("nan")

    # Fallback frame percentage (pflags & 8 means fallback was used)
    fallback_pct = 0
    if pflags is not None:
        fallback_frames = sum(1 for f in pflags if f & 8)
        fallback_pct = fallback_frames / len(pflags) * 100 if pflags else 0

    # GPS avg speed
    gps_avg_speed_kmh = (gps_total_dist / duration_s * 3.6) if duration_s > 0 else 0

    metrics = {
        "name": sim_name,
        "filepath": filepath,
        "mode": mode,
        "has_extended": has_extended,
        "num_points": len(points),
        "duration_s": duration_s,
        "avg_fps": avg_fps,
        "avg_dt_ms": avg_dt_ms,
        # GPS
        "gps_total_dist_m": gps_total_dist,
        "gps_straight_line_m": gps_straight_line,
        "gps_avg_speed_kmh": gps_avg_speed_kmh,
        "gps_xs": gps_xs, "gps_ys": gps_ys, "gps_times": gps_times,
        "ref_lat": ref_lat, "ref_lon": ref_lon,
        # VIO
        "vio_total_dist_m": vio_total_dist,
        "vio_straight_line_m": vio_straight_line,
        "vio_x": vio_x, "vio_z": vio_z,
        "vio_dist_from_origin": vio_dist_from_origin,
        # Comparison
        "distance_ratio": distance_ratio,
        "straight_line_ratio": straight_line_ratio,
        "drift_error_m": drift_error_m,
        "drift_pct": drift_pct,
        "fallback_pct": fallback_pct,
        # Signals
        "times_s": times_s,
        "qualities": qualities,
        "scales": scales,
        "yaws": yaws,
        "mean_flows": mean_flows,
        "inliers": inliers,
        "step_counts": step_counts,
        "step_freqs": step_freqs,
        "strides": strides,
        "headings": headings,
        "accels": accels,
        "pflags": pflags,
    }
    return metrics


# ── Plotting ─────────────────────────────────────────────────────────────────

def plot_simulation(metrics, output_path):
    """Generate a multi-panel figure for one simulation."""
    has_ext = metrics["has_extended"]
    n_rows = 6 if has_ext else 4

    fig = plt.figure(figsize=(16, 4 * n_rows))
    fig.suptitle(
        f"NavSight Simulation: {metrics['name']}\n"
        f"Mode: {metrics['mode'].upper()} | Duration: {metrics['duration_s']:.1f}s | "
        f"FPS: {metrics['avg_fps']:.1f} ({metrics['avg_dt_ms']:.0f}ms/frame) | "
        f"Points: {metrics['num_points']}",
        fontsize=14, fontweight="bold", y=0.98
    )
    gs = GridSpec(n_rows, 2, figure=fig, hspace=0.4, wspace=0.3)

    # ── 1. Trajectory comparison (top left) ──────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title("Trajectory: VIO vs GPS")
    ax1.plot(metrics["vio_x"], metrics["vio_z"], "b-o", markersize=2, label="VIO", alpha=0.8)
    if metrics["gps_xs"]:
        ax1.plot(metrics["gps_xs"], metrics["gps_ys"], "r-s", markersize=3, label="GPS", alpha=0.8)
    ax1.plot(0, 0, "g*", markersize=15, label="Start")
    if metrics["vio_x"]:
        ax1.plot(metrics["vio_x"][-1], metrics["vio_z"][-1], "b^", markersize=10, label="VIO End")
    if metrics["gps_xs"]:
        ax1.plot(metrics["gps_xs"][-1], metrics["gps_ys"][-1], "r^", markersize=10, label="GPS End")
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.legend(fontsize=8)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)

    # ── 2. Distance from origin over time (top right) ────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_title("Distance from Origin over Time")
    ax2.plot(metrics["times_s"], metrics["vio_dist_from_origin"], "b-", label="VIO", linewidth=1.5)
    if metrics["gps_xs"]:
        gps_dist = [math.sqrt(x**2 + y**2) for x, y in zip(metrics["gps_xs"], metrics["gps_ys"])]
        ax2.plot(metrics["gps_times"], gps_dist, "r--", label="GPS", linewidth=1.5)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Distance (m)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # ── 3. Quality & Scale (row 2 left) ──────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.set_title("Tracking Quality")
    ax3.plot(metrics["times_s"], metrics["qualities"], "g-", linewidth=1.5)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Quality (0-1)")
    ax3.set_ylim(-0.05, 1.05)
    ax3.axhline(y=0.15, color="r", linestyle="--", alpha=0.5, label="Min threshold (0.15)")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ── 4. Heading / Yaw over time (row 2 right) ────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.set_title("Heading over Time")
    yaw_deg = [math.degrees(y) for y in metrics["yaws"]]
    ax4.plot(metrics["times_s"], yaw_deg, "m-", linewidth=1.5, label="VIO yaw")
    if metrics["headings"]:
        hdg_deg = [math.degrees(h) for h in metrics["headings"]]
        ax4.plot(metrics["times_s"], hdg_deg, "c--", linewidth=1, label="2D heading")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Heading (deg)")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ── 5. Metrics summary table (row 3 left) ───────────────────────────
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.axis("off")
    ax5.set_title("Metrics Summary", fontweight="bold")
    table_data = [
        ["GPS total distance", f"{metrics['gps_total_dist_m']:.1f} m"],
        ["VIO total distance", f"{metrics['vio_total_dist_m']:.2f} m"],
        ["Distance ratio (VIO/GPS)", f"{metrics['distance_ratio']:.3f}"],
        ["GPS straight-line", f"{metrics['gps_straight_line_m']:.1f} m"],
        ["VIO straight-line", f"{metrics['vio_straight_line_m']:.2f} m"],
        ["Drift error", f"{metrics['drift_error_m']:.1f} m ({metrics['drift_pct']:.0f}%)"],
        ["GPS avg speed", f"{metrics['gps_avg_speed_kmh']:.1f} km/h"],
        ["Avg quality", f"{np.mean(metrics['qualities']):.2f}"],
        ["Fallback frames", f"{metrics['fallback_pct']:.0f}%"],
    ]
    tbl = ax5.table(cellText=table_data, colLabels=["Metric", "Value"],
                    loc="center", cellLoc="left")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.4)
    for (row, col), cell in tbl.get_celld().items():
        if row == 0:
            cell.set_facecolor("#4472C4")
            cell.set_text_props(color="white", fontweight="bold")
        elif row % 2 == 0:
            cell.set_facecolor("#D9E2F3")

    # ── 6. Scale over time (row 3 right) ─────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.set_title("Scale Factor over Time")
    ax6.plot(metrics["times_s"], metrics["scales"], "orange", linewidth=1.5)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Scale")
    ax6.grid(True, alpha=0.3)

    # ── Extended diagnostics (rows 4-6) ──────────────────────────────────
    if has_ext:
        # Row 4 left: Optical flow
        ax7 = fig.add_subplot(gs[3, 0])
        ax7.set_title("Mean Optical Flow")
        ax7.plot(metrics["times_s"], metrics["mean_flows"], "b-", linewidth=1)
        ax7.axhline(y=0.4, color="r", linestyle="--", alpha=0.5, label="MIN_FLOW (0.4px)")
        ax7.set_xlabel("Time (s)")
        ax7.set_ylabel("Flow (px)")
        ax7.legend(fontsize=8)
        ax7.grid(True, alpha=0.3)

        # Row 4 right: Inliers
        ax8 = fig.add_subplot(gs[3, 1])
        ax8.set_title("Essential Matrix Inliers")
        ax8.bar(metrics["times_s"], metrics["inliers"], width=0.3, color="teal", alpha=0.7)
        ax8.axhline(y=8, color="r", linestyle="--", alpha=0.5, label="MIN_INLIERS (8)")
        ax8.set_xlabel("Time (s)")
        ax8.set_ylabel("Inlier count")
        ax8.legend(fontsize=8)
        ax8.grid(True, alpha=0.3)

        # Row 5 left: Step detection
        ax9 = fig.add_subplot(gs[4, 0])
        ax9.set_title("Step Detection")
        ax9.plot(metrics["times_s"], metrics["step_counts"], "k-", linewidth=1.5, label="Cumulative steps")
        ax9_twin = ax9.twinx()
        ax9_twin.plot(metrics["times_s"], metrics["step_freqs"], "r-", linewidth=1, alpha=0.6, label="Step freq (Hz)")
        ax9.set_xlabel("Time (s)")
        ax9.set_ylabel("Steps")
        ax9_twin.set_ylabel("Frequency (Hz)", color="r")
        ax9.legend(loc="upper left", fontsize=8)
        ax9_twin.legend(loc="upper right", fontsize=8)
        ax9.grid(True, alpha=0.3)

        # Row 5 right: Accelerometer magnitude
        ax10 = fig.add_subplot(gs[4, 1])
        ax10.set_title("Accelerometer Magnitude")
        ax10.plot(metrics["times_s"], metrics["accels"], "purple", linewidth=1, alpha=0.7)
        ax10.axhline(y=9.81, color="gray", linestyle="--", alpha=0.5, label="Gravity (9.81)")
        ax10.axhline(y=10.1, color="r", linestyle="--", alpha=0.3, label="Step thresh high (10.1)")
        ax10.axhline(y=9.3, color="b", linestyle="--", alpha=0.3, label="Step thresh low (9.3)")
        ax10.set_xlabel("Time (s)")
        ax10.set_ylabel("Accel magnitude (m/s^2)")
        ax10.legend(fontsize=7)
        ax10.grid(True, alpha=0.3)

        # Row 6: Stride length & code improvements text
        ax11 = fig.add_subplot(gs[5, 0])
        ax11.set_title("Stride Length over Time")
        ax11.plot(metrics["times_s"], metrics["strides"], "brown", linewidth=1.5)
        ax11.set_xlabel("Time (s)")
        ax11.set_ylabel("Stride (m)")
        ax11.set_ylim(0, 1.5)
        ax11.grid(True, alpha=0.3)

        ax12 = fig.add_subplot(gs[5, 1])
        ax12.axis("off")
    else:
        ax12 = fig.add_subplot(gs[3, :])
        ax12.axis("off")

    # ── Code improvements annotation ────────────────────────────────────
    ax12.set_title("Code Improvements Applied", fontweight="bold")
    improvements = get_improvements_text(metrics)
    ax12.text(0.05, 0.95, improvements, transform=ax12.transAxes,
              fontsize=9, verticalalignment="top", fontfamily="monospace",
              bbox=dict(boxstyle="round", facecolor="#F0F0F0", alpha=0.8))

    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def get_improvements_text(m):
    """Return text describing what was improved based on observed issues."""
    lines = []

    # Frame rate issue
    if m["avg_dt_ms"] > 100:
        lines.append(f"[FIX] Camera at {m['avg_fps']:.1f}fps ({m['avg_dt_ms']:.0f}ms/frame)")
        lines.append("  -> dt guards raised: 0.2s->1.5s (scale), 0.5s->2.0s (accept)")
        lines.append("  -> LK window 21->31px, pyramid 3->4 levels")

    # Distance accuracy
    if not math.isnan(m["distance_ratio"]):
        if m["distance_ratio"] < 0.3:
            lines.append(f"[ISSUE] VIO covers only {m['distance_ratio']:.0%} of GPS distance")
            lines.append("  -> 2D heading-based position (replaces broken 3D forward)")
            lines.append("  -> Step-based scale (replaces accel double-integration)")
        elif m["distance_ratio"] < 0.7:
            lines.append(f"[PARTIAL] VIO covers {m['distance_ratio']:.0%} of GPS distance")
            lines.append("  -> Scale bootstrap: initial=0.20, first 10 obs bypass outlier filter")

    # Fallback frames
    if m["fallback_pct"] > 50:
        lines.append(f"[FIX] {m['fallback_pct']:.0f}% fallback frames (quality gate too strict)")
        lines.append("  -> Quality gate 0.25->0.15, scale gate 0.35->0.15")
        lines.append("  -> FB check relaxed 4.0->9.0 sq-px")

    # Step detection in driving mode
    if m["mode"] == "driving" and m["step_counts"] and m["step_counts"][-1] > 5:
        lines.append(f"[FIX] {m['step_counts'][-1]} false steps detected while driving")
        lines.append("  -> Accel variance filter: walking (var>0.15) vs car (<0.15)")
        lines.append("  -> detectStep() now requires is_walking_pattern_=true")
        lines.append("  -> Vehicle speed via integrated forward accel + friction decay")

    # Walking mode step detection
    if m["mode"] == "walking" and m["step_counts"]:
        lines.append(f"[OK] {m['step_counts'][-1]} steps detected, "
                     f"stride={m['strides'][-1]:.2f}m" if m["strides"] else "")
        lines.append("  -> Step thresholds: HIGH=10.1, LOW=9.3, LP_ALPHA=0.20")

    # Quality
    avg_q = np.mean(m["qualities"])
    if avg_q < 0.3:
        lines.append(f"[WARN] Low avg quality ({avg_q:.2f})")
        lines.append("  -> MAX_FEATURES 200->500, MIN_FEATURES 100->150")
        lines.append("  -> QUALITY_LEVEL 0.03->0.01 for more feature candidates")

    if not lines:
        lines.append("[OK] No major issues detected in this recording")

    return "\n".join(lines)


# ── Console Summary ──────────────────────────────────────────────────────────

def print_summary(all_metrics):
    """Print a console summary table for all simulations."""
    print("\n" + "=" * 80)
    print("NavSight Simulation Analysis Report")
    print(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80)

    for i, m in enumerate(all_metrics):
        print(f"\n{'─' * 80}")
        print(f"Simulation {i+1}: {m['name']}")
        print(f"  Mode: {m['mode'].upper()}")
        print(f"  Duration: {m['duration_s']:.1f}s | Frames: {m['num_points']} | "
              f"FPS: {m['avg_fps']:.1f} ({m['avg_dt_ms']:.0f}ms avg)")
        print(f"  Schema: {'Extended (IMU+flow+steps)' if m['has_extended'] else 'Basic (VIO+GPS)'}")
        print()
        print(f"  GPS total path:     {m['gps_total_dist_m']:>8.1f} m")
        print(f"  VIO total path:     {m['vio_total_dist_m']:>8.2f} m")
        print(f"  Distance ratio:     {m['distance_ratio']:>8.3f}  "
              f"({'VIO < GPS' if m['distance_ratio'] < 1 else 'VIO > GPS'})")
        print(f"  GPS straight-line:  {m['gps_straight_line_m']:>8.1f} m")
        print(f"  VIO straight-line:  {m['vio_straight_line_m']:>8.2f} m")
        print(f"  Drift error:        {m['drift_error_m']:>8.1f} m ({m['drift_pct']:.0f}%)")
        print(f"  GPS avg speed:      {m['gps_avg_speed_kmh']:>8.1f} km/h")
        print(f"  Avg quality:        {np.mean(m['qualities']):>8.2f}")
        if m["fallback_pct"] > 0:
            print(f"  Fallback frames:    {m['fallback_pct']:>7.0f}%")
        if m["step_counts"]:
            print(f"  Total steps:        {m['step_counts'][-1]:>8d}")
            print(f"  Final step freq:    {m['step_freqs'][-1]:>8.2f} Hz")
            print(f"  Final stride:       {m['strides'][-1]:>8.2f} m")

        print()
        print("  Code improvements relevant to this simulation:")
        for line in get_improvements_text(m).split("\n"):
            print(f"    {line}")

    # ── Overall summary ──────────────────────────────────────────────────
    print(f"\n{'=' * 80}")
    print("OVERALL CODE CHANGES (applied 2026-03-31):")
    print("=" * 80)
    print("""
  1. 2D HEADING-BASED POSITION
     Old: global_R_ * (0,0,-1) — invariant under phone tilt (Rz rotation)
     New: heading = atan2(R[1][0], R[1][1]), dx = dist*sin(hdg), dz = -dist*cos(hdg)

  2. STEP-BASED SCALE ESTIMATION
     Old: Accel double-integration (drifts to infinity)
     New: stride_length * step_frequency, stride = 0.4 + 0.3*freq (clamped 0.4-1.2m)

  3. DRIVING MODE (new)
     - Accel variance filter: walking (>0.15) vs car vibrations (<0.15)
     - Vehicle speed from integrated forward accel with 0.995 friction decay
     - Step detector requires is_walking_pattern_ = true

  4. TIMING FIXES (for 2.3fps camera)
     - Scale estimation dt guard: 0.2s -> 1.5s
     - Scale acceptance dt guard: 0.5s -> 2.0s
     - LK optical flow window: 21 -> 31px, pyramid: 3 -> 4

  5. THRESHOLD RELAXATION
     - FB check: 4.0 -> 9.0 sq-px  |  Quality gate: 0.25 -> 0.15
     - Scale gate: 0.35 -> 0.15    |  MIN_INLIERS: 10 -> 8
     - MAX_FEATURES: 200 -> 500    |  QUALITY_LEVEL: 0.03 -> 0.01

  6. ANTI-FALSE-ZUPT
     - Step detector cross-check: if steps detected within 2s, override visual ZUPT
     - Prevents position freezing during walking when flow is low

  7. CAMERA ADVICE
     - Hold phone at 30-45 deg from horizontal, NOT pointing at floor
     - Floor-pointing gives zero depth variation -> scale cannot converge
""")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    # Find project root (relative to this script in docs/)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    sim_dir = project_root / "simulator"
    output_dir = script_dir  # Save outputs to docs/

    if not sim_dir.exists():
        print(f"ERROR: simulator/ folder not found at {sim_dir}")
        sys.exit(1)

    json_files = sorted(glob.glob(str(sim_dir / "simulation_data_*.json")))
    if not json_files:
        print(f"ERROR: No simulation_data_*.json files found in {sim_dir}")
        sys.exit(1)

    print(f"Found {len(json_files)} simulation files in {sim_dir}")

    all_metrics = []
    for i, filepath in enumerate(json_files):
        print(f"\nAnalyzing {Path(filepath).name}...")
        metrics = analyze_simulation(filepath)
        all_metrics.append(metrics)

        # Generate figure
        out_name = f"sim_report_{metrics['name'].replace('simulation_data_', '')}.png"
        plot_simulation(metrics, str(output_dir / out_name))

    print_summary(all_metrics)
    print(f"\nDone. Graphs saved to {output_dir}/sim_report_*.png")


if __name__ == "__main__":
    main()
