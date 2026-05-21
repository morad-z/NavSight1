#!/usr/bin/env python3
"""Bug-02 worker — measure gyro bias from sim raw gyro during is_static intervals.

Strategy
--------
1. Load each sim JSON's `points` array.
2. Static-period detection: gyro_norm < kStaticGate AND accel_norm-9.81 < kAccelGate
   for at least N consecutive points (mirrors C++ ZUPT detector at Tracker.cpp:1960).
3. For each static window:
   - report (mean_gx, mean_gy, mean_gz)  -> instantaneous bias estimate
   - report  std    of each axis         -> noise within window
4. Time series of static-window means -> answers "bias growing over time or constant?"
5. Compare against Allan-calibrated  sigma_bg = 7.46e-7 rad/s/sqrt(s)
   over the walk duration to compute expected vs observed drift.

Outputs to stdout only. No file artifacts. No source-code changes.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any

# Allan-variance-calibrated noise (from IMUPreintegrator.h:303-311):
#   sigma_g_n  = 1.98e-4  rad/s/sqrt(Hz)   <-- white noise density
#   sigma_bg_n = 7.46e-7  rad/s/sqrt(s)    <-- bias random walk
ALLAN_SIGMA_G_WHITE = 1.98e-4
ALLAN_SIGMA_BG_RW = 7.46e-7

# Worker's task spec quotes "Allan_sigma_bg = 1e-5 rad/s" and observed
# bias = 4.9e-3 rad/s = 490x sigma. We'll use the actually-calibrated
# value but flag the discrepancy.

# Static detector — matches Tracker.cpp:1960 ZRUP window (kZrupWindow=20)
# but applied retroactively to sim JSON which is downsampled to ~19 Hz.
# Use looser bounds because we don't have the EKF's gyro_buf, just the
# camera-frame-cadence snapshot.
STATIC_GYRO_NORM_GATE_RPS = 0.1     # rad/s, matches kRotationStillGate (Tracker.cpp:2021)
STATIC_ACCEL_DEV_GATE = 0.5         # m/s^2 deviation from 9.81 (loose; raw includes hand jitter)
STATIC_MIN_WINDOW_PTS = 8           # ~0.4 s at 19 Hz


def load_sim(path: Path) -> list[dict[str, Any]]:
    with open(path) as fh:
        d = json.load(fh)
    return d.get("points", [])


def find_static_windows(pts: list[dict[str, Any]]) -> list[tuple[int, int]]:
    """Return list of (start_idx, end_idx) inclusive ranges that are static."""
    in_window = False
    start = 0
    out: list[tuple[int, int]] = []
    for i, p in enumerate(pts):
        gx, gy, gz = p.get("gx", 0.0), p.get("gy", 0.0), p.get("gz", 0.0)
        ax, ay, az = p.get("ax", 0.0), p.get("ay", 0.0), p.get("az", 0.0)
        gnorm = math.sqrt(gx * gx + gy * gy + gz * gz)
        anorm = math.sqrt(ax * ax + ay * ay + az * az)
        is_still = (
            gnorm < STATIC_GYRO_NORM_GATE_RPS
            and abs(anorm - 9.81) < STATIC_ACCEL_DEV_GATE
        )
        if is_still:
            if not in_window:
                in_window = True
                start = i
        else:
            if in_window:
                if i - start >= STATIC_MIN_WINDOW_PTS:
                    out.append((start, i - 1))
                in_window = False
    if in_window and len(pts) - start >= STATIC_MIN_WINDOW_PTS:
        out.append((start, len(pts) - 1))
    return out


def window_stats(pts: list[dict[str, Any]], rng: tuple[int, int]) -> dict[str, Any]:
    s, e = rng
    gxs = [pts[i]["gx"] for i in range(s, e + 1)]
    gys = [pts[i]["gy"] for i in range(s, e + 1)]
    gzs = [pts[i]["gz"] for i in range(s, e + 1)]
    n = len(gxs)
    t0_ms = pts[s]["ts"]
    t1_ms = pts[e]["ts"]
    dur_s = (t1_ms - t0_ms) / 1000.0
    return {
        "start": s,
        "end": e,
        "n": n,
        "t_start_s": t0_ms / 1000.0,
        "dur_s": dur_s,
        "mean_gx": statistics.fmean(gxs),
        "mean_gy": statistics.fmean(gys),
        "mean_gz": statistics.fmean(gzs),
        "std_gx": statistics.pstdev(gxs) if n > 1 else 0.0,
        "std_gy": statistics.pstdev(gys) if n > 1 else 0.0,
        "std_gz": statistics.pstdev(gzs) if n > 1 else 0.0,
        "norm_mean_xy": math.sqrt(
            statistics.fmean(gxs) ** 2 + statistics.fmean(gys) ** 2
        ),
        "norm_mean_xyz": math.sqrt(
            statistics.fmean(gxs) ** 2
            + statistics.fmean(gys) ** 2
            + statistics.fmean(gzs) ** 2
        ),
    }


def report(sim_path: Path) -> None:
    pts = load_sim(sim_path)
    n = len(pts)
    if n < 50:
        print(f"  [skip] {sim_path.name}: only {n} points")
        return
    duration_s = (pts[-1]["ts"] - pts[0]["ts"]) / 1000.0
    rate_hz = (n - 1) / duration_s if duration_s > 0 else 0.0

    print(f"\n=== {sim_path.name} ===")
    print(f"  N={n} pts  duration={duration_s:.1f}s  rate={rate_hz:.1f} Hz")

    # All-walk gyro stats (motion contaminates these; reported for reference only)
    gxs_all = [p["gx"] for p in pts]
    gys_all = [p["gy"] for p in pts]
    gzs_all = [p["gz"] for p in pts]
    print(
        f"  ALL-WALK gyro mean (motion-contaminated):"
        f"  gx={statistics.fmean(gxs_all):+.5f}"
        f"  gy={statistics.fmean(gys_all):+.5f}"
        f"  gz={statistics.fmean(gzs_all):+.5f}  rad/s"
    )

    wins = find_static_windows(pts)
    print(f"  static windows: {len(wins)}")
    if not wins:
        print(
            "  -> no static intervals long enough; bias measurement requires "
            "is_static periods (start/end of walk most reliable)"
        )
        return

    stats_list = [window_stats(pts, r) for r in wins]
    for s in stats_list:
        print(
            f"   win @t={s['t_start_s']-pts[0]['ts']/1000.0:6.1f}s  dur={s['dur_s']:4.1f}s  N={s['n']:3d}  "
            f"mean=({s['mean_gx']:+.5f},{s['mean_gy']:+.5f},{s['mean_gz']:+.5f}) rad/s  "
            f"|XY|={s['norm_mean_xy']:.5f}  |XYZ|={s['norm_mean_xyz']:.5f}  "
            f"std=({s['std_gx']:.5f},{s['std_gy']:.5f},{s['std_gz']:.5f})"
        )

    # Cross-window drift: is bias constant or growing?
    if len(stats_list) >= 2:
        first = stats_list[0]
        last = stats_list[-1]
        d_gx = last["mean_gx"] - first["mean_gx"]
        d_gy = last["mean_gy"] - first["mean_gy"]
        d_gz = last["mean_gz"] - first["mean_gz"]
        dt = last["t_start_s"] - first["t_start_s"]
        print(
            f"  -> bias delta first→last: dgx={d_gx:+.5f} dgy={d_gy:+.5f} dgz={d_gz:+.5f} rad/s "
            f"over {dt:.1f}s"
        )
        # Allan-RW expectation: sigma_b * sqrt(dt)
        rw_expect = ALLAN_SIGMA_BG_RW * math.sqrt(max(dt, 1.0))
        print(
            f"     expected 1-sigma RW change over {dt:.1f}s = {rw_expect:.2e} rad/s "
            f"({rw_expect*180/math.pi:.2e} deg/s)"
        )
        print(
            f"     observed |dgz| / expected = {abs(d_gz)/max(rw_expect,1e-12):.0f}× "
            "(>>1 means thermal/gain drift, not Brownian RW)"
        )

    # Critical: yaw drift from XY bias on Z-up phone in vertical orientation.
    # For a vertical phone the gyro Z axis is roughly the world-Z (yaw) axis,
    # so bias_gz directly integrates to yaw drift. Compute predicted yaw drift
    # over the walk from the most-stable bias estimate (longest static window).
    longest = max(stats_list, key=lambda s: s["dur_s"])
    bias_gz = longest["mean_gz"]
    bias_gz_dps = bias_gz * 180.0 / math.pi
    predicted_yaw_drift_deg = bias_gz_dps * duration_s
    print(
        f"  best-window bias_gz = {bias_gz:+.5f} rad/s = {bias_gz_dps:+.3f} deg/s  "
        f"→ predicted yaw drift over walk = {predicted_yaw_drift_deg:+.1f}°"
    )
    print(
        f"     vs Allan steady-state σ_bg = 7.46e-7 rad/s/√s = "
        f"{ALLAN_SIGMA_BG_RW*180/math.pi*3600:.3e} deg/s/√s (Brownian)"
    )
    # 490x sigma ratio per task spec — recompute against actual Allan
    sigma_floor_rps = ALLAN_SIGMA_BG_RW  # rad/s/sqrt(s); compare per-second
    ratio = abs(bias_gz) / sigma_floor_rps
    print(
        f"     |bias_gz| / σ_bg = {ratio:.1f}× (task spec claims ~490× → indicates "
        f"thermal/gain miscalibration, NOT random-walk regime)"
    )


def main() -> int:
    base = Path(__file__).resolve().parent.parent / "tests/sims/regression/visual"
    targets = [
        "heading_walk_1_2026_05_20.json",
        "heading_walk_2_2026_05_20.json",
        "parallax_fix_walk_2026_05_20.json",
        "bug3_walk_2026_05_21.json",
        "bug4_walk_2026_05_21.json",
        "promo_parallax_walk_2026_05_21.json",
    ]
    print("Bug-02 gyro-bias analysis — measures bias from static intervals")
    print("Allan-calibrated σ_bg = 7.46e-7 rad/s/√s (IMUPreintegrator.h:311)")
    for name in targets:
        p = base / name
        if not p.exists():
            print(f"  [missing] {p}")
            continue
        report(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
