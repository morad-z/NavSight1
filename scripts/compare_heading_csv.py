"""
compare_heading_csv.py — compare two replay-harness CSVs row-by-row.

The replay harness is deterministic (identical input), so any difference
between two runs is caused purely by the code change between them. This script
reports, for the displayed Madgwick heading (`vout_heading_rad`) and the EKF
yaw (`ekf_yaw_rad`):
  * each run's own start / end / max-excursion heading
  * the row-matched divergence (max |Δ|, final Δ, and the FIRST ts where the
    two runs diverge by more than a threshold — i.e. WHEN the code change
    starts to bite)

Usage:
    python scripts/compare_heading_csv.py A.csv B.csv [labelA labelB]
"""
from __future__ import annotations
import csv
import math
import sys


def wrap180(a_deg):
    while a_deg > 180.0:
        a_deg -= 360.0
    while a_deg < -180.0:
        a_deg += 360.0
    return a_deg


def load(path):
    rows = {}
    with open(path, newline="") as fh:
        r = csv.DictReader(fh)
        for row in r:
            try:
                ts = int(row["ts_ns"])
            except (KeyError, ValueError):
                continue
            def fget(k):
                try:
                    return float(row[k])
                except (KeyError, ValueError, TypeError):
                    return float("nan")
            rows[ts] = (fget("vout_heading_rad"), fget("ekf_yaw_rad"))
    return rows


def deg(x):
    return x * 180.0 / math.pi if not math.isnan(x) else float("nan")


def summarize(name, rows, idx):
    vals = [deg(v[idx]) for v in rows.values() if not math.isnan(v[idx])]
    if not vals:
        print(f"  {name}: no data")
        return
    print(f"  {name}: start={vals[0]:+.1f}  end={vals[-1]:+.1f}  "
          f"min={min(vals):+.1f}  max={max(vals):+.1f}  "
          f"net={wrap180(vals[-1]-vals[0]):+.1f}°")


def diverge(a, b, idx, label, thresh_deg=2.0):
    common = sorted(set(a) & set(b))
    if not common:
        print(f"  {label}: no common timestamps"); return
    max_d = 0.0; max_ts = None; first_div_ts = None
    for ts in common:
        va, vb = a[ts][idx], b[ts][idx]
        if math.isnan(va) or math.isnan(vb):
            continue
        d = abs(wrap180(deg(va) - deg(vb)))
        if d > max_d:
            max_d = d; max_ts = ts
        if first_div_ts is None and d > thresh_deg:
            first_div_ts = ts
    final_ts = common[-1]
    fd = wrap180(deg(a[final_ts][idx]) - deg(b[final_ts][idx]))
    t0 = common[0]
    fdiv = f"{(first_div_ts-t0)/1e9:.1f}s" if first_div_ts else "never"
    mts = f"{(max_ts-t0)/1e9:.1f}s" if max_ts else "-"
    print(f"  {label}: max|Δ|={max_d:.1f}° (at {mts})  final Δ={fd:+.1f}°  "
          f"first >{thresh_deg}° at {fdiv}  (n={len(common)})")


def main(argv):
    a_path, b_path = argv[1], argv[2]
    la = argv[3] if len(argv) > 3 else "A"
    lb = argv[4] if len(argv) > 4 else "B"
    a, b = load(a_path), load(b_path)
    print(f"\n=== displayed heading (vout_heading_rad) ===")
    summarize(la, a, 0); summarize(lb, b, 0)
    diverge(a, b, 0, f"{la} vs {lb}")
    print(f"\n=== EKF yaw (ekf_yaw_rad) ===")
    summarize(la, a, 1); summarize(lb, b, 1)
    diverge(a, b, 1, f"{la} vs {lb}")


if __name__ == "__main__":
    main(sys.argv)
