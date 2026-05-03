#!/usr/bin/env python3
"""Step 7 — Replay scorer.

Consumes the CSV produced by ``tests/cpp/replay_harness`` and emits regression
metrics. CI uses this to fail when a code change drifts the IMU-only EKF
trajectory away from the recorded baseline by more than configured thresholds.

Metrics
-------
* heading_rmse_rad            RMSE between EKF yaw and the recorded ``vyaw``
                              produced by the runtime at recording time.
* drift_per_meter             cumulative |Δ(EKF_xz)| divided by cumulative
                              |Δ(recorded_v_xz)| (unitless ratio).
* loop_closure_gap_m          XZ distance between the first and last EKF pose
                              after the EKF has fully initialized.
* v_shape_yaw_deg             magnitude of the largest yaw swing during the
                              recording (deg). Used as the "180° turn"
                              detector — fixtures named ``*_180.json`` must
                              clear ``--v-shape-min-deg`` (default 170).

Exit codes
----------
0  all metrics within threshold
1  metric regression
2  CSV unreadable / fixture invalid
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


@dataclass
class Row:
    ts_ns: int
    ekf_init: int
    ekf_x: float
    ekf_y: float
    ekf_z: float
    ekf_yaw_rad: float
    sigma_xx: float
    sigma_xz: float
    sigma_zz: float
    recorded_vx: float
    recorded_vy: float
    recorded_vz: float
    recorded_vyaw_rad: float


def _parse_float(value: str) -> float:
    if value == "" or value is None:
        return float("nan")
    try:
        return float(value)
    except ValueError:
        return float("nan")


def load_csv(path: Path) -> List[Row]:
    rows: List[Row] = []
    with path.open("r", newline="") as fh:
        reader = csv.DictReader(fh)
        required = {
            "ts_ns", "ekf_init",
            "ekf_x", "ekf_y", "ekf_z", "ekf_yaw_rad",
            "sigma_xx", "sigma_xz", "sigma_zz",
            "recorded_vx", "recorded_vy", "recorded_vz", "recorded_vyaw_rad",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing required columns: {sorted(missing)}")
        for r in reader:
            rows.append(Row(
                ts_ns=int(r["ts_ns"]),
                ekf_init=int(r["ekf_init"]),
                ekf_x=_parse_float(r["ekf_x"]),
                ekf_y=_parse_float(r["ekf_y"]),
                ekf_z=_parse_float(r["ekf_z"]),
                ekf_yaw_rad=_parse_float(r["ekf_yaw_rad"]),
                sigma_xx=_parse_float(r["sigma_xx"]),
                sigma_xz=_parse_float(r["sigma_xz"]),
                sigma_zz=_parse_float(r["sigma_zz"]),
                recorded_vx=_parse_float(r["recorded_vx"]),
                recorded_vy=_parse_float(r["recorded_vy"]),
                recorded_vz=_parse_float(r["recorded_vz"]),
                recorded_vyaw_rad=_parse_float(r["recorded_vyaw_rad"]),
            ))
    return rows


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_rmse_rad(rows: List[Row]) -> Optional[float]:
    diffs: List[float] = []
    for r in rows:
        if r.ekf_init != 1:
            continue
        if math.isnan(r.ekf_yaw_rad) or math.isnan(r.recorded_vyaw_rad):
            continue
        d = _wrap_pi(r.ekf_yaw_rad - r.recorded_vyaw_rad)
        diffs.append(d * d)
    if not diffs:
        return None
    return math.sqrt(sum(diffs) / len(diffs))


def drift_per_meter(rows: List[Row]) -> Optional[float]:
    ekf_path = 0.0
    rec_path = 0.0
    prev: Optional[Row] = None
    for r in rows:
        if r.ekf_init != 1:
            prev = None
            continue
        if prev is not None:
            ekf_dx = r.ekf_x - prev.ekf_x
            ekf_dz = r.ekf_z - prev.ekf_z
            rec_dx = r.recorded_vx - prev.recorded_vx
            rec_dz = r.recorded_vz - prev.recorded_vz
            if not (math.isnan(ekf_dx) or math.isnan(ekf_dz)):
                ekf_path += math.hypot(ekf_dx, ekf_dz)
            if not (math.isnan(rec_dx) or math.isnan(rec_dz)):
                rec_path += math.hypot(rec_dx, rec_dz)
        prev = r
    if rec_path < 1e-6:
        return None
    return ekf_path / rec_path


def loop_closure_gap_m(rows: List[Row]) -> Optional[float]:
    init_rows = [r for r in rows if r.ekf_init == 1]
    if len(init_rows) < 2:
        return None
    a, b = init_rows[0], init_rows[-1]
    if any(math.isnan(v) for v in (a.ekf_x, a.ekf_z, b.ekf_x, b.ekf_z)):
        return None
    return math.hypot(b.ekf_x - a.ekf_x, b.ekf_z - a.ekf_z)


def v_shape_yaw_deg(rows: List[Row]) -> Optional[float]:
    yaws = [r.ekf_yaw_rad for r in rows
            if r.ekf_init == 1 and not math.isnan(r.ekf_yaw_rad)]
    if len(yaws) < 2:
        return None
    # Unwrap so multi-radian swings accumulate correctly.
    unwrapped = [yaws[0]]
    for y in yaws[1:]:
        prev = unwrapped[-1]
        d = _wrap_pi(y - prev)
        unwrapped.append(prev + d)
    span = max(unwrapped) - min(unwrapped)
    return math.degrees(span)


def evaluate(csv_path: Path,
             max_heading_rmse_rad: float,
             max_drift_per_meter: float,
             max_loop_gap_m: float,
             v_shape_min_deg: Optional[float]) -> dict:
    rows = load_csv(csv_path)
    if not rows:
        raise ValueError(f"{csv_path}: empty CSV")

    metrics = {
        "rows": len(rows),
        "rows_initialized": sum(1 for r in rows if r.ekf_init == 1),
        "heading_rmse_rad": heading_rmse_rad(rows),
        "drift_per_meter": drift_per_meter(rows),
        "loop_closure_gap_m": loop_closure_gap_m(rows),
        "v_shape_yaw_deg": v_shape_yaw_deg(rows),
    }

    failures: List[str] = []
    h = metrics["heading_rmse_rad"]
    if h is not None and h > max_heading_rmse_rad:
        failures.append(
            f"heading_rmse_rad={h:.4f} exceeds max={max_heading_rmse_rad:.4f}")
    d = metrics["drift_per_meter"]
    if d is not None and d > max_drift_per_meter:
        failures.append(
            f"drift_per_meter={d:.4f} exceeds max={max_drift_per_meter:.4f}")
    g = metrics["loop_closure_gap_m"]
    if g is not None and g > max_loop_gap_m:
        failures.append(
            f"loop_closure_gap_m={g:.4f} exceeds max={max_loop_gap_m:.4f}")
    if v_shape_min_deg is not None:
        v = metrics["v_shape_yaw_deg"]
        if v is None:
            failures.append("v_shape_yaw_deg: not computable from this fixture")
        elif v < v_shape_min_deg:
            failures.append(
                f"v_shape_yaw_deg={v:.2f} below required={v_shape_min_deg:.2f}")

    metrics["failures"] = failures
    metrics["pass"] = len(failures) == 0
    return metrics


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="NavSight replay scorer")
    parser.add_argument("csv", type=Path, help="CSV produced by replay_harness")
    parser.add_argument("--max-heading-rmse-rad", type=float, default=0.35,
                        help="max heading RMSE in radians (~20 deg)")
    parser.add_argument("--max-drift-per-meter", type=float, default=4.0,
                        help="max EKF/recorded path-length ratio (IMU-only is loose by design)")
    parser.add_argument("--max-loop-gap-m", type=float, default=50.0,
                        help="max XZ distance between first and last initialized pose")
    parser.add_argument("--v-shape-min-deg", type=float, default=None,
                        help="if set, require yaw span >= this many degrees (use 170 for 180° turns)")
    parser.add_argument("--json-out", type=Path, default=None,
                        help="optional path to dump full metrics as JSON")
    args = parser.parse_args(argv)

    try:
        result = evaluate(
            args.csv,
            args.max_heading_rmse_rad,
            args.max_drift_per_meter,
            args.max_loop_gap_m,
            args.v_shape_min_deg,
        )
    except (FileNotFoundError, ValueError) as e:
        print(f"replay_scorer: {e}", file=sys.stderr)
        return 2

    print(f"== replay scorer: {args.csv}")
    print(f"   rows={result['rows']} initialized={result['rows_initialized']}")
    for key in ("heading_rmse_rad", "drift_per_meter",
                "loop_closure_gap_m", "v_shape_yaw_deg"):
        v = result[key]
        print(f"   {key}: {'n/a' if v is None else f'{v:.4f}'}")
    if result["failures"]:
        for f in result["failures"]:
            print(f"   FAIL: {f}")

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        # JSON-serializable copy (None preserved).
        with args.json_out.open("w") as fh:
            json.dump(result, fh, indent=2)

    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
