#!/usr/bin/env python3
"""Step 7 — Replay scorer (extended for Step 9 visual coverage).

Consumes the CSV produced by ``tests/cpp/replay_harness`` plus its sidecar
``<csv>.event_counters.json`` and emits regression metrics. CI uses this to
fail when a code change drifts the EKF trajectory or visual front-end stats
away from the recorded baseline by more than configured thresholds.

Inertial metrics (Step 7, ADR-007)
----------------------------------
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

Visual metrics (Step 9, ADR-014)
--------------------------------
* inlier_ratio_mean           mean of (inlier_count / max(tracked_count, 1))
                              across frames where the visual front-end ran
                              (vision_valid == 1 AND frame_loaded == 1).
* mean_flow_p50               median optical-flow magnitude (px) across
                              vision-valid frames.
* loop_closures_detected      EventCounters.loop_closure_corrections_applied,
                              the count of accepted loop-closure injections
                              (Step 7 / ADR-013).
* msckf_huber_rejected_sum    EventCounters value, total observations zeroed
                              by the MSCKF Huber kernel across the run.
* slam_promotions_total       EventCounters value, count of features promoted
                              into the EKF SLAM block (Step 3b / ADR-009).

Visual metrics report ``None`` (n/a) when there are no recorded-frame ticks
in the CSV, so IMU-only fixtures don't trigger spurious failures.

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
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


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
    # Step 9 visual columns. Optional in the CSV header for backward
    # compatibility with old fixtures recorded against the IMU-only harness.
    inlier_count: int = 0
    tracked_count: int = 0
    total_count: int = 0
    mean_flow: float = float("nan")
    pose_flags: int = 0
    vision_valid: int = 0
    frame_loaded: int = 0
    keyframe_stored: int = 0


# Inertial CSV columns required since Step 7 (ADR-007).
_INERTIAL_REQUIRED = {
    "ts_ns", "ekf_init",
    "ekf_x", "ekf_y", "ekf_z", "ekf_yaw_rad",
    "sigma_xx", "sigma_xz", "sigma_zz",
    "recorded_vx", "recorded_vy", "recorded_vz", "recorded_vyaw_rad",
}

# Step 9 visual CSV columns. If any are missing, the scorer treats the CSV
# as legacy (visual metrics return None) instead of erroring out.
# `keyframe_stored` was added later than the other seven; checked separately
# so a fixture recorded between the two harness changes still scores cleanly.
_VISUAL_OPTIONAL = {
    "inlier_count", "tracked_count", "total_count", "mean_flow",
    "pose_flags", "vision_valid", "frame_loaded",
}
_KEYFRAME_OPTIONAL = {"keyframe_stored"}


def _parse_float(value: str) -> float:
    if value == "" or value is None:
        return float("nan")
    try:
        return float(value)
    except ValueError:
        return float("nan")


def _parse_int(value: str) -> int:
    if value == "" or value is None:
        return 0
    try:
        return int(float(value))
    except ValueError:
        return 0


def load_csv(path: Path) -> List[Row]:
    rows: List[Row] = []
    with path.open("r", newline="") as fh:
        reader = csv.DictReader(fh)
        fieldnames = set(reader.fieldnames or [])
        missing = _INERTIAL_REQUIRED - fieldnames
        if missing:
            raise ValueError(f"CSV missing required columns: {sorted(missing)}")
        has_visual = _VISUAL_OPTIONAL.issubset(fieldnames)
        has_keyframe = _KEYFRAME_OPTIONAL.issubset(fieldnames)
        for r in reader:
            row = Row(
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
            )
            if has_visual:
                row.inlier_count = _parse_int(r.get("inlier_count", ""))
                row.tracked_count = _parse_int(r.get("tracked_count", ""))
                row.total_count = _parse_int(r.get("total_count", ""))
                row.mean_flow = _parse_float(r.get("mean_flow", ""))
                row.pose_flags = _parse_int(r.get("pose_flags", ""))
                row.vision_valid = _parse_int(r.get("vision_valid", ""))
                row.frame_loaded = _parse_int(r.get("frame_loaded", ""))
            if has_keyframe:
                row.keyframe_stored = _parse_int(r.get("keyframe_stored", ""))
            rows.append(row)
    return rows


def load_event_counters(csv_path: Path) -> Optional[Dict[str, int]]:
    """Load the sidecar ``<csv>.event_counters.json`` if present.

    Returns ``None`` if the file is missing or malformed — visual metrics
    that rely on event counters then report n/a rather than crashing.
    """
    sidecar = csv_path.with_name(csv_path.name + ".event_counters.json")
    if not sidecar.is_file():
        return None
    try:
        with sidecar.open("r") as fh:
            data = json.load(fh)
    except (json.JSONDecodeError, OSError):
        return None
    if not isinstance(data, dict):
        return None
    # Coerce all values to int — EventCounters serialises as integers but
    # JSON parsers may surface them as float on edge cases.
    return {k: int(v) for k, v in data.items() if isinstance(v, (int, float))}


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


# ── Step 9 visual metrics ────────────────────────────────────────────────


def inlier_ratio_mean(rows: List[Row]) -> Optional[float]:
    """Mean of inlier_count / max(tracked_count, 1) over vision-valid frames.

    Restricted to frames where a real recorded frame was loaded
    (``frame_loaded == 1``) AND the visual front-end consumed it
    (``vision_valid == 1``). Synthetic-grey rows have tracked_count == 0
    and would otherwise drag the mean to undefined values.
    """
    ratios: List[float] = []
    for r in rows:
        if r.frame_loaded != 1 or r.vision_valid != 1:
            continue
        denom = max(r.tracked_count, 1)
        ratios.append(r.inlier_count / denom)
    if not ratios:
        return None
    return sum(ratios) / len(ratios)


def mean_flow_p50(rows: List[Row]) -> Optional[float]:
    flows = [r.mean_flow for r in rows
             if r.frame_loaded == 1 and r.vision_valid == 1
             and not math.isnan(r.mean_flow)]
    if not flows:
        return None
    return float(statistics.median(flows))


def _percentile(values: List[float], pct: float) -> float:
    """95th percentile via linear interpolation between sorted neighbours.

    Avoids the numpy dependency for the CI-only scorer; matches numpy's
    default linear interpolation for small N.
    """
    if not values:
        return float("nan")
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    rank = (len(s) - 1) * pct / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (rank - lo)


def keyframe_match_count_p95(rows: List[Row]) -> Optional[float]:
    """95th percentile of tracked_count over rows where a keyframe was committed.

    `tracked_count` at the moment of keyframe storage is the number of KLT
    survivors carried into the keyframe — the closest equivalent in this
    pipeline to ORB-SLAM's keyframe match count.
    """
    counts = [float(r.tracked_count) for r in rows
              if r.keyframe_stored == 1 and r.frame_loaded == 1]
    if not counts:
        return None
    return _percentile(counts, 95.0)


def loop_closures_detected(events: Optional[Dict[str, int]]) -> Optional[int]:
    if events is None:
        return None
    # `loop_closure_corrections_applied` is the post-χ²-gate count, i.e. how
    # many corrections actually reached EKFState::updateAbsolutePose.
    return events.get("loop_closure_corrections_applied")


def msckf_huber_rejected_sum(events: Optional[Dict[str, int]]) -> Optional[int]:
    if events is None:
        return None
    return events.get("msckf_huber_rejected_sum")


def slam_promotions_total(events: Optional[Dict[str, int]]) -> Optional[int]:
    if events is None:
        return None
    return events.get("slam_promotions_total")


def slam_feature_lifetime_obs_mean(events: Optional[Dict[str, int]]) -> Optional[float]:
    """Mean SLAM-feature lifetime in observations.

    Plan §Step 9 asked for `slam_feature_lifetime_p50`. The instrumentation
    landed as a histogram (sum + count) rather than a per-feature stream,
    so we surface mean instead of median. ADR-014 §"Why we didn't add
    keyframe_match_count_p95 and slam_feature_lifetime_p50 yet" notes the
    deviation; a follow-up that exports per-feature lifetime events can
    upgrade this to true p50.
    """
    if events is None:
        return None
    s = events.get("slam_lifetime_obs_sum")
    n = events.get("slam_lifetime_count")
    if s is None or n is None or n <= 0:
        return None
    return s / n


def evaluate(csv_path: Path,
             max_heading_rmse_rad: float,
             max_drift_per_meter: float,
             max_loop_gap_m: float,
             v_shape_min_deg: Optional[float],
             min_inlier_ratio: Optional[float],
             min_loop_closures: Optional[int]) -> dict:
    rows = load_csv(csv_path)
    if not rows:
        raise ValueError(f"{csv_path}: empty CSV")
    events = load_event_counters(csv_path)

    metrics: Dict[str, object] = {
        "rows": len(rows),
        "rows_initialized": sum(1 for r in rows if r.ekf_init == 1),
        "rows_with_frame": sum(1 for r in rows if r.frame_loaded == 1),
        # Inertial
        "heading_rmse_rad": heading_rmse_rad(rows),
        "drift_per_meter": drift_per_meter(rows),
        "loop_closure_gap_m": loop_closure_gap_m(rows),
        "v_shape_yaw_deg": v_shape_yaw_deg(rows),
        # Visual (Step 9)
        "inlier_ratio_mean": inlier_ratio_mean(rows),
        "mean_flow_p50": mean_flow_p50(rows),
        "keyframe_match_count_p95": keyframe_match_count_p95(rows),
        "loop_closures_detected": loop_closures_detected(events),
        "msckf_huber_rejected_sum": msckf_huber_rejected_sum(events),
        "slam_promotions_total": slam_promotions_total(events),
        "slam_feature_lifetime_obs_mean": slam_feature_lifetime_obs_mean(events),
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
    # Visual gates only fire when the threshold was supplied AND the metric
    # is computable (i.e. recorded-frame fixture). IMU-only fixtures
    # naturally pass with ``None`` for these.
    if min_inlier_ratio is not None:
        ir = metrics["inlier_ratio_mean"]
        if ir is None:
            failures.append(
                "inlier_ratio_mean: required by --min-inlier-ratio but "
                "no vision-valid frames in this fixture")
        elif ir < min_inlier_ratio:
            failures.append(
                f"inlier_ratio_mean={ir:.3f} below required={min_inlier_ratio:.3f}")
    if min_loop_closures is not None:
        lc = metrics["loop_closures_detected"]
        if lc is None:
            failures.append(
                "loop_closures_detected: required by --min-loop-closures but "
                "no event_counters.json sidecar found")
        elif lc < min_loop_closures:
            failures.append(
                f"loop_closures_detected={lc} below required={min_loop_closures}")

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
    # Step 9 visual thresholds. None means "do not gate", which is the
    # default for IMU-only fixtures.
    parser.add_argument("--min-inlier-ratio", type=float, default=None,
                        help="require essential-matrix inlier_ratio_mean >= this "
                             "(0.6 is a reasonable starting point for clean walks)")
    parser.add_argument("--min-loop-closures", type=int, default=None,
                        help="require loop_closures_detected >= this on a "
                             "fixture that loops back on itself")
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
            args.min_inlier_ratio,
            args.min_loop_closures,
        )
    except (FileNotFoundError, ValueError) as e:
        print(f"replay_scorer: {e}", file=sys.stderr)
        return 2

    print(f"== replay scorer: {args.csv}")
    print(f"   rows={result['rows']} initialized={result['rows_initialized']} "
          f"with_frame={result['rows_with_frame']}")
    for key in ("heading_rmse_rad", "drift_per_meter",
                "loop_closure_gap_m", "v_shape_yaw_deg",
                "inlier_ratio_mean", "mean_flow_p50",
                "keyframe_match_count_p95",
                "loop_closures_detected", "msckf_huber_rejected_sum",
                "slam_promotions_total",
                "slam_feature_lifetime_obs_mean"):
        v = result[key]
        if v is None:
            disp = "n/a"
        elif isinstance(v, float):
            disp = f"{v:.4f}"
        else:
            disp = str(v)
        print(f"   {key}: {disp}")
    if result["failures"]:
        for f in result["failures"]:
            print(f"   FAIL: {f}")

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with args.json_out.open("w") as fh:
            json.dump(result, fh, indent=2)

    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
