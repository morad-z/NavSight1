"""
Verify v22 acceptance metrics from a real-walk sim.

Reads scripts/da3_benchmark/output/... — wait, wrong file path.
Reads tests/sims/regression/visual/v22_verification_walk.json and prints the
event_summary block side-by-side with the acceptance criteria from
project_pending_verification_walk.md.

Usage:
    python scripts/inspect_v22_walk.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

SIM = Path("tests/sims/regression/visual/v22_verification_walk.json")


def fmt(v):
    if isinstance(v, float):
        return f"{v:.3f}"
    return str(v)


def main() -> int:
    if not SIM.exists():
        print(f"FAIL: {SIM} not found", file=sys.stderr)
        return 1

    with open(SIM, encoding="utf-8") as f:
        d = json.load(f)

    es = d.get("event_summary", {})
    fm = d.get("frames_meta", {})
    pts = d.get("points", [])
    start_ms = d.get("startTime", 0)

    print(f"=== v22 verification walk ===")
    print(f"  sim file: {SIM.name}")
    print(f"  startTime: {start_ms}")
    print(f"  points: {len(pts)}")
    if pts:
        dur_s = (pts[-1]["ts"] - pts[0]["ts"]) / 1000.0
        print(f"  duration: {dur_s:.1f} s")
    print(f"  frames written: {fm.get('written', '?')}  dropped: {fm.get('dropped', '?')}")
    if fm.get("written", 0) + fm.get("dropped", 0) > 0:
        drop_pct = 100.0 * fm.get("dropped", 0) / (fm.get("written", 0) + fm.get("dropped", 0))
        print(f"  drop ratio: {drop_pct:.1f} %")

    print()
    print("--- event_summary (sorted) ---")
    for k in sorted(es.keys()):
        print(f"  {k:50s} = {fmt(es[k])}")

    print()
    print("--- v22 acceptance check (from project_pending_verification_walk.md) ---")
    checks = [
        ("loop_closure_corrections_applied",   ">", 50,   "(v21 baseline ~6; Step 2 target ~180)"),
        ("loop_closure_lower_rank_accepts",    ">", 0,    "(multi-candidate retry should fire)"),
        ("loop_closure_low_score_rejects",     "<", 50,   "(v21 baseline ~184; should drop sharply)"),
        ("cam_fps_mean_milli_hz",              "≈", 30000, "(30 fps locked by Camera2Interop; tol ±1500)"),
        ("cam_fps_stdev_milli_hz",             "<", 2000, "(locked = low jitter)"),
    ]
    for key, op, target, note in checks:
        actual = es.get(key)
        if actual is None:
            verdict = "MISSING"
        elif op == ">":
            verdict = "PASS" if actual > target else "FAIL"
        elif op == "<":
            verdict = "PASS" if actual < target else "FAIL"
        elif op == "≈":
            verdict = "PASS" if abs(actual - target) < 1500 else "FAIL"
        else:
            verdict = "?"
        print(f"  [{verdict:7s}] {key:42s} actual={fmt(actual):>10s}  {op} {target:<7} {note}")

    # Trajectory diagnostics — start, end, max distance from origin
    if pts:
        print()
        print("--- trajectory ---")
        # JSON vy/vz/vx are Y-up after JNI swap
        start_x, start_y, start_z = pts[0]["vx"], pts[0]["vy"], pts[0]["vz"]
        end_x, end_y, end_z = pts[-1]["vx"], pts[-1]["vy"], pts[-1]["vz"]
        # Distance from start = sqrt(sum of squared diffs vs starting point)
        dists_from_start = []
        for p in pts:
            dx, dy, dz = p["vx"]-start_x, p["vy"]-start_y, p["vz"]-start_z
            dists_from_start.append(math.sqrt(dx*dx + dy*dy + dz*dz))
        max_dist = max(dists_from_start)
        end_dist = dists_from_start[-1]
        print(f"  start (vx,vy,vz) = ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")
        print(f"  end   (vx,vy,vz) = ({end_x:.2f}, {end_y:.2f}, {end_z:.2f})")
        print(f"  max distance from start: {max_dist:.2f} m")
        print(f"  end-of-walk distance from start: {end_dist:.2f} m  (close-loop error if 2-loop walk)")
        # Vertical motion check (vy is up after JNI swap)
        vy_min = min(p["vy"] for p in pts)
        vy_max = max(p["vy"] for p in pts)
        print(f"  vy (up) range: [{vy_min:.2f}, {vy_max:.2f}] m  span={vy_max-vy_min:.2f}")

        # Total path length
        path = 0.0
        for i in range(1, len(pts)):
            a, b = pts[i-1], pts[i]
            path += math.sqrt((b["vx"]-a["vx"])**2 + (b["vy"]-a["vy"])**2 + (b["vz"]-a["vz"])**2)
        print(f"  total path length: {path:.1f} m")
        print(f"  expected (from event_summary.total_path_dm): {es.get('total_path_dm', '?')} dm = "
              f"{es.get('total_path_dm', 0) / 10.0:.1f} m")

    return 0


if __name__ == "__main__":
    sys.exit(main())
