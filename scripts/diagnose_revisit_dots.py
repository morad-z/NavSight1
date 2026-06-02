#!/usr/bin/env python3
"""
Diagnose why orange landmark dots don't reappear on revisit.

Hypothesis: with pure axial motion (walk-back / walk-forward facing wall),
monocular VIO loses depth scale. p_G drifts during the round-trip. Landmark
world positions stored at keyframe time use the *drifted* p_G. On revisit,
LC corrects camera pose by a small delta (<0.2 m), but landmarks remain at
their stale positions. The drift accumulated WITHOUT LC is what mislocates
the dots.

This script parses the sim JSON + logcat and:
  1) Extracts EKF p_G evolution over time (from LC_ABS / EKF state logs).
  2) Identifies the revisit window (when LC ACCEPT fires).
  3) Compares p_G at revisit vs p_G at corresponding earlier time (start).
  4) Estimates landmark projection error from observed drift.

Run: python scripts/diagnose_revisit_dots.py <sim.json> <sim.logcat.txt>
"""
import json
import math
import re
import sys
from collections import defaultdict


def parse_lc_abs_lines(logcat_path):
    """Return list of (timestamp_ms, p_G_zup, target_p_zup, r_p, m2) tuples."""
    pat = re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*LC_ABS:.*"
        r"r_R=\[(.+?)\] r_p=\[(.+?)\] p_G=\[(.+?)\] target_p=\[(.+?)\]"
        r".*m2=([0-9.eE+-]+)"
    )
    out = []
    with open(logcat_path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            m = pat.search(line)
            if not m:
                continue
            ts = m.group(1)
            r_R = [float(x) for x in m.group(2).split()]
            r_p = [float(x) for x in m.group(3).split()]
            p_G = [float(x) for x in m.group(4).split()]
            target_p = [float(x) for x in m.group(5).split()]
            m2 = float(m.group(6))
            out.append({
                "ts": ts, "r_R": r_R, "r_p": r_p,
                "p_G": p_G, "target_p": target_p, "m2": m2,
            })
    return out


def parse_lc_accepts(logcat_path):
    """LOOP_CLOSURE: ACCEPT (geom) now_kf=... match_kf=... inl=..."""
    pat = re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*LOOP_CLOSURE:.*ACCEPT.*"
        r"now_kf=(\d+).*match_kf=(\d+).*inl=(\d+)"
    )
    out = []
    with open(logcat_path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            m = pat.search(line)
            if not m:
                continue
            out.append({
                "ts": m.group(1),
                "now_kf": int(m.group(2)),
                "match_kf": int(m.group(3)),
                "inl": int(m.group(4)),
            })
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    sim_path = sys.argv[1]
    log_path = sys.argv[2]

    with open(sim_path, "r") as fh:
        sim = json.load(fh)
    es = sim.get("event_summary", {})
    pts = sim.get("points", [])

    print("=" * 72)
    print(f"Sim:  {sim_path}")
    print(f"Log:  {log_path}")
    print("=" * 72)
    print()

    # ── 1. LC ACCEPTS + LC_ABS ─────────────────────────────────────────────
    lc_abs = parse_lc_abs_lines(log_path)
    lc_acc = parse_lc_accepts(log_path)
    print(f"Loop closure ACCEPTS in logcat: {len(lc_acc)}")
    for a in lc_acc[-3:]:
        print(f"  [{a['ts']}] now_kf={a['now_kf']} match_kf={a['match_kf']} inl={a['inl']}")
    print()
    print(f"LC_ABS lines in logcat: {len(lc_abs)}")
    if lc_abs:
        first = lc_abs[0]
        last = lc_abs[-1]
        print(f"  First: [{first['ts']}] p_G={first['p_G']} target_p={first['target_p']}")
        print(f"         r_p={first['r_p']}  m2={first['m2']:.3f}")
        print(f"  Last:  [{last['ts']}] p_G={last['p_G']} target_p={last['target_p']}")
        print(f"         r_p={last['r_p']}  m2={last['m2']:.3f}")
    print()

    # ── 2. Trajectory drift over time ──────────────────────────────────────
    # Y-up output trajectory (vx, vy, vz) vs duration
    # Convert to Z-up for comparison with logcat LC_ABS p_G
    if not pts:
        print("No trajectory points.")
        return
    p0_yup = (pts[0]["vx"], pts[0]["vy"], pts[0]["vz"])
    print(f"Y-up trajectory start: ({p0_yup[0]:+.3f}, {p0_yup[1]:+.3f}, {p0_yup[2]:+.3f})")
    p_end_yup = (pts[-1]["vx"], pts[-1]["vy"], pts[-1]["vz"])
    print(f"Y-up trajectory end:   ({p_end_yup[0]:+.3f}, {p_end_yup[1]:+.3f}, {p_end_yup[2]:+.3f})")
    # Convert Y-up to Z-up: x=x, z=y (was Y-up Up), y=z (was Y-up depth)
    p0_zup = (p0_yup[0], p0_yup[2], p0_yup[1])
    p_end_zup = (p_end_yup[0], p_end_yup[2], p_end_yup[1])
    print(f"Z-up trajectory start: ({p0_zup[0]:+.3f}, {p0_zup[1]:+.3f}, {p0_zup[2]:+.3f})")
    print(f"Z-up trajectory end:   ({p_end_zup[0]:+.3f}, {p_end_zup[1]:+.3f}, {p_end_zup[2]:+.3f})")
    print()

    # ── 3. EKF p_G (Z-up) vs recorded trajectory at LC moment ──────────────
    if lc_abs:
        last_lc = lc_abs[-1]
        ekf_p_G = last_lc["p_G"]  # Z-up
        # The recorded JSON trajectory is global_t_, in Y-up
        # global_t_ vs p_G divergence:
        # Y-up: vx, vy(up), vz(north)  ↔  Z-up: x, y(north), z(up)
        global_t_zup = p_end_zup
        delta = [ekf_p_G[i] - global_t_zup[i] for i in range(3)]
        delta_mag = math.sqrt(sum(d * d for d in delta))
        print("── Critical divergence: EKF.p_G vs Tracker.global_t_ ───────────────")
        print(f"EKF.p_G  (Z-up): ({ekf_p_G[0]:+.3f}, {ekf_p_G[1]:+.3f}, {ekf_p_G[2]:+.3f})")
        print(f"global_t (Z-up): ({global_t_zup[0]:+.3f}, {global_t_zup[1]:+.3f}, {global_t_zup[2]:+.3f})")
        print(f"DELTA:           ({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})  |Δ|={delta_mag:.3f} m")
        print()

    # ── 4. Drift estimate for landmark projection error ────────────────────
    # If p_G drifted by Δ during the walk, landmarks at depth ~1 m stored
    # via drifted p_G project at wrong pixel by:
    #   px_err ≈ fx * (Δ_lateral / depth)
    # Typical fx for S21 Ultra rear cam ≈ 700-1000.
    if lc_abs:
        fx_typ = 700.0
        depth_typ = 1.0
        delta_lat = math.sqrt(delta[0] ** 2 + delta[1] ** 2)  # horizontal
        px_err_typ = fx_typ * delta_lat / depth_typ
        print("── Estimated landmark projection error at typical 1 m depth ─────────")
        print(f"  fx≈{fx_typ:.0f}, depth_typ=1m, lateral drift={delta_lat:.3f}m")
        print(f"  Estimated screen px error: {px_err_typ:.0f} px")
        if px_err_typ > 100:
            print("  → LARGE: dots would render >100px from their actual feature.")
            print("    User experience: dots appear to NOT reappear on revisit (they're")
            print("    actually drawn but at random screen positions).")
        elif px_err_typ > 30:
            print("  → MODERATE: dots offset by ~30-100px, visible as 'drift'.")
        else:
            print("  → SMALL: dots should be near their features. Bug is elsewhere.")
    print()

    # ── 5. Landmark / matching counters ────────────────────────────────────
    print("── Landmark health ────────────────────────────────────────────────")
    print(f"  landmark_map_size                 = {es.get('landmark_map_size')}")
    print(f"  landmarks_added_total             = {es.get('landmarks_added_total')}")
    print(f"  landmarks_merged_total            = {es.get('landmarks_merged_total')}  (merges = revisit dedup)")
    print(f"  landmarks_matched_total           = {es.get('landmarks_matched_total')}")
    print(f"  landmarks_rendered_world_fixed_total = {es.get('landmarks_rendered_world_fixed_total')}")
    print(f"  landmarks_rendered_anchor_total      = {es.get('landmarks_rendered_anchor_total')}")
    print(f"  ratio of merges/adds              = {es.get('landmarks_merged_total', 0) / max(1, es.get('landmarks_added_total', 1)):.2%}")
    print()

    # ── 6. Verdict ─────────────────────────────────────────────────────────
    print("=" * 72)
    print("VERDICT")
    print("=" * 72)
    if lc_abs and delta_mag > 1.0:
        print(f"✓ HYPOTHESIS CONFIRMED: EKF.p_G and Tracker.global_t_ diverged by")
        print(f"  {delta_mag:.2f} m. The user-facing trajectory uses global_t_, but")
        print(f"  landmark rendering uses EKF.p_G. With ~{px_err_typ:.0f}px projection")
        print(f"  error at typical depth, dots render far from their visible features.")
        print()
        print("  Root cause: with pure axial motion (walk-back/forward facing wall),")
        print("  monocular VIO has no parallax for depth — Tracker.global_t_ tracks")
        print("  via visual displacement (which fails on axial motion) while EKF.p_G")
        print("  is driven mostly by IMU integration (which lacks a constant scale).")
        print()
        print("  FIX: pose-graph back-propagation on LC ACCEPT.")
        print("       - Apply LC correction to LandmarkMap entries (rigid SE(3) shift)")
        print("       - Then also unify the user-facing pose with EKF.p_G")
        print("         (drop the Tracker.global_t_ parallel integrator)")
    elif lc_abs and delta_mag > 0.3:
        print(f"⚠ MODERATE divergence: {delta_mag:.2f} m between EKF and global_t_.")
        print("  Some projection error but not catastrophic. Suggests partial fix")
        print("  by unifying poses; pose-graph less urgent.")
    else:
        print("? Inconclusive. Either no LC fired or divergence is small.")
        print("  Investigate another failure mode (Kotlin render filter, etc.)")


if __name__ == "__main__":
    main()
