#!/usr/bin/env python3
"""
Hunt the source of R_GtoI rotation drift during pure-translational motion.

Background: with phone facing a fixed scene and pure-axial translation
(walk backward / walk forward, no rotation), R_GtoI should be invariant.
The fix9_revisit_2026_05_19 walk shows EKF r_R = 2.72 rad (~156°) at LC
ACCEPT time, meaning R_GtoI drifted by ~156° during the walk. Diagnose
which update path is responsible.

Reads the logcat and traces:
  1) EKF init R_GtoI (gravity-alignment, Madgwick seed).
  2) Per-update R_GtoI deltas from updateRelativeRotation / MSCKF /
     SLAM live updates / gravity-alignment / ZRUP / etc.
  3) Cumulative rotation magnitude.
  4) Identifies the update path contributing the most rotation.

Run: python scripts/hunt_rgtoi_drift.py <logcat.txt>
"""
import re
import sys
from collections import defaultdict


# Patterns to extract events with rotation-relevant data.
PATTERNS = {
    "EKF_INIT": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*(EKF init|EKF_INIT_TILT|EKF initialized)"
    ),
    "MADGWICK_NUDGE": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*LC_MADG_NUDGE.*delta=([+-]?[0-9.]+)°"
    ),
    "LC_ABS": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*LC_ABS:.*r_R=\[(.+?)\].*m2_R=([0-9.eE+-]+).*m2_p=([0-9.eE+-]+)"
    ),
    "LC_ACCEPT": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*LOOP_CLOSURE: ACCEPT.*now_kf=(\d+).*inl=(\d+)"
    ),
    "ZUPT": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*ZUPT.*STATIONARY"
    ),
    "SLAM_PROMOTE": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*SLAM promote.*slot=(\d+).*anchor=(\d+)"
    ),
    "SLAM_DEMOTE": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*SLAM demote.*slot=(\d+)"
    ),
    "SLAM_LIVE": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*SLAM_RMS_LIVE.*rms=([0-9.]+)"
    ),
    "POSE_VALID": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*POSE_VALID_INNER.*inliers=(\d+)/(\d+).*ratio=([0-9.]+).*inner_pass=(\d)"
    ),
    "WALK_BG": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*WALK_BG:.*ekf_bg=\((.+?)\)"
    ),
    "GATES": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*GATES:.*flow=([0-9.]+).*motion=(\d).*parallax=(\d).*static=(\d).*rot=(\d)"
    ),
    "ROT_GATE": re.compile(
        r"(\d{2}:\d{2}:\d{2}\.\d{3}).*ROT_GATE:.*R/N=([0-9.]+).*pure_rot=(\d)"
    ),
}


def parse_time(ts_str):
    """Convert HH:MM:SS.mmm to seconds since midnight."""
    h, m, s = ts_str.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    log_path = sys.argv[1]

    events = []  # (ts_sec, kind, dict)
    with open(log_path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            for kind, pat in PATTERNS.items():
                m = pat.search(line)
                if not m:
                    continue
                ts = parse_time(m.group(1))
                row = {"ts_str": m.group(1), "ts": ts, "kind": kind, "raw": line.rstrip()}
                if kind == "LC_ABS":
                    r_R = [float(x) for x in m.group(2).split()]
                    row.update({
                        "r_R": r_R,
                        "|r_R|": sum(x * x for x in r_R) ** 0.5,
                        "m2_R": float(m.group(3)),
                        "m2_p": float(m.group(4)),
                    })
                elif kind == "GATES":
                    row.update({
                        "flow": float(m.group(2)),
                        "motion": int(m.group(3)),
                        "parallax": int(m.group(4)),
                        "static": int(m.group(5)),
                        "rot": int(m.group(6)),
                    })
                elif kind == "POSE_VALID":
                    row.update({
                        "inliers": int(m.group(2)),
                        "tracked": int(m.group(3)),
                        "ratio": float(m.group(4)),
                        "pass": int(m.group(5)),
                    })
                events.append(row)
                break

    if not events:
        print("No matching events in logcat.")
        return

    events.sort(key=lambda e: e["ts"])
    t0 = events[0]["ts"]

    print("=" * 80)
    print(f"Logcat: {log_path}")
    print(f"Time span: {events[0]['ts_str']} .. {events[-1]['ts_str']}")
    print(f"          ({(events[-1]['ts'] - t0):.1f} sec)")
    print()

    # 1) EKF init events
    init_events = [e for e in events if e["kind"] == "EKF_INIT"]
    print(f"EKF init events: {len(init_events)}")
    for e in init_events[:5]:
        print(f"  [{e['ts_str']}] {e['raw'][30:200]}")
    print()

    # 2) LC_ABS r_R growth
    lc_abs = [e for e in events if e["kind"] == "LC_ABS"]
    print(f"LC_ABS lines: {len(lc_abs)}")
    if lc_abs:
        print(f'{"t":>6} {"|r_R|deg":>9} {"r_R":>30} {"m2_R":>8} {"m2_p":>8}')
        for i in range(0, len(lc_abs), max(1, len(lc_abs) // 15)):
            e = lc_abs[i]
            dt = e["ts"] - t0
            rR_deg = e["|r_R|"] * 180.0 / 3.14159
            rR_str = "[" + " ".join(f"{x:+.2f}" for x in e["r_R"]) + "]"
            print(f"{dt:>6.1f} {rR_deg:>9.1f} {rR_str:>30} {e['m2_R']:>8.2f} {e['m2_p']:>8.2f}")
    print()

    # 3) Madgwick / LC nudges
    nudges = [e for e in events if e["kind"] == "MADGWICK_NUDGE"]
    print(f"LC_MADG_NUDGE events: {len(nudges)}")
    for e in nudges[:10]:
        print(f"  [{e['ts_str']}] {e['raw'][30:200]}")
    print()

    # 4) Motion gates over time
    gates = [e for e in events if e["kind"] == "GATES"]
    if gates:
        print(f"GATES events: {len(gates)}")
        rot_count = sum(1 for e in gates if e["rot"] == 1)
        static_count = sum(1 for e in gates if e["static"] == 1)
        motion_count = sum(1 for e in gates if e["motion"] == 1)
        parallax_count = sum(1 for e in gates if e["parallax"] == 1)
        print(f"  rot=1:      {rot_count} ({rot_count/len(gates)*100:.1f}%)")
        print(f"  static=1:   {static_count} ({static_count/len(gates)*100:.1f}%)")
        print(f"  motion=1:   {motion_count} ({motion_count/len(gates)*100:.1f}%)")
        print(f"  parallax=1: {parallax_count} ({parallax_count/len(gates)*100:.1f}%)")
        print(f"  → if rot=1 but user wasn't rotating, ROT_GATE misfired (pure-rotation detector confused)")
    print()

    # 5) POSE_VALID success rate
    pose_valid = [e for e in events if e["kind"] == "POSE_VALID"]
    if pose_valid:
        passes = sum(1 for e in pose_valid if e["pass"] == 1)
        print(f"POSE_VALID_INNER: {len(pose_valid)} samples, {passes} passes ({passes/len(pose_valid)*100:.1f}%)")
        avg_ratio = sum(e["ratio"] for e in pose_valid) / len(pose_valid)
        print(f"  Average inlier ratio: {avg_ratio:.2f}")
    print()

    # 6) Promotions vs demotions
    promos = sum(1 for e in events if e["kind"] == "SLAM_PROMOTE")
    demos = sum(1 for e in events if e["kind"] == "SLAM_DEMOTE")
    print(f"SLAM lifecycle: {promos} promotions, {demos} demotions")
    print()

    # ── Verdict ────────────────────────────────────────────────────────────
    print("=" * 80)
    print("HYPOTHESIS RANKING (based on logcat evidence)")
    print("=" * 80)
    if lc_abs:
        first_r_R_deg = lc_abs[0]["|r_R|"] * 180.0 / 3.14159
        last_r_R_deg = lc_abs[-1]["|r_R|"] * 180.0 / 3.14159
        print(f"R_GtoI residual at first LC_ABS: {first_r_R_deg:.1f}°")
        print(f"R_GtoI residual at last LC_ABS:  {last_r_R_deg:.1f}°")
        if first_r_R_deg > 90:
            print(f"→ R_GtoI was ALREADY ~{first_r_R_deg:.0f}° off when the first LC fired.")
            print(f"  This means R_GtoI drifted between EKF init and the first revisit.")
            print(f"  Look at MSCKF / SLAM updates during the walk-away phase.")
    if gates:
        rot_pct = sum(1 for e in gates if e["rot"] == 1) / len(gates) * 100
        if rot_pct > 20:
            print(f"→ ROT_GATE flagged rotation in {rot_pct:.1f}% of frames despite")
            print(f"  user reporting pure-translational motion. ROT_GATE is misfiring,")
            print(f"  likely because pure-axial motion gives Rayleigh-style flow that")
            print(f"  the pure-rotation detector confuses with rotation. Worth muting")
            print(f"  visual rotation updates when ROT_GATE=1 AND parallax=0.")
    print()
    print("Suggested next step: per the navsight-vio-specialist Playbook B,")
    print("run scripts/analyze_chi2_rejections.py on this same logcat for the")
    print("formal block-dominance + |r_R| / |r_p| breakdown.")


if __name__ == "__main__":
    main()
