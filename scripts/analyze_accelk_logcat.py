#!/usr/bin/env python3
"""Parse an on-device logcat from a diagnostic walk and pin why accel-K does/doesn't
self-calibrate. Reads the ACCEL_K_STATE (per-frame gate state), ACCEL_K_CALIB[df]/[loom]
(calib success/reject), and WALK_BG (is_static) lines emitted by Tracker.cpp.

This is the trustworthy validation source for Fix A (the replay harness is NOT — it
never calls reset(), so secs_since_zupt_ stays at -1.0 and the calib window can't open).

Usage: python scripts/analyze_accelk_logcat.py <walk.log>
"""
import re
import sys

# ACCEL_K_STATE: is_static=0 tsz=1.23 window_open=1 accel_dist=0.45m accel_spd=0.78 vis_rel_df=0.0123 vis_rel_loom=0.0040 K_df=-1.0 K_loom=-1.0
RE_STATE = re.compile(
    r"ACCEL_K_STATE: is_static=(\d) tsz=([\-\d.]+) window_open=(\d) "
    r"accel_dist=([\-\d.]+)m accel_spd=([\-\d.]+) vis_rel_df=([\-\d.]+) "
    r"vis_rel_loom=([\-\d.]+) K_df=([\-\d.]+) K_loom=([\-\d.]+)")
RE_CALIB_OK = re.compile(
    r"ACCEL_K_CALIB\[(df|loom)\]: k_obs=([\-\d.]+) accel_dist=([\-\d.]+)m "
    r"vis_rel=([\-\d.]+) tsz=([\-\d.]+)s.*?-> new_K=([\-\d.]+)")
RE_CALIB_REJECT = re.compile(r"ACCEL_K_CALIB\[(df|loom)\]: REJECT.*?k_obs=([\-\d.]+)")
RE_WALKBG = re.compile(r"WALK_BG: is_static=(\d)")


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_accelk_logcat.py <walk.log>")
        return 1
    states, calib_ok, calib_rej = [], [], []
    walkbg_static = walkbg_total = 0
    with open(sys.argv[1], errors="ignore") as fp:
        for line in fp:
            m = RE_STATE.search(line)
            if m:
                states.append(dict(
                    is_static=int(m.group(1)), tsz=float(m.group(2)),
                    window_open=int(m.group(3)), accel_dist=float(m.group(4)),
                    accel_spd=float(m.group(5)), vis_df=float(m.group(6)),
                    vis_loom=float(m.group(7)), kdf=float(m.group(8)),
                    kloom=float(m.group(9))))
                continue
            m = RE_CALIB_OK.search(line)
            if m:
                calib_ok.append(dict(path=m.group(1), k_obs=float(m.group(2)),
                                     accel_dist=float(m.group(3)),
                                     vis_rel=float(m.group(4)), tsz=float(m.group(5)),
                                     new_k=float(m.group(6))))
                continue
            m = RE_CALIB_REJECT.search(line)
            if m:
                calib_rej.append(dict(path=m.group(1), k_obs=float(m.group(2))))
                continue
            m = RE_WALKBG.search(line)
            if m:
                walkbg_total += 1
                walkbg_static += int(m.group(1))

    if not states:
        print("No ACCEL_K_STATE lines found. Is this the instrumented build? "
              "(grep the log for 'NavSight-Tracker' / 'ACCEL_K')")
        return 1

    n = len(states)
    static_n = sum(s["is_static"] for s in states)
    window_n = sum(s["window_open"] for s in states)
    win_states = [s for s in states if s["window_open"]]
    max_dist_in_win = max((s["accel_dist"] for s in win_states), default=0.0)
    max_dist_any = max(s["accel_dist"] for s in states)
    max_spd = max(s["accel_spd"] for s in states)
    kdf_final = states[-1]["kdf"]
    kloom_final = states[-1]["kloom"]
    kdf_ever = max(s["kdf"] for s in states)
    kloom_ever = max(s["kloom"] for s in states)

    print("=" * 70)
    print(f"ACCEL-K CALIBRATION DIAGNOSIS — {sys.argv[1]}")
    print("=" * 70)
    print(f"frames logged (ACCEL_K_STATE, /15): {n}")
    print(f"is_static (ZUPT) frames: {static_n}/{n} ({100*static_n/n:.0f}%)  "
          f"| WALK_BG is_static {walkbg_static}/{walkbg_total}")
    print(f"calib WINDOW open (tsz in [0.3,2.5]): {window_n}/{n} frames")
    print(f"accel_dist: max in-window {max_dist_in_win:.2f}m  (gate needs >1.0m)  "
          f"| max anytime {max_dist_any:.2f}m")
    print(f"accel_spd (world horiz): max {max_spd:.2f} m/s")
    print(f"K_df: final {kdf_final:.1f}  ever-max {kdf_ever:.1f}   "
          f"K_loom: final {kloom_final:.1f}  ever-max {kloom_ever:.1f}  (-1 = never calibrated)")
    print(f"calib FIRES: {len(calib_ok)} (df={sum(1 for c in calib_ok if c['path']=='df')}, "
          f"loom={sum(1 for c in calib_ok if c['path']=='loom')})  | rejects {len(calib_rej)}")
    for c in calib_ok[:8]:
        print(f"   [{c['path']}] k_obs={c['k_obs']:.1f} accel_dist={c['accel_dist']:.2f}m "
              f"vis_rel={c['vis_rel']:.4f} tsz={c['tsz']:.2f}s -> K={c['new_k']:.1f}")

    print("\n--- VERDICT (binding constraint) ---")
    if len(calib_ok) > 0:
        print(f"✓ accel-K DOES self-calibrate on device ({len(calib_ok)} fires). "
              f"K_df={kdf_ever:.0f} K_loom={kloom_ever:.0f}. The replay's calib=0 was the "
              f"harness artifact (no reset()). Fix A may be unnecessary or narrow — "
              f"check whether K is STABLE and CORRECT-magnitude, not whether it fires.")
    elif static_n == 0:
        print("✗ ZUPT never fired (is_static=0). The calib window only opens after a "
              "ZUPT (or init reset). If the walk had real stops, the ZUPT DETECTOR is "
              "the bug (threshold/mean_flow override). → Fix A = make ZUPT fire at stops.")
    elif window_n == 0:
        print("✗ Window never opened despite ZUPT — secs_since_zupt_ never in [0.3,2.5]. "
              "Likely secs_since_zupt_ not initialized (reset path) or reset every frame. "
              "→ Fix A = window/secs_since_zupt_ lifecycle.")
    elif max_dist_in_win <= 1.0:
        print(f"✗ Window opens but accel_dist maxes at {max_dist_in_win:.2f}m < 1.0m gate. "
              f"The accel reference under-integrates (accel_spd max {max_spd:.2f} m/s vs real "
              f"walking ~1.3) — gravity-leak/bias. → Fix A = accel integration (bias/gravity) "
              f"OR lower kAccelKMinDistM with evidence, OR a different metric reference.")
    else:
        print(f"✗ Window opens AND accel_dist reaches {max_dist_in_win:.2f}m, but calib still "
              f"didn't fire → the binding gate is downstream (vis_rel threshold, "
              f"forward_fraction>0.5 for loom, or verification_ok for df). Inspect vis_rel "
              f"(df max {max(s['vis_df'] for s in states):.4f}, loom max "
              f"{max(s['vis_loom'] for s in states):.4f}) and the per-path REJECT lines.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
