"""
compare_walks.py — Multi-walk regression diagnostic.

Reads a list of NavSight simulation_data_<ts>.json files and reports, for each:
  * Walk duration and reported total path length (PDR step-derived)
  * GPS-derived path length (Haversine summed)
  * Final |p_G| (VIO position from origin) vs final GPS distance from start
  * Heading bias: median (VIO_heading - GPS_bearing) over moving segments
  * Speed accuracy: VIO velocity magnitude vs GPS Doppler / Haversine derivative
  * Turn detection: count of |delta_heading| > 45 degree events in VIO vs GPS

This is the diagnostic Morad asked for ("compare to gps see how shit it is").
Designed to be re-runnable on any new walk — pass JSON paths as args.

Usage:
    python scripts/compare_walks.py walk1.json walk2.json ...
"""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path


EARTH_R_M = 6_371_000.0


def haversine_m(lat1, lon1, lat2, lon2):
    if None in (lat1, lon1, lat2, lon2):
        return 0.0
    a = math.radians(lat2 - lat1)
    b = math.radians(lon2 - lon1)
    s = (math.sin(a / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(b / 2) ** 2)
    return 2 * EARTH_R_M * math.asin(min(1.0, math.sqrt(s)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing from (lat1, lon1) -> (lat2, lon2), degrees clockwise from north."""
    if None in (lat1, lon1, lat2, lon2):
        return None
    y = (math.sin(math.radians(lon2 - lon1))
         * math.cos(math.radians(lat2)))
    x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
         - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.cos(math.radians(lon2 - lon1)))
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def analyse(path: Path):
    with open(path) as fh:
        sim = json.load(fh)
    pts = sim["points"]
    if not pts:
        return None

    t0_ms = pts[0]["ts"]
    duration_s = (pts[-1]["ts"] - t0_ms) / 1000.0

    # VIO trajectory (vx,vy,vz in Y-up, JNI-swapped from internal Z-up).
    # NOTE: `hdg` field is in RADIANS — convert to degrees for comparison
    # against GPS bearings (which are degrees clockwise from North).
    vio_path = 0.0
    vio_speeds = []
    vio_headings_deg = []  # degrees, converted from radians
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        dx = b["vx"] - a["vx"]
        dy = b["vy"] - a["vy"]
        dz = b["vz"] - a["vz"]
        dt = (b["ts"] - a["ts"]) / 1000.0
        if dt <= 0:
            continue
        d = math.sqrt(dx*dx + dy*dy + dz*dz)
        # Bounded — anything > 5 m in one tick (40 ms typical) is a loop-closure jump.
        if d < 5.0:
            vio_path += d
        vio_speeds.append(d / dt)
        # hdg is in radians (Madgwick yaw, nav-CW convention, [-π, π]).
        # Convert to degrees [0, 360) to match GPS bearing convention.
        h_deg = (math.degrees(b.get("hdg", 0.0)) + 360.0) % 360.0
        vio_headings_deg.append(h_deg)

    final_vio = math.sqrt(pts[-1]["vx"]**2 + pts[-1]["vy"]**2 + pts[-1]["vz"]**2)
    max_vio = max(math.sqrt(p["vx"]**2 + p["vy"]**2 + p["vz"]**2) for p in pts)

    # GPS trajectory — Haversine summed when both adjacent fixes are valid.
    gps_path = 0.0
    gps_speeds = []
    gps_bearings = []
    gps_first = None
    gps_last = None
    have_gps_pts = 0
    for i in range(1, len(pts)):
        a, b = pts[i - 1], pts[i]
        la1, lo1 = a.get("glat"), a.get("glng")
        la2, lo2 = b.get("glat"), b.get("glng")
        if None in (la1, lo1, la2, lo2):
            continue
        if la1 == 0.0 and lo1 == 0.0:
            continue
        if la2 == 0.0 and lo2 == 0.0:
            continue
        have_gps_pts += 1
        if gps_first is None:
            gps_first = (la1, lo1)
        gps_last = (la2, lo2)
        d = haversine_m(la1, lo1, la2, lo2)
        # Bounded — > 20 m in one tick is GPS jump (jamming).
        if d < 20.0:
            gps_path += d
        dt = (b["ts"] - a["ts"]) / 1000.0
        if dt > 0:
            gps_speeds.append(d / dt)
        br = bearing_deg(la1, lo1, la2, lo2)
        if br is not None and d > 0.3:  # only when moving — bearings are noise at standstill
            gps_bearings.append((br, i))

    if gps_first and gps_last:
        gps_displacement = haversine_m(*gps_first, *gps_last)
    else:
        gps_displacement = 0.0

    # Heading bias: VIO heading at moments where we also have a valid GPS bearing.
    # Both now in degrees, same nav-CW convention.
    heading_diffs = []
    for br, idx in gps_bearings:
        if idx - 1 < len(vio_headings_deg):
            vh = vio_headings_deg[idx - 1]
            heading_diffs.append(wrap180(vh - br))
    heading_bias = (sum(heading_diffs) / len(heading_diffs)) if heading_diffs else None
    heading_diffs_sorted = sorted(heading_diffs) if heading_diffs else []
    heading_p50 = heading_diffs_sorted[len(heading_diffs_sorted)//2] if heading_diffs_sorted else None

    # Speed comparison: mean of moving points only.
    def mean_moving(speeds, floor):
        moving = [s for s in speeds if s > floor]
        return (sum(moving) / len(moving)) if moving else 0.0
    vio_speed_mean = mean_moving(vio_speeds, 0.2)
    gps_speed_mean = mean_moving(gps_speeds, 0.3)

    # Turn detection: count |delta_heading| > 45 in 1.5-second window.
    def count_turns(headings, window_n):
        n = 0
        for i in range(window_n, len(headings)):
            if abs(wrap180(headings[i] - headings[i - window_n])) > 45.0:
                n += 1
        return n
    vio_turns = count_turns(vio_headings_deg, 30)  # ~1.5 s at ~20 Hz
    # GPS turns — derive heading from successive bearings
    gps_heading_track = [br for br, _ in gps_bearings]
    gps_turns = count_turns(gps_heading_track, 5)

    es = sim.get("event_summary", {}) or {}

    return {
        "file": path.name,
        "duration_s": duration_s,
        "vio_path_m": vio_path,
        "gps_path_m": gps_path,
        "vio_final_m": final_vio,
        "vio_max_m": max_vio,
        "gps_displacement_m": gps_displacement,
        "have_gps": have_gps_pts,
        "vio_speed_mean": vio_speed_mean,
        "gps_speed_mean": gps_speed_mean,
        "heading_bias_mean": heading_bias,
        "heading_bias_p50": heading_p50,
        "vio_turn_events": vio_turns,
        "gps_turn_events": gps_turns,
        "lc_attempts": es.get("loop_closure_attempts", 0),
        "lc_accepts": es.get("loop_closure_accepts", 0),
        "lc_corrections_applied": es.get("loop_closure_corrections_applied", 0),
        "slam_promotions": es.get("slam_promotions_total", 0),
        "lm_accepted": es.get("landmarks_msckf_accepted_total", 0),
        "lm_observed": es.get("landmarks_observed_total", 0),
    }


def main(argv):
    rows = []
    for arg in argv:
        p = Path(arg)
        if not p.exists():
            print(f"skip: {p} (missing)", file=sys.stderr)
            continue
        r = analyse(p)
        if r:
            rows.append(r)
    if not rows:
        return 1

    # Compact column-aligned table.
    cols = [
        ("file",                 28, "{:<28}"),
        ("duration_s",           7,  "{:7.1f}"),
        ("vio_path_m",           8,  "{:8.1f}"),
        ("gps_path_m",           8,  "{:8.1f}"),
        ("vio_final_m",          8,  "{:8.1f}"),
        ("vio_max_m",            8,  "{:8.1f}"),
        ("gps_displ_m",          8,  "{:8.1f}"),
        ("vio_spd",              7,  "{:7.2f}"),
        ("gps_spd",              7,  "{:7.2f}"),
        ("hdg_bias",             8,  "{:>8}"),
        ("vio_turns",            5,  "{:5d}"),
        ("gps_turns",            5,  "{:5d}"),
        ("lc_corr",              7,  "{:7d}"),
        ("slam_p",               6,  "{:6d}"),
        ("lm_acc",               7,  "{:7d}"),
    ]
    # header
    print("  ".join(f"{name:<{w}}" if name == "file" else f"{name:>{w}}"
                    for name, w, _ in cols))
    for r in rows:
        vals = []
        vals.append(cols[0][2].format(r["file"][:28]))
        vals.append(cols[1][2].format(r["duration_s"]))
        vals.append(cols[2][2].format(r["vio_path_m"]))
        vals.append(cols[3][2].format(r["gps_path_m"]))
        vals.append(cols[4][2].format(r["vio_final_m"]))
        vals.append(cols[5][2].format(r["vio_max_m"]))
        vals.append(cols[6][2].format(r["gps_displacement_m"]))
        vals.append(cols[7][2].format(r["vio_speed_mean"]))
        vals.append(cols[8][2].format(r["gps_speed_mean"]))
        hb = r["heading_bias_p50"]
        vals.append(cols[9][2].format(f"{hb:+6.1f}°" if hb is not None else "n/a"))
        vals.append(cols[10][2].format(r["vio_turn_events"]))
        vals.append(cols[11][2].format(r["gps_turn_events"]))
        vals.append(cols[12][2].format(r["lc_corrections_applied"]))
        vals.append(cols[13][2].format(r["slam_promotions"]))
        vals.append(cols[14][2].format(r["lm_accepted"]))
        print("  ".join(vals))

    print()
    print("Legend:")
    print("  vio_path_m / gps_path_m   total walked distance — VIO drifts large; GPS is ground truth")
    print("  vio_final_m / gps_displ_m end-to-end displacement — VIO should match GPS if no drift")
    print("  vio_max_m                 worst |p_G| during walk — large = divergence event")
    print("  vio_spd / gps_spd         mean speed of MOVING segments only (>0.2 m/s VIO, >0.3 GPS)")
    print("  hdg_bias                  median (VIO heading - GPS bearing) on moving segments")
    print("  vio_turns / gps_turns     count of >45° heading swings")
    print("  lc_corr                   loop_closure_corrections_applied (0 = no LC ever helped)")
    print("  slam_p / lm_acc           slam_promotions / landmarks_msckf_accepted (Phase 6)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
