"""
analyze_loop_heading.py — per-loop heading-drift comparison across walks.

Loop boundaries are detected from GPS (return-to-start minima), so VIO position
drift does NOT bias where a loop is judged to end. For each loop we report the
VIO displayed-heading (`hdg`, Madgwick) drift relative to the GPS bearing change
over that loop — i.e. the in-walk heading error growth, with the startup
magnetometer offset removed by referencing loop-start.

Loop 1 contains NO loop closure (revisits happen in loop 2+), so loop-1 drift is
pure dead-reckoning and is the cleanest cross-walk heading comparison.

Usage:
    python scripts/analyze_loop_heading.py walk1.json [walk2.json ...]
"""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path

EARTH_R_M = 6_371_000.0


def haversine_m(lat1, lon1, lat2, lon2):
    a = math.radians(lat2 - lat1)
    b = math.radians(lon2 - lon1)
    s = (math.sin(a / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(b / 2) ** 2)
    return 2 * EARTH_R_M * math.asin(min(1.0, math.sqrt(s)))


def bearing_deg(lat1, lon1, lat2, lon2):
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
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


def vio_heading_deg(p):
    """Displayed heading. `hdg` is the Madgwick heading the UI shows."""
    h = p.get("hdg")
    if h is None:
        return None
    # stored radians or degrees? hdg values seen in [-pi,pi] -> radians.
    return math.degrees(h) if abs(h) <= math.pi + 0.01 else h


def analyse(path: Path):
    d = json.load(open(path))
    pts = d.get("points", [])
    if not pts:
        print(f"{path.name}: no points"); return

    # GPS origin = first valid fix
    g0 = next(((p["glat"], p["glng"]) for p in pts
               if p.get("glat") and p.get("glng")), None)
    if not g0:
        print(f"{path.name}: no GPS"); return

    # distance-from-GPS-start over time
    series = []
    for p in pts:
        if not (p.get("glat") and p.get("glng")):
            continue
        dist = haversine_m(g0[0], g0[1], p["glat"], p["glng"])
        series.append((p["ts"], dist, p))
    if len(series) < 10:
        print(f"{path.name}: too few GPS pts"); return

    # max excursion -> loop returns are local minima of dist after the peak
    max_d = max(s[1] for s in series)
    # find return points: dist drops below 0.30*max_d having been above 0.70*max_d
    returns = []
    above = False
    for ts, dist, p in series:
        if dist > 0.70 * max_d:
            above = True
        if above and dist < 0.30 * max_d:
            returns.append((ts, dist, p))
            above = False
    # loop boundaries = start, each return, end
    bounds = [series[0]] + returns + [series[-1]]

    print(f"\n=== {path.name} ===")
    print(f"  GPS max excursion {max_d:.1f} m, {len(returns)} return(s) detected")
    for i in range(len(bounds) - 1):
        a_ts, _, pa = bounds[i]
        b_ts, _, pb = bounds[i + 1]
        seg = [s for s in series if a_ts <= s[0] <= b_ts]
        if len(seg) < 3:
            continue
        # GPS bearing change = bearing(start->mid) vs bearing(mid->end) net heading
        va = vio_heading_deg(pa); vb = vio_heading_deg(pb)
        vio_change = wrap180(vb - va) if (va is not None and vb is not None) else None
        # GPS net displacement bearing across the loop (≈0 for a closed loop)
        gps_disp = haversine_m(pa["glat"], pa["glng"], pb["glat"], pb["glng"])
        dur = (b_ts - a_ts) / 1e9 if b_ts - a_ts > 1e6 else (b_ts - a_ts)
        # heading-vs-gps-bearing on moving segments inside this loop
        biases = []
        prev = None
        for ts, dist, p in seg:
            if prev is not None:
                step = haversine_m(prev["glat"], prev["glng"], p["glat"], p["glng"])
                if step > 0.3:
                    gb = bearing_deg(prev["glat"], prev["glng"], p["glat"], p["glng"])
                    vh = vio_heading_deg(p)
                    if vh is not None:
                        biases.append(wrap180(vh - gb))
            prev = p
        biases.sort()
        med = biases[len(biases)//2] if biases else None
        spread = (biases[-1]-biases[0]) if len(biases) > 1 else None
        label = f"loop{i+1}" if i < len(returns) else "tail"
        print(f"  {label}: dur={dur:5.1f}s  vio_hdg_net_change={vio_change}"
              f"  gps_close_err={gps_disp:4.1f}m  hdg-gps_bias med={med}"
              f"  spread={spread}")


def main(argv):
    for a in argv[1:]:
        analyse(Path(a))


if __name__ == "__main__":
    main(sys.argv)
