"""Quick inspection of heading + GPS alignment for v22 walk."""
import json
import math

with open("tests/sims/regression/visual/v22_verification_walk.json", encoding="utf-8") as f:
    d = json.load(f)

pts = d["points"]
print(f"total points: {len(pts)}")
print()

# First 5 with heading
print("--- first 5 points (heading + GPS) ---")
for p in pts[:5]:
    hdg = p.get("hdg")
    glat = p.get("glat")
    glng = p.get("glng")
    print(f"  ts={p['ts']}  hdg={hdg}  vx={p['vx']:.2f} vy={p['vy']:.2f} vz={p['vz']:.2f}  glat={glat} glng={glng}")

# Find first frame with valid GPS
print()
print("--- first 5 frames with valid GPS ---")
n = 0
for p in pts:
    if p.get("glat") is not None and p.get("glng") is not None:
        print(f"  ts={p['ts']}  hdg={p.get('hdg')}  vx={p['vx']:.2f} vz={p['vz']:.2f}  "
              f"glat={p['glat']:.7f} glng={p['glng']:.7f}  gacc={p.get('gacc')}")
        n += 1
        if n >= 5:
            break

# Stats on heading
hdgs = [p["hdg"] for p in pts if p.get("hdg") is not None]
if hdgs:
    print()
    print(f"--- heading stats (n={len(hdgs)}) ---")
    print(f"  first: {hdgs[0]:.3f} deg")
    print(f"  last:  {hdgs[-1]:.3f} deg")
    print(f"  min/max: {min(hdgs):.3f} / {max(hdgs):.3f}")

# Compute initial GPS bearing if we have two valid GPS points
gps_pts = [(p["ts"], p["glat"], p["glng"]) for p in pts
           if p.get("glat") is not None and p.get("glng") is not None]
print()
print(f"--- GPS coverage ---")
print(f"  valid GPS frames: {len(gps_pts)} / {len(pts)}")
if len(gps_pts) >= 2:
    lat0, lng0 = gps_pts[0][1], gps_pts[0][2]
    # Find a GPS point ~10 m away (forward bearing only meaningful after some motion)
    for ts, lat, lng in gps_pts[1:]:
        # rough flat-earth distance
        dlat_m = (lat - lat0) * 111320.0
        dlng_m = (lng - lng0) * 111320.0 * math.cos(math.radians(lat0))
        d_m = math.sqrt(dlat_m**2 + dlng_m**2)
        if d_m > 5.0:
            # bearing from (lat0, lng0) to (lat, lng): atan2(E, N), then convert nav-conv (CW from N)
            bearing_rad = math.atan2(dlng_m, dlat_m)
            bearing_deg = (math.degrees(bearing_rad) + 360.0) % 360.0
            print(f"  first GPS displacement > 5 m at ts={ts}: dE={dlng_m:.2f} dN={dlat_m:.2f} "
                  f"d={d_m:.2f} bearing={bearing_deg:.1f} deg")
            break

# VIO initial bearing (first ~5 m of motion)
print()
print("--- VIO initial bearing ---")
for i in range(1, len(pts)):
    p0, p = pts[0], pts[i]
    dx = p["vx"] - p0["vx"]
    dz = p["vz"] - p0["vz"]
    d_m = math.sqrt(dx*dx + dz*dz)
    if d_m > 5.0:
        # In the sim JSON: vx and vz are horizontal (vy is up after Y-up JNI swap).
        # The notebook plots (vx, vz) as (East, North) typically — confirm in code.
        bearing_rad = math.atan2(dx, dz)
        bearing_deg = (math.degrees(bearing_rad) + 360.0) % 360.0
        print(f"  first VIO displacement > 5 m at ts={p['ts']}: vx={dx:.2f} vz={dz:.2f} "
              f"d={d_m:.2f} VIO heading (if vz=North, vx=East): {bearing_deg:.1f} deg")
        # Also compute as (vx, -vz) interpretation
        bearing2_rad = math.atan2(dx, -dz)
        bearing2_deg = (math.degrees(bearing2_rad) + 360.0) % 360.0
        print(f"  VIO heading (alt interpretation vz=South, vx=East): {bearing2_deg:.1f} deg")
        break
