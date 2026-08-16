#!/usr/bin/env python3
"""Generate NavSight presentation diagrams as branded SVG (fully local, no network).

Produces 6 professional, color-coded diagrams in the NavSight "technical / precision"
palette (blue = UI / Roey, cyan = VIO engine / Morad, violet = map-matching / Tamir).

Output: ./svg/*.svg   (PNG conversion is done separately via rsvg-convert)
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SVG = os.path.join(HERE, "svg")
os.makedirs(SVG, exist_ok=True)

F = "'Segoe UI', 'Helvetica Neue', Arial, sans-serif"
MONO = "'Consolas', 'DejaVu Sans Mono', monospace"

# palette: (fill, stroke, text)
C = {
    "ui":     ("#DBEAFE", "#2563EB", "#0F172A"),
    "bridge": ("#E2E8F0", "#475569", "#0F172A"),
    "engine": ("#CFFAFE", "#0E7490", "#0F172A"),
    "map":    ("#EDE9FE", "#7C3AED", "#0F172A"),
    "ball":   ("#FEF3C7", "#D97706", "#0F172A"),
    "good":   ("#DCFCE7", "#16A34A", "#14532D"),
    "dec":    ("#FEF9C3", "#CA8A04", "#0F172A"),
    "accent": ("#0891B2", "#0E7490", "#FFFFFF"),
    "accentv":("#7C3AED", "#5B21B6", "#FFFFFF"),
    "io":     ("#DBEAFE", "#2563EB", "#0F172A"),
    "step":   ("#F1F5F9", "#334155", "#0F172A"),
    "ghost":  ("#F8FAFC", "#94A3B8", "#64748B"),
}
ARROW = "#475569"


# Diagrams 04/05/06 are reused as book figures 7.3/7.2/7.1. Print at ~176 mm wide
# needs larger relative type than a projector slide does (supervisor, 08/2026).
BOOK_FONT_SCALE = float(__import__("os").environ.get("DIAGRAM_FONT_SCALE", "1"))

def _fs(v):
    v = float(v)
    if v >= 28:            # headline text already fits; scaling it overruns the canvas
        return round(v, 1)
    return round(v * BOOK_FONT_SCALE, 1)


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def header(w, h, title, subtitle=""):
    s = []
    s.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
             f'viewBox="0 0 {w} {h}" font-family="{F}">')
    s.append('<defs>')
    for name, col in [("slate", ARROW), ("red", "#DC2626"), ("cyan", "#0E7490"),
                      ("green", "#16A34A"), ("violet", "#7C3AED"), ("gray", "#94A3B8"),
                      ("ball", "#D97706")]:
        s.append(f'<marker id="ah-{name}" markerWidth="12" markerHeight="12" refX="9" '
                 f'refY="4" orient="auto" markerUnits="userSpaceOnUse">'
                 f'<path d="M0,0 L9,4 L0,8 Z" fill="{col}"/></marker>')
    s.append(f'<filter id="ds" x="-10%" y="-10%" width="120%" height="130%">'
             f'<feDropShadow dx="0" dy="2" stdDeviation="3" flood-color="#0F172A" flood-opacity="0.18"/></filter>')
    s.append('</defs>')
    s.append(f'<rect x="0" y="0" width="{w}" height="{h}" rx="18" fill="#F8FAFC" stroke="#E2E8F0" stroke-width="2"/>')
    s.append(f'<rect x="0" y="0" width="10" height="{h}" rx="0" fill="#0891B2"/>')
    s.append(f'<text x="42" y="56" font-size="{_fs(30)}" font-weight="700" fill="#0F172A">{esc(title)}</text>')
    if subtitle:
        s.append(f'<text x="42" y="86" font-size="{_fs(18)}" fill="#475569">{esc(subtitle)}</text>')
    return s


def footer(s):
    s.append('</svg>')
    return "\n".join(s)


def box(s, x, y, w, h, lines, cls, fs=16, rx=12, shadow=True, bold0=True):
    fill, stroke, txt = C[cls]
    sh = ' filter="url(#ds)"' if shadow else ''
    s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
             f'stroke="{stroke}" stroke-width="2.5"{sh}/>')
    _text(s, x + w / 2, y + h / 2, lines, txt, fs, bold0)


def diamond(s, cx, cy, w, h, lines, cls, fs=15):
    fill, stroke, txt = C[cls]
    pts = f"{cx},{cy-h/2} {cx+w/2},{cy} {cx},{cy+h/2} {cx-w/2},{cy}"
    s.append(f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="2.5" filter="url(#ds)"/>')
    _text(s, cx, cy, lines, txt, fs, True)


def _text(s, cx, cy, lines, txt, fs, bold0):
    lh = fs * 1.32
    start = cy - (len(lines) - 1) * lh / 2
    for i, ln in enumerate(lines):
        fam = MONO if (isinstance(ln, tuple) and ln[1] == "mono") else F
        raw = ln[0] if isinstance(ln, tuple) else ln
        weight = "700" if (i == 0 and bold0) else "400"
        size = fs + 2 if (i == 0 and bold0 and len(lines) > 1) else fs
        s.append(f'<text x="{cx:.1f}" y="{start + i*lh:.1f}" font-size="{_fs(size)}" '
                 f'font-family="{fam}" font-weight="{weight}" fill="{txt}" '
                 f'text-anchor="middle" dominant-baseline="middle">{esc(raw)}</text>')


def panel(s, x, y, w, h, title, cls):
    fill, stroke, _ = {
        "ui": ("#EFF6FF", "#2563EB", ""), "engine": ("#ECFEFF", "#0E7490", ""),
        "map": ("#F5F3FF", "#7C3AED", ""), "step": ("#F8FAFC", "#94A3B8", ""),
    }[cls]
    s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="16" fill="{fill}" '
             f'stroke="{stroke}" stroke-width="2.5" stroke-dasharray="2 0"/>')
    s.append(f'<text x="{x+20}" y="{y+30}" font-size="{_fs(18)}" font-weight="700" fill="{stroke}">{esc(title)}</text>')


def arrow(s, x1, y1, x2, y2, color="slate", label=None, dashed=False, fs=14, lw=2.5):
    colmap = {"slate": ARROW, "red": "#DC2626", "cyan": "#0E7490", "green": "#16A34A",
              "violet": "#7C3AED", "gray": "#94A3B8", "ball": "#D97706"}
    dash = ' stroke-dasharray="8 6"' if dashed else ''
    s.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{colmap[color]}" '
             f'stroke-width="{lw}"{dash} marker-end="url(#ah-{color})"/>')
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        w = len(label) * fs * 0.56 + 14
        s.append(f'<rect x="{mx-w/2:.1f}" y="{my-fs*0.95:.1f}" width="{w:.1f}" height="{fs*1.9:.1f}" '
                 f'rx="6" fill="#FFFFFF" stroke="{colmap[color]}" stroke-width="1" opacity="0.95"/>')
        s.append(f'<text x="{mx:.1f}" y="{my:.1f}" font-size="{_fs(fs)}" fill="#0F172A" '
                 f'text-anchor="middle" dominant-baseline="middle">{esc(label)}</text>')


def poly(s, points, color="slate", label=None, dashed=False, fs=14):
    colmap = {"slate": ARROW, "red": "#DC2626", "cyan": "#0E7490", "green": "#16A34A",
              "violet": "#7C3AED", "gray": "#94A3B8", "ball": "#D97706"}
    dash = ' stroke-dasharray="8 6"' if dashed else ''
    p = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    s.append(f'<polyline points="{p}" fill="none" stroke="{colmap[color]}" stroke-width="2.5"{dash} '
             f'marker-end="url(#ah-{color})"/>')
    if label:
        # label near the longest vertical/horizontal segment midpoint -> use point index 1..2
        (x1, y1), (x2, y2) = points[1], points[2] if len(points) > 2 else points[1]
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        w = len(label) * fs * 0.56 + 14
        s.append(f'<rect x="{mx-w/2:.1f}" y="{my-fs*0.95:.1f}" width="{w:.1f}" height="{fs*1.9:.1f}" '
                 f'rx="6" fill="#FFFFFF" stroke="{colmap[color]}" stroke-width="1" opacity="0.95"/>')
        s.append(f'<text x="{mx:.1f}" y="{my:.1f}" font-size="{_fs(fs)}" fill="#0F172A" '
                 f'text-anchor="middle" dominant-baseline="middle">{esc(label)}</text>')


def write(name, s):
    with open(os.path.join(SVG, name + ".svg"), "w", encoding="utf-8") as f:
        f.write(footer(s))
    print("svg:", name)


# ----------------------------------------------------------------------------- 01 architecture
def d01():
    W, H = 1540, 1210
    s = header(W, H, "NavSight  -  4-Tier System Architecture", "v1.0-osm  -  GPS-denied, fully offline, on-device")
    panel(s, 60, 110, 1280, 170, "TIER 1  -  PRESENTATION / UI   (Kotlin / Jetpack Compose)   -   Roey", "ui")
    box(s, 95, 165, 380, 90, ["CameraX", "rear camera"], "ui", 16)
    box(s, 540, 165, 400, 90, ["SensorRepository", "accel + gyro ~50 Hz"], "ui", 16)
    box(s, 1000, 165, 305, 90, ["NavSightViewModel", "MapScreen - CameraScreen"], "ui", 15)
    box(s, 320, 320, 960, 80, ["TIER 2  -  JNI BRIDGE", "NativeBridge  -  zero-copy ByteBuffer frames"], "bridge", 16)
    panel(s, 60, 450, 1280, 320, "TIER 3  -  VIO ENGINE   (native C++17 / OpenCV 4.5.3)   -   Morad", "engine")
    box(s, 540, 500, 420, 70, ["VioEngine  -  orchestrator"], "engine", 16)
    box(s, 110, 610, 540, 70, ["Tracker", "KLT flow - ORB reloc - keyframes"], "engine", 15)
    box(s, 760, 610, 520, 70, ["IMUPreintegrator", "Madgwick AHRS + preintegration"], "engine", 15)
    box(s, 380, 700, 760, 56, ["EKF  -  15-DOF error state  +  measurement updaters"], "engine", 15)
    panel(s, 60, 840, 1280, 250, "TIER 4  -  OFFLINE OSM MAP-MATCHING   -   Tamir", "map")
    box(s, 110, 905, 640, 110, ["LocalMatcher  -  Viterbi HMM", "Haifa OSM ~4.67 MB bundled in APK"], "map", 16)
    box(s, 830, 905, 450, 110, ["Graph-rail", "'ball on the road' + live speed"], "ball", 16)
    # arrows
    arrow(s, 285, 255, 470, 320, "slate", "frame")
    arrow(s, 740, 255, 700, 320, "slate", "IMU")
    arrow(s, 800, 400, 750, 500, "slate")
    arrow(s, 750, 570, 380, 610, "slate")     # VIO -> Tracker
    arrow(s, 770, 570, 1010, 610, "slate")    # VIO -> IMU
    arrow(s, 380, 680, 600, 700, "slate")     # Tracker -> EKF
    arrow(s, 1010, 680, 850, 700, "slate")    # IMU -> EKF
    arrow(s, 760, 756, 430, 905, "cyan", "speed + pose")
    arrow(s, 750, 960, 830, 960, "violet")    # MATCH -> BALL
    poly(s, [(1280, 960), (1430, 960), (1430, 210), (1305, 210)], "ball", "matched position + speed")
    write("01-system-architecture", s)


# ----------------------------------------------------------------------------- 02 frame lifecycle
def d02():
    W, H = 2280, 470
    s = header(W, H, "NavSight  -  Single Camera Frame Lifecycle",
               "one frame every ~43 ms (native ~23 fps)  -  median 15.4 ms work")
    steps = [
        ("io", ["1 - Capture", "640x480", "zero-copy ByteBuffer"]),
        ("step", ["2 - Lens-correct", "undistort -> normalized", "(fx ~ 451)"]),
        ("step", ["3 - KLT track", "pyramidal", "Lucas-Kanade"]),
        ("step", ["4 - FB check", "track fwd + back", "reject blur / occlusion"]),
        ("step", ["5 - De-rotate flow", "subtract gyro rotation", "-> translation only"]),
        ("step", ["6 - Feed IPM + EKF", "speed + pose update", "chi^2 22.5 gate"]),
        ("ball", ["Publish VioData", "-> UI", "ball on road + speed"]),
    ]
    x, y, w, h, gap = 42, 150, 270, 150, 38
    cx = []
    for i, (cls, lines) in enumerate(steps):
        bx = x + i * (w + gap)
        box(s, bx, y, w, h, lines, cls, 16)
        cx.append((bx, bx + w))
    for i in range(len(steps) - 1):
        arrow(s, cx[i][1], y + h / 2, cx[i + 1][0], y + h / 2, "slate")
    box(s, 42, 360, W - 84, 70,
        ["Per-frame budget:  15.4 ms median  -  ~78 ms max     vs     200 ms SDD     =>     13x under at the median, 2.5x worst-case margin"],
        "good", 18, shadow=False)
    write("02-frame-lifecycle", s)


# ----------------------------------------------------------------------------- 03 data flow
def d03():
    W, H = 2120, 760
    s = header(W, H, "NavSight  -  End-to-End Data Flow", "sensors -> VIO engine -> offline map ball -> UI   (no GPS in the hot path)")
    panel(s, 50, 150, 360, 470, "SENSORS  (on-device)", "step")
    box(s, 80, 210, 300, 90, ["Rear camera", "640x480 ~23 fps"], "io", 15)
    box(s, 80, 330, 300, 90, ["IMU", "accel + gyro ~50 Hz"], "io", 15)
    box(s, 80, 450, 300, 90, ["GPS", "health-check only", "(not in hot path)"], "ghost", 14)
    panel(s, 500, 200, 380, 360, "NATIVE VIO CORE  -  Morad", "engine")
    box(s, 530, 270, 320, 90, ["Tracker", "KLT + ORB reloc"], "engine", 15)
    box(s, 530, 400, 320, 90, ["EKF  15-DOF", "+ IMU propagation"], "engine", 15)
    panel(s, 960, 200, 360, 360, "ESTIMATES", "step")
    box(s, 990, 270, 300, 90, ["Speed", "IPM + inertial bridge"], "engine", 15)
    box(s, 990, 400, 300, 90, ["Pose / heading"], "engine", 15)
    panel(s, 1400, 200, 400, 360, "MAP MATCHING  -  Tamir", "map")
    box(s, 1430, 270, 340, 90, ["Graph-rail ball", "advanced by speed"], "ball", 15)
    box(s, 1430, 400, 340, 90, ["Viterbi HMM matcher", "(offline OSM)"], "map", 15)
    box(s, 1860, 300, 210, 130, ["Compose UI", "ball on road", "+ speed", "(Roey)"], "ball", 15)
    arrow(s, 380, 255, 530, 305, "slate")        # cam -> tracker
    arrow(s, 380, 375, 530, 435, "slate")        # imu -> ekf
    arrow(s, 690, 360, 690, 400, "slate")        # tracker -> ekf
    arrow(s, 850, 420, 990, 320, "slate")        # ekf -> speed
    arrow(s, 850, 460, 990, 445, "slate")        # ekf -> pose
    arrow(s, 1290, 315, 1430, 315, "slate", "advances the ball", fs=12)   # speed -> ball
    arrow(s, 1290, 445, 1430, 445, "slate", "junction steering", fs=12)   # pose -> matcher
    arrow(s, 1600, 400, 1600, 360, "violet")     # matcher -> ball
    arrow(s, 1770, 315, 1860, 360, "slate")      # ball -> UI
    poly(s, [(230, 540), (230, 615), (1600, 615), (1600, 490)], "gray", "GPS: verify only", dashed=True, fs=13)
    write("03-data-flow", s)


# ----------------------------------------------------------------------------- 04 map matching
def d04():
    W, H = 1240, 1180
    s = header(W, H, "NavSight  -  HMM Map Matching", "Newson & Krumm (2009), Viterbi-decoded over offline OSM")
    box(s, 380, 130, 480, 70, ["Estimated position + heading  (from EKF)"], "io", 16)
    panel(s, 250, 250, 740, 150, "Candidate road segments = HMM states  (within 30 m)", "map")
    box(s, 300, 305, 180, 70, ["segment A"], "map", 15)
    box(s, 530, 305, 180, 70, ["segment B"], "map", 15)
    box(s, 760, 305, 180, 70, ["segment C"], "map", 15)
    box(s, 140, 460, 440, 110, ["Emission", "log p_e = -d_perp^2 / (2 sigma_z^2)", "sigma_z = 20 m"], "step", 15)
    box(s, 660, 460, 460, 110, ["Transition", "log p_t = -|d_gc - d_route|/b + b_way + b_rail", "b = 6  -  b_way = 0.7  -  b_rail = 1.2"], "step", 14)
    box(s, 360, 640, 520, 80, ["Viterbi decode", "argmax  sum ( log p_e + log p_t )"], "accentv", 16)
    box(s, 340, 780, 560, 80, ["Junction steering", "branch by gyro-relative heading offset"], "step", 15)
    box(s, 300, 920, 640, 100, ["Graph-rail ball", "advances only along road-graph edges  (cap 15 m/s)", "(can never teleport across town)"], "ball", 15)
    arrow(s, 620, 200, 620, 250, "slate")
    arrow(s, 480, 400, 360, 460, "slate")    # states -> emission
    arrow(s, 760, 400, 880, 460, "slate")    # states -> transition
    arrow(s, 360, 570, 560, 640, "slate")    # emission -> viterbi
    arrow(s, 880, 570, 700, 640, "slate")    # transition -> viterbi
    arrow(s, 620, 720, 620, 780, "slate")
    arrow(s, 620, 860, 620, 920, "violet")
    write("04-map-matching-hmm", s)


# ----------------------------------------------------------------------------- 05 speed IPM
def d05():
    W, H = 1360, 1340
    s = header(W, H, "NavSight  -  Speed Estimation  (Ground-Plane IPM + Inertial Bridge)",
               "metric speed from optical flow; no learned depth in the speed path")
    box(s, 360, 130, 640, 64, ["Camera ray r_i per road pixel  (640x480, fx ~ 451)"], "io", 15)
    box(s, 410, 224, 540, 56, ["Ground plane at mount height  h = 1.05 m"], "step", 15)
    box(s, 410, 310, 540, 56, [("Depth   Z_i = -h / (n_hat . r_i)", "mono")], "step", 15)
    box(s, 380, 396, 600, 56, [("Flow-per-speed   a_i = (u_fwd - r_i u_fwd,z) / Z_i", "mono")], "step", 14)
    box(s, 360, 482, 640, 56, [("Per-point speed   v_i = -(f_i.a_i)/(a_i.a_i) (1/dt)", "mono")], "step", 14)
    box(s, 40, 452, 310, 108, ["Noise floor", "sigma_v = (sigma_px/fx)/(|a_i| dt)", "sigma_px = 0.5 px"], "step", 11)
    panel(s, 230, 600, 900, 150, "Two-sided vote taxonomy", "step")
    box(s, 270, 655, 380, 80, ["VOTE", "v_i > 3 sigma  AND", "forward-coherent (45 deg cone)"], "step", 14)
    box(s, 710, 655, 380, 80, ["ZERO-WITNESS", "|v_i| < 3 sigma  AND", "floor <= 1 m/s"], "step", 14)
    diamond(s, 680, 850, 220, 120, ["Resolve"], "dec", 17)
    box(s, 120, 990, 360, 80, ["Median speed", "(>= 5 votes)"], "accent", 15)
    box(s, 520, 990, 380, 90, ["EXACT 0 km/h", "standstill zero-lock", "(>= 5 zero-witnesses)"], "good", 15)
    box(s, 950, 990, 320, 90, ["Inertial bridge", "alpha = 0.15", "trusted ~6 s, then decays"], "ball", 14)
    arrow(s, 680, 194, 680, 224, "slate")
    arrow(s, 680, 280, 680, 310, "slate")
    arrow(s, 680, 366, 680, 396, "slate")
    arrow(s, 680, 452, 680, 482, "slate")
    arrow(s, 460, 538, 460, 655, "slate")     # vi -> vote
    arrow(s, 800, 538, 850, 655, "slate")     # vi -> zero
    arrow(s, 195, 560, 270, 670, "slate")     # floor -> vote
    arrow(s, 320, 545, 760, 660, "gray", dashed=True)  # floor -> zero
    arrow(s, 500, 735, 640, 800, "slate")     # vote -> resolve
    arrow(s, 880, 735, 720, 800, "slate")     # zero -> resolve
    arrow(s, 600, 880, 300, 990, "slate", ">=5 votes", fs=12)
    arrow(s, 690, 910, 700, 990, "slate", ">=5 zeros", fs=12)
    arrow(s, 780, 880, 1050, 990, "slate", "else", fs=12)
    write("05-speed-estimation-ipm", s)


# ----------------------------------------------------------------------------- 06 EKF
def d06():
    W, H = 1360, 1120
    s = header(W, H, "NavSight  -  15-DOF Error-State EKF Pipeline",
               "tightly-coupled visual-inertial fusion, ENU Z-up world frame")
    box(s, 230, 130, 900, 100,
        ["Error state  -  15-DOF core + t_d + phi_bc = 19-DOF   (ENU Z-up)",
         "[ attitude - gyro bias - velocity - accel bias - position ]",
         "+ pose clones (<= 15)  -  SLAM slots (<= 12, idle)"], "engine", 14)
    box(s, 330, 280, 700, 80, ["IMU propagation  (Forster midpoint)", "R <- R . dR ;   v, p integrate dv / dp + gravity"], "step", 15)
    panel(s, 180, 420, 1000, 200, "Measurement updates   (Joseph form + per-row Huber)", "engine")
    box(s, 220, 490, 280, 100, ["MSCKF", "visual update"], "step", 15)
    box(s, 540, 490, 280, 100, ["Gravity-alignment", "update", "keeps p_G bounded"], "good", 14)
    box(s, 860, 490, 280, 100, ["ZUPT", "v_G = 0 when", "stationary"], "step", 14)
    diamond(s, 680, 730, 360, 140, ["chi^2 gate = 22.5", "(~ chi2(0.999, 6 dof))"], "dec", 16)
    box(s, 300, 940, 760, 90, ["Bounded, physically consistent state", "speed + pose  ->  map matcher"], "accent", 16)
    arrow(s, 680, 230, 680, 280, "slate")
    arrow(s, 600, 360, 360, 490, "slate")    # prop -> msckf
    arrow(s, 680, 360, 680, 490, "slate")    # prop -> grav
    arrow(s, 760, 360, 1000, 490, "slate")   # prop -> zupt
    arrow(s, 360, 590, 600, 668, "slate")    # msckf -> gate
    arrow(s, 680, 590, 680, 660, "slate")    # grav -> gate
    arrow(s, 1000, 590, 760, 668, "slate")   # zupt -> gate
    arrow(s, 680, 800, 680, 940, "cyan", "accept")
    poly(s, [(860, 730), (1230, 730), (1230, 320), (1030, 320)], "red", "reject outlier", dashed=True)
    poly(s, [(300, 985), (130, 985), (130, 180), (230, 180)], "cyan", "feedback", dashed=True)
    write("06-ekf-pipeline", s)


if __name__ == "__main__":
    d01(); d02(); d03(); d04(); d05(); d06()
    print("done ->", SVG)
