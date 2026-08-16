#!/usr/bin/env python3
"""NavSight full-system architecture - nested "layer" infographic, grid-aligned.

Dashed color-coded nested containers with rounded corner-tab labels, icon boxes with
properly centered text, and clean ORTHOGONAL (right-angle) flow arrows whose labels
float as haloed text in clear gaps. Reads: the Kotlin app (Tier 1) hosts everything;
camera + IMU cross the JNI bridge (Tier 2) into the native C++ VIO engine (Tier 3);
the pose is snapped to a road by the offline map-matcher (Tier 4); the result returns
to the app's live map.  One Tier 1, no GPS node, no legend, no owner names.
Projector-legible type.  Local only -> SVG; PNG via rsvg-convert.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SVG = os.path.join(HERE, "svg")
os.makedirs(SVG, exist_ok=True)
F = "'Segoe UI', 'Helvetica Neue', Arial, sans-serif"

BLUE = "#2563EB"; CYAN = "#0891B2"; CYAN_D = "#0E7490"; VIOLET = "#7C3AED"
GREEN = "#16A34A"; SLATE = "#475569"; AMBER = "#D97706"; INK = "#0F172A"


def esc(t):
    return t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def head(w, h, title):
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" font-family="{F}">']
    s.append('<defs>')
    for n, c in [("slate", SLATE), ("cyan", CYAN_D), ("violet", VIOLET), ("amber", AMBER), ("green", GREEN), ("blue", BLUE)]:
        s.append(f'<marker id="m-{n}" markerWidth="14" markerHeight="14" refX="8.5" refY="4.5" orient="auto" markerUnits="userSpaceOnUse"><path d="M0,0 L10,4.5 L0,9 Z" fill="{c}"/></marker>')
    s.append('<filter id="sh" x="-25%" y="-25%" width="150%" height="170%"><feDropShadow dx="0" dy="3" stdDeviation="4" flood-color="#0F172A" flood-opacity="0.18"/></filter>')
    s.append('</defs>')
    s.append(f'<rect x="0" y="0" width="{w}" height="{h}" rx="20" fill="#FFFFFF" stroke="#E2E8F0" stroke-width="2"/>')
    s.append(f'<text x="50" y="54" font-size="46" font-weight="800" fill="{INK}">{esc(title)}</text>')
    return s


def foot(s):
    s.append('</svg>')
    return "\n".join(s)


def container(s, x, y, w, h, color):
    s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="22" fill="none" stroke="{color}" stroke-width="3.4" stroke-dasharray="13 9"/>')


def tab(s, x, y, label, desc, color, fs=31, dfs=26):
    """Rounded corner-tab pill sitting just above a container's top border (border at y)."""
    pw = len(label) * fs * 0.64 + 32
    pcy = y - 20
    s.append(f'<rect x="{x}" y="{pcy-26:.0f}" width="{pw:.0f}" height="52" rx="26" fill="{color}" filter="url(#sh)"/>')
    s.append(f'<text x="{x+pw/2:.0f}" y="{pcy+9:.0f}" font-size="{fs}" font-weight="800" fill="#FFFFFF" text-anchor="middle">{esc(label)}</text>')
    if desc:
        dx = x + pw + 22
        s.append(f'<text x="{dx:.0f}" y="{pcy+9:.0f}" font-size="{dfs}" font-weight="700" fill="#FFFFFF" stroke="#FFFFFF" stroke-width="6" stroke-linejoin="round">{esc(desc)}</text>')
        s.append(f'<text x="{dx:.0f}" y="{pcy+9:.0f}" font-size="{dfs}" font-weight="700" fill="#1F2937">{esc(desc)}</text>')


def tline(s, x, y, t, fs, col, w="400", anchor="start"):
    s.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{fs}" font-weight="{w}" fill="{col}" text-anchor="{anchor}">{esc(t)}</text>')


def box(s, x, y, w, h, fill, icon, title, subs, tfs=31, sfs=22, icon_pos="top"):
    """Solid icon box. Text block is vertically centered and stays inside the box."""
    s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="15" fill="{fill}" stroke="#FFFFFF" stroke-width="2" filter="url(#sh)"/>')
    cx = x + w / 2
    ls = sfs * 1.35
    if icon_pos == "left":
        icon(s, x + 46, y + h / 2, 50, "#FFFFFF")
        tx = x + 88 + (w - 88) / 2
        block = tfs * 1.15 + len(subs) * ls
        top = y + (h - block) / 2
        tline(s, tx, top + tfs * 0.82, title, tfs, "#FFFFFF", "700", "middle")
        for i, sub in enumerate(subs):
            tline(s, tx, top + tfs * 1.15 + i * ls + sfs * 0.82, sub, sfs, "#EAF2FF", "400", "middle")
    else:
        isz = 44
        block = isz + 14 + tfs * 1.15 + len(subs) * ls
        top = y + (h - block) / 2
        icon(s, cx, top + isz / 2, isz, "#FFFFFF")
        ty0 = top + isz + 14
        tline(s, cx, ty0 + tfs * 0.82, title, tfs, "#FFFFFF", "700", "middle")
        for i, sub in enumerate(subs):
            tline(s, cx, ty0 + tfs * 1.15 + i * ls + sfs * 0.82, sub, sfs, "#EAF2FF", "400", "middle")


def alabel(s, x, y, t, fs=24, col=INK):
    """Floating arrow label: dark text with a white halo (no box)."""
    s.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{fs}" font-weight="700" text-anchor="middle" dominant-baseline="middle" fill="#FFFFFF" stroke="#FFFFFF" stroke-width="6" stroke-linejoin="round">{esc(t)}</text>')
    s.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{fs}" font-weight="700" text-anchor="middle" dominant-baseline="middle" fill="{col}">{esc(t)}</text>')


def arrow(s, x1, y1, x2, y2, col="slate", label=None, dashed=False, fs=24, lw=4.0):
    cm = {"slate": SLATE, "cyan": CYAN_D, "violet": VIOLET, "amber": AMBER, "green": GREEN, "blue": BLUE}
    dash = ' stroke-dasharray="9 6"' if dashed else ''
    s.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{cm[col]}" stroke-width="{lw}"{dash} marker-end="url(#m-{col})"/>')
    if label:
        alabel(s, (x1 + x2) / 2, (y1 + y2) / 2, label, fs)


def poly(s, pts, col="slate", label=None, dashed=False, fs=24, lab_idx=0, lw=4.0):
    cm = {"slate": SLATE, "cyan": CYAN_D, "violet": VIOLET, "amber": AMBER, "green": GREEN, "blue": BLUE}
    dash = ' stroke-dasharray="9 6"' if dashed else ''
    p = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
    s.append(f'<polyline points="{p}" fill="none" stroke="{cm[col]}" stroke-width="{lw}"{dash} stroke-linejoin="round" marker-end="url(#m-{col})"/>')
    if label:
        (x1, y1), (x2, y2) = pts[lab_idx], pts[lab_idx + 1]
        alabel(s, (x1 + x2) / 2, (y1 + y2) / 2, label, fs)


# ---- icons (white stroke on solid boxes) ----
def _g(s, col, sz):
    return f'<g fill="none" stroke="{col}" stroke-width="{max(2.6, sz*0.078):.1f}" stroke-linecap="round" stroke-linejoin="round">'


def ic_camera(s, cx, cy, sz, col):
    w, h = sz * 0.96, sz * 0.62
    s.append(_g(s, col, sz))
    s.append(f'<rect x="{cx-w/2:.1f}" y="{cy-h/2+sz*0.06:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{sz*0.12:.1f}"/>')
    s.append(f'<path d="M{cx-sz*0.18:.1f},{cy-h/2+sz*0.06:.1f} l{sz*0.08:.1f},{-sz*0.13:.1f} l{sz*0.2:.1f},0 l{sz*0.08:.1f},{sz*0.13:.1f}"/>')
    s.append(f'<circle cx="{cx:.1f}" cy="{cy+sz*0.08:.1f}" r="{sz*0.17:.1f}"/>')
    s.append('</g>')


def ic_imu(s, cx, cy, sz, col):
    s.append(_g(s, col, sz))
    s.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{sz*0.34:.1f}"/>')
    s.append(f'<ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{sz*0.34:.1f}" ry="{sz*0.13:.1f}"/>')
    s.append(f'<ellipse cx="{cx:.1f}" cy="{cy:.1f}" rx="{sz*0.13:.1f}" ry="{sz*0.34:.1f}"/>')
    s.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{sz*0.05:.1f}" fill="{col}"/>')
    s.append('</g>')


def ic_bridge(s, cx, cy, sz, col):
    s.append(_g(s, col, sz))
    s.append(f'<path d="M{cx-sz*0.32:.1f},{cy-sz*0.14:.1f} L{cx+sz*0.30:.1f},{cy-sz*0.14:.1f} M{cx+sz*0.12:.1f},{cy-sz*0.30:.1f} L{cx+sz*0.32:.1f},{cy-sz*0.14:.1f} L{cx+sz*0.12:.1f},{cy+sz*0.02:.1f}"/>')
    s.append(f'<path d="M{cx+sz*0.32:.1f},{cy+sz*0.16:.1f} L{cx-sz*0.30:.1f},{cy+sz*0.16:.1f} M{cx-sz*0.12:.1f},{cy:.1f} L{cx-sz*0.32:.1f},{cy+sz*0.16:.1f} L{cx-sz*0.12:.1f},{cy+sz*0.32:.1f}"/>')
    s.append('</g>')


def ic_chip(s, cx, cy, sz, col):
    a = sz * 0.30
    s.append(_g(s, col, sz))
    s.append(f'<rect x="{cx-a:.1f}" y="{cy-a:.1f}" width="{2*a:.1f}" height="{2*a:.1f}" rx="{sz*0.05:.1f}"/>')
    s.append(f'<rect x="{cx-a*0.45:.1f}" y="{cy-a*0.45:.1f}" width="{a*0.9:.1f}" height="{a*0.9:.1f}" rx="{sz*0.03:.1f}"/>')
    for d in (-0.16, 0.16):
        s.append(f'<line x1="{cx+sz*d:.1f}" y1="{cy-a:.1f}" x2="{cx+sz*d:.1f}" y2="{cy-a-sz*0.12:.1f}"/>')
        s.append(f'<line x1="{cx+sz*d:.1f}" y1="{cy+a:.1f}" x2="{cx+sz*d:.1f}" y2="{cy+a+sz*0.12:.1f}"/>')
        s.append(f'<line x1="{cx-a:.1f}" y1="{cy+sz*d:.1f}" x2="{cx-a-sz*0.12:.1f}" y2="{cy+sz*d:.1f}"/>')
        s.append(f'<line x1="{cx+a:.1f}" y1="{cy+sz*d:.1f}" x2="{cx+a+sz*0.12:.1f}" y2="{cy+sz*d:.1f}"/>')
    s.append('</g>')


def ic_target(s, cx, cy, sz, col):
    s.append(_g(s, col, sz))
    s.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{sz*0.32:.1f}"/>')
    s.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{sz*0.14:.1f}"/>')
    s.append(f'<line x1="{cx:.1f}" y1="{cy-sz*0.44:.1f}" x2="{cx:.1f}" y2="{cy-sz*0.24:.1f}"/>')
    s.append(f'<line x1="{cx:.1f}" y1="{cy+sz*0.24:.1f}" x2="{cx:.1f}" y2="{cy+sz*0.44:.1f}"/>')
    s.append(f'<line x1="{cx-sz*0.44:.1f}" y1="{cy:.1f}" x2="{cx-sz*0.24:.1f}" y2="{cy:.1f}"/>')
    s.append(f'<line x1="{cx+sz*0.24:.1f}" y1="{cy:.1f}" x2="{cx+sz*0.44:.1f}" y2="{cy:.1f}"/>')
    s.append('</g>')


def ic_map(s, cx, cy, sz, col):
    w, h = sz * 0.74, sz * 0.56
    s.append(_g(s, col, sz))
    s.append(f'<rect x="{cx-w/2:.1f}" y="{cy-h/2:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{sz*0.05:.1f}"/>')
    s.append(f'<line x1="{cx-w/6:.1f}" y1="{cy-h/2:.1f}" x2="{cx-w/6:.1f}" y2="{cy+h/2:.1f}"/>')
    s.append(f'<line x1="{cx+w/6:.1f}" y1="{cy-h/2:.1f}" x2="{cx+w/6:.1f}" y2="{cy+h/2:.1f}"/>')
    s.append(f'<circle cx="{cx+w*0.12:.1f}" cy="{cy-sz*0.02:.1f}" r="{sz*0.07:.1f}" fill="{col}"/>')
    s.append('</g>')


def ic_db(s, cx, cy, sz, col):
    w, ry = sz * 0.30, sz * 0.11
    top, bot = cy - sz * 0.26, cy + sz * 0.26
    s.append(_g(s, col, sz))
    s.append(f'<ellipse cx="{cx:.1f}" cy="{top:.1f}" rx="{w:.1f}" ry="{ry:.1f}"/>')
    s.append(f'<path d="M{cx-w:.1f},{top:.1f} L{cx-w:.1f},{bot:.1f} A{w:.1f},{ry:.1f} 0 0 0 {cx+w:.1f},{bot:.1f} L{cx+w:.1f},{top:.1f}"/>')
    s.append(f'<path d="M{cx-w:.1f},{cy:.1f} A{w:.1f},{ry:.1f} 0 0 0 {cx+w:.1f},{cy:.1f}"/>')
    s.append('</g>')


def ic_ball(s, cx, cy, sz, col):
    s.append(_g(s, col, sz))
    s.append(f'<line x1="{cx-sz*0.42:.1f}" y1="{cy+sz*0.22:.1f}" x2="{cx+sz*0.42:.1f}" y2="{cy+sz*0.22:.1f}"/>')
    s.append(f'<circle cx="{cx:.1f}" cy="{cy-sz*0.04:.1f}" r="{sz*0.20:.1f}" fill="{col}"/>')
    s.append('</g>')


def ic_phone(s, cx, cy, sz, col):
    w, h = sz * 0.56, sz * 0.92
    s.append(_g(s, col, sz))
    s.append(f'<rect x="{cx-w/2:.1f}" y="{cy-h/2:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{sz*0.1:.1f}"/>')
    s.append(f'<line x1="{cx-sz*0.08:.1f}" y1="{cy-h/2+sz*0.07:.1f}" x2="{cx+sz*0.08:.1f}" y2="{cy-h/2+sz*0.07:.1f}"/>')
    s.append(f'<line x1="{cx-w/2:.1f}" y1="{cy+h/2-sz*0.16:.1f}" x2="{cx+w/2:.1f}" y2="{cy+h/2-sz*0.16:.1f}"/>')
    s.append(f'<circle cx="{cx+sz*0.07:.1f}" cy="{cy-sz*0.06:.1f}" r="{sz*0.09:.1f}" fill="{col}"/>')
    s.append('</g>')


def ic_off(s, cx, cy, sz, col):
    s.append(_g(s, col, sz))
    s.append(f'<path d="M{cx-sz*0.34:.1f},{cy+sz*0.12:.1f} a{sz*0.16:.1f},{sz*0.16:.1f} 0 0 1 {sz*0.05:.1f},{-sz*0.31:.1f} a{sz*0.22:.1f},{sz*0.22:.1f} 0 0 1 {sz*0.42:.1f},{sz*0.05:.1f} a{sz*0.14:.1f},{sz*0.14:.1f} 0 0 1 {-sz*0.04:.1f},{sz*0.27:.1f} Z"/>')
    s.append(f'<line x1="{cx-sz*0.34:.1f}" y1="{cy-sz*0.34:.1f}" x2="{cx+sz*0.34:.1f}" y2="{cy+sz*0.30:.1f}"/>')
    s.append('</g>')


def build():
    # Single left-to-right pipeline: 5 stages on one horizontal axis (y = AX).
    # Wide canvas + large boxes -> large, projector-legible type.
    # Linear pipeline. Tier 1 wraps ONLY the sensors (camera + IMU); Tiers 2-4 are
    # sibling containers; the live map is the standalone output (right).
    W, H = 2500, 740
    AX = 400
    s = head(W, H, "NavSight  -  System Architecture")

    # offline badge top-right
    bw = 470
    s.append(f'<rect x="{W-bw-44}" y="26" width="{bw}" height="58" rx="29" fill="{INK}"/>')
    ic_off(s, W - bw - 44 + 36, 55, 36, "#FFFFFF")
    tline(s, W - bw - 44 + 70, 64, "No GPS / no network in the hot path", 22, "#FFFFFF", "700")

    # ---- Tier 1 : sensors only (camera + IMU) ----
    container(s, 50, 160, 392, 470, BLUE)
    tab(s, 78, 160, "Tier 1", "Sensors  -  Kotlin app", BLUE, fs=30, dfs=24)
    box(s, 74, 235, 344, 175, GREEN, ic_camera, "Rear camera", ["640x480 - 30 fps"], tfs=38, sfs=27)
    box(s, 74, 440, 344, 175, GREEN, ic_imu, "IMU", ["accel + gyro ~50 Hz"], tfs=38, sfs=27)

    # ---- Tier 2 : JNI bridge ----
    container(s, 472, 270, 210, 260, SLATE)
    tab(s, 496, 270, "Tier 2", "", SLATE, fs=30)
    box(s, 494, 305, 166, 190, SLATE, ic_bridge, "JNI Bridge", ["zero-copy"], tfs=30, sfs=22)

    # ---- Tier 3 : VIO engine (VioEngine fans to Tracker + IMU-pre, merges to EKF) ----
    container(s, 740, 160, 680, 470, CYAN_D)
    tab(s, 768, 160, "Tier 3", "VIO Engine  -  C++17 / OpenCV", CYAN_D, fs=30, dfs=25)
    box(s, 768, 300, 190, 200, CYAN, ic_chip, "VioEngine", ["C++17 core"], tfs=33, sfs=23)
    box(s, 980, 220, 232, 170, CYAN, ic_target, "Tracker", ["KLT - ORB"], tfs=34, sfs=24)
    box(s, 980, 410, 232, 170, CYAN, ic_imu, "IMU preint.", ["Madgwick"], tfs=31, sfs=23)
    box(s, 1232, 300, 172, 200, CYAN_D, ic_chip, "EKF", ["15-DOF"], tfs=36, sfs=24)

    # ---- Tier 4 : offline OSM map-matching ----
    container(s, 1470, 160, 660, 470, VIOLET)
    tab(s, 1498, 160, "Tier 4", "Offline OSM Map-Matching  -  Kotlin", VIOLET, fs=30, dfs=25)
    box(s, 1498, 260, 290, 230, VIOLET, ic_map, "LocalMatcher", ["Viterbi HMM"], tfs=34, sfs=24)
    box(s, 1880, 260, 200, 230, AMBER, ic_ball, "Ball on road", ["+ live speed"], tfs=30, sfs=22)
    box(s, 1556, 522, 300, 96, VIOLET, ic_db, "OSM graph", ["bundled in APK"], tfs=29, sfs=21, icon_pos="left")

    # ---- live map : standalone output ----
    box(s, 2180, 260, 280, 270, BLUE, ic_phone, "Live map UI", ["ball + km/h", "on Google Maps"], tfs=34, sfs=24)

    # ===== flow arrows : straight through-line at y=AX, VIO fans out/in =====
    arrow(s, 418, 350, 494, 350, "green", "frame")
    arrow(s, 418, 450, 494, 450, "green", "IMU")
    arrow(s, 660, 400, 768, 400, "slate", "frames")
    arrow(s, 958, 360, 980, 330, "cyan")
    arrow(s, 958, 440, 980, 470, "cyan")
    arrow(s, 1212, 330, 1232, 365, "cyan")
    arrow(s, 1212, 470, 1232, 435, "cyan")
    arrow(s, 1404, 400, 1498, 400, "cyan", "speed + pose")
    arrow(s, 1700, 522, 1660, 490, "violet", "road graph")
    arrow(s, 1788, 400, 1880, 400, "violet", "snap")
    arrow(s, 2080, 400, 2180, 400, "amber")

    with open(os.path.join(SVG, "00-system-architecture-full.svg"), "w", encoding="utf-8") as f:
        f.write(foot(s))
    print("wrote 00-system-architecture-full.svg")


if __name__ == "__main__":
    build()
