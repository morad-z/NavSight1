#!/usr/bin/env python3
"""Parse logcat LC_ABS lines and characterise the chi² rejection pattern.

Produces:
  * Distribution of m²_R vs m²_p (which block dominates rejection?)
  * Distribution of |r_R| (rotation residual) vs |r_p| (position residual)
  * Trajectory of p_G over time
  * Trajectory of |target_p - p_G| over time

Usage:
    python scripts/analyze_chi2_rejections.py <logcat.txt>
"""
from __future__ import annotations

import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

LC_ABS_RE = re.compile(
    r"(?P<ts>\d\d-\d\d \d\d:\d\d:\d\d\.\d\d\d).*?LC_ABS:\s*"
    r"r_R=\[(?P<rR>[^\]]+)\]\s+"
    r"r_p=\[(?P<rp>[^\]]+)\]\s+"
    r"p_G=\[(?P<pG>[^\]]+)\]\s+"
    r"target_p=\[(?P<tp>[^\]]+)\]\s+"
    r"var_R=(?P<vR>[\d.eE+-]+)\s+"
    r"var_p=(?P<vp>[\d.eE+-]+)\s+"
    r"m2=(?P<m2>[\d.eE+-]+)\s+"
    r"m2_R=(?P<m2R>[\d.eE+-]+)\s+"
    r"m2_p=(?P<m2p>[\d.eE+-]+)"
)


def parse_vec3(s: str) -> tuple[float, float, float]:
    parts = s.replace(",", " ").split()
    return float(parts[0]), float(parts[1]), float(parts[2])


@dataclass
class LcAbsRow:
    ts: str
    r_R: tuple[float, float, float]
    r_p: tuple[float, float, float]
    p_G: tuple[float, float, float]
    target_p: tuple[float, float, float]
    var_R: float
    var_p: float
    m2: float
    m2_R: float
    m2_p: float


def parse(logcat_path: Path) -> list[LcAbsRow]:
    rows: list[LcAbsRow] = []
    with logcat_path.open(encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            m = LC_ABS_RE.search(line)
            if not m:
                continue
            rows.append(
                LcAbsRow(
                    ts=m["ts"],
                    r_R=parse_vec3(m["rR"]),
                    r_p=parse_vec3(m["rp"]),
                    p_G=parse_vec3(m["pG"]),
                    target_p=parse_vec3(m["tp"]),
                    var_R=float(m["vR"]),
                    var_p=float(m["vp"]),
                    m2=float(m["m2"]),
                    m2_R=float(m["m2R"]),
                    m2_p=float(m["m2p"]),
                )
            )
    return rows


def mag(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def quantile(vals: list[float], q: float) -> float:
    if not vals:
        return float("nan")
    s = sorted(vals)
    rank = (len(s) - 1) * q
    lo = int(rank)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (rank - lo)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    path = Path(argv[1])
    rows = parse(path)
    if not rows:
        print(f"No LC_ABS lines found in {path}", file=sys.stderr)
        return 1

    print(f"Parsed {len(rows)} LC_ABS rows from {path.name}")
    print()

    rejected = [r for r in rows if r.m2 > 22.5]
    accepted = [r for r in rows if r.m2 <= 22.5]
    print(f"  rejected (m² > 22.5):  {len(rejected)}")
    print(f"  accepted (m² <= 22.5): {len(accepted)}")
    print()

    print("=== Residual magnitudes (|r_R| in deg, |r_p| in m) ===")
    rR_deg = [math.degrees(mag(r.r_R)) for r in rejected]
    rp_m = [mag(r.r_p) for r in rejected]
    for label, vals, unit in (("|r_R|", rR_deg, "deg"), ("|r_p|", rp_m, "m")):
        print(
            f"  {label:6s}  min={min(vals):8.2f}  p50={quantile(vals, 0.5):8.2f}  "
            f"p95={quantile(vals, 0.95):8.2f}  max={max(vals):8.2f}  ({unit})"
        )
    print()

    print("=== Block dominance (m²_p / m²_R ratio per reject) ===")
    ratios = [
        (r.m2_p / max(r.m2_R, 1e-9))
        for r in rejected
        if r.m2_p > 0 or r.m2_R > 0
    ]
    pos_dom = sum(1 for r in rejected if r.m2_p > r.m2_R)
    rot_dom = sum(1 for r in rejected if r.m2_R > r.m2_p)
    print(f"  position-dominated rejects: {pos_dom}/{len(rejected)} "
          f"({100 * pos_dom / max(len(rejected), 1):.0f}%)")
    print(f"  rotation-dominated rejects: {rot_dom}/{len(rejected)} "
          f"({100 * rot_dom / max(len(rejected), 1):.0f}%)")
    if ratios:
        print(
            f"  m²_p / m²_R ratio  median={quantile(ratios, 0.5):.1f}  "
            f"max={max(ratios):.1f}"
        )
    print()

    print("=== EKF p_G magnitude over time (every 10th rejected sample) ===")
    print("  ts             |p_G|       p_G[0]    p_G[1]    p_G[2]    |target_p|  |r_p|")
    print("  " + "-" * 88)
    for i, r in enumerate(rejected):
        if i % 10 != 0:
            continue
        print(
            f"  {r.ts}  {mag(r.p_G):8.1f}  "
            f"{r.p_G[0]:8.1f}  {r.p_G[1]:8.1f}  {r.p_G[2]:8.1f}  "
            f"{mag(r.target_p):8.1f}    {mag(r.r_p):7.1f}"
        )
    print()

    print("=== Z-axis specifically (gravity-direction divergence) ===")
    pg_z = [r.p_G[2] for r in rejected]
    rp_z = [r.r_p[2] for r in rejected]
    print(
        f"  p_G[Z]    min={min(pg_z):8.1f}  median={quantile(pg_z,0.5):8.1f}  "
        f"max={max(pg_z):8.1f}  (m)"
    )
    print(
        f"  r_p[Z]    min={min(rp_z):8.1f}  median={quantile(rp_z,0.5):8.1f}  "
        f"max={max(rp_z):8.1f}  (m)"
    )
    print()

    print("=== Required σ_p to admit observed residuals ===")
    # m²_p = (|r_p| / σ_p)²·k where k is dim. With k=3 dofs and gate 22.5,
    # accept iff |r_p| ≤ σ_p · sqrt(22.5/3) = σ_p · 2.74. So σ needed = |r_p|/2.74.
    sigmas_needed = [mag(r.r_p) / 2.74 for r in rejected]
    print(f"  σ_p needed  median={quantile(sigmas_needed, 0.5):.1f} m  "
          f"max={max(sigmas_needed):.1f} m")
    print(f"  current σ_p formula = max(2.0, 0.032 × path_m). On a 100 m walk: 3.2 m.")
    print(f"  ratio (needed / current): "
          f"{quantile(sigmas_needed, 0.5) / 3.2:.1f}× median, "
          f"{max(sigmas_needed) / 3.2:.1f}× max")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
