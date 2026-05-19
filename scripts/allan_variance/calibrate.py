#!/usr/bin/env python3
"""
Allan-variance IMU characterization for NavSight.

Computes per-axis noise density (sigma_white) and random-walk (sigma_b)
for the gyroscope and accelerometer, then prints a snippet you can paste
into `app/src/main/cpp/EKFState.cpp` to replace the ad-hoc constants.

Usage
-----
    # Existing sim JSON (low rate, ~19 Hz; usable but biased)
    python calibrate.py path/to/simulation_data_<ts>.json

    # Dedicated high-rate CSV (recommended)
    # CSV format: ts_ns,gx,gy,gz,ax,ay,az
    python calibrate.py path/to/imu_log.csv

    # Quick test with sparse data:
    python calibrate.py path/to/sim.json --min-tau 0.1 --max-tau 60

Output
------
- stdout: per-axis sigma_white (noise density, units/sqrt(Hz)) and
  sigma_b (bias random walk, units/sqrt(s))
- plot_allan_dev_<channel>.png — log-log Allan deviation curves
- ekf_constants_snippet.txt — ready-to-paste C++ block

Theory (Maybeck 1979 / IEEE 1554-2005)
--------------------------------------
Allan deviation sigma_y(tau) of a sample series has characteristic slopes
in log-log:
    slope -1   : quantization noise
    slope -1/2 : white noise (angle/velocity random walk)  ← gives sigma_white
    slope  0   : flicker / 1/f
    slope +1/2 : bias random walk                          ← gives sigma_b
    slope +1   : rate ramp

For VIO EKF process noise we read:
    sigma_white = ADEV(tau=1s)           ... white noise floor
    sigma_b     = ADEV(tau=3s) * 3       ... random walk slope intercept

Units (after fit):
    gyro white  : rad/s/sqrt(Hz)   →   sigma_g_n in EKFState.cpp
    gyro bias_rw: rad/s^2/sqrt(Hz) →   sigma_bg_n
    accel white : m/s^2/sqrt(Hz)   →   sigma_a_n
    accel bias_rw: m/s^3/sqrt(Hz)  →   sigma_ba_n
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    print("[warn] matplotlib not installed; plots will be skipped")
    HAVE_MPL = False


# ---------------------------------------------------------------------------
# I/O — read sim JSON or CSV into (t_s, gx, gy, gz, ax, ay, az) arrays
# ---------------------------------------------------------------------------

def load_sim_json(path: Path):
    """Read NavSight sim JSON. Returns arrays in SI units (seconds, rad/s, m/s^2)."""
    with open(path) as fh:
        d = json.load(fh)
    pts = d.get("points", [])
    if not pts:
        raise ValueError(f"No 'points' in {path}")
    # Schema: ts (ms), ax/ay/az (m/s^2 body), gx/gy/gz (rad/s body)
    t  = np.array([p["ts"] for p in pts], dtype=np.float64) / 1000.0  # ms -> s
    ax = np.array([p.get("ax", 0.0) for p in pts], dtype=np.float64)
    ay = np.array([p.get("ay", 0.0) for p in pts], dtype=np.float64)
    az = np.array([p.get("az", 0.0) for p in pts], dtype=np.float64)
    gx = np.array([p.get("gx", 0.0) for p in pts], dtype=np.float64)
    gy = np.array([p.get("gy", 0.0) for p in pts], dtype=np.float64)
    gz = np.array([p.get("gz", 0.0) for p in pts], dtype=np.float64)
    return t, gx, gy, gz, ax, ay, az


def load_csv(path: Path):
    """Read raw IMU CSV. Header: ts_ns,gx,gy,gz,ax,ay,az."""
    arr = np.loadtxt(path, delimiter=",", skiprows=1)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(f"CSV must have 7 columns: ts_ns,gx,gy,gz,ax,ay,az; got {arr.shape}")
    t  = arr[:, 0] / 1e9  # ns -> s
    gx, gy, gz = arr[:, 1], arr[:, 2], arr[:, 3]
    ax, ay, az = arr[:, 4], arr[:, 5], arr[:, 6]
    return t, gx, gy, gz, ax, ay, az


def load_any(path: Path):
    if path.suffix.lower() == ".json":
        return load_sim_json(path)
    if path.suffix.lower() == ".csv":
        return load_csv(path)
    raise ValueError(f"Unsupported input format: {path.suffix}")


# ---------------------------------------------------------------------------
# Allan deviation — non-overlapping estimator (sufficient for our data sizes)
# ---------------------------------------------------------------------------

def allan_deviation(x: np.ndarray, rate_hz: float, taus: np.ndarray) -> np.ndarray:
    """Non-overlapping Allan deviation of a sample series.

    For a series x sampled at rate_hz Hz, ADEV at averaging time tau is:
        ADEV(tau) = sqrt( 0.5 * mean[(y[k+1] - y[k])^2] )
    where y is the series of non-overlapping tau-second cluster averages.

    Inputs
    ------
    x        : float array, the raw sensor channel
    rate_hz  : sample rate of x (Hz)
    taus     : array of averaging times to evaluate (seconds)

    Returns
    -------
    adev     : Allan deviation at each tau. NaN if tau is too short (< 1 sample)
               or too long (fewer than 2 clusters fit in x).
    """
    N = len(x)
    out = np.full_like(taus, np.nan, dtype=np.float64)
    for i, tau in enumerate(taus):
        m = int(round(tau * rate_hz))
        if m < 1:
            continue
        n_clusters = N // m
        if n_clusters < 2:
            continue
        # cluster averages over non-overlapping windows of length m
        y = x[: n_clusters * m].reshape(n_clusters, m).mean(axis=1)
        diffs = np.diff(y)
        out[i] = math.sqrt(0.5 * float(np.mean(diffs * diffs)))
    return out


def fit_white_noise_sigma(taus: np.ndarray, adev: np.ndarray) -> float:
    """Read the white-noise sigma at tau=1s by interpolation.

    On a log-log plot, white noise has slope -1/2. ADEV(1s) is the conventional
    'noise density' read-out in IEEE 1554. If the sample rate is too low to
    sample tau=1s with a healthy cluster count, we fall back to the smallest
    valid tau (which will UNDERESTIMATE if real flicker contaminates short tau,
    but is the best we can do).
    """
    mask = np.isfinite(adev)
    if not np.any(mask):
        return float("nan")
    valid_taus = taus[mask]
    valid_adev = adev[mask]
    # Closest valid tau to 1 s
    idx = int(np.argmin(np.abs(np.log10(valid_taus) - 0.0)))
    return float(valid_adev[idx])


def fit_random_walk_sigma(taus: np.ndarray, adev: np.ndarray) -> float:
    """Fit bias random walk sigma_b from the +1/2 slope region (long tau).

    Theory: bias random walk contributes ADEV(tau) = sigma_b * sqrt(tau/3).
    So sigma_b = ADEV(tau) * sqrt(3/tau) on the +1/2-slope part of the curve.

    We pick the largest valid tau and assume it sits in the random-walk regime.
    For a true 2-hour stationary recording this is accurate; for a 2-minute
    sample it's only suggestive.
    """
    mask = np.isfinite(adev)
    if not np.any(mask):
        return float("nan")
    valid_taus = taus[mask]
    valid_adev = adev[mask]
    tau_max = valid_taus[-1]
    adev_max = valid_adev[-1]
    return float(adev_max * math.sqrt(3.0 / tau_max))


# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------

CHANNEL_LABELS = [
    ("gx", "gyro x", "rad/s"),
    ("gy", "gyro y", "rad/s"),
    ("gz", "gyro z", "rad/s"),
    ("ax", "accel x", "m/s^2"),
    ("ay", "accel y", "m/s^2"),
    ("az", "accel z", "m/s^2"),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", type=Path, help="sim .json or raw IMU .csv")
    ap.add_argument("--min-tau", type=float, default=None,
                    help="shortest averaging time (s); default = 2/rate")
    ap.add_argument("--max-tau", type=float, default=None,
                    help="longest averaging time (s); default = duration/8")
    ap.add_argument("--n-taus", type=int, default=40,
                    help="number of tau points (log-spaced)")
    ap.add_argument("--out-dir", type=Path, default=Path("scripts/allan_variance/output"),
                    help="output directory for plots + snippet")
    args = ap.parse_args()

    if not args.input.exists():
        print(f"[error] input not found: {args.input}")
        sys.exit(1)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[load] {args.input}")
    t, gx, gy, gz, ax, ay, az = load_any(args.input)
    N = len(t)
    duration_s = float(t[-1] - t[0])
    rate_hz = (N - 1) / duration_s
    print(f"[load] N={N} samples, duration={duration_s:.1f}s, rate={rate_hz:.1f} Hz")

    if duration_s < 600:
        print(f"[warn] duration {duration_s:.0f}s is short for Allan variance.")
        print( "       For final calibration, record a stationary session ≥ 1 hour.")

    min_tau = args.min_tau if args.min_tau is not None else 2.0 / rate_hz
    max_tau = args.max_tau if args.max_tau is not None else duration_s / 8.0
    if max_tau <= min_tau:
        print(f"[error] max_tau ({max_tau}) <= min_tau ({min_tau})")
        sys.exit(1)
    taus = np.logspace(np.log10(min_tau), np.log10(max_tau), args.n_taus)
    print(f"[allan] taus: {len(taus)} points in [{min_tau:.3g}, {max_tau:.3g}] s")

    channels = {
        "gx": gx, "gy": gy, "gz": gz,
        "ax": ax, "ay": ay, "az": az,
    }
    results = {}
    for key, label, unit in CHANNEL_LABELS:
        x = channels[key]
        # Subtract mean — Allan ignores DC; we want the noise content
        x_zm = x - np.mean(x)
        adev = allan_deviation(x_zm, rate_hz, taus)
        sigma_white = fit_white_noise_sigma(taus, adev)
        sigma_b     = fit_random_walk_sigma(taus, adev)
        results[key] = {
            "label": label,
            "unit": unit,
            "taus": taus,
            "adev": adev,
            "sigma_white": sigma_white,
            "sigma_b": sigma_b,
        }
        print(f"  {label:10s}  sigma_white={sigma_white:.4e} {unit}/sqrt(Hz)   "
              f"sigma_b={sigma_b:.4e} {unit}/sqrt(s)")

    # Per-axis aggregation (use worst-axis as conservative estimate)
    g_white = max(results[k]["sigma_white"] for k in ("gx", "gy", "gz"))
    g_b     = max(results[k]["sigma_b"]     for k in ("gx", "gy", "gz"))
    a_white = max(results[k]["sigma_white"] for k in ("ax", "ay", "az"))
    a_b     = max(results[k]["sigma_b"]     for k in ("ax", "ay", "az"))

    print()
    print("[aggregate] worst-axis values for EKF process noise:")
    print(f"  sigma_g_n  (gyro white)        = {g_white:.4e} rad/s/sqrt(Hz)")
    print(f"  sigma_bg_n (gyro bias RW)      = {g_b:.4e} rad/s^2/sqrt(Hz)")
    print(f"  sigma_a_n  (accel white)       = {a_white:.4e} m/s^2/sqrt(Hz)")
    print(f"  sigma_ba_n (accel bias RW)     = {a_b:.4e} m/s^3/sqrt(Hz)")

    # Plot — one figure per sensor type (gyro, accel) with all 3 axes overlaid
    if HAVE_MPL:
        for sensor, keys, unit in [("gyro", ("gx", "gy", "gz"), "rad/s"),
                                   ("accel", ("ax", "ay", "az"), "m/s^2")]:
            fig, ax2 = plt.subplots(figsize=(8, 6))
            for k in keys:
                r = results[k]
                ax2.loglog(r["taus"], r["adev"], "o-", label=r["label"], markersize=3)
            ax2.set_xlabel("Averaging time τ (s)")
            ax2.set_ylabel(f"Allan deviation σ(τ) ({unit})")
            ax2.set_title(f"NavSight {sensor} — Allan deviation\n"
                          f"input: {args.input.name}, duration={duration_s:.0f}s, rate≈{rate_hz:.0f}Hz")
            ax2.grid(True, which="both", alpha=0.4)
            ax2.legend()
            out_path = args.out_dir / f"allan_dev_{sensor}.png"
            fig.savefig(out_path, dpi=110, bbox_inches="tight")
            plt.close(fig)
            print(f"[plot] {out_path}")

    # Emit a C++ snippet ready for EKFState.cpp
    snippet = f"""// Auto-generated by scripts/allan_variance/calibrate.py
// Input: {args.input.name}
// Duration: {duration_s:.0f}s   Rate: {rate_hz:.1f} Hz   N: {N}
//
// Worst-axis Allan-variance fit. Replace the corresponding ad-hoc constants
// in EKFState.cpp. Use these as Q-matrix process-noise standard deviations.
//
// Gyro white noise        sigma_g_n  = {g_white:.6e} rad/s/sqrt(Hz)
// Gyro bias random walk   sigma_bg_n = {g_b:.6e} rad/s^2/sqrt(Hz)
// Accel white noise       sigma_a_n  = {a_white:.6e} m/s^2/sqrt(Hz)
// Accel bias random walk  sigma_ba_n = {a_b:.6e} m/s^3/sqrt(Hz)

constexpr double SIGMA_G_N  = {g_white:.6e};  // gyro white noise
constexpr double SIGMA_BG_N = {g_b:.6e};      // gyro bias RW
constexpr double SIGMA_A_N  = {a_white:.6e};  // accel white noise
constexpr double SIGMA_BA_N = {a_b:.6e};      // accel bias RW
"""
    snippet_path = args.out_dir / "ekf_constants_snippet.txt"
    snippet_path.write_text(snippet)
    print(f"[snippet] {snippet_path}")

    if duration_s < 3600:
        print()
        print("[caveat] Duration < 1 hour. The bias-random-walk number is "
              "extrapolated from an underdetermined tail. Treat sigma_b as "
              "an upper bound until a 1-2 hour stationary recording lands.")


if __name__ == "__main__":
    main()
