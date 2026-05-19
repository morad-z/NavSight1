"""
Smoke test for Qualcomm's pre-converted Depth-Anything-V3 TFLite.

Goal: confirm the model loads in vanilla TFLite (not just QAIRT) and produces
sensible output. If this passes, we know the TFLite ops are all in the standard
op set and the file can run on Mali-G78 / NNAPI on the S21 Ultra (no Hexagon
required).

Inputs  per metadata.json:
    image: (1, 518, 518, 3) float32, range [0, 1]
Outputs:
    depth_estimates: (1, 518, 518, 1) float32

Usage:
    python scripts/da3_benchmark/smoke_qualcomm_tflite.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

TFLITE_PATH = (
    Path(__file__).resolve().parent
    / "output"
    / "qualcomm_da3_v3"
    / "depth_anything_v3-tflite-float"
    / "depth_anything_v3.tflite"
)


def info(msg: str) -> None:
    print(f"[smoke_qcom] {msg}")


def main() -> int:
    if not TFLITE_PATH.exists():
        info(f"FAIL: tflite not found at {TFLITE_PATH}")
        return 1

    info(f"loading {TFLITE_PATH.name} ({TFLITE_PATH.stat().st_size / 1e6:.1f} MB)")

    try:
        import tensorflow as tf
    except ImportError:
        info("FAIL: tensorflow not installed")
        return 1

    info(f"tensorflow {tf.__version__}")

    interp = tf.lite.Interpreter(model_path=str(TFLITE_PATH))
    interp.allocate_tensors()

    in_details = interp.get_input_details()
    out_details = interp.get_output_details()

    info(f"inputs ({len(in_details)}):")
    for i, d in enumerate(in_details):
        info(f"  [{i}] name={d['name']!r} shape={tuple(d['shape'])} dtype={d['dtype'].__name__}")

    info(f"outputs ({len(out_details)}):")
    for i, d in enumerate(out_details):
        info(f"  [{i}] name={d['name']!r} shape={tuple(d['shape'])} dtype={d['dtype'].__name__}")

    # Build dummy input matching expected shape
    in_shape = tuple(in_details[0]["shape"])
    in_dtype = in_details[0]["dtype"]
    rng = np.random.default_rng(seed=0xDA3)
    dummy = rng.uniform(0.0, 1.0, size=in_shape).astype(in_dtype)
    info(f"dummy input: shape={dummy.shape} dtype={dummy.dtype} range=[{dummy.min():.3f}, {dummy.max():.3f}]")

    # Warm-up + timed runs
    info("warmup ...")
    interp.set_tensor(in_details[0]["index"], dummy)
    interp.invoke()

    n_runs = 5
    times_ms = []
    for i in range(n_runs):
        interp.set_tensor(in_details[0]["index"], dummy)
        t0 = time.perf_counter()
        interp.invoke()
        dt = (time.perf_counter() - t0) * 1000
        times_ms.append(dt)
        info(f"  run {i+1}: {dt:.1f} ms")

    info(f"PC CPU median: {np.median(times_ms):.1f} ms (over {n_runs} runs)")

    out_tensor = interp.get_tensor(out_details[0]["index"])
    info(f"output[0]: shape={out_tensor.shape} "
         f"min={out_tensor.min():.4f} max={out_tensor.max():.4f} "
         f"mean={out_tensor.mean():.4f}")

    # Sanity: output should be non-trivial (not all zeros, not all NaN)
    if not np.isfinite(out_tensor).all():
        info("FAIL: output contains non-finite values (NaN/Inf)")
        return 1
    if out_tensor.std() < 1e-6:
        info("FAIL: output has near-zero variance — model may be broken")
        return 1

    info("smoke test PASSED — Qualcomm TFLite loads and produces valid output on vanilla TFLite runtime.")
    info("Implication: this .tflite should also run on Mali-G78 via TFLite GPU delegate (no Hexagon required).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
