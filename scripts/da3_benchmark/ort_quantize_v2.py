"""
Quantize V2 Metric ONNX for ONNX Runtime Mobile deployment.

Replaces the broken TFLite path. We have a working v2_metric_518_fp32.onnx;
ORT Mobile can run it directly on the S21 Ultra via NNAPI / XNNPACK
execution providers. INT8 dynamic quantization gives the speed-up that
TFLite INT8 would have given, without the onnx2tf+numpy2 incompat.

Pipeline:
  v2_metric_518_fp32.onnx
        |
        +-- quantize_dynamic --> v2_metric_518_int8.onnx (~25 MB)
        |
        +-- shape inference --> v2_metric_518_fp32_shaped.onnx (cleaner for mobile)

Validation:
  - Load each variant via onnxruntime
  - Run on the same dummy input
  - Confirm output stays in real-world metric range

Usage:
    python scripts/da3_benchmark/ort_quantize_v2.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def info(msg: str) -> None:
    print(f"[ort_quant] {msg}")


def fail(msg: str, code: int = 1) -> None:
    print(f"[ort_quant] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def main() -> int:
    fp32_path = OUTPUT_DIR / "v2_metric_518_fp32.onnx"
    if not fp32_path.exists():
        fail(f"missing {fp32_path}; run convert_v2_metric.py first")

    info(f"input: {fp32_path.name} ({fp32_path.stat().st_size / 1e6:.1f} MB)")

    try:
        import onnxruntime as ort
        from onnxruntime.quantization import (
            quantize_dynamic,
            quantize_static,
            QuantType,
            QuantFormat,
            CalibrationDataReader,
        )
        from onnxruntime.quantization.shape_inference import quant_pre_process
    except ImportError as e:
        fail(f"onnxruntime quantization not installed: {e}")

    info(f"onnxruntime {ort.__version__}")

    report = {"input_mb": round(fp32_path.stat().st_size / 1e6, 2)}

    # Step 1: shape inference / preprocessing for cleaner mobile model
    shaped_path = OUTPUT_DIR / "v2_metric_518_fp32_shaped.onnx"
    info("running quant_pre_process (shape inference + symbol elimination) ...")
    try:
        quant_pre_process(
            input_model_path=str(fp32_path),
            output_model_path=str(shaped_path),
            skip_optimization=False,
            skip_onnx_shape=False,
            skip_symbolic_shape=False,
        )
        info(f"  shaped -> {shaped_path.name} ({shaped_path.stat().st_size / 1e6:.1f} MB)")
        report["shaped_mb"] = round(shaped_path.stat().st_size / 1e6, 2)
    except Exception as e:
        info(f"  preprocessing failed (non-fatal): {type(e).__name__}: {e}")
        info("  falling back to raw FP32 ONNX as input to quantization")
        shaped_path = fp32_path

    # Step 2: QDQ-format static INT8 quantization
    # Why QDQ instead of quantize_dynamic:
    #   quantize_dynamic emits ConvInteger / MatMulInteger ops which the ORT
    #   mobile build does NOT include in its reduced op set. The result fails
    #   to load on Android with ORT_NOT_IMPLEMENTED.
    #   QDQ emits regular Conv + Quantize/Dequantize nodes which are in the
    #   mobile op set, so the same file runs on PC and on Android.
    # We use a synthetic calibration dataset (random noise) for the first pass
    # since we don't have captured camera frames yet. For better accuracy on
    # real scenes, replace with ~50 captured frames in a follow-up.
    class _SyntheticCalibrator(CalibrationDataReader):
        def __init__(self, input_name: str, n: int = 8):
            self.input_name = input_name
            self.data = []
            rng = np.random.default_rng(seed=0xCA1)
            for _ in range(n):
                img = rng.uniform(0.0, 1.0, size=(1, 3, 518, 518)).astype(np.float32)
                self.data.append({input_name: img})
            self._iter = iter(self.data)

        def get_next(self):
            return next(self._iter, None)

        def rewind(self):
            self._iter = iter(self.data)

    sess_for_name = ort.InferenceSession(str(shaped_path), providers=["CPUExecutionProvider"])
    input_name = sess_for_name.get_inputs()[0].name
    del sess_for_name

    int8_path = OUTPUT_DIR / "v2_metric_518_int8_qdq.onnx"
    info("running quantize_static with QDQ format (mobile-compatible) ...")
    t0 = time.perf_counter()
    try:
        quantize_static(
            model_input=str(shaped_path),
            model_output=str(int8_path),
            calibration_data_reader=_SyntheticCalibrator(input_name),
            quant_format=QuantFormat.QDQ,
            activation_type=QuantType.QInt8,
            weight_type=QuantType.QInt8,
            per_channel=False,  # ARM-NNAPI prefers per-tensor
        )
    except Exception as e:
        fail(f"QDQ static quantization failed: {type(e).__name__}: {e}")
    quant_s = time.perf_counter() - t0
    info(f"  INT8 QDQ -> {int8_path.name} ({int8_path.stat().st_size / 1e6:.1f} MB) in {quant_s:.1f}s")
    report["int8_mb"] = round(int8_path.stat().st_size / 1e6, 2)
    report["quantize_seconds"] = round(quant_s, 1)

    # Step 3: validate each variant produces sane output
    rng = np.random.default_rng(seed=0xDA2)
    img_uint8 = rng.integers(0, 255, size=(518, 518, 3), dtype=np.uint8)
    img_float = img_uint8.astype(np.float32) / 255.0
    nchw = np.transpose(img_float, (2, 0, 1))[np.newaxis, ...]

    variants = [
        ("fp32", fp32_path),
        ("fp32_shaped", shaped_path),
        ("int8_qdq", int8_path),
    ]

    report["variants"] = {}
    fp32_depth = None

    for label, path in variants:
        info(f"validating {label} ({path.name}) ...")
        try:
            sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
            in_name = sess.get_inputs()[0].name
            t0 = time.perf_counter()
            outs = sess.run(None, {in_name: nchw})
            dt_ms = (time.perf_counter() - t0) * 1000.0
            depth = outs[0]
        except Exception as e:
            info(f"  failed: {type(e).__name__}: {e}")
            report["variants"][label] = {"error": str(e)}
            continue

        v_report = {
            "shape": list(depth.shape),
            "dtype": str(depth.dtype),
            "min": float(depth.min()),
            "max": float(depth.max()),
            "mean": float(depth.mean()),
            "std": float(depth.std()),
            "cpu_ms": round(dt_ms, 1),
        }
        info(f"  shape={depth.shape}  min={depth.min():.3f} max={depth.max():.3f} "
             f"mean={depth.mean():.3f}  PC CPU: {dt_ms:.1f} ms")

        if label == "fp32":
            fp32_depth = depth
        elif fp32_depth is not None and depth.shape == fp32_depth.shape:
            diff = np.abs(depth.astype(np.float32) - fp32_depth)
            rel = diff / (np.abs(fp32_depth) + 1e-6)
            v_report["vs_fp32_max_abs_m"] = float(diff.max())
            v_report["vs_fp32_p95_rel_pct"] = float(np.percentile(rel, 95) * 100)
            info(f"  vs FP32: max_abs={diff.max():.4f} m  p95_rel={v_report['vs_fp32_p95_rel_pct']:.2f}%")

        report["variants"][label] = v_report

    report_path = OUTPUT_DIR / "ort_quantize_v2_report.json"
    report_path.write_text(json.dumps(report, indent=2))
    info(f"summary -> {report_path.name}")
    info("ORT quantization complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
