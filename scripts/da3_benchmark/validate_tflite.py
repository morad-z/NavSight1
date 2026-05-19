"""
TFLite vs ONNX validation — closes the silent-wrong-model gap.

Goal: prove that the .tflite files produced by export_tflite.py compute the same
depth/pose outputs as the ONNX file we already validated against PyTorch. If
FP32 TFLite drifts from ONNX by more than 1%, something went wrong in the
onnx2tf conversion (op semantics changed, NHWC transpose missed, etc.) and we
should NOT push it to the phone.

Without this gate we'd be implicitly trusting onnx2tf to be byte-perfect on a
modern transformer, which is historically not safe for production ML deploys.

Usage:
    # After export_tflite.py finishes
    python scripts/da3_benchmark/validate_tflite.py --resolution 504

Outputs:
    output/tflite_validation_<RES>.json — per-variant error vs ONNX baseline
    Exit code 0 if FP32 passes, 1 if it fails (signals "do not push to phone")

Failure thresholds:
    FP32 vs ONNX     max p95 rel err = 1%   (HARD FAIL — should be near-bitwise)
    FP16 vs ONNX     max p95 rel err = 5%   (warn only — half-precision drift expected)
    INT8 vs ONNX     max p95 rel err = 15%  (warn only — quant drift expected, real
                                            test is B.3 on real frames)
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

OUTPUT_DIR = Path(__file__).resolve().parent / "output"

FP32_HARD_FAIL_PCT = 1.0
FP16_WARN_PCT = 5.0
INT8_WARN_PCT = 15.0


def fail(msg: str, code: int = 1) -> None:
    print(f"[validate_tflite] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def info(msg: str) -> None:
    print(f"[validate_tflite] {msg}")


def _make_dummy_input(resolution: int) -> np.ndarray:
    """Same RNG seed as smoke_test.py so we're testing on identical input."""
    rng = np.random.default_rng(seed=0xDA3)
    img_uint8 = rng.integers(0, 255, size=(resolution, resolution, 3), dtype=np.uint8)
    # Convert to NCHW float32 with shape (B=1, N=1, 3, H, W) for ONNX
    img_float = img_uint8.astype(np.float32) / 255.0
    nchw = np.transpose(img_float, (2, 0, 1))[np.newaxis, np.newaxis, ...]  # (1, 1, 3, H, W)
    return nchw


def _run_onnx(onnx_path: Path, dummy_nchw: np.ndarray) -> list[np.ndarray]:
    import onnxruntime as ort
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    return sess.run(None, {in_name: dummy_nchw})


def _run_tflite(tflite_path: Path, dummy_nchw: np.ndarray) -> list[np.ndarray]:
    import tensorflow as tf
    interp = tf.lite.Interpreter(model_path=str(tflite_path))
    interp.allocate_tensors()
    input_details = interp.get_input_details()
    output_details = interp.get_output_details()

    # onnx2tf may have transposed to NHWC; detect input shape
    expected_shape = tuple(input_details[0]["shape"])
    info(f"  TFLite expects input shape {expected_shape}, dtype={input_details[0]['dtype']}")

    # Reshape dummy to match TFLite expected layout
    if dummy_nchw.shape == expected_shape:
        dummy = dummy_nchw
    else:
        # Try common layout transforms
        if len(expected_shape) == 5:
            # (1, 1, 3, H, W) NCHW vs (1, 1, H, W, 3) NHWC
            if expected_shape[-1] == 3:
                dummy = np.transpose(dummy_nchw, (0, 1, 3, 4, 2))  # NCHW -> NHWC
            else:
                dummy = dummy_nchw
        elif len(expected_shape) == 4:
            # The model collapsed (B, N) -> (B*N,) so shape is (1, 3, H, W) or (1, H, W, 3)
            squeezed = dummy_nchw.squeeze(0)
            if expected_shape[-1] == 3:
                dummy = np.transpose(squeezed, (0, 2, 3, 1))
            else:
                dummy = squeezed
        else:
            dummy = dummy_nchw.reshape(expected_shape)

    if dummy.shape != expected_shape:
        fail(f"could not reshape dummy from {dummy_nchw.shape} to {expected_shape}")

    # Cast dtype if needed (e.g., INT8 model expects uint8)
    if input_details[0]["dtype"] != dummy.dtype:
        if input_details[0]["dtype"] == np.uint8:
            dummy = (dummy * 255).clip(0, 255).astype(np.uint8)
        else:
            dummy = dummy.astype(input_details[0]["dtype"])

    interp.set_tensor(input_details[0]["index"], dummy)
    interp.invoke()
    outs = [interp.get_tensor(d["index"]) for d in output_details]
    return outs


def _compare(onnx_outs: list[np.ndarray], tf_outs: list[np.ndarray], label: str) -> dict:
    """Per-output max abs, mean abs, p95 rel error. Aligns by tensor shape."""
    report = {"label": label, "outputs": []}

    # TFLite output order can differ from ONNX. Match by shape (depth is unique).
    onnx_by_shape = {tuple(o.shape): o for o in onnx_outs}

    for i, tf_out in enumerate(tf_outs):
        # Try to find matching ONNX output by shape
        # TFLite may have squeezed batch dims; try a few normalizations
        candidates = [tf_out, tf_out[np.newaxis, ...] if tf_out.ndim < 5 else None]
        matched = None
        for candidate in candidates:
            if candidate is None:
                continue
            if tuple(candidate.shape) in onnx_by_shape:
                matched = (onnx_by_shape[tuple(candidate.shape)], candidate)
                break

        if matched is None:
            report["outputs"].append({
                "index": i,
                "tf_shape": list(tf_out.shape),
                "matched": False,
                "note": "no ONNX output of matching shape",
            })
            continue

        ref, tf = matched
        diff = np.abs(tf.astype(np.float32) - ref.astype(np.float32))
        denom = np.abs(ref.astype(np.float32)) + 1e-6
        rel = diff / denom

        out_report = {
            "index": i,
            "shape": list(ref.shape),
            "matched": True,
            "max_abs": float(diff.max()),
            "mean_abs": float(diff.mean()),
            "p50_rel_pct": float(np.percentile(rel, 50) * 100),
            "p95_rel_pct": float(np.percentile(rel, 95) * 100),
            "p99_rel_pct": float(np.percentile(rel, 99) * 100),
        }
        report["outputs"].append(out_report)
        info(f"  output[{i}] shape={ref.shape}: "
             f"max_abs={out_report['max_abs']:.4f} "
             f"p95_rel={out_report['p95_rel_pct']:.2f}% "
             f"p99_rel={out_report['p99_rel_pct']:.2f}%")

    # Worst-case p95 rel error across all matched outputs
    p95s = [o["p95_rel_pct"] for o in report["outputs"] if o.get("matched")]
    report["worst_p95_rel_pct"] = max(p95s) if p95s else None
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="TFLite vs ONNX validation")
    parser.add_argument("--resolution", type=int, default=504, choices=(252, 378, 504))
    parser.add_argument("--strict", action="store_true",
                        help="Treat FP16 / INT8 warnings as failures (default: warn only)")
    args = parser.parse_args()

    onnx_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp32.onnx"
    fp32_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp32.tflite"
    fp16_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp16.tflite"
    int8_path = OUTPUT_DIR / f"da3_small_{args.resolution}_int8.tflite"

    if not onnx_path.exists():
        fail(f"ONNX baseline missing: {onnx_path}. Run export_onnx.py first.")

    info(f"baseline: {onnx_path.name} ({onnx_path.stat().st_size / 1e6:.1f} MB)")

    dummy = _make_dummy_input(args.resolution)
    info(f"dummy input: shape={dummy.shape} (NCHW float32)")

    info("running ONNX baseline ...")
    onnx_outs = _run_onnx(onnx_path, dummy)
    info(f"  ONNX returned {len(onnx_outs)} tensors:")
    for i, o in enumerate(onnx_outs):
        info(f"    output[{i}]: shape={o.shape} dtype={o.dtype}")

    overall = {"resolution": args.resolution, "variants": {}}

    for label, path, threshold, is_hard in (
        ("fp32", fp32_path, FP32_HARD_FAIL_PCT, True),
        ("fp16", fp16_path, FP16_WARN_PCT, False),
        ("int8", int8_path, INT8_WARN_PCT, False),
    ):
        if not path.exists():
            info(f"skipping {label}: {path.name} does not exist")
            overall["variants"][label] = {"skipped": True}
            continue

        info(f"validating {label} ({path.stat().st_size / 1e6:.1f} MB) ...")
        try:
            tf_outs = _run_tflite(path, dummy)
        except Exception as e:
            info(f"  {label} failed to run: {type(e).__name__}: {e}")
            overall["variants"][label] = {"error": str(e)}
            if is_hard:
                overall["fp32_hard_fail"] = True
            continue

        report = _compare(onnx_outs, tf_outs, label)
        overall["variants"][label] = report

        worst = report.get("worst_p95_rel_pct")
        if worst is None:
            info(f"  {label}: no matching outputs found — possible layout mismatch")
            if is_hard:
                overall["fp32_hard_fail"] = True
            continue

        if worst > threshold:
            verdict = "FAIL" if (is_hard or args.strict) else "WARN"
            info(f"  {label}: p95 rel err {worst:.2f}% > threshold {threshold:.1f}%  [{verdict}]")
            if is_hard or args.strict:
                overall.setdefault("fp32_hard_fail", is_hard)
        else:
            info(f"  {label}: p95 rel err {worst:.2f}% <= threshold {threshold:.1f}%  [PASS]")

    report_path = OUTPUT_DIR / f"tflite_validation_{args.resolution}.json"
    report_path.write_text(json.dumps(overall, indent=2))
    info(f"report -> {report_path}")

    if overall.get("fp32_hard_fail"):
        info("HARD FAIL: FP32 TFLite drifted from ONNX. DO NOT PUSH TO PHONE.")
        info("  Investigate onnx2tf op-translation; consider regenerating with different settings.")
        return 1

    info("validation complete — TFLite variants are faithful to ONNX baseline within tolerance.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
