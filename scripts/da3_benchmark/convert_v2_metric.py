"""
Depth-Anything V2 Metric Outdoor Small — full conversion pipeline.

Goal: produce a TFLite file for V2 Metric Outdoor Small that can be pushed
to the S21 Ultra. This is the model that enables Phase 2 Step 4 (scooter
speed) because it outputs metric depth in meters directly.

Why V2 instead of V3:
- V3's mobile tooling is broken (litert-torch unbuildable on PyPI; onnx2tf
  paged the system at 22 GB RAM)
- Qualcomm's V3 port is depth-only (loses pose/conf/intrinsics anyway)
- V2 small is 24.7M params vs V3's 34.3M — conversion should fit in RAM
- V2 has measured 32 ms on Snapdragon 8 Gen 3 (extrapolates to ~60-90 ms
  on Mali-G78, well within our budget)

Pipeline:
1. Load V2 Metric Outdoor Small from HuggingFace via transformers
2. Sanity check: run one inference on noise, confirm output isn't all black
   (known issue per HF discussion thread — full black output means model is
   miscalibrated for our use; would need to fall back to indoor variant)
3. Export to ONNX (legacy TorchScript exporter, opset 17)
4. Validate ONNX vs PyTorch (max abs / p95 rel error on depth)
5. Convert ONNX -> TFLite (FP32, FP16, INT8 dynamic) via onnx2tf
6. Validate TFLite vs ONNX

Usage:
    python scripts/da3_benchmark/convert_v2_metric.py --resolution 518
    # Or the smaller 252 if 518 paged the system in V3 work
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path
from typing import Tuple

import numpy as np

OUTPUT_DIR = Path(__file__).resolve().parent / "output"
DEFAULT_VARIANT = "depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf"


def fail(msg: str, code: int = 1) -> None:
    print(f"[v2_convert] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def info(msg: str) -> None:
    print(f"[v2_convert] {msg}")


def step_load_pytorch(variant: str, device: str):
    info(f"loading {variant} on {device} ...")
    t0 = time.perf_counter()
    try:
        from transformers import AutoModelForDepthEstimation, AutoImageProcessor
    except ImportError:
        fail("transformers not installed in venv")

    try:
        model = AutoModelForDepthEstimation.from_pretrained(variant).to(device).eval().float()
        processor = AutoImageProcessor.from_pretrained(variant)
    except Exception as e:
        fail(f"model load failed: {type(e).__name__}: {e}")

    n_params = sum(p.numel() for p in model.parameters())
    elapsed = (time.perf_counter() - t0) * 1000
    info(f"loaded in {elapsed:.0f} ms | params: {n_params/1e6:.1f}M")
    return model, processor, n_params


def step_sanity_inference(model, processor, resolution: int, device: str) -> Tuple[np.ndarray, dict]:
    """Run one inference; return depth array + sanity report. Aborts on black output."""
    info(f"sanity inference at {resolution}x{resolution} ...")
    import torch

    rng = np.random.default_rng(seed=0xDA2)
    img_uint8 = rng.integers(0, 255, size=(resolution, resolution, 3), dtype=np.uint8)

    # Build a tensor (B, 3, H, W) normalized as the processor would
    img_float = img_uint8.astype(np.float32) / 255.0
    nchw = np.transpose(img_float, (2, 0, 1))[np.newaxis, ...]
    pixel_values = torch.from_numpy(nchw).to(device)

    t0 = time.perf_counter()
    with torch.no_grad():
        outputs = model(pixel_values=pixel_values)
    dt_ms = (time.perf_counter() - t0) * 1000.0

    # transformers DepthEstimation returns predicted_depth: (B, H, W) in meters
    depth = outputs.predicted_depth.cpu().numpy()
    info(f"  forward: {dt_ms:.1f} ms | output shape={depth.shape} dtype={depth.dtype}")
    info(f"  depth: min={depth.min():.3f} max={depth.max():.3f} mean={depth.mean():.3f} std={depth.std():.3f}")

    sanity = {
        "shape": list(depth.shape),
        "min": float(depth.min()),
        "max": float(depth.max()),
        "mean": float(depth.mean()),
        "std": float(depth.std()),
        "forward_ms": dt_ms,
    }

    # Black-output check: if std < 1e-3 the model is producing a constant (broken)
    if depth.std() < 1e-3:
        fail(f"output has near-zero variance (std={depth.std():.6f}) — model producing flat/black depth. "
             f"Known issue with V2 Metric Outdoor Small. Consider switching to V2 Metric Indoor.")
    if not np.isfinite(depth).all():
        fail("output contains non-finite values (NaN/Inf)")

    return depth, sanity


def step_export_onnx(model, resolution: int, opset: int, out_path: Path) -> Tuple[Path, dict]:
    info(f"exporting ONNX to {out_path.name} (opset {opset}) ...")
    import torch
    import torch.nn as nn

    # Move to CPU for export (avoids CUDA-specific ops baked into the graph
    # and resolves device-mismatch errors with the dummy tensor).
    model_cpu = model.to("cpu").eval()

    # Wrap model so forward returns just the depth tensor
    class _Wrapper(nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, pixel_values):
            with torch.no_grad():
                out = self.m(pixel_values=pixel_values)
            return out.predicted_depth

    wrapper = _Wrapper(model_cpu).eval()

    dummy = torch.randn(1, 3, resolution, resolution, dtype=torch.float32)

    t0 = time.perf_counter()
    try:
        torch.onnx.export(
            wrapper,
            (dummy,),
            str(out_path),
            input_names=["pixel_values"],
            output_names=["predicted_depth"],
            opset_version=opset,
            do_constant_folding=True,
            dynamic_axes=None,  # static shapes for mobile
            verbose=False,
            dynamo=False,
        )
    except Exception as e:
        fail(f"ONNX export failed: {type(e).__name__}: {e}")
    dt_s = time.perf_counter() - t0
    file_mb = out_path.stat().st_size / 1e6
    info(f"  exported in {dt_s:.1f}s | size {file_mb:.1f} MB")
    return out_path, {"export_seconds": dt_s, "size_mb": file_mb}


def step_validate_onnx(onnx_path: Path, py_depth: np.ndarray, resolution: int) -> dict:
    info("validating ONNX vs PyTorch ...")
    import onnxruntime as ort

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])

    rng = np.random.default_rng(seed=0xDA2)
    img_uint8 = rng.integers(0, 255, size=(resolution, resolution, 3), dtype=np.uint8)
    img_float = img_uint8.astype(np.float32) / 255.0
    nchw = np.transpose(img_float, (2, 0, 1))[np.newaxis, ...]

    in_name = sess.get_inputs()[0].name
    onnx_outs = sess.run(None, {in_name: nchw})
    onnx_depth = onnx_outs[0]

    if onnx_depth.shape != py_depth.shape:
        info(f"  shape mismatch: pytorch {py_depth.shape} vs onnx {onnx_depth.shape}")
        return {"validated": False, "shape_mismatch": True}

    diff = np.abs(onnx_depth - py_depth)
    rel = diff / (np.abs(py_depth) + 1e-6)
    report = {
        "validated": True,
        "max_abs": float(diff.max()),
        "mean_abs": float(diff.mean()),
        "p95_rel_pct": float(np.percentile(rel, 95) * 100),
    }
    info(f"  max_abs={report['max_abs']:.6f} mean_abs={report['mean_abs']:.6f} "
         f"p95_rel={report['p95_rel_pct']:.2f}%")
    return report


def step_export_tflite(onnx_path: Path, resolution: int) -> dict:
    info("converting ONNX -> TFLite (FP32 + FP16 + INT8 dynamic) ...")
    try:
        # Monkey-patch numpy.load to allow pickle by default before importing
        # onnx2tf. Newer numpy 2.x rejects pickled object arrays unless
        # allow_pickle=True; older onnx2tf calls np.load without that flag.
        import numpy as _np
        _orig_np_load = _np.load
        def _np_load_allow_pickle(*args, **kwargs):
            kwargs.setdefault("allow_pickle", True)
            return _orig_np_load(*args, **kwargs)
        _np.load = _np_load_allow_pickle

        import onnx2tf
        import tensorflow as tf
    except ImportError:
        fail("onnx2tf or tensorflow not installed")

    saved_model_dir = OUTPUT_DIR / f"v2_metric_{resolution}_saved_model"
    if saved_model_dir.exists():
        shutil.rmtree(saved_model_dir)

    info(f"  ONNX -> TF saved_model in {saved_model_dir.name}/")
    t0 = time.perf_counter()
    try:
        onnx2tf.convert(
            input_onnx_file_path=str(onnx_path),
            output_folder_path=str(saved_model_dir),
            copy_onnx_input_output_names_to_tflite=True,
            non_verbose=True,
        )
    except Exception as e:
        fail(f"onnx2tf conversion failed: {type(e).__name__}: {e}")
    info(f"  saved_model done in {(time.perf_counter()-t0)/60:.1f} min")

    # Find the FP32 tflite onnx2tf produced
    default_tflite = next(saved_model_dir.glob("*_float32.tflite"), None)
    if default_tflite is None:
        fail(f"onnx2tf did not produce a .tflite in {saved_model_dir}")

    report = {"variants": {}}

    fp32_path = OUTPUT_DIR / f"v2_metric_{resolution}_fp32.tflite"
    shutil.copy2(default_tflite, fp32_path)
    info(f"  FP32 -> {fp32_path.name} ({fp32_path.stat().st_size/1e6:.1f} MB)")
    report["variants"]["fp32"] = {"size_mb": round(fp32_path.stat().st_size / 1e6, 2)}

    # FP16
    info("  emitting FP16 ...")
    converter = tf.lite.TFLiteConverter.from_saved_model(str(saved_model_dir))
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_types = [tf.float16]
    fp16_blob = converter.convert()
    fp16_path = OUTPUT_DIR / f"v2_metric_{resolution}_fp16.tflite"
    fp16_path.write_bytes(fp16_blob)
    info(f"  FP16 -> {fp16_path.name} ({fp16_path.stat().st_size/1e6:.1f} MB)")
    report["variants"]["fp16"] = {"size_mb": round(fp16_path.stat().st_size / 1e6, 2)}

    # INT8 dynamic
    info("  emitting INT8 dynamic ...")
    converter = tf.lite.TFLiteConverter.from_saved_model(str(saved_model_dir))
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    int8_blob = converter.convert()
    int8_path = OUTPUT_DIR / f"v2_metric_{resolution}_int8.tflite"
    int8_path.write_bytes(int8_blob)
    info(f"  INT8 -> {int8_path.name} ({int8_path.stat().st_size/1e6:.1f} MB)")
    report["variants"]["int8_dynamic"] = {"size_mb": round(int8_path.stat().st_size / 1e6, 2)}

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="V2 Metric Outdoor Small full conversion")
    parser.add_argument("--resolution", type=int, default=518,
                        help="Square input. V2 was trained at 518; smaller may need re-validation.")
    parser.add_argument("--variant", default=DEFAULT_VARIANT)
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--skip-tflite", action="store_true",
                        help="Stop after ONNX export + validation")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    info(f"V2 Metric Outdoor conversion | resolution={args.resolution}")
    info(f"variant: {args.variant}")

    import torch
    device = "cuda" if torch.cuda.is_available() else "cpu"
    info(f"torch {torch.__version__} | device={device}")

    overall = {
        "variant": args.variant,
        "resolution": args.resolution,
        "opset": args.opset,
    }

    model, processor, n_params = step_load_pytorch(args.variant, device)
    overall["params_million"] = round(n_params / 1e6, 1)

    py_depth, sanity = step_sanity_inference(model, processor, args.resolution, device)
    overall["sanity"] = sanity

    onnx_path = OUTPUT_DIR / f"v2_metric_{args.resolution}_fp32.onnx"
    _, export_report = step_export_onnx(model, args.resolution, args.opset, onnx_path)
    overall["onnx_export"] = export_report

    overall["onnx_validation"] = step_validate_onnx(onnx_path, py_depth, args.resolution)

    if args.skip_tflite:
        info("skipping TFLite (--skip-tflite)")
    else:
        overall["tflite"] = step_export_tflite(onnx_path, args.resolution)

    report_path = OUTPUT_DIR / f"v2_metric_{args.resolution}_report.json"
    report_path.write_text(json.dumps(overall, indent=2))
    info(f"summary -> {report_path}")
    info("V2 Metric conversion complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
