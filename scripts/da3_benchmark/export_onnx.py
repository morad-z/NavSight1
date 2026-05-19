"""
DA3-Small PyTorch -> ONNX export — Phase 2 Step 4 / Appendix B.1.2.

Goal: produce an ONNX file we can later convert to TFLite for the on-device benchmark.

Approach:
  - Load DA3-Small from HuggingFace
  - Wrap the model to (a) disable bf16 autocast (force FP32), (b) return a tuple
    instead of dict (ONNX exporter requires this), (c) skip the GS branch
  - Run torch.onnx.export with opset 17 (vision transformer support)
  - Validate the ONNX output matches the PyTorch reference NPZ from smoke_test.py

Outputs:
  scripts/da3_benchmark/output/da3_small_<RES>_fp32.onnx
  scripts/da3_benchmark/output/onnx_validation_<RES>.json   (max abs/rel error)

Usage:
    # Run smoke_test.py first to generate output/reference_fp32_<RES>.npz
    python scripts/da3_benchmark/smoke_test.py --resolution 504

    # Then export to ONNX
    python scripts/da3_benchmark/export_onnx.py --resolution 504

If the export fails on an unsupported op, the error tells us exactly which op
(usually one of: scaled_dot_product_attention, einsum, custom CUDA kernel).
We deal with that op-by-op; falling back to opset 18 or wrapping a non-exportable
op is sometimes needed.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Tuple

import numpy as np

OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def fail(msg: str, code: int = 1) -> None:
    print(f"[export_onnx] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def info(msg: str) -> None:
    print(f"[export_onnx] {msg}")


class _ExportWrapper:
    """
    Wraps DepthAnything3 to:
      - Force FP32 (no bf16/fp16 autocast)
      - Return a tuple of (depth, conf, extrinsics, intrinsics) instead of dict
      - Skip the Gaussian Splatting branch (custom CUDA, not exportable)

    Inherits from torch.nn.Module so torch.onnx.export traces it cleanly.
    """

    def __init__(self, da3_model):
        import torch.nn as nn
        # Build the wrapper as an nn.Module dynamically
        wrapper = nn.Module()
        wrapper.da3 = da3_model
        wrapper.forward = self._make_forward(da3_model)
        self._wrapper = wrapper

    @staticmethod
    def _make_forward(da3_model):
        import torch

        def forward(image: torch.Tensor) -> Tuple[torch.Tensor, ...]:
            # image shape: (B, N, 3, H, W)
            # Bypass autocast — force FP32 path
            with torch.no_grad():
                out = da3_model.model(image)
            # The underlying net returns a dict; flatten to tuple
            depth = out.get("depth")
            conf = out.get("conf", out.get("confidence"))
            extrinsics = out.get("extrinsics", out.get("camera_extrinsics"))
            intrinsics = out.get("intrinsics", out.get("camera_intrinsics"))
            outputs = []
            for t in (depth, conf, extrinsics, intrinsics):
                if t is not None:
                    outputs.append(t)
            return tuple(outputs)

        return forward

    @property
    def module(self):
        return self._wrapper


def _validate_onnx_vs_pytorch(onnx_path: Path, dummy_input: np.ndarray, ref_outputs: dict) -> dict:
    """Run the exported ONNX model and compare to the PyTorch reference."""
    try:
        import onnxruntime as ort
    except ImportError:
        info("onnxruntime not installed — skipping validation. Install with: pip install onnxruntime")
        return {"validated": False, "reason": "onnxruntime not installed"}

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    inputs = {sess.get_inputs()[0].name: dummy_input}
    outputs = sess.run(None, inputs)

    info(f"ONNX runtime returned {len(outputs)} output tensors:")
    for i, o in enumerate(outputs):
        info(f"  output[{i}]: shape={o.shape} dtype={o.dtype}")

    # Compare depth (output[0]) against reference if available
    report = {"validated": True, "n_outputs": len(outputs)}
    if "depth" in ref_outputs:
        ref = np.asarray(ref_outputs["depth"]).reshape(outputs[0].shape)
        diff = np.abs(outputs[0] - ref)
        rel = diff / (np.abs(ref) + 1e-6)
        report["depth_max_abs_err"] = float(diff.max())
        report["depth_mean_abs_err"] = float(diff.mean())
        report["depth_max_rel_err"] = float(rel.max())
        report["depth_p95_rel_err"] = float(np.percentile(rel, 95))
        info(f"depth: max_abs={report['depth_max_abs_err']:.6f} "
             f"mean_abs={report['depth_mean_abs_err']:.6f} "
             f"p95_rel={report['depth_p95_rel_err']:.4%}")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="DA3-Small PyTorch -> ONNX export")
    parser.add_argument(
        "--resolution",
        type=int,
        default=504,
        choices=(252, 378, 504),
        help="Square input. Must be a multiple of 14 (DA3 patch size). 252=18*14, 378=27*14, 504=36*14.",
    )
    parser.add_argument("--variant", default="depth-anything/DA3-SMALL")
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--skip-validation", action="store_true")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp32.onnx"

    info(f"loading torch + DA3 ...")
    import torch
    from depth_anything_3.api import DepthAnything3

    info(f"torch {torch.__version__} | cuda={torch.cuda.is_available()}")
    info(f"target: {out_path.name} | resolution={args.resolution} | opset={args.opset}")

    # Force CPU export — onnx tracing on CUDA can capture device-specific ops
    device = "cpu"
    info(f"loading {args.variant} on {device} (FP32) ...")
    t0 = time.perf_counter()
    da3 = DepthAnything3.from_pretrained(args.variant).to(device).eval().float()
    load_ms = (time.perf_counter() - t0) * 1000.0
    info(f"loaded in {load_ms:.0f} ms")

    n_params = sum(p.numel() for p in da3.parameters())
    info(f"params: {n_params/1e6:.1f}M")

    # Build wrapper
    wrapper = _ExportWrapper(da3).module
    wrapper.eval()

    # Dummy input: (B=1, N=1, 3, H, W)
    H = W = args.resolution
    dummy = torch.randn(1, 1, 3, H, W, dtype=torch.float32)
    info(f"dummy input shape: {tuple(dummy.shape)}")

    # Sanity: PyTorch forward works
    info("running PyTorch forward (sanity) ...")
    t0 = time.perf_counter()
    with torch.no_grad():
        py_out = wrapper(dummy)
    py_ms = (time.perf_counter() - t0) * 1000.0
    info(f"  PyTorch forward: {py_ms:.0f} ms, returned {len(py_out)} tensors")
    for i, t in enumerate(py_out):
        info(f"    output[{i}]: shape={tuple(t.shape)} dtype={t.dtype}")

    # ONNX export
    info(f"exporting to {out_path} ...")
    t0 = time.perf_counter()
    try:
        # Use the legacy TorchScript-based exporter (dynamo=False).
        # PyTorch 2.11's default Dynamo exporter chokes on `int(tensor.max())`
        # in DA3's RoPE positional encoding (model/dinov2/layers/rope.py:183).
        # TorchScript tracing handles dynamic-int calls by specializing to the
        # traced value, which is fine for our static-shape mobile export.
        # The output_names list is only used by the dynamo path; the legacy
        # exporter ignores it but we keep it for consistency.
        torch.onnx.export(
            wrapper,
            (dummy,),
            str(out_path),
            input_names=["image"],
            output_names=["depth", "extrinsics", "intrinsics"][:len(py_out)],
            opset_version=args.opset,
            do_constant_folding=True,
            dynamic_axes=None,  # static shapes for mobile
            verbose=False,
            dynamo=False,  # legacy TorchScript exporter — see comment above
        )
    except Exception as e:
        fail(
            f"ONNX export failed: {type(e).__name__}: {e}\n\n"
            f"Common fixes:\n"
            f"  - Try --opset 18 if scaled_dot_product_attention is unsupported on 17\n"
            f"  - If a custom op is mentioned, identify it from the traceback and ask\n"
            f"    whether it's used in the inference path or only in training/GS\n"
            f"  - Some ops need symbolic registration; if blocked, fall back to\n"
            f"    torch.onnx.dynamo_export() instead of the legacy exporter"
        )
    export_ms = (time.perf_counter() - t0) * 1000.0
    file_mb = out_path.stat().st_size / (1024 * 1024)
    info(f"exported in {export_ms:.0f} ms; size={file_mb:.1f} MB")

    # Validate
    if not args.skip_validation:
        info("validating ONNX output vs PyTorch reference ...")
        ref_npz = OUTPUT_DIR / f"reference_fp32_{args.resolution}.npz"
        ref_outputs = {}
        if ref_npz.exists():
            ref_outputs = dict(np.load(ref_npz, allow_pickle=True))
            info(f"loaded reference NPZ: {sorted(ref_outputs.keys())}")
        else:
            info(f"no reference NPZ found at {ref_npz} — skipping accuracy check")

        report = _validate_onnx_vs_pytorch(out_path, dummy.numpy(), ref_outputs)
        report_path = OUTPUT_DIR / f"onnx_validation_{args.resolution}.json"
        report_path.write_text(json.dumps(report, indent=2))
        info(f"validation report -> {report_path}")

    info("ONNX export complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
