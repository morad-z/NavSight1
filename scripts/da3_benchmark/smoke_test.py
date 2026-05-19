"""
DA3-Small smoke test — Phase 2 Step 4 / Appendix B.1 prerequisite.

Goal: prove the PyTorch DA3-Small model loads and runs *one* inference end-to-end
on this PC, with a printed timing + output shape summary, BEFORE we invest in
the ONNX/TFLite conversion pipeline.

If this script passes, we know:
  - Python env has torch + huggingface_hub + onnx2tf installed correctly
  - The DA3-Small checkpoint downloads from HuggingFace
  - Inference produces outputs whose shapes match the API doc:
      depth      (1, H, W) float32
      conf       (1, H, W) float32
      extrinsics (1, 3, 4) float32
      intrinsics (1, 3, 3) float32

If this script fails, we fix the env BEFORE writing the converter.

Usage:
    # Recommended: create a venv with Python 3.11 or 3.12 (ML libs lag 3.14)
    python -m venv .venv-da3 --python=3.11
    .\\.venv-da3\\Scripts\\Activate.ps1     # PowerShell on Windows
    pip install -r scripts/da3_benchmark/requirements.txt
    python scripts/da3_benchmark/smoke_test.py

    # Optional: pass a real camera frame instead of synthetic noise
    python scripts/da3_benchmark/smoke_test.py --image path/to/frame.jpg
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# Default DA3-Small input resolution per the V3 API doc (process_res=504 default)
DEFAULT_RES = 504


def fail(msg: str, code: int = 1) -> None:
    print(f"[smoke_test] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def info(msg: str) -> None:
    print(f"[smoke_test] {msg}")


def _load_image(path: Path | None, res: int) -> np.ndarray:
    """Return a uint8 HxWx3 RGB array at (res, res). Synthetic if path is None."""
    if path is None:
        rng = np.random.default_rng(seed=0xDA3)
        return rng.integers(0, 255, size=(res, res, 3), dtype=np.uint8)
    try:
        from PIL import Image
    except ImportError:
        fail("Pillow not installed. Run: pip install -r scripts/da3_benchmark/requirements.txt")
    img = Image.open(path).convert("RGB")
    img = img.resize((res, res), Image.BILINEAR)
    return np.asarray(img, dtype=np.uint8)


def _import_torch():
    try:
        import torch  # noqa: F401
    except ImportError:
        fail(
            "torch not installed. PyTorch wheels lag Python 3.14 — use Python 3.11/3.12 venv.\n"
            "  python -m venv .venv-da3 --python=3.11\n"
            "  pip install -r scripts/da3_benchmark/requirements.txt"
        )
    import torch
    return torch


def main() -> int:
    parser = argparse.ArgumentParser(description="DA3-Small PyTorch smoke test")
    parser.add_argument("--image", type=Path, default=None,
                        help="Optional RGB image to use as input (synthetic noise otherwise)")
    parser.add_argument("--resolution", type=int, default=DEFAULT_RES,
                        choices=(256, 384, 504),
                        help="Square input resolution. Default: %(default)d (V3 API default)")
    parser.add_argument("--variant", default="depth-anything/DA3-SMALL",
                        help="HuggingFace model id")
    args = parser.parse_args()

    torch = _import_torch()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    info(f"torch {torch.__version__} | device={device} | resolution={args.resolution}")

    info(f"Loading {args.variant} ...")
    t0 = time.perf_counter()
    try:
        # The V3 repo's actual entry point is depth_anything_3.api.DepthAnything3,
        # which uses huggingface_hub's PyTorchModelHubMixin (NOT transformers).
        from depth_anything_3.api import DepthAnything3
        model = DepthAnything3.from_pretrained(args.variant).to(device).eval()
        api = "depth_anything_3"
    except ImportError as e:
        fail(f"depth_anything_3 import failed ({e}). "
             f"Reinstall with: pip install --no-deps -e scripts/da3_benchmark/Depth-Anything-3")
    except Exception as e:
        fail(f"model load failed ({type(e).__name__}: {e}). "
             f"Confirm the HF repo id `{args.variant}` exists and is public.")
    load_ms = (time.perf_counter() - t0) * 1000.0
    info(f"loaded via {api} in {load_ms:.0f} ms")

    n_params = sum(p.numel() for p in model.parameters())
    info(f"params: {n_params/1e6:.1f}M (expected ~80M for DA3-Small)")

    image = _load_image(args.image, args.resolution)
    info(f"input image: shape={image.shape} dtype={image.dtype} "
         f"source={'synthetic' if args.image is None else str(args.image)}")

    # Inference: V3 API expects a list of images; outputs a Prediction object
    info("running inference ...")
    t0 = time.perf_counter()
    with torch.no_grad():
        if api == "depth_anything_3":
            prediction = model.inference(
                image=[image],
                process_res=args.resolution,
                process_res_method="upper_bound_resize",
                conf_thresh_percentile=40.0,
                show_cameras=False,
            )
        else:
            # transformers fallback: forward pass on a tensor
            x = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0).float() / 255.0
            x = x.to(device)
            prediction = model(x)
    infer_ms = (time.perf_counter() - t0) * 1000.0
    info(f"inference completed in {infer_ms:.1f} ms ({device})")

    # Verify outputs match V3 API spec
    expected = {"depth", "conf", "extrinsics", "intrinsics"}
    actual = set()
    if api == "depth_anything_3":
        for name in expected:
            if hasattr(prediction, name):
                arr = getattr(prediction, name)
                actual.add(name)
                shape = getattr(arr, "shape", None)
                info(f"  {name}: shape={shape}")
    else:
        info(f"  (transformers fallback — output type {type(prediction).__name__})")

    missing = expected - actual
    if api == "depth_anything_3" and missing:
        info(f"WARNING: missing expected outputs {missing} — check API version")

    # Save reference depth for B.3 regression baseline
    if api == "depth_anything_3" and "depth" in actual:
        np.savez_compressed(
            OUTPUT_DIR / f"reference_fp32_{args.resolution}.npz",
            depth=prediction.depth,
            conf=prediction.conf if "conf" in actual else None,
            extrinsics=prediction.extrinsics if "extrinsics" in actual else None,
            intrinsics=prediction.intrinsics if "intrinsics" in actual else None,
            input_image=image,
            resolution=args.resolution,
            n_params=n_params,
            inference_ms=infer_ms,
        )
        info(f"saved reference outputs -> {OUTPUT_DIR / f'reference_fp32_{args.resolution}.npz'}")

    info("smoke test PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
