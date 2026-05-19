"""
DA3-Small ONNX -> TFLite conversion — Phase 2 Step 4 / Appendix B.1.3.

Goal: take the FP32 ONNX produced by export_onnx.py and emit three TFLite
variants for on-device benchmarking:

    da3_small_<RES>_fp32.tflite    full precision baseline
    da3_small_<RES>_fp16.tflite    half precision (Mali-G78 GPU delegate target)
    da3_small_<RES>_int8.tflite    INT8 dynamic-range quantization (NNAPI target)

The conversion path is ONNX -> TF saved_model -> TFLite via the `onnx2tf` package.
This is the most reliable path for transformer ops as of May 2026; the older
`onnx-tf` package has worse op coverage.

Usage:
    python scripts/da3_benchmark/export_tflite.py --resolution 504

Outputs:
    output/da3_small_<RES>_fp32.tflite
    output/da3_small_<RES>_fp16.tflite
    output/da3_small_<RES>_int8.tflite
    output/tflite_conversion_<RES>.json   (sizes, per-variant op coverage)

Note: int8 *static* quantization (with calibration data) gives better accuracy
than dynamic-range, but requires a representative dataset of camera frames.
We start with dynamic-range here; upgrade to static once we have ~50 captured
frames from the phone.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

OUTPUT_DIR = Path(__file__).resolve().parent / "output"


def fail(msg: str, code: int = 1) -> None:
    print(f"[export_tflite] FAIL: {msg}", file=sys.stderr)
    sys.exit(code)


def info(msg: str) -> None:
    print(f"[export_tflite] {msg}")


def main() -> int:
    parser = argparse.ArgumentParser(description="DA3-Small ONNX -> TFLite (FP32/FP16/INT8)")
    parser.add_argument("--resolution", type=int, default=504, choices=(252, 378, 504))
    parser.add_argument("--skip-fp32", action="store_true")
    parser.add_argument("--skip-fp16", action="store_true")
    parser.add_argument("--skip-int8", action="store_true")
    args = parser.parse_args()

    onnx_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp32.onnx"
    if not onnx_path.exists():
        fail(f"ONNX not found at {onnx_path}. Run export_onnx.py first.")

    info(f"input: {onnx_path.name} ({onnx_path.stat().st_size / 1e6:.1f} MB)")

    try:
        import onnx2tf
    except ImportError:
        fail("onnx2tf not installed. Run: pip install onnx2tf")
    try:
        import tensorflow as tf
    except ImportError:
        fail("tensorflow not installed. Run: pip install tensorflow")

    info(f"onnx2tf {onnx2tf.__version__} | tensorflow {tf.__version__}")

    report = {"resolution": args.resolution, "variants": {}}

    # Step 1: ONNX -> TF saved_model -> TFLite FP32
    saved_model_dir = OUTPUT_DIR / f"da3_small_{args.resolution}_saved_model"
    if saved_model_dir.exists():
        shutil.rmtree(saved_model_dir)

    info(f"converting ONNX -> TF saved_model in {saved_model_dir.name}/ ...")
    t0 = time.perf_counter()
    try:
        # onnx2tf produces both a saved_model dir AND a default fp32 .tflite
        # alongside it. We capture the saved_model and re-emit each variant
        # explicitly so we control quantization config.
        onnx2tf.convert(
            input_onnx_file_path=str(onnx_path),
            output_folder_path=str(saved_model_dir),
            copy_onnx_input_output_names_to_tflite=True,
            non_verbose=True,
        )
    except Exception as e:
        fail(f"onnx2tf conversion failed: {type(e).__name__}: {e}\n"
             f"Common causes: unsupported op (look for 'aten::xxx' in error), "
             f"shape mismatch, or onnx version mismatch (try pip install onnx==1.16.2)")
    o2t_ms = (time.perf_counter() - t0) * 1000.0
    info(f"  ONNX -> saved_model done in {o2t_ms/1000:.1f} s")

    # onnx2tf already emits a default fp32 tflite in the saved_model dir.
    # Find it and rename to our target.
    default_tflite = next(saved_model_dir.glob("*_float32.tflite"), None)
    if default_tflite is None:
        fail(f"onnx2tf did not produce a .tflite in {saved_model_dir} — check logs")

    if not args.skip_fp32:
        fp32_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp32.tflite"
        shutil.copy2(default_tflite, fp32_path)
        size_mb = fp32_path.stat().st_size / 1e6
        info(f"  FP32 -> {fp32_path.name}  ({size_mb:.1f} MB)")
        report["variants"]["fp32"] = {"size_mb": round(size_mb, 2), "path": fp32_path.name}

    # Step 2: FP16 (post-training half-precision)
    if not args.skip_fp16:
        info("emitting FP16 variant ...")
        # onnx2tf already produces a *_float16.tflite if -oiqt is used; but
        # since we ran without quant flags, re-run from saved_model:
        converter = tf.lite.TFLiteConverter.from_saved_model(str(saved_model_dir))
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.target_spec.supported_types = [tf.float16]
        try:
            tflite_fp16 = converter.convert()
        except Exception as e:
            info(f"  FP16 conversion failed: {type(e).__name__}: {e}")
            report["variants"]["fp16"] = {"error": str(e)}
        else:
            fp16_path = OUTPUT_DIR / f"da3_small_{args.resolution}_fp16.tflite"
            fp16_path.write_bytes(tflite_fp16)
            size_mb = fp16_path.stat().st_size / 1e6
            info(f"  FP16 -> {fp16_path.name}  ({size_mb:.1f} MB)")
            report["variants"]["fp16"] = {"size_mb": round(size_mb, 2), "path": fp16_path.name}

    # Step 3: INT8 dynamic-range quantization (no calibration dataset)
    if not args.skip_int8:
        info("emitting INT8 dynamic-range variant ...")
        converter = tf.lite.TFLiteConverter.from_saved_model(str(saved_model_dir))
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        # Dynamic range: weights INT8, activations FP32 at runtime; quantized
        # to INT8 dynamically. No calibration dataset required.
        try:
            tflite_int8 = converter.convert()
        except Exception as e:
            info(f"  INT8 conversion failed: {type(e).__name__}: {e}")
            report["variants"]["int8_dynamic"] = {"error": str(e)}
        else:
            int8_path = OUTPUT_DIR / f"da3_small_{args.resolution}_int8.tflite"
            int8_path.write_bytes(tflite_int8)
            size_mb = int8_path.stat().st_size / 1e6
            info(f"  INT8 -> {int8_path.name}  ({size_mb:.1f} MB)")
            report["variants"]["int8_dynamic"] = {"size_mb": round(size_mb, 2), "path": int8_path.name}

    # Summary
    report_path = OUTPUT_DIR / f"tflite_conversion_{args.resolution}.json"
    report_path.write_text(json.dumps(report, indent=2))
    info(f"summary -> {report_path}")
    info("TFLite conversion complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
