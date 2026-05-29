#!/usr/bin/env python3
"""Run midas_v21_small.tflite on recorded frames -> raw-disparity rasters.

Replicates DepthEstimator.kt exactly so the replay_harness can feed the SAME
MiDaS depth the on-device engine sees (the harness otherwise never calls
setDepthMap, starving updateDepthFlowSpeed). Per-frame output is a 256x256
float32 raw-disparity raster named "<frame_stem>.f32" (row-major, matching the
C++ depth_map_ layout: index = y*256 + x).

On-device contract (DepthEstimator.kt):
  - model midas_v21_small.tflite, input [1,256,256,3] f32 NHWC, RGB
  - resize full frame -> 256x256 BILINEAR (non-aspect-preserving stretch)
  - normalize: (pixel/255 - ImageNet_mean) / ImageNet_std, per RGB channel
  - output [1,256,256,1] f32 = raw disparity (relative inverse depth)

Fidelity caveat (logged, not hidden): recorded frames are GRAYSCALE (the device
SimulationFrameRecorder saves only the Y plane). On-device MiDaS ran on the
COLOR camera bitmap. We replicate gray->RGB; structure is preserved but values
differ slightly from the live color run. This is the closest faithful offline
reproduction available from grayscale fixtures.

Usage:
  python scripts/gen_midas_depth.py <frames_dir> <out_depth_dir> [--limit N]
"""
import argparse
import os
import sys
import struct
import numpy as np
from PIL import Image
from ai_edge_litert.interpreter import Interpreter

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "app", "src", "main", "assets", "midas_v21_small.tflite")
INPUT_SIZE = 256
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--limit", type=int, default=0, help="cap frames (debug)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    it = Interpreter(model_path=os.path.abspath(MODEL))
    it.allocate_tensors()
    in_idx = it.get_input_details()[0]["index"]
    out_idx = it.get_output_details()[0]["index"]

    pngs = sorted(f for f in os.listdir(args.frames_dir) if f.endswith(".png"))
    if args.limit:
        pngs = pngs[: args.limit]
    if not pngs:
        print("no PNGs in", args.frames_dir)
        return 1

    n = len(pngs)
    dmin_all, dmax_all = float("inf"), -float("inf")
    for i, fn in enumerate(pngs):
        stem = os.path.splitext(fn)[0]
        img = Image.open(os.path.join(args.frames_dir, fn)).convert("RGB")
        img = img.resize((INPUT_SIZE, INPUT_SIZE), Image.BILINEAR)
        arr = np.asarray(img, dtype=np.float32) / 255.0          # [0,1]
        arr = (arr - IMAGENET_MEAN) / IMAGENET_STD               # ImageNet
        it.set_tensor(in_idx, arr[np.newaxis, ...])
        it.invoke()
        depth = it.get_tensor(out_idx)[0, :, :, 0].astype(np.float32)  # 256x256
        dmin_all = min(dmin_all, float(depth.min()))
        dmax_all = max(dmax_all, float(depth.max()))
        # row-major float32 (y*256+x) == C++ depth_map_ layout
        with open(os.path.join(args.out_dir, stem + ".f32"), "wb") as fp:
            fp.write(depth.tobytes(order="C"))
        if i < 3 or (i + 1) % 200 == 0 or i == n - 1:
            print(f"  [{i+1}/{n}] {stem}  disp[min={depth.min():.3f} "
                  f"max={depth.max():.3f} mean={depth.mean():.3f}]")
    print(f"done: {n} depth rasters -> {args.out_dir}  "
          f"(global disp range {dmin_all:.3f}..{dmax_all:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
