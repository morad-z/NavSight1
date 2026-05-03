#!/usr/bin/env python3
"""Offline camera-intrinsics calibration tool for NavSight.

This script consumes a directory of checkerboard photos taken with the
target Android phone (the one that will run NavSight) and produces a
JSON file describing the camera's pinhole intrinsics and rational-model
distortion coefficients. Step 8b of ``docs/VISUAL_PRODUCTION_PLAN.md``
will wire this JSON into ``LensCorrector`` and ``Tracker::initialize``
so ``cv::recoverPose`` operates on geometrically clean rays instead of
the current zero-distortion passthrough.

User workflow (see ``docs/CALIBRATION.md`` for the full procedure):
    1. Print a 9x6 OpenCV checkerboard at known square size.
    2. Take 30-50 photos at varied angles, distances, and lighting.
    3. Drop the photos into ``tests/calibration_images/<session>/``.
    4. Run::

        python scripts/calibrate_camera.py tests/calibration_images/<session>

       producing ``app/src/main/assets/camera_intrinsics.json`` plus a
       per-image reprojection error report next to it.

Implementation notes:
    * Uses ``cv2.findChessboardCornersSB`` (OpenCV >= 4.0). The "SB"
      variant is more robust to motion blur and uneven lighting than
      legacy ``findChessboardCorners``; it also returns sub-pixel
      corners directly so we do not need a separate ``cornerSubPix``
      refinement pass.
    * Uses the ``CALIB_RATIONAL_MODEL`` (8 distortion coefficients:
      k1, k2, p1, p2, k3, k4, k5, k6) matching what the Android side's
      ``LensCorrector`` will apply via ``cv::undistortPoints``.
    * Image-size sanity-checks every photo against the first one so
      that zoom-cropped or screenshot images do not pollute the fit.
    * Skips images where corners are not detected (e.g. board partly
      outside the frame, severe glare) and reports the skip count so
      the user can capture replacements for the failing perspectives.

Dependencies: ``opencv-python`` (``cv2``) and ``numpy`` only — both are
already used elsewhere in the project; no new requirement is introduced.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np

# OpenCV's reference checkerboard is 9 inner corners across by 6 down
# (i.e. printable as 10x7 squares). Default square edge is 25 mm when
# printed at 100% scale onto A4 paper — close enough for calibration
# purposes and verifiable with a ruler.
DEFAULT_COLS: int = 9
DEFAULT_ROWS: int = 6
DEFAULT_SQUARE_MM: float = 25.0
DEFAULT_OUTPUT: str = "app/src/main/assets/camera_intrinsics.json"

# RMS thresholds (px). These are warnings printed at the end, not hard
# gates, so the user can still inspect a marginal calibration. The
# Android-side runtime gate (Step 1 plan, RMS < 0.6 px) is enforced
# separately when the JSON is loaded into the app.
RMS_GOOD_THRESHOLD_PX: float = 0.5
RMS_USABLE_THRESHOLD_PX: float = 1.0

# Minimum useful number of accepted images. Below this the calibration
# is poorly constrained regardless of RMS.
MIN_IMAGES_REQUIRED: int = 10

SUPPORTED_EXTENSIONS: tuple[str, ...] = (
    ".jpg",
    ".jpeg",
    ".png",
    ".bmp",
    ".tif",
    ".tiff",
)


@dataclass(frozen=True)
class CheckerboardSpec:
    """Geometry of the printed checkerboard target.

    ``cols`` and ``rows`` are *inner* corner counts (e.g. 9x6 means
    a printed 10x7 grid of squares). ``square_mm`` is the edge length
    of one square in millimetres at print scale. The unit only affects
    the units of the output translation vectors; intrinsics and
    distortion are unaffected.
    """

    cols: int
    rows: int
    square_mm: float

    @property
    def pattern_size(self) -> tuple[int, int]:
        return (self.cols, self.rows)

    def object_points(self) -> np.ndarray:
        """Return the (cols*rows, 3) array of 3D corner positions in mm."""
        grid = np.zeros((self.rows * self.cols, 3), dtype=np.float32)
        grid[:, :2] = np.mgrid[0 : self.cols, 0 : self.rows].T.reshape(-1, 2)
        grid *= self.square_mm
        return grid


@dataclass(frozen=True)
class ImageDetection:
    """Result of running corner detection on a single image."""

    path: Path
    corners: np.ndarray  # shape (cols*rows, 1, 2), float32
    image_size: tuple[int, int]  # (width, height)


@dataclass(frozen=True)
class CalibrationResult:
    """Output of ``cv2.calibrateCamera`` plus bookkeeping."""

    image_size: tuple[int, int]
    camera_matrix: np.ndarray  # 3x3
    dist_coeffs: np.ndarray  # rational model: 8 values
    rms: float
    per_image_rms: list[float]
    accepted_paths: list[Path]


def find_image_files(image_dir: Path) -> list[Path]:
    """Return supported image files in ``image_dir`` sorted by name."""
    files: list[Path] = []
    for entry in sorted(image_dir.iterdir()):
        if entry.is_file() and entry.suffix.lower() in SUPPORTED_EXTENSIONS:
            files.append(entry)
    return files


def detect_corners(
    image_path: Path, board: CheckerboardSpec
) -> ImageDetection | None:
    """Detect checkerboard corners in a single image.

    Returns ``None`` when the image cannot be read or the corners are
    not detected. ``findChessboardCornersSB`` is used with the
    ``EXHAUSTIVE`` and ``ACCURACY`` flags — slower but more reliable
    on phone photos with mild motion blur or uneven lighting.
    """
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None

    height, width = image.shape[:2]

    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(
        image, board.pattern_size, flags=flags
    )
    if not found or corners is None:
        return None

    return ImageDetection(
        path=image_path,
        corners=corners.astype(np.float32),
        image_size=(width, height),
    )


def collect_detections(
    image_paths: Sequence[Path], board: CheckerboardSpec
) -> tuple[list[ImageDetection], list[Path]]:
    """Run corner detection on every image, returning kept and skipped lists."""
    kept: list[ImageDetection] = []
    skipped: list[Path] = []

    reference_size: tuple[int, int] | None = None

    for path in image_paths:
        detection = detect_corners(path, board)
        if detection is None:
            print(f"  [skip] {path.name}: corners not detected")
            skipped.append(path)
            continue

        if reference_size is None:
            reference_size = detection.image_size
        elif detection.image_size != reference_size:
            # Mixing resolutions would silently produce a wrong fx/fy
            # so we reject mismatches loudly.
            print(
                f"  [skip] {path.name}: image size {detection.image_size} "
                f"differs from reference {reference_size}"
            )
            skipped.append(path)
            continue

        kept.append(detection)

    return kept, skipped


def calibrate(
    detections: Sequence[ImageDetection], board: CheckerboardSpec
) -> CalibrationResult:
    """Run ``cv2.calibrateCamera`` with the rational distortion model."""
    if not detections:
        raise ValueError("calibrate() requires at least one detection")

    image_size = detections[0].image_size
    object_points_template = board.object_points()

    object_points: list[np.ndarray] = [
        object_points_template.copy() for _ in detections
    ]
    image_points: list[np.ndarray] = [d.corners for d in detections]

    flags = cv2.CALIB_RATIONAL_MODEL
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=flags,
    )

    per_image_rms = compute_per_image_rms(
        object_points, image_points, rvecs, tvecs, camera_matrix, dist_coeffs
    )

    return CalibrationResult(
        image_size=image_size,
        camera_matrix=np.asarray(camera_matrix),
        dist_coeffs=np.asarray(dist_coeffs).reshape(-1),
        rms=float(rms),
        per_image_rms=per_image_rms,
        accepted_paths=[d.path for d in detections],
    )


def compute_per_image_rms(
    object_points: Iterable[np.ndarray],
    image_points: Iterable[np.ndarray],
    rvecs: Sequence[np.ndarray],
    tvecs: Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> list[float]:
    """Reproject each image's points and return its RMS pixel error."""
    errors: list[float] = []
    for obj_pts, img_pts, rvec, tvec in zip(
        object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            obj_pts, rvec, tvec, camera_matrix, dist_coeffs
        )
        diff = projected.reshape(-1, 2) - img_pts.reshape(-1, 2)
        rms_px = float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))
        errors.append(rms_px)
    return errors


def build_output_payload(
    result: CalibrationResult,
    board: CheckerboardSpec,
) -> dict:
    """Convert a ``CalibrationResult`` into the on-disk JSON schema."""
    fx = float(result.camera_matrix[0, 0])
    fy = float(result.camera_matrix[1, 1])
    cx = float(result.camera_matrix[0, 2])
    cy = float(result.camera_matrix[1, 2])

    # Rational model: [k1, k2, p1, p2, k3, k4, k5, k6]. Pad with zeros
    # if OpenCV returns a shorter vector (older builds may return 5
    # values without CALIB_RATIONAL_MODEL fully active).
    coeffs = np.zeros(8, dtype=np.float64)
    src = result.dist_coeffs.reshape(-1)
    coeffs[: min(8, src.size)] = src[: min(8, src.size)]

    return {
        "image_size": [result.image_size[0], result.image_size[1]],
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "dist": {
            "k1": float(coeffs[0]),
            "k2": float(coeffs[1]),
            "p1": float(coeffs[2]),
            "p2": float(coeffs[3]),
            "k3": float(coeffs[4]),
            "k4": float(coeffs[5]),
            "k5": float(coeffs[6]),
            "k6": float(coeffs[7]),
        },
        "rms_reprojection_error_px": result.rms,
        "image_count": len(result.accepted_paths),
        "checkerboard": {
            "cols": board.cols,
            "rows": board.rows,
            "square_mm": board.square_mm,
        },
    }


def write_report(
    report_path: Path,
    result: CalibrationResult,
    skipped: Sequence[Path],
    board: CheckerboardSpec,
) -> None:
    """Write a human-readable per-image RMS report next to the JSON."""
    lines: list[str] = []
    lines.append("NavSight camera calibration report")
    lines.append("=" * 60)
    lines.append(
        f"Checkerboard: {board.cols}x{board.rows} inner corners, "
        f"{board.square_mm:.2f} mm squares"
    )
    lines.append(
        f"Image size:   {result.image_size[0]} x {result.image_size[1]} px"
    )
    lines.append(f"Accepted:     {len(result.accepted_paths)} image(s)")
    lines.append(f"Skipped:      {len(skipped)} image(s)")
    lines.append(f"Overall RMS:  {result.rms:.4f} px")
    lines.append("")
    lines.append("Per-image reprojection RMS (px):")
    pairs = sorted(
        zip(result.accepted_paths, result.per_image_rms),
        key=lambda kv: kv[1],
        reverse=True,
    )
    for path, rms_px in pairs:
        marker = " <-- worst" if rms_px == pairs[0][1] else ""
        lines.append(f"  {rms_px:7.4f}  {path.name}{marker}")

    if skipped:
        lines.append("")
        lines.append("Skipped images (corners not detected):")
        for path in skipped:
            lines.append(f"  {path.name}")

    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Calibrate a phone camera from a directory of checkerboard "
            "photos and emit camera_intrinsics.json for NavSight."
        ),
    )
    parser.add_argument(
        "image_dir",
        type=Path,
        help="Directory containing checkerboard photos (jpg/png/...).",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=DEFAULT_COLS,
        help=(
            f"Inner corner columns on the checkerboard "
            f"(default: {DEFAULT_COLS})."
        ),
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=DEFAULT_ROWS,
        help=(
            f"Inner corner rows on the checkerboard "
            f"(default: {DEFAULT_ROWS})."
        ),
    )
    parser.add_argument(
        "--square-mm",
        type=float,
        default=DEFAULT_SQUARE_MM,
        help=(
            f"Square edge length in millimetres at print scale "
            f"(default: {DEFAULT_SQUARE_MM})."
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(DEFAULT_OUTPUT),
        help=(
            f"Destination JSON path. A sibling .report.txt is also "
            f"written. (default: {DEFAULT_OUTPUT})"
        ),
    )
    return parser.parse_args(argv)


def print_next_steps(
    rms: float, image_count: int, output_path: Path
) -> None:
    """Print actionable advice based on the achieved RMS."""
    print("")
    if image_count < MIN_IMAGES_REQUIRED:
        print(
            f"  WARNING: only {image_count} images accepted "
            f"(>= {MIN_IMAGES_REQUIRED} recommended). Calibration is "
            f"under-constrained — capture more photos at varied angles."
        )
    if rms < RMS_GOOD_THRESHOLD_PX:
        verdict = f"RMS = {rms:.3f} px — good."
    elif rms < RMS_USABLE_THRESHOLD_PX:
        verdict = (
            f"RMS = {rms:.3f} px — usable but could be tighter. "
            f"Look at the report for the worst images and replace them."
        )
    else:
        verdict = (
            f"RMS = {rms:.3f} px — too high. Re-shoot the checkerboard "
            f"with sharper focus and more angle variety, then re-run."
        )
    print(f"  {verdict}")
    print(f"  Wrote: {output_path}")
    print(f"  Report: {output_path.with_suffix(output_path.suffix + '.report.txt')}")
    print(
        "  Next: Step 8b of docs/VISUAL_PRODUCTION_PLAN.md will load this "
        "JSON in LensCorrector and Tracker::initialize."
    )


def run(args: argparse.Namespace) -> int:
    """End-to-end calibration entry point. Returns process exit code."""
    image_dir: Path = args.image_dir
    if not image_dir.is_dir():
        print(f"error: not a directory: {image_dir}", file=sys.stderr)
        return 2

    board = CheckerboardSpec(
        cols=args.cols, rows=args.rows, square_mm=args.square_mm
    )

    image_paths = find_image_files(image_dir)
    if not image_paths:
        print(
            f"error: no supported image files in {image_dir} "
            f"(extensions: {', '.join(SUPPORTED_EXTENSIONS)})",
            file=sys.stderr,
        )
        return 2

    print(
        f"Detecting {board.cols}x{board.rows} corners "
        f"in {len(image_paths)} image(s)..."
    )
    detections, skipped = collect_detections(image_paths, board)
    print(
        f"Accepted {len(detections)} image(s), skipped {len(skipped)}."
    )

    if len(detections) < 4:
        print(
            "error: need at least 4 valid detections to calibrate "
            f"(got {len(detections)}).",
            file=sys.stderr,
        )
        return 1

    print("Running cv2.calibrateCamera (rational model)...")
    result = calibrate(detections, board)

    output_path: Path = args.output
    output_path.parent.mkdir(parents=True, exist_ok=True)

    payload = build_output_payload(result, board)
    output_path.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )

    report_path = output_path.with_suffix(output_path.suffix + ".report.txt")
    write_report(report_path, result, skipped, board)

    print_next_steps(result.rms, len(detections), output_path)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
