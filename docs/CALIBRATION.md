# Camera Intrinsics Calibration (Step 1)

This is the **one-time, per-phone** procedure that produces the
`camera_intrinsics.json` file consumed by NavSight's visual frontend.
Without it, `LensCorrector` runs as a zero-distortion passthrough and
`cv::recoverPose` operates on lens-distorted rays — the dominant
non-IMU bias source on phone wide-angle cameras (see Step 1 of
`docs/VISUAL_PRODUCTION_PLAN.md`).

You only need to run this once per device. Re-run it if you swap
phones, or if the user reports the camera was repaired/replaced.

---

## 1. Print the checkerboard

Use the standard OpenCV checkerboard:

- Pattern: **9×6 inner corners** (i.e. a printed grid of 10×7 squares).
- Size: roughly **A4** at 100% scale, which gives **25 mm squares**.
- Source: OpenCV's `pattern.png`
  (https://github.com/opencv/opencv/blob/4.x/doc/pattern.png).

After printing, **measure one square with a ruler**. If your printer
scaled it (common on "fit to page"), pass the actual measured value to
`--square-mm` instead of the default `25.0`. The intrinsics fit is
unaffected by the absolute square size, but the pose translation units
in the report depend on it being correct.

**Mounting matters:** tape the print to a flat, rigid surface (a
clipboard, hardback book, or piece of foamboard). A curled or wavy
print will silently inflate reprojection error.

---

## 2. Take the photos

Use the **same phone** that will run NavSight. Prefer the default
back camera at the **same resolution** the app uses for VIO frames
(currently the rear camera in landscape).

Aim for **30–50 photos** with varied:

- **Angles**: tilt forward, back, left, right (board fills different
  parts of the frame each time). Include some shots where the board
  is in a corner, not just centred.
- **Distances**: 0.3 m up to ~1.5 m. Closer shots constrain `fx`/`fy`;
  farther shots constrain principal point and distortion tail.
- **Lighting**: half indoor, half outdoor. A few in shade, a few in
  direct sunlight. Avoid harsh glare on the print.
- **Orientation**: hold the phone in **landscape** (NavSight runs
  landscape). Do not rotate the phone between shots — that would
  reproject corners onto an inconsistent image plane.

Keep the **board fully visible** in every frame. If a corner is
clipped, the image is silently skipped.

Hold steady — motion blur does not strictly fail
`findChessboardCornersSB` but it raises per-image reprojection error.
A 1/60 s shutter or faster is usually fine handheld.

---

## 3. Where to put the photos

```
tests/calibration_images/<session>/
    IMG_0001.jpg
    IMG_0002.jpg
    ...
```

`<session>` is any folder name you like (e.g. the date or the phone
model). The folder is **user data** and is not committed to git.

---

## 4. Run the script

From the repo root:

```bash
python scripts/calibrate_camera.py tests/calibration_images/<session>
```

Optional flags (defaults match the print procedure above):

| Flag | Default | Use when |
| --- | --- | --- |
| `--cols 9` | 9 inner corners across | Different checkerboard print. |
| `--rows 6` | 6 inner corners down | Different checkerboard print. |
| `--square-mm 25.0` | 25 mm | Your printer scaled the page. |
| `--output <path>` | `app/src/main/assets/camera_intrinsics.json` | Saving to a different location. |

Two files are written:

- `app/src/main/assets/camera_intrinsics.json` — the app-ingestible
  schema (see "Output" below).
- `app/src/main/assets/camera_intrinsics.json.report.txt` — per-image
  reprojection RMS, sorted worst-first, plus the list of skipped
  images.

The script needs only `cv2` (`opencv-python`) and `numpy`. No new
project dependency is added.

---

## 5. Verify the result

Open the report file. The script also prints a short verdict:

| Overall RMS | Verdict | Action |
| --- | --- | --- |
| **< 0.5 px** | Good. | Done. Commit the JSON. |
| 0.5 – 1.0 px | Usable but loose. | Look at the worst images in the report and re-shoot those perspectives. |
| **> 1.0 px** | Bad fit. | Re-shoot from scratch — likely cause: blurry photos, curled print, or measured `--square-mm` is wrong. |

Also verify:

- **`image_count` ≥ 20** in the JSON. Below 10 the calibration is
  under-constrained regardless of RMS and the script will warn.
- **`image_size`** matches the resolution your phone actually shoots
  in landscape. If the phone produced portrait files because the IMU
  flipped during capture, EXIF rotation can lie — re-shoot in true
  landscape orientation.
- **`fx ≈ fy`** to within a few percent. Phones have square pixels;
  large differences indicate a bad fit.
- **`cx ≈ width/2`** and **`cy ≈ height/2`** within ~5%. Big offsets
  suggest the lens is not centered on the sensor (possible) or the
  fit is bad (more likely).
- **`k1` is negative and small** (typically −0.3 to 0.0). Large
  positive values are suspicious.

If any check fails, capture more photos covering the missing
perspectives and re-run.

---

## 6. Output schema

```json
{
  "image_size": [3840, 2160],
  "fx": 2890.1, "fy": 2891.7,
  "cx": 1923.4, "cy": 1077.2,
  "dist": {
    "k1": -0.215, "k2":  0.061,
    "p1":  0.0003, "p2": -0.0001,
    "k3": -0.012,
    "k4":  0.0,    "k5":  0.0,   "k6":  0.0
  },
  "rms_reprojection_error_px": 0.42,
  "image_count": 38,
  "checkerboard": {"cols": 9, "rows": 6, "square_mm": 25.0}
}
```

Coefficient order matches OpenCV's rational model
(`CALIB_RATIONAL_MODEL`): `[k1, k2, p1, p2, k3, k4, k5, k6]`. The
last three (`k4..k6`) are usually near zero on phone cameras and may
be omitted by older OpenCV builds — the script pads with zeros if so.

---

## 7. Wiring it into the app (Step 8b)

**Not done as part of this task.** Step 8b of
`docs/VISUAL_PRODUCTION_PLAN.md` will:

1. Load `app/src/main/assets/camera_intrinsics.json` in `MainActivity`
   on startup.
2. Push intrinsics into `Tracker::setCameraIntrinsics(fx, fy, cx, cy)`.
3. Push distortion into `LensCorrector::setDistortion(k1..k6, p1, p2)`,
   re-enabling the currently commented-out path.
4. Refuse to use the file if `rms_reprojection_error_px > 0.6` (the
   runtime gate from Step 1 of the production plan), falling back to
   the zero-distortion default with a warning.

Until that wiring lands, the JSON sits in `assets/` unused but ready.
Re-running this script with new photos overwrites the file in place.
