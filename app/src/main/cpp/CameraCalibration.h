#pragma once
#include <string>

// Step 1 (Visual Production Plan) — runtime loader for the in-app camera
// calibration JSON written by the Kotlin/Compose calibration screen.
//
// The schema (fixed, shallow) is:
//   {
//     "image_size": [w, h],
//     "fx": ..., "fy": ..., "cx": ..., "cy": ...,
//     "dist": {"k1":..., "k2":..., "p1":..., "p2":..., "k3":...,
//              "k4":0, "k5":0, "k6":0},
//     "rms_reprojection_error_px": ...,
//     "image_count": N,
//     "checkerboard": {"cols":9, "rows":6, "square_mm":25.0},
//     "captured_at": "<ISO 8601>",
//     "source": "in_app" | "offline_script"
//   }
//
// The runtime app does not link nlohmann/json (only the test/replay binaries
// do), so we hand-roll a tiny string-search parser for this fixed schema.
// Validation guards: file exists, all required fields present, RMS <= 1.0 px,
// image_size dims > 0, fx/fy > 0, cx/cy in [0, image_size]. On any failure
// the loader returns false and logs the reason.

struct CameraIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double k1{0.0};
    double k2{0.0};
    double k3{0.0};
    double k4{0.0};
    double k5{0.0};
    double k6{0.0};
    double p1{0.0};
    double p2{0.0};
    int image_w{0};
    int image_h{0};
    double rms_px{0.0};
};

// Returns true when the file parses, every required field is present, and
// every validation gate passes. On false, `out` is left untouched.
bool loadCalibrationFromJson(const std::string& path, CameraIntrinsics& out);
