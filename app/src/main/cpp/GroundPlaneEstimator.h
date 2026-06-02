#pragma once

#include <opencv2/core.hpp>
#include <vector>

// GroundPlaneEstimator (2026-06-02) — focused, LIVE port of the ground-plane metric-scale math that
// was buried (and disabled) in Mapper.cpp. Recovers the monocular scale that VIO lacks by fitting a
// plane to the lower-image triangulated points and dividing the KNOWN camera-to-ground height by the
// plane's VIO-scale height: ground_scale = camera_height_m / h_vio. On a scooter the phone is mounted
// at a roughly fixed height/attitude, so this geometry is observable even at constant cruise — exactly
// where accel-K is blind. It ALSO returns the per-point ground inlier mask + the ground vanishing line
// (horizon row) for the camera debug overlay. Pure (OpenCV core only), seeded RNG → deterministic,
// READ-ONLY (the caller logs/draws it; it never feeds the dot until validated). See gpt_speed_suggestion.md.
struct GroundPlaneResult {
    bool is_valid = false;
    double ground_scale = 0.0;          // camera_height_m / h_vio  (multiply VIO displacement by this → metres)
    double confidence = 0.0;            // inliers / candidates
    double h_vio = 0.0;                 // camera-to-plane distance in VIO (un-metric) units
    int n_candidates = 0;
    int n_inliers = 0;
    cv::Vec3d normal_cam{0.0, 0.0, 0.0};// unit plane normal in (prev) camera frame, oriented "up" (n.y<0)
    double horizon_v_px = -1.0;         // image row of the ground vanishing line (−1 when unavailable)
    std::vector<unsigned char> inlier_mask;  // parallel to the input points: 1 = ground-plane inlier
};

class GroundPlaneEstimator {
public:
    // points2d (pixels, current frame) and points3d (triangulated, PREV camera frame, VIO scale) are
    // PARALLEL arrays. camera_height_m is the measured camera-to-road distance (scooter mount ≈ 1.0–1.1 m).
    // analyzer_rotation_deg = CameraX rotationDegrees (0/90/180/270): the rotation applied to display
    // the sensor frame upright. The road is at the BOTTOM of the upright (portrait) view, which maps to
    // a SIDE of the sensor frame on a portrait mount — so the candidate region must track it.
    GroundPlaneResult estimate(const std::vector<cv::Point2f>& points2d,
                               const std::vector<cv::Point3f>& points3d,
                               int img_width, int img_height,
                               double fx, double fy, double cx, double cy,
                               double camera_height_m,
                               int analyzer_rotation_deg = 0);

private:
    // Deterministic LCG so offline replay runs are reproducible (Mapper used unseeded rand()).
    unsigned int rng_state_ = 0x9E3779B9u;
    double frand();   // [0,1)

    static constexpr double kLowerImageFrac   = 0.55;  // candidates must sit below this image fraction
    static constexpr int    kRansacIters      = 80;
    static constexpr double kInlierFracOfDepth= 0.04;  // plane-distance inlier band = 4% of median depth
    static constexpr int    kMinCandidates    = 12;
    static constexpr int    kMinInliers       = 8;
    static constexpr double kMinInlierRatio   = 0.40;
    static constexpr double kMinVerticalNormal= 0.55;  // |n.y| must dominate → roughly horizontal plane
};
