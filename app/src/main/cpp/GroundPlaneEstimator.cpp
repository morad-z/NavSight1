#include "GroundPlaneEstimator.h"

#include <algorithm>
#include <cmath>

double GroundPlaneEstimator::frand() {
    // Numerical Recipes LCG — deterministic, good enough for RANSAC sampling.
    rng_state_ = rng_state_ * 1664525u + 1013904223u;
    return static_cast<double>(rng_state_) / 4294967296.0;
}

GroundPlaneResult GroundPlaneEstimator::estimate(
        const std::vector<cv::Point2f>& points2d,
        const std::vector<cv::Point3f>& points3d,
        int img_width, int img_height,
        double fx, double fy, double cx, double cy,
        double camera_height_m,
        int analyzer_rotation_deg) {
    GroundPlaneResult r;
    const size_t n = std::min(points2d.size(), points3d.size());
    r.inlier_mask.assign(n, 0);
    if (n < static_cast<size_t>(kMinCandidates) || camera_height_m <= 0.0 || img_height <= 0) {
        return r;
    }

    // ── 1. Candidate ground points: lower image region + a finite, in-front VIO depth. The road
    // surface projects into the lower part of the frame; points3d.z is depth along the prev-cam optical
    // axis in VIO units (positive = in front). We keep all positive finite depths (the scale is what we
    // are solving for, so we must NOT assume a metric depth band here). ────────────────────────────────
    // Road region = the analyzer sub-image that maps to the BOTTOM of the upright (portrait) display.
    // For a portrait mount the sensor frame is rotated, so the road is on a SIDE in analyzer coords.
    const int rot = ((analyzer_rotation_deg % 360) + 360) % 360;
    const double frac = kLowerImageFrac;
    auto inRoadRegion = [&](float px, float py) -> bool {
        switch (rot) {
            case 90:  return px > frac * img_width;             // road → right of sensor
            case 180: return py < (1.0 - frac) * img_height;    // road → top of sensor
            case 270: return px < (1.0 - frac) * img_width;     // road → left of sensor
            default:  return py > frac * img_height;            // 0 → road at sensor bottom
        }
    };
    std::vector<int> cand;
    cand.reserve(n);
    std::vector<double> cand_depth;
    for (size_t i = 0; i < n; ++i) {
        const float z = points3d[i].z;
        if (!inRoadRegion(points2d[i].x, points2d[i].y)) continue;
        if (!std::isfinite(z) || z <= 1e-4f) continue;
        if (!std::isfinite(points3d[i].x) || !std::isfinite(points3d[i].y)) continue;
        cand.push_back(static_cast<int>(i));
        cand_depth.push_back(std::abs(static_cast<double>(z)));
    }
    r.n_candidates = static_cast<int>(cand.size());
    if (cand.size() < static_cast<size_t>(kMinCandidates)) return r;

    // Adaptive inlier band: a fraction of the median candidate depth, so it tracks the (unknown) VIO
    // scale instead of a fixed metric threshold that would be wrong by the VIO-scale factor.
    std::vector<double> sorted_depth = cand_depth;
    std::nth_element(sorted_depth.begin(), sorted_depth.begin() + sorted_depth.size() / 2, sorted_depth.end());
    const double median_depth = sorted_depth[sorted_depth.size() / 2];
    const double inlier_thr = std::max(1e-4, kInlierFracOfDepth * median_depth);

    // ── 2. RANSAC plane fit over the candidates. ─────────────────────────────────────────────────────
    cv::Vec3d best_n(0, 0, 0);
    double best_d = 0.0;
    int best_inliers = 0;
    const int m = static_cast<int>(cand.size());
    for (int it = 0; it < kRansacIters; ++it) {
        int ia = cand[static_cast<int>(frand() * m) % m];
        int ib = cand[static_cast<int>(frand() * m) % m];
        int ic = cand[static_cast<int>(frand() * m) % m];
        if (ia == ib || ib == ic || ia == ic) continue;
        const cv::Vec3d pa(points3d[ia].x, points3d[ia].y, points3d[ia].z);
        const cv::Vec3d pb(points3d[ib].x, points3d[ib].y, points3d[ib].z);
        const cv::Vec3d pc(points3d[ic].x, points3d[ic].y, points3d[ic].z);
        cv::Vec3d nrm = (pb - pa).cross(pc - pa);
        const double nn = cv::norm(nrm);
        if (nn < 1e-9) continue;
        nrm /= nn;
        const double d = -nrm.dot(pa);
        int inliers = 0;
        for (int k = 0; k < m; ++k) {
            const int idx = cand[k];
            const cv::Vec3d p(points3d[idx].x, points3d[idx].y, points3d[idx].z);
            if (std::abs(nrm.dot(p) + d) < inlier_thr) inliers++;
        }
        if (inliers > best_inliers) { best_inliers = inliers; best_n = nrm; best_d = d; }
    }

    r.n_inliers = best_inliers;
    if (best_inliers < kMinInliers ||
        static_cast<double>(best_inliers) < kMinInlierRatio * cand.size()) {
        return r;
    }

    // ── 3. Orient the normal "up" (camera y is image-DOWN, so up = n.y < 0) and gate on it being
    // roughly vertical — rejects walls / car sides whose normal is horizontal (the missing gravity
    // check when there is no EKF). ─────────────────────────────────────────────────────────────────
    if (best_n[1] > 0.0) { best_n = -best_n; best_d = -best_d; }
    if (std::abs(best_n[1]) < kMinVerticalNormal) return r;

    // h_vio = camera-to-plane distance in VIO units = |d| (n is unit). Scale = known metric height / h_vio.
    const double h_vio = std::abs(best_d);
    if (h_vio < 1e-6) return r;
    const double ground_scale = camera_height_m / h_vio;
    if (!std::isfinite(ground_scale) || ground_scale <= 0.0) return r;

    // ── 4. Fill the result + the per-point inlier mask (for the overlay). ──────────────────────────
    for (int k = 0; k < m; ++k) {
        const int idx = cand[k];
        const cv::Vec3d p(points3d[idx].x, points3d[idx].y, points3d[idx].z);
        if (std::abs(best_n.dot(p) + best_d) < inlier_thr) r.inlier_mask[idx] = 1;
    }

    // Ground vanishing line (horizon) image row at u=cx: the plane's vanishing line is l = K^{-T}·n;
    // the row where it crosses the image centre column is v = -(l0·cx + l2)/l1. Used by the overlay.
    const double l0 = best_n[0] / fx;
    const double l1 = best_n[1] / fy;
    const double l2 = best_n[2] - (cx * best_n[0] / fx) - (cy * best_n[1] / fy);
    if (std::abs(l1) > 1e-9) {
        const double v = -(l0 * cx + l2) / l1;
        if (std::isfinite(v) && v > -img_height && v < 2.0 * img_height) r.horizon_v_px = v;
    }

    r.is_valid = true;
    r.ground_scale = ground_scale;
    r.h_vio = h_vio;
    r.normal_cam = best_n;
    r.confidence = static_cast<double>(best_inliers) / static_cast<double>(cand.size());
    return r;
}
