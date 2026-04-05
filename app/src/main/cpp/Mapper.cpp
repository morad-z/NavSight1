#include "Mapper.h"
#include "Tracker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <android/log.h>
#include <cmath>
#include <algorithm>

#define TAG "NavSight-Mapper"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

// ── Constructor / Reset ──────────────────────────────────────────────────────

Mapper::Mapper() {
    ground_plane_estimate_ = {};
    LOGI("Mapper created");
}

void Mapper::reset() {
    keyframe_window_ = KeyframeWindow();
    bundle_adj_scale_ = 1.0;
    bundle_adj_iteration_ = 0;
    last_ba_timestamp_ns_ = 0;
    loop_closure_detector_.reset();
    frames_since_last_keyframe_ = 0;
    last_keyframe_position_ = {0, 0, 0};
    ground_plane_estimate_ = {};
    LOGI("Mapper reset");
}

// ── process ──────────────────────────────────────────────────────────────────

MapperResult Mapper::process(const TrackerFrame& frame, double current_smooth_scale) {
    MapperResult result{};
    result.ground_plane = {false, 0.0, 0.0};
    result.bundle_adj   = {false, 0.0, 0.0};
    result.loop_closure = {false, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Ground plane runs on every frame (needs only image + features, not pose).
    // Provides absolute scale from camera height even during fallback mode.
    result.ground_plane = detectGroundPlane(frame);

    // BA and loop closure need valid pose for meaningful optimization
    if (frame.pose_valid) {
        result.bundle_adj = runBundleAdjustment(frame, current_smooth_scale);
    }
    result.loop_closure = detectLoopClosure(frame);
    return result;
}

// ── Ground Plane Detection ───────────────────────────────────────────────────
// Uses Canny + HoughLinesP → vanishing point → camera pitch → ground distance.
// Provides absolute scale constraint from stable ground geometry.

GroundPlaneResult Mapper::detectGroundPlane(const TrackerFrame& frame) {
    GroundPlaneResult result{false, 0.0, 0.0};

    cv::Mat edges;
    cv::Canny(frame.gray, edges, 50, 150);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180.0, 40, 25, 10);

    // Adaptive horizon from point distribution
    double y_horizon = frame.gray.rows * 0.5;
    int below = 0, above = 0;
    for (const auto& pt : frame.next_good) {
        if (pt.y > y_horizon) below++; else above++;
    }
    if (below > above * 1.5)      y_horizon = frame.gray.rows * 0.35;
    else if (above > below * 1.5) y_horizon = frame.gray.rows * 0.65;

    // Filter horizontal lines near horizon
    std::vector<cv::Vec4i> ground_lines;
    for (const auto& line : lines) {
        double dx = line[2] - line[0];
        double dy = line[3] - line[1];
        if (dx < 10) continue;
        double angle = std::abs(std::atan2(dy, dx)) * 180.0 / M_PI;
        if (angle < 10.0 || angle > 170.0) {
            double y_center = (line[1] + line[3]) / 2.0;
            if (std::abs(y_center - y_horizon) < frame.gray.rows * 0.15)
                ground_lines.push_back(line);
        }
    }

    if (ground_lines.size() < 2) return result;

    // Vanishing point from line intersections (median, robust to outliers)
    std::vector<cv::Point2f> vp_candidates;
    size_t n = std::min(size_t(8), ground_lines.size());
    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            const auto& l1 = ground_lines[i];
            const auto& l2 = ground_lines[j];
            double dx1 = l1[2] - l1[0], dy1 = l1[3] - l1[1];
            double dx2 = l2[2] - l2[0], dy2 = l2[3] - l2[1];
            double denom = dx1 * dy2 - dx2 * dy1;
            if (std::abs(denom) < 1e-6) continue;
            double t = ((l2[0] - l1[0]) * dy2 - (l2[1] - l1[1]) * dx2) / denom;
            double ix = l1[0] + t * dx1, iy = l1[1] + t * dy1;
            if (ix > -frame.gray.cols && ix < 2 * frame.gray.cols &&
                iy >= y_horizon * 0.5 && iy <= frame.gray.rows * 1.5)
                vp_candidates.emplace_back(static_cast<float>(ix), static_cast<float>(iy));
        }
    }
    if (vp_candidates.empty()) return result;

    std::sort(vp_candidates.begin(), vp_candidates.end(),
        [](const auto& a, const auto& b) { return a.x < b.x; });
    float vp_x = vp_candidates[vp_candidates.size() / 2].x;
    std::sort(vp_candidates.begin(), vp_candidates.end(),
        [](const auto& a, const auto& b) { return a.y < b.y; });
    float vp_y = vp_candidates[vp_candidates.size() / 2].y;

    // Pitch estimation from vanishing point offset
    double focal = (frame.fx > 0 && frame.fy > 0) ? (frame.fx + frame.fy) / 2.0 : 375.0;
    double pitch = std::atan2(vp_y - y_horizon, focal);
    double abs_pitch = std::abs(pitch);
    if (abs_pitch <= 0.08) return result;

    double tan_pitch = std::tan(abs_pitch);
    if (tan_pitch <= 0.02) return result;

    double raw_distance = CAMERA_HEIGHT / tan_pitch;
    if (raw_distance <= 0.8 || raw_distance >= 8.0) return result;

    double conf = 0.6 * std::min(1.0, ground_lines.size() / 4.0)
                + 0.4 * std::min(1.0, vp_candidates.size() / 6.0);
    if (conf <= 0.4) return result;

    result.is_valid = true;
    result.ground_scale = raw_distance / CAMERA_HEIGHT;
    result.confidence = conf;

    ground_plane_estimate_ = {cv::Point2f(vp_x, vp_y), pitch, raw_distance, conf, true};

    if (frame.frame_counter % 90 == 0) {
        LOGI("GROUND_PLANE: vp=(%.0f,%.0f) pitch=%.2f° dist=%.2fm conf=%.2f",
             vp_x, vp_y, pitch * 180.0 / M_PI, raw_distance, conf);
    }
    return result;
}

// ── Bundle Adjustment (LM + Huber Loss) ──────────────────────────────────────
// Jointly optimizes scale and point depths via reprojection error minimization.
// Runs every BA_INTERVAL frames on a sliding window of keyframes.

BAResult Mapper::runBundleAdjustment(const TrackerFrame& frame,
                                      double current_smooth_scale) {
    BAResult result{false, 0.0, 0.0};

    if (frame.points_3d.empty() || frame.frame_counter % BA_INTERVAL != 0 ||
        frame.tracked < 15)
        return result;

    Keyframe kf;
    kf.frame_id = frame.frame_counter;
    kf.pose = cv::Mat::eye(4, 4, CV_64F);
    kf.keypoints = frame.next_good;
    kf.points_3d = frame.points_3d;
    kf.timestamp_ns = frame.timestamp_ns;
    keyframe_window_.add(kf);

    if (!keyframe_window_.has_minimum()) return result;

    // Sliding window BA: collect depth observations from ALL keyframes in window.
    // Newer keyframes weighted higher for temporal relevance.
    struct BAObs {
        cv::Point3f pt3d;
        cv::Point2f pt2d;
        double depth;
        double weight;
    };
    std::vector<BAObs> observations;
    observations.reserve(200);

    const auto& kf_data = keyframe_window_.data();
    for (size_t ki = 0; ki < kf_data.size(); ki++) {
        const auto& wkf = kf_data[ki];
        // Recency weight: oldest=0.3, newest=1.0
        double w = 0.3 + 0.7 * static_cast<double>(ki) / std::max(size_t(1), kf_data.size() - 1);
        size_t max_pts = std::min(wkf.keypoints.size(), wkf.points_3d.size());
        for (size_t i = 0; i < max_pts && observations.size() < 200; i++) {
            if (wkf.points_3d[i].z > 0.1 && wkf.points_3d[i].z < 20.0) {
                observations.push_back({wkf.points_3d[i], wkf.keypoints[i],
                                         wkf.points_3d[i].z, w});
            }
        }
    }
    if (observations.size() < 10) return result;

    // Extract initial depths for optimization
    std::vector<double> depths(observations.size());
    std::vector<double> depth_params(observations.size());
    for (size_t i = 0; i < observations.size(); i++) {
        depths[i] = observations[i].depth;
        depth_params[i] = observations[i].depth;
    }

    double scale = frame.estimated_scale;
    double fx = (frame.fx > 0) ? frame.fx : 375.0;
    double fy = (frame.fy > 0) ? frame.fy : 375.0;
    double cx = (frame.cx > 0) ? frame.cx : 320.0;
    double cy = (frame.cy > 0) ? frame.cy : 240.0;
    double lambda = 0.001, prev_cost = 1e9;

    for (int iter = 0; iter < BA_MAX_ITER; iter++) {
        double cost = 0.0;
        std::vector<double> js(observations.size(), 0), jd(observations.size(), 0);

        for (size_t i = 0; i < observations.size(); i++) {
            const auto& obs = observations[i];

            double zs = (obs.pt3d.z * scale) * (depth_params[i] / depths[i]);
            if (zs < 0.1) continue;

            double xp = fx * (obs.pt3d.x * scale) / zs + cx;
            double yp = fy * (obs.pt3d.y * scale) / zs + cy;
            double rx = xp - obs.pt2d.x, ry = yp - obs.pt2d.y;
            double rsq = rx * rx + ry * ry, rn = std::sqrt(rsq);

            // Huber loss weighted by observation recency
            double hw = (rn < HUBER_THRESHOLD) ? 1.0 : HUBER_THRESHOLD / (rn + 1e-6);
            cost += obs.weight * ((rn < HUBER_THRESHOLD) ? 0.5 * rsq
                  : HUBER_THRESHOLD * (rn - 0.5 * HUBER_THRESHOLD));

            js[i] = obs.weight * hw * 2.0 * (rx * fx * obs.pt3d.x / zs
                                             + ry * fy * obs.pt3d.y / zs);
            double dz = (obs.pt3d.z * scale) / depths[i];
            jd[i] = obs.weight * hw * -2.0 * (rx * fx * obs.pt3d.x * dz / (zs * zs)
                                + ry * fy * obs.pt3d.y * dz / (zs * zs));
        }

        if (std::abs(prev_cost - cost) < 0.01) break;

        double sg = 0;
        for (double j : js) sg += j;
        double a = 0.001 * (1.0 - lambda / (lambda + 1.0));
        scale -= a * sg;

        for (size_t i = 0; i < depth_params.size(); i++) {
            depth_params[i] -= (a * lambda / (lambda + 10.0)) * jd[i];
            depth_params[i] = std::max(0.2, std::min(3.0, depth_params[i]));
        }

        lambda *= (cost < prev_cost) ? 0.7 : 1.3;
        lambda = std::max(0.00001, std::min(10.0, lambda));
        prev_cost = cost;
    }

    scale = std::max(0.005, std::min(20.0, scale));

    // Temporal smoothness constraint
    double dt = (frame.timestamp_ns - last_ba_timestamp_ns_) / 1e9;
    last_ba_timestamp_ns_ = frame.timestamp_ns;
    if (dt > 0 && dt < 2.0) {
        double max_rate = (frame.quality > 0.7) ? 0.08 : 0.15;
        double max_change = max_rate * dt;
        double delta = std::abs(scale - current_smooth_scale);
        if (delta > max_change) {
            double sign = (scale > current_smooth_scale) ? 1.0 : -1.0;
            scale = current_smooth_scale + sign * max_change;
        }
    }

    bundle_adj_scale_ = scale;
    bundle_adj_iteration_++;

    if (bundle_adj_iteration_ % 15 == 0) {
        LOGI("BA: scale=%.4f obs=%zu kfs=%zu cost=%.2f",
             scale, observations.size(), kf_data.size(), prev_cost);
    }

    result.is_valid = true;
    result.optimized_scale = scale;
    result.alpha = 0.08;
    return result;
}

// ── Loop Closure Detection ───────────────────────────────────────────────────
// ORB descriptor matching against stored keyframes.
// Returns blended correction (30% per detection) to avoid teleportation.

LoopClosureResult Mapper::detectLoopClosure(const TrackerFrame& frame) {
    LoopClosureResult result{false, 0, 0, 0, 0, 0};

    cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8);
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kps;
    orb->detectAndCompute(frame.gray, cv::noArray(), kps, descriptors);

    std::vector<cv::Point2f> keypoints;
    keypoints.reserve(kps.size());
    for (const auto& kp : kps) keypoints.push_back(kp.pt);

    cv::Point3f pos = frame.global_position;

    // Check for loop closure against stored keyframes
    LoopClosureKeyframe matched_kf;
    if (loop_closure_detector_.detectLoopClosure(
            descriptors, keypoints, pos,
            static_cast<float>(frame.heading), matched_kf)) {
        result.detected = true;
        result.target_x = matched_kf.position.x;
        result.target_y = matched_kf.position.y;
        result.target_z = matched_kf.position.z;
        result.target_heading = matched_kf.heading;
        result.blend = 0.30;

        LOGI("LOOP_CLOSURE: matched at (%.1f, %.1f) heading=%.1f°",
             matched_kf.position.x, matched_kf.position.z,
             matched_kf.heading * 180.0 / M_PI);
    }

    // Store new keyframe every 0.3m of motion or every 10 frames
    frames_since_last_keyframe_++;
    float dist = std::sqrt(
        std::pow(pos.x - last_keyframe_position_.x, 2) +
        std::pow(pos.z - last_keyframe_position_.z, 2));

    if (frames_since_last_keyframe_ > 10 || dist > 0.3f) {
        LoopClosureKeyframe new_kf;
        new_kf.timestamp_ns = frame.timestamp_ns;
        new_kf.position = pos;
        new_kf.heading = static_cast<float>(frame.heading);
        new_kf.descriptors = descriptors.clone();
        new_kf.keypoints = keypoints;
        loop_closure_detector_.addKeyframe(new_kf);
        frames_since_last_keyframe_ = 0;
        last_keyframe_position_ = pos;
    }

    return result;
}
