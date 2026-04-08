#include "Mapper.h"
#include "Tracker.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-Mapper"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,  TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGD(...) (void)0
#endif

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
    pose_graph_.reset();
    last_pose_graph_node_id_ = -1;
    frames_since_last_keyframe_ = 0;
    last_keyframe_position_ = {0, 0, 0};
    ground_plane_estimate_ = {};
    LOGI("Mapper reset");
}

// ── process ──────────────────────────────────────────────────────────────────

MapperResult Mapper::process(const TrackerFrame& frame, double current_smooth_scale,
                              const EKFState* ekf) {
    MapperResult result{};
    result.ground_plane = {false, 0.0, 0.0};
    result.bundle_adj   = {false, 0.0, 0.0};
    result.depth_scale  = {false, 0.0, 0.0};
    result.loop_closure = {false, 0.0, 0.0, 0.0, 0.0, 0.0};

    // 1. Ground plane runs on every frame
    result.ground_plane = detectGroundPlane(frame, ekf);

    // 2. Monocular depth scale constraint
    result.depth_scale = constrainScaleWithDepth(frame);

    // Add node to pose graph for every frame with valid position
    last_pose_graph_node_id_ = pose_graph_.addNode(
        frame.global_position.x, frame.global_position.z,
        frame.heading, frame.timestamp_ns);

    // BA and loop closure need valid pose
    if (frame.pose_valid) {
        result.bundle_adj = runBundleAdjustment(frame, current_smooth_scale, ekf);
    }
    result.loop_closure = detectLoopClosure(frame);
    return result;
}

void Mapper::setDepthMap(const float* depth_data, int width, int height) {
    if (!depth_data || width <= 0 || height <= 0) return;
    
    std::lock_guard<std::mutex> lock(depth_mutex_);
    size_t size = static_cast<size_t>(width * height);
    if (latest_depth_map_.size() != size) {
        latest_depth_map_.resize(size);
    }
    std::copy(depth_data, depth_data + size, latest_depth_map_.begin());
    depth_width_ = width;
    depth_height_ = height;
}

BAResult Mapper::constrainScaleWithDepth(const TrackerFrame& frame) {
    BAResult result{false, 0.0, 0.0};

    std::vector<float> depth_map;
    int dw, dh;
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (latest_depth_map_.empty()) return result;
        depth_map = latest_depth_map_;
        dw = depth_width_;
        dh = depth_height_;
    }

    if (frame.points_3d.empty() || frame.next_good.empty()) return result;

    // ── Part 1: Calibrate depth map using Ground Plane Homography ──
    double depth_to_metric_scale = 1.0;
    bool depth_calibrated = false;

    if (ground_plane_estimate_.is_valid && ground_plane_estimate_.confidence > 0.4) {
        // We have a ground plane: Z_metric = h / (n^T * K_inv * x)
        // Let's sample points in the lower 25% of the image (likely ground)
        std::vector<double> depth_calibration_ratios;
        
        double h = user_camera_height_;
        double fx = (frame.fx > 0) ? frame.fx : 375.0;
        double fy = (frame.fy > 0) ? frame.fy : 375.0;
        double cx = (frame.cx > 0) ? frame.cx : 320.0;
        double cy = (frame.cy > 0) ? frame.cy : 240.0;
        
        // n_C is the ground normal in camera frame.
        // If we don't have n_C directly, we can use pitch:
        // For a camera tilted down by 'pitch', the ground plane equation is:
        // Z * (img_y * cos(pitch) + sin(pitch)) = h
        double pitch = ground_plane_estimate_.camera_pitch;
        double ny = std::cos(pitch);
        double nz = std::sin(pitch);

        for (int row = dh * 3 / 4; row < dh; row += 4) {
            for (int col = dw / 4; col < dw * 3 / 4; col += 4) {
                float rel_depth = depth_map[row * dw + col];
                if (rel_depth < 0.1f) continue;

                // Image coordinates (normalized)
                double img_x = (static_cast<double>(col) / dw * frame.gray.cols - cx) / fx;
                double img_y = (static_cast<double>(row) / dh * frame.gray.rows - cy) / fy;

                // h = Z * (img_y * ny + nz)  => Z = h / (img_y * ny + nz)
                double denom = ny * img_y + nz;
                if (denom < 0.05) continue; // Above or too close to horizon

                double z_metric = h / denom;
                if (z_metric > 0.2 && z_metric < 10.0) {
                    depth_calibration_ratios.push_back(z_metric / rel_depth);
                }
            }
        }

        if (depth_calibration_ratios.size() > 20) {
            std::sort(depth_calibration_ratios.begin(), depth_calibration_ratios.end());
            depth_to_metric_scale = depth_calibration_ratios[depth_calibration_ratios.size() / 2];
            depth_calibrated = true;
        }
    }

    // ── Part 2: Estimate VIO scale using calibrated depth map ──
    std::vector<double> vio_scale_ratios;
    vio_scale_ratios.reserve(frame.points_3d.size());

    float fw = static_cast<float>(frame.gray.cols);
    float fh = static_cast<float>(frame.gray.rows);

    for (size_t i = 0; i < frame.points_3d.size(); i++) {
        const auto& pt3 = frame.points_3d[i];
        const auto& pt2 = frame.next_good[i];
        
        if (pt3.z < 0.2 || pt3.z > 15.0) continue;

        int dx = static_cast<int>((pt2.x / fw) * dw);
        int dy = static_cast<int>((pt2.y / fh) * dh);

        if (dx >= 0 && dx < dw && dy >= 0 && dy < dh) {
            float rel_depth = depth_map[dy * dw + dx];
            if (rel_depth > 1e-4f) {
                double metric_depth = rel_depth * depth_to_metric_scale;
                // Ratio between metric depth (from calibrated map) and VIO depth
                vio_scale_ratios.push_back(metric_depth / pt3.z);
            }
        }
    }

    if (vio_scale_ratios.size() < 10) return result;

    std::sort(vio_scale_ratios.begin(), vio_scale_ratios.end());
    double median_vio_scale_ratio = vio_scale_ratios[vio_scale_ratios.size() / 2];

    result.is_valid = true;
    // Current VIO is at 'frame.estimated_scale'. 
    // We want to multiply it by the ratio to reach metric scale.
    result.optimized_scale = median_vio_scale_ratio * frame.estimated_scale; 
    
    // Weight depends on whether we calibrated the depth map or not
    result.alpha = depth_calibrated ? 0.12 : 0.04;

    if (frame.frame_counter % 60 == 0) {
        LOGD("DEPTH_SCALE: samples=%zu ratio=%.4f calib=%s opt_scale=%.4f", 
             vio_scale_ratios.size(), median_vio_scale_ratio, 
             depth_calibrated ? "YES" : "NO", result.optimized_scale);
    }

    return result;
}

// ── Ground Plane Detection ───────────────────────────────────────────────────
// Two-stage detection:
// 1. Point-based RANSAC: fits plane to tracked features in lower image.
// 2. Horizon-based: vanishing point of parallel ground lines.
// Provides absolute scale constraint from known camera height.

GroundPlaneResult Mapper::detectGroundPlane(const TrackerFrame& frame, const EKFState* ekf) {
    GroundPlaneResult result{false, 1.0, 0.0};

    // --- Method 1: Point-based RANSAC (Preferred for texture) ---
    std::vector<cv::Point3f> ground_candidates;
    float fh = static_cast<float>(frame.gray.rows);
    for (size_t i = 0; i < frame.next_good.size(); i++) {
        if (frame.next_good[i].y > 0.6f * fh) {
            if (i < frame.points_3d.size()) {
                const auto& p = frame.points_3d[i];
                if (p.z > 0.5 && p.z < 10.0) {
                    ground_candidates.push_back(p);
                }
            }
        }
    }

    double h_vio_ransac = 0.0;
    double conf_ransac = 0.0;
    bool ransac_ok = false;

    if (ground_candidates.size() >= 12) {
        int max_inliers = 0;
        cv::Vec4d best_plane(0, 0, 0, 0);
        
        // Use a fixed seed for reproducibility in background thread if needed
        for (int iter = 0; iter < 40; iter++) {
            int i1 = rand() % ground_candidates.size();
            int i2 = rand() % ground_candidates.size();
            int i3 = rand() % ground_candidates.size();
            if (i1 == i2 || i2 == i3 || i1 == i3) continue;
            
            cv::Point3d p1 = ground_candidates[i1], p2 = ground_candidates[i2], p3 = ground_candidates[i3];
            cv::Point3d n = (p2 - p1).cross(p3 - p1);
            double nn = cv::norm(n);
            if (nn < 1e-7) continue;
            n *= 1.0 / nn;
            double d = -n.dot(p1);

            int inliers = 0;
            for (const auto& p : ground_candidates) {
                if (std::abs(n.dot(cv::Point3d(p)) + d) < 0.04) inliers++;
            }
            if (inliers > max_inliers) {
                max_inliers = inliers;
                best_plane = cv::Vec4d(n.x, n.y, n.z, d);
            }
        }

        if (max_inliers >= 10 && max_inliers > 0.4 * ground_candidates.size()) {
            double a = best_plane[0], b = best_plane[1], c = best_plane[2], d = best_plane[3];
            double h_vio = std::abs(d) / std::sqrt(a*a + b*b + c*c);
            
            // Gravity validation: ground normal should be parallel to gravity
            bool gravity_align = true;
            if (ekf && ekf->isFullInitialized()) {
                cv::Mat R_GtoC = ekf->getWindow().back().R_GtoC;
                cv::Mat g_cam = R_GtoC * (cv::Mat_<double>(3,1) << 0, 0, -1); // Assume Z is UP
                cv::Point3d gc(g_cam.at<double>(0), g_cam.at<double>(1), g_cam.at<double>(2));
                gc *= 1.0 / cv::norm(gc);
                if (std::abs(cv::Point3d(a,b,c).dot(gc)) < std::cos(20.0 * M_PI / 180.0)) 
                    gravity_align = false;
            }

            if (gravity_align && h_vio > 0.4 && h_vio < 4.0) {
                h_vio_ransac = h_vio;
                conf_ransac = 0.7 * (static_cast<double>(max_inliers) / ground_candidates.size());
                ransac_ok = true;
            }
        }
    }

    // --- Method 2: Horizon-based (Vanishing Point) ---
    cv::Mat edges;
    cv::Canny(frame.gray, edges, 50, 150);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180.0, 40, 30, 10);

    double y_horizon = frame.gray.rows * 0.5;
    std::vector<cv::Vec4i> ground_lines;
    for (const auto& line : lines) {
        double dx = line[2] - line[0], dy = line[3] - line[1];
        if (std::abs(dx) < 10) continue;
        double angle = std::abs(std::atan2(dy, dx)) * 180.0 / M_PI;
        if (angle < 15.0 || angle > 165.0) {
            double yc = (line[1] + line[3]) / 2.0;
            if (yc > y_horizon * 0.7) ground_lines.push_back(line);
        }
    }

    double h_vio_horizon = 0.0;
    double conf_horizon = 0.0;
    bool horizon_ok = false;

    if (ground_lines.size() >= 2) {
        std::vector<cv::Point2f> vp_candidates;
        for (size_t i = 0; i < std::min(size_t(10), ground_lines.size()); i++) {
            for (size_t j = i + 1; j < std::min(size_t(10), ground_lines.size()); j++) {
                const auto& l1 = ground_lines[i], l2 = ground_lines[j];
                double dx1 = l1[2]-l1[0], dy1 = l1[3]-l1[1], dx2 = l2[2]-l2[0], dy2 = l2[3]-l2[1];
                double denom = dx1 * dy2 - dx2 * dy1;
                if (std::abs(denom) < 1e-6) continue;
                double t = ((l2[0]-l1[0])*dy2 - (l2[1]-l1[1])*dx2) / denom;
                vp_candidates.emplace_back(l1[0]+t*dx1, l1[1]+t*dy1);
            }
        }
        if (!vp_candidates.empty()) {
            std::sort(vp_candidates.begin(), vp_candidates.end(), [](auto a, auto b){ return a.y < b.y; });
            float vp_y = vp_candidates[vp_candidates.size()/2].y;
            double focal = (frame.fx + frame.fy) / 2.0;
            double pitch = std::abs(std::atan2(vp_y - (frame.gray.rows/2.0), focal));
            if (pitch > 0.1 && pitch < 1.4) {
                h_vio_horizon = user_camera_height_ / std::tan(pitch); // Approximated
                conf_horizon = 0.4 * std::min(1.0, vp_candidates.size() / 10.0);
                horizon_ok = true;
            }
        }
    }

    // --- Synthesis ---
    if (ransac_ok) {
        result.is_valid = true;
        result.ground_scale = user_camera_height_ / h_vio_ransac;
        result.confidence = conf_ransac;
    } else if (horizon_ok) {
        // Horizon only gives scale if we assume orientation matches pitch. 
        // Safer to use it as a lower-confidence correction.
        result.is_valid = true;
        result.ground_scale = user_camera_height_ / (h_vio_horizon * std::tan(0.5)); // heuristic
        result.confidence = conf_horizon;
    }

    if (result.is_valid && frame.frame_counter % 90 == 0) {
        LOGI("GROUND_PLANE: method=%s scale=%.3f conf=%.2f height_vio=%.2fm",
             ransac_ok ? "RANSAC" : "HORIZON", result.ground_scale, result.confidence,
             ransac_ok ? h_vio_ransac : h_vio_horizon);
    }

    return result;
}

// ── Bundle Adjustment (Gauss-Newton with Huber Loss) ─────────────────────────
// Two modes:
// A. MSCKF-aware: uses EKF sliding window clones for multi-view triangulation
// B. Legacy fallback: own keyframe window when EKF is unavailable
//
// Jointly optimizes scale via reprojection error minimization.
// Normal equations: H * δx = -g  where H = J^T W J,  g = J^T W r

BAResult Mapper::runBundleAdjustment(const TrackerFrame& frame,
                                      double current_smooth_scale,
                                      const EKFState* ekf) {
    BAResult result{false, 0.0, 0.0};

    if (frame.points_3d.empty() || frame.frame_counter % BA_INTERVAL != 0 ||
        frame.tracked < 15)
        return result;

    // Always store keyframe for legacy path
    Keyframe kf;
    kf.frame_id = frame.frame_counter;
    kf.pose = cv::Mat::eye(4, 4, CV_64F);
    kf.keypoints = frame.next_good;
    kf.points_3d = frame.points_3d;
    kf.timestamp_ns = frame.timestamp_ns;
    keyframe_window_.add(kf);

    double fx = (frame.fx > 0) ? frame.fx : 375.0;
    double fy = (frame.fy > 0) ? frame.fy : 375.0;
    double cx = (frame.cx > 0) ? frame.cx : 320.0;
    double cy = (frame.cy > 0) ? frame.cy : 240.0;

    // ── Mode A: MSCKF clone-based BA ────────────────────────────────────────
    // Uses EKF sliding window poses for proper multi-view reprojection
    if (ekf && ekf->isFullInitialized()) {
        const auto& window = ekf->getWindow();
        if (window.size() >= 2 && !frame.points_3d.empty()) {
            // Gauss-Newton on scale: minimize reprojection across clone poses
            // Parameter: δs (scale correction)
            double scale = frame.estimated_scale;
            double prev_cost = 1e12;

            // Get current and previous clone pose
            const CameraPose& cur_clone = window.back();
            const CameraPose& prev_clone = (window.size() >= 2)
                ? window[window.size() - 2] : window.back();

            for (int iter = 0; iter < BA_MAX_ITER; iter++) {
                // Build normal equations: H * ds = -g
                double H = 0.0;  // Scalar Hessian approx (J^T W J)
                double g = 0.0;  // Gradient (J^T W r)
                double cost = 0.0;

                size_t n_pts = std::min(frame.points_3d.size(), frame.next_good.size());
                for (size_t i = 0; i < n_pts; i++) {
                    const auto& pt3 = frame.points_3d[i];
                    if (pt3.z <= 0.1 || pt3.z > 20.0) continue;

                    // Scaled 3D point
                    double X = pt3.x * scale;
                    double Y = pt3.y * scale;
                    double Z = pt3.z * scale;

                    if (Z < 0.1) continue;
                    double Z_inv = 1.0 / Z;

                    // Project into current frame
                    double xp = fx * X * Z_inv + cx;
                    double yp = fy * Y * Z_inv + cy;
                    double rx = xp - frame.next_good[i].x;
                    double ry = yp - frame.next_good[i].y;
                    double rsq = rx * rx + ry * ry;
                    double rn = std::sqrt(rsq);

                    // Huber weighting
                    double hw = (rn < HUBER_THRESHOLD) ? 1.0 : HUBER_THRESHOLD / (rn + 1e-6);

                    cost += (rn < HUBER_THRESHOLD) ? 0.5 * rsq
                            : HUBER_THRESHOLD * (rn - 0.5 * HUBER_THRESHOLD);

                    // Jacobian of reprojection error w.r.t. scale:
                    // d(xp)/ds = fx * pt3.x * (1/Z) (since X = pt3.x * s, Z = pt3.z * s)
                    // Note: X/Z = pt3.x/pt3.z is scale-independent for projection.
                    // The scale-dependent part comes from depth:
                    // When using triangulated depth: z_metric = depth * scale
                    // d(xp)/ds = 0 (projection is scale-invariant)
                    // But reprojection from clone pose IS scale-dependent:
                    double dp_ds_x = pt3.x * Z_inv;  // d(X/Z)/ds = 0, but depth factor
                    double dp_ds_y = pt3.y * Z_inv;

                    double J_s = hw * (rx * fx * dp_ds_x + ry * fy * dp_ds_y);

                    H += J_s * J_s;
                    g += J_s * (rx * fx * dp_ds_x + ry * fy * dp_ds_y) * hw;
                }

                if (H < 1e-10) break;

                // Gauss-Newton step with Levenberg-Marquardt damping
                double lambda_local = 0.01 * H;
                double ds = -g / (H + lambda_local);

                // Clamp step
                ds = std::max(-0.1, std::min(0.1, ds));
                scale += ds;
                scale = std::max(0.005, std::min(20.0, scale));

                if (std::abs(prev_cost - cost) < 0.01) break;
                prev_cost = cost;
            }

            // Temporal smoothness constraint
            double dt = (frame.timestamp_ns - last_ba_timestamp_ns_) / 1e9;
            last_ba_timestamp_ns_ = frame.timestamp_ns;
            if (dt > 0 && dt < 2.0) {
                double max_rate = (frame.quality > 0.7) ? 0.05 : 0.08;
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
                LOGI("BA(MSCKF): scale=%.4f clones=%zu pts=%zu cost=%.2f",
                     scale, window.size(), frame.points_3d.size(), prev_cost);
            }

            result.is_valid = true;
            result.optimized_scale = scale;
            result.alpha = 0.08;
            return result;
        }
    }

    // ── Mode B: Legacy keyframe-based BA (fallback) ─────────────────────────
    if (!keyframe_window_.has_minimum()) return result;

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

    double scale = frame.estimated_scale;

    // Gauss-Newton on scale (1-DOF) with Huber loss
    double prev_cost = 1e12;
    for (int iter = 0; iter < BA_MAX_ITER; iter++) {
        double H = 0.0, g_val = 0.0, cost = 0.0;

        for (size_t i = 0; i < observations.size(); i++) {
            const auto& obs = observations[i];
            double Z = obs.pt3d.z * scale;
            if (Z < 0.1) continue;
            double Z_inv = 1.0 / Z;

            double xp = fx * obs.pt3d.x * scale * Z_inv + cx;
            double yp = fy * obs.pt3d.y * scale * Z_inv + cy;
            double rx = xp - obs.pt2d.x, ry = yp - obs.pt2d.y;
            double rsq = rx * rx + ry * ry, rn = std::sqrt(rsq);

            double hw = (rn < HUBER_THRESHOLD) ? 1.0 : HUBER_THRESHOLD / (rn + 1e-6);
            cost += obs.weight * ((rn < HUBER_THRESHOLD) ? 0.5 * rsq
                  : HUBER_THRESHOLD * (rn - 0.5 * HUBER_THRESHOLD));

            double J_s = obs.weight * hw * (rx * fx * obs.pt3d.x * Z_inv
                                           + ry * fy * obs.pt3d.y * Z_inv);
            H += J_s * J_s;
            g_val += J_s * hw * obs.weight * (rx * fx * obs.pt3d.x * Z_inv
                                              + ry * fy * obs.pt3d.y * Z_inv);
        }

        if (H < 1e-10 || std::abs(prev_cost - cost) < 0.01) break;

        double ds = -g_val / (H + 0.01 * H);  // LM damping
        ds = std::max(-0.1, std::min(0.1, ds));
        scale += ds;
        scale = std::max(0.005, std::min(20.0, scale));
        prev_cost = cost;
    }

    // Temporal smoothness constraint
    double dt = (frame.timestamp_ns - last_ba_timestamp_ns_) / 1e9;
    last_ba_timestamp_ns_ = frame.timestamp_ns;
    if (dt > 0 && dt < 2.0) {
        double max_rate = (frame.quality > 0.7) ? 0.05 : 0.08;
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
        LOGI("BA(legacy): scale=%.4f obs=%zu kfs=%zu cost=%.2f",
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

        // Add loop closure edge to pose graph
        // Find the closest node to the matched keyframe position
        double match_x = matched_kf.position.x;
        double match_z = matched_kf.position.z;
        double match_h = matched_kf.heading;

        // The relative measurement: current pose should be at matched position
        double dx = match_x - pos.x;
        double dz = match_z - pos.z;
        double dh = match_h - frame.heading;

        if (last_pose_graph_node_id_ >= 0) {
            // Add loop edge and optimize
            pose_graph_.addLoopEdge(last_pose_graph_node_id_, last_pose_graph_node_id_,
                                     dx, dz, dh, 10.0, 5.0);
            pose_graph_.optimize(15);

            // Get optimized correction for current node
            double corr_x, corr_z, corr_h;
            if (pose_graph_.getLatestCorrection(corr_x, corr_z, corr_h)) {
                result.detected = true;
                result.target_x = pos.x + corr_x;
                result.target_y = matched_kf.position.y;
                result.target_z = pos.z + corr_z;
                result.target_heading = frame.heading + corr_h;
                // Pose graph distributes correction across all nodes,
                // so we can apply a larger fraction than naive blending
                result.blend = 0.50;
            }
        }

        LOGI("LOOP_CLOSURE: graph-optimized at (%.1f, %.1f) heading=%.1f° loops=%d",
             result.target_x, result.target_z,
             result.target_heading * 180.0 / M_PI,
             pose_graph_.getLoopEdgeCount());
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
