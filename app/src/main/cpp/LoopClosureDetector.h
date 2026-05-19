#pragma once

// LoopClosureDetector — DBoW2-backed same-session place recognition.
//
// Plan Step 7 / ADR-013. Replaces the spatial-prior visual-match stub used
// by Steps 1-6. The hand-rolled BoW heuristic worked on dense indoor walks
// but failed in the open-loop 1 km field test where genuine returns to the
// start point were rejected as "no nearby keyframe" because VIO had drifted
// by 11+ m. DBoW2 is independent of position estimates: it scores image
// similarity against the full keyframe database via inverted index, then
// we run BFMatcher + solvePnPRansac for geometric verification.
//
// Threading: this class is shared between two threads (see Tracker.h §11.5):
//   - the camera thread calls addKeyframe() after every accepted keyframe
//   - the loop-closure worker thread (1 Hz) calls tryDetectLoop()
// All public methods take an internal mutex so call sites do not need to
// serialise at the Tracker layer.
//
// Pipeline inside tryDetectLoop():
//   1. Convert the query's CV_8U Nx32 descriptor mat to a vector<cv::Mat>
//      of single-row mats (DBoW2's ORB feature form).
//   2. db_.query(features, results, max_results=4, max_id=excluded_max_id)
//      where excluded_max_id is the highest db entry whose timestamp is
//      older than now_ns - TEMPORAL_EXCLUSION_NS. Hard-prevents matches
//      with the immediate-past keyframes — those are not loops, they are
//      the same place tracked continuously.
//   3. Top result must score ≥ BOW_SCORE_THRESHOLD (0.05, Galvez-Lopez
//      2012 §V default) — anything lower is noise.
//   4. Geometric verification: BFMatcher Hamming + Lowe ratio (0.75)
//      between query descriptors and the candidate's stored descriptors,
//      yielding 2D-3D pairs.
//   5. cv::solvePnPRansac with reproj threshold 4.0 px, conf 0.99, 100
//      iters. Inliers must be ≥ PNP_MIN_INLIERS (30).
//   6. Populate LoopMatch with the candidate's stored world pose plus
//      the relative cam-now → cam-match transform from PnP.

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

// Forward declarations of DBoW2 types — pulled from third_party/DBoW2.
// We include the real headers from the .cpp instead of here so callers
// of LoopClosureDetector.h are not forced to add the DBoW2 include path.
namespace DBoW2 {
template <class TDescriptor, class F>
class TemplatedVocabulary;
template <class TDescriptor, class F>
class TemplatedDatabase;
class FORB;
}  // namespace DBoW2

class LoopClosureDetector {
public:
    LoopClosureDetector();
    ~LoopClosureDetector();

    // Non-copyable, non-movable: holds an OrbDatabase that internally points
    // at the OrbVocabulary; copying would silently double-free.
    LoopClosureDetector(const LoopClosureDetector&)            = delete;
    LoopClosureDetector& operator=(const LoopClosureDetector&) = delete;

    // Load the ORB-SLAM2 plain-text vocabulary file from `vocab_path`.
    // Kotlin extracts assets/ORBvoc.txt.tar.gz to a real filesystem path
    // (cv::FileStorage cannot read AssetManager URIs) before calling JNI.
    // Returns true on success and isReady() will then return true.
    // On failure logs once at ERROR level and leaves the detector idle —
    // subsequent addKeyframe / tryDetectLoop calls become no-ops.
    bool loadVocabulary(const std::string& vocab_path);

    // Register a keyframe with the BoW database. Stores the descriptors,
    // keypoints, 3D world points, and pose alongside the BoW entry so
    // later geometric verification has everything it needs.
    //
    // Thread-safe: takes internal mutex. Called from the camera thread.
    //
    //   kf_id              caller-assigned keyframe id (e.g. clone slot id)
    //   timestamp_ns       monotonic-clock timestamp in ns
    //   descriptors        CV_8U Nx32 ORB descriptor mat
    //   keypoints          per-row keypoints (size == descriptors.rows)
    //   pts3d_world        per-row triangulated 3D world points
    //                      (cv::Point3f(NaN,NaN,NaN) for rows without
    //                      depth — those rows skip PnP but stay BoW-active)
    //   R_world_cam        camera→world rotation at this keyframe
    //   t_cam_world        camera position in world frame
    //   yaw_rad            camera heading at this keyframe (Tracker::scalar_heading_,
    //                      measured from north, clockwise). Used by tryDetectLoop
    //                      to gate same-direction revisits.
    void addKeyframe(uint64_t kf_id, double timestamp_ns,
                     const cv::Mat& descriptors,
                     const std::vector<cv::KeyPoint>& keypoints,
                     const std::vector<cv::Point3f>& pts3d_world,
                     const cv::Matx33d& R_world_cam,
                     const cv::Vec3d&   t_cam_world,
                     double yaw_rad);

    // Match payload returned by tryDetectLoop. Identical layout to the
    // one declared in Tracker.h's loop_closure_pending_match_.
    //
    // Frame conventions (Tracker::consumeLoopClosureMatchIfReady →
    // EKFState::updateAbsolutePose depends on getting these right):
    //
    //   R_now_to_match : 3x3 rotation taking now-cam-frame VECTORS into
    //                    match-cam-frame. For any 3-D point X expressed
    //                    in now-cam coordinates X_now,
    //                        X_match = R_now_to_match * X_now
    //                                + t_now_to_match.
    //   t_now_to_match : translation expressed in MATCH-CAM coordinates
    //                    so the equation above holds. NOT a "camera
    //                    position difference" — it is the OpenCV-tvec-
    //                    style translation component of the homogeneous
    //                    now→match transform. Built in
    //                    LoopClosureDetector.cpp:450 as
    //                        t_match_world - R_now_to_match * t_now_world
    //                    where the inputs are world→cam translations.
    //   R_world_cam_match : cam→world rotation for the matched keyframe.
    //                       X_world = R_world_cam_match * X_match
    //                              + t_cam_world_match.
    //   t_cam_world_match : matched-camera POSITION in world frame (a
    //                       world-frame point, NOT a homogeneous tvec).
    //                       Composes correctly for the now-cam world
    //                       position as
    //                         p_now_world = R_world_cam_match *
    //                                       t_now_to_match
    //                                     + t_cam_world_match.
    // 2026-05-13 Phase 1 Step 5 (post_v19_sprint_plan.md): apply a 4-DOF
    // correction (Δp, Δyaw) to a stored keyframe's pose, in place.
    //
    // After PoseGraph::optimize() runs at LC accept, each keyframe between
    // K_match and K_now gets a per-node correction. This method writes those
    // corrections back into the stored keyframe DB so:
    //   * Future BoW matches resolve against corrected poses (not pre-drift)
    //   * PnP at revisit operates on consistent 3D points + reference pose
    //   * The heading gate uses the corrected yaw_rad
    //
    // Convention: Δp is added to t_cam_world (Z-up world meters). Δyaw is
    // applied as a left-multiplication of R_world_cam by R_z(Δyaw), i.e.
    // a rotation around world-Z. yaw_rad is incremented and wrapped to
    // [-π, π]. The stored ORB descriptors / keypoints / pts3d_world are
    // NOT touched — only the pose-frame they reference moves.
    //
    // Thread-safe: takes the internal mutex briefly while iterating the
    // keyframe vector. Returns false if kf_id is unknown.
    bool applyKeyframePoseCorrection(uint64_t kf_id,
                                      double dx, double dy, double dz,
                                      double dyaw);

    struct LoopMatch {
        uint64_t    matched_kf_id        = 0;
        double      bow_score            = 0.0;
        int         pnp_inliers          = 0;
        // Homogeneous now-cam → match-cam transform from PnP, applied as
        //   X_match = R_now_to_match * X_now + t_now_to_match.
        cv::Matx33d R_now_to_match       = cv::Matx33d::eye();
        cv::Vec3d   t_now_to_match       = cv::Vec3d(0, 0, 0);
        // The matched keyframe's stored world pose (cam→world rotation +
        // camera position in world).
        cv::Matx33d R_world_cam_match    = cv::Matx33d::eye();
        cv::Vec3d   t_cam_world_match    = cv::Vec3d(0, 0, 0);
    };

    // Run the BoW query + geometric verification. Returns true if a
    // loop was detected; in that case `out_match` is populated.
    //
    // Thread-safe: takes internal mutex briefly while reading the database
    // and the candidate's stored entry, then releases the lock before the
    // (potentially slow) PnP step.
    //
    //   now_kf_id                 caller-assigned id for the query frame
    //   now_ns                    monotonic-clock timestamp of query frame
    //   descriptors / keypoints   CV_8U Nx32 ORB + matching keypoint vec
    //   fx, fy, cx, cy            pinhole intrinsics in pixels
    //   temporal_exclusion_ns     suppress matches against any keyframe
    //                             newer than (now_ns - this) — i.e. the
    //                             recent past; 30 s is the project default
    //                             (see TEMPORAL_EXCLUSION_NS in the .cpp).
    //   current_yaw_rad           camera heading of the query frame
    //                             (Tracker::scalar_heading_). Candidates
    //                             whose stored yaw differs by > π/2 are
    //                             rejected before PnP — ORB descriptors
    //                             are not 180°-invariant and produce
    //                             geometrically inconsistent 3D-2D pairs
    //                             when the viewpoint is reversed.
    //   out_match                 populated on success.
    bool tryDetectLoop(uint64_t now_kf_id, int64_t now_ns,
                       const cv::Mat& descriptors,
                       const std::vector<cv::KeyPoint>& keypoints,
                       double fx, double fy, double cx, double cy,
                       int64_t temporal_exclusion_ns,
                       double current_yaw_rad,
                       LoopMatch& out_match);

    // ── Phase 6.4c — Spatial pre-filter overload ──────────────────────────
    //
    // Same pipeline as tryDetectLoop above (adaptive BoW gate, heading,
    // BFMatcher Lowe ratio, PnP-RANSAC), but the DBoW2 query is restricted
    // to keyframes whose kf_id appears in `candidate_kf_ids`. Use this when
    // the caller has a position estimate and wants to suppress geographic
    // false positives (visually-similar places that are physically far).
    //
    // Pass an EMPTY `candidate_kf_ids` to fall back to the full-database
    // search semantics of tryDetectLoop (a safety net for when the
    // LandmarkMap is still seeding or for queries far from any mapped
    // landmark). The caller is responsible for counting fallbacks via the
    // EventCounters loop_closure_spatial_* fields.
    //
    // Thread-safe: same locking as tryDetectLoop.
    bool tryDetectLoopWithCandidates(
        uint64_t now_kf_id, int64_t now_ns,
        const cv::Mat& descriptors,
        const std::vector<cv::KeyPoint>& keypoints,
        double fx, double fy, double cx, double cy,
        int64_t temporal_exclusion_ns,
        double current_yaw_rad,
        const std::vector<uint64_t>& candidate_kf_ids,
        LoopMatch& out_match);

    // ── Step 7.1 — Geometric loop closure (additive to BoW) ────────────────
    //
    // Direction-invariant detection that bypasses ORB descriptors entirely.
    // Spec: docs/VISUAL_PLAN_STEP_7_1_GEOMETRIC_LOOP.md.
    //
    // Pipeline:
    //   1. Spatial filter: keyframes within `position_search_radius_m` of
    //      `predicted_t_cam_world`, older than `temporal_exclusion_ns`.
    //   2. For each candidate, project its stored `pts3d_world` into the
    //      predicted current camera (predicted_R_world_cam, predicted_t_cam_world).
    //   3. Reject candidates with < kGeomMinInFrame projections in-frame
    //      with positive depth in [0.5, 50] m.
    //   4. For each in-frame projection: find current ORB keypoints within
    //      kGeomMatchRadiusPx px, pick the one with min Hamming distance
    //      to the stored keyframe descriptor. Reject pair if min Hamming
    //      > kGeomDescriptorMaxDistance. 2026-05-13 Step 7.1 fix: replaces
    //      the appearance-blind KLT-corner NN that admitted false positives
    //      in low-texture scenes.
    //   5. solvePnPRansac on the (3D world, 2D current image) pairs; same
    //      thresholds as the BoW path.
    //   6. Build LoopMatch identical to the BoW return value so the EKF
    //      injection path doesn't care which detector fired.
    //
    // The heading gate (current_yaw_rad / kMaxHeadingDiffRad in tryDetectLoop)
    // is intentionally NOT applied here — opposite-direction U-turn revisits
    // are exactly what this method is designed to catch. The downstream
    // χ²(0.999, 6) gate at EKFState.cpp:1000 is the safety net for genuinely
    // wrong matches.
    //
    // Thread-safe: takes internal mutex briefly while filtering keyframes,
    // then releases before the (potentially slow) projection / PnP work.
    //
    //   now_kf_id                 caller-assigned id for the query frame
    //   now_ns                    monotonic-clock timestamp of query frame
    //   current_descriptors       CV_8U Nx32 ORB descriptors for the current
    //                             frame (same matrix the BoW path consumes).
    //                             2026-05-13 Step 7.1 fix: replaces
    //                             current_klt_corners. The KLT-corner NN
    //                             matcher admitted appearance-blind
    //                             correspondences in low-texture indoor
    //                             scenes (v24 walk: geom accept with r_R=37°
    //                             passed chi² because var_p was inflated by
    //                             drift_path). Hamming verification against
    //                             stored ORB descriptors gates that out per
    //                             ORB-SLAM3 standard (plan Step 6 line 280).
    //   current_keypoints         per-row keypoints aligned with
    //                             current_descriptors (size ==
    //                             current_descriptors.rows). The keypoint's
    //                             .pt is the 2D pixel used in the PnP pair.
    //   predicted_R_world_cam     EKF's predicted current camera→world
    //                             rotation. For NavSight: ekf.getRotation()
    //                             (R_GtoI = world→body) inverted ×
    //                             R_bc.t() to get cam→world.
    //   predicted_t_cam_world     EKF's predicted current camera position
    //                             in world frame. For NavSight handheld:
    //                             ekf.getPosition() (body=camera).
    //   fx, fy, cx, cy            pinhole intrinsics in pixels
    //   img_width, img_height     image dimensions in pixels — defines the
    //                             in-frame test
    //   position_search_radius_m  see spec §"Position search radius
    //                             derivation": typically 3 × σ_p_xy with a
    //                             [2.0, 30.0] m clamp.
    //   temporal_exclusion_ns     same as the BoW path (typically 30 s)
    //   out_match                 populated on success.
    bool tryDetectLoopGeometric(
        uint64_t now_kf_id, int64_t now_ns,
        const cv::Mat& current_descriptors,
        const std::vector<cv::KeyPoint>& current_keypoints,
        const cv::Matx33d& predicted_R_world_cam,
        const cv::Vec3d&   predicted_t_cam_world,
        double fx, double fy, double cx, double cy,
        int img_width, int img_height,
        double position_search_radius_m,
        int64_t temporal_exclusion_ns,
        LoopMatch& out_match);

    // Returns the number of keyframes currently in the BoW database.
    size_t getKeyframeCount() const;

    // True iff a vocabulary has been loaded successfully and the database
    // is willing to accept keyframes / answer queries. Cheap atomic load.
    bool isReady() const;

private:
    // Phase 6.4c — shared core. If `candidate_kf_ids` is non-empty, BoW
    // results are intersected against it before geometric verification.
    bool tryDetectLoopImpl(
        uint64_t now_kf_id, int64_t now_ns,
        const cv::Mat& descriptors,
        const std::vector<cv::KeyPoint>& keypoints,
        double fx, double fy, double cx, double cy,
        int64_t temporal_exclusion_ns,
        double current_yaw_rad,
        const std::vector<uint64_t>* candidate_kf_ids,
        LoopMatch& out_match);

    // PIMPL-ish: DBoW2 types are template-heavy and dragging them into
    // this header forces every Tracker translation unit to recompile
    // when third_party/DBoW2 changes. Keep the BoW types behind a forward
    // decl and a unique_ptr<Impl>.
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
