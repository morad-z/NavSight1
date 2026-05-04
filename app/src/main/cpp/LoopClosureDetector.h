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
    void addKeyframe(uint64_t kf_id, double timestamp_ns,
                     const cv::Mat& descriptors,
                     const std::vector<cv::KeyPoint>& keypoints,
                     const std::vector<cv::Point3f>& pts3d_world,
                     const cv::Matx33d& R_world_cam,
                     const cv::Vec3d&   t_cam_world);

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
    //   out_match                 populated on success.
    bool tryDetectLoop(uint64_t now_kf_id, int64_t now_ns,
                       const cv::Mat& descriptors,
                       const std::vector<cv::KeyPoint>& keypoints,
                       double fx, double fy, double cx, double cy,
                       int64_t temporal_exclusion_ns,
                       LoopMatch& out_match);

    // Returns the number of keyframes currently in the BoW database.
    size_t getKeyframeCount() const;

    // True iff a vocabulary has been loaded successfully and the database
    // is willing to accept keyframes / answer queries. Cheap atomic load.
    bool isReady() const;

private:
    // PIMPL-ish: DBoW2 types are template-heavy and dragging them into
    // this header forces every Tracker translation unit to recompile
    // when third_party/DBoW2 changes. Keep the BoW types behind a forward
    // decl and a unique_ptr<Impl>.
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
