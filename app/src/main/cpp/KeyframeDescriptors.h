#pragma once
#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

// Per-keyframe ORB descriptor record stored in FeatureManager's
// ring buffer for relocalization (Plan Step 4, ADR-010).
//
// - keypoints / descriptors / feature_ids are parallel arrays.
// - feature_ids[i] = -1 when an ORB keypoint did not align with any
//   tracked KLT corner at storage time (no FeatureManager id available).
// - descriptors is a CV_8U matrix with 32 columns (ORB binary).
struct KeyframeDescriptors {
    uint64_t                  keyframe_id   = 0;
    double                    timestamp_ns  = 0.0;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;
    std::vector<int>          feature_ids;
};
