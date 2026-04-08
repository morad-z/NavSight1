#include "LoopClosureDetector.h"
#include <cmath>
#include <algorithm>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "LoopClosureDetector"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGI(...) (void)0
#define LOGD(...) (void)0
#endif

LoopClosureDetector::LoopClosureDetector() {
    orb_matcher_ = cv::ORB::create(500, 1.2f, 8);
}

LoopClosureDetector::~LoopClosureDetector() {
    reset();
}

int LoopClosureDetector::hammingDistance(const uint8_t* a, const uint8_t* b, int bytes) {
    int distance = 0;
    for (int i = 0; i < bytes; ++i) {
        uint8_t xor_val = a[i] ^ b[i];
        // Count set bits
        while (xor_val) {
            distance += xor_val & 1;
            xor_val >>= 1;
        }
    }
    return distance;
}

int LoopClosureDetector::matchDescriptors(
        const cv::Mat& descriptors1,
        const cv::Mat& descriptors2,
        float max_distance_ratio) {
    if (descriptors1.empty() || descriptors2.empty()) {
        return 0;
    }

    int matches = 0;
    const int bytes_per_desc = 32;  // ORB descriptors are 32 bytes

    for (int i = 0; i < descriptors1.rows; ++i) {
        const uint8_t* desc1 = descriptors1.ptr<uint8_t>(i);
        
        int best_dist = 256;
        int second_best_dist = 256;
        
        for (int j = 0; j < descriptors2.rows; ++j) {
            const uint8_t* desc2 = descriptors2.ptr<uint8_t>(j);
            int dist = hammingDistance(desc1, desc2, bytes_per_desc);
            
            if (dist < best_dist) {
                second_best_dist = best_dist;
                best_dist = dist;
            } else if (dist < second_best_dist) {
                second_best_dist = dist;
            }
        }
        
        // Lowe's ratio test: accept match if best is significantly better than second
        if (best_dist < 50 && (second_best_dist == 256 || best_dist / (float)second_best_dist < max_distance_ratio)) {
            matches++;
        }
    }
    
    return matches;
}

bool LoopClosureDetector::detectLoopClosure(
        const cv::Mat& current_descriptors,
        const std::vector<cv::Point2f>& current_keypoints,
        const cv::Point3f& current_position,
        float current_heading,
        LoopClosureKeyframe& matched_keyframe) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (keyframes_.empty()) {
        return false;  // No previous keyframes to match against
    }
    
    if (current_descriptors.empty()) {
        return false;  // No descriptors to match
    }
    
    // ✅ STRATEGY: Spatial-visual match prioritization
    // 1. First filter by position (spatial constraint)
    // 2. Then verify with visual descriptors (visual constraint)
    // This avoids expensive descriptor matching on far-away keyframes
    
    std::vector<int> spatial_candidates;
    
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        const LoopClosureKeyframe& kf = keyframes_[i];
        
        // Spatial filter: position must be within threshold
        float dx = current_position.x - kf.position.x;
        float dy = current_position.y - kf.position.y;
        float dz = current_position.z - kf.position.z;
        float position_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (position_dist < POSITION_MATCH_THRESHOLD_M) {
            spatial_candidates.push_back(i);
        }
    }
    
    // No keyframes nearby
    if (spatial_candidates.empty()) {
        return false;
    }
    
    // ✅ STRATEGY: For nearby keyframes, verify with visual descriptors
    int best_match_idx = -1;
    int best_match_count = 0;
    
    for (int idx : spatial_candidates) {
        const LoopClosureKeyframe& kf = keyframes_[idx];
        
        // Heading filter: must not be facing opposite direction
        float heading_diff = current_heading - kf.heading;
        // Normalize to [-π, π]
        while (heading_diff > M_PI) heading_diff -= 2.0f * M_PI;
        while (heading_diff < -M_PI) heading_diff += 2.0f * M_PI;
        
        if (std::abs(heading_diff) > HEADING_MATCH_THRESHOLD_RAD) {
            continue;  // Too different heading
        }
        
        // Visual descriptor matching
        int match_count = matchDescriptors(current_descriptors, kf.descriptors);
        
        LOGD("Keyframe %d: pos_dist=%.2fm, heading_diff=%.1f°, visual_matches=%d",
             kf.keyframe_id,
             std::sqrt(std::pow(current_position.x - kf.position.x, 2) + 
                      std::pow(current_position.z - kf.position.z, 2)),
             heading_diff * 180.0f / M_PI,
             match_count);
        
        if (match_count > best_match_count) {
            best_match_count = match_count;
            best_match_idx = idx;
        }
    }
    
    // Loop closure confirmed: enough visual matches
    if (best_match_count >= MIN_DESCRIPTOR_MATCHES && best_match_idx >= 0) {
        matched_keyframe = keyframes_[best_match_idx];
        
        float correction_dist = std::sqrt(
            std::pow(current_position.x - matched_keyframe.position.x, 2) +
            std::pow(current_position.z - matched_keyframe.position.z, 2)
        );
        float correction_heading = current_heading - matched_keyframe.heading;
        
        LOGI("✅ LOOP CLOSURE DETECTED!");
        LOGI("  Matched keyframe #%d at position (%.2f, %.2f, %.2f)",
             matched_keyframe.keyframe_id,
             matched_keyframe.position.x, matched_keyframe.position.y, matched_keyframe.position.z);
        LOGI("  Current position: (%.2f, %.2f, %.2f)",
             current_position.x, current_position.y, current_position.z);
        LOGI("  Position correction: %.2f m, Heading correction: %.1f°",
             correction_dist, correction_heading * 180.0f / M_PI);
        LOGI("  Visual matches: %d / threshold: %d",
             best_match_count, MIN_DESCRIPTOR_MATCHES);
        
        return true;
    }
    
    return false;
}

void LoopClosureDetector::addKeyframe(const LoopClosureKeyframe& keyframe) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Only store every Nth frame to keep database manageable
    frame_count_++;
    if (frame_count_ % KEYFRAME_SKIP != 0) {
        return;
    }
    
    LoopClosureKeyframe kf = keyframe;
    kf.keyframe_id = next_keyframe_id_++;
    
    keyframes_.push_back(kf);
    
    // Keep database size bounded (e.g., last 500 keyframes = ~17 seconds at 30fps with skip=10)
    static constexpr size_t MAX_KEYFRAMES = 500;
    if (keyframes_.size() > MAX_KEYFRAMES) {
        keyframes_.erase(keyframes_.begin());  // Remove oldest
    }
    
    LOGD("Added keyframe #%d at position (%.2f, %.2f, %.2f), total_keyframes=%zu",
         kf.keyframe_id, kf.position.x, kf.position.y, kf.position.z,
         keyframes_.size());
}

void LoopClosureDetector::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.clear();
    next_keyframe_id_ = 0;
    frame_count_ = 0;
}

int LoopClosureDetector::getKeyframeCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.size();
}
