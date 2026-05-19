#include "VisualMap.h"

void VisualMap::addOrUpdate(int feature_id,
                             const cv::Mat& p_world,
                             int anchor_clone_id,
                             int64_t ts_ns) {
    if (feature_id < 0) return;
    if (p_world.empty() || p_world.rows != 3 || p_world.cols != 1 ||
        p_world.type() != CV_64F) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto& mp = points_[feature_id];
    mp.feature_id      = feature_id;
    mp.p_world         = p_world.clone();
    mp.anchor_clone_id = anchor_clone_id;
    mp.promoted_ts_ns  = ts_ns;
}

bool VisualMap::getWorldPosition(int feature_id, cv::Mat& p_world_out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = points_.find(feature_id);
    if (it == points_.end()) return false;
    if (it->second.p_world.empty()) return false;
    p_world_out = it->second.p_world.clone();
    return true;
}

void VisualMap::noteReprojectionRMS(int feature_id, double rms_px) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = points_.find(feature_id);
    if (it == points_.end()) return;
    it->second.last_rms_px = rms_px;
}

void VisualMap::remove(int feature_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.erase(feature_id);
}

void VisualMap::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.clear();
}

std::vector<VisualMap::MapPoint> VisualMap::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<MapPoint> out;
    out.reserve(points_.size());
    for (const auto& [fid, mp] : points_) {
        out.push_back(mp);
    }
    return out;
}

size_t VisualMap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return points_.size();
}
