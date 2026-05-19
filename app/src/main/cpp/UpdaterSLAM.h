#pragma once
#include <vector>
#include <opencv2/core.hpp>

class EKFState;

/**
 * @brief SLAM feature updater following OpenVINS doctrine.
 *
 * This class implements the update_skf (Sequential Kalman Filter) logic
 * for persistent SLAM features in the EKF state.
 */
class UpdaterSLAM {
public:
    struct Options {
        double chi2_threshold;        // 95% threshold for 2 DOF (default 5.991)
        double min_parallax_ratio;     // baseline-to-depth ratio (default 0.01)
        double min_depth_observability; // Jacobian norm threshold (default 0.001)

        Options() : chi2_threshold(5.991),
                    min_parallax_ratio(0.01),
                    min_depth_observability(0.001) {}
    };

    UpdaterSLAM(const Options& options = Options()) : options_(options) {}

    /**
     * @brief Update a single SLAM feature slot using one or more observations.
     * Implements parallax, depth-observability, and chi-squared gating.
     * @return true if an update was successfully applied.
     */
    bool update_skf(EKFState& state, int slot,
                    const std::vector<cv::Point2f>& observations,
                    const std::vector<int>& clone_ids,
                    double pixel_noise_sq);

private:
    Options options_;
};
