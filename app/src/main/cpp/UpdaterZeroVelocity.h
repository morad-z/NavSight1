#pragma once
#include <vector>
#include <deque>
#include <opencv2/core.hpp>
#include "IMUPreintegrator.h"

/**
 * @brief Robust stationary detection based on OpenVINS' UpdaterZeroVelocity.
 */
class UpdaterZeroVelocity {
public:
    struct Options {
        int window_size;
        double sigma_g;
        double sigma_a;
        double chi2_multiplier;
        double max_disparity;
        double gravity_mag;

        Options() :
            window_size(20),
            sigma_g(0.025),            // Was 0.005; phone gyros are 5-10x noisier than research IMUs
            sigma_a(0.15),             // Was 0.05; phone accels are noisier too
            chi2_multiplier(3.0),      // Was 2.0; give margin for noisy Android IMUs
            max_disparity(1.5),        // Was 1.0; KLT has noise even when stationary
            gravity_mag(9.81) {}
    };

    UpdaterZeroVelocity(const Options& options = Options()) : options_(options) {}

    /**
     * @brief Checks if the device is stationary based on IMU and vision.
     */
    bool is_stationary(const std::vector<AccelSample>& accel_window,
                       const std::vector<GyroSample>& gyro_window,
                       double visual_disparity);

private:
    Options options_;

    // Chi-squared 95% threshold for k degrees of freedom
    double get_chi2_threshold(int dof) const;
};
