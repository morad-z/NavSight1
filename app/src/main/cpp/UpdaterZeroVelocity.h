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
            sigma_g(0.06),             // 2023 Standard: was 0.025; handheld phone gyros have jitter
            sigma_a(0.20),             // 2023 Standard: was 0.15; handle micro-vibrations
            chi2_multiplier(2.5),      // Give moderate margin for non-Gaussian tail noise
            max_disparity(3.0),        // Was 1.5; handle low-light camera jitter/noise
            gravity_mag(9.81) {}
    };

    UpdaterZeroVelocity(const Options& options = Options()) : options_(options) {}

    /**
     * @brief Updates the detector options (e.g. for different gait modes).
     */
    void setOptions(const Options& options) { options_ = options; }

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
