#include "UpdaterZeroVelocity.h"
#include <cmath>
#include <numeric>
#include <algorithm>

#ifdef __ANDROID__
#include <android/log.h>
#define TAG "NavSight-ZUPT"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#else
#define LOGD(...) (void)0
#endif

bool UpdaterZeroVelocity::is_stationary(const std::vector<AccelSample>& accel_window,
                                       const std::vector<GyroSample>& gyro_window,
                                       double visual_disparity) {
    if (accel_window.size() < static_cast<size_t>(options_.window_size) ||
        gyro_window.size() < static_cast<size_t>(options_.window_size)) {
        return false;
    }

    // 1. Visual Disparity Check (Gate)
    if (visual_disparity > options_.max_disparity) {
        return false;
    }

    const int N = options_.window_size;
    const int start_idx_a = static_cast<int>(accel_window.size()) - N;
    const int start_idx_g = static_cast<int>(gyro_window.size()) - N;

    // 2. Gyroscope Variance Test
    double mean_gx = 0, mean_gy = 0, mean_gz = 0;
    for (int i = 0; i < N; ++i) {
        mean_gx += gyro_window[start_idx_g + i].x;
        mean_gy += gyro_window[start_idx_g + i].y;
        mean_gz += gyro_window[start_idx_g + i].z;
    }
    mean_gx /= N; mean_gy /= N; mean_gz /= N;

    double sum_sq_g = 0;
    for (int i = 0; i < N; ++i) {
        double dx = gyro_window[start_idx_g + i].x - mean_gx;
        double dy = gyro_window[start_idx_g + i].y - mean_gy;
        double dz = gyro_window[start_idx_g + i].z - mean_gz;
        sum_sq_g += (dx*dx + dy*dy + dz*dz);
    }

    // Chi2_g = sum((w - mean)^2 / sigma_g^2)
    // sigma_g is noise density, but we need discrete noise: sigma_d = sigma_g / sqrt(dt)
    // However, for simplicity and alignment with OpenVINS, we use a calibrated threshold.
    // In many implementations, sum_sq_g / (sigma_g^2) is compared to chi2 threshold.
    double chi2_g = sum_sq_g / (options_.sigma_g * options_.sigma_g);

    // 3. Accelerometer Variance Test
    double mean_ax = 0, mean_ay = 0, mean_az = 0;
    for (int i = 0; i < N; ++i) {
        mean_ax += accel_window[start_idx_a + i].x;
        mean_ay += accel_window[start_idx_a + i].y;
        mean_az += accel_window[start_idx_a + i].z;
    }
    mean_ax /= N; mean_ay /= N; mean_az /= N;

    double sum_sq_a = 0;
    for (int i = 0; i < N; ++i) {
        double dx = accel_window[start_idx_a + i].x - mean_ax;
        double dy = accel_window[start_idx_a + i].y - mean_ay;
        double dz = accel_window[start_idx_a + i].z - mean_az;
        sum_sq_a += (dx*dx + dy*dy + dz*dz);
    }
    double chi2_a = sum_sq_a / (options_.sigma_a * options_.sigma_a);

    // 4. Gravity Magnitude Test (Is mean accel close to gravity?)
    double mean_accel_mag = std::sqrt(mean_ax*mean_ax + mean_ay*mean_ay + mean_az*mean_az);
    double gravity_err = std::abs(mean_accel_mag - options_.gravity_mag);
    bool gravity_ok = (gravity_err < options_.sigma_a * 3.0);

    // Thresholds for dof = 3*(N-1)
    double threshold = get_chi2_threshold(3 * (N - 1)) * options_.chi2_multiplier;

    bool stationary = (chi2_g < threshold) && (chi2_a < threshold) && gravity_ok;

    if (stationary || chi2_g < threshold * 2.0) {
        LOGD("ZUPT Test: chi2_g=%.2f, chi2_a=%.2f, threshold=%.2f, disp=%.2f, g_err=%.2f -> %s",
             chi2_g, chi2_a, threshold, visual_disparity, gravity_err, stationary ? "STATIONARY" : "MOVING");
    }

    return stationary;
}

double UpdaterZeroVelocity::get_chi2_threshold(int dof) const {
    // Wilson-Hilferty approximation for Chi-squared inverse CDF (p=0.95)
    // F^-1(p; k) = k * (1 - 2/(9k) + z * sqrt(2/(9k)))^3
    const double z = 1.645; // 95th percentile of standard normal
    double k = static_cast<double>(dof);
    double term = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
    return k * term * term * term;
}
