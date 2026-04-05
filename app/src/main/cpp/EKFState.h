#pragma once

// 1-state Extended Kalman Filter for scale estimation.
//
// State: x = [scale]
//   scale: visual odometry scale factor (meters/unit)
//
// Decoupled from heading to prevent cross-covariance leakage.
// Heading is handled by global_R_ accumulation in Tracker (proven stable).
// Scale EKF provides: uncertainty tracking, Mahalanobis outlier rejection,
// and smooth convergence from step-based observations.
class EKFState {
public:
    EKFState();

    // Initialize scale (called at VIO start)
    void initialize(double initial_scale);

    // Scale update: step-based or ground-plane scale observation
    void updateScale(double observed_scale, double confidence);

    // ZUPT update: when stationary, freeze scale drift
    void updateZUPT();

    // Cross-validation: return consistency score (0=disagree, 1=agree)
    double checkConsistency(double camera_disp, double step_disp) const;

    // Accessors
    double getScale() const { return scale_; }
    double getScaleStd() const;

    void reset();

private:
    double scale_;       // scale state
    double P_;           // scale variance

    bool initialized_{false};
    int update_count_{0};

    // Process noise
    static constexpr double SIGMA_SCALE_RW = 0.001;  // scale random walk per second

    // Measurement noise
    static constexpr double SIGMA_SCALE_MEAS = 0.1;  // scale observation noise
};
