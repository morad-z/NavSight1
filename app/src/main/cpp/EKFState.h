#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <opencv2/core.hpp>

// Camera pose clone for the MSCKF sliding window.
// Based on OpenVINS First-Estimate Jacobians (FEJ) paradigm.
struct CameraPose {
    cv::Mat R_GtoC;     // 3x3 CV_64F: Rotation from Global to Camera frame
    cv::Mat p_G;        // 3x1 CV_64F: Position in Global frame

    // First-Estimate Jacobians (FEJ) storage.
    // Locked linearization points to maintain observability of global yaw and position.
    cv::Mat R_FEJ;      // Fixed rotation when this pose was first initialized
    cv::Mat p_FEJ;      // Fixed position when this pose was first initialized

    int64_t timestamp_ns;
    int state_id;

    CameraPose() : timestamp_ns(0), state_id(-1) {
        R_GtoC = cv::Mat::eye(3, 3, CV_64F);
        p_G = cv::Mat::zeros(3, 1, CV_64F);
        R_FEJ = cv::Mat::eye(3, 3, CV_64F);
        p_FEJ = cv::Mat::zeros(3, 1, CV_64F);
    }
};

// Extended Kalman Filter for VIO.
// Full error-state EKF with MSCKF sliding window support.
//
// IMU error-state: [δθ(3), δb_g(3), δv(3), δb_a(3), δp(3)] = 15 DOF
// Each clone adds: [δθ_c(3), δp_c(3)] = 6 DOF
// Total state dimension: 15 + 6*N_clones
class EKFState {
public:
    EKFState();

    // --- Legacy Scale Support (kept for backward compatibility) ---
    void initialize(double initial_scale);
    void updateScale(double observed_scale, double confidence);
    void updateZUPT();
    // DEAD CODE: checkConsistency, getScaleStd — never called
    // double checkConsistency(double camera_disp, double step_disp) const;
    double getScale() const { return scale_; }
    // double getScaleStd() const;
    void reset();

    // --- Full Error-State MSCKF ---

    // Initialize full IMU state from initial rotation and gravity
    void initializeFull(const cv::Mat& R_GtoI, const cv::Point3f& gyro_bias,
                        const cv::Point3f& accel_bias);

    // IMU state propagation (covariance propagation using preintegration)
    void propagateIMU(const cv::Mat& deltaR, const cv::Mat& deltaV,
                      const cv::Mat& deltaP, double dt,
                      const cv::Mat& imu_cov,
                      const cv::Mat& J_R_bg, const cv::Mat& J_V_bg,
                      const cv::Mat& J_V_ba, const cv::Mat& J_P_bg,
                      const cv::Mat& J_P_ba);

    // Clone management
    void addClone(const cv::Mat& R_GtoC, const cv::Mat& p_G, int64_t timestamp_ns);
    void pruneWindow(size_t max_poses = 11);
    void marginalizeOldestClone();

    // MSCKF update: apply stacked null-space-projected measurement
    void applyMSCKFUpdate(const cv::Mat& H, const cv::Mat& res, const cv::Mat& R_noise);

    // Clone accessors (used by UpdaterMSCKF)
    bool getClonePose(int state_id, cv::Mat& R_GtoC, cv::Mat& p_G) const;
    bool getCloneFEJ(int state_id, cv::Mat& R_FEJ, cv::Mat& p_FEJ) const;
    int getCloneCovIdx(int state_id) const;
    int getStateDim() const;
    cv::Mat getCovariance() const { return P_.clone(); }

    // Window access
    const std::deque<CameraPose>& getWindow() const { return window_; }
    int getLatestCloneId() const;

    // DEAD CODE: getFEJ — only used by disabled MSCKF updater
    // void getFEJ(int state_id, cv::Mat& R_fej, cv::Mat& p_fej) const;

    // IMU state accessors
    cv::Mat getRotation() const { return R_GtoI_.clone(); }
    cv::Mat getPosition() const { return p_G_.clone(); }
    // DEAD CODE: getVelocity — never called
    // cv::Mat getVelocity() const { return v_G_.clone(); }
    bool isFullInitialized() const { return full_initialized_; }

    // DEAD CODE: updateTemporal — never called
    // void updateTemporal(double observed_scale, double confidence, double H_td);
    void setTimeOffset(double td_seconds);
    double getTimeOffset() const { return t_offset_cam_imu_; }
    double getTimeOffsetStd() const { return std::sqrt(std::max(0.0, P_td_)); }

    static constexpr int IMU_STATE_DIM = 15;      // δθ, δb_g, δv, δb_a, δp
    static constexpr int CLONE_DIM = 6;            // δθ_c, δp_c
    static constexpr int MAX_CLONES = 11;

private:
    // Legacy scale estimation state
    double scale_;
    double scale_fej_{-1.0};
    double P_scale_;

    // Full IMU state (mean)
    cv::Mat R_GtoI_;    // 3x3 rotation Global-to-IMU
    cv::Mat b_g_;       // 3x1 gyro bias
    cv::Mat v_G_;       // 3x1 velocity in global frame
    cv::Mat b_a_;       // 3x1 accel bias
    cv::Mat p_G_;       // 3x1 position in global frame

    // Full covariance: (15 + 6*N_clones) x (15 + 6*N_clones)
    cv::Mat P_;

    // MSCKF Sliding Window
    std::deque<CameraPose> window_;
    int next_state_id_{0};
    bool full_initialized_{false};

    // Online calibration
    double t_offset_cam_imu_{0.010};
    double P_td_{0.005 * 0.005};

    bool initialized_{false};

    // FEJ Global Locking
    cv::Mat global_first_estimate_R_;
    cv::Mat global_first_estimate_p_;
    bool global_fej_initialized_{false};

    // IMU noise parameters (continuous-time, used for covariance propagation)
    double sigma_g_{0.01};      // gyro noise density (rad/s/sqrt(Hz))
    double sigma_a_{0.1};       // accel noise density (m/s^2/sqrt(Hz))
    double sigma_bg_{0.0001};   // gyro random walk
    double sigma_ba_{0.001};    // accel random walk

    // Constants
    static constexpr double SIGMA_SCALE_RW = 0.001;
    static constexpr double SIGMA_SCALE_MEAS = 0.1;
};
