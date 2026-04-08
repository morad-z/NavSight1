#pragma once
#include <vector>
#include <opencv2/core.hpp>

class EKFState;

/**
 * @brief Feature observation from a specific camera clone.
 */
struct FeatureObservation {
    int clone_state_id;    // EKF clone state ID
    cv::Point2f pixel_ud;  // Undistorted normalized coordinates
};

/**
 * @brief A feature track that has been lost (no longer tracked).
 * Contains all observations across the sliding window.
 */
struct LostFeature {
    int feature_id;
    std::vector<FeatureObservation> observations;  // At least 3 for MSCKF
};

/**
 * @brief MSCKF updater using null-space feature marginalization.
 *
 * When a feature track ends, this class:
 * 1. Triangulates the 3D point from multi-view observations
 * 2. Forms measurement Jacobians w.r.t. feature (H_f) and state (H_x)
 * 3. Projects onto the left null-space of H_f to eliminate the feature
 * 4. Applies the resulting constraint as an EKF update on camera poses
 *
 * Reference: Mourikis & Roumeliotis, ICRA 2007; Geneva et al., ICRA 2020.
 */
class UpdaterMSCKF {
public:
    struct Options {
        double chi2_multiplier;
        int min_obs;
        double max_reproj_px;
        double pixel_noise;

        Options() : chi2_multiplier(1.5), min_obs(3),
                    max_reproj_px(5.0), pixel_noise(1.0) {}
    };

    UpdaterMSCKF(const Options& options = Options()) : options_(options) {}

    /**
     * @brief Process lost features: triangulate, null-space project, update EKF.
     * @param state The EKF state (will be modified by update)
     * @param lost_features Features that are no longer tracked
     * @param fx, fy, cx, cy Camera intrinsics
     * @return Number of features successfully used for update
     */
    int processLostFeatures(EKFState& state,
                            const std::vector<LostFeature>& lost_features,
                            double fx, double fy, double cx, double cy);

private:
    /**
     * @brief Triangulate a feature from multi-view observations using DLT.
     * @return true if triangulation succeeded (positive depth, reasonable error)
     */
    bool triangulate(const LostFeature& feat, const EKFState& state,
                     cv::Point3d& pt3d_out) const;

    /**
     * @brief Build Jacobians H_f (w.r.t feature) and H_x (w.r.t state), plus residual.
     */
    void getFeatureJacobian(const LostFeature& feat, const EKFState& state,
                            const cv::Point3d& pt3d,
                            double fx, double fy, double cx, double cy,
                            cv::Mat& H_f, cv::Mat& H_x, cv::Mat& res) const;

    /**
     * @brief QR null-space projection: eliminates feature DOF from Jacobian.
     * Modifies H_x and res in-place to be projected versions.
     */
    static void nullspaceProject(const cv::Mat& H_f, cv::Mat& H_x, cv::Mat& res);

    /**
     * @brief Chi-squared test for Mahalanobis gating.
     */
    double getChi2Threshold(int dof) const;

    Options options_;
};
