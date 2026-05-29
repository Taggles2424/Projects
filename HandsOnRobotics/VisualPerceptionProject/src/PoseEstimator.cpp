#include "PoseEstimator.hpp"
#include <iostream>

PoseEstimator::PoseEstimator(const cv::Mat& K) {
    K.convertTo(K_, CV_64F);
    // Assume a pinhole camera with no distortion (distortion coefficients = 0)
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
}

bool PoseEstimator::estimatePose(const std::vector<cv::Point3f>& points3D,
                                 const std::vector<cv::Point2f>& points2D,
                                 cv::Mat& R, cv::Mat& t,
                                 std::vector<int>& inliers) {
    
    // Clear outputs
    inliers.clear();

    // 1. Verify we have enough points to run PnP solver
    // PnP requires at least 4 points, but for RANSAC and stability, we require at least 12 points
    if (points3D.size() < 12 || points2D.size() < 12) {
        std::cout << "[PoseEstimator] FAILED: Too few 3D-2D correspondences (" 
                  << points3D.size() << ") for tracking." << std::endl;
        return false;
    }

    cv::Mat rvec, tvec;
    
    // 2. Solve Perspective-n-Point using RANSAC with SQPNP non-iterative solver inside try-catch to prevent degenerate point crashes
    // parameters: 100 iterations, 2.0 pixels maximum reprojection error, 99% confidence
    bool success = false;
    try {
        success = cv::solvePnPRansac(points3D, points2D, K_, dist_coeffs_, rvec, tvec,
                                     false, 100, 2.0, 0.99, inliers, cv::SOLVEPNP_SQPNP);
    } catch (const cv::Exception& e) {
        std::cout << "[PoseEstimator] WARNING: solvePnPRansac threw exception: " << e.what() << std::endl;
        success = false;
    }

    if (!success) {
        std::cout << "[PoseEstimator] FAILED: PnP RANSAC solver failed to find a valid pose." << std::endl;
        return false;
    }

    // 3. Verify inlier count is sufficient (requires at least 12 inliers for stable pose tracking)
    if (inliers.size() < 12) {
        std::cout << "[PoseEstimator] FAILED: Insufficient tracking inliers (" 
                  << inliers.size() << ") to maintain stable localization." << std::endl;
        return false;
    }

    // 4. Convert Rotation Vector (rvec) to 3x3 Rotation Matrix (R) using Rodrigues' formula
    cv::Rodrigues(rvec, R);
    
    // Ensure rotation and translation matrices are double-precision (CV_64F)
    R.convertTo(R, CV_64F);
    tvec.convertTo(t, CV_64F);

    return true;
}
