#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class PoseEstimator {
public:
    // Constructor taking the camera intrinsic matrix K
    PoseEstimator(const cv::Mat& K);

    // Estimates the 3D camera pose (Rotation matrix R and Translation vector t) using PnP RANSAC.
    // Returns true on success, and fills in R, t, and the list of inlier feature indices.
    bool estimatePose(const std::vector<cv::Point3f>& points3D,
                      const std::vector<cv::Point2f>& points2D,
                      cv::Mat& R, cv::Mat& t,
                      std::vector<int>& inliers);

private:
    cv::Mat K_;
    cv::Mat dist_coeffs_; // Zero distortion assumed unless specified
};
