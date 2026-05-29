#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class MapInitializer {
public:
    // Constructor taking the camera intrinsic matrix K
    MapInitializer(const cv::Mat& K);

    // Attempts to bootstrap the 3D map from two sets of tracked 2D features.
    // Returns true on success, filling in rotation R, translation t, 3D point cloud, and an inlier mask.
    bool initialize(const std::vector<cv::Point2f>& points1,
                    const std::vector<cv::Point2f>& points2,
                    cv::Mat& R, cv::Mat& t,
                    std::vector<cv::Point3f>& points3D,
                    std::vector<bool>& inlier_mask);

private:
    cv::Mat K_;
};
