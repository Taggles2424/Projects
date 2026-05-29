#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class Visualizer {
public:
    // Constructor setting up the GUI window names
    Visualizer(const std::string& window_2d_name = "SLAM 2D Feature Tracking",
               const std::string& window_3d_name = "SLAM 3D Map & Trajectory");

    // Renders the 2D frame with color-coded features (green for tracked 3D inliers, red for others)
    void draw2D(const cv::Mat& frame, 
                const std::vector<cv::Point2f>& points2D,
                const std::vector<bool>& inlier_mask, 
                int frame_id);

    // Custom 3D Projection Engine: Projects global 3D points and camera poses
    // onto a virtual 2D canvas (providing Perspective, Top-Down, and Side views) and displays it.
    void draw3D(const std::vector<cv::Point3f>& map_points,
                const std::vector<cv::Mat>& trajectory_t,
                const std::vector<cv::Point3f>& dense_points = {});

private:
    std::string win_2d_;
    std::string win_3d_;
    
    // Virtual viewer camera parameters
    double zoom_;
    int x_offset_;
    int y_offset_;
};
