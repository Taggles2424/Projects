#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class PerspectiveVisualizer {
public:
    // Constructor setting up the 3D viewer window
    PerspectiveVisualizer(const std::string& window_name = "SLAM Real-Time 3D Perspective Viewer");

    // Renders the 3D scene from an orbiting virtual camera perspective
    void render3D(const std::vector<cv::Point3f>& map_points,
                  const std::vector<cv::Mat>& trajectory,
                  const std::vector<cv::Point3f>& dense_points = {},
                  const cv::Mat& current_R = cv::Mat(),
                  const cv::Mat& current_t = cv::Mat());

private:
    std::string win_name_;

    // Orbit parameters
    double angle_y_;     // Azimuth rotation angle (yaw)
    double angle_x_;     // Elevation angle (pitch)
    double distance_;    // Camera zoom distance
    
    // Viewer dimensions
    int width_;
    int height_;
};
