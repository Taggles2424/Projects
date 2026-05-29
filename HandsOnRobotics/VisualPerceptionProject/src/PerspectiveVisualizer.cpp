#include "PerspectiveVisualizer.hpp"
#include <iostream>
#include <cmath>

PerspectiveVisualizer::PerspectiveVisualizer(const std::string& window_name)
    : win_name_(window_name), angle_y_(45.0), angle_x_(-30.0), distance_(8.0),
      width_(800), height_(600) {
    
    cv::namedWindow(win_name_, cv::WINDOW_NORMAL);
}

void PerspectiveVisualizer::render3D(const std::vector<cv::Point3f>& map_points,
                                     const std::vector<cv::Mat>& trajectory,
                                     const std::vector<cv::Point3f>& dense_points,
                                     const cv::Mat& current_R,
                                     const cv::Mat& current_t) {
    
    // 1. Create a black canvas (800x600 pixels)
    cv::Mat canvas = cv::Mat::zeros(height_, width_, CV_8UC3);

    // 2. Increment rotation angle slightly at each frame for smooth 3D auto-orbiting animation
    angle_y_ += 0.4;
    if (angle_y_ >= 360.0) angle_y_ -= 360.0;

    // Convert angles to radians
    double theta = angle_y_ * CV_PI / 180.0;
    double phi = angle_x_ * CV_PI / 180.0;

    // 3. Compute map center to act as our orbiting pivot target
    cv::Point3f pivot(0.0f, 0.0f, 6.0f); // Default target if map is small
    if (map_points.size() > 10) {
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& pt : map_points) {
            sum_x += pt.x;
            sum_y += pt.y;
            sum_z += pt.z;
        }
        pivot.x = static_cast<float>(sum_x / map_points.size());
        pivot.y = static_cast<float>(sum_y / map_points.size());
        pivot.z = static_cast<float>(sum_z / map_points.size());
    }

    // 4. Position of virtual viewer camera in World Coordinates
    double cv_x = pivot.x + distance_ * cos(phi) * sin(theta);
    double cv_y = pivot.y + distance_ * sin(phi);
    double cv_z = pivot.z - distance_ * cos(phi) * cos(theta);

    cv::Point3f C_v(static_cast<float>(cv_x), static_cast<float>(cv_y), static_cast<float>(cv_z));

    // 5. Compute Look-At Rotation Matrix R_view (World-to-Camera)
    // Forward vector F points from camera to pivot
    cv::Point3f F = pivot - C_v;
    float norm_F = std::sqrt(F.x * F.x + F.y * F.y + F.z * F.z);
    if (norm_F > 1e-5) { F.x /= norm_F; F.y /= norm_F; F.z /= norm_F; }

    // World Up = (0, -1, 0)
    cv::Point3f Y_up(0.0f, -1.0f, 0.0f);

    // Right = Y_up x F
    cv::Point3f R_vec;
    R_vec.x = Y_up.y * F.z - Y_up.z * F.y;
    R_vec.y = Y_up.z * F.x - Y_up.x * F.z;
    R_vec.z = Y_up.x * F.y - Y_up.y * F.x;
    float norm_R = std::sqrt(R_vec.x * R_vec.x + R_vec.y * R_vec.y + R_vec.z * R_vec.z);
    if (norm_R > 1e-5) { R_vec.x /= norm_R; R_vec.y /= norm_R; R_vec.z /= norm_R; }

    // Actual Up = F x Right
    cv::Point3f U_vec;
    U_vec.x = F.y * R_vec.z - F.z * R_vec.y;
    U_vec.y = F.z * R_vec.x - F.x * R_vec.z;
    U_vec.z = F.x * R_vec.y - F.y * R_vec.x;

    // Build rotation and translation matrices
    cv::Mat R_view = (cv::Mat_<double>(3, 3) <<
        R_vec.x, R_vec.y, R_vec.z,
        U_vec.x, U_vec.y, U_vec.z,
        F.x,     F.y,     F.z);

    cv::Mat C_v_mat = (cv::Mat_<double>(3, 1) << C_v.x, C_v.y, C_v.z);
    cv::Mat t_view = -R_view * C_v_mat;

    // 6. Perspective Projection Engine Parameters
    double f_view = 600.0; // Virtual focal length
    double cx = width_ / 2.0;
    double cy = height_ / 2.0;

    auto projectPoint = [&](const cv::Point3f& pt, cv::Point& pixel) -> bool {
        cv::Mat pt_w = (cv::Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
        cv::Mat pt_c = R_view * pt_w + t_view;

        double zc = pt_c.at<double>(2, 0);
        if (zc < 0.1) return false;

        double xc = pt_c.at<double>(0, 0);
        double yc = pt_c.at<double>(1, 0);

        pixel.x = static_cast<int>(f_view * (xc / zc) + cx);
        pixel.y = static_cast<int>(f_view * (yc / zc) + cy);

        return (pixel.x >= 0 && pixel.x < width_ && pixel.y >= 0 && pixel.y < height_);
    };

    // 7. Draw RGB Coordinate Axes at the World Origin (0,0,0)
    cv::Point p_origin, p_x, p_y, p_z;
    double axis_len = 1.0;
    if (projectPoint(cv::Point3f(0, 0, 0), p_origin)) {
        if (projectPoint(cv::Point3f(axis_len, 0, 0), p_x)) {
            cv::line(canvas, p_origin, p_x, cv::Scalar(0, 0, 255), 2); // X-axis = Red
        }
        if (projectPoint(cv::Point3f(0, -axis_len, 0), p_y)) { // -Y points up
            cv::line(canvas, p_origin, p_y, cv::Scalar(0, 255, 0), 2); // Y-axis = Green
        }
        if (projectPoint(cv::Point3f(0, 0, axis_len), p_z)) {
            cv::line(canvas, p_origin, p_z, cv::Scalar(255, 0, 0), 2); // Z-axis = Blue
        }
    }

    // 8. Draw global dense 3D points (rendered first as soft cyan-blue background points)
    for (const auto& pt : dense_points) {
        cv::Point pixel;
        if (projectPoint(pt, pixel)) {
            canvas.at<cv::Vec3b>(pixel.y, pixel.x) = cv::Vec3b(200, 150, 60); // Soft Cyan
        }
    }

    // 9. Draw global sparse 3D MapPoints (bright white dots)
    for (const auto& pt : map_points) {
        cv::Point pixel;
        if (projectPoint(pt, pixel)) {
            cv::circle(canvas, pixel, 2, cv::Scalar(255, 255, 255), -1);
        }
    }

    // 10. Draw Camera Trajectory Path
    if (trajectory.size() > 1) {
        for (size_t i = 1; i < trajectory.size(); ++i) {
            cv::Mat t1 = trajectory[i - 1];
            cv::Mat t2 = trajectory[i];

            cv::Point3f pt1(t1.at<double>(0, 0), t1.at<double>(1, 0), t1.at<double>(2, 0));
            cv::Point3f pt2(t2.at<double>(0, 0), t2.at<double>(1, 0), t2.at<double>(2, 0));

            cv::Point pixel1, pixel2;
            if (projectPoint(pt1, pixel1) && projectPoint(pt2, pixel2)) {
                cv::line(canvas, pixel1, pixel2, cv::Scalar(0, 255, 255), 2); // Yellow path
            }
        }
    }

    // 11. Draw 3D Wireframe Camera Frustum at the current camera pose
    if (!current_R.empty() && !current_t.empty() && current_R.rows == 3 && current_R.cols == 3) {
        cv::Mat R_slam_inv = current_R.t();
        cv::Mat C_slam_mat = -R_slam_inv * current_t;
        
        cv::Point3f C_slam(
            static_cast<float>(C_slam_mat.at<double>(0, 0)),
            static_cast<float>(C_slam_mat.at<double>(1, 0)),
            static_cast<float>(C_slam_mat.at<double>(2, 0))
        );

        // Derive orientation vectors of camera frame
        cv::Point3f f_slam(
            static_cast<float>(R_slam_inv.at<double>(0, 2)),
            static_cast<float>(R_slam_inv.at<double>(1, 2)),
            static_cast<float>(R_slam_inv.at<double>(2, 2))
        );
        cv::Point3f r_slam(
            static_cast<float>(R_slam_inv.at<double>(0, 0)),
            static_cast<float>(R_slam_inv.at<double>(1, 0)),
            static_cast<float>(R_slam_inv.at<double>(2, 0))
        );
        cv::Point3f u_slam(
            static_cast<float>(R_slam_inv.at<double>(0, 1)),
            static_cast<float>(R_slam_inv.at<double>(1, 1)),
            static_cast<float>(R_slam_inv.at<double>(2, 1))
        );

        // Define wireframe frustum corners
        double d_f = 0.4;
        cv::Point3f v1 = C_slam + static_cast<float>(d_f) * f_slam - 0.2f * r_slam - 0.15f * u_slam;
        cv::Point3f v2 = C_slam + static_cast<float>(d_f) * f_slam + 0.2f * r_slam - 0.15f * u_slam;
        cv::Point3f v3 = C_slam + static_cast<float>(d_f) * f_slam + 0.2f * r_slam + 0.15f * u_slam;
        cv::Point3f v4 = C_slam + static_cast<float>(d_f) * f_slam - 0.2f * r_slam + 0.15f * u_slam;

        cv::Point p_c, p_v1, p_v2, p_v3, p_v4;
        if (projectPoint(C_slam, p_c) &&
            projectPoint(v1, p_v1) &&
            projectPoint(v2, p_v2) &&
            projectPoint(v3, p_v3) &&
            projectPoint(v4, p_v4)) {
            
            // Connect apex to corners
            cv::line(canvas, p_c, p_v1, cv::Scalar(0, 0, 255), 2); // Red frustum lines
            cv::line(canvas, p_c, p_v2, cv::Scalar(0, 0, 255), 2);
            cv::line(canvas, p_c, p_v3, cv::Scalar(0, 0, 255), 2);
            cv::line(canvas, p_c, p_v4, cv::Scalar(0, 0, 255), 2);

            // Connect corners to form image plane
            cv::line(canvas, p_v1, p_v2, cv::Scalar(0, 0, 255), 2);
            cv::line(canvas, p_v2, p_v3, cv::Scalar(0, 0, 255), 2);
            cv::line(canvas, p_v3, p_v4, cv::Scalar(0, 0, 255), 2);
            cv::line(canvas, p_v4, p_v1, cv::Scalar(0, 0, 255), 2);
        }
    }

    // 12. Overlay Information Text
    cv::putText(canvas, "SLAM 3D Perspective Orbit Viewer", cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(canvas, "Red: camera frustum | Yellow: path", cv::Point(20, 65),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    cv::putText(canvas, "Origin Axes: X(Red) Y(Green) Z(Blue)", cv::Point(20, 95),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

    cv::imshow(win_name_, canvas);
}
