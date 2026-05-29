#include "Visualizer.hpp"
#include <iostream>

Visualizer::Visualizer(const std::string& window_2d_name, const std::string& window_3d_name)
    : win_2d_(window_2d_name), win_3d_(window_3d_name) {
    
    // Create GUI windows with resizing enabled
    cv::namedWindow(win_2d_, cv::WINDOW_NORMAL);
    cv::namedWindow(win_3d_, cv::WINDOW_NORMAL);
    
    // Zoom factor mapping 3D metric coordinates to 2D canvas pixels
    zoom_ = 10.0; 
}

void Visualizer::draw2D(const cv::Mat& frame, 
                        const std::vector<cv::Point2f>& points2D,
                        const std::vector<bool>& inlier_mask, 
                        int frame_id) {
    cv::Mat display = frame.clone();

    // Color-code the features
    int tracked_count = 0;
    for (size_t i = 0; i < points2D.size(); ++i) {
        if (inlier_mask[i]) {
            // Active 3D inlier tracked in the PnP pose = Green
            cv::circle(display, points2D[i], 4, cv::Scalar(0, 255, 0), -1);
            tracked_count++;
        } else {
            // General tracked search feature = Red
            cv::circle(display, points2D[i], 2, cv::Scalar(0, 0, 255), -1);
        }
    }

    // Overlay visual text dashboard
    cv::putText(display, "Live Tracking Feed", cv::Point(20, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    cv::putText(display, "Frame: " + std::to_string(frame_id), cv::Point(20, 65),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(display, "Active 3D Landmarks: " + std::to_string(tracked_count), cv::Point(20, 95),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

    cv::imshow(win_2d_, display);
}

void Visualizer::draw3D(const std::vector<cv::Point3f>& map_points,
                        const std::vector<cv::Mat>& trajectory_t,
                        const std::vector<cv::Point3f>& dense_points) {
    // 1. Create a black double-panel canvas (800x600 pixels)
    cv::Mat canvas = cv::Mat::zeros(500, 800, CV_8UC3);

    // Draw dividers
    cv::line(canvas, cv::Point(400, 0), cv::Point(400, 500), cv::Scalar(100, 100, 100), 2);

    // Left Panel: Top-Down View (X-Z plane). Center is (200, 250)
    cv::Point center1(200, 250);
    cv::line(canvas, cv::Point(0, 250), cv::Point(400, 250), cv::Scalar(40, 40, 40), 1);
    cv::line(canvas, cv::Point(200, 0), cv::Point(200, 500), cv::Scalar(40, 40, 40), 1);
    cv::putText(canvas, "Top-Down View (X - Z)", cv::Point(15, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 2);

    // Right Panel: Side View (Z-Y plane). Center is (600, 250)
    cv::Point center2(600, 250);
    cv::line(canvas, cv::Point(400, 250), cv::Point(800, 250), cv::Scalar(40, 40, 40), 1);
    cv::line(canvas, cv::Point(600, 0), cv::Point(600, 500), cv::Scalar(40, 40, 40), 1);
    cv::putText(canvas, "Side View (Depth Z - Y)", cv::Point(415, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 2);

    // 2. Draw global dense 3D points in a soft cyan/blue color to represent surfaces
    for (const auto& pt : dense_points) {
        // Top-Down View
        int u1 = center1.x + static_cast<int>(pt.x * zoom_);
        int v1 = center1.y - static_cast<int>(pt.z * zoom_);

        if (u1 >= 0 && u1 < 400 && v1 >= 0 && v1 < 500) {
            canvas.at<cv::Vec3b>(v1, u1) = cv::Vec3b(180, 120, 50); // Soft cyan
        }

        // Side View
        int u2 = center2.x + static_cast<int>(pt.z * zoom_);
        int v2 = center2.y + static_cast<int>(pt.y * zoom_);

        if (u2 >= 400 && u2 < 800 && v2 >= 0 && v2 < 500) {
            canvas.at<cv::Vec3b>(v2, u2) = cv::Vec3b(180, 120, 50);
        }
    }

    // 3. Draw global sparse 3D MapPoints
    for (const auto& pt : map_points) {
        // Top-Down View: u = center1.x + x * zoom_, v = center1.y - z * zoom_
        int u1 = center1.x + static_cast<int>(pt.x * zoom_);
        int v1 = center1.y - static_cast<int>(pt.z * zoom_);

        if (u1 >= 0 && u1 < 400 && v1 >= 0 && v1 < 500) {
            cv::circle(canvas, cv::Point(u1, v1), 1, cv::Scalar(255, 255, 255), -1); // White point
        }

        // Side View: u = center2.x + z * zoom_, v = center2.y + y * zoom_
        int u2 = center2.x + static_cast<int>(pt.z * zoom_);
        int v2 = center2.y + static_cast<int>(pt.y * zoom_);

        if (u2 >= 400 && u2 < 800 && v2 >= 0 && v2 < 500) {
            cv::circle(canvas, cv::Point(u2, v2), 1, cv::Scalar(255, 255, 255), -1); // White point
        }
    }

    // 3. Draw Camera Trajectory Path
    if (trajectory_t.size() > 1) {
        for (size_t i = 1; i < trajectory_t.size(); ++i) {
            // Get previous and current translation vectors
            cv::Mat t1 = trajectory_t[i - 1];
            cv::Mat t2 = trajectory_t[i];

            // Project coordinates for Top-Down View
            int u1_1 = center1.x + static_cast<int>(t1.at<double>(0, 0) * zoom_);
            int v1_1 = center1.y - static_cast<int>(t1.at<double>(2, 0) * zoom_);
            int u1_2 = center1.x + static_cast<int>(t2.at<double>(0, 0) * zoom_);
            int v1_2 = center1.y - static_cast<int>(t2.at<double>(2, 0) * zoom_);

            if (u1_1 >= 0 && u1_1 < 400 && v1_1 >= 0 && v1_1 < 500 &&
                u1_2 >= 0 && u1_2 < 400 && v1_2 >= 0 && v1_2 < 500) {
                // Connect with a smooth cyan line
                cv::line(canvas, cv::Point(u1_1, v1_1), cv::Point(u1_2, v1_2), cv::Scalar(255, 255, 0), 2);
            }

            // Project coordinates for Side View
            int u2_1 = center2.x + static_cast<int>(t1.at<double>(2, 0) * zoom_);
            int v2_1 = center2.y + static_cast<int>(t1.at<double>(1, 0) * zoom_);
            int u2_2 = center2.x + static_cast<int>(t2.at<double>(2, 0) * zoom_);
            int v2_2 = center2.y + static_cast<int>(t2.at<double>(1, 0) * zoom_);

            if (u2_1 >= 400 && u2_1 < 800 && v2_1 >= 0 && v2_1 < 500 &&
                u2_2 >= 400 && u2_2 < 800 && v2_2 >= 0 && v2_2 < 500) {
                // Connect with a smooth cyan line
                cv::line(canvas, cv::Point(u2_1, v2_1), cv::Point(u2_2, v2_2), cv::Scalar(255, 255, 0), 2);
            }
        }
    }

    // 4. Draw Current Camera Position as a red indicator
    if (!trajectory_t.empty()) {
        cv::Mat t_curr = trajectory_t.back();
        
        int u1 = center1.x + static_cast<int>(t_curr.at<double>(0, 0) * zoom_);
        int v1 = center1.y - static_cast<int>(t_curr.at<double>(2, 0) * zoom_);
        if (u1 >= 0 && u1 < 400 && v1 >= 0 && v1 < 500) {
            cv::circle(canvas, cv::Point(u1, v1), 6, cv::Scalar(0, 0, 255), -1); // Red dot
            cv::circle(canvas, cv::Point(u1, v1), 8, cv::Scalar(0, 0, 255), 1);  // Outer ring
        }

        int u2 = center2.x + static_cast<int>(t_curr.at<double>(2, 0) * zoom_);
        int v2 = center2.y + static_cast<int>(t_curr.at<double>(1, 0) * zoom_);
        if (u2 >= 400 && u2 < 800 && v2 >= 0 && v2 < 500) {
            cv::circle(canvas, cv::Point(u2, v2), 6, cv::Scalar(0, 0, 255), -1); // Red dot
            cv::circle(canvas, cv::Point(u2, v2), 8, cv::Scalar(0, 0, 255), 1);  // Outer ring
        }
    }

    cv::imshow(win_3d_, canvas);
}
