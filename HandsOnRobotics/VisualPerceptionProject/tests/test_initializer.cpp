#include "FeatureTracker.hpp"
#include "MapInitializer.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "   Map Initializer Intermediate Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // Default video path
    std::string video_path = "../../../temp_videos/ReadySetGo_1920x1080_30fps_x264_1200k.mp4";
    if (argc > 1) {
        video_path = argv[1];
    }

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open video file: " << video_path << std::endl;
        return -1;
    }

    // 1. Setup Camera Intrinsic Matrix K (Scaled for 800x450 resolution)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 
        500.0,   0.0, 400.0,
          0.0, 500.0, 225.0,
          0.0,   0.0,   1.0);

    std::cout << "Camera Intrinsic Matrix K:" << std::endl << K << std::endl << std::endl;

    // 2. Load the first frame (Frame 0)
    cv::Mat frame1;
    if (!cap.read(frame1)) {
        std::cerr << "ERROR: Could not read first frame." << std::endl;
        return -1;
    }

    // Resize frame to 800x450 and convert to grayscale
    cv::Mat display_frame1, gray1;
    double scale = 800.0 / frame1.cols;
    cv::resize(frame1, display_frame1, cv::Size(), scale, scale);
    cv::cvtColor(display_frame1, gray1, cv::COLOR_BGR2GRAY);

    // Detect initial features using FAST detector
    std::vector<cv::KeyPoint> keypoints1;
    cv::FAST(gray1, keypoints1, 20, true);
    
    std::vector<cv::Point2f> points_init;
    for (const auto& kp : keypoints1) {
        points_init.push_back(kp.pt);
    }
    
    // Limit to top 500 strongest features
    if (points_init.size() > 500) {
        points_init.resize(500);
    }

    std::cout << "Initialized " << points_init.size() << " tracking points on Frame 1." << std::endl;

    // 3. Track these EXACT features over 30 frames using manual Optical Flow to preserve 1-to-1 correspondences
    cv::Mat prev_gray = gray1.clone();
    std::vector<cv::Point2f> points_prev = points_init;

    cv::Mat frame2;
    int baseline_frames = 30;
    std::cout << "Tracking features over " << baseline_frames << " frames to build baseline..." << std::endl;
    
    for (int i = 0; i < baseline_frames; ++i) {
        if (!cap.read(frame2)) {
            std::cerr << "ERROR: Ended early at frame " << i << std::endl;
            return -1;
        }
        
        cv::Mat display_frame2, gray2;
        cv::resize(frame2, display_frame2, cv::Size(), scale, scale);
        cv::cvtColor(display_frame2, gray2, cv::COLOR_BGR2GRAY);

        if (points_prev.empty()) {
            std::cerr << "ERROR: All tracking points lost!" << std::endl;
            return -1;
        }

        std::vector<cv::Point2f> points_next;
        std::vector<uchar> status;
        std::vector<float> err;

        // Perform Lucas-Kanade optical flow
        cv::calcOpticalFlowPyrLK(prev_gray, gray2, points_prev, points_next, status, err, cv::Size(21, 21), 3);

        // Filter out failed tracking points in BOTH lists to maintain exact indices correspondence
        std::vector<cv::Point2f> good_prev;
        std::vector<cv::Point2f> good_next;
        for (size_t j = 0; j < points_prev.size(); ++j) {
            if (status[j] &&
                points_next[j].x >= 0 && points_next[j].x < gray2.cols &&
                points_next[j].y >= 0 && points_next[j].y < gray2.rows) {
                good_prev.push_back(points_init[j]); // Note: we store original point position from Frame 1
                good_next.push_back(points_next[j]); // and current point position
            }
        }

        // Update lists for next iteration
        points_init = good_prev;
        points_prev = good_next;
        prev_gray = gray2.clone();
    }

    std::cout << "Feature tracking complete. Surviving corresponding points: " << points_init.size() << std::endl;

    // 4. Run MapInitializer using the surviving 1-to-1 correspondences
    MapInitializer initializer(K);
    cv::Mat R, t;
    std::vector<cv::Point3f> points3D;
    std::vector<bool> inliers;

    std::cout << "Attempting 3D Map Triangulation..." << std::endl;
    bool success = initializer.initialize(points_init, points_prev, R, t, points3D, inliers);

    if (success) {
        std::cout << std::endl << "=== BOOTSTRAP INITIALIZATION SUCCESSFUL ===" << std::endl;
        std::cout << "Relative Rotation Matrix R:" << std::endl << R << std::endl << std::endl;
        std::cout << "Relative Translation Vector t:" << std::endl << t << std::endl << std::endl;

        // Print first 10 successfully triangulated 3D points
        std::cout << "Sample Triangulated 3D Points (First 10):" << std::endl;
        int count = 0;
        for (size_t i = 0; i < points3D.size(); ++i) {
            if (inliers[i]) {
                std::cout << "  Point " << count << ": [" 
                          << points3D[i].x << ", " 
                          << points3D[i].y << ", " 
                          << points3D[i].z << "]" << std::endl;
                count++;
                if (count >= 10) break;
            }
        }
    } else {
        std::cout << std::endl << "=== BOOTSTRAP INITIALIZATION FAILED ===" << std::endl;
        std::cout << "Not enough parallax or inliers to reconstruct the 3D scene." << std::endl;
    }

    return 0;
}
