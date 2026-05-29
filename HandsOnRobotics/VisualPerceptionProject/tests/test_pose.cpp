#include "FeatureTracker.hpp"
#include "MapInitializer.hpp"
#include "PoseEstimator.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "     Pose Estimator Intermediate Test" << std::endl;
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

    // 2. Map Bootstrapping Phase (Frame 1 to Frame 30)
    cv::Mat frame1;
    if (!cap.read(frame1)) {
        std::cerr << "ERROR: Could not read first frame." << std::endl;
        return -1;
    }

    cv::Mat display_frame1, gray1;
    double scale = 800.0 / frame1.cols;
    cv::resize(frame1, display_frame1, cv::Size(), scale, scale);
    cv::cvtColor(display_frame1, gray1, cv::COLOR_BGR2GRAY);

    // Detect initial features on Frame 1
    std::vector<cv::KeyPoint> keypoints1;
    cv::FAST(gray1, keypoints1, 20, true);
    
    std::vector<cv::Point2f> points_init;
    for (const auto& kp : keypoints1) {
        points_init.push_back(kp.pt);
    }
    if (points_init.size() > 500) {
        points_init.resize(500);
    }

    cv::Mat prev_gray = gray1.clone();
    std::vector<cv::Point2f> points_prev = points_init;

    cv::Mat frame2;
    int baseline_frames = 30;
    std::cout << "Bootstrapping: Tracking features over " << baseline_frames << " frames..." << std::endl;
    
    for (int i = 0; i < baseline_frames; ++i) {
        if (!cap.read(frame2)) {
            std::cerr << "ERROR: Ended early during bootstrap at frame " << i << std::endl;
            return -1;
        }
        
        cv::Mat display_frame2, gray2;
        cv::resize(frame2, display_frame2, cv::Size(), scale, scale);
        cv::cvtColor(display_frame2, gray2, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> points_next;
        std::vector<uchar> status;
        std::vector<float> err;

        cv::calcOpticalFlowPyrLK(prev_gray, gray2, points_prev, points_next, status, err, cv::Size(21, 21), 3);

        std::vector<cv::Point2f> good_prev;
        std::vector<cv::Point2f> good_next;

        for (size_t j = 0; j < points_prev.size(); ++j) {
            if (status[j] &&
                points_next[j].x >= 0 && points_next[j].x < gray2.cols &&
                points_next[j].y >= 0 && points_next[j].y < gray2.rows) {
                good_prev.push_back(points_init[j]);
                good_next.push_back(points_next[j]);
            }
        }
        points_init = good_prev;
        points_prev = good_next;
        prev_gray = gray2.clone();
    }

    // Run MapInitializer to get 3D coordinates
    MapInitializer initializer(K);
    cv::Mat R_init, t_init;
    std::vector<cv::Point3f> points3D;
    std::vector<bool> inliers_init;
    
    // We pass points_init and points_prev which have matching indices
    bool init_success = initializer.initialize(points_init, points_prev, R_init, t_init, points3D, inliers_init);
    if (!init_success) {
        std::cerr << "ERROR: Map bootstrapping failed! Try a different video section or threshold." << std::endl;
        return -1;
    }

    // Initialize active_indices to keep track of point indices relative to the bootstrap output (points3D)
    std::vector<int> active_indices;
    for (size_t i = 0; i < points3D.size(); ++i) {
        active_indices.push_back(static_cast<int>(i));
    }

    // 3. Real-Time Tracking Phase (Frame 31 to Frame 60)
    std::cout << std::endl << "Tracking Camera Pose (Visual Odometry) Frame-by-Frame..." << std::endl;
    PoseEstimator estimator(K);

    int tracking_frames = 30;
    for (int i = 0; i < tracking_frames; ++i) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            break;
        }

        cv::Mat display_frame, gray;
        cv::resize(frame, display_frame, cv::Size(), scale, scale);
        cv::cvtColor(display_frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> points_next;
        std::vector<uchar> status;
        std::vector<float> err;

        // Track keypoints from the previous frame to the current frame
        cv::calcOpticalFlowPyrLK(prev_gray, gray, points_prev, points_next, status, err, cv::Size(21, 21), 3);

        // Filter the tracked points, keeping corresponding 3D points in the map
        std::vector<cv::Point3f> active_3d_points;
        std::vector<cv::Point2f> active_2d_points;
        std::vector<cv::Point2f> good_next;
        std::vector<int> good_indices;

        for (size_t j = 0; j < points_prev.size(); ++j) {
            if (status[j] &&
                points_next[j].x >= 0 && points_next[j].x < gray.cols &&
                points_next[j].y >= 0 && points_next[j].y < gray.rows) {
                
                good_next.push_back(points_next[j]);
                good_indices.push_back(active_indices[j]);

                // MapInitializer filled 'points3D' and 'inliers_init' in corresponding index to 'points_init'
                // Track 'active_indices[j]' relative to the bootstrap output
                int bootstrap_idx = active_indices[j];
                if (inliers_init[bootstrap_idx]) {
                    active_3d_points.push_back(points3D[bootstrap_idx]);
                    active_2d_points.push_back(points_next[j]);
                }
            }
        }

        cv::Mat R, t;
        std::vector<int> pnp_inliers;

        // Estimate current camera pose relative to our map
        bool track_success = estimator.estimatePose(active_3d_points, active_2d_points, R, t, pnp_inliers);

        if (track_success) {
            std::cout << "  Frame " << (baseline_frames + i + 1) << " | "
                      << "Tracked: " << pnp_inliers.size() << " / " << active_3d_points.size() << " features | "
                      << "Pose t: [" << t.at<double>(0, 0) << ", " 
                                     << t.at<double>(1, 0) << ", " 
                                     << t.at<double>(2, 0) << "]" << std::endl;
        } else {
            std::cout << "  Frame " << (baseline_frames + i + 1) << " | FAILED TO LOCALISE CAMERA!" << std::endl;
        }

        // Prepare for next iteration
        points_prev = good_next;
        active_indices = good_indices;
        prev_gray = gray.clone();
    }

    std::cout << std::endl << "Visual Odometry Tracking Test Complete!" << std::endl;
    return 0;
}
