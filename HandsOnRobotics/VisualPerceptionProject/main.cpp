#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "FeatureTracker.hpp"
#include "MapInitializer.hpp"
#include "PoseEstimator.hpp"
#include "MapManager.hpp"
#include "Visualizer.hpp"
#include "PerspectiveVisualizer.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    cout << "=========================================================" << endl;
    cout << "      Modular Monocular Visual SLAM from Scratch" << endl;
    cout << "=========================================================" << endl;

    // 1. Default video path
    string video_path = "../../../temp_videos/ReadySetGo_1920x1080_30fps_x264_1200k.mp4";
    if (argc > 1) {
        video_path = argv[1];
    }

    VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open video file: " << video_path << endl;
        return -1;
    }

    // 2. Setup Camera Intrinsic Matrix K (Scaled for 800x450 resolution)
    Mat K = (Mat_<double>(3, 3) << 
        500.0,   0.0, 400.0,
          0.0, 500.0, 225.0,
          0.0,   0.0,   1.0);

    // 3. Initialize SLAM Modules
    FeatureTracker tracker(400, 15.0);
    MapManager map_manager(0.2, 8.0); // Trigger keyframe if translation > 0.2 units or rotation > 8 degrees
    Visualizer visualizer;
    PerspectiveVisualizer perspective_visualizer;
    
    vector<Mat> trajectory; // Stores all camera translation matrices in 3D world space

    // 4. Map Bootstrapping Phase
    cout << "System Bootstrapping: Tracking features to initialize 3D map..." << endl;
    Mat frame1;
    if (!cap.read(frame1)) {
        cerr << "ERROR: Failed to read first frame." << endl;
        return -1;
    }

    Mat display_frame1, gray1;
    double scale = 800.0 / frame1.cols;
    resize(frame1, display_frame1, Size(), scale, scale);
    cvtColor(display_frame1, gray1, COLOR_BGR2GRAY);

    vector<Point2f> points_init;
    tracker.processFrame(display_frame1, points_init);

    Mat prev_gray = gray1.clone();
    vector<Point2f> points_prev = points_init;

    Mat R_init = Mat::eye(3, 3, CV_64F);
    Mat t_init = Mat::zeros(3, 1, CV_64F);

    bool is_initialized = false;
    int bootstrap_frames = 30;
    int current_frame_id = 1;

    for (int i = 0; i < bootstrap_frames; ++i) {
        Mat frame;
        if (!cap.read(frame)) {
            cerr << "ERROR: Video ended before initialization completed." << endl;
            return -1;
        }
        current_frame_id++;

        Mat display_frame, gray;
        resize(frame, display_frame, Size(), scale, scale);
        cvtColor(display_frame, gray, COLOR_BGR2GRAY);

        vector<Point2f> points_next;
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(prev_gray, gray, points_prev, points_next, status, err, Size(31, 31), 4);

        vector<Point2f> good_prev, good_next;
        for (size_t j = 0; j < points_prev.size(); ++j) {
            if (status[j] &&
                points_next[j].x >= 0 && points_next[j].x < gray.cols &&
                points_next[j].y >= 0 && points_next[j].y < gray.rows) {
                good_prev.push_back(points_init[j]);
                good_next.push_back(points_next[j]);
            }
        }
        points_init = good_prev;
        points_prev = good_next;
        prev_gray = gray.clone();

        // Overlay status during initialization
        vector<bool> dummy_mask(points_prev.size(), false);
        visualizer.draw2D(display_frame, points_prev, dummy_mask, current_frame_id);
        waitKey(1);
    }

    // Run MapInitializer to bootstrap 3D coordinates
    MapInitializer initializer(K);
    vector<Point3f> points3D;
    vector<bool> inliers_init;
    Mat R_relative, t_relative;

    is_initialized = initializer.initialize(points_init, points_prev, R_relative, t_relative, points3D, inliers_init);

    if (!is_initialized) {
        cerr << "ERROR: Bootstrapping failed. Paradigm requires larger camera translation." << endl;
        return -1;
    }

    // Populate the first keyframe (Origin)
    vector<int> initial_3d_indices(points_init.size(), -1);
    vector<int> bootstrap_ids(points_init.size());
    for (size_t i = 0; i < points_init.size(); ++i) {
        bootstrap_ids[i] = static_cast<int>(i);
        if (inliers_init[i]) {
            int pt_idx = map_manager.addMapPoint(points3D[i]);
            initial_3d_indices[i] = pt_idx;
        }
    }

    // Add Keyframe 0 (Origin) and Keyframe 1 (Relative Translation)
    map_manager.addKeyframe(gray1, R_init, t_init, points_init, initial_3d_indices, bootstrap_ids);
    map_manager.addKeyframe(prev_gray, R_relative, t_relative, points_prev, initial_3d_indices, bootstrap_ids);

    trajectory.push_back(t_init);
    trajectory.push_back(-R_relative.t() * t_relative);

    // Reset active indices mapping relative to the bootstrap keyframe output
    vector<int> active_indices;
    for (size_t i = 0; i < points3D.size(); ++i) {
        active_indices.push_back(static_cast<int>(i));
    }

    // 5. Continuous Visual SLAM Tracking and Map Growth
    cout << endl << "Initialization Success! Entering continuous SLAM tracking..." << endl;
    PoseEstimator estimator(K);

    Mat frame;
    while (cap.read(frame)) {
        current_frame_id++;

        Mat display_frame, gray;
        resize(frame, display_frame, Size(), scale, scale);
        cvtColor(display_frame, gray, COLOR_BGR2GRAY);

        vector<Point2f> points_next;
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(prev_gray, gray, points_prev, points_next, status, err, Size(31, 31), 4);

        // Filter and compile active 3D-to-2D correspondences for PnP localization
        vector<Point3f> active_3d_points;
        vector<Point2f> active_2d_points;
        vector<Point2f> good_next;
        vector<int> good_indices;
        vector<int> good_next_to_active_pnp_map;

        for (size_t j = 0; j < points_prev.size(); ++j) {
            if (status[j] &&
                points_next[j].x >= 0 && points_next[j].x < gray.cols &&
                points_next[j].y >= 0 && points_next[j].y < gray.rows) {
                
                int good_next_idx = static_cast<int>(good_next.size());
                good_next.push_back(points_next[j]);
                good_indices.push_back(active_indices[j]);
                good_next_to_active_pnp_map.push_back(-1);

                int bootstrap_idx = active_indices[j];
                // Check if this feature was successfully triangulated
                if (inliers_init[bootstrap_idx]) {
                    good_next_to_active_pnp_map[good_next_idx] = static_cast<int>(active_2d_points.size());
                    active_3d_points.push_back(points3D[bootstrap_idx]);
                    active_2d_points.push_back(points_next[j]);
                }
            }
        }

        Mat R, t;
        vector<int> pnp_inliers;
        bool track_success = estimator.estimatePose(active_3d_points, active_2d_points, R, t, pnp_inliers);

        if (track_success) {
            Mat C = -R.t() * t;
            Mat C_prev = trajectory.back();
            double pose_jump = cv::norm(C - C_prev);
            if (pose_jump > 50.0) {
                cout << "[SLAM] Warning: Rejecting pose estimate on Frame " << current_frame_id 
                     << " due to physically impossible motion jump: " << pose_jump << " units (Threshold = 50.0)" << endl;
                track_success = false;
            }
        }

        vector<bool> inlier_mask(good_next.size(), false);

        if (track_success) {
            // Map camera origin coordinate C = -R^T * t
            Mat C = -R.t() * t;
            trajectory.push_back(C);

            // Filter out PnP RANSAC outliers from our active tracking set
            vector<bool> is_pnp_inlier(active_2d_points.size(), false);
            for (int idx : pnp_inliers) {
                is_pnp_inlier[idx] = true;
            }

            vector<Point2f> filtered_good_next;
            vector<int> filtered_good_indices;
            vector<bool> filtered_inlier_mask;

            for (size_t k = 0; k < good_next.size(); ++k) {
                int pnp_idx = good_next_to_active_pnp_map[k];
                if (pnp_idx != -1) {
                    // This feature has a 3D point and was used in PnP
                    if (is_pnp_inlier[pnp_idx]) {
                        // Keep it! It's a pristine inlier!
                        filtered_good_next.push_back(good_next[k]);
                        filtered_good_indices.push_back(good_indices[k]);
                        filtered_inlier_mask.push_back(true);
                    } else {
                        // Outlier! Purge it!
                        // Set its global status to untriangulated so we don't try to use its bad 3D coordinate again
                        int global_idx = good_indices[k];
                        inliers_init[global_idx] = false;
                        initial_3d_indices[global_idx] = -1;
                    }
                } else {
                    // This feature is untriangulated, keep it so we can triangulate it later!
                    filtered_good_next.push_back(good_next[k]);
                    filtered_good_indices.push_back(good_indices[k]);
                    filtered_inlier_mask.push_back(false);
                }
            }

            good_next = filtered_good_next;
            good_indices = filtered_good_indices;
            inlier_mask = filtered_inlier_mask;

            // Check if we need to trigger a new Keyframe to grow the map
            bool should_create_kf = map_manager.checkNewKeyframe(R, t, pnp_inliers.size(), active_3d_points.size());
            if (should_create_kf) {
                // 1. Extract new keypoints in the current frame to replace lost points and keep tracking strong
                std::vector<Point2f> new_features;
                tracker.processFrame(display_frame, new_features);
                
                // Merge old tracked features with newly detected ones BEFORE saving the keyframe
                // To keep indices aligned, we push back new features and update our indices list
                for (const auto& pt : new_features) {
                    // Make sure we don't add duplicate features
                    bool too_close = false;
                    for (const auto& existing : good_next) {
                        if (cv::norm(existing - pt) < 15.0) {
                            too_close = true;
                            break;
                        }
                    }
                    if (!too_close) {
                        good_next.push_back(pt);
                        // Add index mapping for new points
                        int new_idx = static_cast<int>(points3D.size());
                        points3D.push_back(Point3f(0, 0, 0)); // Dummy coordinate until triangulated
                        inliers_init.push_back(false);        // Un-inliered
                        initial_3d_indices.push_back(-1);     // Unmapped
                        good_indices.push_back(new_idx);
                        inlier_mask.push_back(false);
                    }
                }

                // 2. Compile 3D indices for the new keyframe
                vector<int> current_3d_indices(good_next.size(), -1);
                for (size_t k = 0; k < good_next.size(); ++k) {
                    int original_idx = good_indices[k];
                    if (inliers_init[original_idx]) {
                        // Retrieve index in global map point cloud
                        current_3d_indices[k] = initial_3d_indices[original_idx];
                    }
                }

                // 3. Add Keyframe to database (including the newly added features and their unique good_indices as feature_ids)
                map_manager.addKeyframe(gray, R, t, good_next, current_3d_indices, good_indices);

                // 4. Triangulate new features between last two keyframes to grow the sparse point cloud
                int last_kf_idx = static_cast<int>(map_manager.getKeyframes().size()) - 1;
                map_manager.triangulateNewPoints(last_kf_idx - 1, last_kf_idx, K);

                // Update our local tracking variables with the newly triangulated 3D points
                auto& kf_last = map_manager.getKeyframesRef().back();
                const auto& map_points = map_manager.getMapPoints();
                for (size_t k = 0; k < kf_last.point3D_indices.size(); ++k) {
                    int pt_idx = kf_last.point3D_indices[k];
                    if (pt_idx != -1) {
                        int original_idx = good_indices[k];
                        points3D[original_idx] = map_points[pt_idx];
                        inliers_init[original_idx] = true;
                        initial_3d_indices[original_idx] = pt_idx;
                    }
                }
            }
        } else {
            cout << "[SLAM] Warning: Camera localization lost on Frame " << current_frame_id << endl;
        }

        // Draw 2D & 3D real-time visualizers
        visualizer.draw2D(display_frame, good_next, inlier_mask, current_frame_id);
        if (current_frame_id % 5 == 0) {
            // Generate dense 3D reconstruction points for the environment
            std::vector<cv::Point3f> dense_points = map_manager.generateDenseMap();
            visualizer.draw3D(map_manager.getMapPoints(), trajectory, dense_points);
            perspective_visualizer.render3D(map_manager.getMapPoints(), trajectory, dense_points, R, t);
        }

        // Control frame rate playback and exit keys (reduced to 1ms delay for max performance)
        char key = (char)waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') { // ESC or Q key to exit SLAM
            cout << "SLAM execution terminated by user." << endl;
            break;
        }

        // Prepare for next iteration
        points_prev = good_next;
        active_indices = good_indices;
        prev_gray = gray.clone();
    }

    cout << "=========================================================" << endl;
    cout << "            SLAM Run Completed Successfully!" << endl;
    cout << "=========================================================" << endl;
    cout << "Total Keyframes Stored: " << map_manager.getKeyframes().size() << endl;
    cout << "Total 3D Points Mapped: " << map_manager.getMapPoints().size() << endl;
    
    cout << endl << ">>> SLAM Completed! Press any key in the visualizer windows to close and exit..." << endl;
    cv::waitKey(0);
    
    cv::destroyAllWindows();
    return 0;
}
