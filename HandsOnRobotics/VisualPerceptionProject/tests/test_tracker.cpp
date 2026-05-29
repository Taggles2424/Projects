#include "FeatureTracker.hpp"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "   Feature Tracker Intermediate Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // Default video path
    std::string video_path = "../../temp_videos/ReadySetGo_1920x1080_30fps_x264_1200k.mp4";
    
    // Allow overriding via command line argument
    if (argc > 1) {
        video_path = argv[1];
    }

    std::cout << "Opening video file: " << video_path << std::endl;
    cv::VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open video file: " << video_path << std::endl;
        std::cerr << "Usage: ./test_tracker [path_to_video]" << std::endl;
        return -1;
    }

    // Create the tracker
    FeatureTracker tracker(500, 15.0);
    std::vector<cv::Point2f> tracked_points;

    cv::Mat frame;
    std::string window_name = "Feature Tracker Output";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);

    int frame_count = 0;
    while (cap.read(frame)) {
        frame_count++;

        // Resize large frames to make rendering fast
        cv::Mat display_frame;
        if (frame.cols > 800) {
            double scale = 800.0 / frame.cols;
            cv::resize(frame, display_frame, cv::Size(), scale, scale);
        } else {
            display_frame = frame.clone();
        }

        // Run the tracker on the frame
        tracker.processFrame(display_frame, tracked_points);

        // Draw the tracked features as colored circles
        for (const auto& pt : tracked_points) {
            cv::circle(display_frame, pt, 4, cv::Scalar(0, 255, 0), -1); // Green circle
        }

        // Overlay status text
        cv::putText(display_frame, "Frame: " + std::to_string(frame_count), cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(display_frame, "Features: " + std::to_string(tracked_points.size()), cv::Point(20, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Display frame
        cv::imshow(window_name, display_frame);

        // Pump the GTK event queue (30ms delay to control playback speed)
        char key = (char)cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') { // ESC or Q key to exit early
            std::cout << "User terminated tracking test early." << std::endl;
            break;
        }
    }

    std::cout << "Tracking finished! Total frames processed: " << frame_count << std::endl;
    cv::destroyAllWindows();
    return 0;
}
