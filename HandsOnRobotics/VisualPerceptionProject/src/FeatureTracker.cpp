#include "FeatureTracker.hpp"

#include <iostream>

FeatureTracker::FeatureTracker(int max_features, double min_distance)
    : max_features_(max_features), min_distance_(min_distance) {
    // Create a FAST feature detector with a threshold of 20 and non-maximum suppression enabled
    detector_ = cv::FastFeatureDetector::create(20, true);
}

void FeatureTracker::processFrame(const cv::Mat& frame, std::vector<cv::Point2f>& tracked_points) {
    // 1. Convert frame to grayscale
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.clone();
    }

    // 2. Handle first frame initialization
    if (prev_frame_.empty()) {
        std::cout << "[FeatureTracker] Initializing tracker on first frame..." << std::endl;
        detectNewFeatures(gray, tracked_points);
        prev_frame_ = gray.clone();
        return;
    }

    // 3. Track existing features if we have any
    if (!tracked_points.empty()) {
        std::vector<cv::Point2f> next_points;
        std::vector<uchar> status;
        std::vector<float> err;

        // Perform Lucas-Kanade Sparse Optical Flow
        cv::calcOpticalFlowPyrLK(prev_frame_, gray, tracked_points, next_points, status, err, cv::Size(21, 21), 3);

        // Filter out points that failed to track or went out of bounds
        std::vector<cv::Point2f> good_points;
        for (size_t i = 0; i < tracked_points.size(); ++i) {
            if (status[i] && 
                next_points[i].x >= 0 && next_points[i].x < gray.cols &&
                next_points[i].y >= 0 && next_points[i].y < gray.rows) {
                good_points.push_back(next_points[i]);
            }
        }
        tracked_points = good_points;
    }

    // 4. Redetect new features if the number of active features drops too low
    int min_required_features = max_features_ / 2;
    if (tracked_points.size() < static_cast<size_t>(min_required_features)) {
        std::cout << "[FeatureTracker] Feature count low (" << tracked_points.size() 
                  << "). Redetecting new points..." << std::endl;
        detectNewFeatures(gray, tracked_points);
    }

    // 5. Store current frame for next iteration
    prev_frame_ = gray.clone();
}

void FeatureTracker::detectNewFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& points) {
    // Calculate how many new features we need to reach max_features_
    int features_to_detect = max_features_ - static_cast<int>(points.size());
    if (features_to_detect <= 0) return;

    // Create a exclusion mask based on existing tracked points to ensure even spatial distribution
    cv::Mat mask = cv::Mat::ones(frame.size(), CV_8UC1);
    for (const auto& pt : points) {
        cv::circle(mask, pt, static_cast<int>(min_distance_), 0, -1);
    }

    // Detect new keypoints
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(frame, keypoints, mask);

    // Sort detected keypoints by response (strength of corner)
    std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
        return a.response > b.response;
    });

    // Convert KeyPoints to Point2f and add up to features_to_detect
    int added = 0;
    for (const auto& kp : keypoints) {
        if (added >= features_to_detect) break;
        points.push_back(kp.pt);
        added++;
    }

    std::cout << "[FeatureTracker] Added " << added << " new features. Total active: " 
              << points.size() << std::endl;
}
