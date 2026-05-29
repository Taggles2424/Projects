#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureTracker {
public:
    // Constructor specifying max feature count and minimum distance between features
    FeatureTracker(int max_features = 1000, double min_distance = 15.0);

    // Process a new frame: tracks existing points from prev_frame_ to current frame using Lucas-Kanade
    void processFrame(const cv::Mat& frame, std::vector<cv::Point2f>& tracked_points);

private:
    // Detects new features using FAST to keep feature count high
    void detectNewFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& points);

    int max_features_;
    double min_distance_;
    cv::Mat prev_frame_;
    cv::Ptr<cv::FastFeatureDetector> detector_;
};
