#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// Struct representing a SLAM Keyframe
struct Keyframe {
    int id;
    cv::Mat image;                      // Grayscale image buffer (stored for potential dense mapping later)
    cv::Mat R;                          // Rotation matrix (world-to-camera)
    cv::Mat t;                          // Translation vector (world-to-camera)
    std::vector<cv::Point2f> points2D;  // 2D keypoint coordinates in this frame
    std::vector<int> point3D_indices;   // Indices of matching 3D points in the global map (-1 if unmapped)
    std::vector<int> feature_ids;       // Unique global feature IDs to match features across keyframes
};

class MapManager {
public:
    // Constructor setting up keyframe thresholds
    MapManager(double min_translation = 0.5, double min_rotation = 15.0);

    // Adds a new keyframe to the database
    void addKeyframe(const cv::Mat& image, const cv::Mat& R, const cv::Mat& t,
                     const std::vector<cv::Point2f>& points2D,
                     const std::vector<int>& point3D_indices,
                     const std::vector<int>& feature_ids);

    // Adds a new 3D map point to the global map and returns its index
    int addMapPoint(const cv::Point3f& position);

    // Updates an existing map point position
    void updateMapPoint(int index, const cv::Point3f& position);

    // Checks whether the current frame has moved enough to qualify as a new keyframe
    bool checkNewKeyframe(const cv::Mat& current_R, const cv::Mat& current_t,
                          int tracked_inliers, int total_map_features);

    // Triangulates new 3D points between two keyframes to grow the sparse map
    void triangulateNewPoints(int kf1_idx, int kf2_idx, const cv::Mat& K);

    // Clear map database
    void clear();

    // Getters for modularity (allows the visualizer and dense reconstruction to query the database)
    const std::vector<Keyframe>& getKeyframes() const { return keyframes_; }
    const std::vector<cv::Point3f>& getMapPoints() const { return map_points_; }
    
    // Generates a dense 3D reconstruction using Delaunay Triangulation and Barycentric Depth Propagation
    std::vector<cv::Point3f> generateDenseMap() const;
    
    // Non-const references for editing
    std::vector<Keyframe>& getKeyframesRef() { return keyframes_; }
    std::vector<cv::Point3f>& getMapPointsRef() { return map_points_; }

private:
    std::vector<Keyframe> keyframes_;
    std::vector<cv::Point3f> map_points_;
    int next_kf_id_;

    // Thresholds for keyframe triggering
    double min_translation_;            // Translation threshold in metric units
    double min_rotation_;               // Rotation threshold in degrees
};
