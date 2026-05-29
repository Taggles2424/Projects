#include "MapManager.hpp"
#include <cmath>
#include <iostream>

MapManager::MapManager(double min_translation, double min_rotation)
    : next_kf_id_(0), min_translation_(min_translation),
      min_rotation_(min_rotation) {}

void MapManager::addKeyframe(const cv::Mat &image, const cv::Mat &R,
                             const cv::Mat &t,
                             const std::vector<cv::Point2f> &points2D,
                             const std::vector<int> &point3D_indices,
                             const std::vector<int> &feature_ids) {
  Keyframe kf;
  kf.id = next_kf_id_++;
  kf.image = image.clone();
  R.convertTo(kf.R, CV_64F);
  t.convertTo(kf.t, CV_64F);
  kf.points2D = points2D;
  kf.point3D_indices = point3D_indices;
  kf.feature_ids = feature_ids;

  keyframes_.push_back(kf);

  // OPTIMIZATION: Release the heavy grayscale image buffer of old keyframes
  // to prevent memory leakage over long sequences. We only retain the last 3 keyframe image buffers.
  if (keyframes_.size() > 3) {
      keyframes_[keyframes_.size() - 4].image.release();
  }

  std::cout << "[MapManager] Added Keyframe " << kf.id
            << " to database. Total keyframes: " << keyframes_.size()
            << std::endl;
}

int MapManager::addMapPoint(const cv::Point3f &position) {
  map_points_.push_back(position);
  return static_cast<int>(map_points_.size() - 1);
}

void MapManager::updateMapPoint(int index, const cv::Point3f &position) {
  if (index >= 0 && index < static_cast<int>(map_points_.size())) {
    map_points_[index] = position;
  }
}

bool MapManager::checkNewKeyframe(const cv::Mat &current_R,
                                  const cv::Mat &current_t, int tracked_inliers,
                                  int total_map_features) {
  // If no keyframes exist, we definitely need a new keyframe to start
  if (keyframes_.empty()) {
    return true;
  }

  // Get the pose of the last keyframe in the database
  const auto &last_kf = keyframes_.back();

  cv::Mat last_R, curr_R;
  last_kf.R.convertTo(last_R, CV_64F);
  current_R.convertTo(curr_R, CV_64F);

  cv::Mat last_t, curr_t;
  last_kf.t.convertTo(last_t, CV_64F);
  current_t.convertTo(curr_t, CV_64F);

  // 1. Calculate relative translation distance of camera centers in world coordinates
  cv::Mat C_last = -last_R.t() * last_t;
  cv::Mat C_curr = -curr_R.t() * curr_t;
  double translation_dist = cv::norm(C_curr - C_last);

  // 2. Calculate relative rotation angle in degrees
  cv::Mat R_rel = curr_R * last_R.t();
  double trace = cv::trace(R_rel)[0];
  // Clamp trace to avoid NaN in acos due to minor floating point precision errors
  double cos_theta = std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0));
  double rotation_angle = std::acos(cos_theta) * 180.0 / CV_PI;

  // 3. Trigger keyframe if translation or rotation exceeds threshold
  if (translation_dist > min_translation_ || rotation_angle > min_rotation_) {
    std::cout << "[MapManager] Keyframe trigger: Translation = "
              << translation_dist << " (Threshold = " << min_translation_
              << "), Rotation = " << rotation_angle
              << " deg (Threshold = " << min_rotation_ << ")" << std::endl;
    return true;
  }

  // 4. Trigger keyframe if tracking quality drops (less than 40% of the active points are inliers)
  // BUT only do this if the camera has moved at least a small baseline (e.g., 0.1 units) since the last keyframe
  // to ensure triangulation is mathematically possible and to prevent triggering loops on static frames.
  if (total_map_features > 30 && tracked_inliers < total_map_features * 0.40 && translation_dist >= 0.1) {
    std::cout << "[MapManager] Keyframe trigger: Tracking inliers low ("
              << tracked_inliers << " / " << total_map_features << ") with baseline " << translation_dist << std::endl;
    return true;
  }

  return false;
}

void MapManager::triangulateNewPoints(int kf1_idx, int kf2_idx,
                                      const cv::Mat &K) {
  if (kf1_idx < 0 || kf1_idx >= static_cast<int>(keyframes_.size()) ||
      kf2_idx < 0 || kf2_idx >= static_cast<int>(keyframes_.size())) {
    return;
  }

  auto &kf1 = keyframes_[kf1_idx];
  auto &kf2 = keyframes_[kf2_idx];

  // Baseline baseline check: Skip triangulation if keyframes are too close to prevent ill-conditioned depths
  cv::Mat C1 = -kf1.R.t() * kf1.t;
  cv::Mat C2 = -kf2.R.t() * kf2.t;
  double baseline = cv::norm(C2 - C1);
  if (baseline < 0.1) {
    return;
  }

  cv::Mat K_double;
  K.convertTo(K_double, CV_64F);

  // 1. Build camera projection matrices for both keyframes
  // P = K * [R | t]
  cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
  kf1.R.copyTo(P1(cv::Rect(0, 0, 3, 3)));
  kf1.t.copyTo(P1(cv::Rect(3, 0, 1, 3)));
  cv::Mat P1_proj = K_double * P1;

  cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);
  kf2.R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
  kf2.t.copyTo(P2(cv::Rect(3, 0, 1, 3)));
  cv::Mat P2_proj = K_double * P2;

  // 2. Identify corresponding 2D points that are NOT yet mapped to a 3D point
  std::vector<cv::Point2f> pts1, pts2;
  std::vector<int> feat_indices_kf1;
  std::vector<int> feat_indices_kf2;

  for (size_t i = 0; i < kf2.points2D.size(); ++i) {
    if (kf2.point3D_indices[i] == -1 && i < kf2.feature_ids.size()) {
      int fid2 = kf2.feature_ids[i];
      // Find matching feature in kf1
      for (size_t j = 0; j < kf1.points2D.size(); ++j) {
        if (j < kf1.feature_ids.size() && kf1.point3D_indices[j] == -1 && kf1.feature_ids[j] == fid2) {
          pts1.push_back(kf1.points2D[j]);
          pts2.push_back(kf2.points2D[i]);
          feat_indices_kf1.push_back(static_cast<int>(j));
          feat_indices_kf2.push_back(static_cast<int>(i));
          break;
        }
      }
    }
  }

  std::cout << "[MapManager] Triangulation debug: Found " << pts1.size() << " untriangulated matching features between Keyframe " << kf1.id << " and " << kf2.id << std::endl;

  if (pts1.size() < 3)
    return; // Need a baseline of points to triangulate

  // 3. Triangulate the new correspondences
  cv::Mat points4D;
  cv::triangulatePoints(P1_proj, P2_proj, pts1, pts2, points4D);

  int added_points = 0;
  for (size_t i = 0; i < pts1.size(); ++i) {
    float w = points4D.at<float>(3, i);
    if (std::abs(w) > 1e-5) {
      float x = points4D.at<float>(0, i) / w;
      float y = points4D.at<float>(1, i) / w;
      float z = points4D.at<float>(2, i) / w;

      // Chirality verification: ensure point is in front of both camera frames
      // Camera 1: z1 = R1_row2 * pt + t1.z
      cv::Mat pt3D_mat = (cv::Mat_<double>(3, 1) << x, y, z);
      cv::Mat pt1_cam = kf1.R * pt3D_mat + kf1.t;
      cv::Mat pt2_cam = kf2.R * pt3D_mat + kf2.t;

      double z1 = pt1_cam.at<double>(2, 0);
      double z2 = pt2_cam.at<double>(2, 0);

      if (z1 > 0.1 && z2 > 0.1) {
        // Calculate reprojection error in both keyframes to filter out noisy triangulations
        double x1_proj = P1_proj.at<double>(0, 0) * x + P1_proj.at<double>(0, 1) * y + P1_proj.at<double>(0, 2) * z + P1_proj.at<double>(0, 3);
        double y1_proj = P1_proj.at<double>(1, 0) * x + P1_proj.at<double>(1, 1) * y + P1_proj.at<double>(1, 2) * z + P1_proj.at<double>(1, 3);
        double z1_proj = P1_proj.at<double>(2, 0) * x + P1_proj.at<double>(2, 1) * y + P1_proj.at<double>(2, 2) * z + P1_proj.at<double>(2, 3);

        double x2_proj = P2_proj.at<double>(0, 0) * x + P2_proj.at<double>(0, 1) * y + P2_proj.at<double>(0, 2) * z + P2_proj.at<double>(0, 3);
        double y2_proj = P2_proj.at<double>(1, 0) * x + P2_proj.at<double>(1, 1) * y + P2_proj.at<double>(1, 2) * z + P2_proj.at<double>(1, 3);
        double z2_proj = P2_proj.at<double>(2, 0) * x + P2_proj.at<double>(2, 1) * y + P2_proj.at<double>(2, 2) * z + P2_proj.at<double>(2, 3);

        if (std::abs(z1_proj) > 1e-5 && std::abs(z2_proj) > 1e-5) {
          double u1 = x1_proj / z1_proj;
          double v1 = y1_proj / z1_proj;
          double u2 = x2_proj / z2_proj;
          double v2 = y2_proj / z2_proj;

          double err1 = std::sqrt((u1 - pts1[i].x) * (u1 - pts1[i].x) + (v1 - pts1[i].y) * (v1 - pts1[i].y));
          double err2 = std::sqrt((u2 - pts2[i].x) * (u2 - pts2[i].x) + (v2 - pts2[i].y) * (v2 - pts2[i].y));

          // Set reprojection error limit of 2.0 pixels for robust SLAM mapping
          if (err1 < 2.0 && err2 < 2.0) {
            // Add point to global point cloud
            int pt_idx = addMapPoint(cv::Point3f(x, y, z));

            // Link keyframe features to this newly created 3D map point
            int feat_idx1 = feat_indices_kf1[i];
            int feat_idx2 = feat_indices_kf2[i];
            kf1.point3D_indices[feat_idx1] = pt_idx;
            kf2.point3D_indices[feat_idx2] = pt_idx;

            added_points++;
          }
        }
      }
    }
  }

  std::cout << "[MapManager] Triangulated " << added_points
            << " new 3D points between Keyframe " << kf1.id << " and " << kf2.id
            << std::endl;
}

void MapManager::clear() {
  keyframes_.clear();
  map_points_.clear();
  next_kf_id_ = 0;
}

std::vector<cv::Point3f> MapManager::generateDenseMap() const {
    std::vector<cv::Point3f> dense_points;
    
    // Bounding box for Delaunay Triangulation (assumed image size of 800x450)
    cv::Rect rect(0, 0, 800, 450);

    for (const auto& kf : keyframes_) {
        // Collect 2D points that have active 3D landmark associations
        std::vector<cv::Point2f> valid_pts2D;
        std::vector<cv::Point3f> valid_pts3D;
        for (size_t i = 0; i < kf.points2D.size(); ++i) {
            if (i < kf.point3D_indices.size() && kf.point3D_indices[i] != -1) {
                int pt_idx = kf.point3D_indices[i];
                if (pt_idx >= 0 && pt_idx < static_cast<int>(map_points_.size())) {
                    cv::Point2f pt = kf.points2D[i];
                    if (rect.contains(pt)) {
                        valid_pts2D.push_back(pt);
                        valid_pts3D.push_back(map_points_[pt_idx]);
                    }
                }
            }
        }

        if (valid_pts2D.size() < 4) continue;

        // Perform Delaunay Triangulation inside the image plane bounding box
        cv::Subdiv2D subdiv(rect);
        for (const auto& pt : valid_pts2D) {
            subdiv.insert(pt);
        }

        std::vector<cv::Vec6f> triangleList;
        subdiv.getTriangleList(triangleList);

        // Helper lambda to find the index of the closest keypoint
        auto find_closest_idx = [&](const cv::Point2f& p) -> int {
            int best_idx = -1;
            double min_dist = 1e9;
            for (size_t i = 0; i < valid_pts2D.size(); ++i) {
                double dist = cv::norm(valid_pts2D[i] - p);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = static_cast<int>(i);
                }
            }
            if (min_dist < 1.0) return best_idx;
            return -1;
        };

        for (const auto& t : triangleList) {
            cv::Point2f pt1(t[0], t[1]);
            cv::Point2f pt2(t[2], t[3]);
            cv::Point2f pt3(t[4], t[5]);

            // Ensure all vertices are within the image
            if (!rect.contains(pt1) || !rect.contains(pt2) || !rect.contains(pt3)) continue;

            // Find matching landmark indices
            int idx1 = find_closest_idx(pt1);
            int idx2 = find_closest_idx(pt2);
            int idx3 = find_closest_idx(pt3);

            if (idx1 == -1 || idx2 == -1 || idx3 == -1) continue;

            // Retrieve corresponding 3D world coordinates
            cv::Point3f P1 = valid_pts3D[idx1];
            cv::Point3f P2 = valid_pts3D[idx2];
            cv::Point3f P3 = valid_pts3D[idx3];

            // Sample pixels on a regular grid inside the bounding box of the triangle
            float min_u = std::min({pt1.x, pt2.x, pt3.x});
            float max_u = std::max({pt1.x, pt2.x, pt3.x});
            float min_v = std::min({pt1.y, pt2.y, pt3.y});
            float max_v = std::max({pt1.y, pt2.y, pt3.y});

            // Step size of 6 pixels to balance density and performance
            int step = 6;
            double denom = (pt2.y - pt3.y) * (pt1.x - pt3.x) + (pt3.x - pt2.x) * (pt1.y - pt3.y);
            if (std::abs(denom) < 1e-6) continue;

            for (float u = std::floor(min_u); u <= std::ceil(max_u); u += step) {
                for (float v = std::floor(min_v); v <= std::ceil(max_v); v += step) {
                    // Compute Barycentric coordinates
                    double lambda1 = ((pt2.y - pt3.y) * (u - pt3.x) + (pt3.x - pt2.x) * (v - pt3.y)) / denom;
                    double lambda2 = ((pt3.y - pt1.y) * (u - pt3.x) + (pt1.x - pt3.x) * (v - pt3.y)) / denom;
                    double lambda3 = 1.0 - lambda1 - lambda2;

                    // Check if point is inside the triangle
                    if (lambda1 >= -1e-3 && lambda2 >= -1e-3 && lambda3 >= -1e-3) {
                        // Interpolate 3D position
                        cv::Point3f P_dense = lambda1 * P1 + lambda2 * P2 + lambda3 * P3;
                        dense_points.push_back(P_dense);
                    }
                }
            }
        }
    }
    
    return dense_points;
}
