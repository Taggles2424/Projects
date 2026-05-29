#include "MapInitializer.hpp"
#include <iostream>

MapInitializer::MapInitializer(const cv::Mat& K) {
    K.convertTo(K_, CV_64F);
}

bool MapInitializer::initialize(const std::vector<cv::Point2f>& points1,
                                const std::vector<cv::Point2f>& points2,
                                cv::Mat& R, cv::Mat& t,
                                std::vector<cv::Point3f>& points3D,
                                std::vector<bool>& inlier_mask) {
    
    // Clear outputs
    points3D.clear();
    inlier_mask.clear();

    // 1. Check if we have enough points to initialize
    if (points1.size() < 50 || points2.size() < 50) {
        std::cout << "[MapInitializer] FAILED: Too few point correspondences (" 
                  << points1.size() << ")" << std::endl;
        return false;
    }

    // 2. Estimate the Essential Matrix using RANSAC
    cv::Mat mask;
    // Threshold of 1.0 pixel for RANSAC inlier verification
    cv::Mat E = cv::findEssentialMat(points1, points2, K_, cv::RANSAC, 0.999, 1.0, mask);

    if (E.empty() || E.rows < 3 || E.cols < 3) {
        std::cout << "[MapInitializer] FAILED: Essential Matrix calculation failed." << std::endl;
        return false;
    }

    // 3. Decompose E to find relative Rotation (R) and Translation (t)
    // recoverPose automatically performs chirality check (ensures points are in front of camera)
    int inliers = cv::recoverPose(E, points1, points2, K_, R, t, mask);

    std::cout << "[MapInitializer] RANSAC Inliers: " << inliers << " / " << points1.size() << std::endl;

    // We require a solid number of inliers (at least 30) for a stable initialization
    if (inliers < 30) {
        std::cout << "[MapInitializer] FAILED: Too few matching inliers for pose recovery." << std::endl;
        return false;
    }

    // 4. Setup projection matrices P1 and P2
    // P1 = K * [I | 0]
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
    I.copyTo(P1(cv::Rect(0, 0, 3, 3)));
    cv::Mat P1_proj = K_ * P1;

    // P2 = K * [R | t]
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);
    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);
    R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(P2(cv::Rect(3, 0, 1, 3)));
    cv::Mat P2_proj = K_ * P2;

    // 5. Perform Triangulation
    cv::Mat points4D;
    cv::triangulatePoints(P1_proj, P2_proj, points1, points2, points4D);

    // 6. Convert homogeneous 4D coordinates to Cartesian 3D coordinates and filter
    points3D.resize(points1.size());
    inlier_mask.resize(points1.size(), false);

    int valid_3d_points = 0;
    for (size_t i = 0; i < points1.size(); ++i) {
        // Only triangulate RANSAC inliers
        if (mask.at<uchar>(i) == 0) {
            continue;
        }

        // Convert homogeneous to 3D Cartesian coordinates
        float w = points4D.at<float>(3, i);
        if (std::abs(w) > 1e-5) {
            float x = points4D.at<float>(0, i) / w;
            float y = points4D.at<float>(1, i) / w;
            float z = points4D.at<float>(2, i) / w;

            cv::Point3f pt3D(x, y, z);

            // Chirality checks: Ensure the point is in front of BOTH cameras
            // Camera 1 (origin): pt3D.z > 0
            // Camera 2: z2 = R_row2 * pt3D + t.z > 0
            cv::Mat pt3D_mat = (cv::Mat_<double>(3, 1) << x, y, z);
            cv::Mat pt2_mat = R * pt3D_mat + t;
            double z2 = pt2_mat.at<double>(2, 0);

            if (z > 0 && z2 > 0 && z < 100.0) { // filter points too far away (e.g. > 100 units)
                points3D[i] = pt3D;
                inlier_mask[i] = true;
                valid_3d_points++;
            }
        }
    }

    std::cout << "[MapInitializer] SUCCESS: Initialized map with " << valid_3d_points 
              << " valid 3D points." << std::endl;

    return (valid_3d_points > 20); // Require at least 20 stable 3D points
}
