#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    cout << "========================================" << endl;
    cout << "  Generating Synthetic Feature Video" << endl;
    cout << "========================================" << endl;

    // 1. Setup Video Writer
    int width = 800;
    int height = 450;
    string out_path = "../../temp_videos/synthetic_features.mp4";
    
    VideoWriter writer;
    writer.open(out_path, VideoWriter::fourcc('m', 'p', '4', 'v'), 30, Size(width, height));
    if (!writer.isOpened()) {
        cout << "[Warning] Could not open MP4 writer, falling back to AVI/MJPG..." << endl;
        out_path = "../../temp_videos/synthetic_features.avi";
        writer.open(out_path, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(width, height));
    }

    if (!writer.isOpened()) {
        cerr << "ERROR: Failed to initialize VideoWriter!" << endl;
        return -1;
    }

    // 2. Camera Intrinsic Matrix K (Scale for 800x450)
    double fx = 500.0, fy = 500.0;
    double cx = 400.0, cy = 225.0;

    // 3. Generate Random 3D Landmarks in World Space
    // We distribute landmarks in a box: X in [-2, 2], Y in [-2, 2], Z in [3.5, 12.5]
    int num_landmarks = 100;
    vector<Point3f> landmarks;
    mt19937 rng(42); // Seeded for deterministic generation
    uniform_real_distribution<float> dist_x(-2.0f, 2.0f);
    uniform_real_distribution<float> dist_y(-2.0f, 2.0f);
    uniform_real_distribution<float> dist_z(3.5f, 12.5f);

    for (int i = 0; i < num_landmarks; ++i) {
        landmarks.push_back(Point3f(dist_x(rng), dist_y(rng), dist_z(rng)));
    }

    // 4. Generate 600 Frames with Smooth Spiral Camera Path
    int num_frames = 600;
    Point3f target(0.0f, 0.0f, 8.0f); // Center look-at target shifted to 8.0

    for (int k = 0; k < num_frames; ++k) {
        // Spiral camera path in world coordinates C_w
        double t_factor = static_cast<double>(k) / num_frames;
        double cx_w = 1.0 * sin(2.0 * CV_PI * t_factor * 3.0); // 3.0 rotations
        double cy_w = 0.6 * cos(2.0 * CV_PI * t_factor * 3.0);
        double cz_w = -2.0 + 3.0 * t_factor; // Translating forward from -2.0 to +1.0 (so camera never gets too close to points!)

        Point3f C_w(static_cast<float>(cx_w), static_cast<float>(cy_w), static_cast<float>(cz_w));

        // Compute Look-At Rotation Matrix R (World-to-Camera)
        // Forward vector = target - camera center
        Point3f F = target - C_w;
        float norm_F = sqrt(F.x * F.x + F.y * F.y + F.z * F.z);
        F.x /= norm_F; F.y /= norm_F; F.z /= norm_F;

        // Up vector = (0, 1, 0)
        Point3f U_temp(0.0f, 1.0f, 0.0f);
        
        // Right vector = U_temp x F
        Point3f R_vec;
        R_vec.x = U_temp.y * F.z - U_temp.z * F.y;
        R_vec.y = U_temp.z * F.x - U_temp.x * F.z;
        R_vec.z = U_temp.x * F.y - U_temp.y * F.x;
        float norm_R = sqrt(R_vec.x * R_vec.x + R_vec.y * R_vec.y + R_vec.z * R_vec.z);
        R_vec.x /= norm_R; R_vec.y /= norm_R; R_vec.z /= norm_R;

        // Actual Up vector = F x Right vector
        Point3f U_vec;
        U_vec.x = F.y * R_vec.z - F.z * R_vec.y;
        U_vec.y = F.z * R_vec.x - F.x * R_vec.z;
        U_vec.z = F.x * R_vec.y - F.y * R_vec.x;

        // Populate Rotation matrix
        Mat R = (Mat_<double>(3, 3) <<
            R_vec.x, R_vec.y, R_vec.z,
            U_vec.x, U_vec.y, U_vec.z,
            F.x,     F.y,     F.z);

        // Compute Translation vector t = -R * C_w
        Mat C_w_mat = (Mat_<double>(3, 1) << C_w.x, C_w.y, C_w.z);
        Mat t = -R * C_w_mat;

        // Create black canvas
        Mat canvas = Mat::zeros(height, width, CV_8UC3);

        // Draw projected landmarks
        for (const auto& pt : landmarks) {
            // Transform landmark to camera frame
            Mat pt_w = (Mat_<double>(3, 1) << pt.x, pt.y, pt.z);
            Mat pt_c = R * pt_w + t;

            double xc = pt_c.at<double>(0, 0);
            double yc = pt_c.at<double>(1, 0);
            double zc = pt_c.at<double>(2, 0);

            if (zc > 0.1) {
                // Project to image plane
                double u = fx * (xc / zc) + cx;
                double v = fy * (yc / zc) + cy;

                // Draw solid white circles to represent high-contrast visual features
                if (u >= 10 && u < width - 10 && v >= 10 && v < height - 10) {
                    circle(canvas, Point(static_cast<int>(u), static_cast<int>(v)), 6, Scalar(255, 255, 255), -1);
                }
            }
        }

        // Write frame to video
        writer.write(canvas);
    }

    writer.release();
    cout << "[Success] Synthetic video generated successfully at: " << out_path << " (300 frames, 800x450)" << endl;
    return 0;
}
