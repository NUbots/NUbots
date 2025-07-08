#ifndef UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP
#define UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace utility::vision {

    struct Lens {
        Eigen::Vector4d k;       // Distortion coefficients: [k1, k2, k3, k4]
        double fx;               // Focal length in x
        double fy;               // Focal length in y
        Eigen::Vector2d centre;  // Principal point: [cx, cy]
    };

    // Coordinate transformation matrices
    // NUbots: X=forward, Y=left, Z=up
    // OpenCV: X=right, Y=down, Z=forward
    static const Eigen::Matrix3d R_nubots_to_opencv = (Eigen::Matrix3d() << 0,
                                                       -1,
                                                       0,  // OpenCV X (right) = -NUbots Y (left)
                                                       0,
                                                       0,
                                                       -1,  // OpenCV Y (down)  = -NUbots Z (up)
                                                       1,
                                                       0,
                                                       0  // OpenCV Z (forward) = NUbots X (forward)
                                                       )
                                                          .finished();

    static const Eigen::Matrix3d R_opencv_to_nubots = R_nubots_to_opencv.transpose();

    /**
     * @brief Projects a 3D unit ray into pixel coordinates using OpenCV fisheye model
     * @param ray  3D unit vector in NUbots coordinate system (X=forward, Y=left, Z=up)
     * @param lens OpenCV-compatible lens parameters
     * @return 2D pixel coordinate
     */
    inline Eigen::Vector2d project(const Eigen::Vector3d& ray, const Lens& lens) {
        // Convert NUbots ray to OpenCV coordinate system
        Eigen::Vector3d ray_opencv = R_nubots_to_opencv * ray;

        std::vector<cv::Point3d> objectPoints = {cv::Point3d(ray_opencv(0), ray_opencv(1), ray_opencv(2))};
        std::vector<cv::Point2d> imagePoints;

        cv::Mat K = (cv::Mat_<double>(3, 3) << lens.fx, 0, lens.centre.x(), 0, lens.fy, lens.centre.y(), 0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << lens.k(0), lens.k(1), lens.k(2), lens.k(3));

        cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D);

        return Eigen::Vector2d(imagePoints[0].x, imagePoints[0].y);
    }

    /**
     * @brief Unprojects a 2D pixel coordinate to a 3D unit vector using OpenCV fisheye model
     * @param px   2D pixel coordinate
     * @param lens OpenCV-compatible lens parameters
     * @return 3D unit vector in NUbots coordinate system (X=forward, Y=left, Z=up)
     */
    inline Eigen::Vector3d unproject(const Eigen::Vector2d& px, const Lens& lens) {
        std::vector<cv::Point2d> distortedPoints = {cv::Point2d(px(0), px(1))};
        std::vector<cv::Point2d> undistorted;

        cv::Mat K = (cv::Mat_<double>(3, 3) << lens.fx, 0, lens.centre.x(), 0, lens.fy, lens.centre.y(), 0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << lens.k(0), lens.k(1), lens.k(2), lens.k(3));

        cv::fisheye::undistortPoints(distortedPoints, undistorted, K, D);

        // Convert normalized coordinates to 3D vector in OpenCV coordinate system
        Eigen::Vector3d ray_opencv(undistorted[0].x, undistorted[0].y, 1.0);
        ray_opencv.normalize();

        // Convert back to NUbots coordinate system
        return R_opencv_to_nubots * ray_opencv;
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP
