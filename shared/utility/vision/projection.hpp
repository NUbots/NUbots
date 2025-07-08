#ifndef UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP
#define UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace utility::vision {

    struct Lens {
        Eigen::Vector4d k;       // Distortion coefficients: [k1, k2, k3, k4]
        double focal_length;     // Shared focal length (fx = fy = f)
        Eigen::Vector2d centre;  // Principal point: [cx, cy]
    };

    /**
     * @brief Projects a 3D unit ray into pixel coordinates using OpenCV fisheye model
     *        via cv::fisheye::projectPoints
     */
    inline Eigen::Vector2d project(const Eigen::Vector3d& ray, const Lens& lens) {
        // Convert ray to OpenCV format
        std::vector<cv::Point3d> objectPoints = {cv::Point3d(ray(0), ray(1), ray(2))};
        std::vector<cv::Point2d> imagePoints;

        cv::Mat K = (cv::Mat_<double>(3, 3) << lens.focal_length,
                     0,
                     lens.centre.x(),
                     0,
                     lens.focal_length,
                     lens.centre.y(),
                     0,
                     0,
                     1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << lens.k(0), lens.k(1), lens.k(2), lens.k(3));

        cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D);

        return Eigen::Vector2d(imagePoints[0].x, imagePoints[0].y);
    }

    /**
     * @brief Unprojects a 2D pixel coordinate to a 3D unit vector using OpenCV fisheye model
     *        via cv::fisheye::undistortPoints
     */
    inline Eigen::Vector3d unproject(const Eigen::Vector2d& px, const Lens& lens) {
        std::vector<cv::Point2d> distortedPoints = {cv::Point2d(px(0), px(1))};
        std::vector<cv::Point2d> undistorted;

        cv::Mat K = (cv::Mat_<double>(3, 3) << lens.focal_length,
                     0,
                     lens.centre.x(),
                     0,
                     lens.focal_length,
                     lens.centre.y(),
                     0,
                     0,
                     1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << lens.k(0), lens.k(1), lens.k(2), lens.k(3));

        cv::fisheye::undistortPoints(distortedPoints, undistorted, K, D);

        // Convert to 3D direction vector (z=1, then normalize)
        Eigen::Vector3d ray(undistorted[0].x, undistorted[0].y, 1.0);
        return ray.normalized();
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP
