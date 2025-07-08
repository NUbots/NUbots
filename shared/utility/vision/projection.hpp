#ifndef UTILITY_VISION_PROJECTION_HPP
#define UTILITY_VISION_PROJECTION_HPP

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace utility::vision {

    // Coordinate transformation matrices
    // NUbots: X = forward, Y = left, Z = up
    // OpenCV: X = right, Y = down, Z = forward
    static const Eigen::Matrix3d R_nubots_to_opencv = (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();

    static const Eigen::Matrix3d R_opencv_to_nubots = R_nubots_to_opencv.transpose();

    /**
     * @brief Projects a 3D unit ray into pixel coordinates using OpenCV fisheye model
     * @tparam T     Scalar type (float, double)
     * @tparam Lens  Lens struct type with fx, fy, centre, k
     * @param ray    3D unit vector in NUbots coordinate system (X=forward, Y=left, Z=up)
     * @param lens   Lens parameters
     * @return 2D pixel coordinate
     */
    template <typename T, typename Lens>
    inline Eigen::Matrix<T, 2, 1> project(const Eigen::Matrix<T, 3, 1>& ray, const Lens& lens) {
        // Convert to OpenCV coordinate frame
        Eigen::Matrix<T, 3, 1> ray_opencv = R_nubots_to_opencv.template cast<T>() * ray;

        std::vector<cv::Point3d> objectPoints = {cv::Point3d(static_cast<double>(ray_opencv.x()),
                                                             static_cast<double>(ray_opencv.y()),
                                                             static_cast<double>(ray_opencv.z()))};

        std::vector<cv::Point2d> imagePoints;

        cv::Mat K = (cv::Mat_<double>(3, 3) << static_cast<double>(lens.fx),
                     0.0,
                     static_cast<double>(lens.centre.x()),
                     0.0,
                     static_cast<double>(lens.fy),
                     static_cast<double>(lens.centre.y()),
                     0.0,
                     0.0,
                     1.0);

        cv::Mat D = (cv::Mat_<double>(4, 1) << static_cast<double>(lens.k(0)),
                     static_cast<double>(lens.k(1)),
                     static_cast<double>(lens.k(2)),
                     static_cast<double>(lens.k(3)));

        cv::fisheye::projectPoints(objectPoints, imagePoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D);

        return Eigen::Matrix<T, 2, 1>(static_cast<T>(imagePoints[0].x), static_cast<T>(imagePoints[0].y));
    }

    /**
     * @brief Unprojects a 2D pixel coordinate to a 3D unit vector using OpenCV fisheye model
     * @tparam T     Scalar type
     * @tparam Lens  Lens struct type with fx, fy, centre, k
     * @param px     2D pixel coordinate
     * @param lens   Lens parameters
     * @return 3D unit vector in NUbots coordinate system (X=forward, Y=left, Z=up)
     */
    template <typename T, typename Lens>
    inline Eigen::Matrix<T, 3, 1> unproject(const Eigen::Matrix<T, 2, 1>& px, const Lens& lens) {
        std::vector<cv::Point2d> distortedPoints = {
            cv::Point2d(static_cast<double>(px.x()), static_cast<double>(px.y()))};

        std::vector<cv::Point2d> undistorted;

        cv::Mat K = (cv::Mat_<double>(3, 3) << static_cast<double>(lens.fx),
                     0.0,
                     static_cast<double>(lens.centre.x()),
                     0.0,
                     static_cast<double>(lens.fy),
                     static_cast<double>(lens.centre.y()),
                     0.0,
                     0.0,
                     1.0);

        cv::Mat D = (cv::Mat_<double>(4, 1) << static_cast<double>(lens.k(0)),
                     static_cast<double>(lens.k(1)),
                     static_cast<double>(lens.k(2)),
                     static_cast<double>(lens.k(3)));

        cv::fisheye::undistortPoints(distortedPoints, undistorted, K, D);

        Eigen::Matrix<T, 3, 1> ray_opencv(static_cast<T>(undistorted[0].x),
                                          static_cast<T>(undistorted[0].y),
                                          static_cast<T>(1.0));
        ray_opencv.normalize();

        return R_opencv_to_nubots.template cast<T>() * ray_opencv;
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_PROJECTION_HPP
