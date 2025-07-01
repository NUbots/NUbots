#ifndef UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP
#define UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace utility::vision {

    /**
     * @brief Applies OpenCV fisheye distortion model to an angle θ
     *
     * @tparam T the scalar type
     * @param theta  the angle from the optical axis
     * @param lens   the lens struct containing k[0..3] coefficients
     * @return distorted angle θ_d
     */
    template <typename T, typename Lens>
    inline T distort(const T& theta, const Lens& lens) {
        const auto& k  = lens.k;
        const T theta2 = theta * theta;
        const T theta4 = theta2 * theta2;
        const T theta6 = theta4 * theta2;
        const T theta8 = theta4 * theta4;
        return theta * (T(1) + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
    }

    /**
     * @brief Inverts OpenCV fisheye distortion model via Newton-Raphson
     *
     * @tparam T the scalar type
     * @param theta_d  distorted angle
     * @param lens     lens with k[0..3]
     * @return undistorted angle θ
     */
    template <typename T, typename Lens>
    T undistort(const T& theta_d, const Lens& lens) {
        T theta = theta_d;  // initial guess
        for (int i = 0; i < 5; ++i) {
            T theta2 = theta * theta;
            T theta4 = theta2 * theta2;
            T theta6 = theta4 * theta2;
            T theta8 = theta4 * theta4;
            T f = theta * (T(1) + lens.k[0] * theta2 + lens.k[1] * theta4 + lens.k[2] * theta6 + lens.k[3] * theta8)
                  - theta_d;
            T df  = (T(1) + 3 * lens.k[0] * theta2 + 5 * lens.k[1] * theta4 + 7 * lens.k[2] * theta6
                    + 9 * lens.k[3] * theta8);
            theta = theta - f / df;
        }
        return theta;
    }

    /**
     * @brief Projects a 3D unit ray into pixel coordinates using OpenCV fisheye model
     *
     * @tparam T scalar type
     * @param ray       unit vector in camera space
     * @param lens      lens parameters with k[0..3], focal_length, and centre
     * @param dimensions image resolution
     * @return pixel coordinate
     */
    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time, typename Lens>
    Eigen::Matrix<T, 2, 1> project(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const Lens& lens,
        const Eigen::Matrix<T, 2, 1>& dimensions) {

        const T theta     = std::acos(ray.x());
        const T theta_d   = distort(theta, lens);
        const T sin_theta = std::sqrt(1 - ray.x() * ray.x());

        Eigen::Matrix<T, 2, 1> screen =
            ray.x() >= T(1) ? Eigen::Matrix<T, 2, 1>::Zero()
                            : Eigen::Matrix<T, 2, 1>(theta_d * ray.y() / sin_theta, theta_d * ray.z() / sin_theta);

        return (dimensions * T(0.5)) - screen - lens.centre.template cast<T>();
    }

    /**
     * @brief Unprojects a 2D pixel into a 3D unit vector using OpenCV fisheye model
     *
     * @tparam T scalar type
     * @param px         pixel coordinate
     * @param lens       lens parameters with k[0..3], focal_length, and centre
     * @param dimensions image resolution
     * @return unit vector in camera space
     */
    template <typename T, typename Lens>
    Eigen::Matrix<T, 3, 1> unproject(const Eigen::Matrix<T, 2, 1>& px,
                                     const Lens& lens,
                                     const Eigen::Matrix<T, 2, 1>& dimensions) {

        Eigen::Matrix<T, 2, 1> screen = (dimensions * T(0.5)) - px - lens.centre.template cast<T>();

        const T r_d = screen.norm();
        if (r_d == T(0)) {
            return Eigen::Matrix<T, 3, 1>::UnitX();
        }

        const T theta     = undistort(r_d, lens);
        const T sin_theta = std::sin(theta);

        return Eigen::Matrix<T, 3, 1>(std::cos(theta), sin_theta * screen.x() / r_d, sin_theta * screen.y() / r_d);
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_PROJECTION_OPENCV_FISHEYE_HPP
