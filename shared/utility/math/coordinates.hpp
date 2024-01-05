/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILITY_MATH_COORDINATES_HPP
#define UTILITY_MATH_COORDINATES_HPP

#include <Eigen/Core>
#include <cmath>


/**
 * Functions to convert between coordinate representations.
 *
 * (r,phi,theta) represent radial distance, bearing (counter-clockwise from x-axis in xy-plane) and elevation
 * (measured from the xy plane) (in radians)
 * @author Alex Biddulph
 */
namespace utility::math::coordinates {

    template <typename T, typename U = typename T::Scalar>
    [[nodiscard]] inline Eigen::Matrix<U, 3, 1> sphericalToCartesian(const Eigen::MatrixBase<T>& sphericalCoordinates) {
        double distance  = sphericalCoordinates.x();
        double cos_theta = std::cos(sphericalCoordinates.y());
        double sin_theta = std::sin(sphericalCoordinates.y());
        double cos_phi   = std::cos(sphericalCoordinates.z());
        double sin_phi   = std::sin(sphericalCoordinates.z());
        Eigen::Matrix<U, 3, 1> result;

        result.x() = distance * cos_theta * cos_phi;
        result.y() = distance * sin_theta * cos_phi;
        result.z() = distance * sin_phi;

        return result;
    }

    template <typename T, typename U = typename T::Scalar>
    [[nodiscard]] inline Eigen::Matrix<U, 3, 1> cartesianToSpherical(const Eigen::MatrixBase<T>& cartesianCoordinates) {
        const U x = cartesianCoordinates.x();
        const U y = cartesianCoordinates.y();
        const U z = cartesianCoordinates.z();
        Eigen::Matrix<U, 3, 1> result;

        result.x() = std::sqrt(x * x + y * y + z * z);                                                     // r
        result.y() = std::atan2(y, x);                                                                     // theta
        result.z() = (result.x() == static_cast<U>(0)) ? static_cast<U>(0) : std::asin(z / (result.x()));  // phi

        return result;
    }

    template <typename T, typename U = typename T::Scalar>
    [[nodiscard]] inline Eigen::Matrix<U, 3, 1> cartesianToReciprocalSpherical(
        const Eigen::MatrixBase<T>& cartesianCoordinates) {
        const Eigen::Matrix<U, 3, 1> sphericalCoordinates = cartesianToSpherical(cartesianCoordinates);
        return {U(1) / sphericalCoordinates.x(), sphericalCoordinates.y(), sphericalCoordinates.z()};
    }

    template <typename T, typename U = typename T::Scalar>
    [[nodiscard]] inline Eigen::Matrix<U, 3, 1> reciprocalSphericalToCartesian(
        const Eigen::MatrixBase<T>& reciprocalSphericalCoordinates) {
        Eigen::Matrix<U, 3, 1> sphericalCoordinates;
        sphericalCoordinates.x()                          = U(1) / reciprocalSphericalCoordinates.x();
        sphericalCoordinates.y()                          = reciprocalSphericalCoordinates.y();
        sphericalCoordinates.z()                          = reciprocalSphericalCoordinates.z();
        const Eigen::Matrix<U, 3, 1> cartesianCoordinates = sphericalToCartesian(sphericalCoordinates);
        return cartesianCoordinates;
    }

    /// @brief Converts the given vector from a cartesian direction vector to a 2d screen angular coordinates
    /// @tparam T scalar template for the vectors
    /// @param v The cartesian direction vector
    /// @return The screen angular coordinates
    template <typename T>
    [[nodiscard]] inline Eigen::Matrix<T, 2, 1> screen_angular_from_object_direction(const Eigen::Matrix<T, 3, 1>& v) {
        return {std::atan2(v.y(), v.x()), std::atan2(v.z(), v.x())};
    }

}  // namespace utility::math::coordinates


#endif  // UTILITY_MATH_COORDINATES_HPP
