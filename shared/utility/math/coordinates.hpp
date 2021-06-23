/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
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
    inline Eigen::Matrix<U, 3, 1> sphericalToCartesian(const Eigen::MatrixBase<T>& sphericalCoordinates) {
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
    inline Eigen::Matrix<U, 3, 1> cartesianToSpherical(const Eigen::MatrixBase<T>& cartesianCoordinates) {
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
    inline Eigen::Matrix<U, 3, 1> inverseDistanceCartesianToSpherical(
        const Eigen::MatrixBase<T>& cartesianCoordinates) {
        Eigen::Matrix<U, 3, 1> result = cartesianToSpherical(cartesianCoordinates);
        return {U(1) / result.x(), result.y(), result.z()};

        return result;
    }

}  // namespace utility::math::coordinates


#endif  // UTILITY_MATH_COORDINATES_HPP
