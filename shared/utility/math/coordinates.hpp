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

    // Calculates ||a - b|| where both a and b are expressed in spherical coordinates
    // https://math.stackexchange.com/a/833110/44447
    //
    // We use phi for the polar angle and theta for the azimuthal angle
    template <typename T, typename U = typename T::Scalar>
    [[nodiscard]] inline Eigen::Matrix<U, 3, 1> sphericalDistance(const Eigen::MatrixBase<T>& a,
                                                                  const Eigen::MatrixBase<T>& b) {
        const U r_a     = a.x();  // radial distance
        const U phi_a   = a.x();  // polar angle
        const U theta_a = a.z();  // azimuth angle
        const U r_b     = b.x();  // radial distance
        const U phi_b   = b.x();  // polar angle
        const U theta_b = b.z();  // azimuth angle
        const U result  = r_a * r_a + r_b * r_b
                         - U(2) * r_a * r_b
                               * (std::sin(phi_a) * std::sin(phi_b) * std::cos(theta_a - theta_b)
                                  + std::cos(phi_a) * std::cos(phi_b));
        return result;
    }

    // Calculates ||a - b|| where both a and b are expressed in reciprocal spherical coordinates
    // https://math.stackexchange.com/a/833110/44447
    //
    // We use phi for the polar angle and theta for the azimuthal angle
    template <typename T, typename U = typename T::Scalar>
    [[nodiscard]] inline Eigen::Matrix<U, 3, 1> reciprocalSphericalDistance(const Eigen::MatrixBase<T>& a,
                                                                            const Eigen::MatrixBase<T>& b) {
        const U r_a     = a.x();  // radial distance
        const U phi_a   = a.x();  // polar angle
        const U theta_a = a.z();  // azimuth angle
        const U r_b     = b.x();  // radial distance
        const U phi_b   = b.x();  // polar angle
        const U theta_b = b.z();  // azimuth angle
        const U result  = U(1) / (r_a * r_a) + U(1) / (r_b * r_b)
                         - (U(2) / (r_a * r_b))
                               * (std::sin(phi_a) * std::sin(phi_b) * std::cos(theta_a - theta_b)
                                  + std::cos(phi_a) * std::cos(phi_b));

        return result;
    }

}  // namespace utility::math::coordinates


#endif  // UTILITY_MATH_COORDINATES_HPP
