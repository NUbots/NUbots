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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_MATRIX_ROTATION3D_H
#define UTILITY_MATH_MATRIX_ROTATION3D_H

#include <armadillo>

namespace utility {
namespace math {
namespace matrix {

    template <int Dimensions>
    class Rotation;

    using Rotation3D = Rotation<3>;

    template <>
    class Rotation<3> : public arma::mat33 {
        using arma::mat33::mat33; // inherit constructors

        public:
            /**
             * @brief Default constructor creates an identity matrix
             */
            Rotation();

            /**
             * @brief Convert from a quaternions vec4
             */
            Rotation(const arma::vec4& q);
            /**
             * @brief Create a rotation matrix based on a vec3 and an angle
             */
            Rotation(const arma::vec3& axis, double angle);

            /**
             * @brief Rotates matrix around the local X axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation3D rotateX(double radians) const;

            /**
             * @brief Rotates matrix around the local Y axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation3D rotateY(double radians) const;

            /**
             * @brief Rotates matrix around the local Z axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation3D rotateZ(double radians) const;

            /**
             * @brief Performs an inverse and returns a new copy
             * Note: Assumes current transform is orthonormal and invertible (which it should be given normal use)
             *
             * @return The inverse transform
             */
            Rotation3D i() const;

            /**
             * @return Pair containing the axis of the rotation as a unit vector followed by the rotation angle.
             */
            std::pair<arma::vec3, double> axisAngle() const;

            /**
             * @brief Creates a rotation matrix around the X axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation3D createRotationX(double radians);

            /**
             * @brief Creates a rotation matrix around the Y axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation3D createRotationY(double radians);

            /**
             * @brief Creates a rotation matrix around the Z axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation3D createRotationZ(double radians);
    };

}  // matrix
}  // math
}  // utility

#endif  // UTILITY_MATH_MATRIX_ROTATION3D_H
