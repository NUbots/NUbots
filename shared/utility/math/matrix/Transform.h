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

#ifndef UTILITY_MATH_TRANSFORM_H
#define UTILITY_MATH_TRANSFORM_H

#include <armadillo>

namespace utility {
namespace math {
namespace matrix {

    /**
     * @brief A 4x4 homogeneous orthonormal basis matrix class for representing 3D transformations
     *
     * See:
     * http://en.wikipedia.org/wiki/Transformation_matrix
     * http://en.wikipedia.org/wiki/Rotation_group_SO(3)
     * http://en.wikipedia.org/wiki/Orthogonal_matrix
     * http://en.wikipedia.org/wiki/Orthonormal_basis
     *
     * @author Brendan Annable
     */
    class Transform : public arma::mat44 {
        using arma::mat44::mat44; // inherit constructors

        public:
            /**
             * @brief Default constructor creates an identity matrix
             */
            Transform();

            /**
             * @brief Convert from a quaternions vec4
             */
            Transform(arma::vec4 q);

            /**
             * @brief Convert from a vec6 representing [position_x, position_y, position_z, rotation_x, rotation_y, rotation_z]
             */
            Transform(arma::vec6 in);

            /**
             * @brief Translate the current basis by the given 3D vector
             *
             * @param translation The 3D translation vector to translate by
             * @return The transformed basis matrix
             */
            Transform translate(const arma::vec3& translation) const;

            /*
             * @brief Translate the current basis along the local X axis
             *
             * This translates along the column vector submatrix(0,0,2,0)
             *
             * @param translation The amount to translate by
             * @return The transformed basis matrix
             */
            Transform translateX(double translation) const;

            /**
             * @brief Translate the current basis along the local Y axis
             *
             * This translates along the column vector submatrix(0,1,2,1)
             *
             * @param translation The amount to translate by
             * @return The transformed basis matrix
             */
            Transform translateY(double translation) const;

            /**
             * @brief Translate the current basis along the local Z axis
             *
             * This translates along the column vector submatrix(0,2,2,2)
             *
             * @param translation The amount to translate by
             * @return The transformed basis matrix
             */
            Transform translateZ(double translation) const;

            /**
             * @brief Rotates basis matrix around the local X axis
             *
             * @param radians The amount to radians to rotate by
             * @return The transformed basis matrix
             */
            Transform rotateX(double radians) const;

            /**
             * @brief Rotates basis matrix around the local Y axis
             *
             * @param radians The amount to radians to rotate by
             * @return The transformed basis matrix
             */
            Transform rotateY(double radians) const;

            /**
             * @brief Rotates basis matrix around the local Z axis
             *
             * @param radians The amount to radians to rotate by
             * @return The transformed basis matrix
             */
            Transform rotateZ(double radians) const;

            /**
             * @brief Transforms current basis from world coordinates (i.e. standard basis) to be local to 'reference'
             *
             * @param reference A basis matrix to become relatively local to
             * @return The transformed basis matrix
             */
            Transform worldToLocal(const Transform& reference) const;

            /**
             * @brief Transforms current basis from local coordinates relative to 'reference', to world coordinates (i.e. standard basis)
             *
             * @param reference The basis matrix that the current basis is relative to
             * @return The transformed basis matrix
             */
            Transform localToWorld(const Transform& reference) const;

            /**
             * @brief Performs an orthonormal inverse and returns a new copy
             * Note: Assumes current transform is orthonormal and invertible (which it should be given normal use)
             * Note: Unlike most methods this returns a new transform and does not modify the current transform
             *
             * @return The inverse transform
             */
            Transform i() const;

            /**
             * @brief Creates a translation transform by the given 3D vector
             *
             * @param translation The 3D translation vector to translate by
             * @return The translation transform
             */
            static Transform createTranslation(const arma::vec3& translation);

            /**
             * @brief Creates a rotation transform around the X axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation transform
             */
            static Transform createRotationX(double radians);

            /**
             * @brief Creates a rotation transform around the Y axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation transform
             */
            static Transform createRotationY(double radians);

            /**
             * @brief Creates a rotation transform around the Z axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation transform
             */
            static Transform createRotationZ(double radians);
    };

}  // matrix
}  // math
}  // utility

#endif  // UTILITY_MATH_TRANSFORM_H

