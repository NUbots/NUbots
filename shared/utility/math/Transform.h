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
             * @brief Translate the current basis by the given 3D vector
             *
             * @param translation The 3D translation vector to translate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& translate(const arma::vec3& translation);
            /*
             * @brief Translate the current basis along the local X axis
             *
             * This translates along the column vector submatrix(0,0,2,0)
             *
             * @param translation The amount to translate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& translateX(double translation);
            /**
             * @brief Translate the current basis along the local Y axis
             *
             * This translates along the column vector submatrix(0,1,2,1)
             *
             * @param translation The amount to translate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& translateY(double translation);
            /**
             * @brief Translate the current basis along the local Z axis
             *
             * This translates along the column vector submatrix(0,2,2,2)
             *
             * @param translation The amount to translate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& translateZ(double translation);
            /**
             * @brief Rotates basis matrix around the local X axis
             *
             * @param radians The amount to radians to rotate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& rotateX(double radians);
            /**
             * @brief Rotates basis matrix around the local Y axis
             *
             * @param radians The amount to radians to rotate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& rotateY(double radians);
            /**
             * @brief Rotates basis matrix around the local Z axis
             *
             * @param radians The amount to radians to rotate by
             * @return A reference to *this, to be used for chaining
             */
            Transform& rotateZ(double radians);
            /**
             * @brief Transforms current basis from world coordinates (i.e. standard basis) to be local to 'reference'
             *
             * Pseudocode: this = reference.inverse() * this
             *
             * @param reference A basis matrix to become relatively local to
             * @return A reference to *this, to be used for chaining
             */
            Transform& worldToLocal(const Transform& reference);
            /**
             * @brief Transforms current basis from local coordinates relative to 'reference', to world coordinates (i.e. standard basis)
             *
             * Pseudocode: this = reference * this
             *
             * @param reference A basis matrix to become relatively local to
             * @return A reference to *this, to be used for chaining
             */
            Transform& localToWorld(const Transform& reference);
            /**
             * @brief Performs an orthonormal inverse and returns a new copy
             * Note: Assumes current transform is orthonormal and invertible (which it should be given normal use)
             * Note: Unlike most methods this returns a new transform and does not modify the current transform
             *
             * @return The inverse transform
             */
            Transform i() const;
    };

}  // math
}  // utility

#endif  // UTILITY_MATH_TRANSFORM_H

