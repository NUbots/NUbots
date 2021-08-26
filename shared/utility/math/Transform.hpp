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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_TRANSFORM_HPP
#define UTILITY_MATH_TRANSFORM_HPP

#include <Eigen/Geometry>
#include <iostream>
#include <type_traits>

namespace utility::math {
    namespace space {
        static constexpr size_t TORSO  = 0;
        static constexpr size_t FIELD  = 1;
        static constexpr size_t CAMERA = 2;
        static constexpr size_t WORLD  = 3;
        static constexpr size_t GROUND = 4;
    }  // namespace space

    template <typename Scalar, int TO_SPACE, int FROM_SPACE>
    class Transform {
    public:
        Eigen::Transform<Scalar, 3, Eigen::Affine> transform = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();

        Transform() = default;
        Transform(Eigen::Transform<Scalar, 3, Eigen::Affine> transform_) : transform(transform_) {}

        template <int OTHER_TO_SPACE, int OTHER_FROM_SPACE>
        Transform<Scalar, TO_SPACE, OTHER_FROM_SPACE> operator*(
            const Transform<Scalar, OTHER_TO_SPACE, OTHER_FROM_SPACE>& other) {
            static_assert(FROM_SPACE == OTHER_TO_SPACE,
                          "Incompatible spaces used in transform multiplication. "
                          "Left Transform's FROM_SPACE does not match right Transform's TO_SPACE.");

            return Transform<Scalar, TO_SPACE, OTHER_FROM_SPACE>(
                Eigen::Transform<Scalar, 3, Eigen::Affine>(this->matrix() * other.matrix()));
        }

        // Matrix stuff
        // (Inverse will require the spaces to be flipped)

        [[nodiscard]] Eigen::Matrix<Scalar, 4, 4>& matrix() {
            return transform.matrix();
        }

        [[nodiscard]] const Eigen::Matrix<Scalar, 4, 4>& matrix() const {
            return transform.matrix();
        }
    };

}  // namespace utility::math

#endif  // UTILITY_MATH_TRANSFORM_HPP
