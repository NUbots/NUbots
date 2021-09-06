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
#include <algorithm>

namespace utility::math {

    template <int str_len>
    struct [[nodiscard]] Space {

        consteval Space(const char (&str)[str_len]) {
            std::copy_n(str, str_len, value);
        }

        [[nodiscard]] consteval bool operator==(const Space& other) const {
            for (int i = 0; i < str_len; ++i) {
                if (value[i] != other.value[i]) {
                    return false;
                }
            }
            return true;
        }

        char value[str_len];
    };

    template <Space Into, Space From, typename Scalar = double, size_t Dim = 3>
    class [[nodiscard]] Transform {
    public:
        Eigen::Transform<Scalar, Dim, Eigen::Affine> transform = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();

        Transform() = default;
        Transform(Eigen::Transform<Scalar, Dim, Eigen::Affine> transform_) : transform(transform_) {}
        Transform(Eigen::Matrix<Scalar, Dim + 1, Dim + 1> transform_) : transform(transform_) {}

        template <Space OtherInto, Space OtherFrom>
        [[nodiscard]] Transform<Into, OtherFrom, Scalar, Dim> operator*(
            const Transform<OtherInto, OtherFrom, Scalar, Dim>& other) const {
            static_assert(From == OtherInto,
                          "Incompatible spaces used in transform multiplication. "
                          "Left Transform's From Space does not match right Transform's Into Space.");

            return Transform<Into, OtherFrom, Scalar, Dim>(
                Eigen::Transform<Scalar, Dim, Eigen::Affine>(transform * other.transform));
        }

        template <Space NewFrom>
        [[nodiscard]] Transform<Into, NewFrom, Scalar, Dim> cast_from_space() const {
            return Transfrom<Into, NewFrom, Scalar, Dim>(transform);
        }

        template <Space NewInto>
        [[nodiscard]] Transform<NewInto, From, Scalar, Dim> cast_into_space() const {
            return Transform<NewInto, From, Scalar, Dim>(transform);
        }

        [[nodiscard]] Transform<From, Into, Scalar, Dim> inverse() const {
            return Transform<From, Into, Scalar, Dim>(transform.inverse());
        }

        [[nodiscard]] Eigen::Matrix<Scalar, Dim, 1>& translation() {
            return transform.translation();
        }

        [[nodiscard]] const Eigen::Matrix<Scalar, Dim, 1>& translation() const {
            return transform.translation();
        }

        [[nodiscard]] Eigen::Matrix<Scalar, Dim, Dim>& rotation() {
            return transform.linear();
        }

        [[nodiscard]] const Eigen::Matrix<Scalar, Dim, Dim>& rotation() const {
            return transform.rotation();
        }
    };

}  // namespace utility::math

#endif  // UTILITY_MATH_TRANSFORM_HPP
