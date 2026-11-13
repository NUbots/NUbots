/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef UTILITY_GAUSSIAN_FILTERING_POSE_HPP
#define UTILITY_GAUSSIAN_FILTERING_POSE_HPP

#include <Eigen/Core>

namespace utility::gaussian_filtering {

    /**
     * @brief A rigid-body transform: a rotation and a translation.
     *
     * Stands in for the 4x4 homogeneous matrix [R r; 0 1] without forming it.
     *
     * @tparam Scalar The scalar type (default: double)
     */
    template <typename Scalar = double>
    struct Pose {
        using Matrix3 = Eigen::Matrix3<Scalar>;
        using Vector3 = Eigen::Vector3<Scalar>;

        Matrix3 rotation_matrix;     ///< Rab, rotating a vector from frame {b} to frame {a}
        Vector3 translation_vector;  ///< rBAa, the origin of {b} relative to {a}, expressed in {a}

        /// @brief Identity rotation, zero translation
        Pose() : rotation_matrix(Matrix3::Identity()), translation_vector(Vector3::Zero()) {}

        Pose(const Matrix3& R, const Vector3& t) : rotation_matrix(R), translation_vector(t) {}

        /// @brief Copy with scalar type conversion, for switching between double and an
        ///        autodiff dual type
        template <typename OtherScalar>
        Pose(const Pose<OtherScalar>& T)
            : rotation_matrix(T.rotation_matrix.template cast<Scalar>())
            , translation_vector(T.translation_vector.template cast<Scalar>()) {}

        /// @brief Compose two transforms: Tac = Tab * Tbc
        Pose operator*(const Pose& other) const {
            Pose result;
            result.rotation_matrix    = rotation_matrix * other.rotation_matrix;
            result.translation_vector = rotation_matrix * other.translation_vector + translation_vector;
            return result;
        }

        /// @brief Map a point into the other frame: rPAa = Rab * rPBb + rBAa
        Vector3 operator*(const Vector3& r) const {
            return rotation_matrix * r + translation_vector;
        }

        /// @brief Inverse transform Tba
        Pose inverse() const {
            Pose result;
            result.rotation_matrix    = rotation_matrix.transpose();
            result.translation_vector = -result.rotation_matrix * translation_vector;
            return result;
        }
    };

}  // namespace utility::gaussian_filtering

#endif  // UTILITY_GAUSSIAN_FILTERING_POSE_HPP
