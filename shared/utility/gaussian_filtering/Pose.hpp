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

#ifndef UTILITY_SLAM_POSE_HPP
#define UTILITY_SLAM_POSE_HPP

#include <Eigen/Core>

namespace utility::gaussian_filtering {

    /**
     * @brief A rigid-body transform: a rotation and a translation.
     *
     * Stands in for the 4x4 homogeneous matrix [R r; 0 1] without ever forming it,
     * so composition and point transforms stay 3x3.
     *
     * @tparam Scalar The scalar type (default: double)
     */
    template <typename Scalar = double>
    struct Pose {
        using Matrix3 = Eigen::Matrix3<Scalar>;
        using Vector3 = Eigen::Vector3<Scalar>;

        Matrix3 rotationMatrix;     ///< Rab, rotating a vector from frame {b} to frame {a}
        Vector3 translationVector;  ///< rBAa, the origin of {b} relative to {a}, expressed in {a}

        /**
         * @brief Default constructor: identity rotation, zero translation.
         */
        Pose() : rotationMatrix(Matrix3::Identity()), translationVector(Vector3::Zero()) {}

        /**
         * @brief Constructor from a rotation matrix and translation vector
         * @param R Rotation matrix
         * @param t Translation vector
         */
        Pose(const Matrix3& R, const Vector3& t) : rotationMatrix(R), translationVector(t) {}

        /**
         * @brief Copy constructor with scalar type conversion, for switching between
         * double and an autodiff dual type.
         *
         * @tparam OtherScalar The scalar type of the input Pose
         * @param T The input Pose object to copy and convert
         */
        template <typename OtherScalar>
        Pose(const Pose<OtherScalar>& T)
            : rotationMatrix(T.rotationMatrix.template cast<Scalar>())
            , translationVector(T.translationVector.template cast<Scalar>()) {}

        /**
         * @brief Compose two transforms: Tac = Tab * Tbc.
         *
         * @param other The other pose to compose with
         * @return The resulting composed pose
         */
        Pose operator*(const Pose& other) const {
            Pose result;
            result.rotationMatrix    = rotationMatrix * other.rotationMatrix;
            result.translationVector = rotationMatrix * other.translationVector + translationVector;
            return result;
        }

        /**
         * @brief Map a point into the other frame: rPAa = Rab * rPBb + rBAa.
         *
         * @param r The point to transform
         * @return The transformed point
         */
        Vector3 operator*(const Vector3& r) const {
            return rotationMatrix * r + translationVector;
        }

        /**
         * @brief Inverse transform Tba, using the transpose rather than a matrix
         * inverse since the rotation is orthonormal.
         *
         * @return The inverse pose
         */
        Pose inverse() const {
            Pose result;
            result.rotationMatrix    = rotationMatrix.transpose();
            result.translationVector = -result.rotationMatrix * translationVector;
            return result;
        }
    };

}  // namespace utility::gaussian_filtering

#endif  // UTILITY_SLAM_POSE_HPP
