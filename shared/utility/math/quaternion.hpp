/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
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

#ifndef UTILITY_MATH_QUATERNION_HPP
#define UTILITY_MATH_QUATERNION_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace utility::math::quaternion {
    // Take the mean of list of quaternions
    //
    // Implementation based on https://github.com/tolgabirdal/averaging_quaternions
    template <typename Iterator,
              typename QType  = std::remove_cv_t<std::remove_reference_t<decltype(*std::declval<Iterator>())>>,
              typename Scalar = typename QType::Scalar>
    inline QType mean(const Iterator& begin, const Iterator& end) {
        if (std::distance(begin, end) == 0) {
            return QType::Identity();
        }
        // Initialise our accumulator matrix
        Eigen::Matrix<Scalar, 4, 4> A = Eigen::Matrix<Scalar, 4, 4>::Zero();

        // Accumulate the quaternions across all particles
        for (Iterator it = begin; it != end; ++it) {
            // Convert quaternion to vec4
            Eigen::Matrix<Scalar, 4, 1> q(it->coeffs());

            // Rank 1 update
            A += q * q.transpose();
        }

        // Scale the accumulator matrix
        A /= static_cast<Scalar>(std::distance(begin, end));

        // Solve for the eigenvectors of the accumulator matrix
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 4, 4>> eigensolver(A);
        if (eigensolver.info() != Eigen::Success) {
            // Failed to get the mean using Merkley's method, try just averaging naively
            Eigen::Vector4d mean = Eigen::Vector4d::Zero();
            for (Iterator it = begin; it != end; ++it) {
                mean += it->coeffs();
            }
            mean /= static_cast<Scalar>(std::distance(begin, end));
            return QType(mean).normalized();
        }

        // We want the eigenvector corresponding to the largest eigenvector
        QType mean = QType(eigensolver.eigenvectors().template rightCols<1>());
        return mean.normalized();
    }

    // Normalises to ensure scalar component is non-negative
    template <typename Iterator,
              typename QType  = std::remove_cv_t<std::remove_reference_t<decltype(*std::declval<Iterator>())>>,
              typename Scalar = typename QType::Scalar>
    inline QType meanRotation(const Iterator& begin, const Iterator& end) {
        if (std::distance(begin, end) == 0) {
            return QType::Identity();
        }
        // Initialise our accumulator matrix
        Eigen::Matrix<Scalar, 4, 4> A = Eigen::Matrix<Scalar, 4, 4>::Zero();

        // Accumulate the quaternions across all particles
        for (Iterator it = begin; it != end; ++it) {
            // Convert quaternion to vec4
            Eigen::Matrix<Scalar, 4, 1> q(it->coeffs());
            if (q.w() < Scalar(0)) {
                q *= Scalar(-1);
            }

            // Rank 1 update
            A += q * q.transpose();
        }

        // Scale the accumulator matrix
        A /= static_cast<Scalar>(std::distance(begin, end));

        // Solve for the eigenvectors of the accumulator matrix
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 4, 4>> eigensolver(A);
        if (eigensolver.info() != Eigen::Success) {
            // Failed to get the mean using Merkley's method, try just averaging naively
            Eigen::Vector4d mean = Eigen::Vector4d::Zero();
            for (Iterator it = begin; it != end; ++it) {
                mean += it->coeffs();
            }
            mean /= static_cast<Scalar>(std::distance(begin, end));
            if (mean.w() < Scalar(0)) {
                mean *= Scalar(-1);
            }
            return QType(mean).normalized();
        }

        // We want the eigenvector corresponding to the largest eigenvector
        QType mean = QType(eigensolver.eigenvectors().template rightCols<1>());
        if (mean.w() < Scalar(0)) {
            mean.w() *= Scalar(-1);
            mean.vec() *= Scalar(-1);
        }
        return mean.normalized();
    }

    template <typename QType>
    inline QType difference(const QType& a, const QType& b) {
        // Difference between two rotations
        // Calculate the rotation from a to b
        // https://www.gamedev.net/forums/topic/423462-rotation-difference-between-two-quaternions/?do=findComment&comment=3818213
        return a.inverse() * b;
    }
}  // namespace utility::math::quaternion


#endif  // UTILITY_MATH_QUATERNION_HPP
