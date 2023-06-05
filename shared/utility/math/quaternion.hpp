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
            Eigen::Matrix<Scalar, 4, 1> mean = Eigen::Matrix<Scalar, 4, 1>::Zero();
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
