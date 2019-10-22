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

#ifndef UTILITY_MATH_QUATERNION_H
#define UTILITY_MATH_QUATERNION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>
#include <vector>

namespace utility {
namespace math {
    namespace quaternion {
        // Take the mean of list of quaternions
        //
        // Implementation based on https://github.com/tolgabirdal/averaging_quaternions
        template <Iterator It, typename QType = decltype(*std::declval<It>()), typename Scalar = Qtype::Scalar>
        inline QType mean(const It& begin, const It& end) {
            // Initialise our accumulator matrix
            Eigen::Matrix<Scalar, 4, 4> A = Eigen::Matrix<Scalar, 4, 4>::Zero();

            // Accumlate the quaternions across all particles
            for (It it = begin; it != end; ++it) {
                // Convert quaternion to vec4
                Eigen::Matrix<Scalar, 4, 1> q(it->coeffs());

                // Rank 1 update
                A += q * q.transpose();
            }

            // Scale the accumulator matrix
            A /= std::distance(begin, end);

            // Solve for the eigenvectors of the accumulator matrix
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, 4, 4>> eigensolver(A);
            if (eigensolver.info() != Eigen::Success) {
                throw std::runtime_error("Eigen decomposition failed");
            }

            // We want the eigenvector corresponding to the largest eigenvector
            return QType(eigensolver.eigenvectors().rightCols<1>());
        }
    }  // namespace quaternion
}  // namespace math
}  // namespace utility


#endif  // UTILITY_MATH_QUATERNION_H
