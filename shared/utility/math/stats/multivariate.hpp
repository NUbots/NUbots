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

#ifndef UTILITY_MATH_STATS_MULTIVARIATE_HPP
#define UTILITY_MATH_STATS_MULTIVARIATE_HPP

#include <Eigen/Core>
#include <random>

namespace utility::math::stats {

    // Implementation taken from
    // https://stackoverflow.com/a/40245513/4795763
    template <typename Scalar, int N>
    struct MultivariateNormal {
        // Must construct with at least a covariance matrix
        MultivariateNormal() = delete;

        MultivariateNormal(const Eigen::Matrix<Scalar, N, N>& covariance)
            : MultivariateNormal(Eigen::Matrix<Scalar, N, 1>::Zero(), covariance) {}
        MultivariateNormal(const Eigen::Matrix<Scalar, N, 1>& mean_, const Eigen::Matrix<Scalar, N, N>& covariance)
            : mean(mean_), gen(rd()) {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, N, N>> solver(covariance);
            transform = solver.eigenvectors() * solver.eigenvalues().cwiseSqrt().asDiagonal();
        }

        [[nodiscard]] Eigen::Matrix<Scalar, N, 1> operator()() {
            return mean + transform * Eigen::Matrix<Scalar, N, 1>().NullaryExpr([this]() { return dist(gen); });
        }

    private:
        Eigen::Matrix<Scalar, N, 1> mean      = Eigen::Matrix<Scalar, N, 1>::Zero();
        Eigen::Matrix<Scalar, N, N> transform = Eigen::Matrix<Scalar, N, N>::Zero();

        std::random_device rd{};
        std::mt19937 gen;
        std::normal_distribution<Scalar> dist;
    };

}  // namespace utility::math::stats

#endif  //  UTILITY_MATH_STATS_MULTIVARIATE_HPP
