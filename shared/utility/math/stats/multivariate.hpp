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

#ifndef UTILITY_MATH_STATS_MULTIVARIATE_HPP
#define UTILITY_MATH_STATS_MULTIVARIATE_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <random>
#include <utility>

namespace utility::math::stats {

    /**
     * @author Alex Biddulph
     *
     * @brief N dimension multivariate normal distribution
     * Implementation taken from https://stackoverflow.com/a/40245513/4795763 and
     * https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     *
     * @tparam Scalar The scalar type to use for all calculations.
     * @tparam N The dimensionality of the multivariate distribution
     */
    template <typename Scalar, int N>
    struct MultivariateNormal {
    private:
        // Must construct with at least a covariance matrix
        MultivariateNormal() = delete;

    public:
        /**
         * @brief Construct a multivariate normal distribution of dimension N with zero mean and the provided covariance
         *
         * @param covariance_ The covariance matrix for this zero-mean multivariate normal distribution
         */
        MultivariateNormal(const Eigen::Matrix<Scalar, N, N>& covariance_)
            : MultivariateNormal(Eigen::Matrix<Scalar, N, 1>::Zero(), covariance_) {}

        /**
         * @brief Construct a multivariate normal distribution of dimension N with the provided mean and covariance
         *
         * @param mean_ The mean for this mean multivariate normal distribution
         * @param covariance_ The covariance matrix for this multivariate normal distribution
         */
        MultivariateNormal(Eigen::Matrix<Scalar, N, 1> mean_, const Eigen::Matrix<Scalar, N, N>& covariance_)
            : mean(std::move(mean_)), covariance(covariance_), gen(std::random_device()()) {

            // Calculate the spectral decomposition of the covariance matrix
            // This is needed MultivariateNormal::sample
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, N, N>> solver(covariance_);
            transform = solver.eigenvectors() * solver.eigenvalues().cwiseSqrt().asDiagonal();
        }

        /**
         * @brief Samples a N dimensional random vector from this N dimensional multivariate normal distribution
         *
         * @details To sample a random vector, x, from a N dimensional multivariate normal distribution, the following
         * steps are taken
         * 1. Find any real matrix A such that AA' = \Sigma
         *  - If \Sigma is positive-definite then the Cholesky decomposition can be used
         *  - An alternative is to use the A = U\Lambda^{1/2} obtained from the spectral decomposition \Sigma = U\Lambda
         *  U^{-1}
         * 2. Let z be an N dimensional vector whose components are sampled from a standard normal distribution
         * 3. Let x = \mu + Az
         * https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Drawing_values_from_the_distribution
         */
        [[nodiscard]] Eigen::Matrix<Scalar, N, 1> sample() {
            return mean + transform * Eigen::Matrix<Scalar, N, 1>().NullaryExpr([this]() { return dist(gen); });
        }

        /**
         * @brief Calculate the density of the multivariate normal distribution at the point x
         *
         * @details This calculation assumes that the covariance matrix is symmetric and positive definite. In this
         * case, the multivariate normal distribution is "non-degenerate".
         *
         * Details of the formula we are using can be found here
         * https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Non-degenerate_case
         *
         * This function can be used to determine the relative likelihood that x was sampled from this distribution.
         */
        [[nodiscard]] Scalar density(const Eigen::Matrix<Scalar, N, 1>& x) {
            const auto z = Scalar(-0.5) * ((x - mean).transpose() * covariance.inverse() * (x - mean))(0);
            return std::exp(z) / std::sqrt(std::pow(Scalar(2.0 * M_PI), N) * std::abs(covariance.determinant()));
        }

    private:
        /// @brief The mean (\mu) of the multivariate normal distribution
        Eigen::Matrix<Scalar, N, 1> mean = Eigen::Matrix<Scalar, N, 1>::Zero();
        /// @brief The covariance (\Sigma) of the multivariate normal distribution
        Eigen::Matrix<Scalar, N, N> covariance = Eigen::Matrix<Scalar, N, N>::Zero();
        /// @brief A matrix A such that AA' = \Sigma
        Eigen::Matrix<Scalar, N, N> transform = Eigen::Matrix<Scalar, N, N>::Zero();

        /// @brief Standard normal distribution. Used in MultivariateNormal::sample
        std::mt19937 gen;
        std::normal_distribution<Scalar> dist;
    };

}  // namespace utility::math::stats

#endif  //  UTILITY_MATH_STATS_MULTIVARIATE_HPP
