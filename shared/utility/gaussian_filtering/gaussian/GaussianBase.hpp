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

#ifndef UTILITY_SLAM_GAUSSIAN_GAUSSIAN_BASE_HPP
#define UTILITY_SLAM_GAUSSIAN_GAUSSIAN_BASE_HPP

#include <Eigen/Core>

#include "DensityBase.hpp"

namespace utility::gaussian_filtering::gaussian {

    /**
     * @brief Base class for Gaussian distributions.
     *
     * This class provides a common interface and utility functions for Gaussian distributions.
     *
     * @tparam Scalar The scalar type used for calculations (default: double).
     */
    template <typename Scalar = double>
    class GaussianBase : public DensityBase<Scalar> {
    public:
        /**
         * @brief Virtual destructor.
         */
        virtual ~GaussianBase() override = default;

        /**
         * @brief Returns the dimension of the Gaussian distribution.
         * @return The dimension of the distribution.
         */
        virtual Eigen::Index dim() const = 0;

        /**
         * @brief Returns the mean of the Gaussian distribution.
         * @return The mean vector.
         */
        virtual Eigen::VectorX<Scalar> mean() const = 0;

        /**
         * @brief Returns the square root of the covariance matrix.
         * @return The square root of the covariance matrix.
         */
        virtual Eigen::MatrixX<Scalar> sqrtCov() const = 0;

        /**
         * @brief Returns the covariance matrix.
         * @return The covariance matrix.
         */
        virtual Eigen::MatrixX<Scalar> cov() const = 0;

        /**
         * @brief Returns the square root of the information matrix.
         * @return The square root of the information matrix.
         */
        virtual Eigen::MatrixX<Scalar> sqrtInfoMat() const = 0;
    };

}  // namespace utility::gaussian_filtering::gaussian

#endif  // UTILITY_SLAM_GAUSSIAN_GAUSSIAN_BASE_HPP
