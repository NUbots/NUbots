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

#ifndef UTILITY_SLAM_GAUSSIAN_DENSITY_BASE_HPP
#define UTILITY_SLAM_GAUSSIAN_DENSITY_BASE_HPP

#include <cmath>

#include <Eigen/Core>

namespace utility::slam::gaussian {

    /**
     * @brief Base class for probability density functions.
     *
     * @tparam Scalar The scalar type used for calculations (default: double).
     */
    template <typename Scalar = double>
    class DensityBase {
    public:
        /**
         * @brief Virtual destructor.
         */
        virtual ~DensityBase() = default;

        /**
         * @brief Computes the log of the probability density function.
         *
         * @param x The input vector.
         * @return The log of the probability density at x.
         */
        virtual Scalar log(const Eigen::VectorX<Scalar>& x) const = 0;

        /**
         * @brief Evaluates the probability density function.
         *
         * @param x The input vector.
         * @return The probability density at x.
         */
        Scalar eval(const Eigen::VectorX<Scalar>& x) const {
            using std::exp;
            return exp(log(x));
        }
    };

}  // namespace utility::slam::gaussian

#endif  // UTILITY_SLAM_GAUSSIAN_DENSITY_BASE_HPP
