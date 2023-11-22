/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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

#ifndef UTILITY_MATH_FILTER_EXPONENTIAL_HPP
#define UTILITY_MATH_FILTER_EXPONENTIAL_HPP

#include <Eigen/Core>
#include <algorithm>

namespace utility::math::filter {

    /**
     * @brief ExponentialFilter is a type of filter that applies a smoothing factor (alpha) to data. The alpha value
     * ranges between 0 and 1 and determines the weight given to new data versus historical data. A higher alpha makes
     * the filter more responsive to recent changes, while a lower alpha makes it smoother and less responsive to
     * short-term fluctuations. https://en.wikipedia.org/wiki/Exponential_smoothing
     */
    template <typename Scalar, int Size>
    class ExponentialFilter {
    public:
        /// @brief Default constructor for ExponentialFilter
        ExponentialFilter() = default;

        /**
         * @brief Construct an ExponentialFilter with the given alpha and zero initial value
         *
         * @param alpha Smoothing factor
         */
        explicit ExponentialFilter(Scalar alpha)
            : alpha(std::clamp(alpha, Scalar(0), Scalar(1))), filtered_value(Eigen::Matrix<Scalar, Size, 1>::Zero()) {}

        /**
         * @brief Construct an ExponentialFilter with the given parameters
         *
         * @param alpha Smoothing factor
         * @param initial_value Initial value for the filter
         */
        ExponentialFilter(Scalar alpha, Eigen::Matrix<Scalar, Size, 1> initial_value)
            : alpha(std::clamp(alpha, Scalar(0), Scalar(1))), filtered_value(initial_value) {}

        /**
         * @brief Update the filter with a new measurement and return the smoothed value.
         *
         * @param measurement The new measurement to incorporate.
         * @return The smoothed value after the update.
         */
        Eigen::Matrix<Scalar, Size, 1> update(const Eigen::Matrix<Scalar, Size, 1>& measurement) {
            filtered_value = alpha * measurement + (Scalar(1) - alpha) * filtered_value;
            return filtered_value;
        }

        /// @brief Get the current filtered value
        const Eigen::Matrix<Scalar, Size, 1>& get_value() const {
            return filtered_value;
        }

        /// @brief Set the current filtered value
        void set_value(const Eigen::Matrix<Scalar, Size, 1>& new_value) {
            filtered_value = new_value;
        }

        /// @brief Set the smoothing factor (alpha)
        void set_alpha(const Scalar& new_alpha) {
            alpha = std::clamp(new_alpha, Scalar(0), Scalar(1));
        }

    private:
        /// @brief Smoothing factor
        Scalar alpha = Scalar(0.1);

        /// @brief Current filtered value
        Eigen::Matrix<Scalar, Size, 1> filtered_value = Eigen::Matrix<Scalar, Size, 1>::Zero();
    };

}  // namespace utility::math::filter

#endif
