/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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

#ifndef UTILITY_MATH_FILTER_RESAMPLE_SYSTEMATIC_HPP
#define UTILITY_MATH_FILTER_RESAMPLE_SYSTEMATIC_HPP

#include <algorithm>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "resample.hpp"

namespace utility::math::stats::resample {

    /**
     * @brief Systematic resampling. See [1] for details on the algorithm (a copy is in
     * doc/Localisation/On_Resampling_Algorithms_for_Particle_Filters.pdf)
     *
     * [1]J. Hol, T. Schon and F. Dustafsson, "On Resampling Algorithms for Particle Filters", in 2006 IEEE Nonlinear
     * Statistical Signal Processing Workshop, Cambridge. UK, 2006.
     *
     * @tparam Scalar The scalar type to used for the particle weights
     */
    template <typename Scalar>
    struct Systematic {
        /**
         * @brief Systematic resampling weight generator
         *
         * @param count_ Number of particles being resampled
         */
        Systematic(const int& count_)
            : gen((std::random_device()())), dist(Scalar(0.0), Scalar(1.0)), count(count_), rng(dist(gen)) {}

        /**
         * @brief Calculate the next resampling weight
         *
         * @param i Particle index being resampling
         */
        [[nodiscard]] Scalar operator()(const int& i) {
            return (Scalar(i) + rng) / count;
        }

    private:
        /// @brief Uniform distribution on the interval [0, 1)
        std::mt19937 gen;
        std::uniform_real_distribution<Scalar> dist;

        /// @brief Number of particles being resampled
        Scalar count;
        /// @brief Systematic resampling uses a static weight offset
        Scalar rng;
    };

    /**
     * @brief Call the resampler algorithm using the above resampling weight calculation
     *
     * @param count Number of particles being resampled
     * @param begin Iterator to the first weight to use for resampling
     * @param end Iterator to the last weight to use for resampling
     */
    template <typename Iterator>
    [[nodiscard]] std::vector<int> systematic(const int& count, Iterator&& begin, Iterator&& end) {
        return resample<Systematic>(count, std::forward<Iterator>(begin), std::forward<Iterator>(end));
    }

}  // namespace utility::math::stats::resample

#endif  // UTILITY_MATH_FILTER_RESAMPLE_SYSTEMATIC_HPP
