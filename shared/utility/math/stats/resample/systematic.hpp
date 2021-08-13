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
