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

#ifndef UTILITY_MATH_STATS_RESAMPLE_RESAMPLE_BASE_HPP
#define UTILITY_MATH_STATS_RESAMPLE_RESAMPLE_BASE_HPP

#include <algorithm>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

// Resampling techniques!!!
// http://users.isy.liu.se/rt/schon/Publications/HolSG2006.pdf
namespace utility::math::stats::resample {

    /**
     * @brief The core resampling algorithm for multinomial, stratified, and systematic resampling schemes.
     *
     * @details The algorithm takes the following steps
     *  1. Normalise weights so they sum to 1
     *  2. Calculate cumulative sum of the normalised weights
     *  3. For each of the count particles
     *   a. Get a new resampling weight from the generator
     *   b. Find the first index in the cumulative sum where the generated resampling weight is >= cumulative sum
     *
     * The index in step 3b is the index of the particle that should be sampled
     *
     * @tparam Generator Type of the generator to use for resampling weights
     * @tparam Iterator The iterator type for the weights array
     *
     * @param count The number of particles being resampled
     * @param begin Iterator to the first weight to use for resampling
     * @param end Iterator to the last weight to use for resampling
     */
    template <template <typename> class Generator, typename Iterator>
    [[nodiscard]] std::vector<int> resample(const int& count, Iterator begin, Iterator end) {
        using Scalar = std::remove_reference_t<decltype(*begin)>;

        // Get number of weights
        const auto n_weights = std::distance(begin, end);

        // Create our distribution generator
        Generator<Scalar> fn(count);

        // Normalise the weights
        std::vector<Scalar> normalised(n_weights, Scalar(0));
        const Scalar sum = std::accumulate(begin, end, Scalar(0));
        std::transform(begin, end, normalised.begin(), [&](const Scalar& w) { return w / sum; });

        // Make a cumulative sum of the weights
        std::vector<Scalar> cumsum(n_weights, Scalar(0));
        std::partial_sum(normalised.begin(), normalised.end(), cumsum.begin());

        // Randomly sample from the distribution using our random number generator
        std::vector<int> idx(count, 0);
        for (int i = 0; i < count; ++i) {
            idx[i] = std::distance(cumsum.begin(), std::lower_bound(cumsum.begin(), cumsum.end(), fn(i)));
        }

        return idx;
    }

}  // namespace utility::math::stats::resample

#endif  // UTILITY_MATH_STATS_RESAMPLE_RESAMPLE_BASE_HPP
