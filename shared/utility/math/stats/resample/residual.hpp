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

#ifndef UTILITY_MATH_STATS_RESAMPLE_RESIDUAL_HPP
#define UTILITY_MATH_STATS_RESAMPLE_RESIDUAL_HPP

#include <algorithm>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

// Resampling techniques!!!
// http://users.isy.liu.se/rt/schon/Publications/HolSG2006.pdf
namespace utility::math::stats::resample {

    /**
     * @brief Residual resampling. See [1] for details on the algorithm (a copy is in
     * doc/Localisation/On_Resampling_Algorithms_for_Particle_Filters.pdf)
     *
     * [1]J. Hol, T. Schon and F. Dustafsson, "On Resampling Algorithms for Particle Filters", in 2006 IEEE Nonlinear
     * Statistical Signal Processing Workshop, Cambridge. UK, 2006.
     *
     * @details The algorithm takes the following steps
     *  1. Normalise the weights so they sum to 1
     *  2. Scale the normalised weights by the number of particles being resampled and floor them (take integer
     *      component)
     *   - These scaled weights are counts of the number of times each particle should be resmapled (i.e. particles are
     *      duplicated)
     *  3. Resample the particles according to the counts obtained in the previous step
     *  4. Determine how many particles are left to resample (these particles are the residuals)
     *  5. Calculate the residual weights
     *  6. Use one of the other resampling methods (e.g. multinomial, stratified, systematic) to resample the residual
     *      particles
     *
     * @tparam Scalar The scalar type to used for the particle weights
     * @tparam ResidualSampler The type of the sampler to use for sampling the residual particles
     *
     * @param count Number of particles being resampled
     * @param begin Iterator to the first weight to use for resampling
     * @param end Iterator to the last weight to use for resampling
     * @param residual_sample The sampler to use when resampling the residual particles
     */
    template <typename Iterator, typename ResidualSampler>
    std::vector<int> residual(int count, Iterator begin, Iterator end, ResidualSampler&& residual_sampler) {
        using Scalar = typename std::iterator_traits<Iterator>::value_type;

        // Get number of weights
        const auto n_weights = std::distance(begin, end);

        // Normalize the weights and multiply by count
        std::vector<Scalar> scaled_weights(n_weights);
        const Scalar sum_weights = std::accumulate(begin, end, Scalar(0));
        std::transform(begin, end, scaled_weights.begin(), [count, sum_weights](const Scalar& w) {
            return w * count / sum_weights;
        });

        // Calculate integer parts and residuals
        std::vector<int> indices;
        std::vector<Scalar> residuals(n_weights);
        int total_int_parts = 0;

        for (int i = 0; i < n_weights; ++i) {
            int int_part = static_cast<int>(scaled_weights[i]);
            indices.insert(indices.end(), int_part, i);
            total_int_parts += int_part;
            residuals[i] = scaled_weights[i] - int_part;
        }

        // Determine how many additional samples to draw
        int residual_count = count - total_int_parts;

        // Use the residual sampler to sample additional indices based on residuals
        if (residual_count > 0) {
            auto additional_indices = residual_sampler(residual_count, residuals.begin(), residuals.end());
            indices.insert(indices.end(), additional_indices.begin(), additional_indices.end());
        }

        return indices;
    }

}  // namespace utility::math::stats::resample

#endif  // UTILITY_MATH_STATS_RESAMPLE_RESIDUAL_HPP
