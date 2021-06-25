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

#ifndef UTILITY_MATH_STATS_RESAMPLE_MULTINOMIAL_HPP
#define UTILITY_MATH_STATS_RESAMPLE_MULTINOMIAL_HPP

#include <algorithm>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "resample.hpp"

// Resampling techniques!!!
// http://users.isy.liu.se/rt/schon/Publications/HolSG2006.pdf
namespace utility::math::stats::resample {

    template <typename Scalar>
    struct Multinomial {
        Multinomial(const int& /*count*/) : gen((std::random_device()())), dist(Scalar(0.0), Scalar(1.0)) {}

        [[nodiscard]] Scalar operator()(const int& /*i*/) {
            return dist(gen);
        }

    private:
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_real_distribution<Scalar> dist;
    };

    template <typename Iterator>
    [[nodiscard]] std::vector<int> multinomial(const int& count, Iterator&& begin, Iterator&& end) {
        return resample<Multinomial>(count, std::forward<Iterator>(begin), std::forward<Iterator>(end));
    }

}  // namespace utility::math::stats::resample

#endif  // UTILITY_MATH_STATS_RESAMPLE_MULTINOMIAL_HPP
