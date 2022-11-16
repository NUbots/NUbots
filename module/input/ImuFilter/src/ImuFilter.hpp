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

#ifndef MODULES_INPUT_IMUFILTER_HPP
#define MODULES_INPUT_IMUFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "utility/math/filter/KalmanFilter.hpp"

namespace module::input {

    class ImuFilter : public NUClear::Reactor {
    public:
        explicit ImuFilter(std::unique_ptr<NUClear::Environment> environment);

        struct Config {
            Config() = default;

        } cfg;

    private:
        // Define the model dimensions
        static const size_t n_states  = 3;
        static const size_t n_inputs  = 0;
        static const size_t n_outputs = 2;

        // Define filter
        utility::math::filter::KalmanFilter<double, n_states, n_inputs, n_outputs> filter{};

        // Time of last update
        NUClear::clock::time_point last_update_time;
    };
}  // namespace module::input
#endif  // MODULES_INPUT_IMUFILTER_HPP
