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
#ifndef MODULE_PLANNING_PLANLOOK_HPP
#define MODULE_PLANNING_PLANLOOK_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class PlanLook : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief How long to look at each search position
            float search_fixation_time = 0.0f;
            /// @brief List of search positions for the search pattern
            std::vector<Eigen::Vector2d> search_positions{};
        } cfg;

        /// @brief  Time since last search position transition
        NUClear::clock::time_point search_last_moved = NUClear::clock::now();

        /// @brief Current index in the list of search positions
        long unsigned int search_idx = 0;

    public:
        /// @brief Called by the powerplant to build and setup the PlanLook reactor.
        explicit PlanLook(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_PLANLOOK_HPP
