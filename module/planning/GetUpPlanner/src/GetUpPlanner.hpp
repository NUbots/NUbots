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
#ifndef MODULE_PLANNING_GETUPPLANNER_HPP
#define MODULE_PLANNING_GETUPPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class GetUpPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Threshold angle for executing getup, between torso z axis and world z axis
            float fallen_angle = 0.0f;
            int delay_time = 1;
            std::chrono::seconds start_delay = std::chrono::seconds(delay_time);
        } cfg;

        struct StartGetUp {};

    public:
        /// @brief Called by the powerplant to build and setup the GetUpPlanner reactor.
        explicit GetUpPlanner(std::unique_ptr<NUClear::Environment> environment);
    };


}  // namespace module::planning


#endif  // MODULE_PLANNING_GETUPPLANNER_HPP
