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
#ifndef MODULE_STRATEGY_WALKINSIDEBOUNDEDBOX_HPP
#define MODULE_STRATEGY_WALKINSIDEBOUNDEDBOX_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class WalkInsideBoundedBox : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            /// @brief Bounded box of region to defend the ball - x min, x max, y max, y min
            double bounded_region_x_min = 0.0;
            double bounded_region_x_max = 0.0;
            double bounded_region_y_min = 0.0;
            double bounded_region_y_max = 0.0;
            /// @brief Length of time before the ball detection is too old and we should search for the ball
            NUClear::clock::duration ball_search_timeout{};
            /// @brief Ready position to walk to (x, y, theta)
            Eigen::Vector3d ready_position = Eigen::Vector3d::Zero();

        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the Defend reactor.
        explicit WalkInsideBoundedBox(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKINSIDEBOUNDED_HPP
