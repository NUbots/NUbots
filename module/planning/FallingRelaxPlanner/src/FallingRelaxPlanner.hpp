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
#ifndef MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
#define MODULE_PLANNING_FALLINGRELAXPLANNER_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::planning {

    class FallingRelaxPlanner : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {

            struct Levels {
                /// @brief The mean value of this sensor to subtract
                double mean;
                /// @brief The threshold for this sensor to be considered unstable
                double unstable;
                /// @brief The threshold for this sensor to be considered falling
                double falling;
                /// @brief The smoothing factor for this sensor
                double smoothing;
            };

            /// @brief The configuration for the gyroscope magnitude check
            Levels gyro_mag;
            /// @brief The configuration for the accelerometer magnitude check
            Levels acc_mag;
            /// @brief The configuration for the accelerometer angle check
            Levels acc_angle;
            /// @brief The script to run when falling
            std::string fall_script;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the FallingRelaxPlanner reactor.
        explicit FallingRelaxPlanner(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief the current smoothed value of the gyroscope magnitude
        double gyro_mag = 0.0;
        /// @brief the current smoothed value of the accelerometer magnitude
        double acc_mag = 0.0;
        /// @brief the current smoothed value of the accelerometer angle from upright
        double acc_angle = 0.0;
    };

}  // namespace module::planning

#endif  // MODULE_PLANNING_FALLINGRELAXPLANNER_HPP
