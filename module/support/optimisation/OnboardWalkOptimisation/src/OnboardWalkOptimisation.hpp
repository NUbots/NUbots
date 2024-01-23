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
#ifndef MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
#define MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP

#include <nuclear>

namespace module::support::optimisation {

    class OnboardWalkOptimisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Threshold angle for standing stably, from torso z axis to world z axis.
            float standing_angle = 0.0f;

            /// @brief Time required to be standing before starting a new individual.
            float wait_time = 0.0f;
        } cfg;

        /// @brief Indicates that the optimisation needs resetting for the next individual.
        bool resetting = false;

        /// @brief Used for stable stand, indicates whether we are still standing upright.
        bool is_upright = false;

        /// @brief Used to make sure the robot has been standing for enough time before starting.
        NUClear::clock::time_point start_time{NUClear::clock::now()};

    public:
        /// @brief Called by the powerplant to build and setup the OnboardWalkOptimisation reactor.
        explicit OnboardWalkOptimisation(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_ONBOARDWALKOPTIMISATION_HPP
