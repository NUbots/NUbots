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
#ifndef MODULE_STRATEGY_WALKTOREADYPOSITION_HPP
#define MODULE_STRATEGY_WALKTOREADYPOSITION_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class WalkToReadyPosition : public ::extension::behaviour::BehaviourReactor {
    private:
        struct Config {
            double stop_threshold    = 0.0;
            double stopped_threshold = 0.0;

            NUClear::clock::duration stand_duration{};
        } cfg;

        double current_threshold = 0.0;

        NUClear::clock::time_point start_time = NUClear::clock::now();
        bool waiting                          = false;

    public:
        /// @brief Called by the powerplant to build and setup the WalkToReadyPosition reactor.
        explicit WalkToReadyPosition(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_WALKTOREADYPOSITION_HPP