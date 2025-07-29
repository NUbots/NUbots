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
#ifndef MODULE_STRATEGY_FINDOBJECT_HPP
#define MODULE_STRATEGY_FINDOBJECT_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::strategy {

    class FindObject : public ::extension::behaviour::BehaviourReactor {

    private:
        /// @brief State machine states for ball searching behavior
        enum class SearchState {
            TURNING_ON_SPOT,   ///< Robot is turning on the spot to search
            MOVING_TO_CENTRE,  ///< Robot is moving towards centre of field
            PATROLLING         ///< Robot is patrolling around the field
        };

        /// @brief Stores configuration values
        struct Config {
            /// @brief Length of time before the ball detection is too old and we should search for the ball
            NUClear::clock::duration ball_search_timeout = std::chrono::seconds(0);
            /// @brief Distance from field border that triggers moving to centre
            double border_threshold = 0.0;
            /// @brief How far to move towards centre
            double centre_move_ratio = 0.0;
            /// @brief Distance from centre to be considered "at centre"
            double centre_reached_threshold = 0.0;
            /// @brief Distance from patrol point to be considered "reached"
            double patrol_reached_threshold = 0.0;
            /// @brief Angular threshold for completing a turn (radians)
            double turn_completion_threshold = 0.0;
            /// @brief Duration to turn on the spot (seconds)
            NUClear::clock::duration turn_duration = std::chrono::seconds(0);
        } cfg;

        /// @brief Current state of the search state machine
        SearchState current_state = SearchState::TURNING_ON_SPOT;

        /// @brief Current patrol target index for cycling through patrol points
        int patrol_target = 0;

        /// @brief Time when current state started (for fallback transitions)
        NUClear::clock::time_point state_start_time = NUClear::clock::now();

    public:
        /// @brief Called by the powerplant to build and setup the FindObject reactor.
        explicit FindObject(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::strategy

#endif  // MODULE_STRATEGY_FINDOBJECT_HPP
