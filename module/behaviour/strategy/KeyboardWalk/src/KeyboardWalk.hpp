/*
 * This file is part of NUbots Codebase.
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

#ifndef MODULES_BEHAVIOUR_STRATEGY_CONTROLLABLEDARWIN_HPP
#define MODULES_BEHAVIOUR_STRATEGY_CONTROLLABLEDARWIN_HPP

#include <armadillo>
#include <mutex>
#include <ncurses.h>
#include <nuclear>

#include "utility/input/LimbID.hpp"

namespace module {
namespace behaviour {
    namespace strategy {

        enum class LogColours : short {
            TRACE_COLOURS = 1,
            DEBUG_COLOURS = 2,
            INFO_COLOURS  = 3,
            WARN_COLOURS  = 4,
            ERROR_COLOURS = 5,
            FATAL_COLOURS = 6
        };

        class KeyboardWalk : public NUClear::Reactor {
        private:
            static constexpr const double DIFF     = 0.01;
            static constexpr const double ROT_DIFF = 0.10;

            static constexpr const double HEAD_DIFF = 1 * M_PI / 180;

            bool moving = false;
            arma::vec2 velocity;
            float rotation = 0;

            float head_yaw   = 0.0f;
            float head_pitch = 0.0f;

            std::shared_ptr<WINDOW> command_window;
            std::shared_ptr<WINDOW> log_window;
            bool colours_enabled;

            std::mutex mutex;

            void create_windows();
            void forward();
            void left();
            void back();
            void right();
            void turn_left();
            void turn_right();
            void get_up();
            void reset();
            void kick(utility::input::LimbID::Value l);
            void look_left();
            void look_right();
            void look_up();
            void look_down();
            void walk_toggle();
            void quit();

            void update_command();
            void print_status();
            void update_window(const std::shared_ptr<WINDOW>& window,
                               const LogColours& colours,
                               const std::string& source,
                               const std::string& message,
                               const bool& print_level);

        public:
            /// @brief Called by the powerplant to build and setup the KeyboardWalk reactor.
            explicit KeyboardWalk(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module


#endif
