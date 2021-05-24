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

#include <Eigen/Core>
// #include <mutex>
#include <nuclear>

#include "utility/input/LimbID.hpp"

namespace module::behaviour::strategy {

    class KeyboardWalk : public NUClear::Reactor {
    private:
        static constexpr const float DIFF     = 0.01f;
        static constexpr const float ROT_DIFF = 0.1f;

        bool moving = false;
        Eigen::Vector2f velocity;
        float rotation = 0.0f;

        // std::mutex mutex;

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

        void update_command();
        void print_status();

    public:
        /// @brief Called by the powerplant to build and setup the KeyboardWalk reactor.
        explicit KeyboardWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::behaviour::strategy


#endif
