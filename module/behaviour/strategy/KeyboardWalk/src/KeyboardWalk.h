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

#ifndef MODULES_BEHAVIOUR_STRATEGY_CONTROLLABLEDARWIN_H
#define MODULES_BEHAVIOUR_STRATEGY_CONTROLLABLEDARWIN_H

#include <armadillo>
#include <nuclear>

#include "utility/input/LimbID.h"

namespace module {
namespace behaviour {
    namespace strategy {

        class KeyboardWalk : public NUClear::Reactor {
        private:
            static constexpr const double DIFF     = 0.01;
            static constexpr const double ROT_DIFF = 0.10;

            static constexpr const double HEAD_DIFF = 1 * M_PI / 180;

            bool moving = false;
            arma::vec2 velocity;
            float rotation = 0;

            float headYaw   = 0;
            float headPitch = 0;

            void forward();
            void left();
            void back();
            void right();
            void turnLeft();
            void turnRight();
            void getUp();
            void reset();
            void kick(utility::input::LimbID::Value l);
            void lookLeft();
            void lookRight();
            void lookUp();
            void lookDown();
            void walkToggle();
            void quit();

            void updateCommand();
            void printStatus();

        public:
            /// @brief Called by the powerplant to build and setup the KeyboardWalk reactor.
            explicit KeyboardWalk(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module


#endif
