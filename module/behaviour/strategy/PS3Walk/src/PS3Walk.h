/*
 * This file is part of PS3Walk.
 *
 * PS3Walk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PS3Walk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PS3Walk.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_STRATEGY_PS3WALK_H
#define MODULES_BEHAVIOUR_STRATEGY_PS3WALK_H


#include <armadillo>
#include <nuclear>
#include "Joystick.h"

namespace module {
namespace behaviour {
    namespace strategy {

        class PS3Walk : public NUClear::Reactor {
        public:
            // axes
            static constexpr unsigned int AXIS_LEFT_JOYSTICK_HORIZONTAL  = 0;
            static constexpr unsigned int AXIS_LEFT_JOYSTICK_VERTICAL    = 1;
            static constexpr unsigned int AXIS_RIGHT_JOYSTICK_HORIZONTAL = 2;
            static constexpr unsigned int AXIS_RIGHT_JOYSTICK_VERTICAL   = 3;
            static constexpr unsigned int AXIS_L2                        = 12;
            static constexpr unsigned int AXIS_R2                        = 13;
            static constexpr unsigned int AXIS_ACCEL_Y                   = 23;
            static constexpr unsigned int AXIS_ACCEL_X                   = 24;
            static constexpr unsigned int AXIS_ACCEL_Z                   = 25;

            // buttons
            static constexpr unsigned int BUTTON_SELECT         = 0;
            static constexpr unsigned int BUTTON_LEFT_JOYSTICK  = 1;
            static constexpr unsigned int BUTTON_RIGHT_JOYSTICK = 2;
            static constexpr unsigned int BUTTON_START          = 3;
            static constexpr unsigned int BUTTON_DPAD_UP        = 4;
            static constexpr unsigned int BUTTON_DPAD_RIGHT     = 5;
            static constexpr unsigned int BUTTON_DPAD_DOWN      = 6;
            static constexpr unsigned int BUTTON_DPAD_LEFT      = 7;
            static constexpr unsigned int BUTTON_L2             = 8;
            static constexpr unsigned int BUTTON_R2             = 9;
            static constexpr unsigned int BUTTON_L1             = 10;
            static constexpr unsigned int BUTTON_R1             = 11;
            static constexpr unsigned int BUTTON_TRIANGLE       = 12;
            static constexpr unsigned int BUTTON_CIRCLE         = 13;
            static constexpr unsigned int BUTTON_CROSS          = 14;
            static constexpr unsigned int BUTTON_SQUARE         = 15;

            /// @brief Called by the powerplant to build and setup the PS3Walk reactor.
            explicit PS3Walk(std::unique_ptr<NUClear::Environment> environment);

        private:
            Joystick joystick;  // TODO: make configurable
            arma::vec strafe{0, 0};
            float rotationalSpeed = 0;
            bool moving           = false;
            bool headLocked       = false;
            float headPitch       = 0;
            float headYaw         = 0;
        };
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module


#endif
