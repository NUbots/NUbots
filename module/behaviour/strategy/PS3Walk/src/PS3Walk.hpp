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

#ifndef MODULES_BEHAVIOUR_STRATEGY_PS3WALK_HPP
#define MODULES_BEHAVIOUR_STRATEGY_PS3WALK_HPP


#include <Eigen/Core>
#include <nuclear>

#include "Joystick.hpp"

namespace module::behaviour::strategy {

    class PS3Walk : public NUClear::Reactor {
    public:
        // axes
        static constexpr uint AXIS_LEFT_JOYSTICK_HORIZONTAL  = 0;
        static constexpr uint AXIS_LEFT_JOYSTICK_VERTICAL    = 1;
        static constexpr uint AXIS_RIGHT_JOYSTICK_HORIZONTAL = 3;
        static constexpr uint AXIS_RIGHT_JOYSTICK_VERTICAL   = 4;
        static constexpr uint AXIS_L2                        = 2;
        static constexpr uint AXIS_R2                        = 5;

        // buttons
        static constexpr uint BUTTON_SELECT         = 8;
        static constexpr uint BUTTON_LEFT_JOYSTICK  = 11;
        static constexpr uint BUTTON_RIGHT_JOYSTICK = 12;
        static constexpr uint BUTTON_START          = 9;
        static constexpr uint BUTTON_DPAD_UP        = 13;
        static constexpr uint BUTTON_DPAD_RIGHT     = 16;
        static constexpr uint BUTTON_DPAD_DOWN      = 14;
        static constexpr uint BUTTON_DPAD_LEFT      = 15;
        static constexpr uint BUTTON_L2             = 6;
        static constexpr uint BUTTON_R2             = 7;
        static constexpr uint BUTTON_L1             = 4;
        static constexpr uint BUTTON_R1             = 5;
        static constexpr uint BUTTON_TRIANGLE       = 2;
        static constexpr uint BUTTON_CIRCLE         = 1;
        static constexpr uint BUTTON_CROSS          = 0;
        static constexpr uint BUTTON_SQUARE         = 3;

        /// @brief Called by the powerplant to build and setup the PS3Walk reactor.
        explicit PS3Walk(std::unique_ptr<NUClear::Environment> environment);

    private:
        Joystick joystick;
        Eigen::Vector2d strafe = Eigen::Vector2d::Zero();
        float rotationalSpeed  = 0.0f;
        bool moving            = false;
        bool headLocked        = false;
        float headPitch        = 0.0f;
        float headYaw          = 0.0f;
    };
}  // namespace module::behaviour::strategy


#endif
