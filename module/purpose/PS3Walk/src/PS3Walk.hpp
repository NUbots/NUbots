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
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_PURPOSE_PS3WALK_HPP
#define MODULE_PURPOSE_PS3WALK_HPP


#include <Eigen/Core>
#include <nuclear>

#include "Joystick.hpp"

namespace module::purpose {

    class PS3Walk : public NUClear::Reactor {
    public:
        // axes
        static constexpr uint AXIS_LEFT_JOYSTICK_VERTICAL    = 1;
        static constexpr uint AXIS_LEFT_JOYSTICK_HORIZONTAL  = 0;
        static constexpr uint AXIS_RIGHT_JOYSTICK_HORIZONTAL = 3;
        static constexpr uint AXIS_RIGHT_JOYSTICK_VERTICAL   = 4;
        static constexpr uint AXIS_L2                        = 12;
        static constexpr uint AXIS_R2                        = 13;
        static constexpr uint AXIS_ACCEL_Y                   = 23;
        static constexpr uint AXIS_ACCEL_X                   = 24;
        static constexpr uint AXIS_ACCEL_Z                   = 25;

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
        static constexpr uint BUTTON_HOME           = 10;

        /// @brief Called by the powerplant to build and setup the PS3Walk reactor.
        explicit PS3Walk(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Stores configuration values
        struct Config {
            float maximum_forward_velocity    = 0;
            float maximum_rotational_velocity = 0;
        } cfg;

        /// @brief
        Joystick joystick;

        /// @brief
        Eigen::Vector3f walk_command = Eigen::Vector3f::Zero();

        /// @brief
        bool moving = false;

        /// @brief
        bool head_locked = false;

        /// @brief
        float head_pitch = 0.0f;

        /// @brief
        float head_yaw = 0.0f;
    };
}  // namespace module::purpose


#endif  // MODULE_PURPOSE_PS3WALK_HPP
