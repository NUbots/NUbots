/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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
            double maximum_forward_velocity    = 0;
            double maximum_rotational_velocity = 0;
        } cfg;

        /// @brief Controls interactions with the PS3 controller
        Joystick joystick;

        /// @brief Stores a vector describing the current direction to walk in.
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();

        /// @brief Stores whether the robot can move or not
        bool moving = false;

        /// @brief Stores whether the robot will change its head direction or not
        bool head_locked = false;

        /// @brief stores the head pitch value
        double head_pitch = 0.0;

        /// @brief stores the head yaw value
        double head_yaw = 0.0;
    };
}  // namespace module::purpose


#endif  // MODULE_PURPOSE_PS3WALK_HPP
