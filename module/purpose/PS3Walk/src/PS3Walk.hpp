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

#ifndef MODULES_PURPOSE_PS3WALK_HPP
#define MODULES_PURPOSE_PS3WALK_HPP


#include <Eigen/Core>
#include <nuclear>

#include "Joystick.hpp"

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class PS3Walk : public ::extension::behaviour::BehaviourReactor {
    public:
        // axes
        static constexpr uint AXIS_LEFT_JOYSTICK_HORIZONTAL  = 0;
        static constexpr uint AXIS_LEFT_JOYSTICK_VERTICAL    = 1;
        static constexpr uint AXIS_RIGHT_JOYSTICK_HORIZONTAL = 2;
        static constexpr uint AXIS_RIGHT_JOYSTICK_VERTICAL   = 3;
        static constexpr uint AXIS_L2                        = 12;
        static constexpr uint AXIS_R2                        = 13;
        static constexpr uint AXIS_ACCEL_Y                   = 23;
        static constexpr uint AXIS_ACCEL_X                   = 24;
        static constexpr uint AXIS_ACCEL_Z                   = 25;

        // buttons
        static constexpr uint BUTTON_SELECT         = 0;
        static constexpr uint BUTTON_LEFT_JOYSTICK  = 1;
        static constexpr uint BUTTON_RIGHT_JOYSTICK = 2;
        static constexpr uint BUTTON_START          = 3;
        static constexpr uint BUTTON_DPAD_UP        = 4;
        static constexpr uint BUTTON_DPAD_RIGHT     = 5;
        static constexpr uint BUTTON_DPAD_DOWN      = 6;
        static constexpr uint BUTTON_DPAD_LEFT      = 7;
        static constexpr uint BUTTON_L2             = 8;
        static constexpr uint BUTTON_R2             = 9;
        static constexpr uint BUTTON_L1             = 10;
        static constexpr uint BUTTON_R1             = 11;
        static constexpr uint BUTTON_TRIANGLE       = 12;
        static constexpr uint BUTTON_CIRCLE         = 13;
        static constexpr uint BUTTON_CROSS          = 14;
        static constexpr uint BUTTON_SQUARE         = 15;

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
}  // namespace module::purpose


#endif  // MODULE_PURPOSE_PS3WALK_HPP
