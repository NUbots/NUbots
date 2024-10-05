/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#ifndef UTILITY_ACTUATION_SERVOMAP_HPP
#define UTILITY_ACTUATION_SERVOMAP_HPP

#include "message/actuation/Limbs.hpp"
#include "message/actuation/Servos.hpp"

namespace utility::actuation {

    using message::actuation::HeadPitch;
    using message::actuation::HeadYaw;
    using message::actuation::LeftAnklePitch;
    using message::actuation::LeftAnkleRoll;
    using message::actuation::LeftElbow;
    using message::actuation::LeftHipPitch;
    using message::actuation::LeftHipRoll;
    using message::actuation::LeftHipYaw;
    using message::actuation::LeftKnee;
    using message::actuation::LeftShoulderPitch;
    using message::actuation::LeftShoulderRoll;
    using message::actuation::RightAnklePitch;
    using message::actuation::RightAnkleRoll;
    using message::actuation::RightElbow;
    using message::actuation::RightHipPitch;
    using message::actuation::RightHipRoll;
    using message::actuation::RightHipYaw;
    using message::actuation::RightKnee;
    using message::actuation::RightShoulderPitch;
    using message::actuation::RightShoulderRoll;
    using message::actuation::ServoID;

    template <typename Servo>
    struct ServoMap;
    // clang-format off
        template <> struct ServoMap<RightShoulderPitch> { static constexpr ServoID::Value value = ServoID::R_SHOULDER_PITCH; };
        template <> struct ServoMap<LeftShoulderPitch> { static constexpr ServoID::Value value = ServoID::L_SHOULDER_PITCH; };
        template <> struct ServoMap<RightShoulderRoll> { static constexpr ServoID::Value value = ServoID::R_SHOULDER_ROLL; };
        template <> struct ServoMap<LeftShoulderRoll> { static constexpr ServoID::Value value = ServoID::L_SHOULDER_ROLL; };
        template <> struct ServoMap<RightElbow> { static constexpr ServoID::Value value = ServoID::R_ELBOW; };
        template <> struct ServoMap<LeftElbow> { static constexpr ServoID::Value value = ServoID::L_ELBOW; };
        template <> struct ServoMap<RightHipYaw> { static constexpr ServoID::Value value = ServoID::R_HIP_YAW; };
        template <> struct ServoMap<LeftHipYaw> { static constexpr ServoID::Value value = ServoID::L_HIP_YAW; };
        template <> struct ServoMap<RightHipRoll> { static constexpr ServoID::Value value = ServoID::R_HIP_ROLL; };
        template <> struct ServoMap<LeftHipRoll> { static constexpr ServoID::Value value = ServoID::L_HIP_ROLL; };
        template <> struct ServoMap<RightHipPitch> { static constexpr ServoID::Value value = ServoID::R_HIP_PITCH; };
        template <> struct ServoMap<LeftHipPitch> { static constexpr ServoID::Value value = ServoID::L_HIP_PITCH; };
        template <> struct ServoMap<RightKnee> { static constexpr ServoID::Value value = ServoID::R_KNEE; };
        template <> struct ServoMap<LeftKnee> { static constexpr ServoID::Value value = ServoID::L_KNEE; };
        template <> struct ServoMap<RightAnklePitch> { static constexpr ServoID::Value value = ServoID::R_ANKLE_PITCH; };
        template <> struct ServoMap<LeftAnklePitch> { static constexpr ServoID::Value value = ServoID::L_ANKLE_PITCH; };
        template <> struct ServoMap<RightAnkleRoll> { static constexpr ServoID::Value value = ServoID::R_ANKLE_ROLL; };
        template <> struct ServoMap<LeftAnkleRoll> { static constexpr ServoID::Value value = ServoID::L_ANKLE_ROLL; };
        template <> struct ServoMap<HeadYaw> { static constexpr ServoID::Value value = ServoID::HEAD_YAW; };
        template <> struct ServoMap<HeadPitch> { static constexpr ServoID::Value value = ServoID::HEAD_PITCH; };
    // clang-format on

}  // namespace utility::actuation

#endif  // UTILITY_ACTUATION_SERVOMAP_HPP
