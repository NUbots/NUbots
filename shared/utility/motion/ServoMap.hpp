/*
 * This file is part of the NUbots Codebase.
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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_SERVOMAP_HPP
#define UTILITY_MOTION_SERVOMAP_HPP

#include "message/motion/Limbs.hpp"
#include "message/motion/Servos.hpp"

#include "utility/input/ServoID.hpp"

namespace utility::motion {

    using message::motion::HeadPitch;
    using message::motion::HeadYaw;
    using message::motion::LeftAnklePitch;
    using message::motion::LeftAnkleRoll;
    using message::motion::LeftElbow;
    using message::motion::LeftHipPitch;
    using message::motion::LeftHipRoll;
    using message::motion::LeftHipYaw;
    using message::motion::LeftKnee;
    using message::motion::LeftShoulderPitch;
    using message::motion::LeftShoulderRoll;
    using message::motion::RightAnklePitch;
    using message::motion::RightAnkleRoll;
    using message::motion::RightElbow;
    using message::motion::RightHipPitch;
    using message::motion::RightHipRoll;
    using message::motion::RightHipYaw;
    using message::motion::RightKnee;
    using message::motion::RightShoulderPitch;
    using message::motion::RightShoulderRoll;
    using utility::input::ServoID;

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

}  // namespace utility::motion

#endif  // UTILITY_MOTION_SERVOMAP_HPP
