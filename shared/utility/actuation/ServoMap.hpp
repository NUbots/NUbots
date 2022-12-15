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

#ifndef UTILITY_ACTUATION_SERVOMAP_HPP
#define UTILITY_ACTUATION_SERVOMAP_HPP

#include "message/actuation/Limbs.hpp"
#include "message/actuation/Servos.hpp"

#include "utility/input/ServoID.hpp"

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

}  // namespace utility::actuation

#endif  // UTILITY_ACTUATION_SERVOMAP_HPP
