/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
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
#ifndef MODULE_SKILL_WALK_K1_JOINTS_HPP
#define MODULE_SKILL_WALK_K1_JOINTS_HPP

#include <array>
#include <cstddef>

#include "message/input/Sensors.hpp"

#include "utility/input/ServoID.hpp"

namespace module::skill::k1 {

    /// All K1 joints, in the Booster SDK JointIndexK1 serial order
    inline constexpr std::size_t JOINT_COUNT = 22;

    /// ServoID of each joint, in JointIndexK1 order
    inline constexpr std::array<utility::input::ServoID::Value, JOINT_COUNT> SERVO_IDS = {
        utility::input::ServoID::HEAD_YAW,         utility::input::ServoID::HEAD_PITCH,
        utility::input::ServoID::L_SHOULDER_PITCH, utility::input::ServoID::L_SHOULDER_ROLL,
        utility::input::ServoID::L_ELBOW,          utility::input::ServoID::L_ELBOW_YAW,
        utility::input::ServoID::R_SHOULDER_PITCH, utility::input::ServoID::R_SHOULDER_ROLL,
        utility::input::ServoID::R_ELBOW,          utility::input::ServoID::R_ELBOW_YAW,
        utility::input::ServoID::L_HIP_PITCH,      utility::input::ServoID::L_HIP_ROLL,
        utility::input::ServoID::L_HIP_YAW,        utility::input::ServoID::L_KNEE,
        utility::input::ServoID::L_ANKLE_PITCH,    utility::input::ServoID::L_ANKLE_ROLL,
        utility::input::ServoID::R_HIP_PITCH,      utility::input::ServoID::R_HIP_ROLL,
        utility::input::ServoID::R_HIP_YAW,        utility::input::ServoID::R_KNEE,
        utility::input::ServoID::R_ANKLE_PITCH,    utility::input::ServoID::R_ANKLE_ROLL};

    /// Servo feedback in JointIndexK1 order. Sensors::servo is indexed by ServoID.
    inline std::array<const message::input::Sensors::Servo*, JOINT_COUNT> servos_of(
        const message::input::Sensors& sensors) {
        std::array<const message::input::Sensors::Servo*, JOINT_COUNT> servos{};
        for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
            servos[j] = &sensors.servo[SERVO_IDS[j]];
        }
        return servos;
    }

}  // namespace module::skill::k1

#endif  // MODULE_SKILL_WALK_K1_JOINTS_HPP
