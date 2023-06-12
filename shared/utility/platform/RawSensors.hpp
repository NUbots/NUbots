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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */
#ifndef UTILITY_PLATFORM_RAWSENSORS_HPP
#define UTILITY_PLATFORM_RAWSENSORS_HPP

#include "message/platform/RawSensors.hpp"

#include "utility/input/ServoID.hpp"

namespace utility::platform {

    using message::platform::RawSensors;
    using utility::input::ServoID;

    /// @todo Update error handling to protocol v2
    /// @see RawSensors.proto, StatusReturn.proto
    inline std::string make_error_string(const std::string& src, const uint error_code) {
        std::stringstream s;

        s << "Error on ";
        s << src;
        s << ":";

        if ((error_code & RawSensors::Error::INPUT_VOLTAGE) != 0u) {
            s << " Input Voltage ";
        }
        if ((error_code & RawSensors::Error::ANGLE_LIMIT) != 0u) {
            s << " Angle Limit ";
        }
        if ((error_code & RawSensors::Error::OVERHEATING) != 0u) {
            s << " Overheating ";
        }
        if ((error_code & RawSensors::Error::OVERLOAD) != 0u) {
            s << " Overloaded ";
        }
        if ((error_code & RawSensors::Error::INSTRUCTION) != 0u) {
            s << " Bad Instruction ";
        }
        if ((error_code & RawSensors::Error::CORRUPT_DATA) != 0u) {
            s << " Corrupt Data ";
        }
        if ((error_code & RawSensors::Error::TIMEOUT) != 0u) {
            s << " Timeout ";
        }

        return s.str();
    }

    inline std::string make_servo_error_string(const RawSensors::Servo& servo, const uint32_t servo_id) {
        std::stringstream s;
        s << "Error on Servo " << (servo_id + 1) << " (" << static_cast<ServoID>(servo_id) << "):";

        if (RawSensors::Error::INPUT_VOLTAGE != 0u) {
            s << " Input Voltage - " << servo.voltage;
        }
        if (RawSensors::Error::ANGLE_LIMIT != 0u) {
            s << " Angle Limit - " << servo.present_position;
        }
        if (RawSensors::Error::OVERHEATING != 0u) {
            s << " Overheating - " << servo.temperature;
        }
        if (RawSensors::Error::OVERLOAD != 0u) {
            s << " Overloaded - " << servo.present_current;
        }
        if (RawSensors::Error::INSTRUCTION != 0u) {
            s << " Bad Instruction ";
        }
        if (RawSensors::Error::CORRUPT_DATA != 0u) {
            s << " Corrupt Data ";
        }
        if (RawSensors::Error::TIMEOUT != 0u) {
            s << " Timeout ";
        }
        return s.str();
    }

    inline const RawSensors::Servo& getRawServo(ServoID servoId, const RawSensors& sensors) {

        switch (servoId.value) {
            case ServoID::R_SHOULDER_PITCH: return sensors.servo.r_shoulder_pitch;
            case ServoID::L_SHOULDER_PITCH: return sensors.servo.l_shoulder_pitch;
            case ServoID::R_SHOULDER_ROLL: return sensors.servo.r_shoulder_roll;
            case ServoID::L_SHOULDER_ROLL: return sensors.servo.l_shoulder_roll;
            case ServoID::R_ELBOW: return sensors.servo.r_elbow;
            case ServoID::L_ELBOW: return sensors.servo.l_elbow;
            case ServoID::R_HIP_YAW: return sensors.servo.r_hip_yaw;
            case ServoID::L_HIP_YAW: return sensors.servo.l_hip_yaw;
            case ServoID::R_HIP_ROLL: return sensors.servo.r_hip_roll;
            case ServoID::L_HIP_ROLL: return sensors.servo.l_hip_roll;
            case ServoID::R_HIP_PITCH: return sensors.servo.r_hip_pitch;
            case ServoID::L_HIP_PITCH: return sensors.servo.l_hip_pitch;
            case ServoID::R_KNEE: return sensors.servo.r_knee;
            case ServoID::L_KNEE: return sensors.servo.l_knee;
            case ServoID::R_ANKLE_PITCH: return sensors.servo.r_ankle_pitch;
            case ServoID::L_ANKLE_PITCH: return sensors.servo.l_ankle_pitch;
            case ServoID::R_ANKLE_ROLL: return sensors.servo.r_ankle_roll;
            case ServoID::L_ANKLE_ROLL: return sensors.servo.l_ankle_roll;
            case ServoID::HEAD_YAW: return sensors.servo.head_pan;
            case ServoID::HEAD_PITCH: return sensors.servo.head_tilt;

            default: throw std::runtime_error("Out of bounds");
        }
    }

    inline RawSensors::Servo& getRawServo(ServoID servoId, RawSensors& sensors) {

        switch (servoId.value) {
            case ServoID::R_SHOULDER_PITCH: return sensors.servo.r_shoulder_pitch;
            case ServoID::L_SHOULDER_PITCH: return sensors.servo.l_shoulder_pitch;
            case ServoID::R_SHOULDER_ROLL: return sensors.servo.r_shoulder_roll;
            case ServoID::L_SHOULDER_ROLL: return sensors.servo.l_shoulder_roll;
            case ServoID::R_ELBOW: return sensors.servo.r_elbow;
            case ServoID::L_ELBOW: return sensors.servo.l_elbow;
            case ServoID::R_HIP_YAW: return sensors.servo.r_hip_yaw;
            case ServoID::L_HIP_YAW: return sensors.servo.l_hip_yaw;
            case ServoID::R_HIP_ROLL: return sensors.servo.r_hip_roll;
            case ServoID::L_HIP_ROLL: return sensors.servo.l_hip_roll;
            case ServoID::R_HIP_PITCH: return sensors.servo.r_hip_pitch;
            case ServoID::L_HIP_PITCH: return sensors.servo.l_hip_pitch;
            case ServoID::R_KNEE: return sensors.servo.r_knee;
            case ServoID::L_KNEE: return sensors.servo.l_knee;
            case ServoID::R_ANKLE_PITCH: return sensors.servo.r_ankle_pitch;
            case ServoID::L_ANKLE_PITCH: return sensors.servo.l_ankle_pitch;
            case ServoID::R_ANKLE_ROLL: return sensors.servo.r_ankle_roll;
            case ServoID::L_ANKLE_ROLL: return sensors.servo.l_ankle_roll;
            case ServoID::HEAD_YAW: return sensors.servo.head_pan;
            case ServoID::HEAD_PITCH: return sensors.servo.head_tilt;

            default: throw std::runtime_error("Out of bounds");
        }
    }
}  // namespace utility::platform

#endif  // UTILITY_PLATFORM_RAWSENSORS_HPP
