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

    /**
     * @brief Functions to create log strings for packet errors and servo errors
     * @note functions appended with `_v1` are compatibility functions for protocol v1
     */

    inline std::string make_packet_error_string(const std::string& src, const uint error_code) {
        std::stringstream s;

        s << "Error on ";
        s << src;
        s << ":";

        if ((error_code & RawSensors::PacketError::RESULT_FAIL) != 0u) {
            s << " Result Fail ";
        }
        if ((error_code & RawSensors::PacketError::INSTRUCTION) != 0u) {
            s << " Bad Instruction ";
        }
        if ((error_code & RawSensors::PacketError::CRC) != 0u) {
            s << " CRC ";
        }
        if ((error_code & RawSensors::PacketError::DATA_RANGE) != 0u) {
            s << " Data Range ";
        }
        if ((error_code & RawSensors::PacketError::DATA_LENGTH) != 0u) {
            s << " Data Length ";
        }
        if ((error_code & RawSensors::PacketError::DATA_LIMIT) != 0u) {
            s << " Data Limit ";
        }
        if ((error_code & RawSensors::PacketError::ACCESS) != 0u) {
            s << " Access ";
        }
        if ((error_code & RawSensors::PacketError::ALERT) != 0u) {
            s << " Alert Bit ";
        }

        return s.str();
    }

    inline std::string make_servo_hardware_error_string(const RawSensors::Servo& servo, const uint32_t servo_id) {
        std::stringstream s;
        s << "Error on Servo " << (servo_id + 1) << " (" << static_cast<ServoID>(servo_id) << "):";

        if ((servo.hardware_error & RawSensors::HardwareError::INPUT_VOLTAGE) != 0u) {
            s << " Input Voltage - " << servo.voltage;
        }
        if ((servo.hardware_error & RawSensors::HardwareError::OVERHEATING) != 0u) {
            s << " Overheating - " << servo.temperature;
        }
        if ((servo.hardware_error & RawSensors::HardwareError::MOTOR_ENCODER) != 0u) {
            s << " Motor Encoder Malfunction ";
        }
        if ((servo.hardware_error & RawSensors::HardwareError::ELECTRICAL_SHOCK) != 0u) {
            s << " Electrical Shock or Insufficient Power ";
        }
        if ((servo.hardware_error & RawSensors::HardwareError::OVERLOAD) != 0u) {
            s << " Overloaded - " << servo.present_current;
        }
        return s.str();
    }

    inline std::string make_error_string_v1(const std::string& src, const uint error_code) {
        std::stringstream s;

        s << "Error on ";
        s << src;
        s << ":";

        if ((error_code & RawSensors::Error::_INPUT_VOLTAGE) != 0u) {
            s << " Input Voltage ";
        }
        if ((error_code & RawSensors::Error::_ANGLE_LIMIT) != 0u) {
            s << " Angle Limit ";
        }
        if ((error_code & RawSensors::Error::_OVERHEATING) != 0u) {
            s << " Overheating ";
        }
        if ((error_code & RawSensors::Error::_OVERLOAD) != 0u) {
            s << " Overloaded ";
        }
        if ((error_code & RawSensors::Error::_INSTRUCTION) != 0u) {
            s << " Bad Instruction ";
        }
        if ((error_code & RawSensors::Error::_CORRUPT_DATA) != 0u) {
            s << " Corrupt Data ";
        }
        if ((error_code & RawSensors::Error::_TIMEOUT) != 0u) {
            s << " Timeout ";
        }

        return s.str();
    }

    inline std::string make_servo_error_string_v1(const RawSensors::Servo& servo, const uint32_t servo_id) {
        std::stringstream s;
        s << "Error on Servo " << (servo_id + 1) << " (" << static_cast<ServoID>(servo_id) << "):";

        if ((servo.error_flags & RawSensors::Error::_INPUT_VOLTAGE) != 0u) {
            s << " Input Voltage - " << servo.voltage;
        }
        if ((servo.error_flags & RawSensors::Error::_ANGLE_LIMIT) != 0u) {
            s << " Angle Limit - " << servo.present_position;
        }
        if ((servo.error_flags & RawSensors::Error::_OVERHEATING) != 0u) {
            s << " Overheating - " << servo.temperature;
        }
        if ((servo.error_flags & RawSensors::Error::_OVERLOAD) != 0u) {
            s << " Overloaded - " << servo.present_current;
        }
        if ((servo.error_flags & RawSensors::Error::_INSTRUCTION) != 0u) {
            s << " Bad Instruction ";
        }
        if ((servo.error_flags & RawSensors::Error::_CORRUPT_DATA) != 0u) {
            s << " Corrupt Data ";
        }
        if ((servo.error_flags & RawSensors::Error::_TIMEOUT) != 0u) {
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
