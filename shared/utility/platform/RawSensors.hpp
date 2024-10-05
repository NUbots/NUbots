/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#ifndef UTILITY_PLATFORM_RAWSENSORS_HPP
#define UTILITY_PLATFORM_RAWSENSORS_HPP

#include "message/actuation/Servos.hpp"
#include "message/platform/RawSensors.hpp"

namespace utility::platform {

    using message::actuation::Servo;
    using message::actuation::ServoID;
    using message::platform::RawSensors;

    /**
     * @brief Functions to create log strings for packet errors and servo errors
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

    inline std::string make_servo_hardware_error_string(const Servo& servo) {
        std::stringstream s;
        s << "Error on Servo " << (servo.state.id + 1) << " (" << static_cast<ServoID>(servo.state.id) << "):";

        if ((servo.state.hardware_error & RawSensors::HardwareError::INPUT_VOLTAGE) != 0u) {
            s << " Input Voltage - " << servo.state.voltage;
        }
        if ((servo.state.hardware_error & RawSensors::HardwareError::OVERHEATING) != 0u) {
            s << " Overheating - " << servo.state.temperature;
        }
        if ((servo.state.hardware_error & RawSensors::HardwareError::MOTOR_ENCODER) != 0u) {
            s << " Motor Encoder Malfunction ";
        }
        if ((servo.state.hardware_error & RawSensors::HardwareError::ELECTRICAL_SHOCK) != 0u) {
            s << " Electrical Shock or Insufficient Power ";
        }
        if ((servo.state.hardware_error & RawSensors::HardwareError::OVERLOAD) != 0u) {
            s << " Overloaded - " << servo.state.present_current;
        }
        return s.str();
    }
}  // namespace utility::platform

#endif  // UTILITY_PLATFORM_RAWSENSORS_HPP
