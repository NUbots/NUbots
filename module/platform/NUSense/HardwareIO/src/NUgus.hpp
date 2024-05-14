/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef MODULE_PLATFORM_NUSENSE_NUGUS_HPP
#define MODULE_PLATFORM_NUSENSE_NUGUS_HPP

#include <array>
#include <stdexcept>

namespace module::platform::NUSense {

    class NUgus {
    public:

        /// @brief The direction (clockwise or anticlockwise) to measure each motor in
        std::array<int8_t, 20> servo_direction{};

        /// @brief Offsets the radian angles of motors to change their 0 position
        std::array<double, 20> servo_offset{};

        enum class ID : uint8_t {
            NO_ID            = 0,
            R_SHOULDER_PITCH = 1,
            L_SHOULDER_PITCH = 2,
            R_SHOULDER_ROLL  = 3,
            L_SHOULDER_ROLL  = 4,
            R_ELBOW          = 5,
            L_ELBOW          = 6,
            R_HIP_YAW        = 7,
            L_HIP_YAW        = 8,
            R_HIP_ROLL       = 9,
            L_HIP_ROLL       = 10,
            R_HIP_PITCH      = 11,
            L_HIP_PITCH      = 12,
            R_KNEE           = 13,
            L_KNEE           = 14,
            R_ANKLE_PITCH    = 15,
            L_ANKLE_PITCH    = 16,
            R_ANKLE_ROLL     = 17,
            L_ANKLE_ROLL     = 18,
            HEAD_YAW         = 19,
            HEAD_PITCH       = 20,
            R_FSR            = 111,  // Not yet implemented
            L_FSR            = 112,  // Not yet implemented
            NUSENSE          = 200,
            BROADCAST        = 254
        };

        constexpr std::string device_name(ID id) const {
            switch (id) {
                case ID::NO_ID: return "NO_ID";
                case ID::R_SHOULDER_PITCH: return "R_SHOULDER_PITCH";
                case ID::L_SHOULDER_PITCH: return "L_SHOULDER_PITCH";
                case ID::R_SHOULDER_ROLL: return "R_SHOULDER_ROLL";
                case ID::L_SHOULDER_ROLL: return "L_SHOULDER_ROLL";
                case ID::R_ELBOW: return "R_ELBOW";
                case ID::L_ELBOW: return "L_ELBOW";
                case ID::R_HIP_YAW: return "R_HIP_YAW";
                case ID::L_HIP_YAW: return "L_HIP_YAW";
                case ID::R_HIP_ROLL: return "R_HIP_ROLL";
                case ID::L_HIP_ROLL: return "L_HIP_ROLL";
                case ID::R_HIP_PITCH: return "R_HIP_PITCH";
                case ID::L_HIP_PITCH: return "L_HIP_PITCH";
                case ID::R_KNEE: return "R_KNEE";
                case ID::L_KNEE: return "L_KNEE";
                case ID::R_ANKLE_PITCH: return "R_ANKLE_PITCH";
                case ID::L_ANKLE_PITCH: return "L_ANKLE_PITCH";
                case ID::R_ANKLE_ROLL: return "R_ANKLE_ROLL";
                case ID::L_ANKLE_ROLL: return "L_ANKLE_ROLL";
                case ID::HEAD_YAW: return "HEAD_YAW";
                case ID::HEAD_PITCH: return "HEAD_PITCH";
                case ID::R_FSR: return "R_FSR";
                case ID::L_FSR: return "L_FSR";
                case ID::NUSENSE: return "NUSENSE";
                case ID::BROADCAST: return "BROADCAST";
                default: throw std::runtime_error("enum NUgus::ID's value is corrupt, unknown value stored");
            }
        }
    };

}  // namespace module::platform::NUSense

#endif  // MODULE_PLATFORM_NUSENSE_NUGUS_HPP
