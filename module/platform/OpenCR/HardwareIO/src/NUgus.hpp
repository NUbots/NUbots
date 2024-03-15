/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#ifndef MODULE_PLATFORM_OPENCR_NUGUS_HPP
#define MODULE_PLATFORM_OPENCR_NUGUS_HPP

#include <array>
#include <stdexcept>

#include "dynamixel/v2/DynamixelServo.hpp"
#include "dynamixel/v2/FSR.hpp"
#include "dynamixel/v2/OpenCR.hpp"

namespace module::platform::OpenCR {

    struct MX64 : public DynamixelServo {
        MX64(uint8_t ID) : ID(ID) {}
        const uint8_t ID;
    };

    struct MX106 : public DynamixelServo {
        MX106(uint8_t ID) : ID(ID) {}
        const uint8_t ID;
    };

    struct XH540_W270 : public DynamixelServo {
        XH540_W270(uint8_t ID) : ID(ID) {}
        const uint8_t ID;
    };

    class NUgus {
    public:
        NUgus();

        /// @brief The direction (clockwise or anticlockwise) to measure each motor in
        std::array<int8_t, 20> servo_direction{};

        /// @brief Offsets the radian angles of motors to change their 0 position
        std::array<double, 20> servo_offset{};

        ///@brief Sets the upper limit of servo rotation
        std::array<float, 20> servo_max_position_limits{};

        ///@brief Sets the lower limit of servo rotation
        std::array<float, 20> servo_min_position_limits{};

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
            R_FSR            = 111,
            L_FSR            = 112,
            OPENCR           = 200,
            BROADCAST        = 254
        };

        OpenCR OPENCR;
        MX64 R_SHOULDER_PITCH;
        MX64 L_SHOULDER_PITCH;
        MX64 R_SHOULDER_ROLL;
        MX64 L_SHOULDER_ROLL;
        MX64 R_ELBOW;
        MX64 L_ELBOW;
        MX106 R_HIP_YAW;
        MX106 L_HIP_YAW;
        MX106 R_HIP_ROLL;
        MX106 L_HIP_ROLL;
        MX106 R_HIP_PITCH;
        MX106 L_HIP_PITCH;
        XH540_W270 R_KNEE;
        XH540_W270 L_KNEE;
        MX106 R_ANKLE_PITCH;
        MX106 L_ANKLE_PITCH;
        MX106 R_ANKLE_ROLL;
        MX106 L_ANKLE_ROLL;
        MX64 HEAD_YAW;
        MX64 HEAD_PITCH;

        /// @brief Get a reference to the DynamixelDevice with the given ID
        /// @param id The ID of the device to get
        /// @return A reference to the DynamixelDevice with the given ID
        constexpr DynamixelDevice& operator[](const ID& id) {
            switch (id) {
                case ID::OPENCR: return OPENCR;
                case ID::R_SHOULDER_PITCH: return R_SHOULDER_PITCH;
                case ID::L_SHOULDER_PITCH: return L_SHOULDER_PITCH;
                case ID::R_SHOULDER_ROLL: return R_SHOULDER_ROLL;
                case ID::L_SHOULDER_ROLL: return L_SHOULDER_ROLL;
                case ID::R_ELBOW: return R_ELBOW;
                case ID::L_ELBOW: return L_ELBOW;
                case ID::R_HIP_YAW: return R_HIP_YAW;
                case ID::L_HIP_YAW: return L_HIP_YAW;
                case ID::R_HIP_ROLL: return R_HIP_ROLL;
                case ID::L_HIP_ROLL: return L_HIP_ROLL;
                case ID::R_HIP_PITCH: return R_HIP_PITCH;
                case ID::L_HIP_PITCH: return L_HIP_PITCH;
                case ID::R_KNEE: return R_KNEE;
                case ID::L_KNEE: return L_KNEE;
                case ID::R_ANKLE_PITCH: return R_ANKLE_PITCH;
                case ID::L_ANKLE_PITCH: return L_ANKLE_PITCH;
                case ID::R_ANKLE_ROLL: return R_ANKLE_ROLL;
                case ID::L_ANKLE_ROLL: return L_ANKLE_ROLL;
                case ID::HEAD_YAW: return HEAD_YAW;
                case ID::HEAD_PITCH: return HEAD_PITCH;
                default: throw std::runtime_error("Unknown device id");
            }
        }

        /// @brief Get an array of the dynamixel IDs of each servo
        /// @warning Different to utility::input::ServoID which is zero indexed
        /// @return Array of uint8_t containing the IDs in order
        constexpr std::array<uint8_t, 20> servo_ids() const {
            return {uint8_t(ID::R_SHOULDER_PITCH), uint8_t(ID::L_SHOULDER_PITCH), uint8_t(ID::R_SHOULDER_ROLL),
                    uint8_t(ID::L_SHOULDER_ROLL),  uint8_t(ID::R_ELBOW),          uint8_t(ID::L_ELBOW),
                    uint8_t(ID::R_HIP_YAW),        uint8_t(ID::L_HIP_YAW),        uint8_t(ID::R_HIP_ROLL),
                    uint8_t(ID::L_HIP_ROLL),       uint8_t(ID::R_HIP_PITCH),      uint8_t(ID::L_HIP_PITCH),
                    uint8_t(ID::R_KNEE),           uint8_t(ID::L_KNEE),           uint8_t(ID::R_ANKLE_PITCH),
                    uint8_t(ID::L_ANKLE_PITCH),    uint8_t(ID::R_ANKLE_ROLL),     uint8_t(ID::L_ANKLE_ROLL),
                    uint8_t(ID::HEAD_YAW),         uint8_t(ID::HEAD_PITCH)};
        }

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
                case ID::OPENCR: return "OPENCR";
                case ID::BROADCAST: return "BROADCAST";
                default: throw std::runtime_error("enum NUgus::ID's value is corrupt, unknown value stored");
            }
        }
    };

    /// @brief The first part of the servo data to write to the dynamixel
    struct DynamixelServoWriteDataPart1 {
        uint8_t torque_enable;
        uint16_t velocity_i_gain;
        uint16_t velocity_p_gain;
        uint16_t position_d_gain;
        uint16_t position_i_gain;
        uint16_t position_p_gain;
    } __attribute__((packed));

    /// @brief The second part of the servo data to write to the dynamixel
    struct DynamixelServoWriteDataPart2 {
        uint16_t feedforward_1st_gain;
        uint16_t feedforward_2nd_gain;
        int16_t goal_pwm;
        int16_t goal_current;
        int32_t goal_velocity;
        uint32_t profile_acceleration;
        uint32_t profile_velocity;
        uint32_t goal_position;
    } __attribute__((packed));

    /// @brief The servo data to read from the dynamixel
    struct DynamixelServoReadData {
        uint8_t torque_enable;
        uint8_t hardware_error_status;
        int16_t present_pwm;
        int16_t present_current;
        int32_t present_velocity;
        uint32_t present_position;
        uint16_t present_voltage;
        uint8_t present_temperature;
    } __attribute__((packed));

    /// @brief The data to write to the OpenCR device
    struct OpenCRWriteData {
        uint8_t led;
        uint16_t rgb_led;
        uint16_t buzzer;
    } __attribute__((packed));

    /// @brief The data to read from the OpenCR device
    struct OpenCRReadData {
        uint8_t led;
        uint16_t rgb_led;
        uint16_t buzzer;
        uint8_t button;
        uint8_t voltage;
        int16_t gyro[3];
        int16_t acc[3];
    } __attribute__((packed));

    /// @brief Document addresses used for read/writing to dynamixel devices, especially
    /// where indirect addressing is used.
    enum class AddressBook : uint16_t {
        SERVO_READ_ADDRESS    = uint16_t(DynamixelServo::Address::INDIRECT_ADDRESS_1_L),
        SERVO_READ            = uint16_t(DynamixelServo::Address::INDIRECT_DATA_1),
        SERVO_WRITE_ADDRESS_1 = uint16_t(DynamixelServo::Address::INDIRECT_ADDRESS_18_L),
        SERVO_WRITE_ADDRESS_2 = uint16_t(DynamixelServo::Address::INDIRECT_ADDRESS_29_L),
        SERVO_WRITE_1         = uint16_t(DynamixelServo::Address::INDIRECT_DATA_18),
        SERVO_WRITE_2         = uint16_t(DynamixelServo::Address::INDIRECT_DATA_29),
        FSR_READ              = uint16_t(FSR::Address::FSR1_L)
    };

}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFOR_OPENCR_NUGUS_HPP
