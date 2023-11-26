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
#ifndef MODULE_PLATFORM_OPENCR_OPENCR_HPP
#define MODULE_PLATFORM_OPENCR_OPENCR_HPP

#include "dynamixel/v2/DynamixelDevice.hpp"

namespace module::platform::OpenCR {
    class OpenCR : public DynamixelDevice {
    public:
        OpenCR(uint8_t ID) : ID(ID) {}

        const uint8_t ID;

        /**
         * @brief Holds the addresses of the various bytes in the OpenCR RAM table.
         *
         * @details
         *  for additional details see
         * http://support.robotis.com/en/product/robotis-op3/sub_controller(opencr).htm
         */
        enum class Address : uint8_t {
            MODEL_NUMBER_L      = 0,
            MODEL_NUMBER_H      = 1,
            VERSION             = 2,
            ID                  = 3,
            BAUD_RATE           = 4,
            RETURN_DELAY_TIME   = 5,
            STATUS_RETURN_LEVEL = 16,
            DYNAMIXEL_POWER     = 24,
            LED                 = 25,
            LED_RGB_L           = 26,
            LED_RGB_H           = 27,
            BUZZER_L            = 28,
            BUZZER_H            = 29,
            BUTTON              = 30,
            VOLTAGE             = 31,
            GYRO_Z_L            = 32,
            GYRO_Z_H            = 33,
            GYRO_Y_L            = 34,
            GYRO_Y_H            = 35,
            GYRO_X_L            = 36,
            GYRO_X_H            = 37,
            ACC_X_L             = 38,
            ACC_X_H             = 39,
            ACC_Y_L             = 40,
            ACC_Y_H             = 41,
            ACC_Z_L             = 42,
            ACC_Z_H             = 43,
            ROLL_L              = 44,
            ROLL_H              = 45,
            PITCH_L             = 46,
            PITCH_H             = 47,
            YAW_L               = 48,
            YAW_H               = 49,
            IMU_CONTROL         = 50
        };
    };

}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFORM_OPENCR_OPENCR_HPP
