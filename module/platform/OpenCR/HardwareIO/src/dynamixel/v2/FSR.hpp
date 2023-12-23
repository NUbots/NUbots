/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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
#ifndef MODULE_PLATFORM_OPENCR_FSR_HPP
#define MODULE_PLATFORM_OPENCR_FSR_HPP

#include <cstdint>

#include "dynamixel/v2/DynamixelDevice.hpp"

namespace module::platform::OpenCR {
    struct FSR : public DynamixelDevice {
        /**
         * @brief This struct holds the addresses of the various bytes in the FSR (Force Sensitive Resistors in the
         * feet).
         *
         * @details
         *  for additional details see
         * http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
         *
         * @note this is an out of date placeholder
         */
        enum class Address : uint16_t {
            MODEL_NUMBER_L         = 0,
            MODEL_NUMBER_H         = 1,
            VERSION                = 2,
            ID                     = 3,
            BAUD_RATE              = 4,
            RETURN_DELAY_TIME      = 5,
            RETURN_LEVEL           = 16,
            OPERATING_MODE         = 19,
            LED                    = 25,
            FSR1_L                 = 26,
            FSR1_H                 = 27,
            FSR2_L                 = 28,
            FSR2_H                 = 29,
            FSR3_L                 = 30,
            FSR3_H                 = 31,
            FSR4_L                 = 32,
            FSR4_H                 = 33,
            FSR_X                  = 34,
            FSR_Y                  = 35,
            PRESENT_VOLTAGE        = 42,
            REGISTERED_INSTRUCTION = 44,
            LOCK                   = 47
        };
    };
}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFORM_OPENCR_FSR_HPP
