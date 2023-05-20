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
