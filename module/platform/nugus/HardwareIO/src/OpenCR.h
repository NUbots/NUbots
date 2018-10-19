#ifndef MODULE_PLATFORM_NUGUS_OPENCR_H
#define MODULE_PLATFORM_NUGUS_OPENCR_H

namespace module {
namespace platform {
    namespace nugus {
        class OpenCR {
        public:
            static constexpr uint8_t ID = 200;

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
    }  // namespace nugus
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_NUGUS_OPENCR_H
