#ifndef MODULE_PLATFORM_NUGUS_NUGUS_H
#define MODULE_PLATFORM_NUGUS_NUGUS_H

#include <array>
#include <stdexcept>

#include "dynamixel/v2/DynamixelServo.h"
#include "dynamixel/v2/OpenCR.h"

#include "utility/math/comparison.h"

namespace module {
namespace platform {
    namespace nugus {

        struct MX64 : public DynamixelServo {
            MX64(uint8_t ID) : ID(ID) {}
            const uint8_t ID;
        };

        struct MX106 : public DynamixelServo {
            MX106(uint8_t ID) : ID(ID) {}
            const uint8_t ID;
        };

        class NUgus {
        public:
            NUgus();

            /// Picks which direction a motor should be measured in (forward or reverse) -- configurable based on the
            /// specific humanoid being used.
            std::array<int8_t, 20> servo_direction;

            /// Offsets the radian angles of motors to change their 0 position -- configurable based on the specific
            /// humanoid being used.
            std::array<double, 20> servo_offset;

            enum class ID : uint8_t {
                OPENCR           = 200,
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
            MX106 R_KNEE;
            MX106 L_KNEE;
            MX106 R_ANKLE_PITCH;
            MX106 L_ANKLE_PITCH;
            MX106 R_ANKLE_ROLL;
            MX106 L_ANKLE_ROLL;
            MX64 HEAD_YAW;
            MX64 HEAD_PITCH;

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

            constexpr std::array<uint8_t, 20> servo_ids() const {
                return {uint8_t(ID::R_SHOULDER_PITCH), uint8_t(ID::L_SHOULDER_PITCH), uint8_t(ID::R_SHOULDER_ROLL),
                        uint8_t(ID::L_SHOULDER_ROLL),  uint8_t(ID::R_ELBOW),          uint8_t(ID::L_ELBOW),
                        uint8_t(ID::R_HIP_YAW),        uint8_t(ID::L_HIP_YAW),        uint8_t(ID::R_HIP_ROLL),
                        uint8_t(ID::L_HIP_ROLL),       uint8_t(ID::R_HIP_PITCH),      uint8_t(ID::L_HIP_PITCH),
                        uint8_t(ID::R_KNEE),           uint8_t(ID::L_KNEE),           uint8_t(ID::R_ANKLE_PITCH),
                        uint8_t(ID::L_ANKLE_PITCH),    uint8_t(ID::R_ANKLE_ROLL),     uint8_t(ID::L_ANKLE_ROLL),
                        uint8_t(ID::HEAD_YAW),         uint8_t(ID::HEAD_PITCH)};
            }

            // Convert gyroscope readings between control table values and actual values
            float convertGyro(int16_t gyro) const;
            int16_t convertGyro(float gyro) const;

            // Convert accelerometer readings between control table values and actual values
            float convertAcc(int16_t gyro) const;
            int16_t convertAcc(float gyro) const;

            // Convert PWM readings between control table values and actual values
            int16_t convertPWM(int16_t pwm) const;

            // Convert temperature readings between control table values and actual values
            float convertTemperature(uint8_t temp) const;
            uint8_t convertTemperature(float temp) const;

            // Convert voltage readings between control table values and actual values
            float convertVoltage(uint8_t voltage) const;
            float convertVoltage(uint16_t voltage) const;
            uint8_t convertVoltage(float voltage) const;

            // Convert position readings between control table values and actual values
            float convertPosition(uint8_t id, uint32_t position) const;
            uint32_t convertPosition(uint8_t id, float position) const;

            // Convert velocity readings between control table values and actual values
            float convertVelocity(int32_t velocity) const;
            int32_t convertVelocity(float velocity) const;

            // Convert current readings between control table values and actual values
            float convertCurrent(int16_t current) const;
            int16_t convertCurrent(float current) const;

            // Convert P, I, D, and feed-forward gain readings between control table values and actual values
            float convertPGain(uint16_t p_gain) const;
            uint16_t convertPGain(float p_gain) const;
            float convertIGain(uint16_t i_gain) const;
            uint16_t convertIGain(float i_gain) const;
            float convertDGain(uint16_t d_gain) const;
            uint16_t convertDGain(float d_gain) const;
            float convertFFGain(uint16_t ff_gain) const;
            uint16_t convertFFGain(float ff_gain) const;
        };

#pragma pack(push, 1)  // Here we disable the OS putting in padding bytes so we can raw memcpy into this data
        struct DynamixelServoWriteDataPart1 {
            uint8_t torqueEnable;
            uint16_t velocityIGain;
            uint16_t velocityPGain;
            uint16_t velocityDGain;
            uint16_t positionIGain;
            uint16_t positionPGain;
        };

        struct DynamixelServoWriteDataPart2 {
            uint16_t feedforward1stGain;
            uint16_t feedforward2ndGain;
            int16_t goalPWM;
            int16_t goalCurrent;
            int32_t goalVelocity;
            uint32_t profileAcceleration;
            uint32_t profileVelocity;
            uint32_t goalPosition;
        };

        struct DynamixelServoReadData {
            uint8_t torqueEnable;
            uint8_t hardwareErrorStatus;
            int16_t presentPWM;
            int16_t presentCurrent;
            int32_t presentVelocity;
            uint32_t presentPosition;
            uint16_t presentVoltage;
            uint8_t presentTemperature;
        };

        struct OpenCRWriteData {
            uint8_t led;
            uint16_t rgbLED;
            uint8_t buzzer;
        };

        struct OpenCRReadData {
            uint8_t led;
            uint16_t rgbLed;
            uint16_t buzzer;
            uint8_t button;
            uint8_t voltage;
            int16_t gyro[3];
            int16_t acc[3];
        };
#pragma pack(pop)  // Stop bitpacking our results

    }  // namespace nugus
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_NUGUS_NUGUS_H
