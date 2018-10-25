#ifndef MODULE_PLATFORM_NUGUS_NUGUS_H
#define MODULE_PLATFORM_NUGUS_NUGUS_H

#include "DynamixelServo.h"
#include "OpenCR.h"

#include "utility/math/comparison.h"

namespace module {
namespace platform {
    namespace nugus {

        struct MX64 : DynamixelServo {
            uint8_t ID;
            MX64(uint8_t ID) : ID(ID) {}
        };

        struct MX106 : DynamixelServo {
            uint8_t ID;
            MX106(uint8_t ID) : ID(ID) {}
        };

        class NUgus {
        public:
            enum ID {
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

            static OpenCR OPENCR         = OpenCR(ID::OPENCR);
            static MX64 R_SHOULDER_PITCH = MX64(ID::R_SHOULDER_PITCH);
            static MX64 L_SHOULDER_PITCH = MX64(ID::L_SHOULDER_PITCH);
            static MX64 R_SHOULDER_ROLL  = MX64(ID::R_SHOULDER_ROLL);
            static MX64 L_SHOULDER_ROLL  = MX64(ID::L_SHOULDER_ROLL);
            static MX64 R_ELBOW          = MX64(ID::R_ELBOW);
            static MX64 L_ELBOW          = MX64(ID::L_ELBOW);
            static MX106 R_HIP_YAW       = MX106(ID::R_HIP_YAW);
            static MX106 L_HIP_YAW       = MX106(ID::L_HIP_YAW);
            static MX106 R_HIP_ROLL      = MX106(ID::R_HIP_ROLL);
            static MX106 L_HIP_ROLL      = MX106(ID::L_HIP_ROLL);
            static MX106 R_HIP_PITCH     = MX106(ID::R_HIP_PITCH);
            static MX106 L_HIP_PITCH     = MX106(ID::L_HIP_PITCH);
            static MX106 R_KNEE          = MX106(ID::R_KNEE);
            static MX106 L_KNEE          = MX106(ID::L_KNEE);
            static MX106 R_ANKLE_PITCH   = MX106(ID::R_ANKLE_PITCH);
            static MX106 L_ANKLE_PITCH   = MX106(ID::L_ANKLE_PITCH);
            static MX106 R_ANKLE_ROLL    = MX106(ID::R_ANKLE_ROLL);
            static MX106 L_ANKLE_ROLL    = MX106(ID::L_ANKLE_ROLL);
            static MX64 HEAD_YAW         = MX64(ID::HEAD_YAW);
            static MX64 HEAD_PITCH       = MX64(ID::HEAD_PITCH);

            static constexpr DynamixelDevice& operator[](const ID& id) {
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

            static constexpr std::array<uint8_t, ID::HEAD_PITCH - ID::R_SHOULDER_PITCH + 1> servo_ids() const {
                std::array<uint8_t, ID::HEAD_PITCH - ID::R_SHOULDER_PITCH + 1> ids;
                std::iota(ids.begin(), ids.end(), R_SHOULDER_PITCH);
                return ids;
            }

            static uint16_t convertPWM(uint16_t pwm) {
                // Range: -885 - +885
                return utility::math::clamp(-885, pwm, 885);
            }

            static float convertVelocity(uint16_t velocity) {
                // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
                // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
                return utility::math::clamp(-210, velocity, 210) * 0.229f / 60.0f;
            }

            static uint16_t convertVelocity(float velocity) {
                // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
                // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
                return uint16_t(utility::math::clamp(-210.0f, velocity * 60.0f / 0.229f, 210.0f));
            }

            static float convertCurrent(uint16_t current) {
                // Base unit: 3.36mA = 0.00336A
                // Range: -2047 - +2047 = -6.87792A - +6.87792A
                return utility::math::clamp(-2047, current, 2047) * 0.00336f;
            }

            static uint16_t convertCurrent(float current) {
                // Base unit: 3.36mA = 0.00336A
                // Range: -2047 - +2047 = -6.87792A - +6.87792A
                return uint16_t(utility::math::clamp(-6.87792f, current, 6.87792f) / 0.00336f);
            }

            // Convert P, I, D, and feed-forward gains between control table values and actual values
            static float convertPGain(uint16_t p_gain) const {
                return p_gain / 128.0f;
            }

            static uint16_t convertPGain(float p_gain) const {
                return uint16_t(p_gain * 128.0f);
            }

            static float convertIGain(uint16_t i_gain) const {
                return i_gain / 65536.0f;
            }

            static uint16_t convertIGain(float i_gain) const {
                return uint16_t(i_gain * 65536.0f);
            }

            static float convertDGain(uint16_t d_gain) const {
                return d_gain / 18.0f;
            }

            static uint16_t convertDGain(float d_gain) const {
                return uint16_t(d_gain * 18.0f);
            }

            static float convertFFGain(uint16_t ff_gain) const {
                return ff_gain / 4.0f;
            }

            static uint16_t convertFFGain(float ff_gain) const {
                return uint16_t(ff_gain * 4.0f);
            }
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
            uint16_t goalPwm;
            uint16_t goalCurrent;
            uint32_t goalVelocity;
            uint32_t profileAcceleration;
            uint32_t profileVelocity;
            uint32_t goalPosition;
        };

        struct DynamixelServoReadData {
            uint8_t torqueEnable;
            uint8_t hardwareErrorStatus;
            uint16_t presentPwm;
            uint16_t presentCurrent;
            uint32_t presentVelocity;
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
            uint16_t[3] gyro;
            uint16_t[3] acc;
        };
#pragma pack(pop)  // Stop bitpacking our results

    }  // namespace nugus
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_NUGUS_NUGUS_H
