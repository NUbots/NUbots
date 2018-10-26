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
            /// Picks which direction a motor should be measured in (forward or reverse) -- configurable based on the
            /// specific humanoid being used.
            static std::array<int8_t, 20> SERVO_DIRECTION;

            /// Offsets the radian angles of motors to change their 0 position -- configurable based on the specific
            /// humanoid being used.
            static std::array<double, 20> SERVO_OFFSET;

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

            static float convertGyro(uint16_t gyro) {
                // Range: -32800 - +32800 = -2000dps - +2000dps
                //
                //               gyro(raw) - min_gyro(raw)
                // gyro(dps) = ----------------------------- * (max_gyro(dps) - min_gyro(dps)) + min_gyro(dps)
                //             max_gyro(raw) - min_gyro(raw)
                //
                uint16_t clamped = utility::math::clamp(-32800, gyro, 32800);
                float normalised = (clamped + 32800.0f) / 65600.0f;
                return normalised * 4000.0f - 2000.0f;
            }

            static uint16_t convertGyro(float gyro) {
                // Range: -32800 - +32800 = -2000dps - +2000dps
                //
                //               gyro(dps) - min_gyro(dps)
                // gyro(raw) = ----------------------------- * (max_gyro(raw) - min_gyro(raw)) + min_gyro(raw)
                //             max_gyro(dps) - min_gyro(dps)
                //
                float clamped    = utility::math::clamp(-2000.0f, gyro, 2000.0f);
                float normalised = (clamped + 2000.0f) / 4000.0f;
                return uint16_t(normalised * 65600.0f - 32800.0f);
            }

            static float convertAcc(uint16_t gyro) {
                // Range: -32768 - +32768 = -2g - +2g
                //
                //               acc(raw) - min_acc(raw)
                // acc(dps) = ----------------------------- * (max_acc(dps) - min_acc(dps)) + min_acc(dps)
                //             max_acc(raw) - min_acc(raw)
                //
                uint16_t clamped = utility::math::clamp(-32768, gyro, 32768);
                float normalised = (clamped + 32768.0f) / 65536.0f;
                return normalised * 4.0f - 2.0f;
            }

            static uint16_t convertAcc(float gyro) {
                // Range: -32768 - +32768 = -2g - +2g
                //
                //               acc(dps) - min_acc(dps)
                // acc(raw) = ----------------------------- * (max_acc(raw) - min_acc(raw)) + min_acc(raw)
                //             max_acc(dps) - min_acc(dps)
                //
                float clamped    = utility::math::clamp(-2.0f, gyro, 2.0f);
                float normalised = (clamped + 2.0f) / 4.0f;
                return uint16_t(normalised * 65536.0f - 32768.0f);
            }

            static uint16_t convertPWM(uint16_t pwm) {
                // Range: -885 - +885
                return utility::math::clamp(-885, pwm, 885);
            }

            static float convertTemperature(uint8_t temp) {
                // Base unit: 1 degree C
                // Range: 0 - 100 = 0 - 100 degrees C
                return utility::math::clamp(0, temp, 100) * 1.0f;
            }

            static uint8_t convertTemperature(float temp) {
                // Base unit: 1 degree C
                // Range: 0 - 100 = 0 - 100 degrees C
                return uint8_t(utility::math::clamp(0.0f, temp, 100.0f));
            }

            static uint8_t convertTemperature(float temp) {
                // Base unit: 1 degree C
                // Range: 0 - 100 = 0 - 100 degrees C
                return uint16_t(utility::math::clamp(0.0f, temp, 100.0f));
            }

            static float convertVoltage(uint16_t voltage) {
                // Base unit: 0.1 volts
                // Range: 95 - 160 =  9.5V - 16.0V
                return utility::math::clamp(95, voltage, 160) * 0.1;
            }

            static uint16_t convertVoltage(float voltage) {
                // Base unit: 0.1 volts
                // Range: 95 - 160 =  9.5V - 16.0V
                return uint16_t(utility::math::clamp(9.5f, voltage * 10.f, 16.0f));
            }

            static float convertVoltage(uint8_t voltage) {
                // Base unit: 0.1 volts
                // Range: 95 - 160 =  9.5V - 16.0V
                return utility::math::clamp(95, voltage, 160) * 0.1;
            }

            static uint8_t convertVoltage(float voltage) {
                // Base unit: 0.1 volts
                // Range: 95 - 160 =  9.5V - 16.0V
                return uint8_t(utility::math::clamp(9.5f, voltage * 10.f, 16.0f));
            }

            static float convertPosition(uint8_t id, uint32_t position) {
                // Base unit: 0.088 degrees = 0.0015358897 rad
                // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
                return utility::math::angle::normalizeAngle(
                    (utility::math::clamp(0, position, 4095) * 0.0015358897f * SERVO_DIRECTION[id]) + SERVO_OFFSET[id]);
            }

            static uint32_t convertPosition(uint8_t id, float position) {
                // Base unit: 0.088 degrees = 0.0015358897 rad
                // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
                float angle = utility::math::angle::normalizeAngle((position - SERVO_OFFSET[id]) * SERVO_DIRECTION[id]);

                return uint32_t(utility::math::clamp(0.0f, angle / 0.0015358897f, 4095.0f));
            }

            static float convertVelocity(uint32_t velocity) {
                // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
                // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
                return utility::math::clamp(-210, velocity, 210) * 0.229f / 60.0f;
            }

            static uint32_t convertVelocity(float velocity) {
                // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
                // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
                return uint32_t(utility::math::clamp(-210.0f, velocity * 60.0f / 0.229f, 210.0f));
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
