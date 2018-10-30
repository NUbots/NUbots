#include "NUgus.h"

#include "utility/math/angle.h"

namespace module {
namespace platform {
    namespace nugus {

        NUgus::NUgus()
            : OPENCR(uint8_t(ID::OPENCR))
            , R_SHOULDER_PITCH(uint8_t(ID::R_SHOULDER_PITCH))
            , L_SHOULDER_PITCH(uint8_t(ID::L_SHOULDER_PITCH))
            , R_SHOULDER_ROLL(uint8_t(ID::R_SHOULDER_ROLL))
            , L_SHOULDER_ROLL(uint8_t(ID::L_SHOULDER_ROLL))
            , R_ELBOW(uint8_t(ID::R_ELBOW))
            , L_ELBOW(uint8_t(ID::L_ELBOW))
            , R_HIP_YAW(uint8_t(ID::R_HIP_YAW))
            , L_HIP_YAW(uint8_t(ID::L_HIP_YAW))
            , R_HIP_ROLL(uint8_t(ID::R_HIP_ROLL))
            , L_HIP_ROLL(uint8_t(ID::L_HIP_ROLL))
            , R_HIP_PITCH(uint8_t(ID::R_HIP_PITCH))
            , L_HIP_PITCH(uint8_t(ID::L_HIP_PITCH))
            , R_KNEE(uint8_t(ID::R_KNEE))
            , L_KNEE(uint8_t(ID::L_KNEE))
            , R_ANKLE_PITCH(uint8_t(ID::R_ANKLE_PITCH))
            , L_ANKLE_PITCH(uint8_t(ID::L_ANKLE_PITCH))
            , R_ANKLE_ROLL(uint8_t(ID::R_ANKLE_ROLL))
            , L_ANKLE_ROLL(uint8_t(ID::L_ANKLE_ROLL))
            , HEAD_YAW(uint8_t(ID::HEAD_YAW))
            , HEAD_PITCH(uint8_t(ID::HEAD_PITCH)) {}

        float NUgus::convertGyro(int16_t gyro) const {
            // Range: -32800 - +32800 = -2000dps - +2000dps
            //
            //               gyro(raw) - min_gyro(raw)
            // gyro(dps) = ----------------------------- * (max_gyro(dps) - min_gyro(dps)) + min_gyro(dps)
            //             max_gyro(raw) - min_gyro(raw)
            //
            int16_t clamped  = utility::math::clamp(int16_t(-32800), gyro, int16_t(32800));
            float normalised = (clamped + 32800.0f) / 65600.0f;
            return normalised * 4000.0f - 2000.0f;
        }

        int16_t NUgus::convertGyro(float gyro) const {
            // Range: -32800 - +32800 = -2000dps - +2000dps
            //
            //               gyro(dps) - min_gyro(dps)
            // gyro(raw) = ----------------------------- * (max_gyro(raw) - min_gyro(raw)) + min_gyro(raw)
            //             max_gyro(dps) - min_gyro(dps)
            //
            float clamped    = utility::math::clamp(-2000.0f, gyro, 2000.0f);
            float normalised = (clamped + 2000.0f) / 4000.0f;
            return int16_t(normalised * 65600.0f - 32800.0f);
        }

        float NUgus::convertAcc(int16_t gyro) const {
            // Range: -32768 - +32768 = -2g - +2g
            //
            //               acc(raw) - min_acc(raw)
            // acc(dps) = ----------------------------- * (max_acc(dps) - min_acc(dps)) + min_acc(dps)
            //             max_acc(raw) - min_acc(raw)
            //
            int16_t clamped  = utility::math::clamp(int16_t(-32768), gyro, int16_t(32768));
            float normalised = (clamped + 32768.0f) / 65536.0f;
            return normalised * 4.0f - 2.0f;
        }

        int16_t NUgus::convertAcc(float gyro) const {
            // Range: -32768 - +32768 = -2g - +2g
            //
            //               acc(dps) - min_acc(dps)
            // acc(raw) = ----------------------------- * (max_acc(raw) - min_acc(raw)) + min_acc(raw)
            //             max_acc(dps) - min_acc(dps)
            //
            float clamped    = utility::math::clamp(-2.0f, gyro, 2.0f);
            float normalised = (clamped + 2.0f) / 4.0f;
            return int16_t(normalised * 65536.0f - 32768.0f);
        }

        int16_t NUgus::convertPWM(int16_t pwm) const {
            // Range: -885 - +885
            return utility::math::clamp(int16_t(-885), pwm, int16_t(885));
        }

        float NUgus::convertTemperature(uint8_t temp) const {
            // Base unit: 1 degree C
            // Range: 0 - 100 = 0 - 100 degrees C
            return utility::math::clamp(uint8_t(0), temp, uint8_t(100)) * 1.0f;
        }

        uint8_t NUgus::convertTemperature(float temp) const {
            // Base unit: 1 degree C
            // Range: 0 - 100 = 0 - 100 degrees C
            return uint8_t(utility::math::clamp(0.0f, temp, 100.0f));
        }

        float NUgus::convertVoltage(uint8_t voltage) const {
            // Base unit: 0.1 volts
            // Range: 95 - 160 =  9.5V - 16.0V
            return utility::math::clamp(uint8_t(95), voltage, uint8_t(160)) * 0.1;
        }

        float NUgus::convertVoltage(uint16_t voltage) const {
            // Base unit: 0.1 volts
            // Range: 95 - 160 =  9.5V - 16.0V
            return utility::math::clamp(uint16_t(95), voltage, uint16_t(160)) * 0.1;
        }

        uint8_t NUgus::convertVoltage(float voltage) const {
            // Base unit: 0.1 volts
            // Range: 95 - 160 =  9.5V - 16.0V
            return uint8_t(utility::math::clamp(9.5f, voltage * 10.f, 16.0f));
        }

        float NUgus::convertPosition(uint8_t id, uint32_t position) const {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
            return utility::math::angle::normalizeAngle(
                (utility::math::clamp(uint32_t(0), position, uint32_t(4095)) * 0.0015358897f * servo_direction[id])
                + servo_offset[id]);
        }

        uint32_t NUgus::convertPosition(uint8_t id, float position) const {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
            float angle = utility::math::angle::normalizeAngle((position - servo_offset[id]) * servo_direction[id]);

            return uint32_t(utility::math::clamp(0.0f, angle / 0.0015358897f, 4095.0f));
        }

        float NUgus::convertVelocity(int32_t velocity) const {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            return utility::math::clamp(int32_t(-210), velocity, int32_t(210)) * 0.229f / 60.0f;
        }

        int32_t NUgus::convertVelocity(float velocity) const {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            return int32_t(utility::math::clamp(-210.0f, velocity * 60.0f / 0.229f, 210.0f));
        }

        float NUgus::convertCurrent(int16_t current) const {
            // Base unit: 3.36mA = 0.00336A
            // Range: -2047 - +2047 = -6.87792A - +6.87792A
            return utility::math::clamp(int16_t(-2047), current, int16_t(2047)) * 0.00336f;
        }

        int16_t NUgus::convertCurrent(float current) const {
            // Base unit: 3.36mA = 0.00336A
            // Range: -2047 - +2047 = -6.87792A - +6.87792A
            return int16_t(utility::math::clamp(-6.87792f, current, 6.87792f) / 0.00336f);
        }

        float NUgus::convertPGain(uint16_t p_gain) const {
            return p_gain / 128.0f;
        }

        uint16_t NUgus::convertPGain(float p_gain) const {
            return uint16_t(p_gain * 128.0f);
        }

        float NUgus::convertIGain(uint16_t i_gain) const {
            return i_gain / 65536.0f;
        }

        uint16_t NUgus::convertIGain(float i_gain) const {
            return uint16_t(i_gain * 65536.0f);
        }

        float NUgus::convertDGain(uint16_t d_gain) const {
            return d_gain / 18.0f;
        }

        uint16_t NUgus::convertDGain(float d_gain) const {
            return uint16_t(d_gain * 18.0f);
        }

        float NUgus::convertFFGain(uint16_t ff_gain) const {
            return ff_gain / 4.0f;
        }

        uint16_t NUgus::convertFFGain(float ff_gain) const {
            return uint16_t(ff_gain * 4.0f);
        }
    }  // namespace nugus
}  // namespace platform
}  // namespace module
