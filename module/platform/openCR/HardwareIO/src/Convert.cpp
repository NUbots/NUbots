#include "Convert.hpp"

#include "NUgus.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"

namespace module::platform::openCR {

    namespace convert {

        float gyro(int16_t gyro) {
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

        int16_t gyro(float gyro) {
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


        float acc(int16_t gyro) {
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

        int16_t acc(float gyro) {
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


        int16_t PWM(int16_t pwm) {
            // Range: -885 - +885
            return utility::math::clamp(int16_t(-885), pwm, int16_t(885));
        }


        float temperature(uint8_t temp) {
            // Base unit: 1 degree C
            // Range: 0 - 100 = 0 - 100 degrees C
            return utility::math::clamp(uint8_t(0), temp, uint8_t(100)) * 1.0f;
        }

        uint8_t temperature(float temp) {
            // Base unit: 1 degree C
            // Range: 0 - 100 = 0 - 100 degrees C
            return uint8_t(utility::math::clamp(0.0f, temp, 100.0f));
        }


        float voltage(uint8_t voltage) {
            // Base unit: 0.1 volts
            // Range: 95 - 160 =  9.5V - 16.0V
            return utility::math::clamp(uint8_t(95), voltage, uint8_t(160)) * 0.1;
        }

        float voltage(uint16_t voltage) {
            // Base unit: 0.1 volts
            // Range: 95 - 160 =  9.5V - 16.0V
            return utility::math::clamp(uint16_t(95), voltage, uint16_t(160)) * 0.1;
        }

        uint8_t voltage(float voltage) {
            // Base unit: 0.1 volts
            // Range: 95 - 160 =  9.5V - 16.0V
            return uint8_t(utility::math::clamp(9.5f, voltage * 10.f, 16.0f));
        }

        // takes in a refernce to the robot we're working with to allow it to access servo_direction
        float position(uint8_t id, uint32_t position, NUgus& robot) {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
            return utility::math::angle::normalizeAngle((utility::math::clamp(uint32_t(0), position, uint32_t(4095))
                                                         * 0.0015358897f * robot::servo_direction[id])
                                                        + servo_offset[id]);
        }

        // takes in a refernce to the robot we're working with to allow it to access servo_direction
        uint32_t position(uint8_t id, float position, NUgus& robot) {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
            float angle =
                utility::math::angle::normalizeAngle((position - NUgus::servo_offset[id]) * robot::servo_direction[id]);

            return uint32_t(utility::math::clamp(0.0f, angle / 0.0015358897f, 4095.0f));
        }


        float velocity(int32_t velocity) {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            return utility::math::clamp(int32_t(-210), velocity, int32_t(210)) * 0.229f / 60.0f;
        }

        int32_t velocity(float velocity) {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            return int32_t(utility::math::clamp(-210.0f, velocity * 60.0f / 0.229f, 210.0f));
        }


        float current(int16_t current) {
            // Base unit: 3.36mA = 0.00336A
            // Range: -2047 - +2047 = -6.87792A - +6.87792A
            return utility::math::clamp(int16_t(-2047), current, int16_t(2047)) * 0.00336f;
        }

        int16_t current(float current) {
            // Base unit: 3.36mA = 0.00336A
            // Range: -2047 - +2047 = -6.87792A - +6.87792A
            return int16_t(utility::math::clamp(-6.87792f, current, 6.87792f) / 0.00336f);
        }


        float pGain(uint16_t p_gain) {
            return float(p_gain / 128.0f);
        }

        uint16_t pGain(float p_gain) {
            return uint16_t(p_gain * 128.0f);
        }


        float iGain(uint16_t i_gain) {
            return i_gain / 65536.0f;
        }

        uint16_t iGain(float i_gain) {
            return uint16_t(i_gain * 65536.0f);
        }


        float dGain(uint16_t d_gain) {
            return d_gain / 18.0f;
        }

        uint16_t dGain(float d_gain) {
            return uint16_t(d_gain * 18.0f);
        }


        float ffGain(uint16_t ff_gain) {
            return ff_gain / 4.0f;
        }

        uint16_t ffGain(float ff_gain) {
            return uint16_t(ff_gain * 4.0f);
        }


        float fsrForce(const uint16_t& value) {
            // Base unit: 1mN = 0.001N
            return value * 0.001;
        }

        float fsrCentre(const bool& left, const uint8_t& value) {
            if (value == 0xFF) {
                // Return NaN if there is no centre
                return std::numeric_limits<float>::quiet_NaN();
            }
            // Flips right foot coordinates to match robot coords
            // See:
            // http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
            if (left) {
                // This normalises the value between -1 and 1
                return double(value - 127) / 127.0;
            }
            // This normalises the value between -1 and 1
            return double(127 - value) / 127.0;
        }
    }  // namespace convert

}  // namespace module::platform::openCR