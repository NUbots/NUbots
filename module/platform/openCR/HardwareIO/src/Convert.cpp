#include "Convert.hpp"

#include <fmt/format.h>
#include <limits>
#include <nuclear>

#include "NUgus.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"

namespace module::platform::openCR {

    namespace convert {

        /**
         * @todo properly name and comment these functions. Copied accross from
         * old code but it's horribly documented. The overloading of param/return
         * types is what implies whether its a forward or backward conversion.
         */

        float gyro(int16_t raw) {
            // Range: -32768 - +32767 = -2000dps - +2000dps
            //
            //               gyro(raw) - min_gyro(raw)
            // gyro(dps) = ----------------------------- * (max_gyro(dps) - min_gyro(dps)) + min_gyro(dps)
            //             max_gyro(raw) - min_gyro(raw)
            //

            const float converted = raw * 2000.0 / INT16_MAX;

            return float(converted * M_PI / 180);  // conv to rad/s
        }

        int16_t gyro(float SI) {
            // Range: -32800 - +32800 = -2000dps - +2000dps
            //
            //               gyro(dps) - min_gyro(dps)
            // gyro(raw) = ----------------------------- * (max_gyro(raw) - min_gyro(raw)) + min_gyro(raw)
            //             max_gyro(dps) - min_gyro(dps)
            //

            float dps = SI * 180 / M_PI;  // first convert from system unit rad/s

            int16_t converted = int16_t(dps * INT16_MAX / 2000);

            return converted;
        }


        float acc(int16_t raw) {
            // Range: -32768 - +32768 = -2g - +2g
            //
            //               acc(raw) - min_acc(raw)
            // acc(dps) = ----------------------------- * (max_acc(dps) - min_acc(dps)) + min_acc(dps)
            //             max_acc(raw) - min_acc(raw)
            //

            return float(raw * 2 / INT16_MAX);
        }

        int16_t acc(float gs) {
            // Range: -32768 - +32768 = -2g - +2g
            //
            //               acc(dps) - min_acc(dps)
            // acc(raw) = ----------------------------- * (max_acc(raw) - min_acc(raw)) + min_acc(raw)
            //             max_acc(dps) - min_acc(dps)
            //

            return int16_t(gs * INT16_MAX / 2);
        }

        /// @todo Maybe store pwm internally as a float percentage?

        int16_t PWM(float pwm) {
            // Range: -885 - +885
            return int16_t(utility::math::clamp(-885.0f, pwm, 885.0f));
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

        /**
         * @todo the servo offset can now be replaced by the Homing Offset in the control table I think
         *       This still requires some verification and I think the offset may be the opposite of what
         *       we currently have (i.e. will be added instead of subtracted).
         *       See https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#homing-offset
         */

        float position(uint8_t id,
                       uint32_t position,
                       std::array<int8_t, 20> servo_direction,
                       std::array<double, 20> servo_offset) {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
            return utility::math::angle::normalizeAngle(
                (utility::math::clamp(uint32_t(0), position, uint32_t(4095)) * 0.0015358897f * servo_direction[id])
                + servo_offset[id]);
        }

        uint32_t position(uint8_t id,
                          float position,
                          std::array<int8_t, 20> servo_direction,
                          std::array<double, 20> servo_offset) {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad
            float angle = utility::math::angle::normalizeAngle((position - servo_offset[id]) * servo_direction[id]);

            return uint32_t(utility::math::clamp(0.0f, angle / 0.0015358897f, 4095.0f));
        }


        float velocity(int32_t velocity) {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            return utility::math::clamp(int32_t(-1023), velocity, int32_t(1023)) * 0.229f / 60.0f;
        }

        int32_t velocity(float velocity) {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            return int32_t(utility::math::clamp(-1023.0f, (velocity * 60.0f / 0.229f), 1023.0f));
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

        /// @todo clamp gain values to [0,16383]

        /**
         * Controller gain = Ram table gain / 128
         */
        float PGain(uint16_t p_gain) {
            return float(p_gain / 128.0f);
        }

        uint16_t PGain(float p_gain) {
            return uint16_t(p_gain * 128.0f);
        }

        /**
         * Controller gain = Ram table gain / 65536
         */
        float IGain(uint16_t i_gain) {
            return i_gain / 65536.0f;
        }

        uint16_t IGain(float i_gain) {
            return uint16_t(i_gain * 65536.0f);
        }

        /**
         * Controller gain = Ram table gain / 16
         */
        float DGain(uint16_t d_gain) {
            return d_gain / 16.0f;
        }

        uint16_t DGain(float d_gain) {
            return uint16_t(d_gain * 16.0f);
        }


        float FFGain(uint16_t ff_gain) {
            return ff_gain / 4.0f;
        }

        uint16_t FFGain(float ff_gain) {
            return uint16_t(ff_gain * 4.0f);
        }


        // float fsrForce(const uint16_t& value) {
        //     // Base unit: 1mN = 0.001N
        //     return value * 0.001;
        // }

        // float fsrCentre(const bool& left, const uint8_t& value) {
        //     if (value == 0xFF) {
        //         // Return NaN if there is no centre
        //         return std::numeric_limits<float>::quiet_NaN();
        //     }
        //     // Flips right foot coordinates to match robot coords
        //     // See:
        //     //
        //     http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
        //     if (left) {
        //         // This normalises the value between -1 and 1
        //         return double(value - 127) / 127.0;
        //     }
        //     // This normalises the value between -1 and 1
        //     return double(127 - value) / 127.0;
        // }
    }  // namespace convert

}  // namespace module::platform::openCR
