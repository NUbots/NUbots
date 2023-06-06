#include "Convert.hpp"

#include <fmt/format.h>
#include <limits>
#include <nuclear>

#include "NUgus.hpp"

#include "utility/math/angle.hpp"
#include "utility/math/comparison.hpp"

namespace module::platform::OpenCR {

    namespace convert {

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

        float position(uint8_t id,
                       uint32_t data,
                       std::array<int8_t, 20> servo_direction,
                       std::array<double, 20> servo_offset) {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad

            // Ensure we're working within our expected range
            data = utility::math::clamp(uint32_t(0), data, uint32_t(4095));

            // Data is sent as an unsigned int, need to offset the value since the angle can be negative
            int32_t scaled_value = int32_t(data) - 2048;

            // Do the actual converstion to angle
            float angle = scaled_value * 0.0015358897f;

            // Apply the servo specific operations
            angle *= servo_direction[id];
            angle += servo_offset[id];

            // Normalise the angle to (-pi, pi] for internal use
            return utility::math::angle::normalizeAngle(angle);
        }

        uint32_t position(uint8_t id,
                          float angle,
                          std::array<int8_t, 20> servo_direction,
                          std::array<double, 20> servo_offset) {
            // Base unit: 0.088 degrees = 0.0015358897 rad
            // Range: 0 - 4095 = 0 - 360.36 = 6.2894683215 rad

            // first undo the servo specific operations
            angle -= servo_offset[id];
            angle *= servo_direction[id];

            // Normalise the angle to (-pi, pi] in case the offset changed this
            angle = utility::math::angle::normalizeAngle(angle);

            // Do the actual conversion to the control table data and apply the correction as per the block comment in
            // the function above.
            // Note the addition must happen inline as it ensures `data` doesn't hold a negative value.
            uint32_t data = (angle / 0.0015358897f) + 2048;


            return utility::math::clamp(uint32_t(0), data, uint32_t(4095));
        }


        float velocity(int32_t velocity) {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            // Default servo limits for velocity - minimum of all for X-Series, MX-106 and MX-64
            // X-Series has the minimum at 167
            return utility::math::clamp(int32_t(-167), velocity, int32_t(167)) * 0.229f / 60.0f;
        }

        int32_t velocity(float velocity) {
            // Base unit: 0.229 rpm = 0.0038166667 Hz (factor = 1/60)
            // Range: -210 - +210 = -48.09 rpm - +48.09 rpm
            // Default servo limits for velocity - minimum of all for X-Series, MX-106 and MX-64
            // X-Series has the minimum at 167
            return int32_t(utility::math::clamp(-167.0f, (velocity * 60.0f / 0.229f), 167.0f));
        }

        uint32_t profile_velocity(float profile_velocity) {
            // Base unit: 1 millisecond
            // Range: 0 - 32767 milliseconds
            // Time-based Profile sets the time span to reach the goal position
            uint32_t data = uint32_t(profile_velocity);
            return utility::math::clamp(uint32_t(0), data, uint32_t(32767));
        }


        float current(int16_t current) {
            // Base unit: 3.36mA = 0.00336A
            // Range: -1941 - +1941
            // Default servo limits for current
            return utility::math::clamp(int16_t(-1941), current, int16_t(1941)) * 0.00336f;
        }

        int16_t current(float current) {
            // Base unit: 3.36mA = 0.00336A
            // Range: -1941 - +1941
            // Default servo limits for current
            return int16_t(utility::math::clamp(-1941, int(current / 0.00336f), 1941));
        }

        /**
         * Controller gain = Ram table gain / 128
         */
        float p_gain(uint16_t p_gain) {
            return float(p_gain / 128.0f);
        }

        uint16_t p_gain(float p_gain) {
            return uint16_t(p_gain * 128.0f);
        }

        /**
         * Controller gain = Ram table gain / 65536
         */
        float i_gain(uint16_t i_gain) {
            return i_gain / 65536.0f;
        }

        uint16_t i_gain(float i_gain) {
            return uint16_t(i_gain * 65536.0f);
        }

        /**
         * Controller gain = Ram table gain / 16
         */
        float d_gain(uint16_t d_gain) {
            return d_gain / 16.0f;
        }

        uint16_t d_gain(float d_gain) {
            return uint16_t(d_gain * 16.0f);
        }


        float ff_gain(uint16_t ff_gain) {
            return ff_gain / 4.0f;
        }

        uint16_t ff_gain(float ff_gain) {
            return uint16_t(ff_gain * 4.0f);
        }

    }  // namespace convert

}  // namespace module::platform::OpenCR
