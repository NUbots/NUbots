#ifndef MODULE_PLATFORM_OPENCR_CONVERT_HPP
#define MODULE_PLATFORM_OPENCR_CONVERT_HPP

#include <array>
#include <stdexcept>

#include "dynamixel/v2/OpenCR.hpp"

namespace module::platform::OpenCR {

    namespace convert {
        /// @brief Converts a gyroscope reading from the dynamixel to the radian value
        /// @param gyro The dynamixel value to convert
        /// @return The gyro value in radians/second
        float gyro(int16_t gyro);

        /// @brief Converts a radian gyro value to the dynamixel gyro value
        /// @param gyro The gyro value in radians/second to convert
        /// @return The dynamixel gyro value
        int16_t gyro(float gyro);

        /// @brief Converts an accelerometer reading from the dynamixel to a gravity-based value
        /// @param acc The dynamixel value to convert
        /// @return The acc value in g's
        float acc(int16_t acc);

        /// @brief Converts a gravity-based accelerometer value to the dynamixel accelerometer value
        /// @param acc The gravity-based accelerometer value to convert
        /// @return The dynamixel accelerometer value
        int16_t acc(float acc);

        // Convert PWM readings between control table values and actual values
        int16_t PWM(int16_t pwm);

        // Convert temperature readings between control table values and actual values
        float temperature(uint8_t temp);
        uint8_t temperature(float temp);

        // Convert voltage readings between control table values and actual values
        float voltage(uint8_t voltage);
        float voltage(uint16_t voltage);
        uint8_t voltage(float voltage);

        // Convert position readings between control table values and actual values
        float position(uint8_t id,
                       uint32_t position,
                       std::array<int8_t, 20> servo_direction,
                       std::array<double, 20> servo_offset);
        uint32_t position(uint8_t id,
                          float position,
                          std::array<int8_t, 20> servo_direction,
                          std::array<double, 20> servo_offset);

        // Convert velocity readings between control table values and actual values
        float velocity(int32_t velocity);
        int32_t velocity(float velocity);

        // Convert current readings between control table values and actual values
        float current(int16_t current);
        int16_t current(float current);

        // Convert P, I, D, and feed-forward gain readings between control table values and actual values
        float p_gain(uint16_t p_gain);
        uint16_t p_gain(float p_gain);
        float i_gain(uint16_t i_gain);
        uint16_t i_gain(float i_gain);
        float d_gain(uint16_t d_gain);
        uint16_t d_gain(float d_gain);
        float ff_gain(uint16_t ff_gain);
        uint16_t ff_gain(float ff_gain);

    }  // namespace convert

}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFOR_OPENCR_CONVERT_HPP
