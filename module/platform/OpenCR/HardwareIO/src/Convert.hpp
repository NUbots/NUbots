#ifndef MODULE_PLATFORM_OPENCR_CONVERT_HPP
#define MODULE_PLATFORM_OPENCR_CONVERT_HPP

#include <array>
#include <stdexcept>

#include "dynamixel/v2/OpenCR.hpp"

namespace module::platform::OpenCR {

    namespace convert {
        /// @brief Converts a gyroscope reading from the dynamixel to the radian value
        /// @param gyro The dynamixel value to convert, in the range of -32768 to +32767
        /// @return The gyro value in radians/second
        float gyro(int16_t gyro);

        /// @brief Converts a radian gyro value to the dynamixel gyro value
        /// @param gyro The gyro value in radians/second to convert, in the range of -34.90659 to 34.90659
        /// @return The dynamixel gyro value
        int16_t gyro(float gyro);

        /// @brief Converts an accelerometer reading from the dynamixel to a gravity-based value
        /// @param acc The dynamixel value to convert, in the range of -32768 to +32768
        /// @return The acc value in g's
        float acc(int16_t acc);

        /// @brief Converts a gravity-based accelerometer value to the dynamixel accelerometer value
        /// @param acc The gravity-based accelerometer value to convert, in the range of +2g to -2g
        /// @return The dynamixel accelerometer value
        int16_t acc(float acc);

        /// @brief Clamps the pulse width modulation value to the range of -885 to +885
        /// @param pwm The raw dynamixel pwm value
        /// @return The stored clamped pwm value
        int16_t PWM(int16_t pwm);

        /// @brief Clamps the temperature between 0 and 100 degrees celsius
        /// @param temp The raw dynamixel temperature value
        /// @return The clamped value as a float
        float temperature(uint8_t temp);

        /// @brief Clamps the temperature between 0 and 100 degrees celsius
        /// @param temp The stored temperature value
        /// @return The clamped value to send to the dynamixel
        uint8_t temperature(float temp);

        /// @brief Converts the raw dynamixel voltage value in decivolts to volts, clamped betwen 9.5 and 16.0 volts
        /// @param voltage raw dynamixel voltage in decivolts
        /// @return The voltage in volts between 9.5 and 16.0
        float voltage(uint8_t voltage);

        /// @brief Converts the raw dynamixel voltage value in decivolts to volts, clamped betwen 9.5 and 16.0 volts
        /// @param voltage raw dynamixel voltage in decivolts
        /// @return The voltage in volts between 9.5 and 16.0
        float voltage(uint16_t voltage);

        /// @brief Converts the voltage to the raw dynamixel voltage value in decivolts, clamped betwen 95 and 160
        /// @param voltage The voltage in volts between 9.5 and 16
        /// @return The raw dynamixel voltage value in decivolts
        uint8_t voltage(float voltage);

        /// @brief Converts the raw dynamixel position value to radians
        /// @param id the id of the servo to convert
        /// @param position the raw position value
        /// @param servo_direction the direction of the servo (clockwise or anticlockwise)
        /// @param servo_offset the zero offset of the servo
        /// @return the position value in radians
        float position(uint8_t id,
                       uint32_t position,
                       std::array<int8_t, 20> servo_direction,
                       std::array<double, 20> servo_offset);

        /// @brief Converts the radian position value to the raw dynamixel position value
        /// @param id the id of the servo to convert
        /// @param position the raw position value
        /// @param servo_direction the direction of the servo (clockwise or anticlockwise)
        /// @param servo_offset the zero offset of the servo
        /// @return The raw dynamixel position value in the range 0 to 4095
        uint32_t position(uint8_t id,
                          float position,
                          std::array<int8_t, 20> servo_direction,
                          std::array<double, 20> servo_offset);

        /// @brief Converts the raw dynamixel velocity value to radians/second
        /// @param velocity Raw dynamixel velocity in the range -210 to 210 where each unit is 0.229 rpm
        /// @return The velocity value in radians/second
        float velocity(int32_t velocity);

        /// @brief Converts the velocity value in radians/second to the raw dynamixel velocity value
        /// @param velocity The velocity value in radians/second
        /// @return Raw dynamixel velocity in the range -210 to 210 where each unit is 0.229 rpm
        int32_t velocity(float velocity);

        /// @brief Converts the raw dynamixel current value to amps
        /// @param current The current value between -1941 and 1941 where each unit is 3.36mA
        /// @return The current value in amps
        float current(int16_t current);

        /// @brief Converts the current value in amps to the raw dynamixel current value
        /// @param current The current value in amps
        /// @return The current value between -1941 and 1941 where each unit is 3.36mA
        int16_t current(float current);

        /// @brief Converts the raw dynamixel p gain value to the p gain by dividing by 128
        /// @param p_gain The raw dynamixel p gain
        /// @return The converted p gain
        float p_gain(uint16_t p_gain);

        /// @brief Converts the p gain to the raw dynamixel p gain value by multiplying by 128
        /// @param p_gain The stored p gain value
        /// @return The raw dynamixel p gain value
        uint16_t p_gain(float p_gain);

        /// @brief Converts the raw dynamixel i gain value to the i gain by dividing by 65536
        /// @param i_gain The raw dynamixel i gain
        /// @return The converted i gain
        float i_gain(uint16_t i_gain);

        /// @brief Converts the i gain to the raw dynamixel i gain value by multiplying by 65536
        /// @param i_gain The stored i gain value
        /// @return The raw dynamixel i gain value
        uint16_t i_gain(float i_gain);

        /// @brief Converts the raw dynamixel d gain value to the d gain by dividing by 16
        /// @param d_gain The raw dynamixel d gain
        /// @return The converted d gain
        float d_gain(uint16_t d_gain);

        /// @brief Converts the d gain to the raw dynamixel d gain value by multiplying by 16
        /// @param d_gain The stored d gain value
        /// @return The raw dynamixel d gain value
        uint16_t d_gain(float d_gain);

        /// @brief Converts the raw dynamixel ff1 gain value to the ff1 gain by dividing by 4
        /// @param ff_gain The raw dynamixel ff1 gain
        /// @return The converted ff1 gain
        float ff_gain(uint16_t ff_gain);

        /// @brief Converts the ff1 gain to the raw dynamixel ff1 gain value by multiplying by 4
        /// @param ff_gain The stored ff1 gain value
        /// @return The raw dynamixel ff1 gain value
        uint16_t ff_gain(float ff_gain);

    }  // namespace convert

}  // namespace module::platform::OpenCR

#endif  // MODULE_PLATFOR_OPENCR_CONVERT_HPP
