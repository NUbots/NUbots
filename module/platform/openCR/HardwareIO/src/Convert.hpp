#ifndef MODULE_PLATFORM_OPENCR_NUGUS_HPP
#define MODULE_PLATFORM_OPENCR_NUGUS_HPP

#include <array>
#include <stdexcept>

// #include "dynamixel/v2/DynamixelServo.h"
#include "dynamixel/v2/OpenCR.h"

namespace module::platform::openCR {

    namespace convert {
        // Convert gyroscope readings between control table values and actual values
        float gryo(int16_t gyro);
        int16_t gyro(float gyro);

        // Convert accelerometer readings between control table values and actual values
        float acc(int16_t gyro);
        int16_t acc(float gyro);

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
        float position(uint8_t id, uint32_t position, NUgus& robot);
        uint32_t position(uint8_t id, float position, NUgus& robot);

        // Convert velocity readings between control table values and actual values
        float velocity(int32_t velocity);
        int32_t velocity(float velocity);

        // Convert current readings between control table values and actual values
        float current(int16_t current);
        int16_t current(float current);

        // Convert P, I, D, and feed-forward gain readings between control table values and actual values
        float PGain(uint16_t p_gain);
        uint16_t PGain(float p_gain);
        float IGain(uint16_t i_gain);
        uint16_t IGain(float i_gain);
        float DGain(uint16_t d_gain);
        uint16_t DGain(float d_gain);
        float FFGain(uint16_t ff_gain);
        uint16_t FFGain(float ff_gain);

        // Convert FSR readings
        float fsrForce(const uint16_t& value);
        float fsrCentre(const bool& left, const uint8_t& value);

    }  // namespace module::platform::openCR::convert

}  // namespace module::platform::openCR

#endif  // MODULE_PLATFOR_OPENCR_NUGUS_HPP
