/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "SensorFilter.hpp"

#include "utility/input/FrameID.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::input {

    using message::input::Sensors;
    using message::platform::RawSensors;

    using utility::input::FrameID;
    using utility::platform::get_raw_servo;
    using utility::platform::make_packet_error_string;
    using utility::platform::make_servo_hardware_error_string;

    void SensorFilter::update_raw_sensors(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {

        // Mask to ignore the alert bit (because servo errors are handled separately)
        bool subcontroller_packet_error =
            (raw_sensors.subcontroller_error & ~RawSensors::PacketError::ALERT) != RawSensors::PacketError::PACKET_OK;

        // Check for errors on the platform and FSRs
        if (subcontroller_packet_error) {
            log<WARN>(make_packet_error_string("Platform", raw_sensors.subcontroller_error));
        }

        // **************** Servos ****************
        for (uint32_t id = 0; id < n_servos; ++id) {
            const auto& raw_servo       = get_raw_servo(id, raw_sensors);
            const auto& hardware_status = raw_servo.hardware_error;

            // Check for an error on the servo and report it
            if (hardware_status != RawSensors::HardwareError::HARDWARE_OK) {
                log<WARN>(make_servo_hardware_error_string(raw_servo, id));
            }

            // Determine the current position with potential fallback to the last known good position
            double current_position = raw_servo.present_position;
            if (previous_sensors && (hardware_status == RawSensors::HardwareError::MOTOR_ENCODER)) {
                current_position = previous_sensors->servo[id].present_position;
                log<DEBUG>("Suspected encoder error on servo ", id, ": Using last known good position.");
            }

            sensors->servo.emplace_back(
                hardware_status,
                id,
                raw_servo.torque_enabled,
                raw_servo.position_p_gain,
                raw_servo.position_i_gain,
                raw_servo.position_d_gain,
                raw_servo.goal_position,
                raw_servo.profile_velocity,
                current_position,
                /* If there is an encoder error, then use the last known good velocity */
                ((hardware_status == RawSensors::HardwareError::MOTOR_ENCODER) && previous_sensors)
                    ? previous_sensors->servo[id].present_velocity
                    : raw_servo.present_velocity,
                raw_servo.present_current,
                raw_servo.voltage,
                static_cast<float>(raw_servo.temperature));
        }

        // **************** Accelerometer and Gyroscope ****************
        // If we have a previous Sensors and our platform has errors then reuse our last sensor value of the
        // accelerometer
        sensors->accelerometer =
            subcontroller_packet_error ? previous_sensors->accelerometer : raw_sensors.accelerometer.cast<double>();
        sensors->gyroscope =
            subcontroller_packet_error ? previous_sensors->gyroscope : raw_sensors.gyroscope.cast<double>();

        // If we have a previous Sensors message AND (our platform has errors OR the gyro is spinning too fast) then
        // reuse our last sensor value of the gyroscope. Note: One of the gyros would occasionally
        // throw massive numbers without an error flag and if our hardware is working as intended, it should never
        // read that we're spinning at 2 revs/s
        if (raw_sensors.gyroscope.norm() > 4.0 * M_PI) {
            log<WARN>("Bad gyroscope value", raw_sensors.gyroscope.norm());
            sensors->gyroscope = previous_sensors ? previous_sensors->gyroscope : Eigen::Vector3d::Zero();
        }

        // **************** Timestamp ****************
        sensors->timestamp = raw_sensors.timestamp;

        // **************** Battery Voltage  ****************
        // Update the current battery voltage of the whole robot
        sensors->voltage = raw_sensors.battery;

        // **************** Buttons and LEDs ****************
        sensors->button.reserve(2);
        sensors->button.emplace_back(0, raw_sensors.buttons.left);
        sensors->button.emplace_back(1, raw_sensors.buttons.middle);
        sensors->led.reserve(5);
        sensors->led.emplace_back(0, raw_sensors.led_panel.led2 ? 0xFF0000 : 0);
        sensors->led.emplace_back(1, raw_sensors.led_panel.led3 ? 0xFF0000 : 0);
        sensors->led.emplace_back(2, raw_sensors.led_panel.led4 ? 0xFF0000 : 0);
        sensors->led.emplace_back(3, raw_sensors.head_led.RGB);  // Head
        sensors->led.emplace_back(4, raw_sensors.eye_led.RGB);   // Eye
    }
}  // namespace module::input
