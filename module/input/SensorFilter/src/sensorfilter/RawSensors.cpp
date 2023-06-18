/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#include "utility/platform/RawSensors.hpp"

#include "SensorFilter.hpp"

namespace module::input {

    using utility::platform::getRawServo;
    using utility::platform::make_error_string;
    using utility::platform::make_servo_error_string;

    using message::platform::ButtonLeftDown;
    using message::platform::ButtonLeftUp;
    using message::platform::ButtonMiddleDown;
    using message::platform::ButtonMiddleUp;

    void SensorFilter::update_raw_sensors(std::unique_ptr<Sensors>& sensors,
                                          const std::shared_ptr<const Sensors>& previous_sensors,
                                          const RawSensors& raw_sensors) {
        // Check for errors on the platform and FSRs
        if (raw_sensors.platform_error_flags != RawSensors::Error::OK) {
            NUClear::log<NUClear::WARN>(make_error_string("Platform", raw_sensors.platform_error_flags));
        }
        if (raw_sensors.fsr.left.error_flags != RawSensors::Error::OK) {
            NUClear::log<NUClear::WARN>(make_error_string("Left FSR", raw_sensors.fsr.left.error_flags));
        }
        if (raw_sensors.fsr.right.error_flags != RawSensors::Error::OK) {
            NUClear::log<NUClear::WARN>(make_error_string("Right FSR", raw_sensors.fsr.right.error_flags));
        }

        // **************** Servos ****************
        for (uint32_t id = 0; id < 20; ++id) {
            const auto& original = getRawServo(id, raw_sensors);
            const auto& error    = original.error_flags;
            // Check for an error on the servo and report it
            if (error != RawSensors::Error::OK) {
                NUClear::log<NUClear::WARN>(make_servo_error_string(original, id));
            }
            // If current Sensors message for this servo has an error and we have a previous sensors
            // message available, then we use our previous sensor values with some updates
            if (error != RawSensors::Error::OK && previous_sensors) {
                // Add the sensor values to the system properly
                sensors->servo.emplace_back(error,
                                            id,
                                            original.torque_enabled,
                                            original.position_p_gain,
                                            original.position_i_gain,
                                            original.position_d_gain,
                                            original.goal_position,
                                            original.profile_velocity,
                                            previous_sensors->servo[id].present_position,
                                            previous_sensors->servo[id].present_velocity,
                                            previous_sensors->servo[id].load,
                                            previous_sensors->servo[id].voltage,
                                            previous_sensors->servo[id].temperature);
            }
            // Otherwise we can just use the new values as is
            else {
                // Add the sensor values to the system properly
                sensors->servo.emplace_back(error,
                                            id,
                                            original.torque_enabled,
                                            original.position_p_gain,
                                            original.position_i_gain,
                                            original.position_d_gain,
                                            original.goal_position,
                                            original.profile_velocity,
                                            original.present_position,
                                            original.present_velocity,
                                            original.present_current,
                                            original.voltage,
                                            static_cast<float>(original.temperature));
            }

            // **************** Accelerometer and Gyroscope ****************
            // If we have a previous Sensors and our platform has errors then reuse our last sensor value of the
            // accelerometer
            if ((raw_sensors.platform_error_flags != 0u) && previous_sensors) {
                sensors->accelerometer = previous_sensors->accelerometer;
            }
            else {
                sensors->accelerometer = raw_sensors.accelerometer.cast<double>();
            }

            // If we have a previous Sensors message, our platform has errors, and the gyro is spinning too fast then
            // reuse our last sensor value of the gyroscope. Note: One of the gyros would occasionally
            // throw massive numbers without an error flag and if our hardware is working as intended, it should never
            // read that we're spinning at 2 revs/s
            if (previous_sensors
                && ((raw_sensors.platform_error_flags != 0u) || raw_sensors.gyroscope.norm() > 4.0 * M_PI)) {
                NUClear::log<NUClear::WARN>("Bad gyroscope value", raw_sensors.gyroscope.norm());
                sensors->gyroscope = previous_sensors->gyroscope;
            }
            else {
                sensors->gyroscope = raw_sensors.gyroscope.cast<double>();
            }
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

    void SensorFilter::detect_button_press(const std::list<std::shared_ptr<const RawSensors>>& sensors) {
        int left_count   = 0;
        int middle_count = 0;
        // If we have any downs in the last 20 frames then we are button pushed
        for (const auto& s : sensors) {
            if (s->buttons.left && (s->platform_error_flags == 0u)) {
                ++left_count;
            }
            if (s->buttons.middle && (s->platform_error_flags == 0u)) {
                ++middle_count;
            }
        }
        bool new_left_down   = left_count > cfg.buttons.debounce_threshold;
        bool new_middle_down = middle_count > cfg.buttons.debounce_threshold;
        if (new_left_down != left_down) {
            left_down = new_left_down;
            if (new_left_down) {
                log<NUClear::INFO>("Left Button Down");
                emit(std::make_unique<ButtonLeftDown>());
            }
            else {
                log<NUClear::INFO>("Left Button Up");
                emit(std::make_unique<ButtonLeftUp>());
            }
        }
        if (new_middle_down != middle_down) {
            middle_down = new_middle_down;
            if (new_middle_down) {
                log<NUClear::INFO>("Middle Button Down");
                emit(std::make_unique<ButtonMiddleDown>());
            }
            else {
                log<NUClear::INFO>("Middle Button Up");
                emit(std::make_unique<ButtonMiddleUp>());
            }
        }
    }

}  // namespace module::input
