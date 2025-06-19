/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "HardwareIO.hpp"

#include "utility/math/angle.hpp"

namespace module::platform::OpenCR {

    using message::platform::RawSensors;

    RawSensors HardwareIO::construct_sensors() {
        RawSensors sensors;

        // Timestamp when this message was created (data itsself could be old)
        sensors.timestamp = NUClear::clock::now();

        /* OpenCR data */
        sensors.subcontroller_error = opencr_state.packet_error;
        sensors.led_panel           = opencr_state.led_panel;
        sensors.head_led            = opencr_state.head_led;
        sensors.eye_led             = opencr_state.eye_led;
        sensors.buttons             = opencr_state.buttons;
        sensors.accelerometer       = opencr_state.acc;
        sensors.gyroscope           = opencr_state.gyro;

        /* Battery data */
        sensors.battery = battery_state.current_voltage;

        /* Servos data */
        for (int i = 0; i < 20; i++) {
            // Get a reference to the servo we are populating
            RawSensors::Servo& servo = utility::platform::get_raw_servo(i, sensors);


            // Booleans
            servo.torque_enabled = servo_states[i].torque_enabled;

            // Gain
            servo.position_p_gain = servo_states[i].position_p_gain;
            servo.position_i_gain = servo_states[i].position_i_gain;
            servo.position_d_gain = servo_states[i].position_d_gain;

            // Targets
            servo.goal_position    = servo_states[i].goal_position;
            servo.profile_velocity = servo_states[i].profile_velocity;


            // If we are faking this hardware, simulate its motion
            if (servo_states[i].simulated) {
                // Work out how fast we should be moving
                // 5.236 == 50 rpm which is similar to the max speed of the servos
                float moving_speed = (servo_states[i].profile_velocity == 0 ? 5.236 : servo_states[i].profile_velocity)
                                     / UPDATE_FREQUENCY;

                // Get our offset for this servo and apply it
                // The values are now between -pi and pi around the servos axis
                auto offset  = nugus.servo_offset[i];
                auto present = utility::math::angle::normalise_angle(servo_states[i].present_position - offset);
                auto goal    = utility::math::angle::normalise_angle(servo_states[i].goal_position - offset);

                // We have reached our destination
                if (std::abs(present - goal) < moving_speed) {
                    servo_states[i].present_position = servo_states[i].goal_position;
                    servo_states[i].present_velocity = 0;
                }
                // We have to move towards our destination at moving speed
                else {
                    servo_states[i].present_position = utility::math::angle::normalise_angle(
                        (present + moving_speed * (goal > present ? 1 : -1)) + offset);
                    servo_states[i].present_velocity = moving_speed;
                }

                // Store our simulated values
                servo.present_position = servo_states[i].present_position;
                servo.present_velocity = servo_states[i].present_velocity;
                servo.voltage          = servo_states[i].voltage;
                servo.temperature      = servo_states[i].temperature;
            }

            // If we are using real data, get it from the packet
            else {
                // Error code
                servo.hardware_error = servo_states[i].hardware_error;

                // Present Data
                servo.present_position = servo_states[i].present_position;
                servo.present_velocity = servo_states[i].present_velocity;

                // Diagnostic Information
                servo.voltage     = servo_states[i].voltage;
                servo.temperature = servo_states[i].temperature;

                // Clear Overvoltage flag if voltage is below the (normal) max battery voltage, as this isn't dangerous
                if (servo.voltage <= battery_state.charged_voltage) {
                    servo.hardware_error &= ~RawSensors::HardwareError::INPUT_VOLTAGE;
                }
            }
        }

        return sensors;
    }

}  // namespace module::platform::OpenCR
