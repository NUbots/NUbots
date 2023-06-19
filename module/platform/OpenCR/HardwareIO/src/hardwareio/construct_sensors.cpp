#include "HardwareIO.hpp"

#include "utility/math/angle.hpp"

namespace module::platform::OpenCR {

    using message::platform::RawSensors;

    RawSensors HardwareIO::construct_sensors() {
        RawSensors sensors;

        // Timestamp when this message was created (data itsself could be old)
        sensors.timestamp = NUClear::clock::now();

        /* OpenCR data */
        sensors.subcontroller_error = opencr_state.error_flags;
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
            RawSensors::Servo& servo = utility::platform::getRawServo(i, sensors);


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
                auto present = utility::math::angle::normalizeAngle(servo_states[i].present_position - offset);
                auto goal    = utility::math::angle::normalizeAngle(servo_states[i].goal_position - offset);

                // We have reached our destination
                if (std::abs(present - goal) < moving_speed) {
                    servo_states[i].present_position = servo_states[i].goal_position;
                    servo_states[i].present_velocity = 0;
                }
                // We have to move towards our destination at moving speed
                else {
                    servo_states[i].present_position = utility::math::angle::normalizeAngle(
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
                servo.error_flags = servo_states[i].hardware_error;

                // spoof error
                log<NUClear::DEBUG>("Spoofing all errors on all servos");
                servo.error_flags |= RawSensors::HardwareError::INPUT_VOLTAGE;
                servo.error_flags |= RawSensors::HardwareError::OVERHEATING;
                servo.error_flags |= RawSensors::HardwareError::MOTOR_ENCODER;
                servo.error_flags |= RawSensors::HardwareError::ELECTRICAL_SHOCK;
                servo.error_flags |= RawSensors::HardwareError::OVERLOAD;

                // Accumulate all packet error flags to read at once
                sensors.subcontroller_error |= servo_states[i].packet_error;

                // Present Data
                servo.present_position = servo_states[i].present_position;
                servo.present_velocity = servo_states[i].present_velocity;

                // Diagnostic Information
                servo.voltage     = servo_states[i].voltage;
                servo.temperature = servo_states[i].temperature;

                /* Note: removed input voltage bit clear here as it wasn't clear how it fits into the refactor. If there
                 * are problems with input voltage errors coming up too frequently then this is likely the culprit */
            }
        }

        // handle unused compatibility fields
        sensors.platform_error_flags  = RawSensors::Error::_OK;
        sensors.fsr.left.error_flags  = RawSensors::Error::_OK;
        sensors.fsr.right.error_flags = RawSensors::Error::_OK;

        return sensors;
    }

}  // namespace module::platform::OpenCR
