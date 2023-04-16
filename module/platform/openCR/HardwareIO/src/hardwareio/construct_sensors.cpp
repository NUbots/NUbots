#include "HardwareIO.hpp"

#include "utility/math/angle.hpp"

namespace module::platform::openCR {

    using message::platform::RawSensors;

    RawSensors HardwareIO::construct_sensors() {
        RawSensors sensors;

        // Timestamp when this message was created (data itsself could be old)
        sensors.timestamp = NUClear::clock::now();

        /* OpenCR data */
        sensors.platform_error_flags = RawSensors::Error::OK;
        /// @todo Add proper error handling to translate new errors into rawsensors errors, using
        /// opencrState.errorFlags.errorNumber and opencrState.errorFlags.alertFlag
        sensors.led_panel     = opencrState.ledPanel;
        sensors.head_led      = opencrState.headLED;
        sensors.eye_led       = opencrState.eyeLED;
        sensors.buttons       = opencrState.buttons;
        sensors.accelerometer = opencrState.acc;
        sensors.gyroscope     = opencrState.gyro;

        /* Battery data */
        sensors.battery = batteryState.currentVoltage;

        /* Servos data */
        for (int i = 0; i < 20; i++) {
            // Get a reference to the servo we are populating
            RawSensors::Servo& servo = utility::platform::getRawServo(i, sensors);


            // Booleans
            servo.torque_enabled = servoStates[i].torqueEnabled;

            // Gain
            servo.velocity_p_gain = servoStates[i].velocityPGain;
            servo.velocity_i_gain = servoStates[i].velocityIGain;
            servo.velocity_d_gain = servoStates[i].velocityDGain;

            // Targets
            servo.goal_position    = servoStates[i].goalPosition;
            servo.profile_velocity = servoStates[i].profileVelocity;


            // If we are faking this hardware, simulate its motion
            if (servoStates[i].simulated) {
                // Work out how fast we should be moving
                // 5.236 == 50 rpm which is similar to the max speed of the servos
                float movingSpeed =
                    (servoStates[i].profileVelocity == 0 ? 5.236 : servoStates[i].profileVelocity) / UPDATE_FREQUENCY;

                // Get our offset for this servo and apply it
                // The values are now between -pi and pi around the servos axis
                auto offset  = nugus.servo_offset[i];
                auto present = utility::math::angle::normalizeAngle(servoStates[i].presentPosition - offset);
                auto goal    = utility::math::angle::normalizeAngle(servoStates[i].goalPosition - offset);

                // We have reached our destination
                if (std::abs(present - goal) < movingSpeed) {
                    servoStates[i].presentPosition = servoStates[i].goalPosition;
                    servoStates[i].presentVelocity = 0;
                }
                // We have to move towards our destination at moving speed
                else {
                    servoStates[i].presentPosition = utility::math::angle::normalizeAngle(
                        (present + movingSpeed * (goal > present ? 1 : -1)) + offset);
                    servoStates[i].presentVelocity = movingSpeed;
                }

                // Store our simulated values
                servo.present_position = servoStates[i].presentPosition;
                servo.present_velocity = servoStates[i].presentVelocity;
                servo.voltage          = servoStates[i].voltage;
                servo.temperature      = servoStates[i].temperature;
            }

            // If we are using real data, get it from the packet
            else {
                // Error code
                servo.error_flags = servoStates[i].errorFlags;

                // Present Data
                servo.present_position = servoStates[i].presentPosition;
                servo.present_velocity = servoStates[i].presentVelocity;

                // Diagnostic Information
                servo.voltage     = servoStates[i].voltage;
                servo.temperature = servoStates[i].temperature;

                // Clear Overvoltage flag if current voltage is greater than maximum expected voltage
                if (servo.voltage <= batteryState.chargedVoltage) {
                    servo.error_flags &= ~RawSensors::Error::INPUT_VOLTAGE;
                }
            }
        }

        /* FSRs data */

        return sensors;
    }

}  // namespace module::platform::openCR
