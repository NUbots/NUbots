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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "HardwareIO.hpp"

#include <iomanip>

#include "Convert.hpp"

#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/angle.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::platform::cm740 {

    using extension::Configuration;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::platform::RawSensors;
    using utility::support::Expression;

    RawSensors HardwareIO::parseSensors(const CM740::BulkReadResults& data) {
        RawSensors sensors;

        // Timestamp when our data was taken
        sensors.timestamp = NUClear::clock::now();

        /*
         CM740 Data
         */

        // Read our Error code
        sensors.platform_error_flags =
            data.cm740ErrorCode == 0xFF ? RawSensors::Error::TIMEOUT_ : RawSensors::Error(data.cm740ErrorCode).value;

        // LED Panel
        sensors.led_panel = led_state.led_panel;

        // Head LED
        sensors.head_led = led_state.head_LED;

        // Eye LED
        sensors.eye_led = led_state.eye_LED;

        // Buttons
        sensors.buttons.left   = Convert::getBit<0>(data.cm740.buttons);
        sensors.buttons.middle = Convert::getBit<1>(data.cm740.buttons);

        // Voltage (in volts)
        sensors.battery = Convert::voltage(data.cm740.voltage);

        if (sensors.battery <= cfg.battery.charged_voltage) {
            sensors.platform_error_flags &= ~RawSensors::Error::INPUT_VOLTAGE_;
        }

        // Accelerometer (in m/s^2)
        // Swizzle axes to that
        //      x axis reports a +1g acceleration when robot is laying on its back
        //      y axis reports a +1g acceleration when robot is laying on its right side
        //      z axis reports a +1g acceleration when robot is vertical
        // The CM740 currently has
        //      x is backward, y is to the left, and z is up
        sensors.accelerometer.x() = Convert::accelerometer(data.cm740.accelerometer.x);
        sensors.accelerometer.y() = -Convert::accelerometer(data.cm740.accelerometer.y);
        sensors.accelerometer.z() = -Convert::accelerometer(data.cm740.accelerometer.z);

        // Gyroscope (in radians/second)
        // Swizzle axes to that
        //      x is forward, y is to the left, and z is up
        // The CM740 currently has
        //      x is to the right, y is backward, and z is down
        sensors.gyroscope.x() = Convert::gyroscope(data.cm740.gyroscope.y);
        sensors.gyroscope.y() = Convert::gyroscope(data.cm740.gyroscope.x);
        sensors.gyroscope.z() = -Convert::gyroscope(data.cm740.gyroscope.z);

        /*
         Force Sensitive Resistor Data
         */

        // Right Sensor
        // Error
        sensors.fsr.right.error_flags =
            data.fsrErrorCodes[0] == 0xFF ? RawSensors::Error::TIMEOUT_ : RawSensors::Error(data.fsrErrorCodes[0]).value;

        // Sensors
        sensors.fsr.right.fsr1 = Convert::fsrForce(data.fsr[0].fsr1);
        sensors.fsr.right.fsr2 = Convert::fsrForce(data.fsr[0].fsr2);
        sensors.fsr.right.fsr3 = Convert::fsrForce(data.fsr[0].fsr3);
        sensors.fsr.right.fsr4 = Convert::fsrForce(data.fsr[0].fsr4);

        // Centre, swaps X and Y coords to robot
        // see
        // http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
        sensors.fsr.right.centre_x = Convert::fsrCentre(false, data.fsr[0].centreY);
        sensors.fsr.right.centre_y = Convert::fsrCentre(false, data.fsr[0].centreX);

        // Left Sensor
        // Error
        sensors.fsr.left.error_flags =
            data.fsrErrorCodes[1] == 0xFF ? RawSensors::Error::TIMEOUT_ : RawSensors::Error(data.fsrErrorCodes[1]).value;

        // Sensors
        sensors.fsr.left.fsr1 = Convert::fsrForce(data.fsr[1].fsr1);
        sensors.fsr.left.fsr2 = Convert::fsrForce(data.fsr[1].fsr2);
        sensors.fsr.left.fsr3 = Convert::fsrForce(data.fsr[1].fsr3);
        sensors.fsr.left.fsr4 = Convert::fsrForce(data.fsr[1].fsr4);

        // Centre, swaps X and Y coords to robot
        // see
        // http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
        sensors.fsr.left.centre_x = Convert::fsrCentre(true, data.fsr[1].centreY);
        sensors.fsr.left.centre_y = Convert::fsrCentre(true, data.fsr[1].centreX);

        /*
         Servos
         */

        for (int i = 0; i < 20; ++i) {
            // Get a reference to the servo we are populating
            RawSensors::Servo& servo = utility::platform::getRawServo(i, sensors);


            // Booleans
            servo.torque_enabled = servo_state[i].torque_enabled;

            // Gain
            servo.position_p_gain = servo_state[i].p_gain;
            servo.position_i_gain = servo_state[i].i_gain;
            servo.position_d_gain = servo_state[i].d_gain;

            // Targets
            servo.goal_position    = servo_state[i].goal_position;
            servo.profile_velocity = servo_state[i].moving_speed;

            // If we are faking this hardware, simulate its motion
            if (servo_state[i].simulated) {
                // Work out how fast we should be moving
                // 5.236 == 50 rpm which is similar to the max speed of the servos
                float moving_speed =
                    (servo_state[i].moving_speed == 0 ? 5.236 : servo_state[i].moving_speed) / UPDATE_FREQUENCY;

                // Get our offset for this servo and apply it
                // The values are now between -pi and pi around the servos axis
                auto offset  = Convert::SERVO_OFFSET[i];
                auto present = utility::math::angle::normalizeAngle(servo_state[i].present_position - offset);
                auto goal    = utility::math::angle::normalizeAngle(servo_state[i].goal_position - offset);

                // We have reached our destination
                if (std::abs(present - goal) < moving_speed) {
                    servo_state[i].present_position = servo_state[i].goal_position;
                    servo_state[i].present_speed    = 0;
                }
                // We have to move towards our destination at moving speed
                else {
                    servo_state[i].present_position = utility::math::angle::normalizeAngle(
                        (present + moving_speed * (goal > present ? 1 : -1)) + offset);
                    servo_state[i].present_speed = servo_state[i].moving_speed;
                }

                // Store our simulated values
                servo.present_position = servo_state[i].present_position;
                servo.present_velocity = servo_state[i].present_speed;
                servo.voltage          = servo_state[i].voltage;
                servo.temperature      = servo_state[i].temperature;
            }

            // If we are using real data, get it from the packet
            else {
                // Error code
                servo.error_flags = data.servoErrorCodes[i] == 0xFF ? RawSensors::Error::TIMEOUT_
                                                                    : RawSensors::Error(data.servoErrorCodes[i]).value;

                // Present Data
                servo.present_position = Convert::servoPosition(i, data.servos[i].presentPosition);
                servo.present_velocity = Convert::servoSpeed(i, data.servos[i].presentSpeed);

                // Diagnostic Information
                servo.voltage     = Convert::voltage(data.servos[i].voltage);
                servo.temperature = Convert::temperature(data.servos[i].temperature);

                // Clear Overvoltage flag if current voltage is greater than maximum expected voltage
                if (servo.voltage <= cfg.battery.charged_voltage) {
                    servo.error_flags &= ~RawSensors::Error::INPUT_VOLTAGE_;
                }
            }
        }
        return sensors;
    }

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Startup>().then("HardwareIO Startup", [this] {
            auto CM740Model   = cm740.cm740.read<uint16_t>(CM740::CM740Data::Address::MODEL_NUMBER_L);
            auto CM740Version = cm740.cm740.read<uint8_t>(CM740::CM740Data::Address::VERSION);
            std::stringstream version;
            std::stringstream model;
            model << "0x" << std::setw(4) << std::setfill('0') << std::hex << int(CM740Model);
            version << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(CM740Version);
            log<NUClear::INFO>("CM740 Model:", model.str());
            log<NUClear::INFO>("CM740 Firmware Version:", version.str());
        });

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Set config for the packet waiting
            cm740.setConfig(config);

            for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                Convert::SERVO_OFFSET[i]    = config["servos"][i]["offset"].as<Expression>();
                Convert::SERVO_DIRECTION[i] = config["servos"][i]["direction"].as<Expression>();
                servo_state[i].simulated    = config["servos"][i]["simulated"].as<bool>();
            }

            cfg.battery.charged_voltage = config["battery"]["charged_voltage"].as<float>();
            cfg.battery.nominal_voltage = config["battery"]["nominal_voltage"].as<float>();
            cfg.battery.flat_voltage    = config["battery"]["flat_voltage"].as<float>();
        });

        // This trigger gets the sensor data from the CM740
        on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then("Hardware Loop", [this] {
            // Our final sensor output
            auto sensors = std::make_unique<RawSensors>();

            std::vector<uint8_t> command = {0xFF,
                                            0xFF,
                                            CM740::ID::BROADCAST,
                                            0x00,  // The size, fill this in later
                                            CM740::CM740Interface::Instruction::SYNC_WRITE,
                                            CM740::Servo::Address::D_GAIN,
                                            0x0A};

            for (uint i = 0; i < servo_state.size(); ++i) {

                if (servo_state[i].dirty) {

                    // Clear our dirty flag
                    servo_state[i].dirty = false;

                    // If our torque should be disabled then we disable our torque
                    if (servo_state[i].torque_enabled
                        && (std::isnan(servo_state[i].goal_position) || servo_state[i].torque == 0)) {
                        servo_state[i].torque_enabled = false;
                        cm740[i + 1].write(CM740::Servo::Address::TORQUE_ENABLE, false);
                    }
                    else {
                        // If our torque was disabled but is now enabled
                        if (!servo_state[i].torque_enabled && !std::isnan(servo_state[i].goal_position)
                            && servo_state[i].torque != 0) {
                            servo_state[i].torque_enabled = true;
                            cm740[i + 1].write(CM740::Servo::Address::TORQUE_ENABLE, true);
                        }

                        // Get our goal position and speed
                        uint16_t goal_position = Convert::servoPositionInverse(i, servo_state[i].goal_position);
                        uint16_t moving_speed  = Convert::servoSpeedInverse(servo_state[i].moving_speed);
                        uint16_t torque        = Convert::torqueLimitInverse(servo_state[i].torque);

                        // Add to our sync write command
                        command.insert(command.end(),
                                       {
                                           uint8_t(i + 1),
                                           Convert::gainInverse(servo_state[i].d_gain),  // D Gain
                                           Convert::gainInverse(servo_state[i].i_gain),  // I Gain
                                           Convert::gainInverse(servo_state[i].p_gain),  // P Gain
                                           0,                                            // Reserved
                                           uint8_t(0xFF & goal_position),                // Goal Position L
                                           uint8_t(0xFF & (goal_position >> 8)),         // Goal Position H
                                           uint8_t(0xFF & moving_speed),                 // Goal Speed L
                                           uint8_t(0xFF & (moving_speed >> 8)),          // Goal Speed H
                                           uint8_t(0xFF & torque),                       // Torque Limit L
                                           uint8_t(0xFF & (torque >> 8))                 // Torque Limit H
                                       });
                    }
                }
            }

            // Write our data (if we need to)
            if (command.size() > 7) {
                // Calculate our length
                command[CM740::Packet::LENGTH] = command.size() - 3;

                // Do a checksum
                command.push_back(0);
                command.back() = CM740::calculateChecksum(command.data());

                cm740.sendRawCommand(command);
            }

            // Read our data
            CM740::BulkReadResults data = cm740.bulkRead();

            // Parse our data
            *sensors = parseSensors(data);

            // Work out a battery charged percentage
            sensors->battery = std::max(0.0f,
                                        (sensors->battery - cfg.battery.flat_voltage)
                                            / (cfg.battery.charged_voltage - cfg.battery.flat_voltage));

            // cm740 leds to display battery voltage
            uint32_t ledl            = 0;
            uint32_t ledr            = 0;
            std::array<bool, 3> ledp = {false, false, false};

            if (sensors->battery > 0.9) {
                ledp = {true, true, true};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->battery > 0.7) {
                ledp = {false, true, true};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->battery > 0.5) {
                ledp = {false, false, true};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->battery > 0.3) {
                ledp = {false, false, false};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->battery > 0.2) {
                ledp = {false, false, false};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            else if (sensors->battery > 0) {
                ledp = {false, false, false};
                ledl = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            // Error in reading voltage blue
            else {
                ledp = {false, false, false};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0x00) << 8) | uint8_t(0xFF);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0x00) << 8) | uint8_t(0xFF);
            }
            emit(std::make_unique<RawSensors::LEDPanel>(ledp[2], ledp[1], ledp[0]));
            emit(std::make_unique<RawSensors::EyeLED>(ledl));
            emit(std::make_unique<RawSensors::HeadLED>(ledr));

            // Send our nicely computed sensor data out to the world
            emit(std::move(sensors));
        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<ServoTargets>, With<RawSensors>>().then([this](const ServoTargets& commands,
                                                                  const RawSensors& sensors) {
            // Loop through each of our commands
            for (const auto& command : commands.targets) {
                float diff = utility::math::angle::difference(
                    command.position,
                    utility::platform::getRawServo(command.id, sensors).present_position);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed = 0.0f;
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }
                else {
                    speed = 0;
                }

                // Update our internal state
                if (servo_state[command.id].p_gain != command.gain || servo_state[command.id].i_gain != command.gain * 0
                    || servo_state[command.id].d_gain != command.gain * 0
                    || servo_state[command.id].moving_speed != speed
                    || servo_state[command.id].goal_position != command.position
                    || servo_state[command.id].torque != command.torque) {

                    servo_state[command.id].dirty = true;

                    servo_state[command.id].p_gain        = command.gain;
                    servo_state[command.id].i_gain        = command.gain * 0;
                    servo_state[command.id].d_gain        = command.gain * 0;
                    servo_state[command.id].moving_speed  = speed;
                    servo_state[command.id].goal_position = command.position;

                    servo_state[command.id].torque       = command.torque;
                    servo_state[uint(command.id)].torque = command.torque;
                }
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto command_list = std::make_unique<ServoTargets>();
            command_list->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(command_list);
        });

        // If we get a HeadLED command then write it
        on<Trigger<RawSensors::HeadLED>>().then([this](const RawSensors::HeadLED& led) {
            // Update our internal state
            led_state.head_LED = led;

            cm740.cm740.write(CM740::CM740Data::Address::LED_HEAD_L,
                              Convert::colourLEDInverse(static_cast<uint8_t>((led.RGB & 0x00FF0000) >> 24),
                                                        static_cast<uint8_t>((led.RGB & 0x0000FF00) >> 8),
                                                        static_cast<uint8_t>(led.RGB & 0x000000FF)));
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::EyeLED>>().then([this](const RawSensors::EyeLED& led) {
            // Update our internal state
            led_state.eye_LED = led;

            cm740.cm740.write(CM740::CM740Data::Address::LED_EYE_L,
                              Convert::colourLEDInverse(static_cast<uint8_t>((led.RGB & 0x00FF0000) >> 24),
                                                        static_cast<uint8_t>((led.RGB & 0x0000FF00) >> 8),
                                                        static_cast<uint8_t>(led.RGB & 0x000000FF)));
        });

        // If we get a LEDPanel command then write it
        on<Trigger<RawSensors::LEDPanel>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            led_state.led_panel = led;

            cm740.cm740.write(CM740::CM740Data::Address::LED_PANNEL,
                              ((uint8_t(led.led2) << 2) | (uint8_t(led.led3) << 1) | uint8_t((led.led4))));
        });
    }
}  // namespace module::platform::cm740
