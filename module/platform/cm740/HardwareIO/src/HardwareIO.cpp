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

#include "message/motion/ServoTarget.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/angle.hpp"
#include "utility/platform/RawSensors.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::platform::cm740 {

    using extension::Configuration;
    using message::motion::ServoTarget;
    using message::motion::ServoTargets;
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
            data.cm740ErrorCode == 0xFF ? RawSensors::Error::TIMEOUT : RawSensors::Error(data.cm740ErrorCode).value;

        // LED Panel
        sensors.led_panel = cm740State.ledPanel;

        // Buttons
        sensors.buttons.left   = Convert::getBit<0>(data.cm740.buttons);
        sensors.buttons.middle = Convert::getBit<1>(data.cm740.buttons);

        // Voltage (in volts)
        sensors.voltage = Convert::voltage(data.cm740.voltage);

        if (sensors.voltage <= chargedVoltage) {
            sensors.platform_error_flags &= ~RawSensors::Error::INPUT_VOLTAGE;
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
            data.fsrErrorCodes[0] == 0xFF ? RawSensors::Error::TIMEOUT : RawSensors::Error(data.fsrErrorCodes[0]).value;

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
            data.fsrErrorCodes[1] == 0xFF ? RawSensors::Error::TIMEOUT : RawSensors::Error(data.fsrErrorCodes[1]).value;

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
            servo.torque_enabled = servoState[i].torqueEnabled;

            // Gain
            servo.p_gain = servoState[i].pGain;
            servo.i_gain = servoState[i].iGain;
            servo.d_gain = servoState[i].dGain;

            // Torque
            servo.torque = servoState[i].torque;

            // Targets
            servo.goal_position = servoState[i].goalPosition;
            servo.moving_speed  = servoState[i].movingSpeed;

            // If we are faking this hardware, simulate its motion
            if (servoState[i].simulated) {
                // Work out how fast we should be moving
                // 5.236 == 50 rpm which is similar to the max speed of the servos
                float movingSpeed =
                    (servoState[i].movingSpeed == 0 ? 5.236 : servoState[i].movingSpeed) / UPDATE_FREQUENCY;

                // Get our offset for this servo and apply it
                // The values are now between -pi and pi around the servos axis
                auto offset  = Convert::SERVO_OFFSET[i];
                auto present = utility::math::angle::normalizeAngle(servoState[i].presentPosition - offset);
                auto goal    = utility::math::angle::normalizeAngle(servoState[i].goalPosition - offset);

                // We have reached our destination
                if (std::abs(present - goal) < movingSpeed) {
                    servoState[i].presentPosition = servoState[i].goalPosition;
                    servoState[i].presentSpeed    = 0;
                }
                // We have to move towards our destination at moving speed
                else {
                    servoState[i].presentPosition = utility::math::angle::normalizeAngle(
                        (present + movingSpeed * (goal > present ? 1 : -1)) + offset);
                    servoState[i].presentSpeed = servoState[i].movingSpeed;
                }

                // Store our simulated values
                servo.present_position = servoState[i].presentPosition;
                servo.present_speed    = servoState[i].goalPosition;
                servo.load             = servoState[i].load;
                servo.voltage          = servoState[i].voltage;
                servo.temperature      = servoState[i].temperature;
            }

            // If we are using real data, get it from the packet
            else {
                // Error code
                servo.error_flags = data.servoErrorCodes[i] == 0xFF ? RawSensors::Error::TIMEOUT
                                                                    : RawSensors::Error(data.servoErrorCodes[i]).value;

                // Present Data
                servo.present_position = Convert::servoPosition(i, data.servos[i].presentPosition);
                servo.present_speed    = Convert::servoSpeed(i, data.servos[i].presentSpeed);
                servo.load             = Convert::servoLoad(i, data.servos[i].load);

                // Diagnostic Information
                servo.voltage     = Convert::voltage(data.servos[i].voltage);
                servo.temperature = Convert::temperature(data.servos[i].temperature);

                // Clear Overvoltage flag if current voltage is greater than maximum expected voltage
                if (servo.voltage <= chargedVoltage) {
                    servo.error_flags &= ~RawSensors::Error::INPUT_VOLTAGE;
                }
            }
        }
        return sensors;
    }

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), cm740("/dev/CM740"), chargedVoltage(0.0f), flatVoltage(0.0f) {

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
                servoState[i].simulated     = config["servos"][i]["simulated"].as<bool>();
            }

            chargedVoltage = config["battery"]["charged_voltage"].as<float>();
            flatVoltage    = config["battery"]["flat_voltage"].as<float>();
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

            for (uint i = 0; i < servoState.size(); ++i) {

                if (servoState[i].dirty) {

                    // Clear our dirty flag
                    servoState[i].dirty = false;

                    // If our torque should be disabled then we disable our torque
                    if (servoState[i].torqueEnabled
                        && (std::isnan(servoState[i].goalPosition) || servoState[i].torque == 0)) {
                        servoState[i].torqueEnabled = false;
                        cm740[i + 1].write(CM740::Servo::Address::TORQUE_ENABLE, false);
                    }
                    else {
                        // If our torque was disabled but is now enabled
                        if (!servoState[i].torqueEnabled && !std::isnan(servoState[i].goalPosition)
                            && servoState[i].torque != 0) {
                            servoState[i].torqueEnabled = true;
                            cm740[i + 1].write(CM740::Servo::Address::TORQUE_ENABLE, true);
                        }

                        // Get our goal position and speed
                        uint16_t goal_position = Convert::servoPositionInverse(i, servoState[i].goalPosition);
                        uint16_t movingSpeed   = Convert::servoSpeedInverse(servoState[i].movingSpeed);
                        uint16_t torque        = Convert::torqueLimitInverse(servoState[i].torque);

                        // Add to our sync write command
                        command.insert(command.end(),
                                       {
                                           uint8_t(i + 1),
                                           Convert::gainInverse(servoState[i].dGain),  // D Gain
                                           Convert::gainInverse(servoState[i].iGain),  // I Gain
                                           Convert::gainInverse(servoState[i].pGain),  // P Gain
                                           0,                                          // Reserved
                                           uint8_t(0xFF & goal_position),              // Goal Position L
                                           uint8_t(0xFF & (goal_position >> 8)),       // Goal Position H
                                           uint8_t(0xFF & movingSpeed),                // Goal Speed L
                                           uint8_t(0xFF & (movingSpeed >> 8)),         // Goal Speed H
                                           uint8_t(0xFF & torque),                     // Torque Limit L
                                           uint8_t(0xFF & (torque >> 8))               // Torque Limit H
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
            sensors->voltage = std::max(0.0f, (sensors->voltage - flatVoltage) / (chargedVoltage - flatVoltage));

            // cm740 leds to display battery voltage
            std::array<bool, 3> ledp = {false, false, false};

            if (sensors->voltage > 0.9) {
                ledp = {true, true, true};
            }
            else if (sensors->voltage > 0.7) {
                ledp = {false, true, true};
            }
            else if (sensors->voltage > 0.5) {
                ledp = {false, false, true};
            }
            // Error in reading voltage blue
            else {
                ledp = {false, false, false};
            }
            emit(std::make_unique<RawSensors::LEDPanel>(ledp[0]));

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
                if (servoState[command.id].pGain != command.gain || servoState[command.id].iGain != command.gain * 0
                    || servoState[command.id].dGain != command.gain * 0 || servoState[command.id].movingSpeed != speed
                    || servoState[command.id].goalPosition != command.position
                    || servoState[command.id].torque != command.torque) {

                    servoState[command.id].dirty = true;

                    servoState[command.id].pGain        = command.gain;
                    servoState[command.id].iGain        = command.gain * 0;
                    servoState[command.id].dGain        = command.gain * 0;
                    servoState[command.id].movingSpeed  = speed;
                    servoState[command.id].goalPosition = command.position;

                    servoState[command.id].torque       = command.torque;
                    servoState[uint(command.id)].torque = command.torque;
                }
            }
        });

        on<Trigger<ServoTarget>>().then([this](const ServoTarget& command) {
            auto commandList = std::make_unique<ServoTargets>();
            commandList->targets.push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(commandList);
        });

        // If we get a LEDPanel command then write it
        on<Trigger<RawSensors::LEDPanel>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            cm740State.ledPanel = led;

            cm740.cm740.write(CM740::CM740Data::Address::LED_PANNEL,
                              ((uint8_t(led.led2) << 2) | (uint8_t(led.led3) << 1) | uint8_t((led.led4))));
        });
    }
}  // namespace module::platform::cm740
