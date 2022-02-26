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


namespace module::platform::darwin {

    using extension::Configuration;
    using message::motion::ServoTarget;
    using message::motion::ServoTargets;
    using message::platform::RawSensors;
    using utility::support::Expression;

    RawSensors HardwareIO::parseSensors(const Darwin::BulkReadResults& data) {
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

        // Head LED
        sensors.head_led = cm740State.headLED;

        // Eye LED
        sensors.eye_led = cm740State.eyeLED;

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
        : Reactor(std::move(environment)), darwin("/dev/CM740"), chargedVoltage(0.0f), flatVoltage(0.0f) {

        on<Startup>().then("HardwareIO Startup", [this] {
            auto CM740Model   = darwin.cm740.read<uint16_t>(Darwin::CM740::Address::MODEL_NUMBER_L);
            auto CM740Version = darwin.cm740.read<uint8_t>(Darwin::CM740::Address::VERSION);
            std::stringstream version;
            std::stringstream model;
            model << "0x" << std::setw(4) << std::setfill('0') << std::hex << int(CM740Model);
            version << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(CM740Version);
            log<NUClear::INFO>("CM740 Model:", model.str());
            log<NUClear::INFO>("CM740 Firmware Version:", version.str());
        });

        on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {
            // Set config for the packet waiting
            darwin.setConfig(config);

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
                                            Darwin::ID::BROADCAST,
                                            0x00,  // The size, fill this in later
                                            Darwin::DarwinDevice::Instruction::SYNC_WRITE,
                                            Darwin::MX28::Address::D_GAIN,
                                            0x0A};

            for (uint i = 0; i < servoState.size(); ++i) {

                if (servoState[i].dirty) {

                    // Clear our dirty flag
                    servoState[i].dirty = false;

                    // If our torque should be disabled then we disable our torque
                    if (servoState[i].torqueEnabled
                        && (std::isnan(servoState[i].goalPosition) || servoState[i].torque == 0)) {
                        servoState[i].torqueEnabled = false;
                        darwin[i + 1].write(Darwin::MX28::Address::TORQUE_ENABLE, false);
                    }
                    else {
                        // If our torque was disabled but is now enabled
                        if (!servoState[i].torqueEnabled && !std::isnan(servoState[i].goalPosition)
                            && servoState[i].torque != 0) {
                            servoState[i].torqueEnabled = true;
                            darwin[i + 1].write(Darwin::MX28::Address::TORQUE_ENABLE, true);
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
                command[Darwin::Packet::LENGTH] = command.size() - 3;

                // Do a checksum
                command.push_back(0);
                command.back() = Darwin::calculateChecksum(command.data());

                darwin.sendRawCommand(command);
            }

            // Read our data
            Darwin::BulkReadResults data = darwin.bulkRead();

            // Parse our data
            *sensors = parseSensors(data);

            // Work out a battery charged percentage
            sensors->voltage = std::max(0.0f, (sensors->voltage - flatVoltage) / (chargedVoltage - flatVoltage));

            // cm740 leds to display battery voltage
            uint32_t ledl            = 0;
            uint32_t ledr            = 0;
            std::array<bool, 3> ledp = {false, false, false};

            if (sensors->voltage > 0.9) {
                ledp = {true, true, true};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->voltage > 0.7) {
                ledp = {false, true, true};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->voltage > 0.5) {
                ledp = {false, false, true};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->voltage > 0.3) {
                ledp = {false, false, false};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
            }
            else if (sensors->voltage > 0.2) {
                ledp = {false, false, false};
                ledl = (uint8_t(0x00) << 16) | (uint8_t(0xFF) << 8) | uint8_t(0x00);
                ledr = (uint8_t(0xFF) << 16) | (uint8_t(0x00) << 8) | uint8_t(0x00);
            }
            else if (sensors->voltage > 0) {
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

        // If we get a HeadLED command then write it
        on<Trigger<RawSensors::HeadLED>>().then([this](const RawSensors::HeadLED& led) {
            // Update our internal state
            cm740State.headLED = led;

            darwin.cm740.write(Darwin::CM740::Address::LED_HEAD_L,
                               Convert::colourLEDInverse(static_cast<uint8_t>((led.RGB & 0x00FF0000) >> 24),
                                                         static_cast<uint8_t>((led.RGB & 0x0000FF00) >> 8),
                                                         static_cast<uint8_t>(led.RGB & 0x000000FF)));
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::EyeLED>>().then([this](const RawSensors::EyeLED& led) {
            // Update our internal state
            cm740State.eyeLED = led;

            darwin.cm740.write(Darwin::CM740::Address::LED_EYE_L,
                               Convert::colourLEDInverse(static_cast<uint8_t>((led.RGB & 0x00FF0000) >> 24),
                                                         static_cast<uint8_t>((led.RGB & 0x0000FF00) >> 8),
                                                         static_cast<uint8_t>(led.RGB & 0x000000FF)));
        });

        // If we get a EyeLED command then write it
        on<Trigger<RawSensors::LEDPanel>>().then([this](const RawSensors::LEDPanel& led) {
            // Update our internal state
            cm740State.ledPanel = led;

            darwin.cm740.write(Darwin::CM740::Address::LED_PANNEL,
                               ((uint8_t(led.led2) << 2) | (uint8_t(led.led3) << 1) | uint8_t((led.led4))));
        });
    }
}  // namespace module::platform::darwin
