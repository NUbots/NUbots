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

#include "HardwareIO.h"
#include "Convert.h"

#include <iomanip>

#include "extension/Configuration.h"

#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/math/angle.h"
#include "utility/platform/darwin/DarwinSensors.h"
#include "utility/support/yaml_expression.h"


namespace module {
namespace platform {
    namespace darwin {

        using extension::Configuration;
        using message::motion::ServoTarget;
        using message::platform::darwin::DarwinSensors;
        using utility::support::Expression;

        DarwinSensors HardwareIO::parseSensors(const Darwin::BulkReadResults& data) {
            DarwinSensors sensors;

            // Timestamp when our data was taken
            sensors.timestamp = NUClear::clock::now();

            /*
             CM730 Data
             */

            // Read our Error code
            sensors.cm730ErrorFlags = data.cm730ErrorCode == 0xFF ? DarwinSensors::Error::TIMEOUT
                                                                  : DarwinSensors::Error(data.cm730ErrorCode).value;

            // LED Panel
            sensors.ledPanel = cm730State.ledPanel;

            // Head LED
            sensors.headLED = cm730State.headLED;

            // Eye LED
            sensors.eyeLED = cm730State.eyeLED;

            // Buttons
            sensors.buttons.left   = Convert::getBit<0>(data.cm730.buttons);
            sensors.buttons.middle = Convert::getBit<1>(data.cm730.buttons);

            // Voltage (in volts)
            sensors.voltage = Convert::voltage(data.cm730.voltage);

            if (sensors.voltage <= maxVoltage) {
                sensors.cm730ErrorFlags &= ~DarwinSensors::Error::INPUT_VOLTAGE;
            }

            // Accelerometer (in m/s^2)
            sensors.accelerometer.x = Convert::accelerometer(data.cm730.accelerometer.x);
            sensors.accelerometer.y = Convert::accelerometer(data.cm730.accelerometer.y);
            sensors.accelerometer.z = Convert::accelerometer(data.cm730.accelerometer.z);

            // Gyroscope (in radians/second)
            sensors.gyroscope.x = Convert::gyroscope(data.cm730.gyroscope.x);
            sensors.gyroscope.y = Convert::gyroscope(data.cm730.gyroscope.y);
            sensors.gyroscope.z = Convert::gyroscope(data.cm730.gyroscope.z);

            /*
             Force Sensitive Resistor Data
             */

            // Right Sensor
            // Error
            sensors.fsr.right.errorFlags = data.fsrErrorCodes[0] == 0xFF
                                               ? DarwinSensors::Error::TIMEOUT
                                               : DarwinSensors::Error(data.fsrErrorCodes[0]).value;

            // Sensors
            sensors.fsr.right.fsr1 = Convert::fsrForce(data.fsr[0].fsr1);
            sensors.fsr.right.fsr2 = Convert::fsrForce(data.fsr[0].fsr2);
            sensors.fsr.right.fsr3 = Convert::fsrForce(data.fsr[0].fsr3);
            sensors.fsr.right.fsr4 = Convert::fsrForce(data.fsr[0].fsr4);

            // Centre, swaps X and Y coords to robot
            // see
            // http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
            sensors.fsr.right.centreX = Convert::fsrCentre(false, data.fsr[0].centreY);
            sensors.fsr.right.centreY = Convert::fsrCentre(false, data.fsr[0].centreX);

            // Left Sensor
            // Error
            sensors.fsr.left.errorFlags = data.fsrErrorCodes[1] == 0xFF
                                              ? DarwinSensors::Error::TIMEOUT
                                              : DarwinSensors::Error(data.fsrErrorCodes[1]).value;

            // Sensors
            sensors.fsr.left.fsr1 = Convert::fsrForce(data.fsr[1].fsr1);
            sensors.fsr.left.fsr2 = Convert::fsrForce(data.fsr[1].fsr2);
            sensors.fsr.left.fsr3 = Convert::fsrForce(data.fsr[1].fsr3);
            sensors.fsr.left.fsr4 = Convert::fsrForce(data.fsr[1].fsr4);

            // Centre, swaps X and Y coords to robot
            // see
            // http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/optional_components/fsr.htm
            sensors.fsr.left.centreX = Convert::fsrCentre(true, data.fsr[1].centreY);
            sensors.fsr.left.centreY = Convert::fsrCentre(true, data.fsr[1].centreX);

            /*
             Servos
             */

            for (int i = 0; i < 20; ++i) {
                // Get a reference to the servo we are populating
                DarwinSensors::Servo& servo = utility::platform::darwin::getDarwinServo(i, sensors);


                // Booleans
                servo.torqueEnabled = servoState[i].torqueEnabled;

                // Gain
                servo.pGain = servoState[i].pGain;
                servo.iGain = servoState[i].iGain;
                servo.dGain = servoState[i].dGain;

                // Torque
                servo.torque = servoState[i].torque;

                // Targets
                servo.goalPosition = servoState[i].goalPosition;
                servo.movingSpeed  = servoState[i].movingSpeed;

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
                    servo.presentPosition = servoState[i].presentPosition;
                    servo.presentSpeed    = servoState[i].goalPosition;
                    servo.load            = servoState[i].load;
                    servo.voltage         = servoState[i].voltage;
                    servo.temperature     = servoState[i].temperature;
                }

                // If we are using real data, get it from the packet
                else {
                    // Error code
                    servo.errorFlags = data.servoErrorCodes[i] == 0xFF
                                           ? DarwinSensors::Error::TIMEOUT
                                           : DarwinSensors::Error(data.servoErrorCodes[i]).value;

                    // Present Data
                    servo.presentPosition = Convert::servoPosition(i, data.servos[i].presentPosition);
                    servo.presentSpeed    = Convert::servoSpeed(i, data.servos[i].presentSpeed);
                    servo.load            = Convert::servoLoad(i, data.servos[i].load);

                    // Diagnostic Information
                    servo.voltage     = Convert::voltage(data.servos[i].voltage);
                    servo.temperature = Convert::temperature(data.servos[i].temperature);

                    // Clear Overvoltage flag if current voltage is greater than maximum expected voltage
                    if (servo.voltage <= maxVoltage) {
                        servo.errorFlags &= ~DarwinSensors::Error::INPUT_VOLTAGE;
                    }
                }
            }
            return sensors;
        }

        HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , darwin("/dev/CM730")
            , cm730State()
            , servoState()
            , maxVoltage(0.0f)
            , minVoltage(0.0f) {

            on<Startup>().then("HardwareIO Startup", [this] {
                uint16_t CM730Model  = darwin.cm730.read<uint16_t>(Darwin::CM730::Address::MODEL_NUMBER_L);
                uint8_t CM730Version = darwin.cm730.read<uint8_t>(Darwin::CM730::Address::VERSION);
                std::stringstream version, model;
                model << "0x" << std::setw(4) << std::setfill('0') << std::hex << int(CM730Model);
                version << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(CM730Version);
                log<NUClear::INFO>("CM730 Model:", model.str());
                log<NUClear::INFO>("CM730 Firmware Version:", version.str());
            });

            on<Configuration>("HardwareIO.yaml").then([this](const Configuration& config) {

                // Set config for the packet waiting
                darwin.setConfig(config);

                for (size_t i = 0; i < config["servos"].config.size(); ++i) {
                    Convert::SERVO_OFFSET[i]    = config["servos"][i]["offset"].as<Expression>();
                    Convert::SERVO_DIRECTION[i] = config["servos"][i]["direction"].as<Expression>();
                    servoState[i].simulated     = config["servos"][i]["simulated"].as<bool>();
                }

                maxVoltage = config["battery"]["max"].as<float>();
                minVoltage = config["battery"]["min"].as<float>();
            });

            // This trigger gets the sensor data from the CM730
            on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then(
                "Hardware Loop", [this] {

                    // Our final sensor output
                    auto sensors = std::make_unique<DarwinSensors>();

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
                                uint16_t goalPosition = Convert::servoPositionInverse(i, servoState[i].goalPosition);
                                uint16_t movingSpeed  = Convert::servoSpeedInverse(servoState[i].movingSpeed);
                                uint16_t torque       = Convert::torqueLimitInverse(servoState[i].torque);

                                // Add to our sync write command
                                command.insert(command.end(),
                                               {
                                                   uint8_t(i + 1),
                                                   Convert::gainInverse(servoState[i].dGain),  // D Gain
                                                   Convert::gainInverse(servoState[i].iGain),  // I Gain
                                                   Convert::gainInverse(servoState[i].pGain),  // P Gain
                                                   0,                                          // Reserved
                                                   uint8_t(0xFF & goalPosition),               // Goal Position L
                                                   uint8_t(0xFF & (goalPosition >> 8)),        // Goal Position H
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

                    // Send our nicely computed sensor data out to the world
                    emit(std::move(sensors));
                });

            // This trigger writes the servo positions to the hardware
            on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>().then(
                [this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {

                    // Loop through each of our commands
                    for (const auto& command : commands) {
                        float diff = utility::math::angle::difference(
                            command.position,
                            utility::platform::darwin::getDarwinServo(command.id, sensors).presentPosition);
                        NUClear::clock::duration duration = command.time - NUClear::clock::now();

                        float speed;
                        if (duration.count() > 0) {
                            speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                        }
                        else {
                            speed = 0;
                        }

                        // Update our internal state
                        if (servoState[command.id].pGain != command.gain
                            || servoState[command.id].iGain != command.gain * 0
                            || servoState[command.id].dGain != command.gain * 0
                            || servoState[command.id].movingSpeed != speed
                            || servoState[command.id].goalPosition != command.position
                            || servoState[command.id].torque != command.torque) {

                            servoState[command.id].dirty = true;

                            servoState[command.id].pGain = command.gain;
                            servoState[command.id].iGain = command.gain * 0;
                            servoState[command.id].dGain = command.gain * 0;

                            servoState[command.id].movingSpeed  = speed;
                            servoState[command.id].goalPosition = command.position;

                            servoState[command.id].torque       = command.torque;
                            servoState[uint(command.id)].torque = command.torque;
                        }
                    }
                });

            on<Trigger<ServoTarget>>().then([this](const ServoTarget command) {
                auto commandList = std::make_unique<std::vector<ServoTarget>>();
                commandList->push_back(command);

                // Emit it so it's captured by the reaction above
                emit<Scope::DIRECT>(std::move(commandList));
            });

            // If we get a HeadLED command then write it
            on<Trigger<DarwinSensors::HeadLED>>().then([this](const DarwinSensors::HeadLED& led) {
                // Update our internal state
                cm730State.headLED = led;

                darwin.cm730.write(Darwin::CM730::Address::LED_HEAD_L,
                                   Convert::colourLEDInverse(static_cast<uint8_t>((led.RGB & 0x00FF0000) >> 24),
                                                             static_cast<uint8_t>((led.RGB & 0x0000FF00) >> 8),
                                                             static_cast<uint8_t>(led.RGB & 0x000000FF)));
            });

            // If we get a EyeLED command then write it
            on<Trigger<DarwinSensors::EyeLED>>().then([this](const DarwinSensors::EyeLED& led) {
                // Update our internal state
                cm730State.eyeLED = led;

                darwin.cm730.write(Darwin::CM730::Address::LED_EYE_L,
                                   Convert::colourLEDInverse(static_cast<uint8_t>((led.RGB & 0x00FF0000) >> 24),
                                                             static_cast<uint8_t>((led.RGB & 0x0000FF00) >> 8),
                                                             static_cast<uint8_t>(led.RGB & 0x000000FF)));
            });
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
