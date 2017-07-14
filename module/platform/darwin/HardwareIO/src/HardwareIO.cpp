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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "HardwareIO.h"
#include "Convert.h"

#include "extension/Configuration.h"

#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/platform/darwin/Firmware.h"

#include "utility/math/angle.h"
#include "utility/platform/darwin/DarwinSensors.h"
#include "utility/support/yaml_expression.h"


namespace module {
namespace platform {
    namespace darwin {

        using message::platform::darwin::DarwinSensors;
        using message::platform::darwin::FlashCM730Firmware;
        using message::motion::ServoTarget;
        using extension::Configuration;
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

                // Error code
                servo.errorFlags = data.servoErrorCodes[i] == 0xFF
                                       ? DarwinSensors::Error::TIMEOUT
                                       : DarwinSensors::Error(data.servoErrorCodes[i]).value;

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

                // Present Data
                servo.presentPosition = Convert::servoPosition(i, data.servos[i].presentPosition);
                servo.presentSpeed    = Convert::servoSpeed(i, data.servos[i].presentSpeed);
                servo.load            = Convert::servoLoad(i, data.servos[i].load);

                // Diagnostic Information
                servo.voltage     = Convert::voltage(data.servos[i].voltage);
                servo.temperature = Convert::temperature(data.servos[i].temperature);
            }

            return sensors;
        }

        HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), darwin("/dev/CM730"), cm730State(), servoState(), hardwareLoop() {

            on<Configuration>("DarwinPlatform.yaml").then([this](const Configuration& config) {
                darwin.setConfig(config);

                for (size_t i = 0; i < config["servo_offset"].config.size(); i++) {
                    Convert::SERVO_OFFSET[i] = config["servo_offset"][i].as<Expression>();
                }

                for (size_t i = 0; i < config["servo_direction"].config.size(); i++) {
                    Convert::SERVO_DIRECTION[i] = config["servo_direction"][i].as<int>();
                }
            });

            // This trigger gets the sensor data from the CM730
            hardwareLoop =
                on<Every<90, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then("Hardware Loop", [this] {

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
            on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>().then([this](
                const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {

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
                    if (servoState[command.id].pGain != command.gain || servoState[command.id].iGain != command.gain * 0
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

            on<Trigger<FlashCM730Firmware>>().then([this](const FlashCM730Firmware& fw) {
                // Disable the hardware loop reaction. We don't want to be interrupted.
                hardwareLoop.disable();

                log<NUClear::INFO>("Press DARwIn-OP's Reset button to start...");

                std::vector<uint8_t> buf;

                for (size_t retries = 0; retries < 100; retries++) {
                    darwin.cm730.writeBytes(std::vector<uint8_t>{'#'});

                    std::this_thread::sleep_for(std::chrono::milliseconds(20));

                    if (darwin.cm730.readBytes(buf, 1) > 0) {
                        if (buf.front() == '#') {
                            darwin.cm730.writeBytes(std::vector<uint8_t>{'\r'});
                            break;
                        }
                    }
                }

                /*+++ start download +++*/
                darwin.cm730.writeBytes(std::vector<uint8_t>{'l', '\r'});

                for (size_t retries = 0; retries < 100; retries++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(135));

                    size_t count = 0;

                    if ((count = darwin.cm730.readBytes(buf, 256)) > 0) {
                        if (count == 256) {
                            buf.push_back(0);
                        }

                        else {
                            buf[count] = 0;
                        }

                        std::stringstream data;
                        std::copy(buf.begin(), buf.end(), std::ostream_iterator<uint8_t>(data, ""));
                        log<NUClear::INFO>(data.str());
                    }

                    else {
                        log<NUClear::INFO>("Erase block complete...");
                        break;
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                // Calculate checksum.
                uint8_t byteSum = 0x00;

                std::for_each(fw.firmware.begin() + fw.startAddress,
                              fw.firmware.begin() + (fw.startAddress + (128 * 1024)),
                              [&](const uint8_t& byte) { byteSum += byte; });

                const size_t MAX_UNIT = 64;
                size_t size           = 0;

                while (size < fw.binSize) {
                    size_t unit = fw.binSize - size;

                    if (unit > MAX_UNIT) {
                        unit = MAX_UNIT;
                    }

                    size_t offset = fw.startAddress + size;

                    size_t count = darwin.cm730.writeBytes(
                        std::vector<uint8_t>{fw.firmware.begin() + offset, fw.firmware.begin() + offset + unit});

                    if (count > 0) {
                        size += count;
                        log<NUClear::INFO>("Downloading Firmware:", size, "bytes out of", fw.binSize, "bytes written.");
                    }
                }

                darwin.cm730.writeBytes(std::vector<uint8_t>{byteSum});
                log<NUClear::INFO>("Downloading Bytesum:", byteSum);

                for (int x = 0; x < 100; x++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));

                    size_t count = 0;

                    if ((count = darwin.cm730.readBytes(buf, 256)) > 0) {
                        if (count == 256) {
                            buf.push_back(0);
                        }

                        else {
                            buf[count] = 0;
                        }

                        std::stringstream data;
                        std::copy(buf.begin(), buf.end(), std::ostream_iterator<uint8_t>(data, ""));
                        log<NUClear::INFO>(data.str());
                    }
                }
                /*--- end download ---*/

                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                // Exit bootloader
                darwin.cm730.writeBytes(std::vector<uint8_t>{'\r', 'g', 'o', '\r'});

                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                size_t count = 0;

                if ((count = darwin.cm730.readBytes(buf, 256)) > 0) {
                    if (count == 256) {
                        buf.push_back(0);
                    }

                    else {
                        buf[count] = 0;
                    }

                    std::stringstream data;
                    std::copy(buf.begin(), buf.end(), std::ostream_iterator<uint8_t>(data, ""));
                    log<NUClear::INFO>(data.str());
                }

                // Reenable hardware loop.
                hardwareLoop.enable();
            });
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace module
