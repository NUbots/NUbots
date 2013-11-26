/*
 * This file is part of Darwin Hardware IO.
 *
 * Darwin Hardware IO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Hardware IO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Hardware IO.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "HardwareIO.h"

#include "messages/DarwinServoCommand.h"
#include "messages/DarwinSensors.h"
#include "Convert.h"

namespace modules {
namespace Platform {
namespace Darwin {

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), darwin("/dev/ttyUSB0") {

        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<20, std::chrono::milliseconds>>>([this](const time_t& time) {

            // Read our data
            darwin::BulkReadResults data = darwin.bulkRead();

            auto sensors = std::make_unique<messages::DarwinSensors>();

            /*
             CM730 Data
             */

            // Read our Error code
            sensors->cm730ErrorFlags = data.cm730ErrorCode == -1 ? messages::DarwinSensors::Error::TIMEOUT : data.cm730ErrorCode;

            // LED Panel
            sensors->ledPanel.led2 = Convert::getBit<1>(data.cm730.ledPanel);
            sensors->ledPanel.led3 = Convert::getBit<2>(data.cm730.ledPanel);
            sensors->ledPanel.led4 = Convert::getBit<3>(data.cm730.ledPanel);

            // Head LED
            std::tie(sensors->headLED.r, sensors->headLED.g, sensors->headLED.b) = Convert::colourLED(data.cm730.headLED);

            // Head LED
            std::tie(sensors->eyeLED.r, sensors->eyeLED.g, sensors->eyeLED.b) = Convert::colourLED(data.cm730.eyeLED);

            // Buttons
            sensors->buttons.left = Convert::getBit<1>(data.cm730.buttons);
            sensors->buttons.middle = Convert::getBit<1>(data.cm730.buttons);

            // Voltage (in volts)
            sensors->voltage = Convert::voltage(data.cm730.voltage);

            // Acceleronometer (in m/s^2)
            sensors->acceleronometer.x = Convert::accelerometer(data.cm730.acceleronometer.x);
            sensors->acceleronometer.y = Convert::accelerometer(data.cm730.acceleronometer.y);
            sensors->acceleronometer.z = Convert::accelerometer(data.cm730.acceleronometer.z);

            // Gyroscope (in radians/second)
            sensors->gyroscope.x = Convert::gyroscope(data.cm730.gyroscope.x);
            sensors->gyroscope.y = Convert::gyroscope(data.cm730.gyroscope.y);
            sensors->gyroscope.z = Convert::gyroscope(data.cm730.gyroscope.z);

            /*
             Force Sensitive Resistor Data
             */

            // Right Sensor
            // Error
            sensors->fsr.right.errorFlags = data.fsrErrorCodes[0] == -1 ? messages::DarwinSensors::Error::TIMEOUT : data.fsrErrorCodes[0];

            // Sensors
            sensors->fsr.right.fsr1 = Convert::fsrForce(data.fsr[0].fsr1);
            sensors->fsr.right.fsr2 = Convert::fsrForce(data.fsr[0].fsr2);
            sensors->fsr.right.fsr3 = Convert::fsrForce(data.fsr[0].fsr3);
            sensors->fsr.right.fsr4 = Convert::fsrForce(data.fsr[0].fsr4);

            // Centre
            sensors->fsr.right.centreX = Convert::fsrCentre(false, true, data.fsr[0].centreX);
            sensors->fsr.right.centreY = Convert::fsrCentre(false, false, data.fsr[0].centreY);

            // Left Sensor
            // Error
            sensors->fsr.left.errorFlags = data.fsrErrorCodes[1] == -1 ? messages::DarwinSensors::Error::TIMEOUT : data.fsrErrorCodes[1];

            // Sensors
            sensors->fsr.left.fsr1 = Convert::fsrForce(data.fsr[1].fsr1);
            sensors->fsr.left.fsr2 = Convert::fsrForce(data.fsr[1].fsr2);
            sensors->fsr.left.fsr3 = Convert::fsrForce(data.fsr[1].fsr3);
            sensors->fsr.left.fsr4 = Convert::fsrForce(data.fsr[1].fsr4);

            // Centre
            sensors->fsr.left.centreX = Convert::fsrCentre(true, true, data.fsr[1].centreX);
            sensors->fsr.left.centreY = Convert::fsrCentre(true, false, data.fsr[1].centreY);

            /*
             Servos
             */

            for(int i = 0; i < 20; ++i) {
                // Get a reference to the servo we are populating
                messages::DarwinSensors::Servo& servo = sensors->servo[i];

                // Error code
                servo.errorFlags = data.servoErrorCodes[i] == -1 ? messages::DarwinSensors::Error::TIMEOUT : data.servoErrorCodes[i];

                // Booleans
                servo.torqueEnabled = data.servos[i].torqueEnabled;
                servo.led = data.servos[i].LED;

                // Gain
                servo.dGain = Convert::gain(data.servos[i].dGain);
                servo.iGain = Convert::gain(data.servos[i].iGain);
                servo.pGain = Convert::gain(data.servos[i].pGain);

                // Targets
                servo.goalPosition = Convert::servoPosition(i, data.servos[i].goalPosition);
                servo.movingSpeed = Convert::servoSpeed(i, data.servos[i].movingSpeed);
                servo.torqueLimit = Convert::torqueLimit(data.servos[i].torqueLimit);

                // Present Data
                servo.presentPosition = Convert::servoPosition(i, data.servos[i].presentPosition);
                servo.presentSpeed = Convert::servoSpeed(i, data.servos[i].presentSpeed);
                servo.load = Convert::servoLoad(i, data.servos[i].load);

                // Diagnostic Information
                servo.voltage = Convert::voltage(data.servos[i].voltage);
                servo.temperature = Convert::temperature(data.servos[i].temperature);
            }

            // Send our nicely computed sensor data out to the world
            emit(std::move(sensors));
        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<std::vector<messages::DarwinServoCommand>>>([this](const std::vector<messages::DarwinServoCommand>& commands) {

            std::vector<darwin::Types::ServoValues> values;

            // Loop through each of our commands
            for (const auto& command : commands) {
                // If all our gains are 0 then do a normal write to disable torque (syncwrite won't write to torqueEnable)
                if(command.pGain == 0 && command.iGain == 0 && command.dGain == 0) {
                    darwin[static_cast<int>(command.id) + 1].write(darwin::MX28::Address::TORQUE_ENABLE, false);
                }
                // Otherwise write the command using sync write
                else {
                    values.push_back({
                        static_cast<uint8_t>(static_cast<int>(command.id) + 1),  // The id's on the robot start with ID 1
                        Convert::gainInverse(command.dGain),
                        Convert::gainInverse(command.iGain),
                        Convert::gainInverse(command.pGain),
                        0,
                        Convert::servoPositionInverse(static_cast<int>(command.id), command.goalPosition),
                        Convert::servoSpeedInverse(static_cast<int>(command.id), command.movingSpeed)
                    });
                }
            }

            // Syncwrite our values
            darwin.writeServos(values);
        });

        on<Trigger<messages::DarwinServoCommand>>([this](const messages::DarwinServoCommand command) {
            auto commandList = std::make_unique<std::vector<messages::DarwinServoCommand>>();
            commandList->push_back(command);

            // Emit it so it's captured by the reaction above
            emit(std::move(commandList));
        });

        // If we get a HeadLED command then write it
        on<Trigger<messages::DarwinSensors::HeadLED>>([this](const messages::DarwinSensors::HeadLED& led) {
            darwin.cm730.write(darwin::CM730::Address::LED_HEAD_L, Convert::colourLEDInverse(led.r, led.g, led.b));
        });

        // If we get a HeadLED command then write it
        on<Trigger<messages::DarwinSensors::EyeLED>>([this](const messages::DarwinSensors::EyeLED& led) {
            darwin.cm730.write(darwin::CM730::Address::LED_EYE_L, Convert::colourLEDInverse(led.r, led.g, led.b));
        });

        on<Trigger<messages::LMissile>>([this](const messages::LMissile&) {
            darwin.lMissile.fire();
        });

        on<Trigger<messages::RMissile>>([this](const messages::RMissile&) {
            darwin.rMissile.fire();
        });
    }
}
}
}
