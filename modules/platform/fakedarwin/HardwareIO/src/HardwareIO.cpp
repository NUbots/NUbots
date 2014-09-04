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

#include <armadillo>

#include "messages/motion/ServoTarget.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/ServoID.h"
#include "utility/math/angle.h"

using messages::platform::darwin::DarwinSensors;
using messages::motion::ServoTarget;
using messages::input::ServoID;

namespace modules {
namespace platform {
namespace fakedarwin {

    HardwareIO::HardwareIO(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        /*
         CM730 Data
         */

        // Read our Error code
        sensors.cm730ErrorFlags = 0;

        // LED Panel
        sensors.ledPanel.led2 = 0;
        sensors.ledPanel.led3 = 0;
        sensors.ledPanel.led4 = 0;

        // Head LED
        sensors.headLED.r = 0;
        sensors.headLED.g = 0;
        sensors.headLED.b = 0;

        // Head LED
        sensors.eyeLED.r = 0;
        sensors.eyeLED.g = 0;
        sensors.eyeLED.b = 0;

        // Buttons
        sensors.buttons.left = 0;
        sensors.buttons.middle = 0;

        // Voltage (in volts)
        sensors.voltage = 0;

        // Accelerometer (in m/s^2)
        sensors.accelerometer.x = 0;
        sensors.accelerometer.y = 0;
        sensors.accelerometer.z = 9.8;

        // Gyroscope (in radians/second)
        sensors.gyroscope.x = 0;
        sensors.gyroscope.y = 0;
        sensors.gyroscope.z = 0;

        /*
         Force Sensitive Resistor Data
         */

        // Right Sensor
        // Error
        sensors.fsr.right.errorFlags = 0;

        // Sensors
        sensors.fsr.right.fsr1 = 0;
        sensors.fsr.right.fsr2 = 0;
        sensors.fsr.right.fsr3 = 0;
        sensors.fsr.right.fsr4 = 0;

        // Centre
        sensors.fsr.right.centreX = 0;
        sensors.fsr.right.centreY = 0;

        // Left Sensor
        // Error
        sensors.fsr.left.errorFlags = 0;

        // Sensors
        sensors.fsr.left.fsr1 = 0;
        sensors.fsr.left.fsr2 = 0;
        sensors.fsr.left.fsr3 = 0;
        sensors.fsr.left.fsr4 = 0;

        // Centre
        sensors.fsr.left.centreX = 0;
        sensors.fsr.left.centreY = 0;

        /*
         Servos
         */

        for(int i = 0; i < 20; ++i) {
            // Get a reference to the servo we are populating
            DarwinSensors::Servo& servo = sensors.servo[i];

            // Error code
            servo.errorFlags = 0;

            // Booleans
            servo.torqueEnabled = true;

            // Gain
            servo.dGain = 0;
            servo.iGain = 0;
            servo.pGain = 0;

            // Targets
            servo.goalPosition = 0;
            servo.movingSpeed = M_PI_4;

            // Present Data
            servo.presentPosition = 0;
            servo.presentSpeed = 0;
            servo.load = 0;

            // Diagnostic Information
            servo.voltage = 0;
            servo.temperature = 0;
        }

        // This trigger gets the sensor data from the CM730
        on<Trigger<Every<60, Per<std::chrono::seconds> > >, Options<Single> >([this](const time_t&) {

            for (int i = 0; i < 20; ++i) {

                auto& servo = sensors.servo[i];
                float movingSpeed = servo.movingSpeed == 0 ? 0.1 : servo.movingSpeed / 60;
                movingSpeed = movingSpeed > 0.1 ? 0.1 : movingSpeed;


                if (std::abs(servo.presentPosition - servo.goalPosition) < movingSpeed) {
                    servo.presentPosition = servo.goalPosition;
                }
                else {
                    arma::vec3 present = { cos(servo.presentPosition), sin(servo.presentPosition), 0 };
                    arma::vec3 goal = { cos(servo.goalPosition), sin(servo.goalPosition), 0 };

                    arma::vec3 cross = arma::cross(present, goal);
                    if(cross[2] > 0) {
                        servo.presentPosition = utility::math::angle::normalizeAngle(servo.presentPosition + movingSpeed);
                    }
                    else {
                        servo.presentPosition = utility::math::angle::normalizeAngle(servo.presentPosition - movingSpeed);
                    }
                }
            }

            // Send our nicely computed sensor data out to the world
            emit(std::make_unique<DarwinSensors>(sensors));

        });

        // NOTE: dev test, makes random angles between -PI and PI
        /*on<Trigger<Every<2, std::chrono::seconds> >, Options<Single> >([this](const time_t& time) {

//          sensors.servo[ServoID::R_SHOULDER_PITCH].goalPosition = target;
//          sensors.servo[ServoID::L_SHOULDER_PITCH].goalPosition = target;

            for (int i = 0; i < 20; ++i) {
                float target = (float(rand()) / RAND_MAX) * 2 * M_PI - M_PI;
                sensors.servo[i].goalPosition = target;
            }

        });*/

        // This trigger writes the servo positions to the hardware
        on<Trigger<std::vector<ServoTarget> > >([this](const std::vector<ServoTarget>& commands) {
            for (auto& command : commands) {

                // Calculate our moving speed
                float diff = utility::math::angle::difference(command.position, sensors.servo[command.id].presentPosition);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();
                float speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));

                // Set our variables
                auto& servo = sensors.servo[command.id];
                servo.movingSpeed = speed;
                servo.goalPosition = command.position;
            }

            // Send our nicely computed sensor data out to the world
            emit(std::make_unique<DarwinSensors>(sensors));
        });

        on<Trigger<ServoTarget> >([this](const ServoTarget command) {
            auto commandList = std::make_unique<std::vector<ServoTarget>>();
            commandList->push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });
    }
}
}
}
