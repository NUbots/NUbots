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

#include "NUbugger.h"

#include "message/input/Sensors.h"
#include "message/input/proto/Sensors.h"

#include "utility/support/eigen_armadillo.h"
#include "utility/time/time.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::input::Sensors;
    using ProtoSensors = message::input::proto::Sensors;

    void NUbugger::provideSensors() {

        // This trigger gets the output from the sensors (unfiltered)
        handles["sensor_data"].push_back(on<Trigger<Sensors>, Single, Priority::LOW>().then([this](const Sensors& sensors) {

            ProtoSensors sensorData;

            sensorData.timestamp = sensors.timestamp.time_since_epoch().count();
            sensorData.voltage = sensors.voltage;
            sensorData.battery = sensors.battery;

            // Add each of the servos into the protocol buffer
            for(const auto& servo : sensors.servos) {

                ProtoSensors::Servo protoServo;

                protoServo.error_flags = servo.errorFlags;

                protoServo.id = static_cast<ProtoSensors::ServoID::Value>(servo.id);

                protoServo.enabled = servo.enabled;

                protoServo.p_gain = servo.pGain;
                protoServo.i_gain = servo.iGain;
                protoServo.d_gain = servo.dGain;

                protoServo.goal_position = servo.goalPosition;
                protoServo.goal_velocity = servo.goalVelocity;

                protoServo.present_position = servo.presentPosition;
                protoServo.present_velocity = servo.presentVelocity;

                protoServo.load = servo.load;
                protoServo.voltage = servo.voltage;
                protoServo.temperature = servo.temperature;

                sensorData.servo.push_back(protoServo);
            }

            // The gyroscope values (x,y,z)
            sensorData.gyroscope = convert<double, 3>(sensors.gyroscope);

            // The accelerometer values (x,y,z)
            sensorData.accelerometer = convert<double, 3>(sensors.accelerometer);

            // The orientation matrix
            sensorData.world = convert<double, 4, 4>(sensors.world);

            // The FSR values
            for (auto& fsr : sensors.fsrs) {
                ProtoSensors::FSR protoFSR;
                protoFSR.centre = convert<double, 2>(fsr.centre);

                for (auto& v : fsr.values) {
                    protoFSR.value.push_back(v);
                }

                sensorData.fsr.push_back(protoFSR);
            }

            // The LEDs
            for(auto& led : sensors.leds) {
                ProtoSensors::LED protoLED;
                protoLED.id     = led.id;
                protoLED.colour = led.colour;
                sensorData.led.push_back(protoLED);
            }

            // The Buttons
            for(auto& button : sensors.buttons) {
                ProtoSensors::Button protoButton;
                protoButton.id    = button.id;
                protoButton.value = button.value;
                sensorData.button.push_back(protoButton);
            }

            send(sensorData, 1, false, sensors.timestamp);

        }));
    }
}
}
