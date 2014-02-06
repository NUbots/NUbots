/*
 * This file is part of Darwin Sensor Filter.
 *
 * Darwin Sensor Filter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Sensor Filter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Sensor Filter.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SensorFilter.h"

#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Sensors.h"
#include "utility/NUbugger/NUgraph.h"

namespace modules {
    namespace platform {
        namespace darwin {
            
            using messages::platform::darwin::DarwinSensors;
            using messages::input::Sensors;
            using utility::NUbugger::graph;
            using messages::input::ServoID;



            SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), orientationFilter(arma::vec3("1,0,0")) {
                on<Trigger<DarwinSensors>>([this](const DarwinSensors& input) {
                    
                    auto sensors = std::make_unique<Sensors>();

                    sensors->timestamp = input.timestamp;
                    
                    for(uint i = 0; i < 20; ++i) {
                        auto& original = input.servo[i];

                        sensors->servos.push_back({
                            original.errorFlags,
                            static_cast<ServoID>(i),
                            original.torqueEnabled,
                            original.pGain,
                            original.iGain,
                            original.dGain,
                            original.goalPosition,
                            original.movingSpeed,
                            original.torqueLimit,
                            original.presentPosition,
                            original.presentSpeed,
                            original.load,
                            original.voltage,
                            original.temperature
                        });
                    }

                    sensors->accelerometer = {-input.accelerometer.y, input.accelerometer.y, -input.accelerometer.z};
                    sensors->gyroscope = {-input.gyroscope.x, -input.gyroscope.y, input.gyroscope.z};

                    // Kalman filter for orientation
                    double deltaT = (lastUpdate - input.timestamp).count() / double(NUClear::clock::period::den);
                    lastUpdate = input.timestamp;

                    orientationFilter.timeUpdate(deltaT, sensors->gyroscope);
                    float quality = orientationFilter.measurementUpdate(sensors->accelerometer, 0.001 * arma::eye(3, 3));
                    arma::vec3 orientation = orientationFilter.get();

                    sensors->orientation = orientationFilter.get();

                    emit(graph("Filtered Orientation",
                            float(orientation[0]),
                            float(orientation[1]),
                            float(orientation[2])
                        ));

                    emit(graph("Orientation Quality", quality
                        ));
                    // Kalman filter all of the incoming data!

                    
                    // Output the filtered data
                    
                    emit(std::move(sensors));
                });
            }
            
        }  // darwin
    }  // platform
}  // modules
