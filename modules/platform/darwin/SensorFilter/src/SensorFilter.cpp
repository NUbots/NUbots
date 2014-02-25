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
#include "messages/support/Configuration.h"
#include "utility/NUbugger/NUgraph.h"
#include "utility/math/matrix.h"

namespace modules {
    namespace platform {
        namespace darwin {
            

            using messages::support::Configuration;
            using messages::platform::darwin::DarwinSensors;
            using messages::input::Sensors;
            using utility::NUbugger::graph;
            using messages::input::ServoID;



            SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), orientationFilter(arma::vec("0,0,-1,1,0,0")) , frameLimiter(0), lastOrientationMatrix(arma::eye(3,3)) {
                
                on<Trigger<Configuration<SensorFilter>>>([this](const Configuration<SensorFilter>& file){
                    DEFAULT_NOISE_GAIN = file.config["DEFAULT_NOISE_GAIN"];
                    HIGH_NOISE_THRESHOLD = file.config["HIGH_NOISE_THRESHOLD"];
                    HIGH_NOISE_GAIN = file.config["HIGH_NOISE_GAIN"];
                    LOW_NOISE_THRESHOLD = file.config["LOW_NOISE_THRESHOLD"];
                });

                on<Trigger<DarwinSensors>, Options<Single>>([this](const DarwinSensors& input) {
                    
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
                            float(original.temperature)
                        });
                    }

                    sensors->accelerometer = {-input.accelerometer.y, input.accelerometer.x, -input.accelerometer.z};
                    sensors->gyroscope = {-input.gyroscope.x, -input.gyroscope.y, input.gyroscope.z};
                     
                    // Kalman filter for orientation
                    double deltaT = (lastUpdate - input.timestamp).count() / double(NUClear::clock::period::den);
                    lastUpdate = input.timestamp;                     
                    orientationFilter.timeUpdate(deltaT, sensors->gyroscope);                     

                    arma::mat observationNoise = arma::eye(3,3) * DEFAULT_NOISE_GAIN;                     
                    double normAcc = std::abs(arma::norm(sensors->accelerometer,2) - 9.807);

                    if(normAcc > HIGH_NOISE_THRESHOLD){
                        observationNoise *= HIGH_NOISE_GAIN;
                    } else if(normAcc > LOW_NOISE_THRESHOLD){
                        observationNoise = arma::eye(3,3) * (HIGH_NOISE_GAIN - DEFAULT_NOISE_GAIN) * (normAcc - LOW_NOISE_THRESHOLD) / (HIGH_NOISE_THRESHOLD - LOW_NOISE_THRESHOLD);
                    }
                     
                    float quality = orientationFilter.measurementUpdate(sensors->accelerometer, observationNoise);                     
                    arma::vec orientation = orientationFilter.get();
                    sensors->orientation.col(2) = -orientation.rows(0,2);
                    sensors->orientation.col(0) = orientation.rows(3,5);
                    sensors->orientation.col(1) = arma::cross(sensors->orientation.col(2), sensors->orientation.col(0));

                    // Kalman filter for orientation END



                    //BEGIN CALCULATE FILTERED GYRO
                    arma::mat33 SORAMatrix = sensors->orientation * lastOrientationMatrix.t();            
                    std::pair<arma::vec3, double> axisAngle = utility::math::matrix::axisAngleFromRotationMatrix(SORAMatrix);
                    sensors->gyroscope = axisAngle.first * (axisAngle.second / deltaT);
                    //END CALCULATE FILTERED GYRO


                    if(++frameLimiter % 3 == 0){
                        emit(graph("Filtered Gravity Vector",
                                float(orientation[0]*9.807),
                                float(orientation[1]*9.807),
                                float(orientation[2]*9.807)
                            ));
                         emit(graph("Filtered Forward Vector",
                                float(orientation[3]),
                                float(orientation[4]),
                                float(orientation[5])
                            ));
                        emit(graph("Orientation Quality", quality
                            ));
                        emit(graph("Difference from gravity", normAcc
                            ));
                        emit(graph("Gyro Filtered", sensors->gyroscope[0],sensors->gyroscope[1], sensors->gyroscope[2]
                            ));
                        frameLimiter = 1;
                    }   

                    lastOrientationMatrix = sensors->orientation;

                    emit(std::move(sensors));
                });
            }
            
        }  // darwin
    }  // platform
}  // modules
