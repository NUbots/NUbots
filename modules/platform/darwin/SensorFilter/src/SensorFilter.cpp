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

#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Sensors.h"

#include "MotionManager.h"
#include "utility/math/angle.h"

namespace modules {
    namespace platform {
        namespace darwin {
            
            using messages::platform::darwin::DarwinSensorData;
            using messages::input::Sensors;

            SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                on<Trigger<DarwinSensorData>>([this](const DarwinSensorData& input) {
                    
                    auto sensors = std::make_unique<Sensors>();
                    
                    // Kalman filter all of the incoming data!
                    
                    // Output the filtered data
                    
                    emit(std::move(sensors));
                });
            }
            
        }  // darwin
    }  // platform
}  // modules
