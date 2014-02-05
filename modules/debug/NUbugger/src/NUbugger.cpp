
/*
 * This file is part of NUbugger.
 *
 * NUbugger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NUbugger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NUbugger.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "messages/platform/darwin/DarwinSensors.h"
#include "utility/NUbugger/NUgraph.h"

namespace modules {
namespace debug {

    using NUClear::DEBUG;
    using utility::NUbugger::graph;
    using messages::platform::darwin::DarwinSensors;
    using std::chrono::milliseconds;

    NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<Every<50, milliseconds>>>([this](const time_t&) {

            double period = 10;
            double freq = 1 / period;
            double t = NUClear::clock::now().time_since_epoch().count() / double(NUClear::clock::period::den);
            float sine = sin(2 * M_PI * freq * t);
            float cosine = cos(2 * M_PI * freq * t);
            float dsine = 2 * sine;
            float dcosine = 4 * cosine;

            emit(graph("Debug Waves", sine, cosine, dsine, dcosine));

        });

        on<Trigger<DarwinSensors>>([this](const DarwinSensors& sensors) {

            emit(graph(
                "Accelerometer", 
                sensors.accelerometer.x,
                sensors.accelerometer.y,
                sensors.accelerometer.z
                
            ));

            emit(graph(
                "Gyro",
                sensors.gyroscope.x,
                sensors.gyroscope.y,
                sensors.gyroscope.z
            ));

        });

        NUClear::log<DEBUG>("NUbugger Debug Module Enabled");

    }

} // debug
} // modules
