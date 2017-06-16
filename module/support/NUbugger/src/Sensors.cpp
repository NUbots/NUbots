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

#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {

    using message::input::Sensors;

    void NUbugger::provideSensors() {

        // This trigger gets the output from the sensors (unfiltered)
        handles["sensor_data"].push_back(on<Trigger<Sensors>, Single, Priority::LOW>().then([this](const Sensors& sensors) {

            send(sensors, 1, false, sensors.timestamp);

        }));
    }
}
}
