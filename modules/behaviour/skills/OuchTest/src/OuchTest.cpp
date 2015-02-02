/*
 * This file is part of NUbots Codebase.
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

#include "OuchTest.h"
#include "messages/input/Sensors.h"
#include "messages/output/Say.h"


namespace modules {
    namespace behaviour {
        namespace skills {

            using messages::input::Sensors;
            using messages::output::Say;

            OuchTest::OuchTest(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

                on<Trigger<Sensors>, Options<Single>>([this] (const Sensors& sensors) {

                    //Check if the orientation angle is smaller than the cosine of our fallen angle then say ouch! if fallen
                    if(fabs(sensors.orientation(2,2)) < 0.6 && not fallen) {
                        // Say ouch!
                        emit(std::make_unique<Say>("Ouch!"));
                        fallen = true;
                    }
                    else if(fabs(sensors.orientation(2,2)) >= 0.7) {
                        fallen = false;
                    }
                });
            }
        }
    }
}

