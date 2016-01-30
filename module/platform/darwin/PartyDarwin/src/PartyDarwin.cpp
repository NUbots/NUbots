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

#include "PartyDarwin.h"
#include "message/DarwinSensors.h"

namespace module {
    namespace platform {
        namespace darwin {

            PartyDarwin::PartyDarwin(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

                on<Every<125, std::chrono::milliseconds>>([this] {

                    auto eyes = std::make_unique<message::DarwinSensors::EyeLED>();
                    eyes->r = rand();
                    eyes->g = rand();
                    eyes->b = rand();

                    auto head = std::make_unique<message::DarwinSensors::HeadLED>();
                    head->r = rand();
                    head->g = rand();
                    head->b = rand();

                    emit(std::move(eyes));
                    emit(std::move(head));
                });
            }
        }
    }
}
