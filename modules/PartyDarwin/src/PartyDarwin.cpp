/*
 * This file is part of PartyDarwin.
 *
 * PartyDarwin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PartyDarwin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PartyDarwin.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "PartyDarwin.h"
#include "messages/DarwinSensors.h"

namespace modules {

    PartyDarwin::PartyDarwin(NUClear::PowerPlant* plant) : Reactor(plant) {

        on<Trigger<Every<125, std::chrono::milliseconds>>>([this](const time_t&) {

            auto eyes = std::make_unique<messages::DarwinSensors::EyeLED>();
            eyes->r = rand();
            eyes->g = rand();
            eyes->b = rand();

            auto head = std::make_unique<messages::DarwinSensors::HeadLED>();
            head->r = rand();
            head->g = rand();
            head->b = rand();

            emit(std::move(eyes));
            emit(std::move(head));
        });
    }
}
