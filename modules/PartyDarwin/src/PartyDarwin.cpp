/*
 * This file is part of PartyDarwin.
 *
 * PartyDarwin is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * PartyDarwin is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with PartyDarwin.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "PartyDarwin.h"
#include "messages/DarwinSensors.h"

namespace modules {

    PartyDarwin::PartyDarwin(NUClear::PowerPlant* plant) : Reactor(plant) {
        
        on<Trigger<Every<125, std::chrono::milliseconds>>>([this](const time_t&) {
            
            auto* eyes = new messages::DarwinSensors::EyeLED();
            eyes->r = rand();
            eyes->g = rand();
            eyes->b = rand();
            
            auto* head = new messages::DarwinSensors::HeadLED();
            head->r = rand();
            head->g = rand();
            head->b = rand();
            
            emit(eyes);
            emit(head);
        });
    }
}
