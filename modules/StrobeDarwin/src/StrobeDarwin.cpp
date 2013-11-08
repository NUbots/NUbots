/*
 * This file is part of StrobeDarwin.
 *
 * StrobeDarwin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * StrobeDarwin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with StrobeDarwin.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "StrobeDarwin.h"
#include "messages/DarwinSensors.h"
#include "messages/Beat.h"

namespace modules {

    StrobeDarwin::StrobeDarwin(NUClear::PowerPlant* plant) : Reactor(plant) {
        // Strobe up
        on<Trigger<messages::Beat>>([this](const messages::Beat& beat) {
            auto eyes = std::make_unique<messages::DarwinSensors::EyeLED>();
            eyes->r = 0xff;
            eyes->g = 0xff;
            eyes->b = 0xff;

            auto head = std::make_unique<messages::DarwinSensors::HeadLED>();
            head->r = 0xff;
            head->g = 0xff;
            head->b = 0xff;

            emit(std::move(eyes));
            emit(std::move(head));
        });

        // Strobe down
        on<Trigger<Every<50, std::chrono::milliseconds>>,
           With<messages::DarwinSensors>>([this](
            const time_t&, const messages::DarwinSensors& data) {

            const int down = 25;
            
            auto eyes = std::make_unique<messages::DarwinSensors::EyeLED>();
            eyes->r = data.headLED.r > down ? data.headLED.r - down : 0;
            eyes->g = data.headLED.g > down ? data.headLED.g - down : 0;
            eyes->b = data.headLED.b > down ? data.headLED.b - down : 0;

            auto head = std::make_unique<messages::DarwinSensors::HeadLED>();
            head->r = data.headLED.r > down ? data.headLED.r - down : 0;
            head->g = data.headLED.g > down ? data.headLED.g - down : 0;
            head->b = data.headLED.b > down ? data.headLED.b - down : 0;

            emit(std::move(eyes));
            emit(std::move(head));
        });
    }
}
