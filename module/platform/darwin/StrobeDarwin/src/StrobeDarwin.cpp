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

#include "StrobeDarwin.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "message/audio/Beat.h"

namespace module {
    namespace platform {
        namespace darwin {

            using message::audio::Beat;
            using message::platform::darwin::DarwinSensors;

            StrobeDarwin::StrobeDarwin(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
                // Strobe up
                on<Trigger<Beat>>().then([this] {
                    auto eyes = std::make_unique<DarwinSensors::EyeLED>();
                    eyes->r = 0xff;
                    eyes->g = 0xff;
                    eyes->b = 0xff;

                    auto head = std::make_unique<DarwinSensors::HeadLED>();
                    head->r = 0xff;
                    head->g = 0xff;
                    head->b = 0xff;

                    emit(std::move(eyes));
                    emit(std::move(head));
                });

                // Strobe down
                on<Every<50, std::chrono::milliseconds>, With<DarwinSensors>>()
                .then([this](const DarwinSensors& data) {

                    const int down = 25;

                    auto eyes = std::make_unique<DarwinSensors::EyeLED>();
                    eyes->r = data.headLED.r > down ? data.headLED.r - down : 0;
                    eyes->g = data.headLED.g > down ? data.headLED.g - down : 0;
                    eyes->b = data.headLED.b > down ? data.headLED.b - down : 0;

                    auto head = std::make_unique<DarwinSensors::HeadLED>();
                    head->r = data.headLED.r > down ? data.headLED.r - down : 0;
                    head->g = data.headLED.g > down ? data.headLED.g - down : 0;
                    head->b = data.headLED.b > down ? data.headLED.b - down : 0;

                    emit(std::move(eyes));
                    emit(std::move(head));
                });
            }

        }  // darwin
    }  // platform
}  // modules