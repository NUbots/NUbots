/*
 * This file is part of PS3Walk.
 *
 * PS3Walk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PS3Walk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PS3Walk.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "PS3Walk.h"
#include <nuclear>
#include "messages/motion/WalkCommand.h"

namespace modules {
namespace behaviour {
namespace planning {

    using messages::motion::WalkCommand;

    PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Every<1, std::chrono::milliseconds>>>([this](const time_t&) {
            JoystickEvent event;
            if (joystick.sample(&event)) {
                if (event.isAxis()) {
                    switch (event.number) {
                        case LEFT_JOYSTICK_HORIZONTAL:
                            strafe[0] = -event.value;
                            break;
                        case LEFT_JOYSTICK_VERTICAL:
                            strafe[1] = -event.value;
                            break;
                        case RIGHT_JOYSTICK_HORIZONTAL:
                            rotationalSpeed = -event.value;
                            break;
                    }
                }
            }
        });

        on<Trigger<Every<50, std::chrono::milliseconds>>>([this](const time_t&) {
            auto strafeNorm = strafe / std::numeric_limits<short>::max();
            auto rotationalSpeedNorm = rotationalSpeed / std::numeric_limits<short>::max();
            NUClear::log("Command\n", strafeNorm, rotationalSpeedNorm);
            emit(std::make_unique<WalkCommand>(WalkCommand{
                strafeNorm * 0.03,
                rotationalSpeedNorm
            }));
        });
    }

//    double normalize(short value) {
//
//    }

}
}
}

