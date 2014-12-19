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
#include "messages/motion/KickCommand.h"
#include "messages/behaviour/Action.h"

namespace modules {
namespace behaviour {
namespace strategy {

    using messages::motion::KickCommand;
    using messages::motion::WalkCommand;
    using messages::motion::WalkStartCommand;
    using messages::motion::WalkStopCommand;
    using messages::behaviour::LimbID;

    PS3Walk::PS3Walk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Every<1, std::chrono::milliseconds>>, Options<Single>>([this](const time_t&) {

            JoystickEvent event;
            // read from joystick
            if (joystick.sample(&event)) {

                if (event.isAxis()) {
                    // event was an axis event
                    switch (event.number) {
                        case AXIS_LEFT_JOYSTICK_HORIZONTAL:
                            // y is left relative to robot
                            strafe[1] = -event.value;
                            break;
                        case AXIS_LEFT_JOYSTICK_VERTICAL:
                            // x is forward relative to robot
                            strafe[0] = -event.value;
                            break;
                        case AXIS_RIGHT_JOYSTICK_HORIZONTAL:
                            rotationalSpeed = -event.value;
                            break;
                    }
                } else if (event.isButton()) {
                    // event was a button event
                    switch (event.number) {
                        case BUTTON_TRIANGLE:
                            if (event.value > 0) { // button down
                                if (moving) {
                                    NUClear::log("Stop walking");
                                    emit(std::make_unique<WalkStopCommand>());
                                } else {
                                    NUClear::log("Start walking");
                                    emit(std::make_unique<WalkStartCommand>());
                                }
                                moving = !moving;
                            }
                            break;
                        case BUTTON_L1:
                            if (event.value > 0) { // button down
                                NUClear::log("Requesting Left Side Kick");
                                emit(std::make_unique<KickCommand>(KickCommand{
                                    {0, -1, 0}, // vector pointing right relative to robot
                                    LimbID::LEFT_LEG
                                }));
                            }
                            break;
                        case BUTTON_L2:
                            if (event.value > 0) { // button down
                                NUClear::log("Requesting Left Front Kick");
                                emit(std::make_unique<KickCommand>(KickCommand{
                                    {1, 0, 0}, // vector pointing forward relative to robot
                                    LimbID::LEFT_LEG
                                }));
                            }
                            break;
                        case BUTTON_R1:
                            if (event.value > 0) { // button down
                                NUClear::log("Requesting Right Side Kick");
                                emit(std::make_unique<KickCommand>(KickCommand{
                                    {0, 1, 0}, // vector pointing left relative to robot
                                    LimbID::RIGHT_LEG
                                }));
                            }
                            break;
                        case BUTTON_R2:
                            if (event.value > 0) { // button down
                                NUClear::log("Requesting Right Front Kick");
                                emit(std::make_unique<KickCommand>(KickCommand{
                                    {1, 0, 0}, // vector pointing forward relative to robot
                                    LimbID::RIGHT_LEG
                                }));
                            }
                            break;
                    }
                }
            }
            // If it's closed then we should try to reconnect
            else if (!joystick.valid()) {

                joystick.reconnect();
            }
        });

        // output walk command based on updated strafe and rotation speed from joystick
        // TODO: potential performance gain: ignore if value hasn't changed since last emit?
        on<Trigger<Every<50, std::chrono::milliseconds>>>([this](const time_t&) {

            // Why armadillo why!!!
            arma::vec s = { strafe[0], strafe[1] };
            arma::vec strafeNorm = s / std::numeric_limits<short>::max();

            auto rotationalSpeedNorm = rotationalSpeed / std::numeric_limits<short>::max();

            auto walkCommand = std::make_unique<WalkCommand>();
            walkCommand->velocity = strafeNorm;
            walkCommand->rotationalSpeed = rotationalSpeedNorm;
            emit(std::move(walkCommand));
        });
    }

}
}
}

