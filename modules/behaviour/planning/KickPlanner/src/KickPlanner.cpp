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

#include "KickPlanner.h"

#include "messages/motion/WalkCommand.h"
#include "messages/motion/KickCommand.h"


namespace modules {
namespace behaviour {
namespace planning {

    using messages::localisation::Ball;
    using messages::localisation::Goal;
    using messages::motion::KickCommand;
    using messages::motion::WalkStopCommand;

    KickPlanner::KickPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Ball>, With<Goal>>([this] (const Ball& ball, const Goal& goal) {

            // TODO check if the ball is within our kick box using some math or something

            // TODO check if we are alligned enough with the goal

            // TODO If we satisfy condtions then pick a kick to execute

            // Left side kick
            emit(std::make_unique<WalkStopCommand>()); // Stop the walk
            emit(std::make_unique<KickCommand>(KickCommand{{0, -1, 0}, LimbID::LEFT_LEG }));
            // TODO when the kick finishes, we need to start the walk
            // Probably need to add something to the KickScript.cpp

            // Left front kick
            emit(std::make_unique<WalkStopCommand>()); // Stop the walk
            emit(std::make_unique<KickCommand>(KickCommand{{1,  0, 0}, LimbID::LEFT_LEG }));
            // TODO when the kick finishes, we need to start the walk
            // Probably need to add something to the KickScript.cpp

            // Right side kick
            emit(std::make_unique<WalkStopCommand>()); // Stop the walk
            emit(std::make_unique<KickCommand>(KickCommand{{0,  1, 0}, LimbID::RIGHT_LEG }));
            // TODO when the kick finishes, we need to start the walk
            // Probably need to add something to the KickScript.cpp

            // Right front kick
            emit(std::make_unique<WalkStopCommand>()); // Stop the walk
            emit(std::make_unique<KickCommand>(KickCommand{{1,  0, 0}, LimbID::RIGHT_LEG }));
            // TODO when the kick finishes, we need to start the walk
            // Probably need to add something to the KickScript.cpp


            // Most of this code will be similar to that in PS3Walk.cpp

        });

    }

}
}
}

